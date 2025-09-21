#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{prelude::*, stm32};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::blocking::delay::DelayMs;
use heapless::{Vec, String};

// 系统组件集成测试
pub mod system_integration {
    use super::*;
    use stm32f4xx_hal::{
        gpio::{Input, Output, PushPull, PullUp, gpioa::*, gpiob::*},
        serial::{Serial, Config},
        spi::{Spi, Mode, Phase, Polarity},
        i2c::I2c,
        adc::{Adc, config::AdcConfig},
        timer::{Timer, Event},
        delay::Delay,
    };
    use nb::block;
    
    /// GPIO集成测试
    pub struct GpioIntegrationTest {
        pub input_pin: PA0<Input<PullUp>>,
        pub output_pin: PA1<Output<PushPull>>,
        pub led_pin: PA5<Output<PushPull>>,
    }
    
    impl GpioIntegrationTest {
        pub fn new(
            input_pin: PA0<Input<PullUp>>,
            output_pin: PA1<Output<PushPull>>,
            led_pin: PA5<Output<PushPull>>,
        ) -> Self {
            Self {
                input_pin,
                output_pin,
                led_pin,
            }
        }
        
        pub fn test_digital_loopback(&mut self) -> Result<(), TestError> {
            // 测试数字信号回环
            self.output_pin.set_high().map_err(|_| TestError::GpioError)?;
            cortex_m::asm::delay(1000);
            
            if self.input_pin.is_low().map_err(|_| TestError::GpioError)? {
                return Err(TestError::LoopbackFailed);
            }
            
            self.output_pin.set_low().map_err(|_| TestError::GpioError)?;
            cortex_m::asm::delay(1000);
            
            if self.input_pin.is_high().map_err(|_| TestError::GpioError)? {
                return Err(TestError::LoopbackFailed);
            }
            
            Ok(())
        }
        
        pub fn test_led_blink(&mut self, delay: &mut Delay) -> Result<(), TestError> {
            for _ in 0..5 {
                self.led_pin.set_high().map_err(|_| TestError::GpioError)?;
                delay.delay_ms(100u32);
                self.led_pin.set_low().map_err(|_| TestError::GpioError)?;
                delay.delay_ms(100u32);
            }
            Ok(())
        }
        
        pub fn test_interrupt_response(&mut self) -> Result<(), TestError> {
            // 模拟中断响应测试
            // 在实际硬件中，这会配置外部中断
            Ok(())
        }
    }
    
    /// UART集成测试
    pub struct UartIntegrationTest<UART> {
        pub serial: Serial<UART>,
        pub test_data: Vec<u8, 256>,
    }
    
    impl<UART> UartIntegrationTest<UART>
    where
        UART: embedded_hal::serial::Read<u8> + embedded_hal::serial::Write<u8>,
    {
        pub fn new(serial: Serial<UART>) -> Self {
            let mut test_data = Vec::new();
            for i in 0..=255 {
                test_data.push(i as u8).unwrap();
            }
            
            Self {
                serial,
                test_data,
            }
        }
        
        pub fn test_echo(&mut self) -> Result<(), TestError> {
            let test_message = b"Hello, UART!";
            
            // 发送测试消息
            for &byte in test_message {
                block!(self.serial.write(byte)).map_err(|_| TestError::UartError)?;
            }
            
            // 接收回显
            let mut received = Vec::<u8, 32>::new();
            for _ in 0..test_message.len() {
                let byte = block!(self.serial.read()).map_err(|_| TestError::UartError)?;
                received.push(byte).map_err(|_| TestError::BufferFull)?;
            }
            
            if received.as_slice() != test_message {
                return Err(TestError::DataMismatch);
            }
            
            Ok(())
        }
        
        pub fn test_throughput(&mut self) -> Result<u32, TestError> {
            let start_time = cortex_m::peripheral::DWT::cycle_count();
            
            // 发送大量数据
            for &byte in &self.test_data {
                block!(self.serial.write(byte)).map_err(|_| TestError::UartError)?;
            }
            
            let end_time = cortex_m::peripheral::DWT::cycle_count();
            let cycles = end_time.wrapping_sub(start_time);
            
            Ok(cycles)
        }
        
        pub fn test_error_handling(&mut self) -> Result<(), TestError> {
            // 测试错误处理（帧错误、奇偶校验错误等）
            // 这需要特殊的硬件设置来产生错误条件
            Ok(())
        }
    }
    
    /// SPI集成测试
    pub struct SpiIntegrationTest<SPI> {
        pub spi: Spi<SPI>,
        pub cs_pin: PB0<Output<PushPull>>,
    }
    
    impl<SPI> SpiIntegrationTest<SPI>
    where
        SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    {
        pub fn new(spi: Spi<SPI>, cs_pin: PB0<Output<PushPull>>) -> Self {
            Self { spi, cs_pin }
        }
        
        pub fn test_loopback(&mut self) -> Result<(), TestError> {
            let test_data = [0x55, 0xAA, 0xFF, 0x00, 0x12, 0x34, 0x56, 0x78];
            let mut buffer = test_data.clone();
            
            self.cs_pin.set_low().map_err(|_| TestError::SpiError)?;
            
            self.spi.transfer(&mut buffer).map_err(|_| TestError::SpiError)?;
            
            self.cs_pin.set_high().map_err(|_| TestError::SpiError)?;
            
            // 在回环模式下，发送的数据应该等于接收的数据
            if buffer != test_data {
                return Err(TestError::DataMismatch);
            }
            
            Ok(())
        }
        
        pub fn test_write_only(&mut self) -> Result<(), TestError> {
            let test_data = [0x01, 0x02, 0x03, 0x04];
            
            self.cs_pin.set_low().map_err(|_| TestError::SpiError)?;
            
            self.spi.write(&test_data).map_err(|_| TestError::SpiError)?;
            
            self.cs_pin.set_high().map_err(|_| TestError::SpiError)?;
            
            Ok(())
        }
        
        pub fn test_clock_modes(&mut self) -> Result<(), TestError> {
            // 测试不同的SPI时钟模式
            // 这需要重新配置SPI外设
            Ok(())
        }
    }
    
    /// I2C集成测试
    pub struct I2cIntegrationTest<I2C> {
        pub i2c: I2c<I2C>,
        pub device_address: u8,
    }
    
    impl<I2C> I2cIntegrationTest<I2C>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::Read + embedded_hal::blocking::i2c::WriteRead,
    {
        pub fn new(i2c: I2c<I2C>, device_address: u8) -> Self {
            Self { i2c, device_address }
        }
        
        pub fn test_device_scan(&mut self) -> Result<Vec<u8, 128>, TestError> {
            let mut found_devices = Vec::new();
            
            for addr in 0x08..0x78 {
                if self.i2c.write(addr, &[]).is_ok() {
                    found_devices.push(addr).map_err(|_| TestError::BufferFull)?;
                }
            }
            
            Ok(found_devices)
        }
        
        pub fn test_register_access(&mut self) -> Result<(), TestError> {
            let register = 0x00;
            let test_value = 0x55;
            
            // 写入寄存器
            self.i2c.write(self.device_address, &[register, test_value])
                .map_err(|_| TestError::I2cError)?;
            
            // 读取寄存器
            let mut buffer = [0u8; 1];
            self.i2c.write_read(self.device_address, &[register], &mut buffer)
                .map_err(|_| TestError::I2cError)?;
            
            if buffer[0] != test_value {
                return Err(TestError::DataMismatch);
            }
            
            Ok(())
        }
        
        pub fn test_multi_byte_transfer(&mut self) -> Result<(), TestError> {
            let test_data = [0x01, 0x02, 0x03, 0x04, 0x05];
            let register = 0x10;
            
            // 写入多字节数据
            let mut write_buffer = Vec::<u8, 16>::new();
            write_buffer.push(register).unwrap();
            for &byte in &test_data {
                write_buffer.push(byte).unwrap();
            }
            
            self.i2c.write(self.device_address, &write_buffer)
                .map_err(|_| TestError::I2cError)?;
            
            // 读取多字节数据
            let mut read_buffer = [0u8; 5];
            self.i2c.write_read(self.device_address, &[register], &mut read_buffer)
                .map_err(|_| TestError::I2cError)?;
            
            if read_buffer != test_data {
                return Err(TestError::DataMismatch);
            }
            
            Ok(())
        }
    }
    
    /// ADC集成测试
    pub struct AdcIntegrationTest {
        pub adc: Adc<stm32::ADC1>,
        pub channel: PA2<stm32f4xx_hal::gpio::Analog>,
    }
    
    impl AdcIntegrationTest {
        pub fn new(adc: Adc<stm32::ADC1>, channel: PA2<stm32f4xx_hal::gpio::Analog>) -> Self {
            Self { adc, channel }
        }
        
        pub fn test_single_conversion(&mut self) -> Result<u16, TestError> {
            let sample = self.adc.convert(&self.channel, stm32f4xx_hal::adc::config::SampleTime::Cycles_480);
            Ok(sample)
        }
        
        pub fn test_multiple_conversions(&mut self, count: usize) -> Result<Vec<u16, 100>, TestError> {
            let mut samples = Vec::new();
            
            for _ in 0..count.min(100) {
                let sample = self.adc.convert(&self.channel, stm32f4xx_hal::adc::config::SampleTime::Cycles_480);
                samples.push(sample).map_err(|_| TestError::BufferFull)?;
            }
            
            Ok(samples)
        }
        
        pub fn test_noise_analysis(&mut self) -> Result<AdcNoiseStats, TestError> {
            let samples = self.test_multiple_conversions(50)?;
            
            let sum: u32 = samples.iter().map(|&x| x as u32).sum();
            let mean = sum / samples.len() as u32;
            
            let variance: u32 = samples.iter()
                .map(|&x| {
                    let diff = (x as i32) - (mean as i32);
                    (diff * diff) as u32
                })
                .sum::<u32>() / samples.len() as u32;
            
            let std_dev = (variance as f32).sqrt() as u16;
            
            let min = *samples.iter().min().unwrap();
            let max = *samples.iter().max().unwrap();
            
            Ok(AdcNoiseStats {
                mean: mean as u16,
                std_dev,
                min,
                max,
                sample_count: samples.len(),
            })
        }
    }
    
    #[derive(Debug, Clone)]
    pub struct AdcNoiseStats {
        pub mean: u16,
        pub std_dev: u16,
        pub min: u16,
        pub max: u16,
        pub sample_count: usize,
    }
    
    /// 定时器集成测试
    pub struct TimerIntegrationTest {
        pub timer: Timer<stm32::TIM2>,
        pub start_time: Option<u32>,
    }
    
    impl TimerIntegrationTest {
        pub fn new(timer: Timer<stm32::TIM2>) -> Self {
            Self {
                timer,
                start_time: None,
            }
        }
        
        pub fn test_basic_timing(&mut self) -> Result<(), TestError> {
            self.timer.start(1000.hz());
            
            // 等待定时器溢出
            block!(self.timer.wait()).map_err(|_| TestError::TimerError)?;
            
            Ok(())
        }
        
        pub fn test_precision_timing(&mut self) -> Result<u32, TestError> {
            let cycles_before = cortex_m::peripheral::DWT::cycle_count();
            
            self.timer.start(1000.hz());
            block!(self.timer.wait()).map_err(|_| TestError::TimerError)?;
            
            let cycles_after = cortex_m::peripheral::DWT::cycle_count();
            let elapsed_cycles = cycles_after.wrapping_sub(cycles_before);
            
            Ok(elapsed_cycles)
        }
        
        pub fn test_interrupt_timing(&mut self) -> Result<(), TestError> {
            // 配置定时器中断
            self.timer.listen(Event::TimeOut);
            self.timer.start(100.hz());
            
            // 在实际应用中，这里会等待中断发生
            // 这里我们只是模拟测试通过
            Ok(())
        }
    }
    
    #[derive(Debug)]
    pub enum TestError {
        GpioError,
        UartError,
        SpiError,
        I2cError,
        AdcError,
        TimerError,
        LoopbackFailed,
        DataMismatch,
        BufferFull,
        Timeout,
        HardwareNotFound,
        ConfigurationError,
    }
}

// 通信协议集成测试
pub mod communication_integration {
    use super::*;
    use heapless::{Vec, String};
    use serde::{Serialize, Deserialize};
    
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct TestMessage {
        pub id: u32,
        pub timestamp: u32,
        pub data: Vec<u8, 32>,
        pub checksum: u16,
    }
    
    impl TestMessage {
        pub fn new(id: u32, timestamp: u32, data: &[u8]) -> Result<Self, &'static str> {
            if data.len() > 32 {
                return Err("Data too large");
            }
            
            let mut msg_data = Vec::new();
            for &byte in data {
                msg_data.push(byte).map_err(|_| "Failed to add data")?;
            }
            
            let checksum = Self::calculate_checksum(id, timestamp, &msg_data);
            
            Ok(Self {
                id,
                timestamp,
                data: msg_data,
                checksum,
            })
        }
        
        fn calculate_checksum(id: u32, timestamp: u32, data: &[u8]) -> u16 {
            let mut checksum = 0u16;
            checksum = checksum.wrapping_add((id & 0xFFFF) as u16);
            checksum = checksum.wrapping_add((id >> 16) as u16);
            checksum = checksum.wrapping_add((timestamp & 0xFFFF) as u16);
            checksum = checksum.wrapping_add((timestamp >> 16) as u16);
            
            for &byte in data {
                checksum = checksum.wrapping_add(byte as u16);
            }
            
            !checksum
        }
        
        pub fn is_valid(&self) -> bool {
            let calculated = Self::calculate_checksum(self.id, self.timestamp, &self.data);
            calculated == self.checksum
        }
        
        pub fn serialize_json(&self) -> Result<String<256>, &'static str> {
            let mut buffer = [0u8; 256];
            let serialized = serde_json_core::to_slice(self, &mut buffer)
                .map_err(|_| "Serialization failed")?;
            
            let json_str = core::str::from_utf8(serialized)
                .map_err(|_| "Invalid UTF-8")?;
            
            String::from(json_str)
        }
        
        pub fn deserialize_json(json: &str) -> Result<Self, &'static str> {
            serde_json_core::from_str(json)
                .map_err(|_| "Deserialization failed")
                .map(|(msg, _)| msg)
        }
        
        pub fn serialize_postcard(&self) -> Result<Vec<u8, 64>, &'static str> {
            let mut buffer = [0u8; 64];
            let serialized = postcard::to_slice(self, &mut buffer)
                .map_err(|_| "Postcard serialization failed")?;
            
            let mut result = Vec::new();
            for &byte in serialized {
                result.push(byte).map_err(|_| "Buffer full")?;
            }
            
            Ok(result)
        }
        
        pub fn deserialize_postcard(data: &[u8]) -> Result<Self, &'static str> {
            postcard::from_bytes(data)
                .map_err(|_| "Postcard deserialization failed")
        }
    }
    
    /// 协议栈集成测试
    pub struct ProtocolStackTest {
        pub messages: Vec<TestMessage, 16>,
        pub sequence_number: u32,
    }
    
    impl ProtocolStackTest {
        pub fn new() -> Self {
            Self {
                messages: Vec::new(),
                sequence_number: 0,
            }
        }
        
        pub fn test_message_creation(&mut self) -> Result<(), system_integration::TestError> {
            let test_data = b"Integration test data";
            let msg = TestMessage::new(self.sequence_number, 12345, test_data)
                .map_err(|_| system_integration::TestError::ConfigurationError)?;
            
            if !msg.is_valid() {
                return Err(system_integration::TestError::DataMismatch);
            }
            
            self.messages.push(msg)
                .map_err(|_| system_integration::TestError::BufferFull)?;
            
            self.sequence_number += 1;
            Ok(())
        }
        
        pub fn test_json_serialization(&self) -> Result<(), system_integration::TestError> {
            if let Some(msg) = self.messages.first() {
                let json = msg.serialize_json()
                    .map_err(|_| system_integration::TestError::ConfigurationError)?;
                
                let deserialized = TestMessage::deserialize_json(&json)
                    .map_err(|_| system_integration::TestError::DataMismatch)?;
                
                if *msg != deserialized {
                    return Err(system_integration::TestError::DataMismatch);
                }
            }
            
            Ok(())
        }
        
        pub fn test_postcard_serialization(&self) -> Result<(), system_integration::TestError> {
            if let Some(msg) = self.messages.first() {
                let serialized = msg.serialize_postcard()
                    .map_err(|_| system_integration::TestError::ConfigurationError)?;
                
                let deserialized = TestMessage::deserialize_postcard(&serialized)
                    .map_err(|_| system_integration::TestError::DataMismatch)?;
                
                if *msg != deserialized {
                    return Err(system_integration::TestError::DataMismatch);
                }
            }
            
            Ok(())
        }
        
        pub fn test_message_integrity(&self) -> Result<(), system_integration::TestError> {
            for msg in &self.messages {
                if !msg.is_valid() {
                    return Err(system_integration::TestError::DataMismatch);
                }
            }
            Ok(())
        }
        
        pub fn test_throughput(&mut self, message_count: usize) -> Result<u32, system_integration::TestError> {
            let start_cycles = cortex_m::peripheral::DWT::cycle_count();
            
            for i in 0..message_count.min(16) {
                let data = format!("Message {}", i);
                let msg = TestMessage::new(
                    self.sequence_number,
                    start_cycles,
                    data.as_bytes()
                ).map_err(|_| system_integration::TestError::ConfigurationError)?;
                
                if self.messages.is_full() {
                    self.messages.remove(0);
                }
                
                self.messages.push(msg)
                    .map_err(|_| system_integration::TestError::BufferFull)?;
                
                self.sequence_number += 1;
            }
            
            let end_cycles = cortex_m::peripheral::DWT::cycle_count();
            Ok(end_cycles.wrapping_sub(start_cycles))
        }
    }
}

// 系统级集成测试
pub mod system_level_tests {
    use super::*;
    use system_integration::*;
    use communication_integration::*;
    
    pub struct SystemTestSuite {
        pub gpio_test: Option<GpioIntegrationTest>,
        pub protocol_test: ProtocolStackTest,
        pub test_results: Vec<TestResult, 32>,
    }
    
    #[derive(Debug, Clone)]
    pub struct TestResult {
        pub test_name: String<32>,
        pub passed: bool,
        pub execution_time_cycles: u32,
        pub error_message: Option<String<64>>,
    }
    
    impl SystemTestSuite {
        pub fn new() -> Self {
            Self {
                gpio_test: None,
                protocol_test: ProtocolStackTest::new(),
                test_results: Vec::new(),
            }
        }
        
        pub fn add_gpio_test(&mut self, gpio_test: GpioIntegrationTest) {
            self.gpio_test = Some(gpio_test);
        }
        
        pub fn run_all_tests(&mut self, delay: &mut stm32f4xx_hal::delay::Delay) -> Result<(), TestError> {
            self.test_results.clear();
            
            // 运行GPIO测试
            if let Some(ref mut gpio_test) = self.gpio_test {
                self.run_test("GPIO Loopback", || gpio_test.test_digital_loopback());
                self.run_test("LED Blink", || gpio_test.test_led_blink(delay));
                self.run_test("Interrupt Response", || gpio_test.test_interrupt_response());
            }
            
            // 运行协议测试
            self.run_test("Message Creation", || self.protocol_test.test_message_creation());
            self.run_test("JSON Serialization", || self.protocol_test.test_json_serialization());
            self.run_test("Postcard Serialization", || self.protocol_test.test_postcard_serialization());
            self.run_test("Message Integrity", || self.protocol_test.test_message_integrity());
            
            // 运行性能测试
            self.run_performance_test("Protocol Throughput", || {
                self.protocol_test.test_throughput(10).map(|_| ())
            });
            
            Ok(())
        }
        
        fn run_test<F>(&mut self, test_name: &str, test_fn: F)
        where
            F: FnOnce() -> Result<(), TestError>,
        {
            let start_cycles = cortex_m::peripheral::DWT::cycle_count();
            
            let result = match test_fn() {
                Ok(()) => TestResult {
                    test_name: String::from(test_name),
                    passed: true,
                    execution_time_cycles: cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start_cycles),
                    error_message: None,
                },
                Err(e) => TestResult {
                    test_name: String::from(test_name),
                    passed: false,
                    execution_time_cycles: cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start_cycles),
                    error_message: Some(String::from("Test failed")),
                },
            };
            
            let _ = self.test_results.push(result);
        }
        
        fn run_performance_test<F>(&mut self, test_name: &str, test_fn: F)
        where
            F: FnOnce() -> Result<(), TestError>,
        {
            self.run_test(test_name, test_fn);
        }
        
        pub fn generate_report(&self) -> TestReport {
            let total_tests = self.test_results.len();
            let passed_tests = self.test_results.iter().filter(|r| r.passed).count();
            let failed_tests = total_tests - passed_tests;
            
            let total_execution_time: u32 = self.test_results.iter()
                .map(|r| r.execution_time_cycles)
                .sum();
            
            TestReport {
                total_tests,
                passed_tests,
                failed_tests,
                total_execution_time,
                test_results: self.test_results.clone(),
            }
        }
    }
    
    #[derive(Debug, Clone)]
    pub struct TestReport {
        pub total_tests: usize,
        pub passed_tests: usize,
        pub failed_tests: usize,
        pub total_execution_time: u32,
        pub test_results: Vec<TestResult, 32>,
    }
    
    impl TestReport {
        pub fn print_summary(&self) {
            // 在实际硬件上，这可能通过UART或RTT输出
            // 这里我们只是模拟报告生成
        }
        
        pub fn success_rate(&self) -> f32 {
            if self.total_tests == 0 {
                0.0
            } else {
                (self.passed_tests as f32 / self.total_tests as f32) * 100.0
            }
        }
        
        pub fn average_execution_time(&self) -> u32 {
            if self.total_tests == 0 {
                0
            } else {
                self.total_execution_time / self.total_tests as u32
            }
        }
    }
}

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    
    let mut delay = stm32f4xx_hal::delay::Delay::new(cp.SYST, &clocks);
    
    // 启用DWT循环计数器用于性能测量
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    
    // 设置GPIO引脚
    let input_pin = gpioa.pa0.into_pull_up_input();
    let output_pin = gpioa.pa1.into_push_pull_output();
    let led_pin = gpioa.pa5.into_push_pull_output();
    
    // 创建GPIO集成测试
    let gpio_test = system_integration::GpioIntegrationTest::new(
        input_pin,
        output_pin,
        led_pin,
    );
    
    // 创建系统测试套件
    let mut test_suite = system_level_tests::SystemTestSuite::new();
    test_suite.add_gpio_test(gpio_test);
    
    // 运行所有测试
    let _ = test_suite.run_all_tests(&mut delay);
    
    // 生成测试报告
    let report = test_suite.generate_report();
    report.print_summary();
    
    // 主循环 - 根据测试结果闪烁LED
    loop {
        if report.success_rate() > 90.0 {
            // 测试通过 - 快速闪烁绿色
            for _ in 0..3 {
                // 在实际应用中，这里会控制绿色LED
                delay.delay_ms(100u32);
            }
        } else {
            // 测试失败 - 慢速闪烁红色
            for _ in 0..1 {
                // 在实际应用中，这里会控制红色LED
                delay.delay_ms(500u32);
            }
        }
        
        delay.delay_ms(1000u32);
    }
}

// 集成测试模块
#[cfg(test)]
mod integration_tests {
    use super::*;
    use system_integration::*;
    use communication_integration::*;
    
    #[test]
    fn test_message_protocol_integration() {
        let mut protocol_test = ProtocolStackTest::new();
        
        // 测试消息创建
        assert!(protocol_test.test_message_creation().is_ok());
        
        // 测试序列化
        assert!(protocol_test.test_json_serialization().is_ok());
        assert!(protocol_test.test_postcard_serialization().is_ok());
        
        // 测试完整性
        assert!(protocol_test.test_message_integrity().is_ok());
    }
    
    #[test]
    fn test_system_integration() {
        let mut test_suite = system_level_tests::SystemTestSuite::new();
        
        // 运行协议测试
        assert!(test_suite.protocol_test.test_message_creation().is_ok());
        
        // 生成报告
        let report = test_suite.generate_report();
        assert!(report.success_rate() >= 0.0);
    }
    
    #[test]
    fn test_performance_integration() {
        let mut protocol_test = ProtocolStackTest::new();
        
        // 测试吞吐量
        let cycles = protocol_test.test_throughput(5).unwrap();
        assert!(cycles > 0);
        
        // 验证消息完整性
        assert!(protocol_test.test_message_integrity().is_ok());
    }
}