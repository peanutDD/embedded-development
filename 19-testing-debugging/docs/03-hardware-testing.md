# 硬件测试

硬件测试是嵌入式系统开发中的关键环节，确保硬件组件的功能正确性、性能指标和可靠性。本章介绍各种硬件测试方法和工具。

## 硬件在环测试 (HIL)

### HIL测试框架

```rust
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub trait HardwareInterface {
    fn initialize(&mut self) -> Result<(), HardwareError>;
    fn read_register(&self, address: u32) -> Result<u32, HardwareError>;
    fn write_register(&mut self, address: u32, value: u32) -> Result<(), HardwareError>;
    fn reset(&mut self) -> Result<(), HardwareError>;
    fn get_status(&self) -> HardwareStatus;
}

#[derive(Debug, Clone)]
pub enum HardwareError {
    InitializationFailed,
    RegisterAccessFailed,
    CommunicationTimeout,
    InvalidAddress,
    DeviceNotReady,
    PowerFailure,
}

#[derive(Debug, Clone, PartialEq)]
pub enum HardwareStatus {
    Ready,
    Busy,
    Error,
    PowerDown,
    Initializing,
}

pub struct HilTestFramework {
    hardware_interfaces: HashMap<String, Box<dyn HardwareInterface>>,
    test_results: Vec<HilTestResult>,
    current_test: Option<String>,
}

#[derive(Debug, Clone)]
pub struct HilTestResult {
    pub test_name: String,
    pub passed: bool,
    pub duration: Duration,
    pub error_message: Option<String>,
    pub measurements: HashMap<String, f64>,
}

impl HilTestFramework {
    pub fn new() -> Self {
        Self {
            hardware_interfaces: HashMap::new(),
            test_results: Vec::new(),
            current_test: None,
        }
    }
    
    pub fn add_hardware_interface(&mut self, name: String, interface: Box<dyn HardwareInterface>) {
        self.hardware_interfaces.insert(name, interface);
    }
    
    pub fn run_test<F>(&mut self, test_name: &str, test_fn: F) -> HilTestResult
    where
        F: FnOnce(&mut HashMap<String, Box<dyn HardwareInterface>>) -> Result<HashMap<String, f64>, HardwareError>,
    {
        self.current_test = Some(test_name.to_string());
        let start_time = Instant::now();
        
        let result = match test_fn(&mut self.hardware_interfaces) {
            Ok(measurements) => HilTestResult {
                test_name: test_name.to_string(),
                passed: true,
                duration: start_time.elapsed(),
                error_message: None,
                measurements,
            },
            Err(error) => HilTestResult {
                test_name: test_name.to_string(),
                passed: false,
                duration: start_time.elapsed(),
                error_message: Some(format!("{:?}", error)),
                measurements: HashMap::new(),
            },
        };
        
        self.test_results.push(result.clone());
        self.current_test = None;
        result
    }
    
    pub fn get_test_results(&self) -> &Vec<HilTestResult> {
        &self.test_results
    }
    
    pub fn generate_report(&self) -> HilTestReport {
        let total_tests = self.test_results.len();
        let passed_tests = self.test_results.iter().filter(|r| r.passed).count();
        let failed_tests = total_tests - passed_tests;
        
        let total_duration: Duration = self.test_results.iter().map(|r| r.duration).sum();
        
        HilTestReport {
            total_tests,
            passed_tests,
            failed_tests,
            pass_rate: if total_tests > 0 {
                (passed_tests as f64 / total_tests as f64) * 100.0
            } else {
                0.0
            },
            total_duration,
            test_results: self.test_results.clone(),
        }
    }
}

#[derive(Debug)]
pub struct HilTestReport {
    pub total_tests: usize,
    pub passed_tests: usize,
    pub failed_tests: usize,
    pub pass_rate: f64,
    pub total_duration: Duration,
    pub test_results: Vec<HilTestResult>,
}

// STM32F4 硬件接口实现
pub struct Stm32F4Interface {
    base_address: u32,
    initialized: bool,
    status: HardwareStatus,
}

impl Stm32F4Interface {
    pub fn new(base_address: u32) -> Self {
        Self {
            base_address,
            initialized: false,
            status: HardwareStatus::PowerDown,
        }
    }
}

impl HardwareInterface for Stm32F4Interface {
    fn initialize(&mut self) -> Result<(), HardwareError> {
        // 模拟硬件初始化过程
        self.status = HardwareStatus::Initializing;
        
        // 检查电源状态
        if self.base_address == 0 {
            return Err(HardwareError::InitializationFailed);
        }
        
        // 模拟初始化延迟
        std::thread::sleep(Duration::from_millis(10));
        
        self.initialized = true;
        self.status = HardwareStatus::Ready;
        Ok(())
    }
    
    fn read_register(&self, address: u32) -> Result<u32, HardwareError> {
        if !self.initialized {
            return Err(HardwareError::DeviceNotReady);
        }
        
        if address >= 0x1000 {
            return Err(HardwareError::InvalidAddress);
        }
        
        // 模拟寄存器读取
        let value = match address {
            0x00 => 0x12345678, // 设备ID寄存器
            0x04 => 0x00000001, // 状态寄存器
            0x08 => 0x00000000, // 控制寄存器
            _ => 0xDEADBEEF,    // 默认值
        };
        
        Ok(value)
    }
    
    fn write_register(&mut self, address: u32, value: u32) -> Result<(), HardwareError> {
        if !self.initialized {
            return Err(HardwareError::DeviceNotReady);
        }
        
        if address >= 0x1000 {
            return Err(HardwareError::InvalidAddress);
        }
        
        // 模拟寄存器写入
        match address {
            0x08 => {
                // 控制寄存器写入
                if value & 0x80000000 != 0 {
                    self.status = HardwareStatus::Busy;
                } else {
                    self.status = HardwareStatus::Ready;
                }
            }
            _ => {}
        }
        
        Ok(())
    }
    
    fn reset(&mut self) -> Result<(), HardwareError> {
        self.status = HardwareStatus::Initializing;
        std::thread::sleep(Duration::from_millis(5));
        self.status = HardwareStatus::Ready;
        Ok(())
    }
    
    fn get_status(&self) -> HardwareStatus {
        self.status.clone()
    }
}
```

### GPIO测试

```rust
pub struct GpioTestSuite {
    framework: HilTestFramework,
}

impl GpioTestSuite {
    pub fn new() -> Self {
        let mut framework = HilTestFramework::new();
        
        // 添加GPIO硬件接口
        let gpio_interface = Box::new(Stm32F4Interface::new(0x40020000)); // GPIOA base address
        framework.add_hardware_interface("gpio".to_string(), gpio_interface);
        
        Self { framework }
    }
    
    pub fn run_all_tests(&mut self) -> Vec<HilTestResult> {
        let mut results = Vec::new();
        
        results.push(self.test_gpio_initialization());
        results.push(self.test_gpio_output());
        results.push(self.test_gpio_input());
        results.push(self.test_gpio_alternate_function());
        results.push(self.test_gpio_interrupt());
        
        results
    }
    
    fn test_gpio_initialization(&mut self) -> HilTestResult {
        self.framework.run_test("gpio_initialization", |interfaces| {
            let gpio = interfaces.get_mut("gpio").unwrap();
            
            // 初始化GPIO
            gpio.initialize()?;
            
            // 验证初始化状态
            let status = gpio.get_status();
            if status != HardwareStatus::Ready {
                return Err(HardwareError::InitializationFailed);
            }
            
            // 读取设备ID
            let device_id = gpio.read_register(0x00)?;
            
            let mut measurements = HashMap::new();
            measurements.insert("device_id".to_string(), device_id as f64);
            
            Ok(measurements)
        })
    }
    
    fn test_gpio_output(&mut self) -> HilTestResult {
        self.framework.run_test("gpio_output", |interfaces| {
            let gpio = interfaces.get_mut("gpio").unwrap();
            
            // 配置GPIO为输出模式
            gpio.write_register(0x00, 0x00000001)?; // MODER寄存器，设置PA0为输出
            
            // 设置输出高电平
            gpio.write_register(0x18, 0x00000001)?; // BSRR寄存器，设置PA0
            
            // 读取输出数据寄存器
            let odr = gpio.read_register(0x14)?; // ODR寄存器
            
            // 验证输出状态
            if odr & 0x00000001 == 0 {
                return Err(HardwareError::RegisterAccessFailed);
            }
            
            // 设置输出低电平
            gpio.write_register(0x18, 0x00010000)?; // BSRR寄存器，复位PA0
            
            let odr = gpio.read_register(0x14)?;
            if odr & 0x00000001 != 0 {
                return Err(HardwareError::RegisterAccessFailed);
            }
            
            let mut measurements = HashMap::new();
            measurements.insert("output_high_level".to_string(), 1.0);
            measurements.insert("output_low_level".to_string(), 0.0);
            
            Ok(measurements)
        })
    }
    
    fn test_gpio_input(&mut self) -> HilTestResult {
        self.framework.run_test("gpio_input", |interfaces| {
            let gpio = interfaces.get_mut("gpio").unwrap();
            
            // 配置GPIO为输入模式
            gpio.write_register(0x00, 0x00000000)?; // MODER寄存器，设置PA0为输入
            
            // 配置上拉电阻
            gpio.write_register(0x0C, 0x00000001)?; // PUPDR寄存器，设置PA0上拉
            
            // 读取输入数据寄存器
            let idr = gpio.read_register(0x10)?; // IDR寄存器
            
            // 模拟输入信号变化测试
            let mut input_changes = 0;
            for _ in 0..10 {
                let current_idr = gpio.read_register(0x10)?;
                if current_idr != idr {
                    input_changes += 1;
                }
                std::thread::sleep(Duration::from_millis(1));
            }
            
            let mut measurements = HashMap::new();
            measurements.insert("input_value".to_string(), (idr & 0x00000001) as f64);
            measurements.insert("input_changes".to_string(), input_changes as f64);
            
            Ok(measurements)
        })
    }
    
    fn test_gpio_alternate_function(&mut self) -> HilTestResult {
        self.framework.run_test("gpio_alternate_function", |interfaces| {
            let gpio = interfaces.get_mut("gpio").unwrap();
            
            // 配置GPIO为复用功能模式
            gpio.write_register(0x00, 0x00000002)?; // MODER寄存器，设置PA0为复用功能
            
            // 设置复用功能选择
            gpio.write_register(0x20, 0x00000007)?; // AFRL寄存器，设置PA0为AF7
            
            // 验证配置
            let moder = gpio.read_register(0x00)?;
            let afrl = gpio.read_register(0x20)?;
            
            if (moder & 0x00000003) != 0x00000002 {
                return Err(HardwareError::RegisterAccessFailed);
            }
            
            if (afrl & 0x0000000F) != 0x00000007 {
                return Err(HardwareError::RegisterAccessFailed);
            }
            
            let mut measurements = HashMap::new();
            measurements.insert("alternate_function".to_string(), 7.0);
            measurements.insert("mode_setting".to_string(), 2.0);
            
            Ok(measurements)
        })
    }
    
    fn test_gpio_interrupt(&mut self) -> HilTestResult {
        self.framework.run_test("gpio_interrupt", |interfaces| {
            let gpio = interfaces.get_mut("gpio").unwrap();
            
            // 配置GPIO为输入模式
            gpio.write_register(0x00, 0x00000000)?; // MODER寄存器
            
            // 模拟EXTI配置（外部中断）
            // 这里简化处理，实际需要配置EXTI寄存器
            
            // 模拟中断触发测试
            let start_time = Instant::now();
            let mut interrupt_count = 0;
            
            // 模拟中断信号
            for i in 0..5 {
                // 模拟边沿触发
                gpio.write_register(0x100, i % 2)?; // 模拟中断状态寄存器
                
                let interrupt_status = gpio.read_register(0x100)?;
                if interrupt_status != 0 {
                    interrupt_count += 1;
                }
                
                std::thread::sleep(Duration::from_millis(2));
            }
            
            let test_duration = start_time.elapsed();
            
            let mut measurements = HashMap::new();
            measurements.insert("interrupt_count".to_string(), interrupt_count as f64);
            measurements.insert("test_duration_ms".to_string(), test_duration.as_millis() as f64);
            
            Ok(measurements)
        })
    }
}
```

## ADC/DAC测试

### ADC测试套件

```rust
pub struct AdcTestSuite {
    framework: HilTestFramework,
}

impl AdcTestSuite {
    pub fn new() -> Self {
        let mut framework = HilTestFramework::new();
        
        // 添加ADC硬件接口
        let adc_interface = Box::new(Stm32F4Interface::new(0x40012000)); // ADC1 base address
        framework.add_hardware_interface("adc".to_string(), adc_interface);
        
        Self { framework }
    }
    
    pub fn run_all_tests(&mut self) -> Vec<HilTestResult> {
        let mut results = Vec::new();
        
        results.push(self.test_adc_initialization());
        results.push(self.test_adc_single_conversion());
        results.push(self.test_adc_continuous_conversion());
        results.push(self.test_adc_multi_channel());
        results.push(self.test_adc_dma_transfer());
        results.push(self.test_adc_calibration());
        
        results
    }
    
    fn test_adc_initialization(&mut self) -> HilTestResult {
        self.framework.run_test("adc_initialization", |interfaces| {
            let adc = interfaces.get_mut("adc").unwrap();
            
            // 初始化ADC
            adc.initialize()?;
            
            // 配置ADC控制寄存器
            adc.write_register(0x08, 0x00000001)?; // CR2寄存器，使能ADC
            
            // 等待ADC稳定
            std::thread::sleep(Duration::from_millis(1));
            
            // 读取状态寄存器
            let sr = adc.read_register(0x00)?; // SR寄存器
            
            // 验证ADC就绪状态
            if sr & 0x00000001 == 0 {
                return Err(HardwareError::InitializationFailed);
            }
            
            let mut measurements = HashMap::new();
            measurements.insert("adc_ready".to_string(), 1.0);
            measurements.insert("status_register".to_string(), sr as f64);
            
            Ok(measurements)
        })
    }
    
    fn test_adc_single_conversion(&mut self) -> HilTestResult {
        self.framework.run_test("adc_single_conversion", |interfaces| {
            let adc = interfaces.get_mut("adc").unwrap();
            
            // 配置单次转换模式
            adc.write_register(0x08, 0x00000001)?; // CR2寄存器，使能ADC
            adc.write_register(0x04, 0x00000000)?; // CR1寄存器，单次转换模式
            
            // 配置采样时间
            adc.write_register(0x10, 0x00000007)?; // SMPR2寄存器，设置通道0采样时间
            
            // 配置转换序列
            adc.write_register(0x2C, 0x00000000)?; // SQR3寄存器，设置第一个转换为通道0
            adc.write_register(0x30, 0x00000000)?; // SQR1寄存器，设置转换长度为1
            
            // 启动转换
            adc.write_register(0x08, 0x40000001)?; // CR2寄存器，启动转换
            
            // 等待转换完成
            let start_time = Instant::now();
            let mut conversion_complete = false;
            
            while start_time.elapsed() < Duration::from_millis(10) {
                let sr = adc.read_register(0x00)?;
                if sr & 0x00000002 != 0 { // EOC位
                    conversion_complete = true;
                    break;
                }
                std::thread::sleep(Duration::from_micros(10));
            }
            
            if !conversion_complete {
                return Err(HardwareError::CommunicationTimeout);
            }
            
            // 读取转换结果
            let dr = adc.read_register(0x4C)?; // DR寄存器
            let conversion_time = start_time.elapsed();
            
            // 验证转换结果范围（12位ADC）
            if dr > 4095 {
                return Err(HardwareError::RegisterAccessFailed);
            }
            
            let mut measurements = HashMap::new();
            measurements.insert("conversion_result".to_string(), dr as f64);
            measurements.insert("conversion_time_us".to_string(), conversion_time.as_micros() as f64);
            measurements.insert("voltage_mv".to_string(), (dr as f64 * 3300.0) / 4095.0);
            
            Ok(measurements)
        })
    }
    
    fn test_adc_continuous_conversion(&mut self) -> HilTestResult {
        self.framework.run_test("adc_continuous_conversion", |interfaces| {
            let adc = interfaces.get_mut("adc").unwrap();
            
            // 配置连续转换模式
            adc.write_register(0x08, 0x00000003)?; // CR2寄存器，使能ADC和连续转换
            adc.write_register(0x04, 0x00000002)?; // CR1寄存器，连续转换模式
            
            // 启动连续转换
            adc.write_register(0x08, 0x40000003)?; // CR2寄存器，启动转换
            
            let mut conversion_results = Vec::new();
            let test_duration = Duration::from_millis(100);
            let start_time = Instant::now();
            
            while start_time.elapsed() < test_duration {
                let sr = adc.read_register(0x00)?;
                if sr & 0x00000002 != 0 { // EOC位
                    let dr = adc.read_register(0x4C)?;
                    conversion_results.push(dr);
                    
                    // 清除EOC标志（读取DR寄存器会自动清除）
                }
                std::thread::sleep(Duration::from_micros(100));
            }
            
            // 停止转换
            adc.write_register(0x08, 0x00000001)?; // CR2寄存器，停止转换
            
            if conversion_results.is_empty() {
                return Err(HardwareError::CommunicationTimeout);
            }
            
            // 计算统计信息
            let min_value = *conversion_results.iter().min().unwrap();
            let max_value = *conversion_results.iter().max().unwrap();
            let avg_value = conversion_results.iter().sum::<u32>() as f64 / conversion_results.len() as f64;
            let conversion_rate = conversion_results.len() as f64 / test_duration.as_secs_f64();
            
            let mut measurements = HashMap::new();
            measurements.insert("total_conversions".to_string(), conversion_results.len() as f64);
            measurements.insert("conversion_rate_hz".to_string(), conversion_rate);
            measurements.insert("min_value".to_string(), min_value as f64);
            measurements.insert("max_value".to_string(), max_value as f64);
            measurements.insert("avg_value".to_string(), avg_value);
            
            Ok(measurements)
        })
    }
    
    fn test_adc_multi_channel(&mut self) -> HilTestResult {
        self.framework.run_test("adc_multi_channel", |interfaces| {
            let adc = interfaces.get_mut("adc").unwrap();
            
            // 配置多通道扫描模式
            adc.write_register(0x04, 0x00000100)?; // CR1寄存器，使能扫描模式
            adc.write_register(0x08, 0x00000001)?; // CR2寄存器，使能ADC
            
            // 配置转换序列（通道0, 1, 2）
            adc.write_register(0x2C, 0x00000210)?; // SQR3寄存器，设置前3个转换
            adc.write_register(0x30, 0x00200000)?; // SQR1寄存器，设置转换长度为3
            
            // 配置采样时间
            adc.write_register(0x10, 0x00000777)?; // SMPR2寄存器，设置通道0-2采样时间
            
            // 启动转换
            adc.write_register(0x08, 0x40000001)?; // CR2寄存器，启动转换
            
            let mut channel_results = HashMap::new();
            let start_time = Instant::now();
            
            // 等待所有通道转换完成
            for channel in 0..3 {
                let mut conversion_complete = false;
                let channel_start = Instant::now();
                
                while channel_start.elapsed() < Duration::from_millis(10) {
                    let sr = adc.read_register(0x00)?;
                    if sr & 0x00000002 != 0 { // EOC位
                        let dr = adc.read_register(0x4C)?;
                        channel_results.insert(format!("channel_{}", channel), dr as f64);
                        conversion_complete = true;
                        break;
                    }
                    std::thread::sleep(Duration::from_micros(10));
                }
                
                if !conversion_complete {
                    return Err(HardwareError::CommunicationTimeout);
                }
            }
            
            let total_time = start_time.elapsed();
            
            let mut measurements = HashMap::new();
            measurements.extend(channel_results);
            measurements.insert("total_conversion_time_us".to_string(), total_time.as_micros() as f64);
            measurements.insert("channels_converted".to_string(), 3.0);
            
            Ok(measurements)
        })
    }
    
    fn test_adc_dma_transfer(&mut self) -> HilTestResult {
        self.framework.run_test("adc_dma_transfer", |interfaces| {
            let adc = interfaces.get_mut("adc").unwrap();
            
            // 配置DMA模式
            adc.write_register(0x08, 0x00000101)?; // CR2寄存器，使能ADC和DMA
            adc.write_register(0x04, 0x00000102)?; // CR1寄存器，连续转换和扫描模式
            
            // 模拟DMA配置（实际需要配置DMA控制器）
            let dma_buffer_size = 100;
            let mut dma_transfers = 0;
            
            // 启动DMA转换
            adc.write_register(0x08, 0x40000101)?; // CR2寄存器，启动转换
            
            let start_time = Instant::now();
            let test_duration = Duration::from_millis(50);
            
            while start_time.elapsed() < test_duration {
                // 模拟DMA传输完成检查
                let sr = adc.read_register(0x00)?;
                if sr & 0x00000002 != 0 { // EOC位
                    dma_transfers += 1;
                    
                    // 模拟读取DMA缓冲区
                    let _dr = adc.read_register(0x4C)?;
                }
                
                std::thread::sleep(Duration::from_micros(50));
            }
            
            // 停止DMA转换
            adc.write_register(0x08, 0x00000001)?; // CR2寄存器，停止转换
            
            let transfer_rate = dma_transfers as f64 / test_duration.as_secs_f64();
            
            let mut measurements = HashMap::new();
            measurements.insert("dma_transfers".to_string(), dma_transfers as f64);
            measurements.insert("transfer_rate_hz".to_string(), transfer_rate);
            measurements.insert("buffer_utilization".to_string(), 
                              (dma_transfers as f64 / dma_buffer_size as f64) * 100.0);
            
            Ok(measurements)
        })
    }
    
    fn test_adc_calibration(&mut self) -> HilTestResult {
        self.framework.run_test("adc_calibration", |interfaces| {
            let adc = interfaces.get_mut("adc").unwrap();
            
            // 启动ADC校准
            adc.write_register(0x08, 0x00000004)?; // CR2寄存器，启动校准
            
            // 等待校准完成
            let start_time = Instant::now();
            let mut calibration_complete = false;
            
            while start_time.elapsed() < Duration::from_millis(100) {
                let cr2 = adc.read_register(0x08)?;
                if cr2 & 0x00000004 == 0 { // CAL位清零表示校准完成
                    calibration_complete = true;
                    break;
                }
                std::thread::sleep(Duration::from_millis(1));
            }
            
            if !calibration_complete {
                return Err(HardwareError::CommunicationTimeout);
            }
            
            let calibration_time = start_time.elapsed();
            
            // 验证校准后的转换精度
            adc.write_register(0x08, 0x00000001)?; // CR2寄存器，使能ADC
            
            // 进行几次测试转换
            let mut test_results = Vec::new();
            for _ in 0..5 {
                adc.write_register(0x08, 0x40000001)?; // 启动转换
                
                // 等待转换完成
                let conv_start = Instant::now();
                while conv_start.elapsed() < Duration::from_millis(10) {
                    let sr = adc.read_register(0x00)?;
                    if sr & 0x00000002 != 0 {
                        let dr = adc.read_register(0x4C)?;
                        test_results.push(dr);
                        break;
                    }
                    std::thread::sleep(Duration::from_micros(10));
                }
            }
            
            // 计算转换结果的稳定性
            if !test_results.is_empty() {
                let avg = test_results.iter().sum::<u32>() as f64 / test_results.len() as f64;
                let variance = test_results.iter()
                    .map(|&x| (x as f64 - avg).powi(2))
                    .sum::<f64>() / test_results.len() as f64;
                let std_dev = variance.sqrt();
                
                let mut measurements = HashMap::new();
                measurements.insert("calibration_time_ms".to_string(), calibration_time.as_millis() as f64);
                measurements.insert("post_cal_avg".to_string(), avg);
                measurements.insert("post_cal_std_dev".to_string(), std_dev);
                measurements.insert("calibration_success".to_string(), 1.0);
                
                Ok(measurements)
            } else {
                Err(HardwareError::CommunicationTimeout)
            }
        })
    }
}
```

## 通信接口测试

### UART测试

```rust
pub struct UartTestSuite {
    framework: HilTestFramework,
}

impl UartTestSuite {
    pub fn new() -> Self {
        let mut framework = HilTestFramework::new();
        
        // 添加UART硬件接口
        let uart_interface = Box::new(Stm32F4Interface::new(0x40011000)); // USART1 base address
        framework.add_hardware_interface("uart".to_string(), uart_interface);
        
        Self { framework }
    }
    
    pub fn run_all_tests(&mut self) -> Vec<HilTestResult> {
        let mut results = Vec::new();
        
        results.push(self.test_uart_initialization());
        results.push(self.test_uart_baud_rate());
        results.push(self.test_uart_data_transmission());
        results.push(self.test_uart_data_reception());
        results.push(self.test_uart_flow_control());
        results.push(self.test_uart_error_handling());
        
        results
    }
    
    fn test_uart_initialization(&mut self) -> HilTestResult {
        self.framework.run_test("uart_initialization", |interfaces| {
            let uart = interfaces.get_mut("uart").unwrap();
            
            // 初始化UART
            uart.initialize()?;
            
            // 配置UART控制寄存器
            uart.write_register(0x0C, 0x0000200C)?; // CR1寄存器，使能UART、发送和接收
            
            // 配置波特率（假设系统时钟84MHz，波特率115200）
            let brr_value = 84000000 / 115200;
            uart.write_register(0x08, brr_value)?; // BRR寄存器
            
            // 读取状态寄存器
            let sr = uart.read_register(0x00)?; // SR寄存器
            
            // 验证UART就绪状态
            if sr & 0x00000080 == 0 { // TXE位应该为1
                return Err(HardwareError::InitializationFailed);
            }
            
            let mut measurements = HashMap::new();
            measurements.insert("uart_ready".to_string(), 1.0);
            measurements.insert("baud_rate".to_string(), 115200.0);
            measurements.insert("status_register".to_string(), sr as f64);
            
            Ok(measurements)
        })
    }
    
    fn test_uart_baud_rate(&mut self) -> HilTestResult {
        self.framework.run_test("uart_baud_rate", |interfaces| {
            let uart = interfaces.get_mut("uart").unwrap();
            
            let test_baud_rates = vec![9600, 38400, 115200, 230400, 460800];
            let mut baud_rate_results = HashMap::new();
            
            for &baud_rate in &test_baud_rates {
                // 计算BRR值
                let brr_value = 84000000 / baud_rate;
                uart.write_register(0x08, brr_value)?; // BRR寄存器
                
                // 验证波特率设置
                let actual_brr = uart.read_register(0x08)?;
                let actual_baud_rate = 84000000 / actual_brr;
                
                // 计算误差
                let error_percent = ((actual_baud_rate as f64 - baud_rate as f64).abs() / baud_rate as f64) * 100.0;
                
                baud_rate_results.insert(
                    format!("baud_{}_error_percent", baud_rate),
                    error_percent
                );
                
                // 误差应该小于2%
                if error_percent > 2.0 {
                    return Err(HardwareError::RegisterAccessFailed);
                }
            }
            
            let mut measurements = HashMap::new();
            measurements.extend(baud_rate_results);
            measurements.insert("tested_baud_rates".to_string(), test_baud_rates.len() as f64);
            
            Ok(measurements)
        })
    }
    
    fn test_uart_data_transmission(&mut self) -> HilTestResult {
        self.framework.run_test("uart_data_transmission", |interfaces| {
            let uart = interfaces.get_mut("uart").unwrap();
            
            let test_data = b"Hello, UART Test!";
            let mut transmitted_bytes = 0;
            let start_time = Instant::now();
            
            for &byte in test_data {
                // 等待发送缓冲区空
                let mut tx_ready = false;
                let tx_start = Instant::now();
                
                while tx_start.elapsed() < Duration::from_millis(10) {
                    let sr = uart.read_register(0x00)?; // SR寄存器
                    if sr & 0x00000080 != 0 { // TXE位
                        tx_ready = true;
                        break;
                    }
                    std::thread::sleep(Duration::from_micros(10));
                }
                
                if !tx_ready {
                    return Err(HardwareError::CommunicationTimeout);
                }
                
                // 发送数据
                uart.write_register(0x04, byte as u32)?; // DR寄存器
                transmitted_bytes += 1;
                
                // 等待传输完成
                let mut tx_complete = false;
                let tc_start = Instant::now();
                
                while tc_start.elapsed() < Duration::from_millis(10) {
                    let sr = uart.read_register(0x00)?;
                    if sr & 0x00000040 != 0 { // TC位
                        tx_complete = true;
                        break;
                    }
                    std::thread::sleep(Duration::from_micros(10));
                }
                
                if !tx_complete {
                    return Err(HardwareError::CommunicationTimeout);
                }
            }
            
            let transmission_time = start_time.elapsed();
            let throughput = (transmitted_bytes as f64 * 8.0) / transmission_time.as_secs_f64(); // bits per second
            
            let mut measurements = HashMap::new();
            measurements.insert("transmitted_bytes".to_string(), transmitted_bytes as f64);
            measurements.insert("transmission_time_ms".to_string(), transmission_time.as_millis() as f64);
            measurements.insert("throughput_bps".to_string(), throughput);
            
            Ok(measurements)
        })
    }
    
    fn test_uart_data_reception(&mut self) -> HilTestResult {
        self.framework.run_test("uart_data_reception", |interfaces| {
            let uart = interfaces.get_mut("uart").unwrap();
            
            // 模拟接收数据（在实际测试中，需要外部设备发送数据）
            let expected_data = b"Test Reception";
            let mut received_bytes = Vec::new();
            let start_time = Instant::now();
            let timeout = Duration::from_millis(100);
            
            // 模拟数据接收过程
            for &expected_byte in expected_data {
                // 模拟数据到达
                uart.write_register(0x04, expected_byte as u32)?; // 模拟DR寄存器接收数据
                uart.write_register(0x00, 0x00000020)?; // 设置RXNE位
                
                // 检查接收缓冲区
                let mut rx_ready = false;
                let rx_start = Instant::now();
                
                while rx_start.elapsed() < Duration::from_millis(10) {
                    let sr = uart.read_register(0x00)?; // SR寄存器
                    if sr & 0x00000020 != 0 { // RXNE位
                        rx_ready = true;
                        break;
                    }
                    std::thread::sleep(Duration::from_micros(10));
                }
                
                if !rx_ready {
                    return Err(HardwareError::CommunicationTimeout);
                }
                
                // 读取接收数据
                let received_byte = uart.read_register(0x04)? as u8; // DR寄存器
                received_bytes.push(received_byte);
                
                // 清除RXNE标志（读取DR寄存器会自动清除）
                uart.write_register(0x00, 0x00000000)?;
            }
            
            let reception_time = start_time.elapsed();
            
            // 验证接收数据
            if received_bytes != expected_data {
                return Err(HardwareError::RegisterAccessFailed);
            }
            
            let throughput = (received_bytes.len() as f64 * 8.0) / reception_time.as_secs_f64();
            
            let mut measurements = HashMap::new();
            measurements.insert("received_bytes".to_string(), received_bytes.len() as f64);
            measurements.insert("reception_time_ms".to_string(), reception_time.as_millis() as f64);
            measurements.insert("reception_throughput_bps".to_string(), throughput);
            measurements.insert("data_integrity".to_string(), 1.0);
            
            Ok(measurements)
        })
    }
    
    fn test_uart_flow_control(&mut self) -> HilTestResult {
        self.framework.run_test("uart_flow_control", |interfaces| {
            let uart = interfaces.get_mut("uart").unwrap();
            
            // 配置硬件流控制
            uart.write_register(0x10, 0x00000300)?; // CR3寄存器，使能RTS/CTS
            
            // 测试RTS信号控制
            uart.write_register(0x10, 0x00000100)?; // 设置RTS
            let cr3_rts = uart.read_register(0x10)?;
            
            if cr3_rts & 0x00000100 == 0 {
                return Err(HardwareError::RegisterAccessFailed);
            }
            
            // 测试CTS信号检测
            uart.write_register(0x00, 0x00000200)?; // 模拟CTS信号
            let sr_cts = uart.read_register(0x00)?;
            
            // 模拟流控制场景
            let mut flow_control_events = 0;
            
            for i in 0..10 {
                // 模拟发送缓冲区满
                if i % 3 == 0 {
                    uart.write_register(0x00, 0x00000000)?; // 清除TXE位，模拟缓冲区满
                    flow_control_events += 1;
                } else {
                    uart.write_register(0x00, 0x00000080)?; // 设置TXE位，缓冲区可用
                }
                
                std::thread::sleep(Duration::from_millis(1));
            }
            
            let mut measurements = HashMap::new();
            measurements.insert("rts_control".to_string(), 1.0);
            measurements.insert("cts_detection".to_string(), if sr_cts & 0x00000200 != 0 { 1.0 } else { 0.0 });
            measurements.insert("flow_control_events".to_string(), flow_control_events as f64);
            
            Ok(measurements)
        })
    }
    
    fn test_uart_error_handling(&mut self) -> HilTestResult {
        self.framework.run_test("uart_error_handling", |interfaces| {
            let uart = interfaces.get_mut("uart").unwrap();
            
            let mut error_tests = HashMap::new();
            
            // 测试帧错误
            uart.write_register(0x00, 0x00000008)?; // 设置FE位
            let sr_fe = uart.read_register(0x00)?;
            error_tests.insert("frame_error_detection", if sr_fe & 0x00000008 != 0 { 1.0 } else { 0.0 });
            
            // 清除帧错误
            uart.write_register(0x00, 0x00000000)?;
            
            // 测试噪声错误
            uart.write_register(0x00, 0x00000004)?; // 设置NE位
            let sr_ne = uart.read_register(0x00)?;
            error_tests.insert("noise_error_detection", if sr_ne & 0x00000004 != 0 { 1.0 } else { 0.0 });
            
            // 清除噪声错误
            uart.write_register(0x00, 0x00000000)?;
            
            // 测试溢出错误
            uart.write_register(0x00, 0x00000008)?; // 设置ORE位
            let sr_ore = uart.read_register(0x00)?;
            error_tests.insert("overrun_error_detection", if sr_ore & 0x00000008 != 0 { 1.0 } else { 0.0 });
            
            // 清除溢出错误
            uart.write_register(0x00, 0x00000000)?;
            
            // 测试奇偶校验错误
            uart.write_register(0x00, 0x00000001)?; // 设置PE位
            let sr_pe = uart.read_register(0x00)?;
            error_tests.insert("parity_error_detection", if sr_pe & 0x00000001 != 0 { 1.0 } else { 0.0 });
            
            // 清除奇偶校验错误
            uart.write_register(0x00, 0x00000000)?;
            
            // 验证错误恢复
            let final_sr = uart.read_register(0x00)?;
            error_tests.insert("error_recovery", if final_sr == 0 { 1.0 } else { 0.0 });
            
            let mut measurements = HashMap::new();
            measurements.extend(error_tests);
            measurements.insert("total_error_types_tested".to_string(), 4.0);
            
            Ok(measurements)
        })
    }
}
```

## 总结

硬件测试是确保嵌入式系统可靠性的重要环节：

1. **HIL测试框架**：提供统一的硬件测试接口和结果管理
2. **GPIO测试**：验证数字I/O功能的正确性和性能
3. **ADC/DAC测试**：确保模拟信号转换的精度和稳定性
4. **通信接口测试**：验证串行通信的可靠性和性能
5. **自动化测试**：通过自动化测试提高测试效率和覆盖率

通过系统性的硬件测试，可以及早发现硬件问题，确保系统在各种条件下的稳定运行。