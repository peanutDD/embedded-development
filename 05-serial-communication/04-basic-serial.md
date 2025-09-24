# 基础串口应用

## 概述

本文档介绍如何使用Rust在STM32平台上实现基础的串口通信应用。从简单的字符收发到复杂的数据处理，逐步构建完整的串口通信系统。

## 开发环境准备

### 硬件要求

- STM32F407VG Discovery开发板
- USB转串口模块（CP2102/FT232RL）
- 杜邦线若干
- 面包板（可选）

### 软件环境

```toml
# Cargo.toml
[package]
name = "basic-serial"
version = "0.1.0"
edition = "2021"

[dependencies]
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"
stm32f4xx-hal = { version = "0.14", features = ["stm32f407"] }
embedded-hal = "0.2"
nb = "1.0"
heapless = "0.7"

[dependencies.cortex-m-semihosting]
version = "0.5"
optional = true

[[bin]]
name = "echo_server"
path = "src/echo_server.rs"

[[bin]]
name = "command_parser"
path = "src/command_parser.rs"

[[bin]]
name = "data_logger"
path = "src/data_logger.rs"
```

### 硬件连接

```
STM32F407VG Discovery    USB转串口模块
--------------------    --------------
PA9 (USART1_TX)    →    RX
PA10 (USART1_RX)   ←    TX
GND                →    GND
```

## 基础示例

### 1. 简单回显服务器

最基础的串口应用是回显服务器，接收数据并原样返回。

```rust
// src/echo_server.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{config::Config, Serial},
};
use embedded_hal::serial::{Read, Write};
use nb::block;

#[entry]
fn main() -> ! {
    // 获取外设
    let dp = pac::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    
    // 配置串口
    let config = Config::default()
        .baudrate(115200.bps())
        .wordlength_8()
        .parity_none()
        .stopbits(stm32f4xx_hal::serial::StopBits::STOP1);
    
    let mut uart = Serial::new(
        dp.USART1,
        (tx_pin, rx_pin),
        config,
        &clocks,
    ).unwrap();
    
    // 发送启动消息
    let welcome_msg = b"Echo Server Started\r\n";
    for &byte in welcome_msg {
        block!(uart.write(byte)).unwrap();
    }
    block!(uart.flush()).unwrap();
    
    // 主循环：回显接收到的数据
    loop {
        match block!(uart.read()) {
            Ok(byte) => {
                // 回显接收到的字节
                block!(uart.write(byte)).unwrap();
                
                // 如果是回车，添加换行
                if byte == b'\r' {
                    block!(uart.write(b'\n')).unwrap();
                }
            }
            Err(_) => {
                // 处理错误
            }
        }
    }
}
```

### 2. 格式化输出支持

为了方便调试和输出，实现格式化输出功能。

```rust
// src/lib.rs
#![no_std]

use core::fmt::Write;
use embedded_hal::serial::Write as SerialWrite;
use nb::block;

pub struct UartWriter<UART> {
    uart: UART,
}

impl<UART> UartWriter<UART> {
    pub fn new(uart: UART) -> Self {
        Self { uart }
    }
}

impl<UART> Write for UartWriter<UART>
where
    UART: SerialWrite<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.bytes() {
            block!(self.uart.write(byte)).map_err(|_| core::fmt::Error)?;
        }
        Ok(())
    }
}

// 便利宏
#[macro_export]
macro_rules! uart_print {
    ($uart:expr, $($arg:tt)*) => {
        write!($uart, $($arg)*).ok()
    };
}

#[macro_export]
macro_rules! uart_println {
    ($uart:expr, $($arg:tt)*) => {
        writeln!($uart, $($arg)*).ok()
    };
}
```

### 3. 命令解析器

实现一个简单的命令行界面，支持基本命令处理。

```rust
// src/command_parser.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{config::Config, Serial},
    gpio::{Output, PushPull, gpiod::*},
};
use embedded_hal::serial::{Read, Write};
use nb::block;
use heapless::{String, Vec};
use core::fmt::Write;
use basic_serial::{UartWriter, uart_println};

// LED控制结构
struct LedController {
    led_green: PD12<Output<PushPull>>,
    led_orange: PD13<Output<PushPull>>,
    led_red: PD14<Output<PushPull>>,
    led_blue: PD15<Output<PushPull>>,
}

impl LedController {
    fn new(
        led_green: PD12<Output<PushPull>>,
        led_orange: PD13<Output<PushPull>>,
        led_red: PD14<Output<PushPull>>,
        led_blue: PD15<Output<PushPull>>,
    ) -> Self {
        Self {
            led_green,
            led_orange,
            led_red,
            led_blue,
        }
    }
    
    fn set_led(&mut self, color: &str, state: bool) -> Result<(), &'static str> {
        match color {
            "green" => {
                if state {
                    self.led_green.set_high();
                } else {
                    self.led_green.set_low();
                }
            }
            "orange" => {
                if state {
                    self.led_orange.set_high();
                } else {
                    self.led_orange.set_low();
                }
            }
            "red" => {
                if state {
                    self.led_red.set_high();
                } else {
                    self.led_red.set_low();
                }
            }
            "blue" => {
                if state {
                    self.led_blue.set_high();
                } else {
                    self.led_blue.set_low();
                }
            }
            _ => return Err("Invalid LED color"),
        }
        Ok(())
    }
    
    fn all_off(&mut self) {
        self.led_green.set_low();
        self.led_orange.set_low();
        self.led_red.set_low();
        self.led_blue.set_low();
    }
}

// 命令处理器
struct CommandProcessor {
    buffer: String<128>,
    led_controller: LedController,
}

impl CommandProcessor {
    fn new(led_controller: LedController) -> Self {
        Self {
            buffer: String::new(),
            led_controller,
        }
    }
    
    fn process_char<UART>(&mut self, uart: &mut UartWriter<UART>, ch: u8) 
    where
        UART: Write<u8>,
    {
        match ch {
            b'\r' | b'\n' => {
                if !self.buffer.is_empty() {
                    self.execute_command(uart);
                    self.buffer.clear();
                }
                uart_print!(uart, "\r\n> ");
            }
            b'\x08' | b'\x7f' => { // Backspace or DEL
                if self.buffer.pop().is_some() {
                    uart_print!(uart, "\x08 \x08"); // 退格并清除字符
                }
            }
            0x20..=0x7E => { // 可打印字符
                if self.buffer.len() < self.buffer.capacity() - 1 {
                    self.buffer.push(ch as char).ok();
                    uart_print!(uart, "{}", ch as char);
                }
            }
            _ => {
                // 忽略其他字符
            }
        }
    }
    
    fn execute_command<UART>(&mut self, uart: &mut UartWriter<UART>)
    where
        UART: Write<u8>,
    {
        let cmd_line = self.buffer.trim();
        let parts: Vec<&str, 8> = cmd_line.split_whitespace().collect();
        
        if parts.is_empty() {
            return;
        }
        
        match parts[0] {
            "help" => {
                uart_println!(uart, "Available commands:");
                uart_println!(uart, "  help           - Show this help");
                uart_println!(uart, "  led <color> on - Turn on LED (green/orange/red/blue)");
                uart_println!(uart, "  led <color> off- Turn off LED");
                uart_println!(uart, "  led all off    - Turn off all LEDs");
                uart_println!(uart, "  status         - Show system status");
                uart_println!(uart, "  echo <text>    - Echo back text");
            }
            "led" => {
                self.handle_led_command(uart, &parts[1..]);
            }
            "status" => {
                uart_println!(uart, "System Status:");
                uart_println!(uart, "  MCU: STM32F407VG");
                uart_println!(uart, "  Clock: 168MHz");
                uart_println!(uart, "  UART: 115200 bps");
            }
            "echo" => {
                if parts.len() > 1 {
                    let text = &cmd_line[5..]; // Skip "echo "
                    uart_println!(uart, "{}", text);
                } else {
                    uart_println!(uart, "Usage: echo <text>");
                }
            }
            _ => {
                uart_println!(uart, "Unknown command: {}. Type 'help' for available commands.", parts[0]);
            }
        }
    }
    
    fn handle_led_command<UART>(&mut self, uart: &mut UartWriter<UART>, args: &[&str])
    where
        UART: Write<u8>,
    {
        if args.len() < 2 {
            uart_println!(uart, "Usage: led <color> <on|off> or led all off");
            return;
        }
        
        if args[0] == "all" && args[1] == "off" {
            self.led_controller.all_off();
            uart_println!(uart, "All LEDs turned off");
            return;
        }
        
        let color = args[0];
        let state = match args[1] {
            "on" => true,
            "off" => false,
            _ => {
                uart_println!(uart, "Invalid state. Use 'on' or 'off'");
                return;
            }
        };
        
        match self.led_controller.set_led(color, state) {
            Ok(()) => {
                uart_println!(uart, "LED {} turned {}", color, if state { "on" } else { "off" });
            }
            Err(e) => {
                uart_println!(uart, "Error: {}", e);
            }
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiod = dp.GPIOD.split();
    
    // 配置串口引脚
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    
    // 配置LED引脚
    let led_green = gpiod.pd12.into_push_pull_output();
    let led_orange = gpiod.pd13.into_push_pull_output();
    let led_red = gpiod.pd14.into_push_pull_output();
    let led_blue = gpiod.pd15.into_push_pull_output();
    
    // 初始化串口
    let config = Config::default().baudrate(115200.bps());
    let uart = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    let mut uart_writer = UartWriter::new(uart);
    
    // 初始化命令处理器
    let led_controller = LedController::new(led_green, led_orange, led_red, led_blue);
    let mut cmd_processor = CommandProcessor::new(led_controller);
    
    // 发送欢迎消息
    uart_println!(uart_writer, "\r\nSTM32F407 Command Interface");
    uart_println!(uart_writer, "Type 'help' for available commands");
    uart_print!(uart_writer, "> ");
    
    // 主循环
    loop {
        match uart_writer.uart.read() {
            Ok(byte) => {
                cmd_processor.process_char(&mut uart_writer, byte);
            }
            Err(nb::Error::WouldBlock) => {
                // 没有数据，继续循环
            }
            Err(_) => {
                // 处理其他错误
            }
        }
    }
}
```

### 4. 数据记录器

实现一个简单的数据记录器，定时采集ADC数据并通过串口输出。

```rust
// src/data_logger.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{config::Config, Serial},
    adc::{Adc, config::AdcConfig},
    timer::{Timer, Event},
};
use embedded_hal::serial::Write;
use nb::block;
use core::fmt::Write;
use basic_serial::{UartWriter, uart_println};

struct DataLogger<UART, ADC, TIMER> {
    uart: UartWriter<UART>,
    adc: ADC,
    timer: TIMER,
    sample_count: u32,
}

impl<UART, ADC, TIMER> DataLogger<UART, ADC, TIMER>
where
    UART: Write<u8>,
    ADC: embedded_hal::adc::OneShot<ADC, u16, stm32f4xx_hal::gpio::gpioa::PA0<stm32f4xx_hal::gpio::Analog>>,
    TIMER: embedded_hal::timer::CountDown,
{
    fn new(uart: UartWriter<UART>, adc: ADC, timer: TIMER) -> Self {
        Self {
            uart,
            adc,
            timer,
            sample_count: 0,
        }
    }
    
    fn log_data(&mut self, pin: &mut stm32f4xx_hal::gpio::gpioa::PA0<stm32f4xx_hal::gpio::Analog>) {
        match self.adc.read(pin) {
            Ok(value) => {
                self.sample_count += 1;
                
                // 转换为电压值 (假设3.3V参考电压)
                let voltage = (value as f32 * 3.3) / 4095.0;
                
                uart_println!(
                    self.uart,
                    "Sample {}: ADC={}, Voltage={:.3}V",
                    self.sample_count,
                    value,
                    voltage
                );
            }
            Err(_) => {
                uart_println!(self.uart, "ADC read error");
            }
        }
    }
    
    fn run(&mut self, pin: &mut stm32f4xx_hal::gpio::gpioa::PA0<stm32f4xx_hal::gpio::Analog>) -> ! {
        uart_println!(self.uart, "Data Logger Started");
        uart_println!(self.uart, "Sampling every 1 second...");
        
        loop {
            // 等待定时器超时
            block!(self.timer.wait()).unwrap();
            
            // 记录数据
            self.log_data(pin);
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    
    // 配置串口
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    let config = Config::default().baudrate(115200.bps());
    let uart = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    let uart_writer = UartWriter::new(uart);
    
    // 配置ADC
    let mut adc_pin = gpioa.pa0.into_analog();
    let adc_config = AdcConfig::default();
    let adc = Adc::adc1(dp.ADC1, true, adc_config);
    
    // 配置定时器 (1秒间隔)
    let mut timer = Timer::tim2(dp.TIM2, &clocks).start_count_down(1.hz());
    timer.listen(Event::TimeOut);
    
    // 创建数据记录器并运行
    let mut data_logger = DataLogger::new(uart_writer, adc, timer);
    data_logger.run(&mut adc_pin);
}
```

## 错误处理和调试

### 1. 错误类型定义

```rust
#[derive(Debug)]
pub enum SerialError {
    HardwareError,
    BufferOverflow,
    InvalidData,
    Timeout,
}

impl From<stm32f4xx_hal::serial::Error> for SerialError {
    fn from(e: stm32f4xx_hal::serial::Error) -> Self {
        match e {
            stm32f4xx_hal::serial::Error::Overrun => SerialError::BufferOverflow,
            stm32f4xx_hal::serial::Error::Framing => SerialError::InvalidData,
            stm32f4xx_hal::serial::Error::Parity => SerialError::InvalidData,
            _ => SerialError::HardwareError,
        }
    }
}
```

### 2. 调试辅助函数

```rust
pub fn debug_uart_status<UART>(uart: &UART) 
where
    UART: embedded_hal::serial::Read<u8> + embedded_hal::serial::Write<u8>,
{
    // 这里可以添加状态检查逻辑
    // 实际实现取决于具体的HAL接口
}

// 十六进制转储函数
pub fn hex_dump<UART>(uart: &mut UartWriter<UART>, data: &[u8], label: &str)
where
    UART: Write<u8>,
{
    uart_println!(uart, "{} ({} bytes):", label, data.len());
    
    for (i, chunk) in data.chunks(16).enumerate() {
        uart_print!(uart, "{:04X}: ", i * 16);
        
        // 十六进制部分
        for (j, &byte) in chunk.iter().enumerate() {
            uart_print!(uart, "{:02X} ", byte);
            if j == 7 {
                uart_print!(uart, " ");
            }
        }
        
        // 填充空格
        for _ in chunk.len()..16 {
            uart_print!(uart, "   ");
        }
        
        uart_print!(uart, " |");
        
        // ASCII部分
        for &byte in chunk {
            let ch = if byte >= 0x20 && byte <= 0x7E {
                byte as char
            } else {
                '.'
            };
            uart_print!(uart, "{}", ch);
        }
        
        uart_println!(uart, "|");
    }
}
```

### 3. 性能监控

```rust
use cortex_m::peripheral::DWT;

pub struct PerformanceMonitor {
    dwt: DWT,
    last_cycle_count: u32,
}

impl PerformanceMonitor {
    pub fn new(mut dwt: DWT) -> Self {
        dwt.enable_cycle_counter();
        Self {
            dwt,
            last_cycle_count: 0,
        }
    }
    
    pub fn start_measurement(&mut self) {
        self.last_cycle_count = self.dwt.cyccnt.read();
    }
    
    pub fn end_measurement(&mut self) -> u32 {
        let current = self.dwt.cyccnt.read();
        current.wrapping_sub(self.last_cycle_count)
    }
    
    pub fn cycles_to_us(&self, cycles: u32, cpu_freq_hz: u32) -> u32 {
        cycles / (cpu_freq_hz / 1_000_000)
    }
}
```

## 测试和验证

### 1. 单元测试框架

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::serial::{Mock as SerialMock, Transaction};
    
    #[test]
    fn test_uart_writer() {
        let expectations = [
            Transaction::write(b'H'),
            Transaction::write(b'e'),
            Transaction::write(b'l'),
            Transaction::write(b'l'),
            Transaction::write(b'o'),
        ];
        
        let mock = SerialMock::new(&expectations);
        let mut writer = UartWriter::new(mock);
        
        write!(writer, "Hello").unwrap();
        writer.uart.done();
    }
}
```

### 2. 集成测试

```rust
// tests/integration_test.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // 运行各种测试
    test_echo_functionality();
    test_command_parsing();
    test_data_logging();
    
    loop {}
}

fn test_echo_functionality() {
    // 实现回显功能测试
}

fn test_command_parsing() {
    // 实现命令解析测试
}

fn test_data_logging() {
    // 实现数据记录测试
}
```

## 部署和使用

### 1. 编译和烧录

```bash
# 编译回显服务器
cargo build --bin echo_server --release

# 烧录到开发板
cargo run --bin echo_server --release

# 或使用probe-rs
probe-rs run --chip STM32F407VG{"file_path": "/Users/tyone/github/embedded-development/05-serial-communication/04-basic-serial.md", "explanation": "创建基础串口应用文档", "content":