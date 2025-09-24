# 中断驱动串口通信

## 概述

中断驱动的串口通信是嵌入式系统中的重要技术，它允许系统在数据到达时立即响应，而不需要持续轮询。本文档详细介绍如何在STM32平台上使用Rust实现高效的中断驱动串口通信系统。

## 中断驱动原理

### 1. 工作机制

```
┌─────────────┐    中断信号    ┌─────────────┐
│   UART硬件   │ ──────────→   │   NVIC      │
│             │               │             │
│ • 接收FIFO  │               │ • 中断控制   │
│ • 发送FIFO  │               │ • 优先级管理 │
│ • 状态寄存器 │               │             │
└─────────────┘               └─────────────┘
                                     │
                                     ▼
                              ┌─────────────┐
                              │ 中断服务程序  │
                              │             │
                              │ • 数据处理   │
                              │ • 缓冲区管理 │
                              │ • 状态更新   │
                              └─────────────┘
```

### 2. 中断类型

- **RXNE (Receive Not Empty)**：接收缓冲区非空
- **TXE (Transmit Empty)**：发送缓冲区空
- **TC (Transmission Complete)**：传输完成
- **IDLE**：空闲线检测
- **ORE (Overrun Error)**：溢出错误
- **FE (Framing Error)**：帧错误
- **PE (Parity Error)**：奇偶校验错误

## 基础实现

### 1. 项目配置

```toml
# Cargo.toml
[package]
name = "interrupt-driven-serial"
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
cortex-m-rtic = "1.1"

[dependencies.cortex-m-semihosting]
version = "0.5"
optional = true

[[bin]]
name = "basic_interrupt"
path = "src/basic_interrupt.rs"

[[bin]]
name = "buffered_uart"
path = "src/buffered_uart.rs"

[[bin]]
name = "rtic_uart"
path = "src/rtic_uart.rs"
```

### 2. 基础中断处理

```rust
// src/basic_interrupt.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac::{self, interrupt, USART1},
    prelude::*,
    serial::{config::Config, Serial, Event},
};
use embedded_hal::serial::{Read, Write};
use cortex_m::interrupt::{free, Mutex};
use core::cell::RefCell;
use heapless::spsc::{Consumer, Producer, Queue};

// 全局变量定义
type UartType = Serial<USART1, (
    stm32f4xx_hal::gpio::gpioa::PA9<stm32f4xx_hal::gpio::Alternate<7>>,
    stm32f4xx_hal::gpio::gpioa::PA10<stm32f4xx_hal::gpio::Alternate<7>>,
)>;

static G_UART: Mutex<RefCell<Option<UartType>>> = Mutex::new(RefCell::new(None));

// 接收和发送队列
static mut RX_QUEUE: Queue<u8, 256> = Queue::new();
static mut TX_QUEUE: Queue<u8, 256> = Queue::new();

static mut RX_PRODUCER: Option<Producer<u8, 256>> = None;
static mut RX_CONSUMER: Option<Consumer<u8, 256>> = None;
static mut TX_PRODUCER: Option<Producer<u8, 256>> = None;
static mut TX_CONSUMER: Option<Consumer<u8, 256>> = None;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    
    // 配置串口
    let config = Config::default().baudrate(115200.bps());
    let mut uart = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    
    // 启用中断
    uart.listen(Event::Rxne);  // 接收中断
    uart.listen(Event::Idle);  // 空闲中断
    
    // 初始化队列
    let (rx_prod, rx_cons) = unsafe { RX_QUEUE.split() };
    let (tx_prod, tx_cons) = unsafe { TX_QUEUE.split() };
    
    unsafe {
        RX_PRODUCER = Some(rx_prod);
        RX_CONSUMER = Some(rx_cons);
        TX_PRODUCER = Some(tx_prod);
        TX_CONSUMER = Some(tx_cons);
    }
    
    // 将UART移动到全局变量
    free(|cs| {
        G_UART.borrow(cs).replace(Some(uart));
    });
    
    // 配置NVIC
    let mut nvic = cp.NVIC;
    unsafe {
        nvic.set_priority(interrupt::USART1, 1);
        pac::NVIC::unmask(interrupt::USART1);
    }
    
    // 发送启动消息
    send_string("Interrupt-driven UART initialized\r\n");
    send_string("Type something and press Enter\r\n");
    
    // 主循环
    loop {
        // 处理接收到的数据
        if let Some(byte) = receive_byte() {
            // 回显字符
            send_byte(byte);
            
            // 如果是回车，发送换行
            if byte == b'\r' {
                send_byte(b'\n');
            }
        }
        
        // 其他主循环任务
        cortex_m::asm::wfi(); // 等待中断
    }
}

// 中断服务程序
#[interrupt]
fn USART1() {
    free(|cs| {
        if let Some(ref mut uart) = G_UART.borrow(cs).borrow_mut().as_mut() {
            // 处理接收中断
            if uart.is_rxne() {
                match uart.read() {
                    Ok(byte) => {
                        // 将数据放入接收队列
                        if let Some(ref mut producer) = unsafe { &mut RX_PRODUCER } {
                            producer.enqueue(byte).ok();
                        }
                    }
                    Err(_) => {
                        // 处理接收错误
                    }
                }
            }
            
            // 处理发送中断
            if uart.is_txe() {
                if let Some(ref mut consumer) = unsafe { &mut TX_CONSUMER } {
                    if let Some(byte) = consumer.dequeue() {
                        uart.write(byte).ok();
                    } else {
                        // 没有更多数据要发送，禁用发送中断
                        uart.unlisten(Event::Txe);
                    }
                }
            }
            
            // 处理空闲中断
            if uart.is_idle() {
                uart.clear_idle_interrupt();
                // 可以在这里处理完整的数据包
            }
            
            // 处理错误
            if uart.is_overrun() {
                uart.clear_overrun_error();
                // 记录溢出错误
            }
        }
    });
}

// 辅助函数
fn send_byte(byte: u8) {
    if let Some(ref mut producer) = unsafe { &mut TX_PRODUCER } {
        // 将数据放入发送队列
        while producer.enqueue(byte).is_err() {
            // 队列满，等待
            cortex_m::asm::nop();
        }
        
        // 启用发送中断
        free(|cs| {
            if let Some(ref mut uart) = G_UART.borrow(cs).borrow_mut().as_mut() {
                uart.listen(Event::Txe);
            }
        });
    }
}

fn send_string(s: &str) {
    for byte in s.bytes() {
        send_byte(byte);
    }
}

fn receive_byte() -> Option<u8> {
    if let Some(ref mut consumer) = unsafe { &mut RX_CONSUMER } {
        consumer.dequeue()
    } else {
        None
    }
}
```

### 3. 缓冲区管理

```rust
// src/buffered_uart.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use heapless::{spsc::Queue, String, Vec};
use core::sync::atomic::{AtomicBool, Ordering};

// 缓冲区大小配置
const RX_BUFFER_SIZE: usize = 512;
const TX_BUFFER_SIZE: usize = 512;
const LINE_BUFFER_SIZE: usize = 128;

// 缓冲区状态
static RX_OVERFLOW: AtomicBool = AtomicBool::new(false);
static TX_OVERFLOW: AtomicBool = AtomicBool::new(false);

// 高级缓冲区管理器
pub struct BufferedUart {
    rx_queue: Queue<u8, RX_BUFFER_SIZE>,
    tx_queue: Queue<u8, TX_BUFFER_SIZE>,
    line_buffer: String<LINE_BUFFER_SIZE>,
    line_ready: bool,
}

impl BufferedUart {
    pub fn new() -> Self {
        Self {
            rx_queue: Queue::new(),
            tx_queue: Queue::new(),
            line_buffer: String::new(),
            line_ready: false,
        }
    }
    
    // 从中断中调用
    pub fn handle_rx_interrupt(&mut self, byte: u8) {
        if self.rx_queue.enqueue(byte).is_err() {
            RX_OVERFLOW.store(true, Ordering::Relaxed);
        }
    }
    
    // 从中断中调用
    pub fn handle_tx_interrupt(&mut self) -> Option<u8> {
        self.tx_queue.dequeue()
    }
    
    // 主循环中调用
    pub fn process_received_data(&mut self) {
        while let Some(byte) = self.rx_queue.dequeue() {
            match byte {
                b'\r' | b'\n' => {
                    if !self.line_buffer.is_empty() {
                        self.line_ready = true;
                    }
                }
                b'\x08' | b'\x7f' => { // 退格
                    self.line_buffer.pop();
                }
                0x20..=0x7E => { // 可打印字符
                    if self.line_buffer.len() < self.line_buffer.capacity() {
                        self.line_buffer.push(byte as char).ok();
                    }
                }
                _ => {
                    // 忽略其他字符
                }
            }
        }
    }
    
    // 检查是否有完整的行
    pub fn get_line(&mut self) -> Option<String<LINE_BUFFER_SIZE>> {
        if self.line_ready {
            self.line_ready = false;
            let line = self.line_buffer.clone();
            self.line_buffer.clear();
            Some(line)
        } else {
            None
        }
    }
    
    // 发送数据
    pub fn send_bytes(&mut self, data: &[u8]) -> Result<(), ()> {
        for &byte in data {
            if self.tx_queue.enqueue(byte).is_err() {
                TX_OVERFLOW.store(true, Ordering::Relaxed);
                return Err(());
            }
        }
        
        // 启用发送中断
        self.enable_tx_interrupt();
        Ok(())
    }
    
    pub fn send_string(&mut self, s: &str) -> Result<(), ()> {
        self.send_bytes(s.as_bytes())
    }
    
    // 格式化发送
    pub fn send_formatted(&mut self, args: core::fmt::Arguments) -> Result<(), ()> {
        let mut buffer = String::<256>::new();
        use core::fmt::Write;
        write!(buffer, "{}", args).map_err(|_| ())?;
        self.send_string(&buffer)
    }
    
    // 获取统计信息
    pub fn get_stats(&self) -> UartStats {
        UartStats {
            rx_queue_len: self.rx_queue.len(),
            tx_queue_len: self.tx_queue.len(),
            rx_overflow: RX_OVERFLOW.load(Ordering::Relaxed),
            tx_overflow: TX_OVERFLOW.load(Ordering::Relaxed),
        }
    }
    
    // 清除错误状态
    pub fn clear_errors(&mut self) {
        RX_OVERFLOW.store(false, Ordering::Relaxed);
        TX_OVERFLOW.store(false, Ordering::Relaxed);
    }
    
    fn enable_tx_interrupt(&self) {
        // 实际实现中需要访问UART外设
        // 这里是占位符
    }
}

#[derive(Debug)]
pub struct UartStats {
    pub rx_queue_len: usize,
    pub tx_queue_len: usize,
    pub rx_overflow: bool,
    pub tx_overflow: bool,
}

// 宏定义，简化格式化输出
#[macro_export]
macro_rules! uart_write {
    ($uart:expr, $($arg:tt)*) => {
        $uart.send_formatted(format_args!($($arg)*))
    };
}

#[entry]
fn main() -> ! {
    // 初始化代码...
    
    let mut buffered_uart = BufferedUart::new();
    
    loop {
        // 处理接收数据
        buffered_uart.process_received_data();
        
        // 检查是否有完整的命令行
        if let Some(line) = buffered_uart.get_line() {
            // 处理命令
            process_command(&mut buffered_uart, &line);
        }
        
        // 检查错误状态
        let stats = buffered_uart.get_stats();
        if stats.rx_overflow || stats.tx_overflow {
            uart_write!(buffered_uart, "Buffer overflow detected!\r\n").ok();
            buffered_uart.clear_errors();
        }
        
        cortex_m::asm::wfi();
    }
}

fn process_command(uart: &mut BufferedUart, command: &str) {
    let parts: Vec<&str, 8> = command.trim().split_whitespace().collect();
    
    if parts.is_empty() {
        return;
    }
    
    match parts[0] {
        "stats" => {
            let stats = uart.get_stats();
            uart_write!(uart, "UART Statistics:\r\n").ok();
            uart_write!(uart, "  RX Queue: {}/{}\r\n", stats.rx_queue_len, RX_BUFFER_SIZE).ok();
            uart_write!(uart, "  TX Queue: {}/{}\r\n", stats.tx_queue_len, TX_BUFFER_SIZE).ok();
            uart_write!(uart, "  Overflows: RX={}, TX={}\r\n", stats.rx_overflow, stats.tx_overflow).ok();
        }
        "echo" => {
            if parts.len() > 1 {
                let message = &command[5..]; // Skip "echo "
                uart_write!(uart, "Echo: {}\r\n", message).ok();
            }
        }
        "help" => {
            uart.send_string("Available commands:\r\n").ok();
            uart.send_string("  stats - Show UART statistics\r\n").ok();
            uart.send_string("  echo <msg> - Echo message\r\n").ok();
            uart.send_string("  help - Show this help\r\n").ok();
        }
        _ => {
            uart_write!(uart, "Unknown command: {}\r\n", parts[0]).ok();
        }
    }
}
```

## 使用RTIC框架

### 1. RTIC配置

```rust
// src/rtic_uart.rs
#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{config::Config, Serial, Event},
    gpio::{Output, PushPull, gpiod::PD12},
};
use heapless::{spsc::Queue, String};

#[app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use super::*;
    
    // 共享资源
    #[shared]
    struct Shared {
        uart: Serial<pac::USART1, (
            stm32f4xx_hal::gpio::gpioa::PA9<stm32f4xx_hal::gpio::Alternate<7>>,
            stm32f4xx_hal::gpio::gpioa::PA10<stm32f4xx_hal::gpio::Alternate<7>>,
        )>,
        led: PD12<Output<PushPull>>,
        command_buffer: String<128>,
    }
    
    // 本地资源
    #[local]
    struct Local {
        rx_queue: Queue<u8, 256>,
        tx_queue: Queue<u8, 256>,
    }
    
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        
        // 配置时钟
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();
        
        // 配置GPIO
        let gpioa = dp.GPIOA.split();
        let gpiod = dp.GPIOD.split();
        
        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let led = gpiod.pd12.into_push_pull_output();
        
        // 配置串口
        let config = Config::default().baudrate(115200.bps());
        let mut uart = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
        
        // 启用中断
        uart.listen(Event::Rxne);
        uart.listen(Event::Idle);
        
        (
            Shared {
                uart,
                led,
                command_buffer: String::new(),
            },
            Local {
                rx_queue: Queue::new(),
                tx_queue: Queue::new(),
            },
            init::Monotonics(),
        )
    }
    
    // 空闲任务
    #[idle(shared = [led], local = [rx_queue])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            // 处理接收队列中的数据
            if let Some(byte) = ctx.local.rx_queue.dequeue() {
                // 处理接收到的字节
                process_received_byte::spawn(byte).ok();
            }
            
            // LED闪烁表示系统运行
            ctx.shared.led.lock(|led| {
                led.toggle();
            });
            
            cortex_m::asm::delay(8_000_000); // 简单延时
        }
    }
    
    // UART中断处理
    #[task(binds = USART1, shared = [uart], local = [rx_queue, tx_queue], priority = 2)]
    fn usart1_interrupt(mut ctx: usart1_interrupt::Context) {
        ctx.shared.uart.lock(|uart| {
            // 处理接收中断
            if uart.is_rxne() {
                match uart.read() {
                    Ok(byte) => {
                        // 将数据放入本地队列
                        ctx.local.rx_queue.enqueue(byte).ok();
                    }
                    Err(_) => {
                        // 处理错误
                    }
                }
            }
            
            // 处理发送中断
            if uart.is_txe() {
                if let Some(byte) = ctx.local.tx_queue.dequeue() {
                    uart.write(byte).ok();
                } else {
                    // 没有更多数据，禁用发送中断
                    uart.unlisten(Event::Txe);
                }
            }
            
            // 处理空闲中断
            if uart.is_idle() {
                uart.clear_idle_interrupt();
                // 触发命令处理任务
                process_command_line::spawn().ok();
            }
        });
    }
    
    // 处理接收字节的任务
    #[task(shared = [command_buffer], priority = 1)]
    fn process_received_byte(mut ctx: process_received_byte::Context, byte: u8) {
        ctx.shared.command_buffer.lock(|buffer| {
            match byte {
                b'\r' | b'\n' => {
                    // 命令结束，触发处理
                    if !buffer.is_empty() {
                        process_command_line::spawn().ok();
                    }
                }
                b'\x08' | b'\x7f' => {
                    // 退格
                    buffer.pop();
                }
                0x20..=0x7E => {
                    // 可打印字符
                    if buffer.len() < buffer.capacity() {
                        buffer.push(byte as char).ok();
                    }
                }
                _ => {
                    // 忽略其他字符
                }
            }
        });
        
        // 回显字符
        send_byte::spawn(byte).ok();
    }
    
    // 处理命令行的任务
    #[task(shared = [command_buffer], priority = 1)]
    fn process_command_line(mut ctx: process_command_line::Context) {
        let command = ctx.shared.command_buffer.lock(|buffer| {
            let cmd = buffer.clone();
            buffer.clear();
            cmd
        });
        
        if !command.is_empty() {
            execute_command::spawn(command).ok();
        }
    }
    
    // 执行命令的任务
    #[task(shared = [led], priority = 1)]
    fn execute_command(mut ctx: execute_command::Context, command: String<128>) {
        let parts: heapless::Vec<&str, 8> = command.trim().split_whitespace().collect();
        
        if parts.is_empty() {
            return;
        }
        
        match parts[0] {
            "led" => {
                if parts.len() > 1 {
                    match parts[1] {
                        "on" => {
                            ctx.shared.led.lock(|led| led.set_high());
                            send_string::spawn("LED ON\r\n").ok();
                        }
                        "off" => {
                            ctx.shared.led.lock(|led| led.set_low());
                            send_string::spawn("LED OFF\r\n").ok();
                        }
                        "toggle" => {
                            ctx.shared.led.lock(|led| led.toggle());
                            send_string::spawn("LED TOGGLED\r\n").ok();
                        }
                        _ => {
                            send_string::spawn("Usage: led on|off|toggle\r\n").ok();
                        }
                    }
                }
            }
            "help" => {
                send_string::spawn("Available commands:\r\n").ok();
                send_string::spawn("  led on|off|toggle - Control LED\r\n").ok();
                send_string::spawn("  help - Show this help\r\n").ok();
            }
            _ => {
                send_string::spawn("Unknown command\r\n").ok();
            }
        }
    }
    
    // 发送单个字节的任务
    #[task(shared = [uart], local = [tx_queue], priority = 1)]
    fn send_byte(mut ctx: send_byte::Context, byte: u8) {
        // 将字节放入发送队列
        ctx.local.tx_queue.enqueue(byte).ok();
        
        // 启用发送中断
        ctx.shared.uart.lock(|uart| {
            uart.listen(Event::Txe);
        });
    }
    
    // 发送字符串的任务
    #[task(priority = 1)]
    fn send_string(ctx: send_string::Context, s: &'static str) {
        for byte in s.bytes() {
            send_byte::spawn(byte).ok();
        }
    }
}
```

## 高级特性

### 1. 流控制

```rust
// 硬件流控制实现
pub struct FlowControlledUart {
    uart: UartType,
    rts_pin: stm32f4xx_hal::gpio::gpioa::PA1<Output<PushPull>>,
    cts_pin: stm32f4xx_hal::gpio::gpioa::PA0<Input<PullUp>>,
    flow_control_enabled: bool,
}

impl FlowControlledUart {
    pub fn new(
        uart: UartType,
        rts_pin: stm32f4xx_hal::gpio::gpioa::PA1<Output<PushPull>>,
        cts_pin: stm32f4xx_hal::gpio::gpioa::PA0<Input<PullUp>>,
    ) -> Self {
        let mut instance = Self {
            uart,
            rts_pin,
            cts_pin,
            flow_control_enabled: true,
        };
        
        // 初始状态：准备接收
        instance.rts_pin.set_low();
        instance
    }
    
    pub fn can_send(&self) -> bool {
        if self.flow_control_enabled {
            self.cts_pin.is_low()
        } else {
            true
        }
    }
    
    pub fn set_ready_to_receive(&mut self, ready: bool) {
        if self.flow_control_enabled {
            if ready {
                self.rts_pin.set_low(); // 准备接收
            } else {
                self.rts_pin.set_high(); // 暂停发送
            }
        }
    }
    
    pub fn send_with_flow_control(&mut self, data: &[u8]) -> Result<usize, ()> {
        let mut sent = 0;
        
        for &byte in data {
            // 检查CTS信号
            if !self.can_send() {
                break;
            }
            
            // 发送字节
            match self.uart.write(byte) {
                Ok(()) => sent += 1,
                Err(_) => break,
            }
        }
        
        Ok(sent)
    }
}
```

### 2. 协议解析

```rust
// 协议解析器
#[derive(Debug, PartialEq)]
pub enum ProtocolState {
    WaitingForHeader,
    WaitingForLength,
    WaitingForData,
    WaitingForChecksum,
}

pub struct ProtocolParser {
    state: ProtocolState,
    expected_length: usize,
    received_data: heapless::Vec<u8, 256>,
    calculated_checksum: u8,
}

impl ProtocolParser {
    pub fn new() -> Self {
        Self {
            state: ProtocolState::WaitingForHeader,
            expected_length: 0,
            received_data: heapless::Vec::new(),
            calculated_checksum: 0,
        }
    }
    
    pub fn process_byte(&mut self, byte: u8) -> Option<heapless::Vec<u8, 256>> {
        match self.state {
            ProtocolState::WaitingForHeader => {
                if byte == 0xAA {
                    self.state = ProtocolState::WaitingForLength;
                    self.calculated_checksum = byte;
                }
            }
            ProtocolState::WaitingForLength => {
                self.expected_length = byte as usize;
                self.state = ProtocolState::WaitingForData;
                self.calculated_checksum = self.calculated_checksum.wrapping_add(byte);
                self.received_data.clear();
            }
            ProtocolState::WaitingForData => {
                self.received_data.push(byte).ok();
                self.calculated_checksum = self.calculated_checksum.wrapping_add(byte);
                
                if self.received_data.len() >= self.expected_length {
                    self.state = ProtocolState::WaitingForChecksum;
                }
            }
            ProtocolState::WaitingForChecksum => {
                if byte == self.calculated_checksum {
                    // 校验成功，返回数据
                    let data = self.received_data.clone();
                    self.reset();
                    return Some(data);
                } else {
                    // 校验失败，重置状态
                    self.reset();
                }
            }
        }
        
        None
    }
    
    fn reset(&mut self) {
        self.state = ProtocolState::WaitingForHeader;
        self.expected_length = 0;
        self.received_data.clear();
        self.calculated_checksum = 0;
    }
    
    pub fn encode_packet(data: &[u8]) -> heapless::Vec<u8, 260> {
        let mut packet = heapless::Vec::new();
        
        // 帧头
        packet.push(0xAA).ok();
        
        // 长度
        packet.push(data.len() as u8).ok();
        
        // 数据
        for &byte in data {
            packet.push(byte).ok();
        }
        
        // 校验和
        let checksum = packet.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
        packet.push(checksum).ok();
        
        packet
    }
}

// 使用示例
fn protocol_example() {
    let mut parser = ProtocolParser::new();
    
    // 模拟接收数据
    let received_bytes = [0xAA, 0x04, 0x01, 0x02, 0x03, 0x04, 0xB4];
    
    for byte in received_bytes {
        if let Some(data) = parser.process_byte(byte) {
            // 处理完整的数据包
            process_packet(&data);
        }
    }
}

fn process_packet(data: &[u8]) {
    // 处理接收到的数据包
}
```

### 3. 性能监控

```rust
// 性能监控器
pub struct UartPerformanceMonitor {
    bytes_sent: u32,
    bytes_received: u32,
    errors_count: u32,
    start_time: u32,
    last_activity: u32,
}

impl UartPerformanceMonitor {
    pub fn new() -> Self {
        Self {
            bytes_sent: 0,
            bytes_received: 0,
            errors_count: 0,
            start_time: get_system_time(),
            last_activity: get_system_time(),
        }
    }
    
    pub fn record_byte_sent(&mut self) {
        self.bytes_sent += 1;
        self.last_activity = get_system_time();
    }
    
    pub fn record_byte_received(&mut self) {
        self.bytes_received += 1;
        self.last_activity = get_system_time();
    }
    
    pub fn record_error(&mut self) {
        self.errors_count += 1;
    }
    
    pub fn get_throughput_bps(&self) -> (u32, u32) {
        let elapsed = get_system_time() - self.start_time;
        if elapsed > 0 {
            let tx_bps = (self.bytes_sent * 8 * 1000) / elapsed;
            let rx_bps = (self.bytes_received * 8 * 1000) / elapsed;
            (tx_bps, rx_bps)
        } else {
            (0, 0)
        }
    }
    
    pub fn get_error_rate(&self) -> f32 {
        let total_bytes = self.bytes_sent + self.bytes_received;
        if total_bytes > 0 {
            (self.errors_count as f32) / (total_bytes as f32)
        } else {
            0.0
        }
    }
    
    pub fn is_idle(&self, timeout_ms: u32) -> bool {
        (get_system_time() - self.last_activity) > timeout_ms
    }
    
    pub fn reset(&mut self) {
        self.bytes_sent = 0;
        self.bytes_received = 0;
        self.errors_count = 0;
        self.start_time = get_system_time();
        self.last_activity = get_system_time();
    }
}

fn get_system_time() -> u32 {
    // 实际实现中应该返回系统时间（毫秒）
    0 // 占位符
}
```

## 错误处理和恢复

### 1. 错误检测

```rust
#[derive(Debug, Clone, Copy)]
pub enum UartError {
    Overrun,
    Framing,
    Parity,
    Noise,
    BufferOverflow,
    Timeout,
}

pub struct ErrorHandler {
    error_counts: [u32; 6],
    last_error_time: u32,
    recovery_attempts: u32,
}

impl ErrorHandler {
    pub fn new() -> Self {
        Self {
            error_counts: [0; 6],
            last_error_time: 0,
            recovery_attempts: 0,
        }
    }
    
    pub fn handle_error(&mut self, error: UartError) -> bool {
        let error_index = error as usize;
        self.error_counts[error_index] += 1;
        self.last_error_time = get_system_time();
        
        match error {
            UartError::Overrun => {
                // 清除溢出错误，重置接收器
                self.clear_overrun_error();
                true
            }
            UartError::Framing => {
                // 帧错误可能是波特率问题
                if self.error_counts[error_index] > 10 {
                    self.attempt_baud_rate_recovery();
                }
                true
            }
            UartError::BufferOverflow => {
                // 缓冲区溢出，清空缓冲区
                self.clear_buffers();
                true
            }
            UartError::Timeout => {
                // 超时错误，重置通信
                self.reset_communication();
                true
            }
            _ => false,
        }
    }
    
    fn clear_overrun_error(&mut self) {
        // 实现溢出错误清除
    }
    
    fn attempt_baud_rate_recovery(&mut self) {
        // 尝试自动波特率检测
        self.recovery_attempts += 1;
    }
    
    fn clear_buffers(&mut self) {
        // 清空所有缓冲区
    }
    
    fn reset_communication(&mut self) {
        // 重置通信状态
    }
    
    pub fn get_error_statistics(&self) -> ErrorStatistics {
        ErrorStatistics {
            overrun_count: self.error_counts[0],
            framing_count: self.error_counts[1],
            parity_count: self.error_counts[2],
            noise_count: self.error_counts[3],
            buffer_overflow_count: self.error_counts[4],
            timeout_count: self.error_counts[5],
            recovery_attempts: self.recovery_attempts,
        }
    }
}

#[derive(Debug)]
pub struct ErrorStatistics {
    pub overrun_count: u32,
    pub framing_count: u32,
    pub parity_count: u32,
    pub noise_count: u32,
    pub buffer_overflow_count: u32,
    pub timeout_count: u32,
    pub recovery_attempts: u32,
}
```

## 测试和调试

### 1. 单元测试

```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_protocol_parser() {
        let mut parser = ProtocolParser::new();
        
        // 测试正确的数据包
        let packet = [0xAA, 0x03, 0x01, 0x02, 0x03, 0xB3];
        let mut result = None;
        
        for byte in packet {
            if let Some(data) = parser.process_byte(byte) {
                result = Some(data);
                break;
            }
        }
        
        assert!(result.is_some());
        let data = result.unwrap();
        assert_eq!(data.len(), 3);
        assert_eq!(data[0], 0x01);
        assert_eq!(data[1], 0x02);
        assert_eq!(data[2], 0x03);
    }
    
    #[test]
    fn test_protocol_parser_invalid_checksum() {
        let mut parser = ProtocolParser::new();
        
        // 测试错误的校验和
        let packet = [0xAA, 0x03, 0x01, 0x02, 0x03, 0xFF];
        
        for byte in packet {
            assert!(parser.process_byte(byte).is_none());
        }
    }
    
    #[test]
    fn test_error_handler() {
        let mut handler = ErrorHandler::new();
        
        // 测试错误处理
        assert!(handler.handle_error(UartError::Overrun));
        assert!(handler.handle_error(UartError::BufferOverflow));
        
        let stats = handler.get_error_statistics();
        assert_eq!(stats.overrun_count, 1);
        assert_eq!(stats.buffer_overflow_count, 1);
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
    // 初始化测试环境
    test_interrupt_latency();
    test_throughput();
    test_error_recovery();
    test_protocol_parsing();
    
    loop {}
}

fn test_interrupt_latency() {
    // 测试中断响应延迟
}

fn test_throughput() {
    // 测试数据吞吐量
}

fn test_error_recovery() {
    // 测试错误恢复机制
}

fn test_protocol_parsing() {
    // 测试协议解析
}
```

## 性能优化建议

### 1. 中断优化

- **中断优先级设置**：合理设置中断优先级，避免高频中断被低优先级中断阻塞
- **中断处理时间**：保持中断服务程序简短，将复杂处理移到主循环
- **批量处理**：在可能的情况下批量处理多个字节

### 2. 缓冲区优化

- **缓冲区大小**：根据应用需求选择合适的缓冲区大小
- **无锁队列**：使用无锁数据结构提高并发性能
- **内存对齐**：确保缓冲区内存对齐以提高访问效率

### 3. 功耗优化

- **动态中断控制**：根据需要动态启用/禁用中断
- **睡眠模式**：在空闲时进入低功耗模式
- **时钟门控**：不使用时关闭UART时钟

## 总结

中断驱动的串口通信是嵌入式系统中的核心技术。通过合理的设计和实现，可以实现高效、可靠的数据通信。关键要点包括：

1. **正确的中断配置**：选择合适的中断类型和优先级
2. **高效的缓冲区管理**：使用无锁队列和合适的缓冲区大小
3. **完善的错误处理**：检测和恢复各种通信错误
4. **性能监控**：实时监控通信性能和状态
5. **协议支持**：实现可靠的数据包协议

这些技术为构建复杂的串口通信系统提供了坚实的基础。

## 参考资料

- [STM32F4xx Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [RTIC Book](https://rtic.rs/)
- [Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [heapless Documentation](https://docs.rs/heapless/)