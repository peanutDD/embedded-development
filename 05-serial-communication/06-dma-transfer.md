# DMA传输

## 概述

DMA（Direct Memory Access，直接内存访问）是一种高效的数据传输技术，允许外设直接与内存进行数据交换，无需CPU干预。在串口通信中使用DMA可以显著提高数据传输效率，降低CPU负载，特别适合高速、大量数据传输的场景。

## DMA工作原理

### 1. DMA基本概念

```
┌─────────────┐    DMA请求    ┌─────────────┐
│   UART外设   │ ──────────→   │ DMA控制器    │
│             │               │             │
│ • TX FIFO   │               │ • 通道管理   │
│ • RX FIFO   │               │ • 传输控制   │
│ • DMA使能   │               │ • 中断生成   │
└─────────────┘               └─────────────┘
                                     │
                                     ▼
                              ┌─────────────┐
                              │   系统内存   │
                              │             │
                              │ • 发送缓冲区 │
                              │ • 接收缓冲区 │
                              │ • 数据处理   │
                              └─────────────┘
```

### 2. DMA传输模式

- **单次传输**：传输完成后停止
- **循环传输**：传输完成后自动重新开始
- **双缓冲传输**：使用两个缓冲区交替传输

### 3. STM32 DMA特性

- **多通道支持**：每个DMA控制器支持多个通道
- **优先级控制**：可配置通道优先级
- **传输宽度**：支持8位、16位、32位传输
- **地址递增**：支持源地址和目标地址递增
- **中断支持**：传输完成、半传输完成、错误中断

## 基础实现

### 1. 项目配置

```toml
# Cargo.toml
[package]
name = "dma-serial"
version = "0.1.0"
edition = "2021"

[dependencies]
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"
stm32f4xx-hal = { version = "0.14", features = ["stm32f407"] }
embedded-hal = "0.2"
embedded-dma = "0.2"
nb = "1.0"
heapless = "0.7"

[[bin]]
name = "basic_dma"
path = "src/basic_dma.rs"

[[bin]]
name = "circular_dma"
path = "src/circular_dma.rs"

[[bin]]
name = "double_buffer"
path = "src/double_buffer.rs"
```

### 2. 基础DMA传输

```rust
// src/basic_dma.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{config::Config, Serial},
    dma::{StreamsTuple, Transfer, Stream2, Stream7, Channel4},
};
use embedded_hal::serial::{Read, Write};
use heapless::Vec;

// DMA类型定义
type TxDma = Transfer<
    Stream7<pac::DMA2>,
    Channel4,
    Serial<pac::USART1, (
        stm32f4xx_hal::gpio::gpioa::PA9<stm32f4xx_hal::gpio::Alternate<7>>,
        stm32f4xx_hal::gpio::gpioa::PA10<stm32f4xx_hal::gpio::Alternate<7>>,
    )>,
    stm32f4xx_hal::dma::MemoryToPeripheral,
    &'static mut [u8],
>;

type RxDma = Transfer<
    Stream2<pac::DMA2>,
    Channel4,
    Serial<pac::USART1, (
        stm32f4xx_hal::gpio::gpioa::PA9<stm32f4xx_hal::gpio::Alternate<7>>,
        stm32f4xx_hal::gpio::gpioa::PA10<stm32f4xx_hal::gpio::Alternate<7>>,
    )>,
    stm32f4xx_hal::dma::PeripheralToMemory,
    &'static mut [u8],
>;

// 静态缓冲区
static mut TX_BUFFER: [u8; 256] = [0; 256];
static mut RX_BUFFER: [u8; 256] = [0; 256];

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    
    // 配置串口
    let config = Config::default().baudrate(115200.bps());
    let serial = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    
    // 配置DMA
    let dma = StreamsTuple::new(dp.DMA2);
    
    // 分离串口的发送和接收部分
    let (tx, rx) = serial.split();
    
    // 创建DMA传输
    let tx_stream = dma.7;
    let rx_stream = dma.2;
    
    let mut tx_transfer = Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        unsafe { &mut TX_BUFFER },
        None,
        stm32f4xx_hal::dma::config::DmaConfig::default(),
    );
    
    let mut rx_transfer = Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        unsafe { &mut RX_BUFFER },
        None,
        stm32f4xx_hal::dma::config::DmaConfig::default(),
    );
    
    // 启动接收DMA
    rx_transfer.start(|_rx| {});
    
    // 准备发送数据
    let message = b"Hello from DMA UART!\r\n";
    unsafe {
        TX_BUFFER[..message.len()].copy_from_slice(message);
    }
    
    // 启动发送DMA
    tx_transfer.start(|_tx| {});
    
    // 主循环
    loop {
        // 检查发送是否完成
        if tx_transfer.is_complete() {
            let (_stream, _tx, _buffer, _) = tx_transfer.free();
            // 可以重新配置发送
        }
        
        // 检查接收是否完成
        if rx_transfer.is_complete() {
            let (_stream, _rx, buffer, _) = rx_transfer.free();
            
            // 处理接收到的数据
            process_received_data(buffer);
            
            // 重新启动接收
            // rx_transfer = Transfer::init_peripheral_to_memory(...);
            // rx_transfer.start(|_rx| {});
        }
        
        cortex_m::asm::wfi();
    }
}

fn process_received_data(buffer: &[u8]) {
    // 处理接收到的数据
    for &byte in buffer {
        if byte != 0 {
            // 处理有效数据
        }
    }
}
```

### 3. 循环DMA传输

```rust
// src/circular_dma.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac::{self, interrupt},
    prelude::*,
    serial::{config::Config, Serial},
    dma::{StreamsTuple, Transfer, Stream2, Channel4, config::DmaConfig},
};
use cortex_m::interrupt::{free, Mutex};
use core::cell::RefCell;
use heapless::spsc::{Consumer, Producer, Queue};

// 全局变量
static mut RX_BUFFER: [u8; 512] = [0; 512];
static mut DATA_QUEUE: Queue<u8, 1024> = Queue::new();
static mut QUEUE_PRODUCER: Option<Producer<u8, 1024>> = None;
static mut QUEUE_CONSUMER: Option<Consumer<u8, 1024>> = None;

// DMA传输状态
static RX_TRANSFER: Mutex<RefCell<Option<Transfer<
    Stream2<pac::DMA2>,
    Channel4,
    Serial<pac::USART1, (
        stm32f4xx_hal::gpio::gpioa::PA9<stm32f4xx_hal::gpio::Alternate<7>>,
        stm32f4xx_hal::gpio::gpioa::PA10<stm32f4xx_hal::gpio::Alternate<7>>,
    )>,
    stm32f4xx_hal::dma::PeripheralToMemory,
    &'static mut [u8],
>>>> = Mutex::new(RefCell::new(None));

static mut LAST_POSITION: usize = 0;

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
    let serial = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    let (_tx, rx) = serial.split();
    
    // 配置DMA
    let dma = StreamsTuple::new(dp.DMA2);
    let rx_stream = dma.2;
    
    // 配置循环DMA
    let mut dma_config = DmaConfig::default();
    dma_config.circular_buffer = true;
    dma_config.half_transfer_interrupt = true;
    dma_config.transfer_complete_interrupt = true;
    
    let rx_transfer = Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        unsafe { &mut RX_BUFFER },
        None,
        dma_config,
    );
    
    // 初始化队列
    let (producer, consumer) = unsafe { DATA_QUEUE.split() };
    unsafe {
        QUEUE_PRODUCER = Some(producer);
        QUEUE_CONSUMER = Some(consumer);
    }
    
    // 将DMA传输移动到全局变量
    free(|cs| {
        RX_TRANSFER.borrow(cs).replace(Some(rx_transfer));
    });
    
    // 启动DMA传输
    free(|cs| {
        if let Some(ref mut transfer) = RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            transfer.start(|_rx| {});
        }
    });
    
    // 配置中断
    let mut nvic = cp.NVIC;
    unsafe {
        nvic.set_priority(interrupt::DMA2_STREAM2, 1);
        pac::NVIC::unmask(interrupt::DMA2_STREAM2);
    }
    
    // 主循环
    loop {
        // 处理队列中的数据
        if let Some(ref mut consumer) = unsafe { &mut QUEUE_CONSUMER } {
            while let Some(byte) = consumer.dequeue() {
                process_byte(byte);
            }
        }
        
        cortex_m::asm::wfi();
    }
}

// DMA中断处理
#[interrupt]
fn DMA2_STREAM2() {
    free(|cs| {
        if let Some(ref mut transfer) = RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            let buffer_size = unsafe { RX_BUFFER.len() };
            let current_position = buffer_size - transfer.number_of_transfers() as usize;
            
            // 处理半传输完成中断
            if transfer.is_half_complete() {
                transfer.clear_half_complete_interrupt();
                process_buffer_segment(0, buffer_size / 2, current_position);
            }
            
            // 处理传输完成中断
            if transfer.is_complete() {
                transfer.clear_transfer_complete_interrupt();
                process_buffer_segment(buffer_size / 2, buffer_size, current_position);
            }
            
            unsafe {
                LAST_POSITION = current_position;
            }
        }
    });
}

fn process_buffer_segment(start: usize, end: usize, current_pos: usize) {
    let last_pos = unsafe { LAST_POSITION };
    
    // 确定需要处理的数据范围
    let (process_start, process_end) = if current_pos >= last_pos {
        (last_pos, current_pos)
    } else {
        // 缓冲区回绕的情况
        if last_pos < end && start < current_pos {
            (last_pos, end)
        } else {
            return;
        }
    };
    
    // 将数据放入队列
    if let Some(ref mut producer) = unsafe { &mut QUEUE_PRODUCER } {
        for i in process_start..process_end {
            let byte = unsafe { RX_BUFFER[i] };
            producer.enqueue(byte).ok();
        }
    }
}

fn process_byte(byte: u8) {
    // 处理接收到的字节
    match byte {
        b'\r' | b'\n' => {
            // 行结束
        }
        0x20..=0x7E => {
            // 可打印字符
        }
        _ => {
            // 其他字符
        }
    }
}
```

### 4. 双缓冲DMA

```rust
// src/double_buffer.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac::{self, interrupt},
    prelude::*,
    serial::{config::Config, Serial},
    dma::{StreamsTuple, Transfer, Stream2, Channel4, config::DmaConfig},
};
use cortex_m::interrupt::{free, Mutex};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

// 双缓冲区
static mut BUFFER_A: [u8; 256] = [0; 256];
static mut BUFFER_B: [u8; 256] = [0; 256];

// 缓冲区状态
static BUFFER_A_READY: AtomicBool = AtomicBool::new(false);
static BUFFER_B_READY: AtomicBool = AtomicBool::new(false);
static CURRENT_BUFFER: AtomicUsize = AtomicUsize::new(0); // 0 = A, 1 = B

// DMA传输
static RX_TRANSFER: Mutex<RefCell<Option<Transfer<
    Stream2<pac::DMA2>,
    Channel4,
    Serial<pac::USART1, (
        stm32f4xx_hal::gpio::gpioa::PA9<stm32f4xx_hal::gpio::Alternate<7>>,
        stm32f4xx_hal::gpio::gpioa::PA10<stm32f4xx_hal::gpio::Alternate<7>>,
    )>,
    stm32f4xx_hal::dma::PeripheralToMemory,
    &'static mut [u8],
>>>> = Mutex::new(RefCell::new(None));

struct DoubleBufferManager {
    processing_buffer: usize,
}

impl DoubleBufferManager {
    fn new() -> Self {
        Self {
            processing_buffer: 0,
        }
    }
    
    fn get_ready_buffer(&mut self) -> Option<&'static [u8]> {
        // 检查缓冲区A
        if BUFFER_A_READY.load(Ordering::Acquire) {
            BUFFER_A_READY.store(false, Ordering::Release);
            self.processing_buffer = 0;
            return Some(unsafe { &BUFFER_A });
        }
        
        // 检查缓冲区B
        if BUFFER_B_READY.load(Ordering::Acquire) {
            BUFFER_B_READY.store(false, Ordering::Release);
            self.processing_buffer = 1;
            return Some(unsafe { &BUFFER_B });
        }
        
        None
    }
    
    fn switch_buffer(&self) {
        let current = CURRENT_BUFFER.load(Ordering::Acquire);
        let next = 1 - current;
        
        free(|cs| {
            if let Some(ref mut transfer) = RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
                // 停止当前传输
                transfer.pause(|_rx| {});
                
                // 切换缓冲区
                let new_buffer = if next == 0 {
                    unsafe { &mut BUFFER_A }
                } else {
                    unsafe { &mut BUFFER_B }
                };
                
                // 重新配置DMA
                // 注意：实际实现中需要重新创建Transfer对象
                // 这里简化处理
                
                // 重新启动传输
                transfer.start(|_rx| {});
            }
        });
        
        CURRENT_BUFFER.store(next, Ordering::Release);
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 初始化代码...
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    
    let config = Config::default().baudrate(115200.bps());
    let serial = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    let (_tx, rx) = serial.split();
    
    // 配置DMA
    let dma = StreamsTuple::new(dp.DMA2);
    let rx_stream = dma.2;
    
    let mut dma_config = DmaConfig::default();
    dma_config.transfer_complete_interrupt = true;
    
    let rx_transfer = Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        unsafe { &mut BUFFER_A },
        None,
        dma_config,
    );
    
    free(|cs| {
        RX_TRANSFER.borrow(cs).replace(Some(rx_transfer));
    });
    
    // 启动DMA
    free(|cs| {
        if let Some(ref mut transfer) = RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            transfer.start(|_rx| {});
        }
    });
    
    // 配置中断
    let mut nvic = cp.NVIC;
    unsafe {
        nvic.set_priority(interrupt::DMA2_STREAM2, 1);
        pac::NVIC::unmask(interrupt::DMA2_STREAM2);
    }
    
    let mut buffer_manager = DoubleBufferManager::new();
    
    // 主循环
    loop {
        // 检查是否有准备好的缓冲区
        if let Some(buffer) = buffer_manager.get_ready_buffer() {
            process_buffer_data(buffer);
        }
        
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn DMA2_STREAM2() {
    free(|cs| {
        if let Some(ref mut transfer) = RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            if transfer.is_complete() {
                transfer.clear_transfer_complete_interrupt();
                
                // 标记当前缓冲区为就绪
                let current = CURRENT_BUFFER.load(Ordering::Acquire);
                if current == 0 {
                    BUFFER_A_READY.store(true, Ordering::Release);
                } else {
                    BUFFER_B_READY.store(true, Ordering::Release);
                }
                
                // 切换到另一个缓冲区
                // 注意：实际实现中需要更复杂的缓冲区切换逻辑
            }
        }
    });
}

fn process_buffer_data(buffer: &[u8]) {
    // 处理缓冲区中的数据
    for &byte in buffer {
        if byte != 0 {
            // 处理有效数据
            process_received_byte(byte);
        }
    }
}

fn process_received_byte(byte: u8) {
    // 处理单个字节
    match byte {
        b'\r' | b'\n' => {
            // 行结束处理
        }
        0x20..=0x7E => {
            // 可打印字符处理
        }
        _ => {
            // 其他字符处理
        }
    }
}
```

## 高级DMA特性

### 1. DMA性能优化

```rust
// DMA性能优化配置
pub struct DmaOptimizer {
    burst_size: u8,
    fifo_threshold: u8,
    priority: u8,
}

impl DmaOptimizer {
    pub fn new() -> Self {
        Self {
            burst_size: 4,      // 4字节突发传输
            fifo_threshold: 2,  // FIFO阈值
            priority: 3,        // 最高优先级
        }
    }
    
    pub fn configure_high_performance(&mut self) -> DmaConfig {
        let mut config = DmaConfig::default();
        
        // 启用FIFO模式
        config.fifo_enable = true;
        config.fifo_threshold = stm32f4xx_hal::dma::config::FifoThreshold::HalfFull;
        
        // 配置突发传输
        config.memory_burst = stm32f4xx_hal::dma::config::BurstMode::Incr4;
        config.peripheral_burst = stm32f4xx_hal::dma::config::BurstMode::Incr4;
        
        // 设置优先级
        config.priority = stm32f4xx_hal::dma::config::Priority::VeryHigh;
        
        // 启用双缓冲模式
        config.double_buffer = true;
        
        config
    }
    
    pub fn configure_low_power(&mut self) -> DmaConfig {
        let mut config = DmaConfig::default();
        
        // 禁用FIFO以降低功耗
        config.fifo_enable = false;
        
        // 单次传输
        config.memory_burst = stm32f4xx_hal::dma::config::BurstMode::Single;
        config.peripheral_burst = stm32f4xx_hal::dma::config::BurstMode::Single;
        
        // 低优先级
        config.priority = stm32f4xx_hal::dma::config::Priority::Low;
        
        config
    }
}

// 性能监控
pub struct DmaPerformanceMonitor {
    transfer_count: u32,
    error_count: u32,
    total_bytes: u32,
    start_time: u32,
}

impl DmaPerformanceMonitor {
    pub fn new() -> Self {
        Self {
            transfer_count: 0,
            error_count: 0,
            total_bytes: 0,
            start_time: get_system_time(),
        }
    }
    
    pub fn record_transfer(&mut self, bytes: u32) {
        self.transfer_count += 1;
        self.total_bytes += bytes;
    }
    
    pub fn record_error(&mut self) {
        self.error_count += 1;
    }
    
    pub fn get_throughput_mbps(&self) -> f32 {
        let elapsed_ms = get_system_time() - self.start_time;
        if elapsed_ms > 0 {
            (self.total_bytes as f32 * 8.0) / (elapsed_ms as f32 * 1000.0)
        } else {
            0.0
        }
    }
    
    pub fn get_error_rate(&self) -> f32 {
        if self.transfer_count > 0 {
            (self.error_count as f32) / (self.transfer_count as f32)
        } else {
            0.0
        }
    }
}

fn get_system_time() -> u32 {
    // 实际实现中应该返回系统时间
    0
}
```

### 2. 内存管理

```rust
use heapless::pool::{Pool, Node};

// 内存池管理
pub struct DmaMemoryPool {
    pool: Pool<Node<[u8; 256]>>,
    memory: [Node<[u8; 256]>; 8],
}

impl DmaMemoryPool {
    pub fn new() -> Self {
        Self {
            pool: Pool::new(),
            memory: [Node::new(); 8],
        }
    }
    
    pub fn init(&mut self) {
        for node in &mut self.memory {
            self.pool.manage(node);
        }
    }
    
    pub fn allocate(&mut self) -> Option<heapless::pool::Managed<[u8; 256]>> {
        self.pool.alloc([0; 256])
    }
    
    pub fn get_available_count(&self) -> usize {
        // 返回可用缓冲区数量
        // 实际实现需要跟踪使用情况
        0
    }
}

// 零拷贝缓冲区管理
pub struct ZeroCopyBuffer {
    buffer: &'static mut [u8],
    length: usize,
    capacity: usize,
}

impl ZeroCopyBuffer {
    pub fn new(buffer: &'static mut [u8]) -> Self {
        let capacity = buffer.len();
        Self {
            buffer,
            length: 0,
            capacity,
        }
    }
    
    pub fn as_slice(&self) -> &[u8] {
        &self.buffer[..self.length]
    }
    
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.buffer[..self.capacity]
    }
    
    pub fn set_length(&mut self, length: usize) {
        if length <= self.capacity {
            self.length = length;
        }
    }
    
    pub fn capacity(&self) -> usize {
        self.capacity
    }
    
    pub fn len(&self) -> usize {
        self.length
    }
}
```

### 3. 错误处理和恢复

```rust
#[derive(Debug, Clone, Copy)]
pub enum DmaError {
    TransferError,
    FifoError,
    DirectModeError,
    BufferOverflow,
    ConfigurationError,
}

pub struct DmaErrorHandler {
    error_counts: [u32; 5],
    recovery_strategies: [RecoveryStrategy; 5],
}

#[derive(Debug, Clone, Copy)]
enum RecoveryStrategy {
    Restart,
    Reconfigure,
    SwitchBuffer,
    Ignore,
}

impl DmaErrorHandler {
    pub fn new() -> Self {
        Self {
            error_counts: [0; 5],
            recovery_strategies: [
                RecoveryStrategy::Restart,      // TransferError
                RecoveryStrategy::Reconfigure,  // FifoError
                RecoveryStrategy::Reconfigure,  // DirectModeError
                RecoveryStrategy::SwitchBuffer, // BufferOverflow
                RecoveryStrategy::Restart,      // ConfigurationError
            ],
        }
    }
    
    pub fn handle_error(&mut self, error: DmaError) -> bool {
        let error_index = error as usize;
        self.error_counts[error_index] += 1;
        
        match self.recovery_strategies[error_index] {
            RecoveryStrategy::Restart => {
                self.restart_dma();
                true
            }
            RecoveryStrategy::Reconfigure => {
                self.reconfigure_dma();
                true
            }
            RecoveryStrategy::SwitchBuffer => {
                self.switch_buffer();
                true
            }
            RecoveryStrategy::Ignore => {
                false
            }
        }
    }
    
    fn restart_dma(&self) {
        // 重启DMA传输
    }
    
    fn reconfigure_dma(&self) {
        // 重新配置DMA参数
    }
    
    fn switch_buffer(&self) {
        // 切换到备用缓冲区
    }
    
    pub fn get_error_statistics(&self) -> [u32; 5] {
        self.error_counts
    }
}
```

## 应用示例

### 1. 高速数据采集

```rust
// 高速ADC数据采集通过DMA传输
pub struct HighSpeedDataAcquisition {
    sample_buffer: &'static mut [u16],
    sample_rate: u32,
    channels: u8,
    dma_transfer: Option<AdcDmaTransfer>,
}

type AdcDmaTransfer = Transfer<
    Stream0<pac::DMA2>,
    Channel0,
    pac::ADC1,
    stm32f4xx_hal::dma::PeripheralToMemory,
    &'static mut [u16],
>;

impl HighSpeedDataAcquisition {
    pub fn new(
        buffer: &'static mut [u16],
        sample_rate: u32,
        channels: u8,
    ) -> Self {
        Self {
            sample_buffer: buffer,
            sample_rate,
            channels,
            dma_transfer: None,
        }
    }
    
    pub fn start_acquisition(&mut self) {
        if let Some(ref mut transfer) = self.dma_transfer {
            transfer.start(|_adc| {});
        }
    }
    
    pub fn stop_acquisition(&mut self) {
        if let Some(ref mut transfer) = self.dma_transfer {
            transfer.pause(|_adc| {});
        }
    }
    
    pub fn process_samples(&self) -> Vec<f32, 1024> {
        let mut processed = Vec::new();
        
        for &sample in self.sample_buffer {
            // 转换ADC值为电压
            let voltage = (sample as f32 * 3.3) / 4095.0;
            processed.push(voltage).ok();
        }
        
        processed
    }
    
    pub fn get_sample_rate(&self) -> u32 {
        self.sample_rate
    }
    
    pub fn get_buffer_utilization(&self) -> f32 {
        // 返回缓冲区使用率
        if let Some(ref transfer) = self.dma_transfer {
            let remaining = transfer.number_of_transfers() as usize;
            let total = self.sample_buffer.len();
            ((total - remaining) as f32) / (total as f32)
        } else {
            0.0
        }
    }
}
```

### 2. 流式数据处理

```rust
// 流式数据处理器
pub struct StreamProcessor {
    input_buffer: &'static mut [u8],
    output_buffer: &'static mut [u8],
    processing_function: fn(&[u8]) -> Vec<u8, 512>,
}

impl StreamProcessor {
    pub fn new(
        input_buffer: &'static mut [u8],
        output_buffer: &'static mut [u8],
        processing_function: fn(&[u8]) -> Vec<u8, 512>,
    ) -> Self {
        Self {
            input_buffer,
            output_buffer,
            processing_function,
        }
    }
    
    pub fn process_stream(&mut self) {
        // 处理输入缓冲区中的数据
        let processed = (self.processing_function)(self.input_buffer);
        
        // 将处理结果写入输出缓冲区
        let copy_len = core::cmp::min(processed.len(), self.output_buffer.len());
        self.output_buffer[..copy_len].copy_from_slice(&processed[..copy_len]);
    }
    
    pub fn get_input_buffer(&self) -> &[u8] {
        self.input_buffer
    }
    
    pub fn get_output_buffer(&self) -> &[u8] {
        self.output_buffer
    }
}

// 数据处理函数示例
fn filter_data(input: &[u8]) -> Vec<u8, 512> {
    let mut output = Vec::new();
    
    for &byte in input {
        // 简单的滤波处理
        if byte > 0x10 && byte < 0xF0 {
            output.push(byte).ok();
        }
    }
    
    output
}

fn compress_data(input: &[u8]) -> Vec<u8, 512> {
    let mut output = Vec::new();
    let mut count = 1u8;
    let mut last_byte = input[0];
    
    for &byte in &input[1..] {
        if byte == last_byte && count < 255 {
            count += 1;
        } else {
            // 输出压缩数据：[计数][字节]
            output.push(count).ok();
            output.push(last_byte).ok();
            last_byte = byte;
            count = 1;
        }
    }
    
    // 输出最后一组
    output.push(count).ok();
    output.push(last_byte).ok();
    
    output
}
```

## 性能测试和调优

### 1. 性能基准测试

```rust
pub struct DmaBenchmark {
    test_data: &'static [u8],
    iterations: u32,
    results: BenchmarkResults,
}

#[derive(Debug)]
pub struct BenchmarkResults {
    pub throughput_mbps: f32,
    pub latency_us: u32,
    pub cpu_utilization: f32,
    pub error_rate: f32,
}

impl DmaBenchmark {
    pub fn new(test_data: &'static [u8], iterations: u32) -> Self {
        Self {
            test_data,
            iterations,
            results: BenchmarkResults {
                throughput_mbps: 0.0,
                latency_us: 0,
                cpu_utilization: 0.0,
                error_rate: 0.0,
            },
        }
    }
    
    pub fn run_throughput_test(&mut self) {
        let start_time = get_cycle_count();
        let start_cpu_time = get_cpu_time();
        
        for _ in 0..self.iterations {
            // 执行DMA传输
            self.perform_dma_transfer();
        }
        
        let end_time = get_cycle_count();
        let end_cpu_time = get_cpu_time();
        
        // 计算结果
        let elapsed_cycles = end_time - start_time;
        let elapsed_us = cycles_to_microseconds(elapsed_cycles);
        let total_bytes = self.test_data.len() * self.iterations as usize;
        
        self.results.throughput_mbps = (total_bytes as f32 * 8.0) / (elapsed_us as f32);
        self.results.latency_us = elapsed_us / self.iterations;
        self.results.cpu_utilization = ((end_cpu_time - start_cpu_time) as f32) / (elapsed_cycles as f32);
    }
    
    pub fn run_latency_test(&mut self) {
        let mut total_latency = 0u32;
        
        for _ in 0..self.iterations {
            let start = get_cycle_count();
            self.perform_single_transfer();
            let end = get_cycle_count();
            
            total_latency += cycles_to_microseconds(end - start);
        }
        
        self.results.latency_us = total_latency / self.iterations;
    }
    
    fn perform_dma_transfer(&self) {
        // 执行DMA传输的实际代码
    }
    
    fn perform_single_transfer(&self) {
        // 执行单次传输
    }
    
    pub fn get_results(&self) -> &BenchmarkResults {
        &self.results
    }
}

fn get_cycle_count() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

fn get_cpu_time() -> u32 {
    // 返回CPU使用时间
    0
}

fn cycles_to_microseconds(cycles: u32) -> u32 {
    // 假设168MHz系统时钟
    cycles / 168
}
```

### 2. 调优建议

```rust
pub struct DmaTuningGuide {
    system_config: SystemConfig,
    recommendations: Vec<TuningRecommendation, 16>,
}

#[derive(Debug)]
pub struct SystemConfig {
    pub cpu_frequency: u32,
    pub memory_type: MemoryType,
    pub data_size: usize,
    pub transfer_frequency: u32,
}

#[derive(Debug)]
pub enum MemoryType {
    Sram,
    Flash,
    External,
}

#[derive(Debug)]
pub struct TuningRecommendation {
    pub parameter: String<32>,
    pub recommended_value: String<32>,
    pub reason: String<128>,
}

impl DmaTuningGuide {
    pub fn analyze_system(&mut self, config: SystemConfig) {
        self.system_config = config;
        self.recommendations.clear();
        
        // 分析并生成建议
        self.analyze_burst_size();
        self.analyze_fifo_settings();
        self.analyze_priority_settings();
        self.analyze_buffer_size();
    }
    
    fn analyze_burst_size(&mut self) {
        let recommended_burst = if self.system_config.data_size > 1024 {
            "INCR8"
        } else if self.system_config.data_size > 256 {
            "INCR4"
        } else {
            "SINGLE"
        };
        
        self.recommendations.push(TuningRecommendation {
            parameter: String::from("burst_size"),
            recommended_value: String::from(recommended_burst),
            reason: String::from("Optimized for data size and memory bandwidth"),
        }).ok();
    }
    
    fn analyze_fifo_settings(&mut self) {
        let enable_fifo = self.system_config.transfer_frequency > 100_000;
        
        self.recommendations.push(TuningRecommendation {
            parameter: String::from("fifo_enable"),
            recommended_value: String::from(if enable_fifo { "true" } else { "false" }),
            reason: String::from("FIFO improves performance for high-frequency transfers"),
        }).ok();
    }
    
    fn analyze_priority_settings(&mut self) {
        let priority = if self.system_config.transfer_frequency > 1_000_000 {
            "VeryHigh"
        } else if self.system_config.transfer_frequency > 100_000 {
            "High"
        } else {
            "Medium"
        };
        
        self.recommendations.push(TuningRecommendation {
            parameter: String::from("priority"),
            recommended_value: String::from(priority),
            reason: String::from("Priority based on transfer frequency requirements"),
        }).ok();
    }
    
    fn analyze_buffer_size(&mut self) {
        let recommended_size = if self.system_config.transfer_frequency > 1_000_000 {
            "2048"
        } else if self.system_config.transfer_frequency > 100_000 {
            "1024"
        } else {
            "512"
        };
        
        self.recommendations.push(TuningRecommendation {
            parameter: String::from("buffer_size"),
            recommended_value: String::from(recommended_size),
            reason: String::from("Buffer size optimized for transfer frequency"),
        }).ok();
    }
    
    pub fn get_recommendations(&self) -> &[TuningRecommendation] {
        &self.recommendations
    }
}
```

## 调试和故障排除

### 1. DMA调试工具

```rust
pub struct DmaDebugger {
    transfer_log: Vec<TransferEvent, 64>,
    error_log: Vec<ErrorEvent, 32>,
    performance_counters: PerformanceCounters,
}

#[derive(Debug)]
pub struct TransferEvent {
    pub timestamp: u32,
    pub event_type: TransferEventType,
    pub bytes_transferred: usize,
    pub buffer_address: u32,
}

#[derive(Debug)]
pub enum TransferEventType {
    Started,
    HalfComplete,
    Complete,
    Error,
    Paused,
    Resumed,
}

#[derive(Debug)]
pub struct ErrorEvent {
    pub timestamp: u32,
    pub error_type: DmaError,
    pub register_state: RegisterState,
}

#[derive(Debug)]
pub struct RegisterState {
    pub cr: u32,    // Control Register
    pub ndtr: u32,  // Number of Data Transfer Register
    pub par: u32,   // Peripheral Address Register
    pub m0ar: u32,  // Memory 0 Address Register
}

#[derive(Debug)]
pub struct PerformanceCounters {
    pub total_transfers: u32,
    pub successful_transfers: u32,
    pub error_count: u32,
    pub total_bytes: u32,
    pub average_latency_us: u32,
}

impl DmaDebugger {
    pub fn new() -> Self {
        Self {
            transfer_log: Vec::new(),
            error_log: Vec::new(),
            performance_counters: PerformanceCounters {
                total_transfers: 0,
                successful_transfers: 0,
                error_count: 0,
                total_bytes: 0,
                average_latency_us: 0,
            },
        }
    }
    
    pub fn log_transfer_event(&mut self, event_type: TransferEventType, bytes: usize, address: u32) {
        let event = TransferEvent {
            timestamp: get_system_time(),
            event_type,
            bytes_transferred: bytes,
            buffer_address: address,
        };
        
        self.transfer_log.push(event).ok();
        
        // 更新性能计数器
        match event_type {
            TransferEventType::Started => {
                self.performance_counters.total_transfers += 1;
            }
            TransferEventType::Complete => {
                self.performance_counters.successful_transfers += 1;
                self.performance_counters.total_bytes += bytes as u32;
            }
            TransferEventType::Error => {
                self.performance_counters.error_count += 1;
            }
            _ => {}
        }
    }
    
    pub fn log_error(&mut self, error: DmaError, registers: RegisterState) {
        let error_event = ErrorEvent {
            timestamp: get_system_time(),
            error_type: error,
            register_state: registers,
        };
        
        self.error_log.push(error_event).ok();
    }
    
    pub fn dump_transfer_log(&self) -> &[TransferEvent] {
        &self.transfer_log
    }
    
    pub fn dump_error_log(&self) -> &[ErrorEvent] {
        &self.error_log
    }
    
    pub fn get_performance_summary(&self) -> &PerformanceCounters {
        &self.performance_counters
    }
    
    pub fn clear_logs(&mut self) {
        self.transfer_log.clear();
        self.error_log.clear();
    }
}

// 调试宏
#[macro_export]
macro_rules! dma_debug {
    ($debugger:expr, $event:expr, $bytes:expr, $addr:expr) => {
        #[cfg(debug_assertions)]
        $debugger.log_transfer_event($event, $bytes, $addr);
    };
}
```

### 2. 常见问题诊断

```rust
pub struct DmaDiagnostics;

impl DmaDiagnostics {
    pub fn diagnose_transfer_failure(error: DmaError, registers: &RegisterState) -> DiagnosisResult {
        match error {
            DmaError::TransferError => {
                if registers.ndtr == 0 {
                    DiagnosisResult::new(
                        "Transfer completed but error flag set",
                        "Check peripheral status and clear error flags",
                        Severity::Warning,
                    )
                } else {
                    DiagnosisResult::new(
                        "Transfer interrupted unexpectedly",
                        "Check bus arbitration and memory access permissions",
                        Severity::Error,
                    )
                }
            }
            DmaError::FifoError => {
                DiagnosisResult::new(
                    "FIFO overrun or underrun",
                    "Adjust FIFO threshold or disable FIFO mode",
                    Severity::Error,
                )
            }
            DmaError::DirectModeError => {
                DiagnosisResult::new(
                    "Direct mode error with FIFO enabled",
                    "Disable FIFO mode or configure proper threshold",
                    Severity::Error,
                )
            }
            DmaError::BufferOverflow => {
                DiagnosisResult::new(
                    "Buffer overflow detected",
                    "Increase buffer size or processing speed",
                    Severity::Critical,
                )
            }
            DmaError::ConfigurationError => {
                DiagnosisResult::new(
                    "Invalid DMA configuration",
                    "Review channel mapping and transfer parameters",
                    Severity::Error,
                )
            }
        }
    }
    
    pub fn check_performance_issues(counters: &PerformanceCounters) -> Vec<DiagnosisResult, 8> {
        let mut issues = Vec::new();
        
        // 检查错误率
        let error_rate = (counters.error_count as f32) / (counters.total_transfers as f32);
        if error_rate > 0.05 {
            issues.push(DiagnosisResult::new(
                "High error rate detected",
                "Review system configuration and signal integrity",
                Severity::Warning,
            )).ok();
        }
        
        // 检查延迟
        if counters.average_latency_us > 1000 {
            issues.push(DiagnosisResult::new(
                "High transfer latency",
                "Optimize DMA configuration and reduce system load",
                Severity::Warning,
            )).ok();
        }
        
        // 检查成功率
        let success_rate = (counters.successful_transfers as f32) / (counters.total_transfers as f32);
        if success_rate < 0.95 {
            issues.push(DiagnosisResult::new(
                "Low transfer success rate",
                "Investigate hardware issues and configuration errors",
                Severity::Critical,
            )).ok();
        }
        
        issues
    }
}

#[derive(Debug)]
pub struct DiagnosisResult {
    pub issue: String<64>,
    pub recommendation: String<128>,
    pub severity: Severity,
}

#[derive(Debug)]
pub enum Severity {
    Info,
    Warning,
    Error,
    Critical,
}

impl DiagnosisResult {
    pub fn new(issue: &str, recommendation: &str, severity: Severity) -> Self {
        Self {
            issue: String::from(issue),
            recommendation: String::from(recommendation),
            severity,
        }
    }
}
```

## 总结

DMA传输是嵌入式系统中实现高效数据传输的关键技术。通过合理的配置和使用，可以显著提高系统性能并降低CPU负载。关键要点包括：

1. **正确的DMA配置**：选择合适的传输模式、缓冲区大小和优先级
2. **高效的内存管理**：使用内存池和零拷贝技术
3. **完善的错误处理**：检测和恢复各种DMA错误
4. **性能优化**：根据应用需求调整DMA参数
5. **调试支持**：实现完整的调试和诊断功能

这些技术为构建高性能的串口通信系统提供了强大的支持。

## 参考资料

- [STM32F4xx DMA Controller Reference](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [embedded-dma Documentation](https://docs.rs/embedded-dma/)
- [STM32F4xx HAL DMA Documentation](https://docs.rs/stm32f4xx-hal/latest/stm32f4xx_hal/dma/)
- [ARM Cortex-M DMA Best Practices](https://developer.arm.com/documentation/)