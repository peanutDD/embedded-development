//! 双缓冲DMA串口传输示例
//! 
//! 本示例演示如何使用双缓冲DMA模式实现高效的串口数据传输：
//! - 双缓冲区配置和管理
//! - 缓冲区切换机制
//! - 零拷贝数据处理
//! - 高吞吐量数据流处理

#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::interrupt::{free, Mutex};
use stm32f4xx_hal::{
    dma::{
        config::DmaConfig,
        MemoryToPeripheral, PeripheralToMemory,
        Stream2, Stream7,
        StreamsTuple, Transfer,
    },
    gpio::{Alternate, Pin},
    pac::{self, interrupt, DMA2, USART1},
    prelude::*,
    rcc::RccExt,
    serial::{Config, Serial, Rx, Tx},
};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

// 双缓冲区大小
const BUFFER_SIZE: usize = 1024;
const PROCESSING_CHUNK_SIZE: usize = 256;

// DMA传输类型定义
type TxTransfer = Transfer<Stream7<DMA2>, Tx<USART1>, MemoryToPeripheral, &'static mut [u8]>;
type RxTransfer = Transfer<Stream2<DMA2>, Rx<USART1>, PeripheralToMemory, &'static mut [u8]>;

// 全局DMA传输句柄
static G_TX_TRANSFER: Mutex<RefCell<Option<TxTransfer>>> = Mutex::new(RefCell::new(None));
static G_RX_TRANSFER: Mutex<RefCell<Option<RxTransfer>>> = Mutex::new(RefCell::new(None));

// 双缓冲区
static mut RX_BUFFER_A: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut RX_BUFFER_B: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut TX_BUFFER_A: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut TX_BUFFER_B: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

// 缓冲区状态管理
static RX_ACTIVE_BUFFER: AtomicBool = AtomicBool::new(false); // false = A, true = B
static TX_ACTIVE_BUFFER: AtomicBool = AtomicBool::new(false);
static RX_BUFFER_READY: AtomicBool = AtomicBool::new(false);
static TX_BUFFER_READY: AtomicBool = AtomicBool::new(false);
static RX_DATA_SIZE: AtomicUsize = AtomicUsize::new(0);
static TX_DATA_SIZE: AtomicUsize = AtomicUsize::new(0);

// 统计信息
static mut STATS: DoubleBufferStats = DoubleBufferStats::new();

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate::<7>();
    let rx_pin = gpioa.pa10.into_alternate::<7>();

    // 配置串口
    let config = Config::default()
        .baudrate(921600.bps()) // 高波特率测试
        .wordlength_8()
        .parity_none()
        .stopbits(stm32f4xx_hal::serial::StopBits::STOP1);

    let serial = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    let (tx, rx) = serial.split();

    // 初始化DMA
    let dma2 = StreamsTuple::new(dp.DMA2);

    // 设置双缓冲DMA传输
    setup_double_buffer_dma(dma2.7, dma2.2, tx, rx);

    // 启用DMA中断
    unsafe {
        pac::NVIC::unmask(interrupt::DMA2_STREAM7); // TX中断
        pac::NVIC::unmask(interrupt::DMA2_STREAM2); // RX中断
    }

    // 开始第一次接收
    start_reception_on_buffer_a();

    let mut delay = cp.SYST.delay(&clocks);
    let mut data_processor = DataProcessor::new();

    loop {
        // 检查是否有接收缓冲区准备好处理
        if RX_BUFFER_READY.load(Ordering::Acquire) {
            let data_size = RX_DATA_SIZE.load(Ordering::Acquire);
            let buffer = get_inactive_rx_buffer();
            
            // 处理接收到的数据
            data_processor.process_received_data(buffer, data_size);
            
            // 标记缓冲区已处理
            RX_BUFFER_READY.store(false, Ordering::Release);
            
            unsafe {
                STATS.buffers_processed += 1;
            }
        }

        // 检查是否需要发送数据
        if !TX_BUFFER_READY.load(Ordering::Acquire) {
            let tx_buffer = get_inactive_tx_buffer();
            let data_size = data_processor.prepare_transmit_data(tx_buffer);
            
            if data_size > 0 {
                TX_DATA_SIZE.store(data_size, Ordering::Release);
                TX_BUFFER_READY.store(true, Ordering::Release);
                start_transmission();
            }
        }

        // 定期输出统计信息
        if unsafe { STATS.buffers_processed } % 100 == 0 && unsafe { STATS.buffers_processed } > 0 {
            output_statistics();
        }

        delay.delay_us(100u32);
    }
}

/// 设置双缓冲DMA传输
fn setup_double_buffer_dma(
    tx_stream: Stream7<DMA2>,
    rx_stream: Stream2<DMA2>,
    tx: Tx<USART1>,
    rx: Rx<USART1>,
) {
    // 配置接收DMA
    let rx_config = DmaConfig::default()
        .memory_increment(true)
        .peripheral_increment(false)
        .transfer_complete_interrupt(true)
        .double_buffer(true);

    let rx_transfer = Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        unsafe { &mut RX_BUFFER_A },
        Some(unsafe { &mut RX_BUFFER_B }),
        rx_config,
    );

    // 配置发送DMA
    let tx_config = DmaConfig::default()
        .memory_increment(true)
        .peripheral_increment(false)
        .transfer_complete_interrupt(true)
        .double_buffer(true);

    let tx_transfer = Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        unsafe { &mut TX_BUFFER_A },
        Some(unsafe { &mut TX_BUFFER_B }),
        tx_config,
    );

    // 存储到全局变量
    free(|cs| {
        G_TX_TRANSFER.borrow(cs).replace(Some(tx_transfer));
        G_RX_TRANSFER.borrow(cs).replace(Some(rx_transfer));
    });
}

/// 开始在缓冲区A上接收
fn start_reception_on_buffer_a() {
    free(|cs| {
        if let Some(ref mut transfer) = G_RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            transfer.start(BUFFER_SIZE as u16);
            RX_ACTIVE_BUFFER.store(false, Ordering::Release); // A缓冲区激活
        }
    });
}

/// 开始传输
fn start_transmission() {
    if TX_BUFFER_READY.load(Ordering::Acquire) {
        let data_size = TX_DATA_SIZE.load(Ordering::Acquire);
        
        free(|cs| {
            if let Some(ref mut transfer) = G_TX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
                transfer.start(data_size as u16);
            }
        });
    }
}

/// 获取非活动的接收缓冲区
fn get_inactive_rx_buffer() -> &'static mut [u8] {
    unsafe {
        if RX_ACTIVE_BUFFER.load(Ordering::Acquire) {
            &mut RX_BUFFER_A // B是活动的，返回A
        } else {
            &mut RX_BUFFER_B // A是活动的，返回B
        }
    }
}

/// 获取非活动的发送缓冲区
fn get_inactive_tx_buffer() -> &'static mut [u8] {
    unsafe {
        if TX_ACTIVE_BUFFER.load(Ordering::Acquire) {
            &mut TX_BUFFER_A // B是活动的，返回A
        } else {
            &mut TX_BUFFER_B // A是活动的，返回B
        }
    }
}

/// 数据处理器
pub struct DataProcessor {
    packet_counter: u32,
    echo_enabled: bool,
    processing_buffer: [u8; PROCESSING_CHUNK_SIZE],
}

impl DataProcessor {
    pub fn new() -> Self {
        Self {
            packet_counter: 0,
            echo_enabled: true,
            processing_buffer: [0; PROCESSING_CHUNK_SIZE],
        }
    }
    
    /// 处理接收到的数据
    pub fn process_received_data(&mut self, buffer: &[u8], size: usize) {
        let data = &buffer[..size];
        
        // 数据包解析和处理
        for chunk in data.chunks(PROCESSING_CHUNK_SIZE) {
            self.process_chunk(chunk);
        }
        
        unsafe {
            STATS.bytes_processed += size as u32;
        }
    }
    
    /// 处理数据块
    fn process_chunk(&mut self, chunk: &[u8]) {
        // 简单的数据处理逻辑
        for (i, &byte) in chunk.iter().enumerate() {
            self.processing_buffer[i] = byte.wrapping_add(1); // 简单变换
        }
        
        self.packet_counter += 1;
    }
    
    /// 准备发送数据
    pub fn prepare_transmit_data(&mut self, buffer: &mut [u8]) -> usize {
        if !self.echo_enabled {
            return 0;
        }
        
        // 生成测试数据
        let message = self.generate_test_message();
        let len = message.len().min(buffer.len());
        
        buffer[..len].copy_from_slice(&message[..len]);
        len
    }
    
    /// 生成测试消息
    fn generate_test_message(&self) -> heapless::Vec<u8, 256> {
        let mut message = heapless::Vec::new();
        use core::fmt::Write;
        
        write!(
            message,
            "Double Buffer Test #{} - Processed {} packets\r\n",
            self.packet_counter,
            self.packet_counter
        ).unwrap();
        
        message
    }
    
    pub fn set_echo_enabled(&mut self, enabled: bool) {
        self.echo_enabled = enabled;
    }
    
    pub fn get_packet_count(&self) -> u32 {
        self.packet_counter
    }
}

/// DMA接收完成中断
#[interrupt]
fn DMA2_STREAM2() {
    free(|cs| {
        if let Some(ref mut transfer) = G_RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            if transfer.is_complete() {
                transfer.clear_transfer_complete_interrupt();
                
                // 获取接收到的数据大小
                let remaining = transfer.get_number_of_transfers() as usize;
                let received = BUFFER_SIZE - remaining;
                
                // 存储数据大小
                RX_DATA_SIZE.store(received, Ordering::Release);
                
                // 切换缓冲区
                let current_buffer = RX_ACTIVE_BUFFER.load(Ordering::Acquire);
                RX_ACTIVE_BUFFER.store(!current_buffer, Ordering::Release);
                
                // 标记缓冲区准备好处理
                RX_BUFFER_READY.store(true, Ordering::Release);
                
                // 重新启动接收
                transfer.start(BUFFER_SIZE as u16);
                
                unsafe {
                    STATS.rx_transfers += 1;
                    STATS.bytes_received += received as u32;
                }
            }
        }
    });
}

/// DMA发送完成中断
#[interrupt]
fn DMA2_STREAM7() {
    free(|cs| {
        if let Some(ref mut transfer) = G_TX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            if transfer.is_complete() {
                transfer.clear_transfer_complete_interrupt();
                
                // 切换发送缓冲区
                let current_buffer = TX_ACTIVE_BUFFER.load(Ordering::Acquire);
                TX_ACTIVE_BUFFER.store(!current_buffer, Ordering::Release);
                
                // 标记发送完成
                TX_BUFFER_READY.store(false, Ordering::Release);
                
                unsafe {
                    STATS.tx_transfers += 1;
                    let data_size = TX_DATA_SIZE.load(Ordering::Acquire);
                    STATS.bytes_transmitted += data_size as u32;
                }
            }
        }
    });
}

/// 输出统计信息
fn output_statistics() {
    unsafe {
        // 这里可以通过其他方式输出统计信息
        // 例如通过另一个串口或者存储到内存中
        STATS.update_throughput();
    }
}

/// 双缓冲统计信息
#[derive(Debug)]
pub struct DoubleBufferStats {
    pub rx_transfers: u32,
    pub tx_transfers: u32,
    pub bytes_received: u32,
    pub bytes_transmitted: u32,
    pub bytes_processed: u32,
    pub buffers_processed: u32,
    pub buffer_switches: u32,
    pub throughput_bps: f32,
    pub last_update_time: u32,
}

impl DoubleBufferStats {
    pub const fn new() -> Self {
        Self {
            rx_transfers: 0,
            tx_transfers: 0,
            bytes_received: 0,
            bytes_transmitted: 0,
            bytes_processed: 0,
            buffers_processed: 0,
            buffer_switches: 0,
            throughput_bps: 0.0,
            last_update_time: 0,
        }
    }
    
    pub fn update_throughput(&mut self) {
        let current_time = get_system_time_ms();
        let elapsed = current_time - self.last_update_time;
        
        if elapsed > 0 {
            self.throughput_bps = (self.bytes_processed as f32 * 8.0 * 1000.0) / elapsed as f32;
            self.last_update_time = current_time;
        }
    }
    
    pub fn reset(&mut self) {
        *self = Self::new();
        self.last_update_time = get_system_time_ms();
    }
    
    pub fn get_efficiency(&self) -> f32 {
        if self.rx_transfers > 0 {
            (self.bytes_processed as f32) / (self.bytes_received as f32)
        } else {
            0.0
        }
    }
    
    pub fn get_average_buffer_size(&self) -> f32 {
        if self.rx_transfers > 0 {
            (self.bytes_received as f32) / (self.rx_transfers as f32)
        } else {
            0.0
        }
    }
}

/// 缓冲区管理器
pub struct BufferManager {
    buffer_a: &'static mut [u8],
    buffer_b: &'static mut [u8],
    active_buffer: bool, // false = A, true = B
    buffer_size: usize,
}

impl BufferManager {
    pub fn new(buffer_a: &'static mut [u8], buffer_b: &'static mut [u8]) -> Self {
        let buffer_size = buffer_a.len().min(buffer_b.len());
        
        Self {
            buffer_a,
            buffer_b,
            active_buffer: false,
            buffer_size,
        }
    }
    
    pub fn get_active_buffer(&mut self) -> &mut [u8] {
        if self.active_buffer {
            &mut self.buffer_b[..self.buffer_size]
        } else {
            &mut self.buffer_a[..self.buffer_size]
        }
    }
    
    pub fn get_inactive_buffer(&mut self) -> &mut [u8] {
        if self.active_buffer {
            &mut self.buffer_a[..self.buffer_size]
        } else {
            &mut self.buffer_b[..self.buffer_size]
        }
    }
    
    pub fn switch_buffers(&mut self) {
        self.active_buffer = !self.active_buffer;
    }
    
    pub fn is_buffer_a_active(&self) -> bool {
        !self.active_buffer
    }
    
    pub fn clear_active_buffer(&mut self) {
        let buffer = self.get_active_buffer();
        buffer.fill(0);
    }
    
    pub fn clear_inactive_buffer(&mut self) {
        let buffer = self.get_inactive_buffer();
        buffer.fill(0);
    }
    
    pub fn clear_all_buffers(&mut self) {
        self.buffer_a.fill(0);
        self.buffer_b.fill(0);
    }
}

/// 性能监控器
pub struct PerformanceMonitor {
    start_time: u32,
    sample_count: u32,
    min_latency: u32,
    max_latency: u32,
    total_latency: u64,
}

impl PerformanceMonitor {
    pub fn new() -> Self {
        Self {
            start_time: get_system_time_ms(),
            sample_count: 0,
            min_latency: u32::MAX,
            max_latency: 0,
            total_latency: 0,
        }
    }
    
    pub fn record_latency(&mut self, latency: u32) {
        self.sample_count += 1;
        self.total_latency += latency as u64;
        self.min_latency = self.min_latency.min(latency);
        self.max_latency = self.max_latency.max(latency);
    }
    
    pub fn get_average_latency(&self) -> f32 {
        if self.sample_count > 0 {
            (self.total_latency as f32) / (self.sample_count as f32)
        } else {
            0.0
        }
    }
    
    pub fn get_latency_stats(&self) -> LatencyStats {
        LatencyStats {
            min: self.min_latency,
            max: self.max_latency,
            average: self.get_average_latency(),
            sample_count: self.sample_count,
        }
    }
    
    pub fn reset(&mut self) {
        self.start_time = get_system_time_ms();
        self.sample_count = 0;
        self.min_latency = u32::MAX;
        self.max_latency = 0;
        self.total_latency = 0;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LatencyStats {
    pub min: u32,
    pub max: u32,
    pub average: f32,
    pub sample_count: u32,
}

/// 获取系统时间（毫秒）
fn get_system_time_ms() -> u32 {
    // 这里应该实现实际的系统时间获取
    // 可以使用DWT、SysTick或RTC
    0
}