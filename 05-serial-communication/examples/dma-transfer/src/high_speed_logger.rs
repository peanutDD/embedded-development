//! 高速数据记录器DMA示例
//! 
//! 本示例演示如何使用DMA实现高速数据记录系统：
//! - 高速数据采集和记录
//! - 多通道DMA协调
//! - 数据压缩和存储
//! - 实时性能监控

#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::interrupt::{free, Mutex};
use stm32f4xx_hal::{
    dma::{
        config::DmaConfig,
        MemoryToPeripheral, PeripheralToMemory,
        Stream0, Stream1, Stream2, Stream7,
        StreamsTuple, Transfer,
    },
    gpio::{Alternate, Pin},
    pac::{self, interrupt, DMA1, DMA2, USART1, USART2},
    prelude::*,
    rcc::RccExt,
    serial::{Config, Serial, Rx, Tx},
    timer::{Timer, Event},
};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
use heapless::{
    pool::{Pool, Node},
    spsc::{Queue, Producer, Consumer},
    Vec,
};

// 系统配置常量
const LOG_BUFFER_SIZE: usize = 2048;
const COMPRESSION_BUFFER_SIZE: usize = 1024;
const METADATA_SIZE: usize = 64;
const MAX_CHANNELS: usize = 4;
const SAMPLE_RATE_HZ: u32 = 100000; // 100kHz采样率

// DMA传输类型定义
type UartTxTransfer = Transfer<Stream7<DMA2>, Tx<USART1>, MemoryToPeripheral, &'static mut [u8]>;
type UartRxTransfer = Transfer<Stream2<DMA2>, Rx<USART1>, PeripheralToMemory, &'static mut [u8]>;

// 全局DMA传输句柄
static G_UART_TX: Mutex<RefCell<Option<UartTxTransfer>>> = Mutex::new(RefCell::new(None));
static G_UART_RX: Mutex<RefCell<Option<UartRxTransfer>>> = Mutex::new(RefCell::new(None));

// 数据缓冲区
static mut LOG_BUFFER_A: [u8; LOG_BUFFER_SIZE] = [0; LOG_BUFFER_SIZE];
static mut LOG_BUFFER_B: [u8; LOG_BUFFER_SIZE] = [0; LOG_BUFFER_SIZE];
static mut COMPRESSION_BUFFER: [u8; COMPRESSION_BUFFER_SIZE] = [0; COMPRESSION_BUFFER_SIZE];
static mut METADATA_BUFFER: [u8; METADATA_SIZE] = [0; METADATA_SIZE];

// 系统状态
static LOGGING_ACTIVE: AtomicBool = AtomicBool::new(false);
static CURRENT_BUFFER: AtomicBool = AtomicBool::new(false); // false = A, true = B
static BUFFER_READY: AtomicBool = AtomicBool::new(false);
static SAMPLES_COLLECTED: AtomicU32 = AtomicU32::new(0);
static BYTES_LOGGED: AtomicUsize = AtomicUsize::new(0);

// 性能统计
static mut PERFORMANCE_STATS: PerformanceStats = PerformanceStats::new();

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟 - 最高性能
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

    // 配置高速串口
    let config = Config::default()
        .baudrate(2_000_000.bps()) // 2Mbps高速传输
        .wordlength_8()
        .parity_none()
        .stopbits(stm32f4xx_hal::serial::StopBits::STOP1);

    let serial = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    let (tx, rx) = serial.split();

    // 初始化DMA
    let dma2 = StreamsTuple::new(dp.DMA2);

    // 设置高速DMA传输
    setup_high_speed_dma(dma2.7, dma2.2, tx, rx);

    // 配置定时器用于采样控制
    let mut timer = Timer::new(dp.TIM2, &clocks).start_count_down(SAMPLE_RATE_HZ.Hz());
    timer.listen(Event::TimeOut);

    // 启用中断
    unsafe {
        pac::NVIC::unmask(interrupt::DMA2_STREAM7);
        pac::NVIC::unmask(interrupt::DMA2_STREAM2);
        pac::NVIC::unmask(interrupt::TIM2);
    }

    // 初始化数据记录器
    let mut logger = HighSpeedLogger::new();
    let mut data_processor = DataProcessor::new();
    let mut compression_engine = CompressionEngine::new();

    let mut delay = cp.SYST.delay(&clocks);

    // 开始记录
    logger.start_logging();
    start_continuous_reception();

    loop {
        // 检查是否有缓冲区准备好处理
        if BUFFER_READY.load(Ordering::Acquire) {
            let buffer = get_ready_buffer();
            let data_size = BYTES_LOGGED.load(Ordering::Acquire);
            
            // 处理数据
            let processed_data = data_processor.process_buffer(buffer, data_size);
            
            // 压缩数据
            let compressed_data = compression_engine.compress(&processed_data);
            
            // 记录到日志
            logger.log_data(&compressed_data);
            
            // 标记缓冲区已处理
            BUFFER_READY.store(false, Ordering::Release);
            
            unsafe {
                PERFORMANCE_STATS.buffers_processed += 1;
            }
        }

        // 定期输出性能统计
        if unsafe { PERFORMANCE_STATS.buffers_processed } % 100 == 0 {
            output_performance_stats();
        }

        // 检查系统健康状态
        check_system_health();

        delay.delay_us(10u32);
    }
}

/// 设置高速DMA传输
fn setup_high_speed_dma(
    tx_stream: Stream7<DMA2>,
    rx_stream: Stream2<DMA2>,
    tx: Tx<USART1>,
    rx: Rx<USART1>,
) {
    // 配置接收DMA - 高优先级，循环模式
    let rx_config = DmaConfig::default()
        .memory_increment(true)
        .peripheral_increment(false)
        .circular_buffer(true)
        .half_transfer_interrupt(true)
        .transfer_complete_interrupt(true)
        .priority(stm32f4xx_hal::dma::config::Priority::VeryHigh);

    let rx_transfer = Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        unsafe { &mut LOG_BUFFER_A },
        None,
        rx_config,
    );

    // 配置发送DMA - 高优先级
    let tx_config = DmaConfig::default()
        .memory_increment(true)
        .peripheral_increment(false)
        .transfer_complete_interrupt(true)
        .priority(stm32f4xx_hal::dma::config::Priority::High);

    let tx_transfer = Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        unsafe { &mut COMPRESSION_BUFFER },
        None,
        tx_config,
    );

    // 存储到全局变量
    free(|cs| {
        G_UART_TX.borrow(cs).replace(Some(tx_transfer));
        G_UART_RX.borrow(cs).replace(Some(rx_transfer));
    });
}

/// 开始连续接收
fn start_continuous_reception() {
    free(|cs| {
        if let Some(ref mut transfer) = G_UART_RX.borrow(cs).borrow_mut().as_mut() {
            transfer.start(LOG_BUFFER_SIZE as u16);
        }
    });
}

/// 获取准备好的缓冲区
fn get_ready_buffer() -> &'static [u8] {
    unsafe {
        if CURRENT_BUFFER.load(Ordering::Acquire) {
            &LOG_BUFFER_B
        } else {
            &LOG_BUFFER_A
        }
    }
}

/// 高速数据记录器
pub struct HighSpeedLogger {
    session_id: u32,
    start_time: u32,
    total_samples: u64,
    dropped_samples: u32,
    compression_ratio: f32,
}

impl HighSpeedLogger {
    pub fn new() -> Self {
        Self {
            session_id: generate_session_id(),
            start_time: get_system_time_ms(),
            total_samples: 0,
            dropped_samples: 0,
            compression_ratio: 1.0,
        }
    }
    
    pub fn start_logging(&mut self) {
        LOGGING_ACTIVE.store(true, Ordering::Release);
        self.start_time = get_system_time_ms();
        
        // 写入会话头部信息
        self.write_session_header();
    }
    
    pub fn stop_logging(&mut self) {
        LOGGING_ACTIVE.store(false, Ordering::Release);
        
        // 写入会话结束信息
        self.write_session_footer();
    }
    
    pub fn log_data(&mut self, data: &[u8]) {
        if LOGGING_ACTIVE.load(Ordering::Acquire) {
            self.total_samples += data.len() as u64;
            
            // 发送数据
            self.send_data_packet(data);
        }
    }
    
    fn write_session_header(&self) {
        let header = LogHeader {
            session_id: self.session_id,
            timestamp: self.start_time,
            sample_rate: SAMPLE_RATE_HZ,
            channels: MAX_CHANNELS as u8,
            data_format: DataFormat::Compressed,
        };
        
        let header_bytes = header.to_bytes();
        self.send_control_packet(PacketType::SessionStart, &header_bytes);
    }
    
    fn write_session_footer(&self) {
        let footer = LogFooter {
            session_id: self.session_id,
            total_samples: self.total_samples,
            dropped_samples: self.dropped_samples,
            compression_ratio: self.compression_ratio,
            duration_ms: get_system_time_ms() - self.start_time,
        };
        
        let footer_bytes = footer.to_bytes();
        self.send_control_packet(PacketType::SessionEnd, &footer_bytes);
    }
    
    fn send_data_packet(&self, data: &[u8]) {
        let packet = DataPacket {
            packet_type: PacketType::Data,
            sequence: SAMPLES_COLLECTED.load(Ordering::Acquire),
            timestamp: get_system_time_ms(),
            data_length: data.len() as u16,
            checksum: calculate_checksum(data),
        };
        
        // 发送数据包头部
        let header_bytes = packet.to_bytes();
        self.transmit_bytes(&header_bytes);
        
        // 发送数据
        self.transmit_bytes(data);
    }
    
    fn send_control_packet(&self, packet_type: PacketType, data: &[u8]) {
        let packet = ControlPacket {
            packet_type,
            timestamp: get_system_time_ms(),
            data_length: data.len() as u16,
            checksum: calculate_checksum(data),
        };
        
        let header_bytes = packet.to_bytes();
        self.transmit_bytes(&header_bytes);
        self.transmit_bytes(data);
    }
    
    fn transmit_bytes(&self, data: &[u8]) {
        // 使用DMA发送数据
        unsafe {
            let len = data.len().min(COMPRESSION_BUFFER.len());
            COMPRESSION_BUFFER[..len].copy_from_slice(&data[..len]);
            
            free(|cs| {
                if let Some(ref mut transfer) = G_UART_TX.borrow(cs).borrow_mut().as_mut() {
                    transfer.start(len as u16);
                }
            });
        }
    }
}

/// 数据处理器
pub struct DataProcessor {
    filter_enabled: bool,
    calibration_data: [f32; MAX_CHANNELS],
    processing_buffer: Vec<u8, 512>,
}

impl DataProcessor {
    pub fn new() -> Self {
        Self {
            filter_enabled: true,
            calibration_data: [1.0; MAX_CHANNELS],
            processing_buffer: Vec::new(),
        }
    }
    
    pub fn process_buffer(&mut self, buffer: &[u8], size: usize) -> Vec<u8, 512> {
        self.processing_buffer.clear();
        
        let data = &buffer[..size];
        
        if self.filter_enabled {
            self.apply_digital_filter(data)
        } else {
            // 直接复制数据
            for &byte in data.iter().take(self.processing_buffer.capacity()) {
                let _ = self.processing_buffer.push(byte);
            }
            self.processing_buffer.clone()
        }
    }
    
    fn apply_digital_filter(&mut self, data: &[u8]) -> Vec<u8, 512> {
        // 简单的移动平均滤波器
        let window_size = 4;
        let mut filtered = Vec::new();
        
        for chunk in data.chunks(window_size) {
            let sum: u32 = chunk.iter().map(|&x| x as u32).sum();
            let avg = (sum / chunk.len() as u32) as u8;
            let _ = filtered.push(avg);
            
            if filtered.len() >= filtered.capacity() {
                break;
            }
        }
        
        self.processing_buffer = filtered.clone();
        filtered
    }
    
    pub fn set_calibration(&mut self, channel: usize, factor: f32) {
        if channel < MAX_CHANNELS {
            self.calibration_data[channel] = factor;
        }
    }
    
    pub fn enable_filter(&mut self, enabled: bool) {
        self.filter_enabled = enabled;
    }
}

/// 压缩引擎
pub struct CompressionEngine {
    compression_enabled: bool,
    compression_level: u8,
    dictionary: [u8; 256],
}

impl CompressionEngine {
    pub fn new() -> Self {
        Self {
            compression_enabled: true,
            compression_level: 3,
            dictionary: [0; 256],
        }
    }
    
    pub fn compress(&mut self, data: &[u8]) -> Vec<u8, 512> {
        if !self.compression_enabled || data.is_empty() {
            let mut result = Vec::new();
            for &byte in data.iter().take(result.capacity()) {
                let _ = result.push(byte);
            }
            return result;
        }
        
        // 简单的RLE压缩
        self.run_length_encode(data)
    }
    
    fn run_length_encode(&self, data: &[u8]) -> Vec<u8, 512> {
        let mut compressed = Vec::new();
        let mut i = 0;
        
        while i < data.len() && compressed.len() < compressed.capacity() - 2 {
            let current_byte = data[i];
            let mut count = 1u8;
            
            // 计算连续相同字节的数量
            while i + count as usize < data.len() && 
                  data[i + count as usize] == current_byte && 
                  count < 255 {
                count += 1;
            }
            
            if count > 3 {
                // 使用RLE编码
                let _ = compressed.push(0xFF); // 转义字符
                let _ = compressed.push(count);
                let _ = compressed.push(current_byte);
            } else {
                // 直接存储
                for _ in 0..count {
                    if compressed.len() < compressed.capacity() {
                        let _ = compressed.push(current_byte);
                    }
                }
            }
            
            i += count as usize;
        }
        
        compressed
    }
    
    pub fn set_compression_level(&mut self, level: u8) {
        self.compression_level = level.min(9);
    }
    
    pub fn enable_compression(&mut self, enabled: bool) {
        self.compression_enabled = enabled;
    }
}

/// DMA接收中断处理
#[interrupt]
fn DMA2_STREAM2() {
    free(|cs| {
        if let Some(ref mut transfer) = G_UART_RX.borrow(cs).borrow_mut().as_mut() {
            if transfer.is_half_complete() {
                transfer.clear_half_transfer_interrupt();
                
                // 处理前半部分数据
                BYTES_LOGGED.store(LOG_BUFFER_SIZE / 2, Ordering::Release);
                BUFFER_READY.store(true, Ordering::Release);
                
                unsafe {
                    PERFORMANCE_STATS.half_transfers += 1;
                }
            }
            
            if transfer.is_complete() {
                transfer.clear_transfer_complete_interrupt();
                
                // 处理完整缓冲区
                BYTES_LOGGED.store(LOG_BUFFER_SIZE, Ordering::Release);
                BUFFER_READY.store(true, Ordering::Release);
                
                // 切换缓冲区
                let current = CURRENT_BUFFER.load(Ordering::Acquire);
                CURRENT_BUFFER.store(!current, Ordering::Release);
                
                unsafe {
                    PERFORMANCE_STATS.full_transfers += 1;
                }
            }
        }
    });
}

/// DMA发送完成中断
#[interrupt]
fn DMA2_STREAM7() {
    free(|cs| {
        if let Some(ref mut transfer) = G_UART_TX.borrow(cs).borrow_mut().as_mut() {
            if transfer.is_complete() {
                transfer.clear_transfer_complete_interrupt();
                
                unsafe {
                    PERFORMANCE_STATS.tx_transfers += 1;
                }
            }
        }
    });
}

/// 定时器中断 - 采样控制
#[interrupt]
fn TIM2() {
    // 清除中断标志
    unsafe {
        (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit());
    }
    
    // 更新采样计数
    SAMPLES_COLLECTED.fetch_add(1, Ordering::Relaxed);
    
    unsafe {
        PERFORMANCE_STATS.timer_interrupts += 1;
    }
}

/// 输出性能统计
fn output_performance_stats() {
    unsafe {
        PERFORMANCE_STATS.calculate_rates();
        // 这里可以通过其他方式输出统计信息
    }
}

/// 检查系统健康状态
fn check_system_health() {
    unsafe {
        let stats = &PERFORMANCE_STATS;
        
        // 检查缓冲区溢出
        if stats.buffer_overruns > 10 {
            // 系统过载，需要降低采样率或增加处理能力
        }
        
        // 检查传输错误率
        let error_rate = stats.transmission_errors as f32 / stats.tx_transfers as f32;
        if error_rate > 0.01 {
            // 传输错误率过高
        }
    }
}

// 数据结构定义
#[repr(C, packed)]
pub struct LogHeader {
    pub session_id: u32,
    pub timestamp: u32,
    pub sample_rate: u32,
    pub channels: u8,
    pub data_format: DataFormat,
}

#[repr(C, packed)]
pub struct LogFooter {
    pub session_id: u32,
    pub total_samples: u64,
    pub dropped_samples: u32,
    pub compression_ratio: f32,
    pub duration_ms: u32,
}

#[repr(C, packed)]
pub struct DataPacket {
    pub packet_type: PacketType,
    pub sequence: u32,
    pub timestamp: u32,
    pub data_length: u16,
    pub checksum: u16,
}

#[repr(C, packed)]
pub struct ControlPacket {
    pub packet_type: PacketType,
    pub timestamp: u32,
    pub data_length: u16,
    pub checksum: u16,
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum PacketType {
    Data = 0x01,
    SessionStart = 0x02,
    SessionEnd = 0x03,
    Error = 0x04,
    Status = 0x05,
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum DataFormat {
    Raw = 0x00,
    Compressed = 0x01,
    Encrypted = 0x02,
}

/// 性能统计
#[derive(Debug)]
pub struct PerformanceStats {
    pub half_transfers: u32,
    pub full_transfers: u32,
    pub tx_transfers: u32,
    pub timer_interrupts: u32,
    pub buffers_processed: u32,
    pub buffer_overruns: u32,
    pub transmission_errors: u32,
    pub samples_per_second: f32,
    pub bytes_per_second: f32,
    pub last_update_time: u32,
}

impl PerformanceStats {
    pub const fn new() -> Self {
        Self {
            half_transfers: 0,
            full_transfers: 0,
            tx_transfers: 0,
            timer_interrupts: 0,
            buffers_processed: 0,
            buffer_overruns: 0,
            transmission_errors: 0,
            samples_per_second: 0.0,
            bytes_per_second: 0.0,
            last_update_time: 0,
        }
    }
    
    pub fn calculate_rates(&mut self) {
        let current_time = get_system_time_ms();
        let elapsed = current_time - self.last_update_time;
        
        if elapsed > 0 {
            self.samples_per_second = (self.timer_interrupts as f32 * 1000.0) / elapsed as f32;
            self.bytes_per_second = (self.buffers_processed as f32 * LOG_BUFFER_SIZE as f32 * 1000.0) / elapsed as f32;
            self.last_update_time = current_time;
        }
    }
}

// 辅助函数实现
impl LogHeader {
    pub fn to_bytes(&self) -> [u8; core::mem::size_of::<Self>()] {
        unsafe { core::mem::transmute(*self) }
    }
}

impl LogFooter {
    pub fn to_bytes(&self) -> [u8; core::mem::size_of::<Self>()] {
        unsafe { core::mem::transmute(*self) }
    }
}

impl DataPacket {
    pub fn to_bytes(&self) -> [u8; core::mem::size_of::<Self>()] {
        unsafe { core::mem::transmute(*self) }
    }
}

impl ControlPacket {
    pub fn to_bytes(&self) -> [u8; core::mem::size_of::<Self>()] {
        unsafe { core::mem::transmute(*self) }
    }
}

fn generate_session_id() -> u32 {
    // 简单的会话ID生成
    get_system_time_ms()
}

fn calculate_checksum(data: &[u8]) -> u16 {
    // 简单的校验和计算
    data.iter().map(|&x| x as u16).sum()
}

fn get_system_time_ms() -> u32 {
    // 这里应该实现实际的系统时间获取
    0
}