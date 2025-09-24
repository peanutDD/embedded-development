//! 循环DMA串口传输示例
//! 
//! 本示例演示如何使用循环DMA模式进行连续的串口数据传输：
//! - 循环缓冲区配置
//! - 半传输和全传输中断处理
//! - 连续数据流处理
//! - 缓冲区溢出检测和处理

#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::interrupt::{free, Mutex};
use stm32f4xx_hal::{
    dma::{
        config::DmaConfig,
        MemoryToPeripheral, PeripheralToMemory,
        Stream2, Stream5, Stream7,
        StreamsTuple, Transfer,
    },
    gpio::{Alternate, Pin},
    pac::{self, interrupt, DMA2, USART1},
    prelude::*,
    rcc::RccExt,
    serial::{Config, Serial, Rx, Tx},
};
use core::cell::RefCell;
use heapless::{
    pool::{Pool, Node},
    spsc::{Queue, Producer, Consumer},
};

// 循环缓冲区大小
const CIRCULAR_BUFFER_SIZE: usize = 512;
const PROCESSING_BUFFER_SIZE: usize = 256;

// DMA传输类型定义
type TxTransfer = Transfer<Stream7<DMA2>, Tx<USART1>, MemoryToPeripheral, &'static mut [u8]>;
type RxTransfer = Transfer<Stream2<DMA2>, Rx<USART1>, PeripheralToMemory, &'static mut [u8]>;

// 全局DMA传输句柄
static G_TX_TRANSFER: Mutex<RefCell<Option<TxTransfer>>> = Mutex::new(RefCell::new(None));
static G_RX_TRANSFER: Mutex<RefCell<Option<RxTransfer>>> = Mutex::new(RefCell::new(None));

// 循环缓冲区
static mut RX_CIRCULAR_BUFFER: [u8; CIRCULAR_BUFFER_SIZE] = [0; CIRCULAR_BUFFER_SIZE];
static mut TX_CIRCULAR_BUFFER: [u8; CIRCULAR_BUFFER_SIZE] = [0; CIRCULAR_BUFFER_SIZE];

// 数据处理队列
static mut RX_QUEUE: Queue<u8, 1024> = Queue::new();
static mut TX_QUEUE: Queue<u8, 1024> = Queue::new();

// 缓冲区状态
static mut RX_HEAD: usize = 0;
static mut RX_TAIL: usize = 0;
static mut TX_HEAD: usize = 0;
static mut TX_TAIL: usize = 0;

// 统计信息
static mut STATS: CircularDmaStats = CircularDmaStats::new();

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
        .baudrate(115200.bps())
        .wordlength_8()
        .parity_none()
        .stopbits(stm32f4xx_hal::serial::StopBits::STOP1);

    let serial = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
    let (tx, rx) = serial.split();

    // 初始化DMA
    let dma2 = StreamsTuple::new(dp.DMA2);

    // 设置循环DMA传输
    setup_circular_dma(dma2.7, dma2.2, tx, rx);

    // 启用DMA中断
    unsafe {
        pac::NVIC::unmask(interrupt::DMA2_STREAM7); // TX中断
        pac::NVIC::unmask(interrupt::DMA2_STREAM2); // RX中断
    }

    // 开始循环接收
    start_circular_reception();

    let mut delay = cp.SYST.delay(&clocks);
    let mut message_counter = 0u32;

    // 初始化队列
    let (mut rx_producer, mut rx_consumer) = unsafe { RX_QUEUE.split() };
    let (mut tx_producer, mut tx_consumer) = unsafe { TX_QUEUE.split() };

    loop {
        // 处理接收到的数据
        process_received_data(&mut rx_consumer, &mut tx_producer);

        // 处理发送队列
        process_transmit_queue(&mut tx_consumer);

        // 定期发送测试消息
        if message_counter % 1000 == 0 {
            send_test_message(&mut tx_producer, message_counter / 1000);
        }

        // 检查缓冲区状态
        check_buffer_health();

        message_counter += 1;
        delay.delay_ms(1u32);
    }
}

/// 设置循环DMA传输
fn setup_circular_dma(
    tx_stream: Stream7<DMA2>,
    rx_stream: Stream2<DMA2>,
    tx: Tx<USART1>,
    rx: Rx<USART1>,
) {
    // 配置接收DMA（循环模式）
    let rx_config = DmaConfig::default()
        .memory_increment(true)
        .peripheral_increment(false)
        .circular_buffer(true)
        .half_transfer_interrupt(true)
        .transfer_complete_interrupt(true);

    let rx_transfer = Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        unsafe { &mut RX_CIRCULAR_BUFFER },
        None,
        rx_config,
    );

    // 配置发送DMA（普通模式，按需启动）
    let tx_config = DmaConfig::default()
        .memory_increment(true)
        .peripheral_increment(false)
        .transfer_complete_interrupt(true);

    let tx_transfer = Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        unsafe { &mut TX_CIRCULAR_BUFFER },
        None,
        tx_config,
    );

    // 存储到全局变量
    free(|cs| {
        G_TX_TRANSFER.borrow(cs).replace(Some(tx_transfer));
        G_RX_TRANSFER.borrow(cs).replace(Some(rx_transfer));
    });
}

/// 开始循环接收
fn start_circular_reception() {
    free(|cs| {
        if let Some(ref mut transfer) = G_RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            transfer.start(CIRCULAR_BUFFER_SIZE as u16);
        }
    });
}

/// 处理接收到的数据
fn process_received_data(
    rx_consumer: &mut Consumer<u8, 1024>,
    tx_producer: &mut Producer<u8, 1024>,
) {
    // 从循环缓冲区读取新数据
    let new_data = get_new_received_data();
    
    for &byte in new_data {
        // 简单的回显处理
        let _ = tx_producer.enqueue(byte);
        
        // 更新统计信息
        unsafe {
            STATS.bytes_received += 1;
        }
    }
}

/// 获取新接收到的数据
fn get_new_received_data() -> &'static [u8] {
    unsafe {
        // 获取DMA当前位置
        let current_pos = get_dma_current_position();
        
        if current_pos != RX_TAIL {
            let data = if current_pos > RX_TAIL {
                // 数据在尾部到当前位置之间
                &RX_CIRCULAR_BUFFER[RX_TAIL..current_pos]
            } else {
                // 数据跨越了缓冲区边界，先处理到末尾的部分
                &RX_CIRCULAR_BUFFER[RX_TAIL..]
            };
            
            RX_TAIL = current_pos;
            data
        } else {
            &[]
        }
    }
}

/// 获取DMA当前位置
fn get_dma_current_position() -> usize {
    free(|cs| {
        if let Some(ref transfer) = G_RX_TRANSFER.borrow(cs).borrow().as_ref() {
            let remaining = transfer.get_number_of_transfers() as usize;
            CIRCULAR_BUFFER_SIZE - remaining
        } else {
            0
        }
    })
}

/// 处理发送队列
fn process_transmit_queue(tx_consumer: &mut Consumer<u8, 1024>) {
    // 检查是否有数据需要发送且DMA空闲
    if !tx_consumer.is_empty() && is_tx_dma_idle() {
        let mut tx_count = 0;
        
        unsafe {
            // 从队列中取出数据到发送缓冲区
            while tx_count < CIRCULAR_BUFFER_SIZE && !tx_consumer.is_empty() {
                if let Some(byte) = tx_consumer.dequeue() {
                    TX_CIRCULAR_BUFFER[tx_count] = byte;
                    tx_count += 1;
                }
            }
            
            if tx_count > 0 {
                start_dma_transmission(tx_count);
                STATS.bytes_transmitted += tx_count as u32;
            }
        }
    }
}

/// 检查发送DMA是否空闲
fn is_tx_dma_idle() -> bool {
    free(|cs| {
        if let Some(ref transfer) = G_TX_TRANSFER.borrow(cs).borrow().as_ref() {
            transfer.is_complete()
        } else {
            true
        }
    })
}

/// 开始DMA发送传输
fn start_dma_transmission(length: usize) {
    free(|cs| {
        if let Some(ref mut transfer) = G_TX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            transfer.start(length as u16);
        }
    });
}

/// 发送测试消息
fn send_test_message(tx_producer: &mut Producer<u8, 1024>, counter: u32) {
    let message = format_test_message(counter);
    
    for &byte in message {
        let _ = tx_producer.enqueue(byte);
    }
}

/// 格式化测试消息
fn format_test_message(counter: u32) -> &'static [u8] {
    static mut MESSAGE_BUFFER: [u8; 64] = [0; 64];
    
    unsafe {
        let mut cursor = heapless::Vec::<u8, 64>::new();
        use core::fmt::Write;
        write!(cursor, "Circular DMA Test #{}\r\n", counter).unwrap();
        
        let len = cursor.len();
        MESSAGE_BUFFER[..len].copy_from_slice(&cursor);
        &MESSAGE_BUFFER[..len]
    }
}

/// 检查缓冲区健康状态
fn check_buffer_health() {
    unsafe {
        // 检查接收缓冲区是否接近满
        let rx_usage = calculate_buffer_usage(RX_HEAD, RX_TAIL, CIRCULAR_BUFFER_SIZE);
        if rx_usage > 0.8 {
            STATS.buffer_overruns += 1;
            // 可以在这里实现缓冲区清理逻辑
        }
        
        // 更新统计信息
        STATS.max_buffer_usage = STATS.max_buffer_usage.max(rx_usage);
    }
}

/// 计算缓冲区使用率
fn calculate_buffer_usage(head: usize, tail: usize, size: usize) -> f32 {
    let used = if head >= tail {
        head - tail
    } else {
        size - tail + head
    };
    
    used as f32 / size as f32
}

/// DMA接收中断处理（半传输）
#[interrupt]
fn DMA2_STREAM2() {
    free(|cs| {
        if let Some(ref mut transfer) = G_RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            // 检查是否是半传输中断
            if transfer.is_half_complete() {
                transfer.clear_half_transfer_complete_interrupt();
                
                unsafe {
                    // 处理前半部分数据
                    RX_HEAD = CIRCULAR_BUFFER_SIZE / 2;
                    STATS.half_transfers += 1;
                }
            }
            
            // 检查是否是全传输中断
            if transfer.is_complete() {
                transfer.clear_transfer_complete_interrupt();
                
                unsafe {
                    // 处理后半部分数据
                    RX_HEAD = 0;
                    STATS.full_transfers += 1;
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
                
                unsafe {
                    STATS.tx_transfers += 1;
                }
            }
        }
    });
}

/// 循环DMA统计信息
#[derive(Debug)]
pub struct CircularDmaStats {
    pub bytes_received: u32,
    pub bytes_transmitted: u32,
    pub half_transfers: u32,
    pub full_transfers: u32,
    pub tx_transfers: u32,
    pub buffer_overruns: u32,
    pub max_buffer_usage: f32,
}

impl CircularDmaStats {
    pub const fn new() -> Self {
        Self {
            bytes_received: 0,
            bytes_transmitted: 0,
            half_transfers: 0,
            full_transfers: 0,
            tx_transfers: 0,
            buffer_overruns: 0,
            max_buffer_usage: 0.0,
        }
    }
    
    pub fn reset(&mut self) {
        *self = Self::new();
    }
    
    pub fn get_total_transfers(&self) -> u32 {
        self.half_transfers + self.full_transfers + self.tx_transfers
    }
    
    pub fn get_error_rate(&self) -> f32 {
        if self.bytes_received > 0 {
            self.buffer_overruns as f32 / self.bytes_received as f32
        } else {
            0.0
        }
    }
}

/// 循环缓冲区管理器
pub struct CircularBufferManager {
    buffer: &'static mut [u8],
    head: usize,
    tail: usize,
    size: usize,
}

impl CircularBufferManager {
    pub fn new(buffer: &'static mut [u8]) -> Self {
        let size = buffer.len();
        Self {
            buffer,
            head: 0,
            tail: 0,
            size,
        }
    }
    
    pub fn write(&mut self, data: &[u8]) -> usize {
        let mut written = 0;
        
        for &byte in data {
            if self.is_full() {
                break;
            }
            
            self.buffer[self.head] = byte;
            self.head = (self.head + 1) % self.size;
            written += 1;
        }
        
        written
    }
    
    pub fn read(&mut self, buffer: &mut [u8]) -> usize {
        let mut read_count = 0;
        
        for i in 0..buffer.len() {
            if self.is_empty() {
                break;
            }
            
            buffer[i] = self.buffer[self.tail];
            self.tail = (self.tail + 1) % self.size;
            read_count += 1;
        }
        
        read_count
    }
    
    pub fn available(&self) -> usize {
        if self.head >= self.tail {
            self.head - self.tail
        } else {
            self.size - self.tail + self.head
        }
    }
    
    pub fn free_space(&self) -> usize {
        self.size - self.available() - 1 // 保留一个字节区分满和空
    }
    
    pub fn is_empty(&self) -> bool {
        self.head == self.tail
    }
    
    pub fn is_full(&self) -> bool {
        (self.head + 1) % self.size == self.tail
    }
    
    pub fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
    }
    
    pub fn usage_percentage(&self) -> f32 {
        (self.available() as f32 / (self.size - 1) as f32) * 100.0
    }
}