//! 基础DMA串口传输示例
//!
//! 本示例演示如何使用DMA进行串口数据传输，包括：
//! - DMA初始化和配置
//! - 内存到外设的传输（发送）
//! - 外设到内存的传输（接收）
//! - 传输完成中断处理

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
  dma::{
    config::DmaConfig, MemoryToPeripheral, PeripheralToMemory, Stream2, Stream5, Stream7,
    StreamsTuple, Transfer,
  },
  gpio::{Alternate, Pin},
  pac::{self, interrupt, DMA2, USART1},
  prelude::*,
  rcc::RccExt,
  serial::{Config, Rx, Serial, Tx},
};

// 全局DMA传输句柄
type TxTransfer = Transfer<Stream7<DMA2>, Tx<USART1>, MemoryToPeripheral, &'static mut [u8]>;
type RxTransfer = Transfer<Stream2<DMA2>, Rx<USART1>, PeripheralToMemory, &'static mut [u8]>;

static G_TX_TRANSFER: Mutex<RefCell<Option<TxTransfer>>> = Mutex::new(RefCell::new(None));
static G_RX_TRANSFER: Mutex<RefCell<Option<RxTransfer>>> = Mutex::new(RefCell::new(None));

// 静态缓冲区
static mut TX_BUFFER: [u8; 256] = [0; 256];
static mut RX_BUFFER: [u8; 256] = [0; 256];

// 传输状态
static mut TX_COMPLETE: bool = false;
static mut RX_COMPLETE: bool = false;
static mut BYTES_RECEIVED: usize = 0;

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

  // 配置DMA传输
  setup_dma_transfers(dma2.7, dma2.2, tx, rx);

  // 启用DMA中断
  unsafe {
    pac::NVIC::unmask(interrupt::DMA2_STREAM7); // TX完成中断
    pac::NVIC::unmask(interrupt::DMA2_STREAM2); // RX完成中断
  }

  // 准备发送数据
  let message = b"Hello from DMA UART!\r\n";
  unsafe {
    TX_BUFFER[..message.len()].copy_from_slice(message);
  }

  // 开始DMA传输
  start_dma_transmission(message.len());
  start_dma_reception(RX_BUFFER.len());

  let mut delay = cp.SYST.delay(&clocks);

  loop {
    // 检查传输状态
    unsafe {
      if TX_COMPLETE {
        TX_COMPLETE = false;

        // 准备下一次传输
        delay.delay_ms(1000u32);

        let counter_msg = format_message(get_counter());
        start_dma_transmission(counter_msg.len());
      }

      if RX_COMPLETE {
        RX_COMPLETE = false;

        // 处理接收到的数据
        process_received_data(&RX_BUFFER[..BYTES_RECEIVED]);

        // 重新开始接收
        start_dma_reception(RX_BUFFER.len());
      }
    }

    delay.delay_ms(10u32);
  }
}

/// 设置DMA传输
fn setup_dma_transfers(
  tx_stream: Stream7<DMA2>,
  rx_stream: Stream2<DMA2>,
  tx: Tx<USART1>,
  rx: Rx<USART1>,
) {
  // 配置发送DMA
  let tx_config = DmaConfig::default()
    .memory_increment(true)
    .peripheral_increment(false)
    .transfer_complete_interrupt(true);

  let tx_transfer =
    Transfer::init_memory_to_peripheral(tx_stream, tx, unsafe { &mut TX_BUFFER }, None, tx_config);

  // 配置接收DMA
  let rx_config = DmaConfig::default()
    .memory_increment(true)
    .peripheral_increment(false)
    .transfer_complete_interrupt(true);

  let rx_transfer =
    Transfer::init_peripheral_to_memory(rx_stream, rx, unsafe { &mut RX_BUFFER }, None, rx_config);

  // 存储到全局变量
  free(|cs| {
    G_TX_TRANSFER.borrow(cs).replace(Some(tx_transfer));
    G_RX_TRANSFER.borrow(cs).replace(Some(rx_transfer));
  });
}

/// 开始DMA发送传输
fn start_dma_transmission(length: usize) {
  free(|cs| {
    if let Some(ref mut transfer) = G_TX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
      transfer.start(length as u16);
    }
  });
}

/// 开始DMA接收传输
fn start_dma_reception(length: usize) {
  free(|cs| {
    if let Some(ref mut transfer) = G_RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
      transfer.start(length as u16);
    }
  });
}

/// 格式化消息
fn format_message(counter: u32) -> &'static [u8] {
  static mut MESSAGE_BUFFER: [u8; 64] = [0; 64];

  unsafe {
    let mut cursor = heapless::Vec::<u8, 64>::new();
    write!(cursor, "Counter: {}\r\n", counter).unwrap();

    let len = cursor.len();
    MESSAGE_BUFFER[..len].copy_from_slice(&cursor);
    TX_BUFFER[..len].copy_from_slice(&cursor);

    &MESSAGE_BUFFER[..len]
  }
}

/// 获取计数器值
fn get_counter() -> u32 {
  static mut COUNTER: u32 = 0;
  unsafe {
    COUNTER += 1;
    COUNTER
  }
}

/// 处理接收到的数据
fn process_received_data(data: &[u8]) {
  // 简单的回显处理
  if !data.is_empty() {
    unsafe {
      // 将接收到的数据复制到发送缓冲区
      let len = data.len().min(TX_BUFFER.len());
      TX_BUFFER[..len].copy_from_slice(&data[..len]);

      // 开始回显传输
      start_dma_transmission(len);
    }
  }
}

/// DMA发送完成中断
#[interrupt]
fn DMA2_STREAM7() {
  free(|cs| {
    if let Some(ref mut transfer) = G_TX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
      if transfer.is_complete() {
        transfer.clear_transfer_complete_interrupt();
        unsafe {
          TX_COMPLETE = true;
        }
      }
    }
  });
}

/// DMA接收完成中断
#[interrupt]
fn DMA2_STREAM2() {
  free(|cs| {
    if let Some(ref mut transfer) = G_RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
      if transfer.is_complete() {
        transfer.clear_transfer_complete_interrupt();

        // 获取实际接收的字节数
        let remaining = transfer.get_number_of_transfers();
        let total = unsafe { RX_BUFFER.len() };

        unsafe {
          BYTES_RECEIVED = total - remaining as usize;
          RX_COMPLETE = true;
        }
      }
    }
  });
}

/// DMA错误处理
pub fn handle_dma_error() {
  // 重置DMA传输
  free(|cs| {
    // 重新初始化传输
    if let Some(ref mut tx_transfer) = G_TX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
      tx_transfer.clear_all_interrupts();
    }

    if let Some(ref mut rx_transfer) = G_RX_TRANSFER.borrow(cs).borrow_mut().as_mut() {
      rx_transfer.clear_all_interrupts();
    }
  });
}

/// 获取DMA传输统计信息
pub struct DmaStats {
  pub tx_transfers: u32,
  pub rx_transfers: u32,
  pub tx_bytes: u32,
  pub rx_bytes: u32,
  pub errors: u32,
}

impl DmaStats {
  pub fn new() -> Self {
    Self {
      tx_transfers: 0,
      rx_transfers: 0,
      tx_bytes: 0,
      rx_bytes: 0,
      errors: 0,
    }
  }

  pub fn record_tx_transfer(&mut self, bytes: u32) {
    self.tx_transfers += 1;
    self.tx_bytes += bytes;
  }

  pub fn record_rx_transfer(&mut self, bytes: u32) {
    self.rx_transfers += 1;
    self.rx_bytes += bytes;
  }

  pub fn record_error(&mut self) {
    self.errors += 1;
  }

  pub fn get_throughput(&self, elapsed_ms: u32) -> (f32, f32) {
    if elapsed_ms == 0 {
      return (0.0, 0.0);
    }

    let tx_kbps = (self.tx_bytes as f32 * 8.0) / (elapsed_ms as f32);
    let rx_kbps = (self.rx_bytes as f32 * 8.0) / (elapsed_ms as f32);

    (tx_kbps, rx_kbps)
  }
}

/// DMA配置辅助函数
pub struct DmaHelper;

impl DmaHelper {
  /// 计算最优的DMA传输大小
  pub fn calculate_optimal_transfer_size(data_size: usize, max_chunk: usize) -> usize {
    if data_size <= max_chunk {
      data_size
    } else {
      // 选择能整除数据大小的最大块大小
      for chunk_size in (1..=max_chunk).rev() {
        if data_size % chunk_size == 0 {
          return chunk_size;
        }
      }
      max_chunk
    }
  }

  /// 检查内存对齐
  pub fn is_memory_aligned(addr: usize, alignment: usize) -> bool {
    addr % alignment == 0
  }

  /// 获取DMA传输优先级建议
  pub fn get_priority_recommendation(data_type: DataType) -> DmaPriority {
    match data_type {
      DataType::RealTimeControl => DmaPriority::VeryHigh,
      DataType::AudioVideo => DmaPriority::High,
      DataType::NetworkData => DmaPriority::Medium,
      DataType::FileTransfer => DmaPriority::Low,
    }
  }
}

#[derive(Debug, Clone, Copy)]
pub enum DataType {
  RealTimeControl,
  AudioVideo,
  NetworkData,
  FileTransfer,
}

#[derive(Debug, Clone, Copy)]
pub enum DmaPriority {
  Low,
  Medium,
  High,
  VeryHigh,
}
