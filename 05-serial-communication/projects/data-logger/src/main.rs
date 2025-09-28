//! 高性能数据记录器
//!
//! 本项目实现了一个高性能的数据记录系统，支持：
//! - 多通道数据采集
//! - 实时数据压缩
//! - 串口数据传输
//! - 数据完整性校验
//! - 可配置的采样率和存储格式

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicUsize, Ordering};
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use heapless::{
  spsc::{Consumer, Producer, Queue},
  Vec,
};
use panic_halt as _;
use stm32f4xx_hal::{
  adc::{config::AdcConfig, config::SampleTime, Adc},
  dma::{
    config::DmaConfig, MemoryToPeripheral, PeripheralToMemory, Stream0, Stream1, Stream2, Stream7,
    StreamsTuple, Transfer,
  },
  gpio::{Alternate, Analog, Pin},
  pac::{self, interrupt, ADC1, DMA1, DMA2, TIM2, USART1},
  prelude::*,
  rcc::RccExt,
  serial::{Config, Rx, Serial, Tx},
  timer::{Event, Timer},
};

// 系统配置常量
const SAMPLE_RATE_HZ: u32 = 10000; // 10kHz采样率
const ADC_CHANNELS: usize = 8;
const BUFFER_SIZE: usize = 1024;
const COMPRESSION_BUFFER_SIZE: usize = 512;
const UART_BAUD_RATE: u32 = 921600; // 高速串口
const DATA_PACKET_SIZE: usize = 64;
const MAX_QUEUE_SIZE: usize = 16;

// DMA传输类型定义
type AdcDmaTransfer = Transfer<Stream0<DMA2>, Adc<ADC1>, PeripheralToMemory, &'static mut [u16]>;
type UartTxTransfer = Transfer<Stream7<DMA2>, Tx<USART1>, MemoryToPeripheral, &'static mut [u8]>;

// 全局DMA传输句柄
static G_ADC_DMA: Mutex<RefCell<Option<AdcDmaTransfer>>> = Mutex::new(RefCell::new(None));
static G_UART_TX: Mutex<RefCell<Option<UartTxTransfer>>> = Mutex::new(RefCell::new(None));
static G_UART_RX: Mutex<RefCell<Option<Rx<USART1>>>> = Mutex::new(RefCell::new(None));

// 数据缓冲区
static mut ADC_BUFFER_A: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut ADC_BUFFER_B: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut COMPRESSION_BUFFER: [u8; COMPRESSION_BUFFER_SIZE] = [0; COMPRESSION_BUFFER_SIZE];
static mut UART_TX_BUFFER: [u8; DATA_PACKET_SIZE] = [0; DATA_PACKET_SIZE];

// 系统状态
static LOGGING_ACTIVE: AtomicBool = AtomicBool::new(false);
static CURRENT_ADC_BUFFER: AtomicBool = AtomicBool::new(false); // false = A, true = B
static ADC_BUFFER_READY: AtomicBool = AtomicBool::new(false);
static SAMPLES_COLLECTED: AtomicU32 = AtomicU32::new(0);
static PACKETS_SENT: AtomicU32 = AtomicU32::new(0);
static COMPRESSION_RATIO: AtomicU16 = AtomicU16::new(100); // 百分比

// 性能统计
static mut PERFORMANCE_METRICS: PerformanceMetrics = PerformanceMetrics::new();

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
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 串口引脚
  let tx_pin = gpioa.pa9.into_alternate::<7>();
  let rx_pin = gpioa.pa10.into_alternate::<7>();

  // ADC引脚 - 8个模拟输入通道
  let adc_pins = [
    gpioa.pa0.into_analog(),
    gpioa.pa1.into_analog(),
    gpioa.pa2.into_analog(),
    gpioa.pa3.into_analog(),
    gpioa.pa4.into_analog(),
    gpioa.pa5.into_analog(),
    gpioa.pa6.into_analog(),
    gpioa.pa7.into_analog(),
  ];

  // 配置高速串口
  let config = Config::default()
    .baudrate(UART_BAUD_RATE.bps())
    .wordlength_8()
    .parity_none()
    .stopbits(stm32f4xx_hal::serial::StopBits::STOP1);

  let serial = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();
  let (tx, rx) = serial.split();

  // 配置ADC
  let adc_config = AdcConfig::default()
    .resolution(stm32f4xx_hal::adc::config::Resolution::Twelve)
    .scan(stm32f4xx_hal::adc::config::Scan::Enabled)
    .continuous(stm32f4xx_hal::adc::config::Continuous::Single)
    .dma(stm32f4xx_hal::adc::config::Dma::Continuous);

  let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

  // 配置ADC采样时间
  for i in 0..ADC_CHANNELS {
    adc.configure_channel(
      &adc_pins[i],
      stm32f4xx_hal::adc::config::Sequence::One,
      SampleTime::Cycles_3,
    );
  }

  // 初始化DMA
  let dma1 = StreamsTuple::new(dp.DMA1);
  let dma2 = StreamsTuple::new(dp.DMA2);

  // 设置ADC DMA传输
  setup_adc_dma(dma2.0, adc);

  // 设置UART DMA传输
  setup_uart_dma(dma2.7, tx);

  // 存储UART RX到全局变量
  free(|cs| {
    G_UART_RX.borrow(cs).replace(Some(rx));
  });

  // 配置采样定时器
  let mut timer = Timer::new(dp.TIM2, &clocks).start_count_down(SAMPLE_RATE_HZ.Hz());
  timer.listen(Event::TimeOut);

  // 启用中断
  unsafe {
    pac::NVIC::unmask(interrupt::DMA2_STREAM0); // ADC DMA
    pac::NVIC::unmask(interrupt::DMA2_STREAM7); // UART TX DMA
    pac::NVIC::unmask(interrupt::USART1); // UART RX
    pac::NVIC::unmask(interrupt::TIM2); // 采样定时器
  }

  // 初始化数据处理器
  let mut data_processor = DataProcessor::new();
  let mut compression_engine = CompressionEngine::new();
  let mut packet_manager = PacketManager::new();
  let mut command_processor = CommandProcessor::new();

  let mut delay = cp.SYST.delay(&clocks);

  // 开始数据记录
  start_data_logging();

  loop {
    // 处理串口命令
    if let Some(command) = receive_command() {
      let response = command_processor.process_command(command);
      send_response(&response);
    }

    // 检查ADC缓冲区是否准备好
    if ADC_BUFFER_READY.load(Ordering::Acquire) {
      let buffer = get_ready_adc_buffer();

      // 处理ADC数据
      let processed_data = data_processor.process_adc_data(buffer);

      // 压缩数据
      let compressed_data = compression_engine.compress(&processed_data);

      // 创建数据包
      let packet = packet_manager.create_data_packet(&compressed_data);

      // 发送数据包
      send_data_packet(&packet);

      // 标记缓冲区已处理
      ADC_BUFFER_READY.store(false, Ordering::Release);

      unsafe {
        PERFORMANCE_METRICS.buffers_processed += 1;
      }
    }

    // 定期发送状态信息
    if unsafe { PERFORMANCE_METRICS.buffers_processed } % 100 == 0 {
      let status_packet = packet_manager.create_status_packet();
      send_data_packet(&status_packet);
    }

    delay.delay_us(50u32);
  }
}

/// 设置ADC DMA传输
fn setup_adc_dma(stream: Stream0<DMA2>, adc: Adc<ADC1>) {
  let config = DmaConfig::default()
    .memory_increment(true)
    .peripheral_increment(false)
    .circular_buffer(true)
    .half_transfer_interrupt(true)
    .transfer_complete_interrupt(true)
    .priority(stm32f4xx_hal::dma::config::Priority::VeryHigh);

  let transfer =
    Transfer::init_peripheral_to_memory(stream, adc, unsafe { &mut ADC_BUFFER_A }, None, config);

  free(|cs| {
    G_ADC_DMA.borrow(cs).replace(Some(transfer));
  });
}

/// 设置UART DMA传输
fn setup_uart_dma(stream: Stream7<DMA2>, tx: Tx<USART1>) {
  let config = DmaConfig::default()
    .memory_increment(true)
    .peripheral_increment(false)
    .transfer_complete_interrupt(true)
    .priority(stm32f4xx_hal::dma::config::Priority::High);

  let transfer =
    Transfer::init_memory_to_peripheral(stream, tx, unsafe { &mut UART_TX_BUFFER }, None, config);

  free(|cs| {
    G_UART_TX.borrow(cs).replace(Some(transfer));
  });
}

/// 开始数据记录
fn start_data_logging() {
  LOGGING_ACTIVE.store(true, Ordering::Release);

  // 启动ADC DMA传输
  free(|cs| {
    if let Some(ref mut transfer) = G_ADC_DMA.borrow(cs).borrow_mut().as_mut() {
      transfer.start(BUFFER_SIZE as u16);
    }
  });
}

/// 停止数据记录
fn stop_data_logging() {
  LOGGING_ACTIVE.store(false, Ordering::Release);

  // 停止ADC DMA传输
  free(|cs| {
    if let Some(ref mut transfer) = G_ADC_DMA.borrow(cs).borrow_mut().as_mut() {
      // transfer.pause();
    }
  });
}

/// 获取准备好的ADC缓冲区
fn get_ready_adc_buffer() -> &'static [u16] {
  unsafe {
    if CURRENT_ADC_BUFFER.load(Ordering::Acquire) {
      &ADC_BUFFER_B
    } else {
      &ADC_BUFFER_A
    }
  }
}

/// 发送数据包
fn send_data_packet(packet: &[u8]) {
  if packet.len() <= DATA_PACKET_SIZE {
    unsafe {
      UART_TX_BUFFER[..packet.len()].copy_from_slice(packet);

      free(|cs| {
        if let Some(ref mut transfer) = G_UART_TX.borrow(cs).borrow_mut().as_mut() {
          transfer.start(packet.len() as u16);
        }
      });
    }

    PACKETS_SENT.fetch_add(1, Ordering::Relaxed);
  }
}

/// 接收命令
fn receive_command() -> Option<Command> {
  // 简化实现 - 实际应该从串口接收并解析命令
  None
}

/// 发送响应
fn send_response(response: &[u8]) {
  send_data_packet(response);
}

/// 数据处理器
pub struct DataProcessor {
  calibration_factors: [f32; ADC_CHANNELS],
  filter_enabled: bool,
  baseline_values: [u16; ADC_CHANNELS],
}

impl DataProcessor {
  pub fn new() -> Self {
    Self {
      calibration_factors: [1.0; ADC_CHANNELS],
      filter_enabled: true,
      baseline_values: [0; ADC_CHANNELS],
    }
  }

  pub fn process_adc_data(&mut self, buffer: &[u16]) -> Vec<u8, COMPRESSION_BUFFER_SIZE> {
    let mut processed = Vec::new();

    // 按通道处理数据
    let samples_per_channel = buffer.len() / ADC_CHANNELS;

    for channel in 0..ADC_CHANNELS {
      for sample_idx in 0..samples_per_channel {
        let buffer_idx = sample_idx * ADC_CHANNELS + channel;
        if buffer_idx < buffer.len() {
          let raw_value = buffer[buffer_idx];

          // 应用校准
          let calibrated = (raw_value as f32 * self.calibration_factors[channel]) as u16;

          // 应用滤波
          let filtered = if self.filter_enabled {
            self.apply_filter(channel, calibrated)
          } else {
            calibrated
          };

          // 转换为字节
          let bytes = filtered.to_le_bytes();
          if processed.len() + 2 <= processed.capacity() {
            let _ = processed.push(bytes[0]);
            let _ = processed.push(bytes[1]);
          }
        }
      }
    }

    processed
  }

  fn apply_filter(&mut self, channel: usize, value: u16) -> u16 {
    // 简单的移动平均滤波器
    let alpha = 0.1f32;
    let filtered = self.baseline_values[channel] as f32 * (1.0 - alpha) + value as f32 * alpha;
    self.baseline_values[channel] = filtered as u16;
    filtered as u16
  }

  pub fn set_calibration(&mut self, channel: usize, factor: f32) {
    if channel < ADC_CHANNELS {
      self.calibration_factors[channel] = factor;
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
}

impl CompressionEngine {
  pub fn new() -> Self {
    Self {
      compression_enabled: true,
      compression_level: 3,
    }
  }

  pub fn compress(&mut self, data: &[u8]) -> Vec<u8, COMPRESSION_BUFFER_SIZE> {
    if !self.compression_enabled {
      let mut result = Vec::new();
      for &byte in data.iter().take(result.capacity()) {
        let _ = result.push(byte);
      }
      return result;
    }

    // 简单的差分压缩
    self.differential_compress(data)
  }

  fn differential_compress(&self, data: &[u8]) -> Vec<u8, COMPRESSION_BUFFER_SIZE> {
    let mut compressed = Vec::new();

    if data.is_empty() {
      return compressed;
    }

    // 第一个值直接存储
    let _ = compressed.push(data[0]);

    // 后续值存储差分
    for i in 1..data.len() {
      if compressed.len() >= compressed.capacity() {
        break;
      }

      let diff = data[i].wrapping_sub(data[i - 1]);
      let _ = compressed.push(diff);
    }

    // 更新压缩比
    let ratio = (compressed.len() * 100) / data.len().max(1);
    COMPRESSION_RATIO.store(ratio as u16, Ordering::Relaxed);

    compressed
  }

  pub fn set_compression_level(&mut self, level: u8) {
    self.compression_level = level.min(9);
  }

  pub fn enable_compression(&mut self, enabled: bool) {
    self.compression_enabled = enabled;
  }
}

/// 数据包管理器
pub struct PacketManager {
  sequence_number: u32,
  session_id: u16,
}

impl PacketManager {
  pub fn new() -> Self {
    Self {
      sequence_number: 0,
      session_id: generate_session_id(),
    }
  }

  pub fn create_data_packet(&mut self, data: &[u8]) -> Vec<u8, DATA_PACKET_SIZE> {
    let mut packet = Vec::new();

    // 包头
    let _ = packet.push(0xAA); // 同步字节
    let _ = packet.push(PacketType::Data as u8);

    // 序列号
    let seq_bytes = self.sequence_number.to_le_bytes();
    for &byte in &seq_bytes {
      let _ = packet.push(byte);
    }

    // 会话ID
    let session_bytes = self.session_id.to_le_bytes();
    for &byte in &session_bytes {
      let _ = packet.push(byte);
    }

    // 数据长度
    let len = (data.len() as u16).min((DATA_PACKET_SIZE - 12) as u16);
    let len_bytes = len.to_le_bytes();
    for &byte in &len_bytes {
      let _ = packet.push(byte);
    }

    // 数据
    for &byte in data.iter().take(len as usize) {
      if packet.len() < packet.capacity() - 2 {
        let _ = packet.push(byte);
      }
    }

    // 校验和
    let checksum = calculate_checksum(&packet[1..]);
    let checksum_bytes = checksum.to_le_bytes();
    for &byte in &checksum_bytes {
      let _ = packet.push(byte);
    }

    self.sequence_number = self.sequence_number.wrapping_add(1);
    packet
  }

  pub fn create_status_packet(&self) -> Vec<u8, DATA_PACKET_SIZE> {
    let mut packet = Vec::new();

    // 包头
    let _ = packet.push(0xAA);
    let _ = packet.push(PacketType::Status as u8);

    // 状态数据
    let samples = SAMPLES_COLLECTED.load(Ordering::Relaxed);
    let packets = PACKETS_SENT.load(Ordering::Relaxed);
    let compression = COMPRESSION_RATIO.load(Ordering::Relaxed);

    // 添加状态信息
    for &byte in &samples.to_le_bytes() {
      let _ = packet.push(byte);
    }
    for &byte in &packets.to_le_bytes() {
      let _ = packet.push(byte);
    }
    for &byte in &compression.to_le_bytes() {
      let _ = packet.push(byte);
    }

    // 校验和
    let checksum = calculate_checksum(&packet[1..]);
    let checksum_bytes = checksum.to_le_bytes();
    for &byte in &checksum_bytes {
      let _ = packet.push(byte);
    }

    packet
  }
}

/// 命令处理器
pub struct CommandProcessor {
  logging_active: bool,
}

impl CommandProcessor {
  pub fn new() -> Self {
    Self {
      logging_active: false,
    }
  }

  pub fn process_command(&mut self, command: Command) -> Vec<u8, DATA_PACKET_SIZE> {
    let mut response = Vec::new();

    match command {
      Command::StartLogging => {
        start_data_logging();
        self.logging_active = true;
        let _ = response.push(ResponseCode::Success as u8);
      }
      Command::StopLogging => {
        stop_data_logging();
        self.logging_active = false;
        let _ = response.push(ResponseCode::Success as u8);
      }
      Command::GetStatus => {
        let _ = response.push(ResponseCode::Success as u8);
        let _ = response.push(if self.logging_active { 1 } else { 0 });
      }
      Command::SetSampleRate(rate) => {
        // 实际实现中应该重新配置定时器
        let _ = response.push(ResponseCode::Success as u8);
      }
      Command::Calibrate(channel, factor) => {
        // 实际实现中应该设置校准参数
        let _ = response.push(ResponseCode::Success as u8);
      }
    }

    response
  }
}

/// ADC DMA中断处理
#[interrupt]
fn DMA2_STREAM0() {
  free(|cs| {
    if let Some(ref mut transfer) = G_ADC_DMA.borrow(cs).borrow_mut().as_mut() {
      if transfer.is_half_complete() {
        transfer.clear_half_transfer_interrupt();

        // 前半部分数据准备好
        ADC_BUFFER_READY.store(true, Ordering::Release);

        unsafe {
          PERFORMANCE_METRICS.half_transfers += 1;
        }
      }

      if transfer.is_complete() {
        transfer.clear_transfer_complete_interrupt();

        // 完整缓冲区准备好
        ADC_BUFFER_READY.store(true, Ordering::Release);

        // 切换缓冲区
        let current = CURRENT_ADC_BUFFER.load(Ordering::Acquire);
        CURRENT_ADC_BUFFER.store(!current, Ordering::Release);

        unsafe {
          PERFORMANCE_METRICS.full_transfers += 1;
        }
      }
    }
  });
}

/// UART TX DMA中断处理
#[interrupt]
fn DMA2_STREAM7() {
  free(|cs| {
    if let Some(ref mut transfer) = G_UART_TX.borrow(cs).borrow_mut().as_mut() {
      if transfer.is_complete() {
        transfer.clear_transfer_complete_interrupt();

        unsafe {
          PERFORMANCE_METRICS.tx_transfers += 1;
        }
      }
    }
  });
}

/// UART接收中断
#[interrupt]
fn USART1() {
  free(|cs| {
    if let Some(ref mut rx) = G_UART_RX.borrow(cs).borrow_mut().as_mut() {
      if let Ok(_byte) = rx.read() {
        // 处理接收到的命令字节
        // 实际实现中应该解析命令协议
        unsafe {
          PERFORMANCE_METRICS.rx_bytes += 1;
        }
      }
    }
  });
}

/// 采样定时器中断
#[interrupt]
fn TIM2() {
  // 清除中断标志
  unsafe {
    (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit());
  }

  // 更新采样计数
  SAMPLES_COLLECTED.fetch_add(1, Ordering::Relaxed);

  unsafe {
    PERFORMANCE_METRICS.timer_interrupts += 1;
  }
}

// 数据类型定义
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum PacketType {
  Data = 0x01,
  Status = 0x02,
  Command = 0x03,
  Response = 0x04,
  Error = 0x05,
}

#[derive(Debug, Clone, Copy)]
pub enum Command {
  StartLogging,
  StopLogging,
  GetStatus,
  SetSampleRate(u32),
  Calibrate(u8, f32),
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum ResponseCode {
  Success = 0x00,
  Error = 0x01,
  InvalidCommand = 0x02,
  InvalidParameter = 0x03,
}

/// 性能指标
#[derive(Debug)]
pub struct PerformanceMetrics {
  pub half_transfers: u32,
  pub full_transfers: u32,
  pub tx_transfers: u32,
  pub timer_interrupts: u32,
  pub buffers_processed: u32,
  pub rx_bytes: u32,
  pub samples_per_second: f32,
  pub throughput_bps: f32,
}

impl PerformanceMetrics {
  pub const fn new() -> Self {
    Self {
      half_transfers: 0,
      full_transfers: 0,
      tx_transfers: 0,
      timer_interrupts: 0,
      buffers_processed: 0,
      rx_bytes: 0,
      samples_per_second: 0.0,
      throughput_bps: 0.0,
    }
  }

  pub fn calculate_rates(&mut self, elapsed_ms: u32) {
    if elapsed_ms > 0 {
      self.samples_per_second = (self.timer_interrupts as f32 * 1000.0) / elapsed_ms as f32;
      self.throughput_bps =
        (self.buffers_processed as f32 * BUFFER_SIZE as f32 * 2.0 * 8.0 * 1000.0)
          / elapsed_ms as f32;
    }
  }
}

// 辅助函数
fn calculate_checksum(data: &[u8]) -> u16 {
  data.iter().map(|&x| x as u16).sum()
}

fn generate_session_id() -> u16 {
  // 简单的会话ID生成
  (get_system_time_ms() & 0xFFFF) as u16
}

fn get_system_time_ms() -> u32 {
  // 简化实现
  static COUNTER: AtomicU32 = AtomicU32::new(0);
  COUNTER.fetch_add(1, Ordering::Relaxed)
}
