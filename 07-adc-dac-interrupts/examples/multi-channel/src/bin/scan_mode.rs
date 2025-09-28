#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m_rt::entry;
use heapless::{String, Vec};
use nb::block;
use panic_halt as _;
use stm32f4xx_hal::{
  adc::{Adc, AdcConfig, SampleTime},
  dma::{MemoryToPeripheral, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
  gpio::Analog,
  pac,
  prelude::*,
  serial::{Config, Serial},
  timer::{Event, Timer},
};

const CHANNEL_COUNT: usize = 8;
const BUFFER_SIZE: usize = 64;

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 配置串口
  let tx_pin = gpioa.pa2.into_alternate();
  let rx_pin = gpioa.pa3.into_alternate();
  let mut serial = Serial::new(
    dp.USART2,
    (tx_pin, rx_pin),
    Config::default().baudrate(115200.bps()),
    &clocks,
  )
  .unwrap();

  // 配置ADC引脚 (8个通道)
  let _adc_pin0 = gpioa.pa0.into_analog(); // ADC1_IN0
  let _adc_pin1 = gpioa.pa1.into_analog(); // ADC1_IN1
  let _adc_pin4 = gpioa.pa4.into_analog(); // ADC1_IN4
  let _adc_pin5 = gpioa.pa5.into_analog(); // ADC1_IN5
  let _adc_pin6 = gpioa.pa6.into_analog(); // ADC1_IN6
  let _adc_pin7 = gpioa.pa7.into_analog(); // ADC1_IN7
  let _adc_pin8 = gpiob.pb0.into_analog(); // ADC1_IN8
  let _adc_pin9 = gpiob.pb1.into_analog(); // ADC1_IN9

  // 配置ADC为扫描模式
  let adc_config = AdcConfig::default().sample_time(SampleTime::Cycles_480);
  let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

  // 配置DMA
  let dma = StreamsTuple::new(dp.DMA2);

  // ADC数据缓冲区
  static mut ADC_BUFFER: [u16; BUFFER_SIZE * CHANNEL_COUNT] = [0; BUFFER_SIZE * CHANNEL_COUNT];

  writeln!(serial, "ADC扫描模式示例启动").ok();
  writeln!(serial, "通道数量: {}", CHANNEL_COUNT).ok();
  writeln!(serial, "缓冲区大小: {}", BUFFER_SIZE).ok();
  writeln!(serial, "扫描模式: 连续扫描所有通道").ok();
  writeln!(serial, "").ok();

  // 配置定时器触发ADC转换
  let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
  timer.start(1000.Hz()).unwrap(); // 1kHz触发频率

  let mut scan_count = 0u32;
  let vref = 3.3f32;

  // 通道配置序列
  let channel_sequence = [0, 1, 4, 5, 6, 7, 8, 9];

  loop {
    // 等待定时器触发
    block!(timer.wait()).ok();

    // 执行扫描转换
    let mut scan_results: Vec<u16, CHANNEL_COUNT> = Vec::new();

    // 扫描所有配置的通道
    for &channel in &channel_sequence {
      // 这里需要根据实际HAL API实现通道选择和转换
      // let adc_value = adc.read_channel(channel);
      let adc_value = simulate_adc_reading(channel, scan_count);
      scan_results.push(adc_value).ok();
    }

    scan_count += 1;

    // 处理扫描结果
    process_scan_results(&mut serial, &scan_results, scan_count, vref);

    // 每1000次扫描输出统计信息
    if scan_count % 1000 == 0 {
      output_scan_statistics(&mut serial, &scan_results, scan_count);
    }

    // 检测异常值
    detect_anomalies(&mut serial, &scan_results, scan_count);
  }
}

/// 处理扫描结果
fn process_scan_results(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  results: &Vec<u16, CHANNEL_COUNT>,
  scan_count: u32,
  vref: f32,
) {
  // 每100次扫描输出一次详细数据
  if scan_count % 100 == 0 {
    let mut output: String<256> = String::new();
    write!(output, "Scan #{:05}: ", scan_count).ok();

    for (i, &adc_value) in results.iter().enumerate() {
      let voltage = adc_to_voltage(adc_value, vref, 12);
      write!(output, "CH{}: {:.3}V ", i, voltage).ok();
    }

    writeln!(serial, "{}", output).ok();
  }

  // 实时监控关键通道
  if let Some(&ch0_value) = results.get(0) {
    let voltage = adc_to_voltage(ch0_value, vref, 12);
    if voltage > 3.0 || voltage < 0.3 {
      writeln!(serial, "警告: CH0电压异常 = {:.3}V", voltage).ok();
    }
  }
}

/// 输出扫描统计信息
fn output_scan_statistics(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  current_results: &Vec<u16, CHANNEL_COUNT>,
  scan_count: u32,
) {
  writeln!(serial, "\n=== 扫描统计 (第{}次扫描) ===", scan_count).ok();

  for (i, &adc_value) in current_results.iter().enumerate() {
    let voltage = adc_to_voltage(adc_value, vref, 12);
    writeln!(serial, "CH{}: ADC={:4}, 电压={:.3}V", i, adc_value, voltage).ok();
  }

  // 计算总扫描速率
  let scan_rate = scan_count as f32 / (scan_count as f32 / 1000.0); // 假设1kHz触发
  writeln!(serial, "扫描速率: {:.1} scans/sec", scan_rate).ok();
  writeln!(serial, "").ok();
}

/// 检测异常值
fn detect_anomalies(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  results: &Vec<u16, CHANNEL_COUNT>,
  scan_count: u32,
) {
  const VOLTAGE_THRESHOLD_HIGH: f32 = 3.1;
  const VOLTAGE_THRESHOLD_LOW: f32 = 0.2;

  for (i, &adc_value) in results.iter().enumerate() {
    let voltage = adc_to_voltage(adc_value, vref, 12);

    if voltage > VOLTAGE_THRESHOLD_HIGH {
      writeln!(
        serial,
        "异常检测: CH{} 电压过高 {:.3}V (扫描#{})",
        i, voltage, scan_count
      )
      .ok();
    } else if voltage < VOLTAGE_THRESHOLD_LOW {
      writeln!(
        serial,
        "异常检测: CH{} 电压过低 {:.3}V (扫描#{})",
        i, voltage, scan_count
      )
      .ok();
    }
  }
}

/// ADC值转换为电压
fn adc_to_voltage(adc_value: u16, vref: f32, resolution: u8) -> f32 {
  let max_value = (1 << resolution) - 1;
  (adc_value as f32 / max_value as f32) * vref
}

/// 模拟ADC读取 (用于演示)
fn simulate_adc_reading(channel: u8, scan_count: u32) -> u16 {
  use micromath::F32Ext;

  // 模拟不同通道的不同信号
  let base_value = match channel {
    0 => 2048,                                                    // ~1.65V
    1 => 1024,                                                    // ~0.825V
    4 => 3072,                                                    // ~2.475V
    5 => 512,                                                     // ~0.4125V
    6 => 2560,                                                    // ~2.06V
    7 => 1536,                                                    // ~1.24V
    8 => 2048 + (200.0 * (scan_count as f32 * 0.1).sin()) as i16, // 正弦波
    9 => 2048 + (scan_count % 100) as i16 * 20,                   // 锯齿波
    _ => 2048,
  };

  // 添加一些噪声
  let noise = (scan_count % 7) as i16 - 3;
  (base_value + noise).max(0).min(4095) as u16
}

/// 扫描模式配置结构
struct ScanModeConfig {
  channels: Vec<u8, CHANNEL_COUNT>,
  sample_time: SampleTime,
  continuous: bool,
  dma_enabled: bool,
}

impl Default for ScanModeConfig {
  fn default() -> Self {
    let mut channels = Vec::new();
    // 添加默认通道
    for i in 0..CHANNEL_COUNT {
      channels.push(i as u8).ok();
    }

    Self {
      channels,
      sample_time: SampleTime::Cycles_480,
      continuous: true,
      dma_enabled: true,
    }
  }
}

/// 扫描模式管理器
struct ScanModeManager {
  config: ScanModeConfig,
  scan_buffer: Vec<u16, BUFFER_SIZE>,
  channel_averages: Vec<f32, CHANNEL_COUNT>,
}

impl ScanModeManager {
  fn new(config: ScanModeConfig) -> Self {
    Self {
      config,
      scan_buffer: Vec::new(),
      channel_averages: Vec::new(),
    }
  }

  fn configure_scan_sequence(&mut self) {
    // 配置ADC扫描序列
    // 实际实现需要设置ADC_SQR寄存器
  }

  fn start_scan(&mut self) {
    // 启动扫描转换
    // 设置ADC_CR2的SWSTART位
  }

  fn process_scan_data(&mut self, data: &[u16]) {
    // 处理扫描数据
    for (i, &value) in data.iter().enumerate() {
      if i < self.channel_averages.len() {
        // 简单的移动平均
        let current_avg = self.channel_averages[i];
        let new_avg = current_avg * 0.9 + (value as f32) * 0.1;
        self.channel_averages[i] = new_avg;
      }
    }
  }
}
