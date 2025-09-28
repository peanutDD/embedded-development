#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use heapless::{String, Vec};
use nb::block;
use panic_halt as _;
use stm32f4xx_hal::{
  adc::{Adc, AdcConfig, SampleTime},
  gpio::Analog,
  pac::{self, interrupt, Interrupt},
  prelude::*,
  serial::{Config, Serial},
  timer::{Event, Timer},
};

const BUFFER_SIZE: usize = 1024;
const HALF_BUFFER_SIZE: usize = BUFFER_SIZE / 2;

// 双缓冲区结构
#[derive(Clone, Copy)]
enum BufferState {
  FirstHalf,
  SecondHalf,
}

struct DualBuffer {
  buffer: [u16; BUFFER_SIZE],
  current_state: BufferState,
  write_index: usize,
  half_complete_flag: bool,
  full_complete_flag: bool,
  overrun_flag: bool,
}

impl DualBuffer {
  fn new() -> Self {
    Self {
      buffer: [0; BUFFER_SIZE],
      current_state: BufferState::FirstHalf,
      write_index: 0,
      half_complete_flag: false,
      full_complete_flag: false,
      overrun_flag: false,
    }
  }

  fn write_sample(&mut self, sample: u16) -> Result<(), ()> {
    if self.write_index >= BUFFER_SIZE {
      self.overrun_flag = true;
      return Err(());
    }

    self.buffer[self.write_index] = sample;
    self.write_index += 1;

    // 检查半缓冲区完成
    if self.write_index == HALF_BUFFER_SIZE {
      self.half_complete_flag = true;
      self.current_state = BufferState::SecondHalf;
    }

    // 检查全缓冲区完成
    if self.write_index == BUFFER_SIZE {
      self.full_complete_flag = true;
      self.write_index = 0; // 重置到缓冲区开始
      self.current_state = BufferState::FirstHalf;
    }

    Ok(())
  }

  fn get_ready_buffer(&mut self) -> Option<&[u16]> {
    if self.half_complete_flag {
      self.half_complete_flag = false;
      Some(&self.buffer[0..HALF_BUFFER_SIZE])
    } else if self.full_complete_flag {
      self.full_complete_flag = false;
      Some(&self.buffer[HALF_BUFFER_SIZE..BUFFER_SIZE])
    } else {
      None
    }
  }

  fn is_overrun(&mut self) -> bool {
    if self.overrun_flag {
      self.overrun_flag = false;
      true
    } else {
      false
    }
  }

  fn get_current_usage(&self) -> f32 {
    (self.write_index as f32 / BUFFER_SIZE as f32) * 100.0
  }
}

// 全局双缓冲区
static DUAL_BUFFER: Mutex<RefCell<DualBuffer>> = Mutex::new(RefCell::new(DualBuffer::new()));

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = pac::Peripherals::take().unwrap();
  let mut cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();

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

  // 配置ADC引脚
  let adc_pin = gpioa.pa0.into_analog(); // ADC1_IN0

  // 配置ADC
  let adc_config = AdcConfig::default().sample_time(SampleTime::Cycles_480);
  let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

  // 配置定时器用于模拟DMA采样
  let mut sample_timer = Timer::new(dp.TIM2, &clocks).counter_hz();
  sample_timer.start(50000.Hz()).unwrap(); // 50kHz采样率

  // 配置处理定时器
  let mut status_timer = Timer::new(dp.TIM3, &clocks).counter_hz();
  status_timer.start(10.Hz()).unwrap(); // 10Hz状态输出

  // 启用定时器中断
  cp.NVIC.enable(Interrupt::TIM2);

  writeln!(serial, "双缓冲区DMA采样示例启动").ok();
  writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
  writeln!(serial, "采样率: 50kHz").ok();
  writeln!(serial, "缓冲区大小: {} 样本", BUFFER_SIZE).ok();
  writeln!(serial, "半缓冲区大小: {} 样本", HALF_BUFFER_SIZE).ok();
  writeln!(serial, "").ok();

  let mut sample_count = 0u32;
  let mut process_count = 0u32;
  let mut total_processed_samples = 0u32;
  let vref = 3.3f32;

  // 性能监控
  let mut performance_monitor = DualBufferPerformanceMonitor::new();

  loop {
    // 模拟ADC采样 (实际中由DMA中断处理)
    if sample_timer.wait().is_ok() {
      sample_count += 1;

      // 模拟ADC读取
      let adc_value = simulate_adc_reading(sample_count);

      // 写入双缓冲区
      free(|cs| {
        let mut buffer = DUAL_BUFFER.borrow(cs).borrow_mut();
        match buffer.write_sample(adc_value) {
          Ok(()) => performance_monitor.record_successful_write(),
          Err(()) => performance_monitor.record_overrun(),
        }
      });
    }

    // 处理就绪的缓冲区数据
    let ready_buffer = free(|cs| {
      let mut buffer = DUAL_BUFFER.borrow(cs).borrow_mut();

      // 检查溢出
      if buffer.is_overrun() {
        performance_monitor.record_overrun();
        writeln!(serial, "警告: 双缓冲区溢出! (样本#{})", sample_count).ok();
      }

      buffer.get_ready_buffer().map(|slice| {
        let mut vec = Vec::<u16, HALF_BUFFER_SIZE>::new();
        for &sample in slice {
          vec.push(sample).ok();
        }
        vec
      })
    });

    if let Some(data) = ready_buffer {
      process_count += 1;
      total_processed_samples += data.len() as u32;

      // 处理数据块
      process_buffer_data(&mut serial, &data, process_count, vref);

      // 更新性能监控
      performance_monitor.record_buffer_processed(data.len());
    }

    // 定期输出状态信息
    if status_timer.wait().is_ok() {
      output_system_status(
        &mut serial,
        sample_count,
        total_processed_samples,
        &performance_monitor,
      );
    }
  }
}

/// 处理缓冲区数据
fn process_buffer_data(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  data: &Vec<u16, HALF_BUFFER_SIZE>,
  process_count: u32,
  vref: f32,
) {
  // 计算基本统计信息
  let stats = calculate_buffer_statistics(data, vref);

  // 执行信号处理
  let processed_data = apply_signal_processing(data);

  // 频谱分析
  let spectrum_info = perform_spectrum_analysis(&processed_data);

  // 每10次处理输出详细信息
  if process_count % 10 == 0 {
    writeln!(serial, "\n=== 缓冲区处理 #{:04} ===", process_count).ok();
    writeln!(serial, "样本数: {}", data.len()).ok();
    writeln!(
      serial,
      "均值: {:.3}V, 峰峰值: {:.3}V",
      stats.mean, stats.peak_to_peak
    )
    .ok();
    writeln!(
      serial,
      "RMS: {:.3}V, 标准差: {:.3}V",
      stats.rms, stats.std_dev
    )
    .ok();
    writeln!(
      serial,
      "主频: {:.1}Hz, 功率: {:.2}dB",
      spectrum_info.dominant_frequency, spectrum_info.power_db
    )
    .ok();
    writeln!(serial, "").ok();
  }

  // 异常检测
  detect_signal_anomalies(serial, &stats, &spectrum_info, process_count);

  // 质量评估
  let quality_score = assess_signal_quality(&stats, &spectrum_info);
  if quality_score < 0.6 {
    writeln!(
      serial,
      "警告: 信号质量低 {:.1}% (处理#{})",
      quality_score * 100.0,
      process_count
    )
    .ok();
  }
}

/// 缓冲区统计信息
#[derive(Debug)]
struct BufferStatistics {
  mean: f32,
  max: f32,
  min: f32,
  peak_to_peak: f32,
  rms: f32,
  std_dev: f32,
  dc_component: f32,
  ac_component: f32,
}

/// 计算缓冲区统计信息
fn calculate_buffer_statistics(data: &Vec<u16, HALF_BUFFER_SIZE>, vref: f32) -> BufferStatistics {
  if data.is_empty() {
    return BufferStatistics {
      mean: 0.0,
      max: 0.0,
      min: 0.0,
      peak_to_peak: 0.0,
      rms: 0.0,
      std_dev: 0.0,
      dc_component: 0.0,
      ac_component: 0.0,
    };
  }

  // 转换为电压
  let voltages: Vec<f32, HALF_BUFFER_SIZE> = data
    .iter()
    .map(|&adc_val| adc_to_voltage(adc_val, vref, 12))
    .collect();

  // 基本统计
  let sum: f32 = voltages.iter().sum();
  let mean = sum / voltages.len() as f32;

  let max = voltages.iter().fold(0.0f32, |a, &b| a.max(b));
  let min = voltages.iter().fold(vref, |a, &b| a.min(b));
  let peak_to_peak = max - min;

  // RMS计算
  let sum_squares: f32 = voltages.iter().map(|&v| v * v).sum();
  use micromath::F32Ext;
  let rms = (sum_squares / voltages.len() as f32).sqrt();

  // 标准差
  let variance: f32 = voltages
    .iter()
    .map(|&v| {
      let diff = v - mean;
      diff * diff
    })
    .sum::<f32>()
    / voltages.len() as f32;
  let std_dev = variance.sqrt();

  // DC和AC分量
  let dc_component = mean;
  let ac_component = (rms * rms - dc_component * dc_component).max(0.0).sqrt();

  BufferStatistics {
    mean,
    max,
    min,
    peak_to_peak,
    rms,
    std_dev,
    dc_component,
    ac_component,
  }
}

/// 应用信号处理
fn apply_signal_processing(data: &Vec<u16, HALF_BUFFER_SIZE>) -> Vec<u16, HALF_BUFFER_SIZE> {
  let mut processed = Vec::new();

  if data.len() < 5 {
    // 数据太少，直接返回
    for &sample in data {
      processed.push(sample).ok();
    }
    return processed;
  }

  // 应用多级滤波
  let filtered1 = apply_moving_average_filter(data, 3);
  let filtered2 = apply_median_filter(&filtered1, 3);
  let filtered3 = apply_high_pass_filter(&filtered2);

  filtered3
}

/// 移动平均滤波器
fn apply_moving_average_filter(
  data: &Vec<u16, HALF_BUFFER_SIZE>,
  window: usize,
) -> Vec<u16, HALF_BUFFER_SIZE> {
  let mut filtered = Vec::new();

  if data.len() < window {
    return data.clone();
  }

  for i in 0..data.len() {
    let start = if i >= window / 2 { i - window / 2 } else { 0 };
    let end = (i + window / 2 + 1).min(data.len());

    let sum: u32 = (start..end).map(|idx| data[idx] as u32).sum();
    let avg = (sum / (end - start) as u32) as u16;
    filtered.push(avg).ok();
  }

  filtered
}

/// 中值滤波器
fn apply_median_filter(
  data: &Vec<u16, HALF_BUFFER_SIZE>,
  window: usize,
) -> Vec<u16, HALF_BUFFER_SIZE> {
  let mut filtered = Vec::new();

  if data.len() < window {
    return data.clone();
  }

  for i in 0..data.len() {
    let start = if i >= window / 2 { i - window / 2 } else { 0 };
    let end = (i + window / 2 + 1).min(data.len());

    let mut window_data: Vec<u16, 16> = Vec::new();
    for idx in start..end {
      window_data.push(data[idx]).ok();
    }

    // 简单排序找中值
    window_data.sort();
    let median = window_data[window_data.len() / 2];
    filtered.push(median).ok();
  }

  filtered
}

/// 高通滤波器 (简单差分)
fn apply_high_pass_filter(data: &Vec<u16, HALF_BUFFER_SIZE>) -> Vec<u16, HALF_BUFFER_SIZE> {
  let mut filtered = Vec::new();

  if data.is_empty() {
    return filtered;
  }

  // 第一个样本保持不变
  filtered.push(data[0]).ok();

  // 应用高通滤波 (差分)
  for i in 1..data.len() {
    let diff = if data[i] > data[i - 1] {
      data[i] - data[i - 1]
    } else {
      data[i - 1] - data[i]
    };

    // 限制输出范围
    let output = (data[i] as i32 + diff as i32 / 2).max(0).min(4095) as u16;
    filtered.push(output).ok();
  }

  filtered
}

/// 频谱分析信息
#[derive(Debug)]
struct SpectrumInfo {
  dominant_frequency: f32,
  power_db: f32,
  frequency_bins: Vec<f32, 32>,
  power_spectrum: Vec<f32, 32>,
}

/// 执行频谱分析
fn perform_spectrum_analysis(data: &Vec<u16, HALF_BUFFER_SIZE>) -> SpectrumInfo {
  // 简化的频谱分析 (基于FFT的概念，但这里用简单方法)
  let sample_rate = 50000.0; // 50kHz

  // 计算功率谱密度 (简化版本)
  let mut frequency_bins = Vec::new();
  let mut power_spectrum = Vec::new();

  // 分析几个关键频率
  let test_frequencies = [
    100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 20000.0,
  ];

  for &freq in &test_frequencies {
    let power = calculate_power_at_frequency(data, freq, sample_rate);
    frequency_bins.push(freq).ok();
    power_spectrum.push(power).ok();
  }

  // 找到主频
  let max_power_idx = power_spectrum
    .iter()
    .enumerate()
    .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    .map(|(idx, _)| idx)
    .unwrap_or(0);

  let dominant_frequency = frequency_bins[max_power_idx];
  let power_db = 10.0 * power_spectrum[max_power_idx].log10();

  SpectrumInfo {
    dominant_frequency,
    power_db,
    frequency_bins,
    power_spectrum,
  }
}

/// 计算特定频率的功率
fn calculate_power_at_frequency(
  data: &Vec<u16, HALF_BUFFER_SIZE>,
  frequency: f32,
  sample_rate: f32,
) -> f32 {
  if data.is_empty() {
    return 0.0;
  }

  use micromath::F32Ext;

  let mut real_sum = 0.0f32;
  let mut imag_sum = 0.0f32;

  for (i, &sample) in data.iter().enumerate() {
    let t = i as f32 / sample_rate;
    let phase = 2.0 * core::f32::consts::PI * frequency * t;

    let sample_f = sample as f32 - 2048.0; // 去除DC分量
    real_sum += sample_f * phase.cos();
    imag_sum += sample_f * phase.sin();
  }

  let magnitude = (real_sum * real_sum + imag_sum * imag_sum).sqrt();
  magnitude / data.len() as f32
}

/// 检测信号异常
fn detect_signal_anomalies(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  stats: &BufferStatistics,
  spectrum: &SpectrumInfo,
  process_count: u32,
) {
  // 幅度异常检测
  if stats.peak_to_peak > 3.0 {
    writeln!(
      serial,
      "异常: 信号幅度过大 {:.3}V (处理#{})",
      stats.peak_to_peak, process_count
    )
    .ok();
  }

  if stats.peak_to_peak < 0.1 {
    writeln!(
      serial,
      "异常: 信号幅度过小 {:.3}V (处理#{})",
      stats.peak_to_peak, process_count
    )
    .ok();
  }

  // 频率异常检测
  if spectrum.dominant_frequency > 15000.0 {
    writeln!(
      serial,
      "异常: 检测到高频信号 {:.1}Hz (处理#{})",
      spectrum.dominant_frequency, process_count
    )
    .ok();
  }

  // DC偏移异常
  if stats.dc_component > 2.8 || stats.dc_component < 0.5 {
    writeln!(
      serial,
      "异常: DC偏移异常 {:.3}V (处理#{})",
      stats.dc_component, process_count
    )
    .ok();
  }

  // 噪声水平异常
  if stats.std_dev > 0.5 {
    writeln!(
      serial,
      "异常: 噪声水平过高 {:.3}V (处理#{})",
      stats.std_dev, process_count
    )
    .ok();
  }
}

/// 评估信号质量
fn assess_signal_quality(stats: &BufferStatistics, spectrum: &SpectrumInfo) -> f32 {
  // 信噪比因子
  let snr_factor = if stats.std_dev > 0.0 {
    (stats.ac_component / stats.std_dev).min(10.0) / 10.0
  } else {
    1.0
  };

  // 幅度因子
  let amplitude_factor = if stats.peak_to_peak > 0.2 && stats.peak_to_peak < 2.8 {
    1.0
  } else {
    0.5
  };

  // 频率稳定性因子
  let freq_factor = if spectrum.dominant_frequency > 50.0 && spectrum.dominant_frequency < 10000.0 {
    1.0
  } else {
    0.7
  };

  // DC稳定性因子
  let dc_factor = if stats.dc_component > 1.0 && stats.dc_component < 2.5 {
    1.0
  } else {
    0.8
  };

  snr_factor * amplitude_factor * freq_factor * dc_factor
}

/// 输出系统状态
fn output_system_status(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  sample_count: u32,
  total_processed_samples: u32,
  monitor: &DualBufferPerformanceMonitor,
) {
  let buffer_usage = free(|cs| {
    let buffer = DUAL_BUFFER.borrow(cs).borrow();
    buffer.get_current_usage()
  });

  writeln!(serial, "\n=== 系统状态 ===").ok();
  writeln!(serial, "采样计数: {}", sample_count).ok();
  writeln!(serial, "已处理样本: {}", total_processed_samples).ok();
  writeln!(serial, "缓冲区使用率: {:.1}%", buffer_usage).ok();
  writeln!(
    serial,
    "处理效率: {:.1}%",
    monitor.get_processing_efficiency() * 100.0
  )
  .ok();
  writeln!(serial, "溢出次数: {}", monitor.get_overrun_count()).ok();
  writeln!(
    serial,
    "处理延迟: {:.2}ms",
    monitor.get_average_processing_delay()
  )
  .ok();
  writeln!(serial, "").ok();
}

/// 双缓冲区性能监控
struct DualBufferPerformanceMonitor {
  total_writes: u32,
  successful_writes: u32,
  overrun_count: u32,
  buffers_processed: u32,
  total_samples_processed: u32,
  processing_start_time: u32,
}

impl DualBufferPerformanceMonitor {
  fn new() -> Self {
    Self {
      total_writes: 0,
      successful_writes: 0,
      overrun_count: 0,
      buffers_processed: 0,
      total_samples_processed: 0,
      processing_start_time: 0,
    }
  }

  fn record_successful_write(&mut self) {
    self.total_writes += 1;
    self.successful_writes += 1;
  }

  fn record_overrun(&mut self) {
    self.total_writes += 1;
    self.overrun_count += 1;
  }

  fn record_buffer_processed(&mut self, samples: usize) {
    self.buffers_processed += 1;
    self.total_samples_processed += samples as u32;
  }

  fn get_processing_efficiency(&self) -> f32 {
    if self.total_writes == 0 {
      return 0.0;
    }
    self.successful_writes as f32 / self.total_writes as f32
  }

  fn get_overrun_count(&self) -> u32 {
    self.overrun_count
  }

  fn get_average_processing_delay(&self) -> f32 {
    // 简化的处理延迟计算
    if self.buffers_processed == 0 {
      return 0.0;
    }

    let samples_per_buffer = HALF_BUFFER_SIZE as f32;
    let sample_rate = 50000.0; // 50kHz
    let buffer_time_ms = (samples_per_buffer / sample_rate) * 1000.0;

    buffer_time_ms * 0.1 // 假设处理延迟为缓冲区时间的10%
  }
}

/// ADC值转换为电压
fn adc_to_voltage(adc_value: u16, vref: f32, resolution: u8) -> f32 {
  let max_value = (1 << resolution) - 1;
  (adc_value as f32 / max_value as f32) * vref
}

/// 模拟ADC读取
fn simulate_adc_reading(sample_count: u32) -> u16 {
  use micromath::F32Ext;

  // 生成复合信号：直流 + 多个正弦波 + 噪声
  let dc_offset = 2048.0; // 1.65V直流偏移

  // 主信号：1kHz正弦波
  let main_amplitude = 600.0;
  let main_freq = 1000.0;
  let sample_rate = 50000.0;

  let t = sample_count as f32 / sample_rate;
  let main_signal = main_amplitude * (2.0 * core::f32::consts::PI * main_freq * t).sin();

  // 谐波：3kHz
  let harmonic_amplitude = 200.0;
  let harmonic_freq = 3000.0;
  let harmonic_signal =
    harmonic_amplitude * (2.0 * core::f32::consts::PI * harmonic_freq * t).sin();

  // 高频噪声
  let noise_amplitude = 50.0;
  let noise = noise_amplitude * ((sample_count * 17) % 41) as f32 / 20.0 - noise_amplitude;

  let result = dc_offset + main_signal + harmonic_signal + noise;
  result.max(0.0).min(4095.0) as u16
}

/// 定时器中断处理 (模拟DMA采样)
#[interrupt]
fn TIM2() {
  // 在实际应用中，这里会由DMA中断处理
  // 这里只是清除中断标志
}
