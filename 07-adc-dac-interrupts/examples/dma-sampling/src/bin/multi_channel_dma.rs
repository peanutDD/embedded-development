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

const CHANNEL_COUNT: usize = 4;
const SAMPLES_PER_CHANNEL: usize = 512;
const TOTAL_BUFFER_SIZE: usize = CHANNEL_COUNT * SAMPLES_PER_CHANNEL;

// 多通道DMA缓冲区结构
struct MultiChannelDmaBuffer {
  buffer: [u16; TOTAL_BUFFER_SIZE],
  write_index: usize,
  channel_buffers: [[u16; SAMPLES_PER_CHANNEL]; CHANNEL_COUNT],
  buffer_ready: [bool; CHANNEL_COUNT],
  current_channel: usize,
  samples_in_current_channel: usize,
}

impl MultiChannelDmaBuffer {
  fn new() -> Self {
    Self {
      buffer: [0; TOTAL_BUFFER_SIZE],
      write_index: 0,
      channel_buffers: [[0; SAMPLES_PER_CHANNEL]; CHANNEL_COUNT],
      buffer_ready: [false; CHANNEL_COUNT],
      current_channel: 0,
      samples_in_current_channel: 0,
    }
  }

  fn write_sample(&mut self, channel: usize, sample: u16) -> Result<(), ()> {
    if channel >= CHANNEL_COUNT {
      return Err(());
    }

    if self.samples_in_current_channel >= SAMPLES_PER_CHANNEL {
      return Err(()); // 缓冲区满
    }

    // 写入通道缓冲区
    self.channel_buffers[channel][self.samples_in_current_channel] = sample;

    // 写入交错缓冲区
    let interleaved_index = self.samples_in_current_channel * CHANNEL_COUNT + channel;
    if interleaved_index < TOTAL_BUFFER_SIZE {
      self.buffer[interleaved_index] = sample;
    }

    // 如果是最后一个通道，增加样本计数
    if channel == CHANNEL_COUNT - 1 {
      self.samples_in_current_channel += 1;

      // 检查缓冲区是否满
      if self.samples_in_current_channel >= SAMPLES_PER_CHANNEL {
        for i in 0..CHANNEL_COUNT {
          self.buffer_ready[i] = true;
        }
        self.samples_in_current_channel = 0;
      }
    }

    Ok(())
  }

  fn get_channel_buffer(&mut self, channel: usize) -> Option<&[u16; SAMPLES_PER_CHANNEL]> {
    if channel < CHANNEL_COUNT && self.buffer_ready[channel] {
      self.buffer_ready[channel] = false;
      Some(&self.channel_buffers[channel])
    } else {
      None
    }
  }

  fn is_any_buffer_ready(&self) -> bool {
    self.buffer_ready.iter().any(|&ready| ready)
  }

  fn get_fill_percentage(&self) -> f32 {
    (self.samples_in_current_channel as f32 / SAMPLES_PER_CHANNEL as f32) * 100.0
  }
}

// 全局多通道缓冲区
static MULTI_CHANNEL_BUFFER: Mutex<RefCell<MultiChannelDmaBuffer>> =
  Mutex::new(RefCell::new(MultiChannelDmaBuffer::new()));

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

  // 配置多个ADC引脚
  let adc_pin0 = gpioa.pa0.into_analog(); // ADC1_IN0
  let adc_pin1 = gpioa.pa1.into_analog(); // ADC1_IN1
  let adc_pin2 = gpioa.pa4.into_analog(); // ADC1_IN4
  let adc_pin3 = gpioa.pa5.into_analog(); // ADC1_IN5

  // 配置ADC
  let adc_config = AdcConfig::default().sample_time(SampleTime::Cycles_480);
  let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

  // 配置采样定时器
  let mut sample_timer = Timer::new(dp.TIM2, &clocks).counter_hz();
  sample_timer.start(40000.Hz()).unwrap(); // 40kHz总采样率 (每通道10kHz)

  // 配置处理定时器
  let mut process_timer = Timer::new(dp.TIM3, &clocks).counter_hz();
  process_timer.start(20.Hz()).unwrap(); // 20Hz处理频率

  // 配置状态输出定时器
  let mut status_timer = Timer::new(dp.TIM4, &clocks).counter_hz();
  status_timer.start(2.Hz()).unwrap(); // 2Hz状态输出

  // 启用定时器中断
  cp.NVIC.enable(Interrupt::TIM2);

  writeln!(serial, "多通道DMA采样示例启动").ok();
  writeln!(serial, "通道数: {}", CHANNEL_COUNT).ok();
  writeln!(serial, "每通道样本数: {}", SAMPLES_PER_CHANNEL).ok();
  writeln!(serial, "总采样率: 40kHz").ok();
  writeln!(serial, "每通道采样率: 10kHz").ok();
  writeln!(serial, "ADC通道映射:").ok();
  writeln!(serial, "  CH0: PA0 (ADC1_IN0)").ok();
  writeln!(serial, "  CH1: PA1 (ADC1_IN1)").ok();
  writeln!(serial, "  CH2: PA4 (ADC1_IN4)").ok();
  writeln!(serial, "  CH3: PA5 (ADC1_IN5)").ok();
  writeln!(serial, "").ok();

  let mut sample_count = 0u32;
  let mut process_count = 0u32;
  let mut current_channel = 0usize;
  let vref = 3.3f32;

  // 多通道性能监控
  let mut channel_monitors = [ChannelPerformanceMonitor::new(); CHANNEL_COUNT];
  let mut cross_channel_analyzer = CrossChannelAnalyzer::new();

  loop {
    // 多通道采样 (模拟DMA扫描模式)
    if sample_timer.wait().is_ok() {
      sample_count += 1;

      // 模拟多通道ADC读取
      let adc_value = simulate_multi_channel_adc(current_channel, sample_count);

      // 写入多通道缓冲区
      free(|cs| {
        let mut buffer = MULTI_CHANNEL_BUFFER.borrow(cs).borrow_mut();
        match buffer.write_sample(current_channel, adc_value) {
          Ok(()) => {
            channel_monitors[current_channel].record_successful_sample();
          }
          Err(()) => {
            channel_monitors[current_channel].record_overflow();
          }
        }
      });

      // 切换到下一个通道
      current_channel = (current_channel + 1) % CHANNEL_COUNT;
    }

    // 处理就绪的通道数据
    if process_timer.wait().is_ok() {
      process_count += 1;

      // 检查并处理每个通道的数据
      for channel in 0..CHANNEL_COUNT {
        let channel_data = free(|cs| {
          let mut buffer = MULTI_CHANNEL_BUFFER.borrow(cs).borrow_mut();
          buffer.get_channel_buffer(channel).map(|data| *data)
        });

        if let Some(data) = channel_data {
          process_channel_data(
            &mut serial,
            channel,
            &data,
            process_count,
            vref,
            &mut channel_monitors[channel],
          );
        }
      }

      // 执行跨通道分析
      perform_cross_channel_analysis(
        &mut serial,
        &channel_monitors,
        &mut cross_channel_analyzer,
        process_count,
      );
    }

    // 输出系统状态
    if status_timer.wait().is_ok() {
      output_multi_channel_status(
        &mut serial,
        sample_count,
        process_count,
        &channel_monitors,
        &cross_channel_analyzer,
      );
    }
  }
}

/// 处理单个通道数据
fn process_channel_data(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  channel: usize,
  data: &[u16; SAMPLES_PER_CHANNEL],
  process_count: u32,
  vref: f32,
  monitor: &mut ChannelPerformanceMonitor,
) {
  // 计算通道统计信息
  let stats = calculate_channel_statistics(data, vref);

  // 更新性能监控
  monitor.update_statistics(&stats);

  // 应用通道特定的信号处理
  let processed_data = apply_channel_processing(data, channel);

  // 检测通道异常
  detect_channel_anomalies(serial, channel, &stats, process_count);

  // 每50次处理输出通道详细信息
  if process_count % 50 == 0 {
    writeln!(
      serial,
      "CH{}: 均值={:.3}V, RMS={:.3}V, 峰峰值={:.3}V",
      channel, stats.mean, stats.rms, stats.peak_to_peak
    )
    .ok();
  }
}

/// 通道统计信息
#[derive(Debug, Clone, Copy)]
struct ChannelStatistics {
  mean: f32,
  max: f32,
  min: f32,
  rms: f32,
  std_dev: f32,
  peak_to_peak: f32,
  dc_offset: f32,
  ac_component: f32,
  snr: f32,
}

/// 计算通道统计信息
fn calculate_channel_statistics(data: &[u16; SAMPLES_PER_CHANNEL], vref: f32) -> ChannelStatistics {
  // 转换为电压
  let mut voltages = [0.0f32; SAMPLES_PER_CHANNEL];
  for (i, &sample) in data.iter().enumerate() {
    voltages[i] = adc_to_voltage(sample, vref, 12);
  }

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
  let dc_offset = mean;
  let ac_component = (rms * rms - dc_offset * dc_offset).max(0.0).sqrt();

  // 信噪比
  let snr = if std_dev > 0.0 {
    20.0 * (ac_component / std_dev).log10()
  } else {
    100.0
  };

  ChannelStatistics {
    mean,
    max,
    min,
    rms,
    std_dev,
    peak_to_peak,
    dc_offset,
    ac_component,
    snr,
  }
}

/// 应用通道特定处理
fn apply_channel_processing(
  data: &[u16; SAMPLES_PER_CHANNEL],
  channel: usize,
) -> [u16; SAMPLES_PER_CHANNEL] {
  let mut processed = *data;

  match channel {
    0 => {
      // 通道0：低通滤波
      apply_low_pass_filter(&mut processed);
    }
    1 => {
      // 通道1：高通滤波
      apply_high_pass_filter(&mut processed);
    }
    2 => {
      // 通道2：带通滤波
      apply_band_pass_filter(&mut processed);
    }
    3 => {
      // 通道3：陷波滤波
      apply_notch_filter(&mut processed);
    }
    _ => {}
  }

  processed
}

/// 低通滤波器
fn apply_low_pass_filter(data: &mut [u16; SAMPLES_PER_CHANNEL]) {
  // 简单的3点移动平均
  if data.len() < 3 {
    return;
  }

  let mut temp = *data;
  for i in 1..data.len() - 1 {
    let sum = temp[i - 1] as u32 + temp[i] as u32 + temp[i + 1] as u32;
    data[i] = (sum / 3) as u16;
  }
}

/// 高通滤波器
fn apply_high_pass_filter(data: &mut [u16; SAMPLES_PER_CHANNEL]) {
  if data.len() < 2 {
    return;
  }

  let mut temp = *data;
  for i in 1..data.len() {
    let diff = if temp[i] > temp[i - 1] {
      temp[i] - temp[i - 1]
    } else {
      temp[i - 1] - temp[i]
    };
    data[i] = (temp[i] as i32 + diff as i32 / 2).max(0).min(4095) as u16;
  }
}

/// 带通滤波器
fn apply_band_pass_filter(data: &mut [u16; SAMPLES_PER_CHANNEL]) {
  // 先应用高通，再应用低通
  apply_high_pass_filter(data);
  apply_low_pass_filter(data);
}

/// 陷波滤波器 (50Hz)
fn apply_notch_filter(data: &mut [u16; SAMPLES_PER_CHANNEL]) {
  // 简化的陷波滤波器实现
  let sample_rate = 10000.0; // 10kHz
  let notch_freq = 50.0; // 50Hz

  // 计算陷波系数
  use micromath::F32Ext;
  let omega = 2.0 * core::f32::consts::PI * notch_freq / sample_rate;
  let cos_omega = omega.cos();

  // 应用简单的陷波滤波
  if data.len() >= 3 {
    let mut temp = *data;
    for i in 2..data.len() {
      let input = temp[i] as f32;
      let delayed1 = temp[i - 1] as f32;
      let delayed2 = temp[i - 2] as f32;

      // 简化的陷波滤波器方程
      let output = input - 2.0 * cos_omega * delayed1 + delayed2;
      data[i] = output.max(0.0).min(4095.0) as u16;
    }
  }
}

/// 检测通道异常
fn detect_channel_anomalies(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  channel: usize,
  stats: &ChannelStatistics,
  process_count: u32,
) {
  // 幅度异常
  if stats.peak_to_peak > 3.0 {
    writeln!(
      serial,
      "CH{} 异常: 幅度过大 {:.3}V (处理#{})",
      channel, stats.peak_to_peak, process_count
    )
    .ok();
  }

  if stats.peak_to_peak < 0.05 {
    writeln!(
      serial,
      "CH{} 异常: 幅度过小 {:.3}V (处理#{})",
      channel, stats.peak_to_peak, process_count
    )
    .ok();
  }

  // DC偏移异常
  if stats.dc_offset > 2.8 || stats.dc_offset < 0.5 {
    writeln!(
      serial,
      "CH{} 异常: DC偏移 {:.3}V (处理#{})",
      channel, stats.dc_offset, process_count
    )
    .ok();
  }

  // 信噪比异常
  if stats.snr < 20.0 {
    writeln!(
      serial,
      "CH{} 异常: SNR过低 {:.1}dB (处理#{})",
      channel, stats.snr, process_count
    )
    .ok();
  }
}

/// 通道性能监控
#[derive(Debug, Clone, Copy)]
struct ChannelPerformanceMonitor {
  total_samples: u32,
  successful_samples: u32,
  overflow_count: u32,
  avg_mean: f32,
  avg_rms: f32,
  avg_snr: f32,
  min_snr: f32,
  max_snr: f32,
}

impl ChannelPerformanceMonitor {
  fn new() -> Self {
    Self {
      total_samples: 0,
      successful_samples: 0,
      overflow_count: 0,
      avg_mean: 0.0,
      avg_rms: 0.0,
      avg_snr: 0.0,
      min_snr: 100.0,
      max_snr: 0.0,
    }
  }

  fn record_successful_sample(&mut self) {
    self.total_samples += 1;
    self.successful_samples += 1;
  }

  fn record_overflow(&mut self) {
    self.total_samples += 1;
    self.overflow_count += 1;
  }

  fn update_statistics(&mut self, stats: &ChannelStatistics) {
    // 更新平均值 (简单移动平均)
    let alpha = 0.1f32; // 平滑因子
    self.avg_mean = self.avg_mean * (1.0 - alpha) + stats.mean * alpha;
    self.avg_rms = self.avg_rms * (1.0 - alpha) + stats.rms * alpha;
    self.avg_snr = self.avg_snr * (1.0 - alpha) + stats.snr * alpha;

    // 更新SNR范围
    self.min_snr = self.min_snr.min(stats.snr);
    self.max_snr = self.max_snr.max(stats.snr);
  }

  fn get_success_rate(&self) -> f32 {
    if self.total_samples == 0 {
      return 0.0;
    }
    self.successful_samples as f32 / self.total_samples as f32
  }
}

/// 跨通道分析器
struct CrossChannelAnalyzer {
  correlation_matrix: [[f32; CHANNEL_COUNT]; CHANNEL_COUNT],
  phase_differences: [f32; CHANNEL_COUNT],
  amplitude_ratios: [f32; CHANNEL_COUNT],
  coherence_values: [f32; CHANNEL_COUNT],
}

impl CrossChannelAnalyzer {
  fn new() -> Self {
    Self {
      correlation_matrix: [[0.0; CHANNEL_COUNT]; CHANNEL_COUNT],
      phase_differences: [0.0; CHANNEL_COUNT],
      amplitude_ratios: [0.0; CHANNEL_COUNT],
      coherence_values: [0.0; CHANNEL_COUNT],
    }
  }

  fn update_analysis(&mut self, monitors: &[ChannelPerformanceMonitor; CHANNEL_COUNT]) {
    // 计算通道间相关性
    for i in 0..CHANNEL_COUNT {
      for j in 0..CHANNEL_COUNT {
        if i != j {
          self.correlation_matrix[i][j] = calculate_channel_correlation(&monitors[i], &monitors[j]);
        } else {
          self.correlation_matrix[i][j] = 1.0;
        }
      }
    }

    // 计算相位差和幅度比
    let reference_channel = 0;
    for i in 1..CHANNEL_COUNT {
      self.phase_differences[i] =
        calculate_phase_difference(&monitors[reference_channel], &monitors[i]);
      self.amplitude_ratios[i] =
        calculate_amplitude_ratio(&monitors[reference_channel], &monitors[i]);
      self.coherence_values[i] = calculate_coherence(&monitors[reference_channel], &monitors[i]);
    }
  }

  fn get_max_correlation(&self) -> f32 {
    let mut max_corr = 0.0f32;
    for i in 0..CHANNEL_COUNT {
      for j in 0..CHANNEL_COUNT {
        if i != j {
          max_corr = max_corr.max(self.correlation_matrix[i][j].abs());
        }
      }
    }
    max_corr
  }
}

/// 计算通道间相关性
fn calculate_channel_correlation(
  monitor1: &ChannelPerformanceMonitor,
  monitor2: &ChannelPerformanceMonitor,
) -> f32 {
  // 简化的相关性计算，基于平均值和RMS
  let mean_diff = (monitor1.avg_mean - monitor2.avg_mean).abs();
  let rms_diff = (monitor1.avg_rms - monitor2.avg_rms).abs();

  // 归一化相关性 (简化版本)
  let correlation = 1.0 - (mean_diff + rms_diff) / 2.0;
  correlation.max(0.0).min(1.0)
}

/// 计算相位差
fn calculate_phase_difference(
  monitor1: &ChannelPerformanceMonitor,
  monitor2: &ChannelPerformanceMonitor,
) -> f32 {
  // 简化的相位差计算
  use micromath::F32Ext;
  let ratio = if monitor1.avg_mean > 0.0 {
    monitor2.avg_mean / monitor1.avg_mean
  } else {
    1.0
  };
  ratio.atan() * 180.0 / core::f32::consts::PI
}

/// 计算幅度比
fn calculate_amplitude_ratio(
  monitor1: &ChannelPerformanceMonitor,
  monitor2: &ChannelPerformanceMonitor,
) -> f32 {
  if monitor1.avg_rms > 0.0 {
    monitor2.avg_rms / monitor1.avg_rms
  } else {
    1.0
  }
}

/// 计算相干性
fn calculate_coherence(
  monitor1: &ChannelPerformanceMonitor,
  monitor2: &ChannelPerformanceMonitor,
) -> f32 {
  // 简化的相干性计算
  let snr_product = monitor1.avg_snr * monitor2.avg_snr;
  if snr_product > 0.0 {
    use micromath::F32Ext;
    (snr_product / (snr_product + 100.0)).sqrt()
  } else {
    0.0
  }
}

/// 执行跨通道分析
fn perform_cross_channel_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  monitors: &[ChannelPerformanceMonitor; CHANNEL_COUNT],
  analyzer: &mut CrossChannelAnalyzer,
  process_count: u32,
) {
  // 更新跨通道分析
  analyzer.update_analysis(monitors);

  // 每100次处理输出跨通道分析结果
  if process_count % 100 == 0 {
    writeln!(serial, "\n=== 跨通道分析 (处理#{}) ===", process_count).ok();
    writeln!(serial, "最大相关性: {:.3}", analyzer.get_max_correlation()).ok();

    for i in 1..CHANNEL_COUNT {
      writeln!(
        serial,
        "CH0-CH{}: 相位差={:.1}°, 幅度比={:.2}, 相干性={:.3}",
        i,
        analyzer.phase_differences[i],
        analyzer.amplitude_ratios[i],
        analyzer.coherence_values[i]
      )
      .ok();
    }
    writeln!(serial, "").ok();
  }

  // 检测跨通道异常
  detect_cross_channel_anomalies(serial, analyzer, process_count);
}

/// 检测跨通道异常
fn detect_cross_channel_anomalies(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  analyzer: &CrossChannelAnalyzer,
  process_count: u32,
) {
  // 检测异常高的相关性 (可能的串扰)
  let max_correlation = analyzer.get_max_correlation();
  if max_correlation > 0.9 {
    writeln!(
      serial,
      "警告: 检测到通道间串扰 {:.3} (处理#{})",
      max_correlation, process_count
    )
    .ok();
  }

  // 检测异常的幅度比
  for i in 1..CHANNEL_COUNT {
    if analyzer.amplitude_ratios[i] > 5.0 || analyzer.amplitude_ratios[i] < 0.2 {
      writeln!(
        serial,
        "警告: CH0-CH{} 幅度比异常 {:.2} (处理#{})",
        i, analyzer.amplitude_ratios[i], process_count
      )
      .ok();
    }
  }

  // 检测低相干性
  for i in 1..CHANNEL_COUNT {
    if analyzer.coherence_values[i] < 0.3 {
      writeln!(
        serial,
        "警告: CH0-CH{} 相干性低 {:.3} (处理#{})",
        i, analyzer.coherence_values[i], process_count
      )
      .ok();
    }
  }
}

/// 输出多通道状态
fn output_multi_channel_status(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  sample_count: u32,
  process_count: u32,
  monitors: &[ChannelPerformanceMonitor; CHANNEL_COUNT],
  analyzer: &CrossChannelAnalyzer,
) {
  let buffer_fill = free(|cs| {
    let buffer = MULTI_CHANNEL_BUFFER.borrow(cs).borrow();
    buffer.get_fill_percentage()
  });

  writeln!(serial, "\n=== 多通道系统状态 ===").ok();
  writeln!(serial, "总采样数: {}", sample_count).ok();
  writeln!(serial, "处理计数: {}", process_count).ok();
  writeln!(serial, "缓冲区填充: {:.1}%", buffer_fill).ok();

  // 输出各通道状态
  for (i, monitor) in monitors.iter().enumerate() {
    writeln!(
      serial,
      "CH{}: 成功率={:.1}%, 平均SNR={:.1}dB, 溢出={}",
      i,
      monitor.get_success_rate() * 100.0,
      monitor.avg_snr,
      monitor.overflow_count
    )
    .ok();
  }

  writeln!(
    serial,
    "最大通道相关性: {:.3}",
    analyzer.get_max_correlation()
  )
  .ok();
  writeln!(serial, "").ok();
}

/// ADC值转换为电压
fn adc_to_voltage(adc_value: u16, vref: f32, resolution: u8) -> f32 {
  let max_value = (1 << resolution) - 1;
  (adc_value as f32 / max_value as f32) * vref
}

/// 模拟多通道ADC读取
fn simulate_multi_channel_adc(channel: usize, sample_count: u32) -> u16 {
  use micromath::F32Ext;

  let dc_offset = 2048.0;
  let sample_rate = 10000.0; // 每通道10kHz
  let t = (sample_count / CHANNEL_COUNT as u32) as f32 / sample_rate;

  let signal = match channel {
    0 => {
      // 通道0：1kHz正弦波
      let amplitude = 600.0;
      let frequency = 1000.0;
      amplitude * (2.0 * core::f32::consts::PI * frequency * t).sin()
    }
    1 => {
      // 通道1：2kHz正弦波，相位偏移
      let amplitude = 500.0;
      let frequency = 2000.0;
      let phase = core::f32::consts::PI / 4.0; // 45度相位偏移
      amplitude * (2.0 * core::f32::consts::PI * frequency * t + phase).sin()
    }
    2 => {
      // 通道2：500Hz方波
      let amplitude = 400.0;
      let frequency = 500.0;
      let phase = 2.0 * core::f32::consts::PI * frequency * t;
      if phase.sin() > 0.0 {
        amplitude
      } else {
        -amplitude
      }
    }
    3 => {
      // 通道3：1.5kHz三角波
      let amplitude = 300.0;
      let frequency = 1500.0;
      let phase = 2.0 * core::f32::consts::PI * frequency * t;
      amplitude * (2.0 / core::f32::consts::PI) * phase.sin().asin()
    }
    _ => 0.0,
  };

  // 添加通道特定的噪声
  let noise_level = 20.0 + channel as f32 * 5.0;
  let noise = ((sample_count * (channel as u32 + 1) * 13) % 41) as f32 - 20.0;
  let scaled_noise = noise * noise_level / 20.0;

  let result = dc_offset + signal + scaled_noise;
  result.max(0.0).min(4095.0) as u16
}

/// 定时器中断处理
#[interrupt]
fn TIM2() {
  // 在实际应用中，这里会由DMA中断处理多通道采样
}
