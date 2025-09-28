#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use heapless::{String, Vec};
use micromath::F32Ext;
use nb::block;
use panic_halt as _;
use signal_analyzer::{adc_to_voltage, Complex, FFTProcessor};
use stm32f4xx_hal::{
  adc::{Adc, AdcConfig, SampleTime},
  gpio::Analog,
  pac::{self, interrupt, Interrupt},
  prelude::*,
  serial::{Config, Serial},
  timer::{Event, Timer},
};

const FFT_SIZE: usize = 256;
const SAMPLE_RATE: f32 = 10000.0; // 10kHz采样率

// 全局FFT缓冲区
static FFT_BUFFER: Mutex<RefCell<[f32; FFT_SIZE]>> = Mutex::new(RefCell::new([0.0; FFT_SIZE]));
static mut BUFFER_INDEX: usize = 0;
static mut BUFFER_READY: bool = false;

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

  // 配置采样定时器
  let mut sample_timer = Timer::new(dp.TIM2, &clocks).counter_hz();
  sample_timer.start((SAMPLE_RATE as u32).Hz()).unwrap();

  // 配置分析定时器
  let mut analysis_timer = Timer::new(dp.TIM3, &clocks).counter_hz();
  analysis_timer.start(5.Hz()).unwrap(); // 5Hz分析频率

  // 配置状态输出定时器
  let mut status_timer = Timer::new(dp.TIM4, &clocks).counter_hz();
  status_timer.start(1.Hz()).unwrap(); // 1Hz状态输出

  // 启用定时器中断
  cp.NVIC.enable(Interrupt::TIM2);

  // 创建FFT处理器
  let fft_processor = FFTProcessor::<FFT_SIZE>::new();

  writeln!(serial, "FFT分析器示例启动").ok();
  writeln!(serial, "FFT大小: {}", FFT_SIZE).ok();
  writeln!(serial, "采样率: {:.0} Hz", SAMPLE_RATE).ok();
  writeln!(
    serial,
    "频率分辨率: {:.2} Hz",
    SAMPLE_RATE / FFT_SIZE as f32
  )
  .ok();
  writeln!(serial, "最大分析频率: {:.0} Hz", SAMPLE_RATE / 2.0).ok();
  writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
  writeln!(serial, "").ok();

  let mut sample_count = 0u32;
  let mut analysis_count = 0u32;
  let vref = 3.3f32;

  // FFT分析统计
  let mut fft_stats = FFTStatistics::new();

  loop {
    // ADC采样
    if sample_timer.wait().is_ok() {
      sample_count += 1;

      // 模拟ADC读取
      let adc_value = simulate_complex_signal(sample_count);
      let voltage = adc_to_voltage(adc_value, vref, 12);

      // 存储到FFT缓冲区
      let buffer_full = free(|cs| {
        let mut buffer = FFT_BUFFER.borrow(cs).borrow_mut();
        unsafe {
          buffer[BUFFER_INDEX] = voltage;
          BUFFER_INDEX += 1;

          if BUFFER_INDEX >= FFT_SIZE {
            BUFFER_INDEX = 0;
            BUFFER_READY = true;
            true
          } else {
            false
          }
        }
      });

      if buffer_full {
        fft_stats.record_buffer_fill();
      }
    }

    // FFT分析
    if analysis_timer.wait().is_ok() {
      analysis_count += 1;

      let analysis_ready = unsafe { BUFFER_READY };
      if analysis_ready {
        // 复制缓冲区进行分析
        let mut analysis_buffer = [0.0f32; FFT_SIZE];
        free(|cs| {
          let buffer = FFT_BUFFER.borrow(cs).borrow();
          analysis_buffer.copy_from_slice(&*buffer);
        });

        // 执行FFT分析
        perform_fft_analysis(
          &mut serial,
          &fft_processor,
          &mut analysis_buffer,
          analysis_count,
          &mut fft_stats,
        );

        unsafe {
          BUFFER_READY = false;
        }
      }
    }

    // 输出系统状态
    if status_timer.wait().is_ok() {
      output_fft_status(&mut serial, sample_count, analysis_count, &fft_stats);
    }
  }
}

/// 执行FFT分析
fn perform_fft_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  fft_processor: &FFTProcessor<FFT_SIZE>,
  signal: &mut [f32; FFT_SIZE],
  analysis_count: u32,
  stats: &mut FFTStatistics,
) {
  // 记录原始信号统计
  let original_stats = calculate_signal_statistics(signal);

  // 应用窗函数
  fft_processor.apply_window(signal);

  // 执行FFT
  let mut fft_output = [Complex::new(0.0, 0.0); FFT_SIZE];
  fft_processor.fft(signal, &mut fft_output);

  // 计算功率谱
  let mut power_spectrum = [0.0f32; FFT_SIZE];
  fft_processor.power_spectrum(&fft_output, &mut power_spectrum);

  // 计算幅度谱
  let mut magnitude_spectrum = [0.0f32; FFT_SIZE];
  fft_processor.magnitude_spectrum(&fft_output, &mut magnitude_spectrum);

  // 计算相位谱
  let mut phase_spectrum = [0.0f32; FFT_SIZE];
  fft_processor.phase_spectrum(&fft_output, &mut phase_spectrum);

  // 分析频谱特征
  let spectrum_features = analyze_spectrum_features(&magnitude_spectrum, &power_spectrum);

  // 更新统计信息
  stats.update(&original_stats, &spectrum_features);

  // 输出分析结果
  if analysis_count % 5 == 0 {
    // 每5次分析输出一次详细结果
    output_detailed_analysis(
      serial,
      analysis_count,
      &original_stats,
      &spectrum_features,
      &magnitude_spectrum,
    );
  }

  // 检测频谱异常
  detect_spectrum_anomalies(serial, &spectrum_features, analysis_count);
}

/// 信号统计信息
#[derive(Debug, Clone, Copy)]
struct SignalStatistics {
  mean: f32,
  rms: f32,
  peak: f32,
  peak_to_peak: f32,
  std_dev: f32,
  crest_factor: f32,
}

/// 计算信号统计信息
fn calculate_signal_statistics(signal: &[f32; FFT_SIZE]) -> SignalStatistics {
  let sum: f32 = signal.iter().sum();
  let mean = sum / signal.len() as f32;

  let max = signal.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
  let min = signal.iter().fold(f32::INFINITY, |a, &b| a.min(b));
  let peak = max.abs().max(min.abs());
  let peak_to_peak = max - min;

  // RMS计算
  let sum_squares: f32 = signal.iter().map(|&x| x * x).sum();
  let rms = (sum_squares / signal.len() as f32).sqrt();

  // 标准差
  let variance: f32 = signal
    .iter()
    .map(|&x| {
      let diff = x - mean;
      diff * diff
    })
    .sum::<f32>()
    / signal.len() as f32;
  let std_dev = variance.sqrt();

  // 峰值因子
  let crest_factor = if rms > 0.0 { peak / rms } else { 0.0 };

  SignalStatistics {
    mean,
    rms,
    peak,
    peak_to_peak,
    std_dev,
    crest_factor,
  }
}

/// 频谱特征
#[derive(Debug, Clone)]
struct SpectrumFeatures {
  dominant_frequency: f32,
  dominant_magnitude: f32,
  total_power: f32,
  spectral_centroid: f32,
  spectral_spread: f32,
  spectral_rolloff: f32,
  spectral_flatness: f32,
  peak_count: usize,
  snr_estimate: f32,
}

/// 分析频谱特征
fn analyze_spectrum_features(
  magnitude: &[f32; FFT_SIZE],
  power: &[f32; FFT_SIZE],
) -> SpectrumFeatures {
  let half_size = FFT_SIZE / 2;
  let freq_resolution = SAMPLE_RATE / FFT_SIZE as f32;

  // 找到主导频率
  let mut max_magnitude = 0.0f32;
  let mut dominant_bin = 0;
  for i in 1..half_size {
    if magnitude[i] > max_magnitude {
      max_magnitude = magnitude[i];
      dominant_bin = i;
    }
  }
  let dominant_frequency = dominant_bin as f32 * freq_resolution;

  // 计算总功率
  let total_power: f32 = power[1..half_size].iter().sum();

  // 计算频谱质心
  let mut weighted_sum = 0.0f32;
  for i in 1..half_size {
    weighted_sum += i as f32 * magnitude[i];
  }
  let magnitude_sum: f32 = magnitude[1..half_size].iter().sum();
  let spectral_centroid = if magnitude_sum > 0.0 {
    weighted_sum / magnitude_sum * freq_resolution
  } else {
    0.0
  };

  // 计算频谱扩散
  let mut spread_sum = 0.0f32;
  for i in 1..half_size {
    let freq = i as f32 * freq_resolution;
    let diff = freq - spectral_centroid;
    spread_sum += diff * diff * magnitude[i];
  }
  let spectral_spread = if magnitude_sum > 0.0 {
    (spread_sum / magnitude_sum).sqrt()
  } else {
    0.0
  };

  // 计算频谱滚降点 (85%能量点)
  let target_energy = total_power * 0.85;
  let mut cumulative_power = 0.0f32;
  let mut rolloff_bin = half_size - 1;
  for i in 1..half_size {
    cumulative_power += power[i];
    if cumulative_power >= target_energy {
      rolloff_bin = i;
      break;
    }
  }
  let spectral_rolloff = rolloff_bin as f32 * freq_resolution;

  // 计算频谱平坦度
  let geometric_mean = calculate_geometric_mean(&magnitude[1..half_size]);
  let arithmetic_mean = magnitude_sum / (half_size - 1) as f32;
  let spectral_flatness = if arithmetic_mean > 0.0 {
    geometric_mean / arithmetic_mean
  } else {
    0.0
  };

  // 计算峰值数量
  let peak_count = count_spectral_peaks(&magnitude[1..half_size], max_magnitude * 0.1);

  // 估算信噪比
  let snr_estimate = estimate_snr(&magnitude[1..half_size], dominant_bin - 1);

  SpectrumFeatures {
    dominant_frequency,
    dominant_magnitude: max_magnitude,
    total_power,
    spectral_centroid,
    spectral_spread,
    spectral_rolloff,
    spectral_flatness,
    peak_count,
    snr_estimate,
  }
}

/// 计算几何平均数
fn calculate_geometric_mean(data: &[f32]) -> f32 {
  if data.is_empty() {
    return 0.0;
  }

  let mut log_sum = 0.0f32;
  let mut count = 0;

  for &value in data {
    if value > 1e-10 {
      // 避免log(0)
      log_sum += value.ln();
      count += 1;
    }
  }

  if count > 0 {
    (log_sum / count as f32).exp()
  } else {
    0.0
  }
}

/// 计算频谱峰值数量
fn count_spectral_peaks(magnitude: &[f32], threshold: f32) -> usize {
  let mut peak_count = 0;

  for i in 1..magnitude.len() - 1 {
    if magnitude[i] > threshold
      && magnitude[i] > magnitude[i - 1]
      && magnitude[i] > magnitude[i + 1]
    {
      peak_count += 1;
    }
  }

  peak_count
}

/// 估算信噪比
fn estimate_snr(magnitude: &[f32], signal_bin: usize) -> f32 {
  if signal_bin >= magnitude.len() {
    return 0.0;
  }

  let signal_power = magnitude[signal_bin] * magnitude[signal_bin];

  // 估算噪声功率（排除信号附近的频段）
  let mut noise_power = 0.0f32;
  let mut noise_count = 0;

  let exclusion_range = 5; // 排除信号附近5个频段
  for (i, &mag) in magnitude.iter().enumerate() {
    if (i as i32 - signal_bin as i32).abs() > exclusion_range {
      noise_power += mag * mag;
      noise_count += 1;
    }
  }

  if noise_count > 0 {
    noise_power /= noise_count as f32;

    if noise_power > 0.0 {
      10.0 * (signal_power / noise_power).log10()
    } else {
      100.0
    }
  } else {
    0.0
  }
}

/// FFT统计信息
struct FFTStatistics {
  total_analyses: u32,
  buffer_fills: u32,
  avg_dominant_freq: f32,
  avg_total_power: f32,
  avg_snr: f32,
  max_peak_count: usize,
  avg_spectral_centroid: f32,
  avg_spectral_flatness: f32,
}

impl FFTStatistics {
  fn new() -> Self {
    Self {
      total_analyses: 0,
      buffer_fills: 0,
      avg_dominant_freq: 0.0,
      avg_total_power: 0.0,
      avg_snr: 0.0,
      max_peak_count: 0,
      avg_spectral_centroid: 0.0,
      avg_spectral_flatness: 0.0,
    }
  }

  fn record_buffer_fill(&mut self) {
    self.buffer_fills += 1;
  }

  fn update(&mut self, _signal_stats: &SignalStatistics, spectrum_features: &SpectrumFeatures) {
    self.total_analyses += 1;

    // 更新移动平均
    let alpha = 0.1f32; // 平滑因子
    self.avg_dominant_freq =
      self.avg_dominant_freq * (1.0 - alpha) + spectrum_features.dominant_frequency * alpha;
    self.avg_total_power =
      self.avg_total_power * (1.0 - alpha) + spectrum_features.total_power * alpha;
    self.avg_snr = self.avg_snr * (1.0 - alpha) + spectrum_features.snr_estimate * alpha;
    self.avg_spectral_centroid =
      self.avg_spectral_centroid * (1.0 - alpha) + spectrum_features.spectral_centroid * alpha;
    self.avg_spectral_flatness =
      self.avg_spectral_flatness * (1.0 - alpha) + spectrum_features.spectral_flatness * alpha;

    // 更新最大峰值数
    self.max_peak_count = self.max_peak_count.max(spectrum_features.peak_count);
  }
}

/// 输出详细分析结果
fn output_detailed_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  analysis_count: u32,
  signal_stats: &SignalStatistics,
  spectrum_features: &SpectrumFeatures,
  magnitude: &[f32; FFT_SIZE],
) {
  writeln!(serial, "\n=== FFT分析结果 #{} ===", analysis_count).ok();

  // 时域统计
  writeln!(serial, "时域统计:").ok();
  writeln!(serial, "  均值: {:.4}V", signal_stats.mean).ok();
  writeln!(serial, "  RMS: {:.4}V", signal_stats.rms).ok();
  writeln!(serial, "  峰值: {:.4}V", signal_stats.peak).ok();
  writeln!(serial, "  峰峰值: {:.4}V", signal_stats.peak_to_peak).ok();
  writeln!(serial, "  峰值因子: {:.2}", signal_stats.crest_factor).ok();

  // 频域特征
  writeln!(serial, "频域特征:").ok();
  writeln!(
    serial,
    "  主导频率: {:.1} Hz",
    spectrum_features.dominant_frequency
  )
  .ok();
  writeln!(
    serial,
    "  主导幅度: {:.4}",
    spectrum_features.dominant_magnitude
  )
  .ok();
  writeln!(serial, "  总功率: {:.6}", spectrum_features.total_power).ok();
  writeln!(
    serial,
    "  频谱质心: {:.1} Hz",
    spectrum_features.spectral_centroid
  )
  .ok();
  writeln!(
    serial,
    "  频谱扩散: {:.1} Hz",
    spectrum_features.spectral_spread
  )
  .ok();
  writeln!(
    serial,
    "  频谱滚降: {:.1} Hz",
    spectrum_features.spectral_rolloff
  )
  .ok();
  writeln!(
    serial,
    "  频谱平坦度: {:.3}",
    spectrum_features.spectral_flatness
  )
  .ok();
  writeln!(serial, "  峰值数量: {}", spectrum_features.peak_count).ok();
  writeln!(
    serial,
    "  SNR估算: {:.1} dB",
    spectrum_features.snr_estimate
  )
  .ok();

  // 输出前10个最强的频率分量
  writeln!(serial, "主要频率分量:").ok();
  output_top_frequencies(serial, magnitude);

  writeln!(serial, "").ok();
}

/// 输出主要频率分量
fn output_top_frequencies(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  magnitude: &[f32; FFT_SIZE],
) {
  let half_size = FFT_SIZE / 2;
  let freq_resolution = SAMPLE_RATE / FFT_SIZE as f32;

  // 创建频率-幅度对
  let mut freq_mag_pairs: heapless::Vec<(f32, f32), 128> = heapless::Vec::new();

  for i in 1..half_size {
    let freq = i as f32 * freq_resolution;
    let mag = magnitude[i];
    if mag > 0.01 {
      // 只考虑显著的分量
      let _ = freq_mag_pairs.push((freq, mag));
    }
  }

  // 按幅度排序
  freq_mag_pairs.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(core::cmp::Ordering::Equal));

  // 输出前10个
  for (i, (freq, mag)) in freq_mag_pairs.iter().take(10).enumerate() {
    writeln!(serial, "  {}: {:.1} Hz ({:.4})", i + 1, freq, mag).ok();
  }
}

/// 检测频谱异常
fn detect_spectrum_anomalies(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  features: &SpectrumFeatures,
  analysis_count: u32,
) {
  // 检测异常高的峰值数量
  if features.peak_count > 20 {
    writeln!(
      serial,
      "警告: 检测到过多频谱峰值 {} (分析#{})",
      features.peak_count, analysis_count
    )
    .ok();
  }

  // 检测异常低的SNR
  if features.snr_estimate < 10.0 {
    writeln!(
      serial,
      "警告: SNR过低 {:.1}dB (分析#{})",
      features.snr_estimate, analysis_count
    )
    .ok();
  }

  // 检测异常的频谱平坦度
  if features.spectral_flatness > 0.8 {
    writeln!(
      serial,
      "警告: 频谱过于平坦 {:.3} (可能为噪声) (分析#{})",
      features.spectral_flatness, analysis_count
    )
    .ok();
  }

  // 检测异常的主导频率
  if features.dominant_frequency > SAMPLE_RATE * 0.4 {
    writeln!(
      serial,
      "警告: 主导频率过高 {:.1}Hz (分析#{})",
      features.dominant_frequency, analysis_count
    )
    .ok();
  }
}

/// 输出FFT状态
fn output_fft_status(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  sample_count: u32,
  analysis_count: u32,
  stats: &FFTStatistics,
) {
  let buffer_fill_rate = if sample_count > 0 {
    stats.buffer_fills as f32 / (sample_count as f32 / FFT_SIZE as f32) * 100.0
  } else {
    0.0
  };

  writeln!(serial, "\n=== FFT系统状态 ===").ok();
  writeln!(serial, "采样数: {}", sample_count).ok();
  writeln!(serial, "分析数: {}", analysis_count).ok();
  writeln!(
    serial,
    "缓冲区填充: {} ({:.1}%)",
    stats.buffer_fills, buffer_fill_rate
  )
  .ok();
  writeln!(serial, "平均主导频率: {:.1} Hz", stats.avg_dominant_freq).ok();
  writeln!(serial, "平均总功率: {:.6}", stats.avg_total_power).ok();
  writeln!(serial, "平均SNR: {:.1} dB", stats.avg_snr).ok();
  writeln!(serial, "最大峰值数: {}", stats.max_peak_count).ok();
  writeln!(
    serial,
    "平均频谱质心: {:.1} Hz",
    stats.avg_spectral_centroid
  )
  .ok();
  writeln!(serial, "平均频谱平坦度: {:.3}", stats.avg_spectral_flatness).ok();
  writeln!(serial, "").ok();
}

/// 模拟复杂信号
fn simulate_complex_signal(sample_count: u32) -> u16 {
  let t = sample_count as f32 / SAMPLE_RATE;
  let dc_offset = 2048.0;

  // 多频率分量信号
  let signal = 
        // 基频 1kHz
        400.0 * (2.0 * core::f32::consts::PI * 1000.0 * t).sin() +
        // 二次谐波 2kHz
        200.0 * (2.0 * core::f32::consts::PI * 2000.0 * t).sin() +
        // 三次谐波 3kHz
        100.0 * (2.0 * core::f32::consts::PI * 3000.0 * t).sin() +
        // 高频分量 1.5kHz
        150.0 * (2.0 * core::f32::consts::PI * 1500.0 * t).sin() +
        // 调制信号 (AM调制)
        300.0 * (2.0 * core::f32::consts::PI * 800.0 * t).sin() * 
        (1.0 + 0.5 * (2.0 * core::f32::consts::PI * 50.0 * t).sin());

  // 添加白噪声
  let noise = ((sample_count * 17 + 23) % 101) as f32 - 50.0;
  let scaled_noise = noise * 15.0;

  let result = dc_offset + signal + scaled_noise;
  result.max(0.0).min(4095.0) as u16
}

/// 定时器中断处理
#[interrupt]
fn TIM2() {
  // 在实际应用中，这里会处理ADC采样中断
}
