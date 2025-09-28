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
use signal_analyzer::{adc_to_voltage, SpectrumAnalyzer};
use stm32f4xx_hal::{
  adc::{Adc, AdcConfig, SampleTime},
  gpio::Analog,
  pac::{self, interrupt, Interrupt},
  prelude::*,
  serial::{Config, Serial},
  timer::{Event, Timer},
};

const SPECTRUM_SIZE: usize = 512;
const SAMPLE_RATE: f32 = 20000.0; // 20kHz采样率

// 全局频谱缓冲区
static SPECTRUM_BUFFER: Mutex<RefCell<[f32; SPECTRUM_SIZE]>> =
  Mutex::new(RefCell::new([0.0; SPECTRUM_SIZE]));
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
  analysis_timer.start(3.Hz()).unwrap(); // 3Hz分析频率

  // 配置状态输出定时器
  let mut status_timer = Timer::new(dp.TIM4, &clocks).counter_hz();
  status_timer.start(1.Hz()).unwrap(); // 1Hz状态输出

  // 启用定时器中断
  cp.NVIC.enable(Interrupt::TIM2);

  // 创建频谱分析器
  let mut spectrum_analyzer = SpectrumAnalyzer::<SPECTRUM_SIZE>::new(SAMPLE_RATE);
  spectrum_analyzer.set_peak_threshold(0.05);
  spectrum_analyzer.set_noise_floor(0.001);

  writeln!(serial, "频谱分析器示例启动").ok();
  writeln!(serial, "频谱大小: {}", SPECTRUM_SIZE).ok();
  writeln!(serial, "采样率: {:.0} Hz", SAMPLE_RATE).ok();
  writeln!(
    serial,
    "频率分辨率: {:.2} Hz",
    SAMPLE_RATE / SPECTRUM_SIZE as f32
  )
  .ok();
  writeln!(serial, "分析范围: 0 - {:.0} Hz", SAMPLE_RATE / 2.0).ok();
  writeln!(serial, "峰值阈值: 0.05").ok();
  writeln!(serial, "噪声底限: 0.001").ok();
  writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
  writeln!(serial, "").ok();

  let mut sample_count = 0u32;
  let mut analysis_count = 0u32;
  let vref = 3.3f32;

  // 频谱分析统计
  let mut spectrum_stats = SpectrumStatistics::new();
  let mut frequency_tracker = FrequencyTracker::new();

  loop {
    // ADC采样
    if sample_timer.wait().is_ok() {
      sample_count += 1;

      // 模拟ADC读取
      let adc_value = simulate_multi_tone_signal(sample_count);
      let voltage = adc_to_voltage(adc_value, vref, 12);

      // 存储到频谱缓冲区
      let buffer_full = free(|cs| {
        let mut buffer = SPECTRUM_BUFFER.borrow(cs).borrow_mut();
        unsafe {
          buffer[BUFFER_INDEX] = voltage;
          BUFFER_INDEX += 1;

          if BUFFER_INDEX >= SPECTRUM_SIZE {
            BUFFER_INDEX = 0;
            BUFFER_READY = true;
            true
          } else {
            false
          }
        }
      });

      if buffer_full {
        spectrum_stats.record_buffer_fill();
      }
    }

    // 频谱分析
    if analysis_timer.wait().is_ok() {
      analysis_count += 1;

      let analysis_ready = unsafe { BUFFER_READY };
      if analysis_ready {
        // 复制缓冲区进行分析
        let mut analysis_buffer = [0.0f32; SPECTRUM_SIZE];
        free(|cs| {
          let buffer = SPECTRUM_BUFFER.borrow(cs).borrow();
          analysis_buffer.copy_from_slice(&*buffer);
        });

        // 执行频谱分析
        perform_spectrum_analysis(
          &mut serial,
          &spectrum_analyzer,
          &mut analysis_buffer,
          analysis_count,
          &mut spectrum_stats,
          &mut frequency_tracker,
        );

        unsafe {
          BUFFER_READY = false;
        }
      }
    }

    // 输出系统状态
    if status_timer.wait().is_ok() {
      output_spectrum_status(
        &mut serial,
        sample_count,
        analysis_count,
        &spectrum_stats,
        &frequency_tracker,
      );
    }
  }
}

/// 执行频谱分析
fn perform_spectrum_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  analyzer: &SpectrumAnalyzer<SPECTRUM_SIZE>,
  signal: &mut [f32; SPECTRUM_SIZE],
  analysis_count: u32,
  stats: &mut SpectrumStatistics,
  tracker: &mut FrequencyTracker,
) {
  // 执行频谱分析
  let result = analyzer.analyze(signal);

  // 更新统计信息
  stats.update(&result);

  // 更新频率跟踪
  tracker.update(&result.peaks);

  // 输出分析结果
  if analysis_count % 3 == 0 {
    // 每3次分析输出一次详细结果
    output_detailed_spectrum_analysis(serial, analysis_count, &result);
  }

  // 检测频谱变化
  detect_spectrum_changes(serial, &result, tracker, analysis_count);

  // 频段能量分析
  analyze_frequency_bands(serial, &result, analysis_count);
}

/// 频谱统计信息
struct SpectrumStatistics {
  total_analyses: u32,
  buffer_fills: u32,
  avg_total_power: f32,
  avg_snr: f32,
  avg_thd: f32,
  avg_peak_count: f32,
  max_peak_count: usize,
  avg_fundamental_freq: f32,
}

impl SpectrumStatistics {
  fn new() -> Self {
    Self {
      total_analyses: 0,
      buffer_fills: 0,
      avg_total_power: 0.0,
      avg_snr: 0.0,
      avg_thd: 0.0,
      avg_peak_count: 0.0,
      max_peak_count: 0,
      avg_fundamental_freq: 0.0,
    }
  }

  fn record_buffer_fill(&mut self) {
    self.buffer_fills += 1;
  }

  fn update(&mut self, result: &signal_analyzer::SpectrumResult<SPECTRUM_SIZE>) {
    self.total_analyses += 1;

    // 更新移动平均
    let alpha = 0.1f32;
    self.avg_total_power = self.avg_total_power * (1.0 - alpha) + result.total_power * alpha;
    self.avg_snr = self.avg_snr * (1.0 - alpha) + result.snr * alpha;
    self.avg_thd = self.avg_thd * (1.0 - alpha) + result.thd * alpha;
    self.avg_peak_count = self.avg_peak_count * (1.0 - alpha) + result.peaks.len() as f32 * alpha;
    self.avg_fundamental_freq =
      self.avg_fundamental_freq * (1.0 - alpha) + result.fundamental_frequency * alpha;

    // 更新最大峰值数
    self.max_peak_count = self.max_peak_count.max(result.peaks.len());
  }
}

/// 频率跟踪器
struct FrequencyTracker {
  tracked_frequencies: heapless::Vec<TrackedFrequency, 16>,
  frequency_tolerance: f32,
}

#[derive(Debug, Clone, Copy)]
struct TrackedFrequency {
  frequency: f32,
  magnitude: f32,
  stability_count: u32,
  last_seen: u32,
  avg_magnitude: f32,
  magnitude_variance: f32,
}

impl FrequencyTracker {
  fn new() -> Self {
    Self {
      tracked_frequencies: heapless::Vec::new(),
      frequency_tolerance: 10.0, // 10Hz容差
    }
  }

  fn update(&mut self, peaks: &heapless::Vec<signal_analyzer::Peak, 32>) {
    // 更新现有频率
    for tracked in self.tracked_frequencies.iter_mut() {
      tracked.last_seen += 1;

      // 查找匹配的峰值
      let mut found = false;
      for peak in peaks.iter() {
        if (peak.frequency - tracked.frequency).abs() < self.frequency_tolerance {
          // 更新频率信息
          tracked.frequency = peak.frequency;
          tracked.magnitude = peak.magnitude;
          tracked.stability_count += 1;
          tracked.last_seen = 0;

          // 更新平均幅度
          let alpha = 0.1f32;
          tracked.avg_magnitude = tracked.avg_magnitude * (1.0 - alpha) + peak.magnitude * alpha;

          // 更新幅度方差
          let diff = peak.magnitude - tracked.avg_magnitude;
          tracked.magnitude_variance =
            tracked.magnitude_variance * (1.0 - alpha) + diff * diff * alpha;

          found = true;
          break;
        }
      }

      if !found {
        tracked.stability_count = tracked.stability_count.saturating_sub(1);
      }
    }

    // 移除长时间未见的频率
    self.tracked_frequencies.retain(|freq| freq.last_seen < 10);

    // 添加新频率
    for peak in peaks.iter() {
      let mut is_new = true;
      for tracked in self.tracked_frequencies.iter() {
        if (peak.frequency - tracked.frequency).abs() < self.frequency_tolerance {
          is_new = false;
          break;
        }
      }

      if is_new && self.tracked_frequencies.len() < 16 {
        let new_freq = TrackedFrequency {
          frequency: peak.frequency,
          magnitude: peak.magnitude,
          stability_count: 1,
          last_seen: 0,
          avg_magnitude: peak.magnitude,
          magnitude_variance: 0.0,
        };
        let _ = self.tracked_frequencies.push(new_freq);
      }
    }
  }

  fn get_stable_frequencies(&self) -> heapless::Vec<TrackedFrequency, 16> {
    let mut stable = heapless::Vec::new();
    for freq in self.tracked_frequencies.iter() {
      if freq.stability_count >= 5 {
        // 至少稳定出现5次
        let _ = stable.push(*freq);
      }
    }
    stable
  }

  fn get_frequency_count(&self) -> usize {
    self.tracked_frequencies.len()
  }

  fn get_stable_count(&self) -> usize {
    self.get_stable_frequencies().len()
  }
}

/// 输出详细频谱分析结果
fn output_detailed_spectrum_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  analysis_count: u32,
  result: &signal_analyzer::SpectrumResult<SPECTRUM_SIZE>,
) {
  writeln!(serial, "\n=== 频谱分析结果 #{} ===", analysis_count).ok();

  // 基本频谱信息
  writeln!(serial, "频谱信息:").ok();
  writeln!(serial, "  总功率: {:.6}", result.total_power).ok();
  writeln!(serial, "  信噪比: {:.1} dB", result.snr).ok();
  writeln!(serial, "  总谐波失真: {:.2}%", result.thd).ok();
  writeln!(serial, "  基频: {:.1} Hz", result.fundamental_frequency).ok();
  writeln!(serial, "  峰值数量: {}", result.peaks.len()).ok();

  // 输出主要峰值
  writeln!(serial, "主要频率峰值:").ok();
  for (i, peak) in result.peaks.iter().take(8).enumerate() {
    writeln!(
      serial,
      "  {}: {:.1} Hz ({:.4})",
      i + 1,
      peak.frequency,
      peak.magnitude
    )
    .ok();
  }

  writeln!(serial, "").ok();
}

/// 检测频谱变化
fn detect_spectrum_changes(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  result: &signal_analyzer::SpectrumResult<SPECTRUM_SIZE>,
  tracker: &FrequencyTracker,
  analysis_count: u32,
) {
  // 检测新出现的频率
  let stable_frequencies = tracker.get_stable_frequencies();
  for peak in result.peaks.iter() {
    let mut is_stable = false;
    for stable in stable_frequencies.iter() {
      if (peak.frequency - stable.frequency).abs() < 10.0 {
        is_stable = true;
        break;
      }
    }

    if !is_stable && peak.magnitude > 0.1 {
      writeln!(
        serial,
        "新频率检测: {:.1} Hz ({:.4}) (分析#{})",
        peak.frequency, peak.magnitude, analysis_count
      )
      .ok();
    }
  }

  // 检测频率消失
  for stable in stable_frequencies.iter() {
    if stable.last_seen > 3 {
      writeln!(
        serial,
        "频率消失: {:.1} Hz (分析#{})",
        stable.frequency, analysis_count
      )
      .ok();
    }
  }

  // 检测幅度显著变化
  for stable in stable_frequencies.iter() {
    if stable.magnitude_variance > 0.01 {
      writeln!(
        serial,
        "频率幅度不稳定: {:.1} Hz (方差={:.4}) (分析#{})",
        stable.frequency, stable.magnitude_variance, analysis_count
      )
      .ok();
    }
  }
}

/// 频段能量分析
fn analyze_frequency_bands(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  result: &signal_analyzer::SpectrumResult<SPECTRUM_SIZE>,
  analysis_count: u32,
) {
  // 定义频段
  let bands = [
    ("低频", 0.0, 500.0),
    ("中低频", 500.0, 2000.0),
    ("中频", 2000.0, 5000.0),
    ("中高频", 5000.0, 8000.0),
    ("高频", 8000.0, 10000.0),
  ];

  // 每10次分析输出一次频段分析
  if analysis_count % 10 == 0 {
    writeln!(serial, "频段能量分析 (分析#{}):", analysis_count).ok();

    for (name, low_freq, high_freq) in bands.iter() {
      let mut band_power = 0.0f32;
      let mut band_peak_count = 0;

      for (i, &power) in result.power_spectrum.iter().enumerate() {
        let freq = result.frequency_bins[i];
        if freq >= *low_freq && freq < *high_freq {
          band_power += power;
        }
      }

      for peak in result.peaks.iter() {
        if peak.frequency >= *low_freq && peak.frequency < *high_freq {
          band_peak_count += 1;
        }
      }

      let band_percentage = if result.total_power > 0.0 {
        band_power / result.total_power * 100.0
      } else {
        0.0
      };

      writeln!(
        serial,
        "  {}: {:.2}% ({} 峰值)",
        name, band_percentage, band_peak_count
      )
      .ok();
    }
    writeln!(serial, "").ok();
  }
}

/// 输出频谱状态
fn output_spectrum_status(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  sample_count: u32,
  analysis_count: u32,
  stats: &SpectrumStatistics,
  tracker: &FrequencyTracker,
) {
  let buffer_fill_rate = if sample_count > 0 {
    stats.buffer_fills as f32 / (sample_count as f32 / SPECTRUM_SIZE as f32) * 100.0
  } else {
    0.0
  };

  writeln!(serial, "\n=== 频谱系统状态 ===").ok();
  writeln!(serial, "采样数: {}", sample_count).ok();
  writeln!(serial, "分析数: {}", analysis_count).ok();
  writeln!(
    serial,
    "缓冲区填充: {} ({:.1}%)",
    stats.buffer_fills, buffer_fill_rate
  )
  .ok();
  writeln!(serial, "平均总功率: {:.6}", stats.avg_total_power).ok();
  writeln!(serial, "平均SNR: {:.1} dB", stats.avg_snr).ok();
  writeln!(serial, "平均THD: {:.2}%", stats.avg_thd).ok();
  writeln!(serial, "平均峰值数: {:.1}", stats.avg_peak_count).ok();
  writeln!(serial, "最大峰值数: {}", stats.max_peak_count).ok();
  writeln!(serial, "平均基频: {:.1} Hz", stats.avg_fundamental_freq).ok();
  writeln!(serial, "跟踪频率数: {}", tracker.get_frequency_count()).ok();
  writeln!(serial, "稳定频率数: {}", tracker.get_stable_count()).ok();
  writeln!(serial, "").ok();
}

/// 模拟多音调信号
fn simulate_multi_tone_signal(sample_count: u32) -> u16 {
  let t = sample_count as f32 / SAMPLE_RATE;
  let dc_offset = 2048.0;

  // 多音调信号组合
  let signal = 
        // 主音调 800Hz
        500.0 * (2.0 * core::f32::consts::PI * 800.0 * t).sin() +
        // 次音调 1200Hz
        300.0 * (2.0 * core::f32::consts::PI * 1200.0 * t).sin() +
        // 高频分量 2400Hz
        200.0 * (2.0 * core::f32::consts::PI * 2400.0 * t).sin() +
        // 低频分量 400Hz
        250.0 * (2.0 * core::f32::consts::PI * 400.0 * t).sin() +
        // 调制信号 (FM调制)
        350.0 * (2.0 * core::f32::consts::PI * 1000.0 * t + 
                 0.5 * (2.0 * core::f32::consts::PI * 100.0 * t).sin()).sin() +
        // 脉冲信号 (方波)
        if ((t * 600.0) as u32 % 2) == 0 { 150.0 } else { -150.0 } +
        // 扫频信号
        200.0 * (2.0 * core::f32::consts::PI * (500.0 + 1000.0 * (t * 0.1).sin()) * t).sin();

  // 添加有色噪声
  let noise_freq = 5000.0; // 高频噪声
  let noise = 30.0
    * (2.0 * core::f32::consts::PI * noise_freq * t).sin()
    * ((sample_count * 7 + 13) % 31) as f32
    / 31.0;

  // 添加白噪声
  let white_noise = ((sample_count * 23 + 41) % 71) as f32 - 35.0;
  let scaled_white_noise = white_noise * 10.0;

  let result = dc_offset + signal + noise + scaled_white_noise;
  result.max(0.0).min(4095.0) as u16
}

/// 定时器中断处理
#[interrupt]
fn TIM2() {
  // 在实际应用中，这里会处理ADC采样中断
}
