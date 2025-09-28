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
use signal_analyzer::{
  adc_to_voltage, DigitalFilter, FilterType, HarmonicAnalyzer, RealTimeAnalyzer, SpectrumAnalyzer,
};
use stm32f4xx_hal::{
  adc::{Adc, AdcConfig, SampleTime},
  gpio::Analog,
  pac::{self, interrupt, Interrupt},
  prelude::*,
  serial::{Config, Serial},
  timer::{Event, Timer},
};

const REALTIME_SIZE: usize = 256;
const SAMPLE_RATE: f32 = 16000.0; // 16kHz采样率
const ANALYSIS_WINDOW_MS: u32 = 100; // 100ms分析窗口

// 全局实时缓冲区
static REALTIME_BUFFER: Mutex<RefCell<[f32; REALTIME_SIZE]>> =
  Mutex::new(RefCell::new([0.0; REALTIME_SIZE]));
static mut BUFFER_INDEX: usize = 0;
static mut SAMPLE_READY: bool = false;

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

  // 配置实时分析定时器
  let mut realtime_timer = Timer::new(dp.TIM3, &clocks).counter_hz();
  realtime_timer.start(10.Hz()).unwrap(); // 10Hz实时分析

  // 配置状态输出定时器
  let mut status_timer = Timer::new(dp.TIM4, &clocks).counter_hz();
  status_timer.start(2.Hz()).unwrap(); // 2Hz状态输出

  // 配置模式切换定时器
  let mut mode_switch_timer = Timer::new(dp.TIM5, &clocks).counter_hz();
  mode_switch_timer.start(0.2.Hz()).unwrap(); // 每5秒切换模式

  // 启用定时器中断
  cp.NVIC.enable(Interrupt::TIM2);

  // 创建实时分析器
  let mut realtime_analyzer = RealTimeAnalyzer::<REALTIME_SIZE>::new(SAMPLE_RATE);
  let spectrum_analyzer = SpectrumAnalyzer::<REALTIME_SIZE>::new(SAMPLE_RATE);
  let harmonic_analyzer = HarmonicAnalyzer::<REALTIME_SIZE>::new(SAMPLE_RATE);

  // 创建滤波器
  let mut prefilter = DigitalFilter::new_butterworth_lowpass(8000.0, SAMPLE_RATE, 4);

  writeln!(serial, "实时信号分析器启动").ok();
  writeln!(serial, "缓冲区大小: {}", REALTIME_SIZE).ok();
  writeln!(serial, "采样率: {:.0} Hz", SAMPLE_RATE).ok();
  writeln!(serial, "分析窗口: {} ms", ANALYSIS_WINDOW_MS).ok();
  writeln!(serial, "实时分析频率: 10 Hz").ok();
  writeln!(serial, "分析模式:").ok();
  writeln!(serial, "  1. 基础实时分析").ok();
  writeln!(serial, "  2. 频谱实时分析").ok();
  writeln!(serial, "  3. 谐波实时分析").ok();
  writeln!(serial, "  4. 综合实时分析").ok();
  writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
  writeln!(serial, "").ok();

  let mut sample_count = 0u32;
  let mut analysis_count = 0u32;
  let mut current_mode = 0usize;
  let vref = 3.3f32;

  // 实时分析统计
  let mut realtime_stats = RealTimeAnalysisStatistics::new();

  // 性能监控
  let mut performance_monitor = RealTimePerformanceMonitor::new();

  loop {
    // ADC采样
    if sample_timer.wait().is_ok() {
      let start_time = get_timestamp();
      sample_count += 1;

      // 模拟ADC读取
      let adc_value = simulate_realtime_signal(sample_count);
      let voltage = adc_to_voltage(adc_value, vref, 12);

      // 预滤波
      let filtered_voltage = prefilter.process(voltage);

      // 添加样本到实时分析器
      realtime_analyzer.add_sample(filtered_voltage);

      // 存储到缓冲区用于其他分析
      let sample_added = free(|cs| {
        let mut buffer = REALTIME_BUFFER.borrow(cs).borrow_mut();
        unsafe {
          buffer[BUFFER_INDEX] = filtered_voltage;
          BUFFER_INDEX += 1;

          if BUFFER_INDEX >= REALTIME_SIZE {
            BUFFER_INDEX = 0;
            SAMPLE_READY = true;
            true
          } else {
            false
          }
        }
      });

      if sample_added {
        realtime_stats.record_buffer_fill();
      }

      let end_time = get_timestamp();
      performance_monitor.record_sampling_time(end_time - start_time);
    }

    // 实时分析
    if realtime_timer.wait().is_ok() {
      let start_time = get_timestamp();
      analysis_count += 1;

      // 检查实时分析器是否准备好
      if realtime_analyzer.is_ready() {
        // 执行实时分析
        let realtime_result = realtime_analyzer.analyze();

        // 根据当前模式执行不同的分析
        match current_mode {
          0 => perform_basic_realtime_analysis(
            &mut serial,
            &realtime_result,
            analysis_count,
            &mut realtime_stats,
          ),
          1 => perform_spectrum_realtime_analysis(
            &mut serial,
            &spectrum_analyzer,
            analysis_count,
            &mut realtime_stats,
          ),
          2 => perform_harmonic_realtime_analysis(
            &mut serial,
            &harmonic_analyzer,
            analysis_count,
            &mut realtime_stats,
          ),
          3 => perform_comprehensive_realtime_analysis(
            &mut serial,
            &realtime_result,
            &spectrum_analyzer,
            &harmonic_analyzer,
            analysis_count,
            &mut realtime_stats,
          ),
          _ => {}
        }

        // 重置实时分析器
        realtime_analyzer.reset();
      }

      let end_time = get_timestamp();
      performance_monitor.record_analysis_time(end_time - start_time);
    }

    // 切换分析模式
    if mode_switch_timer.wait().is_ok() {
      current_mode = (current_mode + 1) % 4;

      let mode_name = match current_mode {
        0 => "基础实时分析",
        1 => "频谱实时分析",
        2 => "谐波实时分析",
        3 => "综合实时分析",
        _ => "未知模式",
      };

      writeln!(serial, "\n>>> 切换到: {} <<<", mode_name).ok();
      realtime_stats.record_mode_switch(current_mode);
    }

    // 输出系统状态
    if status_timer.wait().is_ok() {
      output_realtime_status(
        &mut serial,
        sample_count,
        analysis_count,
        current_mode,
        &realtime_stats,
        &performance_monitor,
      );
    }
  }
}

/// 实时分析统计
struct RealTimeAnalysisStatistics {
  total_analyses: u32,
  buffer_fills: u32,
  mode_switch_count: u32,
  mode_usage_count: [u32; 4],
  signal_characteristics: SignalCharacteristics,
  anomaly_count: u32,
  peak_detection_count: u32,
}

#[derive(Debug, Clone, Copy)]
struct SignalCharacteristics {
  avg_amplitude: f32,
  avg_frequency: f32,
  avg_snr: f32,
  avg_thd: f32,
  signal_stability: f32,
  noise_level: f32,
}

impl SignalCharacteristics {
  fn new() -> Self {
    Self {
      avg_amplitude: 0.0,
      avg_frequency: 0.0,
      avg_snr: 0.0,
      avg_thd: 0.0,
      signal_stability: 0.0,
      noise_level: 0.0,
    }
  }
}

impl RealTimeAnalysisStatistics {
  fn new() -> Self {
    Self {
      total_analyses: 0,
      buffer_fills: 0,
      mode_switch_count: 0,
      mode_usage_count: [0; 4],
      signal_characteristics: SignalCharacteristics::new(),
      anomaly_count: 0,
      peak_detection_count: 0,
    }
  }

  fn record_buffer_fill(&mut self) {
    self.buffer_fills += 1;
  }

  fn record_mode_switch(&mut self, mode: usize) {
    self.mode_switch_count += 1;
    if mode < 4 {
      self.mode_usage_count[mode] += 1;
    }
  }

  fn update_signal_characteristics(
    &mut self,
    result: &signal_analyzer::RealTimeResult<REALTIME_SIZE>,
  ) {
    self.total_analyses += 1;
    let alpha = 0.1f32;

    // 更新平均值
    self.signal_characteristics.avg_amplitude =
      self.signal_characteristics.avg_amplitude * (1.0 - alpha) + result.rms_amplitude * alpha;

    self.signal_characteristics.avg_frequency =
      self.signal_characteristics.avg_frequency * (1.0 - alpha) + result.dominant_frequency * alpha;

    // 更新信号稳定性
    let stability = if result.frequency_stability > 0.0 {
      1.0 / (1.0 + result.frequency_stability)
    } else {
      1.0
    };
    self.signal_characteristics.signal_stability =
      self.signal_characteristics.signal_stability * (1.0 - alpha) + stability * alpha;

    // 更新噪声水平
    self.signal_characteristics.noise_level =
      self.signal_characteristics.noise_level * (1.0 - alpha) + result.noise_level * alpha;
  }

  fn record_anomaly(&mut self) {
    self.anomaly_count += 1;
  }

  fn record_peak_detection(&mut self) {
    self.peak_detection_count += 1;
  }
}

/// 实时性能监控器
struct RealTimePerformanceMonitor {
  avg_sampling_time: f32,
  avg_analysis_time: f32,
  max_sampling_time: u32,
  max_analysis_time: u32,
  sampling_overruns: u32,
  analysis_overruns: u32,
}

impl RealTimePerformanceMonitor {
  fn new() -> Self {
    Self {
      avg_sampling_time: 0.0,
      avg_analysis_time: 0.0,
      max_sampling_time: 0,
      max_analysis_time: 0,
      sampling_overruns: 0,
      analysis_overruns: 0,
    }
  }

  fn record_sampling_time(&mut self, time_us: u32) {
    let alpha = 0.1f32;
    self.avg_sampling_time = self.avg_sampling_time * (1.0 - alpha) + time_us as f32 * alpha;
    self.max_sampling_time = self.max_sampling_time.max(time_us);

    // 检查采样超时 (假设目标是50us)
    if time_us > 50 {
      self.sampling_overruns += 1;
    }
  }

  fn record_analysis_time(&mut self, time_us: u32) {
    let alpha = 0.1f32;
    self.avg_analysis_time = self.avg_analysis_time * (1.0 - alpha) + time_us as f32 * alpha;
    self.max_analysis_time = self.max_analysis_time.max(time_us);

    // 检查分析超时 (假设目标是1000us)
    if time_us > 1000 {
      self.analysis_overruns += 1;
    }
  }

  fn get_cpu_utilization(&self) -> f32 {
    // 简化的CPU利用率计算
    let sampling_load = self.avg_sampling_time / 62.5; // 16kHz采样周期
    let analysis_load = self.avg_analysis_time / 100000.0; // 10Hz分析周期
    (sampling_load + analysis_load) * 100.0
  }
}

/// 执行基础实时分析
fn perform_basic_realtime_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  result: &signal_analyzer::RealTimeResult<REALTIME_SIZE>,
  analysis_count: u32,
  stats: &mut RealTimeAnalysisStatistics,
) {
  // 更新统计信息
  stats.update_signal_characteristics(result);

  // 输出基础分析结果
  if analysis_count % 5 == 0 {
    // 每5次分析输出一次
    writeln!(serial, "\n=== 基础实时分析 #{} ===", analysis_count).ok();
    writeln!(serial, "RMS幅度: {:.4} V", result.rms_amplitude).ok();
    writeln!(serial, "峰值幅度: {:.4} V", result.peak_amplitude).ok();
    writeln!(serial, "主导频率: {:.1} Hz", result.dominant_frequency).ok();
    writeln!(serial, "频率稳定性: {:.3}", result.frequency_stability).ok();
    writeln!(serial, "噪声水平: {:.4} V", result.noise_level).ok();
    writeln!(serial, "信号质量: {:.1}%", result.signal_quality * 100.0).ok();

    // 检测异常
    detect_realtime_anomalies(serial, result, stats);

    writeln!(serial, "").ok();
  }

  // 实时警报
  check_realtime_alerts(serial, result, analysis_count);
}

/// 执行频谱实时分析
fn perform_spectrum_realtime_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  spectrum_analyzer: &SpectrumAnalyzer<REALTIME_SIZE>,
  analysis_count: u32,
  stats: &mut RealTimeAnalysisStatistics,
) {
  let analysis_ready = unsafe { SAMPLE_READY };
  if !analysis_ready {
    return;
  }

  // 复制缓冲区进行频谱分析
  let mut analysis_buffer = [0.0f32; REALTIME_SIZE];
  free(|cs| {
    let buffer = REALTIME_BUFFER.borrow(cs).borrow();
    analysis_buffer.copy_from_slice(&*buffer);
  });

  // 执行频谱分析
  let spectrum_result = spectrum_analyzer.analyze(&mut analysis_buffer);

  // 输出频谱分析结果
  if analysis_count % 3 == 0 {
    // 每3次分析输出一次
    writeln!(serial, "\n=== 频谱实时分析 #{} ===", analysis_count).ok();
    writeln!(serial, "总功率: {:.6}", spectrum_result.total_power).ok();
    writeln!(
      serial,
      "基频: {:.1} Hz",
      spectrum_result.fundamental_frequency
    )
    .ok();
    writeln!(serial, "信噪比: {:.1} dB", spectrum_result.snr).ok();
    writeln!(serial, "峰值数量: {}", spectrum_result.peaks.len()).ok();

    // 显示主要频率分量
    writeln!(serial, "主要频率分量:").ok();
    for (i, peak) in spectrum_result.peaks.iter().take(3).enumerate() {
      writeln!(
        serial,
        "  {}: {:.1} Hz ({:.4})",
        i + 1,
        peak.frequency,
        peak.magnitude
      )
      .ok();
    }

    // 频谱变化检测
    detect_spectrum_changes(serial, &spectrum_result, analysis_count);

    writeln!(serial, "").ok();
  }

  unsafe {
    SAMPLE_READY = false;
  }
}

/// 执行谐波实时分析
fn perform_harmonic_realtime_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  harmonic_analyzer: &HarmonicAnalyzer<REALTIME_SIZE>,
  analysis_count: u32,
  stats: &mut RealTimeAnalysisStatistics,
) {
  let analysis_ready = unsafe { SAMPLE_READY };
  if !analysis_ready {
    return;
  }

  // 复制缓冲区进行谐波分析
  let mut analysis_buffer = [0.0f32; REALTIME_SIZE];
  free(|cs| {
    let buffer = REALTIME_BUFFER.borrow(cs).borrow();
    analysis_buffer.copy_from_slice(&*buffer);
  });

  // 执行谐波分析
  let harmonic_result = harmonic_analyzer.analyze(&mut analysis_buffer);

  // 输出谐波分析结果
  if analysis_count % 4 == 0 {
    // 每4次分析输出一次
    writeln!(serial, "\n=== 谐波实时分析 #{} ===", analysis_count).ok();
    writeln!(
      serial,
      "基频: {:.1} Hz",
      harmonic_result.fundamental_frequency
    )
    .ok();
    writeln!(serial, "基频功率: {:.6}", harmonic_result.fundamental_power).ok();
    writeln!(serial, "THD: {:.2}%", harmonic_result.thd).ok();
    writeln!(serial, "谐波数量: {}", harmonic_result.harmonics.len()).ok();

    // 显示主要谐波
    writeln!(serial, "主要谐波:").ok();
    for (i, harmonic) in harmonic_result.harmonics.iter().take(5).enumerate() {
      let percentage = if harmonic_result.fundamental_power > 0.0 {
        (harmonic.power / harmonic_result.fundamental_power) * 100.0
      } else {
        0.0
      };
      writeln!(
        serial,
        "  H{}: {:.1} Hz ({:.1}%)",
        harmonic.order, harmonic.frequency, percentage
      )
      .ok();
    }

    // THD趋势分析
    analyze_thd_trend(serial, &harmonic_result, analysis_count);

    writeln!(serial, "").ok();
  }

  unsafe {
    SAMPLE_READY = false;
  }
}

/// 执行综合实时分析
fn perform_comprehensive_realtime_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  realtime_result: &signal_analyzer::RealTimeResult<REALTIME_SIZE>,
  spectrum_analyzer: &SpectrumAnalyzer<REALTIME_SIZE>,
  harmonic_analyzer: &HarmonicAnalyzer<REALTIME_SIZE>,
  analysis_count: u32,
  stats: &mut RealTimeAnalysisStatistics,
) {
  // 更新统计信息
  stats.update_signal_characteristics(realtime_result);

  let analysis_ready = unsafe { SAMPLE_READY };
  if !analysis_ready {
    return;
  }

  // 复制缓冲区进行分析
  let mut analysis_buffer = [0.0f32; REALTIME_SIZE];
  free(|cs| {
    let buffer = REALTIME_BUFFER.borrow(cs).borrow();
    analysis_buffer.copy_from_slice(&*buffer);
  });

  // 执行所有分析
  let spectrum_result = spectrum_analyzer.analyze(&mut analysis_buffer.clone());
  let harmonic_result = harmonic_analyzer.analyze(&mut analysis_buffer);

  // 输出综合分析结果
  if analysis_count % 6 == 0 {
    // 每6次分析输出一次
    writeln!(serial, "\n=== 综合实时分析 #{} ===", analysis_count).ok();

    // 时域分析
    writeln!(serial, "时域分析:").ok();
    writeln!(serial, "  RMS: {:.4} V", realtime_result.rms_amplitude).ok();
    writeln!(serial, "  峰值: {:.4} V", realtime_result.peak_amplitude).ok();
    writeln!(serial, "  噪声: {:.4} V", realtime_result.noise_level).ok();

    // 频域分析
    writeln!(serial, "频域分析:").ok();
    writeln!(
      serial,
      "  主频: {:.1} Hz",
      spectrum_result.fundamental_frequency
    )
    .ok();
    writeln!(serial, "  SNR: {:.1} dB", spectrum_result.snr).ok();
    writeln!(serial, "  峰值数: {}", spectrum_result.peaks.len()).ok();

    // 谐波分析
    writeln!(serial, "谐波分析:").ok();
    writeln!(serial, "  THD: {:.2}%", harmonic_result.thd).ok();
    writeln!(serial, "  谐波数: {}", harmonic_result.harmonics.len()).ok();

    // 信号质量评估
    let overall_quality =
      assess_overall_signal_quality(realtime_result, &spectrum_result, &harmonic_result);
    writeln!(serial, "整体质量: {:.1}%", overall_quality * 100.0).ok();

    // 综合异常检测
    detect_comprehensive_anomalies(
      serial,
      realtime_result,
      &spectrum_result,
      &harmonic_result,
      stats,
    );

    writeln!(serial, "").ok();
  }

  unsafe {
    SAMPLE_READY = false;
  }
}

/// 检测实时异常
fn detect_realtime_anomalies(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  result: &signal_analyzer::RealTimeResult<REALTIME_SIZE>,
  stats: &mut RealTimeAnalysisStatistics,
) {
  let mut anomalies = heapless::Vec::<&str, 10>::new();

  // 检查幅度异常
  if result.rms_amplitude > 3.0 {
    anomalies.push("RMS幅度过高").ok();
  } else if result.rms_amplitude < 0.01 {
    anomalies.push("RMS幅度过低").ok();
  }

  // 检查频率异常
  if result.dominant_frequency > 8000.0 {
    anomalies.push("主导频率过高").ok();
  } else if result.dominant_frequency < 10.0 {
    anomalies.push("主导频率过低").ok();
  }

  // 检查噪声异常
  if result.noise_level > 0.5 {
    anomalies.push("噪声水平过高").ok();
  }

  // 检查信号质量异常
  if result.signal_quality < 0.3 {
    anomalies.push("信号质量差").ok();
  }

  // 检查频率稳定性异常
  if result.frequency_stability > 0.1 {
    anomalies.push("频率不稳定").ok();
  }

  if !anomalies.is_empty() {
    writeln!(serial, "实时异常检测:").ok();
    for anomaly in anomalies.iter() {
      writeln!(serial, "  ⚠️ {}", anomaly).ok();
    }
    stats.record_anomaly();
  }
}

/// 检查实时警报
fn check_realtime_alerts(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  result: &signal_analyzer::RealTimeResult<REALTIME_SIZE>,
  analysis_count: u32,
) {
  // 峰值检测警报
  if result.peak_amplitude > 2.5 {
    writeln!(
      serial,
      "🚨 峰值警报: {:.3} V (分析#{})",
      result.peak_amplitude, analysis_count
    )
    .ok();
  }

  // 频率跳变警报
  static mut LAST_FREQUENCY: f32 = 0.0;
  unsafe {
    if LAST_FREQUENCY > 0.0 {
      let freq_change = (result.dominant_frequency - LAST_FREQUENCY).abs();
      if freq_change > 100.0 {
        writeln!(
          serial,
          "🚨 频率跳变: {:.1} Hz -> {:.1} Hz (分析#{})",
          LAST_FREQUENCY, result.dominant_frequency, analysis_count
        )
        .ok();
      }
    }
    LAST_FREQUENCY = result.dominant_frequency;
  }

  // 信号丢失警报
  if result.rms_amplitude < 0.005 {
    writeln!(serial, "🚨 信号丢失警报 (分析#{})", analysis_count).ok();
  }
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
  result: &signal_analyzer::SpectrumResult<REALTIME_SIZE>,
  analysis_count: u32,
) {
  static mut LAST_PEAK_COUNT: usize = 0;
  static mut LAST_TOTAL_POWER: f32 = 0.0;

  unsafe {
    // 检测峰值数量变化
    if LAST_PEAK_COUNT > 0 {
      let peak_change = (result.peaks.len() as i32 - LAST_PEAK_COUNT as i32).abs();
      if peak_change > 2 {
        writeln!(
          serial,
          "频谱变化: 峰值数 {} -> {} (分析#{})",
          LAST_PEAK_COUNT,
          result.peaks.len(),
          analysis_count
        )
        .ok();
      }
    }
    LAST_PEAK_COUNT = result.peaks.len();

    // 检测功率变化
    if LAST_TOTAL_POWER > 0.0 {
      let power_ratio = result.total_power / LAST_TOTAL_POWER;
      if power_ratio > 2.0 || power_ratio < 0.5 {
        writeln!(
          serial,
          "功率变化: {:.6} -> {:.6} (分析#{})",
          LAST_TOTAL_POWER, result.total_power, analysis_count
        )
        .ok();
      }
    }
    LAST_TOTAL_POWER = result.total_power;
  }
}

/// 分析THD趋势
fn analyze_thd_trend(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  result: &signal_analyzer::HarmonicResult<REALTIME_SIZE>,
  analysis_count: u32,
) {
  static mut THD_HISTORY: [f32; 5] = [0.0; 5];
  static mut THD_INDEX: usize = 0;

  unsafe {
    THD_HISTORY[THD_INDEX] = result.thd;
    THD_INDEX = (THD_INDEX + 1) % THD_HISTORY.len();

    // 计算THD趋势
    if analysis_count > 5 {
      let recent_avg = THD_HISTORY[..3].iter().sum::<f32>() / 3.0;
      let older_avg = THD_HISTORY[3..].iter().sum::<f32>() / 2.0;
      let trend = recent_avg - older_avg;

      if trend > 1.0 {
        writeln!(serial, "THD趋势: 上升 (+{:.1}%)", trend).ok();
      } else if trend < -1.0 {
        writeln!(serial, "THD趋势: 下降 ({:.1}%)", trend).ok();
      }
    }
  }
}

/// 评估整体信号质量
fn assess_overall_signal_quality(
  realtime_result: &signal_analyzer::RealTimeResult<REALTIME_SIZE>,
  spectrum_result: &signal_analyzer::SpectrumResult<REALTIME_SIZE>,
  harmonic_result: &signal_analyzer::HarmonicResult<REALTIME_SIZE>,
) -> f32 {
  let mut quality_score = 0.0f32;
  let mut weight_sum = 0.0f32;

  // 时域质量 (权重: 0.3)
  let time_quality = realtime_result.signal_quality;
  quality_score += time_quality * 0.3;
  weight_sum += 0.3;

  // SNR质量 (权重: 0.3)
  let snr_quality = if spectrum_result.snr > 0.0 {
    (spectrum_result.snr / 60.0).min(1.0)
  } else {
    0.0
  };
  quality_score += snr_quality * 0.3;
  weight_sum += 0.3;

  // THD质量 (权重: 0.2)
  let thd_quality = if harmonic_result.thd < 100.0 {
    1.0 - (harmonic_result.thd / 100.0)
  } else {
    0.0
  };
  quality_score += thd_quality * 0.2;
  weight_sum += 0.2;

  // 频率稳定性质量 (权重: 0.2)
  let stability_quality = 1.0 / (1.0 + realtime_result.frequency_stability * 10.0);
  quality_score += stability_quality * 0.2;
  weight_sum += 0.2;

  if weight_sum > 0.0 {
    quality_score / weight_sum
  } else {
    0.0
  }
}

/// 检测综合异常
fn detect_comprehensive_anomalies(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  realtime_result: &signal_analyzer::RealTimeResult<REALTIME_SIZE>,
  spectrum_result: &signal_analyzer::SpectrumResult<REALTIME_SIZE>,
  harmonic_result: &signal_analyzer::HarmonicResult<REALTIME_SIZE>,
  stats: &mut RealTimeAnalysisStatistics,
) {
  let mut anomalies = heapless::Vec::<&str, 10>::new();

  // 综合质量异常
  let overall_quality =
    assess_overall_signal_quality(realtime_result, spectrum_result, harmonic_result);
  if overall_quality < 0.3 {
    anomalies.push("整体信号质量差").ok();
  }

  // 频域时域不一致
  let freq_diff =
    (realtime_result.dominant_frequency - spectrum_result.fundamental_frequency).abs();
  if freq_diff > 50.0 {
    anomalies.push("频域时域频率不一致").ok();
  }

  // 谐波异常丰富
  if harmonic_result.harmonics.len() > 10 && harmonic_result.thd > 20.0 {
    anomalies.push("谐波异常丰富").ok();
  }

  // 噪声与SNR不匹配
  if realtime_result.noise_level > 0.1 && spectrum_result.snr > 40.0 {
    anomalies.push("噪声与SNR不匹配").ok();
  }

  if !anomalies.is_empty() {
    writeln!(serial, "综合异常检测:").ok();
    for anomaly in anomalies.iter() {
      writeln!(serial, "  ⚠️ {}", anomaly).ok();
    }
    stats.record_anomaly();
  }
}

/// 输出实时状态
fn output_realtime_status(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  sample_count: u32,
  analysis_count: u32,
  current_mode: usize,
  stats: &RealTimeAnalysisStatistics,
  performance: &RealTimePerformanceMonitor,
) {
  let current_mode_name = match current_mode {
    0 => "基础实时分析",
    1 => "频谱实时分析",
    2 => "谐波实时分析",
    3 => "综合实时分析",
    _ => "未知模式",
  };

  writeln!(serial, "\n=== 实时分析系统状态 ===").ok();
  writeln!(serial, "采样数: {}", sample_count).ok();
  writeln!(serial, "分析数: {}", analysis_count).ok();
  writeln!(serial, "缓冲区填充: {}", stats.buffer_fills).ok();
  writeln!(serial, "当前模式: {}", current_mode_name).ok();
  writeln!(serial, "模式切换次数: {}", stats.mode_switch_count).ok();

  // 性能统计
  writeln!(serial, "性能统计:").ok();
  writeln!(
    serial,
    "  平均采样时间: {:.1} μs",
    performance.avg_sampling_time
  )
  .ok();
  writeln!(
    serial,
    "  平均分析时间: {:.1} μs",
    performance.avg_analysis_time
  )
  .ok();
  writeln!(
    serial,
    "  CPU利用率: {:.1}%",
    performance.get_cpu_utilization()
  )
  .ok();
  writeln!(serial, "  采样超时: {}", performance.sampling_overruns).ok();
  writeln!(serial, "  分析超时: {}", performance.analysis_overruns).ok();

  // 信号特征
  writeln!(serial, "信号特征:").ok();
  writeln!(
    serial,
    "  平均幅度: {:.4} V",
    stats.signal_characteristics.avg_amplitude
  )
  .ok();
  writeln!(
    serial,
    "  平均频率: {:.1} Hz",
    stats.signal_characteristics.avg_frequency
  )
  .ok();
  writeln!(
    serial,
    "  信号稳定性: {:.3}",
    stats.signal_characteristics.signal_stability
  )
  .ok();
  writeln!(
    serial,
    "  噪声水平: {:.4} V",
    stats.signal_characteristics.noise_level
  )
  .ok();

  // 异常统计
  writeln!(serial, "异常统计:").ok();
  writeln!(serial, "  异常次数: {}", stats.anomaly_count).ok();
  writeln!(serial, "  峰值检测: {}", stats.peak_detection_count).ok();

  writeln!(serial, "").ok();
}

/// 获取时间戳 (微秒)
fn get_timestamp() -> u32 {
  // 简化的时间戳实现
  // 在实际应用中，这里应该使用系统定时器
  static mut TIMESTAMP: u32 = 0;
  unsafe {
    TIMESTAMP += 1;
    TIMESTAMP
  }
}

/// 模拟实时信号
fn simulate_realtime_signal(sample_count: u32) -> u16 {
  let t = sample_count as f32 / SAMPLE_RATE;
  let dc_offset = 2048.0;

  // 动态变化的信号
  let phase_shift = (sample_count / 1000) as f32 * 0.1;
  let amplitude_modulation = 1.0 + 0.3 * (2.0 * core::f32::consts::PI * 2.0 * t).sin();

  // 主信号 (频率缓慢变化)
  let main_freq = 1000.0 + 200.0 * (2.0 * core::f32::consts::PI * 0.5 * t).sin();
  let main_signal = 600.0
    * amplitude_modulation
    * (2.0 * core::f32::consts::PI * main_freq * t + phase_shift).sin();

  // 谐波分量
  let harmonic2 = 120.0 * (2.0 * core::f32::consts::PI * main_freq * 2.0 * t).sin();
  let harmonic3 = 80.0 * (2.0 * core::f32::consts::PI * main_freq * 3.0 * t).sin();

  // 噪声分量
  let white_noise = ((sample_count * 17 + 29) % 61) as f32 - 30.0;
  let scaled_noise = white_noise * 15.0;

  // 间歇性干扰
  let interference = if (sample_count % 2000) < 100 {
    200.0 * (2.0 * core::f32::consts::PI * 50.0 * t).sin()
  } else {
    0.0
  };

  let result = dc_offset + main_signal + harmonic2 + harmonic3 + scaled_noise + interference;
  result.max(0.0).min(4095.0) as u16
}

/// 定时器中断处理
#[interrupt]
fn TIM2() {
  // 在实际应用中，这里会处理ADC采样中断
}
