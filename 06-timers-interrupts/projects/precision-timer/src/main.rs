#![no_std]
#![no_main]

use cortex_m::{asm, peripheral::DWT};
use cortex_m_rt::entry;
use fugit::{Duration, Instant, Rate};
use heapless::{FnvIndexMap, Vec};
use libm::{fabs, sqrt};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{Output, Pin, PushPull},
  pac::{self, TIM2, TIM3, TIM5},
  prelude::*,
  rcc::Clocks,
  timer::{Counter, Event, Timer},
};

/// 高精度定时器抽象
pub struct PrecisionTimer<TIM> {
  timer: Counter<TIM, 1_000_000>, // 1MHz时基
  calibration_factor: f32,
  statistics: TimingStatistics,
}

/// 时间测量统计
#[derive(Debug, Clone)]
pub struct TimingStatistics {
  samples: Vec<u32, 1000>,
  min_time: u32,
  max_time: u32,
  total_time: u64,
  count: u32,
}

/// 定时器校准器
pub struct TimerCalibrator {
  reference_timer: Counter<TIM5, 1_000_000>,
  test_timer: Counter<TIM2, 1_000_000>,
  dwt_cycles: u32,
}

/// 微秒级延时实现
pub struct MicrosecondDelay<TIM> {
  timer: PrecisionTimer<TIM>,
  overhead_compensation: u32,
}

/// 时间戳生成器
pub struct TimestampGenerator {
  timer: Counter<TIM3, 1_000_000>,
  epoch_offset: u64,
  rollover_count: u32,
}

impl<TIM> PrecisionTimer<TIM>
where
  TIM: pac::timer::Instance,
{
  /// 创建新的精密定时器
  pub fn new(timer: Timer<TIM>, clocks: &Clocks) -> Self {
    let mut counter = timer.counter_us(clocks);
    counter
      .start(Duration::<u32, 1, 1_000_000>::from_ticks(u32::MAX))
      .unwrap();

    Self {
      timer: counter,
      calibration_factor: 1.0,
      statistics: TimingStatistics::new(),
    }
  }

  /// 微秒级延时
  pub fn delay_us(&mut self, us: u32) {
    let adjusted_us = (us as f32 * self.calibration_factor) as u32;
    let start = self.timer.now();

    while self.timer.now().duration_since(start).ticks() < adjusted_us {
      asm::nop();
    }

    // 记录实际延时时间用于统计
    let actual_time = self.timer.now().duration_since(start).ticks();
    self.statistics.add_sample(actual_time);
  }

  /// 纳秒级延时（基于DWT周期计数器）
  pub fn delay_ns(&self, ns: u32) {
    let cycles = (ns as u64 * 168_000_000) / 1_000_000_000; // 168MHz系统时钟
    let start = DWT::cycle_count();

    while DWT::cycle_count().wrapping_sub(start) < cycles as u32 {
      asm::nop();
    }
  }

  /// 测量代码执行时间
  pub fn measure_execution<F, R>(&mut self, f: F) -> (R, Duration<u32, 1, 1_000_000>)
  where
    F: FnOnce() -> R,
  {
    let start = self.timer.now();
    let result = f();
    let end = self.timer.now();

    let duration = end.duration_since(start);
    self.statistics.add_sample(duration.ticks());

    (result, duration)
  }

  /// 设置校准因子
  pub fn set_calibration_factor(&mut self, factor: f32) {
    self.calibration_factor = factor;
  }

  /// 获取统计信息
  pub fn get_statistics(&self) -> &TimingStatistics {
    &self.statistics
  }

  /// 重置统计信息
  pub fn reset_statistics(&mut self) {
    self.statistics.reset();
  }
}

impl TimingStatistics {
  pub fn new() -> Self {
    Self {
      samples: Vec::new(),
      min_time: u32::MAX,
      max_time: 0,
      total_time: 0,
      count: 0,
    }
  }

  pub fn add_sample(&mut self, time: u32) {
    if self.samples.push(time).is_err() {
      // 如果样本缓冲区满了，移除最旧的样本
      self.samples.remove(0);
      self.samples.push(time).ok();
    }

    self.min_time = self.min_time.min(time);
    self.max_time = self.max_time.max(time);
    self.total_time += time as u64;
    self.count += 1;
  }

  pub fn average(&self) -> f32 {
    if self.count == 0 {
      0.0
    } else {
      self.total_time as f32 / self.count as f32
    }
  }

  pub fn standard_deviation(&self) -> f32 {
    if self.samples.len() < 2 {
      return 0.0;
    }

    let avg = self.average();
    let variance: f32 = self
      .samples
      .iter()
      .map(|&x| {
        let diff = x as f32 - avg;
        diff * diff
      })
      .sum::<f32>()
      / (self.samples.len() - 1) as f32;

    sqrt(variance)
  }

  pub fn jitter(&self) -> u32 {
    self.max_time - self.min_time
  }

  pub fn reset(&mut self) {
    self.samples.clear();
    self.min_time = u32::MAX;
    self.max_time = 0;
    self.total_time = 0;
    self.count = 0;
  }
}

impl TimerCalibrator {
  pub fn new(reference_timer: Timer<TIM5>, test_timer: Timer<TIM2>, clocks: &Clocks) -> Self {
    let mut ref_counter = reference_timer.counter_us(clocks);
    let mut test_counter = test_timer.counter_us(clocks);

    ref_counter
      .start(Duration::<u32, 1, 1_000_000>::from_ticks(u32::MAX))
      .unwrap();
    test_counter
      .start(Duration::<u32, 1, 1_000_000>::from_ticks(u32::MAX))
      .unwrap();

    Self {
      reference_timer: ref_counter,
      test_timer: test_counter,
      dwt_cycles: 0,
    }
  }

  /// 校准定时器精度
  pub fn calibrate(&mut self, target_delay_us: u32) -> f32 {
    const CALIBRATION_SAMPLES: usize = 100;
    let mut calibration_factors = Vec::<f32, CALIBRATION_SAMPLES>::new();

    for _ in 0..CALIBRATION_SAMPLES {
      // 使用参考定时器测量实际延时
      let ref_start = self.reference_timer.now();

      // 执行待校准的延时
      let test_start = self.test_timer.now();
      while self.test_timer.now().duration_since(test_start).ticks() < target_delay_us {
        asm::nop();
      }

      let ref_end = self.reference_timer.now();
      let actual_delay = ref_end.duration_since(ref_start).ticks();

      // 计算校准因子
      let factor = target_delay_us as f32 / actual_delay as f32;
      calibration_factors.push(factor).ok();
    }

    // 计算平均校准因子
    let sum: f32 = calibration_factors.iter().sum();
    sum / calibration_factors.len() as f32
  }

  /// 使用DWT进行高精度校准
  pub fn calibrate_with_dwt(&mut self, target_cycles: u32) -> f32 {
    let dwt_start = DWT::cycle_count();
    let timer_start = self.test_timer.now();

    // 等待指定的DWT周期数
    while DWT::cycle_count().wrapping_sub(dwt_start) < target_cycles {
      asm::nop();
    }

    let timer_end = self.test_timer.now();
    let timer_duration = timer_end.duration_since(timer_start).ticks();

    // 计算期望的定时器计数值
    let expected_timer_ticks = (target_cycles as u64 * 1_000_000) / 168_000_000; // 168MHz -> 1MHz

    expected_timer_ticks as f32 / timer_duration as f32
  }
}

impl<TIM> MicrosecondDelay<TIM>
where
  TIM: pac::timer::Instance,
{
  pub fn new(timer: Timer<TIM>, clocks: &Clocks) -> Self {
    let precision_timer = PrecisionTimer::new(timer, clocks);

    Self {
      timer: precision_timer,
      overhead_compensation: 0,
    }
  }

  /// 校准延时开销
  pub fn calibrate_overhead(&mut self) {
    const SAMPLES: usize = 1000;
    let mut overhead_samples = Vec::<u32, SAMPLES>::new();

    for _ in 0..SAMPLES {
      let start = self.timer.timer.now();
      // 空操作，测量函数调用开销
      asm::nop();
      let end = self.timer.timer.now();

      let overhead = end.duration_since(start).ticks();
      overhead_samples.push(overhead).ok();
    }

    // 使用中位数作为开销补偿值
    overhead_samples.sort();
    self.overhead_compensation = overhead_samples[SAMPLES / 2];
  }

  /// 补偿开销的精确延时
  pub fn delay_us_compensated(&mut self, us: u32) {
    if us > self.overhead_compensation {
      self.timer.delay_us(us - self.overhead_compensation);
    } else {
      // 对于极短延时，使用DWT
      self.timer.delay_ns(us * 1000);
    }
  }
}

impl TimestampGenerator {
  pub fn new(timer: Timer<TIM3>, clocks: &Clocks) -> Self {
    let mut counter = timer.counter_us(clocks);
    counter
      .start(Duration::<u32, 1, 1_000_000>::from_ticks(u32::MAX))
      .unwrap();

    Self {
      timer: counter,
      epoch_offset: 0,
      rollover_count: 0,
    }
  }

  /// 获取64位时间戳（微秒）
  pub fn timestamp_us(&mut self) -> u64 {
    let current_ticks = self.timer.now().ticks();

    // 检测定时器溢出
    if current_ticks < (u32::MAX / 2) && self.rollover_count > 0 {
      self.rollover_count += 1;
    }

    self.epoch_offset + (self.rollover_count as u64 * u32::MAX as u64) + current_ticks as u64
  }

  /// 设置时间戳起始点
  pub fn set_epoch(&mut self, epoch_us: u64) {
    self.epoch_offset = epoch_us;
    self.rollover_count = 0;
  }
}

/// 性能基准测试套件
pub struct PerformanceBenchmark {
  timer: PrecisionTimer<TIM2>,
  results: FnvIndexMap<&'static str, TimingStatistics, 16>,
}

impl PerformanceBenchmark {
  pub fn new(timer: Timer<TIM2>, clocks: &Clocks) -> Self {
    Self {
      timer: PrecisionTimer::new(timer, clocks),
      results: FnvIndexMap::new(),
    }
  }

  /// 运行基准测试
  pub fn benchmark<F>(&mut self, name: &'static str, iterations: u32, f: F)
  where
    F: Fn(),
  {
    let mut stats = TimingStatistics::new();

    for _ in 0..iterations {
      let (_, duration) = self.timer.measure_execution(|| f());
      stats.add_sample(duration.ticks());
    }

    self.results.insert(name, stats).ok();
  }

  /// 获取基准测试结果
  pub fn get_results(&self) -> &FnvIndexMap<&'static str, TimingStatistics, 16> {
    &self.results
  }

  /// 打印基准测试报告
  pub fn print_report(&self) {
    for (name, stats) in &self.results {
      let avg = stats.average();
      let std_dev = stats.standard_deviation();
      let jitter = stats.jitter();

      rtt_target::rprintln!(
        "Benchmark: {} | Avg: {:.2}μs | StdDev: {:.2}μs | Jitter: {}μs | Samples: {}",
        name,
        avg,
        std_dev,
        jitter,
        stats.count
      );
    }
  }
}

#[entry]
fn main() -> ! {
  // 初始化RTT日志
  rtt_target::rtt_init_print!();
  rtt_target::rprintln!("精密定时器系统启动");

  // 初始化外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟到168MHz
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(8.MHz())
    .sysclk(168.MHz())
    .pclk1(42.MHz())
    .pclk2(84.MHz())
    .freeze();

  // 启用DWT周期计数器
  let mut dwt = cp.DWT;
  let dcb = cp.DCB;
  dwt.enable_cycle_counter(&dcb);

  // 创建定时器实例
  let tim2 = Timer::new(dp.TIM2, &clocks);
  let tim3 = Timer::new(dp.TIM3, &clocks);
  let tim5 = Timer::new(dp.TIM5, &clocks);

  // 创建精密定时器
  let mut precision_timer = PrecisionTimer::new(tim2, &clocks);

  // 创建校准器
  let mut calibrator = TimerCalibrator::new(tim5, Timer::new(dp.TIM4, &clocks), &clocks);

  // 执行校准
  rtt_target::rprintln!("开始定时器校准...");
  let calibration_factor = calibrator.calibrate(1000); // 校准1ms延时
  precision_timer.set_calibration_factor(calibration_factor);
  rtt_target::rprintln!("校准完成，校准因子: {:.6}", calibration_factor);

  // 创建时间戳生成器
  let mut timestamp_gen = TimestampGenerator::new(tim3, &clocks);
  timestamp_gen.set_epoch(0);

  // 创建性能基准测试
  let mut benchmark = PerformanceBenchmark::new(Timer::new(dp.TIM6, &clocks), &clocks);

  // 运行基准测试
  rtt_target::rprintln!("开始性能基准测试...");

  benchmark.benchmark("1μs延时", 1000, || {
    precision_timer.delay_us(1);
  });

  benchmark.benchmark("10μs延时", 1000, || {
    precision_timer.delay_us(10);
  });

  benchmark.benchmark("100μs延时", 1000, || {
    precision_timer.delay_us(100);
  });

  benchmark.benchmark("1ms延时", 100, || {
    precision_timer.delay_us(1000);
  });

  // 测试纳秒级延时
  benchmark.benchmark("100ns延时", 10000, || {
    precision_timer.delay_ns(100);
  });

  // 打印基准测试结果
  benchmark.print_report();

  // 主循环演示
  let mut counter = 0u32;
  loop {
    // 每秒执行一次
    precision_timer.delay_us(1_000_000);

    let timestamp = timestamp_gen.timestamp_us();
    let stats = precision_timer.get_statistics();

    rtt_target::rprintln!(
      "计数: {} | 时间戳: {}μs | 平均延时: {:.2}μs | 抖动: {}μs",
      counter,
      timestamp,
      stats.average(),
      stats.jitter()
    );

    counter += 1;

    // 每10秒重置统计
    if counter % 10 == 0 {
      precision_timer.reset_statistics();
    }
  }
}
