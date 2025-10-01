#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};
use cortex_m::peripheral::{DWT, NVIC};
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use panic_halt as _;
use precision_timer::{
  delay_ns_precise, delay_us_precise, update_global_time, PrecisionTimer, TimerConfig, TimerMode,
  TimerPrecision, Timestamp,
};
use stm32f4xx_hal::{
  gpio::{
    gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7},
    Output, PushPull,
  },
  gpio::{gpioc::PC13, Input, PullUp},
  interrupt,
  prelude::*,
  stm32,
  timer::{Event, Timer},
};

// LED类型定义
type StatusLed = PB0<Output<PushPull>>;
type TimerLed = PB1<Output<PushPull>>;
type MicrosecondLed = PB2<Output<PushPull>>;
type NanosecondLed = PB3<Output<PushPull>>;
type PrecisionLed = PB4<Output<PushPull>>;
type DelayLed = PB5<Output<PushPull>>;
type AccuracyLed = PB6<Output<PushPull>>;
type ErrorLed = PB7<Output<PushPull>>;

type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut TIMER_QUEUE: Queue<TimerEvent, 128> = Queue::new();
static mut TIMER_PRODUCER: Option<Producer<TimerEvent, 128>> = None;
static mut TIMER_CONSUMER: Option<Consumer<TimerEvent, 128>> = None;

static SYSTEM_TIME_US: AtomicU32 = AtomicU32::new(0);
static TIMER_MODE: AtomicU8 = AtomicU8::new(0);
static PRECISION_LEVEL: AtomicU16 = AtomicU16::new(1000); // 纳秒
static DELAY_ACCURACY: AtomicU32 = AtomicU32::new(0);
static TIMER_ENABLED: AtomicBool = AtomicBool::new(true);

// 精度测试计数器
static PRECISION_COUNTERS: [AtomicU32; 4] = [
  AtomicU32::new(0), // 1μs精度
  AtomicU32::new(0), // 100ns精度
  AtomicU32::new(0), // 10ns精度
  AtomicU32::new(0), // 1ns精度
];

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = stm32::Peripherals::take().unwrap();
  let mut cp = cortex_m::Peripherals::take().unwrap();

  // 启用DWT计数器用于高精度时间测量
  cp.DCB.enable_trace();
  cp.DWT.enable_cycle_counter();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(168.mhz()).freeze(); // 使用最高频率获得最佳精度

  // 配置GPIO
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  let mut status_led = gpiob.pb0.into_push_pull_output();
  let mut timer_led = gpiob.pb1.into_push_pull_output();
  let mut microsecond_led = gpiob.pb2.into_push_pull_output();
  let mut nanosecond_led = gpiob.pb3.into_push_pull_output();
  let mut precision_led = gpiob.pb4.into_push_pull_output();
  let mut delay_led = gpiob.pb5.into_push_pull_output();
  let mut accuracy_led = gpiob.pb6.into_push_pull_output();
  let mut error_led = gpiob.pb7.into_push_pull_output();

  let mode_button = gpioc.pc13.into_pull_up_input();

  // 初始化定时器事件队列
  let (producer, consumer) = unsafe { TIMER_QUEUE.split() };
  unsafe {
    TIMER_PRODUCER = Some(producer);
    TIMER_CONSUMER = Some(consumer);
  }

  // 配置高精度定时器 - 使用TIM2 (32位定时器)
  let mut timer2 = Timer::tim2(dp.TIM2, &clocks);
  let mut timer3 = Timer::tim3(dp.TIM3, &clocks);
  let mut timer4 = Timer::tim4(dp.TIM4, &clocks);
  let mut timer5 = Timer::tim5(dp.TIM5, &clocks);

  // 设置不同精度的定时器
  timer2.start(1000000.hz()); // 1MHz - 1μs精度
  timer3.start(10000000.hz()); // 10MHz - 100ns精度
  timer4.start(100000000.hz()); // 100MHz - 10ns精度
  timer5.start(1000.hz()); // 1kHz - 用于系统时间更新

  // 启用定时器中断
  timer2.listen(Event::TimeOut);
  timer3.listen(Event::TimeOut);
  timer4.listen(Event::TimeOut);
  timer5.listen(Event::TimeOut);

  // 配置NVIC
  unsafe {
    NVIC::unmask(stm32::Interrupt::TIM2);
    NVIC::unmask(stm32::Interrupt::TIM3);
    NVIC::unmask(stm32::Interrupt::TIM4);
    NVIC::unmask(stm32::Interrupt::TIM5);
  }

  // 创建高精度定时器实例
  let timer_config = TimerConfig {
    precision: TimerPrecision::Nanosecond,
    mode: TimerMode::Periodic,
    prescaler: 0,     // 最小预分频获得最高精度
    auto_reload: 168, // 对于168MHz时钟，得到1MHz定时器频率
    interrupt_enabled: true,
    dma_enabled: false,
  };

  let precision_timer = PrecisionTimer::new(timer_config, 168_000_000);
  precision_timer.init().ok();
  precision_timer.start().ok();

  // 配置延时定时器
  let mut delay_timer = Timer::tim6(dp.TIM6, &clocks).delay_ms();

  // 系统启动指示
  status_led.set_high();
  delay_timer.delay_ms(1000u32);

  // 微秒定时器管理器
  let mut microsecond_manager = MicrosecondTimerManager::new(precision_timer);
  let mut button_debouncer = ButtonDebouncer::new();
  let mut demo_counter = 0u32;

  loop {
    // 更新系统时间和全局时间
    demo_counter += 1;
    let current_time_us = demo_counter * 100; // 100μs per loop
    SYSTEM_TIME_US.store(current_time_us, Ordering::Relaxed);
    update_global_time(current_time_us as u64 * 1000); // 转换为纳秒

    // 按钮防抖处理
    let button_pressed = button_debouncer.update(mode_button.is_low());

    if button_pressed {
      // 切换定时器模式
      let current_mode = TIMER_MODE.load(Ordering::Relaxed);
      let new_mode = (current_mode + 1) % 4;
      TIMER_MODE.store(new_mode, Ordering::Relaxed);

      // 应用新的定时器配置
      microsecond_manager.apply_timer_mode(new_mode);

      // 重置统计
      microsecond_manager.reset_statistics();

      // 模式切换指示
      for _ in 0..3 {
        timer_led.set_low();
        delay_timer.delay_ms(100u32);
        timer_led.set_high();
        delay_timer.delay_ms(100u32);
      }
    }

    // 处理定时器事件
    if let Some(consumer) = unsafe { TIMER_CONSUMER.as_mut() } {
      while let Some(event) = consumer.dequeue() {
        microsecond_manager.handle_timer_event(event);
      }
    }

    // 执行精度测试
    microsecond_manager.run_precision_tests();

    // 显示定时器状态
    display_timer_status(
      &microsecond_manager,
      &mut microsecond_led,
      &mut nanosecond_led,
      &mut precision_led,
      &mut delay_led,
      &mut accuracy_led,
      &mut error_led,
      demo_counter,
    );

    // 状态LED心跳
    if demo_counter % 100 == 0 {
      status_led.toggle();
    }

    // 定时器活动指示
    timer_led
      .set_state(TIMER_ENABLED.load(Ordering::Relaxed).into())
      .ok();

    delay_timer.delay_ms(100u32);
  }
}

/// 显示定时器状态
fn display_timer_status(
  manager: &MicrosecondTimerManager,
  microsecond_led: &mut impl embedded_hal::digital::v2::OutputPin,
  nanosecond_led: &mut impl embedded_hal::digital::v2::OutputPin,
  precision_led: &mut impl embedded_hal::digital::v2::OutputPin,
  delay_led: &mut impl embedded_hal::digital::v2::OutputPin,
  accuracy_led: &mut impl embedded_hal::digital::v2::OutputPin,
  error_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  let stats = manager.get_timer_statistics();

  // 根据精度级别显示LED
  let precision_ns = PRECISION_LEVEL.load(Ordering::Relaxed);

  microsecond_led
    .set_state((precision_ns >= 1000 && (counter / 10) % 2 == 0).into())
    .ok();
  nanosecond_led
    .set_state((precision_ns < 1000 && (counter / 8) % 2 == 0).into())
    .ok();

  // 精度指示
  if precision_ns <= 10 {
    precision_led
      .set_state(((counter / 5) % 2 == 0).into())
      .ok();
  } else if precision_ns <= 100 {
    precision_led
      .set_state(((counter / 8) % 2 == 0).into())
      .ok();
  } else {
    precision_led
      .set_state(((counter / 12) % 2 == 0).into())
      .ok();
  }

  // 延时测试指示
  let delay_accuracy = DELAY_ACCURACY.load(Ordering::Relaxed);
  if delay_accuracy > 0 {
    delay_led.set_state(((counter / 6) % 2 == 0).into()).ok();
  } else {
    delay_led.set_low().ok();
  }

  // 精度指示
  if stats.avg_precision_error < 50 {
    accuracy_led.set_high().ok();
  } else if stats.avg_precision_error < 200 {
    accuracy_led
      .set_state(((counter / 15) % 2 == 0).into())
      .ok();
  } else {
    accuracy_led.set_low().ok();
  }

  // 错误指示
  if stats.error_count > 0 {
    error_led.set_state(((counter / 7) % 2 == 0).into()).ok();
  } else {
    error_led.set_low().ok();
  }
}

/// 微秒定时器管理器
struct MicrosecondTimerManager {
  precision_timer: PrecisionTimer,
  current_mode: u8,
  timer_stats: TimerStatistics,
  test_results: [PrecisionTestResult; 4],
}

impl MicrosecondTimerManager {
  fn new(precision_timer: PrecisionTimer) -> Self {
    Self {
      precision_timer,
      current_mode: 0,
      timer_stats: TimerStatistics::default(),
      test_results: [PrecisionTestResult::default(); 4],
    }
  }

  fn apply_timer_mode(&mut self, mode: u8) {
    self.current_mode = mode;

    match mode {
      0 => {
        // 微秒精度模式
        PRECISION_LEVEL.store(1000, Ordering::Relaxed);
        TIMER_ENABLED.store(true, Ordering::Relaxed);
      }
      1 => {
        // 100纳秒精度模式
        PRECISION_LEVEL.store(100, Ordering::Relaxed);
        TIMER_ENABLED.store(true, Ordering::Relaxed);
      }
      2 => {
        // 10纳秒精度模式
        PRECISION_LEVEL.store(10, Ordering::Relaxed);
        TIMER_ENABLED.store(true, Ordering::Relaxed);
      }
      3 => {
        // 自适应精度模式
        let cpu_load = self.estimate_cpu_load();
        if cpu_load > 80 {
          PRECISION_LEVEL.store(1000, Ordering::Relaxed);
        } else if cpu_load > 50 {
          PRECISION_LEVEL.store(100, Ordering::Relaxed);
        } else {
          PRECISION_LEVEL.store(10, Ordering::Relaxed);
        }
        TIMER_ENABLED.store(true, Ordering::Relaxed);
      }
      _ => {}
    }
  }

  fn handle_timer_event(&mut self, event: TimerEvent) {
    self.timer_stats.total_events += 1;

    // 更新精度统计
    let precision_error = event.actual_time_ns.abs_diff(event.expected_time_ns);
    self.timer_stats.total_precision_error += precision_error as u64;
    self.timer_stats.avg_precision_error =
      (self.timer_stats.total_precision_error / self.timer_stats.total_events as u64) as u32;

    if precision_error > self.timer_stats.max_precision_error {
      self.timer_stats.max_precision_error = precision_error;
    }

    // 更新精度计数器
    let precision_index = match event.precision_level {
      p if p >= 1000 => 0,
      p if p >= 100 => 1,
      p if p >= 10 => 2,
      _ => 3,
    };

    PRECISION_COUNTERS[precision_index].fetch_add(1, Ordering::Relaxed);
  }

  fn run_precision_tests(&mut self) {
    let current_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

    // 每秒运行一次精度测试
    if current_time % 10000 == 0 {
      // 每1秒
      self.test_microsecond_precision();
      self.test_nanosecond_precision();
      self.test_delay_accuracy();
      self.test_timing_consistency();
    }
  }

  fn test_microsecond_precision(&mut self) {
    let start_cycles = DWT::get_cycle_count();

    // 测试1微秒延时
    delay_us_precise(1);

    let end_cycles = DWT::get_cycle_count();
    let actual_cycles = end_cycles.wrapping_sub(start_cycles);
    let actual_ns = (actual_cycles as u64 * 1_000_000_000) / 168_000_000; // 168MHz时钟

    let expected_ns = 1000; // 1微秒
    let error_ns = actual_ns.abs_diff(expected_ns);

    self.test_results[0].add_sample(error_ns as u32);
  }

  fn test_nanosecond_precision(&mut self) {
    let start_cycles = DWT::get_cycle_count();

    // 测试100纳秒延时
    delay_ns_precise(100);

    let end_cycles = DWT::get_cycle_count();
    let actual_cycles = end_cycles.wrapping_sub(start_cycles);
    let actual_ns = (actual_cycles as u64 * 1_000_000_000) / 168_000_000;

    let expected_ns = 100;
    let error_ns = actual_ns.abs_diff(expected_ns);

    self.test_results[1].add_sample(error_ns as u32);
  }

  fn test_delay_accuracy(&mut self) {
    let delays_us = [1, 5, 10, 50, 100];
    let mut total_error = 0u32;

    for &delay_us in &delays_us {
      let start_cycles = DWT::get_cycle_count();
      delay_us_precise(delay_us);
      let end_cycles = DWT::get_cycle_count();

      let actual_cycles = end_cycles.wrapping_sub(start_cycles);
      let actual_us = (actual_cycles as u64 * 1_000_000) / 168_000_000;
      let error_us = actual_us.abs_diff(delay_us as u64);

      total_error += error_us as u32;
    }

    let avg_error = total_error / delays_us.len() as u32;
    DELAY_ACCURACY.store(avg_error, Ordering::Relaxed);

    self.test_results[2].add_sample(avg_error);
  }

  fn test_timing_consistency(&mut self) {
    let mut measurements = [0u32; 10];

    // 进行10次相同的测量
    for i in 0..10 {
      let start_cycles = DWT::get_cycle_count();
      delay_us_precise(10); // 10微秒延时
      let end_cycles = DWT::get_cycle_count();

      let actual_cycles = end_cycles.wrapping_sub(start_cycles);
      measurements[i] = (actual_cycles * 1000) / 168; // 转换为纳秒
    }

    // 计算标准差
    let avg: u32 = measurements.iter().sum::<u32>() / measurements.len() as u32;
    let variance: u32 = measurements
      .iter()
      .map(|&x| {
        let diff = if x > avg { x - avg } else { avg - x };
        diff * diff
      })
      .sum::<u32>()
      / measurements.len() as u32;

    let std_dev = (variance as f32).sqrt() as u32;

    self.test_results[3].add_sample(std_dev);
  }

  fn get_timer_statistics(&self) -> &TimerStatistics {
    &self.timer_stats
  }

  fn reset_statistics(&mut self) {
    self.timer_stats = TimerStatistics::default();

    // 重置计数器
    for counter in &PRECISION_COUNTERS {
      counter.store(0, Ordering::Relaxed);
    }

    DELAY_ACCURACY.store(0, Ordering::Relaxed);

    // 重置测试结果
    for result in &mut self.test_results {
      *result = PrecisionTestResult::default();
    }
  }

  fn estimate_cpu_load(&self) -> u32 {
    // 简化的CPU负载估算
    let total_events = self.timer_stats.total_events;
    let elapsed_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

    if elapsed_time > 0 {
      ((total_events as u64 * 1000) / elapsed_time as u64).min(100) as u32
    } else {
      0
    }
  }
}

/// 定时器统计信息
#[derive(Default)]
struct TimerStatistics {
  total_events: u32,
  total_precision_error: u64,
  avg_precision_error: u32,
  max_precision_error: u32,
  error_count: u32,
}

/// 精度测试结果
#[derive(Clone, Copy)]
struct PrecisionTestResult {
  sample_count: u32,
  min_error: u32,
  max_error: u32,
  avg_error: f32,
  total_error: u64,
}

impl Default for PrecisionTestResult {
  fn default() -> Self {
    Self {
      sample_count: 0,
      min_error: u32::MAX,
      max_error: 0,
      avg_error: 0.0,
      total_error: 0,
    }
  }
}

impl PrecisionTestResult {
  fn add_sample(&mut self, error: u32) {
    self.sample_count += 1;
    self.total_error += error as u64;

    if error < self.min_error {
      self.min_error = error;
    }

    if error > self.max_error {
      self.max_error = error;
    }

    self.avg_error = self.total_error as f32 / self.sample_count as f32;
  }
}

/// 定时器事件
#[derive(Clone, Copy)]
struct TimerEvent {
  timer_id: u8,
  timestamp: u32,
  expected_time_ns: u32,
  actual_time_ns: u32,
  precision_level: u16,
}

/// 按钮防抖器
struct ButtonDebouncer {
  last_state: bool,
  stable_state: bool,
  counter: u8,
}

impl ButtonDebouncer {
  fn new() -> Self {
    Self {
      last_state: false,
      stable_state: false,
      counter: 0,
    }
  }

  fn update(&mut self, current_state: bool) -> bool {
    if current_state != self.last_state {
      self.counter = 0;
      self.last_state = current_state;
    } else {
      if self.counter < 5 {
        self.counter += 1;
      }
    }

    if self.counter >= 5 && current_state != self.stable_state {
      self.stable_state = current_state;
      return current_state;
    }

    false
  }
}

// 中断处理函数
#[interrupt]
fn TIM2() {
  // 1μs精度定时器中断
  let event = TimerEvent {
    timer_id: 2,
    timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
    expected_time_ns: 1000,
    actual_time_ns: 1000, // 实际应该从硬件读取
    precision_level: 1000,
  };

  if let Some(producer) = unsafe { TIMER_PRODUCER.as_mut() } {
    producer.enqueue(event).ok();
  }

  // 清除中断标志
  unsafe {
    let tim2 = &(*stm32::TIM2::ptr());
    tim2.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM3() {
  // 100ns精度定时器中断
  let event = TimerEvent {
    timer_id: 3,
    timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
    expected_time_ns: 100,
    actual_time_ns: 100,
    precision_level: 100,
  };

  if let Some(producer) = unsafe { TIMER_PRODUCER.as_mut() } {
    producer.enqueue(event).ok();
  }

  // 清除中断标志
  unsafe {
    let tim3 = &(*stm32::TIM3::ptr());
    tim3.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM4() {
  // 10ns精度定时器中断
  let event = TimerEvent {
    timer_id: 4,
    timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
    expected_time_ns: 10,
    actual_time_ns: 10,
    precision_level: 10,
  };

  if let Some(producer) = unsafe { TIMER_PRODUCER.as_mut() } {
    producer.enqueue(event).ok();
  }

  // 清除中断标志
  unsafe {
    let tim4 = &(*stm32::TIM4::ptr());
    tim4.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM5() {
  // 系统时间更新定时器
  let current_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
  update_global_time(current_time as u64 * 1000);

  // 清除中断标志
  unsafe {
    let tim5 = &(*stm32::TIM5::ptr());
    tim5.sr.modify(|_, w| w.uif().clear_bit());
  }
}
