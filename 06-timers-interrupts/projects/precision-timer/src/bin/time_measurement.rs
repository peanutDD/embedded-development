#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};
use cortex_m::peripheral::{DWT, NVIC};
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use panic_halt as _;
use precision_timer::{
  get_global_time, update_global_time, MeasurementResult, MeasurementType, TimeMeasurement,
  Timestamp,
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
type MeasurementLed = PB1<Output<PushPull>>;
type DurationLed = PB2<Output<PushPull>>;
type IntervalLed = PB3<Output<PushPull>>;
type FrequencyLed = PB4<Output<PushPull>>;
type AccuracyLed = PB5<Output<PushPull>>;
type OverflowLed = PB6<Output<PushPull>>;
type ErrorLed = PB7<Output<PushPull>>;

type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut MEASUREMENT_QUEUE: Queue<MeasurementEvent, 128> = Queue::new();
static mut MEASUREMENT_PRODUCER: Option<Producer<MeasurementEvent, 128>> = None;
static mut MEASUREMENT_CONSUMER: Option<Consumer<MeasurementEvent, 128>> = None;

static SYSTEM_TIME_US: AtomicU32 = AtomicU32::new(0);
static MEASUREMENT_MODE: AtomicU8 = AtomicU8::new(0);
static MEASUREMENT_COUNT: AtomicU32 = AtomicU32::new(0);
static TOTAL_MEASUREMENTS: AtomicU32 = AtomicU32::new(0);
static MEASUREMENT_ACTIVE: AtomicBool = AtomicBool::new(false);

// 测量统计
static DURATION_STATS: [AtomicU32; 4] = [
  AtomicU32::new(0), // 最小持续时间
  AtomicU32::new(0), // 最大持续时间
  AtomicU32::new(0), // 平均持续时间
  AtomicU32::new(0), // 总持续时间
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
  let clocks = rcc.cfgr.sysclk(168.mhz()).freeze();

  // 配置GPIO
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  let mut status_led = gpiob.pb0.into_push_pull_output();
  let mut measurement_led = gpiob.pb1.into_push_pull_output();
  let mut duration_led = gpiob.pb2.into_push_pull_output();
  let mut interval_led = gpiob.pb3.into_push_pull_output();
  let mut frequency_led = gpiob.pb4.into_push_pull_output();
  let mut accuracy_led = gpiob.pb5.into_push_pull_output();
  let mut overflow_led = gpiob.pb6.into_push_pull_output();
  let mut error_led = gpiob.pb7.into_push_pull_output();

  let mode_button = gpioc.pc13.into_pull_up_input();

  // 初始化测量事件队列
  let (producer, consumer) = unsafe { MEASUREMENT_QUEUE.split() };
  unsafe {
    MEASUREMENT_PRODUCER = Some(producer);
    MEASUREMENT_CONSUMER = Some(consumer);
  }

  // 配置定时器用于测量基准
  let mut timer2 = Timer::tim2(dp.TIM2, &clocks);
  let mut timer3 = Timer::tim3(dp.TIM3, &clocks);
  let mut timer4 = Timer::tim4(dp.TIM4, &clocks);
  let mut timer5 = Timer::tim5(dp.TIM5, &clocks);

  // 设置不同频率的定时器用于测量测试
  timer2.start(1000.hz()); // 1kHz - 1ms间隔
  timer3.start(10000.hz()); // 10kHz - 100μs间隔
  timer4.start(100000.hz()); // 100kHz - 10μs间隔
  timer5.start(1.hz()); // 1Hz - 系统时间更新

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

  // 创建时间测量实例
  let mut time_measurement = TimeMeasurement::new(168_000_000); // 168MHz时钟
  time_measurement.init().ok();

  // 配置延时定时器
  let mut delay_timer = Timer::tim6(dp.TIM6, &clocks).delay_ms();

  // 系统启动指示
  status_led.set_high();
  delay_timer.delay_ms(1000u32);

  // 时间测量管理器
  let mut measurement_manager = TimeMeasurementManager::new(time_measurement);
  let mut button_debouncer = ButtonDebouncer::new();
  let mut demo_counter = 0u32;

  loop {
    // 更新系统时间
    demo_counter += 1;
    let current_time_us = demo_counter * 100; // 100μs per loop
    SYSTEM_TIME_US.store(current_time_us, Ordering::Relaxed);
    update_global_time(current_time_us as u64 * 1000);

    // 按钮防抖处理
    let button_pressed = button_debouncer.update(mode_button.is_low());

    if button_pressed {
      // 切换测量模式
      let current_mode = MEASUREMENT_MODE.load(Ordering::Relaxed);
      let new_mode = (current_mode + 1) % 4;
      MEASUREMENT_MODE.store(new_mode, Ordering::Relaxed);

      // 应用新的测量模式
      measurement_manager.set_measurement_mode(new_mode);

      // 重置统计
      measurement_manager.reset_statistics();

      // 模式切换指示
      for _ in 0..3 {
        measurement_led.set_low();
        delay_timer.delay_ms(100u32);
        measurement_led.set_high();
        delay_timer.delay_ms(100u32);
      }
    }

    // 处理测量事件
    if let Some(consumer) = unsafe { MEASUREMENT_CONSUMER.as_mut() } {
      while let Some(event) = consumer.dequeue() {
        measurement_manager.handle_measurement_event(event);
      }
    }

    // 执行测量任务
    measurement_manager.run_measurement_tasks();

    // 显示测量状态
    display_measurement_status(
      &measurement_manager,
      &mut duration_led,
      &mut interval_led,
      &mut frequency_led,
      &mut accuracy_led,
      &mut overflow_led,
      &mut error_led,
      demo_counter,
    );

    // 状态LED心跳
    if demo_counter % 100 == 0 {
      status_led.toggle();
    }

    // 测量活动指示
    measurement_led
      .set_state(MEASUREMENT_ACTIVE.load(Ordering::Relaxed).into())
      .ok();

    delay_timer.delay_ms(100u32);
  }
}

/// 显示测量状态
fn display_measurement_status(
  manager: &TimeMeasurementManager,
  duration_led: &mut impl embedded_hal::digital::v2::OutputPin,
  interval_led: &mut impl embedded_hal::digital::v2::OutputPin,
  frequency_led: &mut impl embedded_hal::digital::v2::OutputPin,
  accuracy_led: &mut impl embedded_hal::digital::v2::OutputPin,
  overflow_led: &mut impl embedded_hal::digital::v2::OutputPin,
  error_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  let stats = manager.get_measurement_statistics();
  let mode = MEASUREMENT_MODE.load(Ordering::Relaxed);

  // 根据测量模式显示LED
  match mode {
    0 => {
      // 持续时间测量模式
      duration_led.set_state(((counter / 8) % 2 == 0).into()).ok();
      interval_led.set_low().ok();
      frequency_led.set_low().ok();
    }
    1 => {
      // 间隔测量模式
      duration_led.set_low().ok();
      interval_led
        .set_state(((counter / 10) % 2 == 0).into())
        .ok();
      frequency_led.set_low().ok();
    }
    2 => {
      // 频率测量模式
      duration_led.set_low().ok();
      interval_led.set_low().ok();
      frequency_led
        .set_state(((counter / 12) % 2 == 0).into())
        .ok();
    }
    3 => {
      // 综合测量模式
      duration_led.set_state(((counter / 6) % 2 == 0).into()).ok();
      interval_led.set_state(((counter / 8) % 2 == 0).into()).ok();
      frequency_led
        .set_state(((counter / 10) % 2 == 0).into())
        .ok();
    }
    _ => {}
  }

  // 精度指示
  if stats.accuracy_percentage > 95.0 {
    accuracy_led.set_high().ok();
  } else if stats.accuracy_percentage > 90.0 {
    accuracy_led
      .set_state(((counter / 15) % 2 == 0).into())
      .ok();
  } else {
    accuracy_led.set_low().ok();
  }

  // 溢出指示
  if stats.overflow_count > 0 {
    overflow_led.set_state(((counter / 5) % 2 == 0).into()).ok();
  } else {
    overflow_led.set_low().ok();
  }

  // 错误指示
  if stats.error_count > 0 {
    error_led.set_state(((counter / 7) % 2 == 0).into()).ok();
  } else {
    error_led.set_low().ok();
  }
}

/// 时间测量管理器
struct TimeMeasurementManager {
  time_measurement: TimeMeasurement,
  current_mode: u8,
  measurement_stats: MeasurementStatistics,
  active_measurements: [Option<ActiveMeasurement>; 8],
  measurement_history: [MeasurementResult; 32],
  history_index: usize,
}

impl TimeMeasurementManager {
  fn new(time_measurement: TimeMeasurement) -> Self {
    Self {
      time_measurement,
      current_mode: 0,
      measurement_stats: MeasurementStatistics::default(),
      active_measurements: [None; 8],
      measurement_history: [MeasurementResult::default(); 32],
      history_index: 0,
    }
  }

  fn set_measurement_mode(&mut self, mode: u8) {
    self.current_mode = mode;

    // 停止所有活动测量
    for measurement in &mut self.active_measurements {
      if let Some(active) = measurement {
        if let Ok(result) = self.time_measurement.stop_measurement(active.id) {
          self.add_measurement_result(result);
        }
      }
      *measurement = None;
    }

    // 根据模式启动新的测量
    match mode {
      0 => self.start_duration_measurements(),
      1 => self.start_interval_measurements(),
      2 => self.start_frequency_measurements(),
      3 => self.start_comprehensive_measurements(),
      _ => {}
    }
  }

  fn start_duration_measurements(&mut self) {
    // 启动持续时间测量
    for i in 0..4 {
      if let Ok(id) = self
        .time_measurement
        .start_measurement(MeasurementType::Duration)
      {
        self.active_measurements[i] = Some(ActiveMeasurement {
          id,
          measurement_type: MeasurementType::Duration,
          start_time: get_global_time(),
          expected_duration: (i + 1) as u64 * 1000000, // 1-4ms
        });
      }
    }
  }

  fn start_interval_measurements(&mut self) {
    // 启动间隔测量
    for i in 0..4 {
      if let Ok(id) = self
        .time_measurement
        .start_measurement(MeasurementType::Interval)
      {
        self.active_measurements[i] = Some(ActiveMeasurement {
          id,
          measurement_type: MeasurementType::Interval,
          start_time: get_global_time(),
          expected_duration: (i + 1) as u64 * 100000, // 100μs-400μs
        });
      }
    }
  }

  fn start_frequency_measurements(&mut self) {
    // 启动频率测量
    for i in 0..4 {
      if let Ok(id) = self
        .time_measurement
        .start_measurement(MeasurementType::Frequency)
      {
        self.active_measurements[i] = Some(ActiveMeasurement {
          id,
          measurement_type: MeasurementType::Frequency,
          start_time: get_global_time(),
          expected_duration: 1000000, // 1ms测量窗口
        });
      }
    }
  }

  fn start_comprehensive_measurements(&mut self) {
    // 启动综合测量
    let measurement_types = [
      MeasurementType::Duration,
      MeasurementType::Interval,
      MeasurementType::Frequency,
      MeasurementType::Period,
    ];

    for (i, &measurement_type) in measurement_types.iter().enumerate() {
      if let Ok(id) = self.time_measurement.start_measurement(measurement_type) {
        self.active_measurements[i] = Some(ActiveMeasurement {
          id,
          measurement_type,
          start_time: get_global_time(),
          expected_duration: 500000, // 500μs
        });
      }
    }
  }

  fn handle_measurement_event(&mut self, event: MeasurementEvent) {
    self.measurement_stats.total_events += 1;

    // 查找对应的活动测量
    for measurement in &mut self.active_measurements {
      if let Some(active) = measurement {
        if active.id == event.measurement_id {
          // 更新测量统计
          let actual_duration = event.timestamp - active.start_time;
          let error = actual_duration.abs_diff(active.expected_duration);

          self.measurement_stats.total_error += error;
          self.measurement_stats.measurement_count += 1;

          if error > self.measurement_stats.max_error {
            self.measurement_stats.max_error = error;
          }

          // 计算精度
          let accuracy = if active.expected_duration > 0 {
            100.0 - (error as f32 / active.expected_duration as f32 * 100.0)
          } else {
            0.0
          };

          self.measurement_stats.accuracy_percentage =
            (self.measurement_stats.accuracy_percentage + accuracy) / 2.0;

          break;
        }
      }
    }

    MEASUREMENT_COUNT.fetch_add(1, Ordering::Relaxed);
  }

  fn run_measurement_tasks(&mut self) {
    let current_time = get_global_time();

    // 检查活动测量是否应该停止
    for measurement in &mut self.active_measurements {
      if let Some(active) = measurement {
        let elapsed = current_time - active.start_time;

        if elapsed >= active.expected_duration {
          // 停止测量
          if let Ok(result) = self.time_measurement.stop_measurement(active.id) {
            self.add_measurement_result(result);
          }

          // 重新启动相同类型的测量
          if let Ok(new_id) = self
            .time_measurement
            .start_measurement(active.measurement_type)
          {
            active.id = new_id;
            active.start_time = current_time;
          } else {
            *measurement = None;
          }
        }
      }
    }

    // 更新统计信息
    self.update_duration_statistics();

    // 设置测量活动状态
    let has_active = self.active_measurements.iter().any(|m| m.is_some());
    MEASUREMENT_ACTIVE.store(has_active, Ordering::Relaxed);
  }

  fn add_measurement_result(&mut self, result: MeasurementResult) {
    self.measurement_history[self.history_index] = result;
    self.history_index = (self.history_index + 1) % self.measurement_history.len();

    TOTAL_MEASUREMENTS.fetch_add(1, Ordering::Relaxed);
  }

  fn update_duration_statistics(&self) {
    let mut min_duration = u32::MAX;
    let mut max_duration = 0u32;
    let mut total_duration = 0u64;
    let mut count = 0u32;

    for result in &self.measurement_history {
      if result.duration_ns > 0 {
        let duration_us = (result.duration_ns / 1000) as u32;

        if duration_us < min_duration {
          min_duration = duration_us;
        }

        if duration_us > max_duration {
          max_duration = duration_us;
        }

        total_duration += duration_us as u64;
        count += 1;
      }
    }

    if count > 0 {
      DURATION_STATS[0].store(min_duration, Ordering::Relaxed);
      DURATION_STATS[1].store(max_duration, Ordering::Relaxed);
      DURATION_STATS[2].store((total_duration / count as u64) as u32, Ordering::Relaxed);
      DURATION_STATS[3].store(total_duration as u32, Ordering::Relaxed);
    }
  }

  fn get_measurement_statistics(&self) -> &MeasurementStatistics {
    &self.measurement_stats
  }

  fn reset_statistics(&mut self) {
    self.measurement_stats = MeasurementStatistics::default();

    // 重置计数器
    MEASUREMENT_COUNT.store(0, Ordering::Relaxed);
    TOTAL_MEASUREMENTS.store(0, Ordering::Relaxed);

    // 重置统计数组
    for stat in &DURATION_STATS {
      stat.store(0, Ordering::Relaxed);
    }

    // 清空历史记录
    self.measurement_history = [MeasurementResult::default(); 32];
    self.history_index = 0;
  }
}

/// 测量统计信息
#[derive(Default)]
struct MeasurementStatistics {
  total_events: u32,
  measurement_count: u32,
  total_error: u64,
  max_error: u64,
  accuracy_percentage: f32,
  overflow_count: u32,
  error_count: u32,
}

/// 活动测量
struct ActiveMeasurement {
  id: u8,
  measurement_type: MeasurementType,
  start_time: u64,
  expected_duration: u64,
}

/// 测量事件
#[derive(Clone, Copy)]
struct MeasurementEvent {
  measurement_id: u8,
  timestamp: u64,
  event_type: u8,
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
  // 1ms间隔测量事件
  let event = MeasurementEvent {
    measurement_id: 1,
    timestamp: get_global_time(),
    event_type: 0,
  };

  if let Some(producer) = unsafe { MEASUREMENT_PRODUCER.as_mut() } {
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
  // 100μs间隔测量事件
  let event = MeasurementEvent {
    measurement_id: 2,
    timestamp: get_global_time(),
    event_type: 1,
  };

  if let Some(producer) = unsafe { MEASUREMENT_PRODUCER.as_mut() } {
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
  // 10μs间隔测量事件
  let event = MeasurementEvent {
    measurement_id: 3,
    timestamp: get_global_time(),
    event_type: 2,
  };

  if let Some(producer) = unsafe { MEASUREMENT_PRODUCER.as_mut() } {
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
  // 系统时间更新
  let current_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
  update_global_time(current_time as u64 * 1000);

  // 清除中断标志
  unsafe {
    let tim5 = &(*stm32::TIM5::ptr());
    tim5.sr.modify(|_, w| w.uif().clear_bit());
  }
}
