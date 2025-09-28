#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};
use cortex_m::peripheral::{DWT, NVIC};
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use panic_halt as _;
use precision_timer::{
  get_global_time, update_global_time, FrequencyConfig, FrequencyGenerator, WaveformType,
};
use stm32f4xx_hal::{
  gpio::{
    gpioa::{PA0, PA1, PA2, PA3},
    Alternate, AF1,
  },
  gpio::{
    gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7},
    Output, PushPull,
  },
  gpio::{gpioc::PC13, Input, PullUp},
  interrupt,
  prelude::*,
  pwm::PwmChannels,
  stm32,
  timer::{Channel, Event, PwmChannel, Timer},
};

// PWM输出引脚类型定义
type PwmPin1 = PA0<Alternate<AF1>>;
type PwmPin2 = PA1<Alternate<AF1>>;
type PwmPin3 = PA2<Alternate<AF1>>;
type PwmPin4 = PA3<Alternate<AF1>>;

// LED类型定义
type StatusLed = PB0<Output<PushPull>>;
type GeneratorLed = PB1<Output<PushPull>>;
type FrequencyLed = PB2<Output<PushPull>>;
type WaveformLed = PB3<Output<PushPull>>;
type SyncLed = PB4<Output<PushPull>>;
type AccuracyLed = PB5<Output<PushPull>>;
type OverloadLed = PB6<Output<PushPull>>;
type ErrorLed = PB7<Output<PushPull>>;

type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut FREQUENCY_QUEUE: Queue<FrequencyEvent, 128> = Queue::new();
static mut FREQUENCY_PRODUCER: Option<Producer<FrequencyEvent, 128>> = None;
static mut FREQUENCY_CONSUMER: Option<Consumer<FrequencyEvent, 128>> = None;

static SYSTEM_TIME_US: AtomicU32 = AtomicU32::new(0);
static GENERATOR_MODE: AtomicU8 = AtomicU8::new(0);
static TARGET_FREQUENCY: AtomicU32 = AtomicU32::new(1000); // 1kHz
static ACTUAL_FREQUENCY: AtomicU32 = AtomicU32::new(0);
static GENERATOR_ACTIVE: AtomicBool = AtomicBool::new(false);

// 频率生成统计
static FREQUENCY_STATS: [AtomicU32; 4] = [
  AtomicU32::new(0), // 生成的周期数
  AtomicU32::new(0), // 频率误差(Hz)
  AtomicU32::new(0), // 占空比误差(%)
  AtomicU32::new(0), // 相位误差(度)
];

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = stm32::Peripherals::take().unwrap();
  let mut cp = cortex_m::Peripherals::take().unwrap();

  // 启用DWT计数器
  cp.DCB.enable_trace();
  cp.DWT.enable_cycle_counter();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(168.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // PWM输出引脚
  let pwm_pin1 = gpioa.pa0.into_alternate_af1();
  let pwm_pin2 = gpioa.pa1.into_alternate_af1();
  let pwm_pin3 = gpioa.pa2.into_alternate_af1();
  let pwm_pin4 = gpioa.pa3.into_alternate_af1();

  // LED引脚
  let mut status_led = gpiob.pb0.into_push_pull_output();
  let mut generator_led = gpiob.pb1.into_push_pull_output();
  let mut frequency_led = gpiob.pb2.into_push_pull_output();
  let mut waveform_led = gpiob.pb3.into_push_pull_output();
  let mut sync_led = gpiob.pb4.into_push_pull_output();
  let mut accuracy_led = gpiob.pb5.into_push_pull_output();
  let mut overload_led = gpiob.pb6.into_push_pull_output();
  let mut error_led = gpiob.pb7.into_push_pull_output();

  let mode_button = gpioc.pc13.into_pull_up_input();

  // 初始化频率事件队列
  let (producer, consumer) = unsafe { FREQUENCY_QUEUE.split() };
  unsafe {
    FREQUENCY_PRODUCER = Some(producer);
    FREQUENCY_CONSUMER = Some(consumer);
  }

  // 配置PWM定时器
  let mut pwm_timer1 = Timer::tim2(dp.TIM2, &clocks);
  let mut pwm_timer2 = Timer::tim3(dp.TIM3, &clocks);
  let mut pwm_timer3 = Timer::tim4(dp.TIM4, &clocks);
  let mut pwm_timer4 = Timer::tim5(dp.TIM5, &clocks);

  // 配置系统定时器
  let mut system_timer = Timer::tim6(dp.TIM6, &clocks);
  system_timer.start(1000.hz()); // 1kHz系统时钟
  system_timer.listen(Event::TimeOut);

  // 配置NVIC
  unsafe {
    NVIC::unmask(stm32::Interrupt::TIM6_DAC);
  }

  // 创建频率生成器实例
  let frequency_configs = [
    FrequencyConfig {
      frequency_hz: 1000.0,
      duty_cycle_percent: 50.0,
      phase_degrees: 0.0,
      waveform_type: WaveformType::Square,
    },
    FrequencyConfig {
      frequency_hz: 5000.0,
      duty_cycle_percent: 25.0,
      phase_degrees: 90.0,
      waveform_type: WaveformType::Square,
    },
    FrequencyConfig {
      frequency_hz: 10000.0,
      duty_cycle_percent: 75.0,
      phase_degrees: 180.0,
      waveform_type: WaveformType::Square,
    },
    FrequencyConfig {
      frequency_hz: 50000.0,
      duty_cycle_percent: 50.0,
      phase_degrees: 270.0,
      waveform_type: WaveformType::Square,
    },
  ];

  let mut frequency_generators = [
    FrequencyGenerator::new(frequency_configs[0], 168_000_000),
    FrequencyGenerator::new(frequency_configs[1], 168_000_000),
    FrequencyGenerator::new(frequency_configs[2], 168_000_000),
    FrequencyGenerator::new(frequency_configs[3], 168_000_000),
  ];

  // 初始化频率生成器
  for generator in &mut frequency_generators {
    generator.init().ok();
  }

  // 配置延时定时器
  let mut delay_timer = Timer::tim7(dp.TIM7, &clocks).delay_ms();

  // 系统启动指示
  status_led.set_high();
  delay_timer.delay_ms(1000u32);

  // 频率生成管理器
  let mut frequency_manager = FrequencyGeneratorManager::new(frequency_generators);
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
      // 切换生成器模式
      let current_mode = GENERATOR_MODE.load(Ordering::Relaxed);
      let new_mode = (current_mode + 1) % 4;
      GENERATOR_MODE.store(new_mode, Ordering::Relaxed);

      // 应用新的生成器模式
      frequency_manager.set_generator_mode(new_mode);

      // 重置统计
      frequency_manager.reset_statistics();

      // 模式切换指示
      for _ in 0..3 {
        generator_led.set_low();
        delay_timer.delay_ms(100u32);
        generator_led.set_high();
        delay_timer.delay_ms(100u32);
      }
    }

    // 处理频率事件
    if let Some(consumer) = unsafe { FREQUENCY_CONSUMER.as_mut() } {
      while let Some(event) = consumer.dequeue() {
        frequency_manager.handle_frequency_event(event);
      }
    }

    // 更新频率生成
    frequency_manager.update_frequency_generation();

    // 显示生成器状态
    display_generator_status(
      &frequency_manager,
      &mut frequency_led,
      &mut waveform_led,
      &mut sync_led,
      &mut accuracy_led,
      &mut overload_led,
      &mut error_led,
      demo_counter,
    );

    // 状态LED心跳
    if demo_counter % 100 == 0 {
      status_led.toggle();
    }

    // 生成器活动指示
    generator_led
      .set_state(GENERATOR_ACTIVE.load(Ordering::Relaxed).into())
      .ok();

    delay_timer.delay_ms(100u32);
  }
}

/// 显示生成器状态
fn display_generator_status(
  manager: &FrequencyGeneratorManager,
  frequency_led: &mut impl embedded_hal::digital::v2::OutputPin,
  waveform_led: &mut impl embedded_hal::digital::v2::OutputPin,
  sync_led: &mut impl embedded_hal::digital::v2::OutputPin,
  accuracy_led: &mut impl embedded_hal::digital::v2::OutputPin,
  overload_led: &mut impl embedded_hal::digital::v2::OutputPin,
  error_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  let stats = manager.get_generator_statistics();
  let mode = GENERATOR_MODE.load(Ordering::Relaxed);

  // 频率指示 - 根据目标频率调整闪烁速度
  let target_freq = TARGET_FREQUENCY.load(Ordering::Relaxed);
  let blink_rate = match target_freq {
    f if f < 1000 => 20,
    f if f < 10000 => 15,
    f if f < 50000 => 10,
    _ => 5,
  };
  frequency_led
    .set_state(((counter / blink_rate) % 2 == 0).into())
    .ok();

  // 波形类型指示
  match mode {
    0 => waveform_led.set_state(((counter / 8) % 2 == 0).into()).ok(), // 方波
    1 => waveform_led.set_state(((counter / 6) % 2 == 0).into()).ok(), // 三角波
    2 => waveform_led.set_state(((counter / 4) % 2 == 0).into()).ok(), // 正弦波
    3 => waveform_led
      .set_state(((counter / 12) % 2 == 0).into())
      .ok(), // 自定义
    _ => waveform_led.set_low().ok(),
  }

  // 同步指示
  if stats.phase_sync_error < 5.0 {
    sync_led.set_high().ok();
  } else if stats.phase_sync_error < 15.0 {
    sync_led.set_state(((counter / 10) % 2 == 0).into()).ok();
  } else {
    sync_led.set_low().ok();
  }

  // 精度指示
  if stats.frequency_accuracy > 99.0 {
    accuracy_led.set_high().ok();
  } else if stats.frequency_accuracy > 95.0 {
    accuracy_led
      .set_state(((counter / 15) % 2 == 0).into())
      .ok();
  } else {
    accuracy_led.set_low().ok();
  }

  // 过载指示
  if stats.cpu_load > 80.0 {
    overload_led.set_state(((counter / 5) % 2 == 0).into()).ok();
  } else if stats.cpu_load > 60.0 {
    overload_led
      .set_state(((counter / 20) % 2 == 0).into())
      .ok();
  } else {
    overload_led.set_low().ok();
  }

  // 错误指示
  if stats.error_count > 0 {
    error_led.set_state(((counter / 7) % 2 == 0).into()).ok();
  } else {
    error_led.set_low().ok();
  }
}

/// 频率生成管理器
struct FrequencyGeneratorManager {
  generators: [FrequencyGenerator; 4],
  current_mode: u8,
  generator_stats: GeneratorStatistics,
  output_states: [bool; 4],
  last_update_time: u64,
  cycle_counters: [u32; 4],
}

impl FrequencyGeneratorManager {
  fn new(generators: [FrequencyGenerator; 4]) -> Self {
    Self {
      generators,
      current_mode: 0,
      generator_stats: GeneratorStatistics::default(),
      output_states: [false; 4],
      last_update_time: 0,
      cycle_counters: [0; 4],
    }
  }

  fn set_generator_mode(&mut self, mode: u8) {
    self.current_mode = mode;

    // 停止所有生成器
    for generator in &mut self.generators {
      generator.stop().ok();
    }

    // 根据模式配置生成器
    match mode {
      0 => self.setup_single_frequency_mode(),
      1 => self.setup_multi_frequency_mode(),
      2 => self.setup_sweep_frequency_mode(),
      3 => self.setup_synchronized_mode(),
      _ => {}
    }

    GENERATOR_ACTIVE.store(true, Ordering::Relaxed);
  }

  fn setup_single_frequency_mode(&mut self) {
    // 单频率模式 - 只启动第一个生成器
    let config = FrequencyConfig {
      frequency_hz: 1000.0,
      duty_cycle_percent: 50.0,
      phase_degrees: 0.0,
      waveform_type: WaveformType::Square,
    };

    self.generators[0].set_config(config).ok();
    self.generators[0].start().ok();

    TARGET_FREQUENCY.store(1000, Ordering::Relaxed);
  }

  fn setup_multi_frequency_mode(&mut self) {
    // 多频率模式 - 启动所有生成器，不同频率
    let frequencies = [1000.0, 2000.0, 5000.0, 10000.0];

    for (i, &freq) in frequencies.iter().enumerate() {
      let config = FrequencyConfig {
        frequency_hz: freq,
        duty_cycle_percent: 50.0,
        phase_degrees: 0.0,
        waveform_type: WaveformType::Square,
      };

      self.generators[i].set_config(config).ok();
      self.generators[i].start().ok();
    }

    TARGET_FREQUENCY.store(frequencies[0] as u32, Ordering::Relaxed);
  }

  fn setup_sweep_frequency_mode(&mut self) {
    // 扫频模式 - 频率会动态变化
    let config = FrequencyConfig {
      frequency_hz: 100.0, // 起始频率
      duty_cycle_percent: 50.0,
      phase_degrees: 0.0,
      waveform_type: WaveformType::Square,
    };

    self.generators[0].set_config(config).ok();
    self.generators[0].start().ok();

    TARGET_FREQUENCY.store(100, Ordering::Relaxed);
  }

  fn setup_synchronized_mode(&mut self) {
    // 同步模式 - 相同频率，不同相位
    let phases = [0.0, 90.0, 180.0, 270.0];

    for (i, &phase) in phases.iter().enumerate() {
      let config = FrequencyConfig {
        frequency_hz: 5000.0,
        duty_cycle_percent: 50.0,
        phase_degrees: phase,
        waveform_type: WaveformType::Square,
      };

      self.generators[i].set_config(config).ok();
      self.generators[i].start().ok();
    }

    TARGET_FREQUENCY.store(5000, Ordering::Relaxed);
  }

  fn handle_frequency_event(&mut self, event: FrequencyEvent) {
    self.generator_stats.total_events += 1;

    // 更新频率统计
    if event.generator_id < 4 {
      self.cycle_counters[event.generator_id as usize] += 1;

      // 计算实际频率
      let elapsed_time = event.timestamp - self.last_update_time;
      if elapsed_time > 1000000 {
        // 1秒
        let actual_freq = (self.cycle_counters[event.generator_id as usize] as f32 * 1000000.0)
          / elapsed_time as f32;
        ACTUAL_FREQUENCY.store(actual_freq as u32, Ordering::Relaxed);

        // 计算频率精度
        let target_freq = TARGET_FREQUENCY.load(Ordering::Relaxed) as f32;
        if target_freq > 0.0 {
          let accuracy = 100.0 - ((actual_freq - target_freq).abs() / target_freq * 100.0);
          self.generator_stats.frequency_accuracy = accuracy;
        }

        // 重置计数器
        self.cycle_counters[event.generator_id as usize] = 0;
        self.last_update_time = event.timestamp;
      }
    }
  }

  fn update_frequency_generation(&mut self) {
    let current_time = get_global_time();

    // 扫频模式的频率更新
    if self.current_mode == 2 {
      let elapsed_seconds = (current_time - self.last_update_time) / 1000000;
      if elapsed_seconds > 0 {
        // 每秒增加100Hz，从100Hz到10kHz循环
        let base_freq = 100.0;
        let max_freq = 10000.0;
        let sweep_rate = 100.0; // Hz/s

        let current_freq =
          base_freq + (elapsed_seconds as f32 * sweep_rate) % (max_freq - base_freq);

        let config = FrequencyConfig {
          frequency_hz: current_freq,
          duty_cycle_percent: 50.0,
          phase_degrees: 0.0,
          waveform_type: WaveformType::Square,
        };

        self.generators[0].set_config(config).ok();
        TARGET_FREQUENCY.store(current_freq as u32, Ordering::Relaxed);
      }
    }

    // 更新输出状态
    for (i, generator) in self.generators.iter().enumerate() {
      if let Ok(status) = generator.get_status() {
        self.output_states[i] = status.is_running;
      }
    }

    // 更新统计信息
    self.update_generator_statistics();
  }

  fn update_generator_statistics(&mut self) {
    // 计算CPU负载（简化估算）
    let active_generators = self.output_states.iter().filter(|&&state| state).count();
    let target_freq = TARGET_FREQUENCY.load(Ordering::Relaxed) as f32;

    // 基于活动生成器数量和频率估算CPU负载
    self.generator_stats.cpu_load = (active_generators as f32 * target_freq / 1000.0).min(100.0);

    // 更新相位同步误差（模拟）
    if self.current_mode == 3 {
      // 同步模式
      self.generator_stats.phase_sync_error = 2.0; // 良好同步
    } else {
      self.generator_stats.phase_sync_error = 45.0; // 无同步要求
    }

    // 更新统计数组
    FREQUENCY_STATS[0].store(self.cycle_counters.iter().sum(), Ordering::Relaxed);
    FREQUENCY_STATS[1].store(
      (TARGET_FREQUENCY.load(Ordering::Relaxed) as i32
        - ACTUAL_FREQUENCY.load(Ordering::Relaxed) as i32)
        .abs() as u32,
      Ordering::Relaxed,
    );
    FREQUENCY_STATS[2].store(
      (100.0 - self.generator_stats.frequency_accuracy) as u32,
      Ordering::Relaxed,
    );
    FREQUENCY_STATS[3].store(
      self.generator_stats.phase_sync_error as u32,
      Ordering::Relaxed,
    );
  }

  fn get_generator_statistics(&self) -> &GeneratorStatistics {
    &self.generator_stats
  }

  fn reset_statistics(&mut self) {
    self.generator_stats = GeneratorStatistics::default();

    // 重置计数器
    self.cycle_counters = [0; 4];
    ACTUAL_FREQUENCY.store(0, Ordering::Relaxed);

    // 重置统计数组
    for stat in &FREQUENCY_STATS {
      stat.store(0, Ordering::Relaxed);
    }

    self.last_update_time = get_global_time();
  }
}

/// 生成器统计信息
#[derive(Default)]
struct GeneratorStatistics {
  total_events: u32,
  frequency_accuracy: f32,
  phase_sync_error: f32,
  cpu_load: f32,
  error_count: u32,
}

/// 频率事件
#[derive(Clone, Copy)]
struct FrequencyEvent {
  generator_id: u8,
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
fn TIM6_DAC() {
  // 系统时钟中断 - 用于频率事件生成
  let current_time = get_global_time();

  // 为每个活动的生成器生成事件
  for i in 0..4 {
    let event = FrequencyEvent {
      generator_id: i,
      timestamp: current_time,
      event_type: 0,
    };

    if let Some(producer) = unsafe { FREQUENCY_PRODUCER.as_mut() } {
      producer.enqueue(event).ok();
    }
  }

  // 清除中断标志
  unsafe {
    let tim6 = &(*stm32::TIM6::ptr());
    tim6.sr.modify(|_, w| w.uif().clear_bit());
  }
}
