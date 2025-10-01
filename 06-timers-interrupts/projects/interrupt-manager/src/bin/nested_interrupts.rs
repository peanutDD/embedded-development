#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use interrupt_manager::{
  get_global_interrupt_manager, init_global_interrupt_manager, interrupt_handler, InterruptId,
  InterruptManager, Priority,
};
use panic_halt as _;
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
type Level1Led = PB1<Output<PushPull>>;
type Level2Led = PB2<Output<PushPull>>;
type Level3Led = PB3<Output<PushPull>>;
type Level4Led = PB4<Output<PushPull>>;
type NestedLed = PB5<Output<PushPull>>;
type OverflowLed = PB6<Output<PushPull>>;
type ErrorLed = PB7<Output<PushPull>>;

type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut NESTED_QUEUE: Queue<NestedEvent, 128> = Queue::new();
static mut NESTED_PRODUCER: Option<Producer<NestedEvent, 128>> = None;
static mut NESTED_CONSUMER: Option<Consumer<NestedEvent, 128>> = None;

static SYSTEM_TIME_US: AtomicU32 = AtomicU32::new(0);
static NESTED_MODE: AtomicU8 = AtomicU8::new(0);
static CURRENT_NESTED_LEVEL: AtomicU16 = AtomicU16::new(0);
static MAX_NESTED_LEVEL: AtomicU16 = AtomicU16::new(0);
static NESTED_OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);
static NESTED_ENABLED: AtomicBool = AtomicBool::new(true);

// 嵌套级别计数器
static NESTED_COUNTERS: [AtomicU32; 5] = [
  AtomicU32::new(0), // Level 0
  AtomicU32::new(0), // Level 1
  AtomicU32::new(0), // Level 2
  AtomicU32::new(0), // Level 3
  AtomicU32::new(0), // Level 4+
];

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  let mut status_led = gpiob.pb0.into_push_pull_output();
  let mut level1_led = gpiob.pb1.into_push_pull_output();
  let mut level2_led = gpiob.pb2.into_push_pull_output();
  let mut level3_led = gpiob.pb3.into_push_pull_output();
  let mut level4_led = gpiob.pb4.into_push_pull_output();
  let mut nested_led = gpiob.pb5.into_push_pull_output();
  let mut overflow_led = gpiob.pb6.into_push_pull_output();
  let mut error_led = gpiob.pb7.into_push_pull_output();

  let mode_button = gpioc.pc13.into_pull_up_input();

  // 初始化全局中断管理器
  init_global_interrupt_manager();

  // 初始化嵌套事件队列
  let (producer, consumer) = unsafe { NESTED_QUEUE.split() };
  unsafe {
    NESTED_PRODUCER = Some(producer);
    NESTED_CONSUMER = Some(consumer);
  }

  // 配置定时器 - 不同优先级的中断源
  let mut timer2 = Timer::tim2(dp.TIM2, &clocks);
  let mut timer3 = Timer::tim3(dp.TIM3, &clocks);
  let mut timer4 = Timer::tim4(dp.TIM4, &clocks);
  let mut timer5 = Timer::tim5(dp.TIM5, &clocks);

  // 设置不同频率的定时器
  timer2.start(500.hz()); // 500Hz - 最高优先级
  timer3.start(300.hz()); // 300Hz - 高优先级
  timer4.start(200.hz()); // 200Hz - 中等优先级
  timer5.start(100.hz()); // 100Hz - 低优先级

  // 启用定时器中断
  timer2.listen(Event::TimeOut);
  timer3.listen(Event::TimeOut);
  timer4.listen(Event::TimeOut);
  timer5.listen(Event::TimeOut);

  // 配置中断管理器和优先级
  if let Some(manager) = get_global_interrupt_manager() {
    // 注册中断 - 设置不同优先级以允许嵌套
    manager
      .register_interrupt(InterruptId::Timer2, Priority::Highest)
      .ok();
    manager
      .register_interrupt(InterruptId::Timer3, Priority::High)
      .ok();
    manager
      .register_interrupt(InterruptId::Timer4, Priority::Medium)
      .ok();
    manager
      .register_interrupt(InterruptId::Timer5, Priority::Low)
      .ok();

    // 启用中断
    manager.enable_interrupt(InterruptId::Timer2).ok();
    manager.enable_interrupt(InterruptId::Timer3).ok();
    manager.enable_interrupt(InterruptId::Timer4).ok();
    manager.enable_interrupt(InterruptId::Timer5).ok();
  }

  // 配置NVIC优先级以允许嵌套
  unsafe {
    // 设置不同的抢占优先级
    NVIC::set_priority(stm32::Interrupt::TIM2, 0 << 4); // 最高优先级
    NVIC::set_priority(stm32::Interrupt::TIM3, 1 << 4); // 高优先级
    NVIC::set_priority(stm32::Interrupt::TIM4, 2 << 4); // 中等优先级
    NVIC::set_priority(stm32::Interrupt::TIM5, 3 << 4); // 低优先级

    // 启用中断
    NVIC::unmask(stm32::Interrupt::TIM2);
    NVIC::unmask(stm32::Interrupt::TIM3);
    NVIC::unmask(stm32::Interrupt::TIM4);
    NVIC::unmask(stm32::Interrupt::TIM5);
  }

  // 配置延时定时器
  let mut delay_timer = Timer::tim6(dp.TIM6, &clocks).delay_ms();

  // 系统启动指示
  status_led.set_high();
  delay_timer.delay_ms(1000u32);

  // 嵌套中断管理器
  let mut nested_manager = NestedInterruptManager::new();
  let mut button_debouncer = ButtonDebouncer::new();
  let mut demo_counter = 0u32;

  loop {
    // 更新系统时间
    demo_counter += 1;
    let current_time = demo_counter * 20; // 20ms per loop
    SYSTEM_TIME_US.store(current_time * 1000, Ordering::Relaxed);

    if let Some(manager) = get_global_interrupt_manager() {
      manager.update_system_time(current_time * 1000);
    }

    // 按钮防抖处理
    let button_pressed = button_debouncer.update(mode_button.is_low());

    if button_pressed {
      // 切换嵌套模式
      let current_mode = NESTED_MODE.load(Ordering::Relaxed);
      let new_mode = (current_mode + 1) % 4;
      NESTED_MODE.store(new_mode, Ordering::Relaxed);

      // 应用新的嵌套配置
      nested_manager.apply_nested_mode(new_mode);

      // 重置统计
      nested_manager.reset_statistics();

      // 模式切换指示
      for _ in 0..3 {
        nested_led.set_low();
        delay_timer.delay_ms(100u32);
        nested_led.set_high();
        delay_timer.delay_ms(100u32);
      }
    }

    // 处理嵌套事件
    if let Some(consumer) = unsafe { NESTED_CONSUMER.as_mut() } {
      while let Some(event) = consumer.dequeue() {
        nested_manager.handle_nested_event(event);
      }
    }

    // 显示嵌套状态
    display_nested_status(
      &nested_manager,
      &mut level1_led,
      &mut level2_led,
      &mut level3_led,
      &mut level4_led,
      &mut overflow_led,
      &mut error_led,
      demo_counter,
    );

    // 状态LED心跳
    if demo_counter % 50 == 0 {
      status_led.toggle();
    }

    // 嵌套指示LED
    let current_level = CURRENT_NESTED_LEVEL.load(Ordering::Relaxed);
    nested_led.set_state((current_level > 0).into()).ok();

    delay_timer.delay_ms(20u32);
  }
}

/// 显示嵌套状态
fn display_nested_status(
  manager: &NestedInterruptManager,
  level1_led: &mut impl embedded_hal::digital::v2::OutputPin,
  level2_led: &mut impl embedded_hal::digital::v2::OutputPin,
  level3_led: &mut impl embedded_hal::digital::v2::OutputPin,
  level4_led: &mut impl embedded_hal::digital::v2::OutputPin,
  overflow_led: &mut impl embedded_hal::digital::v2::OutputPin,
  error_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  let stats = manager.get_nested_statistics();

  // 根据嵌套级别计数显示LED
  let count1 = NESTED_COUNTERS[1].load(Ordering::Relaxed);
  let count2 = NESTED_COUNTERS[2].load(Ordering::Relaxed);
  let count3 = NESTED_COUNTERS[3].load(Ordering::Relaxed);
  let count4 = NESTED_COUNTERS[4].load(Ordering::Relaxed);

  level1_led
    .set_state((count1 > 0 && (counter / 10) % 2 == 0).into())
    .ok();
  level2_led
    .set_state((count2 > 0 && (counter / 15) % 2 == 0).into())
    .ok();
  level3_led
    .set_state((count3 > 0 && (counter / 20) % 2 == 0).into())
    .ok();
  level4_led
    .set_state((count4 > 0 && (counter / 25) % 2 == 0).into())
    .ok();

  // 溢出指示
  let overflow_count = NESTED_OVERFLOW_COUNT.load(Ordering::Relaxed);
  if overflow_count > 0 {
    overflow_led.set_state(((counter / 5) % 2 == 0).into()).ok();
  } else {
    overflow_led.set_low().ok();
  }

  // 错误指示
  if stats.error_count > 0 {
    error_led.set_state(((counter / 8) % 2 == 0).into()).ok();
  } else {
    error_led.set_low().ok();
  }
}

/// 嵌套中断管理器
struct NestedInterruptManager {
  current_mode: u8,
  nested_stats: NestedStatistics,
  max_allowed_level: u8,
}

impl NestedInterruptManager {
  fn new() -> Self {
    Self {
      current_mode: 0,
      nested_stats: NestedStatistics::default(),
      max_allowed_level: 4,
    }
  }

  fn apply_nested_mode(&mut self, mode: u8) {
    self.current_mode = mode;

    match mode {
      0 => {
        // 标准嵌套模式 - 允许所有级别嵌套
        self.max_allowed_level = 4;
        NESTED_ENABLED.store(true, Ordering::Relaxed);
      }
      1 => {
        // 限制嵌套模式 - 最多2级嵌套
        self.max_allowed_level = 2;
        NESTED_ENABLED.store(true, Ordering::Relaxed);
      }
      2 => {
        // 禁用嵌套模式 - 不允许嵌套
        self.max_allowed_level = 0;
        NESTED_ENABLED.store(false, Ordering::Relaxed);
      }
      3 => {
        // 动态嵌套模式 - 根据负载动态调整
        let cpu_load = self.estimate_cpu_load();
        if cpu_load > 80 {
          self.max_allowed_level = 1;
        } else if cpu_load > 50 {
          self.max_allowed_level = 2;
        } else {
          self.max_allowed_level = 4;
        }
        NESTED_ENABLED.store(true, Ordering::Relaxed);
      }
      _ => {}
    }
  }

  fn handle_nested_event(&mut self, event: NestedEvent) {
    self.nested_stats.total_events += 1;

    if event.nested_level > self.nested_stats.max_nested_level {
      self.nested_stats.max_nested_level = event.nested_level;
    }

    if event.nested_level > self.max_allowed_level {
      self.nested_stats.overflow_events += 1;
      NESTED_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    // 更新级别计数器
    let level_index = if event.nested_level > 4 {
      4
    } else {
      event.nested_level as usize
    };
    NESTED_COUNTERS[level_index].fetch_add(1, Ordering::Relaxed);

    self.nested_stats.avg_execution_time = (self.nested_stats.avg_execution_time
      * (self.nested_stats.total_events - 1) as f32
      + event.execution_time_us as f32)
      / self.nested_stats.total_events as f32;
  }

  fn get_nested_statistics(&self) -> &NestedStatistics {
    &self.nested_stats
  }

  fn reset_statistics(&mut self) {
    self.nested_stats = NestedStatistics::default();

    // 重置计数器
    for counter in &NESTED_COUNTERS {
      counter.store(0, Ordering::Relaxed);
    }

    NESTED_OVERFLOW_COUNT.store(0, Ordering::Relaxed);
    MAX_NESTED_LEVEL.store(0, Ordering::Relaxed);
  }

  fn estimate_cpu_load(&self) -> u32 {
    // 简化的CPU负载估算
    let total_time = self.nested_stats.total_events as f32 * self.nested_stats.avg_execution_time;
    let elapsed_time = SYSTEM_TIME_US.load(Ordering::Relaxed) as f32;

    if elapsed_time > 0.0 {
      ((total_time / elapsed_time) * 100.0) as u32
    } else {
      0
    }
  }
}

/// 嵌套统计信息
#[derive(Default)]
struct NestedStatistics {
  total_events: u32,
  max_nested_level: u8,
  overflow_events: u32,
  avg_execution_time: f32,
  error_count: u32,
}

/// 嵌套事件
#[derive(Clone, Copy)]
struct NestedEvent {
  interrupt_id: InterruptId,
  nested_level: u8,
  timestamp: u32,
  execution_time_us: u32,
  preempted_by: Option<InterruptId>,
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

/// 进入嵌套中断
fn enter_nested_interrupt(interrupt_id: InterruptId) -> u16 {
  let current_level = CURRENT_NESTED_LEVEL.fetch_add(1, Ordering::Relaxed);

  // 更新最大嵌套级别
  let max_level = MAX_NESTED_LEVEL.load(Ordering::Relaxed);
  if current_level + 1 > max_level {
    MAX_NESTED_LEVEL.store(current_level + 1, Ordering::Relaxed);
  }

  current_level
}

/// 退出嵌套中断
fn exit_nested_interrupt(interrupt_id: InterruptId, entry_level: u16) {
  CURRENT_NESTED_LEVEL.fetch_sub(1, Ordering::Relaxed);
}

// 中断处理函数
#[interrupt]
fn TIM2() {
  let entry_level = enter_nested_interrupt(InterruptId::Timer2);
  let start_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

  interrupt_handler!(InterruptId::Timer2, {
    // 高优先级中断 - 快速处理
    for _ in 0..100 {
      cortex_m::asm::nop();
    }

    // 在处理过程中可能被更高优先级中断打断
    // 这里模拟一些可能被打断的处理
    for i in 0..50 {
      cortex_m::asm::nop();
      if i % 10 == 0 {
        // 允许其他中断有机会执行
        cortex_m::asm::wfi();
      }
    }
  });

  let end_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
  let execution_time = end_time.wrapping_sub(start_time);

  // 创建嵌套事件
  let event = NestedEvent {
    interrupt_id: InterruptId::Timer2,
    nested_level: entry_level as u8,
    timestamp: start_time,
    execution_time_us: execution_time,
    preempted_by: None,
  };

  if let Some(producer) = unsafe { NESTED_PRODUCER.as_mut() } {
    producer.enqueue(event).ok();
  }

  exit_nested_interrupt(InterruptId::Timer2, entry_level);

  // 清除中断标志
  unsafe {
    let tim2 = &(*stm32::TIM2::ptr());
    tim2.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM3() {
  let entry_level = enter_nested_interrupt(InterruptId::Timer3);
  let start_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

  interrupt_handler!(InterruptId::Timer3, {
    // 中等优先级中断 - 中等处理时间
    for _ in 0..300 {
      cortex_m::asm::nop();
    }

    // 模拟一些计算，可能被TIM2中断打断
    let mut sum = 0u32;
    for i in 0..100 {
      sum = sum.wrapping_add(i);
      if i % 20 == 0 {
        cortex_m::asm::wfi();
      }
    }
  });

  let end_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
  let execution_time = end_time.wrapping_sub(start_time);

  // 创建嵌套事件
  let event = NestedEvent {
    interrupt_id: InterruptId::Timer3,
    nested_level: entry_level as u8,
    timestamp: start_time,
    execution_time_us: execution_time,
    preempted_by: None,
  };

  if let Some(producer) = unsafe { NESTED_PRODUCER.as_mut() } {
    producer.enqueue(event).ok();
  }

  exit_nested_interrupt(InterruptId::Timer3, entry_level);

  // 清除中断标志
  unsafe {
    let tim3 = &(*stm32::TIM3::ptr());
    tim3.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM4() {
  let entry_level = enter_nested_interrupt(InterruptId::Timer4);
  let start_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

  interrupt_handler!(InterruptId::Timer4, {
    // 低优先级中断 - 较长处理时间
    for _ in 0..500 {
      cortex_m::asm::nop();
    }

    // 模拟复杂处理，很可能被高优先级中断打断
    for i in 0..200 {
      let mut temp = i * i;
      temp = temp.wrapping_add(i);

      if i % 30 == 0 {
        cortex_m::asm::wfi();
      }
    }
  });

  let end_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
  let execution_time = end_time.wrapping_sub(start_time);

  // 创建嵌套事件
  let event = NestedEvent {
    interrupt_id: InterruptId::Timer4,
    nested_level: entry_level as u8,
    timestamp: start_time,
    execution_time_us: execution_time,
    preempted_by: None,
  };

  if let Some(producer) = unsafe { NESTED_PRODUCER.as_mut() } {
    producer.enqueue(event).ok();
  }

  exit_nested_interrupt(InterruptId::Timer4, entry_level);

  // 清除中断标志
  unsafe {
    let tim4 = &(*stm32::TIM4::ptr());
    tim4.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM5() {
  let entry_level = enter_nested_interrupt(InterruptId::Timer5);
  let start_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

  interrupt_handler!(InterruptId::Timer5, {
    // 最低优先级中断 - 最长处理时间
    for _ in 0..1000 {
      cortex_m::asm::nop();
    }

    // 模拟非常复杂的处理，经常被其他中断打断
    for i in 0..500 {
      let mut result = 1u32;
      for j in 1..10 {
        result = result.wrapping_mul(j);
      }

      if i % 50 == 0 {
        cortex_m::asm::wfi();
      }
    }
  });

  let end_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
  let execution_time = end_time.wrapping_sub(start_time);

  // 创建嵌套事件
  let event = NestedEvent {
    interrupt_id: InterruptId::Timer5,
    nested_level: entry_level as u8,
    timestamp: start_time,
    execution_time_us: execution_time,
    preempted_by: None,
  };

  if let Some(producer) = unsafe { NESTED_PRODUCER.as_mut() } {
    producer.enqueue(event).ok();
  }

  exit_nested_interrupt(InterruptId::Timer5, entry_level);

  // 清除中断标志
  unsafe {
    let tim5 = &(*stm32::TIM5::ptr());
    tim5.sr.modify(|_, w| w.uif().clear_bit());
  }
}
