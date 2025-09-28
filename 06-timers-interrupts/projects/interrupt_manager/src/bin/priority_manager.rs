#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use interrupt_manager::{
  get_global_interrupt_manager, init_global_interrupt_manager, interrupt_handler, InterruptError,
  InterruptId, InterruptManager, Priority,
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
type Priority1Led = PB1<Output<PushPull>>;
type Priority2Led = PB2<Output<PushPull>>;
type Priority3Led = PB3<Output<PushPull>>;
type Priority4Led = PB4<Output<PushPull>>;
type ActiveLed = PB5<Output<PushPull>>;
type ErrorLed = PB6<Output<PushPull>>;
type NestedLed = PB7<Output<PushPull>>;

type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut EVENT_QUEUE: Queue<PriorityEvent, 64> = Queue::new();
static mut EVENT_PRODUCER: Option<Producer<PriorityEvent, 64>> = None;
static mut EVENT_CONSUMER: Option<Consumer<PriorityEvent, 64>> = None;

static SYSTEM_TIME_US: AtomicU32 = AtomicU32::new(0);
static DEMO_MODE: AtomicU8 = AtomicU8::new(0);
static INTERRUPT_COUNTERS: [AtomicU32; 4] = [
  AtomicU32::new(0),
  AtomicU32::new(0),
  AtomicU32::new(0),
  AtomicU32::new(0),
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
  let mut priority1_led = gpiob.pb1.into_push_pull_output();
  let mut priority2_led = gpiob.pb2.into_push_pull_output();
  let mut priority3_led = gpiob.pb3.into_push_pull_output();
  let mut priority4_led = gpiob.pb4.into_push_pull_output();
  let mut active_led = gpiob.pb5.into_push_pull_output();
  let mut error_led = gpiob.pb6.into_push_pull_output();
  let mut nested_led = gpiob.pb7.into_push_pull_output();

  let mode_button = gpioc.pc13.into_pull_up_input();

  // 初始化全局中断管理器
  init_global_interrupt_manager();

  // 初始化事件队列
  let (producer, consumer) = unsafe { EVENT_QUEUE.split() };
  unsafe {
    EVENT_PRODUCER = Some(producer);
    EVENT_CONSUMER = Some(consumer);
  }

  // 配置定时器
  let mut timer2 = Timer::tim2(dp.TIM2, &clocks);
  let mut timer3 = Timer::tim3(dp.TIM3, &clocks);
  let mut timer4 = Timer::tim4(dp.TIM4, &clocks);
  let mut timer5 = Timer::tim5(dp.TIM5, &clocks);

  // 设置不同频率的定时器
  timer2.start(1000.hz()); // 1kHz - 最高优先级
  timer3.start(500.hz()); // 500Hz - 高优先级
  timer4.start(250.hz()); // 250Hz - 中等优先级
  timer5.start(125.hz()); // 125Hz - 低优先级

  // 启用定时器中断
  timer2.listen(Event::TimeOut);
  timer3.listen(Event::TimeOut);
  timer4.listen(Event::TimeOut);
  timer5.listen(Event::TimeOut);

  // 配置中断管理器
  if let Some(manager) = get_global_interrupt_manager() {
    // 注册中断
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

  // 启用NVIC中断
  unsafe {
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

  // 优先级管理器
  let mut priority_manager = PriorityManager::new();
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
      // 切换演示模式
      let current_mode = DEMO_MODE.load(Ordering::Relaxed);
      let new_mode = (current_mode + 1) % 4;
      DEMO_MODE.store(new_mode, Ordering::Relaxed);

      // 应用新的优先级配置
      priority_manager.apply_demo_mode(new_mode);

      // 模式切换指示
      for _ in 0..3 {
        status_led.set_low();
        delay_timer.delay_ms(100u32);
        status_led.set_high();
        delay_timer.delay_ms(100u32);
      }
    }

    // 处理优先级事件
    if let Some(consumer) = unsafe { EVENT_CONSUMER.as_mut() } {
      while let Some(event) = consumer.dequeue() {
        active_led.set_high();
        priority_manager.handle_event(event);
        delay_timer.delay_ms(10u32);
        active_led.set_low();
      }
    }

    // 显示优先级状态
    display_priority_status(
      &priority_manager,
      &mut priority1_led,
      &mut priority2_led,
      &mut priority3_led,
      &mut priority4_led,
      &mut nested_led,
      &mut error_led,
      demo_counter,
    );

    // 状态LED心跳
    if demo_counter % 50 == 0 {
      status_led.toggle();
    }

    delay_timer.delay_ms(20u32);
  }
}

/// 显示优先级状态
fn display_priority_status(
  manager: &PriorityManager,
  priority1_led: &mut impl embedded_hal::digital::v2::OutputPin,
  priority2_led: &mut impl embedded_hal::digital::v2::OutputPin,
  priority3_led: &mut impl embedded_hal::digital::v2::OutputPin,
  priority4_led: &mut impl embedded_hal::digital::v2::OutputPin,
  nested_led: &mut impl embedded_hal::digital::v2::OutputPin,
  error_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  let stats = manager.get_priority_stats();

  // 根据中断计数显示LED
  let count1 = INTERRUPT_COUNTERS[0].load(Ordering::Relaxed);
  let count2 = INTERRUPT_COUNTERS[1].load(Ordering::Relaxed);
  let count3 = INTERRUPT_COUNTERS[2].load(Ordering::Relaxed);
  let count4 = INTERRUPT_COUNTERS[3].load(Ordering::Relaxed);

  priority1_led
    .set_state(((count1 / 10) % 2 == 0).into())
    .ok();
  priority2_led
    .set_state(((count2 / 20) % 2 == 0).into())
    .ok();
  priority3_led
    .set_state(((count3 / 40) % 2 == 0).into())
    .ok();
  priority4_led
    .set_state(((count4 / 80) % 2 == 0).into())
    .ok();

  // 嵌套中断指示
  if let Some(global_manager) = get_global_interrupt_manager() {
    let nested_level = global_manager.get_nested_level();
    nested_led.set_state((nested_level > 0).into()).ok();
  }

  // 错误指示
  if stats.error_count > 0 {
    error_led.set_state(((counter / 5) % 2 == 0).into()).ok();
  } else {
    error_led.set_low().ok();
  }
}

/// 优先级管理器
struct PriorityManager {
  current_mode: u8,
  priority_stats: PriorityStats,
  last_update_time: u32,
}

impl PriorityManager {
  fn new() -> Self {
    Self {
      current_mode: 0,
      priority_stats: PriorityStats::default(),
      last_update_time: 0,
    }
  }

  fn apply_demo_mode(&mut self, mode: u8) {
    self.current_mode = mode;

    if let Some(manager) = get_global_interrupt_manager() {
      match mode {
        0 => {
          // 标准优先级模式
          manager
            .set_priority(InterruptId::Timer2, Priority::Highest)
            .ok();
          manager
            .set_priority(InterruptId::Timer3, Priority::High)
            .ok();
          manager
            .set_priority(InterruptId::Timer4, Priority::Medium)
            .ok();
          manager
            .set_priority(InterruptId::Timer5, Priority::Low)
            .ok();
        }
        1 => {
          // 反向优先级模式
          manager
            .set_priority(InterruptId::Timer2, Priority::Low)
            .ok();
          manager
            .set_priority(InterruptId::Timer3, Priority::Medium)
            .ok();
          manager
            .set_priority(InterruptId::Timer4, Priority::High)
            .ok();
          manager
            .set_priority(InterruptId::Timer5, Priority::Highest)
            .ok();
        }
        2 => {
          // 均等优先级模式
          manager
            .set_priority(InterruptId::Timer2, Priority::Medium)
            .ok();
          manager
            .set_priority(InterruptId::Timer3, Priority::Medium)
            .ok();
          manager
            .set_priority(InterruptId::Timer4, Priority::Medium)
            .ok();
          manager
            .set_priority(InterruptId::Timer5, Priority::Medium)
            .ok();
        }
        3 => {
          // 动态优先级模式
          let time = SYSTEM_TIME_US.load(Ordering::Relaxed);
          let cycle = (time / 1_000_000) % 4; // 每秒切换一次

          match cycle {
            0 => {
              manager
                .set_priority(InterruptId::Timer2, Priority::Highest)
                .ok();
              manager
                .set_priority(InterruptId::Timer3, Priority::Low)
                .ok();
            }
            1 => {
              manager
                .set_priority(InterruptId::Timer3, Priority::Highest)
                .ok();
              manager
                .set_priority(InterruptId::Timer4, Priority::Low)
                .ok();
            }
            2 => {
              manager
                .set_priority(InterruptId::Timer4, Priority::Highest)
                .ok();
              manager
                .set_priority(InterruptId::Timer5, Priority::Low)
                .ok();
            }
            _ => {
              manager
                .set_priority(InterruptId::Timer5, Priority::Highest)
                .ok();
              manager
                .set_priority(InterruptId::Timer2, Priority::Low)
                .ok();
            }
          }
        }
        _ => {}
      }
    }
  }

  fn handle_event(&mut self, event: PriorityEvent) {
    self.priority_stats.total_events += 1;

    match event.priority {
      Priority::Highest => self.priority_stats.highest_count += 1,
      Priority::High => self.priority_stats.high_count += 1,
      Priority::Medium => self.priority_stats.medium_count += 1,
      Priority::Low => self.priority_stats.low_count += 1,
      Priority::Lowest => self.priority_stats.lowest_count += 1,
    }

    if event.execution_time_us > self.priority_stats.max_execution_time {
      self.priority_stats.max_execution_time = event.execution_time_us;
    }

    if event.nested_level > 0 {
      self.priority_stats.nested_events += 1;
    }
  }

  fn get_priority_stats(&self) -> &PriorityStats {
    &self.priority_stats
  }
}

/// 优先级统计
#[derive(Default)]
struct PriorityStats {
  total_events: u32,
  highest_count: u32,
  high_count: u32,
  medium_count: u32,
  low_count: u32,
  lowest_count: u32,
  nested_events: u32,
  max_execution_time: u32,
  error_count: u32,
}

/// 优先级事件
#[derive(Clone, Copy)]
struct PriorityEvent {
  interrupt_id: InterruptId,
  priority: Priority,
  timestamp: u32,
  execution_time_us: u32,
  nested_level: u8,
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
  interrupt_handler!(InterruptId::Timer2, {
    INTERRUPT_COUNTERS[0].fetch_add(1, Ordering::Relaxed);

    // 模拟一些处理时间
    for _ in 0..100 {
      cortex_m::asm::nop();
    }

    // 发送事件
    let event = PriorityEvent {
      interrupt_id: InterruptId::Timer2,
      priority: Priority::Highest,
      timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
      execution_time_us: 50, // 模拟执行时间
      nested_level: 0,
    };

    if let Some(producer) = unsafe { EVENT_PRODUCER.as_mut() } {
      producer.enqueue(event).ok();
    }
  });

  // 清除中断标志
  unsafe {
    let tim2 = &(*stm32::TIM2::ptr());
    tim2.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM3() {
  interrupt_handler!(InterruptId::Timer3, {
    INTERRUPT_COUNTERS[1].fetch_add(1, Ordering::Relaxed);

    // 模拟一些处理时间
    for _ in 0..200 {
      cortex_m::asm::nop();
    }

    // 发送事件
    let event = PriorityEvent {
      interrupt_id: InterruptId::Timer3,
      priority: Priority::High,
      timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
      execution_time_us: 100, // 模拟执行时间
      nested_level: 0,
    };

    if let Some(producer) = unsafe { EVENT_PRODUCER.as_mut() } {
      producer.enqueue(event).ok();
    }
  });

  // 清除中断标志
  unsafe {
    let tim3 = &(*stm32::TIM3::ptr());
    tim3.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM4() {
  interrupt_handler!(InterruptId::Timer4, {
    INTERRUPT_COUNTERS[2].fetch_add(1, Ordering::Relaxed);

    // 模拟一些处理时间
    for _ in 0..300 {
      cortex_m::asm::nop();
    }

    // 发送事件
    let event = PriorityEvent {
      interrupt_id: InterruptId::Timer4,
      priority: Priority::Medium,
      timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
      execution_time_us: 150, // 模拟执行时间
      nested_level: 0,
    };

    if let Some(producer) = unsafe { EVENT_PRODUCER.as_mut() } {
      producer.enqueue(event).ok();
    }
  });

  // 清除中断标志
  unsafe {
    let tim4 = &(*stm32::TIM4::ptr());
    tim4.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM5() {
  interrupt_handler!(InterruptId::Timer5, {
    INTERRUPT_COUNTERS[3].fetch_add(1, Ordering::Relaxed);

    // 模拟一些处理时间
    for _ in 0..400 {
      cortex_m::asm::nop();
    }

    // 发送事件
    let event = PriorityEvent {
      interrupt_id: InterruptId::Timer5,
      priority: Priority::Low,
      timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
      execution_time_us: 200, // 模拟执行时间
      nested_level: 0,
    };

    if let Some(producer) = unsafe { EVENT_PRODUCER.as_mut() } {
      producer.enqueue(event).ok();
    }
  });

  // 清除中断标志
  unsafe {
    let tim5 = &(*stm32::TIM5::ptr());
    tim5.sr.modify(|_, w| w.uif().clear_bit());
  }
}
