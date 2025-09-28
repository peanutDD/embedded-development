#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::gpioc::PC13,
  gpio::{
    gpiob::{PB0, PB1, PB2, PB3},
    Input, Output, PullUp, PushPull,
  },
  interrupt,
  prelude::*,
  stm32,
  timer::{Counter, Event, Timer},
};

type StatusLed = PB0<Output<PushPull>>;
type Timer1Led = PB1<Output<PushPull>>;
type Timer2Led = PB2<Output<PushPull>>;
type Timer3Led = PB3<Output<PushPull>>;
type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut TIMER_QUEUE: Queue<TimerEvent, 16> = Queue::new();
static mut TIMER_PRODUCER: Option<Producer<TimerEvent, 16>> = None;
static mut TIMER_CONSUMER: Option<Consumer<TimerEvent, 16>> = None;

static CASCADE_COUNTER: AtomicU32 = AtomicU32::new(0);
static TIMER1_COUNTER: AtomicU32 = AtomicU32::new(0);
static TIMER2_COUNTER: AtomicU32 = AtomicU32::new(0);
static TIMER3_COUNTER: AtomicU32 = AtomicU32::new(0);

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
  let mut timer1_led = gpiob.pb1.into_push_pull_output();
  let mut timer2_led = gpiob.pb2.into_push_pull_output();
  let mut timer3_led = gpiob.pb3.into_push_pull_output();
  let mode_button = gpioc.pc13.into_pull_up_input();

  // 初始化队列
  let (producer, consumer) = unsafe { TIMER_QUEUE.split() };
  unsafe {
    TIMER_PRODUCER = Some(producer);
    TIMER_CONSUMER = Some(consumer);
  }

  // 配置级联定时器系统
  let mut cascade_system = CascadeTimerSystem::new(dp.TIM2, dp.TIM3, dp.TIM4, &clocks);
  cascade_system.start();

  // 配置延时定时器
  let mut delay = Timer::tim5(dp.TIM5, &clocks).delay_ms();

  // 系统启动指示
  status_led.set_high();
  delay.delay_ms(1000u32);

  // 按钮防抖器
  let mut button_debouncer = ButtonDebouncer::new();
  let mut cascade_mode = CascadeMode::Sequential;
  let mut demo_counter = 0u32;

  loop {
    // 按钮防抖处理
    let button_pressed = button_debouncer.update(mode_button.is_low());

    if button_pressed {
      // 切换级联模式
      cascade_mode = match cascade_mode {
        CascadeMode::Sequential => CascadeMode::Parallel,
        CascadeMode::Parallel => CascadeMode::Synchronized,
        CascadeMode::Synchronized => CascadeMode::Frequency,
        CascadeMode::Frequency => CascadeMode::Sequential,
      };

      // 重新配置级联系统
      cascade_system.set_mode(cascade_mode);

      // 模式切换指示
      for _ in 0..3 {
        status_led.set_low();
        delay.delay_ms(100u32);
        status_led.set_high();
        delay.delay_ms(100u32);
      }
    }

    // 处理定时器事件
    if let Some(consumer) = unsafe { TIMER_CONSUMER.as_mut() } {
      while let Some(event) = consumer.dequeue() {
        handle_timer_event(event, &mut timer1_led, &mut timer2_led, &mut timer3_led);
      }
    }

    // 显示级联状态
    display_cascade_status(
      cascade_mode,
      &mut timer1_led,
      &mut timer2_led,
      &mut timer3_led,
      demo_counter,
    );

    demo_counter += 1;

    // 状态LED心跳
    if demo_counter % 100 == 0 {
      status_led.toggle();
    }

    delay.delay_ms(10u32);
  }
}

/// 处理定时器事件
fn handle_timer_event(
  event: TimerEvent,
  timer1_led: &mut impl embedded_hal::digital::v2::OutputPin,
  timer2_led: &mut impl embedded_hal::digital::v2::OutputPin,
  timer3_led: &mut impl embedded_hal::digital::v2::OutputPin,
) {
  match event {
    TimerEvent::Timer1Overflow => {
      timer1_led.toggle().ok();
      TIMER1_COUNTER.fetch_add(1, Ordering::Relaxed);
    }
    TimerEvent::Timer2Overflow => {
      timer2_led.toggle().ok();
      TIMER2_COUNTER.fetch_add(1, Ordering::Relaxed);
    }
    TimerEvent::Timer3Overflow => {
      timer3_led.toggle().ok();
      TIMER3_COUNTER.fetch_add(1, Ordering::Relaxed);
    }
    TimerEvent::CascadeComplete => {
      CASCADE_COUNTER.fetch_add(1, Ordering::Relaxed);
    }
  }
}

/// 显示级联状态
fn display_cascade_status(
  mode: CascadeMode,
  timer1_led: &mut impl embedded_hal::digital::v2::OutputPin,
  timer2_led: &mut impl embedded_hal::digital::v2::OutputPin,
  timer3_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  match mode {
    CascadeMode::Sequential => {
      // 顺序闪烁
      let phase = (counter / 20) % 3;
      timer1_led.set_state((phase == 0).into()).ok();
      timer2_led.set_state((phase == 1).into()).ok();
      timer3_led.set_state((phase == 2).into()).ok();
    }
    CascadeMode::Parallel => {
      // 并行闪烁（所有LED同时）
      let state = (counter / 50) % 2 == 0;
      timer1_led.set_state(state.into()).ok();
      timer2_led.set_state(state.into()).ok();
      timer3_led.set_state(state.into()).ok();
    }
    CascadeMode::Synchronized => {
      // 同步模式：根据计数器显示不同模式
      let pattern = (counter / 30) % 4;
      match pattern {
        0 => {
          timer1_led.set_high().ok();
          timer2_led.set_low().ok();
          timer3_led.set_low().ok();
        }
        1 => {
          timer1_led.set_low().ok();
          timer2_led.set_high().ok();
          timer3_led.set_low().ok();
        }
        2 => {
          timer1_led.set_low().ok();
          timer2_led.set_low().ok();
          timer3_led.set_high().ok();
        }
        _ => {
          timer1_led.set_low().ok();
          timer2_led.set_low().ok();
          timer3_led.set_low().ok();
        }
      }
    }
    CascadeMode::Frequency => {
      // 频率分频显示
      timer1_led.set_state(((counter / 10) % 2 == 0).into()).ok();
      timer2_led.set_state(((counter / 20) % 2 == 0).into()).ok();
      timer3_led.set_state(((counter / 40) % 2 == 0).into()).ok();
    }
  }
}

/// 级联定时器系统
struct CascadeTimerSystem {
  timer1: Timer<stm32::TIM2>,
  timer2: Timer<stm32::TIM3>,
  timer3: Timer<stm32::TIM4>,
  mode: CascadeMode,
}

impl CascadeTimerSystem {
  fn new(
    tim2: stm32::TIM2,
    tim3: stm32::TIM3,
    tim4: stm32::TIM4,
    clocks: &stm32f4xx_hal::rcc::Clocks,
  ) -> Self {
    let mut timer1 = Timer::tim2(tim2, clocks);
    let mut timer2 = Timer::tim3(tim3, clocks);
    let mut timer3 = Timer::tim4(tim4, clocks);

    // 配置初始频率
    timer1.start(1.hz());
    timer2.start(2.hz());
    timer3.start(4.hz());

    // 启用中断
    timer1.listen(Event::TimeOut);
    timer2.listen(Event::TimeOut);
    timer3.listen(Event::TimeOut);

    Self {
      timer1,
      timer2,
      timer3,
      mode: CascadeMode::Sequential,
    }
  }

  fn start(&mut self) {
    // 启用NVIC中断
    unsafe {
      NVIC::unmask(stm32::Interrupt::TIM2);
      NVIC::unmask(stm32::Interrupt::TIM3);
      NVIC::unmask(stm32::Interrupt::TIM4);
    }
  }

  fn set_mode(&mut self, mode: CascadeMode) {
    self.mode = mode;

    match mode {
      CascadeMode::Sequential => {
        // 顺序模式：不同频率
        self.timer1.start(1.hz());
        self.timer2.start(2.hz());
        self.timer3.start(4.hz());
      }
      CascadeMode::Parallel => {
        // 并行模式：相同频率
        self.timer1.start(2.hz());
        self.timer2.start(2.hz());
        self.timer3.start(2.hz());
      }
      CascadeMode::Synchronized => {
        // 同步模式：级联触发
        self.timer1.start(1.hz());
        self.timer2.start(500.millis());
        self.timer3.start(250.millis());
      }
      CascadeMode::Frequency => {
        // 频率分频模式
        self.timer1.start(10.hz());
        self.timer2.start(5.hz());
        self.timer3.start(2.hz());
      }
    }
  }
}

/// 定时器事件
#[derive(Clone, Copy)]
enum TimerEvent {
  Timer1Overflow,
  Timer2Overflow,
  Timer3Overflow,
  CascadeComplete,
}

/// 级联模式
#[derive(Clone, Copy)]
enum CascadeMode {
  Sequential,   // 顺序模式
  Parallel,     // 并行模式
  Synchronized, // 同步模式
  Frequency,    // 频率分频模式
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
      return current_state; // 返回按下事件
    }

    false
  }
}

// 中断处理函数
#[interrupt]
fn TIM2() {
  if let Some(producer) = unsafe { TIMER_PRODUCER.as_mut() } {
    producer.enqueue(TimerEvent::Timer1Overflow).ok();
  }

  // 清除中断标志
  unsafe {
    let tim2 = &(*stm32::TIM2::ptr());
    tim2.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM3() {
  if let Some(producer) = unsafe { TIMER_PRODUCER.as_mut() } {
    producer.enqueue(TimerEvent::Timer2Overflow).ok();
  }

  // 清除中断标志
  unsafe {
    let tim3 = &(*stm32::TIM3::ptr());
    tim3.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM4() {
  if let Some(producer) = unsafe { TIMER_PRODUCER.as_mut() } {
    producer.enqueue(TimerEvent::Timer3Overflow).ok();
  }

  // 检查级联完成条件
  let cascade_count = CASCADE_COUNTER.load(Ordering::Relaxed);
  if cascade_count % 10 == 0 {
    if let Some(producer) = unsafe { TIMER_PRODUCER.as_mut() } {
      producer.enqueue(TimerEvent::CascadeComplete).ok();
    }
  }

  // 清除中断标志
  unsafe {
    let tim4 = &(*stm32::TIM4::ptr());
    tim4.sr.modify(|_, w| w.uif().clear_bit());
  }
}
