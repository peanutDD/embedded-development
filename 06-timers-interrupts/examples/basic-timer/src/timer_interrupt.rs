#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::{interrupt::Mutex, peripheral::NVIC};
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{
    gpiob::{PB0, PB1, PB2, PB3},
    Output, PushPull,
  },
  prelude::*,
  stm32::{self, interrupt, TIM2, TIM3, TIM4},
  timer::{Event, Timer},
};

// 全局变量，用于在中断中访问
static G_TIM2: Mutex<RefCell<Option<Timer<TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM3: Mutex<RefCell<Option<Timer<TIM3>>>> = Mutex::new(RefCell::new(None));
static G_TIM4: Mutex<RefCell<Option<Timer<TIM4>>>> = Mutex::new(RefCell::new(None));

static G_LED1: Mutex<RefCell<Option<PB0<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static G_LED2: Mutex<RefCell<Option<PB1<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static G_LED3: Mutex<RefCell<Option<PB2<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static G_STATUS_LED: Mutex<RefCell<Option<PB3<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

// 事件队列
static mut EVENT_QUEUE: Queue<TimerEvent, 32> = Queue::new();
static G_EVENT_PRODUCER: Mutex<RefCell<Option<Producer<TimerEvent, 32>>>> =
  Mutex::new(RefCell::new(None));

// 统计计数器
static G_COUNTERS: Mutex<RefCell<InterruptCounters>> =
  Mutex::new(RefCell::new(InterruptCounters::new()));

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = stm32::Peripherals::take().unwrap();
  let mut cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpiob = dp.GPIOB.split();
  let led1 = gpiob.pb0.into_push_pull_output();
  let led2 = gpiob.pb1.into_push_pull_output();
  let led3 = gpiob.pb2.into_push_pull_output();
  let status_led = gpiob.pb3.into_push_pull_output();

  // 配置定时器
  let mut timer2 = Timer::tim2(dp.TIM2, &clocks);
  let mut timer3 = Timer::tim3(dp.TIM3, &clocks);
  let mut timer4 = Timer::tim4(dp.TIM4, &clocks);

  // 设置定时器频率
  timer2.start(1.hz()); // 1Hz - 1秒中断一次
  timer3.start(2.hz()); // 2Hz - 0.5秒中断一次
  timer4.start(4.hz()); // 4Hz - 0.25秒中断一次

  // 启用定时器中断
  timer2.listen(Event::TimeOut);
  timer3.listen(Event::TimeOut);
  timer4.listen(Event::TimeOut);

  // 初始化事件队列
  let (producer, consumer) = unsafe { EVENT_QUEUE.split() };

  // 将外设移动到全局变量中
  cortex_m::interrupt::free(|cs| {
    G_TIM2.borrow(cs).replace(Some(timer2));
    G_TIM3.borrow(cs).replace(Some(timer3));
    G_TIM4.borrow(cs).replace(Some(timer4));
    G_LED1.borrow(cs).replace(Some(led1));
    G_LED2.borrow(cs).replace(Some(led2));
    G_LED3.borrow(cs).replace(Some(led3));
    G_STATUS_LED.borrow(cs).replace(Some(status_led));
    G_EVENT_PRODUCER.borrow(cs).replace(Some(producer));
  });

  // 启用中断
  unsafe {
    NVIC::unmask(interrupt::TIM2);
    NVIC::unmask(interrupt::TIM3);
    NVIC::unmask(interrupt::TIM4);
  }

  // 创建延时定时器用于主循环
  let mut delay_timer = Timer::tim5(dp.TIM5, &clocks);
  delay_timer.start(100.hz()); // 10ms

  let mut main_loop_counter = 0u32;
  let mut last_status_toggle = 0u32;

  loop {
    // 处理事件队列中的事件
    while let Some(event) = consumer.dequeue() {
      handle_timer_event(event);
    }

    // 主循环状态LED闪烁（表示系统运行）
    if main_loop_counter - last_status_toggle >= 50 {
      // 500ms
      cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = G_STATUS_LED.borrow(cs).borrow_mut().as_mut() {
          led.toggle();
        }
      });
      last_status_toggle = main_loop_counter;
    }

    // 每秒打印统计信息
    if main_loop_counter % 100 == 0 {
      print_statistics();
    }

    // 等待下一个主循环周期
    nb::block!(delay_timer.wait()).ok();
    main_loop_counter += 1;
  }
}

/// 处理定时器事件
fn handle_timer_event(event: TimerEvent) {
  match event.timer_id {
    2 => {
      // TIM2事件：切换LED1状态
      cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = G_LED1.borrow(cs).borrow_mut().as_mut() {
          led.toggle();
        }
      });
    }
    3 => {
      // TIM3事件：切换LED2状态
      cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = G_LED2.borrow(cs).borrow_mut().as_mut() {
          led.toggle();
        }
      });
    }
    4 => {
      // TIM4事件：切换LED3状态
      cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = G_LED3.borrow(cs).borrow_mut().as_mut() {
          led.toggle();
        }
      });
    }
    _ => {}
  }
}

/// 打印统计信息
fn print_statistics() {
  cortex_m::interrupt::free(|cs| {
    let counters = G_COUNTERS.borrow(cs).borrow();
    // 在实际应用中，这里可以通过RTT或UART输出统计信息
    // 这里只是示例，实际输出需要配置相应的输出接口
  });
}

/// 定时器事件结构
#[derive(Clone, Copy)]
struct TimerEvent {
  timer_id: u8,
  timestamp: u32,
}

/// 中断统计计数器
#[derive(Clone, Copy)]
struct InterruptCounters {
  tim2_count: u32,
  tim3_count: u32,
  tim4_count: u32,
  total_interrupts: u32,
}

impl InterruptCounters {
  const fn new() -> Self {
    Self {
      tim2_count: 0,
      tim3_count: 0,
      tim4_count: 0,
      total_interrupts: 0,
    }
  }

  fn increment_tim2(&mut self) {
    self.tim2_count += 1;
    self.total_interrupts += 1;
  }

  fn increment_tim3(&mut self) {
    self.tim3_count += 1;
    self.total_interrupts += 1;
  }

  fn increment_tim4(&mut self) {
    self.tim4_count += 1;
    self.total_interrupts += 1;
  }
}

/// TIM2中断处理程序
#[interrupt]
fn TIM2() {
  cortex_m::interrupt::free(|cs| {
    if let Some(ref mut timer) = G_TIM2.borrow(cs).borrow_mut().as_mut() {
      timer.clear_interrupt(Event::TimeOut);
    }

    // 更新统计计数器
    G_COUNTERS.borrow(cs).borrow_mut().increment_tim2();

    // 发送事件到队列
    if let Some(ref mut producer) = G_EVENT_PRODUCER.borrow(cs).borrow_mut().as_mut() {
      let event = TimerEvent {
        timer_id: 2,
        timestamp: cortex_m::peripheral::DWT::cycle_count(),
      };
      producer.enqueue(event).ok();
    }
  });
}

/// TIM3中断处理程序
#[interrupt]
fn TIM3() {
  cortex_m::interrupt::free(|cs| {
    if let Some(ref mut timer) = G_TIM3.borrow(cs).borrow_mut().as_mut() {
      timer.clear_interrupt(Event::TimeOut);
    }

    // 更新统计计数器
    G_COUNTERS.borrow(cs).borrow_mut().increment_tim3();

    // 发送事件到队列
    if let Some(ref mut producer) = G_EVENT_PRODUCER.borrow(cs).borrow_mut().as_mut() {
      let event = TimerEvent {
        timer_id: 3,
        timestamp: cortex_m::peripheral::DWT::cycle_count(),
      };
      producer.enqueue(event).ok();
    }
  });
}

/// TIM4中断处理程序
#[interrupt]
fn TIM4() {
  cortex_m::interrupt::free(|cs| {
    if let Some(ref mut timer) = G_TIM4.borrow(cs).borrow_mut().as_mut() {
      timer.clear_interrupt(Event::TimeOut);
    }

    // 更新统计计数器
    G_COUNTERS.borrow(cs).borrow_mut().increment_tim4();

    // 发送事件到队列
    if let Some(ref mut producer) = G_EVENT_PRODUCER.borrow(cs).borrow_mut().as_mut() {
      let event = TimerEvent {
        timer_id: 4,
        timestamp: cortex_m::peripheral::DWT::cycle_count(),
      };
      producer.enqueue(event).ok();
    }
  });
}
