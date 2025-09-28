#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{gpioa::*, gpiob::*, gpioc::*, Edge, Input, Output, PullUp, PushPull},
  interrupt,
  prelude::*,
  stm32,
  syscfg::SysCfg,
};

type ButtonPin = PC13<Input<PullUp>>;
type LedPin = PB0<Output<PushPull>>;

// 全局变量
static BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static INTERRUPT_MANAGER: Mutex<RefCell<Option<ExternalInterruptManager>>> =
  Mutex::new(RefCell::new(None));
static mut EVENT_QUEUE: Queue<InterruptEvent, 16> = Queue::new();
static mut EVENT_PRODUCER: Option<Producer<InterruptEvent, 16>> = None;
static mut EVENT_CONSUMER: Option<Consumer<InterruptEvent, 16>> = None;

#[entry]
fn main() -> ! {
  // 获取外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  let mut button = gpioc.pc13.into_pull_up_input();
  let led = gpiob.pb0.into_push_pull_output();

  // 配置额外的按钮和LED
  let button2 = gpioa.pa0.into_pull_up_input();
  let button3 = gpioa.pa1.into_pull_up_input();
  let led2 = gpiob.pb1.into_push_pull_output();
  let led3 = gpiob.pb2.into_push_pull_output();

  // 配置系统配置控制器
  let mut syscfg = SysCfg::new(dp.SYSCFG, &mut rcc.apb2);

  // 配置外部中断
  button.make_interrupt_source(&mut syscfg);
  button.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
  button.enable_interrupt(&mut dp.EXTI);

  // 初始化事件队列
  let (producer, consumer) = unsafe { EVENT_QUEUE.split() };
  unsafe {
    EVENT_PRODUCER = Some(producer);
    EVENT_CONSUMER = Some(consumer);
  }

  // 创建中断管理器
  let interrupt_manager = ExternalInterruptManager::new();

  // 将对象移动到全局变量
  cortex_m::interrupt::free(|cs| {
    BUTTON.borrow(cs).replace(Some(button));
    LED.borrow(cs).replace(Some(led));
    INTERRUPT_MANAGER
      .borrow(cs)
      .replace(Some(interrupt_manager));
  });

  // 启用中断
  unsafe {
    NVIC::unmask(stm32::Interrupt::EXTI15_10);
  }

  // 配置系统定时器用于延时
  let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);

  // 创建多按钮管理器
  let mut multi_button_manager = MultiButtonManager::new();
  multi_button_manager.add_button(0, ButtonConfig::new(50, 1000, 200)); // 按钮0: 50ms防抖, 1s长按, 200ms重复
  multi_button_manager.add_button(1, ButtonConfig::new(30, 800, 150)); // 按钮1
  multi_button_manager.add_button(2, ButtonConfig::new(40, 1200, 250)); // 按钮2

  // 主循环
  loop {
    // 处理事件队列
    if let Some(ref mut consumer) = unsafe { EVENT_CONSUMER.as_mut() } {
      while let Some(event) = consumer.dequeue() {
        match event.event_type {
          InterruptEventType::ButtonPress => {
            // 处理按钮按下事件
            cortex_m::interrupt::free(|cs| {
              if let Some(ref mut led) = LED.borrow(cs).borrow_mut().as_mut() {
                led.toggle();
              }
            });
          }
          InterruptEventType::ButtonRelease => {
            // 处理按钮释放事件
          }
          InterruptEventType::ButtonLongPress => {
            // 处理长按事件
            cortex_m::interrupt::free(|cs| {
              if let Some(ref mut manager) = INTERRUPT_MANAGER.borrow(cs).borrow_mut().as_mut() {
                manager.reset_statistics();
              }
            });
          }
          InterruptEventType::ButtonDoubleClick => {
            // 处理双击事件
          }
        }
      }
    }

    // 更新多按钮管理器
    multi_button_manager.update();

    // 定期输出统计信息
    delay.delay_ms(100u32);

    cortex_m::interrupt::free(|cs| {
      if let Some(ref manager) = INTERRUPT_MANAGER.borrow(cs).borrow().as_ref() {
        let stats = manager.get_statistics();
        // 这里可以通过串口输出统计信息
      }
    });
  }
}

/// 中断事件类型
#[derive(Clone, Copy, PartialEq)]
pub enum InterruptEventType {
  ButtonPress,
  ButtonRelease,
  ButtonLongPress,
  ButtonDoubleClick,
}

/// 中断事件
#[derive(Clone, Copy)]
pub struct InterruptEvent {
  pub event_type: InterruptEventType,
  pub source_id: u8,
  pub timestamp: u32,
}

impl InterruptEvent {
  pub fn new(event_type: InterruptEventType, source_id: u8, timestamp: u32) -> Self {
    Self {
      event_type,
      source_id,
      timestamp,
    }
  }
}

/// 外部中断统计信息
#[derive(Clone, Copy)]
pub struct InterruptStatistics {
  pub total_interrupts: u32,
  pub button_presses: u32,
  pub button_releases: u32,
  pub long_presses: u32,
  pub double_clicks: u32,
  pub last_interrupt_time: u32,
  pub max_interrupt_interval: u32,
  pub min_interrupt_interval: u32,
}

impl InterruptStatistics {
  pub fn new() -> Self {
    Self {
      total_interrupts: 0,
      button_presses: 0,
      button_releases: 0,
      long_presses: 0,
      double_clicks: 0,
      last_interrupt_time: 0,
      max_interrupt_interval: 0,
      min_interrupt_interval: u32::MAX,
    }
  }
}

/// 外部中断管理器
pub struct ExternalInterruptManager {
  statistics: InterruptStatistics,
  debounce_time: u32,
  last_button_time: u32,
  button_state: bool,
  long_press_time: u32,
  double_click_time: u32,
  click_count: u8,
  last_click_time: u32,
}

impl ExternalInterruptManager {
  /// 创建新的外部中断管理器
  pub fn new() -> Self {
    Self {
      statistics: InterruptStatistics::new(),
      debounce_time: 50, // 50ms防抖时间
      last_button_time: 0,
      button_state: false,
      long_press_time: 1000,  // 1s长按时间
      double_click_time: 300, // 300ms双击时间
      click_count: 0,
      last_click_time: 0,
    }
  }

  /// 处理按钮中断
  pub fn handle_button_interrupt(&mut self, current_time: u32) {
    self.statistics.total_interrupts += 1;

    // 防抖处理
    if current_time.wrapping_sub(self.last_button_time) < self.debounce_time {
      return;
    }

    // 更新中断间隔统计
    if self.statistics.last_interrupt_time > 0 {
      let interval = current_time.wrapping_sub(self.statistics.last_interrupt_time);
      if interval > self.statistics.max_interrupt_interval {
        self.statistics.max_interrupt_interval = interval;
      }
      if interval < self.statistics.min_interrupt_interval {
        self.statistics.min_interrupt_interval = interval;
      }
    }

    self.statistics.last_interrupt_time = current_time;
    self.last_button_time = current_time;

    // 检测按钮状态变化
    let new_state = self.read_button_state();

    if new_state != self.button_state {
      self.button_state = new_state;

      if new_state {
        // 按钮按下
        self.statistics.button_presses += 1;
        self.handle_button_press(current_time);
      } else {
        // 按钮释放
        self.statistics.button_releases += 1;
        self.handle_button_release(current_time);
      }
    }
  }

  /// 处理按钮按下
  fn handle_button_press(&mut self, current_time: u32) {
    // 发送按钮按下事件
    self.send_event(InterruptEventType::ButtonPress, 0, current_time);

    // 检测双击
    if current_time.wrapping_sub(self.last_click_time) < self.double_click_time {
      self.click_count += 1;
      if self.click_count >= 2 {
        self.statistics.double_clicks += 1;
        self.send_event(InterruptEventType::ButtonDoubleClick, 0, current_time);
        self.click_count = 0;
      }
    } else {
      self.click_count = 1;
    }

    self.last_click_time = current_time;
  }

  /// 处理按钮释放
  fn handle_button_release(&mut self, current_time: u32) {
    // 检测长按
    let press_duration = current_time.wrapping_sub(self.last_button_time);
    if press_duration >= self.long_press_time {
      self.statistics.long_presses += 1;
      self.send_event(InterruptEventType::ButtonLongPress, 0, current_time);
    }

    // 发送按钮释放事件
    self.send_event(InterruptEventType::ButtonRelease, 0, current_time);
  }

  /// 读取按钮状态 (需要根据实际硬件实现)
  fn read_button_state(&self) -> bool {
    // 这里应该读取实际的按钮状态
    // 由于在中断中，我们假设按钮被按下
    true
  }

  /// 发送事件到队列
  fn send_event(&self, event_type: InterruptEventType, source_id: u8, timestamp: u32) {
    let event = InterruptEvent::new(event_type, source_id, timestamp);

    unsafe {
      if let Some(ref mut producer) = EVENT_PRODUCER.as_mut() {
        producer.enqueue(event).ok();
      }
    }
  }

  /// 获取统计信息
  pub fn get_statistics(&self) -> InterruptStatistics {
    self.statistics
  }

  /// 重置统计信息
  pub fn reset_statistics(&mut self) {
    self.statistics = InterruptStatistics::new();
  }

  /// 设置防抖时间
  pub fn set_debounce_time(&mut self, time_ms: u32) {
    self.debounce_time = time_ms;
  }

  /// 设置长按时间
  pub fn set_long_press_time(&mut self, time_ms: u32) {
    self.long_press_time = time_ms;
  }
}

/// 按钮配置
#[derive(Clone, Copy)]
pub struct ButtonConfig {
  pub debounce_time: u32,
  pub long_press_time: u32,
  pub repeat_time: u32,
}

impl ButtonConfig {
  pub fn new(debounce_time: u32, long_press_time: u32, repeat_time: u32) -> Self {
    Self {
      debounce_time,
      long_press_time,
      repeat_time,
    }
  }
}

/// 按钮状态
#[derive(Clone, Copy, PartialEq)]
pub enum ButtonState {
  Released,
  Pressed,
  LongPressed,
  Repeating,
}

/// 按钮信息
pub struct ButtonInfo {
  config: ButtonConfig,
  state: ButtonState,
  last_change_time: u32,
  press_start_time: u32,
  last_repeat_time: u32,
  press_count: u32,
}

impl ButtonInfo {
  pub fn new(config: ButtonConfig) -> Self {
    Self {
      config,
      state: ButtonState::Released,
      last_change_time: 0,
      press_start_time: 0,
      last_repeat_time: 0,
      press_count: 0,
    }
  }
}

/// 多按钮管理器
pub struct MultiButtonManager {
  buttons: [Option<ButtonInfo>; 8],
  button_count: usize,
}

impl MultiButtonManager {
  /// 创建多按钮管理器
  pub fn new() -> Self {
    Self {
      buttons: [None, None, None, None, None, None, None, None],
      button_count: 0,
    }
  }

  /// 添加按钮
  pub fn add_button(&mut self, id: usize, config: ButtonConfig) -> bool {
    if id < 8 && self.buttons[id].is_none() {
      self.buttons[id] = Some(ButtonInfo::new(config));
      self.button_count += 1;
      true
    } else {
      false
    }
  }

  /// 更新按钮状态
  pub fn update(&mut self) {
    let current_time = self.get_current_time();

    for (id, button_info) in self.buttons.iter_mut().enumerate() {
      if let Some(ref mut info) = button_info {
        let button_pressed = self.read_button_state(id);
        self.update_button_state(info, button_pressed, current_time);
      }
    }
  }

  /// 更新单个按钮状态
  fn update_button_state(&mut self, info: &mut ButtonInfo, pressed: bool, current_time: u32) {
    match info.state {
      ButtonState::Released => {
        if pressed {
          info.state = ButtonState::Pressed;
          info.press_start_time = current_time;
          info.last_change_time = current_time;
          info.press_count += 1;
        }
      }
      ButtonState::Pressed => {
        if !pressed {
          // 检查是否满足防抖时间
          if current_time.wrapping_sub(info.last_change_time) >= info.config.debounce_time {
            info.state = ButtonState::Released;
            info.last_change_time = current_time;
          }
        } else {
          // 检查是否达到长按时间
          if current_time.wrapping_sub(info.press_start_time) >= info.config.long_press_time {
            info.state = ButtonState::LongPressed;
            info.last_repeat_time = current_time;
          }
        }
      }
      ButtonState::LongPressed => {
        if !pressed {
          info.state = ButtonState::Released;
          info.last_change_time = current_time;
        } else {
          // 检查是否开始重复
          if current_time.wrapping_sub(info.last_repeat_time) >= info.config.repeat_time {
            info.state = ButtonState::Repeating;
            info.last_repeat_time = current_time;
          }
        }
      }
      ButtonState::Repeating => {
        if !pressed {
          info.state = ButtonState::Released;
          info.last_change_time = current_time;
        } else {
          // 继续重复
          if current_time.wrapping_sub(info.last_repeat_time) >= info.config.repeat_time {
            info.last_repeat_time = current_time;
            // 触发重复事件
          }
        }
      }
    }
  }

  /// 读取按钮状态 (需要根据实际硬件实现)
  fn read_button_state(&self, button_id: usize) -> bool {
    // 这里应该读取实际的按钮状态
    false
  }

  /// 获取当前时间 (需要根据实际系统实现)
  fn get_current_time(&self) -> u32 {
    // 这里应该返回系统时间戳
    0
  }

  /// 获取按钮状态
  pub fn get_button_state(&self, button_id: usize) -> Option<ButtonState> {
    if button_id < 8 {
      self.buttons[button_id].as_ref().map(|info| info.state)
    } else {
      None
    }
  }

  /// 获取按钮按压次数
  pub fn get_press_count(&self, button_id: usize) -> u32 {
    if button_id < 8 {
      self.buttons[button_id]
        .as_ref()
        .map_or(0, |info| info.press_count)
    } else {
      0
    }
  }
}

/// EXTI15_10中断处理程序
#[interrupt]
fn EXTI15_10() {
  cortex_m::interrupt::free(|cs| {
    if let Some(ref mut manager) = INTERRUPT_MANAGER.borrow(cs).borrow_mut().as_mut() {
      // 获取当前时间 (这里需要实际的时间戳)
      let current_time = 0; // 应该从系统定时器获取

      manager.handle_button_interrupt(current_time);
    }

    // 清除中断标志
    if let Some(ref mut button) = BUTTON.borrow(cs).borrow_mut().as_mut() {
      button.clear_interrupt_pending_bit();
    }
  });
}
