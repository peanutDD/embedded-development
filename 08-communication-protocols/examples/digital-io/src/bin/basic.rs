#![no_std]
#![no_main]

//! # 基础数字IO操作示例
//!
//! 演示基本的数字输入输出操作：
//! - GPIO配置和控制
//! - 按键输入处理
//! - LED输出控制
//! - 中断处理

use core::cell::RefCell;
use cortex_m_rt::entry;
use critical_section::Mutex;
use heapless::Vec;
use panic_probe as _;
use stm32f4xx_hal::{
  gpio::{
    gpioa::{PA0, PA1, PA2, PA3},
    gpiob::{PB0, PB1, PB2, PB3},
    gpioc::{PC13, PC14, PC15},
    Edge, Input, Output, PullUp, PushPull,
  },
  pac::{self, interrupt, EXTI},
  prelude::*,
  syscfg::SysCfg,
};

// 全局状态
static BUTTON_STATE: Mutex<RefCell<ButtonState>> = Mutex::new(RefCell::new(ButtonState::new()));
static LED_CONTROLLER: Mutex<RefCell<Option<LedController>>> = Mutex::new(RefCell::new(None));

/// 按键状态
#[derive(Debug, Clone, Copy)]
pub struct ButtonState {
  pub button1_pressed: bool,
  pub button2_pressed: bool,
  pub button3_pressed: bool,
  pub button4_pressed: bool,
  pub press_count: u32,
  pub last_press_time: u32,
}

impl ButtonState {
  pub const fn new() -> Self {
    Self {
      button1_pressed: false,
      button2_pressed: false,
      button3_pressed: false,
      button4_pressed: false,
      press_count: 0,
      last_press_time: 0,
    }
  }

  pub fn update_button(&mut self, button: u8, pressed: bool) {
    match button {
      1 => self.button1_pressed = pressed,
      2 => self.button2_pressed = pressed,
      3 => self.button3_pressed = pressed,
      4 => self.button4_pressed = pressed,
      _ => {}
    }

    if pressed {
      self.press_count += 1;
      self.last_press_time = get_system_time();
    }
  }

  pub fn any_pressed(&self) -> bool {
    self.button1_pressed || self.button2_pressed || self.button3_pressed || self.button4_pressed
  }

  pub fn get_pressed_buttons(&self) -> Vec<u8, 4> {
    let mut pressed = Vec::new();

    if self.button1_pressed {
      pressed.push(1).ok();
    }
    if self.button2_pressed {
      pressed.push(2).ok();
    }
    if self.button3_pressed {
      pressed.push(3).ok();
    }
    if self.button4_pressed {
      pressed.push(4).ok();
    }

    pressed
  }
}

/// LED控制器
pub struct LedController {
  pub led1: PC13<Output<PushPull>>,
  pub led2: PC14<Output<PushPull>>,
  pub led3: PC15<Output<PushPull>>,
  pub status_led: PB0<Output<PushPull>>,
  pub pattern: LedPattern,
  pub brightness: u8,
  pub blink_counter: u32,
}

/// LED模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LedPattern {
  Off,
  On,
  Blink,
  FastBlink,
  Breathing,
  Chase,
  Random,
}

impl LedController {
  pub fn new(
    led1: PC13<Output<PushPull>>,
    led2: PC14<Output<PushPull>>,
    led3: PC15<Output<PushPull>>,
    status_led: PB0<Output<PushPull>>,
  ) -> Self {
    Self {
      led1,
      led2,
      led3,
      status_led,
      pattern: LedPattern::Off,
      brightness: 255,
      blink_counter: 0,
    }
  }

  pub fn set_pattern(&mut self, pattern: LedPattern) {
    self.pattern = pattern;
    self.blink_counter = 0;
  }

  pub fn set_brightness(&mut self, brightness: u8) {
    self.brightness = brightness;
  }

  pub fn update(&mut self) {
    self.blink_counter += 1;

    match self.pattern {
      LedPattern::Off => {
        self.led1.set_high();
        self.led2.set_high();
        self.led3.set_high();
      }
      LedPattern::On => {
        self.led1.set_low();
        self.led2.set_low();
        self.led3.set_low();
      }
      LedPattern::Blink => {
        if self.blink_counter % 1000 < 500 {
          self.led1.set_low();
          self.led2.set_low();
          self.led3.set_low();
        } else {
          self.led1.set_high();
          self.led2.set_high();
          self.led3.set_high();
        }
      }
      LedPattern::FastBlink => {
        if self.blink_counter % 200 < 100 {
          self.led1.set_low();
          self.led2.set_low();
          self.led3.set_low();
        } else {
          self.led1.set_high();
          self.led2.set_high();
          self.led3.set_high();
        }
      }
      LedPattern::Breathing => {
        let phase = (self.blink_counter % 2000) as f32 / 2000.0;
        let intensity = ((phase * 2.0 * core::f32::consts::PI).sin() + 1.0) / 2.0;

        if intensity > 0.5 {
          self.led1.set_low();
        } else {
          self.led1.set_high();
        }

        if intensity > 0.3 {
          self.led2.set_low();
        } else {
          self.led2.set_high();
        }

        if intensity > 0.7 {
          self.led3.set_low();
        } else {
          self.led3.set_high();
        }
      }
      LedPattern::Chase => {
        let phase = self.blink_counter % 1500;

        self.led1.set_high();
        self.led2.set_high();
        self.led3.set_high();

        if phase < 500 {
          self.led1.set_low();
        } else if phase < 1000 {
          self.led2.set_low();
        } else {
          self.led3.set_low();
        }
      }
      LedPattern::Random => {
        if self.blink_counter % 300 == 0 {
          let random = get_pseudo_random();

          if random & 0x01 != 0 {
            self.led1.set_low();
          } else {
            self.led1.set_high();
          }
          if random & 0x02 != 0 {
            self.led2.set_low();
          } else {
            self.led2.set_high();
          }
          if random & 0x04 != 0 {
            self.led3.set_low();
          } else {
            self.led3.set_high();
          }
        }
      }
    }

    // 状态LED显示系统状态
    if self.blink_counter % 2000 < 100 {
      self.status_led.set_low();
    } else {
      self.status_led.set_high();
    }
  }

  pub fn set_individual_led(&mut self, led: u8, state: bool) {
    match led {
      1 => {
        if state {
          self.led1.set_low();
        } else {
          self.led1.set_high();
        }
      }
      2 => {
        if state {
          self.led2.set_low();
        } else {
          self.led2.set_high();
        }
      }
      3 => {
        if state {
          self.led3.set_low();
        } else {
          self.led3.set_high();
        }
      }
      _ => {}
    }
  }

  pub fn display_binary(&mut self, value: u8) {
    self.set_individual_led(1, value & 0x01 != 0);
    self.set_individual_led(2, value & 0x02 != 0);
    self.set_individual_led(3, value & 0x04 != 0);
  }
}

/// 数字输入处理器
pub struct DigitalInputProcessor {
  pub debounce_time: u32,
  pub last_state: [bool; 4],
  pub stable_state: [bool; 4],
  pub last_change_time: [u32; 4],
  pub press_duration: [u32; 4],
}

impl DigitalInputProcessor {
  pub fn new() -> Self {
    Self {
      debounce_time: 50, // 50ms debounce
      last_state: [false; 4],
      stable_state: [false; 4],
      last_change_time: [0; 4],
      press_duration: [0; 4],
    }
  }

  pub fn process_input(&mut self, button: usize, current_state: bool) -> Option<ButtonEvent> {
    let current_time = get_system_time();

    // 防抖处理
    if current_state != self.last_state[button] {
      self.last_change_time[button] = current_time;
      self.last_state[button] = current_state;
      return None;
    }

    // 检查是否已经稳定
    if current_time - self.last_change_time[button] < self.debounce_time {
      return None;
    }

    // 状态改变
    if current_state != self.stable_state[button] {
      self.stable_state[button] = current_state;

      if current_state {
        // 按键按下
        self.press_duration[button] = current_time;
        return Some(ButtonEvent::Pressed(button as u8));
      } else {
        // 按键释放
        let duration = current_time - self.press_duration[button];
        return Some(ButtonEvent::Released(button as u8, duration));
      }
    }

    None
  }

  pub fn is_long_press(&self, button: usize, threshold: u32) -> bool {
    if !self.stable_state[button] {
      return false;
    }

    let current_time = get_system_time();
    current_time - self.press_duration[button] > threshold
  }
}

/// 按键事件
#[derive(Debug, Clone, Copy)]
pub enum ButtonEvent {
  Pressed(u8),
  Released(u8, u32), // button, duration
  LongPress(u8),
  DoubleClick(u8),
}

/// GPIO管理器
pub struct GpioManager {
  pub input_processor: DigitalInputProcessor,
  pub button_pins: ButtonPins,
}

pub struct ButtonPins {
  pub button1: PA0<Input<PullUp>>,
  pub button2: PA1<Input<PullUp>>,
  pub button3: PA2<Input<PullUp>>,
  pub button4: PA3<Input<PullUp>>,
}

impl GpioManager {
  pub fn new(button_pins: ButtonPins) -> Self {
    Self {
      input_processor: DigitalInputProcessor::new(),
      button_pins,
    }
  }

  pub fn scan_inputs(&mut self) -> Vec<ButtonEvent, 8> {
    let mut events = Vec::new();

    // 读取按键状态
    let states = [
      self.button_pins.button1.is_low(),
      self.button_pins.button2.is_low(),
      self.button_pins.button3.is_low(),
      self.button_pins.button4.is_low(),
    ];

    // 处理每个按键
    for (i, &state) in states.iter().enumerate() {
      if let Some(event) = self.input_processor.process_input(i, state) {
        events.push(event).ok();
      }

      // 检查长按
      if self.input_processor.is_long_press(i, 2000) {
        // 2秒长按
        events.push(ButtonEvent::LongPress(i as u8)).ok();
      }
    }

    events
  }
}

#[entry]
fn main() -> ! {
  // 获取外设句柄
  let mut dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

  // 配置 GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 配置按键输入 (带上拉)
  let button1 = gpioa.pa0.into_pull_up_input();
  let button2 = gpioa.pa1.into_pull_up_input();
  let button3 = gpioa.pa2.into_pull_up_input();
  let button4 = gpioa.pa3.into_pull_up_input();

  // 配置LED输出
  let led1 = gpioc.pc13.into_push_pull_output();
  let led2 = gpioc.pc14.into_push_pull_output();
  let led3 = gpioc.pc15.into_push_pull_output();
  let status_led = gpiob.pb0.into_push_pull_output();

  // 创建控制器
  let mut led_controller = LedController::new(led1, led2, led3, status_led);
  let mut gpio_manager = GpioManager::new(ButtonPins {
    button1,
    button2,
    button3,
    button4,
  });

  // 配置中断 (可选)
  let mut syscfg = SysCfg::new(dp.SYSCFG, &mut dp.RCC);
  let mut exti = dp.EXTI;

  // 设置初始LED模式
  led_controller.set_pattern(LedPattern::Breathing);

  // 存储LED控制器到全局变量
  critical_section::with(|cs| {
    LED_CONTROLLER.borrow(cs).replace(Some(led_controller));
  });

  // 启动指示
  startup_sequence();

  let mut loop_counter = 0u32;
  let mut current_pattern = 0usize;
  let patterns = [
    LedPattern::Breathing,
    LedPattern::Chase,
    LedPattern::Blink,
    LedPattern::FastBlink,
    LedPattern::Random,
  ];

  loop {
    loop_counter += 1;

    // 扫描输入
    let events = gpio_manager.scan_inputs();

    // 处理按键事件
    for event in events {
      match event {
        ButtonEvent::Pressed(button) => {
          // 更新全局按键状态
          critical_section::with(|cs| {
            BUTTON_STATE
              .borrow(cs)
              .borrow_mut()
              .update_button(button, true);
          });

          // 根据按键切换LED模式
          match button {
            1 => {
              current_pattern = (current_pattern + 1) % patterns.len();
              critical_section::with(|cs| {
                if let Some(ref mut controller) = LED_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                  controller.set_pattern(patterns[current_pattern]);
                }
              });
            }
            2 => {
              critical_section::with(|cs| {
                if let Some(ref mut controller) = LED_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                  controller.set_pattern(LedPattern::On);
                }
              });
            }
            3 => {
              critical_section::with(|cs| {
                if let Some(ref mut controller) = LED_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                  controller.set_pattern(LedPattern::Off);
                }
              });
            }
            4 => {
              // 显示按键计数的二进制表示
              let count = critical_section::with(|cs| BUTTON_STATE.borrow(cs).borrow().press_count);

              critical_section::with(|cs| {
                if let Some(ref mut controller) = LED_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                  controller.display_binary(count as u8);
                }
              });
            }
            _ => {}
          }
        }
        ButtonEvent::Released(button, duration) => {
          critical_section::with(|cs| {
            BUTTON_STATE
              .borrow(cs)
              .borrow_mut()
              .update_button(button, false);
          });

          // 短按和长按的不同处理
          if duration > 2000 {
            // 长按：重置系统
            critical_section::with(|cs| {
              *BUTTON_STATE.borrow(cs).borrow_mut() = ButtonState::new();
              if let Some(ref mut controller) = LED_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                controller.set_pattern(LedPattern::Breathing);
              }
            });
            current_pattern = 0;
          }
        }
        ButtonEvent::LongPress(button) => {
          // 长按处理
          match button {
            1 => {
              // 长按按键1：进入配置模式
              critical_section::with(|cs| {
                if let Some(ref mut controller) = LED_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                  controller.set_pattern(LedPattern::FastBlink);
                }
              });
            }
            _ => {}
          }
        }
        _ => {}
      }
    }

    // 更新LED
    critical_section::with(|cs| {
      if let Some(ref mut controller) = LED_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
        controller.update();
      }
    });

    // 系统状态监控
    if loop_counter % 10000 == 0 {
      let button_state = critical_section::with(|cs| *BUTTON_STATE.borrow(cs).borrow());

      // 可以在这里添加调试输出或状态检查
      if button_state.any_pressed() {
        // 有按键被按下时的特殊处理
      }
    }

    // 简单延时
    delay_us(100);
  }
}

fn startup_sequence() {
  // 启动时的LED序列
  critical_section::with(|cs| {
    if let Some(ref mut controller) = LED_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
      // 依次点亮LED
      controller.set_individual_led(1, true);
      delay_ms(200);
      controller.set_individual_led(2, true);
      delay_ms(200);
      controller.set_individual_led(3, true);
      delay_ms(200);

      // 全部熄灭
      controller.set_individual_led(1, false);
      controller.set_individual_led(2, false);
      controller.set_individual_led(3, false);
      delay_ms(500);
    }
  });
}

// 中断处理函数 (可选)
#[interrupt]
fn EXTI0() {
  // 按键1中断处理
  critical_section::with(|cs| {
    BUTTON_STATE.borrow(cs).borrow_mut().update_button(1, true);
  });

  // 清除中断标志
  unsafe {
    (*EXTI::ptr()).pr.write(|w| w.pr0().set_bit());
  }
}

// 辅助函数
fn get_system_time() -> u32 {
  // 简单的系统时间实现
  static mut COUNTER: u32 = 0;
  unsafe {
    COUNTER += 1;
    COUNTER
  }
}

fn get_pseudo_random() -> u8 {
  static mut SEED: u32 = 1;
  unsafe {
    SEED = SEED.wrapping_mul(1103515245).wrapping_add(12345);
    (SEED >> 16) as u8
  }
}

fn delay_ms(ms: u32) {
  for _ in 0..(ms * 8400) {
    cortex_m::asm::nop();
  }
}

fn delay_us(us: u32) {
  for _ in 0..(us * 8) {
    cortex_m::asm::nop();
  }
}
