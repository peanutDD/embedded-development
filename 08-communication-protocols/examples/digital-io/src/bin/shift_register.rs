#![no_std]
#![no_main]

//! # 移位寄存器控制示例
//!
//! 演示74HC595移位寄存器的使用：
//! - 串行数据输入
//! - 并行数据输出
//! - 级联多个移位寄存器
//! - LED条形图显示
//! - 数字显示控制

use core::cell::RefCell;
use cortex_m_rt::entry;
use critical_section::Mutex;
use heapless::{String, Vec};
use panic_probe as _;
use stm32f4xx_hal::{
  gpio::{
    gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
    gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7},
    gpioc::{PC0, PC1, PC13, PC2},
    Input, Output, PullUp, PushPull,
  },
  pac,
  prelude::*,
  timer::{Event, Timer},
};

// 移位寄存器配置
const MAX_SHIFT_REGISTERS: usize = 4;
const BITS_PER_REGISTER: usize = 8;
const TOTAL_OUTPUTS: usize = MAX_SHIFT_REGISTERS * BITS_PER_REGISTER;

// 全局移位寄存器状态
static SHIFT_REGISTER_STATE: Mutex<RefCell<ShiftRegisterState>> =
  Mutex::new(RefCell::new(ShiftRegisterState::new()));

/// 移位寄存器状态
#[derive(Debug, Clone)]
pub struct ShiftRegisterState {
  pub output_data: [u8; MAX_SHIFT_REGISTERS],
  pub dirty: bool,
  pub update_counter: u32,
}

impl ShiftRegisterState {
  pub const fn new() -> Self {
    Self {
      output_data: [0; MAX_SHIFT_REGISTERS],
      dirty: true,
      update_counter: 0,
    }
  }

  pub fn set_bit(&mut self, register: usize, bit: usize, value: bool) {
    if register < MAX_SHIFT_REGISTERS && bit < BITS_PER_REGISTER {
      if value {
        self.output_data[register] |= 1 << bit;
      } else {
        self.output_data[register] &= !(1 << bit);
      }
      self.dirty = true;
    }
  }

  pub fn get_bit(&self, register: usize, bit: usize) -> bool {
    if register < MAX_SHIFT_REGISTERS && bit < BITS_PER_REGISTER {
      (self.output_data[register] >> bit) & 1 != 0
    } else {
      false
    }
  }

  pub fn set_register(&mut self, register: usize, value: u8) {
    if register < MAX_SHIFT_REGISTERS {
      self.output_data[register] = value;
      self.dirty = true;
    }
  }

  pub fn get_register(&self, register: usize) -> u8 {
    if register < MAX_SHIFT_REGISTERS {
      self.output_data[register]
    } else {
      0
    }
  }

  pub fn clear_all(&mut self) {
    self.output_data = [0; MAX_SHIFT_REGISTERS];
    self.dirty = true;
  }

  pub fn set_all(&mut self) {
    self.output_data = [0xFF; MAX_SHIFT_REGISTERS];
    self.dirty = true;
  }
}

/// 移位寄存器控制器
pub struct ShiftRegisterController {
  // 控制引脚
  pub data_pin: PA0<Output<PushPull>>,  // SER (Serial Data Input)
  pub clock_pin: PA1<Output<PushPull>>, // SRCLK (Shift Register Clock)
  pub latch_pin: PA2<Output<PushPull>>, // RCLK (Register Clock/Latch)
  pub enable_pin: PA3<Output<PushPull>>, // OE (Output Enable, active low)
  pub clear_pin: PA4<Output<PushPull>>, // SRCLR (Shift Register Clear, active low)

  // 状态
  pub num_registers: usize,
  pub clock_delay_us: u32,
}

impl ShiftRegisterController {
  pub fn new(
    data_pin: PA0<Output<PushPull>>,
    clock_pin: PA1<Output<PushPull>>,
    latch_pin: PA2<Output<PushPull>>,
    enable_pin: PA3<Output<PushPull>>,
    clear_pin: PA4<Output<PushPull>>,
    num_registers: usize,
  ) -> Self {
    let mut controller = Self {
      data_pin,
      clock_pin,
      latch_pin,
      enable_pin,
      clear_pin,
      num_registers: num_registers.min(MAX_SHIFT_REGISTERS),
      clock_delay_us: 1,
    };

    // 初始化引脚状态
    controller.init();
    controller
  }

  /// 初始化移位寄存器
  pub fn init(&mut self) {
    // 设置初始状态
    self.data_pin.set_low();
    self.clock_pin.set_low();
    self.latch_pin.set_low();
    self.enable_pin.set_low(); // 启用输出
    self.clear_pin.set_high(); // 不清除

    delay_us(10);

    // 清除所有寄存器
    self.clear_all_registers();
  }

  /// 清除所有寄存器
  pub fn clear_all_registers(&mut self) {
    self.clear_pin.set_low();
    delay_us(1);
    self.clear_pin.set_high();
    delay_us(1);

    // 锁存清除状态
    self.latch_pin.set_high();
    delay_us(1);
    self.latch_pin.set_low();
  }

  /// 移位输出数据
  pub fn shift_out(&mut self, data: &[u8]) {
    // 确保数据长度不超过寄存器数量
    let data_len = data.len().min(self.num_registers);

    // 从最高位寄存器开始输出（级联时最后一个寄存器先输出）
    for register_idx in (0..data_len).rev() {
      let register_data = data[register_idx];

      // 从最高位开始输出
      for bit_idx in (0..8).rev() {
        let bit_value = (register_data >> bit_idx) & 1 != 0;

        // 设置数据位
        if bit_value {
          self.data_pin.set_high();
        } else {
          self.data_pin.set_low();
        }

        delay_us(self.clock_delay_us);

        // 时钟脉冲
        self.clock_pin.set_high();
        delay_us(self.clock_delay_us);
        self.clock_pin.set_low();
        delay_us(self.clock_delay_us);
      }
    }

    // 锁存数据到输出寄存器
    self.latch_pin.set_high();
    delay_us(1);
    self.latch_pin.set_low();
  }

  /// 更新输出
  pub fn update(&mut self) {
    critical_section::with(|cs| {
      let mut state = SHIFT_REGISTER_STATE.borrow(cs).borrow_mut();

      if state.dirty {
        // 输出数据到移位寄存器
        let data = &state.output_data[0..self.num_registers];
        self.shift_out(data);

        state.dirty = false;
        state.update_counter += 1;
      }
    });
  }

  /// 设置时钟延时
  pub fn set_clock_delay(&mut self, delay_us: u32) {
    self.clock_delay_us = delay_us;
  }

  /// 启用/禁用输出
  pub fn set_output_enable(&mut self, enable: bool) {
    if enable {
      self.enable_pin.set_low(); // 低电平有效
    } else {
      self.enable_pin.set_high();
    }
  }
}

/// LED条形图控制器
pub struct LedBarGraph {
  pub num_leds: usize,
  pub register_start: usize,
  pub current_level: u8,
  pub max_level: u8,
  pub mode: BarGraphMode,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BarGraphMode {
  /// 条形图模式（从底部开始点亮）
  Bar,
  /// 点模式（只点亮当前位置）
  Dot,
  /// VU表模式（带峰值保持）
  VuMeter,
}

impl LedBarGraph {
  pub fn new(num_leds: usize, register_start: usize) -> Self {
    Self {
      num_leds: num_leds.min(TOTAL_OUTPUTS),
      register_start,
      current_level: 0,
      max_level: num_leds as u8,
      mode: BarGraphMode::Bar,
    }
  }

  /// 设置电平
  pub fn set_level(&mut self, level: u8) {
    self.current_level = level.min(self.max_level);
    self.update_display();
  }

  /// 设置模式
  pub fn set_mode(&mut self, mode: BarGraphMode) {
    self.mode = mode;
    self.update_display();
  }

  /// 更新显示
  fn update_display(&self) {
    critical_section::with(|cs| {
      let mut state = SHIFT_REGISTER_STATE.borrow(cs).borrow_mut();

      // 清除当前LED区域
      for i in 0..self.num_leds {
        let bit_position = self.register_start + i;
        let register = bit_position / BITS_PER_REGISTER;
        let bit = bit_position % BITS_PER_REGISTER;
        state.set_bit(register, bit, false);
      }

      // 根据模式设置LED
      match self.mode {
        BarGraphMode::Bar => {
          // 条形图模式：点亮从0到current_level的所有LED
          for i in 0..(self.current_level as usize).min(self.num_leds) {
            let bit_position = self.register_start + i;
            let register = bit_position / BITS_PER_REGISTER;
            let bit = bit_position % BITS_PER_REGISTER;
            state.set_bit(register, bit, true);
          }
        }
        BarGraphMode::Dot => {
          // 点模式：只点亮当前位置的LED
          if self.current_level > 0 && (self.current_level as usize) <= self.num_leds {
            let i = (self.current_level as usize) - 1;
            let bit_position = self.register_start + i;
            let register = bit_position / BITS_PER_REGISTER;
            let bit = bit_position % BITS_PER_REGISTER;
            state.set_bit(register, bit, true);
          }
        }
        BarGraphMode::VuMeter => {
          // VU表模式：条形图 + 峰值指示
          // 条形图部分
          let bar_level = (self.current_level * 3 / 4).min(self.max_level);
          for i in 0..(bar_level as usize).min(self.num_leds) {
            let bit_position = self.register_start + i;
            let register = bit_position / BITS_PER_REGISTER;
            let bit = bit_position % BITS_PER_REGISTER;
            state.set_bit(register, bit, true);
          }

          // 峰值指示
          if self.current_level > 0 && (self.current_level as usize) <= self.num_leds {
            let i = (self.current_level as usize) - 1;
            let bit_position = self.register_start + i;
            let register = bit_position / BITS_PER_REGISTER;
            let bit = bit_position % BITS_PER_REGISTER;
            state.set_bit(register, bit, true);
          }
        }
      }
    });
  }
}

/// 7段数码管控制器
pub struct SevenSegmentDisplay {
  pub register_start: usize,
  pub num_digits: usize,
  pub current_value: u16,
  pub leading_zeros: bool,
}

// 7段数码管字符编码 (共阴极)
const SEVEN_SEG_DIGITS: [u8; 16] = [
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b01110111, // A
  0b01111100, // b
  0b00111001, // C
  0b01011110, // d
  0b01111001, // E
  0b01110001, // F
];

impl SevenSegmentDisplay {
  pub fn new(register_start: usize, num_digits: usize) -> Self {
    Self {
      register_start,
      num_digits: num_digits.min(4), // 最多4位数字
      current_value: 0,
      leading_zeros: false,
    }
  }

  /// 设置显示值
  pub fn set_value(&mut self, value: u16) {
    self.current_value = value;
    self.update_display();
  }

  /// 设置是否显示前导零
  pub fn set_leading_zeros(&mut self, show: bool) {
    self.leading_zeros = show;
    self.update_display();
  }

  /// 更新显示
  fn update_display(&self) {
    critical_section::with(|cs| {
      let mut state = SHIFT_REGISTER_STATE.borrow(cs).borrow_mut();

      let mut value = self.current_value;
      let mut has_non_zero = false;

      // 从最低位开始处理
      for digit_idx in 0..self.num_digits {
        let digit = (value % 10) as usize;
        value /= 10;

        // 决定是否显示这一位
        let should_display = if digit != 0 {
          has_non_zero = true;
          true
        } else if has_non_zero || self.leading_zeros || digit_idx == 0 {
          true
        } else {
          false
        };

        let register_idx = self.register_start + digit_idx;
        if register_idx < MAX_SHIFT_REGISTERS {
          if should_display {
            state.set_register(register_idx, SEVEN_SEG_DIGITS[digit]);
          } else {
            state.set_register(register_idx, 0); // 空白
          }
        }
      }
    });
  }

  /// 显示十六进制值
  pub fn set_hex_value(&mut self, value: u16) {
    critical_section::with(|cs| {
      let mut state = SHIFT_REGISTER_STATE.borrow(cs).borrow_mut();

      let mut hex_value = value;

      for digit_idx in 0..self.num_digits {
        let digit = (hex_value & 0xF) as usize;
        hex_value >>= 4;

        let register_idx = self.register_start + digit_idx;
        if register_idx < MAX_SHIFT_REGISTERS {
          state.set_register(register_idx, SEVEN_SEG_DIGITS[digit]);
        }
      }
    });
  }
}

/// 应用控制器
pub struct ApplicationController {
  pub bar_graph: LedBarGraph,
  pub seven_segment: SevenSegmentDisplay,
  pub demo_mode: DemoMode,
  pub demo_counter: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DemoMode {
  BarGraph,
  SevenSegment,
  Combined,
  Pattern,
}

impl ApplicationController {
  pub fn new() -> Self {
    Self {
      bar_graph: LedBarGraph::new(8, 0),             // 使用前8位
      seven_segment: SevenSegmentDisplay::new(1, 2), // 使用寄存器1-2
      demo_mode: DemoMode::BarGraph,
      demo_counter: 0,
    }
  }

  /// 更新演示
  pub fn update_demo(&mut self) {
    self.demo_counter += 1;

    match self.demo_mode {
      DemoMode::BarGraph => {
        self.demo_bar_graph();
      }
      DemoMode::SevenSegment => {
        self.demo_seven_segment();
      }
      DemoMode::Combined => {
        self.demo_combined();
      }
      DemoMode::Pattern => {
        self.demo_pattern();
      }
    }

    // 每5秒切换模式
    if self.demo_counter % 5000 == 0 {
      self.demo_mode = match self.demo_mode {
        DemoMode::BarGraph => DemoMode::SevenSegment,
        DemoMode::SevenSegment => DemoMode::Combined,
        DemoMode::Combined => DemoMode::Pattern,
        DemoMode::Pattern => DemoMode::BarGraph,
      };
    }
  }

  fn demo_bar_graph(&mut self) {
    // 条形图演示
    let cycle = (self.demo_counter / 100) % 32;
    let level = if cycle < 16 {
      cycle as u8
    } else {
      (32 - cycle) as u8
    };

    self.bar_graph.set_level(level / 2);

    // 每2秒切换模式
    if (self.demo_counter / 200) % 3 == 0 {
      self.bar_graph.set_mode(BarGraphMode::Bar);
    } else if (self.demo_counter / 200) % 3 == 1 {
      self.bar_graph.set_mode(BarGraphMode::Dot);
    } else {
      self.bar_graph.set_mode(BarGraphMode::VuMeter);
    }
  }

  fn demo_seven_segment(&mut self) {
    // 数码管演示
    let value = (self.demo_counter / 50) % 100;
    self.seven_segment.set_value(value as u16);

    // 切换前导零显示
    self
      .seven_segment
      .set_leading_zeros((self.demo_counter / 1000) % 2 == 0);
  }

  fn demo_combined(&mut self) {
    // 组合演示
    let cycle = self.demo_counter / 50;

    // 条形图显示计数器的低位
    let bar_level = (cycle % 8) as u8;
    self.bar_graph.set_level(bar_level);
    self.bar_graph.set_mode(BarGraphMode::Bar);

    // 数码管显示计数器的值
    self.seven_segment.set_value((cycle % 100) as u16);
  }

  fn demo_pattern(&mut self) {
    // 图案演示
    critical_section::with(|cs| {
      let mut state = SHIFT_REGISTER_STATE.borrow(cs).borrow_mut();

      let pattern_type = (self.demo_counter / 1000) % 4;
      let phase = (self.demo_counter / 100) % 8;

      match pattern_type {
        0 => {
          // 流水灯
          state.clear_all();
          for reg in 0..MAX_SHIFT_REGISTERS {
            if reg == phase % MAX_SHIFT_REGISTERS {
              state.set_register(reg, 0xFF);
            }
          }
        }
        1 => {
          // 呼吸灯
          let brightness_pattern = match phase {
            0 => 0b00000001,
            1 => 0b00000011,
            2 => 0b00000111,
            3 => 0b00001111,
            4 => 0b00011111,
            5 => 0b00111111,
            6 => 0b01111111,
            7 => 0b11111111,
            _ => 0,
          };

          for reg in 0..MAX_SHIFT_REGISTERS {
            state.set_register(reg, brightness_pattern);
          }
        }
        2 => {
          // 移位图案
          let pattern = 0b10101010u8.rotate_left(phase as u32);
          for reg in 0..MAX_SHIFT_REGISTERS {
            state.set_register(reg, pattern);
          }
        }
        3 => {
          // 随机图案
          let seed = self.demo_counter;
          for reg in 0..MAX_SHIFT_REGISTERS {
            let pattern = ((seed * (reg as u32 + 1) * 1103515245 + 12345) >> 16) as u8;
            state.set_register(reg, pattern);
          }
        }
        _ => {}
      }
    });
  }
}

#[entry]
fn main() -> ! {
  // 获取外设句柄
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

  // 配置 GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 配置移位寄存器控制引脚
  let data_pin = gpioa.pa0.into_push_pull_output();
  let clock_pin = gpioa.pa1.into_push_pull_output();
  let latch_pin = gpioa.pa2.into_push_pull_output();
  let enable_pin = gpioa.pa3.into_push_pull_output();
  let clear_pin = gpioa.pa4.into_push_pull_output();

  // 配置输入按键
  let button1 = gpiob.pb0.into_pull_up_input();
  let button2 = gpiob.pb1.into_pull_up_input();
  let button3 = gpiob.pb2.into_pull_up_input();

  // 配置状态LED
  let mut status_led = gpioc.pc13.into_push_pull_output();

  // 创建移位寄存器控制器
  let mut shift_controller =
    ShiftRegisterController::new(data_pin, clock_pin, latch_pin, enable_pin, clear_pin, 4);

  // 创建应用控制器
  let mut app_controller = ApplicationController::new();

  // 配置定时器
  let mut timer = Timer::new(dp.TIM2, &clocks);
  timer.start(1000.Hz()); // 1kHz更新频率
  timer.listen(Event::Update);

  // 启动序列
  startup_sequence(&mut shift_controller);

  let mut loop_counter = 0u32;
  let mut button_states = [false; 3];
  let mut last_button_states = [false; 3];

  loop {
    loop_counter += 1;

    // 读取按键状态
    button_states[0] = button1.is_low();
    button_states[1] = button2.is_low();
    button_states[2] = button3.is_low();

    // 检测按键按下事件
    for i in 0..3 {
      if button_states[i] && !last_button_states[i] {
        handle_button_press(i, &mut app_controller);
      }
    }
    last_button_states = button_states;

    // 更新演示
    if loop_counter % 100 == 0 {
      app_controller.update_demo();
    }

    // 更新移位寄存器输出
    if loop_counter % 10 == 0 {
      shift_controller.update();
    }

    // 状态LED闪烁
    if loop_counter % 50000 == 0 {
      status_led.toggle();
    }

    // 短暂延时
    delay_us(100);
  }
}

fn startup_sequence(controller: &mut ShiftRegisterController) {
  // 启动动画序列

  // 1. 全部点亮测试
  critical_section::with(|cs| {
    SHIFT_REGISTER_STATE.borrow(cs).borrow_mut().set_all();
  });
  controller.update();
  delay_ms(500);

  // 2. 全部熄灭
  critical_section::with(|cs| {
    SHIFT_REGISTER_STATE.borrow(cs).borrow_mut().clear_all();
  });
  controller.update();
  delay_ms(200);

  // 3. 逐个点亮测试
  for reg in 0..MAX_SHIFT_REGISTERS {
    for bit in 0..BITS_PER_REGISTER {
      critical_section::with(|cs| {
        let mut state = SHIFT_REGISTER_STATE.borrow(cs).borrow_mut();
        state.clear_all();
        state.set_bit(reg, bit, true);
      });
      controller.update();
      delay_ms(50);
    }
  }

  // 4. 流水灯效果
  for _ in 0..3 {
    for reg in 0..MAX_SHIFT_REGISTERS {
      critical_section::with(|cs| {
        let mut state = SHIFT_REGISTER_STATE.borrow(cs).borrow_mut();
        state.clear_all();
        state.set_register(reg, 0xFF);
      });
      controller.update();
      delay_ms(100);
    }
  }

  // 5. 清除所有输出
  critical_section::with(|cs| {
    SHIFT_REGISTER_STATE.borrow(cs).borrow_mut().clear_all();
  });
  controller.update();
  delay_ms(200);
}

fn handle_button_press(button: usize, app: &mut ApplicationController) {
  match button {
    0 => {
      // 按键1：切换演示模式
      app.demo_mode = match app.demo_mode {
        DemoMode::BarGraph => DemoMode::SevenSegment,
        DemoMode::SevenSegment => DemoMode::Combined,
        DemoMode::Combined => DemoMode::Pattern,
        DemoMode::Pattern => DemoMode::BarGraph,
      };
      app.demo_counter = 0; // 重置计数器
    }
    1 => {
      // 按键2：切换条形图模式
      let new_mode = match app.bar_graph.mode {
        BarGraphMode::Bar => BarGraphMode::Dot,
        BarGraphMode::Dot => BarGraphMode::VuMeter,
        BarGraphMode::VuMeter => BarGraphMode::Bar,
      };
      app.bar_graph.set_mode(new_mode);
    }
    2 => {
      // 按键3：切换数码管前导零显示
      app
        .seven_segment
        .set_leading_zeros(!app.seven_segment.leading_zeros);
    }
    _ => {}
  }
}

// 辅助函数
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
