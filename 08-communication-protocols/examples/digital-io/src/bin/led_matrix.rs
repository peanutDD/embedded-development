#![no_std]
#![no_main]

//! # LED矩阵显示示例
//!
//! 演示8x8 LED矩阵的控制：
//! - 行列扫描显示
//! - 图案和动画
//! - 文字滚动
//! - 亮度控制

use core::cell::RefCell;
use cortex_m_rt::entry;
use critical_section::Mutex;
use heapless::{String, Vec};
use panic_probe as _;
use stm32f4xx_hal::{
  gpio::{
    gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
    gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7},
    gpioc::PC13,
    Input, Output, PullUp, PushPull,
  },
  pac,
  prelude::*,
  timer::{Event, Timer},
};

// LED矩阵配置
const MATRIX_SIZE: usize = 8;
const SCAN_FREQUENCY: u32 = 1000; // 1kHz扫描频率
const BRIGHTNESS_LEVELS: u8 = 16;
const ANIMATION_SPEED: u32 = 100; // 动画速度 (ms)

// 全局显示缓冲区
static DISPLAY_BUFFER: Mutex<RefCell<DisplayBuffer>> =
  Mutex::new(RefCell::new(DisplayBuffer::new()));

/// 显示缓冲区
#[derive(Debug, Clone)]
pub struct DisplayBuffer {
  pub frame_buffer: [[u8; MATRIX_SIZE]; MATRIX_SIZE],
  pub brightness: u8,
  pub current_row: usize,
  pub scan_counter: u32,
  pub dirty: bool,
}

impl DisplayBuffer {
  pub const fn new() -> Self {
    Self {
      frame_buffer: [[0; MATRIX_SIZE]; MATRIX_SIZE],
      brightness: 8,
      current_row: 0,
      scan_counter: 0,
      dirty: true,
    }
  }

  pub fn set_pixel(&mut self, x: usize, y: usize, brightness: u8) {
    if x < MATRIX_SIZE && y < MATRIX_SIZE {
      self.frame_buffer[y][x] = brightness.min(BRIGHTNESS_LEVELS - 1);
      self.dirty = true;
    }
  }

  pub fn get_pixel(&self, x: usize, y: usize) -> u8 {
    if x < MATRIX_SIZE && y < MATRIX_SIZE {
      self.frame_buffer[y][x]
    } else {
      0
    }
  }

  pub fn clear(&mut self) {
    self.frame_buffer = [[0; MATRIX_SIZE]; MATRIX_SIZE];
    self.dirty = true;
  }

  pub fn fill(&mut self, brightness: u8) {
    let level = brightness.min(BRIGHTNESS_LEVELS - 1);
    for row in &mut self.frame_buffer {
      for pixel in row {
        *pixel = level;
      }
    }
    self.dirty = true;
  }

  pub fn set_brightness(&mut self, brightness: u8) {
    self.brightness = brightness.min(BRIGHTNESS_LEVELS - 1);
  }

  pub fn copy_from(&mut self, other: &[[u8; MATRIX_SIZE]; MATRIX_SIZE]) {
    self.frame_buffer.copy_from_slice(other);
    self.dirty = true;
  }
}

/// LED矩阵控制器
pub struct LedMatrix {
  // 行控制引脚 (阳极)
  pub row_pins: RowPins,
  // 列控制引脚 (阴极)
  pub col_pins: ColPins,
  // PWM计数器
  pub pwm_counter: u8,
  // 扫描状态
  pub scan_phase: ScanPhase,
}

pub struct RowPins {
  pub row0: PA0<Output<PushPull>>,
  pub row1: PA1<Output<PushPull>>,
  pub row2: PA2<Output<PushPull>>,
  pub row3: PA3<Output<PushPull>>,
  pub row4: PA4<Output<PushPull>>,
  pub row5: PA5<Output<PushPull>>,
  pub row6: PA6<Output<PushPull>>,
  pub row7: PA7<Output<PushPull>>,
}

pub struct ColPins {
  pub col0: PB0<Output<PushPull>>,
  pub col1: PB1<Output<PushPull>>,
  pub col2: PB2<Output<PushPull>>,
  pub col3: PB3<Output<PushPull>>,
  pub col4: PB4<Output<PushPull>>,
  pub col5: PB5<Output<PushPull>>,
  pub col6: PB6<Output<PushPull>>,
  pub col7: PB7<Output<PushPull>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScanPhase {
  RowSelect,
  ColOutput,
  Blank,
}

impl LedMatrix {
  pub fn new(row_pins: RowPins, col_pins: ColPins) -> Self {
    Self {
      row_pins,
      col_pins,
      pwm_counter: 0,
      scan_phase: ScanPhase::RowSelect,
    }
  }

  /// 扫描显示
  pub fn scan(&mut self) {
    critical_section::with(|cs| {
      let mut buffer = DISPLAY_BUFFER.borrow(cs).borrow_mut();

      match self.scan_phase {
        ScanPhase::RowSelect => {
          // 关闭所有行和列
          self.disable_all_rows();
          self.disable_all_cols();

          // 选择当前行
          self.enable_row(buffer.current_row);
          self.scan_phase = ScanPhase::ColOutput;
        }
        ScanPhase::ColOutput => {
          // 输出当前行的列数据
          for col in 0..MATRIX_SIZE {
            let pixel_brightness = buffer.frame_buffer[buffer.current_row][col];
            let adjusted_brightness = (pixel_brightness * buffer.brightness) / BRIGHTNESS_LEVELS;

            // PWM控制亮度
            if self.pwm_counter < adjusted_brightness {
              self.enable_col(col);
            } else {
              self.disable_col(col);
            }
          }
          self.scan_phase = ScanPhase::Blank;
        }
        ScanPhase::Blank => {
          // 消隐阶段，防止重影
          self.disable_all_rows();
          self.disable_all_cols();

          // 更新扫描状态
          self.pwm_counter = (self.pwm_counter + 1) % BRIGHTNESS_LEVELS;

          if self.pwm_counter == 0 {
            buffer.current_row = (buffer.current_row + 1) % MATRIX_SIZE;
            buffer.scan_counter += 1;
          }

          self.scan_phase = ScanPhase::RowSelect;
        }
      }
    });
  }

  /// 启用指定行
  fn enable_row(&mut self, row: usize) {
    match row {
      0 => self.row_pins.row0.set_high(),
      1 => self.row_pins.row1.set_high(),
      2 => self.row_pins.row2.set_high(),
      3 => self.row_pins.row3.set_high(),
      4 => self.row_pins.row4.set_high(),
      5 => self.row_pins.row5.set_high(),
      6 => self.row_pins.row6.set_high(),
      7 => self.row_pins.row7.set_high(),
      _ => {}
    }
  }

  /// 禁用所有行
  fn disable_all_rows(&mut self) {
    self.row_pins.row0.set_low();
    self.row_pins.row1.set_low();
    self.row_pins.row2.set_low();
    self.row_pins.row3.set_low();
    self.row_pins.row4.set_low();
    self.row_pins.row5.set_low();
    self.row_pins.row6.set_low();
    self.row_pins.row7.set_low();
  }

  /// 启用指定列
  fn enable_col(&mut self, col: usize) {
    match col {
      0 => self.col_pins.col0.set_low(), // 共阴极，低电平点亮
      1 => self.col_pins.col1.set_low(),
      2 => self.col_pins.col2.set_low(),
      3 => self.col_pins.col3.set_low(),
      4 => self.col_pins.col4.set_low(),
      5 => self.col_pins.col5.set_low(),
      6 => self.col_pins.col6.set_low(),
      7 => self.col_pins.col7.set_low(),
      _ => {}
    }
  }

  /// 禁用指定列
  fn disable_col(&mut self, col: usize) {
    match col {
      0 => self.col_pins.col0.set_high(),
      1 => self.col_pins.col1.set_high(),
      2 => self.col_pins.col2.set_high(),
      3 => self.col_pins.col3.set_high(),
      4 => self.col_pins.col4.set_high(),
      5 => self.col_pins.col5.set_high(),
      6 => self.col_pins.col6.set_high(),
      7 => self.col_pins.col7.set_high(),
      _ => {}
    }
  }

  /// 禁用所有列
  fn disable_all_cols(&mut self) {
    self.col_pins.col0.set_high();
    self.col_pins.col1.set_high();
    self.col_pins.col2.set_high();
    self.col_pins.col3.set_high();
    self.col_pins.col4.set_high();
    self.col_pins.col5.set_high();
    self.col_pins.col6.set_high();
    self.col_pins.col7.set_high();
  }
}

/// 图形绘制工具
pub struct Graphics;

impl Graphics {
  /// 绘制点
  pub fn draw_pixel(x: usize, y: usize, brightness: u8) {
    critical_section::with(|cs| {
      DISPLAY_BUFFER
        .borrow(cs)
        .borrow_mut()
        .set_pixel(x, y, brightness);
    });
  }

  /// 绘制线段
  pub fn draw_line(x0: i32, y0: i32, x1: i32, y1: i32, brightness: u8) {
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx - dy;

    let mut x = x0;
    let mut y = y0;

    loop {
      if x >= 0 && x < MATRIX_SIZE as i32 && y >= 0 && y < MATRIX_SIZE as i32 {
        Self::draw_pixel(x as usize, y as usize, brightness);
      }

      if x == x1 && y == y1 {
        break;
      }

      let e2 = 2 * err;
      if e2 > -dy {
        err -= dy;
        x += sx;
      }
      if e2 < dx {
        err += dx;
        y += sy;
      }
    }
  }

  /// 绘制矩形
  pub fn draw_rect(x: usize, y: usize, width: usize, height: usize, brightness: u8) {
    for i in 0..width {
      for j in 0..height {
        if x + i < MATRIX_SIZE && y + j < MATRIX_SIZE {
          Self::draw_pixel(x + i, y + j, brightness);
        }
      }
    }
  }

  /// 绘制圆
  pub fn draw_circle(cx: i32, cy: i32, radius: i32, brightness: u8) {
    let mut x = radius;
    let mut y = 0;
    let mut err = 0;

    while x >= y {
      Self::plot_circle_points(cx, cy, x, y, brightness);

      if err <= 0 {
        y += 1;
        err += 2 * y + 1;
      }

      if err > 0 {
        x -= 1;
        err -= 2 * x + 1;
      }
    }
  }

  fn plot_circle_points(cx: i32, cy: i32, x: i32, y: i32, brightness: u8) {
    let points = [
      (cx + x, cy + y),
      (cx - x, cy + y),
      (cx + x, cy - y),
      (cx - x, cy - y),
      (cx + y, cy + x),
      (cx - y, cy + x),
      (cx + y, cy - x),
      (cx - y, cy - x),
    ];

    for (px, py) in points.iter() {
      if *px >= 0 && *px < MATRIX_SIZE as i32 && *py >= 0 && *py < MATRIX_SIZE as i32 {
        Self::draw_pixel(*px as usize, *py as usize, brightness);
      }
    }
  }

  /// 清屏
  pub fn clear() {
    critical_section::with(|cs| {
      DISPLAY_BUFFER.borrow(cs).borrow_mut().clear();
    });
  }

  /// 填充
  pub fn fill(brightness: u8) {
    critical_section::with(|cs| {
      DISPLAY_BUFFER.borrow(cs).borrow_mut().fill(brightness);
    });
  }
}

/// 动画控制器
pub struct AnimationController {
  pub current_animation: AnimationType,
  pub frame_counter: u32,
  pub animation_speed: u32,
  pub direction: i32,
  pub position: i32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnimationType {
  Static,
  Blink,
  Scroll,
  Bounce,
  Spiral,
  Rain,
  Wave,
  Heart,
}

impl AnimationController {
  pub fn new() -> Self {
    Self {
      current_animation: AnimationType::Static,
      frame_counter: 0,
      animation_speed: ANIMATION_SPEED,
      direction: 1,
      position: 0,
    }
  }

  pub fn set_animation(&mut self, animation: AnimationType) {
    self.current_animation = animation;
    self.frame_counter = 0;
    self.position = 0;
    self.direction = 1;
  }

  pub fn update(&mut self) {
    self.frame_counter += 1;

    if self.frame_counter % self.animation_speed == 0 {
      match self.current_animation {
        AnimationType::Static => {
          // 静态显示
          self.draw_static_pattern();
        }
        AnimationType::Blink => {
          // 闪烁效果
          self.draw_blink_pattern();
        }
        AnimationType::Scroll => {
          // 滚动效果
          self.draw_scroll_pattern();
        }
        AnimationType::Bounce => {
          // 弹跳效果
          self.draw_bounce_pattern();
        }
        AnimationType::Spiral => {
          // 螺旋效果
          self.draw_spiral_pattern();
        }
        AnimationType::Rain => {
          // 雨滴效果
          self.draw_rain_pattern();
        }
        AnimationType::Wave => {
          // 波浪效果
          self.draw_wave_pattern();
        }
        AnimationType::Heart => {
          // 心形效果
          self.draw_heart_pattern();
        }
      }
    }
  }

  fn draw_static_pattern(&self) {
    Graphics::clear();
    // 绘制一个简单的笑脸
    Graphics::draw_circle(3, 3, 3, 8);
    Graphics::draw_pixel(2, 2, 15);
    Graphics::draw_pixel(4, 2, 15);
    Graphics::draw_line(1, 4, 5, 4, 15);
  }

  fn draw_blink_pattern(&mut self) {
    if (self.frame_counter / self.animation_speed) % 2 == 0 {
      Graphics::fill(15);
    } else {
      Graphics::clear();
    }
  }

  fn draw_scroll_pattern(&mut self) {
    Graphics::clear();

    // 滚动文字 "HELLO"
    let text_pattern = [
      0b01111110, // H
      0b10000001, 0b10000001, 0b11111111, 0b10000001, 0b10000001, 0b10000001, 0b00000000,
    ];

    let offset = self.position % (MATRIX_SIZE as i32 + 8);

    for (y, &pattern) in text_pattern.iter().enumerate() {
      for x in 0..8 {
        if (pattern >> (7 - x)) & 1 != 0 {
          let display_x = x as i32 + offset;
          if display_x >= 0 && display_x < MATRIX_SIZE as i32 {
            Graphics::draw_pixel(display_x as usize, y, 12);
          }
        }
      }
    }

    self.position += self.direction;
    if self.position >= MATRIX_SIZE as i32 + 8 {
      self.position = -(8);
    }
  }

  fn draw_bounce_pattern(&mut self) {
    Graphics::clear();

    // 弹跳的球
    let ball_x = 3;
    let ball_y = self.position as usize;

    if ball_y < MATRIX_SIZE {
      Graphics::draw_pixel(ball_x, ball_y, 15);
      Graphics::draw_pixel(ball_x + 1, ball_y, 10);
      Graphics::draw_pixel(ball_x, ball_y + 1, 10);
    }

    self.position += self.direction;

    if self.position >= (MATRIX_SIZE - 2) as i32 {
      self.direction = -1;
    } else if self.position <= 0 {
      self.direction = 1;
    }
  }

  fn draw_spiral_pattern(&mut self) {
    Graphics::clear();

    let center_x = 3.5;
    let center_y = 3.5;
    let angle = (self.frame_counter as f32 / 10.0) % (2.0 * 3.14159);
    let radius = 2.0 + (angle / 2.0);

    let x = center_x + radius * angle.cos();
    let y = center_y + radius * angle.sin();

    if x >= 0.0 && x < MATRIX_SIZE as f32 && y >= 0.0 && y < MATRIX_SIZE as f32 {
      Graphics::draw_pixel(x as usize, y as usize, 15);
    }
  }

  fn draw_rain_pattern(&mut self) {
    // 随机雨滴效果
    if self.frame_counter % 3 == 0 {
      // 在顶部随机生成雨滴
      let x = (self.frame_counter * 7) % MATRIX_SIZE as u32;
      Graphics::draw_pixel(x as usize, 0, 12);
    }

    // 移动现有雨滴
    critical_section::with(|cs| {
      let mut buffer = DISPLAY_BUFFER.borrow(cs).borrow_mut();

      // 从下往上扫描，移动雨滴
      for y in (1..MATRIX_SIZE).rev() {
        for x in 0..MATRIX_SIZE {
          if buffer.frame_buffer[y - 1][x] > 0 {
            buffer.frame_buffer[y][x] = buffer.frame_buffer[y - 1][x];
            buffer.frame_buffer[y - 1][x] = 0;
          }
        }
      }

      buffer.dirty = true;
    });
  }

  fn draw_wave_pattern(&mut self) {
    Graphics::clear();

    let phase = (self.frame_counter as f32 / 5.0) % (2.0 * 3.14159);

    for x in 0..MATRIX_SIZE {
      let y = 3.5 + 2.0 * ((x as f32 * 0.5 + phase).sin());
      let y_int = y as usize;

      if y_int < MATRIX_SIZE {
        Graphics::draw_pixel(x, y_int, 12);
      }
    }
  }

  fn draw_heart_pattern(&self) {
    Graphics::clear();

    // 心形图案
    let heart_pattern = [
      [0, 1, 1, 0, 0, 1, 1, 0],
      [1, 1, 1, 1, 1, 1, 1, 1],
      [1, 1, 1, 1, 1, 1, 1, 1],
      [1, 1, 1, 1, 1, 1, 1, 1],
      [0, 1, 1, 1, 1, 1, 1, 0],
      [0, 0, 1, 1, 1, 1, 0, 0],
      [0, 0, 0, 1, 1, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0],
    ];

    let brightness = if (self.frame_counter / 20) % 2 == 0 {
      15
    } else {
      8
    };

    for (y, row) in heart_pattern.iter().enumerate() {
      for (x, &pixel) in row.iter().enumerate() {
        if pixel == 1 {
          Graphics::draw_pixel(x, y, brightness);
        }
      }
    }
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

  // 配置LED矩阵引脚
  let row_pins = RowPins {
    row0: gpioa.pa0.into_push_pull_output(),
    row1: gpioa.pa1.into_push_pull_output(),
    row2: gpioa.pa2.into_push_pull_output(),
    row3: gpioa.pa3.into_push_pull_output(),
    row4: gpioa.pa4.into_push_pull_output(),
    row5: gpioa.pa5.into_push_pull_output(),
    row6: gpioa.pa6.into_push_pull_output(),
    row7: gpioa.pa7.into_push_pull_output(),
  };

  let col_pins = ColPins {
    col0: gpiob.pb0.into_push_pull_output(),
    col1: gpiob.pb1.into_push_pull_output(),
    col2: gpiob.pb2.into_push_pull_output(),
    col3: gpiob.pb3.into_push_pull_output(),
    col4: gpiob.pb4.into_push_pull_output(),
    col5: gpiob.pb5.into_push_pull_output(),
    col6: gpiob.pb6.into_push_pull_output(),
    col7: gpiob.pb7.into_push_pull_output(),
  };

  // 配置状态LED
  let status_led = gpioc.pc13.into_push_pull_output();

  // 创建控制器
  let mut led_matrix = LedMatrix::new(row_pins, col_pins);
  let mut animation = AnimationController::new();

  // 配置定时器用于扫描
  let mut timer = Timer::new(dp.TIM2, &clocks);
  timer.start(SCAN_FREQUENCY.Hz());
  timer.listen(Event::Update);

  // 启动序列
  startup_animation(&mut animation);

  let mut loop_counter = 0u32;
  let mut animation_index = 0usize;
  let animations = [
    AnimationType::Heart,
    AnimationType::Blink,
    AnimationType::Scroll,
    AnimationType::Bounce,
    AnimationType::Spiral,
    AnimationType::Rain,
    AnimationType::Wave,
  ];

  loop {
    loop_counter += 1;

    // 高频扫描显示
    if loop_counter % 100 == 0 {
      led_matrix.scan();
    }

    // 更新动画
    if loop_counter % 1000 == 0 {
      animation.update();
    }

    // 切换动画
    if loop_counter % 500000 == 0 {
      animation_index = (animation_index + 1) % animations.len();
      animation.set_animation(animations[animation_index]);
    }

    // 亮度调节演示
    if loop_counter % 100000 == 0 {
      let brightness = ((loop_counter / 100000) % 16) as u8;
      critical_section::with(|cs| {
        DISPLAY_BUFFER
          .borrow(cs)
          .borrow_mut()
          .set_brightness(brightness);
      });
    }

    // 短暂延时
    delay_us(10);
  }
}

fn startup_animation(animation: &mut AnimationController) {
  // 启动动画序列
  animation.set_animation(AnimationType::Spiral);

  for _ in 0..100 {
    animation.update();
    delay_ms(50);
  }

  Graphics::clear();
  delay_ms(500);

  // 显示启动完成
  animation.set_animation(AnimationType::Heart);
  for _ in 0..20 {
    animation.update();
    delay_ms(100);
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
