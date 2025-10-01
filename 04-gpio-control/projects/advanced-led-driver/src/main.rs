//! # 高级LED驱动项目
//!
//! 本项目实现了高级LED驱动功能，包括PWM调光、RGB颜色控制、
//! 动态效果和LED矩阵控制等功能。

#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_halt as _;

// RTT调试输出
use rtt_target::{rprintln, rtt_init_print};

// 数学库
use heapless::{FnvIndexMap, Vec};
use micromath::F32Ext;
use num_traits::float::Float;

// 时间
use fugit::{Duration, Instant, Rate};

// 颜色处理
#[cfg(feature = "rgb-support")]
use palette::{FromColor, Hsv, IntoColor, Srgb};

/// PWM控制器抽象
pub struct PwmController {
  frequency: u32,  // PWM频率 (Hz)
  resolution: u16, // PWM分辨率 (位)
  duty_cycle: u16, // 当前占空比
}

impl PwmController {
  /// 创建PWM控制器
  pub fn new(frequency: u32, resolution: u16) -> Self {
    rprintln!("创建PWM控制器: {}Hz, {}位分辨率", frequency, resolution);
    Self {
      frequency,
      resolution,
      duty_cycle: 0,
    }
  }

  /// 设置占空比 (0-100%)
  pub fn set_duty_cycle(&mut self, percentage: f32) {
    let max_value = (1 << self.resolution) - 1;
    self.duty_cycle = ((percentage / 100.0) * max_value as f32) as u16;
    rprintln!("设置PWM占空比: {:.1}% ({})", percentage, self.duty_cycle);
  }

  /// 获取当前占空比
  pub fn get_duty_cycle(&self) -> f32 {
    let max_value = (1 << self.resolution) - 1;
    (self.duty_cycle as f32 / max_value as f32) * 100.0
  }

  /// 渐变到目标占空比
  pub fn fade_to(&mut self, target_percentage: f32, steps: u16, delay_ms: u32) {
    let current = self.get_duty_cycle();
    let step_size = (target_percentage - current) / steps as f32;

    rprintln!(
      "PWM渐变: {:.1}% -> {:.1}% ({}步)",
      current,
      target_percentage,
      steps
    );

    for i in 0..steps {
      let new_duty = current + step_size * (i + 1) as f32;
      self.set_duty_cycle(new_duty);

      // 模拟延迟
      for _ in 0..(delay_ms * 1000) {
        asm::nop();
      }
    }
  }
}

/// RGB LED控制器
#[cfg(feature = "rgb-support")]
pub struct RgbLed {
  red_pwm: PwmController,
  green_pwm: PwmController,
  blue_pwm: PwmController,
  current_color: Srgb<f32>,
}

#[cfg(feature = "rgb-support")]
impl RgbLed {
  /// 创建RGB LED控制器
  pub fn new(pwm_freq: u32, pwm_resolution: u16) -> Self {
    rprintln!("创建RGB LED控制器");
    Self {
      red_pwm: PwmController::new(pwm_freq, pwm_resolution),
      green_pwm: PwmController::new(pwm_freq, pwm_resolution),
      blue_pwm: PwmController::new(pwm_freq, pwm_resolution),
      current_color: Srgb::new(0.0, 0.0, 0.0),
    }
  }

  /// 设置RGB颜色 (0.0-1.0)
  pub fn set_rgb(&mut self, r: f32, g: f32, b: f32) {
    self.current_color = Srgb::new(r, g, b);
    self.red_pwm.set_duty_cycle(r * 100.0);
    self.green_pwm.set_duty_cycle(g * 100.0);
    self.blue_pwm.set_duty_cycle(b * 100.0);

    rprintln!("设置RGB颜色: ({:.2}, {:.2}, {:.2})", r, g, b);
  }

  /// 设置HSV颜色
  pub fn set_hsv(&mut self, h: f32, s: f32, v: f32) {
    let hsv = Hsv::new(h, s, v);
    let rgb: Srgb<f32> = Srgb::from_color(hsv);
    self.set_rgb(rgb.red, rgb.green, rgb.blue);

    rprintln!("设置HSV颜色: ({:.1}°, {:.2}, {:.2})", h, s, v);
  }

  /// 设置预定义颜色
  pub fn set_color(&mut self, color: PredefinedColor) {
    let (r, g, b) = color.to_rgb();
    self.set_rgb(r, g, b);
    rprintln!("设置预定义颜色: {:?}", color);
  }

  /// 颜色渐变
  pub fn fade_to_rgb(
    &mut self,
    target_r: f32,
    target_g: f32,
    target_b: f32,
    steps: u16,
    delay_ms: u32,
  ) {
    let current = self.current_color;
    let step_r = (target_r - current.red) / steps as f32;
    let step_g = (target_g - current.green) / steps as f32;
    let step_b = (target_b - current.blue) / steps as f32;

    rprintln!(
      "RGB颜色渐变: ({:.2},{:.2},{:.2}) -> ({:.2},{:.2},{:.2})",
      current.red,
      current.green,
      current.blue,
      target_r,
      target_g,
      target_b
    );

    for i in 0..steps {
      let new_r = current.red + step_r * (i + 1) as f32;
      let new_g = current.green + step_g * (i + 1) as f32;
      let new_b = current.blue + step_b * (i + 1) as f32;

      self.set_rgb(new_r, new_g, new_b);

      // 模拟延迟
      for _ in 0..(delay_ms * 1000) {
        asm::nop();
      }
    }
  }
}

/// 预定义颜色
#[derive(Debug, Clone, Copy)]
pub enum PredefinedColor {
  Red,
  Green,
  Blue,
  Yellow,
  Cyan,
  Magenta,
  White,
  Orange,
  Purple,
  Pink,
}

impl PredefinedColor {
  pub fn to_rgb(self) -> (f32, f32, f32) {
    match self {
      Self::Red => (1.0, 0.0, 0.0),
      Self::Green => (0.0, 1.0, 0.0),
      Self::Blue => (0.0, 0.0, 1.0),
      Self::Yellow => (1.0, 1.0, 0.0),
      Self::Cyan => (0.0, 1.0, 1.0),
      Self::Magenta => (1.0, 0.0, 1.0),
      Self::White => (1.0, 1.0, 1.0),
      Self::Orange => (1.0, 0.5, 0.0),
      Self::Purple => (0.5, 0.0, 1.0),
      Self::Pink => (1.0, 0.5, 0.8),
    }
  }
}

/// LED矩阵控制器
pub struct LedMatrix<const ROWS: usize, const COLS: usize> {
  buffer: [[bool; COLS]; ROWS],
  brightness: f32,
  scan_row: usize,
}

impl<const ROWS: usize, const COLS: usize> LedMatrix<ROWS, COLS> {
  /// 创建LED矩阵
  pub fn new() -> Self {
    rprintln!("创建{}x{}LED矩阵", ROWS, COLS);
    Self {
      buffer: [[false; COLS]; ROWS],
      brightness: 1.0,
      scan_row: 0,
    }
  }

  /// 设置像素
  pub fn set_pixel(&mut self, row: usize, col: usize, state: bool) {
    if row < ROWS && col < COLS {
      self.buffer[row][col] = state;
    }
  }

  /// 清空矩阵
  pub fn clear(&mut self) {
    self.buffer = [[false; COLS]; ROWS];
    rprintln!("清空LED矩阵");
  }

  /// 设置亮度
  pub fn set_brightness(&mut self, brightness: f32) {
    self.brightness = brightness.clamp(0.0, 1.0);
    rprintln!("设置矩阵亮度: {:.1}%", self.brightness * 100.0);
  }

  /// 显示字符
  pub fn display_char(&mut self, ch: char, x_offset: usize, y_offset: usize) {
    // 简单的5x7字符显示
    let pattern = get_char_pattern(ch);

    for (row, &row_data) in pattern.iter().enumerate() {
      for col in 0..5 {
        let pixel_on = (row_data >> (4 - col)) & 1 == 1;
        let matrix_row = y_offset + row;
        let matrix_col = x_offset + col;

        if matrix_row < ROWS && matrix_col < COLS {
          self.set_pixel(matrix_row, matrix_col, pixel_on);
        }
      }
    }

    rprintln!("显示字符: '{}'", ch);
  }

  /// 滚动文本
  pub fn scroll_text(&mut self, text: &str, delay_ms: u32) {
    rprintln!("滚动显示文本: \"{}\"", text);

    for ch in text.chars() {
      self.clear();
      self.display_char(ch, 0, 0);
      self.refresh();

      // 模拟延迟
      for _ in 0..(delay_ms * 10000) {
        asm::nop();
      }
    }
  }

  /// 刷新显示 (行扫描)
  pub fn refresh(&mut self) {
    // 模拟行扫描显示
    for col in 0..COLS {
      if self.buffer[self.scan_row][col] {
        // 点亮当前行的LED
      }
    }

    self.scan_row = (self.scan_row + 1) % ROWS;
  }
}

/// 获取字符点阵模式 (5x7)
fn get_char_pattern(ch: char) -> [u8; 7] {
  match ch {
    'A' => [0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x00],
    'B' => [0x1E, 0x11, 0x1E, 0x1E, 0x11, 0x1E, 0x00],
    'C' => [0x0E, 0x11, 0x10, 0x10, 0x11, 0x0E, 0x00],
    'H' => [0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x00],
    'E' => [0x1F, 0x10, 0x1E, 0x1E, 0x10, 0x1F, 0x00],
    'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x1F, 0x00],
    'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x00],
    ' ' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    _ => [0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00], // 默认实心块
  }
}

/// LED效果引擎
pub struct EffectEngine {
  time_counter: u32,
  effect_speed: f32,
}

impl EffectEngine {
  pub fn new() -> Self {
    rprintln!("创建LED效果引擎");
    Self {
      time_counter: 0,
      effect_speed: 1.0,
    }
  }

  /// 设置效果速度
  pub fn set_speed(&mut self, speed: f32) {
    self.effect_speed = speed.clamp(0.1, 10.0);
    rprintln!("设置效果速度: {:.1}x", self.effect_speed);
  }

  /// 呼吸灯效果
  #[cfg(feature = "rgb-support")]
  pub fn breathing_effect(&mut self, led: &mut RgbLed, color: PredefinedColor) {
    let (r, g, b) = color.to_rgb();

    // 使用正弦波产生呼吸效果
    let phase =
      (self.time_counter as f32 * self.effect_speed * 0.01) % (2.0 * core::f32::consts::PI);
    let brightness = (phase.sin() + 1.0) / 2.0; // 0.0 到 1.0

    led.set_rgb(r * brightness, g * brightness, b * brightness);

    if self.time_counter % 100 == 0 {
      rprintln!("呼吸灯效果: 亮度 {:.1}%", brightness * 100.0);
    }
  }

  /// 彩虹效果
  #[cfg(feature = "rgb-support")]
  pub fn rainbow_effect(&mut self, led: &mut RgbLed) {
    let hue = (self.time_counter as f32 * self.effect_speed * 0.5) % 360.0;
    led.set_hsv(hue, 1.0, 1.0);

    if self.time_counter % 50 == 0 {
      rprintln!("彩虹效果: 色相 {:.1}°", hue);
    }
  }

  /// 闪烁效果
  pub fn blink_effect(&mut self, pwm: &mut PwmController, frequency: f32) {
    let period = (1.0 / frequency / self.effect_speed) * 1000.0; // ms
    let half_period = period / 2.0;
    let phase = (self.time_counter as f32) % period;

    let brightness = if phase < half_period { 100.0 } else { 0.0 };
    pwm.set_duty_cycle(brightness);

    if self.time_counter % 100 == 0 {
      rprintln!(
        "闪烁效果: {}Hz, 当前状态: {}",
        frequency,
        if brightness > 0.0 { "ON" } else { "OFF" }
      );
    }
  }

  /// 更新时间计数器
  pub fn update(&mut self) {
    self.time_counter = self.time_counter.wrapping_add(1);
  }
}

/// LED驱动器性能监控
pub struct PerformanceMonitor {
  pwm_update_count: u32,
  color_change_count: u32,
  effect_render_count: u32,
  last_report_time: u32,
}

impl PerformanceMonitor {
  pub fn new() -> Self {
    Self {
      pwm_update_count: 0,
      color_change_count: 0,
      effect_render_count: 0,
      last_report_time: 0,
    }
  }

  pub fn record_pwm_update(&mut self) {
    self.pwm_update_count += 1;
  }

  pub fn record_color_change(&mut self) {
    self.color_change_count += 1;
  }

  pub fn record_effect_render(&mut self) {
    self.effect_render_count += 1;
  }

  pub fn report_performance(&mut self, current_time: u32) {
    if current_time - self.last_report_time >= 10000 {
      // 每10秒报告一次
      rprintln!("=== LED驱动性能报告 ===");
      rprintln!("PWM更新次数: {}", self.pwm_update_count);
      rprintln!("颜色变化次数: {}", self.color_change_count);
      rprintln!("效果渲染次数: {}", self.effect_render_count);

      // 重置计数器
      self.pwm_update_count = 0;
      self.color_change_count = 0;
      self.effect_render_count = 0;
      self.last_report_time = current_time;
    }
  }
}

/// 应用程序演示
fn demonstrate_pwm_control() {
  rprintln!("\n=== PWM调光演示 ===");

  let mut pwm = PwmController::new(1000, 12); // 1kHz, 12位

  // 基本PWM控制
  pwm.set_duty_cycle(25.0);
  pwm.set_duty_cycle(50.0);
  pwm.set_duty_cycle(75.0);
  pwm.set_duty_cycle(100.0);

  // 渐变效果
  pwm.fade_to(0.0, 50, 10);
  pwm.fade_to(100.0, 100, 5);

  rprintln!("PWM调光演示完成");
}

#[cfg(feature = "rgb-support")]
fn demonstrate_rgb_control() {
  rprintln!("\n=== RGB颜色控制演示 ===");

  let mut rgb_led = RgbLed::new(2000, 10); // 2kHz, 10位

  // 基本颜色设置
  rgb_led.set_color(PredefinedColor::Red);
  rgb_led.set_color(PredefinedColor::Green);
  rgb_led.set_color(PredefinedColor::Blue);

  // HSV颜色设置
  rgb_led.set_hsv(0.0, 1.0, 1.0); // 红色
  rgb_led.set_hsv(120.0, 1.0, 1.0); // 绿色
  rgb_led.set_hsv(240.0, 1.0, 1.0); // 蓝色

  // 颜色渐变
  rgb_led.fade_to_rgb(1.0, 1.0, 0.0, 30, 20); // 渐变到黄色

  rprintln!("RGB颜色控制演示完成");
}

fn demonstrate_led_matrix() {
  rprintln!("\n=== LED矩阵演示 ===");

  let mut matrix = LedMatrix::<8, 8>::new();

  // 显示字符
  matrix.display_char('H', 0, 0);
  matrix.refresh();

  // 设置亮度
  matrix.set_brightness(0.5);

  // 滚动文本
  matrix.scroll_text("HELLO", 500);

  rprintln!("LED矩阵演示完成");
}

#[cfg(feature = "rgb-support")]
fn demonstrate_effects() {
  rprintln!("\n=== LED效果演示 ===");

  let mut rgb_led = RgbLed::new(5000, 8); // 5kHz, 8位
  let mut pwm = PwmController::new(1000, 10);
  let mut effect_engine = EffectEngine::new();

  effect_engine.set_speed(2.0);

  // 模拟效果循环
  for _ in 0..200 {
    effect_engine.breathing_effect(&mut rgb_led, PredefinedColor::Blue);
    effect_engine.update();

    // 模拟延迟
    for _ in 0..10000 {
      asm::nop();
    }
  }

  for _ in 0..100 {
    effect_engine.rainbow_effect(&mut rgb_led);
    effect_engine.update();

    // 模拟延迟
    for _ in 0..10000 {
      asm::nop();
    }
  }

  for _ in 0..50 {
    effect_engine.blink_effect(&mut pwm, 2.0); // 2Hz闪烁
    effect_engine.update();

    // 模拟延迟
    for _ in 0..10000 {
      asm::nop();
    }
  }

  rprintln!("LED效果演示完成");
}

#[entry]
fn main() -> ! {
  // 初始化RTT调试输出
  rtt_init_print!();

  rprintln!("=== 高级LED驱动项目 ===");
  rprintln!("PWM调光、RGB颜色控制和动态效果演示\n");

  // PWM调光演示
  demonstrate_pwm_control();

  // RGB颜色控制演示
  #[cfg(feature = "rgb-support")]
  demonstrate_rgb_control();

  // LED矩阵演示
  demonstrate_led_matrix();

  // LED效果演示
  #[cfg(feature = "rgb-support")]
  demonstrate_effects();

  // 性能监控
  let mut monitor = PerformanceMonitor::new();

  rprintln!("\n=== 技术特性总结 ===");
  rprintln!("1. PWM调光: 高精度亮度控制");
  rprintln!("2. RGB颜色: HSV/RGB颜色空间支持");
  rprintln!("3. LED矩阵: 行扫描显示技术");
  rprintln!("4. 动态效果: 呼吸、彩虹、闪烁效果");
  rprintln!("5. 性能监控: 实时性能统计");

  rprintln!("\n程序完成，进入监控循环");

  let mut counter = 0u32;

  // 主循环 - 性能监控
  loop {
    counter = counter.wrapping_add(1);

    // 模拟LED操作
    if counter % 1000 == 0 {
      monitor.record_pwm_update();
    }
    if counter % 5000 == 0 {
      monitor.record_color_change();
    }
    if counter % 2000 == 0 {
      monitor.record_effect_render();
    }

    // 性能报告
    monitor.report_performance(counter);

    asm::wfi();
  }
}
