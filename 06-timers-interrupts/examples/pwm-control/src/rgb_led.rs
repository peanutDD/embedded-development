#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{
    gpioa::{PA10, PA8, PA9},
    Alternate, AF1,
  },
  prelude::*,
  stm32,
  timer::{Channel, PwmChannels, Timer},
};

type RedPin = PA8<Alternate<AF1>>;
type GreenPin = PA9<Alternate<AF1>>;
type BluePin = PA10<Alternate<AF1>>;

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
  let red_pin = gpioa.pa8.into_alternate_af1();
  let green_pin = gpioa.pa9.into_alternate_af1();
  let blue_pin = gpioa.pa10.into_alternate_af1();

  // 配置定时器1用于PWM
  let mut timer = Timer::tim1(dp.TIM1, &clocks);
  let red_pwm = timer.pwm::<Timer1Ch1>(red_pin, 1.khz());
  let green_pwm = timer.pwm::<Timer1Ch2>(green_pin, 1.khz());
  let blue_pwm = timer.pwm::<Timer1Ch3>(blue_pin, 1.khz());

  // 创建RGB LED控制器
  let mut rgb_led = RgbLedController::new(red_pwm, green_pwm, blue_pwm);

  // 配置系统定时器用于延时
  let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);

  // RGB LED控制主循环
  loop {
    // 基本颜色演示
    rgb_led.set_color(Color::Red);
    delay.delay_ms(1000u32);

    rgb_led.set_color(Color::Green);
    delay.delay_ms(1000u32);

    rgb_led.set_color(Color::Blue);
    delay.delay_ms(1000u32);

    rgb_led.set_color(Color::White);
    delay.delay_ms(1000u32);

    rgb_led.set_color(Color::Yellow);
    delay.delay_ms(1000u32);

    rgb_led.set_color(Color::Cyan);
    delay.delay_ms(1000u32);

    rgb_led.set_color(Color::Magenta);
    delay.delay_ms(1000u32);

    // 彩虹效果
    rgb_led.rainbow_effect(50, 20);

    // 呼吸灯效果
    rgb_led.breathing_effect(Color::Blue, 100, 2000);

    // 渐变效果
    rgb_led.fade_between_colors(Color::Red, Color::Blue, 50, 50);

    // 关闭
    rgb_led.turn_off();
    delay.delay_ms(1000u32);
  }
}

/// RGB颜色结构体
#[derive(Clone, Copy)]
pub struct RgbColor {
  pub red: u8,
  pub green: u8,
  pub blue: u8,
}

/// 预定义颜色
pub enum Color {
  Red,
  Green,
  Blue,
  White,
  Black,
  Yellow,
  Cyan,
  Magenta,
  Orange,
  Purple,
  Pink,
  Custom(RgbColor),
}

impl Color {
  /// 转换为RGB值
  pub fn to_rgb(&self) -> RgbColor {
    match self {
      Color::Red => RgbColor {
        red: 255,
        green: 0,
        blue: 0,
      },
      Color::Green => RgbColor {
        red: 0,
        green: 255,
        blue: 0,
      },
      Color::Blue => RgbColor {
        red: 0,
        green: 0,
        blue: 255,
      },
      Color::White => RgbColor {
        red: 255,
        green: 255,
        blue: 255,
      },
      Color::Black => RgbColor {
        red: 0,
        green: 0,
        blue: 0,
      },
      Color::Yellow => RgbColor {
        red: 255,
        green: 255,
        blue: 0,
      },
      Color::Cyan => RgbColor {
        red: 0,
        green: 255,
        blue: 255,
      },
      Color::Magenta => RgbColor {
        red: 255,
        green: 0,
        blue: 255,
      },
      Color::Orange => RgbColor {
        red: 255,
        green: 165,
        blue: 0,
      },
      Color::Purple => RgbColor {
        red: 128,
        green: 0,
        blue: 128,
      },
      Color::Pink => RgbColor {
        red: 255,
        green: 192,
        blue: 203,
      },
      Color::Custom(rgb) => *rgb,
    }
  }
}

/// RGB LED控制器结构体
pub struct RgbLedController<R_PIN, G_PIN, B_PIN> {
  red_pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, R_PIN>,
  green_pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, G_PIN>,
  blue_pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, B_PIN>,
  max_duty: u16,
  current_color: RgbColor,
}

impl<R_PIN, G_PIN, B_PIN> RgbLedController<R_PIN, G_PIN, B_PIN> {
  /// 创建新的RGB LED控制器
  pub fn new(
    mut red_pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, R_PIN>,
    mut green_pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, G_PIN>,
    mut blue_pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, B_PIN>,
  ) -> Self {
    // 启用所有PWM通道
    red_pwm.enable(Channel::C1);
    green_pwm.enable(Channel::C2);
    blue_pwm.enable(Channel::C3);

    let max_duty = red_pwm.get_max_duty();

    Self {
      red_pwm,
      green_pwm,
      blue_pwm,
      max_duty,
      current_color: RgbColor {
        red: 0,
        green: 0,
        blue: 0,
      },
    }
  }

  /// 设置RGB颜色
  pub fn set_rgb(&mut self, red: u8, green: u8, blue: u8) {
    let red_duty = (self.max_duty * red as u16) / 255;
    let green_duty = (self.max_duty * green as u16) / 255;
    let blue_duty = (self.max_duty * blue as u16) / 255;

    self.red_pwm.set_duty(Channel::C1, red_duty);
    self.green_pwm.set_duty(Channel::C2, green_duty);
    self.blue_pwm.set_duty(Channel::C3, blue_duty);

    self.current_color = RgbColor { red, green, blue };
  }

  /// 设置颜色
  pub fn set_color(&mut self, color: Color) {
    let rgb = color.to_rgb();
    self.set_rgb(rgb.red, rgb.green, rgb.blue);
  }

  /// 设置亮度 (0-100%)
  pub fn set_brightness(&mut self, brightness: u8) {
    let brightness = brightness.min(100);
    let factor = brightness as u16;

    let red = (self.current_color.red as u16 * factor) / 100;
    let green = (self.current_color.green as u16 * factor) / 100;
    let blue = (self.current_color.blue as u16 * factor) / 100;

    self.set_rgb(red as u8, green as u8, blue as u8);
  }

  /// 关闭LED
  pub fn turn_off(&mut self) {
    self.set_rgb(0, 0, 0);
  }

  /// 获取当前颜色
  pub fn get_current_color(&self) -> RgbColor {
    self.current_color
  }

  /// HSV转RGB
  pub fn hsv_to_rgb(hue: u16, saturation: u8, value: u8) -> RgbColor {
    let h = hue % 360;
    let s = saturation.min(100) as f32 / 100.0;
    let v = value.min(100) as f32 / 100.0;

    let c = v * s;
    let x = c * (1.0 - ((h as f32 / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;

    let (r_prime, g_prime, b_prime) = match h {
      0..=59 => (c, x, 0.0),
      60..=119 => (x, c, 0.0),
      120..=179 => (0.0, c, x),
      180..=239 => (0.0, x, c),
      240..=299 => (x, 0.0, c),
      300..=359 => (c, 0.0, x),
      _ => (0.0, 0.0, 0.0),
    };

    RgbColor {
      red: ((r_prime + m) * 255.0) as u8,
      green: ((g_prime + m) * 255.0) as u8,
      blue: ((b_prime + m) * 255.0) as u8,
    }
  }

  /// 设置HSV颜色
  pub fn set_hsv(&mut self, hue: u16, saturation: u8, value: u8) {
    let rgb = Self::hsv_to_rgb(hue, saturation, value);
    self.set_rgb(rgb.red, rgb.green, rgb.blue);
  }

  /// 彩虹效果
  pub fn rainbow_effect(&mut self, steps: u16, delay_ms: u32) {
    for step in 0..steps {
      let hue = (360 * step) / steps;
      self.set_hsv(hue, 100, 100);
      // 延时处理
    }
  }

  /// 呼吸灯效果
  pub fn breathing_effect(&mut self, color: Color, steps: u8, cycle_ms: u32) {
    let rgb = color.to_rgb();
    let step_delay = cycle_ms / (steps as u32 * 2);

    // 渐亮
    for step in 0..=steps {
      let brightness = (step * 100) / steps;
      let red = (rgb.red as u16 * brightness as u16) / 100;
      let green = (rgb.green as u16 * brightness as u16) / 100;
      let blue = (rgb.blue as u16 * brightness as u16) / 100;

      self.set_rgb(red as u8, green as u8, blue as u8);
      // 延时处理
    }

    // 渐暗
    for step in (0..=steps).rev() {
      let brightness = (step * 100) / steps;
      let red = (rgb.red as u16 * brightness as u16) / 100;
      let green = (rgb.green as u16 * brightness as u16) / 100;
      let blue = (rgb.blue as u16 * brightness as u16) / 100;

      self.set_rgb(red as u8, green as u8, blue as u8);
      // 延时处理
    }
  }

  /// 两种颜色间渐变
  pub fn fade_between_colors(&mut self, from: Color, to: Color, steps: u8, delay_ms: u32) {
    let from_rgb = from.to_rgb();
    let to_rgb = to.to_rgb();

    for step in 0..=steps {
      let factor = step as u16;
      let inv_factor = (steps - step) as u16;

      let red = (from_rgb.red as u16 * inv_factor + to_rgb.red as u16 * factor) / steps as u16;
      let green =
        (from_rgb.green as u16 * inv_factor + to_rgb.green as u16 * factor) / steps as u16;
      let blue = (from_rgb.blue as u16 * inv_factor + to_rgb.blue as u16 * factor) / steps as u16;

      self.set_rgb(red as u8, green as u8, blue as u8);
      // 延时处理
    }
  }

  /// 闪烁效果
  pub fn blink(&mut self, color: Color, on_time_ms: u32, off_time_ms: u32, count: u8) {
    for _ in 0..count {
      self.set_color(color);
      // 延时 on_time_ms

      self.turn_off();
      // 延时 off_time_ms
    }
  }

  /// 频闪效果
  pub fn strobe(&mut self, color: Color, frequency_hz: u16, duration_ms: u32) {
    let period_ms = 1000 / frequency_hz as u32;
    let cycles = duration_ms / period_ms;

    for _ in 0..cycles {
      self.set_color(color);
      // 延时 period_ms/2

      self.turn_off();
      // 延时 period_ms/2
    }
  }
}
