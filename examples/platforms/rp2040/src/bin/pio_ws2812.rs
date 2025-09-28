#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rp2040_hal::{
  clocks::{init_clocks_and_plls, Clock},
  pac,
  pio::PIOExt,
  sio::Sio,
  watchdog::Watchdog,
};

use rp2040_platform::{debug::print_info, GpioManager, PioManager, SystemInit, Utils};

#[entry]
fn main() -> ! {
  // 系统初始化
  let mut pac = pac::Peripherals::take().unwrap();
  let core = pac::CorePeripherals::take().unwrap();

  print_info("RP2040 PIO WS2812示例启动");

  // 初始化看门狗
  let mut watchdog = Watchdog::new(pac.WATCHDOG);

  // 初始化时钟
  let clocks = init_clocks_and_plls(
    rp2040_hal::XOSC_CRYSTAL_FREQ,
    pac.XOSC,
    pac.CLOCKS,
    pac.PLL_SYS,
    pac.PLL_USB,
    &mut pac.RESETS,
    &mut watchdog,
  )
  .ok()
  .unwrap();

  // 初始化SIO
  let sio = Sio::new(pac.SIO);

  // 初始化GPIO
  let pins = rp2040_hal::gpio::Pins::new(
    pac.IO_BANK0,
    pac.PADS_BANK0,
    sio.gpio_bank0,
    &mut pac.RESETS,
  );

  let mut gpio = GpioManager::new(pins);

  // 初始化PIO
  let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
  let mut pio_manager = PioManager::new();

  print_info("系统初始化完成");

  // 创建WS2812控制器
  let mut ws2812_controller = WS2812Controller::new(&mut pio, sm0, pins.gpio2.into_mode());
  let mut led_effects = LedEffects::new();
  let mut button_handler = ButtonHandler::new();

  print_info("WS2812控制器已初始化");

  // 主循环
  loop {
    // 检查按钮状态
    button_handler.update(&gpio);

    if button_handler.is_button_pressed(1) {
      led_effects.next_effect();
      print_info("切换到下一个LED效果");
    }

    if button_handler.is_button_pressed(2) {
      led_effects.adjust_brightness();
      print_info("调整亮度");
    }

    if button_handler.is_button_pressed(3) {
      led_effects.adjust_speed();
      print_info("调整速度");
    }

    // 更新LED效果
    led_effects.update();

    // 渲染LED
    ws2812_controller.render(&led_effects.get_colors());

    // 状态LED指示
    gpio.set_led(1, led_effects.is_active()).unwrap();

    Utils::delay_ms(led_effects.get_delay());
  }
}

// WS2812控制器
struct WS2812Controller<P> {
  pio_program: P,
  led_count: usize,
}

impl<P> WS2812Controller<P> {
  fn new(
    pio: &mut rp2040_hal::pio::PIO<rp2040_hal::pac::PIO0>,
    sm: rp2040_hal::pio::UninitStateMachine<(rp2040_hal::pac::PIO0, rp2040_hal::pio::SM0)>,
    pin: rp2040_hal::gpio::Pin<rp2040_hal::gpio::pin::bank0::Gpio2, rp2040_hal::gpio::FunctionPio0>,
  ) -> Self {
    // 安装WS2812 PIO程序
    let program = pio_proc::pio_asm!(
      ".side_set 1",
      ".wrap_target",
      "bitloop:",
      "    out x, 1       side 0 [T3 - 1]",
      "    jmp !x do_zero side 1 [T1 - 1]",
      "    jmp  bitloop   side 1 [T2 - 1]",
      "do_zero:",
      "    nop            side 0 [T2 - 1]",
      ".wrap",
      options(max_program_size = 32)
    );

    Self {
      pio_program: (),
      led_count: 64, // 假设有64个LED
    }
  }

  fn render(&mut self, colors: &[RGB]) {
    for color in colors.iter().take(self.led_count) {
      self.send_pixel(*color);
    }
    self.latch();
  }

  fn send_pixel(&mut self, color: RGB) {
    // 发送24位颜色数据 (GRB格式)
    let grb = ((color.g as u32) << 16) | ((color.r as u32) << 8) | (color.b as u32);

    // 通过PIO发送数据
    // 这里需要实际的PIO数据发送实现
  }

  fn latch(&mut self) {
    // 发送latch信号
    Utils::delay_us(50);
  }

  fn clear(&mut self) {
    let black = RGB { r: 0, g: 0, b: 0 };
    for _ in 0..self.led_count {
      self.send_pixel(black);
    }
    self.latch();
  }
}

// RGB颜色结构
#[derive(Clone, Copy)]
struct RGB {
  r: u8,
  g: u8,
  b: u8,
}

impl RGB {
  fn new(r: u8, g: u8, b: u8) -> Self {
    Self { r, g, b }
  }

  fn from_hsv(h: f32, s: f32, v: f32) -> Self {
    let c = v * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;

    let (r_prime, g_prime, b_prime) = if h < 60.0 {
      (c, x, 0.0)
    } else if h < 120.0 {
      (x, c, 0.0)
    } else if h < 180.0 {
      (0.0, c, x)
    } else if h < 240.0 {
      (0.0, x, c)
    } else if h < 300.0 {
      (x, 0.0, c)
    } else {
      (c, 0.0, x)
    };

    Self {
      r: ((r_prime + m) * 255.0) as u8,
      g: ((g_prime + m) * 255.0) as u8,
      b: ((b_prime + m) * 255.0) as u8,
    }
  }

  fn scale(&self, factor: f32) -> Self {
    Self {
      r: ((self.r as f32 * factor).min(255.0)) as u8,
      g: ((self.g as f32 * factor).min(255.0)) as u8,
      b: ((self.b as f32 * factor).min(255.0)) as u8,
    }
  }
}

// LED效果管理器
struct LedEffects {
  current_effect: EffectType,
  brightness: f32,
  speed: u32,
  frame: u32,
  colors: [RGB; 64],
  is_active: bool,
}

#[derive(Clone, Copy)]
enum EffectType {
  Rainbow,
  Breathing,
  Chase,
  Sparkle,
  Fire,
  Wave,
  Static,
}

impl LedEffects {
  fn new() -> Self {
    Self {
      current_effect: EffectType::Rainbow,
      brightness: 0.5,
      speed: 50,
      frame: 0,
      colors: [RGB::new(0, 0, 0); 64],
      is_active: true,
    }
  }

  fn next_effect(&mut self) {
    self.current_effect = match self.current_effect {
      EffectType::Rainbow => EffectType::Breathing,
      EffectType::Breathing => EffectType::Chase,
      EffectType::Chase => EffectType::Sparkle,
      EffectType::Sparkle => EffectType::Fire,
      EffectType::Fire => EffectType::Wave,
      EffectType::Wave => EffectType::Static,
      EffectType::Static => EffectType::Rainbow,
    };
    self.frame = 0;
  }

  fn adjust_brightness(&mut self) {
    self.brightness += 0.2;
    if self.brightness > 1.0 {
      self.brightness = 0.1;
    }
  }

  fn adjust_speed(&mut self) {
    self.speed = match self.speed {
      10 => 25,
      25 => 50,
      50 => 100,
      100 => 200,
      _ => 10,
    };
  }

  fn update(&mut self) {
    match self.current_effect {
      EffectType::Rainbow => self.rainbow_effect(),
      EffectType::Breathing => self.breathing_effect(),
      EffectType::Chase => self.chase_effect(),
      EffectType::Sparkle => self.sparkle_effect(),
      EffectType::Fire => self.fire_effect(),
      EffectType::Wave => self.wave_effect(),
      EffectType::Static => self.static_effect(),
    }

    // 应用亮度
    for color in &mut self.colors {
      *color = color.scale(self.brightness);
    }

    self.frame = self.frame.wrapping_add(1);
  }

  fn rainbow_effect(&mut self) {
    for i in 0..64 {
      let hue = ((self.frame + i as u32 * 4) % 360) as f32;
      self.colors[i] = RGB::from_hsv(hue, 1.0, 1.0);
    }
  }

  fn breathing_effect(&mut self) {
    let brightness = (((self.frame as f32 * 0.1).sin() + 1.0) / 2.0);
    let color = RGB::from_hsv(240.0, 1.0, brightness);
    for i in 0..64 {
      self.colors[i] = color;
    }
  }

  fn chase_effect(&mut self) {
    // 清除所有LED
    for color in &mut self.colors {
      *color = RGB::new(0, 0, 0);
    }

    // 设置追逐点
    let pos = (self.frame / 2) % 64;
    for i in 0..5 {
      let idx = (pos + i) % 64;
      let brightness = 1.0 - (i as f32 * 0.2);
      self.colors[idx as usize] = RGB::from_hsv(120.0, 1.0, brightness);
    }
  }

  fn sparkle_effect(&mut self) {
    // 随机闪烁
    for i in 0..64 {
      if (self.frame + i as u32) % 20 == 0 {
        self.colors[i] = RGB::new(255, 255, 255);
      } else {
        self.colors[i] = self.colors[i].scale(0.9);
      }
    }
  }

  fn fire_effect(&mut self) {
    // 火焰效果
    for i in 0..64 {
      let heat = ((self.frame + i as u32 * 2) % 100) as f32 / 100.0;
      let hue = 30.0 - heat * 30.0; // 从红到黄
      self.colors[i] = RGB::from_hsv(hue, 1.0, heat);
    }
  }

  fn wave_effect(&mut self) {
    for i in 0..64 {
      let wave = ((self.frame as f32 * 0.1 + i as f32 * 0.2).sin() + 1.0) / 2.0;
      self.colors[i] = RGB::from_hsv(180.0, 1.0, wave);
    }
  }

  fn static_effect(&mut self) {
    // 静态颜色
    let color = RGB::new(255, 100, 0); // 橙色
    for i in 0..64 {
      self.colors[i] = color;
    }
  }

  fn get_colors(&self) -> &[RGB] {
    &self.colors
  }

  fn get_delay(&self) -> u32 {
    self.speed
  }

  fn is_active(&self) -> bool {
    self.is_active
  }
}

// 按钮处理器
struct ButtonHandler {
  button_states: [bool; 4],
  last_button_states: [bool; 4],
}

impl ButtonHandler {
  fn new() -> Self {
    Self {
      button_states: [false; 4],
      last_button_states: [false; 4],
    }
  }

  fn update(&mut self, gpio: &GpioManager) {
    self.last_button_states = self.button_states;

    for i in 0..4 {
      self.button_states[i] = gpio.read_button((i + 1) as u8).unwrap_or(false);
    }
  }

  fn is_button_pressed(&self, button: u8) -> bool {
    let idx = (button - 1) as usize;
    if idx >= 4 {
      return false;
    }

    // 检测按钮按下边沿
    self.button_states[idx] && !self.last_button_states[idx]
  }
}

// 数学工具
mod math_utils {
  pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
  }

  pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
      min
    } else if value > max {
      max
    } else {
      value
    }
  }

  pub fn map_range(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
  }
}
