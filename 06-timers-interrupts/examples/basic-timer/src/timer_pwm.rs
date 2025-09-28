#![no_std]
#![no_main]

use cortex_m_rt::entry;
use micromath::F32Ext;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::gpioc::PC13,
  gpio::{
    gpioa::{PA10, PA11, PA8, PA9},
    Alternate, AF1,
  },
  gpio::{
    gpiob::{PB0, PB1, PB2, PB3},
    Input, Output, PullUp, PushPull,
  },
  prelude::*,
  pwm::PwmChannel,
  stm32,
  timer::{Channel, PwmChannels, Timer},
};

type PwmPin1 = PA8<Alternate<AF1>>;
type PwmPin2 = PA9<Alternate<AF1>>;
type PwmPin3 = PA10<Alternate<AF1>>;
type PwmPin4 = PA11<Alternate<AF1>>;

type StatusLed = PB0<Output<PushPull>>;
type Channel1Led = PB1<Output<PushPull>>;
type Channel2Led = PB2<Output<PushPull>>;
type Channel3Led = PB3<Output<PushPull>>;
type ModeButton = PC13<Input<PullUp>>;

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // PWM输出引脚
  let pwm_pins = (
    gpioa.pa8.into_alternate_af1(),
    gpioa.pa9.into_alternate_af1(),
    gpioa.pa10.into_alternate_af1(),
    gpioa.pa11.into_alternate_af1(),
  );

  // LED指示灯
  let mut status_led = gpiob.pb0.into_push_pull_output();
  let mut channel1_led = gpiob.pb1.into_push_pull_output();
  let mut channel2_led = gpiob.pb2.into_push_pull_output();
  let mut channel3_led = gpiob.pb3.into_push_pull_output();
  let mode_button = gpioc.pc13.into_pull_up_input();

  // 配置PWM定时器
  let mut pwm = Timer::tim1(dp.TIM1, &clocks).pwm(pwm_pins, 1.khz());
  let max_duty = pwm.get_max_duty();

  // 启用PWM通道
  pwm.enable(Channel::C1);
  pwm.enable(Channel::C2);
  pwm.enable(Channel::C3);
  pwm.enable(Channel::C4);

  // 配置延时定时器
  let mut delay = Timer::tim2(dp.TIM2, &clocks).delay_ms();

  // 系统启动指示
  status_led.set_high();
  delay.delay_ms(1000u32);

  // PWM控制器
  let mut pwm_controller = PwmController::new(max_duty);
  let mut button_debouncer = ButtonDebouncer::new();
  let mut pwm_mode = PwmMode::Static;
  let mut demo_counter = 0u32;

  loop {
    // 按钮防抖处理
    let button_pressed = button_debouncer.update(mode_button.is_low());

    if button_pressed {
      // 切换PWM模式
      pwm_mode = match pwm_mode {
        PwmMode::Static => PwmMode::Breathing,
        PwmMode::Breathing => PwmMode::Wave,
        PwmMode::Wave => PwmMode::Rainbow,
        PwmMode::Rainbow => PwmMode::Static,
      };

      // 模式切换指示
      for _ in 0..3 {
        status_led.set_low();
        delay.delay_ms(100u32);
        status_led.set_high();
        delay.delay_ms(100u32);
      }
    }

    // 更新PWM输出
    let pwm_values = pwm_controller.update(pwm_mode, demo_counter);

    pwm.set_duty(Channel::C1, pwm_values.channel1);
    pwm.set_duty(Channel::C2, pwm_values.channel2);
    pwm.set_duty(Channel::C3, pwm_values.channel3);
    pwm.set_duty(Channel::C4, pwm_values.channel4);

    // 更新LED指示
    update_channel_leds(
      &pwm_values,
      max_duty,
      &mut channel1_led,
      &mut channel2_led,
      &mut channel3_led,
    );

    demo_counter += 1;

    // 状态LED心跳
    if demo_counter % 100 == 0 {
      status_led.toggle();
    }

    delay.delay_ms(20u32);
  }
}

/// 更新通道LED指示
fn update_channel_leds(
  pwm_values: &PwmValues,
  max_duty: u16,
  channel1_led: &mut impl embedded_hal::digital::v2::OutputPin,
  channel2_led: &mut impl embedded_hal::digital::v2::OutputPin,
  channel3_led: &mut impl embedded_hal::digital::v2::OutputPin,
) {
  let threshold = max_duty / 2;

  channel1_led
    .set_state((pwm_values.channel1 > threshold).into())
    .ok();
  channel2_led
    .set_state((pwm_values.channel2 > threshold).into())
    .ok();
  channel3_led
    .set_state((pwm_values.channel3 > threshold).into())
    .ok();
}

/// PWM控制器
struct PwmController {
  max_duty: u16,
  phase_offset: f32,
}

impl PwmController {
  fn new(max_duty: u16) -> Self {
    Self {
      max_duty,
      phase_offset: 0.0,
    }
  }

  fn update(&mut self, mode: PwmMode, counter: u32) -> PwmValues {
    match mode {
      PwmMode::Static => self.static_mode(),
      PwmMode::Breathing => self.breathing_mode(counter),
      PwmMode::Wave => self.wave_mode(counter),
      PwmMode::Rainbow => self.rainbow_mode(counter),
    }
  }

  fn static_mode(&self) -> PwmValues {
    PwmValues {
      channel1: self.max_duty / 4,
      channel2: self.max_duty / 2,
      channel3: (self.max_duty * 3) / 4,
      channel4: self.max_duty,
    }
  }

  fn breathing_mode(&self, counter: u32) -> PwmValues {
    let phase = (counter as f32 * 0.05).sin();
    let duty = ((phase + 1.0) * 0.5 * self.max_duty as f32) as u16;

    PwmValues {
      channel1: duty,
      channel2: duty,
      channel3: duty,
      channel4: duty,
    }
  }

  fn wave_mode(&self, counter: u32) -> PwmValues {
    let base_phase = counter as f32 * 0.1;

    let duty1 = ((base_phase.sin() + 1.0) * 0.5 * self.max_duty as f32) as u16;
    let duty2 = (((base_phase + 1.57).sin() + 1.0) * 0.5 * self.max_duty as f32) as u16; // π/2偏移
    let duty3 = (((base_phase + 3.14).sin() + 1.0) * 0.5 * self.max_duty as f32) as u16; // π偏移
    let duty4 = (((base_phase + 4.71).sin() + 1.0) * 0.5 * self.max_duty as f32) as u16; // 3π/2偏移

    PwmValues {
      channel1: duty1,
      channel2: duty2,
      channel3: duty3,
      channel4: duty4,
    }
  }

  fn rainbow_mode(&self, counter: u32) -> PwmValues {
    let hue = (counter as f32 * 0.02) % (2.0 * 3.14159);

    // HSV到RGB转换的简化版本
    let r = ((hue.sin() + 1.0) * 0.5 * self.max_duty as f32) as u16;
    let g = (((hue + 2.09).sin() + 1.0) * 0.5 * self.max_duty as f32) as u16; // 2π/3偏移
    let b = (((hue + 4.19).sin() + 1.0) * 0.5 * self.max_duty as f32) as u16; // 4π/3偏移
    let w = (self.max_duty as f32 * 0.3) as u16; // 白色分量

    PwmValues {
      channel1: r,
      channel2: g,
      channel3: b,
      channel4: w,
    }
  }
}

/// PWM输出值
#[derive(Clone, Copy)]
struct PwmValues {
  channel1: u16,
  channel2: u16,
  channel3: u16,
  channel4: u16,
}

/// PWM模式
#[derive(Clone, Copy)]
enum PwmMode {
  Static,    // 静态输出
  Breathing, // 呼吸灯效果
  Wave,      // 波浪效果
  Rainbow,   // 彩虹效果
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
