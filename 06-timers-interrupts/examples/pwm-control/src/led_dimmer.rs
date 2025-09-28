#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{gpioa::PA8, Alternate, AF1},
  prelude::*,
  stm32,
  timer::{Channel, PwmChannels, Timer},
};

type LedPin = PA8<Alternate<AF1>>;

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
  let led_pin = gpioa.pa8.into_alternate_af1();

  // 配置定时器1用于PWM
  let mut timer = Timer::tim1(dp.TIM1, &clocks);
  let mut pwm = timer.pwm::<Timer1Ch1>(led_pin, 1.khz());

  // 启用PWM通道
  pwm.enable(Channel::C1);

  // 获取最大占空比
  let max_duty = pwm.get_max_duty();

  // 配置系统定时器用于延时
  let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);

  // LED调光主循环
  loop {
    // 渐亮
    for brightness in 0..=100 {
      let duty = (max_duty * brightness) / 100;
      pwm.set_duty(Channel::C1, duty);
      delay.delay_ms(20u32);
    }

    // 渐暗
    for brightness in (0..=100).rev() {
      let duty = (max_duty * brightness) / 100;
      pwm.set_duty(Channel::C1, duty);
      delay.delay_ms(20u32);
    }
  }
}

/// PWM LED控制器结构体
pub struct PwmLedController<PIN> {
  pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, PIN>,
  max_duty: u16,
}

impl<PIN> PwmLedController<PIN> {
  /// 创建新的LED控制器
  pub fn new(mut pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, PIN>) -> Self {
    pwm.enable(Channel::C1);
    let max_duty = pwm.get_max_duty();

    Self { pwm, max_duty }
  }

  /// 设置亮度 (0-100%)
  pub fn set_brightness(&mut self, brightness: u8) {
    let brightness = brightness.min(100);
    let duty = (self.max_duty * brightness as u16) / 100;
    self.pwm.set_duty(Channel::C1, duty);
  }

  /// 渐变到指定亮度
  pub fn fade_to(&mut self, target_brightness: u8, steps: u16, delay_ms: u32) {
    let current = self.get_current_brightness();
    let target = target_brightness.min(100);

    if current == target {
      return;
    }

    let step_size = if current > target {
      (current - target) as i16 / steps as i16
    } else {
      (target - current) as i16 / steps as i16
    };

    for step in 0..steps {
      let new_brightness = if current > target {
        current - ((step_size * step as i16) as u8)
      } else {
        current + ((step_size * step as i16) as u8)
      };

      self.set_brightness(new_brightness);
      // 这里需要延时，实际应用中可以使用定时器中断
    }

    self.set_brightness(target);
  }

  /// 获取当前亮度
  pub fn get_current_brightness(&self) -> u8 {
    let current_duty = self.pwm.get_duty(Channel::C1);
    ((current_duty * 100) / self.max_duty) as u8
  }

  /// 呼吸灯效果
  pub fn breathing_effect(&mut self, min_brightness: u8, max_brightness: u8, cycle_ms: u32) {
    let min = min_brightness.min(100);
    let max = max_brightness.min(100);
    let steps = 50;
    let step_delay = cycle_ms / (steps * 2);

    // 渐亮
    self.fade_to(max, steps, step_delay);
    // 渐暗
    self.fade_to(min, steps, step_delay);
  }
}
