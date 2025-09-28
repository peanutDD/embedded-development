#![no_std]
#![no_main]

use cortex_m_rt::entry;
use micromath::F32Ext;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
  gpio::{gpioa::*, Alternate},
  pac,
  prelude::*,
  rcc::Clocks,
  timer::{Channel, PwmChannels, Timer},
};

// PWM频率设置
const PWM_FREQUENCY: u32 = 1000; // 1kHz
const BREATHING_PERIOD_MS: u32 = 2000; // 2秒完整呼吸周期
const STEPS_PER_CYCLE: u32 = 100; // 每个周期的步数

// PWM通道类型定义
type PwmPin = PA8<Alternate<1>>;
type PwmTimer = Timer<pac::TIM1>;

#[entry]
fn main() -> ! {
  // 初始化RTT调试输出
  rtt_init_print!();
  rprintln!("PWM呼吸灯效果启动");

  // 获取外设访问权限
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(25.mhz())
    .sysclk(168.mhz())
    .pclk1(42.mhz())
    .pclk2(84.mhz())
    .freeze();

  rprintln!("系统时钟配置完成: SYSCLK={}MHz", clocks.sysclk().to_MHz());

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let pwm_pin = gpioa.pa8.into_alternate();

  // 配置PWM定时器
  let mut pwm_timer = Timer::new(dp.TIM1, &clocks);
  let mut pwm_channels = pwm_timer.pwm_hz(pwm_pin, PWM_FREQUENCY.hz());

  // 获取PWM通道
  let mut pwm_channel = pwm_channels.0;
  let max_duty = pwm_channel.get_max_duty();

  rprintln!(
    "PWM配置完成: 频率={}Hz, 最大占空比={}",
    PWM_FREQUENCY,
    max_duty
  );

  // 启用PWM输出
  pwm_channel.enable();

  // 配置系统定时器用于延时
  let mut delay = Timer::new(dp.TIM2, &clocks).delay_ms();

  // 呼吸灯控制器
  let mut breathing_controller =
    BreathingController::new(max_duty, BREATHING_PERIOD_MS, STEPS_PER_CYCLE);

  rprintln!("开始呼吸灯循环");

  let mut cycle_count = 0u32;

  loop {
    // 更新呼吸灯状态
    let duty_cycle = breathing_controller.update();
    pwm_channel.set_duty(duty_cycle);

    // 输出调试信息（每10步输出一次）
    if breathing_controller.step % 10 == 0 {
      let brightness_percent = (duty_cycle * 100) / max_duty;
      rprintln!(
        "周期: {}, 步骤: {}/{}, 占空比: {}/{} ({}%)",
        cycle_count,
        breathing_controller.step,
        STEPS_PER_CYCLE,
        duty_cycle,
        max_duty,
        brightness_percent
      );
    }

    // 检查是否完成一个完整周期
    if breathing_controller.is_cycle_complete() {
      cycle_count += 1;
      rprintln!("完成第{}个呼吸周期", cycle_count);
    }

    // 延时
    delay.delay_ms(BREATHING_PERIOD_MS / STEPS_PER_CYCLE);
  }
}

/// 呼吸灯控制器
struct BreathingController {
  max_duty: u16,
  period_ms: u32,
  steps_per_cycle: u32,
  step: u32,
  direction: BreathingDirection,
  waveform: BreathingWaveform,
}

#[derive(Clone, Copy)]
enum BreathingDirection {
  Increasing,
  Decreasing,
}

#[derive(Clone, Copy)]
enum BreathingWaveform {
  Linear,
  Sine,
  Exponential,
}

impl BreathingController {
  fn new(max_duty: u16, period_ms: u32, steps_per_cycle: u32) -> Self {
    Self {
      max_duty,
      period_ms,
      steps_per_cycle,
      step: 0,
      direction: BreathingDirection::Increasing,
      waveform: BreathingWaveform::Sine,
    }
  }

  fn update(&mut self) -> u16 {
    let duty_cycle = match self.waveform {
      BreathingWaveform::Linear => self.calculate_linear_duty(),
      BreathingWaveform::Sine => self.calculate_sine_duty(),
      BreathingWaveform::Exponential => self.calculate_exponential_duty(),
    };

    self.advance_step();
    duty_cycle
  }

  fn calculate_linear_duty(&self) -> u16 {
    let progress = self.step as f32 / (self.steps_per_cycle / 2) as f32;
    let normalized_progress = match self.direction {
      BreathingDirection::Increasing => progress,
      BreathingDirection::Decreasing => 2.0 - progress,
    };

    let clamped_progress = normalized_progress.max(0.0).min(1.0);
    (clamped_progress * self.max_duty as f32) as u16
  }

  fn calculate_sine_duty(&self) -> u16 {
    // 使用正弦波形实现平滑的呼吸效果
    let angle = (self.step as f32 / self.steps_per_cycle as f32) * 2.0 * core::f32::consts::PI;
    let sine_value = (angle.sin() + 1.0) / 2.0; // 将-1到1映射到0到1
    (sine_value * self.max_duty as f32) as u16
  }

  fn calculate_exponential_duty(&self) -> u16 {
    let progress = self.step as f32 / (self.steps_per_cycle / 2) as f32;
    let normalized_progress = match self.direction {
      BreathingDirection::Increasing => progress,
      BreathingDirection::Decreasing => 2.0 - progress,
    };

    let clamped_progress = normalized_progress.max(0.0).min(1.0);
    // 使用指数函数创建更自然的呼吸效果
    let exponential_value = (clamped_progress * clamped_progress);
    (exponential_value * self.max_duty as f32) as u16
  }

  fn advance_step(&mut self) {
    self.step += 1;

    // 检查方向切换
    match self.direction {
      BreathingDirection::Increasing => {
        if self.step >= self.steps_per_cycle / 2 {
          self.direction = BreathingDirection::Decreasing;
        }
      }
      BreathingDirection::Decreasing => {
        if self.step >= self.steps_per_cycle {
          self.step = 0;
          self.direction = BreathingDirection::Increasing;
        }
      }
    }
  }

  fn is_cycle_complete(&self) -> bool {
    self.step == 0 && matches!(self.direction, BreathingDirection::Increasing)
  }

  fn set_waveform(&mut self, waveform: BreathingWaveform) {
    self.waveform = waveform;
  }

  fn get_brightness_percentage(&self) -> u8 {
    let current_duty = match self.waveform {
      BreathingWaveform::Linear => self.calculate_linear_duty(),
      BreathingWaveform::Sine => self.calculate_sine_duty(),
      BreathingWaveform::Exponential => self.calculate_exponential_duty(),
    };
    ((current_duty as u32 * 100) / self.max_duty as u32) as u8
  }
}

/// 高级呼吸灯控制器，支持多种效果
struct AdvancedBreathingController {
  controllers: [BreathingController; 3], // RGB三通道
  current_effect: BreathingEffect,
  effect_timer: u32,
}

#[derive(Clone, Copy)]
enum BreathingEffect {
  SingleColor, // 单色呼吸
  ColorCycle,  // 颜色循环
  Rainbow,     // 彩虹效果
  Pulse,       // 脉冲效果
}

impl AdvancedBreathingController {
  fn new(max_duty: u16) -> Self {
    Self {
      controllers: [
        BreathingController::new(max_duty, 2000, 100), // Red
        BreathingController::new(max_duty, 2200, 100), // Green
        BreathingController::new(max_duty, 2400, 100), // Blue
      ],
      current_effect: BreathingEffect::SingleColor,
      effect_timer: 0,
    }
  }

  fn update(&mut self) -> (u16, u16, u16) {
    self.effect_timer += 1;

    match self.current_effect {
      BreathingEffect::SingleColor => {
        let duty = self.controllers[0].update();
        (duty, 0, 0) // 只有红色
      }
      BreathingEffect::ColorCycle => {
        // 循环切换颜色
        let cycle_phase = (self.effect_timer / 200) % 3;
        match cycle_phase {
          0 => (self.controllers[0].update(), 0, 0), // Red
          1 => (0, self.controllers[1].update(), 0), // Green
          _ => (0, 0, self.controllers[2].update()), // Blue
        }
      }
      BreathingEffect::Rainbow => {
        // 彩虹效果 - 所有颜色同时变化但相位不同
        (
          self.controllers[0].update(),
          self.controllers[1].update(),
          self.controllers[2].update(),
        )
      }
      BreathingEffect::Pulse => {
        // 脉冲效果 - 快速闪烁
        if (self.effect_timer % 20) < 10 {
          (self.controllers[0].max_duty, 0, 0)
        } else {
          (0, 0, 0)
        }
      }
    }
  }

  fn switch_effect(&mut self) {
    self.current_effect = match self.current_effect {
      BreathingEffect::SingleColor => BreathingEffect::ColorCycle,
      BreathingEffect::ColorCycle => BreathingEffect::Rainbow,
      BreathingEffect::Rainbow => BreathingEffect::Pulse,
      BreathingEffect::Pulse => BreathingEffect::SingleColor,
    };
    self.effect_timer = 0;
  }
}

/// PWM配置辅助函数
mod pwm_utils {
  use super::*;

  pub fn calculate_optimal_prescaler(
    timer_clock: u32,
    desired_frequency: u32,
    max_resolution: u16,
  ) -> (u16, u16) {
    // 计算最优的预分频器和自动重载值
    let target_period = timer_clock / desired_frequency;

    for prescaler in 1..=65535u32 {
      let arr = target_period / prescaler;
      if arr <= max_resolution as u32 && arr > 0 {
        return ((prescaler - 1) as u16, (arr - 1) as u16);
      }
    }

    // 如果找不到合适的值，返回默认值
    (0, 999)
  }

  pub fn duty_to_percentage(duty: u16, max_duty: u16) -> u8 {
    ((duty as u32 * 100) / max_duty as u32) as u8
  }

  pub fn percentage_to_duty(percentage: u8, max_duty: u16) -> u16 {
    ((percentage as u32 * max_duty as u32) / 100) as u16
  }
}

/// 测试模块
#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_breathing_controller() {
    let mut controller = BreathingController::new(1000, 2000, 100);

    // 测试初始状态
    assert_eq!(controller.step, 0);
    assert!(matches!(
      controller.direction,
      BreathingDirection::Increasing
    ));

    // 测试更新
    let duty1 = controller.update();
    assert!(duty1 > 0);
    assert_eq!(controller.step, 1);
  }

  #[test]
  fn test_pwm_utils() {
    let (prescaler, arr) = pwm_utils::calculate_optimal_prescaler(84_000_000, 1000, 1000);
    assert!(prescaler > 0);
    assert!(arr > 0);

    let percentage = pwm_utils::duty_to_percentage(500, 1000);
    assert_eq!(percentage, 50);

    let duty = pwm_utils::percentage_to_duty(75, 1000);
    assert_eq!(duty, 750);
  }
}
