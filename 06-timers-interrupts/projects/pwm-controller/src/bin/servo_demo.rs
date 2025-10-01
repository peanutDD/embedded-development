#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
  gpio::{Input, Pin, PullUp},
  pac,
  prelude::*,
  rcc::Clocks,
  timer::Timer,
};

use pwm_controller::{PwmChannel, ServoController};

/// STM32 PWM通道包装器（简化版）
pub struct Stm32PwmChannel<TIM, const C: u8> {
  pwm: stm32f4xx_hal::pwm::PwmChannel<TIM, C>,
  frequency: u32,
  max_duty: u16,
}

impl<TIM, const C: u8> Stm32PwmChannel<TIM, C>
where
  TIM: pac::timer::Instance,
{
  pub fn new(pwm: stm32f4xx_hal::pwm::PwmChannel<TIM, C>, frequency: u32) -> Self {
    let max_duty = pwm.get_max_duty();
    Self {
      pwm,
      frequency,
      max_duty,
    }
  }
}

impl<TIM, const C: u8> PwmChannel for Stm32PwmChannel<TIM, C>
where
  TIM: pac::timer::Instance,
{
  type Error = ();

  fn set_duty_cycle(&mut self, duty: f32) -> Result<(), Self::Error> {
    let duty_value = (duty * self.max_duty as f32) as u16;
    self.pwm.set_duty(duty_value);
    Ok(())
  }

  fn set_frequency(&mut self, freq: u32) -> Result<(), Self::Error> {
    self.frequency = freq;
    Ok(())
  }

  fn enable(&mut self) -> Result<(), Self::Error> {
    self.pwm.enable();
    Ok(())
  }

  fn disable(&mut self) -> Result<(), Self::Error> {
    self.pwm.disable();
    Ok(())
  }

  fn get_duty_cycle(&self) -> f32 {
    self.pwm.get_duty() as f32 / self.max_duty as f32
  }

  fn get_frequency(&self) -> u32 {
    self.frequency
  }
}

/// 舵机演示模式
#[derive(Debug, Clone, Copy)]
enum ServoDemo {
  Sweep,       // 扫描模式
  Position,    // 位置控制
  Speed,       // 速度控制
  Sequence,    // 序列动作
  Interactive, // 交互控制
}

impl ServoDemo {
  fn next(self) -> Self {
    match self {
      ServoDemo::Sweep => ServoDemo::Position,
      ServoDemo::Position => ServoDemo::Speed,
      ServoDemo::Speed => ServoDemo::Sequence,
      ServoDemo::Sequence => ServoDemo::Interactive,
      ServoDemo::Interactive => ServoDemo::Sweep,
    }
  }

  fn name(self) -> &'static str {
    match self {
      ServoDemo::Sweep => "Sweep Mode",
      ServoDemo::Position => "Position Control",
      ServoDemo::Speed => "Speed Control",
      ServoDemo::Sequence => "Sequence Mode",
      ServoDemo::Interactive => "Interactive Control",
    }
  }
}

/// 舵机序列动作
struct ServoSequence {
  positions: &'static [f32],
  durations: &'static [u32], // 每个位置的持续时间(ms)
  current_step: usize,
  step_timer: u32,
}

impl ServoSequence {
  fn new(positions: &'static [f32], durations: &'static [u32]) -> Self {
    Self {
      positions,
      durations,
      current_step: 0,
      step_timer: 0,
    }
  }

  fn update(&mut self, dt_ms: u32) -> Option<f32> {
    if self.positions.is_empty() {
      return None;
    }

    self.step_timer += dt_ms;

    if self.step_timer >= self.durations[self.current_step] {
      self.step_timer = 0;
      self.current_step = (self.current_step + 1) % self.positions.len();
    }

    Some(self.positions[self.current_step])
  }

  fn reset(&mut self) {
    self.current_step = 0;
    self.step_timer = 0;
  }
}

#[entry]
fn main() -> ! {
  // 初始化RTT调试输出
  rtt_init_print!();
  rprintln!("Servo Control Demo Starting...");

  // 获取外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(8.mhz())
    .sysclk(168.mhz())
    .pclk1(42.mhz())
    .pclk2(84.mhz())
    .freeze();

  rprintln!("System Clock: {} MHz", clocks.sysclk().to_MHz());

  // 配置GPIO引脚
  let gpioa = dp.GPIOA.split();
  let gpioc = dp.GPIOC.split();

  // 舵机PWM引脚 (TIM3)
  let servo1_pin = gpioa.pa6.into_alternate(); // TIM3_CH1
  let servo2_pin = gpioa.pa7.into_alternate(); // TIM3_CH2

  // 按钮引脚
  let button = gpioc.pc13.into_pull_up_input(); // 用户按钮

  // 配置定时器 - 50Hz用于舵机控制
  let tim3 = Timer::new(dp.TIM3, &clocks);
  let pwm3 = tim3.pwm_hz((servo1_pin, servo2_pin), 50.hz(), &clocks);
  let (mut pwm3_ch1, mut pwm3_ch2) = pwm3;

  // 启用PWM输出
  pwm3_ch1.enable();
  pwm3_ch2.enable();

  rprintln!("Servo PWM initialized at 50Hz");

  // 创建舵机控制器
  let stm32_pwm1 = Stm32PwmChannel::new(pwm3_ch1, 50);
  let stm32_pwm2 = Stm32PwmChannel::new(pwm3_ch2, 50);

  let mut servo1 = ServoController::new(stm32_pwm1, 1000, 2000); // SG90舵机 (1-2ms)
  let mut servo2 = ServoController::new(stm32_pwm2, 500, 2500); // MG996R舵机 (0.5-2.5ms)

  // 设置舵机速度
  servo1.move_to_angle(90.0, 90.0).unwrap(); // 90度/秒
  servo2.move_to_angle(90.0, 60.0).unwrap(); // 60度/秒

  rprintln!("Servo controllers initialized");

  // 演示状态
  let mut demo_mode = ServoDemo::Sweep;
  let mut mode_timer = 0u32;
  let mut tick_count = 0u32;
  let mut button_pressed = false;
  let mut last_button_state = false;

  // 扫描模式变量
  let mut sweep_angle = 0.0f32;
  let mut sweep_direction = 1.0f32;

  // 位置控制变量
  let positions = [0.0, 45.0, 90.0, 135.0, 180.0];
  let mut position_index = 0;
  let mut position_timer = 0u32;

  // 速度控制变量
  let mut speed_test_angle = 90.0f32;
  let mut speed_test_speed = 30.0f32; // 开始速度

  // 序列动作
  let sequence_positions = &[90.0, 0.0, 180.0, 90.0, 45.0, 135.0];
  let sequence_durations = &[1000, 1500, 1500, 1000, 2000, 2000]; // ms
  let mut servo_sequence = ServoSequence::new(sequence_positions, sequence_durations);

  rprintln!("Starting demo: {}", demo_mode.name());

  loop {
    tick_count += 1;
    mode_timer += 1;

    // 检测按钮按下（用于切换模式）
    let button_state = button.is_low();
    if button_state && !last_button_state {
      button_pressed = true;
    }
    last_button_state = button_state;

    // 按钮切换模式或自动切换（每15秒）
    if button_pressed || mode_timer >= 15000 {
      demo_mode = demo_mode.next();
      mode_timer = 0;
      button_pressed = false;
      servo_sequence.reset();
      rprintln!("Switching to: {}", demo_mode.name());
    }

    match demo_mode {
      ServoDemo::Sweep => {
        // 扫描模式：两个舵机同步扫描
        sweep_angle += sweep_direction * 0.3; // 0.3度/ms

        if sweep_angle >= 180.0 {
          sweep_angle = 180.0;
          sweep_direction = -1.0;
          rprintln!("Sweep: Reached 180°, reversing direction");
        } else if sweep_angle <= 0.0 {
          sweep_angle = 0.0;
          sweep_direction = 1.0;
          rprintln!("Sweep: Reached 0°, reversing direction");
        }

        servo1.set_angle(sweep_angle).unwrap();
        servo2.set_angle(180.0 - sweep_angle).unwrap(); // 反向运动

        if tick_count % 2000 == 0 {
          rprintln!(
            "Sweep angles: {:.1}°, {:.1}°",
            sweep_angle,
            180.0 - sweep_angle
          );
        }
      }

      ServoDemo::Position => {
        // 位置控制：定点移动
        position_timer += 1;

        if position_timer >= 3000 {
          // 每3秒切换位置
          position_index = (position_index + 1) % positions.len();
          position_timer = 0;

          let target_angle = positions[position_index];
          servo1.set_angle(target_angle).unwrap();
          servo2.set_angle(target_angle).unwrap();

          rprintln!("Moving to position: {:.0}°", target_angle);
        }
      }

      ServoDemo::Speed => {
        // 速度控制测试
        if mode_timer % 5000 == 0 && mode_timer > 0 {
          // 每5秒改变速度和目标位置
          speed_test_angle = if speed_test_angle == 90.0 { 0.0 } else { 90.0 };
          speed_test_speed += 30.0;
          if speed_test_speed > 180.0 {
            speed_test_speed = 30.0;
          }

          servo1
            .move_to_angle(speed_test_angle, speed_test_speed)
            .unwrap();
          servo2
            .move_to_angle(180.0 - speed_test_angle, speed_test_speed * 0.8)
            .unwrap();

          rprintln!(
            "Speed test: {:.0}°/s to {:.0}°",
            speed_test_speed,
            speed_test_angle
          );
        }
      }

      ServoDemo::Sequence => {
        // 序列动作演示
        if let Some(target_angle) = servo_sequence.update(1) {
          servo1.set_angle(target_angle).unwrap();
          servo2.set_angle(target_angle * 0.8).unwrap(); // 第二个舵机80%角度

          if mode_timer % 1000 == 0 {
            rprintln!("Sequence step: {:.0}°", target_angle);
          }
        }
      }

      ServoDemo::Interactive => {
        // 交互控制：基于时间的正弦波运动
        let time_s = mode_timer as f32 / 1000.0;
        let angle1 = 90.0 + 60.0 * libm::sin(time_s * 0.5); // 0.5Hz
        let angle2 = 90.0 + 45.0 * libm::cos(time_s * 0.3); // 0.3Hz

        servo1.set_angle(angle1).unwrap();
        servo2.set_angle(angle2).unwrap();

        if tick_count % 2000 == 0 {
          rprintln!("Interactive: {:.1}°, {:.1}°", angle1, angle2);
        }
      }
    }

    // 更新舵机状态
    servo1.update(1).unwrap(); // 1ms更新间隔
    servo2.update(1).unwrap();

    // 状态报告
    if tick_count % 10000 == 0 {
      rprintln!(
        "System status - Tick: {}, Mode: {}",
        tick_count,
        demo_mode.name()
      );
      rprintln!(
        "Servo1: current={:.1}°, target={:.1}°",
        servo1.get_current_angle(),
        servo1.get_target_angle()
      );
      rprintln!(
        "Servo2: current={:.1}°, target={:.1}°",
        servo2.get_current_angle(),
        servo2.get_target_angle()
      );
    }

    // 错误检测和恢复
    if tick_count % 30000 == 0 {
      // 每30秒进行一次自检
      rprintln!("Performing servo health check...");

      // 检查舵机是否响应
      let test_angle = 90.0;
      servo1.set_angle(test_angle).unwrap();
      servo2.set_angle(test_angle).unwrap();

      // 等待一段时间让舵机移动
      for _ in 0..1000 {
        servo1.update(1).unwrap();
        servo2.update(1).unwrap();
        asm::delay(168_000); // 1ms延时
      }

      rprintln!("Health check completed");
    }

    // 1ms延时
    asm::delay(168_000); // 168MHz系统时钟，1ms延时
  }
}
