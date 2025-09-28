#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_halt as _;

use stm32f4xx_hal::{
  gpio::{Input, Output, Pin, PullUp, PushPull},
  pac,
  prelude::*,
  rcc::Clocks,
  timer::{Channel, PwmChannel, Timer},
};

use heapless::Vec;
use libm::{cosf, fabsf, sinf};

// 舵机控制器
pub struct ServoController<T, const C: u8> {
  pwm_channel: PwmChannel<T, C>,
  min_pulse_us: u16,
  max_pulse_us: u16,
  max_angle: f32,
  current_angle: f32,
  target_angle: f32,
  speed: f32, // 度/秒
}

impl<T, const C: u8> ServoController<T, C>
where
  PwmChannel<T, C>: embedded_hal::pwm::SetDutyCycle,
{
  pub fn new(
    pwm_channel: PwmChannel<T, C>,
    min_pulse_us: u16,
    max_pulse_us: u16,
    max_angle: f32,
  ) -> Self {
    Self {
      pwm_channel,
      min_pulse_us,
      max_pulse_us,
      max_angle,
      current_angle: 0.0,
      target_angle: 0.0,
      speed: 90.0, // 默认90度/秒
    }
  }

  pub fn set_angle(&mut self, angle: f32) {
    let clamped_angle = angle.max(0.0).min(self.max_angle);
    self.target_angle = clamped_angle;

    // 计算PWM占空比
    let pulse_range = self.max_pulse_us - self.min_pulse_us;
    let pulse_us =
      self.min_pulse_us + ((clamped_angle / self.max_angle) * pulse_range as f32) as u16;

    // PWM周期为20ms (50Hz)，占空比 = pulse_us / 20000
    let duty_cycle = (pulse_us as u32 * 65535) / 20000;
    let _ = self.pwm_channel.set_duty_cycle(duty_cycle as u16);

    self.current_angle = clamped_angle;
    defmt::info!(
      "舵机角度设置为: {:.1}°, 脉宽: {}μs",
      clamped_angle,
      pulse_us
    );
  }

  pub fn set_speed(&mut self, speed: f32) {
    self.speed = speed.max(1.0).min(360.0);
  }

  pub fn smooth_move_to(&mut self, target_angle: f32, dt_ms: u32) {
    self.target_angle = target_angle.max(0.0).min(self.max_angle);

    let angle_diff = self.target_angle - self.current_angle;
    let max_step = (self.speed * dt_ms as f32) / 1000.0;

    if fabsf(angle_diff) <= max_step {
      self.set_angle(self.target_angle);
    } else {
      let step = if angle_diff > 0.0 {
        max_step
      } else {
        -max_step
      };
      self.set_angle(self.current_angle + step);
    }
  }

  pub fn get_current_angle(&self) -> f32 {
    self.current_angle
  }

  pub fn is_at_target(&self) -> bool {
    fabsf(self.current_angle - self.target_angle) < 1.0
  }
}

// 多舵机控制器
pub struct MultiServoController {
  servos: Vec<ServoInfo, 8>,
  sequence_index: usize,
  sequence_timer: u32,
}

#[derive(Clone, Copy)]
pub struct ServoInfo {
  pub id: u8,
  pub current_angle: f32,
  pub target_angle: f32,
  pub speed: f32,
}

impl MultiServoController {
  pub fn new() -> Self {
    Self {
      servos: Vec::new(),
      sequence_index: 0,
      sequence_timer: 0,
    }
  }

  pub fn add_servo(&mut self, id: u8, initial_angle: f32, speed: f32) -> Result<(), ()> {
    if self.servos.len() >= 8 {
      return Err(());
    }

    let servo_info = ServoInfo {
      id,
      current_angle: initial_angle,
      target_angle: initial_angle,
      speed,
    };

    self.servos.push(servo_info).map_err(|_| ())
  }

  pub fn set_servo_angle(&mut self, id: u8, angle: f32) {
    for servo in self.servos.iter_mut() {
      if servo.id == id {
        servo.target_angle = angle;
        break;
      }
    }
  }

  pub fn update(&mut self, dt_ms: u32) {
    for servo in self.servos.iter_mut() {
      let angle_diff = servo.target_angle - servo.current_angle;
      let max_step = (servo.speed * dt_ms as f32) / 1000.0;

      if fabsf(angle_diff) <= max_step {
        servo.current_angle = servo.target_angle;
      } else {
        let step = if angle_diff > 0.0 {
          max_step
        } else {
          -max_step
        };
        servo.current_angle += step;
      }
    }
  }
}

// 舵机动作序列
pub struct ServoSequence {
  steps: Vec<SequenceStep, 16>,
  current_step: usize,
  step_timer: u32,
  is_running: bool,
  loop_sequence: bool,
}

#[derive(Clone, Copy)]
pub struct SequenceStep {
  pub servo_id: u8,
  pub angle: f32,
  pub duration_ms: u32,
  pub delay_ms: u32,
}

impl ServoSequence {
  pub fn new() -> Self {
    Self {
      steps: Vec::new(),
      current_step: 0,
      step_timer: 0,
      is_running: false,
      loop_sequence: false,
    }
  }

  pub fn add_step(
    &mut self,
    servo_id: u8,
    angle: f32,
    duration_ms: u32,
    delay_ms: u32,
  ) -> Result<(), ()> {
    let step = SequenceStep {
      servo_id,
      angle,
      duration_ms,
      delay_ms,
    };
    self.steps.push(step).map_err(|_| ())
  }

  pub fn start(&mut self, loop_sequence: bool) {
    self.current_step = 0;
    self.step_timer = 0;
    self.is_running = true;
    self.loop_sequence = loop_sequence;
    defmt::info!("开始执行舵机序列，共{}步", self.steps.len());
  }

  pub fn stop(&mut self) {
    self.is_running = false;
    defmt::info!("停止舵机序列");
  }

  pub fn update(&mut self, dt_ms: u32, multi_servo: &mut MultiServoController) {
    if !self.is_running || self.steps.is_empty() {
      return;
    }

    self.step_timer += dt_ms;

    if self.current_step < self.steps.len() {
      let step = self.steps[self.current_step];

      if self.step_timer >= step.delay_ms {
        multi_servo.set_servo_angle(step.servo_id, step.angle);

        if self.step_timer >= step.delay_ms + step.duration_ms {
          self.current_step += 1;
          self.step_timer = 0;

          if self.current_step >= self.steps.len() {
            if self.loop_sequence {
              self.current_step = 0;
              defmt::info!("舵机序列循环重启");
            } else {
              self.is_running = false;
              defmt::info!("舵机序列执行完成");
            }
          }
        }
      }
    }
  }

  pub fn is_running(&self) -> bool {
    self.is_running
  }
}

// 按钮管理器
pub struct ButtonManager {
  mode_button: Pin<'A', 0, Input<PullUp>>,
  angle_button: Pin<'A', 1, Input<PullUp>>,
  speed_button: Pin<'A', 2, Input<PullUp>>,
  sequence_button: Pin<'A', 3, Input<PullUp>>,
  last_mode_state: bool,
  last_angle_state: bool,
  last_speed_state: bool,
  last_sequence_state: bool,
}

impl ButtonManager {
  pub fn new(
    mode_button: Pin<'A', 0, Input<PullUp>>,
    angle_button: Pin<'A', 1, Input<PullUp>>,
    speed_button: Pin<'A', 2, Input<PullUp>>,
    sequence_button: Pin<'A', 3, Input<PullUp>>,
  ) -> Self {
    Self {
      mode_button,
      angle_button,
      speed_button,
      sequence_button,
      last_mode_state: true,
      last_angle_state: true,
      last_speed_state: true,
      last_sequence_state: true,
    }
  }

  pub fn update(&mut self) -> ButtonEvents {
    let mut events = ButtonEvents::default();

    let mode_pressed = self.mode_button.is_low();
    let angle_pressed = self.angle_button.is_low();
    let speed_pressed = self.speed_button.is_low();
    let sequence_pressed = self.sequence_button.is_low();

    if mode_pressed && self.last_mode_state {
      events.mode_pressed = true;
    }
    if angle_pressed && self.last_angle_state {
      events.angle_pressed = true;
    }
    if speed_pressed && self.last_speed_state {
      events.speed_pressed = true;
    }
    if sequence_pressed && self.last_sequence_state {
      events.sequence_pressed = true;
    }

    self.last_mode_state = !mode_pressed;
    self.last_angle_state = !angle_pressed;
    self.last_speed_state = !speed_pressed;
    self.last_sequence_state = !sequence_pressed;

    events
  }
}

#[derive(Default)]
pub struct ButtonEvents {
  pub mode_pressed: bool,
  pub angle_pressed: bool,
  pub speed_pressed: bool,
  pub sequence_pressed: bool,
}

// 系统状态管理
#[derive(Clone, Copy, PartialEq)]
pub enum SystemMode {
  Manual,      // 手动控制
  Sweep,       // 扫描模式
  Sequence,    // 序列模式
  Calibration, // 校准模式
}

pub struct SystemState {
  mode: SystemMode,
  manual_angle: f32,
  sweep_amplitude: f32,
  sweep_frequency: f32,
  sweep_center: f32,
  sweep_time: f32,
  speed_level: u8,
  status_led: Pin<'C', 13, Output<PushPull>>,
}

impl SystemState {
  pub fn new(status_led: Pin<'C', 13, Output<PushPull>>) -> Self {
    Self {
      mode: SystemMode::Manual,
      manual_angle: 90.0,
      sweep_amplitude: 45.0,
      sweep_frequency: 0.5,
      sweep_center: 90.0,
      sweep_time: 0.0,
      speed_level: 2,
      status_led,
    }
  }

  pub fn handle_button_events(&mut self, events: ButtonEvents) {
    if events.mode_pressed {
      self.mode = match self.mode {
        SystemMode::Manual => SystemMode::Sweep,
        SystemMode::Sweep => SystemMode::Sequence,
        SystemMode::Sequence => SystemMode::Calibration,
        SystemMode::Calibration => SystemMode::Manual,
      };
      defmt::info!("切换到模式: {:?}", self.mode);
    }

    if events.angle_pressed {
      match self.mode {
        SystemMode::Manual => {
          self.manual_angle += 30.0;
          if self.manual_angle > 180.0 {
            self.manual_angle = 0.0;
          }
          defmt::info!("手动角度: {:.1}°", self.manual_angle);
        }
        SystemMode::Sweep => {
          self.sweep_amplitude += 15.0;
          if self.sweep_amplitude > 90.0 {
            self.sweep_amplitude = 15.0;
          }
          defmt::info!("扫描幅度: {:.1}°", self.sweep_amplitude);
        }
        _ => {}
      }
    }

    if events.speed_pressed {
      self.speed_level = (self.speed_level % 5) + 1;
      defmt::info!("速度等级: {}", self.speed_level);
    }
  }

  pub fn update(&mut self, dt_ms: u32) -> f32 {
    // 更新状态LED
    match self.mode {
      SystemMode::Manual => self.status_led.set_high(),
      SystemMode::Sweep => {
        // 闪烁指示扫描模式
        if (self.sweep_time * 4.0) as u32 % 2 == 0 {
          self.status_led.set_high();
        } else {
          self.status_led.set_low();
        }
      }
      SystemMode::Sequence => self.status_led.set_low(),
      SystemMode::Calibration => {
        // 快速闪烁指示校准模式
        if (self.sweep_time * 10.0) as u32 % 2 == 0 {
          self.status_led.set_high();
        } else {
          self.status_led.set_low();
        }
      }
    }

    // 计算目标角度
    match self.mode {
      SystemMode::Manual => self.manual_angle,
      SystemMode::Sweep => {
        self.sweep_time += dt_ms as f32 / 1000.0;
        let angle_offset =
          self.sweep_amplitude * sinf(2.0 * 3.14159 * self.sweep_frequency * self.sweep_time);
        self.sweep_center + angle_offset
      }
      SystemMode::Sequence => 90.0, // 序列模式由序列控制器管理
      SystemMode::Calibration => {
        // 校准模式：在0-180度之间缓慢移动
        self.sweep_time += dt_ms as f32 / 1000.0;
        90.0 + 90.0 * sinf(0.2 * self.sweep_time)
      }
    }
  }

  pub fn get_speed(&self) -> f32 {
    match self.speed_level {
      1 => 30.0,
      2 => 60.0,
      3 => 90.0,
      4 => 120.0,
      5 => 180.0,
      _ => 90.0,
    }
  }

  pub fn get_mode(&self) -> SystemMode {
    self.mode
  }
}

// 性能统计
pub struct PerformanceStats {
  update_count: u32,
  total_update_time: u32,
  max_update_time: u32,
  last_stats_time: u32,
}

impl PerformanceStats {
  pub fn new() -> Self {
    Self {
      update_count: 0,
      total_update_time: 0,
      max_update_time: 0,
      last_stats_time: 0,
    }
  }

  pub fn record_update(&mut self, update_time_us: u32) {
    self.update_count += 1;
    self.total_update_time += update_time_us;
    if update_time_us > self.max_update_time {
      self.max_update_time = update_time_us;
    }
  }

  pub fn print_stats(&mut self, current_time: u32) {
    if current_time - self.last_stats_time >= 5000 {
      if self.update_count > 0 {
        let avg_time = self.total_update_time / self.update_count;
        defmt::info!(
          "性能统计 - 平均更新时间: {}μs, 最大: {}μs, 更新次数: {}",
          avg_time,
          self.max_update_time,
          self.update_count
        );

        self.update_count = 0;
        self.total_update_time = 0;
        self.max_update_time = 0;
      }
      self.last_stats_time = current_time;
    }
  }
}

// 简单延时函数
fn delay_ms(ms: u32) {
  for _ in 0..(ms * 1000) {
    cortex_m::asm::nop();
  }
}

#[entry]
fn main() -> ! {
  defmt::info!("舵机控制系统启动");

  // 初始化外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 状态LED (PC13)
  let mut status_led = gpioc.pc13.into_push_pull_output();
  status_led.set_high();

  // 按钮配置 (PA0-PA3)
  let mode_button = gpioa.pa0.into_pull_up_input();
  let angle_button = gpioa.pa1.into_pull_up_input();
  let speed_button = gpioa.pa2.into_pull_up_input();
  let sequence_button = gpioa.pa3.into_pull_up_input();

  // PWM配置 (TIM3, PB4-PB7)
  let servo_pins = (
    gpiob.pb4.into_alternate(), // TIM3_CH1
    gpiob.pb5.into_alternate(), // TIM3_CH2
    gpiob.pb0.into_alternate(), // TIM3_CH3
    gpiob.pb1.into_alternate(), // TIM3_CH4
  );

  let mut pwm = dp.TIM3.pwm_hz(servo_pins, 50.hz(), &clocks);
  pwm.enable(Channel::C1);
  pwm.enable(Channel::C2);
  pwm.enable(Channel::C3);
  pwm.enable(Channel::C4);

  // 创建舵机控制器
  let mut servo1 = ServoController::new(pwm.split().0, 500, 2500, 180.0);
  let mut servo2 = ServoController::new(pwm.split().1, 500, 2500, 180.0);
  let mut servo3 = ServoController::new(pwm.split().2, 500, 2500, 180.0);
  let mut servo4 = ServoController::new(pwm.split().3, 500, 2500, 180.0);

  // 初始化舵机到中位
  servo1.set_angle(90.0);
  servo2.set_angle(90.0);
  servo3.set_angle(90.0);
  servo4.set_angle(90.0);

  // 创建多舵机控制器
  let mut multi_servo = MultiServoController::new();
  let _ = multi_servo.add_servo(1, 90.0, 90.0);
  let _ = multi_servo.add_servo(2, 90.0, 90.0);
  let _ = multi_servo.add_servo(3, 90.0, 90.0);
  let _ = multi_servo.add_servo(4, 90.0, 90.0);

  // 创建动作序列
  let mut sequence = ServoSequence::new();
  let _ = sequence.add_step(1, 0.0, 1000, 0);
  let _ = sequence.add_step(2, 180.0, 1000, 500);
  let _ = sequence.add_step(3, 0.0, 1000, 500);
  let _ = sequence.add_step(4, 180.0, 1000, 500);
  let _ = sequence.add_step(1, 180.0, 1000, 500);
  let _ = sequence.add_step(2, 0.0, 1000, 500);
  let _ = sequence.add_step(3, 180.0, 1000, 500);
  let _ = sequence.add_step(4, 0.0, 1000, 500);

  // 创建按钮管理器和系统状态
  let mut button_manager =
    ButtonManager::new(mode_button, angle_button, speed_button, sequence_button);
  let mut system_state = SystemState::new(status_led);
  let mut performance_stats = PerformanceStats::new();

  // 系统定时器
  let mut timer = Timer::new(dp.TIM2, &clocks);
  timer.start(20.millis()); // 50Hz更新频率

  let mut last_time = 0u32;
  let mut system_time = 0u32;

  defmt::info!("舵机控制系统初始化完成");
  defmt::info!("模式按钮: 切换控制模式");
  defmt::info!("角度按钮: 调整角度/参数");
  defmt::info!("速度按钮: 调整速度等级");
  defmt::info!("序列按钮: 启动/停止动作序列");

  loop {
    // 等待定时器
    nb::block!(timer.wait()).unwrap();

    let start_time = system_time;
    system_time += 20; // 20ms更新周期
    let dt_ms = 20;

    // 更新按钮状态
    let button_events = button_manager.update();
    system_state.handle_button_events(button_events);

    // 处理序列按钮
    if button_events.sequence_pressed {
      if system_state.get_mode() == SystemMode::Sequence {
        if sequence.is_running() {
          sequence.stop();
        } else {
          sequence.start(true);
        }
      }
    }

    // 更新系统状态
    let target_angle = system_state.update(dt_ms);
    let speed = system_state.get_speed();

    // 更新舵机控制
    match system_state.get_mode() {
      SystemMode::Sequence => {
        sequence.update(dt_ms, &mut multi_servo);
        multi_servo.update(dt_ms);

        // 应用多舵机控制器的角度到实际舵机
        for servo_info in multi_servo.servos.iter() {
          match servo_info.id {
            1 => {
              servo1.set_speed(servo_info.speed);
              servo1.smooth_move_to(servo_info.current_angle, dt_ms);
            }
            2 => {
              servo2.set_speed(servo_info.speed);
              servo2.smooth_move_to(servo_info.current_angle, dt_ms);
            }
            3 => {
              servo3.set_speed(servo_info.speed);
              servo3.smooth_move_to(servo_info.current_angle, dt_ms);
            }
            4 => {
              servo4.set_speed(servo_info.speed);
              servo4.smooth_move_to(servo_info.current_angle, dt_ms);
            }
            _ => {}
          }
        }
      }
      _ => {
        // 其他模式：所有舵机同步运动
        servo1.set_speed(speed);
        servo2.set_speed(speed);
        servo3.set_speed(speed);
        servo4.set_speed(speed);

        servo1.smooth_move_to(target_angle, dt_ms);
        servo2.smooth_move_to(target_angle + 30.0, dt_ms);
        servo3.smooth_move_to(target_angle - 30.0, dt_ms);
        servo4.smooth_move_to(180.0 - target_angle, dt_ms);
      }
    }

    // 性能统计
    let update_time = (system_time - start_time) * 1000; // 转换为微秒
    performance_stats.record_update(update_time);
    performance_stats.print_stats(system_time);

    // 定期输出状态信息
    if system_time % 2000 == 0 {
      defmt::info!(
        "模式: {:?}, 角度: {:.1}°, 速度: {:.1}°/s",
        system_state.get_mode(),
        servo1.get_current_angle(),
        speed
      );
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_servo_angle_calculation() {
    // 测试角度到脉宽的转换
    let min_pulse = 500;
    let max_pulse = 2500;
    let max_angle = 180.0;

    // 0度应该对应最小脉宽
    let pulse_0 = min_pulse + ((0.0 / max_angle) * (max_pulse - min_pulse) as f32) as u16;
    assert_eq!(pulse_0, min_pulse);

    // 90度应该对应中间脉宽
    let pulse_90 = min_pulse + ((90.0 / max_angle) * (max_pulse - min_pulse) as f32) as u16;
    assert_eq!(pulse_90, 1500);

    // 180度应该对应最大脉宽
    let pulse_180 = min_pulse + ((180.0 / max_angle) * (max_pulse - min_pulse) as f32) as u16;
    assert_eq!(pulse_180, max_pulse);
  }

  #[test]
  fn test_multi_servo_controller() {
    let mut controller = MultiServoController::new();

    // 添加舵机
    assert!(controller.add_servo(1, 90.0, 60.0).is_ok());
    assert!(controller.add_servo(2, 45.0, 90.0).is_ok());

    // 设置角度
    controller.set_servo_angle(1, 180.0);
    controller.set_servo_angle(2, 0.0);

    // 更新一次
    controller.update(100); // 100ms

    // 检查角度是否朝目标移动
    assert!(controller.servos[0].current_angle > 90.0);
    assert!(controller.servos[1].current_angle < 45.0);
  }

  #[test]
  fn test_servo_sequence() {
    let mut sequence = ServoSequence::new();

    // 添加步骤
    assert!(sequence.add_step(1, 90.0, 1000, 0).is_ok());
    assert!(sequence.add_step(1, 180.0, 1000, 500).is_ok());

    // 启动序列
    sequence.start(false);
    assert!(sequence.is_running());

    let mut multi_servo = MultiServoController::new();
    let _ = multi_servo.add_servo(1, 0.0, 90.0);

    // 更新序列
    sequence.update(100, &mut multi_servo);

    // 检查是否正在执行
    assert!(sequence.is_running());
  }
}
