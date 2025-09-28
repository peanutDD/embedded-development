#![no_std]

use core::marker::PhantomData;
use embedded_hal::pwm::SetDutyCycle;
use heapless::{FnvIndexMap, Vec};
use libm::{atan2, cos, sin, sqrt};
use micromath::F32Ext;

/// PWM通道抽象
pub trait PwmChannel {
  type Error;

  /// 设置占空比 (0.0 - 1.0)
  fn set_duty_cycle(&mut self, duty: f32) -> Result<(), Self::Error>;

  /// 设置频率 (Hz)
  fn set_frequency(&mut self, freq: u32) -> Result<(), Self::Error>;

  /// 启用PWM输出
  fn enable(&mut self) -> Result<(), Self::Error>;

  /// 禁用PWM输出
  fn disable(&mut self) -> Result<(), Self::Error>;

  /// 获取当前占空比
  fn get_duty_cycle(&self) -> f32;

  /// 获取当前频率
  fn get_frequency(&self) -> u32;
}

/// 多通道PWM控制器
pub struct MultiChannelPwmController<PWM, const N: usize> {
  channels: [Option<PWM>; N],
  channel_configs: [ChannelConfig; N],
  sync_enabled: bool,
}

/// PWM通道配置
#[derive(Debug, Clone, Copy)]
pub struct ChannelConfig {
  pub frequency: u32,
  pub duty_cycle: f32,
  pub phase_offset: f32,
  pub enabled: bool,
  pub inverted: bool,
}

/// 舵机控制器
pub struct ServoController<PWM> {
  pwm: PWM,
  min_pulse_us: u32,
  max_pulse_us: u32,
  period_us: u32,
  current_angle: f32,
  target_angle: f32,
  max_speed: f32, // 度/秒
}

/// 电机驱动器
pub struct MotorDriver<PWM1, PWM2> {
  pwm_forward: PWM1,
  pwm_backward: PWM2,
  current_speed: f32,
  target_speed: f32,
  acceleration: f32,
  max_speed: f32,
  brake_enabled: bool,
}

/// LED调光器
pub struct LedDimmer<PWM> {
  pwm: PWM,
  brightness: f32,
  target_brightness: f32,
  fade_speed: f32,
  gamma_correction: bool,
}

/// PWM波形生成器
pub struct WaveformGenerator<PWM> {
  pwm: PWM,
  waveform_type: WaveformType,
  amplitude: f32,
  frequency: f32,
  phase: f32,
  offset: f32,
  sample_rate: u32,
}

/// 波形类型
#[derive(Debug, Clone, Copy)]
pub enum WaveformType {
  Sine,
  Square,
  Triangle,
  Sawtooth,
  Custom(fn(f32) -> f32),
}

/// PWM信号分析器
pub struct PwmAnalyzer {
  samples: Vec<f32, 1000>,
  frequency: f32,
  duty_cycle: f32,
  rms_value: f32,
  thd: f32, // 总谐波失真
}

impl Default for ChannelConfig {
  fn default() -> Self {
    Self {
      frequency: 1000,
      duty_cycle: 0.0,
      phase_offset: 0.0,
      enabled: false,
      inverted: false,
    }
  }
}

impl<PWM, const N: usize> MultiChannelPwmController<PWM, N>
where
  PWM: PwmChannel,
{
  /// 创建新的多通道PWM控制器
  pub fn new() -> Self {
    Self {
      channels: [const { None }; N],
      channel_configs: [ChannelConfig::default(); N],
      sync_enabled: false,
    }
  }

  /// 添加PWM通道
  pub fn add_channel(
    &mut self,
    index: usize,
    pwm: PWM,
    config: ChannelConfig,
  ) -> Result<(), &'static str> {
    if index >= N {
      return Err("Channel index out of bounds");
    }

    self.channels[index] = Some(pwm);
    self.channel_configs[index] = config;
    Ok(())
  }

  /// 设置通道占空比
  pub fn set_duty_cycle(&mut self, channel: usize, duty: f32) -> Result<(), &'static str> {
    if channel >= N {
      return Err("Channel index out of bounds");
    }

    if let Some(ref mut pwm) = self.channels[channel] {
      let adjusted_duty = if self.channel_configs[channel].inverted {
        1.0 - duty
      } else {
        duty
      };

      pwm
        .set_duty_cycle(adjusted_duty)
        .map_err(|_| "Failed to set duty cycle")?;
      self.channel_configs[channel].duty_cycle = duty;
    }

    Ok(())
  }

  /// 设置所有通道占空比
  pub fn set_all_duty_cycles(&mut self, duties: &[f32]) -> Result<(), &'static str> {
    for (i, &duty) in duties.iter().enumerate().take(N) {
      self.set_duty_cycle(i, duty)?;
    }
    Ok(())
  }

  /// 启用同步模式
  pub fn enable_sync(&mut self) {
    self.sync_enabled = true;
  }

  /// 同步更新所有通道
  pub fn sync_update(&mut self) -> Result<(), &'static str> {
    if !self.sync_enabled {
      return Ok(());
    }

    // 在实际硬件中，这里会触发同步更新
    // 这里只是示例实现
    for (i, channel) in self.channels.iter_mut().enumerate() {
      if let Some(ref mut pwm) = channel {
        if self.channel_configs[i].enabled {
          pwm.enable().map_err(|_| "Failed to enable channel")?;
        }
      }
    }

    Ok(())
  }
}

impl<PWM> ServoController<PWM>
where
  PWM: PwmChannel,
{
  /// 创建新的舵机控制器
  pub fn new(pwm: PWM, min_pulse_us: u32, max_pulse_us: u32) -> Self {
    Self {
      pwm,
      min_pulse_us,
      max_pulse_us,
      period_us: 20_000, // 20ms周期，50Hz
      current_angle: 90.0,
      target_angle: 90.0,
      max_speed: 180.0, // 180度/秒
    }
  }

  /// 设置目标角度 (0-180度)
  pub fn set_angle(&mut self, angle: f32) -> Result<(), &'static str> {
    if angle < 0.0 || angle > 180.0 {
      return Err("Angle out of range (0-180)");
    }

    self.target_angle = angle;
    self.update_pwm()?;
    Ok(())
  }

  /// 平滑移动到目标角度
  pub fn move_to_angle(&mut self, angle: f32, speed: f32) -> Result<(), &'static str> {
    if angle < 0.0 || angle > 180.0 {
      return Err("Angle out of range (0-180)");
    }

    self.target_angle = angle;
    self.max_speed = speed;
    Ok(())
  }

  /// 更新舵机位置（需要定期调用）
  pub fn update(&mut self, dt_ms: u32) -> Result<(), &'static str> {
    let dt_s = dt_ms as f32 / 1000.0;
    let angle_diff = self.target_angle - self.current_angle;

    if angle_diff.abs() > 0.1 {
      let max_change = self.max_speed * dt_s;
      let change = if angle_diff.abs() < max_change {
        angle_diff
      } else {
        max_change * angle_diff.signum()
      };

      self.current_angle += change;
      self.update_pwm()?;
    }

    Ok(())
  }

  /// 更新PWM输出
  fn update_pwm(&mut self) -> Result<(), &'static str> {
    // 将角度转换为脉宽
    let pulse_width = self.min_pulse_us
      + ((self.current_angle / 180.0) * (self.max_pulse_us - self.min_pulse_us) as f32) as u32;

    // 计算占空比
    let duty_cycle = pulse_width as f32 / self.period_us as f32;

    self
      .pwm
      .set_duty_cycle(duty_cycle)
      .map_err(|_| "Failed to set PWM duty cycle")?;
    Ok(())
  }

  /// 获取当前角度
  pub fn get_current_angle(&self) -> f32 {
    self.current_angle
  }

  /// 获取目标角度
  pub fn get_target_angle(&self) -> f32 {
    self.target_angle
  }
}

impl<PWM1, PWM2> MotorDriver<PWM1, PWM2>
where
  PWM1: PwmChannel,
  PWM2: PwmChannel,
{
  /// 创建新的电机驱动器
  pub fn new(pwm_forward: PWM1, pwm_backward: PWM2) -> Self {
    Self {
      pwm_forward,
      pwm_backward,
      current_speed: 0.0,
      target_speed: 0.0,
      acceleration: 100.0, // 100%/秒
      max_speed: 100.0,
      brake_enabled: false,
    }
  }

  /// 设置电机速度 (-100.0 到 100.0)
  pub fn set_speed(&mut self, speed: f32) -> Result<(), &'static str> {
    let clamped_speed = speed.clamp(-self.max_speed, self.max_speed);
    self.target_speed = clamped_speed;
    self.update_pwm()?;
    Ok(())
  }

  /// 设置加速度
  pub fn set_acceleration(&mut self, acceleration: f32) {
    self.acceleration = acceleration.abs();
  }

  /// 启用制动
  pub fn enable_brake(&mut self) -> Result<(), &'static str> {
    self.brake_enabled = true;
    self
      .pwm_forward
      .set_duty_cycle(1.0)
      .map_err(|_| "Failed to set forward PWM")?;
    self
      .pwm_backward
      .set_duty_cycle(1.0)
      .map_err(|_| "Failed to set backward PWM")?;
    Ok(())
  }

  /// 禁用制动
  pub fn disable_brake(&mut self) -> Result<(), &'static str> {
    self.brake_enabled = false;
    self.update_pwm()?;
    Ok(())
  }

  /// 更新电机状态（需要定期调用）
  pub fn update(&mut self, dt_ms: u32) -> Result<(), &'static str> {
    if self.brake_enabled {
      return Ok(());
    }

    let dt_s = dt_ms as f32 / 1000.0;
    let speed_diff = self.target_speed - self.current_speed;

    if speed_diff.abs() > 0.1 {
      let max_change = self.acceleration * dt_s;
      let change = if speed_diff.abs() < max_change {
        speed_diff
      } else {
        max_change * speed_diff.signum()
      };

      self.current_speed += change;
      self.update_pwm()?;
    }

    Ok(())
  }

  /// 更新PWM输出
  fn update_pwm(&mut self) -> Result<(), &'static str> {
    if self.brake_enabled {
      return Ok(());
    }

    let speed_abs = self.current_speed.abs() / 100.0;

    if self.current_speed >= 0.0 {
      // 正向旋转
      self
        .pwm_forward
        .set_duty_cycle(speed_abs)
        .map_err(|_| "Failed to set forward PWM")?;
      self
        .pwm_backward
        .set_duty_cycle(0.0)
        .map_err(|_| "Failed to set backward PWM")?;
    } else {
      // 反向旋转
      self
        .pwm_forward
        .set_duty_cycle(0.0)
        .map_err(|_| "Failed to set forward PWM")?;
      self
        .pwm_backward
        .set_duty_cycle(speed_abs)
        .map_err(|_| "Failed to set backward PWM")?;
    }

    Ok(())
  }

  /// 获取当前速度
  pub fn get_current_speed(&self) -> f32 {
    self.current_speed
  }
}

impl<PWM> LedDimmer<PWM>
where
  PWM: PwmChannel,
{
  /// 创建新的LED调光器
  pub fn new(pwm: PWM) -> Self {
    Self {
      pwm,
      brightness: 0.0,
      target_brightness: 0.0,
      fade_speed: 50.0, // 50%/秒
      gamma_correction: true,
    }
  }

  /// 设置亮度 (0.0 - 100.0)
  pub fn set_brightness(&mut self, brightness: f32) -> Result<(), &'static str> {
    let clamped_brightness = brightness.clamp(0.0, 100.0);
    self.target_brightness = clamped_brightness;
    self.update_pwm()?;
    Ok(())
  }

  /// 淡入淡出到目标亮度
  pub fn fade_to(&mut self, brightness: f32, speed: f32) -> Result<(), &'static str> {
    let clamped_brightness = brightness.clamp(0.0, 100.0);
    self.target_brightness = clamped_brightness;
    self.fade_speed = speed.abs();
    Ok(())
  }

  /// 更新LED状态（需要定期调用）
  pub fn update(&mut self, dt_ms: u32) -> Result<(), &'static str> {
    let dt_s = dt_ms as f32 / 1000.0;
    let brightness_diff = self.target_brightness - self.brightness;

    if brightness_diff.abs() > 0.1 {
      let max_change = self.fade_speed * dt_s;
      let change = if brightness_diff.abs() < max_change {
        brightness_diff
      } else {
        max_change * brightness_diff.signum()
      };

      self.brightness += change;
      self.update_pwm()?;
    }

    Ok(())
  }

  /// 更新PWM输出
  fn update_pwm(&mut self) -> Result<(), &'static str> {
    let mut duty_cycle = self.brightness / 100.0;

    // 应用伽马校正
    if self.gamma_correction {
      duty_cycle = duty_cycle.powf(2.2);
    }

    self
      .pwm
      .set_duty_cycle(duty_cycle)
      .map_err(|_| "Failed to set PWM duty cycle")?;
    Ok(())
  }

  /// 启用/禁用伽马校正
  pub fn set_gamma_correction(&mut self, enabled: bool) {
    self.gamma_correction = enabled;
  }

  /// 获取当前亮度
  pub fn get_brightness(&self) -> f32 {
    self.brightness
  }
}

impl<PWM> WaveformGenerator<PWM>
where
  PWM: PwmChannel,
{
  /// 创建新的波形生成器
  pub fn new(pwm: PWM, sample_rate: u32) -> Self {
    Self {
      pwm,
      waveform_type: WaveformType::Sine,
      amplitude: 1.0,
      frequency: 1.0,
      phase: 0.0,
      offset: 0.0,
      sample_rate,
    }
  }

  /// 设置波形参数
  pub fn set_waveform(
    &mut self,
    waveform_type: WaveformType,
    amplitude: f32,
    frequency: f32,
    offset: f32,
  ) {
    self.waveform_type = waveform_type;
    self.amplitude = amplitude;
    self.frequency = frequency;
    self.offset = offset;
  }

  /// 生成下一个样本
  pub fn generate_sample(&mut self, time: f32) -> Result<f32, &'static str> {
    let normalized_time = time * self.frequency + self.phase;

    let sample = match self.waveform_type {
      WaveformType::Sine => sin(2.0 * core::f32::consts::PI * normalized_time),
      WaveformType::Square => {
        if (normalized_time % 1.0) < 0.5 {
          1.0
        } else {
          -1.0
        }
      }
      WaveformType::Triangle => {
        let t = normalized_time % 1.0;
        if t < 0.5 {
          4.0 * t - 1.0
        } else {
          3.0 - 4.0 * t
        }
      }
      WaveformType::Sawtooth => 2.0 * (normalized_time % 1.0) - 1.0,
      WaveformType::Custom(func) => func(normalized_time),
    };

    let output = self.amplitude * sample + self.offset;
    let duty_cycle = (output + 1.0) / 2.0; // 转换到0-1范围

    self
      .pwm
      .set_duty_cycle(duty_cycle.clamp(0.0, 1.0))
      .map_err(|_| "Failed to set PWM")?;

    Ok(output)
  }

  /// 更新相位
  pub fn update_phase(&mut self, dt: f32) {
    self.phase += self.frequency * dt;
    if self.phase >= 1.0 {
      self.phase -= 1.0;
    }
  }
}

impl PwmAnalyzer {
  /// 创建新的PWM分析器
  pub fn new() -> Self {
    Self {
      samples: Vec::new(),
      frequency: 0.0,
      duty_cycle: 0.0,
      rms_value: 0.0,
      thd: 0.0,
    }
  }

  /// 添加样本
  pub fn add_sample(&mut self, sample: f32) -> Result<(), &'static str> {
    self
      .samples
      .push(sample)
      .map_err(|_| "Sample buffer full")?;
    Ok(())
  }

  /// 分析PWM信号
  pub fn analyze(&mut self) -> Result<(), &'static str> {
    if self.samples.is_empty() {
      return Err("No samples to analyze");
    }

    // 计算占空比
    let high_count = self.samples.iter().filter(|&&x| x > 0.5).count();
    self.duty_cycle = high_count as f32 / self.samples.len() as f32;

    // 计算RMS值
    let sum_squares: f32 = self.samples.iter().map(|&x| x * x).sum();
    self.rms_value = sqrt(sum_squares / self.samples.len() as f32);

    // 简化的频率检测（边沿计数）
    let mut edge_count = 0;
    for i in 1..self.samples.len() {
      if (self.samples[i] > 0.5) != (self.samples[i - 1] > 0.5) {
        edge_count += 1;
      }
    }
    // 假设采样率为1kHz，计算频率
    self.frequency = (edge_count as f32 / 2.0) * 1000.0 / self.samples.len() as f32;

    Ok(())
  }

  /// 获取分析结果
  pub fn get_results(&self) -> (f32, f32, f32, f32) {
    (self.frequency, self.duty_cycle, self.rms_value, self.thd)
  }

  /// 清除样本
  pub fn clear(&mut self) {
    self.samples.clear();
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  struct MockPwm {
    duty_cycle: f32,
    frequency: u32,
    enabled: bool,
  }

  impl MockPwm {
    fn new() -> Self {
      Self {
        duty_cycle: 0.0,
        frequency: 1000,
        enabled: false,
      }
    }
  }

  impl PwmChannel for MockPwm {
    type Error = ();

    fn set_duty_cycle(&mut self, duty: f32) -> Result<(), Self::Error> {
      self.duty_cycle = duty.clamp(0.0, 1.0);
      Ok(())
    }

    fn set_frequency(&mut self, freq: u32) -> Result<(), Self::Error> {
      self.frequency = freq;
      Ok(())
    }

    fn enable(&mut self) -> Result<(), Self::Error> {
      self.enabled = true;
      Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
      self.enabled = false;
      Ok(())
    }

    fn get_duty_cycle(&self) -> f32 {
      self.duty_cycle
    }

    fn get_frequency(&self) -> u32 {
      self.frequency
    }
  }

  #[test]
  fn test_servo_controller() {
    let pwm = MockPwm::new();
    let mut servo = ServoController::new(pwm, 1000, 2000);

    assert!(servo.set_angle(90.0).is_ok());
    assert_eq!(servo.get_current_angle(), 90.0);

    assert!(servo.set_angle(0.0).is_ok());
    assert_eq!(servo.get_target_angle(), 0.0);
  }

  #[test]
  fn test_motor_driver() {
    let pwm1 = MockPwm::new();
    let pwm2 = MockPwm::new();
    let mut motor = MotorDriver::new(pwm1, pwm2);

    assert!(motor.set_speed(50.0).is_ok());
    assert_eq!(motor.get_current_speed(), 50.0);

    assert!(motor.enable_brake().is_ok());
    assert!(motor.disable_brake().is_ok());
  }

  #[test]
  fn test_led_dimmer() {
    let pwm = MockPwm::new();
    let mut dimmer = LedDimmer::new(pwm);

    assert!(dimmer.set_brightness(75.0).is_ok());
    assert_eq!(dimmer.get_brightness(), 75.0);

    assert!(dimmer.fade_to(25.0, 10.0).is_ok());
  }
}
