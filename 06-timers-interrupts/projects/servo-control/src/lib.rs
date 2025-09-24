#![no_std]

use embedded_hal::PwmPin;
use micromath::F32Ext;

/// 舵机控制器
pub struct ServoController<PWM> {
    pwm: PWM,
    min_pulse_us: u32,
    max_pulse_us: u32,
    max_angle: f32,
    current_angle: f32,
    pwm_frequency: u32,
    pwm_max_duty: u32,
}

impl<PWM> ServoController<PWM>
where
    PWM: PwmPin<Duty = u32>,
{
    /// 创建新的舵机控制器
    pub fn new(
        mut pwm: PWM,
        min_pulse_us: u32,
        max_pulse_us: u32,
        max_angle: f32,
        pwm_frequency: u32,
    ) -> Self {
        let pwm_max_duty = pwm.get_max_duty();
        pwm.enable();
        
        Self {
            pwm,
            min_pulse_us,
            max_pulse_us,
            max_angle,
            current_angle: 0.0,
            pwm_frequency,
            pwm_max_duty,
        }
    }

    /// 设置舵机角度
    pub fn set_angle(&mut self, angle: f32) -> Result<(), ServoError> {
        if angle < 0.0 || angle > self.max_angle {
            return Err(ServoError::AngleOutOfRange);
        }

        let pulse_us = self.angle_to_pulse_width(angle);
        let duty_cycle = self.pulse_width_to_duty_cycle(pulse_us);
        
        self.pwm.set_duty(duty_cycle);
        self.current_angle = angle;
        
        Ok(())
    }

    /// 获取当前角度
    pub fn get_angle(&self) -> f32 {
        self.current_angle
    }

    /// 角度转换为脉宽
    fn angle_to_pulse_width(&self, angle: f32) -> u32 {
        let ratio = angle / self.max_angle;
        let pulse_range = self.max_pulse_us - self.min_pulse_us;
        self.min_pulse_us + (pulse_range as f32 * ratio) as u32
    }

    /// 脉宽转换为占空比
    fn pulse_width_to_duty_cycle(&self, pulse_us: u32) -> u32 {
        let period_us = 1_000_000 / self.pwm_frequency;
        let duty_ratio = pulse_us as f32 / period_us as f32;
        (self.pwm_max_duty as f32 * duty_ratio) as u32
    }

    /// 平滑移动到目标角度
    pub fn smooth_move(&mut self, target_angle: f32, steps: u32) -> Result<(), ServoError> {
        if target_angle < 0.0 || target_angle > self.max_angle {
            return Err(ServoError::AngleOutOfRange);
        }

        let angle_step = (target_angle - self.current_angle) / steps as f32;
        
        for i in 1..=steps {
            let intermediate_angle = self.current_angle + angle_step * i as f32;
            self.set_angle(intermediate_angle)?;
            // 这里需要延时，在实际使用中由调用者控制
        }
        
        Ok(())
    }

    /// 禁用PWM输出
    pub fn disable(&mut self) {
        self.pwm.disable();
    }

    /// 启用PWM输出
    pub fn enable(&mut self) {
        self.pwm.enable();
    }
}

/// 多舵机控制器
pub struct MultiServoController<PWM1, PWM2, PWM3, PWM4> {
    servo1: Option<ServoController<PWM1>>,
    servo2: Option<ServoController<PWM2>>,
    servo3: Option<ServoController<PWM3>>,
    servo4: Option<ServoController<PWM4>>,
}

impl<PWM1, PWM2, PWM3, PWM4> MultiServoController<PWM1, PWM2, PWM3, PWM4>
where
    PWM1: PwmPin<Duty = u32>,
    PWM2: PwmPin<Duty = u32>,
    PWM3: PwmPin<Duty = u32>,
    PWM4: PwmPin<Duty = u32>,
{
    /// 创建多舵机控制器
    pub fn new() -> Self {
        Self {
            servo1: None,
            servo2: None,
            servo3: None,
            servo4: None,
        }
    }

    /// 添加舵机
    pub fn add_servo1(&mut self, servo: ServoController<PWM1>) {
        self.servo1 = Some(servo);
    }

    pub fn add_servo2(&mut self, servo: ServoController<PWM2>) {
        self.servo2 = Some(servo);
    }

    pub fn add_servo3(&mut self, servo: ServoController<PWM3>) {
        self.servo3 = Some(servo);
    }

    pub fn add_servo4(&mut self, servo: ServoController<PWM4>) {
        self.servo4 = Some(servo);
    }

    /// 设置所有舵机角度
    pub fn set_all_angles(&mut self, angles: [f32; 4]) -> Result<(), ServoError> {
        if let Some(ref mut servo) = self.servo1 {
            servo.set_angle(angles[0])?;
        }
        if let Some(ref mut servo) = self.servo2 {
            servo.set_angle(angles[1])?;
        }
        if let Some(ref mut servo) = self.servo3 {
            servo.set_angle(angles[2])?;
        }
        if let Some(ref mut servo) = self.servo4 {
            servo.set_angle(angles[3])?;
        }
        Ok(())
    }

    /// 获取所有舵机角度
    pub fn get_all_angles(&self) -> [f32; 4] {
        [
            self.servo1.as_ref().map_or(0.0, |s| s.get_angle()),
            self.servo2.as_ref().map_or(0.0, |s| s.get_angle()),
            self.servo3.as_ref().map_or(0.0, |s| s.get_angle()),
            self.servo4.as_ref().map_or(0.0, |s| s.get_angle()),
        ]
    }

    /// 同步移动所有舵机
    pub fn sync_move(&mut self, target_angles: [f32; 4], steps: u32) -> Result<(), ServoError> {
        for step in 1..=steps {
            let progress = step as f32 / steps as f32;
            
            if let Some(ref mut servo) = self.servo1 {
                let current = servo.get_angle();
                let target = target_angles[0];
                let intermediate = current + (target - current) * progress;
                servo.set_angle(intermediate)?;
            }
            
            if let Some(ref mut servo) = self.servo2 {
                let current = servo.get_angle();
                let target = target_angles[1];
                let intermediate = current + (target - current) * progress;
                servo.set_angle(intermediate)?;
            }
            
            if let Some(ref mut servo) = self.servo3 {
                let current = servo.get_angle();
                let target = target_angles[2];
                let intermediate = current + (target - current) * progress;
                servo.set_angle(intermediate)?;
            }
            
            if let Some(ref mut servo) = self.servo4 {
                let current = servo.get_angle();
                let target = target_angles[3];
                let intermediate = current + (target - current) * progress;
                servo.set_angle(intermediate)?;
            }
            
            // 延时由调用者控制
        }
        
        Ok(())
    }
}

/// 舵机扫描器
pub struct ServoSweeper<PWM> {
    servo: ServoController<PWM>,
    min_angle: f32,
    max_angle: f32,
    step_size: f32,
    current_direction: SweepDirection,
}

impl<PWM> ServoSweeper<PWM>
where
    PWM: PwmPin<Duty = u32>,
{
    /// 创建舵机扫描器
    pub fn new(
        servo: ServoController<PWM>,
        min_angle: f32,
        max_angle: f32,
        step_size: f32,
    ) -> Self {
        Self {
            servo,
            min_angle,
            max_angle,
            step_size,
            current_direction: SweepDirection::Forward,
        }
    }

    /// 执行一步扫描
    pub fn step(&mut self) -> Result<f32, ServoError> {
        let current_angle = self.servo.get_angle();
        
        let next_angle = match self.current_direction {
            SweepDirection::Forward => {
                let next = current_angle + self.step_size;
                if next >= self.max_angle {
                    self.current_direction = SweepDirection::Backward;
                    self.max_angle
                } else {
                    next
                }
            }
            SweepDirection::Backward => {
                let next = current_angle - self.step_size;
                if next <= self.min_angle {
                    self.current_direction = SweepDirection::Forward;
                    self.min_angle
                } else {
                    next
                }
            }
        };
        
        self.servo.set_angle(next_angle)?;
        Ok(next_angle)
    }

    /// 获取当前角度
    pub fn get_angle(&self) -> f32 {
        self.servo.get_angle()
    }

    /// 获取扫描方向
    pub fn get_direction(&self) -> SweepDirection {
        self.current_direction
    }
}

/// 舵机校准器
pub struct ServoCalibrator<PWM> {
    servo: ServoController<PWM>,
    calibration_points: heapless::Vec<CalibrationPoint, 10>,
}

impl<PWM> ServoCalibrator<PWM>
where
    PWM: PwmPin<Duty = u32>,
{
    /// 创建舵机校准器
    pub fn new(servo: ServoController<PWM>) -> Self {
        Self {
            servo,
            calibration_points: heapless::Vec::new(),
        }
    }

    /// 添加校准点
    pub fn add_calibration_point(&mut self, angle: f32, actual_angle: f32) -> Result<(), ServoError> {
        let point = CalibrationPoint { angle, actual_angle };
        self.calibration_points.push(point).map_err(|_| ServoError::CalibrationFull)?;
        Ok(())
    }

    /// 校准角度
    pub fn calibrate_angle(&self, target_angle: f32) -> f32 {
        if self.calibration_points.is_empty() {
            return target_angle;
        }

        // 线性插值校准
        for i in 0..self.calibration_points.len() - 1 {
            let p1 = &self.calibration_points[i];
            let p2 = &self.calibration_points[i + 1];
            
            if target_angle >= p1.angle && target_angle <= p2.angle {
                let ratio = (target_angle - p1.angle) / (p2.angle - p1.angle);
                return p1.actual_angle + ratio * (p2.actual_angle - p1.actual_angle);
            }
        }
        
        target_angle
    }

    /// 设置校准后的角度
    pub fn set_calibrated_angle(&mut self, angle: f32) -> Result<(), ServoError> {
        let calibrated_angle = self.calibrate_angle(angle);
        self.servo.set_angle(calibrated_angle)
    }
}

/// 扫描方向
#[derive(Clone, Copy)]
pub enum SweepDirection {
    Forward,
    Backward,
}

/// 校准点
#[derive(Clone, Copy)]
pub struct CalibrationPoint {
    pub angle: f32,
    pub actual_angle: f32,
}

/// 舵机错误类型
#[derive(Debug)]
pub enum ServoError {
    AngleOutOfRange,
    CalibrationFull,
    InvalidConfiguration,
}

/// 舵机配置
pub struct ServoConfig {
    pub min_pulse_us: u32,
    pub max_pulse_us: u32,
    pub max_angle: f32,
    pub pwm_frequency: u32,
}

impl Default for ServoConfig {
    fn default() -> Self {
        Self {
            min_pulse_us: 1000,  // 1ms
            max_pulse_us: 2000,  // 2ms
            max_angle: 180.0,    // 180度
            pwm_frequency: 50,   // 50Hz
        }
    }
}

/// 标准舵机配置
impl ServoConfig {
    /// SG90舵机配置
    pub fn sg90() -> Self {
        Self {
            min_pulse_us: 500,
            max_pulse_us: 2400,
            max_angle: 180.0,
            pwm_frequency: 50,
        }
    }

    /// MG996R舵机配置
    pub fn mg996r() -> Self {
        Self {
            min_pulse_us: 1000,
            max_pulse_us: 2000,
            max_angle: 180.0,
            pwm_frequency: 50,
        }
    }

    /// 连续旋转舵机配置
    pub fn continuous() -> Self {
        Self {
            min_pulse_us: 1000,
            max_pulse_us: 2000,
            max_angle: 360.0,
            pwm_frequency: 50,
        }
    }
}