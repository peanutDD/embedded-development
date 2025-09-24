#![no_std]

use embedded_hal::{PwmPin, digital::v2::OutputPin};
use micromath::F32Ext;
use pid::Pid;

/// 直流电机控制器
pub struct DcMotorController<PWM, DIR1, DIR2> {
    pwm: PWM,
    dir1: DIR1,
    dir2: DIR2,
    max_speed: f32,
    current_speed: f32,
    direction: MotorDirection,
}

impl<PWM, DIR1, DIR2> DcMotorController<PWM, DIR1, DIR2>
where
    PWM: PwmPin<Duty = u32>,
    DIR1: OutputPin,
    DIR2: OutputPin,
{
    /// 创建新的直流电机控制器
    pub fn new(mut pwm: PWM, mut dir1: DIR1, mut dir2: DIR2) -> Self {
        pwm.enable();
        dir1.set_low().ok();
        dir2.set_low().ok();
        
        Self {
            pwm,
            dir1,
            dir2,
            max_speed: 100.0,
            current_speed: 0.0,
            direction: MotorDirection::Stop,
        }
    }

    /// 设置电机速度和方向
    pub fn set_speed(&mut self, speed: f32) -> Result<(), MotorError> {
        if speed.abs() > self.max_speed {
            return Err(MotorError::SpeedOutOfRange);
        }

        self.current_speed = speed.abs();
        
        if speed > 0.0 {
            self.direction = MotorDirection::Forward;
            self.dir1.set_high().ok();
            self.dir2.set_low().ok();
        } else if speed < 0.0 {
            self.direction = MotorDirection::Backward;
            self.dir1.set_low().ok();
            self.dir2.set_high().ok();
        } else {
            self.direction = MotorDirection::Stop;
            self.dir1.set_low().ok();
            self.dir2.set_low().ok();
        }

        let duty_cycle = self.speed_to_duty_cycle(self.current_speed);
        self.pwm.set_duty(duty_cycle);
        
        Ok(())
    }

    /// 停止电机
    pub fn stop(&mut self) {
        self.set_speed(0.0).ok();
    }

    /// 刹车（短路制动）
    pub fn brake(&mut self) {
        self.dir1.set_high().ok();
        self.dir2.set_high().ok();
        self.pwm.set_duty(self.pwm.get_max_duty());
        self.direction = MotorDirection::Brake;
        self.current_speed = 0.0;
    }

    /// 获取当前速度
    pub fn get_speed(&self) -> f32 {
        match self.direction {
            MotorDirection::Forward => self.current_speed,
            MotorDirection::Backward => -self.current_speed,
            _ => 0.0,
        }
    }

    /// 获取当前方向
    pub fn get_direction(&self) -> MotorDirection {
        self.direction
    }

    /// 速度转换为占空比
    fn speed_to_duty_cycle(&self, speed: f32) -> u32 {
        let ratio = speed / self.max_speed;
        (self.pwm.get_max_duty() as f32 * ratio) as u32
    }

    /// 设置最大速度
    pub fn set_max_speed(&mut self, max_speed: f32) {
        self.max_speed = max_speed;
    }
}

/// 步进电机控制器
pub struct StepperMotorController<STEP, DIR, EN> {
    step_pin: STEP,
    dir_pin: DIR,
    enable_pin: EN,
    steps_per_revolution: u32,
    current_position: i32,
    step_delay_us: u32,
    direction: StepperDirection,
    enabled: bool,
}

impl<STEP, DIR, EN> StepperMotorController<STEP, DIR, EN>
where
    STEP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
{
    /// 创建新的步进电机控制器
    pub fn new(
        mut step_pin: STEP,
        mut dir_pin: DIR,
        mut enable_pin: EN,
        steps_per_revolution: u32,
    ) -> Self {
        step_pin.set_low().ok();
        dir_pin.set_low().ok();
        enable_pin.set_high().ok(); // 通常高电平禁用
        
        Self {
            step_pin,
            dir_pin,
            enable_pin,
            steps_per_revolution,
            current_position: 0,
            step_delay_us: 1000, // 1ms默认延时
            direction: StepperDirection::Clockwise,
            enabled: false,
        }
    }

    /// 启用电机
    pub fn enable(&mut self) {
        self.enable_pin.set_low().ok();
        self.enabled = true;
    }

    /// 禁用电机
    pub fn disable(&mut self) {
        self.enable_pin.set_high().ok();
        self.enabled = false;
    }

    /// 设置方向
    pub fn set_direction(&mut self, direction: StepperDirection) {
        self.direction = direction;
        match direction {
            StepperDirection::Clockwise => self.dir_pin.set_low().ok(),
            StepperDirection::CounterClockwise => self.dir_pin.set_high().ok(),
        }
    }

    /// 执行单步
    pub fn step(&mut self) -> Result<(), MotorError> {
        if !self.enabled {
            return Err(MotorError::MotorDisabled);
        }

        self.step_pin.set_high().ok();
        // 这里需要延时，在实际使用中由调用者控制
        self.step_pin.set_low().ok();

        match self.direction {
            StepperDirection::Clockwise => self.current_position += 1,
            StepperDirection::CounterClockwise => self.current_position -= 1,
        }

        Ok(())
    }

    /// 移动指定步数
    pub fn move_steps(&mut self, steps: i32) -> Result<(), MotorError> {
        if steps == 0 {
            return Ok(());
        }

        let direction = if steps > 0 {
            StepperDirection::Clockwise
        } else {
            StepperDirection::CounterClockwise
        };

        self.set_direction(direction);

        for _ in 0..steps.abs() {
            self.step()?;
            // 延时由调用者控制
        }

        Ok(())
    }

    /// 移动到指定位置
    pub fn move_to_position(&mut self, target_position: i32) -> Result<(), MotorError> {
        let steps = target_position - self.current_position;
        self.move_steps(steps)
    }

    /// 旋转指定角度
    pub fn rotate_degrees(&mut self, degrees: f32) -> Result<(), MotorError> {
        let steps = (degrees * self.steps_per_revolution as f32 / 360.0) as i32;
        self.move_steps(steps)
    }

    /// 获取当前位置
    pub fn get_position(&self) -> i32 {
        self.current_position
    }

    /// 重置位置
    pub fn reset_position(&mut self) {
        self.current_position = 0;
    }

    /// 设置步进延时
    pub fn set_step_delay(&mut self, delay_us: u32) {
        self.step_delay_us = delay_us;
    }

    /// 获取步进延时
    pub fn get_step_delay(&self) -> u32 {
        self.step_delay_us
    }
}

/// 无刷电机控制器（三相）
pub struct BrushlessMotorController<PWM_A, PWM_B, PWM_C> {
    pwm_a: PWM_A,
    pwm_b: PWM_B,
    pwm_c: PWM_C,
    commutation_step: u8,
    speed: f32,
    direction: MotorDirection,
}

impl<PWM_A, PWM_B, PWM_C> BrushlessMotorController<PWM_A, PWM_B, PWM_C>
where
    PWM_A: PwmPin<Duty = u32>,
    PWM_B: PwmPin<Duty = u32>,
    PWM_C: PwmPin<Duty = u32>,
{
    /// 创建新的无刷电机控制器
    pub fn new(mut pwm_a: PWM_A, mut pwm_b: PWM_B, mut pwm_c: PWM_C) -> Self {
        pwm_a.enable();
        pwm_b.enable();
        pwm_c.enable();
        
        Self {
            pwm_a,
            pwm_b,
            pwm_c,
            commutation_step: 0,
            speed: 0.0,
            direction: MotorDirection::Stop,
        }
    }

    /// 设置速度
    pub fn set_speed(&mut self, speed: f32) {
        self.speed = speed.abs();
        self.direction = if speed > 0.0 {
            MotorDirection::Forward
        } else if speed < 0.0 {
            MotorDirection::Backward
        } else {
            MotorDirection::Stop
        };
    }

    /// 执行换相
    pub fn commutate(&mut self) {
        if self.direction == MotorDirection::Stop {
            self.pwm_a.set_duty(0);
            self.pwm_b.set_duty(0);
            self.pwm_c.set_duty(0);
            return;
        }

        let duty_cycle = self.speed_to_duty_cycle(self.speed);
        
        match self.commutation_step {
            0 => {
                self.pwm_a.set_duty(duty_cycle);
                self.pwm_b.set_duty(0);
                self.pwm_c.set_duty(0);
            }
            1 => {
                self.pwm_a.set_duty(duty_cycle);
                self.pwm_b.set_duty(duty_cycle);
                self.pwm_c.set_duty(0);
            }
            2 => {
                self.pwm_a.set_duty(0);
                self.pwm_b.set_duty(duty_cycle);
                self.pwm_c.set_duty(0);
            }
            3 => {
                self.pwm_a.set_duty(0);
                self.pwm_b.set_duty(duty_cycle);
                self.pwm_c.set_duty(duty_cycle);
            }
            4 => {
                self.pwm_a.set_duty(0);
                self.pwm_b.set_duty(0);
                self.pwm_c.set_duty(duty_cycle);
            }
            5 => {
                self.pwm_a.set_duty(duty_cycle);
                self.pwm_b.set_duty(0);
                self.pwm_c.set_duty(duty_cycle);
            }
            _ => {}
        }

        // 更新换相步骤
        if self.direction == MotorDirection::Forward {
            self.commutation_step = (self.commutation_step + 1) % 6;
        } else {
            self.commutation_step = if self.commutation_step == 0 { 5 } else { self.commutation_step - 1 };
        }
    }

    /// 速度转换为占空比
    fn speed_to_duty_cycle(&self, speed: f32) -> u32 {
        let ratio = speed / 100.0; // 假设最大速度为100
        (self.pwm_a.get_max_duty() as f32 * ratio) as u32
    }

    /// 停止电机
    pub fn stop(&mut self) {
        self.speed = 0.0;
        self.direction = MotorDirection::Stop;
        self.commutate();
    }
}

/// 带编码器的电机控制器
pub struct EncoderMotorController<PWM, DIR1, DIR2, ENC_A, ENC_B> {
    motor: DcMotorController<PWM, DIR1, DIR2>,
    encoder_a: ENC_A,
    encoder_b: ENC_B,
    encoder_position: i32,
    target_position: i32,
    pid_controller: Pid<f32>,
    last_encoder_a: bool,
}

impl<PWM, DIR1, DIR2, ENC_A, ENC_B> EncoderMotorController<PWM, DIR1, DIR2, ENC_A, ENC_B>
where
    PWM: PwmPin<Duty = u32>,
    DIR1: OutputPin,
    DIR2: OutputPin,
    ENC_A: embedded_hal::digital::v2::InputPin,
    ENC_B: embedded_hal::digital::v2::InputPin,
{
    /// 创建新的编码器电机控制器
    pub fn new(
        motor: DcMotorController<PWM, DIR1, DIR2>,
        encoder_a: ENC_A,
        encoder_b: ENC_B,
        pid_kp: f32,
        pid_ki: f32,
        pid_kd: f32,
    ) -> Self {
        let pid_controller = Pid::new(pid_kp, pid_ki, pid_kd, 100.0, 100.0, 100.0, 0.0);
        
        Self {
            motor,
            encoder_a,
            encoder_b,
            encoder_position: 0,
            target_position: 0,
            pid_controller,
            last_encoder_a: false,
        }
    }

    /// 更新编码器读数
    pub fn update_encoder(&mut self) -> Result<(), MotorError> {
        let encoder_a = self.encoder_a.is_high().map_err(|_| MotorError::EncoderError)?;
        let encoder_b = self.encoder_b.is_high().map_err(|_| MotorError::EncoderError)?;

        if encoder_a && !self.last_encoder_a {
            if encoder_b {
                self.encoder_position += 1;
            } else {
                self.encoder_position -= 1;
            }
        }

        self.last_encoder_a = encoder_a;
        Ok(())
    }

    /// 设置目标位置
    pub fn set_target_position(&mut self, position: i32) {
        self.target_position = position;
    }

    /// 执行位置控制
    pub fn position_control(&mut self) -> Result<(), MotorError> {
        self.update_encoder()?;
        
        let error = (self.target_position - self.encoder_position) as f32;
        let output = self.pid_controller.next_control_output(error);
        
        self.motor.set_speed(output.output)?;
        
        Ok(())
    }

    /// 获取编码器位置
    pub fn get_encoder_position(&self) -> i32 {
        self.encoder_position
    }

    /// 重置编码器位置
    pub fn reset_encoder_position(&mut self) {
        self.encoder_position = 0;
    }

    /// 获取位置误差
    pub fn get_position_error(&self) -> i32 {
        self.target_position - self.encoder_position
    }
}

/// 电机方向
#[derive(Clone, Copy, PartialEq)]
pub enum MotorDirection {
    Forward,
    Backward,
    Stop,
    Brake,
}

/// 步进电机方向
#[derive(Clone, Copy, PartialEq)]
pub enum StepperDirection {
    Clockwise,
    CounterClockwise,
}

/// 电机错误类型
#[derive(Debug)]
pub enum MotorError {
    SpeedOutOfRange,
    MotorDisabled,
    EncoderError,
    PositionOutOfRange,
    InvalidConfiguration,
}

/// 电机配置
pub struct MotorConfig {
    pub max_speed: f32,
    pub acceleration: f32,
    pub deceleration: f32,
    pub pid_kp: f32,
    pub pid_ki: f32,
    pub pid_kd: f32,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            max_speed: 100.0,
            acceleration: 10.0,
            deceleration: 10.0,
            pid_kp: 1.0,
            pid_ki: 0.1,
            pid_kd: 0.01,
        }
    }
}