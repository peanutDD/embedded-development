#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Channel, PwmChannels},
    gpio::{gpioa::{PA8, PA9}, gpiob::PB0, Alternate, Output, PushPull, AF1},
};

type MotorPwmPin = PA8<Alternate<AF1>>;
type MotorDirPin1 = PA9<Output<PushPull>>;
type MotorDirPin2 = PB0<Output<PushPull>>;

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
    let gpiob = dp.GPIOB.split();
    
    let motor_pwm = gpioa.pa8.into_alternate_af1();
    let motor_dir1 = gpioa.pa9.into_push_pull_output();
    let motor_dir2 = gpiob.pb0.into_push_pull_output();

    // 配置定时器1用于PWM (20kHz for motor control)
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    let pwm = timer.pwm::<Timer1Ch1>(motor_pwm, 20.khz());
    
    // 创建电机控制器
    let mut motor = MotorController::new(pwm, motor_dir1, motor_dir2);
    
    // 配置系统定时器用于延时
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);

    // 电机控制主循环
    loop {
        // 正转加速
        motor.set_direction(MotorDirection::Forward);
        for speed in (0..=100).step_by(10) {
            motor.set_speed(speed);
            delay.delay_ms(200u32);
        }
        
        // 正转减速
        for speed in (0..=100).step_by(10).rev() {
            motor.set_speed(speed);
            delay.delay_ms(200u32);
        }
        
        // 停止
        motor.stop();
        delay.delay_ms(1000u32);
        
        // 反转加速
        motor.set_direction(MotorDirection::Reverse);
        for speed in (0..=100).step_by(10) {
            motor.set_speed(speed);
            delay.delay_ms(200u32);
        }
        
        // 反转减速
        for speed in (0..=100).step_by(10).rev() {
            motor.set_speed(speed);
            delay.delay_ms(200u32);
        }
        
        // 停止
        motor.stop();
        delay.delay_ms(1000u32);
        
        // 演示刹车功能
        motor.set_direction(MotorDirection::Forward);
        motor.set_speed(80);
        delay.delay_ms(2000u32);
        motor.brake();
        delay.delay_ms(1000u32);
    }
}

/// 电机方向枚举
#[derive(Clone, Copy, PartialEq)]
pub enum MotorDirection {
    Forward,
    Reverse,
    Brake,
    Coast,
}

/// 电机控制器结构体
pub struct MotorController<PWM_PIN, DIR1_PIN, DIR2_PIN> {
    pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, PWM_PIN>,
    dir1: DIR1_PIN,
    dir2: DIR2_PIN,
    max_duty: u16,
    current_speed: u8,
    current_direction: MotorDirection,
}

impl<PWM_PIN, DIR1_PIN, DIR2_PIN> MotorController<PWM_PIN, DIR1_PIN, DIR2_PIN>
where
    DIR1_PIN: embedded_hal::digital::v2::OutputPin,
    DIR2_PIN: embedded_hal::digital::v2::OutputPin,
{
    /// 创建新的电机控制器
    pub fn new(
        mut pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, PWM_PIN>,
        mut dir1: DIR1_PIN,
        mut dir2: DIR2_PIN,
    ) -> Self {
        pwm.enable(Channel::C1);
        let max_duty = pwm.get_max_duty();
        
        // 初始化方向引脚
        let _ = dir1.set_low();
        let _ = dir2.set_low();
        
        Self {
            pwm,
            dir1,
            dir2,
            max_duty,
            current_speed: 0,
            current_direction: MotorDirection::Coast,
        }
    }
    
    /// 设置电机方向
    pub fn set_direction(&mut self, direction: MotorDirection) {
        match direction {
            MotorDirection::Forward => {
                let _ = self.dir1.set_high();
                let _ = self.dir2.set_low();
            },
            MotorDirection::Reverse => {
                let _ = self.dir1.set_low();
                let _ = self.dir2.set_high();
            },
            MotorDirection::Brake => {
                let _ = self.dir1.set_high();
                let _ = self.dir2.set_high();
            },
            MotorDirection::Coast => {
                let _ = self.dir1.set_low();
                let _ = self.dir2.set_low();
            },
        }
        
        self.current_direction = direction;
    }
    
    /// 设置电机速度 (0-100%)
    pub fn set_speed(&mut self, speed: u8) {
        let speed = speed.min(100);
        let duty = (self.max_duty * speed as u16) / 100;
        
        self.pwm.set_duty(Channel::C1, duty);
        self.current_speed = speed;
    }
    
    /// 停止电机 (滑行停止)
    pub fn stop(&mut self) {
        self.set_speed(0);
        self.set_direction(MotorDirection::Coast);
    }
    
    /// 刹车停止
    pub fn brake(&mut self) {
        self.set_direction(MotorDirection::Brake);
        self.set_speed(100); // 全功率刹车
        
        // 实际应用中应该有延时后切换到滑行模式
        // delay.delay_ms(100);
        // self.stop();
    }
    
    /// 获取当前速度
    pub fn get_speed(&self) -> u8 {
        self.current_speed
    }
    
    /// 获取当前方向
    pub fn get_direction(&self) -> MotorDirection {
        self.current_direction
    }
    
    /// 平滑加速到目标速度
    pub fn accelerate_to(&mut self, target_speed: u8, acceleration: u8) {
        let target = target_speed.min(100);
        let accel = acceleration.max(1).min(10);
        
        while self.current_speed != target {
            if self.current_speed < target {
                let next_speed = (self.current_speed + accel).min(target);
                self.set_speed(next_speed);
            } else {
                let next_speed = self.current_speed.saturating_sub(accel).max(target);
                self.set_speed(next_speed);
            }
            
            // 延时处理，实际应用中使用定时器中断
        }
    }
    
    /// 软启动
    pub fn soft_start(&mut self, target_speed: u8, ramp_time_ms: u32) {
        let target = target_speed.min(100);
        let steps = 20;
        let step_delay = ramp_time_ms / steps;
        let speed_step = target / (steps as u8);
        
        for step in 1..=steps {
            let speed = (speed_step * step as u8).min(target);
            self.set_speed(speed);
            // 延时处理
        }
    }
    
    /// 软停止
    pub fn soft_stop(&mut self, ramp_time_ms: u32) {
        let initial_speed = self.current_speed;
        let steps = 20;
        let step_delay = ramp_time_ms / steps;
        let speed_step = initial_speed / (steps as u8);
        
        for step in 1..=steps {
            let speed = initial_speed.saturating_sub(speed_step * step as u8);
            self.set_speed(speed);
            // 延时处理
        }
        
        self.stop();
    }
}

/// 双电机控制器 (用于差速驱动)
pub struct DualMotorController<PWM1, PWM2, DIR1A, DIR1B, DIR2A, DIR2B> {
    motor1: MotorController<PWM1, DIR1A, DIR1B>,
    motor2: MotorController<PWM2, DIR2A, DIR2B>,
}

impl<PWM1, PWM2, DIR1A, DIR1B, DIR2A, DIR2B> DualMotorController<PWM1, PWM2, DIR1A, DIR1B, DIR2A, DIR2B>
where
    DIR1A: embedded_hal::digital::v2::OutputPin,
    DIR1B: embedded_hal::digital::v2::OutputPin,
    DIR2A: embedded_hal::digital::v2::OutputPin,
    DIR2B: embedded_hal::digital::v2::OutputPin,
{
    /// 创建双电机控制器
    pub fn new(motor1: MotorController<PWM1, DIR1A, DIR1B>, motor2: MotorController<PWM2, DIR2A, DIR2B>) -> Self {
        Self { motor1, motor2 }
    }
    
    /// 直行
    pub fn move_forward(&mut self, speed: u8) {
        self.motor1.set_direction(MotorDirection::Forward);
        self.motor2.set_direction(MotorDirection::Forward);
        self.motor1.set_speed(speed);
        self.motor2.set_speed(speed);
    }
    
    /// 后退
    pub fn move_backward(&mut self, speed: u8) {
        self.motor1.set_direction(MotorDirection::Reverse);
        self.motor2.set_direction(MotorDirection::Reverse);
        self.motor1.set_speed(speed);
        self.motor2.set_speed(speed);
    }
    
    /// 左转
    pub fn turn_left(&mut self, speed: u8) {
        self.motor1.set_direction(MotorDirection::Reverse);
        self.motor2.set_direction(MotorDirection::Forward);
        self.motor1.set_speed(speed);
        self.motor2.set_speed(speed);
    }
    
    /// 右转
    pub fn turn_right(&mut self, speed: u8) {
        self.motor1.set_direction(MotorDirection::Forward);
        self.motor2.set_direction(MotorDirection::Reverse);
        self.motor1.set_speed(speed);
        self.motor2.set_speed(speed);
    }
    
    /// 差速转弯
    pub fn differential_turn(&mut self, left_speed: u8, right_speed: u8) {
        self.motor1.set_direction(MotorDirection::Forward);
        self.motor2.set_direction(MotorDirection::Forward);
        self.motor1.set_speed(left_speed);
        self.motor2.set_speed(right_speed);
    }
    
    /// 停止所有电机
    pub fn stop_all(&mut self) {
        self.motor1.stop();
        self.motor2.stop();
    }
    
    /// 刹车所有电机
    pub fn brake_all(&mut self) {
        self.motor1.brake();
        self.motor2.brake();
    }
}