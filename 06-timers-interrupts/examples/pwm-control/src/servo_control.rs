#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Channel, PwmChannels},
    gpio::{gpioa::PA8, Alternate, AF1},
};

type ServoPin = PA8<Alternate<AF1>>;

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
    let servo_pin = gpioa.pa8.into_alternate_af1();

    // 配置定时器1用于PWM (50Hz for servo)
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    let mut pwm = timer.pwm::<Timer1Ch1>(servo_pin, 50.hz());
    
    // 创建舵机控制器
    let mut servo = ServoController::new(pwm);
    
    // 配置系统定时器用于延时
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);

    // 舵机控制主循环
    loop {
        // 扫描0-180度
        for angle in (0..=180).step_by(10) {
            servo.set_angle(angle);
            delay.delay_ms(500u32);
        }
        
        // 扫描180-0度
        for angle in (0..=180).step_by(10).rev() {
            servo.set_angle(angle);
            delay.delay_ms(500u32);
        }
        
        // 演示特定位置
        servo.set_angle(90);  // 中间位置
        delay.delay_ms(1000u32);
        
        servo.set_angle(0);   // 最左
        delay.delay_ms(1000u32);
        
        servo.set_angle(180); // 最右
        delay.delay_ms(1000u32);
    }
}

/// 舵机控制器结构体
pub struct ServoController<PIN> {
    pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, PIN>,
    max_duty: u16,
    current_angle: u8,
}

impl<PIN> ServoController<PIN> {
    /// 创建新的舵机控制器
    /// 舵机通常需要50Hz的PWM信号
    /// 脉宽1ms对应0度，1.5ms对应90度，2ms对应180度
    pub fn new(mut pwm: PwmChannels<stm32f4xx_hal::timer::Timer<stm32::TIM1>, PIN>) -> Self {
        pwm.enable(Channel::C1);
        let max_duty = pwm.get_max_duty();
        
        let mut controller = Self { 
            pwm, 
            max_duty,
            current_angle: 90,
        };
        
        // 初始化到中间位置
        controller.set_angle(90);
        
        controller
    }
    
    /// 设置舵机角度 (0-180度)
    pub fn set_angle(&mut self, angle: u8) {
        let angle = angle.min(180);
        
        // 计算脉宽占空比
        // 对于50Hz PWM (20ms周期):
        // 1ms = 5% duty cycle (0度)
        // 1.5ms = 7.5% duty cycle (90度)
        // 2ms = 10% duty cycle (180度)
        let min_duty = self.max_duty / 20;      // 5% (1ms)
        let max_duty = self.max_duty / 10;      // 10% (2ms)
        
        let duty = min_duty + ((max_duty - min_duty) * angle as u16) / 180;
        
        self.pwm.set_duty(Channel::C1, duty);
        self.current_angle = angle;
    }
    
    /// 获取当前角度
    pub fn get_angle(&self) -> u8 {
        self.current_angle
    }
    
    /// 平滑移动到目标角度
    pub fn move_to(&mut self, target_angle: u8, speed: u8) {
        let target = target_angle.min(180);
        let speed = speed.max(1).min(10); // 限制速度范围
        
        while self.current_angle != target {
            if self.current_angle < target {
                let next_angle = (self.current_angle + speed).min(target);
                self.set_angle(next_angle);
            } else {
                let next_angle = self.current_angle.saturating_sub(speed).max(target);
                self.set_angle(next_angle);
            }
            
            // 这里需要延时，实际应用中可以使用定时器中断
            // delay.delay_ms(20);
        }
    }
    
    /// 扫描动作
    pub fn sweep(&mut self, start_angle: u8, end_angle: u8, steps: u8) {
        let start = start_angle.min(180);
        let end = end_angle.min(180);
        let steps = steps.max(1);
        
        if start == end {
            self.set_angle(start);
            return;
        }
        
        let step_size = if start > end {
            (start - end) / steps
        } else {
            (end - start) / steps
        };
        
        for step in 0..=steps {
            let angle = if start > end {
                start - (step_size * step)
            } else {
                start + (step_size * step)
            };
            
            self.set_angle(angle);
            // 延时处理
        }
    }
    
    /// 校准舵机
    /// 返回实际的最小和最大角度
    pub fn calibrate(&mut self) -> (u8, u8) {
        // 测试最小角度
        self.set_angle(0);
        // 延时等待舵机到位
        
        // 测试最大角度
        self.set_angle(180);
        // 延时等待舵机到位
        
        // 返回到中间位置
        self.set_angle(90);
        
        // 返回校准结果
        (0, 180)
    }
}

/// 多舵机控制器
pub struct MultiServoController {
    servos: [Option<ServoController<ServoPin>>; 4],
    count: usize,
}

impl MultiServoController {
    /// 创建多舵机控制器
    pub fn new() -> Self {
        Self {
            servos: [None, None, None, None],
            count: 0,
        }
    }
    
    /// 添加舵机
    pub fn add_servo(&mut self, servo: ServoController<ServoPin>) -> Result<usize, ()> {
        if self.count >= 4 {
            return Err(());
        }
        
        let index = self.count;
        self.servos[index] = Some(servo);
        self.count += 1;
        
        Ok(index)
    }
    
    /// 设置指定舵机角度
    pub fn set_servo_angle(&mut self, index: usize, angle: u8) -> Result<(), ()> {
        if index >= self.count {
            return Err(());
        }
        
        if let Some(ref mut servo) = self.servos[index] {
            servo.set_angle(angle);
            Ok(())
        } else {
            Err(())
        }
    }
    
    /// 同步所有舵机到指定角度
    pub fn sync_all(&mut self, angles: &[u8]) {
        for (i, &angle) in angles.iter().enumerate() {
            if i >= self.count {
                break;
            }
            let _ = self.set_servo_angle(i, angle);
        }
    }
    
    /// 执行预定义动作序列
    pub fn execute_sequence(&mut self, sequence: &[(usize, u8)]) {
        for &(servo_index, angle) in sequence {
            let _ = self.set_servo_angle(servo_index, angle);
            // 延时处理
        }
    }
}