#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Channel},
    gpio::{gpioa::{PA8, PA9, PA10, PA11}, Alternate, AF1},
};
use servo_control::{ServoController, MultiServoController, ServoConfig};

type ServoPin1 = PA8<Alternate<AF1>>;
type ServoPin2 = PA9<Alternate<AF1>>;
type ServoPin3 = PA10<Alternate<AF1>>;
type ServoPin4 = PA11<Alternate<AF1>>;

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
    let servo_pin1 = gpioa.pa8.into_alternate_af1();
    let servo_pin2 = gpioa.pa9.into_alternate_af1();
    let servo_pin3 = gpioa.pa10.into_alternate_af1();
    let servo_pin4 = gpioa.pa11.into_alternate_af1();

    // 配置定时器1用于PWM输出
    let mut timer1 = Timer::tim1(dp.TIM1, &clocks);
    let mut pwm1 = timer1.pwm((servo_pin1, servo_pin2, servo_pin3, servo_pin4), 50.hz());
    pwm1.enable(Channel::C1);
    pwm1.enable(Channel::C2);
    pwm1.enable(Channel::C3);
    pwm1.enable(Channel::C4);

    let (pwm_ch1, pwm_ch2, pwm_ch3, pwm_ch4) = pwm1.split();

    // 创建舵机配置
    let config = ServoConfig::sg90();

    // 创建单个舵机控制器
    let servo1 = ServoController::new(
        pwm_ch1,
        config.min_pulse_us,
        config.max_pulse_us,
        config.max_angle,
        config.pwm_frequency,
    );

    let servo2 = ServoController::new(
        pwm_ch2,
        config.min_pulse_us,
        config.max_pulse_us,
        config.max_angle,
        config.pwm_frequency,
    );

    let servo3 = ServoController::new(
        pwm_ch3,
        config.min_pulse_us,
        config.max_pulse_us,
        config.max_angle,
        config.pwm_frequency,
    );

    let servo4 = ServoController::new(
        pwm_ch4,
        config.min_pulse_us,
        config.max_pulse_us,
        config.max_angle,
        config.pwm_frequency,
    );

    // 创建多舵机控制器
    let mut multi_servo = MultiServoController::new();
    multi_servo.add_servo1(servo1);
    multi_servo.add_servo2(servo2);
    multi_servo.add_servo3(servo3);
    multi_servo.add_servo4(servo4);

    // 创建延时定时器
    let mut delay = Timer::tim2(dp.TIM2, &clocks).delay_us();

    // 预定义的动作序列
    let sequences = [
        [0.0, 0.0, 0.0, 0.0],       // 初始位置
        [90.0, 90.0, 90.0, 90.0],   // 中间位置
        [180.0, 180.0, 180.0, 180.0], // 最大位置
        [45.0, 135.0, 45.0, 135.0], // 交替位置
        [180.0, 0.0, 180.0, 0.0],   // 对角位置
    ];

    let mut sequence_index = 0;

    loop {
        // 执行当前序列
        let target_angles = sequences[sequence_index];
        
        // 同步移动所有舵机
        multi_servo.sync_move(target_angles, 30).ok();
        
        // 为每一步提供延时
        for _ in 0..30 {
            delay.delay_ms(50u32);
        }

        // 保持位置1秒
        delay.delay_ms(1000u32);

        // 切换到下一个序列
        sequence_index = (sequence_index + 1) % sequences.len();

        // 波浪动作演示
        if sequence_index == 0 {
            wave_motion(&mut multi_servo, &mut delay);
        }
    }
}

/// 波浪动作演示
fn wave_motion<PWM1, PWM2, PWM3, PWM4>(
    multi_servo: &mut MultiServoController<PWM1, PWM2, PWM3, PWM4>,
    delay: &mut impl embedded_hal::blocking::delay::DelayMs<u32>,
) where
    PWM1: embedded_hal::PwmPin<Duty = u32>,
    PWM2: embedded_hal::PwmPin<Duty = u32>,
    PWM3: embedded_hal::PwmPin<Duty = u32>,
    PWM4: embedded_hal::PwmPin<Duty = u32>,
{
    // 创建波浪效果
    for phase in 0..360 {
        let angle1 = 90.0 + 45.0 * micromath::F32Ext::sin((phase as f32).to_radians());
        let angle2 = 90.0 + 45.0 * micromath::F32Ext::sin((phase as f32 + 90.0).to_radians());
        let angle3 = 90.0 + 45.0 * micromath::F32Ext::sin((phase as f32 + 180.0).to_radians());
        let angle4 = 90.0 + 45.0 * micromath::F32Ext::sin((phase as f32 + 270.0).to_radians());

        multi_servo.set_all_angles([angle1, angle2, angle3, angle4]).ok();
        delay.delay_ms(20u32);
    }
}