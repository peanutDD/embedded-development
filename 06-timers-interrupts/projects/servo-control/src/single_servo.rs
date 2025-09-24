#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Channel},
    gpio::{gpioa::PA8, Alternate, AF1},
};
use servo_control::{ServoController, ServoConfig};

type ServoPin = PA8<Alternate<AF1>>;

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
    let servo_pin = gpioa.pa8.into_alternate_af1();

    // 配置定时器1用于PWM输出
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    let mut pwm = timer.pwm(servo_pin, 50.hz());
    pwm.enable(Channel::C1);

    // 创建舵机控制器
    let config = ServoConfig::sg90();
    let mut servo = ServoController::new(
        pwm.split().0,
        config.min_pulse_us,
        config.max_pulse_us,
        config.max_angle,
        config.pwm_frequency,
    );

    // 创建延时定时器
    let mut delay = Timer::tim2(dp.TIM2, &clocks).delay_us();

    loop {
        // 移动到0度
        servo.set_angle(0.0).ok();
        delay.delay_ms(1000u32);

        // 移动到90度
        servo.set_angle(90.0).ok();
        delay.delay_ms(1000u32);

        // 移动到180度
        servo.set_angle(180.0).ok();
        delay.delay_ms(1000u32);

        // 平滑移动回0度
        servo.smooth_move(0.0, 20).ok();
        for _ in 0..20 {
            delay.delay_ms(50u32);
        }

        delay.delay_ms(1000u32);
    }
}