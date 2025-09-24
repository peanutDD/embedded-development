#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Channel},
    gpio::{gpioa::PA8, gpiob::{PB0, PB1}, Alternate, AF1, Output, PushPull},
};
use servo_control::{ServoController, ServoSweeper, ServoConfig, SweepDirection};

type ServoPin = PA8<Alternate<AF1>>;
type DirectionLed = PB0<Output<PushPull>>;
type StatusLed = PB1<Output<PushPull>>;

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
    let gpiob = dp.GPIOB.split();
    let servo_pin = gpioa.pa8.into_alternate_af1();
    let mut direction_led = gpiob.pb0.into_push_pull_output();
    let mut status_led = gpiob.pb1.into_push_pull_output();

    // 配置定时器1用于PWM输出
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    let mut pwm = timer.pwm(servo_pin, 50.hz());
    pwm.enable(Channel::C1);

    // 创建舵机控制器
    let config = ServoConfig::sg90();
    let servo = ServoController::new(
        pwm.split().0,
        config.min_pulse_us,
        config.max_pulse_us,
        config.max_angle,
        config.pwm_frequency,
    );

    // 创建舵机扫描器
    let mut sweeper = ServoSweeper::new(servo, 0.0, 180.0, 2.0);

    // 创建延时定时器
    let mut delay = Timer::tim2(dp.TIM2, &clocks).delay_us();

    // 状态LED闪烁表示系统运行
    status_led.set_high();

    let mut step_count = 0;
    let mut last_direction = SweepDirection::Forward;

    loop {
        // 执行扫描步骤
        match sweeper.step() {
            Ok(angle) => {
                // 根据扫描方向控制LED
                match sweeper.get_direction() {
                    SweepDirection::Forward => {
                        direction_led.set_high();
                        if matches!(last_direction, SweepDirection::Backward) {
                            // 方向改变，状态LED闪烁
                            status_led.set_low();
                            delay.delay_ms(100u32);
                            status_led.set_high();
                        }
                    }
                    SweepDirection::Backward => {
                        direction_led.set_low();
                        if matches!(last_direction, SweepDirection::Forward) {
                            // 方向改变，状态LED闪烁
                            status_led.set_low();
                            delay.delay_ms(100u32);
                            status_led.set_high();
                        }
                    }
                }

                last_direction = sweeper.get_direction();
                step_count += 1;

                // 每100步改变扫描参数
                if step_count % 100 == 0 {
                    // 创建新的扫描器，改变扫描范围和步长
                    let new_min = if step_count % 200 == 0 { 30.0 } else { 0.0 };
                    let new_max = if step_count % 200 == 0 { 150.0 } else { 180.0 };
                    let new_step = if step_count % 300 == 0 { 1.0 } else { 2.0 };

                    // 重新创建扫描器
                    let config = ServoConfig::sg90();
                    let servo = ServoController::new(
                        pwm.split().0,
                        config.min_pulse_us,
                        config.max_pulse_us,
                        config.max_angle,
                        config.pwm_frequency,
                    );
                    sweeper = ServoSweeper::new(servo, new_min, new_max, new_step);

                    // 指示参数改变
                    for _ in 0..3 {
                        status_led.set_low();
                        delay.delay_ms(200u32);
                        status_led.set_high();
                        delay.delay_ms(200u32);
                    }
                }
            }
            Err(_) => {
                // 错误指示
                status_led.set_low();
                delay.delay_ms(500u32);
                status_led.set_high();
            }
        }

        // 扫描速度控制
        delay.delay_ms(50u32);

        // 每1000步进行一次完整的扫描演示
        if step_count % 1000 == 0 {
            full_range_demo(&mut sweeper, &mut delay, &mut status_led);
        }
    }
}

/// 完整范围扫描演示
fn full_range_demo<PWM>(
    sweeper: &mut ServoSweeper<PWM>,
    delay: &mut impl embedded_hal::blocking::delay::DelayMs<u32>,
    status_led: &mut impl embedded_hal::digital::v2::OutputPin,
) where
    PWM: embedded_hal::PwmPin<Duty = u32>,
{
    // 快速扫描演示
    for _ in 0..50 {
        sweeper.step().ok();
        delay.delay_ms(20u32);
    }

    // 状态指示
    for _ in 0..5 {
        status_led.set_low().ok();
        delay.delay_ms(100u32);
        status_led.set_high().ok();
        delay.delay_ms(100u32);
    }

    // 慢速扫描演示
    for _ in 0..20 {
        sweeper.step().ok();
        delay.delay_ms(200u32);
    }
}