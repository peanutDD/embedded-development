#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Channel},
    gpio::{
        gpioa::PA8,
        gpiob::{PB0, PB1, PB2, PB3, PB4, PB5},
        gpioc::{PC13, PC14, PC15},
        Alternate, AF1, Output, PushPull, Input, PullUp
    },
};
use motor_driver::{DcMotorController, MotorDirection};

type MotorPwm = PA8<Alternate<AF1>>;
type MotorDir1 = PB0<Output<PushPull>>;
type MotorDir2 = PB1<Output<PushPull>>;
type StatusLed = PB2<Output<PushPull>>;
type DirectionLed = PB3<Output<PushPull>>;
type SpeedLed = PB4<Output<PushPull>>;
type ForwardButton = PC13<Input<PullUp>>;
type BackwardButton = PC14<Input<PullUp>>;
type StopButton = PC15<Input<PullUp>>;

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
    let gpioc = dp.GPIOC.split();
    
    let motor_pwm = gpioa.pa8.into_alternate_af1();
    let motor_dir1 = gpiob.pb0.into_push_pull_output();
    let motor_dir2 = gpiob.pb1.into_push_pull_output();
    
    let mut status_led = gpiob.pb2.into_push_pull_output();
    let mut direction_led = gpiob.pb3.into_push_pull_output();
    let mut speed_led = gpiob.pb4.into_push_pull_output();
    
    let forward_button = gpioc.pc13.into_pull_up_input();
    let backward_button = gpioc.pc14.into_pull_up_input();
    let stop_button = gpioc.pc15.into_pull_up_input();

    // 配置定时器1用于PWM输出
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    let mut pwm = timer.pwm(motor_pwm, 1.khz());
    pwm.enable(Channel::C1);

    // 创建直流电机控制器
    let mut motor = DcMotorController::new(pwm.split().0, motor_dir1, motor_dir2);
    motor.set_max_speed(100.0);

    // 创建延时定时器
    let mut delay = Timer::tim2(dp.TIM2, &clocks).delay_us();

    // 系统启动指示
    status_led.set_high();
    delay.delay_ms(1000u32);

    // 按钮状态跟踪
    let mut last_forward = false;
    let mut last_backward = false;
    let mut last_stop = false;

    // 电机控制状态
    let mut current_speed = 0.0f32;
    let mut target_speed = 0.0f32;
    let mut control_mode = ControlMode::Manual;
    let mut demo_step = 0;
    let mut demo_timer = 0u32;

    loop {
        // 读取按钮状态
        let forward_pressed = forward_button.is_low();
        let backward_pressed = backward_button.is_low();
        let stop_pressed = stop_button.is_low();

        // 检测按钮按下事件
        let forward_clicked = forward_pressed && !last_forward;
        let backward_clicked = backward_pressed && !last_backward;
        let stop_clicked = stop_pressed && !last_stop;

        match control_mode {
            ControlMode::Manual => {
                // 手动控制模式
                if forward_clicked {
                    target_speed = if target_speed >= 0.0 { 
                        (target_speed + 20.0).min(100.0) 
                    } else { 
                        20.0 
                    };
                } else if backward_clicked {
                    target_speed = if target_speed <= 0.0 { 
                        (target_speed - 20.0).max(-100.0) 
                    } else { 
                        -20.0 
                    };
                } else if stop_clicked {
                    if target_speed != 0.0 {
                        target_speed = 0.0;
                    } else {
                        // 切换到演示模式
                        control_mode = ControlMode::Demo;
                        demo_step = 0;
                        demo_timer = 0;
                        
                        // 模式切换指示
                        for _ in 0..3 {
                            status_led.set_low();
                            delay.delay_ms(200u32);
                            status_led.set_high();
                            delay.delay_ms(200u32);
                        }
                    }
                }

                // 平滑加速/减速
                smooth_speed_control(&mut current_speed, target_speed, 2.0);
            }

            ControlMode::Demo => {
                // 演示模式
                demo_timer += 1;
                
                match demo_step {
                    0 => {
                        // 正向加速
                        target_speed = 50.0;
                        if demo_timer > 100 {
                            demo_step = 1;
                            demo_timer = 0;
                        }
                    }
                    1 => {
                        // 正向全速
                        target_speed = 100.0;
                        if demo_timer > 100 {
                            demo_step = 2;
                            demo_timer = 0;
                        }
                    }
                    2 => {
                        // 减速停止
                        target_speed = 0.0;
                        if demo_timer > 100 {
                            demo_step = 3;
                            demo_timer = 0;
                        }
                    }
                    3 => {
                        // 反向加速
                        target_speed = -50.0;
                        if demo_timer > 100 {
                            demo_step = 4;
                            demo_timer = 0;
                        }
                    }
                    4 => {
                        // 反向全速
                        target_speed = -100.0;
                        if demo_timer > 100 {
                            demo_step = 5;
                            demo_timer = 0;
                        }
                    }
                    5 => {
                        // 急停演示
                        motor.brake();
                        delay.delay_ms(500u32);
                        demo_step = 6;
                        demo_timer = 0;
                    }
                    6 => {
                        // 速度变化演示
                        let phase = (demo_timer as f32 * 0.1).sin();
                        target_speed = phase * 80.0;
                        if demo_timer > 314 { // 约一个周期
                            demo_step = 0;
                            demo_timer = 0;
                        }
                    }
                    _ => {
                        demo_step = 0;
                    }
                }

                smooth_speed_control(&mut current_speed, target_speed, 1.0);

                // 检查退出演示模式
                if stop_clicked {
                    control_mode = ControlMode::Manual;
                    target_speed = 0.0;
                    
                    // 模式切换指示
                    for _ in 0..2 {
                        status_led.set_low();
                        delay.delay_ms(300u32);
                        status_led.set_high();
                        delay.delay_ms(300u32);
                    }
                }
            }
        }

        // 应用电机速度
        motor.set_speed(current_speed).ok();

        // 更新LED指示
        update_leds(
            &motor,
            current_speed,
            &mut status_led,
            &mut direction_led,
            &mut speed_led,
        );

        // 更新按钮状态
        last_forward = forward_pressed;
        last_backward = backward_pressed;
        last_stop = stop_pressed;

        delay.delay_ms(50u32);
    }
}

/// 平滑速度控制
fn smooth_speed_control(current: &mut f32, target: f32, acceleration: f32) {
    let error = target - *current;
    let step = if error.abs() < acceleration {
        error
    } else if error > 0.0 {
        acceleration
    } else {
        -acceleration
    };
    
    *current += step;
}

/// 更新LED指示
fn update_leds<PWM, DIR1, DIR2>(
    motor: &DcMotorController<PWM, DIR1, DIR2>,
    speed: f32,
    status_led: &mut impl embedded_hal::digital::v2::OutputPin,
    direction_led: &mut impl embedded_hal::digital::v2::OutputPin,
    speed_led: &mut impl embedded_hal::digital::v2::OutputPin,
) where
    PWM: embedded_hal::PwmPin<Duty = u32>,
    DIR1: embedded_hal::digital::v2::OutputPin,
    DIR2: embedded_hal::digital::v2::OutputPin,
{
    // 状态LED：运行时常亮
    if speed.abs() > 1.0 {
        status_led.set_high().ok();
    } else {
        status_led.set_low().ok();
    }

    // 方向LED：正向亮，反向灭
    match motor.get_direction() {
        MotorDirection::Forward => direction_led.set_high().ok(),
        MotorDirection::Backward => direction_led.set_low().ok(),
        _ => {
            // 停止时闪烁
            static mut BLINK_COUNTER: u32 = 0;
            unsafe {
                BLINK_COUNTER += 1;
                if BLINK_COUNTER % 20 < 10 {
                    direction_led.set_high().ok();
                } else {
                    direction_led.set_low().ok();
                }
            }
        }
    }

    // 速度LED：根据速度PWM闪烁
    static mut SPEED_COUNTER: u32 = 0;
    unsafe {
        SPEED_COUNTER += 1;
        let threshold = (100.0 - speed.abs()) as u32;
        if SPEED_COUNTER % 100 < threshold {
            speed_led.set_low().ok();
        } else {
            speed_led.set_high().ok();
        }
    }
}

/// 控制模式
#[derive(Clone, Copy)]
enum ControlMode {
    Manual,
    Demo,
}