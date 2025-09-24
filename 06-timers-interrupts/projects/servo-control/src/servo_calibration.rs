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
        gpiob::{PB0, PB1, PB2}, 
        gpioc::{PC13, PC14, PC15},
        Alternate, AF1, Output, PushPull, Input, PullUp
    },
};
use servo_control::{ServoController, ServoCalibrator, ServoConfig};

type ServoPin = PA8<Alternate<AF1>>;
type StatusLed = PB0<Output<PushPull>>;
type CalibrationLed = PB1<Output<PushPull>>;
type ErrorLed = PB2<Output<PushPull>>;
type CalibrationButton = PC13<Input<PullUp>>;
type NextButton = PC14<Input<PullUp>>;
type SaveButton = PC15<Input<PullUp>>;

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
    
    let servo_pin = gpioa.pa8.into_alternate_af1();
    let mut status_led = gpiob.pb0.into_push_pull_output();
    let mut calibration_led = gpiob.pb1.into_push_pull_output();
    let mut error_led = gpiob.pb2.into_push_pull_output();
    
    let calibration_button = gpioc.pc13.into_pull_up_input();
    let next_button = gpioc.pc14.into_pull_up_input();
    let save_button = gpioc.pc15.into_pull_up_input();

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

    // 创建舵机校准器
    let mut calibrator = ServoCalibrator::new(servo);

    // 创建延时定时器
    let mut delay = Timer::tim2(dp.TIM2, &clocks).delay_us();

    // 系统启动指示
    status_led.set_high();
    delay.delay_ms(1000u32);

    // 校准点定义
    let calibration_angles = [0.0, 45.0, 90.0, 135.0, 180.0];
    let mut calibration_index = 0;
    let mut calibration_mode = CalibrationMode::WaitingForStart;

    // 按钮状态跟踪
    let mut last_calibration_button = false;
    let mut last_next_button = false;
    let mut last_save_button = false;

    loop {
        // 读取按钮状态
        let calibration_pressed = calibration_button.is_low();
        let next_pressed = next_button.is_low();
        let save_pressed = save_button.is_low();

        // 检测按钮按下事件（边沿触发）
        let calibration_clicked = calibration_pressed && !last_calibration_button;
        let next_clicked = next_pressed && !last_next_button;
        let save_clicked = save_pressed && !last_save_button;

        match calibration_mode {
            CalibrationMode::WaitingForStart => {
                // 等待开始校准
                status_led.set_high();
                calibration_led.set_low();
                error_led.set_low();

                if calibration_clicked {
                    calibration_mode = CalibrationMode::Positioning;
                    calibration_index = 0;
                    
                    // 移动到第一个校准点
                    calibrator.set_calibrated_angle(calibration_angles[calibration_index]).ok();
                    
                    // 指示开始校准
                    for _ in 0..3 {
                        calibration_led.set_high();
                        delay.delay_ms(200u32);
                        calibration_led.set_low();
                        delay.delay_ms(200u32);
                    }
                }
            }

            CalibrationMode::Positioning => {
                // 定位到校准点
                calibration_led.set_high();
                
                if next_clicked {
                    // 记录当前校准点
                    let target_angle = calibration_angles[calibration_index];
                    // 在实际应用中，这里应该从用户输入或传感器获取实际角度
                    let actual_angle = target_angle; // 简化处理
                    
                    match calibrator.add_calibration_point(target_angle, actual_angle) {
                        Ok(_) => {
                            // 成功添加校准点
                            calibration_led.set_low();
                            delay.delay_ms(100u32);
                            calibration_led.set_high();
                            delay.delay_ms(100u32);
                            calibration_led.set_low();
                            
                            calibration_index += 1;
                            
                            if calibration_index < calibration_angles.len() {
                                // 移动到下一个校准点
                                calibrator.set_calibrated_angle(calibration_angles[calibration_index]).ok();
                            } else {
                                // 所有校准点完成
                                calibration_mode = CalibrationMode::Testing;
                            }
                        }
                        Err(_) => {
                            // 校准点添加失败
                            error_led.set_high();
                            delay.delay_ms(1000u32);
                            error_led.set_low();
                        }
                    }
                }

                if save_clicked {
                    // 跳过当前校准点
                    calibration_index += 1;
                    if calibration_index < calibration_angles.len() {
                        calibrator.set_calibrated_angle(calibration_angles[calibration_index]).ok();
                    } else {
                        calibration_mode = CalibrationMode::Testing;
                    }
                }
            }

            CalibrationMode::Testing => {
                // 测试校准结果
                status_led.set_low();
                calibration_led.set_low();
                
                // 测试所有角度
                for &angle in &calibration_angles {
                    calibrator.set_calibrated_angle(angle).ok();
                    
                    // LED指示当前测试角度
                    let blink_count = (angle / 45.0) as u32 + 1;
                    for _ in 0..blink_count {
                        status_led.set_high();
                        delay.delay_ms(200u32);
                        status_led.set_low();
                        delay.delay_ms(200u32);
                    }
                    
                    delay.delay_ms(1000u32);
                }

                if save_clicked {
                    // 保存校准并完成
                    calibration_mode = CalibrationMode::Completed;
                    
                    // 保存成功指示
                    for _ in 0..5 {
                        calibration_led.set_high();
                        delay.delay_ms(100u32);
                        calibration_led.set_low();
                        delay.delay_ms(100u32);
                    }
                } else if calibration_clicked {
                    // 重新开始校准
                    calibration_mode = CalibrationMode::WaitingForStart;
                    calibration_index = 0;
                }
            }

            CalibrationMode::Completed => {
                // 校准完成，正常运行模式
                status_led.set_high();
                
                // 演示校准后的精确控制
                for &angle in &[0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0] {
                    calibrator.set_calibrated_angle(angle).ok();
                    delay.delay_ms(500u32);
                }

                if calibration_clicked {
                    // 重新开始校准
                    calibration_mode = CalibrationMode::WaitingForStart;
                    calibration_index = 0;
                }
            }
        }

        // 更新按钮状态
        last_calibration_button = calibration_pressed;
        last_next_button = next_pressed;
        last_save_button = save_pressed;

        delay.delay_ms(50u32);
    }
}

/// 校准模式
#[derive(Clone, Copy)]
enum CalibrationMode {
    WaitingForStart,
    Positioning,
    Testing,
    Completed,
}