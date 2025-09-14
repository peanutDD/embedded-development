//! 基础控制器模板
//! 
//! 这是一个通用的嵌入式控制器模板，提供了常见的功能模块：
//! - GPIO 控制
//! - 串口通信
//! - 定时器
//! - ADC 采样
//! - 状态机管理
//! - 错误处理
//! 
//! 可以基于此模板快速开发各种控制器应用

#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

use stm32f4xx_hal::{
    adc::{Adc, config::AdcConfig},
    gpio::{gpioa::PA0, gpioc::PC13, Analog, Input, Output, PushPull},
    pac::{ADC1, TIM2, USART2},
    prelude::*,
    serial::{config::Config, Serial},
    timer::{CounterUs, Event},
};

use heapless::{String, Vec};
use nb::block;

// 系统配置常量
const SYSTEM_FREQ: u32 = 84_000_000; // 84 MHz
const TIMER_FREQ: u32 = 1000; // 1 kHz (1ms)
const ADC_SAMPLES: usize = 10;
const SERIAL_BAUD: u32 = 115200;

// 控制器状态枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum ControllerState {
    Idle,
    Running,
    Error,
    Maintenance,
}

// 系统错误类型
#[derive(Debug, Clone, Copy)]
enum SystemError {
    AdcError,
    SerialError,
    TimerError,
    InvalidState,
}

// 控制参数结构体
#[derive(Debug, Clone, Copy)]
struct ControlParameters {
    setpoint: u16,
    kp: f32,
    ki: f32,
    kd: f32,
    output_limit: u16,
}

impl Default for ControlParameters {
    fn default() -> Self {
        Self {
            setpoint: 2048, // 中点值
            kp: 1.0,
            ki: 0.1,
            kd: 0.01,
            output_limit: 4095,
        }
    }
}

// PID 控制器结构体
#[derive(Debug, Clone, Copy)]
struct PidController {
    params: ControlParameters,
    integral: f32,
    previous_error: f32,
    output: u16,
}

impl PidController {
    fn new(params: ControlParameters) -> Self {
        Self {
            params,
            integral: 0.0,
            previous_error: 0.0,
            output: 0,
        }
    }

    fn update(&mut self, input: u16, dt: f32) -> u16 {
        let error = self.params.setpoint as f32 - input as f32;
        
        // 比例项
        let proportional = self.params.kp * error;
        
        // 积分项
        self.integral += error * dt;
        let integral = self.params.ki * self.integral;
        
        // 微分项
        let derivative = self.params.kd * (error - self.previous_error) / dt;
        self.previous_error = error;
        
        // PID 输出
        let output = proportional + integral + derivative;
        
        // 限幅
        self.output = if output < 0.0 {
            0
        } else if output > self.params.output_limit as f32 {
            self.params.output_limit
        } else {
            output as u16
        };
        
        self.output
    }

    fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.output = 0;
    }
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        controller_state: ControllerState,
        pid_controller: PidController,
        adc_value: u16,
        system_error: Option<SystemError>,
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        button: PA0<Input>,
        adc: Adc<ADC1>,
        adc_pin: PA0<Analog>,
        serial: Serial<USART2>,
        timer: CounterUs<TIM2>,
        tick_counter: u32,
        adc_samples: Vec<u16, ADC_SAMPLES>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // 初始化 RTT 调试输出
        rtt_init_print!();
        rprintln!("🚀 基础控制器模板启动中...");

        let dp = ctx.device;

        // 配置时钟
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(SYSTEM_FREQ.hz())
            .pclk1(42.mhz())
            .pclk2(84.mhz())
            .freeze();

        rprintln!("⚡ 系统时钟配置完成: {} MHz", SYSTEM_FREQ / 1_000_000);

        // 配置 GPIO
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        // LED (PC13)
        let led = gpioc.pc13.into_push_pull_output();

        // 按键 (PA0)
        let button = gpioa.pa0.into_pull_up_input();

        // ADC 配置
        let adc_pin = gpioa.pa1.into_analog();
        let adc_config = AdcConfig::default()
            .resolution(stm32f4xx_hal::adc::config::Resolution::Twelve)
            .scan(stm32f4xx_hal::adc::config::Scan::Disabled);
        let mut adc = Adc::adc1(dp.ADC1, true, adc_config);
        adc.set_sample_time(stm32f4xx_hal::adc::config::SampleTime::Cycles_480);

        // 串口配置 (USART2: PA2=TX, PA3=RX)
        let tx_pin = gpioa.pa2.into_alternate();
        let rx_pin = gpioa.pa3.into_alternate();
        let serial_config = Config::default().baudrate(SERIAL_BAUD.bps());
        let serial = Serial::new(dp.USART2, (tx_pin, rx_pin), serial_config, &clocks).unwrap();

        // 定时器配置
        let mut timer = dp.TIM2.counter_us(&clocks);
        timer.start(1000.micros()).unwrap(); // 1ms 周期
        timer.listen(Event::Update);

        rprintln!("🔧 外设初始化完成");

        // 初始化 PID 控制器
        let control_params = ControlParameters::default();
        let pid_controller = PidController::new(control_params);

        rprintln!("🎛️  PID 控制器初始化完成");
        rprintln!("📊 控制参数: Kp={}, Ki={}, Kd={}", 
                 control_params.kp, control_params.ki, control_params.kd);

        // 启动系统任务
        system_monitor::spawn().ok();
        data_acquisition::spawn().ok();
        
        rprintln!("✅ 系统启动完成，进入运行状态");

        (
            Shared {
                controller_state: ControllerState::Running,
                pid_controller,
                adc_value: 0,
                system_error: None,
            },
            Local {
                led,
                button,
                adc,
                adc_pin,
                serial,
                timer,
                tick_counter: 0,
                adc_samples: Vec::new(),
            },
            init::Monotonics(),
        )
    }

    // 1ms 定时器中断 - 系统心跳
    #[task(binds = TIM2, local = [timer, tick_counter, led], shared = [controller_state])]
    fn timer_tick(mut ctx: timer_tick::Context) {
        ctx.local.timer.clear_interrupt(Event::Update);
        *ctx.local.tick_counter += 1;

        // 每秒切换 LED 状态
        if *ctx.local.tick_counter % 1000 == 0 {
            ctx.local.led.toggle();
            
            // 检查系统状态
            ctx.shared.controller_state.lock(|state| {
                match *state {
                    ControllerState::Running => {
                        // 正常运行，LED 慢闪
                    },
                    ControllerState::Error => {
                        // 错误状态，LED 快闪
                        ctx.local.led.toggle();
                    },
                    _ => {}
                }
            });
        }

        // 每 10ms 触发控制任务
        if *ctx.local.tick_counter % 10 == 0 {
            control_loop::spawn().ok();
        }

        // 每 100ms 触发数据采集
        if *ctx.local.tick_counter % 100 == 0 {
            data_acquisition::spawn().ok();
        }

        // 每秒触发系统监控
        if *ctx.local.tick_counter % 1000 == 0 {
            system_monitor::spawn().ok();
        }
    }

    // 数据采集任务
    #[task(local = [adc, adc_pin, adc_samples], shared = [adc_value, system_error])]
    fn data_acquisition(mut ctx: data_acquisition::Context) {
        // 读取 ADC 值
        match ctx.local.adc.convert(ctx.local.adc_pin, stm32f4xx_hal::adc::config::SampleTime::Cycles_480) {
            Ok(sample) => {
                // 添加到样本缓冲区
                if ctx.local.adc_samples.len() >= ADC_SAMPLES {
                    ctx.local.adc_samples.clear();
                }
                ctx.local.adc_samples.push(sample).ok();

                // 计算平均值
                if ctx.local.adc_samples.len() == ADC_SAMPLES {
                    let sum: u32 = ctx.local.adc_samples.iter().map(|&x| x as u32).sum();
                    let average = (sum / ADC_SAMPLES as u32) as u16;

                    ctx.shared.adc_value.lock(|value| {
                        *value = average;
                    });
                }
            },
            Err(_) => {
                ctx.shared.system_error.lock(|error| {
                    *error = Some(SystemError::AdcError);
                });
            }
        }
    }

    // 控制回路任务
    #[task(shared = [pid_controller, adc_value, controller_state])]
    fn control_loop(mut ctx: control_loop::Context) {
        let current_value = ctx.shared.adc_value.lock(|value| *value);
        
        ctx.shared.controller_state.lock(|state| {
            if *state == ControllerState::Running {
                ctx.shared.pid_controller.lock(|pid| {
                    let output = pid.update(current_value, 0.01); // 10ms = 0.01s
                    
                    // 这里可以添加输出到 DAC 或 PWM 的代码
                    // 例如：dac.set_value(output);
                    
                    // 调试输出
                    if current_value != 0 {
                        rprintln!("🎛️  控制回路: 输入={}, 输出={}", current_value, output);
                    }
                });
            }
        });
    }

    // 系统监控任务
    #[task(local = [serial], shared = [controller_state, adc_value, pid_controller, system_error])]
    fn system_monitor(mut ctx: system_monitor::Context) {
        let state = ctx.shared.controller_state.lock(|s| *s);
        let adc_val = ctx.shared.adc_value.lock(|v| *v);
        let error = ctx.shared.system_error.lock(|e| *e);

        // 发送系统状态报告
        let mut status_msg: String<128> = String::new();
        
        match write!(&mut status_msg, "STATUS: {:?}, ADC: {}, ", state, adc_val) {
            Ok(_) => {
                if let Some(err) = error {
                    write!(&mut status_msg, "ERROR: {:?}", err).ok();
                } else {
                    write!(&mut status_msg, "OK").ok();
                }
                status_msg.push('\n').ok();

                // 通过串口发送状态
                for byte in status_msg.as_bytes() {
                    block!(ctx.local.serial.write(*byte)).ok();
                }
            },
            Err(_) => {
                // 字符串格式化失败
            }
        }

        // RTT 调试输出
        rprintln!("📊 系统状态: {:?}, ADC: {}", state, adc_val);
        
        if let Some(err) = error {
            rprintln!("⚠️  系统错误: {:?}", err);
            
            // 清除错误状态
            ctx.shared.system_error.lock(|e| *e = None);
        }

        // 输出 PID 参数
        ctx.shared.pid_controller.lock(|pid| {
            rprintln!("🎛️  PID 输出: {}", pid.output);
        });
    }

    // 按键中断处理
    #[task(binds = EXTI0, local = [button], shared = [controller_state, pid_controller])]
    fn button_handler(mut ctx: button_handler::Context) {
        // 清除中断标志
        // 注意：这里需要根据实际的中断控制器实现
        
        if ctx.local.button.is_low() {
            ctx.shared.controller_state.lock(|state| {
                *state = match *state {
                    ControllerState::Idle => ControllerState::Running,
                    ControllerState::Running => ControllerState::Idle,
                    ControllerState::Error => ControllerState::Idle,
                    ControllerState::Maintenance => ControllerState::Running,
                };
                
                rprintln!("🔘 按键按下，状态切换到: {:?}", *state);
            });

            // 重置 PID 控制器
            ctx.shared.pid_controller.lock(|pid| {
                pid.reset();
            });
        }
    }

    // 空闲任务
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // 进入低功耗模式
            cortex_m::asm::wfi();
        }
    }
}

// 格式化宏支持
use core::fmt::Write;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("💥 系统崩溃: {}", info);
    
    loop {
        cortex_m::asm::bkpt();
    }
}