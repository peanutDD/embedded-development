#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::asm;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    timer::{Timer, Channel1, Channel2, Channel3, Channel4},
    gpio::{Output, PushPull, Pin, Alternate},
    rcc::Clocks,
    pwm::{PwmChannel as HalPwmChannel, C1, C2, C3, C4},
};
use rtt_target::{rprintln, rtt_init_print};
use heapless::Vec;

use pwm_controller::{
    PwmChannel, MultiChannelPwmController, ServoController, MotorDriver, 
    LedDimmer, WaveformGenerator, WaveformType, ChannelConfig
};

/// STM32 PWM通道包装器
pub struct Stm32PwmChannel<TIM, const C: u8> {
    pwm: HalPwmChannel<TIM, C>,
    frequency: u32,
    max_duty: u16,
}

impl<TIM, const C: u8> Stm32PwmChannel<TIM, C>
where
    TIM: pac::timer::Instance,
{
    pub fn new(pwm: HalPwmChannel<TIM, C>, frequency: u32) -> Self {
        let max_duty = pwm.get_max_duty();
        Self {
            pwm,
            frequency,
            max_duty,
        }
    }
}

impl<TIM, const C: u8> PwmChannel for Stm32PwmChannel<TIM, C>
where
    TIM: pac::timer::Instance,
{
    type Error = ();

    fn set_duty_cycle(&mut self, duty: f32) -> Result<(), Self::Error> {
        let duty_value = (duty * self.max_duty as f32) as u16;
        self.pwm.set_duty(duty_value);
        Ok(())
    }

    fn set_frequency(&mut self, freq: u32) -> Result<(), Self::Error> {
        // 注意：在实际应用中，改变频率需要重新配置定时器
        // 这里只是更新记录的频率值
        self.frequency = freq;
        Ok(())
    }

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.pwm.enable();
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.pwm.disable();
        Ok(())
    }

    fn get_duty_cycle(&self) -> f32 {
        self.pwm.get_duty() as f32 / self.max_duty as f32
    }

    fn get_frequency(&self) -> u32 {
        self.frequency
    }
}

/// 系统状态
struct SystemState {
    tick_count: u32,
    demo_mode: DemoMode,
    mode_timer: u32,
}

/// 演示模式
#[derive(Debug, Clone, Copy)]
enum DemoMode {
    ServoSweep,
    MotorControl,
    LedFading,
    WaveformGeneration,
    MultiChannel,
}

impl DemoMode {
    fn next(self) -> Self {
        match self {
            DemoMode::ServoSweep => DemoMode::MotorControl,
            DemoMode::MotorControl => DemoMode::LedFading,
            DemoMode::LedFading => DemoMode::WaveformGeneration,
            DemoMode::WaveformGeneration => DemoMode::MultiChannel,
            DemoMode::MultiChannel => DemoMode::ServoSweep,
        }
    }

    fn name(self) -> &'static str {
        match self {
            DemoMode::ServoSweep => "Servo Sweep",
            DemoMode::MotorControl => "Motor Control",
            DemoMode::LedFading => "LED Fading",
            DemoMode::WaveformGeneration => "Waveform Generation",
            DemoMode::MultiChannel => "Multi-Channel",
        }
    }
}

#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    rprintln!("PWM Controller Demo Starting...");

    // 获取外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(168.mhz())
        .pclk1(42.mhz())
        .pclk2(84.mhz())
        .freeze();

    rprintln!("System Clock: {} MHz", clocks.sysclk().to_MHz());

    // 配置GPIO引脚
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // TIM1 PWM引脚 (高级定时器)
    let tim1_ch1 = gpioa.pa8.into_alternate(); // TIM1_CH1
    let tim1_ch2 = gpioa.pa9.into_alternate(); // TIM1_CH2
    let tim1_ch3 = gpioa.pa10.into_alternate(); // TIM1_CH3
    let tim1_ch4 = gpioa.pa11.into_alternate(); // TIM1_CH4

    // TIM2 PWM引脚 (通用定时器)
    let tim2_ch1 = gpioa.pa0.into_alternate(); // TIM2_CH1
    let tim2_ch2 = gpioa.pa1.into_alternate(); // TIM2_CH2

    // TIM3 PWM引脚 (通用定时器)
    let tim3_ch1 = gpioa.pa6.into_alternate(); // TIM3_CH1
    let tim3_ch2 = gpioa.pa7.into_alternate(); // TIM3_CH2

    // 配置定时器
    let tim1 = Timer::new(dp.TIM1, &clocks);
    let tim2 = Timer::new(dp.TIM2, &clocks);
    let tim3 = Timer::new(dp.TIM3, &clocks);

    // 配置PWM - 1kHz频率
    let pwm1 = tim1.pwm_hz((tim1_ch1, tim1_ch2, tim1_ch3, tim1_ch4), 1.khz(), &clocks);
    let (mut pwm1_ch1, mut pwm1_ch2, mut pwm1_ch3, mut pwm1_ch4) = pwm1;

    let pwm2 = tim2.pwm_hz((tim2_ch1, tim2_ch2), 1.khz(), &clocks);
    let (mut pwm2_ch1, mut pwm2_ch2) = pwm2;

    let pwm3 = tim3.pwm_hz((tim3_ch1, tim3_ch2), 50.hz(), &clocks); // 50Hz用于舵机
    let (mut pwm3_ch1, mut pwm3_ch2) = pwm3;

    // 启用PWM输出
    pwm1_ch1.enable();
    pwm1_ch2.enable();
    pwm1_ch3.enable();
    pwm1_ch4.enable();
    pwm2_ch1.enable();
    pwm2_ch2.enable();
    pwm3_ch1.enable();
    pwm3_ch2.enable();

    rprintln!("PWM channels initialized");

    // 创建PWM通道包装器
    let stm32_pwm1_ch1 = Stm32PwmChannel::new(pwm1_ch1, 1000);
    let stm32_pwm1_ch2 = Stm32PwmChannel::new(pwm1_ch2, 1000);
    let stm32_pwm1_ch3 = Stm32PwmChannel::new(pwm1_ch3, 1000);
    let stm32_pwm1_ch4 = Stm32PwmChannel::new(pwm1_ch4, 1000);
    let stm32_pwm2_ch1 = Stm32PwmChannel::new(pwm2_ch1, 1000);
    let stm32_pwm2_ch2 = Stm32PwmChannel::new(pwm2_ch2, 1000);
    let stm32_pwm3_ch1 = Stm32PwmChannel::new(pwm3_ch1, 50);
    let stm32_pwm3_ch2 = Stm32PwmChannel::new(pwm3_ch2, 50);

    // 创建控制器
    let mut servo1 = ServoController::new(stm32_pwm3_ch1, 1000, 2000); // SG90舵机
    let mut servo2 = ServoController::new(stm32_pwm3_ch2, 1000, 2000);
    
    let mut motor = MotorDriver::new(stm32_pwm2_ch1, stm32_pwm2_ch2);
    motor.set_acceleration(50.0); // 50%/秒加速度
    
    let mut led_dimmer = LedDimmer::new(stm32_pwm1_ch1);
    led_dimmer.set_gamma_correction(true);
    
    let mut waveform_gen = WaveformGenerator::new(stm32_pwm1_ch2, 1000);
    
    // 多通道控制器
    let mut multi_pwm: MultiChannelPwmController<Stm32PwmChannel<pac::TIM1, 3>, 2> = 
        MultiChannelPwmController::new();
    
    let config1 = ChannelConfig {
        frequency: 1000,
        duty_cycle: 0.0,
        phase_offset: 0.0,
        enabled: true,
        inverted: false,
    };
    
    let config2 = ChannelConfig {
        frequency: 1000,
        duty_cycle: 0.0,
        phase_offset: 90.0, // 90度相位差
        enabled: true,
        inverted: false,
    };
    
    multi_pwm.add_channel(0, stm32_pwm1_ch3, config1).unwrap();
    multi_pwm.add_channel(1, stm32_pwm1_ch4, config2).unwrap();
    multi_pwm.enable_sync();

    // 系统状态
    let mut system_state = SystemState {
        tick_count: 0,
        demo_mode: DemoMode::ServoSweep,
        mode_timer: 0,
    };

    // 演示变量
    let mut servo_angle = 0.0f32;
    let mut servo_direction = 1.0f32;
    let mut motor_speed = 0.0f32;
    let mut motor_direction = 1.0f32;
    let mut led_brightness = 0.0f32;
    let mut led_direction = 1.0f32;
    let mut waveform_time = 0.0f32;
    let mut multi_phase = 0.0f32;

    rprintln!("Starting demo loop...");

    loop {
        system_state.tick_count += 1;
        system_state.mode_timer += 1;

        // 每10秒切换演示模式
        if system_state.mode_timer >= 10000 {
            system_state.demo_mode = system_state.demo_mode.next();
            system_state.mode_timer = 0;
            rprintln!("Switching to: {}", system_state.demo_mode.name());
        }

        match system_state.demo_mode {
            DemoMode::ServoSweep => {
                // 舵机扫描演示
                servo_angle += servo_direction * 0.5;
                if servo_angle >= 180.0 {
                    servo_angle = 180.0;
                    servo_direction = -1.0;
                } else if servo_angle <= 0.0 {
                    servo_angle = 0.0;
                    servo_direction = 1.0;
                }
                
                servo1.set_angle(servo_angle).unwrap();
                servo2.set_angle(180.0 - servo_angle).unwrap(); // 反向运动
                
                if system_state.tick_count % 1000 == 0 {
                    rprintln!("Servo angles: {:.1}°, {:.1}°", servo_angle, 180.0 - servo_angle);
                }
            },

            DemoMode::MotorControl => {
                // 电机控制演示
                motor_speed += motor_direction * 0.2;
                if motor_speed >= 100.0 {
                    motor_speed = 100.0;
                    motor_direction = -1.0;
                } else if motor_speed <= -100.0 {
                    motor_speed = -100.0;
                    motor_direction = 1.0;
                }
                
                motor.set_speed(motor_speed).unwrap();
                motor.update(1).unwrap(); // 1ms更新间隔
                
                if system_state.tick_count % 1000 == 0 {
                    rprintln!("Motor speed: {:.1}%", motor.get_current_speed());
                }
            },

            DemoMode::LedFading => {
                // LED淡入淡出演示
                led_brightness += led_direction * 0.5;
                if led_brightness >= 100.0 {
                    led_brightness = 100.0;
                    led_direction = -1.0;
                } else if led_brightness <= 0.0 {
                    led_brightness = 0.0;
                    led_direction = 1.0;
                }
                
                led_dimmer.set_brightness(led_brightness).unwrap();
                led_dimmer.update(1).unwrap(); // 1ms更新间隔
                
                if system_state.tick_count % 1000 == 0 {
                    rprintln!("LED brightness: {:.1}%", led_dimmer.get_brightness());
                }
            },

            DemoMode::WaveformGeneration => {
                // 波形生成演示
                waveform_time += 0.001; // 1ms时间步长
                
                // 每2.5秒切换波形类型
                let waveform_type = match (system_state.mode_timer / 2500) % 4 {
                    0 => WaveformType::Sine,
                    1 => WaveformType::Square,
                    2 => WaveformType::Triangle,
                    _ => WaveformType::Sawtooth,
                };
                
                waveform_gen.set_waveform(waveform_type, 0.5, 2.0, 0.5); // 2Hz波形
                waveform_gen.generate_sample(waveform_time).unwrap();
                
                if system_state.tick_count % 2500 == 0 {
                    let waveform_name = match waveform_type {
                        WaveformType::Sine => "Sine",
                        WaveformType::Square => "Square",
                        WaveformType::Triangle => "Triangle",
                        WaveformType::Sawtooth => "Sawtooth",
                        _ => "Custom",
                    };
                    rprintln!("Generating {} wave", waveform_name);
                }
            },

            DemoMode::MultiChannel => {
                // 多通道同步演示
                multi_phase += 0.01;
                if multi_phase >= 2.0 * core::f32::consts::PI {
                    multi_phase = 0.0;
                }
                
                let duty1 = 0.5 + 0.4 * libm::sin(multi_phase);
                let duty2 = 0.5 + 0.4 * libm::sin(multi_phase + core::f32::consts::PI / 2.0); // 90度相位差
                
                multi_pwm.set_duty_cycle(0, duty1).unwrap();
                multi_pwm.set_duty_cycle(1, duty2).unwrap();
                multi_pwm.sync_update().unwrap();
                
                if system_state.tick_count % 1000 == 0 {
                    rprintln!("Multi-channel duties: {:.2}, {:.2}", duty1, duty2);
                }
            },
        }

        // 系统状态输出
        if system_state.tick_count % 5000 == 0 {
            rprintln!("System tick: {}, Mode: {}", 
                system_state.tick_count, 
                system_state.demo_mode.name()
            );
        }

        // 1ms延时
        asm::delay(168_000); // 168MHz系统时钟，1ms延时
    }
}