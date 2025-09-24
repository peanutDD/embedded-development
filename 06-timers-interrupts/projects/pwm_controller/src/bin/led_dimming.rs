#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, PushPull, Pin},
    timer::{Timer, Channel1, Channel2, Channel3, Channel4, PwmHz},
    rcc::Clocks,
};
use heapless::spsc::{Queue, Producer, Consumer};
use rtt_target::{rprintln, rtt_init_print};
use pwm_controller::{LedDimmer, PwmChannel};
use libm::{sinf, cosf, fabsf, powf};

// LED调光事件类型
#[derive(Debug, Clone, Copy)]
enum DimmingEvent {
    BrightnessChange(f32),
    FadeToLevel(f32, f32), // 目标亮度, 淡入淡出速度
    ToggleGammaCorrection,
    PatternChange(DimmingPattern),
}

#[derive(Debug, Clone, Copy)]
enum DimmingPattern {
    Static,      // 静态亮度
    Breathing,   // 呼吸灯效果
    Pulsing,     // 脉冲效果
    Rainbow,     // 彩虹效果（多通道）
    Strobe,      // 频闪效果
    Fade,        // 淡入淡出
}

// PWM通道包装器
struct PwmChannelWrapper<TIM, CHANNEL> {
    pwm: PwmHz<TIM, CHANNEL>,
    duty_cycle: f32,
    frequency: u32,
}

impl<TIM, CHANNEL> PwmChannelWrapper<TIM, CHANNEL>
where
    PwmHz<TIM, CHANNEL>: embedded_hal::pwm::SetDutyCycle,
{
    fn new(pwm: PwmHz<TIM, CHANNEL>, frequency: u32) -> Self {
        Self {
            pwm,
            duty_cycle: 0.0,
            frequency,
        }
    }
}

impl<TIM, CHANNEL> PwmChannel for PwmChannelWrapper<TIM, CHANNEL>
where
    PwmHz<TIM, CHANNEL>: embedded_hal::pwm::SetDutyCycle,
{
    type Error = ();

    fn set_duty_cycle(&mut self, duty: f32) -> Result<(), Self::Error> {
        self.duty_cycle = duty.clamp(0.0, 1.0);
        let max_duty = self.pwm.get_max_duty();
        let duty_value = (max_duty as f32 * self.duty_cycle) as u16;
        self.pwm.set_duty(duty_value);
        Ok(())
    }

    fn set_frequency(&mut self, freq: u32) -> Result<(), Self::Error> {
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
        self.duty_cycle
    }

    fn get_frequency(&self) -> u32 {
        self.frequency
    }
}

// LED调光管理器
struct LedDimmingManager<PWM1, PWM2, PWM3, PWM4> {
    red_dimmer: LedDimmer<PWM1>,
    green_dimmer: LedDimmer<PWM2>,
    blue_dimmer: LedDimmer<PWM3>,
    white_dimmer: LedDimmer<PWM4>,
    current_pattern: DimmingPattern,
    pattern_phase: f32,
    pattern_speed: f32,
    statistics: DimmingStatistics,
    event_producer: Producer<'static, DimmingEvent, 32>,
}

#[derive(Debug, Default)]
struct DimmingStatistics {
    total_runtime: u32,
    pattern_changes: u32,
    brightness_changes: u32,
    gamma_corrections: u32,
    max_brightness: f32,
    average_brightness: f32,
    power_consumption: f32,
}

impl<PWM1, PWM2, PWM3, PWM4> LedDimmingManager<PWM1, PWM2, PWM3, PWM4>
where
    PWM1: PwmChannel,
    PWM2: PwmChannel,
    PWM3: PwmChannel,
    PWM4: PwmChannel,
{
    fn new(
        red_dimmer: LedDimmer<PWM1>,
        green_dimmer: LedDimmer<PWM2>,
        blue_dimmer: LedDimmer<PWM3>,
        white_dimmer: LedDimmer<PWM4>,
        event_producer: Producer<'static, DimmingEvent, 32>,
    ) -> Self {
        Self {
            red_dimmer,
            green_dimmer,
            blue_dimmer,
            white_dimmer,
            current_pattern: DimmingPattern::Static,
            pattern_phase: 0.0,
            pattern_speed: 1.0,
            statistics: DimmingStatistics::default(),
            event_producer,
        }
    }

    fn set_pattern(&mut self, pattern: DimmingPattern) {
        self.current_pattern = pattern;
        self.pattern_phase = 0.0;
        self.statistics.pattern_changes += 1;
        rprintln!("LED模式切换到: {:?}", pattern);
    }

    fn update_pattern(&mut self, dt_ms: u32) -> Result<(), &'static str> {
        let dt = dt_ms as f32 / 1000.0; // 转换为秒
        self.pattern_phase += dt * self.pattern_speed;

        match self.current_pattern {
            DimmingPattern::Static => {
                // 静态模式，保持当前亮度
            }
            DimmingPattern::Breathing => {
                // 呼吸灯效果
                let brightness = (sinf(self.pattern_phase * 2.0) + 1.0) / 2.0;
                self.set_all_brightness(brightness)?;
            }
            DimmingPattern::Pulsing => {
                // 脉冲效果
                let pulse = if sinf(self.pattern_phase * 4.0) > 0.0 { 1.0 } else { 0.1 };
                self.set_all_brightness(pulse)?;
            }
            DimmingPattern::Rainbow => {
                // 彩虹效果（RGB循环）
                let red = (sinf(self.pattern_phase) + 1.0) / 2.0;
                let green = (sinf(self.pattern_phase + 2.094) + 1.0) / 2.0;   // 120度相位差
                let blue = (sinf(self.pattern_phase + 4.188) + 1.0) / 2.0;    // 240度相位差
                
                self.red_dimmer.set_brightness(red)?;
                self.green_dimmer.set_brightness(green)?;
                self.blue_dimmer.set_brightness(blue)?;
                self.white_dimmer.set_brightness(0.0)?;
            }
            DimmingPattern::Strobe => {
                // 频闪效果
                let strobe_freq = 10.0; // 10Hz频闪
                let brightness = if (self.pattern_phase * strobe_freq) % 1.0 < 0.1 { 1.0 } else { 0.0 };
                self.set_all_brightness(brightness)?;
            }
            DimmingPattern::Fade => {
                // 淡入淡出效果
                let fade_cycle = self.pattern_phase % 4.0;
                let brightness = if fade_cycle < 2.0 {
                    fade_cycle / 2.0  // 淡入
                } else {
                    2.0 - (fade_cycle - 2.0) / 2.0  // 淡出
                };
                self.set_all_brightness(brightness)?;
            }
        }

        // 更新所有调光器
        self.red_dimmer.update(dt_ms)?;
        self.green_dimmer.update(dt_ms)?;
        self.blue_dimmer.update(dt_ms)?;
        self.white_dimmer.update(dt_ms)?;

        Ok(())
    }

    fn set_all_brightness(&mut self, brightness: f32) -> Result<(), &'static str> {
        self.red_dimmer.set_brightness(brightness)?;
        self.green_dimmer.set_brightness(brightness)?;
        self.blue_dimmer.set_brightness(brightness)?;
        self.white_dimmer.set_brightness(brightness)?;
        Ok(())
    }

    fn fade_all_to(&mut self, brightness: f32, speed: f32) -> Result<(), &'static str> {
        self.red_dimmer.fade_to(brightness, speed)?;
        self.green_dimmer.fade_to(brightness, speed)?;
        self.blue_dimmer.fade_to(brightness, speed)?;
        self.white_dimmer.fade_to(brightness, speed)?;
        Ok(())
    }

    fn toggle_gamma_correction(&mut self) {
        let enabled = !self.red_dimmer.get_brightness().is_nan(); // 简化的状态检查
        self.red_dimmer.set_gamma_correction(enabled);
        self.green_dimmer.set_gamma_correction(enabled);
        self.blue_dimmer.set_gamma_correction(enabled);
        self.white_dimmer.set_gamma_correction(enabled);
        self.statistics.gamma_corrections += 1;
        rprintln!("伽马校正: {}", if enabled { "开启" } else { "关闭" });
    }

    fn update_statistics(&mut self, dt_ms: u32) {
        self.statistics.total_runtime += dt_ms;
        
        // 计算平均亮度
        let total_brightness = self.red_dimmer.get_brightness() +
                              self.green_dimmer.get_brightness() +
                              self.blue_dimmer.get_brightness() +
                              self.white_dimmer.get_brightness();
        let avg_brightness = total_brightness / 4.0;
        
        if avg_brightness > self.statistics.max_brightness {
            self.statistics.max_brightness = avg_brightness;
        }
        
        // 更新平均亮度（移动平均）
        self.statistics.average_brightness = 
            (self.statistics.average_brightness * 0.99) + (avg_brightness * 0.01);
        
        // 估算功耗（简化计算）
        self.statistics.power_consumption = total_brightness * 0.5; // 假设每通道最大0.5W
    }

    fn send_event(&mut self, event: DimmingEvent) {
        if self.event_producer.enqueue(event).is_err() {
            rprintln!("警告: 事件队列已满");
        }
    }
}

static mut DIMMING_EVENT_QUEUE: Queue<DimmingEvent, 32> = Queue::new();

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("LED调光系统启动");

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 配置按钮（模式切换）
    let button = gpioc.pc13.into_pull_up_input();

    // 配置LED指示灯
    let mut status_led = gpiob.pb0.into_push_pull_output();      // 系统状态
    let mut dimming_led = gpiob.pb1.into_push_pull_output();     // 调光活动指示
    let mut red_status = gpiob.pb2.into_push_pull_output();      // 红色通道状态
    let mut green_status = gpiob.pb3.into_push_pull_output();    // 绿色通道状态
    let mut blue_status = gpiob.pb4.into_push_pull_output();     // 蓝色通道状态
    let mut white_status = gpiob.pb5.into_push_pull_output();    // 白色通道状态
    let mut pattern_led = gpiob.pb6.into_push_pull_output();     // 模式指示
    let mut error_led = gpiob.pb7.into_push_pull_output();       // 错误指示

    // 配置PWM引脚（RGBW四通道）
    let pwm_pins_tim2 = (
        gpioa.pa0.into_alternate(),  // TIM2_CH1 - 红色
        gpioa.pa1.into_alternate(),  // TIM2_CH2 - 绿色
    );
    
    let pwm_pins_tim3 = (
        gpioa.pa6.into_alternate(),  // TIM3_CH1 - 蓝色
        gpioa.pa7.into_alternate(),  // TIM3_CH2 - 白色
    );

    // 配置定时器
    let mut timer2 = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer2.start(1.kHz()).unwrap();

    // 配置PWM（高频率以避免闪烁）
    let pwm_freq = 1.kHz(); // 1kHz PWM频率，适合LED调光
    let mut pwm2 = dp.TIM2.pwm_hz(pwm_pins_tim2, pwm_freq, &clocks);
    let mut pwm3 = dp.TIM3.pwm_hz(pwm_pins_tim3, pwm_freq, &clocks);
    
    let (pwm2_ch1, pwm2_ch2) = pwm2.split();
    let (pwm3_ch1, pwm3_ch2) = pwm3.split();
    
    let red_pwm = PwmChannelWrapper::new(pwm2_ch1, 1000);
    let green_pwm = PwmChannelWrapper::new(pwm2_ch2, 1000);
    let blue_pwm = PwmChannelWrapper::new(pwm3_ch1, 1000);
    let white_pwm = PwmChannelWrapper::new(pwm3_ch2, 1000);

    // 创建事件队列
    let (event_producer, mut event_consumer) = unsafe {
        DIMMING_EVENT_QUEUE.split()
    };

    // 创建LED调光器
    let red_dimmer = LedDimmer::new(red_pwm);
    let green_dimmer = LedDimmer::new(green_pwm);
    let blue_dimmer = LedDimmer::new(blue_pwm);
    let white_dimmer = LedDimmer::new(white_pwm);
    
    // 创建LED调光管理器
    let mut dimming_manager = LedDimmingManager::new(
        red_dimmer, green_dimmer, blue_dimmer, white_dimmer, event_producer
    );

    // 系统定时器
    let mut delay = cp.SYST.delay(&clocks);
    let mut last_button_state = button.is_high();
    let mut button_debounce = 0u32;
    let mut system_tick = 0u32;
    let mut demo_phase = 0u32;

    rprintln!("LED调光系统就绪");

    loop {
        let current_time = system_tick;
        system_tick = system_tick.wrapping_add(1);

        // 按钮处理（模式切换）
        let button_state = button.is_high();
        if button_state != last_button_state {
            button_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button_debounce) > 50 && button_state && !last_button_state {
            // 按钮按下，切换调光模式
            let new_pattern = match dimming_manager.current_pattern {
                DimmingPattern::Static => DimmingPattern::Breathing,
                DimmingPattern::Breathing => DimmingPattern::Pulsing,
                DimmingPattern::Pulsing => DimmingPattern::Rainbow,
                DimmingPattern::Rainbow => DimmingPattern::Strobe,
                DimmingPattern::Strobe => DimmingPattern::Fade,
                DimmingPattern::Fade => DimmingPattern::Static,
            };
            dimming_manager.send_event(DimmingEvent::PatternChange(new_pattern));
        }
        last_button_state = button_state;

        // 处理事件队列
        while let Some(event) = event_consumer.dequeue() {
            match event {
                DimmingEvent::BrightnessChange(brightness) => {
                    if let Err(e) = dimming_manager.set_all_brightness(brightness) {
                        rprintln!("亮度设置错误: {}", e);
                        error_led.set_high();
                    } else {
                        dimming_manager.statistics.brightness_changes += 1;
                        error_led.set_low();
                    }
                }
                DimmingEvent::FadeToLevel(brightness, speed) => {
                    if let Err(e) = dimming_manager.fade_all_to(brightness, speed) {
                        rprintln!("淡入淡出错误: {}", e);
                        error_led.set_high();
                    } else {
                        error_led.set_low();
                    }
                }
                DimmingEvent::ToggleGammaCorrection => {
                    dimming_manager.toggle_gamma_correction();
                }
                DimmingEvent::PatternChange(pattern) => {
                    dimming_manager.set_pattern(pattern);
                }
            }
        }

        // 更新调光模式
        if let Err(e) = dimming_manager.update_pattern(10) {
            rprintln!("模式更新错误: {}", e);
            error_led.set_high();
        }

        // 更新统计信息
        dimming_manager.update_statistics(10);

        // LED指示更新
        // 系统状态LED（心跳）
        if system_tick % 100 == 0 {
            status_led.toggle();
        }

        // 调光活动指示
        let total_brightness = dimming_manager.red_dimmer.get_brightness() +
                              dimming_manager.green_dimmer.get_brightness() +
                              dimming_manager.blue_dimmer.get_brightness() +
                              dimming_manager.white_dimmer.get_brightness();
        
        if total_brightness > 0.1 {
            dimming_led.set_high();
        } else {
            dimming_led.set_low();
        }

        // 各通道状态指示
        if dimming_manager.red_dimmer.get_brightness() > 0.1 {
            red_status.set_high();
        } else {
            red_status.set_low();
        }

        if dimming_manager.green_dimmer.get_brightness() > 0.1 {
            green_status.set_high();
        } else {
            green_status.set_low();
        }

        if dimming_manager.blue_dimmer.get_brightness() > 0.1 {
            blue_status.set_high();
        } else {
            blue_status.set_low();
        }

        if dimming_manager.white_dimmer.get_brightness() > 0.1 {
            white_status.set_high();
        } else {
            white_status.set_low();
        }

        // 模式指示（不同模式不同闪烁频率）
        let pattern_blink_period = match dimming_manager.current_pattern {
            DimmingPattern::Static => 1000,
            DimmingPattern::Breathing => 500,
            DimmingPattern::Pulsing => 200,
            DimmingPattern::Rainbow => 100,
            DimmingPattern::Strobe => 50,
            DimmingPattern::Fade => 300,
        };
        
        if system_tick % pattern_blink_period < pattern_blink_period / 2 {
            pattern_led.set_high();
        } else {
            pattern_led.set_low();
        }

        // 定期输出统计信息
        if system_tick % 5000 == 0 {
            rprintln!("=== LED调光统计信息 ===");
            rprintln!("运行时间: {}ms", dimming_manager.statistics.total_runtime);
            rprintln!("模式切换次数: {}", dimming_manager.statistics.pattern_changes);
            rprintln!("亮度调整次数: {}", dimming_manager.statistics.brightness_changes);
            rprintln!("伽马校正次数: {}", dimming_manager.statistics.gamma_corrections);
            rprintln!("最大亮度: {:.2}", dimming_manager.statistics.max_brightness);
            rprintln!("平均亮度: {:.2}", dimming_manager.statistics.average_brightness);
            rprintln!("估算功耗: {:.2}W", dimming_manager.statistics.power_consumption);
            rprintln!("当前模式: {:?}", dimming_manager.current_pattern);
            rprintln!("红色: {:.2}, 绿色: {:.2}, 蓝色: {:.2}, 白色: {:.2}",
                     dimming_manager.red_dimmer.get_brightness(),
                     dimming_manager.green_dimmer.get_brightness(),
                     dimming_manager.blue_dimmer.get_brightness(),
                     dimming_manager.white_dimmer.get_brightness());
        }

        demo_phase = demo_phase.wrapping_add(1);
        delay.delay_ms(10u32);
    }
}