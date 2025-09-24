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
use pwm_controller::{PwmChannel, MultiChannelPwm};
use libm::{sinf, cosf, fabsf, fmodf};

// PWM控制事件类型
#[derive(Debug, Clone, Copy)]
enum PwmEvent {
    SetDutyCycle(u8, f32),       // 通道ID, 占空比
    SetFrequency(u8, u32),       // 通道ID, 频率
    SetPhase(u8, f32),           // 通道ID, 相位
    EnableChannel(u8),           // 通道ID
    DisableChannel(u8),          // 通道ID
    PatternChange(PwmPattern),   // 模式切换
    SyncChannels,                // 同步所有通道
}

#[derive(Debug, Clone, Copy)]
enum PwmPattern {
    Static,      // 静态输出
    Sweep,       // 扫频模式
    Phase,       // 相位控制
    Synchronized,// 同步模式
    Independent, // 独立控制
    Waveform,    // 波形生成
}

#[derive(Debug, Clone, Copy)]
enum WaveformType {
    Sine,        // 正弦波
    Triangle,    // 三角波
    Sawtooth,    // 锯齿波
    Square,      // 方波
    Custom,      // 自定义波形
}

// PWM通道包装器
struct PwmChannelWrapper<TIM, CHANNEL> {
    pwm: PwmHz<TIM, CHANNEL>,
    duty_cycle: f32,
    frequency: u32,
    phase: f32,
    enabled: bool,
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
            phase: 0.0,
            enabled: false,
        }
    }

    fn set_phase(&mut self, phase: f32) {
        self.phase = phase;
    }

    fn get_phase(&self) -> f32 {
        self.phase
    }
}

impl<TIM, CHANNEL> PwmChannel for PwmChannelWrapper<TIM, CHANNEL>
where
    PwmHz<TIM, CHANNEL>: embedded_hal::pwm::SetDutyCycle,
{
    type Error = ();

    fn set_duty_cycle(&mut self, duty: f32) -> Result<(), Self::Error> {
        self.duty_cycle = duty.clamp(0.0, 1.0);
        if self.enabled {
            let max_duty = self.pwm.get_max_duty();
            let duty_value = (max_duty as f32 * self.duty_cycle) as u16;
            self.pwm.set_duty(duty_value);
        }
        Ok(())
    }

    fn set_frequency(&mut self, freq: u32) -> Result<(), Self::Error> {
        self.frequency = freq;
        Ok(())
    }

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.enabled = true;
        self.pwm.enable();
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.enabled = false;
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

// 多通道PWM管理器
struct MultiChannelPwmManager<PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8> {
    channels: [Option<Box<dyn PwmChannel<Error = ()>>>; 8],
    current_pattern: PwmPattern,
    waveform_type: WaveformType,
    pattern_phase: f32,
    pattern_speed: f32,
    base_frequency: u32,
    sync_enabled: bool,
    statistics: PwmStatistics,
    event_producer: Producer<'static, PwmEvent, 64>,
}

#[derive(Debug, Default)]
struct PwmStatistics {
    total_runtime: u32,
    pattern_changes: u32,
    duty_cycle_changes: u32,
    frequency_changes: u32,
    phase_changes: u32,
    sync_operations: u32,
    channel_enables: u32,
    channel_disables: u32,
    total_cycles: u64,
    max_frequency: u32,
    min_frequency: u32,
    pwm_errors: u32,
}

impl<PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8> 
    MultiChannelPwmManager<PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8>
where
    PWM1: PwmChannel<Error = ()> + 'static,
    PWM2: PwmChannel<Error = ()> + 'static,
    PWM3: PwmChannel<Error = ()> + 'static,
    PWM4: PwmChannel<Error = ()> + 'static,
    PWM5: PwmChannel<Error = ()> + 'static,
    PWM6: PwmChannel<Error = ()> + 'static,
    PWM7: PwmChannel<Error = ()> + 'static,
    PWM8: PwmChannel<Error = ()> + 'static,
{
    fn new(
        ch1: PWM1, ch2: PWM2, ch3: PWM3, ch4: PWM4,
        ch5: PWM5, ch6: PWM6, ch7: PWM7, ch8: PWM8,
        event_producer: Producer<'static, PwmEvent, 64>,
    ) -> Self {
        let mut channels: [Option<Box<dyn PwmChannel<Error = ()>>>; 8] = [
            None, None, None, None, None, None, None, None
        ];
        
        channels[0] = Some(Box::new(ch1));
        channels[1] = Some(Box::new(ch2));
        channels[2] = Some(Box::new(ch3));
        channels[3] = Some(Box::new(ch4));
        channels[4] = Some(Box::new(ch5));
        channels[5] = Some(Box::new(ch6));
        channels[6] = Some(Box::new(ch7));
        channels[7] = Some(Box::new(ch8));

        Self {
            channels,
            current_pattern: PwmPattern::Static,
            waveform_type: WaveformType::Sine,
            pattern_phase: 0.0,
            pattern_speed: 1.0,
            base_frequency: 1000,
            sync_enabled: false,
            statistics: PwmStatistics {
                min_frequency: u32::MAX,
                ..Default::default()
            },
            event_producer,
        }
    }

    fn set_pattern(&mut self, pattern: PwmPattern) {
        self.current_pattern = pattern;
        self.pattern_phase = 0.0;
        self.statistics.pattern_changes += 1;
        rprintln!("PWM模式切换到: {:?}", pattern);
    }

    fn set_waveform_type(&mut self, waveform: WaveformType) {
        self.waveform_type = waveform;
        rprintln!("波形类型切换到: {:?}", waveform);
    }

    fn update_pattern(&mut self, dt_ms: u32) -> Result<(), &'static str> {
        let dt = dt_ms as f32 / 1000.0; // 转换为秒
        self.pattern_phase += dt * self.pattern_speed;

        match self.current_pattern {
            PwmPattern::Static => {
                // 静态模式，保持当前设置
            }
            PwmPattern::Sweep => {
                // 扫频模式：所有通道同步扫频
                let freq_ratio = (sinf(self.pattern_phase) + 1.0) / 2.0; // 0-1
                let frequency = (self.base_frequency as f32 * (0.1 + freq_ratio * 0.9)) as u32;
                
                for i in 0..8 {
                    if let Some(ref mut channel) = self.channels[i] {
                        channel.set_frequency(frequency)?;
                        channel.set_duty_cycle(0.5)?; // 50%占空比
                    }
                }
            }
            PwmPattern::Phase => {
                // 相位控制模式：不同通道不同相位
                for i in 0..8 {
                    if let Some(ref mut channel) = self.channels[i] {
                        let phase_offset = (i as f32) * 0.785; // 45度相位差
                        let phase_value = self.pattern_phase + phase_offset;
                        let duty = (sinf(phase_value) + 1.0) / 2.0; // 0-1
                        channel.set_duty_cycle(duty)?;
                        channel.set_frequency(self.base_frequency)?;
                    }
                }
            }
            PwmPattern::Synchronized => {
                // 同步模式：所有通道同步变化
                let duty = (sinf(self.pattern_phase) + 1.0) / 2.0; // 0-1
                for i in 0..8 {
                    if let Some(ref mut channel) = self.channels[i] {
                        channel.set_duty_cycle(duty)?;
                        channel.set_frequency(self.base_frequency)?;
                    }
                }
            }
            PwmPattern::Independent => {
                // 独立控制模式：每个通道独立变化
                for i in 0..8 {
                    if let Some(ref mut channel) = self.channels[i] {
                        let freq_mult = 1.0 + (i as f32) * 0.2; // 不同频率倍数
                        let phase_mult = 1.0 + (i as f32) * 0.3; // 不同相位倍数
                        
                        let duty = (sinf(self.pattern_phase * phase_mult) + 1.0) / 2.0;
                        let frequency = (self.base_frequency as f32 * freq_mult) as u32;
                        
                        channel.set_duty_cycle(duty)?;
                        channel.set_frequency(frequency)?;
                    }
                }
            }
            PwmPattern::Waveform => {
                // 波形生成模式
                self.generate_waveform()?;
            }
        }

        Ok(())
    }

    fn generate_waveform(&mut self) -> Result<(), &'static str> {
        for i in 0..8 {
            if let Some(ref mut channel) = self.channels[i] {
                let phase_offset = (i as f32) * 0.785; // 45度相位差
                let phase = self.pattern_phase + phase_offset;
                
                let duty = match self.waveform_type {
                    WaveformType::Sine => {
                        (sinf(phase) + 1.0) / 2.0
                    }
                    WaveformType::Triangle => {
                        let normalized_phase = fmodf(phase, 6.283) / 6.283; // 0-1
                        if normalized_phase < 0.5 {
                            normalized_phase * 2.0
                        } else {
                            2.0 - normalized_phase * 2.0
                        }
                    }
                    WaveformType::Sawtooth => {
                        fmodf(phase, 6.283) / 6.283
                    }
                    WaveformType::Square => {
                        if sinf(phase) > 0.0 { 1.0 } else { 0.0 }
                    }
                    WaveformType::Custom => {
                        // 自定义波形：正弦波 + 三次谐波
                        let fundamental = sinf(phase);
                        let harmonic = sinf(phase * 3.0) * 0.3;
                        (fundamental + harmonic + 1.0) / 2.0
                    }
                };
                
                channel.set_duty_cycle(duty.clamp(0.0, 1.0))?;
                channel.set_frequency(self.base_frequency)?;
            }
        }
        Ok(())
    }

    fn set_channel_duty_cycle(&mut self, channel_id: u8, duty: f32) -> Result<(), &'static str> {
        if channel_id >= 8 {
            return Err("无效的通道ID");
        }
        
        if let Some(ref mut channel) = self.channels[channel_id as usize] {
            channel.set_duty_cycle(duty)?;
            self.statistics.duty_cycle_changes += 1;
        }
        Ok(())
    }

    fn set_channel_frequency(&mut self, channel_id: u8, frequency: u32) -> Result<(), &'static str> {
        if channel_id >= 8 {
            return Err("无效的通道ID");
        }
        
        if let Some(ref mut channel) = self.channels[channel_id as usize] {
            channel.set_frequency(frequency)?;
            self.statistics.frequency_changes += 1;
            
            if frequency > self.statistics.max_frequency {
                self.statistics.max_frequency = frequency;
            }
            if frequency < self.statistics.min_frequency {
                self.statistics.min_frequency = frequency;
            }
        }
        Ok(())
    }

    fn enable_channel(&mut self, channel_id: u8) -> Result<(), &'static str> {
        if channel_id >= 8 {
            return Err("无效的通道ID");
        }
        
        if let Some(ref mut channel) = self.channels[channel_id as usize] {
            channel.enable()?;
            self.statistics.channel_enables += 1;
        }
        Ok(())
    }

    fn disable_channel(&mut self, channel_id: u8) -> Result<(), &'static str> {
        if channel_id >= 8 {
            return Err("无效的通道ID");
        }
        
        if let Some(ref mut channel) = self.channels[channel_id as usize] {
            channel.disable()?;
            self.statistics.channel_disables += 1;
        }
        Ok(())
    }

    fn sync_all_channels(&mut self) -> Result<(), &'static str> {
        // 同步所有通道到相同的相位
        self.pattern_phase = 0.0;
        self.statistics.sync_operations += 1;
        rprintln!("所有通道已同步");
        Ok(())
    }

    fn get_channel_status(&self, channel_id: u8) -> Result<(f32, u32), &'static str> {
        if channel_id >= 8 {
            return Err("无效的通道ID");
        }
        
        if let Some(ref channel) = self.channels[channel_id as usize] {
            Ok((channel.get_duty_cycle(), channel.get_frequency()))
        } else {
            Err("通道未初始化")
        }
    }

    fn update_statistics(&mut self, dt_ms: u32) {
        self.statistics.total_runtime += dt_ms;
        
        // 计算总周期数（简化计算）
        let cycles_per_second = self.base_frequency as u64;
        let cycles_this_update = cycles_per_second * (dt_ms as u64) / 1000;
        self.statistics.total_cycles += cycles_this_update;
    }

    fn send_event(&mut self, event: PwmEvent) {
        if self.event_producer.enqueue(event).is_err() {
            rprintln!("警告: 事件队列已满");
        }
    }
}

static mut PWM_EVENT_QUEUE: Queue<PwmEvent, 64> = Queue::new();

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("多通道PWM控制系统启动");

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 配置按钮
    let button1 = gpioc.pc13.into_pull_up_input(); // 模式切换
    let button2 = gpioc.pc14.into_pull_up_input(); // 波形切换

    // 配置LED指示灯
    let mut status_led = gpiob.pb0.into_push_pull_output();      // 系统状态
    let mut pattern_led = gpiob.pb1.into_push_pull_output();     // 模式指示
    let mut waveform_led = gpiob.pb2.into_push_pull_output();    // 波形指示
    let mut sync_led = gpiob.pb3.into_push_pull_output();        // 同步指示
    let mut ch1_led = gpiob.pb4.into_push_pull_output();         // 通道1活动
    let mut ch2_led = gpiob.pb5.into_push_pull_output();         // 通道2活动
    let mut ch3_led = gpiob.pb6.into_push_pull_output();         // 通道3活动
    let mut error_led = gpiob.pb7.into_push_pull_output();       // 错误指示

    // 配置PWM引脚（8个通道）
    let pwm_pins_tim2 = (
        gpioa.pa0.into_alternate(),  // TIM2_CH1
        gpioa.pa1.into_alternate(),  // TIM2_CH2
        gpioa.pa2.into_alternate(),  // TIM2_CH3
        gpioa.pa3.into_alternate(),  // TIM2_CH4
    );
    
    let pwm_pins_tim3 = (
        gpioa.pa6.into_alternate(),  // TIM3_CH1
        gpioa.pa7.into_alternate(),  // TIM3_CH2
        gpiob.pb0.into_alternate(),  // TIM3_CH3
        gpiob.pb1.into_alternate(),  // TIM3_CH4
    );

    // 配置定时器
    let mut timer2 = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer2.start(1.kHz()).unwrap();

    // 配置PWM
    let pwm_freq = 1.kHz();
    let mut pwm2 = dp.TIM2.pwm_hz(pwm_pins_tim2, pwm_freq, &clocks);
    let mut pwm3 = dp.TIM3.pwm_hz(pwm_pins_tim3, pwm_freq, &clocks);
    
    let (pwm2_ch1, pwm2_ch2, pwm2_ch3, pwm2_ch4) = pwm2.split();
    let (pwm3_ch1, pwm3_ch2, pwm3_ch3, pwm3_ch4) = pwm3.split();
    
    // 创建PWM通道包装器
    let ch1 = PwmChannelWrapper::new(pwm2_ch1, 1000);
    let ch2 = PwmChannelWrapper::new(pwm2_ch2, 1000);
    let ch3 = PwmChannelWrapper::new(pwm2_ch3, 1000);
    let ch4 = PwmChannelWrapper::new(pwm2_ch4, 1000);
    let ch5 = PwmChannelWrapper::new(pwm3_ch1, 1000);
    let ch6 = PwmChannelWrapper::new(pwm3_ch2, 1000);
    let ch7 = PwmChannelWrapper::new(pwm3_ch3, 1000);
    let ch8 = PwmChannelWrapper::new(pwm3_ch4, 1000);

    // 创建事件队列
    let (event_producer, mut event_consumer) = unsafe {
        PWM_EVENT_QUEUE.split()
    };

    // 创建多通道PWM管理器
    let mut pwm_manager = MultiChannelPwmManager::new(
        ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, event_producer
    );

    // 启用所有通道
    for i in 0..8 {
        if let Err(e) = pwm_manager.enable_channel(i) {
            rprintln!("启用通道 {} 失败: {}", i, e);
        }
    }

    // 系统定时器
    let mut delay = cp.SYST.delay(&clocks);
    let mut last_button1_state = button1.is_high();
    let mut last_button2_state = button2.is_high();
    let mut button1_debounce = 0u32;
    let mut button2_debounce = 0u32;
    let mut system_tick = 0u32;

    rprintln!("多通道PWM控制系统就绪");

    loop {
        let current_time = system_tick;
        system_tick = system_tick.wrapping_add(1);

        // 按钮1处理（模式切换）
        let button1_state = button1.is_high();
        if button1_state != last_button1_state {
            button1_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button1_debounce) > 50 && button1_state && !last_button1_state {
            let new_pattern = match pwm_manager.current_pattern {
                PwmPattern::Static => PwmPattern::Sweep,
                PwmPattern::Sweep => PwmPattern::Phase,
                PwmPattern::Phase => PwmPattern::Synchronized,
                PwmPattern::Synchronized => PwmPattern::Independent,
                PwmPattern::Independent => PwmPattern::Waveform,
                PwmPattern::Waveform => PwmPattern::Static,
            };
            pwm_manager.send_event(PwmEvent::PatternChange(new_pattern));
        }
        last_button1_state = button1_state;

        // 按钮2处理（波形切换）
        let button2_state = button2.is_high();
        if button2_state != last_button2_state {
            button2_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button2_debounce) > 50 && button2_state && !last_button2_state {
            let new_waveform = match pwm_manager.waveform_type {
                WaveformType::Sine => WaveformType::Triangle,
                WaveformType::Triangle => WaveformType::Sawtooth,
                WaveformType::Sawtooth => WaveformType::Square,
                WaveformType::Square => WaveformType::Custom,
                WaveformType::Custom => WaveformType::Sine,
            };
            pwm_manager.set_waveform_type(new_waveform);
        }
        last_button2_state = button2_state;

        // 处理事件队列
        while let Some(event) = event_consumer.dequeue() {
            match event {
                PwmEvent::SetDutyCycle(channel_id, duty) => {
                    if let Err(e) = pwm_manager.set_channel_duty_cycle(channel_id, duty) {
                        rprintln!("设置占空比错误: {}", e);
                        error_led.set_high();
                    } else {
                        error_led.set_low();
                    }
                }
                PwmEvent::SetFrequency(channel_id, frequency) => {
                    if let Err(e) = pwm_manager.set_channel_frequency(channel_id, frequency) {
                        rprintln!("设置频率错误: {}", e);
                        error_led.set_high();
                    } else {
                        error_led.set_low();
                    }
                }
                PwmEvent::EnableChannel(channel_id) => {
                    if let Err(e) = pwm_manager.enable_channel(channel_id) {
                        rprintln!("启用通道错误: {}", e);
                    }
                }
                PwmEvent::DisableChannel(channel_id) => {
                    if let Err(e) = pwm_manager.disable_channel(channel_id) {
                        rprintln!("禁用通道错误: {}", e);
                    }
                }
                PwmEvent::PatternChange(pattern) => {
                    pwm_manager.set_pattern(pattern);
                }
                PwmEvent::SyncChannels => {
                    if let Err(e) = pwm_manager.sync_all_channels() {
                        rprintln!("同步通道错误: {}", e);
                    }
                }
                _ => {}
            }
        }

        // 更新PWM模式
        if let Err(e) = pwm_manager.update_pattern(10) {
            rprintln!("模式更新错误: {}", e);
            error_led.set_high();
            pwm_manager.statistics.pwm_errors += 1;
        }

        // 更新统计信息
        pwm_manager.update_statistics(10);

        // LED指示更新
        // 系统状态LED（心跳）
        if system_tick % 100 == 0 {
            status_led.toggle();
        }

        // 模式指示LED（不同模式不同闪烁频率）
        let pattern_blink_period = match pwm_manager.current_pattern {
            PwmPattern::Static => 2000,
            PwmPattern::Sweep => 1000,
            PwmPattern::Phase => 500,
            PwmPattern::Synchronized => 200,
            PwmPattern::Independent => 100,
            PwmPattern::Waveform => 300,
        };
        
        if system_tick % pattern_blink_period < pattern_blink_period / 2 {
            pattern_led.set_high();
        } else {
            pattern_led.set_low();
        }

        // 波形指示LED
        let waveform_blink_count = match pwm_manager.waveform_type {
            WaveformType::Sine => 1,
            WaveformType::Triangle => 2,
            WaveformType::Sawtooth => 3,
            WaveformType::Square => 4,
            WaveformType::Custom => 5,
        };
        
        let blink_cycle = system_tick % 1000;
        let blink_on = blink_cycle < (waveform_blink_count * 100);
        if blink_on {
            waveform_led.set_high();
        } else {
            waveform_led.set_low();
        }

        // 通道活动指示（根据占空比调制LED）
        if let Ok((duty1, _)) = pwm_manager.get_channel_status(0) {
            let intensity = (duty1 * 100.0) as u32;
            if system_tick % 100 < intensity {
                ch1_led.set_high();
            } else {
                ch1_led.set_low();
            }
        }

        if let Ok((duty2, _)) = pwm_manager.get_channel_status(1) {
            let intensity = (duty2 * 100.0) as u32;
            if system_tick % 100 < intensity {
                ch2_led.set_high();
            } else {
                ch2_led.set_low();
            }
        }

        if let Ok((duty3, _)) = pwm_manager.get_channel_status(2) {
            let intensity = (duty3 * 100.0) as u32;
            if system_tick % 100 < intensity {
                ch3_led.set_high();
            } else {
                ch3_led.set_low();
            }
        }

        // 同步指示
        if pwm_manager.sync_enabled {
            sync_led.set_high();
        } else {
            sync_led.set_low();
        }

        // 定期输出统计信息
        if system_tick % 5000 == 0 {
            rprintln!("=== 多通道PWM统计信息 ===");
            rprintln!("运行时间: {}ms", pwm_manager.statistics.total_runtime);
            rprintln!("模式切换次数: {}", pwm_manager.statistics.pattern_changes);
            rprintln!("占空比变更: {}", pwm_manager.statistics.duty_cycle_changes);
            rprintln!("频率变更: {}", pwm_manager.statistics.frequency_changes);
            rprintln!("相位变更: {}", pwm_manager.statistics.phase_changes);
            rprintln!("同步操作: {}", pwm_manager.statistics.sync_operations);
            rprintln!("通道启用: {}", pwm_manager.statistics.channel_enables);
            rprintln!("通道禁用: {}", pwm_manager.statistics.channel_disables);
            rprintln!("总周期数: {}", pwm_manager.statistics.total_cycles);
            rprintln!("频率范围: {} - {} Hz", 
                     pwm_manager.statistics.min_frequency, 
                     pwm_manager.statistics.max_frequency);
            rprintln!("错误次数: {}", pwm_manager.statistics.pwm_errors);
            rprintln!("当前模式: {:?}", pwm_manager.current_pattern);
            rprintln!("当前波形: {:?}", pwm_manager.waveform_type);
            
            // 显示各通道状态
            for i in 0..4 {
                if let Ok((duty, freq)) = pwm_manager.get_channel_status(i) {
                    rprintln!("通道{}: 占空比={:.1}%, 频率={}Hz", i+1, duty*100.0, freq);
                }
            }
        }

        delay.delay_ms(10u32);
    }
}