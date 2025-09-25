#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    adc::{Adc, config::{AdcConfig, SampleTime, Sequence, Eoc, Scan, Continuous}},
    gpio::{Analog, Pin},
    serial::{config::Config, Serial},
    timer::{Timer, Event},
};
use nb::block;
use oscilloscope::{
    TriggerSystem, TriggerConfig, TriggerType, TriggerSlope, TriggerMode,
    MultiChannelBuffer, MeasurementEngine, OscilloscopeStatistics
};

// 触发示波器配置常量
const VREF_MV: u16 = 3300;
const SAMPLE_RATE: u32 = 500000; // 500kHz
const BUFFER_SIZE: usize = 2048;
const PRE_TRIGGER_SIZE: usize = 512; // 25%预触发
const POST_TRIGGER_SIZE: usize = 1536; // 75%后触发
const TRIGGER_TIMEOUT_MS: u32 = 1000;

// 高级触发类型
#[derive(Clone, Copy)]
enum AdvancedTriggerType {
    Edge,           // 边沿触发
    Pulse,          // 脉冲触发
    Window,         // 窗口触发
    Pattern,        // 模式触发
    Runt,           // 欠幅脉冲触发
    Timeout,        // 超时触发
    Video,          // 视频触发
    Logic,          // 逻辑触发
}

// 触发条件
#[derive(Clone, Copy)]
struct TriggerCondition {
    trigger_type: AdvancedTriggerType,
    channel: u8,
    level: u16,
    hysteresis: u16,
    slope: TriggerSlope,
    pulse_width_min: u32,
    pulse_width_max: u32,
    timeout_us: u32,
    window_upper: u16,
    window_lower: u16,
    pattern_mask: u8,
    pattern_value: u8,
}

impl Default for TriggerCondition {
    fn default() -> Self {
        Self {
            trigger_type: AdvancedTriggerType::Edge,
            channel: 0,
            level: 2048,
            hysteresis: 50,
            slope: TriggerSlope::Rising,
            pulse_width_min: 10,
            pulse_width_max: 1000,
            timeout_us: 1000,
            window_upper: 3000,
            window_lower: 1000,
            pattern_mask: 0xFF,
            pattern_value: 0x55,
        }
    }
}

// 高级触发系统
struct AdvancedTriggerSystem {
    condition: TriggerCondition,
    state: TriggerState,
    pre_trigger_buffer: [u16; PRE_TRIGGER_SIZE],
    pre_trigger_index: usize,
    trigger_position: usize,
    trigger_count: u32,
    false_trigger_count: u32,
    last_trigger_time: u32,
    pulse_start_time: u32,
    pulse_width: u32,
    timeout_counter: u32,
    pattern_history: u8,
    window_violations: u16,
    trigger_statistics: TriggerStatistics,
}

#[derive(Clone, Copy, PartialEq)]
enum TriggerState {
    Armed,
    WaitingForTrigger,
    Triggered,
    PostTrigger,
    Timeout,
    Holdoff,
}

struct TriggerStatistics {
    total_triggers: u32,
    false_triggers: u32,
    trigger_rate: f32,
    average_trigger_interval: u32,
    min_trigger_interval: u32,
    max_trigger_interval: u32,
    trigger_jitter: f32,
    trigger_efficiency: f32,
}

impl AdvancedTriggerSystem {
    fn new(condition: TriggerCondition) -> Self {
        Self {
            condition,
            state: TriggerState::Armed,
            pre_trigger_buffer: [0; PRE_TRIGGER_SIZE],
            pre_trigger_index: 0,
            trigger_position: 0,
            trigger_count: 0,
            false_trigger_count: 0,
            last_trigger_time: 0,
            pulse_start_time: 0,
            pulse_width: 0,
            timeout_counter: 0,
            pattern_history: 0,
            window_violations: 0,
            trigger_statistics: TriggerStatistics {
                total_triggers: 0,
                false_triggers: 0,
                trigger_rate: 0.0,
                average_trigger_interval: 0,
                min_trigger_interval: u32::MAX,
                max_trigger_interval: 0,
                trigger_jitter: 0.0,
                trigger_efficiency: 0.0,
            },
        }
    }

    fn process_sample(&mut self, sample: u16, channel: u8, timestamp: u32) -> bool {
        // 更新预触发缓冲区
        self.pre_trigger_buffer[self.pre_trigger_index] = sample;
        self.pre_trigger_index = (self.pre_trigger_index + 1) % PRE_TRIGGER_SIZE;

        match self.state {
            TriggerState::Armed | TriggerState::WaitingForTrigger => {
                if self.check_trigger_condition(sample, channel, timestamp) {
                    self.trigger_detected(timestamp);
                    return true;
                }
                
                // 检查超时
                self.timeout_counter += 1;
                if self.timeout_counter > (TRIGGER_TIMEOUT_MS * SAMPLE_RATE / 1000) {
                    self.state = TriggerState::Timeout;
                    self.timeout_counter = 0;
                }
            },
            TriggerState::Triggered => {
                self.state = TriggerState::PostTrigger;
            },
            TriggerState::PostTrigger => {
                // 后触发采集完成后重新武装
                if self.trigger_position >= POST_TRIGGER_SIZE {
                    self.state = TriggerState::Armed;
                    self.trigger_position = 0;
                }
            },
            TriggerState::Timeout => {
                // 超时后重新武装
                self.state = TriggerState::Armed;
                self.timeout_counter = 0;
            },
            TriggerState::Holdoff => {
                // 保持期结束后重新武装
                if timestamp.wrapping_sub(self.last_trigger_time) > 10000 { // 10ms holdoff
                    self.state = TriggerState::Armed;
                }
            },
        }

        false
    }

    fn check_trigger_condition(&mut self, sample: u16, channel: u8, timestamp: u32) -> bool {
        if channel != self.condition.channel {
            return false;
        }

        match self.condition.trigger_type {
            AdvancedTriggerType::Edge => self.check_edge_trigger(sample),
            AdvancedTriggerType::Pulse => self.check_pulse_trigger(sample, timestamp),
            AdvancedTriggerType::Window => self.check_window_trigger(sample),
            AdvancedTriggerType::Pattern => self.check_pattern_trigger(sample),
            AdvancedTriggerType::Runt => self.check_runt_trigger(sample, timestamp),
            AdvancedTriggerType::Timeout => self.check_timeout_trigger(sample, timestamp),
            AdvancedTriggerType::Video => self.check_video_trigger(sample),
            AdvancedTriggerType::Logic => self.check_logic_trigger(sample),
        }
    }

    fn check_edge_trigger(&self, sample: u16) -> bool {
        let prev_sample = self.get_previous_sample();
        
        match self.condition.slope {
            TriggerSlope::Rising => {
                prev_sample < self.condition.level.saturating_sub(self.condition.hysteresis) &&
                sample > self.condition.level.saturating_add(self.condition.hysteresis)
            },
            TriggerSlope::Falling => {
                prev_sample > self.condition.level.saturating_add(self.condition.hysteresis) &&
                sample < self.condition.level.saturating_sub(self.condition.hysteresis)
            },
        }
    }

    fn check_pulse_trigger(&mut self, sample: u16, timestamp: u32) -> bool {
        let above_threshold = sample > self.condition.level;
        
        if above_threshold && self.pulse_start_time == 0 {
            // 脉冲开始
            self.pulse_start_time = timestamp;
        } else if !above_threshold && self.pulse_start_time != 0 {
            // 脉冲结束
            self.pulse_width = timestamp.wrapping_sub(self.pulse_start_time);
            let pulse_width_us = self.pulse_width * 1000000 / SAMPLE_RATE;
            
            let trigger_condition = pulse_width_us >= self.condition.pulse_width_min &&
                                  pulse_width_us <= self.condition.pulse_width_max;
            
            self.pulse_start_time = 0;
            return trigger_condition;
        }
        
        false
    }

    fn check_window_trigger(&mut self, sample: u16) -> bool {
        let in_window = sample >= self.condition.window_lower && sample <= self.condition.window_upper;
        
        if !in_window {
            self.window_violations += 1;
            return true; // 触发于窗口外
        }
        
        false
    }

    fn check_pattern_trigger(&mut self, sample: u16) -> bool {
        // 将样本转换为数字位
        let digital_bit = if sample > self.condition.level { 1 } else { 0 };
        
        // 更新模式历史
        self.pattern_history = (self.pattern_history << 1) | digital_bit;
        
        // 检查模式匹配
        (self.pattern_history & self.condition.pattern_mask) == self.condition.pattern_value
    }

    fn check_runt_trigger(&mut self, sample: u16, timestamp: u32) -> bool {
        // 欠幅脉冲：超过下限但未达到上限的脉冲
        let above_lower = sample > self.condition.window_lower;
        let below_upper = sample < self.condition.window_upper;
        
        if above_lower && below_upper {
            if self.pulse_start_time == 0 {
                self.pulse_start_time = timestamp;
            }
        } else {
            if self.pulse_start_time != 0 {
                let pulse_duration = timestamp.wrapping_sub(self.pulse_start_time);
                let pulse_duration_us = pulse_duration * 1000000 / SAMPLE_RATE;
                
                // 检查是否为欠幅脉冲
                if pulse_duration_us >= self.condition.pulse_width_min &&
                   pulse_duration_us <= self.condition.pulse_width_max {
                    self.pulse_start_time = 0;
                    return true;
                }
                self.pulse_start_time = 0;
            }
        }
        
        false
    }

    fn check_timeout_trigger(&mut self, sample: u16, timestamp: u32) -> bool {
        let above_threshold = sample > self.condition.level;
        
        if above_threshold {
            self.pulse_start_time = timestamp;
        } else if self.pulse_start_time != 0 {
            let time_since_pulse = timestamp.wrapping_sub(self.pulse_start_time);
            let time_us = time_since_pulse * 1000000 / SAMPLE_RATE;
            
            if time_us > self.condition.timeout_us {
                return true; // 超时触发
            }
        }
        
        false
    }

    fn check_video_trigger(&self, sample: u16) -> bool {
        // 简化的视频触发：检测同步脉冲
        let sync_level = self.condition.level;
        let prev_sample = self.get_previous_sample();
        
        // 检测负向同步脉冲
        prev_sample > sync_level && sample < sync_level
    }

    fn check_logic_trigger(&self, sample: u16) -> bool {
        // 逻辑触发：基于多个条件的组合
        let condition1 = sample > self.condition.level;
        let condition2 = self.get_previous_sample() < self.condition.window_lower;
        
        // 简单的AND逻辑
        condition1 && condition2
    }

    fn get_previous_sample(&self) -> u16 {
        let prev_index = if self.pre_trigger_index == 0 {
            PRE_TRIGGER_SIZE - 1
        } else {
            self.pre_trigger_index - 1
        };
        self.pre_trigger_buffer[prev_index]
    }

    fn trigger_detected(&mut self, timestamp: u32) {
        self.state = TriggerState::Triggered;
        self.trigger_count += 1;
        
        // 更新触发统计
        if self.last_trigger_time != 0 {
            let interval = timestamp.wrapping_sub(self.last_trigger_time);
            self.update_trigger_statistics(interval);
        }
        
        self.last_trigger_time = timestamp;
        self.trigger_position = 0;
    }

    fn update_trigger_statistics(&mut self, interval: u32) {
        self.trigger_statistics.total_triggers += 1;
        
        // 更新间隔统计
        if interval < self.trigger_statistics.min_trigger_interval {
            self.trigger_statistics.min_trigger_interval = interval;
        }
        if interval > self.trigger_statistics.max_trigger_interval {
            self.trigger_statistics.max_trigger_interval = interval;
        }
        
        // 计算平均间隔
        let total_intervals = self.trigger_statistics.total_triggers;
        if total_intervals > 0 {
            self.trigger_statistics.average_trigger_interval = 
                (self.trigger_statistics.average_trigger_interval * (total_intervals - 1) + interval) / total_intervals;
        }
        
        // 计算触发效率
        self.trigger_statistics.trigger_efficiency = 
            (self.trigger_statistics.total_triggers as f32) / 
            ((self.trigger_statistics.total_triggers + self.false_trigger_count) as f32) * 100.0;
    }

    fn get_trigger_type_name(&self) -> &'static str {
        match self.condition.trigger_type {
            AdvancedTriggerType::Edge => "边沿触发",
            AdvancedTriggerType::Pulse => "脉冲触发",
            AdvancedTriggerType::Window => "窗口触发",
            AdvancedTriggerType::Pattern => "模式触发",
            AdvancedTriggerType::Runt => "欠幅脉冲触发",
            AdvancedTriggerType::Timeout => "超时触发",
            AdvancedTriggerType::Video => "视频触发",
            AdvancedTriggerType::Logic => "逻辑触发",
        }
    }

    fn get_state_name(&self) -> &'static str {
        match self.state {
            TriggerState::Armed => "武装",
            TriggerState::WaitingForTrigger => "等待触发",
            TriggerState::Triggered => "已触发",
            TriggerState::PostTrigger => "后触发采集",
            TriggerState::Timeout => "超时",
            TriggerState::Holdoff => "保持期",
        }
    }
}

// 触发示波器性能监控
struct TriggerScopePerformance {
    acquisition_rate: f32,
    trigger_rate: f32,
    dead_time: f32,
    memory_usage: f32,
    processing_load: f32,
    trigger_latency: u32,
    false_trigger_rate: f32,
    capture_efficiency: f32,
}

impl TriggerScopePerformance {
    fn new() -> Self {
        Self {
            acquisition_rate: 0.0,
            trigger_rate: 0.0,
            dead_time: 0.0,
            memory_usage: 0.0,
            processing_load: 0.0,
            trigger_latency: 0,
            false_trigger_rate: 0.0,
            capture_efficiency: 0.0,
        }
    }

    fn update(&mut self, trigger_system: &AdvancedTriggerSystem, sample_count: u32) {
        // 计算采集速率
        self.acquisition_rate = SAMPLE_RATE as f32;
        
        // 计算触发速率
        if sample_count > 0 {
            self.trigger_rate = (trigger_system.trigger_count as f32) / 
                               (sample_count as f32 / SAMPLE_RATE as f32);
        }
        
        // 计算死区时间
        self.dead_time = (POST_TRIGGER_SIZE as f32 / SAMPLE_RATE as f32) * 100.0;
        
        // 估算内存使用率
        self.memory_usage = ((BUFFER_SIZE + PRE_TRIGGER_SIZE) as f32 / 8192.0) * 100.0;
        
        // 计算处理负载
        self.processing_load = (trigger_system.trigger_count as f32 / sample_count as f32) * 100.0;
        
        // 计算误触发率
        if trigger_system.trigger_count > 0 {
            self.false_trigger_rate = (trigger_system.false_trigger_count as f32) / 
                                    (trigger_system.trigger_count as f32) * 100.0;
        }
        
        // 计算捕获效率
        self.capture_efficiency = 100.0 - self.dead_time;
    }

    fn get_performance_rating(&self) -> &'static str {
        let score = (self.capture_efficiency + (100.0 - self.false_trigger_rate) + 
                    (100.0 - self.processing_load.min(100.0))) / 3.0;
        
        if score > 90.0 {
            "优秀"
        } else if score > 80.0 {
            "良好"
        } else if score > 70.0 {
            "一般"
        } else {
            "需要优化"
        }
    }
}

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();

    // 配置串口用于输出
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    let serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &clocks,
    ).unwrap();
    let (mut tx, _rx) = serial.split();

    // 配置ADC引脚
    let adc_pin1 = gpioa.pa0.into_analog();
    let adc_pin2 = gpioa.pa1.into_analog();

    // 配置ADC
    let adc_config = AdcConfig::default()
        .end_of_conversion_interrupt(Eoc::Conversion)
        .scan(Scan::Enabled)
        .continuous(Continuous::Single);
    
    let mut adc = Adc::new(dp.ADC1, true, adc_config);
    adc.configure_channel(&adc_pin1, Sequence::One, SampleTime::Cycles_480);
    adc.configure_channel(&adc_pin2, Sequence::Two, SampleTime::Cycles_480);

    // 配置定时器用于采样触发
    let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer.start(SAMPLE_RATE.hz()).unwrap();
    timer.listen(Event::Update);

    // 初始化触发系统
    let mut trigger_condition = TriggerCondition::default();
    trigger_condition.trigger_type = AdvancedTriggerType::Edge;
    trigger_condition.level = 2048; // 1.65V
    trigger_condition.hysteresis = 100;
    trigger_condition.slope = TriggerSlope::Rising;
    
    let mut trigger_system = AdvancedTriggerSystem::new(trigger_condition);
    let mut trigger_performance = TriggerScopePerformance::new();

    // 初始化缓冲区和统计
    let mut buffer = MultiChannelBuffer::new(BUFFER_SIZE, 2);
    let mut measurement_engine = MeasurementEngine::new();
    let mut statistics = OscilloscopeStatistics::new();

    // 状态变量
    let mut sample_count = 0u32;
    let mut last_status_time = 0u32;
    let mut trigger_type_switch_time = 0u32;
    let mut current_trigger_type = 0usize;
    let trigger_types = [
        AdvancedTriggerType::Edge,
        AdvancedTriggerType::Pulse,
        AdvancedTriggerType::Window,
        AdvancedTriggerType::Pattern,
        AdvancedTriggerType::Runt,
    ];
    let trigger_switch_interval = 3000000; // 6秒 @ 500kHz

    writeln!(tx, "触发示波器启动").unwrap();
    writeln!(tx, "采样率: {}Hz", SAMPLE_RATE).unwrap();
    writeln!(tx, "缓冲区大小: {}", BUFFER_SIZE).unwrap();
    writeln!(tx, "预触发: {}样本", PRE_TRIGGER_SIZE).unwrap();
    writeln!(tx, "后触发: {}样本", POST_TRIGGER_SIZE).unwrap();
    writeln!(tx, "触发类型: {}种", trigger_types.len()).unwrap();

    loop {
        // 检查定时器事件
        if timer.wait().is_ok() {
            // 检查是否需要切换触发类型
            if sample_count.wrapping_sub(trigger_type_switch_time) >= trigger_switch_interval {
                trigger_type_switch_time = sample_count;
                current_trigger_type = (current_trigger_type + 1) % trigger_types.len();
                
                // 更新触发条件
                trigger_condition.trigger_type = trigger_types[current_trigger_type];
                
                // 根据触发类型调整参数
                match trigger_condition.trigger_type {
                    AdvancedTriggerType::Pulse => {
                        trigger_condition.pulse_width_min = 50; // 50us
                        trigger_condition.pulse_width_max = 500; // 500us
                    },
                    AdvancedTriggerType::Window => {
                        trigger_condition.window_lower = 1500;
                        trigger_condition.window_upper = 2500;
                    },
                    AdvancedTriggerType::Pattern => {
                        trigger_condition.pattern_mask = 0x0F;
                        trigger_condition.pattern_value = 0x0A; // 1010 pattern
                    },
                    AdvancedTriggerType::Runt => {
                        trigger_condition.window_lower = 1800;
                        trigger_condition.window_upper = 2300;
                        trigger_condition.pulse_width_min = 20;
                        trigger_condition.pulse_width_max = 200;
                    },
                    _ => {}
                }
                
                trigger_system = AdvancedTriggerSystem::new(trigger_condition);
            }
            
            // 读取ADC通道1
            let sample1: u16 = adc.convert(&adc_pin1, SampleTime::Cycles_480);
            
            // 读取ADC通道2  
            let sample2: u16 = adc.convert(&adc_pin2, SampleTime::Cycles_480);
            
            // 处理触发
            let triggered = trigger_system.process_sample(sample1, 0, sample_count);
            
            // 存储样本到缓冲区
            buffer.add_sample(0, sample1);
            buffer.add_sample(1, sample2);
            
            // 如果触发，进行测量
            if triggered {
                let channel1_data = buffer.get_channel_data(0);
                let channel2_data = buffer.get_channel_data(1);
                
                measurement_engine.measure_all(channel1_data, SAMPLE_RATE);
                statistics.update_from_measurements(&measurement_engine);
            }
            
            sample_count += 1;
        }

        // 定期输出状态信息
        if sample_count.wrapping_sub(last_status_time) >= 1000000 { // 每2秒
            last_status_time = sample_count;
            
            // 更新性能统计
            trigger_performance.update(&trigger_system, sample_count);
            
            writeln!(tx, "\n=== 触发示波器状态 ===").unwrap();
            writeln!(tx, "运行时间: {}s", sample_count / SAMPLE_RATE).unwrap();
            writeln!(tx, "总样本数: {}", sample_count).unwrap();
            writeln!(tx, "触发次数: {}", trigger_system.trigger_count).unwrap();
            
            // 显示当前触发配置
            writeln!(tx, "\n--- 触发配置 ---").unwrap();
            writeln!(tx, "触发类型: {}", trigger_system.get_trigger_type_name()).unwrap();
            writeln!(tx, "触发通道: CH{}", trigger_condition.channel + 1).unwrap();
            writeln!(tx, "触发电平: {} ({}mV)", trigger_condition.level,
                     (trigger_condition.level as u32 * VREF_MV as u32) / 4096).unwrap();
            writeln!(tx, "触发斜率: {:?}", trigger_condition.slope).unwrap();
            writeln!(tx, "滞回: {} ({}mV)", trigger_condition.hysteresis,
                     (trigger_condition.hysteresis as u32 * VREF_MV as u32) / 4096).unwrap();
            
            // 显示特定触发参数
            match trigger_condition.trigger_type {
                AdvancedTriggerType::Pulse => {
                    writeln!(tx, "脉冲宽度范围: {}-{}us", 
                             trigger_condition.pulse_width_min,
                             trigger_condition.pulse_width_max).unwrap();
                },
                AdvancedTriggerType::Window => {
                    writeln!(tx, "窗口范围: {}-{} ({}mV-{}mV)",
                             trigger_condition.window_lower,
                             trigger_condition.window_upper,
                             (trigger_condition.window_lower as u32 * VREF_MV as u32) / 4096,
                             (trigger_condition.window_upper as u32 * VREF_MV as u32) / 4096).unwrap();
                },
                AdvancedTriggerType::Pattern => {
                    writeln!(tx, "模式掩码: 0x{:02X}", trigger_condition.pattern_mask).unwrap();
                    writeln!(tx, "模式值: 0x{:02X}", trigger_condition.pattern_value).unwrap();
                },
                AdvancedTriggerType::Runt => {
                    writeln!(tx, "欠幅范围: {}-{} ({}mV-{}mV)",
                             trigger_condition.window_lower,
                             trigger_condition.window_upper,
                             (trigger_condition.window_lower as u32 * VREF_MV as u32) / 4096,
                             (trigger_condition.window_upper as u32 * VREF_MV as u32) / 4096).unwrap();
                    writeln!(tx, "脉冲宽度: {}-{}us",
                             trigger_condition.pulse_width_min,
                             trigger_condition.pulse_width_max).unwrap();
                },
                _ => {}
            }
            
            // 显示触发状态
            writeln!(tx, "\n--- 触发状态 ---").unwrap();
            writeln!(tx, "当前状态: {}", trigger_system.get_state_name()).unwrap();
            writeln!(tx, "触发位置: {}", trigger_system.trigger_position).unwrap();
            writeln!(tx, "预触发缓冲: {}/{}", trigger_system.pre_trigger_index, PRE_TRIGGER_SIZE).unwrap();
            writeln!(tx, "窗口违规: {}", trigger_system.window_violations).unwrap();
            writeln!(tx, "模式历史: 0b{:08b}", trigger_system.pattern_history).unwrap();
            
            // 显示触发统计
            writeln!(tx, "\n--- 触发统计 ---").unwrap();
            writeln!(tx, "总触发数: {}", trigger_system.trigger_statistics.total_triggers).unwrap();
            writeln!(tx, "误触发数: {}", trigger_system.trigger_statistics.false_triggers).unwrap();
            writeln!(tx, "触发效率: {:.1}%", trigger_system.trigger_statistics.trigger_efficiency).unwrap();
            writeln!(tx, "平均间隔: {}ms", 
                     trigger_system.trigger_statistics.average_trigger_interval / (SAMPLE_RATE / 1000)).unwrap();
            writeln!(tx, "最小间隔: {}ms",
                     trigger_system.trigger_statistics.min_trigger_interval / (SAMPLE_RATE / 1000)).unwrap();
            writeln!(tx, "最大间隔: {}ms",
                     trigger_system.trigger_statistics.max_trigger_interval / (SAMPLE_RATE / 1000)).unwrap();
            
            // 显示性能指标
            writeln!(tx, "\n--- 性能指标 ---").unwrap();
            writeln!(tx, "采集速率: {:.0}Hz", trigger_performance.acquisition_rate).unwrap();
            writeln!(tx, "触发速率: {:.2}Hz", trigger_performance.trigger_rate).unwrap();
            writeln!(tx, "死区时间: {:.1}%", trigger_performance.dead_time).unwrap();
            writeln!(tx, "内存使用: {:.1}%", trigger_performance.memory_usage).unwrap();
            writeln!(tx, "处理负载: {:.1}%", trigger_performance.processing_load).unwrap();
            writeln!(tx, "误触发率: {:.2}%", trigger_performance.false_trigger_rate).unwrap();
            writeln!(tx, "捕获效率: {:.1}%", trigger_performance.capture_efficiency).unwrap();
            writeln!(tx, "性能评级: {}", trigger_performance.get_performance_rating()).unwrap();
            
            // 显示信号测量结果（如果有触发）
            if trigger_system.trigger_count > 0 {
                writeln!(tx, "\n--- 信号测量 ---").unwrap();
                writeln!(tx, "CH1 幅度: {:.1}mV", statistics.ch1_amplitude).unwrap();
                writeln!(tx, "CH1 频率: {:.1}Hz", statistics.ch1_frequency).unwrap();
                writeln!(tx, "CH1 直流偏移: {:.1}mV", statistics.ch1_dc_offset).unwrap();
                writeln!(tx, "CH1 RMS: {:.1}mV", statistics.ch1_rms).unwrap();
                writeln!(tx, "CH1 占空比: {:.1}%", statistics.ch1_duty_cycle).unwrap();
                
                writeln!(tx, "CH2 幅度: {:.1}mV", statistics.ch2_amplitude).unwrap();
                writeln!(tx, "CH2 频率: {:.1}Hz", statistics.ch2_frequency).unwrap();
                writeln!(tx, "CH2 直流偏移: {:.1}mV", statistics.ch2_dc_offset).unwrap();
                writeln!(tx, "CH2 RMS: {:.1}mV", statistics.ch2_rms).unwrap();
                writeln!(tx, "CH2 占空比: {:.1}%", statistics.ch2_duty_cycle).unwrap();
            }
            
            // 显示下一个触发类型预告
            let next_trigger_type = (current_trigger_type + 1) % trigger_types.len();
            let remaining_time = (trigger_switch_interval - (sample_count - trigger_type_switch_time)) / SAMPLE_RATE;
            
            writeln!(tx, "\n--- 下一个触发类型 ---").unwrap();
            let next_name = match trigger_types[next_trigger_type] {
                AdvancedTriggerType::Edge => "边沿触发",
                AdvancedTriggerType::Pulse => "脉冲触发", 
                AdvancedTriggerType::Window => "窗口触发",
                AdvancedTriggerType::Pattern => "模式触发",
                AdvancedTriggerType::Runt => "欠幅脉冲触发",
                _ => "未知触发",
            };
            writeln!(tx, "下一个类型: {}", next_name).unwrap();
            writeln!(tx, "切换倒计时: {}秒", remaining_time).unwrap();
            
            // 显示系统状态
            writeln!(tx, "\n--- 系统状态 ---").unwrap();
            writeln!(tx, "系统时钟: {}MHz", clocks.sysclk().0 / 1_000_000).unwrap();
            writeln!(tx, "ADC分辨率: 12位").unwrap();
            writeln!(tx, "参考电压: {}mV", VREF_MV).unwrap();
            writeln!(tx, "缓冲区状态: 正常").unwrap();
            writeln!(tx, "触发系统: 正常").unwrap();
        }
    }
}