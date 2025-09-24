# 调试指南

## 概述

定时器和中断系统的调试是嵌入式开发中的重要技能。本指南提供系统性的调试方法、工具使用和问题诊断技巧，帮助开发者快速定位和解决定时器、中断相关的问题。

## 调试环境搭建

### 硬件调试工具

1. **ST-Link调试器**
   - 支持SWD/JTAG接口
   - 实时调试和跟踪功能
   - 支持断点和单步调试

2. **逻辑分析仪**
   - 多通道信号捕获
   - 时序分析功能
   - 协议解码支持

3. **示波器**
   - 高精度时间测量
   - 信号质量分析
   - 触发和捕获功能

### 软件调试工具

```rust
// 调试工具配置
pub struct DebugToolkit {
    trace_enabled: bool,
    profiling_enabled: bool,
    log_level: LogLevel,
    debug_channels: Vec<DebugChannel>,
}

#[derive(Debug, Clone)]
pub enum LogLevel {
    Error,
    Warning,
    Info,
    Debug,
    Trace,
}

#[derive(Debug, Clone)]
pub struct DebugChannel {
    pub channel_id: u8,
    pub name: &'static str,
    pub enabled: bool,
    pub buffer_size: usize,
}

impl DebugToolkit {
    pub fn new() -> Self {
        Self {
            trace_enabled: false,
            profiling_enabled: false,
            log_level: LogLevel::Info,
            debug_channels: vec![
                DebugChannel {
                    channel_id: 0,
                    name: "TIMER",
                    enabled: true,
                    buffer_size: 1024,
                },
                DebugChannel {
                    channel_id: 1,
                    name: "INTERRUPT",
                    enabled: true,
                    buffer_size: 512,
                },
                DebugChannel {
                    channel_id: 2,
                    name: "PWM",
                    enabled: false,
                    buffer_size: 256,
                },
            ],
        }
    }
    
    pub fn enable_trace(&mut self) {
        self.trace_enabled = true;
        // 配置ARM CoreSight ETM
        self.configure_etm();
    }
    
    fn configure_etm(&self) {
        // ETM配置代码
        unsafe {
            // 启用ETM时钟
            let etm_base = 0xE0041000u32 as *mut u32;
            core::ptr::write_volatile(etm_base.offset(0x000), 0x00000001); // ETM Control
            core::ptr::write_volatile(etm_base.offset(0x004), 0x0000406F); // ETM Configuration
            core::ptr::write_volatile(etm_base.offset(0x008), 0x00000000); // ETM Trigger Event
        }
    }
    
    pub fn log(&self, level: LogLevel, channel: &str, message: &str) {
        if self.should_log(level) {
            let timestamp = get_timestamp_us();
            // 格式化并输出日志
            self.output_log(timestamp, level, channel, message);
        }
    }
    
    fn should_log(&self, level: LogLevel) -> bool {
        match (&self.log_level, &level) {
            (LogLevel::Error, LogLevel::Error) => true,
            (LogLevel::Warning, LogLevel::Error | LogLevel::Warning) => true,
            (LogLevel::Info, LogLevel::Error | LogLevel::Warning | LogLevel::Info) => true,
            (LogLevel::Debug, LogLevel::Error | LogLevel::Warning | LogLevel::Info | LogLevel::Debug) => true,
            (LogLevel::Trace, _) => true,
            _ => false,
        }
    }
    
    fn output_log(&self, timestamp: u64, level: LogLevel, channel: &str, message: &str) {
        // 输出到ITM或RTT
        #[cfg(feature = "itm")]
        {
            use cortex_m::itm;
            if let Some(mut itm) = itm::ITM::new() {
                itm::write_fmt(&mut itm.stim[0], format_args!("[{}] {}: {} - {}\n", 
                    timestamp, level_to_str(level), channel, message)).ok();
            }
        }
        
        #[cfg(feature = "rtt")]
        {
            use rtt_target::{rprintln, rprint};
            rprintln!("[{}] {}: {} - {}", timestamp, level_to_str(level), channel, message);
        }
    }
}

fn level_to_str(level: LogLevel) -> &'static str {
    match level {
        LogLevel::Error => "ERROR",
        LogLevel::Warning => "WARN",
        LogLevel::Info => "INFO",
        LogLevel::Debug => "DEBUG",
        LogLevel::Trace => "TRACE",
    }
}

fn get_timestamp_us() -> u64 {
    // 使用DWT CYCCNT获取高精度时间戳
    unsafe {
        let cyccnt = core::ptr::read_volatile(0xE0001004 as *const u32);
        (cyccnt as u64 * 1_000_000) / 168_000_000 // 假设168MHz系统时钟
    }
}
```

## 定时器调试

### 定时器状态监控

```rust
// 定时器调试监控器
pub struct TimerDebugMonitor {
    monitored_timers: Vec<TimerDebugInfo>,
    capture_enabled: bool,
    capture_buffer: Vec<TimerEvent>,
}

#[derive(Debug, Clone)]
pub struct TimerDebugInfo {
    pub timer_id: u8,
    pub name: &'static str,
    pub base_address: u32,
    pub monitoring_enabled: bool,
    pub event_count: u32,
    pub last_event_timestamp: u64,
}

#[derive(Debug, Clone)]
pub struct TimerEvent {
    pub timer_id: u8,
    pub event_type: TimerEventType,
    pub timestamp: u64,
    pub counter_value: u32,
    pub prescaler_value: u32,
    pub auto_reload_value: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TimerEventType {
    Started,
    Stopped,
    Updated,
    Overflow,
    Compare,
    Capture,
    Break,
}

impl TimerDebugMonitor {
    pub fn new() -> Self {
        Self {
            monitored_timers: vec![
                TimerDebugInfo {
                    timer_id: 1,
                    name: "TIM1",
                    base_address: 0x40010000,
                    monitoring_enabled: false,
                    event_count: 0,
                    last_event_timestamp: 0,
                },
                TimerDebugInfo {
                    timer_id: 2,
                    name: "TIM2",
                    base_address: 0x40000000,
                    monitoring_enabled: false,
                    event_count: 0,
                    last_event_timestamp: 0,
                },
                TimerDebugInfo {
                    timer_id: 6,
                    name: "TIM6",
                    base_address: 0x40001000,
                    monitoring_enabled: false,
                    event_count: 0,
                    last_event_timestamp: 0,
                },
            ],
            capture_enabled: false,
            capture_buffer: Vec::new(),
        }
    }
    
    pub fn enable_timer_monitoring(&mut self, timer_id: u8) {
        if let Some(timer_info) = self.monitored_timers.iter_mut()
            .find(|t| t.timer_id == timer_id) {
            timer_info.monitoring_enabled = true;
            self.setup_timer_monitoring(timer_info);
        }
    }
    
    fn setup_timer_monitoring(&self, timer_info: &TimerDebugInfo) {
        // 配置定时器调试监控
        unsafe {
            let tim_base = timer_info.base_address as *mut u32;
            
            // 启用更新中断用于监控
            let dier = tim_base.offset(0x0C / 4);
            let current_dier = core::ptr::read_volatile(dier);
            core::ptr::write_volatile(dier, current_dier | 0x01); // UIE
            
            // 启用比较中断
            core::ptr::write_volatile(dier, current_dier | 0x1E); // CC1IE-CC4IE
        }
    }
    
    pub fn capture_timer_state(&self, timer_id: u8) -> TimerState {
        let timer_info = self.monitored_timers.iter()
            .find(|t| t.timer_id == timer_id)
            .expect("Timer not found");
        
        unsafe {
            let tim_base = timer_info.base_address as *mut u32;
            
            TimerState {
                timer_id,
                control_register: core::ptr::read_volatile(tim_base.offset(0x00 / 4)),
                counter_value: core::ptr::read_volatile(tim_base.offset(0x24 / 4)),
                prescaler: core::ptr::read_volatile(tim_base.offset(0x28 / 4)),
                auto_reload: core::ptr::read_volatile(tim_base.offset(0x2C / 4)),
                status_register: core::ptr::read_volatile(tim_base.offset(0x10 / 4)),
                interrupt_enable: core::ptr::read_volatile(tim_base.offset(0x0C / 4)),
                capture_compare: [
                    core::ptr::read_volatile(tim_base.offset(0x34 / 4)),
                    core::ptr::read_volatile(tim_base.offset(0x38 / 4)),
                    core::ptr::read_volatile(tim_base.offset(0x3C / 4)),
                    core::ptr::read_volatile(tim_base.offset(0x40 / 4)),
                ],
                timestamp: get_timestamp_us(),
            }
        }
    }
    
    pub fn record_timer_event(&mut self, timer_id: u8, event_type: TimerEventType) {
        if !self.capture_enabled {
            return;
        }
        
        let state = self.capture_timer_state(timer_id);
        
        let event = TimerEvent {
            timer_id,
            event_type,
            timestamp: state.timestamp,
            counter_value: state.counter_value,
            prescaler_value: state.prescaler,
            auto_reload_value: state.auto_reload,
        };
        
        self.capture_buffer.push(event);
        
        // 更新定时器信息
        if let Some(timer_info) = self.monitored_timers.iter_mut()
            .find(|t| t.timer_id == timer_id) {
            timer_info.event_count += 1;
            timer_info.last_event_timestamp = state.timestamp;
        }
        
        // 限制缓冲区大小
        if self.capture_buffer.len() > 1000 {
            self.capture_buffer.remove(0);
        }
    }
    
    pub fn analyze_timer_behavior(&self, timer_id: u8, duration_ms: u32) -> TimerAnalysis {
        let cutoff_time = get_timestamp_us() - (duration_ms as u64 * 1000);
        
        let events: Vec<_> = self.capture_buffer.iter()
            .filter(|e| e.timer_id == timer_id && e.timestamp >= cutoff_time)
            .collect();
        
        if events.is_empty() {
            return TimerAnalysis::default();
        }
        
        let mut overflow_count = 0;
        let mut compare_count = 0;
        let mut capture_count = 0;
        let mut intervals = Vec::new();
        
        let mut last_timestamp = events[0].timestamp;
        
        for event in &events[1..] {
            let interval = event.timestamp - last_timestamp;
            intervals.push(interval);
            last_timestamp = event.timestamp;
            
            match event.event_type {
                TimerEventType::Overflow => overflow_count += 1,
                TimerEventType::Compare => compare_count += 1,
                TimerEventType::Capture => capture_count += 1,
                _ => {}
            }
        }
        
        let average_interval = if !intervals.is_empty() {
            intervals.iter().sum::<u64>() / intervals.len() as u64
        } else {
            0
        };
        
        let min_interval = intervals.iter().min().copied().unwrap_or(0);
        let max_interval = intervals.iter().max().copied().unwrap_or(0);
        
        TimerAnalysis {
            timer_id,
            analysis_duration_ms: duration_ms,
            total_events: events.len(),
            overflow_events: overflow_count,
            compare_events: compare_count,
            capture_events: capture_count,
            average_interval_us: average_interval,
            min_interval_us: min_interval,
            max_interval_us: max_interval,
            jitter_us: max_interval - min_interval,
            frequency_hz: if average_interval > 0 { 
                1_000_000.0 / average_interval as f32 
            } else { 
                0.0 
            },
        }
    }
    
    pub fn generate_timer_report(&self, timer_id: u8) -> String {
        let timer_info = self.monitored_timers.iter()
            .find(|t| t.timer_id == timer_id)
            .expect("Timer not found");
        
        let state = self.capture_timer_state(timer_id);
        let analysis = self.analyze_timer_behavior(timer_id, 1000); // 最近1秒
        
        format!(
            "Timer {} ({}) Debug Report\n\
            ================================\n\
            Current State:\n\
            - Counter: {}\n\
            - Prescaler: {}\n\
            - Auto-reload: {}\n\
            - Status: 0x{:08X}\n\
            - Control: 0x{:08X}\n\
            \n\
            Recent Activity (1s):\n\
            - Total Events: {}\n\
            - Overflow Events: {}\n\
            - Compare Events: {}\n\
            - Capture Events: {}\n\
            - Average Frequency: {:.2} Hz\n\
            - Jitter: {} μs\n\
            \n\
            Monitoring Status:\n\
            - Enabled: {}\n\
            - Total Event Count: {}\n\
            - Last Event: {} μs ago\n",
            timer_id,
            timer_info.name,
            state.counter_value,
            state.prescaler,
            state.auto_reload,
            state.status_register,
            state.control_register,
            analysis.total_events,
            analysis.overflow_events,
            analysis.compare_events,
            analysis.capture_events,
            analysis.frequency_hz,
            analysis.jitter_us,
            timer_info.monitoring_enabled,
            timer_info.event_count,
            get_timestamp_us() - timer_info.last_event_timestamp
        )
    }
}

#[derive(Debug)]
pub struct TimerState {
    pub timer_id: u8,
    pub control_register: u32,
    pub counter_value: u32,
    pub prescaler: u32,
    pub auto_reload: u32,
    pub status_register: u32,
    pub interrupt_enable: u32,
    pub capture_compare: [u32; 4],
    pub timestamp: u64,
}

#[derive(Debug, Default)]
pub struct TimerAnalysis {
    pub timer_id: u8,
    pub analysis_duration_ms: u32,
    pub total_events: usize,
    pub overflow_events: usize,
    pub compare_events: usize,
    pub capture_events: usize,
    pub average_interval_us: u64,
    pub min_interval_us: u64,
    pub max_interval_us: u64,
    pub jitter_us: u64,
    pub frequency_hz: f32,
}
```

### PWM信号调试

```rust
// PWM调试工具
pub struct PwmDebugger {
    pwm_channels: Vec<PwmChannelDebug>,
    measurement_enabled: bool,
    measurement_buffer: Vec<PwmMeasurement>,
}

#[derive(Debug, Clone)]
pub struct PwmChannelDebug {
    pub timer_id: u8,
    pub channel: u8,
    pub gpio_pin: u8,
    pub expected_frequency: f32,
    pub expected_duty_cycle: f32,
    pub monitoring_enabled: bool,
}

#[derive(Debug, Clone)]
pub struct PwmMeasurement {
    pub timer_id: u8,
    pub channel: u8,
    pub timestamp: u64,
    pub period_us: u32,
    pub high_time_us: u32,
    pub duty_cycle: f32,
    pub frequency: f32,
}

impl PwmDebugger {
    pub fn new() -> Self {
        Self {
            pwm_channels: Vec::new(),
            measurement_enabled: false,
            measurement_buffer: Vec::new(),
        }
    }
    
    pub fn add_pwm_channel(&mut self, timer_id: u8, channel: u8, gpio_pin: u8, 
                          expected_freq: f32, expected_duty: f32) {
        self.pwm_channels.push(PwmChannelDebug {
            timer_id,
            channel,
            gpio_pin,
            expected_frequency: expected_freq,
            expected_duty_cycle: expected_duty,
            monitoring_enabled: false,
        });
    }
    
    pub fn enable_pwm_monitoring(&mut self, timer_id: u8, channel: u8) {
        if let Some(pwm_channel) = self.pwm_channels.iter_mut()
            .find(|c| c.timer_id == timer_id && c.channel == channel) {
            pwm_channel.monitoring_enabled = true;
            self.setup_pwm_capture(pwm_channel);
        }
    }
    
    fn setup_pwm_capture(&self, pwm_channel: &PwmChannelDebug) {
        // 配置输入捕获来测量PWM信号
        unsafe {
            let tim_base = self.get_timer_base_address(pwm_channel.timer_id);
            
            // 配置输入捕获模式
            let ccmr_offset = if pwm_channel.channel <= 2 { 0x18 } else { 0x1C };
            let ccmr = tim_base.offset(ccmr_offset / 4);
            
            let shift = ((pwm_channel.channel - 1) % 2) * 8;
            let current_ccmr = core::ptr::read_volatile(ccmr);
            
            // 设置为输入捕获模式，上升沿触发
            let new_ccmr = (current_ccmr & !(0xFF << shift)) | (0x01 << shift);
            core::ptr::write_volatile(ccmr, new_ccmr);
            
            // 启用捕获中断
            let dier = tim_base.offset(0x0C / 4);
            let current_dier = core::ptr::read_volatile(dier);
            core::ptr::write_volatile(dier, current_dier | (1 << pwm_channel.channel));
        }
    }
    
    fn get_timer_base_address(&self, timer_id: u8) -> *mut u32 {
        match timer_id {
            1 => 0x40010000 as *mut u32,
            2 => 0x40000000 as *mut u32,
            3 => 0x40000400 as *mut u32,
            4 => 0x40000800 as *mut u32,
            _ => 0x40000000 as *mut u32,
        }
    }
    
    pub fn measure_pwm_signal(&mut self, timer_id: u8, channel: u8) -> Option<PwmMeasurement> {
        // 使用输入捕获测量PWM信号
        static mut LAST_CAPTURE: [u32; 8] = [0; 8];
        static mut CAPTURE_STATE: [u8; 8] = [0; 8]; // 0: 等待上升沿, 1: 等待下降沿
        
        unsafe {
            let tim_base = self.get_timer_base_address(timer_id);
            let ccr_offset = 0x34 + (channel - 1) * 4;
            let current_capture = core::ptr::read_volatile(tim_base.offset(ccr_offset / 4));
            
            let index = (timer_id - 1) * 4 + (channel - 1);
            if index >= 8 { return None; }
            
            match CAPTURE_STATE[index as usize] {
                0 => {
                    // 捕获到上升沿，切换到下降沿捕获
                    LAST_CAPTURE[index as usize] = current_capture;
                    CAPTURE_STATE[index as usize] = 1;
                    
                    // 配置为下降沿触发
                    let ccmr_offset = if channel <= 2 { 0x18 } else { 0x1C };
                    let ccmr = tim_base.offset(ccmr_offset / 4);
                    let shift = ((channel - 1) % 2) * 8;
                    let current_ccmr = core::ptr::read_volatile(ccmr);
                    let new_ccmr = (current_ccmr & !(0xFF << shift)) | (0x02 << shift);
                    core::ptr::write_volatile(ccmr, new_ccmr);
                    
                    None
                },
                1 => {
                    // 捕获到下降沿，计算高电平时间
                    let high_time_ticks = if current_capture > LAST_CAPTURE[index as usize] {
                        current_capture - LAST_CAPTURE[index as usize]
                    } else {
                        // 处理计数器溢出
                        (0xFFFF - LAST_CAPTURE[index as usize]) + current_capture + 1
                    };
                    
                    CAPTURE_STATE[index as usize] = 2;
                    
                    // 配置为上升沿触发，准备测量周期
                    let ccmr_offset = if channel <= 2 { 0x18 } else { 0x1C };
                    let ccmr = tim_base.offset(ccmr_offset / 4);
                    let shift = ((channel - 1) % 2) * 8;
                    let current_ccmr = core::ptr::read_volatile(ccmr);
                    let new_ccmr = (current_ccmr & !(0xFF << shift)) | (0x01 << shift);
                    core::ptr::write_volatile(ccmr, new_ccmr);
                    
                    // 暂存高电平时间
                    LAST_CAPTURE[index as usize] = high_time_ticks;
                    None
                },
                2 => {
                    // 捕获到下一个上升沿，计算周期
                    let period_ticks = if current_capture > LAST_CAPTURE[index as usize] {
                        current_capture - LAST_CAPTURE[index as usize]
                    } else {
                        (0xFFFF - LAST_CAPTURE[index as usize]) + current_capture + 1
                    };
                    
                    CAPTURE_STATE[index as usize] = 0;
                    
                    // 计算实际时间值
                    let timer_freq = self.get_timer_frequency(timer_id);
                    let tick_period_us = 1_000_000.0 / timer_freq;
                    
                    let high_time_us = (LAST_CAPTURE[index as usize] as f32 * tick_period_us) as u32;
                    let period_us = (period_ticks as f32 * tick_period_us) as u32;
                    let duty_cycle = (high_time_us as f32 / period_us as f32) * 100.0;
                    let frequency = 1_000_000.0 / period_us as f32;
                    
                    let measurement = PwmMeasurement {
                        timer_id,
                        channel,
                        timestamp: get_timestamp_us(),
                        period_us,
                        high_time_us,
                        duty_cycle,
                        frequency,
                    };
                    
                    self.measurement_buffer.push(measurement.clone());
                    
                    // 限制缓冲区大小
                    if self.measurement_buffer.len() > 100 {
                        self.measurement_buffer.remove(0);
                    }
                    
                    Some(measurement)
                },
                _ => None,
            }
        }
    }
    
    fn get_timer_frequency(&self, timer_id: u8) -> f32 {
        // 根据定时器配置计算频率
        unsafe {
            let tim_base = self.get_timer_base_address(timer_id);
            let prescaler = core::ptr::read_volatile(tim_base.offset(0x28 / 4)) + 1;
            
            // 假设APB1时钟为84MHz，APB2时钟为168MHz
            let base_freq = if timer_id == 1 || timer_id == 8 { 168_000_000.0 } else { 84_000_000.0 };
            base_freq / prescaler as f32
        }
    }
    
    pub fn analyze_pwm_accuracy(&self, timer_id: u8, channel: u8, duration_ms: u32) -> PwmAccuracyAnalysis {
        let cutoff_time = get_timestamp_us() - (duration_ms as u64 * 1000);
        
        let measurements: Vec<_> = self.measurement_buffer.iter()
            .filter(|m| m.timer_id == timer_id && m.channel == channel && m.timestamp >= cutoff_time)
            .collect();
        
        if measurements.is_empty() {
            return PwmAccuracyAnalysis::default();
        }
        
        let expected = self.pwm_channels.iter()
            .find(|c| c.timer_id == timer_id && c.channel == channel)
            .expect("PWM channel not found");
        
        let frequencies: Vec<f32> = measurements.iter().map(|m| m.frequency).collect();
        let duty_cycles: Vec<f32> = measurements.iter().map(|m| m.duty_cycle).collect();
        
        let avg_frequency = frequencies.iter().sum::<f32>() / frequencies.len() as f32;
        let avg_duty_cycle = duty_cycles.iter().sum::<f32>() / duty_cycles.len() as f32;
        
        let freq_error = ((avg_frequency - expected.expected_frequency) / expected.expected_frequency * 100.0).abs();
        let duty_error = (avg_duty_cycle - expected.expected_duty_cycle).abs();
        
        let freq_jitter = frequencies.iter().map(|f| (f - avg_frequency).abs()).fold(0.0f32, f32::max);
        let duty_jitter = duty_cycles.iter().map(|d| (d - avg_duty_cycle).abs()).fold(0.0f32, f32::max);
        
        PwmAccuracyAnalysis {
            timer_id,
            channel,
            measurement_count: measurements.len(),
            expected_frequency: expected.expected_frequency,
            measured_frequency: avg_frequency,
            frequency_error_percent: freq_error,
            frequency_jitter_hz: freq_jitter,
            expected_duty_cycle: expected.expected_duty_cycle,
            measured_duty_cycle: avg_duty_cycle,
            duty_cycle_error_percent: duty_error,
            duty_cycle_jitter_percent: duty_jitter,
        }
    }
}

#[derive(Debug, Default)]
pub struct PwmAccuracyAnalysis {
    pub timer_id: u8,
    pub channel: u8,
    pub measurement_count: usize,
    pub expected_frequency: f32,
    pub measured_frequency: f32,
    pub frequency_error_percent: f32,
    pub frequency_jitter_hz: f32,
    pub expected_duty_cycle: f32,
    pub measured_duty_cycle: f32,
    pub duty_cycle_error_percent: f32,
    pub duty_cycle_jitter_percent: f32,
}
```

## 中断系统调试

### 中断跟踪和分析

```rust
// 中断调试跟踪器
pub struct InterruptTracer {
    trace_enabled: bool,
    interrupt_stats: Vec<InterruptStats>,
    trace_buffer: Vec<InterruptTrace>,
    nesting_depth: u8,
    max_nesting_depth: u8,
}

#[derive(Debug, Clone)]
pub struct InterruptStats {
    pub irq_number: u8,
    pub name: &'static str,
    pub call_count: u32,
    pub total_execution_time_us: u64,
    pub min_execution_time_us: u32,
    pub max_execution_time_us: u32,
    pub last_execution_time_us: u32,
    pub priority: u8,
    pub enabled: bool,
}

#[derive(Debug, Clone)]
pub struct InterruptTrace {
    pub irq_number: u8,
    pub event_type: InterruptEvent,
    pub timestamp: u64,
    pub nesting_level: u8,
    pub execution_time_us: Option<u32>,
    pub preempted_irq: Option<u8>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum InterruptEvent {
    Entry,
    Exit,
    Preempted,
    Resumed,
    Missed,
    Overrun,
}

impl InterruptTracer {
    pub fn new() -> Self {
        Self {
            trace_enabled: false,
            interrupt_stats: Self::initialize_interrupt_stats(),
            trace_buffer: Vec::new(),
            nesting_depth: 0,
            max_nesting_depth: 0,
        }
    }
    
    fn initialize_interrupt_stats() -> Vec<InterruptStats> {
        vec![
            InterruptStats {
                irq_number: 25, // TIM1_UP
                name: "TIM1_UP",
                call_count: 0,
                total_execution_time_us: 0,
                min_execution_time_us: u32::MAX,
                max_execution_time_us: 0,
                last_execution_time_us: 0,
                priority: 0,
                enabled: false,
            },
            InterruptStats {
                irq_number: 28, // TIM2
                name: "TIM2",
                call_count: 0,
                total_execution_time_us: 0,
                min_execution_time_us: u32::MAX,
                max_execution_time_us: 0,
                last_execution_time_us: 0,
                priority: 0,
                enabled: false,
            },
            InterruptStats {
                irq_number: 54, // TIM6_DAC
                name: "TIM6_DAC",
                call_count: 0,
                total_execution_time_us: 0,
                min_execution_time_us: u32::MAX,
                max_execution_time_us: 0,
                last_execution_time_us: 0,
                priority: 0,
                enabled: false,
            },
        ]
    }
    
    pub fn enable_tracing(&mut self) {
        self.trace_enabled = true;
        self.setup_interrupt_monitoring();
    }
    
    fn setup_interrupt_monitoring(&self) {
        // 配置DWT和ITM进行中断跟踪
        unsafe {
            // 启用DWT
            let dwt_ctrl = 0xE0001000 as *mut u32;
            core::ptr::write_volatile(dwt_ctrl, 0x40000001);
            
            // 启用异常跟踪
            let demcr = 0xE000EDFC as *mut u32;
            let current_demcr = core::ptr::read_volatile(demcr);
            core::ptr::write_volatile(demcr, current_demcr | 0x01000000); // TRCENA
        }
    }
    
    pub fn on_interrupt_entry(&mut self, irq_number: u8) {
        if !self.trace_enabled {
            return;
        }
        
        let timestamp = get_timestamp_us();
        self.nesting_depth += 1;
        self.max_nesting_depth = self.max_nesting_depth.max(self.nesting_depth);
        
        // 记录中断入口事件
        let trace = InterruptTrace {
            irq_number,
            event_type: InterruptEvent::Entry,
            timestamp,
            nesting_level: self.nesting_depth,
            execution_time_us: None,
            preempted_irq: self.get_current_interrupt(),
        };
        
        self.trace_buffer.push(trace);
        
        // 更新统计信息
        if let Some(stats) = self.interrupt_stats.iter_mut()
            .find(|s| s.irq_number == irq_number) {
            stats.call_count += 1;
        }
        
        // 限制缓冲区大小
        if self.trace_buffer.len() > 1000 {
            self.trace_buffer.remove(0);
        }
    }
    
    pub fn on_interrupt_exit(&mut self, irq_number: u8) {
        if !self.trace_enabled {
            return;
        }
        
        let timestamp = get_timestamp_us();
        
        // 查找对应的入口事件计算执行时间
        let execution_time = self.trace_buffer.iter().rev()
            .find(|t| t.irq_number == irq_number && t.event_type == InterruptEvent::Entry)
            .map(|entry| (timestamp - entry.timestamp) as u32);
        
        // 记录中断出口事件
        let trace = InterruptTrace {
            irq_number,
            event_type: InterruptEvent::Exit,
            timestamp,
            nesting_level: self.nesting_depth,
            execution_time_us: execution_time,
            preempted_irq: None,
        };
        
        self.trace_buffer.push(trace);
        
        // 更新统计信息
        if let Some(stats) = self.interrupt_stats.iter_mut()
            .find(|s| s.irq_number == irq_number) {
            if let Some(exec_time) = execution_time {
                stats.total_execution_time_us += exec_time as u64;
                stats.min_execution_time_us = stats.min_execution_time_us.min(exec_time);
                stats.max_execution_time_us = stats.max_execution_time_us.max(exec_time);
                stats.last_execution_time_us = exec_time;
            }
        }
        
        self.nesting_depth = self.nesting_depth.saturating_sub(1);
    }
    
    fn get_current_interrupt(&self) -> Option<u8> {
        // 从NVIC获取当前活动的中断
        unsafe {
            let icsr = 0xE000ED04 as *const u32;
            let icsr_value = core::ptr::read_volatile(icsr);
            let vectactive = (icsr_value & 0x1FF) as u8;
            
            if vectactive >= 16 {
                Some(vectactive - 16) // 减去系统异常偏移
            } else {
                None
            }
        }
    }
    
    pub fn analyze_interrupt_performance(&self, duration_ms: u32) -> InterruptPerformanceAnalysis {
        let cutoff_time = get_timestamp_us() - (duration_ms as u64 * 1000);
        
        let recent_traces: Vec<_> = self.trace_buffer.iter()
            .filter(|t| t.timestamp >= cutoff_time)
            .collect();
        
        let mut interrupt_loads = std::collections::HashMap::new();
        let mut total_interrupt_time = 0u64;
        let mut max_latency = 0u32;
        let mut preemption_count = 0;
        
        for trace in &recent_traces {
            match trace.event_type {
                InterruptEvent::Entry => {
                    // 计算中断延迟（简化版本）
                    let latency = 12; // ARM Cortex-M4典型中断延迟
                    max_latency = max_latency.max(latency);
                },
                InterruptEvent::Exit => {
                    if let Some(exec_time) = trace.execution_time_us {
                        *interrupt_loads.entry(trace.irq_number).or_insert(0u64) += exec_time as u64;
                        total_interrupt_time += exec_time as u64;
                    }
                },
                InterruptEvent::Preempted => {
                    preemption_count += 1;
                },
                _ => {}
            }
        }
        
        let analysis_duration_us = duration_ms as u64 * 1000;
        let total_interrupt_load = if analysis_duration_us > 0 {
            (total_interrupt_time as f32 / analysis_duration_us as f32) * 100.0
        } else {
            0.0
        };
        
        InterruptPerformanceAnalysis {
            analysis_duration_ms: duration_ms,
            total_interrupt_events: recent_traces.len(),
            total_interrupt_load_percent: total_interrupt_load,
            max_interrupt_latency_us: max_latency,
            max_nesting_depth: self.max_nesting_depth,
            preemption_count,
            interrupt_loads,
        }
    }
    
    pub fn detect_interrupt_problems(&self) -> Vec<InterruptProblem> {
        let mut problems = Vec::new();
        
        // 检查中断负载过高
        let analysis = self.analyze_interrupt_performance(1000);
        if analysis.total_interrupt_load_percent > 50.0 {
            problems.push(InterruptProblem {
                problem_type: InterruptProblemType::HighInterruptLoad,
                severity: ProblemSeverity::Warning,
                description: format!("High interrupt load: {:.1}%", analysis.total_interrupt_load_percent),
                affected_interrupts: vec![],
                recommendation: "Consider optimizing interrupt handlers or reducing interrupt frequency".to_string(),
            });
        }
        
        // 检查中断延迟过高
        if analysis.max_interrupt_latency_us > 100 {
            problems.push(InterruptProblem {
                problem_type: InterruptProblemType::HighLatency,
                severity: ProblemSeverity::Critical,
                description: format!("High interrupt latency: {} μs", analysis.max_interrupt_latency_us),
                affected_interrupts: vec![],
                recommendation: "Check for interrupt priority conflicts or long-running handlers".to_string(),
            });
        }
        
        // 检查嵌套深度过深
        if analysis.max_nesting_depth > 4 {
            problems.push(InterruptProblem {
                problem_type: InterruptProblemType::DeepNesting,
                severity: ProblemSeverity::Warning,
                description: format!("Deep interrupt nesting: {} levels", analysis.max_nesting_depth),
                affected_interrupts: vec![],
                recommendation: "Review interrupt priorities to reduce nesting depth".to_string(),
            });
        }
        
        // 检查频繁抢占
        if analysis.preemption_count > 100 {
            problems.push(InterruptProblem {
                problem_type: InterruptProblemType::FrequentPreemption,
                severity: ProblemSeverity::Info,
                description: format!("Frequent preemptions: {} in {}ms", 
                                   analysis.preemption_count, analysis.analysis_duration_ms),
                affected_interrupts: vec![],
                recommendation: "Consider adjusting interrupt priorities to reduce preemptions".to_string(),
            });
        }
        
        problems
    }
    
    pub fn generate_interrupt_report(&self) -> String {
        let mut report = String::new();
        report.push_str("Interrupt System Debug Report\n");
        report.push_str("==============================\n\n");
        
        // 统计信息
        report.push_str("Interrupt Statistics:\n");
        for stats in &self.interrupt_stats {
            if stats.call_count > 0 {
                let avg_time = if stats.call_count > 0 {
                    stats.total_execution_time_us / stats.call_count as u64
                } else {
                    0
                };
                
                report.push_str(&format!(
                    "  {} (IRQ {}): {} calls, avg: {} μs, min: {} μs, max: {} μs\n",
                    stats.name,
                    stats.irq_number,
                    stats.call_count,
                    avg_time,
                    stats.min_execution_time_us,
                    stats.max_execution_time_us
                ));
            }
        }
        
        // 性能分析
        let analysis = self.analyze_interrupt_performance(1000);
        report.push_str(&format!(
            "\nPerformance Analysis (last 1s):\n\
            - Total interrupt load: {:.1}%\n\
            - Max interrupt latency: {} μs\n\
            - Max nesting depth: {}\n\
            - Preemption count: {}\n",
            analysis.total_interrupt_load_percent,
            analysis.max_interrupt_latency_us,
            analysis.max_nesting_depth,
            analysis.preemption_count
        ));
        
        // 问题检测
        let problems = self.detect_interrupt_problems();
        if !problems.is_empty() {
            report.push_str("\nDetected Problems:\n");
            for problem in &problems {
                report.push_str(&format!(
                    "  [{:?}] {}: {}\n    Recommendation: {}\n",
                    problem.severity,
                    format!("{:?}", problem.problem_type),
                    problem.description,
                    problem.recommendation
                ));
            }
        } else {
            report.push_str("\nNo problems detected.\n");
        }
        
        report
    }
}

#[derive(Debug)]
pub struct InterruptPerformanceAnalysis {
    pub analysis_duration_ms: u32,
    pub total_interrupt_events: usize,
    pub total_interrupt_load_percent: f32,
    pub max_interrupt_latency_us: u32,
    pub max_nesting_depth: u8,
    pub preemption_count: u32,
    pub interrupt_loads: std::collections::HashMap<u8, u64>,
}

#[derive(Debug)]
pub struct InterruptProblem {
    pub problem_type: InterruptProblemType,
    pub severity: ProblemSeverity,
    pub description: String,
    pub affected_interrupts: Vec<u8>,
    pub recommendation: String,
}

#[derive(Debug, PartialEq)]
pub enum InterruptProblemType {
    HighInterruptLoad,
    HighLatency,
    DeepNesting,
    FrequentPreemption,
    MissedInterrupts,
    PriorityInversion,
}

#[derive(Debug, PartialEq)]
pub enum ProblemSeverity {
    Info,
    Warning,
    Critical,
    Emergency,
}
```

## 常见问题诊断

### 问题诊断流程

```rust
// 自动问题诊断系统
pub struct ProblemDiagnosticSystem {
    diagnostic_rules: Vec<DiagnosticRule>,
    system_state: SystemDiagnosticState,
    diagnostic_history: Vec<DiagnosticResult>,
}

#[derive(Debug, Clone)]
pub struct DiagnosticRule {
    pub rule_id: u8,
    pub name: &'static str,
    pub condition: DiagnosticCondition,
    pub severity: ProblemSeverity,
    pub description: &'static str,
    pub solution: &'static str,
}

#[derive(Debug, Clone)]
pub enum DiagnosticCondition {
    TimerNotRunning(u8),
    InterruptNotFiring(u8),
    PwmFrequencyWrong { timer_id: u8, channel: u8, expected: f32, tolerance: f32 },
    InterruptLatencyHigh(u32),
    CpuLoadHigh(f32),
    MemoryUsageHigh(f32),
    ClockConfigurationError,
    PriorityConfigurationError,
}

#[derive(Debug)]
pub struct SystemDiagnosticState {
    pub timers_running: Vec<u8>,
    pub interrupts_enabled: Vec<u8>,
    pub cpu_utilization: f32,
    pub memory_usage: f32,
    pub interrupt_latency: u32,
    pub clock_frequency: u32,
    pub last_update: u64,
}

#[derive(Debug)]
pub struct DiagnosticResult {
    pub timestamp: u64,
    pub rule_id: u8,
    pub problem_detected: bool,
    pub severity: ProblemSeverity,
    pub description: String,
    pub solution: String,
    pub system_state_snapshot: SystemDiagnosticState,
}

impl ProblemDiagnosticSystem {
    pub fn new() -> Self {
        Self {
            diagnostic_rules: Self::initialize_diagnostic_rules(),
            system_state: SystemDiagnosticState {
                timers_running: Vec::new(),
                interrupts_enabled: Vec::new(),
                cpu_utilization: 0.0,
                memory_usage: 0.0,
                interrupt_latency: 0,
                clock_frequency: 0,
                last_update: 0,
            },
            diagnostic_history: Vec::new(),
        }
    }
    
    fn initialize_diagnostic_rules() -> Vec<DiagnosticRule> {
        vec![
            DiagnosticRule {
                rule_id: 1,
                name: "Timer Not Running",
                condition: DiagnosticCondition::TimerNotRunning(0), // 通配符
                severity: ProblemSeverity::Warning,
                description: "Timer is configured but not running",
                solution: "Check timer enable bit (CEN) in TIMx_CR1 register",
            },
            DiagnosticRule {
                rule_id: 2,
                name: "Interrupt Not Firing",
                condition: DiagnosticCondition::InterruptNotFiring(0),
                severity: ProblemSeverity::Critical,
                description: "Interrupt is enabled but not being triggered",
                solution: "Check interrupt enable bits, NVIC configuration, and interrupt source",
            },
            DiagnosticRule {
                rule_id: 3,
                name: "PWM Frequency Wrong",
                condition: DiagnosticCondition::PwmFrequencyWrong { 
                    timer_id: 0, channel: 0, expected: 0.0, tolerance: 5.0 
                },
                severity: ProblemSeverity::Warning,
                description: "PWM frequency differs from expected value",
                solution: "Check prescaler and auto-reload values, verify clock configuration",
            },
            DiagnosticRule {
                rule_id: 4,
                name: "High Interrupt Latency",
                condition: DiagnosticCondition::InterruptLatencyHigh(100),
                severity: ProblemSeverity::Critical,
                description: "Interrupt response time is too high",
                solution: "Check interrupt priorities, reduce handler execution time",
            },
            DiagnosticRule {
                rule_id: 5,
                name: "High CPU Load",
                condition: DiagnosticCondition::CpuLoadHigh(80.0),
                severity: ProblemSeverity::Warning,
                description: "CPU utilization is too high",
                solution: "Optimize code, reduce interrupt frequency, use DMA",
            },
        ]
    }
    
    pub fn update_system_state(&mut self) {
        self.system_state = SystemDiagnosticState {
            timers_running: self.get_running_timers(),
            interrupts_enabled: self.get_enabled_interrupts(),
            cpu_utilization: self.measure_cpu_utilization(),
            memory_usage: self.measure_memory_usage(),
            interrupt_latency: self.measure_interrupt_latency(),
            clock_frequency: self.get_system_clock_frequency(),
            last_update: get_timestamp_us(),
        };
    }
    
    fn get_running_timers(&self) -> Vec<u8> {
        let mut running_timers = Vec::new();
        
        // 检查各个定时器的运行状态
        for timer_id in 1..=8 {
            if self.is_timer_running(timer_id) {
                running_timers.push(timer_id);
            }
        }
        
        running_timers
    }
    
    fn is_timer_running(&self, timer_id: u8) -> bool {
        unsafe {
            let tim_base = match timer_id {
                1 => 0x40010000 as *mut u32,
                2 => 0x40000000 as *mut u32,
                3 => 0x40000400 as *mut u32,
                4 => 0x40000800 as *mut u32,
                5 => 0x40000C00 as *mut u32,
                6 => 0x40001000 as *mut u32,
                7 => 0x40001400 as *mut u32,
                8 => 0x40010400 as *mut u32,
                _ => return false,
            };
            
            let cr1 = core::ptr::read_volatile(tim_base.offset(0x00 / 4));
            (cr1 & 0x01) != 0 // CEN bit
        }
    }
    
    fn get_enabled_interrupts(&self) -> Vec<u8> {
        let mut enabled_interrupts = Vec::new();
        
        unsafe {
            // 检查NVIC中断使能寄存器
            for i in 0..8 {
                let iser = (0xE000E100 + i * 4) as *const u32;
                let iser_value = core::ptr::read_volatile(iser);
                
                for bit in 0..32 {
                    if (iser_value & (1 << bit)) != 0 {
                        enabled_interrupts.push((i * 32 + bit) as u8);
                    }
                }
            }
        }
        
        enabled_interrupts
    }
    
    fn measure_cpu_utilization(&self) -> f32 {
        // 简化的CPU利用率测量
        // 实际实现需要基于空闲任务运行时间
        50.0 // 示例值
    }
    
    fn measure_memory_usage(&self) -> f32 {
        // 简化的内存使用率测量
        // 实际实现需要检查堆栈使用情况
        30.0 // 示例值
    }
    
    fn measure_interrupt_latency(&self) -> u32 {
        // 简化的中断延迟测量
        // 实际实现需要使用DWT或专门的测量代码
        15 // 示例值，单位微秒
    }
    
    fn get_system_clock_frequency(&self) -> u32 {
        // 读取系统时钟配置
        unsafe {
            let rcc_cfgr = 0x40023808 as *const u32;
            let cfgr_value = core::ptr::read_volatile(rcc_cfgr);
            
            // 简化的时钟频率计算
            // 实际实现需要解析完整的时钟配置
            match (cfgr_value >> 2) & 0x03 {
                0 => 16_000_000,  // HSI
                1 => 25_000_000,  // HSE (假设25MHz晶振)
                2 => 168_000_000, // PLL (假设配置为168MHz)
                _ => 16_000_000,
            }
        }
    }
    
    pub fn run_diagnostics(&mut self) -> Vec<DiagnosticResult> {
        self.update_system_state();
        let mut results = Vec::new();
        
        for rule in &self.diagnostic_rules {
            let problem_detected = self.evaluate_rule(rule);
            
            let result = DiagnosticResult {
                timestamp: get_timestamp_us(),
                rule_id: rule.rule_id,
                problem_detected,
                severity: rule.severity.clone(),
                description: if problem_detected { 
                    rule.description.to_string() 
                } else { 
                    "No problem detected".to_string() 
                },
                solution: rule.solution.to_string(),
                system_state_snapshot: SystemDiagnosticState {
                    timers_running: self.system_state.timers_running.clone(),
                    interrupts_enabled: self.system_state.interrupts_enabled.clone(),
                    cpu_utilization: self.system_state.cpu_utilization,
                    memory_usage: self.system_state.memory_usage,
                    interrupt_latency: self.system_state.interrupt_latency,
                    clock_frequency: self.system_state.clock_frequency,
                    last_update: self.system_state.last_update,
                },
            };
            
            results.push(result.clone());
            self.diagnostic_history.push(result);
        }
        
        // 限制历史记录大小
        if self.diagnostic_history.len() > 1000 {
            self.diagnostic_history.drain(0..100);
        }
        
        results
    }
    
    fn evaluate_rule(&self, rule: &DiagnosticRule) -> bool {
        match &rule.condition {
            DiagnosticCondition::TimerNotRunning(timer_id) => {
                if *timer_id == 0 {
                    // 检查所有定时器
                    for id in 1..=8 {
                        if self.is_timer_configured(id) && !self.system_state.timers_running.contains(&id) {
                            return true;
                        }
                    }
                    false
                } else {
                    self.is_timer_configured(*timer_id) && !self.system_state.timers_running.contains(timer_id)
                }
            },
            DiagnosticCondition::InterruptNotFiring(irq_number) => {
                if *irq_number == 0 {
                    // 检查所有定时器中断
                    for irq in [25, 28, 29, 30, 50, 54, 55, 43] { // 定时器中断号
                        if self.system_state.interrupts_enabled.contains(&irq) && 
                           !self.is_interrupt_firing(irq) {
                            return true;
                        }
                    }
                    false
                } else {
                    self.system_state.interrupts_enabled.contains(irq_number) && 
                    !self.is_interrupt_firing(*irq_number)
                }
            },
            DiagnosticCondition::PwmFrequencyWrong { timer_id, channel, expected, tolerance } => {
                if let Some(actual_freq) = self.measure_pwm_frequency(*timer_id, *channel) {
                    let error_percent = ((actual_freq - expected) / expected * 100.0).abs();
                    error_percent > *tolerance
                } else {
                    false
                }
            },
            DiagnosticCondition::InterruptLatencyHigh(threshold) => {
                self.system_state.interrupt_latency > *threshold
            },
            DiagnosticCondition::CpuLoadHigh(threshold) => {
                self.system_state.cpu_utilization > *threshold
            },
            DiagnosticCondition::MemoryUsageHigh(threshold) => {
                self.system_state.memory_usage > *threshold
            },
            DiagnosticCondition::ClockConfigurationError => {
                self.check_clock_configuration_error()
            },
            DiagnosticCondition::PriorityConfigurationError => {
                self.check_priority_configuration_error()
            },
        }
    }
    
    fn is_timer_configured(&self, timer_id: u8) -> bool {
        // 检查定时器是否已配置（非零的预分频器或自动重载值）
        unsafe {
            let tim_base = match timer_id {
                1 => 0x40010000 as *mut u32,
                2 => 0x40000000 as *mut u32,
                3 => 0x40000400 as *mut u32,
                4 => 0x40000800 as *mut u32,
                5 => 0x40000C00 as *mut u32,
                6 => 0x40001000 as *mut u32,
                7 => 0x40001400 as *mut u32,
                8 => 0x40010400 as *mut u32,
                _ => return false,
            };
            
            let psc = core::ptr::read_volatile(tim_base.offset(0x28 / 4));
            let arr = core::ptr::read_volatile(tim_base.offset(0x2C / 4));
            
            psc > 0 || arr > 0
        }
    }
    
    fn is_interrupt_firing(&self, irq_number: u8) -> bool {
        // 简化的中断触发检测
        // 实际实现需要维护中断计数器
        true // 示例值
    }
    
    fn measure_pwm_frequency(&self, timer_id: u8, channel: u8) -> Option<f32> {
        // 简化的PWM频率测量
        // 实际实现需要使用输入捕获或外部测量
        Some(1000.0) // 示例值
    }
    
    fn check_clock_configuration_error(&self) -> bool {
        // 检查时钟配置是否合理
        let expected_freq = 168_000_000; // 期望的系统时钟频率
        let tolerance = 1_000_000; // 1MHz容差
        
        (self.system_state.clock_frequency as i32 - expected_freq as i32).abs() > tolerance as i32
    }
    
    fn check_priority_configuration_error(&self) -> bool {
        // 检查中断优先级配置是否合理
        // 简化实现：检查是否有相同优先级的中断
        false // 示例值
    }
    
    pub fn generate_diagnostic_report(&self) -> String {
        let mut report = String::new();
        report.push_str("System Diagnostic Report\n");
        report.push_str("========================\n\n");
        
        // 系统状态
        report.push_str("Current System State:\n");
        report.push_str(&format!("- CPU Utilization: {:.1}%\n", self.system_state.cpu_utilization));
        report.push_str(&format!("- Memory Usage: {:.1}%\n", self.system_state.memory_usage));
        report.push_str(&format!("- Interrupt Latency: {} μs\n", self.system_state.interrupt_latency));
        report.push_str(&format!("- System Clock: {} Hz\n", self.system_state.clock_frequency));
        report.push_str(&format!("- Running Timers: {:?}\n", self.system_state.timers_running));
        report.push_str(&format!("- Enabled Interrupts: {} total\n", self.system_state.interrupts_enabled.len()));
        
        // 最近的诊断结果
        let recent_problems: Vec<_> = self.diagnostic_history.iter()
            .rev()
            .take(10)
            .filter(|r| r.problem_detected)
            .collect();
        
        if !recent_problems.is_empty() {
            report.push_str("\nRecent Problems Detected:\n");
            for problem in recent_problems {
                report.push_str(&format!(
                    "- [{:?}] {} (Rule {})\n  Solution: {}\n",
                    problem.severity,
                    problem.description,
                    problem.rule_id,
                    problem.solution
                ));
            }
        } else {
            report.push_str("\nNo recent problems detected.\n");
        }
        
        report
    }
}
```

## 调试最佳实践

### 1. 系统性调试方法

```rust
// 调试会话管理器
pub struct DebugSession {
    session_id: u32,
    start_time: u64,
    debug_tools: Vec<Box<dyn DebugTool>>,
    log_entries: Vec<DebugLogEntry>,
    breakpoints: Vec<Breakpoint>,
    watchpoints: Vec<Watchpoint>,
}

pub trait DebugTool {
    fn name(&self) -> &'static str;
    fn initialize(&mut self) -> Result<(), DebugError>;
    fn collect_data(&mut self) -> Result<DebugData, DebugError>;
    fn analyze_data(&self, data: &DebugData) -> DebugAnalysis;
}

#[derive(Debug)]
pub struct DebugLogEntry {
    pub timestamp: u64,
    pub level: LogLevel,
    pub source: String,
    pub message: String,
    pub context: Option<DebugContext>,
}

#[derive(Debug)]
pub struct DebugContext {
    pub timer_states: Vec<TimerState>,
    pub interrupt_status: InterruptStatus,
    pub cpu_registers: CpuRegisters,
    pub memory_snapshot: MemorySnapshot,
}

#[derive(Debug)]
pub struct Breakpoint {
    pub address: u32,
    pub condition: Option<String>,
    pub hit_count: u32,
    pub enabled: bool,
}

#[derive(Debug)]
pub struct Watchpoint {
    pub address: u32,
    pub size: u8,
    pub access_type: WatchpointType,
    pub condition: Option<String>,
    pub enabled: bool,
}

#[derive(Debug)]
pub enum WatchpointType {
    Read,
    Write,
    ReadWrite,
}

impl DebugSession {
    pub fn new() -> Self {
        Self {
            session_id: Self::generate_session_id(),
            start_time: get_timestamp_us(),
            debug_tools: Vec::new(),
            log_entries: Vec::new(),
            breakpoints: Vec::new(),
            watchpoints: Vec::new(),
        }
    }
    
    fn generate_session_id() -> u32 {
        // 生成唯一的会话ID
        (get_timestamp_us() & 0xFFFFFFFF) as u32
    }
    
    pub fn add_debug_tool(&mut self, tool: Box<dyn DebugTool>) {
        self.debug_tools.push(tool);
    }
    
    pub fn set_breakpoint(&mut self, address: u32, condition: Option<String>) {
        let breakpoint = Breakpoint {
            address,
            condition,
            hit_count: 0,
            enabled: true,
        };
        
        self.breakpoints.push(breakpoint);
        
        // 配置硬件断点
        self.configure_hardware_breakpoint(address);
    }
    
    fn configure_hardware_breakpoint(&self, address: u32) {
        unsafe {
            // 配置ARM Cortex-M4 FPB (Flash Patch and Breakpoint)
            let fpb_ctrl = 0xE0002000 as *mut u32;
            let fpb_comp0 = 0xE0002008 as *mut u32;
            
            // 启用FPB
            core::ptr::write_volatile(fpb_ctrl, 0x00000003);
            
            // 设置断点地址
            core::ptr::write_volatile(fpb_comp0, address | 0x00000001);
        }
    }
    
    pub fn set_watchpoint(&mut self, address: u32, size: u8, access_type: WatchpointType) {
        let watchpoint = Watchpoint {
            address,
            size,
            access_type,
            condition: None,
            enabled: true,
        };
        
        self.watchpoints.push(watchpoint);
        
        // 配置硬件观察点
        self.configure_hardware_watchpoint(address, size, &access_type);
    }
    
    fn configure_hardware_watchpoint(&self, address: u32, size: u8, access_type: &WatchpointType) {
        unsafe {
            // 配置ARM Cortex-M4 DWT (Data Watchpoint and Trace)
            let dwt_ctrl = 0xE0001000 as *mut u32;
            let dwt_comp0 = 0xE0001020 as *mut u32;
            let dwt_mask0 = 0xE0001024 as *mut u32;
            let dwt_function0 = 0xE0001028 as *mut u32;
            
            // 启用DWT
            let current_ctrl = core::ptr::read_volatile(dwt_ctrl);
            core::ptr::write_volatile(dwt_ctrl, current_ctrl | 0x00000001);
            
            // 设置比较地址
            core::ptr::write_volatile(dwt_comp0, address);
            
            // 设置掩码（根据大小）
            let mask = match size {
                1 => 0,
                2 => 1,
                4 => 2,
                8 => 3,
                _ => 0,
            };
            core::ptr::write_volatile(dwt_mask0, mask);
            
            // 设置功能（读/写/读写）
            let function = match access_type {
                WatchpointType::Read => 0x00000005,
                WatchpointType::Write => 0x00000006,
                WatchpointType::ReadWrite => 0x00000007,
            };
            core::ptr::write_volatile(dwt_function0, function);
        }
    }
    
    pub fn log_debug_event(&mut self, level: LogLevel, source: String, message: String, 
                          context: Option<DebugContext>) {
        let entry = DebugLogEntry {
            timestamp: get_timestamp_us(),
            level,
            source,
            message,
            context,
        };
        
        self.log_entries.push(entry);
        
        // 限制日志条目数量
        if self.log_entries.len() > 10000 {
            self.log_entries.drain(0..1000);
        }
    }
    
    pub fn collect_system_snapshot(&self) -> SystemSnapshot {
        SystemSnapshot {
            timestamp: get_timestamp_us(),
            cpu_registers: self.capture_cpu_registers(),
            timer_states: self.capture_all_timer_states(),
            interrupt_status: self.capture_interrupt_status(),
            memory_regions: self.capture_memory_regions(),
            stack_trace: self.capture_stack_trace(),
        }
    }
    
    fn capture_cpu_registers(&self) -> CpuRegisters {
        unsafe {
            CpuRegisters {
                r0: 0, // 需要从调试器或异常处理程序获取
                r1: 0,
                r2: 0,
                r3: 0,
                r4: 0,
                r5: 0,
                r6: 0,
                r7: 0,
                r8: 0,
                r9: 0,
                r10: 0,
                r11: 0,
                r12: 0,
                sp: cortex_m::register::msp::read(),
                lr: 0, // 需要从栈中获取
                pc: 0, // 需要从调试器获取
                psr: cortex_m::register::apsr::read(),
            }
        }
    }
    
    fn capture_all_timer_states(&self) -> Vec<TimerState> {
        let mut states = Vec::new();
        
        for timer_id in 1..=8 {
            if self.is_timer_available(timer_id) {
                states.push(self.capture_timer_state(timer_id));
            }
        }
        
        states
    }
    
    fn is_timer_available(&self, timer_id: u8) -> bool {
        // 检查定时器时钟是否使能
        unsafe {
            let rcc_apb1enr = 0x40023840 as *const u32;
            let rcc_apb2enr = 0x40023844 as *const u32;
            
            match timer_id {
                1 | 8 => {
                    let apb2enr = core::ptr::read_volatile(rcc_apb2enr);
                    (apb2enr & (1 << (timer_id - 1))) != 0
                },
                2..=7 => {
                    let apb1enr = core::ptr::read_volatile(rcc_apb1enr);
                    (apb1enr & (1 << (timer_id - 1))) != 0
                },
                _ => false,
            }
        }
    }
    
    fn capture_timer_state(&self, timer_id: u8) -> TimerState {
        // 重用之前定义的TimerState捕获逻辑
        TimerState {
            timer_id,
            control_register: 0,
            counter_value: 0,
            prescaler: 0,
            auto_reload: 0,
            status_register: 0,
            interrupt_enable: 0,
            capture_compare: [0; 4],
            timestamp: get_timestamp_us(),
        }
    }
    
    fn capture_interrupt_status(&self) -> InterruptStatus {
        unsafe {
            let nvic_iser = [
                core::ptr::read_volatile(0xE000E100 as *const u32),
                core::ptr::read_volatile(0xE000E104 as *const u32),
                core::ptr::read_volatile(0xE000E108 as *const u32),
            ];
            
            let nvic_ispr = [
                core::ptr::read_volatile(0xE000E200 as *const u32),
                core::ptr::read_volatile(0xE000E204 as *const u32),
                core::ptr::read_volatile(0xE000E208 as *const u32),
            ];
            
            let nvic_iabr = [
                core::ptr::read_volatile(0xE000E300 as *const u32),
                core::ptr::read_volatile(0xE000E304 as *const u32),
                core::ptr::read_volatile(0xE000E308 as *const u32),
            ];
            
            InterruptStatus {
                enabled_interrupts: nvic_iser,
                pending_interrupts: nvic_ispr,
                active_interrupts: nvic_iabr,
                current_interrupt: self.get_current_interrupt_number(),
                interrupt_nesting_level: self.get_interrupt_nesting_level(),
            }
        }
    }
    
    fn get_current_interrupt_number(&self) -> Option<u8> {
        unsafe {
            let icsr = core::ptr::read_volatile(0xE000ED04 as *const u32);
            let vectactive = (icsr & 0x1FF) as u8;
            
            if vectactive >= 16 {
                Some(vectactive - 16)
            } else {
                None
            }
        }
    }
    
    fn get_interrupt_nesting_level(&self) -> u8 {
        // 简化实现，实际需要维护嵌套计数器
        0
    }
    
    fn capture_memory_regions(&self) -> Vec<MemoryRegion> {
        vec![
            MemoryRegion {
                name: "SRAM".to_string(),
                start_address: 0x20000000,
                size: 128 * 1024, // 128KB
                usage_bytes: self.estimate_sram_usage(),
            },
            MemoryRegion {
                name: "Flash".to_string(),
                start_address: 0x08000000,
                size: 512 * 1024, // 512KB
                usage_bytes: self.estimate_flash_usage(),
            },
        ]
    }
    
    fn estimate_sram_usage(&self) -> u32 {
        // 简化的SRAM使用估算
        32 * 1024 // 示例值
    }
    
    fn estimate_flash_usage(&self) -> u32 {
        // 简化的Flash使用估算
        64 * 1024 // 示例值
    }
    
    fn capture_stack_trace(&self) -> Vec<StackFrame> {
        // 简化的栈跟踪实现
        vec![
            StackFrame {
                function_name: "main".to_string(),
                address: 0x08000100,
                file: Some("main.rs".to_string()),
                line: Some(42),
            },
        ]
    }
    
    pub fn generate_debug_report(&self) -> String {
        let mut report = String::new();
        
        report.push_str(&format!("Debug Session Report (ID: {})\n", self.session_id));
        report.push_str("=====================================\n\n");
        
        let duration = get_timestamp_us() - self.start_time;
        report.push_str(&format!("Session Duration: {} ms\n", duration / 1000));
        report.push_str(&format!("Log Entries: {}\n", self.log_entries.len()));
        report.push_str(&format!("Breakpoints: {}\n", self.breakpoints.len()));
        report.push_str(&format!("Watchpoints: {}\n", self.watchpoints.len()));
        
        // 错误和警告统计
        let error_count = self.log_entries.iter()
            .filter(|e| matches!(e.level, LogLevel::Error))
            .count();
        let warning_count = self.log_entries.iter()
            .filter(|e| matches!(e.level, LogLevel::Warning))
            .count();
        
        report.push_str(&format!("Errors: {}, Warnings: {}\n\n", error_count, warning_count));
        
        // 最近的重要事件
        let recent_events: Vec<_> = self.log_entries.iter()
            .rev()
            .take(10)
            .filter(|e| matches!(e.level, LogLevel::Error | LogLevel::Warning))
            .collect();
        
        if !recent_events.is_empty() {
            report.push_str("Recent Important Events:\n");
            for event in recent_events {
                report.push_str(&format!(
                    "- [{}] {}: {}\n",
                    level_to_str(event.level.clone()),
                    event.source,
                    event.message
                ));
            }
        }
        
        report
    }
}

#[derive(Debug)]
pub struct SystemSnapshot {
    pub timestamp: u64,
    pub cpu_registers: CpuRegisters,
    pub timer_states: Vec<TimerState>,
    pub interrupt_status: InterruptStatus,
    pub memory_regions: Vec<MemoryRegion>,
    pub stack_trace: Vec<StackFrame>,
}

#[derive(Debug)]
pub struct CpuRegisters {
    pub r0: u32, pub r1: u32, pub r2: u32, pub r3: u32,
    pub r4: u32, pub r5: u32, pub r6: u32, pub r7: u32,
    pub r8: u32, pub r9: u32, pub r10: u32, pub r11: u32,
    pub r12: u32, pub sp: u32, pub lr: u32, pub pc: u32,
    pub psr: u32,
}

#[derive(Debug)]
pub struct InterruptStatus {
    pub enabled_interrupts: [u32; 3],
    pub pending_interrupts: [u32; 3],
    pub active_interrupts: [u32; 3],
    pub current_interrupt: Option<u8>,
    pub interrupt_nesting_level: u8,
}

#[derive(Debug)]
pub struct MemoryRegion {
    pub name: String,
    pub start_address: u32,
    pub size: u32,
    pub usage_bytes: u32,
}

#[derive(Debug)]
pub struct StackFrame {
    pub function_name: String,
    pub address: u32,
    pub file: Option<String>,
    pub line: Option<u32>,
}

#[derive(Debug)]
pub enum DebugError {
    InitializationFailed,
    DataCollectionFailed,
    HardwareNotSupported,
    InvalidConfiguration,
}

#[derive(Debug)]
pub struct DebugData {
    pub timestamp: u64,
    pub data_type: String,
    pub raw_data: Vec<u8>,
}

#[derive(Debug)]
pub struct DebugAnalysis {
    pub summary: String,
    pub findings: Vec<String>,
    pub recommendations: Vec<String>,
}

#[derive(Debug)]
pub struct MemorySnapshot {
    pub regions: Vec<MemoryRegion>,
    pub heap_usage: u32,
    pub stack_usage: u32,
}
```

### 2. 调试工具使用指南

#### 使用ST-Link调试器

1. **连接配置**
   - 确保ST-Link驱动正确安装
   - 检查SWD连接线路
   - 验证目标板供电

2. **调试会话设置**
   ```rust
   // 调试配置示例
   pub fn setup_debug_session() {
       let mut session = DebugSession::new();
       
       // 添加定时器监控工具
       session.add_debug_tool(Box::new(TimerDebugMonitor::new()));
       
       // 添加中断跟踪工具
       session.add_debug_tool(Box::new(InterruptTracer::new()));
       
       // 设置关键断点
       session.set_breakpoint(0x08000100, None); // main函数
       session.set_breakpoint(0x08000200, Some("timer_counter > 1000".to_string()));
       
       // 设置内存观察点
       session.set_watchpoint(0x20000000, 4, WatchpointType::Write);
   }
   ```

#### 使用逻辑分析仪

1. **信号连接**
   - PWM输出信号
   - 中断触发信号
   - 时钟信号
   - 调试输出信号

2. **触发配置**
   - 上升沿/下降沿触发
   - 脉宽触发
   - 模式触发

#### 使用示波器

1. **时序测量**
   - PWM频率和占空比
   - 中断响应时间
   - 信号建立和保持时间

2. **信号质量分析**
   - 上升/下降时间
   - 过冲和振铃
   - 噪声水平

### 3. 常见问题快速诊断

```rust
// 快速诊断工具
pub struct QuickDiagnostic;

impl QuickDiagnostic {
    pub fn diagnose_timer_not_working(timer_id: u8) -> Vec<String> {
        let mut checks = Vec::new();
        
        // 检查时钟使能
        if !Self::is_timer_clock_enabled(timer_id) {
            checks.push(format!("Timer {} clock not enabled in RCC", timer_id));
        }
        
        // 检查定时器使能
        if !Self::is_timer_enabled(timer_id) {
            checks.push(format!("Timer {} not enabled (CEN bit)", timer_id));
        }
        
        // 检查配置
        if !Self::is_timer_configured(timer_id) {
            checks.push(format!("Timer {} not properly configured", timer_id));
        }
        
        checks
    }
    
    pub fn diagnose_interrupt_not_firing(irq_number: u8) -> Vec<String> {
        let mut checks = Vec::new();
        
        // 检查NVIC使能
        if !Self::is_nvic_interrupt_enabled(irq_number) {
            checks.push(format!("Interrupt {} not enabled in NVIC", irq_number));
        }
        
        // 检查中断源使能
        if !Self::is_interrupt_source_enabled(irq_number) {
            checks.push(format!("Interrupt {} source not enabled", irq_number));
        }
        
        // 检查优先级配置
        if Self::get_interrupt_priority(irq_number) == 0 {
            checks.push(format!("Interrupt {} priority not configured", irq_number));
        }
        
        checks
    }
    
    pub fn diagnose_pwm_not_working(timer_id: u8, channel: u8) -> Vec<String> {
        let mut checks = Vec::new();
        
        // 检查定时器基本功能
        checks.extend(Self::diagnose_timer_not_working(timer_id));
        
        // 检查PWM模式配置
        if !Self::is_pwm_mode_configured(timer_id, channel) {
            checks.push(format!("PWM mode not configured for Timer{} CH{}", timer_id, channel));
        }
        
        // 检查GPIO配置
        if !Self::is_gpio_configured_for_pwm(timer_id, channel) {
            checks.push(format!("GPIO not configured for Timer{} CH{} PWM output", timer_id, channel));
        }
        
        // 检查比较值
        if !Self::is_compare_value_valid(timer_id, channel) {
            checks.push(format!("Invalid compare value for Timer{} CH{}", timer_id, channel));
        }
        
        checks
    }
    
    // 辅助检查函数的简化实现
    fn is_timer_clock_enabled(timer_id: u8) -> bool { true }
    fn is_timer_enabled(timer_id: u8) -> bool { true }
    fn is_timer_configured(timer_id: u8) -> bool { true }
    fn is_nvic_interrupt_enabled(irq_number: u8) -> bool { true }
    fn is_interrupt_source_enabled(irq_number: u8) -> bool { true }
    fn get_interrupt_priority(irq_number: u8) -> u8 { 1 }
    fn is_pwm_mode_configured(timer_id: u8, channel: u8) -> bool { true }
    fn is_gpio_configured_for_pwm(timer_id: u8, channel: u8) -> bool { true }
    fn is_compare_value_valid(timer_id: u8, channel: u8) -> bool { true }
}
```

## 总结

本调试指南提供了完整的定时器和中断系统调试方法，包括：

1. **调试环境搭建** - 硬件和软件工具配置
2. **定时器调试** - 状态监控、PWM信号分析
3. **中断系统调试** - 中断跟踪、性能分析
4. **问题诊断** - 自动化问题检测和解决方案
5. **调试最佳实践** - 系统性调试方法和工具使用

通过这些工具和方法，开发者可以快速定位和解决定时器、中断相关的问题，提高开发效率和系统可靠性。
        report.push_str