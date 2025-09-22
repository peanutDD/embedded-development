#![no_std]
#![no_main]

//! # 数字输入处理程序
//! 
//! 专门演示数字输入处理的各种功能：
//! - 多通道输入监控
//! - 边沿检测和防抖
//! - 输入模式识别
//! - 事件队列管理

use panic_halt as _;
use cortex_m_rt::entry;
use heapless::{Vec, Deque, String};
use critical_section::Mutex;
use core::cell::RefCell;

use digital_io::{
    DigitalInputController, InputConfig, PinState, PinMode,
    EdgeDetection, DebounceConfig, EdgeHistory,
};

/// 输入事件类型
#[derive(Debug, Clone, Copy)]
pub enum InputEvent {
    /// 引脚状态变化
    StateChange { pin: usize, old_state: PinState, new_state: PinState, timestamp: u32 },
    /// 边沿检测
    EdgeDetected { pin: usize, edge_type: EdgeType, timestamp: u32 },
    /// 模式识别
    PatternDetected { pattern: InputPattern, confidence: u8, timestamp: u32 },
    /// 超时事件
    Timeout { pin: usize, duration: u32, timestamp: u32 },
}

/// 边沿类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeType {
    Rising,
    Falling,
}

/// 输入模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InputPattern {
    SingleClick,
    DoubleClick,
    LongPress,
    Sequence,
    Morse,
    Custom(u8),
}

/// 输入处理器配置
#[derive(Debug, Clone, Copy)]
pub struct InputProcessorConfig {
    /// 事件队列大小
    pub event_queue_size: usize,
    /// 模式识别使能
    pub enable_pattern_recognition: bool,
    /// 超时检测使能
    pub enable_timeout_detection: bool,
    /// 统计收集使能
    pub enable_statistics: bool,
}

/// 输入处理器
pub struct InputProcessor {
    /// 数字输入控制器
    digital_input: DigitalInputController<32>,
    /// 事件队列
    event_queue: Deque<InputEvent, 64>,
    /// 模式识别器
    pattern_recognizer: PatternRecognizer,
    /// 超时监控器
    timeout_monitor: TimeoutMonitor,
    /// 配置
    config: InputProcessorConfig,
    /// 统计信息
    stats: ProcessorStats,
}

/// 模式识别器
pub struct PatternRecognizer {
    /// 按键序列缓冲区
    key_sequence: Vec<KeyPress, 16>,
    /// 识别状态
    recognition_state: RecognitionState,
    /// 配置参数
    config: PatternConfig,
}

/// 按键按下记录
#[derive(Debug, Clone, Copy)]
pub struct KeyPress {
    pub pin: usize,
    pub press_time: u32,
    pub release_time: u32,
    pub duration: u32,
}

/// 识别状态
#[derive(Debug, Clone, Copy)]
pub enum RecognitionState {
    Idle,
    Collecting,
    Analyzing,
    Completed,
}

/// 模式配置
#[derive(Debug, Clone, Copy)]
pub struct PatternConfig {
    /// 单击最大持续时间
    pub single_click_max_duration: u32,
    /// 双击最大间隔
    pub double_click_max_interval: u32,
    /// 长按最小持续时间
    pub long_press_min_duration: u32,
    /// 序列最大间隔
    pub sequence_max_interval: u32,
}

/// 超时监控器
pub struct TimeoutMonitor {
    /// 监控的引脚
    monitored_pins: [TimeoutConfig; 32],
    /// 活动超时
    active_timeouts: Vec<ActiveTimeout, 32>,
}

/// 超时配置
#[derive(Debug, Clone, Copy)]
pub struct TimeoutConfig {
    pub enabled: bool,
    pub timeout_duration: u32,
    pub auto_reset: bool,
}

/// 活动超时
#[derive(Debug, Clone, Copy)]
pub struct ActiveTimeout {
    pub pin: usize,
    pub start_time: u32,
    pub timeout_duration: u32,
}

/// 处理器统计
#[derive(Debug, Default)]
pub struct ProcessorStats {
    pub total_events: u32,
    pub state_changes: u32,
    pub edge_detections: u32,
    pub patterns_detected: u32,
    pub timeouts_occurred: u32,
    pub queue_overflows: u32,
}

impl Default for InputProcessorConfig {
    fn default() -> Self {
        Self {
            event_queue_size: 64,
            enable_pattern_recognition: true,
            enable_timeout_detection: true,
            enable_statistics: true,
        }
    }
}

impl Default for PatternConfig {
    fn default() -> Self {
        Self {
            single_click_max_duration: 200,
            double_click_max_interval: 500,
            long_press_min_duration: 1000,
            sequence_max_interval: 2000,
        }
    }
}

impl Default for TimeoutConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            timeout_duration: 5000,
            auto_reset: true,
        }
    }
}

impl PatternRecognizer {
    /// 创建新的模式识别器
    pub fn new(config: PatternConfig) -> Self {
        Self {
            key_sequence: Vec::new(),
            recognition_state: RecognitionState::Idle,
            config,
        }
    }
    
    /// 添加按键事件
    pub fn add_key_event(&mut self, pin: usize, press_time: u32, release_time: u32) -> Option<InputPattern> {
        let duration = release_time.saturating_sub(press_time);
        
        let key_press = KeyPress {
            pin,
            press_time,
            release_time,
            duration,
        };
        
        // 添加到序列
        if self.key_sequence.push(key_press).is_err() {
            // 队列满，移除最旧的事件
            self.key_sequence.remove(0);
            let _ = self.key_sequence.push(key_press);
        }
        
        // 分析模式
        self.analyze_pattern()
    }
    
    /// 分析输入模式
    fn analyze_pattern(&mut self) -> Option<InputPattern> {
        if self.key_sequence.is_empty() {
            return None;
        }
        
        let last_key = self.key_sequence[self.key_sequence.len() - 1];
        
        // 检测长按
        if last_key.duration >= self.config.long_press_min_duration {
            self.clear_sequence();
            return Some(InputPattern::LongPress);
        }
        
        // 检测单击
        if last_key.duration <= self.config.single_click_max_duration {
            // 检查是否可能是双击
            if self.key_sequence.len() >= 2 {
                let prev_key = self.key_sequence[self.key_sequence.len() - 2];
                let interval = last_key.press_time.saturating_sub(prev_key.release_time);
                
                if interval <= self.config.double_click_max_interval && 
                   prev_key.pin == last_key.pin &&
                   prev_key.duration <= self.config.single_click_max_duration {
                    self.clear_sequence();
                    return Some(InputPattern::DoubleClick);
                }
            }
            
            // 等待可能的第二次点击
            return None;
        }
        
        // 检测序列模式
        if self.key_sequence.len() >= 3 {
            if self.detect_sequence_pattern() {
                self.clear_sequence();
                return Some(InputPattern::Sequence);
            }
        }
        
        // 检测摩尔斯码
        if self.detect_morse_pattern() {
            self.clear_sequence();
            return Some(InputPattern::Morse);
        }
        
        None
    }
    
    /// 检测序列模式
    fn detect_sequence_pattern(&self) -> bool {
        if self.key_sequence.len() < 3 {
            return false;
        }
        
        // 检查是否为递增或递减序列
        let mut increasing = true;
        let mut decreasing = true;
        
        for i in 1..self.key_sequence.len() {
            let curr_duration = self.key_sequence[i].duration;
            let prev_duration = self.key_sequence[i-1].duration;
            
            if curr_duration <= prev_duration {
                increasing = false;
            }
            if curr_duration >= prev_duration {
                decreasing = false;
            }
        }
        
        increasing || decreasing
    }
    
    /// 检测摩尔斯码模式
    fn detect_morse_pattern(&self) -> bool {
        if self.key_sequence.len() < 2 {
            return false;
        }
        
        // 简单的摩尔斯码检测：短按和长按的组合
        let mut has_short = false;
        let mut has_long = false;
        
        for key in &self.key_sequence {
            if key.duration < 300 {
                has_short = true;
            } else if key.duration > 300 && key.duration < 1000 {
                has_long = true;
            }
        }
        
        has_short && has_long
    }
    
    /// 清除序列
    fn clear_sequence(&mut self) {
        self.key_sequence.clear();
        self.recognition_state = RecognitionState::Idle;
    }
    
    /// 超时处理
    pub fn handle_timeout(&mut self, timestamp: u32) {
        // 清理过期的按键事件
        self.key_sequence.retain(|key| {
            timestamp.saturating_sub(key.release_time) < self.config.sequence_max_interval
        });
        
        if self.key_sequence.is_empty() {
            self.recognition_state = RecognitionState::Idle;
        }
    }
}

impl TimeoutMonitor {
    /// 创建新的超时监控器
    pub fn new() -> Self {
        Self {
            monitored_pins: [TimeoutConfig::default(); 32],
            active_timeouts: Vec::new(),
        }
    }
    
    /// 配置引脚超时
    pub fn configure_pin_timeout(&mut self, pin: usize, config: TimeoutConfig) -> Result<(), ()> {
        if pin >= 32 {
            return Err(());
        }
        
        self.monitored_pins[pin] = config;
        Ok(())
    }
    
    /// 开始超时监控
    pub fn start_timeout(&mut self, pin: usize, timestamp: u32) -> Result<(), ()> {
        if pin >= 32 || !self.monitored_pins[pin].enabled {
            return Err(());
        }
        
        let timeout = ActiveTimeout {
            pin,
            start_time: timestamp,
            timeout_duration: self.monitored_pins[pin].timeout_duration,
        };
        
        // 移除该引脚的现有超时
        self.active_timeouts.retain(|t| t.pin != pin);
        
        // 添加新超时
        if self.active_timeouts.push(timeout).is_err() {
            // 队列满，移除最旧的超时
            self.active_timeouts.remove(0);
            let _ = self.active_timeouts.push(timeout);
        }
        
        Ok(())
    }
    
    /// 停止超时监控
    pub fn stop_timeout(&mut self, pin: usize) {
        self.active_timeouts.retain(|t| t.pin != pin);
    }
    
    /// 检查超时
    pub fn check_timeouts(&mut self, timestamp: u32) -> Vec<InputEvent, 8> {
        let mut timeout_events = Vec::new();
        let mut expired_timeouts = Vec::new();
        
        for (index, timeout) in self.active_timeouts.iter().enumerate() {
            let elapsed = timestamp.saturating_sub(timeout.start_time);
            
            if elapsed >= timeout.timeout_duration {
                let event = InputEvent::Timeout {
                    pin: timeout.pin,
                    duration: elapsed,
                    timestamp,
                };
                
                if timeout_events.push(event).is_err() {
                    break;
                }
                
                expired_timeouts.push(index);
            }
        }
        
        // 移除过期的超时
        for &index in expired_timeouts.iter().rev() {
            self.active_timeouts.remove(index);
        }
        
        timeout_events
    }
}

impl InputProcessor {
    /// 创建新的输入处理器
    pub fn new(config: InputProcessorConfig) -> Self {
        // 配置输入引脚
        let input_configs = [
            InputConfig {
                mode: PinMode::InputPullUp,
                edge_detection: EdgeDetection::Both,
                enable_debounce: true,
                enable_interrupt: true,
            }; 32
        ];
        
        let debounce_config = DebounceConfig {
            debounce_time_ms: 50,
            stable_count: 3,
            enable_hysteresis: true,
        };
        
        Self {
            digital_input: DigitalInputController::new(input_configs, debounce_config),
            event_queue: Deque::new(),
            pattern_recognizer: PatternRecognizer::new(PatternConfig::default()),
            timeout_monitor: TimeoutMonitor::new(),
            config,
            stats: ProcessorStats::default(),
        }
    }
    
    /// 初始化处理器
    pub fn init(&mut self) -> Result<(), ()> {
        // 配置超时监控
        for pin in 0..4 {
            let timeout_config = TimeoutConfig {
                enabled: true,
                timeout_duration: 5000,
                auto_reset: true,
            };
            self.timeout_monitor.configure_pin_timeout(pin, timeout_config)?;
        }
        
        // 重置统计
        self.digital_input.reset_stats();
        
        Ok(())
    }
    
    /// 处理输入更新
    pub fn process(&mut self, timestamp: u32) -> Result<(), ()> {
        // 读取硬件输入
        let hardware_inputs = self.read_hardware_inputs();
        
        // 处理每个引脚
        for (pin, &new_state) in hardware_inputs.iter().enumerate() {
            let pin_state = if new_state { PinState::High } else { PinState::Low };
            
            if let Ok(old_state) = self.digital_input.read_pin(pin) {
                if let Ok(edge_detected) = self.digital_input.update_pin_state(pin, pin_state, timestamp) {
                    // 处理状态变化
                    if old_state != pin_state {
                        self.handle_state_change(pin, old_state, pin_state, timestamp)?;
                    }
                    
                    // 处理边沿检测
                    if edge_detected {
                        self.handle_edge_detection(pin, old_state, pin_state, timestamp)?;
                    }
                }
            }
        }
        
        // 检查超时
        if self.config.enable_timeout_detection {
            let timeout_events = self.timeout_monitor.check_timeouts(timestamp);
            for event in timeout_events {
                self.add_event(event)?;
            }
        }
        
        // 处理模式识别超时
        if self.config.enable_pattern_recognition {
            self.pattern_recognizer.handle_timeout(timestamp);
        }
        
        Ok(())
    }
    
    /// 读取硬件输入（模拟）
    fn read_hardware_inputs(&self) -> [bool; 32] {
        // 模拟输入变化
        let mut inputs = [false; 32];
        
        // 模拟一些输入活动
        let time = self.stats.total_events;
        inputs[0] = (time / 100) % 10 < 2;  // 20% 占空比
        inputs[1] = (time / 200) % 20 < 1;  // 5% 占空比
        inputs[2] = (time / 50) % 5 < 1;    // 20% 占空比
        inputs[3] = (time / 300) % 30 < 3;  // 10% 占空比
        
        inputs
    }
    
    /// 处理状态变化
    fn handle_state_change(&mut self, pin: usize, old_state: PinState, new_state: PinState, timestamp: u32) -> Result<(), ()> {
        let event = InputEvent::StateChange {
            pin,
            old_state,
            new_state,
            timestamp,
        };
        
        self.add_event(event)?;
        self.stats.state_changes += 1;
        
        // 处理超时监控
        if self.config.enable_timeout_detection {
            match new_state {
                PinState::High => {
                    self.timeout_monitor.start_timeout(pin, timestamp)?;
                },
                PinState::Low => {
                    self.timeout_monitor.stop_timeout(pin);
                },
                _ => {}
            }
        }
        
        Ok(())
    }
    
    /// 处理边沿检测
    fn handle_edge_detection(&mut self, pin: usize, old_state: PinState, new_state: PinState, timestamp: u32) -> Result<(), ()> {
        let edge_type = match (old_state, new_state) {
            (PinState::Low, PinState::High) => EdgeType::Rising,
            (PinState::High, PinState::Low) => EdgeType::Falling,
            _ => return Ok(()),
        };
        
        let event = InputEvent::EdgeDetected {
            pin,
            edge_type,
            timestamp,
        };
        
        self.add_event(event)?;
        self.stats.edge_detections += 1;
        
        // 模式识别处理
        if self.config.enable_pattern_recognition && edge_type == EdgeType::Falling {
            // 获取按键持续时间
            if let Ok(history) = self.digital_input.get_edge_history(pin) {
                let press_time = history.last_edge_time;
                let duration = timestamp.saturating_sub(press_time);
                
                if let Some(pattern) = self.pattern_recognizer.add_key_event(pin, press_time, timestamp) {
                    let pattern_event = InputEvent::PatternDetected {
                        pattern,
                        confidence: 85, // 模拟置信度
                        timestamp,
                    };
                    
                    self.add_event(pattern_event)?;
                    self.stats.patterns_detected += 1;
                }
            }
        }
        
        Ok(())
    }
    
    /// 添加事件到队列
    fn add_event(&mut self, event: InputEvent) -> Result<(), ()> {
        if self.event_queue.push_back(event).is_err() {
            // 队列满，移除最旧的事件
            self.event_queue.pop_front();
            self.event_queue.push_back(event).map_err(|_| ())?;
            self.stats.queue_overflows += 1;
        }
        
        self.stats.total_events += 1;
        Ok(())
    }
    
    /// 获取下一个事件
    pub fn get_next_event(&mut self) -> Option<InputEvent> {
        self.event_queue.pop_front()
    }
    
    /// 获取事件队列长度
    pub fn get_queue_length(&self) -> usize {
        self.event_queue.len()
    }
    
    /// 清空事件队列
    pub fn clear_event_queue(&mut self) {
        self.event_queue.clear();
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> &ProcessorStats {
        &self.stats
    }
    
    /// 获取详细统计
    pub fn get_detailed_stats(&self) -> DetailedInputStats {
        DetailedInputStats {
            processor: self.stats,
            digital_input: *self.digital_input.get_stats(),
            queue_length: self.event_queue.len(),
        }
    }
}

/// 详细输入统计
#[derive(Debug)]
pub struct DetailedInputStats {
    pub processor: ProcessorStats,
    pub digital_input: digital_io::InputStats,
    pub queue_length: usize,
}

/// 全局输入处理器
static INPUT_PROCESSOR: Mutex<RefCell<Option<InputProcessor>>> = 
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 创建配置
    let config = InputProcessorConfig {
        event_queue_size: 64,
        enable_pattern_recognition: true,
        enable_timeout_detection: true,
        enable_statistics: true,
    };
    
    // 初始化输入处理器
    let mut processor = InputProcessor::new(config);
    processor.init().unwrap();
    
    // 存储到全局变量
    critical_section::with(|cs| {
        INPUT_PROCESSOR.borrow(cs).replace(Some(processor));
    });
    
    let mut timestamp = 0u32;
    
    loop {
        timestamp = timestamp.wrapping_add(1);
        
        // 处理输入
        critical_section::with(|cs| {
            if let Some(ref mut processor) = INPUT_PROCESSOR.borrow(cs).borrow_mut().as_mut() {
                let _ = processor.process(timestamp);
                
                // 处理事件队列
                while let Some(event) = processor.get_next_event() {
                    // 在实际应用中，这里可以处理具体的事件
                    match event {
                        InputEvent::StateChange { pin, old_state, new_state, timestamp } => {
                            // 处理状态变化
                        },
                        InputEvent::EdgeDetected { pin, edge_type, timestamp } => {
                            // 处理边沿检测
                        },
                        InputEvent::PatternDetected { pattern, confidence, timestamp } => {
                            // 处理模式识别
                        },
                        InputEvent::Timeout { pin, duration, timestamp } => {
                            // 处理超时事件
                        }
                    }
                }
                
                // 每1000次循环输出统计信息
                if timestamp % 1000 == 0 {
                    let stats = processor.get_detailed_stats();
                    // 在实际应用中，这里可以通过串口输出统计信息
                }
            }
        });
        
        // 简单延时
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
    }
}