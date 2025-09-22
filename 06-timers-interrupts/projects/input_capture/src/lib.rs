#![no_std]

use heapless::{Vec, Deque};
use libm::{sqrt, fabs};
use core::marker::PhantomData;

/// 输入捕获特征
pub trait InputCapture {
    type Error;
    type CaptureValue;
    
    /// 启动捕获
    fn start_capture(&mut self) -> Result<(), Self::Error>;
    
    /// 停止捕获
    fn stop_capture(&mut self) -> Result<(), Self::Error>;
    
    /// 读取捕获值
    fn read_capture(&mut self) -> Result<Option<Self::CaptureValue>, Self::Error>;
    
    /// 设置捕获模式
    fn set_capture_mode(&mut self, mode: CaptureMode) -> Result<(), Self::Error>;
    
    /// 获取捕获状态
    fn is_capture_ready(&self) -> bool;
}

/// 捕获模式
#[derive(Debug, Clone, Copy)]
pub enum CaptureMode {
    RisingEdge,
    FallingEdge,
    BothEdges,
}

/// 捕获事件
#[derive(Debug, Clone, Copy)]
pub struct CaptureEvent {
    pub timestamp: u32,
    pub edge_type: EdgeType,
    pub channel: u8,
}

/// 边沿类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeType {
    Rising,
    Falling,
}

/// 频率计
pub struct FrequencyMeter {
    capture_buffer: Deque<CaptureEvent, 100>,
    last_capture: Option<u32>,
    frequency: f32,
    period_us: f32,
    timer_frequency: u32,
    measurement_window: u32,
    edge_count: u32,
    capture_mode: CaptureMode,
}

/// 脉冲计数器
pub struct PulseCounter {
    pulse_count: u64,
    last_timestamp: u32,
    pulse_rate: f32,
    pulse_buffer: Vec<u32, 1000>,
    counting_enabled: bool,
    count_mode: CountMode,
    debounce_time_us: u32,
    timer_frequency: u32,
}

/// 计数模式
#[derive(Debug, Clone, Copy)]
pub enum CountMode {
    RisingEdge,
    FallingEdge,
    BothEdges,
}

/// 旋转编码器
pub struct RotaryEncoder {
    position: i32,
    last_a_state: bool,
    last_b_state: bool,
    direction: RotationDirection,
    steps_per_revolution: u32,
    velocity: f32,
    last_update_time: u32,
    velocity_buffer: Deque<f32, 10>,
    timer_frequency: u32,
}

/// 旋转方向
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotationDirection {
    Clockwise,
    CounterClockwise,
    Stopped,
}

/// 超声波传感器
pub struct UltrasonicSensor {
    trigger_sent: bool,
    echo_start_time: Option<u32>,
    echo_end_time: Option<u32>,
    distance_cm: f32,
    measurement_timeout_us: u32,
    sound_speed_cm_per_us: f32,
    timer_frequency: u32,
    measurements: Deque<f32, 20>,
}

/// 输入捕获通道
pub struct CaptureChannel<T> {
    channel_id: u8,
    timer_peripheral: PhantomData<T>,
    capture_mode: CaptureMode,
    prescaler: u16,
    filter: u8,
    polarity: CapturePolarity,
}

/// 捕获极性
#[derive(Debug, Clone, Copy)]
pub enum CapturePolarity {
    Rising,
    Falling,
    Both,
}

/// 多通道输入捕获管理器
pub struct MultiChannelCapture<T> {
    channels: Vec<CaptureChannel<T>, 4>,
    capture_buffer: Vec<CaptureEvent, 1000>,
    timer_frequency: u32,
    active_channels: u8,
}

/// 测量结果
#[derive(Debug, Clone)]
pub struct MeasurementResult {
    pub frequency_hz: f32,
    pub period_us: f32,
    pub duty_cycle: f32,
    pub pulse_width_us: f32,
    pub pulse_count: u64,
    pub measurement_time_ms: u32,
}

/// 统计信息
#[derive(Debug, Clone)]
pub struct CaptureStatistics {
    pub total_captures: u32,
    pub missed_captures: u32,
    pub min_period_us: f32,
    pub max_period_us: f32,
    pub avg_period_us: f32,
    pub jitter_us: f32,
}

impl FrequencyMeter {
    /// 创建新的频率计
    pub fn new(timer_frequency: u32) -> Self {
        Self {
            capture_buffer: Deque::new(),
            last_capture: None,
            frequency: 0.0,
            period_us: 0.0,
            timer_frequency,
            measurement_window: timer_frequency, // 1秒测量窗口
            edge_count: 0,
            capture_mode: CaptureMode::RisingEdge,
        }
    }

    /// 设置测量窗口
    pub fn set_measurement_window(&mut self, window_ms: u32) {
        self.measurement_window = (self.timer_frequency / 1000) * window_ms;
    }

    /// 添加捕获事件
    pub fn add_capture(&mut self, timestamp: u32) -> Result<(), &'static str> {
        let event = CaptureEvent {
            timestamp,
            edge_type: EdgeType::Rising,
            channel: 0,
        };

        self.capture_buffer.push_back(event).map_err(|_| "Buffer full")?;
        
        // 保持缓冲区大小
        if self.capture_buffer.len() > 50 {
            self.capture_buffer.pop_front();
        }

        self.edge_count += 1;
        Ok(())
    }

    /// 计算频率
    pub fn calculate_frequency(&mut self) -> f32 {
        if self.capture_buffer.len() < 2 {
            return 0.0;
        }

        let mut periods = Vec::<f32, 50>::new();
        let mut last_timestamp = None;

        for event in &self.capture_buffer {
            if let Some(last_ts) = last_timestamp {
                let period_ticks = if event.timestamp >= last_ts {
                    event.timestamp - last_ts
                } else {
                    // 处理定时器溢出
                    (u32::MAX - last_ts) + event.timestamp + 1
                };
                
                let period_us = (period_ticks as f64 * 1_000_000.0 / self.timer_frequency as f64) as f32;
                periods.push(period_us).ok();
            }
            last_timestamp = Some(event.timestamp);
        }

        if periods.is_empty() {
            return 0.0;
        }

        // 计算平均周期
        let avg_period: f32 = periods.iter().sum::<f32>() / periods.len() as f32;
        self.period_us = avg_period;
        
        if avg_period > 0.0 {
            self.frequency = 1_000_000.0 / avg_period;
        } else {
            self.frequency = 0.0;
        }

        self.frequency
    }

    /// 获取测量结果
    pub fn get_measurement(&self) -> MeasurementResult {
        MeasurementResult {
            frequency_hz: self.frequency,
            period_us: self.period_us,
            duty_cycle: 0.5, // 简化值
            pulse_width_us: self.period_us / 2.0,
            pulse_count: self.edge_count as u64,
            measurement_time_ms: 1000, // 固定1秒
        }
    }

    /// 重置测量
    pub fn reset(&mut self) {
        self.capture_buffer.clear();
        self.last_capture = None;
        self.frequency = 0.0;
        self.period_us = 0.0;
        self.edge_count = 0;
    }
}

impl PulseCounter {
    /// 创建新的脉冲计数器
    pub fn new(timer_frequency: u32) -> Self {
        Self {
            pulse_count: 0,
            last_timestamp: 0,
            pulse_rate: 0.0,
            pulse_buffer: Vec::new(),
            counting_enabled: true,
            count_mode: CountMode::RisingEdge,
            debounce_time_us: 1000, // 1ms防抖
            timer_frequency,
        }
    }

    /// 设置计数模式
    pub fn set_count_mode(&mut self, mode: CountMode) {
        self.count_mode = mode;
    }

    /// 设置防抖时间
    pub fn set_debounce_time(&mut self, debounce_us: u32) {
        self.debounce_time_us = debounce_us;
    }

    /// 添加脉冲
    pub fn add_pulse(&mut self, timestamp: u32, edge_type: EdgeType) -> Result<(), &'static str> {
        if !self.counting_enabled {
            return Ok(());
        }

        // 检查防抖
        let time_diff = if timestamp >= self.last_timestamp {
            timestamp - self.last_timestamp
        } else {
            (u32::MAX - self.last_timestamp) + timestamp + 1
        };

        let time_diff_us = (time_diff as f64 * 1_000_000.0 / self.timer_frequency as f64) as u32;
        
        if time_diff_us < self.debounce_time_us {
            return Ok(()); // 忽略抖动
        }

        // 根据计数模式决定是否计数
        let should_count = match self.count_mode {
            CountMode::RisingEdge => edge_type == EdgeType::Rising,
            CountMode::FallingEdge => edge_type == EdgeType::Falling,
            CountMode::BothEdges => true,
        };

        if should_count {
            self.pulse_count += 1;
            self.pulse_buffer.push(timestamp).map_err(|_| "Buffer full")?;
            
            // 保持缓冲区大小
            if self.pulse_buffer.len() > 500 {
                self.pulse_buffer.remove(0);
            }
        }

        self.last_timestamp = timestamp;
        Ok(())
    }

    /// 计算脉冲率
    pub fn calculate_pulse_rate(&mut self) -> f32 {
        if self.pulse_buffer.len() < 2 {
            return 0.0;
        }

        let window_size = 10.min(self.pulse_buffer.len());
        let start_idx = self.pulse_buffer.len() - window_size;
        
        let time_span = self.pulse_buffer[self.pulse_buffer.len() - 1] - self.pulse_buffer[start_idx];
        let time_span_s = time_span as f64 / self.timer_frequency as f64;
        
        if time_span_s > 0.0 {
            self.pulse_rate = (window_size - 1) as f32 / time_span_s as f32;
        } else {
            self.pulse_rate = 0.0;
        }

        self.pulse_rate
    }

    /// 获取脉冲计数
    pub fn get_pulse_count(&self) -> u64 {
        self.pulse_count
    }

    /// 重置计数器
    pub fn reset(&mut self) {
        self.pulse_count = 0;
        self.pulse_buffer.clear();
        self.pulse_rate = 0.0;
        self.last_timestamp = 0;
    }

    /// 启用/禁用计数
    pub fn set_counting_enabled(&mut self, enabled: bool) {
        self.counting_enabled = enabled;
    }
}

impl RotaryEncoder {
    /// 创建新的旋转编码器
    pub fn new(steps_per_revolution: u32, timer_frequency: u32) -> Self {
        Self {
            position: 0,
            last_a_state: false,
            last_b_state: false,
            direction: RotationDirection::Stopped,
            steps_per_revolution,
            velocity: 0.0,
            last_update_time: 0,
            velocity_buffer: Deque::new(),
            timer_frequency,
        }
    }

    /// 更新编码器状态
    pub fn update(&mut self, a_state: bool, b_state: bool, timestamp: u32) {
        // 检测A通道边沿
        if a_state != self.last_a_state {
            if a_state { // A通道上升沿
                if b_state {
                    // 顺时针
                    self.position += 1;
                    self.direction = RotationDirection::Clockwise;
                } else {
                    // 逆时针
                    self.position -= 1;
                    self.direction = RotationDirection::CounterClockwise;
                }
                
                // 计算速度
                self.calculate_velocity(timestamp);
            }
        }

        self.last_a_state = a_state;
        self.last_b_state = b_state;
    }

    /// 计算旋转速度
    fn calculate_velocity(&mut self, timestamp: u32) {
        if self.last_update_time != 0 {
            let time_diff = if timestamp >= self.last_update_time {
                timestamp - self.last_update_time
            } else {
                (u32::MAX - self.last_update_time) + timestamp + 1
            };

            let time_diff_s = time_diff as f64 / self.timer_frequency as f64;
            
            if time_diff_s > 0.0 {
                // 步/秒
                let step_velocity = 1.0 / time_diff_s as f32;
                
                // 转/分钟 (RPM)
                let rpm = (step_velocity * 60.0) / self.steps_per_revolution as f32;
                
                self.velocity_buffer.push_back(rpm).ok();
                if self.velocity_buffer.len() > 5 {
                    self.velocity_buffer.pop_front();
                }
                
                // 平均速度
                let sum: f32 = self.velocity_buffer.iter().sum();
                self.velocity = sum / self.velocity_buffer.len() as f32;
            }
        }
        
        self.last_update_time = timestamp;
    }

    /// 获取位置（步数）
    pub fn get_position(&self) -> i32 {
        self.position
    }

    /// 获取角度（度）
    pub fn get_angle_degrees(&self) -> f32 {
        (self.position as f32 * 360.0) / self.steps_per_revolution as f32
    }

    /// 获取旋转速度（RPM）
    pub fn get_velocity_rpm(&self) -> f32 {
        self.velocity
    }

    /// 获取旋转方向
    pub fn get_direction(&self) -> RotationDirection {
        self.direction
    }

    /// 重置位置
    pub fn reset_position(&mut self) {
        self.position = 0;
    }

    /// 设置位置
    pub fn set_position(&mut self, position: i32) {
        self.position = position;
    }
}

impl UltrasonicSensor {
    /// 创建新的超声波传感器
    pub fn new(timer_frequency: u32) -> Self {
        Self {
            trigger_sent: false,
            echo_start_time: None,
            echo_end_time: None,
            distance_cm: 0.0,
            measurement_timeout_us: 30000, // 30ms超时
            sound_speed_cm_per_us: 0.0343, // 343m/s = 0.0343cm/μs
            timer_frequency,
            measurements: Deque::new(),
        }
    }

    /// 开始测量
    pub fn start_measurement(&mut self) {
        self.trigger_sent = true;
        self.echo_start_time = None;
        self.echo_end_time = None;
    }

    /// 处理回声开始
    pub fn echo_start(&mut self, timestamp: u32) {
        if self.trigger_sent {
            self.echo_start_time = Some(timestamp);
        }
    }

    /// 处理回声结束
    pub fn echo_end(&mut self, timestamp: u32) {
        if let Some(start_time) = self.echo_start_time {
            self.echo_end_time = Some(timestamp);
            self.calculate_distance(start_time, timestamp);
            self.trigger_sent = false;
        }
    }

    /// 计算距离
    fn calculate_distance(&mut self, start_time: u32, end_time: u32) {
        let echo_duration = if end_time >= start_time {
            end_time - start_time
        } else {
            (u32::MAX - start_time) + end_time + 1
        };

        let echo_duration_us = (echo_duration as f64 * 1_000_000.0 / self.timer_frequency as f64) as f32;
        
        // 距离 = (声速 * 时间) / 2 (往返)
        self.distance_cm = (self.sound_speed_cm_per_us * echo_duration_us) / 2.0;
        
        // 添加到测量历史
        self.measurements.push_back(self.distance_cm).ok();
        if self.measurements.len() > 10 {
            self.measurements.pop_front();
        }
    }

    /// 获取距离
    pub fn get_distance_cm(&self) -> f32 {
        self.distance_cm
    }

    /// 获取平均距离
    pub fn get_average_distance_cm(&self) -> f32 {
        if self.measurements.is_empty() {
            return 0.0;
        }
        
        let sum: f32 = self.measurements.iter().sum();
        sum / self.measurements.len() as f32
    }

    /// 检查测量是否完成
    pub fn is_measurement_complete(&self) -> bool {
        self.echo_start_time.is_some() && self.echo_end_time.is_some()
    }

    /// 设置声速
    pub fn set_sound_speed(&mut self, speed_cm_per_us: f32) {
        self.sound_speed_cm_per_us = speed_cm_per_us;
    }
}

impl<T> CaptureChannel<T> {
    /// 创建新的捕获通道
    pub fn new(channel_id: u8) -> Self {
        Self {
            channel_id,
            timer_peripheral: PhantomData,
            capture_mode: CaptureMode::RisingEdge,
            prescaler: 1,
            filter: 0,
            polarity: CapturePolarity::Rising,
        }
    }

    /// 设置捕获模式
    pub fn set_capture_mode(&mut self, mode: CaptureMode) {
        self.capture_mode = mode;
    }

    /// 设置预分频器
    pub fn set_prescaler(&mut self, prescaler: u16) {
        self.prescaler = prescaler;
    }

    /// 设置输入滤波器
    pub fn set_filter(&mut self, filter: u8) {
        self.filter = filter;
    }

    /// 获取通道ID
    pub fn get_channel_id(&self) -> u8 {
        self.channel_id
    }
}

impl<T> MultiChannelCapture<T> {
    /// 创建新的多通道捕获管理器
    pub fn new(timer_frequency: u32) -> Self {
        Self {
            channels: Vec::new(),
            capture_buffer: Vec::new(),
            timer_frequency,
            active_channels: 0,
        }
    }

    /// 添加捕获通道
    pub fn add_channel(&mut self, channel: CaptureChannel<T>) -> Result<(), &'static str> {
        if self.channels.len() >= 4 {
            return Err("Maximum channels exceeded");
        }
        
        self.channels.push(channel).map_err(|_| "Failed to add channel")?;
        self.active_channels |= 1 << (self.channels.len() - 1);
        Ok(())
    }

    /// 添加捕获事件
    pub fn add_capture_event(&mut self, event: CaptureEvent) -> Result<(), &'static str> {
        self.capture_buffer.push(event).map_err(|_| "Buffer full")?;
        
        // 保持缓冲区大小
        if self.capture_buffer.len() > 500 {
            self.capture_buffer.remove(0);
        }
        
        Ok(())
    }

    /// 获取指定通道的捕获事件
    pub fn get_channel_events(&self, channel_id: u8) -> Vec<CaptureEvent, 100> {
        let mut events = Vec::new();
        
        for event in &self.capture_buffer {
            if event.channel == channel_id {
                events.push(*event).ok();
            }
        }
        
        events
    }

    /// 清除缓冲区
    pub fn clear_buffer(&mut self) {
        self.capture_buffer.clear();
    }

    /// 获取活动通道掩码
    pub fn get_active_channels(&self) -> u8 {
        self.active_channels
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frequency_meter() {
        let mut meter = FrequencyMeter::new(1_000_000); // 1MHz定时器
        
        // 模拟1kHz信号
        for i in 0..10 {
            let timestamp = i * 1000; // 1ms间隔
            meter.add_capture(timestamp).unwrap();
        }
        
        let frequency = meter.calculate_frequency();
        assert!(frequency > 900.0 && frequency < 1100.0);
    }

    #[test]
    fn test_pulse_counter() {
        let mut counter = PulseCounter::new(1_000_000);
        
        // 添加10个脉冲
        for i in 0..10 {
            let timestamp = i * 10000; // 10ms间隔
            counter.add_pulse(timestamp, EdgeType::Rising).unwrap();
        }
        
        assert_eq!(counter.get_pulse_count(), 10);
        
        let rate = counter.calculate_pulse_rate();
        assert!(rate > 90.0 && rate < 110.0); // 约100Hz
    }

    #[test]
    fn test_rotary_encoder() {
        let mut encoder = RotaryEncoder::new(100, 1_000_000); // 100步/转
        
        // 模拟顺时针旋转
        encoder.update(true, false, 1000);   // A上升，B低
        encoder.update(true, true, 2000);    // B上升
        encoder.update(false, true, 3000);   // A下降
        encoder.update(false, false, 4000);  // B下降
        
        assert_eq!(encoder.get_position(), 1);
        assert_eq!(encoder.get_direction(), RotationDirection::Clockwise);
    }

    #[test]
    fn test_ultrasonic_sensor() {
        let mut sensor = UltrasonicSensor::new(1_000_000);
        
        sensor.start_measurement();
        sensor.echo_start(1000);    // 1ms后开始
        sensor.echo_end(6860);      // 约6ms后结束（对应1m距离）
        
        let distance = sensor.get_distance_cm();
        assert!(distance > 95.0 && distance < 105.0); // 约100cm
    }
}