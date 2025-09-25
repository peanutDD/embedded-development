//! 数据日志管理模块
//! 
//! 功能:
//! - 传感器数据缓冲
//! - 数据统计分析
//! - 日志级别管理
//! - 数据压缩和过滤

use heapless::{Vec, String};
use micromath::F32Ext;
use crate::sensors::SensorData;

// 日志配置
const LOG_BUFFER_SIZE: usize = 100;     // 日志缓冲区大小
const STATS_WINDOW_SIZE: usize = 60;    // 统计窗口大小 (60个样本)
const MAX_LOG_ENTRIES: usize = 50;      // 最大日志条目数

// 日志级别
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}

impl LogLevel {
    pub fn as_str(&self) -> &'static str {
        match self {
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO",
            LogLevel::Warning => "WARN",
            LogLevel::Error => "ERROR",
            LogLevel::Critical => "CRIT",
        }
    }

    pub fn priority(&self) -> u8 {
        match self {
            LogLevel::Debug => 0,
            LogLevel::Info => 1,
            LogLevel::Warning => 2,
            LogLevel::Error => 3,
            LogLevel::Critical => 4,
        }
    }
}

// 日志条目
#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: u32,
    pub level: LogLevel,
    pub message: String<64>,
    pub data: Option<u32>,
}

impl LogEntry {
    pub fn new(level: LogLevel, message: &str) -> Self {
        Self {
            timestamp: get_system_time(),
            level,
            message: String::from(message),
            data: None,
        }
    }

    pub fn with_data(level: LogLevel, message: &str, data: u32) -> Self {
        Self {
            timestamp: get_system_time(),
            level,
            message: String::from(message),
            data: Some(data),
        }
    }
}

// 数据统计信息
#[derive(Debug, Clone, Copy)]
pub struct DataStatistics {
    pub temperature: StatValues,
    pub pressure: StatValues,
    pub accel_magnitude: StatValues,
    pub gyro_magnitude: StatValues,
    pub sample_count: u32,
    pub error_count: u32,
    pub last_update: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct StatValues {
    pub min: f32,
    pub max: f32,
    pub avg: f32,
    pub sum: f32,
    pub count: u32,
}

impl StatValues {
    pub fn new() -> Self {
        Self {
            min: f32::MAX,
            max: f32::MIN,
            avg: 0.0,
            sum: 0.0,
            count: 0,
        }
    }

    pub fn update(&mut self, value: f32) {
        if value < self.min {
            self.min = value;
        }
        if value > self.max {
            self.max = value;
        }
        
        self.sum += value;
        self.count += 1;
        self.avg = self.sum / self.count as f32;
    }

    pub fn reset(&mut self) {
        self.min = f32::MAX;
        self.max = f32::MIN;
        self.avg = 0.0;
        self.sum = 0.0;
        self.count = 0;
    }
}

impl DataStatistics {
    pub fn new() -> Self {
        Self {
            temperature: StatValues::new(),
            pressure: StatValues::new(),
            accel_magnitude: StatValues::new(),
            gyro_magnitude: StatValues::new(),
            sample_count: 0,
            error_count: 0,
            last_update: 0,
        }
    }

    pub fn update(&mut self, data: &SensorData) {
        self.temperature.update(data.temperature);
        self.pressure.update(data.pressure);
        
        // 计算加速度和陀螺仪的幅值
        let accel_mag = (data.accel_x * data.accel_x + 
                        data.accel_y * data.accel_y + 
                        data.accel_z * data.accel_z).sqrt();
        let gyro_mag = (data.gyro_x * data.gyro_x + 
                       data.gyro_y * data.gyro_y + 
                       data.gyro_z * data.gyro_z).sqrt();
        
        self.accel_magnitude.update(accel_mag);
        self.gyro_magnitude.update(gyro_mag);
        
        self.sample_count += 1;
        self.last_update = data.timestamp;
    }

    pub fn reset(&mut self) {
        self.temperature.reset();
        self.pressure.reset();
        self.accel_magnitude.reset();
        self.gyro_magnitude.reset();
        self.sample_count = 0;
        self.error_count = 0;
        self.last_update = 0;
    }
}

// 数据过滤器
#[derive(Debug)]
pub struct DataFilter {
    temp_threshold: f32,
    pressure_threshold: f32,
    accel_threshold: f32,
    gyro_threshold: f32,
    enabled: bool,
}

impl DataFilter {
    pub fn new() -> Self {
        Self {
            temp_threshold: 5.0,    // 温度变化阈值 (°C)
            pressure_threshold: 100.0, // 压力变化阈值 (Pa)
            accel_threshold: 0.5,   // 加速度变化阈值 (g)
            gyro_threshold: 10.0,   // 陀螺仪变化阈值 (°/s)
            enabled: false,
        }
    }

    pub fn enable(&mut self) {
        self.enabled = true;
    }

    pub fn disable(&mut self) {
        self.enabled = false;
    }

    pub fn set_thresholds(&mut self, temp: f32, pressure: f32, accel: f32, gyro: f32) {
        self.temp_threshold = temp;
        self.pressure_threshold = pressure;
        self.accel_threshold = accel;
        self.gyro_threshold = gyro;
    }

    pub fn should_log(&self, current: &SensorData, previous: &SensorData) -> bool {
        if !self.enabled {
            return true;
        }

        let temp_diff = (current.temperature - previous.temperature).abs();
        let pressure_diff = (current.pressure - previous.pressure).abs();
        
        let accel_diff = ((current.accel_x - previous.accel_x).powi(2) +
                         (current.accel_y - previous.accel_y).powi(2) +
                         (current.accel_z - previous.accel_z).powi(2)).sqrt();
        
        let gyro_diff = ((current.gyro_x - previous.gyro_x).powi(2) +
                        (current.gyro_y - previous.gyro_y).powi(2) +
                        (current.gyro_z - previous.gyro_z).powi(2)).sqrt();

        temp_diff > self.temp_threshold ||
        pressure_diff > self.pressure_threshold ||
        accel_diff > self.accel_threshold ||
        gyro_diff > self.gyro_threshold
    }
}

// 数据日志管理器
pub struct DataLogger {
    data_buffer: Vec<SensorData, LOG_BUFFER_SIZE>,
    log_entries: Vec<LogEntry, MAX_LOG_ENTRIES>,
    statistics: DataStatistics,
    filter: DataFilter,
    min_log_level: LogLevel,
    last_data: Option<SensorData>,
    buffer_full_count: u32,
}

impl DataLogger {
    pub fn new() -> Self {
        Self {
            data_buffer: Vec::new(),
            log_entries: Vec::new(),
            statistics: DataStatistics::new(),
            filter: DataFilter::new(),
            min_log_level: LogLevel::Info,
            last_data: None,
            buffer_full_count: 0,
        }
    }

    pub fn set_log_level(&mut self, level: LogLevel) {
        self.min_log_level = level;
    }

    pub fn enable_filter(&mut self) {
        self.filter.enable();
    }

    pub fn disable_filter(&mut self) {
        self.filter.disable();
    }

    pub fn set_filter_thresholds(&mut self, temp: f32, pressure: f32, accel: f32, gyro: f32) {
        self.filter.set_thresholds(temp, pressure, accel, gyro);
    }

    pub fn log_data(&mut self, mut data: SensorData, sample_id: u32) {
        // 设置时间戳
        data.timestamp = sample_id;
        
        // 验证数据
        if !self.validate_data(&data) {
            self.log(LogLevel::Warning, "Invalid sensor data detected");
            self.statistics.error_count += 1;
            return;
        }

        // 应用过滤器
        let should_log = if let Some(ref last) = self.last_data {
            self.filter.should_log(&data, last)
        } else {
            true
        };

        if should_log {
            // 添加到缓冲区
            if self.data_buffer.push(data).is_err() {
                // 缓冲区满，移除最旧的数据
                self.data_buffer.remove(0);
                if self.data_buffer.push(data).is_err() {
                    self.log(LogLevel::Error, "Failed to add data to buffer");
                    return;
                }
                self.buffer_full_count += 1;
            }

            // 更新统计信息
            self.statistics.update(&data);

            // 检查异常值
            self.check_anomalies(&data);
        }

        self.last_data = Some(data);
    }

    fn validate_data(&self, data: &SensorData) -> bool {
        // 检查数据范围
        if data.temperature < -50.0 || data.temperature > 100.0 {
            return false;
        }
        
        if data.pressure < 30000.0 || data.pressure > 120000.0 {
            return false;
        }

        // 检查加速度计数据 (应该在合理范围内)
        let accel_mag = (data.accel_x * data.accel_x + 
                        data.accel_y * data.accel_y + 
                        data.accel_z * data.accel_z).sqrt();
        if accel_mag > 10.0 {  // 超过10g可能是异常
            return false;
        }

        // 检查陀螺仪数据
        let gyro_mag = (data.gyro_x * data.gyro_x + 
                       data.gyro_y * data.gyro_y + 
                       data.gyro_z * data.gyro_z).sqrt();
        if gyro_mag > 1000.0 {  // 超过1000°/s可能是异常
            return false;
        }

        // 验证校验和
        data.verify_checksum()
    }

    fn check_anomalies(&mut self, data: &SensorData) {
        // 检查温度异常
        if data.temperature > 50.0 {
            self.log(LogLevel::Warning, "High temperature detected");
        } else if data.temperature < -10.0 {
            self.log(LogLevel::Warning, "Low temperature detected");
        }

        // 检查压力异常
        if data.pressure > 110000.0 {
            self.log(LogLevel::Warning, "High pressure detected");
        } else if data.pressure < 90000.0 {
            self.log(LogLevel::Warning, "Low pressure detected");
        }

        // 检查加速度异常
        let accel_mag = (data.accel_x * data.accel_x + 
                        data.accel_y * data.accel_y + 
                        data.accel_z * data.accel_z).sqrt();
        if accel_mag > 2.0 {
            self.log(LogLevel::Info, "High acceleration detected");
        }

        // 检查陀螺仪异常
        let gyro_mag = (data.gyro_x * data.gyro_x + 
                       data.gyro_y * data.gyro_y + 
                       data.gyro_z * data.gyro_z).sqrt();
        if gyro_mag > 50.0 {
            self.log(LogLevel::Info, "High rotation rate detected");
        }
    }

    pub fn log(&mut self, level: LogLevel, message: &str) {
        if level.priority() < self.min_log_level.priority() {
            return;
        }

        let entry = LogEntry::new(level, message);
        
        if self.log_entries.push(entry).is_err() {
            // 日志缓冲区满，移除最旧的条目
            self.log_entries.remove(0);
            let _ = self.log_entries.push(LogEntry::new(level, message));
        }
    }

    pub fn log_with_data(&mut self, level: LogLevel, message: &str, data: u32) {
        if level.priority() < self.min_log_level.priority() {
            return;
        }

        let entry = LogEntry::with_data(level, message, data);
        
        if self.log_entries.push(entry).is_err() {
            // 日志缓冲区满，移除最旧的条目
            self.log_entries.remove(0);
            let _ = self.log_entries.push(LogEntry::with_data(level, message, data));
        }
    }

    pub fn get_buffer(&self) -> &[SensorData] {
        &self.data_buffer
    }

    pub fn get_buffer_mut(&mut self) -> &mut Vec<SensorData, LOG_BUFFER_SIZE> {
        &mut self.data_buffer
    }

    pub fn clear_buffer(&mut self) {
        self.data_buffer.clear();
    }

    pub fn get_statistics(&self) -> &DataStatistics {
        &self.statistics
    }

    pub fn reset_statistics(&mut self) {
        self.statistics.reset();
    }

    pub fn get_log_entries(&self) -> &[LogEntry] {
        &self.log_entries
    }

    pub fn clear_logs(&mut self) {
        self.log_entries.clear();
    }

    pub fn get_buffer_usage(&self) -> (usize, usize) {
        (self.data_buffer.len(), LOG_BUFFER_SIZE)
    }

    pub fn get_log_usage(&self) -> (usize, usize) {
        (self.log_entries.len(), MAX_LOG_ENTRIES)
    }

    pub fn is_buffer_full(&self) -> bool {
        self.data_buffer.len() >= LOG_BUFFER_SIZE
    }

    pub fn get_buffer_full_count(&self) -> u32 {
        self.buffer_full_count
    }

    pub fn get_latest_data(&self) -> Option<&SensorData> {
        self.data_buffer.last()
    }

    pub fn get_data_range(&self, start: usize, count: usize) -> &[SensorData] {
        let end = core::cmp::min(start + count, self.data_buffer.len());
        let start = core::cmp::min(start, self.data_buffer.len());
        &self.data_buffer[start..end]
    }

    pub fn compress_old_data(&mut self, keep_recent: usize) {
        if self.data_buffer.len() <= keep_recent {
            return;
        }

        // 保留最近的数据，移除较旧的数据
        let remove_count = self.data_buffer.len() - keep_recent;
        for _ in 0..remove_count {
            self.data_buffer.remove(0);
        }

        self.log(LogLevel::Info, "Compressed old data");
    }

    pub fn export_summary(&self) -> DataSummary {
        DataSummary {
            total_samples: self.statistics.sample_count,
            error_count: self.statistics.error_count,
            buffer_full_count: self.buffer_full_count,
            current_buffer_size: self.data_buffer.len(),
            log_entries_count: self.log_entries.len(),
            temperature_range: (self.statistics.temperature.min, self.statistics.temperature.max),
            pressure_range: (self.statistics.pressure.min, self.statistics.pressure.max),
            last_update: self.statistics.last_update,
        }
    }
}

// 数据摘要结构
#[derive(Debug)]
pub struct DataSummary {
    pub total_samples: u32,
    pub error_count: u32,
    pub buffer_full_count: u32,
    pub current_buffer_size: usize,
    pub log_entries_count: usize,
    pub temperature_range: (f32, f32),
    pub pressure_range: (f32, f32),
    pub last_update: u32,
}

// 获取系统时间的占位函数
fn get_system_time() -> u32 {
    // 这里应该返回实际的系统时间戳
    // 可以使用定时器或RTC来实现
    0
}