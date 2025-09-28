//! 数据收集器模块
//! 
//! 负责从各种传感器收集数据，提供统一的数据收集接口。
//! 支持定时收集、批量收集和按需收集等多种模式。

use core::fmt;
use heapless::{Vec, FnvIndexMap};
use embedded_hal::blocking::delay::DelayMs;
use crate::sensors::{
    SensorData, SensorError, SensorType, SensorTrait,
    temperature::TemperatureSensor,
    humidity::HumiditySensor,
    pressure::PressureSensor,
    light::LightSensor,
};

/// 收集器错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum CollectorError {
    /// 传感器错误
    SensorError(SensorError),
    /// 传感器未找到
    SensorNotFound(SensorType),
    /// 缓冲区满
    BufferFull,
    /// 收集超时
    CollectionTimeout,
    /// 配置错误
    ConfigurationError,
    /// 初始化失败
    InitializationFailed,
}

impl fmt::Display for CollectorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CollectorError::SensorError(e) => write!(f, "Sensor error: {:?}", e),
            CollectorError::SensorNotFound(sensor_type) => write!(f, "Sensor not found: {:?}", sensor_type),
            CollectorError::BufferFull => write!(f, "Collection buffer is full"),
            CollectorError::CollectionTimeout => write!(f, "Collection timeout"),
            CollectorError::ConfigurationError => write!(f, "Configuration error"),
            CollectorError::InitializationFailed => write!(f, "Initialization failed"),
        }
    }
}

/// 收集模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollectionMode {
    /// 单次收集
    Single,
    /// 连续收集
    Continuous,
    /// 定时收集
    Periodic,
    /// 触发收集
    Triggered,
}

/// 收集配置
#[derive(Debug, Clone)]
pub struct CollectionConfig {
    /// 收集模式
    pub mode: CollectionMode,
    /// 收集间隔 (毫秒)
    pub interval_ms: u32,
    /// 超时时间 (毫秒)
    pub timeout_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 启用的传感器类型
    pub enabled_sensors: Vec<SensorType, 8>,
    /// 数据缓冲区大小
    pub buffer_size: usize,
}

impl Default for CollectionConfig {
    fn default() -> Self {
        let mut enabled_sensors = Vec::new();
        let _ = enabled_sensors.push(SensorType::Temperature);
        let _ = enabled_sensors.push(SensorType::Humidity);
        let _ = enabled_sensors.push(SensorType::Pressure);
        let _ = enabled_sensors.push(SensorType::Light);
        
        Self {
            mode: CollectionMode::Periodic,
            interval_ms: 1000,
            timeout_ms: 5000,
            retry_count: 3,
            enabled_sensors,
            buffer_size: 32,
        }
    }
}

/// 收集统计信息
#[derive(Debug, Clone, Default)]
pub struct CollectionStats {
    /// 总收集次数
    pub total_collections: u32,
    /// 成功收集次数
    pub successful_collections: u32,
    /// 失败收集次数
    pub failed_collections: u32,
    /// 超时次数
    pub timeout_count: u32,
    /// 重试次数
    pub retry_count: u32,
    /// 平均收集时间 (微秒)
    pub avg_collection_time_us: u32,
    /// 每种传感器的收集统计
    pub sensor_stats: FnvIndexMap<SensorType, SensorCollectionStats, 8>,
}

/// 单个传感器的收集统计
#[derive(Debug, Clone, Default)]
pub struct SensorCollectionStats {
    /// 收集次数
    pub collections: u32,
    /// 成功次数
    pub successes: u32,
    /// 失败次数
    pub failures: u32,
    /// 平均读取时间 (微秒)
    pub avg_read_time_us: u32,
}

/// 数据收集器
pub struct DataCollector<D> 
where 
    D: DelayMs<u32>,
{
    config: CollectionConfig,
    stats: CollectionStats,
    delay: D,
    data_buffer: Vec<SensorData, 64>,
    last_collection_time: u32,
}

impl<D> DataCollector<D> 
where 
    D: DelayMs<u32>,
{
    /// 创建新的数据收集器
    pub fn new(config: CollectionConfig, delay: D) -> Self {
        Self {
            config,
            stats: CollectionStats::default(),
            delay,
            data_buffer: Vec::new(),
            last_collection_time: 0,
        }
    }
    
    /// 从所有启用的传感器收集数据
    pub fn collect_all_sensors(&mut self) -> Result<Vec<SensorData, 32>, CollectorError> {
        let mut collected_data = Vec::new();
        
        for &sensor_type in &self.config.enabled_sensors {
            match self.collect_from_sensor(sensor_type) {
                Ok(data) => {
                    if collected_data.push(data).is_err() {
                        return Err(CollectorError::BufferFull);
                    }
                    self.update_sensor_stats(sensor_type, true);
                },
                Err(e) => {
                    self.update_sensor_stats(sensor_type, false);
                    // 根据配置决定是否继续收集其他传感器
                    if self.config.retry_count == 0 {
                        return Err(e);
                    }
                }
            }
        }
        
        self.stats.total_collections += 1;
        if !collected_data.is_empty() {
            self.stats.successful_collections += 1;
        } else {
            self.stats.failed_collections += 1;
        }
        
        Ok(collected_data)
    }
    
    /// 从指定传感器收集数据
    pub fn collect_from_sensor(&mut self, sensor_type: SensorType) -> Result<SensorData, CollectorError> {
        let mut retry_count = 0;
        
        loop {
            let result = match sensor_type {
                SensorType::Temperature => self.collect_temperature_data(),
                SensorType::Humidity => self.collect_humidity_data(),
                SensorType::Pressure => self.collect_pressure_data(),
                SensorType::Light => self.collect_light_data(),
                _ => Err(CollectorError::SensorNotFound(sensor_type)),
            };
            
            match result {
                Ok(data) => return Ok(data),
                Err(e) => {
                    retry_count += 1;
                    if retry_count > self.config.retry_count {
                        return Err(e);
                    }
                    self.stats.retry_count += 1;
                    self.delay.delay_ms(100); // 重试前等待
                }
            }
        }
    }
    
    /// 收集温度数据
    fn collect_temperature_data(&mut self) -> Result<SensorData, CollectorError> {
        // 这里应该调用实际的温度传感器
        // 为了示例，我们创建模拟数据
        Ok(SensorData {
            sensor_type: SensorType::Temperature,
            timestamp: self.get_current_timestamp(),
            value: 25.0, // 模拟温度值
            unit: "°C".into(),
            quality: crate::data::DataQuality::High,
        })
    }
    
    /// 收集湿度数据
    fn collect_humidity_data(&mut self) -> Result<SensorData, CollectorError> {
        // 这里应该调用实际的湿度传感器
        Ok(SensorData {
            sensor_type: SensorType::Humidity,
            timestamp: self.get_current_timestamp(),
            value: 60.0, // 模拟湿度值
            unit: "%RH".into(),
            quality: crate::data::DataQuality::High,
        })
    }
    
    /// 收集压力数据
    fn collect_pressure_data(&mut self) -> Result<SensorData, CollectorError> {
        // 这里应该调用实际的压力传感器
        Ok(SensorData {
            sensor_type: SensorType::Pressure,
            timestamp: self.get_current_timestamp(),
            value: 1013.25, // 模拟压力值
            unit: "hPa".into(),
            quality: crate::data::DataQuality::High,
        })
    }
    
    /// 收集光照数据
    fn collect_light_data(&mut self) -> Result<SensorData, CollectorError> {
        // 这里应该调用实际的光照传感器
        Ok(SensorData {
            sensor_type: SensorType::Light,
            timestamp: self.get_current_timestamp(),
            value: 500.0, // 模拟光照值
            unit: "lux".into(),
            quality: crate::data::DataQuality::High,
        })
    }
    
    /// 批量收集数据
    pub fn collect_batch(&mut self, count: usize) -> Result<Vec<SensorData, 32>, CollectorError> {
        let mut batch_data = Vec::new();
        
        for _ in 0..count {
            let data = self.collect_all_sensors()?;
            for item in data {
                if batch_data.push(item).is_err() {
                    return Err(CollectorError::BufferFull);
                }
            }
            
            if self.config.mode == CollectionMode::Periodic {
                self.delay.delay_ms(self.config.interval_ms);
            }
        }
        
        Ok(batch_data)
    }
    
    /// 定时收集数据
    pub fn collect_periodic(&mut self) -> Result<Option<Vec<SensorData, 32>>, CollectorError> {
        let current_time = self.get_current_timestamp();
        
        if current_time - self.last_collection_time >= self.config.interval_ms {
            let data = self.collect_all_sensors()?;
            self.last_collection_time = current_time;
            Ok(Some(data))
        } else {
            Ok(None)
        }
    }
    
    /// 启动连续收集
    pub fn start_continuous_collection(&mut self) -> Result<(), CollectorError> {
        // 这里应该启动一个定时器或中断来定期收集数据
        // 在实际实现中，这可能涉及设置硬件定时器
        Ok(())
    }
    
    /// 停止连续收集
    pub fn stop_continuous_collection(&mut self) -> Result<(), CollectorError> {
        // 停止定时器或中断
        Ok(())
    }
    
    /// 更新传感器统计信息
    fn update_sensor_stats(&mut self, sensor_type: SensorType, success: bool) {
        let stats = self.stats.sensor_stats.entry(sensor_type)
            .or_insert(SensorCollectionStats::default());
        
        stats.collections += 1;
        if success {
            stats.successes += 1;
        } else {
            stats.failures += 1;
        }
    }
    
    /// 获取当前时间戳 (模拟)
    fn get_current_timestamp(&self) -> u32 {
        // 在实际实现中，这应该返回真实的时间戳
        // 这里返回一个模拟值
        0
    }
    
    /// 获取收集统计信息
    pub fn get_stats(&self) -> &CollectionStats {
        &self.stats
    }
    
    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats = CollectionStats::default();
    }
    
    /// 更新配置
    pub fn update_config(&mut self, config: CollectionConfig) {
        self.config = config;
    }
    
    /// 获取配置
    pub fn get_config(&self) -> &CollectionConfig {
        &self.config
    }
    
    /// 获取数据缓冲区
    pub fn get_buffer(&self) -> &Vec<SensorData, 64> {
        &self.data_buffer
    }
    
    /// 清空数据缓冲区
    pub fn clear_buffer(&mut self) {
        self.data_buffer.clear();
    }
    
    /// 添加数据到缓冲区
    pub fn add_to_buffer(&mut self, data: SensorData) -> Result<(), CollectorError> {
        self.data_buffer.push(data)
            .map_err(|_| CollectorError::BufferFull)
    }
    
    /// 检查是否需要收集数据
    pub fn should_collect(&self) -> bool {
        match self.config.mode {
            CollectionMode::Single => false,
            CollectionMode::Continuous => true,
            CollectionMode::Periodic => {
                let current_time = self.get_current_timestamp();
                current_time - self.last_collection_time >= self.config.interval_ms
            },
            CollectionMode::Triggered => false, // 需要外部触发
        }
    }
    
    /// 触发收集 (用于触发模式)
    pub fn trigger_collection(&mut self) -> Result<Vec<SensorData, 32>, CollectorError> {
        if self.config.mode == CollectionMode::Triggered {
            self.collect_all_sensors()
        } else {
            Err(CollectorError::ConfigurationError)
        }
    }
}

/// 收集器工具模块
pub mod collector_utils {
    use super::*;
    
    /// 创建默认的收集器配置
    pub fn create_default_config() -> CollectionConfig {
        CollectionConfig::default()
    }
    
    /// 创建快速收集配置
    pub fn create_fast_config() -> CollectionConfig {
        let mut config = CollectionConfig::default();
        config.interval_ms = 100;
        config.timeout_ms = 1000;
        config
    }
    
    /// 创建节能收集配置
    pub fn create_power_saving_config() -> CollectionConfig {
        let mut config = CollectionConfig::default();
        config.interval_ms = 10000; // 10秒间隔
        config.timeout_ms = 2000;
        config.retry_count = 1;
        config
    }
    
    /// 验证收集配置
    pub fn validate_config(config: &CollectionConfig) -> Result<(), CollectorError> {
        if config.interval_ms == 0 {
            return Err(CollectorError::ConfigurationError);
        }
        
        if config.timeout_ms == 0 {
            return Err(CollectorError::ConfigurationError);
        }
        
        if config.enabled_sensors.is_empty() {
            return Err(CollectorError::ConfigurationError);
        }
        
        if config.buffer_size == 0 {
            return Err(CollectorError::ConfigurationError);
        }
        
        Ok(())
    }
    
    /// 计算收集效率
    pub fn calculate_efficiency(stats: &CollectionStats) -> f32 {
        if stats.total_collections == 0 {
            0.0
        } else {
            stats.successful_collections as f32 / stats.total_collections as f32 * 100.0
        }
    }
    
    /// 获取传感器成功率
    pub fn get_sensor_success_rate(stats: &SensorCollectionStats) -> f32 {
        if stats.collections == 0 {
            0.0
        } else {
            stats.successes as f32 / stats.collections as f32 * 100.0
        }
    }
}