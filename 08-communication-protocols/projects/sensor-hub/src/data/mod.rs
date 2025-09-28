//! 数据处理模块
//! 
//! 该模块负责传感器数据的收集、处理和存储。
//! 提供统一的数据管理接口和高效的数据处理流水线。

use core::fmt;
use heapless::{Vec, FnvIndexMap};
use embedded_hal::blocking::delay::DelayMs;
use crate::sensors::{SensorData, SensorError, SensorType};

pub mod collector;
pub mod processor;
pub mod storage;

pub use collector::*;
pub use processor::*;
pub use storage::*;

/// 数据处理错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum DataError {
    /// 收集器错误
    CollectorError(CollectorError),
    /// 处理器错误
    ProcessorError(ProcessorError),
    /// 存储错误
    StorageError(StorageError),
    /// 缓冲区满
    BufferFull,
    /// 数据无效
    InvalidData,
    /// 配置错误
    ConfigError,
    /// 超时
    Timeout,
}

impl fmt::Display for DataError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DataError::CollectorError(e) => write!(f, "Collector error: {:?}", e),
            DataError::ProcessorError(e) => write!(f, "Processor error: {:?}", e),
            DataError::StorageError(e) => write!(f, "Storage error: {:?}", e),
            DataError::BufferFull => write!(f, "Buffer is full"),
            DataError::InvalidData => write!(f, "Invalid data"),
            DataError::ConfigError => write!(f, "Configuration error"),
            DataError::Timeout => write!(f, "Operation timeout"),
        }
    }
}

/// 数据处理管道
/// 
/// 集成数据收集、处理和存储功能的完整数据处理流水线
pub struct DataPipeline {
    collector: DataCollector,
    processor: DataProcessor,
    storage: DataStorage,
}

impl DataPipeline {
    /// 创建新的数据处理管道
    pub fn new(
        collector_config: CollectionConfig,
        processor_config: ProcessingConfig,
        storage_config: StorageConfig,
    ) -> Self {
        Self {
            collector: DataCollector::new(collector_config),
            processor: DataProcessor::new(processor_config),
            storage: DataStorage::new(storage_config),
        }
    }
    
    /// 处理单个传感器数据
    pub fn process_sensor_data(&mut self, data: SensorData) -> Result<u32, DataError> {
        // 收集数据
        self.collector.collect_data(data.clone())
            .map_err(DataError::CollectorError)?;
        
        // 处理数据
        let processed_data = self.processor.process_data(data)
            .map_err(DataError::ProcessorError)?;
        
        // 存储数据
        let record_id = self.storage.store_data(processed_data.data)
            .map_err(DataError::StorageError)?;
        
        Ok(record_id)
    }
    
    /// 批量处理数据
    pub fn process_batch(&mut self, data_batch: &[SensorData]) -> Result<Vec<u32, 32>, DataError> {
        let mut record_ids = Vec::new();
        
        for data in data_batch {
            match self.process_sensor_data(data.clone()) {
                Ok(record_id) => {
                    if record_ids.push(record_id).is_err() {
                        break; // 缓冲区满
                    }
                },
                Err(e) => {
                    // 记录错误但继续处理其他数据
                    log::warn!("Failed to process sensor data: {:?}", e);
                }
            }
        }
        
        Ok(record_ids)
    }
    
    /// 获取收集器统计信息
    pub fn get_collector_stats(&self) -> CollectionStats {
        self.collector.get_stats()
    }
    
    /// 获取处理器统计信息
    pub fn get_processor_stats(&self) -> ProcessingStats {
        self.processor.get_stats()
    }
    
    /// 获取存储器统计信息
    pub fn get_storage_stats(&mut self) -> StorageStats {
        self.storage.get_stats()
    }
    
    /// 查询存储的数据
    pub fn query_data(&mut self, condition: QueryCondition) -> Result<Vec<DataRecord, 64>, DataError> {
        self.storage.query_data(condition)
            .map_err(DataError::StorageError)
    }
    
    /// 清理过期数据和优化存储
    pub fn cleanup(&mut self) -> Result<(), DataError> {
        // 清理收集器
        self.collector.cleanup()
            .map_err(DataError::CollectorError)?;
        
        // 清理处理器
        self.processor.cleanup()
            .map_err(DataError::ProcessorError)?;
        
        // 压缩存储
        self.storage.compress_storage()
            .map_err(DataError::StorageError)?;
        
        Ok(())
    }
    
    /// 获取管道统计信息
    pub fn get_pipeline_stats(&mut self) -> DataPipelineStats {
        let collector_stats = self.get_collector_stats();
        let processor_stats = self.get_processor_stats();
        let storage_stats = self.get_storage_stats();
        
        let total_processed = collector_stats.total_collected;
        let total_errors = collector_stats.error_count + 
                          processor_stats.error_count + 
                          storage_stats.error_count;
        
        let error_rate = if total_processed > 0 {
            total_errors as f32 / total_processed as f32
        } else {
            0.0
        };
        
        DataPipelineStats {
            collector: collector_stats,
            processor: processor_stats,
            storage: storage_stats,
            total_processed,
            total_errors,
            processing_rate: 0.0, // 需要基于时间计算
            error_rate,
        }
    }
    
    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.collector.reset_stats();
        self.processor.reset_stats();
        // 存储器统计信息通常不重置，因为它们反映持久状态
    }
    
    /// 更新配置
    pub fn update_config(&mut self, config: DataPipelineConfig) -> Result<(), DataError> {
        self.collector.update_config(config.collector)
            .map_err(DataError::CollectorError)?;
        
        self.processor.update_config(config.processor)
            .map_err(DataError::ProcessorError)?;
        
        self.storage.update_config(config.storage)
            .map_err(DataError::StorageError)?;
        
        Ok(())
    }
}

/// 数据处理管道配置
#[derive(Debug, Clone)]
pub struct DataPipelineConfig {
    pub collector: CollectionConfig,
    pub processor: ProcessingConfig,
    pub storage: StorageConfig,
}

impl Default for DataPipelineConfig {
    fn default() -> Self {
        Self {
            collector: CollectionConfig::default(),
            processor: ProcessingConfig::default(),
            storage: StorageConfig::default(),
        }
    }
}

/// 数据处理管道统计信息
#[derive(Debug, Clone, Default)]
pub struct DataPipelineStats {
    pub collector: CollectionStats,
    pub processor: ProcessingStats,
    pub storage: StorageStats,
    pub total_processed: u32,
    pub total_errors: u32,
    pub processing_rate: f32, // 每秒处理的数据条数
    pub error_rate: f32,      // 错误率
}

/// 数据质量等级
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataQuality {
    /// 高质量数据
    High,
    /// 中等质量数据
    Medium,
    /// 低质量数据
    Low,
    /// 无效数据
    Invalid,
}

/// 数据处理配置
#[derive(Debug, Clone)]
pub struct DataConfig {
    /// 收集间隔 (毫秒)
    pub collection_interval_ms: u32,
    /// 缓冲区大小
    pub buffer_size: usize,
    /// 数据质量阈值
    pub quality_threshold: DataQuality,
    /// 启用数据压缩
    pub enable_compression: bool,
    /// 启用数据校验
    pub enable_validation: bool,
    /// 最大重试次数
    pub max_retries: u8,
}

impl Default for DataConfig {
    fn default() -> Self {
        Self {
            collection_interval_ms: 1000,
            buffer_size: 64,
            quality_threshold: DataQuality::Medium,
            enable_compression: false,
            enable_validation: true,
            max_retries: 3,
        }
    }
}

/// 数据统计信息
#[derive(Debug, Clone, Default)]
pub struct DataStatistics {
    /// 总收集次数
    pub total_collections: u32,
    /// 成功收集次数
    pub successful_collections: u32,
    /// 失败收集次数
    pub failed_collections: u32,
    /// 处理的数据点数
    pub processed_points: u32,
    /// 存储的数据点数
    pub stored_points: u32,
    /// 平均处理时间 (微秒)
    pub avg_processing_time_us: u32,
    /// 数据质量分布
    pub quality_distribution: [u32; 4], // [High, Medium, Low, Invalid]
}

impl DataStatistics {
    /// 创建新的统计实例
    pub fn new() -> Self {
        Self::default()
    }
    
    /// 记录收集成功
    pub fn record_collection_success(&mut self) {
        self.total_collections += 1;
        self.successful_collections += 1;
    }
    
    /// 记录收集失败
    pub fn record_collection_failure(&mut self) {
        self.total_collections += 1;
        self.failed_collections += 1;
    }
    
    /// 记录数据处理
    pub fn record_processing(&mut self, processing_time_us: u32, quality: DataQuality) {
        self.processed_points += 1;
        self.avg_processing_time_us = 
            (self.avg_processing_time_us * (self.processed_points - 1) + processing_time_us) 
            / self.processed_points;
        
        match quality {
            DataQuality::High => self.quality_distribution[0] += 1,
            DataQuality::Medium => self.quality_distribution[1] += 1,
            DataQuality::Low => self.quality_distribution[2] += 1,
            DataQuality::Invalid => self.quality_distribution[3] += 1,
        }
    }
    
    /// 记录数据存储
    pub fn record_storage(&mut self) {
        self.stored_points += 1;
    }
    
    /// 获取成功率
    pub fn success_rate(&self) -> f32 {
        if self.total_collections == 0 {
            0.0
        } else {
            self.successful_collections as f32 / self.total_collections as f32
        }
    }
    
    /// 获取数据质量分布百分比
    pub fn quality_percentages(&self) -> [f32; 4] {
        let total = self.quality_distribution.iter().sum::<u32>();
        if total == 0 {
            [0.0; 4]
        } else {
            [
                self.quality_distribution[0] as f32 / total as f32 * 100.0,
                self.quality_distribution[1] as f32 / total as f32 * 100.0,
                self.quality_distribution[2] as f32 / total as f32 * 100.0,
                self.quality_distribution[3] as f32 / total as f32 * 100.0,
            ]
        }
    }
    
    /// 重置统计信息
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// 数据管理器
/// 
/// 协调数据收集、处理和存储的主要组件
pub struct DataManager<D> 
where 
    D: DelayMs<u32>,
{
    collector: DataCollector<D>,
    processor: DataProcessor,
    storage: DataStorage,
    config: DataConfig,
    statistics: DataStatistics,
}

impl<D> DataManager<D> 
where 
    D: DelayMs<u32>,
{
    /// 创建新的数据管理器
    pub fn new(
        collector: DataCollector<D>,
        processor: DataProcessor,
        storage: DataStorage,
        config: DataConfig,
    ) -> Self {
        Self {
            collector,
            processor,
            storage,
            config,
            statistics: DataStatistics::new(),
        }
    }
    
    /// 执行完整的数据处理流水线
    pub fn process_data_pipeline(&mut self) -> Result<usize, DataError> {
        // 收集数据
        let raw_data = self.collector.collect_all_sensors()
            .map_err(DataError::CollectorError)?;
        
        if raw_data.is_empty() {
            return Ok(0);
        }
        
        let mut processed_count = 0;
        
        // 处理每个数据点
        for data in raw_data {
            // 验证数据质量
            let quality = self.processor.assess_data_quality(&data)
                .map_err(DataError::ProcessorError)?;
            
            if quality as u8 >= self.config.quality_threshold as u8 {
                // 处理数据
                let processed_data = self.processor.process_sensor_data(data)
                    .map_err(DataError::ProcessorError)?;
                
                // 存储数据
                self.storage.store_data(processed_data)
                    .map_err(DataError::StorageError)?;
                
                processed_count += 1;
                self.statistics.record_storage();
            }
            
            self.statistics.record_processing(0, quality); // TODO: 实际测量处理时间
        }
        
        self.statistics.record_collection_success();
        Ok(processed_count)
    }
    
    /// 获取统计信息
    pub fn get_statistics(&self) -> &DataStatistics {
        &self.statistics
    }
    
    /// 获取可变统计信息
    pub fn get_statistics_mut(&mut self) -> &mut DataStatistics {
        &mut self.statistics
    }
    
    /// 更新配置
    pub fn update_config(&mut self, config: DataConfig) {
        self.config = config;
    }
    
    /// 获取配置
    pub fn get_config(&self) -> &DataConfig {
        &self.config
    }
    
    /// 获取收集器引用
    pub fn get_collector(&self) -> &DataCollector<D> {
        &self.collector
    }
    
    /// 获取可变收集器引用
    pub fn get_collector_mut(&mut self) -> &mut DataCollector<D> {
        &mut self.collector
    }
    
    /// 获取处理器引用
    pub fn get_processor(&self) -> &DataProcessor {
        &self.processor
    }
    
    /// 获取可变处理器引用
    pub fn get_processor_mut(&mut self) -> &mut DataProcessor {
        &mut self.processor
    }
    
    /// 获取存储器引用
    pub fn get_storage(&self) -> &DataStorage {
        &self.storage
    }
    
    /// 获取可变存储器引用
    pub fn get_storage_mut(&mut self) -> &mut DataStorage {
        &mut self.storage
    }
}

/// 数据工具模块
pub mod data_utils {
    use super::*;
    
    /// 计算数据的移动平均值
    pub fn calculate_moving_average(values: &[f32], window_size: usize) -> Vec<f32, 32> {
        let mut result = Vec::new();
        
        if values.len() < window_size {
            return result;
        }
        
        for i in window_size - 1..values.len() {
            let sum: f32 = values[i - window_size + 1..=i].iter().sum();
            let avg = sum / window_size as f32;
            let _ = result.push(avg);
        }
        
        result
    }
    
    /// 检测数据异常值
    pub fn detect_outliers(values: &[f32], threshold: f32) -> Vec<usize, 16> {
        let mut outliers = Vec::new();
        
        if values.len() < 3 {
            return outliers;
        }
        
        let mean = values.iter().sum::<f32>() / values.len() as f32;
        let variance = values.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>() / values.len() as f32;
        let std_dev = variance.sqrt();
        
        for (i, &value) in values.iter().enumerate() {
            if (value - mean).abs() > threshold * std_dev {
                let _ = outliers.push(i);
            }
        }
        
        outliers
    }
    
    /// 数据插值
    pub fn linear_interpolate(x1: f32, y1: f32, x2: f32, y2: f32, x: f32) -> f32 {
        if (x2 - x1).abs() < f32::EPSILON {
            return y1;
        }
        y1 + (y2 - y1) * (x - x1) / (x2 - x1)
    }
    
    /// 数据归一化 (0-1)
    pub fn normalize_data(values: &mut [f32]) {
        if values.is_empty() {
            return;
        }
        
        let min_val = values.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        let max_val = values.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        
        if (max_val - min_val).abs() < f32::EPSILON {
            return;
        }
        
        for value in values.iter_mut() {
            *value = (*value - min_val) / (max_val - min_val);
        }
    }
    
    /// 计算数据的基本统计信息
    pub fn calculate_basic_stats(values: &[f32]) -> Option<(f32, f32, f32, f32)> {
        if values.is_empty() {
            return None;
        }
        
        let sum: f32 = values.iter().sum();
        let mean = sum / values.len() as f32;
        
        let min_val = values.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        let max_val = values.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        
        let variance = values.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>() / values.len() as f32;
        let std_dev = variance.sqrt();
        
        Some((mean, std_dev, min_val, max_val))
    }
}