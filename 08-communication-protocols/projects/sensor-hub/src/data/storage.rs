//! 数据存储模块
//! 
//! 负责传感器数据的持久化存储，支持多种存储后端和数据格式。
//! 提供数据压缩、索引、查询和备份功能。

use core::fmt;
use heapless::{Vec, FnvIndexMap, String};
use crate::sensors::{SensorData, SensorType};

/// 存储错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum StorageError {
    /// 存储空间不足
    InsufficientSpace,
    /// 写入失败
    WriteFailed,
    /// 读取失败
    ReadFailed,
    /// 数据损坏
    DataCorrupted,
    /// 索引错误
    IndexError,
    /// 配置错误
    ConfigurationError,
    /// 存储后端不可用
    BackendUnavailable,
    /// 数据格式错误
    FormatError,
    /// 压缩失败
    CompressionFailed,
    /// 解压缩失败
    DecompressionFailed,
}

impl fmt::Display for StorageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StorageError::InsufficientSpace => write!(f, "Insufficient storage space"),
            StorageError::WriteFailed => write!(f, "Write operation failed"),
            StorageError::ReadFailed => write!(f, "Read operation failed"),
            StorageError::DataCorrupted => write!(f, "Data corruption detected"),
            StorageError::IndexError => write!(f, "Index operation error"),
            StorageError::ConfigurationError => write!(f, "Storage configuration error"),
            StorageError::BackendUnavailable => write!(f, "Storage backend unavailable"),
            StorageError::FormatError => write!(f, "Data format error"),
            StorageError::CompressionFailed => write!(f, "Data compression failed"),
            StorageError::DecompressionFailed => write!(f, "Data decompression failed"),
        }
    }
}

/// 存储后端类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StorageBackend {
    /// 内存存储
    Memory,
    /// Flash存储
    Flash,
    /// EEPROM存储
    Eeprom,
    /// SD卡存储
    SdCard,
    /// 外部SPI Flash
    ExternalFlash,
}

/// 数据格式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataFormat {
    /// 原始二进制
    Binary,
    /// JSON格式
    Json,
    /// CSV格式
    Csv,
    /// 压缩二进制
    CompressedBinary,
    /// 自定义格式
    Custom,
}

/// 压缩算法
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompressionAlgorithm {
    /// 无压缩
    None,
    /// RLE压缩
    Rle,
    /// LZ77压缩
    Lz77,
    /// 简单差分压缩
    Delta,
}

/// 存储配置
#[derive(Debug, Clone)]
pub struct StorageConfig {
    /// 存储后端
    pub backend: StorageBackend,
    /// 数据格式
    pub format: DataFormat,
    /// 压缩算法
    pub compression: CompressionAlgorithm,
    /// 最大存储大小 (字节)
    pub max_size: usize,
    /// 缓冲区大小
    pub buffer_size: usize,
    /// 自动压缩阈值
    pub auto_compress_threshold: usize,
    /// 启用索引
    pub indexing_enabled: bool,
    /// 启用校验和
    pub checksum_enabled: bool,
    /// 数据保留时间 (秒)
    pub retention_time: u32,
    /// 循环存储
    pub circular_storage: bool,
}

/// 存储统计信息
#[derive(Debug, Clone, Default)]
pub struct StorageStats {
    /// 总写入次数
    pub total_writes: u32,
    /// 总读取次数
    pub total_reads: u32,
    /// 写入字节数
    pub bytes_written: u32,
    /// 读取字节数
    pub bytes_read: u32,
    /// 存储的记录数
    pub record_count: u32,
    /// 压缩比率
    pub compression_ratio: f32,
    /// 错误计数
    pub error_count: u32,
    /// 可用空间 (字节)
    pub available_space: u32,
    /// 已用空间 (字节)
    pub used_space: u32,
    /// 碎片化程度 (0.0-1.0)
    pub fragmentation: f32,
}

/// 数据记录
#[derive(Debug, Clone)]
pub struct DataRecord {
    /// 记录ID
    pub id: u32,
    /// 时间戳
    pub timestamp: u64,
    /// 传感器数据
    pub data: SensorData,
    /// 校验和
    pub checksum: u32,
    /// 压缩标志
    pub compressed: bool,
    /// 数据大小
    pub size: u16,
}

/// 查询条件
#[derive(Debug, Clone)]
pub struct QueryCondition {
    /// 传感器类型过滤
    pub sensor_type: Option<SensorType>,
    /// 开始时间
    pub start_time: Option<u64>,
    /// 结束时间
    pub end_time: Option<u64>,
    /// 值范围过滤
    pub value_range: Option<(f32, f32)>,
    /// 最大记录数
    pub limit: Option<usize>,
}

/// 存储索引
#[derive(Debug, Clone)]
struct StorageIndex {
    /// 传感器类型索引
    sensor_type_index: FnvIndexMap<SensorType, Vec<u32, 64>, 8>,
    /// 时间索引 (时间戳 -> 记录ID)
    time_index: Vec<(u64, u32), 128>,
    /// 值索引 (用于范围查询)
    value_index: FnvIndexMap<SensorType, Vec<(f32, u32), 32>, 8>,
}

/// 数据存储器
pub struct DataStorage {
    config: StorageConfig,
    stats: StorageStats,
    records: Vec<DataRecord, 256>,
    index: StorageIndex,
    write_buffer: Vec<u8, 512>,
    next_record_id: u32,
}

impl Default for StorageConfig {
    fn default() -> Self {
        Self {
            backend: StorageBackend::Memory,
            format: DataFormat::Binary,
            compression: CompressionAlgorithm::None,
            max_size: 8192, // 8KB
            buffer_size: 512,
            auto_compress_threshold: 4096,
            indexing_enabled: true,
            checksum_enabled: true,
            retention_time: 86400, // 24小时
            circular_storage: true,
        }
    }
}

impl Default for StorageIndex {
    fn default() -> Self {
        Self {
            sensor_type_index: FnvIndexMap::new(),
            time_index: Vec::new(),
            value_index: FnvIndexMap::new(),
        }
    }
}

impl DataStorage {
    /// 创建新的数据存储器
    pub fn new(config: StorageConfig) -> Self {
        Self {
            config,
            stats: StorageStats::default(),
            records: Vec::new(),
            index: StorageIndex::default(),
            write_buffer: Vec::new(),
            next_record_id: 1,
        }
    }
    
    /// 存储单个数据记录
    pub fn store_data(&mut self, data: SensorData) -> Result<u32, StorageError> {
        // 检查存储空间
        if self.is_storage_full() {
            if self.config.circular_storage {
                self.cleanup_old_records()?;
            } else {
                return Err(StorageError::InsufficientSpace);
            }
        }
        
        // 创建数据记录
        let record = self.create_record(data)?;
        let record_id = record.id;
        
        // 存储记录
        if self.records.push(record.clone()).is_err() {
            return Err(StorageError::InsufficientSpace);
        }
        
        // 更新索引
        if self.config.indexing_enabled {
            self.update_index(&record)?;
        }
        
        // 更新统计信息
        self.stats.total_writes += 1;
        self.stats.record_count += 1;
        self.stats.bytes_written += record.size as u32;
        self.stats.used_space += record.size as u32;
        
        // 检查是否需要自动压缩
        if self.stats.used_space >= self.config.auto_compress_threshold as u32 {
            let _ = self.compress_old_records();
        }
        
        Ok(record_id)
    }
    
    /// 批量存储数据
    pub fn store_batch(&mut self, data_batch: &[SensorData]) -> Result<Vec<u32, 32>, StorageError> {
        let mut record_ids = Vec::new();
        
        for data in data_batch {
            let record_id = self.store_data(data.clone())?;
            if record_ids.push(record_id).is_err() {
                break; // 缓冲区满
            }
        }
        
        Ok(record_ids)
    }
    
    /// 读取数据记录
    pub fn read_data(&mut self, record_id: u32) -> Result<DataRecord, StorageError> {
        if let Some(record) = self.records.iter().find(|r| r.id == record_id) {
            self.stats.total_reads += 1;
            self.stats.bytes_read += record.size as u32;
            
            // 验证校验和
            if self.config.checksum_enabled {
                let calculated_checksum = self.calculate_checksum(&record.data);
                if calculated_checksum != record.checksum {
                    self.stats.error_count += 1;
                    return Err(StorageError::DataCorrupted);
                }
            }
            
            Ok(record.clone())
        } else {
            Err(StorageError::ReadFailed)
        }
    }
    
    /// 查询数据记录
    pub fn query_data(&mut self, condition: QueryCondition) -> Result<Vec<DataRecord, 64>, StorageError> {
        let mut results = Vec::new();
        let mut count = 0;
        
        for record in &self.records {
            // 检查传感器类型过滤
            if let Some(sensor_type) = condition.sensor_type {
                if record.data.sensor_type != sensor_type {
                    continue;
                }
            }
            
            // 检查时间范围过滤
            if let Some(start_time) = condition.start_time {
                if record.timestamp < start_time {
                    continue;
                }
            }
            
            if let Some(end_time) = condition.end_time {
                if record.timestamp > end_time {
                    continue;
                }
            }
            
            // 检查值范围过滤
            if let Some((min_val, max_val)) = condition.value_range {
                if record.data.value < min_val || record.data.value > max_val {
                    continue;
                }
            }
            
            // 添加到结果
            if results.push(record.clone()).is_err() {
                break; // 结果缓冲区满
            }
            
            count += 1;
            
            // 检查限制
            if let Some(limit) = condition.limit {
                if count >= limit {
                    break;
                }
            }
        }
        
        self.stats.total_reads += count as u32;
        Ok(results)
    }
    
    /// 删除数据记录
    pub fn delete_data(&mut self, record_id: u32) -> Result<(), StorageError> {
        if let Some(pos) = self.records.iter().position(|r| r.id == record_id) {
            let record = self.records.remove(pos);
            
            // 更新索引
            if self.config.indexing_enabled {
                self.remove_from_index(&record)?;
            }
            
            // 更新统计信息
            self.stats.record_count -= 1;
            self.stats.used_space -= record.size as u32;
            
            Ok(())
        } else {
            Err(StorageError::ReadFailed)
        }
    }
    
    /// 清空所有数据
    pub fn clear_all(&mut self) -> Result<(), StorageError> {
        self.records.clear();
        self.index = StorageIndex::default();
        self.stats.record_count = 0;
        self.stats.used_space = 0;
        self.next_record_id = 1;
        Ok(())
    }
    
    /// 压缩存储
    pub fn compress_storage(&mut self) -> Result<(), StorageError> {
        if self.config.compression == CompressionAlgorithm::None {
            return Ok(());
        }
        
        let mut compressed_count = 0;
        
        for record in &mut self.records {
            if !record.compressed {
                match self.compress_record_data(record) {
                    Ok(_) => {
                        record.compressed = true;
                        compressed_count += 1;
                    },
                    Err(_) => {
                        self.stats.error_count += 1;
                    }
                }
            }
        }
        
        // 更新压缩比率
        if compressed_count > 0 {
            self.update_compression_ratio();
        }
        
        Ok(())
    }
    
    /// 创建数据记录
    fn create_record(&mut self, data: SensorData) -> Result<DataRecord, StorageError> {
        let record_id = self.next_record_id;
        self.next_record_id += 1;
        
        let checksum = if self.config.checksum_enabled {
            self.calculate_checksum(&data)
        } else {
            0
        };
        
        let size = self.estimate_record_size(&data);
        
        Ok(DataRecord {
            id: record_id,
            timestamp: self.get_current_timestamp(),
            data,
            checksum,
            compressed: false,
            size,
        })
    }
    
    /// 更新索引
    fn update_index(&mut self, record: &DataRecord) -> Result<(), StorageError> {
        // 更新传感器类型索引
        let sensor_index = self.index.sensor_type_index
            .entry(record.data.sensor_type)
            .or_insert_with(Vec::new);
        
        if sensor_index.push(record.id).is_err() {
            return Err(StorageError::IndexError);
        }
        
        // 更新时间索引
        if self.index.time_index.push((record.timestamp, record.id)).is_err() {
            // 移除最老的条目
            self.index.time_index.remove(0);
            let _ = self.index.time_index.push((record.timestamp, record.id));
        }
        
        // 保持时间索引排序
        self.index.time_index.sort_by_key(|&(timestamp, _)| timestamp);
        
        // 更新值索引
        let value_index = self.index.value_index
            .entry(record.data.sensor_type)
            .or_insert_with(Vec::new);
        
        if value_index.push((record.data.value, record.id)).is_err() {
            // 移除最老的条目
            value_index.remove(0);
            let _ = value_index.push((record.data.value, record.id));
        }
        
        Ok(())
    }
    
    /// 从索引中移除记录
    fn remove_from_index(&mut self, record: &DataRecord) -> Result<(), StorageError> {
        // 从传感器类型索引中移除
        if let Some(sensor_index) = self.index.sensor_type_index.get_mut(&record.data.sensor_type) {
            sensor_index.retain(|&id| id != record.id);
        }
        
        // 从时间索引中移除
        self.index.time_index.retain(|&(_, id)| id != record.id);
        
        // 从值索引中移除
        if let Some(value_index) = self.index.value_index.get_mut(&record.data.sensor_type) {
            value_index.retain(|&(_, id)| id != record.id);
        }
        
        Ok(())
    }
    
    /// 计算校验和
    fn calculate_checksum(&self, data: &SensorData) -> u32 {
        // 简单的CRC32校验和实现
        let mut crc = 0xFFFFFFFFu32;
        let bytes = data.value.to_le_bytes();
        
        for &byte in &bytes {
            crc ^= byte as u32;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
        }
        
        !crc
    }
    
    /// 估算记录大小
    fn estimate_record_size(&self, _data: &SensorData) -> u16 {
        match self.config.format {
            DataFormat::Binary => 32, // 基本二进制格式
            DataFormat::Json => 64,   // JSON格式较大
            DataFormat::Csv => 48,    // CSV格式中等
            DataFormat::CompressedBinary => 24, // 压缩后较小
            DataFormat::Custom => 40, // 自定义格式
        }
    }
    
    /// 获取当前时间戳
    fn get_current_timestamp(&self) -> u64 {
        // 在实际实现中，这里应该返回真实的时间戳
        // 这里使用简单的递增计数器作为示例
        self.next_record_id as u64
    }
    
    /// 检查存储是否已满
    fn is_storage_full(&self) -> bool {
        self.stats.used_space >= self.config.max_size as u32
    }
    
    /// 清理旧记录
    fn cleanup_old_records(&mut self) -> Result<(), StorageError> {
        let current_time = self.get_current_timestamp();
        let retention_threshold = current_time.saturating_sub(self.config.retention_time as u64);
        
        let mut removed_count = 0;
        
        // 移除过期记录
        self.records.retain(|record| {
            if record.timestamp < retention_threshold {
                removed_count += 1;
                false
            } else {
                true
            }
        });
        
        // 如果还是空间不足，移除最老的记录
        while self.is_storage_full() && !self.records.is_empty() {
            self.records.remove(0);
            removed_count += 1;
        }
        
        // 重建索引
        if removed_count > 0 {
            self.rebuild_index()?;
            self.stats.record_count = self.records.len() as u32;
            self.recalculate_used_space();
        }
        
        Ok(())
    }
    
    /// 压缩旧记录
    fn compress_old_records(&mut self) -> Result<(), StorageError> {
        let current_time = self.get_current_timestamp();
        let compress_threshold = current_time.saturating_sub(3600); // 1小时前的数据
        
        for record in &mut self.records {
            if record.timestamp < compress_threshold && !record.compressed {
                let _ = self.compress_record_data(record);
            }
        }
        
        self.update_compression_ratio();
        Ok(())
    }
    
    /// 压缩记录数据
    fn compress_record_data(&self, record: &mut DataRecord) -> Result<(), StorageError> {
        match self.config.compression {
            CompressionAlgorithm::None => Ok(()),
            CompressionAlgorithm::Rle => {
                // RLE压缩实现 (简化版)
                record.size = (record.size as f32 * 0.8) as u16; // 假设压缩率
                Ok(())
            },
            CompressionAlgorithm::Delta => {
                // 差分压缩实现 (简化版)
                record.size = (record.size as f32 * 0.6) as u16; // 假设压缩率
                Ok(())
            },
            _ => Err(StorageError::CompressionFailed),
        }
    }
    
    /// 重建索引
    fn rebuild_index(&mut self) -> Result<(), StorageError> {
        self.index = StorageIndex::default();
        
        for record in &self.records {
            self.update_index(record)?;
        }
        
        Ok(())
    }
    
    /// 重新计算已用空间
    fn recalculate_used_space(&mut self) {
        self.stats.used_space = self.records.iter()
            .map(|r| r.size as u32)
            .sum();
    }
    
    /// 更新压缩比率
    fn update_compression_ratio(&mut self) {
        let compressed_count = self.records.iter()
            .filter(|r| r.compressed)
            .count();
        
        if self.records.len() > 0 {
            self.stats.compression_ratio = compressed_count as f32 / self.records.len() as f32;
        }
    }
    
    /// 获取统计信息
    pub fn get_stats(&mut self) -> StorageStats {
        // 更新可用空间
        self.stats.available_space = (self.config.max_size as u32)
            .saturating_sub(self.stats.used_space);
        
        // 计算碎片化程度
        self.stats.fragmentation = self.calculate_fragmentation();
        
        self.stats.clone()
    }
    
    /// 计算碎片化程度
    fn calculate_fragmentation(&self) -> f32 {
        if self.records.is_empty() {
            return 0.0;
        }
        
        // 简化的碎片化计算
        let avg_record_size = self.stats.used_space as f32 / self.records.len() as f32;
        let size_variance = self.records.iter()
            .map(|r| {
                let diff = r.size as f32 - avg_record_size;
                diff * diff
            })
            .sum::<f32>() / self.records.len() as f32;
        
        (size_variance.sqrt() / avg_record_size).min(1.0)
    }
    
    /// 更新配置
    pub fn update_config(&mut self, config: StorageConfig) -> Result<(), StorageError> {
        // 验证配置
        if config.max_size == 0 || config.buffer_size == 0 {
            return Err(StorageError::ConfigurationError);
        }
        
        self.config = config;
        Ok(())
    }
    
    /// 获取配置
    pub fn get_config(&self) -> &StorageConfig {
        &self.config
    }
    
    /// 导出数据
    pub fn export_data(&self, format: DataFormat) -> Result<Vec<u8, 1024>, StorageError> {
        let mut export_data = Vec::new();
        
        match format {
            DataFormat::Json => {
                // JSON导出实现 (简化版)
                let json_str = b"{\"records\":[]}";
                for &byte in json_str {
                    if export_data.push(byte).is_err() {
                        return Err(StorageError::InsufficientSpace);
                    }
                }
            },
            DataFormat::Csv => {
                // CSV导出实现 (简化版)
                let csv_header = b"timestamp,sensor_type,value\n";
                for &byte in csv_header {
                    if export_data.push(byte).is_err() {
                        return Err(StorageError::InsufficientSpace);
                    }
                }
            },
            _ => return Err(StorageError::FormatError),
        }
        
        Ok(export_data)
    }
    
    /// 导入数据
    pub fn import_data(&mut self, data: &[u8], format: DataFormat) -> Result<u32, StorageError> {
        match format {
            DataFormat::Json | DataFormat::Csv => {
                // 导入实现 (简化版)
                // 在实际实现中，这里应该解析数据并创建记录
                Ok(0)
            },
            _ => Err(StorageError::FormatError),
        }
    }
}

/// 存储工具模块
pub mod storage_utils {
    use super::*;
    
    /// 创建内存存储配置
    pub fn create_memory_config(max_size: usize) -> StorageConfig {
        StorageConfig {
            backend: StorageBackend::Memory,
            max_size,
            ..Default::default()
        }
    }
    
    /// 创建Flash存储配置
    pub fn create_flash_config(max_size: usize, compression: CompressionAlgorithm) -> StorageConfig {
        StorageConfig {
            backend: StorageBackend::Flash,
            compression,
            max_size,
            ..Default::default()
        }
    }
    
    /// 验证存储配置
    pub fn validate_storage_config(config: &StorageConfig) -> Result<(), StorageError> {
        if config.max_size == 0 {
            return Err(StorageError::ConfigurationError);
        }
        
        if config.buffer_size == 0 || config.buffer_size > config.max_size {
            return Err(StorageError::ConfigurationError);
        }
        
        if config.auto_compress_threshold > config.max_size {
            return Err(StorageError::ConfigurationError);
        }
        
        Ok(())
    }
    
    /// 计算存储效率
    pub fn calculate_storage_efficiency(stats: &StorageStats, config: &StorageConfig) -> f32 {
        if config.max_size == 0 {
            return 0.0;
        }
        
        let utilization = stats.used_space as f32 / config.max_size as f32;
        let compression_benefit = stats.compression_ratio * 0.5; // 压缩带来的效率提升
        let fragmentation_penalty = stats.fragmentation * 0.3; // 碎片化的效率损失
        
        (utilization + compression_benefit - fragmentation_penalty).max(0.0).min(1.0)
    }
    
    /// 估算存储容量需求
    pub fn estimate_storage_capacity(
        records_per_day: u32,
        avg_record_size: u16,
        retention_days: u32,
        compression_ratio: f32
    ) -> usize {
        let total_records = records_per_day * retention_days;
        let raw_size = total_records * avg_record_size as u32;
        let compressed_size = raw_size as f32 * (1.0 - compression_ratio);
        
        (compressed_size * 1.2) as usize // 添加20%的安全边际
    }
    
    /// 创建查询条件构建器
    pub fn query_builder() -> QueryBuilder {
        QueryBuilder::new()
    }
}

/// 查询条件构建器
pub struct QueryBuilder {
    condition: QueryCondition,
}

impl QueryBuilder {
    pub fn new() -> Self {
        Self {
            condition: QueryCondition {
                sensor_type: None,
                start_time: None,
                end_time: None,
                value_range: None,
                limit: None,
            },
        }
    }
    
    pub fn sensor_type(mut self, sensor_type: SensorType) -> Self {
        self.condition.sensor_type = Some(sensor_type);
        self
    }
    
    pub fn time_range(mut self, start: u64, end: u64) -> Self {
        self.condition.start_time = Some(start);
        self.condition.end_time = Some(end);
        self
    }
    
    pub fn value_range(mut self, min: f32, max: f32) -> Self {
        self.condition.value_range = Some((min, max));
        self
    }
    
    pub fn limit(mut self, limit: usize) -> Self {
        self.condition.limit = Some(limit);
        self
    }
    
    pub fn build(self) -> QueryCondition {
        self.condition
    }
}