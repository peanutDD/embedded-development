//! # 缓冲区管理模块
//!
//! 提供高效的异步UART缓冲区管理功能，包括环形缓冲区、DMA缓冲区等。

pub mod ring_buffer;
pub mod dma_buffer;
pub mod stream_buffer;

pub use ring_buffer::{RingBuffer, RingBufferReader, RingBufferWriter};
pub use dma_buffer::{DmaBuffer, DmaBufferConfig};
pub use stream_buffer::{StreamBuffer, StreamBufferConfig};

use crate::error::{BufferError, Result};
use core::fmt;

/// 缓冲区特征
pub trait Buffer {
    /// 获取缓冲区容量
    fn capacity(&self) -> usize;
    
    /// 获取已使用的字节数
    fn len(&self) -> usize;
    
    /// 检查缓冲区是否为空
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    
    /// 检查缓冲区是否已满
    fn is_full(&self) -> bool {
        self.len() == self.capacity()
    }
    
    /// 获取可用空间
    fn available(&self) -> usize {
        self.capacity() - self.len()
    }
    
    /// 清空缓冲区
    fn clear(&mut self);
    
    /// 获取缓冲区统计信息
    fn stats(&self) -> BufferStats;
}

/// 可读缓冲区特征
pub trait ReadableBuffer: Buffer {
    /// 读取数据到切片
    fn read(&mut self, buf: &mut [u8]) -> Result<usize>;
    
    /// 读取单个字节
    fn read_byte(&mut self) -> Result<u8>;
    
    /// 窥视数据（不移除）
    fn peek(&self, buf: &mut [u8]) -> Result<usize>;
    
    /// 窥视单个字节
    fn peek_byte(&self) -> Result<u8>;
    
    /// 跳过指定数量的字节
    fn skip(&mut self, count: usize) -> Result<usize>;
    
    /// 读取直到指定分隔符
    fn read_until(&mut self, delimiter: u8, buf: &mut [u8]) -> Result<usize>;
    
    /// 读取一行（直到\n）
    fn read_line(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.read_until(b'\n', buf)
    }
}

/// 可写缓冲区特征
pub trait WritableBuffer: Buffer {
    /// 写入数据从切片
    fn write(&mut self, buf: &[u8]) -> Result<usize>;
    
    /// 写入单个字节
    fn write_byte(&mut self, byte: u8) -> Result<()>;
    
    /// 写入所有数据
    fn write_all(&mut self, buf: &[u8]) -> Result<()> {
        let mut written = 0;
        while written < buf.len() {
            let n = self.write(&buf[written..])?;
            if n == 0 {
                return Err(BufferError::Full.into());
            }
            written += n;
        }
        Ok(())
    }
    
    /// 刷新缓冲区
    fn flush(&mut self) -> Result<()>;
    
    /// 预留空间
    fn reserve(&mut self, additional: usize) -> Result<()>;
}

/// 缓冲区统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct BufferStats {
    /// 总读取字节数
    pub bytes_read: u64,
    /// 总写入字节数
    pub bytes_written: u64,
    /// 读取操作次数
    pub read_operations: u64,
    /// 写入操作次数
    pub write_operations: u64,
    /// 溢出次数
    pub overflows: u64,
    /// 欠载次数
    pub underflows: u64,
    /// 最大使用量
    pub peak_usage: usize,
    /// 当前使用量
    pub current_usage: usize,
}

impl BufferStats {
    /// 创建新的统计信息
    pub fn new() -> Self {
        Self::default()
    }
    
    /// 记录读取操作
    pub fn record_read(&mut self, bytes: usize) {
        self.bytes_read += bytes as u64;
        self.read_operations += 1;
    }
    
    /// 记录写入操作
    pub fn record_write(&mut self, bytes: usize) {
        self.bytes_written += bytes as u64;
        self.write_operations += 1;
    }
    
    /// 记录溢出
    pub fn record_overflow(&mut self) {
        self.overflows += 1;
    }
    
    /// 记录欠载
    pub fn record_underflow(&mut self) {
        self.underflows += 1;
    }
    
    /// 更新使用量
    pub fn update_usage(&mut self, current: usize) {
        self.current_usage = current;
        if current > self.peak_usage {
            self.peak_usage = current;
        }
    }
    
    /// 获取读取吞吐量（字节/操作）
    pub fn read_throughput(&self) -> f64 {
        if self.read_operations > 0 {
            self.bytes_read as f64 / self.read_operations as f64
        } else {
            0.0
        }
    }
    
    /// 获取写入吞吐量（字节/操作）
    pub fn write_throughput(&self) -> f64 {
        if self.write_operations > 0 {
            self.bytes_written as f64 / self.write_operations as f64
        } else {
            0.0
        }
    }
    
    /// 获取使用率
    pub fn usage_ratio(&self, capacity: usize) -> f64 {
        if capacity > 0 {
            self.current_usage as f64 / capacity as f64
        } else {
            0.0
        }
    }
    
    /// 获取峰值使用率
    pub fn peak_usage_ratio(&self, capacity: usize) -> f64 {
        if capacity > 0 {
            self.peak_usage as f64 / capacity as f64
        } else {
            0.0
        }
    }
    
    /// 重置统计信息
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

impl fmt::Display for BufferStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Buffer Statistics:")?;
        writeln!(f, "  Bytes read: {}", self.bytes_read)?;
        writeln!(f, "  Bytes written: {}", self.bytes_written)?;
        writeln!(f, "  Read operations: {}", self.read_operations)?;
        writeln!(f, "  Write operations: {}", self.write_operations)?;
        writeln!(f, "  Overflows: {}", self.overflows)?;
        writeln!(f, "  Underflows: {}", self.underflows)?;
        writeln!(f, "  Peak usage: {} bytes", self.peak_usage)?;
        writeln!(f, "  Current usage: {} bytes", self.current_usage)?;
        writeln!(f, "  Read throughput: {:.2} bytes/op", self.read_throughput())?;
        writeln!(f, "  Write throughput: {:.2} bytes/op", self.write_throughput())?;
        Ok(())
    }
}

/// 缓冲区配置
#[derive(Debug, Clone)]
pub struct BufferConfig {
    /// 缓冲区大小
    pub size: usize,
    /// 是否允许溢出
    pub allow_overflow: bool,
    /// 溢出策略
    pub overflow_strategy: OverflowStrategy,
    /// 是否启用统计
    pub enable_stats: bool,
    /// 内存对齐要求
    pub alignment: usize,
    /// 是否使用DMA
    pub use_dma: bool,
}

impl Default for BufferConfig {
    fn default() -> Self {
        Self {
            size: 256,
            allow_overflow: false,
            overflow_strategy: OverflowStrategy::Block,
            enable_stats: true,
            alignment: 4,
            use_dma: false,
        }
    }
}

impl BufferConfig {
    /// 创建新的配置
    pub fn new(size: usize) -> Self {
        Self {
            size,
            ..Default::default()
        }
    }
    
    /// 设置溢出策略
    pub fn overflow_strategy(mut self, strategy: OverflowStrategy) -> Self {
        self.overflow_strategy = strategy;
        self
    }
    
    /// 启用溢出
    pub fn allow_overflow(mut self) -> Self {
        self.allow_overflow = true;
        self
    }
    
    /// 禁用统计
    pub fn disable_stats(mut self) -> Self {
        self.enable_stats = false;
        self
    }
    
    /// 设置内存对齐
    pub fn alignment(mut self, alignment: usize) -> Self {
        self.alignment = alignment;
        self
    }
    
    /// 启用DMA
    pub fn enable_dma(mut self) -> Self {
        self.use_dma = true;
        self
    }
    
    /// 验证配置
    pub fn validate(&self) -> Result<()> {
        if self.size == 0 {
            return Err(BufferError::InvalidSize.into());
        }
        
        if self.alignment == 0 || !self.alignment.is_power_of_two() {
            return Err(BufferError::Misaligned.into());
        }
        
        if self.size % self.alignment != 0 {
            return Err(BufferError::Misaligned.into());
        }
        
        Ok(())
    }
}

/// 溢出策略
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverflowStrategy {
    /// 阻塞直到有空间
    Block,
    /// 丢弃新数据
    DropNew,
    /// 丢弃旧数据
    DropOld,
    /// 返回错误
    Error,
    /// 扩展缓冲区
    Expand,
}

impl fmt::Display for OverflowStrategy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OverflowStrategy::Block => write!(f, "Block"),
            OverflowStrategy::DropNew => write!(f, "Drop new data"),
            OverflowStrategy::DropOld => write!(f, "Drop old data"),
            OverflowStrategy::Error => write!(f, "Return error"),
            OverflowStrategy::Expand => write!(f, "Expand buffer"),
        }
    }
}

/// 缓冲区工厂
pub struct BufferFactory;

impl BufferFactory {
    /// 创建环形缓冲区
    pub fn create_ring_buffer(config: BufferConfig) -> Result<RingBuffer> {
        config.validate()?;
        RingBuffer::new(config.size)
    }
    
    /// 创建DMA缓冲区
    pub fn create_dma_buffer(config: BufferConfig) -> Result<DmaBuffer> {
        config.validate()?;
        let dma_config = DmaBufferConfig {
            size: config.size,
            alignment: config.alignment,
            enable_stats: config.enable_stats,
        };
        DmaBuffer::new(dma_config)
    }
    
    /// 创建流缓冲区
    pub fn create_stream_buffer(config: BufferConfig) -> Result<StreamBuffer> {
        config.validate()?;
        let stream_config = StreamBufferConfig {
            size: config.size,
            overflow_strategy: config.overflow_strategy,
            enable_stats: config.enable_stats,
        };
        StreamBuffer::new(stream_config)
    }
}

/// 缓冲区类型枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferType {
    /// 环形缓冲区
    Ring,
    /// DMA缓冲区
    Dma,
    /// 流缓冲区
    Stream,
}

impl fmt::Display for BufferType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BufferType::Ring => write!(f, "Ring Buffer"),
            BufferType::Dma => write!(f, "DMA Buffer"),
            BufferType::Stream => write!(f, "Stream Buffer"),
        }
    }
}

/// 缓冲区信息
#[derive(Debug, Clone)]
pub struct BufferInfo {
    /// 缓冲区类型
    pub buffer_type: BufferType,
    /// 容量
    pub capacity: usize,
    /// 当前长度
    pub length: usize,
    /// 统计信息
    pub stats: BufferStats,
    /// 配置信息
    pub config: BufferConfig,
}

impl BufferInfo {
    /// 创建新的缓冲区信息
    pub fn new(buffer_type: BufferType, capacity: usize, length: usize, stats: BufferStats, config: BufferConfig) -> Self {
        Self {
            buffer_type,
            capacity,
            length,
            stats,
            config,
        }
    }
    
    /// 获取使用率
    pub fn usage_ratio(&self) -> f64 {
        if self.capacity > 0 {
            self.length as f64 / self.capacity as f64
        } else {
            0.0
        }
    }
    
    /// 检查是否接近满
    pub fn is_nearly_full(&self, threshold: f64) -> bool {
        self.usage_ratio() >= threshold
    }
    
    /// 检查是否接近空
    pub fn is_nearly_empty(&self, threshold: f64) -> bool {
        self.usage_ratio() <= threshold
    }
}

impl fmt::Display for BufferInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Buffer Info:")?;
        writeln!(f, "  Type: {}", self.buffer_type)?;
        writeln!(f, "  Capacity: {} bytes", self.capacity)?;
        writeln!(f, "  Length: {} bytes", self.length)?;
        writeln!(f, "  Usage: {:.1}%", self.usage_ratio() * 100.0)?;
        writeln!(f, "  Available: {} bytes", self.capacity - self.length)?;
        write!(f, "{}", self.stats)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_buffer_stats() {
        let mut stats = BufferStats::new();
        
        stats.record_read(100);
        stats.record_write(200);
        stats.update_usage(150);
        
        assert_eq!(stats.bytes_read, 100);
        assert_eq!(stats.bytes_written, 200);
        assert_eq!(stats.read_operations, 1);
        assert_eq!(stats.write_operations, 1);
        assert_eq!(stats.current_usage, 150);
        assert_eq!(stats.peak_usage, 150);
        
        assert_eq!(stats.read_throughput(), 100.0);
        assert_eq!(stats.write_throughput(), 200.0);
        assert_eq!(stats.usage_ratio(300), 0.5);
    }
    
    #[test]
    fn test_buffer_config() {
        let config = BufferConfig::new(1024)
            .overflow_strategy(OverflowStrategy::DropOld)
            .allow_overflow()
            .alignment(8)
            .enable_dma();
        
        assert_eq!(config.size, 1024);
        assert_eq!(config.overflow_strategy, OverflowStrategy::DropOld);
        assert!(config.allow_overflow);
        assert_eq!(config.alignment, 8);
        assert!(config.use_dma);
        
        assert!(config.validate().is_ok());
    }
    
    #[test]
    fn test_buffer_config_validation() {
        let config = BufferConfig::new(0);
        assert!(config.validate().is_err());
        
        let config = BufferConfig::new(1024).alignment(0);
        assert!(config.validate().is_err());
        
        let config = BufferConfig::new(1024).alignment(3);
        assert!(config.validate().is_err());
        
        let config = BufferConfig::new(1023).alignment(8);
        assert!(config.validate().is_err());
    }
    
    #[test]
    fn test_buffer_info() {
        let config = BufferConfig::new(1024);
        let stats = BufferStats::new();
        let info = BufferInfo::new(BufferType::Ring, 1024, 512, stats, config);
        
        assert_eq!(info.buffer_type, BufferType::Ring);
        assert_eq!(info.capacity, 1024);
        assert_eq!(info.length, 512);
        assert_eq!(info.usage_ratio(), 0.5);
        assert!(info.is_nearly_full(0.4));
        assert!(!info.is_nearly_empty(0.4));
    }
    
    #[test]
    fn test_overflow_strategy_display() {
        assert_eq!(OverflowStrategy::Block.to_string(), "Block");
        assert_eq!(OverflowStrategy::DropNew.to_string(), "Drop new data");
        assert_eq!(OverflowStrategy::DropOld.to_string(), "Drop old data");
        assert_eq!(OverflowStrategy::Error.to_string(), "Return error");
        assert_eq!(OverflowStrategy::Expand.to_string(), "Expand buffer");
    }
    
    #[test]
    fn test_buffer_type_display() {
        assert_eq!(BufferType::Ring.to_string(), "Ring Buffer");
        assert_eq!(BufferType::Dma.to_string(), "DMA Buffer");
        assert_eq!(BufferType::Stream.to_string(), "Stream Buffer");
    }
}