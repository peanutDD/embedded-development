//! DMA传输示例库
//! 
//! 本库提供了多种DMA串口传输的示例实现：
//! - 基础DMA传输
//! - 循环DMA传输
//! - 双缓冲DMA传输
//! - 高速数据记录器

#![no_std]

pub mod basic_dma;
pub mod circular_dma;
pub mod double_buffer;
pub mod high_speed_logger;

// 重新导出主要类型和函数
pub use basic_dma::{
    DmaTransferType, DmaStatistics, DmaConfig as BasicDmaConfig,
    setup_dma_transfer, start_dma_transmission, start_dma_reception,
};

pub use circular_dma::{
    CircularDmaManager, CircularBufferState, DataProcessor as CircularDataProcessor,
    setup_circular_dma, process_circular_data,
};

pub use double_buffer::{
    DoubleBufferManager, BufferState, DataProcessor as DoubleBufferDataProcessor,
    PerformanceMonitor, setup_double_buffer_dma,
};

pub use high_speed_logger::{
    HighSpeedLogger, DataProcessor as LoggerDataProcessor, CompressionEngine,
    PerformanceStats, PacketType, DataFormat,
};

/// DMA传输优先级
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaPriority {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

/// DMA传输方向
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaDirection {
    MemoryToPeripheral,
    PeripheralToMemory,
    MemoryToMemory,
}

/// DMA传输模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaMode {
    Normal,
    Circular,
    DoubleBuffer,
}

/// DMA错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaError {
    TransferError,
    FifoError,
    DirectModeError,
    BufferOverrun,
    BufferUnderrun,
    ConfigurationError,
    TimeoutError,
}

/// DMA传输状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaStatus {
    Idle,
    InProgress,
    HalfComplete,
    Complete,
    Error(DmaError),
}

/// 通用DMA配置结构
#[derive(Debug, Clone)]
pub struct DmaConfiguration {
    pub priority: DmaPriority,
    pub direction: DmaDirection,
    pub mode: DmaMode,
    pub memory_increment: bool,
    pub peripheral_increment: bool,
    pub memory_data_size: DataSize,
    pub peripheral_data_size: DataSize,
    pub circular_buffer: bool,
    pub double_buffer: bool,
    pub fifo_mode: bool,
    pub fifo_threshold: FifoThreshold,
}

/// 数据大小
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DataSize {
    Byte = 0,
    HalfWord = 1,
    Word = 2,
}

/// FIFO阈值
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FifoThreshold {
    Quarter = 0,
    Half = 1,
    ThreeQuarters = 2,
    Full = 3,
}

impl Default for DmaConfiguration {
    fn default() -> Self {
        Self {
            priority: DmaPriority::Medium,
            direction: DmaDirection::MemoryToPeripheral,
            mode: DmaMode::Normal,
            memory_increment: true,
            peripheral_increment: false,
            memory_data_size: DataSize::Byte,
            peripheral_data_size: DataSize::Byte,
            circular_buffer: false,
            double_buffer: false,
            fifo_mode: false,
            fifo_threshold: FifoThreshold::Half,
        }
    }
}

/// DMA传输统计信息
#[derive(Debug, Default)]
pub struct DmaTransferStats {
    pub total_transfers: u32,
    pub successful_transfers: u32,
    pub failed_transfers: u32,
    pub bytes_transferred: u64,
    pub average_transfer_time_us: u32,
    pub max_transfer_time_us: u32,
    pub min_transfer_time_us: u32,
    pub error_count: u32,
    pub last_error: Option<DmaError>,
}

impl DmaTransferStats {
    pub fn new() -> Self {
        Self {
            min_transfer_time_us: u32::MAX,
            ..Default::default()
        }
    }
    
    pub fn record_transfer(&mut self, bytes: u32, time_us: u32, success: bool) {
        self.total_transfers += 1;
        
        if success {
            self.successful_transfers += 1;
            self.bytes_transferred += bytes as u64;
            
            // 更新时间统计
            self.max_transfer_time_us = self.max_transfer_time_us.max(time_us);
            self.min_transfer_time_us = self.min_transfer_time_us.min(time_us);
            
            // 计算平均时间
            let total_time = self.average_transfer_time_us as u64 * (self.successful_transfers - 1) as u64 + time_us as u64;
            self.average_transfer_time_us = (total_time / self.successful_transfers as u64) as u32;
        } else {
            self.failed_transfers += 1;
        }
    }
    
    pub fn record_error(&mut self, error: DmaError) {
        self.error_count += 1;
        self.last_error = Some(error);
    }
    
    pub fn success_rate(&self) -> f32 {
        if self.total_transfers == 0 {
            0.0
        } else {
            self.successful_transfers as f32 / self.total_transfers as f32
        }
    }
    
    pub fn throughput_bps(&self) -> f32 {
        if self.average_transfer_time_us == 0 {
            0.0
        } else {
            (self.bytes_transferred as f32 * 1_000_000.0) / 
            (self.total_transfers as f32 * self.average_transfer_time_us as f32)
        }
    }
}

/// DMA传输回调特征
pub trait DmaCallback {
    fn on_transfer_complete(&mut self, bytes_transferred: usize);
    fn on_half_transfer(&mut self, bytes_transferred: usize);
    fn on_transfer_error(&mut self, error: DmaError);
}

/// 空回调实现
pub struct NullCallback;

impl DmaCallback for NullCallback {
    fn on_transfer_complete(&mut self, _bytes_transferred: usize) {}
    fn on_half_transfer(&mut self, _bytes_transferred: usize) {}
    fn on_transfer_error(&mut self, _error: DmaError) {}
}

/// DMA传输管理器特征
pub trait DmaManager {
    type Error;
    
    fn start_transfer(&mut self, buffer: &[u8]) -> Result<(), Self::Error>;
    fn stop_transfer(&mut self) -> Result<(), Self::Error>;
    fn is_busy(&self) -> bool;
    fn get_status(&self) -> DmaStatus;
    fn get_stats(&self) -> &DmaTransferStats;
    fn reset_stats(&mut self);
}

/// 缓冲区管理器特征
pub trait BufferManager {
    type Buffer;
    
    fn get_buffer(&mut self) -> Option<&mut Self::Buffer>;
    fn return_buffer(&mut self, buffer: Self::Buffer);
    fn available_buffers(&self) -> usize;
    fn total_buffers(&self) -> usize;
}

/// 数据处理器特征
pub trait DataProcessor {
    type Input;
    type Output;
    type Error;
    
    fn process(&mut self, input: Self::Input) -> Result<Self::Output, Self::Error>;
    fn reset(&mut self);
}

/// 性能监控器特征
pub trait PerformanceMonitor {
    fn update_metrics(&mut self);
    fn get_throughput(&self) -> f32;
    fn get_latency(&self) -> f32;
    fn get_error_rate(&self) -> f32;
    fn reset_metrics(&mut self);
}

/// 实用工具函数
pub mod utils {
    use super::*;
    
    /// 计算最优DMA传输大小
    pub fn calculate_optimal_transfer_size(
        data_size: usize,
        max_transfer_size: usize,
        alignment: usize,
    ) -> usize {
        let aligned_size = (data_size + alignment - 1) & !(alignment - 1);
        aligned_size.min(max_transfer_size)
    }
    
    /// 检查内存对齐
    pub fn is_aligned(address: usize, alignment: usize) -> bool {
        address & (alignment - 1) == 0
    }
    
    /// 获取DMA优先级建议
    pub fn get_priority_recommendation(
        data_rate_bps: u32,
        latency_requirement_us: u32,
    ) -> DmaPriority {
        match (data_rate_bps, latency_requirement_us) {
            (rate, latency) if rate > 1_000_000 || latency < 100 => DmaPriority::VeryHigh,
            (rate, latency) if rate > 100_000 || latency < 1000 => DmaPriority::High,
            (rate, latency) if rate > 10_000 || latency < 10000 => DmaPriority::Medium,
            _ => DmaPriority::Low,
        }
    }
    
    /// 计算FIFO阈值
    pub fn calculate_fifo_threshold(transfer_size: usize) -> FifoThreshold {
        match transfer_size {
            size if size >= 16 => FifoThreshold::Full,
            size if size >= 8 => FifoThreshold::ThreeQuarters,
            size if size >= 4 => FifoThreshold::Half,
            _ => FifoThreshold::Quarter,
        }
    }
    
    /// 验证DMA配置
    pub fn validate_dma_config(config: &DmaConfiguration) -> Result<(), DmaError> {
        // 检查配置的有效性
        if config.double_buffer && config.circular_buffer {
            return Err(DmaError::ConfigurationError);
        }
        
        if config.fifo_mode && config.memory_data_size != config.peripheral_data_size {
            return Err(DmaError::ConfigurationError);
        }
        
        Ok(())
    }
}

/// 测试模块
#[cfg(test)]
mod tests {
    use super::*;
    use super::utils::*;
    
    #[test]
    fn test_dma_stats() {
        let mut stats = DmaTransferStats::new();
        
        // 记录成功传输
        stats.record_transfer(1024, 100, true);
        assert_eq!(stats.successful_transfers, 1);
        assert_eq!(stats.bytes_transferred, 1024);
        assert_eq!(stats.average_transfer_time_us, 100);
        
        // 记录失败传输
        stats.record_transfer(512, 50, false);
        assert_eq!(stats.failed_transfers, 1);
        assert_eq!(stats.total_transfers, 2);
        
        // 检查成功率
        assert_eq!(stats.success_rate(), 0.5);
    }
    
    #[test]
    fn test_optimal_transfer_size() {
        assert_eq!(calculate_optimal_transfer_size(100, 1024, 4), 100);
        assert_eq!(calculate_optimal_transfer_size(1000, 512, 4), 512);
        assert_eq!(calculate_optimal_transfer_size(7, 1024, 4), 8);
    }
    
    #[test]
    fn test_alignment_check() {
        assert!(is_aligned(0x1000, 4));
        assert!(is_aligned(0x1004, 4));
        assert!(!is_aligned(0x1001, 4));
        assert!(!is_aligned(0x1002, 4));
    }
    
    #[test]
    fn test_priority_recommendation() {
        assert_eq!(get_priority_recommendation(2_000_000, 50), DmaPriority::VeryHigh);
        assert_eq!(get_priority_recommendation(500_000, 500), DmaPriority::High);
        assert_eq!(get_priority_recommendation(50_000, 5000), DmaPriority::Medium);
        assert_eq!(get_priority_recommendation(1000, 50000), DmaPriority::Low);
    }
    
    #[test]
    fn test_config_validation() {
        let mut config = DmaConfiguration::default();
        assert!(validate_dma_config(&config).is_ok());
        
        // 测试无效配置
        config.double_buffer = true;
        config.circular_buffer = true;
        assert!(validate_dma_config(&config).is_err());
    }
}