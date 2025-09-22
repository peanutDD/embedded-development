//! 跨平台错误处理模块
//! 
//! 定义了统一的错误类型，用于在不同平台间提供一致的错误处理体验。

use core::fmt;

/// 跨平台错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrossPlatformError {
    /// GPIO相关错误
    GpioError,
    
    /// 通信错误
    CommunicationError,
    
    /// 传感器错误
    SensorError,
    
    /// 定时器错误
    TimerError,
    
    /// 无效参数
    InvalidParameter,
    
    /// 资源耗尽
    ResourceExhausted,
    
    /// 初始化错误
    InitializationError,
    
    /// 超时错误
    TimeoutError,
    
    /// 未连接
    NotConnected,
    
    /// 不支持的操作
    UnsupportedOperation,
    
    /// 硬件错误
    HardwareError,
    
    /// 内存错误
    MemoryError,
    
    /// 配置错误
    ConfigurationError,
    
    /// 校验和错误
    ChecksumError,
    
    /// 协议错误
    ProtocolError,
    
    /// 平台特定错误
    PlatformSpecific(u16),
}

// 手动实现hash32::Hash trait
impl hash32::Hash for CrossPlatformError {
    fn hash<H>(&self, state: &mut H) 
    where 
        H: hash32::Hasher,
    {
        // 使用简单的数值映射来实现hash
        let value = match self {
            CrossPlatformError::GpioError => 1u8,
            CrossPlatformError::CommunicationError => 2u8,
            CrossPlatformError::SensorError => 3u8,
            CrossPlatformError::TimerError => 4u8,
            CrossPlatformError::InvalidParameter => 5u8,
            CrossPlatformError::ResourceExhausted => 6u8,
            CrossPlatformError::InitializationError => 7u8,
            CrossPlatformError::TimeoutError => 8u8,
            CrossPlatformError::NotConnected => 9u8,
            CrossPlatformError::UnsupportedOperation => 10u8,
            CrossPlatformError::HardwareError => 11u8,
            CrossPlatformError::MemoryError => 12u8,
            CrossPlatformError::ConfigurationError => 13u8,
            CrossPlatformError::ChecksumError => 14u8,
            CrossPlatformError::ProtocolError => 15u8,
            CrossPlatformError::PlatformSpecific(code) => {
                hash32::Hash::hash(&16u8, state);
                hash32::Hash::hash(code, state);
                return;
            }
        };
        hash32::Hash::hash(&value, state);
    }
}

impl fmt::Display for CrossPlatformError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CrossPlatformError::GpioError => write!(f, "GPIO操作错误"),
            CrossPlatformError::CommunicationError => write!(f, "通信错误"),
            CrossPlatformError::SensorError => write!(f, "传感器错误"),
            CrossPlatformError::TimerError => write!(f, "定时器错误"),
            CrossPlatformError::InvalidParameter => write!(f, "无效参数"),
            CrossPlatformError::ResourceExhausted => write!(f, "资源耗尽"),
            CrossPlatformError::InitializationError => write!(f, "初始化错误"),
            CrossPlatformError::TimeoutError => write!(f, "超时错误"),
            CrossPlatformError::NotConnected => write!(f, "未连接"),
            CrossPlatformError::UnsupportedOperation => write!(f, "不支持的操作"),
            CrossPlatformError::HardwareError => write!(f, "硬件错误"),
            CrossPlatformError::MemoryError => write!(f, "内存错误"),
            CrossPlatformError::ConfigurationError => write!(f, "配置错误"),
            CrossPlatformError::ChecksumError => write!(f, "校验和错误"),
            CrossPlatformError::ProtocolError => write!(f, "协议错误"),
            CrossPlatformError::PlatformSpecific(code) => write!(f, "平台特定错误: {}", code),
        }
    }
}

/// 错误转换特征
pub trait ErrorConversion<T> {
    fn to_cross_platform_error(self) -> CrossPlatformError;
}

/// 结果类型别名
pub type Result<T> = core::result::Result<T, CrossPlatformError>;

/// 错误处理工具
pub struct ErrorHandler {
    error_count: u32,
    last_error: Option<CrossPlatformError>,
}

impl ErrorHandler {
    pub fn new() -> Self {
        Self {
            error_count: 0,
            last_error: None,
        }
    }
    
    pub fn handle_error(&mut self, error: CrossPlatformError) {
        self.error_count += 1;
        self.last_error = Some(error);
        
        // 根据错误类型执行相应的处理逻辑
        match error {
            CrossPlatformError::HardwareError => {
                // 硬件错误可能需要重置
                self.handle_hardware_error();
            }
            CrossPlatformError::CommunicationError => {
                // 通信错误可能需要重连
                self.handle_communication_error();
            }
            CrossPlatformError::TimeoutError => {
                // 超时错误可能需要重试
                self.handle_timeout_error();
            }
            _ => {
                // 其他错误的通用处理
                self.handle_generic_error(error);
            }
        }
    }
    
    fn handle_hardware_error(&self) {
        // 硬件错误处理逻辑
        #[cfg(feature = "debug")]
        crate::debug::print_error("硬件错误，尝试重置");
    }
    
    fn handle_communication_error(&self) {
        // 通信错误处理逻辑
        #[cfg(feature = "debug")]
        crate::debug::print_error("通信错误，尝试重连");
    }
    
    fn handle_timeout_error(&self) {
        // 超时错误处理逻辑
        #[cfg(feature = "debug")]
        crate::debug::print_error("超时错误，准备重试");
    }
    
    fn handle_generic_error(&self, _error: CrossPlatformError) {
        // 通用错误处理逻辑
        #[cfg(feature = "debug")]
        crate::debug::print_error(&format!("发生错误: {:?}", _error));
    }
    
    pub fn get_error_count(&self) -> u32 {
        self.error_count
    }
    
    pub fn get_last_error(&self) -> Option<CrossPlatformError> {
        self.last_error
    }
    
    pub fn clear_errors(&mut self) {
        self.error_count = 0;
        self.last_error = None;
    }
    
    pub fn has_critical_errors(&self) -> bool {
        matches!(
            self.last_error,
            Some(CrossPlatformError::HardwareError) |
            Some(CrossPlatformError::MemoryError) |
            Some(CrossPlatformError::InitializationError)
        )
    }
}

/// 错误恢复策略
#[derive(Debug, Clone, Copy)]
pub enum RecoveryStrategy {
    /// 忽略错误继续执行
    Ignore,
    /// 重试操作
    Retry(u8),
    /// 重置组件
    Reset,
    /// 降级操作
    Fallback,
    /// 停止执行
    Halt,
}

/// 错误恢复管理器
pub struct RecoveryManager {
    strategies: heapless::FnvIndexMap<CrossPlatformError, RecoveryStrategy, 16>,
    retry_counts: heapless::FnvIndexMap<CrossPlatformError, u8, 16>,
}

impl RecoveryManager {
    pub fn new() -> Self {
        let mut manager = Self {
            strategies: heapless::FnvIndexMap::new(),
            retry_counts: heapless::FnvIndexMap::new(),
        };
        
        // 设置默认恢复策略
        manager.set_default_strategies();
        manager
    }
    
    fn set_default_strategies(&mut self) {
        // GPIO错误 - 重试3次
        self.strategies.insert(CrossPlatformError::GpioError, RecoveryStrategy::Retry(3)).ok();
        
        // 通信错误 - 重试5次
        self.strategies.insert(CrossPlatformError::CommunicationError, RecoveryStrategy::Retry(5)).ok();
        
        // 传感器错误 - 重置
        self.strategies.insert(CrossPlatformError::SensorError, RecoveryStrategy::Reset).ok();
        
        // 超时错误 - 重试2次
        self.strategies.insert(CrossPlatformError::TimeoutError, RecoveryStrategy::Retry(2)).ok();
        
        // 硬件错误 - 停止执行
        self.strategies.insert(CrossPlatformError::HardwareError, RecoveryStrategy::Halt).ok();
        
        // 内存错误 - 停止执行
        self.strategies.insert(CrossPlatformError::MemoryError, RecoveryStrategy::Halt).ok();
        
        // 无效参数 - 忽略
        self.strategies.insert(CrossPlatformError::InvalidParameter, RecoveryStrategy::Ignore).ok();
    }
    
    pub fn set_strategy(&mut self, error: CrossPlatformError, strategy: RecoveryStrategy) -> Result<()> {
        self.strategies.insert(error, strategy)
            .map_err(|_| CrossPlatformError::ResourceExhausted)?;
        Ok(())
    }
    
    pub fn handle_error(&mut self, error: CrossPlatformError) -> RecoveryAction {
        let strategy = self.strategies.get(&error)
            .copied()
            .unwrap_or(RecoveryStrategy::Ignore);
        
        match strategy {
            RecoveryStrategy::Ignore => RecoveryAction::Continue,
            RecoveryStrategy::Retry(max_retries) => {
                let current_retries = self.retry_counts.get(&error).copied().unwrap_or(0);
                if current_retries < max_retries {
                    self.retry_counts.insert(error, current_retries + 1).ok();
                    RecoveryAction::Retry
                } else {
                    self.retry_counts.insert(error, 0).ok();
                    RecoveryAction::Fallback
                }
            }
            RecoveryStrategy::Reset => RecoveryAction::Reset,
            RecoveryStrategy::Fallback => RecoveryAction::Fallback,
            RecoveryStrategy::Halt => RecoveryAction::Halt,
        }
    }
    
    pub fn reset_retry_count(&mut self, error: CrossPlatformError) {
        self.retry_counts.insert(error, 0).ok();
    }
    
    pub fn get_retry_count(&self, error: CrossPlatformError) -> u8 {
        self.retry_counts.get(&error).copied().unwrap_or(0)
    }
}

/// 恢复动作
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryAction {
    /// 继续执行
    Continue,
    /// 重试操作
    Retry,
    /// 重置组件
    Reset,
    /// 使用备用方案
    Fallback,
    /// 停止执行
    Halt,
}

/// 错误统计
#[derive(Debug, Clone)]
pub struct ErrorStatistics {
    pub total_errors: u32,
    pub error_counts: heapless::FnvIndexMap<CrossPlatformError, u32, 16>,
    pub last_error_time: Option<u32>,
}

impl ErrorStatistics {
    pub fn new() -> Self {
        Self {
            total_errors: 0,
            error_counts: heapless::FnvIndexMap::new(),
            last_error_time: None,
        }
    }
    
    pub fn record_error(&mut self, error: CrossPlatformError, timestamp: u32) {
        self.total_errors += 1;
        self.last_error_time = Some(timestamp);
        
        let count = self.error_counts.get(&error).copied().unwrap_or(0);
        self.error_counts.insert(error, count + 1).ok();
    }
    
    pub fn get_error_count(&self, error: CrossPlatformError) -> u32 {
        self.error_counts.get(&error).copied().unwrap_or(0)
    }
    
    pub fn get_most_frequent_error(&self) -> Option<(CrossPlatformError, u32)> {
        self.error_counts
            .iter()
            .max_by_key(|(_, &count)| count)
            .map(|(&error, &count)| (error, count))
    }
    
    pub fn clear(&mut self) {
        self.total_errors = 0;
        self.error_counts.clear();
        self.last_error_time = None;
    }
}

// 调试支持
#[cfg(feature = "debug")]
pub mod debug {
    use super::*;
    
    pub fn print_error(msg: &str) {
        #[cfg(feature = "defmt")]
        defmt::error!("{}", msg);
        
        #[cfg(not(feature = "defmt"))]
        {
            #[cfg(feature = "semihosting")]
            cortex_m_semihosting::hprintln!("[ERROR] {}", msg).ok();
        }
    }
    
    pub fn print_error_details(error: CrossPlatformError) {
        let msg = match error {
            CrossPlatformError::GpioError => "GPIO操作失败",
            CrossPlatformError::CommunicationError => "通信失败",
            CrossPlatformError::SensorError => "传感器读取失败",
            CrossPlatformError::TimerError => "定时器操作失败",
            CrossPlatformError::InvalidParameter => "参数无效",
            CrossPlatformError::ResourceExhausted => "资源不足",
            CrossPlatformError::InitializationError => "初始化失败",
            CrossPlatformError::TimeoutError => "操作超时",
            CrossPlatformError::NotConnected => "设备未连接",
            CrossPlatformError::UnsupportedOperation => "操作不支持",
            CrossPlatformError::HardwareError => "硬件故障",
            CrossPlatformError::MemoryError => "内存错误",
            CrossPlatformError::ConfigurationError => "配置错误",
            CrossPlatformError::ChecksumError => "数据校验失败",
            CrossPlatformError::ProtocolError => "协议错误",
            CrossPlatformError::PlatformSpecific(code) => {
                print_error(&format!("平台特定错误，代码: {}", code));
                return;
            }
        };
        
        print_error(msg);
    }
}