#![no_std]

//! 多设备通信管理库
//! 
//! 本库提供了多设备I2C/SPI通信的统一管理接口，
//! 支持设备发现、总线调度、错误恢复等功能。

pub mod bus_manager;

pub use bus_manager::*;

/// 库版本信息
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// 支持的设备类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeviceType {
    /// I2C设备
    I2c,
    /// SPI设备
    Spi,
    /// 未知设备
    Unknown,
}

/// 设备优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    /// 低优先级
    Low = 0,
    /// 普通优先级
    Normal = 1,
    /// 高优先级
    High = 2,
    /// 紧急优先级
    Critical = 3,
}

/// 总线类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BusType {
    /// I2C总线
    I2c,
    /// SPI总线
    Spi,
}

/// 设备状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeviceStatus {
    /// 设备在线
    Online,
    /// 设备离线
    Offline,
    /// 设备错误
    Error,
    /// 设备忙碌
    Busy,
    /// 设备未知状态
    Unknown,
}

/// 通信错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CommunicationError {
    /// 总线错误
    BusError,
    /// 设备无响应
    NoResponse,
    /// 数据校验错误
    ChecksumError,
    /// 超时错误
    Timeout,
    /// 地址冲突
    AddressConflict,
    /// 缓冲区满
    BufferFull,
    /// 无效参数
    InvalidParameter,
}

/// 已知I2C设备地址
pub mod i2c_addresses {
    pub const PCF8574: u8 = 0x20;
    pub const AT24C256: u8 = 0x50;
    pub const DS3231_RTC: u8 = 0x68;
    pub const BMP280: u8 = 0x76;
    pub const BMP280_ALT: u8 = 0x77;
}

/// SPI设备配置
pub mod spi_config {
    use fugit::HertzU32;
    
    pub const W25Q128_MAX_FREQ: HertzU32 = HertzU32::MHz(80);
    pub const MCP3008_MAX_FREQ: HertzU32 = HertzU32::MHz(3);
    pub const MAX7219_MAX_FREQ: HertzU32 = HertzU32::MHz(10);
}

/// 性能统计信息
#[derive(Debug, Clone, Copy)]
pub struct PerformanceStats {
    /// 总操作次数
    pub total_operations: u32,
    /// 成功操作次数
    pub successful_operations: u32,
    /// 失败操作次数
    pub failed_operations: u32,
    /// 平均响应时间（微秒）
    pub average_response_time_us: u32,
    /// 总线使用率（百分比）
    pub bus_utilization_percent: u8,
    /// 最后错误时间戳
    pub last_error_timestamp: u32,
}

impl PerformanceStats {
    pub fn new() -> Self {
        Self {
            total_operations: 0,
            successful_operations: 0,
            failed_operations: 0,
            average_response_time_us: 0,
            bus_utilization_percent: 0,
            last_error_timestamp: 0,
        }
    }
    
    pub fn success_rate(&self) -> f32 {
        if self.total_operations == 0 {
            0.0
        } else {
            (self.successful_operations as f32) / (self.total_operations as f32) * 100.0
        }
    }
}

/// 设备信息
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    /// 设备ID
    pub id: u8,
    /// 设备类型
    pub device_type: DeviceType,
    /// 设备地址（I2C）或CS引脚（SPI）
    pub address: u8,
    /// 设备名称
    pub name: heapless::String<32>,
    /// 设备状态
    pub status: DeviceStatus,
    /// 设备优先级
    pub priority: Priority,
    /// 最后访问时间
    pub last_access_time: u32,
    /// 性能统计
    pub stats: PerformanceStats,
}

impl DeviceInfo {
    pub fn new(id: u8, device_type: DeviceType, address: u8, name: &str) -> Self {
        let mut device_name = heapless::String::new();
        device_name.push_str(name).ok();
        
        Self {
            id,
            device_type,
            address,
            name: device_name,
            status: DeviceStatus::Unknown,
            priority: Priority::Normal,
            last_access_time: 0,
            stats: PerformanceStats::new(),
        }
    }
}