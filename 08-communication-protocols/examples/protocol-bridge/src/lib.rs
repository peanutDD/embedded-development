#![no_std]

//! # Protocol Bridge Library
//! 
//! I2C-SPI协议桥接库，提供不同通信协议之间的转换功能。

/// 库版本信息
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// 默认I2C从设备地址
pub const DEFAULT_I2C_ADDRESS: u8 = 0x48;

/// 支持的I2C地址范围
pub const I2C_ADDRESS_MIN: u8 = 0x08;
pub const I2C_ADDRESS_MAX: u8 = 0x77;

/// SPI设备配置
pub const MAX_SPI_DEVICES: usize = 4;
pub const SPI_MAX_FREQUENCY: u32 = 10_000_000; // 10MHz

/// 数据缓冲区大小
pub const BUFFER_SIZE: usize = 256;
pub const COMMAND_BUFFER_SIZE: usize = 64;

/// 协议转换超时时间 (毫秒)
pub const CONVERSION_TIMEOUT_MS: u32 = 100;
pub const I2C_TIMEOUT_MS: u32 = 50;
pub const SPI_TIMEOUT_MS: u32 = 20;

/// CRC多项式
pub const CRC_POLYNOMIAL: u16 = 0x1021; // CRC-16-CCITT

/// 命令类型定义
pub mod commands {
    /// 读命令前缀
    pub const READ_CMD: u8 = 0x80;
    /// 写命令前缀
    pub const WRITE_CMD: u8 = 0x00;
    /// 状态查询命令
    pub const STATUS_CMD: u8 = 0xF0;
    /// 复位命令
    pub const RESET_CMD: u8 = 0xFF;
}

/// 状态码定义
pub mod status {
    /// 操作成功
    pub const SUCCESS: u8 = 0x00;
    /// 设备忙
    pub const BUSY: u8 = 0x01;
    /// 无效命令
    pub const INVALID_COMMAND: u8 = 0x02;
    /// 设备未找到
    pub const DEVICE_NOT_FOUND: u8 = 0x03;
    /// CRC错误
    pub const CRC_ERROR: u8 = 0x04;
    /// 超时错误
    pub const TIMEOUT: u8 = 0x05;
    /// 缓冲区溢出
    pub const BUFFER_OVERFLOW: u8 = 0x06;
}

// 模块声明
pub mod bridge;
pub mod i2c_slave;
pub mod spi_master;
pub mod protocol;