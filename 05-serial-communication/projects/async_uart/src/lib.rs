//! # 异步UART通信库
//!
//! 这是一个基于Rust async/await的高性能嵌入式UART通信库，支持多种硬件平台。
//! 
//! ## 特性
//! 
//! - 🚀 **高性能异步IO**: 基于Embassy框架的零成本异步抽象
//! - 🔧 **多平台支持**: STM32、ESP32、RP2040、nRF52等主流平台
//! - 📦 **DMA支持**: 高效的DMA传输，减少CPU占用
//! - 🛡️ **类型安全**: 编译时保证的类型安全和内存安全
//! - 🔄 **流控制**: 硬件和软件流控制支持
//! - 📊 **缓冲管理**: 智能缓冲区管理和背压处理
//! - 🔌 **协议支持**: 内置常用串口协议支持
//! 
//! ## 快速开始
//! 
//! ```rust,no_run
//! use async_uart::{AsyncUart, Config, Baudrate};
//! use embassy_executor::Spawner;
//! 
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     let config = Config::default()
//!         .baudrate(Baudrate::Bps115200)
//!         .data_bits(8)
//!         .stop_bits(1);
//!     
//!     let mut uart = AsyncUart::new(uart_peripheral, config).await?;
//!     
//!     // 异步发送数据
//!     uart.write_all(b"Hello, World!").await?;
//!     
//!     // 异步接收数据
//!     let mut buffer = [0u8; 64];
//!     let len = uart.read(&mut buffer).await?;
//!     
//!     println!("Received: {:?}", &buffer[..len]);
//! }
//! ```

#![no_std]
#![deny(unsafe_code)]
#![warn(
    missing_docs,
    rust_2018_idioms,
    unused_qualifications,
    missing_debug_implementations
)]

// 核心模块
pub mod traits;
pub mod config;
pub mod error;
pub mod buffer;
pub mod dma;
pub mod protocol;
pub mod utils;

// 平台特定HAL适配器
#[cfg(feature = "stm32")]
pub mod hal;

// 重新导出核心类型
pub use traits::{AsyncRead, AsyncWrite, AsyncUart as AsyncUartTrait};
pub use config::{Config, Baudrate, DataBits, StopBits, Parity, FlowControl};
pub use error::{Error, Result};
pub use buffer::{RingBuffer, BufferManager};

// 条件导出平台特定实现
#[cfg(feature = "stm32")]
pub use hal::stm32::Stm32AsyncUart as AsyncUart;

#[cfg(feature = "esp32")]
pub use hal::esp32::Esp32AsyncUart as AsyncUart;

#[cfg(feature = "rp2040")]
pub use hal::rp2040::Rp2040AsyncUart as AsyncUart;

#[cfg(feature = "nrf52")]
pub use hal::nrf52::Nrf52AsyncUart as AsyncUart;

/// 库版本信息
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// 库名称
pub const NAME: &str = env!("CARGO_PKG_NAME");

/// 库描述
pub const DESCRIPTION: &str = env!("CARGO_PKG_DESCRIPTION");

/// 预定义的常用配置
pub mod presets {
    use super::*;
    
    /// 标准115200波特率配置
    pub const STANDARD_115200: Config = Config {
        baudrate: Baudrate::Bps115200,
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: Parity::None,
        flow_control: FlowControl::None,
        buffer_size: 256,
        timeout_ms: 1000,
        dma_enabled: true,
    };
    
    /// 高速921600波特率配置
    pub const HIGH_SPEED_921600: Config = Config {
        baudrate: Baudrate::Bps921600,
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: Parity::None,
        flow_control: FlowControl::RtsCts,
        buffer_size: 1024,
        timeout_ms: 500,
        dma_enabled: true,
    };
    
    /// 低功耗9600波特率配置
    pub const LOW_POWER_9600: Config = Config {
        baudrate: Baudrate::Bps9600,
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: Parity::None,
        flow_control: FlowControl::None,
        buffer_size: 64,
        timeout_ms: 5000,
        dma_enabled: false,
    };
    
    /// Modbus RTU配置
    pub const MODBUS_RTU: Config = Config {
        baudrate: Baudrate::Bps19200,
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: Parity::Even,
        flow_control: FlowControl::None,
        buffer_size: 256,
        timeout_ms: 100,
        dma_enabled: true,
    };
}

/// 异步UART构建器
pub struct AsyncUartBuilder {
    config: Config,
}

impl AsyncUartBuilder {
    /// 创建新的构建器
    pub fn new() -> Self {
        Self {
            config: Config::default(),
        }
    }
    
    /// 设置波特率
    pub fn baudrate(mut self, baudrate: Baudrate) -> Self {
        self.config.baudrate = baudrate;
        self
    }
    
    /// 设置数据位
    pub fn data_bits(mut self, data_bits: DataBits) -> Self {
        self.config.data_bits = data_bits;
        self
    }
    
    /// 设置停止位
    pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.config.stop_bits = stop_bits;
        self
    }
    
    /// 设置校验位
    pub fn parity(mut self, parity: Parity) -> Self {
        self.config.parity = parity;
        self
    }
    
    /// 设置流控制
    pub fn flow_control(mut self, flow_control: FlowControl) -> Self {
        self.config.flow_control = flow_control;
        self
    }
    
    /// 设置缓冲区大小
    pub fn buffer_size(mut self, size: usize) -> Self {
        self.config.buffer_size = size;
        self
    }
    
    /// 设置超时时间
    pub fn timeout_ms(mut self, timeout: u32) -> Self {
        self.config.timeout_ms = timeout;
        self
    }
    
    /// 启用DMA
    pub fn enable_dma(mut self) -> Self {
        self.config.dma_enabled = true;
        self
    }
    
    /// 禁用DMA
    pub fn disable_dma(mut self) -> Self {
        self.config.dma_enabled = false;
        self
    }
    
    /// 获取配置
    pub fn config(&self) -> &Config {
        &self.config
    }
    
    /// 构建配置
    pub fn build(self) -> Config {
        self.config
    }
}

impl Default for AsyncUartBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// 异步UART统计信息
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Statistics {
    /// 发送字节数
    pub bytes_sent: u64,
    /// 接收字节数
    pub bytes_received: u64,
    /// 发送错误数
    pub send_errors: u32,
    /// 接收错误数
    pub receive_errors: u32,
    /// 超时次数
    pub timeouts: u32,
    /// 缓冲区溢出次数
    pub buffer_overflows: u32,
    /// DMA传输次数
    pub dma_transfers: u32,
}

impl Default for Statistics {
    fn default() -> Self {
        Self {
            bytes_sent: 0,
            bytes_received: 0,
            send_errors: 0,
            receive_errors: 0,
            timeouts: 0,
            buffer_overflows: 0,
            dma_transfers: 0,
        }
    }
}

impl Statistics {
    /// 重置统计信息
    pub fn reset(&mut self) {
        *self = Self::default();
    }
    
    /// 获取总传输字节数
    pub fn total_bytes(&self) -> u64 {
        self.bytes_sent + self.bytes_received
    }
    
    /// 获取总错误数
    pub fn total_errors(&self) -> u32 {
        self.send_errors + self.receive_errors + self.timeouts + self.buffer_overflows
    }
    
    /// 计算错误率（百分比）
    pub fn error_rate(&self) -> f32 {
        let total = self.total_bytes();
        if total == 0 {
            0.0
        } else {
            (self.total_errors() as f32 / total as f32) * 100.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_builder_pattern() {
        let config = AsyncUartBuilder::new()
            .baudrate(Baudrate::Bps115200)
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .buffer_size(512)
            .timeout_ms(1000)
            .enable_dma()
            .build();
        
        assert_eq!(config.baudrate, Baudrate::Bps115200);
        assert_eq!(config.data_bits, DataBits::Eight);
        assert_eq!(config.stop_bits, StopBits::One);
        assert_eq!(config.parity, Parity::None);
        assert_eq!(config.flow_control, FlowControl::None);
        assert_eq!(config.buffer_size, 512);
        assert_eq!(config.timeout_ms, 1000);
        assert!(config.dma_enabled);
    }
    
    #[test]
    fn test_statistics() {
        let mut stats = Statistics::default();
        
        stats.bytes_sent = 100;
        stats.bytes_received = 200;
        stats.send_errors = 1;
        stats.receive_errors = 2;
        
        assert_eq!(stats.total_bytes(), 300);
        assert_eq!(stats.total_errors(), 3);
        assert_eq!(stats.error_rate(), 1.0);
    }
    
    #[test]
    fn test_presets() {
        assert_eq!(presets::STANDARD_115200.baudrate, Baudrate::Bps115200);
        assert_eq!(presets::HIGH_SPEED_921600.baudrate, Baudrate::Bps921600);
        assert_eq!(presets::LOW_POWER_9600.baudrate, Baudrate::Bps9600);
        assert_eq!(presets::MODBUS_RTU.parity, Parity::Even);
    }
}