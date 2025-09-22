//! # GPIO HAL抽象层
//! 
//! 这个库提供了一个通用的GPIO硬件抽象层(HAL)，支持多种嵌入式平台。
//! 
//! ## 特性
//! 
//! - 类型安全的GPIO操作
//! - 零成本抽象
//! - 支持多种平台 (STM32, ESP32, RP2040等)
//! - 统一的API接口
//! - 编译时配置验证
//! - 中断处理支持
//! - 异步GPIO操作
//! 
//! ## 使用示例
//! 
//! ```rust
//! use hal_abstraction::gpio::{GpioPin, OutputPin, InputPin};
//! use hal_abstraction::platform::mock::MockPlatform;
//! 
//! // 创建模拟平台用于测试
//! let mut platform = MockPlatform::new();
//! let mut pin = platform.get_pin(5).unwrap();
//! 
//! // 配置为输出模式
//! pin.set_mode(PinMode::Output)?;
//! 
//! // 设置输出状态
//! pin.set_high()?;
//! assert!(pin.is_set_high()?);
//! ```

#![no_std]
#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod gpio;
pub mod error;
pub mod platform;
pub mod interrupt;
pub mod async_gpio;

// 重新导出核心特征
pub use gpio::{
    GpioPin, ConfigurablePin, AdvancedGpioPin,
    InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin,
    PinMode, InputMode, OutputMode, DriveStrength, SlewRate, Pull
};

pub use error::{HalError, HardwareError, ConfigError, ResourceError, TimingError};

// 平台特定重新导出
#[cfg(feature = "stm32f4")]
pub use platform::stm32::*;

#[cfg(feature = "esp32")]
pub use platform::esp32::*;

#[cfg(feature = "rp2040")]
pub use platform::rp2040::*;

#[cfg(feature = "nrf52")]
pub use platform::nrf52::*;