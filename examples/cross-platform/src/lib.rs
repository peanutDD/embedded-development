#![no_std]

//! 跨平台嵌入式开发抽象层
//! 
//! 提供统一的API接口，支持多种嵌入式平台

use embedded_hal::digital::{InputPin, OutputPin};
use heapless::{Vec, String};
use nb;

// 重新导出常用类型
pub use embedded_hal;
pub use heapless;

// 模块声明
pub mod traits;
pub mod gpio;
pub mod sensor;
pub mod communication;
pub mod time;
pub mod error;

// 工具模块
pub mod utils;
pub mod algorithms;
pub mod data;

// 可选功能模块
#[cfg(feature = "async")]
pub mod async_support;

#[cfg(feature = "testing")]
pub mod testing;

// 重新导出主要类型
pub use error::CrossPlatformError as Error;
pub type Result<T> = core::result::Result<T, Error>;