//! # Rust基础知识库
//! 
//! 本库提供了Rust基础概念的实用工具和示例代码，专为嵌入式开发学习设计。
//! 
//! ## 特性
//! 
//! - **零成本抽象**: 展示Rust如何在编译时优化高级抽象
//! - **内存安全**: 演示无垃圾回收的内存安全编程
//! - **性能可预测**: 提供性能分析和优化技巧
//! - **嵌入式友好**: 关注资源约束和实时性要求
//! 
//! ## 模块组织
//! 
//! - [`ownership`] - 所有权系统工具和示例
//! - [`functions`] - 函数式编程工具
//! - [`collections`] - 高效集合操作
//! - [`error_handling`] - 错误处理最佳实践
//! - [`concurrency`] - 并发编程工具
//! - [`performance`] - 性能分析和优化
//! 
//! ## 使用示例
//! 
//! ```rust
//! use rust_basics::ownership::SafeBuffer;
//! use rust_basics::error_handling::Result;
//! 
//! fn main() -> Result<()> {
//!     let buffer = SafeBuffer::new(1024)?;
//!     println!("Created buffer with {} bytes", buffer.len());
//!     Ok(())
//! }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![warn(missing_docs)]
#![warn(rust_2018_idioms)]
#![warn(clippy::all)]
#![allow(clippy::module_name_repetitions)]

// 条件编译：标准库支持
#[cfg(feature = "std")]
extern crate std;

// 核心模块
pub mod ownership;
pub mod functions;
pub mod collections;
pub mod error_handling;

// 高级模块
#[cfg(feature = "std")]
pub mod concurrency;
#[cfg(feature = "std")]
pub mod performance;

// 嵌入式专用模块
#[cfg(feature = "embedded")]
pub mod embedded;

// 重新导出常用类型
pub use error_handling::{Error, Result};

/// 库版本信息
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// 检查是否运行在嵌入式环境
#[cfg(feature = "embedded")]
pub const fn is_embedded() -> bool {
    true
}

/// 检查是否运行在标准环境
#[cfg(not(feature = "embedded"))]
pub const fn is_embedded() -> bool {
    false
}

/// 库初始化函数
#[cfg(feature = "std")]
pub fn init() -> Result<()> {
    // 初始化日志系统
    env_logger::try_init().map_err(|e| Error::InitializationError(e.to_string()))?;
    
    log::info!("Rust基础库初始化完成，版本: {}", VERSION);
    Ok(())
}

/// 嵌入式环境初始化
#[cfg(feature = "embedded")]
pub fn init_embedded() -> Result<()> {
    // 嵌入式环境的初始化逻辑
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
    }

    #[test]
    fn test_embedded_flag() {
        // 测试嵌入式标志
        #[cfg(feature = "embedded")]
        assert!(is_embedded());
        
        #[cfg(not(feature = "embedded"))]
        assert!(!is_embedded());
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_init() {
        // 测试初始化函数
        let result = init();
        // 初始化可能失败（如果已经初始化过），这是正常的
        assert!(result.is_ok() || result.is_err());
    }
}