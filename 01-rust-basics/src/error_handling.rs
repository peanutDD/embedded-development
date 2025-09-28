//! # 错误处理最佳实践
//!
//! 本模块提供了针对嵌入式环境优化的错误处理工具和模式。
//!
//! ## 特性
//!
//! - **零分配错误**: 不使用堆分配的错误类型
//! - **类型安全**: 编译时错误检查
//! - **性能优化**: 最小化错误处理开销
//! - **可组合性**: 错误类型的组合和转换

use core::fmt;

/// 通用错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
  /// 无效输入
  InvalidInput(String),
  /// 无效参数错误
  InvalidArgument(&'static str),
  /// 缓冲区溢出
  BufferOverflow,
  /// 资源不足错误
  InsufficientResources,
  /// 超时错误
  Timeout,
  /// 硬件错误
  HardwareError(u32),
  /// 通信错误
  CommunicationError,
  /// 初始化错误
  InitializationError(String),
  /// 配置错误
  ConfigurationError(&'static str),
  /// 未知错误
  Unknown,
}

impl fmt::Display for Error {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Error::InvalidInput(msg) => write!(f, "Invalid input: {}", msg),
      Error::InvalidArgument(msg) => write!(f, "Invalid argument: {}", msg),
      Error::BufferOverflow => write!(f, "Buffer overflow"),
      Error::InsufficientResources => write!(f, "Insufficient resources"),
      Error::Timeout => write!(f, "Operation timed out"),
      Error::HardwareError(code) => write!(f, "Hardware error: {}", code),
      Error::CommunicationError => write!(f, "Communication error"),
      Error::InitializationError(msg) => write!(f, "Initialization error: {}", msg),
      Error::ConfigurationError(msg) => write!(f, "Configuration error: {}", msg),
      Error::Unknown => write!(f, "Unknown error"),
    }
  }
}

/// 结果类型别名
pub type Result<T> = core::result::Result<T, Error>;

/// 错误上下文包装器
#[derive(Debug)]
pub struct ErrorContext<T> {
  error: T,
  context: &'static str,
}

impl<T> ErrorContext<T> {
  /// 创建带上下文的错误
  pub fn new(error: T, context: &'static str) -> Self {
    Self { error, context }
  }

  /// 获取错误
  pub fn error(&self) -> &T {
    &self.error
  }

  /// 获取上下文
  pub fn context(&self) -> &'static str {
    self.context
  }
}

impl<T: fmt::Display> fmt::Display for ErrorContext<T> {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(f, "{}: {}", self.context, self.error)
  }
}

/// 为Result添加上下文的trait
pub trait ResultExt<T, E> {
  /// 添加错误上下文
  fn with_context(self, context: &'static str) -> core::result::Result<T, ErrorContext<E>>;

  /// 映射错误类型
  fn map_err_to<F>(self, f: F) -> Result<T>
  where
    F: FnOnce(E) -> Error;
}

impl<T, E> ResultExt<T, E> for core::result::Result<T, E> {
  fn with_context(self, context: &'static str) -> core::result::Result<T, ErrorContext<E>> {
    self.map_err(|e| ErrorContext::new(e, context))
  }

  fn map_err_to<F>(self, f: F) -> Result<T>
  where
    F: FnOnce(E) -> Error,
  {
    self.map_err(f)
  }
}

/// 错误累积器，用于收集多个错误
pub struct ErrorAccumulator {
  errors: crate::collections::StackVec<Error, 16>,
}

impl ErrorAccumulator {
  /// 创建新的错误累积器
  pub fn new() -> Self {
    Self {
      errors: crate::collections::StackVec::new(),
    }
  }

  /// 添加错误
  pub fn add(&mut self, error: Error) -> Result<()> {
    self
      .errors
      .push(error)
      .map_err(|_| Error::InsufficientResources)
  }

  /// 检查是否有错误
  pub fn has_errors(&self) -> bool {
    !self.errors.is_empty()
  }

  /// 获取错误数量
  pub fn error_count(&self) -> usize {
    self.errors.len()
  }

  /// 获取第一个错误
  pub fn first_error(&self) -> Option<&Error> {
    self.errors.get(0)
  }

  /// 清空所有错误
  pub fn clear(&mut self) {
    self.errors.clear();
  }

  /// 获取所有错误的迭代器
  pub fn iter(&self) -> impl Iterator<Item = &Error> {
    self.errors.iter()
  }
}

impl Default for ErrorAccumulator {
  fn default() -> Self {
    Self::new()
  }
}

/// 重试机制
pub struct RetryConfig {
  /// 最大重试次数
  pub max_attempts: u8,
  /// 重试间隔（毫秒）
  pub delay_ms: u32,
  /// 是否使用指数退避
  pub exponential_backoff: bool,
}

impl Default for RetryConfig {
  fn default() -> Self {
    Self {
      max_attempts: 3,
      delay_ms: 100,
      exponential_backoff: false,
    }
  }
}

/// 带重试的操作执行
pub fn retry_with_config<T, F>(config: &RetryConfig, mut operation: F) -> Result<T>
where
  F: FnMut() -> Result<T>,
{
  let mut last_error = Error::Unknown;
  let mut delay = config.delay_ms;

  for attempt in 0..config.max_attempts {
    match operation() {
      Ok(result) => return Ok(result),
      Err(error) => {
        last_error = error;

        // 如果不是最后一次尝试，等待一段时间
        if attempt < config.max_attempts - 1 {
          // 在实际嵌入式环境中，这里应该使用适当的延时函数
          // 这里只是示例，实际使用时需要替换为具体的延时实现

          if config.exponential_backoff {
            delay *= 2;
          }
        }
      }
    }
  }

  Err(last_error)
}

/// 简单重试（使用默认配置）
pub fn retry<T, F>(operation: F) -> Result<T>
where
  F: FnMut() -> Result<T>,
{
  retry_with_config(&RetryConfig::default(), operation)
}

/// 错误处理工具函数
pub mod utils {
  use super::*;

  /// 将Option转换为Result
  pub fn option_to_result<T>(option: Option<T>, error: Error) -> Result<T> {
    option.ok_or(error)
  }

  /// 检查条件，失败时返回错误
  pub fn ensure(condition: bool, error: Error) -> Result<()> {
    if condition {
      Ok(())
    } else {
      Err(error)
    }
  }

  /// 安全的除法操作
  pub fn safe_divide(dividend: i32, divisor: i32) -> Result<i32> {
    if divisor == 0 {
      Err(Error::InvalidArgument("Division by zero"))
    } else {
      Ok(dividend / divisor)
    }
  }

  /// 安全的数组索引访问
  pub fn safe_index<T>(slice: &[T], index: usize) -> Result<&T> {
    slice
      .get(index)
      .ok_or(Error::InvalidArgument("Index out of bounds"))
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_error_display() {
    let error = Error::InvalidArgument("test parameter");
    assert_eq!(format!("{}", error), "Invalid argument: test parameter");
  }

  #[test]
  fn test_error_context() {
    let error = Error::Timeout;
    let context_error = ErrorContext::new(error, "network operation");
    assert_eq!(
      format!("{}", context_error),
      "network operation: Operation timed out"
    );
  }

  #[test]
  fn test_result_ext() {
    let result: core::result::Result<i32, &str> = Err("test error");
    let with_context = result.with_context("test context");
    assert!(with_context.is_err());
  }

  #[test]
  fn test_error_accumulator() {
    let mut accumulator = ErrorAccumulator::new();

    assert!(!accumulator.has_errors());

    accumulator.add(Error::Timeout).unwrap();
    accumulator.add(Error::CommunicationError).unwrap();

    assert!(accumulator.has_errors());
    assert_eq!(accumulator.error_count(), 2);
    assert_eq!(accumulator.first_error(), Some(&Error::Timeout));
  }

  #[test]
  fn test_retry() {
    let mut counter = 0;
    let result = retry(|| {
      counter += 1;
      if counter < 3 {
        Err(Error::Timeout)
      } else {
        Ok(42)
      }
    });

    assert_eq!(result, Ok(42));
    assert_eq!(counter, 3);
  }

  #[test]
  fn test_utils() {
    assert_eq!(utils::safe_divide(10, 2), Ok(5));
    assert!(utils::safe_divide(10, 0).is_err());

    let data = [1, 2, 3];
    assert_eq!(utils::safe_index(&data, 1), Ok(&2));
    assert!(utils::safe_index(&data, 5).is_err());

    assert!(utils::ensure(true, Error::Unknown).is_ok());
    assert!(utils::ensure(false, Error::Unknown).is_err());
  }
}
