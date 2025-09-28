//! # HAL错误处理模块
//!
//! 定义了HAL层的错误类型和错误处理策略

use core::fmt;

/// HAL主错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HalError {
  /// 硬件相关错误
  Hardware(HardwareError),
  /// 配置错误
  Configuration(ConfigError),
  /// 资源错误
  Resource(ResourceError),
  /// 时序错误
  Timing(TimingError),
}

/// 硬件错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareError {
  /// 寄存器访问错误
  RegisterAccess,
  /// 时钟未启用
  ClockNotEnabled,
  /// 引脚被锁定
  PinLocked,
  /// 硬件故障
  HardwareFault,
  /// 电源问题
  PowerIssue,
}

/// 配置错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
  /// 无效的模式配置
  InvalidMode,
  /// 不支持的功能
  UnsupportedFeature,
  /// 配置冲突
  ConflictingConfiguration,
  /// 无效的参数
  InvalidParameter,
  /// 配置超出范围
  OutOfRange,
}

/// 资源错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceError {
  /// 引脚已被占用
  PinAlreadyTaken,
  /// 资源不足
  InsufficientResources,
  /// 资源忙碌
  ResourceBusy,
  /// 资源未找到
  ResourceNotFound,
  /// 权限不足
  PermissionDenied,
}

/// 时序错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimingError {
  /// 操作超时
  Timeout,
  /// 建立时间违反
  SetupViolation,
  /// 保持时间违反
  HoldViolation,
  /// 时钟抖动过大
  ClockJitter,
  /// 频率超出范围
  FrequencyOutOfRange,
}

impl fmt::Display for HalError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      HalError::Hardware(err) => write!(f, "硬件错误: {}", err),
      HalError::Configuration(err) => write!(f, "配置错误: {}", err),
      HalError::Resource(err) => write!(f, "资源错误: {}", err),
      HalError::Timing(err) => write!(f, "时序错误: {}", err),
    }
  }
}

impl fmt::Display for HardwareError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      HardwareError::RegisterAccess => write!(f, "寄存器访问失败"),
      HardwareError::ClockNotEnabled => write!(f, "时钟未启用"),
      HardwareError::PinLocked => write!(f, "引脚被锁定"),
      HardwareError::HardwareFault => write!(f, "硬件故障"),
      HardwareError::PowerIssue => write!(f, "电源问题"),
    }
  }
}

impl fmt::Display for ConfigError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      ConfigError::InvalidMode => write!(f, "无效的模式配置"),
      ConfigError::UnsupportedFeature => write!(f, "不支持的功能"),
      ConfigError::ConflictingConfiguration => write!(f, "配置冲突"),
      ConfigError::InvalidParameter => write!(f, "无效的参数"),
      ConfigError::OutOfRange => write!(f, "参数超出范围"),
    }
  }
}

impl fmt::Display for ResourceError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      ResourceError::PinAlreadyTaken => write!(f, "引脚已被占用"),
      ResourceError::InsufficientResources => write!(f, "资源不足"),
      ResourceError::ResourceBusy => write!(f, "资源忙碌"),
      ResourceError::ResourceNotFound => write!(f, "资源未找到"),
      ResourceError::PermissionDenied => write!(f, "权限不足"),
    }
  }
}

impl fmt::Display for TimingError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      TimingError::Timeout => write!(f, "操作超时"),
      TimingError::SetupViolation => write!(f, "建立时间违反"),
      TimingError::HoldViolation => write!(f, "保持时间违反"),
      TimingError::ClockJitter => write!(f, "时钟抖动过大"),
      TimingError::FrequencyOutOfRange => write!(f, "频率超出范围"),
    }
  }
}

/// 错误恢复特征
pub trait ErrorRecovery {
  /// 检查错误是否可以恢复
  fn can_recover(&self) -> bool;

  /// 尝试从错误中恢复
  fn attempt_recovery(&mut self) -> Result<(), HalError>;

  /// 获取恢复建议
  fn recovery_suggestion(&self) -> &'static str;
}

impl ErrorRecovery for HalError {
  fn can_recover(&self) -> bool {
    match self {
      HalError::Hardware(hw_err) => match hw_err {
        HardwareError::RegisterAccess => true,  // 可以重试
        HardwareError::ClockNotEnabled => true, // 可以启用时钟
        HardwareError::PinLocked => false,      // 需要重置
        HardwareError::HardwareFault => false,  // 硬件故障
        HardwareError::PowerIssue => false,     // 电源问题
      },
      HalError::Configuration(cfg_err) => match cfg_err {
        ConfigError::InvalidMode => true,              // 可以重新配置
        ConfigError::UnsupportedFeature => false,      // 功能不支持
        ConfigError::ConflictingConfiguration => true, // 可以解决冲突
        ConfigError::InvalidParameter => true,         // 可以修正参数
        ConfigError::OutOfRange => true,               // 可以调整范围
      },
      HalError::Resource(res_err) => match res_err {
        ResourceError::PinAlreadyTaken => false, // 需要释放资源
        ResourceError::InsufficientResources => false, // 资源不足
        ResourceError::ResourceBusy => true,     // 可以等待
        ResourceError::ResourceNotFound => false, // 资源不存在
        ResourceError::PermissionDenied => false, // 权限问题
      },
      HalError::Timing(tim_err) => match tim_err {
        TimingError::Timeout => true,             // 可以重试
        TimingError::SetupViolation => true,      // 可以调整时序
        TimingError::HoldViolation => true,       // 可以调整时序
        TimingError::ClockJitter => true,         // 可以调整时钟
        TimingError::FrequencyOutOfRange => true, // 可以调整频率
      },
    }
  }

  fn attempt_recovery(&mut self) -> Result<(), HalError> {
    match self {
      HalError::Hardware(HardwareError::RegisterAccess) => {
        // 尝试重新访问寄存器
        log::info!("尝试恢复寄存器访问");
        Ok(())
      }
      HalError::Hardware(HardwareError::ClockNotEnabled) => {
        // 尝试启用时钟
        log::info!("尝试启用时钟");
        Ok(())
      }
      HalError::Configuration(_) => {
        // 重置配置到默认状态
        log::info!("重置配置到默认状态");
        Ok(())
      }
      HalError::Resource(ResourceError::ResourceBusy) => {
        // 等待资源可用
        log::info!("等待资源可用");
        Ok(())
      }
      HalError::Timing(_) => {
        // 调整时序参数
        log::info!("调整时序参数");
        Ok(())
      }
      _ => Err(*self),
    }
  }

  fn recovery_suggestion(&self) -> &'static str {
    match self {
      HalError::Hardware(hw_err) => match hw_err {
        HardwareError::RegisterAccess => "检查硬件连接和电源",
        HardwareError::ClockNotEnabled => "启用相应的时钟域",
        HardwareError::PinLocked => "执行系统重置",
        HardwareError::HardwareFault => "检查硬件是否损坏",
        HardwareError::PowerIssue => "检查电源供应",
      },
      HalError::Configuration(cfg_err) => match cfg_err {
        ConfigError::InvalidMode => "使用有效的模式配置",
        ConfigError::UnsupportedFeature => "使用支持的功能",
        ConfigError::ConflictingConfiguration => "解决配置冲突",
        ConfigError::InvalidParameter => "使用有效的参数值",
        ConfigError::OutOfRange => "使用范围内的参数值",
      },
      HalError::Resource(res_err) => match res_err {
        ResourceError::PinAlreadyTaken => "释放已占用的引脚",
        ResourceError::InsufficientResources => "释放不需要的资源",
        ResourceError::ResourceBusy => "等待资源可用",
        ResourceError::ResourceNotFound => "检查资源是否存在",
        ResourceError::PermissionDenied => "检查访问权限",
      },
      HalError::Timing(tim_err) => match tim_err {
        TimingError::Timeout => "增加超时时间或检查硬件响应",
        TimingError::SetupViolation => "增加建立时间",
        TimingError::HoldViolation => "增加保持时间",
        TimingError::ClockJitter => "使用更稳定的时钟源",
        TimingError::FrequencyOutOfRange => "调整到支持的频率范围",
      },
    }
  }
}

/// 错误统计器
pub struct ErrorStatistics {
  hardware_errors: u32,
  config_errors: u32,
  resource_errors: u32,
  timing_errors: u32,
  total_errors: u32,
  recovery_attempts: u32,
  successful_recoveries: u32,
}

impl ErrorStatistics {
  /// 创建新的错误统计器
  pub fn new() -> Self {
    Self {
      hardware_errors: 0,
      config_errors: 0,
      resource_errors: 0,
      timing_errors: 0,
      total_errors: 0,
      recovery_attempts: 0,
      successful_recoveries: 0,
    }
  }

  /// 记录错误
  pub fn record_error(&mut self, error: &HalError) {
    self.total_errors += 1;

    match error {
      HalError::Hardware(_) => self.hardware_errors += 1,
      HalError::Configuration(_) => self.config_errors += 1,
      HalError::Resource(_) => self.resource_errors += 1,
      HalError::Timing(_) => self.timing_errors += 1,
    }
  }

  /// 记录恢复尝试
  pub fn record_recovery_attempt(&mut self, success: bool) {
    self.recovery_attempts += 1;
    if success {
      self.successful_recoveries += 1;
    }
  }

  /// 获取错误统计信息
  pub fn get_statistics(&self) -> ErrorStatisticsReport {
    ErrorStatisticsReport {
      hardware_errors: self.hardware_errors,
      config_errors: self.config_errors,
      resource_errors: self.resource_errors,
      timing_errors: self.timing_errors,
      total_errors: self.total_errors,
      recovery_attempts: self.recovery_attempts,
      successful_recoveries: self.successful_recoveries,
      recovery_success_rate: if self.recovery_attempts > 0 {
        (self.successful_recoveries * 100) / self.recovery_attempts
      } else {
        0
      },
    }
  }

  /// 重置统计信息
  pub fn reset(&mut self) {
    *self = Self::new();
  }
}

/// 错误统计报告
#[derive(Debug, Clone, Copy)]
pub struct ErrorStatisticsReport {
  pub hardware_errors: u32,
  pub config_errors: u32,
  pub resource_errors: u32,
  pub timing_errors: u32,
  pub total_errors: u32,
  pub recovery_attempts: u32,
  pub successful_recoveries: u32,
  pub recovery_success_rate: u32, // 百分比
}

impl fmt::Display for ErrorStatisticsReport {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    writeln!(f, "错误统计报告:")?;
    writeln!(f, "  总错误数: {}", self.total_errors)?;
    writeln!(f, "  硬件错误: {}", self.hardware_errors)?;
    writeln!(f, "  配置错误: {}", self.config_errors)?;
    writeln!(f, "  资源错误: {}", self.resource_errors)?;
    writeln!(f, "  时序错误: {}", self.timing_errors)?;
    writeln!(f, "  恢复尝试: {}", self.recovery_attempts)?;
    writeln!(f, "  成功恢复: {}", self.successful_recoveries)?;
    writeln!(f, "  恢复成功率: {}%", self.recovery_success_rate)?;
    Ok(())
  }
}

/// 错误处理宏
#[macro_export]
macro_rules! handle_error {
  ($result:expr, $error_stats:expr) => {
    match $result {
      Ok(value) => Ok(value),
      Err(mut error) => {
        $error_stats.record_error(&error);

        if error.can_recover() {
          match error.attempt_recovery() {
            Ok(()) => {
              $error_stats.record_recovery_attempt(true);
              log::info!("错误恢复成功: {}", error.recovery_suggestion());
              // 重试原操作
              $result
            }
            Err(recovery_error) => {
              $error_stats.record_recovery_attempt(false);
              log::error!("错误恢复失败: {}", recovery_error);
              Err(error)
            }
          }
        } else {
          log::error!("无法恢复的错误: {}", error.recovery_suggestion());
          Err(error)
        }
      }
    }
  };
}

/// 错误日志记录函数
pub fn log_error(error: &HalError, context: &str) {
  match error {
    HalError::Hardware(hw_err) => {
      log::error!(
        "[{}] 硬件错误: {} - {}",
        context,
        hw_err,
        error.recovery_suggestion()
      );
    }
    HalError::Configuration(cfg_err) => {
      log::warn!(
        "[{}] 配置错误: {} - {}",
        context,
        cfg_err,
        error.recovery_suggestion()
      );
    }
    HalError::Resource(res_err) => {
      log::info!(
        "[{}] 资源错误: {} - {}",
        context,
        res_err,
        error.recovery_suggestion()
      );
    }
    HalError::Timing(tim_err) => {
      log::error!(
        "[{}] 时序错误: {} - {}",
        context,
        tim_err,
        error.recovery_suggestion()
      );
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_error_recovery() {
    let mut error = HalError::Hardware(HardwareError::RegisterAccess);
    assert!(error.can_recover());
    assert!(error.attempt_recovery().is_ok());

    let error = HalError::Hardware(HardwareError::HardwareFault);
    assert!(!error.can_recover());
  }

  #[test]
  fn test_error_statistics() {
    let mut stats = ErrorStatistics::new();

    let error1 = HalError::Hardware(HardwareError::RegisterAccess);
    let error2 = HalError::Configuration(ConfigError::InvalidMode);

    stats.record_error(&error1);
    stats.record_error(&error2);
    stats.record_recovery_attempt(true);
    stats.record_recovery_attempt(false);

    let report = stats.get_statistics();
    assert_eq!(report.total_errors, 2);
    assert_eq!(report.hardware_errors, 1);
    assert_eq!(report.config_errors, 1);
    assert_eq!(report.recovery_attempts, 2);
    assert_eq!(report.successful_recoveries, 1);
    assert_eq!(report.recovery_success_rate, 50);
  }

  #[test]
  fn test_error_display() {
    let error = HalError::Hardware(HardwareError::RegisterAccess);
    let error_str = format!("{}", error);
    assert!(error_str.contains("硬件错误"));
    assert!(error_str.contains("寄存器访问失败"));
  }
}
