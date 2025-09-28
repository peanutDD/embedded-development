//! # 错误处理模块
//!
//! 定义了异步UART通信中可能出现的各种错误类型。

use core::fmt;

/// UART错误类型
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UartError {
  /// 配置错误
  Config(crate::config::ConfigError),
  /// 硬件错误
  Hardware(HardwareError),
  /// 缓冲区错误
  Buffer(BufferError),
  /// 超时错误
  Timeout(TimeoutError),
  /// DMA错误
  Dma(DmaError),
  /// 协议错误
  Protocol(ProtocolError),
  /// IO错误
  Io(IoError),
  /// 状态错误
  State(StateError),
  /// 资源错误
  Resource(ResourceError),
  /// 中断错误
  Interrupt(InterruptError),
}

impl fmt::Display for UartError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      UartError::Config(e) => write!(f, "Configuration error: {}", e),
      UartError::Hardware(e) => write!(f, "Hardware error: {}", e),
      UartError::Buffer(e) => write!(f, "Buffer error: {}", e),
      UartError::Timeout(e) => write!(f, "Timeout error: {}", e),
      UartError::Dma(e) => write!(f, "DMA error: {}", e),
      UartError::Protocol(e) => write!(f, "Protocol error: {}", e),
      UartError::Io(e) => write!(f, "IO error: {}", e),
      UartError::State(e) => write!(f, "State error: {}", e),
      UartError::Resource(e) => write!(f, "Resource error: {}", e),
      UartError::Interrupt(e) => write!(f, "Interrupt error: {}", e),
    }
  }
}

impl From<crate::config::ConfigError> for UartError {
  fn from(error: crate::config::ConfigError) -> Self {
    UartError::Config(error)
  }
}

impl From<HardwareError> for UartError {
  fn from(error: HardwareError) -> Self {
    UartError::Hardware(error)
  }
}

impl From<BufferError> for UartError {
  fn from(error: BufferError) -> Self {
    UartError::Buffer(error)
  }
}

impl From<TimeoutError> for UartError {
  fn from(error: TimeoutError) -> Self {
    UartError::Timeout(error)
  }
}

impl From<DmaError> for UartError {
  fn from(error: DmaError) -> Self {
    UartError::Dma(error)
  }
}

impl From<ProtocolError> for UartError {
  fn from(error: ProtocolError) -> Self {
    UartError::Protocol(error)
  }
}

impl From<IoError> for UartError {
  fn from(error: IoError) -> Self {
    UartError::Io(error)
  }
}

impl From<StateError> for UartError {
  fn from(error: StateError) -> Self {
    UartError::State(error)
  }
}

impl From<ResourceError> for UartError {
  fn from(error: ResourceError) -> Self {
    UartError::Resource(error)
  }
}

impl From<InterruptError> for UartError {
  fn from(error: InterruptError) -> Self {
    UartError::Interrupt(error)
  }
}

/// 硬件错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareError {
  /// 奇偶校验错误
  ParityError,
  /// 帧错误
  FramingError,
  /// 噪声错误
  NoiseError,
  /// 溢出错误
  OverrunError,
  /// 欠载错误
  UnderrunError,
  /// 线路中断
  LineBreak,
  /// 硬件未就绪
  NotReady,
  /// 硬件故障
  HardwareFault,
  /// 时钟错误
  ClockError,
  /// 电源错误
  PowerError,
  /// 引脚配置错误
  PinConfigError,
  /// 不支持的操作
  UnsupportedOperation,
}

impl fmt::Display for HardwareError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      HardwareError::ParityError => write!(f, "Parity error"),
      HardwareError::FramingError => write!(f, "Framing error"),
      HardwareError::NoiseError => write!(f, "Noise error"),
      HardwareError::OverrunError => write!(f, "Overrun error"),
      HardwareError::UnderrunError => write!(f, "Underrun error"),
      HardwareError::LineBreak => write!(f, "Line break detected"),
      HardwareError::NotReady => write!(f, "Hardware not ready"),
      HardwareError::HardwareFault => write!(f, "Hardware fault"),
      HardwareError::ClockError => write!(f, "Clock error"),
      HardwareError::PowerError => write!(f, "Power error"),
      HardwareError::PinConfigError => write!(f, "Pin configuration error"),
      HardwareError::UnsupportedOperation => write!(f, "Unsupported operation"),
    }
  }
}

/// 缓冲区错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferError {
  /// 缓冲区已满
  Full,
  /// 缓冲区为空
  Empty,
  /// 缓冲区溢出
  Overflow,
  /// 缓冲区欠载
  Underflow,
  /// 无效的缓冲区大小
  InvalidSize,
  /// 缓冲区未对齐
  Misaligned,
  /// 缓冲区已损坏
  Corrupted,
  /// 内存不足
  OutOfMemory,
  /// 缓冲区正在使用中
  InUse,
  /// 缓冲区未初始化
  Uninitialized,
}

impl fmt::Display for BufferError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      BufferError::Full => write!(f, "Buffer is full"),
      BufferError::Empty => write!(f, "Buffer is empty"),
      BufferError::Overflow => write!(f, "Buffer overflow"),
      BufferError::Underflow => write!(f, "Buffer underflow"),
      BufferError::InvalidSize => write!(f, "Invalid buffer size"),
      BufferError::Misaligned => write!(f, "Buffer is misaligned"),
      BufferError::Corrupted => write!(f, "Buffer is corrupted"),
      BufferError::OutOfMemory => write!(f, "Out of memory"),
      BufferError::InUse => write!(f, "Buffer is in use"),
      BufferError::Uninitialized => write!(f, "Buffer is uninitialized"),
    }
  }
}

/// 超时错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeoutError {
  /// 读取超时
  ReadTimeout,
  /// 写入超时
  WriteTimeout,
  /// 连接超时
  ConnectTimeout,
  /// 响应超时
  ResponseTimeout,
  /// 同步超时
  SyncTimeout,
  /// 握手超时
  HandshakeTimeout,
  /// 流控制超时
  FlowControlTimeout,
  /// 操作超时
  OperationTimeout,
}

impl fmt::Display for TimeoutError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      TimeoutError::ReadTimeout => write!(f, "Read operation timed out"),
      TimeoutError::WriteTimeout => write!(f, "Write operation timed out"),
      TimeoutError::ConnectTimeout => write!(f, "Connection timed out"),
      TimeoutError::ResponseTimeout => write!(f, "Response timed out"),
      TimeoutError::SyncTimeout => write!(f, "Synchronization timed out"),
      TimeoutError::HandshakeTimeout => write!(f, "Handshake timed out"),
      TimeoutError::FlowControlTimeout => write!(f, "Flow control timed out"),
      TimeoutError::OperationTimeout => write!(f, "Operation timed out"),
    }
  }
}

/// DMA错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaError {
  /// DMA传输错误
  TransferError,
  /// DMA配置错误
  ConfigError,
  /// DMA通道忙
  ChannelBusy,
  /// DMA通道不可用
  ChannelUnavailable,
  /// DMA地址错误
  AddressError,
  /// DMA大小错误
  SizeError,
  /// DMA对齐错误
  AlignmentError,
  /// DMA权限错误
  PermissionError,
  /// DMA硬件错误
  HardwareError,
  /// DMA中断错误
  InterruptError,
  /// DMA超时
  Timeout,
  /// DMA未初始化
  Uninitialized,
}

impl fmt::Display for DmaError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      DmaError::TransferError => write!(f, "DMA transfer error"),
      DmaError::ConfigError => write!(f, "DMA configuration error"),
      DmaError::ChannelBusy => write!(f, "DMA channel is busy"),
      DmaError::ChannelUnavailable => write!(f, "DMA channel is unavailable"),
      DmaError::AddressError => write!(f, "DMA address error"),
      DmaError::SizeError => write!(f, "DMA size error"),
      DmaError::AlignmentError => write!(f, "DMA alignment error"),
      DmaError::PermissionError => write!(f, "DMA permission error"),
      DmaError::HardwareError => write!(f, "DMA hardware error"),
      DmaError::InterruptError => write!(f, "DMA interrupt error"),
      DmaError::Timeout => write!(f, "DMA timeout"),
      DmaError::Uninitialized => write!(f, "DMA is uninitialized"),
    }
  }
}

/// 协议错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolError {
  /// 无效的帧格式
  InvalidFrame,
  /// 校验和错误
  ChecksumError,
  /// CRC错误
  CrcError,
  /// 序列号错误
  SequenceError,
  /// 协议版本不匹配
  VersionMismatch,
  /// 无效的命令
  InvalidCommand,
  /// 无效的参数
  InvalidParameter,
  /// 协议状态错误
  StateError,
  /// 协议超时
  Timeout,
  /// 协议同步丢失
  SyncLost,
  /// 数据长度错误
  LengthError,
  /// 编码错误
  EncodingError,
}

impl fmt::Display for ProtocolError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      ProtocolError::InvalidFrame => write!(f, "Invalid frame format"),
      ProtocolError::ChecksumError => write!(f, "Checksum error"),
      ProtocolError::CrcError => write!(f, "CRC error"),
      ProtocolError::SequenceError => write!(f, "Sequence error"),
      ProtocolError::VersionMismatch => write!(f, "Protocol version mismatch"),
      ProtocolError::InvalidCommand => write!(f, "Invalid command"),
      ProtocolError::InvalidParameter => write!(f, "Invalid parameter"),
      ProtocolError::StateError => write!(f, "Protocol state error"),
      ProtocolError::Timeout => write!(f, "Protocol timeout"),
      ProtocolError::SyncLost => write!(f, "Protocol synchronization lost"),
      ProtocolError::LengthError => write!(f, "Data length error"),
      ProtocolError::EncodingError => write!(f, "Encoding error"),
    }
  }
}

/// IO错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IoError {
  /// 连接中断
  ConnectionAborted,
  /// 连接重置
  ConnectionReset,
  /// 连接拒绝
  ConnectionRefused,
  /// 网络不可达
  NetworkUnreachable,
  /// 主机不可达
  HostUnreachable,
  /// 地址已在使用
  AddressInUse,
  /// 地址不可用
  AddressNotAvailable,
  /// 权限被拒绝
  PermissionDenied,
  /// 资源暂时不可用
  WouldBlock,
  /// 操作被中断
  Interrupted,
  /// 无效的输入
  InvalidInput,
  /// 无效的数据
  InvalidData,
  /// 意外的文件结束
  UnexpectedEof,
  /// 写入零字节
  WriteZero,
  /// 其他错误
  Other,
}

impl fmt::Display for IoError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      IoError::ConnectionAborted => write!(f, "Connection aborted"),
      IoError::ConnectionReset => write!(f, "Connection reset"),
      IoError::ConnectionRefused => write!(f, "Connection refused"),
      IoError::NetworkUnreachable => write!(f, "Network unreachable"),
      IoError::HostUnreachable => write!(f, "Host unreachable"),
      IoError::AddressInUse => write!(f, "Address in use"),
      IoError::AddressNotAvailable => write!(f, "Address not available"),
      IoError::PermissionDenied => write!(f, "Permission denied"),
      IoError::WouldBlock => write!(f, "Resource temporarily unavailable"),
      IoError::Interrupted => write!(f, "Operation interrupted"),
      IoError::InvalidInput => write!(f, "Invalid input"),
      IoError::InvalidData => write!(f, "Invalid data"),
      IoError::UnexpectedEof => write!(f, "Unexpected end of file"),
      IoError::WriteZero => write!(f, "Write zero bytes"),
      IoError::Other => write!(f, "Other IO error"),
    }
  }
}

/// 状态错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateError {
  /// 无效状态
  InvalidState,
  /// 状态转换错误
  TransitionError,
  /// 未初始化
  Uninitialized,
  /// 已初始化
  AlreadyInitialized,
  /// 未连接
  NotConnected,
  /// 已连接
  AlreadyConnected,
  /// 未启动
  NotStarted,
  /// 已启动
  AlreadyStarted,
  /// 未停止
  NotStopped,
  /// 已停止
  AlreadyStopped,
  /// 正在使用中
  InUse,
  /// 已关闭
  Closed,
  /// 状态冲突
  StateConflict,
}

impl fmt::Display for StateError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      StateError::InvalidState => write!(f, "Invalid state"),
      StateError::TransitionError => write!(f, "State transition error"),
      StateError::Uninitialized => write!(f, "Not initialized"),
      StateError::AlreadyInitialized => write!(f, "Already initialized"),
      StateError::NotConnected => write!(f, "Not connected"),
      StateError::AlreadyConnected => write!(f, "Already connected"),
      StateError::NotStarted => write!(f, "Not started"),
      StateError::AlreadyStarted => write!(f, "Already started"),
      StateError::NotStopped => write!(f, "Not stopped"),
      StateError::AlreadyStopped => write!(f, "Already stopped"),
      StateError::InUse => write!(f, "In use"),
      StateError::Closed => write!(f, "Closed"),
      StateError::StateConflict => write!(f, "State conflict"),
    }
  }
}

/// 资源错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceError {
  /// 资源不足
  Insufficient,
  /// 资源不可用
  Unavailable,
  /// 资源已耗尽
  Exhausted,
  /// 资源冲突
  Conflict,
  /// 资源泄漏
  Leak,
  /// 资源损坏
  Corrupted,
  /// 资源锁定
  Locked,
  /// 资源未找到
  NotFound,
  /// 资源访问被拒绝
  AccessDenied,
  /// 资源配额超限
  QuotaExceeded,
  /// 资源版本不匹配
  VersionMismatch,
  /// 资源依赖缺失
  DependencyMissing,
}

impl fmt::Display for ResourceError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      ResourceError::Insufficient => write!(f, "Insufficient resources"),
      ResourceError::Unavailable => write!(f, "Resource unavailable"),
      ResourceError::Exhausted => write!(f, "Resources exhausted"),
      ResourceError::Conflict => write!(f, "Resource conflict"),
      ResourceError::Leak => write!(f, "Resource leak"),
      ResourceError::Corrupted => write!(f, "Resource corrupted"),
      ResourceError::Locked => write!(f, "Resource locked"),
      ResourceError::NotFound => write!(f, "Resource not found"),
      ResourceError::AccessDenied => write!(f, "Resource access denied"),
      ResourceError::QuotaExceeded => write!(f, "Resource quota exceeded"),
      ResourceError::VersionMismatch => write!(f, "Resource version mismatch"),
      ResourceError::DependencyMissing => write!(f, "Resource dependency missing"),
    }
  }
}

/// 中断错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptError {
  /// 中断未启用
  NotEnabled,
  /// 中断已启用
  AlreadyEnabled,
  /// 中断处理程序未设置
  HandlerNotSet,
  /// 中断处理程序已设置
  HandlerAlreadySet,
  /// 中断优先级错误
  InvalidPriority,
  /// 中断向量错误
  InvalidVector,
  /// 中断嵌套错误
  NestingError,
  /// 中断上下文错误
  ContextError,
  /// 中断延迟过长
  LatencyTooHigh,
  /// 中断丢失
  Lost,
  /// 中断风暴
  Storm,
  /// 中断硬件错误
  HardwareError,
}

impl fmt::Display for InterruptError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      InterruptError::NotEnabled => write!(f, "Interrupt not enabled"),
      InterruptError::AlreadyEnabled => write!(f, "Interrupt already enabled"),
      InterruptError::HandlerNotSet => write!(f, "Interrupt handler not set"),
      InterruptError::HandlerAlreadySet => write!(f, "Interrupt handler already set"),
      InterruptError::InvalidPriority => write!(f, "Invalid interrupt priority"),
      InterruptError::InvalidVector => write!(f, "Invalid interrupt vector"),
      InterruptError::NestingError => write!(f, "Interrupt nesting error"),
      InterruptError::ContextError => write!(f, "Interrupt context error"),
      InterruptError::LatencyTooHigh => write!(f, "Interrupt latency too high"),
      InterruptError::Lost => write!(f, "Interrupt lost"),
      InterruptError::Storm => write!(f, "Interrupt storm"),
      InterruptError::HardwareError => write!(f, "Interrupt hardware error"),
    }
  }
}

/// 错误严重程度
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ErrorSeverity {
  /// 信息
  Info,
  /// 警告
  Warning,
  /// 错误
  Error,
  /// 严重错误
  Critical,
  /// 致命错误
  Fatal,
}

impl fmt::Display for ErrorSeverity {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      ErrorSeverity::Info => write!(f, "INFO"),
      ErrorSeverity::Warning => write!(f, "WARNING"),
      ErrorSeverity::Error => write!(f, "ERROR"),
      ErrorSeverity::Critical => write!(f, "CRITICAL"),
      ErrorSeverity::Fatal => write!(f, "FATAL"),
    }
  }
}

impl UartError {
  /// 获取错误严重程度
  pub fn severity(&self) -> ErrorSeverity {
    match self {
      UartError::Config(_) => ErrorSeverity::Error,
      UartError::Hardware(e) => match e {
        HardwareError::ParityError | HardwareError::FramingError | HardwareError::NoiseError => {
          ErrorSeverity::Warning
        }
        HardwareError::OverrunError | HardwareError::UnderrunError => ErrorSeverity::Error,
        HardwareError::HardwareFault | HardwareError::PowerError => ErrorSeverity::Critical,
        _ => ErrorSeverity::Error,
      },
      UartError::Buffer(e) => match e {
        BufferError::Full | BufferError::Empty => ErrorSeverity::Warning,
        BufferError::Overflow | BufferError::Underflow => ErrorSeverity::Error,
        BufferError::Corrupted | BufferError::OutOfMemory => ErrorSeverity::Critical,
        _ => ErrorSeverity::Error,
      },
      UartError::Timeout(_) => ErrorSeverity::Warning,
      UartError::Dma(e) => match e {
        DmaError::ChannelBusy => ErrorSeverity::Warning,
        DmaError::HardwareError => ErrorSeverity::Critical,
        _ => ErrorSeverity::Error,
      },
      UartError::Protocol(_) => ErrorSeverity::Error,
      UartError::Io(e) => match e {
        IoError::WouldBlock | IoError::Interrupted => ErrorSeverity::Info,
        IoError::ConnectionAborted | IoError::ConnectionReset => ErrorSeverity::Warning,
        _ => ErrorSeverity::Error,
      },
      UartError::State(_) => ErrorSeverity::Error,
      UartError::Resource(e) => match e {
        ResourceError::Unavailable => ErrorSeverity::Warning,
        ResourceError::Exhausted | ResourceError::Corrupted => ErrorSeverity::Critical,
        _ => ErrorSeverity::Error,
      },
      UartError::Interrupt(e) => match e {
        InterruptError::LatencyTooHigh => ErrorSeverity::Warning,
        InterruptError::Storm | InterruptError::HardwareError => ErrorSeverity::Critical,
        _ => ErrorSeverity::Error,
      },
    }
  }

  /// 检查错误是否可恢复
  pub fn is_recoverable(&self) -> bool {
    match self {
      UartError::Config(_) => false,
      UartError::Hardware(e) => match e {
        HardwareError::ParityError | HardwareError::FramingError | HardwareError::NoiseError => {
          true
        }
        HardwareError::OverrunError | HardwareError::UnderrunError => true,
        HardwareError::HardwareFault | HardwareError::PowerError => false,
        _ => true,
      },
      UartError::Buffer(e) => match e {
        BufferError::Full | BufferError::Empty => true,
        BufferError::Corrupted | BufferError::OutOfMemory => false,
        _ => true,
      },
      UartError::Timeout(_) => true,
      UartError::Dma(e) => match e {
        DmaError::HardwareError => false,
        _ => true,
      },
      UartError::Protocol(_) => true,
      UartError::Io(e) => match e {
        IoError::PermissionDenied => false,
        _ => true,
      },
      UartError::State(_) => true,
      UartError::Resource(e) => match e {
        ResourceError::Corrupted => false,
        _ => true,
      },
      UartError::Interrupt(e) => match e {
        InterruptError::HardwareError => false,
        _ => true,
      },
    }
  }

  /// 检查错误是否需要重试
  pub fn should_retry(&self) -> bool {
    match self {
      UartError::Timeout(_) => true,
      UartError::Buffer(BufferError::Full) => true,
      UartError::Dma(DmaError::ChannelBusy) => true,
      UartError::Io(IoError::WouldBlock) => true,
      UartError::Resource(ResourceError::Unavailable) => true,
      _ => false,
    }
  }

  /// 获取错误代码
  pub fn error_code(&self) -> u32 {
    match self {
      UartError::Config(e) => 0x1000 + (*e as u32),
      UartError::Hardware(e) => 0x2000 + (*e as u32),
      UartError::Buffer(e) => 0x3000 + (*e as u32),
      UartError::Timeout(e) => 0x4000 + (*e as u32),
      UartError::Dma(e) => 0x5000 + (*e as u32),
      UartError::Protocol(e) => 0x6000 + (*e as u32),
      UartError::Io(e) => 0x7000 + (*e as u32),
      UartError::State(e) => 0x8000 + (*e as u32),
      UartError::Resource(e) => 0x9000 + (*e as u32),
      UartError::Interrupt(e) => 0xA000 + (*e as u32),
    }
  }
}

/// 结果类型别名
pub type Result<T> = core::result::Result<T, UartError>;

/// 错误上下文信息
#[derive(Debug, Clone)]
pub struct ErrorContext {
  /// 错误发生的位置
  pub location: &'static str,
  /// 错误发生的时间戳
  pub timestamp: u64,
  /// 附加信息
  pub details: Option<&'static str>,
  /// 错误计数
  pub count: u32,
}

impl ErrorContext {
  /// 创建新的错误上下文
  pub fn new(location: &'static str) -> Self {
    Self {
      location,
      timestamp: 0, // 在实际实现中应该使用系统时间
      details: None,
      count: 1,
    }
  }

  /// 添加详细信息
  pub fn with_details(mut self, details: &'static str) -> Self {
    self.details = Some(details);
    self
  }

  /// 增加错误计数
  pub fn increment_count(&mut self) {
    self.count += 1;
  }
}

/// 带上下文的错误
#[derive(Debug, Clone)]
pub struct ContextualError {
  /// 错误
  pub error: UartError,
  /// 上下文
  pub context: ErrorContext,
}

impl ContextualError {
  /// 创建带上下文的错误
  pub fn new(error: UartError, context: ErrorContext) -> Self {
    Self { error, context }
  }
}

impl fmt::Display for ContextualError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "{} at {} (count: {})",
      self.error, self.context.location, self.context.count
    )?;

    if let Some(details) = self.context.details {
      write!(f, " - {}", details)?;
    }

    Ok(())
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_error_display() {
    let error = UartError::Hardware(HardwareError::ParityError);
    assert_eq!(error.to_string(), "Hardware error: Parity error");

    let error = UartError::Buffer(BufferError::Full);
    assert_eq!(error.to_string(), "Buffer error: Buffer is full");
  }

  #[test]
  fn test_error_conversion() {
    let config_error = crate::config::ConfigError::InvalidBufferSize;
    let uart_error: UartError = config_error.into();
    assert!(matches!(uart_error, UartError::Config(_)));

    let hardware_error = HardwareError::ParityError;
    let uart_error: UartError = hardware_error.into();
    assert!(matches!(uart_error, UartError::Hardware(_)));
  }

  #[test]
  fn test_error_severity() {
    let error = UartError::Hardware(HardwareError::ParityError);
    assert_eq!(error.severity(), ErrorSeverity::Warning);

    let error = UartError::Hardware(HardwareError::HardwareFault);
    assert_eq!(error.severity(), ErrorSeverity::Critical);

    let error = UartError::Timeout(TimeoutError::ReadTimeout);
    assert_eq!(error.severity(), ErrorSeverity::Warning);
  }

  #[test]
  fn test_error_recoverability() {
    let error = UartError::Hardware(HardwareError::ParityError);
    assert!(error.is_recoverable());

    let error = UartError::Hardware(HardwareError::HardwareFault);
    assert!(!error.is_recoverable());

    let error = UartError::Config(crate::config::ConfigError::InvalidBufferSize);
    assert!(!error.is_recoverable());
  }

  #[test]
  fn test_error_retry() {
    let error = UartError::Timeout(TimeoutError::ReadTimeout);
    assert!(error.should_retry());

    let error = UartError::Buffer(BufferError::Full);
    assert!(error.should_retry());

    let error = UartError::Hardware(HardwareError::HardwareFault);
    assert!(!error.should_retry());
  }

  #[test]
  fn test_error_code() {
    let error = UartError::Hardware(HardwareError::ParityError);
    assert_eq!(error.error_code(), 0x2000);

    let error = UartError::Buffer(BufferError::Full);
    assert_eq!(error.error_code(), 0x3000);
  }

  #[test]
  fn test_error_context() {
    let mut context = ErrorContext::new("test_function").with_details("Test error details");

    assert_eq!(context.location, "test_function");
    assert_eq!(context.count, 1);
    assert_eq!(context.details, Some("Test error details"));

    context.increment_count();
    assert_eq!(context.count, 2);
  }

  #[test]
  fn test_contextual_error() {
    let error = UartError::Hardware(HardwareError::ParityError);
    let context = ErrorContext::new("uart_read").with_details("Parity check failed");
    let contextual_error = ContextualError::new(error, context);

    let error_string = contextual_error.to_string();
    assert!(error_string.contains("Hardware error: Parity error"));
    assert!(error_string.contains("uart_read"));
    assert!(error_string.contains("count: 1"));
    assert!(error_string.contains("Parity check failed"));
  }
}
