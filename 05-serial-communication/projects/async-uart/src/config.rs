//! # UART配置模块
//!
//! 定义了UART通信的各种配置选项，包括波特率、数据位、停止位、校验位等。

use core::fmt;

/// UART配置结构体
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
  /// 波特率
  pub baudrate: Baudrate,
  /// 数据位数
  pub data_bits: DataBits,
  /// 停止位数
  pub stop_bits: StopBits,
  /// 校验位
  pub parity: Parity,
  /// 流控制
  pub flow_control: FlowControl,
  /// 缓冲区大小
  pub buffer_size: usize,
  /// 超时时间（毫秒）
  pub timeout_ms: u32,
  /// 是否启用DMA
  pub dma_enabled: bool,
}

impl Default for Config {
  fn default() -> Self {
    Self {
      baudrate: Baudrate::Bps115200,
      data_bits: DataBits::Eight,
      stop_bits: StopBits::One,
      parity: Parity::None,
      flow_control: FlowControl::None,
      buffer_size: 256,
      timeout_ms: 1000,
      dma_enabled: true,
    }
  }
}

impl Config {
  /// 创建新的配置
  pub const fn new() -> Self {
    Self {
      baudrate: Baudrate::Bps115200,
      data_bits: DataBits::Eight,
      stop_bits: StopBits::One,
      parity: Parity::None,
      flow_control: FlowControl::None,
      buffer_size: 256,
      timeout_ms: 1000,
      dma_enabled: true,
    }
  }

  /// 设置波特率
  pub const fn baudrate(mut self, baudrate: Baudrate) -> Self {
    self.baudrate = baudrate;
    self
  }

  /// 设置数据位
  pub const fn data_bits(mut self, data_bits: DataBits) -> Self {
    self.data_bits = data_bits;
    self
  }

  /// 设置停止位
  pub const fn stop_bits(mut self, stop_bits: StopBits) -> Self {
    self.stop_bits = stop_bits;
    self
  }

  /// 设置校验位
  pub const fn parity(mut self, parity: Parity) -> Self {
    self.parity = parity;
    self
  }

  /// 设置流控制
  pub const fn flow_control(mut self, flow_control: FlowControl) -> Self {
    self.flow_control = flow_control;
    self
  }

  /// 设置缓冲区大小
  pub const fn buffer_size(mut self, size: usize) -> Self {
    self.buffer_size = size;
    self
  }

  /// 设置超时时间
  pub const fn timeout_ms(mut self, timeout: u32) -> Self {
    self.timeout_ms = timeout;
    self
  }

  /// 启用DMA
  pub const fn enable_dma(mut self) -> Self {
    self.dma_enabled = true;
    self
  }

  /// 禁用DMA
  pub const fn disable_dma(mut self) -> Self {
    self.dma_enabled = false;
    self
  }

  /// 验证配置是否有效
  pub fn validate(&self) -> Result<(), ConfigError> {
    // 检查缓冲区大小
    if self.buffer_size == 0 {
      return Err(ConfigError::InvalidBufferSize);
    }

    if self.buffer_size > 65536 {
      return Err(ConfigError::BufferSizeTooLarge);
    }

    // 检查超时时间
    if self.timeout_ms == 0 {
      return Err(ConfigError::InvalidTimeout);
    }

    // 检查数据位和校验位的组合
    match (self.data_bits, self.parity) {
      (DataBits::Nine, Parity::Even | Parity::Odd) => {
        return Err(ConfigError::InvalidDataParityCombination);
      }
      _ => {}
    }

    Ok(())
  }

  /// 计算每字符的总位数
  pub fn bits_per_char(&self) -> u8 {
    let data_bits = match self.data_bits {
      DataBits::Five => 5,
      DataBits::Six => 6,
      DataBits::Seven => 7,
      DataBits::Eight => 8,
      DataBits::Nine => 9,
    };

    let parity_bits = match self.parity {
      Parity::None => 0,
      Parity::Even | Parity::Odd => 1,
    };

    let stop_bits = match self.stop_bits {
      StopBits::One => 1,
      StopBits::OnePointFive => 2, // 近似为2
      StopBits::Two => 2,
    };

    1 + data_bits + parity_bits + stop_bits // 1个起始位
  }

  /// 计算理论最大传输速率（字符/秒）
  pub fn max_chars_per_second(&self) -> u32 {
    self.baudrate.as_u32() / self.bits_per_char() as u32
  }

  /// 计算理论最大传输速率（字节/秒）
  pub fn max_bytes_per_second(&self) -> u32 {
    if self.data_bits == DataBits::Eight {
      self.max_chars_per_second()
    } else {
      // 对于非8位数据，需要考虑编码效率
      (self.max_chars_per_second() * 8) / self.data_bits.as_u8() as u32
    }
  }
}

/// 波特率枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Baudrate {
  /// 1200 bps
  Bps1200,
  /// 2400 bps
  Bps2400,
  /// 4800 bps
  Bps4800,
  /// 9600 bps
  Bps9600,
  /// 19200 bps
  Bps19200,
  /// 38400 bps
  Bps38400,
  /// 57600 bps
  Bps57600,
  /// 115200 bps
  Bps115200,
  /// 230400 bps
  Bps230400,
  /// 460800 bps
  Bps460800,
  /// 921600 bps
  Bps921600,
  /// 1000000 bps
  Bps1000000,
  /// 1500000 bps
  Bps1500000,
  /// 2000000 bps
  Bps2000000,
  /// 自定义波特率
  Custom(u32),
}

impl Baudrate {
  /// 获取波特率数值
  pub const fn as_u32(&self) -> u32 {
    match self {
      Baudrate::Bps1200 => 1200,
      Baudrate::Bps2400 => 2400,
      Baudrate::Bps4800 => 4800,
      Baudrate::Bps9600 => 9600,
      Baudrate::Bps19200 => 19200,
      Baudrate::Bps38400 => 38400,
      Baudrate::Bps57600 => 57600,
      Baudrate::Bps115200 => 115200,
      Baudrate::Bps230400 => 230400,
      Baudrate::Bps460800 => 460800,
      Baudrate::Bps921600 => 921600,
      Baudrate::Bps1000000 => 1000000,
      Baudrate::Bps1500000 => 1500000,
      Baudrate::Bps2000000 => 2000000,
      Baudrate::Custom(rate) => *rate,
    }
  }

  /// 从数值创建波特率
  pub const fn from_u32(rate: u32) -> Self {
    match rate {
      1200 => Baudrate::Bps1200,
      2400 => Baudrate::Bps2400,
      4800 => Baudrate::Bps4800,
      9600 => Baudrate::Bps9600,
      19200 => Baudrate::Bps19200,
      38400 => Baudrate::Bps38400,
      57600 => Baudrate::Bps57600,
      115200 => Baudrate::Bps115200,
      230400 => Baudrate::Bps230400,
      460800 => Baudrate::Bps460800,
      921600 => Baudrate::Bps921600,
      1000000 => Baudrate::Bps1000000,
      1500000 => Baudrate::Bps1500000,
      2000000 => Baudrate::Bps2000000,
      _ => Baudrate::Custom(rate),
    }
  }

  /// 检查波特率是否为标准值
  pub const fn is_standard(&self) -> bool {
    !matches!(self, Baudrate::Custom(_))
  }

  /// 获取所有标准波特率
  pub const fn standard_rates() -> &'static [Baudrate] {
    &[
      Baudrate::Bps1200,
      Baudrate::Bps2400,
      Baudrate::Bps4800,
      Baudrate::Bps9600,
      Baudrate::Bps19200,
      Baudrate::Bps38400,
      Baudrate::Bps57600,
      Baudrate::Bps115200,
      Baudrate::Bps230400,
      Baudrate::Bps460800,
      Baudrate::Bps921600,
      Baudrate::Bps1000000,
      Baudrate::Bps1500000,
      Baudrate::Bps2000000,
    ]
  }
}

impl fmt::Display for Baudrate {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(f, "{} bps", self.as_u32())
  }
}

/// 数据位数枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DataBits {
  /// 5位数据
  Five,
  /// 6位数据
  Six,
  /// 7位数据
  Seven,
  /// 8位数据
  Eight,
  /// 9位数据
  Nine,
}

impl DataBits {
  /// 获取数据位数值
  pub const fn as_u8(&self) -> u8 {
    match self {
      DataBits::Five => 5,
      DataBits::Six => 6,
      DataBits::Seven => 7,
      DataBits::Eight => 8,
      DataBits::Nine => 9,
    }
  }

  /// 从数值创建数据位
  pub const fn from_u8(bits: u8) -> Option<Self> {
    match bits {
      5 => Some(DataBits::Five),
      6 => Some(DataBits::Six),
      7 => Some(DataBits::Seven),
      8 => Some(DataBits::Eight),
      9 => Some(DataBits::Nine),
      _ => None,
    }
  }
}

impl fmt::Display for DataBits {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(f, "{} bits", self.as_u8())
  }
}

/// 停止位数枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StopBits {
  /// 1个停止位
  One,
  /// 1.5个停止位
  OnePointFive,
  /// 2个停止位
  Two,
}

impl StopBits {
  /// 获取停止位数值（以位为单位）
  pub const fn as_f32(&self) -> f32 {
    match self {
      StopBits::One => 1.0,
      StopBits::OnePointFive => 1.5,
      StopBits::Two => 2.0,
    }
  }
}

impl fmt::Display for StopBits {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      StopBits::One => write!(f, "1 stop bit"),
      StopBits::OnePointFive => write!(f, "1.5 stop bits"),
      StopBits::Two => write!(f, "2 stop bits"),
    }
  }
}

/// 校验位枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Parity {
  /// 无校验
  None,
  /// 偶校验
  Even,
  /// 奇校验
  Odd,
}

impl fmt::Display for Parity {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Parity::None => write!(f, "No parity"),
      Parity::Even => write!(f, "Even parity"),
      Parity::Odd => write!(f, "Odd parity"),
    }
  }
}

/// 流控制枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FlowControl {
  /// 无流控制
  None,
  /// 软件流控制（XON/XOFF）
  Software,
  /// 硬件流控制（RTS/CTS）
  RtsCts,
  /// 硬件流控制（DTR/DSR）
  DtrDsr,
}

impl fmt::Display for FlowControl {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      FlowControl::None => write!(f, "No flow control"),
      FlowControl::Software => write!(f, "Software flow control (XON/XOFF)"),
      FlowControl::RtsCts => write!(f, "Hardware flow control (RTS/CTS)"),
      FlowControl::DtrDsr => write!(f, "Hardware flow control (DTR/DSR)"),
    }
  }
}

/// 配置错误枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
  /// 无效的缓冲区大小
  InvalidBufferSize,
  /// 缓冲区大小过大
  BufferSizeTooLarge,
  /// 无效的超时时间
  InvalidTimeout,
  /// 无效的数据位和校验位组合
  InvalidDataParityCombination,
  /// 不支持的波特率
  UnsupportedBaudrate,
  /// 不支持的数据位
  UnsupportedDataBits,
  /// 不支持的停止位
  UnsupportedStopBits,
  /// 不支持的校验位
  UnsupportedParity,
  /// 不支持的流控制
  UnsupportedFlowControl,
}

impl fmt::Display for ConfigError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      ConfigError::InvalidBufferSize => write!(f, "Invalid buffer size"),
      ConfigError::BufferSizeTooLarge => write!(f, "Buffer size too large"),
      ConfigError::InvalidTimeout => write!(f, "Invalid timeout"),
      ConfigError::InvalidDataParityCombination => {
        write!(f, "Invalid data bits and parity combination")
      }
      ConfigError::UnsupportedBaudrate => write!(f, "Unsupported baudrate"),
      ConfigError::UnsupportedDataBits => write!(f, "Unsupported data bits"),
      ConfigError::UnsupportedStopBits => write!(f, "Unsupported stop bits"),
      ConfigError::UnsupportedParity => write!(f, "Unsupported parity"),
      ConfigError::UnsupportedFlowControl => write!(f, "Unsupported flow control"),
    }
  }
}

/// 预定义配置
pub mod presets {
  use super::*;

  /// 标准串口配置（115200, 8N1）
  pub const STANDARD: Config = Config {
    baudrate: Baudrate::Bps115200,
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: Parity::None,
    flow_control: FlowControl::None,
    buffer_size: 256,
    timeout_ms: 1000,
    dma_enabled: true,
  };

  /// 高速配置（921600, 8N1, RTS/CTS）
  pub const HIGH_SPEED: Config = Config {
    baudrate: Baudrate::Bps921600,
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: Parity::None,
    flow_control: FlowControl::RtsCts,
    buffer_size: 1024,
    timeout_ms: 500,
    dma_enabled: true,
  };

  /// 低功耗配置（9600, 8N1）
  pub const LOW_POWER: Config = Config {
    baudrate: Baudrate::Bps9600,
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: Parity::None,
    flow_control: FlowControl::None,
    buffer_size: 64,
    timeout_ms: 5000,
    dma_enabled: false,
  };

  /// Modbus RTU配置（19200, 8E1）
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

  /// GPS配置（4800, 8N1）
  pub const GPS: Config = Config {
    baudrate: Baudrate::Bps4800,
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: Parity::None,
    flow_control: FlowControl::None,
    buffer_size: 512,
    timeout_ms: 2000,
    dma_enabled: true,
  };

  /// 蓝牙模块配置（38400, 8N1）
  pub const BLUETOOTH: Config = Config {
    baudrate: Baudrate::Bps38400,
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: Parity::None,
    flow_control: FlowControl::None,
    buffer_size: 256,
    timeout_ms: 1000,
    dma_enabled: true,
  };
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_config_default() {
    let config = Config::default();
    assert_eq!(config.baudrate, Baudrate::Bps115200);
    assert_eq!(config.data_bits, DataBits::Eight);
    assert_eq!(config.stop_bits, StopBits::One);
    assert_eq!(config.parity, Parity::None);
    assert_eq!(config.flow_control, FlowControl::None);
    assert_eq!(config.buffer_size, 256);
    assert_eq!(config.timeout_ms, 1000);
    assert!(config.dma_enabled);
  }

  #[test]
  fn test_config_builder() {
    let config = Config::new()
      .baudrate(Baudrate::Bps9600)
      .data_bits(DataBits::Seven)
      .stop_bits(StopBits::Two)
      .parity(Parity::Even)
      .flow_control(FlowControl::RtsCts)
      .buffer_size(512)
      .timeout_ms(2000)
      .disable_dma();

    assert_eq!(config.baudrate, Baudrate::Bps9600);
    assert_eq!(config.data_bits, DataBits::Seven);
    assert_eq!(config.stop_bits, StopBits::Two);
    assert_eq!(config.parity, Parity::Even);
    assert_eq!(config.flow_control, FlowControl::RtsCts);
    assert_eq!(config.buffer_size, 512);
    assert_eq!(config.timeout_ms, 2000);
    assert!(!config.dma_enabled);
  }

  #[test]
  fn test_baudrate_conversion() {
    assert_eq!(Baudrate::Bps115200.as_u32(), 115200);
    assert_eq!(Baudrate::from_u32(115200), Baudrate::Bps115200);
    assert_eq!(Baudrate::from_u32(12345), Baudrate::Custom(12345));

    let custom = Baudrate::Custom(12345);
    assert_eq!(custom.as_u32(), 12345);
    assert!(!custom.is_standard());
  }

  #[test]
  fn test_data_bits_conversion() {
    assert_eq!(DataBits::Eight.as_u8(), 8);
    assert_eq!(DataBits::from_u8(8), Some(DataBits::Eight));
    assert_eq!(DataBits::from_u8(10), None);
  }

  #[test]
  fn test_config_validation() {
    let mut config = Config::default();
    assert!(config.validate().is_ok());

    // 测试无效缓冲区大小
    config.buffer_size = 0;
    assert_eq!(config.validate(), Err(ConfigError::InvalidBufferSize));

    config.buffer_size = 100000;
    assert_eq!(config.validate(), Err(ConfigError::BufferSizeTooLarge));

    // 测试无效超时
    config.buffer_size = 256;
    config.timeout_ms = 0;
    assert_eq!(config.validate(), Err(ConfigError::InvalidTimeout));

    // 测试无效数据位和校验位组合
    config.timeout_ms = 1000;
    config.data_bits = DataBits::Nine;
    config.parity = Parity::Even;
    assert_eq!(
      config.validate(),
      Err(ConfigError::InvalidDataParityCombination)
    );
  }

  #[test]
  fn test_bits_per_char() {
    let config = Config::default(); // 8N1
    assert_eq!(config.bits_per_char(), 10); // 1 start + 8 data + 0 parity + 1 stop

    let config = Config::default().parity(Parity::Even); // 8E1
    assert_eq!(config.bits_per_char(), 11); // 1 start + 8 data + 1 parity + 1 stop

    let config = Config::default().stop_bits(StopBits::Two); // 8N2
    assert_eq!(config.bits_per_char(), 11); // 1 start + 8 data + 0 parity + 2 stop
  }

  #[test]
  fn test_max_transfer_rate() {
    let config = Config::default(); // 115200 bps, 8N1
    let expected_chars_per_sec = 115200 / 10; // 10 bits per char
    assert_eq!(config.max_chars_per_second(), expected_chars_per_sec);
    assert_eq!(config.max_bytes_per_second(), expected_chars_per_sec);
  }

  #[test]
  fn test_presets() {
    assert!(presets::STANDARD.validate().is_ok());
    assert!(presets::HIGH_SPEED.validate().is_ok());
    assert!(presets::LOW_POWER.validate().is_ok());
    assert!(presets::MODBUS_RTU.validate().is_ok());
    assert!(presets::GPS.validate().is_ok());
    assert!(presets::BLUETOOTH.validate().is_ok());

    assert_eq!(presets::MODBUS_RTU.parity, Parity::Even);
    assert_eq!(presets::HIGH_SPEED.flow_control, FlowControl::RtsCts);
    assert!(!presets::LOW_POWER.dma_enabled);
  }
}
