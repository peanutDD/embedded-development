//! # GPIO核心特征定义
//!
//! 这个模块定义了GPIO HAL的核心特征和类型，提供了硬件无关的
//! GPIO操作接口。

use crate::error::HalError;
use core::marker::PhantomData;

// 重新导出embedded-hal特征
pub use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};

/// GPIO引脚基础特征
///
/// 提供所有GPIO引脚的基本信息和状态查询功能
pub trait GpioPin {
  /// 错误类型
  type Error;

  /// 获取引脚编号 (0-31)
  fn pin_number(&self) -> u8;

  /// 获取端口标识 ('A', 'B', 'C', etc.)
  fn port_id(&self) -> char;

  /// 获取当前配置模式
  fn current_mode(&self) -> PinMode;

  /// 获取引脚的完整标识符 (如 "PA5")
  fn pin_id(&self) -> heapless::String<8> {
    let mut id = heapless::String::new();
    let _ = id.push(self.port_id());
    let _ = id.push_str(itoa::Buffer::new().format(self.pin_number()));
    id
  }
}

/// 可配置GPIO特征
///
/// 提供引脚模式配置和转换功能
pub trait ConfigurablePin: GpioPin {
  /// 输入引脚类型
  type Input: InputPin<Error = Self::Error>;
  /// 输出引脚类型
  type Output: OutputPin<Error = Self::Error>;
  /// 开漏输出引脚类型
  type OpenDrain: OutputPin<Error = Self::Error>;
  /// 复用功能引脚类型
  type Alternate<const AF: u8>: GpioPin<Error = Self::Error>;
  /// 模拟引脚类型
  type Analog: GpioPin<Error = Self::Error>;

  /// 配置为浮空输入
  fn into_floating_input(self) -> Result<Self::Input, Self::Error>;

  /// 配置为上拉输入
  fn into_pull_up_input(self) -> Result<Self::Input, Self::Error>;

  /// 配置为下拉输入
  fn into_pull_down_input(self) -> Result<Self::Input, Self::Error>;

  /// 配置为推挽输出
  fn into_push_pull_output(self) -> Result<Self::Output, Self::Error>;

  /// 配置为开漏输出
  fn into_open_drain_output(self) -> Result<Self::OpenDrain, Self::Error>;

  /// 配置为复用功能
  fn into_alternate<const AF: u8>(self) -> Result<Self::Alternate<AF>, Self::Error>;

  /// 配置为模拟模式
  fn into_analog(self) -> Result<Self::Analog, Self::Error>;
}

/// 高级GPIO特征
///
/// 提供高级配置选项，如驱动强度、压摆率等
pub trait AdvancedGpioPin: ConfigurablePin {
  /// 设置驱动强度
  fn set_drive_strength(&mut self, strength: DriveStrength) -> Result<(), Self::Error>;

  /// 设置压摆率
  fn set_slew_rate(&mut self, rate: SlewRate) -> Result<(), Self::Error>;

  /// 配置上拉/下拉
  fn set_pull(&mut self, pull: Pull) -> Result<(), Self::Error>;

  /// 配置施密特触发器
  fn set_schmitt_trigger(&mut self, enable: bool) -> Result<(), Self::Error>;

  /// 获取当前驱动强度
  fn drive_strength(&self) -> DriveStrength;

  /// 获取当前压摆率
  fn slew_rate(&self) -> SlewRate;

  /// 获取当前上拉/下拉配置
  fn pull_configuration(&self) -> Pull;

  /// 检查施密特触发器是否启用
  fn is_schmitt_trigger_enabled(&self) -> bool;
}

/// 引脚模式枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinMode {
  /// 输入模式
  Input(InputMode),
  /// 输出模式
  Output(OutputMode),
  /// 复用功能模式
  Alternate(u8),
  /// 模拟模式
  Analog,
}

/// 输入模式配置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputMode {
  /// 浮空输入
  Floating,
  /// 上拉输入
  PullUp,
  /// 下拉输入
  PullDown,
}

/// 输出模式配置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
  /// 推挽输出
  PushPull,
  /// 开漏输出
  OpenDrain,
}

/// 驱动强度配置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveStrength {
  /// 低驱动强度 (2mA)
  Low,
  /// 中等驱动强度 (8mA)
  Medium,
  /// 高驱动强度 (12mA)
  High,
  /// 最大驱动强度 (20mA)
  Maximum,
}

impl DriveStrength {
  /// 获取驱动电流 (mA)
  pub fn current_ma(&self) -> u8 {
    match self {
      DriveStrength::Low => 2,
      DriveStrength::Medium => 8,
      DriveStrength::High => 12,
      DriveStrength::Maximum => 20,
    }
  }
}

/// 压摆率控制
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlewRate {
  /// 慢速切换 (减少EMI)
  Slow,
  /// 快速切换 (高速应用)
  Fast,
}

/// 上拉/下拉配置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull {
  /// 无上拉/下拉
  None,
  /// 上拉
  Up,
  /// 下拉
  Down,
}

/// 类型状态模式的引脚实现
///
/// 使用类型参数来确保编译时的状态安全
pub struct Pin<MODE> {
  pin_number: u8,
  port_id: char,
  _mode: PhantomData<MODE>,
}

/// 输入状态标记
pub struct Input<MODE> {
  _mode: PhantomData<MODE>,
}

/// 输出状态标记
pub struct Output<MODE> {
  _mode: PhantomData<MODE>,
}

/// 复用功能状态标记
pub struct Alternate<const AF: u8>;

/// 模拟状态标记
pub struct Analog;

/// 浮空输入标记
pub struct Floating;

/// 上拉输入标记
pub struct PullUp;

/// 下拉输入标记
pub struct PullDown;

/// 推挽输出标记
pub struct PushPull;

/// 开漏输出标记
pub struct OpenDrain;

impl<MODE> Pin<MODE> {
  /// 创建新的引脚实例
  pub fn new(pin_number: u8, port_id: char) -> Self {
    Self {
      pin_number,
      port_id,
      _mode: PhantomData,
    }
  }
}

impl<MODE> GpioPin for Pin<MODE> {
  type Error = HalError;

  fn pin_number(&self) -> u8 {
    self.pin_number
  }

  fn port_id(&self) -> char {
    self.port_id
  }

  fn current_mode(&self) -> PinMode {
    // 这里应该读取实际的硬件寄存器
    // 为了演示，返回默认值
    PinMode::Input(InputMode::Floating)
  }
}

// 状态转换实现
impl<MODE> Pin<MODE> {
  /// 转换为浮空输入
  pub fn into_floating_input(self) -> Pin<Input<Floating>> {
    Pin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      _mode: PhantomData,
    }
  }

  /// 转换为上拉输入
  pub fn into_pull_up_input(self) -> Pin<Input<PullUp>> {
    Pin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      _mode: PhantomData,
    }
  }

  /// 转换为下拉输入
  pub fn into_pull_down_input(self) -> Pin<Input<PullDown>> {
    Pin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      _mode: PhantomData,
    }
  }

  /// 转换为推挽输出
  pub fn into_push_pull_output(self) -> Pin<Output<PushPull>> {
    Pin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      _mode: PhantomData,
    }
  }

  /// 转换为开漏输出
  pub fn into_open_drain_output(self) -> Pin<Output<OpenDrain>> {
    Pin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      _mode: PhantomData,
    }
  }

  /// 转换为复用功能
  pub fn into_alternate<const AF: u8>(self) -> Pin<Alternate<AF>> {
    Pin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      _mode: PhantomData,
    }
  }

  /// 转换为模拟模式
  pub fn into_analog(self) -> Pin<Analog> {
    Pin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      _mode: PhantomData,
    }
  }
}

// InputPin特征实现
impl<MODE> InputPin for Pin<Input<MODE>> {
  type Error = HalError;

  fn is_high(&self) -> Result<bool, Self::Error> {
    // 这里应该读取实际的GPIO输入寄存器
    // 为了演示，返回false
    Ok(false)
  }

  fn is_low(&self) -> Result<bool, Self::Error> {
    self.is_high().map(|high| !high)
  }
}

// OutputPin特征实现
impl<MODE> OutputPin for Pin<Output<MODE>> {
  type Error = HalError;

  fn set_high(&mut self) -> Result<(), Self::Error> {
    // 这里应该设置实际的GPIO输出寄存器
    log::debug!("设置引脚 {}{} 为高电平", self.port_id, self.pin_number);
    Ok(())
  }

  fn set_low(&mut self) -> Result<(), Self::Error> {
    // 这里应该设置实际的GPIO输出寄存器
    log::debug!("设置引脚 {}{} 为低电平", self.port_id, self.pin_number);
    Ok(())
  }
}

// StatefulOutputPin特征实现
impl<MODE> StatefulOutputPin for Pin<Output<MODE>> {
  fn is_set_high(&self) -> Result<bool, Self::Error> {
    // 这里应该读取实际的GPIO输出寄存器
    Ok(false)
  }

  fn is_set_low(&self) -> Result<bool, Self::Error> {
    self.is_set_high().map(|high| !high)
  }
}

// ToggleableOutputPin特征实现
impl<MODE> ToggleableOutputPin for Pin<Output<MODE>> {
  type Error = HalError;

  fn toggle(&mut self) -> Result<(), Self::Error> {
    if self.is_set_high()? {
      self.set_low()
    } else {
      self.set_high()
    }
  }
}

/// 批量GPIO操作特征
pub trait BulkGpioOperations {
  /// 错误类型
  type Error;

  /// 批量设置多个引脚
  fn set_pins(&mut self, pins: &[u8], states: &[bool]) -> Result<(), Self::Error>;

  /// 批量读取多个引脚
  fn read_pins(&self, pins: &[u8]) -> Result<heapless::Vec<bool, 32>, Self::Error>;

  /// 原子性端口写入
  fn atomic_port_write(&mut self, mask: u32, value: u32) -> Result<(), Self::Error>;

  /// 原子性端口读取
  fn atomic_port_read(&self) -> Result<u32, Self::Error>;
}

/// GPIO端口抽象
pub struct GpioPort<const N: usize> {
  port_id: char,
  pins: [Option<Pin<Input<Floating>>>; N],
}

impl<const N: usize> GpioPort<N> {
  /// 创建新的GPIO端口
  pub fn new(port_id: char) -> Self {
    Self {
      port_id,
      pins: [const { None }; N],
    }
  }

  /// 获取指定引脚
  pub fn pin(&mut self, pin_number: u8) -> Option<Pin<Input<Floating>>> {
    if (pin_number as usize) < N {
      self.pins[pin_number as usize].take()
    } else {
      None
    }
  }

  /// 释放引脚回端口
  pub fn release_pin(&mut self, pin: Pin<Input<Floating>>) {
    let pin_number = pin.pin_number() as usize;
    if pin_number < N {
      self.pins[pin_number] = Some(pin);
    }
  }
}

impl<const N: usize> BulkGpioOperations for GpioPort<N> {
  type Error = HalError;

  fn set_pins(&mut self, pins: &[u8], states: &[bool]) -> Result<(), Self::Error> {
    if pins.len() != states.len() {
      return Err(HalError::Configuration(
        crate::error::ConfigError::InvalidConfiguration,
      ));
    }

    // 这里应该实现实际的批量设置逻辑
    for (pin, state) in pins.iter().zip(states.iter()) {
      log::debug!(
        "设置引脚 {}{} 为 {}",
        self.port_id,
        pin,
        if *state { "高" } else { "低" }
      );
    }

    Ok(())
  }

  fn read_pins(&self, pins: &[u8]) -> Result<heapless::Vec<bool, 32>, Self::Error> {
    let mut results = heapless::Vec::new();

    for pin in pins {
      // 这里应该读取实际的GPIO状态
      let _ = results.push(false);
    }

    Ok(results)
  }

  fn atomic_port_write(&mut self, mask: u32, value: u32) -> Result<(), Self::Error> {
    // 这里应该实现原子性端口写入
    log::debug!(
      "原子写入端口 {}: mask=0x{:08X}, value=0x{:08X}",
      self.port_id,
      mask,
      value
    );
    Ok(())
  }

  fn atomic_port_read(&self) -> Result<u32, Self::Error> {
    // 这里应该读取实际的端口状态
    Ok(0)
  }
}

/// 引脚配置验证特征
pub trait PinConfiguration {
  /// 配置是否有效
  const IS_VALID: bool;

  /// 验证配置
  fn validate() -> Result<(), HalError> {
    if Self::IS_VALID {
      Ok(())
    } else {
      Err(HalError::Configuration(
        crate::error::ConfigError::InvalidConfiguration,
      ))
    }
  }
}

/// 复用功能映射验证
pub trait AlternateFunctionMapping<const AF: u8> {
  /// 是否支持该复用功能
  const IS_SUPPORTED: bool = false;

  /// 获取功能描述
  fn function_description() -> &'static str {
    "未知功能"
  }
}

// 示例：为特定引脚实现复用功能支持
pub struct PA9;

impl AlternateFunctionMapping<7> for PA9 {
  const IS_SUPPORTED: bool = true;

  fn function_description() -> &'static str {
    "USART1_TX"
  }
}

impl AlternateFunctionMapping<1> for PA9 {
  const IS_SUPPORTED: bool = true;

  fn function_description() -> &'static str {
    "TIM1_CH2"
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_pin_creation() {
    let pin = Pin::new(5, 'A');
    assert_eq!(pin.pin_number(), 5);
    assert_eq!(pin.port_id(), 'A');
    assert_eq!(pin.pin_id(), "A5");
  }

  #[test]
  fn test_pin_mode_transitions() {
    let pin = Pin::new(0, 'A');

    // 测试状态转换
    let input_pin = pin.into_floating_input();
    let output_pin = input_pin.into_push_pull_output();
    let _analog_pin = output_pin.into_analog();

    // 编译通过说明类型转换正确
  }

  #[test]
  fn test_drive_strength() {
    assert_eq!(DriveStrength::Low.current_ma(), 2);
    assert_eq!(DriveStrength::Medium.current_ma(), 8);
    assert_eq!(DriveStrength::High.current_ma(), 12);
    assert_eq!(DriveStrength::Maximum.current_ma(), 20);
  }

  #[test]
  fn test_alternate_function_mapping() {
    assert!(PA9::IS_SUPPORTED);
    assert_eq!(PA9::function_description(), "USART1_TX");
  }
}
