//! # 模拟平台实现
//!
//! 用于测试和演示的模拟GPIO平台

use crate::error::*;
use crate::gpio::*;
use crate::platform::Platform;
use core::cell::RefCell;
use heapless::Vec;

/// 模拟平台结构体
pub struct MockPlatform;

impl Platform for MockPlatform {
  const NAME: &'static str = "Mock Platform";
  const MAX_PINS: usize = 16;
  const MAX_PORTS: usize = 4;
  const SUPPORTS_INTERRUPTS: bool = true;
  const SUPPORTS_DMA: bool = true;
  const SUPPORTS_ANALOG: bool = true;
}

/// 模拟引脚状态
#[derive(Debug, Clone, Copy)]
pub struct MockPinState {
  pub mode: PinMode,
  pub level: bool,
  pub pull: Pull,
  pub drive_strength: DriveStrength,
  pub slew_rate: SlewRate,
  pub schmitt_trigger: bool,
}

impl Default for MockPinState {
  fn default() -> Self {
    Self {
      mode: PinMode::Input(InputMode::Floating),
      level: false,
      pull: Pull::None,
      drive_strength: DriveStrength::Medium,
      slew_rate: SlewRate::Fast,
      schmitt_trigger: false,
    }
  }
}

/// 模拟GPIO控制器
pub struct MockGpioController {
  pins: RefCell<[MockPinState; 64]>, // 支持4个端口，每个16个引脚
}

impl MockGpioController {
  /// 创建新的模拟GPIO控制器
  pub fn new() -> Self {
    Self {
      pins: RefCell::new([MockPinState::default(); 64]),
    }
  }

  /// 获取引脚索引
  fn pin_index(port_id: char, pin_number: u8) -> usize {
    let port_offset = match port_id {
      'A' => 0,
      'B' => 16,
      'C' => 32,
      'D' => 48,
      _ => 0,
    };
    port_offset + (pin_number as usize)
  }

  /// 设置引脚状态
  pub fn set_pin_state(&self, port_id: char, pin_number: u8, state: MockPinState) {
    let index = Self::pin_index(port_id, pin_number);
    if index < 64 {
      self.pins.borrow_mut()[index] = state;
    }
  }

  /// 获取引脚状态
  pub fn get_pin_state(&self, port_id: char, pin_number: u8) -> MockPinState {
    let index = Self::pin_index(port_id, pin_number);
    if index < 64 {
      self.pins.borrow()[index]
    } else {
      MockPinState::default()
    }
  }

  /// 设置引脚电平
  pub fn set_pin_level(&self, port_id: char, pin_number: u8, level: bool) {
    let index = Self::pin_index(port_id, pin_number);
    if index < 64 {
      self.pins.borrow_mut()[index].level = level;
    }
  }

  /// 获取引脚电平
  pub fn get_pin_level(&self, port_id: char, pin_number: u8) -> bool {
    let index = Self::pin_index(port_id, pin_number);
    if index < 64 {
      self.pins.borrow()[index].level
    } else {
      false
    }
  }

  /// 配置引脚模式
  pub fn configure_pin_mode(&self, port_id: char, pin_number: u8, mode: PinMode) {
    let index = Self::pin_index(port_id, pin_number);
    if index < 64 {
      self.pins.borrow_mut()[index].mode = mode;
    }
  }

  /// 批量设置引脚
  pub fn bulk_set_pins(&self, port_id: char, mask: u16, value: u16) {
    let port_offset = match port_id {
      'A' => 0,
      'B' => 16,
      'C' => 32,
      'D' => 48,
      _ => return,
    };

    let mut pins = self.pins.borrow_mut();
    for i in 0..16 {
      if (mask & (1 << i)) != 0 {
        let index = port_offset + i;
        if index < 64 {
          pins[index].level = (value & (1 << i)) != 0;
        }
      }
    }
  }

  /// 批量读取引脚
  pub fn bulk_read_pins(&self, port_id: char) -> u16 {
    let port_offset = match port_id {
      'A' => 0,
      'B' => 16,
      'C' => 32,
      'D' => 48,
      _ => return 0,
    };

    let pins = self.pins.borrow();
    let mut value = 0u16;

    for i in 0..16 {
      let index = port_offset + i;
      if index < 64 && pins[index].level {
        value |= 1 << i;
      }
    }

    value
  }
}

/// 模拟GPIO引脚
pub struct MockPin<MODE> {
  pin_number: u8,
  port_id: char,
  controller: &'static MockGpioController,
  _mode: core::marker::PhantomData<MODE>,
}

impl<MODE> MockPin<MODE> {
  /// 创建新的模拟引脚
  pub fn new(pin_number: u8, port_id: char, controller: &'static MockGpioController) -> Self {
    Self {
      pin_number,
      port_id,
      controller,
      _mode: core::marker::PhantomData,
    }
  }
}

impl<MODE> GpioPin for MockPin<MODE> {
  type Error = HalError;

  fn pin_number(&self) -> u8 {
    self.pin_number
  }

  fn port_id(&self) -> char {
    self.port_id
  }

  fn current_mode(&self) -> PinMode {
    self
      .controller
      .get_pin_state(self.port_id, self.pin_number)
      .mode
  }
}

// 状态转换实现
impl<MODE> MockPin<MODE> {
  /// 转换为浮空输入
  pub fn into_floating_input(self) -> MockPin<Input<Floating>> {
    self.controller.configure_pin_mode(
      self.port_id,
      self.pin_number,
      PinMode::Input(InputMode::Floating),
    );

    MockPin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      controller: self.controller,
      _mode: core::marker::PhantomData,
    }
  }

  /// 转换为上拉输入
  pub fn into_pull_up_input(self) -> MockPin<Input<PullUp>> {
    self.controller.configure_pin_mode(
      self.port_id,
      self.pin_number,
      PinMode::Input(InputMode::PullUp),
    );

    MockPin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      controller: self.controller,
      _mode: core::marker::PhantomData,
    }
  }

  /// 转换为下拉输入
  pub fn into_pull_down_input(self) -> MockPin<Input<PullDown>> {
    self.controller.configure_pin_mode(
      self.port_id,
      self.pin_number,
      PinMode::Input(InputMode::PullDown),
    );

    MockPin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      controller: self.controller,
      _mode: core::marker::PhantomData,
    }
  }

  /// 转换为推挽输出
  pub fn into_push_pull_output(self) -> MockPin<Output<PushPull>> {
    self.controller.configure_pin_mode(
      self.port_id,
      self.pin_number,
      PinMode::Output(OutputMode::PushPull),
    );

    MockPin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      controller: self.controller,
      _mode: core::marker::PhantomData,
    }
  }

  /// 转换为开漏输出
  pub fn into_open_drain_output(self) -> MockPin<Output<OpenDrain>> {
    self.controller.configure_pin_mode(
      self.port_id,
      self.pin_number,
      PinMode::Output(OutputMode::OpenDrain),
    );

    MockPin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      controller: self.controller,
      _mode: core::marker::PhantomData,
    }
  }

  /// 转换为复用功能
  pub fn into_alternate<const AF: u8>(self) -> MockPin<Alternate<AF>> {
    self
      .controller
      .configure_pin_mode(self.port_id, self.pin_number, PinMode::Alternate(AF));

    MockPin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      controller: self.controller,
      _mode: core::marker::PhantomData,
    }
  }

  /// 转换为模拟模式
  pub fn into_analog(self) -> MockPin<Analog> {
    self
      .controller
      .configure_pin_mode(self.port_id, self.pin_number, PinMode::Analog);

    MockPin {
      pin_number: self.pin_number,
      port_id: self.port_id,
      controller: self.controller,
      _mode: core::marker::PhantomData,
    }
  }
}

// InputPin特征实现
impl<MODE> InputPin for MockPin<Input<MODE>> {
  type Error = HalError;

  fn is_high(&self) -> Result<bool, Self::Error> {
    Ok(self.controller.get_pin_level(self.port_id, self.pin_number))
  }

  fn is_low(&self) -> Result<bool, Self::Error> {
    Ok(!self.controller.get_pin_level(self.port_id, self.pin_number))
  }
}

// OutputPin特征实现
impl<MODE> OutputPin for MockPin<Output<MODE>> {
  type Error = HalError;

  fn set_high(&mut self) -> Result<(), Self::Error> {
    self
      .controller
      .set_pin_level(self.port_id, self.pin_number, true);
    log::debug!("模拟引脚 {}{} 设置为高电平", self.port_id, self.pin_number);
    Ok(())
  }

  fn set_low(&mut self) -> Result<(), Self::Error> {
    self
      .controller
      .set_pin_level(self.port_id, self.pin_number, false);
    log::debug!("模拟引脚 {}{} 设置为低电平", self.port_id, self.pin_number);
    Ok(())
  }
}

// StatefulOutputPin特征实现
impl<MODE> StatefulOutputPin for MockPin<Output<MODE>> {
  fn is_set_high(&self) -> Result<bool, Self::Error> {
    Ok(self.controller.get_pin_level(self.port_id, self.pin_number))
  }

  fn is_set_low(&self) -> Result<bool, Self::Error> {
    Ok(!self.controller.get_pin_level(self.port_id, self.pin_number))
  }
}

// ToggleableOutputPin特征实现
impl<MODE> ToggleableOutputPin for MockPin<Output<MODE>> {
  type Error = HalError;

  fn toggle(&mut self) -> Result<(), Self::Error> {
    let current_level = self.controller.get_pin_level(self.port_id, self.pin_number);
    self
      .controller
      .set_pin_level(self.port_id, self.pin_number, !current_level);
    log::debug!("模拟引脚 {}{} 切换电平", self.port_id, self.pin_number);
    Ok(())
  }
}

// AdvancedGpioPin特征实现
impl<MODE> AdvancedGpioPin for MockPin<MODE> {
  fn set_drive_strength(&mut self, strength: DriveStrength) -> Result<(), Self::Error> {
    let index = MockGpioController::pin_index(self.port_id, self.pin_number);
    if index < 64 {
      self.controller.pins.borrow_mut()[index].drive_strength = strength;
      log::debug!(
        "模拟引脚 {}{} 设置驱动强度: {:?}",
        self.port_id,
        self.pin_number,
        strength
      );
    }
    Ok(())
  }

  fn set_slew_rate(&mut self, rate: SlewRate) -> Result<(), Self::Error> {
    let index = MockGpioController::pin_index(self.port_id, self.pin_number);
    if index < 64 {
      self.controller.pins.borrow_mut()[index].slew_rate = rate;
      log::debug!(
        "模拟引脚 {}{} 设置压摆率: {:?}",
        self.port_id,
        self.pin_number,
        rate
      );
    }
    Ok(())
  }

  fn set_pull(&mut self, pull: Pull) -> Result<(), Self::Error> {
    let index = MockGpioController::pin_index(self.port_id, self.pin_number);
    if index < 64 {
      self.controller.pins.borrow_mut()[index].pull = pull;
      log::debug!(
        "模拟引脚 {}{} 设置上拉/下拉: {:?}",
        self.port_id,
        self.pin_number,
        pull
      );
    }
    Ok(())
  }

  fn set_schmitt_trigger(&mut self, enable: bool) -> Result<(), Self::Error> {
    let index = MockGpioController::pin_index(self.port_id, self.pin_number);
    if index < 64 {
      self.controller.pins.borrow_mut()[index].schmitt_trigger = enable;
      log::debug!(
        "模拟引脚 {}{} 设置施密特触发器: {}",
        self.port_id,
        self.pin_number,
        enable
      );
    }
    Ok(())
  }

  fn drive_strength(&self) -> DriveStrength {
    self
      .controller
      .get_pin_state(self.port_id, self.pin_number)
      .drive_strength
  }

  fn slew_rate(&self) -> SlewRate {
    self
      .controller
      .get_pin_state(self.port_id, self.pin_number)
      .slew_rate
  }

  fn pull_configuration(&self) -> Pull {
    self
      .controller
      .get_pin_state(self.port_id, self.pin_number)
      .pull
  }

  fn is_schmitt_trigger_enabled(&self) -> bool {
    self
      .controller
      .get_pin_state(self.port_id, self.pin_number)
      .schmitt_trigger
  }
}

/// 模拟GPIO端口
pub struct MockGpioPort {
  port_id: char,
  controller: &'static MockGpioController,
}

impl MockGpioPort {
  /// 创建新的模拟GPIO端口
  pub fn new(port_id: char, controller: &'static MockGpioController) -> Self {
    Self {
      port_id,
      controller,
    }
  }

  /// 获取引脚
  pub fn pin<const N: u8>(&self) -> MockPin<Input<Floating>> {
    MockPin::new(N, self.port_id, self.controller)
  }
}

impl BulkGpioOperations for MockGpioPort {
  type Error = HalError;

  fn set_pins(&mut self, pins: &[u8], states: &[bool]) -> Result<(), Self::Error> {
    if pins.len() != states.len() {
      return Err(HalError::Configuration(ConfigError::InvalidParameter));
    }

    for (pin, state) in pins.iter().zip(states.iter()) {
      if *pin < 16 {
        self.controller.set_pin_level(self.port_id, *pin, *state);
      }
    }

    log::debug!("模拟端口 {} 批量设置 {} 个引脚", self.port_id, pins.len());
    Ok(())
  }

  fn read_pins(&self, pins: &[u8]) -> Result<Vec<bool, 32>, Self::Error> {
    let mut results = Vec::new();

    for pin in pins {
      if *pin < 16 {
        let level = self.controller.get_pin_level(self.port_id, *pin);
        let _ = results.push(level);
      } else {
        let _ = results.push(false);
      }
    }

    Ok(results)
  }

  fn atomic_port_write(&mut self, mask: u32, value: u32) -> Result<(), Self::Error> {
    let mask16 = mask as u16;
    let value16 = value as u16;

    self.controller.bulk_set_pins(self.port_id, mask16, value16);
    log::debug!(
      "模拟端口 {} 原子写入: mask=0x{:04X}, value=0x{:04X}",
      self.port_id,
      mask16,
      value16
    );
    Ok(())
  }

  fn atomic_port_read(&self) -> Result<u32, Self::Error> {
    let value = self.controller.bulk_read_pins(self.port_id);
    Ok(value as u32)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  static MOCK_CONTROLLER: MockGpioController = MockGpioController {
    pins: RefCell::new(
      [MockPinState {
        mode: PinMode::Input(InputMode::Floating),
        level: false,
        pull: Pull::None,
        drive_strength: DriveStrength::Medium,
        slew_rate: SlewRate::Fast,
        schmitt_trigger: false,
      }; 64],
    ),
  };

  #[test]
  fn test_mock_pin_creation() {
    let pin = MockPin::new(5, 'A', &MOCK_CONTROLLER);
    assert_eq!(pin.pin_number(), 5);
    assert_eq!(pin.port_id(), 'A');
  }

  #[test]
  fn test_mock_pin_mode_transitions() {
    let pin = MockPin::new(0, 'A', &MOCK_CONTROLLER);

    let input_pin = pin.into_floating_input();
    assert_eq!(
      input_pin.current_mode(),
      PinMode::Input(InputMode::Floating)
    );

    let output_pin = input_pin.into_push_pull_output();
    assert_eq!(
      output_pin.current_mode(),
      PinMode::Output(OutputMode::PushPull)
    );
  }

  #[test]
  fn test_mock_pin_operations() {
    let pin = MockPin::new(1, 'A', &MOCK_CONTROLLER);
    let mut output_pin = pin.into_push_pull_output();

    // 测试输出操作
    assert!(output_pin.set_high().is_ok());
    assert!(output_pin.is_set_high().unwrap());

    assert!(output_pin.set_low().is_ok());
    assert!(output_pin.is_set_low().unwrap());

    // 测试切换操作
    assert!(output_pin.toggle().is_ok());
    assert!(output_pin.is_set_high().unwrap());
  }

  #[test]
  fn test_mock_bulk_operations() {
    let mut port = MockGpioPort::new('B', &MOCK_CONTROLLER);

    let pins = [0, 1, 2, 3];
    let states = [true, false, true, false];

    assert!(port.set_pins(&pins, &states).is_ok());

    let read_states = port.read_pins(&pins).unwrap();
    assert_eq!(read_states.len(), 4);
    assert_eq!(read_states[0], true);
    assert_eq!(read_states[1], false);
    assert_eq!(read_states[2], true);
    assert_eq!(read_states[3], false);
  }

  #[test]
  fn test_mock_advanced_features() {
    let pin = MockPin::new(2, 'C', &MOCK_CONTROLLER);
    let mut output_pin = pin.into_push_pull_output();

    // 测试高级配置
    assert!(output_pin.set_drive_strength(DriveStrength::High).is_ok());
    assert_eq!(output_pin.drive_strength(), DriveStrength::High);

    assert!(output_pin.set_slew_rate(SlewRate::Slow).is_ok());
    assert_eq!(output_pin.slew_rate(), SlewRate::Slow);

    assert!(output_pin.set_pull(Pull::Up).is_ok());
    assert_eq!(output_pin.pull_configuration(), Pull::Up);

    assert!(output_pin.set_schmitt_trigger(true).is_ok());
    assert!(output_pin.is_schmitt_trigger_enabled());
  }
}
