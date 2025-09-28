use embedded_hal::i2c::I2c;
use heapless::Vec;

/// PCF8574错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pcf8574Error<E> {
  /// I2C通信错误
  I2c(E),
  /// 无效的设备地址
  InvalidAddress,
  /// 设备未响应
  DeviceNotFound,
  /// 引脚配置错误
  InvalidPin,
  /// 缓冲区满
  BufferFull,
}

impl<E> From<E> for Pcf8574Error<E> {
  fn from(error: E) -> Self {
    Pcf8574Error::I2c(error)
  }
}

/// PCF8574引脚状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PinState {
  Low = 0,
  High = 1,
}

impl From<bool> for PinState {
  fn from(value: bool) -> Self {
    if value {
      PinState::High
    } else {
      PinState::Low
    }
  }
}

impl From<PinState> for bool {
  fn from(state: PinState) -> Self {
    match state {
      PinState::High => true,
      PinState::Low => false,
    }
  }
}

/// PCF8574引脚编号
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pin {
  P0 = 0,
  P1 = 1,
  P2 = 2,
  P3 = 3,
  P4 = 4,
  P5 = 5,
  P6 = 6,
  P7 = 7,
}

impl Pin {
  /// 获取引脚掩码
  pub fn mask(self) -> u8 {
    1 << (self as u8)
  }

  /// 从引脚编号创建
  pub fn from_number(pin: u8) -> Option<Self> {
    match pin {
      0 => Some(Pin::P0),
      1 => Some(Pin::P1),
      2 => Some(Pin::P2),
      3 => Some(Pin::P3),
      4 => Some(Pin::P4),
      5 => Some(Pin::P5),
      6 => Some(Pin::P6),
      7 => Some(Pin::P7),
      _ => None,
    }
  }
}

/// PCF8574设备信息
#[derive(Debug, Clone)]
pub struct DeviceInfo {
  pub address: u8,
  pub last_state: u8,
  pub input_mask: u8,
  pub output_mask: u8,
}

impl DeviceInfo {
  pub fn new(address: u8) -> Self {
    Self {
      address,
      last_state: 0xFF,
      input_mask: 0xFF,
      output_mask: 0x00,
    }
  }
}

/// PCF8574驱动
pub struct Pcf8574<I2C> {
  i2c: I2C,
  devices: Vec<DeviceInfo, 8>,
  retry_count: u8,
}

impl<I2C, E> Pcf8574<I2C>
where
  I2C: I2c<Error = E>,
{
  /// 创建新的PCF8574驱动实例
  pub fn new(i2c: I2C) -> Self {
    Self {
      i2c,
      devices: Vec::new(),
      retry_count: 3,
    }
  }

  /// 设置重试次数
  pub fn set_retry_count(&mut self, count: u8) {
    self.retry_count = count;
  }

  /// 添加设备
  pub fn add_device(&mut self, address: u8) -> Result<(), Pcf8574Error<E>> {
    if address < 0x20 || address > 0x27 {
      return Err(Pcf8574Error::InvalidAddress);
    }

    // 检查设备是否已存在
    if self.devices.iter().any(|dev| dev.address == address) {
      return Ok(());
    }

    // 测试设备连接
    self.test_device(address)?;

    // 添加设备信息
    let device_info = DeviceInfo::new(address);
    self
      .devices
      .push(device_info)
      .map_err(|_| Pcf8574Error::BufferFull)?;

    Ok(())
  }

  /// 测试设备连接
  fn test_device(&mut self, address: u8) -> Result<(), Pcf8574Error<E>> {
    let mut buffer = [0u8; 1];

    for _ in 0..self.retry_count {
      match self.i2c.read(address, &mut buffer) {
        Ok(_) => return Ok(()),
        Err(e) => {
          // 短暂延时后重试
          cortex_m::asm::delay(1000);
          if self.retry_count == 1 {
            return Err(Pcf8574Error::I2c(e));
          }
        }
      }
    }

    Err(Pcf8574Error::DeviceNotFound)
  }

  /// 获取设备信息
  fn get_device_mut(&mut self, address: u8) -> Result<&mut DeviceInfo, Pcf8574Error<E>> {
    self
      .devices
      .iter_mut()
      .find(|dev| dev.address == address)
      .ok_or(Pcf8574Error::DeviceNotFound)
  }

  /// 读取设备状态
  pub fn read_device(&mut self, address: u8) -> Result<u8, Pcf8574Error<E>> {
    let mut buffer = [0u8; 1];

    for _ in 0..self.retry_count {
      match self.i2c.read(address, &mut buffer) {
        Ok(_) => {
          let device = self.get_device_mut(address)?;
          device.last_state = buffer[0];
          return Ok(buffer[0]);
        }
        Err(e) => {
          cortex_m::asm::delay(1000);
          if self.retry_count == 1 {
            return Err(Pcf8574Error::I2c(e));
          }
        }
      }
    }

    Err(Pcf8574Error::DeviceNotFound)
  }

  /// 写入设备状态
  pub fn write_device(&mut self, address: u8, value: u8) -> Result<(), Pcf8574Error<E>> {
    let buffer = [value];

    for _ in 0..self.retry_count {
      match self.i2c.write(address, &buffer) {
        Ok(_) => {
          let device = self.get_device_mut(address)?;
          device.last_state = value;
          return Ok(());
        }
        Err(e) => {
          cortex_m::asm::delay(1000);
          if self.retry_count == 1 {
            return Err(Pcf8574Error::I2c(e));
          }
        }
      }
    }

    Err(Pcf8574Error::DeviceNotFound)
  }

  /// 设置引脚为输出模式
  pub fn set_pin_output(&mut self, address: u8, pin: Pin) -> Result<(), Pcf8574Error<E>> {
    let device = self.get_device_mut(address)?;
    device.output_mask |= pin.mask();
    device.input_mask &= !pin.mask();
    Ok(())
  }

  /// 设置引脚为输入模式
  pub fn set_pin_input(&mut self, address: u8, pin: Pin) -> Result<(), Pcf8574Error<E>> {
    let device = self.get_device_mut(address)?;
    device.input_mask |= pin.mask();
    device.output_mask &= !pin.mask();

    // 输入模式需要写入高电平
    let current_state = device.last_state | pin.mask();
    self.write_device(address, current_state)?;

    Ok(())
  }

  /// 设置引脚状态
  pub fn set_pin(&mut self, address: u8, pin: Pin, state: PinState) -> Result<(), Pcf8574Error<E>> {
    let device = self.get_device_mut(address)?;

    let new_state = match state {
      PinState::High => device.last_state | pin.mask(),
      PinState::Low => device.last_state & !pin.mask(),
    };

    self.write_device(address, new_state)?;
    Ok(())
  }

  /// 读取引脚状态
  pub fn read_pin(&mut self, address: u8, pin: Pin) -> Result<PinState, Pcf8574Error<E>> {
    let state = self.read_device(address)?;
    let pin_state = (state & pin.mask()) != 0;
    Ok(pin_state.into())
  }

  /// 切换引脚状态
  pub fn toggle_pin(&mut self, address: u8, pin: Pin) -> Result<(), Pcf8574Error<E>> {
    let current_state = self.read_pin(address, pin)?;
    let new_state = match current_state {
      PinState::High => PinState::Low,
      PinState::Low => PinState::High,
    };
    self.set_pin(address, pin, new_state)
  }

  /// 设置多个引脚状态
  pub fn set_pins(
    &mut self,
    address: u8,
    mask: u8,
    state: PinState,
  ) -> Result<(), Pcf8574Error<E>> {
    let device = self.get_device_mut(address)?;

    let new_state = match state {
      PinState::High => device.last_state | mask,
      PinState::Low => device.last_state & !mask,
    };

    self.write_device(address, new_state)?;
    Ok(())
  }

  /// 读取多个引脚状态
  pub fn read_pins(&mut self, address: u8, mask: u8) -> Result<u8, Pcf8574Error<E>> {
    let state = self.read_device(address)?;
    Ok(state & mask)
  }

  /// 获取设备列表
  pub fn get_devices(&self) -> &[DeviceInfo] {
    &self.devices
  }

  /// 扫描I2C总线上的PCF8574设备
  pub fn scan_devices(&mut self) -> Vec<u8, 8> {
    let mut found_devices = Vec::new();

    for addr in 0x20..=0x27 {
      if self.test_device(addr).is_ok() {
        let _ = found_devices.push(addr);
      }
    }

    found_devices
  }

  /// 重置所有设备
  pub fn reset_all_devices(&mut self) -> Result<(), Pcf8574Error<E>> {
    for device in &mut self.devices {
      self.write_device(device.address, 0xFF)?;
      device.last_state = 0xFF;
      device.input_mask = 0xFF;
      device.output_mask = 0x00;
    }
    Ok(())
  }
}

/// PCF8574管理器 - 提供更高级的接口
pub struct Pcf8574Manager<I2C> {
  pcf: Pcf8574<I2C>,
  interrupt_states: Vec<u8, 8>,
}

impl<I2C, E> Pcf8574Manager<I2C>
where
  I2C: I2c<Error = E>,
{
  /// 创建新的管理器
  pub fn new(i2c: I2C) -> Self {
    Self {
      pcf: Pcf8574::new(i2c),
      interrupt_states: Vec::new(),
    }
  }

  /// 初始化设备
  pub fn init_device(
    &mut self,
    address: u8,
    input_pins: u8,
    output_pins: u8,
  ) -> Result<(), Pcf8574Error<E>> {
    self.pcf.add_device(address)?;

    // 配置引脚模式
    for pin_num in 0..8 {
      if let Some(pin) = Pin::from_number(pin_num) {
        if (input_pins & pin.mask()) != 0 {
          self.pcf.set_pin_input(address, pin)?;
        } else if (output_pins & pin.mask()) != 0 {
          self.pcf.set_pin_output(address, pin)?;
        }
      }
    }

    // 记录中断状态
    let initial_state = self.pcf.read_device(address)?;
    self.interrupt_states.push(initial_state).ok();

    Ok(())
  }

  /// 检查中断状态变化
  pub fn check_interrupts(&mut self, address: u8) -> Result<Option<u8>, Pcf8574Error<E>> {
    let current_state = self.pcf.read_device(address)?;

    // 查找对应的中断状态
    if let Some(device_index) = self
      .pcf
      .devices
      .iter()
      .position(|dev| dev.address == address)
    {
      if device_index < self.interrupt_states.len() {
        let last_state = self.interrupt_states[device_index];
        let changed_pins = current_state ^ last_state;

        if changed_pins != 0 {
          self.interrupt_states[device_index] = current_state;
          return Ok(Some(changed_pins));
        }
      }
    }

    Ok(None)
  }

  /// 获取PCF8574驱动的可变引用
  pub fn pcf_mut(&mut self) -> &mut Pcf8574<I2C> {
    &mut self.pcf
  }

  /// 获取PCF8574驱动的不可变引用
  pub fn pcf(&self) -> &Pcf8574<I2C> {
    &self.pcf
  }
}
