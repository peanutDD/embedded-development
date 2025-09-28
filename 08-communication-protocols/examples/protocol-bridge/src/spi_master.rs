use crate::protocol::{OperationType, ProtocolError, ProtocolMessage};
use crate::{BUFFER_SIZE, MAX_SPI_DEVICES, SPI_MAX_FREQUENCY, SPI_TIMEOUT_MS};
use heapless::Vec;
use stm32f4xx_hal::{
  gpio::{Alternate, Output, Pin, PushPull},
  pac::SPI1,
  prelude::*,
  spi::{Error as SpiError, Mode, Phase, Polarity, Spi},
};

/// SPI设备配置
#[derive(Debug, Clone, Copy)]
pub struct SpiDeviceConfig {
  pub device_id: u8,
  pub cs_pin: u8, // 片选引脚编号
  pub frequency: u32,
  pub mode: SpiMode,
  pub bit_order: BitOrder,
  pub timeout_ms: u32,
}

/// SPI模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpiMode {
  Mode0, // CPOL=0, CPHA=0
  Mode1, // CPOL=0, CPHA=1
  Mode2, // CPOL=1, CPHA=0
  Mode3, // CPOL=1, CPHA=1
}

impl From<SpiMode> for Mode {
  fn from(mode: SpiMode) -> Self {
    match mode {
      SpiMode::Mode0 => Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
      },
      SpiMode::Mode1 => Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnSecondTransition,
      },
      SpiMode::Mode2 => Mode {
        polarity: Polarity::IdleHigh,
        phase: Phase::CaptureOnFirstTransition,
      },
      SpiMode::Mode3 => Mode {
        polarity: Polarity::IdleHigh,
        phase: Phase::CaptureOnSecondTransition,
      },
    }
  }
}

/// 位序
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BitOrder {
  MsbFirst,
  LsbFirst,
}

/// SPI设备状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeviceState {
  Idle,
  Busy,
  Error,
  Offline,
}

/// SPI设备信息
#[derive(Debug, Clone)]
pub struct SpiDevice {
  pub config: SpiDeviceConfig,
  pub state: DeviceState,
  pub last_communication_ms: u32,
  pub error_count: u32,
  pub transaction_count: u32,
}

impl SpiDevice {
  /// 创建新的SPI设备
  pub fn new(config: SpiDeviceConfig) -> Self {
    Self {
      config,
      state: DeviceState::Idle,
      last_communication_ms: 0,
      error_count: 0,
      transaction_count: 0,
    }
  }

  /// 更新设备状态
  pub fn update_state(&mut self, state: DeviceState, current_time_ms: u32) {
    self.state = state;
    if state != DeviceState::Error {
      self.last_communication_ms = current_time_ms;
    }
  }

  /// 增加错误计数
  pub fn increment_error(&mut self) {
    self.error_count += 1;
    self.state = DeviceState::Error;
  }

  /// 增加事务计数
  pub fn increment_transaction(&mut self) {
    self.transaction_count += 1;
  }

  /// 检查设备是否在线
  pub fn is_online(&self, current_time_ms: u32) -> bool {
    match self.state {
      DeviceState::Offline => false,
      DeviceState::Error => {
        // 如果错误次数过多，认为离线
        self.error_count < 10
      }
      _ => {
        // 检查最后通信时间
        let elapsed = current_time_ms.wrapping_sub(self.last_communication_ms);
        elapsed < 30000 // 30秒超时
      }
    }
  }
}

/// SPI主设备驱动
pub struct SpiMaster {
  spi: Spi<SPI1>,
  devices: Vec<SpiDevice, MAX_SPI_DEVICES>,
  cs_pins: Vec<Pin<Output<PushPull>>, MAX_SPI_DEVICES>,
  current_device: Option<u8>,
  tx_buffer: Vec<u8, BUFFER_SIZE>,
  rx_buffer: Vec<u8, BUFFER_SIZE>,
}

impl SpiMaster {
  /// 创建新的SPI主设备
  pub fn new(spi: Spi<SPI1>) -> Self {
    Self {
      spi,
      devices: Vec::new(),
      cs_pins: Vec::new(),
      current_device: None,
      tx_buffer: Vec::new(),
      rx_buffer: Vec::new(),
    }
  }

  /// 添加SPI设备
  pub fn add_device(
    &mut self,
    config: SpiDeviceConfig,
    cs_pin: Pin<Output<PushPull>>,
  ) -> Result<(), ProtocolError> {
    if self.devices.len() >= MAX_SPI_DEVICES {
      return Err(ProtocolError::BufferOverflow);
    }

    let device = SpiDevice::new(config);
    self
      .devices
      .push(device)
      .map_err(|_| ProtocolError::BufferOverflow)?;
    self
      .cs_pins
      .push(cs_pin)
      .map_err(|_| ProtocolError::BufferOverflow)?;

    defmt::info!(
      "Added SPI device {} with CS pin {}",
      config.device_id,
      config.cs_pin
    );
    Ok(())
  }

  /// 选择设备
  fn select_device(&mut self, device_id: u8) -> Result<usize, ProtocolError> {
    let device_index = self
      .devices
      .iter()
      .position(|dev| dev.config.device_id == device_id)
      .ok_or(ProtocolError::DeviceNotFound)?;

    // 如果当前有选中的设备，先取消选择
    if let Some(current_id) = self.current_device {
      if let Some(current_index) = self
        .devices
        .iter()
        .position(|dev| dev.config.device_id == current_id)
      {
        self.cs_pins[current_index].set_high();
      }
    }

    // 配置SPI参数
    let device = &self.devices[device_index];
    self.configure_spi_for_device(&device.config)?;

    // 选择新设备
    self.cs_pins[device_index].set_low();
    self.current_device = Some(device_id);

    Ok(device_index)
  }

  /// 取消选择设备
  fn deselect_device(&mut self) {
    if let Some(current_id) = self.current_device {
      if let Some(current_index) = self
        .devices
        .iter()
        .position(|dev| dev.config.device_id == current_id)
      {
        self.cs_pins[current_index].set_high();
      }
    }
    self.current_device = None;
  }

  /// 为设备配置SPI参数
  fn configure_spi_for_device(&mut self, config: &SpiDeviceConfig) -> Result<(), ProtocolError> {
    // 这里应该重新配置SPI的频率、模式等参数
    // 由于HAL限制，这里只是示例代码
    defmt::debug!(
      "Configuring SPI for device {}: freq={}, mode={:?}",
      config.device_id,
      config.frequency,
      config.mode
    );
    Ok(())
  }

  /// 执行SPI传输
  pub fn transfer(
    &mut self,
    message: &ProtocolMessage,
    current_time_ms: u32,
  ) -> Result<ProtocolMessage, ProtocolError> {
    let device_index = self.select_device(message.device_id)?;

    // 更新设备状态
    self.devices[device_index].update_state(DeviceState::Busy, current_time_ms);

    let result = self.perform_transfer(message, current_time_ms);

    // 更新设备状态和统计
    match &result {
      Ok(_) => {
        self.devices[device_index].update_state(DeviceState::Idle, current_time_ms);
        self.devices[device_index].increment_transaction();
      }
      Err(_) => {
        self.devices[device_index].increment_error();
      }
    }

    self.deselect_device();
    result
  }

  /// 执行实际的SPI传输
  fn perform_transfer(
    &mut self,
    message: &ProtocolMessage,
    current_time_ms: u32,
  ) -> Result<ProtocolMessage, ProtocolError> {
    // 准备发送缓冲区
    self.tx_buffer.clear();
    self.rx_buffer.clear();

    // 构建SPI命令
    match message.operation {
      OperationType::Read => {
        // 读命令格式: [READ_CMD | address] [dummy_bytes...]
        self
          .tx_buffer
          .push(0x80 | (message.address & 0x7F) as u8)
          .map_err(|_| ProtocolError::BufferOverflow)?;

        // 添加虚拟字节用于读取数据
        let read_length = if message.data.is_empty() {
          1
        } else {
          message.data.len()
        };
        for _ in 0..read_length {
          self
            .tx_buffer
            .push(0x00)
            .map_err(|_| ProtocolError::BufferOverflow)?;
        }
      }
      OperationType::Write => {
        // 写命令格式: [WRITE_CMD | address] [data...]
        self
          .tx_buffer
          .push((message.address & 0x7F) as u8)
          .map_err(|_| ProtocolError::BufferOverflow)?;

        for &byte in &message.data {
          self
            .tx_buffer
            .push(byte)
            .map_err(|_| ProtocolError::BufferOverflow)?;
        }
      }
      OperationType::Status => {
        // 状态查询命令
        self
          .tx_buffer
          .push(0xF0)
          .map_err(|_| ProtocolError::BufferOverflow)?;
        self
          .tx_buffer
          .push(0x00)
          .map_err(|_| ProtocolError::BufferOverflow)?;
      }
      OperationType::Reset => {
        // 复位命令
        self
          .tx_buffer
          .push(0xFF)
          .map_err(|_| ProtocolError::BufferOverflow)?;
      }
    }

    // 准备接收缓冲区
    for _ in 0..self.tx_buffer.len() {
      self
        .rx_buffer
        .push(0x00)
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }

    // 执行SPI传输
    let start_time = current_time_ms;
    let transfer_result = self
      .spi
      .transfer(&mut self.rx_buffer[..], &self.tx_buffer[..]);

    match transfer_result {
      Ok(_) => {
        let elapsed = current_time_ms.wrapping_sub(start_time);
        defmt::debug!("SPI transfer completed in {}ms", elapsed);

        // 构建响应消息
        let mut response =
          ProtocolMessage::new(message.device_id, message.operation, message.address);

        // 处理接收到的数据
        match message.operation {
          OperationType::Read => {
            // 跳过命令字节，获取数据
            if self.rx_buffer.len() > 1 {
              response.add_data(&self.rx_buffer[1..])?;
            }
          }
          OperationType::Write => {
            // 写操作通常返回状态
            if !self.rx_buffer.is_empty() {
              response.add_data(&[self.rx_buffer[0]])?;
            }
          }
          OperationType::Status => {
            // 返回状态信息
            if self.rx_buffer.len() > 1 {
              response.add_data(&self.rx_buffer[1..])?;
            }
          }
          OperationType::Reset => {
            // 复位操作返回确认
            response.add_data(&[crate::status::SUCCESS])?;
          }
        }

        response.calculate_crc();
        Ok(response)
      }
      Err(error) => {
        defmt::error!("SPI transfer error: {:?}", error);
        Err(ProtocolError::Timeout)
      }
    }
  }

  /// 扫描所有设备
  pub fn scan_devices(&mut self, current_time_ms: u32) -> Vec<u8, MAX_SPI_DEVICES> {
    let mut online_devices = Vec::new();

    for device in &mut self.devices {
      // 尝试读取设备状态
      let status_message = ProtocolMessage::new(device.config.device_id, OperationType::Status, 0);

      match self.transfer(&status_message, current_time_ms) {
        Ok(_) => {
          device.update_state(DeviceState::Idle, current_time_ms);
          let _ = online_devices.push(device.config.device_id);
        }
        Err(_) => {
          device.increment_error();
          if device.error_count > 5 {
            device.state = DeviceState::Offline;
          }
        }
      }
    }

    online_devices
  }

  /// 获取设备信息
  pub fn get_device(&self, device_id: u8) -> Option<&SpiDevice> {
    self
      .devices
      .iter()
      .find(|dev| dev.config.device_id == device_id)
  }

  /// 获取设备信息（可变引用）
  pub fn get_device_mut(&mut self, device_id: u8) -> Option<&mut SpiDevice> {
    self
      .devices
      .iter_mut()
      .find(|dev| dev.config.device_id == device_id)
  }

  /// 获取所有设备列表
  pub fn get_all_devices(&self) -> &Vec<SpiDevice, MAX_SPI_DEVICES> {
    &self.devices
  }

  /// 重置设备错误计数
  pub fn reset_device_errors(&mut self, device_id: u8) -> Result<(), ProtocolError> {
    if let Some(device) = self.get_device_mut(device_id) {
      device.error_count = 0;
      device.state = DeviceState::Idle;
      Ok(())
    } else {
      Err(ProtocolError::DeviceNotFound)
    }
  }

  /// 获取SPI统计信息
  pub fn get_stats(&self) -> SpiStats {
    let mut total_transactions = 0;
    let mut total_errors = 0;
    let mut online_devices = 0;

    for device in &self.devices {
      total_transactions += device.transaction_count;
      total_errors += device.error_count;
      if device.state != DeviceState::Offline {
        online_devices += 1;
      }
    }

    SpiStats {
      total_devices: self.devices.len() as u32,
      online_devices,
      total_transactions,
      total_errors,
      current_device: self.current_device,
    }
  }
}

/// SPI统计信息
#[derive(Debug, Clone, Copy)]
pub struct SpiStats {
  pub total_devices: u32,
  pub online_devices: u32,
  pub total_transactions: u32,
  pub total_errors: u32,
  pub current_device: Option<u8>,
}

/// SPI设备管理器
pub struct SpiDeviceManager {
  masters: Vec<SpiMaster, 2>, // 支持多个SPI总线
}

impl SpiDeviceManager {
  /// 创建新的管理器
  pub fn new() -> Self {
    Self {
      masters: Vec::new(),
    }
  }

  /// 添加SPI主设备
  pub fn add_master(&mut self, master: SpiMaster) -> Result<(), ProtocolError> {
    self
      .masters
      .push(master)
      .map_err(|_| ProtocolError::BufferOverflow)
  }

  /// 执行传输（自动选择合适的主设备）
  pub fn transfer(
    &mut self,
    message: &ProtocolMessage,
    current_time_ms: u32,
  ) -> Result<ProtocolMessage, ProtocolError> {
    // 查找包含目标设备的主设备
    for master in &mut self.masters {
      if master.get_device(message.device_id).is_some() {
        return master.transfer(message, current_time_ms);
      }
    }

    Err(ProtocolError::DeviceNotFound)
  }

  /// 扫描所有设备
  pub fn scan_all_devices(&mut self, current_time_ms: u32) -> Vec<u8, 16> {
    let mut all_devices = Vec::new();

    for master in &mut self.masters {
      let devices = master.scan_devices(current_time_ms);
      for device_id in devices {
        let _ = all_devices.push(device_id);
      }
    }

    all_devices
  }

  /// 获取所有统计信息
  pub fn get_all_stats(&self) -> Vec<SpiStats, 2> {
    let mut stats = Vec::new();
    for master in &self.masters {
      let _ = stats.push(master.get_stats());
    }
    stats
  }
}
