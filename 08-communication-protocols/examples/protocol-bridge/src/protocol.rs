use crate::{commands, status, BUFFER_SIZE, CRC_POLYNOMIAL};
use heapless::Vec;

/// 协议转换错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProtocolError {
  InvalidCommand,
  InvalidLength,
  CrcError,
  BufferOverflow,
  DeviceNotFound,
  Timeout,
}

/// 设备类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeviceType {
  I2c,
  Spi,
}

/// 操作类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperationType {
  Read,
  Write,
  Status,
  Reset,
}

/// 协议消息结构
#[derive(Debug, Clone)]
pub struct ProtocolMessage {
  pub device_id: u8,
  pub operation: OperationType,
  pub address: u16,
  pub data: Vec<u8, BUFFER_SIZE>,
  pub crc: Option<u16>,
}

impl ProtocolMessage {
  /// 创建新的协议消息
  pub fn new(device_id: u8, operation: OperationType, address: u16) -> Self {
    Self {
      device_id,
      operation,
      address,
      data: Vec::new(),
      crc: None,
    }
  }

  /// 添加数据到消息
  pub fn add_data(&mut self, data: &[u8]) -> Result<(), ProtocolError> {
    for &byte in data {
      self
        .data
        .push(byte)
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }
    Ok(())
  }

  /// 计算并设置CRC
  pub fn calculate_crc(&mut self) {
    let mut crc = 0xFFFFu16;

    // 包含设备ID、操作类型、地址
    crc = update_crc(crc, self.device_id);
    crc = update_crc(crc, self.operation as u8);
    crc = update_crc(crc, (self.address >> 8) as u8);
    crc = update_crc(crc, (self.address & 0xFF) as u8);

    // 包含数据
    for &byte in &self.data {
      crc = update_crc(crc, byte);
    }

    self.crc = Some(crc);
  }

  /// 验证CRC
  pub fn verify_crc(&self) -> bool {
    if let Some(expected_crc) = self.crc {
      let mut calculated_crc = 0xFFFFu16;

      calculated_crc = update_crc(calculated_crc, self.device_id);
      calculated_crc = update_crc(calculated_crc, self.operation as u8);
      calculated_crc = update_crc(calculated_crc, (self.address >> 8) as u8);
      calculated_crc = update_crc(calculated_crc, (self.address & 0xFF) as u8);

      for &byte in &self.data {
        calculated_crc = update_crc(calculated_crc, byte);
      }

      calculated_crc == expected_crc
    } else {
      false
    }
  }

  /// 序列化为字节数组
  pub fn serialize(&self) -> Result<Vec<u8, BUFFER_SIZE>, ProtocolError> {
    let mut buffer = Vec::new();

    // 添加头部信息
    buffer
      .push(self.device_id)
      .map_err(|_| ProtocolError::BufferOverflow)?;
    buffer
      .push(self.operation as u8)
      .map_err(|_| ProtocolError::BufferOverflow)?;
    buffer
      .push((self.address >> 8) as u8)
      .map_err(|_| ProtocolError::BufferOverflow)?;
    buffer
      .push((self.address & 0xFF) as u8)
      .map_err(|_| ProtocolError::BufferOverflow)?;
    buffer
      .push(self.data.len() as u8)
      .map_err(|_| ProtocolError::BufferOverflow)?;

    // 添加数据
    for &byte in &self.data {
      buffer
        .push(byte)
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }

    // 添加CRC
    if let Some(crc) = self.crc {
      buffer
        .push((crc >> 8) as u8)
        .map_err(|_| ProtocolError::BufferOverflow)?;
      buffer
        .push((crc & 0xFF) as u8)
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }

    Ok(buffer)
  }

  /// 从字节数组反序列化
  pub fn deserialize(buffer: &[u8]) -> Result<Self, ProtocolError> {
    if buffer.len() < 7 {
      // 最小长度：设备ID + 操作 + 地址(2) + 长度 + CRC(2)
      return Err(ProtocolError::InvalidLength);
    }

    let device_id = buffer[0];
    let operation = match buffer[1] {
      op if op & commands::READ_CMD != 0 => OperationType::Read,
      commands::WRITE_CMD => OperationType::Write,
      commands::STATUS_CMD => OperationType::Status,
      commands::RESET_CMD => OperationType::Reset,
      _ => return Err(ProtocolError::InvalidCommand),
    };

    let address = ((buffer[2] as u16) << 8) | (buffer[3] as u16);
    let data_len = buffer[4] as usize;

    if buffer.len() < 5 + data_len + 2 {
      return Err(ProtocolError::InvalidLength);
    }

    let mut data = Vec::new();
    for i in 0..data_len {
      data
        .push(buffer[5 + i])
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }

    let crc = ((buffer[5 + data_len] as u16) << 8) | (buffer[5 + data_len + 1] as u16);

    let mut message = Self {
      device_id,
      operation,
      address,
      data,
      crc: Some(crc),
    };

    if !message.verify_crc() {
      return Err(ProtocolError::CrcError);
    }

    Ok(message)
  }
}

/// I2C到SPI协议转换器
pub struct I2cToSpiConverter {
  device_mappings: Vec<(u8, u8), 16>, // (I2C地址, SPI设备ID)
}

impl I2cToSpiConverter {
  /// 创建新的转换器
  pub fn new() -> Self {
    Self {
      device_mappings: Vec::new(),
    }
  }

  /// 添加设备映射
  pub fn add_mapping(&mut self, i2c_addr: u8, spi_device_id: u8) -> Result<(), ProtocolError> {
    self
      .device_mappings
      .push((i2c_addr, spi_device_id))
      .map_err(|_| ProtocolError::BufferOverflow)
  }

  /// 转换I2C消息到SPI消息
  pub fn convert_i2c_to_spi(&self, i2c_data: &[u8]) -> Result<ProtocolMessage, ProtocolError> {
    if i2c_data.len() < 3 {
      return Err(ProtocolError::InvalidLength);
    }

    let i2c_addr = i2c_data[0] >> 1; // 移除读写位
    let register_addr = i2c_data[1];
    let is_read = (i2c_data[0] & 0x01) != 0;

    // 查找设备映射
    let spi_device_id = self
      .device_mappings
      .iter()
      .find(|(addr, _)| *addr == i2c_addr)
      .map(|(_, id)| *id)
      .ok_or(ProtocolError::DeviceNotFound)?;

    let operation = if is_read {
      OperationType::Read
    } else {
      OperationType::Write
    };

    let mut message = ProtocolMessage::new(spi_device_id, operation, register_addr as u16);

    // 添加数据（如果是写操作）
    if !is_read && i2c_data.len() > 2 {
      message.add_data(&i2c_data[2..])?;
    }

    message.calculate_crc();
    Ok(message)
  }

  /// 转换SPI响应到I2C格式
  pub fn convert_spi_to_i2c(
    &self,
    spi_message: &ProtocolMessage,
  ) -> Result<Vec<u8, BUFFER_SIZE>, ProtocolError> {
    let mut i2c_response = Vec::new();

    // 添加状态字节
    i2c_response
      .push(status::SUCCESS)
      .map_err(|_| ProtocolError::BufferOverflow)?;

    // 添加数据
    for &byte in &spi_message.data {
      i2c_response
        .push(byte)
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }

    Ok(i2c_response)
  }
}

/// SPI到I2C协议转换器
pub struct SpiToI2cConverter {
  device_mappings: Vec<(u8, u8), 16>, // (SPI设备ID, I2C地址)
}

impl SpiToI2cConverter {
  /// 创建新的转换器
  pub fn new() -> Self {
    Self {
      device_mappings: Vec::new(),
    }
  }

  /// 添加设备映射
  pub fn add_mapping(&mut self, spi_device_id: u8, i2c_addr: u8) -> Result<(), ProtocolError> {
    self
      .device_mappings
      .push((spi_device_id, i2c_addr))
      .map_err(|_| ProtocolError::BufferOverflow)
  }

  /// 转换SPI消息到I2C格式
  pub fn convert_spi_to_i2c(
    &self,
    spi_message: &ProtocolMessage,
  ) -> Result<Vec<u8, BUFFER_SIZE>, ProtocolError> {
    // 查找设备映射
    let i2c_addr = self
      .device_mappings
      .iter()
      .find(|(id, _)| *id == spi_message.device_id)
      .map(|(_, addr)| *addr)
      .ok_or(ProtocolError::DeviceNotFound)?;

    let mut i2c_data = Vec::new();

    // 添加I2C地址（包含读写位）
    let addr_with_rw = (i2c_addr << 1)
      | match spi_message.operation {
        OperationType::Read => 0x01,
        _ => 0x00,
      };
    i2c_data
      .push(addr_with_rw)
      .map_err(|_| ProtocolError::BufferOverflow)?;

    // 添加寄存器地址
    i2c_data
      .push((spi_message.address & 0xFF) as u8)
      .map_err(|_| ProtocolError::BufferOverflow)?;

    // 添加数据
    for &byte in &spi_message.data {
      i2c_data
        .push(byte)
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }

    Ok(i2c_data)
  }
}

/// CRC计算辅助函数
fn update_crc(crc: u16, data: u8) -> u16 {
  let mut crc = crc;
  crc ^= (data as u16) << 8;

  for _ in 0..8 {
    if (crc & 0x8000) != 0 {
      crc = (crc << 1) ^ CRC_POLYNOMIAL;
    } else {
      crc <<= 1;
    }
  }

  crc
}

/// 协议统计信息
#[derive(Debug, Clone, Copy)]
pub struct ProtocolStats {
  pub messages_converted: u32,
  pub conversion_errors: u32,
  pub crc_errors: u32,
  pub timeout_errors: u32,
  pub last_conversion_time_us: u32,
  pub average_conversion_time_us: u32,
}

impl ProtocolStats {
  /// 创建新的统计信息
  pub fn new() -> Self {
    Self {
      messages_converted: 0,
      conversion_errors: 0,
      crc_errors: 0,
      timeout_errors: 0,
      last_conversion_time_us: 0,
      average_conversion_time_us: 0,
    }
  }

  /// 更新转换统计
  pub fn update_conversion(&mut self, conversion_time_us: u32, success: bool) {
    if success {
      self.messages_converted += 1;
      self.last_conversion_time_us = conversion_time_us;

      // 计算平均转换时间
      if self.messages_converted == 1 {
        self.average_conversion_time_us = conversion_time_us;
      } else {
        self.average_conversion_time_us =
          (self.average_conversion_time_us * (self.messages_converted - 1) + conversion_time_us)
            / self.messages_converted;
      }
    } else {
      self.conversion_errors += 1;
    }
  }

  /// 更新CRC错误统计
  pub fn update_crc_error(&mut self) {
    self.crc_errors += 1;
  }

  /// 更新超时错误统计
  pub fn update_timeout_error(&mut self) {
    self.timeout_errors += 1;
  }

  /// 重置统计信息
  pub fn reset(&mut self) {
    *self = Self::new();
  }
}
