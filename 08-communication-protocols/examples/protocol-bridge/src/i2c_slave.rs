use crate::protocol::{OperationType, ProtocolError, ProtocolMessage};
use crate::{status, BUFFER_SIZE, COMMAND_BUFFER_SIZE, I2C_TIMEOUT_MS};
use heapless::{
  pool::{Node, Pool},
  Vec,
};
use stm32f4xx_hal::{
  gpio::{Alternate, OpenDrain, Pin},
  i2c::{Error as I2cError, Event, I2c},
  pac::I2C1,
  prelude::*,
};

/// I2C从设备状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SlaveState {
  Idle,
  ReceivingAddress,
  ReceivingData,
  SendingData,
  Error,
}

/// I2C从设备事件
#[derive(Debug, Clone, Copy)]
pub enum SlaveEvent {
  AddressMatched(bool), // true for read, false for write
  DataReceived(u8),
  DataRequested,
  Stop,
  Error(I2cError),
}

/// I2C从设备配置
#[derive(Debug, Clone, Copy)]
pub struct SlaveConfig {
  pub address: u8,
  pub enable_general_call: bool,
  pub enable_clock_stretching: bool,
  pub timeout_ms: u32,
}

impl Default for SlaveConfig {
  fn default() -> Self {
    Self {
      address: crate::DEFAULT_I2C_ADDRESS,
      enable_general_call: false,
      enable_clock_stretching: true,
      timeout_ms: I2C_TIMEOUT_MS,
    }
  }
}

/// I2C从设备驱动
pub struct I2cSlave {
  i2c: I2c<I2C1>,
  config: SlaveConfig,
  state: SlaveState,
  rx_buffer: Vec<u8, BUFFER_SIZE>,
  tx_buffer: Vec<u8, BUFFER_SIZE>,
  current_register: Option<u8>,
  bytes_received: usize,
  bytes_to_send: usize,
  bytes_sent: usize,
  last_activity_ms: u32,
}

impl I2cSlave {
  /// 创建新的I2C从设备
  pub fn new(i2c: I2c<I2C1>, config: SlaveConfig) -> Self {
    Self {
      i2c,
      config,
      state: SlaveState::Idle,
      rx_buffer: Vec::new(),
      tx_buffer: Vec::new(),
      current_register: None,
      bytes_received: 0,
      bytes_to_send: 0,
      bytes_sent: 0,
      last_activity_ms: 0,
    }
  }

  /// 初始化I2C从设备
  pub fn init(&mut self) -> Result<(), ProtocolError> {
    // 配置从设备地址
    self.set_slave_address(self.config.address)?;

    // 启用I2C从设备模式
    self.enable_slave_mode()?;

    // 启用相关中断
    self.enable_interrupts()?;

    self.state = SlaveState::Idle;
    defmt::info!(
      "I2C slave initialized at address 0x{:02X}",
      self.config.address
    );

    Ok(())
  }

  /// 设置从设备地址
  fn set_slave_address(&mut self, address: u8) -> Result<(), ProtocolError> {
    if address < crate::I2C_ADDRESS_MIN || address > crate::I2C_ADDRESS_MAX {
      return Err(ProtocolError::InvalidCommand);
    }

    // 这里应该配置硬件寄存器设置从设备地址
    // 由于HAL限制，这里只是示例代码
    self.config.address = address;

    Ok(())
  }

  /// 启用从设备模式
  fn enable_slave_mode(&mut self) -> Result<(), ProtocolError> {
    // 配置I2C为从设备模式
    // 实际实现需要直接操作寄存器
    Ok(())
  }

  /// 启用中断
  fn enable_interrupts(&mut self) -> Result<(), ProtocolError> {
    // 启用地址匹配、接收、发送等中断
    Ok(())
  }

  /// 处理I2C事件
  pub fn handle_event(
    &mut self,
    event: SlaveEvent,
    current_time_ms: u32,
  ) -> Result<Option<ProtocolMessage>, ProtocolError> {
    self.last_activity_ms = current_time_ms;

    match event {
      SlaveEvent::AddressMatched(is_read) => self.handle_address_matched(is_read),
      SlaveEvent::DataReceived(data) => self.handle_data_received(data),
      SlaveEvent::DataRequested => self.handle_data_requested(),
      SlaveEvent::Stop => self.handle_stop(),
      SlaveEvent::Error(error) => self.handle_error(error),
    }
  }

  /// 处理地址匹配事件
  fn handle_address_matched(
    &mut self,
    is_read: bool,
  ) -> Result<Option<ProtocolMessage>, ProtocolError> {
    match self.state {
      SlaveState::Idle => {
        if is_read {
          self.state = SlaveState::SendingData;
          self.bytes_sent = 0;
        } else {
          self.state = SlaveState::ReceivingAddress;
          self.rx_buffer.clear();
          self.bytes_received = 0;
        }
        Ok(None)
      }
      _ => {
        self.state = SlaveState::Error;
        Err(ProtocolError::InvalidCommand)
      }
    }
  }

  /// 处理接收数据事件
  fn handle_data_received(&mut self, data: u8) -> Result<Option<ProtocolMessage>, ProtocolError> {
    match self.state {
      SlaveState::ReceivingAddress => {
        self.current_register = Some(data);
        self.state = SlaveState::ReceivingData;
        Ok(None)
      }
      SlaveState::ReceivingData => {
        if self.rx_buffer.push(data).is_err() {
          self.state = SlaveState::Error;
          return Err(ProtocolError::BufferOverflow);
        }
        self.bytes_received += 1;
        Ok(None)
      }
      _ => {
        self.state = SlaveState::Error;
        Err(ProtocolError::InvalidCommand)
      }
    }
  }

  /// 处理数据请求事件
  fn handle_data_requested(&mut self) -> Result<Option<ProtocolMessage>, ProtocolError> {
    match self.state {
      SlaveState::SendingData => {
        if self.bytes_sent < self.tx_buffer.len() {
          let data = self.tx_buffer[self.bytes_sent];
          self.bytes_sent += 1;

          // 这里应该将数据写入I2C数据寄存器
          // 由于HAL限制，这里只是示例

          Ok(None)
        } else {
          // 没有更多数据发送
          self.state = SlaveState::Idle;
          Ok(None)
        }
      }
      _ => {
        self.state = SlaveState::Error;
        Err(ProtocolError::InvalidCommand)
      }
    }
  }

  /// 处理停止条件事件
  fn handle_stop(&mut self) -> Result<Option<ProtocolMessage>, ProtocolError> {
    let result = match self.state {
      SlaveState::ReceivingData => {
        // 构建协议消息
        if let Some(register) = self.current_register {
          let mut message =
            ProtocolMessage::new(self.config.address, OperationType::Write, register as u16);

          message.add_data(&self.rx_buffer)?;
          message.calculate_crc();

          Some(message)
        } else {
          None
        }
      }
      SlaveState::SendingData => {
        // 发送完成
        None
      }
      _ => None,
    };

    // 重置状态
    self.state = SlaveState::Idle;
    self.current_register = None;
    self.rx_buffer.clear();
    self.bytes_received = 0;
    self.bytes_sent = 0;

    Ok(result)
  }

  /// 处理错误事件
  fn handle_error(&mut self, error: I2cError) -> Result<Option<ProtocolMessage>, ProtocolError> {
    defmt::error!("I2C slave error: {:?}", error);

    self.state = SlaveState::Error;

    // 尝试恢复
    self.recover_from_error()?;

    Err(ProtocolError::Timeout)
  }

  /// 从错误中恢复
  fn recover_from_error(&mut self) -> Result<(), ProtocolError> {
    // 清除错误状态
    self.state = SlaveState::Idle;
    self.current_register = None;
    self.rx_buffer.clear();
    self.tx_buffer.clear();
    self.bytes_received = 0;
    self.bytes_sent = 0;

    // 重新初始化I2C外设
    // 这里应该重置I2C寄存器

    defmt::info!("I2C slave recovered from error");
    Ok(())
  }

  /// 准备发送数据
  pub fn prepare_response(&mut self, data: &[u8]) -> Result<(), ProtocolError> {
    self.tx_buffer.clear();

    for &byte in data {
      self
        .tx_buffer
        .push(byte)
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }

    self.bytes_to_send = self.tx_buffer.len();
    self.bytes_sent = 0;

    Ok(())
  }

  /// 检查超时
  pub fn check_timeout(&mut self, current_time_ms: u32) -> bool {
    if self.state != SlaveState::Idle {
      let elapsed = current_time_ms.wrapping_sub(self.last_activity_ms);
      if elapsed > self.config.timeout_ms {
        defmt::warn!("I2C slave timeout, resetting state");
        self.state = SlaveState::Idle;
        self.current_register = None;
        self.rx_buffer.clear();
        self.bytes_received = 0;
        return true;
      }
    }
    false
  }

  /// 获取当前状态
  pub fn get_state(&self) -> SlaveState {
    self.state
  }

  /// 获取统计信息
  pub fn get_stats(&self) -> SlaveStats {
    SlaveStats {
      state: self.state,
      bytes_received: self.bytes_received,
      bytes_sent: self.bytes_sent,
      buffer_usage: self.rx_buffer.len(),
      last_activity_ms: self.last_activity_ms,
    }
  }
}

/// I2C从设备统计信息
#[derive(Debug, Clone, Copy)]
pub struct SlaveStats {
  pub state: SlaveState,
  pub bytes_received: usize,
  pub bytes_sent: usize,
  pub buffer_usage: usize,
  pub last_activity_ms: u32,
}

/// I2C从设备管理器
pub struct I2cSlaveManager {
  slaves: Vec<I2cSlave, 4>,
  message_pool: Pool<Node<ProtocolMessage>>,
}

impl I2cSlaveManager {
  /// 创建新的管理器
  pub fn new() -> Self {
    Self {
      slaves: Vec::new(),
      message_pool: Pool::new(),
    }
  }

  /// 添加从设备
  pub fn add_slave(&mut self, slave: I2cSlave) -> Result<(), ProtocolError> {
    self
      .slaves
      .push(slave)
      .map_err(|_| ProtocolError::BufferOverflow)
  }

  /// 处理所有从设备的事件
  pub fn handle_events(&mut self, current_time_ms: u32) -> Vec<ProtocolMessage, 8> {
    let mut messages = Vec::new();

    for slave in &mut self.slaves {
      // 检查超时
      slave.check_timeout(current_time_ms);

      // 这里应该检查硬件中断标志并处理相应事件
      // 由于HAL限制，这里只是示例代码
    }

    messages
  }

  /// 获取指定地址的从设备
  pub fn get_slave_mut(&mut self, address: u8) -> Option<&mut I2cSlave> {
    self
      .slaves
      .iter_mut()
      .find(|slave| slave.config.address == address)
  }

  /// 获取所有从设备统计信息
  pub fn get_all_stats(&self) -> Vec<SlaveStats, 4> {
    let mut stats = Vec::new();
    for slave in &self.slaves {
      let _ = stats.push(slave.get_stats());
    }
    stats
  }
}
