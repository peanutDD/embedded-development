use crate::i2c_slave::{I2cSlaveManager, SlaveEvent};
use crate::protocol::{
  I2cToSpiConverter, OperationType, ProtocolError, ProtocolMessage, ProtocolStats,
  SpiToI2cConverter,
};
use crate::spi_master::SpiDeviceManager;
use crate::{BUFFER_SIZE, CONVERSION_TIMEOUT_MS};
use heapless::{Deque, Vec};

/// 桥接操作类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BridgeOperation {
  I2cToSpi,
  SpiToI2c,
  Bidirectional,
}

/// 桥接状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BridgeState {
  Idle,
  Processing,
  Error,
  Disabled,
}

/// 桥接请求
#[derive(Debug, Clone)]
pub struct BridgeRequest {
  pub id: u32,
  pub operation: BridgeOperation,
  pub source_message: ProtocolMessage,
  pub timestamp_ms: u32,
  pub priority: u8,
}

impl BridgeRequest {
  /// 创建新的桥接请求
  pub fn new(
    id: u32,
    operation: BridgeOperation,
    source_message: ProtocolMessage,
    timestamp_ms: u32,
  ) -> Self {
    Self {
      id,
      operation,
      source_message,
      timestamp_ms,
      priority: 0,
    }
  }

  /// 设置优先级
  pub fn with_priority(mut self, priority: u8) -> Self {
    self.priority = priority;
    self
  }

  /// 检查是否超时
  pub fn is_expired(&self, current_time_ms: u32, timeout_ms: u32) -> bool {
    current_time_ms.wrapping_sub(self.timestamp_ms) > timeout_ms
  }
}

/// 桥接响应
#[derive(Debug, Clone)]
pub struct BridgeResponse {
  pub request_id: u32,
  pub result: Result<ProtocolMessage, ProtocolError>,
  pub processing_time_us: u32,
  pub timestamp_ms: u32,
}

/// 协议桥接器
pub struct ProtocolBridge {
  state: BridgeState,
  i2c_to_spi_converter: I2cToSpiConverter,
  spi_to_i2c_converter: SpiToI2cConverter,
  i2c_manager: I2cSlaveManager,
  spi_manager: SpiDeviceManager,
  request_queue: Deque<BridgeRequest, 16>,
  response_queue: Deque<BridgeResponse, 16>,
  next_request_id: u32,
  stats: BridgeStats,
  config: BridgeConfig,
}

/// 桥接配置
#[derive(Debug, Clone, Copy)]
pub struct BridgeConfig {
  pub max_queue_size: usize,
  pub timeout_ms: u32,
  pub enable_crc_check: bool,
  pub enable_auto_retry: bool,
  pub max_retry_count: u8,
  pub priority_scheduling: bool,
}

impl Default for BridgeConfig {
  fn default() -> Self {
    Self {
      max_queue_size: 16,
      timeout_ms: CONVERSION_TIMEOUT_MS,
      enable_crc_check: true,
      enable_auto_retry: true,
      max_retry_count: 3,
      priority_scheduling: true,
    }
  }
}

/// 桥接统计信息
#[derive(Debug, Clone, Copy)]
pub struct BridgeStats {
  pub total_requests: u32,
  pub successful_conversions: u32,
  pub failed_conversions: u32,
  pub timeout_errors: u32,
  pub crc_errors: u32,
  pub queue_overflows: u32,
  pub average_processing_time_us: u32,
  pub max_processing_time_us: u32,
  pub current_queue_size: usize,
  pub protocol_stats: ProtocolStats,
}

impl BridgeStats {
  /// 创建新的统计信息
  pub fn new() -> Self {
    Self {
      total_requests: 0,
      successful_conversions: 0,
      failed_conversions: 0,
      timeout_errors: 0,
      crc_errors: 0,
      queue_overflows: 0,
      average_processing_time_us: 0,
      max_processing_time_us: 0,
      current_queue_size: 0,
      protocol_stats: ProtocolStats::new(),
    }
  }

  /// 更新处理统计
  pub fn update_processing(&mut self, processing_time_us: u32, success: bool) {
    self.total_requests += 1;

    if success {
      self.successful_conversions += 1;
    } else {
      self.failed_conversions += 1;
    }

    // 更新处理时间统计
    if processing_time_us > self.max_processing_time_us {
      self.max_processing_time_us = processing_time_us;
    }

    if self.total_requests == 1 {
      self.average_processing_time_us = processing_time_us;
    } else {
      self.average_processing_time_us =
        (self.average_processing_time_us * (self.total_requests - 1) + processing_time_us)
          / self.total_requests;
    }
  }

  /// 重置统计信息
  pub fn reset(&mut self) {
    *self = Self::new();
  }
}

impl ProtocolBridge {
  /// 创建新的协议桥接器
  pub fn new(
    i2c_manager: I2cSlaveManager,
    spi_manager: SpiDeviceManager,
    config: BridgeConfig,
  ) -> Self {
    Self {
      state: BridgeState::Idle,
      i2c_to_spi_converter: I2cToSpiConverter::new(),
      spi_to_i2c_converter: SpiToI2cConverter::new(),
      i2c_manager,
      spi_manager,
      request_queue: Deque::new(),
      response_queue: Deque::new(),
      next_request_id: 1,
      stats: BridgeStats::new(),
      config,
    }
  }

  /// 初始化桥接器
  pub fn init(&mut self) -> Result<(), ProtocolError> {
    self.state = BridgeState::Idle;

    // 设置设备映射
    self.setup_device_mappings()?;

    defmt::info!("Protocol bridge initialized");
    Ok(())
  }

  /// 设置设备映射
  fn setup_device_mappings(&mut self) -> Result<(), ProtocolError> {
    // 添加I2C到SPI的映射
    self.i2c_to_spi_converter.add_mapping(0x48, 1)?; // EEPROM
    self.i2c_to_spi_converter.add_mapping(0x50, 2)?; // Flash
    self.i2c_to_spi_converter.add_mapping(0x68, 3)?; // RTC

    // 添加SPI到I2C的映射
    self.spi_to_i2c_converter.add_mapping(1, 0x48)?;
    self.spi_to_i2c_converter.add_mapping(2, 0x50)?;
    self.spi_to_i2c_converter.add_mapping(3, 0x68)?;

    Ok(())
  }

  /// 添加桥接请求
  pub fn add_request(
    &mut self,
    operation: BridgeOperation,
    message: ProtocolMessage,
    current_time_ms: u32,
  ) -> Result<u32, ProtocolError> {
    if self.request_queue.len() >= self.config.max_queue_size {
      self.stats.queue_overflows += 1;
      return Err(ProtocolError::BufferOverflow);
    }

    let request_id = self.next_request_id;
    self.next_request_id = self.next_request_id.wrapping_add(1);

    let request = BridgeRequest::new(request_id, operation, message, current_time_ms);

    if self.config.priority_scheduling {
      self.insert_request_by_priority(request)?;
    } else {
      self
        .request_queue
        .push_back(request)
        .map_err(|_| ProtocolError::BufferOverflow)?;
    }

    self.stats.current_queue_size = self.request_queue.len();
    Ok(request_id)
  }

  /// 按优先级插入请求
  fn insert_request_by_priority(&mut self, request: BridgeRequest) -> Result<(), ProtocolError> {
    // 简单的优先级调度：高优先级插入到队列前面
    if request.priority > 5 {
      self
        .request_queue
        .push_front(request)
        .map_err(|_| ProtocolError::BufferOverflow)
    } else {
      self
        .request_queue
        .push_back(request)
        .map_err(|_| ProtocolError::BufferOverflow)
    }
  }

  /// 处理桥接请求
  pub fn process_requests(&mut self, current_time_ms: u32) -> Vec<BridgeResponse, 8> {
    let mut responses = Vec::new();

    // 处理I2C事件
    let i2c_messages = self.i2c_manager.handle_events(current_time_ms);
    for message in i2c_messages {
      if let Ok(request_id) = self.add_request(BridgeOperation::I2cToSpi, message, current_time_ms)
      {
        defmt::debug!("Added I2C->SPI request {}", request_id);
      }
    }

    // 处理队列中的请求
    while let Some(request) = self.request_queue.pop_front() {
      // 检查超时
      if request.is_expired(current_time_ms, self.config.timeout_ms) {
        self.stats.timeout_errors += 1;
        let response = BridgeResponse {
          request_id: request.id,
          result: Err(ProtocolError::Timeout),
          processing_time_us: 0,
          timestamp_ms: current_time_ms,
        };
        let _ = responses.push(response);
        continue;
      }

      // 处理请求
      let start_time_us = current_time_ms * 1000; // 简化的微秒时间
      let result = self.process_single_request(&request, current_time_ms);
      let processing_time_us = (current_time_ms * 1000).wrapping_sub(start_time_us);

      // 更新统计
      self
        .stats
        .update_processing(processing_time_us, result.is_ok());
      if result.is_ok() {
        self
          .stats
          .protocol_stats
          .update_conversion(processing_time_us, true);
      } else {
        self
          .stats
          .protocol_stats
          .update_conversion(processing_time_us, false);
      }

      let response = BridgeResponse {
        request_id: request.id,
        result,
        processing_time_us,
        timestamp_ms: current_time_ms,
      };

      let _ = responses.push(response);

      // 限制单次处理的请求数量
      if responses.len() >= 4 {
        break;
      }
    }

    self.stats.current_queue_size = self.request_queue.len();
    responses
  }

  /// 处理单个桥接请求
  fn process_single_request(
    &mut self,
    request: &BridgeRequest,
    current_time_ms: u32,
  ) -> Result<ProtocolMessage, ProtocolError> {
    self.state = BridgeState::Processing;

    let result = match request.operation {
      BridgeOperation::I2cToSpi => {
        self.process_i2c_to_spi(&request.source_message, current_time_ms)
      }
      BridgeOperation::SpiToI2c => {
        self.process_spi_to_i2c(&request.source_message, current_time_ms)
      }
      BridgeOperation::Bidirectional => {
        // 根据消息来源决定转换方向
        if request.source_message.device_id < 0x80 {
          self.process_i2c_to_spi(&request.source_message, current_time_ms)
        } else {
          self.process_spi_to_i2c(&request.source_message, current_time_ms)
        }
      }
    };

    self.state = if result.is_ok() {
      BridgeState::Idle
    } else {
      BridgeState::Error
    };
    result
  }

  /// 处理I2C到SPI的转换
  fn process_i2c_to_spi(
    &mut self,
    i2c_message: &ProtocolMessage,
    current_time_ms: u32,
  ) -> Result<ProtocolMessage, ProtocolError> {
    // 序列化I2C消息
    let i2c_data = i2c_message.serialize()?;

    // 转换为SPI消息
    let spi_message = self.i2c_to_spi_converter.convert_i2c_to_spi(&i2c_data)?;

    // 验证CRC（如果启用）
    if self.config.enable_crc_check && !spi_message.verify_crc() {
      self.stats.crc_errors += 1;
      self.stats.protocol_stats.update_crc_error();
      return Err(ProtocolError::CrcError);
    }

    // 执行SPI传输
    let spi_response = self.spi_manager.transfer(&spi_message, current_time_ms)?;

    // 转换SPI响应为I2C格式
    let i2c_response_data = self
      .i2c_to_spi_converter
      .convert_spi_to_i2c(&spi_response)?;

    // 构建响应消息
    let mut response = ProtocolMessage::new(
      i2c_message.device_id,
      i2c_message.operation,
      i2c_message.address,
    );
    response.add_data(&i2c_response_data)?;
    response.calculate_crc();

    // 准备I2C从设备响应
    if let Some(slave) = self.i2c_manager.get_slave_mut(i2c_message.device_id) {
      slave.prepare_response(&i2c_response_data)?;
    }

    Ok(response)
  }

  /// 处理SPI到I2C的转换
  fn process_spi_to_i2c(
    &mut self,
    spi_message: &ProtocolMessage,
    current_time_ms: u32,
  ) -> Result<ProtocolMessage, ProtocolError> {
    // 转换SPI消息为I2C格式
    let i2c_data = self.spi_to_i2c_converter.convert_spi_to_i2c(spi_message)?;

    // 这里应该执行I2C主设备传输
    // 由于当前实现是I2C从设备，这里只是示例

    // 构建响应消息
    let mut response = ProtocolMessage::new(
      spi_message.device_id,
      spi_message.operation,
      spi_message.address,
    );
    response.add_data(&[crate::status::SUCCESS])?;
    response.calculate_crc();

    Ok(response)
  }

  /// 获取响应
  pub fn get_response(&mut self, request_id: u32) -> Option<BridgeResponse> {
    // 查找并移除匹配的响应
    if let Some(pos) = self
      .response_queue
      .iter()
      .position(|r| r.request_id == request_id)
    {
      // 由于Deque没有remove方法，我们需要重新构建队列
      let mut temp_queue = Deque::new();
      let mut target_response = None;

      while let Some(response) = self.response_queue.pop_front() {
        if response.request_id == request_id && target_response.is_none() {
          target_response = Some(response);
        } else {
          let _ = temp_queue.push_back(response);
        }
      }

      // 恢复队列
      while let Some(response) = temp_queue.pop_front() {
        let _ = self.response_queue.push_back(response);
      }

      target_response
    } else {
      None
    }
  }

  /// 清理过期的响应
  pub fn cleanup_expired_responses(&mut self, current_time_ms: u32) {
    let mut temp_queue = Deque::new();

    while let Some(response) = self.response_queue.pop_front() {
      let elapsed = current_time_ms.wrapping_sub(response.timestamp_ms);
      if elapsed < self.config.timeout_ms * 2 {
        // 保留响应2倍超时时间
        let _ = temp_queue.push_back(response);
      }
    }

    self.response_queue = temp_queue;
  }

  /// 获取桥接器状态
  pub fn get_state(&self) -> BridgeState {
    self.state
  }

  /// 获取统计信息
  pub fn get_stats(&self) -> &BridgeStats {
    &self.stats
  }

  /// 重置统计信息
  pub fn reset_stats(&mut self) {
    self.stats.reset();
  }

  /// 启用/禁用桥接器
  pub fn set_enabled(&mut self, enabled: bool) {
    self.state = if enabled {
      BridgeState::Idle
    } else {
      BridgeState::Disabled
    };
  }

  /// 检查桥接器是否启用
  pub fn is_enabled(&self) -> bool {
    self.state != BridgeState::Disabled
  }

  /// 获取队列状态
  pub fn get_queue_status(&self) -> (usize, usize) {
    (self.request_queue.len(), self.response_queue.len())
  }

  /// 清空所有队列
  pub fn clear_queues(&mut self) {
    self.request_queue.clear();
    self.response_queue.clear();
    self.stats.current_queue_size = 0;
  }
}
