//! # 原始数据协议
//!
//! 提供最基本的原始数据传输协议，不进行任何数据处理。

use super::{
    MessageType, ProtocolConfig, ProtocolHandler, ProtocolMessage, ProtocolState, ProtocolStats,
    ProtocolType,
};
use crate::error::Result;
use core::time::Duration;
use std::time::Instant;

/// 原始协议配置
#[derive(Debug, Clone)]
pub struct RawProtocolConfig {
    /// 超时时间
    pub timeout: Duration,
    /// 最大消息长度
    pub max_message_length: usize,
    /// 缓冲区大小
    pub buffer_size: usize,
    /// 是否启用校验
    pub enable_checksum: bool,
    /// 是否启用时间戳
    pub enable_timestamp: bool,
    /// 批处理大小
    pub batch_size: usize,
}

impl Default for RawProtocolConfig {
    fn default() -> Self {
        Self {
            timeout: Duration::from_secs(5),
            max_message_length: 4096,
            buffer_size: 8192,
            enable_checksum: false,
            enable_timestamp: true,
            batch_size: 1,
        }
    }
}

impl ProtocolConfig for RawProtocolConfig {
    fn timeout(&self) -> Duration {
        self.timeout
    }
    
    fn max_message_length(&self) -> usize {
        self.max_message_length
    }
    
    fn buffer_size(&self) -> usize {
        self.buffer_size
    }
    
    fn enable_checksum(&self) -> bool {
        self.enable_checksum
    }
    
    fn clone_box(&self) -> Box<dyn ProtocolConfig> {
        Box::new(self.clone())
    }
}

/// 原始协议处理器
pub struct RawProtocolHandler {
    /// 协议配置
    config: RawProtocolConfig,
    /// 当前状态
    state: ProtocolState,
    /// 统计信息
    stats: ProtocolStats,
    /// 接收缓冲区
    rx_buffer: Vec<u8>,
    /// 最后活动时间
    last_activity: Option<Instant>,
    /// 消息ID计数器
    message_id_counter: u32,
}

impl RawProtocolHandler {
    /// 创建新的原始协议处理器
    pub fn new() -> Self {
        Self::with_config(RawProtocolConfig::default())
    }
    
    /// 使用指定配置创建处理器
    pub fn with_config(config: RawProtocolConfig) -> Self {
        Self {
            config,
            state: ProtocolState::Idle,
            stats: ProtocolStats::default(),
            rx_buffer: Vec::new(),
            last_activity: None,
            message_id_counter: 0,
        }
    }
    
    /// 生成下一个消息ID
    fn next_message_id(&mut self) -> u32 {
        self.message_id_counter = self.message_id_counter.wrapping_add(1);
        self.message_id_counter
    }
    
    /// 更新活动时间
    fn update_activity(&mut self) {
        self.last_activity = Some(Instant::now());
    }
    
    /// 检查是否超时
    fn is_timeout(&self) -> bool {
        if let Some(last_activity) = self.last_activity {
            last_activity.elapsed() > self.config.timeout
        } else {
            false
        }
    }
    
    /// 处理单个数据块
    fn process_data_chunk(&mut self, data: &[u8]) -> Result<Vec<ProtocolMessage>> {
        let start_time = Instant::now();
        
        // 检查数据长度
        if data.len() > self.config.max_message_length {
            self.stats.update_error_stats();
            return Err(crate::error::ProtocolError::MessageTooLarge.into());
        }
        
        // 创建消息
        let mut message = ProtocolMessage::new(MessageType::Data, data.to_vec());
        
        // 添加ID
        message.id = Some(self.next_message_id());
        
        // 添加时间戳
        if self.config.enable_timestamp {
            message.timestamp = Some(Instant::now());
        }
        
        // 添加元数据
        message.metadata.insert("protocol".to_string(), "raw".to_string());
        message.metadata.insert("length".to_string(), data.len().to_string());
        
        // 更新统计
        let processing_time = start_time.elapsed().as_micros() as u64;
        self.stats.update_message_stats(false, true);
        self.stats.update_bytes_stats(data.len() as u64);
        self.stats.update_processing_time(processing_time);
        self.update_activity();
        
        Ok(vec![message])
    }
    
    /// 批处理数据
    fn batch_process_data(&mut self, data: &[u8]) -> Result<Vec<ProtocolMessage>> {
        if self.config.batch_size <= 1 || data.len() <= self.config.batch_size {
            return self.process_data_chunk(data);
        }
        
        let mut messages = Vec::new();
        let mut offset = 0;
        
        while offset < data.len() {
            let end = (offset + self.config.batch_size).min(data.len());
            let chunk = &data[offset..end];
            
            let mut chunk_messages = self.process_data_chunk(chunk)?;
            messages.append(&mut chunk_messages);
            
            offset = end;
        }
        
        Ok(messages)
    }
}

impl Default for RawProtocolHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolHandler for RawProtocolHandler {
    fn protocol_type(&self) -> ProtocolType {
        ProtocolType::Raw
    }
    
    fn state(&self) -> ProtocolState {
        self.state
    }
    
    async fn handle_received_data(&mut self, data: &[u8]) -> Result<Vec<ProtocolMessage>> {
        if data.is_empty() {
            return Ok(Vec::new());
        }
        
        self.state = ProtocolState::ProcessingData;
        
        // 将数据添加到缓冲区
        if self.rx_buffer.len() + data.len() > self.config.buffer_size {
            self.stats.update_error_stats();
            self.state = ProtocolState::Error;
            return Err(crate::error::ProtocolError::BufferOverflow.into());
        }
        
        self.rx_buffer.extend_from_slice(data);
        
        // 处理缓冲区中的数据
        let buffer_data = self.rx_buffer.clone();
        self.rx_buffer.clear();
        
        let result = self.batch_process_data(&buffer_data);
        
        match result {
            Ok(messages) => {
                self.state = if messages.is_empty() {
                    ProtocolState::WaitingForData
                } else {
                    ProtocolState::Complete
                };
                Ok(messages)
            }
            Err(e) => {
                self.state = ProtocolState::Error;
                Err(e)
            }
        }
    }
    
    async fn encode_message(&self, message: &ProtocolMessage) -> Result<Vec<u8>> {
        let start_time = Instant::now();
        
        // 检查消息长度
        if message.data.len() > self.config.max_message_length {
            return Err(crate::error::ProtocolError::MessageTooLarge.into());
        }
        
        // 原始协议直接返回数据
        let mut encoded = message.data.clone();
        
        // 如果启用校验，添加简单的校验和
        if self.config.enable_checksum {
            let checksum = encoded.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
            encoded.push(checksum);
        }
        
        // 更新统计（这里是发送统计）
        let processing_time = start_time.elapsed().as_micros() as u64;
        // 注意：这里不能修改self，因为方法是不可变的
        // 在实际使用中，统计更新应该在调用方进行
        
        Ok(encoded)
    }
    
    async fn decode_message(&mut self, data: &[u8]) -> Result<Option<ProtocolMessage>> {
        if data.is_empty() {
            return Ok(None);
        }
        
        let start_time = Instant::now();
        
        let mut message_data = data.to_vec();
        
        // 如果启用校验，验证校验和
        if self.config.enable_checksum && !message_data.is_empty() {
            let received_checksum = message_data.pop().unwrap();
            let calculated_checksum = message_data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
            
            if received_checksum != calculated_checksum {
                self.stats.update_error_stats();
                return Err(crate::error::ProtocolError::ChecksumMismatch.into());
            }
        }
        
        // 创建消息
        let mut message = ProtocolMessage::new(MessageType::Data, message_data);
        message.id = Some(self.next_message_id());
        
        if self.config.enable_timestamp {
            message.timestamp = Some(Instant::now());
        }
        
        // 添加元数据
        message.metadata.insert("protocol".to_string(), "raw".to_string());
        message.metadata.insert("decoded_length".to_string(), message.data.len().to_string());
        
        // 更新统计
        let processing_time = start_time.elapsed().as_micros() as u64;
        self.stats.update_message_stats(false, true);
        self.stats.update_bytes_stats(data.len() as u64);
        self.stats.update_processing_time(processing_time);
        self.update_activity();
        
        Ok(Some(message))
    }
    
    async fn reset(&mut self) -> Result<()> {
        self.state = ProtocolState::Idle;
        self.rx_buffer.clear();
        self.last_activity = None;
        self.message_id_counter = 0;
        // 不重置统计信息，保留历史数据
        Ok(())
    }
    
    fn config(&self) -> &dyn ProtocolConfig {
        &self.config
    }
    
    async fn set_config(&mut self, config: Box<dyn ProtocolConfig>) -> Result<()> {
        // 尝试转换为RawProtocolConfig
        if let Ok(raw_config) = config.clone_box().downcast::<RawProtocolConfig>() {
            self.config = *raw_config;
            Ok(())
        } else {
            Err(crate::error::ProtocolError::InvalidConfiguration.into())
        }
    }
    
    async fn handle_timeout(&mut self) -> Result<()> {
        if self.is_timeout() {
            self.stats.update_timeout_stats();
            self.state = ProtocolState::Error;
            self.rx_buffer.clear();
            return Err(crate::error::ProtocolError::Timeout.into());
        }
        
        Ok(())
    }
    
    fn get_stats(&self) -> ProtocolStats {
        self.stats.clone()
    }
}

/// 原始协议构建器
pub struct RawProtocolBuilder {
    config: RawProtocolConfig,
}

impl RawProtocolBuilder {
    /// 创建新的构建器
    pub fn new() -> Self {
        Self {
            config: RawProtocolConfig::default(),
        }
    }
    
    /// 设置超时时间
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.config.timeout = timeout;
        self
    }
    
    /// 设置最大消息长度
    pub fn max_message_length(mut self, length: usize) -> Self {
        self.config.max_message_length = length;
        self
    }
    
    /// 设置缓冲区大小
    pub fn buffer_size(mut self, size: usize) -> Self {
        self.config.buffer_size = size;
        self
    }
    
    /// 启用校验
    pub fn enable_checksum(mut self, enable: bool) -> Self {
        self.config.enable_checksum = enable;
        self
    }
    
    /// 启用时间戳
    pub fn enable_timestamp(mut self, enable: bool) -> Self {
        self.config.enable_timestamp = enable;
        self
    }
    
    /// 设置批处理大小
    pub fn batch_size(mut self, size: usize) -> Self {
        self.config.batch_size = size;
        self
    }
    
    /// 构建处理器
    pub fn build(self) -> RawProtocolHandler {
        RawProtocolHandler::with_config(self.config)
    }
}

impl Default for RawProtocolBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio_test;
    
    #[tokio::test]
    async fn test_raw_protocol_handler_creation() {
        let handler = RawProtocolHandler::new();
        assert_eq!(handler.protocol_type(), ProtocolType::Raw);
        assert_eq!(handler.state(), ProtocolState::Idle);
    }
    
    #[tokio::test]
    async fn test_raw_protocol_handle_data() {
        let mut handler = RawProtocolHandler::new();
        let test_data = b"Hello, World!";
        
        let messages = handler.handle_received_data(test_data).await.unwrap();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].data, test_data);
        assert_eq!(messages[0].message_type, MessageType::Data);
        assert!(messages[0].id.is_some());
    }
    
    #[tokio::test]
    async fn test_raw_protocol_encode_message() {
        let handler = RawProtocolHandler::new();
        let test_data = b"Test message".to_vec();
        let message = ProtocolMessage::new(MessageType::Data, test_data.clone());
        
        let encoded = handler.encode_message(&message).await.unwrap();
        assert_eq!(encoded, test_data);
    }
    
    #[tokio::test]
    async fn test_raw_protocol_decode_message() {
        let mut handler = RawProtocolHandler::new();
        let test_data = b"Test message";
        
        let message = handler.decode_message(test_data).await.unwrap();
        assert!(message.is_some());
        
        let message = message.unwrap();
        assert_eq!(message.data, test_data);
        assert_eq!(message.message_type, MessageType::Data);
    }
    
    #[tokio::test]
    async fn test_raw_protocol_with_checksum() {
        let config = RawProtocolConfig {
            enable_checksum: true,
            ..Default::default()
        };
        let mut handler = RawProtocolHandler::with_config(config);
        
        let test_data = b"Test".to_vec();
        let message = ProtocolMessage::new(MessageType::Data, test_data.clone());
        
        // 编码消息（会添加校验和）
        let encoded = handler.encode_message(&message).await.unwrap();
        assert_eq!(encoded.len(), test_data.len() + 1); // +1 for checksum
        
        // 解码消息（会验证校验和）
        let decoded = handler.decode_message(&encoded).await.unwrap();
        assert!(decoded.is_some());
        assert_eq!(decoded.unwrap().data, test_data);
    }
    
    #[tokio::test]
    async fn test_raw_protocol_checksum_error() {
        let config = RawProtocolConfig {
            enable_checksum: true,
            ..Default::default()
        };
        let mut handler = RawProtocolHandler::with_config(config);
        
        let mut test_data = b"Test".to_vec();
        test_data.push(0xFF); // 错误的校验和
        
        let result = handler.decode_message(&test_data).await;
        assert!(result.is_err());
    }
    
    #[tokio::test]
    async fn test_raw_protocol_batch_processing() {
        let config = RawProtocolConfig {
            batch_size: 4,
            ..Default::default()
        };
        let mut handler = RawProtocolHandler::with_config(config);
        
        let test_data = b"Hello, World!"; // 13 bytes, will be split into batches
        
        let messages = handler.handle_received_data(test_data).await.unwrap();
        assert!(messages.len() > 1); // Should be split into multiple messages
        
        // 重新组合数据验证
        let mut combined_data = Vec::new();
        for message in messages {
            combined_data.extend_from_slice(&message.data);
        }
        assert_eq!(combined_data, test_data);
    }
    
    #[tokio::test]
    async fn test_raw_protocol_buffer_overflow() {
        let config = RawProtocolConfig {
            buffer_size: 10,
            ..Default::default()
        };
        let mut handler = RawProtocolHandler::with_config(config);
        
        let large_data = vec![0u8; 20]; // Larger than buffer size
        
        let result = handler.handle_received_data(&large_data).await;
        assert!(result.is_err());
        assert_eq!(handler.state(), ProtocolState::Error);
    }
    
    #[tokio::test]
    async fn test_raw_protocol_reset() {
        let mut handler = RawProtocolHandler::new();
        
        // 添加一些数据
        handler.rx_buffer.extend_from_slice(b"test");
        handler.state = ProtocolState::ProcessingData;
        handler.message_id_counter = 10;
        
        handler.reset().await.unwrap();
        
        assert_eq!(handler.state(), ProtocolState::Idle);
        assert!(handler.rx_buffer.is_empty());
        assert_eq!(handler.message_id_counter, 0);
    }
    
    #[tokio::test]
    async fn test_raw_protocol_builder() {
        let handler = RawProtocolBuilder::new()
            .timeout(Duration::from_secs(10))
            .max_message_length(2048)
            .buffer_size(4096)
            .enable_checksum(true)
            .enable_timestamp(false)
            .batch_size(8)
            .build();
        
        assert_eq!(handler.config.timeout, Duration::from_secs(10));
        assert_eq!(handler.config.max_message_length, 2048);
        assert_eq!(handler.config.buffer_size, 4096);
        assert!(handler.config.enable_checksum);
        assert!(!handler.config.enable_timestamp);
        assert_eq!(handler.config.batch_size, 8);
    }
    
    #[tokio::test]
    async fn test_raw_protocol_stats() {
        let mut handler = RawProtocolHandler::new();
        let test_data = b"Test data";
        
        // 处理一些数据
        handler.handle_received_data(test_data).await.unwrap();
        
        let stats = handler.get_stats();
        assert!(stats.messages_processed > 0);
        assert!(stats.bytes_processed > 0);
        assert!(stats.last_activity.is_some());
    }
}