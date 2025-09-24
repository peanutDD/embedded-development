//! # 协议处理模块
//!
//! 提供各种通信协议的实现和状态机管理。

use crate::{
    error::{ProtocolError, Result, UartError},
    traits::{AsyncRead, AsyncWrite},
};
use core::{
    fmt,
    future::Future,
    pin::Pin,
    task::{Context, Poll},
    time::Duration,
};
use std::collections::HashMap;

/// 协议类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProtocolType {
    /// 原始数据传输
    Raw,
    /// 基于行的协议
    LineBasedProtocol,
    /// 基于长度的协议
    LengthPrefixedProtocol,
    /// 基于分隔符的协议
    DelimiterBasedProtocol,
    /// 基于帧的协议
    FrameBasedProtocol,
    /// AT命令协议
    AtCommandProtocol,
    /// Modbus协议
    ModbusProtocol,
    /// 自定义协议
    CustomProtocol(u32),
}

impl fmt::Display for ProtocolType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ProtocolType::Raw => write!(f, "Raw"),
            ProtocolType::LineBasedProtocol => write!(f, "Line-Based"),
            ProtocolType::LengthPrefixedProtocol => write!(f, "Length-Prefixed"),
            ProtocolType::DelimiterBasedProtocol => write!(f, "Delimiter-Based"),
            ProtocolType::FrameBasedProtocol => write!(f, "Frame-Based"),
            ProtocolType::AtCommandProtocol => write!(f, "AT Command"),
            ProtocolType::ModbusProtocol => write!(f, "Modbus"),
            ProtocolType::CustomProtocol(id) => write!(f, "Custom({})", id),
        }
    }
}

/// 协议状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolState {
    /// 空闲状态
    Idle,
    /// 等待数据
    WaitingForData,
    /// 处理数据
    ProcessingData,
    /// 等待响应
    WaitingForResponse,
    /// 发送响应
    SendingResponse,
    /// 错误状态
    Error,
    /// 完成状态
    Complete,
}

/// 协议消息
#[derive(Debug, Clone)]
pub struct ProtocolMessage {
    /// 消息类型
    pub message_type: MessageType,
    /// 消息数据
    pub data: Vec<u8>,
    /// 消息ID
    pub id: Option<u32>,
    /// 时间戳
    pub timestamp: Option<std::time::Instant>,
    /// 元数据
    pub metadata: HashMap<String, String>,
}

/// 消息类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    /// 命令
    Command,
    /// 响应
    Response,
    /// 通知
    Notification,
    /// 错误
    Error,
    /// 数据
    Data,
    /// 心跳
    Heartbeat,
}

impl ProtocolMessage {
    /// 创建新消息
    pub fn new(message_type: MessageType, data: Vec<u8>) -> Self {
        Self {
            message_type,
            data,
            id: None,
            timestamp: Some(std::time::Instant::now()),
            metadata: HashMap::new(),
        }
    }
    
    /// 设置消息ID
    pub fn with_id(mut self, id: u32) -> Self {
        self.id = Some(id);
        self
    }
    
    /// 添加元数据
    pub fn with_metadata(mut self, key: String, value: String) -> Self {
        self.metadata.insert(key, value);
        self
    }
    
    /// 获取数据长度
    pub fn len(&self) -> usize {
        self.data.len()
    }
    
    /// 检查是否为空
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
    
    /// 获取数据引用
    pub fn data(&self) -> &[u8] {
        &self.data
    }
    
    /// 获取可变数据引用
    pub fn data_mut(&mut self) -> &mut Vec<u8> {
        &mut self.data
    }
}

/// 协议处理器特征
pub trait ProtocolHandler: Send + Sync {
    /// 获取协议类型
    fn protocol_type(&self) -> ProtocolType;
    
    /// 获取当前状态
    fn state(&self) -> ProtocolState;
    
    /// 处理接收到的数据
    fn handle_received_data(&mut self, data: &[u8]) -> impl Future<Output = Result<Vec<ProtocolMessage>>> + Send;
    
    /// 编码消息为字节
    fn encode_message(&self, message: &ProtocolMessage) -> impl Future<Output = Result<Vec<u8>>> + Send;
    
    /// 解码字节为消息
    fn decode_message(&mut self, data: &[u8]) -> impl Future<Output = Result<Option<ProtocolMessage>>> + Send;
    
    /// 重置协议状态
    fn reset(&mut self) -> impl Future<Output = Result<()>> + Send;
    
    /// 获取协议配置
    fn config(&self) -> &dyn ProtocolConfig;
    
    /// 设置协议配置
    fn set_config(&mut self, config: Box<dyn ProtocolConfig>) -> impl Future<Output = Result<()>> + Send;
    
    /// 处理超时
    fn handle_timeout(&mut self) -> impl Future<Output = Result<()>> + Send;
    
    /// 获取统计信息
    fn get_stats(&self) -> ProtocolStats;
}

/// 协议配置特征
pub trait ProtocolConfig: Send + Sync {
    /// 获取超时时间
    fn timeout(&self) -> Duration;
    
    /// 获取最大消息长度
    fn max_message_length(&self) -> usize;
    
    /// 获取缓冲区大小
    fn buffer_size(&self) -> usize;
    
    /// 是否启用校验
    fn enable_checksum(&self) -> bool;
    
    /// 克隆配置
    fn clone_box(&self) -> Box<dyn ProtocolConfig>;
}

/// 协议统计信息
#[derive(Debug, Clone, Default)]
pub struct ProtocolStats {
    /// 处理的消息数
    pub messages_processed: u64,
    /// 发送的消息数
    pub messages_sent: u64,
    /// 接收的消息数
    pub messages_received: u64,
    /// 错误数
    pub errors: u64,
    /// 超时数
    pub timeouts: u64,
    /// 处理的字节数
    pub bytes_processed: u64,
    /// 平均处理时间（微秒）
    pub avg_processing_time_us: u64,
    /// 最后活动时间
    pub last_activity: Option<std::time::Instant>,
}

impl ProtocolStats {
    /// 更新消息统计
    pub fn update_message_stats(&mut self, sent: bool, received: bool) {
        self.messages_processed += 1;
        if sent {
            self.messages_sent += 1;
        }
        if received {
            self.messages_received += 1;
        }
        self.last_activity = Some(std::time::Instant::now());
    }
    
    /// 更新错误统计
    pub fn update_error_stats(&mut self) {
        self.errors += 1;
        self.last_activity = Some(std::time::Instant::now());
    }
    
    /// 更新超时统计
    pub fn update_timeout_stats(&mut self) {
        self.timeouts += 1;
        self.last_activity = Some(std::time::Instant::now());
    }
    
    /// 更新字节统计
    pub fn update_bytes_stats(&mut self, bytes: u64) {
        self.bytes_processed += bytes;
        self.last_activity = Some(std::time::Instant::now());
    }
    
    /// 更新处理时间统计
    pub fn update_processing_time(&mut self, time_us: u64) {
        // 简单的移动平均
        if self.avg_processing_time_us == 0 {
            self.avg_processing_time_us = time_us;
        } else {
            self.avg_processing_time_us = (self.avg_processing_time_us * 7 + time_us) / 8;
        }
    }
    
    /// 重置统计
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// 协议管理器
pub struct ProtocolManager {
    /// 注册的协议处理器
    handlers: HashMap<ProtocolType, Box<dyn ProtocolHandler>>,
    /// 当前活动的协议
    active_protocol: Option<ProtocolType>,
    /// 默认协议
    default_protocol: ProtocolType,
    /// 全局统计
    global_stats: ProtocolStats,
}

impl ProtocolManager {
    /// 创建新的协议管理器
    pub fn new() -> Self {
        Self {
            handlers: HashMap::new(),
            active_protocol: None,
            default_protocol: ProtocolType::Raw,
            global_stats: ProtocolStats::default(),
        }
    }
    
    /// 注册协议处理器
    pub fn register_handler(&mut self, handler: Box<dyn ProtocolHandler>) -> Result<()> {
        let protocol_type = handler.protocol_type();
        self.handlers.insert(protocol_type, handler);
        Ok(())
    }
    
    /// 注销协议处理器
    pub fn unregister_handler(&mut self, protocol_type: ProtocolType) -> Result<()> {
        self.handlers.remove(&protocol_type);
        if self.active_protocol == Some(protocol_type) {
            self.active_protocol = None;
        }
        Ok(())
    }
    
    /// 设置活动协议
    pub fn set_active_protocol(&mut self, protocol_type: ProtocolType) -> Result<()> {
        if !self.handlers.contains_key(&protocol_type) {
            return Err(ProtocolError::UnsupportedProtocol.into());
        }
        
        self.active_protocol = Some(protocol_type);
        Ok(())
    }
    
    /// 获取活动协议
    pub fn get_active_protocol(&self) -> Option<ProtocolType> {
        self.active_protocol
    }
    
    /// 设置默认协议
    pub fn set_default_protocol(&mut self, protocol_type: ProtocolType) {
        self.default_protocol = protocol_type;
    }
    
    /// 处理接收到的数据
    pub async fn handle_received_data(&mut self, data: &[u8]) -> Result<Vec<ProtocolMessage>> {
        let protocol_type = self.active_protocol.unwrap_or(self.default_protocol);
        
        if let Some(handler) = self.handlers.get_mut(&protocol_type) {
            let start_time = std::time::Instant::now();
            let result = handler.handle_received_data(data).await;
            let processing_time = start_time.elapsed().as_micros() as u64;
            
            match result {
                Ok(messages) => {
                    self.global_stats.update_message_stats(false, true);
                    self.global_stats.update_bytes_stats(data.len() as u64);
                    self.global_stats.update_processing_time(processing_time);
                    Ok(messages)
                }
                Err(e) => {
                    self.global_stats.update_error_stats();
                    Err(e)
                }
            }
        } else {
            Err(ProtocolError::UnsupportedProtocol.into())
        }
    }
    
    /// 编码消息
    pub async fn encode_message(&self, message: &ProtocolMessage) -> Result<Vec<u8>> {
        let protocol_type = self.active_protocol.unwrap_or(self.default_protocol);
        
        if let Some(handler) = self.handlers.get(&protocol_type) {
            handler.encode_message(message).await
        } else {
            Err(ProtocolError::UnsupportedProtocol.into())
        }
    }
    
    /// 解码消息
    pub async fn decode_message(&mut self, data: &[u8]) -> Result<Option<ProtocolMessage>> {
        let protocol_type = self.active_protocol.unwrap_or(self.default_protocol);
        
        if let Some(handler) = self.handlers.get_mut(&protocol_type) {
            handler.decode_message(data).await
        } else {
            Err(ProtocolError::UnsupportedProtocol.into())
        }
    }
    
    /// 重置所有协议
    pub async fn reset_all(&mut self) -> Result<()> {
        for handler in self.handlers.values_mut() {
            handler.reset().await?;
        }
        self.global_stats.reset();
        Ok(())
    }
    
    /// 重置特定协议
    pub async fn reset_protocol(&mut self, protocol_type: ProtocolType) -> Result<()> {
        if let Some(handler) = self.handlers.get_mut(&protocol_type) {
            handler.reset().await
        } else {
            Err(ProtocolError::UnsupportedProtocol.into())
        }
    }
    
    /// 获取协议状态
    pub fn get_protocol_state(&self, protocol_type: ProtocolType) -> Option<ProtocolState> {
        self.handlers.get(&protocol_type).map(|h| h.state())
    }
    
    /// 获取协议统计
    pub fn get_protocol_stats(&self, protocol_type: ProtocolType) -> Option<ProtocolStats> {
        self.handlers.get(&protocol_type).map(|h| h.get_stats())
    }
    
    /// 获取全局统计
    pub fn get_global_stats(&self) -> &ProtocolStats {
        &self.global_stats
    }
    
    /// 获取所有注册的协议类型
    pub fn get_registered_protocols(&self) -> Vec<ProtocolType> {
        self.handlers.keys().copied().collect()
    }
    
    /// 处理超时
    pub async fn handle_timeout(&mut self, protocol_type: Option<ProtocolType>) -> Result<()> {
        let target_protocol = protocol_type.unwrap_or_else(|| self.active_protocol.unwrap_or(self.default_protocol));
        
        if let Some(handler) = self.handlers.get_mut(&target_protocol) {
            let result = handler.handle_timeout().await;
            if result.is_err() {
                self.global_stats.update_timeout_stats();
            }
            result
        } else {
            Err(ProtocolError::UnsupportedProtocol.into())
        }
    }
}

impl Default for ProtocolManager {
    fn default() -> Self {
        Self::new()
    }
}

/// 协议适配器
pub struct ProtocolAdapter<T: AsyncRead + AsyncWrite> {
    /// 底层传输
    transport: T,
    /// 协议管理器
    manager: ProtocolManager,
    /// 接收缓冲区
    rx_buffer: Vec<u8>,
    /// 发送缓冲区
    tx_buffer: Vec<u8>,
    /// 最大缓冲区大小
    max_buffer_size: usize,
}

impl<T: AsyncRead + AsyncWrite> ProtocolAdapter<T> {
    /// 创建新的协议适配器
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            manager: ProtocolManager::new(),
            rx_buffer: Vec::new(),
            tx_buffer: Vec::new(),
            max_buffer_size: 4096,
        }
    }
    
    /// 设置最大缓冲区大小
    pub fn set_max_buffer_size(&mut self, size: usize) {
        self.max_buffer_size = size;
    }
    
    /// 获取协议管理器引用
    pub fn manager(&self) -> &ProtocolManager {
        &self.manager
    }
    
    /// 获取可变协议管理器引用
    pub fn manager_mut(&mut self) -> &mut ProtocolManager {
        &mut self.manager
    }
    
    /// 发送消息
    pub async fn send_message(&mut self, message: ProtocolMessage) -> Result<()> {
        let encoded = self.manager.encode_message(&message).await?;
        
        // 检查缓冲区空间
        if self.tx_buffer.len() + encoded.len() > self.max_buffer_size {
            return Err(ProtocolError::BufferOverflow.into());
        }
        
        self.tx_buffer.extend_from_slice(&encoded);
        self.flush_tx_buffer().await?;
        
        Ok(())
    }
    
    /// 接收消息
    pub async fn receive_message(&mut self) -> Result<Option<ProtocolMessage>> {
        // 尝试从现有缓冲区解码消息
        if let Some(message) = self.manager.decode_message(&self.rx_buffer).await? {
            return Ok(Some(message));
        }
        
        // 读取更多数据
        let mut temp_buffer = vec![0u8; 1024];
        match self.transport.read(&mut temp_buffer).await {
            Ok(0) => Ok(None), // EOF
            Ok(n) => {
                if self.rx_buffer.len() + n > self.max_buffer_size {
                    return Err(ProtocolError::BufferOverflow.into());
                }
                
                self.rx_buffer.extend_from_slice(&temp_buffer[..n]);
                self.manager.decode_message(&self.rx_buffer).await
            }
            Err(e) => Err(e),
        }
    }
    
    /// 刷新发送缓冲区
    async fn flush_tx_buffer(&mut self) -> Result<()> {
        while !self.tx_buffer.is_empty() {
            let written = self.transport.write(&self.tx_buffer).await?;
            self.tx_buffer.drain(..written);
        }
        
        self.transport.flush().await?;
        Ok(())
    }
    
    /// 清空接收缓冲区
    pub fn clear_rx_buffer(&mut self) {
        self.rx_buffer.clear();
    }
    
    /// 清空发送缓冲区
    pub fn clear_tx_buffer(&mut self) {
        self.tx_buffer.clear();
    }
    
    /// 获取缓冲区使用情况
    pub fn buffer_usage(&self) -> (usize, usize, usize) {
        (self.rx_buffer.len(), self.tx_buffer.len(), self.max_buffer_size)
    }
}

// 声明子模块
pub mod line_based;
pub mod length_prefixed;
pub mod delimiter_based;
pub mod frame_based;
pub mod at_command;
pub mod modbus;
pub mod raw;

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_protocol_type_display() {
        assert_eq!(ProtocolType::Raw.to_string(), "Raw");
        assert_eq!(ProtocolType::LineBasedProtocol.to_string(), "Line-Based");
        assert_eq!(ProtocolType::CustomProtocol(42).to_string(), "Custom(42)");
    }
    
    #[test]
    fn test_protocol_message_creation() {
        let data = b"Hello, World!".to_vec();
        let message = ProtocolMessage::new(MessageType::Command, data.clone());
        
        assert_eq!(message.message_type, MessageType::Command);
        assert_eq!(message.data, data);
        assert!(message.id.is_none());
        assert!(message.timestamp.is_some());
        assert!(message.metadata.is_empty());
    }
    
    #[test]
    fn test_protocol_message_with_id() {
        let data = b"Test".to_vec();
        let message = ProtocolMessage::new(MessageType::Response, data)
            .with_id(123);
        
        assert_eq!(message.id, Some(123));
    }
    
    #[test]
    fn test_protocol_message_with_metadata() {
        let data = b"Test".to_vec();
        let message = ProtocolMessage::new(MessageType::Data, data)
            .with_metadata("key".to_string(), "value".to_string());
        
        assert_eq!(message.metadata.get("key"), Some(&"value".to_string()));
    }
    
    #[test]
    fn test_protocol_stats() {
        let mut stats = ProtocolStats::default();
        
        stats.update_message_stats(true, false);
        assert_eq!(stats.messages_sent, 1);
        assert_eq!(stats.messages_received, 0);
        assert_eq!(stats.messages_processed, 1);
        
        stats.update_error_stats();
        assert_eq!(stats.errors, 1);
        
        stats.update_timeout_stats();
        assert_eq!(stats.timeouts, 1);
        
        stats.update_bytes_stats(100);
        assert_eq!(stats.bytes_processed, 100);
        
        stats.reset();
        assert_eq!(stats.messages_processed, 0);
        assert_eq!(stats.errors, 0);
    }
    
    #[test]
    fn test_protocol_manager_creation() {
        let manager = ProtocolManager::new();
        assert!(manager.get_active_protocol().is_none());
        assert_eq!(manager.default_protocol, ProtocolType::Raw);
        assert!(manager.get_registered_protocols().is_empty());
    }
}