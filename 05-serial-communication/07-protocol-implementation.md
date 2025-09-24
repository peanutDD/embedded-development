# 协议实现

## 概述

在嵌入式系统中，串口通信协议的实现是构建可靠通信系统的关键。本章将详细介绍如何在Rust中实现各种常见的串口通信协议，包括自定义协议、标准工业协议以及协议栈的设计模式。

## 协议设计原则

### 1. 协议分层架构

```
┌─────────────────┐
│   应用层协议     │  ← 业务逻辑、命令处理
├─────────────────┤
│   传输层协议     │  ← 数据完整性、流控制
├─────────────────┤
│   数据链路层     │  ← 帧格式、错误检测
├─────────────────┤
│   物理层        │  ← UART硬件接口
└─────────────────┘
```

### 2. 协议设计要素

- **帧格式定义**：起始标志、数据长度、数据内容、校验码、结束标志
- **错误检测**：CRC、校验和、奇偶校验
- **流控制**：ACK/NAK、滑动窗口、超时重传
- **地址管理**：设备地址、广播地址、组播地址
- **命令系统**：命令编码、参数格式、响应格式

## 基础协议框架

### 1. 协议特征定义

```rust
// src/protocol/traits.rs
use heapless::Vec;

pub trait Protocol {
    type Error;
    type Frame;
    type Address;
    
    /// 编码数据帧
    fn encode(&self, data: &[u8], address: Self::Address) -> Result<Vec<u8, 256>, Self::Error>;
    
    /// 解码数据帧
    fn decode(&self, buffer: &[u8]) -> Result<Option<Self::Frame>, Self::Error>;
    
    /// 验证帧完整性
    fn validate_frame(&self, frame: &Self::Frame) -> bool;
    
    /// 获取协议名称
    fn name(&self) -> &'static str;
}

pub trait FrameParser {
    type Frame;
    type Error;
    
    /// 解析单个帧
    fn parse_frame(&mut self, byte: u8) -> Result<Option<Self::Frame>, Self::Error>;
    
    /// 重置解析器状态
    fn reset(&mut self);
    
    /// 获取当前解析状态
    fn state(&self) -> ParserState;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParserState {
    WaitingStart,
    ReadingHeader,
    ReadingData,
    ReadingChecksum,
    FrameComplete,
    Error,
}

pub trait Checksum {
    /// 计算校验值
    fn calculate(&self, data: &[u8]) -> u16;
    
    /// 验证校验值
    fn verify(&self, data: &[u8], checksum: u16) -> bool {
        self.calculate(data) == checksum
    }
}
```

### 2. 基础数据结构

```rust
// src/protocol/frame.rs
use heapless::Vec;

#[derive(Debug, Clone)]
pub struct Frame {
    pub header: FrameHeader,
    pub payload: Vec<u8, 128>,
    pub checksum: u16,
}

#[derive(Debug, Clone)]
pub struct FrameHeader {
    pub start_marker: u8,
    pub frame_type: FrameType,
    pub source_addr: u8,
    pub dest_addr: u8,
    pub sequence: u8,
    pub length: u8,
    pub flags: FrameFlags,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FrameType {
    Data = 0x01,
    Command = 0x02,
    Response = 0x03,
    Ack = 0x04,
    Nak = 0x05,
    Heartbeat = 0x06,
}

#[derive(Debug, Clone, Copy)]
pub struct FrameFlags {
    pub require_ack: bool,
    pub is_broadcast: bool,
    pub is_encrypted: bool,
    pub priority: Priority,
}

#[derive(Debug, Clone, Copy)]
pub enum Priority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
}

impl Frame {
    pub fn new(frame_type: FrameType, source: u8, dest: u8) -> Self {
        Self {
            header: FrameHeader {
                start_marker: 0xAA,
                frame_type,
                source_addr: source,
                dest_addr: dest,
                sequence: 0,
                length: 0,
                flags: FrameFlags {
                    require_ack: false,
                    is_broadcast: false,
                    is_encrypted: false,
                    priority: Priority::Normal,
                },
            },
            payload: Vec::new(),
            checksum: 0,
        }
    }
    
    pub fn with_payload(mut self, data: &[u8]) -> Self {
        self.payload.extend_from_slice(data).ok();
        self.header.length = self.payload.len() as u8;
        self
    }
    
    pub fn with_sequence(mut self, seq: u8) -> Self {
        self.header.sequence = seq;
        self
    }
    
    pub fn require_ack(mut self) -> Self {
        self.header.flags.require_ack = true;
        self
    }
    
    pub fn as_broadcast(mut self) -> Self {
        self.header.flags.is_broadcast = true;
        self.header.dest_addr = 0xFF; // 广播地址
        self
    }
    
    pub fn with_priority(mut self, priority: Priority) -> Self {
        self.header.flags.priority = priority;
        self
    }
}

impl FrameFlags {
    pub fn to_byte(&self) -> u8 {
        let mut flags = 0u8;
        if self.require_ack { flags |= 0x01; }
        if self.is_broadcast { flags |= 0x02; }
        if self.is_encrypted { flags |= 0x04; }
        flags |= (self.priority as u8) << 3;
        flags
    }
    
    pub fn from_byte(byte: u8) -> Self {
        Self {
            require_ack: (byte & 0x01) != 0,
            is_broadcast: (byte & 0x02) != 0,
            is_encrypted: (byte & 0x04) != 0,
            priority: match (byte >> 3) & 0x03 {
                0 => Priority::Low,
                1 => Priority::Normal,
                2 => Priority::High,
                3 => Priority::Critical,
                _ => Priority::Normal,
            },
        }
    }
}
```

### 3. 校验算法实现

```rust
// src/protocol/checksum.rs
use crate::protocol::traits::Checksum;

pub struct Crc16;
pub struct SimpleChecksum;
pub struct XorChecksum;

impl Checksum for Crc16 {
    fn calculate(&self, data: &[u8]) -> u16 {
        let mut crc = 0xFFFFu16;
        const POLYNOMIAL: u16 = 0x1021; // CRC-16-CCITT
        
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ POLYNOMIAL;
                } else {
                    crc <<= 1;
                }
            }
        }
        
        crc
    }
}

impl Checksum for SimpleChecksum {
    fn calculate(&self, data: &[u8]) -> u16 {
        data.iter().map(|&b| b as u16).sum()
    }
}

impl Checksum for XorChecksum {
    fn calculate(&self, data: &[u8]) -> u16 {
        data.iter().fold(0u8, |acc, &b| acc ^ b) as u16
    }
}

// CRC查表法实现（更高效）
pub struct Crc16Table {
    table: [u16; 256],
}

impl Crc16Table {
    pub fn new() -> Self {
        let mut table = [0u16; 256];
        const POLYNOMIAL: u16 = 0x1021;
        
        for i in 0..256 {
            let mut crc = (i as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ POLYNOMIAL;
                } else {
                    crc <<= 1;
                }
            }
            table[i] = crc;
        }
        
        Self { table }
    }
}

impl Checksum for Crc16Table {
    fn calculate(&self, data: &[u8]) -> u16 {
        let mut crc = 0xFFFFu16;
        
        for &byte in data {
            let table_index = ((crc >> 8) ^ (byte as u16)) as usize & 0xFF;
            crc = (crc << 8) ^ self.table[table_index];
        }
        
        crc
    }
}
```

## 自定义协议实现

### 1. 简单命令响应协议

```rust
// src/protocol/simple_protocol.rs
use crate::protocol::{
    traits::{Protocol, FrameParser, Checksum},
    frame::{Frame, FrameType, FrameHeader, FrameFlags},
    checksum::Crc16,
};
use heapless::Vec;

pub struct SimpleProtocol {
    checksum: Crc16,
    device_address: u8,
}

#[derive(Debug)]
pub enum SimpleProtocolError {
    InvalidFrame,
    ChecksumMismatch,
    BufferOverflow,
    InvalidAddress,
}

impl SimpleProtocol {
    pub fn new(device_address: u8) -> Self {
        Self {
            checksum: Crc16,
            device_address,
        }
    }
    
    pub fn create_command(&self, cmd: u8, params: &[u8], dest: u8) -> Vec<u8, 256> {
        let mut payload = Vec::new();
        payload.push(cmd).ok();
        payload.extend_from_slice(params).ok();
        
        let frame = Frame::new(FrameType::Command, self.device_address, dest)
            .with_payload(&payload)
            .require_ack();
            
        self.encode_frame(&frame)
    }
    
    pub fn create_response(&self, cmd: u8, data: &[u8], dest: u8, success: bool) -> Vec<u8, 256> {
        let mut payload = Vec::new();
        payload.push(cmd).ok();
        payload.push(if success { 0x00 } else { 0xFF }).ok(); // 状态码
        payload.extend_from_slice(data).ok();
        
        let frame = Frame::new(FrameType::Response, self.device_address, dest)
            .with_payload(&payload);
            
        self.encode_frame(&frame)
    }
    
    pub fn create_ack(&self, sequence: u8, dest: u8) -> Vec<u8, 256> {
        let frame = Frame::new(FrameType::Ack, self.device_address, dest)
            .with_sequence(sequence);
            
        self.encode_frame(&frame)
    }
    
    pub fn create_nak(&self, sequence: u8, dest: u8, error_code: u8) -> Vec<u8, 256> {
        let mut payload = Vec::new();
        payload.push(error_code).ok();
        
        let frame = Frame::new(FrameType::Nak, self.device_address, dest)
            .with_sequence(sequence)
            .with_payload(&payload);
            
        self.encode_frame(&frame)
    }
    
    fn encode_frame(&self, frame: &Frame) -> Vec<u8, 256> {
        let mut buffer = Vec::new();
        
        // 帧头
        buffer.push(frame.header.start_marker).ok();
        buffer.push(frame.header.frame_type as u8).ok();
        buffer.push(frame.header.source_addr).ok();
        buffer.push(frame.header.dest_addr).ok();
        buffer.push(frame.header.sequence).ok();
        buffer.push(frame.header.length).ok();
        buffer.push(frame.header.flags.to_byte()).ok();
        
        // 载荷
        buffer.extend_from_slice(&frame.payload).ok();
        
        // 计算校验和（不包括校验和本身）
        let checksum = self.checksum.calculate(&buffer);
        buffer.push((checksum >> 8) as u8).ok();
        buffer.push((checksum & 0xFF) as u8).ok();
        
        buffer
    }
}

impl Protocol for SimpleProtocol {
    type Error = SimpleProtocolError;
    type Frame = Frame;
    type Address = u8;
    
    fn encode(&self, data: &[u8], address: Self::Address) -> Result<Vec<u8, 256>, Self::Error> {
        let frame = Frame::new(FrameType::Data, self.device_address, address)
            .with_payload(data);
        Ok(self.encode_frame(&frame))
    }
    
    fn decode(&self, buffer: &[u8]) -> Result<Option<Self::Frame>, Self::Error> {
        if buffer.len() < 9 { // 最小帧长度
            return Ok(None);
        }
        
        // 验证起始标志
        if buffer[0] != 0xAA {
            return Err(SimpleProtocolError::InvalidFrame);
        }
        
        let frame_type = match buffer[1] {
            0x01 => FrameType::Data,
            0x02 => FrameType::Command,
            0x03 => FrameType::Response,
            0x04 => FrameType::Ack,
            0x05 => FrameType::Nak,
            0x06 => FrameType::Heartbeat,
            _ => return Err(SimpleProtocolError::InvalidFrame),
        };
        
        let source_addr = buffer[2];
        let dest_addr = buffer[3];
        let sequence = buffer[4];
        let length = buffer[5] as usize;
        let flags = FrameFlags::from_byte(buffer[6]);
        
        // 检查帧长度
        if buffer.len() < 7 + length + 2 {
            return Ok(None); // 数据不完整
        }
        
        // 提取载荷
        let mut payload = Vec::new();
        if length > 0 {
            payload.extend_from_slice(&buffer[7..7+length])
                .map_err(|_| SimpleProtocolError::BufferOverflow)?;
        }
        
        // 验证校验和
        let received_checksum = ((buffer[7 + length] as u16) << 8) | (buffer[7 + length + 1] as u16);
        let calculated_checksum = self.checksum.calculate(&buffer[..7 + length]);
        
        if received_checksum != calculated_checksum {
            return Err(SimpleProtocolError::ChecksumMismatch);
        }
        
        let frame = Frame {
            header: FrameHeader {
                start_marker: buffer[0],
                frame_type,
                source_addr,
                dest_addr,
                sequence,
                length: length as u8,
                flags,
            },
            payload,
            checksum: received_checksum,
        };
        
        Ok(Some(frame))
    }
    
    fn validate_frame(&self, frame: &Self::Frame) -> bool {
        // 验证地址
        if !frame.header.flags.is_broadcast && 
           frame.header.dest_addr != self.device_address &&
           frame.header.dest_addr != 0xFF {
            return false;
        }
        
        // 验证载荷长度
        if frame.header.length != frame.payload.len() as u8 {
            return false;
        }
        
        true
    }
    
    fn name(&self) -> &'static str {
        "SimpleProtocol"
    }
}

// 流式解析器
pub struct SimpleProtocolParser {
    state: ParserState,
    buffer: Vec<u8, 256>,
    expected_length: usize,
    checksum: Crc16,
}

impl SimpleProtocolParser {
    pub fn new() -> Self {
        Self {
            state: ParserState::WaitingStart,
            buffer: Vec::new(),
            expected_length: 0,
            checksum: Crc16,
        }
    }
}

impl FrameParser for SimpleProtocolParser {
    type Frame = Frame;
    type Error = SimpleProtocolError;
    
    fn parse_frame(&mut self, byte: u8) -> Result<Option<Self::Frame>, Self::Error> {
        match self.state {
            ParserState::WaitingStart => {
                if byte == 0xAA {
                    self.buffer.clear();
                    self.buffer.push(byte).ok();
                    self.state = ParserState::ReadingHeader;
                }
            }
            
            ParserState::ReadingHeader => {
                self.buffer.push(byte).ok();
                
                if self.buffer.len() == 7 {
                    // 读取完头部，获取载荷长度
                    self.expected_length = self.buffer[5] as usize;
                    
                    if self.expected_length == 0 {
                        self.state = ParserState::ReadingChecksum;
                    } else {
                        self.state = ParserState::ReadingData;
                    }
                }
            }
            
            ParserState::ReadingData => {
                self.buffer.push(byte).ok();
                
                if self.buffer.len() == 7 + self.expected_length {
                    self.state = ParserState::ReadingChecksum;
                }
            }
            
            ParserState::ReadingChecksum => {
                self.buffer.push(byte).ok();
                
                if self.buffer.len() == 7 + self.expected_length + 2 {
                    // 帧接收完成，解析
                    let protocol = SimpleProtocol::new(0); // 临时协议实例
                    match protocol.decode(&self.buffer) {
                        Ok(Some(frame)) => {
                            self.state = ParserState::FrameComplete;
                            return Ok(Some(frame));
                        }
                        Ok(None) => {
                            self.state = ParserState::Error;
                            return Err(SimpleProtocolError::InvalidFrame);
                        }
                        Err(e) => {
                            self.state = ParserState::Error;
                            return Err(e);
                        }
                    }
                }
            }
            
            ParserState::FrameComplete | ParserState::Error => {
                // 重置状态，开始新的帧解析
                self.reset();
                if byte == 0xAA {
                    self.buffer.push(byte).ok();
                    self.state = ParserState::ReadingHeader;
                }
            }
        }
        
        Ok(None)
    }
    
    fn reset(&mut self) {
        self.state = ParserState::WaitingStart;
        self.buffer.clear();
        self.expected_length = 0;
    }
    
    fn state(&self) -> ParserState {
        self.state
    }
}
```

### 2. 可靠传输协议

```rust
// src/protocol/reliable_protocol.rs
use crate::protocol::{
    traits::{Protocol, Checksum},
    frame::{Frame, FrameType},
    checksum::Crc16Table,
};
use heapless::{Vec, FnvIndexMap};
use core::time::Duration;

pub struct ReliableProtocol {
    device_address: u8,
    checksum: Crc16Table,
    sequence_counter: u8,
    pending_acks: FnvIndexMap<u8, PendingFrame, 16>, // sequence -> frame
    receive_window: FnvIndexMap<u8, Frame, 16>,      // sequence -> frame
    window_size: u8,
    timeout_ms: u32,
}

#[derive(Debug, Clone)]
struct PendingFrame {
    frame: Frame,
    timestamp: u32,
    retry_count: u8,
    max_retries: u8,
}

#[derive(Debug)]
pub enum ReliableProtocolError {
    WindowFull,
    SequenceOutOfOrder,
    MaxRetriesExceeded,
    InvalidFrame,
    ChecksumMismatch,
    Timeout,
}

impl ReliableProtocol {
    pub fn new(device_address: u8) -> Self {
        Self {
            device_address,
            checksum: Crc16Table::new(),
            sequence_counter: 0,
            pending_acks: FnvIndexMap::new(),
            receive_window: FnvIndexMap::new(),
            window_size: 8,
            timeout_ms: 1000,
        }
    }
    
    pub fn send_reliable(&mut self, data: &[u8], dest: u8) -> Result<Vec<u8, 256>, ReliableProtocolError> {
        // 检查发送窗口
        if self.pending_acks.len() >= self.window_size as usize {
            return Err(ReliableProtocolError::WindowFull);
        }
        
        let sequence = self.next_sequence();
        let frame = Frame::new(FrameType::Data, self.device_address, dest)
            .with_payload(data)
            .with_sequence(sequence)
            .require_ack();
        
        // 编码帧
        let encoded = self.encode_frame(&frame);
        
        // 添加到待确认列表
        let pending = PendingFrame {
            frame: frame.clone(),
            timestamp: get_current_time(),
            retry_count: 0,
            max_retries: 3,
        };
        
        self.pending_acks.insert(sequence, pending).ok();
        
        Ok(encoded)
    }
    
    pub fn handle_ack(&mut self, sequence: u8) -> bool {
        self.pending_acks.remove(&sequence).is_some()
    }
    
    pub fn handle_nak(&mut self, sequence: u8) -> Option<Vec<u8, 256>> {
        if let Some(pending) = self.pending_acks.get_mut(&sequence) {
            pending.retry_count += 1;
            pending.timestamp = get_current_time();
            
            if pending.retry_count <= pending.max_retries {
                Some(self.encode_frame(&pending.frame))
            } else {
                self.pending_acks.remove(&sequence);
                None // 超过最大重试次数
            }
        } else {
            None
        }
    }
    
    pub fn process_received_frame(&mut self, frame: Frame) -> Result<Option<Frame>, ReliableProtocolError> {
        let sequence = frame.header.sequence;
        
        // 检查是否需要发送ACK
        if frame.header.flags.require_ack {
            // 发送ACK的逻辑应该由上层处理
        }
        
        // 检查序列号
        if self.receive_window.contains_key(&sequence) {
            // 重复帧，丢弃
            return Ok(None);
        }
        
        // 添加到接收窗口
        if self.receive_window.len() >= self.window_size as usize {
            // 窗口满，移除最旧的帧
            let oldest_seq = self.find_oldest_sequence();
            self.receive_window.remove(&oldest_seq);
        }
        
        self.receive_window.insert(sequence, frame.clone()).ok();
        
        Ok(Some(frame))
    }
    
    pub fn check_timeouts(&mut self) -> Vec<Vec<u8, 256>, 16> {
        let mut retransmissions = Vec::new();
        let current_time = get_current_time();
        let mut expired_sequences = Vec::<u8, 16>::new();
        
        for (&sequence, pending) in &mut self.pending_acks {
            if current_time - pending.timestamp > self.timeout_ms {
                if pending.retry_count < pending.max_retries {
                    pending.retry_count += 1;
                    pending.timestamp = current_time;
                    
                    let encoded = self.encode_frame(&pending.frame);
                    retransmissions.push(encoded).ok();
                } else {
                    expired_sequences.push(sequence).ok();
                }
            }
        }
        
        // 移除超时的帧
        for sequence in expired_sequences {
            self.pending_acks.remove(&sequence);
        }
        
        retransmissions
    }
    
    fn next_sequence(&mut self) -> u8 {
        self.sequence_counter = self.sequence_counter.wrapping_add(1);
        self.sequence_counter
    }
    
    fn find_oldest_sequence(&self) -> u8 {
        // 简化实现：返回第一个序列号
        *self.receive_window.keys().next().unwrap_or(&0)
    }
    
    fn encode_frame(&self, frame: &Frame) -> Vec<u8, 256> {
        // 使用SimpleProtocol的编码逻辑
        let mut buffer = Vec::new();
        
        buffer.push(frame.header.start_marker).ok();
        buffer.push(frame.header.frame_type as u8).ok();
        buffer.push(frame.header.source_addr).ok();
        buffer.push(frame.header.dest_addr).ok();
        buffer.push(frame.header.sequence).ok();
        buffer.push(frame.header.length).ok();
        buffer.push(frame.header.flags.to_byte()).ok();
        
        buffer.extend_from_slice(&frame.payload).ok();
        
        let checksum = self.checksum.calculate(&buffer);
        buffer.push((checksum >> 8) as u8).ok();
        buffer.push((checksum & 0xFF) as u8).ok();
        
        buffer
    }
    
    pub fn get_statistics(&self) -> ProtocolStatistics {
        ProtocolStatistics {
            pending_acks: self.pending_acks.len() as u8,
            receive_window_size: self.receive_window.len() as u8,
            current_sequence: self.sequence_counter,
            window_utilization: (self.pending_acks.len() as f32) / (self.window_size as f32),
        }
    }
}

#[derive(Debug)]
pub struct ProtocolStatistics {
    pub pending_acks: u8,
    pub receive_window_size: u8,
    pub current_sequence: u8,
    pub window_utilization: f32,
}

fn get_current_time() -> u32 {
    // 实际实现中应该返回系统时间戳
    0
}
```

## 工业协议实现

### 1. Modbus RTU协议

```rust
// src/protocol/modbus_rtu.rs
use crate::protocol::{
    traits::{Protocol, Checksum},
    checksum::Crc16,
};
use heapless::Vec;

pub struct ModbusRtu {
    device_address: u8,
    checksum: Crc16,
}

#[derive(Debug, Clone)]
pub struct ModbusFrame {
    pub address: u8,
    pub function_code: u8,
    pub data: Vec<u8, 252>,
    pub crc: u16,
}

#[derive(Debug, Clone, Copy)]
pub enum ModbusFunction {
    ReadCoils = 0x01,
    ReadDiscreteInputs = 0x02,
    ReadHoldingRegisters = 0x03,
    ReadInputRegisters = 0x04,
    WriteSingleCoil = 0x05,
    WriteSingleRegister = 0x06,
    WriteMultipleCoils = 0x0F,
    WriteMultipleRegisters = 0x10,
}

#[derive(Debug)]
pub enum ModbusError {
    InvalidFunction,
    InvalidAddress,
    InvalidData,
    CrcMismatch,
    FrameTooShort,
    FrameTooLong,
    IllegalDataAddress,
    IllegalDataValue,
    SlaveDeviceFailure,
}

impl ModbusRtu {
    pub fn new(device_address: u8) -> Self {
        Self {
            device_address,
            checksum: Crc16,
        }
    }
    
    pub fn read_holding_registers(&self, start_addr: u16, count: u16) -> Vec<u8, 256> {
        let mut frame = Vec::new();
        frame.push(self.device_address).ok();
        frame.push(ModbusFunction::ReadHoldingRegisters as u8).ok();
        frame.push((start_addr >> 8) as u8).ok();
        frame.push((start_addr & 0xFF) as u8).ok();
        frame.push((count >> 8) as u8).ok();
        frame.push((count & 0xFF) as u8).ok();
        
        let crc = self.calculate_crc(&frame);
        frame.push((crc & 0xFF) as u8).ok();
        frame.push((crc >> 8) as u8).ok();
        
        frame
    }
    
    pub fn write_single_register(&self, addr: u16, value: u16) -> Vec<u8, 256> {
        let mut frame = Vec::new();
        frame.push(self.device_address).ok();
        frame.push(ModbusFunction::WriteSingleRegister as u8).ok();
        frame.push((addr >> 8) as u8).ok();
        frame.push((addr & 0xFF) as u8).ok();
        frame.push((value >> 8) as u8).ok();
        frame.push((value & 0xFF) as u8).ok();
        
        let crc = self.calculate_crc(&frame);
        frame.push((crc & 0xFF) as u8).ok();
        frame.push((crc >> 8) as u8).ok();
        
        frame
    }
    
    pub fn write_multiple_registers(&self, start_addr: u16, values: &[u16]) -> Vec<u8, 256> {
        let mut frame = Vec::new();
        frame.push(self.device_address).ok();
        frame.push(ModbusFunction::WriteMultipleRegisters as u8).ok();
        frame.push((start_addr >> 8) as u8).ok();
        frame.push((start_addr & 0xFF) as u8).ok();
        frame.push((values.len() >> 8) as u8).ok();
        frame.push((values.len() & 0xFF) as u8).ok();
        frame.push((values.len() * 2) as u8).ok(); // 字节数
        
        for &value in values {
            frame.push((value >> 8) as u8).ok();
            frame.push((value & 0xFF) as u8).ok();
        }
        
        let crc = self.calculate_crc(&frame);
        frame.push((crc & 0xFF) as u8).ok();
        frame.push((crc >> 8) as u8).ok();
        
        frame
    }
    
    pub fn create_response(&self, function_code: u8, data: &[u8]) -> Vec<u8, 256> {
        let mut frame = Vec::new();
        frame.push(self.device_address).ok();
        frame.push(function_code).ok();
        frame.extend_from_slice(data).ok();
        
        let crc = self.calculate_crc(&frame);
        frame.push((crc & 0xFF) as u8).ok();
        frame.push((crc >> 8) as u8).ok();
        
        frame
    }
    
    pub fn create_error_response(&self, function_code: u8, error_code: u8) -> Vec<u8, 256> {
        let mut frame = Vec::new();
        frame.push(self.device_address).ok();
        frame.push(function_code | 0x80).ok(); // 错误响应标志
        frame.push(error_code).ok();
        
        let crc = self.calculate_crc(&frame);
        frame.push((crc & 0xFF) as u8).ok();
        frame.push((crc >> 8) as u8).ok();
        
        frame
    }
    
    fn calculate_crc(&self, data: &[u8]) -> u16 {
        let mut crc = 0xFFFFu16;
        
        for &byte in data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if crc & 0x0001 != 0 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        
        crc
    }
}

impl Protocol for ModbusRtu {
    type Error = ModbusError;
    type Frame = ModbusFrame;
    type Address = u8;
    
    fn encode(&self, data: &[u8], address: Self::Address) -> Result<Vec<u8, 256>, Self::Error> {
        if data.is_empty() {
            return Err(ModbusError::InvalidData);
        }
        
        let mut frame = Vec::new();
        frame.push(address).ok();
        frame.extend_from_slice(data).map_err(|_| ModbusError::FrameTooLong)?;
        
        let crc = self.calculate_crc(&frame);
        frame.push((crc & 0xFF) as u8).ok();
        frame.push((crc >> 8) as u8).ok();
        
        Ok(frame)
    }
    
    fn decode(&self, buffer: &[u8]) -> Result<Option<Self::Frame>, Self::Error> {
        if buffer.len() < 4 {
            return Ok(None); // 最小帧长度
        }
        
        let address = buffer[0];
        let function_code = buffer[1];
        
        // 提取数据部分（不包括地址、功能码和CRC）
        let data_len = buffer.len() - 4;
        let mut data = Vec::new();
        data.extend_from_slice(&buffer[2..2+data_len])
            .map_err(|_| ModbusError::FrameTooLong)?;
        
        // 验证CRC
        let received_crc = (buffer[buffer.len()-1] as u16) << 8 | (buffer[buffer.len()-2] as u16);
        let calculated_crc = self.calculate_crc(&buffer[..buffer.len()-2]);
        
        if received_crc != calculated_crc {
            return Err(ModbusError::CrcMismatch);
        }
        
        Ok(Some(ModbusFrame {
            address,
            function_code,
            data,
            crc: received_crc,
        }))
    }
    
    fn validate_frame(&self, frame: &Self::Frame) -> bool {
        // 验证地址
        if frame.address != self.device_address && frame.address != 0 {
            return false;
        }
        
        // 验证功能码
        match frame.function_code {
            0x01..=0x06 | 0x0F | 0x10 => true,
            0x81..=0x86 | 0x8F | 0x90 => true, // 错误响应
            _ => false,
        }
    }
    
    fn name(&self) -> &'static str {
        "ModbusRTU"
    }
}

// Modbus从站实现
pub struct ModbusSlave {
    protocol: ModbusRtu,
    holding_registers: Vec<u16, 100>,
    input_registers: Vec<u16, 100>,
    coils: Vec<bool, 100>,
    discrete_inputs: Vec<bool, 100>,
}

impl ModbusSlave {
    pub fn new(address: u8) -> Self {
        Self {
            protocol: ModbusRtu::new(address),
            holding_registers: Vec::new(),
            input_registers: Vec::new(),
            coils: Vec::new(),
            discrete_inputs: Vec::new(),
        }
    }
    
    pub fn process_request(&mut self, frame: &ModbusFrame) -> Result<Vec<u8, 256>, ModbusError> {
        match frame.function_code {
            0x03 => self.handle_read_holding_registers(frame),
            0x04 => self.handle_read_input_registers(frame),
            0x06 => self.handle_write_single_register(frame),
            0x10 => self.handle_write_multiple_registers(frame),
            _ => {
                // 不支持的功能码
                Ok(self.protocol.create_error_response(frame.function_code, 0x01))
            }
        }
    }
    
    fn handle_read_holding_registers(&self, frame: &ModbusFrame) -> Result<Vec<u8, 256>, ModbusError> {
        if frame.data.len() < 4 {
            return Err(ModbusError::InvalidData);
        }
        
        let start_addr = ((frame.data[0] as u16) << 8) | (frame.data[1] as u16);
        let count = ((frame.data[2] as u16) << 8) | (frame.data[3] as u16);
        
        if start_addr as usize + count as usize > self.holding_registers.len() {
            return Ok(self.protocol.create_error_response(frame.function_code, 0x02));
        }
        
        let mut response_data = Vec::new();
        response_data.push((count * 2) as u8).ok(); // 字节数
        
        for i in start_addr..start_addr + count {
            let value = self.holding_registers.get(i as usize).unwrap_or(&0);
            response_data.push((value >> 8) as u8).ok();
            response_data.push((value & 0xFF) as u8).ok();
        }
        
        Ok(self.protocol.create_response(frame.function_code, &response_data))
    }
    
    fn handle_read_input_registers(&self, frame: &ModbusFrame) -> Result<Vec<u8, 256>, ModbusError> {
        // 类似于读取保持寄存器的实现
        self.handle_read_holding_registers(frame)
    }
    
    fn handle_write_single_register(&mut self, frame: &ModbusFrame) -> Result<Vec<u8, 256>, ModbusError> {
        if frame.data.len() < 4 {
            return Err(ModbusError::InvalidData);
        }
        
        let addr = ((frame.data[0] as u16) << 8) | (frame.data[1] as u16);
        let value = ((frame.data[2] as u16) << 8) | (frame.data[3] as u16);
        
        if addr as usize >= self.holding_registers.len() {
            return Ok(self.protocol.create_error_response(frame.function_code, 0x02));
        }
        
        // 写入寄存器
        if let Some(register) = self.holding_registers.get_mut(addr as usize) {
            *register = value;
        }
        
        // 返回原始请求作为响应
        Ok(self.protocol.create_response(frame.function_code, &frame.data))
    }
    
    fn handle_write_multiple_registers(&mut self, frame: &ModbusFrame) -> Result<Vec<u8, 256>, ModbusError> {
        if frame.data.len() < 5 {
            return Err(ModbusError::InvalidData);
        }
        
        let start_addr = ((frame.data[0] as u16) << 8) | (frame.data[1] as u16);
        let count = ((frame.data[2] as u16) << 8) | (frame.data[3] as u16);
        let byte_count = frame.data[4] as usize;
        
        if byte_count != count as usize * 2 || frame.data.len() < 5 + byte_count {
            return Err(ModbusError::InvalidData);
        }
        
        if start_addr as usize + count as usize > self.holding_registers.len() {
            return Ok(self.protocol.create_error_response(frame.function_code, 0x02));
        }
        
        // 写入多个寄存器
        for i in 0..count {
            let data_offset = 5 + (i as usize * 2);
            let value = ((frame.data[data_offset] as u16) << 8) | (frame.data[data_offset + 1] as u16);
            
            if let Some(register) = self.holding_registers.get_mut((start_addr + i) as usize) {
                *register = value;
            }
        }
        
        // 响应包含起始地址和写入的寄存器数量
        let response_data = &frame.data[..4];
        Ok(self.protocol.create_response(frame.function_code, response_data))
    }
    
    pub fn set_holding_register(&mut self, addr: u16, value: u16) -> Result<(), ModbusError> {
        if addr as usize >= self.holding_registers.len() {
            // 扩展寄存器数组
            while self.holding_registers.len() <= addr as usize {
                self.holding_registers.push(0).map_err(|_| ModbusError::IllegalDataAddress)?;
            }
        }
        
        self.holding_registers[addr as usize] = value;
        Ok(())
    }
    
    pub fn get_holding_register(&self, addr: u16) -> Option<u16> {
        self.holding_registers.get(addr as usize).copied()
    }
}
```

### 2. 自定义JSON协议

```rust
// src/protocol/json_protocol.rs
use crate::protocol::traits::{Protocol, FrameParser};
use heapless::{Vec, String};
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct JsonMessage {
    pub id: u32,
    pub timestamp: u32,
    pub device_id: String<16>,
    pub message_type: String<16>,
    pub payload: JsonPayload,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(untagged)]
pub enum JsonPayload {
    Command {
        command: String<32>,
        parameters: Vec<String<32>, 8>,
    },
    Response {
        status: String<16>,
        data: Vec<f32, 16>,
    },
    Event {
        event_type: String<32>,
        severity: u8,
        description: String<64>,
    },
}

pub struct JsonProtocol {
    device_id: String<16>,
    message_counter: u32,
}

#[derive(Debug)]
pub enum JsonProtocolError {
    SerializationError,
    DeserializationError,
    InvalidMessage,
    BufferOverflow,
}

impl JsonProtocol {
    pub fn new(device_id: &str) -> Self {
        Self {
            device_id: String::from(device_id),
            message_counter: 0,
        }
    }
    
    pub fn create_command(&mut self, command: &str, params: &[&str]) -> Result<Vec<u8, 512>, JsonProtocolError> {
        let mut parameters = Vec::new();
        for &param in params {
            parameters.push(String::from(param))
                .map_err(|_| JsonProtocolError::BufferOverflow)?;
        }
        
        let message = JsonMessage {
            id: self.next_message_id(),
            timestamp: get_current_timestamp(),
            device_id: self.device_id.clone(),
            message_type: String::from("command"),
            payload: JsonPayload::Command {
                command: String::from(command),
                parameters,
            },
        };
        
        self.serialize_message(&message)
    }
    
    pub fn create_response(&mut self, status: &str, data: &[f32]) -> Result<Vec<u8, 512>, JsonProtocolError> {
        let mut response_data = Vec::new();
        for &value in data {
            response_data.push(value)
                .map_err(|_| JsonProtocolError::BufferOverflow)?;
        }
        
        let message = JsonMessage {
            id: self.next_message_id(),
            timestamp: get_current_timestamp(),
            device_id: self.device_id.clone(),
            message_type: String::from("response"),
            payload: JsonPayload::Response {
                status: String::from(status),
                data: response_data,
            },
        };
        
        self.serialize_message(&message)
    }
    
    pub fn create_event(&mut self, event_type: &str, severity: u8, description: &str) -> Result<Vec<u8, 512>, JsonProtocolError> {
        let message = JsonMessage {
            id: self.next_message_id(),
            timestamp: get_current_timestamp(),
            device_id: self.device_id.clone(),
            message_type: String::from("event"),
            payload: JsonPayload::Event {
                event_type: String::from(event_type),
                severity,
                description: String::from(description),
            },
        };
        
        self.serialize_message(&message)
    }
    
    fn serialize_message(&self, message: &JsonMessage) -> Result<Vec<u8, 512>, JsonProtocolError> {
        // 简化的JSON序列化（实际应用中使用serde_json_core）
        let mut buffer = Vec::new();
        
        // 手动构建JSON字符串
        let json_str = self.build_json_string(message)?;
        buffer.extend_from_slice(json_str.as_bytes())
            .map_err(|_| JsonProtocolError::BufferOverflow)?;
        
        // 添加换行符作为消息分隔符
        buffer.push(b'\n').ok();
        
        Ok(buffer)
    }
    
    fn build_json_string(&self, message: &JsonMessage) -> Result<String<512>, JsonProtocolError> {
        let mut json = String::new();
        
        json.push('{').ok();
        json.push_str(&format!("\"id\":{},", message.id)).ok();
        json.push_str(&format!("\"timestamp\":{},", message.timestamp)).ok();
        json.push_str(&format!("\"device_id\":\"{}\",", message.device_id)).ok();
        json.push_str(&format!("\"message_type\":\"{}\",", message.message_type)).ok();
        
        json.push_str("\"payload\":").ok();
        match &message.payload {
            JsonPayload::Command { command, parameters } => {
                json.push('{').ok();
                json.push_str(&format!("\"command\":\"{}\",", command)).ok();
                json.push_str("\"parameters\":[").ok();
                
                for (i, param) in parameters.iter().enumerate() {
                    if i > 0 { json.push(',').ok(); }
                    json.push_str(&format!("\"{}\"", param)).ok();
                }
                
                json.push_str("]}").ok();
            }
            JsonPayload::Response { status, data } => {
                json.push('{').ok();
                json.push_str(&format!("\"status\":\"{}\",", status)).ok();
                json.push_str("\"data\":[").ok();
                
                for (i, &value) in data.iter().enumerate() {
                    if i > 0 { json.push(',').ok(); }
                    json.push_str(&format!("{}", value)).ok();
                }
                
                json.push_str("]}").ok();
            }
            JsonPayload::Event { event_type, severity, description } => {
                json.push('{').ok();
                json.push_str(&format!("\"event_type\":\"{}\",", event_type)).ok();
                json.push_str(&format!("\"severity\":{},", severity)).ok();
                json.push_str(&format!("\"description\":\"{}\"", description)).ok();
                json.push('}').ok();
            }
        }
        
        json.push('}').ok();
        
        Ok(json)
    }
    
    fn next_message_id(&mut self) -> u32 {
        self.message_counter += 1;
        self.message_counter
    }
}

impl Protocol for JsonProtocol {
    type Error = JsonProtocolError;
    type Frame = JsonMessage;
    type Address = String<16>;
    
    fn encode(&self, data: &[u8], _address: Self::Address) -> Result<Vec<u8, 256>, Self::Error> {
        // 简单地将数据包装成事件消息
        let data_str = core::str::from_utf8(data)
            .map_err(|_| JsonProtocolError::InvalidMessage)?;
        
        let mut protocol = self.clone();
        let encoded = protocol.create_event("data", 1, data_str)?;
        
        // 截断到256字节
        let mut result = Vec::new();
        let copy_len = core::cmp::min(encoded.len(), 256);
        result.extend_from_slice(&encoded[..copy_len])
            .map_err(|_| JsonProtocolError::BufferOverflow)?;
        
        Ok(result)
    }
    
    fn decode(&self, buffer: &[u8]) -> Result<Option<Self::Frame>, Self::Error> {
        // 查找换行符
        if let Some(pos) = buffer.iter().position(|&b| b == b'\n') {
            let json_data = &buffer[..pos];
            let json_str = core::str::from_utf8(json_data)
                .map_err(|_| JsonProtocolError::DeserializationError)?;
            
            // 简化的JSON解析（实际应用中使用serde_json_core）
            self.parse_json_message(json_str)
        } else {
            Ok(None) // 消息不完整
        }
    }
    
    fn validate_frame(&self, frame: &Self::Frame) -> bool {
        // 验证设备ID和消息格式
        !frame.device_id.is_empty() && !frame.message_type.is_empty()
    }
    
    fn name(&self) -> &'static str {
        "JsonProtocol"
    }
}

impl Clone for JsonProtocol {
    fn clone(&self) -> Self {
        Self {
            device_id: self.device_id.clone(),
            message_counter: self.message_counter,
        }
    }
}

impl JsonProtocol {
    fn parse_json_message(&self, json_str: &str) -> Result<Option<JsonMessage>, JsonProtocolError> {
        // 简化的JSON解析实现
        // 实际应用中应该使用serde_json_core或其他JSON解析库
        
        // 这里只是一个示例，实际解析会更复杂
        if json_str.contains("\"message_type\":\"command\"") {
            // 解析命令消息
            Ok(Some(JsonMessage {
                id: 1,
                timestamp: get_current_timestamp(),
                device_id: String::from("unknown"),
                message_type: String::from("command"),
                payload: JsonPayload::Command {
                    command: String::from("unknown"),
                    parameters: Vec::new(),
                },
            }))
        } else {
            Err(JsonProtocolError::DeserializationError)
        }
    }
}

// JSON协议解析器
pub struct JsonProtocolParser {
    buffer: Vec<u8, 1024>,
    protocol: JsonProtocol,
}

impl JsonProtocolParser {
    pub fn new(device_id: &str) -> Self {
        Self {
            buffer: Vec::new(),
            protocol: JsonProtocol::new(device_id),
        }
    }
}

impl FrameParser for JsonProtocolParser {
    type Frame = JsonMessage;
    type Error = JsonProtocolError;
    
    fn parse_frame(&mut self, byte: u8) -> Result<Option<Self::Frame>, Self::Error> {
        self.buffer.push(byte).map_err(|_| JsonProtocolError::BufferOverflow)?;
        
        // 查找消息分隔符
        if byte == b'\n' {
            let message = self.protocol.decode(&self.buffer)?;
            self.buffer.clear();
            Ok(message)
        } else {
            Ok(None)
        }
    }
    
    fn reset(&mut self) {
        self.buffer.clear();
    }
    
    fn state(&self) -> crate::protocol::traits::ParserState {
        if self.buffer.is_empty() {
            crate::protocol::traits::ParserState::WaitingStart
        } else {
            crate::protocol::traits::ParserState::ReadingData
        }
    }
}

fn get_current_timestamp() -> u32 {
    // 实际实现中应该返回当前时间戳
    0
}
```

## 协议测试和验证

### 1. 协议测试框架

```rust
// src/protocol/test_framework.rs
use crate::protocol::traits::{Protocol, FrameParser};
use heapless::Vec;

pub struct ProtocolTester<P: Protocol> {
    protocol: P,
    test_cases: Vec<TestCase, 32>,
    results: Vec<TestResult, 32>,
}

#[derive(Debug, Clone)]
pub struct TestCase {
    pub name: String<64>,
    pub input_data: Vec<u8, 256>,
    pub expected_output: Option<Vec<u8, 256>>,
    pub should_fail: bool,
}

#[derive(Debug)]
pub struct TestResult {
    pub test_name: String<64>,
    pub passed: bool,
    pub error_message: Option<String<128>>,
    pub execution_time_us: u32,
}

impl<P: Protocol> ProtocolTester<P> {
    pub fn new(protocol: P) -> Self {
        Self {
            protocol,
            test_cases: Vec::new(),
            results: Vec::new(),
        }
    }
    
    pub fn add_test_case(&mut self, test_case: TestCase) -> Result<(), &'static str> {
        self.test_cases.push(test_case).map_err(|_| "Test case buffer full")
    }
    
    pub fn run_all_tests(&mut self) {
        self.results.clear();
        
        for test_case in &self.test_cases {
            let result = self.run_single_test(test_case);
            self.results.push(result).ok();
        }
    }
    
    fn run_single_test(&self, test_case: &TestCase) -> TestResult {
        let start_time = get_microsecond_timer();
        
        let result = match self.protocol.decode(&test_case.input_data) {
            Ok(Some(_frame)) => {
                if test_case.should_fail {
                    TestResult {
                        test_name: test_case.name.clone(),
                        passed: false,
                        error_message: Some(String::from("Expected failure but test passed")),
                        execution_time_us: get_microsecond_timer() - start_time,
                    }
                } else {
                    TestResult {
                        test_name: test_case.name.clone(),
                        passed: true,
                        error_message: None,
                        execution_time_us: get_microsecond_timer() - start_time,
                    }
                }
            }
            Ok(None) => {
                TestResult {
                    test_name: test_case.name.clone(),
                    passed: !test_case.should_fail,
                    error_message: if test_case.should_fail { 
                        None 
                    } else { 
                        Some(String::from("No frame decoded")) 
                    },
                    execution_time_us: get_microsecond_timer() - start_time,
                }
            }
            Err(_e) => {
                TestResult {
                    test_name: test_case.name.clone(),
                    passed: test_case.should_fail,
                    error_message: if test_case.should_fail { 
                        None 
                    } else { 
                        Some(String::from("Decode error")) 
                    },
                    execution_time_us: get_microsecond_timer() - start_time,
                }
            }
        };
        
        result
    }
    
    pub fn get_test_summary(&self) -> TestSummary {
        let total_tests = self.results.len();
        let passed_tests = self.results.iter().filter(|r| r.passed).count();
        let failed_tests = total_tests - passed_tests;
        
        let total_time: u32 = self.results.iter().map(|r| r.execution_time_us).sum();
        let average_time = if total_tests > 0 { total_time / total_tests as u32 } else { 0 };
        
        TestSummary {
            total_tests: total_tests as u32,
            passed_tests: passed_tests as u32,
            failed_tests: failed_tests as u32,
            success_rate: if total_tests > 0 { 
                (passed_tests as f32) / (total_tests as f32) 
            } else { 
                0.0 
            },
            total_execution_time_us: total_time,
            average_execution_time_us: average_time,
        }
    }
    
    pub fn get_failed_tests(&self) -> Vec<&TestResult, 32> {
        let mut failed = Vec::new();
        for result in &self.results {
            if !result.passed {
                failed.push(result).ok();
            }
        }
        failed
    }
}

#[derive(Debug)]
pub struct TestSummary {
    pub total_tests: u32,
    pub passed_tests: u32,
    pub failed_tests: u32,
    pub success_rate: f32,
    pub total_execution_time_us: u32,
    pub average_execution_time_us: u32,
}

// 性能基准测试
pub struct ProtocolBenchmark<P: Protocol> {
    protocol: P,
    test_data: Vec<Vec<u8, 256>, 16>,
}

impl<P: Protocol> ProtocolBenchmark<P> {
    pub fn new(protocol: P) -> Self {
        Self {
            protocol,
            test_data: Vec::new(),
        }
    }
    
    pub fn add_benchmark_data(&mut self, data: Vec<u8, 256>) -> Result<(), &'static str> {
        self.test_data.push(data).map_err(|_| "Benchmark data buffer full")
    }
    
    pub fn run_decode_benchmark(&self, iterations: u32) -> BenchmarkResult {
        let start_time = get_microsecond_timer();
        let mut successful_decodes = 0u32;
        let mut total_bytes = 0u32;
        
        for _ in 0..iterations {
            for test_data in &self.test_data {
                total_bytes += test_data.len() as u32;
                
                match self.protocol.decode(test_data) {
                    Ok(Some(_)) => successful_decodes += 1,
                    _ => {}
                }
            }
        }
        
        let end_time = get_microsecond_timer();
        let total_time = end_time - start_time;
        
        BenchmarkResult {
            operation: String::from("decode"),
            iterations,
            total_time_us: total_time,
            average_time_us: total_time / iterations,
            throughput_mbps: if total_time > 0 {
                (total_bytes as f32 * 8.0) / (total_time as f32)
            } else {
                0.0
            },
            success_rate: (successful_decodes as f32) / ((iterations * self.test_data.len() as u32) as f32),
        }
    }
    
    pub fn run_encode_benchmark(&self, iterations: u32, address: P::Address) -> BenchmarkResult 
    where 
        P::Address: Clone,
    {
        let start_time = get_microsecond_timer();
        let mut successful_encodes = 0u32;
        let mut total_bytes = 0u32;
        
        for _ in 0..iterations {
            for test_data in &self.test_data {
                total_bytes += test_data.len() as u32;
                
                match self.protocol.encode(test_data, address.clone()) {
                    Ok(_) => successful_encodes += 1,
                    _ => {}
                }
            }
        }
        
        let end_time = get_microsecond_timer();
        let total_time = end_time - start_time;
        
        BenchmarkResult {
            operation: String::from("encode"),
            iterations,
            total_time_us: total_time,
            average_time_us: total_time / iterations,
            throughput_mbps: if total_time > 0 {
                (total_bytes as f32 * 8.0) / (total_time as f32)
            } else {
                0.0
            },
            success_rate: (successful_encodes as f32) / ((iterations * self.test_data.len() as u32) as f32),
        }
    }
}

#[derive(Debug)]
pub struct BenchmarkResult {
    pub operation: String<16>,
    pub iterations: u32,
    pub total_time_us: u32,
    pub average_time_us: u32,
    pub throughput_mbps: f32,
    pub success_rate: f32,
}

fn get_microsecond_timer() -> u32 {
    // 实际实现中应该返回微秒级时间戳
    0
}
```

### 2. 协议兼容性测试

```rust
// src/protocol/compatibility_test.rs
use crate::protocol::traits::Protocol;
use heapless::Vec;

pub struct CompatibilityTester;

impl CompatibilityTester {
    pub fn test_protocol_interoperability<P1, P2>(
        protocol1: &P1,
        protocol2: &P2,
        test_data: &[u8],
        address1: P1::Address,
        address2: P2::Address,
    ) -> InteroperabilityResult
    where
        P1: Protocol,
        P2: Protocol,
        P1::Address: Clone,
        P2::Address: Clone,
    {
        let mut results = InteroperabilityResult::new();
        
        // 测试P1编码 -> P2解码
        match protocol1.encode(test_data, address1.clone()) {
            Ok(encoded1) => {
                match protocol2.decode(&encoded1) {
                    Ok(Some(_)) => results.p1_to_p2_success = true,
                    Ok(None) => results.p1_to_p2_partial = true,
                    Err(_) => results.p1_to_p2_error = true,
                }
            }
            Err(_) => results.p1_encode_error = true,
        }
        
        // 测试P2编码 -> P1解码
        match protocol2.encode(test_data, address2.clone()) {
            Ok(encoded2) => {
                match protocol1.decode(&encoded2) {
                    Ok(Some(_)) => results.p2_to_p1_success = true,
                    Ok(None) => results.p2_to_p1_partial = true,
                    Err(_) => results.p2_to_p1_error = true,
                }
            }
            Err(_) => results.p2_encode_error = true,
        }
        
        results
    }
    
    pub fn test_version_compatibility<P: Protocol>(
        old_protocol: &P,
        new_protocol: &P,
        test_cases: &[Vec<u8, 256>],
        address: P::Address,
    ) -> VersionCompatibilityResult
    where
        P::Address: Clone,
    {
        let mut results = VersionCompatibilityResult::new();
        
        for test_data in test_cases {
            // 测试向前兼容性（新协议解码旧协议数据）
            match old_protocol.encode(test_data, address.clone()) {
                Ok(old_encoded) => {
                    match new_protocol.decode(&old_encoded) {
                        Ok(Some(_)) => results.forward_compatible_count += 1,
                        _ => results.forward_incompatible_count += 1,
                    }
                }
                Err(_) => results.old_encode_errors += 1,
            }
            
            // 测试向后兼容性（旧协议解码新协议数据）
            match new_protocol.encode(test_data, address.clone()) {
                Ok(new_encoded) => {
                    match old_protocol.decode(&new_encoded) {
                        Ok(Some(_)) => results.backward_compatible_count += 1,
                        _ => results.backward_incompatible_count += 1,
                    }
                }
                Err(_) => results.new_encode_errors += 1,
            }
        }
        
        results.total_tests = test_cases.len() as u32;
        results
    }
}

#[derive(Debug)]
pub struct InteroperabilityResult {
    pub p1_to_p2_success: bool,
    pub p1_to_p2_partial: bool,
    pub p1_to_p2_error: bool,
    pub p2_to_p1_success: bool,
    pub p2_to_p1_partial: bool,
    pub p2_to_p1_error: bool,
    pub p1_encode_error: bool,
    pub p2_encode_error: bool,
}

impl InteroperabilityResult {
    fn new() -> Self {
        Self {
            p1_to_p2_success: false,
            p1_to_p2_partial: false,
            p1_to_p2_error: false,
            p2_to_p1_success: false,
            p2_to_p1_partial: false,
            p2_to_p1_error: false,
            p1_encode_error: false,
            p2_encode_error: false,
        }
    }
    
    pub fn is_fully_compatible(&self) -> bool {
        self.p1_to_p2_success && self.p2_to_p1_success &&
        !self.p1_encode_error && !self.p2_encode_error
    }
}

#[derive(Debug)]
pub struct VersionCompatibilityResult {
    pub total_tests: u32,
    pub forward_compatible_count: u32,
    pub forward_incompatible_count: u32,
    pub backward_compatible_count: u32,
    pub backward_incompatible_count: u32,
    pub old_encode_errors: u32,
    pub new_encode_errors: u32,
}

impl VersionCompatibilityResult {
    fn new() -> Self {
        Self {
            total_tests: 0,
            forward_compatible_count: 0,
            forward_incompatible_count: 0,
            backward_compatible_count: 0,
            backward_incompatible_count: 0,
            old_encode_errors: 0,
            new_encode_errors: 0,
        }
    }
    
    pub fn forward_compatibility_rate(&self) -> f32 {
        if self.total_tests > 0 {
            (self.forward_compatible_count as f32) / (self.total_tests as f32)
        } else {
            0.0
        }
    }
    
    pub fn backward_compatibility_rate(&self) -> f32 {
        if self.total_tests > 0 {
            (self.backward_compatible_count as f32) / (self.total_tests as f32)
        } else {
            0.0
        }
    }
}
```

## 总结

协议实现是串口通信系统的核心组件，需要考虑以下关键要素：

1. **协议设计**：清晰的分层架构和完整的错误处理机制
2. **编码解码**：高效的数据序列化和反序列化
3. **可靠性**：确认应答、重传机制和流控制
4. **兼容性**：版本兼容和互操作性
5. **性能**：编码解码效率和内存使用优化
6. **测试验证**：完整的测试框架和基准测试

通过合理的协议设计和实现，可以构建稳定、高效的串口通信系统，满足各种应用场景的需求。

## 参考资料

- [Modbus Protocol Specification](https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)
- [Serial Communication Protocols](https://en.wikipedia.org/wiki/Serial_communication)
- [Rust Embedded Book - Serial Communication](https://docs.rust-embedded.org/book/)
- [serde Documentation](https://serde.rs/)
- [heapless Documentation](https://docs.rs/heapless/)