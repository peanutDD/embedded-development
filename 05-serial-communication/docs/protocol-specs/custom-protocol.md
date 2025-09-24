# 自定义串口协议规范

## 概述

本文档定义了一套轻量级、高效的自定义串口通信协议，专为嵌入式系统设计。该协议支持可靠传输、流控制和扩展性，适用于各种工业和物联网应用场景。

## 设计原则

### 核心特性
- **轻量级**: 最小化协议开销
- **可靠性**: 内置错误检测和重传机制
- **扩展性**: 支持自定义命令和数据类型
- **实时性**: 低延迟通信
- **兼容性**: 支持不同波特率和硬件平台

### 设计目标
- 协议开销 < 10%
- 端到端延迟 < 10ms
- 错误检测率 > 99.9%
- 支持设备数量 > 255

## 协议架构

### 分层结构
```
+------------------+
|   应用层 (APP)   |  <- 用户数据和命令
+------------------+
|   传输层 (TXP)   |  <- 可靠传输和流控制
+------------------+
|   数据链路层     |  <- 帧格式和错误检测
+------------------+
|   物理层 (PHY)   |  <- UART硬件接口
+------------------+
```

## 帧格式

### 基本帧结构
```
+------+------+------+------+--------+------+------+
| SOF  | LEN  | SEQ  | CMD  |  DATA  | CRC  | EOF  |
| 1B   | 1B   | 1B   | 1B   | 0-250B | 2B   | 1B   |
+------+------+------+------+--------+------+------+
```

### 字段详细说明

#### 1. 帧起始符 (SOF - Start of Frame)
- **值**: 0xAA
- **用途**: 帧同步和边界检测

#### 2. 长度字段 (LEN - Length)
- **范围**: 0-250
- **含义**: DATA字段的字节数
- **特殊值**: 0表示无数据载荷

#### 3. 序列号 (SEQ - Sequence)
```rust
#[repr(u8)]
pub struct SequenceNumber {
    seq_num: u8,  // 位0-5: 序列号 (0-63)
    ack_req: u8,  // 位6: 需要确认 (0/1)
    is_ack: u8,   // 位7: 确认帧 (0/1)
}

impl SequenceNumber {
    pub fn new(seq: u8, ack_required: bool, is_ack: bool) -> Self {
        Self {
            seq_num: seq & 0x3F,
            ack_req: if ack_required { 1 } else { 0 },
            is_ack: if is_ack { 1 } else { 0 },
        }
    }
    
    pub fn to_byte(&self) -> u8 {
        self.seq_num | (self.ack_req << 6) | (self.is_ack << 7)
    }
}
```

#### 4. 命令字段 (CMD - Command)
```rust
#[repr(u8)]
pub enum CommandType {
    // 系统命令 (0x00-0x0F)
    Ping = 0x00,
    Pong = 0x01,
    Reset = 0x02,
    GetInfo = 0x03,
    SetConfig = 0x04,
    GetConfig = 0x05,
    
    // 数据传输 (0x10-0x1F)
    DataTransfer = 0x10,
    StreamStart = 0x11,
    StreamData = 0x12,
    StreamEnd = 0x13,
    
    // 控制命令 (0x20-0x2F)
    SetOutput = 0x20,
    GetInput = 0x21,
    SetPwm = 0x22,
    GetAdc = 0x23,
    
    // 文件操作 (0x30-0x3F)
    FileOpen = 0x30,
    FileRead = 0x31,
    FileWrite = 0x32,
    FileClose = 0x33,
    FileList = 0x34,
    
    // 诊断命令 (0x40-0x4F)
    GetStatus = 0x40,
    GetStats = 0x41,
    ClearStats = 0x42,
    RunTest = 0x43,
    
    // 用户自定义 (0x80-0xFF)
    UserDefined = 0x80,
}
```

#### 5. 数据字段 (DATA)
- **长度**: 0-250字节
- **格式**: 根据命令类型确定
- **编码**: 二进制或JSON

#### 6. CRC校验 (CRC)
- **算法**: CRC-16-CCITT
- **多项式**: 0x1021
- **初始值**: 0xFFFF
- **范围**: SOF到DATA的所有字节

#### 7. 帧结束符 (EOF - End of Frame)
- **值**: 0x55
- **用途**: 帧结束标识

## 数据类型定义

### 基本数据类型
```rust
#[repr(C, packed)]
pub struct BasicTypes {
    pub bool_val: bool,      // 1字节
    pub u8_val: u8,          // 1字节
    pub i8_val: i8,          // 1字节
    pub u16_val: u16,        // 2字节，小端序
    pub i16_val: i16,        // 2字节，小端序
    pub u32_val: u32,        // 4字节，小端序
    pub i32_val: i32,        // 4字节，小端序
    pub f32_val: f32,        // 4字节，IEEE 754
}
```

### 复合数据类型
```rust
#[derive(Serialize, Deserialize)]
pub struct DeviceInfo {
    pub device_id: u32,
    pub firmware_version: [u8; 4],  // major.minor.patch.build
    pub hardware_version: u16,
    pub serial_number: [u8; 16],
    pub capabilities: u32,          // 位掩码
}

#[derive(Serialize, Deserialize)]
pub struct SensorData {
    pub timestamp: u32,             // Unix时间戳
    pub sensor_id: u8,
    pub data_type: u8,
    pub value: f32,
    pub unit: [u8; 4],              // 单位字符串
    pub quality: u8,                // 数据质量 0-100
}
```

## 命令详细规范

### 系统命令

#### PING (0x00)
```rust
// 请求
struct PingRequest {
    timestamp: u32,     // 发送时间戳
    payload: [u8; 16],  // 可选载荷
}

// 响应
struct PingResponse {
    timestamp: u32,     // 原始时间戳
    response_time: u32, // 响应时间戳
    payload: [u8; 16],  // 回显载荷
}
```

#### GET_INFO (0x03)
```rust
// 请求
struct GetInfoRequest {
    info_type: u8,      // 信息类型
}

// 响应
struct GetInfoResponse {
    info_type: u8,
    data_length: u8,
    data: Vec<u8>,      // 设备信息数据
}
```

### 数据传输命令

#### DATA_TRANSFER (0x10)
```rust
// 单次数据传输
struct DataTransferFrame {
    data_type: u8,      // 数据类型标识
    data_length: u8,    // 数据长度
    data: Vec<u8>,      // 实际数据
}
```

#### STREAM_START (0x11)
```rust
// 流传输开始
struct StreamStartFrame {
    stream_id: u16,     // 流标识符
    total_length: u32,  // 总数据长度
    chunk_size: u16,    // 分块大小
    data_type: u8,      // 数据类型
}
```

#### STREAM_DATA (0x12)
```rust
// 流数据块
struct StreamDataFrame {
    stream_id: u16,     // 流标识符
    chunk_index: u16,   // 块索引
    chunk_data: Vec<u8>, // 块数据
}
```

### 控制命令

#### SET_OUTPUT (0x20)
```rust
struct SetOutputFrame {
    pin_mask: u32,      // 引脚掩码
    pin_values: u32,    // 引脚值
}
```

#### GET_INPUT (0x21)
```rust
// 请求
struct GetInputRequest {
    pin_mask: u32,      // 要读取的引脚掩码
}

// 响应
struct GetInputResponse {
    pin_values: u32,    // 引脚当前值
    timestamp: u32,     // 读取时间戳
}
```

## 错误处理

### 错误代码
```rust
#[repr(u8)]
pub enum ErrorCode {
    Success = 0x00,
    
    // 协议错误 (0x01-0x0F)
    InvalidFrame = 0x01,
    InvalidCrc = 0x02,
    InvalidLength = 0x03,
    InvalidCommand = 0x04,
    InvalidSequence = 0x05,
    
    // 系统错误 (0x10-0x1F)
    SystemBusy = 0x10,
    InsufficientMemory = 0x11,
    HardwareError = 0x12,
    TimeoutError = 0x13,
    
    // 应用错误 (0x20-0x2F)
    InvalidParameter = 0x20,
    OperationFailed = 0x21,
    ResourceNotFound = 0x22,
    PermissionDenied = 0x23,
    
    // 用户定义 (0x80-0xFF)
    UserError = 0x80,
}
```

### 错误响应帧
```rust
struct ErrorResponse {
    error_code: u8,     // 错误代码
    error_data: Vec<u8>, // 错误详细信息
}
```

## 流控制机制

### 滑动窗口协议
```rust
pub struct SlidingWindow {
    window_size: u8,        // 窗口大小 (1-32)
    send_base: u8,          // 发送窗口基址
    next_seq_num: u8,       // 下一个序列号
    expected_seq_num: u8,   // 期望接收序列号
    ack_buffer: [bool; 64], // 确认缓冲区
}

impl SlidingWindow {
    pub fn can_send(&self) -> bool {
        let outstanding = (self.next_seq_num - self.send_base) & 0x3F;
        outstanding < self.window_size
    }
    
    pub fn send_frame(&mut self) -> u8 {
        let seq = self.next_seq_num;
        self.next_seq_num = (self.next_seq_num + 1) & 0x3F;
        seq
    }
    
    pub fn receive_ack(&mut self, ack_num: u8) {
        if self.is_in_window(ack_num) {
            self.ack_buffer[ack_num as usize] = true;
            self.advance_window();
        }
    }
}
```

### 重传机制
```rust
pub struct RetransmissionManager {
    timeout_ms: u32,        // 重传超时
    max_retries: u8,        // 最大重传次数
    pending_frames: HashMap<u8, PendingFrame>,
}

struct PendingFrame {
    data: Vec<u8>,
    timestamp: u32,
    retry_count: u8,
}
```

## 性能优化

### 缓冲区管理
```rust
pub struct ProtocolBuffers {
    tx_buffer: RingBuffer<u8, 1024>,    // 发送缓冲区
    rx_buffer: RingBuffer<u8, 1024>,    // 接收缓冲区
    frame_buffer: [u8; 256],            // 帧组装缓冲区
}
```

### 零拷贝优化
```rust
pub trait ZeroCopyFrame {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, ProtocolError>;
    fn deserialize_from(buffer: &[u8]) -> Result<Self, ProtocolError>
    where
        Self: Sized;
}
```

## 安全特性

### 认证机制
```rust
struct AuthFrame {
    auth_type: u8,      // 认证类型
    challenge: [u8; 16], // 挑战值
    response: [u8; 16],  // 响应值
}
```

### 加密支持
```rust
#[cfg(feature = "encryption")]
struct EncryptedFrame {
    iv: [u8; 16],       // 初始化向量
    encrypted_data: Vec<u8>, // 加密数据
    auth_tag: [u8; 16], // 认证标签
}
```

## 实现示例

### 协议栈初始化
```rust
use heapless::pool::{Pool, Node};

pub struct CustomProtocol {
    config: ProtocolConfig,
    state: ProtocolState,
    buffers: ProtocolBuffers,
    window: SlidingWindow,
    retrans: RetransmissionManager,
}

impl CustomProtocol {
    pub fn new(config: ProtocolConfig) -> Self {
        Self {
            config,
            state: ProtocolState::Idle,
            buffers: ProtocolBuffers::new(),
            window: SlidingWindow::new(config.window_size),
            retrans: RetransmissionManager::new(config.timeout_ms),
        }
    }
    
    pub fn send_frame(&mut self, cmd: CommandType, data: &[u8]) -> Result<(), ProtocolError> {
        if !self.window.can_send() {
            return Err(ProtocolError::WindowFull);
        }
        
        let seq = self.window.send_frame();
        let frame = self.build_frame(seq, cmd, data)?;
        self.transmit_frame(&frame)?;
        
        Ok(())
    }
    
    pub fn process_received_data(&mut self, data: &[u8]) -> Result<Vec<ReceivedFrame>, ProtocolError> {
        self.buffers.rx_buffer.extend_from_slice(data)?;
        let mut frames = Vec::new();
        
        while let Some(frame) = self.extract_frame()? {
            if self.validate_frame(&frame)? {
                frames.push(frame);
            }
        }
        
        Ok(frames)
    }
}
```

## 测试和验证

### 单元测试
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_frame_serialization() {
        let frame = Frame::new(CommandType::Ping, &[1, 2, 3, 4]);
        let serialized = frame.serialize();
        let deserialized = Frame::deserialize(&serialized).unwrap();
        assert_eq!(frame, deserialized);
    }
    
    #[test]
    fn test_sliding_window() {
        let mut window = SlidingWindow::new(4);
        assert!(window.can_send());
        
        for _ in 0..4 {
            window.send_frame();
        }
        assert!(!window.can_send());
    }
}
```

### 性能基准测试
```rust
#[cfg(test)]
mod benchmarks {
    use criterion::{black_box, criterion_group, criterion_main, Criterion};
    
    fn benchmark_frame_processing(c: &mut Criterion) {
        c.bench_function("frame_serialize", |b| {
            let frame = Frame::new(CommandType::DataTransfer, &[0u8; 100]);
            b.iter(|| black_box(frame.serialize()));
        });
    }
    
    criterion_group!(benches, benchmark_frame_processing);
    criterion_main!(benches);
}
```

## 参考实现

### 配置示例
```rust
pub struct ProtocolConfig {
    pub device_id: u32,
    pub window_size: u8,
    pub timeout_ms: u32,
    pub max_retries: u8,
    pub enable_encryption: bool,
    pub enable_compression: bool,
}

impl Default for ProtocolConfig {
    fn default() -> Self {
        Self {
            device_id: 0x12345678,
            window_size: 8,
            timeout_ms: 1000,
            max_retries: 3,
            enable_encryption: false,
            enable_compression: false,
        }
    }
}
```

## 版本历史

- **v1.0** (2024-01): 初始协议定义
- **v1.1** (2024-02): 添加流控制机制
- **v1.2** (2024-03): 增加安全特性和性能优化
- **v1.3** (2024-04): 完善错误处理和测试框架