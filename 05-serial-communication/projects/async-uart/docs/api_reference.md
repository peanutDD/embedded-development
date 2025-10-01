# AsyncUART API 参考

本文档提供了AsyncUART库所有公共API的详细参考。

## 目录

1. [核心类型](#核心类型)
2. [配置API](#配置api)
3. [缓冲区API](#缓冲区api)
4. [协议API](#协议api)
5. [HAL适配器API](#hal适配器api)
6. [错误类型](#错误类型)
7. [Trait定义](#trait定义)

## 核心类型

### AsyncUart

主要的UART通信接口。

```rust
pub struct AsyncUart<T: HalAdapter> {
    // 内部字段...
}
```

#### 方法

##### `read`
```rust
pub async fn read(&mut self, buf: &mut [u8]) -> AsyncUartResult<usize>
```
从UART读取数据到缓冲区。

**参数**:
- `buf`: 目标缓冲区

**返回**: 读取的字节数

**示例**:
```rust
let mut buffer = [0u8; 64];
let bytes_read = uart.read(&mut buffer).await?;
```

##### `write_all`
```rust
pub async fn write_all(&mut self, buf: &[u8]) -> AsyncUartResult<usize>
```
将所有数据写入UART。

**参数**:
- `buf`: 要写入的数据

**返回**: 写入的字节数

**示例**:
```rust
let data = b"Hello, UART!";
let bytes_written = uart.write_all(data).await?;
```

##### `write_u8`
```rust
pub async fn write_u8(&mut self, byte: u8) -> AsyncUartResult<()>
```
写入单个字节。

##### `read_u8`
```rust
pub async fn read_u8(&mut self) -> AsyncUartResult<u8>
```
读取单个字节。

##### `flush`
```rust
pub async fn flush(&mut self) -> AsyncUartResult<()>
```
刷新输出缓冲区。

##### `close`
```rust
pub async fn close(self) -> AsyncUartResult<()>
```
关闭UART连接。

##### `set_baud_rate`
```rust
pub async fn set_baud_rate(&mut self, baud_rate: u32) -> AsyncUartResult<()>
```
设置波特率。

##### `set_timeout`
```rust
pub async fn set_timeout(&mut self, timeout: Duration) -> AsyncUartResult<()>
```
设置操作超时。

##### `get_stats`
```rust
pub fn get_stats(&self) -> UartStats
```
获取UART统计信息。

##### `reset`
```rust
pub async fn reset(&mut self) -> AsyncUartResult<()>
```
重置UART状态。

### AsyncUartBuilder

用于构建AsyncUart实例的构建器。

```rust
pub struct AsyncUartBuilder<T: HalAdapter> {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new() -> Self
```
创建新的构建器实例。

##### `with_config`
```rust
pub fn with_config(mut self, config: UartConfig) -> Self
```
设置UART配置。

##### `with_pins`
```rust
pub fn with_pins(mut self, pins: UartPins) -> Self
```
设置引脚配置。

##### `with_adapter`
```rust
pub fn with_adapter(mut self, adapter: T) -> Self
```
设置HAL适配器。

##### `with_buffer`
```rust
pub fn with_buffer<B: Buffer>(mut self, buffer: B) -> Self
```
设置自定义缓冲区。

##### `build`
```rust
pub async fn build(self) -> AsyncUartResult<AsyncUart<T>>
```
构建AsyncUart实例。

**示例**:
```rust
let uart = AsyncUartBuilder::new()
    .with_config(presets::standard_115200())
    .with_pins(UartPins::new(1, 2))
    .with_adapter(GenericHalAdapter::new())
    .build()
    .await?;
```

## 配置API

### UartConfig

UART配置结构体。

```rust
pub struct UartConfig {
    pub baud_rate: u32,
    pub data_bits: u8,
    pub stop_bits: StopBits,
    pub parity: Parity,
    pub flow_control: FlowControl,
    pub timeout: Duration,
    pub buffer_size: usize,
    pub enable_dma: bool,
}
```

#### 方法

##### `builder`
```rust
pub fn builder() -> UartConfigBuilder
```
创建配置构建器。

##### `validate`
```rust
pub fn validate(&self) -> Result<(), ConfigError>
```
验证配置有效性。

### UartConfigBuilder

配置构建器。

```rust
pub struct UartConfigBuilder {
    // 内部字段...
}
```

#### 方法

##### `baud_rate`
```rust
pub fn baud_rate(mut self, baud_rate: u32) -> Self
```
设置波特率。

##### `data_bits`
```rust
pub fn data_bits(mut self, data_bits: u8) -> Self
```
设置数据位数。

##### `stop_bits`
```rust
pub fn stop_bits(mut self, stop_bits: StopBits) -> Self
```
设置停止位。

##### `parity`
```rust
pub fn parity(mut self, parity: Parity) -> Self
```
设置校验位。

##### `flow_control`
```rust
pub fn flow_control(mut self, flow_control: FlowControl) -> Self
```
设置流控制。

##### `timeout`
```rust
pub fn timeout(mut self, timeout: Duration) -> Self
```
设置超时时间。

##### `buffer_size`
```rust
pub fn buffer_size(mut self, size: usize) -> Self
```
设置缓冲区大小。

##### `enable_dma`
```rust
pub fn enable_dma(mut self, enable: bool) -> Self
```
启用/禁用DMA。

##### `build`
```rust
pub fn build(self) -> Result<UartConfig, ConfigError>
```
构建配置。

### 枚举类型

#### StopBits
```rust
pub enum StopBits {
    One,
    OneAndHalf,
    Two,
}
```

#### Parity
```rust
pub enum Parity {
    None,
    Even,
    Odd,
    Mark,
    Space,
}
```

#### FlowControl
```rust
pub enum FlowControl {
    None,
    Software,
    Hardware,
    RtsCts,
}
```

### 预设配置

#### `presets` 模块

```rust
pub mod presets {
    pub fn standard_115200() -> UartConfig;
    pub fn high_speed_921600() -> UartConfig;
    pub fn low_power_9600() -> UartConfig;
    pub fn debug_config() -> UartConfig;
}
```

## 缓冲区API

### Buffer Trait

所有缓冲区的基础trait。

```rust
pub trait Buffer: Send + Sync {
    async fn write(&mut self, data: &[u8]) -> Result<usize, BufferError>;
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, BufferError>;
    fn available_space(&self) -> usize;
    fn available_data(&self) -> usize;
    fn capacity(&self) -> usize;
    fn is_empty(&self) -> bool;
    fn is_full(&self) -> bool;
    async fn flush(&mut self) -> Result<(), BufferError>;
    async fn clear(&mut self) -> Result<(), BufferError>;
}
```

### ReadableBuffer Trait

可读缓冲区trait。

```rust
pub trait ReadableBuffer: Buffer {
    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), BufferError>;
    async fn read_until(&mut self, delimiter: u8, buf: &mut Vec<u8>) -> Result<usize, BufferError>;
    fn peek(&self, buf: &mut [u8]) -> Result<usize, BufferError>;
}
```

### WritableBuffer Trait

可写缓冲区trait。

```rust
pub trait WritableBuffer: Buffer {
    async fn write_all(&mut self, data: &[u8]) -> Result<(), BufferError>;
    async fn write_vectored(&mut self, bufs: &[&[u8]]) -> Result<usize, BufferError>;
}
```

### RingBuffer

环形缓冲区实现。

```rust
pub struct RingBuffer {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new(capacity: usize) -> Self
```
创建指定容量的环形缓冲区。

##### `with_overflow_strategy`
```rust
pub fn with_overflow_strategy(capacity: usize, strategy: OverflowStrategy) -> Self
```
创建带溢出策略的环形缓冲区。

##### `len`
```rust
pub fn len(&self) -> usize
```
获取当前数据长度。

##### `remaining`
```rust
pub fn remaining(&self) -> usize
```
获取剩余空间。

### DmaBuffer

DMA缓冲区实现。

```rust
pub struct DmaBuffer {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new(config: DmaBufferConfig) -> Result<Self, BufferError>
```
创建DMA缓冲区。

##### `get_stats`
```rust
pub fn get_stats(&self) -> DmaStats
```
获取DMA统计信息。

### DmaBufferConfig

DMA缓冲区配置。

```rust
pub struct DmaBufferConfig {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new() -> Self
```
创建默认配置。

##### `with_size`
```rust
pub fn with_size(mut self, size: usize) -> Self
```
设置缓冲区大小。

##### `with_alignment`
```rust
pub fn with_alignment(mut self, alignment: usize) -> Self
```
设置内存对齐。

##### `enable_stats`
```rust
pub fn enable_stats(mut self, enable: bool) -> Self
```
启用统计功能。

### StreamBuffer

流式缓冲区实现。

```rust
pub struct StreamBuffer {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new(config: StreamBufferConfig) -> Result<Self, BufferError>
```
创建流式缓冲区。

##### `pause`
```rust
pub async fn pause(&mut self) -> Result<(), BufferError>
```
暂停流。

##### `resume`
```rust
pub async fn resume(&mut self) -> Result<(), BufferError>
```
恢复流。

##### `is_paused`
```rust
pub fn is_paused(&self) -> bool
```
检查是否暂停。

##### `write_block`
```rust
pub async fn write_block(&mut self, data: &[u8]) -> Result<(), BufferError>
```
写入数据块。

##### `read_block`
```rust
pub async fn read_block(&mut self, size: usize) -> Result<Vec<u8>, BufferError>
```
读取数据块。

### OverflowStrategy

溢出策略枚举。

```rust
pub enum OverflowStrategy {
    Block,      // 阻塞等待
    Overwrite,  // 覆写旧数据
    Drop,       // 丢弃新数据
    Error,      // 返回错误
}
```

## 协议API

### ProtocolHandler Trait

协议处理器基础trait。

```rust
pub trait ProtocolHandler: Send + Sync {
    async fn handle_received_data(&mut self, data: &[u8]) -> Result<Vec<ProtocolMessage>, ProtocolError>;
    async fn encode_message(&mut self, message: &ProtocolMessage) -> Result<Vec<u8>, ProtocolError>;
    async fn decode_message(&mut self, data: &[u8]) -> Result<ProtocolMessage, ProtocolError>;
    async fn reset(&mut self) -> Result<(), ProtocolError>;
    fn get_config(&self) -> &ProtocolConfig;
    async fn set_config(&mut self, config: ProtocolConfig) -> Result<(), ProtocolError>;
    async fn handle_timeout(&mut self) -> Result<(), ProtocolError>;
    fn get_stats(&self) -> ProtocolStats;
}
```

### ProtocolMessage

协议消息结构体。

```rust
pub struct ProtocolMessage {
    pub message_type: MessageType,
    pub data: Vec<u8>,
    pub id: Option<u32>,
    pub timestamp: Option<Instant>,
    pub metadata: HashMap<String, String>,
}
```

#### 方法

##### `new`
```rust
pub fn new(message_type: MessageType, data: Vec<u8>) -> Self
```
创建新消息。

##### `with_id`
```rust
pub fn with_id(mut self, id: u32) -> Self
```
设置消息ID。

##### `with_timestamp`
```rust
pub fn with_timestamp(mut self, timestamp: Instant) -> Self
```
设置时间戳。

##### `add_metadata`
```rust
pub fn add_metadata(mut self, key: String, value: String) -> Self
```
添加元数据。

### MessageType

消息类型枚举。

```rust
pub enum MessageType {
    Command,
    Response,
    Notification,
    Error,
    Data,
    Heartbeat,
}
```

### RawProtocolHandler

原始协议处理器。

```rust
pub struct RawProtocolHandler {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new() -> Self
```
创建新的原始协议处理器。

### RawProtocolBuilder

原始协议构建器。

```rust
pub struct RawProtocolBuilder {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new() -> Self
```
创建构建器。

##### `enable_checksum`
```rust
pub fn enable_checksum(mut self, enable: bool) -> Self
```
启用校验和。

##### `enable_timestamps`
```rust
pub fn enable_timestamps(mut self, enable: bool) -> Self
```
启用时间戳。

##### `batch_size`
```rust
pub fn batch_size(mut self, size: usize) -> Self
```
设置批处理大小。

##### `build`
```rust
pub fn build(self) -> RawProtocolHandler
```
构建处理器。

### ProtocolManager

协议管理器。

```rust
pub struct ProtocolManager {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new() -> Self
```
创建协议管理器。

##### `register_protocol`
```rust
pub fn register_protocol(&mut self, protocol_type: ProtocolType, handler: Box<dyn ProtocolHandler>) -> Result<(), ProtocolError>
```
注册协议处理器。

##### `unregister_protocol`
```rust
pub fn unregister_protocol(&mut self, protocol_type: ProtocolType) -> Result<(), ProtocolError>
```
注销协议处理器。

##### `set_active_protocol`
```rust
pub fn set_active_protocol(&mut self, protocol_type: ProtocolType) -> Result<(), ProtocolError>
```
设置活动协议。

##### `get_active_protocol`
```rust
pub fn get_active_protocol(&self) -> Option<ProtocolType>
```
获取活动协议。

## HAL适配器API

### HalAdapter Trait

硬件抽象层适配器trait。

```rust
pub trait HalAdapter: Send + Sync {
    async fn initialize(&mut self, config: &UartConfig, pins: &UartPins) -> Result<(), HalError>;
    async fn enable(&mut self) -> Result<(), HalError>;
    async fn disable(&mut self) -> Result<(), HalError>;
    async fn reset(&mut self) -> Result<(), HalError>;
    async fn get_status(&self) -> Result<UartStatus, HalError>;
    async fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), HalError>;
    async fn enable_dma(&mut self, enable: bool) -> Result<(), HalError>;
    async fn set_interrupts(&mut self, interrupts: UartInterrupts) -> Result<(), HalError>;
    async fn clear_errors(&mut self) -> Result<(), HalError>;
}
```

### GenericHalAdapter

通用HAL适配器（用于测试）。

```rust
pub struct GenericHalAdapter {
    // 内部字段...
}
```

#### 方法

##### `new`
```rust
pub fn new() -> Self
```
创建通用适配器。

##### `set_buffer_size`
```rust
pub fn set_buffer_size(&mut self, rx_size: usize, tx_size: usize)
```
设置缓冲区大小。

##### `enable_delay_simulation`
```rust
pub fn enable_delay_simulation(&mut self, enable: bool)
```
启用延迟模拟。

##### `enable_error_injection`
```rust
pub fn enable_error_injection(&mut self, enable: bool)
```
启用错误注入。

##### `add_received_data`
```rust
pub fn add_received_data(&mut self, data: &[u8])
```
添加接收数据（用于测试）。

##### `get_transmitted_data`
```rust
pub fn get_transmitted_data(&self) -> Vec<u8>
```
获取发送的数据（用于测试）。

### UartPins

UART引脚配置。

```rust
pub struct UartPins {
    pub tx_pin: u8,
    pub rx_pin: u8,
    pub rts_pin: Option<u8>,
    pub cts_pin: Option<u8>,
}
```

#### 方法

##### `new`
```rust
pub fn new(tx_pin: u8, rx_pin: u8) -> Self
```
创建基本引脚配置。

##### `with_flow_control`
```rust
pub fn with_flow_control(mut self, rts_pin: u8, cts_pin: u8) -> Self
```
添加流控制引脚。

##### `validate`
```rust
pub fn validate(&self) -> Result<(), ConfigError>
```
验证引脚配置。

## 错误类型

### AsyncUartError

主要错误类型。

```rust
pub enum AsyncUartError {
    Timeout,
    BufferFull,
    BufferEmpty,
    HardwareError(HalError),
    ConfigurationError(ConfigError),
    ProtocolError(ProtocolError),
    IoError(std::io::Error),
}
```

### BufferError

缓冲区错误。

```rust
pub enum BufferError {
    Full,
    Empty,
    InvalidSize,
    AlignmentError,
    AllocationFailed,
}
```

### ProtocolError

协议错误。

```rust
pub enum ProtocolError {
    InvalidMessage,
    ChecksumMismatch,
    UnknownProtocol,
    EncodingError,
    DecodingError,
    Timeout,
}
```

### HalError

硬件抽象层错误。

```rust
pub enum HalError {
    InitializationFailed,
    InvalidPin,
    UnsupportedBaudRate,
    DmaError,
    InterruptError,
    HardwareFault,
}
```

### ConfigError

配置错误。

```rust
pub enum ConfigError {
    InvalidBaudRate,
    InvalidDataBits,
    InvalidBufferSize,
    InvalidTimeout,
    UnsupportedFeature,
}
```

## Trait定义

### AsyncRead

异步读取trait。

```rust
pub trait AsyncRead {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, std::io::Error>;
    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), std::io::Error>;
}
```

### AsyncWrite

异步写入trait。

```rust
pub trait AsyncWrite {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, std::io::Error>;
    async fn write_all(&mut self, buf: &[u8]) -> Result<(), std::io::Error>;
    async fn flush(&mut self) -> Result<(), std::io::Error>;
}
```

### AsyncUart Trait

UART特定的异步操作trait。

```rust
pub trait AsyncUart: AsyncRead + AsyncWrite {
    async fn set_baud_rate(&mut self, baud_rate: u32) -> AsyncUartResult<()>;
    async fn set_timeout(&mut self, timeout: Duration) -> AsyncUartResult<()>;
    async fn reset(&mut self) -> AsyncUartResult<()>;
    fn get_stats(&self) -> UartStats;
}
```

## 统计类型

### UartStats

UART统计信息。

```rust
pub struct UartStats {
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub messages_sent: u64,
    pub messages_received: u64,
    pub error_count: u64,
    pub timeout_count: u64,
    pub last_activity: Option<Instant>,
    pub uptime: Duration,
}
```

### ProtocolStats

协议统计信息。

```rust
pub struct ProtocolStats {
    pub messages_processed: u64,
    pub messages_sent: u64,
    pub messages_received: u64,
    pub encoding_errors: u64,
    pub decoding_errors: u64,
    pub timeout_count: u64,
    pub bytes_processed: u64,
    pub average_processing_time: Duration,
    pub last_activity: Option<Instant>,
}
```

这个API参考文档涵盖了AsyncUART库的所有主要公共接口。每个API都包含了详细的参数说明、返回值和使用示例。