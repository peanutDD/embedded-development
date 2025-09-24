# AsyncUART 用户指南

本指南将帮助您深入了解和使用AsyncUART库的各种功能。

## 目录

1. [安装和设置](#安装和设置)
2. [基础概念](#基础概念)
3. [配置管理](#配置管理)
4. [缓冲区使用](#缓冲区使用)
5. [协议处理](#协议处理)
6. [错误处理](#错误处理)
7. [性能优化](#性能优化)
8. [平台特定指南](#平台特定指南)
9. [故障排除](#故障排除)
10. [最佳实践](#最佳实践)

## 安装和设置

### 基本安装

在您的 `Cargo.toml` 文件中添加依赖：

```toml
[dependencies]
async-uart = "0.1.0"
tokio = { version = "1.0", features = ["full"] }

# 根据目标平台选择相应的HAL
# 对于STM32
stm32f4xx-hal = "0.14"

# 对于ESP32
esp-idf-hal = "0.42"

# 对于RP2040
rp-pico = "0.7"
```

### 功能特性配置

```toml
[dependencies]
async-uart = { version = "0.1.0", features = ["std-support", "dma-support"] }
```

可用特性：
- `std-support`: 标准库支持（用于测试和开发）
- `dma-support`: DMA传输支持
- `protocol-support`: 协议处理支持
- `statistics`: 统计和监控功能
- `defmt-logging`: defmt日志支持

### 嵌入式环境设置

对于 `no_std` 环境：

```toml
[dependencies]
async-uart = { version = "0.1.0", default-features = false, features = ["dma-support"] }
```

## 基础概念

### 异步I/O模型

AsyncUART基于Rust的 `async/await` 模型，提供非阻塞的I/O操作：

```rust
use async_uart::{AsyncUartBuilder, config::presets};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .build()
        .await?;

    // 异步写入
    uart.write_all(b"Hello").await?;
    
    // 异步读取
    let mut buffer = [0u8; 32];
    let n = uart.read(&mut buffer).await?;
    
    Ok(())
}
```

### 核心组件

1. **UART实例**: 主要的通信接口
2. **缓冲区**: 数据存储和管理
3. **协议处理器**: 数据格式化和解析
4. **HAL适配器**: 硬件抽象层

## 配置管理

### 使用预设配置

```rust
use async_uart::config::presets;

// 标准串口配置 (115200, 8N1)
let config = presets::standard_115200();

// 高速配置 (921600, 8N1)
let config = presets::high_speed_921600();

// 低功耗配置 (9600, 8N1)
let config = presets::low_power_9600();

// 调试配置 (带详细日志)
let config = presets::debug_config();
```

### 自定义配置

```rust
use async_uart::config::{UartConfig, Parity, StopBits, FlowControl};
use std::time::Duration;

let config = UartConfig::builder()
    .baud_rate(115200)
    .data_bits(8)
    .stop_bits(StopBits::One)
    .parity(Parity::None)
    .flow_control(FlowControl::None)
    .timeout(Duration::from_millis(1000))
    .buffer_size(2048)
    .enable_dma(false)
    .build()?;
```

### 配置验证

```rust
// 验证配置是否有效
match config.validate() {
    Ok(()) => println!("配置有效"),
    Err(e) => println!("配置错误: {:?}", e),
}
```

### 运行时配置修改

```rust
// 修改波特率
uart.set_baud_rate(921600).await?;

// 修改超时
uart.set_timeout(Duration::from_millis(2000)).await?;

// 启用/禁用DMA
uart.enable_dma(true).await?;
```

## 缓冲区使用

### 环形缓冲区

适用于连续数据流：

```rust
use async_uart::buffer::{ring_buffer::RingBuffer, OverflowStrategy};

// 创建基本环形缓冲区
let mut buffer = RingBuffer::new(1024);

// 指定溢出策略
let mut buffer = RingBuffer::with_overflow_strategy(
    1024, 
    OverflowStrategy::Overwrite
);

// 写入数据
let data = b"Hello, World!";
let written = buffer.write(data).await?;

// 读取数据
let mut read_buf = [0u8; 64];
let read = buffer.read(&mut read_buf).await?;
```

### DMA缓冲区

适用于高性能传输：

```rust
use async_uart::buffer::dma_buffer::{DmaBuffer, DmaBufferConfig};

let config = DmaBufferConfig::new()
    .with_size(2048)
    .with_alignment(64)  // 64字节对齐
    .enable_stats(true)
    .enable_cache_coherency(true);

let mut buffer = DmaBuffer::new(config)?;

// DMA传输
let data = vec![0x55u8; 1024];
let written = buffer.write(&data).await?;

// 获取统计信息
let stats = buffer.get_stats();
println!("DMA传输: {} 次, 总字节: {}", stats.transfer_count, stats.total_bytes);
```

### 流式缓冲区

支持背压控制：

```rust
use async_uart::buffer::stream_buffer::{StreamBuffer, StreamBufferConfig};

let config = StreamBufferConfig::new()
    .with_size(4096)
    .with_high_watermark(3072)  // 75%
    .with_low_watermark(1024)   // 25%
    .enable_backpressure(true)
    .with_block_size(256);

let mut buffer = StreamBuffer::new(config)?;

// 检查水位状态
if buffer.is_high_watermark_reached() {
    println!("缓冲区接近满载，启用背压");
}

// 块操作
let block_data = vec![0xAAu8; 256];
buffer.write_block(&block_data).await?;

let read_block = buffer.read_block(128).await?;
```

### 缓冲区选择指南

| 缓冲区类型 | 适用场景 | 优点 | 缺点 |
|-----------|----------|------|------|
| 环形缓冲区 | 连续数据流 | 简单高效 | 无硬件加速 |
| DMA缓冲区 | 高性能传输 | 硬件加速 | 内存对齐要求 |
| 流式缓冲区 | 流控制应用 | 背压支持 | 复杂度较高 |

## 协议处理

### 原始协议

直接数据传输：

```rust
use async_uart::protocol::raw::{RawProtocolHandler, RawProtocolBuilder};

let mut handler = RawProtocolBuilder::new()
    .enable_checksum(true)
    .enable_timestamps(true)
    .batch_size(10)
    .build();

// 处理接收数据
let received_data = b"Hello, Protocol!";
let messages = handler.handle_received_data(received_data).await?;

for message in messages {
    println!("收到消息: {:?}", message);
}
```

### 协议管理器

管理多种协议：

```rust
use async_uart::protocol::{ProtocolManager, ProtocolType};

let mut manager = ProtocolManager::new();

// 注册协议
manager.register_protocol(ProtocolType::Raw, Box::new(raw_handler))?;
manager.register_protocol(ProtocolType::LineBased, Box::new(line_handler))?;

// 设置活动协议
manager.set_active_protocol(ProtocolType::Raw)?;

// 处理数据
let data = b"Protocol data";
let messages = manager.handle_data(data).await?;
```

### 协议适配器

将协议与传输层结合：

```rust
use async_uart::protocol::{ProtocolAdapter, ProtocolMessage, MessageType};

let adapter = ProtocolAdapter::new(uart, manager);

// 发送消息
let message = ProtocolMessage::new(MessageType::Command, b"GET_STATUS".to_vec());
adapter.send_message(&message).await?;

// 接收消息
let received = adapter.receive_message().await?;
println!("收到响应: {:?}", received);
```

## 错误处理

### 错误类型

```rust
use async_uart::{AsyncUartError, AsyncUartResult};

match uart.write_all(&data).await {
    Ok(bytes_written) => {
        println!("成功写入 {} 字节", bytes_written);
    }
    Err(AsyncUartError::Timeout) => {
        // 处理超时
        println!("操作超时，尝试重新连接");
    }
    Err(AsyncUartError::BufferFull) => {
        // 处理缓冲区满
        println!("缓冲区已满，等待空间释放");
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    Err(AsyncUartError::HardwareError(hw_err)) => {
        // 处理硬件错误
        println!("硬件错误: {:?}", hw_err);
        uart.reset().await?;
    }
    Err(AsyncUartError::ConfigurationError(config_err)) => {
        // 处理配置错误
        println!("配置错误: {:?}", config_err);
    }
    Err(e) => {
        println!("未知错误: {:?}", e);
    }
}
```

### 错误恢复策略

```rust
async fn robust_write(uart: &mut AsyncUart, data: &[u8]) -> AsyncUartResult<usize> {
    let max_retries = 3;
    let mut retries = 0;
    
    loop {
        match uart.write_all(data).await {
            Ok(written) => return Ok(written),
            Err(AsyncUartError::Timeout) if retries < max_retries => {
                retries += 1;
                println!("写入超时，重试 {}/{}", retries, max_retries);
                tokio::time::sleep(Duration::from_millis(100 * retries)).await;
            }
            Err(AsyncUartError::HardwareError(_)) if retries < max_retries => {
                retries += 1;
                println!("硬件错误，尝试重置");
                uart.reset().await?;
                tokio::time::sleep(Duration::from_millis(500)).await;
            }
            Err(e) => return Err(e),
        }
    }
}
```

## 性能优化

### 缓冲区大小优化

```rust
// 根据数据特征选择缓冲区大小
let config = UartConfig::builder()
    .buffer_size(match data_pattern {
        DataPattern::SmallFrequent => 512,    // 小数据频繁传输
        DataPattern::LargeBurst => 8192,      // 大数据突发传输
        DataPattern::Streaming => 4096,       // 连续流数据
    })
    .build()?;
```

### DMA优化

```rust
// 启用DMA以减少CPU负载
let config = UartConfig::builder()
    .enable_dma(true)
    .buffer_size(4096)  // DMA缓冲区应该较大
    .build()?;

// 使用DMA缓冲区
let dma_config = DmaBufferConfig::new()
    .with_size(4096)
    .with_alignment(64)  // 确保正确对齐
    .enable_cache_coherency(true);
```

### 批量操作

```rust
// 批量写入比单字节写入更高效
let data = vec![0x55u8; 1024];
uart.write_all(&data).await?;  // 好

// 避免
for byte in data {
    uart.write_u8(byte).await?;  // 低效
}
```

### 零拷贝操作

```rust
// 使用切片避免数据复制
let buffer = [0u8; 1024];
let slice = &buffer[..512];
uart.write_all(slice).await?;

// 使用引用计数共享数据
use std::sync::Arc;
let shared_data = Arc::new(vec![0u8; 1024]);
uart.write_all(&shared_data).await?;
```

## 平台特定指南

### STM32平台

```rust
use async_uart::hal::stm32::Stm32HalAdapter;

// STM32F4系列
let adapter = Stm32HalAdapter::new()
    .with_usart(USART1)
    .with_dma_channel(DMA2_STREAM7)
    .with_clock_config(ClockConfig::default());

let uart = AsyncUartBuilder::new()
    .with_adapter(adapter)
    .build()
    .await?;
```

### ESP32平台

```rust
use async_uart::hal::esp32::Esp32HalAdapter;

let adapter = Esp32HalAdapter::new()
    .with_uart_num(0)
    .with_tx_pin(1)
    .with_rx_pin(3)
    .with_rts_pin(Some(22))
    .with_cts_pin(Some(19));

let uart = AsyncUartBuilder::new()
    .with_adapter(adapter)
    .build()
    .await?;
```

### RP2040平台

```rust
use async_uart::hal::rp2040::Rp2040HalAdapter;

let adapter = Rp2040HalAdapter::new()
    .with_uart(uart0)
    .with_pins(pins.gpio0.into_mode(), pins.gpio1.into_mode());

let uart = AsyncUartBuilder::new()
    .with_adapter(adapter)
    .build()
    .await?;
```

## 故障排除

### 常见问题

#### 1. 数据丢失

**症状**: 接收到的数据不完整或有缺失

**可能原因**:
- 缓冲区太小
- 波特率不匹配
- 硬件流控制问题

**解决方案**:
```rust
// 增加缓冲区大小
let config = UartConfig::builder()
    .buffer_size(4096)  // 增加到4KB
    .build()?;

// 启用硬件流控制
let config = UartConfig::builder()
    .flow_control(FlowControl::RtsCts)
    .build()?;
```

#### 2. 传输超时

**症状**: 操作经常超时

**可能原因**:
- 超时设置太短
- 硬件响应慢
- 网络延迟

**解决方案**:
```rust
// 增加超时时间
let config = UartConfig::builder()
    .timeout(Duration::from_millis(5000))  // 5秒超时
    .build()?;

// 使用重试机制
async fn write_with_retry(uart: &mut AsyncUart, data: &[u8]) -> AsyncUartResult<usize> {
    for attempt in 1..=3 {
        match uart.write_all(data).await {
            Ok(written) => return Ok(written),
            Err(AsyncUartError::Timeout) if attempt < 3 => {
                println!("重试 {}/3", attempt);
                continue;
            }
            Err(e) => return Err(e),
        }
    }
    unreachable!()
}
```

#### 3. 内存使用过高

**症状**: 程序内存占用持续增长

**可能原因**:
- 缓冲区泄漏
- 未正确释放资源
- 缓冲区配置过大

**解决方案**:
```rust
// 使用合适的缓冲区大小
let config = UartConfig::builder()
    .buffer_size(1024)  // 根据实际需求调整
    .build()?;

// 定期清理缓冲区
uart.flush().await?;

// 正确关闭资源
uart.close().await?;
```

### 调试技巧

#### 启用详细日志

```rust
use async_uart::config::presets;

// 使用调试配置
let config = presets::debug_config();

// 或者手动启用日志
let config = UartConfig::builder()
    .enable_logging(true)
    .log_level(LogLevel::Debug)
    .build()?;
```

#### 统计信息监控

```rust
// 获取UART统计信息
let stats = uart.get_stats();
println!("发送字节: {}", stats.bytes_sent);
println!("接收字节: {}", stats.bytes_received);
println!("错误次数: {}", stats.error_count);

// 获取缓冲区统计信息
let buffer_stats = buffer.get_stats();
println!("缓冲区使用率: {:.1}%", buffer_stats.utilization * 100.0);
```

## 最佳实践

### 1. 配置管理

```rust
// 使用预设配置作为起点
let mut config = presets::standard_115200();

// 根据需要调整
config.buffer_size = 2048;
config.timeout = Duration::from_millis(2000);
```

### 2. 错误处理

```rust
// 总是处理错误
match uart.write_all(&data).await {
    Ok(_) => { /* 成功处理 */ }
    Err(e) => {
        log::error!("UART写入失败: {:?}", e);
        // 适当的错误恢复
    }
}

// 使用Result类型传播错误
async fn send_command(uart: &mut AsyncUart, cmd: &[u8]) -> AsyncUartResult<Vec<u8>> {
    uart.write_all(cmd).await?;
    
    let mut response = vec![0u8; 256];
    let n = uart.read(&mut response).await?;
    response.truncate(n);
    
    Ok(response)
}
```

### 3. 资源管理

```rust
// 使用RAII模式管理资源
{
    let mut uart = AsyncUartBuilder::new()
        .with_config(config)
        .build()
        .await?;
    
    // 使用uart...
    
    // uart在作用域结束时自动清理
}

// 或者显式关闭
uart.close().await?;
```

### 4. 性能优化

```rust
// 批量操作
let chunks: Vec<&[u8]> = data.chunks(1024).collect();
for chunk in chunks {
    uart.write_all(chunk).await?;
}

// 使用适当的缓冲区大小
let optimal_size = match use_case {
    UseCase::RealTime => 512,
    UseCase::Throughput => 8192,
    UseCase::LowMemory => 256,
};
```

### 5. 并发处理

```rust
// 读写分离
let (mut reader, mut writer) = uart.split();

let read_task = tokio::spawn(async move {
    let mut buffer = [0u8; 1024];
    loop {
        match reader.read(&mut buffer).await {
            Ok(n) => {
                // 处理接收数据
                process_received_data(&buffer[..n]);
            }
            Err(e) => {
                log::error!("读取错误: {:?}", e);
                break;
            }
        }
    }
});

let write_task = tokio::spawn(async move {
    // 发送数据
    writer.write_all(b"Hello").await?;
    Ok::<(), AsyncUartError>(())
});

// 等待任务完成
let (read_result, write_result) = tokio::join!(read_task, write_task);
```

这个用户指南涵盖了AsyncUART库的主要使用方法和最佳实践。根据您的具体需求，可以参考相应的章节进行配置和使用。