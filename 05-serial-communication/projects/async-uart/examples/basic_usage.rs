//! # 基本使用示例
//!
//! 演示async_uart库的基本功能，包括初始化、读写操作和错误处理。

use async_uart::{
    config::{presets, Config},
    error::Result,
    hal::{generic::GenericHalAdapter, HalAdapterFactory, Platform, UartPins},
    traits::{AsyncRead, AsyncWrite, AsyncUart},
    AsyncUartBuilder,
};
use std::time::Duration;
use tokio::time::timeout;

#[tokio::main]
async fn main() -> Result<()> {
    println!("🚀 Async UART 基本使用示例");
    println!("================================");

    // 示例1: 使用预设配置创建UART
    basic_uart_example().await?;

    // 示例2: 自定义配置
    custom_config_example().await?;

    // 示例3: 错误处理
    error_handling_example().await?;

    // 示例4: 超时操作
    timeout_example().await?;

    // 示例5: 批量数据传输
    batch_transfer_example().await?;

    println!("\n✅ 所有示例执行完成！");
    Ok(())
}

/// 示例1: 基本UART操作
async fn basic_uart_example() -> Result<()> {
    println!("\n📡 示例1: 基本UART操作");
    println!("-----------------------");

    // 创建HAL适配器
    let adapter = GenericHalAdapter::new();
    
    // 使用构建器创建UART实例
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(adapter)
        .build()
        .await?;

    println!("✓ UART初始化成功");

    // 发送数据
    let message = b"Hello, Async UART!";
    uart.write_all(message).await?;
    uart.flush().await?;
    println!("✓ 发送数据: {:?}", String::from_utf8_lossy(message));

    // 模拟接收数据（在实际应用中，数据来自硬件）
    if let Some(adapter) = uart.adapter_mut().downcast_mut::<GenericHalAdapter>() {
        adapter.add_rx_data(b"Response from device")?;
    }

    // 读取数据
    let mut buffer = [0u8; 64];
    let bytes_read = uart.read(&mut buffer).await?;
    let received = &buffer[..bytes_read];
    println!("✓ 接收数据: {:?}", String::from_utf8_lossy(received));

    // 获取统计信息
    let stats = uart.get_stats();
    println!("✓ 统计信息: 发送 {} 字节, 接收 {} 字节", 
             stats.bytes_written, stats.bytes_read);

    Ok(())
}

/// 示例2: 自定义配置
async fn custom_config_example() -> Result<()> {
    println!("\n⚙️  示例2: 自定义配置");
    println!("---------------------");

    // 创建自定义配置
    let config = Config::builder()
        .baudrate(9600)
        .data_bits(8)
        .stop_bits(1)
        .parity(async_uart::config::Parity::None)
        .flow_control(async_uart::config::FlowControl::None)
        .rx_buffer_size(2048)
        .tx_buffer_size(2048)
        .timeout(Duration::from_secs(3))
        .build()?;

    println!("✓ 自定义配置创建成功");
    println!("  - 波特率: {}", config.baudrate);
    println!("  - 数据位: {}", config.data_bits);
    println!("  - 停止位: {}", config.stop_bits);
    println!("  - 校验位: {:?}", config.parity);
    println!("  - 流控制: {:?}", config.flow_control);

    // 创建带流控制的引脚配置
    let pins = UartPins::new(3, 4).with_flow_control(5, 6);
    println!("✓ 引脚配置: TX={:?}, RX={:?}, RTS={:?}, CTS={:?}", 
             pins.tx, pins.rx, pins.rts, pins.cts);

    // 创建UART实例
    let adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(config)
        .with_pins(pins)
        .with_adapter(adapter)
        .build()
        .await?;

    println!("✓ 自定义UART创建成功");

    // 测试配置
    let test_data = b"Custom config test";
    uart.write_all(test_data).await?;
    println!("✓ 使用自定义配置发送数据成功");

    Ok(())
}

/// 示例3: 错误处理
async fn error_handling_example() -> Result<()> {
    println!("\n❌ 示例3: 错误处理");
    println!("------------------");

    // 创建一个会产生错误的配置
    let result = Config::builder()
        .baudrate(0) // 无效波特率
        .build();

    match result {
        Ok(_) => println!("⚠️  预期的错误没有发生"),
        Err(e) => println!("✓ 捕获到预期错误: {}", e),
    }

    // 创建正常的UART用于演示运行时错误
    let adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(adapter)
        .build()
        .await?;

    // 尝试读取大量数据（可能超时）
    let mut large_buffer = vec![0u8; 1024];
    match timeout(Duration::from_millis(100), uart.read(&mut large_buffer)).await {
        Ok(Ok(bytes_read)) => println!("✓ 读取了 {} 字节", bytes_read),
        Ok(Err(e)) => println!("✓ 读取错误: {}", e),
        Err(_) => println!("✓ 读取超时（这是预期的）"),
    }

    // 演示错误恢复
    println!("✓ 错误处理和恢复演示完成");

    Ok(())
}

/// 示例4: 超时操作
async fn timeout_example() -> Result<()> {
    println!("\n⏰ 示例4: 超时操作");
    println!("------------------");

    let adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(adapter)
        .build()
        .await?;

    // 带超时的写操作
    let data = b"Timeout test data";
    match timeout(Duration::from_secs(1), uart.write_all(data)).await {
        Ok(Ok(())) => println!("✓ 写操作在超时前完成"),
        Ok(Err(e)) => println!("❌ 写操作失败: {}", e),
        Err(_) => println!("⏰ 写操作超时"),
    }

    // 带超时的读操作
    let mut buffer = [0u8; 32];
    match timeout(Duration::from_millis(500), uart.read(&mut buffer)).await {
        Ok(Ok(bytes_read)) => println!("✓ 读取了 {} 字节", bytes_read),
        Ok(Err(e)) => println!("❌ 读操作失败: {}", e),
        Err(_) => println!("⏰ 读操作超时（这是预期的，因为没有数据）"),
    }

    println!("✓ 超时操作演示完成");

    Ok(())
}

/// 示例5: 批量数据传输
async fn batch_transfer_example() -> Result<()> {
    println!("\n📦 示例5: 批量数据传输");
    println!("------------------------");

    let adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::high_speed_921600())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(adapter)
        .build()
        .await?;

    // 准备大量数据
    let mut large_data = Vec::new();
    for i in 0..1000 {
        large_data.extend_from_slice(format!("Data packet {}\n", i).as_bytes());
    }

    println!("✓ 准备了 {} 字节的测试数据", large_data.len());

    // 分批发送数据
    let chunk_size = 256;
    let mut total_sent = 0;
    let start_time = std::time::Instant::now();

    for (i, chunk) in large_data.chunks(chunk_size).enumerate() {
        uart.write_all(chunk).await?;
        total_sent += chunk.len();
        
        if i % 10 == 0 {
            println!("  已发送 {} 字节 ({:.1}%)", 
                     total_sent, 
                     (total_sent as f64 / large_data.len() as f64) * 100.0);
        }
    }

    uart.flush().await?;
    let elapsed = start_time.elapsed();

    println!("✓ 批量传输完成:");
    println!("  - 总数据量: {} 字节", total_sent);
    println!("  - 传输时间: {:?}", elapsed);
    println!("  - 传输速率: {:.2} KB/s", 
             (total_sent as f64 / 1024.0) / elapsed.as_secs_f64());

    // 模拟接收确认
    if let Some(adapter) = uart.adapter_mut().downcast_mut::<GenericHalAdapter>() {
        adapter.add_rx_data(b"BATCH_TRANSFER_COMPLETE")?;
    }

    let mut ack_buffer = [0u8; 32];
    let ack_bytes = uart.read(&mut ack_buffer).await?;
    let ack_message = &ack_buffer[..ack_bytes];
    println!("✓ 接收到确认: {:?}", String::from_utf8_lossy(ack_message));

    Ok(())
}

/// 辅助函数：打印分隔线
#[allow(dead_code)]
fn print_separator() {
    println!("{}", "=".repeat(50));
}

/// 辅助函数：格式化字节数
#[allow(dead_code)]
fn format_bytes(bytes: usize) -> String {
    if bytes < 1024 {
        format!("{} B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.2} KB", bytes as f64 / 1024.0)
    } else {
        format!("{:.2} MB", bytes as f64 / (1024.0 * 1024.0))
    }
}

/// 辅助函数：格式化传输速率
#[allow(dead_code)]
fn format_transfer_rate(bytes: usize, duration: Duration) -> String {
    let rate = bytes as f64 / duration.as_secs_f64();
    if rate < 1024.0 {
        format!("{:.2} B/s", rate)
    } else if rate < 1024.0 * 1024.0 {
        format!("{:.2} KB/s", rate / 1024.0)
    } else {
        format!("{:.2} MB/s", rate / (1024.0 * 1024.0))
    }
}