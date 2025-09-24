//! # 缓冲区演示示例
//!
//! 演示async_uart库中各种缓冲区的使用方法和性能特性。

use async_uart::{
    buffer::{
        dma_buffer::{DmaBuffer, DmaBufferConfig, DmaDirection, DmaMode},
        ring_buffer::RingBuffer,
        stream_buffer::{StreamBuffer, StreamBufferConfig},
        Buffer, BufferConfig, BufferFactory, BufferType, OverflowStrategy,
        ReadableBuffer, WritableBuffer,
    },
    error::Result,
};
use futures_util::{SinkExt, StreamExt};
use std::time::{Duration, Instant};
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<()> {
    println!("🗂️  Async UART 缓冲区演示");
    println!("========================");

    // 示例1: 环形缓冲区演示
    ring_buffer_demo().await?;

    // 示例2: DMA缓冲区演示
    dma_buffer_demo().await?;

    // 示例3: 流式缓冲区演示
    stream_buffer_demo().await?;

    // 示例4: 缓冲区工厂演示
    buffer_factory_demo().await?;

    // 示例5: 性能基准测试
    performance_benchmark().await?;

    // 示例6: 缓冲区比较
    buffer_comparison().await?;

    println!("\n✅ 所有缓冲区演示完成！");
    Ok(())
}

/// 示例1: 环形缓冲区演示
async fn ring_buffer_demo() -> Result<()> {
    println!("\n🔄 示例1: 环形缓冲区演示");
    println!("------------------------");

    // 创建环形缓冲区
    let mut ring_buffer = RingBuffer::new(1024);
    println!("✓ 创建了 {} 字节的环形缓冲区", ring_buffer.capacity());

    // 基本读写操作
    let test_data = b"Hello, Ring Buffer!";
    let written = ring_buffer.write(test_data).await?;
    println!("✓ 写入 {} 字节数据", written);

    let mut read_buffer = vec![0u8; test_data.len()];
    let read = ring_buffer.read(&mut read_buffer).await?;
    println!("✓ 读取 {} 字节数据: {:?}", read, String::from_utf8_lossy(&read_buffer[..read]));

    // 演示环绕特性
    println!("\n🔄 演示环绕特性:");
    let large_data = vec![b'A'; 800];
    ring_buffer.write(&large_data).await?;
    println!("✓ 写入 {} 字节数据", large_data.len());

    let more_data = vec![b'B'; 400]; // 这会导致环绕
    let written = ring_buffer.write(&more_data).await?;
    println!("✓ 写入 {} 字节数据（环绕）", written);

    // 检查缓冲区状态
    let stats = ring_buffer.stats();
    println!("✓ 缓冲区统计:");
    println!("  - 容量: {} 字节", stats.capacity);
    println!("  - 已用: {} 字节", stats.used);
    println!("  - 可用: {} 字节", stats.available);
    println!("  - 使用率: {:.1}%", stats.usage_percentage);

    // 演示窥视功能
    let mut peek_buffer = vec![0u8; 10];
    let peeked = ring_buffer.peek(&mut peek_buffer).await?;
    println!("✓ 窥视 {} 字节: {:?}", peeked, String::from_utf8_lossy(&peek_buffer[..peeked]));

    // 演示跳过功能
    ring_buffer.skip(5).await?;
    println!("✓ 跳过 5 字节");

    // 分割为读写器
    let (mut reader, mut writer) = ring_buffer.split();
    println!("✓ 分割为独立的读写器");

    // 使用分离的读写器
    let write_data = b"Split test";
    writer.write(write_data).await?;
    println!("✓ 通过写器写入: {:?}", String::from_utf8_lossy(write_data));

    let mut read_buf = vec![0u8; write_data.len()];
    let read_len = reader.read(&mut read_buf).await?;
    println!("✓ 通过读器读取: {:?}", String::from_utf8_lossy(&read_buf[..read_len]));

    Ok(())
}

/// 示例2: DMA缓冲区演示
async fn dma_buffer_demo() -> Result<()> {
    println!("\n⚡ 示例2: DMA缓冲区演示");
    println!("----------------------");

    // 创建DMA缓冲区配置
    let config = DmaBufferConfig::new()
        .with_size(2048)
        .with_alignment(64)
        .enable_stats(true);

    // 创建DMA缓冲区
    let mut dma_buffer = DmaBuffer::new(config)?;
    println!("✓ 创建了 {} 字节的DMA缓冲区", dma_buffer.capacity());
    println!("  - 对齐: {} 字节", dma_buffer.alignment());
    println!("  - 状态: {:?}", dma_buffer.state());

    // 准备DMA传输
    dma_buffer.prepare_transfer(DmaDirection::MemoryToPeripheral, DmaMode::Normal)?;
    println!("✓ 准备DMA传输 (内存到外设)");

    // 写入数据
    let test_data = b"DMA Buffer Test Data - High Performance Transfer";
    let written = dma_buffer.write(test_data).await?;
    println!("✓ 写入 {} 字节到DMA缓冲区", written);

    // 开始DMA传输
    dma_buffer.start_transfer()?;
    println!("✓ 开始DMA传输");

    // 模拟传输过程
    sleep(Duration::from_millis(10)).await;

    // 完成传输
    dma_buffer.complete_transfer(written)?;
    println!("✓ DMA传输完成");

    // 演示缓存操作
    dma_buffer.invalidate_cache()?;
    println!("✓ 缓存无效化完成");

    dma_buffer.flush_cache()?;
    println!("✓ 缓存刷新完成");

    // 获取DMA统计
    let stats = dma_buffer.stats();
    println!("✓ DMA缓冲区统计:");
    println!("  - 总传输次数: {}", stats.total_transfers);
    println!("  - 传输字节数: {}", stats.bytes_transferred);
    println!("  - 错误次数: {}", stats.errors);

    // 演示双缓冲模式
    println!("\n🔄 演示双缓冲模式:");
    dma_buffer.prepare_transfer(DmaDirection::PeripheralToMemory, DmaMode::Circular)?;
    println!("✓ 配置为循环模式");

    // 读取数据
    let mut read_buffer = vec![0u8; test_data.len()];
    let read = dma_buffer.read(&mut read_buffer).await?;
    println!("✓ 从DMA缓冲区读取 {} 字节: {:?}", 
             read, String::from_utf8_lossy(&read_buffer[..read]));

    // 重置DMA缓冲区
    dma_buffer.reset().await?;
    println!("✓ DMA缓冲区重置完成");

    Ok(())
}

/// 示例3: 流式缓冲区演示
async fn stream_buffer_demo() -> Result<()> {
    println!("\n🌊 示例3: 流式缓冲区演示");
    println!("------------------------");

    // 创建流式缓冲区配置
    let config = StreamBufferConfig::new()
        .with_size(1024)
        .with_overflow_strategy(OverflowStrategy::Overwrite)
        .with_high_watermark(800)
        .with_low_watermark(200)
        .enable_backpressure(true)
        .enable_stats(true)
        .with_block_size(64);

    // 创建流式缓冲区
    let mut stream_buffer = StreamBuffer::new(config)?;
    println!("✓ 创建了流式缓冲区");
    println!("  - 容量: {} 字节", stream_buffer.capacity());
    println!("  - 溢出策略: {:?}", stream_buffer.config().overflow_strategy);
    println!("  - 高水位: {} 字节", stream_buffer.config().high_watermark);
    println!("  - 低水位: {} 字节", stream_buffer.config().low_watermark);

    // 基本流操作
    let test_data = b"Stream Buffer Test - Continuous Data Flow";
    let written = stream_buffer.write(test_data).await?;
    println!("✓ 写入 {} 字节到流缓冲区", written);

    // 检查水位状态
    let (is_high, is_low) = stream_buffer.watermark_status();
    println!("✓ 水位状态: 高水位={}, 低水位={}", is_high, is_low);

    // 演示背压控制
    println!("\n🚦 演示背压控制:");
    let large_data = vec![b'X'; 900]; // 超过高水位
    let written = stream_buffer.write(&large_data).await?;
    println!("✓ 写入大量数据: {} 字节", written);

    let (is_high, _) = stream_buffer.watermark_status();
    if is_high {
        println!("⚠️  达到高水位，触发背压");
        stream_buffer.pause().await?;
        println!("✓ 流已暂停");
        
        // 读取一些数据以降低水位
        let mut read_buffer = vec![0u8; 400];
        let read = stream_buffer.read(&mut read_buffer).await?;
        println!("✓ 读取 {} 字节以降低水位", read);
        
        stream_buffer.resume().await?;
        println!("✓ 流已恢复");
    }

    // 演示块操作
    println!("\n📦 演示块操作:");
    let block_data = vec![b'B'; 128];
    stream_buffer.write_block(&block_data).await?;
    println!("✓ 写入数据块: {} 字节", block_data.len());

    let read_block = stream_buffer.read_block(64).await?;
    if let Some(block) = read_block {
        println!("✓ 读取数据块: {} 字节", block.len());
    }

    // 演示流接口 (Stream trait)
    println!("\n🔄 演示Stream trait:");
    let mut stream_clone = stream_buffer.clone();
    
    // 作为Stream使用
    if let Some(item) = stream_clone.next().await {
        match item {
            Ok(data) => println!("✓ 从Stream读取: {} 字节", data.len()),
            Err(e) => println!("❌ Stream错误: {:?}", e),
        }
    }

    // 作为Sink使用
    let sink_data = vec![b'S'; 32];
    stream_buffer.send(sink_data.clone()).await?;
    println!("✓ 通过Sink发送: {} 字节", sink_data.len());

    // 刷新缓冲区
    stream_buffer.flush().await?;
    println!("✓ 缓冲区刷新完成");

    // 获取统计信息
    let stats = stream_buffer.stats();
    println!("✓ 流缓冲区统计:");
    println!("  - 容量: {} 字节", stats.capacity);
    println!("  - 已用: {} 字节", stats.used);
    println!("  - 使用率: {:.1}%", stats.usage_percentage);

    // 关闭流
    stream_buffer.close().await?;
    println!("✓ 流缓冲区已关闭");

    Ok(())
}

/// 示例4: 缓冲区工厂演示
async fn buffer_factory_demo() -> Result<()> {
    println!("\n🏭 示例4: 缓冲区工厂演示");
    println!("-------------------------");

    // 创建缓冲区工厂
    let factory = BufferFactory::new();
    println!("✓ 创建缓冲区工厂");

    // 获取支持的缓冲区类型
    let supported_types = factory.supported_types();
    println!("✓ 支持的缓冲区类型: {:?}", supported_types);

    // 创建不同类型的缓冲区
    let configs = vec![
        (BufferType::Ring, BufferConfig::new().with_size(512)),
        (BufferType::Dma, BufferConfig::new().with_size(1024).with_alignment(32)),
        (BufferType::Stream, BufferConfig::new().with_size(2048)
            .with_overflow_strategy(OverflowStrategy::Block)),
    ];

    for (buffer_type, config) in configs {
        println!("\n创建 {:?} 缓冲区:", buffer_type);
        
        match factory.create_buffer(buffer_type, config) {
            Ok(buffer) => {
                println!("✓ 成功创建缓冲区");
                println!("  - 类型: {:?}", buffer.buffer_type());
                println!("  - 容量: {} 字节", buffer.capacity());
                println!("  - 可用: {} 字节", buffer.available());
                
                // 测试基本操作
                let test_data = format!("Test data for {:?} buffer", buffer_type);
                let test_bytes = test_data.as_bytes();
                
                // 这里我们需要将Box<dyn Buffer>转换为具体类型才能调用write
                // 在实际应用中，你会知道具体的缓冲区类型
                println!("  - 测试数据长度: {} 字节", test_bytes.len());
            }
            Err(e) => {
                println!("❌ 创建失败: {:?}", e);
            }
        }
    }

    // 演示缓冲区信息
    println!("\n📋 缓冲区信息演示:");
    let ring_config = BufferConfig::new().with_size(1024);
    if let Ok(ring_buffer) = factory.create_buffer(BufferType::Ring, ring_config) {
        let info = ring_buffer.info();
        println!("✓ 环形缓冲区信息:");
        println!("  - ID: {}", info.id);
        println!("  - 类型: {:?}", info.buffer_type);
        println!("  - 容量: {} 字节", info.capacity);
        println!("  - 创建时间: {:?}", info.created_at);
        println!("  - 特性: {:?}", info.features);
    }

    Ok(())
}

/// 示例5: 性能基准测试
async fn performance_benchmark() -> Result<()> {
    println!("\n🏃 示例5: 性能基准测试");
    println!("----------------------");

    let test_sizes = vec![64, 256, 1024, 4096];
    let iterations = 1000;

    for size in test_sizes {
        println!("\n测试数据大小: {} 字节", size);
        println!("迭代次数: {}", iterations);
        
        // 准备测试数据
        let test_data = vec![b'T'; size];
        
        // 测试环形缓冲区
        let ring_time = benchmark_ring_buffer(&test_data, iterations).await?;
        println!("  环形缓冲区: {:.2} ms ({:.2} MB/s)", 
                 ring_time.as_millis(),
                 calculate_throughput(size * iterations, ring_time));
        
        // 测试DMA缓冲区
        let dma_time = benchmark_dma_buffer(&test_data, iterations).await?;
        println!("  DMA缓冲区:  {:.2} ms ({:.2} MB/s)", 
                 dma_time.as_millis(),
                 calculate_throughput(size * iterations, dma_time));
        
        // 测试流式缓冲区
        let stream_time = benchmark_stream_buffer(&test_data, iterations).await?;
        println!("  流式缓冲区: {:.2} ms ({:.2} MB/s)", 
                 stream_time.as_millis(),
                 calculate_throughput(size * iterations, stream_time));
    }

    Ok(())
}

/// 示例6: 缓冲区比较
async fn buffer_comparison() -> Result<()> {
    println!("\n⚖️  示例6: 缓冲区比较");
    println!("--------------------");

    // 创建相同大小的不同缓冲区
    let buffer_size = 1024;
    let mut ring_buffer = RingBuffer::new(buffer_size);
    let mut dma_buffer = DmaBuffer::new(
        DmaBufferConfig::new().with_size(buffer_size).enable_stats(true)
    )?;
    let mut stream_buffer = StreamBuffer::new(
        StreamBufferConfig::new().with_size(buffer_size).enable_stats(true)
    )?;

    println!("✓ 创建了三种类型的缓冲区，每个 {} 字节", buffer_size);

    // 比较特性
    println!("\n📊 特性比较:");
    println!("| 特性         | 环形缓冲区 | DMA缓冲区 | 流式缓冲区 |");
    println!("| ------------ | ---------- | --------- | ---------- |");
    println!("| 容量         | {:>10} | {:>9} | {:>10} |", 
             ring_buffer.capacity(), dma_buffer.capacity(), stream_buffer.capacity());
    println!("| 可用空间     | {:>10} | {:>9} | {:>10} |", 
             ring_buffer.available(), dma_buffer.available(), stream_buffer.available());
    println!("| 已用空间     | {:>10} | {:>9} | {:>10} |", 
             ring_buffer.used(), dma_buffer.used(), stream_buffer.used());

    // 写入相同数据并比较
    let test_data = b"Comparison test data for all buffer types";
    
    let ring_written = ring_buffer.write(test_data).await?;
    let dma_written = dma_buffer.write(test_data).await?;
    let stream_written = stream_buffer.write(test_data).await?;
    
    println!("\n✓ 写入测试数据 ({} 字节):", test_data.len());
    println!("  环形缓冲区写入: {} 字节", ring_written);
    println!("  DMA缓冲区写入:  {} 字节", dma_written);
    println!("  流式缓冲区写入: {} 字节", stream_written);

    // 比较读取性能
    let mut read_buffer = vec![0u8; test_data.len()];
    
    let ring_start = Instant::now();
    let ring_read = ring_buffer.read(&mut read_buffer).await?;
    let ring_read_time = ring_start.elapsed();
    
    let dma_start = Instant::now();
    let dma_read = dma_buffer.read(&mut read_buffer).await?;
    let dma_read_time = dma_start.elapsed();
    
    let stream_start = Instant::now();
    let stream_read = stream_buffer.read(&mut read_buffer).await?;
    let stream_read_time = stream_start.elapsed();
    
    println!("\n⏱️  读取性能比较:");
    println!("  环形缓冲区: {} 字节, {:?}", ring_read, ring_read_time);
    println!("  DMA缓冲区:  {} 字节, {:?}", dma_read, dma_read_time);
    println!("  流式缓冲区: {} 字节, {:?}", stream_read, stream_read_time);

    // 内存使用比较
    println!("\n💾 内存使用比较:");
    println!("  环形缓冲区: ~{} 字节 (数据 + 少量元数据)", buffer_size);
    println!("  DMA缓冲区:  ~{} 字节 (对齐的数据 + DMA描述符)", buffer_size + 64);
    println!("  流式缓冲区: ~{} 字节 (数据 + 流控制结构)", buffer_size + 128);

    // 使用场景建议
    println!("\n💡 使用场景建议:");
    println!("  环形缓冲区: 适用于简单的FIFO操作，低延迟要求");
    println!("  DMA缓冲区:  适用于高性能数据传输，硬件DMA支持");
    println!("  流式缓冲区: 适用于复杂的流处理，需要背压控制");

    Ok(())
}

// 基准测试辅助函数

async fn benchmark_ring_buffer(data: &[u8], iterations: usize) -> Result<Duration> {
    let mut buffer = RingBuffer::new(data.len() * 2);
    let mut read_buf = vec![0u8; data.len()];
    
    let start = Instant::now();
    for _ in 0..iterations {
        buffer.write(data).await?;
        buffer.read(&mut read_buf).await?;
    }
    Ok(start.elapsed())
}

async fn benchmark_dma_buffer(data: &[u8], iterations: usize) -> Result<Duration> {
    let config = DmaBufferConfig::new().with_size(data.len() * 2);
    let mut buffer = DmaBuffer::new(config)?;
    let mut read_buf = vec![0u8; data.len()];
    
    let start = Instant::now();
    for _ in 0..iterations {
        buffer.write(data).await?;
        buffer.read(&mut read_buf).await?;
    }
    Ok(start.elapsed())
}

async fn benchmark_stream_buffer(data: &[u8], iterations: usize) -> Result<Duration> {
    let config = StreamBufferConfig::new().with_size(data.len() * 2);
    let mut buffer = StreamBuffer::new(config)?;
    let mut read_buf = vec![0u8; data.len()];
    
    let start = Instant::now();
    for _ in 0..iterations {
        buffer.write(data).await?;
        buffer.read(&mut read_buf).await?;
    }
    Ok(start.elapsed())
}

fn calculate_throughput(bytes: usize, duration: Duration) -> f64 {
    if duration.as_secs_f64() > 0.0 {
        (bytes as f64 / (1024.0 * 1024.0)) / duration.as_secs_f64() // MB/s
    } else {
        0.0
    }
}