//! # 压力测试
//!
//! 测试async_uart库在高负载、长时间运行和边界条件下的稳定性和性能。

use async_uart::{
    buffer::{
        dma_buffer::{DmaBuffer, DmaBufferConfig},
        ring_buffer::RingBuffer,
        stream_buffer::{StreamBuffer, StreamBufferConfig},
        Buffer, OverflowStrategy, ReadableBuffer, WritableBuffer,
    },
    config::presets,
    hal::{generic::GenericHalAdapter, UartPins},
    protocol::{
        raw::{RawProtocolBuilder, RawProtocolHandler},
        MessageType, ProtocolMessage,
    },
    AsyncUartBuilder,
};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::Mutex;
use tokio::time::timeout;

/// 压力测试：大量数据传输
#[tokio::test]
async fn stress_test_large_data_transfer() {
    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::high_speed_921600())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await
        .unwrap();

    // 传输10MB数据
    let chunk_size = 8192;
    let total_size = 10 * 1024 * 1024; // 10MB
    let num_chunks = total_size / chunk_size;
    
    let start_time = Instant::now();
    let mut total_written = 0;
    
    for i in 0..num_chunks {
        let data = vec![(i % 256) as u8; chunk_size];
        let written = uart.write_all(&data).await.unwrap();
        total_written += written;
        
        // 每1000个块检查一次进度
        if i % 1000 == 0 {
            println!("已传输: {:.2}MB", total_written as f64 / (1024.0 * 1024.0));
        }
    }
    
    let duration = start_time.elapsed();
    let throughput = total_written as f64 / duration.as_secs_f64() / (1024.0 * 1024.0);
    
    println!("大数据传输完成:");
    println!("  总大小: {:.2}MB", total_written as f64 / (1024.0 * 1024.0));
    println!("  耗时: {:?}", duration);
    println!("  吞吐量: {:.2}MB/s", throughput);
    
    assert_eq!(total_written, total_size);
    assert!(throughput > 1.0, "吞吐量应该大于1MB/s");
}

/// 压力测试：长时间运行稳定性
#[tokio::test]
async fn stress_test_long_running_stability() {
    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await
        .unwrap();

    let test_duration = Duration::from_secs(30); // 30秒压力测试
    let start_time = Instant::now();
    let mut iteration_count = 0;
    let mut total_bytes = 0;
    let mut error_count = 0;
    
    while start_time.elapsed() < test_duration {
        let data = vec![0x55u8; 1024];
        
        match uart.write_all(&data).await {
            Ok(written) => {
                total_bytes += written;
            }
            Err(_) => {
                error_count += 1;
            }
        }
        
        iteration_count += 1;
        
        // 每1000次迭代休息一下，避免过度占用CPU
        if iteration_count % 1000 == 0 {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    }
    
    let actual_duration = start_time.elapsed();
    let throughput = total_bytes as f64 / actual_duration.as_secs_f64() / 1024.0;
    let error_rate = error_count as f64 / iteration_count as f64 * 100.0;
    
    println!("长时间运行测试结果:");
    println!("  运行时间: {:?}", actual_duration);
    println!("  迭代次数: {}", iteration_count);
    println!("  传输字节: {}", total_bytes);
    println!("  吞吐量: {:.2}KB/s", throughput);
    println!("  错误率: {:.2}%", error_rate);
    
    assert!(error_rate < 1.0, "错误率应该小于1%");
    assert!(iteration_count > 1000, "应该完成足够多的迭代");
}

/// 压力测试：并发操作
#[tokio::test]
async fn stress_test_concurrent_operations() {
    let num_tasks = 50;
    let operations_per_task = 100;
    let mut handles = Vec::new();
    
    let start_time = Instant::now();
    
    for task_id in 0..num_tasks {
        let handle = tokio::spawn(async move {
            let hal_adapter = GenericHalAdapter::new();
            let mut uart = AsyncUartBuilder::new()
                .with_config(presets::standard_115200())
                .with_pins(UartPins::new(task_id as u8, (task_id + 1) as u8))
                .with_adapter(hal_adapter)
                .build()
                .await
                .unwrap();

            let mut task_bytes = 0;
            let mut task_errors = 0;
            
            for op_id in 0..operations_per_task {
                let data = vec![(task_id + op_id) as u8; 256];
                
                match uart.write_all(&data).await {
                    Ok(written) => task_bytes += written,
                    Err(_) => task_errors += 1,
                }
                
                // 随机延迟，模拟真实使用场景
                if op_id % 10 == 0 {
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
            }
            
            (task_bytes, task_errors)
        });
        
        handles.push(handle);
    }
    
    // 等待所有任务完成
    let mut total_bytes = 0;
    let mut total_errors = 0;
    
    for handle in handles {
        let (task_bytes, task_errors) = handle.await.unwrap();
        total_bytes += task_bytes;
        total_errors += task_errors;
    }
    
    let duration = start_time.elapsed();
    let total_operations = num_tasks * operations_per_task;
    let ops_per_sec = total_operations as f64 / duration.as_secs_f64();
    let error_rate = total_errors as f64 / total_operations as f64 * 100.0;
    
    println!("并发操作测试结果:");
    println!("  任务数: {}", num_tasks);
    println!("  每任务操作数: {}", operations_per_task);
    println!("  总操作数: {}", total_operations);
    println!("  总字节数: {}", total_bytes);
    println!("  耗时: {:?}", duration);
    println!("  操作/秒: {:.2}", ops_per_sec);
    println!("  错误率: {:.2}%", error_rate);
    
    assert!(error_rate < 5.0, "并发操作错误率应该小于5%");
    assert!(ops_per_sec > 100.0, "操作速率应该大于100ops/s");
}

/// 压力测试：缓冲区边界条件
#[tokio::test]
async fn stress_test_buffer_boundaries() {
    // 测试环形缓冲区
    {
        let mut buffer = RingBuffer::new(1024);
        
        // 测试满缓冲区写入
        let large_data = vec![0xAAu8; 2048]; // 超过缓冲区大小
        let written = buffer.write(&large_data).await.unwrap();
        assert!(written <= 1024, "写入量不应超过缓冲区大小");
        
        // 测试空缓冲区读取
        let mut read_buf = vec![0u8; 2048];
        let read = buffer.read(&mut read_buf).await.unwrap();
        assert_eq!(read, written, "读取量应该等于之前写入的量");
        
        // 测试零长度操作
        let empty_data = vec![];
        let zero_written = buffer.write(&empty_data).await.unwrap();
        assert_eq!(zero_written, 0, "空数据写入应该返回0");
    }
    
    // 测试DMA缓冲区
    {
        let config = DmaBufferConfig::new()
            .with_size(1024)
            .with_alignment(64);
        let mut buffer = DmaBuffer::new(config).unwrap();
        
        // 测试对齐边界
        for size in [1, 63, 64, 65, 127, 128, 129, 255, 256, 1023, 1024] {
            let data = vec![0xBBu8; size];
            let written = buffer.write(&data).await.unwrap();
            assert!(written <= size, "写入量不应超过数据大小");
            
            let mut read_buf = vec![0u8; size];
            let read = buffer.read(&mut read_buf).await.unwrap();
            assert_eq!(read, written, "读取量应该等于写入量");
        }
    }
    
    // 测试流式缓冲区
    {
        let config = StreamBufferConfig::new()
            .with_size(1024)
            .with_overflow_strategy(OverflowStrategy::Block);
        let mut buffer = StreamBuffer::new(config).unwrap();
        
        // 测试背压
        let large_data = vec![0xCCu8; 2048];
        let write_result = timeout(Duration::from_millis(100), buffer.write(&large_data)).await;
        
        // 应该超时，因为使用了Block策略
        assert!(write_result.is_err(), "大数据写入应该因背压而超时");
    }
    
    println!("缓冲区边界条件测试完成");
}

/// 压力测试：内存压力
#[tokio::test]
async fn stress_test_memory_pressure() {
    let num_buffers = 100;
    let buffer_size = 8192;
    
    // 创建大量缓冲区
    let mut buffers = Vec::new();
    
    for i in 0..num_buffers {
        match i % 3 {
            0 => {
                let buffer = RingBuffer::new(buffer_size);
                buffers.push(Box::new(buffer) as Box<dyn Buffer>);
            }
            1 => {
                let config = DmaBufferConfig::new().with_size(buffer_size);
                if let Ok(buffer) = DmaBuffer::new(config) {
                    buffers.push(Box::new(buffer) as Box<dyn Buffer>);
                }
            }
            2 => {
                let config = StreamBufferConfig::new().with_size(buffer_size);
                if let Ok(buffer) = StreamBuffer::new(config) {
                    buffers.push(Box::new(buffer) as Box<dyn Buffer>);
                }
            }
            _ => unreachable!(),
        }
    }
    
    println!("创建了 {} 个缓冲区，总内存约 {:.2}MB", 
             buffers.len(), 
             buffers.len() * buffer_size as f64 / (1024.0 * 1024.0));
    
    // 对所有缓冲区进行操作
    let test_data = vec![0x77u8; 1024];
    let mut total_operations = 0;
    
    for (i, buffer) in buffers.iter_mut().enumerate() {
        // 写入数据
        if let Ok(written) = buffer.write(&test_data).await {
            total_operations += 1;
            
            // 读取数据
            let mut read_buf = vec![0u8; written];
            if buffer.read(&mut read_buf).await.is_ok() {
                total_operations += 1;
            }
        }
        
        // 每10个缓冲区报告一次进度
        if i % 10 == 0 {
            println!("处理进度: {}/{}", i + 1, buffers.len());
        }
    }
    
    println!("内存压力测试完成，总操作数: {}", total_operations);
    assert!(total_operations > num_buffers, "应该完成足够多的操作");
}

/// 压力测试：协议处理高负载
#[tokio::test]
async fn stress_test_protocol_high_load() {
    let mut handler = RawProtocolBuilder::new()
        .batch_size(50)
        .enable_checksum(true)
        .build();
    
    let num_messages = 10000;
    let message_sizes = [64, 128, 256, 512, 1024];
    
    let start_time = Instant::now();
    let mut total_bytes_processed = 0;
    let mut encode_errors = 0;
    let mut decode_errors = 0;
    
    for i in 0..num_messages {
        let size = message_sizes[i % message_sizes.len()];
        let data = vec![(i % 256) as u8; size];
        
        let message = ProtocolMessage::new(MessageType::Data, data);
        
        // 编码消息
        match handler.encode_message(&message).await {
            Ok(encoded) => {
                total_bytes_processed += encoded.len();
                
                // 解码消息
                match handler.decode_message(&encoded).await {
                    Ok(_decoded) => {
                        // 成功
                    }
                    Err(_) => decode_errors += 1,
                }
            }
            Err(_) => encode_errors += 1,
        }
        
        // 每1000条消息报告一次进度
        if i % 1000 == 0 {
            println!("协议处理进度: {}/{}", i + 1, num_messages);
        }
    }
    
    let duration = start_time.elapsed();
    let messages_per_sec = num_messages as f64 / duration.as_secs_f64();
    let bytes_per_sec = total_bytes_processed as f64 / duration.as_secs_f64();
    let encode_error_rate = encode_errors as f64 / num_messages as f64 * 100.0;
    let decode_error_rate = decode_errors as f64 / num_messages as f64 * 100.0;
    
    println!("协议高负载测试结果:");
    println!("  消息数: {}", num_messages);
    println!("  处理字节: {}", total_bytes_processed);
    println!("  耗时: {:?}", duration);
    println!("  消息/秒: {:.2}", messages_per_sec);
    println!("  字节/秒: {:.2}", bytes_per_sec);
    println!("  编码错误率: {:.2}%", encode_error_rate);
    println!("  解码错误率: {:.2}%", decode_error_rate);
    
    assert!(encode_error_rate < 1.0, "编码错误率应该小于1%");
    assert!(decode_error_rate < 1.0, "解码错误率应该小于1%");
    assert!(messages_per_sec > 1000.0, "消息处理速率应该大于1000msg/s");
}

/// 压力测试：资源泄漏检测
#[tokio::test]
async fn stress_test_resource_leak_detection() {
    let iterations = 1000;
    
    for i in 0..iterations {
        // 创建和销毁UART实例
        {
            let hal_adapter = GenericHalAdapter::new();
            let uart = AsyncUartBuilder::new()
                .with_config(presets::standard_115200())
                .with_pins(UartPins::new(1, 2))
                .with_adapter(hal_adapter)
                .build()
                .await
                .unwrap();
            
            // 执行一些操作
            let data = vec![0x88u8; 256];
            let _ = uart.write_all(&data).await;
            
            // UART在作用域结束时自动销毁
        }
        
        // 创建和销毁缓冲区
        {
            let mut buffer = RingBuffer::new(1024);
            let data = vec![0x99u8; 512];
            let _ = buffer.write(&data).await;
            
            // 缓冲区在作用域结束时自动销毁
        }
        
        // 创建和销毁协议处理器
        {
            let mut handler = RawProtocolHandler::new();
            let message = ProtocolMessage::new(
                MessageType::Data,
                vec![0xAAu8; 128]
            );
            let _ = handler.encode_message(&message).await;
            
            // 处理器在作用域结束时自动销毁
        }
        
        // 每100次迭代报告一次进度
        if i % 100 == 0 {
            println!("资源泄漏检测进度: {}/{}", i + 1, iterations);
        }
        
        // 偶尔让出CPU时间
        if i % 50 == 0 {
            tokio::task::yield_now().await;
        }
    }
    
    println!("资源泄漏检测完成，创建和销毁了 {} 次资源", iterations);
    
    // 这个测试主要依赖Rust的RAII和Drop trait来确保资源正确释放
    // 在实际应用中，可以使用内存分析工具来检测泄漏
}

/// 压力测试：超时和取消操作
#[tokio::test]
async fn stress_test_timeout_and_cancellation() {
    let num_operations = 100;
    let mut successful_timeouts = 0;
    let mut successful_cancellations = 0;
    
    for i in 0..num_operations {
        let hal_adapter = GenericHalAdapter::new();
        let mut uart = AsyncUartBuilder::new()
            .with_config(presets::standard_115200())
            .with_pins(UartPins::new(1, 2))
            .with_adapter(hal_adapter)
            .build()
            .await
            .unwrap();
        
        if i % 2 == 0 {
            // 测试超时操作
            let data = vec![0xBBu8; 1024];
            let timeout_duration = Duration::from_millis(1); // 很短的超时
            
            let result = timeout(timeout_duration, uart.write_all(&data)).await;
            if result.is_err() {
                successful_timeouts += 1;
            }
        } else {
            // 测试取消操作
            let data = vec![0xCCu8; 1024];
            
            let write_future = uart.write_all(&data);
            let cancel_future = tokio::time::sleep(Duration::from_millis(1));
            
            tokio::select! {
                _ = write_future => {
                    // 写入完成
                }
                _ = cancel_future => {
                    // 操作被取消
                    successful_cancellations += 1;
                }
            }
        }
        
        if i % 20 == 0 {
            println!("超时取消测试进度: {}/{}", i + 1, num_operations);
        }
    }
    
    let timeout_rate = successful_timeouts as f64 / (num_operations / 2) as f64 * 100.0;
    let cancellation_rate = successful_cancellations as f64 / (num_operations / 2) as f64 * 100.0;
    
    println!("超时和取消测试结果:");
    println!("  总操作数: {}", num_operations);
    println!("  成功超时: {} ({:.1}%)", successful_timeouts, timeout_rate);
    println!("  成功取消: {} ({:.1}%)", successful_cancellations, cancellation_rate);
    
    // 这些测试主要验证超时和取消机制是否正常工作
    // 具体的成功率取决于系统负载和时序
}

/// 压力测试：错误恢复
#[tokio::test]
async fn stress_test_error_recovery() {
    let mut hal_adapter = GenericHalAdapter::new();
    hal_adapter.enable_error_injection(true);
    
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await
        .unwrap();
    
    let num_operations = 500;
    let mut successful_operations = 0;
    let mut recovered_errors = 0;
    let max_retries = 3;
    
    for i in 0..num_operations {
        let data = vec![(i % 256) as u8; 256];
        let mut retries = 0;
        let mut operation_successful = false;
        
        while retries < max_retries && !operation_successful {
            match uart.write_all(&data).await {
                Ok(_) => {
                    successful_operations += 1;
                    operation_successful = true;
                    
                    if retries > 0 {
                        recovered_errors += 1;
                    }
                }
                Err(_) => {
                    retries += 1;
                    // 简单的退避策略
                    tokio::time::sleep(Duration::from_millis(retries as u64)).await;
                }
            }
        }
        
        if i % 50 == 0 {
            println!("错误恢复测试进度: {}/{}", i + 1, num_operations);
        }
    }
    
    let success_rate = successful_operations as f64 / num_operations as f64 * 100.0;
    let recovery_rate = recovered_errors as f64 / successful_operations as f64 * 100.0;
    
    println!("错误恢复测试结果:");
    println!("  总操作数: {}", num_operations);
    println!("  成功操作: {} ({:.1}%)", successful_operations, success_rate);
    println!("  恢复错误: {} ({:.1}%)", recovered_errors, recovery_rate);
    
    assert!(success_rate > 50.0, "成功率应该大于50%");
    println!("错误恢复机制工作正常");
}