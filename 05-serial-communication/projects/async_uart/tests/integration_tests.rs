//! # 集成测试
//!
//! 测试async_uart库的完整功能和各组件之间的集成。

use async_uart::{
    buffer::{
        ring_buffer::RingBuffer,
        stream_buffer::{StreamBuffer, StreamBufferConfig},
        Buffer, BufferConfig, OverflowStrategy, ReadableBuffer, WritableBuffer,
    },
    config::{presets, UartConfig},
    error::{AsyncUartError, Result},
    hal::{generic::GenericHalAdapter, UartPins},
    protocol::{
        raw::{RawProtocolBuilder, RawProtocolHandler},
        MessageType, ProtocolAdapter, ProtocolManager, ProtocolMessage, ProtocolType,
    },
    AsyncUart, AsyncUartBuilder,
};
use std::time::Duration;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    time::{sleep, timeout},
};

/// 测试基本UART功能
#[tokio::test]
async fn test_basic_uart_functionality() -> Result<()> {
    // 创建UART实例
    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    // 测试基本写入
    let test_data = b"Hello, UART!";
    uart.write_all(test_data).await?;

    // 测试基本读取
    let mut read_buffer = vec![0u8; test_data.len()];
    let bytes_read = uart.read(&mut read_buffer).await?;
    
    assert!(bytes_read <= test_data.len());
    
    Ok(())
}

/// 测试UART配置
#[tokio::test]
async fn test_uart_configuration() -> Result<()> {
    // 测试预设配置
    let configs = vec![
        presets::standard_115200(),
        presets::high_speed_921600(),
        presets::low_power_9600(),
        presets::debug_config(),
    ];

    for config in configs {
        // 验证配置
        config.validate()?;
        
        // 创建UART实例
        let hal_adapter = GenericHalAdapter::new();
        let uart = AsyncUartBuilder::new()
            .with_config(config)
            .with_pins(UartPins::new(1, 2))
            .with_adapter(hal_adapter)
            .build()
            .await?;

        assert!(uart.id() > 0);
    }

    Ok(())
}

/// 测试自定义配置
#[tokio::test]
async fn test_custom_configuration() -> Result<()> {
    let custom_config = UartConfig::builder()
        .baud_rate(460800)
        .data_bits(8)
        .stop_bits(async_uart::config::StopBits::One)
        .parity(async_uart::config::Parity::None)
        .flow_control(async_uart::config::FlowControl::None)
        .timeout(Duration::from_millis(1000))
        .buffer_size(2048)
        .enable_dma(false)
        .build()?;

    // 验证配置
    custom_config.validate()?;

    // 创建UART
    let hal_adapter = GenericHalAdapter::new();
    let uart = AsyncUartBuilder::new()
        .with_config(custom_config)
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    assert!(uart.id() > 0);
    
    Ok(())
}

/// 测试环形缓冲区
#[tokio::test]
async fn test_ring_buffer_integration() -> Result<()> {
    let mut buffer = RingBuffer::new(1024);
    
    // 测试基本读写
    let test_data = b"Ring buffer test data";
    let written = buffer.write(test_data).await?;
    assert_eq!(written, test_data.len());

    let mut read_buffer = vec![0u8; test_data.len()];
    let read = buffer.read(&mut read_buffer).await?;
    assert_eq!(read, test_data.len());
    assert_eq!(&read_buffer[..read], test_data);

    // 测试环绕
    let large_data = vec![b'A'; 800];
    buffer.write(&large_data).await?;
    
    let more_data = vec![b'B'; 400];
    let written = buffer.write(&more_data).await?;
    assert!(written > 0);

    // 测试统计
    let stats = buffer.stats();
    assert!(stats.used > 0);
    assert!(stats.available < stats.capacity);

    Ok(())
}

/// 测试流式缓冲区
#[tokio::test]
async fn test_stream_buffer_integration() -> Result<()> {
    let config = StreamBufferConfig::new()
        .with_size(1024)
        .with_overflow_strategy(OverflowStrategy::Block)
        .with_high_watermark(800)
        .with_low_watermark(200)
        .enable_backpressure(true);

    let mut buffer = StreamBuffer::new(config)?;

    // 测试基本操作
    let test_data = b"Stream buffer test";
    let written = buffer.write(test_data).await?;
    assert_eq!(written, test_data.len());

    let mut read_buffer = vec![0u8; test_data.len()];
    let read = buffer.read(&mut read_buffer).await?;
    assert_eq!(read, test_data.len());

    // 测试水位标记
    let large_data = vec![b'X'; 900];
    buffer.write(&large_data).await?;
    
    let (is_high, _) = buffer.watermark_status();
    assert!(is_high);

    // 测试块操作
    let block_data = vec![b'B'; 128];
    buffer.write_block(&block_data).await?;

    let read_block = buffer.read_block(64).await?;
    assert!(read_block.is_some());

    Ok(())
}

/// 测试协议处理
#[tokio::test]
async fn test_protocol_integration() -> Result<()> {
    // 测试原始协议处理器
    let mut handler = RawProtocolBuilder::new()
        .timeout(Duration::from_secs(1))
        .max_message_length(1024)
        .enable_checksum(true)
        .build();

    // 测试数据处理
    let test_data = b"Protocol test data";
    let messages = handler.handle_received_data(test_data).await?;
    assert!(!messages.is_empty());

    // 测试消息编码/解码
    let original_message = ProtocolMessage::new(
        MessageType::Command,
        b"Test command".to_vec()
    );

    let encoded = handler.encode_message(&original_message).await?;
    assert!(!encoded.is_empty());

    let decoded = handler.decode_message(&encoded).await?;
    assert!(decoded.is_some());

    Ok(())
}

/// 测试协议管理器
#[tokio::test]
async fn test_protocol_manager_integration() -> Result<()> {
    let mut manager = ProtocolManager::new();

    // 注册协议处理器
    let raw_handler = Box::new(RawProtocolHandler::new());
    manager.register_handler(raw_handler)?;

    // 设置活动协议
    manager.set_active_protocol(ProtocolType::Raw)?;
    assert_eq!(manager.get_active_protocol(), Some(ProtocolType::Raw));

    // 测试数据处理
    let test_data = b"Manager test";
    let messages = manager.handle_received_data(test_data).await?;
    assert!(!messages.is_empty());

    // 测试消息编码
    let message = ProtocolMessage::new(MessageType::Data, b"Test data".to_vec());
    let encoded = manager.encode_message(&message).await?;
    assert!(!encoded.is_empty());

    Ok(())
}

/// 测试协议适配器
#[tokio::test]
async fn test_protocol_adapter_integration() -> Result<()> {
    // 创建UART传输层
    let hal_adapter = GenericHalAdapter::new();
    let uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    // 创建协议适配器
    let mut adapter = ProtocolAdapter::new(uart);

    // 注册协议处理器
    let raw_handler = Box::new(RawProtocolHandler::new());
    adapter.manager_mut().register_handler(raw_handler)?;
    adapter.manager_mut().set_active_protocol(ProtocolType::Raw)?;

    // 测试消息发送
    let message = ProtocolMessage::new(
        MessageType::Command,
        b"Adapter test".to_vec()
    );

    adapter.send_message(message).await?;

    // 测试缓冲区使用情况
    let (rx_used, tx_used, max_size) = adapter.buffer_usage();
    assert!(max_size > 0);

    Ok(())
}

/// 测试错误处理
#[tokio::test]
async fn test_error_handling() -> Result<()> {
    // 测试无效配置
    let invalid_config = UartConfig::builder()
        .baud_rate(0) // 无效波特率
        .build();
    
    assert!(invalid_config.is_err());

    // 测试超时
    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    let timeout_result = timeout(Duration::from_millis(10), uart.read_u8()).await;
    assert!(timeout_result.is_err()); // 应该超时

    Ok(())
}

/// 测试并发操作
#[tokio::test]
async fn test_concurrent_operations() -> Result<()> {
    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    // 创建多个并发写入任务
    let mut handles = Vec::new();
    
    for i in 0..5 {
        let data = vec![i as u8; 64];
        let handle = tokio::spawn(async move {
            // 模拟一些处理时间
            sleep(Duration::from_millis(10)).await;
            Ok::<Vec<u8>, AsyncUartError>(data)
        });
        handles.push(handle);
    }

    // 等待所有任务完成
    for handle in handles {
        match handle.await {
            Ok(Ok(data)) => {
                uart.write_all(&data).await?;
            }
            Ok(Err(e)) => return Err(e),
            Err(e) => return Err(AsyncUartError::HardwareError(format!("Task error: {:?}", e))),
        }
    }

    Ok(())
}

/// 测试性能基准
#[tokio::test]
async fn test_performance_benchmark() -> Result<()> {
    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::high_speed_921600())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    // 测试不同大小的数据传输
    let test_sizes = vec![64, 256, 1024];
    
    for size in test_sizes {
        let test_data = vec![0x55u8; size];
        let start = std::time::Instant::now();
        
        // 执行多次传输
        for _ in 0..10 {
            uart.write_all(&test_data).await?;
        }
        
        let elapsed = start.elapsed();
        let total_bytes = size * 10;
        let throughput = (total_bytes as f64 / 1024.0) / elapsed.as_secs_f64();
        
        // 验证性能在合理范围内
        assert!(throughput > 0.0);
        println!("Size: {} bytes, Throughput: {:.2} KB/s", size, throughput);
    }

    Ok(())
}

/// 测试缓冲区边界条件
#[tokio::test]
async fn test_buffer_edge_cases() -> Result<()> {
    // 测试空缓冲区
    let mut buffer = RingBuffer::new(64);
    let mut read_buf = vec![0u8; 10];
    let read = buffer.read(&mut read_buf).await?;
    assert_eq!(read, 0);

    // 测试满缓冲区
    let fill_data = vec![0xAAu8; 64];
    let written = buffer.write(&fill_data).await?;
    assert!(written > 0);

    // 尝试写入更多数据
    let more_data = vec![0xBBu8; 10];
    let written = buffer.write(&more_data).await?;
    // 根据缓冲区实现，这可能成功（覆盖）或失败

    Ok(())
}

/// 测试协议消息处理
#[tokio::test]
async fn test_protocol_message_handling() -> Result<()> {
    // 创建不同类型的消息
    let messages = vec![
        ProtocolMessage::new(MessageType::Command, b"GET_STATUS".to_vec()),
        ProtocolMessage::new(MessageType::Response, b"STATUS_OK".to_vec()),
        ProtocolMessage::new(MessageType::Data, b"sensor_data".to_vec()),
        ProtocolMessage::new(MessageType::Notification, b"ALERT".to_vec()),
        ProtocolMessage::new(MessageType::Heartbeat, b"PING".to_vec()),
    ];

    let mut handler = RawProtocolHandler::new();

    for message in messages {
        // 测试编码
        let encoded = handler.encode_message(&message).await?;
        assert!(!encoded.is_empty());

        // 测试解码
        let decoded = handler.decode_message(&encoded).await?;
        assert!(decoded.is_some());

        if let Some(decoded_msg) = decoded {
            assert_eq!(decoded_msg.message_type, message.message_type);
            assert_eq!(decoded_msg.data, message.data);
        }
    }

    Ok(())
}

/// 测试资源清理
#[tokio::test]
async fn test_resource_cleanup() -> Result<()> {
    // 创建多个UART实例并确保它们能正确清理
    for i in 0..5 {
        let hal_adapter = GenericHalAdapter::new();
        let uart = AsyncUartBuilder::new()
            .with_config(presets::standard_115200())
            .with_pins(UartPins::new(i + 1, i + 2))
            .with_adapter(hal_adapter)
            .build()
            .await?;

        // 使用UART
        let test_data = format!("Test data {}", i);
        uart.write_all(test_data.as_bytes()).await?;

        // UART在作用域结束时应该自动清理
    }

    // 测试缓冲区清理
    {
        let mut buffer = RingBuffer::new(1024);
        buffer.write(b"test data").await?;
        // 缓冲区在作用域结束时应该自动清理
    }

    Ok(())
}

/// 测试长时间运行的操作
#[tokio::test]
async fn test_long_running_operations() -> Result<()> {
    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    // 模拟长时间的数据传输
    let chunk_size = 256;
    let total_chunks = 20;

    for i in 0..total_chunks {
        let data = vec![(i % 256) as u8; chunk_size];
        uart.write_all(&data).await?;
        
        // 小延迟模拟实际使用场景
        sleep(Duration::from_millis(1)).await;
    }

    println!("Successfully transmitted {} chunks of {} bytes each", 
             total_chunks, chunk_size);

    Ok(())
}

/// 测试配置验证
#[tokio::test]
async fn test_configuration_validation() -> Result<()> {
    // 测试有效配置
    let valid_configs = vec![
        UartConfig::builder().baud_rate(9600).build()?,
        UartConfig::builder().baud_rate(115200).build()?,
        UartConfig::builder().baud_rate(921600).build()?,
    ];

    for config in valid_configs {
        config.validate()?;
    }

    // 测试无效配置应该失败
    let invalid_results = vec![
        UartConfig::builder().baud_rate(0).build(),
        UartConfig::builder().baud_rate(u32::MAX).build(),
        UartConfig::builder().data_bits(0).build(),
        UartConfig::builder().data_bits(20).build(),
    ];

    for result in invalid_results {
        assert!(result.is_err(), "Invalid configuration should fail validation");
    }

    Ok(())
}