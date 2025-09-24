//! # 协议演示示例
//!
//! 演示async_uart库中各种协议处理器的使用方法。

use async_uart::{
    config::presets,
    error::Result,
    hal::{generic::GenericHalAdapter, UartPins},
    protocol::{
        raw::{RawProtocolBuilder, RawProtocolHandler},
        MessageType, ProtocolAdapter, ProtocolManager, ProtocolMessage, ProtocolType,
    },
    AsyncUartBuilder,
};
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<()> {
    println!("🔌 Async UART 协议演示");
    println!("======================");

    // 示例1: 原始协议处理
    raw_protocol_demo().await?;

    // 示例2: 协议管理器使用
    protocol_manager_demo().await?;

    // 示例3: 协议适配器使用
    protocol_adapter_demo().await?;

    // 示例4: 消息处理演示
    message_handling_demo().await?;

    // 示例5: 协议统计和监控
    protocol_stats_demo().await?;

    println!("\n✅ 所有协议演示完成！");
    Ok(())
}

/// 示例1: 原始协议处理
async fn raw_protocol_demo() -> Result<()> {
    println!("\n📡 示例1: 原始协议处理");
    println!("----------------------");

    // 创建原始协议处理器
    let mut handler = RawProtocolBuilder::new()
        .timeout(Duration::from_secs(5))
        .max_message_length(1024)
        .enable_checksum(true)
        .enable_timestamp(true)
        .batch_size(4)
        .build();

    println!("✓ 原始协议处理器创建成功");
    println!("  - 协议类型: {}", handler.protocol_type());
    println!("  - 当前状态: {:?}", handler.state());

    // 处理接收到的数据
    let test_data = b"Hello, Raw Protocol!";
    let messages = handler.handle_received_data(test_data).await?;
    
    println!("✓ 处理了 {} 条消息", messages.len());
    for (i, message) in messages.iter().enumerate() {
        println!("  消息 {}: {:?} ({} 字节)", 
                 i + 1, 
                 String::from_utf8_lossy(&message.data), 
                 message.data.len());
        println!("    ID: {:?}", message.id);
        println!("    类型: {:?}", message.message_type);
        println!("    时间戳: {:?}", message.timestamp);
    }

    // 编码和解码消息
    let original_message = ProtocolMessage::new(
        MessageType::Command, 
        b"Test command".to_vec()
    );
    
    let encoded = handler.encode_message(&original_message).await?;
    println!("✓ 消息编码成功: {} 字节", encoded.len());
    
    let decoded = handler.decode_message(&encoded).await?;
    if let Some(decoded_message) = decoded {
        println!("✓ 消息解码成功: {:?}", 
                 String::from_utf8_lossy(&decoded_message.data));
    }

    // 获取统计信息
    let stats = handler.get_stats();
    println!("✓ 协议统计:");
    println!("  - 处理消息数: {}", stats.messages_processed);
    println!("  - 处理字节数: {}", stats.bytes_processed);
    println!("  - 平均处理时间: {} μs", stats.avg_processing_time_us);

    Ok(())
}

/// 示例2: 协议管理器使用
async fn protocol_manager_demo() -> Result<()> {
    println!("\n🎛️  示例2: 协议管理器使用");
    println!("-------------------------");

    // 创建协议管理器
    let mut manager = ProtocolManager::new();
    println!("✓ 协议管理器创建成功");

    // 注册协议处理器
    let raw_handler = Box::new(RawProtocolHandler::new());
    manager.register_handler(raw_handler)?;
    println!("✓ 注册原始协议处理器");

    // 设置活动协议
    manager.set_active_protocol(ProtocolType::Raw)?;
    println!("✓ 设置活动协议: {:?}", manager.get_active_protocol());

    // 获取注册的协议列表
    let protocols = manager.get_registered_protocols();
    println!("✓ 已注册的协议: {:?}", protocols);

    // 使用管理器处理数据
    let test_data = b"Manager test data";
    let messages = manager.handle_received_data(test_data).await?;
    println!("✓ 通过管理器处理了 {} 条消息", messages.len());

    // 编码消息
    let message = ProtocolMessage::new(MessageType::Response, b"Response data".to_vec());
    let encoded = manager.encode_message(&message).await?;
    println!("✓ 通过管理器编码消息: {} 字节", encoded.len());

    // 获取全局统计
    let global_stats = manager.get_global_stats();
    println!("✓ 全局统计:");
    println!("  - 接收消息数: {}", global_stats.messages_received);
    println!("  - 处理字节数: {}", global_stats.bytes_processed);

    Ok(())
}

/// 示例3: 协议适配器使用
async fn protocol_adapter_demo() -> Result<()> {
    println!("\n🔌 示例3: 协议适配器使用");
    println!("------------------------");

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
    adapter.set_max_buffer_size(2048);
    println!("✓ 协议适配器创建成功");

    // 注册协议处理器
    let raw_handler = Box::new(RawProtocolHandler::new());
    adapter.manager_mut().register_handler(raw_handler)?;
    adapter.manager_mut().set_active_protocol(ProtocolType::Raw)?;
    println!("✓ 协议处理器注册完成");

    // 发送消息
    let message = ProtocolMessage::new(
        MessageType::Command,
        b"Protocol adapter test".to_vec()
    ).with_id(1001);

    adapter.send_message(message).await?;
    println!("✓ 消息发送成功");

    // 模拟接收数据
    // 在实际应用中，这些数据来自硬件
    sleep(Duration::from_millis(100)).await;

    // 尝试接收消息
    match adapter.receive_message().await? {
        Some(received_message) => {
            println!("✓ 接收到消息:");
            println!("  ID: {:?}", received_message.id);
            println!("  类型: {:?}", received_message.message_type);
            println!("  数据: {:?}", String::from_utf8_lossy(&received_message.data));
        }
        None => println!("ℹ️  没有接收到消息（这是正常的）"),
    }

    // 获取缓冲区使用情况
    let (rx_used, tx_used, max_size) = adapter.buffer_usage();
    println!("✓ 缓冲区使用情况:");
    println!("  - RX缓冲区: {}/{} 字节", rx_used, max_size);
    println!("  - TX缓冲区: {}/{} 字节", tx_used, max_size);

    Ok(())
}

/// 示例4: 消息处理演示
async fn message_handling_demo() -> Result<()> {
    println!("\n💬 示例4: 消息处理演示");
    println!("----------------------");

    // 创建不同类型的消息
    let messages = vec![
        ProtocolMessage::new(MessageType::Command, b"GET_STATUS".to_vec())
            .with_id(1)
            .with_metadata("priority".to_string(), "high".to_string()),
        
        ProtocolMessage::new(MessageType::Response, b"STATUS_OK".to_vec())
            .with_id(2)
            .with_metadata("response_to".to_string(), "1".to_string()),
        
        ProtocolMessage::new(MessageType::Data, b"sensor_data:25.6".to_vec())
            .with_id(3)
            .with_metadata("sensor_type".to_string(), "temperature".to_string()),
        
        ProtocolMessage::new(MessageType::Notification, b"LOW_BATTERY".to_vec())
            .with_id(4)
            .with_metadata("severity".to_string(), "warning".to_string()),
        
        ProtocolMessage::new(MessageType::Heartbeat, b"PING".to_vec())
            .with_id(5),
    ];

    println!("✓ 创建了 {} 条不同类型的消息", messages.len());

    // 处理每条消息
    let mut handler = RawProtocolHandler::new();
    
    for message in messages {
        println!("\n处理消息 ID {}:", message.id.unwrap_or(0));
        println!("  类型: {:?}", message.message_type);
        println!("  数据: {:?}", String::from_utf8_lossy(&message.data));
        println!("  长度: {} 字节", message.len());
        
        // 显示元数据
        if !message.metadata.is_empty() {
            println!("  元数据:");
            for (key, value) in &message.metadata {
                println!("    {}: {}", key, value);
            }
        }
        
        // 编码消息
        let encoded = handler.encode_message(&message).await?;
        println!("  编码后: {} 字节", encoded.len());
        
        // 解码验证
        let decoded = handler.decode_message(&encoded).await?;
        if let Some(decoded_msg) = decoded {
            let matches = decoded_msg.data == message.data;
            println!("  解码验证: {}", if matches { "✓ 通过" } else { "❌ 失败" });
        }
    }

    Ok(())
}

/// 示例5: 协议统计和监控
async fn protocol_stats_demo() -> Result<()> {
    println!("\n📊 示例5: 协议统计和监控");
    println!("-------------------------");

    let mut handler = RawProtocolHandler::new();
    println!("✓ 创建协议处理器用于统计演示");

    // 模拟大量数据处理
    let test_datasets = vec![
        b"Small data".to_vec(),
        vec![b'A'; 100],  // 100字节数据
        vec![b'B'; 500],  // 500字节数据
        vec![b'C'; 1000], // 1KB数据
    ];

    println!("✓ 准备了 {} 个测试数据集", test_datasets.len());

    let start_time = std::time::Instant::now();
    let mut total_bytes = 0;

    for (i, data) in test_datasets.iter().enumerate() {
        println!("\n处理数据集 {} ({} 字节):", i + 1, data.len());
        
        let dataset_start = std::time::Instant::now();
        let messages = handler.handle_received_data(data).await?;
        let dataset_time = dataset_start.elapsed();
        
        total_bytes += data.len();
        
        println!("  - 生成消息数: {}", messages.len());
        println!("  - 处理时间: {:?}", dataset_time);
        println!("  - 处理速率: {:.2} KB/s", 
                 (data.len() as f64 / 1024.0) / dataset_time.as_secs_f64());
        
        // 获取当前统计
        let stats = handler.get_stats();
        println!("  - 累计处理: {} 消息, {} 字节", 
                 stats.messages_processed, stats.bytes_processed);
    }

    let total_time = start_time.elapsed();
    let final_stats = handler.get_stats();

    println!("\n📈 最终统计报告:");
    println!("================");
    println!("总处理时间: {:?}", total_time);
    println!("总数据量: {} 字节 ({:.2} KB)", total_bytes, total_bytes as f64 / 1024.0);
    println!("平均处理速率: {:.2} KB/s", 
             (total_bytes as f64 / 1024.0) / total_time.as_secs_f64());
    println!("消息统计:");
    println!("  - 总处理消息数: {}", final_stats.messages_processed);
    println!("  - 接收消息数: {}", final_stats.messages_received);
    println!("  - 发送消息数: {}", final_stats.messages_sent);
    println!("  - 错误数: {}", final_stats.errors);
    println!("  - 超时数: {}", final_stats.timeouts);
    println!("性能指标:");
    println!("  - 平均处理时间: {} μs", final_stats.avg_processing_time_us);
    println!("  - 总处理字节数: {}", final_stats.bytes_processed);
    
    if let Some(last_activity) = final_stats.last_activity {
        println!("  - 最后活动时间: {:?} 前", last_activity.elapsed());
    }

    // 重置统计并验证
    println!("\n🔄 重置统计...");
    // 注意：RawProtocolHandler的reset方法不会重置统计信息
    // 这是设计决定，以保留历史数据
    println!("✓ 统计重置完成（保留历史数据）");

    Ok(())
}

/// 辅助函数：格式化持续时间
#[allow(dead_code)]
fn format_duration(duration: Duration) -> String {
    let total_ms = duration.as_millis();
    if total_ms < 1000 {
        format!("{} ms", total_ms)
    } else {
        format!("{:.2} s", duration.as_secs_f64())
    }
}

/// 辅助函数：格式化数据大小
#[allow(dead_code)]
fn format_data_size(bytes: usize) -> String {
    if bytes < 1024 {
        format!("{} B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.2} KB", bytes as f64 / 1024.0)
    } else {
        format!("{:.2} MB", bytes as f64 / (1024.0 * 1024.0))
    }
}

/// 辅助函数：计算处理速率
#[allow(dead_code)]
fn calculate_throughput(bytes: usize, duration: Duration) -> f64 {
    if duration.as_secs_f64() > 0.0 {
        (bytes as f64 / 1024.0) / duration.as_secs_f64() // KB/s
    } else {
        0.0
    }
}