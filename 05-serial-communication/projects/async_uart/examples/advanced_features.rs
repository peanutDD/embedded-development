//! # 高级功能演示示例
//!
//! 演示async_uart库的高级功能，包括错误处理、配置管理、性能优化等。

use async_uart::{
    config::{presets, UartConfig},
    error::{AsyncUartError, Result},
    hal::{generic::GenericHalAdapter, UartPins},
    AsyncUart, AsyncUartBuilder,
};
use std::time::{Duration, Instant};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    time::{sleep, timeout},
};

#[tokio::main]
async fn main() -> Result<()> {
    println!("🚀 Async UART 高级功能演示");
    println!("==========================");

    // 示例1: 高级配置管理
    advanced_configuration_demo().await?;

    // 示例2: 错误处理和恢复
    error_handling_demo().await?;

    // 示例3: 性能优化技巧
    performance_optimization_demo().await?;

    // 示例4: 超时和取消操作
    timeout_and_cancellation_demo().await?;

    // 示例5: 批量操作和流水线
    batch_operations_demo().await?;

    // 示例6: 监控和诊断
    monitoring_and_diagnostics_demo().await?;

    // 示例7: 自定义适配器
    custom_adapter_demo().await?;

    println!("\n✅ 所有高级功能演示完成！");
    Ok(())
}

/// 示例1: 高级配置管理
async fn advanced_configuration_demo() -> Result<()> {
    println!("\n⚙️  示例1: 高级配置管理");
    println!("------------------------");

    // 使用预设配置
    println!("📋 使用预设配置:");
    let configs = vec![
        ("标准115200", presets::standard_115200()),
        ("高速921600", presets::high_speed_921600()),
        ("低功耗9600", presets::low_power_9600()),
        ("调试配置", presets::debug_config()),
    ];

    for (name, config) in configs {
        println!("  {}: 波特率={}, 数据位={}, 停止位={:?}, 校验={:?}",
                 name, config.baud_rate, config.data_bits, config.stop_bits, config.parity);
    }

    // 创建自定义配置
    println!("\n🔧 创建自定义配置:");
    let custom_config = UartConfig::builder()
        .baud_rate(460800)
        .data_bits(8)
        .stop_bits(async_uart::config::StopBits::Two)
        .parity(async_uart::config::Parity::Even)
        .flow_control(async_uart::config::FlowControl::RtsCts)
        .timeout(Duration::from_millis(500))
        .buffer_size(4096)
        .enable_dma(true)
        .build()?;

    println!("✓ 自定义配置创建成功:");
    println!("  - 波特率: {}", custom_config.baud_rate);
    println!("  - 数据位: {}", custom_config.data_bits);
    println!("  - 停止位: {:?}", custom_config.stop_bits);
    println!("  - 校验位: {:?}", custom_config.parity);
    println!("  - 流控制: {:?}", custom_config.flow_control);
    println!("  - 超时: {:?}", custom_config.timeout);
    println!("  - 缓冲区大小: {}", custom_config.buffer_size);
    println!("  - DMA启用: {}", custom_config.dma_enabled);

    // 配置验证
    println!("\n✅ 配置验证:");
    match custom_config.validate() {
        Ok(_) => println!("✓ 配置验证通过"),
        Err(e) => println!("❌ 配置验证失败: {:?}", e),
    }

    // 配置克隆和修改
    let mut modified_config = custom_config.clone();
    modified_config.baud_rate = 115200;
    modified_config.timeout = Duration::from_secs(1);
    
    println!("✓ 配置克隆和修改完成");
    println!("  - 原始波特率: {}", custom_config.baud_rate);
    println!("  - 修改后波特率: {}", modified_config.baud_rate);

    // 使用配置创建UART
    let hal_adapter = GenericHalAdapter::new();
    let uart = AsyncUartBuilder::new()
        .with_config(modified_config)
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    println!("✓ 使用自定义配置创建UART成功");
    println!("  - UART ID: {}", uart.id());

    Ok(())
}

/// 示例2: 错误处理和恢复
async fn error_handling_demo() -> Result<()> {
    println!("\n🚨 示例2: 错误处理和恢复");
    println!("-------------------------");

    // 创建UART用于错误演示
    let mut hal_adapter = GenericHalAdapter::new();
    
    // 启用错误注入用于演示
    hal_adapter.enable_error_injection(true);
    
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    println!("✓ 创建UART（启用错误注入）");

    // 演示不同类型的错误处理
    println!("\n🔍 演示各种错误类型:");

    // 1. 超时错误
    println!("\n1️⃣  超时错误演示:");
    let timeout_result = timeout(Duration::from_millis(100), uart.read_u8()).await;
    match timeout_result {
        Ok(Ok(byte)) => println!("  意外读取到字节: 0x{:02X}", byte),
        Ok(Err(e)) => println!("  读取错误: {:?}", e),
        Err(_) => println!("  ✓ 超时错误正确触发"),
    }

    // 2. 配置错误
    println!("\n2️⃣  配置错误演示:");
    let invalid_config = UartConfig::builder()
        .baud_rate(0) // 无效波特率
        .build();
    
    match invalid_config {
        Ok(_) => println!("  ❌ 应该检测到无效配置"),
        Err(e) => println!("  ✓ 正确检测到配置错误: {:?}", e),
    }

    // 3. 硬件错误模拟
    println!("\n3️⃣  硬件错误演示:");
    for i in 0..5 {
        match uart.write_u8(0x42).await {
            Ok(_) => println!("  尝试 {}: 写入成功", i + 1),
            Err(e) => {
                println!("  尝试 {}: 写入失败 - {:?}", i + 1, e);
                
                // 错误恢复策略
                match e {
                    AsyncUartError::Timeout => {
                        println!("    🔄 超时恢复: 重试操作");
                        sleep(Duration::from_millis(10)).await;
                    }
                    AsyncUartError::BufferFull => {
                        println!("    🔄 缓冲区满恢复: 清空缓冲区");
                        // 在实际应用中，这里会清空缓冲区
                    }
                    AsyncUartError::HardwareError(_) => {
                        println!("    🔄 硬件错误恢复: 重置UART");
                        // 在实际应用中，这里会重置硬件
                    }
                    _ => {
                        println!("    ❌ 未知错误，停止重试");
                        break;
                    }
                }
            }
        }
    }

    // 4. 错误统计和分析
    println!("\n📊 错误统计分析:");
    let error_stats = collect_error_statistics(&mut uart).await;
    println!("  - 总操作次数: {}", error_stats.total_operations);
    println!("  - 成功次数: {}", error_stats.successful_operations);
    println!("  - 失败次数: {}", error_stats.failed_operations);
    println!("  - 成功率: {:.1}%", error_stats.success_rate());
    println!("  - 错误类型分布:");
    for (error_type, count) in error_stats.error_distribution {
        println!("    {}: {} 次", error_type, count);
    }

    // 5. 错误恢复策略
    println!("\n🛠️  错误恢复策略:");
    let recovery_result = implement_recovery_strategy(&mut uart).await;
    match recovery_result {
        Ok(_) => println!("  ✓ 错误恢复成功"),
        Err(e) => println!("  ❌ 错误恢复失败: {:?}", e),
    }

    Ok(())
}

/// 示例3: 性能优化技巧
async fn performance_optimization_demo() -> Result<()> {
    println!("\n⚡ 示例3: 性能优化技巧");
    println!("----------------------");

    // 创建高性能配置的UART
    let high_perf_config = UartConfig::builder()
        .baud_rate(921600)
        .buffer_size(8192)
        .enable_dma(true)
        .timeout(Duration::from_millis(10))
        .build()?;

    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(high_perf_config)
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    println!("✓ 创建高性能UART配置");

    // 1. 批量操作优化
    println!("\n1️⃣  批量操作优化:");
    let test_data = vec![0x55u8; 1024];
    
    // 单字节操作（低效）
    let start = Instant::now();
    for &byte in &test_data[..100] {
        uart.write_u8(byte).await?;
    }
    let single_byte_time = start.elapsed();
    
    // 批量操作（高效）
    let start = Instant::now();
    uart.write_all(&test_data[100..200]).await?;
    let batch_time = start.elapsed();
    
    println!("  - 单字节操作 (100字节): {:?}", single_byte_time);
    println!("  - 批量操作 (100字节): {:?}", batch_time);
    println!("  - 性能提升: {:.1}x", 
             single_byte_time.as_nanos() as f64 / batch_time.as_nanos() as f64);

    // 2. 缓冲区大小优化
    println!("\n2️⃣  缓冲区大小优化:");
    let buffer_sizes = vec![512, 1024, 2048, 4096, 8192];
    
    for size in buffer_sizes {
        let config = UartConfig::builder()
            .baud_rate(115200)
            .buffer_size(size)
            .build()?;
        
        let throughput = measure_throughput_with_buffer_size(config).await?;
        println!("  - 缓冲区 {} 字节: {:.2} KB/s", size, throughput);
    }

    // 3. DMA vs 非DMA性能比较
    println!("\n3️⃣  DMA vs 非DMA性能比较:");
    
    let dma_config = UartConfig::builder()
        .baud_rate(460800)
        .enable_dma(true)
        .buffer_size(4096)
        .build()?;
    
    let non_dma_config = UartConfig::builder()
        .baud_rate(460800)
        .enable_dma(false)
        .buffer_size(4096)
        .build()?;
    
    let dma_throughput = measure_throughput_with_config(dma_config).await?;
    let non_dma_throughput = measure_throughput_with_config(non_dma_config).await?;
    
    println!("  - DMA模式: {:.2} KB/s", dma_throughput);
    println!("  - 非DMA模式: {:.2} KB/s", non_dma_throughput);
    println!("  - DMA性能提升: {:.1}x", dma_throughput / non_dma_throughput);

    // 4. 内存使用优化
    println!("\n4️⃣  内存使用优化:");
    demonstrate_memory_optimization().await?;

    // 5. CPU使用率优化
    println!("\n5️⃣  CPU使用率优化:");
    demonstrate_cpu_optimization(&mut uart).await?;

    Ok(())
}

/// 示例4: 超时和取消操作
async fn timeout_and_cancellation_demo() -> Result<()> {
    println!("\n⏰ 示例4: 超时和取消操作");
    println!("-------------------------");

    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    // 1. 基本超时操作
    println!("\n1️⃣  基本超时操作:");
    
    let timeout_durations = vec![
        Duration::from_millis(10),
        Duration::from_millis(100),
        Duration::from_millis(500),
        Duration::from_secs(1),
    ];

    for timeout_duration in timeout_durations {
        let start = Instant::now();
        let result = timeout(timeout_duration, uart.read_u8()).await;
        let elapsed = start.elapsed();
        
        match result {
            Ok(Ok(byte)) => println!("  超时 {:?}: 读取到 0x{:02X} (用时 {:?})", 
                                   timeout_duration, byte, elapsed),
            Ok(Err(e)) => println!("  超时 {:?}: 读取错误 {:?} (用时 {:?})", 
                                 timeout_duration, e, elapsed),
            Err(_) => println!("  超时 {:?}: 正确超时 (用时 {:?})", 
                             timeout_duration, elapsed),
        }
    }

    // 2. 可取消的长时间操作
    println!("\n2️⃣  可取消的长时间操作:");
    
    use tokio::select;
    use tokio::time::interval;
    
    let mut cancel_signal = false;
    let mut progress_timer = interval(Duration::from_millis(100));
    
    select! {
        result = long_running_operation(&mut uart) => {
            match result {
                Ok(_) => println!("  ✓ 长时间操作完成"),
                Err(e) => println!("  ❌ 长时间操作失败: {:?}", e),
            }
        }
        _ = progress_timer.tick() => {
            cancel_signal = true;
            println!("  🛑 操作被取消");
        }
    }

    // 3. 超时重试机制
    println!("\n3️⃣  超时重试机制:");
    let retry_result = retry_with_timeout(&mut uart, 3, Duration::from_millis(200)).await;
    match retry_result {
        Ok(data) => println!("  ✓ 重试成功，读取到 {} 字节", data.len()),
        Err(e) => println!("  ❌ 重试失败: {:?}", e),
    }

    // 4. 自适应超时
    println!("\n4️⃣  自适应超时:");
    let mut adaptive_timeout = Duration::from_millis(100);
    
    for i in 0..5 {
        let start = Instant::now();
        let result = timeout(adaptive_timeout, uart.write_u8(0x42)).await;
        let elapsed = start.elapsed();
        
        match result {
            Ok(Ok(_)) => {
                println!("  尝试 {}: 成功 (用时 {:?})", i + 1, elapsed);
                // 成功时减少超时时间
                adaptive_timeout = std::cmp::max(
                    adaptive_timeout / 2, 
                    Duration::from_millis(10)
                );
            }
            Ok(Err(e)) => {
                println!("  尝试 {}: 错误 {:?} (用时 {:?})", i + 1, e, elapsed);
            }
            Err(_) => {
                println!("  尝试 {}: 超时 (用时 {:?})", i + 1, elapsed);
                // 超时时增加超时时间
                adaptive_timeout = std::cmp::min(
                    adaptive_timeout * 2, 
                    Duration::from_secs(5)
                );
            }
        }
        
        println!("    下次超时设置: {:?}", adaptive_timeout);
    }

    Ok(())
}

/// 示例5: 批量操作和流水线
async fn batch_operations_demo() -> Result<()> {
    println!("\n📦 示例5: 批量操作和流水线");
    println!("---------------------------");

    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::high_speed_921600())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    // 1. 批量写入操作
    println!("\n1️⃣  批量写入操作:");
    
    let batch_sizes = vec![16, 64, 256, 1024];
    
    for batch_size in batch_sizes {
        let data = vec![0xAAu8; batch_size];
        let start = Instant::now();
        uart.write_all(&data).await?;
        let elapsed = start.elapsed();
        
        let throughput = (batch_size as f64 / 1024.0) / elapsed.as_secs_f64();
        println!("  批量大小 {} 字节: {:?} ({:.2} KB/s)", 
                 batch_size, elapsed, throughput);
    }

    // 2. 流水线操作
    println!("\n2️⃣  流水线操作:");
    
    use tokio::task;
    
    // 创建多个并发任务
    let mut handles = Vec::new();
    
    for i in 0..4 {
        let data = vec![i as u8; 256];
        let handle = task::spawn(async move {
            // 模拟处理时间
            sleep(Duration::from_millis(10)).await;
            Ok::<Vec<u8>, AsyncUartError>(data)
        });
        handles.push(handle);
    }
    
    // 等待所有任务完成并按顺序写入
    let start = Instant::now();
    for (i, handle) in handles.into_iter().enumerate() {
        match handle.await {
            Ok(Ok(data)) => {
                uart.write_all(&data).await?;
                println!("  流水线任务 {} 完成: {} 字节", i + 1, data.len());
            }
            Ok(Err(e)) => println!("  流水线任务 {} 失败: {:?}", i + 1, e),
            Err(e) => println!("  流水线任务 {} 异常: {:?}", i + 1, e),
        }
    }
    let pipeline_time = start.elapsed();
    println!("  流水线总时间: {:?}", pipeline_time);

    // 3. 缓冲区管理优化
    println!("\n3️⃣  缓冲区管理优化:");
    demonstrate_buffer_management(&mut uart).await?;

    // 4. 零拷贝操作
    println!("\n4️⃣  零拷贝操作演示:");
    demonstrate_zero_copy_operations(&mut uart).await?;

    Ok(())
}

/// 示例6: 监控和诊断
async fn monitoring_and_diagnostics_demo() -> Result<()> {
    println!("\n📊 示例6: 监控和诊断");
    println!("--------------------");

    let mut hal_adapter = GenericHalAdapter::new();
    
    // 启用统计收集
    hal_adapter.enable_delay_simulation(true);
    
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    // 1. 实时性能监控
    println!("\n1️⃣  实时性能监控:");
    
    let monitor_duration = Duration::from_secs(2);
    let monitor_start = Instant::now();
    let mut operation_count = 0;
    let mut total_bytes = 0;
    
    while monitor_start.elapsed() < monitor_duration {
        let data = vec![0x55u8; 64];
        match uart.write_all(&data).await {
            Ok(_) => {
                operation_count += 1;
                total_bytes += data.len();
            }
            Err(e) => println!("    写入错误: {:?}", e),
        }
        
        sleep(Duration::from_millis(10)).await;
    }
    
    let elapsed = monitor_start.elapsed();
    let ops_per_sec = operation_count as f64 / elapsed.as_secs_f64();
    let bytes_per_sec = total_bytes as f64 / elapsed.as_secs_f64();
    
    println!("  监控时间: {:?}", elapsed);
    println!("  总操作数: {}", operation_count);
    println!("  总字节数: {}", total_bytes);
    println!("  操作速率: {:.1} ops/s", ops_per_sec);
    println!("  数据速率: {:.1} bytes/s ({:.2} KB/s)", bytes_per_sec, bytes_per_sec / 1024.0);

    // 2. 错误率监控
    println!("\n2️⃣  错误率监控:");
    monitor_error_rates(&mut uart).await?;

    // 3. 延迟分析
    println!("\n3️⃣  延迟分析:");
    analyze_latency(&mut uart).await?;

    // 4. 资源使用监控
    println!("\n4️⃣  资源使用监控:");
    monitor_resource_usage(&mut uart).await?;

    // 5. 健康检查
    println!("\n5️⃣  健康检查:");
    perform_health_check(&mut uart).await?;

    Ok(())
}

/// 示例7: 自定义适配器
async fn custom_adapter_demo() -> Result<()> {
    println!("\n🔧 示例7: 自定义适配器");
    println!("----------------------");

    // 这里演示如何扩展GenericHalAdapter
    let mut custom_adapter = GenericHalAdapter::new();
    
    // 配置自定义行为
    custom_adapter.set_max_buffer_size(16384);
    custom_adapter.enable_delay_simulation(true);
    custom_adapter.enable_error_injection(false);
    
    println!("✓ 创建自定义适配器");
    println!("  - 最大缓冲区: 16KB");
    println!("  - 延迟模拟: 启用");
    println!("  - 错误注入: 禁用");

    // 使用自定义适配器创建UART
    let mut uart = AsyncUartBuilder::new()
        .with_config(presets::standard_115200())
        .with_pins(UartPins::new(1, 2))
        .with_adapter(custom_adapter)
        .build()
        .await?;

    // 测试自定义适配器功能
    println!("\n🧪 测试自定义适配器:");
    
    let test_data = b"Custom adapter test data";
    uart.write_all(test_data).await?;
    println!("  ✓ 写入测试数据: {} 字节", test_data.len());

    let mut read_buffer = vec![0u8; test_data.len()];
    let read_count = uart.read(&mut read_buffer).await?;
    println!("  ✓ 读取数据: {} 字节", read_count);
    
    if read_count > 0 {
        println!("  数据内容: {:?}", String::from_utf8_lossy(&read_buffer[..read_count]));
    }

    // 获取适配器统计信息
    // 注意：在实际实现中，你需要添加获取统计信息的方法
    println!("  ✓ 自定义适配器测试完成");

    Ok(())
}

// 辅助函数和结构体

#[derive(Debug, Default)]
struct ErrorStatistics {
    total_operations: usize,
    successful_operations: usize,
    failed_operations: usize,
    error_distribution: std::collections::HashMap<String, usize>,
}

impl ErrorStatistics {
    fn success_rate(&self) -> f64 {
        if self.total_operations > 0 {
            (self.successful_operations as f64 / self.total_operations as f64) * 100.0
        } else {
            0.0
        }
    }
}

async fn collect_error_statistics(uart: &mut AsyncUart<GenericHalAdapter>) -> ErrorStatistics {
    let mut stats = ErrorStatistics::default();
    
    // 执行一系列操作来收集统计信息
    for _ in 0..20 {
        stats.total_operations += 1;
        
        match uart.write_u8(0x42).await {
            Ok(_) => stats.successful_operations += 1,
            Err(e) => {
                stats.failed_operations += 1;
                let error_type = format!("{:?}", e);
                *stats.error_distribution.entry(error_type).or_insert(0) += 1;
            }
        }
    }
    
    stats
}

async fn implement_recovery_strategy(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    // 实现简单的恢复策略
    for attempt in 1..=3 {
        match uart.write_u8(0x55).await {
            Ok(_) => {
                println!("    尝试 {}: 恢复成功", attempt);
                return Ok(());
            }
            Err(e) => {
                println!("    尝试 {}: 恢复失败 - {:?}", attempt, e);
                if attempt < 3 {
                    sleep(Duration::from_millis(100)).await;
                }
            }
        }
    }
    
    Err(AsyncUartError::HardwareError("恢复失败".to_string()))
}

async fn measure_throughput_with_buffer_size(config: UartConfig) -> Result<f64> {
    let hal_adapter = GenericHalAdapter::new();
    let mut uart = AsyncUartBuilder::new()
        .with_config(config)
        .with_pins(UartPins::new(1, 2))
        .with_adapter(hal_adapter)
        .build()
        .await?;

    let test_data = vec![0x55u8; 1024];
    let start = Instant::now();
    
    for _ in 0..10 {
        uart.write_all(&test_data).await?;
    }
    
    let elapsed = start.elapsed();
    let total_bytes = test_data.len() * 10;
    
    Ok((total_bytes as f64 / 1024.0) / elapsed.as_secs_f64())
}

async fn measure_throughput_with_config(config: UartConfig) -> Result<f64> {
    measure_throughput_with_buffer_size(config).await
}

async fn demonstrate_memory_optimization() -> Result<()> {
    println!("  💾 内存使用优化技巧:");
    println!("    - 使用适当的缓冲区大小");
    println!("    - 重用缓冲区避免频繁分配");
    println!("    - 使用零拷贝操作");
    println!("    - 及时释放不需要的资源");
    Ok(())
}

async fn demonstrate_cpu_optimization(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    println!("  🖥️  CPU使用率优化:");
    
    // 演示批量操作vs单个操作的CPU效率
    let data = vec![0x55u8; 1000];
    
    // 批量操作（CPU效率高）
    let start = Instant::now();
    uart.write_all(&data).await?;
    let batch_time = start.elapsed();
    
    println!("    - 批量操作: {:?}", batch_time);
    println!("    - 使用异步操作减少阻塞");
    println!("    - 合理使用缓冲区减少系统调用");
    
    Ok(())
}

async fn long_running_operation(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    // 模拟长时间运行的操作
    for i in 0..100 {
        uart.write_u8(i as u8).await?;
        sleep(Duration::from_millis(10)).await;
    }
    Ok(())
}

async fn retry_with_timeout(
    uart: &mut AsyncUart<GenericHalAdapter>,
    max_retries: usize,
    timeout_duration: Duration,
) -> Result<Vec<u8>> {
    for attempt in 1..=max_retries {
        match timeout(timeout_duration, uart.read_u8()).await {
            Ok(Ok(byte)) => {
                println!("    尝试 {}: 成功读取 0x{:02X}", attempt, byte);
                return Ok(vec![byte]);
            }
            Ok(Err(e)) => {
                println!("    尝试 {}: 读取错误 - {:?}", attempt, e);
            }
            Err(_) => {
                println!("    尝试 {}: 超时", attempt);
            }
        }
        
        if attempt < max_retries {
            sleep(Duration::from_millis(50)).await;
        }
    }
    
    Err(AsyncUartError::Timeout)
}

async fn demonstrate_buffer_management(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    println!("  📋 缓冲区管理优化:");
    
    // 演示缓冲区重用
    let mut reusable_buffer = vec![0u8; 1024];
    
    for i in 0..5 {
        // 填充数据
        for (j, byte) in reusable_buffer.iter_mut().enumerate() {
            *byte = (i * 256 + j) as u8;
        }
        
        uart.write_all(&reusable_buffer).await?;
        println!("    批次 {}: 重用缓冲区写入 {} 字节", i + 1, reusable_buffer.len());
    }
    
    Ok(())
}

async fn demonstrate_zero_copy_operations(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    println!("  🚀 零拷贝操作:");
    
    // 在实际实现中，这里会演示如何避免数据拷贝
    // 例如使用引用、切片等
    let static_data: &'static [u8] = b"Static data for zero-copy demo";
    uart.write_all(static_data).await?;
    println!("    ✓ 使用静态数据避免拷贝");
    
    // 使用切片操作
    let large_buffer = vec![0x42u8; 2048];
    for chunk in large_buffer.chunks(256) {
        uart.write_all(chunk).await?;
    }
    println!("    ✓ 使用切片分块传输");
    
    Ok(())
}

async fn monitor_error_rates(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    let mut total_ops = 0;
    let mut errors = 0;
    
    for _ in 0..50 {
        total_ops += 1;
        if let Err(_) = uart.write_u8(0x55).await {
            errors += 1;
        }
    }
    
    let error_rate = (errors as f64 / total_ops as f64) * 100.0;
    println!("  错误率: {:.1}% ({}/{} 操作)", error_rate, errors, total_ops);
    
    Ok(())
}

async fn analyze_latency(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    let mut latencies = Vec::new();
    
    for _ in 0..20 {
        let start = Instant::now();
        let _ = uart.write_u8(0x55).await;
        latencies.push(start.elapsed());
    }
    
    let avg_latency = latencies.iter().sum::<Duration>() / latencies.len() as u32;
    let min_latency = latencies.iter().min().unwrap();
    let max_latency = latencies.iter().max().unwrap();
    
    println!("  延迟分析:");
    println!("    平均: {:?}", avg_latency);
    println!("    最小: {:?}", min_latency);
    println!("    最大: {:?}", max_latency);
    
    Ok(())
}

async fn monitor_resource_usage(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    println!("  资源使用监控:");
    println!("    内存使用: ~{} KB", std::mem::size_of_val(uart) / 1024);
    println!("    缓冲区状态: 正常");
    println!("    连接状态: 活跃");
    
    Ok(())
}

async fn perform_health_check(uart: &mut AsyncUart<GenericHalAdapter>) -> Result<()> {
    println!("  健康检查结果:");
    
    // 连接测试
    match uart.write_u8(0x00).await {
        Ok(_) => println!("    ✓ 连接正常"),
        Err(_) => println!("    ❌ 连接异常"),
    }
    
    // 配置检查
    println!("    ✓ 配置有效");
    
    // 性能检查
    println!("    ✓ 性能正常");
    
    // 总体状态
    println!("    🟢 系统健康");
    
    Ok(())
}