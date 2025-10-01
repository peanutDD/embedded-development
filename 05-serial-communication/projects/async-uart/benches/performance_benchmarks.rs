//! # 性能基准测试
//!
//! 使用criterion库对async_uart的各个组件进行性能基准测试。

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
use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use std::time::Duration;
use tokio::runtime::Runtime;

/// 基准测试：环形缓冲区性能
fn bench_ring_buffer(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("ring_buffer");

    // 测试不同大小的缓冲区
    for buffer_size in [512, 1024, 2048, 4096, 8192].iter() {
        group.throughput(Throughput::Bytes(*buffer_size as u64));
        
        group.bench_with_input(
            BenchmarkId::new("write_read", buffer_size),
            buffer_size,
            |b, &size| {
                b.to_async(&rt).iter(|| async {
                    let mut buffer = RingBuffer::new(size);
                    let data = vec![0x55u8; size / 2];
                    
                    // 写入数据
                    let written = buffer.write(black_box(&data)).await.unwrap();
                    
                    // 读取数据
                    let mut read_buf = vec![0u8; written];
                    let read = buffer.read(black_box(&mut read_buf)).await.unwrap();
                    
                    black_box((written, read));
                });
            },
        );
    }

    // 测试环绕性能
    group.bench_function("ring_wrap_around", |b| {
        b.to_async(&rt).iter(|| async {
            let mut buffer = RingBuffer::new(1024);
            let data = vec![0xAAu8; 800];
            
            // 填充大部分缓冲区
            buffer.write(&data).await.unwrap();
            
            // 触发环绕
            let wrap_data = vec![0xBBu8; 400];
            let written = buffer.write(black_box(&wrap_data)).await.unwrap();
            
            black_box(written);
        });
    });

    group.finish();
}

/// 基准测试：DMA缓冲区性能
fn bench_dma_buffer(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("dma_buffer");

    for buffer_size in [1024, 2048, 4096, 8192].iter() {
        group.throughput(Throughput::Bytes(*buffer_size as u64));
        
        group.bench_with_input(
            BenchmarkId::new("dma_transfer", buffer_size),
            buffer_size,
            |b, &size| {
                b.to_async(&rt).iter(|| async {
                    let config = DmaBufferConfig::new()
                        .with_size(size)
                        .with_alignment(64)
                        .enable_stats(true);
                    
                    let mut buffer = DmaBuffer::new(config).unwrap();
                    let data = vec![0x55u8; size / 2];
                    
                    // 写入数据
                    let written = buffer.write(black_box(&data)).await.unwrap();
                    
                    // 读取数据
                    let mut read_buf = vec![0u8; written];
                    let read = buffer.read(black_box(&mut read_buf)).await.unwrap();
                    
                    black_box((written, read));
                });
            },
        );
    }

    group.finish();
}

/// 基准测试：流式缓冲区性能
fn bench_stream_buffer(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("stream_buffer");

    for buffer_size in [1024, 2048, 4096, 8192].iter() {
        group.throughput(Throughput::Bytes(*buffer_size as u64));
        
        group.bench_with_input(
            BenchmarkId::new("stream_operations", buffer_size),
            buffer_size,
            |b, &size| {
                b.to_async(&rt).iter(|| async {
                    let config = StreamBufferConfig::new()
                        .with_size(size)
                        .with_overflow_strategy(OverflowStrategy::Overwrite)
                        .enable_stats(true);
                    
                    let mut buffer = StreamBuffer::new(config).unwrap();
                    let data = vec![0x55u8; size / 4];
                    
                    // 写入数据
                    let written = buffer.write(black_box(&data)).await.unwrap();
                    
                    // 读取数据
                    let mut read_buf = vec![0u8; written];
                    let read = buffer.read(black_box(&mut read_buf)).await.unwrap();
                    
                    black_box((written, read));
                });
            },
        );
    }

    // 测试块操作性能
    group.bench_function("block_operations", |b| {
        b.to_async(&rt).iter(|| async {
            let config = StreamBufferConfig::new()
                .with_size(4096)
                .with_block_size(256);
            
            let mut buffer = StreamBuffer::new(config).unwrap();
            let data = vec![0xAAu8; 256];
            
            // 写入块
            buffer.write_block(black_box(&data)).await.unwrap();
            
            // 读取块
            let read_block = buffer.read_block(black_box(128)).await.unwrap();
            
            black_box(read_block);
        });
    });

    group.finish();
}

/// 基准测试：协议处理性能
fn bench_protocol_processing(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("protocol_processing");

    // 测试不同大小的消息处理
    for message_size in [64, 256, 1024, 4096].iter() {
        group.throughput(Throughput::Bytes(*message_size as u64));
        
        group.bench_with_input(
            BenchmarkId::new("raw_protocol_encode_decode", message_size),
            message_size,
            |b, &size| {
                b.to_async(&rt).iter(|| async {
                    let mut handler = RawProtocolHandler::new();
                    let data = vec![0x42u8; size];
                    
                    let message = ProtocolMessage::new(
                        MessageType::Data,
                        black_box(data)
                    );
                    
                    // 编码消息
                    let encoded = handler.encode_message(black_box(&message)).await.unwrap();
                    
                    // 解码消息
                    let decoded = handler.decode_message(black_box(&encoded)).await.unwrap();
                    
                    black_box(decoded);
                });
            },
        );
    }

    // 测试批量消息处理
    group.bench_function("batch_message_processing", |b| {
        b.to_async(&rt).iter(|| async {
            let mut handler = RawProtocolBuilder::new()
                .batch_size(10)
                .build();
            
            let test_data = b"Batch processing test data for performance measurement";
            let messages = handler.handle_received_data(black_box(test_data)).await.unwrap();
            
            black_box(messages);
        });
    });

    group.finish();
}

/// 基准测试：UART操作性能
fn bench_uart_operations(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("uart_operations");

    // 测试不同配置的UART性能
    let configs = vec![
        ("standard_115200", presets::standard_115200()),
        ("high_speed_921600", presets::high_speed_921600()),
        ("debug_config", presets::debug_config()),
    ];

    for (name, config) in configs {
        group.bench_function(&format!("uart_write_{}", name), |b| {
            b.to_async(&rt).iter(|| async {
                let hal_adapter = GenericHalAdapter::new();
                let mut uart = AsyncUartBuilder::new()
                    .with_config(config.clone())
                    .with_pins(UartPins::new(1, 2))
                    .with_adapter(hal_adapter)
                    .build()
                    .await
                    .unwrap();

                let data = vec![0x55u8; 256];
                uart.write_all(black_box(&data)).await.unwrap();
            });
        });
    }

    // 测试批量vs单字节操作
    group.bench_function("single_byte_operations", |b| {
        b.to_async(&rt).iter(|| async {
            let hal_adapter = GenericHalAdapter::new();
            let mut uart = AsyncUartBuilder::new()
                .with_config(presets::standard_115200())
                .with_pins(UartPins::new(1, 2))
                .with_adapter(hal_adapter)
                .build()
                .await
                .unwrap();

            // 写入100个单字节
            for i in 0..100u8 {
                uart.write_u8(black_box(i)).await.unwrap();
            }
        });
    });

    group.bench_function("batch_operations", |b| {
        b.to_async(&rt).iter(|| async {
            let hal_adapter = GenericHalAdapter::new();
            let mut uart = AsyncUartBuilder::new()
                .with_config(presets::standard_115200())
                .with_pins(UartPins::new(1, 2))
                .with_adapter(hal_adapter)
                .build()
                .await
                .unwrap();

            // 批量写入100字节
            let data: Vec<u8> = (0..100).collect();
            uart.write_all(black_box(&data)).await.unwrap();
        });
    });

    group.finish();
}

/// 基准测试：内存分配性能
fn bench_memory_allocation(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("memory_allocation");

    // 测试缓冲区创建性能
    group.bench_function("ring_buffer_creation", |b| {
        b.iter(|| {
            let buffer = RingBuffer::new(black_box(4096));
            black_box(buffer);
        });
    });

    group.bench_function("dma_buffer_creation", |b| {
        b.iter(|| {
            let config = DmaBufferConfig::new().with_size(black_box(4096));
            let buffer = DmaBuffer::new(config).unwrap();
            black_box(buffer);
        });
    });

    group.bench_function("stream_buffer_creation", |b| {
        b.iter(|| {
            let config = StreamBufferConfig::new().with_size(black_box(4096));
            let buffer = StreamBuffer::new(config).unwrap();
            black_box(buffer);
        });
    });

    // 测试UART创建性能
    group.bench_function("uart_creation", |b| {
        b.to_async(&rt).iter(|| async {
            let hal_adapter = GenericHalAdapter::new();
            let uart = AsyncUartBuilder::new()
                .with_config(presets::standard_115200())
                .with_pins(UartPins::new(1, 2))
                .with_adapter(hal_adapter)
                .build()
                .await
                .unwrap();
            
            black_box(uart);
        });
    });

    group.finish();
}

/// 基准测试：并发性能
fn bench_concurrent_operations(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("concurrent_operations");

    // 测试并发缓冲区操作
    group.bench_function("concurrent_ring_buffer", |b| {
        b.to_async(&rt).iter(|| async {
            let mut buffer = RingBuffer::new(8192);
            let data = vec![0x55u8; 1024];
            
            // 创建多个并发任务
            let mut handles = Vec::new();
            
            for i in 0..4 {
                let mut data_copy = data.clone();
                data_copy[0] = i as u8; // 区分不同任务的数据
                
                let handle = tokio::spawn(async move {
                    // 模拟一些处理
                    tokio::time::sleep(Duration::from_nanos(100)).await;
                    data_copy
                });
                handles.push(handle);
            }
            
            // 等待所有任务完成并写入数据
            for handle in handles {
                let task_data = handle.await.unwrap();
                buffer.write(black_box(&task_data)).await.unwrap();
            }
        });
    });

    // 测试并发UART操作
    group.bench_function("concurrent_uart_operations", |b| {
        b.to_async(&rt).iter(|| async {
            let hal_adapter = GenericHalAdapter::new();
            let mut uart = AsyncUartBuilder::new()
                .with_config(presets::high_speed_921600())
                .with_pins(UartPins::new(1, 2))
                .with_adapter(hal_adapter)
                .build()
                .await
                .unwrap();

            // 创建多个并发写入任务
            let mut handles = Vec::new();
            
            for i in 0..4 {
                let data = vec![i as u8; 256];
                let handle = tokio::spawn(async move {
                    tokio::time::sleep(Duration::from_nanos(50)).await;
                    data
                });
                handles.push(handle);
            }
            
            // 顺序执行写入操作
            for handle in handles {
                let task_data = handle.await.unwrap();
                uart.write_all(black_box(&task_data)).await.unwrap();
            }
        });
    });

    group.finish();
}

/// 基准测试：错误处理性能
fn bench_error_handling(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("error_handling");

    // 测试正常操作vs错误处理的性能差异
    group.bench_function("normal_operations", |b| {
        b.to_async(&rt).iter(|| async {
            let hal_adapter = GenericHalAdapter::new();
            let mut uart = AsyncUartBuilder::new()
                .with_config(presets::standard_115200())
                .with_pins(UartPins::new(1, 2))
                .with_adapter(hal_adapter)
                .build()
                .await
                .unwrap();

            let data = vec![0x55u8; 256];
            let result = uart.write_all(black_box(&data)).await;
            black_box(result);
        });
    });

    group.bench_function("error_recovery", |b| {
        b.to_async(&rt).iter(|| async {
            let mut hal_adapter = GenericHalAdapter::new();
            hal_adapter.enable_error_injection(true);
            
            let mut uart = AsyncUartBuilder::new()
                .with_config(presets::standard_115200())
                .with_pins(UartPins::new(1, 2))
                .with_adapter(hal_adapter)
                .build()
                .await
                .unwrap();

            let data = vec![0x55u8; 256];
            
            // 尝试操作，可能会失败
            for _ in 0..3 {
                if uart.write_all(black_box(&data)).await.is_ok() {
                    break;
                }
                // 简单的重试逻辑
                tokio::time::sleep(Duration::from_nanos(100)).await;
            }
        });
    });

    group.finish();
}

/// 基准测试：配置验证性能
fn bench_configuration_validation(c: &mut Criterion) {
    let mut group = c.benchmark_group("configuration_validation");

    // 测试配置创建和验证性能
    group.bench_function("config_creation_and_validation", |b| {
        b.iter(|| {
            let config = async_uart::config::UartConfig::builder()
                .baud_rate(black_box(115200))
                .data_bits(black_box(8))
                .stop_bits(async_uart::config::StopBits::One)
                .parity(async_uart::config::Parity::None)
                .flow_control(async_uart::config::FlowControl::None)
                .timeout(Duration::from_millis(black_box(1000)))
                .buffer_size(black_box(2048))
                .enable_dma(black_box(false))
                .build()
                .unwrap();

            // 验证配置
            let validation_result = config.validate();
            black_box(validation_result);
        });
    });

    // 测试预设配置性能
    group.bench_function("preset_configs", |b| {
        b.iter(|| {
            let configs = vec![
                presets::standard_115200(),
                presets::high_speed_921600(),
                presets::low_power_9600(),
                presets::debug_config(),
            ];
            
            for config in configs {
                let validation_result = config.validate();
                black_box(validation_result);
            }
        });
    });

    group.finish();
}

// 配置基准测试组
criterion_group!(
    benches,
    bench_ring_buffer,
    bench_dma_buffer,
    bench_stream_buffer,
    bench_protocol_processing,
    bench_uart_operations,
    bench_memory_allocation,
    bench_concurrent_operations,
    bench_error_handling,
    bench_configuration_validation
);

criterion_main!(benches);