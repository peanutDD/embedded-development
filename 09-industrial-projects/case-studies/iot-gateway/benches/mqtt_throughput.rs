//! MQTT吞吐量基准测试
//! 
//! 测试MQTT消息传输的性能

use criterion::{black_box, criterion_group, criterion_main, Criterion};

#[cfg(feature = "mqtt-support")]
fn benchmark_mqtt_throughput(c: &mut Criterion) {
    c.bench_function("mqtt_message_processing", |b| {
        b.iter(|| {
            // 模拟MQTT消息处理
            let message = create_test_message();
            let processed = process_mqtt_message(&message);
            black_box(processed)
        })
    });
    
    c.bench_function("mqtt_message_serialization", |b| {
        b.iter(|| {
            // 模拟MQTT消息序列化
            let data = [1, 2, 3, 4, 5, 6, 7, 8];
            let serialized = serialize_message(&data);
            black_box(serialized)
        })
    });
}

#[cfg(feature = "mqtt-support")]
fn create_test_message() -> Vec<u8> {
    // 创建测试消息
    vec![0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80]
}

#[cfg(feature = "mqtt-support")]
fn process_mqtt_message(message: &[u8]) -> Vec<u8> {
    // 处理MQTT消息
    message.iter().map(|&b| b ^ 0xFF).collect()
}

#[cfg(feature = "mqtt-support")]
fn serialize_message(data: &[u8]) -> Vec<u8> {
    // 序列化消息
    let mut result = Vec::with_capacity(data.len() + 4);
    result.extend_from_slice(&(data.len() as u32).to_le_bytes());
    result.extend_from_slice(data);
    result
}

#[cfg(feature = "mqtt-support")]
criterion_group!(benches, benchmark_mqtt_throughput);

#[cfg(not(feature = "mqtt-support"))]
fn benchmark_disabled(_c: &mut Criterion) {
    // 当特性未启用时的占位符
}

#[cfg(not(feature = "mqtt-support"))]
criterion_group!(benches, benchmark_disabled);

criterion_main!(benches);