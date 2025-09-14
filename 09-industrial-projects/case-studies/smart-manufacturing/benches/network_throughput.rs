//! 网络吞吐量性能基准测试

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::collections::HashMap;
use std::time::{Duration, Instant};

fn benchmark_network_throughput(c: &mut Criterion) {
    c.bench_function("tcp_data_transfer", |b| {
        b.iter(|| {
            // 模拟TCP数据传输
            black_box(simulate_tcp_transfer(1024))
        })
    });
    
    c.bench_function("udp_data_transfer", |b| {
        b.iter(|| {
            // 模拟UDP数据传输
            black_box(simulate_udp_transfer(512))
        })
    });
    
    c.bench_function("ethernet_frame_processing", |b| {
        b.iter(|| {
            // 模拟以太网帧处理
            black_box(simulate_ethernet_frame_processing())
        })
    });
    
    c.bench_function("protocol_conversion", |b| {
        b.iter(|| {
            // 模拟协议转换
            black_box(simulate_protocol_conversion())
        })
    });
}

fn simulate_tcp_transfer(size: usize) -> Vec<u8> {
    let mut data = Vec::with_capacity(size);
    
    // 模拟数据包创建
    for i in 0..size {
        data.push((i % 256) as u8);
    }
    
    // 模拟传输延迟
    std::thread::sleep(Duration::from_nanos(size as u64 * 10));
    
    data
}

fn simulate_udp_transfer(size: usize) -> Vec<u8> {
    let mut data = Vec::with_capacity(size);
    
    // 模拟数据包创建
    for i in 0..size {
        data.push((i % 256) as u8);
    }
    
    // UDP传输更快
    std::thread::sleep(Duration::from_nanos(size as u64 * 5));
    
    data
}

fn simulate_ethernet_frame_processing() -> HashMap<String, u32> {
    let mut stats = HashMap::new();
    
    // 模拟处理多个以太网帧
    for i in 0..10 {
        let frame_type = match i % 3 {
            0 => "ARP",
            1 => "IP",
            _ => "OTHER",
        };
        
        *stats.entry(frame_type.to_string()).or_insert(0) += 1;
    }
    
    // 模拟处理延迟
    std::thread::sleep(Duration::from_nanos(500));
    
    stats
}

fn simulate_protocol_conversion() -> Vec<u8> {
    // 模拟Modbus到MQTT的协议转换
    let modbus_data = vec![0x01, 0x03, 0x00, 0x00, 0x00, 0x0A];
    let mut mqtt_payload = Vec::new();
    
    // 简单的协议转换逻辑
    mqtt_payload.extend_from_slice(b"{");
    mqtt_payload.extend_from_slice(b"\"function_code\":");
    mqtt_payload.push(modbus_data[1] + b'0');
    mqtt_payload.extend_from_slice(b",\"data\":[\"]");
    
    for (i, byte) in modbus_data[2..].iter().enumerate() {
        if i > 0 {
            mqtt_payload.extend_from_slice(b",");
        }
        mqtt_payload.extend_from_slice(byte.to_string().as_bytes());
    }
    
    mqtt_payload.extend_from_slice(b"]}");
    
    // 模拟转换延迟
    std::thread::sleep(Duration::from_nanos(200));
    
    mqtt_payload
}

criterion_group!(benches, benchmark_network_throughput);
criterion_main!(benches);