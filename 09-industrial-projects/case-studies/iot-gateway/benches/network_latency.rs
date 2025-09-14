//! 网络延迟基准测试
//! 
//! 测试网络通信的延迟性能

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::time::{Duration, Instant};

#[cfg(feature = "wifi-support")]
fn benchmark_network_latency(c: &mut Criterion) {
    c.bench_function("packet_processing", |b| {
        b.iter(|| {
            // 模拟数据包处理
            let packet = create_test_packet();
            let processed = process_packet(&packet);
            black_box(processed)
        })
    });
    
    c.bench_function("connection_setup", |b| {
        b.iter(|| {
            // 模拟连接建立
            let connection = simulate_connection_setup();
            black_box(connection)
        })
    });
    
    c.bench_function("data_transmission", |b| {
        b.iter(|| {
            // 模拟数据传输
            let data = generate_transmission_data();
            let transmitted = simulate_transmission(&data);
            black_box(transmitted)
        })
    });
}

#[cfg(feature = "wifi-support")]
fn create_test_packet() -> Vec<u8> {
    // 创建测试数据包
    let mut packet = Vec::with_capacity(64);
    
    // 添加头部
    packet.extend_from_slice(&[0x08, 0x00]); // 协议类型
    packet.extend_from_slice(&[0x45, 0x00]); // IP版本和头长度
    packet.extend_from_slice(&[0x00, 0x3C]); // 总长度
    
    // 添加负载数据
    for i in 0..56 {
        packet.push((i % 256) as u8);
    }
    
    packet
}

#[cfg(feature = "wifi-support")]
fn process_packet(packet: &[u8]) -> Vec<u8> {
    // 处理数据包 - 简单的校验和计算
    let mut checksum = 0u16;
    
    for chunk in packet.chunks(2) {
        if chunk.len() == 2 {
            let word = u16::from_be_bytes([chunk[0], chunk[1]]);
            checksum = checksum.wrapping_add(word);
        } else {
            checksum = checksum.wrapping_add((chunk[0] as u16) << 8);
        }
    }
    
    checksum.to_be_bytes().to_vec()
}

#[cfg(feature = "wifi-support")]
fn simulate_connection_setup() -> bool {
    // 模拟连接建立过程
    let start = Instant::now();
    
    // 模拟一些处理时间
    let mut sum = 0u64;
    for i in 0..1000 {
        sum = sum.wrapping_add(i);
    }
    
    let elapsed = start.elapsed();
    elapsed < Duration::from_millis(1) && sum > 0
}

#[cfg(feature = "wifi-support")]
fn generate_transmission_data() -> Vec<u8> {
    // 生成传输数据
    (0..128).map(|i| (i * 3 + 7) as u8).collect()
}

#[cfg(feature = "wifi-support")]
fn simulate_transmission(data: &[u8]) -> Vec<u8> {
    // 模拟数据传输 - 添加错误检测码
    let mut result = Vec::with_capacity(data.len() + 4);
    
    // 添加原始数据
    result.extend_from_slice(data);
    
    // 计算CRC32 (简化版)
    let mut crc = 0xFFFFFFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    // 添加CRC
    result.extend_from_slice(&crc.to_le_bytes());
    
    result
}

#[cfg(feature = "wifi-support")]
criterion_group!(benches, benchmark_network_latency);

#[cfg(not(feature = "wifi-support"))]
fn benchmark_disabled(_c: &mut Criterion) {
    // 当特性未启用时的占位符
}

#[cfg(not(feature = "wifi-support"))]
criterion_group!(benches, benchmark_disabled);

criterion_main!(benches);