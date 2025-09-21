//! Modbus通信性能基准测试

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::collections::HashMap;

fn benchmark_modbus_performance(c: &mut Criterion) {
    c.bench_function("modbus_read_coils", |b| {
        b.iter(|| {
            // 模拟Modbus读取线圈
            black_box(simulate_modbus_read_coils(100))
        })
    });
    
    c.bench_function("modbus_read_holding_registers", |b| {
        b.iter(|| {
            // 模拟Modbus读取保持寄存器
            black_box(simulate_modbus_read_holding_registers(50))
        })
    });
    
    c.bench_function("modbus_write_single_coil", |b| {
        b.iter(|| {
            // 模拟Modbus写单个线圈
            black_box(simulate_modbus_write_single_coil(0x0001, true))
        })
    });
    
    c.bench_function("modbus_write_multiple_registers", |b| {
        b.iter(|| {
            // 模拟Modbus写多个寄存器
            let values = vec![0x1234, 0x5678, 0x9ABC, 0xDEF0];
            black_box(simulate_modbus_write_multiple_registers(0x0000, values))
        })
    });
}

fn simulate_modbus_read_coils(count: u16) -> Vec<bool> {
    let mut coils = Vec::new();
    
    for i in 0..count {
        coils.push((i % 3) == 0);
    }
    
    coils
}

fn simulate_modbus_read_holding_registers(count: u16) -> Vec<u16> {
    let mut registers = Vec::new();
    
    for i in 0..count {
        registers.push(0x1000 + i);
    }
    
    registers
}

fn simulate_modbus_write_single_coil(address: u16, value: bool) -> bool {
    // 模拟写操作延迟
    std::thread::sleep(std::time::Duration::from_nanos(100));
    value
}

fn simulate_modbus_write_multiple_registers(start_address: u16, values: Vec<u16>) -> usize {
    // 模拟写操作延迟
    std::thread::sleep(std::time::Duration::from_nanos(values.len() as u64 * 50));
    values.len()
}

criterion_group!(benches, benchmark_modbus_performance);
criterion_main!(benches);