//! I/O扫描性能基准测试

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::collections::HashMap;

fn benchmark_io_scanning(c: &mut Criterion) {
    c.bench_function("digital_input_scan", |b| {
        b.iter(|| {
            // 模拟数字输入扫描
            black_box(simulate_digital_input_scan())
        })
    });
    
    c.bench_function("analog_input_scan", |b| {
        b.iter(|| {
            // 模拟模拟输入扫描
            black_box(simulate_analog_input_scan())
        })
    });
    
    c.bench_function("output_update", |b| {
        b.iter(|| {
            // 模拟输出更新
            black_box(simulate_output_update())
        })
    });
}

fn simulate_digital_input_scan() -> HashMap<u32, bool> {
    let mut inputs = HashMap::new();
    
    // 模拟扫描64个数字输入
    for i in 0..64 {
        inputs.insert(i, (i % 3) == 0);
    }
    
    inputs
}

fn simulate_analog_input_scan() -> HashMap<u32, f32> {
    let mut inputs = HashMap::new();
    
    // 模拟扫描16个模拟输入
    for i in 0..16 {
        let value = (i as f32) * 0.1 + 1.0;
        inputs.insert(i, value);
    }
    
    inputs
}

fn simulate_output_update() -> HashMap<u32, bool> {
    let mut outputs = HashMap::new();
    
    // 模拟更新32个数字输出
    for i in 0..32 {
        outputs.insert(i, (i % 2) == 0);
    }
    
    outputs
}

criterion_group!(benches, benchmark_io_scanning);
criterion_main!(benches);