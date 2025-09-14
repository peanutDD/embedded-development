//! 传感器性能基准测试

use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn benchmark_sensor_performance(c: &mut Criterion) {
    c.bench_function("sensor_data_processing", |b| {
        b.iter(|| {
            // 模拟传感器数据处理
            black_box(simulate_sensor_data_processing())
        })
    });
    
    c.bench_function("sensor_calibration", |b| {
        b.iter(|| {
            // 模拟传感器校准
            black_box(simulate_sensor_calibration())
        })
    });
}

fn simulate_sensor_data_processing() -> f32 {
    // 模拟传感器数据处理
    let mut sum = 0.0;
    for i in 0..100 {
        sum += (i as f32) * 0.1;
    }
    sum
}

fn simulate_sensor_calibration() -> f32 {
    // 模拟传感器校准计算
    let raw_value = 1024.0;
    let offset = 50.0;
    let scale = 0.01;
    (raw_value - offset) * scale
}

criterion_group!(benches, benchmark_sensor_performance);
criterion_main!(benches);