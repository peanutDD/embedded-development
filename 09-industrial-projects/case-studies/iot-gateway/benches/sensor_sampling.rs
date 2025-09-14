//! 传感器采样基准测试
//! 
//! 测试传感器数据采样的性能

use criterion::{black_box, criterion_group, criterion_main, Criterion};

#[cfg(feature = "sensor-support")]
fn benchmark_sensor_sampling(c: &mut Criterion) {
    c.bench_function("sensor_sampling", |b| {
        b.iter(|| {
            // 模拟传感器采样
            let data = simulate_sensor_reading();
            black_box(data)
        })
    });
}

#[cfg(feature = "sensor-support")]
fn simulate_sensor_reading() -> [u8; 8] {
    // 模拟传感器读取
    [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]
}

#[cfg(feature = "sensor-support")]
criterion_group!(benches, benchmark_sensor_sampling);

#[cfg(not(feature = "sensor-support"))]
fn benchmark_disabled(_c: &mut Criterion) {
    // 当特性未启用时的占位符
}

#[cfg(not(feature = "sensor-support"))]
criterion_group!(benches, benchmark_disabled);

criterion_main!(benches);