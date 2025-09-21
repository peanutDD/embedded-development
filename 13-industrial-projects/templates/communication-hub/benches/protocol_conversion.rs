//! 协议转换性能基准测试

use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn benchmark_protocol_conversion(c: &mut Criterion) {
    c.bench_function("modbus_to_mqtt", |b| {
        b.iter(|| {
            // 模拟Modbus到MQTT协议转换
            black_box(simulate_modbus_to_mqtt_conversion())
        })
    });
}

fn simulate_modbus_to_mqtt_conversion() -> u32 {
    // 模拟协议转换处理
    let mut result = 0;
    for i in 0..500 {
        result += i * 2;
    }
    result
}

criterion_group!(benches, benchmark_protocol_conversion);
criterion_main!(benches);