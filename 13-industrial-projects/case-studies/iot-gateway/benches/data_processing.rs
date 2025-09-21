//! 数据处理基准测试
//! 
//! 测试数据过滤和处理的性能

use criterion::{black_box, criterion_group, criterion_main, Criterion};

#[cfg(feature = "data-filtering")]
fn benchmark_data_processing(c: &mut Criterion) {
    c.bench_function("data_filtering", |b| {
        b.iter(|| {
            // 模拟数据过滤
            let data = generate_test_data();
            let filtered = apply_filter(&data);
            black_box(filtered)
        })
    });
    
    c.bench_function("data_aggregation", |b| {
        b.iter(|| {
            // 模拟数据聚合
            let data = generate_test_data();
            let aggregated = aggregate_data(&data);
            black_box(aggregated)
        })
    });
    
    c.bench_function("data_transformation", |b| {
        b.iter(|| {
            // 模拟数据转换
            let data = generate_test_data();
            let transformed = transform_data(&data);
            black_box(transformed)
        })
    });
}

#[cfg(feature = "data-filtering")]
fn generate_test_data() -> Vec<f32> {
    // 生成测试数据
    (0..1000).map(|i| (i as f32) * 0.1).collect()
}

#[cfg(feature = "data-filtering")]
fn apply_filter(data: &[f32]) -> Vec<f32> {
    // 应用简单的低通滤波器
    let mut filtered = Vec::with_capacity(data.len());
    let mut prev = 0.0;
    
    for &value in data {
        let filtered_value = 0.8 * prev + 0.2 * value;
        filtered.push(filtered_value);
        prev = filtered_value;
    }
    
    filtered
}

#[cfg(feature = "data-filtering")]
fn aggregate_data(data: &[f32]) -> Vec<f32> {
    // 数据聚合 - 计算移动平均
    let window_size = 10;
    let mut aggregated = Vec::new();
    
    for i in 0..data.len() {
        let start = if i >= window_size { i - window_size } else { 0 };
        let sum: f32 = data[start..=i].iter().sum();
        let avg = sum / (i - start + 1) as f32;
        aggregated.push(avg);
    }
    
    aggregated
}

#[cfg(feature = "data-filtering")]
fn transform_data(data: &[f32]) -> Vec<f32> {
    // 数据转换 - 归一化
    let max_val = data.iter().fold(0.0f32, |acc, &x| acc.max(x));
    let min_val = data.iter().fold(f32::INFINITY, |acc, &x| acc.min(x));
    let range = max_val - min_val;
    
    if range == 0.0 {
        vec![0.0; data.len()]
    } else {
        data.iter().map(|&x| (x - min_val) / range).collect()
    }
}

#[cfg(feature = "data-filtering")]
criterion_group!(benches, benchmark_data_processing);

#[cfg(not(feature = "data-filtering"))]
fn benchmark_disabled(_c: &mut Criterion) {
    // 当特性未启用时的占位符
}

#[cfg(not(feature = "data-filtering"))]
criterion_group!(benches, benchmark_disabled);

criterion_main!(benches);