#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::peripheral::DWT;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    timer::Timer,
};
use heapless::Vec;
use micromath::F32Ext;

// 性能测试结果
#[derive(Debug, Clone, Copy)]
struct BenchmarkResult {
    name: &'static str,
    cycles: u32,
    iterations: u32,
    cycles_per_iteration: f32,
}

// 算法优化示例
struct AlgorithmOptimization;

impl AlgorithmOptimization {
    // 未优化的冒泡排序
    fn bubble_sort_naive(data: &mut [u32]) {
        let len = data.len();
        for i in 0..len {
            for j in 0..len - 1 - i {
                if data[j] > data[j + 1] {
                    data.swap(j, j + 1);
                }
            }
        }
    }
    
    // 优化的冒泡排序（提前退出）
    fn bubble_sort_optimized(data: &mut [u32]) {
        let len = data.len();
        for i in 0..len {
            let mut swapped = false;
            for j in 0..len - 1 - i {
                if data[j] > data[j + 1] {
                    data.swap(j, j + 1);
                    swapped = true;
                }
            }
            if !swapped {
                break; // 已经排序完成
            }
        }
    }
    
    // 快速排序
    fn quick_sort(data: &mut [u32]) {
        if data.len() <= 1 {
            return;
        }
        
        let pivot_index = Self::partition(data);
        let (left, right) = data.split_at_mut(pivot_index);
        
        Self::quick_sort(left);
        if right.len() > 1 {
            Self::quick_sort(&mut right[1..]);
        }
    }
    
    fn partition(data: &mut [u32]) -> usize {
        let len = data.len();
        let pivot_index = len / 2;
        data.swap(pivot_index, len - 1);
        
        let pivot = data[len - 1];
        let mut i = 0;
        
        for j in 0..len - 1 {
            if data[j] <= pivot {
                data.swap(i, j);
                i += 1;
            }
        }
        
        data.swap(i, len - 1);
        i
    }
    
    // 插入排序（对小数组优化）
    fn insertion_sort(data: &mut [u32]) {
        for i in 1..data.len() {
            let key = data[i];
            let mut j = i;
            
            while j > 0 && data[j - 1] > key {
                data[j] = data[j - 1];
                j -= 1;
            }
            
            data[j] = key;
        }
    }
    
    // 混合排序（大数组用快排，小数组用插入排序）
    fn hybrid_sort(data: &mut [u32]) {
        const THRESHOLD: usize = 16;
        
        if data.len() <= THRESHOLD {
            Self::insertion_sort(data);
        } else {
            Self::quick_sort(data);
        }
    }
    
    // 二分查找
    fn binary_search(data: &[u32], target: u32) -> Option<usize> {
        let mut left = 0;
        let mut right = data.len();
        
        while left < right {
            let mid = left + (right - left) / 2;
            
            match data[mid].cmp(&target) {
                core::cmp::Ordering::Equal => return Some(mid),
                core::cmp::Ordering::Less => left = mid + 1,
                core::cmp::Ordering::Greater => right = mid,
            }
        }
        
        None
    }
    
    // 线性查找
    fn linear_search(data: &[u32], target: u32) -> Option<usize> {
        for (index, &value) in data.iter().enumerate() {
            if value == target {
                return Some(index);
            }
        }
        None
    }
}

// 数学运算优化
struct MathOptimization;

impl MathOptimization {
    // 未优化的平方根计算
    fn sqrt_naive(x: f32) -> f32 {
        if x < 0.0 {
            return f32::NAN;
        }
        if x == 0.0 {
            return 0.0;
        }
        
        let mut guess = x / 2.0;
        for _ in 0..20 {
            guess = (guess + x / guess) / 2.0;
        }
        guess
    }
    
    // 优化的平方根计算（牛顿法 + 快速倒数平方根）
    fn sqrt_optimized(x: f32) -> f32 {
        if x < 0.0 {
            return f32::NAN;
        }
        if x == 0.0 {
            return 0.0;
        }
        
        // 使用快速倒数平方根作为初始猜测
        let mut guess = Self::fast_inv_sqrt(x) * x;
        
        // 牛顿法迭代（减少迭代次数）
        for _ in 0..3 {
            guess = (guess + x / guess) / 2.0;
        }
        
        guess
    }
    
    // 快速倒数平方根（Quake算法的简化版本）
    fn fast_inv_sqrt(x: f32) -> f32 {
        let i = x.to_bits();
        let i = 0x5f3759df - (i >> 1);
        let y = f32::from_bits(i);
        
        // 一次牛顿迭代
        y * (1.5 - 0.5 * x * y * y)
    }
    
    // 三角函数查找表
    const SIN_TABLE: [f32; 256] = {
        let mut table = [0.0; 256];
        let mut i = 0;
        while i < 256 {
            let angle = (i as f32) * 2.0 * core::f32::consts::PI / 256.0;
            table[i] = libm::sinf(angle);
            i += 1;
        }
        table
    };
    
    fn sin_lookup(angle: f32) -> f32 {
        let normalized = angle / (2.0 * core::f32::consts::PI);
        let index = ((normalized.fract() * 256.0) as usize) % 256;
        Self::SIN_TABLE[index]
    }
    
    // 整数幂运算优化
    fn power_naive(base: u32, exp: u32) -> u32 {
        let mut result = 1;
        for _ in 0..exp {
            result *= base;
        }
        result
    }
    
    fn power_optimized(mut base: u32, mut exp: u32) -> u32 {
        let mut result = 1;
        
        while exp > 0 {
            if exp & 1 == 1 {
                result *= base;
            }
            base *= base;
            exp >>= 1;
        }
        
        result
    }
    
    // 定点数运算
    #[derive(Debug, Clone, Copy)]
    struct FixedPoint {
        value: i32,
    }
    
    impl FixedPoint {
        const FRACTIONAL_BITS: u32 = 16;
        const SCALE: i32 = 1 << Self::FRACTIONAL_BITS;
        
        fn from_float(f: f32) -> Self {
            Self {
                value: (f * Self::SCALE as f32) as i32,
            }
        }
        
        fn to_float(self) -> f32 {
            self.value as f32 / Self::SCALE as f32
        }
        
        fn multiply(self, other: Self) -> Self {
            Self {
                value: ((self.value as i64 * other.value as i64) >> Self::FRACTIONAL_BITS) as i32,
            }
        }
        
        fn divide(self, other: Self) -> Self {
            Self {
                value: ((self.value as i64) << Self::FRACTIONAL_BITS / other.value as i64) as i32,
            }
        }
    }
}

// 内存访问优化
struct MemoryOptimization;

impl MemoryOptimization {
    // 缓存友好的矩阵乘法
    fn matrix_multiply_naive(a: &[[f32; 64]; 64], b: &[[f32; 64]; 64], c: &mut [[f32; 64]; 64]) {
        for i in 0..64 {
            for j in 0..64 {
                c[i][j] = 0.0;
                for k in 0..64 {
                    c[i][j] += a[i][k] * b[k][j];
                }
            }
        }
    }
    
    fn matrix_multiply_optimized(a: &[[f32; 64]; 64], b: &[[f32; 64]; 64], c: &mut [[f32; 64]; 64]) {
        const BLOCK_SIZE: usize = 8;
        
        // 分块矩阵乘法，提高缓存命中率
        for ii in (0..64).step_by(BLOCK_SIZE) {
            for jj in (0..64).step_by(BLOCK_SIZE) {
                for kk in (0..64).step_by(BLOCK_SIZE) {
                    for i in ii..core::cmp::min(ii + BLOCK_SIZE, 64) {
                        for j in jj..core::cmp::min(jj + BLOCK_SIZE, 64) {
                            let mut sum = c[i][j];
                            for k in kk..core::cmp::min(kk + BLOCK_SIZE, 64) {
                                sum += a[i][k] * b[k][j];
                            }
                            c[i][j] = sum;
                        }
                    }
                }
            }
        }
    }
    
    // 数据预取优化
    fn sum_array_with_prefetch(data: &[u32]) -> u32 {
        let mut sum = 0;
        const PREFETCH_DISTANCE: usize = 8;
        
        for i in 0..data.len() {
            // 预取未来的数据
            if i + PREFETCH_DISTANCE < data.len() {
                unsafe {
                    let ptr = data.as_ptr().add(i + PREFETCH_DISTANCE);
                    #[cfg(target_arch = "arm")]
                    core::arch::asm!("pld [{}]", in(reg) ptr);
                }
            }
            
            sum += data[i];
        }
        
        sum
    }
    
    // 内存对齐优化
    #[repr(C, align(32))]
    struct AlignedBuffer {
        data: [u8; 1024],
    }
    
    impl AlignedBuffer {
        fn new() -> Self {
            Self {
                data: [0; 1024],
            }
        }
        
        fn process_aligned(&mut self) {
            // 对齐的内存访问更高效
            for chunk in self.data.chunks_exact_mut(4) {
                let value = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                let processed = value.wrapping_mul(2);
                let bytes = processed.to_le_bytes();
                chunk.copy_from_slice(&bytes);
            }
        }
    }
}

// 编译器优化示例
struct CompilerOptimization;

impl CompilerOptimization {
    // 内联函数优化
    #[inline(always)]
    fn fast_add(a: u32, b: u32) -> u32 {
        a.wrapping_add(b)
    }
    
    #[inline(never)]
    fn slow_add(a: u32, b: u32) -> u32 {
        a.wrapping_add(b)
    }
    
    // 循环展开
    fn sum_unrolled(data: &[u32]) -> u32 {
        let mut sum = 0;
        let mut i = 0;
        
        // 4路循环展开
        while i + 4 <= data.len() {
            sum += data[i] + data[i + 1] + data[i + 2] + data[i + 3];
            i += 4;
        }
        
        // 处理剩余元素
        while i < data.len() {
            sum += data[i];
            i += 1;
        }
        
        sum
    }
    
    // 分支预测优化
    fn process_with_likely(data: &[i32]) -> i32 {
        let mut sum = 0;
        
        for &value in data {
            // 假设正数更常见
            if likely(value > 0) {
                sum += value;
            } else {
                sum -= value;
            }
        }
        
        sum
    }
    
    // 常量传播优化
    const MULTIPLIER: u32 = 42;
    
    fn constant_multiply(x: u32) -> u32 {
        x * Self::MULTIPLIER // 编译器会优化为移位和加法
    }
    
    // 死代码消除示例
    fn dead_code_example(x: u32) -> u32 {
        let _unused = x * 2; // 死代码，会被编译器消除
        x + 1
    }
}

// 分支预测提示
#[inline(always)]
fn likely(b: bool) -> bool {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::intrinsics::likely(b)
    }
    #[cfg(not(target_arch = "arm"))]
    b
}

#[inline(always)]
fn unlikely(b: bool) -> bool {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::intrinsics::unlikely(b)
    }
    #[cfg(not(target_arch = "arm"))]
    b
}

// 性能测试框架
struct PerformanceTester {
    results: Vec<BenchmarkResult, 32>,
}

impl PerformanceTester {
    fn new() -> Self {
        Self {
            results: Vec::new(),
        }
    }
    
    fn benchmark<F>(&mut self, name: &'static str, iterations: u32, mut func: F) 
    where
        F: FnMut(),
    {
        // 预热
        for _ in 0..10 {
            func();
        }
        
        // 测量
        let start_cycles = DWT::cycle_count();
        
        for _ in 0..iterations {
            func();
        }
        
        let end_cycles = DWT::cycle_count();
        let total_cycles = end_cycles.wrapping_sub(start_cycles);
        
        let result = BenchmarkResult {
            name,
            cycles: total_cycles,
            iterations,
            cycles_per_iteration: total_cycles as f32 / iterations as f32,
        };
        
        self.results.push(result).ok();
        
        cortex_m_log::println!(
            "Benchmark {}: {} cycles total, {:.2} cycles/iter",
            name,
            total_cycles,
            result.cycles_per_iteration
        );
    }
    
    fn print_summary(&self) {
        cortex_m_log::println!("\n=== Performance Summary ===");
        
        for result in &self.results {
            cortex_m_log::println!(
                "{}: {:.2} cycles/iter",
                result.name,
                result.cycles_per_iteration
            );
        }
    }
    
    fn compare_results(&self, baseline: &str, optimized: &str) {
        let baseline_result = self.results.iter().find(|r| r.name == baseline);
        let optimized_result = self.results.iter().find(|r| r.name == optimized);
        
        if let (Some(base), Some(opt)) = (baseline_result, optimized_result) {
            let speedup = base.cycles_per_iteration / opt.cycles_per_iteration;
            let improvement = (1.0 - opt.cycles_per_iteration / base.cycles_per_iteration) * 100.0;
            
            cortex_m_log::println!(
                "Optimization: {} -> {}: {:.2}x speedup ({:.1}% improvement)",
                baseline,
                optimized,
                speedup,
                improvement
            );
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置系统时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();
    
    // 启用DWT用于性能测试
    let mut dwt = cp.DWT;
    dwt.enable_cycle_counter();
    
    cortex_m_log::println!("Starting performance optimization tests...");
    
    let mut tester = PerformanceTester::new();
    
    // 测试排序算法
    test_sorting_algorithms(&mut tester);
    
    // 测试数学运算
    test_math_operations(&mut tester);
    
    // 测试内存访问
    test_memory_access(&mut tester);
    
    // 测试编译器优化
    test_compiler_optimizations(&mut tester);
    
    // 打印总结
    tester.print_summary();
    
    // 比较优化效果
    tester.compare_results("bubble_sort_naive", "bubble_sort_optimized");
    tester.compare_results("bubble_sort_naive", "quick_sort");
    tester.compare_results("sqrt_naive", "sqrt_optimized");
    tester.compare_results("power_naive", "power_optimized");
    
    cortex_m_log::println!("Performance tests completed!");
    
    loop {
        cortex_m::asm::wfi();
    }
}

fn test_sorting_algorithms(tester: &mut PerformanceTester) {
    cortex_m_log::println!("\n=== Testing Sorting Algorithms ===");
    
    let test_data = [64, 34, 25, 12, 22, 11, 90, 88, 76, 50, 42, 30, 5, 77, 55, 20];
    
    // 冒泡排序（未优化）
    tester.benchmark("bubble_sort_naive", 100, || {
        let mut data = test_data;
        AlgorithmOptimization::bubble_sort_naive(&mut data);
    });
    
    // 冒泡排序（优化）
    tester.benchmark("bubble_sort_optimized", 100, || {
        let mut data = test_data;
        AlgorithmOptimization::bubble_sort_optimized(&mut data);
    });
    
    // 快速排序
    tester.benchmark("quick_sort", 100, || {
        let mut data = test_data;
        AlgorithmOptimization::quick_sort(&mut data);
    });
    
    // 插入排序
    tester.benchmark("insertion_sort", 100, || {
        let mut data = test_data;
        AlgorithmOptimization::insertion_sort(&mut data);
    });
    
    // 混合排序
    tester.benchmark("hybrid_sort", 100, || {
        let mut data = test_data;
        AlgorithmOptimization::hybrid_sort(&mut data);
    });
}

fn test_math_operations(tester: &mut PerformanceTester) {
    cortex_m_log::println!("\n=== Testing Math Operations ===");
    
    // 平方根计算
    tester.benchmark("sqrt_naive", 1000, || {
        let _ = MathOptimization::sqrt_naive(2.0);
    });
    
    tester.benchmark("sqrt_optimized", 1000, || {
        let _ = MathOptimization::sqrt_optimized(2.0);
    });
    
    // 幂运算
    tester.benchmark("power_naive", 1000, || {
        let _ = MathOptimization::power_naive(2, 10);
    });
    
    tester.benchmark("power_optimized", 1000, || {
        let _ = MathOptimization::power_optimized(2, 10);
    });
    
    // 三角函数
    tester.benchmark("sin_libm", 1000, || {
        let _ = libm::sinf(1.0);
    });
    
    tester.benchmark("sin_lookup", 1000, || {
        let _ = MathOptimization::sin_lookup(1.0);
    });
}

fn test_memory_access(tester: &mut PerformanceTester) {
    cortex_m_log::println!("\n=== Testing Memory Access ===");
    
    let data = [1u32; 1000];
    
    // 数组求和
    tester.benchmark("sum_normal", 100, || {
        let mut sum = 0;
        for &value in data.iter() {
            sum += value;
        }
    });
    
    tester.benchmark("sum_with_prefetch", 100, || {
        let _ = MemoryOptimization::sum_array_with_prefetch(&data);
    });
    
    // 内存对齐测试
    let mut aligned_buffer = MemoryOptimization::AlignedBuffer::new();
    
    tester.benchmark("aligned_processing", 100, || {
        aligned_buffer.process_aligned();
    });
}

fn test_compiler_optimizations(tester: &mut PerformanceTester) {
    cortex_m_log::println!("\n=== Testing Compiler Optimizations ===");
    
    // 内联函数测试
    tester.benchmark("inline_add", 10000, || {
        let _ = CompilerOptimization::fast_add(1, 2);
    });
    
    tester.benchmark("no_inline_add", 10000, || {
        let _ = CompilerOptimization::slow_add(1, 2);
    });
    
    // 循环展开测试
    let data = [1u32; 100];
    
    tester.benchmark("sum_normal_loop", 1000, || {
        let mut sum = 0;
        for &value in data.iter() {
            sum += value;
        }
    });
    
    tester.benchmark("sum_unrolled", 1000, || {
        let _ = CompilerOptimization::sum_unrolled(&data);
    });
    
    // 常量优化测试
    tester.benchmark("constant_multiply", 10000, || {
        let _ = CompilerOptimization::constant_multiply(123);
    });
}