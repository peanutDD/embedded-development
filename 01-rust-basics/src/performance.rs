//! # 性能优化工具
//! 
//! 本模块提供了各种性能优化工具和技术。
//! 
//! ## 特性
//! 
//! - **内存管理**: 内存池、对象池等
//! - **缓存优化**: 缓存友好的数据结构
//! - **性能测量**: 基准测试和性能分析
//! - **算法优化**: 高效算法实现

use std::time::{Duration, Instant};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::marker::PhantomData;
use crate::error_handling::{Error, Result};

/// 内存池
pub struct MemoryPool<T> {
    pool: Vec<T>,
    capacity: usize,
}

impl<T: Default> MemoryPool<T> {
    /// 创建新的内存池
    pub fn new(capacity: usize) -> Self {
        let mut pool = Vec::with_capacity(capacity);
        for _ in 0..capacity {
            pool.push(T::default());
        }
        
        Self { pool, capacity }
    }
    
    /// 从池中获取对象
    pub fn acquire(&mut self) -> Option<T> {
        self.pool.pop()
    }
    
    /// 将对象返回到池中
    pub fn release(&mut self, item: T) {
        if self.pool.len() < self.capacity {
            self.pool.push(item);
        }
    }
    
    /// 获取池的容量
    pub fn capacity(&self) -> usize {
        self.capacity
    }
    
    /// 获取当前可用对象数量
    pub fn available(&self) -> usize {
        self.pool.len()
    }
    
    /// 检查池是否为空
    pub fn is_empty(&self) -> bool {
        self.pool.is_empty()
    }
    
    /// 检查池是否已满
    pub fn is_full(&self) -> bool {
        self.pool.len() == self.capacity
    }
}

/// 对象池（线程安全）
pub struct ObjectPool<T> {
    pool: Arc<Mutex<Vec<T>>>,
    factory: Box<dyn Fn() -> T + Send + Sync>,
    capacity: usize,
}

impl<T: Send + 'static> ObjectPool<T> {
    /// 创建新的对象池
    pub fn new<F>(capacity: usize, factory: F) -> Self
    where
        F: Fn() -> T + Send + Sync + 'static,
    {
        Self {
            pool: Arc::new(Mutex::new(Vec::new())),
            factory: Box::new(factory),
            capacity,
        }
    }
    
    /// 从池中获取对象
    pub fn acquire(&self) -> T {
        let mut pool = self.pool.lock().unwrap();
        pool.pop().unwrap_or_else(|| (self.factory)())
    }
    
    /// 将对象返回到池中
    pub fn release(&self, item: T) {
        let mut pool = self.pool.lock().unwrap();
        if pool.len() < self.capacity {
            pool.push(item);
        }
    }
    
    /// 获取当前可用对象数量
    pub fn available(&self) -> usize {
        self.pool.lock().unwrap().len()
    }
}

/// 缓存友好的向量
pub struct CacheFriendlyVec<T> {
    data: Vec<T>,
    chunk_size: usize,
}

impl<T> CacheFriendlyVec<T> {
    /// 创建新的缓存友好向量
    pub fn new(chunk_size: usize) -> Self {
        Self {
            data: Vec::new(),
            chunk_size,
        }
    }
    
    /// 添加元素
    pub fn push(&mut self, item: T) {
        self.data.push(item);
    }
    
    /// 按块处理数据
    pub fn process_chunks<F>(&self, mut processor: F)
    where
        F: FnMut(&[T]),
    {
        for chunk in self.data.chunks(self.chunk_size) {
            processor(chunk);
        }
    }
    
    /// 获取长度
    pub fn len(&self) -> usize {
        self.data.len()
    }
    
    /// 检查是否为空
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
    
    /// 获取块大小
    pub fn chunk_size(&self) -> usize {
        self.chunk_size
    }
}

/// 性能计时器
pub struct Timer {
    start: Instant,
    name: String,
}

impl Timer {
    /// 创建新的计时器
    pub fn new(name: &str) -> Self {
        Self {
            start: Instant::now(),
            name: name.to_string(),
        }
    }
    
    /// 获取经过的时间
    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }
    
    /// 重置计时器
    pub fn reset(&mut self) {
        self.start = Instant::now();
    }
    
    /// 获取计时器名称
    pub fn name(&self) -> &str {
        &self.name
    }
}

impl Drop for Timer {
    fn drop(&mut self) {
        println!("Timer '{}': {:?}", self.name, self.elapsed());
    }
}

/// 基准测试工具
pub struct Benchmark {
    measurements: Vec<Duration>,
    name: String,
}

impl Benchmark {
    /// 创建新的基准测试
    pub fn new(name: &str) -> Self {
        Self {
            measurements: Vec::new(),
            name: name.to_string(),
        }
    }
    
    /// 运行基准测试
    pub fn run<F>(&mut self, iterations: usize, mut test_fn: F)
    where
        F: FnMut(),
    {
        for _ in 0..iterations {
            let start = Instant::now();
            test_fn();
            let duration = start.elapsed();
            self.measurements.push(duration);
        }
    }
    
    /// 获取平均时间
    pub fn average(&self) -> Duration {
        if self.measurements.is_empty() {
            return Duration::from_nanos(0);
        }
        
        let total: Duration = self.measurements.iter().sum();
        total / self.measurements.len() as u32
    }
    
    /// 获取最小时间
    pub fn min(&self) -> Duration {
        self.measurements.iter().min().copied().unwrap_or_default()
    }
    
    /// 获取最大时间
    pub fn max(&self) -> Duration {
        self.measurements.iter().max().copied().unwrap_or_default()
    }
    
    /// 获取标准差
    pub fn std_dev(&self) -> Duration {
        if self.measurements.len() < 2 {
            return Duration::from_nanos(0);
        }
        
        let avg = self.average();
        let variance: f64 = self.measurements
            .iter()
            .map(|&x| {
                let diff = x.as_nanos() as f64 - avg.as_nanos() as f64;
                diff * diff
            })
            .sum::<f64>() / (self.measurements.len() - 1) as f64;
        
        Duration::from_nanos(variance.sqrt() as u64)
    }
    
    /// 打印统计信息
    pub fn print_stats(&self) {
        println!("Benchmark '{}' results:", self.name);
        println!("  Iterations: {}", self.measurements.len());
        println!("  Average: {:?}", self.average());
        println!("  Min: {:?}", self.min());
        println!("  Max: {:?}", self.max());
        println!("  Std Dev: {:?}", self.std_dev());
    }
}

/// 性能分析器
pub struct Profiler {
    timers: HashMap<String, Vec<Duration>>,
}

impl Profiler {
    /// 创建新的性能分析器
    pub fn new() -> Self {
        Self {
            timers: HashMap::new(),
        }
    }
    
    /// 开始计时
    pub fn start_timer(&self, name: &str) -> ProfilerGuard {
        ProfilerGuard::new(name.to_string())
    }
    
    /// 记录时间
    pub fn record(&mut self, name: &str, duration: Duration) {
        self.timers.entry(name.to_string()).or_insert_with(Vec::new).push(duration);
    }
    
    /// 获取统计信息
    pub fn get_stats(&self, name: &str) -> Option<ProfileStats> {
        self.timers.get(name).map(|durations| {
            let count = durations.len();
            let total: Duration = durations.iter().sum();
            let average = if count > 0 { total / count as u32 } else { Duration::from_nanos(0) };
            let min = durations.iter().min().copied().unwrap_or_default();
            let max = durations.iter().max().copied().unwrap_or_default();
            
            ProfileStats {
                name: name.to_string(),
                count,
                total,
                average,
                min,
                max,
            }
        })
    }
    
    /// 打印所有统计信息
    pub fn print_all_stats(&self) {
        for name in self.timers.keys() {
            if let Some(stats) = self.get_stats(name) {
                stats.print();
            }
        }
    }
    
    /// 清除所有数据
    pub fn clear(&mut self) {
        self.timers.clear();
    }
}

impl Default for Profiler {
    fn default() -> Self {
        Self::new()
    }
}

/// 性能分析器守卫
pub struct ProfilerGuard {
    name: String,
    start: Instant,
}

impl ProfilerGuard {
    fn new(name: String) -> Self {
        Self {
            name,
            start: Instant::now(),
        }
    }
    
    /// 获取经过的时间
    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }
}

/// 性能统计信息
pub struct ProfileStats {
    pub name: String,
    pub count: usize,
    pub total: Duration,
    pub average: Duration,
    pub min: Duration,
    pub max: Duration,
}

impl ProfileStats {
    /// 打印统计信息
    pub fn print(&self) {
        println!("Profile '{}' stats:", self.name);
        println!("  Count: {}", self.count);
        println!("  Total: {:?}", self.total);
        println!("  Average: {:?}", self.average);
        println!("  Min: {:?}", self.min);
        println!("  Max: {:?}", self.max);
    }
}

/// 循环展开宏
#[macro_export]
macro_rules! unroll {
    ($n:expr, $body:block) => {
        {
            const N: usize = $n;
            let mut i = 0;
            while i < N {
                $body
                i += 1;
            }
        }
    };
}

/// 性能优化工具函数
pub mod utils {
    use super::*;
    
    /// 预取内存
    #[inline(always)]
    pub fn prefetch<T>(ptr: *const T) {
        #[cfg(target_arch = "x86_64")]
        unsafe {
            std::arch::x86_64::_mm_prefetch(ptr as *const i8, std::arch::x86_64::_MM_HINT_T0);
        }
        
        #[cfg(not(target_arch = "x86_64"))]
        {
            // 在其他架构上，这是一个空操作
            let _ = ptr;
        }
    }
    
    /// 分支预测提示
    #[inline(always)]
    pub fn likely(condition: bool) -> bool {
        #[cfg(target_feature = "likely")]
        {
            std::intrinsics::likely(condition)
        }
        #[cfg(not(target_feature = "likely"))]
        {
            condition
        }
    }
    
    /// 分支预测提示（不太可能）
    #[inline(always)]
    pub fn unlikely(condition: bool) -> bool {
        #[cfg(target_feature = "likely")]
        {
            std::intrinsics::unlikely(condition)
        }
        #[cfg(not(target_feature = "likely"))]
        {
            condition
        }
    }
    
    /// 快速整数平方根
    pub fn fast_sqrt(n: u32) -> u32 {
        if n == 0 {
            return 0;
        }
        
        let mut x = n;
        let mut y = (x + 1) / 2;
        
        while y < x {
            x = y;
            y = (x + n / x) / 2;
        }
        
        x
    }
    
    /// 快速幂运算
    pub fn fast_pow(mut base: u64, mut exp: u64) -> u64 {
        let mut result = 1;
        
        while exp > 0 {
            if exp % 2 == 1 {
                result *= base;
            }
            base *= base;
            exp /= 2;
        }
        
        result
    }
    
    /// 位计数（popcount）
    pub fn popcount(mut n: u64) -> u32 {
        let mut count = 0;
        while n != 0 {
            count += 1;
            n &= n - 1; // 清除最低位的1
        }
        count
    }
    
    /// 前导零计数
    pub fn leading_zeros(n: u64) -> u32 {
        n.leading_zeros()
    }
    
    /// 尾随零计数
    pub fn trailing_zeros(n: u64) -> u32 {
        n.trailing_zeros()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;
    
    #[test]
    fn test_memory_pool() {
        let mut pool = MemoryPool::<i32>::new(5);
        
        assert_eq!(pool.capacity(), 5);
        assert_eq!(pool.available(), 5);
        
        let item = pool.acquire().unwrap();
        assert_eq!(pool.available(), 4);
        
        pool.release(item);
        assert_eq!(pool.available(), 5);
    }
    
    #[test]
    fn test_object_pool() {
        let pool = ObjectPool::new(3, || vec![1, 2, 3]);
        
        let obj1 = pool.acquire();
        let obj2 = pool.acquire();
        
        assert_eq!(obj1, vec![1, 2, 3]);
        assert_eq!(obj2, vec![1, 2, 3]);
        
        pool.release(obj1);
        assert_eq!(pool.available(), 1);
    }
    
    #[test]
    fn test_cache_friendly_vec() {
        let mut vec = CacheFriendlyVec::new(2);
        
        vec.push(1);
        vec.push(2);
        vec.push(3);
        vec.push(4);
        
        let mut chunks = Vec::new();
        vec.process_chunks(|chunk| {
            chunks.push(chunk.to_vec());
        });
        
        assert_eq!(chunks, vec![vec![1, 2], vec![3, 4]]);
    }
    
    #[test]
    fn test_timer() {
        let timer = Timer::new("test");
        thread::sleep(Duration::from_millis(10));
        
        let elapsed = timer.elapsed();
        assert!(elapsed >= Duration::from_millis(10));
    }
    
    #[test]
    fn test_benchmark() {
        let mut bench = Benchmark::new("test");
        
        bench.run(5, || {
            thread::sleep(Duration::from_millis(1));
        });
        
        let avg = bench.average();
        assert!(avg >= Duration::from_millis(1));
        
        bench.print_stats();
    }
    
    #[test]
    fn test_profiler() {
        let mut profiler = Profiler::new();
        
        {
            let _guard = profiler.start_timer("test");
            thread::sleep(Duration::from_millis(5));
            profiler.record("test", Duration::from_millis(5));
        }
        
        let stats = profiler.get_stats("test").unwrap();
        assert_eq!(stats.count, 1);
        assert!(stats.total >= Duration::from_millis(5));
    }
    
    #[test]
    fn test_utils() {
        use utils::*;
        
        assert_eq!(fast_sqrt(16), 4);
        assert_eq!(fast_sqrt(25), 5);
        
        assert_eq!(fast_pow(2, 10), 1024);
        assert_eq!(fast_pow(3, 4), 81);
        
        assert_eq!(popcount(0b1010101), 4);
        assert_eq!(popcount(0b1111), 4);
        
        assert_eq!(leading_zeros(0b1000), 60); // 64 - 4
        assert_eq!(trailing_zeros(0b1000), 3);
    }
}