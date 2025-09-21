#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::peripheral::DWT;
use stm32f4xx_hal::{
    pac,
    prelude::*,
};
use heapless::{Vec, pool::{Pool, Node}};
use core::mem::{size_of, align_of, MaybeUninit};

// 内存统计信息
#[derive(Debug, Clone, Copy)]
struct MemoryStats {
    total_allocations: u32,
    total_deallocations: u32,
    current_allocations: u32,
    peak_allocations: u32,
    total_bytes_allocated: u32,
    peak_bytes_allocated: u32,
    current_bytes_allocated: u32,
    fragmentation_ratio: f32,
}

impl MemoryStats {
    fn new() -> Self {
        Self {
            total_allocations: 0,
            total_deallocations: 0,
            current_allocations: 0,
            peak_allocations: 0,
            total_bytes_allocated: 0,
            peak_bytes_allocated: 0,
            current_bytes_allocated: 0,
            fragmentation_ratio: 0.0,
        }
    }
    
    fn record_allocation(&mut self, size: u32) {
        self.total_allocations += 1;
        self.current_allocations += 1;
        self.total_bytes_allocated += size;
        self.current_bytes_allocated += size;
        
        if self.current_allocations > self.peak_allocations {
            self.peak_allocations = self.current_allocations;
        }
        
        if self.current_bytes_allocated > self.peak_bytes_allocated {
            self.peak_bytes_allocated = self.current_bytes_allocated;
        }
    }
    
    fn record_deallocation(&mut self, size: u32) {
        self.total_deallocations += 1;
        self.current_allocations = self.current_allocations.saturating_sub(1);
        self.current_bytes_allocated = self.current_bytes_allocated.saturating_sub(size);
    }
    
    fn calculate_fragmentation(&mut self, free_blocks: u32, largest_free_block: u32) {
        if free_blocks > 0 && largest_free_block > 0 {
            self.fragmentation_ratio = 1.0 - (largest_free_block as f32 / self.current_bytes_allocated as f32);
        }
    }
}

// 静态内存分配器
struct StaticAllocator {
    memory: [u8; 4096],
    offset: usize,
    stats: MemoryStats,
}

impl StaticAllocator {
    fn new() -> Self {
        Self {
            memory: [0; 4096],
            offset: 0,
            stats: MemoryStats::new(),
        }
    }
    
    fn allocate(&mut self, size: usize, align: usize) -> Option<*mut u8> {
        // 对齐偏移量
        let aligned_offset = (self.offset + align - 1) & !(align - 1);
        
        if aligned_offset + size <= self.memory.len() {
            let ptr = unsafe { self.memory.as_mut_ptr().add(aligned_offset) };
            self.offset = aligned_offset + size;
            self.stats.record_allocation(size as u32);
            Some(ptr)
        } else {
            None
        }
    }
    
    fn reset(&mut self) {
        self.offset = 0;
        self.stats = MemoryStats::new();
    }
    
    fn get_stats(&self) -> MemoryStats {
        self.stats
    }
    
    fn get_usage(&self) -> f32 {
        self.offset as f32 / self.memory.len() as f32
    }
}

// 内存池管理器
struct MemoryPoolManager {
    small_pool: Pool<Node<[u8; 32]>>,
    medium_pool: Pool<Node<[u8; 128]>>,
    large_pool: Pool<Node<[u8; 512]>>,
    small_memory: [Node<[u8; 32]>; 32],
    medium_memory: [Node<[u8; 128]>; 16],
    large_memory: [Node<[u8; 512]>; 8],
    stats: MemoryStats,
}

impl MemoryPoolManager {
    fn new() -> Self {
        Self {
            small_pool: Pool::new(),
            medium_pool: Pool::new(),
            large_pool: Pool::new(),
            small_memory: [Node::new(); 32],
            medium_memory: [Node::new(); 16],
            large_memory: [Node::new(); 8],
            stats: MemoryStats::new(),
        }
    }
    
    fn init(&mut self) {
        // 初始化内存池
        for node in &mut self.small_memory {
            self.small_pool.manage(node);
        }
        
        for node in &mut self.medium_memory {
            self.medium_pool.manage(node);
        }
        
        for node in &mut self.large_memory {
            self.large_pool.manage(node);
        }
    }
    
    fn allocate(&mut self, size: usize) -> Option<&'static mut [u8]> {
        let result = if size <= 32 {
            self.small_pool.alloc().map(|node| {
                self.stats.record_allocation(32);
                &mut node[..size]
            })
        } else if size <= 128 {
            self.medium_pool.alloc().map(|node| {
                self.stats.record_allocation(128);
                &mut node[..size]
            })
        } else if size <= 512 {
            self.large_pool.alloc().map(|node| {
                self.stats.record_allocation(512);
                &mut node[..size]
            })
        } else {
            None
        };
        
        result
    }
    
    fn get_stats(&self) -> MemoryStats {
        self.stats
    }
    
    fn get_pool_usage(&self) -> (f32, f32, f32) {
        let small_used = 32 - self.small_pool.free_count();
        let medium_used = 16 - self.medium_pool.free_count();
        let large_used = 8 - self.large_pool.free_count();
        
        (
            small_used as f32 / 32.0,
            medium_used as f32 / 16.0,
            large_used as f32 / 8.0,
        )
    }
}

// 栈使用分析器
struct StackAnalyzer {
    stack_start: *const u8,
    stack_size: usize,
    max_usage: usize,
    pattern: u32,
}

impl StackAnalyzer {
    fn new(stack_start: *const u8, stack_size: usize) -> Self {
        Self {
            stack_start,
            stack_size,
            max_usage: 0,
            pattern: 0xDEADBEEF,
        }
    }
    
    fn fill_stack_pattern(&self) {
        unsafe {
            let stack_ptr = self.stack_start as *mut u32;
            let words = self.stack_size / 4;
            
            for i in 0..words {
                stack_ptr.add(i).write_volatile(self.pattern);
            }
        }
    }
    
    fn measure_usage(&mut self) -> usize {
        let mut used = 0;
        
        unsafe {
            let stack_ptr = self.stack_start as *const u32;
            let words = self.stack_size / 4;
            
            for i in 0..words {
                if stack_ptr.add(i).read_volatile() != self.pattern {
                    used = (i + 1) * 4;
                }
            }
        }
        
        if used > self.max_usage {
            self.max_usage = used;
        }
        
        used
    }
    
    fn get_max_usage(&self) -> usize {
        self.max_usage
    }
    
    fn get_usage_percentage(&self) -> f32 {
        self.max_usage as f32 / self.stack_size as f32 * 100.0
    }
}

// 缓存优化数据结构
#[repr(C, align(32))]
struct CacheAlignedBuffer<T, const N: usize> {
    data: [T; N],
}

impl<T: Copy + Default, const N: usize> CacheAlignedBuffer<T, N> {
    fn new() -> Self {
        Self {
            data: [T::default(); N],
        }
    }
    
    fn get(&self, index: usize) -> Option<&T> {
        self.data.get(index)
    }
    
    fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        self.data.get_mut(index)
    }
    
    fn len(&self) -> usize {
        N
    }
    
    // 缓存友好的批量操作
    fn batch_process<F>(&mut self, mut func: F) 
    where
        F: FnMut(&mut T),
    {
        const CACHE_LINE_SIZE: usize = 32;
        let elements_per_line = CACHE_LINE_SIZE / size_of::<T>();
        
        for chunk in self.data.chunks_mut(elements_per_line) {
            for item in chunk {
                func(item);
            }
        }
    }
}

// DMA内存管理
#[repr(C, align(4))]
struct DmaBuffer<T, const N: usize> {
    data: [MaybeUninit<T>; N],
    initialized: usize,
}

impl<T: Copy, const N: usize> DmaBuffer<T, N> {
    fn new() -> Self {
        Self {
            data: unsafe { MaybeUninit::uninit().assume_init() },
            initialized: 0,
        }
    }
    
    fn push(&mut self, value: T) -> Result<(), T> {
        if self.initialized < N {
            self.data[self.initialized].write(value);
            self.initialized += 1;
            Ok(())
        } else {
            Err(value)
        }
    }
    
    fn as_ptr(&self) -> *const T {
        self.data.as_ptr() as *const T
    }
    
    fn as_mut_ptr(&mut self) -> *mut T {
        self.data.as_mut_ptr() as *mut T
    }
    
    fn len(&self) -> usize {
        self.initialized
    }
    
    fn capacity(&self) -> usize {
        N
    }
    
    fn is_dma_safe(&self) -> bool {
        // 检查内存对齐和位置
        let ptr = self.as_ptr() as usize;
        let align = align_of::<T>();
        
        // 检查对齐
        if ptr % align != 0 {
            return false;
        }
        
        // 检查是否在RAM中（STM32F4的RAM范围）
        const RAM_START: usize = 0x20000000;
        const RAM_END: usize = 0x20020000;
        
        ptr >= RAM_START && ptr < RAM_END
    }
    
    fn clear(&mut self) {
        self.initialized = 0;
    }
}

// 内存碎片分析器
struct FragmentationAnalyzer {
    free_blocks: Vec<(usize, usize), 64>, // (地址, 大小)
    allocated_blocks: Vec<(usize, usize), 64>,
}

impl FragmentationAnalyzer {
    fn new() -> Self {
        Self {
            free_blocks: Vec::new(),
            allocated_blocks: Vec::new(),
        }
    }
    
    fn add_free_block(&mut self, addr: usize, size: usize) {
        self.free_blocks.push((addr, size)).ok();
    }
    
    fn add_allocated_block(&mut self, addr: usize, size: usize) {
        self.allocated_blocks.push((addr, size)).ok();
    }
    
    fn calculate_fragmentation(&self) -> f32 {
        if self.free_blocks.is_empty() {
            return 0.0;
        }
        
        let total_free: usize = self.free_blocks.iter().map(|(_, size)| *size).sum();
        let largest_free = self.free_blocks.iter().map(|(_, size)| *size).max().unwrap_or(0);
        
        if total_free == 0 {
            0.0
        } else {
            1.0 - (largest_free as f32 / total_free as f32)
        }
    }
    
    fn get_free_block_count(&self) -> usize {
        self.free_blocks.len()
    }
    
    fn get_largest_free_block(&self) -> usize {
        self.free_blocks.iter().map(|(_, size)| *size).max().unwrap_or(0)
    }
    
    fn get_total_free_memory(&self) -> usize {
        self.free_blocks.iter().map(|(_, size)| *size).sum()
    }
    
    fn analyze_fragmentation(&self) -> FragmentationReport {
        let total_free = self.get_total_free_memory();
        let largest_free = self.get_largest_free_block();
        let block_count = self.get_free_block_count();
        let fragmentation = self.calculate_fragmentation();
        
        let average_block_size = if block_count > 0 {
            total_free / block_count
        } else {
            0
        };
        
        FragmentationReport {
            total_free_memory: total_free,
            largest_free_block: largest_free,
            free_block_count: block_count,
            average_block_size,
            fragmentation_ratio: fragmentation,
            fragmentation_level: if fragmentation < 0.1 {
                FragmentationLevel::Low
            } else if fragmentation < 0.3 {
                FragmentationLevel::Medium
            } else {
                FragmentationLevel::High
            },
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct FragmentationReport {
    total_free_memory: usize,
    largest_free_block: usize,
    free_block_count: usize,
    average_block_size: usize,
    fragmentation_ratio: f32,
    fragmentation_level: FragmentationLevel,
}

#[derive(Debug, Clone, Copy)]
enum FragmentationLevel {
    Low,
    Medium,
    High,
}

// 内存性能测试
struct MemoryBenchmark {
    cycles: u32,
    operations: u32,
}

impl MemoryBenchmark {
    fn new() -> Self {
        Self {
            cycles: 0,
            operations: 0,
        }
    }
    
    fn benchmark_allocation<F>(&mut self, iterations: u32, mut allocator: F) 
    where
        F: FnMut() -> bool,
    {
        let start_cycles = DWT::cycle_count();
        
        let mut successful_ops = 0;
        for _ in 0..iterations {
            if allocator() {
                successful_ops += 1;
            }
        }
        
        let end_cycles = DWT::cycle_count();
        
        self.cycles = end_cycles.wrapping_sub(start_cycles);
        self.operations = successful_ops;
    }
    
    fn get_cycles_per_operation(&self) -> f32 {
        if self.operations > 0 {
            self.cycles as f32 / self.operations as f32
        } else {
            f32::INFINITY
        }
    }
    
    fn get_operations_per_second(&self, cpu_freq_hz: u32) -> f32 {
        if self.cycles > 0 {
            (self.operations as f32 * cpu_freq_hz as f32) / self.cycles as f32
        } else {
            0.0
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
    
    cortex_m_log::println!("Starting memory management tests...");
    
    // 测试静态分配器
    test_static_allocator();
    
    // 测试内存池
    test_memory_pools();
    
    // 测试栈使用分析
    test_stack_analysis();
    
    // 测试缓存优化
    test_cache_optimization();
    
    // 测试DMA内存
    test_dma_memory();
    
    // 测试内存碎片分析
    test_fragmentation_analysis();
    
    // 性能基准测试
    run_memory_benchmarks();
    
    cortex_m_log::println!("Memory management tests completed!");
    
    loop {
        cortex_m::asm::wfi();
    }
}

fn test_static_allocator() {
    cortex_m_log::println!("\n=== Testing Static Allocator ===");
    
    let mut allocator = StaticAllocator::new();
    
    // 分配不同大小的内存块
    let ptr1 = allocator.allocate(64, 4);
    let ptr2 = allocator.allocate(128, 8);
    let ptr3 = allocator.allocate(256, 16);
    
    cortex_m_log::println!("Allocation results: {:?}, {:?}, {:?}", 
        ptr1.is_some(), ptr2.is_some(), ptr3.is_some());
    
    let stats = allocator.get_stats();
    cortex_m_log::println!("Allocator stats: {} allocations, {} bytes", 
        stats.total_allocations, stats.total_bytes_allocated);
    
    cortex_m_log::println!("Memory usage: {:.1}%", allocator.get_usage() * 100.0);
}

fn test_memory_pools() {
    cortex_m_log::println!("\n=== Testing Memory Pools ===");
    
    let mut pool_manager = MemoryPoolManager::new();
    pool_manager.init();
    
    // 分配不同大小的内存
    let small_alloc = pool_manager.allocate(16);
    let medium_alloc = pool_manager.allocate(64);
    let large_alloc = pool_manager.allocate(256);
    
    cortex_m_log::println!("Pool allocations: {:?}, {:?}, {:?}", 
        small_alloc.is_some(), medium_alloc.is_some(), large_alloc.is_some());
    
    let (small_usage, medium_usage, large_usage) = pool_manager.get_pool_usage();
    cortex_m_log::println!("Pool usage: small={:.1}%, medium={:.1}%, large={:.1}%", 
        small_usage * 100.0, medium_usage * 100.0, large_usage * 100.0);
}

fn test_stack_analysis() {
    cortex_m_log::println!("\n=== Testing Stack Analysis ===");
    
    // 模拟栈分析（实际应用中需要获取真实的栈地址）
    let stack_buffer = [0u8; 1024];
    let mut analyzer = StackAnalyzer::new(stack_buffer.as_ptr(), stack_buffer.len());
    
    // 填充栈模式
    analyzer.fill_stack_pattern();
    
    // 模拟一些栈使用
    recursive_function(5);
    
    // 测量栈使用
    let usage = analyzer.measure_usage();
    cortex_m_log::println!("Stack usage: {} bytes ({:.1}%)", 
        usage, analyzer.get_usage_percentage());
}

fn recursive_function(depth: u32) {
    if depth > 0 {
        let _local_array = [0u32; 64]; // 消耗栈空间
        recursive_function(depth - 1);
    }
}

fn test_cache_optimization() {
    cortex_m_log::println!("\n=== Testing Cache Optimization ===");
    
    let mut cache_buffer: CacheAlignedBuffer<u32, 256> = CacheAlignedBuffer::new();
    
    // 测试缓存友好的批量处理
    let start_cycles = DWT::cycle_count();
    
    cache_buffer.batch_process(|item| {
        *item = item.wrapping_mul(2).wrapping_add(1);
    });
    
    let end_cycles = DWT::cycle_count();
    let cycles = end_cycles.wrapping_sub(start_cycles);
    
    cortex_m_log::println!("Cache-optimized processing: {} cycles", cycles);
    cortex_m_log::println!("Buffer alignment: {} bytes", align_of::<CacheAlignedBuffer<u32, 256>>());
}

fn test_dma_memory() {
    cortex_m_log::println!("\n=== Testing DMA Memory ===");
    
    let mut dma_buffer: DmaBuffer<u32, 128> = DmaBuffer::new();
    
    // 填充DMA缓冲区
    for i in 0..64 {
        dma_buffer.push(i).ok();
    }
    
    cortex_m_log::println!("DMA buffer: {} / {} elements", 
        dma_buffer.len(), dma_buffer.capacity());
    cortex_m_log::println!("DMA safe: {}", dma_buffer.is_dma_safe());
    cortex_m_log::println!("Buffer address: 0x{:08x}", dma_buffer.as_ptr() as usize);
}

fn test_fragmentation_analysis() {
    cortex_m_log::println!("\n=== Testing Fragmentation Analysis ===");
    
    let mut analyzer = FragmentationAnalyzer::new();
    
    // 模拟内存碎片
    analyzer.add_free_block(0x1000, 64);
    analyzer.add_free_block(0x1100, 32);
    analyzer.add_free_block(0x1200, 128);
    analyzer.add_free_block(0x1400, 16);
    
    analyzer.add_allocated_block(0x1040, 96);
    analyzer.add_allocated_block(0x1120, 224);
    
    let report = analyzer.analyze_fragmentation();
    
    cortex_m_log::println!("Fragmentation analysis:");
    cortex_m_log::println!("  Total free: {} bytes", report.total_free_memory);
    cortex_m_log::println!("  Largest free block: {} bytes", report.largest_free_block);
    cortex_m_log::println!("  Free block count: {}", report.free_block_count);
    cortex_m_log::println!("  Average block size: {} bytes", report.average_block_size);
    cortex_m_log::println!("  Fragmentation ratio: {:.2}", report.fragmentation_ratio);
    cortex_m_log::println!("  Fragmentation level: {:?}", report.fragmentation_level);
}

fn run_memory_benchmarks() {
    cortex_m_log::println!("\n=== Memory Performance Benchmarks ===");
    
    let mut benchmark = MemoryBenchmark::new();
    let mut allocator = StaticAllocator::new();
    
    // 基准测试：静态分配
    benchmark.benchmark_allocation(1000, || {
        allocator.reset();
        allocator.allocate(64, 4).is_some()
    });
    
    cortex_m_log::println!("Static allocation: {:.2} cycles/op, {:.0} ops/sec", 
        benchmark.get_cycles_per_operation(),
        benchmark.get_operations_per_second(168_000_000));
    
    // 基准测试：内存池分配
    let mut pool_manager = MemoryPoolManager::new();
    pool_manager.init();
    
    benchmark.benchmark_allocation(1000, || {
        pool_manager.allocate(64).is_some()
    });
    
    cortex_m_log::println!("Pool allocation: {:.2} cycles/op, {:.0} ops/sec", 
        benchmark.get_cycles_per_operation(),
        benchmark.get_operations_per_second(168_000_000));
}