#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    timer::Timer,
    gpio::{Output, PushPull, gpiod::PD12},
};

use heapless::{
    pool::{Pool, Node},
    Vec,
    String,
};

use linked_list_allocator::LockedHeap;
use log::info;
use rtt_target::{rprintln, rtt_init_print};
use rtt_log::RTTLogger;

extern crate alloc;
use alloc::{
    vec::Vec as AllocVec,
    string::String as AllocString,
    boxed::Box,
    collections::BTreeMap,
};

// 全局堆分配器
#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

// 堆内存区域 (使用CCM RAM)
static mut HEAP: [u8; 32 * 1024] = [0; 32 * 1024];

// 内存池节点类型
type PoolNode = Node<[u8; 64]>;
static mut POOL_MEMORY: [PoolNode; 16] = [Node::new(); 16];
static POOL: Pool<PoolNode> = Pool::new();

// 栈监控结构
#[derive(Debug)]
struct StackMonitor {
    stack_start: usize,
    stack_size: usize,
    max_usage: usize,
    current_usage: usize,
    overflow_count: u32,
}

impl StackMonitor {
    fn new(stack_start: usize, stack_size: usize) -> Self {
        Self {
            stack_start,
            stack_size,
            max_usage: 0,
            current_usage: 0,
            overflow_count: 0,
        }
    }
    
    fn update(&mut self) {
        let current_sp: usize;
        unsafe {
            asm!("mov {}, sp", out(reg) current_sp);
        }
        
        if current_sp < self.stack_start {
            self.current_usage = self.stack_start - current_sp;
            if self.current_usage > self.max_usage {
                self.max_usage = self.current_usage;
            }
            
            // 检查栈溢出
            if self.current_usage > self.stack_size {
                self.overflow_count += 1;
                rprintln!("Stack overflow detected! Usage: {} bytes", self.current_usage);
            }
        }
    }
    
    fn get_usage_percent(&self) -> u32 {
        (self.max_usage * 100) / self.stack_size
    }
}

// 内存分配统计
#[derive(Debug, Default)]
struct AllocStats {
    total_allocations: u32,
    total_deallocations: u32,
    current_allocations: u32,
    peak_allocations: u32,
    total_allocated_bytes: usize,
    total_deallocated_bytes: usize,
    current_allocated_bytes: usize,
    peak_allocated_bytes: usize,
    allocation_failures: u32,
}

static mut ALLOC_STATS: AllocStats = AllocStats {
    total_allocations: 0,
    total_deallocations: 0,
    current_allocations: 0,
    peak_allocations: 0,
    total_allocated_bytes: 0,
    total_deallocated_bytes: 0,
    current_allocated_bytes: 0,
    peak_allocated_bytes: 0,
    allocation_failures: 0,
};

// 内存泄漏检测器
#[derive(Debug)]
struct LeakDetector {
    allocations: BTreeMap<usize, (usize, u32)>, // 地址 -> (大小, 分配ID)
    next_alloc_id: u32,
}

impl LeakDetector {
    fn new() -> Self {
        Self {
            allocations: BTreeMap::new(),
            next_alloc_id: 1,
        }
    }
    
    fn record_allocation(&mut self, ptr: usize, size: usize) {
        self.allocations.insert(ptr, (size, self.next_alloc_id));
        self.next_alloc_id += 1;
    }
    
    fn record_deallocation(&mut self, ptr: usize) -> bool {
        self.allocations.remove(&ptr).is_some()
    }
    
    fn check_leaks(&self) -> Vec<(usize, usize, u32), 32> {
        let mut leaks = Vec::new();
        for (&ptr, &(size, alloc_id)) in &self.allocations {
            if leaks.push((ptr, size, alloc_id)).is_err() {
                break; // Vec已满
            }
        }
        leaks
    }
    
    fn get_leak_count(&self) -> usize {
        self.allocations.len()
    }
    
    fn get_leaked_bytes(&self) -> usize {
        self.allocations.values().map(|(size, _)| *size).sum()
    }
}

static mut LEAK_DETECTOR: Option<LeakDetector> = None;

// 内存管理示例结构
struct MemoryExample {
    led: PD12<Output<PushPull>>,
    timer: Timer<pac::TIM2>,
    stack_monitor: StackMonitor,
    test_counter: u32,
}

impl MemoryExample {
    fn new(led: PD12<Output<PushPull>>, timer: Timer<pac::TIM2>) -> Self {
        // 获取栈信息
        let stack_start: usize;
        unsafe {
            asm!("mov {}, sp", out(reg) stack_start);
        }
        
        Self {
            led,
            timer,
            stack_monitor: StackMonitor::new(stack_start, 8192), // 假设8KB栈
            test_counter: 0,
        }
    }
    
    fn run(&mut self) {
        info!("Starting memory management example...");
        
        loop {
            self.test_counter += 1;
            
            // 更新栈监控
            self.stack_monitor.update();
            
            // 运行不同的内存测试
            match self.test_counter % 6 {
                0 => self.test_heap_allocation(),
                1 => self.test_memory_pools(),
                2 => self.test_stack_usage(),
                3 => self.test_leak_detection(),
                4 => self.test_memory_fragmentation(),
                5 => self.print_memory_stats(),
                _ => {}
            }
            
            // LED指示
            self.led.toggle();
            
            // 延时
            self.timer.start(1.Hz());
            nb::block!(self.timer.wait()).unwrap();
        }
    }
    
    fn test_heap_allocation(&mut self) {
        info!("Testing heap allocation...");
        
        // 测试动态分配
        let mut vec = AllocVec::new();
        for i in 0..10 {
            vec.push(i * i);
        }
        
        let string = AllocString::from("Hello, heap!");
        let boxed_value = Box::new(42u32);
        
        info!("Vec: {:?}", vec.as_slice());
        info!("String: {}", string.as_str());
        info!("Boxed value: {}", *boxed_value);
        
        // 测试大块内存分配
        let large_buffer = AllocVec::<u8>::with_capacity(1024);
        info!("Large buffer capacity: {}", large_buffer.capacity());
        
        // 自动释放内存
        drop(vec);
        drop(string);
        drop(boxed_value);
        drop(large_buffer);
    }
    
    fn test_memory_pools(&mut self) {
        info!("Testing memory pools...");
        
        // 从内存池分配
        if let Some(mut block) = POOL.alloc() {
            // 使用内存块
            block.fill(0xAA);
            info!("Pool block allocated and filled");
            
            // 释放回池
            POOL.free(block);
            info!("Pool block freed");
        } else {
            info!("Pool allocation failed - no free blocks");
        }
        
        // 测试多个分配
        let mut blocks = Vec::<_, 4>::new();
        for i in 0..4 {
            if let Some(block) = POOL.alloc() {
                blocks.push(block).ok();
                info!("Allocated pool block {}", i);
            }
        }
        
        // 释放所有块
        for block in blocks {
            POOL.free(block);
        }
        info!("All pool blocks freed");
    }
    
    fn test_stack_usage(&mut self) {
        info!("Testing stack usage...");
        
        // 递归函数测试栈使用
        fn recursive_function(depth: u32, buffer: &mut [u8; 256]) -> u32 {
            if depth == 0 {
                return buffer.len() as u32;
            }
            
            // 使用栈空间
            buffer.fill(depth as u8);
            let sum: u32 = buffer.iter().map(|&x| x as u32).sum();
            
            recursive_function(depth - 1, buffer) + sum
        }
        
        let mut buffer = [0u8; 256];
        let result = recursive_function(5, &mut buffer);
        
        info!("Recursive function result: {}", result);
        info!("Stack usage: {}% ({} bytes)", 
              self.stack_monitor.get_usage_percent(),
              self.stack_monitor.max_usage);
    }
    
    fn test_leak_detection(&mut self) {
        info!("Testing leak detection...");
        
        unsafe {
            if let Some(ref mut detector) = LEAK_DETECTOR {
                // 模拟内存泄漏
                let leaked_box = Box::new([0u8; 128]);
                let ptr = Box::into_raw(leaked_box) as usize;
                detector.record_allocation(ptr, 128);
                
                // 检查泄漏
                let leaks = detector.check_leaks();
                info!("Detected {} memory leaks", leaks.len());
                info!("Total leaked bytes: {}", detector.get_leaked_bytes());
                
                for (i, &(ptr, size, alloc_id)) in leaks.iter().enumerate() {
                    info!("Leak {}: ptr=0x{:x}, size={}, id={}", i, ptr, size, alloc_id);
                }
            }
        }
    }
    
    fn test_memory_fragmentation(&mut self) {
        info!("Testing memory fragmentation...");
        
        // 分配不同大小的内存块
        let mut allocations = Vec::<Box<[u8]>, 8>::new();
        
        let sizes = [64, 128, 32, 256, 16, 512, 8, 1024];
        for &size in &sizes {
            let buffer = vec![0u8; size].into_boxed_slice();
            if allocations.push(buffer).is_err() {
                info!("Allocation vector full");
                break;
            }
            info!("Allocated {} bytes", size);
        }
        
        // 释放一些块（模拟碎片化）
        if allocations.len() > 4 {
            allocations.remove(1); // 释放128字节块
            allocations.remove(2); // 释放32字节块
            info!("Freed some blocks to create fragmentation");
        }
        
        // 尝试分配中等大小的块
        match AllocVec::<u8>::try_with_capacity(96) {
            Ok(vec) => {
                info!("Successfully allocated 96 bytes despite fragmentation");
                drop(vec);
            }
            Err(_) => {
                info!("Failed to allocate 96 bytes due to fragmentation");
            }
        }
        
        // 清理剩余分配
        allocations.clear();
        info!("Cleaned up fragmentation test allocations");
    }
    
    fn print_memory_stats(&mut self) {
        info!("=== Memory Statistics ===");
        
        unsafe {
            info!("Heap allocations: {} total, {} current", 
                  ALLOC_STATS.total_allocations, 
                  ALLOC_STATS.current_allocations);
            info!("Heap bytes: {} total, {} current, {} peak", 
                  ALLOC_STATS.total_allocated_bytes,
                  ALLOC_STATS.current_allocated_bytes,
                  ALLOC_STATS.peak_allocated_bytes);
            info!("Allocation failures: {}", ALLOC_STATS.allocation_failures);
        }
        
        info!("Stack usage: {}% ({}/{} bytes)", 
              self.stack_monitor.get_usage_percent(),
              self.stack_monitor.max_usage,
              self.stack_monitor.stack_size);
        
        if self.stack_monitor.overflow_count > 0 {
            info!("Stack overflows detected: {}", self.stack_monitor.overflow_count);
        }
        
        unsafe {
            if let Some(ref detector) = LEAK_DETECTOR {
                info!("Memory leaks: {} blocks, {} bytes", 
                      detector.get_leak_count(),
                      detector.get_leaked_bytes());
            }
        }
        
        info!("=========================");
    }
}

// 分配错误处理
#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
    unsafe {
        ALLOC_STATS.allocation_failures += 1;
    }
    rprintln!("Memory allocation failed!");
    panic!("Memory allocation failed");
}

// 自定义分配器钩子（用于统计）
struct StatsAllocator;

#[entry]
fn main() -> ! {
    // 初始化RTT日志
    rtt_init_print!();
    static LOGGER: RTTLogger = RTTLogger::new(log::LevelFilter::Info);
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(log::LevelFilter::Info);
    
    info!("Memory Management Example Starting...");
    
    // 初始化堆分配器
    unsafe {
        ALLOCATOR.lock().init(HEAP.as_mut_ptr(), HEAP.len());
        info!("Heap initialized: {} bytes", HEAP.len());
    }
    
    // 初始化内存池
    unsafe {
        POOL.grow(&mut POOL_MEMORY);
        info!("Memory pool initialized: {} blocks", POOL_MEMORY.len());
    }
    
    // 初始化泄漏检测器
    unsafe {
        LEAK_DETECTOR = Some(LeakDetector::new());
        info!("Leak detector initialized");
    }
    
    // 硬件初始化
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();
    
    // 配置GPIO
    let gpiod = dp.GPIOD.split();
    let led = gpiod.pd12.into_push_pull_output();
    
    // 配置定时器
    let timer = Timer::new(dp.TIM2, &clocks);
    
    // 创建并运行示例
    let mut example = MemoryExample::new(led, timer);
    example.run();
}

// 测试函数
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_stack_monitor() {
        let mut monitor = StackMonitor::new(0x20020000, 8192);
        monitor.current_usage = 1024;
        monitor.max_usage = 1024;
        
        assert_eq!(monitor.get_usage_percent(), 12); // 1024/8192 * 100 ≈ 12%
    }
    
    #[test]
    fn test_leak_detector() {
        let mut detector = LeakDetector::new();
        
        detector.record_allocation(0x1000, 128);
        detector.record_allocation(0x2000, 256);
        
        assert_eq!(detector.get_leak_count(), 2);
        assert_eq!(detector.get_leaked_bytes(), 384);
        
        assert!(detector.record_deallocation(0x1000));
        assert_eq!(detector.get_leak_count(), 1);
        assert_eq!(detector.get_leaked_bytes(), 256);
    }
    
    #[test]
    fn test_alloc_stats() {
        let mut stats = AllocStats::default();
        
        stats.total_allocations = 10;
        stats.current_allocations = 5;
        stats.total_allocated_bytes = 1024;
        stats.current_allocated_bytes = 512;
        
        assert_eq!(stats.total_allocations, 10);
        assert_eq!(stats.current_allocated_bytes, 512);
    }
}