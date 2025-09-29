# RTOS基础理论

## 概述

实时操作系统（Real-Time Operating System，RTOS）是一种专门为实时应用设计的操作系统，能够在严格的时间约束下响应外部事件和处理任务。本文档将深入介绍RTOS的基础理论、核心概念和设计原理。

## 目录

- [实时系统概念](#实时系统概念)
- [RTOS核心特性](#rtos核心特性)
- [任务调度理论](#任务调度理论)
- [时间管理](#时间管理)
- [资源管理](#资源管理)
- [中断处理](#中断处理)
- [内存管理](#内存管理)
- [实时性分析](#实时性分析)
- [RTOS分类](#rtos分类)
- [性能指标](#性能指标)

## 实时系统概念

### 什么是实时系统

实时系统是指系统的正确性不仅依赖于计算结果的逻辑正确性，还依赖于产生结果的时间。实时系统必须在规定的时间内完成特定的任务。

### 实时性分类

#### 硬实时（Hard Real-Time）
- **定义**：任务必须在截止时间前完成，否则系统失效
- **特点**：
  - 严格的时间约束
  - 错过截止时间会导致灾难性后果
  - 需要确定性的响应时间
- **应用场景**：
  - 飞行控制系统
  - 汽车安全系统
  - 医疗设备
  - 核电站控制

#### 软实时（Soft Real-Time）
- **定义**：任务有优选的截止时间，但偶尔错过不会导致系统失效
- **特点**：
  - 相对宽松的时间约束
  - 性能会降级但系统仍可用
  - 允许一定的时间抖动
- **应用场景**：
  - 多媒体系统
  - 网络通信
  - 用户界面
  - 数据采集

#### 固实时（Firm Real-Time）
- **定义**：介于硬实时和软实时之间
- **特点**：
  - 错过截止时间的结果无用但不危险
  - 系统可以丢弃过期的结果
- **应用场景**：
  - 视频流处理
  - 传感器数据采集

## RTOS核心特性

### 1. 确定性（Determinism）

```rust
// 确定性响应时间示例
pub struct DeterministicTimer {
    max_response_time: u32,  // 最大响应时间（微秒）
    actual_response: u32,    // 实际响应时间
}

impl DeterministicTimer {
    pub fn new(max_time: u32) -> Self {
        Self {
            max_response_time: max_time,
            actual_response: 0,
        }
    }
    
    pub fn is_deterministic(&self) -> bool {
        self.actual_response <= self.max_response_time
    }
}
```

### 2. 可预测性（Predictability）

- **时间可预测性**：能够预测任务的执行时间
- **行为可预测性**：系统行为在给定输入下是可预测的
- **资源可预测性**：资源使用模式是可预测的

### 3. 低延迟（Low Latency）

```rust
// 中断延迟测量
pub struct LatencyMeasurement {
    interrupt_time: u64,
    response_time: u64,
}

impl LatencyMeasurement {
    pub fn calculate_latency(&self) -> u64 {
        self.response_time - self.interrupt_time
    }
    
    pub fn is_acceptable(&self, max_latency: u64) -> bool {
        self.calculate_latency() <= max_latency
    }
}
```

### 4. 高可靠性（High Reliability）

- **故障检测**：及时发现系统故障
- **故障隔离**：防止故障扩散
- **故障恢复**：快速恢复正常运行
- **冗余设计**：关键组件的备份

## 任务调度理论

### 调度算法分类

#### 1. 抢占式调度（Preemptive Scheduling）

```rust
#[derive(Debug, Clone)]
pub struct Task {
    pub id: u32,
    pub priority: u8,
    pub period: u32,        // 周期（毫秒）
    pub deadline: u32,      // 截止时间
    pub execution_time: u32, // 执行时间
    pub remaining_time: u32, // 剩余执行时间
}

// 优先级调度
pub fn priority_schedule(tasks: &mut Vec<Task>) -> Option<&Task> {
    tasks.iter()
         .filter(|task| task.remaining_time > 0)
         .max_by_key(|task| task.priority)
}
```

#### 2. 非抢占式调度（Non-Preemptive Scheduling）

```rust
// FIFO调度
pub struct FifoScheduler {
    task_queue: VecDeque<Task>,
    current_task: Option<Task>,
}

impl FifoScheduler {
    pub fn new() -> Self {
        Self {
            task_queue: VecDeque::new(),
            current_task: None,
        }
    }
    
    pub fn add_task(&mut self, task: Task) {
        self.task_queue.push_back(task);
    }
    
    pub fn get_next_task(&mut self) -> Option<Task> {
        if self.current_task.is_none() {
            self.current_task = self.task_queue.pop_front();
        }
        self.current_task.clone()
    }
}
```

### 经典调度算法

#### 1. Rate Monotonic (RM)

```rust
// RM调度算法实现
pub fn rate_monotonic_schedule(tasks: &mut Vec<Task>) -> Vec<u32> {
    // 按周期排序（周期越短，优先级越高）
    tasks.sort_by_key(|task| task.period);
    
    // 分配优先级
    for (index, task) in tasks.iter_mut().enumerate() {
        task.priority = (tasks.len() - index) as u8;
    }
    
    tasks.iter().map(|task| task.id).collect()
}

// RM可调度性分析
pub fn rm_schedulability_test(tasks: &[Task]) -> bool {
    let n = tasks.len() as f64;
    let bound = n * (2.0_f64.powf(1.0/n) - 1.0);
    
    let utilization: f64 = tasks.iter()
        .map(|task| task.execution_time as f64 / task.period as f64)
        .sum();
    
    utilization <= bound
}
```

#### 2. Earliest Deadline First (EDF)

```rust
// EDF调度算法
pub fn edf_schedule(tasks: &mut Vec<Task>, current_time: u32) -> Option<&Task> {
    tasks.iter()
         .filter(|task| task.remaining_time > 0)
         .min_by_key(|task| task.deadline)
}

// EDF可调度性分析
pub fn edf_schedulability_test(tasks: &[Task]) -> bool {
    let utilization: f64 = tasks.iter()
        .map(|task| task.execution_time as f64 / task.period as f64)
        .sum();
    
    utilization <= 1.0
}
```

## 时间管理

### 系统时钟

```rust
pub struct SystemClock {
    tick_frequency: u32,    // 时钟频率（Hz）
    tick_count: u64,        // 时钟计数
    resolution: u32,        // 时间分辨率（微秒）
}

impl SystemClock {
    pub fn new(frequency: u32) -> Self {
        Self {
            tick_frequency: frequency,
            tick_count: 0,
            resolution: 1_000_000 / frequency,
        }
    }
    
    pub fn tick(&mut self) {
        self.tick_count += 1;
    }
    
    pub fn get_time_us(&self) -> u64 {
        self.tick_count * self.resolution as u64
    }
    
    pub fn get_time_ms(&self) -> u64 {
        self.get_time_us() / 1000
    }
}
```

### 定时器管理

```rust
#[derive(Debug, Clone)]
pub struct Timer {
    pub id: u32,
    pub period: u32,        // 定时周期
    pub remaining: u32,     // 剩余时间
    pub auto_reload: bool,  // 自动重载
    pub callback: Option<fn()>, // 回调函数
}

pub struct TimerManager {
    timers: Vec<Timer>,
    next_id: u32,
}

impl TimerManager {
    pub fn new() -> Self {
        Self {
            timers: Vec::new(),
            next_id: 1,
        }
    }
    
    pub fn create_timer(&mut self, period: u32, auto_reload: bool) -> u32 {
        let timer = Timer {
            id: self.next_id,
            period,
            remaining: period,
            auto_reload,
            callback: None,
        };
        
        self.timers.push(timer);
        let id = self.next_id;
        self.next_id += 1;
        id
    }
    
    pub fn tick(&mut self) -> Vec<u32> {
        let mut expired_timers = Vec::new();
        
        for timer in &mut self.timers {
            if timer.remaining > 0 {
                timer.remaining -= 1;
                
                if timer.remaining == 0 {
                    expired_timers.push(timer.id);
                    
                    if timer.auto_reload {
                        timer.remaining = timer.period;
                    }
                }
            }
        }
        
        expired_timers
    }
}
```

## 资源管理

### 互斥锁（Mutex）

```rust
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

pub struct Mutex<T> {
    locked: AtomicBool,
    data: UnsafeCell<T>,
}

unsafe impl<T: Send> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    pub const fn new(data: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            data: UnsafeCell::new(data),
        }
    }
    
    pub fn lock(&self) -> MutexGuard<T> {
        while self.locked.compare_exchange_weak(
            false, true, Ordering::Acquire, Ordering::Relaxed
        ).is_err() {
            // 自旋等待
            core::hint::spin_loop();
        }
        
        MutexGuard { mutex: self }
    }
    
    pub fn try_lock(&self) -> Option<MutexGuard<T>> {
        if self.locked.compare_exchange(
            false, true, Ordering::Acquire, Ordering::Relaxed
        ).is_ok() {
            Some(MutexGuard { mutex: self })
        } else {
            None
        }
    }
}

pub struct MutexGuard<'a, T> {
    mutex: &'a Mutex<T>,
}

impl<T> Drop for MutexGuard<'_, T> {
    fn drop(&mut self) {
        self.mutex.locked.store(false, Ordering::Release);
    }
}

impl<T> core::ops::Deref for MutexGuard<'_, T> {
    type Target = T;
    
    fn deref(&self) -> &T {
        unsafe { &*self.mutex.data.get() }
    }
}

impl<T> core::ops::DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *self.mutex.data.get() }
    }
}
```

### 信号量（Semaphore）

```rust
use core::sync::atomic::{AtomicUsize, Ordering};

pub struct Semaphore {
    count: AtomicUsize,
    max_count: usize,
}

impl Semaphore {
    pub const fn new(initial_count: usize, max_count: usize) -> Self {
        Self {
            count: AtomicUsize::new(initial_count),
            max_count,
        }
    }
    
    pub fn acquire(&self) -> bool {
        loop {
            let current = self.count.load(Ordering::Acquire);
            if current == 0 {
                return false; // 无法获取
            }
            
            if self.count.compare_exchange_weak(
                current, current - 1, Ordering::AcqRel, Ordering::Relaxed
            ).is_ok() {
                return true;
            }
        }
    }
    
    pub fn release(&self) -> bool {
        loop {
            let current = self.count.load(Ordering::Acquire);
            if current >= self.max_count {
                return false; // 已达到最大值
            }
            
            if self.count.compare_exchange_weak(
                current, current + 1, Ordering::AcqRel, Ordering::Relaxed
            ).is_ok() {
                return true;
            }
        }
    }
    
    pub fn count(&self) -> usize {
        self.count.load(Ordering::Acquire)
    }
}
```

## 中断处理

### 中断优先级

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct InterruptPriority(pub u8);

impl InterruptPriority {
    pub const HIGHEST: Self = Self(0);
    pub const HIGH: Self = Self(64);
    pub const MEDIUM: Self = Self(128);
    pub const LOW: Self = Self(192);
    pub const LOWEST: Self = Self(255);
}

#[derive(Debug)]
pub struct InterruptHandler {
    pub priority: InterruptPriority,
    pub handler: fn(),
    pub enabled: bool,
}

pub struct InterruptController {
    handlers: [Option<InterruptHandler>; 256],
    current_priority: InterruptPriority,
}

impl InterruptController {
    pub fn new() -> Self {
        Self {
            handlers: [None; 256],
            current_priority: InterruptPriority::LOWEST,
        }
    }
    
    pub fn register_handler(&mut self, irq: u8, handler: InterruptHandler) {
        self.handlers[irq as usize] = Some(handler);
    }
    
    pub fn handle_interrupt(&mut self, irq: u8) {
        if let Some(handler) = &self.handlers[irq as usize] {
            if handler.enabled && handler.priority < self.current_priority {
                let old_priority = self.current_priority;
                self.current_priority = handler.priority;
                
                (handler.handler)();
                
                self.current_priority = old_priority;
            }
        }
    }
}
```

### 中断延迟分析

```rust
pub struct InterruptLatency {
    pub interrupt_time: u64,    // 中断发生时间
    pub entry_time: u64,        // 进入处理程序时间
    pub exit_time: u64,         // 退出处理程序时间
    pub resume_time: u64,       // 恢复任务时间
}

impl InterruptLatency {
    pub fn response_time(&self) -> u64 {
        self.entry_time - self.interrupt_time
    }
    
    pub fn handling_time(&self) -> u64 {
        self.exit_time - self.entry_time
    }
    
    pub fn recovery_time(&self) -> u64 {
        self.resume_time - self.exit_time
    }
    
    pub fn total_latency(&self) -> u64 {
        self.resume_time - self.interrupt_time
    }
}
```

## 内存管理

### 静态内存分配

```rust
pub struct StaticAllocator {
    memory_pool: &'static mut [u8],
    free_blocks: Vec<(usize, usize)>, // (起始地址, 大小)
    allocated_blocks: Vec<(usize, usize)>,
}

impl StaticAllocator {
    pub fn new(memory_pool: &'static mut [u8]) -> Self {
        let pool_size = memory_pool.len();
        Self {
            memory_pool,
            free_blocks: vec![(0, pool_size)],
            allocated_blocks: Vec::new(),
        }
    }
    
    pub fn allocate(&mut self, size: usize) -> Option<*mut u8> {
        // 寻找合适的空闲块
        for (index, &(start, block_size)) in self.free_blocks.iter().enumerate() {
            if block_size >= size {
                // 分配内存
                self.allocated_blocks.push((start, size));
                
                // 更新空闲块
                if block_size == size {
                    self.free_blocks.remove(index);
                } else {
                    self.free_blocks[index] = (start + size, block_size - size);
                }
                
                return Some(unsafe { self.memory_pool.as_mut_ptr().add(start) });
            }
        }
        
        None
    }
    
    pub fn deallocate(&mut self, ptr: *mut u8, size: usize) {
        let start = unsafe { ptr.offset_from(self.memory_pool.as_ptr()) } as usize;
        
        // 移除已分配块
        self.allocated_blocks.retain(|&(addr, _)| addr != start);
        
        // 添加到空闲块并合并相邻块
        self.free_blocks.push((start, size));
        self.merge_free_blocks();
    }
    
    fn merge_free_blocks(&mut self) {
        self.free_blocks.sort_by_key(|&(start, _)| start);
        
        let mut merged = Vec::new();
        let mut current = self.free_blocks[0];
        
        for &(start, size) in &self.free_blocks[1..] {
            if current.0 + current.1 == start {
                // 合并相邻块
                current.1 += size;
            } else {
                merged.push(current);
                current = (start, size);
            }
        }
        merged.push(current);
        
        self.free_blocks = merged;
    }
}
```

## 实时性分析

### 响应时间分析

```rust
pub struct ResponseTimeAnalysis {
    pub tasks: Vec<Task>,
}

impl ResponseTimeAnalysis {
    pub fn calculate_response_time(&self, task_index: usize) -> Option<u32> {
        let task = &self.tasks[task_index];
        let mut response_time = task.execution_time;
        let mut prev_response_time = 0;
        
        // 迭代计算响应时间
        while response_time != prev_response_time {
            prev_response_time = response_time;
            response_time = task.execution_time;
            
            // 计算高优先级任务的干扰
            for (i, higher_task) in self.tasks.iter().enumerate() {
                if i < task_index { // 假设索引越小优先级越高
                    let interference = (prev_response_time + higher_task.period - 1) 
                                     / higher_task.period 
                                     * higher_task.execution_time;
                    response_time += interference;
                }
            }
            
            // 防止无限循环
            if response_time > task.deadline {
                return None; // 不可调度
            }
        }
        
        Some(response_time)
    }
    
    pub fn is_schedulable(&self) -> bool {
        for i in 0..self.tasks.len() {
            if let Some(response_time) = self.calculate_response_time(i) {
                if response_time > self.tasks[i].deadline {
                    return false;
                }
            } else {
                return false;
            }
        }
        true
    }
}
```

## RTOS分类

### 按内核架构分类

#### 1. 单体内核（Monolithic Kernel）
- **特点**：所有系统服务运行在内核空间
- **优势**：性能高，通信开销小
- **劣势**：可靠性相对较低
- **代表**：FreeRTOS, RT-Thread

#### 2. 微内核（Microkernel）
- **特点**：最小化内核功能，服务运行在用户空间
- **优势**：可靠性高，模块化好
- **劣势**：性能开销较大
- **代表**：QNX, MINIX

#### 3. 混合内核（Hybrid Kernel）
- **特点**：结合单体内核和微内核的优势
- **优势**：平衡性能和可靠性
- **代表**：Windows CE, VxWorks

### 按应用领域分类

#### 1. 通用RTOS
- **特点**：适用于多种应用场景
- **代表**：FreeRTOS, Zephyr, RT-Thread

#### 2. 安全关键RTOS
- **特点**：通过安全认证，用于关键系统
- **代表**：INTEGRITY, PikeOS, SafeRTOS

#### 3. 嵌入式Linux
- **特点**：基于Linux内核的实时扩展
- **代表**：RT-Linux, PREEMPT_RT

## 性能指标

### 关键性能指标

```rust
#[derive(Debug, Default)]
pub struct RTOSMetrics {
    pub task_switch_time: u32,      // 任务切换时间（微秒）
    pub interrupt_latency: u32,     // 中断延迟（微秒）
    pub memory_footprint: u32,      // 内存占用（字节）
    pub cpu_utilization: f32,       // CPU利用率（0-1）
    pub context_switch_overhead: u32, // 上下文切换开销
    pub timer_resolution: u32,      // 定时器分辨率（微秒）
}

impl RTOSMetrics {
    pub fn calculate_efficiency(&self) -> f32 {
        // 综合效率评分
        let latency_score = 1.0 / (self.interrupt_latency as f32 + 1.0);
        let switch_score = 1.0 / (self.task_switch_time as f32 + 1.0);
        let memory_score = 1.0 / (self.memory_footprint as f32 / 1024.0 + 1.0);
        let cpu_score = 1.0 - self.cpu_utilization;
        
        (latency_score + switch_score + memory_score + cpu_score) / 4.0
    }
    
    pub fn is_real_time_capable(&self) -> bool {
        self.interrupt_latency < 100 &&  // 中断延迟小于100微秒
        self.task_switch_time < 50 &&   // 任务切换小于50微秒
        self.cpu_utilization < 0.8      // CPU利用率小于80%
    }
}
```

### 基准测试

```rust
pub struct RTOSBenchmark {
    metrics: RTOSMetrics,
    test_iterations: u32,
}

impl RTOSBenchmark {
    pub fn new() -> Self {
        Self {
            metrics: RTOSMetrics::default(),
            test_iterations: 1000,
        }
    }
    
    pub fn run_benchmark(&mut self) -> RTOSMetrics {
        self.measure_task_switch_time();
        self.measure_interrupt_latency();
        self.measure_memory_usage();
        self.measure_cpu_utilization();
        
        self.metrics.clone()
    }
    
    fn measure_task_switch_time(&mut self) {
        // 任务切换时间测量实现
        let start_time = self.get_current_time();
        
        for _ in 0..self.test_iterations {
            // 模拟任务切换
            self.simulate_task_switch();
        }
        
        let end_time = self.get_current_time();
        self.metrics.task_switch_time = (end_time - start_time) / self.test_iterations;
    }
    
    fn measure_interrupt_latency(&mut self) {
        // 中断延迟测量实现
        // 这里需要硬件支持或模拟
    }
    
    fn measure_memory_usage(&mut self) {
        // 内存使用测量
        // 统计内核和任务的内存占用
    }
    
    fn measure_cpu_utilization(&mut self) {
        // CPU利用率测量
        // 通过空闲任务的运行时间计算
    }
    
    fn get_current_time(&self) -> u32 {
        // 获取当前时间戳
        0 // 占位实现
    }
    
    fn simulate_task_switch(&self) {
        // 模拟任务切换
    }
}
```

## 总结

RTOS基础理论涵盖了实时系统的核心概念、调度算法、资源管理、中断处理等关键技术。理解这些基础理论对于设计和实现高质量的实时系统至关重要。

### 关键要点

1. **实时性**：系统的正确性依赖于时间约束
2. **确定性**：系统行为必须是可预测的
3. **调度算法**：选择合适的调度策略确保实时性
4. **资源管理**：避免优先级反转和死锁
5. **中断处理**：最小化中断延迟
6. **内存管理**：确保内存分配的确定性
7. **性能分析**：验证系统的实时性能

### 下一步学习

- [RTIC框架详解](02-rtic-framework.md)
- [FreeRTOS集成](03-freertos-integration.md)
- [基础任务管理](04-basic-task-management.md)

---

*本文档是嵌入式Rust开发教程的一部分，更多内容请参考项目主页。*