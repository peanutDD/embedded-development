# RTOS基础

实时操作系统(RTOS)是嵌入式系统中的核心组件，提供任务调度、同步机制、内存管理等功能。本文档介绍RTOS的基本概念和在STM32F4上的实现。

## RTOS概述

### 什么是RTOS
实时操作系统(Real-Time Operating System)是一种能够在确定时间内响应外部事件的操作系统。它具有以下特点：

- **确定性**: 系统响应时间可预测
- **实时性**: 能在规定时间内完成任务
- **多任务**: 支持多个任务并发执行
- **优先级调度**: 基于任务优先级进行调度
- **同步机制**: 提供任务间通信和同步

### RTOS vs 裸机编程

| 特性 | 裸机编程 | RTOS |
|------|----------|------|
| 复杂度 | 低 | 中等 |
| 实时性 | 高 | 可配置 |
| 多任务 | 手动实现 | 内置支持 |
| 内存开销 | 最小 | 适中 |
| 开发效率 | 低 | 高 |
| 可维护性 | 低 | 高 |

## RTIC框架介绍

RTIC (Real-Time Interrupt-driven Concurrency) 是Rust生态中的实时框架，专为Cortex-M微控制器设计。

### RTIC特性
- **零成本抽象**: 编译时优化，运行时开销最小
- **内存安全**: Rust的所有权系统保证内存安全
- **硬件抽象**: 基于中断的任务调度
- **资源共享**: 安全的资源访问机制
- **时间管理**: 内置定时器和延时功能

### RTIC应用结构
```rust
#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0, EXTI1])]
mod app {
    use stm32f4xx_hal::prelude::*;
    
    #[shared]
    struct Shared {
        counter: u32,
    }
    
    #[local]
    struct Local {
        led: LedPin,
    }
    
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // 初始化代码
    }
    
    #[task(shared = [counter])]
    fn task1(ctx: task1::Context) {
        // 任务1代码
    }
    
    #[task(local = [led])]
    fn task2(ctx: task2::Context) {
        // 任务2代码
    }
}
```

## 任务管理

### 任务状态
RTOS中的任务通常有以下状态：

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TaskState {
    Ready,      // 就绪状态
    Running,    // 运行状态
    Blocked,    // 阻塞状态
    Suspended,  // 挂起状态
}

/// 任务控制块
pub struct TaskControlBlock {
    pub id: u32,
    pub priority: u8,
    pub state: TaskState,
    pub stack_pointer: *mut u32,
    pub stack_size: usize,
    pub entry_point: fn(),
}

impl TaskControlBlock {
    pub fn new(id: u32, priority: u8, entry_point: fn(), stack_size: usize) -> Self {
        Self {
            id,
            priority,
            state: TaskState::Ready,
            stack_pointer: core::ptr::null_mut(),
            stack_size,
            entry_point,
        }
    }
    
    /// 设置任务状态
    pub fn set_state(&mut self, state: TaskState) {
        self.state = state;
    }
    
    /// 检查任务是否就绪
    pub fn is_ready(&self) -> bool {
        self.state == TaskState::Ready
    }
}
```

### 任务调度器
```rust
use heapless::Vec;

/// 简单的优先级调度器
pub struct PriorityScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    current_task: Option<usize>,
}

impl PriorityScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            current_task: None,
        }
    }
    
    /// 添加任务
    pub fn add_task(&mut self, task: TaskControlBlock) -> Result<(), &'static str> {
        self.tasks.push(task).map_err(|_| "Too many tasks")
    }
    
    /// 获取下一个要运行的任务
    pub fn get_next_task(&mut self) -> Option<&mut TaskControlBlock> {
        let mut highest_priority = 0;
        let mut next_task_index = None;
        
        // 查找最高优先级的就绪任务
        for (index, task) in self.tasks.iter().enumerate() {
            if task.is_ready() && task.priority > highest_priority {
                highest_priority = task.priority;
                next_task_index = Some(index);
            }
        }
        
        if let Some(index) = next_task_index {
            self.current_task = Some(index);
            self.tasks.get_mut(index)
        } else {
            None
        }
    }
    
    /// 任务切换
    pub fn schedule(&mut self) {
        if let Some(next_task) = self.get_next_task() {
            next_task.set_state(TaskState::Running);
            // 执行上下文切换
            self.context_switch(next_task);
        }
    }
    
    /// 上下文切换（简化实现）
    fn context_switch(&self, task: &TaskControlBlock) {
        // 保存当前上下文
        // 加载新任务上下文
        // 跳转到任务入口点
        unsafe {
            // 设置栈指针
            cortex_m::register::msp::write(task.stack_pointer as u32);
            // 调用任务函数
            (task.entry_point)();
        }
    }
}
```

## 同步机制

### 互斥锁 (Mutex)
```rust
use core::cell::UnsafeCell;
use cortex_m::interrupt;

/// 简单的互斥锁实现
pub struct Mutex<T> {
    data: UnsafeCell<T>,
    locked: core::sync::atomic::AtomicBool,
}

unsafe impl<T: Send> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    pub const fn new(data: T) -> Self {
        Self {
            data: UnsafeCell::new(data),
            locked: core::sync::atomic::AtomicBool::new(false),
        }
    }
    
    /// 获取锁
    pub fn lock<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        // 禁用中断以实现原子操作
        interrupt::free(|_| {
            // 自旋等待锁释放
            while self.locked.load(core::sync::atomic::Ordering::Acquire) {
                cortex_m::asm::nop();
            }
            
            // 获取锁
            self.locked.store(true, core::sync::atomic::Ordering::Release);
            
            // 执行临界区代码
            let result = f(unsafe { &mut *self.data.get() });
            
            // 释放锁
            self.locked.store(false, core::sync::atomic::Ordering::Release);
            
            result
        })
    }
}

// 使用示例
static SHARED_COUNTER: Mutex<u32> = Mutex::new(0);

fn increment_counter() {
    SHARED_COUNTER.lock(|counter| {
        *counter += 1;
    });
}
```

### 信号量 (Semaphore)
```rust
use core::sync::atomic::{AtomicUsize, Ordering};

/// 计数信号量
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
    
    /// 获取信号量（P操作）
    pub fn acquire(&self) -> bool {
        loop {
            let current = self.count.load(Ordering::Acquire);
            if current == 0 {
                return false; // 无法获取
            }
            
            if self.count.compare_exchange_weak(
                current,
                current - 1,
                Ordering::Release,
                Ordering::Relaxed
            ).is_ok() {
                return true;
            }
        }
    }
    
    /// 释放信号量（V操作）
    pub fn release(&self) -> bool {
        loop {
            let current = self.count.load(Ordering::Acquire);
            if current >= self.max_count {
                return false; // 已达最大值
            }
            
            if self.count.compare_exchange_weak(
                current,
                current + 1,
                Ordering::Release,
                Ordering::Relaxed
            ).is_ok() {
                return true;
            }
        }
    }
    
    /// 获取当前计数
    pub fn count(&self) -> usize {
        self.count.load(Ordering::Acquire)
    }
}

// 使用示例
static RESOURCE_SEMAPHORE: Semaphore = Semaphore::new(3, 3); // 最多3个资源

fn use_resource() {
    if RESOURCE_SEMAPHORE.acquire() {
        // 使用资源
        // ...
        
        // 释放资源
        RESOURCE_SEMAPHORE.release();
    }
}
```

### 消息队列
```rust
use heapless::{spsc::{Queue, Producer, Consumer}, Vec};

/// 消息类型
#[derive(Debug, Clone, Copy)]
pub enum Message {
    SensorData(u16),
    Command(u8),
    Status(bool),
}

/// 消息队列管理器
pub struct MessageQueue {
    queue: Queue<Message, 16>,
}

impl MessageQueue {
    pub fn new() -> Self {
        Self {
            queue: Queue::new(),
        }
    }
    
    /// 分离生产者和消费者
    pub fn split(&mut self) -> (Producer<Message, 16>, Consumer<Message, 16>) {
        self.queue.split()
    }
}

// 使用示例
static mut MESSAGE_QUEUE: MessageQueue = MessageQueue::new();

fn producer_task() {
    let (mut producer, _) = unsafe { MESSAGE_QUEUE.split() };
    
    loop {
        let sensor_value = read_sensor();
        let message = Message::SensorData(sensor_value);
        
        if producer.enqueue(message).is_err() {
            // 队列满，处理错误
        }
        
        // 延时
        cortex_m::asm::delay(1000);
    }
}

fn consumer_task() {
    let (_, mut consumer) = unsafe { MESSAGE_QUEUE.split() };
    
    loop {
        if let Some(message) = consumer.dequeue() {
            match message {
                Message::SensorData(value) => {
                    // 处理传感器数据
                    process_sensor_data(value);
                },
                Message::Command(cmd) => {
                    // 处理命令
                    execute_command(cmd);
                },
                Message::Status(status) => {
                    // 处理状态
                    update_status(status);
                },
            }
        }
        
        // 短暂延时
        cortex_m::asm::delay(100);
    }
}

fn read_sensor() -> u16 {
    // 模拟传感器读取
    42
}

fn process_sensor_data(value: u16) {
    // 处理传感器数据
}

fn execute_command(cmd: u8) {
    // 执行命令
}

fn update_status(status: bool) {
    // 更新状态
}
```

## 内存管理

### 静态内存分配
```rust
use heapless::pool::{Pool, Node};

/// 内存池管理器
pub struct MemoryPool<const N: usize> {
    pool: Pool<Node<[u8; 64]>>,
    memory: [Node<[u8; 64]>; N],
}

impl<const N: usize> MemoryPool<N> {
    pub fn new() -> Self {
        Self {
            pool: Pool::new(),
            memory: [Node::new(); N],
        }
    }
    
    /// 初始化内存池
    pub fn init(&mut self) {
        for node in &mut self.memory {
            self.pool.manage(node);
        }
    }
    
    /// 分配内存块
    pub fn allocate(&mut self) -> Option<&mut [u8; 64]> {
        self.pool.alloc().map(|node| &mut node.data)
    }
    
    /// 释放内存块
    pub fn deallocate(&mut self, block: &mut [u8; 64]) {
        // 将内存块转换回Node并释放
        let node = unsafe {
            &mut *(block as *mut [u8; 64] as *mut Node<[u8; 64]>)
        };
        self.pool.free(node);
    }
}

// 全局内存池
static mut MEMORY_POOL: MemoryPool<16> = MemoryPool::new();

fn allocate_buffer() -> Option<&'static mut [u8; 64]> {
    unsafe { MEMORY_POOL.allocate() }
}

fn free_buffer(buffer: &mut [u8; 64]) {
    unsafe { MEMORY_POOL.deallocate(buffer) };
}
```

### 栈管理
```rust
/// 任务栈管理
pub struct TaskStack {
    stack: [u32; 1024], // 4KB栈
    stack_pointer: usize,
}

impl TaskStack {
    pub fn new() -> Self {
        Self {
            stack: [0; 1024],
            stack_pointer: 1024, // 栈顶
        }
    }
    
    /// 获取栈顶指针
    pub fn get_stack_top(&self) -> *mut u32 {
        unsafe { self.stack.as_ptr().add(self.stack.len() - 1) as *mut u32 }
    }
    
    /// 初始化任务栈
    pub fn init_task_stack(&mut self, entry_point: fn()) -> *mut u32 {
        // 设置初始栈帧
        let stack_top = self.get_stack_top();
        
        unsafe {
            // 设置初始寄存器值
            *stack_top.offset(-1) = 0x01000000; // xPSR
            *stack_top.offset(-2) = entry_point as u32; // PC
            *stack_top.offset(-3) = 0; // LR
            *stack_top.offset(-4) = 0; // R12
            *stack_top.offset(-5) = 0; // R3
            *stack_top.offset(-6) = 0; // R2
            *stack_top.offset(-7) = 0; // R1
            *stack_top.offset(-8) = 0; // R0
            
            // 返回栈指针
            stack_top.offset(-8)
        }
    }
}
```

## 定时器和延时

### 软件定时器
```rust
use cortex_m::peripheral::DWT;

/// 软件定时器
pub struct SoftwareTimer {
    start_time: u32,
    duration: u32,
    active: bool,
}

impl SoftwareTimer {
    pub fn new() -> Self {
        Self {
            start_time: 0,
            duration: 0,
            active: false,
        }
    }
    
    /// 启动定时器
    pub fn start(&mut self, duration_ms: u32) {
        self.start_time = DWT::cycle_count();
        self.duration = duration_ms * (84_000_000 / 1000); // 84MHz时钟
        self.active = true;
    }
    
    /// 检查定时器是否超时
    pub fn is_expired(&self) -> bool {
        if !self.active {
            return false;
        }
        
        let current_time = DWT::cycle_count();
        let elapsed = current_time.wrapping_sub(self.start_time);
        elapsed >= self.duration
    }
    
    /// 停止定时器
    pub fn stop(&mut self) {
        self.active = false;
    }
    
    /// 重置定时器
    pub fn reset(&mut self) {
        if self.active {
            self.start_time = DWT::cycle_count();
        }
    }
}

// 使用示例
fn timer_example() {
    let mut timer = SoftwareTimer::new();
    
    // 启动1秒定时器
    timer.start(1000);
    
    loop {
        if timer.is_expired() {
            // 定时器超时，执行任务
            perform_periodic_task();
            
            // 重启定时器
            timer.start(1000);
        }
        
        // 其他任务
        do_other_work();
    }
}

fn perform_periodic_task() {
    // 周期性任务
}

fn do_other_work() {
    // 其他工作
}
```

### 延时函数
```rust
/// 精确延时函数
pub struct DelayManager {
    clock_freq: u32,
}

impl DelayManager {
    pub fn new(clock_freq: u32) -> Self {
        Self { clock_freq }
    }
    
    /// 毫秒延时
    pub fn delay_ms(&self, ms: u32) {
        let cycles = ms * (self.clock_freq / 1000);
        self.delay_cycles(cycles);
    }
    
    /// 微秒延时
    pub fn delay_us(&self, us: u32) {
        let cycles = us * (self.clock_freq / 1_000_000);
        self.delay_cycles(cycles);
    }
    
    /// 周期延时
    pub fn delay_cycles(&self, cycles: u32) {
        let start = DWT::cycle_count();
        while DWT::cycle_count().wrapping_sub(start) < cycles {
            cortex_m::asm::nop();
        }
    }
    
    /// 非阻塞延时检查
    pub fn check_delay(&self, start_time: u32, delay_cycles: u32) -> bool {
        let current_time = DWT::cycle_count();
        current_time.wrapping_sub(start_time) >= delay_cycles
    }
}

// 全局延时管理器
static DELAY: DelayManager = DelayManager::new(84_000_000);

fn delay_example() {
    // 延时1秒
    DELAY.delay_ms(1000);
    
    // 延时100微秒
    DELAY.delay_us(100);
    
    // 非阻塞延时
    let start_time = DWT::cycle_count();
    let delay_cycles = 84_000; // 1ms at 84MHz
    
    loop {
        if DELAY.check_delay(start_time, delay_cycles) {
            // 延时完成
            break;
        }
        
        // 在延时期间可以做其他事情
        do_background_work();
    }
}

fn do_background_work() {
    // 后台工作
}
```

## 完整的RTOS示例

### 简单的多任务系统
```rust
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{prelude::*, stm32};

// 任务函数
fn task1() {
    loop {
        // 任务1：LED闪烁
        toggle_led1();
        delay_ms(500);
    }
}

fn task2() {
    loop {
        // 任务2：传感器读取
        let sensor_value = read_sensor();
        process_sensor_data(sensor_value);
        delay_ms(100);
    }
}

fn task3() {
    loop {
        // 任务3：通信处理
        handle_communication();
        delay_ms(50);
    }
}

#[entry]
fn main() -> ! {
    // 硬件初始化
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    
    // 启用DWT循环计数器
    let mut dwt = cp.DWT;
    dwt.enable_cycle_counter();
    
    // 创建调度器
    let mut scheduler = PriorityScheduler::new();
    
    // 添加任务
    scheduler.add_task(TaskControlBlock::new(1, 3, task1, 1024)).unwrap();
    scheduler.add_task(TaskControlBlock::new(2, 2, task2, 1024)).unwrap();
    scheduler.add_task(TaskControlBlock::new(3, 1, task3, 1024)).unwrap();
    
    // 启动调度器
    loop {
        scheduler.schedule();
    }
}

// 辅助函数
fn toggle_led1() {
    // LED切换实现
}

fn read_sensor() -> u16 {
    // 传感器读取实现
    42
}

fn process_sensor_data(value: u16) {
    // 数据处理实现
}

fn handle_communication() {
    // 通信处理实现
}

fn delay_ms(ms: u32) {
    DELAY.delay_ms(ms);
}
```

## 总结

RTOS为嵌入式系统提供了强大的多任务处理能力，通过合理的任务调度、同步机制和资源管理，可以构建复杂而可靠的实时系统。RTIC框架为Rust嵌入式开发提供了现代化的RTOS解决方案，结合Rust的内存安全特性，能够开发出高性能、高可靠性的嵌入式应用。

在实际应用中，需要根据系统需求选择合适的调度策略、同步机制和内存管理方案，并进行充分的测试和优化，以确保系统的实时性和稳定性。