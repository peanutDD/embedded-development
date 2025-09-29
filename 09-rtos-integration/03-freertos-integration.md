# FreeRTOS集成

## 目录

1. [FreeRTOS概述](#freertos概述)
2. [Rust中的FreeRTOS](#rust中的freertos)
3. [环境配置](#环境配置)
4. [基础集成](#基础集成)
5. [任务管理](#任务管理)
6. [同步原语](#同步原语)
7. [内存管理](#内存管理)
8. [中断处理](#中断处理)
9. [定时器管理](#定时器管理)
10. [队列通信](#队列通信)
11. [事件组](#事件组)
12. [任务通知](#任务通知)
13. [性能优化](#性能优化)
14. [调试技巧](#调试技巧)
15. [最佳实践](#最佳实践)

## FreeRTOS概述

### 什么是FreeRTOS

FreeRTOS是一个开源的实时操作系统内核，专为微控制器和小型微处理器设计。它提供了任务调度、内存管理、任务间通信等核心功能。

### 核心特性

- **抢占式调度**：支持基于优先级的抢占式任务调度
- **协作式调度**：可选的协作式调度模式
- **内存管理**：多种内存分配策略
- **同步原语**：信号量、互斥锁、队列等
- **软件定时器**：高精度定时器支持
- **低功耗支持**：Tickless模式

### 架构组件

```rust
// FreeRTOS核心组件
pub mod freertos_core {
    use core::ffi::c_void;
    use core::ptr;
    
    // 任务句柄类型
    pub type TaskHandle = *mut c_void;
    
    // 任务优先级类型
    pub type Priority = u32;
    
    // 任务状态枚举
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum TaskState {
        Running,
        Ready,
        Blocked,
        Suspended,
        Deleted,
    }
    
    // 系统配置
    pub struct SystemConfig {
        pub max_priorities: u32,
        pub minimal_stack_size: u32,
        pub total_heap_size: u32,
        pub tick_rate_hz: u32,
        pub use_preemption: bool,
        pub use_time_slicing: bool,
        pub use_tickless_idle: bool,
    }
    
    impl Default for SystemConfig {
        fn default() -> Self {
            Self {
                max_priorities: 5,
                minimal_stack_size: 128,
                total_heap_size: 8192,
                tick_rate_hz: 1000,
                use_preemption: true,
                use_time_slicing: true,
                use_tickless_idle: false,
            }
        }
    }
}
```

## Rust中的FreeRTOS

### freertos-rust Crate

`freertos-rust`是FreeRTOS的Rust绑定，提供了类型安全的API。

```toml
# Cargo.toml
[dependencies]
freertos-rust = "0.1"
freertos-sys = "0.1"
cortex-m = "0.7"
cortex-m-rt = "0.7"

[build-dependencies]
cc = "1.0"
```

### 基础类型定义

```rust
use freertos_rust::*;
use core::time::Duration;

// 任务函数类型
type TaskFunction = fn();

// 任务配置
#[derive(Debug, Clone)]
pub struct TaskConfig {
    pub name: &'static str,
    pub stack_size: u16,
    pub priority: u8,
    pub function: TaskFunction,
}

// 系统统计信息
#[derive(Debug, Default)]
pub struct SystemStats {
    pub total_tasks: u32,
    pub running_tasks: u32,
    pub free_heap_size: u32,
    pub minimum_ever_free_heap_size: u32,
    pub total_run_time: u64,
    pub idle_run_time: u64,
}
```

## 环境配置

### FreeRTOS配置文件

```c
// FreeRTOSConfig.h
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// 基础配置
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      72000000
#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                128
#define configTOTAL_HEAP_SIZE                   8192
#define configMAX_TASK_NAME_LEN                 16

// 钩子函数配置
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     1
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_STACK_OVERFLOW_HOOK           2

// 同步原语配置
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_QUEUE_SETS                    1
#define configUSE_TASK_NOTIFICATIONS            1

// 定时器配置
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               3
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            128

#endif
```

### 构建配置

```rust
// build.rs
use std::env;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    
    // 编译FreeRTOS源码
    cc::Build::new()
        .include("freertos/include")
        .include("freertos/portable/GCC/ARM_CM4F")
        .include(".")
        .file("freertos/tasks.c")
        .file("freertos/queue.c")
        .file("freertos/list.c")
        .file("freertos/timers.c")
        .file("freertos/event_groups.c")
        .file("freertos/stream_buffer.c")
        .file("freertos/portable/GCC/ARM_CM4F/port.c")
        .file("freertos/portable/MemMang/heap_4.c")
        .compile("freertos");
    
    println!("cargo:rustc-link-lib=static=freertos");
    println!("cargo:rerun-if-changed=freertos/");
}
```

## 基础集成

### 系统初始化

```rust
use freertos_rust::*;
use cortex_m_rt::entry;

// 系统管理器
pub struct FreeRTOSManager {
    config: freertos_core::SystemConfig,
    stats: SystemStats,
    tasks: Vec<TaskHandle>,
}

impl FreeRTOSManager {
    pub fn new(config: freertos_core::SystemConfig) -> Self {
        Self {
            config,
            stats: SystemStats::default(),
            tasks: Vec::new(),
        }
    }
    
    // 系统启动
    pub fn start_scheduler(&mut self) -> Result<(), FreeRtosError> {
        unsafe {
            // 启动调度器
            freertos_sys::vTaskStartScheduler();
        }
        Ok(())
    }
    
    // 创建任务
    pub fn create_task(
        &mut self,
        config: TaskConfig,
    ) -> Result<TaskHandle, FreeRtosError> {
        let task = Task::new()
            .name(config.name)
            .stack_size(config.stack_size)
            .priority(TaskPriority(config.priority))
            .start(config.function)?;
        
        let handle = task.get_handle();
        self.tasks.push(handle);
        Ok(handle)
    }
    
    // 获取系统统计信息
    pub fn get_system_stats(&mut self) -> SystemStats {
        unsafe {
            self.stats.total_tasks = freertos_sys::uxTaskGetNumberOfTasks();
            self.stats.free_heap_size = freertos_sys::xPortGetFreeHeapSize();
            self.stats.minimum_ever_free_heap_size = 
                freertos_sys::xPortGetMinimumEverFreeHeapSize();
        }
        self.stats.clone()
    }
}

// 应用程序入口
#[entry]
fn main() -> ! {
    let config = freertos_core::SystemConfig::default();
    let mut manager = FreeRTOSManager::new(config);
    
    // 创建任务
    let led_task_config = TaskConfig {
        name: "LED_Task",
        stack_size: 256,
        priority: 1,
        function: led_task,
    };
    
    manager.create_task(led_task_config).unwrap();
    
    // 启动调度器
    manager.start_scheduler().unwrap();
    
    loop {}
}

// LED任务示例
fn led_task() {
    loop {
        // LED控制逻辑
        FreeRtosUtils::delay(Duration::from_millis(500));
    }
}
```

## 任务管理

### 任务创建和管理

```rust
use freertos_rust::*;
use core::time::Duration;

// 任务管理器
pub struct TaskManager {
    tasks: Vec<TaskInfo>,
    next_task_id: u32,
}

#[derive(Debug, Clone)]
pub struct TaskInfo {
    pub id: u32,
    pub name: String,
    pub handle: TaskHandle,
    pub priority: u8,
    pub stack_size: u16,
    pub state: freertos_core::TaskState,
    pub run_time: u64,
    pub stack_high_water_mark: u16,
}

impl TaskManager {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            next_task_id: 1,
        }
    }
    
    // 创建高优先级任务
    pub fn create_high_priority_task<F>(
        &mut self,
        name: &str,
        stack_size: u16,
        task_fn: F,
    ) -> Result<u32, FreeRtosError>
    where
        F: Fn() + Send + 'static,
    {
        self.create_task_with_priority(name, stack_size, 4, task_fn)
    }
    
    // 创建普通优先级任务
    pub fn create_normal_priority_task<F>(
        &mut self,
        name: &str,
        stack_size: u16,
        task_fn: F,
    ) -> Result<u32, FreeRtosError>
    where
        F: Fn() + Send + 'static,
    {
        self.create_task_with_priority(name, stack_size, 2, task_fn)
    }
    
    // 创建低优先级任务
    pub fn create_low_priority_task<F>(
        &mut self,
        name: &str,
        stack_size: u16,
        task_fn: F,
    ) -> Result<u32, FreeRtosError>
    where
        F: Fn() + Send + 'static,
    {
        self.create_task_with_priority(name, stack_size, 1, task_fn)
    }
    
    // 创建指定优先级任务
    fn create_task_with_priority<F>(
        &mut self,
        name: &str,
        stack_size: u16,
        priority: u8,
        task_fn: F,
    ) -> Result<u32, FreeRtosError>
    where
        F: Fn() + Send + 'static,
    {
        let task_id = self.next_task_id;
        self.next_task_id += 1;
        
        let task = Task::new()
            .name(name)
            .stack_size(stack_size)
            .priority(TaskPriority(priority))
            .start(move || {
                task_fn();
            })?;
        
        let task_info = TaskInfo {
            id: task_id,
            name: name.to_string(),
            handle: task.get_handle(),
            priority,
            stack_size,
            state: freertos_core::TaskState::Ready,
            run_time: 0,
            stack_high_water_mark: stack_size,
        };
        
        self.tasks.push(task_info);
        Ok(task_id)
    }
    
    // 暂停任务
    pub fn suspend_task(&mut self, task_id: u32) -> Result<(), FreeRtosError> {
        if let Some(task_info) = self.tasks.iter_mut().find(|t| t.id == task_id) {
            unsafe {
                freertos_sys::vTaskSuspend(task_info.handle as *mut c_void);
            }
            task_info.state = freertos_core::TaskState::Suspended;
            Ok(())
        } else {
            Err(FreeRtosError::TaskNotFound)
        }
    }
    
    // 恢复任务
    pub fn resume_task(&mut self, task_id: u32) -> Result<(), FreeRtosError> {
        if let Some(task_info) = self.tasks.iter_mut().find(|t| t.id == task_id) {
            unsafe {
                freertos_sys::vTaskResume(task_info.handle as *mut c_void);
            }
            task_info.state = freertos_core::TaskState::Ready;
            Ok(())
        } else {
            Err(FreeRtosError::TaskNotFound)
        }
    }
    
    // 删除任务
    pub fn delete_task(&mut self, task_id: u32) -> Result<(), FreeRtosError> {
        if let Some(pos) = self.tasks.iter().position(|t| t.id == task_id) {
            let task_info = &self.tasks[pos];
            unsafe {
                freertos_sys::vTaskDelete(task_info.handle as *mut c_void);
            }
            self.tasks.remove(pos);
            Ok(())
        } else {
            Err(FreeRtosError::TaskNotFound)
        }
    }
    
    // 更新任务统计信息
    pub fn update_task_stats(&mut self) {
        for task_info in &mut self.tasks {
            unsafe {
                // 获取任务运行时间统计
                let mut run_time = 0u32;
                freertos_sys::vTaskGetRunTimeStats(
                    task_info.handle as *mut c_void,
                    &mut run_time,
                );
                task_info.run_time = run_time as u64;
                
                // 获取栈使用情况
                task_info.stack_high_water_mark = 
                    freertos_sys::uxTaskGetStackHighWaterMark(
                        task_info.handle as *mut c_void
                    ) as u16;
            }
        }
    }
    
    // 获取任务列表
    pub fn get_task_list(&self) -> &[TaskInfo] {
        &self.tasks
    }
}

// 任务示例
pub fn sensor_task() {
    loop {
        // 读取传感器数据
        let sensor_data = read_sensor();
        
        // 处理数据
        process_sensor_data(sensor_data);
        
        // 延时
        FreeRtosUtils::delay(Duration::from_millis(100));
    }
}

pub fn communication_task() {
    loop {
        // 处理通信
        handle_communication();
        
        // 让出CPU
        CurrentTask::delay(Duration::from_millis(10));
    }
}

// 模拟函数
fn read_sensor() -> u32 { 42 }
fn process_sensor_data(_data: u32) {}
fn handle_communication() {}
```

## 同步原语

### 互斥锁

```rust
use freertos_rust::*;
use core::time::Duration;

// 互斥锁管理器
pub struct MutexManager {
    mutexes: Vec<MutexInfo>,
    next_mutex_id: u32,
}

#[derive(Debug, Clone)]
pub struct MutexInfo {
    pub id: u32,
    pub name: String,
    pub mutex: Mutex<()>,
    pub holder_task: Option<TaskHandle>,
    pub wait_count: u32,
}

impl MutexManager {
    pub fn new() -> Self {
        Self {
            mutexes: Vec::new(),
            next_mutex_id: 1,
        }
    }
    
    // 创建互斥锁
    pub fn create_mutex(&mut self, name: &str) -> Result<u32, FreeRtosError> {
        let mutex_id = self.next_mutex_id;
        self.next_mutex_id += 1;
        
        let mutex = Mutex::new(())?;
        
        let mutex_info = MutexInfo {
            id: mutex_id,
            name: name.to_string(),
            mutex,
            holder_task: None,
            wait_count: 0,
        };
        
        self.mutexes.push(mutex_info);
        Ok(mutex_id)
    }
    
    // 获取互斥锁
    pub fn lock_mutex(
        &mut self,
        mutex_id: u32,
        timeout: Duration,
    ) -> Result<MutexGuard<()>, FreeRtosError> {
        if let Some(mutex_info) = self.mutexes.iter_mut().find(|m| m.id == mutex_id) {
            mutex_info.wait_count += 1;
            let guard = mutex_info.mutex.lock(timeout)?;
            mutex_info.holder_task = Some(CurrentTask::get_handle());
            Ok(guard)
        } else {
            Err(FreeRtosError::MutexNotFound)
        }
    }
    
    // 获取互斥锁信息
    pub fn get_mutex_info(&self, mutex_id: u32) -> Option<&MutexInfo> {
        self.mutexes.iter().find(|m| m.id == mutex_id)
    }
}

// 递归互斥锁
pub struct RecursiveMutexManager {
    recursive_mutexes: Vec<RecursiveMutexInfo>,
    next_mutex_id: u32,
}

#[derive(Debug, Clone)]
pub struct RecursiveMutexInfo {
    pub id: u32,
    pub name: String,
    pub mutex: RecursiveMutex<()>,
    pub holder_task: Option<TaskHandle>,
    pub recursion_count: u32,
}

impl RecursiveMutexManager {
    pub fn new() -> Self {
        Self {
            recursive_mutexes: Vec::new(),
            next_mutex_id: 1,
        }
    }
    
    // 创建递归互斥锁
    pub fn create_recursive_mutex(&mut self, name: &str) -> Result<u32, FreeRtosError> {
        let mutex_id = self.next_mutex_id;
        self.next_mutex_id += 1;
        
        let mutex = RecursiveMutex::new(())?;
        
        let mutex_info = RecursiveMutexInfo {
            id: mutex_id,
            name: name.to_string(),
            mutex,
            holder_task: None,
            recursion_count: 0,
        };
        
        self.recursive_mutexes.push(mutex_info);
        Ok(mutex_id)
    }
    
    // 获取递归互斥锁
    pub fn lock_recursive_mutex(
        &mut self,
        mutex_id: u32,
        timeout: Duration,
    ) -> Result<RecursiveMutexGuard<()>, FreeRtosError> {
        if let Some(mutex_info) = self.recursive_mutexes.iter_mut().find(|m| m.id == mutex_id) {
            let guard = mutex_info.mutex.lock(timeout)?;
            mutex_info.holder_task = Some(CurrentTask::get_handle());
            mutex_info.recursion_count += 1;
            Ok(guard)
        } else {
            Err(FreeRtosError::MutexNotFound)
        }
    }
}
```

### 信号量

```rust
// 信号量管理器
pub struct SemaphoreManager {
    binary_semaphores: Vec<BinarySemaphoreInfo>,
    counting_semaphores: Vec<CountingSemaphoreInfo>,
    next_semaphore_id: u32,
}

#[derive(Debug, Clone)]
pub struct BinarySemaphoreInfo {
    pub id: u32,
    pub name: String,
    pub semaphore: BinarySemaphore,
    pub available: bool,
    pub wait_count: u32,
}

#[derive(Debug, Clone)]
pub struct CountingSemaphoreInfo {
    pub id: u32,
    pub name: String,
    pub semaphore: CountingSemaphore,
    pub max_count: u32,
    pub current_count: u32,
    pub wait_count: u32,
}

impl SemaphoreManager {
    pub fn new() -> Self {
        Self {
            binary_semaphores: Vec::new(),
            counting_semaphores: Vec::new(),
            next_semaphore_id: 1,
        }
    }
    
    // 创建二进制信号量
    pub fn create_binary_semaphore(&mut self, name: &str) -> Result<u32, FreeRtosError> {
        let semaphore_id = self.next_semaphore_id;
        self.next_semaphore_id += 1;
        
        let semaphore = BinarySemaphore::create_binary()?;
        
        let semaphore_info = BinarySemaphoreInfo {
            id: semaphore_id,
            name: name.to_string(),
            semaphore,
            available: true,
            wait_count: 0,
        };
        
        self.binary_semaphores.push(semaphore_info);
        Ok(semaphore_id)
    }
    
    // 创建计数信号量
    pub fn create_counting_semaphore(
        &mut self,
        name: &str,
        max_count: u32,
        initial_count: u32,
    ) -> Result<u32, FreeRtosError> {
        let semaphore_id = self.next_semaphore_id;
        self.next_semaphore_id += 1;
        
        let semaphore = CountingSemaphore::create_counting(max_count, initial_count)?;
        
        let semaphore_info = CountingSemaphoreInfo {
            id: semaphore_id,
            name: name.to_string(),
            semaphore,
            max_count,
            current_count: initial_count,
            wait_count: 0,
        };
        
        self.counting_semaphores.push(semaphore_info);
        Ok(semaphore_id)
    }
    
    // 获取二进制信号量
    pub fn take_binary_semaphore(
        &mut self,
        semaphore_id: u32,
        timeout: Duration,
    ) -> Result<(), FreeRtosError> {
        if let Some(semaphore_info) = self.binary_semaphores.iter_mut().find(|s| s.id == semaphore_id) {
            semaphore_info.wait_count += 1;
            semaphore_info.semaphore.take(timeout)?;
            semaphore_info.available = false;
            Ok(())
        } else {
            Err(FreeRtosError::SemaphoreNotFound)
        }
    }
    
    // 释放二进制信号量
    pub fn give_binary_semaphore(&mut self, semaphore_id: u32) -> Result<(), FreeRtosError> {
        if let Some(semaphore_info) = self.binary_semaphores.iter_mut().find(|s| s.id == semaphore_id) {
            semaphore_info.semaphore.give()?;
            semaphore_info.available = true;
            Ok(())
        } else {
            Err(FreeRtosError::SemaphoreNotFound)
        }
    }
    
    // 获取计数信号量
    pub fn take_counting_semaphore(
        &mut self,
        semaphore_id: u32,
        timeout: Duration,
    ) -> Result<(), FreeRtosError> {
        if let Some(semaphore_info) = self.counting_semaphores.iter_mut().find(|s| s.id == semaphore_id) {
            semaphore_info.wait_count += 1;
            semaphore_info.semaphore.take(timeout)?;
            if semaphore_info.current_count > 0 {
                semaphore_info.current_count -= 1;
            }
            Ok(())
        } else {
            Err(FreeRtosError::SemaphoreNotFound)
        }
    }
    
    // 释放计数信号量
    pub fn give_counting_semaphore(&mut self, semaphore_id: u32) -> Result<(), FreeRtosError> {
        if let Some(semaphore_info) = self.counting_semaphores.iter_mut().find(|s| s.id == semaphore_id) {
            semaphore_info.semaphore.give()?;
            if semaphore_info.current_count < semaphore_info.max_count {
                semaphore_info.current_count += 1;
            }
            Ok(())
        } else {
            Err(FreeRtosError::SemaphoreNotFound)
        }
    }
}
```

## 内存管理

### 堆内存管理

```rust
use core::alloc::{GlobalAlloc, Layout};
use core::ptr;

// FreeRTOS堆分配器
pub struct FreeRTOSAllocator;

unsafe impl GlobalAlloc for FreeRTOSAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        freertos_sys::pvPortMalloc(layout.size()) as *mut u8
    }
    
    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
        freertos_sys::vPortFree(ptr as *mut core::ffi::c_void);
    }
}

#[global_allocator]
static ALLOCATOR: FreeRTOSAllocator = FreeRTOSAllocator;

// 内存管理器
pub struct MemoryManager {
    heap_stats: HeapStats,
    allocation_tracking: bool,
    allocations: Vec<AllocationInfo>,
}

#[derive(Debug, Default, Clone)]
pub struct HeapStats {
    pub total_heap_size: u32,
    pub free_heap_size: u32,
    pub minimum_ever_free_heap_size: u32,
    pub successful_allocations: u32,
    pub successful_frees: u32,
    pub allocation_failures: u32,
}

#[derive(Debug, Clone)]
pub struct AllocationInfo {
    pub ptr: *mut u8,
    pub size: usize,
    pub task_handle: TaskHandle,
    pub timestamp: u64,
}

impl MemoryManager {
    pub fn new() -> Self {
        Self {
            heap_stats: HeapStats::default(),
            allocation_tracking: false,
            allocations: Vec::new(),
        }
    }
    
    // 启用分配跟踪
    pub fn enable_allocation_tracking(&mut self) {
        self.allocation_tracking = true;
    }
    
    // 禁用分配跟踪
    pub fn disable_allocation_tracking(&mut self) {
        self.allocation_tracking = false;
        self.allocations.clear();
    }
    
    // 更新堆统计信息
    pub fn update_heap_stats(&mut self) {
        unsafe {
            self.heap_stats.free_heap_size = freertos_sys::xPortGetFreeHeapSize();
            self.heap_stats.minimum_ever_free_heap_size = 
                freertos_sys::xPortGetMinimumEverFreeHeapSize();
        }
    }
    
    // 获取堆统计信息
    pub fn get_heap_stats(&self) -> &HeapStats {
        &self.heap_stats
    }
    
    // 分配内存
    pub fn allocate(&mut self, size: usize) -> Option<*mut u8> {
        unsafe {
            let ptr = freertos_sys::pvPortMalloc(size) as *mut u8;
            
            if !ptr.is_null() {
                self.heap_stats.successful_allocations += 1;
                
                if self.allocation_tracking {
                    let allocation_info = AllocationInfo {
                        ptr,
                        size,
                        task_handle: CurrentTask::get_handle(),
                        timestamp: self.get_current_time(),
                    };
                    self.allocations.push(allocation_info);
                }
                
                Some(ptr)
            } else {
                self.heap_stats.allocation_failures += 1;
                None
            }
        }
    }
    
    // 释放内存
    pub fn deallocate(&mut self, ptr: *mut u8) {
        if !ptr.is_null() {
            unsafe {
                freertos_sys::vPortFree(ptr as *mut core::ffi::c_void);
            }
            
            self.heap_stats.successful_frees += 1;
            
            if self.allocation_tracking {
                self.allocations.retain(|alloc| alloc.ptr != ptr);
            }
        }
    }
    
    // 检查内存泄漏
    pub fn check_memory_leaks(&self) -> Vec<AllocationInfo> {
        if self.allocation_tracking {
            self.allocations.clone()
        } else {
            Vec::new()
        }
    }
    
    // 获取当前时间（模拟）
    fn get_current_time(&self) -> u64 {
        unsafe {
            freertos_sys::xTaskGetTickCount() as u64
        }
    }
    
    // 内存碎片分析
    pub fn analyze_fragmentation(&self) -> FragmentationReport {
        let free_heap = self.heap_stats.free_heap_size;
        let total_heap = self.heap_stats.total_heap_size;
        let used_heap = total_heap - free_heap;
        
        let fragmentation_ratio = if used_heap > 0 {
            (self.allocations.len() as f32) / (used_heap as f32 / 1024.0)
        } else {
            0.0
        };
        
        FragmentationReport {
            total_heap_size: total_heap,
            free_heap_size: free_heap,
            used_heap_size: used_heap,
            active_allocations: self.allocations.len() as u32,
            fragmentation_ratio,
            recommendation: if fragmentation_ratio > 0.1 {
                "High fragmentation detected. Consider memory pool allocation."
            } else {
                "Memory fragmentation is acceptable."
            }.to_string(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct FragmentationReport {
    pub total_heap_size: u32,
    pub free_heap_size: u32,
    pub used_heap_size: u32,
    pub active_allocations: u32,
    pub fragmentation_ratio: f32,
    pub recommendation: String,
}
```

## 中断处理

### 中断服务例程

```rust
use cortex_m::interrupt;
use freertos_rust::*;

// 中断管理器
pub struct InterruptManager {
    interrupt_handlers: Vec<InterruptHandlerInfo>,
    interrupt_stats: InterruptStats,
}

#[derive(Debug, Clone)]
pub struct InterruptHandlerInfo {
    pub irq_number: u32,
    pub name: String,
    pub priority: u8,
    pub enabled: bool,
    pub count: u32,
    pub max_execution_time: u32,
    pub total_execution_time: u64,
}

#[derive(Debug, Default, Clone)]
pub struct InterruptStats {
    pub total_interrupts: u64,
    pub nested_interrupts: u32,
    pub max_nesting_level: u8,
    pub interrupt_overhead: u32,
}

impl InterruptManager {
    pub fn new() -> Self {
        Self {
            interrupt_handlers: Vec::new(),
            interrupt_stats: InterruptStats::default(),
        }
    }
    
    // 注册中断处理程序
    pub fn register_interrupt_handler(
        &mut self,
        irq_number: u32,
        name: &str,
        priority: u8,
    ) -> Result<(), FreeRtosError> {
        let handler_info = InterruptHandlerInfo {
            irq_number,
            name: name.to_string(),
            priority,
            enabled: false,
            count: 0,
            max_execution_time: 0,
            total_execution_time: 0,
        };
        
        self.interrupt_handlers.push(handler_info);
        Ok(())
    }
    
    // 启用中断
    pub fn enable_interrupt(&mut self, irq_number: u32) -> Result<(), FreeRtosError> {
        if let Some(handler) = self.interrupt_handlers.iter_mut().find(|h| h.irq_number == irq_number) {
            handler.enabled = true;
            unsafe {
                cortex_m::peripheral::NVIC::unmask(cortex_m::interrupt::Nr::nr(irq_number as u8));
            }
            Ok(())
        } else {
            Err(FreeRtosError::InterruptNotFound)
        }
    }
    
    // 禁用中断
    pub fn disable_interrupt(&mut self, irq_number: u32) -> Result<(), FreeRtosError> {
        if let Some(handler) = self.interrupt_handlers.iter_mut().find(|h| h.irq_number == irq_number) {
            handler.enabled = false;
            cortex_m::peripheral::NVIC::mask(cortex_m::interrupt::Nr::nr(irq_number as u8));
            Ok(())
        } else {
            Err(FreeRtosError::InterruptNotFound)
        }
    }
    
    // 中断处理程序执行
    pub fn handle_interrupt(&mut self, irq_number: u32) {
        let start_time = self.get_current_time();
        
        if let Some(handler) = self.interrupt_handlers.iter_mut().find(|h| h.irq_number == irq_number) {
            handler.count += 1;
            
            // 执行中断处理逻辑
            self.execute_interrupt_handler(irq_number);
            
            let execution_time = self.get_current_time() - start_time;
            handler.total_execution_time += execution_time as u64;
            
            if execution_time > handler.max_execution_time {
                handler.max_execution_time = execution_time;
            }
        }
        
        self.interrupt_stats.total_interrupts += 1;
    }
    
    // 执行中断处理逻辑
    fn execute_interrupt_handler(&self, irq_number: u32) {
        match irq_number {
            // UART中断
            37 => self.handle_uart_interrupt(),
            // 定时器中断
            28 => self.handle_timer_interrupt(),
            // GPIO中断
            6 => self.handle_gpio_interrupt(),
            _ => {},
        }
    }
    
    // UART中断处理
    fn handle_uart_interrupt(&self) {
        // 从中断上下文发送通知
        unsafe {
            let task_handle = get_uart_task_handle();
            if !task_handle.is_null() {
                freertos_sys::vTaskNotifyGiveFromISR(
                    task_handle as *mut core::ffi::c_void,
                    ptr::null_mut(),
                );
            }
        }
    }
    
    // 定时器中断处理
    fn handle_timer_interrupt(&self) {
        // 从中断上下文发送到队列
        unsafe {
            let queue_handle = get_timer_queue_handle();
            if !queue_handle.is_null() {
                let timer_event = TimerEvent::Timeout;
                freertos_sys::xQueueSendFromISR(
                    queue_handle as *mut core::ffi::c_void,
                    &timer_event as *const _ as *const core::ffi::c_void,
                    ptr::null_mut(),
                );
            }
        }
    }
    
    // GPIO中断处理
    fn handle_gpio_interrupt(&self) {
        // 从中断上下文释放信号量
        unsafe {
            let semaphore_handle = get_gpio_semaphore_handle();
            if !semaphore_handle.is_null() {
                freertos_sys::xSemaphoreGiveFromISR(
                    semaphore_handle as *mut core::ffi::c_void,
                    ptr::null_mut(),
                );
            }
        }
    }
    
    // 获取中断统计信息
    pub fn get_interrupt_stats(&self) -> &InterruptStats {
        &self.interrupt_stats
    }
    
    // 获取中断处理程序信息
    pub fn get_interrupt_handlers(&self) -> &[InterruptHandlerInfo] {
        &self.interrupt_handlers
    }
    
    // 获取当前时间（模拟）
    fn get_current_time(&self) -> u32 {
        unsafe {
            freertos_sys::xTaskGetTickCountFromISR()
        }
    }
}

// 定时器事件类型
#[derive(Debug, Clone, Copy)]
pub enum TimerEvent {
    Timeout,
    Overflow,
    Compare,
}

// 模拟函数
fn get_uart_task_handle() -> TaskHandle { ptr::null_mut() }
fn get_timer_queue_handle() -> *mut core::ffi::c_void { ptr::null_mut() }
fn get_gpio_semaphore_handle() -> *mut core::ffi::c_void { ptr::null_mut() }
```

## 定时器管理

### 软件定时器

```rust
use freertos_rust::*;
use core::time::Duration;

// 定时器管理器
pub struct TimerManager {
    timers: Vec<TimerInfo>,
    next_timer_id: u32,
}

#[derive(Debug, Clone)]
pub struct TimerInfo {
    pub id: u32,
    pub name: String,
    pub timer: Timer,
    pub period: Duration,
    pub auto_reload: bool,
    pub active: bool,
    pub callback_count: u32,
}

// 定时器回调类型
type TimerCallback = fn(timer_id: u32);

impl TimerManager {
    pub fn new() -> Self {
        Self {
            timers: Vec::new(),
            next_timer_id: 1,
        }
    }
    
    // 创建一次性定时器
    pub fn create_one_shot_timer(
        &mut self,
        name: &str,
        period: Duration,
        callback: TimerCallback,
    ) -> Result<u32, FreeRtosError> {
        self.create_timer(name, period, false, callback)
    }
    
    // 创建周期性定时器
    pub fn create_periodic_timer(
        &mut self,
        name: &str,
        period: Duration,
        callback: TimerCallback,
    ) -> Result<u32, FreeRtosError> {
        self.create_timer(name, period, true, callback)
    }
    
    // 创建定时器
    fn create_timer(
        &mut self,
        name: &str,
        period: Duration,
        auto_reload: bool,
        callback: TimerCallback,
    ) -> Result<u32, FreeRtosError> {
        let timer_id = self.next_timer_id;
        self.next_timer_id += 1;
        
        let timer = Timer::create(
            period,
            auto_reload,
            move |_| {
                callback(timer_id);
            },
        )?;
        
        let timer_info = TimerInfo {
            id: timer_id,
            name: name.to_string(),
            timer,
            period,
            auto_reload,
            active: false,
            callback_count: 0,
        };
        
        self.timers.push(timer_info);
        Ok(timer_id)
    }
    
    // 启动定时器
    pub fn start_timer(&mut self, timer_id: u32) -> Result<(), FreeRtosError> {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.timer.start(Duration::from_millis(0))?;
            timer_info.active = true;
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 停止定时器
    pub fn stop_timer(&mut self, timer_id: u32) -> Result<(), FreeRtosError> {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.timer.stop(Duration::from_millis(0))?;
            timer_info.active = false;
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 重置定时器
    pub fn reset_timer(&mut self, timer_id: u32) -> Result<(), FreeRtosError> {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.timer.reset(Duration::from_millis(0))?;
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 更改定时器周期
    pub fn change_timer_period(
        &mut self,
        timer_id: u32,
        new_period: Duration,
    ) -> Result<(), FreeRtosError> {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.timer.change_period(new_period, Duration::from_millis(0))?;
            timer_info.period = new_period;
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 删除定时器
    pub fn delete_timer(&mut self, timer_id: u32) -> Result<(), FreeRtosError> {
        if let Some(pos) = self.timers.iter().position(|t| t.id == timer_id) {
            let timer_info = &self.timers[pos];
            timer_info.timer.delete(Duration::from_millis(0))?;
            self.timers.remove(pos);
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 获取定时器信息
    pub fn get_timer_info(&self, timer_id: u32) -> Option<&TimerInfo> {
        self.timers.iter().find(|t| t.id == timer_id)
    }
    
    // 获取所有定时器信息
    pub fn get_all_timers(&self) -> &[TimerInfo] {
        &self.timers
    }
    
    // 更新定时器统计信息
    pub fn update_timer_stats(&mut self) {
        for timer_info in &mut self.timers {
            // 这里可以添加更多统计信息的更新逻辑
            if timer_info.active {
                // 模拟回调计数更新
                timer_info.callback_count += 1;
            }
        }
    }
}

// 定时器回调示例
fn led_blink_callback(timer_id: u32) {
    // LED闪烁逻辑
    toggle_led();
}

fn sensor_read_callback(timer_id: u32) {
    // 传感器读取逻辑
    read_and_process_sensor();
}

fn watchdog_callback(timer_id: u32) {
    // 看门狗喂狗逻辑
    feed_watchdog();
}

// 模拟函数
fn toggle_led() {}
fn read_and_process_sensor() {}
fn feed_watchdog() {}
```

## 队列通信

### 消息队列

```rust
use freertos_rust::*;
use core::time::Duration;

// 队列管理器
pub struct QueueManager {
    queues: Vec<QueueInfo>,
    next_queue_id: u32,
}

#[derive(Debug, Clone)]
pub struct QueueInfo {
    pub id: u32,
    pub name: String,
    pub queue: Queue<QueueMessage>,
    pub max_items: u32,
    pub current_items: u32,
    pub item_size: usize,
    pub send_count: u64,
    pub receive_count: u64,
    pub send_failures: u32,
    pub receive_failures: u32,
}

// 通用队列消息
#[derive(Debug, Clone)]
pub struct QueueMessage {
    pub message_type: MessageType,
    pub sender_id: u32,
    pub timestamp: u64,
    pub data: MessageData,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MessageType {
    SensorData,
    Command,
    Status,
    Error,
    Heartbeat,
}

#[derive(Debug, Clone)]
pub enum MessageData {
    SensorReading { sensor_id: u32, value: f32 },
    Command { command_id: u32, parameters: Vec<u8> },
    Status { status_code: u32, message: String },
    Error { error_code: u32, description: String },
    Heartbeat { task_id: u32 },
}

impl QueueManager {
    pub fn new() -> Self {
        Self {
            queues: Vec::new(),
            next_queue_id: 1,
        }
    }
    
    // 创建队列
    pub fn create_queue(
        &mut self,
        name: &str,
        max_items: u32,
    ) -> Result<u32, FreeRtosError> {
        let queue_id = self.next_queue_id;
        self.next_queue_id += 1;
        
        let queue = Queue::new(max_items as usize)?;
        
        let queue_info = QueueInfo {
            id: queue_id,
            name: name.to_string(),
            queue,
            max_items,
            current_items: 0,
            item_size: core::mem::size_of::<QueueMessage>(),
            send_count: 0,
            receive_count: 0,
            send_failures: 0,
            receive_failures: 0,
        };
        
        self.queues.push(queue_info);
        Ok(queue_id)
    }
    
    // 发送消息
    pub fn send_message(
        &mut self,
        queue_id: u32,
        message: QueueMessage,
        timeout: Duration,
    ) -> Result<(), FreeRtosError> {
        if let Some(queue_info) = self.queues.iter_mut().find(|q| q.id == queue_id) {
            match queue_info.queue.send(message, timeout) {
                Ok(_) => {
                    queue_info.send_count += 1;
                    queue_info.current_items = queue_info.current_items.saturating_add(1).min(queue_info.max_items);
                    Ok(())
                }
                Err(e) => {
                    queue_info.send_failures += 1;
                    Err(e)
                }
            }
        } else {
            Err(FreeRtosError::QueueNotFound)
        }
    }
    
    // 接收消息
    pub fn receive_message(
        &mut self,
        queue_id: u32,
        timeout: Duration,
    ) -> Result<QueueMessage, FreeRtosError> {
        if let Some(queue_info) = self.queues.iter_mut().find(|q| q.id == queue_id) {
            match queue_info.queue.receive(timeout) {
                Ok(message) => {
                    queue_info.receive_count += 1;
                    queue_info.current_items = queue_info.current_items.saturating_sub(1);
                    Ok(message)
                }
                Err(e) => {
                    queue_info.receive_failures += 1;
                    Err(e)
                }
            }
        } else {
            Err(FreeRtosError::QueueNotFound)
        }
    }
    
    // 查看队列消息（不移除）
    pub fn peek_message(
        &self,
        queue_id: u32,
        timeout: Duration,
    ) -> Result<QueueMessage, FreeRtosError> {
        if let Some(queue_info) = self.queues.iter().find(|q| q.id == queue_id) {
            queue_info.queue.peek(timeout)
        } else {
            Err(FreeRtosError::QueueNotFound)
        }
    }
    
    // 获取队列状态
    pub fn get_queue_status(&mut self, queue_id: u32) -> Option<QueueStatus> {
        if let Some(queue_info) = self.queues.iter_mut().find(|q| q.id == queue_id) {
            // 更新当前项目数
            queue_info.current_items = queue_info.queue.len() as u32;
            
            Some(QueueStatus {
                id: queue_info.id,
                name: queue_info.name.clone(),
                max_items: queue_info.max_items,
                current_items: queue_info.current_items,
                available_space: queue_info.max_items - queue_info.current_items,
                send_count: queue_info.send_count,
                receive_count: queue_info.receive_count,
                send_failures: queue_info.send_failures,
                receive_failures: queue_info.receive_failures,
                utilization: (queue_info.current_items as f32 / queue_info.max_items as f32) * 100.0,
            })
        } else {
            None
        }
    }
    
    // 清空队列
    pub fn clear_queue(&mut self, queue_id: u32) -> Result<(), FreeRtosError> {
        if let Some(queue_info) = self.queues.iter_mut().find(|q| q.id == queue_id) {
            // 接收所有消息直到队列为空
            while queue_info.queue.receive(Duration::from_millis(0)).is_ok() {
                queue_info.current_items = queue_info.current_items.saturating_sub(1);
            }
            Ok(())
        } else {
            Err(FreeRtosError::QueueNotFound)
        }
    }
    
    // 删除队列
    pub fn delete_queue(&mut self, queue_id: u32) -> Result<(), FreeRtosError> {
        if let Some(pos) = self.queues.iter().position(|q| q.id == queue_id) {
            self.queues.remove(pos);
            Ok(())
        } else {
            Err(FreeRtosError::QueueNotFound)
        }
    }
    
    // 获取所有队列状态
    pub fn get_all_queue_status(&mut self) -> Vec<QueueStatus> {
        let mut statuses = Vec::new();
        for queue_info in &mut self.queues {
            if let Some(status) = self.get_queue_status(queue_info.id) {
                statuses.push(status);
            }
        }
        statuses
    }
}

#[derive(Debug, Clone)]
pub struct QueueStatus {
    pub id: u32,
    pub name: String,
    pub max_items: u32,
    pub current_items: u32,
    pub available_space: u32,
    pub send_count: u64,
    pub receive_count: u64,
    pub send_failures: u32,
    pub receive_failures: u32,
    pub utilization: f32,
}

// 消息构建器
impl QueueMessage {
    pub fn new_sensor_data(sensor_id: u32, value: f32, sender_id: u32) -> Self {
        Self {
            message_type: MessageType::SensorData,
            sender_id,
            timestamp: get_current_timestamp(),
            data: MessageData::SensorReading { sensor_id, value },
        }
    }
    
    pub fn new_command(command_id: u32, parameters: Vec<u8>, sender_id: u32) -> Self {
        Self {
            message_type: MessageType::Command,
            sender_id,
            timestamp: get_current_timestamp(),
            data: MessageData::Command { command_id, parameters },
        }
    }
    
    pub fn new_status(status_code: u32, message: String, sender_id: u32) -> Self {
        Self {
            message_type: MessageType::Status,
            sender_id,
            timestamp: get_current_timestamp(),
            data: MessageData::Status { status_code, message },
        }
    }
    
    pub fn new_error(error_code: u32, description: String, sender_id: u32) -> Self {
        Self {
            message_type: MessageType::Error,
            sender_id,
            timestamp: get_current_timestamp(),
            data: MessageData::Error { error_code, description },
        }
    }
    
    pub fn new_heartbeat(task_id: u32, sender_id: u32) -> Self {
        Self {
            message_type: MessageType::Heartbeat,
            sender_id,
            timestamp: get_current_timestamp(),
            data: MessageData::Heartbeat { task_id },
        }
    }
}

// 模拟函数
fn get_current_timestamp() -> u64 {
    unsafe {
        freertos_sys::xTaskGetTickCount() as u64
    }
}
```

## 完整示例应用程序

### 主应用程序结构

```rust
#![no_std]
#![no_main]

use freertos_rust::*;
use cortex_m_rt::entry;
use panic_halt as _;

mod freertos_integration;
mod memory_management;
mod task_management;
mod synchronization;
mod system_monitoring;
mod debug_tools;

use freertos_integration::*;
use memory_management::*;
use task_management::*;
use synchronization::*;
use system_monitoring::*;
use debug_tools::*;

// 全局系统状态
static mut SYSTEM_STATE: Option<SystemState> = None;

// 系统状态结构
struct SystemState {
    memory_manager: MemoryManager,
    task_manager: TaskManager,
    sync_manager: SynchronizationManager,
    system_monitor: SystemMonitor,
    debug_tools: DebugTools,
}

impl SystemState {
    fn new() -> Self {
        Self {
            memory_manager: MemoryManager::new(),
            task_manager: TaskManager::new(),
            sync_manager: SynchronizationManager::new(),
            system_monitor: SystemMonitor::new(),
            debug_tools: DebugTools::new(),
        }
    }
    
    fn initialize(&mut self) -> Result<(), FreeRtosError> {
        // 初始化内存管理器
        self.memory_manager.initialize()?;
        
        // 启用系统监控
        self.system_monitor.enable_monitoring();
        
        // 设置日志级别
        self.debug_tools.logger().set_min_level(LogLevel::Info);
        
        log_info!(self.debug_tools.logger(), "System initialized successfully");
        
        Ok(())
    }
}

// 获取全局系统状态
fn get_system_state() -> &'static mut SystemState {
    unsafe {
        SYSTEM_STATE.as_mut().unwrap()
    }
}

#[entry]
fn main() -> ! {
    // 初始化系统状态
    unsafe {
        SYSTEM_STATE = Some(SystemState::new());
    }
    
    let system = get_system_state();
    
    // 初始化系统
    if let Err(e) = system.initialize() {
        panic!("Failed to initialize system: {:?}", e);
    }
    
    // 创建应用程序任务
    create_application_tasks(system).expect("Failed to create tasks");
    
    // 启动FreeRTOS调度器
    FreeRtos::start_scheduler();
}

// 创建应用程序任务
fn create_application_tasks(system: &mut SystemState) -> Result<(), FreeRtosError> {
    // 创建传感器数据采集任务
    let sensor_task = Task::new()
        .name("SensorTask")
        .stack_size(2048)
        .priority(TaskPriority(3))
        .start(sensor_task_function)?;
    
    system.task_manager.register_task("sensor".to_string(), sensor_task);
    
    // 创建数据处理任务
    let processing_task = Task::new()
        .name("ProcessingTask")
        .stack_size(4096)
        .priority(TaskPriority(2))
        .start(data_processing_task_function)?;
    
    system.task_manager.register_task("processing".to_string(), processing_task);
    
    // 创建通信任务
    let communication_task = Task::new()
        .name("CommTask")
        .stack_size(3072)
        .priority(TaskPriority(2))
        .start(communication_task_function)?;
    
    system.task_manager.register_task("communication".to_string(), communication_task);
    
    // 创建系统监控任务
    let monitor_task = Task::new()
        .name("MonitorTask")
        .stack_size(2048)
        .priority(TaskPriority(1))
        .start(system_monitor_task_function)?;
    
    system.task_manager.register_task("monitor".to_string(), monitor_task);
    
    // 创建用户界面任务
    let ui_task = Task::new()
        .name("UITask")
        .stack_size(2048)
        .priority(TaskPriority(1))
        .start(user_interface_task_function)?;
    
    system.task_manager.register_task("ui".to_string(), ui_task);
    
    log_info!(system.debug_tools.logger(), "All application tasks created successfully");
    
    Ok(())
}

// 传感器数据采集任务
fn sensor_task_function(_: TaskHandle) {
    let system = get_system_state();
    
    log_info!(system.debug_tools.logger(), "Sensor task started");
    
    // 创建传感器数据队列
    let sensor_queue = Queue::<SensorData>::new(10).expect("Failed to create sensor queue");
    
    // 创建定时器用于周期性采集
    let mut timer = Timer::new(Duration::ms(100)).expect("Failed to create timer");
    
    loop {
        // 等待定时器
        timer.wait().expect("Timer wait failed");
        
        // 模拟传感器数据采集
        let sensor_data = SensorData {
            temperature: read_temperature_sensor(),
            humidity: read_humidity_sensor(),
            pressure: read_pressure_sensor(),
            timestamp: get_current_timestamp(),
        };
        
        // 发送数据到处理任务
        if let Err(e) = sensor_queue.send(sensor_data, Duration::ms(10)) {
            log_warn!(system.debug_tools.logger(), "Failed to send sensor data: {:?}", e);
        }
        
        // 更新任务统计
        system.task_manager.update_task_stats("sensor");
    }
}

// 数据处理任务
fn data_processing_task_function(_: TaskHandle) {
    let system = get_system_state();
    
    log_info!(system.debug_tools.logger(), "Data processing task started");
    
    // 创建处理结果队列
    let result_queue = Queue::<ProcessedData>::new(5).expect("Failed to create result queue");
    
    // 创建数据缓冲区
    let mut data_buffer = Vec::<SensorData>::new();
    
    loop {
        // 接收传感器数据
        if let Ok(sensor_data) = receive_sensor_data(Duration::ms(1000)) {
            data_buffer.push(sensor_data);
            
            // 当缓冲区有足够数据时进行处理
            if data_buffer.len() >= 5 {
                let processed_data = process_sensor_data(&data_buffer);
                
                // 发送处理结果
                if let Err(e) = result_queue.send(processed_data, Duration::ms(10)) {
                    log_warn!(system.debug_tools.logger(), "Failed to send processed data: {:?}", e);
                }
                
                // 清空缓冲区
                data_buffer.clear();
            }
        }
        
        // 更新任务统计
        system.task_manager.update_task_stats("processing");
    }
}

// 通信任务
fn communication_task_function(_: TaskHandle) {
    let system = get_system_state();
    
    log_info!(system.debug_tools.logger(), "Communication task started");
    
    // 创建通信缓冲区
    let mut tx_buffer = [0u8; 1024];
    let mut rx_buffer = [0u8; 1024];
    
    loop {
        // 接收处理后的数据
        if let Ok(processed_data) = receive_processed_data(Duration::ms(500)) {
            // 序列化数据
            if let Ok(serialized) = serialize_data(&processed_data, &mut tx_buffer) {
                // 发送数据
                if let Err(e) = send_data_over_network(&serialized) {
                    log_error!(system.debug_tools.logger(), "Failed to send data: {:?}", e);
                }
            }
        }
        
        // 检查接收到的命令
        if let Ok(command) = receive_network_command(&mut rx_buffer, Duration::ms(10)) {
            handle_network_command(command);
        }
        
        // 更新任务统计
        system.task_manager.update_task_stats("communication");
    }
}

// 系统监控任务
fn system_monitor_task_function(_: TaskHandle) {
    let system = get_system_state();
    
    log_info!(system.debug_tools.logger(), "System monitor task started");
    
    let mut report_timer = Timer::new(Duration::ms(5000)).expect("Failed to create report timer");
    
    loop {
        // 等待报告定时器
        report_timer.wait().expect("Report timer wait failed");
        
        // 更新系统统计
        if let Err(e) = system.system_monitor.update_task_stats() {
            log_error!(system.debug_tools.logger(), "Failed to update task stats: {:?}", e);
        }
        
        system.system_monitor.update_memory_stats(&system.memory_manager);
        
        // 检测系统异常
        let anomalies = system.system_monitor.detect_system_anomalies();
        if !anomalies.is_empty() {
            for anomaly in anomalies {
                log_warn!(system.debug_tools.logger(), "System anomaly detected: {:?}", anomaly);
            }
        }
        
        // 生成系统报告
        if let Ok(report) = system.system_monitor.generate_system_report() {
            log_info!(system.debug_tools.logger(), "System Report:\n{}", report);
        }
        
        // 更新任务统计
        system.task_manager.update_task_stats("monitor");
    }
}

// 用户界面任务
fn user_interface_task_function(_: TaskHandle) {
    let system = get_system_state();
    
    log_info!(system.debug_tools.logger(), "User interface task started");
    
    let mut ui_timer = Timer::new(Duration::ms(100)).expect("Failed to create UI timer");
    
    loop {
        // 等待UI更新定时器
        ui_timer.wait().expect("UI timer wait failed");
        
        // 更新显示
        update_display();
        
        // 处理用户输入
        if let Some(input) = read_user_input() {
            handle_user_input(input);
        }
        
        // 更新任务统计
        system.task_manager.update_task_stats("ui");
    }
}

// 数据结构定义
#[derive(Debug, Clone)]
struct SensorData {
    temperature: f32,
    humidity: f32,
    pressure: f32,
    timestamp: u64,
}

#[derive(Debug, Clone)]
struct ProcessedData {
    avg_temperature: f32,
    avg_humidity: f32,
    avg_pressure: f32,
    trend: DataTrend,
    timestamp: u64,
}

#[derive(Debug, Clone)]
enum DataTrend {
    Increasing,
    Decreasing,
    Stable,
}

#[derive(Debug, Clone)]
enum NetworkCommand {
    GetStatus,
    SetSamplingRate(u32),
    Reset,
    Shutdown,
}

#[derive(Debug, Clone)]
enum UserInput {
    ButtonPress(u8),
    MenuNavigation(MenuAction),
    ParameterChange(String, f32),
}

#[derive(Debug, Clone)]
enum MenuAction {
    Up,
    Down,
    Select,
    Back,
}

// 传感器读取函数（模拟）
fn read_temperature_sensor() -> f32 {
    // 模拟温度传感器读取
    25.0 + (get_current_timestamp() % 100) as f32 * 0.1
}

fn read_humidity_sensor() -> f32 {
    // 模拟湿度传感器读取
    50.0 + (get_current_timestamp() % 50) as f32 * 0.2
}

fn read_pressure_sensor() -> f32 {
    // 模拟压力传感器读取
    1013.25 + (get_current_timestamp() % 20) as f32 * 0.5
}

// 数据处理函数
fn process_sensor_data(data_buffer: &[SensorData]) -> ProcessedData {
    let avg_temperature = data_buffer.iter().map(|d| d.temperature).sum::<f32>() / data_buffer.len() as f32;
    let avg_humidity = data_buffer.iter().map(|d| d.humidity).sum::<f32>() / data_buffer.len() as f32;
    let avg_pressure = data_buffer.iter().map(|d| d.pressure).sum::<f32>() / data_buffer.len() as f32;
    
    // 简单趋势分析
    let trend = if data_buffer.len() >= 2 {
        let first_temp = data_buffer[0].temperature;
        let last_temp = data_buffer[data_buffer.len() - 1].temperature;
        
        if last_temp > first_temp + 1.0 {
            DataTrend::Increasing
        } else if last_temp < first_temp - 1.0 {
            DataTrend::Decreasing
        } else {
            DataTrend::Stable
        }
    } else {
        DataTrend::Stable
    };
    
    ProcessedData {
        avg_temperature,
        avg_humidity,
        avg_pressure,
        trend,
        timestamp: get_current_timestamp(),
    }
}

// 通信相关函数（模拟）
fn receive_sensor_data(timeout: Duration) -> Result<SensorData, FreeRtosError> {
    // 模拟从传感器队列接收数据
    // 实际实现中应该从真实的队列接收
    Err(FreeRtosError::Timeout)
}

fn receive_processed_data(timeout: Duration) -> Result<ProcessedData, FreeRtosError> {
    // 模拟从处理队列接收数据
    // 实际实现中应该从真实的队列接收
    Err(FreeRtosError::Timeout)
}

fn serialize_data(data: &ProcessedData, buffer: &mut [u8]) -> Result<&[u8], FreeRtosError> {
    // 模拟数据序列化
    // 实际实现中应该使用真实的序列化库
    Ok(&buffer[..100])
}

fn send_data_over_network(data: &[u8]) -> Result<(), FreeRtosError> {
    // 模拟网络发送
    // 实际实现中应该使用真实的网络接口
    Ok(())
}

fn receive_network_command(buffer: &mut [u8], timeout: Duration) -> Result<NetworkCommand, FreeRtosError> {
    // 模拟网络命令接收
    // 实际实现中应该从真实的网络接口接收
    Err(FreeRtosError::Timeout)
}

fn handle_network_command(command: NetworkCommand) {
    let system = get_system_state();
    
    match command {
        NetworkCommand::GetStatus => {
            log_info!(system.debug_tools.logger(), "Status request received");
            // 发送状态信息
        }
        NetworkCommand::SetSamplingRate(rate) => {
            log_info!(system.debug_tools.logger(), "Sampling rate set to: {}", rate);
            // 更新采样率
        }
        NetworkCommand::Reset => {
            log_info!(system.debug_tools.logger(), "Reset command received");
            // 执行系统重置
        }
        NetworkCommand::Shutdown => {
            log_info!(system.debug_tools.logger(), "Shutdown command received");
            // 执行系统关闭
        }
    }
}

// 用户界面相关函数（模拟）
fn update_display() {
    // 模拟显示更新
    // 实际实现中应该更新真实的显示设备
}

fn read_user_input() -> Option<UserInput> {
    // 模拟用户输入读取
    // 实际实现中应该从真实的输入设备读取
    None
}

fn handle_user_input(input: UserInput) {
    let system = get_system_state();
    
    match input {
        UserInput::ButtonPress(button_id) => {
            log_info!(system.debug_tools.logger(), "Button {} pressed", button_id);
            // 处理按钮按下
        }
        UserInput::MenuNavigation(action) => {
            log_info!(system.debug_tools.logger(), "Menu navigation: {:?}", action);
            // 处理菜单导航
        }
        UserInput::ParameterChange(param, value) => {
            log_info!(system.debug_tools.logger(), "Parameter {} changed to {}", param, value);
            // 处理参数更改
        }
    }
}
```

### 错误处理和恢复机制

```rust
use freertos_rust::*;

// 错误恢复管理器
pub struct ErrorRecoveryManager {
    error_handlers: Vec<ErrorHandler>,
    recovery_strategies: Vec<RecoveryStrategy>,
    error_history: Vec<SystemError>,
    max_history_size: usize,
}

#[derive(Debug, Clone)]
pub struct SystemError {
    pub error_type: ErrorType,
    pub severity: ErrorSeverity,
    pub timestamp: u64,
    pub task_name: String,
    pub description: String,
    pub recovery_attempted: bool,
    pub recovery_successful: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ErrorType {
    MemoryAllocationFailure,
    TaskCreationFailure,
    QueueOperationFailure,
    SemaphoreOperationFailure,
    TimerOperationFailure,
    HardwareFault,
    CommunicationError,
    DataCorruption,
    SystemOverload,
    UnknownError,
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum ErrorSeverity {
    Low,
    Medium,
    High,
    Critical,
    Fatal,
}

#[derive(Debug, Clone)]
pub struct ErrorHandler {
    pub error_type: ErrorType,
    pub handler_function: fn(&SystemError) -> RecoveryAction,
}

#[derive(Debug, Clone)]
pub struct RecoveryStrategy {
    pub error_type: ErrorType,
    pub severity_threshold: ErrorSeverity,
    pub recovery_function: fn(&SystemError) -> Result<(), FreeRtosError>,
    pub max_retry_count: u32,
}

#[derive(Debug, Clone)]
pub enum RecoveryAction {
    Ignore,
    Log,
    Retry,
    Restart,
    Shutdown,
    CustomAction(fn() -> Result<(), FreeRtosError>),
}

impl ErrorRecoveryManager {
    pub fn new() -> Self {
        Self {
            error_handlers: Vec::new(),
            recovery_strategies: Vec::new(),
            error_history: Vec::new(),
            max_history_size: 100,
        }
    }
    
    // 注册错误处理器
    pub fn register_error_handler(&mut self, error_type: ErrorType, handler: fn(&SystemError) -> RecoveryAction) {
        let error_handler = ErrorHandler {
            error_type,
            handler_function: handler,
        };
        
        self.error_handlers.push(error_handler);
    }
    
    // 注册恢复策略
    pub fn register_recovery_strategy(
        &mut self,
        error_type: ErrorType,
        severity_threshold: ErrorSeverity,
        recovery_function: fn(&SystemError) -> Result<(), FreeRtosError>,
        max_retry_count: u32,
    ) {
        let recovery_strategy = RecoveryStrategy {
            error_type,
            severity_threshold,
            recovery_function,
            max_retry_count,
        };
        
        self.recovery_strategies.push(recovery_strategy);
    }
    
    // 处理系统错误
    pub fn handle_error(&mut self, mut error: SystemError) -> Result<(), FreeRtosError> {
        // 记录错误到历史
        self.add_error_to_history(error.clone());
        
        // 查找对应的错误处理器
        if let Some(handler) = self.error_handlers.iter().find(|h| h.error_type == error.error_type) {
            let recovery_action = (handler.handler_function)(&error);
            
            match recovery_action {
                RecoveryAction::Ignore => {
                    // 忽略错误
                    return Ok(());
                }
                RecoveryAction::Log => {
                    // 仅记录错误
                    self.log_error(&error);
                    return Ok(());
                }
                RecoveryAction::Retry => {
                    // 尝试恢复
                    return self.attempt_recovery(&mut error);
                }
                RecoveryAction::Restart => {
                    // 重启系统
                    return self.restart_system();
                }
                RecoveryAction::Shutdown => {
                    // 关闭系统
                    return self.shutdown_system();
                }
                RecoveryAction::CustomAction(action) => {
                    // 执行自定义动作
                    return action();
                }
            }
        }
        
        // 如果没有找到特定的处理器，使用默认处理
        self.default_error_handling(&mut error)
    }
    
    // 尝试错误恢复
    fn attempt_recovery(&mut self, error: &mut SystemError) -> Result<(), FreeRtosError> {
        // 查找对应的恢复策略
        if let Some(strategy) = self.recovery_strategies.iter()
            .find(|s| s.error_type == error.error_type && error.severity >= s.severity_threshold) {
            
            error.recovery_attempted = true;
            
            // 尝试恢复
            for attempt in 0..strategy.max_retry_count {
                match (strategy.recovery_function)(error) {
                    Ok(()) => {
                        error.recovery_successful = true;
                        self.log_recovery_success(error, attempt + 1);
                        return Ok(());
                    }
                    Err(e) => {
                        self.log_recovery_attempt(error, attempt + 1, &e);
                        
                        // 如果是最后一次尝试，返回错误
                        if attempt == strategy.max_retry_count - 1 {
                            error.recovery_successful = false;
                            return Err(e);
                        }
                        
                        // 等待一段时间后重试
                        CurrentTask::delay(Duration::ms(100));
                    }
                }
            }
        }
        
        Err(FreeRtosError::RecoveryFailed)
    }
    
    // 默认错误处理
    fn default_error_handling(&mut self, error: &mut SystemError) -> Result<(), FreeRtosError> {
        match error.severity {
            ErrorSeverity::Low | ErrorSeverity::Medium => {
                self.log_error(error);
                Ok(())
            }
            ErrorSeverity::High => {
                self.log_error(error);
                self.attempt_recovery(error)
            }
            ErrorSeverity::Critical => {
                self.log_error(error);
                self.restart_system()
            }
            ErrorSeverity::Fatal => {
                self.log_error(error);
                self.shutdown_system()
            }
        }
    }
    
    // 添加错误到历史记录
    fn add_error_to_history(&mut self, error: SystemError) {
        if self.error_history.len() >= self.max_history_size {
            self.error_history.remove(0);
        }
        
        self.error_history.push(error);
    }
    
    // 记录错误
    fn log_error(&self, error: &SystemError) {
        // 这里应该使用实际的日志系统
        // log_error!(logger, "System error: {:?}", error);
    }
    
    // 记录恢复成功
    fn log_recovery_success(&self, error: &SystemError, attempts: u32) {
        // log_info!(logger, "Error recovery successful after {} attempts: {:?}", attempts, error);
    }
    
    // 记录恢复尝试
    fn log_recovery_attempt(&self, error: &SystemError, attempt: u32, recovery_error: &FreeRtosError) {
        // log_warn!(logger, "Recovery attempt {} failed for error {:?}: {:?}", attempt, error, recovery_error);
    }
    
    // 重启系统
    fn restart_system(&self) -> Result<(), FreeRtosError> {
        // 实现系统重启逻辑
        // 这里应该保存重要数据，然后重启系统
        Ok(())
    }
    
    // 关闭系统
    fn shutdown_system(&self) -> Result<(), FreeRtosError> {
        // 实现系统关闭逻辑
        // 这里应该安全地关闭所有任务和资源
        Ok(())
    }
    
    // 获取错误历史
    pub fn get_error_history(&self) -> &[SystemError] {
        &self.error_history
    }
    
    // 获取错误统计
    pub fn get_error_statistics(&self) -> ErrorStatistics {
        let mut stats = ErrorStatistics::default();
        
        for error in &self.error_history {
            stats.total_errors += 1;
            
            match error.severity {
                ErrorSeverity::Low => stats.low_severity_count += 1,
                ErrorSeverity::Medium => stats.medium_severity_count += 1,
                ErrorSeverity::High => stats.high_severity_count += 1,
                ErrorSeverity::Critical => stats.critical_severity_count += 1,
                ErrorSeverity::Fatal => stats.fatal_severity_count += 1,
            }
            
            if error.recovery_attempted {
                stats.recovery_attempts += 1;
                
                if error.recovery_successful {
                    stats.successful_recoveries += 1;
                }
            }
        }
        
        stats
    }
}

#[derive(Debug, Default)]
pub struct ErrorStatistics {
    pub total_errors: u32,
    pub low_severity_count: u32,
    pub medium_severity_count: u32,
    pub high_severity_count: u32,
    pub critical_severity_count: u32,
    pub fatal_severity_count: u32,
    pub recovery_attempts: u32,
    pub successful_recoveries: u32,
}

// 错误处理宏
#[macro_export]
macro_rules! handle_system_error {
    ($error_manager:expr, $error_type:expr, $severity:expr, $description:expr) => {
        {
            let error = SystemError {
                error_type: $error_type,
                severity: $severity,
                timestamp: get_current_timestamp(),
                task_name: get_current_task_name().unwrap_or_else(|| "Unknown".to_string()),
                description: $description.to_string(),
                recovery_attempted: false,
                recovery_successful: false,
            };
            
            $error_manager.handle_error(error)
        }
    };
}
```

## 事件组

### 事件同步

```rust
use freertos_rust::*;
use core::time::Duration;

// 事件组管理器
pub struct EventGroupManager {
    event_groups: Vec<EventGroupInfo>,
    next_group_id: u32,
}

#[derive(Debug, Clone)]
pub struct EventGroupInfo {
    pub id: u32,
    pub name: String,
    pub event_group: EventGroup,
    pub event_bits: u32,
    pub wait_count: u32,
    pub set_count: u32,
    pub clear_count: u32,
}

// 事件位定义
pub mod event_bits {
    pub const SENSOR_DATA_READY: u32 = 0x01;
    pub const COMMUNICATION_READY: u32 = 0x02;
    pub const SYSTEM_INITIALIZED: u32 = 0x04;
    pub const ERROR_OCCURRED: u32 = 0x08;
    pub const SHUTDOWN_REQUESTED: u32 = 0x10;
    pub const CALIBRATION_COMPLETE: u32 = 0x20;
    pub const DATA_PROCESSED: u32 = 0x40;
    pub const NETWORK_CONNECTED: u32 = 0x80;
}

impl EventGroupManager {
    pub fn new() -> Self {
        Self {
            event_groups: Vec::new(),
            next_group_id: 1,
        }
    }
    
    // 创建事件组
    pub fn create_event_group(&mut self, name: &str) -> Result<u32, FreeRtosError> {
        let group_id = self.next_group_id;
        self.next_group_id += 1;
        
        let event_group = EventGroup::new()?;
        
        let group_info = EventGroupInfo {
            id: group_id,
            name: name.to_string(),
            event_group,
            event_bits: 0,
            wait_count: 0,
            set_count: 0,
            clear_count: 0,
        };
        
        self.event_groups.push(group_info);
        Ok(group_id)
    }
    
    // 设置事件位
    pub fn set_event_bits(
        &mut self,
        group_id: u32,
        bits_to_set: u32,
    ) -> Result<u32, FreeRtosError> {
        if let Some(group_info) = self.event_groups.iter_mut().find(|g| g.id == group_id) {
            let result = group_info.event_group.set_bits(bits_to_set)?;
            group_info.event_bits |= bits_to_set;
            group_info.set_count += 1;
            Ok(result)
        } else {
            Err(FreeRtosError::EventGroupNotFound)
        }
    }
    
    // 清除事件位
    pub fn clear_event_bits(
        &mut self,
        group_id: u32,
        bits_to_clear: u32,
    ) -> Result<u32, FreeRtosError> {
        if let Some(group_info) = self.event_groups.iter_mut().find(|g| g.id == group_id) {
            let result = group_info.event_group.clear_bits(bits_to_clear)?;
            group_info.event_bits &= !bits_to_clear;
            group_info.clear_count += 1;
            Ok(result)
        } else {
            Err(FreeRtosError::EventGroupNotFound)
        }
    }
    
    // 等待事件位（所有位都设置）
    pub fn wait_for_all_bits(
        &mut self,
        group_id: u32,
        bits_to_wait_for: u32,
        clear_on_exit: bool,
        timeout: Duration,
    ) -> Result<u32, FreeRtosError> {
        if let Some(group_info) = self.event_groups.iter_mut().find(|g| g.id == group_id) {
            group_info.wait_count += 1;
            group_info.event_group.wait_bits(
                bits_to_wait_for,
                clear_on_exit,
                true, // wait for all bits
                timeout,
            )
        } else {
            Err(FreeRtosError::EventGroupNotFound)
        }
    }
    
    // 等待事件位（任意位设置）
    pub fn wait_for_any_bits(
        &mut self,
        group_id: u32,
        bits_to_wait_for: u32,
        clear_on_exit: bool,
        timeout: Duration,
    ) -> Result<u32, FreeRtosError> {
        if let Some(group_info) = self.event_groups.iter_mut().find(|g| g.id == group_id) {
            group_info.wait_count += 1;
            group_info.event_group.wait_bits(
                bits_to_wait_for,
                clear_on_exit,
                false, // wait for any bit
                timeout,
            )
        } else {
            Err(FreeRtosError::EventGroupNotFound)
        }
    }
    
    // 获取当前事件位
    pub fn get_event_bits(&self, group_id: u32) -> Result<u32, FreeRtosError> {
        if let Some(group_info) = self.event_groups.iter().find(|g| g.id == group_id) {
            Ok(group_info.event_group.get_bits())
        } else {
            Err(FreeRtosError::EventGroupNotFound)
        }
    }
    
    // 删除事件组
    pub fn delete_event_group(&mut self, group_id: u32) -> Result<(), FreeRtosError> {
        if let Some(pos) = self.event_groups.iter().position(|g| g.id == group_id) {
            self.event_groups.remove(pos);
            Ok(())
        } else {
            Err(FreeRtosError::EventGroupNotFound)
        }
    }
    
    // 获取事件组信息
    pub fn get_event_group_info(&self, group_id: u32) -> Option<&EventGroupInfo> {
        self.event_groups.iter().find(|g| g.id == group_id)
    }
}

// 事件组使用示例
pub struct SystemEventManager {
    event_manager: EventGroupManager,
    system_events_id: u32,
}

impl SystemEventManager {
    pub fn new() -> Result<Self, FreeRtosError> {
        let mut event_manager = EventGroupManager::new();
        let system_events_id = event_manager.create_event_group("SystemEvents")?;
        
        Ok(Self {
            event_manager,
            system_events_id,
        })
    }
    
    // 系统初始化完成
    pub fn signal_system_initialized(&mut self) -> Result<(), FreeRtosError> {
        self.event_manager.set_event_bits(
            self.system_events_id,
            event_bits::SYSTEM_INITIALIZED,
        )?;
        Ok(())
    }
    
    // 传感器数据就绪
    pub fn signal_sensor_data_ready(&mut self) -> Result<(), FreeRtosError> {
        self.event_manager.set_event_bits(
            self.system_events_id,
            event_bits::SENSOR_DATA_READY,
        )?;
        Ok(())
    }
    
    // 等待系统就绪
    pub fn wait_for_system_ready(&mut self, timeout: Duration) -> Result<(), FreeRtosError> {
        let required_bits = event_bits::SYSTEM_INITIALIZED | event_bits::COMMUNICATION_READY;
        
        self.event_manager.wait_for_all_bits(
            self.system_events_id,
            required_bits,
            false,
            timeout,
        )?;
        
        Ok(())
    }
    
    // 等待数据处理事件
    pub fn wait_for_data_events(&mut self, timeout: Duration) -> Result<u32, FreeRtosError> {
        let data_bits = event_bits::SENSOR_DATA_READY | event_bits::DATA_PROCESSED;
        
        self.event_manager.wait_for_any_bits(
            self.system_events_id,
            data_bits,
            true, // 清除事件位
            timeout,
        )
    }
}
```

## 任务通知

### 轻量级同步

```rust
use freertos_rust::*;
use core::time::Duration;

// 任务通知管理器
pub struct TaskNotificationManager {
    notification_handlers: Vec<NotificationHandlerInfo>,
}

#[derive(Debug, Clone)]
pub struct NotificationHandlerInfo {
    pub task_handle: TaskHandle,
    pub task_name: String,
    pub notification_count: u64,
    pub last_notification_value: u32,
    pub last_notification_time: u64,
}

// 通知值定义
pub mod notification_values {
    pub const SENSOR_UPDATE: u32 = 0x01;
    pub const TIMER_EXPIRED: u32 = 0x02;
    pub const DATA_READY: u32 = 0x04;
    pub const ERROR_CONDITION: u32 = 0x08;
    pub const SHUTDOWN_REQUEST: u32 = 0x10;
    pub const CALIBRATION_START: u32 = 0x20;
    pub const NETWORK_EVENT: u32 = 0x40;
    pub const USER_INPUT: u32 = 0x80;
}

impl TaskNotificationManager {
    pub fn new() -> Self {
        Self {
            notification_handlers: Vec::new(),
        }
    }
    
    // 注册任务通知处理程序
    pub fn register_task(
        &mut self,
        task_handle: TaskHandle,
        task_name: &str,
    ) {
        let handler_info = NotificationHandlerInfo {
            task_handle,
            task_name: task_name.to_string(),
            notification_count: 0,
            last_notification_value: 0,
            last_notification_time: 0,
        };
        
        self.notification_handlers.push(handler_info);
    }
    
    // 发送通知（设置位）
    pub fn notify_task_set_bits(
        &mut self,
        task_handle: TaskHandle,
        bits_to_set: u32,
    ) -> Result<(), FreeRtosError> {
        // 更新统计信息
        if let Some(handler) = self.notification_handlers.iter_mut()
            .find(|h| h.task_handle == task_handle) {
            handler.notification_count += 1;
            handler.last_notification_value = bits_to_set;
            handler.last_notification_time = get_current_timestamp();
        }
        
        // 发送通知
        unsafe {
            let result = freertos_sys::xTaskNotify(
                task_handle as *mut core::ffi::c_void,
                bits_to_set,
                freertos_sys::eNotifyAction_eSetBits,
            );
            
            if result == freertos_sys::pdPASS {
                Ok(())
            } else {
                Err(FreeRtosError::TaskNotifyFailed)
            }
        }
    }
    
    // 发送通知（增加值）
    pub fn notify_task_increment(
        &mut self,
        task_handle: TaskHandle,
    ) -> Result<(), FreeRtosError> {
        // 更新统计信息
        if let Some(handler) = self.notification_handlers.iter_mut()
            .find(|h| h.task_handle == task_handle) {
            handler.notification_count += 1;
            handler.last_notification_time = get_current_timestamp();
        }
        
        // 发送通知
        unsafe {
            let result = freertos_sys::xTaskNotify(
                task_handle as *mut core::ffi::c_void,
                0,
                freertos_sys::eNotifyAction_eIncrement,
            );
            
            if result == freertos_sys::pdPASS {
                Ok(())
            } else {
                Err(FreeRtosError::TaskNotifyFailed)
            }
        }
    }
    
    // 发送通知（覆盖值）
    pub fn notify_task_set_value(
        &mut self,
        task_handle: TaskHandle,
        value: u32,
    ) -> Result<(), FreeRtosError> {
        // 更新统计信息
        if let Some(handler) = self.notification_handlers.iter_mut()
            .find(|h| h.task_handle == task_handle) {
            handler.notification_count += 1;
            handler.last_notification_value = value;
            handler.last_notification_time = get_current_timestamp();
        }
        
        // 发送通知
        unsafe {
            let result = freertos_sys::xTaskNotify(
                task_handle as *mut core::ffi::c_void,
                value,
                freertos_sys::eNotifyAction_eSetValueWithOverwrite,
            );
            
            if result == freertos_sys::pdPASS {
                Ok(())
            } else {
                Err(FreeRtosError::TaskNotifyFailed)
            }
        }
    }
    
    // 等待通知
    pub fn wait_for_notification(
        &self,
        clear_count_on_exit: bool,
        timeout: Duration,
    ) -> Result<u32, FreeRtosError> {
        unsafe {
            let mut notification_value = 0u32;
            let result = freertos_sys::xTaskNotifyWait(
                0, // 不清除进入时的位
                if clear_count_on_exit { 0xFFFFFFFF } else { 0 }, // 退出时清除的位
                &mut notification_value,
                timeout.as_millis() as u32,
            );
            
            if result == freertos_sys::pdTRUE {
                Ok(notification_value)
            } else {
                Err(FreeRtosError::Timeout)
            }
        }
    }
    
    // 获取通知统计信息
    pub fn get_notification_stats(&self, task_handle: TaskHandle) -> Option<&NotificationHandlerInfo> {
        self.notification_handlers.iter().find(|h| h.task_handle == task_handle)
    }
    
    // 获取所有通知统计信息
    pub fn get_all_notification_stats(&self) -> &[NotificationHandlerInfo] {
        &self.notification_handlers
    }
}

// 任务通知使用示例
pub struct NotificationBasedTask {
    task_handle: TaskHandle,
    notification_manager: TaskNotificationManager,
}

impl NotificationBasedTask {
    pub fn new() -> Self {
        let task_handle = CurrentTask::get_handle();
        let mut notification_manager = TaskNotificationManager::new();
        notification_manager.register_task(task_handle, "NotificationTask");
        
        Self {
            task_handle,
            notification_manager,
        }
    }
    
    // 任务主循环
    pub fn run(&mut self) {
        loop {
            // 等待通知
            match self.notification_manager.wait_for_notification(
                true, // 清除计数
                Duration::from_millis(1000),
            ) {
                Ok(notification_value) => {
                    self.handle_notification(notification_value);
                }
                Err(FreeRtosError::Timeout) => {
                    // 超时处理
                    self.handle_timeout();
                }
                Err(_) => {
                    // 错误处理
                    self.handle_error();
                }
            }
        }
    }
    
    // 处理通知
    fn handle_notification(&self, notification_value: u32) {
        if notification_value & notification_values::SENSOR_UPDATE != 0 {
            self.handle_sensor_update();
        }
        
        if notification_value & notification_values::TIMER_EXPIRED != 0 {
            self.handle_timer_expired();
        }
        
        if notification_value & notification_values::DATA_READY != 0 {
            self.handle_data_ready();
        }
        
        if notification_value & notification_values::ERROR_CONDITION != 0 {
            self.handle_error_condition();
        }
    }
    
    fn handle_sensor_update(&self) {
        // 处理传感器更新
    }
    
    fn handle_timer_expired(&self) {
        // 处理定时器到期
    }
    
    fn handle_data_ready(&self) {
        // 处理数据就绪
    }
    
    fn handle_error_condition(&self) {
        // 处理错误条件
    }
    
    fn handle_timeout(&self) {
        // 处理超时
    }
    
    fn handle_error(&self) {
        // 处理错误
    }
}
```

## 软件定时器

### 定时器管理

```rust
use freertos_rust::*;
use core::time::Duration;

// 软件定时器管理器
pub struct SoftwareTimerManager {
    timers: Vec<TimerInfo>,
    next_timer_id: u32,
}

#[derive(Debug, Clone)]
pub struct TimerInfo {
    pub id: u32,
    pub name: String,
    pub timer: Timer,
    pub period: Duration,
    pub auto_reload: bool,
    pub callback_count: u64,
    pub last_callback_time: u64,
    pub is_active: bool,
}

// 定时器回调函数类型
type TimerCallback = fn(timer_id: u32);

// 定时器回调注册表
static mut TIMER_CALLBACKS: [Option<TimerCallback>; 32] = [None; 32];
static mut CALLBACK_COUNT: usize = 0;

impl SoftwareTimerManager {
    pub fn new() -> Self {
        Self {
            timers: Vec::new(),
            next_timer_id: 1,
        }
    }
    
    // 创建定时器
    pub fn create_timer(
        &mut self,
        name: &str,
        period: Duration,
        auto_reload: bool,
        callback: TimerCallback,
    ) -> Result<u32, FreeRtosError> {
        let timer_id = self.next_timer_id;
        self.next_timer_id += 1;
        
        // 注册回调函数
        unsafe {
            if CALLBACK_COUNT < TIMER_CALLBACKS.len() {
                TIMER_CALLBACKS[CALLBACK_COUNT] = Some(callback);
                CALLBACK_COUNT += 1;
            } else {
                return Err(FreeRtosError::OutOfMemory);
            }
        }
        
        // 创建FreeRTOS定时器
        let timer = Timer::new(
            name,
            period,
            auto_reload,
            timer_callback_wrapper,
        )?;
        
        let timer_info = TimerInfo {
            id: timer_id,
            name: name.to_string(),
            timer,
            period,
            auto_reload,
            callback_count: 0,
            last_callback_time: 0,
            is_active: false,
        };
        
        self.timers.push(timer_info);
        Ok(timer_id)
    }
    
    // 启动定时器
    pub fn start_timer(&mut self, timer_id: u32) -> Result<(), FreeRtosError> {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.timer.start(Duration::from_millis(0))?;
            timer_info.is_active = true;
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 停止定时器
    pub fn stop_timer(&mut self, timer_id: u32) -> Result<(), FreeRtosError> {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.timer.stop(Duration::from_millis(0))?;
            timer_info.is_active = false;
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 重置定时器
    pub fn reset_timer(&mut self, timer_id: u32) -> Result<(), FreeRtosError> {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.timer.reset(Duration::from_millis(0))?;
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 更改定时器周期
    pub fn change_timer_period(
        &mut self,
        timer_id: u32,
        new_period: Duration,
    ) -> Result<(), FreeRtosError> {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.timer.change_period(new_period, Duration::from_millis(0))?;
            timer_info.period = new_period;
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 删除定时器
    pub fn delete_timer(&mut self, timer_id: u32) -> Result<(), FreeRtosError> {
        if let Some(pos) = self.timers.iter().position(|t| t.id == timer_id) {
            let timer_info = &self.timers[pos];
            timer_info.timer.delete(Duration::from_millis(0))?;
            self.timers.remove(pos);
            Ok(())
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
    
    // 获取定时器信息
    pub fn get_timer_info(&self, timer_id: u32) -> Option<&TimerInfo> {
        self.timers.iter().find(|t| t.id == timer_id)
    }
    
    // 获取所有定时器信息
    pub fn get_all_timer_info(&self) -> &[TimerInfo] {
        &self.timers
    }
    
    // 更新定时器统计信息
    pub fn update_timer_stats(&mut self, timer_id: u32) {
        if let Some(timer_info) = self.timers.iter_mut().find(|t| t.id == timer_id) {
            timer_info.callback_count += 1;
            timer_info.last_callback_time = get_current_timestamp();
        }
    }
}

// 定时器回调包装函数
extern "C" fn timer_callback_wrapper(timer_handle: *mut core::ffi::c_void) {
    // 从定时器句柄获取定时器ID
    let timer_id = unsafe {
        freertos_sys::pvTimerGetTimerID(timer_handle) as u32
    };
    
    // 调用注册的回调函数
    unsafe {
        for (index, callback_opt) in TIMER_CALLBACKS.iter().enumerate() {
            if let Some(callback) = callback_opt {
                if index as u32 == timer_id {
                    callback(timer_id);
                    break;
                }
            }
        }
    }
}

// 定时器使用示例
pub struct SystemTimerManager {
    timer_manager: SoftwareTimerManager,
    heartbeat_timer_id: Option<u32>,
    watchdog_timer_id: Option<u32>,
    data_collection_timer_id: Option<u32>,
}

impl SystemTimerManager {
    pub fn new() -> Self {
        Self {
            timer_manager: SoftwareTimerManager::new(),
            heartbeat_timer_id: None,
            watchdog_timer_id: None,
            data_collection_timer_id: None,
        }
    }
    
    // 初始化系统定时器
    pub fn initialize(&mut self) -> Result<(), FreeRtosError> {
        // 创建心跳定时器
        let heartbeat_id = self.timer_manager.create_timer(
            "HeartbeatTimer",
            Duration::from_millis(1000),
            true, // 自动重载
            heartbeat_callback,
        )?;
        self.heartbeat_timer_id = Some(heartbeat_id);
        
        // 创建看门狗定时器
        let watchdog_id = self.timer_manager.create_timer(
            "WatchdogTimer",
            Duration::from_millis(5000),
            false, // 单次触发
            watchdog_callback,
        )?;
        self.watchdog_timer_id = Some(watchdog_id);
        
        // 创建数据采集定时器
        let data_collection_id = self.timer_manager.create_timer(
            "DataCollectionTimer",
            Duration::from_millis(100),
            true, // 自动重载
            data_collection_callback,
        )?;
        self.data_collection_timer_id = Some(data_collection_id);
        
        Ok(())
    }
    
    // 启动所有定时器
    pub fn start_all_timers(&mut self) -> Result<(), FreeRtosError> {
        if let Some(id) = self.heartbeat_timer_id {
            self.timer_manager.start_timer(id)?;
        }
        
        if let Some(id) = self.watchdog_timer_id {
            self.timer_manager.start_timer(id)?;
        }
        
        if let Some(id) = self.data_collection_timer_id {
            self.timer_manager.start_timer(id)?;
        }
        
        Ok(())
    }
    
    // 停止所有定时器
    pub fn stop_all_timers(&mut self) -> Result<(), FreeRtosError> {
        if let Some(id) = self.heartbeat_timer_id {
            self.timer_manager.stop_timer(id)?;
        }
        
        if let Some(id) = self.watchdog_timer_id {
            self.timer_manager.stop_timer(id)?;
        }
        
        if let Some(id) = self.data_collection_timer_id {
            self.timer_manager.stop_timer(id)?;
        }
        
        Ok(())
    }
    
    // 重置看门狗定时器
    pub fn reset_watchdog(&mut self) -> Result<(), FreeRtosError> {
        if let Some(id) = self.watchdog_timer_id {
            self.timer_manager.reset_timer(id)
        } else {
            Err(FreeRtosError::TimerNotFound)
        }
    }
}

// 定时器回调函数
fn heartbeat_callback(timer_id: u32) {
    // 心跳处理
    // 可以在这里发送心跳信号、更新LED状态等
}

fn watchdog_callback(timer_id: u32) {
    // 看门狗超时处理
    // 系统重启或错误处理
}

fn data_collection_callback(timer_id: u32) {
    // 数据采集处理
    // 触发传感器读取、数据处理等
}
```

## 内存管理

### 动态内存分配

```rust
use freertos_rust::*;
use core::alloc::{GlobalAlloc, Layout};
use core::ptr;

// FreeRTOS内存分配器
pub struct FreeRtosAllocator;

unsafe impl GlobalAlloc for FreeRtosAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        freertos_sys::pvPortMalloc(layout.size()) as *mut u8
    }
    
    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
        freertos_sys::vPortFree(ptr as *mut core::ffi::c_void);
    }
}

// 全局分配器
#[global_allocator]
static ALLOCATOR: FreeRtosAllocator = FreeRtosAllocator;

// 内存管理器
pub struct MemoryManager {
    allocations: Vec<AllocationInfo>,
    total_allocated: usize,
    peak_allocated: usize,
    allocation_count: u64,
    deallocation_count: u64,
}

#[derive(Debug, Clone)]
pub struct AllocationInfo {
    pub ptr: *mut u8,
    pub size: usize,
    pub timestamp: u64,
    pub file: &'static str,
    pub line: u32,
}

#[derive(Debug, Clone)]
pub struct MemoryStats {
    pub total_heap_size: usize,
    pub free_heap_size: usize,
    pub minimum_ever_free_heap_size: usize,
    pub number_of_free_blocks: usize,
    pub total_allocated: usize,
    pub peak_allocated: usize,
    pub allocation_count: u64,
    pub deallocation_count: u64,
}

impl MemoryManager {
    pub fn new() -> Self {
        Self {
            allocations: Vec::new(),
            total_allocated: 0,
            peak_allocated: 0,
            allocation_count: 0,
            deallocation_count: 0,
        }
    }
    
    // 分配内存（带跟踪）
    pub fn allocate(
        &mut self,
        size: usize,
        file: &'static str,
        line: u32,
    ) -> Result<*mut u8, FreeRtosError> {
        unsafe {
            let ptr = freertos_sys::pvPortMalloc(size) as *mut u8;
            
            if ptr.is_null() {
                return Err(FreeRtosError::OutOfMemory);
            }
            
            // 记录分配信息
            let allocation_info = AllocationInfo {
                ptr,
                size,
                timestamp: get_current_timestamp(),
                file,
                line,
            };
            
            self.allocations.push(allocation_info);
            self.total_allocated += size;
            self.allocation_count += 1;
            
            if self.total_allocated > self.peak_allocated {
                self.peak_allocated = self.total_allocated;
            }
            
            Ok(ptr)
        }
    }
    
    // 释放内存（带跟踪）
    pub fn deallocate(&mut self, ptr: *mut u8) -> Result<(), FreeRtosError> {
        // 查找分配记录
        if let Some(pos) = self.allocations.iter().position(|a| a.ptr == ptr) {
            let allocation_info = self.allocations.remove(pos);
            self.total_allocated -= allocation_info.size;
            self.deallocation_count += 1;
            
            unsafe {
                freertos_sys::vPortFree(ptr as *mut core::ffi::c_void);
            }
            
            Ok(())
        } else {
            Err(FreeRtosError::InvalidPointer)
        }
    }
    
    // 重新分配内存
    pub fn reallocate(
        &mut self,
        ptr: *mut u8,
        new_size: usize,
        file: &'static str,
        line: u32,
    ) -> Result<*mut u8, FreeRtosError> {
        if ptr.is_null() {
            return self.allocate(new_size, file, line);
        }
        
        // 查找原始分配记录
        if let Some(allocation) = self.allocations.iter().find(|a| a.ptr == ptr) {
            let old_size = allocation.size;
            
            // 分配新内存
            let new_ptr = self.allocate(new_size, file, line)?;
            
            // 复制数据
            unsafe {
                let copy_size = if old_size < new_size { old_size } else { new_size };
                ptr::copy_nonoverlapping(ptr, new_ptr, copy_size);
            }
            
            // 释放旧内存
            self.deallocate(ptr)?;
            
            Ok(new_ptr)
        } else {
            Err(FreeRtosError::InvalidPointer)
        }
    }
    
    // 获取内存统计信息
    pub fn get_memory_stats(&self) -> MemoryStats {
        unsafe {
            MemoryStats {
                total_heap_size: freertos_sys::configTOTAL_HEAP_SIZE as usize,
                free_heap_size: freertos_sys::xPortGetFreeHeapSize() as usize,
                minimum_ever_free_heap_size: freertos_sys::xPortGetMinimumEverFreeHeapSize() as usize,
                number_of_free_blocks: 0, // FreeRTOS没有直接提供这个信息
                total_allocated: self.total_allocated,
                peak_allocated: self.peak_allocated,
                allocation_count: self.allocation_count,
                deallocation_count: self.deallocation_count,
            }
        }
    }
    
    // 检查内存泄漏
    pub fn check_memory_leaks(&self) -> Vec<&AllocationInfo> {
        self.allocations.iter().collect()
    }
    
    // 清理所有分配的内存
    pub fn cleanup_all(&mut self) {
        for allocation in &self.allocations {
            unsafe {
                freertos_sys::vPortFree(allocation.ptr as *mut core::ffi::c_void);
            }
        }
        
        self.allocations.clear();
        self.total_allocated = 0;
    }
}

// 内存分配宏
#[macro_export]
macro_rules! freertos_malloc {
    ($manager:expr, $size:expr) => {
        $manager.allocate($size, file!(), line!())
    };
}

#[macro_export]
macro_rules! freertos_free {
    ($manager:expr, $ptr:expr) => {
        $manager.deallocate($ptr)
    };
}

#[macro_export]
macro_rules! freertos_realloc {
    ($manager:expr, $ptr:expr, $size:expr) => {
        $manager.reallocate($ptr, $size, file!(), line!())
    };
}

// 内存池管理
pub struct MemoryPool {
    pool_ptr: *mut u8,
    block_size: usize,
    block_count: usize,
    free_blocks: Vec<*mut u8>,
    allocated_blocks: Vec<*mut u8>,
}

impl MemoryPool {
    // 创建内存池
    pub fn new(
        block_size: usize,
        block_count: usize,
    ) -> Result<Self, FreeRtosError> {
        let total_size = block_size * block_count;
        
        unsafe {
            let pool_ptr = freertos_sys::pvPortMalloc(total_size) as *mut u8;
            
            if pool_ptr.is_null() {
                return Err(FreeRtosError::OutOfMemory);
            }
            
            let mut free_blocks = Vec::new();
            
            // 初始化空闲块列表
            for i in 0..block_count {
                let block_ptr = pool_ptr.add(i * block_size);
                free_blocks.push(block_ptr);
            }
            
            Ok(Self {
                pool_ptr,
                block_size,
                block_count,
                free_blocks,
                allocated_blocks: Vec::new(),
            })
        }
    }
    
    // 从内存池分配块
    pub fn allocate_block(&mut self) -> Option<*mut u8> {
        if let Some(block_ptr) = self.free_blocks.pop() {
            self.allocated_blocks.push(block_ptr);
            Some(block_ptr)
        } else {
            None
        }
    }
    
    // 释放块到内存池
    pub fn deallocate_block(&mut self, ptr: *mut u8) -> Result<(), FreeRtosError> {
        if let Some(pos) = self.allocated_blocks.iter().position(|&p| p == ptr) {
            self.allocated_blocks.remove(pos);
            self.free_blocks.push(ptr);
            Ok(())
        } else {
            Err(FreeRtosError::InvalidPointer)
        }
    }
    
    // 获取可用块数量
    pub fn available_blocks(&self) -> usize {
        self.free_blocks.len()
    }
    
    // 获取已分配块数量
    pub fn allocated_blocks(&self) -> usize {
        self.allocated_blocks.len()
    }
    
    // 获取块大小
    pub fn block_size(&self) -> usize {
        self.block_size
    }
}

impl Drop for MemoryPool {
    fn drop(&mut self) {
        unsafe {
            freertos_sys::vPortFree(self.pool_ptr as *mut core::ffi::c_void);
        }
    }
}
```

## 系统监控和调试

### 系统状态监控

```rust
use freertos_rust::*;
use core::fmt::Write;

// 系统监控器
pub struct SystemMonitor {
    task_stats: Vec<TaskStats>,
    memory_stats: MemoryStats,
    system_stats: SystemStats,
    monitoring_enabled: bool,
    stats_buffer: [u8; 4096],
}

#[derive(Debug, Clone)]
pub struct TaskStats {
    pub name: String,
    pub handle: TaskHandle,
    pub state: TaskState,
    pub priority: u32,
    pub stack_high_water_mark: u32,
    pub runtime_counter: u64,
    pub runtime_percentage: f32,
}

#[derive(Debug, Clone)]
pub struct SystemStats {
    pub total_runtime: u64,
    pub idle_runtime: u64,
    pub cpu_usage_percentage: f32,
    pub tick_count: u32,
    pub uptime_seconds: u64,
    pub context_switches: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TaskState {
    Running,
    Ready,
    Blocked,
    Suspended,
    Deleted,
    Invalid,
}

impl SystemMonitor {
    pub fn new() -> Self {
        Self {
            task_stats: Vec::new(),
            memory_stats: MemoryStats {
                total_heap_size: 0,
                free_heap_size: 0,
                minimum_ever_free_heap_size: 0,
                number_of_free_blocks: 0,
                total_allocated: 0,
                peak_allocated: 0,
                allocation_count: 0,
                deallocation_count: 0,
            },
            system_stats: SystemStats {
                total_runtime: 0,
                idle_runtime: 0,
                cpu_usage_percentage: 0.0,
                tick_count: 0,
                uptime_seconds: 0,
                context_switches: 0,
            },
            monitoring_enabled: false,
            stats_buffer: [0; 4096],
        }
    }
    
    // 启用监控
    pub fn enable_monitoring(&mut self) {
        self.monitoring_enabled = true;
        
        // 启用FreeRTOS运行时统计
        unsafe {
            freertos_sys::vTaskStartScheduler();
        }
    }
    
    // 禁用监控
    pub fn disable_monitoring(&mut self) {
        self.monitoring_enabled = false;
    }
    
    // 更新任务统计信息
    pub fn update_task_stats(&mut self) -> Result<(), FreeRtosError> {
        if !self.monitoring_enabled {
            return Ok(());
        }
        
        self.task_stats.clear();
        
        unsafe {
            let task_count = freertos_sys::uxTaskGetNumberOfTasks();
            let mut task_status_array = vec![freertos_sys::TaskStatus_t::default(); task_count as usize];
            let mut total_runtime = 0u32;
            
            let actual_count = freertos_sys::uxTaskGetSystemState(
                task_status_array.as_mut_ptr(),
                task_count,
                &mut total_runtime,
            );
            
            for i in 0..actual_count {
                let task_status = &task_status_array[i as usize];
                
                let task_name = core::str::from_utf8_unchecked(
                    core::slice::from_raw_parts(
                        task_status.pcTaskName as *const u8,
                        strlen(task_status.pcTaskName),
                    )
                ).to_string();
                
                let state = match task_status.eCurrentState {
                    0 => TaskState::Running,
                    1 => TaskState::Ready,
                    2 => TaskState::Blocked,
                    3 => TaskState::Suspended,
                    4 => TaskState::Deleted,
                    _ => TaskState::Invalid,
                };
                
                let runtime_percentage = if total_runtime > 0 {
                    (task_status.ulRunTimeCounter as f32 / total_runtime as f32) * 100.0
                } else {
                    0.0
                };
                
                let task_stats = TaskStats {
                    name: task_name,
                    handle: TaskHandle::from_raw(task_status.xHandle),
                    state,
                    priority: task_status.uxCurrentPriority,
                    stack_high_water_mark: task_status.usStackHighWaterMark as u32,
                    runtime_counter: task_status.ulRunTimeCounter as u64,
                    runtime_percentage,
                };
                
                self.task_stats.push(task_stats);
            }
            
            // 更新系统统计信息
            self.system_stats.total_runtime = total_runtime as u64;
            self.system_stats.tick_count = freertos_sys::xTaskGetTickCount();
            self.system_stats.uptime_seconds = self.system_stats.tick_count as u64 / 1000; // 假设tick频率为1000Hz
        }
        
        Ok(())
    }
    
    // 更新内存统计信息
    pub fn update_memory_stats(&mut self, memory_manager: &MemoryManager) {
        if !self.monitoring_enabled {
            return;
        }
        
        self.memory_stats = memory_manager.get_memory_stats();
    }
    
    // 获取CPU使用率
    pub fn get_cpu_usage(&self) -> f32 {
        // 计算CPU使用率（100% - 空闲任务使用率）
        if let Some(idle_task) = self.task_stats.iter().find(|t| t.name.contains("IDLE")) {
            100.0 - idle_task.runtime_percentage
        } else {
            0.0
        }
    }
    
    // 获取任务统计信息
    pub fn get_task_stats(&self) -> &[TaskStats] {
        &self.task_stats
    }
    
    // 获取内存统计信息
    pub fn get_memory_stats(&self) -> &MemoryStats {
        &self.memory_stats
    }
    
    // 获取系统统计信息
    pub fn get_system_stats(&self) -> &SystemStats {
        &self.system_stats
    }
    
    // 生成系统报告
    pub fn generate_system_report(&mut self) -> Result<&str, FreeRtosError> {
        if !self.monitoring_enabled {
            return Err(FreeRtosError::MonitoringDisabled);
        }
        
        self.update_task_stats()?;
        
        let mut buffer = &mut self.stats_buffer[..];
        let mut writer = BufferWriter::new(buffer);
        
        writeln!(writer, "=== 系统监控报告 ===").map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "系统运行时间: {} 秒", self.system_stats.uptime_seconds).map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "CPU使用率: {:.2}%", self.get_cpu_usage()).map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "Tick计数: {}", self.system_stats.tick_count).map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "").map_err(|_| FreeRtosError::BufferOverflow)?;
        
        writeln!(writer, "=== 内存使用情况 ===").map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "总堆大小: {} 字节", self.memory_stats.total_heap_size).map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "可用堆大小: {} 字节", self.memory_stats.free_heap_size).map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "最小可用堆: {} 字节", self.memory_stats.minimum_ever_free_heap_size).map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "已分配内存: {} 字节", self.memory_stats.total_allocated).map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "峰值分配: {} 字节", self.memory_stats.peak_allocated).map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "").map_err(|_| FreeRtosError::BufferOverflow)?;
        
        writeln!(writer, "=== 任务状态 ===").map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "{:<16} {:<10} {:<8} {:<12} {:<8}", "任务名", "状态", "优先级", "栈水位", "CPU%").map_err(|_| FreeRtosError::BufferOverflow)?;
        writeln!(writer, "{}", "-".repeat(60)).map_err(|_| FreeRtosError::BufferOverflow)?;
        
        for task in &self.task_stats {
            let state_str = match task.state {
                TaskState::Running => "运行中",
                TaskState::Ready => "就绪",
                TaskState::Blocked => "阻塞",
                TaskState::Suspended => "挂起",
                TaskState::Deleted => "已删除",
                TaskState::Invalid => "无效",
            };
            
            writeln!(
                writer,
                "{:<16} {:<10} {:<8} {:<12} {:<8.2}",
                task.name,
                state_str,
                task.priority,
                task.stack_high_water_mark,
                task.runtime_percentage
            ).map_err(|_| FreeRtosError::BufferOverflow)?;
        }
        
        let report_len = writer.bytes_written();
        let report_str = core::str::from_utf8(&self.stats_buffer[..report_len])
            .map_err(|_| FreeRtosError::InvalidData)?;
        
        Ok(report_str)
    }
    
    // 检测系统异常
    pub fn detect_system_anomalies(&self) -> Vec<SystemAnomaly> {
        let mut anomalies = Vec::new();
        
        // 检查内存使用率
        let memory_usage_percentage = 
            ((self.memory_stats.total_heap_size - self.memory_stats.free_heap_size) as f32 
             / self.memory_stats.total_heap_size as f32) * 100.0;
        
        if memory_usage_percentage > 90.0 {
            anomalies.push(SystemAnomaly::HighMemoryUsage(memory_usage_percentage));
        }
        
        // 检查CPU使用率
        let cpu_usage = self.get_cpu_usage();
        if cpu_usage > 95.0 {
            anomalies.push(SystemAnomaly::HighCpuUsage(cpu_usage));
        }
        
        // 检查栈溢出风险
        for task in &self.task_stats {
            if task.stack_high_water_mark < 100 { // 栈剩余空间小于100字节
                anomalies.push(SystemAnomaly::LowStackSpace {
                    task_name: task.name.clone(),
                    remaining_bytes: task.stack_high_water_mark,
                });
            }
        }
        
        // 检查任务阻塞时间过长
        for task in &self.task_stats {
            if task.state == TaskState::Blocked && task.runtime_percentage < 0.1 {
                anomalies.push(SystemAnomaly::TaskStarvation {
                    task_name: task.name.clone(),
                    runtime_percentage: task.runtime_percentage,
                });
            }
        }
        
        anomalies
    }
}

#[derive(Debug, Clone)]
pub enum SystemAnomaly {
    HighMemoryUsage(f32),
    HighCpuUsage(f32),
    LowStackSpace {
        task_name: String,
        remaining_bytes: u32,
    },
    TaskStarvation {
        task_name: String,
        runtime_percentage: f32,
    },
}

// 缓冲区写入器
struct BufferWriter<'a> {
    buffer: &'a mut [u8],
    position: usize,
}

impl<'a> BufferWriter<'a> {
    fn new(buffer: &'a mut [u8]) -> Self {
        Self {
            buffer,
            position: 0,
        }
    }
    
    fn bytes_written(&self) -> usize {
        self.position
    }
}

impl<'a> Write for BufferWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        if self.position + bytes.len() > self.buffer.len() {
            return Err(core::fmt::Error);
        }
        
        self.buffer[self.position..self.position + bytes.len()].copy_from_slice(bytes);
        self.position += bytes.len();
        Ok(())
    }
}

// 辅助函数：计算C字符串长度
fn strlen(ptr: *const i8) -> usize {
    let mut len = 0;
    unsafe {
        while *ptr.add(len) != 0 {
            len += 1;
        }
    }
    len
}
```

### 调试和日志系统

```rust
use freertos_rust::*;
use core::fmt::Write;

// 日志级别
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
    Fatal = 5,
}

// 日志条目
#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: u64,
    pub level: LogLevel,
    pub task_name: String,
    pub message: String,
    pub file: &'static str,
    pub line: u32,
}

// 日志系统
pub struct Logger {
    entries: Vec<LogEntry>,
    max_entries: usize,
    min_level: LogLevel,
    output_buffer: [u8; 1024],
    enabled: bool,
}

impl Logger {
    pub fn new(max_entries: usize) -> Self {
        Self {
            entries: Vec::new(),
            max_entries,
            min_level: LogLevel::Info,
            output_buffer: [0; 1024],
            enabled: true,
        }
    }
    
    // 设置最小日志级别
    pub fn set_min_level(&mut self, level: LogLevel) {
        self.min_level = level;
    }
    
    // 启用/禁用日志
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    // 记录日志
    pub fn log(
        &mut self,
        level: LogLevel,
        message: &str,
        file: &'static str,
        line: u32,
    ) {
        if !self.enabled || level < self.min_level {
            return;
        }
        
        let task_name = get_current_task_name().unwrap_or_else(|| "Unknown".to_string());
        
        let entry = LogEntry {
            timestamp: get_current_timestamp(),
            level,
            task_name,
            message: message.to_string(),
            file,
            line,
        };
        
        // 如果日志条目数量超过最大值，删除最旧的条目
        if self.entries.len() >= self.max_entries {
            self.entries.remove(0);
        }
        
        self.entries.push(entry);
    }
    
    // 获取所有日志条目
    pub fn get_entries(&self) -> &[LogEntry] {
        &self.entries
    }
    
    // 获取指定级别的日志条目
    pub fn get_entries_by_level(&self, level: LogLevel) -> Vec<&LogEntry> {
        self.entries.iter().filter(|e| e.level == level).collect()
    }
    
    // 清空日志
    pub fn clear(&mut self) {
        self.entries.clear();
    }
    
    // 格式化日志条目
    pub fn format_entry(&mut self, entry: &LogEntry) -> Result<&str, FreeRtosError> {
        let mut buffer = &mut self.output_buffer[..];
        let mut writer = BufferWriter::new(buffer);
        
        let level_str = match entry.level {
            LogLevel::Trace => "TRACE",
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO ",
            LogLevel::Warn => "WARN ",
            LogLevel::Error => "ERROR",
            LogLevel::Fatal => "FATAL",
        };
        
        write!(
            writer,
            "[{}] {} [{}:{}] {}: {}",
            entry.timestamp,
            level_str,
            entry.file,
            entry.line,
            entry.task_name,
            entry.message
        ).map_err(|_| FreeRtosError::BufferOverflow)?;
        
        let len = writer.bytes_written();
        let formatted = core::str::from_utf8(&self.output_buffer[..len])
            .map_err(|_| FreeRtosError::InvalidData)?;
        
        Ok(formatted)
    }
    
    // 导出日志到字符串
    pub fn export_logs(&mut self) -> Result<String, FreeRtosError> {
        let mut result = String::new();
        
        for entry in &self.entries {
            let formatted = self.format_entry(entry)?;
            result.push_str(formatted);
            result.push('\n');
        }
        
        Ok(result)
    }
}

// 日志宏
#[macro_export]
macro_rules! log_trace {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(
            LogLevel::Trace,
            &format!($($arg)*),
            file!(),
            line!()
        );
    };
}

#[macro_export]
macro_rules! log_debug {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(
            LogLevel::Debug,
            &format!($($arg)*),
            file!(),
            line!()
        );
    };
}

#[macro_export]
macro_rules! log_info {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(
            LogLevel::Info,
            &format!($($arg)*),
            file!(),
            line!()
        );
    };
}

#[macro_export]
macro_rules! log_warn {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(
            LogLevel::Warn,
            &format!($($arg)*),
            file!(),
            line!()
        );
    };
}

#[macro_export]
macro_rules! log_error {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(
            LogLevel::Error,
            &format!($($arg)*),
            file!(),
            line!()
        );
    };
}

#[macro_export]
macro_rules! log_fatal {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(
            LogLevel::Fatal,
            &format!($($arg)*),
            file!(),
            line!()
        );
    };
}

// 断言宏
#[macro_export]
macro_rules! freertos_assert {
    ($condition:expr, $logger:expr, $($arg:tt)*) => {
        if !$condition {
            log_fatal!($logger, "Assertion failed: {}", stringify!($condition));
            log_fatal!($logger, $($arg)*);
            panic!("Assertion failed: {}", stringify!($condition));
        }
    };
}

// 调试工具
pub struct DebugTools {
    logger: Logger,
    monitor: SystemMonitor,
    breakpoints: Vec<Breakpoint>,
    trace_enabled: bool,
}

#[derive(Debug, Clone)]
pub struct Breakpoint {
    pub id: u32,
    pub file: &'static str,
    pub line: u32,
    pub condition: Option<String>,
    pub hit_count: u32,
    pub enabled: bool,
}

impl DebugTools {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(1000),
            monitor: SystemMonitor::new(),
            breakpoints: Vec::new(),
            trace_enabled: false,
        }
    }
    
    // 获取日志系统引用
    pub fn logger(&mut self) -> &mut Logger {
        &mut self.logger
    }
    
    // 获取监控系统引用
    pub fn monitor(&mut self) -> &mut SystemMonitor {
        &mut self.monitor
    }
    
    // 添加断点
    pub fn add_breakpoint(
        &mut self,
        file: &'static str,
        line: u32,
        condition: Option<String>,
    ) -> u32 {
        let id = self.breakpoints.len() as u32 + 1;
        
        let breakpoint = Breakpoint {
            id,
            file,
            line,
            condition,
            hit_count: 0,
            enabled: true,
        };
        
        self.breakpoints.push(breakpoint);
        id
    }
    
    // 移除断点
    pub fn remove_breakpoint(&mut self, id: u32) -> bool {
        if let Some(pos) = self.breakpoints.iter().position(|b| b.id == id) {
            self.breakpoints.remove(pos);
            true
        } else {
            false
        }
    }
    
    // 启用/禁用断点
    pub fn set_breakpoint_enabled(&mut self, id: u32, enabled: bool) -> bool {
        if let Some(breakpoint) = self.breakpoints.iter_mut().find(|b| b.id == id) {
            breakpoint.enabled = enabled;
            true
        } else {
            false
        }
    }
    
    // 检查断点
    pub fn check_breakpoint(&mut self, file: &'static str, line: u32) -> bool {
        for breakpoint in &mut self.breakpoints {
            if breakpoint.enabled && breakpoint.file == file && breakpoint.line == line {
                breakpoint.hit_count += 1;
                
                log_debug!(
                    self.logger,
                    "Breakpoint hit: {}:{} (count: {})",
                    file,
                    line,
                    breakpoint.hit_count
                );
                
                return true;
            }
        }
        
        false
    }
    
    // 启用/禁用跟踪
    pub fn set_trace_enabled(&mut self, enabled: bool) {
        self.trace_enabled = enabled;
    }
    
    // 跟踪函数调用
    pub fn trace_function_call(&mut self, function_name: &str, file: &'static str, line: u32) {
        if self.trace_enabled {
            log_trace!(
                self.logger,
                "Function call: {} at {}:{}",
                function_name,
                file,
                line
            );
        }
    }
    
    // 跟踪函数返回
    pub fn trace_function_return(&mut self, function_name: &str, file: &'static str, line: u32) {
        if self.trace_enabled {
            log_trace!(
                self.logger,
                "Function return: {} at {}:{}",
                function_name,
                file,
                line
            );
        }
    }
}

// 调试宏
#[macro_export]
macro_rules! debug_breakpoint {
    ($debug_tools:expr) => {
        if $debug_tools.check_breakpoint(file!(), line!()) {
            // 在这里可以添加调试器断点或其他调试逻辑
        }
    };
}

#[macro_export]
macro_rules! trace_function {
    ($debug_tools:expr, $func_name:expr) => {
        $debug_tools.trace_function_call($func_name, file!(), line!());
    };
}

// 辅助函数
fn get_current_task_name() -> Option<String> {
    unsafe {
        let task_handle = freertos_sys::xTaskGetCurrentTaskHandle();
        if task_handle.is_null() {
            return None;
        }
        
        let task_name_ptr = freertos_sys::pcTaskGetName(task_handle);
        if task_name_ptr.is_null() {
            return None;
        }
        
        let name_len = strlen(task_name_ptr);
        let name_slice = core::slice::from_raw_parts(task_name_ptr as *const u8, name_len);
        
        if let Ok(name_str) = core::str::from_utf8(name_slice) {
            Some(name_str.to_string())
        } else {
            None
        }
    }
}

fn get_current_timestamp() -> u64 {
    unsafe {
        freertos_sys::xTaskGetTickCount() as u64
    }
}
```