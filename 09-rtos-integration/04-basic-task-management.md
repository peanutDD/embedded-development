# 基础任务管理

## 目录

1. [任务概念](#任务概念)
2. [任务状态](#任务状态)
3. [任务控制块](#任务控制块)
4. [任务调度](#任务调度)
5. [任务创建与删除](#任务创建与删除)
6. [任务优先级管理](#任务优先级管理)
7. [任务同步](#任务同步)
8. [任务监控](#任务监控)
9. [最佳实践](#最佳实践)

## 任务概念

### 什么是任务

在RTOS中，任务（Task）是系统调度的基本单位，每个任务都有自己的执行上下文和栈空间。

```rust
use heapless::Vec;
use core::ptr::NonNull;

/// 任务状态枚举
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TaskState {
    Ready,      // 就绪状态
    Running,    // 运行状态
    Blocked,    // 阻塞状态
    Suspended,  // 挂起状态
    Terminated, // 终止状态
}

/// 任务优先级类型
pub type Priority = u8;

/// 任务ID类型
pub type TaskId = u32;

/// 任务函数类型
pub type TaskFunction = fn();

/// 任务属性
#[derive(Debug, Clone)]
pub struct TaskAttributes {
    pub name: &'static str,
    pub priority: Priority,
    pub stack_size: usize,
    pub auto_start: bool,
}

impl Default for TaskAttributes {
    fn default() -> Self {
        Self {
            name: "unnamed_task",
            priority: 5,
            stack_size: 1024,
            auto_start: true,
        }
    }
}
```

### 任务特征

```rust
/// 任务特征定义
pub trait Task {
    /// 任务执行函数
    fn run(&mut self);
    
    /// 任务初始化
    fn init(&mut self) -> Result<(), TaskError> {
        Ok(())
    }
    
    /// 任务清理
    fn cleanup(&mut self) {
        // 默认实现为空
    }
    
    /// 获取任务名称
    fn name(&self) -> &'static str;
    
    /// 获取任务优先级
    fn priority(&self) -> Priority;
}

/// 任务错误类型
#[derive(Debug, Clone, Copy)]
pub enum TaskError {
    InvalidPriority,
    StackOverflow,
    OutOfMemory,
    TaskNotFound,
    InvalidState,
    Timeout,
}
```

## 任务状态

### 状态转换图

```rust
/// 任务状态管理器
pub struct TaskStateManager {
    current_state: TaskState,
    previous_state: TaskState,
    state_change_count: u32,
}

impl TaskStateManager {
    pub fn new() -> Self {
        Self {
            current_state: TaskState::Ready,
            previous_state: TaskState::Ready,
            state_change_count: 0,
        }
    }
    
    /// 状态转换
    pub fn transition_to(&mut self, new_state: TaskState) -> Result<(), TaskError> {
        if self.is_valid_transition(self.current_state, new_state) {
            self.previous_state = self.current_state;
            self.current_state = new_state;
            self.state_change_count += 1;
            Ok(())
        } else {
            Err(TaskError::InvalidState)
        }
    }
    
    /// 检查状态转换是否有效
    fn is_valid_transition(&self, from: TaskState, to: TaskState) -> bool {
        use TaskState::*;
        match (from, to) {
            (Ready, Running) => true,
            (Running, Ready) => true,
            (Running, Blocked) => true,
            (Running, Suspended) => true,
            (Running, Terminated) => true,
            (Blocked, Ready) => true,
            (Suspended, Ready) => true,
            _ => false,
        }
    }
    
    /// 获取当前状态
    pub fn current_state(&self) -> TaskState {
        self.current_state
    }
    
    /// 获取状态变化次数
    pub fn state_changes(&self) -> u32 {
        self.state_change_count
    }
}
```

## 任务控制块

### TCB结构定义

```rust
use core::mem::MaybeUninit;

/// 任务控制块 (Task Control Block)
#[repr(C)]
pub struct TaskControlBlock {
    pub id: TaskId,
    pub name: &'static str,
    pub priority: Priority,
    pub state: TaskStateManager,
    pub stack_pointer: *mut u8,
    pub stack_base: *mut u8,
    pub stack_size: usize,
    pub stack_usage: usize,
    pub cpu_usage: CpuUsage,
    pub creation_time: u64,
    pub last_run_time: u64,
    pub total_run_time: u64,
    pub context: TaskContext,
}

/// CPU使用率统计
#[derive(Debug, Default)]
pub struct CpuUsage {
    pub total_cycles: u64,
    pub idle_cycles: u64,
    pub percentage: f32,
}

/// 任务上下文
#[repr(C)]
#[derive(Debug, Default)]
pub struct TaskContext {
    // ARM Cortex-M 寄存器
    pub r0: u32,
    pub r1: u32,
    pub r2: u32,
    pub r3: u32,
    pub r4: u32,
    pub r5: u32,
    pub r6: u32,
    pub r7: u32,
    pub r8: u32,
    pub r9: u32,
    pub r10: u32,
    pub r11: u32,
    pub r12: u32,
    pub sp: u32,   // 栈指针
    pub lr: u32,   // 链接寄存器
    pub pc: u32,   // 程序计数器
    pub psr: u32,  // 程序状态寄存器
}

impl TaskControlBlock {
    /// 创建新的TCB
    pub fn new(
        id: TaskId,
        name: &'static str,
        priority: Priority,
        stack_base: *mut u8,
        stack_size: usize,
    ) -> Self {
        Self {
            id,
            name,
            priority,
            state: TaskStateManager::new(),
            stack_pointer: unsafe { stack_base.add(stack_size) },
            stack_base,
            stack_size,
            stack_usage: 0,
            cpu_usage: CpuUsage::default(),
            creation_time: get_system_time(),
            last_run_time: 0,
            total_run_time: 0,
            context: TaskContext::default(),
        }
    }
    
    /// 更新栈使用情况
    pub fn update_stack_usage(&mut self) {
        let current_sp = self.stack_pointer as usize;
        let stack_base = self.stack_base as usize;
        self.stack_usage = stack_base + self.stack_size - current_sp;
    }
    
    /// 检查栈溢出
    pub fn check_stack_overflow(&self) -> bool {
        let current_sp = self.stack_pointer as usize;
        let stack_limit = self.stack_base as usize;
        current_sp <= stack_limit
    }
    
    /// 获取栈使用百分比
    pub fn stack_usage_percentage(&self) -> f32 {
        (self.stack_usage as f32 / self.stack_size as f32) * 100.0
    }
}

/// 获取系统时间（模拟函数）
fn get_system_time() -> u64 {
    // 在实际实现中，这里会调用系统时钟
    0
}
```

## 任务调度

### 调度器实现

```rust
use heapless::binary_heap::{BinaryHeap, Max};
use core::cmp::Ordering;

/// 任务调度器
pub struct TaskScheduler {
    ready_queue: BinaryHeap<TaskRef, Max, 32>,
    current_task: Option<TaskId>,
    next_task_id: TaskId,
    scheduling_enabled: bool,
}

/// 任务引用（用于优先级队列）
#[derive(Debug, Clone)]
pub struct TaskRef {
    pub id: TaskId,
    pub priority: Priority,
}

impl PartialEq for TaskRef {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Eq for TaskRef {}

impl PartialOrd for TaskRef {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for TaskRef {
    fn cmp(&self, other: &Self) -> Ordering {
        self.priority.cmp(&other.priority)
    }
}

impl TaskScheduler {
    pub fn new() -> Self {
        Self {
            ready_queue: BinaryHeap::new(),
            current_task: None,
            next_task_id: 1,
            scheduling_enabled: true,
        }
    }
    
    /// 添加任务到就绪队列
    pub fn add_ready_task(&mut self, task_id: TaskId, priority: Priority) -> Result<(), TaskError> {
        let task_ref = TaskRef { id: task_id, priority };
        self.ready_queue.push(task_ref).map_err(|_| TaskError::OutOfMemory)?;
        Ok(())
    }
    
    /// 获取下一个要运行的任务
    pub fn get_next_task(&mut self) -> Option<TaskId> {
        if !self.scheduling_enabled {
            return self.current_task;
        }
        
        self.ready_queue.pop().map(|task_ref| task_ref.id)
    }
    
    /// 设置当前任务
    pub fn set_current_task(&mut self, task_id: Option<TaskId>) {
        self.current_task = task_id;
    }
    
    /// 启用/禁用调度
    pub fn set_scheduling_enabled(&mut self, enabled: bool) {
        self.scheduling_enabled = enabled;
    }
    
    /// 强制调度
    pub fn yield_current_task(&mut self) {
        if let Some(current_id) = self.current_task {
            // 将当前任务重新加入就绪队列
            // 这里需要获取任务的优先级
            // 在实际实现中，需要从任务管理器获取
        }
    }
    
    /// 获取就绪队列长度
    pub fn ready_queue_length(&self) -> usize {
        self.ready_queue.len()
    }
}
```

### 调度算法

```rust
/// 调度算法类型
#[derive(Debug, Clone, Copy)]
pub enum SchedulingAlgorithm {
    PriorityBased,      // 基于优先级
    RoundRobin,         // 轮转调度
    EarliestDeadline,   // 最早截止时间
    RateMonotonic,      // 速率单调
}

/// 调度策略
pub struct SchedulingPolicy {
    algorithm: SchedulingAlgorithm,
    time_slice: u32,  // 时间片（毫秒）
    preemptive: bool, // 是否抢占式
}

impl Default for SchedulingPolicy {
    fn default() -> Self {
        Self {
            algorithm: SchedulingAlgorithm::PriorityBased,
            time_slice: 10,
            preemptive: true,
        }
    }
}

/// 高级调度器
pub struct AdvancedScheduler {
    policy: SchedulingPolicy,
    scheduler: TaskScheduler,
    time_slice_counter: u32,
}

impl AdvancedScheduler {
    pub fn new(policy: SchedulingPolicy) -> Self {
        Self {
            policy,
            scheduler: TaskScheduler::new(),
            time_slice_counter: 0,
        }
    }
    
    /// 调度决策
    pub fn schedule(&mut self) -> Option<TaskId> {
        match self.policy.algorithm {
            SchedulingAlgorithm::PriorityBased => {
                self.priority_based_schedule()
            }
            SchedulingAlgorithm::RoundRobin => {
                self.round_robin_schedule()
            }
            _ => {
                // 其他算法的实现
                self.scheduler.get_next_task()
            }
        }
    }
    
    /// 基于优先级的调度
    fn priority_based_schedule(&mut self) -> Option<TaskId> {
        self.scheduler.get_next_task()
    }
    
    /// 轮转调度
    fn round_robin_schedule(&mut self) -> Option<TaskId> {
        self.time_slice_counter += 1;
        
        if self.time_slice_counter >= self.policy.time_slice {
            self.time_slice_counter = 0;
            self.scheduler.yield_current_task();
        }
        
        self.scheduler.get_next_task()
    }
    
    /// 系统滴答处理
    pub fn tick(&mut self) {
        if self.policy.algorithm == SchedulingAlgorithm::RoundRobin {
            self.time_slice_counter += 1;
        }
    }
}
```

## 任务创建与删除

### 任务管理器

```rust
use heapless::FnvIndexMap;

/// 任务管理器
pub struct TaskManager {
    tasks: FnvIndexMap<TaskId, TaskControlBlock, 32>,
    scheduler: AdvancedScheduler,
    next_id: TaskId,
    system_stack: [u8; 4096],
}

impl TaskManager {
    pub fn new() -> Self {
        Self {
            tasks: FnvIndexMap::new(),
            scheduler: AdvancedScheduler::new(SchedulingPolicy::default()),
            next_id: 1,
            system_stack: [0; 4096],
        }
    }
    
    /// 创建任务
    pub fn create_task(
        &mut self,
        attributes: TaskAttributes,
        task_fn: TaskFunction,
    ) -> Result<TaskId, TaskError> {
        let task_id = self.next_id;
        self.next_id += 1;
        
        // 分配栈空间
        let stack = self.allocate_stack(attributes.stack_size)?;
        
        // 创建TCB
        let mut tcb = TaskControlBlock::new(
            task_id,
            attributes.name,
            attributes.priority,
            stack.as_mut_ptr(),
            attributes.stack_size,
        );
        
        // 初始化任务上下文
        self.initialize_context(&mut tcb, task_fn);
        
        // 添加到任务列表
        self.tasks.insert(task_id, tcb).map_err(|_| TaskError::OutOfMemory)?;
        
        // 如果自动启动，添加到就绪队列
        if attributes.auto_start {
            self.scheduler.scheduler.add_ready_task(task_id, attributes.priority)?;
        }
        
        Ok(task_id)
    }
    
    /// 删除任务
    pub fn delete_task(&mut self, task_id: TaskId) -> Result<(), TaskError> {
        if let Some(mut tcb) = self.tasks.remove(&task_id) {
            // 清理任务资源
            tcb.state.transition_to(TaskState::Terminated)?;
            
            // 释放栈空间
            self.deallocate_stack(tcb.stack_base, tcb.stack_size);
            
            Ok(())
        } else {
            Err(TaskError::TaskNotFound)
        }
    }
    
    /// 启动任务
    pub fn start_task(&mut self, task_id: TaskId) -> Result<(), TaskError> {
        if let Some(tcb) = self.tasks.get_mut(&task_id) {
            tcb.state.transition_to(TaskState::Ready)?;
            self.scheduler.scheduler.add_ready_task(task_id, tcb.priority)?;
            Ok(())
        } else {
            Err(TaskError::TaskNotFound)
        }
    }
    
    /// 暂停任务
    pub fn suspend_task(&mut self, task_id: TaskId) -> Result<(), TaskError> {
        if let Some(tcb) = self.tasks.get_mut(&task_id) {
            tcb.state.transition_to(TaskState::Suspended)?;
            Ok(())
        } else {
            Err(TaskError::TaskNotFound)
        }
    }
    
    /// 恢复任务
    pub fn resume_task(&mut self, task_id: TaskId) -> Result<(), TaskError> {
        if let Some(tcb) = self.tasks.get_mut(&task_id) {
            tcb.state.transition_to(TaskState::Ready)?;
            self.scheduler.scheduler.add_ready_task(task_id, tcb.priority)?;
            Ok(())
        } else {
            Err(TaskError::TaskNotFound)
        }
    }
    
    /// 分配栈空间（简化实现）
    fn allocate_stack(&mut self, size: usize) -> Result<&mut [u8], TaskError> {
        if size <= self.system_stack.len() {
            Ok(&mut self.system_stack[..size])
        } else {
            Err(TaskError::OutOfMemory)
        }
    }
    
    /// 释放栈空间
    fn deallocate_stack(&mut self, _stack_base: *mut u8, _size: usize) {
        // 在实际实现中，这里会释放栈内存
    }
    
    /// 初始化任务上下文
    fn initialize_context(&self, tcb: &mut TaskControlBlock, task_fn: TaskFunction) {
        // 设置程序计数器指向任务函数
        tcb.context.pc = task_fn as *const fn() as u32;
        
        // 设置栈指针
        tcb.context.sp = tcb.stack_pointer as u32;
        
        // 设置处理器状态寄存器（Thumb模式）
        tcb.context.psr = 0x01000000;
    }
}
```

## 任务优先级管理

### 优先级系统

```rust
/// 优先级管理器
pub struct PriorityManager {
    priority_levels: u8,
    priority_bitmap: u32,
    ready_lists: [Vec<TaskId, 16>; 32],
}

impl PriorityManager {
    pub fn new(levels: u8) -> Self {
        Self {
            priority_levels: levels,
            priority_bitmap: 0,
            ready_lists: [const { Vec::new() }; 32],
        }
    }
    
    /// 添加任务到优先级队列
    pub fn add_task(&mut self, task_id: TaskId, priority: Priority) -> Result<(), TaskError> {
        if priority as usize >= self.ready_lists.len() {
            return Err(TaskError::InvalidPriority);
        }
        
        self.ready_lists[priority as usize]
            .push(task_id)
            .map_err(|_| TaskError::OutOfMemory)?;
        
        // 设置优先级位图
        self.priority_bitmap |= 1 << priority;
        
        Ok(())
    }
    
    /// 获取最高优先级任务
    pub fn get_highest_priority_task(&mut self) -> Option<TaskId> {
        if self.priority_bitmap == 0 {
            return None;
        }
        
        // 找到最高优先级（最高位）
        let highest_priority = 31 - self.priority_bitmap.leading_zeros() as u8;
        
        if let Some(task_id) = self.ready_lists[highest_priority as usize].pop() {
            // 如果该优先级队列为空，清除位图
            if self.ready_lists[highest_priority as usize].is_empty() {
                self.priority_bitmap &= !(1 << highest_priority);
            }
            Some(task_id)
        } else {
            None
        }
    }
    
    /// 移除任务
    pub fn remove_task(&mut self, task_id: TaskId, priority: Priority) -> bool {
        if let Some(pos) = self.ready_lists[priority as usize]
            .iter()
            .position(|&id| id == task_id)
        {
            self.ready_lists[priority as usize].swap_remove(pos);
            
            // 如果队列为空，清除位图
            if self.ready_lists[priority as usize].is_empty() {
                self.priority_bitmap &= !(1 << priority);
            }
            
            true
        } else {
            false
        }
    }
    
    /// 获取优先级统计
    pub fn get_priority_stats(&self) -> PriorityStats {
        let mut stats = PriorityStats::default();
        
        for (priority, list) in self.ready_lists.iter().enumerate() {
            if !list.is_empty() {
                stats.active_priorities += 1;
                stats.total_ready_tasks += list.len();
                
                if priority > stats.highest_priority as usize {
                    stats.highest_priority = priority as u8;
                }
                
                if stats.lowest_priority == 0 || priority < stats.lowest_priority as usize {
                    stats.lowest_priority = priority as u8;
                }
            }
        }
        
        stats
    }
}

/// 优先级统计信息
#[derive(Debug, Default)]
pub struct PriorityStats {
    pub active_priorities: usize,
    pub total_ready_tasks: usize,
    pub highest_priority: u8,
    pub lowest_priority: u8,
}
```

## 任务同步

### 同步原语

```rust
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// 任务同步管理器
pub struct TaskSynchronization {
    mutexes: FnvIndexMap<u32, TaskMutex, 16>,
    semaphores: FnvIndexMap<u32, TaskSemaphore, 16>,
    events: FnvIndexMap<u32, TaskEvent, 16>,
    next_id: AtomicU32,
}

/// 任务互斥锁
pub struct TaskMutex {
    locked: AtomicBool,
    owner: Option<TaskId>,
    waiting_tasks: Vec<TaskId, 8>,
    priority_ceiling: Priority,
}

/// 任务信号量
pub struct TaskSemaphore {
    count: AtomicU32,
    max_count: u32,
    waiting_tasks: Vec<TaskId, 8>,
}

/// 任务事件
pub struct TaskEvent {
    flags: AtomicU32,
    waiting_tasks: Vec<(TaskId, u32), 8>, // (task_id, wait_flags)
}

impl TaskSynchronization {
    pub fn new() -> Self {
        Self {
            mutexes: FnvIndexMap::new(),
            semaphores: FnvIndexMap::new(),
            events: FnvIndexMap::new(),
            next_id: AtomicU32::new(1),
        }
    }
    
    /// 创建互斥锁
    pub fn create_mutex(&mut self, priority_ceiling: Priority) -> Result<u32, TaskError> {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        
        let mutex = TaskMutex {
            locked: AtomicBool::new(false),
            owner: None,
            waiting_tasks: Vec::new(),
            priority_ceiling,
        };
        
        self.mutexes.insert(id, mutex).map_err(|_| TaskError::OutOfMemory)?;
        Ok(id)
    }
    
    /// 锁定互斥锁
    pub fn lock_mutex(&mut self, mutex_id: u32, task_id: TaskId) -> Result<bool, TaskError> {
        if let Some(mutex) = self.mutexes.get_mut(&mutex_id) {
            if mutex.locked.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed).is_ok() {
                mutex.owner = Some(task_id);
                Ok(true) // 成功获取锁
            } else {
                // 添加到等待队列
                mutex.waiting_tasks.push(task_id).map_err(|_| TaskError::OutOfMemory)?;
                Ok(false) // 需要等待
            }
        } else {
            Err(TaskError::TaskNotFound)
        }
    }
    
    /// 解锁互斥锁
    pub fn unlock_mutex(&mut self, mutex_id: u32, task_id: TaskId) -> Result<Option<TaskId>, TaskError> {
        if let Some(mutex) = self.mutexes.get_mut(&mutex_id) {
            if mutex.owner == Some(task_id) {
                mutex.owner = None;
                mutex.locked.store(false, Ordering::Release);
                
                // 唤醒等待的任务
                if let Some(waiting_task) = mutex.waiting_tasks.pop() {
                    Ok(Some(waiting_task))
                } else {
                    Ok(None)
                }
            } else {
                Err(TaskError::InvalidState)
            }
        } else {
            Err(TaskError::TaskNotFound)
        }
    }
    
    /// 创建信号量
    pub fn create_semaphore(&mut self, initial_count: u32, max_count: u32) -> Result<u32, TaskError> {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        
        let semaphore = TaskSemaphore {
            count: AtomicU32::new(initial_count),
            max_count,
            waiting_tasks: Vec::new(),
        };
        
        self.semaphores.insert(id, semaphore).map_err(|_| TaskError::OutOfMemory)?;
        Ok(id)
    }
    
    /// 等待信号量
    pub fn wait_semaphore(&mut self, sem_id: u32, task_id: TaskId) -> Result<bool, TaskError> {
        if let Some(semaphore) = self.semaphores.get_mut(&sem_id) {
            let current_count = semaphore.count.load(Ordering::Acquire);
            if current_count > 0 {
                if semaphore.count.compare_exchange(
                    current_count,
                    current_count - 1,
                    Ordering::Acquire,
                    Ordering::Relaxed
                ).is_ok() {
                    Ok(true) // 成功获取信号量
                } else {
                    // 重试或添加到等待队列
                    semaphore.waiting_tasks.push(task_id).map_err(|_| TaskError::OutOfMemory)?;
                    Ok(false)
                }
            } else {
                // 添加到等待队列
                semaphore.waiting_tasks.push(task_id).map_err(|_| TaskError::OutOfMemory)?;
                Ok(false)
            }
        } else {
            Err(TaskError::TaskNotFound)
        }
    }
    
    /// 释放信号量
    pub fn signal_semaphore(&mut self, sem_id: u32) -> Result<Option<TaskId>, TaskError> {
        if let Some(semaphore) = self.semaphores.get_mut(&sem_id) {
            let current_count = semaphore.count.load(Ordering::Acquire);
            if current_count < semaphore.max_count {
                semaphore.count.store(current_count + 1, Ordering::Release);
                
                // 唤醒等待的任务
                if let Some(waiting_task) = semaphore.waiting_tasks.pop() {
                    Ok(Some(waiting_task))
                } else {
                    Ok(None)
                }
            } else {
                Err(TaskError::InvalidState)
            }
        } else {
            Err(TaskError::TaskNotFound)
        }
    }
}
```

## 任务监控

### 监控系统

```rust
/// 任务监控器
pub struct TaskMonitor {
    monitoring_enabled: bool,
    sample_interval: u32,
    sample_count: u32,
    task_stats: FnvIndexMap<TaskId, TaskStatistics, 32>,
}

/// 任务统计信息
#[derive(Debug, Default)]
pub struct TaskStatistics {
    pub execution_count: u32,
    pub total_execution_time: u64,
    pub average_execution_time: u64,
    pub max_execution_time: u64,
    pub min_execution_time: u64,
    pub cpu_utilization: f32,
    pub stack_high_water_mark: usize,
    pub context_switches: u32,
    pub last_execution_time: u64,
}

impl TaskMonitor {
    pub fn new() -> Self {
        Self {
            monitoring_enabled: true,
            sample_interval: 1000, // 1秒
            sample_count: 0,
            task_stats: FnvIndexMap::new(),
        }
    }
    
    /// 开始监控任务
    pub fn start_monitoring(&mut self, task_id: TaskId) {
        if !self.task_stats.contains_key(&task_id) {
            let _ = self.task_stats.insert(task_id, TaskStatistics::default());
        }
    }
    
    /// 记录任务执行开始
    pub fn task_execution_start(&mut self, task_id: TaskId, timestamp: u64) {
        if let Some(stats) = self.task_stats.get_mut(&task_id) {
            stats.last_execution_time = timestamp;
        }
    }
    
    /// 记录任务执行结束
    pub fn task_execution_end(&mut self, task_id: TaskId, timestamp: u64) {
        if let Some(stats) = self.task_stats.get_mut(&task_id) {
            let execution_time = timestamp - stats.last_execution_time;
            
            stats.execution_count += 1;
            stats.total_execution_time += execution_time;
            stats.average_execution_time = stats.total_execution_time / stats.execution_count as u64;
            
            if execution_time > stats.max_execution_time {
                stats.max_execution_time = execution_time;
            }
            
            if stats.min_execution_time == 0 || execution_time < stats.min_execution_time {
                stats.min_execution_time = execution_time;
            }
        }
    }
    
    /// 记录上下文切换
    pub fn record_context_switch(&mut self, task_id: TaskId) {
        if let Some(stats) = self.task_stats.get_mut(&task_id) {
            stats.context_switches += 1;
        }
    }
    
    /// 更新栈使用情况
    pub fn update_stack_usage(&mut self, task_id: TaskId, stack_usage: usize) {
        if let Some(stats) = self.task_stats.get_mut(&task_id) {
            if stack_usage > stats.stack_high_water_mark {
                stats.stack_high_water_mark = stack_usage;
            }
        }
    }
    
    /// 计算CPU利用率
    pub fn calculate_cpu_utilization(&mut self, total_time: u64) {
        for (_, stats) in self.task_stats.iter_mut() {
            if total_time > 0 {
                stats.cpu_utilization = (stats.total_execution_time as f32 / total_time as f32) * 100.0;
            }
        }
    }
    
    /// 获取任务统计信息
    pub fn get_task_statistics(&self, task_id: TaskId) -> Option<&TaskStatistics> {
        self.task_stats.get(&task_id)
    }
    
    /// 获取系统统计报告
    pub fn generate_system_report(&self) -> SystemReport {
        let mut report = SystemReport::default();
        
        for (task_id, stats) in self.task_stats.iter() {
            report.total_tasks += 1;
            report.total_execution_time += stats.total_execution_time;
            report.total_context_switches += stats.context_switches;
            
            if stats.cpu_utilization > report.highest_cpu_usage {
                report.highest_cpu_usage = stats.cpu_utilization;
                report.most_active_task = Some(*task_id);
            }
            
            if stats.stack_high_water_mark > report.max_stack_usage {
                report.max_stack_usage = stats.stack_high_water_mark;
            }
        }
        
        report
    }
}

/// 系统报告
#[derive(Debug, Default)]
pub struct SystemReport {
    pub total_tasks: u32,
    pub total_execution_time: u64,
    pub total_context_switches: u32,
    pub highest_cpu_usage: f32,
    pub most_active_task: Option<TaskId>,
    pub max_stack_usage: usize,
}
```

## 最佳实践

### 任务设计原则

```rust
/// 任务设计最佳实践示例

/// 1. 高优先级任务 - 中断服务任务
pub struct InterruptServiceTask {
    name: &'static str,
    priority: Priority,
}

impl Task for InterruptServiceTask {
    fn run(&mut self) {
        // 快速处理中断
        // 最小化中断禁用时间
        // 将复杂处理推迟到低优先级任务
    }
    
    fn name(&self) -> &'static str {
        self.name
    }
    
    fn priority(&self) -> Priority {
        self.priority
    }
}

/// 2. 中等优先级任务 - 控制任务
pub struct ControlTask {
    name: &'static str,
    priority: Priority,
    control_period: u32,
}

impl Task for ControlTask {
    fn run(&mut self) {
        // 周期性控制逻辑
        // 确定性执行时间
        // 避免阻塞操作
    }
    
    fn name(&self) -> &'static str {
        self.name
    }
    
    fn priority(&self) -> Priority {
        self.priority
    }
}

/// 3. 低优先级任务 - 后台处理
pub struct BackgroundTask {
    name: &'static str,
    priority: Priority,
}

impl Task for BackgroundTask {
    fn run(&mut self) {
        // 非关键处理
        // 可以被高优先级任务抢占
        // 处理延迟的工作
    }
    
    fn name(&self) -> &'static str {
        self.name
    }
    
    fn priority(&self) -> Priority {
        self.priority
    }
}
```

### 性能优化技巧

```rust
/// 任务性能优化工具
pub struct TaskOptimizer {
    profiler: TaskProfiler,
    analyzer: PerformanceAnalyzer,
}

/// 任务性能分析器
pub struct TaskProfiler {
    enabled: bool,
    samples: Vec<ProfileSample, 100>,
}

#[derive(Debug, Clone)]
pub struct ProfileSample {
    task_id: TaskId,
    start_time: u64,
    end_time: u64,
    stack_usage: usize,
}

/// 性能分析器
pub struct PerformanceAnalyzer {
    bottlenecks: Vec<PerformanceBottleneck, 10>,
    recommendations: Vec<OptimizationRecommendation, 20>,
}

#[derive(Debug)]
pub struct PerformanceBottleneck {
    task_id: TaskId,
    issue_type: BottleneckType,
    severity: Severity,
    description: &'static str,
}

#[derive(Debug)]
pub enum BottleneckType {
    HighCpuUsage,
    ExcessiveStackUsage,
    FrequentContextSwitches,
    LongExecutionTime,
    PriorityInversion,
}

#[derive(Debug)]
pub enum Severity {
    Low,
    Medium,
    High,
    Critical,
}

#[derive(Debug)]
pub struct OptimizationRecommendation {
    target_task: TaskId,
    recommendation_type: RecommendationType,
    description: &'static str,
    expected_improvement: f32,
}

#[derive(Debug)]
pub enum RecommendationType {
    ReduceStackSize,
    IncreaseStackSize,
    AdjustPriority,
    OptimizeAlgorithm,
    ReduceExecutionTime,
    ImproveDataLocality,
}

impl TaskOptimizer {
    pub fn new() -> Self {
        Self {
            profiler: TaskProfiler {
                enabled: true,
                samples: Vec::new(),
            },
            analyzer: PerformanceAnalyzer {
                bottlenecks: Vec::new(),
                recommendations: Vec::new(),
            },
        }
    }
    
    /// 分析任务性能
    pub fn analyze_performance(&mut self, task_manager: &TaskManager) {
        self.analyzer.bottlenecks.clear();
        self.analyzer.recommendations.clear();
        
        for (task_id, tcb) in task_manager.tasks.iter() {
            // 检查CPU使用率
            if tcb.cpu_usage.percentage > 80.0 {
                let _ = self.analyzer.bottlenecks.push(PerformanceBottleneck {
                    task_id: *task_id,
                    issue_type: BottleneckType::HighCpuUsage,
                    severity: Severity::High,
                    description: "Task consuming excessive CPU time",
                });
            }
            
            // 检查栈使用率
            if tcb.stack_usage_percentage() > 90.0 {
                let _ = self.analyzer.bottlenecks.push(PerformanceBottleneck {
                    task_id: *task_id,
                    issue_type: BottleneckType::ExcessiveStackUsage,
                    severity: Severity::Critical,
                    description: "Task approaching stack overflow",
                });
                
                let _ = self.analyzer.recommendations.push(OptimizationRecommendation {
                    target_task: *task_id,
                    recommendation_type: RecommendationType::IncreaseStackSize,
                    description: "Increase stack size to prevent overflow",
                    expected_improvement: 25.0,
                });
            }
        }
    }
    
    /// 生成优化报告
    pub fn generate_optimization_report(&self) -> OptimizationReport {
        OptimizationReport {
            bottlenecks: self.analyzer.bottlenecks.clone(),
            recommendations: self.analyzer.recommendations.clone(),
            overall_score: self.calculate_performance_score(),
        }
    }
    
    /// 计算性能评分
    fn calculate_performance_score(&self) -> f32 {
        let critical_issues = self.analyzer.bottlenecks.iter()
            .filter(|b| matches!(b.severity, Severity::Critical))
            .count();
        
        let high_issues = self.analyzer.bottlenecks.iter()
            .filter(|b| matches!(b.severity, Severity::High))
            .count();
        
        let base_score = 100.0;
        let penalty = (critical_issues * 20 + high_issues * 10) as f32;
        
        (base_score - penalty).max(0.0)
    }
}

/// 优化报告
#[derive(Debug)]
pub struct OptimizationReport {
    pub bottlenecks: Vec<PerformanceBottleneck, 10>,
    pub recommendations: Vec<OptimizationRecommendation, 20>,
    pub overall_score: f32,
}
```

### 测试用例

```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_task_creation() {
        let mut task_manager = TaskManager::new();
        
        let attributes = TaskAttributes {
            name: "test_task",
            priority: 5,
            stack_size: 1024,
            auto_start: true,
        };
        
        let task_id = task_manager.create_task(attributes, test_task_function).unwrap();
        assert!(task_id > 0);
        
        // 验证任务已创建
        assert!(task_manager.tasks.contains_key(&task_id));
    }
    
    #[test]
    fn test_task_state_transitions() {
        let mut state_manager = TaskStateManager::new();
        
        // 测试有效状态转换
        assert!(state_manager.transition_to(TaskState::Running).is_ok());
        assert_eq!(state_manager.current_state(), TaskState::Running);
        
        assert!(state_manager.transition_to(TaskState::Blocked).is_ok());
        assert_eq!(state_manager.current_state(), TaskState::Blocked);
        
        assert!(state_manager.transition_to(TaskState::Ready).is_ok());
        assert_eq!(state_manager.current_state(), TaskState::Ready);
    }
    
    #[test]
    fn test_priority_scheduling() {
        let mut priority_manager = PriorityManager::new(32);
        
        // 添加不同优先级的任务
        priority_manager.add_task(1, 5).unwrap();
        priority_manager.add_task(2, 10).unwrap();
        priority_manager.add_task(3, 3).unwrap();
        
        // 应该返回最高优先级的任务
        assert_eq!(priority_manager.get_highest_priority_task(), Some(2));
        assert_eq!(priority_manager.get_highest_priority_task(), Some(1));
        assert_eq!(priority_manager.get_highest_priority_task(), Some(3));
    }
    
    #[test]
    fn test_mutex_operations() {
        let mut sync_manager = TaskSynchronization::new();
        
        let mutex_id = sync_manager.create_mutex(10).unwrap();
        
        // 第一个任务应该能够获取锁
        assert_eq!(sync_manager.lock_mutex(mutex_id, 1).unwrap(), true);
        
        // 第二个任务应该被阻塞
        assert_eq!(sync_manager.lock_mutex(mutex_id, 2).unwrap(), false);
        
        // 解锁应该唤醒等待的任务
        assert_eq!(sync_manager.unlock_mutex(mutex_id, 1).unwrap(), Some(2));
    }
    
    fn test_task_function() {
        // 测试任务函数
    }
}
```

## 总结

基础任务管理是RTOS的核心功能，包括：

1. **任务概念和特征** - 定义任务的基本属性和行为
2. **任务状态管理** - 实现任务状态转换和验证
3. **任务控制块** - 维护任务的完整上下文信息
4. **任务调度** - 实现多种调度算法和策略
5. **任务生命周期** - 管理任务的创建、运行和销毁
6. **优先级管理** - 实现基于优先级的任务调度
7. **任务同步** - 提供互斥锁、信号量等同步机制
8. **任务监控** - 实时监控任务性能和资源使用
9. **性能优化** - 分析和优化任务性能

通过这些机制，可以构建高效、可靠的实时系统，确保任务能够按照预期的时序和优先级执行。