# 任务调度与管理

任务调度是RTOS的核心功能，决定了系统的实时性能和资源利用效率。本文档详细介绍各种调度算法、优先级管理和性能优化技术。

## 调度算法基础

### 调度算法分类

#### 抢占式调度 (Preemptive Scheduling)
高优先级任务可以中断低优先级任务的执行：

```rust
use cortex_m::interrupt;
use heapless::Vec;

/// 任务状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TaskState {
    Ready,      // 就绪
    Running,    // 运行
    Blocked,    // 阻塞
    Suspended,  // 挂起
}

/// 任务控制块
#[derive(Debug)]
pub struct TaskControlBlock {
    pub id: u32,
    pub priority: u8,
    pub state: TaskState,
    pub stack_pointer: *mut u32,
    pub stack_base: *mut u32,
    pub stack_size: usize,
    pub entry_point: fn(),
    pub time_slice: u32,
    pub remaining_time: u32,
    pub cpu_usage: u32,
}

impl TaskControlBlock {
    pub fn new(
        id: u32,
        priority: u8,
        entry_point: fn(),
        stack_base: *mut u32,
        stack_size: usize,
    ) -> Self {
        Self {
            id,
            priority,
            state: TaskState::Ready,
            stack_pointer: stack_base,
            stack_base,
            stack_size,
            entry_point,
            time_slice: 10, // 默认10ms时间片
            remaining_time: 10,
            cpu_usage: 0,
        }
    }
    
    /// 重置时间片
    pub fn reset_time_slice(&mut self) {
        self.remaining_time = self.time_slice;
    }
    
    /// 减少剩余时间
    pub fn tick(&mut self) -> bool {
        if self.remaining_time > 0 {
            self.remaining_time -= 1;
            self.remaining_time == 0
        } else {
            false
        }
    }
}

/// 抢占式优先级调度器
pub struct PreemptiveScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    current_task: Option<usize>,
    ready_queue: [Vec<usize, 16>; 8], // 8个优先级队列
    tick_count: u32,
}

impl PreemptiveScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            current_task: None,
            ready_queue: [
                Vec::new(), Vec::new(), Vec::new(), Vec::new(),
                Vec::new(), Vec::new(), Vec::new(), Vec::new(),
            ],
            tick_count: 0,
        }
    }
    
    /// 添加任务
    pub fn add_task(&mut self, task: TaskControlBlock) -> Result<usize, &'static str> {
        let task_id = self.tasks.len();
        let priority = task.priority as usize;
        
        if priority >= self.ready_queue.len() {
            return Err("Invalid priority");
        }
        
        self.tasks.push(task).map_err(|_| "Too many tasks")?;
        
        // 将任务添加到对应优先级队列
        if self.tasks[task_id].state == TaskState::Ready {
            self.ready_queue[priority].push(task_id).map_err(|_| "Queue full")?;
        }
        
        Ok(task_id)
    }
    
    /// 获取最高优先级的就绪任务
    pub fn get_next_task(&mut self) -> Option<usize> {
        // 从最高优先级开始查找
        for (priority, queue) in self.ready_queue.iter_mut().enumerate().rev() {
            if let Some(task_id) = queue.pop() {
                // 检查任务是否仍然就绪
                if self.tasks[task_id].state == TaskState::Ready {
                    return Some(task_id);
                }
            }
        }
        None
    }
    
    /// 任务调度
    pub fn schedule(&mut self) -> Option<usize> {
        let next_task_id = self.get_next_task()?;
        
        // 如果有当前任务且不是要调度的任务，进行上下文切换
        if let Some(current_id) = self.current_task {
            if current_id != next_task_id {
                // 保存当前任务状态
                if self.tasks[current_id].state == TaskState::Running {
                    self.tasks[current_id].state = TaskState::Ready;
                    // 重新加入就绪队列
                    let priority = self.tasks[current_id].priority as usize;
                    self.ready_queue[priority].push(current_id).ok();
                }
            }
        }
        
        // 设置新任务为运行状态
        self.tasks[next_task_id].state = TaskState::Running;
        self.tasks[next_task_id].reset_time_slice();
        self.current_task = Some(next_task_id);
        
        Some(next_task_id)
    }
    
    /// 时钟中断处理
    pub fn tick(&mut self) -> bool {
        self.tick_count += 1;
        
        if let Some(current_id) = self.current_task {
            let need_reschedule = self.tasks[current_id].tick();
            
            if need_reschedule {
                // 时间片用完，需要重新调度
                return true;
            }
        }
        
        false
    }
    
    /// 阻塞当前任务
    pub fn block_current_task(&mut self) {
        if let Some(current_id) = self.current_task {
            self.tasks[current_id].state = TaskState::Blocked;
            self.current_task = None;
        }
    }
    
    /// 唤醒任务
    pub fn wake_task(&mut self, task_id: usize) -> Result<(), &'static str> {
        if task_id >= self.tasks.len() {
            return Err("Invalid task ID");
        }
        
        if self.tasks[task_id].state == TaskState::Blocked {
            self.tasks[task_id].state = TaskState::Ready;
            let priority = self.tasks[task_id].priority as usize;
            self.ready_queue[priority].push(task_id).map_err(|_| "Queue full")?;
        }
        
        Ok(())
    }
}
```

#### 协作式调度 (Cooperative Scheduling)
任务主动让出CPU控制权：

```rust
/// 协作式调度器
pub struct CooperativeScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    current_task: Option<usize>,
    ready_queue: Vec<usize, 16>,
}

impl CooperativeScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            current_task: None,
            ready_queue: Vec::new(),
        }
    }
    
    /// 当前任务主动让出CPU
    pub fn yield_task(&mut self) {
        if let Some(current_id) = self.current_task {
            // 将当前任务重新加入就绪队列
            if self.tasks[current_id].state == TaskState::Running {
                self.tasks[current_id].state = TaskState::Ready;
                self.ready_queue.push(current_id).ok();
            }
            self.current_task = None;
        }
    }
    
    /// 调度下一个任务
    pub fn schedule_next(&mut self) -> Option<usize> {
        if let Some(next_task_id) = self.ready_queue.pop() {
            self.tasks[next_task_id].state = TaskState::Running;
            self.current_task = Some(next_task_id);
            Some(next_task_id)
        } else {
            None
        }
    }
}
```

### 时间片轮转调度 (Round Robin)

```rust
/// 时间片轮转调度器
pub struct RoundRobinScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    current_task: Option<usize>,
    ready_queue: Vec<usize, 16>,
    time_quantum: u32,
    current_time_slice: u32,
}

impl RoundRobinScheduler {
    pub fn new(time_quantum: u32) -> Self {
        Self {
            tasks: Vec::new(),
            current_task: None,
            ready_queue: Vec::new(),
            time_quantum,
            current_time_slice: 0,
        }
    }
    
    /// 时钟中断处理
    pub fn tick(&mut self) -> bool {
        self.current_time_slice += 1;
        
        // 检查时间片是否用完
        if self.current_time_slice >= self.time_quantum {
            self.current_time_slice = 0;
            
            // 将当前任务移到队列末尾
            if let Some(current_id) = self.current_task {
                self.tasks[current_id].state = TaskState::Ready;
                self.ready_queue.push(current_id).ok();
                self.current_task = None;
            }
            
            return true; // 需要重新调度
        }
        
        false
    }
    
    /// 调度下一个任务
    pub fn schedule(&mut self) -> Option<usize> {
        // 从队列头部取出下一个任务
        if let Some(next_task_id) = self.ready_queue.remove(0).ok() {
            self.tasks[next_task_id].state = TaskState::Running;
            self.current_task = Some(next_task_id);
            self.current_time_slice = 0;
            Some(next_task_id)
        } else {
            None
        }
    }
}
```

### 最短作业优先 (Shortest Job First)

```rust
use heapless::binary_heap::{BinaryHeap, Min};

/// 任务执行时间估算
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct TaskEstimate {
    pub estimated_time: u32,
    pub task_id: usize,
}

/// 最短作业优先调度器
pub struct SJFScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    current_task: Option<usize>,
    ready_heap: BinaryHeap<TaskEstimate, Min, 16>,
    execution_history: Vec<(usize, u32), 32>, // 任务执行历史
}

impl SJFScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            current_task: None,
            ready_heap: BinaryHeap::new(),
            execution_history: Vec::new(),
        }
    }
    
    /// 估算任务执行时间
    pub fn estimate_execution_time(&self, task_id: usize) -> u32 {
        // 基于历史执行时间的指数加权平均
        let mut total_time = 0;
        let mut count = 0;
        let alpha = 0.5; // 平滑因子
        
        for &(id, time) in self.execution_history.iter().rev() {
            if id == task_id {
                if count == 0 {
                    return time; // 使用最近的执行时间
                }
                total_time = (total_time as f32 * (1.0 - alpha) + time as f32 * alpha) as u32;
                count += 1;
                if count >= 5 { // 只考虑最近5次执行
                    break;
                }
            }
        }
        
        if count > 0 {
            total_time
        } else {
            100 // 默认估算时间
        }
    }
    
    /// 添加就绪任务
    pub fn add_ready_task(&mut self, task_id: usize) -> Result<(), &'static str> {
        let estimated_time = self.estimate_execution_time(task_id);
        let estimate = TaskEstimate {
            estimated_time,
            task_id,
        };
        
        self.ready_heap.push(estimate).map_err(|_| "Heap full")
    }
    
    /// 调度下一个任务
    pub fn schedule(&mut self) -> Option<usize> {
        if let Some(estimate) = self.ready_heap.pop() {
            let task_id = estimate.task_id;
            self.tasks[task_id].state = TaskState::Running;
            self.current_task = Some(task_id);
            Some(task_id)
        } else {
            None
        }
    }
    
    /// 任务完成时记录执行时间
    pub fn task_completed(&mut self, task_id: usize, execution_time: u32) {
        // 记录执行历史
        self.execution_history.push((task_id, execution_time)).ok();
        
        // 保持历史记录在合理范围内
        if self.execution_history.len() > 30 {
            self.execution_history.remove(0);
        }
        
        self.current_task = None;
    }
}
```

## 优先级管理

### 静态优先级系统

```rust
/// 优先级定义
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    Idle = 0,
    Low = 1,
    Normal = 2,
    High = 3,
    Critical = 4,
    Interrupt = 5,
}

impl Priority {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Priority::Idle),
            1 => Some(Priority::Low),
            2 => Some(Priority::Normal),
            3 => Some(Priority::High),
            4 => Some(Priority::Critical),
            5 => Some(Priority::Interrupt),
            _ => None,
        }
    }
    
    pub fn as_u8(self) -> u8 {
        self as u8
    }
}

/// 静态优先级调度器
pub struct StaticPriorityScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    ready_queues: [Vec<usize, 16>; 6], // 6个优先级队列
    current_task: Option<usize>,
}

impl StaticPriorityScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            ready_queues: [
                Vec::new(), Vec::new(), Vec::new(),
                Vec::new(), Vec::new(), Vec::new(),
            ],
            current_task: None,
        }
    }
    
    /// 添加任务到就绪队列
    pub fn make_ready(&mut self, task_id: usize) -> Result<(), &'static str> {
        if task_id >= self.tasks.len() {
            return Err("Invalid task ID");
        }
        
        let priority = self.tasks[task_id].priority as usize;
        if priority >= self.ready_queues.len() {
            return Err("Invalid priority");
        }
        
        self.tasks[task_id].state = TaskState::Ready;
        self.ready_queues[priority].push(task_id).map_err(|_| "Queue full")
    }
    
    /// 获取最高优先级的就绪任务
    pub fn get_highest_priority_task(&mut self) -> Option<usize> {
        // 从最高优先级开始搜索
        for queue in self.ready_queues.iter_mut().rev() {
            if let Some(task_id) = queue.remove(0).ok() {
                return Some(task_id);
            }
        }
        None
    }
    
    /// 检查是否需要抢占
    pub fn should_preempt(&self, new_task_priority: u8) -> bool {
        if let Some(current_id) = self.current_task {
            new_task_priority > self.tasks[current_id].priority
        } else {
            true
        }
    }
    
    /// 抢占式调度
    pub fn preemptive_schedule(&mut self, new_task_id: usize) -> bool {
        let new_priority = self.tasks[new_task_id].priority;
        
        if self.should_preempt(new_priority) {
            // 抢占当前任务
            if let Some(current_id) = self.current_task {
                if self.tasks[current_id].state == TaskState::Running {
                    self.make_ready(current_id).ok();
                }
            }
            
            // 运行新任务
            self.tasks[new_task_id].state = TaskState::Running;
            self.current_task = Some(new_task_id);
            true
        } else {
            // 不需要抢占，将新任务加入就绪队列
            self.make_ready(new_task_id).ok();
            false
        }
    }
}
```

### 动态优先级系统

```rust
/// 动态优先级调度器
pub struct DynamicPriorityScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    base_priorities: Vec<u8, 16>,      // 基础优先级
    current_priorities: Vec<u8, 16>,   // 当前优先级
    aging_factor: u8,                  // 老化因子
    ready_queues: [Vec<usize, 16>; 8],
    current_task: Option<usize>,
    tick_count: u32,
}

impl DynamicPriorityScheduler {
    pub fn new(aging_factor: u8) -> Self {
        Self {
            tasks: Vec::new(),
            base_priorities: Vec::new(),
            current_priorities: Vec::new(),
            aging_factor,
            ready_queues: [
                Vec::new(), Vec::new(), Vec::new(), Vec::new(),
                Vec::new(), Vec::new(), Vec::new(), Vec::new(),
            ],
            current_task: None,
            tick_count: 0,
        }
    }
    
    /// 添加任务
    pub fn add_task(&mut self, mut task: TaskControlBlock) -> Result<usize, &'static str> {
        let task_id = self.tasks.len();
        let base_priority = task.priority;
        
        self.base_priorities.push(base_priority).map_err(|_| "Too many tasks")?;
        self.current_priorities.push(base_priority).map_err(|_| "Too many tasks")?;
        
        task.id = task_id as u32;
        self.tasks.push(task).map_err(|_| "Too many tasks")?;
        
        Ok(task_id)
    }
    
    /// 优先级老化处理
    pub fn age_priorities(&mut self) {
        for (i, &base_priority) in self.base_priorities.iter().enumerate() {
            if self.tasks[i].state == TaskState::Ready {
                // 增加等待任务的优先级
                let current = self.current_priorities[i];
                let max_priority = 7; // 最高优先级
                
                if current < max_priority {
                    self.current_priorities[i] = (current + self.aging_factor).min(max_priority);
                    self.tasks[i].priority = self.current_priorities[i];
                    
                    // 重新排队
                    self.requeue_task(i);
                }
            }
        }
    }
    
    /// 重置任务优先级
    pub fn reset_priority(&mut self, task_id: usize) {
        if task_id < self.base_priorities.len() {
            self.current_priorities[task_id] = self.base_priorities[task_id];
            self.tasks[task_id].priority = self.base_priorities[task_id];
        }
    }
    
    /// 重新排队任务
    fn requeue_task(&mut self, task_id: usize) {
        // 从所有队列中移除任务
        for queue in &mut self.ready_queues {
            if let Some(pos) = queue.iter().position(|&x| x == task_id) {
                queue.remove(pos);
                break;
            }
        }
        
        // 添加到新的优先级队列
        let priority = self.tasks[task_id].priority as usize;
        if priority < self.ready_queues.len() {
            self.ready_queues[priority].push(task_id).ok();
        }
    }
    
    /// 时钟中断处理
    pub fn tick(&mut self) {
        self.tick_count += 1;
        
        // 每100个时钟周期进行一次优先级老化
        if self.tick_count % 100 == 0 {
            self.age_priorities();
        }
        
        // 处理当前任务的时间片
        if let Some(current_id) = self.current_task {
            if self.tasks[current_id].tick() {
                // 时间片用完，重置优先级并重新调度
                self.reset_priority(current_id);
                self.make_ready(current_id).ok();
                self.current_task = None;
            }
        }
    }
    
    /// 将任务设为就绪状态
    pub fn make_ready(&mut self, task_id: usize) -> Result<(), &'static str> {
        if task_id >= self.tasks.len() {
            return Err("Invalid task ID");
        }
        
        self.tasks[task_id].state = TaskState::Ready;
        let priority = self.tasks[task_id].priority as usize;
        
        if priority < self.ready_queues.len() {
            self.ready_queues[priority].push(task_id).map_err(|_| "Queue full")
        } else {
            Err("Invalid priority")
        }
    }
    
    /// 调度下一个任务
    pub fn schedule(&mut self) -> Option<usize> {
        // 从最高优先级队列开始查找
        for queue in self.ready_queues.iter_mut().rev() {
            if let Some(task_id) = queue.remove(0).ok() {
                self.tasks[task_id].state = TaskState::Running;
                self.tasks[task_id].reset_time_slice();
                self.current_task = Some(task_id);
                return Some(task_id);
            }
        }
        None
    }
}
```

## 实时调度算法

### 速率单调调度 (Rate Monotonic Scheduling)

```rust
use heapless::Vec;

/// 周期性任务
#[derive(Debug, Clone)]
pub struct PeriodicTask {
    pub id: usize,
    pub period: u32,           // 周期 (ms)
    pub execution_time: u32,   // 执行时间 (ms)
    pub deadline: u32,         // 截止时间 (ms)
    pub next_release: u32,     // 下次释放时间
    pub priority: u8,          // 优先级 (周期越短优先级越高)
    pub state: TaskState,
}

impl PeriodicTask {
    pub fn new(id: usize, period: u32, execution_time: u32) -> Self {
        // 速率单调：周期越短，优先级越高
        let priority = (255 - (period / 10).min(255)) as u8;
        
        Self {
            id,
            period,
            execution_time,
            deadline: period, // 截止时间等于周期
            next_release: 0,
            priority,
            state: TaskState::Ready,
        }
    }
    
    /// 检查是否到达释放时间
    pub fn is_ready_to_release(&self, current_time: u32) -> bool {
        current_time >= self.next_release
    }
    
    /// 释放任务
    pub fn release(&mut self, current_time: u32) {
        self.next_release = current_time + self.period;
        self.state = TaskState::Ready;
    }
    
    /// 检查是否错过截止时间
    pub fn is_deadline_missed(&self, current_time: u32) -> bool {
        current_time > (self.next_release - self.period + self.deadline)
    }
}

/// 速率单调调度器
pub struct RateMonotonicScheduler {
    tasks: Vec<PeriodicTask, 16>,
    ready_queue: Vec<usize, 16>,
    current_task: Option<usize>,
    current_time: u32,
    hyperperiod: u32, // 超周期
}

impl RateMonotonicScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            ready_queue: Vec::new(),
            current_task: None,
            current_time: 0,
            hyperperiod: 1,
        }
    }
    
    /// 添加周期性任务
    pub fn add_task(&mut self, task: PeriodicTask) -> Result<(), &'static str> {
        // 计算新的超周期 (所有任务周期的最小公倍数)
        self.hyperperiod = lcm(self.hyperperiod, task.period);
        
        self.tasks.push(task).map_err(|_| "Too many tasks")
    }
    
    /// 可调度性分析
    pub fn schedulability_test(&self) -> bool {
        let n = self.tasks.len() as f32;
        let bound = n * (2.0_f32.powf(1.0 / n) - 1.0);
        
        let utilization: f32 = self.tasks.iter()
            .map(|task| task.execution_time as f32 / task.period as f32)
            .sum();
        
        utilization <= bound
    }
    
    /// 时钟中断处理
    pub fn tick(&mut self) {
        self.current_time += 1;
        
        // 检查任务释放
        for (i, task) in self.tasks.iter_mut().enumerate() {
            if task.is_ready_to_release(self.current_time) {
                // 检查是否错过上一个截止时间
                if task.is_deadline_missed(self.current_time) {
                    // 处理截止时间错过
                    self.handle_deadline_miss(i);
                }
                
                task.release(self.current_time);
                self.add_to_ready_queue(i);
            }
        }
        
        // 检查当前任务是否完成
        if let Some(current_id) = self.current_task {
            // 简化：假设每个tick执行1ms
            if self.tasks[current_id].execution_time <= 1 {
                // 任务完成
                self.tasks[current_id].state = TaskState::Suspended;
                self.current_task = None;
            } else {
                self.tasks[current_id].execution_time -= 1;
            }
        }
    }
    
    /// 添加任务到就绪队列（按优先级排序）
    fn add_to_ready_queue(&mut self, task_id: usize) {
        let task_priority = self.tasks[task_id].priority;
        
        // 找到插入位置（保持优先级顺序）
        let mut insert_pos = self.ready_queue.len();
        for (i, &existing_id) in self.ready_queue.iter().enumerate() {
            if self.tasks[existing_id].priority < task_priority {
                insert_pos = i;
                break;
            }
        }
        
        self.ready_queue.insert(insert_pos, task_id).ok();
    }
    
    /// 调度下一个任务
    pub fn schedule(&mut self) -> Option<usize> {
        if let Some(next_task_id) = self.ready_queue.remove(0).ok() {
            self.tasks[next_task_id].state = TaskState::Running;
            self.current_task = Some(next_task_id);
            Some(next_task_id)
        } else {
            None
        }
    }
    
    /// 处理截止时间错过
    fn handle_deadline_miss(&mut self, task_id: usize) {
        // 记录错过的截止时间
        // 可以触发错误处理或系统重配置
    }
}

/// 计算最大公约数
fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// 计算最小公倍数
fn lcm(a: u32, b: u32) -> u32 {
    a * b / gcd(a, b)
}
```

### 最早截止时间优先 (Earliest Deadline First)

```rust
use heapless::binary_heap::{BinaryHeap, Min};

/// 带截止时间的任务
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeadlineTask {
    pub task_id: usize,
    pub absolute_deadline: u32,
    pub remaining_time: u32,
}

impl PartialOrd for DeadlineTask {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for DeadlineTask {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        // 截止时间越早，优先级越高
        self.absolute_deadline.cmp(&other.absolute_deadline)
    }
}

/// 最早截止时间优先调度器
pub struct EDFScheduler {
    tasks: Vec<TaskControlBlock, 16>,
    deadline_heap: BinaryHeap<DeadlineTask, Min, 16>,
    current_task: Option<DeadlineTask>,
    current_time: u32,
    missed_deadlines: u32,
}

impl EDFScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            deadline_heap: BinaryHeap::new(),
            current_task: None,
            current_time: 0,
            missed_deadlines: 0,
        }
    }
    
    /// 添加任务
    pub fn add_task(&mut self, task: TaskControlBlock, deadline: u32) -> Result<(), &'static str> {
        let task_id = self.tasks.len();
        self.tasks.push(task).map_err(|_| "Too many tasks")?;
        
        let deadline_task = DeadlineTask {
            task_id,
            absolute_deadline: self.current_time + deadline,
            remaining_time: 100, // 假设执行时间
        };
        
        self.deadline_heap.push(deadline_task).map_err(|_| "Heap full")
    }
    
    /// 可调度性测试
    pub fn schedulability_test(&self, tasks: &[(u32, u32)]) -> bool {
        // EDF可调度性条件：总利用率 <= 1
        let total_utilization: f32 = tasks.iter()
            .map(|(execution_time, period)| *execution_time as f32 / *period as f32)
            .sum();
        
        total_utilization <= 1.0
    }
    
    /// 时钟中断处理
    pub fn tick(&mut self) {
        self.current_time += 1;
        
        // 检查当前任务
        if let Some(ref mut current) = self.current_task {
            current.remaining_time -= 1;
            
            // 检查任务是否完成
            if current.remaining_time == 0 {
                self.tasks[current.task_id].state = TaskState::Suspended;
                self.current_task = None;
            }
            // 检查是否错过截止时间
            else if self.current_time > current.absolute_deadline {
                self.missed_deadlines += 1;
                self.handle_deadline_miss(*current);
                self.current_task = None;
            }
        }
        
        // 检查堆中的任务截止时间
        let mut temp_tasks = Vec::<DeadlineTask, 16>::new();
        
        while let Some(task) = self.deadline_heap.pop() {
            if self.current_time > task.absolute_deadline {
                // 错过截止时间
                self.missed_deadlines += 1;
                self.handle_deadline_miss(task);
            } else {
                temp_tasks.push(task).ok();
            }
        }
        
        // 将未错过截止时间的任务放回堆中
        for task in temp_tasks {
            self.deadline_heap.push(task).ok();
        }
    }
    
    /// 调度下一个任务
    pub fn schedule(&mut self) -> Option<usize> {
        // 如果当前有任务在运行，检查是否需要抢占
        if let Some(current) = self.current_task {
            if let Some(&next) = self.deadline_heap.peek() {
                if next.absolute_deadline < current.absolute_deadline {
                    // 需要抢占
                    self.tasks[current.task_id].state = TaskState::Ready;
                    self.deadline_heap.push(current).ok();
                    self.current_task = None;
                }
            }
        }
        
        // 如果没有当前任务，从堆中取出最早截止时间的任务
        if self.current_task.is_none() {
            if let Some(next_task) = self.deadline_heap.pop() {
                self.tasks[next_task.task_id].state = TaskState::Running;
                self.current_task = Some(next_task);
                return Some(next_task.task_id);
            }
        }
        
        self.current_task.map(|task| task.task_id)
    }
    
    /// 处理截止时间错过
    fn handle_deadline_miss(&mut self, task: DeadlineTask) {
        // 记录错过的截止时间
        // 可以触发错误处理、任务重启或系统重配置
        self.tasks[task.task_id].state = TaskState::Suspended;
    }
    
    /// 获取调度统计信息
    pub fn get_statistics(&self) -> SchedulingStatistics {
        SchedulingStatistics {
            total_tasks: self.tasks.len(),
            missed_deadlines: self.missed_deadlines,
            current_time: self.current_time,
            utilization: self.calculate_utilization(),
        }
    }
    
    /// 计算CPU利用率
    fn calculate_utilization(&self) -> f32 {
        // 简化计算
        let active_tasks = self.tasks.iter()
            .filter(|task| task.state != TaskState::Suspended)
            .count();
        
        active_tasks as f32 / self.tasks.len() as f32
    }
}

/// 调度统计信息
#[derive(Debug)]
pub struct SchedulingStatistics {
    pub total_tasks: usize,
    pub missed_deadlines: u32,
    pub current_time: u32,
    pub utilization: f32,
}
```

## 性能监控和优化

### 调度性能监控

```rust
use heapless::Vec;

/// 任务性能统计
#[derive(Debug, Clone, Copy)]
pub struct TaskPerformance {
    pub task_id: usize,
    pub total_execution_time: u32,
    pub total_wait_time: u32,
    pub context_switches: u32,
    pub deadline_misses: u32,
    pub last_start_time: u32,
    pub last_end_time: u32,
}

impl TaskPerformance {
    pub fn new(task_id: usize) -> Self {
        Self {
            task_id,
            total_execution_time: 0,
            total_wait_time: 0,
            context_switches: 0,
            deadline_misses: 0,
            last_start_time: 0,
            last_end_time: 0,
        }
    }
    
    /// 计算平均响应时间
    pub fn average_response_time(&self) -> f32 {
        if self.context_switches > 0 {
            (self.total_execution_time + self.total_wait_time) as f32 / self.context_switches as f32
        } else {
            0.0
        }
    }
    
    /// 计算CPU利用率
    pub fn cpu_utilization(&self, total_time: u32) -> f32 {
        if total_time > 0 {
            self.total_execution_time as f32 / total_time as f32
        } else {
            0.0
        }
    }
}

/// 性能监控器
pub struct PerformanceMonitor {
    task_stats: Vec<TaskPerformance, 16>,
    system_start_time: u32,
    total_context_switches: u32,
    total_interrupts: u32,
    idle_time: u32,
}

impl PerformanceMonitor {
    pub fn new(system_start_time: u32) -> Self {
        Self {
            task_stats: Vec::new(),
            system_start_time,
            total_context_switches: 0,
            total_interrupts: 0,
            idle_time: 0,
        }
    }
    
    /// 注册任务
    pub fn register_task(&mut self, task_id: usize) -> Result<(), &'static str> {
        let stats = TaskPerformance::new(task_id);
        self.task_stats.push(stats).map_err(|_| "Too many tasks")
    }
    
    /// 任务开始执行
    pub fn task_started(&mut self, task_id: usize, current_time: u32) {
        if let Some(stats) = self.find_task_stats_mut(task_id) {
            stats.last_start_time = current_time;
            stats.context_switches += 1;
        }
        self.total_context_switches += 1;
    }
    
    /// 任务结束执行
    pub fn task_ended(&mut self, task_id: usize, current_time: u32) {
        if let Some(stats) = self.find_task_stats_mut(task_id) {
            stats.last_end_time = current_time;
            if stats.last_start_time > 0 {
                stats.total_execution_time += current_time - stats.last_start_time;
            }
        }
    }
    
    /// 任务错过截止时间
    pub fn deadline_missed(&mut self, task_id: usize) {
        if let Some(stats) = self.find_task_stats_mut(task_id) {
            stats.deadline_misses += 1;
        }
    }
    
    /// 记录中断
    pub fn interrupt_occurred(&mut self) {
        self.total_interrupts += 1;
    }
    
    /// 记录空闲时间
    pub fn idle_tick(&mut self) {
        self.idle_time += 1;
    }
    
    /// 查找任务统计信息
    fn find_task_stats_mut(&mut self, task_id: usize) -> Option<&mut TaskPerformance> {
        self.task_stats.iter_mut().find(|stats| stats.task_id == task_id)
    }
    
    /// 生成性能报告
    pub fn generate_report(&self, current_time: u32) -> PerformanceReport {
        let total_time = current_time - self.system_start_time;
        let system_utilization = if total_time > 0 {
            1.0 - (self.idle_time as f32 / total_time as f32)
        } else {
            0.0
        };
        
        let mut task_reports = Vec::<TaskReport, 16>::new();
        for stats in &self.task_stats {
            let report = TaskReport {
                task_id: stats.task_id,
                cpu_utilization: stats.cpu_utilization(total_time),
                average_response_time: stats.average_response_time(),
                context_switches: stats.context_switches,
                deadline_misses: stats.deadline_misses,
            };
            task_reports.push(report).ok();
        }
        
        PerformanceReport {
            system_utilization,
            total_context_switches: self.total_context_switches,
            total_interrupts: self.total_interrupts,
            uptime: total_time,
            task_reports,
        }
    }
}

/// 任务性能报告
#[derive(Debug, Clone, Copy)]
pub struct TaskReport {
    pub task_id: usize,
    pub cpu_utilization: f32,
    pub average_response_time: f32,
    pub context_switches: u32,
    pub deadline_misses: u32,
}

/// 系统性能报告
#[derive(Debug)]
pub struct PerformanceReport {
    pub system_utilization: f32,
    pub total_context_switches: u32,
    pub total_interrupts: u32,
    pub uptime: u32,
    pub task_reports: Vec<TaskReport, 16>,
}
```

### 调度优化技术

```rust
/// 自适应调度器
pub struct AdaptiveScheduler {
    base_scheduler: PreemptiveScheduler,
    performance_monitor: PerformanceMonitor,
    optimization_enabled: bool,
    load_balancing_threshold: f32,
    last_optimization_time: u32,
}

impl AdaptiveScheduler {
    pub fn new() -> Self {
        Self {
            base_scheduler: PreemptiveScheduler::new(),
            performance_monitor: PerformanceMonitor::new(0),
            optimization_enabled: true,
            load_balancing_threshold: 0.8,
            last_optimization_time: 0,
        }
    }
    
    /// 自适应调度
    pub fn adaptive_schedule(&mut self, current_time: u32) -> Option<usize> {
        // 定期进行性能分析和优化
        if current_time - self.last_optimization_time > 1000 { // 每1000ms优化一次
            self.optimize_scheduling(current_time);
            self.last_optimization_time = current_time;
        }
        
        // 执行基础调度
        let scheduled_task = self.base_scheduler.schedule();
        
        // 记录调度事件
        if let Some(task_id) = scheduled_task {
            self.performance_monitor.task_started(task_id, current_time);
        }
        
        scheduled_task
    }
    
    /// 调度优化
    fn optimize_scheduling(&mut self, current_time: u32) {
        if !self.optimization_enabled {
            return;
        }
        
        let report = self.performance_monitor.generate_report(current_time);
        
        // 检查系统负载
        if report.system_utilization > self.load_balancing_threshold {
            self.handle_high_load(&report);
        }
        
        // 检查任务性能
        for task_report in &report.task_reports {
            if task_report.deadline_misses > 0 {
                self.handle_deadline_misses(task_report.task_id);
            }
            
            if task_report.cpu_utilization > 0.5 {
                self.handle_high_cpu_usage(task_report.task_id);
            }
        }
    }
    
    /// 处理高负载
    fn handle_high_load(&mut self, _report: &PerformanceReport) {
        // 可以实施的优化策略：
        // 1. 降低低优先级任务的频率
        // 2. 启用更激进的抢占
        // 3. 调整时间片大小
        // 4. 启用任务合并
    }
    
    /// 处理截止时间错过
    fn handle_deadline_misses(&mut self, task_id: usize) {
        // 可以实施的策略：
        // 1. 提高任务优先级
        // 2. 增加时间片
        // 3. 减少其他任务的时间片
        // 4. 启用专用CPU核心（多核系统）
    }
    
    /// 处理高CPU使用率
    fn handle_high_cpu_usage(&mut self, task_id: usize) {
        // 可以实施的策略：
        // 1. 分解任务为更小的子任务
        // 2. 降低任务执行频率
        // 3. 优化任务算法
        // 4. 使用协作式调度
    }
}
```

## 完整的调度系统示例

```rust
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{prelude::*, stm32};

// 全局调度器
static mut SCHEDULER: Option<PreemptiveScheduler> = None;
static mut PERFORMANCE_MONITOR: Option<PerformanceMonitor> = None;

#[entry]
fn main() -> ! {
    // 硬件初始化
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 初始化调度器
    let mut scheduler = PreemptiveScheduler::new();
    let mut monitor = PerformanceMonitor::new(0);
    
    // 创建任务
    let task1 = TaskControlBlock::new(1, 3, task1_function, core::ptr::null_mut(), 1024);
    let task2 = TaskControlBlock::new(2, 2, task2_function, core::ptr::null_mut(), 1024);
    let task3 = TaskControlBlock::new(3, 1, task3_function, core::ptr::null_mut(), 1024);
    
    // 添加任务到调度器
    let task1_id = scheduler.add_task(task1).unwrap();
    let task2_id = scheduler.add_task(task2).unwrap();
    let task3_id = scheduler.add_task(task3).unwrap();
    
    // 注册任务到性能监控器
    monitor.register_task(task1_id).unwrap();
    monitor.register_task(task2_id).unwrap();
    monitor.register_task(task3_id).unwrap();
    
    // 设置全局调度器
    unsafe {
        SCHEDULER = Some(scheduler);
        PERFORMANCE_MONITOR = Some(monitor);
    }
    
    // 配置SysTick定时器
    let mut systick = cp.SYST;
    systick.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    systick.set_reload(84_000 - 1); // 1ms @ 84MHz
    systick.clear_current();
    systick.enable_counter();
    systick.enable_interrupt();
    
    // 主调度循环
    loop {
        unsafe {
            if let Some(ref mut scheduler) = SCHEDULER {
                if let Some(task_id) = scheduler.schedule() {
                    // 执行任务（简化实现）
                    execute_task(task_id);
                }
            }
        }
        
        // 进入低功耗模式等待中断
        cortex_m::asm::wfi();
    }
}

// SysTick中断处理
#[cortex_m_rt::exception]
fn SysTick() {
    unsafe {
        if let Some(ref mut scheduler) = SCHEDULER {
            if scheduler.tick() {
                // 需要重新调度
                // 在实际实现中，这里会触发PendSV中断进行上下文切换
            }
        }
        
        if let Some(ref mut monitor) = PERFORMANCE_MONITOR {
            // 更新性能统计
            monitor.idle_tick();
        }
    }
}

// 任务函数
fn task1_function() {
    // 高优先级任务：LED控制
    loop {
        toggle_led();
        delay_ms(100);
    }
}

fn task2_function() {
    // 中优先级任务：传感器读取
    loop {
        let sensor_data = read_sensor();
        process_sensor_data(sensor_data);
        delay_ms(200);
    }
}

fn task3_function() {
    // 低优先级任务：数据处理
    loop {
        process_background_data();
        delay_ms(500);
    }
}

// 简化的任务执行函数
fn execute_task(task_id: usize) {
    match task_id {
        0 => task1_function(),
        1 => task2_function(),
        2 => task3_function(),
        _ => {}
    }
}

// 辅助函数
fn toggle_led() {
    // LED切换实现
}

fn read_sensor() -> u16 {
    // 传感器读取实现
    42
}

fn process_sensor_data(data: u16) {
    // 传感器数据处理
}

fn process_background_data() {
    // 后台数据处理
}

fn delay_ms(ms: u32) {
    // 延时实现
    for _ in 0..(ms * 1000) {
        cortex_m::asm::nop();
    }
}
```

## 总结

任务调度是RTOS的核心功能，不同的调度算法适用于不同的应用场景：

- **抢占式调度**: 适用于实时性要求高的系统
- **协作式调度**: 适用于任务间协作良好的系统
- **时间片轮转**: 适用于公平性要求高的系统
- **优先级调度**: 适用于任务重要性差异明显的系统
- **实时调度**: 适用于硬实时系统

在实际应用中，需要根据系统需求选择合适的调度算法，并通过性能监控和优化技术来提升系统性能。合理的任务设计、优先级分配和资源管理是构建高效实时系统的关键。