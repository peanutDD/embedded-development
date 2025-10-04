# 实时系统基础

## 概述

实时系统是指能够在规定时间内完成特定任务的计算机系统。在嵌入式开发中，实时性是一个关键要求，特别是在工业控制、汽车电子、医疗设备等对时间敏感的应用中。

## 学习目标

完成本章节后，你将掌握：
- 实时系统的定义、分类和特点
- 任务调度算法和调度策略
- 时间约束和截止期管理
- 优先级反转问题及其解决方案
- 实时性分析和验证方法
- 实时系统的设计和实现技术

## 1. 实时系统基础概念

### 1.1 实时系统定义

实时系统的正确性不仅取决于计算结果的逻辑正确性，还取决于产生结果的时间：

```rust
// 实时任务的基本特征
struct RealTimeTask {
    id: u32,
    period: u32,           // 周期 (ms)
    deadline: u32,         // 截止期 (ms)
    wcet: u32,            // 最坏情况执行时间 (ms)
    priority: u8,         // 优先级
    release_time: u32,    // 释放时间
    completion_time: u32, // 完成时间
}

impl RealTimeTask {
    fn is_deadline_met(&self) -> bool {
        self.completion_time <= self.deadline
    }
    
    fn response_time(&self) -> u32 {
        self.completion_time - self.release_time
    }
}
```

### 1.2 实时系统分类

#### 按时间约束严格程度分类

```rust
#[derive(Debug, Clone, Copy)]
enum RealTimeType {
    Hard,    // 硬实时：错过截止期会导致系统失效
    Soft,    // 软实时：错过截止期会降低系统性能
    Firm,    // 固实时：错过截止期的结果无用但不危险
}

struct SystemRequirement {
    task_type: RealTimeType,
    max_deadline_miss_rate: f32,  // 最大截止期错失率
    consequence_of_miss: &'static str,
}

const SAFETY_CRITICAL: SystemRequirement = SystemRequirement {
    task_type: RealTimeType::Hard,
    max_deadline_miss_rate: 0.0,
    consequence_of_miss: "System failure, potential safety hazard",
};

const USER_INTERFACE: SystemRequirement = SystemRequirement {
    task_type: RealTimeType::Soft,
    max_deadline_miss_rate: 0.05,  // 5%的错失率可接受
    consequence_of_miss: "Reduced user experience",
};
```

#### 按任务特性分类

```rust
#[derive(Debug, Clone, Copy)]
enum TaskType {
    Periodic,     // 周期性任务
    Aperiodic,    // 非周期性任务
    Sporadic,     // 偶发性任务
}

struct TaskCharacteristics {
    task_type: TaskType,
    min_inter_arrival_time: Option<u32>,  // 最小到达间隔
    max_execution_time: u32,              // 最大执行时间
    relative_deadline: u32,               // 相对截止期
}

// 周期性任务示例：传感器数据采集
const SENSOR_TASK: TaskCharacteristics = TaskCharacteristics {
    task_type: TaskType::Periodic,
    min_inter_arrival_time: Some(100),  // 每100ms执行一次
    max_execution_time: 5,               // 最多执行5ms
    relative_deadline: 100,              // 截止期等于周期
};

// 偶发性任务示例：紧急事件处理
const EMERGENCY_TASK: TaskCharacteristics = TaskCharacteristics {
    task_type: TaskType::Sporadic,
    min_inter_arrival_time: Some(1000), // 最小间隔1秒
    max_execution_time: 50,              // 最多执行50ms
    relative_deadline: 200,              // 200ms内必须完成
};
```

## 2. 任务调度算法

### 2.1 静态优先级调度

#### 速率单调调度(Rate Monotonic, RM)

```rust
use heapless::Vec;
use heapless::binary_heap::{BinaryHeap, Max};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PeriodicTask {
    id: u32,
    period: u32,
    execution_time: u32,
    next_deadline: u32,
    priority: u32,  // 优先级 = 1/period (数值越大优先级越高)
}

impl PartialOrd for PeriodicTask {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PeriodicTask {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.priority.cmp(&other.priority)
    }
}

struct RateMonotonicScheduler {
    tasks: Vec<PeriodicTask, 16>,
    ready_queue: BinaryHeap<PeriodicTask, Max, 16>,
    current_time: u32,
}

impl RateMonotonicScheduler {
    fn new() -> Self {
        Self {
            tasks: Vec::new(),
            ready_queue: BinaryHeap::new(),
            current_time: 0,
        }
    }
    
    fn add_task(&mut self, mut task: PeriodicTask) -> Result<(), &'static str> {
        // RM调度：周期越短，优先级越高
        task.priority = 1000000 / task.period;  // 避免浮点运算
        task.next_deadline = task.period;
        
        self.tasks.push(task).map_err(|_| "Task queue full")?;
        Ok(())
    }
    
    fn schedule(&mut self) -> Option<PeriodicTask> {
        // 检查是否有任务到达
        for task in &self.tasks {
            if self.current_time % task.period == 0 {
                let mut ready_task = *task;
                ready_task.next_deadline = self.current_time + task.period;
                let _ = self.ready_queue.push(ready_task);
            }
        }
        
        // 选择最高优先级任务
        self.ready_queue.pop()
    }
    
    fn is_schedulable(&self) -> bool {
        // RM可调度性测试：利用率测试
        let n = self.tasks.len() as f32;
        let utilization_bound = n * (2.0_f32.powf(1.0 / n) - 1.0);
        
        let total_utilization: f32 = self.tasks.iter()
            .map(|task| task.execution_time as f32 / task.period as f32)
            .sum();
        
        total_utilization <= utilization_bound
    }
}
```

#### 截止期单调调度(Deadline Monotonic, DM)

```rust
struct DeadlineMonotonicScheduler {
    tasks: Vec<PeriodicTask, 16>,
    ready_queue: BinaryHeap<PeriodicTask, Max, 16>,
}

impl DeadlineMonotonicScheduler {
    fn add_task(&mut self, mut task: PeriodicTask, deadline: u32) -> Result<(), &'static str> {
        // DM调度：截止期越短，优先级越高
        task.priority = 1000000 / deadline;
        self.tasks.push(task).map_err(|_| "Task queue full")?;
        Ok(())
    }
    
    fn response_time_analysis(&self, task_index: usize) -> Option<u32> {
        let task = &self.tasks[task_index];
        let mut response_time = task.execution_time;
        
        loop {
            let mut interference = 0;
            
            // 计算高优先级任务的干扰
            for other_task in &self.tasks {
                if other_task.priority > task.priority {
                    interference += (response_time + other_task.period - 1) / other_task.period 
                                  * other_task.execution_time;
                }
            }
            
            let new_response_time = task.execution_time + interference;
            
            if new_response_time == response_time {
                return Some(response_time);
            }
            
            if new_response_time > task.period {
                return None;  // 不可调度
            }
            
            response_time = new_response_time;
        }
    }
}
```

### 2.2 动态优先级调度

#### 最早截止期优先(Earliest Deadline First, EDF)

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct EDFTask {
    id: u32,
    execution_time: u32,
    absolute_deadline: u32,
    remaining_time: u32,
}

impl PartialOrd for EDFTask {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for EDFTask {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        // 截止期越早，优先级越高（注意：BinaryHeap是最大堆）
        other.absolute_deadline.cmp(&self.absolute_deadline)
    }
}

struct EDFScheduler {
    ready_queue: BinaryHeap<EDFTask, Max, 16>,
    current_time: u32,
}

impl EDFScheduler {
    fn new() -> Self {
        Self {
            ready_queue: BinaryHeap::new(),
            current_time: 0,
        }
    }
    
    fn add_task(&mut self, task: EDFTask) -> Result<(), &'static str> {
        self.ready_queue.push(task).map_err(|_| "Queue full")
    }
    
    fn schedule(&mut self) -> Option<EDFTask> {
        // 移除已过期的任务
        while let Some(task) = self.ready_queue.peek() {
            if task.absolute_deadline < self.current_time {
                self.ready_queue.pop();
            } else {
                break;
            }
        }
        
        self.ready_queue.pop()
    }
    
    fn is_schedulable(&self, tasks: &[PeriodicTask]) -> bool {
        // EDF可调度性测试：总利用率不超过100%
        let total_utilization: f32 = tasks.iter()
            .map(|task| task.execution_time as f32 / task.period as f32)
            .sum();
        
        total_utilization <= 1.0
    }
}
```

### 2.3 混合调度策略

```rust
#[derive(Debug, Clone, Copy)]
enum SchedulingPolicy {
    RateMonotonic,
    DeadlineMonotonic,
    EarliestDeadlineFirst,
    FixedPriority(u8),
}

struct HybridScheduler {
    high_priority_tasks: BinaryHeap<PeriodicTask, Max, 8>,  // 硬实时任务
    normal_tasks: BinaryHeap<PeriodicTask, Max, 8>,         // 软实时任务
    background_tasks: Vec<PeriodicTask, 8>,                 // 后台任务
}

impl HybridScheduler {
    fn schedule(&mut self) -> Option<PeriodicTask> {
        // 优先调度硬实时任务
        if let Some(task) = self.high_priority_tasks.pop() {
            return Some(task);
        }
        
        // 其次调度软实时任务
        if let Some(task) = self.normal_tasks.pop() {
            return Some(task);
        }
        
        // 最后调度后台任务
        if !self.background_tasks.is_empty() {
            return self.background_tasks.pop();
        }
        
        None
    }
    
    fn add_task(&mut self, task: PeriodicTask, policy: SchedulingPolicy) -> Result<(), &'static str> {
        match policy {
            SchedulingPolicy::FixedPriority(p) if p >= 200 => {
                self.high_priority_tasks.push(task).map_err(|_| "High priority queue full")
            }
            SchedulingPolicy::RateMonotonic | SchedulingPolicy::DeadlineMonotonic => {
                self.normal_tasks.push(task).map_err(|_| "Normal priority queue full")
            }
            _ => {
                self.background_tasks.push(task).map_err(|_| "Background queue full")
            }
        }
    }
}
```

## 3. 时间约束和截止期管理

### 3.1 时间约束类型

```rust
#[derive(Debug, Clone, Copy)]
struct TimeConstraint {
    constraint_type: ConstraintType,
    value: u32,
    tolerance: u32,
}

#[derive(Debug, Clone, Copy)]
enum ConstraintType {
    AbsoluteDeadline(u32),    // 绝对截止期
    RelativeDeadline(u32),    // 相对截止期
    MaxResponseTime(u32),     // 最大响应时间
    MinInterArrival(u32),     // 最小到达间隔
    MaxJitter(u32),           // 最大抖动
}

struct DeadlineManager {
    active_deadlines: Vec<(u32, u32), 32>,  // (task_id, deadline)
    current_time: u32,
    missed_deadlines: u32,
}

impl DeadlineManager {
    fn new() -> Self {
        Self {
            active_deadlines: Vec::new(),
            current_time: 0,
            missed_deadlines: 0,
        }
    }
    
    fn add_deadline(&mut self, task_id: u32, deadline: u32) -> Result<(), &'static str> {
        self.active_deadlines.push((task_id, deadline))
            .map_err(|_| "Deadline queue full")
    }
    
    fn check_deadlines(&mut self) -> Vec<u32, 16> {
        let mut missed_tasks = Vec::new();
        
        self.active_deadlines.retain(|(task_id, deadline)| {
            if *deadline <= self.current_time {
                let _ = missed_tasks.push(*task_id);
                self.missed_deadlines += 1;
                false  // 移除已过期的截止期
            } else {
                true   // 保留未过期的截止期
            }
        });
        
        missed_tasks
    }
    
    fn get_deadline_miss_rate(&self) -> f32 {
        if self.active_deadlines.is_empty() && self.missed_deadlines == 0 {
            0.0
        } else {
            self.missed_deadlines as f32 / 
            (self.active_deadlines.len() + self.missed_deadlines as usize) as f32
        }
    }
}
```

### 3.2 时间预算管理

```rust
struct TimeBudget {
    allocated_time: u32,
    consumed_time: u32,
    start_time: u32,
    budget_exceeded: bool,
}

impl TimeBudget {
    fn new(allocated_time: u32, start_time: u32) -> Self {
        Self {
            allocated_time,
            consumed_time: 0,
            start_time,
            budget_exceeded: false,
        }
    }
    
    fn update(&mut self, current_time: u32) {
        self.consumed_time = current_time - self.start_time;
        self.budget_exceeded = self.consumed_time > self.allocated_time;
    }
    
    fn remaining_time(&self) -> u32 {
        if self.consumed_time >= self.allocated_time {
            0
        } else {
            self.allocated_time - self.consumed_time
        }
    }
    
    fn utilization(&self) -> f32 {
        self.consumed_time as f32 / self.allocated_time as f32
    }
}

struct BudgetScheduler {
    task_budgets: Vec<(u32, TimeBudget), 16>,  // (task_id, budget)
    current_time: u32,
}

impl BudgetScheduler {
    fn allocate_budget(&mut self, task_id: u32, budget_time: u32) -> Result<(), &'static str> {
        let budget = TimeBudget::new(budget_time, self.current_time);
        self.task_budgets.push((task_id, budget))
            .map_err(|_| "Budget table full")
    }
    
    fn check_budget_violation(&mut self) -> Vec<u32, 16> {
        let mut violating_tasks = Vec::new();
        
        for (task_id, budget) in &mut self.task_budgets {
            budget.update(self.current_time);
            if budget.budget_exceeded {
                let _ = violating_tasks.push(*task_id);
            }
        }
        
        violating_tasks
    }
}
```

## 4. 优先级反转问题

### 4.1 优先级反转现象

```rust
use core::sync::atomic::{AtomicBool, Ordering};

// 模拟优先级反转场景
struct SharedResource {
    data: u32,
    locked: AtomicBool,
    owner: Option<u32>,  // 持有锁的任务ID
}

impl SharedResource {
    fn new() -> Self {
        Self {
            data: 0,
            locked: AtomicBool::new(false),
            owner: None,
        }
    }
    
    fn try_lock(&mut self, task_id: u32) -> bool {
        if self.locked.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed).is_ok() {
            self.owner = Some(task_id);
            true
        } else {
            false
        }
    }
    
    fn unlock(&mut self, task_id: u32) -> Result<(), &'static str> {
        if self.owner == Some(task_id) {
            self.owner = None;
            self.locked.store(false, Ordering::Release);
            Ok(())
        } else {
            Err("Task does not own the lock")
        }
    }
}

// 优先级反转示例
fn priority_inversion_example() {
    let mut resource = SharedResource::new();
    
    // 低优先级任务获得锁
    assert!(resource.try_lock(1));  // 任务1（低优先级）
    
    // 高优先级任务被阻塞
    assert!(!resource.try_lock(3)); // 任务3（高优先级）被阻塞
    
    // 中优先级任务可以抢占低优先级任务
    // 导致高优先级任务间接被中优先级任务阻塞
}
```

### 4.2 优先级继承协议(Priority Inheritance Protocol, PIP)

```rust
use heapless::FnvIndexMap;

struct PriorityInheritanceMutex {
    locked: AtomicBool,
    owner: Option<u32>,
    original_priority: Option<u8>,
    inherited_priority: Option<u8>,
    waiting_tasks: Vec<(u32, u8), 8>,  // (task_id, priority)
}

impl PriorityInheritanceMutex {
    fn new() -> Self {
        Self {
            locked: AtomicBool::new(false),
            owner: None,
            original_priority: None,
            inherited_priority: None,
            waiting_tasks: Vec::new(),
        }
    }
    
    fn lock(&mut self, task_id: u32, task_priority: u8) -> Result<(), &'static str> {
        if self.locked.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed).is_ok() {
            // 成功获得锁
            self.owner = Some(task_id);
            self.original_priority = Some(task_priority);
            Ok(())
        } else {
            // 锁被占用，检查是否需要优先级继承
            if let Some(owner_priority) = self.inherited_priority.or(self.original_priority) {
                if task_priority > owner_priority {
                    // 提升锁持有者的优先级
                    self.inherited_priority = Some(task_priority);
                    self.boost_owner_priority(task_priority);
                }
            }
            
            // 将任务加入等待队列
            self.waiting_tasks.push((task_id, task_priority))
                .map_err(|_| "Waiting queue full")?;
            
            Err("Lock is held by another task")
        }
    }
    
    fn unlock(&mut self, task_id: u32) -> Result<(), &'static str> {
        if self.owner != Some(task_id) {
            return Err("Task does not own the lock");
        }
        
        // 恢复原始优先级
        if let Some(original) = self.original_priority {
            self.restore_owner_priority(original);
        }
        
        // 释放锁
        self.owner = None;
        self.original_priority = None;
        self.inherited_priority = None;
        self.locked.store(false, Ordering::Release);
        
        // 唤醒等待的最高优先级任务
        if let Some((next_task, next_priority)) = self.get_highest_priority_waiter() {
            self.owner = Some(next_task);
            self.original_priority = Some(next_priority);
            self.locked.store(true, Ordering::Acquire);
        }
        
        Ok(())
    }
    
    fn boost_owner_priority(&self, new_priority: u8) {
        // 实际实现中需要调用调度器API提升任务优先级
        // scheduler.boost_priority(self.owner.unwrap(), new_priority);
    }
    
    fn restore_owner_priority(&self, original_priority: u8) {
        // 实际实现中需要调用调度器API恢复任务优先级
        // scheduler.restore_priority(self.owner.unwrap(), original_priority);
    }
    
    fn get_highest_priority_waiter(&mut self) -> Option<(u32, u8)> {
        if self.waiting_tasks.is_empty() {
            return None;
        }
        
        // 找到最高优先级的等待任务
        let max_index = self.waiting_tasks.iter()
            .enumerate()
            .max_by_key(|(_, (_, priority))| *priority)
            .map(|(index, _)| index)?;
        
        Some(self.waiting_tasks.swap_remove(max_index))
    }
}
```

### 4.3 优先级天花板协议(Priority Ceiling Protocol, PCP)

```rust
struct PriorityCeilingMutex {
    locked: AtomicBool,
    owner: Option<u32>,
    ceiling_priority: u8,  // 天花板优先级
    system_ceiling: u8,    // 系统天花板
}

impl PriorityCeilingMutex {
    fn new(ceiling_priority: u8) -> Self {
        Self {
            locked: AtomicBool::new(false),
            owner: None,
            ceiling_priority,
            system_ceiling: 0,
        }
    }
    
    fn lock(&mut self, task_id: u32, task_priority: u8) -> Result<(), &'static str> {
        // PCP规则：只有当任务优先级高于系统天花板时才能获得锁
        if task_priority <= self.system_ceiling {
            return Err("Task priority too low for current system ceiling");
        }
        
        if self.locked.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed).is_ok() {
            self.owner = Some(task_id);
            // 提升系统天花板
            self.system_ceiling = self.ceiling_priority;
            Ok(())
        } else {
            Err("Lock is held by another task")
        }
    }
    
    fn unlock(&mut self, task_id: u32) -> Result<(), &'static str> {
        if self.owner != Some(task_id) {
            return Err("Task does not own the lock");
        }
        
        self.owner = None;
        self.locked.store(false, Ordering::Release);
        
        // 降低系统天花板
        self.system_ceiling = self.calculate_system_ceiling();
        
        Ok(())
    }
    
    fn calculate_system_ceiling(&self) -> u8 {
        // 计算当前系统中所有被锁定资源的最高天花板
        // 这里简化为0，实际实现需要维护全局状态
        0
    }
}
```

## 5. 实时性分析方法

### 5.1 响应时间分析

```rust
struct ResponseTimeAnalyzer {
    tasks: Vec<PeriodicTask, 16>,
}

impl ResponseTimeAnalyzer {
    fn worst_case_response_time(&self, task_index: usize) -> Option<u32> {
        let task = &self.tasks[task_index];
        let mut response_time = task.execution_time;
        let mut prev_response_time = 0;
        
        // 迭代计算响应时间
        while response_time != prev_response_time && response_time <= task.period {
            prev_response_time = response_time;
            let mut interference = 0;
            
            // 计算高优先级任务的干扰
            for (i, other_task) in self.tasks.iter().enumerate() {
                if i != task_index && other_task.priority > task.priority {
                    let preemptions = (response_time + other_task.period - 1) / other_task.period;
                    interference += preemptions * other_task.execution_time;
                }
            }
            
            response_time = task.execution_time + interference;
        }
        
        if response_time <= task.period {
            Some(response_time)
        } else {
            None  // 任务不可调度
        }
    }
    
    fn schedulability_analysis(&self) -> Vec<(usize, Option<u32>), 16> {
        let mut results = Vec::new();
        
        for i in 0..self.tasks.len() {
            let response_time = self.worst_case_response_time(i);
            let _ = results.push((i, response_time));
        }
        
        results
    }
    
    fn blocking_time_analysis(&self, task_index: usize, resources: &[PriorityCeilingMutex]) -> u32 {
        let task = &self.tasks[task_index];
        let mut max_blocking = 0;
        
        // 计算最大阻塞时间
        for resource in resources {
            if resource.ceiling_priority >= task.priority {
                // 找到可能阻塞该任务的最长临界区
                for other_task in &self.tasks {
                    if other_task.priority < task.priority {
                        // 这里需要知道其他任务使用该资源的时间
                        // 简化处理，假设为执行时间的10%
                        let blocking_time = other_task.execution_time / 10;
                        max_blocking = max_blocking.max(blocking_time);
                    }
                }
            }
        }
        
        max_blocking
    }
}
```

### 5.2 利用率分析

```rust
struct UtilizationAnalyzer {
    tasks: Vec<PeriodicTask, 16>,
}

impl UtilizationAnalyzer {
    fn cpu_utilization(&self) -> f32 {
        self.tasks.iter()
            .map(|task| task.execution_time as f32 / task.period as f32)
            .sum()
    }
    
    fn rm_schedulability_bound(&self) -> f32 {
        let n = self.tasks.len() as f32;
        n * (2.0_f32.powf(1.0 / n) - 1.0)
    }
    
    fn edf_schedulability_test(&self) -> bool {
        self.cpu_utilization() <= 1.0
    }
    
    fn rm_schedulability_test(&self) -> bool {
        self.cpu_utilization() <= self.rm_schedulability_bound()
    }
    
    fn hyperbolic_bound_test(&self) -> bool {
        let product: f32 = self.tasks.iter()
            .map(|task| {
                let utilization = task.execution_time as f32 / task.period as f32;
                utilization + 1.0
            })
            .product();
        
        product <= 2.0
    }
    
    fn detailed_analysis(&self) -> SchedulabilityResult {
        let utilization = self.cpu_utilization();
        let rm_bound = self.rm_schedulability_bound();
        
        SchedulabilityResult {
            total_utilization: utilization,
            rm_bound,
            edf_schedulable: self.edf_schedulability_test(),
            rm_schedulable: self.rm_schedulability_test(),
            hyperbolic_schedulable: self.hyperbolic_bound_test(),
            utilization_percentage: utilization * 100.0,
        }
    }
}

#[derive(Debug)]
struct SchedulabilityResult {
    total_utilization: f32,
    rm_bound: f32,
    edf_schedulable: bool,
    rm_schedulable: bool,
    hyperbolic_schedulable: bool,
    utilization_percentage: f32,
}
```

## 6. 实时系统设计技术

### 6.1 时间触发架构

```rust
use cortex_m::peripheral::SYST;

struct TimeTriggeredScheduler {
    tasks: Vec<TimeTriggeredTask, 16>,
    major_cycle: u32,
    minor_cycle: u32,
    current_slot: u32,
    tick_count: u32,
}

#[derive(Debug, Clone, Copy)]
struct TimeTriggeredTask {
    id: u32,
    execution_slots: &'static [u32],  // 执行时隙
    execution_time: u32,
    handler: fn(),
}

impl TimeTriggeredScheduler {
    fn new(major_cycle: u32, minor_cycle: u32) -> Self {
        Self {
            tasks: Vec::new(),
            major_cycle,
            minor_cycle,
            current_slot: 0,
            tick_count: 0,
        }
    }
    
    fn add_task(&mut self, task: TimeTriggeredTask) -> Result<(), &'static str> {
        // 验证时隙分配的有效性
        for &slot in task.execution_slots {
            if slot >= self.major_cycle / self.minor_cycle {
                return Err("Invalid time slot");
            }
        }
        
        self.tasks.push(task).map_err(|_| "Task queue full")
    }
    
    fn tick(&mut self) {
        self.tick_count += 1;
        self.current_slot = (self.tick_count * self.minor_cycle) % self.major_cycle / self.minor_cycle;
        
        // 执行当前时隙的任务
        for task in &self.tasks {
            if task.execution_slots.contains(&self.current_slot) {
                (task.handler)();
            }
        }
    }
    
    fn setup_system_timer(&self, systick: &mut SYST, system_clock: u32) {
        let reload_value = system_clock / (1000 / self.minor_cycle) - 1;
        systick.set_reload(reload_value);
        systick.clear_current();
        systick.enable_counter();
        systick.enable_interrupt();
    }
}

// 系统滴答中断处理
static mut TT_SCHEDULER: Option<TimeTriggeredScheduler> = None;

#[exception]
fn SysTick() {
    unsafe {
        if let Some(ref mut scheduler) = TT_SCHEDULER {
            scheduler.tick();
        }
    }
}
```

### 6.2 事件触发架构

```rust
use heapless::spsc::{Producer, Consumer, Queue};

#[derive(Debug, Clone, Copy)]
enum SystemEvent {
    TimerExpired(u32),
    GpioInterrupt(u8),
    UartDataReceived(u8),
    AdcConversionComplete(u16),
    UserInput(u32),
}

struct EventTriggeredScheduler {
    event_queue: Queue<SystemEvent, 64>,
    event_handlers: Vec<(SystemEvent, fn(SystemEvent)), 16>,
}

impl EventTriggeredScheduler {
    fn new() -> (Self, Producer<SystemEvent, 64>, Consumer<SystemEvent, 64>) {
        let queue = Queue::new();
        let (producer, consumer) = queue.split();
        
        let scheduler = Self {
            event_queue: Queue::new(),
            event_handlers: Vec::new(),
        };
        
        (scheduler, producer, consumer)
    }
    
    fn register_handler(&mut self, event_type: SystemEvent, handler: fn(SystemEvent)) -> Result<(), &'static str> {
        self.event_handlers.push((event_type, handler))
            .map_err(|_| "Handler table full")
    }
    
    fn process_events(&mut self, consumer: &mut Consumer<SystemEvent, 64>) {
        while let Some(event) = consumer.dequeue() {
            self.handle_event(event);
        }
    }
    
    fn handle_event(&self, event: SystemEvent) {
        for (event_pattern, handler) in &self.event_handlers {
            if core::mem::discriminant(event_pattern) == core::mem::discriminant(&event) {
                handler(event);
                break;
            }
        }
    }
}

// 事件处理函数示例
fn handle_timer_event(event: SystemEvent) {
    if let SystemEvent::TimerExpired(timer_id) = event {
        match timer_id {
            1 => periodic_sensor_reading(),
            2 => heartbeat_transmission(),
            3 => system_health_check(),
            _ => {}
        }
    }
}

fn handle_gpio_event(event: SystemEvent) {
    if let SystemEvent::GpioInterrupt(pin) = event {
        match pin {
            0 => emergency_stop(),
            1 => user_button_pressed(),
            2 => limit_switch_triggered(),
            _ => {}
        }
    }
}
```

### 6.3 混合架构

```rust
struct HybridScheduler {
    time_triggered: TimeTriggeredScheduler,
    event_triggered: EventTriggeredScheduler,
    event_consumer: Consumer<SystemEvent, 64>,
    current_mode: SchedulingMode,
}

#[derive(Debug, Clone, Copy)]
enum SchedulingMode {
    TimeCritical,    // 时间关键模式：只执行时间触发任务
    Normal,          // 正常模式：混合调度
    EventDriven,     // 事件驱动模式：只处理事件
}

impl HybridScheduler {
    fn schedule(&mut self) {
        match self.current_mode {
            SchedulingMode::TimeCritical => {
                // 只执行时间触发的关键任务
                self.time_triggered.tick();
            }
            SchedulingMode::Normal => {
                // 先执行时间触发任务
                self.time_triggered.tick();
                
                // 然后处理事件（如果有时间）
                let start_time = get_current_time();
                while get_current_time() - start_time < self.get_available_time_budget() {
                    if self.event_consumer.dequeue().is_none() {
                        break;
                    }
                }
            }
            SchedulingMode::EventDriven => {
                // 只处理事件
                self.event_triggered.process_events(&mut self.event_consumer);
            }
        }
    }
    
    fn switch_mode(&mut self, new_mode: SchedulingMode) {
        self.current_mode = new_mode;
    }
    
    fn get_available_time_budget(&self) -> u32 {
        // 计算当前时隙剩余的时间
        self.time_triggered.minor_cycle - (get_current_time() % self.time_triggered.minor_cycle)
    }
}

fn get_current_time() -> u32 {
    // 返回当前系统时间（毫秒）
    cortex_m::peripheral::DWT::cycle_count() / (SystemCoreClock / 1000)
}

const SystemCoreClock: u32 = 168_000_000; // 168MHz
```

## 7. 实时性测试和验证

### 7.1 时序测试

```rust
struct TimingTestSuite {
    test_results: Vec<TimingTestResult, 32>,
    current_test: Option<TimingTest>,
}

#[derive(Debug, Clone)]
struct TimingTest {
    name: &'static str,
    expected_max_time: u32,
    expected_min_time: u32,
    start_time: u32,
    iterations: u32,
}

#[derive(Debug, Clone)]
struct TimingTestResult {
    test_name: &'static str,
    min_time: u32,
    max_time: u32,
    avg_time: u32,
    jitter: u32,
    pass: bool,
}

impl TimingTestSuite {
    fn new() -> Self {
        Self {
            test_results: Vec::new(),
            current_test: None,
        }
    }
    
    fn start_test(&mut self, name: &'static str, expected_max: u32, expected_min: u32) {
        self.current_test = Some(TimingTest {
            name,
            expected_max_time: expected_max,
            expected_min_time: expected_min,
            start_time: DWT::cycle_count(),
            iterations: 0,
        });
    }
    
    fn end_test(&mut self) -> Option<TimingTestResult> {
        if let Some(mut test) = self.current_test.take() {
            let end_time = DWT::cycle_count();
            let execution_time = end_time.wrapping_sub(test.start_time);
            
            // 简化的结果计算
            let result = TimingTestResult {
                test_name: test.name,
                min_time: execution_time,
                max_time: execution_time,
                avg_time: execution_time,
                jitter: 0,
                pass: execution_time <= test.expected_max_time && 
                      execution_time >= test.expected_min_time,
            };
            
            let _ = self.test_results.push(result.clone());
            Some(result)
        } else {
            None
        }
    }
    
    fn run_interrupt_latency_test(&mut self) -> TimingTestResult {
        self.start_test("Interrupt Latency", 100, 10); // 10-100 cycles
        
        // 触发软件中断并测量延迟
        cortex_m::peripheral::SCB::set_pendsv();
        
        // 等待中断完成
        while cortex_m::peripheral::SCB::icsr.read().pendsvset().bit_is_set() {}
        
        self.end_test().unwrap()
    }
    
    fn run_context_switch_test(&mut self) -> TimingTestResult {
        self.start_test("Context Switch", 500, 50); // 50-500 cycles
        
        // 模拟上下文切换
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        
        self.end_test().unwrap()
    }
}
```

### 7.2 负载测试

```rust
struct LoadTestSuite {
    cpu_utilization: f32,
    memory_utilization: f32,
    interrupt_load: f32,
    test_duration: u32,
}

impl LoadTestSuite {
    fn run_cpu_load_test(&mut self, target_load: f32, duration_ms: u32) {
        let start_time = get_current_time();
        let load_cycles = (target_load * duration_ms as f32 * SystemCoreClock as f32 / 1000.0) as u32;
        
        while get_current_time() - start_time < duration_ms {
            // 生成CPU负载
            for _ in 0..load_cycles / 1000 {
                cortex_m::asm::nop();
            }
            
            // 测量实际CPU利用率
            self.measure_cpu_utilization();
        }
    }
    
    fn measure_cpu_utilization(&mut self) {
        // 使用DWT计数器测量CPU利用率
        static mut IDLE_CYCLES: u32 = 0;
        static mut TOTAL_CYCLES: u32 = 0;
        static mut LAST_MEASUREMENT: u32 = 0;
        
        let current_cycles = DWT::cycle_count();
        unsafe {
            let elapsed = current_cycles.wrapping_sub(LAST_MEASUREMENT);
            TOTAL_CYCLES += elapsed;
            
            // 如果在空闲任务中，增加空闲计数
            if self.is_idle_task() {
                IDLE_CYCLES += elapsed;
            }
            
            if TOTAL_CYCLES > 0 {
                self.cpu_utilization = 1.0 - (IDLE_CYCLES as f32 / TOTAL_CYCLES as f32);
            }
            
            LAST_MEASUREMENT = current_cycles;
        }
    }
    
    fn is_idle_task(&self) -> bool {
        // 检查当前是否在空闲任务中
        // 这里需要根据具体的RTOS实现
        false
    }
    
    fn stress_test(&mut self, duration_ms: u32) -> StressTestResult {
        let start_time = get_current_time();
        let mut max_response_time = 0;
        let mut deadline_misses = 0;
        let mut total_tasks = 0;
        
        while get_current_time() - start_time < duration_ms {
            // 创建高频率的任务
            let task_start = get_current_time();
            self.execute_test_task();
            let task_end = get_current_time();
            
            let response_time = task_end - task_start;
            max_response_time = max_response_time.max(response_time);
            
            if response_time > 10 { // 假设截止期为10ms
                deadline_misses += 1;
            }
            
            total_tasks += 1;
        }
        
        StressTestResult {
            max_response_time,
            deadline_miss_rate: deadline_misses as f32 / total_tasks as f32,
            total_tasks,
            test_duration: duration_ms,
        }
    }
    
    fn execute_test_task(&self) {
        // 执行测试任务
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
    }
}

#[derive(Debug)]
struct StressTestResult {
    max_response_time: u32,
    deadline_miss_rate: f32,
    total_tasks: u32,
    test_duration: u32,
}
```

## 8. 总结

实时系统是嵌入式开发中的核心概念，正确理解和应用实时系统理论对于开发可靠的嵌入式应用至关重要。

### 关键要点

1. **实时性定义**: 理解硬实时、软实时和固实时的区别
2. **调度算法**: 掌握RM、DM、EDF等调度算法的原理和应用
3. **优先级管理**: 理解优先级反转问题及其解决方案
4. **时间分析**: 学会进行响应时间分析和可调度性测试
5. **系统架构**: 了解时间触发和事件触发架构的特点
6. **测试验证**: 掌握实时性测试和验证方法

### 设计原则

- **可预测性**: 系统行为必须是可预测的
- **确定性**: 相同输入应产生相同的时序行为
- **可分析性**: 系统必须能够进行时序分析
- **容错性**: 系统应能处理异常情况
- **可维护性**: 保持代码的清晰和可维护

### 实践建议

- 从简单的周期性任务开始学习
- 使用仿真工具验证调度算法
- 在实际硬件上测量时序性能
- 建立系统性能基准和监控
- 学习和应用形式化验证方法

### 下一步

完成实时系统学习后，建议继续学习：
- [功耗管理技术](./05-power-management.md)
- [RTOS集成](../../09-rtos-integration/README.md)
- [高级RTOS特性](../../15-rtos-advanced/README.md)