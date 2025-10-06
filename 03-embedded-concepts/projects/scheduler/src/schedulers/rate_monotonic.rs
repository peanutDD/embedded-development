//! 速率单调调度器实现

// 注意：这里不再导入SchedulerImpl，因为我们已经将其重命名为RateMonotonicSchedulerImpl
use crate::{Scheduler, SchedulerConfig, SchedulerError, SchedulerResult, SchedulerStatistics};
use crate::{Task, TaskConfig, TaskPriority, TaskState};
use alloc::vec::Vec;
use alloc::vec;
use core::cell::RefCell;
use core::ops::DerefMut;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicU32, Ordering};

/// 速率单调调度器
/// 
/// 基于任务周期的静态优先级调度算法，周期越短优先级越高
pub struct RateMonotonicScheduler {
    /// 内部调度器实现
    inner: RefCell<RateMonotonicSchedulerImpl>,
    /// 系统时钟计数器
    system_clock: AtomicU32,
    /// 配置
    config: SchedulerConfig,
}

impl RateMonotonicScheduler {
    /// 创建新的速率单调调度器
    pub fn new(config: SchedulerConfig) -> SchedulerResult<Self> {
        let inner = RateMonotonicSchedulerImpl::new(config.max_tasks)?;
        
        Ok(Self {
            inner: RefCell::new(inner),
            system_clock: AtomicU32::new(0),
            config: config.clone(),
        })
    }

    /// 根据任务周期计算优先级
    /// 
    /// 周期越短，优先级越高
    fn calculate_priority(&self, period: u32) -> TaskPriority {
        // 这里使用简单的映射，实际应用中可能需要更复杂的策略
        let priority = if period <= 10 {
            TaskPriority::Critical
        } else if period <= 100 {
            TaskPriority::High
        } else if period <= 1000 {
            TaskPriority::Medium
        } else {
            TaskPriority::Low
        };
        
        priority
    }

    /// 更新系统时钟
    pub fn update_system_clock(&self, ticks: u32) {
        self.system_clock.fetch_add(ticks, Ordering::Relaxed);
    }

    /// 获取当前系统时间
    pub fn current_time(&self) -> u32 {
        self.system_clock.load(Ordering::Relaxed)
    }
}

impl Scheduler for RateMonotonicScheduler {
    fn add_task(&mut self, mut task_config: TaskConfig) -> SchedulerResult<u8> {
        // 根据任务周期设置优先级（周期越短优先级越高）
        if task_config.priority == TaskPriority::Low {
            // 如果用户没有指定优先级，使用速率单调调度的优先级策略
            task_config.priority = self.calculate_priority(task_config.period);
        }
        
        let mut inner = self.inner.borrow_mut();
        inner.add_task(task_config)
    }

    fn remove_task(&mut self, task_id: u8) -> SchedulerResult<()> {
        let mut inner = self.inner.borrow_mut();
        inner.remove_task(task_id)
    }

    fn schedule(&mut self) -> SchedulerResult<Option<u8>> {
        let current_time = self.current_time();
        let mut inner = self.inner.borrow_mut();
        
        // 收集需要更新的任务状态和超时任务数
        let mut deadline_misses_count = 0;
        for task in inner.tasks.iter_mut() {
            if let Some(ref mut task) = task {
                // 检查任务是否到达执行时间
                if task.state == TaskState::Ready && 
                   current_time >= task.next_run_time {
                    task.state = TaskState::Waiting;
                }
                
                // 检查任务是否超时
                if task.state == TaskState::Running && 
                   task.deadline > 0 && 
                   current_time > task.next_run_time + task.deadline {
                    deadline_misses_count += 1;
                }
            }
        }
        
        // 在循环外更新统计信息
        inner.statistics.deadline_misses += deadline_misses_count;
        
        // 选择最高优先级的等待任务
        let mut highest_priority_task: Option<u8> = None;
        let mut highest_priority = TaskPriority::Low;
        
        for (i, task) in inner.tasks.iter().enumerate() {
            if let Some(ref task) = task {
                if task.state == TaskState::Waiting && task.priority > highest_priority {
                    highest_priority = task.priority;
                    highest_priority_task = Some(i as u8);
                }
            }
        }
        
        // 更新统计信息
        inner.statistics.scheduling_decisions += 1;
        
        if let Some(task_id) = highest_priority_task {
            // 先获取需要的统计信息
            let current_avg_response_time = inner.statistics.average_response_time_us;
            let current_max_response_time = inner.statistics.max_response_time_us;
            
            // 更新任务状态
            let task = inner.tasks[task_id as usize].as_mut().unwrap();
            task.state = TaskState::Running;
            task.last_run_time = current_time;
            task.next_run_time = current_time + task.period;
            task.run_count += 1;
            
            // 计算响应时间
            if task.period > 0 {
                let response_time = current_time - task.last_run_time;
                inner.statistics.average_response_time_us = 
                    (current_avg_response_time * (task.run_count - 1) + response_time) / task.run_count;
                
                if response_time > current_max_response_time {
                    inner.statistics.max_response_time_us = response_time;
                }
            }
            
            // 增加上下文切换计数
            if inner.current_task != Some(task_id) {
                inner.statistics.context_switches += 1;
                inner.current_task = Some(task_id);
            }
        } else {
            // 没有等待的任务
            inner.current_task = None;
        }
        
        Ok(highest_priority_task)
    }

    fn is_schedulable(&self) -> bool {
        // 速率单调调度的可调度性测试：CPU利用率 <= n*(2^(1/n)-1)
        let inner = self.inner.borrow();
        let active_tasks = inner.tasks.iter()
            .filter(|t| t.is_some() && t.as_ref().unwrap().state != TaskState::Terminated)
            .count();
        
        if active_tasks == 0 {
            return true;
        }
        
        let utilization = inner.statistics.cpu_utilization;
        
        // 对于Rate Monotonic调度，使用近似值
        // 当任务数大于4时，n*(2^(1/n)-1) ≈ ln(2) ≈ 0.693
        // 为了简化计算，使用固定值0.693作为边界条件
        const RM_BOUND_APPROXIMATION: f32 = 0.693;
        
        utilization <= RM_BOUND_APPROXIMATION
    }

    fn start(&mut self) -> SchedulerResult<()> {
        let mut inner = self.inner.borrow_mut();
        inner.running = true;
        Ok(())
    }

    fn stop(&mut self) -> SchedulerResult<()> {
        let mut inner = self.inner.borrow_mut();
        inner.running = false;
        inner.current_task = None;
        Ok(())
    }

    fn statistics(&self) -> SchedulerStatistics {
        self.inner.borrow().statistics
    }

    fn reset_statistics(&mut self) {
        let mut inner = self.inner.borrow_mut();
        inner.statistics = SchedulerStatistics::new();
    }
}

/// 内部调度器实现
pub struct RateMonotonicSchedulerImpl {
    /// 任务数组
    pub tasks: Vec<Option<Task>>,
    /// 当前运行的任务
    pub current_task: Option<u8>,
    /// 是否正在运行
    pub running: bool,
    /// 统计信息
    pub statistics: SchedulerStatistics,
    /// 最大任务数量
    pub max_tasks: usize,
}

impl RateMonotonicSchedulerImpl {
    /// 创建新的内部调度器实现
    pub fn new(max_tasks: usize) -> SchedulerResult<Self> {
        if max_tasks == 0 {
            return Err(SchedulerError::TooManyTasks);
        }
        
        let mut tasks = Vec::with_capacity(max_tasks);
        for _ in 0..max_tasks {
            tasks.push(None);
        }
        
        Ok(Self {
            tasks,
            current_task: None,
            running: false,
            statistics: SchedulerStatistics::new(),
            max_tasks,
        })
    }

    /// 添加任务
    pub fn add_task(&mut self, task_config: TaskConfig) -> SchedulerResult<u8> {
        // 查找空闲的任务槽
        for (i, task_slot) in self.tasks.iter_mut().enumerate() {
            if task_slot.is_none() {
                // 创建任务
                let task = Task::new(task_config)?;
                *task_slot = Some(task);
                
                // 更新CPU利用率
                if task_config.execution_time > 0 && task_config.period > 0 {
                    let task_utilization = task_config.execution_time as f32 / task_config.period as f32;
                    self.statistics.cpu_utilization += task_utilization;
                }
                
                return Ok(i as u8);
            }
        }
        
        Err(SchedulerError::TooManyTasks)
    }

    /// 移除任务
    pub fn remove_task(&mut self, task_id: u8) -> SchedulerResult<()> {
        if task_id as usize >= self.max_tasks {
            return Err(SchedulerError::InvalidTaskId);
        }
        
        if let Some(task) = &self.tasks[task_id as usize] {
            // 更新CPU利用率
            if task.execution_time > 0 && task.period > 0 {
                let task_utilization = task.execution_time as f32 / task.period as f32;
                self.statistics.cpu_utilization -= task_utilization;
            }
            
            // 如果移除的是当前任务，更新当前任务
            if self.current_task == Some(task_id) {
                self.current_task = None;
            }
            
            self.tasks[task_id as usize] = None;
            Ok(())
        } else {
            Err(SchedulerError::InvalidTaskId)
        }
    }
}