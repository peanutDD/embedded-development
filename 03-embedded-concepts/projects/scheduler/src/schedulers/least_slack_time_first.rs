//! 最少松弛时间优先调度器实现

use crate::{Scheduler, SchedulerConfig, SchedulerError, SchedulerResult, SchedulerStatistics};
use crate::{Task, TaskConfig, TaskPriority, TaskState};
use alloc::vec::Vec;
use alloc::vec;
use core::cell::RefCell;
use core::ops::DerefMut;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicU32, Ordering};

/// 最少松弛时间优先调度器
pub struct LeastSlackTimeFirstScheduler {
    /// 内部调度器实现
    inner: RefCell<LeastSlackTimeFirstSchedulerImpl>,
    /// 系统时钟计数器
    system_clock: AtomicU32,
    /// 配置
    config: SchedulerConfig,
}

impl LeastSlackTimeFirstScheduler {
    /// 创建新的最少松弛时间优先调度器
    pub fn new(config: SchedulerConfig) -> SchedulerResult<Self> {
        let inner = LeastSlackTimeFirstSchedulerImpl::new(config.max_tasks)?;
        
        Ok(Self {
            inner: RefCell::new(inner),
            system_clock: AtomicU32::new(0),
            config: config.clone(),
        })
    }

    /// 获取当前系统时间
    fn current_time(&self) -> u32 {
        self.system_clock.load(Ordering::Relaxed)
    }

    /// 计算任务的松弛时间
    fn calculate_slack_time(&self, task: &Task, current_time: u32) -> i32 {
        // 松弛时间 = 截止期 - 剩余执行时间 - 当前时间
        // 这里简化处理，假设剩余执行时间为0，实际应根据任务状态更新
        task.deadline as i32 - (task.execution_time as i32) - current_time as i32
    }
}

impl Scheduler for LeastSlackTimeFirstScheduler {
    fn add_task(&mut self, task_config: TaskConfig) -> SchedulerResult<u8> {
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
        
        // 选择松弛时间最小的等待任务
        let mut least_slack_time_task: Option<u8> = None;
        let mut min_slack_time = i32::MAX;
        
        for (i, task) in inner.tasks.iter().enumerate() {
            if let Some(ref task) = task {
                if task.state == TaskState::Waiting {
                    let slack_time = self.calculate_slack_time(task, current_time);
                    if slack_time < min_slack_time {
                        min_slack_time = slack_time;
                        least_slack_time_task = Some(i as u8);
                    }
                }
            }
        }
        
        // 如果有任务需要运行，则更新其状态
        if let Some(task_id) = least_slack_time_task {
            let task_switched = inner.current_task != Some(task_id);
            
            if let Some(task) = inner.tasks[task_id as usize].as_mut() {
                task.state = TaskState::Running;
            }
            
            // 在释放task引用后再更新其他字段
            if task_switched {
                // 任务切换
                inner.statistics.context_switches += 1;
                inner.current_task = Some(task_id);
            }
        }
        
        inner.statistics.scheduling_decisions += 1;
        Ok(least_slack_time_task)
    }

    fn is_schedulable(&self) -> bool {
        // TODO: 实现最少松弛时间优先调度的可调度性分析
        // 这是一个复杂的分析，通常涉及利用率测试或响应时间分析。
        // 暂时返回true，待后续实现。
        true
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
pub struct LeastSlackTimeFirstSchedulerImpl {
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

impl LeastSlackTimeFirstSchedulerImpl {
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
        // 查找可用的任务ID
        for (i, task) in self.tasks.iter().enumerate() {
            if task.is_none() {
                let task = Task::new(task_config)?;
                
                // 计算CPU利用率
                if task.period > 0 {
                    let utilization = (task.execution_time as f64 / task.period as f64) as f32;
                    self.statistics.cpu_utilization += utilization;
                }
                
                self.tasks[i] = Some(task);
                return Ok(i as u8);
            }
        }
        
        Err(SchedulerError::TooManyTasks)
    }

    /// 移除任务
    pub fn remove_task(&mut self, task_id: u8) -> SchedulerResult<()> {
        if task_id as usize >= self.tasks.len() {
            return Err(SchedulerError::InvalidTaskId);
        }
        
        if let Some(task) = self.tasks[task_id as usize].take() {
            // 更新CPU利用率
            if task.period > 0 {
                let utilization = (task.execution_time as f64 / task.period as f64) as f32;
                self.statistics.cpu_utilization -= utilization;
            }
            
            // 如果移除的是当前运行的任务，设置当前任务为None
            if self.current_task == Some(task_id) {
                self.current_task = None;
            }
            
            return Ok(());
        }
        
        Err(SchedulerError::InvalidTaskId)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{TaskConfig, TaskPriority};

    #[test]
    fn test_least_slack_time_first_scheduler_creation() {
        let config = SchedulerConfig {
            max_tasks: 4,
            time_slice_us: 1000,
            enable_profiling: false,
            enable_deadlock_detection: false,
            scheduler_type: crate::SchedulerType::LeastSlackTimeFirst,
            system_clock_hz: 84_000_000,
        };
        let scheduler = LeastSlackTimeFirstScheduler::new(config);
        assert!(scheduler.is_ok());
        let scheduler = scheduler.unwrap();
        assert_eq!(scheduler.config.max_tasks, 4);
    }

    #[test]
    fn test_least_slack_time_first_scheduler_add_remove_task() {
        let config = SchedulerConfig {
            max_tasks: 4,
            time_slice_us: 1000,
            enable_profiling: false,
            enable_deadlock_detection: false,
            scheduler_type: crate::SchedulerType::LeastSlackTimeFirst,
            system_clock_hz: 84_000_000,
        };
        let mut scheduler = LeastSlackTimeFirstScheduler::new(config).unwrap();

        let task_config1 = TaskConfig {
            id: 0,
            period: 100,
            execution_time: 20,
            deadline: 50,
            priority: TaskPriority::Low,
        };
        let task_id1 = scheduler.add_task(task_config1).unwrap();
        assert_eq!(task_id1, 0);

        let task_config2 = TaskConfig {
            id: 1,
            period: 200,
            execution_time: 30,
            deadline: 150,
            priority: TaskPriority::Low,
        };
        let task_id2 = scheduler.add_task(task_config2).unwrap();
        assert_eq!(task_id2, 1);

        assert!(scheduler.remove_task(task_id1).is_ok());
        assert!(scheduler.inner.borrow().tasks[0].is_none());
    }

    #[test]
    fn test_least_slack_time_first_scheduler_schedule() {
        let config = SchedulerConfig {
            max_tasks: 4,
            time_slice_us: 1000,
            enable_profiling: false,
            enable_deadlock_detection: false,
            scheduler_type: crate::SchedulerType::LeastSlackTimeFirst,
            system_clock_hz: 84_000_000,
        };
        let mut scheduler = LeastSlackTimeFirstScheduler::new(config).unwrap();

        let task_config1 = TaskConfig {
            id: 0,
            period: 100,
            execution_time: 20,
            deadline: 50,
            priority: TaskPriority::Low,
        };
        scheduler.add_task(task_config1).unwrap();

        let task_config2 = TaskConfig {
            id: 1,
            period: 200,
            execution_time: 30,
            deadline: 150,
            priority: TaskPriority::Low,
        };
        scheduler.add_task(task_config2).unwrap();

        // Simulate time passing and task readiness
        // Task 0 (slack time 50 - 20 = 30) should have higher priority than Task 1 (slack time 150 - 30 = 120)
        scheduler.inner.borrow_mut().tasks[0].as_mut().unwrap().state = TaskState::Waiting;
        scheduler.inner.borrow_mut().tasks[0].as_mut().unwrap().next_run_time = 0;
        scheduler.inner.borrow_mut().tasks[1].as_mut().unwrap().state = TaskState::Waiting;
        scheduler.inner.borrow_mut().tasks[1].as_mut().unwrap().next_run_time = 0;

        // Manually advance system clock for slack time calculation
        scheduler.system_clock.store(0, Ordering::Relaxed);

        let scheduled_task = scheduler.schedule().unwrap();
        assert_eq!(scheduled_task, Some(0)); // Task 0 should be scheduled first

        // Simulate Task 0 completing and Task 1 still waiting
        scheduler.inner.borrow_mut().tasks[0].as_mut().unwrap().state = TaskState::Completed;
        let scheduled_task = scheduler.schedule().unwrap();
        assert_eq!(scheduled_task, Some(1)); // Task 1 should be scheduled next
    }

    #[test]
    fn test_least_slack_time_first_scheduler_statistics() {
        let config = SchedulerConfig {
            max_tasks: 2,
            time_slice_us: 1000,
            enable_profiling: false,
            enable_deadlock_detection: false,
            scheduler_type: crate::SchedulerType::LeastSlackTimeFirst,
            system_clock_hz: 84_000_000,
        };
        let mut scheduler = LeastSlackTimeFirstScheduler::new(config).unwrap();

        let task_config = TaskConfig {
            id: 0,
            period: 100,
            execution_time: 20,
            deadline: 50,
            priority: TaskPriority::Low,
        };
        scheduler.add_task(task_config).unwrap();

        scheduler.inner.borrow_mut().tasks[0].as_mut().unwrap().state = TaskState::Waiting;
        scheduler.inner.borrow_mut().tasks[0].as_mut().unwrap().next_run_time = 0;

        scheduler.schedule().unwrap();
        let stats = scheduler.statistics();
        assert_eq!(stats.scheduling_decisions, 1);
        assert_eq!(stats.context_switches, 1);

        scheduler.reset_statistics();
        let stats = scheduler.statistics();
        assert_eq!(stats.scheduling_decisions, 0);
    }
}