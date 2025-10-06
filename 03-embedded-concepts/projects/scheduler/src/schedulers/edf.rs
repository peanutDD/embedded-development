//! 最早截止期优先调度器实现

use super::SchedulerImpl;
use crate::{Scheduler, SchedulerConfig, SchedulerError, SchedulerResult, SchedulerStatistics};
use crate::{Task, TaskConfig, TaskPriority, TaskState};
use alloc::vec::Vec;
use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

/// 最早截止期优先调度器
/// 
/// 基于任务截止期的动态优先级调度算法，截止期越早优先级越高
pub struct EdfScheduler {
    /// 内部调度器实现
    inner: RefCell<SchedulerImpl>,
    /// 系统时钟计数器
    system_clock: AtomicU32,
    /// 配置
    config: SchedulerConfig,
}

impl EdfScheduler {
    /// 创建新的最早截止期优先调度器
    pub fn new(config: SchedulerConfig) -> SchedulerResult<Self> {
        let inner = SchedulerImpl::new(config.max_tasks)?;
        
        Ok(Self {
            inner: RefCell::new(inner),
            system_clock: AtomicU32::new(0),
            config: config.clone(),
        })
    }

    /// 更新系统时钟
    pub fn update_system_clock(&self, ticks: u32) {
        self.system_clock.fetch_add(ticks, Ordering::Relaxed);
    }

    /// 获取当前系统时间
    pub fn current_time(&self) -> u32 {
        self.system_clock.load(Ordering::Relaxed)
    }

    /// 计算任务的下一个截止期
    fn calculate_deadline(&self, task: &Task) -> u32 {
        if task.deadline > 0 {
            task.next_run_time + task.deadline
        } else {
            // 如果没有指定截止期，使用周期作为截止期
            task.next_run_time + task.period
        }
    }
}

impl Scheduler for EdfScheduler {
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
        
        // 更新任务状态
        for task in inner.tasks.iter_mut() {
            if let Some(ref mut task) = task {
                // 检查任务是否到达执行时间
                if task.state == TaskState::Ready && 
                   current_time >= task.next_run_time {
                    task.state = TaskState::Waiting;
                }
                
                // 检查任务是否超时
                if task.state == TaskState::Running {
                    let deadline = self.calculate_deadline(task);
                    if deadline > 0 && current_time > deadline {
                        inner.statistics.deadline_misses += 1;
                    }
                }
            }
        }
        
        // 选择最早截止期的等待任务（EDF核心逻辑）
        let mut earliest_deadline_task: Option<u8> = None;
        let mut earliest_deadline = u32::MAX;
        
        for (i, task) in inner.tasks.iter().enumerate() {
            if let Some(ref task) = task {
                if task.state == TaskState::Waiting {
                    let deadline = self.calculate_deadline(task);
                    if deadline < earliest_deadline {
                        earliest_deadline = deadline;
                        earliest_deadline_task = Some(i as u8);
                    }
                }
            }
        }
        
        // 更新统计信息
        inner.statistics.scheduling_decisions += 1;
        
        if let Some(task_id) = earliest_deadline_task {
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
                    (inner.statistics.average_response_time_us * (task.run_count - 1) + response_time) / task.run_count;
                
                if response_time > inner.statistics.max_response_time_us {
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
        
        Ok(earliest_deadline_task)
    }

    fn is_schedulable(&self) -> bool {
        // EDF的可调度性测试：CPU利用率 <= 1.0
        let inner = self.inner.borrow();
        inner.statistics.cpu_utilization <= 1.0
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