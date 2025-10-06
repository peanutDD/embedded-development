//! 调度算法实现模块

// 导出调度器实现
pub mod rate_monotonic;
pub mod edf;
pub mod pip;

// 重导出
pub use rate_monotonic::RateMonotonicScheduler;
pub use edf::EdfScheduler;
pub use pip::{PipScheduler, Mutex};

use crate::{Scheduler, SchedulerConfig, SchedulerError, SchedulerResult, SchedulerStatistics};
use crate::{Task, TaskConfig, TaskPriority, TaskState};
use alloc::vec::Vec;
use alloc::vec;
use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

/// 内部调度器实现
pub struct SchedulerImpl {
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

impl SchedulerImpl {
    /// 创建新的内部调度器实现
    pub fn new(max_tasks: u8) -> SchedulerResult<Self> {
        if max_tasks == 0 || max_tasks > 255 {
            return Err(SchedulerError::InvalidConfiguration("Invalid max_tasks value (must be between 1 and 255)"));
        }
        
        // 使用循环初始化tasks向量，因为Task包含不能Clone的AtomicU32
        let mut tasks = Vec::with_capacity(max_tasks as usize);
        for _ in 0..max_tasks as usize {
            tasks.push(None);
        }
        
        Ok(Self {
            tasks,
            current_task: None,
            running: false,
            statistics: SchedulerStatistics::new(),
            max_tasks: max_tasks as usize,
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