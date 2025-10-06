//! 任务管理模块

use crate::{SchedulerError, SchedulerResult};
use core::sync::atomic::{AtomicU32, Ordering};

/// 任务状态枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskState {
    /// 就绪状态，等待运行
    Ready,
    /// 等待状态，已经准备好运行
    Waiting,
    /// 运行状态
    Running,
    /// 挂起状态
    Suspended,
    /// 终止状态
    Terminated,
}

/// 任务优先级类型
pub type TaskPriority = u8;

impl TaskPriority {
    /// 最低优先级
    pub const MIN: Self = 0;
    /// 最高优先级
    pub const MAX: Self = 255;
}

/// 任务配置结构体
#[derive(Debug, Clone, Copy)]
pub struct TaskConfig {
    /// 任务函数指针
    pub function: fn(),
    /// 任务名称
    pub name: &'static str,
    /// 任务优先级
    pub priority: TaskPriority,
    /// 任务周期（微秒），0表示一次性任务
    pub period: u32,
    /// 任务执行时间估计（微秒）
    pub execution_time: u32,
    /// 任务截止期（微秒），0表示使用周期作为截止期
    pub deadline: u32,
    /// 初始延迟（微秒）
    pub initial_delay: u32,
}

impl TaskConfig {
    /// 创建新的任务配置
    pub const fn new(
        function: fn(),
        name: &'static str,
        priority: TaskPriority,
        period: u32,
        execution_time: u32,
        deadline: u32,
        initial_delay: u32,
    ) -> Self {
        Self {
            function,
            name,
            priority,
            period,
            execution_time,
            deadline,
            initial_delay,
        }
    }
}

/// 任务结构体
#[derive(Debug)]
pub struct Task {
    /// 任务函数指针
    pub function: fn(),
    /// 任务名称
    pub name: &'static str,
    /// 任务优先级
    pub priority: TaskPriority,
    /// 任务周期（微秒）
    pub period: u32,
    /// 任务执行时间估计（微秒）
    pub execution_time: u32,
    /// 任务截止期（微秒）
    pub deadline: u32,
    /// 任务状态
    pub state: TaskState,
    /// 上次运行时间
    pub last_run_time: u32,
    /// 下次运行时间
    pub next_run_time: u32,
    /// 运行次数
    pub run_count: u32,
    /// 总运行时间
    pub total_execution_time: AtomicU32,
    /// 最大运行时间
    pub max_execution_time: u32,
    /// 最小运行时间
    pub min_execution_time: u32,
}

impl Task {
    /// 创建新的任务
    pub fn new(config: TaskConfig) -> SchedulerResult<Self> {
        // 验证任务参数
        if config.execution_time > config.period && config.period > 0 {
            // 执行时间不能大于周期
            return Err(SchedulerError::TaskCreationFailed);
        }
        
        Ok(Self {
            function: config.function,
            name: config.name,
            priority: config.priority,
            period: config.period,
            execution_time: config.execution_time,
            deadline: config.deadline,
            state: TaskState::Ready,
            last_run_time: 0,
            next_run_time: config.initial_delay,
            run_count: 0,
            total_execution_time: AtomicU32::new(0),
            max_execution_time: 0,
            min_execution_time: u32::MAX,
        })
    }

    /// 运行任务
    pub fn run(&self) {
        let start_time = cortex_m::peripheral::DWT::get_cycle_count();
        
        // 调用任务函数
        (self.function)();
        
        let end_time = cortex_m::peripheral::DWT::get_cycle_count();
        let execution_time = end_time.wrapping_sub(start_time);
        
        // 更新统计信息
        self.total_execution_time.fetch_add(execution_time, Ordering::Relaxed);
    }

    /// 挂起任务
    pub fn suspend(&mut self) -> SchedulerResult<()> {
        if self.state != TaskState::Terminated {
            self.state = TaskState::Suspended;
            Ok(())
        } else {
            Err(SchedulerError::TaskTerminated)
        }
    }

    /// 恢复任务
    pub fn resume(&mut self) -> SchedulerResult<()> {
        if self.state == TaskState::Suspended {
            self.state = TaskState::Ready;
            Ok(())
        } else {
            Err(SchedulerError::InvalidTaskState)
        }
    }

    /// 终止任务
    pub fn terminate(&mut self) -> SchedulerResult<()> {
        if self.state != TaskState::Terminated {
            self.state = TaskState::Terminated;
            Ok(())
        } else {
            Err(SchedulerError::TaskTerminated)
        }
    }

    /// 更新任务优先级
    pub fn set_priority(&mut self, priority: TaskPriority) {
        self.priority = priority;
    }

    /// 更新任务周期
    pub fn set_period(&mut self, period: u32) {
        self.period = period;
    }

    /// 获取平均执行时间
    pub fn get_average_execution_time(&self) -> u32 {
        if self.run_count > 0 {
            self.total_execution_time.load(Ordering::Relaxed) / self.run_count
        } else {
            0
        }
    }

    /// 检查是否为周期性任务
    pub fn is_periodic(&self) -> bool {
        self.period > 0
    }

    /// 检查是否为实时任务
    pub fn is_real_time(&self) -> bool {
        self.deadline > 0
    }
}