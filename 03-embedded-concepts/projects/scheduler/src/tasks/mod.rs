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
    /// 完成状态
    Completed,
}

/// 任务优先级类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct TaskPriority(u8);

impl TaskPriority {
    /// 最低优先级
    pub const MIN: Self = Self(0);
    /// 最高优先级
    pub const MAX: Self = Self(255);
    
    /// 关键优先级
    pub const Critical: Self = Self(255);
    /// 高优先级
    pub const High: Self = Self(192);
    /// 中优先级
    pub const Medium: Self = Self(128);
    /// 低优先级
    pub const Low: Self = Self(64);
    
    /// 创建一个新的任务优先级
    pub const fn new(value: u8) -> Self {
        Self(value)
    }
    
    /// 获取优先级的数值
    pub const fn get(&self) -> u8 {
        self.0
    }
    
    /// 获取优先级的数值（与get方法相同）
    pub const fn value(&self) -> u8 {
        self.0
    }
}

/// 任务配置结构体
#[derive(Debug, Clone, Copy)]
pub struct TaskConfig {
    /// 任务ID
    pub id: u8,
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
        id: u8,
        function: fn(),
        name: &'static str,
        priority: TaskPriority,
        period: u32,
        execution_time: u32,
        deadline: u32,
        initial_delay: u32,
    ) -> Self {
        Self {
            id,
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
    /// 任务ID
    pub id: u8,
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

/// 任务构建器结构体
#[derive(Debug, Clone, Copy)]
pub struct TaskBuilder {
    config: TaskConfig,
}

impl TaskBuilder {
    /// 创建一个新的任务构建器
    pub const fn new() -> Self {
        Self {
            config: TaskConfig {
                id: 0,
                function: || {},
                name: "Unnamed Task",
                priority: TaskPriority::Medium,
                period: 0,
                execution_time: 0,
                deadline: 0,
                initial_delay: 0,
            },
        }
    }
    
    /// 设置任务ID
    pub const fn id(mut self, id: u8) -> Self {
        self.config.id = id;
        self
    }
    
    /// 设置任务函数
    pub const fn function(mut self, function: fn()) -> Self {
        self.config.function = function;
        self
    }
    
    /// 设置任务名称
    pub const fn name(mut self, name: &'static str) -> Self {
        self.config.name = name;
        self
    }
    
    /// 设置任务优先级
    pub const fn priority(mut self, priority: TaskPriority) -> Self {
        self.config.priority = priority;
        self
    }
    
    /// 设置任务周期
    pub const fn period(mut self, period: u32) -> Self {
        self.config.period = period;
        self
    }
    
    /// 设置任务执行时间
    pub const fn execution_time(mut self, execution_time: u32) -> Self {
        self.config.execution_time = execution_time;
        self
    }
    
    /// 设置任务截止期
    pub const fn deadline(mut self, deadline: u32) -> Self {
        self.config.deadline = deadline;
        self
    }
    
    /// 设置初始延迟
    pub const fn initial_delay(mut self, initial_delay: u32) -> Self {
        self.config.initial_delay = initial_delay;
        self
    }
    
    /// 构建任务配置
    pub const fn build_config(self) -> TaskConfig {
        self.config
    }
    
    /// 直接构建任务
    pub fn build(self) -> SchedulerResult<Task> {
        Task::new(self.config)
    }
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
            id: config.id,
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
    pub fn run(&mut self) {
        let start_time = cortex_m::peripheral::DWT::get_cycle_count();
        
        // 调用任务函数
        (self.function)();
        
        let end_time = cortex_m::peripheral::DWT::get_cycle_count();
        let execution_time = end_time.wrapping_sub(start_time);
        
        // 优化原子操作：仅在需要时才进行原子加法
        // 对于高频任务，减少原子操作频率
        if self.run_count % 10 == 0 {
            self.total_execution_time.fetch_add(execution_time * 10, Ordering::Relaxed);
        } else if self.run_count % 10 == 9 {
            self.total_execution_time.fetch_add(execution_time, Ordering::Relaxed);
        }
        
        // 更新运行次数
        self.run_count += 1;
        
        // 优化最大/最小执行时间更新：使用简单的条件检查
        if execution_time > self.max_execution_time {
            self.max_execution_time = execution_time;
        }
        
        if execution_time < self.min_execution_time {
            self.min_execution_time = execution_time;
        }
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