#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]

//! # 嵌入式实时调度器
//! 
//! 本库提供了多种适用于嵌入式系统的实时调度算法实现。

pub mod schedulers;
pub mod tasks;
pub mod sync;
pub mod analysis;

// 重新导出主要类型
pub use schedulers::{
    rate_monotonic::RateMonotonicScheduler,
    edf::EdfScheduler,
    deadline_monotonic::DeadlineMonotonicScheduler,
};

pub use tasks::{
    task::{Task, TaskConfig, TaskState, TaskPriority},
    task_manager::TaskManager,
};

pub use sync::{
    mutex::PriorityInheritanceMutex,
    semaphore::Semaphore,
};

pub use analysis::{
    schedulability::SchedulabilityAnalyzer,
    response_time::ResponseTimeAnalyzer,
    utilization::UtilizationAnalyzer,
};

/// 调度器错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerError {
    /// 任务数量超出限制
    TooManyTasks,
    /// 任务创建失败
    TaskCreationFailed,
    /// 无效的任务ID
    InvalidTaskId,
    /// 调度失败
    SchedulingFailed,
    /// 任务集不可调度
    NotSchedulable,
    /// 资源不可用
    ResourceUnavailable,
    /// 死锁检测
    DeadlockDetected,
    /// 优先级反转
    PriorityInversion,
}

impl core::fmt::Display for SchedulerError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SchedulerError::TooManyTasks => write!(f, "Too many tasks"),
            SchedulerError::TaskCreationFailed => write!(f, "Task creation failed"),
            SchedulerError::InvalidTaskId => write!(f, "Invalid task ID"),
            SchedulerError::SchedulingFailed => write!(f, "Scheduling failed"),
            SchedulerError::NotSchedulable => write!(f, "Task set not schedulable"),
            SchedulerError::ResourceUnavailable => write!(f, "Resource unavailable"),
            SchedulerError::DeadlockDetected => write!(f, "Deadlock detected"),
            SchedulerError::PriorityInversion => write!(f, "Priority inversion detected"),
        }
    }
}

/// 调度器结果类型
pub type SchedulerResult<T> = Result<T, SchedulerError>;

/// 调度器类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerType {
    /// 速率单调调度
    RateMonotonic,
    /// 截止期单调调度
    DeadlineMonotonic,
    /// 最早截止期优先
    EarliestDeadlineFirst,
    /// 最少松弛时间优先
    LeastSlackTimeFirst,
    /// 混合调度
    Hybrid,
}

/// 调度器配置
#[derive(Debug, Clone)]
pub struct SchedulerConfig {
    /// 最大任务数量
    pub max_tasks: usize,
    /// 时间片长度（微秒）
    pub time_slice_us: u32,
    /// 是否启用性能剖析
    pub enable_profiling: bool,
    /// 是否启用死锁检测
    pub enable_deadlock_detection: bool,
    /// 调度器类型
    pub scheduler_type: SchedulerType,
    /// 系统时钟频率（Hz）
    pub system_clock_hz: u32,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            max_tasks: 16,
            time_slice_us: 1000,
            enable_profiling: true,
            enable_deadlock_detection: false,
            scheduler_type: SchedulerType::RateMonotonic,
            system_clock_hz: 84_000_000,
        }
    }
}

/// 调度器统计信息
#[derive(Debug, Clone, Copy)]
pub struct SchedulerStatistics {
    /// 总的上下文切换次数
    pub context_switches: u32,
    /// 任务调度次数
    pub scheduling_decisions: u32,
    /// 截止期错失次数
    pub deadline_misses: u32,
    /// 平均响应时间（微秒）
    pub average_response_time_us: u32,
    /// 最大响应时间（微秒）
    pub max_response_time_us: u32,
    /// CPU利用率（0.0-1.0）
    pub cpu_utilization: f32,
    /// 调度开销（微秒）
    pub scheduling_overhead_us: u32,
}

impl SchedulerStatistics {
    /// 创建新的统计信息
    pub const fn new() -> Self {
        Self {
            context_switches: 0,
            scheduling_decisions: 0,
            deadline_misses: 0,
            average_response_time_us: 0,
            max_response_time_us: 0,
            cpu_utilization: 0.0,
            scheduling_overhead_us: 0,
        }
    }
    
    /// 计算调度效率
    pub fn scheduling_efficiency(&self) -> f32 {
        if self.scheduling_decisions == 0 {
            1.0
        } else {
            1.0 - (self.deadline_misses as f32 / self.scheduling_decisions as f32)
        }
    }
    
    /// 检查实时性能是否满足要求
    pub fn meets_realtime_requirements(&self, max_deadline_miss_ratio: f32) -> bool {
        let miss_ratio = if self.scheduling_decisions == 0 {
            0.0
        } else {
            self.deadline_misses as f32 / self.scheduling_decisions as f32
        };
        
        miss_ratio <= max_deadline_miss_ratio
    }
}

/// 调度器特征
pub trait Scheduler {
    /// 添加任务
    fn add_task(&mut self, task_config: TaskConfig) -> SchedulerResult<u8>;
    
    /// 移除任务
    fn remove_task(&mut self, task_id: u8) -> SchedulerResult<()>;
    
    /// 执行调度决策
    fn schedule(&mut self) -> SchedulerResult<Option<u8>>;
    
    /// 检查任务集的可调度性
    fn is_schedulable(&self) -> bool;
    
    /// 启动调度器
    fn start(&mut self) -> SchedulerResult<()>;
    
    /// 停止调度器
    fn stop(&mut self) -> SchedulerResult<()>;
    
    /// 获取统计信息
    fn statistics(&self) -> &SchedulerStatistics;
    
    /// 重置统计信息
    fn reset_statistics(&mut self);
}

/// 时间单位转换工具
pub struct TimeUtils;

impl TimeUtils {
    /// 毫秒转微秒
    pub const fn ms_to_us(ms: u32) -> u32 {
        ms * 1000
    }
    
    /// 微秒转毫秒
    pub const fn us_to_ms(us: u32) -> u32 {
        us / 1000
    }
    
    /// 秒转微秒
    pub const fn s_to_us(s: u32) -> u32 {
        s * 1_000_000
    }
    
    /// 微秒转秒
    pub const fn us_to_s(us: u32) -> u32 {
        us / 1_000_000
    }
    
    /// 时钟周期转微秒
    pub fn cycles_to_us(cycles: u32, clock_hz: u32) -> u32 {
        if clock_hz == 0 {
            0
        } else {
            (cycles as u64 * 1_000_000 / clock_hz as u64) as u32
        }
    }
    
    /// 微秒转时钟周期
    pub fn us_to_cycles(us: u32, clock_hz: u32) -> u32 {
        (us as u64 * clock_hz as u64 / 1_000_000) as u32
    }
}

/// 优先级工具
pub struct PriorityUtils;

impl PriorityUtils {
    /// 计算速率单调优先级（周期越短优先级越高）
    pub fn rate_monotonic_priority(period: u32) -> TaskPriority {
        // 使用周期的倒数作为优先级基础
        // 周期越短，优先级数值越大
        if period == 0 {
            TaskPriority::new(255) // 最高优先级
        } else {
            let priority = core::cmp::min(255, 1_000_000 / period);
            TaskPriority::new(priority as u8)
        }
    }
    
    /// 计算截止期单调优先级（截止期越短优先级越高）
    pub fn deadline_monotonic_priority(deadline: u32) -> TaskPriority {
        if deadline == 0 {
            TaskPriority::new(255)
        } else {
            let priority = core::cmp::min(255, 1_000_000 / deadline);
            TaskPriority::new(priority as u8)
        }
    }
    
    /// 比较两个优先级
    pub fn compare_priority(a: TaskPriority, b: TaskPriority) -> core::cmp::Ordering {
        a.value().cmp(&b.value())
    }
}

/// 全局调度器实例管理
pub struct GlobalScheduler {
    scheduler: Option<Box<dyn Scheduler>>,
    config: SchedulerConfig,
}

impl GlobalScheduler {
    /// 创建新的全局调度器
    pub const fn new() -> Self {
        Self {
            scheduler: None,
            config: SchedulerConfig {
                max_tasks: 16,
                time_slice_us: 1000,
                enable_profiling: true,
                enable_deadlock_detection: false,
                scheduler_type: SchedulerType::RateMonotonic,
                system_clock_hz: 84_000_000,
            },
        }
    }
    
    /// 初始化全局调度器
    pub fn init(&mut self, config: SchedulerConfig) -> SchedulerResult<()> {
        self.config = config;
        
        // 根据配置创建相应的调度器
        match config.scheduler_type {
            SchedulerType::RateMonotonic => {
                self.scheduler = Some(Box::new(RateMonotonicScheduler::new(config.max_tasks)?));
            }
            SchedulerType::EarliestDeadlineFirst => {
                self.scheduler = Some(Box::new(EdfScheduler::new(config.max_tasks)?));
            }
            SchedulerType::DeadlineMonotonic => {
                self.scheduler = Some(Box::new(DeadlineMonotonicScheduler::new(config.max_tasks)?));
            }
            _ => {
                return Err(SchedulerError::SchedulingFailed);
            }
        }
        
        Ok(())
    }
    
    /// 获取调度器实例
    pub fn instance(&mut self) -> SchedulerResult<&mut dyn Scheduler> {
        self.scheduler.as_mut()
            .map(|s| s.as_mut())
            .ok_or(SchedulerError::SchedulingFailed)
    }
    
    /// 获取配置
    pub fn config(&self) -> &SchedulerConfig {
        &self.config
    }
}

// 静态全局调度器实例
static mut GLOBAL_SCHEDULER: GlobalScheduler = GlobalScheduler::new();

/// 初始化全局调度器
pub fn init_global_scheduler(config: SchedulerConfig) -> SchedulerResult<()> {
    unsafe {
        GLOBAL_SCHEDULER.init(config)
    }
}

/// 获取全局调度器实例
pub fn global_scheduler() -> SchedulerResult<&'static mut dyn Scheduler> {
    unsafe {
        GLOBAL_SCHEDULER.instance()
    }
}

/// 便利宏：添加任务
#[macro_export]
macro_rules! add_task {
    ($config:expr) => {
        $crate::global_scheduler()?.add_task($config)
    };
}

/// 便利宏：执行调度
#[macro_export]
macro_rules! schedule {
    () => {
        $crate::global_scheduler()?.schedule()
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_utils() {
        assert_eq!(TimeUtils::ms_to_us(1), 1000);
        assert_eq!(TimeUtils::us_to_ms(1000), 1);
        assert_eq!(TimeUtils::s_to_us(1), 1_000_000);
        assert_eq!(TimeUtils::us_to_s(1_000_000), 1);
        
        assert_eq!(TimeUtils::cycles_to_us(84_000, 84_000_000), 1000);
        assert_eq!(TimeUtils::us_to_cycles(1000, 84_000_000), 84_000);
    }

    #[test]
    fn test_priority_utils() {
        let p1 = PriorityUtils::rate_monotonic_priority(100);
        let p2 = PriorityUtils::rate_monotonic_priority(200);
        
        // 周期越短优先级越高
        assert!(p1.value() > p2.value());
    }

    #[test]
    fn test_scheduler_statistics() {
        let mut stats = SchedulerStatistics::new();
        stats.scheduling_decisions = 100;
        stats.deadline_misses = 5;
        
        assert_eq!(stats.scheduling_efficiency(), 0.95);
        assert!(stats.meets_realtime_requirements(0.1));
        assert!(!stats.meets_realtime_requirements(0.01));
    }
}