//! Scheduler Trait Definition

use super::{SchedulerConfig, SchedulerError, SchedulerResult, SchedulerStatistics, TaskConfig};

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
    fn statistics(&self) -> SchedulerStatistics;
    
    /// 重置统计信息
    fn reset_statistics(&mut self);
}