//! Hybrid Scheduler
//!
//! This module implements a Hybrid Scheduler, which combines characteristics of
//! Rate Monotonic Scheduling (RMS) and Earliest Deadline First (EDF) scheduling.
//! It aims to leverage the predictability of RMS for high-priority, periodic tasks
//! and the optimality of EDF for lower-priority, aperiodic tasks.

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use core::cmp::Ordering;

use crate::{
    Scheduler, SchedulerConfig, SchedulerError, SchedulerResult, SchedulerStatistics, Task, TaskId,
};

/// Represents the Hybrid Scheduler.
///
/// This scheduler manages tasks by categorizing them into high-priority (RMS)
/// and low-priority (EDF) groups, applying the respective scheduling policies.
pub struct HybridScheduler {
    /// Configuration for the scheduler.
    config: SchedulerConfig,
    /// Internal implementation details of the Hybrid Scheduler.
    inner: HybridSchedulerImpl,
}

impl HybridScheduler {
    /// Creates a new `HybridScheduler` instance.
    ///
    /// # Arguments
    ///
    /// * `config` - The configuration for the scheduler.
    ///
    /// # Returns
    ///
    /// A `SchedulerResult` containing the `HybridScheduler` instance or a `SchedulerError`.
    pub fn new(config: SchedulerConfig) -> SchedulerResult<Self> {
        Ok(Self {
            config,
            inner: HybridSchedulerImpl::new(),
        })
    }
}

impl Scheduler for HybridScheduler {
    /// Adds a task to the scheduler.
    ///
    /// # Arguments
    ///
    /// * `task` - The task to be added.
    ///
    /// # Returns
    ///
    /// A `SchedulerResult` indicating success or a `SchedulerError`.
    fn add_task(&mut self, task: Task) -> SchedulerResult<()> {
        self.inner.add_task(task)
    }

    /// Removes a task from the scheduler.
    ///
    /// # Arguments
    ///
    /// * `task_id` - The ID of the task to be removed.
    ///
    /// # Returns
    ///
    /// A `SchedulerResult` containing the removed `Task` or a `SchedulerError`.
    fn remove_task(&mut self, task_id: TaskId) -> SchedulerResult<Task> {
        self.inner.remove_task(task_id)
    }

    /// Schedules the next task to run.
    ///
    /// # Returns
    ///
    /// A `SchedulerResult` containing the ID of the next task to run or a `SchedulerError`.
    fn schedule(&mut self) -> SchedulerResult<TaskId> {
        self.inner.schedule()
    }

    /// Checks if the system is schedulable with the current set of tasks.
    ///
    /// # Returns
    ///
    /// `true` if the system is schedulable, `false` otherwise.
    fn is_schedulable(&self) -> bool {
        self.inner.is_schedulable()
    }

    /// Starts the scheduler.
    ///
    /// # Returns
    ///
    /// A `SchedulerResult` indicating success or a `SchedulerError`.
    fn start(&mut self) -> SchedulerResult<()> {
        self.inner.start()
    }

    /// Stops the scheduler.
    ///
    /// # Returns
    ///
    /// A `SchedulerResult` indicating success or a `SchedulerError`.
    fn stop(&mut self) -> SchedulerResult<()> {
        self.inner.stop()
    }

    /// Retrieves the current scheduler statistics.
    ///
    /// # Returns
    ///
    /// A `SchedulerStatistics` struct.
    fn statistics(&self) -> SchedulerStatistics {
        self.inner.statistics()
    }

    /// Resets the scheduler statistics.
    fn reset_statistics(&mut self) {
        self.inner.reset_statistics()
    }
}

/// Internal implementation of the Hybrid Scheduler.
struct HybridSchedulerImpl {
    /// List of all tasks managed by the scheduler.
    tasks: Vec<Task>,
    /// Queue of runnable tasks, ordered by priority/deadline.
    runnable_tasks: VecDeque<TaskId>,
    /// Statistics for the scheduler.
    statistics: SchedulerStatistics,
    /// Flag indicating if the scheduler is running.
    is_running: bool,
}

impl HybridSchedulerImpl {
    /// Creates a new `HybridSchedulerImpl` instance.
    fn new() -> Self {
        Self {
            tasks: Vec::new(),
            runnable_tasks: VecDeque::new(),
            statistics: SchedulerStatistics::new(),
            is_running: false,
        }
    }

    /// Adds a task to the scheduler's internal task list.
    fn add_task(&mut self, task: Task) -> SchedulerResult<()> {
        // TODO: Implement logic to categorize tasks into RMS/EDF groups
        // and add them to appropriate internal data structures.
        self.tasks.push(task);
        Ok(())
    }

    /// Removes a task from the scheduler's internal task list.
    fn remove_task(&mut self, task_id: TaskId) -> SchedulerResult<Task> {
        let index = self
            .tasks
            .iter()
            .position(|t| t.id == task_id)
            .ok_or(SchedulerError::TaskNotFound(task_id))?;
        let task = self.tasks.remove(index);
        // TODO: Remove task from runnable_tasks if present
        Ok(task)
    }

    /// Schedules the next task to run based on hybrid policy.
    fn schedule(&mut self) -> SchedulerResult<TaskId> {
        if !self.is_running {
            return Err(SchedulerError::SchedulerNotRunning);
        }

        // TODO: Implement hybrid scheduling logic (e.g., prioritize RMS tasks, then EDF tasks)
        // For now, a placeholder that just returns the first runnable task.
        self.runnable_tasks
            .pop_front()
            .ok_or(SchedulerError::NoRunnableTasks)
    }

    /// Checks if the system is schedulable.
    fn is_schedulable(&self) -> bool {
        // TODO: Implement schedulability analysis for hybrid scheduling
        true
    }

    /// Starts the scheduler.
    fn start(&mut self) -> SchedulerResult<()> {
        self.is_running = true;
        Ok(())
    }

    /// Stops the scheduler.
    fn stop(&mut self) -> SchedulerResult<()> {
        self.is_running = false;
        Ok(())
    }

    /// Retrieves the current scheduler statistics.
    fn statistics(&self) -> SchedulerStatistics {
        self.statistics.clone()
    }

    /// Resets the scheduler statistics.
    fn reset_statistics(&mut self) {
        self.statistics = SchedulerStatistics::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{TaskBuilder, TaskPriority};

    #[test]
    fn test_hybrid_scheduler_new() {
        let config = SchedulerConfig::default();
        let scheduler = HybridScheduler::new(config).unwrap();
        assert!(!scheduler.inner.is_running);
        assert!(scheduler.inner.tasks.is_empty());
    }

    #[test]
    fn test_hybrid_scheduler_add_task() {
        let config = SchedulerConfig::default();
        let mut scheduler = HybridScheduler::new(config).unwrap();
        let task = TaskBuilder::new()
            .id(1)
            .priority(TaskPriority::High)
            .build();
        scheduler.add_task(task).unwrap();
        assert_eq!(scheduler.inner.tasks.len(), 1);
    }

    #[test]
    fn test_hybrid_scheduler_remove_task() {
        let config = SchedulerConfig::default();
        let mut scheduler = HybridScheduler::new(config).unwrap();
        let task = TaskBuilder::new()
            .id(1)
            .priority(TaskPriority::High)
            .build();
        scheduler.add_task(task).unwrap();
        let removed_task = scheduler.remove_task(1).unwrap();
        assert_eq!(removed_task.id, 1);
        assert!(scheduler.inner.tasks.is_empty());
    }

    #[test]
    fn test_hybrid_scheduler_schedule_not_running() {
        let config = SchedulerConfig::default();
        let mut scheduler = HybridScheduler::new(config).unwrap();
        let result = scheduler.schedule();
        assert!(matches!(result, Err(SchedulerError::SchedulerNotRunning)));
    }

    #[test]
    fn test_hybrid_scheduler_start_stop() {
        let config = SchedulerConfig::default();
        let mut scheduler = HybridScheduler::new(config).unwrap();
        scheduler.start().unwrap();
        assert!(scheduler.inner.is_running);
        scheduler.stop().unwrap();
        assert!(!scheduler.inner.is_running);
    }

    #[test]
    fn test_hybrid_scheduler_statistics() {
        let config = SchedulerConfig::default();
        let scheduler = HybridScheduler::new(config).unwrap();
        let stats = scheduler.statistics();
        assert_eq!(stats.total_tasks, 0);
    }
}