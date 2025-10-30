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
        // 优化的优先级计算：使用对数映射实现更精细的优先级分配
        // 避免使用分支，提高执行效率
        const MAX_PRIORITY: u8 = 255;
        const MIN_PERIOD: u32 = 1;
        const MAX_PERIOD: u32 = 10_000_000; // 10秒
        
        // 确保周期在有效范围内
        let clamped_period = core::cmp::max(MIN_PERIOD, core::cmp::min(period, MAX_PERIOD));
        
        // 使用分段线性映射，避免浮点数运算
        let priority: u8 = if clamped_period <= 100 {
            // 1-100: 最高优先级范围 255-200
            MAX_PRIORITY - (clamped_period * 55 / 100) as u8
        } else if clamped_period <= 1000 {
            // 101-1000: 中高优先级 199-140
            200 - ((clamped_period - 100) * 60 / 900) as u8
        } else if clamped_period <= 10_000 {
            // 1001-10000: 中低优先级 139-80
            140 - ((clamped_period - 1000) * 60 / 9000) as u8
        } else {
            // 10001及以上: 低优先级 79-0
            80 - ((clamped_period - 10_000) * 80 / 9_990_000) as u8
        };
        
        TaskPriority::new(priority)
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
        
        // 检查调度器是否运行
        if !inner.running {
            return Ok(None);
        }
        
        // 步骤1: 完全隔离任务状态更新和统计信息
        let mut waiting_tasks = Vec::new();
        let mut new_deadline_misses = 0;
        
        // 仅更新任务状态，不访问统计信息
        for (i, task) in inner.tasks.iter_mut().enumerate() {
            if let Some(task) = task {
                // 检查任务是否准备好运行
                if task.state == TaskState::Ready && current_time >= task.next_run_time {
                    task.state = TaskState::Waiting;
                    waiting_tasks.push((i as u8, task.priority));
                }
                
                // 检查截止期是否被错过
                if task.deadline > 0 && current_time > task.next_run_time + task.deadline {
                    new_deadline_misses += 1;
                }
            }
        }
        
        // 步骤2: 更新统计信息（任务借用已释放）
        inner.statistics.deadline_misses += new_deadline_misses;
        inner.statistics.scheduling_decisions += 1;
        
        // 步骤3: 查找最高优先级任务
        let mut selected_task_id: Option<u8> = None;
        let mut highest_priority = TaskPriority::MIN;
        
        for (task_id, priority) in waiting_tasks {
            if priority > highest_priority {
                highest_priority = priority;
                selected_task_id = Some(task_id);
            }
        }
        
        // 步骤4: 执行选中的任务
        if let Some(task_id) = selected_task_id {
            let task_index = task_id as usize;
            
            // 存储需要更新的统计信息
            let mut task_statistics_update = None;
            
            // 完全隔离任务执行和统计信息更新
            {
                if let Some(task) = &mut inner.tasks[task_index] {
                    // 保存任务属性的副本
                    let task_period = task.period;
                    let task_deadline = task.deadline;
                    let old_run_count = task.run_count;
                    
                    // 更新任务状态
                    task.state = TaskState::Running;
                    task.last_run_time = current_time;
                    
                    // 运行任务
                    task.run();
                    
                    // 计算响应时间（在任务可变引用作用域内）
                    let response_time = if task_period > 0 {
                        current_time - task.last_run_time
                    } else {
                        0
                    };
                    
                    // 保存统计信息更新数据
                    if task_period > 0 {
                        task_statistics_update = Some(TaskStatisticsUpdate {
                            response_time,
                            old_run_count,
                            task_period,
                            task_deadline,
                            last_run_time: task.last_run_time,
                            current_time,
                        });
                    }
                    
                    // 更新下次运行时间并设置任务状态
                    if task_period > 0 {
                        task.next_run_time = current_time + task_period;
                        task.state = TaskState::Ready;
                    } else {
                        task.state = TaskState::Completed;
                    }
                }
            }
            
            // 步骤5: 更新统计信息（在任务引用完全释放后）
            if let Some(update) = task_statistics_update {
                // 访问统计信息进行更新
                let stats = &mut inner.statistics;
                
                // 更新平均响应时间
                if update.old_run_count <= 10 || update.old_run_count % 10 == 0 {
                    if update.old_run_count <= 10 {
                        stats.average_response_time_us = 
                            (stats.average_response_time_us * update.old_run_count + update.response_time) / 
                            (update.old_run_count + 1);
                    } else {
                        stats.average_response_time_us = 
                            (stats.average_response_time_us * 9 + update.response_time) / 10;
                    }
                }
                
                // 更新最大响应时间
                if update.response_time > stats.max_response_time_us {
                    stats.max_response_time_us = update.response_time;
                }
                
                // 检查截止期
                let deadline = if update.task_deadline > 0 { 
                    update.task_deadline 
                } else { 
                    update.task_period 
                };
                if update.current_time - update.last_run_time > deadline {
                    stats.deadline_misses += 1;
                }
            }
            
            // 更新上下文切换
            if inner.current_task != Some(task_id) {
                inner.statistics.context_switches += 1;
                inner.current_task = Some(task_id);
            }
        } else {
            inner.current_task = None;
        }
        
        Ok(selected_task_id)
    }

    fn is_schedulable(&self) -> bool {
        // 速率单调调度的可调度性测试：CPU利用率 <= n*(2^(1/n)-1)
        let inner = self.inner.borrow();
        let active_tasks = inner.tasks.iter()
            .filter(|t| t.is_some() && t.as_ref().unwrap().state != TaskState::Terminated)
            .count() as u32;
        
        if active_tasks == 0 {
            return true;
        }
        
        let utilization = inner.statistics.cpu_utilization;
        
        // 根据任务数量计算精确的RM边界
        // 预先计算常见任务数的边界值，避免运行时浮点计算
        let bound = match active_tasks {
            1 => 1.0,              // 1*(2^(1/1)-1) = 1
            2 => 0.828,            // 2*(2^(1/2)-1) ≈ 0.828
            3 => 0.779,            // 3*(2^(1/3)-1) ≈ 0.779
            4 => 0.756,            // 4*(2^(1/4)-1) ≈ 0.756
            5 => 0.743,            // 5*(2^(1/5)-1) ≈ 0.743
            6 => 0.734,            // 6*(2^(1/6)-1) ≈ 0.734
            7 => 0.728,            // 7*(2^(1/7)-1) ≈ 0.728
            8 => 0.722,            // 8*(2^(1/8)-1) ≈ 0.722
            9 => 0.718,            // 9*(2^(1/9)-1) ≈ 0.718
            10 => 0.715,           // 10*(2^(1/10)-1) ≈ 0.715
            _ => 0.693,            // 对于更多任务，接近ln(2) ≈ 0.693
        };
        
        // 添加安全余量，提高系统稳定性
        utilization <= bound * 0.95
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

/// 任务统计信息更新结构体
struct TaskStatisticsUpdate {
    response_time: u32,
    old_run_count: u32,
    task_period: u32,
    task_deadline: u32,
    last_run_time: u32,
    current_time: u32,
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