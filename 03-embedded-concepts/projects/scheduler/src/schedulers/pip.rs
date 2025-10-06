//! 优先级继承协议调度器实现

use super::SchedulerImpl;
use crate::{Scheduler, SchedulerConfig, SchedulerError, SchedulerResult, SchedulerStatistics};
use crate::{Task, TaskConfig, TaskPriority, TaskState};
use alloc::vec::Vec;
use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

/// 互斥锁实现，支持优先级继承
#[derive(Debug, Default)]
pub struct Mutex {
    /// 是否被锁定
    locked: bool,
    /// 当前持有锁的任务ID
    owner: Option<u8>,
    /// 原始优先级
    original_priority: Option<TaskPriority>,
}

impl Mutex {
    /// 创建新的互斥锁
    pub fn new() -> Self {
        Self {
            locked: false,
            owner: None,
            original_priority: None,
        }
    }

    /// 尝试获取锁
    pub fn lock(&mut self, task_id: u8, task_priority: TaskPriority) -> bool {
        if !self.locked {
            // 锁未被占用，直接获取
            self.locked = true;
            self.owner = Some(task_id);
            return true;
        }
        false
    }

    /// 释放锁
    pub fn unlock(&mut self) -> bool {
        if self.locked {
            self.locked = false;
            self.owner = None;
            self.original_priority = None;
            return true;
        }
        false
    }

    /// 获取当前锁的状态
    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// 获取当前持有锁的任务ID
    pub fn owner(&self) -> Option<u8> {
        self.owner
    }

    /// 设置优先级继承信息
    pub fn set_priority_inheritance(&mut self, original_priority: TaskPriority) {
        self.original_priority = Some(original_priority);
    }

    /// 获取原始优先级
    pub fn original_priority(&self) -> Option<TaskPriority> {
        self.original_priority
    }
}

/// 优先级继承协议调度器
/// 
/// 解决优先级反转问题的调度算法
pub struct PipScheduler {
    /// 内部调度器实现
    inner: RefCell<SchedulerImpl>,
    /// 系统时钟计数器
    system_clock: AtomicU32,
    /// 互斥锁数组
    mutexes: RefCell<Vec<Mutex>>,
    /// 配置
    config: SchedulerConfig,
}

impl PipScheduler {
    /// 创建新的优先级继承协议调度器
    pub fn new(config: SchedulerConfig) -> SchedulerResult<Self> {
        let inner = SchedulerImpl::new(config.max_tasks)?;
        let mutexes = RefCell::new(Vec::new());
        
        Ok(Self {
            inner: RefCell::new(inner),
            system_clock: AtomicU32::new(0),
            mutexes,
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

    /// 创建新的互斥锁
    pub fn create_mutex(&self) -> u8 {
        let mut mutexes = self.mutexes.borrow_mut();
        let id = mutexes.len() as u8;
        mutexes.push(Mutex::new());
        id
    }

    /// 锁定互斥锁（带优先级继承）
    pub fn acquire_mutex(&mut self, mutex_id: u8, task_id: u8) -> SchedulerResult<bool> {
        let mut mutexes = self.mutexes.borrow_mut();
        let mut inner = self.inner.borrow_mut();
        
        // 检查互斥锁ID是否有效
        if mutex_id as usize >= mutexes.len() {
            return Err(SchedulerError::InvalidMutex);
        }
        
        // 检查任务ID是否有效
        if task_id as usize >= inner.tasks.len() {
            return Err(SchedulerError::InvalidTaskId);
        }
        
        let mutex = &mut mutexes[mutex_id as usize];
        let task = &mut inner.tasks[task_id as usize];
        
        if let Some(ref mut task) = task {
            // 尝试获取锁
            if mutex.lock(task_id, task.priority) {
                return Ok(true);
            }
            
            // 锁已被占用，检查是否需要优先级继承
            if let Some(owner_id) = mutex.owner() {
                let owner_task = &mut inner.tasks[owner_id as usize];
                
                if let Some(ref mut owner_task) = owner_task {
                    // 如果请求锁的任务优先级高于锁所有者的优先级，则进行优先级继承
                    if task.priority > owner_task.priority {
                        // 保存原始优先级
                        if mutex.original_priority().is_none() {
                            mutex.set_priority_inheritance(owner_task.priority);
                        }
                        
                        // 提升所有者优先级
                        owner_task.priority = task.priority;
                    }
                }
            }
        }
        
        Ok(false)
    }

    /// 释放互斥锁（恢复原始优先级）
    pub fn release_mutex(&mut self, mutex_id: u8) -> SchedulerResult<bool> {
        let mut mutexes = self.mutexes.borrow_mut();
        let mut inner = self.inner.borrow_mut();
        
        // 检查互斥锁ID是否有效
        if mutex_id as usize >= mutexes.len() {
            return Err(SchedulerError::InvalidMutex);
        }
        
        let mutex = &mut mutexes[mutex_id as usize];
        
        if mutex.is_locked() {
            // 获取原始优先级并恢复
            if let Some(original_priority) = mutex.original_priority() {
                if let Some(owner_id) = mutex.owner() {
                    if let Some(ref mut owner_task) = inner.tasks[owner_id as usize] {
                        owner_task.priority = original_priority;
                    }
                }
            }
            
            mutex.unlock();
            return Ok(true);
        }
        
        Ok(false)
    }
}

impl Scheduler for PipScheduler {
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
                    if task.deadline > 0 && current_time > task.next_run_time + task.deadline {
                        inner.statistics.deadline_misses += 1;
                    }
                }
            }
        }
        
        // 选择最高优先级的等待任务
        let mut highest_priority_task: Option<u8> = None;
        let mut highest_priority = TaskPriority::MIN;
        
        for (i, task) in inner.tasks.iter().enumerate() {
            if let Some(ref task) = task {
                if task.state == TaskState::Waiting {
                    if task.priority > highest_priority {
                        highest_priority = task.priority;
                        highest_priority_task = Some(i as u8);
                    }
                }
            }
        }
        
        // 更新统计信息
        inner.statistics.scheduling_decisions += 1;
        
        if let Some(task_id) = highest_priority_task {
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
        
        Ok(highest_priority_task)
    }

    fn is_schedulable(&self) -> bool {
        // 使用RM的可调度性条件：对于n个任务，Σ(Ci/Ti) <= n(2^(1/n)-1)
        let inner = self.inner.borrow();
        let n = inner.tasks.iter().filter(|t| t.is_some()).count() as f64;
        
        if n == 0 {
            return true;
        }
        
        let sum = inner.statistics.cpu_utilization;
        let bound = n * (2.0f64.powf(1.0 / n) - 1.0);
        
        sum <= bound
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