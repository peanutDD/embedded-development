//! 同步原语模块

use crate::{SchedulerError, SchedulerResult};
use core::cell::RefCell;
use core::sync::atomic::{AtomicI32, Ordering};

/// 信号量实现
#[derive(Debug)]
pub struct Semaphore {
    /// 当前信号量值
    value: AtomicI32,
    /// 最大信号量值
    max_value: i32,
    /// 等待任务计数
    waiting_tasks: RefCell<usize>,
}

impl Semaphore {
    /// 创建新的信号量
    pub fn new(initial_value: i32, max_value: i32) -> SchedulerResult<Self> {
        if initial_value < 0 || max_value <= 0 || initial_value > max_value {
            return Err(SchedulerError::InvalidSemaphoreValue);
        }
        
        Ok(Self {
            value: AtomicI32::new(initial_value),
            max_value,
            waiting_tasks: RefCell::new(0),
        })
    }

    /// 获取信号量（非阻塞）
    pub fn try_acquire(&self) -> bool {
        let current = self.value.load(Ordering::Relaxed);
        if current > 0 {
            self.value.compare_exchange(current, current - 1, Ordering::Relaxed, Ordering::Relaxed).is_ok()
        } else {
            false
        }
    }

    /// 获取信号量（阻塞，这里实现为非真正阻塞，需要调度器配合）
    pub fn acquire(&self) -> bool {
        let mut result = self.try_acquire();
        if !result {
            *self.waiting_tasks.borrow_mut() += 1;
        }
        result
    }

    /// 释放信号量
    pub fn release(&self) -> SchedulerResult<()> {
        let current = self.value.load(Ordering::Relaxed);
        
        if current < self.max_value {
            self.value.compare_exchange(current, current + 1, Ordering::Relaxed, Ordering::Relaxed).map_err(|_| SchedulerError::SemaphoreReleaseFailed)?;
            
            // 如果有等待的任务，减少等待计数
            if *self.waiting_tasks.borrow() > 0 {
                *self.waiting_tasks.borrow_mut() -= 1;
            }
            
            Ok(())
        } else {
            Err(SchedulerError::SemaphoreValueExceeded)
        }
    }

    /// 获取当前信号量值
    pub fn get_value(&self) -> i32 {
        self.value.load(Ordering::Relaxed)
    }

    /// 获取等待任务数
    pub fn get_waiting_tasks(&self) -> usize {
        *self.waiting_tasks.borrow()
    }

    /// 重置信号量
    pub fn reset(&self, value: i32) -> SchedulerResult<()> {
        if value < 0 || value > self.max_value {
            return Err(SchedulerError::InvalidSemaphoreValue);
        }
        
        self.value.store(value, Ordering::Relaxed);
        *self.waiting_tasks.borrow_mut() = 0;
        
        Ok(())
    }
}

/// 计数信号量特殊实现（无最大限制）
pub struct CountingSemaphore {
    /// 内部信号量
    inner: Semaphore,
}

impl CountingSemaphore {
    /// 创建新的计数信号量
    pub fn new(initial_value: i32) -> SchedulerResult<Self> {
        if initial_value < 0 {
            return Err(SchedulerError::InvalidSemaphoreValue);
        }
        
        // 使用i32的最大值作为最大限制
        let inner = Semaphore::new(initial_value, i32::MAX)?;
        
        Ok(Self {
            inner,
        })
    }

    /// 获取信号量（非阻塞）
    pub fn try_acquire(&self) -> bool {
        self.inner.try_acquire()
    }

    /// 获取信号量（阻塞）
    pub fn acquire(&self) -> bool {
        self.inner.acquire()
    }

    /// 释放信号量
    pub fn release(&self) -> SchedulerResult<()> {
        self.inner.release()
    }

    /// 获取当前信号量值
    pub fn get_value(&self) -> i32 {
        self.inner.get_value()
    }
}

/// 二进制信号量特殊实现（值只能是0或1）
pub struct BinarySemaphore {
    /// 内部信号量
    inner: Semaphore,
}

impl BinarySemaphore {
    /// 创建新的二进制信号量
    pub fn new(initial_value: bool) -> SchedulerResult<Self> {
        let value = if initial_value { 1 } else { 0 };
        let inner = Semaphore::new(value, 1)?;
        
        Ok(Self {
            inner,
        })
    }

    /// 获取信号量（非阻塞）
    pub fn try_acquire(&self) -> bool {
        self.inner.try_acquire()
    }

    /// 获取信号量（阻塞）
    pub fn acquire(&self) -> bool {
        self.inner.acquire()
    }

    /// 释放信号量
    pub fn release(&self) -> SchedulerResult<()> {
        self.inner.release()
    }

    /// 获取当前信号量值
    pub fn get_value(&self) -> bool {
        self.inner.get_value() > 0
    }
}

/// 事件标志组实现
#[derive(Debug)]
pub struct EventGroup {
    /// 事件标志值
    flags: AtomicI32,
    /// 等待任务计数
    waiting_tasks: RefCell<usize>,
}

impl EventGroup {
    /// 创建新的事件标志组
    pub const fn new() -> Self {
        Self {
            flags: AtomicI32::new(0),
            waiting_tasks: RefCell::new(0),
        }
    }

    /// 设置指定的事件标志
    pub fn set(&self, flags: i32) {
        self.flags.fetch_or(flags, Ordering::Relaxed);
    }

    /// 清除指定的事件标志
    pub fn clear(&self, flags: i32) {
        self.flags.fetch_and(!flags, Ordering::Relaxed);
    }

    /// 等待指定的事件标志（任意一个）
    pub fn wait_any(&self, flags: i32) -> bool {
        let current = self.flags.load(Ordering::Relaxed);
        let result = (current & flags) != 0;
        
        if !result {
            *self.waiting_tasks.borrow_mut() += 1;
        }
        
        result
    }

    /// 等待指定的事件标志（全部）
    pub fn wait_all(&self, flags: i32) -> bool {
        let current = self.flags.load(Ordering::Relaxed);
        let result = (current & flags) == flags;
        
        if !result {
            *self.waiting_tasks.borrow_mut() += 1;
        }
        
        result
    }

    /// 尝试等待指定的事件标志（任意一个，非阻塞）
    pub fn try_wait_any(&self, flags: i32) -> bool {
        (self.flags.load(Ordering::Relaxed) & flags) != 0
    }

    /// 尝试等待指定的事件标志（全部，非阻塞）
    pub fn try_wait_all(&self, flags: i32) -> bool {
        (self.flags.load(Ordering::Relaxed) & flags) == flags
    }

    /// 获取当前事件标志值
    pub fn get(&self) -> i32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 重置事件标志组
    pub fn reset(&self) {
        self.flags.store(0, Ordering::Relaxed);
        *self.waiting_tasks.borrow_mut() = 0;
    }
}