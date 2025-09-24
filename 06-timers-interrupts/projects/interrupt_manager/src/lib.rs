#![no_std]

//! # 中断管理器库
//! 
//! 这个库提供了一套完整的中断管理解决方案，包括：
//! - 中断优先级管理
//! - 中断性能分析
//! - 嵌套中断处理
//! - 中断调度器
//! 
//! ## 主要特性
//! 
//! - **优先级管理**：动态调整中断优先级
//! - **性能分析**：实时监控中断执行时间和频率
//! - **嵌套处理**：安全的嵌套中断管理
//! - **调度器**：基于优先级的中断任务调度
//! - **统计分析**：详细的中断统计信息
//! 
//! ## 使用示例
//! 
//! ```rust,no_run
//! use interrupt_manager::{InterruptManager, Priority, InterruptId};
//! 
//! let mut manager = InterruptManager::new();
//! manager.set_priority(InterruptId::Timer2, Priority::High);
//! manager.enable_interrupt(InterruptId::Timer2);
//! ```

use heapless::{Vec, FnvIndexMap};
use core::sync::atomic::{AtomicU32, AtomicU16, AtomicBool, Ordering};
use cortex_m::interrupt;
use micromath::F32Ext;

/// 中断ID枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InterruptId {
    Timer2 = 28,
    Timer3 = 29,
    Timer4 = 30,
    Timer5 = 50,
    Usart1 = 37,
    Usart2 = 38,
    Usart3 = 39,
    Spi1 = 35,
    Spi2 = 36,
    I2c1Ev = 31,
    I2c1Er = 32,
    I2c2Ev = 33,
    I2c2Er = 34,
    Adc = 18,
    Dma1Stream0 = 11,
    Dma1Stream1 = 12,
    Dma1Stream2 = 13,
    Dma1Stream3 = 14,
    Dma1Stream4 = 15,
    Dma1Stream5 = 16,
    Dma1Stream6 = 17,
    Exti0 = 6,
    Exti1 = 7,
    Exti2 = 8,
    Exti3 = 9,
    Exti4 = 10,
    Exti9To5 = 23,
    Exti15To10 = 40,
}

impl InterruptId {
    /// 获取中断号
    pub fn irq_number(self) -> u8 {
        self as u8
    }
    
    /// 从中断号创建InterruptId
    pub fn from_irq_number(irq: u8) -> Option<Self> {
        match irq {
            28 => Some(Self::Timer2),
            29 => Some(Self::Timer3),
            30 => Some(Self::Timer4),
            50 => Some(Self::Timer5),
            37 => Some(Self::Usart1),
            38 => Some(Self::Usart2),
            39 => Some(Self::Usart3),
            35 => Some(Self::Spi1),
            36 => Some(Self::Spi2),
            31 => Some(Self::I2c1Ev),
            32 => Some(Self::I2c1Er),
            33 => Some(Self::I2c2Ev),
            34 => Some(Self::I2c2Er),
            18 => Some(Self::Adc),
            11 => Some(Self::Dma1Stream0),
            12 => Some(Self::Dma1Stream1),
            13 => Some(Self::Dma1Stream2),
            14 => Some(Self::Dma1Stream3),
            15 => Some(Self::Dma1Stream4),
            16 => Some(Self::Dma1Stream5),
            17 => Some(Self::Dma1Stream6),
            6 => Some(Self::Exti0),
            7 => Some(Self::Exti1),
            8 => Some(Self::Exti2),
            9 => Some(Self::Exti3),
            10 => Some(Self::Exti4),
            23 => Some(Self::Exti9To5),
            40 => Some(Self::Exti15To10),
            _ => None,
        }
    }
}

/// 中断优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    Highest = 0,
    High = 1,
    Medium = 2,
    Low = 3,
    Lowest = 4,
}

impl Priority {
    /// 转换为NVIC优先级值
    pub fn to_nvic_priority(self) -> u8 {
        (self as u8) << 4 // STM32F4使用高4位
    }
    
    /// 从NVIC优先级值创建Priority
    pub fn from_nvic_priority(nvic_priority: u8) -> Self {
        match nvic_priority >> 4 {
            0 => Self::Highest,
            1 => Self::High,
            2 => Self::Medium,
            3 => Self::Low,
            _ => Self::Lowest,
        }
    }
}

/// 中断状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptState {
    Disabled,
    Enabled,
    Pending,
    Active,
}

/// 中断统计信息
#[derive(Debug, Clone, Copy)]
pub struct InterruptStats {
    pub count: u32,
    pub total_execution_time_us: u32,
    pub max_execution_time_us: u32,
    pub min_execution_time_us: u32,
    pub avg_execution_time_us: f32,
    pub frequency_hz: f32,
    pub last_execution_time: u32,
    pub nested_count: u16,
    pub max_nested_level: u8,
}

impl Default for InterruptStats {
    fn default() -> Self {
        Self {
            count: 0,
            total_execution_time_us: 0,
            max_execution_time_us: 0,
            min_execution_time_us: u32::MAX,
            avg_execution_time_us: 0.0,
            frequency_hz: 0.0,
            last_execution_time: 0,
            nested_count: 0,
            max_nested_level: 0,
        }
    }
}

impl InterruptStats {
    /// 更新统计信息
    pub fn update(&mut self, execution_time_us: u32, current_time: u32, nested_level: u8) {
        self.count += 1;
        self.total_execution_time_us += execution_time_us;
        
        if execution_time_us > self.max_execution_time_us {
            self.max_execution_time_us = execution_time_us;
        }
        
        if execution_time_us < self.min_execution_time_us {
            self.min_execution_time_us = execution_time_us;
        }
        
        self.avg_execution_time_us = self.total_execution_time_us as f32 / self.count as f32;
        
        if self.last_execution_time > 0 {
            let period_us = current_time - self.last_execution_time;
            if period_us > 0 {
                self.frequency_hz = 1_000_000.0 / period_us as f32;
            }
        }
        
        self.last_execution_time = current_time;
        
        if nested_level > 0 {
            self.nested_count += 1;
            if nested_level > self.max_nested_level {
                self.max_nested_level = nested_level;
            }
        }
    }
    
    /// 重置统计信息
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// 中断信息
#[derive(Debug)]
pub struct InterruptInfo {
    pub id: InterruptId,
    pub priority: Priority,
    pub state: InterruptState,
    pub stats: InterruptStats,
    pub enabled: bool,
}

impl InterruptInfo {
    pub fn new(id: InterruptId, priority: Priority) -> Self {
        Self {
            id,
            priority,
            state: InterruptState::Disabled,
            stats: InterruptStats::default(),
            enabled: false,
        }
    }
}

/// 中断管理器
pub struct InterruptManager {
    interrupts: FnvIndexMap<InterruptId, InterruptInfo, 32>,
    system_time_us: AtomicU32,
    nested_level: AtomicU16,
    profiling_enabled: AtomicBool,
}

impl InterruptManager {
    /// 创建新的中断管理器
    pub fn new() -> Self {
        Self {
            interrupts: FnvIndexMap::new(),
            system_time_us: AtomicU32::new(0),
            nested_level: AtomicU16::new(0),
            profiling_enabled: AtomicBool::new(true),
        }
    }
    
    /// 注册中断
    pub fn register_interrupt(&mut self, id: InterruptId, priority: Priority) -> Result<(), InterruptError> {
        let info = InterruptInfo::new(id, priority);
        self.interrupts.insert(id, info)
            .map_err(|_| InterruptError::RegistrationFailed)?;
        Ok(())
    }
    
    /// 设置中断优先级
    pub fn set_priority(&mut self, id: InterruptId, priority: Priority) -> Result<(), InterruptError> {
        if let Some(info) = self.interrupts.get_mut(&id) {
            info.priority = priority;
            
            // 更新NVIC优先级
            unsafe {
                let nvic = &*cortex_m::peripheral::NVIC::PTR;
                nvic.set_priority(
                    cortex_m::peripheral::interrupt::InterruptNumber::from(id.irq_number()),
                    priority.to_nvic_priority()
                );
            }
            
            Ok(())
        } else {
            Err(InterruptError::InterruptNotFound)
        }
    }
    
    /// 启用中断
    pub fn enable_interrupt(&mut self, id: InterruptId) -> Result<(), InterruptError> {
        if let Some(info) = self.interrupts.get_mut(&id) {
            info.enabled = true;
            info.state = InterruptState::Enabled;
            
            // 启用NVIC中断
            unsafe {
                cortex_m::peripheral::NVIC::unmask(
                    cortex_m::peripheral::interrupt::InterruptNumber::from(id.irq_number())
                );
            }
            
            Ok(())
        } else {
            Err(InterruptError::InterruptNotFound)
        }
    }
    
    /// 禁用中断
    pub fn disable_interrupt(&mut self, id: InterruptId) -> Result<(), InterruptError> {
        if let Some(info) = self.interrupts.get_mut(&id) {
            info.enabled = false;
            info.state = InterruptState::Disabled;
            
            // 禁用NVIC中断
            cortex_m::peripheral::NVIC::mask(
                cortex_m::peripheral::interrupt::InterruptNumber::from(id.irq_number())
            );
            
            Ok(())
        } else {
            Err(InterruptError::InterruptNotFound)
        }
    }
    
    /// 获取中断信息
    pub fn get_interrupt_info(&self, id: InterruptId) -> Option<&InterruptInfo> {
        self.interrupts.get(&id)
    }
    
    /// 获取所有中断信息
    pub fn get_all_interrupts(&self) -> impl Iterator<Item = &InterruptInfo> {
        self.interrupts.values()
    }
    
    /// 开始中断性能分析
    pub fn start_interrupt_profiling(&self, id: InterruptId) -> u32 {
        if self.profiling_enabled.load(Ordering::Relaxed) {
            self.nested_level.fetch_add(1, Ordering::Relaxed);
            self.get_system_time_us()
        } else {
            0
        }
    }
    
    /// 结束中断性能分析
    pub fn end_interrupt_profiling(&mut self, id: InterruptId, start_time: u32) {
        if self.profiling_enabled.load(Ordering::Relaxed) && start_time > 0 {
            let end_time = self.get_system_time_us();
            let execution_time = end_time.wrapping_sub(start_time);
            let nested_level = self.nested_level.fetch_sub(1, Ordering::Relaxed) as u8;
            
            if let Some(info) = self.interrupts.get_mut(&id) {
                info.stats.update(execution_time, end_time, nested_level);
            }
        }
    }
    
    /// 启用/禁用性能分析
    pub fn set_profiling_enabled(&self, enabled: bool) {
        self.profiling_enabled.store(enabled, Ordering::Relaxed);
    }
    
    /// 获取系统时间（微秒）
    pub fn get_system_time_us(&self) -> u32 {
        self.system_time_us.load(Ordering::Relaxed)
    }
    
    /// 更新系统时间
    pub fn update_system_time(&self, time_us: u32) {
        self.system_time_us.store(time_us, Ordering::Relaxed);
    }
    
    /// 重置所有统计信息
    pub fn reset_all_stats(&mut self) {
        for info in self.interrupts.values_mut() {
            info.stats.reset();
        }
    }
    
    /// 获取嵌套级别
    pub fn get_nested_level(&self) -> u16 {
        self.nested_level.load(Ordering::Relaxed)
    }
}

/// 中断错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptError {
    InterruptNotFound,
    RegistrationFailed,
    InvalidPriority,
    AlreadyEnabled,
    AlreadyDisabled,
    NestedTooDeep,
}

/// 中断调度器
pub struct InterruptScheduler {
    task_queue: Vec<ScheduledTask, 32>,
    current_priority: Priority,
    scheduler_enabled: bool,
}

impl InterruptScheduler {
    pub fn new() -> Self {
        Self {
            task_queue: Vec::new(),
            current_priority: Priority::Lowest,
            scheduler_enabled: true,
        }
    }
    
    /// 添加调度任务
    pub fn schedule_task(&mut self, task: ScheduledTask) -> Result<(), InterruptError> {
        self.task_queue.push(task)
            .map_err(|_| InterruptError::RegistrationFailed)?;
        
        // 按优先级排序
        self.task_queue.sort_by(|a, b| a.priority.cmp(&b.priority));
        
        Ok(())
    }
    
    /// 执行下一个任务
    pub fn execute_next_task(&mut self) -> Option<ScheduledTask> {
        if self.scheduler_enabled && !self.task_queue.is_empty() {
            Some(self.task_queue.remove(0))
        } else {
            None
        }
    }
    
    /// 设置调度器状态
    pub fn set_enabled(&mut self, enabled: bool) {
        self.scheduler_enabled = enabled;
    }
    
    /// 获取队列长度
    pub fn queue_length(&self) -> usize {
        self.task_queue.len()
    }
    
    /// 清空任务队列
    pub fn clear_queue(&mut self) {
        self.task_queue.clear();
    }
}

/// 调度任务
#[derive(Debug, Clone, Copy)]
pub struct ScheduledTask {
    pub id: u32,
    pub priority: Priority,
    pub interrupt_id: InterruptId,
    pub delay_us: u32,
    pub repeat: bool,
}

impl ScheduledTask {
    pub fn new(id: u32, priority: Priority, interrupt_id: InterruptId) -> Self {
        Self {
            id,
            priority,
            interrupt_id,
            delay_us: 0,
            repeat: false,
        }
    }
    
    pub fn with_delay(mut self, delay_us: u32) -> Self {
        self.delay_us = delay_us;
        self
    }
    
    pub fn with_repeat(mut self, repeat: bool) -> Self {
        self.repeat = repeat;
        self
    }
}

/// 中断性能分析器
pub struct InterruptProfiler {
    samples: Vec<ProfileSample, 1024>,
    sampling_enabled: bool,
    sample_interval_us: u32,
    last_sample_time: u32,
}

impl InterruptProfiler {
    pub fn new(sample_interval_us: u32) -> Self {
        Self {
            samples: Vec::new(),
            sampling_enabled: false,
            sample_interval_us,
            last_sample_time: 0,
        }
    }
    
    /// 开始采样
    pub fn start_sampling(&mut self) {
        self.sampling_enabled = true;
        self.samples.clear();
    }
    
    /// 停止采样
    pub fn stop_sampling(&mut self) {
        self.sampling_enabled = false;
    }
    
    /// 添加性能样本
    pub fn add_sample(&mut self, sample: ProfileSample) -> Result<(), InterruptError> {
        if self.sampling_enabled {
            self.samples.push(sample)
                .map_err(|_| InterruptError::RegistrationFailed)?;
        }
        Ok(())
    }
    
    /// 获取性能报告
    pub fn get_performance_report(&self) -> PerformanceReport {
        let mut report = PerformanceReport::default();
        
        if self.samples.is_empty() {
            return report;
        }
        
        let mut total_execution_time = 0u32;
        let mut max_execution_time = 0u32;
        let mut min_execution_time = u32::MAX;
        
        for sample in &self.samples {
            total_execution_time += sample.execution_time_us;
            
            if sample.execution_time_us > max_execution_time {
                max_execution_time = sample.execution_time_us;
            }
            
            if sample.execution_time_us < min_execution_time {
                min_execution_time = sample.execution_time_us;
            }
        }
        
        report.sample_count = self.samples.len() as u32;
        report.total_execution_time_us = total_execution_time;
        report.avg_execution_time_us = total_execution_time as f32 / self.samples.len() as f32;
        report.max_execution_time_us = max_execution_time;
        report.min_execution_time_us = min_execution_time;
        
        report
    }
    
    /// 清空样本
    pub fn clear_samples(&mut self) {
        self.samples.clear();
    }
}

/// 性能样本
#[derive(Debug, Clone, Copy)]
pub struct ProfileSample {
    pub timestamp: u32,
    pub interrupt_id: InterruptId,
    pub execution_time_us: u32,
    pub nested_level: u8,
}

/// 性能报告
#[derive(Debug, Default)]
pub struct PerformanceReport {
    pub sample_count: u32,
    pub total_execution_time_us: u32,
    pub avg_execution_time_us: f32,
    pub max_execution_time_us: u32,
    pub min_execution_time_us: u32,
    pub cpu_utilization_percent: f32,
}

/// 中断上下文管理器
pub struct InterruptContext {
    saved_priority: u8,
    nested_count: u8,
}

impl InterruptContext {
    /// 进入中断上下文
    pub fn enter() -> Self {
        let saved_priority = interrupt::free(|_| {
            // 保存当前优先级
            0 // 简化实现
        });
        
        Self {
            saved_priority,
            nested_count: 1,
        }
    }
    
    /// 退出中断上下文
    pub fn exit(self) {
        interrupt::free(|_| {
            // 恢复优先级
        });
    }
}

/// 全局中断管理器实例
static mut GLOBAL_INTERRUPT_MANAGER: Option<InterruptManager> = None;

/// 初始化全局中断管理器
pub fn init_global_interrupt_manager() {
    unsafe {
        GLOBAL_INTERRUPT_MANAGER = Some(InterruptManager::new());
    }
}

/// 获取全局中断管理器
pub fn get_global_interrupt_manager() -> Option<&'static mut InterruptManager> {
    unsafe { GLOBAL_INTERRUPT_MANAGER.as_mut() }
}

/// 中断处理宏
#[macro_export]
macro_rules! interrupt_handler {
    ($interrupt_id:expr, $handler:block) => {
        {
            let start_time = if let Some(manager) = get_global_interrupt_manager() {
                manager.start_interrupt_profiling($interrupt_id)
            } else {
                0
            };
            
            // 执行中断处理代码
            $handler
            
            if let Some(manager) = get_global_interrupt_manager() {
                manager.end_interrupt_profiling($interrupt_id, start_time);
            }
        }
    };
}