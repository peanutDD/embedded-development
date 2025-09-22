#![no_std]

//! # 中断驱动编程库
//! 
//! 本库提供了完整的中断驱动编程框架，包括：
//! - 事件驱动架构
//! - 异步中断处理
//! - 状态机管理
//! - 缓冲区管理
//! - 优先级调度

use embedded_hal::digital::{InputPin, OutputPin};
use heapless::{Vec, Deque};
use critical_section::Mutex;
use core::cell::RefCell;
use smlang::statemachine;

/// 中断驱动系统特征
pub trait InterruptDriven {
    type Event;
    type Error;
    
    /// 处理中断事件
    fn handle_interrupt(&mut self, event: Self::Event) -> Result<(), Self::Error>;
    
    /// 获取中断优先级
    fn get_priority(&self) -> u8;
    
    /// 启用中断
    fn enable_interrupt(&mut self);
    
    /// 禁用中断
    fn disable_interrupt(&mut self);
}

/// 事件类型定义
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemEvent {
    /// GPIO中断事件
    GpioInterrupt { pin: u8, edge: EdgeType },
    /// 定时器中断事件
    TimerInterrupt { timer_id: u8 },
    /// 串口数据接收
    UartReceive { data: u8 },
    /// ADC转换完成
    AdcComplete { channel: u8, value: u16 },
    /// 用户自定义事件
    UserEvent { id: u16, data: u32 },
}

/// 边沿触发类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeType {
    Rising,
    Falling,
    Both,
}

/// 事件优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    Critical = 0,
    High = 1,
    Medium = 2,
    Low = 3,
}

/// 事件处理器
pub struct EventProcessor<const N: usize> {
    /// 事件队列
    event_queue: Mutex<RefCell<Deque<(SystemEvent, Priority), N>>>,
    /// 处理统计
    stats: EventStats,
    /// 配置参数
    config: ProcessorConfig,
}

/// 事件处理统计
#[derive(Debug, Default)]
pub struct EventStats {
    pub total_events: u32,
    pub processed_events: u32,
    pub dropped_events: u32,
    pub max_queue_size: usize,
    pub avg_processing_time: u32,
}

/// 处理器配置
#[derive(Debug)]
pub struct ProcessorConfig {
    pub max_queue_size: usize,
    pub enable_stats: bool,
    pub timeout_ms: u32,
}

impl Default for ProcessorConfig {
    fn default() -> Self {
        Self {
            max_queue_size: 32,
            enable_stats: true,
            timeout_ms: 1000,
        }
    }
}

impl<const N: usize> EventProcessor<N> {
    /// 创建新的事件处理器
    pub fn new(config: ProcessorConfig) -> Self {
        Self {
            event_queue: Mutex::new(RefCell::new(Deque::new())),
            stats: EventStats::default(),
            config,
        }
    }
    
    /// 添加事件到队列
    pub fn push_event(&mut self, event: SystemEvent, priority: Priority) -> Result<(), ()> {
        critical_section::with(|cs| {
            let mut queue = self.event_queue.borrow_ref_mut(cs);
            
            // 检查队列是否已满
            if queue.len() >= self.config.max_queue_size {
                self.stats.dropped_events += 1;
                return Err(());
            }
            
            // 按优先级插入事件
            let mut inserted = false;
            for i in 0..queue.len() {
                if let Some((_, existing_priority)) = queue.get(i) {
                    if priority < *existing_priority {
                        queue.insert(i, (event, priority)).map_err(|_| ())?;
                        inserted = true;
                        break;
                    }
                }
            }
            
            if !inserted {
                queue.push_back((event, priority)).map_err(|_| ())?;
            }
            
            self.stats.total_events += 1;
            self.stats.max_queue_size = self.stats.max_queue_size.max(queue.len());
            
            Ok(())
        })
    }
    
    /// 处理下一个事件
    pub fn process_next_event(&mut self) -> Option<SystemEvent> {
        critical_section::with(|cs| {
            let mut queue = self.event_queue.borrow_ref_mut(cs);
            if let Some((event, _)) = queue.pop_front() {
                self.stats.processed_events += 1;
                Some(event)
            } else {
                None
            }
        })
    }
    
    /// 获取队列长度
    pub fn queue_length(&self) -> usize {
        critical_section::with(|cs| {
            self.event_queue.borrow_ref(cs).len()
        })
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> &EventStats {
        &self.stats
    }
    
    /// 清空事件队列
    pub fn clear_queue(&mut self) {
        critical_section::with(|cs| {
            self.event_queue.borrow_ref_mut(cs).clear();
        });
    }
}

/// 异步中断处理器
pub struct AsyncInterruptHandler<P, const N: usize> 
where
    P: OutputPin,
{
    /// 状态LED
    status_led: P,
    /// 待处理任务队列
    task_queue: Mutex<RefCell<Vec<AsyncTask, N>>>,
    /// 当前状态
    state: HandlerState,
    /// 性能计数器
    performance: PerformanceCounter,
}

/// 异步任务定义
#[derive(Debug, Clone)]
pub struct AsyncTask {
    pub id: u16,
    pub task_type: TaskType,
    pub data: [u8; 8],
    pub timestamp: u32,
    pub retry_count: u8,
}

/// 任务类型
#[derive(Debug, Clone, Copy)]
pub enum TaskType {
    DataProcessing,
    Communication,
    Calculation,
    Storage,
}

/// 处理器状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HandlerState {
    Idle,
    Processing,
    Waiting,
    Error,
}

/// 性能计数器
#[derive(Debug, Default)]
pub struct PerformanceCounter {
    pub task_count: u32,
    pub total_processing_time: u32,
    pub max_processing_time: u32,
    pub error_count: u32,
}

impl<P, const N: usize> AsyncInterruptHandler<P, N>
where
    P: OutputPin,
{
    /// 创建新的异步处理器
    pub fn new(status_led: P) -> Self {
        Self {
            status_led,
            task_queue: Mutex::new(RefCell::new(Vec::new())),
            state: HandlerState::Idle,
            performance: PerformanceCounter::default(),
        }
    }
    
    /// 添加异步任务
    pub fn add_task(&mut self, task: AsyncTask) -> Result<(), ()> {
        critical_section::with(|cs| {
            let mut queue = self.task_queue.borrow_ref_mut(cs);
            queue.push(task).map_err(|_| ())
        })
    }
    
    /// 处理下一个任务
    pub fn process_next_task(&mut self) -> Result<bool, ()> {
        let task = critical_section::with(|cs| {
            let mut queue = self.task_queue.borrow_ref_mut(cs);
            queue.pop()
        });
        
        if let Some(task) = task {
            self.state = HandlerState::Processing;
            let _ = self.status_led.set_high();
            
            // 模拟任务处理
            let start_time = self.get_current_time();
            self.execute_task(&task)?;
            let processing_time = self.get_current_time() - start_time;
            
            // 更新性能统计
            self.performance.task_count += 1;
            self.performance.total_processing_time += processing_time;
            self.performance.max_processing_time = 
                self.performance.max_processing_time.max(processing_time);
            
            self.state = HandlerState::Idle;
            let _ = self.status_led.set_low();
            
            Ok(true)
        } else {
            Ok(false)
        }
    }
    
    /// 执行具体任务
    fn execute_task(&mut self, task: &AsyncTask) -> Result<(), ()> {
        match task.task_type {
            TaskType::DataProcessing => {
                // 数据处理逻辑
                self.process_data(&task.data)
            },
            TaskType::Communication => {
                // 通信处理逻辑
                self.handle_communication(task.id)
            },
            TaskType::Calculation => {
                // 计算处理逻辑
                self.perform_calculation(&task.data)
            },
            TaskType::Storage => {
                // 存储处理逻辑
                self.store_data(&task.data)
            },
        }
    }
    
    /// 数据处理
    fn process_data(&mut self, data: &[u8; 8]) -> Result<(), ()> {
        // 实现数据处理逻辑
        let _checksum: u8 = data.iter().fold(0, |acc, &x| acc.wrapping_add(x));
        Ok(())
    }
    
    /// 通信处理
    fn handle_communication(&mut self, _id: u16) -> Result<(), ()> {
        // 实现通信逻辑
        Ok(())
    }
    
    /// 计算处理
    fn perform_calculation(&mut self, data: &[u8; 8]) -> Result<(), ()> {
        // 实现计算逻辑
        let _result: u32 = data.iter().map(|&x| x as u32).sum();
        Ok(())
    }
    
    /// 存储处理
    fn store_data(&mut self, _data: &[u8; 8]) -> Result<(), ()> {
        // 实现存储逻辑
        Ok(())
    }
    
    /// 获取当前时间（模拟）
    fn get_current_time(&self) -> u32 {
        // 在实际应用中，这里应该返回系统时钟
        0
    }
    
    /// 获取当前状态
    pub fn get_state(&self) -> HandlerState {
        self.state
    }
    
    /// 获取性能统计
    pub fn get_performance(&self) -> &PerformanceCounter {
        &self.performance
    }
    
    /// 获取队列长度
    pub fn queue_length(&self) -> usize {
        critical_section::with(|cs| {
            self.task_queue.borrow_ref(cs).len()
        })
    }
}

/// 状态机定义
statemachine! {
    transitions: {
        *Idle + StartProcessing = Processing,
        Processing + TaskComplete = Idle,
        Processing + TaskError = Error,
        Error + Reset = Idle,
        Idle + Shutdown = Stopped,
    }
}

/// 中断驱动状态机
pub struct InterruptStateMachine {
    /// 状态机实例
    sm: StateMachine<StateMachineContext>,
    /// 上下文数据
    context: StateMachineContext,
}

/// 状态机上下文
#[derive(Debug, Default)]
pub struct StateMachineContext {
    pub task_count: u32,
    pub error_count: u32,
    pub last_event: Option<SystemEvent>,
}

impl InterruptStateMachine {
    /// 创建新的状态机
    pub fn new() -> Self {
        Self {
            sm: StateMachine::new(StateMachineContext::default()),
            context: StateMachineContext::default(),
        }
    }
    
    /// 处理事件
    pub fn handle_event(&mut self, event: SystemEvent) -> Result<(), ()> {
        self.context.last_event = Some(event);
        
        match event {
            SystemEvent::GpioInterrupt { .. } => {
                let _ = self.sm.process_event(Events::StartProcessing);
            },
            SystemEvent::TimerInterrupt { .. } => {
                let _ = self.sm.process_event(Events::TaskComplete);
            },
            _ => {
                // 处理其他事件
            }
        }
        
        Ok(())
    }
    
    /// 获取当前状态
    pub fn current_state(&self) -> &States {
        self.sm.state()
    }
    
    /// 获取上下文
    pub fn get_context(&self) -> &StateMachineContext {
        &self.context
    }
}

/// 环形缓冲区管理器
pub struct RingBufferManager<T, const N: usize> {
    /// 缓冲区数据
    buffer: Mutex<RefCell<Deque<T, N>>>,
    /// 统计信息
    stats: BufferStats,
    /// 配置参数
    config: BufferConfig,
}

/// 缓冲区统计
#[derive(Debug, Default)]
pub struct BufferStats {
    pub total_writes: u32,
    pub total_reads: u32,
    pub overruns: u32,
    pub underruns: u32,
    pub max_fill_level: usize,
}

/// 缓冲区配置
#[derive(Debug)]
pub struct BufferConfig {
    pub watermark_high: usize,
    pub watermark_low: usize,
    pub enable_overwrite: bool,
}

impl Default for BufferConfig {
    fn default() -> Self {
        Self {
            watermark_high: 24,
            watermark_low: 8,
            enable_overwrite: false,
        }
    }
}

impl<T, const N: usize> RingBufferManager<T, N> 
where
    T: Clone,
{
    /// 创建新的缓冲区管理器
    pub fn new(config: BufferConfig) -> Self {
        Self {
            buffer: Mutex::new(RefCell::new(Deque::new())),
            stats: BufferStats::default(),
            config,
        }
    }
    
    /// 写入数据
    pub fn write(&mut self, data: T) -> Result<(), T> {
        critical_section::with(|cs| {
            let mut buffer = self.buffer.borrow_ref_mut(cs);
            
            if buffer.len() >= N {
                if self.config.enable_overwrite {
                    let _ = buffer.pop_front();
                    self.stats.overruns += 1;
                } else {
                    return Err(data);
                }
            }
            
            buffer.push_back(data).map_err(|data| data)?;
            self.stats.total_writes += 1;
            self.stats.max_fill_level = self.stats.max_fill_level.max(buffer.len());
            
            Ok(())
        })
    }
    
    /// 读取数据
    pub fn read(&mut self) -> Option<T> {
        critical_section::with(|cs| {
            let mut buffer = self.buffer.borrow_ref_mut(cs);
            if let Some(data) = buffer.pop_front() {
                self.stats.total_reads += 1;
                Some(data)
            } else {
                self.stats.underruns += 1;
                None
            }
        })
    }
    
    /// 获取填充级别
    pub fn fill_level(&self) -> usize {
        critical_section::with(|cs| {
            self.buffer.borrow_ref(cs).len()
        })
    }
    
    /// 检查水位线
    pub fn check_watermarks(&self) -> (bool, bool) {
        let level = self.fill_level();
        (
            level >= self.config.watermark_high,
            level <= self.config.watermark_low,
        )
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> &BufferStats {
        &self.stats
    }
    
    /// 清空缓冲区
    pub fn clear(&mut self) {
        critical_section::with(|cs| {
            self.buffer.borrow_ref_mut(cs).clear();
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_event_processor() {
        let mut processor = EventProcessor::<16>::new(ProcessorConfig::default());
        
        // 测试事件添加
        let event = SystemEvent::GpioInterrupt { pin: 1, edge: EdgeType::Rising };
        assert!(processor.push_event(event, Priority::High).is_ok());
        assert_eq!(processor.queue_length(), 1);
        
        // 测试事件处理
        let processed = processor.process_next_event();
        assert!(processed.is_some());
        assert_eq!(processor.queue_length(), 0);
    }
    
    #[test]
    fn test_ring_buffer() {
        let mut buffer = RingBufferManager::<u8, 8>::new(BufferConfig::default());
        
        // 测试写入
        for i in 0..5 {
            assert!(buffer.write(i).is_ok());
        }
        assert_eq!(buffer.fill_level(), 5);
        
        // 测试读取
        for i in 0..3 {
            assert_eq!(buffer.read(), Some(i));
        }
        assert_eq!(buffer.fill_level(), 2);
    }
    
    #[test]
    fn test_priority_ordering() {
        let mut processor = EventProcessor::<16>::new(ProcessorConfig::default());
        
        // 添加不同优先级的事件
        let event1 = SystemEvent::TimerInterrupt { timer_id: 1 };
        let event2 = SystemEvent::GpioInterrupt { pin: 1, edge: EdgeType::Rising };
        
        processor.push_event(event1, Priority::Low).unwrap();
        processor.push_event(event2, Priority::Critical).unwrap();
        
        // 验证高优先级事件先处理
        if let Some(SystemEvent::GpioInterrupt { .. }) = processor.process_next_event() {
            // 正确：高优先级事件先处理
        } else {
            panic!("Priority ordering failed");
        }
    }
}