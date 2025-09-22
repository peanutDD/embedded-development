#![no_std]
#![no_main]
#![deny(unsafe_code)]
#![warn(missing_docs)]

//! # 系统集成库
//! 
//! 本库提供嵌入式系统集成的核心功能和框架，包括：
//! - 系统架构设计和模块化管理
//! - 通信协议集成和数据交换
//! - 状态管理和事件处理
//! - 实时调度和任务管理
//! - 故障检测和系统恢复
//! - 性能监控和优化

use core::fmt;
use heapless::{Vec, FnvIndexMap};
use embedded_hal::digital::{InputPin, OutputPin};
use fugit::{Duration, Instant};

/// 系统集成错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemError {
    /// 初始化失败
    InitializationFailed,
    /// 通信错误
    CommunicationError,
    /// 配置错误
    ConfigurationError,
    /// 资源不足
    ResourceExhausted,
    /// 超时错误
    Timeout,
    /// 硬件故障
    HardwareFault,
    /// 协议错误
    ProtocolError,
    /// 状态错误
    InvalidState,
}

impl fmt::Display for SystemError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SystemError::InitializationFailed => write!(f, "系统初始化失败"),
            SystemError::CommunicationError => write!(f, "通信错误"),
            SystemError::ConfigurationError => write!(f, "配置错误"),
            SystemError::ResourceExhausted => write!(f, "资源不足"),
            SystemError::Timeout => write!(f, "操作超时"),
            SystemError::HardwareFault => write!(f, "硬件故障"),
            SystemError::ProtocolError => write!(f, "协议错误"),
            SystemError::InvalidState => write!(f, "无效状态"),
        }
    }
}

/// 系统组件特征
pub trait SystemComponent {
    /// 组件类型
    type ComponentType;
    /// 配置类型
    type Config;
    /// 状态类型
    type State;
    /// 事件类型
    type Event;

    /// 初始化组件
    fn initialize(&mut self, config: Self::Config) -> Result<(), SystemError>;
    
    /// 更新组件状态
    fn update(&mut self) -> Result<(), SystemError>;
    
    /// 处理事件
    fn handle_event(&mut self, event: Self::Event) -> Result<(), SystemError>;
    
    /// 获取组件状态
    fn get_state(&self) -> Self::State;
    
    /// 重置组件
    fn reset(&mut self) -> Result<(), SystemError>;
}

/// 通信接口特征
pub trait CommunicationInterface {
    /// 数据类型
    type Data;
    /// 地址类型
    type Address;

    /// 发送数据
    fn send(&mut self, address: Self::Address, data: &Self::Data) -> Result<(), SystemError>;
    
    /// 接收数据
    fn receive(&mut self) -> Result<Option<(Self::Address, Self::Data)>, SystemError>;
    
    /// 检查连接状态
    fn is_connected(&self) -> bool;
    
    /// 获取统计信息
    fn get_statistics(&self) -> CommunicationStats;
}

/// 通信统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct CommunicationStats {
    /// 发送字节数
    pub bytes_sent: u32,
    /// 接收字节数
    pub bytes_received: u32,
    /// 发送错误数
    pub send_errors: u16,
    /// 接收错误数
    pub receive_errors: u16,
    /// 连接时间
    pub connection_time: u32,
}

/// 系统事件类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemEvent {
    /// 系统启动
    SystemStartup,
    /// 系统关闭
    SystemShutdown,
    /// 组件故障
    ComponentFault(u8),
    /// 通信错误
    CommunicationError(u8),
    /// 配置更新
    ConfigurationUpdate,
    /// 性能警告
    PerformanceWarning,
    /// 安全事件
    SecurityEvent,
    /// 用户输入
    UserInput(u16),
}

/// 系统状态类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemState {
    /// 未初始化
    Uninitialized,
    /// 初始化中
    Initializing,
    /// 运行中
    Running,
    /// 暂停
    Paused,
    /// 错误状态
    Error,
    /// 维护模式
    Maintenance,
    /// 关闭中
    Shutting Down,
}

/// 优先级类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    /// 低优先级
    Low = 0,
    /// 普通优先级
    Normal = 1,
    /// 高优先级
    High = 2,
    /// 紧急优先级
    Critical = 3,
}

/// 任务调度器
pub struct TaskScheduler<const N: usize> {
    tasks: Vec<Task, N>,
    current_time: u32,
    next_task_id: u8,
}

/// 任务定义
#[derive(Debug, Clone)]
pub struct Task {
    /// 任务ID
    pub id: u8,
    /// 任务名称
    pub name: &'static str,
    /// 优先级
    pub priority: Priority,
    /// 执行周期（毫秒）
    pub period_ms: u32,
    /// 下次执行时间
    pub next_execution: u32,
    /// 执行次数
    pub execution_count: u32,
    /// 是否启用
    pub enabled: bool,
}

impl<const N: usize> TaskScheduler<N> {
    /// 创建新的任务调度器
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            current_time: 0,
            next_task_id: 1,
        }
    }

    /// 添加任务
    pub fn add_task(&mut self, name: &'static str, priority: Priority, period_ms: u32) -> Result<u8, SystemError> {
        if self.tasks.len() >= N {
            return Err(SystemError::ResourceExhausted);
        }

        let task_id = self.next_task_id;
        self.next_task_id += 1;

        let task = Task {
            id: task_id,
            name,
            priority,
            period_ms,
            next_execution: self.current_time + period_ms,
            execution_count: 0,
            enabled: true,
        };

        self.tasks.push(task).map_err(|_| SystemError::ResourceExhausted)?;
        Ok(task_id)
    }

    /// 更新时间并获取需要执行的任务
    pub fn update(&mut self, current_time: u32) -> Vec<u8, N> {
        self.current_time = current_time;
        let mut ready_tasks = Vec::new();

        // 按优先级排序
        self.tasks.sort_by(|a, b| b.priority.cmp(&a.priority));

        for task in &mut self.tasks {
            if task.enabled && current_time >= task.next_execution {
                if ready_tasks.push(task.id).is_ok() {
                    task.next_execution = current_time + task.period_ms;
                    task.execution_count += 1;
                }
            }
        }

        ready_tasks
    }

    /// 启用/禁用任务
    pub fn set_task_enabled(&mut self, task_id: u8, enabled: bool) -> Result<(), SystemError> {
        for task in &mut self.tasks {
            if task.id == task_id {
                task.enabled = enabled;
                return Ok(());
            }
        }
        Err(SystemError::ConfigurationError)
    }

    /// 获取任务统计信息
    pub fn get_task_stats(&self, task_id: u8) -> Option<&Task> {
        self.tasks.iter().find(|task| task.id == task_id)
    }
}

/// 事件管理器
pub struct EventManager<const N: usize> {
    events: Vec<(SystemEvent, u32), N>,
    handlers: FnvIndexMap<SystemEvent, fn(SystemEvent), 8>,
}

impl<const N: usize> EventManager<N> {
    /// 创建新的事件管理器
    pub fn new() -> Self {
        Self {
            events: Vec::new(),
            handlers: FnvIndexMap::new(),
        }
    }

    /// 注册事件处理器
    pub fn register_handler(&mut self, event: SystemEvent, handler: fn(SystemEvent)) -> Result<(), SystemError> {
        self.handlers.insert(event, handler).map_err(|_| SystemError::ResourceExhausted)?;
        Ok(())
    }

    /// 发布事件
    pub fn publish_event(&mut self, event: SystemEvent, timestamp: u32) -> Result<(), SystemError> {
        self.events.push((event, timestamp)).map_err(|_| SystemError::ResourceExhausted)?;
        Ok(())
    }

    /// 处理事件队列
    pub fn process_events(&mut self) {
        while let Some((event, _timestamp)) = self.events.pop() {
            if let Some(&handler) = self.handlers.get(&event) {
                handler(event);
            }
        }
    }

    /// 获取事件统计
    pub fn get_event_count(&self) -> usize {
        self.events.len()
    }
}

/// 配置管理器
pub struct ConfigurationManager<T, const N: usize> {
    configs: FnvIndexMap<&'static str, T, N>,
    version: u16,
}

impl<T: Clone, const N: usize> ConfigurationManager<T, N> {
    /// 创建新的配置管理器
    pub fn new() -> Self {
        Self {
            configs: FnvIndexMap::new(),
            version: 1,
        }
    }

    /// 设置配置项
    pub fn set_config(&mut self, key: &'static str, value: T) -> Result<(), SystemError> {
        self.configs.insert(key, value).map_err(|_| SystemError::ResourceExhausted)?;
        self.version += 1;
        Ok(())
    }

    /// 获取配置项
    pub fn get_config(&self, key: &'static str) -> Option<&T> {
        self.configs.get(key)
    }

    /// 获取配置版本
    pub fn get_version(&self) -> u16 {
        self.version
    }

    /// 清除所有配置
    pub fn clear(&mut self) {
        self.configs.clear();
        self.version += 1;
    }
}

/// 性能监控器
#[derive(Debug, Clone, Copy, Default)]
pub struct PerformanceMonitor {
    /// CPU使用率（百分比）
    pub cpu_usage: u8,
    /// 内存使用率（百分比）
    pub memory_usage: u8,
    /// 任务执行时间统计
    pub task_execution_time: u32,
    /// 中断响应时间
    pub interrupt_response_time: u16,
    /// 系统运行时间
    pub uptime: u32,
    /// 错误计数
    pub error_count: u16,
}

impl PerformanceMonitor {
    /// 创建新的性能监控器
    pub fn new() -> Self {
        Self::default()
    }

    /// 更新CPU使用率
    pub fn update_cpu_usage(&mut self, usage: u8) {
        self.cpu_usage = usage.min(100);
    }

    /// 更新内存使用率
    pub fn update_memory_usage(&mut self, usage: u8) {
        self.memory_usage = usage.min(100);
    }

    /// 记录任务执行时间
    pub fn record_task_time(&mut self, execution_time: u32) {
        self.task_execution_time = execution_time;
    }

    /// 记录中断响应时间
    pub fn record_interrupt_time(&mut self, response_time: u16) {
        self.interrupt_response_time = response_time;
    }

    /// 增加错误计数
    pub fn increment_error_count(&mut self) {
        self.error_count = self.error_count.saturating_add(1);
    }

    /// 更新系统运行时间
    pub fn update_uptime(&mut self, uptime: u32) {
        self.uptime = uptime;
    }

    /// 检查性能警告
    pub fn check_performance_warnings(&self) -> Vec<&'static str, 4> {
        let mut warnings = Vec::new();

        if self.cpu_usage > 90 {
            let _ = warnings.push("CPU使用率过高");
        }
        if self.memory_usage > 85 {
            let _ = warnings.push("内存使用率过高");
        }
        if self.task_execution_time > 10000 {
            let _ = warnings.push("任务执行时间过长");
        }
        if self.interrupt_response_time > 1000 {
            let _ = warnings.push("中断响应时间过长");
        }

        warnings
    }
}

/// 系统集成框架
pub struct SystemIntegrationFramework<const N: usize> {
    /// 系统状态
    pub state: SystemState,
    /// 任务调度器
    pub scheduler: TaskScheduler<N>,
    /// 事件管理器
    pub event_manager: EventManager<N>,
    /// 配置管理器
    pub config_manager: ConfigurationManager<u32, N>,
    /// 性能监控器
    pub performance_monitor: PerformanceMonitor,
    /// 系统启动时间
    pub startup_time: u32,
}

impl<const N: usize> SystemIntegrationFramework<N> {
    /// 创建新的系统集成框架
    pub fn new() -> Self {
        Self {
            state: SystemState::Uninitialized,
            scheduler: TaskScheduler::new(),
            event_manager: EventManager::new(),
            config_manager: ConfigurationManager::new(),
            performance_monitor: PerformanceMonitor::new(),
            startup_time: 0,
        }
    }

    /// 初始化系统
    pub fn initialize(&mut self, startup_time: u32) -> Result<(), SystemError> {
        self.state = SystemState::Initializing;
        self.startup_time = startup_time;

        // 发布系统启动事件
        self.event_manager.publish_event(SystemEvent::SystemStartup, startup_time)?;

        self.state = SystemState::Running;
        Ok(())
    }

    /// 系统主循环
    pub fn run_cycle(&mut self, current_time: u32) -> Result<(), SystemError> {
        if self.state != SystemState::Running {
            return Err(SystemError::InvalidState);
        }

        // 更新性能监控
        self.performance_monitor.update_uptime(current_time - self.startup_time);

        // 处理事件
        self.event_manager.process_events();

        // 执行调度任务
        let ready_tasks = self.scheduler.update(current_time);
        for _task_id in ready_tasks {
            // 这里应该调用具体的任务执行函数
            // 由于是库代码，具体实现由应用程序提供
        }

        // 检查性能警告
        let warnings = self.performance_monitor.check_performance_warnings();
        if !warnings.is_empty() {
            self.event_manager.publish_event(SystemEvent::PerformanceWarning, current_time)?;
        }

        Ok(())
    }

    /// 关闭系统
    pub fn shutdown(&mut self, current_time: u32) -> Result<(), SystemError> {
        self.state = SystemState::ShuttingDown;
        self.event_manager.publish_event(SystemEvent::SystemShutdown, current_time)?;
        Ok(())
    }

    /// 获取系统统计信息
    pub fn get_system_stats(&self) -> SystemStats {
        SystemStats {
            state: self.state,
            uptime: self.performance_monitor.uptime,
            task_count: self.scheduler.tasks.len() as u8,
            event_count: self.event_manager.get_event_count() as u16,
            config_version: self.config_manager.get_version(),
            cpu_usage: self.performance_monitor.cpu_usage,
            memory_usage: self.performance_monitor.memory_usage,
            error_count: self.performance_monitor.error_count,
        }
    }
}

/// 系统统计信息
#[derive(Debug, Clone, Copy)]
pub struct SystemStats {
    /// 系统状态
    pub state: SystemState,
    /// 运行时间
    pub uptime: u32,
    /// 任务数量
    pub task_count: u8,
    /// 事件数量
    pub event_count: u16,
    /// 配置版本
    pub config_version: u16,
    /// CPU使用率
    pub cpu_usage: u8,
    /// 内存使用率
    pub memory_usage: u8,
    /// 错误计数
    pub error_count: u16,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_task_scheduler() {
        let mut scheduler = TaskScheduler::<4>::new();
        
        // 添加任务
        let task_id = scheduler.add_task("test_task", Priority::Normal, 100).unwrap();
        assert_eq!(task_id, 1);

        // 更新调度器
        let ready_tasks = scheduler.update(150);
        assert_eq!(ready_tasks.len(), 1);
        assert_eq!(ready_tasks[0], task_id);
    }

    #[test]
    fn test_event_manager() {
        let mut event_manager = EventManager::<4>::new();
        
        // 发布事件
        event_manager.publish_event(SystemEvent::SystemStartup, 0).unwrap();
        assert_eq!(event_manager.get_event_count(), 1);

        // 处理事件
        event_manager.process_events();
        assert_eq!(event_manager.get_event_count(), 0);
    }

    #[test]
    fn test_configuration_manager() {
        let mut config_manager = ConfigurationManager::<u32, 4>::new();
        
        // 设置配置
        config_manager.set_config("test_key", 42).unwrap();
        assert_eq!(config_manager.get_config("test_key"), Some(&42));
        assert_eq!(config_manager.get_version(), 2);
    }

    #[test]
    fn test_performance_monitor() {
        let mut monitor = PerformanceMonitor::new();
        
        // 更新性能指标
        monitor.update_cpu_usage(95);
        monitor.update_memory_usage(90);
        
        // 检查警告
        let warnings = monitor.check_performance_warnings();
        assert_eq!(warnings.len(), 2);
    }

    #[test]
    fn test_system_integration_framework() {
        let mut framework = SystemIntegrationFramework::<8>::new();
        
        // 初始化系统
        framework.initialize(0).unwrap();
        assert_eq!(framework.state, SystemState::Running);

        // 运行系统循环
        framework.run_cycle(100).unwrap();

        // 获取统计信息
        let stats = framework.get_system_stats();
        assert_eq!(stats.state, SystemState::Running);
        assert_eq!(stats.uptime, 100);
    }
}