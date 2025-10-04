# 嵌入式软件架构

## 概述

嵌入式软件架构是系统设计的核心，它决定了系统的可维护性、可扩展性、可靠性和性能。良好的软件架构能够有效管理复杂性，提高开发效率，并确保系统的长期演进能力。

## 学习目标

完成本章节后，你将掌握：
- 嵌入式软件架构的基本原则和模式
- 分层架构的设计和实现
- 状态机模式在嵌入式系统中的应用
- 事件驱动架构的设计方法
- 组件化设计和模块化开发
- 常用设计模式的嵌入式应用

## 1. 软件架构基础

### 1.1 架构设计原则

```rust
// 架构设计原则的代码体现

// 1. 单一职责原则 (Single Responsibility Principle)
pub struct TemperatureSensor {
    adc_channel: u8,
    calibration_offset: f32,
}

impl TemperatureSensor {
    // 只负责温度读取
    pub fn read_temperature(&self) -> Result<f32, SensorError> {
        let raw_value = self.read_adc()?;
        Ok(self.convert_to_celsius(raw_value))
    }
    
    fn read_adc(&self) -> Result<u16, SensorError> {
        // ADC读取逻辑
        Ok(0)
    }
    
    fn convert_to_celsius(&self, raw: u16) -> f32 {
        // 温度转换逻辑
        (raw as f32 * 3.3 / 4096.0 - 0.5) * 100.0 + self.calibration_offset
    }
}

// 2. 开放封闭原则 (Open-Closed Principle)
pub trait Sensor {
    type Output;
    type Error;
    
    fn read(&self) -> Result<Self::Output, Self::Error>;
    fn calibrate(&mut self, reference: Self::Output) -> Result<(), Self::Error>;
}

// 可以扩展新的传感器类型而不修改现有代码
impl Sensor for TemperatureSensor {
    type Output = f32;
    type Error = SensorError;
    
    fn read(&self) -> Result<Self::Output, Self::Error> {
        self.read_temperature()
    }
    
    fn calibrate(&mut self, reference: Self::Output) -> Result<(), Self::Error> {
        // 校准逻辑
        Ok(())
    }
}

// 3. 依赖倒置原则 (Dependency Inversion Principle)
pub struct DataLogger<S: Sensor> {
    sensor: S,
    storage: Box<dyn Storage>,
    sampling_interval: u32,
}

pub trait Storage {
    fn store(&mut self, data: &[u8]) -> Result<(), StorageError>;
    fn retrieve(&self, offset: u32, length: u32) -> Result<Vec<u8, 256>, StorageError>;
}

// 具体实现可以是Flash、EEPROM、SD卡等
pub struct FlashStorage {
    base_address: u32,
    current_offset: u32,
}

impl Storage for FlashStorage {
    fn store(&mut self, data: &[u8]) -> Result<(), StorageError> {
        // Flash存储实现
        Ok(())
    }
    
    fn retrieve(&self, offset: u32, length: u32) -> Result<Vec<u8, 256>, StorageError> {
        // Flash读取实现
        Ok(Vec::new())
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SensorError {
    AdcError,
    CalibrationError,
    CommunicationError,
}

#[derive(Debug, Clone, Copy)]
pub enum StorageError {
    WriteError,
    ReadError,
    FullError,
}
```

### 1.2 架构质量属性

```rust
// 架构质量属性的量化和监控
#[derive(Debug, Clone)]
pub struct ArchitectureMetrics {
    pub performance: PerformanceMetrics,
    pub reliability: ReliabilityMetrics,
    pub maintainability: MaintainabilityMetrics,
    pub scalability: ScalabilityMetrics,
}

#[derive(Debug, Clone)]
pub struct PerformanceMetrics {
    pub response_time_ms: u32,
    pub throughput_ops_per_sec: u32,
    pub cpu_utilization_percent: f32,
    pub memory_utilization_percent: f32,
}

#[derive(Debug, Clone)]
pub struct ReliabilityMetrics {
    pub uptime_hours: u32,
    pub error_rate_per_hour: f32,
    pub recovery_time_ms: u32,
    pub fault_tolerance_level: u8,
}

#[derive(Debug, Clone)]
pub struct MaintainabilityMetrics {
    pub cyclomatic_complexity: u32,
    pub coupling_factor: f32,
    pub cohesion_factor: f32,
    pub test_coverage_percent: f32,
}

#[derive(Debug, Clone)]
pub struct ScalabilityMetrics {
    pub max_concurrent_tasks: u32,
    pub memory_scalability_factor: f32,
    pub processing_scalability_factor: f32,
}

// 架构健康监控器
pub struct ArchitectureMonitor {
    metrics: ArchitectureMetrics,
    thresholds: ArchitectureThresholds,
    alerts: Vec<ArchitectureAlert, 16>,
}

#[derive(Debug, Clone)]
pub struct ArchitectureThresholds {
    pub max_response_time_ms: u32,
    pub max_cpu_utilization: f32,
    pub max_memory_utilization: f32,
    pub max_error_rate: f32,
}

#[derive(Debug, Clone)]
pub struct ArchitectureAlert {
    pub alert_type: AlertType,
    pub severity: AlertSeverity,
    pub message: &'static str,
    pub timestamp: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum AlertType {
    Performance,
    Reliability,
    Resource,
    Security,
}

#[derive(Debug, Clone, Copy)]
pub enum AlertSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

impl ArchitectureMonitor {
    pub fn new(thresholds: ArchitectureThresholds) -> Self {
        Self {
            metrics: ArchitectureMetrics {
                performance: PerformanceMetrics {
                    response_time_ms: 0,
                    throughput_ops_per_sec: 0,
                    cpu_utilization_percent: 0.0,
                    memory_utilization_percent: 0.0,
                },
                reliability: ReliabilityMetrics {
                    uptime_hours: 0,
                    error_rate_per_hour: 0.0,
                    recovery_time_ms: 0,
                    fault_tolerance_level: 0,
                },
                maintainability: MaintainabilityMetrics {
                    cyclomatic_complexity: 0,
                    coupling_factor: 0.0,
                    cohesion_factor: 0.0,
                    test_coverage_percent: 0.0,
                },
                scalability: ScalabilityMetrics {
                    max_concurrent_tasks: 0,
                    memory_scalability_factor: 0.0,
                    processing_scalability_factor: 0.0,
                },
            },
            thresholds,
            alerts: Vec::new(),
        }
    }
    
    pub fn update_metrics(&mut self, new_metrics: ArchitectureMetrics) {
        self.metrics = new_metrics;
        self.check_thresholds();
    }
    
    fn check_thresholds(&mut self) {
        // 检查性能阈值
        if self.metrics.performance.response_time_ms > self.thresholds.max_response_time_ms {
            self.add_alert(ArchitectureAlert {
                alert_type: AlertType::Performance,
                severity: AlertSeverity::Warning,
                message: "Response time exceeded threshold",
                timestamp: self.get_timestamp(),
            });
        }
        
        if self.metrics.performance.cpu_utilization_percent > self.thresholds.max_cpu_utilization {
            self.add_alert(ArchitectureAlert {
                alert_type: AlertType::Resource,
                severity: AlertSeverity::Error,
                message: "CPU utilization too high",
                timestamp: self.get_timestamp(),
            });
        }
        
        if self.metrics.performance.memory_utilization_percent > self.thresholds.max_memory_utilization {
            self.add_alert(ArchitectureAlert {
                alert_type: AlertType::Resource,
                severity: AlertSeverity::Error,
                message: "Memory utilization too high",
                timestamp: self.get_timestamp(),
            });
        }
        
        if self.metrics.reliability.error_rate_per_hour > self.thresholds.max_error_rate {
            self.add_alert(ArchitectureAlert {
                alert_type: AlertType::Reliability,
                severity: AlertSeverity::Critical,
                message: "Error rate exceeded threshold",
                timestamp: self.get_timestamp(),
            });
        }
    }
    
    fn add_alert(&mut self, alert: ArchitectureAlert) {
        if self.alerts.len() >= 16 {
            self.alerts.remove(0); // 移除最旧的告警
        }
        let _ = self.alerts.push(alert);
    }
    
    fn get_timestamp(&self) -> u32 {
        // 获取当前时间戳
        0
    }
    
    pub fn get_health_score(&self) -> f32 {
        // 计算架构健康分数 (0-100)
        let performance_score = self.calculate_performance_score();
        let reliability_score = self.calculate_reliability_score();
        let maintainability_score = self.calculate_maintainability_score();
        
        (performance_score + reliability_score + maintainability_score) / 3.0
    }
    
    fn calculate_performance_score(&self) -> f32 {
        let cpu_score = (100.0 - self.metrics.performance.cpu_utilization_percent).max(0.0);
        let memory_score = (100.0 - self.metrics.performance.memory_utilization_percent).max(0.0);
        let response_score = if self.metrics.performance.response_time_ms <= self.thresholds.max_response_time_ms {
            100.0
        } else {
            (self.thresholds.max_response_time_ms as f32 / self.metrics.performance.response_time_ms as f32 * 100.0).min(100.0)
        };
        
        (cpu_score + memory_score + response_score) / 3.0
    }
    
    fn calculate_reliability_score(&self) -> f32 {
        let error_score = if self.metrics.reliability.error_rate_per_hour <= self.thresholds.max_error_rate {
            100.0
        } else {
            (self.thresholds.max_error_rate / self.metrics.reliability.error_rate_per_hour * 100.0).min(100.0)
        };
        
        let uptime_score = (self.metrics.reliability.uptime_hours as f32 / 24.0 * 100.0).min(100.0);
        
        (error_score + uptime_score) / 2.0
    }
    
    fn calculate_maintainability_score(&self) -> f32 {
        let complexity_score = if self.metrics.maintainability.cyclomatic_complexity <= 10 {
            100.0
        } else {
            (10.0 / self.metrics.maintainability.cyclomatic_complexity as f32 * 100.0).min(100.0)
        };
        
        let coverage_score = self.metrics.maintainability.test_coverage_percent;
        let coupling_score = (1.0 - self.metrics.maintainability.coupling_factor) * 100.0;
        let cohesion_score = self.metrics.maintainability.cohesion_factor * 100.0;
        
        (complexity_score + coverage_score + coupling_score + cohesion_score) / 4.0
    }
}
```

## 2. 分层架构设计

### 2.1 经典分层架构

```rust
// 分层架构实现
pub struct LayeredArchitecture {
    hardware_abstraction_layer: HardwareAbstractionLayer,
    device_driver_layer: DeviceDriverLayer,
    operating_system_layer: OperatingSystemLayer,
    middleware_layer: MiddlewareLayer,
    application_layer: ApplicationLayer,
}

// 硬件抽象层 (HAL)
pub struct HardwareAbstractionLayer {
    gpio_hal: GpioHal,
    timer_hal: TimerHal,
    uart_hal: UartHal,
    spi_hal: SpiHal,
    i2c_hal: I2cHal,
}

pub trait GpioHal {
    fn set_pin_mode(&mut self, pin: u8, mode: PinMode) -> Result<(), HalError>;
    fn write_pin(&mut self, pin: u8, value: bool) -> Result<(), HalError>;
    fn read_pin(&self, pin: u8) -> Result<bool, HalError>;
}

#[derive(Debug, Clone, Copy)]
pub enum PinMode {
    Input,
    Output,
    InputPullUp,
    InputPullDown,
    OutputOpenDrain,
}

pub trait TimerHal {
    fn start_timer(&mut self, timer_id: u8, period_ms: u32) -> Result<(), HalError>;
    fn stop_timer(&mut self, timer_id: u8) -> Result<(), HalError>;
    fn is_timer_expired(&self, timer_id: u8) -> Result<bool, HalError>;
}

pub trait UartHal {
    fn configure(&mut self, config: UartConfig) -> Result<(), HalError>;
    fn send_byte(&mut self, byte: u8) -> Result<(), HalError>;
    fn receive_byte(&mut self) -> Result<Option<u8>, HalError>;
}

// 设备驱动层
pub struct DeviceDriverLayer {
    sensor_drivers: Vec<Box<dyn SensorDriver>, 8>,
    actuator_drivers: Vec<Box<dyn ActuatorDriver>, 8>,
    communication_drivers: Vec<Box<dyn CommunicationDriver>, 4>,
}

pub trait SensorDriver {
    fn initialize(&mut self) -> Result<(), DriverError>;
    fn read_data(&mut self) -> Result<SensorData, DriverError>;
    fn configure(&mut self, config: &SensorConfig) -> Result<(), DriverError>;
    fn get_status(&self) -> SensorStatus;
}

pub trait ActuatorDriver {
    fn initialize(&mut self) -> Result<(), DriverError>;
    fn set_output(&mut self, value: ActuatorValue) -> Result<(), DriverError>;
    fn get_feedback(&self) -> Result<ActuatorFeedback, DriverError>;
}

pub trait CommunicationDriver {
    fn initialize(&mut self) -> Result<(), DriverError>;
    fn send_message(&mut self, message: &[u8]) -> Result<(), DriverError>;
    fn receive_message(&mut self) -> Result<Option<Vec<u8, 256>>, DriverError>;
}

#[derive(Debug, Clone)]
pub struct SensorData {
    pub sensor_id: u8,
    pub timestamp: u32,
    pub value: f32,
    pub unit: &'static str,
    pub quality: DataQuality,
}

#[derive(Debug, Clone, Copy)]
pub enum DataQuality {
    Good,
    Uncertain,
    Bad,
}

#[derive(Debug, Clone)]
pub struct SensorConfig {
    pub sampling_rate: u32,
    pub resolution: u8,
    pub calibration_offset: f32,
    pub calibration_scale: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum SensorStatus {
    Ready,
    Busy,
    Error,
    Calibrating,
}

// 操作系统层
pub struct OperatingSystemLayer {
    task_scheduler: TaskScheduler,
    memory_manager: MemoryManager,
    interrupt_manager: InterruptManager,
    timer_manager: TimerManager,
}

pub struct TaskScheduler {
    tasks: Vec<Task, 16>,
    current_task: Option<usize>,
    scheduler_type: SchedulerType,
}

#[derive(Debug, Clone)]
pub struct Task {
    pub id: u8,
    pub priority: u8,
    pub state: TaskState,
    pub stack_pointer: *mut u8,
    pub stack_size: usize,
    pub entry_point: fn(),
    pub execution_time: u32,
    pub deadline: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskState {
    Ready,
    Running,
    Blocked,
    Suspended,
    Terminated,
}

#[derive(Debug, Clone, Copy)]
pub enum SchedulerType {
    RoundRobin,
    PriorityBased,
    EarliestDeadlineFirst,
    RateMonotonic,
}

impl TaskScheduler {
    pub fn new(scheduler_type: SchedulerType) -> Self {
        Self {
            tasks: Vec::new(),
            current_task: None,
            scheduler_type,
        }
    }
    
    pub fn add_task(&mut self, task: Task) -> Result<(), SchedulerError> {
        if self.tasks.len() >= 16 {
            return Err(SchedulerError::TooManyTasks);
        }
        
        self.tasks.push(task).map_err(|_| SchedulerError::TaskCreationFailed)
    }
    
    pub fn schedule(&mut self) -> Option<usize> {
        match self.scheduler_type {
            SchedulerType::RoundRobin => self.round_robin_schedule(),
            SchedulerType::PriorityBased => self.priority_schedule(),
            SchedulerType::EarliestDeadlineFirst => self.edf_schedule(),
            SchedulerType::RateMonotonic => self.rm_schedule(),
        }
    }
    
    fn round_robin_schedule(&mut self) -> Option<usize> {
        let ready_tasks: Vec<usize, 16> = self.tasks.iter()
            .enumerate()
            .filter(|(_, task)| task.state == TaskState::Ready)
            .map(|(i, _)| i)
            .collect();
        
        if ready_tasks.is_empty() {
            return None;
        }
        
        let next_index = if let Some(current) = self.current_task {
            let current_pos = ready_tasks.iter().position(|&i| i == current);
            if let Some(pos) = current_pos {
                (pos + 1) % ready_tasks.len()
            } else {
                0
            }
        } else {
            0
        };
        
        Some(ready_tasks[next_index])
    }
    
    fn priority_schedule(&self) -> Option<usize> {
        self.tasks.iter()
            .enumerate()
            .filter(|(_, task)| task.state == TaskState::Ready)
            .max_by_key(|(_, task)| task.priority)
            .map(|(i, _)| i)
    }
    
    fn edf_schedule(&self) -> Option<usize> {
        self.tasks.iter()
            .enumerate()
            .filter(|(_, task)| task.state == TaskState::Ready)
            .min_by_key(|(_, task)| task.deadline)
            .map(|(i, _)| i)
    }
    
    fn rm_schedule(&self) -> Option<usize> {
        // Rate Monotonic: 周期越短优先级越高
        // 这里简化为按优先级调度
        self.priority_schedule()
    }
    
    pub fn context_switch(&mut self, new_task_index: usize) {
        if let Some(current) = self.current_task {
            if current < self.tasks.len() {
                self.tasks[current].state = TaskState::Ready;
            }
        }
        
        if new_task_index < self.tasks.len() {
            self.tasks[new_task_index].state = TaskState::Running;
            self.current_task = Some(new_task_index);
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SchedulerError {
    TooManyTasks,
    TaskCreationFailed,
    InvalidTaskId,
    SchedulingError,
}

// 中间件层
pub struct MiddlewareLayer {
    communication_middleware: CommunicationMiddleware,
    data_processing_middleware: DataProcessingMiddleware,
    security_middleware: SecurityMiddleware,
    logging_middleware: LoggingMiddleware,
}

pub struct CommunicationMiddleware {
    protocol_stack: ProtocolStack,
    message_queue: MessageQueue,
    routing_table: RoutingTable,
}

pub struct DataProcessingMiddleware {
    filters: Vec<Box<dyn DataFilter>, 8>,
    transformers: Vec<Box<dyn DataTransformer>, 8>,
    validators: Vec<Box<dyn DataValidator>, 8>,
}

pub trait DataFilter {
    fn filter(&self, data: &SensorData) -> bool;
}

pub trait DataTransformer {
    fn transform(&self, data: SensorData) -> SensorData;
}

pub trait DataValidator {
    fn validate(&self, data: &SensorData) -> ValidationResult;
}

#[derive(Debug, Clone, Copy)]
pub enum ValidationResult {
    Valid,
    Invalid(ValidationError),
}

#[derive(Debug, Clone, Copy)]
pub enum ValidationError {
    OutOfRange,
    InvalidFormat,
    ChecksumError,
    TimeoutError,
}

// 应用层
pub struct ApplicationLayer {
    applications: Vec<Box<dyn Application>, 8>,
    application_manager: ApplicationManager,
}

pub trait Application {
    fn initialize(&mut self) -> Result<(), ApplicationError>;
    fn run(&mut self) -> Result<(), ApplicationError>;
    fn shutdown(&mut self) -> Result<(), ApplicationError>;
    fn get_status(&self) -> ApplicationStatus;
}

#[derive(Debug, Clone, Copy)]
pub enum ApplicationStatus {
    Initializing,
    Running,
    Paused,
    Error,
    Shutdown,
}

#[derive(Debug, Clone, Copy)]
pub enum ApplicationError {
    InitializationFailed,
    RuntimeError,
    ResourceUnavailable,
    ConfigurationError,
}

pub struct ApplicationManager {
    running_applications: Vec<u8, 8>,
    application_configs: Vec<ApplicationConfig, 8>,
}

#[derive(Debug, Clone)]
pub struct ApplicationConfig {
    pub app_id: u8,
    pub priority: u8,
    pub max_memory: usize,
    pub max_cpu_time: u32,
    pub auto_restart: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum HalError {
    InvalidPin,
    InvalidConfig,
    HardwareError,
    TimeoutError,
}

#[derive(Debug, Clone, Copy)]
pub enum DriverError {
    InitializationFailed,
    CommunicationError,
    ConfigurationError,
    HardwareError,
    TimeoutError,
}

// 层间通信接口
pub trait LayerInterface<T> {
    fn send_request(&mut self, request: T) -> Result<(), InterfaceError>;
    fn receive_response(&mut self) -> Result<Option<T>, InterfaceError>;
}

#[derive(Debug, Clone, Copy)]
pub enum InterfaceError {
    BufferFull,
    InvalidRequest,
    LayerNotReady,
    CommunicationError,
}
```

### 2.2 微内核架构

```rust
// 微内核架构实现
pub struct MicrokernelArchitecture {
    kernel: Microkernel,
    services: Vec<Box<dyn SystemService>, 16>,
    drivers: Vec<Box<dyn KernelDriver>, 8>,
}

// 微内核核心
pub struct Microkernel {
    scheduler: MinimalScheduler,
    memory_manager: MinimalMemoryManager,
    ipc_manager: IpcManager,
    interrupt_handler: InterruptHandler,
}

impl Microkernel {
    pub fn new() -> Self {
        Self {
            scheduler: MinimalScheduler::new(),
            memory_manager: MinimalMemoryManager::new(),
            ipc_manager: IpcManager::new(),
            interrupt_handler: InterruptHandler::new(),
        }
    }
    
    pub fn start(&mut self) -> Result<(), KernelError> {
        // 初始化核心组件
        self.memory_manager.initialize()?;
        self.ipc_manager.initialize()?;
        self.interrupt_handler.initialize()?;
        self.scheduler.initialize()?;
        
        // 启动调度器
        self.scheduler.start_scheduling();
        
        Ok(())
    }
    
    pub fn system_call(&mut self, call: SystemCall) -> Result<SystemCallResult, KernelError> {
        match call {
            SystemCall::CreateTask { entry_point, stack_size, priority } => {
                let task_id = self.scheduler.create_task(entry_point, stack_size, priority)?;
                Ok(SystemCallResult::TaskCreated(task_id))
            }
            
            SystemCall::SendMessage { target, message } => {
                self.ipc_manager.send_message(target, message)?;
                Ok(SystemCallResult::MessageSent)
            }
            
            SystemCall::ReceiveMessage { timeout } => {
                let message = self.ipc_manager.receive_message(timeout)?;
                Ok(SystemCallResult::MessageReceived(message))
            }
            
            SystemCall::AllocateMemory { size, alignment } => {
                let ptr = self.memory_manager.allocate(size, alignment)?;
                Ok(SystemCallResult::MemoryAllocated(ptr))
            }
            
            SystemCall::DeallocateMemory { ptr } => {
                self.memory_manager.deallocate(ptr)?;
                Ok(SystemCallResult::MemoryDeallocated)
            }
        }
    }
}

#[derive(Debug, Clone)]
pub enum SystemCall {
    CreateTask {
        entry_point: fn(),
        stack_size: usize,
        priority: u8,
    },
    SendMessage {
        target: u8,
        message: Message,
    },
    ReceiveMessage {
        timeout: u32,
    },
    AllocateMemory {
        size: usize,
        alignment: usize,
    },
    DeallocateMemory {
        ptr: *mut u8,
    },
}

#[derive(Debug, Clone)]
pub enum SystemCallResult {
    TaskCreated(u8),
    MessageSent,
    MessageReceived(Message),
    MemoryAllocated(*mut u8),
    MemoryDeallocated,
}

// 进程间通信管理器
pub struct IpcManager {
    message_queues: Vec<MessageQueue, 16>,
    shared_memory_regions: Vec<SharedMemoryRegion, 8>,
    semaphores: Vec<Semaphore, 16>,
}

#[derive(Debug, Clone)]
pub struct Message {
    pub sender: u8,
    pub receiver: u8,
    pub message_type: MessageType,
    pub payload: Vec<u8, 256>,
    pub timestamp: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum MessageType {
    Request,
    Response,
    Notification,
    Error,
}

pub struct MessageQueue {
    queue_id: u8,
    messages: Vec<Message, 32>,
    max_size: usize,
}

impl MessageQueue {
    pub fn new(queue_id: u8, max_size: usize) -> Self {
        Self {
            queue_id,
            messages: Vec::new(),
            max_size,
        }
    }
    
    pub fn enqueue(&mut self, message: Message) -> Result<(), IpcError> {
        if self.messages.len() >= self.max_size {
            return Err(IpcError::QueueFull);
        }
        
        self.messages.push(message).map_err(|_| IpcError::QueueFull)
    }
    
    pub fn dequeue(&mut self) -> Option<Message> {
        if self.messages.is_empty() {
            None
        } else {
            Some(self.messages.remove(0))
        }
    }
    
    pub fn is_empty(&self) -> bool {
        self.messages.is_empty()
    }
    
    pub fn is_full(&self) -> bool {
        self.messages.len() >= self.max_size
    }
}

pub struct SharedMemoryRegion {
    region_id: u8,
    base_address: *mut u8,
    size: usize,
    access_permissions: AccessPermissions,
    reference_count: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct AccessPermissions {
    pub read: bool,
    pub write: bool,
    pub execute: bool,
}

pub struct Semaphore {
    semaphore_id: u8,
    count: i32,
    max_count: i32,
    waiting_tasks: Vec<u8, 8>,
}

impl Semaphore {
    pub fn new(semaphore_id: u8, initial_count: i32, max_count: i32) -> Self {
        Self {
            semaphore_id,
            count: initial_count,
            max_count,
            waiting_tasks: Vec::new(),
        }
    }
    
    pub fn acquire(&mut self, task_id: u8) -> Result<(), IpcError> {
        if self.count > 0 {
            self.count -= 1;
            Ok(())
        } else {
            if self.waiting_tasks.len() >= 8 {
                return Err(IpcError::TooManyWaiters);
            }
            self.waiting_tasks.push(task_id).map_err(|_| IpcError::TooManyWaiters)?;
            Err(IpcError::WouldBlock)
        }
    }
    
    pub fn release(&mut self) -> Result<Option<u8>, IpcError> {
        if !self.waiting_tasks.is_empty() {
            let task_id = self.waiting_tasks.remove(0);
            Ok(Some(task_id))
        } else if self.count < self.max_count {
            self.count += 1;
            Ok(None)
        } else {
            Err(IpcError::SemaphoreOverflow)
        }
    }
}

impl IpcManager {
    pub fn new() -> Self {
        Self {
            message_queues: Vec::new(),
            shared_memory_regions: Vec::new(),
            semaphores: Vec::new(),
        }
    }
    
    pub fn initialize(&mut self) -> Result<(), KernelError> {
        // 初始化IPC机制
        Ok(())
    }
    
    pub fn send_message(&mut self, target: u8, message: Message) -> Result<(), IpcError> {
        // 查找目标消息队列
        for queue in &mut self.message_queues {
            if queue.queue_id == target {
                return queue.enqueue(message);
            }
        }
        
        Err(IpcError::InvalidTarget)
    }
    
    pub fn receive_message(&mut self, timeout: u32) -> Result<Message, IpcError> {
        // 简化实现：从第一个非空队列接收消息
        for queue in &mut self.message_queues {
            if let Some(message) = queue.dequeue() {
                return Ok(message);
            }
        }
        
        Err(IpcError::NoMessage)
    }
    
    pub fn create_message_queue(&mut self, queue_id: u8, max_size: usize) -> Result<(), IpcError> {
        if self.message_queues.len() >= 16 {
            return Err(IpcError::TooManyQueues);
        }
        
        let queue = MessageQueue::new(queue_id, max_size);
        self.message_queues.push(queue).map_err(|_| IpcError::TooManyQueues)
    }
}

// 系统服务接口
pub trait SystemService {
    fn service_id(&self) -> u8;
    fn initialize(&mut self) -> Result<(), ServiceError>;
    fn handle_request(&mut self, request: ServiceRequest) -> Result<ServiceResponse, ServiceError>;
    fn shutdown(&mut self) -> Result<(), ServiceError>;
}

#[derive(Debug, Clone)]
pub struct ServiceRequest {
    pub request_id: u32,
    pub service_id: u8,
    pub operation: ServiceOperation,
    pub parameters: Vec<u8, 256>,
}

#[derive(Debug, Clone)]
pub struct ServiceResponse {
    pub request_id: u32,
    pub status: ServiceStatus,
    pub data: Vec<u8, 256>,
}

#[derive(Debug, Clone, Copy)]
pub enum ServiceOperation {
    Read,
    Write,
    Configure,
    Status,
    Reset,
}

#[derive(Debug, Clone, Copy)]
pub enum ServiceStatus {
    Success,
    Error,
    InvalidRequest,
    ServiceUnavailable,
}

// 文件系统服务示例
pub struct FileSystemService {
    service_id: u8,
    storage_driver: Box<dyn StorageDriver>,
    file_table: Vec<FileEntry, 32>,
}

#[derive(Debug, Clone)]
pub struct FileEntry {
    pub file_id: u16,
    pub name: [u8; 32],
    pub size: u32,
    pub offset: u32,
    pub attributes: FileAttributes,
}

#[derive(Debug, Clone, Copy)]
pub struct FileAttributes {
    pub read_only: bool,
    pub hidden: bool,
    pub system: bool,
    pub created_time: u32,
    pub modified_time: u32,
}

pub trait StorageDriver {
    fn read(&mut self, offset: u32, buffer: &mut [u8]) -> Result<usize, StorageError>;
    fn write(&mut self, offset: u32, data: &[u8]) -> Result<usize, StorageError>;
    fn erase(&mut self, offset: u32, size: u32) -> Result<(), StorageError>;
    fn get_info(&self) -> StorageInfo;
}

#[derive(Debug, Clone)]
pub struct StorageInfo {
    pub total_size: u32,
    pub sector_size: u32,
    pub page_size: u32,
    pub erase_size: u32,
}

impl SystemService for FileSystemService {
    fn service_id(&self) -> u8 {
        self.service_id
    }
    
    fn initialize(&mut self) -> Result<(), ServiceError> {
        // 初始化文件系统
        self.load_file_table()?;
        Ok(())
    }
    
    fn handle_request(&mut self, request: ServiceRequest) -> Result<ServiceResponse, ServiceError> {
        match request.operation {
            ServiceOperation::Read => self.handle_read_request(request),
            ServiceOperation::Write => self.handle_write_request(request),
            ServiceOperation::Configure => self.handle_configure_request(request),
            ServiceOperation::Status => self.handle_status_request(request),
            ServiceOperation::Reset => self.handle_reset_request(request),
        }
    }
    
    fn shutdown(&mut self) -> Result<(), ServiceError> {
        // 保存文件表并关闭
        self.save_file_table()?;
        Ok(())
    }
}

impl FileSystemService {
    fn load_file_table(&mut self) -> Result<(), ServiceError> {
        // 从存储设备加载文件表
        Ok(())
    }
    
    fn save_file_table(&mut self) -> Result<(), ServiceError> {
        // 保存文件表到存储设备
        Ok(())
    }
    
    fn handle_read_request(&mut self, request: ServiceRequest) -> Result<ServiceResponse, ServiceError> {
        // 处理文件读取请求
        Ok(ServiceResponse {
            request_id: request.request_id,
            status: ServiceStatus::Success,
            data: Vec::new(),
        })
    }
    
    fn handle_write_request(&mut self, request: ServiceRequest) -> Result<ServiceResponse, ServiceError> {
        // 处理文件写入请求
        Ok(ServiceResponse {
            request_id: request.request_id,
            status: ServiceStatus::Success,
            data: Vec::new(),
        })
    }
    
    fn handle_configure_request(&mut self, request: ServiceRequest) -> Result<ServiceResponse, ServiceError> {
        // 处理配置请求
        Ok(ServiceResponse {
            request_id: request.request_id,
            status: ServiceStatus::Success,
            data: Vec::new(),
        })
    }
    
    fn handle_status_request(&mut self, request: ServiceRequest) -> Result<ServiceResponse, ServiceError> {
        // 处理状态查询请求
        Ok(ServiceResponse {
            request_id: request.request_id,
            status: ServiceStatus::Success,
            data: Vec::new(),
        })
    }
    
    fn handle_reset_request(&mut self, request: ServiceRequest) -> Result<ServiceResponse, ServiceError> {
        // 处理重置请求
        Ok(ServiceResponse {
            request_id: request.request_id,
            status: ServiceStatus::Success,
            data: Vec::new(),
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub enum KernelError {
    InitializationFailed,
    InvalidSystemCall,
    ResourceExhausted,
    PermissionDenied,
}

#[derive(Debug, Clone, Copy)]
pub enum IpcError {
    QueueFull,
    InvalidTarget,
    NoMessage,
    TooManyQueues,
    TooManyWaiters,
    WouldBlock,
    SemaphoreOverflow,
}

#[derive(Debug, Clone, Copy)]
pub enum ServiceError {
    InitializationFailed,
    InvalidRequest,
    ResourceUnavailable,
    OperationFailed,
}

#[derive(Debug, Clone, Copy)]
pub enum StorageError {
    ReadError,
    WriteError,
    EraseError,
    DeviceNotReady,
}
```

## 3. 状态机模式

### 3.1 有限状态机实现

```rust
// 状态机模式实现
pub trait State<Context, Event> {
    fn handle_event(&self, context: &mut Context, event: Event) -> Option<Box<dyn State<Context, Event>>>;
    fn entry_action(&self, context: &mut Context) {}
    fn exit_action(&self, context: &mut Context) {}
    fn state_name(&self) -> &'static str;
}

pub struct StateMachine<Context, Event> {
    current_state: Box<dyn State<Context, Event>>,
    context: Context,
    state_history: Vec<&'static str, 16>,
    transition_count: u32,
}

impl<Context, Event> StateMachine<Context, Event> {
    pub fn new(initial_state: Box<dyn State<Context, Event>>, context: Context) -> Self {
        let mut state_history = Vec::new();
        let _ = state_history.push(initial_state.state_name());
        
        Self {
            current_state: initial_state,
            context,
            state_history,
            transition_count: 0,
        }
    }
    
    pub fn handle_event(&mut self, event: Event) {
        if let Some(new_state) = self.current_state.handle_event(&mut self.context, event) {
            self.transition_to(new_state);
        }
    }
    
    fn transition_to(&mut self, new_state: Box<dyn State<Context, Event>>) {
        // 执行退出动作
        self.current_state.exit_action(&mut self.context);
        
        // 记录状态历史
        if self.state_history.len() >= 16 {
            self.state_history.remove(0);
        }
        let _ = self.state_history.push(new_state.state_name());
        
        // 切换状态
        self.current_state = new_state;
        self.transition_count += 1;
        
        // 执行进入动作
        self.current_state.entry_action(&mut self.context);
    }
    
    pub fn get_current_state_name(&self) -> &'static str {
        self.current_state.state_name()
    }
    
    pub fn get_context(&self) -> &Context {
        &self.context
    }
    
    pub fn get_context_mut(&mut self) -> &mut Context {
        &mut self.context
    }
    
    pub fn get_transition_count(&self) -> u32 {
        self.transition_count
    }
    
    pub fn get_state_history(&self) -> &[&'static str] {
        &self.state_history
    }
}

// 示例：LED控制器状态机
#[derive(Debug, Clone)]
pub struct LedController {
    pub led_pin: u8,
    pub brightness: u8,
    pub blink_period: u32,
    pub last_toggle_time: u32,
    pub error_count: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum LedEvent {
    TurnOn,
    TurnOff,
    StartBlink(u32), // 周期
    StopBlink,
    SetBrightness(u8),
    TimerTick(u32),
    Error,
    Reset,
}

// LED状态定义
pub struct LedOffState;
pub struct LedOnState;
pub struct LedBlinkingState;
pub struct LedErrorState;

impl State<LedController, LedEvent> for LedOffState {
    fn handle_event(&self, context: &mut LedController, event: LedEvent) -> Option<Box<dyn State<LedController, LedEvent>>> {
        match event {
            LedEvent::TurnOn => {
                Some(Box::new(LedOnState))
            }
            LedEvent::StartBlink(period) => {
                context.blink_period = period;
                Some(Box::new(LedBlinkingState))
            }
            LedEvent::Error => {
                context.error_count += 1;
                Some(Box::new(LedErrorState))
            }
            _ => None, // 忽略其他事件
        }
    }
    
    fn entry_action(&self, context: &mut LedController) {
        // 关闭LED
        self.set_led_output(context.led_pin, false);
    }
    
    fn state_name(&self) -> &'static str {
        "Off"
    }
}

impl State<LedController, LedEvent> for LedOnState {
    fn handle_event(&self, context: &mut LedController, event: LedEvent) -> Option<Box<dyn State<LedController, LedEvent>>> {
        match event {
            LedEvent::TurnOff => {
                Some(Box::new(LedOffState))
            }
            LedEvent::StartBlink(period) => {
                context.blink_period = period;
                Some(Box::new(LedBlinkingState))
            }
            LedEvent::SetBrightness(brightness) => {
                context.brightness = brightness;
                self.set_led_brightness(context.led_pin, brightness);
                None // 保持在当前状态
            }
            LedEvent::Error => {
                context.error_count += 1;
                Some(Box::new(LedErrorState))
            }
            _ => None,
        }
    }
    
    fn entry_action(&self, context: &mut LedController) {
        // 打开LED
        self.set_led_output(context.led_pin, true);
        self.set_led_brightness(context.led_pin, context.brightness);
    }
    
    fn state_name(&self) -> &'static str {
        "On"
    }
}

impl State<LedController, LedEvent> for LedBlinkingState {
    fn handle_event(&self, context: &mut LedController, event: LedEvent) -> Option<Box<dyn State<LedController, LedEvent>>> {
        match event {
            LedEvent::TurnOn => {
                Some(Box::new(LedOnState))
            }
            LedEvent::TurnOff => {
                Some(Box::new(LedOffState))
            }
            LedEvent::StopBlink => {
                Some(Box::new(LedOffState))
            }
            LedEvent::TimerTick(current_time) => {
                if current_time - context.last_toggle_time >= context.blink_period {
                    // 切换LED状态
                    let current_state = self.get_led_state(context.led_pin);
                    self.set_led_output(context.led_pin, !current_state);
                    context.last_toggle_time = current_time;
                }
                None // 保持在闪烁状态
            }
            LedEvent::Error => {
                context.error_count += 1;
                Some(Box::new(LedErrorState))
            }
            _ => None,
        }
    }
    
    fn entry_action(&self, context: &mut LedController) {
        context.last_toggle_time = self.get_current_time();
    }
    
    fn state_name(&self) -> &'static str {
        "Blinking"
    }
}

impl State<LedController, LedEvent> for LedErrorState {
    fn handle_event(&self, context: &mut LedController, event: LedEvent) -> Option<Box<dyn State<LedController, LedEvent>>> {
        match event {
            LedEvent::Reset => {
                context.error_count = 0;
                Some(Box::new(LedOffState))
            }
            _ => None, // 在错误状态下忽略其他事件
        }
    }
    
    fn entry_action(&self, context: &mut LedController) {
        // 错误指示：快速闪烁
        for _ in 0..6 {
            self.set_led_output(context.led_pin, true);
            self.delay_ms(100);
            self.set_led_output(context.led_pin, false);
            self.delay_ms(100);
        }
    }
    
    ## 4. 事件驱动架构

### 4.1 事件系统设计

```rust
// 事件驱动架构实现
use heapless::{Vec, spsc::{Producer, Consumer, Queue}};

// 事件定义
#[derive(Debug, Clone, Copy)]
pub struct Event {
    pub event_type: EventType,
    pub source: u8,
    pub timestamp: u32,
    pub data: EventData,
}

#[derive(Debug, Clone, Copy)]
pub enum EventType {
    System,
    Hardware,
    User,
    Timer,
    Communication,
    Error,
}

#[derive(Debug, Clone, Copy)]
pub union EventData {
    pub value: u32,
    pub bytes: [u8; 4],
    pub float: f32,
}

// 事件处理器接口
pub trait EventHandler {
    fn handle_event(&mut self, event: Event) -> Result<(), EventError>;
    fn can_handle(&self, event_type: EventType) -> bool;
    fn get_priority(&self) -> u8;
}

// 事件调度器
pub struct EventDispatcher {
    event_queue: Queue<Event, 64>,
    handlers: Vec<Box<dyn EventHandler>, 16>,
    statistics: EventStatistics,
}

#[derive(Debug, Clone)]
pub struct EventStatistics {
    pub events_processed: u32,
    pub events_dropped: u32,
    pub processing_time_total: u32,
    pub max_processing_time: u32,
    pub handler_errors: u32,
}

impl EventDispatcher {
    pub fn new() -> (Self, Producer<Event, 64>, Consumer<Event, 64>) {
        let queue = Queue::new();
        let (producer, consumer) = queue.split();
        
        let dispatcher = Self {
            event_queue: Queue::new(),
            handlers: Vec::new(),
            statistics: EventStatistics {
                events_processed: 0,
                events_dropped: 0,
                processing_time_total: 0,
                max_processing_time: 0,
                handler_errors: 0,
            },
        };
        
        (dispatcher, producer, consumer)
    }
    
    pub fn register_handler(&mut self, handler: Box<dyn EventHandler>) -> Result<(), EventError> {
        if self.handlers.len() >= 16 {
            return Err(EventError::TooManyHandlers);
        }
        
        self.handlers.push(handler).map_err(|_| EventError::TooManyHandlers)
    }
    
    pub fn dispatch_events(&mut self, consumer: &mut Consumer<Event, 64>) {
        while let Some(event) = consumer.dequeue() {
            self.process_event(event);
        }
    }
    
    fn process_event(&mut self, event: Event) {
        let start_time = self.get_timestamp();
        
        // 按优先级排序处理器
        self.handlers.sort_by_key(|h| core::cmp::Reverse(h.get_priority()));
        
        let mut handled = false;
        for handler in &mut self.handlers {
            if handler.can_handle(event.event_type) {
                match handler.handle_event(event) {
                    Ok(_) => {
                        handled = true;
                        break;
                    }
                    Err(_) => {
                        self.statistics.handler_errors += 1;
                    }
                }
            }
        }
        
        if !handled {
            self.statistics.events_dropped += 1;
        } else {
            self.statistics.events_processed += 1;
        }
        
        let processing_time = self.get_timestamp() - start_time;
        self.statistics.processing_time_total += processing_time;
        if processing_time > self.statistics.max_processing_time {
            self.statistics.max_processing_time = processing_time;
        }
    }
    
    fn get_timestamp(&self) -> u32 {
        // 获取当前时间戳
        0
    }
    
    pub fn get_statistics(&self) -> &EventStatistics {
        &self.statistics
    }
}

#[derive(Debug, Clone, Copy)]
pub enum EventError {
    TooManyHandlers,
    HandlerNotFound,
    ProcessingError,
    QueueFull,
}

// 示例事件处理器：LED控制器
pub struct LedEventHandler {
    led_pin: u8,
    priority: u8,
}

impl LedEventHandler {
    pub fn new(led_pin: u8, priority: u8) -> Self {
        Self { led_pin, priority }
    }
}

impl EventHandler for LedEventHandler {
    fn handle_event(&mut self, event: Event) -> Result<(), EventError> {
        match event.event_type {
            EventType::User => {
                unsafe {
                    match event.data.value {
                        1 => self.turn_on_led(),
                        0 => self.turn_off_led(),
                        _ => return Err(EventError::ProcessingError),
                    }
                }
                Ok(())
            }
            _ => Err(EventError::ProcessingError),
        }
    }
    
    fn can_handle(&self, event_type: EventType) -> bool {
        matches!(event_type, EventType::User)
    }
    
    fn get_priority(&self) -> u8 {
        self.priority
    }
}

impl LedEventHandler {
    fn turn_on_led(&self) {
        // 打开LED
    }
    
    fn turn_off_led(&self) {
        // 关闭LED
    }
}

// 温度监控事件处理器
pub struct TemperatureEventHandler {
    threshold: f32,
    priority: u8,
    alarm_callback: Option<fn(f32)>,
}

impl TemperatureEventHandler {
    pub fn new(threshold: f32, priority: u8) -> Self {
        Self {
            threshold,
            priority,
            alarm_callback: None,
        }
    }
    
    pub fn set_alarm_callback(&mut self, callback: fn(f32)) {
        self.alarm_callback = Some(callback);
    }
}

impl EventHandler for TemperatureEventHandler {
    fn handle_event(&mut self, event: Event) -> Result<(), EventError> {
        match event.event_type {
            EventType::Hardware => {
                unsafe {
                    let temperature = event.data.float;
                    if temperature > self.threshold {
                        if let Some(callback) = self.alarm_callback {
                            callback(temperature);
                        }
                    }
                }
                Ok(())
            }
            _ => Err(EventError::ProcessingError),
        }
    }
    
    fn can_handle(&self, event_type: EventType) -> bool {
        matches!(event_type, EventType::Hardware)
    }
    
    fn get_priority(&self) -> u8 {
        self.priority
    }
}
```

### 4.2 发布-订阅模式

```rust
// 发布-订阅模式实现
pub struct EventBus {
    subscribers: Vec<Subscription, 32>,
    event_buffer: Vec<Event, 128>,
    next_subscription_id: u32,
}

#[derive(Debug, Clone)]
pub struct Subscription {
    pub id: u32,
    pub event_type: EventType,
    pub callback: fn(Event),
    pub filter: Option<EventFilter>,
    pub active: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct EventFilter {
    pub source_mask: u8,
    pub data_range: Option<(u32, u32)>,
    pub time_window: Option<(u32, u32)>,
}

impl EventBus {
    pub fn new() -> Self {
        Self {
            subscribers: Vec::new(),
            event_buffer: Vec::new(),
            next_subscription_id: 1,
        }
    }
    
    pub fn subscribe(&mut self, event_type: EventType, callback: fn(Event)) -> Result<u32, EventError> {
        self.subscribe_with_filter(event_type, callback, None)
    }
    
    pub fn subscribe_with_filter(
        &mut self,
        event_type: EventType,
        callback: fn(Event),
        filter: Option<EventFilter>,
    ) -> Result<u32, EventError> {
        if self.subscribers.len() >= 32 {
            return Err(EventError::TooManyHandlers);
        }
        
        let subscription = Subscription {
            id: self.next_subscription_id,
            event_type,
            callback,
            filter,
            active: true,
        };
        
        self.subscribers.push(subscription).map_err(|_| EventError::TooManyHandlers)?;
        
        let id = self.next_subscription_id;
        self.next_subscription_id += 1;
        Ok(id)
    }
    
    pub fn unsubscribe(&mut self, subscription_id: u32) -> Result<(), EventError> {
        if let Some(pos) = self.subscribers.iter().position(|s| s.id == subscription_id) {
            self.subscribers.remove(pos);
            Ok(())
        } else {
            Err(EventError::HandlerNotFound)
        }
    }
    
    pub fn publish(&mut self, event: Event) {
        // 缓存事件
        if self.event_buffer.push(event).is_err() {
            // 缓冲区满，移除最旧的事件
            self.event_buffer.remove(0);
            let _ = self.event_buffer.push(event);
        }
        
        // 立即分发给订阅者
        self.dispatch_to_subscribers(event);
    }
    
    fn dispatch_to_subscribers(&self, event: Event) {
        for subscription in &self.subscribers {
            if subscription.active && 
               subscription.event_type == event.event_type &&
               self.passes_filter(&event, subscription.filter) {
                (subscription.callback)(event);
            }
        }
    }
    
    fn passes_filter(&self, event: &Event, filter: Option<EventFilter>) -> bool {
        if let Some(filter) = filter {
            // 检查源掩码
            if filter.source_mask != 0 && (event.source & filter.source_mask) == 0 {
                return false;
            }
            
            // 检查数据范围
            if let Some((min, max)) = filter.data_range {
                unsafe {
                    if event.data.value < min || event.data.value > max {
                        return false;
                    }
                }
            }
            
            // 检查时间窗口
            if let Some((start, end)) = filter.time_window {
                if event.timestamp < start || event.timestamp > end {
                    return false;
                }
            }
        }
        
        true
    }
    
    pub fn set_subscription_active(&mut self, subscription_id: u32, active: bool) -> Result<(), EventError> {
        if let Some(subscription) = self.subscribers.iter_mut().find(|s| s.id == subscription_id) {
            subscription.active = active;
            Ok(())
        } else {
            Err(EventError::HandlerNotFound)
        }
    }
    
    pub fn get_event_history(&self, event_type: EventType, count: usize) -> Vec<Event, 32> {
        let mut history = Vec::new();
        let mut found = 0;
        
        for event in self.event_buffer.iter().rev() {
            if event.event_type == event_type {
                if history.push(*event).is_err() || found >= count {
                    break;
                }
                found += 1;
            }
        }
        
        history
    }
}

// 使用示例
pub fn event_bus_example() {
    let mut event_bus = EventBus::new();
    
    // 订阅温度事件
    let temp_subscription = event_bus.subscribe(EventType::Hardware, |event| {
        unsafe {
            println!("Temperature: {:.2}°C", event.data.float);
        }
    }).unwrap();
    
    // 订阅用户事件（带过滤器）
    let user_filter = EventFilter {
        source_mask: 0x01, // 只接受来源为1的事件
        data_range: Some((0, 100)),
        time_window: None,
    };
    
    let user_subscription = event_bus.subscribe_with_filter(
        EventType::User,
        |event| {
            unsafe {
                println!("User input: {}", event.data.value);
            }
        },
        Some(user_filter),
    ).unwrap();
    
    // 发布事件
    let temp_event = Event {
        event_type: EventType::Hardware,
        source: 1,
        timestamp: 1000,
        data: EventData { float: 25.5 },
    };
    
    event_bus.publish(temp_event);
    
    let user_event = Event {
        event_type: EventType::User,
        source: 1,
        timestamp: 1001,
        data: EventData { value: 42 },
    };
    
    event_bus.publish(user_event);
}
```

## 5. 总结

嵌入式软件架构是系统成功的关键因素，良好的架构设计能够提高系统的可维护性、可扩展性和可靠性。

### 关键要点

1. **架构原则**: 遵循SOLID原则和其他设计原则
2. **分层设计**: 采用分层架构管理复杂性
3. **状态管理**: 使用状态机模式处理复杂的状态转换
4. **事件驱动**: 实现松耦合的事件驱动架构
5. **模块化**: 采用组件化设计提高代码复用性

### 设计策略

- **关注点分离**: 将不同的关注点分离到不同的层次和模块
- **接口抽象**: 定义清晰的接口和抽象层
- **依赖管理**: 合理管理模块间的依赖关系
- **错误处理**: 建立完善的错误处理机制
- **性能优化**: 在架构层面考虑性能优化

### 实践建议

- 从简单的架构开始，逐步演进
- 使用设计模式解决常见问题
- 建立完善的测试框架
- 定期进行架构评审和重构
- 学习和应用最佳实践

### 下一步

完成软件架构学习后，建议继续学习：
- [GPIO控制实践](../../04-gpio-control/README.md)
- [RTOS集成](../../09-rtos-integration/README.md)
- [系统集成](../../20-system-integration/README.md)

// 状态机辅助方法
impl LedOffState {
    fn set_led_output(&self, pin: u8, state: bool) {
        // 设置LED输出
    }
}

impl LedOnState {
    fn set_led_output(&self, pin: u8, state: bool) {
        // 设置LED输出
    }
    
    fn set_led_brightness(&self, pin: u8, brightness: u8) {
        // 设置LED亮度（PWM）
    }
}

impl LedBlinkingState {
    fn get_led_state(&self, pin: u8) -> bool {
        // 获取LED当前状态
        false
    }
    
    fn set_led_output(&self, pin: u8, state: bool) {
        // 设置LED输出
    }
    
    fn get_current_time(&self) -> u32 {
        // 获取当前时间
        0
    }
}

impl LedErrorState {
    fn set_led_output(&self, pin: u8, state: bool) {
        // 设置LED输出
    }
    
    fn delay_ms(&self, ms: u32) {
        // 延时函数
    }
}

// 使用示例
pub fn led_state_machine_example() {
    let led_controller = LedController {
        led_pin: 13,
        brightness: 255,
        blink_period: 500,
        last_toggle_time: 0,
        error_count: 0,
    };
    
    let mut led_sm = StateMachine::new(
        Box::new(LedOffState),
        led_controller,
    );
    
    // 处理事件序列
    led_sm.handle_event(LedEvent::TurnOn);
    assert_eq!(led_sm.get_current_state_name(), "On");
    
    led_sm.handle_event(LedEvent::StartBlink(1000));
    assert_eq!(led_sm.get_current_state_name(), "Blinking");
    
    led_sm.handle_event(LedEvent::Error);
    assert_eq!(led_sm.get_current_state_name(), "Error");
    
    led_sm.handle_event(LedEvent::Reset);
    assert_eq!(led_sm.get_current_state_name(), "Off");
}
```

### 3.2 分层状态机

```rust
// 分层状态机实现
pub trait HierarchicalState<Context, Event> {
    fn handle_event(&self, context: &mut Context, event: Event) -> StateTransition<Context, Event>;
    fn entry_action(&self, context: &mut Context) {}
    fn exit_action(&self, context: &mut Context) {}
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<Context, Event>>>;
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<Context, Event>>, 8>;
    fn state_name(&self) -> &'static str;
}

#[derive(Debug)]
pub enum StateTransition<Context, Event> {
    None,                                                           // 无转换
    Internal,                                                       // 内部转换
    Transition(Box<dyn HierarchicalState<Context, Event>>),        // 状态转换
    ToParent,                                                       // 转换到父状态
    ToChild(Box<dyn HierarchicalState<Context, Event>>),          // 转换到子状态
}

pub struct HierarchicalStateMachine<Context, Event> {
    current_state: Box<dyn HierarchicalState<Context, Event>>,
    state_stack: Vec<Box<dyn HierarchicalState<Context, Event>>, 8>,
    context: Context,
}

impl<Context, Event> HierarchicalStateMachine<Context, Event> {
    pub fn new(initial_state: Box<dyn HierarchicalState<Context, Event>>, context: Context) -> Self {
        Self {
            current_state: initial_state,
            state_stack: Vec::new(),
            context,
        }
    }
    
    pub fn handle_event(&mut self, event: Event) {
        let transition = self.current_state.handle_event(&mut self.context, event);
        
        match transition {
            StateTransition::None => {
                // 无转换，检查父状态是否能处理
                if let Some(parent) = self.current_state.parent_state() {
                    let parent_transition = parent.handle_event(&mut self.context, event);
                    self.process_transition(parent_transition);
                }
            }
            _ => {
                self.process_transition(transition);
            }
        }
    }
    
    fn process_transition(&mut self, transition: StateTransition<Context, Event>) {
        match transition {
            StateTransition::None | StateTransition::Internal => {
                // 无需状态切换
            }
            StateTransition::Transition(new_state) => {
                self.transition_to(new_state);
            }
            StateTransition::ToParent => {
                if let Some(parent) = self.current_state.parent_state() {
                    self.transition_to(parent);
                }
            }
            StateTransition::ToChild(child_state) => {
                // 保存当前状态到栈中
                let current = core::mem::replace(&mut self.current_state, child_state);
                if self.state_stack.push(current).is_err() {
                    // 栈满，无法保存状态
                }
                self.current_state.entry_action(&mut self.context);
            }
        }
    }
    
    fn transition_to(&mut self, new_state: Box<dyn HierarchicalState<Context, Event>>) {
        self.current_state.exit_action(&mut self.context);
        self.current_state = new_state;
        self.current_state.entry_action(&mut self.context);
    }
}

// 示例：设备控制器分层状态机
#[derive(Debug, Clone)]
pub struct DeviceController {
    pub power_state: PowerState,
    pub operation_mode: OperationMode,
    pub error_code: u32,
    pub temperature: f32,
    pub voltage: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum PowerState {
    Off,
    Standby,
    Active,
}

#[derive(Debug, Clone, Copy)]
pub enum OperationMode {
    Manual,
    Automatic,
    Calibration,
}

#[derive(Debug, Clone, Copy)]
pub enum DeviceEvent {
    PowerOn,
    PowerOff,
    StartOperation,
    StopOperation,
    EnterCalibration,
    ExitCalibration,
    TemperatureAlert(f32),
    VoltageAlert(f32),
    Error(u32),
    Reset,
}

// 顶层状态：设备状态
pub struct DeviceState;

impl HierarchicalState<DeviceController, DeviceEvent> for DeviceState {
    fn handle_event(&self, context: &mut DeviceController, event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        match event {
            DeviceEvent::PowerOff => {
                context.power_state = PowerState::Off;
                StateTransition::Transition(Box::new(PowerOffState))
            }
            DeviceEvent::Error(code) => {
                context.error_code = code;
                StateTransition::Transition(Box::new(ErrorState))
            }
            _ => StateTransition::None,
        }
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        None // 顶层状态
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        let mut children = Vec::new();
        let _ = children.push(Box::new(PowerOffState) as Box<dyn HierarchicalState<DeviceController, DeviceEvent>>);
        let _ = children.push(Box::new(PowerOnState) as Box<dyn HierarchicalState<DeviceController, DeviceEvent>>);
        let _ = children.push(Box::new(ErrorState) as Box<dyn HierarchicalState<DeviceController, DeviceEvent>>);
        children
    }
    
    fn state_name(&self) -> &'static str {
        "Device"
    }
}

// 电源关闭状态
pub struct PowerOffState;

impl HierarchicalState<DeviceController, DeviceEvent> for PowerOffState {
    fn handle_event(&self, context: &mut DeviceController, event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        match event {
            DeviceEvent::PowerOn => {
                context.power_state = PowerState::Standby;
                StateTransition::Transition(Box::new(PowerOnState))
            }
            _ => StateTransition::None,
        }
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        Some(Box::new(DeviceState))
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        Vec::new() // 叶子状态
    }
    
    fn state_name(&self) -> &'static str {
        "PowerOff"
    }
}

// 电源开启状态（复合状态）
pub struct PowerOnState;

impl HierarchicalState<DeviceController, DeviceEvent> for PowerOnState {
    fn handle_event(&self, context: &mut DeviceController, event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        match event {
            DeviceEvent::StartOperation => {
                context.power_state = PowerState::Active;
                StateTransition::ToChild(Box::new(ActiveState))
            }
            _ => StateTransition::None,
        }
    }
    
    fn entry_action(&self, context: &mut DeviceController) {
        context.power_state = PowerState::Standby;
        // 初始化硬件
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        Some(Box::new(DeviceState))
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        let mut children = Vec::new();
        let _ = children.push(Box::new(StandbyState) as Box<dyn HierarchicalState<DeviceController, DeviceEvent>>);
        let _ = children.push(Box::new(ActiveState) as Box<dyn HierarchicalState<DeviceController, DeviceEvent>>);
        children
    }
    
    fn state_name(&self) -> &'static str {
        "PowerOn"
    }
}

// 待机状态
pub struct StandbyState;

impl HierarchicalState<DeviceController, DeviceEvent> for StandbyState {
    fn handle_event(&self, context: &mut DeviceController, event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        match event {
            DeviceEvent::StartOperation => {
                StateTransition::Transition(Box::new(ActiveState))
            }
            _ => StateTransition::None,
        }
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        Some(Box::new(PowerOnState))
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        Vec::new()
    }
    
    fn state_name(&self) -> &'static str {
        "Standby"
    }
}

// 活动状态（复合状态）
pub struct ActiveState;

impl HierarchicalState<DeviceController, DeviceEvent> for ActiveState {
    fn handle_event(&self, context: &mut DeviceController, event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        match event {
            DeviceEvent::StopOperation => {
                StateTransition::Transition(Box::new(StandbyState))
            }
            DeviceEvent::EnterCalibration => {
                context.operation_mode = OperationMode::Calibration;
                StateTransition::ToChild(Box::new(CalibrationState))
            }
            _ => StateTransition::None,
        }
    }
    
    fn entry_action(&self, context: &mut DeviceController) {
        context.power_state = PowerState::Active;
        context.operation_mode = OperationMode::Manual;
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        Some(Box::new(PowerOnState))
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        let mut children = Vec::new();
        let _ = children.push(Box::new(ManualState) as Box<dyn HierarchicalState<DeviceController, DeviceEvent>>);
        let _ = children.push(Box::new(AutomaticState) as Box<dyn HierarchicalState<DeviceController, DeviceEvent>>);
        let _ = children.push(Box::new(CalibrationState) as Box<dyn HierarchicalState<DeviceController, DeviceEvent>>);
        children
    }
    
    fn state_name(&self) -> &'static str {
        "Active"
    }
}

// 手动模式状态
pub struct ManualState;

impl HierarchicalState<DeviceController, DeviceEvent> for ManualState {
    fn handle_event(&self, _context: &mut DeviceController, _event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        StateTransition::None
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        Some(Box::new(ActiveState))
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        Vec::new()
    }
    
    fn state_name(&self) -> &'static str {
        "Manual"
    }
}

// 自动模式状态
pub struct AutomaticState;

impl HierarchicalState<DeviceController, DeviceEvent> for AutomaticState {
    fn handle_event(&self, _context: &mut DeviceController, _event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        StateTransition::None
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        Some(Box::new(ActiveState))
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        Vec::new()
    }
    
    fn state_name(&self) -> &'static str {
        "Automatic"
    }
}

// 校准状态
pub struct CalibrationState;

impl HierarchicalState<DeviceController, DeviceEvent> for CalibrationState {
    fn handle_event(&self, context: &mut DeviceController, event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        match event {
            DeviceEvent::ExitCalibration => {
                context.operation_mode = OperationMode::Manual;
                StateTransition::ToParent
            }
            _ => StateTransition::None,
        }
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        Some(Box::new(ActiveState))
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        Vec::new()
    }
    
    fn state_name(&self) -> &'static str {
        "Calibration"
    }
}

// 错误状态
pub struct ErrorState;

impl HierarchicalState<DeviceController, DeviceEvent> for ErrorState {
    fn handle_event(&self, context: &mut DeviceController, event: DeviceEvent) -> StateTransition<DeviceController, DeviceEvent> {
        match event {
            DeviceEvent::Reset => {
                context.error_code = 0;
                StateTransition::Transition(Box::new(PowerOffState))
            }
            _ => StateTransition::None,
        }
    }
    
    fn parent_state(&self) -> Option<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>> {
        Some(Box::new(DeviceState))
    }
    
    fn child_states(&self) -> Vec<Box<dyn HierarchicalState<DeviceController, DeviceEvent>>, 8> {
        Vec::new()
    }
    
    fn state_name(&self) -> &'static str {
        "Error"
    }
}
```