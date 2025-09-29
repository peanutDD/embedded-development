# RTOS中断处理

中断处理是实时操作系统的核心功能之一，它决定了系统对外部事件的响应速度和实时性能。本文档详细介绍RTOS环境下的中断处理机制、最佳实践和性能优化策略。

## 目录

1. [中断基础概念](#中断基础概念)
2. [中断控制器管理](#中断控制器管理)
3. [中断服务程序](#中断服务程序)
4. [中断优先级管理](#中断优先级管理)
5. [中断与任务交互](#中断与任务交互)
6. [中断性能优化](#中断性能优化)
7. [中断调试和监控](#中断调试和监控)
8. [最佳实践](#最佳实践)

## 中断基础概念

### 中断类型定义

```rust
/// 中断类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptType {
    /// 外部中断
    External,
    /// 定时器中断
    Timer,
    /// 串口中断
    Uart,
    /// SPI中断
    Spi,
    /// I2C中断
    I2c,
    /// ADC中断
    Adc,
    /// DMA中断
    Dma,
    /// 系统中断
    System,
    /// 用户自定义中断
    Custom(u8),
}

/// 中断状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptState {
    /// 禁用
    Disabled,
    /// 启用
    Enabled,
    /// 挂起
    Pending,
    /// 正在处理
    Active,
}

/// 中断优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct InterruptPriority(pub u8);

impl InterruptPriority {
    pub const HIGHEST: Self = Self(0);
    pub const HIGH: Self = Self(64);
    pub const MEDIUM: Self = Self(128);
    pub const LOW: Self = Self(192);
    pub const LOWEST: Self = Self(255);
    
    pub fn new(priority: u8) -> Self {
        Self(priority)
    }
    
    pub fn value(&self) -> u8 {
        self.0
    }
}
```

### 中断描述符

```rust
use heapless::{String, Vec};
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// 中断描述符
#[derive(Debug)]
pub struct InterruptDescriptor {
    /// 中断号
    pub irq_number: u32,
    /// 中断类型
    pub interrupt_type: InterruptType,
    /// 中断优先级
    pub priority: InterruptPriority,
    /// 中断状态
    pub state: InterruptState,
    /// 中断名称
    pub name: String<32>,
    /// 中断处理函数指针
    pub handler: Option<fn()>,
    /// 中断统计信息
    pub stats: InterruptStats,
}

/// 中断统计信息
#[derive(Debug, Default)]
pub struct InterruptStats {
    /// 中断触发次数
    pub trigger_count: AtomicU64,
    /// 总处理时间（微秒）
    pub total_processing_time: AtomicU64,
    /// 最大处理时间（微秒）
    pub max_processing_time: AtomicU32,
    /// 最小处理时间（微秒）
    pub min_processing_time: AtomicU32,
    /// 平均处理时间（微秒）
    pub avg_processing_time: AtomicU32,
    /// 中断嵌套深度
    pub max_nesting_depth: AtomicU32,
    /// 中断丢失次数
    pub missed_count: AtomicU64,
}

impl InterruptStats {
    pub fn new() -> Self {
        Self {
            trigger_count: AtomicU64::new(0),
            total_processing_time: AtomicU64::new(0),
            max_processing_time: AtomicU32::new(0),
            min_processing_time: AtomicU32::new(u32::MAX),
            avg_processing_time: AtomicU32::new(0),
            max_nesting_depth: AtomicU32::new(0),
            missed_count: AtomicU64::new(0),
        }
    }
    
    /// 记录中断处理时间
    pub fn record_processing_time(&self, processing_time: u32) {
        let count = self.trigger_count.fetch_add(1, Ordering::Relaxed);
        let total = self.total_processing_time.fetch_add(processing_time as u64, Ordering::Relaxed);
        
        // 更新最大处理时间
        let mut current_max = self.max_processing_time.load(Ordering::Relaxed);
        while processing_time > current_max {
            match self.max_processing_time.compare_exchange_weak(
                current_max, processing_time, Ordering::Relaxed, Ordering::Relaxed
            ) {
                Ok(_) => break,
                Err(x) => current_max = x,
            }
        }
        
        // 更新最小处理时间
        let mut current_min = self.min_processing_time.load(Ordering::Relaxed);
        while processing_time < current_min {
            match self.min_processing_time.compare_exchange_weak(
                current_min, processing_time, Ordering::Relaxed, Ordering::Relaxed
            ) {
                Ok(_) => break,
                Err(x) => current_min = x,
            }
        }
        
        // 更新平均处理时间
        let avg = (total + processing_time as u64) / (count + 1);
        self.avg_processing_time.store(avg as u32, Ordering::Relaxed);
    }
    
    /// 记录中断丢失
    pub fn record_missed_interrupt(&self) {
        self.missed_count.fetch_add(1, Ordering::Relaxed);
    }
    
    /// 更新嵌套深度
    pub fn update_nesting_depth(&self, depth: u32) {
        let mut current_max = self.max_nesting_depth.load(Ordering::Relaxed);
        while depth > current_max {
            match self.max_nesting_depth.compare_exchange_weak(
                current_max, depth, Ordering::Relaxed, Ordering::Relaxed
            ) {
                Ok(_) => break,
                Err(x) => current_max = x,
            }
        }
    }
}
```

## 中断控制器管理

### 中断控制器

```rust
use heapless::FnvIndexMap;
use cortex_m::interrupt;

/// 中断控制器
pub struct InterruptController {
    /// 中断描述符表
    descriptors: FnvIndexMap<u32, InterruptDescriptor, 64>,
    /// 全局中断使能状态
    global_enabled: bool,
    /// 中断嵌套深度
    nesting_depth: u32,
    /// 中断控制器统计
    stats: ControllerStats,
}

/// 控制器统计信息
#[derive(Debug, Default)]
pub struct ControllerStats {
    /// 总中断次数
    pub total_interrupts: AtomicU64,
    /// 中断处理总时间
    pub total_processing_time: AtomicU64,
    /// 最大嵌套深度
    pub max_nesting_depth: AtomicU32,
    /// 中断延迟统计
    pub interrupt_latency: InterruptLatencyStats,
}

/// 中断延迟统计
#[derive(Debug, Default)]
pub struct InterruptLatencyStats {
    /// 最大延迟（微秒）
    pub max_latency: AtomicU32,
    /// 最小延迟（微秒）
    pub min_latency: AtomicU32,
    /// 平均延迟（微秒）
    pub avg_latency: AtomicU32,
    /// 延迟测量次数
    pub measurement_count: AtomicU64,
}

impl InterruptController {
    /// 创建新的中断控制器
    pub fn new() -> Self {
        Self {
            descriptors: FnvIndexMap::new(),
            global_enabled: false,
            nesting_depth: 0,
            stats: ControllerStats::default(),
        }
    }
    
    /// 注册中断处理程序
    pub fn register_handler(
        &mut self,
        irq_number: u32,
        interrupt_type: InterruptType,
        priority: InterruptPriority,
        name: &str,
        handler: fn(),
    ) -> Result<(), InterruptError> {
        if self.descriptors.contains_key(&irq_number) {
            return Err(InterruptError::AlreadyRegistered);
        }
        
        let descriptor = InterruptDescriptor {
            irq_number,
            interrupt_type,
            priority,
            state: InterruptState::Disabled,
            name: String::from(name),
            handler: Some(handler),
            stats: InterruptStats::new(),
        };
        
        self.descriptors.insert(irq_number, descriptor)
            .map_err(|_| InterruptError::TableFull)?;
        
        // 配置硬件中断控制器
        self.configure_hardware_interrupt(irq_number, priority)?;
        
        Ok(())
    }
    
    /// 启用中断
    pub fn enable_interrupt(&mut self, irq_number: u32) -> Result<(), InterruptError> {
        if let Some(descriptor) = self.descriptors.get_mut(&irq_number) {
            descriptor.state = InterruptState::Enabled;
            
            // 启用硬件中断
            self.enable_hardware_interrupt(irq_number)?;
            
            Ok(())
        } else {
            Err(InterruptError::NotRegistered)
        }
    }
    
    /// 禁用中断
    pub fn disable_interrupt(&mut self, irq_number: u32) -> Result<(), InterruptError> {
        if let Some(descriptor) = self.descriptors.get_mut(&irq_number) {
            descriptor.state = InterruptState::Disabled;
            
            // 禁用硬件中断
            self.disable_hardware_interrupt(irq_number)?;
            
            Ok(())
        } else {
            Err(InterruptError::NotRegistered)
        }
    }
    
    /// 设置中断优先级
    pub fn set_priority(
        &mut self,
        irq_number: u32,
        priority: InterruptPriority,
    ) -> Result<(), InterruptError> {
        if let Some(descriptor) = self.descriptors.get_mut(&irq_number) {
            descriptor.priority = priority;
            
            // 更新硬件优先级
            self.set_hardware_priority(irq_number, priority)?;
            
            Ok(())
        } else {
            Err(InterruptError::NotRegistered)
        }
    }
    
    /// 触发软件中断
    pub fn trigger_software_interrupt(&mut self, irq_number: u32) -> Result<(), InterruptError> {
        if let Some(descriptor) = self.descriptors.get(&irq_number) {
            if descriptor.state == InterruptState::Enabled {
                // 触发软件中断
                self.trigger_hardware_interrupt(irq_number)?;
                Ok(())
            } else {
                Err(InterruptError::Disabled)
            }
        } else {
            Err(InterruptError::NotRegistered)
        }
    }
    
    /// 处理中断
    pub fn handle_interrupt(&mut self, irq_number: u32) -> Result<(), InterruptError> {
        let start_time = self.get_current_time();
        
        // 增加嵌套深度
        self.nesting_depth += 1;
        
        let result = if let Some(descriptor) = self.descriptors.get_mut(&irq_number) {
            // 更新状态
            descriptor.state = InterruptState::Active;
            
            // 更新嵌套深度统计
            descriptor.stats.update_nesting_depth(self.nesting_depth);
            self.stats.max_nesting_depth.store(
                self.stats.max_nesting_depth.load(Ordering::Relaxed).max(self.nesting_depth),
                Ordering::Relaxed,
            );
            
            // 调用中断处理函数
            if let Some(handler) = descriptor.handler {
                handler();
                Ok(())
            } else {
                Err(InterruptError::NoHandler)
            }
        } else {
            Err(InterruptError::NotRegistered)
        };
        
        // 计算处理时间
        let end_time = self.get_current_time();
        let processing_time = end_time - start_time;
        
        // 更新统计信息
        if let Some(descriptor) = self.descriptors.get_mut(&irq_number) {
            descriptor.stats.record_processing_time(processing_time);
            descriptor.state = InterruptState::Enabled;
        }
        
        self.stats.total_interrupts.fetch_add(1, Ordering::Relaxed);
        self.stats.total_processing_time.fetch_add(processing_time as u64, Ordering::Relaxed);
        
        // 减少嵌套深度
        self.nesting_depth -= 1;
        
        result
    }
    
    /// 全局启用中断
    pub fn enable_global_interrupts(&mut self) {
        self.global_enabled = true;
        unsafe {
            interrupt::enable();
        }
    }
    
    /// 全局禁用中断
    pub fn disable_global_interrupts(&mut self) {
        self.global_enabled = false;
        interrupt::disable();
    }
    
    /// 获取中断统计信息
    pub fn get_interrupt_stats(&self, irq_number: u32) -> Option<&InterruptStats> {
        self.descriptors.get(&irq_number).map(|desc| &desc.stats)
    }
    
    /// 获取控制器统计信息
    pub fn get_controller_stats(&self) -> &ControllerStats {
        &self.stats
    }
    
    /// 获取所有中断信息
    pub fn get_all_interrupts(&self) -> Vec<(u32, &InterruptDescriptor), 64> {
        let mut interrupts = Vec::new();
        
        for (irq, desc) in &self.descriptors {
            let _ = interrupts.push((*irq, desc));
        }
        
        interrupts
    }
    
    // 硬件相关的私有方法
    fn configure_hardware_interrupt(
        &self,
        irq_number: u32,
        priority: InterruptPriority,
    ) -> Result<(), InterruptError> {
        // 这里应该调用具体的硬件配置函数
        // 例如：NVIC配置、中断控制器寄存器设置等
        
        // 示例实现（需要根据具体硬件调整）
        #[cfg(feature = "cortex-m")]
        {
            use cortex_m::peripheral::NVIC;
            
            // 设置中断优先级
            unsafe {
                let mut nvic = cortex_m::Peripherals::steal().NVIC;
                nvic.set_priority(irq_number as u8, priority.value());
            }
        }
        
        Ok(())
    }
    
    fn enable_hardware_interrupt(&self, irq_number: u32) -> Result<(), InterruptError> {
        #[cfg(feature = "cortex-m")]
        {
            use cortex_m::peripheral::NVIC;
            
            unsafe {
                let mut nvic = cortex_m::Peripherals::steal().NVIC;
                nvic.enable(irq_number as u8);
            }
        }
        
        Ok(())
    }
    
    fn disable_hardware_interrupt(&self, irq_number: u32) -> Result<(), InterruptError> {
        #[cfg(feature = "cortex-m")]
        {
            use cortex_m::peripheral::NVIC;
            
            unsafe {
                let mut nvic = cortex_m::Peripherals::steal().NVIC;
                nvic.disable(irq_number as u8);
            }
        }
        
        Ok(())
    }
    
    fn set_hardware_priority(
        &self,
        irq_number: u32,
        priority: InterruptPriority,
    ) -> Result<(), InterruptError> {
        #[cfg(feature = "cortex-m")]
        {
            use cortex_m::peripheral::NVIC;
            
            unsafe {
                let mut nvic = cortex_m::Peripherals::steal().NVIC;
                nvic.set_priority(irq_number as u8, priority.value());
            }
        }
        
        Ok(())
    }
    
    fn trigger_hardware_interrupt(&self, irq_number: u32) -> Result<(), InterruptError> {
        #[cfg(feature = "cortex-m")]
        {
            use cortex_m::peripheral::NVIC;
            
            unsafe {
                let mut nvic = cortex_m::Peripherals::steal().NVIC;
                nvic.request(irq_number as u8);
            }
        }
        
        Ok(())
    }
    
    fn get_current_time(&self) -> u32 {
        // 这里应该返回当前时间（微秒）
        // 在实际实现中，可以使用系统定时器或DWT周期计数器
        0 // 占位符
    }
}
```

### 中断错误类型

```rust
/// 中断错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptError {
    /// 中断已注册
    AlreadyRegistered,
    /// 中断未注册
    NotRegistered,
    /// 中断表已满
    TableFull,
    /// 中断已禁用
    Disabled,
    /// 无处理函数
    NoHandler,
    /// 优先级无效
    InvalidPriority,
    /// 硬件错误
    HardwareError,
    /// 嵌套过深
    NestingTooDeep,
}

impl core::fmt::Display for InterruptError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::AlreadyRegistered => write!(f, "中断已注册"),
            Self::NotRegistered => write!(f, "中断未注册"),
            Self::TableFull => write!(f, "中断表已满"),
            Self::Disabled => write!(f, "中断已禁用"),
            Self::NoHandler => write!(f, "无处理函数"),
            Self::InvalidPriority => write!(f, "优先级无效"),
            Self::HardwareError => write!(f, "硬件错误"),
            Self::NestingTooDeep => write!(f, "嵌套过深"),
        }
    }
}
```

## 中断服务程序

### 中断服务程序框架

```rust
use rtic::Mutex;
use cortex_m::interrupt::free;

/// 中断服务程序特征
pub trait InterruptServiceRoutine {
    /// 中断处理函数
    fn handle(&mut self);
    
    /// 获取中断类型
    fn interrupt_type(&self) -> InterruptType;
    
    /// 获取优先级
    fn priority(&self) -> InterruptPriority;
    
    /// 是否可以被抢占
    fn is_preemptible(&self) -> bool {
        true
    }
}

/// 定时器中断服务程序
pub struct TimerISR {
    /// 定时器ID
    timer_id: u8,
    /// 回调函数
    callback: Option<fn()>,
    /// 统计信息
    stats: InterruptStats,
}

impl TimerISR {
    pub fn new(timer_id: u8, callback: Option<fn()>) -> Self {
        Self {
            timer_id,
            callback,
            stats: InterruptStats::new(),
        }
    }
    
    /// 设置回调函数
    pub fn set_callback(&mut self, callback: fn()) {
        self.callback = Some(callback);
    }
}

impl InterruptServiceRoutine for TimerISR {
    fn handle(&mut self) {
        let start_time = self.get_current_time();
        
        // 清除中断标志
        self.clear_interrupt_flag();
        
        // 调用回调函数
        if let Some(callback) = self.callback {
            callback();
        }
        
        // 记录处理时间
        let processing_time = self.get_current_time() - start_time;
        self.stats.record_processing_time(processing_time);
    }
    
    fn interrupt_type(&self) -> InterruptType {
        InterruptType::Timer
    }
    
    fn priority(&self) -> InterruptPriority {
        InterruptPriority::HIGH
    }
}

impl TimerISR {
    fn clear_interrupt_flag(&self) {
        // 清除定时器中断标志
        // 具体实现取决于硬件
    }
    
    fn get_current_time(&self) -> u32 {
        // 获取当前时间
        0 // 占位符
    }
}

/// UART中断服务程序
pub struct UartISR {
    /// UART端口号
    port: u8,
    /// 接收缓冲区
    rx_buffer: heapless::spsc::Queue<u8, 256>,
    /// 发送缓冲区
    tx_buffer: heapless::spsc::Queue<u8, 256>,
    /// 统计信息
    stats: UartStats,
}

#[derive(Debug, Default)]
pub struct UartStats {
    /// 接收字节数
    pub bytes_received: AtomicU64,
    /// 发送字节数
    pub bytes_transmitted: AtomicU64,
    /// 接收错误数
    pub rx_errors: AtomicU32,
    /// 发送错误数
    pub tx_errors: AtomicU32,
    /// 缓冲区溢出次数
    pub buffer_overflows: AtomicU32,
}

impl UartISR {
    pub fn new(port: u8) -> Self {
        Self {
            port,
            rx_buffer: heapless::spsc::Queue::new(),
            tx_buffer: heapless::spsc::Queue::new(),
            stats: UartStats::default(),
        }
    }
    
    /// 发送数据
    pub fn send_data(&mut self, data: &[u8]) -> Result<usize, UartError> {
        let mut sent = 0;
        
        for &byte in data {
            if self.tx_buffer.enqueue(byte).is_ok() {
                sent += 1;
            } else {
                self.stats.buffer_overflows.fetch_add(1, Ordering::Relaxed);
                break;
            }
        }
        
        // 启用发送中断
        self.enable_tx_interrupt();
        
        Ok(sent)
    }
    
    /// 接收数据
    pub fn receive_data(&mut self, buffer: &mut [u8]) -> Result<usize, UartError> {
        let mut received = 0;
        
        for i in 0..buffer.len() {
            if let Some(byte) = self.rx_buffer.dequeue() {
                buffer[i] = byte;
                received += 1;
            } else {
                break;
            }
        }
        
        Ok(received)
    }
    
    fn enable_tx_interrupt(&self) {
        // 启用UART发送中断
        // 具体实现取决于硬件
    }
    
    fn disable_tx_interrupt(&self) {
        // 禁用UART发送中断
        // 具体实现取决于硬件
    }
    
    fn read_uart_data(&self) -> Option<u8> {
        // 从UART硬件寄存器读取数据
        // 具体实现取决于硬件
        None // 占位符
    }
    
    fn write_uart_data(&self, data: u8) -> bool {
        // 向UART硬件寄存器写入数据
        // 具体实现取决于硬件
        true // 占位符
    }
    
    fn get_uart_status(&self) -> UartStatus {
        // 获取UART状态
        // 具体实现取决于硬件
        UartStatus::default()
    }
}

#[derive(Debug, Default)]
struct UartStatus {
    rx_ready: bool,
    tx_ready: bool,
    rx_error: bool,
    tx_error: bool,
}

impl InterruptServiceRoutine for UartISR {
    fn handle(&mut self) {
        let status = self.get_uart_status();
        
        // 处理接收中断
        if status.rx_ready {
            if let Some(data) = self.read_uart_data() {
                if self.rx_buffer.enqueue(data).is_err() {
                    self.stats.buffer_overflows.fetch_add(1, Ordering::Relaxed);
                } else {
                    self.stats.bytes_received.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        
        // 处理发送中断
        if status.tx_ready {
            if let Some(data) = self.tx_buffer.dequeue() {
                if self.write_uart_data(data) {
                    self.stats.bytes_transmitted.fetch_add(1, Ordering::Relaxed);
                } else {
                    self.stats.tx_errors.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                // 发送缓冲区为空，禁用发送中断
                self.disable_tx_interrupt();
            }
        }
        
        // 处理错误
        if status.rx_error {
            self.stats.rx_errors.fetch_add(1, Ordering::Relaxed);
        }
        
        if status.tx_error {
            self.stats.tx_errors.fetch_add(1, Ordering::Relaxed);
        }
    }
    
    fn interrupt_type(&self) -> InterruptType {
        InterruptType::Uart
    }
    
    fn priority(&self) -> InterruptPriority {
        InterruptPriority::MEDIUM
    }
}

/// UART错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    /// 缓冲区满
    BufferFull,
    /// 缓冲区空
    BufferEmpty,
    /// 硬件错误
    HardwareError,
    /// 超时
    Timeout,
}
```

## 中断优先级管理

### 优先级管理器

```rust
/// 中断优先级管理器
pub struct InterruptPriorityManager {
    /// 优先级分组
    priority_groups: FnvIndexMap<u8, PriorityGroup, 16>,
    /// 当前最高优先级
    current_highest_priority: InterruptPriority,
    /// 优先级反转检测
    inversion_detector: PriorityInversionDetector,
}

/// 优先级分组
#[derive(Debug)]
pub struct PriorityGroup {
    /// 分组ID
    pub group_id: u8,
    /// 优先级范围
    pub priority_range: (InterruptPriority, InterruptPriority),
    /// 分组中的中断列表
    pub interrupts: Vec<u32, 32>,
    /// 分组统计
    pub stats: PriorityGroupStats,
}

#[derive(Debug, Default)]
pub struct PriorityGroupStats {
    /// 总执行次数
    pub total_executions: AtomicU64,
    /// 总执行时间
    pub total_execution_time: AtomicU64,
    /// 最大执行时间
    pub max_execution_time: AtomicU32,
    /// 平均执行时间
    pub avg_execution_time: AtomicU32,
}

/// 优先级反转检测器
#[derive(Debug)]
pub struct PriorityInversionDetector {
    /// 检测启用状态
    enabled: bool,
    /// 反转事件记录
    inversion_events: Vec<PriorityInversionEvent, 32>,
    /// 检测阈值（微秒）
    detection_threshold: u32,
}

#[derive(Debug, Clone)]
pub struct PriorityInversionEvent {
    /// 高优先级中断号
    pub high_priority_irq: u32,
    /// 低优先级中断号
    pub low_priority_irq: u32,
    /// 反转开始时间
    pub start_time: u64,
    /// 反转持续时间
    pub duration: u32,
    /// 反转严重程度
    pub severity: InversionSeverity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InversionSeverity {
    /// 轻微
    Minor,
    /// 中等
    Moderate,
    /// 严重
    Severe,
    /// 关键
    Critical,
}

impl InterruptPriorityManager {
    pub fn new() -> Self {
        Self {
            priority_groups: FnvIndexMap::new(),
            current_highest_priority: InterruptPriority::LOWEST,
            inversion_detector: PriorityInversionDetector {
                enabled: true,
                inversion_events: Vec::new(),
                detection_threshold: 1000, // 1ms
            },
        }
    }
    
    /// 创建优先级分组
    pub fn create_priority_group(
        &mut self,
        group_id: u8,
        priority_range: (InterruptPriority, InterruptPriority),
    ) -> Result<(), InterruptError> {
        if self.priority_groups.contains_key(&group_id) {
            return Err(InterruptError::AlreadyRegistered);
        }
        
        let group = PriorityGroup {
            group_id,
            priority_range,
            interrupts: Vec::new(),
            stats: PriorityGroupStats::default(),
        };
        
        self.priority_groups.insert(group_id, group)
            .map_err(|_| InterruptError::TableFull)?;
        
        Ok(())
    }
    
    /// 将中断分配到优先级分组
    pub fn assign_to_group(
        &mut self,
        irq_number: u32,
        group_id: u8,
        priority: InterruptPriority,
    ) -> Result<(), InterruptError> {
        if let Some(group) = self.priority_groups.get_mut(&group_id) {
            // 检查优先级是否在分组范围内
            if priority < group.priority_range.0 || priority > group.priority_range.1 {
                return Err(InterruptError::InvalidPriority);
            }
            
            group.interrupts.push(irq_number)
                .map_err(|_| InterruptError::TableFull)?;
            
            Ok(())
        } else {
            Err(InterruptError::NotRegistered)
        }
    }
    
    /// 检测优先级反转
    pub fn detect_priority_inversion(
        &mut self,
        current_irq: u32,
        current_priority: InterruptPriority,
        blocked_time: u32,
    ) {
        if !self.inversion_detector.enabled || blocked_time < self.inversion_detector.detection_threshold {
            return;
        }
        
        // 查找可能被阻塞的高优先级中断
        for (_, group) in &self.priority_groups {
            for &irq in &group.interrupts {
                if irq != current_irq {
                    // 假设这个中断有更高的优先级但被阻塞
                    let severity = match blocked_time {
                        0..=1000 => InversionSeverity::Minor,
                        1001..=5000 => InversionSeverity::Moderate,
                        5001..=10000 => InversionSeverity::Severe,
                        _ => InversionSeverity::Critical,
                    };
                    
                    let event = PriorityInversionEvent {
                        high_priority_irq: irq,
                        low_priority_irq: current_irq,
                        start_time: self.get_current_time(),
                        duration: blocked_time,
                        severity,
                    };
                    
                    if self.inversion_detector.inversion_events.push(event).is_err() {
                        // 如果事件列表满了，移除最旧的事件
                        self.inversion_detector.inversion_events.remove(0);
                        let _ = self.inversion_detector.inversion_events.push(event);
                    }
                    
                    break;
                }
            }
        }
    }
    
    /// 获取优先级反转事件
    pub fn get_inversion_events(&self) -> &[PriorityInversionEvent] {
        &self.inversion_detector.inversion_events
    }
    
    /// 清除优先级反转事件
    pub fn clear_inversion_events(&mut self) {
        self.inversion_detector.inversion_events.clear();
    }
    
    /// 设置检测阈值
    pub fn set_detection_threshold(&mut self, threshold_us: u32) {
        self.inversion_detector.detection_threshold = threshold_us;
    }
    
    /// 启用/禁用优先级反转检测
    pub fn enable_inversion_detection(&mut self, enabled: bool) {
        self.inversion_detector.enabled = enabled;
    }
    
    /// 获取分组统计信息
    pub fn get_group_stats(&self, group_id: u8) -> Option<&PriorityGroupStats> {
        self.priority_groups.get(&group_id).map(|group| &group.stats)
    }
    
    /// 更新分组统计
    pub fn update_group_stats(&mut self, group_id: u8, execution_time: u32) {
        if let Some(group) = self.priority_groups.get_mut(&group_id) {
            let count = group.stats.total_executions.fetch_add(1, Ordering::Relaxed);
            let total_time = group.stats.total_execution_time.fetch_add(execution_time as u64, Ordering::Relaxed);
            
            // 更新最大执行时间
            let mut current_max = group.stats.max_execution_time.load(Ordering::Relaxed);
            while execution_time > current_max {
                match group.stats.max_execution_time.compare_exchange_weak(
                    current_max, execution_time, Ordering::Relaxed, Ordering::Relaxed
                ) {
                    Ok(_) => break,
                    Err(x) => current_max = x,
                }
            }
            
            // 更新平均执行时间
            let avg = (total_time + execution_time as u64) / (count + 1);
            group.stats.avg_execution_time.store(avg as u32, Ordering::Relaxed);
        }
    }
    
    fn get_current_time(&self) -> u64 {
        // 获取当前时间戳
        0 // 占位符
    }
}
```

## 中断与任务交互

### 中断到任务通信

```rust
use rtic::Mutex;
use heapless::spsc::{Producer, Consumer, Queue};

/// 中断到任务的消息
#[derive(Debug, Clone)]
pub enum InterruptMessage {
    /// 数据接收
    DataReceived {
        source: InterruptType,
        data: Vec<u8, 256>,
        timestamp: u64,
    },
    /// 定时器到期
    TimerExpired {
        timer_id: u8,
        timestamp: u64,
    },
    /// 错误事件
    ErrorOccurred {
        source: InterruptType,
        error_code: u32,
        timestamp: u64,
    },
    /// 状态变化
    StatusChanged {
        source: InterruptType,
        old_status: u32,
        new_status: u32,
        timestamp: u64,
    },
}

/// 中断到任务通信管理器
pub struct InterruptTaskCommunicator {
    /// 消息队列
    message_queue: Queue<InterruptMessage, 64>,
    /// 生产者（中断侧）
    producer: Option<Producer<'static, InterruptMessage, 64>>,
    /// 消费者（任务侧）
    consumer: Option<Consumer<'static, InterruptMessage, 64>>,
    /// 通信统计
    stats: CommunicationStats,
}

#[derive(Debug, Default)]
pub struct CommunicationStats {
    /// 发送的消息数
    pub messages_sent: AtomicU64,
    /// 接收的消息数
    pub messages_received: AtomicU64,
    /// 丢失的消息数
    pub messages_dropped: AtomicU64,
    /// 队列最大使用量
    pub max_queue_usage: AtomicU32,
}

impl InterruptTaskCommunicator {
    pub fn new() -> Self {
        let queue = Queue::new();
        let (producer, consumer) = queue.split();
        
        Self {
            message_queue: queue,
            producer: Some(producer),
            consumer: Some(consumer),
            stats: CommunicationStats::default(),
        }
    }
    
    /// 从中断发送消息（在中断上下文中调用）
    pub fn send_from_interrupt(&mut self, message: InterruptMessage) -> Result<(), InterruptMessage> {
        if let Some(ref mut producer) = self.producer {
            match producer.enqueue(message) {
                Ok(_) => {
                    self.stats.messages_sent.fetch_add(1, Ordering::Relaxed);
                    
                    // 更新队列使用统计
                    let current_usage = producer.len() as u32;
                    let mut max_usage = self.stats.max_queue_usage.load(Ordering::Relaxed);
                    while current_usage > max_usage {
                        match self.stats.max_queue_usage.compare_exchange_weak(
                            max_usage, current_usage, Ordering::Relaxed, Ordering::Relaxed
                        ) {
                            Ok(_) => break,
                            Err(x) => max_usage = x,
                        }
                    }
                    
                    Ok(())
                }
                Err(message) => {
                    self.stats.messages_dropped.fetch_add(1, Ordering::Relaxed);
                    Err(message)
                }
            }
        } else {
            Err(message)
        }
    }
    
    /// 从任务接收消息（在任务上下文中调用）
    pub fn receive_in_task(&mut self) -> Option<InterruptMessage> {
        if let Some(ref mut consumer) = self.consumer {
            match consumer.dequeue() {
                Some(message) => {
                    self.stats.messages_received.fetch_add(1, Ordering::Relaxed);
                    Some(message)
                }
                None => None,
            }
        } else {
            None
        }
    }
    
    /// 获取通信统计
    pub fn get_stats(&self) -> &CommunicationStats {
        &self.stats
    }
    
    /// 清除统计信息
    pub fn clear_stats(&mut self) {
        self.stats.messages_sent.store(0, Ordering::Relaxed);
        self.stats.messages_received.store(0, Ordering::Relaxed);
        self.stats.messages_dropped.store(0, Ordering::Relaxed);
        self.stats.max_queue_usage.store(0, Ordering::Relaxed);
    }
}

/// 中断安全的任务通知
pub struct InterruptSafeNotification {
    /// 通知标志
    notification_flags: AtomicU32,
    /// 等待任务列表
    waiting_tasks: Mutex<Vec<u32, 16>>,
    /// 通知统计
    stats: NotificationStats,
}

#[derive(Debug, Default)]
pub struct NotificationStats {
    /// 发送的通知数
    pub notifications_sent: AtomicU64,
    /// 接收的通知数
    pub notifications_received: AtomicU64,
    /// 丢失的通知数
    pub notifications_missed: AtomicU64,
}

/// 通知标志位定义
pub mod notification_flags {
    pub const DATA_READY: u32 = 1 << 0;
    pub const TIMER_EXPIRED: u32 = 1 << 1;
    pub const ERROR_OCCURRED: u32 = 1 << 2;
    pub const STATUS_CHANGED: u32 = 1 << 3;
    pub const BUFFER_FULL: u32 = 1 << 4;
    pub const BUFFER_EMPTY: u32 = 1 << 5;
    pub const COMMUNICATION_READY: u32 = 1 << 6;
    pub const SYSTEM_EVENT: u32 = 1 << 7;
}

impl InterruptSafeNotification {
    pub fn new() -> Self {
        Self {
            notification_flags: AtomicU32::new(0),
            waiting_tasks: Mutex::new(Vec::new()),
            stats: NotificationStats::default(),
        }
    }
    
    /// 从中断发送通知
    pub fn notify_from_interrupt(&self, flags: u32) {
        // 设置通知标志
        self.notification_flags.fetch_or(flags, Ordering::Relaxed);
        self.stats.notifications_sent.fetch_add(1, Ordering::Relaxed);
        
        // 在实际实现中，这里应该唤醒等待的任务
        // 例如：使用RTOS的任务通知机制
        self.wake_waiting_tasks();
    }
    
    /// 在任务中等待通知
    pub fn wait_for_notification(&self, flags: u32, timeout_ms: u32) -> Option<u32> {
        let start_time = self.get_current_time();
        
        loop {
            // 检查通知标志
            let current_flags = self.notification_flags.load(Ordering::Relaxed);
            let matched_flags = current_flags & flags;
            
            if matched_flags != 0 {
                // 清除匹配的标志
                self.notification_flags.fetch_and(!matched_flags, Ordering::Relaxed);
                self.stats.notifications_received.fetch_add(1, Ordering::Relaxed);
                return Some(matched_flags);
            }
            
            // 检查超时
            if self.get_current_time() - start_time > timeout_ms {
                break;
            }
            
            // 在实际实现中，这里应该让任务进入等待状态
            // 例如：使用RTOS的任务延时或事件等待
            self.task_yield();
        }
        
        None
    }
    
    /// 清除通知标志
    pub fn clear_notifications(&self, flags: u32) {
        self.notification_flags.fetch_and(!flags, Ordering::Relaxed);
    }
    
    /// 获取当前通知状态
    pub fn get_notification_status(&self) -> u32 {
        self.notification_flags.load(Ordering::Relaxed)
    }
    
    /// 获取通知统计
    pub fn get_stats(&self) -> &NotificationStats {
        &self.stats
    }
    
    fn wake_waiting_tasks(&self) {
        // 在实际实现中，这里应该唤醒等待的任务
        // 具体实现取决于使用的RTOS
    }
    
    fn task_yield(&self) {
        // 在实际实现中，这里应该让出CPU时间
        // 具体实现取决于使用的RTOS
    }
    
    fn get_current_time(&self) -> u32 {
        // 获取当前时间（毫秒）
        0 // 占位符
    }
}
```

## 中断性能优化

### 中断延迟优化

```rust
/// 中断延迟优化器
pub struct InterruptLatencyOptimizer {
    /// 延迟测量器
    latency_measurer: LatencyMeasurer,
    /// 优化策略
    optimization_strategies: Vec<OptimizationStrategy, 16>,
    /// 性能目标
    performance_targets: PerformanceTargets,
    /// 优化统计
    stats: OptimizationStats,
}

/// 延迟测量器
#[derive(Debug)]
pub struct LatencyMeasurer {
    /// 测量启用状态
    enabled: bool,
    /// 测量结果历史
    measurements: Vec<LatencyMeasurement, 100>,
    /// 当前测量索引
    current_index: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct LatencyMeasurement {
    /// 中断号
    pub irq_number: u32,
    /// 中断触发时间
    pub trigger_time: u64,
    /// 中断开始处理时间
    pub start_time: u64,
    /// 中断结束处理时间
    pub end_time: u64,
    /// 中断延迟（微秒）
    pub latency: u32,
    /// 处理时间（微秒）
    pub processing_time: u32,
}

/// 优化策略
#[derive(Debug, Clone)]
pub enum OptimizationStrategy {
    /// 减少中断处理时间
    ReduceProcessingTime {
        target_time: u32,
        current_time: u32,
    },
    /// 优化中断优先级
    OptimizePriority {
        irq_number: u32,
        suggested_priority: InterruptPriority,
    },
    /// 启用中断合并
    EnableInterruptCoalescing {
        irq_numbers: Vec<u32, 8>,
        coalescing_time: u32,
    },
    /// 使用中断线程化
    UseInterruptThreading {
        irq_number: u32,
        thread_priority: u8,
    },
    /// 优化缓冲区大小
    OptimizeBufferSize {
        irq_number: u32,
        current_size: usize,
        suggested_size: usize,
    },
}

/// 性能目标
#[derive(Debug, Clone)]
pub struct PerformanceTargets {
    /// 最大中断延迟（微秒）
    pub max_interrupt_latency: u32,
    /// 最大处理时间（微秒）
    pub max_processing_time: u32,
    /// 最大嵌套深度
    pub max_nesting_depth: u32,
    /// 目标CPU使用率（百分比）
    pub target_cpu_usage: u8,
}

#[derive(Debug, Default)]
pub struct OptimizationStats {
    /// 优化建议数
    pub suggestions_generated: AtomicU32,
    /// 应用的优化数
    pub optimizations_applied: AtomicU32,
    /// 性能改进百分比
    pub performance_improvement: AtomicU32,
}

impl InterruptLatencyOptimizer {
    pub fn new() -> Self {
        Self {
            latency_measurer: LatencyMeasurer {
                enabled: true,
                measurements: Vec::new(),
                current_index: 0,
            },
            optimization_strategies: Vec::new(),
            performance_targets: PerformanceTargets {
                max_interrupt_latency: 10,    // 10微秒
                max_processing_time: 50,      // 50微秒
                max_nesting_depth: 3,         // 最大3层嵌套
                target_cpu_usage: 80,         // 80% CPU使用率
            },
            stats: OptimizationStats::default(),
        }
    }
    
    /// 开始延迟测量
    pub fn start_latency_measurement(&mut self, irq_number: u32, trigger_time: u64) {
        if !self.latency_measurer.enabled {
            return;
        }
        
        let measurement = LatencyMeasurement {
            irq_number,
            trigger_time,
            start_time: self.get_current_time(),
            end_time: 0,
            latency: 0,
            processing_time: 0,
        };
        
        if self.latency_measurer.measurements.len() < 100 {
            let _ = self.latency_measurer.measurements.push(measurement);
        } else {
            self.latency_measurer.measurements[self.latency_measurer.current_index] = measurement;
            self.latency_measurer.current_index = (self.latency_measurer.current_index + 1) % 100;
        }
    }
    
    /// 结束延迟测量
    pub fn end_latency_measurement(&mut self, irq_number: u32) {
        if !self.latency_measurer.enabled {
            return;
        }
        
        let end_time = self.get_current_time();
        
        // 查找对应的测量记录
        for measurement in &mut self.latency_measurer.measurements {
            if measurement.irq_number == irq_number && measurement.end_time == 0 {
                measurement.end_time = end_time;
                measurement.latency = (measurement.start_time - measurement.trigger_time) as u32;
                measurement.processing_time = (end_time - measurement.start_time) as u32;
                break;
            }
        }
    }
    
    /// 分析性能并生成优化建议
    pub fn analyze_and_optimize(&mut self) -> Vec<OptimizationStrategy, 16> {
        let mut strategies = Vec::new();
        
        // 分析延迟测量结果
        for measurement in &self.latency_measurer.measurements {
            if measurement.end_time == 0 {
                continue; // 跳过未完成的测量
            }
            
            // 检查延迟是否超过目标
            if measurement.latency > self.performance_targets.max_interrupt_latency {
                let strategy = OptimizationStrategy::OptimizePriority {
                    irq_number: measurement.irq_number,
                    suggested_priority: InterruptPriority::HIGH,
                };
                
                if strategies.push(strategy).is_err() {
                    break;
                }
            }
            
            // 检查处理时间是否超过目标
            if measurement.processing_time > self.performance_targets.max_processing_time {
                let strategy = OptimizationStrategy::ReduceProcessingTime {
                    target_time: self.performance_targets.max_processing_time,
                    current_time: measurement.processing_time,
                };
                
                if strategies.push(strategy).is_err() {
                    break;
                }
            }
        }
        
        // 检查是否需要中断合并
        let frequent_interrupts = self.find_frequent_interrupts();
        if frequent_interrupts.len() > 1 {
            let strategy = OptimizationStrategy::EnableInterruptCoalescing {
                irq_numbers: frequent_interrupts,
                coalescing_time: 100, // 100微秒
            };
            
            let _ = strategies.push(strategy);
        }
        
        self.stats.suggestions_generated.store(strategies.len() as u32, Ordering::Relaxed);
        strategies
    }
    
    /// 应用优化策略
    pub fn apply_optimization(&mut self, strategy: &OptimizationStrategy) -> Result<(), InterruptError> {
        match strategy {
            OptimizationStrategy::ReduceProcessingTime { target_time, current_time } => {
                // 这里应该实现具体的处理时间优化逻辑
                println!("优化处理时间: 目标 {}μs, 当前 {}μs", target_time, current_time);
            }
            
            OptimizationStrategy::OptimizeBufferSize { irq_number, current_size, suggested_size } => {
                // 这里应该实现缓冲区大小优化逻辑
                println!("优化中断 {} 的缓冲区大小: 当前 {} -> 建议 {}", irq_number, current_size, suggested_size);
            }
        }
        
        self.stats.optimizations_applied.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
    
    /// 查找频繁触发的中断
    fn find_frequent_interrupts(&self) -> Vec<u32, 8> {
        let mut frequent_interrupts = Vec::new();
        let mut irq_counts: FnvIndexMap<u32, u32, 32> = FnvIndexMap::new();
        
        // 统计中断频率
        for measurement in &self.latency_measurer.measurements {
            if measurement.end_time == 0 {
                continue;
            }
            
            let count = irq_counts.get(&measurement.irq_number).unwrap_or(&0) + 1;
            let _ = irq_counts.insert(measurement.irq_number, count);
        }
        
        // 找出频繁的中断（出现次数 > 10）
        for (irq, count) in &irq_counts {
            if *count > 10 {
                if frequent_interrupts.push(*irq).is_err() {
                    break;
                }
            }
        }
        
        frequent_interrupts
    }
    
    /// 设置性能目标
    pub fn set_performance_targets(&mut self, targets: PerformanceTargets) {
        self.performance_targets = targets;
    }
    
    /// 获取延迟统计
    pub fn get_latency_statistics(&self) -> LatencyStatistics {
        let mut stats = LatencyStatistics::default();
        
        for measurement in &self.latency_measurer.measurements {
            if measurement.end_time == 0 {
                continue;
            }
            
            stats.total_measurements += 1;
            stats.total_latency += measurement.latency as u64;
            stats.total_processing_time += measurement.processing_time as u64;
            
            if measurement.latency > stats.max_latency {
                stats.max_latency = measurement.latency;
            }
            
            if measurement.latency < stats.min_latency {
                stats.min_latency = measurement.latency;
            }
            
            if measurement.processing_time > stats.max_processing_time {
                stats.max_processing_time = measurement.processing_time;
            }
            
            if measurement.processing_time < stats.min_processing_time {
                stats.min_processing_time = measurement.processing_time;
            }
        }
        
        if stats.total_measurements > 0 {
            stats.avg_latency = (stats.total_latency / stats.total_measurements as u64) as u32;
            stats.avg_processing_time = (stats.total_processing_time / stats.total_measurements as u64) as u32;
        }
        
        stats
    }
    
    /// 启用/禁用延迟测量
    pub fn enable_latency_measurement(&mut self, enabled: bool) {
        self.latency_measurer.enabled = enabled;
    }
    
    /// 清除测量历史
    pub fn clear_measurements(&mut self) {
        self.latency_measurer.measurements.clear();
        self.latency_measurer.current_index = 0;
    }
    
    fn get_current_time(&self) -> u64 {
        // 获取当前时间戳（微秒）
        0 // 占位符
    }
}

/// 延迟统计信息
#[derive(Debug, Default)]
pub struct LatencyStatistics {
    /// 总测量次数
    pub total_measurements: u32,
    /// 总延迟时间
    pub total_latency: u64,
    /// 总处理时间
    pub total_processing_time: u64,
    /// 最大延迟
    pub max_latency: u32,
    /// 最小延迟
    pub min_latency: u32,
    /// 平均延迟
    pub avg_latency: u32,
    /// 最大处理时间
    pub max_processing_time: u32,
    /// 最小处理时间
    pub min_processing_time: u32,
    /// 平均处理时间
    pub avg_processing_time: u32,
}

### 中断合并优化

```rust
/// 中断合并管理器
pub struct InterruptCoalescingManager {
    /// 合并配置
    coalescing_configs: FnvIndexMap<u32, CoalescingConfig, 16>,
    /// 合并缓冲区
    coalescing_buffers: FnvIndexMap<u32, CoalescingBuffer, 16>,
    /// 合并统计
    stats: CoalescingStats,
}

/// 合并配置
#[derive(Debug, Clone)]
pub struct CoalescingConfig {
    /// 中断号
    pub irq_number: u32,
    /// 合并时间窗口（微秒）
    pub time_window: u32,
    /// 最大合并数量
    pub max_count: u32,
    /// 启用状态
    pub enabled: bool,
}

/// 合并缓冲区
#[derive(Debug)]
pub struct CoalescingBuffer {
    /// 缓冲的中断事件
    events: Vec<InterruptEvent, 32>,
    /// 最后一次中断时间
    last_interrupt_time: u64,
    /// 定时器句柄
    timer_handle: Option<u32>,
}

#[derive(Debug, Clone)]
pub struct InterruptEvent {
    /// 中断号
    pub irq_number: u32,
    /// 中断时间
    pub timestamp: u64,
    /// 中断数据
    pub data: Option<Vec<u8, 64>>,
}

#[derive(Debug, Default)]
pub struct CoalescingStats {
    /// 合并的中断数
    pub coalesced_interrupts: AtomicU64,
    /// 节省的中断处理次数
    pub saved_interrupt_calls: AtomicU64,
    /// 平均合并数量
    pub avg_coalescing_count: AtomicU32,
    /// 最大合并数量
    pub max_coalescing_count: AtomicU32,
}

impl InterruptCoalescingManager {
    pub fn new() -> Self {
        Self {
            coalescing_configs: FnvIndexMap::new(),
            coalescing_buffers: FnvIndexMap::new(),
            stats: CoalescingStats::default(),
        }
    }
    
    /// 配置中断合并
    pub fn configure_coalescing(
        &mut self,
        irq_number: u32,
        time_window: u32,
        max_count: u32,
    ) -> Result<(), InterruptError> {
        let config = CoalescingConfig {
            irq_number,
            time_window,
            max_count,
            enabled: true,
        };
        
        self.coalescing_configs.insert(irq_number, config)
            .map_err(|_| InterruptError::TableFull)?;
        
        let buffer = CoalescingBuffer {
            events: Vec::new(),
            last_interrupt_time: 0,
            timer_handle: None,
        };
        
        self.coalescing_buffers.insert(irq_number, buffer)
            .map_err(|_| InterruptError::TableFull)?;
        
        Ok(())
    }
    
    /// 处理中断事件（可能进行合并）
    pub fn handle_interrupt_event(
        &mut self,
        irq_number: u32,
        data: Option<Vec<u8, 64>>,
    ) -> Result<bool, InterruptError> {
        let current_time = self.get_current_time();
        
        if let Some(config) = self.coalescing_configs.get(&irq_number) {
            if !config.enabled {
                return Ok(false); // 不进行合并，立即处理
            }
            
            if let Some(buffer) = self.coalescing_buffers.get_mut(&irq_number) {
                let event = InterruptEvent {
                    irq_number,
                    timestamp: current_time,
                    data,
                };
                
                // 检查是否需要立即处理
                let should_process_immediately = 
                    buffer.events.is_empty() || 
                    (current_time - buffer.last_interrupt_time > config.time_window) ||
                    buffer.events.len() >= config.max_count as usize;
                
                if should_process_immediately {
                    // 处理缓冲区中的所有事件
                    self.process_coalesced_events(irq_number)?;
                    
                    // 添加当前事件
                    buffer.events.push(event).map_err(|_| InterruptError::TableFull)?;
                    buffer.last_interrupt_time = current_time;
                    
                    // 启动定时器
                    self.start_coalescing_timer(irq_number, config.time_window);
                    
                    Ok(false) // 不立即处理当前事件
                } else {
                    // 添加到缓冲区
                    buffer.events.push(event).map_err(|_| InterruptError::TableFull)?;
                    buffer.last_interrupt_time = current_time;
                    
                    self.stats.coalesced_interrupts.fetch_add(1, Ordering::Relaxed);
                    Ok(true) // 事件已合并
                }
            } else {
                Err(InterruptError::NotRegistered)
            }
        } else {
            Ok(false) // 没有配置合并，立即处理
        }
    }
    
    /// 处理合并的事件
    fn process_coalesced_events(&mut self, irq_number: u32) -> Result<(), InterruptError> {
        if let Some(buffer) = self.coalescing_buffers.get_mut(&irq_number) {
            if buffer.events.is_empty() {
                return Ok(());
            }
            
            let event_count = buffer.events.len() as u32;
            
            // 更新统计信息
            self.stats.saved_interrupt_calls.fetch_add(event_count as u64 - 1, Ordering::Relaxed);
            
            let mut current_max = self.stats.max_coalescing_count.load(Ordering::Relaxed);
            while event_count > current_max {
                match self.stats.max_coalescing_count.compare_exchange_weak(
                    current_max, event_count, Ordering::Relaxed, Ordering::Relaxed
                ) {
                    Ok(_) => break,
                    Err(x) => current_max = x,
                }
            }
            
            // 计算平均合并数量
            let total_coalesced = self.stats.coalesced_interrupts.load(Ordering::Relaxed);
            if total_coalesced > 0 {
                let avg = total_coalesced / (total_coalesced / event_count as u64).max(1);
                self.stats.avg_coalescing_count.store(avg as u32, Ordering::Relaxed);
            }
            
            // 在这里应该调用实际的中断处理函数
            // 传递所有合并的事件
            self.call_coalesced_interrupt_handler(irq_number, &buffer.events);
            
            // 清空缓冲区
            buffer.events.clear();
            
            // 停止定时器
            if let Some(timer_handle) = buffer.timer_handle.take() {
                self.stop_coalescing_timer(timer_handle);
            }
            
            Ok(())
        } else {
            Err(InterruptError::NotRegistered)
        }
    }
    
    /// 启用/禁用中断合并
    pub fn enable_coalescing(&mut self, irq_number: u32, enabled: bool) -> Result<(), InterruptError> {
        if let Some(config) = self.coalescing_configs.get_mut(&irq_number) {
            config.enabled = enabled;
            
            if !enabled {
                // 如果禁用合并，立即处理缓冲区中的事件
                self.process_coalesced_events(irq_number)?;
            }
            
            Ok(())
        } else {
            Err(InterruptError::NotRegistered)
        }
    }
    
    /// 获取合并统计
    pub fn get_coalescing_stats(&self) -> &CoalescingStats {
        &self.stats
    }
    
    /// 清除统计信息
    pub fn clear_stats(&mut self) {
        self.stats.coalesced_interrupts.store(0, Ordering::Relaxed);
        self.stats.saved_interrupt_calls.store(0, Ordering::Relaxed);
        self.stats.avg_coalescing_count.store(0, Ordering::Relaxed);
        self.stats.max_coalescing_count.store(0, Ordering::Relaxed);
    }
    
    fn start_coalescing_timer(&mut self, irq_number: u32, timeout: u32) {
        // 在实际实现中，这里应该启动一个硬件定时器
        // 当定时器到期时，调用 process_coalesced_events
        
        if let Some(buffer) = self.coalescing_buffers.get_mut(&irq_number) {
            buffer.timer_handle = Some(self.create_timer(timeout));
        }
    }
    
    fn stop_coalescing_timer(&self, timer_handle: u32) {
        // 停止指定的定时器
        self.destroy_timer(timer_handle);
    }
    
    fn create_timer(&self, timeout: u32) -> u32 {
        // 创建定时器并返回句柄
        // 具体实现取决于硬件平台
        0 // 占位符
    }
    
    fn destroy_timer(&self, timer_handle: u32) {
        // 销毁定时器
        // 具体实现取决于硬件平台
    }
    
    fn call_coalesced_interrupt_handler(&self, irq_number: u32, events: &[InterruptEvent]) {
        // 调用合并后的中断处理函数
        // 在实际实现中，这里应该调用注册的处理函数
        println!("处理合并的中断 {}: {} 个事件", irq_number, events.len());
    }
    
    fn get_current_time(&self) -> u64 {
        // 获取当前时间戳（微秒）
        0 // 占位符
    }
}
## 使用示例

### 基本中断处理

```rust
use embedded_rtos::interrupt::*;

fn main() -> Result<(), InterruptError> {
    // 创建中断管理器
    let mut interrupt_manager = InterruptManager::new();
    
    // 注册中断处理函数
    interrupt_manager.register_handler(
        10, // 中断号
        InterruptHandler::new(
            timer_interrupt_handler,
            InterruptPriority::High,
            InterruptConfig {
                enabled: true,
                edge_triggered: false,
                active_high: true,
                buffer_size: 64,
            },
        ),
    )?;
    
    // 启用中断
    interrupt_manager.enable_interrupt(10)?;
    
    // 主循环
    loop {
        // 处理待处理的中断
        interrupt_manager.process_pending_interrupts();
        
        // 其他任务
        cortex_m::asm::wfi(); // 等待中断
    }
}

// 定时器中断处理函数
fn timer_interrupt_handler(context: &InterruptContext) {
    println!("定时器中断触发，时间戳: {}", context.timestamp);
    
    // 处理定时器中断逻辑
    // ...
}
```

### 中断性能监控

```rust
use embedded_rtos::interrupt::*;

fn setup_interrupt_monitoring() -> Result<(), InterruptError> {
    let mut performance_monitor = InterruptPerformanceMonitor::new();
    
    // 设置性能目标
    performance_monitor.set_performance_targets(PerformanceTargets {
        max_latency: 100,        // 最大延迟 100 微秒
        max_processing_time: 50, // 最大处理时间 50 微秒
        max_frequency: 1000,     // 最大频率 1000 Hz
    });
    
    // 启用延迟测量
    performance_monitor.enable_latency_measurement(true);
    
    // 定期检查性能并优化
    loop {
        // 等待一段时间
        cortex_m::asm::delay(1000000); // 1秒
        
        // 分析性能并应用优化
        if let Ok(optimizations) = performance_monitor.analyze_and_optimize() {
            for optimization in optimizations {
                if let Err(e) = performance_monitor.apply_optimization(optimization) {
                    println!("应用优化失败: {:?}", e);
                }
            }
        }
        
        // 获取统计信息
        let stats = performance_monitor.get_latency_statistics();
        println!("中断延迟统计:");
        println!("  平均延迟: {} 微秒", stats.avg_latency);
        println!("  最大延迟: {} 微秒", stats.max_latency);
        println!("  总测量次数: {}", stats.total_measurements);
    }
}
```

### 中断合并优化

```rust
use embedded_rtos::interrupt::*;

fn setup_interrupt_coalescing() -> Result<(), InterruptError> {
    let mut coalescing_manager = InterruptCoalescingManager::new();
    
    // 为网络中断配置合并
    coalescing_manager.configure_coalescing(
        20,    // 网络中断号
        1000,  // 1毫秒时间窗口
        10,    // 最多合并10个中断
    )?;
    
    // 为磁盘I/O中断配置合并
    coalescing_manager.configure_coalescing(
        21,    // 磁盘中断号
        2000,  // 2毫秒时间窗口
        5,     // 最多合并5个中断
    )?;
    
    // 在中断处理中使用合并
    loop {
        // 模拟中断事件
        let data = Some(vec![0x01, 0x02, 0x03, 0x04]);
        
        match coalescing_manager.handle_interrupt_event(20, data)? {
            true => {
                // 中断已被合并，不需要立即处理
                println!("网络中断已合并");
            }
            false => {
                // 需要立即处理中断
                println!("立即处理网络中断");
            }
        }
        
        // 定期检查合并统计
        let stats = coalescing_manager.get_coalescing_stats();
        println!("合并统计:");
        println!("  合并的中断数: {}", stats.coalesced_interrupts.load(Ordering::Relaxed));
        println!("  节省的处理次数: {}", stats.saved_interrupt_calls.load(Ordering::Relaxed));
        println!("  最大合并数量: {}", stats.max_coalescing_count.load(Ordering::Relaxed));
        
        cortex_m::asm::delay(10000); // 短暂延迟
    }
}
```

### 完整的中断系统集成

```rust
use embedded_rtos::interrupt::*;
use embedded_rtos::rtos::*;

struct InterruptSystem {
    manager: InterruptManager,
    performance_monitor: InterruptPerformanceMonitor,
    coalescing_manager: InterruptCoalescingManager,
}

impl InterruptSystem {
    pub fn new() -> Self {
        Self {
            manager: InterruptManager::new(),
            performance_monitor: InterruptPerformanceMonitor::new(),
            coalescing_manager: InterruptCoalescingManager::new(),
        }
    }
    
    pub fn initialize(&mut self) -> Result<(), InterruptError> {
        // 注册系统中断
        self.register_system_interrupts()?;
        
        // 配置性能监控
        self.setup_performance_monitoring()?;
        
        // 配置中断合并
        self.setup_interrupt_coalescing()?;
        
        Ok(())
    }
    
    fn register_system_interrupts(&mut self) -> Result<(), InterruptError> {
        // 系统定时器中断
        self.manager.register_handler(
            15, // SysTick
            InterruptHandler::new(
                systick_handler,
                InterruptPriority::Highest,
                InterruptConfig::default(),
            ),
        )?;
        
        // UART中断
        self.manager.register_handler(
            37, // UART0
            InterruptHandler::new(
                uart_handler,
                InterruptPriority::High,
                InterruptConfig {
                    enabled: true,
                    edge_triggered: false,
                    active_high: true,
                    buffer_size: 128,
                },
            ),
        )?;
        
        // GPIO中断
        self.manager.register_handler(
            16, // GPIO
            InterruptHandler::new(
                gpio_handler,
                InterruptPriority::Medium,
                InterruptConfig::default(),
            ),
        )?;
        
        Ok(())
    }
    
    fn setup_performance_monitoring(&mut self) -> Result<(), InterruptError> {
        self.performance_monitor.set_performance_targets(PerformanceTargets {
            max_latency: 50,
            max_processing_time: 25,
            max_frequency: 10000,
        });
        
        self.performance_monitor.enable_latency_measurement(true);
        Ok(())
    }
    
    fn setup_interrupt_coalescing(&mut self) -> Result<(), InterruptError> {
        // 为UART接收配置合并
        self.coalescing_manager.configure_coalescing(37, 500, 8)?;
        Ok(())
    }
    
    pub fn run(&mut self) {
        loop {
            // 处理待处理的中断
            self.manager.process_pending_interrupts();
            
            // 定期性能分析和优化
            if let Ok(optimizations) = self.performance_monitor.analyze_and_optimize() {
                for optimization in optimizations {
                    let _ = self.performance_monitor.apply_optimization(optimization);
                }
            }
            
            // 让出CPU给其他任务
            cortex_m::asm::wfi();
        }
    }
}

// 中断处理函数
fn systick_handler(context: &InterruptContext) {
    // 系统时钟处理
    RTOS.tick();
}

fn uart_handler(context: &InterruptContext) {
    // UART数据处理
    if let Some(data) = context.data {
        // 处理接收到的数据
        process_uart_data(data);
    }
}

fn gpio_handler(context: &InterruptContext) {
    // GPIO中断处理
    let pin = context.irq_number - 16; // 假设GPIO从16开始
    handle_gpio_interrupt(pin);
}

fn process_uart_data(data: &[u8]) {
    // 处理UART数据
    println!("收到UART数据: {:?}", data);
}

fn handle_gpio_interrupt(pin: u32) {
    // 处理GPIO中断
    println!("GPIO {} 中断触发", pin);
}

fn main() -> ! {
    let mut interrupt_system = InterruptSystem::new();
    
    if let Err(e) = interrupt_system.initialize() {
        panic!("中断系统初始化失败: {:?}", e);
    }
    
    // 启用全局中断
    unsafe {
        cortex_m::interrupt::enable();
    }
    
    // 运行中断系统
    interrupt_system.run();
}
```

## 总结

本章详细介绍了嵌入式RTOS中的中断处理机制，包括：

### 核心特性

1. **高效的中断管理**：支持中断注册、启用/禁用、优先级管理等基本功能
2. **性能监控**：实时监控中断延迟、处理时间和频率，提供性能统计
3. **自动优化**：基于性能数据自动调整中断优先级和缓冲区大小
4. **中断合并**：对高频中断进行合并处理，减少系统开销
5. **无锁设计**：使用原子操作和无锁数据结构，确保中断处理的实时性

### 设计优势

1. **实时性保证**：通过优先级管理和延迟监控确保关键中断的实时响应
2. **资源高效**：使用heapless容器和静态内存分配，适合资源受限的嵌入式环境
3. **可扩展性**：模块化设计支持不同类型的中断处理需求
4. **调试友好**：提供详细的统计信息和性能指标，便于系统调试和优化
5. **安全性**：通过类型安全的Rust特性避免常见的中断处理错误

### 应用场景

- **实时控制系统**：需要严格时序保证的工业控制应用
- **通信设备**：高频数据处理的网络和通信设备
- **传感器网络**：多传感器数据采集和处理系统
- **嵌入式GUI**：需要响应用户交互的图形界面系统
- **电机控制**：精确时序要求的电机和伺服控制

通过本章的学习，您应该能够：
- 理解嵌入式系统中断处理的核心概念和挑战
- 掌握高效中断管理器的设计和实现
- 学会使用性能监控工具优化中断处理性能
- 了解中断合并等高级优化技术
- 能够在实际项目中应用这些中断处理技术

下一章我们将探讨内存管理，学习如何在嵌入式RTOS中实现高效、安全的内存分配和管理机制。
            }
            
            OptimizationStrategy::EnableInterruptCoalescing { irq_numbers, coalescing_time } => {
                // 这里应该实现中断合并逻辑
                println!("为中断 {:?} 启用合并，合并时间 {}μs", irq_numbers, coalescing_time);
            }
            
            OptimizationStrategy::UseInterruptThreading { irq_number, thread_priority } => {
                // 这里应该实现中断线程化逻辑
                println!("为中断 {} 启用线程化，线程优先级 {}", irq_number, thread_priority);
            }
            
            OptimizationStrategy::Optimize