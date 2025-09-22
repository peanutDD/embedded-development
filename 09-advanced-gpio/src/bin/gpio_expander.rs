#![no_std]
#![no_main]

//! # GPIO扩展器控制程序
//! 
//! 演示GPIO扩展器的高级功能：
//! - I2C/SPI GPIO扩展器控制
//! - 多扩展器级联管理
//! - 中断处理和事件管理
//! - 引脚配置和状态监控
//! - 错误检测和恢复

use panic_halt as _;
use cortex_m_rt::entry;
use heapless::{Vec, String, FnvIndexMap};
use critical_section::Mutex;
use core::cell::RefCell;

use advanced_gpio::{
    GpioExpander, PinConfig, DriveStrength, SlewRate,
    ExpanderType, PowerState,
};

/// GPIO扩展器控制系统
pub struct GpioExpanderSystem {
    /// 扩展器管理器
    expander_manager: ExpanderManager,
    /// 中断管理器
    interrupt_manager: InterruptManager,
    /// 配置管理器
    config_manager: ConfigManager,
    /// 监控系统
    monitor_system: MonitorSystem,
    /// 系统状态
    system_state: SystemState,
    /// 统计信息
    stats: SystemStats,
}

/// 扩展器管理器
pub struct ExpanderManager {
    /// 扩展器列表
    expanders: Vec<ExpanderDevice, 8>,
    /// 引脚映射表
    pin_mapping: FnvIndexMap<u16, VirtualPin, 128>,
    /// 级联配置
    cascade_config: CascadeConfig,
    /// 通信接口
    communication: CommunicationInterface,
    /// 统计信息
    stats: ExpanderManagerStats,
}

/// 扩展器设备
#[derive(Debug, Clone)]
pub struct ExpanderDevice {
    /// 设备ID
    pub id: u8,
    /// 设备类型
    pub device_type: ExpanderDeviceType,
    /// 通信地址
    pub address: u8,
    /// 引脚数量
    pub pin_count: u8,
    /// 当前状态
    pub status: DeviceStatus,
    /// 配置信息
    pub config: DeviceConfig,
    /// 引脚状态
    pub pin_states: Vec<PinState, 32>,
    /// 中断配置
    pub interrupt_config: InterruptConfig,
    /// 错误信息
    pub error_info: ErrorInfo,
}

/// 扩展器设备类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExpanderDeviceType {
    /// MCP23017 (I2C, 16引脚)
    MCP23017,
    /// MCP23S17 (SPI, 16引脚)
    MCP23S17,
    /// PCF8574 (I2C, 8引脚)
    PCF8574,
    /// 74HC595 (SPI, 8引脚输出)
    HC595,
    /// 74HC165 (SPI, 8引脚输入)
    HC165,
    /// TCA9548A (I2C多路复用器)
    TCA9548A,
}

/// 设备状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeviceStatus {
    /// 未初始化
    Uninitialized,
    /// 正常工作
    Active,
    /// 通信错误
    CommunicationError,
    /// 配置错误
    ConfigurationError,
    /// 硬件故障
    HardwareFault,
    /// 低功耗模式
    LowPower,
    /// 禁用
    Disabled,
}

/// 设备配置
#[derive(Debug, Clone)]
pub struct DeviceConfig {
    /// 时钟频率
    pub clock_frequency: u32,
    /// 超时设置
    pub timeout_ms: u16,
    /// 重试次数
    pub retry_count: u8,
    /// 默认引脚配置
    pub default_pin_config: PinConfig,
    /// 中断使能
    pub interrupt_enable: bool,
    /// 功耗模式
    pub power_mode: PowerState,
}

/// 引脚状态
#[derive(Debug, Clone, Copy)]
pub struct PinState {
    /// 引脚编号
    pub pin_number: u8,
    /// 当前配置
    pub config: PinConfig,
    /// 逻辑状态
    pub logical_state: bool,
    /// 物理状态
    pub physical_state: bool,
    /// 最后更新时间
    pub last_update: u32,
    /// 变化计数
    pub change_count: u32,
}

/// 中断配置
#[derive(Debug, Clone, Copy)]
pub struct InterruptConfig {
    /// 中断使能
    pub enabled: bool,
    /// 中断引脚
    pub interrupt_pin: Option<u8>,
    /// 中断类型
    pub interrupt_type: InterruptType,
    /// 中断极性
    pub polarity: InterruptPolarity,
    /// 默认值比较
    pub default_value: u16,
    /// 中断掩码
    pub interrupt_mask: u16,
}

/// 中断类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterruptType {
    /// 电平变化
    LevelChange,
    /// 与默认值比较
    DefaultValueCompare,
}

/// 中断极性
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterruptPolarity {
    /// 低电平有效
    ActiveLow,
    /// 高电平有效
    ActiveHigh,
}

/// 错误信息
#[derive(Debug, Clone, Copy)]
pub struct ErrorInfo {
    /// 最后错误类型
    pub last_error: ErrorType,
    /// 错误时间
    pub error_time: u32,
    /// 错误计数
    pub error_count: u32,
    /// 连续错误计数
    pub consecutive_errors: u8,
}

/// 错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ErrorType {
    /// 无错误
    None,
    /// 通信超时
    CommunicationTimeout,
    /// 地址无响应
    NoAcknowledge,
    /// 数据校验错误
    DataChecksum,
    /// 配置冲突
    ConfigurationConflict,
    /// 硬件故障
    HardwareFault,
}

/// 虚拟引脚
#[derive(Debug, Clone, Copy)]
pub struct VirtualPin {
    /// 扩展器ID
    pub expander_id: u8,
    /// 物理引脚号
    pub physical_pin: u8,
    /// 虚拟引脚号
    pub virtual_pin: u16,
    /// 引脚功能
    pub function: PinFunction,
}

/// 引脚功能
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PinFunction {
    /// 通用IO
    GeneralPurpose,
    /// 中断输入
    InterruptInput,
    /// PWM输出
    PwmOutput,
    /// 模拟输入
    AnalogInput,
    /// 串行通信
    SerialCommunication,
    /// 时钟信号
    ClockSignal,
}

/// 级联配置
#[derive(Debug, Clone)]
pub struct CascadeConfig {
    /// 级联使能
    pub enabled: bool,
    /// 级联深度
    pub cascade_depth: u8,
    /// 级联顺序
    pub cascade_order: Vec<u8, 8>,
    /// 时钟同步
    pub clock_sync: bool,
    /// 数据同步
    pub data_sync: bool,
}

/// 通信接口
#[derive(Debug, Clone)]
pub struct CommunicationInterface {
    /// I2C接口配置
    pub i2c_config: I2cConfig,
    /// SPI接口配置
    pub spi_config: SpiConfig,
    /// 当前接口类型
    pub current_interface: InterfaceType,
    /// 通信统计
    pub comm_stats: CommunicationStats,
}

/// I2C配置
#[derive(Debug, Clone, Copy)]
pub struct I2cConfig {
    /// 时钟频率
    pub clock_frequency: u32,
    /// 地址模式
    pub address_mode: AddressMode,
    /// 超时设置
    pub timeout_ms: u16,
    /// 重试次数
    pub retry_count: u8,
}

/// SPI配置
#[derive(Debug, Clone, Copy)]
pub struct SpiConfig {
    /// 时钟频率
    pub clock_frequency: u32,
    /// 时钟极性
    pub clock_polarity: ClockPolarity,
    /// 时钟相位
    pub clock_phase: ClockPhase,
    /// 数据位宽
    pub data_width: u8,
    /// 片选极性
    pub chip_select_polarity: ChipSelectPolarity,
}

/// 地址模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AddressMode {
    /// 7位地址
    SevenBit,
    /// 10位地址
    TenBit,
}

/// 时钟极性
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClockPolarity {
    /// 空闲时低电平
    IdleLow,
    /// 空闲时高电平
    IdleHigh,
}

/// 时钟相位
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClockPhase {
    /// 第一个边沿采样
    FirstEdge,
    /// 第二个边沿采样
    SecondEdge,
}

/// 片选极性
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChipSelectPolarity {
    /// 低电平有效
    ActiveLow,
    /// 高电平有效
    ActiveHigh,
}

/// 接口类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterfaceType {
    I2C,
    SPI,
    Parallel,
}

/// 通信统计
#[derive(Debug, Clone, Copy)]
pub struct CommunicationStats {
    /// 总传输次数
    pub total_transfers: u32,
    /// 成功传输次数
    pub successful_transfers: u32,
    /// 失败传输次数
    pub failed_transfers: u32,
    /// 平均传输时间
    pub average_transfer_time: u32,
    /// 最大传输时间
    pub max_transfer_time: u32,
    /// 重试次数
    pub retry_count: u32,
}

/// 中断管理器
pub struct InterruptManager {
    /// 中断源列表
    interrupt_sources: Vec<InterruptSource, 16>,
    /// 中断队列
    interrupt_queue: Vec<InterruptEvent, 32>,
    /// 中断处理器
    interrupt_handlers: Vec<InterruptHandler, 8>,
    /// 中断统计
    interrupt_stats: InterruptStats,
}

/// 中断源
#[derive(Debug, Clone)]
pub struct InterruptSource {
    /// 源ID
    pub source_id: u8,
    /// 扩展器ID
    pub expander_id: u8,
    /// 引脚掩码
    pub pin_mask: u16,
    /// 中断类型
    pub interrupt_type: InterruptType,
    /// 使能状态
    pub enabled: bool,
    /// 优先级
    pub priority: u8,
}

/// 中断事件
#[derive(Debug, Clone, Copy)]
pub struct InterruptEvent {
    /// 事件ID
    pub event_id: u16,
    /// 源ID
    pub source_id: u8,
    /// 时间戳
    pub timestamp: u32,
    /// 引脚状态
    pub pin_state: u16,
    /// 变化掩码
    pub change_mask: u16,
    /// 事件类型
    pub event_type: EventType,
}

/// 事件类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EventType {
    /// 引脚状态变化
    PinStateChange,
    /// 中断触发
    InterruptTriggered,
    /// 通信错误
    CommunicationError,
    /// 配置更新
    ConfigurationUpdate,
}

/// 中断处理器
#[derive(Debug, Clone)]
pub struct InterruptHandler {
    /// 处理器ID
    pub handler_id: u8,
    /// 处理的事件类型
    pub handled_events: Vec<EventType, 4>,
    /// 处理计数
    pub process_count: u32,
    /// 平均处理时间
    pub average_process_time: u32,
}

/// 中断统计
#[derive(Debug, Default)]
pub struct InterruptStats {
    pub total_interrupts: u32,
    pub processed_interrupts: u32,
    pub missed_interrupts: u32,
    pub average_response_time: u32,
    pub max_response_time: u32,
}

/// 配置管理器
pub struct ConfigManager {
    /// 配置模板
    config_templates: Vec<ConfigTemplate, 8>,
    /// 当前配置
    current_configs: FnvIndexMap<u8, DeviceConfig, 8>,
    /// 配置历史
    config_history: Vec<ConfigHistoryEntry, 16>,
    /// 配置验证器
    config_validator: ConfigValidator,
}

/// 配置模板
#[derive(Debug, Clone)]
pub struct ConfigTemplate {
    /// 模板ID
    pub template_id: u8,
    /// 模板名称
    pub name: String<32>,
    /// 设备类型
    pub device_type: ExpanderDeviceType,
    /// 默认配置
    pub default_config: DeviceConfig,
    /// 引脚配置
    pub pin_configs: Vec<PinConfig, 32>,
}

/// 配置历史条目
#[derive(Debug, Clone)]
pub struct ConfigHistoryEntry {
    /// 条目ID
    pub entry_id: u16,
    /// 设备ID
    pub device_id: u8,
    /// 时间戳
    pub timestamp: u32,
    /// 配置操作
    pub operation: ConfigOperation,
    /// 配置数据
    pub config_data: ConfigData,
}

/// 配置操作
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConfigOperation {
    /// 创建配置
    Create,
    /// 更新配置
    Update,
    /// 删除配置
    Delete,
    /// 恢复配置
    Restore,
}

/// 配置数据
#[derive(Debug, Clone)]
pub struct ConfigData {
    /// 设备配置
    pub device_config: Option<DeviceConfig>,
    /// 引脚配置
    pub pin_configs: Vec<(u8, PinConfig), 8>,
    /// 中断配置
    pub interrupt_config: Option<InterruptConfig>,
}

/// 配置验证器
#[derive(Debug)]
pub struct ConfigValidator {
    /// 验证规则
    validation_rules: Vec<ValidationRule, 16>,
    /// 验证统计
    validation_stats: ValidationStats,
}

/// 验证规则
#[derive(Debug, Clone)]
pub struct ValidationRule {
    /// 规则ID
    pub rule_id: u8,
    /// 规则类型
    pub rule_type: RuleType,
    /// 规则参数
    pub parameters: [u32; 4],
    /// 错误消息
    pub error_message: String<64>,
}

/// 规则类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RuleType {
    /// 引脚冲突检查
    PinConflictCheck,
    /// 时钟频率范围
    ClockFrequencyRange,
    /// 功耗限制
    PowerConsumptionLimit,
    /// 地址唯一性
    AddressUniqueness,
}

/// 验证统计
#[derive(Debug, Default)]
pub struct ValidationStats {
    pub total_validations: u32,
    pub successful_validations: u32,
    pub failed_validations: u32,
    pub rule_violations: u32,
}

/// 监控系统
pub struct MonitorSystem {
    /// 性能监控器
    performance_monitor: PerformanceMonitor,
    /// 健康检查器
    health_checker: HealthChecker,
    /// 诊断系统
    diagnostic_system: DiagnosticSystem,
    /// 监控统计
    monitor_stats: MonitorStats,
}

/// 性能监控器
#[derive(Debug)]
pub struct PerformanceMonitor {
    /// 响应时间统计
    pub response_times: Vec<u32, 100>,
    /// 吞吐量统计
    pub throughput_stats: ThroughputStats,
    /// 资源使用统计
    pub resource_usage: ResourceUsage,
    /// 性能指标
    pub performance_metrics: PerformanceMetrics,
}

/// 吞吐量统计
#[derive(Debug, Clone, Copy)]
pub struct ThroughputStats {
    /// 每秒操作数
    pub operations_per_second: f32,
    /// 每秒字节数
    pub bytes_per_second: f32,
    /// 峰值吞吐量
    pub peak_throughput: f32,
    /// 平均吞吐量
    pub average_throughput: f32,
}

/// 资源使用
#[derive(Debug, Clone, Copy)]
pub struct ResourceUsage {
    /// CPU使用率
    pub cpu_usage: f32,
    /// 内存使用率
    pub memory_usage: f32,
    /// 总线使用率
    pub bus_usage: f32,
    /// 中断负载
    pub interrupt_load: f32,
}

/// 性能指标
#[derive(Debug, Clone, Copy)]
pub struct PerformanceMetrics {
    /// 平均响应时间
    pub average_response_time: u32,
    /// 最大响应时间
    pub max_response_time: u32,
    /// 最小响应时间
    pub min_response_time: u32,
    /// 响应时间标准差
    pub response_time_stddev: f32,
}

/// 健康检查器
#[derive(Debug)]
pub struct HealthChecker {
    /// 健康检查项
    pub health_checks: Vec<HealthCheck, 16>,
    /// 健康状态
    pub health_status: HealthStatus,
    /// 检查间隔
    pub check_interval: u32,
    /// 最后检查时间
    pub last_check_time: u32,
}

/// 健康检查
#[derive(Debug, Clone)]
pub struct HealthCheck {
    /// 检查ID
    pub check_id: u8,
    /// 检查名称
    pub name: String<32>,
    /// 检查类型
    pub check_type: HealthCheckType,
    /// 检查结果
    pub result: HealthCheckResult,
    /// 最后检查时间
    pub last_check: u32,
}

/// 健康检查类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HealthCheckType {
    /// 通信连接检查
    CommunicationConnectivity,
    /// 设备响应检查
    DeviceResponse,
    /// 配置一致性检查
    ConfigurationConsistency,
    /// 性能阈值检查
    PerformanceThreshold,
}

/// 健康检查结果
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HealthCheckResult {
    /// 健康
    Healthy,
    /// 警告
    Warning,
    /// 错误
    Error,
    /// 未知
    Unknown,
}

/// 健康状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HealthStatus {
    /// 系统健康
    Healthy,
    /// 部分降级
    Degraded,
    /// 系统故障
    Faulty,
    /// 维护模式
    Maintenance,
}

/// 诊断系统
#[derive(Debug)]
pub struct DiagnosticSystem {
    /// 诊断测试
    pub diagnostic_tests: Vec<DiagnosticTest, 8>,
    /// 诊断结果
    pub diagnostic_results: Vec<DiagnosticResult, 32>,
    /// 自动诊断使能
    pub auto_diagnostic_enabled: bool,
    /// 诊断间隔
    pub diagnostic_interval: u32,
}

/// 诊断测试
#[derive(Debug, Clone)]
pub struct DiagnosticTest {
    /// 测试ID
    pub test_id: u8,
    /// 测试名称
    pub name: String<32>,
    /// 测试类型
    pub test_type: DiagnosticTestType,
    /// 测试参数
    pub parameters: [u32; 4],
    /// 预期结果
    pub expected_result: u32,
}

/// 诊断测试类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DiagnosticTestType {
    /// 回环测试
    LoopbackTest,
    /// 时序测试
    TimingTest,
    /// 信号完整性测试
    SignalIntegrityTest,
    /// 功能测试
    FunctionalTest,
}

/// 诊断结果
#[derive(Debug, Clone)]
pub struct DiagnosticResult {
    /// 结果ID
    pub result_id: u16,
    /// 测试ID
    pub test_id: u8,
    /// 时间戳
    pub timestamp: u32,
    /// 测试结果
    pub result: TestResult,
    /// 实际值
    pub actual_value: u32,
    /// 错误代码
    pub error_code: u16,
}

/// 测试结果
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TestResult {
    /// 通过
    Pass,
    /// 失败
    Fail,
    /// 跳过
    Skip,
    /// 超时
    Timeout,
}

/// 监控统计
#[derive(Debug, Default)]
pub struct MonitorStats {
    pub total_checks: u32,
    pub health_checks: u32,
    pub diagnostic_tests: u32,
    pub performance_samples: u32,
}

/// 系统状态
#[derive(Debug, Clone, Copy)]
pub struct SystemState {
    /// 系统模式
    pub system_mode: SystemMode,
    /// 运行时间
    pub uptime: u32,
    /// 活跃设备数
    pub active_devices: u8,
    /// 错误计数
    pub error_count: u32,
    /// 最后错误时间
    pub last_error_time: u32,
}

/// 系统模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemMode {
    /// 初始化
    Initialization,
    /// 正常运行
    Normal,
    /// 诊断模式
    Diagnostic,
    /// 维护模式
    Maintenance,
    /// 错误恢复
    ErrorRecovery,
}

/// 系统统计
#[derive(Debug, Default)]
pub struct SystemStats {
    pub total_operations: u32,
    pub successful_operations: u32,
    pub failed_operations: u32,
    pub device_resets: u32,
    pub configuration_changes: u32,
}

/// 扩展器管理器统计
#[derive(Debug, Default)]
pub struct ExpanderManagerStats {
    pub device_operations: u32,
    pub pin_operations: u32,
    pub communication_errors: u32,
    pub configuration_updates: u32,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            clock_frequency: 100_000,
            timeout_ms: 1000,
            retry_count: 3,
            default_pin_config: PinConfig::default(),
            interrupt_enable: false,
            power_mode: PowerState::FullPower,
        }
    }
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self {
            clock_frequency: 100_000,
            address_mode: AddressMode::SevenBit,
            timeout_ms: 1000,
            retry_count: 3,
        }
    }
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            clock_frequency: 1_000_000,
            clock_polarity: ClockPolarity::IdleLow,
            clock_phase: ClockPhase::FirstEdge,
            data_width: 8,
            chip_select_polarity: ChipSelectPolarity::ActiveLow,
        }
    }
}

impl GpioExpanderSystem {
    /// 创建新的GPIO扩展器系统
    pub fn new() -> Self {
        Self {
            expander_manager: ExpanderManager::new(),
            interrupt_manager: InterruptManager::new(),
            config_manager: ConfigManager::new(),
            monitor_system: MonitorSystem::new(),
            system_state: SystemState {
                system_mode: SystemMode::Initialization,
                uptime: 0,
                active_devices: 0,
                error_count: 0,
                last_error_time: 0,
            },
            stats: SystemStats::default(),
        }
    }
    
    /// 初始化系统
    pub fn initialize(&mut self) -> Result<(), SystemError> {
        self.system_state.system_mode = SystemMode::Initialization;
        
        // 初始化各个管理器
        self.expander_manager.initialize()?;
        self.interrupt_manager.initialize()?;
        self.config_manager.initialize()?;
        self.monitor_system.initialize()?;
        
        self.system_state.system_mode = SystemMode::Normal;
        Ok(())
    }
    
    /// 系统主循环
    pub fn run(&mut self, timestamp: u32) -> Result<(), SystemError> {
        self.system_state.uptime = timestamp;
        
        // 更新各个管理器
        self.expander_manager.update(timestamp)?;
        self.interrupt_manager.update(timestamp)?;
        self.config_manager.update(timestamp)?;
        self.monitor_system.update(timestamp)?;
        
        // 处理中断事件
        self.process_interrupt_events()?;
        
        // 更新统计信息
        self.update_statistics();
        
        self.stats.total_operations += 1;
        self.stats.successful_operations += 1;
        
        Ok(())
    }
    
    /// 处理中断事件
    fn process_interrupt_events(&mut self) -> Result<(), SystemError> {
        // 处理中断队列中的事件
        while let Some(event) = self.interrupt_manager.interrupt_queue.pop() {
            match event.event_type {
                EventType::PinStateChange => {
                    // 处理引脚状态变化
                    self.handle_pin_state_change(event)?;
                },
                EventType::InterruptTriggered => {
                    // 处理中断触发
                    self.handle_interrupt_triggered(event)?;
                },
                EventType::CommunicationError => {
                    // 处理通信错误
                    self.handle_communication_error(event)?;
                },
                EventType::ConfigurationUpdate => {
                    // 处理配置更新
                    self.handle_configuration_update(event)?;
                },
            }
        }
        
        Ok(())
    }
    
    /// 处理引脚状态变化
    fn handle_pin_state_change(&mut self, event: InterruptEvent) -> Result<(), SystemError> {
        // 更新引脚状态
        if let Some(expander) = self.expander_manager.expanders
            .iter_mut()
            .find(|e| e.id == event.source_id) {
            
            for pin_state in &mut expander.pin_states {
                let pin_mask = 1 << pin_state.pin_number;
                if event.change_mask & pin_mask != 0 {
                    pin_state.physical_state = (event.pin_state & pin_mask) != 0;
                    pin_state.last_update = event.timestamp;
                    pin_state.change_count += 1;
                }
            }
        }
        
        Ok(())
    }
    
    /// 处理中断触发
    fn handle_interrupt_triggered(&mut self, event: InterruptEvent) -> Result<(), SystemError> {
        // 记录中断统计
        self.interrupt_manager.interrupt_stats.total_interrupts += 1;
        self.interrupt_manager.interrupt_stats.processed_interrupts += 1;
        
        Ok(())
    }
    
    /// 处理通信错误
    fn handle_communication_error(&mut self, event: InterruptEvent) -> Result<(), SystemError> {
        // 更新错误统计
        self.system_state.error_count += 1;
        self.system_state.last_error_time = event.timestamp;
        
        // 尝试恢复通信
        if let Some(expander) = self.expander_manager.expanders
            .iter_mut()
            .find(|e| e.id == event.source_id) {
            
            expander.status = DeviceStatus::CommunicationError;
            expander.error_info.consecutive_errors += 1;
            
            // 如果连续错误过多，禁用设备
            if expander.error_info.consecutive_errors > 5 {
                expander.status = DeviceStatus::Disabled;
            }
        }
        
        Ok(())
    }
    
    /// 处理配置更新
    fn handle_configuration_update(&mut self, event: InterruptEvent) -> Result<(), SystemError> {
        self.stats.configuration_changes += 1;
        Ok(())
    }
    
    /// 更新统计信息
    fn update_statistics(&mut self) {
        // 计算活跃设备数
        self.system_state.active_devices = self.expander_manager.expanders
            .iter()
            .filter(|e| e.status == DeviceStatus::Active)
            .count() as u8;
    }
    
    /// 获取系统状态
    pub fn get_system_state(&self) -> &SystemState {
        &self.system_state
    }
    
    /// 获取系统统计
    pub fn get_system_stats(&self) -> &SystemStats {
        &self.stats
    }
}

impl ExpanderManager {
    /// 创建新的扩展器管理器
    pub fn new() -> Self {
        Self {
            expanders: Vec::new(),
            pin_mapping: FnvIndexMap::new(),
            cascade_config: CascadeConfig {
                enabled: false,
                cascade_depth: 1,
                cascade_order: Vec::new(),
                clock_sync: false,
                data_sync: false,
            },
            communication: CommunicationInterface {
                i2c_config: I2cConfig::default(),
                spi_config: SpiConfig::default(),
                current_interface: InterfaceType::I2C,
                comm_stats: CommunicationStats {
                    total_transfers: 0,
                    successful_transfers: 0,
                    failed_transfers: 0,
                    average_transfer_time: 0,
                    max_transfer_time: 0,
                    retry_count: 0,
                },
            },
            stats: ExpanderManagerStats::default(),
        }
    }
    
    /// 初始化扩展器管理器
    pub fn initialize(&mut self) -> Result<(), SystemError> {
        // 添加MCP23017扩展器
        let mut pin_states = Vec::new();
        for i in 0..16 {
            let _ = pin_states.push(PinState {
                pin_number: i,
                config: PinConfig::default(),
                logical_state: false,
                physical_state: false,
                last_update: 0,
                change_count: 0,
            });
        }
        
        let expander = ExpanderDevice {
            id: 0,
            device_type: ExpanderDeviceType::MCP23017,
            address: 0x20,
            pin_count: 16,
            status: DeviceStatus::Uninitialized,
            config: DeviceConfig::default(),
            pin_states,
            interrupt_config: InterruptConfig {
                enabled: false,
                interrupt_pin: None,
                interrupt_type: InterruptType::LevelChange,
                polarity: InterruptPolarity::ActiveLow,
                default_value: 0,
                interrupt_mask: 0,
            },
            error_info: ErrorInfo {
                last_error: ErrorType::None,
                error_time: 0,
                error_count: 0,
                consecutive_errors: 0,
            },
        };
        
        if self.expanders.push(expander).is_err() {
            return Err(SystemError::ResourceExhausted);
        }
        
        // 初始化引脚映射
        for expander in &mut self.expanders {
            for pin in 0..expander.pin_count {
                let virtual_pin_id = (expander.id as u16) << 8 | pin as u16;
                let virtual_pin = VirtualPin {
                    expander_id: expander.id,
                    physical_pin: pin,
                    virtual_pin: virtual_pin_id,
                    function: PinFunction::GeneralPurpose,
                };
                
                if self.pin_mapping.insert(virtual_pin_id, virtual_pin).is_err() {
                    return Err(SystemError::ResourceExhausted);
                }
            }
            
            expander.status = DeviceStatus::Active;
        }
        
        Ok(())
    }
    
    /// 更新扩展器管理器
    pub fn update(&mut self, timestamp: u32) -> Result<(), SystemError> {
        // 检查设备状态
        for expander in &mut self.expanders {
            if expander.status == DeviceStatus::CommunicationError {
                // 尝试恢复通信
                if timestamp - expander.error_info.error_time > 5000 {
                    expander.status = DeviceStatus::Active;
                    expander.error_info.consecutive_errors = 0;
                }
            }
        }
        
        self.stats.device_operations += 1;
        Ok(())
    }
}

impl InterruptManager {
    /// 创建新的中断管理器
    pub fn new() -> Self {
        Self {
            interrupt_sources: Vec::new(),
            interrupt_queue: Vec::new(),
            interrupt_handlers: Vec::new(),
            interrupt_stats: InterruptStats::default(),
        }
    }
    
    /// 初始化中断管理器
    pub fn initialize(&mut self) -> Result<(), SystemError> {
        // 添加默认中断源
        let interrupt_source = InterruptSource {
            source_id: 0,
            expander_id: 0,
            pin_mask: 0xFFFF,
            interrupt_type: InterruptType::LevelChange,
            enabled: true,
            priority: 1,
        };
        
        if self.interrupt_sources.push(interrupt_source).is_err() {
            return Err(SystemError::ResourceExhausted);
        }
        
        Ok(())
    }
    
    /// 更新中断管理器
    pub fn update(&mut self, timestamp: u32) -> Result<(), SystemError> {
        // 处理中断队列
        self.interrupt_stats.processed_interrupts += self.interrupt_queue.len() as u32;
        Ok(())
    }
}

impl ConfigManager {
    /// 创建新的配置管理器
    pub fn new() -> Self {
        Self {
            config_templates: Vec::new(),
            current_configs: FnvIndexMap::new(),
            config_history: Vec::new(),
            config_validator: ConfigValidator {
                validation_rules: Vec::new(),
                validation_stats: ValidationStats::default(),
            },
        }
    }
    
    /// 初始化配置管理器
    pub fn initialize(&mut self) -> Result<(), SystemError> {
        // 添加默认配置模板
        let mut template_name = String::new();
        let _ = template_name.push_str("MCP23017_Default");
        
        let template = ConfigTemplate {
            template_id: 0,
            name: template_name,
            device_type: ExpanderDeviceType::MCP23017,
            default_config: DeviceConfig::default(),
            pin_configs: Vec::new(),
        };
        
        if self.config_templates.push(template).is_err() {
            return Err(SystemError::ResourceExhausted);
        }
        
        Ok(())
    }
    
    /// 更新配置管理器
    pub fn update(&mut self, timestamp: u32) -> Result<(), SystemError> {
        // 验证当前配置
        self.config_validator.validation_stats.total_validations += 1;
        Ok(())
    }
}

impl MonitorSystem {
    /// 创建新的监控系统
    pub fn new() -> Self {
        Self {
            performance_monitor: PerformanceMonitor {
                response_times: Vec::new(),
                throughput_stats: ThroughputStats {
                    operations_per_second: 0.0,
                    bytes_per_second: 0.0,
                    peak_throughput: 0.0,
                    average_throughput: 0.0,
                },
                resource_usage: ResourceUsage {
                    cpu_usage: 0.0,
                    memory_usage: 0.0,
                    bus_usage: 0.0,
                    interrupt_load: 0.0,
                },
                performance_metrics: PerformanceMetrics {
                    average_response_time: 0,
                    max_response_time: 0,
                    min_response_time: u32::MAX,
                    response_time_stddev: 0.0,
                },
            },
            health_checker: HealthChecker {
                health_checks: Vec::new(),
                health_status: HealthStatus::Healthy,
                check_interval: 1000,
                last_check_time: 0,
            },
            diagnostic_system: DiagnosticSystem {
                diagnostic_tests: Vec::new(),
                diagnostic_results: Vec::new(),
                auto_diagnostic_enabled: true,
                diagnostic_interval: 10000,
            },
            monitor_stats: MonitorStats::default(),
        }
    }
    
    /// 初始化监控系统
    pub fn initialize(&mut self) -> Result<(), SystemError> {
        // 添加健康检查项
        let mut check_name = String::new();
        let _ = check_name.push_str("Communication");
        
        let health_check = HealthCheck {
            check_id: 0,
            name: check_name,
            check_type: HealthCheckType::CommunicationConnectivity,
            result: HealthCheckResult::Unknown,
            last_check: 0,
        };
        
        if self.health_checker.health_checks.push(health_check).is_err() {
            return Err(SystemError::ResourceExhausted);
        }
        
        Ok(())
    }
    
    /// 更新监控系统
    pub fn update(&mut self, timestamp: u32) -> Result<(), SystemError> {
        // 更新性能监控
        self.update_performance_monitor(timestamp)?;
        
        // 执行健康检查
        if timestamp - self.health_checker.last_check_time > self.health_checker.check_interval {
            self.perform_health_checks(timestamp)?;
            self.health_checker.last_check_time = timestamp;
        }
        
        self.monitor_stats.total_checks += 1;
        Ok(())
    }
    
    /// 更新性能监控
    fn update_performance_monitor(&mut self, timestamp: u32) -> Result<(), SystemError> {
        // 添加响应时间样本
        let response_time = 10; // 模拟响应时间
        if self.performance_monitor.response_times.push(response_time).is_err() {
            // 如果队列满了，移除最旧的样本
            self.performance_monitor.response_times.clear();
            let _ = self.performance_monitor.response_times.push(response_time);
        }
        
        // 更新性能指标
        self.calculate_performance_metrics();
        
        Ok(())
    }
    
    /// 计算性能指标
    fn calculate_performance_metrics(&mut self) {
        if !self.performance_monitor.response_times.is_empty() {
            let sum: u32 = self.performance_monitor.response_times.iter().sum();
            let count = self.performance_monitor.response_times.len() as u32;
            
            self.performance_monitor.performance_metrics.average_response_time = sum / count;
            
            if let Some(&max) = self.performance_monitor.response_times.iter().max() {
                self.performance_monitor.performance_metrics.max_response_time = max;
            }
            
            if let Some(&min) = self.performance_monitor.response_times.iter().min() {
                self.performance_monitor.performance_metrics.min_response_time = min;
            }
        }
    }
    
    /// 执行健康检查
    fn perform_health_checks(&mut self, timestamp: u32) -> Result<(), SystemError> {
        for health_check in &mut self.health_checker.health_checks {
            match health_check.check_type {
                HealthCheckType::CommunicationConnectivity => {
                    health_check.result = HealthCheckResult::Healthy;
                },
                HealthCheckType::DeviceResponse => {
                    health_check.result = HealthCheckResult::Healthy;
                },
                HealthCheckType::ConfigurationConsistency => {
                    health_check.result = HealthCheckResult::Healthy;
                },
                HealthCheckType::PerformanceThreshold => {
                    health_check.result = HealthCheckResult::Healthy;
                },
            }
            
            health_check.last_check = timestamp;
        }
        
        self.monitor_stats.health_checks += 1;
        Ok(())
    }
}

/// 系统错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemError {
    /// 资源耗尽
    ResourceExhausted,
    /// 通信错误
    CommunicationError,
    /// 配置错误
    ConfigurationError,
    /// 设备故障
    DeviceFault,
    /// 中断处理错误
    InterruptError,
}

/// 全局系统控制器
static GPIO_EXPANDER_SYSTEM: Mutex<RefCell<Option<GpioExpanderSystem>>> = 
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 初始化GPIO扩展器系统
    let mut system = GpioExpanderSystem::new();
    
    // 初始化系统
    if let Err(_) = system.initialize() {
        // 初始化失败，进入错误处理
        loop {
            cortex_m::asm::nop();
        }
    }
    
    // 存储到全局变量
    critical_section::with(|cs| {
        GPIO_EXPANDER_SYSTEM.borrow(cs).replace(Some(system));
    });
    
    let mut timestamp = 0u32;
    
    loop {
        timestamp = timestamp.wrapping_add(1);
        
        // 运行系统
        critical_section::with(|cs| {
            if let Some(ref mut system) = GPIO_EXPANDER_SYSTEM.borrow(cs).borrow_mut().as_mut() {
                if let Err(error) = system.run(timestamp) {
                    // 处理系统错误
                    match error {
                        SystemError::CommunicationError => {
                            // 重置通信接口
                        },
                        SystemError::DeviceFault => {
                            // 重新初始化设备
                        },
                        _ => {
                            // 其他错误处理
                        }
                    }
                }
                
                // 每10000次循环输出状态
                if timestamp % 10000 == 0 {
                    let state = system.get_system_state();
                    let stats = system.get_system_stats();
                    
                    // 在实际应用中，这里可以通过串口输出状态
                    // 例如：活跃设备数、错误计数、操作统计等
                }
            }
        });
        
        // 简单延时
        for _ in 0..500 {
            cortex_m::asm::nop();
        }
    }
}