//! 设备管理器
//! 
//! 负责IoT网关中所有设备的生命周期管理，包括初始化、监控、控制和维护。

use heapless::{String, Vec, FnvIndexMap, Deque};
use embedded_hal::blocking::delay::DelayMs;
use embassy_time::{Duration, Timer, Instant};

use super::{
    Device, DeviceError, DeviceConfig, DeviceData, DeviceCommand, DeviceEvent, DeviceState,
    DeviceType, InterfaceType, DevicePriority, EventLevel, DeviceStats, DeviceListener,
    device_utils,
};
use crate::protocols::{I2CManager, SPIManager, UARTManager};

/// 设备管理器配置
#[derive(Debug, Clone)]
pub struct DeviceManagerConfig {
    /// 最大设备数量
    pub max_devices: usize,
    /// 设备扫描间隔（毫秒）
    pub scan_interval_ms: u32,
    /// 健康检查间隔（毫秒）
    pub health_check_interval_ms: u32,
    /// 自动重启失败设备
    pub auto_restart_failed_devices: bool,
    /// 最大重启尝试次数
    pub max_restart_attempts: u8,
    /// 设备超时时间（毫秒）
    pub device_timeout_ms: u32,
    /// 启用设备发现
    pub discovery_enabled: bool,
    /// 事件队列大小
    pub event_queue_size: usize,
    /// 命令队列大小
    pub command_queue_size: usize,
}

impl Default for DeviceManagerConfig {
    fn default() -> Self {
        Self {
            max_devices: 32,
            scan_interval_ms: 30000,
            health_check_interval_ms: 60000,
            auto_restart_failed_devices: true,
            max_restart_attempts: 3,
            device_timeout_ms: 5000,
            discovery_enabled: true,
            event_queue_size: 64,
            command_queue_size: 32,
        }
    }
}

/// 设备管理器统计信息
#[derive(Debug, Clone, Default)]
pub struct DeviceManagerStats {
    /// 基础设备统计
    pub device_stats: DeviceStats,
    /// 设备发现次数
    pub discovery_scans: u32,
    /// 发现的新设备数
    pub devices_discovered: u32,
    /// 设备初始化成功次数
    pub successful_initializations: u32,
    /// 设备初始化失败次数
    pub failed_initializations: u32,
    /// 命令执行次数
    pub commands_executed: u32,
    /// 命令执行成功次数
    pub commands_successful: u32,
    /// 事件处理次数
    pub events_processed: u32,
    /// 平均设备响应时间（微秒）
    pub avg_device_response_time_us: u32,
    /// 系统运行时间（秒）
    pub uptime_seconds: u64,
}

/// 设备实例包装器
struct DeviceInstance {
    /// 设备实现
    device: Box<dyn Device>,
    /// 设备配置
    config: DeviceConfig,
    /// 当前状态
    state: DeviceState,
    /// 最后健康检查时间
    last_health_check: Instant,
    /// 最后数据更新时间
    last_data_update: Instant,
    /// 错误计数
    error_count: u32,
    /// 重启尝试次数
    restart_attempts: u8,
    /// 设备统计
    stats: DeviceInstanceStats,
}

/// 设备实例统计信息
#[derive(Debug, Clone, Default)]
struct DeviceInstanceStats {
    /// 数据读取次数
    pub read_count: u32,
    /// 数据写入次数
    pub write_count: u32,
    /// 命令执行次数
    pub command_count: u32,
    /// 成功操作次数
    pub success_count: u32,
    /// 失败操作次数
    pub failure_count: u32,
    /// 平均响应时间（微秒）
    pub avg_response_time_us: u32,
    /// 最后错误时间
    pub last_error_time: Option<Instant>,
}

impl DeviceInstance {
    fn new(device: Box<dyn Device>, config: DeviceConfig) -> Self {
        Self {
            device,
            config,
            state: DeviceState::Uninitialized,
            last_health_check: Instant::now(),
            last_data_update: Instant::now(),
            error_count: 0,
            restart_attempts: 0,
            stats: DeviceInstanceStats::default(),
        }
    }

    fn update_stats(&mut self, operation_time_us: u32, success: bool) {
        if success {
            self.stats.success_count += 1;
        } else {
            self.stats.failure_count += 1;
            self.stats.last_error_time = Some(Instant::now());
            self.error_count += 1;
        }

        // 更新平均响应时间
        let total_ops = self.stats.success_count + self.stats.failure_count;
        if total_ops == 1 {
            self.stats.avg_response_time_us = operation_time_us;
        } else {
            let alpha = 0.1; // 平滑因子
            let new_avg = (1.0 - alpha) * self.stats.avg_response_time_us as f32 + 
                         alpha * operation_time_us as f32;
            self.stats.avg_response_time_us = new_avg as u32;
        }
    }

    fn calculate_health_score(&self) -> u8 {
        let total_ops = self.stats.success_count + self.stats.failure_count;
        let success_rate = if total_ops > 0 {
            self.stats.success_count as f32 / total_ops as f32
        } else {
            1.0
        };

        let uptime_hours = self.last_health_check.elapsed().as_secs() / 3600;
        
        device_utils::calculate_health_score(
            success_rate,
            self.stats.avg_response_time_us / 1000, // 转换为毫秒
            self.error_count,
            uptime_hours as u32,
        )
    }
}

/// 设备管理器
pub struct DeviceManager {
    /// 配置
    config: DeviceManagerConfig,
    /// 设备实例映射
    devices: FnvIndexMap<String<32>, DeviceInstance, 32>,
    /// 事件队列
    event_queue: Deque<DeviceEvent, 64>,
    /// 命令队列
    command_queue: Deque<DeviceCommand, 32>,
    /// 设备监听器
    listeners: Vec<Box<dyn DeviceListener>, 8>,
    /// 统计信息
    stats: DeviceManagerStats,
    /// 最后扫描时间
    last_scan_time: Instant,
    /// 最后健康检查时间
    last_health_check_time: Instant,
    /// 启动时间
    start_time: Instant,
    /// I2C管理器
    i2c_manager: Option<I2CManager>,
    /// SPI管理器
    spi_manager: Option<SPIManager>,
    /// UART管理器
    uart_manager: Option<UARTManager>,
}

impl DeviceManager {
    /// 创建新的设备管理器
    pub fn new(config: DeviceManagerConfig) -> Self {
        let now = Instant::now();
        
        Self {
            config,
            devices: FnvIndexMap::new(),
            event_queue: Deque::new(),
            command_queue: Deque::new(),
            listeners: Vec::new(),
            stats: DeviceManagerStats::default(),
            last_scan_time: now,
            last_health_check_time: now,
            start_time: now,
            i2c_manager: None,
            spi_manager: None,
            uart_manager: None,
        }
    }

    /// 初始化设备管理器
    pub async fn init(&mut self) -> Result<(), DeviceError> {
        // 初始化协议管理器
        // self.i2c_manager = Some(I2CManager::new()?);
        // self.spi_manager = Some(SPIManager::new()?);
        // self.uart_manager = Some(UARTManager::new()?);

        // 如果启用设备发现，执行初始扫描
        if self.config.discovery_enabled {
            self.discover_devices().await?;
        }

        Ok(())
    }

    /// 注册设备
    pub fn register_device(&mut self, device: Box<dyn Device>, config: DeviceConfig) -> Result<(), DeviceError> {
        // 验证设备名称
        device_utils::validate_device_name(&config.name)?;

        // 检查设备是否已存在
        if self.devices.contains_key(&config.name) {
            return Err(DeviceError::DeviceAlreadyExists(config.name.clone()));
        }

        // 检查设备数量限制
        if self.devices.len() >= self.config.max_devices {
            return Err(DeviceError::ResourceExhausted);
        }

        // 检查设备兼容性
        if !device_utils::check_device_compatibility(config.device_type, config.interface_type) {
            return Err(DeviceError::ConfigurationError("Incompatible device type and interface"));
        }

        // 创建设备实例
        let device_instance = DeviceInstance::new(device, config.clone());
        
        // 添加到设备映射
        self.devices.insert(config.name.clone(), device_instance)
            .map_err(|_| DeviceError::ResourceExhausted)?;

        // 发送设备注册事件
        self.emit_event(DeviceEvent {
            device_name: config.name.clone(),
            event_type: String::from_str("device_registered").unwrap_or_default(),
            level: EventLevel::Info,
            message: String::from_str("Device registered successfully").unwrap_or_default(),
            timestamp: self.get_current_timestamp(),
            data: None,
        });

        Ok(())
    }

    /// 注销设备
    pub fn unregister_device(&mut self, device_name: &str) -> Result<(), DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        if self.devices.remove(&device_name_str).is_none() {
            return Err(DeviceError::DeviceNotFound(device_name_str));
        }

        // 发送设备注销事件
        self.emit_event(DeviceEvent {
            device_name: device_name_str,
            event_type: String::from_str("device_unregistered").unwrap_or_default(),
            level: EventLevel::Info,
            message: String::from_str("Device unregistered").unwrap_or_default(),
            timestamp: self.get_current_timestamp(),
            data: None,
        });

        Ok(())
    }

    /// 初始化设备
    pub async fn initialize_device(&mut self, device_name: &str) -> Result<(), DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        let device_instance = self.devices.get_mut(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str.clone()))?;

        if !device_instance.config.enabled {
            return Err(DeviceError::ConfigurationError("Device is disabled"));
        }

        device_instance.state = DeviceState::Initializing;
        
        let start_time = self.get_current_time_us();
        let result = device_instance.device.init();
        let operation_time = self.get_current_time_us() - start_time;

        match result {
            Ok(()) => {
                device_instance.state = DeviceState::Ready;
                device_instance.restart_attempts = 0;
                device_instance.update_stats(operation_time, true);
                self.stats.successful_initializations += 1;

                self.emit_event(DeviceEvent {
                    device_name: device_name_str,
                    event_type: String::from_str("device_initialized").unwrap_or_default(),
                    level: EventLevel::Info,
                    message: String::from_str("Device initialized successfully").unwrap_or_default(),
                    timestamp: self.get_current_timestamp(),
                    data: None,
                });

                Ok(())
            },
            Err(e) => {
                device_instance.state = DeviceState::Error;
                device_instance.update_stats(operation_time, false);
                self.stats.failed_initializations += 1;

                self.emit_event(DeviceEvent {
                    device_name: device_name_str,
                    event_type: String::from_str("device_init_failed").unwrap_or_default(),
                    level: EventLevel::Error,
                    message: String::from_str("Device initialization failed").unwrap_or_default(),
                    timestamp: self.get_current_timestamp(),
                    data: None,
                });

                Err(e)
            }
        }
    }

    /// 读取设备数据
    pub async fn read_device_data(&mut self, device_name: &str) -> Result<DeviceData, DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        let device_instance = self.devices.get_mut(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str.clone()))?;

        if device_instance.state != DeviceState::Ready && device_instance.state != DeviceState::Running {
            return Err(DeviceError::DeviceOffline);
        }

        let start_time = self.get_current_time_us();
        let result = device_instance.device.read_data();
        let operation_time = self.get_current_time_us() - start_time;

        match result {
            Ok(data) => {
                device_instance.stats.read_count += 1;
                device_instance.update_stats(operation_time, true);
                device_instance.last_data_update = Instant::now();
                self.stats.device_stats.successful_packets += 1;

                // 通知监听器
                for listener in &mut self.listeners {
                    listener.on_device_data_updated(device_name, &data);
                }

                Ok(data)
            },
            Err(e) => {
                device_instance.update_stats(operation_time, false);
                self.stats.device_stats.failed_packets += 1;

                // 通知监听器
                for listener in &mut self.listeners {
                    listener.on_device_error(device_name, &e);
                }

                Err(e)
            }
        }
    }

    /// 写入设备数据
    pub async fn write_device_data(&mut self, device_name: &str, data: &DeviceData) -> Result<(), DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        let device_instance = self.devices.get_mut(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str.clone()))?;

        if device_instance.state != DeviceState::Ready && device_instance.state != DeviceState::Running {
            return Err(DeviceError::DeviceOffline);
        }

        let start_time = self.get_current_time_us();
        let result = device_instance.device.write_data(data);
        let operation_time = self.get_current_time_us() - start_time;

        match result {
            Ok(()) => {
                device_instance.stats.write_count += 1;
                device_instance.update_stats(operation_time, true);
                Ok(())
            },
            Err(e) => {
                device_instance.update_stats(operation_time, false);
                Err(e)
            }
        }
    }

    /// 执行设备命令
    pub async fn execute_device_command(&mut self, command: &DeviceCommand) -> Result<(), DeviceError> {
        let device_instance = self.devices.get_mut(&command.target_device)
            .ok_or_else(|| DeviceError::DeviceNotFound(command.target_device.clone()))?;

        if device_instance.state != DeviceState::Ready && device_instance.state != DeviceState::Running {
            return Err(DeviceError::DeviceOffline);
        }

        let start_time = self.get_current_time_us();
        let result = device_instance.device.execute_command(command);
        let operation_time = self.get_current_time_us() - start_time;

        match result {
            Ok(()) => {
                device_instance.stats.command_count += 1;
                device_instance.update_stats(operation_time, true);
                self.stats.commands_executed += 1;
                self.stats.commands_successful += 1;
                Ok(())
            },
            Err(e) => {
                device_instance.update_stats(operation_time, false);
                self.stats.commands_executed += 1;
                Err(e)
            }
        }
    }

    /// 添加命令到队列
    pub fn enqueue_command(&mut self, command: DeviceCommand) -> Result<(), DeviceError> {
        self.command_queue.push_back(command)
            .map_err(|_| DeviceError::ResourceExhausted)
    }

    /// 处理命令队列
    pub async fn process_command_queue(&mut self) -> Result<(), DeviceError> {
        while let Some(command) = self.command_queue.pop_front() {
            if let Err(e) = self.execute_device_command(&command).await {
                // 记录命令执行失败
                self.emit_event(DeviceEvent {
                    device_name: command.target_device,
                    event_type: String::from_str("command_failed").unwrap_or_default(),
                    level: EventLevel::Error,
                    message: String::from_str("Command execution failed").unwrap_or_default(),
                    timestamp: self.get_current_timestamp(),
                    data: None,
                });
            }
        }
        Ok(())
    }

    /// 设备发现
    pub async fn discover_devices(&mut self) -> Result<Vec<DeviceConfig, 16>, DeviceError> {
        let mut discovered_devices = Vec::new();
        
        self.stats.discovery_scans += 1;
        
        // I2C设备发现
        if let Some(_i2c_manager) = &mut self.i2c_manager {
            // 扫描I2C总线上的设备
            for address in 0x08..=0x77 {
                // 这里应该实际尝试与设备通信
                // 为了演示，我们模拟发现一些设备
                if address == 0x44 || address == 0x76 || address == 0x23 {
                    let mut config = DeviceConfig::default();
                    config.name = String::from_str(&format!("i2c_device_0x{:02x}", address)).unwrap_or_default();
                    config.device_type = DeviceType::Sensor;
                    config.interface_type = InterfaceType::I2C;
                    config.address = address;
                    
                    if discovered_devices.push(config).is_err() {
                        break;
                    }
                    self.stats.devices_discovered += 1;
                }
            }
        }

        // SPI设备发现（通过配置文件或其他方式）
        // UART设备发现（通过配置文件或其他方式）

        self.last_scan_time = Instant::now();
        Ok(discovered_devices)
    }

    /// 健康检查
    pub async fn health_check(&mut self) -> Result<(), DeviceError> {
        let current_time = Instant::now();
        
        for (device_name, device_instance) in self.devices.iter_mut() {
            // 检查是否需要进行健康检查
            if current_time.duration_since(device_instance.last_health_check).as_millis() 
                >= device_instance.config.health_check_interval_ms as u64 {
                
                let start_time = self.get_current_time_us();
                let health_result = device_instance.device.health_check();
                let operation_time = self.get_current_time_us() - start_time;

                match health_result {
                    Ok(is_healthy) => {
                        device_instance.update_stats(operation_time, true);
                        
                        if !is_healthy {
                            device_instance.state = DeviceState::Error;
                            
                            // 如果启用自动重启，尝试重启设备
                            if self.config.auto_restart_failed_devices && 
                               device_instance.restart_attempts < self.config.max_restart_attempts {
                                
                                if let Err(_) = self.restart_device(device_name).await {
                                    device_instance.restart_attempts += 1;
                                }
                            }
                        } else if device_instance.state == DeviceState::Error {
                            device_instance.state = DeviceState::Ready;
                            device_instance.error_count = 0;
                        }
                    },
                    Err(e) => {
                        device_instance.update_stats(operation_time, false);
                        device_instance.state = DeviceState::Error;
                        
                        // 通知监听器
                        for listener in &mut self.listeners {
                            listener.on_device_error(device_name, &e);
                        }
                    }
                }
                
                device_instance.last_health_check = current_time;
            }
        }

        self.last_health_check_time = current_time;
        Ok(())
    }

    /// 重启设备
    pub async fn restart_device(&mut self, device_name: &str) -> Result<(), DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        let device_instance = self.devices.get_mut(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str.clone()))?;

        // 重置设备
        let _ = device_instance.device.reset();
        
        // 等待一段时间
        Timer::after(Duration::from_millis(1000)).await;
        
        // 重新初始化
        self.initialize_device(device_name).await?;
        
        self.stats.device_stats.device_restarts += 1;
        
        self.emit_event(DeviceEvent {
            device_name: device_name_str,
            event_type: String::from_str("device_restarted").unwrap_or_default(),
            level: EventLevel::Info,
            message: String::from_str("Device restarted successfully").unwrap_or_default(),
            timestamp: self.get_current_timestamp(),
            data: None,
        });

        Ok(())
    }

    /// 更新系统
    pub async fn update(&mut self) -> Result<(), DeviceError> {
        let current_time = Instant::now();

        // 处理命令队列
        self.process_command_queue().await?;

        // 处理事件队列
        self.process_event_queue().await?;

        // 定期设备发现
        if self.config.discovery_enabled && 
           current_time.duration_since(self.last_scan_time).as_millis() >= self.config.scan_interval_ms as u64 {
            let _ = self.discover_devices().await;
        }

        // 定期健康检查
        if current_time.duration_since(self.last_health_check_time).as_millis() >= self.config.health_check_interval_ms as u64 {
            let _ = self.health_check().await;
        }

        // 更新统计信息
        self.update_stats();

        Ok(())
    }

    /// 处理事件队列
    async fn process_event_queue(&mut self) -> Result<(), DeviceError> {
        while let Some(event) = self.event_queue.pop_front() {
            // 通知所有监听器
            for listener in &mut self.listeners {
                listener.on_device_event(&event);
            }
            
            self.stats.events_processed += 1;
        }
        Ok(())
    }

    /// 发送事件
    fn emit_event(&mut self, event: DeviceEvent) {
        if self.event_queue.push_back(event).is_err() {
            // 队列满，移除最旧的事件
            self.event_queue.pop_front();
            let _ = self.event_queue.push_back(event);
        }
    }

    /// 添加设备监听器
    pub fn add_listener(&mut self, listener: Box<dyn DeviceListener>) -> Result<(), DeviceError> {
        self.listeners.push(listener)
            .map_err(|_| DeviceError::ResourceExhausted)
    }

    /// 获取设备状态
    pub fn get_device_state(&self, device_name: &str) -> Result<DeviceState, DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        let device_instance = self.devices.get(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str))?;

        Ok(device_instance.state)
    }

    /// 获取所有设备列表
    pub fn get_device_list(&self) -> Vec<String<32>, 32> {
        let mut device_list = Vec::new();
        
        for device_name in self.devices.keys() {
            if device_list.push(device_name.clone()).is_err() {
                break;
            }
        }
        
        device_list
    }

    /// 获取设备统计信息
    pub fn get_device_stats(&self, device_name: &str) -> Result<DeviceInstanceStats, DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        let device_instance = self.devices.get(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str))?;

        Ok(device_instance.stats.clone())
    }

    /// 更新统计信息
    fn update_stats(&mut self) {
        // 更新设备统计
        self.stats.device_stats.total_devices = self.devices.len();
        self.stats.device_stats.online_devices = self.devices.values()
            .filter(|d| d.state == DeviceState::Ready || d.state == DeviceState::Running)
            .count();
        self.stats.device_stats.error_devices = self.devices.values()
            .filter(|d| d.state == DeviceState::Error)
            .count();

        // 计算平均响应时间
        let total_response_time: u64 = self.devices.values()
            .map(|d| d.stats.avg_response_time_us as u64)
            .sum();
        
        if !self.devices.is_empty() {
            self.stats.avg_device_response_time_us = (total_response_time / self.devices.len() as u64) as u32;
        }

        // 更新运行时间
        self.stats.uptime_seconds = self.start_time.elapsed().as_secs();
    }

    /// 获取管理器统计信息
    pub fn get_stats(&self) -> DeviceManagerStats {
        self.stats.clone()
    }

    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats = DeviceManagerStats::default();
        
        // 重置所有设备的统计信息
        for device_instance in self.devices.values_mut() {
            device_instance.stats = DeviceInstanceStats::default();
            device_instance.error_count = 0;
            device_instance.restart_attempts = 0;
        }
    }

    /// 更新配置
    pub fn update_config(&mut self, config: DeviceManagerConfig) -> Result<(), DeviceError> {
        // 验证配置
        if config.max_devices == 0 {
            return Err(DeviceError::ConfigurationError("Max devices cannot be zero"));
        }

        if config.event_queue_size == 0 {
            return Err(DeviceError::ConfigurationError("Event queue size cannot be zero"));
        }

        self.config = config;
        Ok(())
    }

    /// 获取当前时间戳
    fn get_current_timestamp(&self) -> u64 {
        self.start_time.elapsed().as_millis() as u64
    }

    /// 获取当前时间（微秒）
    fn get_current_time_us(&self) -> u32 {
        self.start_time.elapsed().as_micros() as u32
    }

    /// 系统维护
    pub async fn maintenance(&mut self) -> Result<(), DeviceError> {
        // 清理过期事件
        // 重置错误计数器
        for device_instance in self.devices.values_mut() {
            if device_instance.error_count > 100 {
                device_instance.error_count = 0;
            }
        }

        // 执行设备维护
        for (device_name, device_instance) in self.devices.iter_mut() {
            if device_instance.state == DeviceState::Ready || device_instance.state == DeviceState::Running {
                // 可以在这里执行设备特定的维护任务
            }
        }

        Ok(())
    }

    /// 关闭设备管理器
    pub async fn shutdown(&mut self) -> Result<(), DeviceError> {
        // 停止所有设备
        for (_, device_instance) in self.devices.iter_mut() {
            let _ = device_instance.device.set_status(DeviceState::Offline);
        }

        // 清空队列
        self.event_queue.clear();
        self.command_queue.clear();

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 模拟设备实现
    struct MockDevice {
        state: DeviceState,
        fail_operations: bool,
    }

    impl MockDevice {
        fn new() -> Self {
            Self {
                state: DeviceState::Uninitialized,
                fail_operations: false,
            }
        }
    }

    impl Device for MockDevice {
        fn init(&mut self) -> Result<(), DeviceError> {
            if self.fail_operations {
                Err(DeviceError::InitializationFailed(String::from_str("Mock failure").unwrap_or_default()))
            } else {
                self.state = DeviceState::Ready;
                Ok(())
            }
        }

        fn read_data(&mut self) -> Result<DeviceData, DeviceError> {
            if self.fail_operations {
                Err(DeviceError::CommunicationError(String::from_str("Mock failure").unwrap_or_default()))
            } else {
                Ok(DeviceData::default())
            }
        }

        fn write_data(&mut self, _data: &DeviceData) -> Result<(), DeviceError> {
            if self.fail_operations {
                Err(DeviceError::CommunicationError(String::from_str("Mock failure").unwrap_or_default()))
            } else {
                Ok(())
            }
        }

        fn execute_command(&mut self, _command: &DeviceCommand) -> Result<(), DeviceError> {
            if self.fail_operations {
                Err(DeviceError::UnsupportedOperation)
            } else {
                Ok(())
            }
        }

        fn get_status(&self) -> DeviceState {
            self.state
        }

        fn set_status(&mut self, state: DeviceState) -> Result<(), DeviceError> {
            self.state = state;
            Ok(())
        }

        fn health_check(&mut self) -> Result<bool, DeviceError> {
            Ok(!self.fail_operations)
        }

        fn reset(&mut self) -> Result<(), DeviceError> {
            self.state = DeviceState::Uninitialized;
            Ok(())
        }

        fn get_info(&self) -> crate::devices::DeviceInfo {
            crate::devices::DeviceInfo::default()
        }

        fn update_config(&mut self, _config: &DeviceConfig) -> Result<(), DeviceError> {
            Ok(())
        }
    }

    #[tokio::test]
    async fn test_device_registration() {
        let mut manager = DeviceManager::new(DeviceManagerConfig::default());
        let device = Box::new(MockDevice::new());
        let config = DeviceConfig {
            name: String::from_str("test_device").unwrap(),
            device_type: DeviceType::Sensor,
            interface_type: InterfaceType::I2C,
            address: 0x44,
            ..Default::default()
        };

        assert!(manager.register_device(device, config).is_ok());
        assert_eq!(manager.get_device_list().len(), 1);
    }

    #[tokio::test]
    async fn test_device_initialization() {
        let mut manager = DeviceManager::new(DeviceManagerConfig::default());
        let device = Box::new(MockDevice::new());
        let config = DeviceConfig {
            name: String::from_str("test_device").unwrap(),
            device_type: DeviceType::Sensor,
            interface_type: InterfaceType::I2C,
            address: 0x44,
            ..Default::default()
        };

        manager.register_device(device, config).unwrap();
        assert!(manager.initialize_device("test_device").await.is_ok());
        assert_eq!(manager.get_device_state("test_device").unwrap(), DeviceState::Ready);
    }
}