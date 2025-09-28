//! 设备管理模块
//! 
//! 负责IoT网关中各种设备的管理、注册、发现和控制。
//! 支持I2C、SPI、UART、GPIO等多种接口的设备。

use heapless::{String, Vec, FnvIndexMap};
use serde::{Deserialize, Serialize};
use embedded_hal::blocking::delay::DelayMs;

pub mod manager;
pub mod registry;

pub use manager::{DeviceManager, DeviceManagerConfig, DeviceManagerStats};
pub use registry::{DeviceRegistry, DeviceRegistryConfig, DeviceInfo, DeviceStatus};

/// 设备管理错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum DeviceError {
    /// 设备未找到
    DeviceNotFound(String<32>),
    /// 设备已存在
    DeviceAlreadyExists(String<32>),
    /// 设备初始化失败
    InitializationFailed(String<32>),
    /// 设备通信错误
    CommunicationError(String<32>),
    /// 设备配置错误
    ConfigurationError(&'static str),
    /// 设备不支持的操作
    UnsupportedOperation,
    /// 设备忙碌
    DeviceBusy,
    /// 设备离线
    DeviceOffline,
    /// 资源不足
    ResourceExhausted,
    /// 超时
    Timeout,
}

/// 设备类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DeviceType {
    /// 传感器
    Sensor,
    /// 执行器
    Actuator,
    /// 显示器
    Display,
    /// 存储设备
    Storage,
    /// 通信设备
    Communication,
    /// 电源管理
    PowerManagement,
    /// 其他
    Other,
}

/// 设备接口类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum InterfaceType {
    /// I2C接口
    I2C,
    /// SPI接口
    SPI,
    /// UART接口
    UART,
    /// GPIO接口
    GPIO,
    /// ADC接口
    ADC,
    /// PWM接口
    PWM,
    /// 网络接口
    Network,
    /// 虚拟接口
    Virtual,
}

/// 设备状态
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceState {
    /// 未初始化
    Uninitialized,
    /// 初始化中
    Initializing,
    /// 就绪
    Ready,
    /// 运行中
    Running,
    /// 暂停
    Paused,
    /// 错误状态
    Error,
    /// 离线
    Offline,
    /// 维护模式
    Maintenance,
}

/// 设备优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum DevicePriority {
    /// 低优先级
    Low = 0,
    /// 普通优先级
    Normal = 1,
    /// 高优先级
    High = 2,
    /// 关键优先级
    Critical = 3,
}

/// 设备配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceConfig {
    /// 设备名称
    pub name: String<32>,
    /// 设备类型
    pub device_type: DeviceType,
    /// 接口类型
    pub interface_type: InterfaceType,
    /// 设备地址（I2C地址、SPI片选等）
    pub address: u32,
    /// 是否启用
    pub enabled: bool,
    /// 采样率（毫秒）
    pub sample_rate_ms: u32,
    /// 超时时间（毫秒）
    pub timeout_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 设备优先级
    pub priority: DevicePriority,
    /// 自动重启
    pub auto_restart: bool,
    /// 健康检查间隔（毫秒）
    pub health_check_interval_ms: u32,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            name: String::new(),
            device_type: DeviceType::Other,
            interface_type: InterfaceType::Virtual,
            address: 0,
            enabled: true,
            sample_rate_ms: 1000,
            timeout_ms: 5000,
            retry_count: 3,
            priority: DevicePriority::Normal,
            auto_restart: true,
            health_check_interval_ms: 60000,
        }
    }
}

/// 设备数据
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceData {
    /// 设备名称
    pub device_name: String<32>,
    /// 数据类型
    pub data_type: String<16>,
    /// 数据值
    pub value: f32,
    /// 单位
    pub unit: String<8>,
    /// 时间戳
    pub timestamp: u64,
    /// 数据质量
    pub quality: u8, // 0-100
}

impl Default for DeviceData {
    fn default() -> Self {
        Self {
            device_name: String::new(),
            data_type: String::new(),
            value: 0.0,
            unit: String::new(),
            timestamp: 0,
            quality: 100,
        }
    }
}

/// 设备命令
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceCommand {
    /// 目标设备名称
    pub target_device: String<32>,
    /// 命令类型
    pub command_type: String<16>,
    /// 命令参数
    pub parameters: Vec<(String<16>, f32), 8>,
    /// 命令ID
    pub command_id: String<16>,
    /// 超时时间（毫秒）
    pub timeout_ms: u32,
    /// 是否需要确认
    pub require_ack: bool,
}

impl Default for DeviceCommand {
    fn default() -> Self {
        Self {
            target_device: String::new(),
            command_type: String::new(),
            parameters: Vec::new(),
            command_id: String::new(),
            timeout_ms: 5000,
            require_ack: false,
        }
    }
}

/// 设备事件
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceEvent {
    /// 设备名称
    pub device_name: String<32>,
    /// 事件类型
    pub event_type: String<16>,
    /// 事件级别
    pub level: EventLevel,
    /// 事件消息
    pub message: String<64>,
    /// 时间戳
    pub timestamp: u64,
    /// 事件数据
    pub data: Option<DeviceData>,
}

/// 事件级别
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EventLevel {
    /// 调试
    Debug,
    /// 信息
    Info,
    /// 警告
    Warning,
    /// 错误
    Error,
    /// 严重错误
    Critical,
}

/// 设备统计信息
#[derive(Debug, Clone, Default)]
pub struct DeviceStats {
    /// 设备总数
    pub total_devices: usize,
    /// 在线设备数
    pub online_devices: usize,
    /// 错误设备数
    pub error_devices: usize,
    /// 总数据包数
    pub total_packets: u64,
    /// 成功数据包数
    pub successful_packets: u64,
    /// 失败数据包数
    pub failed_packets: u64,
    /// 平均响应时间（微秒）
    pub avg_response_time_us: u32,
    /// 设备重启次数
    pub device_restarts: u32,
    /// 通信错误次数
    pub communication_errors: u32,
}

/// 设备发现配置
#[derive(Debug, Clone)]
pub struct DeviceDiscoveryConfig {
    /// 启用自动发现
    pub auto_discovery_enabled: bool,
    /// 扫描间隔（毫秒）
    pub scan_interval_ms: u32,
    /// 扫描超时（毫秒）
    pub scan_timeout_ms: u32,
    /// 自动注册发现的设备
    pub auto_register: bool,
    /// 支持的接口类型
    pub supported_interfaces: Vec<InterfaceType, 8>,
}

impl Default for DeviceDiscoveryConfig {
    fn default() -> Self {
        let mut interfaces = Vec::new();
        let _ = interfaces.push(InterfaceType::I2C);
        let _ = interfaces.push(InterfaceType::SPI);
        let _ = interfaces.push(InterfaceType::UART);
        
        Self {
            auto_discovery_enabled: true,
            scan_interval_ms: 30000,
            scan_timeout_ms: 5000,
            auto_register: true,
            supported_interfaces: interfaces,
        }
    }
}

/// 设备trait定义
pub trait Device {
    /// 初始化设备
    fn init(&mut self) -> Result<(), DeviceError>;
    
    /// 读取设备数据
    fn read_data(&mut self) -> Result<DeviceData, DeviceError>;
    
    /// 写入设备数据
    fn write_data(&mut self, data: &DeviceData) -> Result<(), DeviceError>;
    
    /// 执行设备命令
    fn execute_command(&mut self, command: &DeviceCommand) -> Result<(), DeviceError>;
    
    /// 获取设备状态
    fn get_status(&self) -> DeviceState;
    
    /// 设置设备状态
    fn set_status(&mut self, state: DeviceState) -> Result<(), DeviceError>;
    
    /// 健康检查
    fn health_check(&mut self) -> Result<bool, DeviceError>;
    
    /// 重置设备
    fn reset(&mut self) -> Result<(), DeviceError>;
    
    /// 获取设备信息
    fn get_info(&self) -> DeviceInfo;
    
    /// 更新设备配置
    fn update_config(&mut self, config: &DeviceConfig) -> Result<(), DeviceError>;
}

/// 设备工厂trait
pub trait DeviceFactory {
    /// 创建设备实例
    fn create_device(&self, config: &DeviceConfig) -> Result<Box<dyn Device>, DeviceError>;
    
    /// 支持的设备类型
    fn supported_types(&self) -> Vec<String<32>, 16>;
    
    /// 验证设备配置
    fn validate_config(&self, config: &DeviceConfig) -> Result<(), DeviceError>;
}

/// 设备监听器trait
pub trait DeviceListener {
    /// 设备状态变化事件
    fn on_device_state_changed(&mut self, device_name: &str, old_state: DeviceState, new_state: DeviceState);
    
    /// 设备数据更新事件
    fn on_device_data_updated(&mut self, device_name: &str, data: &DeviceData);
    
    /// 设备错误事件
    fn on_device_error(&mut self, device_name: &str, error: &DeviceError);
    
    /// 设备事件
    fn on_device_event(&mut self, event: &DeviceEvent);
}

/// 设备工具函数
pub mod device_utils {
    use super::*;

    /// 创建默认设备配置
    pub fn create_default_config(name: &str, device_type: DeviceType, interface_type: InterfaceType) -> DeviceConfig {
        let mut config = DeviceConfig::default();
        config.name = String::from_str(name).unwrap_or_default();
        config.device_type = device_type;
        config.interface_type = interface_type;
        config
    }

    /// 验证设备名称
    pub fn validate_device_name(name: &str) -> Result<(), DeviceError> {
        if name.is_empty() {
            return Err(DeviceError::ConfigurationError("Device name cannot be empty"));
        }
        
        if name.len() > 32 {
            return Err(DeviceError::ConfigurationError("Device name too long"));
        }
        
        // 检查名称是否包含有效字符
        for c in name.chars() {
            if !c.is_alphanumeric() && c != '_' && c != '-' {
                return Err(DeviceError::ConfigurationError("Invalid character in device name"));
            }
        }
        
        Ok(())
    }

    /// 计算设备优先级权重
    pub fn calculate_priority_weight(priority: DevicePriority) -> u8 {
        match priority {
            DevicePriority::Low => 1,
            DevicePriority::Normal => 2,
            DevicePriority::High => 4,
            DevicePriority::Critical => 8,
        }
    }

    /// 检查设备兼容性
    pub fn check_device_compatibility(device_type: DeviceType, interface_type: InterfaceType) -> bool {
        match (device_type, interface_type) {
            (DeviceType::Sensor, InterfaceType::I2C) => true,
            (DeviceType::Sensor, InterfaceType::SPI) => true,
            (DeviceType::Sensor, InterfaceType::UART) => true,
            (DeviceType::Sensor, InterfaceType::ADC) => true,
            (DeviceType::Actuator, InterfaceType::I2C) => true,
            (DeviceType::Actuator, InterfaceType::GPIO) => true,
            (DeviceType::Actuator, InterfaceType::PWM) => true,
            (DeviceType::Display, InterfaceType::SPI) => true,
            (DeviceType::Display, InterfaceType::I2C) => true,
            (DeviceType::Storage, InterfaceType::SPI) => true,
            (DeviceType::Communication, InterfaceType::UART) => true,
            (DeviceType::Communication, InterfaceType::SPI) => true,
            (DeviceType::Communication, InterfaceType::Network) => true,
            _ => false,
        }
    }

    /// 生成设备ID
    pub fn generate_device_id(device_type: DeviceType, interface_type: InterfaceType, address: u32) -> String<32> {
        let type_str = match device_type {
            DeviceType::Sensor => "sen",
            DeviceType::Actuator => "act",
            DeviceType::Display => "disp",
            DeviceType::Storage => "stor",
            DeviceType::Communication => "comm",
            DeviceType::PowerManagement => "pwr",
            DeviceType::Other => "other",
        };
        
        let interface_str = match interface_type {
            InterfaceType::I2C => "i2c",
            InterfaceType::SPI => "spi",
            InterfaceType::UART => "uart",
            InterfaceType::GPIO => "gpio",
            InterfaceType::ADC => "adc",
            InterfaceType::PWM => "pwm",
            InterfaceType::Network => "net",
            InterfaceType::Virtual => "virt",
        };
        
        let id_str = format!("{}_{}_0x{:02x}", type_str, interface_str, address);
        String::from_str(&id_str).unwrap_or_default()
    }

    /// 解析设备地址
    pub fn parse_device_address(address_str: &str) -> Result<u32, DeviceError> {
        if address_str.starts_with("0x") || address_str.starts_with("0X") {
            // 十六进制地址
            u32::from_str_radix(&address_str[2..], 16)
                .map_err(|_| DeviceError::ConfigurationError("Invalid hex address"))
        } else {
            // 十进制地址
            address_str.parse::<u32>()
                .map_err(|_| DeviceError::ConfigurationError("Invalid decimal address"))
        }
    }

    /// 格式化设备地址
    pub fn format_device_address(address: u32, interface_type: InterfaceType) -> String<16> {
        match interface_type {
            InterfaceType::I2C => {
                let addr_str = format!("0x{:02X}", address);
                String::from_str(&addr_str).unwrap_or_default()
            },
            InterfaceType::SPI => {
                let addr_str = format!("CS{}", address);
                String::from_str(&addr_str).unwrap_or_default()
            },
            InterfaceType::GPIO => {
                let addr_str = format!("PIN{}", address);
                String::from_str(&addr_str).unwrap_or_default()
            },
            _ => {
                let addr_str = format!("{}", address);
                String::from_str(&addr_str).unwrap_or_default()
            }
        }
    }

    /// 计算设备健康评分
    pub fn calculate_health_score(
        success_rate: f32,
        response_time_ms: u32,
        error_count: u32,
        uptime_hours: u32,
    ) -> u8 {
        let mut score = 100.0;
        
        // 成功率影响（权重40%）
        score *= success_rate * 0.4 + 0.6;
        
        // 响应时间影响（权重30%）
        let response_factor = if response_time_ms <= 100 {
            1.0
        } else if response_time_ms <= 1000 {
            0.8
        } else {
            0.5
        };
        score *= response_factor * 0.3 + 0.7;
        
        // 错误计数影响（权重20%）
        let error_factor = if error_count == 0 {
            1.0
        } else if error_count <= 5 {
            0.8
        } else {
            0.5
        };
        score *= error_factor * 0.2 + 0.8;
        
        // 运行时间影响（权重10%）
        let uptime_factor = if uptime_hours >= 24 {
            1.0
        } else {
            uptime_hours as f32 / 24.0
        };
        score *= uptime_factor * 0.1 + 0.9;
        
        core::cmp::min(score as u8, 100)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_name_validation() {
        assert!(device_utils::validate_device_name("valid_name").is_ok());
        assert!(device_utils::validate_device_name("").is_err());
        assert!(device_utils::validate_device_name("invalid name with spaces").is_err());
    }

    #[test]
    fn test_device_compatibility() {
        assert!(device_utils::check_device_compatibility(DeviceType::Sensor, InterfaceType::I2C));
        assert!(!device_utils::check_device_compatibility(DeviceType::Display, InterfaceType::ADC));
    }

    #[test]
    fn test_address_parsing() {
        assert_eq!(device_utils::parse_device_address("0x44").unwrap(), 0x44);
        assert_eq!(device_utils::parse_device_address("68").unwrap(), 68);
        assert!(device_utils::parse_device_address("invalid").is_err());
    }

    #[test]
    fn test_health_score_calculation() {
        let score = device_utils::calculate_health_score(1.0, 50, 0, 48);
        assert_eq!(score, 100);
        
        let score = device_utils::calculate_health_score(0.8, 500, 3, 12);
        assert!(score < 100 && score > 50);
    }
}