//! 协议处理模块
//! 
//! 负责IoT网关中各种通信协议的处理，包括I2C、SPI、UART等硬件协议。
//! 提供统一的协议接口和数据传输管理。

use heapless::{String, Vec, FnvIndexMap};
use serde::{Deserialize, Serialize};
use embedded_hal::blocking::delay::DelayMs;

pub mod i2c;
pub mod spi;
pub mod uart;

pub use i2c::{I2CManager, I2CConfig, I2CError, I2CDevice};
pub use spi::{SPIManager, SPIConfig, SPIError, SPIDevice};
pub use uart::{UARTManager, UARTConfig, UARTError, UARTDevice};

/// 协议错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum ProtocolError {
    /// I2C错误
    I2C(I2CError),
    /// SPI错误
    SPI(SPIError),
    /// UART错误
    UART(UARTError),
    /// 协议不支持
    UnsupportedProtocol,
    /// 设备忙碌
    DeviceBusy,
    /// 超时
    Timeout,
    /// 配置错误
    ConfigError(&'static str),
    /// 缓冲区错误
    BufferError,
    /// 校验和错误
    ChecksumError,
    /// 数据格式错误
    DataFormatError,
}

/// 协议类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ProtocolType {
    /// I2C协议
    I2C,
    /// SPI协议
    SPI,
    /// UART协议
    UART,
    /// 1-Wire协议
    OneWire,
    /// CAN协议
    CAN,
    /// Modbus协议
    Modbus,
    /// 自定义协议
    Custom,
}

/// 数据传输模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferMode {
    /// 阻塞模式
    Blocking,
    /// 非阻塞模式
    NonBlocking,
    /// DMA模式
    DMA,
    /// 中断模式
    Interrupt,
}

/// 协议配置
#[derive(Debug, Clone)]
pub struct ProtocolConfig {
    /// 协议类型
    pub protocol_type: ProtocolType,
    /// 传输模式
    pub transfer_mode: TransferMode,
    /// 超时时间（毫秒）
    pub timeout_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 缓冲区大小
    pub buffer_size: usize,
    /// 启用校验和
    pub checksum_enabled: bool,
    /// 启用错误恢复
    pub error_recovery: bool,
    /// 调试模式
    pub debug_mode: bool,
}

impl Default for ProtocolConfig {
    fn default() -> Self {
        Self {
            protocol_type: ProtocolType::I2C,
            transfer_mode: TransferMode::Blocking,
            timeout_ms: 1000,
            retry_count: 3,
            buffer_size: 256,
            checksum_enabled: false,
            error_recovery: true,
            debug_mode: false,
        }
    }
}

/// 协议数据包
#[derive(Debug, Clone)]
pub struct ProtocolPacket {
    /// 协议类型
    pub protocol: ProtocolType,
    /// 设备地址
    pub address: u32,
    /// 寄存器地址（如果适用）
    pub register: Option<u16>,
    /// 数据负载
    pub data: Vec<u8, 256>,
    /// 校验和
    pub checksum: Option<u16>,
    /// 时间戳
    pub timestamp: u64,
    /// 数据方向（读/写）
    pub direction: DataDirection,
}

/// 数据方向
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataDirection {
    /// 读取
    Read,
    /// 写入
    Write,
    /// 读写
    ReadWrite,
}

impl Default for ProtocolPacket {
    fn default() -> Self {
        Self {
            protocol: ProtocolType::I2C,
            address: 0,
            register: None,
            data: Vec::new(),
            checksum: None,
            timestamp: 0,
            direction: DataDirection::Read,
        }
    }
}

impl ProtocolPacket {
    /// 创建新的协议数据包
    pub fn new(protocol: ProtocolType, address: u32, direction: DataDirection) -> Self {
        Self {
            protocol,
            address,
            direction,
            timestamp: Self::get_current_timestamp(),
            ..Default::default()
        }
    }

    /// 设置数据
    pub fn set_data(&mut self, data: &[u8]) -> Result<(), ProtocolError> {
        self.data.clear();
        self.data.extend_from_slice(data)
            .map_err(|_| ProtocolError::BufferError)?;
        Ok(())
    }

    /// 设置寄存器地址
    pub fn set_register(&mut self, register: u16) {
        self.register = Some(register);
    }

    /// 计算校验和
    pub fn calculate_checksum(&self) -> u16 {
        let mut checksum = 0u16;
        
        // 包含地址
        checksum = checksum.wrapping_add(self.address as u16);
        
        // 包含寄存器（如果有）
        if let Some(reg) = self.register {
            checksum = checksum.wrapping_add(reg);
        }
        
        // 包含数据
        for &byte in &self.data {
            checksum = checksum.wrapping_add(byte as u16);
        }
        
        checksum
    }

    /// 验证校验和
    pub fn verify_checksum(&self) -> bool {
        if let Some(expected_checksum) = self.checksum {
            self.calculate_checksum() == expected_checksum
        } else {
            true // 没有校验和时认为有效
        }
    }

    /// 获取当前时间戳
    fn get_current_timestamp() -> u64 {
        // 这里应该使用实际的时间获取函数
        1000000 // 模拟时间戳
    }
}

/// 协议统计信息
#[derive(Debug, Clone, Default)]
pub struct ProtocolStats {
    /// 发送的数据包数
    pub packets_sent: u64,
    /// 接收的数据包数
    pub packets_received: u64,
    /// 发送失败的数据包数
    pub send_failures: u32,
    /// 接收失败的数据包数
    pub receive_failures: u32,
    /// 校验和错误数
    pub checksum_errors: u32,
    /// 超时次数
    pub timeouts: u32,
    /// 重试次数
    pub retries: u32,
    /// 平均传输时间（微秒）
    pub avg_transfer_time_us: u32,
    /// 总传输字节数
    pub total_bytes_transferred: u64,
    /// 错误恢复次数
    pub error_recoveries: u32,
}

/// 协议管理器trait
pub trait ProtocolManager {
    /// 初始化协议管理器
    fn init(&mut self) -> Result<(), ProtocolError>;
    
    /// 发送数据包
    fn send_packet(&mut self, packet: &ProtocolPacket) -> Result<(), ProtocolError>;
    
    /// 接收数据包
    fn receive_packet(&mut self) -> Result<ProtocolPacket, ProtocolError>;
    
    /// 读取数据
    fn read_data(&mut self, address: u32, register: Option<u16>, length: usize) -> Result<Vec<u8, 256>, ProtocolError>;
    
    /// 写入数据
    fn write_data(&mut self, address: u32, register: Option<u16>, data: &[u8]) -> Result<(), ProtocolError>;
    
    /// 检查设备是否存在
    fn device_exists(&mut self, address: u32) -> Result<bool, ProtocolError>;
    
    /// 扫描设备
    fn scan_devices(&mut self) -> Result<Vec<u32, 128>, ProtocolError>;
    
    /// 获取统计信息
    fn get_stats(&self) -> &ProtocolStats;
    
    /// 重置统计信息
    fn reset_stats(&mut self);
    
    /// 设置配置
    fn set_config(&mut self, config: ProtocolConfig) -> Result<(), ProtocolError>;
    
    /// 获取配置
    fn get_config(&self) -> &ProtocolConfig;
}

/// 协议设备trait
pub trait ProtocolDevice {
    /// 读取寄存器
    fn read_register(&mut self, register: u16) -> Result<u8, ProtocolError>;
    
    /// 写入寄存器
    fn write_register(&mut self, register: u16, value: u8) -> Result<(), ProtocolError>;
    
    /// 读取多个寄存器
    fn read_registers(&mut self, start_register: u16, count: usize) -> Result<Vec<u8, 256>, ProtocolError>;
    
    /// 写入多个寄存器
    fn write_registers(&mut self, start_register: u16, data: &[u8]) -> Result<(), ProtocolError>;
    
    /// 读取数据块
    fn read_block(&mut self, length: usize) -> Result<Vec<u8, 256>, ProtocolError>;
    
    /// 写入数据块
    fn write_block(&mut self, data: &[u8]) -> Result<(), ProtocolError>;
    
    /// 获取设备地址
    fn get_address(&self) -> u32;
    
    /// 设置设备地址
    fn set_address(&mut self, address: u32);
    
    /// 检查设备连接
    fn is_connected(&mut self) -> Result<bool, ProtocolError>;
    
    /// 重置设备
    fn reset(&mut self) -> Result<(), ProtocolError>;
}

/// 协议工厂trait
pub trait ProtocolFactory {
    /// 创建协议管理器
    fn create_manager(&self, config: ProtocolConfig) -> Result<Box<dyn ProtocolManager>, ProtocolError>;
    
    /// 创建协议设备
    fn create_device(&self, address: u32, config: ProtocolConfig) -> Result<Box<dyn ProtocolDevice>, ProtocolError>;
    
    /// 支持的协议类型
    fn supported_protocols(&self) -> Vec<ProtocolType, 8>;
    
    /// 验证配置
    fn validate_config(&self, config: &ProtocolConfig) -> Result<(), ProtocolError>;
}

/// 协议工具函数
pub mod protocol_utils {
    use super::*;

    /// 创建I2C配置
    pub fn create_i2c_config(frequency: u32, timeout_ms: u32) -> ProtocolConfig {
        ProtocolConfig {
            protocol_type: ProtocolType::I2C,
            timeout_ms,
            ..Default::default()
        }
    }

    /// 创建SPI配置
    pub fn create_spi_config(frequency: u32, mode: u8, timeout_ms: u32) -> ProtocolConfig {
        ProtocolConfig {
            protocol_type: ProtocolType::SPI,
            timeout_ms,
            ..Default::default()
        }
    }

    /// 创建UART配置
    pub fn create_uart_config(baud_rate: u32, timeout_ms: u32) -> ProtocolConfig {
        ProtocolConfig {
            protocol_type: ProtocolType::UART,
            timeout_ms,
            ..Default::default()
        }
    }

    /// 验证协议配置
    pub fn validate_protocol_config(config: &ProtocolConfig) -> Result<(), ProtocolError> {
        if config.timeout_ms == 0 {
            return Err(ProtocolError::ConfigError("Timeout cannot be zero"));
        }

        if config.buffer_size == 0 {
            return Err(ProtocolError::ConfigError("Buffer size cannot be zero"));
        }

        if config.retry_count > 10 {
            return Err(ProtocolError::ConfigError("Too many retry attempts"));
        }

        Ok(())
    }

    /// 计算传输效率
    pub fn calculate_transfer_efficiency(stats: &ProtocolStats) -> f32 {
        let total_packets = stats.packets_sent + stats.packets_received;
        let failed_packets = stats.send_failures + stats.receive_failures;
        
        if total_packets == 0 {
            return 0.0;
        }
        
        let successful_packets = total_packets - failed_packets as u64;
        successful_packets as f32 / total_packets as f32
    }

    /// 计算平均吞吐量（字节/秒）
    pub fn calculate_throughput(stats: &ProtocolStats, duration_seconds: u32) -> f32 {
        if duration_seconds == 0 {
            return 0.0;
        }
        
        stats.total_bytes_transferred as f32 / duration_seconds as f32
    }

    /// 检查协议兼容性
    pub fn check_protocol_compatibility(protocol1: ProtocolType, protocol2: ProtocolType) -> bool {
        match (protocol1, protocol2) {
            (ProtocolType::I2C, ProtocolType::I2C) => true,
            (ProtocolType::SPI, ProtocolType::SPI) => true,
            (ProtocolType::UART, ProtocolType::UART) => true,
            (ProtocolType::UART, ProtocolType::Modbus) => true,
            (ProtocolType::Modbus, ProtocolType::UART) => true,
            _ => false,
        }
    }

    /// 估算传输时间
    pub fn estimate_transfer_time(
        protocol: ProtocolType,
        data_length: usize,
        frequency: u32,
    ) -> u32 {
        match protocol {
            ProtocolType::I2C => {
                // I2C传输时间估算：地址 + 数据 + ACK/NACK
                let bits = (1 + data_length) * 9; // 每字节9位（包括ACK）
                (bits as u32 * 1_000_000) / frequency
            },
            ProtocolType::SPI => {
                // SPI传输时间估算：纯数据传输
                let bits = data_length * 8;
                (bits as u32 * 1_000_000) / frequency
            },
            ProtocolType::UART => {
                // UART传输时间估算：起始位 + 数据位 + 停止位
                let bits = data_length * 10; // 每字节10位
                (bits as u32 * 1_000_000) / frequency
            },
            _ => {
                // 其他协议的默认估算
                (data_length as u32 * 1000) / (frequency / 1000)
            }
        }
    }

    /// 选择最佳协议
    pub fn select_best_protocol(
        requirements: &TransferRequirements,
        available_protocols: &[ProtocolType],
    ) -> Option<ProtocolType> {
        let mut best_protocol = None;
        let mut best_score = 0.0;

        for &protocol in available_protocols {
            let score = calculate_protocol_score(protocol, requirements);
            if score > best_score {
                best_score = score;
                best_protocol = Some(protocol);
            }
        }

        best_protocol
    }

    /// 计算协议评分
    fn calculate_protocol_score(protocol: ProtocolType, requirements: &TransferRequirements) -> f32 {
        let mut score = 0.0;

        // 速度评分
        match protocol {
            ProtocolType::SPI => score += 3.0,
            ProtocolType::I2C => score += 2.0,
            ProtocolType::UART => score += 1.0,
            _ => score += 1.0,
        }

        // 可靠性评分
        match protocol {
            ProtocolType::I2C => score += 2.0, // 内置ACK
            ProtocolType::SPI => score += 1.5,
            ProtocolType::UART => score += 1.0,
            _ => score += 1.0,
        }

        // 复杂性评分（越简单越好）
        match protocol {
            ProtocolType::SPI => score += 2.0,
            ProtocolType::UART => score += 1.5,
            ProtocolType::I2C => score += 1.0,
            _ => score += 0.5,
        }

        // 根据需求调整评分
        if requirements.high_speed && protocol == ProtocolType::SPI {
            score += 2.0;
        }

        if requirements.multi_device && protocol == ProtocolType::I2C {
            score += 2.0;
        }

        if requirements.long_distance && protocol == ProtocolType::UART {
            score += 2.0;
        }

        score
    }

    /// 传输需求
    pub struct TransferRequirements {
        /// 高速传输需求
        pub high_speed: bool,
        /// 多设备支持需求
        pub multi_device: bool,
        /// 长距离传输需求
        pub long_distance: bool,
        /// 低功耗需求
        pub low_power: bool,
        /// 实时性需求
        pub real_time: bool,
    }

    impl Default for TransferRequirements {
        fn default() -> Self {
            Self {
                high_speed: false,
                multi_device: false,
                long_distance: false,
                low_power: false,
                real_time: false,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_packet_creation() {
        let packet = ProtocolPacket::new(ProtocolType::I2C, 0x44, DataDirection::Read);
        assert_eq!(packet.protocol, ProtocolType::I2C);
        assert_eq!(packet.address, 0x44);
        assert_eq!(packet.direction, DataDirection::Read);
    }

    #[test]
    fn test_checksum_calculation() {
        let mut packet = ProtocolPacket::new(ProtocolType::I2C, 0x44, DataDirection::Write);
        packet.set_data(&[0x01, 0x02, 0x03]).unwrap();
        packet.set_register(0x10);
        
        let checksum = packet.calculate_checksum();
        packet.checksum = Some(checksum);
        
        assert!(packet.verify_checksum());
    }

    #[test]
    fn test_config_validation() {
        let config = ProtocolConfig::default();
        assert!(protocol_utils::validate_protocol_config(&config).is_ok());
        
        let mut invalid_config = config;
        invalid_config.timeout_ms = 0;
        assert!(protocol_utils::validate_protocol_config(&invalid_config).is_err());
    }

    #[test]
    fn test_transfer_efficiency_calculation() {
        let mut stats = ProtocolStats::default();
        stats.packets_sent = 100;
        stats.packets_received = 100;
        stats.send_failures = 5;
        stats.receive_failures = 3;
        
        let efficiency = protocol_utils::calculate_transfer_efficiency(&stats);
        assert!((efficiency - 0.96).abs() < 0.01); // 96% efficiency
    }

    #[test]
    fn test_protocol_compatibility() {
        assert!(protocol_utils::check_protocol_compatibility(ProtocolType::I2C, ProtocolType::I2C));
        assert!(protocol_utils::check_protocol_compatibility(ProtocolType::UART, ProtocolType::Modbus));
        assert!(!protocol_utils::check_protocol_compatibility(ProtocolType::I2C, ProtocolType::SPI));
    }
}