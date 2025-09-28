//! IoT Gateway Library
//! 
//! 一个用于嵌入式系统的物联网网关库，提供多种通信协议支持、
//! 数据路由、安全传输和存储管理功能。
//! 
//! # 功能特性
//! 
//! - **多网络接口**: WiFi、以太网、蜂窝网络、LoRa、蓝牙
//! - **多协议支持**: MQTT、HTTP/HTTPS、CoAP、WebSocket
//! - **安全传输**: TLS/SSL、AES加密、HMAC认证
//! - **数据管理**: 缓存、存储、路由、转换
//! - **异步处理**: Embassy异步框架支持
//! - **低功耗**: 电源管理和睡眠模式
//! 
//! # 快速开始
//! 
//! ```rust,no_run
//! use iot_gateway::prelude::*;
//! 
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     // 创建网关配置
//!     let config = GatewayConfig::default();
//!     
//!     // 创建网关实例
//!     let mut gateway = Gateway::new(config).unwrap();
//!     
//!     // 初始化并启动
//!     gateway.init().await.unwrap();
//!     gateway.start().await.unwrap();
//!     
//!     // 运行网关
//!     gateway.run().await;
//! }
//! ```

#![no_std]
#![deny(unsafe_code)]
#![warn(missing_docs)]

// 重新导出核心依赖
pub use embedded_hal as hal;
pub use embedded_nal as nal;
pub use heapless;
pub use nb;
pub use serde;

// 重新导出异步相关
pub use embassy_executor;
pub use embassy_time;
pub use embassy_sync;
pub use embassy_futures;

// 重新导出网络协议
pub use smoltcp;
pub use minimq;

// 公开模块
pub mod gateway;
pub mod devices;
pub mod protocols;
pub mod network;
pub mod security;
pub mod storage;
pub mod utils;

// 重新导出常用类型
pub use gateway::{Gateway, GatewayConfig, GatewayManager, GatewayError, GatewayInfo, GatewayStats};
pub use network::{NetworkManager, NetworkConfig, NetworkInterface, NetworkError, NetworkInfo, NetworkStats};
pub use protocols::{ProtocolManager, ProtocolConfig, ProtocolError, ProtocolInfo, ProtocolStats};
pub use security::{SecurityManager, SecurityConfig, SecurityError, SecurityInfo, SecurityStats};
pub use storage::{StorageManager, StorageConfig, StorageError, StorageInfo, StorageStats};

/// 统一错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum IoTGatewayError {
    /// 网关错误
    Gateway(GatewayError),
    /// 网络错误
    Network(NetworkError),
    /// 协议错误
    Protocol(ProtocolError),
    /// 安全错误
    Security(SecurityError),
    /// 存储错误
    Storage(StorageError),
    /// 配置错误
    Config(&'static str),
    /// 初始化错误
    Initialization(&'static str),
    /// 运行时错误
    Runtime(&'static str),
}

impl From<GatewayError> for IoTGatewayError {
    fn from(err: GatewayError) -> Self {
        Self::Gateway(err)
    }
}

impl From<NetworkError> for IoTGatewayError {
    fn from(err: NetworkError) -> Self {
        Self::Network(err)
    }
}

impl From<ProtocolError> for IoTGatewayError {
    fn from(err: ProtocolError) -> Self {
        Self::Protocol(err)
    }
}

impl From<SecurityError> for IoTGatewayError {
    fn from(err: SecurityError) -> Self {
        Self::Security(err)
    }
}

impl From<StorageError> for IoTGatewayError {
    fn from(err: StorageError) -> Self {
        Self::Storage(err)
    }
}

/// IoT Gateway 配置
#[derive(Debug, Clone)]
pub struct IoTGatewayConfig {
    /// 网关配置
    pub gateway: GatewayConfig,
    /// 网络配置
    pub network: NetworkConfig,
    /// 协议配置
    pub protocols: ProtocolConfig,
    /// 安全配置
    pub security: SecurityConfig,
    /// 存储配置
    pub storage: StorageConfig,
}

impl Default for IoTGatewayConfig {
    fn default() -> Self {
        Self {
            gateway: GatewayConfig::default(),
            network: NetworkConfig::default(),
            protocols: ProtocolConfig::default(),
            security: SecurityConfig::default(),
            storage: StorageConfig::default(),
        }
    }
}

/// IoT Gateway 统计信息
#[derive(Debug, Clone)]
pub struct IoTGatewayStats {
    /// 网关统计
    pub gateway: GatewayStats,
    /// 网络统计
    pub network: NetworkStats,
    /// 协议统计
    pub protocols: ProtocolStats,
    /// 安全统计
    pub security: SecurityStats,
    /// 存储统计
    pub storage: StorageStats,
    /// 系统运行时间（秒）
    pub uptime_seconds: u64,
    /// 总处理的消息数
    pub total_messages: u64,
    /// 错误计数
    pub error_count: u32,
}

impl Default for IoTGatewayStats {
    fn default() -> Self {
        Self {
            gateway: GatewayStats::default(),
            network: NetworkStats::default(),
            protocols: ProtocolStats::default(),
            security: SecurityStats::default(),
            storage: StorageStats::default(),
            uptime_seconds: 0,
            total_messages: 0,
            error_count: 0,
        }
    }
}

/// IoT Gateway 信息
#[derive(Debug, Clone)]
pub struct IoTGatewayInfo {
    /// 设备ID
    pub device_id: heapless::String<32>,
    /// 固件版本
    pub firmware_version: heapless::String<16>,
    /// 硬件版本
    pub hardware_version: heapless::String<16>,
    /// 网关信息
    pub gateway: GatewayInfo,
    /// 网络信息
    pub network: NetworkInfo,
    /// 协议信息
    pub protocols: ProtocolInfo,
    /// 安全信息
    pub security: SecurityInfo,
    /// 存储信息
    pub storage: StorageInfo,
}

impl Default for IoTGatewayInfo {
    fn default() -> Self {
        Self {
            device_id: heapless::String::from_str("iot-gateway-001").unwrap_or_default(),
            firmware_version: heapless::String::from_str("1.0.0").unwrap_or_default(),
            hardware_version: heapless::String::from_str("1.0").unwrap_or_default(),
            gateway: GatewayInfo::default(),
            network: NetworkInfo::default(),
            protocols: ProtocolInfo::default(),
            security: SecurityInfo::default(),
            storage: StorageInfo::default(),
        }
    }
}

/// IoT Gateway 主结构
pub struct IoTGateway {
    /// 网关管理器
    gateway: Gateway,
    /// 网络管理器
    network: NetworkManager,
    /// 协议管理器
    protocols: ProtocolManager,
    /// 安全管理器
    security: SecurityManager,
    /// 存储管理器
    storage: StorageManager,
    /// 配置
    config: IoTGatewayConfig,
    /// 统计信息
    stats: IoTGatewayStats,
    /// 系统信息
    info: IoTGatewayInfo,
}

impl IoTGateway {
    /// 创建新的 IoT Gateway 实例
    pub fn new(config: IoTGatewayConfig) -> Result<Self, IoTGatewayError> {
        let gateway = Gateway::new(config.gateway.clone())?;
        let network = NetworkManager::new(config.network.clone())?;
        let protocols = ProtocolManager::new(config.protocols.clone())?;
        let security = SecurityManager::new(config.security.clone())?;
        let storage = StorageManager::new(config.storage.clone())?;

        Ok(Self {
            gateway,
            network,
            protocols,
            security,
            storage,
            config,
            stats: IoTGatewayStats::default(),
            info: IoTGatewayInfo::default(),
        })
    }

    /// 使用默认配置创建 IoT Gateway
    pub fn with_default_config() -> Result<Self, IoTGatewayError> {
        Self::new(IoTGatewayConfig::default())
    }

    /// 初始化 IoT Gateway
    pub async fn init(&mut self) -> Result<(), IoTGatewayError> {
        // 按依赖顺序初始化各个组件
        self.storage.init().await?;
        self.security.init().await?;
        self.network.init().await?;
        self.protocols.init().await?;
        self.gateway.init().await?;

        Ok(())
    }

    /// 启动 IoT Gateway
    pub async fn start(&mut self) -> Result<(), IoTGatewayError> {
        self.storage.start().await?;
        self.security.start().await?;
        self.network.start().await?;
        self.protocols.start().await?;
        self.gateway.start().await?;

        Ok(())
    }

    /// 停止 IoT Gateway
    pub async fn stop(&mut self) -> Result<(), IoTGatewayError> {
        self.gateway.stop().await?;
        self.protocols.stop().await?;
        self.network.stop().await?;
        self.security.stop().await?;
        self.storage.stop().await?;

        Ok(())
    }

    /// 更新 IoT Gateway
    pub async fn update(&mut self) -> Result<(), IoTGatewayError> {
        self.storage.update().await?;
        self.security.update().await?;
        self.network.update().await?;
        self.protocols.update().await?;
        self.gateway.update().await?;

        // 更新统计信息
        self.update_stats().await;

        Ok(())
    }

    /// 运行 IoT Gateway 主循环
    pub async fn run(&mut self) -> ! {
        loop {
            // 更新系统
            let _ = self.update().await;

            // 处理数据路由
            self.process_data_routing().await;

            // 健康检查
            self.health_check().await;

            // 短暂延迟
            embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
        }
    }

    /// 处理数据路由
    async fn process_data_routing(&mut self) {
        // 从网络接收数据并路由
        if let Ok(data) = self.network.receive_data().await {
            if let Ok(verified_data) = self.security.verify_data(&data).await {
                if let Ok(processed_data) = self.protocols.process_data(&verified_data).await {
                    let _ = self.gateway.route_data(&processed_data).await;
                    self.stats.total_messages += 1;
                }
            }
        }

        // 从网关获取出站数据并发送
        if let Ok(data) = self.gateway.get_outbound_data().await {
            if let Ok(packaged_data) = self.protocols.package_data(&data).await {
                if let Ok(encrypted_data) = self.security.encrypt_data(&packaged_data).await {
                    let _ = self.network.send_data(&encrypted_data).await;
                }
            }
        }
    }

    /// 健康检查
    pub async fn health_check(&mut self) -> bool {
        let gateway_ok = self.gateway.health_check().await;
        let network_ok = self.network.health_check().await;
        let protocols_ok = self.protocols.health_check().await;
        let security_ok = self.security.health_check().await;
        let storage_ok = self.storage.health_check().await;

        gateway_ok && network_ok && protocols_ok && security_ok && storage_ok
    }

    /// 更新统计信息
    async fn update_stats(&mut self) {
        self.stats.gateway = self.gateway.get_stats().await;
        self.stats.network = self.network.get_stats().await;
        self.stats.protocols = self.protocols.get_stats().await;
        self.stats.security = self.security.get_stats().await;
        self.stats.storage = self.storage.get_stats().await;
        self.stats.uptime_seconds += 1; // 假设每秒调用一次
    }

    /// 获取统计信息
    pub fn get_stats(&self) -> &IoTGatewayStats {
        &self.stats
    }

    /// 获取系统信息
    pub async fn get_info(&mut self) -> IoTGatewayInfo {
        self.info.gateway = self.gateway.get_info().await;
        self.info.network = self.network.get_info().await;
        self.info.protocols = self.protocols.get_info().await;
        self.info.security = self.security.get_info().await;
        self.info.storage = self.storage.get_info().await;
        self.info.clone()
    }

    /// 更新配置
    pub async fn update_config(&mut self, config: IoTGatewayConfig) -> Result<(), IoTGatewayError> {
        self.gateway.update_config(config.gateway).await?;
        self.network.update_config(config.network).await?;
        self.protocols.update_config(config.protocols).await?;
        self.security.update_config(config.security).await?;
        self.storage.update_config(config.storage).await?;
        self.config = config;
        Ok(())
    }

    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats = IoTGatewayStats::default();
        self.gateway.reset_stats();
        self.network.reset_stats();
        self.protocols.reset_stats();
        self.security.reset_stats();
        self.storage.reset_stats();
    }

    /// 系统维护
    pub async fn maintenance(&mut self) -> Result<(), IoTGatewayError> {
        self.storage.maintenance().await?;
        self.security.maintenance().await?;
        self.network.maintenance().await?;
        self.protocols.maintenance().await?;
        self.gateway.maintenance().await?;
        Ok(())
    }

    /// 进入睡眠模式
    pub async fn sleep(&mut self) -> Result<(), IoTGatewayError> {
        self.gateway.sleep().await?;
        self.protocols.sleep().await?;
        self.network.sleep().await?;
        self.security.sleep().await?;
        self.storage.sleep().await?;
        Ok(())
    }

    /// 从睡眠模式唤醒
    pub async fn wake_up(&mut self) -> Result<(), IoTGatewayError> {
        self.storage.wake_up().await?;
        self.security.wake_up().await?;
        self.network.wake_up().await?;
        self.protocols.wake_up().await?;
        self.gateway.wake_up().await?;
        Ok(())
    }
}

/// 预导入模块，包含常用类型和 trait
pub mod prelude {
    pub use crate::{
        IoTGateway, IoTGatewayConfig, IoTGatewayError, IoTGatewayInfo, IoTGatewayStats,
        Gateway, GatewayConfig, GatewayError, GatewayManager,
        NetworkManager, NetworkConfig, NetworkInterface, NetworkError,
        ProtocolManager, ProtocolConfig, ProtocolError,
        SecurityManager, SecurityConfig, SecurityError,
        StorageManager, StorageConfig, StorageError,
    };
    
    pub use embassy_executor::Spawner;
    pub use embassy_time::{Duration, Timer};
    pub use heapless::{String, Vec};
    pub use serde::{Deserialize, Serialize};
}

/// 工具函数模块
pub mod iot_utils {
    use super::*;

    /// 创建默认的 IoT Gateway
    pub fn create_default_gateway() -> Result<IoTGateway, IoTGatewayError> {
        IoTGateway::with_default_config()
    }

    /// 创建调试用的 IoT Gateway
    pub fn create_debug_gateway() -> Result<IoTGateway, IoTGatewayError> {
        let mut config = IoTGatewayConfig::default();
        
        // 启用调试功能
        config.gateway.debug_enabled = true;
        config.network.debug_enabled = true;
        config.protocols.debug_enabled = true;
        config.security.debug_enabled = true;
        config.storage.debug_enabled = true;

        IoTGateway::new(config)
    }

    /// 验证配置
    pub fn validate_config(config: &IoTGatewayConfig) -> Result<(), IoTGatewayError> {
        // 验证网关配置
        if config.gateway.device_id.is_empty() {
            return Err(IoTGatewayError::Config("Device ID cannot be empty"));
        }

        // 验证网络配置
        if config.network.max_connections == 0 {
            return Err(IoTGatewayError::Config("Max connections must be greater than 0"));
        }

        // 验证协议配置
        if !config.protocols.mqtt_enabled && !config.protocols.http_enabled {
            return Err(IoTGatewayError::Config("At least one protocol must be enabled"));
        }

        Ok(())
    }

    /// 计算内存使用情况
    pub fn calculate_memory_usage(config: &IoTGatewayConfig) -> usize {
        let mut total = 0;
        
        // 网关内存
        total += config.gateway.buffer_size * config.gateway.max_connections;
        
        // 网络内存
        total += config.network.buffer_size * config.network.max_connections;
        
        // 协议内存
        total += config.protocols.buffer_size;
        
        // 安全内存
        total += config.security.key_cache_size;
        
        // 存储内存
        total += config.storage.cache_size;
        
        total
    }

    /// 获取推荐配置
    pub fn get_recommended_config(device_type: &str) -> IoTGatewayConfig {
        match device_type {
            "low_power" => {
                let mut config = IoTGatewayConfig::default();
                config.gateway.max_connections = 4;
                config.gateway.buffer_size = 256;
                config.network.max_connections = 2;
                config.storage.cache_size = 512;
                config
            },
            "high_performance" => {
                let mut config = IoTGatewayConfig::default();
                config.gateway.max_connections = 32;
                config.gateway.buffer_size = 2048;
                config.network.max_connections = 16;
                config.storage.cache_size = 4096;
                config
            },
            _ => IoTGatewayConfig::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_validation() {
        let config = IoTGatewayConfig::default();
        assert!(iot_utils::validate_config(&config).is_ok());
    }

    #[test]
    fn test_memory_calculation() {
        let config = IoTGatewayConfig::default();
        let memory = iot_utils::calculate_memory_usage(&config);
        assert!(memory > 0);
    }

    #[test]
    fn test_recommended_configs() {
        let low_power = iot_utils::get_recommended_config("low_power");
        let high_perf = iot_utils::get_recommended_config("high_performance");
        
        assert!(low_power.gateway.max_connections < high_perf.gateway.max_connections);
        assert!(low_power.storage.cache_size < high_perf.storage.cache_size);
    }
}