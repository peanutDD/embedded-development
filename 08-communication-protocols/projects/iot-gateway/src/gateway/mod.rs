//! Gateway Core Module
//! 
//! 网关核心模块，负责数据路由、连接管理和系统协调。

use heapless::{String, Vec, FnvIndexMap};
use serde::{Deserialize, Serialize};
use embassy_time::{Duration, Instant};

pub mod manager;
pub mod router;
pub mod config;

pub use manager::GatewayManager;
pub use router::{DataRouter, RoutingRule, RoutingTable};
pub use config::GatewayConfigManager;

/// 网关错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum GatewayError {
    /// 初始化错误
    Initialization(&'static str),
    /// 配置错误
    Configuration(&'static str),
    /// 连接错误
    Connection(&'static str),
    /// 路由错误
    Routing(&'static str),
    /// 缓冲区满
    BufferFull,
    /// 超时
    Timeout,
    /// 无效数据
    InvalidData,
    /// 资源不足
    ResourceExhausted,
    /// 未找到
    NotFound,
    /// 权限拒绝
    PermissionDenied,
}

/// 连接状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConnectionState {
    /// 断开连接
    Disconnected,
    /// 正在连接
    Connecting,
    /// 已连接
    Connected,
    /// 正在断开
    Disconnecting,
    /// 错误状态
    Error,
}

/// 连接类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConnectionType {
    /// 入站连接
    Inbound,
    /// 出站连接
    Outbound,
    /// 双向连接
    Bidirectional,
}

/// 数据优先级
#[derive(Debug, Clone, Copy, PartialEq, Ord, PartialOrd, Eq)]
pub enum DataPriority {
    /// 低优先级
    Low = 0,
    /// 普通优先级
    Normal = 1,
    /// 高优先级
    High = 2,
    /// 紧急优先级
    Critical = 3,
}

/// 连接信息
#[derive(Debug, Clone)]
pub struct ConnectionInfo {
    /// 连接ID
    pub id: String<32>,
    /// 连接类型
    pub connection_type: ConnectionType,
    /// 连接状态
    pub state: ConnectionState,
    /// 远程地址
    pub remote_address: String<64>,
    /// 本地地址
    pub local_address: String<64>,
    /// 协议类型
    pub protocol: String<16>,
    /// 创建时间
    pub created_at: Instant,
    /// 最后活动时间
    pub last_activity: Instant,
    /// 发送字节数
    pub bytes_sent: u64,
    /// 接收字节数
    pub bytes_received: u64,
    /// 错误计数
    pub error_count: u32,
}

impl Default for ConnectionInfo {
    fn default() -> Self {
        let now = Instant::now();
        Self {
            id: String::new(),
            connection_type: ConnectionType::Bidirectional,
            state: ConnectionState::Disconnected,
            remote_address: String::new(),
            local_address: String::new(),
            protocol: String::new(),
            created_at: now,
            last_activity: now,
            bytes_sent: 0,
            bytes_received: 0,
            error_count: 0,
        }
    }
}

/// 数据包
#[derive(Debug, Clone)]
pub struct DataPacket {
    /// 数据包ID
    pub id: String<32>,
    /// 源地址
    pub source: String<64>,
    /// 目标地址
    pub destination: String<64>,
    /// 协议类型
    pub protocol: String<16>,
    /// 数据优先级
    pub priority: DataPriority,
    /// 数据负载
    pub payload: Vec<u8, 1024>,
    /// 时间戳
    pub timestamp: Instant,
    /// 生存时间（毫秒）
    pub ttl_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 最大重试次数
    pub max_retries: u8,
}

impl Default for DataPacket {
    fn default() -> Self {
        Self {
            id: String::new(),
            source: String::new(),
            destination: String::new(),
            protocol: String::new(),
            priority: DataPriority::Normal,
            payload: Vec::new(),
            timestamp: Instant::now(),
            ttl_ms: 30000, // 30秒默认TTL
            retry_count: 0,
            max_retries: 3,
        }
    }
}

impl DataPacket {
    /// 创建新的数据包
    pub fn new(
        source: &str,
        destination: &str,
        protocol: &str,
        payload: &[u8],
    ) -> Result<Self, GatewayError> {
        let mut packet = Self::default();
        
        packet.source = String::from_str(source)
            .map_err(|_| GatewayError::InvalidData)?;
        packet.destination = String::from_str(destination)
            .map_err(|_| GatewayError::InvalidData)?;
        packet.protocol = String::from_str(protocol)
            .map_err(|_| GatewayError::InvalidData)?;
        
        packet.payload.extend_from_slice(payload)
            .map_err(|_| GatewayError::BufferFull)?;
        
        // 生成简单的ID
        let id = format!("pkt_{}", packet.timestamp.as_millis() % 100000);
        packet.id = String::from_str(&id)
            .map_err(|_| GatewayError::InvalidData)?;
        
        Ok(packet)
    }

    /// 检查数据包是否过期
    pub fn is_expired(&self) -> bool {
        let elapsed = self.timestamp.elapsed();
        elapsed.as_millis() > self.ttl_ms as u64
    }

    /// 检查是否可以重试
    pub fn can_retry(&self) -> bool {
        self.retry_count < self.max_retries
    }

    /// 增加重试次数
    pub fn increment_retry(&mut self) {
        self.retry_count += 1;
    }
}

/// 网关配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GatewayConfig {
    /// 设备ID
    pub device_id: String<32>,
    /// 最大连接数
    pub max_connections: usize,
    /// 缓冲区大小
    pub buffer_size: usize,
    /// 超时时间（毫秒）
    pub timeout_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 心跳间隔（毫秒）
    pub heartbeat_interval_ms: u32,
    /// 健康检查间隔（毫秒）
    pub health_check_interval_ms: u32,
    /// 统计更新间隔（毫秒）
    pub stats_update_interval_ms: u32,
    /// 启用调试
    pub debug_enabled: bool,
    /// 启用压缩
    pub compression_enabled: bool,
    /// 启用加密
    pub encryption_enabled: bool,
}

impl Default for GatewayConfig {
    fn default() -> Self {
        Self {
            device_id: String::from_str("gateway-001").unwrap_or_default(),
            max_connections: 16,
            buffer_size: 1024,
            timeout_ms: 5000,
            retry_count: 3,
            heartbeat_interval_ms: 30000,
            health_check_interval_ms: 60000,
            stats_update_interval_ms: 10000,
            debug_enabled: false,
            compression_enabled: false,
            encryption_enabled: false,
        }
    }
}

/// 网关统计信息
#[derive(Debug, Clone, Default)]
pub struct GatewayStats {
    /// 活跃连接数
    pub active_connections: usize,
    /// 总连接数
    pub total_connections: u64,
    /// 处理的数据包数
    pub packets_processed: u64,
    /// 路由的数据包数
    pub packets_routed: u64,
    /// 丢弃的数据包数
    pub packets_dropped: u64,
    /// 发送的字节数
    pub bytes_sent: u64,
    /// 接收的字节数
    pub bytes_received: u64,
    /// 错误计数
    pub error_count: u32,
    /// 平均延迟（微秒）
    pub avg_latency_us: u32,
    /// 吞吐量（字节/秒）
    pub throughput_bps: u32,
    /// CPU使用率（百分比）
    pub cpu_usage_percent: u8,
    /// 内存使用率（百分比）
    pub memory_usage_percent: u8,
}

/// 网关信息
#[derive(Debug, Clone)]
pub struct GatewayInfo {
    /// 设备ID
    pub device_id: String<32>,
    /// 网关版本
    pub version: String<16>,
    /// 运行时间（秒）
    pub uptime_seconds: u64,
    /// 状态
    pub status: String<16>,
    /// 支持的协议
    pub supported_protocols: Vec<String<16>, 8>,
    /// 配置信息
    pub config: GatewayConfig,
}

impl Default for GatewayInfo {
    fn default() -> Self {
        Self {
            device_id: String::from_str("gateway-001").unwrap_or_default(),
            version: String::from_str("1.0.0").unwrap_or_default(),
            uptime_seconds: 0,
            status: String::from_str("running").unwrap_or_default(),
            supported_protocols: Vec::new(),
            config: GatewayConfig::default(),
        }
    }
}

/// 网关主结构
pub struct Gateway {
    /// 配置
    config: GatewayConfig,
    /// 连接管理器
    manager: GatewayManager,
    /// 数据路由器
    router: DataRouter,
    /// 配置管理器
    config_manager: GatewayConfigManager,
    /// 统计信息
    stats: GatewayStats,
    /// 系统信息
    info: GatewayInfo,
    /// 启动时间
    start_time: Option<Instant>,
}

impl Gateway {
    /// 创建新的网关实例
    pub fn new(config: GatewayConfig) -> Result<Self, GatewayError> {
        let manager = GatewayManager::new(&config)?;
        let router = DataRouter::new(&config)?;
        let config_manager = GatewayConfigManager::new(&config)?;
        
        let mut info = GatewayInfo::default();
        info.device_id = config.device_id.clone();
        info.config = config.clone();

        Ok(Self {
            config,
            manager,
            router,
            config_manager,
            stats: GatewayStats::default(),
            info,
            start_time: None,
        })
    }

    /// 初始化网关
    pub async fn init(&mut self) -> Result<(), GatewayError> {
        self.manager.init().await?;
        self.router.init().await?;
        self.config_manager.init().await?;
        
        self.start_time = Some(Instant::now());
        Ok(())
    }

    /// 启动网关
    pub async fn start(&mut self) -> Result<(), GatewayError> {
        self.manager.start().await?;
        self.router.start().await?;
        self.config_manager.start().await?;
        
        self.info.status = String::from_str("running").unwrap_or_default();
        Ok(())
    }

    /// 停止网关
    pub async fn stop(&mut self) -> Result<(), GatewayError> {
        self.manager.stop().await?;
        self.router.stop().await?;
        self.config_manager.stop().await?;
        
        self.info.status = String::from_str("stopped").unwrap_or_default();
        Ok(())
    }

    /// 更新网关
    pub async fn update(&mut self) -> Result<(), GatewayError> {
        self.manager.update().await?;
        self.router.update().await?;
        self.config_manager.update().await?;
        
        self.update_stats().await;
        Ok(())
    }

    /// 处理数据
    pub async fn process_data(&mut self, data: &[u8]) -> Result<(), GatewayError> {
        self.manager.process_data(data).await?;
        self.stats.packets_processed += 1;
        Ok(())
    }

    /// 路由数据
    pub async fn route_data(&mut self, packet: &DataPacket) -> Result<(), GatewayError> {
        self.router.route_packet(packet).await?;
        self.stats.packets_routed += 1;
        Ok(())
    }

    /// 获取出站数据
    pub async fn get_outbound_data(&mut self) -> Result<DataPacket, GatewayError> {
        self.router.get_outbound_packet().await
    }

    /// 健康检查
    pub async fn health_check(&mut self) -> bool {
        let manager_ok = self.manager.health_check().await;
        let router_ok = self.router.health_check().await;
        let config_ok = self.config_manager.health_check().await;
        
        manager_ok && router_ok && config_ok
    }

    /// 更新统计信息
    async fn update_stats(&mut self) {
        self.stats.active_connections = self.manager.get_active_connections();
        self.stats.total_connections = self.manager.get_total_connections();
        
        if let Some(start_time) = self.start_time {
            self.info.uptime_seconds = start_time.elapsed().as_secs();
        }
    }

    /// 获取统计信息
    pub async fn get_stats(&self) -> GatewayStats {
        self.stats.clone()
    }

    /// 获取系统信息
    pub async fn get_info(&self) -> GatewayInfo {
        self.info.clone()
    }

    /// 更新配置
    pub async fn update_config(&mut self, config: GatewayConfig) -> Result<(), GatewayError> {
        self.config_manager.update_config(&config).await?;
        self.config = config;
        Ok(())
    }

    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats = GatewayStats::default();
        self.manager.reset_stats();
        self.router.reset_stats();
    }

    /// 系统维护
    pub async fn maintenance(&mut self) -> Result<(), GatewayError> {
        self.manager.maintenance().await?;
        self.router.maintenance().await?;
        self.config_manager.maintenance().await?;
        Ok(())
    }

    /// 进入睡眠模式
    pub async fn sleep(&mut self) -> Result<(), GatewayError> {
        self.manager.sleep().await?;
        self.router.sleep().await?;
        self.info.status = String::from_str("sleeping").unwrap_or_default();
        Ok(())
    }

    /// 从睡眠模式唤醒
    pub async fn wake_up(&mut self) -> Result<(), GatewayError> {
        self.manager.wake_up().await?;
        self.router.wake_up().await?;
        self.info.status = String::from_str("running").unwrap_or_default();
        Ok(())
    }

    /// 运行网关主循环
    pub async fn run(&mut self) -> ! {
        loop {
            // 更新网关
            let _ = self.update().await;

            // 处理连接管理
            let _ = self.manager.process_connections().await;

            // 处理数据路由
            let _ = self.router.process_routing().await;

            // 短暂延迟
            embassy_time::Timer::after(Duration::from_millis(10)).await;
        }
    }
}

/// 网关工具函数
pub mod gateway_utils {
    use super::*;

    /// 创建默认网关
    pub fn create_default_gateway() -> Result<Gateway, GatewayError> {
        Gateway::new(GatewayConfig::default())
    }

    /// 创建调试网关
    pub fn create_debug_gateway() -> Result<Gateway, GatewayError> {
        let mut config = GatewayConfig::default();
        config.debug_enabled = true;
        Gateway::new(config)
    }

    /// 验证网关配置
    pub fn validate_config(config: &GatewayConfig) -> Result<(), GatewayError> {
        if config.device_id.is_empty() {
            return Err(GatewayError::Configuration("Device ID cannot be empty"));
        }

        if config.max_connections == 0 {
            return Err(GatewayError::Configuration("Max connections must be greater than 0"));
        }

        if config.buffer_size == 0 {
            return Err(GatewayError::Configuration("Buffer size must be greater than 0"));
        }

        Ok(())
    }

    /// 计算网关内存使用
    pub fn calculate_memory_usage(config: &GatewayConfig) -> usize {
        // 连接管理器内存
        let connection_memory = config.max_connections * 256; // 每个连接约256字节
        
        // 路由器内存
        let router_memory = config.buffer_size * 2; // 输入输出缓冲区
        
        // 配置管理器内存
        let config_memory = 512; // 配置存储约512字节
        
        connection_memory + router_memory + config_memory
    }

    /// 获取推荐配置
    pub fn get_recommended_config(device_type: &str) -> GatewayConfig {
        match device_type {
            "low_power" => GatewayConfig {
                max_connections: 4,
                buffer_size: 256,
                timeout_ms: 10000,
                heartbeat_interval_ms: 60000,
                ..Default::default()
            },
            "high_performance" => GatewayConfig {
                max_connections: 64,
                buffer_size: 4096,
                timeout_ms: 1000,
                heartbeat_interval_ms: 10000,
                ..Default::default()
            },
            _ => GatewayConfig::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gateway_creation() {
        let config = GatewayConfig::default();
        let gateway = Gateway::new(config);
        assert!(gateway.is_ok());
    }

    #[test]
    fn test_data_packet_creation() {
        let packet = DataPacket::new("source", "dest", "mqtt", b"test data");
        assert!(packet.is_ok());
        
        let packet = packet.unwrap();
        assert_eq!(packet.source, "source");
        assert_eq!(packet.destination, "dest");
        assert_eq!(packet.protocol, "mqtt");
        assert_eq!(packet.payload.as_slice(), b"test data");
    }

    #[test]
    fn test_config_validation() {
        let config = GatewayConfig::default();
        assert!(gateway_utils::validate_config(&config).is_ok());
        
        let mut invalid_config = config;
        invalid_config.max_connections = 0;
        assert!(gateway_utils::validate_config(&invalid_config).is_err());
    }
}