//! 网络通信模块
//! 
//! 负责IoT网关的网络连接和通信协议处理，包括WiFi、MQTT、HTTP等。

use heapless::{String, Vec, FnvIndexMap};
use serde::{Deserialize, Serialize};
use embassy_time::{Duration, Timer, Instant};

pub mod wifi;
pub mod mqtt;

pub use wifi::{WiFiManager, WiFiConfig, WiFiError, WiFiStatus, ConnectionState};
pub use mqtt::{MqttManager, MqttConfig, MqttError, MqttMessage, QoSLevel};

/// 网络错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum NetworkError {
    /// WiFi错误
    WiFi(WiFiError),
    /// MQTT错误
    Mqtt(MqttError),
    /// 连接错误
    ConnectionError,
    /// 超时
    Timeout,
    /// 配置错误
    ConfigError(&'static str),
    /// 认证失败
    AuthenticationFailed,
    /// 网络不可达
    NetworkUnreachable,
    /// DNS解析失败
    DnsResolutionFailed,
    /// 协议错误
    ProtocolError,
    /// 缓冲区错误
    BufferError,
}

/// 网络接口类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NetworkInterface {
    /// WiFi接口
    WiFi,
    /// 以太网接口
    Ethernet,
    /// 蜂窝网络接口
    Cellular,
    /// LoRa接口
    LoRa,
    /// 蓝牙接口
    Bluetooth,
}

/// 网络状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkStatus {
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
    /// 未知状态
    Unknown,
}

/// 网络配置
#[derive(Debug, Clone)]
pub struct NetworkConfig {
    /// 网络接口
    pub interface: NetworkInterface,
    /// SSID（WiFi）
    pub ssid: Option<String<32>>,
    /// 密码
    pub password: Option<String<64>>,
    /// 启用DHCP
    pub dhcp: bool,
    /// 静态IP地址
    pub static_ip: Option<String<16>>,
    /// 子网掩码
    pub subnet_mask: Option<String<16>>,
    /// 网关地址
    pub gateway: Option<String<16>>,
    /// DNS服务器
    pub dns_servers: &'static [&'static str],
    /// 连接超时（毫秒）
    pub connection_timeout_ms: u32,
    /// 重连尝试次数
    pub reconnect_attempts: u8,
    /// 重连延迟（毫秒）
    pub reconnect_delay_ms: u32,
    /// 启用调试
    pub debug_enabled: bool,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            interface: NetworkInterface::WiFi,
            ssid: None,
            password: None,
            dhcp: true,
            static_ip: None,
            subnet_mask: None,
            gateway: None,
            dns_servers: &["8.8.8.8", "8.8.4.4"],
            connection_timeout_ms: 30000,
            reconnect_attempts: 5,
            reconnect_delay_ms: 10000,
            debug_enabled: false,
        }
    }
}

/// 网络统计信息
#[derive(Debug, Clone, Default)]
pub struct NetworkStats {
    /// 连接次数
    pub connection_attempts: u32,
    /// 成功连接次数
    pub successful_connections: u32,
    /// 连接失败次数
    pub failed_connections: u32,
    /// 断开连接次数
    pub disconnections: u32,
    /// 发送的数据包数
    pub packets_sent: u64,
    /// 接收的数据包数
    pub packets_received: u64,
    /// 发送的字节数
    pub bytes_sent: u64,
    /// 接收的字节数
    pub bytes_received: u64,
    /// 网络错误次数
    pub network_errors: u32,
    /// 平均延迟（毫秒）
    pub avg_latency_ms: u32,
    /// 信号强度（dBm）
    pub signal_strength_dbm: i8,
    /// 连接质量（百分比）
    pub connection_quality: u8,
}

/// 网络信息
#[derive(Debug, Clone)]
pub struct NetworkInfo {
    /// 接口类型
    pub interface: NetworkInterface,
    /// MAC地址
    pub mac_address: String<18>,
    /// IP地址
    pub ip_address: String<16>,
    /// 子网掩码
    pub subnet_mask: String<16>,
    /// 网关地址
    pub gateway: String<16>,
    /// DNS服务器
    pub dns_servers: Vec<String<16>, 4>,
    /// 网络状态
    pub status: NetworkStatus,
    /// SSID（WiFi）
    pub ssid: Option<String<32>>,
    /// 信号强度
    pub signal_strength: i8,
    /// 连接时间（秒）
    pub connection_time_seconds: u64,
}

impl Default for NetworkInfo {
    fn default() -> Self {
        Self {
            interface: NetworkInterface::WiFi,
            mac_address: String::new(),
            ip_address: String::new(),
            subnet_mask: String::new(),
            gateway: String::new(),
            dns_servers: Vec::new(),
            status: NetworkStatus::Disconnected,
            ssid: None,
            signal_strength: -100,
            connection_time_seconds: 0,
        }
    }
}

/// 网络管理器
pub struct NetworkManager {
    /// 配置
    config: NetworkConfig,
    /// WiFi管理器
    wifi_manager: Option<WiFiManager>,
    /// MQTT管理器
    mqtt_manager: Option<MqttManager>,
    /// 网络统计
    stats: NetworkStats,
    /// 网络信息
    info: NetworkInfo,
    /// 连接开始时间
    connection_start_time: Option<Instant>,
    /// 最后心跳时间
    last_heartbeat_time: Instant,
    /// 重连尝试次数
    reconnect_attempts: u8,
}

impl NetworkManager {
    /// 创建新的网络管理器
    pub fn new(config: NetworkConfig) -> Result<Self, NetworkError> {
        let mut info = NetworkInfo::default();
        info.interface = config.interface;

        Ok(Self {
            config,
            wifi_manager: None,
            mqtt_manager: None,
            stats: NetworkStats::default(),
            info,
            connection_start_time: None,
            last_heartbeat_time: Instant::now(),
            reconnect_attempts: 0,
        })
    }

    /// 初始化网络管理器
    pub async fn init(&mut self) -> Result<(), NetworkError> {
        match self.config.interface {
            NetworkInterface::WiFi => {
                if let Some(ssid) = &self.config.ssid {
                    let wifi_config = WiFiConfig {
                        ssid: ssid.clone(),
                        password: self.config.password.clone().unwrap_or_default(),
                        timeout_ms: self.config.connection_timeout_ms,
                        retry_count: self.config.reconnect_attempts,
                        ..Default::default()
                    };
                    
                    self.wifi_manager = Some(WiFiManager::new(wifi_config)?);
                }
            },
            _ => {
                return Err(NetworkError::ConfigError("Unsupported network interface"));
            }
        }

        Ok(())
    }

    /// 启动网络连接
    pub async fn start(&mut self) -> Result<(), NetworkError> {
        match self.config.interface {
            NetworkInterface::WiFi => {
                if let Some(wifi_manager) = &mut self.wifi_manager {
                    self.info.status = NetworkStatus::Connecting;
                    self.connection_start_time = Some(Instant::now());
                    self.stats.connection_attempts += 1;

                    match wifi_manager.connect().await {
                        Ok(()) => {
                            self.info.status = NetworkStatus::Connected;
                            self.stats.successful_connections += 1;
                            self.reconnect_attempts = 0;
                            
                            // 更新网络信息
                            self.update_network_info().await?;
                            
                            Ok(())
                        },
                        Err(e) => {
                            self.info.status = NetworkStatus::Error;
                            self.stats.failed_connections += 1;
                            Err(NetworkError::WiFi(e))
                        }
                    }
                } else {
                    Err(NetworkError::ConfigError("WiFi manager not initialized"))
                }
            },
            _ => Err(NetworkError::ConfigError("Unsupported network interface")),
        }
    }

    /// 停止网络连接
    pub async fn stop(&mut self) -> Result<(), NetworkError> {
        match self.config.interface {
            NetworkInterface::WiFi => {
                if let Some(wifi_manager) = &mut self.wifi_manager {
                    self.info.status = NetworkStatus::Disconnecting;
                    
                    match wifi_manager.disconnect().await {
                        Ok(()) => {
                            self.info.status = NetworkStatus::Disconnected;
                            self.stats.disconnections += 1;
                            self.connection_start_time = None;
                            Ok(())
                        },
                        Err(e) => {
                            self.info.status = NetworkStatus::Error;
                            Err(NetworkError::WiFi(e))
                        }
                    }
                } else {
                    Ok(()) // 没有初始化，认为已经断开
                }
            },
            _ => Ok(()),
        }
    }

    /// 更新网络管理器
    pub async fn update(&mut self) -> Result<(), NetworkError> {
        // 检查连接状态
        self.check_connection_status().await?;

        // 更新统计信息
        self.update_stats().await;

        // 处理重连逻辑
        if self.info.status == NetworkStatus::Error || self.info.status == NetworkStatus::Disconnected {
            if self.reconnect_attempts < self.config.reconnect_attempts {
                self.reconnect_attempts += 1;
                
                // 等待重连延迟
                Timer::after(Duration::from_millis(self.config.reconnect_delay_ms as u64)).await;
                
                // 尝试重连
                let _ = self.start().await;
            }
        }

        Ok(())
    }

    /// 发送数据
    pub async fn send_data(&mut self, data: &[u8]) -> Result<(), NetworkError> {
        if self.info.status != NetworkStatus::Connected {
            return Err(NetworkError::ConnectionError);
        }

        // 这里应该根据具体的网络接口发送数据
        // 目前只是更新统计信息
        self.stats.packets_sent += 1;
        self.stats.bytes_sent += data.len() as u64;

        Ok(())
    }

    /// 接收数据
    pub async fn receive_data(&mut self) -> Result<Vec<u8, 1024>, NetworkError> {
        if self.info.status != NetworkStatus::Connected {
            return Err(NetworkError::ConnectionError);
        }

        // 这里应该根据具体的网络接口接收数据
        // 目前返回空数据
        let data = Vec::new();
        
        if !data.is_empty() {
            self.stats.packets_received += 1;
            self.stats.bytes_received += data.len() as u64;
        }

        Ok(data)
    }

    /// 检查连接状态
    async fn check_connection_status(&mut self) -> Result<(), NetworkError> {
        match self.config.interface {
            NetworkInterface::WiFi => {
                if let Some(wifi_manager) = &mut self.wifi_manager {
                    let wifi_status = wifi_manager.get_status().await?;
                    
                    self.info.status = match wifi_status.state {
                        ConnectionState::Connected => NetworkStatus::Connected,
                        ConnectionState::Connecting => NetworkStatus::Connecting,
                        ConnectionState::Disconnected => NetworkStatus::Disconnected,
                        ConnectionState::Error => NetworkStatus::Error,
                    };

                    self.info.signal_strength = wifi_status.signal_strength;
                    self.stats.signal_strength_dbm = wifi_status.signal_strength;
                }
            },
            _ => {}
        }

        Ok(())
    }

    /// 更新网络信息
    async fn update_network_info(&mut self) -> Result<(), NetworkError> {
        match self.config.interface {
            NetworkInterface::WiFi => {
                if let Some(wifi_manager) = &mut self.wifi_manager {
                    let wifi_status = wifi_manager.get_status().await?;
                    
                    self.info.ssid = Some(wifi_status.ssid);
                    self.info.ip_address = wifi_status.ip_address;
                    self.info.mac_address = wifi_status.mac_address;
                    self.info.signal_strength = wifi_status.signal_strength;
                }
            },
            _ => {}
        }

        Ok(())
    }

    /// 更新统计信息
    async fn update_stats(&mut self) {
        // 更新连接时间
        if let Some(start_time) = self.connection_start_time {
            self.info.connection_time_seconds = start_time.elapsed().as_secs();
        }

        // 计算连接质量
        if self.info.signal_strength > -50 {
            self.stats.connection_quality = 100;
        } else if self.info.signal_strength > -70 {
            self.stats.connection_quality = 75;
        } else if self.info.signal_strength > -85 {
            self.stats.connection_quality = 50;
        } else {
            self.stats.connection_quality = 25;
        }

        // 更新心跳
        self.last_heartbeat_time = Instant::now();
    }

    /// 健康检查
    pub async fn health_check(&mut self) -> bool {
        match self.check_connection_status().await {
            Ok(()) => self.info.status == NetworkStatus::Connected,
            Err(_) => {
                self.stats.network_errors += 1;
                false
            }
        }
    }

    /// 获取网络统计信息
    pub fn get_stats(&self) -> NetworkStats {
        self.stats.clone()
    }

    /// 获取网络信息
    pub async fn get_info(&mut self) -> NetworkInfo {
        let _ = self.update_network_info().await;
        self.info.clone()
    }

    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats = NetworkStats::default();
    }

    /// 更新配置
    pub async fn update_config(&mut self, config: NetworkConfig) -> Result<(), NetworkError> {
        // 如果接口类型改变，需要重新初始化
        if config.interface != self.config.interface {
            self.stop().await?;
            self.config = config;
            self.init().await?;
        } else {
            self.config = config;
        }

        Ok(())
    }

    /// 系统维护
    pub async fn maintenance(&mut self) -> Result<(), NetworkError> {
        // 清理过期的统计信息
        // 检查网络连接质量
        // 执行网络诊断
        
        if self.stats.connection_quality < 25 {
            // 连接质量差，尝试重连
            let _ = self.stop().await;
            let _ = self.start().await;
        }

        Ok(())
    }

    /// 进入睡眠模式
    pub async fn sleep(&mut self) -> Result<(), NetworkError> {
        match self.config.interface {
            NetworkInterface::WiFi => {
                if let Some(wifi_manager) = &mut self.wifi_manager {
                    wifi_manager.sleep().await?;
                }
            },
            _ => {}
        }

        Ok(())
    }

    /// 从睡眠模式唤醒
    pub async fn wake_up(&mut self) -> Result<(), NetworkError> {
        match self.config.interface {
            NetworkInterface::WiFi => {
                if let Some(wifi_manager) = &mut self.wifi_manager {
                    wifi_manager.wake_up().await?;
                }
            },
            _ => {}
        }

        Ok(())
    }

    /// 获取MQTT管理器
    pub fn get_mqtt_manager(&mut self) -> Option<&mut MqttManager> {
        self.mqtt_manager.as_mut()
    }

    /// 初始化MQTT
    pub async fn init_mqtt(&mut self, mqtt_config: MqttConfig) -> Result<(), NetworkError> {
        if self.info.status != NetworkStatus::Connected {
            return Err(NetworkError::ConnectionError);
        }

        self.mqtt_manager = Some(MqttManager::new(mqtt_config)?);
        
        if let Some(mqtt_manager) = &mut self.mqtt_manager {
            mqtt_manager.connect().await?;
        }

        Ok(())
    }

    /// 发送MQTT消息
    pub async fn publish_mqtt(&mut self, topic: &str, payload: &[u8]) -> Result<(), NetworkError> {
        if let Some(mqtt_manager) = &mut self.mqtt_manager {
            mqtt_manager.publish(topic, payload, QoSLevel::AtMostOnce).await?;
            Ok(())
        } else {
            Err(NetworkError::ConfigError("MQTT not initialized"))
        }
    }

    /// 订阅MQTT主题
    pub async fn subscribe_mqtt(&mut self, topic: &str) -> Result<(), NetworkError> {
        if let Some(mqtt_manager) = &mut self.mqtt_manager {
            mqtt_manager.subscribe(topic, QoSLevel::AtMostOnce).await?;
            Ok(())
        } else {
            Err(NetworkError::ConfigError("MQTT not initialized"))
        }
    }
}

/// 网络工具函数
pub mod network_utils {
    use super::*;

    /// 创建WiFi配置
    pub fn create_wifi_config(ssid: &str, password: &str) -> NetworkConfig {
        NetworkConfig {
            interface: NetworkInterface::WiFi,
            ssid: Some(String::from_str(ssid).unwrap_or_default()),
            password: Some(String::from_str(password).unwrap_or_default()),
            ..Default::default()
        }
    }

    /// 验证IP地址格式
    pub fn validate_ip_address(ip: &str) -> bool {
        let parts: Vec<&str> = ip.split('.').collect();
        if parts.len() != 4 {
            return false;
        }

        for part in parts {
            if let Ok(num) = part.parse::<u8>() {
                // 有效的IP地址段
            } else {
                return false;
            }
        }

        true
    }

    /// 计算网络延迟
    pub fn calculate_latency(start_time: Instant, end_time: Instant) -> u32 {
        end_time.duration_since(start_time).as_millis() as u32
    }

    /// 格式化MAC地址
    pub fn format_mac_address(mac: &[u8; 6]) -> String<18> {
        let mac_str = format!(
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        );
        String::from_str(&mac_str).unwrap_or_default()
    }

    /// 信号强度转换为质量百分比
    pub fn signal_strength_to_quality(rssi: i8) -> u8 {
        if rssi > -50 {
            100
        } else if rssi > -60 {
            80
        } else if rssi > -70 {
            60
        } else if rssi > -80 {
            40
        } else if rssi > -90 {
            20
        } else {
            0
        }
    }

    /// 检查网络接口兼容性
    pub fn check_interface_compatibility(interface1: NetworkInterface, interface2: NetworkInterface) -> bool {
        match (interface1, interface2) {
            (NetworkInterface::WiFi, NetworkInterface::Ethernet) => false,
            (NetworkInterface::Cellular, NetworkInterface::LoRa) => true,
            (NetworkInterface::Bluetooth, NetworkInterface::WiFi) => true,
            _ => interface1 == interface2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_network_config_creation() {
        let config = NetworkConfig::default();
        assert_eq!(config.interface, NetworkInterface::WiFi);
        assert!(config.dhcp);
    }

    #[test]
    fn test_ip_address_validation() {
        assert!(network_utils::validate_ip_address("192.168.1.1"));
        assert!(network_utils::validate_ip_address("10.0.0.1"));
        assert!(!network_utils::validate_ip_address("256.1.1.1"));
        assert!(!network_utils::validate_ip_address("192.168.1"));
    }

    #[test]
    fn test_signal_strength_conversion() {
        assert_eq!(network_utils::signal_strength_to_quality(-40), 100);
        assert_eq!(network_utils::signal_strength_to_quality(-65), 60);
        assert_eq!(network_utils::signal_strength_to_quality(-95), 0);
    }

    #[test]
    fn test_mac_address_formatting() {
        let mac = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let formatted = network_utils::format_mac_address(&mac);
        assert_eq!(formatted, "00:11:22:33:44:55");
    }
}