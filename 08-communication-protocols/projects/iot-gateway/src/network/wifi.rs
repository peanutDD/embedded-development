//! WiFi网络管理模块
//! 
//! 负责WiFi连接的建立、维护和管理。

use heapless::{String, Vec, FnvIndexMap};
use embassy_time::{Duration, Timer, Instant};
use serde::{Deserialize, Serialize};

use super::NetworkError;

/// WiFi错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum WiFiError {
    /// 连接失败
    ConnectionFailed,
    /// 认证失败
    AuthenticationFailed,
    /// 网络未找到
    NetworkNotFound,
    /// 信号太弱
    WeakSignal,
    /// 超时
    Timeout,
    /// 配置错误
    ConfigError(&'static str),
    /// 硬件错误
    HardwareError,
    /// 驱动错误
    DriverError,
    /// DHCP失败
    DhcpFailed,
    /// DNS解析失败
    DnsResolutionFailed,
}

/// 连接状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    /// 断开连接
    Disconnected,
    /// 正在连接
    Connecting,
    /// 已连接
    Connected,
    /// 错误状态
    Error,
}

/// WiFi安全类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SecurityType {
    /// 开放网络
    Open,
    /// WEP加密
    WEP,
    /// WPA加密
    WPA,
    /// WPA2加密
    WPA2,
    /// WPA3加密
    WPA3,
    /// WPS
    WPS,
}

/// WiFi配置
#[derive(Debug, Clone)]
pub struct WiFiConfig {
    /// 网络SSID
    pub ssid: String<32>,
    /// 网络密码
    pub password: String<64>,
    /// 安全类型
    pub security: SecurityType,
    /// 是否隐藏网络
    pub hidden: bool,
    /// 连接超时（毫秒）
    pub timeout_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 自动重连
    pub auto_reconnect: bool,
    /// 省电模式
    pub power_save: bool,
    /// 信号强度阈值
    pub min_signal_strength: i8,
    /// DHCP启用
    pub dhcp_enabled: bool,
    /// 静态IP配置
    pub static_ip: Option<StaticIpConfig>,
}

/// 静态IP配置
#[derive(Debug, Clone)]
pub struct StaticIpConfig {
    /// IP地址
    pub ip: String<16>,
    /// 子网掩码
    pub netmask: String<16>,
    /// 网关
    pub gateway: String<16>,
    /// DNS服务器
    pub dns: Vec<String<16>, 4>,
}

impl Default for WiFiConfig {
    fn default() -> Self {
        Self {
            ssid: String::new(),
            password: String::new(),
            security: SecurityType::WPA2,
            hidden: false,
            timeout_ms: 30000,
            retry_count: 3,
            auto_reconnect: true,
            power_save: false,
            min_signal_strength: -85,
            dhcp_enabled: true,
            static_ip: None,
        }
    }
}

/// WiFi网络信息
#[derive(Debug, Clone)]
pub struct WiFiNetworkInfo {
    /// SSID
    pub ssid: String<32>,
    /// BSSID (MAC地址)
    pub bssid: String<18>,
    /// 信号强度 (dBm)
    pub signal_strength: i8,
    /// 频道
    pub channel: u8,
    /// 安全类型
    pub security: SecurityType,
    /// 是否隐藏
    pub hidden: bool,
}

/// WiFi状态
#[derive(Debug, Clone)]
pub struct WiFiStatus {
    /// 连接状态
    pub state: ConnectionState,
    /// 当前SSID
    pub ssid: String<32>,
    /// IP地址
    pub ip_address: String<16>,
    /// MAC地址
    pub mac_address: String<18>,
    /// 信号强度
    pub signal_strength: i8,
    /// 频道
    pub channel: u8,
    /// 连接时间（秒）
    pub connection_time: u64,
    /// 数据传输统计
    pub tx_bytes: u64,
    pub rx_bytes: u64,
}

impl Default for WiFiStatus {
    fn default() -> Self {
        Self {
            state: ConnectionState::Disconnected,
            ssid: String::new(),
            ip_address: String::new(),
            mac_address: String::new(),
            signal_strength: -100,
            channel: 0,
            connection_time: 0,
            tx_bytes: 0,
            rx_bytes: 0,
        }
    }
}

/// WiFi统计信息
#[derive(Debug, Clone, Default)]
pub struct WiFiStats {
    /// 连接尝试次数
    pub connection_attempts: u32,
    /// 成功连接次数
    pub successful_connections: u32,
    /// 连接失败次数
    pub failed_connections: u32,
    /// 断开连接次数
    pub disconnections: u32,
    /// 重连次数
    pub reconnections: u32,
    /// 扫描次数
    pub scan_count: u32,
    /// 平均连接时间（毫秒）
    pub avg_connection_time_ms: u32,
    /// 总在线时间（秒）
    pub total_online_time_seconds: u64,
    /// 数据传输错误次数
    pub transmission_errors: u32,
}

/// WiFi管理器
pub struct WiFiManager {
    /// 配置
    config: WiFiConfig,
    /// 当前状态
    status: WiFiStatus,
    /// 统计信息
    stats: WiFiStats,
    /// 扫描到的网络列表
    scanned_networks: Vec<WiFiNetworkInfo, 32>,
    /// 连接开始时间
    connection_start_time: Option<Instant>,
    /// 最后扫描时间
    last_scan_time: Option<Instant>,
    /// 重连尝试次数
    reconnect_attempts: u8,
}

impl WiFiManager {
    /// 创建新的WiFi管理器
    pub fn new(config: WiFiConfig) -> Result<Self, WiFiError> {
        // 验证配置
        if config.ssid.is_empty() {
            return Err(WiFiError::ConfigError("SSID cannot be empty"));
        }

        Ok(Self {
            config,
            status: WiFiStatus::default(),
            stats: WiFiStats::default(),
            scanned_networks: Vec::new(),
            connection_start_time: None,
            last_scan_time: None,
            reconnect_attempts: 0,
        })
    }

    /// 扫描WiFi网络
    pub async fn scan_networks(&mut self) -> Result<Vec<WiFiNetworkInfo, 32>, WiFiError> {
        self.stats.scan_count += 1;
        self.last_scan_time = Some(Instant::now());

        // 模拟网络扫描
        self.scanned_networks.clear();
        
        // 添加一些模拟的网络
        let networks = [
            ("WiFi-Network-1", -45, 6, SecurityType::WPA2),
            ("WiFi-Network-2", -67, 11, SecurityType::WPA3),
            ("Open-Network", -72, 1, SecurityType::Open),
            ("Hidden-Network", -55, 9, SecurityType::WPA2),
        ];

        for (ssid, rssi, channel, security) in &networks {
            let network = WiFiNetworkInfo {
                ssid: String::from_str(ssid).unwrap_or_default(),
                bssid: String::from_str("00:11:22:33:44:55").unwrap_or_default(),
                signal_strength: *rssi,
                channel: *channel,
                security: *security,
                hidden: ssid.contains("Hidden"),
            };
            
            if self.scanned_networks.push(network).is_err() {
                break;
            }
        }

        Ok(self.scanned_networks.clone())
    }

    /// 连接到WiFi网络
    pub async fn connect(&mut self) -> Result<(), WiFiError> {
        self.status.state = ConnectionState::Connecting;
        self.connection_start_time = Some(Instant::now());
        self.stats.connection_attempts += 1;

        // 检查网络是否在扫描列表中
        let network_found = self.scanned_networks.iter()
            .any(|network| network.ssid == self.config.ssid);

        if !network_found {
            // 尝试扫描网络
            self.scan_networks().await?;
            
            let network_found = self.scanned_networks.iter()
                .any(|network| network.ssid == self.config.ssid);
            
            if !network_found {
                self.status.state = ConnectionState::Error;
                self.stats.failed_connections += 1;
                return Err(WiFiError::NetworkNotFound);
            }
        }

        // 获取目标网络信息
        let target_network = self.scanned_networks.iter()
            .find(|network| network.ssid == self.config.ssid)
            .unwrap();

        // 检查信号强度
        if target_network.signal_strength < self.config.min_signal_strength {
            self.status.state = ConnectionState::Error;
            self.stats.failed_connections += 1;
            return Err(WiFiError::WeakSignal);
        }

        // 模拟连接过程
        for attempt in 0..=self.config.retry_count {
            // 模拟连接延迟
            Timer::after(Duration::from_millis(2000)).await;

            // 模拟连接成功（90%成功率）
            if attempt < self.config.retry_count || rand_success(90) {
                // 连接成功
                self.status.state = ConnectionState::Connected;
                self.status.ssid = self.config.ssid.clone();
                self.status.signal_strength = target_network.signal_strength;
                self.status.channel = target_network.channel;
                
                // 模拟获取IP地址
                if self.config.dhcp_enabled {
                    self.status.ip_address = String::from_str("192.168.1.100").unwrap_or_default();
                } else if let Some(static_config) = &self.config.static_ip {
                    self.status.ip_address = static_config.ip.clone();
                }

                self.status.mac_address = String::from_str("AA:BB:CC:DD:EE:FF").unwrap_or_default();
                
                self.stats.successful_connections += 1;
                self.reconnect_attempts = 0;

                return Ok(());
            }
        }

        // 连接失败
        self.status.state = ConnectionState::Error;
        self.stats.failed_connections += 1;
        Err(WiFiError::ConnectionFailed)
    }

    /// 断开WiFi连接
    pub async fn disconnect(&mut self) -> Result<(), WiFiError> {
        if self.status.state == ConnectionState::Connected {
            self.status.state = ConnectionState::Disconnected;
            self.stats.disconnections += 1;
            
            // 更新在线时间统计
            if let Some(start_time) = self.connection_start_time {
                self.stats.total_online_time_seconds += start_time.elapsed().as_secs();
            }
            
            self.connection_start_time = None;
        }

        // 清空状态信息
        self.status.ssid.clear();
        self.status.ip_address.clear();
        self.status.signal_strength = -100;
        self.status.channel = 0;

        Ok(())
    }

    /// 重新连接
    pub async fn reconnect(&mut self) -> Result<(), WiFiError> {
        if self.status.state == ConnectionState::Connected {
            self.disconnect().await?;
        }

        self.stats.reconnections += 1;
        self.reconnect_attempts += 1;

        // 等待一段时间后重连
        Timer::after(Duration::from_millis(1000)).await;
        
        self.connect().await
    }

    /// 获取WiFi状态
    pub async fn get_status(&mut self) -> Result<WiFiStatus, WiFiError> {
        // 更新连接时间
        if let Some(start_time) = self.connection_start_time {
            self.status.connection_time = start_time.elapsed().as_secs();
        }

        // 模拟信号强度变化
        if self.status.state == ConnectionState::Connected {
            // 信号强度在±5dBm范围内波动
            let variation = (rand_u8() % 11) as i8 - 5;
            self.status.signal_strength = (self.status.signal_strength + variation)
                .max(-100)
                .min(-30);
        }

        Ok(self.status.clone())
    }

    /// 获取扫描到的网络列表
    pub fn get_scanned_networks(&self) -> &Vec<WiFiNetworkInfo, 32> {
        &self.scanned_networks
    }

    /// 检查是否连接到指定网络
    pub fn is_connected_to(&self, ssid: &str) -> bool {
        self.status.state == ConnectionState::Connected && self.status.ssid == ssid
    }

    /// 获取当前连接的网络信息
    pub fn get_current_network(&self) -> Option<WiFiNetworkInfo> {
        if self.status.state == ConnectionState::Connected {
            self.scanned_networks.iter()
                .find(|network| network.ssid == self.status.ssid)
                .cloned()
        } else {
            None
        }
    }

    /// 更新配置
    pub async fn update_config(&mut self, config: WiFiConfig) -> Result<(), WiFiError> {
        let was_connected = self.status.state == ConnectionState::Connected;
        let ssid_changed = config.ssid != self.config.ssid;

        self.config = config;

        // 如果SSID改变且当前已连接，需要重新连接
        if was_connected && ssid_changed {
            self.disconnect().await?;
            self.connect().await?;
        }

        Ok(())
    }

    /// 获取统计信息
    pub fn get_stats(&self) -> WiFiStats {
        self.stats.clone()
    }

    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats = WiFiStats::default();
    }

    /// 进入省电模式
    pub async fn sleep(&mut self) -> Result<(), WiFiError> {
        if self.status.state == ConnectionState::Connected {
            // 保持连接但进入省电模式
            // 这里可以实现具体的省电逻辑
        }
        Ok(())
    }

    /// 从省电模式唤醒
    pub async fn wake_up(&mut self) -> Result<(), WiFiError> {
        if self.status.state == ConnectionState::Connected {
            // 从省电模式恢复
            // 这里可以实现具体的唤醒逻辑
        }
        Ok(())
    }

    /// 自动重连检查
    pub async fn auto_reconnect_check(&mut self) -> Result<(), WiFiError> {
        if !self.config.auto_reconnect {
            return Ok(());
        }

        if self.status.state == ConnectionState::Disconnected || 
           self.status.state == ConnectionState::Error {
            
            if self.reconnect_attempts < self.config.retry_count {
                self.reconnect().await?;
            }
        }

        Ok(())
    }

    /// 网络质量检查
    pub fn check_network_quality(&self) -> NetworkQuality {
        if self.status.state != ConnectionState::Connected {
            return NetworkQuality::Disconnected;
        }

        match self.status.signal_strength {
            -50..=0 => NetworkQuality::Excellent,
            -60..=-51 => NetworkQuality::Good,
            -70..=-61 => NetworkQuality::Fair,
            -80..=-71 => NetworkQuality::Poor,
            _ => NetworkQuality::VeryPoor,
        }
    }

    /// 获取推荐的网络
    pub fn get_recommended_networks(&self) -> Vec<&WiFiNetworkInfo, 8> {
        let mut recommended = Vec::new();
        
        // 按信号强度排序，选择最好的网络
        let mut sorted_networks: Vec<&WiFiNetworkInfo> = self.scanned_networks.iter().collect();
        sorted_networks.sort_by(|a, b| b.signal_strength.cmp(&a.signal_strength));

        for network in sorted_networks {
            if network.signal_strength >= self.config.min_signal_strength {
                if recommended.push(network).is_err() {
                    break;
                }
            }
        }

        recommended
    }
}

/// 网络质量等级
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkQuality {
    /// 断开连接
    Disconnected,
    /// 优秀
    Excellent,
    /// 良好
    Good,
    /// 一般
    Fair,
    /// 较差
    Poor,
    /// 很差
    VeryPoor,
}

/// 模拟随机成功
fn rand_success(probability: u8) -> bool {
    rand_u8() % 100 < probability
}

/// 模拟随机数生成
fn rand_u8() -> u8 {
    // 简单的伪随机数生成器
    static mut SEED: u32 = 1;
    unsafe {
        SEED = SEED.wrapping_mul(1103515245).wrapping_add(12345);
        (SEED >> 16) as u8
    }
}

impl From<WiFiError> for NetworkError {
    fn from(error: WiFiError) -> Self {
        NetworkError::WiFi(error)
    }
}

/// WiFi工具函数
pub mod wifi_utils {
    use super::*;

    /// 创建开放网络配置
    pub fn create_open_config(ssid: &str) -> WiFiConfig {
        WiFiConfig {
            ssid: String::from_str(ssid).unwrap_or_default(),
            password: String::new(),
            security: SecurityType::Open,
            ..Default::default()
        }
    }

    /// 创建WPA2网络配置
    pub fn create_wpa2_config(ssid: &str, password: &str) -> WiFiConfig {
        WiFiConfig {
            ssid: String::from_str(ssid).unwrap_or_default(),
            password: String::from_str(password).unwrap_or_default(),
            security: SecurityType::WPA2,
            ..Default::default()
        }
    }

    /// 信号强度转换为质量百分比
    pub fn signal_to_quality_percent(rssi: i8) -> u8 {
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

    /// 验证SSID格式
    pub fn validate_ssid(ssid: &str) -> bool {
        !ssid.is_empty() && ssid.len() <= 32 && ssid.chars().all(|c| c.is_ascii())
    }

    /// 验证密码强度
    pub fn validate_password_strength(password: &str, security: SecurityType) -> PasswordStrength {
        match security {
            SecurityType::Open => PasswordStrength::NotRequired,
            SecurityType::WEP => {
                if password.len() == 5 || password.len() == 13 {
                    PasswordStrength::Adequate
                } else {
                    PasswordStrength::Weak
                }
            },
            SecurityType::WPA | SecurityType::WPA2 | SecurityType::WPA3 => {
                if password.len() < 8 {
                    PasswordStrength::Weak
                } else if password.len() >= 12 && 
                         password.chars().any(|c| c.is_uppercase()) &&
                         password.chars().any(|c| c.is_lowercase()) &&
                         password.chars().any(|c| c.is_numeric()) {
                    PasswordStrength::Strong
                } else {
                    PasswordStrength::Adequate
                }
            },
            SecurityType::WPS => PasswordStrength::NotRequired,
        }
    }

    /// 估算连接时间
    pub fn estimate_connection_time(security: SecurityType, signal_strength: i8) -> u32 {
        let base_time = match security {
            SecurityType::Open => 2000,
            SecurityType::WEP => 3000,
            SecurityType::WPA => 4000,
            SecurityType::WPA2 => 5000,
            SecurityType::WPA3 => 6000,
            SecurityType::WPS => 8000,
        };

        // 信号强度影响连接时间
        let signal_factor = if signal_strength > -50 {
            1.0
        } else if signal_strength > -70 {
            1.2
        } else {
            1.5
        };

        (base_time as f32 * signal_factor) as u32
    }
}

/// 密码强度等级
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PasswordStrength {
    /// 不需要密码
    NotRequired,
    /// 弱密码
    Weak,
    /// 足够的密码
    Adequate,
    /// 强密码
    Strong,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_wifi_manager_creation() {
        let config = WiFiConfig {
            ssid: String::from_str("TestNetwork").unwrap(),
            password: String::from_str("password123").unwrap(),
            ..Default::default()
        };
        
        let manager = WiFiManager::new(config);
        assert!(manager.is_ok());
    }

    #[tokio::test]
    async fn test_network_scanning() {
        let config = WiFiConfig {
            ssid: String::from_str("TestNetwork").unwrap(),
            ..Default::default()
        };
        
        let mut manager = WiFiManager::new(config).unwrap();
        let networks = manager.scan_networks().await.unwrap();
        
        assert!(!networks.is_empty());
    }

    #[test]
    fn test_signal_quality_conversion() {
        assert_eq!(wifi_utils::signal_to_quality_percent(-40), 100);
        assert_eq!(wifi_utils::signal_to_quality_percent(-65), 60);
        assert_eq!(wifi_utils::signal_to_quality_percent(-95), 0);
    }

    #[test]
    fn test_ssid_validation() {
        assert!(wifi_utils::validate_ssid("ValidSSID"));
        assert!(!wifi_utils::validate_ssid(""));
        assert!(!wifi_utils::validate_ssid("ThisSSIDIsWayTooLongToBeValid123456"));
    }

    #[test]
    fn test_password_strength() {
        let strength = wifi_utils::validate_password_strength("Str0ngP@ssw0rd!", SecurityType::WPA2);
        assert_eq!(strength, PasswordStrength::Strong);
        
        let weak_strength = wifi_utils::validate_password_strength("123", SecurityType::WPA2);
        assert_eq!(weak_strength, PasswordStrength::Weak);
    }
}