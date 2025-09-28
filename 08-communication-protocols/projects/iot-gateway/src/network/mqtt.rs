//! MQTT协议管理模块
//! 
//! 负责MQTT客户端的连接、消息发布和订阅管理。

use heapless::{String, Vec, FnvIndexMap, Deque};
use embassy_time::{Duration, Timer, Instant};
use serde::{Deserialize, Serialize};

use super::NetworkError;

/// MQTT错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum MqttError {
    /// 连接失败
    ConnectionFailed,
    /// 认证失败
    AuthenticationFailed,
    /// 协议错误
    ProtocolError,
    /// 发布失败
    PublishFailed,
    /// 订阅失败
    SubscribeFailed,
    /// 超时
    Timeout,
    /// 配置错误
    ConfigError(&'static str),
    /// 缓冲区满
    BufferFull,
    /// 消息太大
    MessageTooLarge,
    /// 主题无效
    InvalidTopic,
    /// QoS不支持
    UnsupportedQoS,
    /// 网络错误
    NetworkError,
}

/// QoS等级
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum QoSLevel {
    /// 最多一次传递
    AtMostOnce = 0,
    /// 至少一次传递
    AtLeastOnce = 1,
    /// 恰好一次传递
    ExactlyOnce = 2,
}

/// MQTT连接状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MqttConnectionState {
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

/// MQTT配置
#[derive(Debug, Clone)]
pub struct MqttConfig {
    /// 代理服务器地址
    pub broker_host: String<64>,
    /// 代理服务器端口
    pub broker_port: u16,
    /// 客户端ID
    pub client_id: String<32>,
    /// 用户名
    pub username: Option<String<32>>,
    /// 密码
    pub password: Option<String<64>>,
    /// 保持连接时间（秒）
    pub keep_alive_seconds: u16,
    /// 清除会话
    pub clean_session: bool,
    /// 遗嘱主题
    pub will_topic: Option<String<64>>,
    /// 遗嘱消息
    pub will_message: Option<String<128>>,
    /// 遗嘱QoS
    pub will_qos: QoSLevel,
    /// 遗嘱保留标志
    pub will_retain: bool,
    /// 连接超时（毫秒）
    pub connect_timeout_ms: u32,
    /// 自动重连
    pub auto_reconnect: bool,
    /// 重连间隔（毫秒）
    pub reconnect_interval_ms: u32,
    /// 最大重连次数
    pub max_reconnect_attempts: u8,
}

impl Default for MqttConfig {
    fn default() -> Self {
        Self {
            broker_host: String::from_str("localhost").unwrap_or_default(),
            broker_port: 1883,
            client_id: String::from_str("iot-gateway").unwrap_or_default(),
            username: None,
            password: None,
            keep_alive_seconds: 60,
            clean_session: true,
            will_topic: None,
            will_message: None,
            will_qos: QoSLevel::AtMostOnce,
            will_retain: false,
            connect_timeout_ms: 30000,
            auto_reconnect: true,
            reconnect_interval_ms: 10000,
            max_reconnect_attempts: 5,
        }
    }
}

/// MQTT消息
#[derive(Debug, Clone)]
pub struct MqttMessage {
    /// 消息ID
    pub id: u16,
    /// 主题
    pub topic: String<64>,
    /// 负载数据
    pub payload: Vec<u8, 512>,
    /// QoS等级
    pub qos: QoSLevel,
    /// 保留标志
    pub retain: bool,
    /// 重复标志
    pub duplicate: bool,
    /// 时间戳
    pub timestamp: u64,
}

impl Default for MqttMessage {
    fn default() -> Self {
        Self {
            id: 0,
            topic: String::new(),
            payload: Vec::new(),
            qos: QoSLevel::AtMostOnce,
            retain: false,
            duplicate: false,
            timestamp: 0,
        }
    }
}

impl MqttMessage {
    /// 创建新消息
    pub fn new(topic: &str, payload: &[u8], qos: QoSLevel) -> Result<Self, MqttError> {
        let mut message = Self::default();
        
        message.topic = String::from_str(topic)
            .map_err(|_| MqttError::InvalidTopic)?;
        
        message.payload.extend_from_slice(payload)
            .map_err(|_| MqttError::MessageTooLarge)?;
        
        message.qos = qos;
        message.timestamp = Self::get_current_timestamp();
        message.id = Self::generate_message_id();
        
        Ok(message)
    }

    /// 创建文本消息
    pub fn new_text(topic: &str, text: &str, qos: QoSLevel) -> Result<Self, MqttError> {
        Self::new(topic, text.as_bytes(), qos)
    }

    /// 获取负载为字符串
    pub fn payload_as_string(&self) -> Result<String<512>, MqttError> {
        let text = core::str::from_utf8(&self.payload)
            .map_err(|_| MqttError::ProtocolError)?;
        
        String::from_str(text)
            .map_err(|_| MqttError::MessageTooLarge)
    }

    /// 获取当前时间戳
    fn get_current_timestamp() -> u64 {
        // 这里应该使用实际的时间获取函数
        1000000 // 模拟时间戳
    }

    /// 生成消息ID
    fn generate_message_id() -> u16 {
        static mut COUNTER: u16 = 0;
        unsafe {
            COUNTER = COUNTER.wrapping_add(1);
            if COUNTER == 0 {
                COUNTER = 1; // MQTT消息ID不能为0
            }
            COUNTER
        }
    }
}

/// MQTT订阅信息
#[derive(Debug, Clone)]
pub struct MqttSubscription {
    /// 主题过滤器
    pub topic_filter: String<64>,
    /// QoS等级
    pub qos: QoSLevel,
    /// 订阅时间
    pub subscribed_at: u64,
    /// 接收消息数
    pub message_count: u32,
}

/// MQTT统计信息
#[derive(Debug, Clone, Default)]
pub struct MqttStats {
    /// 连接尝试次数
    pub connection_attempts: u32,
    /// 成功连接次数
    pub successful_connections: u32,
    /// 连接失败次数
    pub failed_connections: u32,
    /// 断开连接次数
    pub disconnections: u32,
    /// 发布的消息数
    pub messages_published: u64,
    /// 接收的消息数
    pub messages_received: u64,
    /// 订阅次数
    pub subscriptions: u32,
    /// 取消订阅次数
    pub unsubscriptions: u32,
    /// 发布失败次数
    pub publish_failures: u32,
    /// 平均延迟（毫秒）
    pub avg_latency_ms: u32,
    /// 总在线时间（秒）
    pub total_online_time_seconds: u64,
}

/// MQTT管理器
pub struct MqttManager {
    /// 配置
    config: MqttConfig,
    /// 连接状态
    connection_state: MqttConnectionState,
    /// 统计信息
    stats: MqttStats,
    /// 订阅列表
    subscriptions: FnvIndexMap<String<64>, MqttSubscription, 32>,
    /// 接收消息队列
    received_messages: Deque<MqttMessage, 64>,
    /// 发送消息队列
    send_queue: Deque<MqttMessage, 32>,
    /// 连接开始时间
    connection_start_time: Option<Instant>,
    /// 最后心跳时间
    last_heartbeat_time: Instant,
    /// 重连尝试次数
    reconnect_attempts: u8,
    /// 下一个消息ID
    next_message_id: u16,
}

impl MqttManager {
    /// 创建新的MQTT管理器
    pub fn new(config: MqttConfig) -> Result<Self, MqttError> {
        // 验证配置
        if config.broker_host.is_empty() {
            return Err(MqttError::ConfigError("Broker host cannot be empty"));
        }

        if config.client_id.is_empty() {
            return Err(MqttError::ConfigError("Client ID cannot be empty"));
        }

        Ok(Self {
            config,
            connection_state: MqttConnectionState::Disconnected,
            stats: MqttStats::default(),
            subscriptions: FnvIndexMap::new(),
            received_messages: Deque::new(),
            send_queue: Deque::new(),
            connection_start_time: None,
            last_heartbeat_time: Instant::now(),
            reconnect_attempts: 0,
            next_message_id: 1,
        })
    }

    /// 连接到MQTT代理
    pub async fn connect(&mut self) -> Result<(), MqttError> {
        self.connection_state = MqttConnectionState::Connecting;
        self.connection_start_time = Some(Instant::now());
        self.stats.connection_attempts += 1;

        // 模拟连接过程
        Timer::after(Duration::from_millis(2000)).await;

        // 模拟连接成功（85%成功率）
        if rand_success(85) {
            self.connection_state = MqttConnectionState::Connected;
            self.stats.successful_connections += 1;
            self.reconnect_attempts = 0;
            self.last_heartbeat_time = Instant::now();
            
            Ok(())
        } else {
            self.connection_state = MqttConnectionState::Error;
            self.stats.failed_connections += 1;
            Err(MqttError::ConnectionFailed)
        }
    }

    /// 断开MQTT连接
    pub async fn disconnect(&mut self) -> Result<(), MqttError> {
        if self.connection_state == MqttConnectionState::Connected {
            self.connection_state = MqttConnectionState::Disconnecting;
            
            // 模拟断开过程
            Timer::after(Duration::from_millis(500)).await;
            
            self.connection_state = MqttConnectionState::Disconnected;
            self.stats.disconnections += 1;
            
            // 更新在线时间统计
            if let Some(start_time) = self.connection_start_time {
                self.stats.total_online_time_seconds += start_time.elapsed().as_secs();
            }
            
            self.connection_start_time = None;
        }

        Ok(())
    }

    /// 发布消息
    pub async fn publish(&mut self, topic: &str, payload: &[u8], qos: QoSLevel) -> Result<(), MqttError> {
        if self.connection_state != MqttConnectionState::Connected {
            return Err(MqttError::NetworkError);
        }

        let message = MqttMessage::new(topic, payload, qos)?;
        
        // 根据QoS等级处理消息
        match qos {
            QoSLevel::AtMostOnce => {
                // QoS 0: 直接发送，不等待确认
                self.send_message_internal(&message).await?;
            },
            QoSLevel::AtLeastOnce => {
                // QoS 1: 发送并等待PUBACK
                self.send_message_internal(&message).await?;
                self.wait_for_puback(message.id).await?;
            },
            QoSLevel::ExactlyOnce => {
                // QoS 2: 四次握手协议
                self.send_message_internal(&message).await?;
                self.wait_for_pubrec(message.id).await?;
                self.send_pubrel(message.id).await?;
                self.wait_for_pubcomp(message.id).await?;
            },
        }

        self.stats.messages_published += 1;
        Ok(())
    }

    /// 发布文本消息
    pub async fn publish_text(&mut self, topic: &str, text: &str, qos: QoSLevel) -> Result<(), MqttError> {
        self.publish(topic, text.as_bytes(), qos).await
    }

    /// 订阅主题
    pub async fn subscribe(&mut self, topic_filter: &str, qos: QoSLevel) -> Result<(), MqttError> {
        if self.connection_state != MqttConnectionState::Connected {
            return Err(MqttError::NetworkError);
        }

        // 验证主题过滤器
        if !self.validate_topic_filter(topic_filter) {
            return Err(MqttError::InvalidTopic);
        }

        let topic_string = String::from_str(topic_filter)
            .map_err(|_| MqttError::InvalidTopic)?;

        // 模拟订阅过程
        Timer::after(Duration::from_millis(500)).await;

        // 模拟订阅成功（95%成功率）
        if rand_success(95) {
            let subscription = MqttSubscription {
                topic_filter: topic_string.clone(),
                qos,
                subscribed_at: MqttMessage::get_current_timestamp(),
                message_count: 0,
            };

            self.subscriptions.insert(topic_string, subscription)
                .map_err(|_| MqttError::BufferFull)?;

            self.stats.subscriptions += 1;
            Ok(())
        } else {
            Err(MqttError::SubscribeFailed)
        }
    }

    /// 取消订阅
    pub async fn unsubscribe(&mut self, topic_filter: &str) -> Result<(), MqttError> {
        if self.connection_state != MqttConnectionState::Connected {
            return Err(MqttError::NetworkError);
        }

        let topic_string = String::from_str(topic_filter)
            .map_err(|_| MqttError::InvalidTopic)?;

        // 模拟取消订阅过程
        Timer::after(Duration::from_millis(300)).await;

        if self.subscriptions.remove(&topic_string).is_some() {
            self.stats.unsubscriptions += 1;
            Ok(())
        } else {
            Err(MqttError::InvalidTopic)
        }
    }

    /// 接收消息
    pub fn receive_message(&mut self) -> Option<MqttMessage> {
        self.received_messages.pop_front()
    }

    /// 检查是否有新消息
    pub fn has_messages(&self) -> bool {
        !self.received_messages.is_empty()
    }

    /// 获取消息数量
    pub fn message_count(&self) -> usize {
        self.received_messages.len()
    }

    /// 处理接收到的消息（模拟）
    pub async fn process_incoming_messages(&mut self) -> Result<(), MqttError> {
        if self.connection_state != MqttConnectionState::Connected {
            return Ok(());
        }

        // 模拟接收消息
        if rand_success(20) { // 20%概率接收到消息
            let topics: [&str; 4] = ["sensors/temperature", "sensors/humidity", "status/gateway", "commands/device"];
            let topic = topics[rand_u8() as usize % topics.len()];
            
            // 检查是否订阅了这个主题
            if self.is_subscribed_to_topic(topic) {
                let payload_data = match topic {
                    "sensors/temperature" => b"{\"temperature\": 25.5, \"unit\": \"C\"}",
                    "sensors/humidity" => b"{\"humidity\": 60.2, \"unit\": \"%\"}",
                    "status/gateway" => b"{\"status\": \"online\", \"uptime\": 3600}",
                    "commands/device" => b"{\"command\": \"restart\", \"device_id\": \"sensor_01\"}",
                    _ => b"{}",
                };

                let message = MqttMessage::new(topic, payload_data, QoSLevel::AtMostOnce)?;
                
                if self.received_messages.push_back(message).is_ok() {
                    self.stats.messages_received += 1;
                    
                    // 更新订阅统计
                    let topic_string = String::from_str(topic).unwrap_or_default();
                    if let Some(subscription) = self.subscriptions.get_mut(&topic_string) {
                        subscription.message_count += 1;
                    }
                }
            }
        }

        Ok(())
    }

    /// 更新MQTT管理器
    pub async fn update(&mut self) -> Result<(), MqttError> {
        // 处理心跳
        self.handle_heartbeat().await?;

        // 处理接收消息
        self.process_incoming_messages().await?;

        // 处理发送队列
        self.process_send_queue().await?;

        // 处理自动重连
        if self.config.auto_reconnect {
            self.handle_auto_reconnect().await?;
        }

        Ok(())
    }

    /// 处理心跳
    async fn handle_heartbeat(&mut self) -> Result<(), MqttError> {
        if self.connection_state != MqttConnectionState::Connected {
            return Ok(());
        }

        let heartbeat_interval = Duration::from_secs(self.config.keep_alive_seconds as u64);
        
        if self.last_heartbeat_time.elapsed() >= heartbeat_interval {
            // 发送PINGREQ
            // 这里应该发送实际的MQTT PINGREQ包
            
            self.last_heartbeat_time = Instant::now();
        }

        Ok(())
    }

    /// 处理发送队列
    async fn process_send_queue(&mut self) -> Result<(), MqttError> {
        while let Some(message) = self.send_queue.pop_front() {
            match self.send_message_internal(&message).await {
                Ok(()) => {
                    self.stats.messages_published += 1;
                },
                Err(e) => {
                    self.stats.publish_failures += 1;
                    return Err(e);
                }
            }
        }

        Ok(())
    }

    /// 处理自动重连
    async fn handle_auto_reconnect(&mut self) -> Result<(), MqttError> {
        if self.connection_state == MqttConnectionState::Disconnected || 
           self.connection_state == MqttConnectionState::Error {
            
            if self.reconnect_attempts < self.config.max_reconnect_attempts {
                self.reconnect_attempts += 1;
                
                // 等待重连间隔
                Timer::after(Duration::from_millis(self.config.reconnect_interval_ms as u64)).await;
                
                // 尝试重连
                let _ = self.connect().await;
            }
        }

        Ok(())
    }

    /// 内部发送消息
    async fn send_message_internal(&mut self, message: &MqttMessage) -> Result<(), MqttError> {
        if self.connection_state != MqttConnectionState::Connected {
            return Err(MqttError::NetworkError);
        }

        // 模拟发送延迟
        Timer::after(Duration::from_millis(50)).await;

        // 模拟发送成功（90%成功率）
        if rand_success(90) {
            Ok(())
        } else {
            Err(MqttError::PublishFailed)
        }
    }

    /// 等待PUBACK（QoS 1）
    async fn wait_for_puback(&mut self, message_id: u16) -> Result<(), MqttError> {
        // 模拟等待PUBACK
        Timer::after(Duration::from_millis(100)).await;
        
        if rand_success(95) {
            Ok(())
        } else {
            Err(MqttError::Timeout)
        }
    }

    /// 等待PUBREC（QoS 2）
    async fn wait_for_pubrec(&mut self, message_id: u16) -> Result<(), MqttError> {
        Timer::after(Duration::from_millis(100)).await;
        
        if rand_success(95) {
            Ok(())
        } else {
            Err(MqttError::Timeout)
        }
    }

    /// 发送PUBREL（QoS 2）
    async fn send_pubrel(&mut self, message_id: u16) -> Result<(), MqttError> {
        Timer::after(Duration::from_millis(50)).await;
        Ok(())
    }

    /// 等待PUBCOMP（QoS 2）
    async fn wait_for_pubcomp(&mut self, message_id: u16) -> Result<(), MqttError> {
        Timer::after(Duration::from_millis(100)).await;
        
        if rand_success(95) {
            Ok(())
        } else {
            Err(MqttError::Timeout)
        }
    }

    /// 验证主题过滤器
    fn validate_topic_filter(&self, topic_filter: &str) -> bool {
        if topic_filter.is_empty() || topic_filter.len() > 64 {
            return false;
        }

        // 检查有效字符
        for c in topic_filter.chars() {
            if !c.is_ascii() || c.is_control() {
                return false;
            }
        }

        // 检查通配符使用
        let mut chars = topic_filter.chars().peekable();
        while let Some(c) = chars.next() {
            match c {
                '+' => {
                    // 单级通配符必须占据整个级别
                    if chars.peek() == Some(&'/') || chars.peek().is_none() {
                        continue;
                    } else {
                        return false;
                    }
                },
                '#' => {
                    // 多级通配符必须是最后一个字符
                    return chars.next().is_none();
                },
                _ => continue,
            }
        }

        true
    }

    /// 检查是否订阅了指定主题
    fn is_subscribed_to_topic(&self, topic: &str) -> bool {
        for subscription in self.subscriptions.values() {
            if self.topic_matches_filter(topic, &subscription.topic_filter) {
                return true;
            }
        }
        false
    }

    /// 检查主题是否匹配过滤器
    fn topic_matches_filter(&self, topic: &str, filter: &str) -> bool {
        if filter == "#" {
            return true;
        }

        let topic_levels: Vec<&str> = topic.split('/').collect();
        let filter_levels: Vec<&str> = filter.split('/').collect();

        let mut topic_idx = 0;
        let mut filter_idx = 0;

        while topic_idx < topic_levels.len() && filter_idx < filter_levels.len() {
            let topic_level = topic_levels[topic_idx];
            let filter_level = filter_levels[filter_idx];

            match filter_level {
                "+" => {
                    // 单级通配符，匹配任意一级
                    topic_idx += 1;
                    filter_idx += 1;
                },
                "#" => {
                    // 多级通配符，匹配剩余所有级别
                    return true;
                },
                _ => {
                    // 精确匹配
                    if topic_level != filter_level {
                        return false;
                    }
                    topic_idx += 1;
                    filter_idx += 1;
                }
            }
        }

        // 检查是否都匹配完了
        topic_idx == topic_levels.len() && filter_idx == filter_levels.len()
    }

    /// 获取连接状态
    pub fn get_connection_state(&self) -> MqttConnectionState {
        self.connection_state
    }

    /// 获取统计信息
    pub fn get_stats(&self) -> MqttStats {
        self.stats.clone()
    }

    /// 获取订阅列表
    pub fn get_subscriptions(&self) -> Vec<&MqttSubscription, 32> {
        let mut subs = Vec::new();
        for subscription in self.subscriptions.values() {
            if subs.push(subscription).is_err() {
                break;
            }
        }
        subs
    }

    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats = MqttStats::default();
    }

    /// 清空消息队列
    pub fn clear_message_queues(&mut self) {
        self.received_messages.clear();
        self.send_queue.clear();
    }

    /// 更新配置
    pub async fn update_config(&mut self, config: MqttConfig) -> Result<(), MqttError> {
        let was_connected = self.connection_state == MqttConnectionState::Connected;
        
        self.config = config;

        // 如果当前已连接且关键配置改变，需要重新连接
        if was_connected {
            self.disconnect().await?;
            self.connect().await?;
        }

        Ok(())
    }
}

/// 模拟随机成功
fn rand_success(probability: u8) -> bool {
    rand_u8() % 100 < probability
}

/// 模拟随机数生成
fn rand_u8() -> u8 {
    static mut SEED: u32 = 12345;
    unsafe {
        SEED = SEED.wrapping_mul(1103515245).wrapping_add(12345);
        (SEED >> 16) as u8
    }
}

impl From<MqttError> for NetworkError {
    fn from(error: MqttError) -> Self {
        NetworkError::Mqtt(error)
    }
}

/// MQTT工具函数
pub mod mqtt_utils {
    use super::*;

    /// 创建基本MQTT配置
    pub fn create_basic_config(broker_host: &str, client_id: &str) -> MqttConfig {
        MqttConfig {
            broker_host: String::from_str(broker_host).unwrap_or_default(),
            client_id: String::from_str(client_id).unwrap_or_default(),
            ..Default::default()
        }
    }

    /// 创建带认证的MQTT配置
    pub fn create_auth_config(
        broker_host: &str,
        client_id: &str,
        username: &str,
        password: &str,
    ) -> MqttConfig {
        MqttConfig {
            broker_host: String::from_str(broker_host).unwrap_or_default(),
            client_id: String::from_str(client_id).unwrap_or_default(),
            username: Some(String::from_str(username).unwrap_or_default()),
            password: Some(String::from_str(password).unwrap_or_default()),
            ..Default::default()
        }
    }

    /// 验证主题名称
    pub fn validate_topic_name(topic: &str) -> bool {
        if topic.is_empty() || topic.len() > 64 {
            return false;
        }

        // 主题名称不能包含通配符
        if topic.contains('+') || topic.contains('#') {
            return false;
        }

        // 检查有效字符
        topic.chars().all(|c| c.is_ascii() && !c.is_control())
    }

    /// 验证客户端ID
    pub fn validate_client_id(client_id: &str) -> bool {
        if client_id.is_empty() || client_id.len() > 23 {
            return false;
        }

        // 客户端ID只能包含字母、数字和连字符
        client_id.chars().all(|c| c.is_alphanumeric() || c == '-' || c == '_')
    }

    /// 估算消息大小
    pub fn estimate_message_size(topic: &str, payload: &[u8]) -> usize {
        // MQTT固定头部 + 可变头部 + 主题长度 + 负载长度
        2 + 2 + topic.len() + payload.len()
    }

    /// 生成唯一客户端ID
    pub fn generate_client_id(prefix: &str) -> String<32> {
        let timestamp = MqttMessage::get_current_timestamp();
        let id_str = format!("{}-{}", prefix, timestamp % 100000);
        String::from_str(&id_str).unwrap_or_default()
    }

    /// QoS等级转换为字符串
    pub fn qos_to_string(qos: QoSLevel) -> &'static str {
        match qos {
            QoSLevel::AtMostOnce => "At Most Once",
            QoSLevel::AtLeastOnce => "At Least Once",
            QoSLevel::ExactlyOnce => "Exactly Once",
        }
    }

    /// 计算重连延迟（指数退避）
    pub fn calculate_backoff_delay(attempt: u8, base_delay_ms: u32) -> u32 {
        let max_delay_ms = 60000; // 最大1分钟
        let delay = base_delay_ms * (2_u32.pow(attempt as u32));
        core::cmp::min(delay, max_delay_ms)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_mqtt_manager_creation() {
        let config = MqttConfig {
            broker_host: String::from_str("test.mosquitto.org").unwrap(),
            client_id: String::from_str("test-client").unwrap(),
            ..Default::default()
        };
        
        let manager = MqttManager::new(config);
        assert!(manager.is_ok());
    }

    #[test]
    fn test_mqtt_message_creation() {
        let message = MqttMessage::new("test/topic", b"Hello, MQTT!", QoSLevel::AtMostOnce);
        assert!(message.is_ok());
        
        let msg = message.unwrap();
        assert_eq!(msg.topic, "test/topic");
        assert_eq!(msg.payload.as_slice(), b"Hello, MQTT!");
        assert_eq!(msg.qos, QoSLevel::AtMostOnce);
    }

    #[test]
    fn test_topic_validation() {
        assert!(mqtt_utils::validate_topic_name("sensors/temperature"));
        assert!(mqtt_utils::validate_topic_name("home/livingroom/light"));
        assert!(!mqtt_utils::validate_topic_name("sensors/+/temperature")); // 包含通配符
        assert!(!mqtt_utils::validate_topic_name("")); // 空主题
    }

    #[test]
    fn test_client_id_validation() {
        assert!(mqtt_utils::validate_client_id("client-123"));
        assert!(mqtt_utils::validate_client_id("IoT_Device_01"));
        assert!(!mqtt_utils::validate_client_id("")); // 空ID
        assert!(!mqtt_utils::validate_client_id("client with spaces")); // 包含空格
    }

    #[tokio::test]
    async fn test_topic_matching() {
        let config = MqttConfig::default();
        let manager = MqttManager::new(config).unwrap();
        
        assert!(manager.topic_matches_filter("sensors/temperature", "sensors/temperature"));
        assert!(manager.topic_matches_filter("sensors/temperature", "sensors/+"));
        assert!(manager.topic_matches_filter("sensors/temperature/room1", "sensors/#"));
        assert!(!manager.topic_matches_filter("actuators/light", "sensors/+"));
    }

    #[test]
    fn test_qos_conversion() {
        assert_eq!(mqtt_utils::qos_to_string(QoSLevel::AtMostOnce), "At Most Once");
        assert_eq!(mqtt_utils::qos_to_string(QoSLevel::AtLeastOnce), "At Least Once");
        assert_eq!(mqtt_utils::qos_to_string(QoSLevel::ExactlyOnce), "Exactly Once");
    }

    #[test]
    fn test_backoff_delay_calculation() {
        assert_eq!(mqtt_utils::calculate_backoff_delay(0, 1000), 1000);
        assert_eq!(mqtt_utils::calculate_backoff_delay(1, 1000), 2000);
        assert_eq!(mqtt_utils::calculate_backoff_delay(2, 1000), 4000);
        assert_eq!(mqtt_utils::calculate_backoff_delay(10, 1000), 60000); // 达到最大值
    }
}