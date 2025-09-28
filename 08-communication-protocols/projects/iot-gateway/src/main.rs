//! IoT Gateway 主程序
//! 
//! 这是 IoT Gateway 的主程序入口，演示如何使用网关库
//! 来创建一个完整的物联网网关系统。

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use iot_gateway::{
    prelude::*,
    gateway::{Gateway, GatewayConfig, GatewayManager},
    network::{NetworkManager, NetworkConfig, NetworkInterface},
    protocols::{ProtocolManager, ProtocolConfig},
    security::{SecurityManager, SecurityConfig},
    storage::{StorageManager, StorageConfig},
};

/// 系统配置
struct SystemConfig {
    gateway: GatewayConfig,
    network: NetworkConfig,
    protocols: ProtocolConfig,
    security: SecurityConfig,
    storage: StorageConfig,
}

impl Default for SystemConfig {
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

/// 主系统结构
struct System {
    gateway: Gateway,
    network_manager: NetworkManager,
    protocol_manager: ProtocolManager,
    security_manager: SecurityManager,
    storage_manager: StorageManager,
}

impl System {
    /// 创建新的系统实例
    fn new(config: SystemConfig) -> Result<Self, GatewayError> {
        let gateway = Gateway::new(config.gateway)?;
        let network_manager = NetworkManager::new(config.network)?;
        let protocol_manager = ProtocolManager::new(config.protocols)?;
        let security_manager = SecurityManager::new(config.security)?;
        let storage_manager = StorageManager::new(config.storage)?;

        Ok(Self {
            gateway,
            network_manager,
            protocol_manager,
            security_manager,
            storage_manager,
        })
    }

    /// 初始化系统
    async fn init(&mut self) -> Result<(), GatewayError> {
        // 初始化存储管理器
        self.storage_manager.init().await?;
        
        // 初始化安全管理器
        self.security_manager.init().await?;
        
        // 初始化网络管理器
        self.network_manager.init().await?;
        
        // 初始化协议管理器
        self.protocol_manager.init().await?;
        
        // 初始化网关
        self.gateway.init().await?;

        Ok(())
    }

    /// 启动系统
    async fn start(&mut self) -> Result<(), GatewayError> {
        // 启动存储服务
        self.storage_manager.start().await?;
        
        // 启动安全服务
        self.security_manager.start().await?;
        
        // 启动网络服务
        self.network_manager.start().await?;
        
        // 启动协议服务
        self.protocol_manager.start().await?;
        
        // 启动网关服务
        self.gateway.start().await?;

        Ok(())
    }

    /// 运行系统主循环
    async fn run(&mut self) -> ! {
        let mut tick_counter = 0u32;
        
        loop {
            // 更新各个管理器
            let _ = self.storage_manager.update().await;
            let _ = self.security_manager.update().await;
            let _ = self.network_manager.update().await;
            let _ = self.protocol_manager.update().await;
            let _ = self.gateway.update().await;

            // 处理数据路由
            self.process_data_routing().await;

            // 健康检查
            if tick_counter % 100 == 0 {
                self.health_check().await;
            }

            // 统计信息更新
            if tick_counter % 1000 == 0 {
                self.update_statistics().await;
            }

            tick_counter = tick_counter.wrapping_add(1);
            Timer::after(Duration::from_millis(10)).await;
        }
    }

    /// 处理数据路由
    async fn process_data_routing(&mut self) {
        // 从网络接收数据
        if let Ok(data) = self.network_manager.receive_data().await {
            // 安全验证
            if let Ok(verified_data) = self.security_manager.verify_data(&data).await {
                // 协议处理
                if let Ok(processed_data) = self.protocol_manager.process_data(&verified_data).await {
                    // 路由到目标
                    let _ = self.gateway.route_data(&processed_data).await;
                }
            }
        }

        // 从网关发送数据
        if let Ok(data) = self.gateway.get_outbound_data().await {
            // 协议封装
            if let Ok(packaged_data) = self.protocol_manager.package_data(&data).await {
                // 安全加密
                if let Ok(encrypted_data) = self.security_manager.encrypt_data(&packaged_data).await {
                    // 网络发送
                    let _ = self.network_manager.send_data(&encrypted_data).await;
                }
            }
        }
    }

    /// 健康检查
    async fn health_check(&mut self) {
        let _ = self.storage_manager.health_check().await;
        let _ = self.security_manager.health_check().await;
        let _ = self.network_manager.health_check().await;
        let _ = self.protocol_manager.health_check().await;
        let _ = self.gateway.health_check().await;
    }

    /// 更新统计信息
    async fn update_statistics(&mut self) {
        let _ = self.gateway.update_statistics().await;
    }

    /// 系统维护
    async fn maintenance(&mut self) -> Result<(), GatewayError> {
        self.storage_manager.maintenance().await?;
        self.security_manager.maintenance().await?;
        self.network_manager.maintenance().await?;
        self.protocol_manager.maintenance().await?;
        self.gateway.maintenance().await?;
        Ok(())
    }

    /// 获取系统信息
    async fn get_system_info(&self) -> SystemInfo {
        SystemInfo {
            gateway_info: self.gateway.get_info().await,
            network_info: self.network_manager.get_info().await,
            protocol_info: self.protocol_manager.get_info().await,
            security_info: self.security_manager.get_info().await,
            storage_info: self.storage_manager.get_info().await,
        }
    }
}

/// 系统信息
#[derive(Debug)]
struct SystemInfo {
    gateway_info: GatewayInfo,
    network_info: NetworkInfo,
    protocol_info: ProtocolInfo,
    security_info: SecurityInfo,
    storage_info: StorageInfo,
}

#[entry]
fn main() -> ! {
    // 初始化系统配置
    let config = SystemConfig::default();
    
    // 创建系统实例
    let mut system = match System::new(config) {
        Ok(sys) => sys,
        Err(_) => {
            // 处理初始化错误
            loop {}
        }
    };

    // 启动 Embassy 执行器
    let executor = embassy_executor::Executor::new();
    executor.run(|spawner| {
        // 生成主任务
        spawner.spawn(main_task(system)).unwrap();
        
        // 生成网络任务
        spawner.spawn(network_task()).unwrap();
        
        // 生成协议任务
        spawner.spawn(protocol_task()).unwrap();
        
        // 生成安全任务
        spawner.spawn(security_task()).unwrap();
        
        // 生成存储任务
        spawner.spawn(storage_task()).unwrap();
    })
}

/// 主任务
#[embassy_executor::task]
async fn main_task(mut system: System) {
    // 初始化系统
    if let Err(_) = system.init().await {
        loop {}
    }

    // 启动系统
    if let Err(_) = system.start().await {
        loop {}
    }

    // 运行系统
    system.run().await;
}

/// 网络任务
#[embassy_executor::task]
async fn network_task() {
    loop {
        // 处理网络事件
        Timer::after(Duration::from_millis(50)).await;
    }
}

/// 协议任务
#[embassy_executor::task]
async fn protocol_task() {
    loop {
        // 处理协议事件
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// 安全任务
#[embassy_executor::task]
async fn security_task() {
    loop {
        // 处理安全事件
        Timer::after(Duration::from_millis(200)).await;
    }
}

/// 存储任务
#[embassy_executor::task]
async fn storage_task() {
    loop {
        // 处理存储事件
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// 演示基本网关功能
async fn demo_basic_gateway() -> Result<(), GatewayError> {
    // 创建基本配置
    let config = GatewayConfig {
        device_id: "gateway-001",
        max_connections: 16,
        buffer_size: 1024,
        timeout_ms: 5000,
        retry_count: 3,
        ..Default::default()
    };

    // 创建网关
    let mut gateway = Gateway::new(config)?;

    // 初始化和启动
    gateway.init().await?;
    gateway.start().await?;

    // 模拟数据处理
    for i in 0..10 {
        let data = format!("sensor_data_{}", i);
        gateway.process_data(data.as_bytes()).await?;
        Timer::after(Duration::from_millis(1000)).await;
    }

    Ok(())
}

/// 演示 WiFi MQTT 网关
async fn demo_wifi_mqtt_gateway() -> Result<(), GatewayError> {
    // WiFi 配置
    let wifi_config = NetworkConfig {
        interface: NetworkInterface::WiFi,
        ssid: Some("IoT-Network"),
        password: Some("password123"),
        dhcp: true,
        static_ip: None,
        dns_servers: &["8.8.8.8", "8.8.4.4"],
    };

    // MQTT 配置
    let mqtt_config = ProtocolConfig {
        mqtt_enabled: true,
        mqtt_broker: Some("mqtt.iot-platform.com"),
        mqtt_port: 1883,
        mqtt_client_id: Some("gateway-wifi-001"),
        mqtt_username: Some("gateway_user"),
        mqtt_password: Some("gateway_pass"),
        ..Default::default()
    };

    // 创建系统配置
    let system_config = SystemConfig {
        network: wifi_config,
        protocols: mqtt_config,
        ..Default::default()
    };

    // 创建并运行系统
    let mut system = System::new(system_config)?;
    system.init().await?;
    system.start().await?;

    // 模拟 MQTT 消息处理
    for i in 0..5 {
        let topic = format!("sensors/temperature/{}", i);
        let payload = format!("{{\"temperature\": {}, \"humidity\": {}}}", 20 + i, 50 + i);
        
        // 发布 MQTT 消息
        system.protocol_manager.publish_mqtt(&topic, payload.as_bytes()).await?;
        Timer::after(Duration::from_millis(2000)).await;
    }

    Ok(())
}

/// 演示安全网关
async fn demo_secure_gateway() -> Result<(), GatewayError> {
    // 安全配置
    let security_config = SecurityConfig {
        tls_enabled: true,
        encryption_enabled: true,
        auth_required: true,
        cert_validation: true,
        key_rotation_interval: 3600, // 1小时
        max_auth_attempts: 3,
    };

    // 创建安全网关
    let system_config = SystemConfig {
        security: security_config,
        ..Default::default()
    };

    let mut system = System::new(system_config)?;
    system.init().await?;
    system.start().await?;

    // 模拟安全数据传输
    for i in 0..3 {
        let sensitive_data = format!("confidential_data_{}", i);
        
        // 加密数据
        let encrypted = system.security_manager.encrypt_data(sensitive_data.as_bytes()).await?;
        
        // 传输加密数据
        system.network_manager.send_data(&encrypted).await?;
        
        Timer::after(Duration::from_millis(3000)).await;
    }

    Ok(())
}

/// 演示数据存储和缓存
async fn demo_storage_cache() -> Result<(), GatewayError> {
    // 存储配置
    let storage_config = StorageConfig {
        cache_enabled: true,
        cache_size: 2048,
        flash_enabled: true,
        backup_enabled: true,
        compression_enabled: true,
        retention_days: 30,
    };

    let system_config = SystemConfig {
        storage: storage_config,
        ..Default::default()
    };

    let mut system = System::new(system_config)?;
    system.init().await?;
    system.start().await?;

    // 模拟数据存储
    for i in 0..20 {
        let data = format!("{{\"id\": {}, \"value\": {}, \"timestamp\": {}}}", 
                          i, i * 10, i * 1000);
        
        // 存储数据
        system.storage_manager.store_data(&format!("sensor_{}", i), data.as_bytes()).await?;
        
        // 每5条数据查询一次
        if i % 5 == 0 {
            let stored_data = system.storage_manager.retrieve_data(&format!("sensor_{}", i)).await?;
            // 处理检索到的数据
        }
        
        Timer::after(Duration::from_millis(500)).await;
    }

    Ok(())
}

/// 演示系统监控和统计
async fn demo_monitoring() -> Result<(), GatewayError> {
    let mut system = System::new(SystemConfig::default())?;
    system.init().await?;
    system.start().await?;

    // 运行监控循环
    for _ in 0..10 {
        // 获取系统信息
        let info = system.get_system_info().await;
        
        // 执行健康检查
        system.health_check().await;
        
        // 更新统计信息
        system.update_statistics().await;
        
        // 执行维护任务
        if let Err(_) = system.maintenance().await {
            // 处理维护错误
        }
        
        Timer::after(Duration::from_millis(5000)).await;
    }

    Ok(())
}