# IoT Gateway - 物联网网关

这是一个完整的IoT网关实现，基于RTOS集成了传感器数据采集、处理、加密和云端通信功能，展示了企业级嵌入式系统的设计和实现。

## 项目概述

IoT网关是现代物联网系统的核心组件，负责连接边缘设备和云端服务。本项目实现了一个功能完整的IoT网关，包含数据采集、处理、传输和管理等核心功能。

### 核心功能
- **多传感器数据采集**: 支持I2C、SPI、ADC等多种接口
- **实时数据处理**: 数据验证、滤波和融合
- **安全数据传输**: 加密、认证和完整性校验
- **网络通信**: TCP/UDP、HTTP、MQTT协议支持
- **设备管理**: 远程配置、状态监控和固件更新
- **边缘计算**: 本地数据分析和决策

### 技术特色
- **RTIC框架**: 高效的实时任务调度
- **异步网络**: 非阻塞的网络通信
- **数据安全**: AES加密和HMAC认证
- **模块化设计**: 可扩展的系统架构
- **性能监控**: 完整的系统统计和分析

## 系统架构

### 整体架构图
```
┌─────────────────────────────────────────────────────────────────┐
│                      IoT Gateway System                        │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Sensor Manager  │  │ Network Manager │  │ Security Module │  │
│  │ - Temperature   │  │ - Ethernet      │  │ - AES Encrypt   │  │
│  │ - Humidity      │  │ - TCP/UDP       │  │ - HMAC Auth     │  │
│  │ - Light Level   │  │ - HTTP Client   │  │ - Data Integrity│  │
│  │ - Voltage       │  │ - MQTT Client   │  │ - Key Management│  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Data Processor  │  │ Storage Manager │  │ Device Manager  │  │
│  │ - Data Fusion   │  │ - Flash Storage │  │ - Configuration │  │
│  │ - Filtering     │  │ - Logging       │  │ - Status Report │  │
│  │ - Validation    │  │ - Buffering     │  │ - Remote Update │  │
│  │ - Analytics     │  │ - Compression   │  │ - Error Recovery│  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 任务调度架构
```
Priority 4: Emergency Handler    (紧急处理)
    │
Priority 3: Sensor Sampling     (传感器采样 - 1Hz)
    │
Priority 2: ┌─ Data Processing  (数据处理 - 5s)
            ├─ Network Comm     (网络通信 - 100ms)
            └─ Button Handler   (按钮处理)
    │
Priority 1: System Monitoring   (系统监控 - 10s)
    │
Idle Task:  Power Management    (功耗管理)
```

## 硬件要求

### 核心硬件
- **MCU**: STM32F407VG (Cortex-M4, 168MHz)
- **Flash**: 1MB (系统固件 + 数据存储)
- **RAM**: 192KB (128KB + 64KB CCM)
- **调试器**: ST-Link V2或兼容设备

### 网络模块
```
ENC28J60 以太网控制器:
- VCC  -> 3.3V
- GND  -> GND
- CS   -> PA4
- SCK  -> PA5
- MOSI -> PA7
- MISO -> PA6
- INT  -> PB0 (可选)
- RST  -> PB1 (可选)
```

### 传感器模块
```
I2C传感器总线 (400kHz):
- SCL  -> PB8
- SDA  -> PB9
- VCC  -> 3.3V
- GND  -> GND

支持的传感器:
- 温度传感器 (地址: 0x48)
- 湿度传感器 (地址: 0x40)
- 气压传感器 (地址: 0x76)
- 光照传感器 (地址: 0x23)
```

### 模拟传感器
```
ADC通道配置:
- PA0 -> 光照传感器 (ADC1_CH0)
- PA1 -> 电压监测 (ADC1_CH1)
- PA2 -> 电流监测 (ADC1_CH2)
- PA3 -> 备用通道 (ADC1_CH3)
```

### 指示和控制
```
LED指示:
- PA5 -> 系统状态LED (绿色)
- PA6 -> 网络状态LED (蓝色)
- PA7 -> 错误指示LED (红色)

用户交互:
- PC13 -> 用户按钮 (内部上拉)
- PC14 -> 复位按钮 (可选)
```

## 软件架构

### 核心数据结构

#### 传感器数据
```rust
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SensorReading {
    pub sensor_id: u8,
    pub sensor_type: SensorType,
    pub value: f32,
    pub timestamp: u32,
    pub quality: u8, // 数据质量 0-100
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum SensorType {
    Temperature,  // 温度 (°C)
    Humidity,     // 湿度 (%)
    Pressure,     // 气压 (hPa)
    Light,        // 光照 (lux)
    Motion,       // 运动检测
    Gas,          // 气体浓度
    Voltage,      // 电压 (V)
    Current,      // 电流 (A)
}
```

#### IoT数据包
```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IoTDataPacket {
    pub device_id: String<32>,
    pub timestamp: u32,
    pub sensors: Vec<SensorReading, SENSOR_COUNT>,
    pub status: DeviceStatus,
    pub checksum: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DeviceStatus {
    pub battery_level: u8,    // 电池电量 (%)
    pub signal_strength: i8,  // 信号强度 (dBm)
    pub temperature: f32,     // 设备温度 (°C)
    pub uptime: u32,         // 运行时间 (s)
    pub error_count: u16,    // 错误计数
}
```

#### 网关配置
```rust
#[derive(Debug, Clone)]
pub struct GatewayConfig {
    pub device_id: String<32>,
    pub server_address: IpAddress,
    pub server_port: u16,
    pub encryption_key: [u8; 16],
    pub sampling_interval: u32,    // 采样间隔 (ms)
    pub transmission_interval: u32, // 传输间隔 (ms)
}
```

### 任务实现

#### 传感器采样任务
```rust
#[task(shared = [system_state, device_status], local = [adc, i2c], priority = 3)]
fn sensor_sampling_task(mut ctx: sensor_sampling_task::Context) {
    // 并发读取多个传感器
    let temperature = read_temperature_sensor(&mut ctx.local.i2c);
    let humidity = read_humidity_sensor(&mut ctx.local.i2c);
    let light_level = read_light_sensor(&mut ctx.local.adc);
    let voltage = read_voltage_sensor(&mut ctx.local.adc);
    
    // 数据验证和质量评估
    let sensor_data = validate_and_package_data(temperature, humidity, light_level, voltage);
    
    // 更新共享状态
    ctx.shared.system_state.lock(|state| {
        update_sensor_data(state, sensor_data);
    });
}
```

#### 数据处理任务
```rust
#[task(shared = [sensor_data, gateway_config, device_status], local = [data_producer], priority = 2)]
fn data_processing_task(mut ctx: data_processing_task::Context) {
    let (sensor_data, config, status) = ctx.shared.lock(|data, cfg, stat| {
        (data.clone(), cfg.clone(), *stat)
    });
    
    // 创建IoT数据包
    let mut packet = IoTDataPacket {
        device_id: config.device_id.clone(),
        timestamp: monotonics::MyMono::now().ticks(),
        sensors: sensor_data,
        status,
        checksum: 0,
    };
    
    // 计算校验和
    packet.checksum = calculate_checksum(&packet);
    
    // 加密数据包
    if let Ok(encrypted_packet) = encrypt_packet(&packet, &config.encryption_key) {
        // 发送到网络队列
        ctx.local.data_producer.enqueue(encrypted_packet).ok();
    }
}
```

#### 网络通信任务
```rust
#[task(shared = [gateway_config, network_stats], local = [data_consumer, network_interface, sockets], priority = 2)]
fn network_communication_task(mut ctx: network_communication_task::Context) {
    // 处理网络接口
    let timestamp = Instant::from_millis(monotonics::MyMono::now().ticks() as i64);
    
    match ctx.local.network_interface.poll(timestamp, ctx.local.ethernet, ctx.local.sockets) {
        Ok(_) => {
            // 网络正常，处理数据传输
            while let Some(packet) = ctx.local.data_consumer.dequeue() {
                if let Ok(json_data) = serialize_packet(&packet) {
                    send_to_cloud(&json_data, ctx.local.sockets, &mut ctx.shared.gateway_config, &mut ctx.shared.network_stats);
                }
            }
        }
        Err(e) => {
            // 网络异常处理
            handle_network_error(e, &mut ctx.shared.network_stats);
        }
    }
}
```

## 安全特性

### 数据加密
```rust
use aes::Aes128;
use aes::cipher::{BlockEncrypt, KeyInit, generic_array::GenericArray};

fn encrypt_packet(packet: &IoTDataPacket, key: &[u8; 16]) -> Result<IoTDataPacket, ()> {
    let cipher = Aes128::new(GenericArray::from_slice(key));
    
    // 序列化数据包
    let serialized = serialize_packet(packet)?;
    
    // AES加密
    let mut encrypted_data = [0u8; 128];
    encrypt_data(&cipher, &serialized, &mut encrypted_data)?;
    
    // 创建加密后的数据包
    Ok(create_encrypted_packet(encrypted_data))
}
```

### 数据完整性
```rust
use hmac::{Hmac, Mac};
use sha2::Sha256;

type HmacSha256 = Hmac<Sha256>;

fn calculate_hmac(data: &[u8], key: &[u8]) -> Result<[u8; 32], ()> {
    let mut mac = HmacSha256::new_from_slice(key).map_err(|_| ())?;
    mac.update(data);
    let result = mac.finalize();
    Ok(result.into_bytes().into())
}
```

### 认证机制
```rust
pub struct AuthenticationManager {
    device_key: [u8; 32],
    session_token: Option<[u8; 16]>,
    token_expiry: u32,
}

impl AuthenticationManager {
    pub fn authenticate(&mut self, challenge: &[u8]) -> Result<[u8; 32], AuthError> {
        // 实现设备认证逻辑
        let response = self.generate_auth_response(challenge)?;
        self.update_session_token()?;
        Ok(response)
    }
}
```

## 网络协议支持

### HTTP客户端
```rust
pub struct HttpClient {
    socket_handle: SocketHandle,
    server_endpoint: SocketAddr,
}

impl HttpClient {
    pub fn send_post_request(&mut self, path: &str, data: &[u8]) -> Result<(), HttpError> {
        let request = format!(
            "POST {} HTTP/1.1\r\n\
             Host: {}\r\n\
             Content-Type: application/json\r\n\
             Content-Length: {}\r\n\
             \r\n",
            path, self.server_endpoint.ip(), data.len()
        );
        
        // 发送HTTP请求
        self.send_data(request.as_bytes())?;
        self.send_data(data)?;
        
        Ok(())
    }
}
```

### MQTT客户端 (简化实现)
```rust
pub struct MqttClient {
    client_id: String<32>,
    keep_alive: u16,
    connection_state: MqttState,
}

impl MqttClient {
    pub fn publish(&mut self, topic: &str, payload: &[u8]) -> Result<(), MqttError> {
        let packet = create_publish_packet(topic, payload)?;
        self.send_packet(&packet)?;
        Ok(())
    }
    
    pub fn subscribe(&mut self, topic: &str) -> Result<(), MqttError> {
        let packet = create_subscribe_packet(topic)?;
        self.send_packet(&packet)?;
        Ok(())
    }
}
```

## 性能监控

### 系统统计
```rust
#[derive(Debug, Default)]
pub struct SystemStats {
    pub uptime: u64,
    pub cpu_usage: u8,
    pub memory_usage: u32,
    pub task_switches: u32,
    pub interrupt_count: u32,
}

#[derive(Debug, Default)]
pub struct NetworkStats {
    pub packets_sent: u32,
    pub packets_received: u32,
    pub bytes_sent: u32,
    pub bytes_received: u32,
    pub connection_errors: u16,
    pub transmission_errors: u16,
}
```

### 性能分析
```rust
fn analyze_system_performance(stats: &SystemStats) -> PerformanceReport {
    PerformanceReport {
        cpu_efficiency: calculate_cpu_efficiency(stats),
        memory_efficiency: calculate_memory_efficiency(stats),
        network_throughput: calculate_network_throughput(stats),
        real_time_compliance: check_real_time_compliance(stats),
    }
}
```

## 编译和部署

### 环境准备
```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf
cargo install probe-run cargo-embed

# 安装网络和加密相关工具
cargo install smoltcp-utils
```

### 编译配置
```toml
# Cargo.toml 关键配置
[dependencies]
smoltcp = { version = "0.10", features = ["proto-ipv4", "proto-tcp", "proto-udp"] }
aes = "0.8"
sha2 = "0.10"
hmac = "0.12"
serde = { version = "1.0", features = ["derive"] }
serde-json-core = "0.5"
```

### 构建和烧录
```bash
# 编译项目
cargo build --release

# 烧录到设备
cargo run --release

# 或使用embed
cargo embed --release
```

## 测试和验证

### 单元测试
```bash
# 运行所有测试
cargo test

# 运行特定模块测试
cargo test sensor_manager
cargo test network_stack
cargo test security_module
```

### 集成测试
```bash
# 硬件在环测试
cargo run --bin integration_test

# 网络连接测试
cargo run --bin network_test

# 性能基准测试
cargo bench
```

### 功能验证清单
- [ ] 传感器数据采集正常
- [ ] 网络连接建立成功
- [ ] 数据加密和传输正常
- [ ] 系统监控功能正常
- [ ] 错误处理和恢复正常
- [ ] 性能指标符合要求

## 部署和运维

### 生产部署
1. **固件配置**: 设置生产环境参数
2. **安全配置**: 配置加密密钥和证书
3. **网络配置**: 设置服务器地址和端口
4. **监控配置**: 启用远程监控和日志

### 远程管理
```rust
pub struct RemoteManager {
    config_version: u32,
    last_update: u32,
    update_server: String<64>,
}

impl RemoteManager {
    pub fn check_for_updates(&mut self) -> Result<bool, UpdateError> {
        // 检查固件更新
    }
    
    pub fn apply_config_update(&mut self, config: &GatewayConfig) -> Result<(), ConfigError> {
        // 应用配置更新
    }
}
```

### 故障诊断
```rust
pub struct DiagnosticManager {
    error_log: Vec<ErrorRecord, 32>,
    system_health: SystemHealth,
}

impl DiagnosticManager {
    pub fn run_diagnostics(&mut self) -> DiagnosticReport {
        DiagnosticReport {
            hardware_status: self.check_hardware(),
            network_status: self.check_network(),
            sensor_status: self.check_sensors(),
            memory_status: self.check_memory(),
        }
    }
}
```

## 扩展和定制

### 添加新传感器
```rust
pub trait SensorInterface {
    type Error;
    
    fn initialize(&mut self) -> Result<(), Self::Error>;
    fn read_data(&mut self) -> Result<SensorReading, Self::Error>;
    fn get_status(&self) -> SensorStatus;
}

// 实现新传感器
pub struct CustomSensor {
    // 传感器特定配置
}

impl SensorInterface for CustomSensor {
    type Error = CustomSensorError;
    
    fn initialize(&mut self) -> Result<(), Self::Error> {
        // 初始化逻辑
    }
    
    fn read_data(&mut self) -> Result<SensorReading, Self::Error> {
        // 数据读取逻辑
    }
}
```

### 扩展网络协议
```rust
pub trait NetworkProtocol {
    type Error;
    
    fn connect(&mut self) -> Result<(), Self::Error>;
    fn send_data(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    fn receive_data(&mut self) -> Result<Vec<u8>, Self::Error>;
    fn disconnect(&mut self) -> Result<(), Self::Error>;
}

// 实现新协议
pub struct CustomProtocol {
    // 协议特定配置
}

impl NetworkProtocol for CustomProtocol {
    type Error = CustomProtocolError;
    
    // 实现协议方法
}
```

## 性能优化建议

### 内存优化
1. 使用静态内存分配
2. 优化数据结构大小
3. 实现内存池管理
4. 减少栈使用

### 网络优化
1. 实现数据压缩
2. 使用连接池
3. 优化缓冲区大小
4. 实现流量控制

### 实时性优化
1. 优化任务优先级
2. 减少中断延迟
3. 使用DMA传输
4. 优化算法复杂度

## 故障排除

### 常见问题

#### 网络连接问题
```bash
# 检查网络配置
ping 192.168.1.100

# 检查端口连接
telnet 192.168.1.100 8080
```

#### 传感器读取问题
```rust
// 添加调试信息
rprintln!("I2C read result: {:?}", result);
rprintln!("Sensor address: 0x{:02X}", sensor_addr);
```

#### 内存不足问题
```bash
# 检查内存使用
cargo size --release

# 分析栈使用
cargo stack-sizes --release
```

### 调试技巧
1. 使用RTT实时日志
2. 添加性能计数器
3. 实现系统状态转储
4. 使用硬件调试器

## 相关资源

### 技术文档
- [RTIC官方文档](https://rtic.rs/)
- [smoltcp网络栈](https://github.com/smoltcp-rs/smoltcp)
- [STM32F4xx HAL](https://docs.rs/stm32f4xx-hal/)

### 开源项目
- [Embassy异步框架](https://embassy.dev/)
- [Rust嵌入式工作组](https://github.com/rust-embedded)

### 学习资源
- [嵌入式Rust书籍](https://docs.rust-embedded.org/book/)
- [Discovery教程](https://docs.rust-embedded.org/discovery/)

## 许可证

本项目采用MIT或Apache-2.0双重许可证。详见LICENSE文件。