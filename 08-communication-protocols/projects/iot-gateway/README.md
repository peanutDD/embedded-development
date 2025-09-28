# IoT Gateway

一个用于嵌入式系统的物联网网关，支持多种通信协议和数据路由功能。

## 功能特性

### 🌐 网络连接
- **WiFi 支持**: ESP32/ESP8266 WiFi 连接
- **以太网支持**: 有线网络连接
- **蜂窝网络**: 4G/5G 模块支持
- **LoRa**: 长距离低功耗通信
- **Zigbee**: 智能家居协议支持
- **蓝牙**: BLE 设备连接

### 📡 通信协议
- **MQTT**: 轻量级消息队列协议
- **HTTP/HTTPS**: RESTful API 支持
- **CoAP**: 受限应用协议
- **WebSocket**: 实时双向通信
- **TCP/UDP**: 底层网络协议

### 🔒 安全功能
- **TLS/SSL**: 加密传输
- **AES 加密**: 数据加密
- **HMAC 认证**: 消息认证
- **证书管理**: X.509 证书支持

### 💾 数据管理
- **数据缓存**: 本地数据存储
- **数据转换**: 协议间数据转换
- **数据路由**: 智能数据路由
- **离线存储**: 网络断开时的数据保存

### ⚡ 性能优化
- **异步处理**: Embassy 异步框架
- **内存优化**: 无堆内存设计
- **低功耗**: 电源管理支持
- **实时性**: 硬实时任务支持

## 项目结构

```
iot-gateway/
├── src/
│   ├── lib.rs              # 库入口
│   ├── main.rs             # 主程序
│   ├── gateway/            # 网关核心
│   │   ├── mod.rs          # 网关模块
│   │   ├── manager.rs      # 网关管理器
│   │   ├── router.rs       # 数据路由器
│   │   └── config.rs       # 配置管理
│   ├── network/            # 网络层
│   │   ├── mod.rs          # 网络模块
│   │   ├── wifi.rs         # WiFi 驱动
│   │   ├── ethernet.rs     # 以太网驱动
│   │   ├── cellular.rs     # 蜂窝网络
│   │   ├── lora.rs         # LoRa 通信
│   │   └── bluetooth.rs    # 蓝牙通信
│   ├── protocols/          # 协议层
│   │   ├── mod.rs          # 协议模块
│   │   ├── mqtt.rs         # MQTT 协议
│   │   ├── http.rs         # HTTP 协议
│   │   ├── coap.rs         # CoAP 协议
│   │   └── websocket.rs    # WebSocket 协议
│   ├── security/           # 安全模块
│   │   ├── mod.rs          # 安全模块
│   │   ├── tls.rs          # TLS 实现
│   │   ├── crypto.rs       # 加密算法
│   │   └── auth.rs         # 认证管理
│   ├── storage/            # 存储模块
│   │   ├── mod.rs          # 存储模块
│   │   ├── cache.rs        # 数据缓存
│   │   ├── flash.rs        # Flash 存储
│   │   └── config.rs       # 配置存储
│   └── utils/              # 工具模块
│       ├── mod.rs          # 工具模块
│       ├── buffer.rs       # 缓冲区管理
│       ├── timer.rs        # 定时器工具
│       └── logger.rs       # 日志系统
├── examples/               # 示例程序
│   ├── basic_gateway.rs    # 基础网关示例
│   ├── wifi_gateway.rs     # WiFi 网关示例
│   ├── ethernet_gateway.rs # 以太网网关示例
│   └── secure_gateway.rs   # 安全网关示例
├── docs/                   # 文档
│   ├── architecture.md     # 架构设计
│   ├── protocols.md        # 协议说明
│   ├── security.md         # 安全设计
│   └── deployment.md       # 部署指南
├── tests/                  # 测试
│   ├── integration/        # 集成测试
│   └── unit/              # 单元测试
├── Cargo.toml             # 项目配置
└── README.md              # 项目说明
```

## 快速开始

### 1. 基础网关示例

```rust
use iot_gateway::prelude::*;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // 初始化网关
    let config = GatewayConfig::default();
    let mut gateway = Gateway::new(config);
    
    // 启动网关服务
    gateway.start().await.unwrap();
    
    // 运行网关
    gateway.run().await;
}
```

### 2. WiFi MQTT 网关

```rust
use iot_gateway::prelude::*;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // WiFi 配置
    let wifi_config = WiFiConfig {
        ssid: "your-wifi-ssid",
        password: "your-wifi-password",
    };
    
    // MQTT 配置
    let mqtt_config = MqttConfig {
        broker: "mqtt.example.com",
        port: 1883,
        client_id: "iot-gateway-001",
    };
    
    // 创建网关
    let mut gateway = Gateway::builder()
        .with_wifi(wifi_config)
        .with_mqtt(mqtt_config)
        .build();
    
    // 启动并运行
    gateway.start().await.unwrap();
    gateway.run().await;
}
```

### 3. 安全网关示例

```rust
use iot_gateway::prelude::*;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // TLS 配置
    let tls_config = TlsConfig {
        ca_cert: include_bytes!("../certs/ca.pem"),
        client_cert: include_bytes!("../certs/client.pem"),
        client_key: include_bytes!("../certs/client.key"),
    };
    
    // 创建安全网关
    let mut gateway = Gateway::builder()
        .with_tls(tls_config)
        .with_encryption(true)
        .build();
    
    gateway.start().await.unwrap();
    gateway.run().await;
}
```

## 支持的平台

- **STM32**: STM32F4, STM32F7, STM32H7 系列
- **ESP32**: ESP32, ESP32-S2, ESP32-S3, ESP32-C3
- **nRF52**: nRF52832, nRF52840
- **RP2040**: Raspberry Pi Pico

## 开发环境

### 依赖要求

- Rust 1.70+
- Embassy 异步框架
- embedded-hal 0.2+

### 构建项目

```bash
# 克隆项目
git clone https://github.com/your-username/iot-gateway.git
cd iot-gateway

# 构建项目
cargo build

# 运行示例
cargo run --example basic_gateway --features "mqtt,http"

# 运行测试
cargo test
```

### 交叉编译

```bash
# 为 ARM Cortex-M4 编译
cargo build --target thumbv7em-none-eabihf

# 为 ESP32 编译
cargo build --target xtensa-esp32-none-elf
```

## 配置选项

### 网络配置

```toml
[features]
# 网络接口
wifi = ["esp-wifi"]
ethernet = ["smoltcp/medium-ethernet"]
cellular = []
lora = []

# 协议支持
mqtt = ["minimq"]
http = ["embedded-svc"]
coap = []
websocket = []

# 安全功能
tls = []
encryption = ["aes", "sha2", "hmac"]
```

### 运行时配置

```rust
let config = GatewayConfig {
    // 网络设置
    network: NetworkConfig {
        interface: NetworkInterface::WiFi,
        dhcp: true,
        static_ip: None,
    },
    
    // 协议设置
    protocols: ProtocolConfig {
        mqtt_enabled: true,
        http_enabled: true,
        coap_enabled: false,
    },
    
    // 安全设置
    security: SecurityConfig {
        tls_enabled: true,
        encryption_enabled: true,
        auth_required: true,
    },
    
    // 存储设置
    storage: StorageConfig {
        cache_size: 1024,
        flash_enabled: true,
        backup_enabled: true,
    },
};
```

## API 文档

### 核心 API

- `Gateway`: 网关主结构
- `GatewayManager`: 网关管理器
- `DataRouter`: 数据路由器
- `ProtocolHandler`: 协议处理器

### 网络 API

- `NetworkManager`: 网络管理器
- `WiFiDriver`: WiFi 驱动
- `EthernetDriver`: 以太网驱动
- `CellularDriver`: 蜂窝网络驱动

### 协议 API

- `MqttClient`: MQTT 客户端
- `HttpClient`: HTTP 客户端
- `CoapClient`: CoAP 客户端
- `WebSocketClient`: WebSocket 客户端

## 性能指标

### 内存使用

- **RAM**: 最小 32KB，推荐 64KB+
- **Flash**: 最小 128KB，推荐 256KB+
- **堆内存**: 无需堆内存（no_std）

### 网络性能

- **吞吐量**: 最高 10Mbps（取决于硬件）
- **延迟**: < 10ms（本地网络）
- **并发连接**: 最多 16 个

### 功耗

- **运行模式**: 50-200mA
- **空闲模式**: 10-50mA
- **睡眠模式**: < 1mA

## 故障排除

### 常见问题

1. **编译错误**: 检查 Rust 版本和依赖
2. **网络连接失败**: 检查网络配置和硬件连接
3. **内存不足**: 调整缓冲区大小和功能配置
4. **协议错误**: 检查协议配置和服务器设置

### 调试工具

- 使用 `defmt` 进行日志输出
- 使用 `probe-rs` 进行调试
- 使用网络抓包工具分析协议

## 贡献指南

1. Fork 项目
2. 创建功能分支
3. 提交更改
4. 创建 Pull Request

## 许可证

本项目采用 MIT 或 Apache-2.0 双重许可证。

## 联系方式

- 项目主页: https://github.com/your-username/iot-gateway
- 问题反馈: https://github.com/your-username/iot-gateway/issues
- 邮箱: your.email@example.com