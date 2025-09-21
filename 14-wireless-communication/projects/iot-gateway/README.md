# IoT Gateway 物联网网关

这是一个基于STM32F4的多协议物联网网关项目，支持WiFi、蓝牙、LoRa等多种无线通信协议，实现设备间的协议转换和数据路由。

## 项目特性

### 支持的通信协议
- **WiFi** - ESP8266/ESP32模块，支持TCP/UDP通信
- **蓝牙** - HC-05经典蓝牙和BLE低功耗蓝牙
- **LoRa** - SX1278长距离通信模块
- **MQTT** - 轻量级消息队列协议
- **CoAP** - 受约束应用协议
- **以太网** - 有线网络连接

### 核心功能
- 多协议设备管理
- 消息路由和协议转换
- 设备状态监控
- 数据缓存和转发
- 远程配置管理
- 实时状态显示

## 硬件连接

### 主控制器 (STM32F407VGT6)
- 系统时钟：84MHz
- Flash：1MB
- RAM：128KB
- 外设：UART×3, SPI×3, I2C×3, ADC×3

### WiFi模块 (ESP8266)
```
ESP8266    STM32F407
VCC    ->  3.3V
GND    ->  GND
TX     ->  PA10 (USART1_RX)
RX     ->  PA9  (USART1_TX)
EN     ->  PA8  (使能控制)
RST    ->  PA7  (复位控制)
```

### 蓝牙模块 (HC-05)
```
HC-05      STM32F407
VCC    ->  3.3V
GND    ->  GND
TX     ->  PA3  (USART2_RX)
RX     ->  PA2  (USART2_TX)
EN     ->  PA1  (使能控制)
STATE  ->  PA0  (状态检测)
```

### LoRa模块 (SX1278)
```
SX1278     STM32F407
VCC    ->  3.3V
GND    ->  GND
MOSI   ->  PB15 (SPI2_MOSI)
MISO   ->  PB14 (SPI2_MISO)
SCK    ->  PB13 (SPI2_SCK)
NSS    ->  PB12 (片选)
RST    ->  PB11 (复位)
DIO0   ->  PB10 (中断)
DIO1   ->  PB9  (中断)
```

### 显示屏 (SSD1306 OLED)
```
SSD1306    STM32F407
VCC    ->  3.3V
GND    ->  GND
SCL    ->  PB6  (I2C1_SCL)
SDA    ->  PB7  (I2C1_SDA)
```

### 状态指示LED
```
功能       引脚    说明
WiFi   ->  PC13   WiFi连接状态
蓝牙   ->  PC14   蓝牙连接状态
LoRa   ->  PC15   LoRa工作状态
系统   ->  PD12   系统运行状态
```

### 用户按键
```
按键       引脚    功能
KEY1   ->  PE4    系统复位
KEY2   ->  PE3    配置模式
KEY3   ->  PE2    网络重连
```

## 软件架构

### 系统组件
```
IoT Gateway
├── WiFi Controller      # WiFi模块控制
├── Bluetooth Controller # 蓝牙模块控制
├── LoRa Controller     # LoRa模块控制
├── MQTT Client         # MQTT客户端
├── Device Manager      # 设备管理器
├── Message Router      # 消息路由器
└── Display Manager     # 显示管理器
```

### 数据流程
1. **设备发现** - 扫描和识别各协议设备
2. **设备注册** - 将设备添加到管理列表
3. **消息接收** - 从各协议接收数据
4. **协议转换** - 根据路由表转换协议
5. **消息转发** - 将消息发送到目标设备
6. **状态监控** - 监控设备在线状态

### 消息格式
```rust
pub struct GatewayMessage {
    pub source_id: String<32>,      // 源设备ID
    pub target_id: String<32>,      // 目标设备ID
    pub message_type: MessageType,  // 消息类型
    pub protocol: Protocol,         // 通信协议
    pub payload: Vec<u8, 256>,      // 消息载荷
    pub timestamp: u32,             // 时间戳
    pub sequence: u16,              // 序列号
}
```

## 构建和烧录

### 环境准备
```bash
# 安装Rust工具链
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add thumbv7em-none-eabihf

# 安装调试工具
cargo install probe-run
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

### 编译项目
```bash
# 进入项目目录
cd 14-wireless-communication/projects/iot-gateway

# 编译项目
cargo build --release

# 生成二进制文件
cargo objcopy --release -- -O binary iot-gateway.bin
```

### 烧录程序
```bash
# 使用probe-run烧录
cargo run --release

# 或使用OpenOCD烧录
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program target/thumbv7em-none-eabihf/release/iot-gateway verify reset exit"
```

## 系统配置

### WiFi配置
```rust
// 连接WiFi网络
gateway.wifi.connect_network("YourSSID", "YourPassword")?;

// 启动TCP服务器
gateway.wifi.start_server(8080)?;
```

### MQTT配置
```rust
// 连接MQTT代理
gateway.mqtt.connect("mqtt.example.com", 1883)?;

// 订阅主题
gateway.mqtt.subscribe("gateway/+/command", 1)?;
gateway.mqtt.subscribe("sensors/+/data", 1)?;
```

### LoRa配置
```rust
// 设置LoRa参数
gateway.lora.set_frequency(433_000_000)?;  // 433MHz
gateway.lora.set_spreading_factor(7)?;     // SF7
gateway.lora.set_bandwidth(125000)?;       // 125kHz
gateway.lora.set_tx_power(14)?;            // 14dBm
```

## 功能特性

### 设备管理
- 自动设备发现和注册
- 设备状态实时监控
- 离线设备自动清理
- 设备信息持久化存储

### 消息路由
- 基于设备ID的智能路由
- 协议间自动转换
- 消息队列缓冲
- 重传机制

### 网络管理
- WiFi自动重连
- 网络状态监控
- 多网络接口支持
- 负载均衡

### 安全特性
- 消息加密传输
- 设备身份验证
- 访问控制列表
- 安全密钥管理

## 扩展功能

### 协议扩展
```rust
// 添加新的通信协议
impl Protocol for CustomProtocol {
    fn send_message(&mut self, message: &GatewayMessage) -> Result<(), Error>;
    fn receive_message(&mut self) -> Result<Option<GatewayMessage>, Error>;
    fn get_status(&self) -> ProtocolStatus;
}
```

### 设备类型扩展
```rust
// 添加新的设备类型
#[derive(Debug, Clone, Copy)]
pub enum CustomDeviceType {
    EnvironmentSensor,
    SecurityCamera,
    SmartLock,
    EnergyMeter,
}
```

### 消息处理扩展
```rust
// 自定义消息处理器
impl MessageProcessor for CustomProcessor {
    fn process_message(&mut self, message: &GatewayMessage) -> Result<(), Error>;
    fn get_supported_types(&self) -> &[MessageType];
}
```

## 调试和测试

### 串口调试
```bash
# 连接调试串口
minicom -D /dev/ttyUSB0 -b 115200

# 查看调试信息
RTT:INFO - Gateway initialized successfully
RTT:INFO - WiFi connected to: YourSSID
RTT:INFO - MQTT connected to: mqtt.example.com
RTT:INFO - LoRa module ready, frequency: 433MHz
```

### 网络测试
```bash
# 测试WiFi连接
ping 192.168.1.100

# 测试MQTT连接
mosquitto_pub -h mqtt.example.com -t "gateway/test" -m "Hello Gateway"

# 测试HTTP接口
curl http://192.168.1.100:8080/status
```

### LoRa测试
```bash
# 使用另一个LoRa设备发送测试消息
# 检查网关是否能正确接收和转发
```

## 性能优化

### 内存优化
- 使用heapless容器避免动态分配
- 消息池复用减少碎片
- 栈大小优化

### 功耗优化
- 模块按需启用
- 低功耗模式支持
- 智能休眠策略

### 通信优化
- 消息压缩算法
- 批量传输机制
- 自适应传输速率

## 故障排除

### 常见问题

1. **WiFi连接失败**
   - 检查SSID和密码
   - 确认信号强度
   - 重启WiFi模块

2. **蓝牙配对失败**
   - 清除配对历史
   - 检查PIN码
   - 重置蓝牙模块

3. **LoRa通信异常**
   - 检查天线连接
   - 确认频率设置
   - 调整发射功率

4. **MQTT连接断开**
   - 检查网络连接
   - 确认代理地址
   - 调整心跳间隔

### 调试命令
```rust
// 系统状态查询
let status = gateway.get_system_status();
println!("Uptime: {}s", status.uptime);
println!("Devices: {}", status.device_count);
println!("Queue: {}", status.message_queue_size);

// 设备列表查询
let devices = gateway.device_manager.get_online_devices(current_time, 60000);
for device in devices {
    println!("Device: {} ({})", device.id, device.protocol);
}
```

## 开发指南

### 添加新协议
1. 实现Protocol trait
2. 添加到MessageRouter
3. 更新配置结构
4. 编写测试用例

### 扩展设备类型
1. 定义设备结构
2. 实现设备接口
3. 添加到DeviceManager
4. 更新显示界面

### 自定义消息格式
1. 定义消息结构
2. 实现序列化方法
3. 添加校验机制
4. 更新路由逻辑

## 许可证

本项目采用MIT或Apache-2.0双重许可证。详见LICENSE文件。

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 联系方式

- 邮箱：dev@example.com
- 项目主页：https://github.com/example/embedded-development