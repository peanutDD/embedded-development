# 嵌入式开发示例集合

本目录包含了全面的嵌入式开发示例，涵盖了从基础概念到高级应用的各个方面。这些示例旨在帮助开发者理解和掌握嵌入式系统开发的核心技术和最佳实践。

## 目录结构

```
examples/
├── README.md                    # 本文档
├── platforms/                   # 多平台特定示例
│   ├── README.md               # 平台示例说明
│   ├── stm32/                  # STM32平台示例
│   ├── esp32/                  # ESP32平台示例
│   ├── rp2040/                 # RP2040平台示例
│   └── nrf52/                  # nRF52平台示例
├── cross-platform/             # 跨平台抽象示例
│   ├── README.md               # 跨平台开发指南
│   ├── Cargo.toml              # 跨平台项目配置
│   └── src/                    # 跨平台源代码
├── protocols/                  # 通信协议示例
│   ├── uart/                   # UART通信示例
│   ├── spi/                    # SPI通信示例
│   ├── i2c/                    # I2C通信示例
│   ├── can/                    # CAN总线示例
│   └── wireless/               # 无线通信示例
├── sensors/                    # 传感器应用示例
│   ├── temperature/            # 温度传感器
│   ├── humidity/               # 湿度传感器
│   ├── pressure/               # 压力传感器
│   ├── accelerometer/          # 加速度计
│   └── gyroscope/              # 陀螺仪
├── actuators/                  # 执行器控制示例
│   ├── motors/                 # 电机控制
│   ├── servos/                 # 舵机控制
│   ├── relays/                 # 继电器控制
│   └── displays/               # 显示器控制
├── power-management/           # 功耗管理示例
│   ├── sleep-modes/            # 睡眠模式
│   ├── clock-gating/           # 时钟门控
│   └── dynamic-voltage/        # 动态电压调节
├── real-time/                  # 实时系统示例
│   ├── rtos/                   # RTOS应用
│   ├── interrupts/             # 中断处理
│   └── scheduling/             # 任务调度
├── security/                   # 安全相关示例
│   ├── encryption/             # 加密算法
│   ├── authentication/         # 身份认证
│   └── secure-boot/            # 安全启动
├── networking/                 # 网络应用示例
│   ├── tcp-ip/                 # TCP/IP协议栈
│   ├── http/                   # HTTP客户端/服务器
│   ├── mqtt/                   # MQTT协议
│   └── coap/                   # CoAP协议
├── storage/                    # 存储系统示例
│   ├── flash/                  # Flash存储
│   ├── eeprom/                 # EEPROM操作
│   ├── sd-card/                # SD卡文件系统
│   └── database/               # 嵌入式数据库
├── testing/                    # 测试相关示例
│   ├── unit-tests/             # 单元测试
│   ├── integration-tests/      # 集成测试
│   ├── hardware-in-loop/       # 硬件在环测试
│   └── simulation/             # 仿真测试
└── tools/                      # 开发工具示例
    ├── debuggers/              # 调试器配置
    ├── profilers/              # 性能分析工具
    ├── code-generators/        # 代码生成工具
    └── build-systems/          # 构建系统配置
```

## 示例分类

### 1. 平台特定示例 (`platforms/`)

展示不同微控制器平台的特有功能和优化技术：

#### STM32平台
- **GPIO控制**: LED控制、按钮检测、中断处理
- **定时器应用**: PWM生成、输入捕获、定时中断
- **ADC/DAC**: 模拟信号采集和生成
- **通信接口**: UART、SPI、I2C、CAN
- **DMA传输**: 高效数据传输
- **低功耗模式**: 睡眠模式和唤醒机制

#### ESP32平台
- **WiFi连接**: 无线网络配置和通信
- **蓝牙通信**: BLE和经典蓝牙应用
- **Web服务器**: HTTP服务器和客户端
- **OTA更新**: 无线固件更新
- **传感器集成**: 多种传感器数据采集
- **云服务连接**: AWS、Azure、阿里云集成

#### RP2040平台
- **PIO编程**: 可编程I/O状态机
- **多核处理**: 双核协作和通信
- **USB设备**: HID、CDC、MSC设备实现
- **WS2812控制**: 可编程LED灯带
- **音频处理**: I2S音频输入输出
- **高速GPIO**: 高频信号处理

#### nRF52平台
- **蓝牙5.0**: BLE应用开发
- **Mesh网络**: 蓝牙Mesh组网
- **NFC通信**: 近场通信应用
- **超低功耗**: 极致功耗优化
- **传感器融合**: 多传感器数据融合
- **无线更新**: 蓝牙OTA升级

### 2. 跨平台抽象示例 (`cross-platform/`)

演示如何创建平台无关的抽象层：

- **硬件抽象层**: 统一的硬件接口定义
- **GPIO抽象**: 跨平台GPIO操作
- **传感器接口**: 统一的传感器读取接口
- **通信协议**: 平台无关的通信实现
- **错误处理**: 统一的错误处理机制
- **资源管理**: 跨平台资源管理策略

### 3. 通信协议示例 (`protocols/`)

各种通信协议的实现和应用：

#### 串行通信
- **UART**: 异步串行通信
- **SPI**: 同步串行外设接口
- **I2C**: 两线制串行总线

#### 网络通信
- **CAN**: 控制器局域网
- **Ethernet**: 以太网通信
- **WiFi**: 无线局域网
- **Bluetooth**: 蓝牙通信
- **LoRa**: 长距离低功耗通信

#### 应用层协议
- **HTTP/HTTPS**: Web协议
- **MQTT**: 消息队列遥测传输
- **CoAP**: 受限应用协议
- **Modbus**: 工业通信协议

### 4. 传感器应用示例 (`sensors/`)

各类传感器的驱动和应用：

#### 环境传感器
- **温湿度传感器**: DHT22、SHT30、BME280
- **气压传感器**: BMP280、MS5611
- **光照传感器**: BH1750、TSL2561
- **空气质量**: MQ系列、PMS5003

#### 运动传感器
- **加速度计**: ADXL345、MPU6050
- **陀螺仪**: L3GD20、ITG3200
- **磁力计**: HMC5883L、QMC5883L
- **IMU**: MPU9250、LSM9DS1

#### 位置传感器
- **GPS**: NEO-8M、NEO-M9N
- **超声波**: HC-SR04、JSN-SR04T
- **激光测距**: VL53L0X、VL53L1X
- **编码器**: 旋转编码器、线性编码器

### 5. 执行器控制示例 (`actuators/`)

各种执行器的控制方法：

#### 电机控制
- **直流电机**: PWM调速、方向控制
- **步进电机**: 步进控制、微步驱动
- **舵机**: PWM位置控制
- **无刷电机**: ESC控制、FOC算法

#### 显示控制
- **LCD显示**: 字符LCD、图形LCD
- **OLED显示**: SSD1306、SH1106
- **TFT显示**: ILI9341、ST7735
- **LED矩阵**: MAX7219、WS2812

#### 其他执行器
- **继电器**: 开关控制、时序控制
- **蜂鸣器**: 音调生成、音乐播放
- **振动器**: 触觉反馈
- **加热器**: 温度控制、PID调节

### 6. 功耗管理示例 (`power-management/`)

嵌入式系统的功耗优化技术：

#### 低功耗模式
- **睡眠模式**: 浅睡眠、深睡眠
- **待机模式**: 系统待机、外设待机
- **关断模式**: 最低功耗模式

#### 动态功耗管理
- **时钟门控**: 动态时钟控制
- **电压调节**: DVFS技术
- **负载均衡**: 任务分配优化

#### 功耗监测
- **功耗测量**: 电流监测、功率计算
- **能耗分析**: 功耗分布、优化建议
- **电池管理**: 电量监测、充电控制

### 7. 实时系统示例 (`real-time/`)

实时系统设计和实现：

#### RTOS应用
- **FreeRTOS**: 任务管理、同步机制
- **Embassy**: 异步运行时
- **RTIC**: 实时中断驱动并发

#### 中断处理
- **中断优先级**: 中断嵌套、优先级管理
- **中断延迟**: 延迟测量、优化技术
- **中断共享**: 中断资源管理

#### 任务调度
- **调度算法**: 优先级调度、时间片轮转
- **任务同步**: 信号量、互斥锁、消息队列
- **死锁避免**: 死锁检测、预防机制

### 8. 安全相关示例 (`security/`)

嵌入式系统安全技术：

#### 加密算法
- **对称加密**: AES、DES、ChaCha20
- **非对称加密**: RSA、ECC
- **哈希算法**: SHA-256、Blake2

#### 身份认证
- **数字签名**: ECDSA、RSA签名
- **证书验证**: X.509证书链
- **密钥管理**: 密钥生成、存储、更新

#### 安全启动
- **启动验证**: 固件签名验证
- **安全更新**: 加密固件更新
- **防篡改**: 硬件安全模块

### 9. 网络应用示例 (`networking/`)

网络通信和物联网应用：

#### 协议栈
- **TCP/IP**: 完整协议栈实现
- **UDP**: 无连接数据传输
- **ICMP**: 网络控制消息

#### 应用协议
- **HTTP服务器**: Web服务器实现
- **MQTT客户端**: 物联网消息传输
- **CoAP服务器**: 受限环境协议
- **WebSocket**: 实时双向通信

#### 网络安全
- **TLS/SSL**: 安全传输层
- **VPN**: 虚拟专用网络
- **防火墙**: 网络访问控制

### 10. 存储系统示例 (`storage/`)

数据存储和文件系统：

#### 存储介质
- **Flash存储**: 内部Flash、外部Flash
- **EEPROM**: 电可擦除存储器
- **SD卡**: FAT32文件系统
- **USB存储**: MSC设备实现

#### 文件系统
- **FAT32**: 兼容性最好的文件系统
- **LittleFS**: 嵌入式文件系统
- **SPIFFS**: SPI Flash文件系统

#### 数据库
- **SQLite**: 轻量级关系数据库
- **键值存储**: 简单数据存储
- **时序数据库**: 传感器数据存储

## 使用指南

### 环境准备

1. **安装Rust工具链**：
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add thumbv7em-none-eabihf  # Cortex-M4/M7
rustup target add thumbv6m-none-eabi     # Cortex-M0/M0+
```

2. **安装调试工具**：
```bash
cargo install probe-run
cargo install cargo-embed
cargo install cargo-flash
```

3. **安装平台特定工具**：
```bash
# ESP32工具链
cargo install espup
espup install

# RP2040工具
cargo install elf2uf2-rs
```

### 编译示例

```bash
# 编译特定平台示例
cd examples/platforms/stm32
cargo build --release

# 编译跨平台示例
cd examples/cross-platform
cargo build --features stm32

# 编译所有示例
cargo build --workspace
```

### 烧录和调试

```bash
# 使用probe-run烧录和运行
cargo run --bin gpio_control

# 使用cargo-embed调试
cargo embed --bin sensor_reading

# 使用cargo-flash烧录
cargo flash --bin communication_demo
```

## 学习路径

### 初学者路径

1. **基础概念** → `platforms/stm32/basic_gpio`
2. **串行通信** → `protocols/uart/basic_uart`
3. **传感器读取** → `sensors/temperature/ds18b20`
4. **执行器控制** → `actuators/motors/dc_motor`
5. **简单项目** → `platforms/stm32/temperature_monitor`

### 进阶路径

1. **中断处理** → `real-time/interrupts/gpio_interrupt`
2. **定时器应用** → `platforms/stm32/timer_pwm`
3. **通信协议** → `protocols/i2c/sensor_network`
4. **功耗管理** → `power-management/sleep-modes`
5. **复杂项目** → `platforms/esp32/iot_gateway`

### 高级路径

1. **RTOS应用** → `real-time/rtos/freertos_demo`
2. **网络通信** → `networking/tcp-ip/web_server`
3. **安全实现** → `security/encryption/aes_demo`
4. **跨平台开发** → `cross-platform/gpio_abstraction`
5. **系统集成** → `platforms/综合项目`

## 最佳实践

### 代码组织

- 使用模块化设计
- 清晰的接口定义
- 适当的抽象层次
- 统一的错误处理

### 性能优化

- 零成本抽象
- 编译时优化
- 内存使用优化
- 实时性保证

### 调试技巧

- 使用调试输出
- 硬件调试器
- 逻辑分析仪
- 示波器测量

### 测试策略

- 单元测试
- 集成测试
- 硬件在环测试
- 长期稳定性测试

## 贡献指南

欢迎贡献新的示例和改进现有代码！

### 贡献步骤

1. Fork本项目
2. 创建功能分支
3. 添加示例代码
4. 编写文档
5. 提交Pull Request

### 示例要求

- 代码清晰易懂
- 包含详细注释
- 提供README文档
- 通过基本测试
- 遵循代码规范

### 文档要求

- 功能描述清晰
- 使用方法详细
- 包含接线图
- 提供预期结果
- 列出注意事项

## 常见问题

### Q: 如何选择合适的示例？

A: 根据你的项目需求和技术水平选择。初学者建议从基础GPIO和串行通信开始，逐步学习更复杂的功能。

### Q: 示例代码可以直接用于产品吗？

A: 示例代码主要用于学习和原型验证。用于产品时需要考虑错误处理、安全性、可靠性等因素。

### Q: 如何移植示例到其他平台？

A: 参考跨平台示例，使用硬件抽象层隔离平台相关代码，然后实现目标平台的HAL接口。

### Q: 遇到编译错误怎么办？

A: 检查目标平台配置、依赖版本、特性开关等。参考各平台的README文档和错误信息进行排查。

## 技术支持

- **GitHub Issues**: 报告问题和建议
- **讨论区**: 技术交流和问答
- **Wiki**: 详细技术文档
- **示例视频**: 操作演示视频

## 许可证

本项目采用MIT许可证，详见各子项目的LICENSE文件。

---

**开始你的嵌入式开发之旅吧！** 🚀