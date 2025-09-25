# 08 - I2C/SPI 通信协议

本章节深入学习 I2C 和 SPI 通信协议的实现和应用，涵盖从基础理论到实际项目的完整学习路径。

## 📚 学习内容

### 核心协议
- **I2C 协议**: 双线串行通信协议，支持多主多从架构
- **SPI 协议**: 高速同步串行通信协议，全双工通信
- **协议对比**: 性能、复杂度、应用场景分析

### 实践技能
- 多设备通信管理与仲裁
- 协议调试技巧与故障排除
- 常见传感器接口集成
- 性能优化与错误处理

## 📁 目录结构

```
08-communication-protocols/
├── README.md                    # 本文件
├── docs/                       # 详细文档
│   ├── 01-i2c-protocol.md      # I2C协议详解
│   ├── 02-spi-protocol.md      # SPI协议详解
│   ├── 03-protocol-comparison.md # 协议对比分析
│   ├── 04-debugging-guide.md   # 调试指南
│   └── 05-sensor-integration.md # 传感器集成指南
├── examples/                   # 代码示例
│   ├── basic-i2c/             # I2C基础示例
│   │   ├── Cargo.toml
│   │   ├── README.md
│   │   └── src/
│   │       ├── main.rs         # I2C基础通信
│   │       ├── scanner.rs      # I2C设备扫描
│   │       └── lib.rs
│   ├── basic-spi/             # SPI基础示例
│   │   ├── Cargo.toml
│   │   ├── README.md
│   │   └── src/
│   │       ├── main.rs         # SPI基础通信
│   │       ├── modes.rs        # SPI模式配置
│   │       └── lib.rs
│   ├── i2c-eeprom/            # I2C EEPROM操作
│   │   ├── Cargo.toml
│   │   ├── README.md
│   │   └── src/
│   │       ├── main.rs         # EEPROM读写示例
│   │       ├── driver.rs       # EEPROM驱动
│   │       └── lib.rs
│   ├── spi-flash/             # SPI Flash操作
│   │   ├── Cargo.toml
│   │   ├── README.md
│   │   └── src/
│   │       ├── main.rs         # Flash读写示例
│   │       ├── driver.rs       # Flash驱动
│   │       └── lib.rs
│   ├── digital-io/            # 数字IO扩展
│   │   ├── Cargo.toml
│   │   ├── README.md
│   │   └── src/
│   │       ├── main.rs         # IO扩展示例
│   │       ├── pcf8574.rs      # PCF8574驱动
│   │       └── lib.rs
│   ├── multi-device/          # 多设备通信
│   │   ├── Cargo.toml
│   │   ├── README.md
│   │   └── src/
│   │       ├── main.rs         # 多设备管理
│   │       ├── bus_manager.rs  # 总线管理器
│   │       └── lib.rs
│   └── protocol-bridge/       # 协议桥接
│       ├── Cargo.toml
│       ├── README.md
│       └── src/
│           ├── main.rs         # I2C-SPI桥接
│           ├── bridge.rs       # 桥接实现
│           └── lib.rs
└── projects/                  # 实践项目
    ├── sensor-hub/            # 传感器集线器
    │   ├── Cargo.toml
    │   ├── README.md
    │   ├── docs/              # 项目文档
    │   │   ├── architecture.md # 架构设计
    │   │   ├── sensors.md     # 传感器说明
    │   │   └── api.md         # API文档
    │   └── src/
    │       ├── main.rs         # 主程序
    │       ├── sensors/        # 传感器模块
    │       │   ├── mod.rs
    │       │   ├── temperature.rs # 温度传感器
    │       │   ├── humidity.rs    # 湿度传感器
    │       │   ├── pressure.rs    # 压力传感器
    │       │   └── light.rs       # 光照传感器
    │       ├── communication/  # 通信模块
    │       │   ├── mod.rs
    │       │   ├── i2c_manager.rs # I2C管理器
    │       │   └── spi_manager.rs # SPI管理器
    │       ├── data/          # 数据处理
    │       │   ├── mod.rs
    │       │   ├── collector.rs   # 数据收集
    │       │   ├── processor.rs   # 数据处理
    │       │   └── storage.rs     # 数据存储
    │       └── lib.rs
    ├── display-controller/    # 显示控制器
    │   ├── Cargo.toml
    │   ├── README.md
    │   └── src/
    │       ├── main.rs         # 显示控制主程序
    │       ├── displays/       # 显示设备
    │       │   ├── mod.rs
    │       │   ├── oled.rs     # OLED显示
    │       │   ├── lcd.rs      # LCD显示
    │       │   └── epaper.rs   # 电子纸显示
    │       ├── graphics/       # 图形处理
    │       │   ├── mod.rs
    │       │   ├── renderer.rs # 渲染器
    │       │   └── fonts.rs    # 字体管理
    │       └── lib.rs
    └── iot-gateway/           # IoT网关
        ├── Cargo.toml
        ├── README.md
        ├── config/            # 配置文件
        │   ├── devices.toml   # 设备配置
        │   └── network.toml   # 网络配置
        └── src/
            ├── main.rs         # 网关主程序
            ├── devices/        # 设备管理
            │   ├── mod.rs
            │   ├── manager.rs  # 设备管理器
            │   └── registry.rs # 设备注册表
            ├── protocols/      # 协议处理
            │   ├── mod.rs
            │   ├── i2c.rs      # I2C协议处理
            │   ├── spi.rs      # SPI协议处理
            │   └── uart.rs     # UART协议处理
            ├── network/        # 网络通信
            │   ├── mod.rs
            │   ├── wifi.rs     # WiFi通信
            │   └── mqtt.rs     # MQTT协议
            └── lib.rs
```

## 🎯 学习目标

完成本章节后，你将能够：

### 基础能力
- ✅ 理解 I2C 和 SPI 协议的工作原理
- ✅ 掌握协议的电气特性和时序要求
- ✅ 实现基本的读写操作

### 进阶技能
- ✅ 处理多设备通信场景和总线仲裁
- ✅ 实现错误检测和恢复机制
- ✅ 优化通信性能和功耗

### 实践项目
- ✅ 集成常见传感器和执行器
- ✅ 构建复杂的通信系统
- ✅ 开发可重用的驱动程序

## 🚀 快速开始

### 环境准备
```bash
# 进入项目目录
cd 08-communication-protocols

# 安装依赖
cargo check

# 运行基础示例
cd examples/basic-i2c
cargo run --example scanner

cd ../basic-spi
cargo run --example loopback
```

### 学习路径

#### 1️⃣ 基础理论学习
- 阅读 [I2C协议详解](docs/01-i2c-protocol.md)
- 阅读 [SPI协议详解](docs/02-spi-protocol.md)
- 学习 [协议对比分析](docs/03-protocol-comparison.md)

#### 2️⃣ 基础示例实践
```bash
# I2C基础示例
cd examples/basic-i2c
cargo run                    # 基础通信
cargo run --bin scanner      # 设备扫描

# SPI基础示例
cd examples/basic-spi
cargo run                    # 基础通信
cargo run --bin modes        # 模式配置
```

#### 3️⃣ 设备驱动开发
```bash
# EEPROM驱动
cd examples/i2c-eeprom
cargo run --example read_write

# Flash驱动
cd examples/spi-flash
cargo run --example sector_erase
```

#### 4️⃣ 综合项目实践
```bash
# 传感器集线器
cd projects/sensor-hub
cargo run

# 显示控制器
cd projects/display-controller
cargo run --example oled_demo

# IoT网关
cd projects/iot-gateway
cargo run
```

## 📊 协议对比

| 特性 | I2C | SPI | 备注 |
|------|-----|-----|------|
| **线数** | 2线 (SDA, SCL) | 4线+ (MOSI, MISO, SCK, CS) | SPI需要更多GPIO |
| **速度** | 标准100kHz, 快速400kHz, 高速3.4MHz | 通常MHz级别 | SPI速度更快 |
| **设备数** | 理论127个 | 受CS引脚限制 | I2C支持更多设备 |
| **复杂度** | 较复杂 | 相对简单 | I2C需要地址管理 |
| **功耗** | 较低 | 较高 | I2C开漏输出更省电 |
| **距离** | 短距离 | 短距离 | 都适合PCB内通信 |

## 🛠️ 开发工具

### 硬件工具
- **逻辑分析仪**: 协议时序分析
- **示波器**: 信号质量检查
- **I2C/SPI调试器**: 专用协议调试工具

### 软件工具
- **probe-rs**: Rust嵌入式调试
- **RTT**: 实时传输调试
- **defmt**: 高效日志记录

## 🔧 调试技巧

### 常见问题
1. **I2C通信失败**
   - 检查上拉电阻 (通常4.7kΩ)
   - 验证设备地址
   - 确认时钟频率

2. **SPI数据错误**
   - 检查时钟极性和相位
   - 验证片选信号时序
   - 确认数据位序

3. **多设备冲突**
   - 实现总线仲裁
   - 添加重试机制
   - 使用互斥锁保护

### 调试流程
```rust
// 启用调试日志
use defmt_rtt as _;
use panic_probe as _;

// I2C调试示例
defmt::info!("Scanning I2C bus...");
for addr in 0x08..=0x77 {
    if i2c.write(addr, &[]).is_ok() {
        defmt::info!("Found device at 0x{:02X}", addr);
    }
}
```

## 📚 学习资源

### 官方文档
- [I2C规范 v6.0](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
- [SPI协议介绍](https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html)
- [STM32 I2C参考手册](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

### 开源项目
- [embedded-hal](https://github.com/rust-embedded/embedded-hal) - Rust嵌入式HAL
- [stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal) - STM32F4 HAL实现
- [shared-bus](https://github.com/Rahix/shared-bus) - 总线共享库

### 在线工具
- [I2C地址计算器](https://www.i2c-bus.org/addressing/)
- [SPI时序生成器](https://www.analog.com/en/design-center/interactive-design-tools/spi-timing-diagram-tool.html)

## 🤝 贡献指南

欢迎提交问题报告、功能请求和代码贡献：

1. **问题报告**: 详细描述问题和复现步骤
2. **功能请求**: 说明需求和使用场景
3. **代码贡献**: 遵循项目编码规范
4. **文档改进**: 修正错误或添加示例

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](../LICENSE) 文件了解详情。

---

**版本**: v1.0  
**最后更新**: 2024年  
**维护者**: 嵌入式Rust学习小组