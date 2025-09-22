# 多平台嵌入式开发示例

本目录包含针对不同嵌入式平台的开发示例，展示了如何在各种微控制器上实现相同的功能。

## 支持的平台

### 1. STM32 系列
- **目录**: `stm32/`
- **芯片**: STM32F4xx系列（主要针对STM32F407VG）
- **特点**: 
  - 高性能ARM Cortex-M4内核
  - 丰富的外设接口
  - 强大的HAL库支持
  - 适合复杂应用开发

### 2. ESP32 系列
- **目录**: `esp32/`
- **芯片**: ESP32, ESP32-S2, ESP32-C3等
- **特点**:
  - 内置WiFi和蓝牙
  - 双核处理器
  - 丰富的网络功能
  - 适合物联网应用

### 3. RP2040 (Raspberry Pi Pico)
- **目录**: `rp2040/`
- **芯片**: RP2040
- **特点**:
  - 双核ARM Cortex-M0+
  - 可编程I/O (PIO)
  - 低成本高性能
  - 适合教育和原型开发

### 4. nRF52 系列
- **目录**: `nrf52/`
- **芯片**: nRF52832, nRF52840
- **特点**:
  - 低功耗蓝牙专家
  - 优秀的射频性能
  - 先进的电源管理
  - 适合可穿戴和IoT设备

## 项目结构

每个平台目录都包含以下结构：

```
platform/
├── Cargo.toml          # 项目配置和依赖
├── memory.x            # 内存布局配置（如适用）
├── src/
│   ├── lib.rs          # 平台抽象库
│   └── bin/            # 示例程序
│       ├── basic_gpio.rs
│       ├── uart_communication.rs
│       ├── i2c_sensor.rs
│       ├── spi_display.rs
│       ├── adc_reading.rs
│       ├── pwm_control.rs
│       └── ...         # 平台特定示例
└── README.md           # 平台特定说明
```

## 通用示例程序

所有平台都实现了以下基础示例：

### 1. 基础GPIO控制 (`basic_gpio.rs`)
- LED控制
- 按钮检测
- 数字输入输出

### 2. 串口通信 (`uart_communication.rs`)
- 串口数据发送接收
- 调试输出
- 协议解析

### 3. I2C传感器 (`i2c_sensor.rs`)
- 温湿度传感器读取
- 多设备通信
- 错误处理

### 4. SPI显示 (`spi_display.rs`)
- OLED/LCD显示控制
- 图形绘制
- 用户界面

### 5. ADC读取 (`adc_reading.rs`)
- 模拟信号采集
- 电压测量
- 数据处理

### 6. PWM控制 (`pwm_control.rs`)
- 电机控制
- LED调光
- 音频输出

## 平台特定示例

每个平台还包含其独有的示例：

### STM32 特有功能
- 以太网通信
- USB设备
- 加密功能
- 高级定时器

### ESP32 特有功能
- WiFi连接
- HTTP服务器
- MQTT客户端
- OTA更新

### RP2040 特有功能
- PIO编程
- 多核处理
- DMA传输
- USB HID

### nRF52 特有功能
- 蓝牙信标
- BLE外设
- NFC标签
- 低功耗模式

## 开发环境配置

### 通用要求
```bash
# 安装Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 安装目标架构
rustup target add thumbv7em-none-eabihf  # STM32F4
rustup target add xtensa-esp32-none-elf   # ESP32
rustup target add thumbv6m-none-eabi      # RP2040
rustup target add thumbv7em-none-eabihf   # nRF52
```

### 平台特定工具

#### STM32
```bash
# 安装调试工具
cargo install probe-rs --features cli
# 或使用OpenOCD + GDB
```

#### ESP32
```bash
# 安装ESP-IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
source ./export.sh

# 安装cargo-espflash
cargo install cargo-espflash
```

#### RP2040
```bash
# 安装probe-rs
cargo install probe-rs --features cli
# 或使用picotool
```

#### nRF52
```bash
# 安装nRF工具
# 下载nRF Command Line Tools
# 安装probe-rs
cargo install probe-rs --features cli
```

## 编译和烧录

### STM32
```bash
cd stm32
cargo build --release --bin basic_gpio
probe-rs run --chip STM32F407VGTx target/thumbv7em-none-eabihf/release/basic_gpio
```

### ESP32
```bash
cd esp32
cargo build --release --bin wifi_connection
cargo espflash flash --monitor target/xtensa-esp32-none-elf/release/wifi_connection
```

### RP2040
```bash
cd rp2040
cargo build --release --bin basic_gpio
probe-rs run --chip RP2040 target/thumbv6m-none-eabi/release/basic_gpio
```

### nRF52
```bash
cd nrf52
cargo build --release --bin bluetooth_beacon
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/bluetooth_beacon
```

## 调试技巧

### 1. 串口调试
所有平台都支持通过串口输出调试信息：
```rust
defmt::info!("调试信息: {}", value);
```

### 2. 探针调试
使用probe-rs进行在线调试：
```bash
probe-rs attach --chip <CHIP_NAME>
```

### 3. RTT调试
实时传输调试信息：
```rust
use defmt_rtt as _;
defmt::info!("RTT调试输出");
```

## 性能对比

| 平台 | 主频 | RAM | Flash | 特色功能 |
|------|------|-----|-------|----------|
| STM32F407 | 168MHz | 192KB | 1MB | 高性能，丰富外设 |
| ESP32 | 240MHz | 520KB | 4MB | WiFi/蓝牙，双核 |
| RP2040 | 133MHz | 264KB | 2MB | PIO，低成本 |
| nRF52840 | 64MHz | 256KB | 1MB | 低功耗蓝牙 |

## 功耗对比

| 平台 | 运行功耗 | 睡眠功耗 | 深度睡眠 |
|------|----------|----------|----------|
| STM32F407 | ~100mA | ~2mA | ~10µA |
| ESP32 | ~160mA | ~10mA | ~5µA |
| RP2040 | ~50mA | ~1mA | ~180µA |
| nRF52840 | ~5mA | ~1µA | ~0.3µA |

## 应用场景建议

### STM32 适用场景
- 工业控制系统
- 高性能数据处理
- 复杂的实时系统
- 多外设集成应用

### ESP32 适用场景
- 物联网设备
- 智能家居
- 网络连接应用
- 数据采集系统

### RP2040 适用场景
- 教育项目
- 快速原型开发
- 成本敏感应用
- 创客项目

### nRF52 适用场景
- 可穿戴设备
- 传感器网络
- 低功耗应用
- 蓝牙设备

## 学习路径建议

### 初学者
1. 从RP2040开始，成本低，资料丰富
2. 学习基础GPIO、串口通信
3. 掌握I2C、SPI协议
4. 了解ADC、PWM应用

### 进阶开发者
1. 学习STM32，掌握复杂外设
2. 研究ESP32网络功能
3. 深入nRF52低功耗技术
4. 对比不同平台特性

### 专业开发者
1. 掌握所有平台特性
2. 能够根据需求选择平台
3. 优化性能和功耗
4. 开发跨平台抽象层

## 常见问题

### Q: 如何选择合适的平台？
A: 根据项目需求：
- 需要网络功能 → ESP32
- 要求低功耗 → nRF52
- 高性能处理 → STM32
- 成本敏感 → RP2040

### Q: 如何移植代码到不同平台？
A: 使用embedded-hal抽象层，将平台特定代码封装在trait实现中。

### Q: 调试时遇到问题怎么办？
A: 
1. 检查硬件连接
2. 确认时钟配置
3. 使用串口输出调试
4. 查看平台特定文档

## 扩展资源

### 官方文档
- [STM32 HAL文档](https://docs.rs/stm32f4xx-hal/)
- [ESP-IDF文档](https://docs.espressif.com/projects/esp-idf/)
- [RP2040文档](https://docs.rs/rp2040-hal/)
- [nRF52文档](https://docs.rs/nrf52840-hal/)

### 社区资源
- [Embedded Rust Book](https://doc.rust-lang.org/embedded-book/)
- [Awesome Embedded Rust](https://github.com/rust-embedded/awesome-embedded-rust)
- [Embassy框架](https://embassy.dev/)

### 开发工具
- [probe-rs](https://probe.rs/) - 通用调试工具
- [cargo-embed](https://github.com/probe-rs/cargo-embed) - 嵌入式开发工具
- [defmt](https://defmt.ferrous-systems.com/) - 高效日志框架

## 贡献指南

欢迎为本项目贡献代码和文档：

1. Fork本仓库
2. 创建功能分支
3. 提交更改
4. 发起Pull Request

请确保：
- 代码风格一致
- 添加适当的注释
- 更新相关文档
- 测试在目标平台上运行正常

## 许可证

本项目采用MIT或Apache-2.0双重许可证。