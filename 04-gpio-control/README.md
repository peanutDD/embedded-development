# GPIO控制项目集合

这是一个全面的嵌入式GPIO控制项目集合，包含从基础到高级的各种GPIO应用示例。项目使用Rust语言开发，支持多种嵌入式平台。

## 🎯 项目概述

本项目旨在提供完整的GPIO控制学习和开发资源，包括：
- 📚 **7个渐进式项目**: 从基础LED控制到复杂的中断系统
- 🔌 **4个平台支持**: STM32F4、ESP32、RP2040、nRF52
- 📖 **完整技术文档**: 电路图、教程、元器件手册
- 🛠️ **实用工具**: 调试、测试、性能分析

## 📁 项目结构

```
04-gpio-control/
├── README.md                    # 项目主文档
├── basic-led/                   # 基础LED闪烁控制
├── button-led/                  # 按钮控制LED
├── pwm-breathing/               # PWM呼吸灯效果
├── traffic-light/               # 交通灯系统
├── led-matrix/                  # LED矩阵显示
├── servo-control/               # 舵机控制
├── gpio-interrupt/              # GPIO中断系统
├── examples/                    # 平台特定示例
│   └── platforms/               # 各平台适配代码
│       ├── stm32f4/            # STM32F4平台
│       ├── esp32/              # ESP32平台
│       ├── rp2040/             # RP2040平台
│       └── nrf52/              # nRF52平台
└── docs/                       # 完整技术文档
    ├── schematics/             # 电路图
    ├── tutorials/              # 教程文档
    └── reference/              # 参考资料
```

## 🚀 快速开始

### 环境准备

1. **安装Rust工具链**:
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add thumbv7em-none-eabihf  # STM32F4
rustup target add xtensa-esp32-none-elf   # ESP32
rustup target add thumbv6m-none-eabi      # RP2040
rustup target add thumbv7em-none-eabihf   # nRF52
```

2. **安装调试工具**:
```bash
cargo install probe-run
cargo install cargo-embed
cargo install cargo-flash
```

### 运行第一个项目

```bash
# 进入基础LED项目
cd basic-led

# 编译项目
cargo build --release

# 烧录到开发板 (以STM32F4为例)
cargo run --release
```

## 📚 项目详解

### 1. [基础LED控制](basic-led/) ⭐☆☆☆☆
**功能**: 控制单个LED以1Hz频率闪烁
**学习目标**: GPIO输出控制、延时函数使用
**硬件需求**: LED + 限流电阻
**代码特点**:
- 简单的GPIO输出控制
- 基础的延时循环
- 适合初学者入门

### 2. [按钮控制LED](button-led/) ⭐⭐☆☆☆
**功能**: 按钮按下时LED亮，松开时LED灭
**学习目标**: GPIO输入检测、上拉电阻使用
**硬件需求**: LED + 按钮 + 上拉电阻
**代码特点**:
- GPIO输入模式配置
- 按钮状态检测
- 软件去抖动处理

### 3. [PWM呼吸灯](pwm-breathing/) ⭐⭐⭐☆☆
**功能**: LED亮度呈正弦波变化，实现呼吸效果
**学习目标**: PWM原理、定时器配置、数学函数应用
**硬件需求**: LED + 限流电阻 (可选MOSFET驱动)
**代码特点**:
- 定时器PWM模式配置
- 正弦波亮度调制
- 平滑过渡算法

### 4. [交通灯系统](traffic-light/) ⭐⭐⭐⭐☆
**功能**: 完整的交通灯控制系统，支持行人过街
**学习目标**: 状态机设计、多GPIO协调控制
**硬件需求**: 6个LED (红黄绿×2) + 按钮
**代码特点**:
- 有限状态机实现
- 定时器中断处理
- 多路GPIO同步控制

### 5. [LED矩阵显示](led-matrix/) ⭐⭐⭐⭐☆
**功能**: 8×8 LED矩阵显示图案和动画
**学习目标**: 矩阵扫描、时分复用、图形显示
**硬件需求**: 8×8 LED矩阵 + 限流电阻
**代码特点**:
- 行列扫描算法
- 图案数据存储
- 动画效果实现

### 6. [舵机控制](servo-control/) ⭐⭐⭐☆☆
**功能**: 精确控制舵机角度和速度
**学习目标**: PWM精确控制、伺服系统原理
**硬件需求**: 标准舵机 (SG90等)
**代码特点**:
- 精确PWM脉宽控制
- 角度到脉宽转换
- 平滑运动控制

### 7. [GPIO中断系统](gpio-interrupt/) ⭐⭐⭐⭐⭐
**功能**: 高级GPIO中断处理，支持多种触发模式
**学习目标**: 中断系统、实时响应、系统优化
**硬件需求**: 按钮、旋转编码器、运动传感器
**代码特点**:
- 外部中断配置
- 中断优先级管理
- 实时事件处理
- 性能统计分析

## 🔌 平台支持

### STM32F4系列
- **开发板**: STM32F4 Discovery, Nucleo-F401RE
- **特色功能**: 丰富的定时器资源、高性能ARM Cortex-M4
- **适用场景**: 工业控制、实时系统

### ESP32系列
- **开发板**: ESP32-DevKitC, ESP32-WROOM-32
- **特色功能**: WiFi/蓝牙连接、双核处理器
- **适用场景**: IoT应用、无线控制

### RP2040 (Raspberry Pi Pico)
- **开发板**: Raspberry Pi Pico, Pico W
- **特色功能**: PIO可编程IO、双核ARM Cortex-M0+
- **适用场景**: 创客项目、教育应用

### nRF52系列
- **开发板**: nRF52840-DK, nRF52832-DK
- **特色功能**: 低功耗蓝牙、高级电源管理
- **适用场景**: 可穿戴设备、传感器网络

## 📖 技术文档

### 🔌 [电路图](docs/schematics/)
- [基础LED控制电路](docs/schematics/basic-led-circuit.svg)
- [按钮控制LED电路](docs/schematics/button-led-circuit.svg)
- [PWM呼吸灯电路](docs/schematics/pwm-breathing-circuit.svg)
- [交通灯系统电路](docs/schematics/traffic-light-circuit.svg)

### 📚 [教程文档](docs/tutorials/)
- [GPIO基础教程](docs/tutorials/gpio-basics.md) - 全面的GPIO入门指南
- [PWM控制教程](docs/tutorials/pwm-control.md) - 深入的PWM技术讲解

### 📖 [参考资料](docs/reference/)
- [元器件数据手册](docs/reference/component-datasheets.md) - 常用元器件参数查询

## 🛠️ 开发工具

### 硬件工具
- **万用表**: 基础电压电流测量
- **示波器**: PWM波形观察 (推荐)
- **逻辑分析仪**: 数字信号分析 (可选)
- **面包板**: 电路搭建和测试

### 软件工具
- **开发环境**: VS Code + rust-analyzer
- **调试工具**: probe-run, cargo-embed
- **仿真软件**: Proteus, LTSpice (电路仿真)
- **版本控制**: Git

## 📊 性能特性

### 实时性能
- **中断响应时间**: <10μs (STM32F4 @ 168MHz)
- **GPIO切换频率**: >1MHz (直接寄存器操作)
- **PWM分辨率**: 16位 (65536级)
- **定时器精度**: 1μs (高速定时器)

### 资源占用
- **Flash使用**: 8KB - 32KB (根据项目复杂度)
- **RAM使用**: 2KB - 8KB (包含栈空间)
- **GPIO需求**: 1-16个引脚 (根据项目需求)
- **定时器需求**: 1-3个定时器

### 功耗分析
- **运行模式**: 20-50mA (根据负载)
- **睡眠模式**: <1mA (支持低功耗模式)
- **待机模式**: <10μA (深度睡眠)

## 🔧 故障排除

### 常见问题
1. **LED不亮**: 检查电路连接、限流电阻值、GPIO配置
2. **按钮无响应**: 检查上拉电阻、去抖动设置、中断配置
3. **PWM频率不对**: 检查时钟配置、预分频值、重载值
4. **编译错误**: 检查目标平台、依赖版本、特性配置

### 调试技巧
```rust
// 使用defmt进行调试输出
defmt::info!("GPIO state: {}", gpio_state);

// 使用probe-run查看实时日志
cargo run --release

// 使用GDB进行断点调试
cargo embed --release
```

## 🤝 贡献指南

我们欢迎各种形式的贡献：

### 代码贡献
- 🐛 **Bug修复**: 发现并修复代码问题
- ✨ **新功能**: 添加新的GPIO控制示例
- 🔧 **优化改进**: 提升代码性能和可读性
- 📱 **平台支持**: 适配更多嵌入式平台

### 文档贡献
- 📝 **教程编写**: 补充更多技术教程
- 🔌 **电路设计**: 提供新的电路图和说明
- 🌐 **翻译工作**: 支持多语言版本
- 📖 **示例补充**: 添加更多应用示例

### 测试贡献
- 🧪 **硬件测试**: 在不同平台上测试代码
- 📊 **性能测试**: 提供性能基准数据
- 🔍 **兼容性测试**: 验证跨平台兼容性

## 📄 许可证

本项目采用 [MIT License](LICENSE) 开源许可证。

## 🙏 致谢

感谢以下开源项目和社区的支持：
- [embedded-hal](https://github.com/rust-embedded/embedded-hal) - Rust嵌入式HAL抽象
- [cortex-m](https://github.com/rust-embedded/cortex-m) - ARM Cortex-M支持
- [stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal) - STM32F4 HAL实现
- [esp-idf](https://github.com/espressif/esp-idf) - ESP32开发框架
- [rp-hal](https://github.com/rp-rs/rp-hal) - RP2040 HAL实现

## 📞 联系我们

- **GitHub Issues**: [提交问题和建议](https://github.com/your-repo/issues)
- **讨论区**: [技术讨论和交流](https://github.com/your-repo/discussions)
- **邮件**: your-email@example.com

---

**开始你的嵌入式GPIO控制之旅吧！** 🚀

从简单的LED闪烁开始，逐步掌握复杂的中断系统，成为嵌入式开发专家！