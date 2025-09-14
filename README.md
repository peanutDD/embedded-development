# 🦀 嵌入式 Rust 开发教程

[![Rust](https://img.shields.io/badge/rust-1.70+-orange.svg)](https://www.rust-lang.org)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](#)

本项目提供了一个完整的嵌入式 Rust 开发学习路径，从基础语法到工业级应用。通过循序渐进的教程和实战项目，帮助开发者掌握现代嵌入式系统开发技能。

## 🎯 学习目标

- 掌握 Rust 在嵌入式系统中的应用
- 理解嵌入式系统的核心概念和设计模式
- 学会使用现代开发工具和调试技术
- 具备开发工业级嵌入式产品的能力
- 了解实时操作系统(RTOS)的集成和应用

## 📚 教程结构

### 基础篇 (Foundation)
- **[01-rust-basics](01-rust-basics/)** - Rust 基础语法和概念
- **[02-environment-setup](02-environment-setup/)** - 开发环境配置和工具链
- **[03-embedded-concepts](03-embedded-concepts/)** - 嵌入式系统核心概念

### 外设控制篇 (Peripheral Control)
- **[04-gpio-control](04-gpio-control/)** - GPIO 控制和数字I/O
- **[05-serial-communication](05-serial-communication/)** - 串口通信(UART/USART)
- **[06-timers-interrupts](06-timers-interrupts/)** - 定时器和中断处理
- **[07-adc-dac](07-adc-dac/)** - 模数转换和数模转换
- **[08-i2c-spi](08-i2c-spi/)** - I2C/SPI 总线通信

### 高级应用篇 (Advanced Applications)
- **[08-rtos-integration](08-rtos-integration/)** - 实时操作系统集成
- **[09-industrial-projects](09-industrial-projects/)** - 工业级项目案例
- **[10-sensor-system](10-sensor-system/)** - 传感器系统设计
- **[11-industrial-controller](11-industrial-controller/)** - 工业控制器开发
- **[12-wireless-communication](12-wireless-communication/)** - 无线通信技术

## 🚀 快速开始

### 环境要求

- **Rust**: 1.70.0 或更高版本
- **目标平台**: STM32F4xx 系列微控制器
- **开发工具**: probe-rs, cargo-embed, RTT
- **硬件**: STM32F4 Discovery 或兼容开发板

### 安装步骤

1. **安装 Rust 工具链**
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   rustup target add thumbv7em-none-eabihf
   ```

2. **安装嵌入式开发工具**
   ```bash
   cargo install probe-rs --features cli
   cargo install cargo-embed
   cargo install cargo-binutils
   ```

3. **克隆项目**
   ```bash
   git clone https://github.com/example/embedded-development.git
   cd embedded-development
   ```

4. **运行第一个示例**
   ```bash
   cd 04-gpio-control/examples/basic-led
   cargo embed --release
   ```

## 🛠️ 项目特色

### 📖 完整的学习路径
- 从零基础到工业应用的完整教程
- 每个章节都有理论讲解、代码示例和实战项目
- 循序渐进的难度设计，适合不同水平的学习者

### 🔧 实用的代码示例
- 所有代码都经过实际硬件测试
- 包含详细的注释和文档
- 提供多种实现方案和最佳实践

### 🏭 工业级项目案例
- **智能制造系统**: 多设备协调控制和实时数据采集
- **IoT边缘网关**: 多协议数据采集与云端同步
- **传感器网络**: 分布式传感器数据收集和处理

### 🔄 现代开发工具
- 使用最新的 Rust 嵌入式生态系统
- 集成 RTIC 实时框架
- 支持 RTT 实时调试和日志输出

## 📊 技术栈

| 类别 | 技术 | 描述 |
|------|------|------|
| **语言** | Rust | 系统级编程语言，内存安全 |
| **框架** | RTIC | 实时中断驱动并发框架 |
| **RTOS** | FreeRTOS | 实时操作系统集成 |
| **HAL** | stm32f4xx-hal | STM32F4 硬件抽象层 |
| **调试** | probe-rs, RTT | 现代调试和日志工具 |
| **通信** | UART, I2C, SPI, CAN | 多种通信协议支持 |
| **网络** | WiFi, LoRaWAN, Ethernet | 无线和有线网络连接 |

## 🎓 学习建议

### 初学者路径
1. 先学习 [Rust 基础语法](01-rust-basics/)
2. 配置好 [开发环境](02-environment-setup/)
3. 理解 [嵌入式概念](03-embedded-concepts/)
4. 从简单的 [GPIO 控制](04-gpio-control/) 开始实践

### 进阶开发者路径
1. 直接从感兴趣的外设章节开始
2. 学习 [RTOS 集成](08-rtos-integration/)
3. 参考 [工业项目案例](09-industrial-projects/)
4. 开发自己的项目

### 项目实战建议
- 每完成一个章节，尝试修改示例代码
- 结合实际硬件进行测试
- 参与开源项目贡献代码
- 分享学习心得和项目经验

## 🔗 相关资源

### 官方文档
- [Rust 嵌入式工作组](https://github.com/rust-embedded)
- [嵌入式 Rust 之书](https://docs.rust-embedded.org/book/)
- [RTIC 官方文档](https://rtic.rs/)

### 硬件资源
- [STM32F4 Discovery 用户手册](https://www.st.com/resource/en/user_manual/dm00039084.pdf)
- [STM32F4 参考手册](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [Cortex-M4 技术参考手册](https://developer.arm.com/documentation/100166/0001)

### 社区支持
- [Rust 嵌入式社区](https://matrix.to/#/#rust-embedded:matrix.org)
- [STM32 Rust 社区](https://github.com/stm32-rs)
- [嵌入式 Rust 论坛](https://users.rust-lang.org/c/embedded/13)

## 🤝 贡献指南

我们欢迎各种形式的贡献！

### 如何贡献
1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add some amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 开启 Pull Request

### 贡献类型
- 🐛 修复 bug
- ✨ 添加新功能
- 📚 改进文档
- 🎨 优化代码结构
- 🔧 添加测试用例
- 🌐 翻译文档

### 代码规范
- 遵循 Rust 官方代码风格
- 添加适当的注释和文档
- 确保代码通过所有测试
- 更新相关文档

## 📈 项目统计

- **教程章节**: 12 个主要章节
- **代码示例**: 50+ 个实用示例
- **实战项目**: 20+ 个完整项目
- **支持硬件**: STM32F4 系列
- **代码行数**: 10,000+ 行

## 🏆 成就系统

完成不同阶段的学习可以获得相应成就：

- 🥉 **入门者**: 完成基础篇所有章节
- 🥈 **实践者**: 完成外设控制篇所有项目
- 🥇 **专家**: 完成高级应用篇所有案例
- 🏆 **大师**: 贡献原创项目或重要改进

## 📞 联系我们

- **项目主页**: https://github.com/example/embedded-development
- **问题反馈**: [GitHub Issues](https://github.com/example/embedded-development/issues)
- **讨论交流**: [GitHub Discussions](https://github.com/example/embedded-development/discussions)
- **邮箱**: embedded-rust@example.com

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 🙏 致谢

感谢以下项目和社区的支持：

- [Rust 嵌入式工作组](https://github.com/rust-embedded)
- [STM32 Rust 社区](https://github.com/stm32-rs)
- [RTIC 框架](https://github.com/rtic-rs/cortex-m-rtic)
- [probe-rs 项目](https://github.com/probe-rs/probe-rs)
- 所有贡献者和用户

---

⭐ 如果这个项目对你有帮助，请给我们一个 Star！

🚀 开始你的嵌入式 Rust 开发之旅吧！