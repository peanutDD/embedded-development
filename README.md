# 🦀 Rust嵌入式开发完整教程

[![Rust](https://img.shields.io/badge/rust-1.70+-orange.svg)](https://www.rust-lang.org)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](scripts/verify_all.sh)

一个全面、系统的Rust嵌入式开发教程，从基础概念到高级应用，涵盖现代嵌入式系统开发的各个方面。

## 📚 教程概述

本教程分为三个主要部分，共19个章节，每个章节都包含理论讲解、实践项目和完整的示例代码：

### 🌱 基础篇 (第1-7章)
掌握Rust嵌入式开发的基础知识和核心技能

### 🚀 进阶篇 (第8-14章)  
深入学习高级通信协议、系统管理和优化技术

### 💼 专业开发篇 (第15-19章)
探索前沿技术，包括RTOS、IoT、AI和系统集成

## 🎯 学习目标

- 🔧 掌握Rust语言在嵌入式系统中的应用
- 🖥️ 理解硬件抽象层(HAL)的设计和实现
- 📡 学会各种通信协议的使用和优化
- ⚡ 掌握实时系统和多任务处理技术
- 🌐 了解现代IoT和工业4.0应用开发
- 🤖 探索嵌入式AI和机器学习应用

## 📖 章节目录

### 🌱 基础篇

| 章节 | 标题 | 主要内容 | 项目 |
|------|------|----------|------|
| [第1章](01-rust-basics/) | Rust基础 | 语法、所有权、生命周期 | 基础语法示例 |
| [第2章](02-environment-setup/) | 环境搭建 | 工具链、调试器、开发板 | 环境配置 |
| [第3章](03-embedded-concepts/) | 嵌入式概念 | HAL设计、寄存器操作 | 硬件抽象层 |
| [第4章](04-gpio-control/) | GPIO控制 | 数字I/O、高级GPIO | LED控制系统 |
| [第5章](05-serial-communication/) | 串口通信 | UART协议、数据传输 | 数据记录器 |
| [第6章](06-timers-interrupts/) | 定时器与中断 | 定时器配置、中断处理 | 定时任务系统 |
| [第7章](07-adc-dac-interrupts/) | ADC/DAC与中断 | 模拟信号、中断驱动 | 数字电压表 |

### 🚀 进阶篇

| 章节 | 标题 | 主要内容 | 项目 |
|------|------|----------|------|
| [第8章](08-communication-protocols/) | 通信协议 | I2C/SPI/数字IO | 传感器集线器 |
| [第9章](09-rtos-basics/) | RTOS基础 | 实时系统、任务调度 | 多任务系统 |
| [第10章](10-sensors-actuators/) | 传感器执行器 | 传感器集成、数据处理 | 环境监测站 |
| [第11章](11-power-management/) | 电源管理 | 低功耗设计、电源优化 | 节能控制器 |
| [第12章](12-industrial-projects/) | 工业项目 | 工业控制、自动化 | 工业控制系统 |
| [第13章](13-security-encryption/) | 安全加密 | 加密算法、安全通信 | 安全通信模块 |
| [第14章](14-wireless-communication/) | 无线通信 | WiFi、蓝牙、LoRa | 无线传感网络 |

### 💼 专业开发篇

| 章节 | 标题 | 主要内容 | 项目 |
|------|------|----------|------|
| [第15章](15-rtos-advanced/) | 高级RTOS | 高级RTOS特性、系统优化 | 复杂实时系统 |
| [第16章](16-performance-optimization/) | 性能优化 | 代码优化、内存管理 | 高性能控制器 |
| [第17章](17-embedded-ai/) | 嵌入式AI | 嵌入式AI、TinyML | 智能识别系统 |
| [第18章](18-testing-debugging/) | 测试与调试 | 单元测试、集成测试 | 测试框架 |
| [第19章](19-system-integration/) | 系统集成 | 系统集成、部署策略 | 完整产品系统 |

## 🛠️ 项目结构

```
embedded-development/
├── 📁 01-rust-basics/           # Rust基础语法和概念
├── 📁 02-environment-setup/     # 开发环境配置
├── 📁 03-embedded-concepts/     # 嵌入式基础概念
├── 📁 04-gpio-control/          # GPIO控制和高级GPIO
├── 📁 05-serial-communication/  # 串口通信
├── 📁 06-timers-interrupts/     # 定时器和中断
├── 📁 07-adc-dac-interrupts/    # ADC/DAC和中断驱动
├── 📁 08-communication-protocols/ # 通信协议(I2C/SPI/数字IO)
├── 📁 09-rtos-basics/           # RTOS基础
├── 📁 10-sensors-actuators/     # 传感器和执行器系统
├── 📁 11-power-management/      # 电源管理
├── 📁 12-industrial-projects/   # 工业项目应用
├── 📁 13-security-encryption/   # 安全和加密
├── 📁 14-wireless-communication/ # 无线通信
├── 📁 15-rtos-advanced/         # 高级RTOS特性
├── 📁 16-performance-optimization/ # 性能优化
├── 📁 17-embedded-ai/           # 嵌入式AI和机器学习
├── 📁 18-testing-debugging/     # 测试和调试
├── 📁 19-system-integration/    # 系统集成和部署
├── 📁 docs/                     # 文档和指南
├── 📁 examples/                 # 跨平台示例
├── 📁 scripts/                  # 构建和验证脚本
├── 📁 tests/                    # 集成测试
├── 📄 Cargo.toml               # 工作空间配置
├── 📄 README.md                # 项目说明
└── 📄 LICENSE                  # 许可证
```

## 🚀 快速开始

### 环境要求

- Rust 1.70+ 
- 目标硬件：STM32F4系列开发板
- 调试器：ST-Link V2/V3
- 操作系统：Windows/Linux/macOS

### 安装步骤

1. **克隆仓库**
   ```bash
   git clone https://github.com/your-username/embedded-development.git
   cd embedded-development
   ```

2. **安装Rust工具链**
   ```bash
   rustup target add thumbv7em-none-eabihf
   cargo install probe-run
   ```

3. **验证环境**
   ```bash
   ./scripts/verify_all.sh
   ```

4. **运行第一个项目**
   ```bash
   cd 04-gpio-control/projects/basic-led
   cargo run
   ```

## 📋 学习路径

### 🎯 初学者路径 (4-6周)
1. 完成第1-3章的理论学习
2. 实践第4-7章的基础项目
3. 掌握基本的硬件控制技能

### 🚀 进阶路径 (6-8周)
1. 深入学习第8-14章的高级主题
2. 完成复杂的通信和控制项目
3. 理解系统级设计原理

### 💼 专家路径 (8-12周)
1. 探索第15-19章的前沿技术
2. 开发完整的产品级项目
3. 掌握工业级开发技能

## 🤝 贡献指南

我们欢迎各种形式的贡献！请查看 [CONTRIBUTING.md](docs/CONTRIBUTING.md) 了解详细信息。

### 贡献方式
- 🐛 报告问题和错误
- 💡 提出新功能建议
- 📝 改进文档和教程
- 🔧 提交代码修复和优化
- 🎯 分享学习心得和项目经验

## 📞 支持与反馈

- 📧 邮箱：support@embedded-rust.dev
- 💬 讨论区：[GitHub Discussions](https://github.com/your-username/embedded-development/discussions)
- 🐛 问题报告：[GitHub Issues](https://github.com/your-username/embedded-development/issues)
- 📚 文档：[在线文档](https://embedded-rust.dev/docs)

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 🙏 致谢

感谢所有为这个项目做出贡献的开发者和社区成员！

---

⭐ 如果这个项目对你有帮助，请给我们一个星标！

🚀 开始你的Rust嵌入式开发之旅吧！