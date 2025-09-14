# 串口通信教程

## 概述

串口通信是嵌入式系统中最基础和重要的通信方式之一。本教程将深入讲解如何在Rust嵌入式环境中实现高效、可靠的串口通信，涵盖从基础概念到工业级应用的完整知识体系。

## 学习目标

完成本教程后，你将能够：

- 深入理解UART/USART通信原理和协议
- 掌握Rust嵌入式HAL库的串口API使用
- 实现可靠的数据传输和错误处理机制
- 设计高效的通信协议和数据格式
- 处理中断驱动的异步通信
- 实现工业级的串口通信应用
- 掌握调试和性能优化技巧

## 教程结构

### 📚 理论基础篇

#### 1. [UART/USART基础原理](01-uart-basics.md)
- UART vs USART区别和特点
- 串口通信协议详解
- 波特率、数据位、停止位、校验位
- 流控制和握手机制
- 常见通信问题和解决方案

#### 2. [STM32串口硬件特性](02-stm32-uart-hardware.md)
- STM32F4系列UART/USART外设
- 引脚复用和GPIO配置
- 时钟配置和波特率计算
- DMA传输支持
- 中断系统集成

#### 3. [Rust HAL库串口API](03-rust-hal-serial.md)
- embedded-hal串口特征
- stm32f4xx-hal串口实现
- 阻塞式和非阻塞式API
- 错误处理和类型安全
- 配置选项和参数设置

### 🛠️ 实践应用篇

#### 4. [基础串口收发](04-basic-serial.md)
- 简单字符串发送和接收
- 格式化输出和调试信息
- 回显服务器实现
- 数据缓冲和队列管理

#### 5. [中断驱动通信](05-interrupt-driven.md)
- UART中断配置和处理
- 异步数据收发
- 环形缓冲区实现
- 中断优先级和嵌套

#### 6. [DMA高效传输](06-dma-transfer.md)
- DMA基础概念和配置
- 串口DMA传输设置
- 双缓冲和循环传输
- 传输完成中断处理

### 🏭 工业级应用篇

#### 7. [通信协议设计](07-protocol-design.md)
- 自定义通信协议
- 数据帧格式设计
- 校验和错误检测
- 命令响应机制

#### 8. [Modbus协议实现](08-modbus-implementation.md)
- Modbus RTU协议详解
- 主从设备实现
- 功能码和数据处理
- 工业现场应用

## 支持的硬件平台

### 主要开发板
- **STM32F407VG Discovery** - 主要演示平台
- **STM32F411CE BlackPill** - 小型化应用
- **STM32F103C8T6 Blue Pill** - 经济型选择
- **ESP32-C3** - 无线通信扩展

### 串口规格
- **UART数量**: 2-6个（根据芯片型号）
- **最大波特率**: 10.5Mbps (STM32F4)
- **数据位**: 7, 8, 9位
- **停止位**: 0.5, 1, 1.5, 2位
- **校验**: 无校验、奇校验、偶校验
- **流控制**: RTS/CTS硬件流控

## 项目结构

```
05-serial-communication/
├── README.md                    # 本文档
├── 01-uart-basics.md           # UART基础理论
├── 02-stm32-uart-hardware.md   # STM32硬件特性
├── 03-rust-hal-serial.md       # Rust HAL API
├── 04-basic-serial.md          # 基础串口应用
├── 05-interrupt-driven.md      # 中断驱动通信
├── 06-dma-transfer.md          # DMA传输
├── 07-protocol-design.md       # 协议设计
├── 08-modbus-implementation.md # Modbus实现
├── examples/                   # 代码示例
│   ├── basic-echo/            # 基础回显服务
│   ├── interrupt-uart/        # 中断驱动UART
│   ├── dma-transfer/          # DMA传输示例
│   └── protocol-parser/       # 协议解析器
├── projects/                   # 实战项目
│   ├── serial-monitor/        # 串口监视器
│   ├── modbus-slave/          # Modbus从设备
│   ├── data-logger/           # 数据记录器
│   └── wireless-bridge/       # 无线串口桥
└── docs/                      # 技术文档
    ├── protocol-specs/        # 协议规范
    ├── hardware-guides/       # 硬件指南
    └── troubleshooting/       # 故障排除
```

## 硬件准备

### 基础设备
1. **开发板**: STM32F407VG Discovery
2. **USB转串口模块**: CP2102或FT232RL
3. **杜邦线**: 公对母连接线
4. **面包板**: 用于电路连接
5. **逻辑分析仪**: 可选，用于信号分析

### 连接示例
```
STM32F407VG    USB转串口模块
-----------    -------------
PA9 (TX1)  →   RX
PA10 (RX1) ←   TX
GND        →   GND
```

## 快速开始

### 1. 环境配置
```bash
# 安装必要的工具
cargo install probe-rs --features cli
cargo install serialport-rs

# 添加目标架构
rustup target add thumbv7em-none-eabihf
```

### 2. 硬件连接
1. 按照上述连接图连接硬件
2. 确保USB转串口驱动已安装
3. 记录串口设备名称（如/dev/ttyUSB0或COM3）

### 3. 运行第一个示例
```bash
# 进入基础回显示例目录
cd 05-serial-communication/examples/basic-echo

# 编译并烧录到开发板
cargo run

# 在另一个终端打开串口监视器
screen /dev/ttyUSB0 115200
# 或使用minicom
minicom -D /dev/ttyUSB0 -b 115200
```

### 4. 验证通信
在串口终端中输入任意字符，应该能看到开发板的回显响应。

## 实战项目列表

### 🟢 初级项目
1. **串口回显服务器** - 基础收发功能
2. **简单命令解析器** - 文本命令处理
3. **传感器数据上报** - 定时数据发送
4. **LED远程控制** - 串口控制GPIO

### 🟡 中级项目
1. **数据记录器** - 传感器数据采集和存储
2. **串口调试助手** - 多功能调试工具
3. **简易示波器** - ADC数据可视化
4. **无线串口桥** - WiFi/蓝牙透传

### 🔴 高级项目
1. **Modbus RTU从站** - 工业通信协议
2. **多设备通信网关** - 协议转换和路由
3. **实时数据采集系统** - 高速数据处理
4. **分布式传感器网络** - 多节点通信

## 性能指标

### 通信性能
- **最大波特率**: 10.5Mbps
- **数据吞吐量**: 1.3MB/s (理论值)
- **延迟**: < 1ms (115200波特率)
- **错误率**: < 0.01% (正常条件下)

### 资源占用
- **Flash使用**: 8-32KB (根据功能复杂度)
- **RAM使用**: 1-8KB (包含缓冲区)
- **CPU占用**: < 5% (中断驱动模式)

## 调试工具

### 软件工具
1. **串口终端**
   - PuTTY (Windows)
   - screen/minicom (Linux/macOS)
   - CoolTerm (跨平台)

2. **协议分析**
   - Wireshark (串口插件)
   - Serial Port Monitor
   - 自定义分析工具

3. **逻辑分析**
   - Saleae Logic
   - PulseView + 廉价逻辑分析仪

### 调试技巧
1. **信号完整性检查**
   - 使用示波器检查信号质量
   - 验证电平标准和时序

2. **协议验证**
   - 抓取和分析数据帧
   - 验证校验和计算

3. **性能测试**
   - 吞吐量测试
   - 延迟测量
   - 错误率统计

## 常见问题解决

### 通信问题
1. **无法收发数据**
   - 检查硬件连接
   - 验证波特率设置
   - 确认GPIO配置

2. **数据乱码**
   - 检查波特率匹配
   - 验证数据位设置
   - 检查时钟配置

3. **数据丢失**
   - 增加缓冲区大小
   - 使用流控制
   - 优化中断处理

### 性能问题
1. **传输速度慢**
   - 提高波特率
   - 使用DMA传输
   - 优化数据格式

2. **CPU占用高**
   - 使用中断驱动
   - 减少轮询频率
   - 优化算法复杂度

## 扩展学习

### 相关协议
- **SPI通信** - 高速同步串行接口
- **I2C通信** - 两线制串行总线
- **CAN总线** - 汽车和工业网络
- **RS485** - 差分信号长距离传输

### 高级主题
- **实时操作系统集成** - FreeRTOS/RTIC
- **网络协议栈** - TCP/IP over 串口
- **加密通信** - 数据安全传输
- **故障诊断** - 自动错误检测和恢复

## 参考资源

### 官方文档
- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [embedded-hal Documentation](https://docs.rs/embedded-hal/)
- [stm32f4xx-hal Documentation](https://docs.rs/stm32f4xx-hal/)

### 标准规范
- [RS-232 Standard](https://www.itu.int/rec/T-REC-V.24/)
- [Modbus Protocol Specification](https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)
- [UART Design Guidelines](https://www.ti.com/lit/an/slaa522/slaa522.pdf)

### 开源项目
- [serialport-rs](https://github.com/serialport/serialport-rs) - Rust串口库
- [tokio-serial](https://github.com/berkowski/tokio-serial) - 异步串口
- [modbus-rs](https://github.com/slowtec/tokio-modbus) - Modbus协议实现

### 社区资源
- [Rust嵌入式工作组](https://github.com/rust-embedded/wg)
- [STM32 Rust社区](https://github.com/stm32-rs)
- [嵌入式Rust论坛](https://users.rust-lang.org/c/embedded/)

## 贡献指南

欢迎为本教程贡献代码、文档或建议：

1. **报告问题**: 在GitHub Issues中报告bug或提出改进建议
2. **提交代码**: 通过Pull Request提交代码改进
3. **完善文档**: 帮助改进教程内容和示例
4. **分享经验**: 在社区中分享使用经验和最佳实践

## 版本历史

- **v1.0.0** - 初始版本，包含基础教程和示例
- **v1.1.0** - 添加DMA传输和中断驱动示例
- **v1.2.0** - 增加Modbus协议实现
- **v1.3.0** - 添加工业级应用案例

---

*本教程是Rust嵌入式编程完整教程的一部分。更多内容请参考主教程目录。*

**下一章**: [定时器和中断处理](../06-timers-interrupts/README.md)