# 定时器和中断处理教程

## 概述

定时器和中断是嵌入式系统的核心组件，它们为系统提供精确的时间控制和高效的事件响应机制。本教程将深入讲解STM32F4系列微控制器的定时器功能和中断系统，以及如何在Rust环境中进行安全、高效的编程实现。

## 学习目标

完成本教程后，你将能够：

- 深入理解定时器的工作原理和应用场景
- 掌握STM32F4定时器的各种工作模式
- 实现精确的PWM波形生成和控制
- 设计高效的中断服务程序
- 处理多个中断源的优先级管理
- 实现实时任务调度和时间管理
- 优化中断响应时间和系统性能
- 调试和分析定时器相关问题

## 教程结构

### 📚 理论基础篇

#### 1. [定时器基础原理](01-timer-basics.md)
- 定时器的基本概念和分类
- 计数器、预分频器、自动重载寄存器
- 定时器时钟源和时钟树
- 定时器的工作模式详解
- 定时精度和误差分析

#### 2. [STM32F4定时器架构](02-stm32-timer-architecture.md)
- STM32F4定时器资源概览
- 通用定时器(TIM2-5, TIM9-14)
- 高级定时器(TIM1, TIM8)
- 基本定时器(TIM6, TIM7)
- 系统定时器(SysTick)
- 看门狗定时器(IWDG, WWDG)

#### 3. [中断系统原理](03-interrupt-system.md)
- ARM Cortex-M4中断控制器(NVIC)
- 中断向量表和中断处理流程
- 中断优先级和抢占机制
- 中断延迟和响应时间
- 临界区保护和原子操作

### 🛠️ 实践应用篇

#### 4. [基础定时器应用](04-basic-timer.md)
- 简单延时和计时功能
- 定时器中断配置
- 毫秒级和微秒级定时
- 多定时器协调工作

#### 5. [PWM波形生成](05-pwm-generation.md)
- PWM基本原理和参数
- 单通道和多通道PWM
- 互补PWM和死区时间
- 频率和占空比动态调节

#### 6. [输入捕获功能](06-input-capture.md)
- 脉冲宽度测量
- 频率测量和周期检测
- 编码器接口应用
- 超声波测距实现

### 🏭 高级应用篇

#### 7. [高级定时器特性](07-advanced-timer.md)
- 三相PWM生成
- 刹车功能和保护机制
- 重复计数器应用
- 定时器同步和级联

#### 8. [实时任务调度](08-real-time-scheduling.md)
- 基于定时器的任务调度
- 时间片轮转调度
- 优先级调度算法
- 任务同步和通信

## 支持的硬件平台

### 主要开发板
- **STM32F407VG Discovery** - 主要演示平台
- **STM32F411CE BlackPill** - 小型化应用
- **STM32F103C8T6 Blue Pill** - 经济型选择
- **STM32F429I Discovery** - 高性能应用

### 定时器资源
- **通用定时器**: TIM2, TIM3, TIM4, TIM5 (32位)
- **通用定时器**: TIM9, TIM10, TIM11, TIM12, TIM13, TIM14 (16位)
- **高级定时器**: TIM1, TIM8 (16位，带互补输出)
- **基本定时器**: TIM6, TIM7 (16位，仅计数)
- **系统定时器**: SysTick (24位)
- **看门狗定时器**: IWDG, WWDG

## 项目结构

```
06-timers-interrupts/
├── README.md                    # 本文档
├── 01-timer-basics.md          # 定时器基础
├── 02-stm32-timer-architecture.md # STM32定时器架构
├── 03-interrupt-system.md      # 中断系统
├── 04-basic-timer.md           # 基础定时器
├── 05-pwm-generation.md        # PWM生成
├── 06-input-capture.md         # 输入捕获
├── 07-advanced-timer.md        # 高级定时器
├── 08-real-time-scheduling.md  # 实时调度
├── examples/                   # 代码示例
│   ├── basic-timer/           # 基础定时器
│   ├── pwm-control/           # PWM控制
│   ├── input-capture/         # 输入捕获
│   ├── interrupt-handler/     # 中断处理
│   └── scheduler/             # 任务调度器
├── projects/                   # 实战项目
│   ├── servo-control/         # 舵机控制
│   ├── motor-driver/          # 电机驱动
│   ├── ultrasonic-sensor/     # 超声波传感器
│   ├── led-matrix/            # LED矩阵
│   └── rtos-kernel/           # 简易RTOS内核
└── docs/                      # 技术文档
    ├── timing-analysis/       # 时序分析
    ├── performance-tuning/    # 性能调优
    └── debugging-guide/       # 调试指南
```

## 硬件准备

### 基础设备
1. **开发板**: STM32F407VG Discovery
2. **示波器**: 用于波形观察和分析
3. **逻辑分析仪**: 数字信号分析
4. **舵机**: SG90或MG996R
5. **LED灯条**: WS2812B或普通LED
6. **超声波模块**: HC-SR04
7. **编码器**: 旋转编码器
8. **电机**: 直流电机或步进电机

### 测试设备
- **万用表**: 电压电流测量
- **频率计**: 精确频率测量
- **信号发生器**: 测试信号生成
- **电源**: 可调直流电源

## 快速开始

### 1. 环境配置
```bash
# 安装必要的工具
cargo install probe-rs --features cli
cargo install cargo-binutils
rustup component add llvm-tools-preview

# 添加目标架构
rustup target add thumbv7em-none-eabihf
```

### 2. 硬件连接
```
STM32F407VG Discovery 引脚分配:
- TIM1_CH1: PA8  (高级定时器通道1)
- TIM2_CH1: PA0  (通用定时器通道1)
- TIM3_CH1: PA6  (通用定时器通道1)
- TIM4_CH1: PB6  (通用定时器通道1)
- LED: PD12-15   (板载LED)
```

### 3. 运行第一个示例
```bash
# 进入基础定时器示例
cd 06-timers-interrupts/examples/basic-timer

# 编译并运行
cargo run

# 观察LED闪烁或串口输出
```

## 实战项目列表

### 🟢 初级项目
1. **LED闪烁控制** - 基础定时器应用
2. **PWM调光器** - 单通道PWM控制
3. **蜂鸣器音调** - 频率可调PWM
4. **按键防抖** - 定时器辅助输入处理

### 🟡 中级项目
1. **舵机控制系统** - 精确角度控制
2. **直流电机调速** - PWM电机驱动
3. **超声波测距仪** - 输入捕获应用
4. **呼吸灯效果** - 渐变PWM控制

### 🔴 高级项目
1. **三相无刷电机控制** - 高级定时器应用
2. **LED矩阵显示** - 多路PWM同步
3. **简易示波器** - 高速采样和显示
4. **实时操作系统内核** - 任务调度实现

## 性能指标

### 定时精度
- **系统时钟**: 168MHz
- **定时器时钟**: 84MHz (APB1), 168MHz (APB2)
- **最小分辨率**: ~6ns (168MHz时钟)
- **最大定时**: 4.29秒 (32位定时器)

### 中断响应
- **中断延迟**: 12-25个时钟周期
- **上下文切换**: 16个寄存器保存/恢复
- **最大中断频率**: ~1MHz (简单ISR)
- **嵌套深度**: 最多256级

### PWM性能
- **最高频率**: 84MHz (理论值)
- **实用频率**: 1kHz-1MHz
- **分辨率**: 16位 (65536级)
- **同步精度**: ±1个时钟周期

## 调试工具

### 软件工具
1. **STM32CubeMX** - 配置生成和参考
2. **STM32CubeIDE** - 调试和分析
3. **probe-rs** - Rust调试工具
4. **RTT Viewer** - 实时日志查看

### 硬件工具
1. **示波器** - 波形分析
   - 带宽: ≥100MHz
   - 采样率: ≥1GSa/s
   - 通道数: ≥2

2. **逻辑分析仪** - 数字信号分析
   - 通道数: ≥8
   - 采样率: ≥100MSa/s
   - 缓存深度: ≥1M样本

### 测量技巧
1. **时序测量**
   - 使用触发功能精确捕获
   - 统计分析功能评估抖动
   - 长时间记录检查稳定性

2. **频率测量**
   - 频率计数器模式
   - FFT分析谐波成分
   - 相位关系分析

## 常见问题解决

### 定时器问题
1. **定时不准确**
   - 检查时钟配置
   - 验证预分频器设置
   - 考虑中断处理延迟

2. **PWM输出异常**
   - 确认GPIO复用配置
   - 检查比较寄存器值
   - 验证输出极性设置

3. **中断丢失**
   - 检查中断优先级
   - 确认中断使能状态
   - 优化ISR执行时间

### 性能问题
1. **中断响应慢**
   - 降低ISR复杂度
   - 使用DMA减少CPU负担
   - 优化中断优先级分配

2. **定时器冲突**
   - 合理分配定时器资源
   - 使用定时器同步功能
   - 考虑软件定时器方案

## 扩展学习

### 相关主题
- **DMA控制器** - 高效数据传输
- **ADC采样** - 定时器触发转换
- **通信协议** - 定时器辅助实现
- **电机控制** - 高级PWM应用

### 高级概念
- **实时系统设计** - 时间约束和调度
- **控制系统理论** - PID控制算法
- **信号处理** - 数字滤波和分析
- **功耗管理** - 低功耗定时器应用

## 参考资源

### 官方文档
- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [STM32F4 Programming Manual](https://www.st.com/resource/en/programming_manual/dm00046982.pdf)
- [Cortex-M4 Technical Reference](https://developer.arm.com/documentation/100166/0001)

### 应用笔记
- [STM32F4 Timer Cookbook](https://www.st.com/resource/en/application_note/dm00042534.pdf)
- [PWM Generation using STM32 Timers](https://www.st.com/resource/en/application_note/dm00236305.pdf)
- [STM32 Interrupt Management](https://www.st.com/resource/en/application_note/dm00164549.pdf)

### 开源项目
- [stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal) - Rust HAL库
- [cortex-m](https://github.com/rust-embedded/cortex-m) - Cortex-M支持
- [rtic](https://github.com/rtic-rs/cortex-m-rtic) - 实时中断驱动并发

### 学习资源
- [Rust嵌入式书籍](https://doc.rust-lang.org/embedded-book/)
- [STM32定时器教程](https://deepbluembedded.com/stm32-timers-tutorial/)
- [ARM Cortex-M中断指南](https://interrupt.memfault.com/blog/arm-cortex-m-exceptions-and-nvic)

## 贡献指南

欢迎为本教程贡献代码、文档或建议：

1. **报告问题**: 在GitHub Issues中报告bug或提出改进建议
2. **提交代码**: 通过Pull Request提交代码改进
3. **完善文档**: 帮助改进教程内容和示例
4. **分享经验**: 在社区中分享使用经验和最佳实践

## 版本历史

- **v1.0.0** - 初始版本，包含基础定时器和中断教程
- **v1.1.0** - 添加PWM生成和输入捕获示例
- **v1.2.0** - 增加高级定时器特性和应用
- **v1.3.0** - 添加实时调度和性能优化内容

---

*本教程是Rust嵌入式编程完整教程的一部分。更多内容请参考主教程目录。*

**上一章**: [串口通信](../05-serial-communication/README.md)  
**下一章**: [ADC/DAC数据采集](../07-adc-dac/README.md)