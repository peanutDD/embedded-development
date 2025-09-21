# 第6章：定时器与中断系统

## 概述

定时器与中断系统是嵌入式系统的时间基准和事件响应核心，决定了系统的实时性能和响应特性。本章深入探讨STM32F4系列微控制器的定时器架构、中断控制器(NVIC)机制，以及Rust语言在嵌入式环境下的安全并发编程模型。

通过硬件定时器的精确时基生成、多级中断优先级管理、零成本抽象的中断处理程序设计，我们将构建高性能、低延迟的实时系统。涵盖从基础的PWM波形生成到复杂的多任务调度，从单一定时器应用到定时器级联同步，全面掌握时间驱动的嵌入式系统设计。

核心技术包括：ARM Cortex-M4中断向量表、定时器时钟域配置、DMA触发的零拷贝数据传输、临界区保护的原子操作、以及基于定时器的软实时调度算法。

## 学习目标

### 硬件层面理解
- **定时器架构**：STM32F4定时器分类(基本/通用/高级)、时钟树配置、预分频计算
- **中断控制器**：NVIC优先级分组、中断向量表、Pending/Active状态管理
- **时序特性**：中断延迟测量、抖动分析、实时性能评估
- **硬件同步**：定时器级联、主从模式、外部触发源配置

### 软件抽象设计
- **类型安全**：编译时定时器配置验证、中断处理程序类型检查
- **零成本抽象**：HAL层封装、寄存器级优化、内联汇编集成
- **并发安全**：临界区保护、原子操作、中断安全的数据结构
- **资源管理**：RAII模式的定时器生命周期、中断资源分配

### 实时系统实现
- **任务调度**：基于定时器的抢占式调度、时间片轮转、优先级继承
- **时间管理**：高精度延时、超时检测、时间戳生成
- **事件驱动**：异步事件处理、回调函数管理、状态机驱动
- **性能优化**：中断响应时间优化、上下文切换开销分析

### 工程实践能力
- **调试技术**：定时器波形分析、中断统计、性能剖析
- **测试验证**：实时性测试、压力测试、边界条件验证
- **系统集成**：多定时器协调、外设同步、功耗管理

## 章节内容

### 6.1 [定时器硬件原理](docs/01-timer-hardware.md)
- **技术点**：定时器时钟域、预分频器计算、自动重载机制、计数模式分析
- **实践项目**：[精密定时器](projects/precision_timer/) - 微秒级定时精度实现

### 6.2 [中断控制器架构](docs/02-nvic-architecture.md)
- **技术点**：NVIC优先级分组、中断向量表、Pending/Active状态、尾链优化
- **实践项目**：[中断管理器](projects/interrupt_manager/) - 多中断源优先级调度

### 6.3 [PWM波形生成](docs/03-pwm-generation.md)
- **技术点**：PWM模式配置、占空比计算、频率调节、互补输出、死区时间
- **实践项目**：[多通道PWM控制器](projects/multi_pwm_controller/) - 电机驱动应用

### 6.4 [输入捕获与测量](docs/04-input-capture.md)
- **技术点**：边沿检测、脉宽测量、频率计算、编码器接口、超声波测距
- **实践项目**：[信号分析仪](projects/signal_analyzer/) - 实时波形参数测量

### 6.5 [定时器同步机制](docs/05-timer-synchronization.md)
- **技术点**：主从模式、触发源选择、定时器级联、相位同步、时钟域crossing
- **实践项目**：[同步控制系统](projects/sync_control/) - 多轴运动控制

### 6.6 [中断安全编程](docs/06-interrupt-safety.md)
- **技术点**：临界区保护、原子操作、中断安全数据结构、优先级反转避免
- **实践项目**：[实时数据采集](projects/realtime_daq/) - 高速ADC数据处理

### 6.7 [实时任务调度](docs/07-realtime-scheduling.md)
- **技术点**：抢占式调度、时间片管理、优先级继承、调度延迟分析
- **实践项目**：[RTOS内核](projects/mini_rtos/) - 轻量级实时操作系统

### 6.8 [性能优化与调试](docs/08-performance-debug.md)
- **技术点**：中断延迟测量、抖动分析、CPU利用率统计、实时性验证
- **实践项目**：[性能监控器](projects/perf_monitor/) - 系统性能实时分析

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