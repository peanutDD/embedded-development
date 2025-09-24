# 第六章：定时器与中断 (Timers and Interrupts)

本章深入探讨STM32F4xx微控制器的定时器功能和中断处理机制，通过三个综合性项目展示了从基础定时器应用到复杂信号处理的完整技术栈。

## 章节概述

定时器是嵌入式系统中最重要的外设之一，它不仅可以提供精确的时间基准，还能实现PWM输出、输入捕获、编码器接口等多种功能。本章通过实际项目展示了定时器的各种应用场景，并结合中断机制实现高效的实时系统。

## 项目结构

```
06-timers-interrupts/
├── README.md                    # 本章节总体介绍
├── projects/
│   ├── basic_timer/             # 基础定时器项目
│   │   ├── src/
│   │   │   ├── lib.rs          # 定时器核心库
│   │   │   └── bin/
│   │   │       ├── simple_timer.rs      # 简单定时器示例
│   │   │       ├── interrupt_timer.rs   # 中断定时器示例
│   │   │       ├── watchdog_timer.rs    # 看门狗定时器示例
│   │   │       └── rtc_calendar.rs      # RTC日历示例
│   │   ├── Cargo.toml
│   │   └── README.md
│   ├── pwm_controller/          # PWM控制器项目
│   │   ├── src/
│   │   │   ├── lib.rs          # PWM核心库
│   │   │   └── bin/
│   │   │       ├── motor_control.rs     # 电机控制示例
│   │   │       ├── led_dimming.rs       # LED调光示例
│   │   │       ├── servo_control.rs     # 舵机控制示例
│   │   │       └── multi_channel_pwm.rs # 多通道PWM示例
│   │   ├── Cargo.toml
│   │   └── README.md
│   └── input_capture/           # 输入捕获项目
│       ├── src/
│       │   ├── lib.rs          # 输入捕获核心库
│       │   └── bin/
│       │       ├── frequency_measurement.rs    # 频率测量示例
│       │       ├── pulse_width_measurement.rs  # 脉宽测量示例
│       │       ├── encoder_interface.rs        # 编码器接口示例
│       │       └── signal_analyzer.rs          # 信号分析器示例
│       ├── Cargo.toml
│       └── README.md
└── docs/                        # 技术文档
    ├── timer_theory.md          # 定时器理论基础
    ├── interrupt_handling.md    # 中断处理机制
    ├── pwm_principles.md        # PWM原理与应用
    └── signal_processing.md     # 信号处理技术
```

## 核心技术要点

### 1. 定时器基础 (Timer Fundamentals)
- **定时器类型**: 基本定时器、通用定时器、高级定时器
- **时钟配置**: 预分频器、自动重载、计数模式
- **中断机制**: 更新中断、比较中断、捕获中断
- **DMA集成**: 定时器触发的DMA传输

### 2. PWM技术 (PWM Technology)
- **PWM原理**: 脉宽调制的基本概念和应用
- **多通道PWM**: 同步和异步PWM输出
- **死区控制**: 功率电子应用中的安全保护
- **波形生成**: 正弦波、三角波、自定义波形

### 3. 输入捕获 (Input Capture)
- **边沿检测**: 上升沿、下降沿、双边沿捕获
- **频率测量**: 基于周期测量的频率计算
- **脉宽测量**: 占空比和脉宽精确测量
- **编码器接口**: 正交编码器信号解码

### 4. 信号处理 (Signal Processing)
- **数字滤波**: 低通、高通、带通滤波器
- **频谱分析**: 基于DFT的频域分析
- **信号质量**: SNR、THD、抖动分析
- **实时处理**: 流式信号处理算法

## 项目详细介绍

### 1. 基础定时器项目 (Basic Timer)
**技术特点**:
- 多种定时器模式：简单定时、中断定时、看门狗、RTC日历
- 高精度时间管理：微秒级定时精度
- 中断优先级管理：多级中断处理
- 实时系统基础：任务调度和时间片管理

**核心功能**:
- 精确延时和定时功能
- 系统时钟和时间戳生成
- 看门狗保护机制
- 实时时钟和日历功能

### 2. PWM控制器项目 (PWM Controller)
**技术特点**:
- 多通道PWM输出：同步和异步控制
- 电机控制算法：PID控制、速度调节
- LED调光技术：亮度渐变、色彩控制
- 舵机精确控制：角度定位、平滑运动

**核心功能**:
- 直流电机速度和方向控制
- LED亮度和颜色调节
- 舵机角度精确控制
- 多通道协调输出

### 3. 输入捕获项目 (Input Capture)
**技术特点**:
- 高精度频率测量：基于周期和脉冲计数
- 脉宽和占空比测量：PWM信号分析
- 编码器接口：正交信号解码、位置和速度测量
- 信号分析：频谱分析、信号质量评估

**核心功能**:
- 外部信号频率精确测量
- PWM信号参数分析
- 旋转编码器位置检测
- 复杂信号的数字处理

## 技术深度分析

### 定时器架构
STM32F4xx系列提供了多种类型的定时器：
- **基本定时器 (TIM6/TIM7)**: 简单的向上计数，主要用于DAC触发和时基生成
- **通用定时器 (TIM2-TIM5, TIM9-TIM14)**: 支持输入捕获、输出比较、PWM生成
- **高级定时器 (TIM1/TIM8)**: 具备互补输出、死区时间、刹车功能

### 中断优先级管理
采用ARM Cortex-M4的NVIC (Nested Vectored Interrupt Controller)：
- **优先级分组**: 支持抢占优先级和子优先级
- **中断嵌套**: 高优先级中断可以打断低优先级中断
- **尾链优化**: 连续中断处理的性能优化

### DMA集成
定时器与DMA的深度集成：
- **触发源**: 定时器更新、比较匹配、捕获事件
- **数据传输**: 零CPU开销的高速数据传输
- **循环模式**: 连续波形生成和数据采集

## 性能指标

### 时间精度
- **定时精度**: ±1μs (基于84MHz系统时钟)
- **PWM分辨率**: 16位 (65536级)
- **频率测量范围**: 1Hz - 42MHz
- **脉宽测量精度**: 12ns (基于84MHz定时器时钟)

### 实时性能
- **中断响应时间**: <1μs (最高优先级)
- **上下文切换开销**: <500ns
- **DMA传输速率**: 高达21MB/s
- **多通道同步精度**: <100ns

## 高级特性

### 多定时器同步
- **主从模式**: 一个定时器控制多个定时器
- **触发链**: 定时器级联触发
- **相位同步**: 多通道PWM相位对齐

### 自适应控制算法
- **PID控制器**: 比例-积分-微分控制
- **模糊控制**: 基于模糊逻辑的智能控制
- **自适应滤波**: 动态调整滤波参数

### 信号处理算法
- **数字滤波器**: IIR/FIR滤波器实现
- **FFT分析**: 快速傅里叶变换
- **统计分析**: 均值、方差、峰值检测

## 调试和测试

### 调试工具
- **逻辑分析仪**: 时序分析和信号捕获
- **示波器**: 波形观察和测量
- **性能分析器**: 中断统计和性能监控

### 测试方法
- **单元测试**: 各功能模块独立测试
- **集成测试**: 多模块协同工作测试
- **压力测试**: 高负载和边界条件测试
- **实时性测试**: 响应时间和抖动测量

## 应用领域

### 工业控制
- **电机驱动**: 步进电机、伺服电机、无刷直流电机
- **过程控制**: 温度、压力、流量控制
- **机器人控制**: 关节控制、路径规划

### 消费电子
- **LED照明**: 智能调光、色彩控制
- **音频处理**: PWM音频输出、音效处理
- **用户界面**: 按键检测、触摸感应

### 汽车电子
- **引擎控制**: 点火时序、燃油喷射
- **车身控制**: 车窗、座椅、空调控制
- **安全系统**: ABS、ESP、安全气囊

### 通信设备
- **信号调制**: PWM调制、频率合成
- **时钟恢复**: 数据时钟提取
- **协议处理**: 时序要求严格的通信协议

## 学习路径建议

### 初级阶段
1. **基础概念**: 理解定时器工作原理和中断机制
2. **简单应用**: 实现基本的定时和PWM功能
3. **调试技能**: 学会使用调试工具分析时序

### 中级阶段
1. **复杂控制**: 实现PID控制和多通道协调
2. **信号处理**: 掌握输入捕获和频率测量
3. **性能优化**: 优化中断响应和DMA使用

### 高级阶段
1. **系统设计**: 设计复杂的实时控制系统
2. **算法实现**: 实现高级控制和信号处理算法
3. **产品开发**: 完成完整的产品级应用

## 扩展阅读

### 技术文档
- [STM32F4xx Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [ARM Cortex-M4 Technical Reference Manual](https://developer.arm.com/documentation/100166/0001)
- [Rust Embedded Book](https://docs.rust-embedded.org/book/)

### 学术论文
- "Real-Time Systems: Design Principles for Distributed Embedded Applications"
- "Digital Signal Processing: Principles, Algorithms, and Applications"
- "Modern Control Engineering" by Katsuhiko Ogata

### 开源项目
- [RTIC (Real-Time Interrupt-driven Concurrency)](https://rtic.rs/)
- [Embassy (Async Rust for Embedded)](https://embassy.dev/)
- [STM32F4xx HAL](https://github.com/stm32-rs/stm32f4xx-hal)

## 总结

本章通过三个综合性项目全面展示了STM32F4xx定时器和中断系统的强大功能。从基础的时间管理到复杂的信号处理，从简单的PWM输出到精密的控制算法，这些项目涵盖了嵌入式系统开发中最重要的时间相关技术。

通过学习本章内容，您将掌握：
- 定时器硬件的深度理解和灵活应用
- 中断系统的高效管理和优化技巧
- PWM技术在各种控制场景中的应用
- 输入捕获在信号测量和分析中的作用
- 实时系统设计的核心原理和实践方法

这些技能将为您在工业控制、消费电子、汽车电子等领域的嵌入式开发奠定坚实的基础。

## 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](../../LICENSE) 文件。

## 贡献指南

欢迎提交问题报告、功能请求和代码贡献。请遵循以下步骤：

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

## 联系方式

如有问题或建议，请通过以下方式联系：
- 提交 GitHub Issue
- 发送邮件至项目维护者
- 参与项目讨论区

---

*本文档持续更新中，最后更新时间：2024年*
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