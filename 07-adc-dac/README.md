# ADC/DAC数据采集教程

## 概述

ADC（模数转换器）和DAC（数模转换器）是嵌入式系统与模拟世界交互的重要桥梁。本教程将深入讲解STM32F4系列微控制器的ADC和DAC功能，以及如何在Rust环境中实现高精度、高效率的数据采集和信号生成系统。

## 学习目标

完成本教程后，你将能够：

- 深入理解ADC和DAC的工作原理
- 掌握STM32F4 ADC的各种采样模式
- 实现高速连续采样和DMA传输
- 设计精确的DAC信号生成系统
- 处理多通道数据采集和同步
- 实现数字信号处理和滤波算法
- 构建完整的数据采集系统
- 优化采样精度和系统性能

## 教程结构

### 📚 理论基础篇

#### 1. [ADC基础原理](01-adc-basics.md)
- ADC工作原理和转换过程
- 采样定理和奈奎斯特频率
- 量化误差和分辨率分析
- ADC性能参数详解
- 信号调理和抗混叠滤波

#### 2. [STM32F4 ADC架构](02-stm32-adc-architecture.md)
- STM32F4 ADC资源概览
- 三个12位ADC模块特性
- 采样时间和转换时间
- 参考电压和校准
- 温度传感器和内部参考

#### 3. [DAC基础原理](03-dac-basics.md)
- DAC工作原理和重构过程
- 输出滤波和信号质量
- DAC性能参数分析
- 波形生成和函数发生器
- 模拟输出应用场景

### 🛠️ 实践应用篇

#### 4. [单通道ADC采样](04-single-channel-adc.md)
- 基础ADC配置和使用
- 阻塞式和非阻塞式读取
- 电压测量和传感器接口
- 采样精度优化技巧

#### 5. [多通道ADC采集](05-multi-channel-adc.md)
- 多通道扫描模式
- 通道切换和时序控制
- 差分输入和单端输入
- 通道间串扰抑制

#### 6. [高速连续采样](06-continuous-sampling.md)
- 连续转换模式配置
- DMA循环传输设置
- 双缓冲和数据处理
- 采样率和带宽分析

### 🏭 高级应用篇

#### 7. [DAC波形生成](07-dac-waveform.md)
- 基础DAC输出配置
- 正弦波、方波、三角波生成
- 任意波形生成器
- 频率和幅度控制

#### 8. [数字信号处理](08-digital-signal-processing.md)
- 数字滤波器设计
- FFT频谱分析
- 信号检测和特征提取
- 实时信号处理算法

## 支持的硬件平台

### 主要开发板
- **STM32F407VG Discovery** - 主要演示平台
- **STM32F411CE BlackPill** - 小型化应用
- **STM32F429I Discovery** - 高性能应用
- **STM32F446RE Nucleo** - 标准开发板

### ADC/DAC规格
- **ADC分辨率**: 12位 (4096级)
- **ADC通道数**: 16个外部通道 + 3个内部通道
- **最大采样率**: 2.4MSPS (单通道)
- **参考电压**: 2.4V - 3.6V
- **DAC分辨率**: 12位 (4096级)
- **DAC通道数**: 2个独立通道
- **最大输出率**: 1MSPS

## 项目结构

```
07-adc-dac/
├── README.md                    # 本文档
├── 01-adc-basics.md            # ADC基础理论
├── 02-stm32-adc-architecture.md # STM32 ADC架构
├── 03-dac-basics.md            # DAC基础理论
├── 04-single-channel-adc.md    # 单通道ADC
├── 05-multi-channel-adc.md     # 多通道ADC
├── 06-continuous-sampling.md   # 连续采样
├── 07-dac-waveform.md          # DAC波形生成
├── 08-digital-signal-processing.md # 数字信号处理
├── examples/                   # 代码示例
│   ├── basic-adc/             # 基础ADC采样
│   ├── multi-channel/         # 多通道采集
│   ├── dma-sampling/          # DMA高速采样
│   ├── dac-generator/         # DAC信号生成
│   └── signal-analyzer/       # 信号分析器
├── projects/                   # 实战项目
│   ├── voltmeter/             # 数字电压表
│   ├── oscilloscope/          # 简易示波器
│   ├── function-generator/    # 函数发生器
│   ├── data-logger/           # 数据记录器
│   └── spectrum-analyzer/     # 频谱分析仪
└── docs/                      # 技术文档
    ├── calibration/           # 校准和精度
    ├── noise-analysis/        # 噪声分析
    └── performance-tuning/    # 性能优化
```

## 硬件准备

### 基础设备
1. **开发板**: STM32F407VG Discovery
2. **示波器**: 用于信号验证和分析
3. **万用表**: 精度验证和校准
4. **信号发生器**: 测试信号源
5. **面包板**: 电路搭建
6. **电阻电容**: 信号调理电路

### 传感器模块
- **电位器**: 模拟电压输入
- **温度传感器**: LM35、DS18B20
- **光敏电阻**: 光强检测
- **压力传感器**: 模拟压力测量
- **加速度计**: 三轴加速度传感器

### 测试设备
- **精密电压源**: 校准参考
- **低通滤波器**: 抗混叠滤波
- **运算放大器**: 信号放大
- **基准电压源**: 精度参考

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
STM32F407VG Discovery ADC引脚:
- ADC1_IN0: PA0  (ADC通道0)
- ADC1_IN1: PA1  (ADC通道1)
- ADC1_IN2: PA2  (ADC通道2)
- ADC1_IN3: PA3  (ADC通道3)
- VREF+: 3.3V    (参考电压正)
- VREF-: GND     (参考电压负)

DAC输出引脚:
- DAC1_OUT: PA4  (DAC通道1)
- DAC2_OUT: PA5  (DAC通道2)
```

### 3. 基础测试电路
```
电位器连接 (电压输入测试):
电位器 VCC → 3.3V
电位器 GND → GND
电位器 OUT → PA0 (ADC1_IN0)

LED指示电路 (DAC输出测试):
PA4 (DAC1) → 1kΩ电阻 → LED → GND
```

### 4. 运行第一个示例
```bash
# 进入基础ADC示例
cd 07-adc-dac/examples/basic-adc

# 编译并运行
cargo run

# 观察串口输出的ADC读数
```

## 实战项目列表

### 🟢 初级项目
1. **数字电压表** - 精确电压测量显示
2. **温度监测器** - 多点温度采集
3. **光强检测器** - 环境光强度测量
4. **简单信号发生器** - 基础波形输出

### 🟡 中级项目
1. **多通道数据记录器** - 同步多路采集
2. **简易示波器** - 波形显示和测量
3. **频率计** - 信号频率测量
4. **PID控制器** - 闭环控制系统

### 🔴 高级项目
1. **高速数据采集系统** - MHz级采样
2. **频谱分析仪** - FFT频域分析
3. **任意波形发生器** - 复杂波形合成
4. **振动分析仪** - 机械振动监测

## 性能指标

### ADC性能
- **分辨率**: 12位 (0.8mV @ 3.3V)
- **转换时间**: 1-15个ADC时钟周期
- **采样时间**: 3-480个ADC时钟周期
- **总转换时间**: 1.17μs (最快配置)
- **最大采样率**: 2.4MSPS (单通道)
- **多通道采样率**: 1.2MSPS (双通道交替)

### DAC性能
- **分辨率**: 12位 (0.8mV @ 3.3V)
- **建立时间**: 3μs (典型值)
- **输出范围**: 0V - VREF
- **最大输出电流**: ±20mA
- **输出阻抗**: 15kΩ (典型值)
- **更新率**: 1MSPS (最大)

### 精度指标
- **积分非线性**: ±2 LSB
- **差分非线性**: ±1 LSB
- **总谐波失真**: -80dB (1kHz)
- **信噪比**: 66dB (典型值)
- **有效位数**: 10.5位 (ENOB)

## 校准和精度优化

### ADC校准方法
1. **偏移校准**
   - 输入接地测量零点偏移
   - 软件补偿偏移误差

2. **增益校准**
   - 使用精密参考电压
   - 计算实际增益系数

3. **线性度校准**
   - 多点校准建立查找表
   - 分段线性插值补偿

### 噪声抑制技术
1. **硬件措施**
   - 模拟电源滤波
   - 接地平面设计
   - 屏蔽和隔离

2. **软件措施**
   - 多次采样平均
   - 数字滤波算法
   - 异常值检测和剔除

## 调试工具

### 软件工具
1. **串口监视器** - 实时数据观察
2. **数据可视化** - Python/MATLAB绘图
3. **频谱分析** - FFT工具
4. **统计分析** - 精度评估工具

### 硬件工具
1. **示波器** - 波形分析
   - 带宽: ≥100MHz
   - 采样率: ≥1GSa/s
   - 精度: ≥8位

2. **万用表** - 精度验证
   - 精度: ≥0.1%
   - 分辨率: ≥4.5位

3. **信号发生器** - 测试信号
   - 频率范围: DC-10MHz
   - 幅度精度: ≥1%

### 测试方法
1. **静态测试**
   - DC精度测量
   - 线性度测试
   - 噪声测量

2. **动态测试**
   - 频率响应测试
   - 谐波失真测量
   - 建立时间测试

## 常见问题解决

### ADC问题
1. **读数不稳定**
   - 检查参考电压稳定性
   - 增加采样时间
   - 添加输入滤波

2. **精度不够**
   - 校准偏移和增益
   - 优化PCB布局
   - 使用外部精密参考

3. **采样率达不到要求**
   - 优化ADC时钟配置
   - 使用DMA传输
   - 减少采样时间

### DAC问题
1. **输出波形失真**
   - 检查负载阻抗
   - 添加输出缓冲器
   - 优化更新频率

2. **输出范围不对**
   - 验证参考电压
   - 检查输出配置
   - 确认负载影响

## 扩展学习

### 相关主题
- **运算放大器** - 信号调理电路
- **滤波器设计** - 模拟和数字滤波
- **控制系统** - 反馈控制理论
- **信号处理** - DSP算法实现

### 高级概念
- **Sigma-Delta ADC** - 高精度转换技术
- **过采样技术** - 提高有效分辨率
- **相关双采样** - 噪声抑制技术
- **自适应滤波** - 智能信号处理

## 参考资源

### 官方文档
- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [STM32F4 ADC Application Note](https://www.st.com/resource/en/application_note/dm00051969.pdf)
- [STM32F4 DAC Application Note](https://www.st.com/resource/en/application_note/dm00087990.pdf)

### 技术标准
- [IEEE 1241 ADC Testing](https://standards.ieee.org/standard/1241-2010.html)
- [IEC 60748 Semiconductor Devices](https://webstore.iec.ch/publication/3299)

### 开源项目
- [stm32f4xx-hal ADC](https://github.com/stm32-rs/stm32f4xx-hal)
- [embedded-hal ADC traits](https://github.com/rust-embedded/embedded-hal)
- [DSP algorithms in Rust](https://github.com/korken89/dsp-rs)

### 学习资源
- [ADC基础教程](https://www.analog.com/en/analog-dialogue/articles/adc-basics.html)
- [DAC设计指南](https://www.ti.com/lit/an/slaa013/slaa013.pdf)
- [数字信号处理](https://www.dspguide.com/)

## 贡献指南

欢迎为本教程贡献代码、文档或建议：

1. **报告问题**: 在GitHub Issues中报告bug或提出改进建议
2. **提交代码**: 通过Pull Request提交代码改进
3. **完善文档**: 帮助改进教程内容和示例
4. **分享经验**: 在社区中分享使用经验和最佳实践

## 版本历史

- **v1.0.0** - 初始版本，包含基础ADC/DAC教程
- **v1.1.0** - 添加多通道采集和DMA传输
- **v1.2.0** - 增加数字信号处理算法
- **v1.3.0** - 添加高级应用和性能优化

---

*本教程是Rust嵌入式编程完整教程的一部分。更多内容请参考主教程目录。*

**上一章**: [定时器和中断处理](../06-timers-interrupts/README.md)  
**下一章**: [I2C/SPI通信](../08-i2c-spi/README.md)