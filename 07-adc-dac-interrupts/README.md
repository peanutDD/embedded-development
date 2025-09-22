# 第7章：ADC/DAC与数据采集

## 概述

ADC/DAC系统是嵌入式设备与模拟世界交互的核心接口，决定了系统的感知能力和控制精度。本章深入探讨STM32F4系列微控制器的高性能ADC/DAC架构、信号处理算法，以及Rust语言在实时数据采集系统中的零成本抽象和内存安全特性。

通过硬件级的采样时序控制、DMA驱动的高速数据传输、数字信号处理算法实现，我们将构建工业级的数据采集与信号生成系统。涵盖从基础的电压测量到复杂的多通道同步采集，从简单的DAC输出到精密的任意波形生成，全面掌握模拟信号处理的嵌入式实现。

核心技术包括：Σ-Δ调制的过采样技术、抗混叠滤波器设计、DMA双缓冲机制、实时FFT频谱分析、自适应数字滤波、以及基于统计学的信号质量评估算法。

## 学习目标

### 硬件层面理解
- **ADC架构**：SAR转换原理、采样保持电路、参考电压配置、温度漂移补偿
- **DAC架构**：R-2R网络、电流舵结构、输出缓冲器、建立时间分析
- **信号调理**：运放电路设计、抗混叠滤波、信号放大、阻抗匹配
- **时序控制**：采样时钟生成、转换触发、通道切换时序、同步机制

### 软件抽象设计
- **类型安全**：编译时通道配置验证、采样率约束检查、数据类型转换
- **零成本抽象**：HAL层优化、DMA零拷贝传输、内联数学运算
- **内存管理**：循环缓冲区、双缓冲机制、堆栈溢出保护
- **并发处理**：中断驱动采集、异步数据处理、线程安全的数据结构

### 信号处理实现
- **数字滤波**：FIR/IIR滤波器、移动平均、中值滤波、卡尔曼滤波
- **频域分析**：FFT/DFT算法、功率谱密度、频谱泄漏抑制
- **统计分析**：均值方差计算、异常值检测、信噪比评估
- **实时处理**：流式数据处理、在线算法、延迟优化

### 工程实践能力
- **系统设计**：采样系统架构、数据流设计、性能需求分析
- **精度优化**：校准算法、非线性校正、温度补偿、噪声抑制
- **调试技术**：信号完整性分析、采样抖动测量、频谱分析
- **可靠性设计**：故障检测、数据完整性验证、系统自检

## 章节内容

### 7.1 [ADC硬件原理](docs/01-adc-hardware.md)
- **技术点**：SAR转换原理、采样保持电路、参考电压系统、温度漂移补偿
- **实践项目**：[高精度电压表](projects/voltmeter/) - 多量程数字万用表实现

### 7.2 [多通道采集系统](docs/02-multi-channel.md)
- **技术点**：通道复用、扫描模式、注入通道、同步采样、串扰抑制
- **实践项目**：[数据采集器](projects/data_logger/) - 16通道同步数据记录

### 7.3 [高速连续采样](docs/03-continuous-sampling.md)
- **技术点**：DMA循环传输、双缓冲机制、采样率控制、数据流管理
- **实践项目**：[示波器](projects/oscilloscope/) - 实时波形显示与分析

### 7.4 [DAC信号生成](docs/04-dac-generation.md)
- **技术点**：R-2R网络、输出缓冲、波形合成、频率精度、相位控制
- **实践项目**：[函数发生器](projects/function_generator/) - 任意波形生成器

### 7.5 [数字信号处理](docs/05-signal-processing.md)
- **技术点**：FIR/IIR滤波器、FFT算法、功率谱分析、自适应滤波
- **实践项目**：[频谱分析仪](projects/spectrum_analyzer/) - 实时频域分析

### 7.6 [采样系统校准](docs/06-calibration.md)
- **技术点**：增益误差校正、偏移误差补偿、非线性校正、温度补偿
- **实践项目**：[精密测量系统](projects/precision_measurement/) - 工业级测量精度

### 7.7 [噪声抑制技术](docs/07-noise-reduction.md)
- **技术点**：过采样技术、数字滤波、统计降噪、信号完整性分析
- **实践项目**：[低噪声采集](projects/low_noise_daq/) - 微弱信号检测

### 7.8 [实时数据处理](docs/08-realtime-processing.md)
- **技术点**：流式处理、在线算法、延迟优化、数据压缩、异常检测
- **实践项目**：[实时监控系统](projects/realtime_monitor/) - 工业过程监控

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