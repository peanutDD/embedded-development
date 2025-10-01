# Precision Timer Project

这是一个基于STM32F4xx微控制器的高精度定时器项目，展示了嵌入式系统中精确时间测量、频率生成和定时分析的实现方法。

## 项目结构

```
precision_timer/
├── Cargo.toml                    # 项目配置文件
├── src/
│   ├── lib.rs                    # 核心库文件
│   ├── main.rs                   # 主程序入口
│   └── bin/
│       ├── microsecond_timer.rs  # 微秒级定时器示例
│       ├── time_measurement.rs   # 时间测量示例
│       ├── frequency_generator.rs # 频率生成器示例
│       └── timing_analysis.rs    # 定时分析示例
└── README.md                     # 项目文档
```

## 功能特性

### 1. 微秒级定时器 (microsecond_timer.rs)

**核心功能：**
- 支持微秒、100纳秒、10纳秒和1纳秒精度级别
- 实时精度测试和误差分析
- 自适应精度调整
- 高精度延时函数

**演示模式：**
- **微秒精度模式**：1μs精度定时，适用于一般应用
- **100纳秒精度模式**：高精度定时，适用于精密控制
- **10纳秒精度模式**：超高精度定时，适用于高速信号处理
- **自适应精度模式**：根据CPU负载自动调整精度级别

**LED指示：**
- PB0: 系统状态指示（心跳）
- PB1: 定时器活动指示
- PB2: 微秒精度指示
- PB3: 纳秒精度指示
- PB4: 精度级别指示
- PB5: 延时测试指示
- PB6: 精度准确性指示
- PB7: 错误指示

### 2. 时间测量 (time_measurement.rs)

**核心功能：**
- 持续时间测量
- 时间间隔测量
- 频率测量
- 周期测量

**演示模式：**
- **持续时间测量模式**：测量事件的持续时间
- **间隔测量模式**：测量事件之间的时间间隔
- **频率测量模式**：测量信号频率
- **综合测量模式**：同时进行多种类型的测量

**LED指示：**
- PB0: 系统状态指示
- PB1: 测量活动指示
- PB2: 持续时间测量指示
- PB3: 间隔测量指示
- PB4: 频率测量指示
- PB5: 测量精度指示
- PB6: 溢出警告指示
- PB7: 测量错误指示

### 3. 频率生成器 (frequency_generator.rs)

**核心功能：**
- 多通道频率生成
- 可调占空比
- 相位控制
- 波形类型选择

**演示模式：**
- **单频率模式**：生成单一频率信号
- **多频率模式**：同时生成多个不同频率
- **扫频模式**：频率动态扫描
- **同步模式**：多通道相位同步

**LED指示：**
- PB0: 系统状态指示
- PB1: 生成器活动指示
- PB2: 频率指示（根据频率调整闪烁速度）
- PB3: 波形类型指示
- PB4: 相位同步指示
- PB5: 频率精度指示
- PB6: CPU负载指示
- PB7: 生成错误指示

### 4. 定时分析 (timing_analysis.rs)

**核心功能：**
- 抖动分析
- 时钟漂移检测
- 稳定性评估
- 综合性能分析

**演示模式：**
- **抖动分析模式**：分析定时器抖动特性
- **漂移分析模式**：检测时钟漂移
- **稳定性分析模式**：评估定时稳定性
- **综合分析模式**：全面的定时性能分析

**LED指示：**
- PB0: 系统状态指示
- PB1: 分析活动指示
- PB2: 抖动水平指示
- PB3: 时钟漂移指示
- PB4: 稳定性指示
- PB5: 精度指示
- PB6: 性能评分指示
- PB7: 分析错误指示

## 硬件连接

### 基本连接
- **系统时钟**：168MHz（最高频率以获得最佳精度）
- **按钮**：PC13（模式切换，内部上拉）
- **LED指示灯**：PB0-PB7（推挽输出）

### 频率生成器额外连接
- **PWM输出**：PA0-PA3（TIM2-TIM5的PWM通道）
- **示波器连接**：连接到PA0-PA3观察生成的波形

### 测量输入（可选）
- **外部信号输入**：可连接到定时器输入捕获引脚进行外部信号测量

## 编译和运行

### 编译单个示例
```bash
# 编译微秒定时器示例
cargo build --bin microsecond_timer --release

# 编译时间测量示例
cargo build --bin time_measurement --release

# 编译频率生成器示例
cargo build --bin frequency_generator --release

# 编译定时分析示例
cargo build --bin timing_analysis --release
```

### 烧录到目标设备
```bash
# 烧录微秒定时器示例
cargo flash --bin microsecond_timer --chip STM32F407VGTx

# 烧录其他示例
cargo flash --bin time_measurement --chip STM32F407VGTx
cargo flash --bin frequency_generator --chip STM32F407VGTx
cargo flash --bin timing_analysis --chip STM32F407VGTx
```

### 调试
```bash
# 启动调试会话
cargo embed --bin microsecond_timer
```

## 核心技术原理

### 1. 高精度时间测量

**DWT计数器使用：**
- 利用ARM Cortex-M4的DWT（Data Watchpoint and Trace）计数器
- 提供CPU周期级别的时间测量精度
- 在168MHz时钟下，理论精度可达约6纳秒

**时间戳管理：**
```rust
// 获取高精度时间戳
let start_cycles = DWT::get_cycle_count();
// 执行被测量的操作
let end_cycles = DWT::get_cycle_count();
let duration_ns = (end_cycles.wrapping_sub(start_cycles) as u64 * 1_000_000_000) / 168_000_000;
```

### 2. 精密定时器配置

**定时器预分频计算：**
```rust
// 计算预分频值以获得所需精度
let prescaler = (clock_frequency / desired_frequency) - 1;
let auto_reload = (desired_frequency / target_frequency) - 1;
```

**中断优先级管理：**
- 高精度定时器使用最高优先级中断
- 系统服务使用较低优先级避免干扰精确定时

### 3. 频率生成算法

**PWM参数计算：**
```rust
// 计算PWM参数
let period = clock_frequency / target_frequency;
let duty_cycles = (period * duty_cycle_percent / 100.0) as u32;
let phase_offset = (period * phase_degrees / 360.0) as u32;
```

**多通道同步：**
- 使用主从定时器模式实现多通道同步
- 通过硬件触发确保相位关系准确

### 4. 统计分析方法

**抖动计算：**
```rust
// 计算相邻测量的抖动
let jitter = current_measurement.abs_diff(previous_measurement);
let avg_jitter = total_jitter / measurement_count;
```

**Allan方差计算：**
```rust
// 用于评估时钟稳定性
let variance = measurements.iter()
    .map(|&x| (x as f64 - mean).powi(2))
    .sum::<f64>() / measurements.len() as f64;
```

## 性能分析

### 精度指标
- **微秒级精度**：±50ns典型误差
- **纳秒级精度**：±10ns典型误差（理论极限约6ns）
- **频率精度**：±0.01%（在稳定温度条件下）
- **相位精度**：±1度

### 资源使用
- **RAM使用**：约8KB（包括测量缓冲区）
- **Flash使用**：约32KB（单个示例）
- **CPU负载**：10-30%（取决于精度要求和测量频率）

### 实时性能
- **中断响应时间**：<1μs
- **测量延迟**：<100ns
- **数据处理延迟**：<10μs

## 注意事项

### 硬件要求
1. **时钟稳定性**：使用外部晶振以获得最佳精度
2. **电源质量**：稳定的电源对高精度测量至关重要
3. **PCB设计**：注意时钟信号的布线和去耦

### 软件注意事项
1. **中断延迟**：避免在高精度测量期间执行长时间的中断服务
2. **内存对齐**：确保关键数据结构的内存对齐
3. **编译优化**：使用release模式以获得最佳性能

### 环境因素
1. **温度影响**：温度变化会影响晶振频率
2. **电磁干扰**：强电磁场可能影响测量精度
3. **机械振动**：可能影响晶振稳定性

## 扩展功能

### 可能的改进
1. **温度补偿**：添加温度传感器进行频率补偿
2. **自动校准**：实现基于外部参考的自动校准
3. **数据记录**：添加测量数据的存储和回放功能
4. **网络接口**：通过以太网或WiFi远程监控

### 高级特性
1. **GPS同步**：使用GPS 1PPS信号作为时间基准
2. **原子钟接口**：连接外部高精度时间源
3. **多设备同步**：实现多个设备之间的时间同步
4. **实时数据分析**：在线统计分析和异常检测

## 应用场景

### 工业应用
- **精密控制系统**：需要微秒级定时精度的控制应用
- **测试设备**：高精度信号发生器和测量仪器
- **同步系统**：多设备协调和同步应用

### 科研应用
- **物理实验**：需要精确时间测量的实验设备
- **信号处理**：高速数据采集和处理系统
- **通信系统**：精确时钟恢复和同步

### 商业应用
- **金融交易**：高频交易系统的时间戳
- **网络设备**：精确的网络时间协议实现
- **音视频设备**：专业音视频设备的同步

## 相关资源

### 技术文档
- [STM32F4xx参考手册](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [ARM Cortex-M4技术参考手册](https://developer.arm.com/documentation/100166/0001)
- [DWT和ITM编程指南](https://developer.arm.com/documentation/ddi0314/h)

### 开发工具
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [probe-rs](https://probe.rs/) - Rust嵌入式调试工具
- [RTT](https://github.com/probe-rs/rtt-target) - 实时传输调试

### 相关库
- [stm32f4xx-hal](https://crates.io/crates/stm32f4xx-hal) - STM32F4 HAL库
- [cortex-m](https://crates.io/crates/cortex-m) - Cortex-M处理器支持
- [embedded-hal](https://crates.io/crates/embedded-hal) - 嵌入式HAL抽象

---

**作者**: Embedded Development Team  
**版本**: 1.0.0  
**最后更新**: 2024年12月