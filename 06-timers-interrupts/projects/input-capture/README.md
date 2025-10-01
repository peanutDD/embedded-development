# 输入捕获项目 (Input Capture Project)

本项目展示了基于STM32F4xx微控制器的输入捕获功能实现，包含频率测量、脉宽测量、编码器接口和信号分析等多个示例。

## 项目结构

```
input_capture/
├── Cargo.toml              # 项目配置文件
├── README.md               # 项目文档
└── src/
    ├── lib.rs              # 核心库文件
    ├── main.rs             # 默认主程序
    └── bin/
        ├── frequency_meter.rs    # 频率计示例
        ├── pulse_width_meter.rs  # 脉冲宽度测量示例
        ├── encoder_reader.rs     # 编码器读取示例
        └── signal_analyzer.rs    # 信号分析器示例
```

## 示例说明

### 1. frequency_meter.rs - 频率计
**功能特性：**
- 高精度频率测量（0.1Hz - 10MHz）
- 多种测量模式（单次、连续、平均）
- 自动量程切换
- 实时频率显示
- 统计分析功能

**硬件连接：**
- PA0: 输入信号
- PB0-PB7: LED指示灯
- PC13: 模式切换按钮

**演示模式：**
- 单次测量：测量一个周期的频率
- 连续测量：持续测量并显示
- 平均测量：多次测量取平均值
- 统计模式：显示最大、最小、平均值

### 2. pulse_width_meter.rs - 脉冲宽度测量
**功能特性：**
- 精确脉冲宽度测量（μs级精度）
- 占空比计算
- 周期测量
- 多种测量模式
- 实时数据更新

**硬件连接：**
- PA0: 脉冲输入信号
- PB0-PB7: 状态指示LED
- PC13: 模式切换按钮

**测量模式：**
- 单脉冲：测量单个脉冲宽度
- 占空比：计算PWM信号占空比
- 频率：基于脉冲宽度计算频率
- 连续：连续测量模式

### 3. encoder_reader.rs - 编码器读取
**功能特性：**
- 增量编码器位置读取
- 速度计算
- 方向检测
- 角度转换
- 多种显示模式

**硬件连接：**
- PA0: 编码器A相
- PA1: 编码器B相
- PB0-PB7: 状态显示LED
- PC13: 显示模式切换按钮

**显示模式：**
- 位置模式：显示当前位置
- 速度模式：显示旋转速度
- 方向模式：显示旋转方向
- 角度模式：显示角度值

### 4. signal_analyzer.rs - 信号分析器
**功能特性：**
- 多通道信号分析
- 频率分析
- 占空比分析
- 相位分析
- 统计分析（抖动、稳定性）

**硬件连接：**
- PA0-PA3: 四路输入信号
- PB0-PB7: 分析结果显示LED
- PC13: 分析模式切换按钮

**分析模式：**
- 频率分析：多通道频率测量
- 占空比分析：PWM信号占空比
- 相位分析：多信号相位关系
- 统计分析：信号质量评估

## 编译和运行

### 编译单个示例
```bash
# 编译频率计示例
cargo build --bin frequency_meter --release

# 编译脉冲宽度测量示例
cargo build --bin pulse_width_meter --release

# 编译编码器读取示例
cargo build --bin encoder_reader --release

# 编译信号分析器示例
cargo build --bin signal_analyzer --release
```

### 烧录到目标板
```bash
# 使用probe-rs烧录
probe-rs run --chip STM32F407VGTx target/thumbv7em-none-eabihf/release/frequency_meter

# 或使用OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program target/thumbv7em-none-eabihf/release/frequency_meter verify reset exit"
```

## 核心技术原理

### 输入捕获工作原理
1. **边沿检测**：检测输入信号的上升沿或下降沿
2. **时间戳记录**：记录边沿发生时的定时器计数值
3. **时间差计算**：通过时间戳差值计算周期、频率等参数
4. **数据处理**：对原始数据进行滤波、平均等处理

### 频率测量方法
```rust
// 基于周期测量的频率计算
let period_ticks = current_timestamp - last_timestamp;
let period_seconds = period_ticks as f32 / timer_frequency as f32;
let frequency_hz = 1.0 / period_seconds;
```

### 脉冲宽度测量
```rust
// 脉冲宽度计算
let pulse_width_ticks = falling_edge_time - rising_edge_time;
let pulse_width_us = (pulse_width_ticks as f32 * 1_000_000.0) / timer_frequency as f32;

// 占空比计算
let duty_cycle = pulse_width_ticks as f32 / period_ticks as f32;
```

### 编码器解码算法
```rust
// 增量编码器四倍频解码
match (current_a, current_b, last_a, last_b) {
    (true, false, false, false) => position += 1,   // A上升，B低
    (true, true, true, false) => position += 1,     // B上升，A高
    (false, true, true, true) => position += 1,     // A下降，B高
    (false, false, false, true) => position += 1,   // B下降，A低
    // 反向旋转的情况...
}
```

## 性能分析

### 测量精度
- **频率测量精度**：±0.01% (取决于参考时钟精度)
- **时间测量分辨率**：1μs (84MHz时钟)
- **最大测量频率**：10MHz
- **最小测量频率**：0.1Hz

### 资源使用
- **RAM使用**：约2KB (包括缓冲区)
- **Flash使用**：约8KB (单个示例)
- **定时器资源**：TIM2 (主要)，TIM3 (辅助)
- **GPIO资源**：4个输入，8个输出

### 实时性能
- **中断响应时间**：<5μs
- **数据处理延迟**：<100μs
- **显示更新频率**：50Hz
- **最大事件处理速率**：100kHz

## 注意事项

### 硬件注意事项
1. **输入信号电平**：确保输入信号电平符合STM32规范（0-3.3V）
2. **信号完整性**：对于高频信号，注意传输线阻抗匹配
3. **抗干扰**：在嘈杂环境中使用施密特触发器输入
4. **上拉电阻**：对于开集电极输出，需要外部上拉电阻

### 软件注意事项
1. **中断优先级**：合理设置中断优先级避免丢失事件
2. **缓冲区大小**：根据信号频率调整事件缓冲区大小
3. **溢出处理**：处理定时器计数器溢出情况
4. **滤波算法**：对于噪声信号实施适当的数字滤波

### 调试技巧
1. **使用逻辑分析仪**：验证输入信号质量
2. **监控中断频率**：确保系统不会过载
3. **检查时钟配置**：验证定时器时钟频率设置
4. **测试边界条件**：测试极高和极低频率信号

## 扩展功能

### 可能的改进
1. **自适应滤波**：根据信号特性自动调整滤波参数
2. **多通道同步**：实现多通道同步采样
3. **数据记录**：添加数据记录和回放功能
4. **通信接口**：添加UART/SPI/I2C数据输出
5. **校准功能**：实现自动校准和误差补偿

### 高级应用
1. **电机控制**：用于电机编码器反馈
2. **信号发生器**：配合PWM输出实现信号发生器
3. **协议解码**：解码各种数字通信协议
4. **振动分析**：分析机械振动信号
5. **音频分析**：基础音频信号分析

## 应用场景

### 工业应用
- 电机速度检测
- 生产线计数
- 振动监测
- 流量测量

### 实验室应用
- 信号发生器校准
- 频率标准比对
- 时序分析
- 协议测试

### 教育应用
- 数字信号处理教学
- 嵌入式系统实验
- 控制系统实践
- 测量技术学习

## 相关资源

### 参考文档
- [STM32F4xx参考手册](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [STM32F4xx HAL库文档](https://www.st.com/resource/en/user_manual/dm00105879.pdf)
- [Cortex-M4编程手册](https://developer.arm.com/documentation/dui0553/latest/)

### 开发工具
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [probe-rs](https://probe.rs/)
- [OpenOCD](http://openocd.org/)

### 相关项目
- [embedded-hal](https://github.com/rust-embedded/embedded-hal)
- [stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal)
- [cortex-m](https://github.com/rust-embedded/cortex-m)

---

**注意**：本项目仅用于教育和学习目的。在实际产品中使用时，请进行充分的测试和验证。