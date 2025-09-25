# DAC基础理论

## 概述

数模转换器（DAC，Digital-to-Analog Converter）是将数字信号转换为模拟信号的关键器件，在嵌入式系统中用于信号生成、控制输出、音频处理等应用。本章深入探讨DAC的工作原理、架构类型、性能参数，以及STM32F4系列微控制器中DAC的具体实现。

## DAC工作原理

### 基本概念

DAC的核心功能是将离散的数字码字转换为连续的模拟电压或电流。转换过程包括：

1. **数字输入**：接收n位二进制数字码
2. **权重求和**：根据各位权重进行模拟量求和
3. **输出缓冲**：提供适当的驱动能力和阻抗匹配
4. **滤波**：去除高频分量，平滑输出波形

### 理想DAC特性

**传输函数**：
```
V_out = V_ref × (D / 2^n)
```
其中：
- V_out：输出电压
- V_ref：参考电压
- D：数字输入码（0到2^n-1）
- n：DAC位数

**分辨率**：
```
LSB = V_ref / 2^n
```

**满量程范围**：
```
FSR = V_ref × (2^n - 1) / 2^n ≈ V_ref
```

## DAC架构类型

### 1. R-2R梯形网络DAC

**工作原理**：
- 使用R和2R电阻构成梯形网络
- 每位对应一个开关
- 通过电阻分压实现权重转换

**优势**：
- 只需两种阻值电阻
- 结构简单，易于集成
- 功耗相对较低

**劣势**：
- 电阻匹配要求高
- 开关电阻影响精度
- 速度相对较慢

**STM32F4采用此架构**

### 2. 电流舵DAC

**工作原理**：
- 每位对应一个电流源
- 通过开关控制电流方向
- 电流求和后转换为电压

**优势**：
- 速度快
- 精度高
- 动态性能好

**劣势**：
- 电流源匹配要求极高
- 功耗较大
- 成本高

### 3. 电荷重分布DAC

**工作原理**：
- 使用电容阵列
- 通过电荷重分布实现转换
- 常用于SAR ADC中

**优势**：
- 功耗极低
- 易于CMOS工艺实现
- 精度较高

**劣势**：
- 速度较慢
- 驱动能力弱
- 温度敏感

### 4. Sigma-Delta DAC

**工作原理**：
- 使用过采样和噪声整形
- 低位DAC + 数字滤波器
- 高分辨率输出

**优势**：
- 分辨率极高（16-24位）
- 线性度好
- 抗干扰能力强

**劣势**：
- 延迟较大
- 复杂度高
- 功耗较大

## DAC性能参数

### 1. 静态参数

**分辨率（Resolution）**
- 定义：DAC能够产生的不同输出电平数
- 单位：位（bits）
- 影响：输出精度和平滑度

**积分非线性（INL）**
- 定义：实际传输特性与理想直线的最大偏差
- 单位：LSB
- 影响：输出精度

**差分非线性（DNL）**
- 定义：相邻码字对应输出差值与理想LSB的偏差
- 单位：LSB
- 影响：单调性

**偏移误差（Offset Error）**
- 定义：零码输入时的输出偏差
- 单位：LSB或mV
- 校正：软件或硬件补偿

**增益误差（Gain Error）**
- 定义：满量程输出与理想值的偏差
- 单位：%或LSB
- 校正：增益校准

**单调性（Monotonicity）**
- 定义：输出随输入单调变化的特性
- 要求：DNL > -1 LSB

### 2. 动态参数

**建立时间（Settling Time）**
- 定义：输出达到最终值±½LSB所需时间
- 影响：最大更新率
- 典型值：1-10μs

**压摆率（Slew Rate）**
- 定义：输出电压变化的最大速率
- 单位：V/μs
- 影响：大信号响应速度

**总谐波失真（THD）**
- 定义：谐波分量功率与基波功率的比值
- 单位：dB
- 影响：信号质量

**信噪比（SNR）**
- 定义：信号功率与噪声功率的比值
- 单位：dB
- 理论值：SNR = 6.02n + 1.76 dB

**无杂散动态范围（SFDR）**
- 定义：基波与最大杂散分量的功率比
- 单位：dB
- 影响：频谱纯度

**互调失真（IMD）**
- 定义：多音信号产生的互调产物
- 单位：dB
- 影响：多信号应用

### 3. 输出特性

**输出阻抗**
- 定义：DAC输出端的等效阻抗
- 影响：负载驱动能力
- STM32F4：约15kΩ

**输出电流**
- 定义：DAC能够提供的最大输出电流
- STM32F4：±20mA

**输出电压范围**
- 定义：DAC输出电压的有效范围
- STM32F4：0V到VREF+

## STM32F4 DAC特性

### 硬件特性

**基本规格**：
- 分辨率：12位（4096级）
- 通道数：2个独立通道
- 输出引脚：PA4（DAC1）、PA5（DAC2）
- 参考电压：VREF+（通常为VDD）
- 最大更新率：1MSPS

**架构特点**：
- R-2R梯形网络
- 内置输出缓冲器
- 可选外部触发
- 支持DMA传输

### 工作模式

**直接输出模式**：
- 直接写入数据寄存器
- 立即更新输出
- 适用于静态输出

**触发模式**：
- 外部触发更新输出
- 精确的时序控制
- 适用于波形生成

**DMA模式**：
- DMA自动传输数据
- 减少CPU负载
- 适用于连续波形

### 数据格式

**12位右对齐**：
```
15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 0  0  0  0 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
```

**12位左对齐**：
```
15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0  0  0  0  0
```

**8位右对齐**：
```
15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 0  0  0  0  0  0  0  0 D7 D6 D5 D4 D3 D2 D1 D0
```

## 触发系统

### 软件触发

**立即更新**：
```rust
// 直接写入DAC数据寄存器
dac.set_value(Channel1, 2048); // 输出约1.65V (3.3V/2)
```

### 硬件触发

**定时器触发源**：
| 触发源 | DAC通道1 | DAC通道2 |
|--------|----------|----------|
| TIM2_TRGO | ✓ | ✓ |
| TIM4_TRGO | ✓ | ✓ |
| TIM5_TRGO | ✓ | ✓ |
| TIM6_TRGO | ✓ | ✓ |
| TIM7_TRGO | ✓ | ✓ |
| TIM8_TRGO | ✓ | ✓ |

**外部触发**：
- EXTI线9
- 软件触发

### 触发配置示例

```rust
// 配置TIM6触发DAC
timer6.configure_trigger_output(TriggerOutput::Update);
dac.configure_trigger(Channel1, TriggerSource::Tim6);
dac.enable_trigger(Channel1);
```

## 波形生成

### 基本波形

**正弦波生成**：
```rust
// 生成正弦波查找表
const SINE_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let angle = 2.0 * PI * i as f32 / 256.0;
        let value = (2047.0 * (1.0 + angle.sin())) as u16;
        table[i] = value;
        i += 1;
    }
    table
};

// 使用DMA循环输出正弦波
dac.configure_dma_circular(&SINE_TABLE);
```

**三角波生成**：
```rust
// 生成三角波查找表
const TRIANGLE_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let value = if i < 128 {
            i * 32  // 上升沿
        } else {
            4095 - (i - 128) * 32  // 下降沿
        };
        table[i] = value as u16;
        i += 1;
    }
    table
};
```

**方波生成**：
```rust
// 生成方波查找表
const SQUARE_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let value = if i < 128 { 0 } else { 4095 };
        table[i] = value;
        i += 1;
    }
    table
};
```

### 复杂波形

**任意波形**：
```rust
// 从数学函数生成波形
fn generate_waveform<F>(func: F, samples: usize) -> Vec<u16>
where
    F: Fn(f32) -> f32,
{
    (0..samples)
        .map(|i| {
            let x = i as f32 / samples as f32;
            let y = func(x);
            ((y + 1.0) * 2047.5) as u16
        })
        .collect()
}

// 生成复合波形
let waveform = generate_waveform(|x| {
    0.5 * (2.0 * PI * x).sin() + 0.3 * (6.0 * PI * x).sin()
}, 1024);
```

**调制波形**：
```rust
// AM调制
fn am_modulation(carrier_freq: f32, mod_freq: f32, mod_depth: f32, samples: usize) -> Vec<u16> {
    (0..samples)
        .map(|i| {
            let t = i as f32 / samples as f32;
            let carrier = (2.0 * PI * carrier_freq * t).sin();
            let modulator = 1.0 + mod_depth * (2.0 * PI * mod_freq * t).sin();
            let signal = carrier * modulator;
            ((signal + 1.0) * 2047.5) as u16
        })
        .collect()
}

// FM调制
fn fm_modulation(carrier_freq: f32, mod_freq: f32, mod_index: f32, samples: usize) -> Vec<u16> {
    (0..samples)
        .map(|i| {
            let t = i as f32 / samples as f32;
            let phase = 2.0 * PI * carrier_freq * t + 
                       mod_index * (2.0 * PI * mod_freq * t).sin();
            let signal = phase.sin();
            ((signal + 1.0) * 2047.5) as u16
        })
        .collect()
}
```

## 频率和相位控制

### 频率控制

**基本公式**：
```
f_out = f_update / N
```
其中：
- f_out：输出频率
- f_update：DAC更新频率
- N：查找表长度

**频率精度**：
```rust
// 计算所需的更新频率
fn calculate_update_frequency(target_freq: f32, table_size: usize) -> f32 {
    target_freq * table_size as f32
}

// 配置定时器频率
fn configure_timer_frequency(timer: &mut Timer, freq: f32) {
    let period = (timer.clock_frequency() / freq) as u16;
    timer.set_period(period);
}
```

### 相位控制

**相位偏移**：
```rust
// 实现相位偏移
struct PhaseAccumulator {
    phase: u32,
    phase_increment: u32,
    table_size: usize,
}

impl PhaseAccumulator {
    fn new(frequency: f32, sample_rate: f32, table_size: usize) -> Self {
        let phase_increment = ((frequency / sample_rate) * (1u64 << 32) as f32) as u32;
        Self {
            phase: 0,
            phase_increment,
            table_size,
        }
    }
    
    fn next_sample(&mut self, table: &[u16]) -> u16 {
        let index = (self.phase >> (32 - self.table_size.trailing_zeros())) as usize;
        self.phase = self.phase.wrapping_add(self.phase_increment);
        table[index % self.table_size]
    }
}
```

**多通道相位关系**：
```rust
// 双通道正交信号生成
struct QuadratureGenerator {
    phase: u32,
    phase_increment: u32,
}

impl QuadratureGenerator {
    fn next_samples(&mut self, sine_table: &[u16]) -> (u16, u16) {
        let i_index = (self.phase >> 24) as usize;
        let q_index = ((self.phase + (1u32 << 30)) >> 24) as usize; // 90度相位差
        
        self.phase = self.phase.wrapping_add(self.phase_increment);
        
        (sine_table[i_index], sine_table[q_index])
    }
}
```

## 输出滤波

### 重建滤波器

**目的**：
- 去除高频分量
- 平滑阶梯波形
- 抑制镜像频率

**设计原则**：
```
f_cutoff ≤ f_sample / 2
```

**简单RC滤波器**：
```
R = 1 / (2π × f_cutoff × C)
```

### 抗混叠考虑

**奈奎斯特频率**：
```
f_nyquist = f_sample / 2
```

**滤波器要求**：
- 通带：0 到 f_signal
- 阻带：f_sample - f_signal 到 f_sample
- 过渡带：尽可能窄

## 性能优化

### 精度优化

**校准技术**：
```rust
// DAC校准
struct DacCalibration {
    offset: i16,
    gain: f32,
}

impl DacCalibration {
    fn apply(&self, value: u16) -> u16 {
        let corrected = (value as f32 * self.gain) as i32 + self.offset as i32;
        corrected.clamp(0, 4095) as u16
    }
}
```

**温度补偿**：
```rust
// 温度系数补偿
fn temperature_compensate(value: u16, temp: f32, temp_coeff: f32) -> u16 {
    let temp_error = (temp - 25.0) * temp_coeff;
    let compensated = value as f32 * (1.0 - temp_error);
    compensated.clamp(0.0, 4095.0) as u16
}
```

### 速度优化

**DMA优化**：
```rust
// 双缓冲DMA配置
let dma_config = DmaConfig::default()
    .memory_increment(true)
    .circular(true)
    .double_buffer(true)
    .transfer_complete_interrupt(true);
```

**查找表优化**：
```rust
// 使用更大的查找表提高精度
const HIGH_RES_SINE: [u16; 4096] = generate_sine_table(4096);

// 线性插值提高分辨率
fn interpolated_lookup(table: &[u16], phase: f32) -> u16 {
    let index = phase * table.len() as f32;
    let i0 = index as usize;
    let i1 = (i0 + 1) % table.len();
    let frac = index - i0 as f32;
    
    let v0 = table[i0] as f32;
    let v1 = table[i1] as f32;
    
    (v0 + frac * (v1 - v0)) as u16
}
```

## 应用实例

### 音频DAC

**音频特性**：
- 采样率：44.1kHz或48kHz
- 分辨率：16位或更高
- THD+N：< -80dB
- 动态范围：> 90dB

**实现要点**：
```rust
// 音频DAC配置
fn configure_audio_dac(dac: &mut Dac, sample_rate: u32) {
    // 配置定时器产生采样时钟
    let timer_freq = sample_rate;
    timer.configure_frequency(timer_freq);
    
    // 配置DAC触发
    dac.configure_trigger(Channel1, TriggerSource::Tim6);
    dac.enable_trigger(Channel1);
    
    // 配置DMA传输
    dac.configure_dma_circular(&audio_buffer);
}
```

### 控制信号生成

**PID控制输出**：
```rust
// PID控制器DAC输出
struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    last_error: f32,
}

impl PidController {
    fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> u16 {
        let error = setpoint - measurement;
        
        self.integral += error * dt;
        let derivative = (error - self.last_error) / dt;
        
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        self.last_error = error;
        
        // 转换为DAC值
        ((output + 1.0) * 2047.5).clamp(0.0, 4095.0) as u16
    }
}
```

### 测试信号生成

**扫频信号**：
```rust
// 线性扫频信号生成
struct ChirpGenerator {
    phase: f32,
    freq_start: f32,
    freq_end: f32,
    duration: f32,
    time: f32,
    sample_rate: f32,
}

impl ChirpGenerator {
    fn next_sample(&mut self) -> u16 {
        let freq = self.freq_start + 
                  (self.freq_end - self.freq_start) * (self.time / self.duration);
        
        let sample = (2.0 * PI * self.phase).sin();
        self.phase += freq / self.sample_rate;
        self.time += 1.0 / self.sample_rate;
        
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }
        
        ((sample + 1.0) * 2047.5) as u16
    }
}
```

## 调试和测试

### 静态测试

**线性度测试**：
```rust
// DAC线性度测试
fn test_dac_linearity(dac: &mut Dac) -> Vec<f32> {
    let mut results = Vec::new();
    
    for code in (0..4096).step_by(64) {
        dac.set_value(Channel1, code);
        delay_ms(10); // 等待建立
        
        let measured = measure_voltage(); // 外部测量
        let expected = 3.3 * code as f32 / 4095.0;
        let error = measured - expected;
        
        results.push(error);
    }
    
    results
}
```

**建立时间测试**：
```rust
// 建立时间测试
fn test_settling_time(dac: &mut Dac) -> f32 {
    dac.set_value(Channel1, 0);
    delay_ms(10);
    
    let start_time = get_time_us();
    dac.set_value(Channel1, 4095);
    
    // 监测输出直到稳定
    loop {
        let voltage = measure_voltage();
        if (voltage - 3.3).abs() < 0.001 { // 1mV精度
            return get_time_us() - start_time;
        }
    }
}
```

### 动态测试

**频率响应测试**：
```rust
// 频率响应测试
fn test_frequency_response(dac: &mut Dac, frequencies: &[f32]) -> Vec<f32> {
    let mut response = Vec::new();
    
    for &freq in frequencies {
        // 生成测试正弦波
        let sine_table = generate_sine_table(1024);
        let update_freq = freq * 1024.0;
        
        configure_timer_frequency(update_freq);
        dac.start_dma_circular(&sine_table);
        
        delay_ms(100); // 稳定时间
        
        let amplitude = measure_amplitude(); // 外部测量
        response.push(amplitude);
        
        dac.stop();
    }
    
    response
}
```

## 常见问题和解决方案

### 输出异常

**问题**：输出电压不正确
**解决**：
1. 检查参考电压
2. 验证数据格式
3. 确认输出缓冲器使能

**问题**：输出不稳定
**解决**：
1. 检查电源噪声
2. 添加去耦电容
3. 优化PCB布局

### 波形失真

**问题**：波形失真严重
**解决**：
1. 降低输出频率
2. 减小负载
3. 添加输出缓冲器

**问题**：高频响应差
**解决**：
1. 优化输出滤波器
2. 提高更新频率
3. 使用更大查找表

### 时序问题

**问题**：触发时序不准确
**解决**：
1. 检查定时器配置
2. 验证触发源
3. 测量实际频率

## 最佳实践

### 硬件设计

1. **电源设计**：
   - 使用低噪声电源
   - 分离模拟和数字电源
   - 添加适当的去耦电容

2. **输出电路**：
   - 添加输出缓冲器
   - 设计合适的滤波器
   - 考虑负载阻抗

3. **PCB布局**：
   - 最短信号路径
   - 避免数字噪声耦合
   - 使用地平面

### 软件设计

1. **配置优化**：
   - 选择合适的触发源
   - 优化DMA配置
   - 平衡精度和速度

2. **波形生成**：
   - 使用查找表
   - 实现相位累加器
   - 考虑内存使用

3. **校准策略**：
   - 实施偏移和增益校准
   - 考虑温度补偿
   - 定期校准

## 总结

DAC是嵌入式系统中重要的信号生成器件，STM32F4的12位DAC提供了良好的性能和灵活性。通过深入理解DAC的工作原理、性能参数和配置方法，可以实现高质量的模拟信号输出。

关键要点：
1. 理解DAC架构和性能参数
2. 合理配置触发和DMA
3. 优化波形生成算法
4. 实施校准和补偿技术
5. 注意硬件设计和信号完整性

## 参考资料

1. STM32F407xx Reference Manual - DAC章节
2. "DAC Architectures and Applications" - Analog Devices
3. "Understanding DACs" - Texas Instruments
4. "Digital Signal Processing" - Alan V. Oppenheim