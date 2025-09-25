# STM32F4 ADC架构详解

## 概述

STM32F4系列微控制器集成了高性能的12位逐次逼近型ADC（SAR ADC），提供了灵活的配置选项和强大的功能。本章详细介绍STM32F4 ADC的硬件架构、寄存器配置、以及编程实现。

## ADC硬件架构

### 整体架构

STM32F407VG包含3个独立的ADC实例：
- **ADC1**：主ADC，功能最全
- **ADC2**：可与ADC1同步工作
- **ADC3**：独立ADC，功能与ADC1相同

每个ADC具有：
- 12位分辨率
- 16个外部输入通道
- 3个内部通道（温度传感器、VREFINT、VBAT）
- 最高2.4MSPS转换速率

### 时钟系统

**ADC时钟源**：
```
ADC_CLK = PCLK2 / ADC_Prescaler
```

**预分频器选项**：
- /2：最高速度，最大84MHz
- /4：平衡选择，42MHz
- /6：较低速度，28MHz
- /8：最低速度，21MHz

**时钟配置示例**：
```rust
// 配置ADC时钟为PCLK2/4 = 21MHz (PCLK2=84MHz)
rcc.cfgr.adcpre().div4();
```

### 输入通道映射

**外部通道映射**（STM32F407VG）：
| 通道 | GPIO引脚 | 复用功能 |
|------|----------|----------|
| ADC123_IN0 | PA0 | ADC1/2/3通道0 |
| ADC123_IN1 | PA1 | ADC1/2/3通道1 |
| ADC123_IN2 | PA2 | ADC1/2/3通道2 |
| ADC123_IN3 | PA3 | ADC1/2/3通道3 |
| ADC12_IN4 | PA4 | ADC1/2通道4 |
| ADC12_IN5 | PA5 | ADC1/2通道5 |
| ADC12_IN6 | PA6 | ADC1/2通道6 |
| ADC12_IN7 | PA7 | ADC1/2通道7 |
| ADC12_IN8 | PB0 | ADC1/2通道8 |
| ADC12_IN9 | PB1 | ADC1/2通道9 |
| ADC123_IN10 | PC0 | ADC1/2/3通道10 |
| ADC123_IN11 | PC1 | ADC1/2/3通道11 |
| ADC123_IN12 | PC2 | ADC1/2/3通道12 |
| ADC123_IN13 | PC3 | ADC1/2/3通道13 |
| ADC12_IN14 | PC4 | ADC1/2通道14 |
| ADC12_IN15 | PC5 | ADC1/2通道15 |

**内部通道**：
- 通道16：温度传感器（仅ADC1）
- 通道17：VREFINT（仅ADC1）
- 通道18：VBAT/2（仅ADC1）

## 转换模式

### 1. 单次转换模式

**特点**：
- 转换完成后自动停止
- 适用于偶发性测量
- 功耗最低

**配置**：
```rust
adc.configure_single_conversion();
let value = adc.read(&mut pin).unwrap();
```

### 2. 连续转换模式

**特点**：
- 转换完成后自动开始下一次转换
- 适用于连续监测
- 可配合DMA使用

**配置**：
```rust
adc.configure_continuous();
adc.start_conversion();
```

### 3. 扫描模式

**特点**：
- 自动扫描预设的通道序列
- 可配置转换顺序和通道数
- 结果存储在数据寄存器中

**配置示例**：
```rust
// 配置扫描序列：通道0 -> 通道1 -> 通道2
adc.configure_scan_mode(&[
    (Channel::C0, SampleTime::Cycles_480),
    (Channel::C1, SampleTime::Cycles_480),
    (Channel::C2, SampleTime::Cycles_480),
]);
```

### 4. 注入模式

**特点**：
- 高优先级转换
- 可中断常规转换序列
- 最多4个注入通道
- 独立的数据寄存器

**应用场景**：
- 紧急监测（过压保护）
- 实时控制反馈
- 高优先级采样

## 触发系统

### 软件触发

**常规转换**：
```rust
// 启动软件触发的转换
adc_cr2.swstart().set_bit();
```

**注入转换**：
```rust
// 启动注入转换
adc_cr2.jswstart().set_bit();
```

### 硬件触发

**定时器触发源**：
| 触发源 | ADC1/2 | ADC3 |
|--------|--------|------|
| TIM1_CC1 | ✓ | ✓ |
| TIM1_CC2 | ✓ | ✓ |
| TIM1_CC3 | ✓ | ✓ |
| TIM2_CC2 | ✓ | ✗ |
| TIM2_CC3 | ✓ | ✗ |
| TIM2_CC4 | ✓ | ✗ |
| TIM2_TRGO | ✓ | ✗ |
| TIM3_CC1 | ✓ | ✗ |
| TIM3_TRGO | ✓ | ✓ |
| TIM4_CC4 | ✓ | ✗ |
| TIM5_CC1 | ✓ | ✗ |
| TIM5_CC2 | ✓ | ✗ |
| TIM5_CC3 | ✓ | ✗ |

**外部触发**：
- EXTI线11（ADC1/2/3）
- 上升沿、下降沿或双沿触发

### 触发配置示例

```rust
// 配置TIM2 TRGO触发ADC
timer2.configure_trigger_output(TriggerOutput::Update);
adc.configure_external_trigger(
    ExternalTrigger::Tim2Trgo,
    TriggerEdge::Rising
);
```

## 数据管理

### 数据寄存器

**常规转换**：
- ADC_DR：16位数据寄存器
- 右对齐或左对齐
- 12位有效数据

**注入转换**：
- ADC_JDR1-4：4个独立的注入数据寄存器
- 16位寄存器，支持偏移补偿

### 数据对齐

**右对齐**（默认）：
```
15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 0  0  0  0 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
```

**左对齐**：
```
15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0  0  0  0  0
```

### DMA集成

**DMA配置**：
```rust
// 配置DMA传输ADC数据
let dma_config = DmaConfig::default()
    .memory_increment(true)
    .peripheral_increment(false)
    .circular(true)
    .transfer_complete_interrupt(true);

let mut transfer = Transfer::init_peripheral_to_memory(
    dma_stream,
    adc,
    buffer,
    None,
    dma_config,
);
```

**双缓冲模式**：
```rust
// 配置双缓冲DMA
let mut transfer = Transfer::init_peripheral_to_memory_double_buffer(
    dma_stream,
    adc,
    buffer1,
    buffer2,
    None,
    dma_config,
);
```

## 采样时间配置

### 采样时间选项

STM32F4提供8种采样时间：
```rust
pub enum SampleTime {
    Cycles_3,    // 3个ADC时钟周期
    Cycles_15,   // 15个ADC时钟周期
    Cycles_28,   // 28个ADC时钟周期
    Cycles_56,   // 56个ADC时钟周期
    Cycles_84,   // 84个ADC时钟周期
    Cycles_112,  // 112个ADC时钟周期
    Cycles_144,  // 144个ADC时钟周期
    Cycles_480,  // 480个ADC时钟周期
}
```

### 采样时间计算

**总转换时间**：
```
T_conv = T_sample + T_sar
T_sample = 采样周期数 / f_ADC
T_sar = 12 / f_ADC  // SAR转换时间固定12个周期
```

**示例计算**（f_ADC = 21MHz）：
- 最快配置：(3 + 12) / 21MHz = 0.71μs
- 最慢配置：(480 + 12) / 21MHz = 23.4μs

### 采样时间选择原则

**源阻抗考虑**：
```
T_sample ≥ 10 × Rs × Cs
```
其中：
- Rs：源阻抗
- Cs：采样电容（约8pF）

**精度要求**：
- 高精度：使用较长采样时间
- 高速度：使用较短采样时间
- 平衡选择：Cycles_84或Cycles_112

## 多ADC同步

### 同步模式

**独立模式**：
- 每个ADC独立工作
- 最大灵活性

**双重模式**：
- ADC1和ADC2同步工作
- 提高采样率或实现差分采样

**三重模式**：
- ADC1、ADC2、ADC3同步工作
- 最高采样率

### 同步配置

**同时采样**：
```rust
// 配置ADC1和ADC2同时采样
adc_ccr.multi().bits(0b00110); // 同时常规模式
adc_ccr.delay().bits(0);       // 无延迟
```

**交替采样**：
```rust
// 配置交替采样提高采样率
adc_ccr.multi().bits(0b01001); // 交替触发模式
adc_ccr.delay().bits(5);       // 5个时钟延迟
```

## 中断系统

### 中断源

**转换完成中断**：
- EOC：单次转换完成
- EOS：序列转换完成
- JEOC：注入转换完成
- JEOS：注入序列完成

**错误中断**：
- OVR：数据覆盖
- AWD：模拟看门狗

### 中断配置

```rust
// 使能转换完成中断
adc.listen(Event::EndOfConversion);

// 中断处理函数
#[interrupt]
fn ADC() {
    cortex_m::interrupt::free(|cs| {
        if let Some(adc) = ADC1.borrow(cs).borrow_mut().as_mut() {
            if adc.is_conversion_complete() {
                let value = adc.current_sample();
                // 处理ADC数据
                adc.clear_end_of_conversion_flag();
            }
        }
    });
}
```

## 模拟看门狗

### 功能特点

**监测范围**：
- 上阈值和下阈值
- 单通道或所有通道
- 常规转换或注入转换

**应用场景**：
- 过压/欠压保护
- 温度监测
- 信号范围检查

### 配置示例

```rust
// 配置模拟看门狗
adc.configure_analog_watchdog(
    AnalogWatchdog::Single(Channel::C0), // 监测通道0
    0x800,  // 下阈值
    0xC00,  // 上阈值
);

adc.listen(Event::AnalogWatchdog);
```

## 温度传感器

### 特性参数

**温度系数**：
- 典型值：2.5mV/°C
- 25°C时电压：约760mV
- 线性范围：-40°C到+125°C

### 温度计算

```rust
// 温度传感器读取和计算
fn read_temperature(adc: &mut Adc<ADC1>) -> f32 {
    // 使能温度传感器
    adc.enable_temperature_sensor();
    
    // 读取温度传感器值
    let temp_raw = adc.read_temperature();
    
    // 转换为电压（mV）
    let temp_voltage = (temp_raw as f32 * 3300.0) / 4095.0;
    
    // 计算温度（°C）
    let temperature = (temp_voltage - 760.0) / 2.5 + 25.0;
    
    temperature
}
```

## 内部参考电压

### VREFINT特性

**参数**：
- 典型值：1.21V
- 精度：±10mV
- 温度系数：±50ppm/°C
- 负载调整率：±0.2mV/mA

### 电源电压测量

```rust
// 使用VREFINT测量VDD
fn measure_vdd(adc: &mut Adc<ADC1>) -> f32 {
    // 读取VREFINT
    let vrefint_raw = adc.read_vref();
    
    // 计算VDD电压
    let vdd = (1.21 * 4095.0) / vrefint_raw as f32;
    
    vdd
}
```

## 性能优化

### 速度优化

**减少转换时间**：
1. 使用最高ADC时钟
2. 选择最短采样时间
3. 优化源阻抗

**并行处理**：
1. 使用多ADC同步
2. DMA传输减少CPU负载
3. 中断驱动处理

### 精度优化

**噪声抑制**：
1. 使用较长采样时间
2. 多次采样平均
3. 数字滤波

**校准技术**：
1. 偏移校准
2. 增益校准
3. 温度补偿

### 功耗优化

**低功耗模式**：
1. 按需使能ADC
2. 降低ADC时钟
3. 使用停止模式

## 调试技术

### 常见问题

**转换结果异常**：
1. 检查参考电压
2. 验证采样时间
3. 确认输入范围

**采样率不足**：
1. 优化ADC时钟配置
2. 减少采样时间
3. 使用DMA传输

**精度不够**：
1. 增加采样时间
2. 实施校准
3. 改善硬件设计

### 调试工具

**寄存器查看**：
```rust
// 调试ADC寄存器状态
fn debug_adc_registers(adc: &Adc<ADC1>) {
    let cr1 = adc.cr1.read();
    let cr2 = adc.cr2.read();
    let sr = adc.sr.read();
    
    rtt_println!("ADC CR1: 0x{:08X}", cr1.bits());
    rtt_println!("ADC CR2: 0x{:08X}", cr2.bits());
    rtt_println!("ADC SR: 0x{:08X}", sr.bits());
}
```

**性能测量**：
```rust
// 测量转换时间
let start = systick.now();
let value = adc.read(&mut pin).unwrap();
let duration = systick.now() - start;
rtt_println!("Conversion time: {} us", duration);
```

## 最佳实践

### 硬件设计

1. **电源设计**：
   - 使用低噪声LDO
   - 添加去耦电容
   - 分离模拟和数字电源

2. **PCB布局**：
   - 模拟信号远离数字信号
   - 使用地平面
   - 最短信号路径

3. **信号调理**：
   - 抗混叠滤波器
   - 阻抗匹配
   - 信号缓冲

### 软件设计

1. **配置优化**：
   - 根据应用选择合适参数
   - 平衡速度和精度
   - 考虑功耗要求

2. **数据处理**：
   - 实时处理vs批处理
   - 内存使用优化
   - 错误处理

3. **校准策略**：
   - 定期校准
   - 温度补偿
   - 非线性校正

## 总结

STM32F4的ADC系统提供了强大而灵活的模拟信号采集能力。通过深入理解其硬件架构和软件配置，可以充分发挥其性能优势，实现高质量的数据采集系统。

关键要点：
1. 合理配置时钟和采样时间
2. 选择适当的转换模式和触发方式
3. 利用DMA提高数据传输效率
4. 实施校准和噪声抑制技术
5. 优化硬件设计和软件算法

## 参考资料

1. STM32F407xx Reference Manual - RM0090
2. STM32F4 ADC Application Note - AN3116
3. "Improving ADC Resolution by Oversampling" - AN2668
4. STM32F4xx HAL Driver Documentation