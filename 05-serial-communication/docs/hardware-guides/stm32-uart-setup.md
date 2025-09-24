# STM32 UART 硬件设置指南

## 概述

本指南详细介绍了STM32微控制器UART外设的硬件配置、引脚映射、时钟设置和电气特性。涵盖STM32F4、STM32F7、STM32H7等主流系列。

## STM32 UART外设概览

### 外设类型对比

| 外设类型 | 特性 | 数量 | 主要用途 |
|---------|------|------|----------|
| USART | 同步/异步，硬件流控制 | 1-6个 | 通用串口通信 |
| UART | 仅异步，简化版本 | 2-8个 | 简单串口通信 |
| LPUART | 低功耗，支持唤醒 | 1个 | 低功耗应用 |

### STM32F4系列UART资源

#### STM32F407/F417
```
USART1: APB2总线, 最高42MHz
USART2: APB1总线, 最高21MHz  
USART3: APB1总线, 最高21MHz
UART4:  APB1总线, 最高21MHz
UART5:  APB1总线, 最高21MHz
USART6: APB2总线, 最高42MHz
```

#### 时钟配置
```rust
// STM32F4xx HAL时钟配置示例
use stm32f4xx_hal::{pac, prelude::*, rcc::RccExt};

fn configure_clocks(dp: pac::Peripherals) -> pac::Peripherals {
    let rcc = dp.RCC.constrain();
    
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())           // 外部晶振8MHz
        .sysclk(168.MHz())          // 系统时钟168MHz
        .pclk1(42.MHz())            // APB1时钟42MHz
        .pclk2(84.MHz())            // APB2时钟84MHz
        .freeze();
    
    dp
}
```

## 引脚配置和复用

### STM32F407引脚映射

#### USART1引脚选择
```rust
// 选项1: PA9(TX), PA10(RX)
let tx_pin = gpioa.pa9.into_alternate::<7>();
let rx_pin = gpioa.pa10.into_alternate::<7>();

// 选项2: PB6(TX), PB7(RX)  
let tx_pin = gpiob.pb6.into_alternate::<7>();
let rx_pin = gpiob.pb7.into_alternate::<7>();
```

#### USART2引脚选择
```rust
// 选项1: PA2(TX), PA3(RX)
let tx_pin = gpioa.pa2.into_alternate::<7>();
let rx_pin = gpioa.pa3.into_alternate::<7>();

// 选项2: PD5(TX), PD6(RX)
let tx_pin = gpiod.pd5.into_alternate::<7>();
let rx_pin = gpiod.pd6.into_alternate::<7>();
```

#### 完整引脚映射表
```rust
pub struct UartPinMap {
    pub usart1: [(u8, u8); 2],  // [(TX_PIN, RX_PIN), ...]
    pub usart2: [(u8, u8); 2],
    pub usart3: [(u8, u8); 2],
    pub uart4: [(u8, u8); 2],
    pub uart5: [(u8, u8); 2],
    pub usart6: [(u8, u8); 2],
}

impl Default for UartPinMap {
    fn default() -> Self {
        Self {
            usart1: [(9, 10), (6, 7)],    // PA9/PA10, PB6/PB7
            usart2: [(2, 3), (5, 6)],     // PA2/PA3, PD5/PD6
            usart3: [(10, 11), (8, 9)],   // PB10/PB11, PC8/PC9
            uart4: [(0, 1), (10, 11)],    // PA0/PA1, PC10/PC11
            uart5: [(12, 2), (12, 2)],    // PC12/PD2
            usart6: [(6, 7), (14, 9)],    // PC6/PC7, PG14/PG9
        }
    }
}
```

### GPIO配置详解

#### 基本GPIO设置
```rust
use stm32f4xx_hal::{
    gpio::{Alternate, Pin, PushPull, Speed},
    pac::gpioa::MODER,
};

// UART TX引脚配置
fn configure_tx_pin<const P: char, const N: u8>(
    pin: Pin<P, N, Alternate<7, PushPull>>
) -> Pin<P, N, Alternate<7, PushPull>> {
    pin.set_speed(Speed::VeryHigh)  // 设置高速模式
}

// UART RX引脚配置  
fn configure_rx_pin<const P: char, const N: u8>(
    pin: Pin<P, N, Alternate<7>>
) -> Pin<P, N, Alternate<7>> {
    pin.internal_pull_up(true)      // 启用内部上拉
}
```

#### 硬件流控制引脚
```rust
// RTS/CTS流控制配置
struct FlowControlPins {
    rts: Pin<'A', 12, Alternate<7, PushPull>>,  // PA12
    cts: Pin<'A', 11, Alternate<7>>,            // PA11
}

impl FlowControlPins {
    pub fn new(gpioa: &mut GPIOA) -> Self {
        Self {
            rts: gpioa.pa12.into_alternate::<7>(),
            cts: gpioa.pa11.into_alternate::<7>(),
        }
    }
}
```

## 电气特性和信号完整性

### 电压电平
```rust
pub struct UartElectricalSpec {
    pub vdd_min: f32,           // 最小供电电压
    pub vdd_max: f32,           // 最大供电电压
    pub vil_max: f32,           // 输入低电平最大值
    pub vih_min: f32,           // 输入高电平最小值
    pub vol_max: f32,           // 输出低电平最大值
    pub voh_min: f32,           // 输出高电平最小值
    pub iil_max: f32,           // 输入漏电流最大值
    pub ioh_max: f32,           // 输出驱动电流最大值
}

impl Default for UartElectricalSpec {
    fn default() -> Self {
        Self {
            vdd_min: 1.8,           // 1.8V
            vdd_max: 3.6,           // 3.6V
            vil_max: 0.3 * 3.3,     // 0.3*VDD
            vih_min: 0.7 * 3.3,     // 0.7*VDD
            vol_max: 0.4,           // 0.4V
            voh_min: 2.4,           // 2.4V
            iil_max: 1e-6,          // 1μA
            ioh_max: 8e-3,          // 8mA
        }
    }
}
```

### 信号时序参数
```rust
pub struct UartTimingSpec {
    pub setup_time_ns: u32,     // 建立时间
    pub hold_time_ns: u32,      // 保持时间
    pub propagation_delay_ns: u32, // 传播延迟
    pub rise_time_ns: u32,      // 上升时间
    pub fall_time_ns: u32,      // 下降时间
}

impl UartTimingSpec {
    pub fn for_baud_rate(baud_rate: u32) -> Self {
        let bit_time_ns = 1_000_000_000 / baud_rate;
        
        Self {
            setup_time_ns: bit_time_ns / 10,        // 10%位时间
            hold_time_ns: bit_time_ns / 10,         // 10%位时间
            propagation_delay_ns: bit_time_ns / 20, // 5%位时间
            rise_time_ns: bit_time_ns / 20,         // 5%位时间
            fall_time_ns: bit_time_ns / 20,         // 5%位时间
        }
    }
}
```

## 波特率配置和时钟计算

### 波特率发生器
```rust
pub struct BaudRateGenerator {
    pub peripheral_clock: u32,  // 外设时钟频率
    pub oversampling: u8,       // 过采样率 (8 or 16)
}

impl BaudRateGenerator {
    pub fn calculate_brr(&self, baud_rate: u32) -> Result<u16, BaudRateError> {
        let usartdiv = (self.peripheral_clock * 25) / (4 * baud_rate);
        
        if self.oversampling == 16 {
            let mantissa = usartdiv / 100;
            let fraction = ((usartdiv - mantissa * 100) * 16 + 50) / 100;
            
            if mantissa > 0xFFF || fraction > 0xF {
                return Err(BaudRateError::OutOfRange);
            }
            
            Ok(((mantissa << 4) | fraction) as u16)
        } else {
            // 8倍过采样
            let mantissa = usartdiv / 100;
            let fraction = ((usartdiv - mantissa * 100) * 8 + 50) / 100;
            
            if mantissa > 0xFFF || fraction > 0x7 {
                return Err(BaudRateError::OutOfRange);
            }
            
            Ok(((mantissa << 4) | fraction) as u16)
        }
    }
    
    pub fn calculate_actual_baud_rate(&self, brr: u16) -> u32 {
        let mantissa = (brr >> 4) & 0xFFF;
        let fraction = brr & 0xF;
        
        if self.oversampling == 16 {
            let usartdiv = mantissa * 100 + (fraction * 100) / 16;
            (self.peripheral_clock * 25) / (4 * usartdiv)
        } else {
            let usartdiv = mantissa * 100 + (fraction * 100) / 8;
            (self.peripheral_clock * 25) / (4 * usartdiv)
        }
    }
    
    pub fn calculate_error_percentage(&self, target_baud: u32, brr: u16) -> f32 {
        let actual_baud = self.calculate_actual_baud_rate(brr);
        let error = (actual_baud as i32 - target_baud as i32).abs() as f32;
        (error / target_baud as f32) * 100.0
    }
}
```

### 常用波特率配置表
```rust
pub struct StandardBaudRates;

impl StandardBaudRates {
    pub const RATES: &'static [u32] = &[
        1200, 2400, 4800, 9600, 14400, 19200, 28800, 
        38400, 57600, 115200, 230400, 460800, 921600
    ];
    
    pub fn get_brr_table(pclk: u32) -> Vec<(u32, u16, f32)> {
        let generator = BaudRateGenerator {
            peripheral_clock: pclk,
            oversampling: 16,
        };
        
        Self::RATES.iter().map(|&baud| {
            let brr = generator.calculate_brr(baud).unwrap_or(0);
            let error = generator.calculate_error_percentage(baud, brr);
            (baud, brr, error)
        }).collect()
    }
}
```

## DMA配置

### DMA控制器映射
```rust
pub struct UartDmaMapping {
    pub usart1_tx: (u8, u8),    // (DMA控制器, 流)
    pub usart1_rx: (u8, u8),
    pub usart2_tx: (u8, u8),
    pub usart2_rx: (u8, u8),
    // ... 其他UART
}

impl Default for UartDmaMapping {
    fn default() -> Self {
        Self {
            usart1_tx: (2, 7),      // DMA2 Stream7
            usart1_rx: (2, 2),      // DMA2 Stream2
            usart2_tx: (1, 6),      // DMA1 Stream6
            usart2_rx: (1, 5),      // DMA1 Stream5
            // STM32F4参考手册表43
        }
    }
}
```

### DMA配置示例
```rust
use stm32f4xx_hal::{
    dma::{config::DmaConfig, PeripheralToMemory, Stream2, StreamsTuple, Transfer},
    pac::{DMA2, USART1},
};

pub struct UartDmaConfig {
    pub memory_increment: bool,
    pub peripheral_increment: bool,
    pub circular: bool,
    pub double_buffer: bool,
    pub priority: Priority,
    pub memory_data_size: DataSize,
    pub peripheral_data_size: DataSize,
}

impl Default for UartDmaConfig {
    fn default() -> Self {
        Self {
            memory_increment: true,
            peripheral_increment: false,
            circular: false,
            double_buffer: false,
            priority: Priority::Medium,
            memory_data_size: DataSize::Byte,
            peripheral_data_size: DataSize::Byte,
        }
    }
}

fn setup_uart_dma_rx(
    stream: Stream2<DMA2>,
    usart: USART1,
    buffer: &'static mut [u8; 1024],
) -> Transfer<Stream2<DMA2>, 4, USART1, PeripheralToMemory, &'static mut [u8]> {
    let config = DmaConfig::default()
        .memory_increment(true)
        .peripheral_increment(false)
        .circular(true);
    
    Transfer::init_peripheral_to_memory(stream, usart, buffer, None, config)
}
```

## 中断配置

### 中断向量表
```rust
pub struct UartInterruptVectors {
    pub usart1: u8,
    pub usart2: u8,
    pub usart3: u8,
    pub uart4: u8,
    pub uart5: u8,
    pub usart6: u8,
}

impl Default for UartInterruptVectors {
    fn default() -> Self {
        Self {
            usart1: 37,     // USART1全局中断
            usart2: 38,     // USART2全局中断
            usart3: 39,     // USART3全局中断
            uart4: 52,      // UART4全局中断
            uart5: 53,      // UART5全局中断
            usart6: 71,     // USART6全局中断
        }
    }
}
```

### 中断优先级配置
```rust
use cortex_m::peripheral::NVIC;
use stm32f4xx_hal::pac::interrupt;

pub struct InterruptPriorities {
    pub uart_rx: u8,        // 接收中断优先级
    pub uart_tx: u8,        // 发送中断优先级
    pub uart_error: u8,     // 错误中断优先级
    pub dma: u8,           // DMA中断优先级
}

impl Default for InterruptPriorities {
    fn default() -> Self {
        Self {
            uart_rx: 1,     // 高优先级
            uart_tx: 2,     // 中等优先级
            uart_error: 0,  // 最高优先级
            dma: 1,        // 高优先级
        }
    }
}

fn configure_uart_interrupts(nvic: &mut NVIC, priorities: &InterruptPriorities) {
    unsafe {
        nvic.set_priority(interrupt::USART1, priorities.uart_rx);
        NVIC::unmask(interrupt::USART1);
        
        nvic.set_priority(interrupt::DMA2_STREAM2, priorities.dma);
        NVIC::unmask(interrupt::DMA2_STREAM2);
    }
}
```

## 硬件调试和测试

### 信号质量测试
```rust
pub struct SignalQualityTest {
    pub test_pattern: Vec<u8>,
    pub baud_rate: u32,
    pub test_duration_ms: u32,
}

impl SignalQualityTest {
    pub fn new_prbs7() -> Self {
        // PRBS7伪随机序列
        let pattern = vec![
            0x7F, 0x3F, 0x9F, 0xCF, 0x67, 0xB3, 0xD9, 0xEC,
            0x76, 0x3B, 0x9D, 0xCE, 0x67, 0x33, 0x99, 0xCC,
        ];
        
        Self {
            test_pattern: pattern,
            baud_rate: 115200,
            test_duration_ms: 10000,
        }
    }
    
    pub fn calculate_bit_error_rate(&self, received: &[u8]) -> f32 {
        let mut errors = 0;
        let mut total_bits = 0;
        
        for (sent, recv) in self.test_pattern.iter().zip(received.iter()) {
            let xor_result = sent ^ recv;
            errors += xor_result.count_ones();
            total_bits += 8;
        }
        
        errors as f32 / total_bits as f32
    }
}
```

### 示波器测量点
```rust
pub struct OscilloscopeProbes {
    pub tx_signal: &'static str,       // TX信号测量点
    pub rx_signal: &'static str,       // RX信号测量点
    pub clock_signal: &'static str,    // 时钟信号
    pub ground: &'static str,          // 接地点
}

impl Default for OscilloscopeProbes {
    fn default() -> Self {
        Self {
            tx_signal: "PA9 (USART1_TX)",
            rx_signal: "PA10 (USART1_RX)",
            clock_signal: "PA8 (MCO1)",
            ground: "VSS",
        }
    }
}
```

## 电磁兼容性(EMC)

### PCB布线指南
```rust
pub struct PcbLayoutGuidelines {
    pub trace_impedance: f32,       // 走线阻抗
    pub differential_impedance: f32, // 差分阻抗
    pub via_size: f32,              // 过孔尺寸
    pub ground_plane: bool,         // 地平面
    pub trace_length_max: f32,      // 最大走线长度
}

impl Default for PcbLayoutGuidelines {
    fn default() -> Self {
        Self {
            trace_impedance: 50.0,      // 50欧姆
            differential_impedance: 100.0, // 100欧姆
            via_size: 0.2,              // 0.2mm
            ground_plane: true,
            trace_length_max: 100.0,    // 100mm
        }
    }
}
```

### EMI滤波器设计
```rust
pub struct EmiFilter {
    pub common_mode_choke: f32,     // 共模电感值(μH)
    pub bypass_capacitor: f32,      // 旁路电容值(nF)
    pub ferrite_bead: String,       // 磁珠型号
}

impl Default for EmiFilter {
    fn default() -> Self {
        Self {
            common_mode_choke: 10.0,    // 10μH
            bypass_capacitor: 100.0,    // 100nF
            ferrite_bead: "BLM18PG221SN1D".to_string(),
        }
    }
}
```

## 故障诊断

### 常见硬件问题
```rust
#[derive(Debug)]
pub enum HardwareFault {
    ClockNotConfigured,
    PinNotMapped,
    IncorrectVoltageLevel,
    GroundLoop,
    SignalReflection,
    PowerSupplyNoise,
    TemperatureDrift,
}

pub struct DiagnosticTest {
    pub fault_type: HardwareFault,
    pub test_procedure: &'static str,
    pub expected_result: &'static str,
    pub corrective_action: &'static str,
}

impl DiagnosticTest {
    pub fn get_tests() -> Vec<Self> {
        vec![
            Self {
                fault_type: HardwareFault::ClockNotConfigured,
                test_procedure: "检查RCC寄存器UART时钟使能位",
                expected_result: "对应位应为1",
                corrective_action: "调用RCC使能函数",
            },
            Self {
                fault_type: HardwareFault::PinNotMapped,
                test_procedure: "检查GPIO复用功能寄存器",
                expected_result: "AFR寄存器应配置为AF7",
                corrective_action: "重新配置GPIO复用功能",
            },
            // ... 更多测试
        ]
    }
}
```

### 自动化测试脚本
```rust
pub struct HardwareTestSuite {
    pub loopback_test: bool,
    pub baud_rate_test: bool,
    pub interrupt_test: bool,
    pub dma_test: bool,
}

impl HardwareTestSuite {
    pub fn run_all_tests(&self) -> TestResults {
        let mut results = TestResults::new();
        
        if self.loopback_test {
            results.loopback = self.run_loopback_test();
        }
        
        if self.baud_rate_test {
            results.baud_rate = self.run_baud_rate_test();
        }
        
        if self.interrupt_test {
            results.interrupt = self.run_interrupt_test();
        }
        
        if self.dma_test {
            results.dma = self.run_dma_test();
        }
        
        results
    }
    
    fn run_loopback_test(&self) -> TestResult {
        // 实现环回测试
        TestResult::Pass
    }
}
```

## 参考资料

### 官方文档
1. **STM32F4xx Reference Manual (RM0090)**
2. **STM32F4xx Datasheet**
3. **AN4776: General-purpose timer cookbook for STM32 microcontrollers**
4. **AN2606: STM32 microcontroller system memory boot mode**

### 应用笔记
1. **AN3070: Managing the Driver Enable signal for RS-485 and IO-Link communications with the STM32F05x USART**
2. **AN4655: Virtually increasing the number of serial communication peripherals in STM32 applications**
3. **AN4989: STM32 microcontroller debug limitations with UART/USART and low-power modes**

### 工具和软件
1. **STM32CubeMX**: 图形化配置工具
2. **STM32CubeIDE**: 集成开发环境
3. **STM32CubeProgrammer**: 编程工具
4. **STM32CubeMonitor**: 实时监控工具

## 版本历史

- **v1.0** (2024-01): 初始版本，基础硬件配置
- **v1.1** (2024-02): 添加DMA和中断配置
- **v1.2** (2024-03): 增加EMC和故障诊断
- **v1.3** (2024-04): 完善测试和验证方法