# STM32 UART硬件特性

## 概述

STM32微控制器系列提供了功能强大的UART/USART外设，支持多种通信模式和高级特性。本文档详细介绍STM32 UART硬件架构、配置方法和性能特性。

## STM32 UART/USART概述

### UART vs USART

STM32提供两种类型的串行通信外设：

#### UART（Universal Asynchronous Receiver Transmitter）
- 仅支持异步通信
- 不提供时钟输出
- 功能相对简单

#### USART（Universal Synchronous/Asynchronous Receiver Transmitter）
- 支持同步和异步通信
- 可提供时钟输出
- 功能更加丰富

### STM32F4系列UART/USART资源

以STM32F407VG为例：
- **USART1, USART6**：高速USART，最大波特率10.5Mbps
- **USART2, USART3**：标准USART，最大波特率5.25Mbps
- **UART4, UART5**：仅UART功能，最大波特率5.25Mbps

## 硬件架构

### 功能框图

```
                    STM32 UART/USART 架构
    ┌─────────────────────────────────────────────────────────┐
    │                   控制逻辑                                │
    ├─────────────────────────────────────────────────────────┤
    │  发送器                    │           接收器              │
    │  ┌─────────┐              │         ┌─────────┐          │
    │  │ TDR     │──→ TSR ──→ TX│         │ RSR ──→ │ RDR     │
    │  └─────────┘              │         └─────────┘          │
    ├─────────────────────────────────────────────────────────┤
    │                 波特率生成器                              │
    │  ┌─────────────────────────────────────────────────────┐ │
    │  │ 分频器 ← PCLK1/PCLK2                               │ │
    │  └─────────────────────────────────────────────────────┘ │
    ├─────────────────────────────────────────────────────────┤
    │                 DMA接口                                 │
    └─────────────────────────────────────────────────────────┘
```

### 主要组件详解

#### 1. 发送器（Transmitter）
- **TDR（Transmit Data Register）**：发送数据寄存器
- **TSR（Transmit Shift Register）**：发送移位寄存器
- **发送控制逻辑**：管理数据传输流程

#### 2. 接收器（Receiver）
- **RDR（Receive Data Register）**：接收数据寄存器
- **RSR（Receive Shift Register）**：接收移位寄存器
- **接收控制逻辑**：管理数据接收流程

#### 3. 波特率生成器
- 基于APB时钟（PCLK1或PCLK2）
- 可编程分频器
- 支持分数分频以获得精确波特率

## 引脚配置

### GPIO复用功能

STM32 UART引脚需要配置为复用功能模式：

#### STM32F407VG UART引脚映射

| UART/USART | TX引脚 | RX引脚 | CTS引脚 | RTS引脚 | CK引脚 |
|------------|--------|--------|---------|---------|--------|
| USART1     | PA9    | PA10   | PA11    | PA12    | PA8    |
| USART2     | PA2    | PA3    | PA0     | PA1     | PA4    |
| USART3     | PB10   | PB11   | PB13    | PB14    | PB12   |
| UART4      | PA0    | PA1    | -       | -       | -      |
| UART5      | PC12   | PD2    | -       | -       | -      |
| USART6     | PC6    | PC7    | PG13    | PG12    | PG7    |

### GPIO配置示例

```rust
// 配置USART1引脚（PA9-TX, PA10-RX）
use stm32f4xx_hal::{gpio::*, pac::GPIOA};

// 获取GPIO端口
let gpioa = dp.GPIOA.split();

// 配置TX引脚（PA9）
let tx_pin = gpioa.pa9.into_alternate::<7>();

// 配置RX引脚（PA10）
let rx_pin = gpioa.pa10.into_alternate::<7>();
```

## 时钟配置

### 时钟源

不同UART外设使用不同的时钟源：

#### STM32F4系列时钟分配
- **USART1, USART6**：APB2时钟（PCLK2）
- **USART2, USART3, UART4, UART5**：APB1时钟（PCLK1）

### 波特率计算

STM32使用以下公式计算波特率：

```
波特率 = fCK / (8 × (2 - OVER8) × USARTDIV)

其中：
- fCK：UART时钟频率（PCLK1或PCLK2）
- OVER8：过采样模式（0=16倍过采样，1=8倍过采样）
- USARTDIV：分频因子
```

#### 分频因子计算

```rust
// 计算USARTDIV
fn calculate_usartdiv(pclk: u32, baudrate: u32, over8: bool) -> u16 {
    let divisor = if over8 { 8 } else { 16 };
    let usartdiv = (2 * pclk) / (divisor * baudrate);
    (usartdiv + 1) / 2
}

// 示例：PCLK2=84MHz，波特率115200，16倍过采样
// USARTDIV = 84000000 / (16 × 115200) = 45.57
// 取整后 USARTDIV = 46 (0x2E)
```

### 时钟精度要求

波特率误差应控制在±2%以内：

```rust
// 计算实际波特率误差
fn calculate_baudrate_error(target: u32, actual: u32) -> f32 {
    ((actual as f32 - target as f32) / target as f32) * 100.0
}
```

## 寄存器配置

### 主要控制寄存器

#### 1. USART_CR1（控制寄存器1）
```
位域说明：
- UE[13]：USART使能
- M[12]：字长选择（0=8位，1=9位）
- PCE[10]：校验控制使能
- PS[9]：校验选择（0=偶校验，1=奇校验）
- PEIE[8]：校验错误中断使能
- TXEIE[7]：发送数据寄存器空中断使能
- TCIE[6]：发送完成中断使能
- RXNEIE[5]：接收数据寄存器非空中断使能
- TE[3]：发送器使能
- RE[2]：接收器使能
```

#### 2. USART_CR2（控制寄存器2）
```
位域说明：
- STOP[13:12]：停止位数量
  - 00：1个停止位
  - 01：0.5个停止位
  - 10：2个停止位
  - 11：1.5个停止位
- CLKEN[11]：时钟使能（仅USART）
```

#### 3. USART_CR3（控制寄存器3）
```
位域说明：
- DMAT[7]：DMA发送使能
- DMAR[6]：DMA接收使能
- RTSE[8]：RTS使能
- CTSE[9]：CTS使能
- HDSEL[3]：半双工选择
```

#### 4. USART_BRR（波特率寄存器）
```
位域说明：
- DIV_Mantissa[15:4]：分频因子整数部分
- DIV_Fraction[3:0]：分频因子小数部分
```

### 配置示例

```rust
use stm32f4xx_hal::pac::USART1;

fn configure_uart(usart: &USART1, baudrate: u32, pclk: u32) {
    // 1. 计算并设置波特率
    let usartdiv = calculate_usartdiv(pclk, baudrate, false);
    usart.brr.write(|w| unsafe { w.bits(usartdiv as u32) });
    
    // 2. 配置数据格式：8位数据，1停止位，无校验
    usart.cr1.modify(|_, w| {
        w.m().bit(false)      // 8位数据
         .pce().bit(false)    // 无校验
    });
    
    usart.cr2.modify(|_, w| {
        w.stop().bits(0b00)   // 1个停止位
    });
    
    // 3. 使能发送器和接收器
    usart.cr1.modify(|_, w| {
        w.te().set_bit()      // 使能发送器
         .re().set_bit()      // 使能接收器
    });
    
    // 4. 使能USART
    usart.cr1.modify(|_, w| w.ue().set_bit());
}
```

## 中断系统

### 中断类型

STM32 UART支持多种中断：

1. **TXE（Transmit Data Register Empty）**
   - 发送数据寄存器空时触发
   - 用于连续发送数据

2. **TC（Transmission Complete）**
   - 数据传输完成时触发
   - 包括移位寄存器也为空

3. **RXNE（Receive Data Register Not Empty）**
   - 接收到数据时触发
   - 最常用的接收中断

4. **IDLE（Idle Line Detected）**
   - 检测到空闲线路时触发
   - 用于帧结束检测

5. **错误中断**
   - PE（Parity Error）：校验错误
   - FE（Frame Error）：帧错误
   - NE（Noise Error）：噪声错误
   - ORE（Overrun Error）：溢出错误

### 中断配置示例

```rust
// 使能接收中断
usart.cr1.modify(|_, w| w.rxneie().set_bit());

// 使能发送完成中断
usart.cr1.modify(|_, w| w.tcie().set_bit());

// 使能错误中断
usart.cr3.modify(|_, w| {
    w.eie().set_bit()  // 错误中断使能
});
```

## DMA支持

### DMA通道分配

STM32F4系列UART DMA通道分配：

| UART/USART | DMA控制器 | TX通道 | RX通道 |
|------------|-----------|--------|--------|
| USART1     | DMA2      | Stream 7, Channel 4 | Stream 2/5, Channel 4 |
| USART2     | DMA1      | Stream 6, Channel 4 | Stream 5, Channel 4 |
| USART3     | DMA1      | Stream 3/4, Channel 4 | Stream 1, Channel 4 |
| UART4      | DMA1      | Stream 4, Channel 4 | Stream 2, Channel 4 |
| UART5      | DMA1      | Stream 7, Channel 4 | Stream 0, Channel 4 |
| USART6     | DMA2      | Stream 6/7, Channel 5 | Stream 1/2, Channel 5 |

### DMA配置

```rust
// 使能UART DMA
usart.cr3.modify(|_, w| {
    w.dmat().set_bit()    // 发送DMA使能
     .dmar().set_bit()    // 接收DMA使能
});
```

## 高级特性

### 1. 硬件流控制

STM32 USART支持RTS/CTS硬件流控制：

```rust
// 使能硬件流控制
usart.cr3.modify(|_, w| {
    w.rtse().set_bit()    // RTS使能
     .ctse().set_bit()    // CTS使能
});
```

### 2. 半双工模式

```rust
// 配置半双工模式
usart.cr3.modify(|_, w| w.hdsel().set_bit());
```

### 3. 同步模式（仅USART）

```rust
// 使能同步模式
usart.cr2.modify(|_, w| {
    w.clken().set_bit()   // 时钟输出使能
});
```

### 4. 多处理器通信

STM32支持9位数据模式用于多处理器通信：

```rust
// 配置9位数据模式
usart.cr1.modify(|_, w| w.m().set_bit());
```

## 性能特性

### 最大波特率

| 外设 | APB时钟 | 最大波特率 |
|------|---------|------------|
| USART1/6 | 84MHz | 10.5Mbps |
| USART2/3 | 42MHz | 5.25Mbps |
| UART4/5 | 42MHz | 5.25Mbps |

### 过采样模式

STM32支持两种过采样模式：
- **16倍过采样**：更好的噪声抑制，标准模式
- **8倍过采样**：支持更高波特率

```rust
// 配置8倍过采样
usart.cr1.modify(|_, w| w.over8().set_bit());
```

## 调试和测试

### 状态寄存器监控

```rust
// 读取UART状态
let status = usart.sr.read();

if status.txe().bit_is_set() {
    // 发送寄存器空
}

if status.rxne().bit_is_set() {
    // 接收寄存器非空
}

if status.pe().bit_is_set() {
    // 校验错误
}
```

### 常见问题诊断

1. **波特率不匹配**
   - 检查时钟配置
   - 验证分频因子计算

2. **数据丢失**
   - 检查中断响应时间
   - 考虑使用DMA

3. **通信错误**
   - 检查电气连接
   - 验证电平匹配

## 最佳实践

### 1. 时钟配置
- 确保APB时钟频率足够支持目标波特率
- 计算并验证波特率误差

### 2. 中断处理
- 保持中断服务程序简短
- 使用缓冲区处理数据

### 3. DMA使用
- 对于高速或大量数据传输使用DMA
- 正确配置内存对齐

### 4. 错误处理
- 实现完整的错误检测和恢复机制
- 监控通信质量

## 总结

STM32的UART/USART外设提供了丰富的功能和灵活的配置选项。理解其硬件架构、寄存器配置和高级特性，是实现高效可靠串口通信的基础。在实际应用中，需要根据具体需求选择合适的配置参数和优化策略。

## 参考资料

- [STM32F4xx Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [STM32F4xx HAL Driver User Manual](https://www.st.com/resource/en/user_manual/dm00105879.pdf)
- [AN4776: General-purpose timer cookbook for STM32 microcontrollers](https://www.st.com/resource/en/application_note/dm00236305.pdf)