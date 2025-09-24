# STM32定时器架构

## 概述

STM32微控制器集成了多种类型的定时器，每种定时器都有其特定的功能和应用场景。本章将详细介绍STM32F4系列定时器的硬件架构、寄存器结构以及各种定时器的特性和差异。

## STM32F4定时器总览

### 定时器分类和数量

| 定时器类型 | 实例 | 位数 | 通道数 | 主要特性 |
|-----------|------|------|--------|----------|
| 高级定时器 | TIM1, TIM8 | 16位 | 4 | 互补输出、死区时间、刹车功能 |
| 通用定时器 | TIM2, TIM5 | 32位 | 4 | 输入捕获、输出比较、PWM |
| 通用定时器 | TIM3, TIM4 | 16位 | 4 | 输入捕获、输出比较、PWM |
| 通用定时器 | TIM9, TIM12 | 16位 | 2 | 简化版通用定时器 |
| 通用定时器 | TIM10, TIM11, TIM13, TIM14 | 16位 | 1 | 单通道定时器 |
| 基本定时器 | TIM6, TIM7 | 16位 | 0 | 基本计数、DAC触发 |

### 时钟域分布

```
AHB时钟 (168MHz)
├── APB1时钟 (42MHz) → APB1定时器时钟 (84MHz)
│   ├── TIM2, TIM3, TIM4, TIM5
│   ├── TIM6, TIM7
│   └── TIM12, TIM13, TIM14
└── APB2时钟 (84MHz) → APB2定时器时钟 (168MHz)
    ├── TIM1, TIM8
    ├── TIM9, TIM10, TIM11
    └── 系统定时器 (SysTick)
```

## 高级定时器架构 (TIM1, TIM8)

### 核心特性

1. **16位自动重载向上/向下/中心对齐计数器**
2. **16位可编程预分频器**
3. **4个独立通道用于输入捕获/输出比较/PWM生成**
4. **互补输出和可编程死区时间**
5. **刹车输入用于安全关断**
6. **支持编码器和霍尔传感器接口**

### 寄存器结构

```rust
// 高级定时器寄存器映射
pub struct TIM1 {
    pub cr1: CR1,      // 控制寄存器1
    pub cr2: CR2,      // 控制寄存器2
    pub smcr: SMCR,    // 从模式控制寄存器
    pub dier: DIER,    // DMA/中断使能寄存器
    pub sr: SR,        // 状态寄存器
    pub egr: EGR,      // 事件生成寄存器
    pub ccmr1: CCMR1,  // 捕获/比较模式寄存器1
    pub ccmr2: CCMR2,  // 捕获/比较模式寄存器2
    pub ccer: CCER,    // 捕获/比较使能寄存器
    pub cnt: CNT,      // 计数器
    pub psc: PSC,      // 预分频器
    pub arr: ARR,      // 自动重载寄存器
    pub rcr: RCR,      // 重复计数器寄存器
    pub ccr1: CCR1,    // 捕获/比较寄存器1
    pub ccr2: CCR2,    // 捕获/比较寄存器2
    pub ccr3: CCR3,    // 捕获/比较寄存器3
    pub ccr4: CCR4,    // 捕获/比较寄存器4
    pub bdtr: BDTR,    // 刹车和死区时间寄存器
    pub dcr: DCR,      // DMA控制寄存器
    pub dmar: DMAR,    // DMA地址寄存器
}
```

### 互补输出配置

```rust
// 配置互补PWM输出
fn setup_complementary_pwm() {
    // 使能定时器时钟
    rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
    
    // 配置GPIO为复用功能
    // PA8: TIM1_CH1, PA9: TIM1_CH2
    // PB13: TIM1_CH1N, PB14: TIM1_CH2N
    
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置PWM模式1
    tim1.ccmr1.modify(|_, w| unsafe {
        w.oc1m().bits(0b110)  // PWM模式1
         .oc2m().bits(0b110)  // PWM模式1
    });
    
    // 使能主输出和互补输出
    tim1.ccer.modify(|_, w| {
        w.cc1e().set_bit()   // 使能CH1
         .cc1ne().set_bit()  // 使能CH1N
         .cc2e().set_bit()   // 使能CH2
         .cc2ne().set_bit()  // 使能CH2N
    });
    
    // 配置死区时间
    tim1.bdtr.modify(|_, w| unsafe {
        w.dtg().bits(100)    // 死区时间
         .moe().set_bit()    // 主输出使能
    });
}
```

### 刹车功能

```rust
// 配置刹车输入
fn setup_brake_input() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置刹车输入
    tim1.bdtr.modify(|_, w| {
        w.bke().set_bit()      // 使能刹车
         .bkp().clear_bit()    // 刹车极性（低电平有效）
         .aoe().set_bit()      // 自动输出使能
    });
    
    // 使能刹车中断
    tim1.dier.modify(|_, w| w.bie().set_bit());
}

// 刹车中断处理
#[interrupt]
fn TIM1_BRK_TIM9() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    if tim1.sr.read().bif().bit_is_set() {
        // 清除刹车中断标志
        tim1.sr.modify(|_, w| w.bif().clear_bit());
        
        // 执行安全关断程序
        emergency_shutdown();
    }
}
```

## 通用定时器架构 (TIM2-5, TIM9-14)

### TIM2和TIM5 (32位定时器)

```rust
// 32位定时器的优势
fn setup_32bit_timer() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 32位计数器可以实现更长的定时周期
    tim2.arr.write(|w| unsafe { w.bits(0xFFFFFFFF) });
    
    // 计算最大定时周期
    // 假设时钟频率84MHz，预分频器为83
    // 最大周期 = (2^32 - 1) / (84MHz / 84) = 4294967295 / 1MHz ≈ 4295秒
}
```

### 输入捕获配置

```rust
// 配置输入捕获测量脉宽
fn setup_input_capture() {
    let tim3 = unsafe { &*TIM3::ptr() };
    
    // 配置通道1为输入捕获模式
    tim3.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // CC1通道配置为输入，IC1映射到TI1
         .ic1psc().bits(0b00)  // 输入捕获预分频器：每个边沿都捕获
         .ic1f().bits(0b0000)  // 输入滤波器：无滤波
    });
    
    // 配置捕获边沿
    tim3.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能捕获
         .cc1p().clear_bit()   // 上升沿捕获
    });
    
    // 使能捕获中断
    tim3.dier.modify(|_, w| w.cc1ie().set_bit());
}

// 输入捕获中断处理
static mut LAST_CAPTURE: u32 = 0;
static mut PULSE_WIDTH: u32 = 0;

#[interrupt]
fn TIM3() {
    let tim3 = unsafe { &*TIM3::ptr() };
    
    if tim3.sr.read().cc1if().bit_is_set() {
        // 清除捕获中断标志
        tim3.sr.modify(|_, w| w.cc1if().clear_bit());
        
        let current_capture = tim3.ccr1.read().bits();
        unsafe {
            PULSE_WIDTH = current_capture.wrapping_sub(LAST_CAPTURE);
            LAST_CAPTURE = current_capture;
        }
    }
}
```

### 编码器接口

```rust
// 配置编码器接口
fn setup_encoder_interface() {
    let tim4 = unsafe { &*TIM4::ptr() };
    
    // 配置为编码器模式3（TI1和TI2边沿都计数）
    tim4.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b011)    // 编码器模式3
    });
    
    // 配置输入通道
    tim4.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // CC1通道配置为输入
         .cc2s().bits(0b01)    // CC2通道配置为输入
         .ic1f().bits(0b1111)  // 输入滤波
         .ic2f().bits(0b1111)  // 输入滤波
    });
    
    // 配置输入极性
    tim4.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能CC1
         .cc2e().set_bit()     // 使能CC2
         .cc1p().clear_bit()   // CC1极性
         .cc2p().clear_bit()   // CC2极性
    });
    
    // 设置计数器范围
    tim4.arr.write(|w| unsafe { w.bits(0xFFFF) });
}
```

## 基本定时器架构 (TIM6, TIM7)

### 特性和应用

基本定时器具有最简单的结构，主要用于：
- DAC转换触发
- ADC转换触发
- 系统时基生成
- 简单的定时功能

```rust
// 配置基本定时器触发DAC
fn setup_basic_timer_for_dac() {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    // 配置定时器周期
    tim6.psc.write(|w| unsafe { w.bits(83) });    // 预分频器：84MHz/84 = 1MHz
    tim6.arr.write(|w| unsafe { w.bits(999) });   // 自动重载：1MHz/1000 = 1kHz
    
    // 配置触发输出
    tim6.cr2.modify(|_, w| unsafe {
        w.mms().bits(0b010)    // 更新事件作为触发输出
    });
    
    // 使能定时器
    tim6.cr1.modify(|_, w| w.cen().set_bit());
}
```

## 定时器同步和级联

### 主从模式配置

```rust
// 配置定时器主从模式
fn setup_master_slave_timers() {
    let tim2 = unsafe { &*TIM2::ptr() };  // 主定时器
    let tim3 = unsafe { &*TIM3::ptr() };  // 从定时器
    
    // 配置TIM2为主模式
    tim2.cr2.modify(|_, w| unsafe {
        w.mms().bits(0b010)    // 更新事件作为触发输出
    });
    
    // 配置TIM3为从模式
    tim3.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b110)    // 触发模式
         .ts().bits(0b001)     // 选择ITR1作为触发源（TIM2）
    });
}
```

### 定时器级联

```rust
// 实现32位+16位=48位计数器
fn setup_cascaded_timers() {
    let tim2 = unsafe { &*TIM2::ptr() };  // 低32位
    let tim3 = unsafe { &*TIM3::ptr() };  // 高16位
    
    // 配置TIM2
    tim2.psc.write(|w| unsafe { w.bits(0) });
    tim2.arr.write(|w| unsafe { w.bits(0xFFFFFFFF) });
    tim2.cr2.modify(|_, w| unsafe {
        w.mms().bits(0b010)    // 更新事件作为触发输出
    });
    
    // 配置TIM3为外部时钟模式
    tim3.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b111)    // 外部时钟模式1
         .ts().bits(0b001)     // ITR1 (TIM2)
    });
    tim3.arr.write(|w| unsafe { w.bits(0xFFFF) });
}
```

## DMA支持

### 定时器DMA请求

```rust
// 配置定时器DMA传输
fn setup_timer_dma() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 使能更新DMA请求
    tim1.dier.modify(|_, w| w.ude().set_bit());
    
    // 配置DMA突发传输
    tim1.dcr.modify(|_, w| unsafe {
        w.dba().bits(0x0D)     // DMA基地址：CCR1
         .dbl().bits(0x03)     // DMA突发长度：4个传输
    });
}
```

## 时钟配置和计算

### 时钟树分析

```rust
// 定时器时钟频率计算
fn calculate_timer_frequency() {
    // STM32F407VG时钟配置
    const HSE_FREQ: u32 = 8_000_000;      // 外部晶振8MHz
    const PLL_M: u32 = 8;                 // PLL分频系数
    const PLL_N: u32 = 336;               // PLL倍频系数
    const PLL_P: u32 = 2;                 // PLL输出分频
    const PLL_Q: u32 = 7;                 // USB时钟分频
    
    // 系统时钟计算
    let pll_input = HSE_FREQ / PLL_M;     // 1MHz
    let pll_vco = pll_input * PLL_N;      // 336MHz
    let sysclk = pll_vco / PLL_P;         // 168MHz
    
    // APB时钟计算
    let ahb_freq = sysclk;                // 168MHz (AHB预分频器=1)
    let apb1_freq = ahb_freq / 4;         // 42MHz (APB1预分频器=4)
    let apb2_freq = ahb_freq / 2;         // 84MHz (APB2预分频器=2)
    
    // 定时器时钟计算
    let tim_apb1_freq = if apb1_freq == ahb_freq { apb1_freq } else { apb1_freq * 2 }; // 84MHz
    let tim_apb2_freq = if apb2_freq == ahb_freq { apb2_freq } else { apb2_freq * 2 }; // 168MHz
}
```

### 精确定时计算

```rust
// 精确定时参数计算
fn calculate_timing_parameters(target_freq: u32, timer_clock: u32) -> (u16, u16) {
    let mut best_error = u32::MAX;
    let mut best_psc = 1u16;
    let mut best_arr = 1u16;
    
    // 遍历可能的预分频器值
    for psc in 1..=65536u32 {
        let timer_freq = timer_clock / psc;
        if timer_freq < target_freq {
            break;
        }
        
        let arr = timer_freq / target_freq;
        if arr > 65536 {
            continue;
        }
        
        let actual_freq = timer_freq / arr;
        let error = if actual_freq > target_freq {
            actual_freq - target_freq
        } else {
            target_freq - actual_freq
        };
        
        if error < best_error {
            best_error = error;
            best_psc = (psc - 1) as u16;
            best_arr = (arr - 1) as u16;
        }
    }
    
    (best_psc, best_arr)
}
```

## 功耗优化

### 低功耗模式下的定时器

```rust
// 低功耗定时器配置
fn setup_low_power_timer() {
    // 使用LPTIM（低功耗定时器）
    // 或者配置普通定时器在停止模式下工作
    
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 降低定时器频率以减少功耗
    tim2.psc.write(|w| unsafe { w.bits(8399) }); // 10kHz
    tim2.arr.write(|w| unsafe { w.bits(9999) }); // 1Hz
    
    // 只在需要时使能定时器
    tim2.cr1.modify(|_, w| w.cen().set_bit());
}
```

## 调试和测试

### 定时器状态监控

```rust
// 定时器状态调试函数
fn debug_timer_status() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    let cnt = tim1.cnt.read().bits();
    let arr = tim1.arr.read().bits();
    let psc = tim1.psc.read().bits();
    let sr = tim1.sr.read().bits();
    
    // 通过RTT或串口输出调试信息
    rprintln!("TIM1 Status:");
    rprintln!("  CNT: {}", cnt);
    rprintln!("  ARR: {}", arr);
    rprintln!("  PSC: {}", psc);
    rprintln!("  SR:  0x{:08X}", sr);
}
```

## 总结

STM32定时器架构提供了丰富的功能和灵活的配置选项，能够满足各种应用需求。理解不同类型定时器的特性和适用场景，掌握寄存器配置和时钟计算方法，是开发高质量嵌入式应用的基础。

在实际应用中，应根据具体需求选择合适的定时器类型，合理配置时钟和参数，并注意功耗和性能的平衡。

## 参考资料

- STM32F407xx Reference Manual (RM0090)
- STM32F4xx HAL Driver User Manual
- AN4013: STM32 Cross-series Timer Overview
- AN4776: General-purpose Timer Cookbook