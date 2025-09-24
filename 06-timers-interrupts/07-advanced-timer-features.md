# 高级定时器功能

## 概述

STM32的高级定时器（TIM1和TIM8）是功能最丰富的定时器，除了具备通用定时器的所有功能外，还提供了许多专门用于电机控制和电源管理的高级功能。这些功能包括互补输出、死区生成、刹车功能、重复计数器、三相PWM生成等，使其成为电机控制、开关电源、UPS等应用的理想选择。

## 高级定时器架构

### 核心组件

高级定时器包含以下核心组件：
- **16位递增/递减/中心对齐计数器**
- **16位可编程预分频器**
- **4个独立通道**，每个通道可配置为：
  - 输入捕获
  - 输出比较
  - PWM生成（边沿对齐或中心对齐）
- **互补输出**（CHxN）
- **可编程死区生成器**
- **刹车输入**和紧急停止功能
- **重复计数器**
- **DMA支持**

```
                    高级定时器架构图
    
    时钟源 ──→ 预分频器 ──→ 计数器 ──→ 自动重载寄存器
                              │
                              ├──→ 通道1 ──→ 输出比较/PWM ──→ CH1
                              │              │
                              │              └──→ 死区生成 ──→ CH1N
                              │
                              ├──→ 通道2 ──→ 输出比较/PWM ──→ CH2
                              │              │
                              │              └──→ 死区生成 ──→ CH2N
                              │
                              ├──→ 通道3 ──→ 输出比较/PWM ──→ CH3
                              │              │
                              │              └──→ 死区生成 ──→ CH3N
                              │
                              └──→ 通道4 ──→ 输出比较/PWM ──→ CH4
                                   
    刹车输入 ──→ 刹车逻辑 ──→ 主输出使能 (MOE)
    
    重复计数器 ──→ 更新事件生成
```

## 互补输出功能

### 基本配置

```rust
use stm32f4xx_hal::pac::{GPIOA, GPIOB, RCC, TIM1};

fn configure_complementary_outputs() {
    let rcc = unsafe { &*RCC::ptr() };
    let gpioa = unsafe { &*GPIOA::ptr() };
    let gpiob = unsafe { &*GPIOB::ptr() };
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 使能时钟
    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit().gpioben().set_bit());
    rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
    
    // 配置GPIO - 正输出
    // PA8 - TIM1_CH1, PA9 - TIM1_CH2, PA10 - TIM1_CH3
    gpioa.moder.modify(|_, w| unsafe {
        w.moder8().bits(0b10)   // AF mode
         .moder9().bits(0b10)   // AF mode
         .moder10().bits(0b10)  // AF mode
    });
    
    gpioa.afrh.modify(|_, w| unsafe {
        w.afrh8().bits(0b0001)  // AF1 (TIM1)
         .afrh9().bits(0b0001)  // AF1 (TIM1)
         .afrh10().bits(0b0001) // AF1 (TIM1)
    });
    
    // 配置GPIO - 互补输出
    // PB13 - TIM1_CH1N, PB14 - TIM1_CH2N, PB15 - TIM1_CH3N
    gpiob.moder.modify(|_, w| unsafe {
        w.moder13().bits(0b10)  // AF mode
         .moder14().bits(0b10)  // AF mode
         .moder15().bits(0b10)  // AF mode
    });
    
    gpiob.afrh.modify(|_, w| unsafe {
        w.afrh13().bits(0b0001) // AF1 (TIM1)
         .afrh14().bits(0b0001) // AF1 (TIM1)
         .afrh15().bits(0b0001) // AF1 (TIM1)
    });
    
    // 配置定时器基本参数
    tim1.psc.write(|w| unsafe { w.bits(0) });     // 无预分频
    tim1.arr.write(|w| unsafe { w.bits(8399) });  // 10kHz PWM @ 84MHz
    
    // 配置PWM模式
    tim1.ccmr1.modify(|_, w| unsafe {
        w.oc1m().bits(0b110)    // PWM模式1
         .oc1pe().set_bit()     // 预装载使能
         .oc2m().bits(0b110)    // PWM模式1
         .oc2pe().set_bit()     // 预装载使能
    });
    
    tim1.ccmr2.modify(|_, w| unsafe {
        w.oc3m().bits(0b110)    // PWM模式1
         .oc3pe().set_bit()     // 预装载使能
    });
    
    // 使能正输出和互补输出
    tim1.ccer.modify(|_, w| {
        w.cc1e().set_bit()      // 使能CH1
         .cc1ne().set_bit()     // 使能CH1N
         .cc1p().clear_bit()    // CH1正极性
         .cc1np().clear_bit()   // CH1N正极性
         .cc2e().set_bit()      // 使能CH2
         .cc2ne().set_bit()     // 使能CH2N
         .cc2p().clear_bit()    // CH2正极性
         .cc2np().clear_bit()   // CH2N正极性
         .cc3e().set_bit()      // 使能CH3
         .cc3ne().set_bit()     // 使能CH3N
         .cc3p().clear_bit()    // CH3正极性
         .cc3np().clear_bit()   // CH3N正极性
    });
    
    // 设置初始占空比
    tim1.ccr1.write(|w| unsafe { w.bits(2100) }); // 25%
    tim1.ccr2.write(|w| unsafe { w.bits(4200) }); // 50%
    tim1.ccr3.write(|w| unsafe { w.bits(6300) }); // 75%
}
```

### 死区时间配置

```rust
// 死区时间配置
fn configure_dead_time() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置死区时间
    // DTG[7:0] = 死区时间配置
    // 死区时间 = DTG × tDTS
    // tDTS = 1/fDTS，其中fDTS通常等于定时器时钟
    
    tim1.bdtr.modify(|_, w| unsafe {
        w.dtg().bits(168)       // 死区时间 = 168 × (1/84MHz) = 2μs
         .lock().bits(0b00)     // 无锁定
         .ossi().clear_bit()    // 空闲状态下关闭输出
         .ossr().clear_bit()    // 运行状态下关闭输出
         .bke().clear_bit()     // 暂不使能刹车
         .bkp().clear_bit()     // 刹车极性
         .aoe().clear_bit()     // 自动输出使能
         .moe().set_bit()       // 主输出使能
    });
}

// 动态调整死区时间
fn set_dead_time_ns(dead_time_ns: u32) {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 计算DTG值
    // 假设定时器时钟为84MHz，tDTS = 11.9ns
    const T_DTS_NS: u32 = 12; // 约12ns
    let dtg_value = (dead_time_ns / T_DTS_NS).min(255) as u8;
    
    tim1.bdtr.modify(|_, w| unsafe { w.dtg().bits(dtg_value) });
}

// 死区时间计算函数
fn calculate_dead_time_register(dead_time_ns: u32, timer_clock_hz: u32) -> u8 {
    let t_dts_ns = 1_000_000_000 / timer_clock_hz;
    
    // 根据死区时间范围选择不同的配置
    if dead_time_ns <= 127 * t_dts_ns {
        // DTG[6:0] = DT, DTG[7] = 0
        (dead_time_ns / t_dts_ns).min(127) as u8
    } else if dead_time_ns <= 254 * t_dts_ns {
        // DTG[5:0] = DT, DTG[7:6] = 10
        let dt = (dead_time_ns / (2 * t_dts_ns)).min(63) as u8;
        0b10000000 | dt
    } else if dead_time_ns <= 504 * t_dts_ns {
        // DTG[4:0] = DT, DTG[7:5] = 110
        let dt = ((dead_time_ns - 128 * t_dts_ns) / (8 * t_dts_ns)).min(31) as u8;
        0b11000000 | dt
    } else {
        // DTG[4:0] = DT, DTG[7:5] = 111
        let dt = ((dead_time_ns - 256 * t_dts_ns) / (16 * t_dts_ns)).min(31) as u8;
        0b11100000 | dt
    }
}
```

## 刹车功能

### 刹车输入配置

```rust
fn configure_brake_function() {
    let rcc = unsafe { &*RCC::ptr() };
    let gpiob = unsafe { &*GPIOB::ptr() };
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置刹车输入引脚 PB12 - TIM1_BKIN
    gpiob.moder.modify(|_, w| unsafe {
        w.moder12().bits(0b10)  // AF mode
    });
    
    gpiob.afrh.modify(|_, w| unsafe {
        w.afrh12().bits(0b0001) // AF1 (TIM1)
    });
    
    // 配置上拉电阻
    gpiob.pupdr.modify(|_, w| unsafe {
        w.pupdr12().bits(0b01)  // 上拉
    });
    
    // 配置刹车功能
    tim1.bdtr.modify(|_, w| unsafe {
        w.bke().set_bit()       // 使能刹车
         .bkp().clear_bit()     // 刹车输入低电平有效
         .aoe().clear_bit()     // 禁用自动输出使能
         .moe().set_bit()       // 主输出使能
    });
    
    // 使能刹车中断
    tim1.dier.modify(|_, w| w.bie().set_bit());
}

// 软件刹车
fn software_brake() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 清除主输出使能位，立即停止所有输出
    tim1.bdtr.modify(|_, w| w.moe().clear_bit());
}

// 刹车恢复
fn brake_recovery() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 检查刹车状态
    if !tim1.sr.read().bif().bit_is_set() {
        // 刹车条件已清除，重新使能输出
        tim1.bdtr.modify(|_, w| w.moe().set_bit());
    }
}

// 刹车中断处理
#[interrupt]
fn TIM1_BRK_TIM9() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    if tim1.sr.read().bif().bit_is_set() {
        // 清除刹车中断标志
        tim1.sr.modify(|_, w| w.bif().clear_bit());
        
        // 执行紧急停止程序
        emergency_stop_procedure();
        
        // 记录故障信息
        log_brake_event();
    }
}

fn emergency_stop_procedure() {
    // 1. 立即停止所有PWM输出
    software_brake();
    
    // 2. 关闭功率器件
    disable_power_devices();
    
    // 3. 设置安全状态
    set_system_safe_state();
    
    // 4. 通知上层应用
    notify_fault_condition();
}
```

### 高级刹车配置

```rust
// 多重刹车源配置
fn configure_multiple_brake_sources() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置刹车输入滤波
    tim1.bdtr.modify(|_, w| unsafe {
        w.bkf().bits(0b1111)    // 最大滤波，减少误触发
    });
    
    // 如果支持刹车输入2（某些型号）
    #[cfg(feature = "brake2")]
    tim1.bdtr.modify(|_, w| {
        w.bk2e().set_bit()      // 使能刹车输入2
         .bk2p().clear_bit()    // 刹车输入2低电平有效
    });
}

// 刹车输出状态配置
fn configure_brake_output_states() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    tim1.bdtr.modify(|_, w| {
        w.ossi().set_bit()      // 空闲状态输出选择
         .ossr().set_bit()      // 运行状态输出选择
    });
    
    // 配置各通道在刹车时的输出状态
    tim1.cr2.modify(|_, w| unsafe {
        w.ois1().clear_bit()    // CH1空闲时输出低电平
         .ois1n().clear_bit()   // CH1N空闲时输出低电平
         .ois2().clear_bit()    // CH2空闲时输出低电平
         .ois2n().clear_bit()   // CH2N空闲时输出低电平
         .ois3().clear_bit()    // CH3空闲时输出低电平
         .ois3n().clear_bit()   // CH3N空闲时输出低电平
         .ois4().clear_bit()    // CH4空闲时输出低电平
    });
}
```

## 重复计数器

### 基本配置

```rust
fn configure_repetition_counter() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 设置重复计数器值
    // 更新事件将在(RCR+1)个PWM周期后产生
    tim1.rcr.write(|w| unsafe { w.bits(2) }); // 每3个PWM周期产生一次更新事件
    
    // 使能更新中断
    tim1.dier.modify(|_, w| w.uie().set_bit());
    
    // 使能预装载
    tim1.cr1.modify(|_, w| w.arpe().set_bit());
}

// 动态调整重复计数
fn set_repetition_count(count: u8) {
    let tim1 = unsafe { &*TIM1::ptr() };
    tim1.rcr.write(|w| unsafe { w.bits(count) });
}

// 重复计数器应用：三相PWM同步更新
fn configure_three_phase_sync_update() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 设置重复计数器为2，每3个PWM周期更新一次
    tim1.rcr.write(|w| unsafe { w.bits(2) });
    
    // 使能预装载
    tim1.cr1.modify(|_, w| w.arpe().set_bit());
    tim1.ccmr1.modify(|_, w| {
        w.oc1pe().set_bit()     // CH1预装载
         .oc2pe().set_bit()     // CH2预装载
    });
    tim1.ccmr2.modify(|_, w| {
        w.oc3pe().set_bit()     // CH3预装载
    });
    
    // 在更新中断中同时更新三相占空比
    tim1.dier.modify(|_, w| w.uie().set_bit());
}

// 更新中断处理
static mut PHASE_A_DUTY: u32 = 0;
static mut PHASE_B_DUTY: u32 = 0;
static mut PHASE_C_DUTY: u32 = 0;

#[interrupt]
fn TIM1_UP_TIM10() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    if tim1.sr.read().uif().bit_is_set() {
        tim1.sr.modify(|_, w| w.uif().clear_bit());
        
        unsafe {
            // 同时更新三相PWM占空比
            tim1.ccr1.write(|w| w.bits(PHASE_A_DUTY));
            tim1.ccr2.write(|w| w.bits(PHASE_B_DUTY));
            tim1.ccr3.write(|w| w.bits(PHASE_C_DUTY));
        }
    }
}
```

## 中心对齐PWM

### 配置中心对齐模式

```rust
fn configure_center_aligned_pwm() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置中心对齐模式
    tim1.cr1.modify(|_, w| unsafe {
        w.cms().bits(0b01)      // 中心对齐模式1
         .dir().clear_bit()     // 向上计数开始
    });
    
    // 在中心对齐模式下，有效PWM频率是边沿对齐模式的一半
    tim1.arr.write(|w| unsafe { w.bits(4199) }); // 10kHz PWM
    
    // 配置PWM模式
    tim1.ccmr1.modify(|_, w| unsafe {
        w.oc1m().bits(0b110)    // PWM模式1
         .oc2m().bits(0b110)    // PWM模式1
    });
    
    tim1.ccmr2.modify(|_, w| unsafe {
        w.oc3m().bits(0b110)    // PWM模式1
    });
    
    // 设置占空比
    tim1.ccr1.write(|w| unsafe { w.bits(1050) }); // 25%
    tim1.ccr2.write(|w| unsafe { w.bits(2100) }); // 50%
    tim1.ccr3.write(|w| unsafe { w.bits(3150) }); // 75%
}

// 中心对齐模式的优势：
// 1. 更好的EMI特性
// 2. 对称的PWM波形
// 3. 更低的输出纹波
// 4. 适合电机控制应用
```

## 三相PWM生成

### 基本三相PWM

```rust
// 三相PWM生成器
struct ThreePhasePWM {
    frequency_hz: u32,
    dead_time_ns: u32,
    phase_shift_degrees: f32,
}

impl ThreePhasePWM {
    fn new(frequency_hz: u32, dead_time_ns: u32) -> Self {
        Self {
            frequency_hz,
            dead_time_ns,
            phase_shift_degrees: 120.0, // 标准三相120度相移
        }
    }
    
    fn configure(&self) {
        let tim1 = unsafe { &*TIM1::ptr() };
        
        // 计算ARR值
        const TIMER_CLOCK: u32 = 84_000_000;
        let arr_value = (TIMER_CLOCK / (2 * self.frequency_hz)) - 1;
        
        // 配置基本参数
        tim1.psc.write(|w| unsafe { w.bits(0) });
        tim1.arr.write(|w| unsafe { w.bits(arr_value) });
        
        // 配置中心对齐模式
        tim1.cr1.modify(|_, w| unsafe {
            w.cms().bits(0b01)      // 中心对齐模式1
        });
        
        // 配置三个通道的PWM
        tim1.ccmr1.modify(|_, w| unsafe {
            w.oc1m().bits(0b110)    // PWM模式1
             .oc1pe().set_bit()     // 预装载使能
             .oc2m().bits(0b110)    // PWM模式1
             .oc2pe().set_bit()     // 预装载使能
        });
        
        tim1.ccmr2.modify(|_, w| unsafe {
            w.oc3m().bits(0b110)    // PWM模式1
             .oc3pe().set_bit()     // 预装载使能
        });
        
        // 使能互补输出
        tim1.ccer.modify(|_, w| {
            w.cc1e().set_bit().cc1ne().set_bit()
             .cc2e().set_bit().cc2ne().set_bit()
             .cc3e().set_bit().cc3ne().set_bit()
        });
        
        // 配置死区时间
        let dtg = calculate_dead_time_register(self.dead_time_ns, TIMER_CLOCK);
        tim1.bdtr.modify(|_, w| unsafe {
            w.dtg().bits(dtg)
             .moe().set_bit()
        });
    }
    
    fn set_duty_cycles(&self, duty_a: f32, duty_b: f32, duty_c: f32) {
        let tim1 = unsafe { &*TIM1::ptr() };
        let arr = tim1.arr.read().bits() as f32;
        
        let ccr_a = (arr * duty_a / 100.0) as u32;
        let ccr_b = (arr * duty_b / 100.0) as u32;
        let ccr_c = (arr * duty_c / 100.0) as u32;
        
        tim1.ccr1.write(|w| unsafe { w.bits(ccr_a) });
        tim1.ccr2.write(|w| unsafe { w.bits(ccr_b) });
        tim1.ccr3.write(|w| unsafe { w.bits(ccr_c) });
    }
    
    fn generate_sinusoidal_pwm(&self, amplitude: f32, frequency: f32, time: f32) {
        use core::f32::consts::PI;
        
        let omega = 2.0 * PI * frequency;
        let phase_shift = 2.0 * PI / 3.0; // 120度相移
        
        // 计算三相正弦波
        let phase_a = amplitude * (omega * time).sin();
        let phase_b = amplitude * (omega * time + phase_shift).sin();
        let phase_c = amplitude * (omega * time + 2.0 * phase_shift).sin();
        
        // 转换为占空比（0-100%）
        let duty_a = 50.0 + phase_a * 50.0;
        let duty_b = 50.0 + phase_b * 50.0;
        let duty_c = 50.0 + phase_c * 50.0;
        
        self.set_duty_cycles(duty_a, duty_b, duty_c);
    }
}
```

### 空间矢量PWM (SVPWM)

```rust
// 空间矢量PWM实现
struct SVPWM {
    sector: u8,
    t1: f32,
    t2: f32,
    t0: f32,
}

impl SVPWM {
    fn new() -> Self {
        Self {
            sector: 0,
            t1: 0.0,
            t2: 0.0,
            t0: 0.0,
        }
    }
    
    fn calculate(&mut self, v_alpha: f32, v_beta: f32, v_dc: f32, ts: f32) {
        use core::f32::consts::PI;
        
        // 计算参考电压的幅值和角度
        let v_ref = (v_alpha * v_alpha + v_beta * v_beta).sqrt();
        let theta = v_beta.atan2(v_alpha);
        
        // 确定扇区
        let theta_deg = theta * 180.0 / PI;
        let theta_norm = if theta_deg < 0.0 { theta_deg + 360.0 } else { theta_deg };
        self.sector = (theta_norm / 60.0) as u8 + 1;
        
        // 计算基本矢量作用时间
        let sqrt3 = 3.0_f32.sqrt();
        let m = v_ref / (v_dc / sqrt3); // 调制度
        
        match self.sector {
            1 => {
                self.t1 = m * ts * (60.0 * PI / 180.0 - theta).sin();
                self.t2 = m * ts * theta.sin();
            },
            2 => {
                self.t1 = m * ts * (theta - 60.0 * PI / 180.0).sin();
                self.t2 = m * ts * (120.0 * PI / 180.0 - theta).sin();
            },
            3 => {
                self.t1 = m * ts * (120.0 * PI / 180.0 - theta).sin();
                self.t2 = m * ts * (theta - 60.0 * PI / 180.0).sin();
            },
            4 => {
                self.t1 = m * ts * (theta - 120.0 * PI / 180.0).sin();
                self.t2 = m * ts * (180.0 * PI / 180.0 - theta).sin();
            },
            5 => {
                self.t1 = m * ts * (180.0 * PI / 180.0 - theta).sin();
                self.t2 = m * ts * (theta - 120.0 * PI / 180.0).sin();
            },
            6 => {
                self.t1 = m * ts * (theta - 180.0 * PI / 180.0).sin();
                self.t2 = m * ts * (240.0 * PI / 180.0 - theta).sin();
            },
            _ => {
                self.t1 = 0.0;
                self.t2 = 0.0;
            }
        }
        
        // 零矢量时间
        self.t0 = ts - self.t1 - self.t2;
    }
    
    fn get_duty_cycles(&self, ts: f32) -> (f32, f32, f32) {
        let t1_norm = self.t1 / ts;
        let t2_norm = self.t2 / ts;
        let t0_norm = self.t0 / ts;
        
        match self.sector {
            1 => (
                t0_norm / 2.0 + t1_norm + t2_norm,
                t0_norm / 2.0 + t2_norm,
                t0_norm / 2.0
            ),
            2 => (
                t0_norm / 2.0 + t1_norm,
                t0_norm / 2.0 + t1_norm + t2_norm,
                t0_norm / 2.0
            ),
            3 => (
                t0_norm / 2.0,
                t0_norm / 2.0 + t1_norm + t2_norm,
                t0_norm / 2.0 + t2_norm
            ),
            4 => (
                t0_norm / 2.0,
                t0_norm / 2.0 + t1_norm,
                t0_norm / 2.0 + t1_norm + t2_norm
            ),
            5 => (
                t0_norm / 2.0 + t2_norm,
                t0_norm / 2.0,
                t0_norm / 2.0 + t1_norm + t2_norm
            ),
            6 => (
                t0_norm / 2.0 + t1_norm + t2_norm,
                t0_norm / 2.0,
                t0_norm / 2.0 + t1_norm
            ),
            _ => (0.5, 0.5, 0.5)
        }
    }
}
```

## 定时器同步

### 主从模式配置

```rust
fn configure_master_slave_timers() {
    let tim1 = unsafe { &*TIM1::ptr() };
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 配置TIM1为主定时器
    tim1.cr2.modify(|_, w| unsafe {
        w.mms().bits(0b010)     // 更新事件作为TRGO
    });
    
    // 配置TIM2为从定时器
    tim2.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b110)     // 触发模式
         .ts().bits(0b000)      // ITR0 (TIM1) 作为触发源
    });
    
    // 启动主定时器，从定时器将自动同步
    tim1.cr1.modify(|_, w| w.cen().set_bit());
}

// 定时器级联配置
fn configure_timer_cascade() {
    let tim2 = unsafe { &*TIM2::ptr() };
    let tim3 = unsafe { &*TIM3::ptr() };
    
    // TIM2配置为主定时器，输出更新事件
    tim2.cr2.modify(|_, w| unsafe {
        w.mms().bits(0b010)     // 更新事件作为TRGO
    });
    
    // TIM3配置为从定时器，使用外部时钟模式1
    tim3.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b111)     // 外部时钟模式1
         .ts().bits(0b001)      // ITR1 (TIM2) 作为触发源
    });
    
    // 这样配置后，TIM3将在TIM2每次溢出时递增
    // 实现32位+16位的扩展计数范围
}
```

## DMA支持

### PWM DMA配置

```rust
use stm32f4xx_hal::pac::DMA2;

// PWM波形数据
static PWM_WAVEFORM: [u32; 360] = {
    let mut waveform = [0u32; 360];
    let mut i = 0;
    while i < 360 {
        // 生成正弦波PWM数据
        let angle = i as f32 * core::f32::consts::PI / 180.0;
        let value = ((angle.sin() + 1.0) * 2100.0) as u32; // 0-4200范围
        waveform[i] = value;
        i += 1;
    }
    waveform
};

fn configure_pwm_dma() {
    let tim1 = unsafe { &*TIM1::ptr() };
    let dma2 = unsafe { &*DMA2::ptr() };
    
    // 使能定时器DMA请求
    tim1.dier.modify(|_, w| {
        w.cc1de().set_bit()     // 通道1 DMA请求
         .ude().set_bit()       // 更新DMA请求
    });
    
    // 配置DMA流6用于TIM1_CH1
    dma2.s6cr.modify(|_, w| unsafe {
        w.chsel().bits(0b110)   // 通道6 (TIM1_CH1)
         .mburst().bits(0b00)   // 单次传输
         .pburst().bits(0b00)   // 单次传输
         .pl().bits(0b11)       // 最高优先级
         .msize().bits(0b10)    // 32位内存数据
         .psize().bits(0b10)    // 32位外设数据
         .minc().set_bit()      // 内存地址递增
         .pinc().clear_bit()    // 外设地址固定
         .circ().set_bit()      // 循环模式
         .dir().bits(0b01)      // 内存到外设
    });
    
    // 设置DMA地址和长度
    dma2.s6par.write(|w| unsafe { 
        w.bits(&tim1.ccr1 as *const _ as u32) 
    });
    dma2.s6m0ar.write(|w| unsafe { 
        w.bits(PWM_WAVEFORM.as_ptr() as u32) 
    });
    dma2.s6ndtr.write(|w| unsafe { 
        w.bits(PWM_WAVEFORM.len() as u16) 
    });
    
    // 启动DMA
    dma2.s6cr.modify(|_, w| w.en().set_bit());
}
```

## 应用实例

### 1. 无刷直流电机控制

```rust
// 无刷直流电机控制器
struct BLDCController {
    commutation_table: [[bool; 6]; 6], // 换相表
    current_step: u8,
    speed_rpm: f32,
    direction: MotorDirection,
}

#[derive(PartialEq)]
enum MotorDirection {
    Clockwise,
    CounterClockwise,
}

impl BLDCController {
    fn new() -> Self {
        // 六步换相表 [AH, AL, BH, BL, CH, CL]
        let commutation_table = [
            [true, false, false, true, false, false],   // Step 1
            [true, false, false, false, false, true],   // Step 2
            [false, false, true, false, false, true],   // Step 3
            [false, true, true, false, false, false],   // Step 4
            [false, true, false, false, true, false],   // Step 5
            [false, false, false, true, true, false],   // Step 6
        ];
        
        Self {
            commutation_table,
            current_step: 0,
            speed_rpm: 0.0,
            direction: MotorDirection::Clockwise,
        }
    }
    
    fn configure_hardware(&self) {
        // 配置三相PWM输出
        configure_complementary_outputs();
        configure_dead_time();
        configure_brake_function();
        
        // 配置霍尔传感器输入
        self.configure_hall_sensors();
    }
    
    fn configure_hall_sensors(&self) {
        // 配置霍尔传感器输入捕获
        let tim2 = unsafe { &*TIM2::ptr() };
        
        // 配置三个通道用于霍尔传感器
        tim2.ccmr1.modify(|_, w| unsafe {
            w.cc1s().bits(0b01)    // IC1映射到TI1
             .cc2s().bits(0b01)    // IC2映射到TI2
        });
        
        tim2.ccmr2.modify(|_, w| unsafe {
            w.cc3s().bits(0b01)    // IC3映射到TI3
        });
        
        // 使能所有通道，双边沿触发
        tim2.ccer.modify(|_, w| {
            w.cc1e().set_bit().cc1p().set_bit().cc1np().set_bit()
             .cc2e().set_bit().cc2p().set_bit().cc2np().set_bit()
             .cc3e().set_bit().cc3p().set_bit().cc3np().set_bit()
        });
        
        // 使能中断
        tim2.dier.modify(|_, w| {
            w.cc1ie().set_bit()
             .cc2ie().set_bit()
             .cc3ie().set_bit()
        });
    }
    
    fn commutate(&mut self, hall_state: u8) {
        // 根据霍尔传感器状态确定换相步骤
        self.current_step = match hall_state {
            0b001 => 0,
            0b011 => 1,
            0b010 => 2,
            0b110 => 3,
            0b100 => 4,
            0b101 => 5,
            _ => return, // 无效状态
        };
        
        // 应用换相表
        let step = if self.direction == MotorDirection::Clockwise {
            self.current_step
        } else {
            (6 - self.current_step) % 6
        };
        
        self.apply_commutation_step(step as usize);
    }
    
    fn apply_commutation_step(&self, step: usize) {
        let tim1 = unsafe { &*TIM1::ptr() };
        let states = self.commutation_table[step];
        
        // 配置各相输出状态
        tim1.ccer.modify(|_, w| {
            w.cc1e().bit(states[0])     // AH
             .cc1ne().bit(states[1])    // AL
             .cc2e().bit(states[2])     // BH
             .cc2ne().bit(states[3])    // BL
             .cc3e().bit(states[4])     // CH
             .cc3ne().bit(states[5])    // CL
        });
    }
    
    fn set_speed(&mut self, rpm: f32) {
        self.speed_rpm = rpm;
        
        // 根据速度调整PWM占空比
        let duty_cycle = (rpm / 3000.0 * 100.0).min(100.0).max(0.0);
        
        let tim1 = unsafe { &*TIM1::ptr() };
        let arr = tim1.arr.read().bits() as f32;
        let ccr_value = (arr * duty_cycle / 100.0) as u32;
        
        tim1.ccr1.write(|w| unsafe { w.bits(ccr_value) });
        tim1.ccr2.write(|w| unsafe { w.bits(ccr_value) });
        tim1.ccr3.write(|w| unsafe { w.bits(ccr_value) });
    }
}

// 霍尔传感器中断处理
static mut BLDC_CONTROLLER: Option<BLDCController> = None;

#[interrupt]
fn TIM2() {
    let tim2 = unsafe { &*TIM2::ptr() };
    let gpioa = unsafe { &*GPIOA::ptr() };
    
    // 检查任一霍尔传感器中断
    if tim2.sr.read().cc1if().bit_is_set() || 
       tim2.sr.read().cc2if().bit_is_set() || 
       tim2.sr.read().cc3if().bit_is_set() {
        
        // 清除中断标志
        tim2.sr.modify(|_, w| {
            w.cc1if().clear_bit()
             .cc2if().clear_bit()
             .cc3if().clear_bit()
        });
        
        // 读取霍尔传感器状态
        let hall_state = (gpioa.idr.read().bits() >> 0) & 0x07; // 假设PA0-PA2
        
        unsafe {
            if let Some(ref mut controller) = BLDC_CONTROLLER {
                controller.commutate(hall_state as u8);
            }
        }
    }
}
```

### 2. 开关电源控制

```rust
// 开关电源PWM控制器
struct SwitchingPowerSupply {
    output_voltage: f32,
    target_voltage: f32,
    current_limit: f32,
    pid_controller: PIDController,
    protection_active: bool,
}

struct PIDController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    last_error: f32,
}

impl SwitchingPowerSupply {
    fn new(target_voltage: f32, current_limit: f32) -> Self {
        Self {
            output_voltage: 0.0,
            target_voltage,
            current_limit,
            pid_controller: PIDController {
                kp: 0.1,
                ki: 0.01,
                kd: 0.001,
                integral: 0.0,
                last_error: 0.0,
            },
            protection_active: false,
        }
    }
    
    fn configure_hardware(&self) {
        // 配置PWM输出
        configure_complementary_outputs();
        
        // 配置较短的死区时间（开关电源通常需要更快的开关）
        set_dead_time_ns(100); // 100ns死区
        
        // 配置过流保护刹车输入
        configure_brake_function();
        
        // 配置ADC用于电压和电流采样
        self.configure_adc_sampling();
        
        // 配置高频PWM（通常100kHz以上）
        self.set_switching_frequency(100000); // 100kHz
    }
    
    fn configure_adc_sampling(&self) {
        // 配置ADC定时器触发
        let tim1 = unsafe { &*TIM1::ptr() };
        
        // 配置TRGO2输出用于ADC触发
        tim1.cr2.modify(|_, w| unsafe {
            w.mms2().bits(0b1001)   // OC1REF作为TRGO2
        });
        
        // 在PWM周期的特定点触发ADC采样
        // 通常在开关管关闭后的稳定时刻采样
    }
    
    fn set_switching_frequency(&self, frequency_hz: u32) {
        let tim1 = unsafe { &*TIM1::ptr() };
        const TIMER_CLOCK: u32 = 168_000_000;
        
        let arr_value = (TIMER_CLOCK / frequency_hz) - 1;
        tim1.arr.write(|w| unsafe { w.bits(arr_value) });
    }
    
    fn update_control_loop(&mut self, measured_voltage: f32, measured_current: f32) {
        // 检查保护条件
        if measured_current > self.current_limit {
            self.activate_protection();
            return;
        }
        
        // 更新测量值
        self.output_voltage = measured_voltage;
        
        // PID控制计算
        let error = self.target_voltage - measured_voltage;
        let output = self.pid_controller.calculate(error);
        
        // 限制占空比范围
        let duty_cycle = output.clamp(0.0, 90.0); // 最大90%占空比
        
        // 更新PWM占空比
        self.set_duty_cycle(duty_cycle);
    }
    
    fn set_duty_cycle(&self, duty_percent: f32) {
        let tim1 = unsafe { &*TIM1::ptr() };
        let arr = tim1.arr.read().bits() as f32;
        let ccr_value = (arr * duty_percent / 100.0) as u32;
        
        tim1.ccr1.write(|w| unsafe { w.bits(ccr_value) });
    }
    
    fn activate_protection(&mut self) {
        self.protection_active = true;
        
        // 立即停止PWM输出
        software_brake();
        
        // 记录故障
        log_protection_event();
    }
    
    fn reset_protection(&mut self) {
        if !self.protection_active {
            return;
        }
        
        self.protection_active = false;
        self.pid_controller.reset();
        
        // 重新使能输出
        brake_recovery();
    }
}

impl PIDController {
    fn calculate(&mut self, error: f32) -> f32 {
        // 积分项
        self.integral += error;
        
        // 积分限幅
        self.integral = self.integral.clamp(-100.0, 100.0);
        
        // 微分项
        let derivative = error - self.last_error;
        self.last_error = error;
        
        // PID输出
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }
    
    fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
    }
}
```

## 调试和优化

### 定时器性能监控

```rust
// 定时器性能监控
struct TimerPerformanceMonitor {
    interrupt_count: u32,
    max_interrupt_time: u32,
    total_interrupt_time: u64,
    last_timestamp: u32,
}

impl TimerPerformanceMonitor {
    fn new() -> Self {
        Self {
            interrupt_count: 0,
            max_interrupt_time: 0,
            total_interrupt_time: 0,
            last_timestamp: 0,
        }
    }
    
    fn start_measurement(&mut self) {
        self.last_timestamp = get_system_timestamp();
    }
    
    fn end_measurement(&mut self) {
        let current_time = get_system_timestamp();
        let interrupt_time = current_time - self.last_timestamp;
        
        self.interrupt_count += 1;
        self.total_interrupt_time += interrupt_time as u64;
        self.max_interrupt_time = self.max_interrupt_time.max(interrupt_time);
    }
    
    fn get_statistics(&self) -> TimerStats {
        TimerStats {
            interrupt_count: self.interrupt_count,
            average_interrupt_time: if self.interrupt_count > 0 {
                (self.total_interrupt_time / self.interrupt_count as u64) as u32
            } else {
                0
            },
            max_interrupt_time: self.max_interrupt_time,
        }
    }
}

#[derive(Debug)]
struct TimerStats {
    interrupt_count: u32,
    average_interrupt_time: u32,
    max_interrupt_time: u32,
}
```

## 总结

STM32高级定时器提供了丰富的功能，特别适合电机控制、电源管理等高性能应用。通过合理配置互补输出、死区时间、刹车功能等特性，可以构建安全可靠的功率控制系统。

掌握高级定时器的各项功能和配置方法，是开发专业级嵌入式控制系统的重要技能。在实际应用中，需要根据具体需求选择合适的工作模式和保护机制，确保系统的稳定性和安全性。

## 参考资料

- STM32F4xx Reference Manual
- AN4776: General-purpose Timer Cookbook
- AN4013: STM32 Cross-series Timer Overview  
- AN1905: Motor Control with STM32F10x
- STM32F4xx HAL Driver User Manual