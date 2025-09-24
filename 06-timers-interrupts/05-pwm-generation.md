# PWM信号生成

## 概述

脉宽调制（PWM，Pulse Width Modulation）是一种通过改变脉冲宽度来控制平均功率的技术。在嵌入式系统中，PWM广泛应用于电机控制、LED调光、音频生成、开关电源等领域。本章将详细介绍STM32定时器的PWM生成功能及其应用。

## PWM基本原理

### PWM信号特性

PWM信号由以下参数定义：
- **周期（Period）**：一个完整PWM周期的时间
- **频率（Frequency）**：PWM信号的频率，频率 = 1/周期
- **占空比（Duty Cycle）**：高电平时间占整个周期的百分比
- **分辨率（Resolution）**：占空比可调节的精度

```
PWM信号示例：
     ┌─────┐     ┌─────┐     ┌─────┐
     │     │     │     │     │     │
─────┘     └─────┘     └─────┘     └─────
     │<-T->│     │<-T->│     │<-T->│
     │<-D->│     │<-D->│     │<-D->│

T = 周期时间
D = 高电平时间
占空比 = D/T × 100%
```

### PWM模式分类

STM32定时器支持两种PWM模式：

1. **PWM模式1**：
   - CNT < CCR时输出有效电平
   - CNT ≥ CCR时输出无效电平

2. **PWM模式2**：
   - CNT < CCR时输出无效电平
   - CNT ≥ CCR时输出有效电平

## 定时器PWM配置

### GPIO配置

```rust
use stm32f4xx_hal::pac::{GPIOA, RCC, TIM1};

fn configure_pwm_gpio() {
    let rcc = unsafe { &*RCC::ptr() };
    let gpioa = unsafe { &*GPIOA::ptr() };
    
    // 使能GPIOA时钟
    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
    
    // 配置PA8为TIM1_CH1复用功能
    gpioa.moder.modify(|_, w| unsafe {
        w.moder8().bits(0b10)  // 复用功能模式
    });
    
    gpioa.otyper.modify(|_, w| {
        w.ot8().clear_bit()    // 推挽输出
    });
    
    gpioa.ospeedr.modify(|_, w| unsafe {
        w.ospeedr8().bits(0b11) // 高速
    });
    
    gpioa.pupdr.modify(|_, w| unsafe {
        w.pupdr8().bits(0b00)   // 无上下拉
    });
    
    // 设置复用功能为AF1 (TIM1)
    gpioa.afrh.modify(|_, w| unsafe {
        w.afrh8().bits(0b0001)
    });
}
```

### 基本PWM配置

```rust
fn configure_basic_pwm() {
    let rcc = unsafe { &*RCC::ptr() };
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 使能TIM1时钟
    rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
    
    // 配置定时器基本参数
    // PWM频率 = 定时器时钟 / ((PSC + 1) * (ARR + 1))
    // 例：168MHz / ((167 + 1) * (999 + 1)) = 168MHz / 168000 = 1kHz
    tim1.psc.write(|w| unsafe { w.bits(167) });  // 预分频器
    tim1.arr.write(|w| unsafe { w.bits(999) });  // 自动重载值
    
    // 配置通道1为PWM模式1
    tim1.ccmr1.modify(|_, w| unsafe {
        w.oc1m().bits(0b110)   // PWM模式1
         .oc1pe().set_bit()    // 使能预装载
         .cc1s().bits(0b00)    // 输出模式
    });
    
    // 使能通道1输出
    tim1.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能输出
         .cc1p().clear_bit()   // 高电平有效
    });
    
    // 设置初始占空比（50%）
    tim1.ccr1.write(|w| unsafe { w.bits(500) });
    
    // 对于高级定时器，需要使能主输出
    tim1.bdtr.modify(|_, w| w.moe().set_bit());
    
    // 启动定时器
    tim1.cr1.modify(|_, w| w.cen().set_bit());
}
```

### 多通道PWM配置

```rust
fn configure_multi_channel_pwm() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置4个通道的PWM
    tim1.ccmr1.modify(|_, w| unsafe {
        w.oc1m().bits(0b110)   // CH1 PWM模式1
         .oc1pe().set_bit()    // CH1 预装载使能
         .oc2m().bits(0b110)   // CH2 PWM模式1
         .oc2pe().set_bit()    // CH2 预装载使能
    });
    
    tim1.ccmr2.modify(|_, w| unsafe {
        w.oc3m().bits(0b110)   // CH3 PWM模式1
         .oc3pe().set_bit()    // CH3 预装载使能
         .oc4m().bits(0b110)   // CH4 PWM模式1
         .oc4pe().set_bit()    // CH4 预装载使能
    });
    
    // 使能所有通道输出
    tim1.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能CH1
         .cc2e().set_bit()     // 使能CH2
         .cc3e().set_bit()     // 使能CH3
         .cc4e().set_bit()     // 使能CH4
    });
    
    // 设置不同的占空比
    tim1.ccr1.write(|w| unsafe { w.bits(250) });  // 25%
    tim1.ccr2.write(|w| unsafe { w.bits(500) });  // 50%
    tim1.ccr3.write(|w| unsafe { w.bits(750) });  // 75%
    tim1.ccr4.write(|w| unsafe { w.bits(900) });  // 90%
}
```

## PWM控制函数

### 占空比控制

```rust
// 设置占空比（0-100%）
fn set_pwm_duty_cycle(channel: u8, duty_percent: f32) {
    let tim1 = unsafe { &*TIM1::ptr() };
    let arr = tim1.arr.read().bits() as f32;
    let ccr_value = (arr * duty_percent / 100.0) as u32;
    
    match channel {
        1 => tim1.ccr1.write(|w| unsafe { w.bits(ccr_value) }),
        2 => tim1.ccr2.write(|w| unsafe { w.bits(ccr_value) }),
        3 => tim1.ccr3.write(|w| unsafe { w.bits(ccr_value) }),
        4 => tim1.ccr4.write(|w| unsafe { w.bits(ccr_value) }),
        _ => {}
    }
}

// 设置绝对脉宽（微秒）
fn set_pwm_pulse_width_us(channel: u8, pulse_width_us: u32) {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 计算定时器分辨率（每个计数的时间）
    let timer_freq = 168_000_000 / ((tim1.psc.read().bits() + 1) as u32);
    let us_per_count = 1_000_000.0 / timer_freq as f32;
    let ccr_value = (pulse_width_us as f32 / us_per_count) as u32;
    
    match channel {
        1 => tim1.ccr1.write(|w| unsafe { w.bits(ccr_value) }),
        2 => tim1.ccr2.write(|w| unsafe { w.bits(ccr_value) }),
        3 => tim1.ccr3.write(|w| unsafe { w.bits(ccr_value) }),
        4 => tim1.ccr4.write(|w| unsafe { w.bits(ccr_value) }),
        _ => {}
    }
}
```

### 频率控制

```rust
fn set_pwm_frequency(frequency_hz: u32) {
    let tim1 = unsafe { &*TIM1::ptr() };
    const TIMER_CLOCK: u32 = 168_000_000; // 168MHz
    
    // 计算最佳的预分频器和自动重载值
    let (prescaler, auto_reload) = calculate_pwm_parameters(TIMER_CLOCK, frequency_hz);
    
    // 停止定时器
    tim1.cr1.modify(|_, w| w.cen().clear_bit());
    
    // 更新参数
    tim1.psc.write(|w| unsafe { w.bits(prescaler) });
    tim1.arr.write(|w| unsafe { w.bits(auto_reload) });
    
    // 生成更新事件
    tim1.egr.write(|w| w.ug().set_bit());
    
    // 重新启动定时器
    tim1.cr1.modify(|_, w| w.cen().set_bit());
}

fn calculate_pwm_parameters(timer_clock: u32, target_freq: u32) -> (u16, u16) {
    let total_counts = timer_clock / target_freq;
    
    // 寻找最佳分辨率的组合
    let mut best_resolution = 0;
    let mut best_psc = 0;
    let mut best_arr = 0;
    
    for psc in 1..=65536u32 {
        let arr = total_counts / psc;
        if arr > 65536 || arr < 2 {
            continue;
        }
        
        let resolution = arr;
        if resolution > best_resolution {
            best_resolution = resolution;
            best_psc = psc - 1;
            best_arr = arr - 1;
        }
    }
    
    (best_psc as u16, best_arr as u16)
}
```

## 高级PWM功能

### 互补PWM输出

```rust
fn configure_complementary_pwm() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置互补输出
    tim1.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能正输出
         .cc1ne().set_bit()    // 使能互补输出
         .cc1p().clear_bit()   // 正输出极性
         .cc1np().clear_bit()  // 互补输出极性
    });
    
    // 配置死区时间
    tim1.bdtr.modify(|_, w| unsafe {
        w.dtg().bits(100)      // 死区时间 = 100 * tDTS
         .moe().set_bit()      // 主输出使能
    });
}
```

### 中心对齐PWM

```rust
fn configure_center_aligned_pwm() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置中心对齐模式
    tim1.cr1.modify(|_, w| unsafe {
        w.cms().bits(0b01)     // 中心对齐模式1
         .dir().clear_bit()    // 向上计数
    });
    
    // 在中心对齐模式下，有效周期是ARR的两倍
    tim1.arr.write(|w| unsafe { w.bits(500) });  // 实际周期 = 1000
    tim1.ccr1.write(|w| unsafe { w.bits(250) }); // 50%占空比
}
```

### PWM输入模式

```rust
fn configure_pwm_input_mode() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 配置PWM输入模式（测量外部PWM信号）
    tim2.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // IC1映射到TI1
         .cc2s().bits(0b10)    // IC2映射到TI1
         .ic1f().bits(0b0000)  // 无滤波
         .ic2f().bits(0b0000)  // 无滤波
    });
    
    // 配置触发边沿
    tim2.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能IC1
         .cc2e().set_bit()     // 使能IC2
         .cc1p().clear_bit()   // IC1上升沿
         .cc2p().set_bit()     // IC2下降沿
    });
    
    // 配置从模式为复位模式
    tim2.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b100)    // 复位模式
         .ts().bits(0b101)     // TI1FP1作为触发
    });
}

// 读取PWM输入测量结果
fn read_pwm_input() -> (u32, u32) {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    let period = tim2.ccr1.read().bits();      // 周期
    let pulse_width = tim2.ccr2.read().bits(); // 脉宽
    
    (period, pulse_width)
}
```

## PWM应用实例

### 1. LED调光

```rust
// LED PWM调光控制
struct LedController {
    current_brightness: u8,
    target_brightness: u8,
    fade_step: i8,
}

impl LedController {
    fn new() -> Self {
        Self {
            current_brightness: 0,
            target_brightness: 0,
            fade_step: 0,
        }
    }
    
    fn set_brightness(&mut self, brightness: u8) {
        self.target_brightness = brightness;
        self.fade_step = if brightness > self.current_brightness { 1 } else { -1 };
    }
    
    fn update(&mut self) {
        if self.current_brightness != self.target_brightness {
            self.current_brightness = 
                (self.current_brightness as i16 + self.fade_step as i16) as u8;
            
            // 应用伽马校正
            let gamma_corrected = self.apply_gamma_correction(self.current_brightness);
            set_pwm_duty_cycle(1, gamma_corrected as f32 / 255.0 * 100.0);
        }
    }
    
    fn apply_gamma_correction(&self, value: u8) -> u8 {
        // 简单的伽马校正表
        const GAMMA_TABLE: [u8; 256] = [
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            // ... 完整的伽马校正表
            255
        ];
        GAMMA_TABLE[value as usize]
    }
}
```

### 2. 舵机控制

```rust
// 舵机PWM控制
struct ServoController {
    min_pulse_us: u32,
    max_pulse_us: u32,
    current_angle: f32,
}

impl ServoController {
    fn new() -> Self {
        Self {
            min_pulse_us: 1000,  // 1ms对应0度
            max_pulse_us: 2000,  // 2ms对应180度
            current_angle: 90.0, // 初始位置90度
        }
    }
    
    fn set_angle(&mut self, angle: f32) {
        let angle = angle.clamp(0.0, 180.0);
        self.current_angle = angle;
        
        // 计算对应的脉宽
        let pulse_width = self.min_pulse_us as f32 + 
            (angle / 180.0) * (self.max_pulse_us - self.min_pulse_us) as f32;
        
        set_pwm_pulse_width_us(1, pulse_width as u32);
    }
    
    fn sweep(&mut self, start_angle: f32, end_angle: f32, steps: u32) {
        let step_size = (end_angle - start_angle) / steps as f32;
        
        for i in 0..=steps {
            let angle = start_angle + i as f32 * step_size;
            self.set_angle(angle);
            
            // 延时等待舵机到位
            delay_ms(20);
        }
    }
}
```

### 3. 电机调速

```rust
// 直流电机PWM调速
struct MotorController {
    speed_percent: f32,
    direction: MotorDirection,
    acceleration: f32,
    max_acceleration: f32,
}

#[derive(PartialEq)]
enum MotorDirection {
    Forward,
    Reverse,
    Brake,
}

impl MotorController {
    fn new() -> Self {
        Self {
            speed_percent: 0.0,
            direction: MotorDirection::Forward,
            acceleration: 0.0,
            max_acceleration: 5.0, // 每次更新最大变化5%
        }
    }
    
    fn set_speed(&mut self, speed: f32, direction: MotorDirection) {
        let target_speed = speed.clamp(0.0, 100.0);
        
        // 计算加速度限制
        let speed_diff = target_speed - self.speed_percent;
        self.acceleration = speed_diff.clamp(-self.max_acceleration, self.max_acceleration);
        
        self.direction = direction;
    }
    
    fn update(&mut self) {
        // 应用加速度限制
        self.speed_percent += self.acceleration;
        self.speed_percent = self.speed_percent.clamp(0.0, 100.0);
        
        match self.direction {
            MotorDirection::Forward => {
                set_pwm_duty_cycle(1, self.speed_percent);  // 正转PWM
                set_pwm_duty_cycle(2, 0.0);                 // 反转PWM关闭
            },
            MotorDirection::Reverse => {
                set_pwm_duty_cycle(1, 0.0);                 // 正转PWM关闭
                set_pwm_duty_cycle(2, self.speed_percent);  // 反转PWM
            },
            MotorDirection::Brake => {
                set_pwm_duty_cycle(1, 100.0);               // 两个PWM都开启
                set_pwm_duty_cycle(2, 100.0);               // 实现电子刹车
            },
        }
    }
}
```

### 4. 音频PWM生成

```rust
// 使用PWM生成音频信号
struct AudioPWM {
    sample_rate: u32,
    current_sample: u16,
    wave_table: [u8; 256],
}

impl AudioPWM {
    fn new(sample_rate: u32) -> Self {
        let mut wave_table = [0u8; 256];
        
        // 生成正弦波表
        for i in 0..256 {
            let angle = 2.0 * core::f32::consts::PI * i as f32 / 256.0;
            wave_table[i] = ((angle.sin() + 1.0) * 127.5) as u8;
        }
        
        Self {
            sample_rate,
            current_sample: 0,
            wave_table,
        }
    }
    
    fn play_tone(&mut self, frequency: f32) {
        // 配置定时器中断以采样率更新PWM
        configure_audio_timer_interrupt(self.sample_rate);
        
        // 计算频率对应的步进值
        let step = (frequency * 256.0 / self.sample_rate as f32) as u16;
        
        // 在定时器中断中更新PWM占空比
        // 这里需要使用全局变量或其他方式传递参数
    }
}

// 音频定时器中断
static mut AUDIO_PWM: Option<AudioPWM> = None;
static mut FREQUENCY_STEP: u16 = 0;
static mut PHASE_ACCUMULATOR: u16 = 0;

#[interrupt]
fn TIM6_DAC() {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    if tim6.sr.read().uif().bit_is_set() {
        tim6.sr.modify(|_, w| w.uif().clear_bit());
        
        unsafe {
            if let Some(ref audio) = AUDIO_PWM {
                PHASE_ACCUMULATOR = PHASE_ACCUMULATOR.wrapping_add(FREQUENCY_STEP);
                let sample_index = (PHASE_ACCUMULATOR >> 8) as usize;
                let sample_value = audio.wave_table[sample_index];
                
                // 更新PWM占空比
                set_pwm_duty_cycle(1, sample_value as f32 / 255.0 * 100.0);
            }
        }
    }
}
```

## PWM性能优化

### DMA驱动的PWM

```rust
use stm32f4xx_hal::pac::DMA2;

fn setup_pwm_dma() {
    let tim1 = unsafe { &*TIM1::ptr() };
    let dma2 = unsafe { &*DMA2::ptr() };
    
    // 使能定时器DMA请求
    tim1.dier.modify(|_, w| {
        w.cc1de().set_bit()    // 通道1 DMA请求
         .cc2de().set_bit()    // 通道2 DMA请求
         .cc3de().set_bit()    // 通道3 DMA请求
         .cc4de().set_bit()    // 通道4 DMA请求
    });
    
    // 配置DMA流（以Stream5为例，用于TIM1_CH1）
    dma2.s5cr.modify(|_, w| unsafe {
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
    
    // 设置DMA地址
    dma2.s5par.write(|w| unsafe { 
        w.bits(&tim1.ccr1 as *const _ as u32) 
    });
    
    // PWM数据缓冲区将在运行时设置
    // dma2.s5m0ar.write(...);
    // dma2.s5ndtr.write(...);
}

// PWM波形数据缓冲区
static PWM_BUFFER: [u32; 1000] = [0; 1000];

fn start_pwm_dma_transfer() {
    let dma2 = unsafe { &*DMA2::ptr() };
    
    // 设置内存地址和传输长度
    dma2.s5m0ar.write(|w| unsafe { 
        w.bits(PWM_BUFFER.as_ptr() as u32) 
    });
    dma2.s5ndtr.write(|w| unsafe { 
        w.bits(PWM_BUFFER.len() as u16) 
    });
    
    // 启动DMA传输
    dma2.s5cr.modify(|_, w| w.en().set_bit());
}
```

### 高分辨率PWM

```rust
// 使用高级定时器实现高分辨率PWM
fn configure_high_resolution_pwm() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 使用最高时钟频率
    tim1.psc.write(|w| unsafe { w.bits(0) }); // 无预分频
    tim1.arr.write(|w| unsafe { w.bits(65535) }); // 最大分辨率
    
    // 配置PWM模式
    tim1.ccmr1.modify(|_, w| unsafe {
        w.oc1m().bits(0b110)   // PWM模式1
         .oc1pe().set_bit()    // 预装载使能
    });
    
    // 使能输出
    tim1.ccer.modify(|_, w| w.cc1e().set_bit());
    tim1.bdtr.modify(|_, w| w.moe().set_bit());
    
    // 启动定时器
    tim1.cr1.modify(|_, w| w.cen().set_bit());
}

// 设置高精度占空比
fn set_high_precision_duty(duty_16bit: u16) {
    let tim1 = unsafe { &*TIM1::ptr() };
    tim1.ccr1.write(|w| unsafe { w.bits(duty_16bit as u32) });
}
```

## 调试和测量

### PWM信号测量

```rust
// PWM信号参数测量
#[derive(Debug)]
struct PWMSignalInfo {
    frequency_hz: f32,
    duty_cycle_percent: f32,
    period_us: f32,
    pulse_width_us: f32,
}

fn measure_pwm_signal() -> PWMSignalInfo {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 读取PWM输入捕获值
    let (period_counts, pulse_counts) = read_pwm_input();
    
    // 计算定时器分辨率
    let timer_freq = 84_000_000 / ((tim2.psc.read().bits() + 1) as u32);
    let us_per_count = 1_000_000.0 / timer_freq as f32;
    
    let period_us = period_counts as f32 * us_per_count;
    let pulse_width_us = pulse_counts as f32 * us_per_count;
    
    PWMSignalInfo {
        frequency_hz: 1_000_000.0 / period_us,
        duty_cycle_percent: pulse_width_us / period_us * 100.0,
        period_us,
        pulse_width_us,
    }
}
```

### PWM质量分析

```rust
// PWM信号质量分析
struct PWMQualityAnalyzer {
    samples: [u32; 100],
    sample_index: usize,
    min_period: u32,
    max_period: u32,
    total_period: u64,
    sample_count: u32,
}

impl PWMQualityAnalyzer {
    fn new() -> Self {
        Self {
            samples: [0; 100],
            sample_index: 0,
            min_period: u32::MAX,
            max_period: 0,
            total_period: 0,
            sample_count: 0,
        }
    }
    
    fn add_sample(&mut self, period: u32) {
        self.samples[self.sample_index] = period;
        self.sample_index = (self.sample_index + 1) % self.samples.len();
        
        self.min_period = self.min_period.min(period);
        self.max_period = self.max_period.max(period);
        self.total_period += period as u64;
        self.sample_count += 1;
    }
    
    fn get_jitter(&self) -> u32 {
        self.max_period - self.min_period
    }
    
    fn get_average_period(&self) -> f32 {
        if self.sample_count > 0 {
            self.total_period as f32 / self.sample_count as f32
        } else {
            0.0
        }
    }
}
```

## 总结

PWM信号生成是嵌入式系统中的重要技术，STM32定时器提供了丰富的PWM功能。通过合理的配置和优化，可以实现高精度、高性能的PWM控制，满足各种应用需求。

掌握PWM的基本原理、配置方法和高级功能，是开发电机控制、LED驱动、音频生成等应用的基础。在实际应用中，需要根据具体需求选择合适的定时器、配置参数和优化策略。

## 参考资料

- STM32F4xx Reference Manual
- AN4776: General-purpose Timer Cookbook
- AN4013: STM32 Cross-series Timer Overview
- STM32F4xx HAL Driver User Manual