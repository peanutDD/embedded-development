# 输入捕获技术

## 概述

输入捕获（Input Capture）是定时器的一项重要功能，用于精确测量外部信号的时间特性。通过捕获外部信号的边沿（上升沿、下降沿或双边沿），可以测量信号的周期、频率、脉宽、占空比等参数。这项技术在频率测量、编码器读取、超声波测距、PWM信号分析等应用中发挥重要作用。

## 输入捕获原理

### 基本工作原理

输入捕获的工作原理基于以下机制：
1. **信号调理**：外部信号通过GPIO引脚输入，经过施密特触发器整形
2. **边沿检测**：检测信号的上升沿、下降沿或双边沿
3. **计数器捕获**：在检测到指定边沿时，将当前定时器计数值存储到捕获寄存器
4. **中断生成**：可选择在捕获事件发生时产生中断

```
外部信号 ──→ GPIO ──→ 施密特触发器 ──→ 边沿检测 ──→ 捕获寄存器
                                           ↓
                                      定时器计数器
                                           ↓
                                      中断控制器
```

### 捕获模式分类

STM32定时器支持多种输入捕获模式：

1. **直接模式**：输入信号直接连接到捕获通道
2. **间接模式**：输入信号通过交叉连接到其他通道
3. **TRC模式**：使用内部触发信号作为捕获源
4. **PWM输入模式**：同时测量周期和脉宽

## 基本配置

### GPIO配置

```rust
use stm32f4xx_hal::pac::{GPIOA, RCC, TIM2};

fn configure_input_capture_gpio() {
    let rcc = unsafe { &*RCC::ptr() };
    let gpioa = unsafe { &*GPIOA::ptr() };
    
    // 使能GPIOA时钟
    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
    
    // 配置PA0为TIM2_CH1输入
    gpioa.moder.modify(|_, w| unsafe {
        w.moder0().bits(0b10)  // 复用功能模式
    });
    
    gpioa.pupdr.modify(|_, w| unsafe {
        w.pupdr0().bits(0b10)  // 下拉
    });
    
    // 设置复用功能为AF1 (TIM2)
    gpioa.afrl.modify(|_, w| unsafe {
        w.afrl0().bits(0b0001)
    });
}
```

### 基本输入捕获配置

```rust
fn configure_basic_input_capture() {
    let rcc = unsafe { &*RCC::ptr() };
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 使能TIM2时钟
    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
    
    // 配置定时器基本参数
    tim2.psc.write(|w| unsafe { w.bits(83) });    // 84MHz / 84 = 1MHz
    tim2.arr.write(|w| unsafe { w.bits(0xFFFFFFFF) }); // 最大计数值
    
    // 配置输入捕获通道1
    tim2.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // IC1映射到TI1
         .ic1psc().bits(0b00)  // 无预分频，每个边沿都捕获
         .ic1f().bits(0b0000)  // 无输入滤波
    });
    
    // 配置捕获边沿和使能
    tim2.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能捕获
         .cc1p().clear_bit()   // 上升沿捕获
         .cc1np().clear_bit()  // 不使用下降沿
    });
    
    // 使能捕获中断
    tim2.dier.modify(|_, w| w.cc1ie().set_bit());
    
    // 启动定时器
    tim2.cr1.modify(|_, w| w.cen().set_bit());
}
```

### 多通道输入捕获

```rust
fn configure_multi_channel_capture() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 配置4个通道的输入捕获
    tim2.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // IC1映射到TI1
         .ic1psc().bits(0b00)  // 无预分频
         .ic1f().bits(0b0011)  // 8个采样点滤波
         .cc2s().bits(0b01)    // IC2映射到TI2
         .ic2psc().bits(0b00)  // 无预分频
         .ic2f().bits(0b0011)  // 8个采样点滤波
    });
    
    tim2.ccmr2.modify(|_, w| unsafe {
        w.cc3s().bits(0b01)    // IC3映射到TI3
         .ic3psc().bits(0b00)  // 无预分频
         .ic3f().bits(0b0011)  // 8个采样点滤波
         .cc4s().bits(0b01)    // IC4映射到TI4
         .ic4psc().bits(0b00)  // 无预分频
         .ic4f().bits(0b0011)  // 8个采样点滤波
    });
    
    // 配置不同的捕获边沿
    tim2.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能CH1
         .cc1p().clear_bit()   // CH1上升沿
         .cc2e().set_bit()     // 使能CH2
         .cc2p().set_bit()     // CH2下降沿
         .cc3e().set_bit()     // 使能CH3
         .cc3p().clear_bit()   // CH3上升沿
         .cc4e().set_bit()     // 使能CH4
         .cc4p().set_bit()     // CH4下降沿
    });
    
    // 使能所有通道中断
    tim2.dier.modify(|_, w| {
        w.cc1ie().set_bit()
         .cc2ie().set_bit()
         .cc3ie().set_bit()
         .cc4ie().set_bit()
    });
}
```

## 高级捕获功能

### PWM输入模式

```rust
fn configure_pwm_input_mode() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // PWM输入模式：同时测量周期和脉宽
    tim2.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // IC1映射到TI1（测量周期）
         .ic1psc().bits(0b00)  // 无预分频
         .ic1f().bits(0b0000)  // 无滤波
         .cc2s().bits(0b10)    // IC2映射到TI1（测量脉宽）
         .ic2psc().bits(0b00)  // 无预分频
         .ic2f().bits(0b0000)  // 无滤波
    });
    
    // 配置触发边沿：IC1上升沿，IC2下降沿
    tim2.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能IC1
         .cc1p().clear_bit()   // IC1上升沿触发
         .cc2e().set_bit()     // 使能IC2
         .cc2p().set_bit()     // IC2下降沿触发
    });
    
    // 配置从模式：复位模式，TI1FP1作为触发源
    tim2.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b100)    // 复位模式
         .ts().bits(0b101)     // TI1FP1作为触发
    });
    
    // 使能中断
    tim2.dier.modify(|_, w| {
        w.cc1ie().set_bit()    // 周期捕获中断
         .cc2ie().set_bit()    // 脉宽捕获中断
    });
}

// PWM输入测量结果
#[derive(Debug)]
struct PWMInputResult {
    period: u32,
    pulse_width: u32,
    frequency: f32,
    duty_cycle: f32,
}

fn read_pwm_input_result() -> PWMInputResult {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    let period = tim2.ccr1.read().bits();
    let pulse_width = tim2.ccr2.read().bits();
    
    // 计算频率和占空比
    let timer_freq = 1_000_000.0; // 1MHz定时器频率
    let frequency = timer_freq / period as f32;
    let duty_cycle = pulse_width as f32 / period as f32 * 100.0;
    
    PWMInputResult {
        period,
        pulse_width,
        frequency,
        duty_cycle,
    }
}
```

### 编码器接口模式

```rust
fn configure_encoder_interface() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 配置编码器接口模式
    tim2.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // IC1映射到TI1
         .ic1f().bits(0b1111)  // 最大滤波
         .cc2s().bits(0b01)    // IC2映射到TI2
         .ic2f().bits(0b1111)  // 最大滤波
    });
    
    // 配置编码器模式
    tim2.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b011)    // 编码器模式3（TI1和TI2边沿都计数）
    });
    
    // 配置输入极性
    tim2.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能IC1
         .cc1p().clear_bit()   // IC1正极性
         .cc2e().set_bit()     // 使能IC2
         .cc2p().clear_bit()   // IC2正极性
    });
    
    // 设置自动重载值（编码器计数范围）
    tim2.arr.write(|w| unsafe { w.bits(0xFFFFFFFF) });
    
    // 启动定时器
    tim2.cr1.modify(|_, w| w.cen().set_bit());
}

// 编码器读取
fn read_encoder_position() -> i32 {
    let tim2 = unsafe { &*TIM2::ptr() };
    tim2.cnt.read().bits() as i32
}

fn reset_encoder_position() {
    let tim2 = unsafe { &*TIM2::ptr() };
    tim2.cnt.write(|w| unsafe { w.bits(0) });
}
```

### 输入滤波配置

```rust
// 输入滤波器配置
#[derive(Clone, Copy)]
enum InputFilter {
    NoFilter = 0b0000,
    Fck2N2 = 0b0001,    // fCK_INT, N=2
    Fck2N4 = 0b0010,    // fCK_INT, N=4
    Fck2N8 = 0b0011,    // fCK_INT, N=8
    Fdts2N6 = 0b0100,   // fDTS/2, N=6
    Fdts2N8 = 0b0101,   // fDTS/2, N=8
    Fdts4N6 = 0b0110,   // fDTS/4, N=6
    Fdts4N8 = 0b0111,   // fDTS/4, N=8
    Fdts8N6 = 0b1000,   // fDTS/8, N=6
    Fdts8N8 = 0b1001,   // fDTS/8, N=8
    Fdts16N5 = 0b1010,  // fDTS/16, N=5
    Fdts16N6 = 0b1011,  // fDTS/16, N=6
    Fdts16N8 = 0b1100,  // fDTS/16, N=8
    Fdts32N5 = 0b1101,  // fDTS/32, N=5
    Fdts32N6 = 0b1110,  // fDTS/32, N=6
    Fdts32N8 = 0b1111,  // fDTS/32, N=8
}

fn set_input_filter(channel: u8, filter: InputFilter) {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    match channel {
        1 => tim2.ccmr1.modify(|_, w| unsafe { w.ic1f().bits(filter as u8) }),
        2 => tim2.ccmr1.modify(|_, w| unsafe { w.ic2f().bits(filter as u8) }),
        3 => tim2.ccmr2.modify(|_, w| unsafe { w.ic3f().bits(filter as u8) }),
        4 => tim2.ccmr2.modify(|_, w| unsafe { w.ic4f().bits(filter as u8) }),
        _ => {}
    }
}
```

## 中断处理

### 输入捕获中断处理

```rust
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

// 全局变量存储捕获数据
static CAPTURE_DATA: Mutex<RefCell<CaptureData>> = Mutex::new(RefCell::new(CaptureData::new()));

struct CaptureData {
    last_capture: u32,
    current_capture: u32,
    period: u32,
    frequency: f32,
    capture_count: u32,
}

impl CaptureData {
    const fn new() -> Self {
        Self {
            last_capture: 0,
            current_capture: 0,
            period: 0,
            frequency: 0.0,
            capture_count: 0,
        }
    }
    
    fn update_capture(&mut self, capture_value: u32) {
        self.last_capture = self.current_capture;
        self.current_capture = capture_value;
        self.capture_count += 1;
        
        if self.capture_count > 1 {
            // 计算周期（处理溢出情况）
            self.period = if self.current_capture >= self.last_capture {
                self.current_capture - self.last_capture
            } else {
                (0xFFFFFFFF - self.last_capture) + self.current_capture + 1
            };
            
            // 计算频率
            const TIMER_FREQ: f32 = 1_000_000.0; // 1MHz
            self.frequency = TIMER_FREQ / self.period as f32;
        }
    }
}

#[interrupt]
fn TIM2() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 检查通道1捕获中断
    if tim2.sr.read().cc1if().bit_is_set() {
        tim2.sr.modify(|_, w| w.cc1if().clear_bit());
        
        let capture_value = tim2.ccr1.read().bits();
        
        cortex_m::interrupt::free(|cs| {
            CAPTURE_DATA.borrow(cs).borrow_mut().update_capture(capture_value);
        });
    }
    
    // 检查通道2捕获中断
    if tim2.sr.read().cc2if().bit_is_set() {
        tim2.sr.modify(|_, w| w.cc2if().clear_bit());
        // 处理通道2捕获
    }
    
    // 检查通道3捕获中断
    if tim2.sr.read().cc3if().bit_is_set() {
        tim2.sr.modify(|_, w| w.cc3if().clear_bit());
        // 处理通道3捕获
    }
    
    // 检查通道4捕获中断
    if tim2.sr.read().cc4if().bit_is_set() {
        tim2.sr.modify(|_, w| w.cc4if().clear_bit());
        // 处理通道4捕获
    }
    
    // 检查溢出中断
    if tim2.sr.read().uif().bit_is_set() {
        tim2.sr.modify(|_, w| w.uif().clear_bit());
        // 处理定时器溢出
    }
}

// 获取测量结果
fn get_frequency_measurement() -> f32 {
    cortex_m::interrupt::free(|cs| {
        CAPTURE_DATA.borrow(cs).borrow().frequency
    })
}
```

### DMA驱动的输入捕获

```rust
use stm32f4xx_hal::pac::DMA1;

// DMA缓冲区
static mut CAPTURE_BUFFER: [u32; 100] = [0; 100];

fn setup_input_capture_dma() {
    let tim2 = unsafe { &*TIM2::ptr() };
    let dma1 = unsafe { &*DMA1::ptr() };
    
    // 使能定时器DMA请求
    tim2.dier.modify(|_, w| w.cc1de().set_bit());
    
    // 配置DMA流（Stream5用于TIM2_CH1）
    dma1.s5cr.modify(|_, w| unsafe {
        w.chsel().bits(0b011)   // 通道3 (TIM2_CH1)
         .mburst().bits(0b00)   // 单次传输
         .pburst().bits(0b00)   // 单次传输
         .pl().bits(0b11)       // 最高优先级
         .msize().bits(0b10)    // 32位内存数据
         .psize().bits(0b10)    // 32位外设数据
         .minc().set_bit()      // 内存地址递增
         .pinc().clear_bit()    // 外设地址固定
         .circ().set_bit()      // 循环模式
         .dir().bits(0b00)      // 外设到内存
    });
    
    // 设置DMA地址
    dma1.s5par.write(|w| unsafe { 
        w.bits(&tim2.ccr1 as *const _ as u32) 
    });
    dma1.s5m0ar.write(|w| unsafe { 
        w.bits(CAPTURE_BUFFER.as_ptr() as u32) 
    });
    dma1.s5ndtr.write(|w| unsafe { 
        w.bits(CAPTURE_BUFFER.len() as u16) 
    });
    
    // 启动DMA
    dma1.s5cr.modify(|_, w| w.en().set_bit());
}
```

## 应用实例

### 1. 频率计

```rust
// 高精度频率计
struct FrequencyMeter {
    gate_time_ms: u32,
    pulse_count: u32,
    last_measurement: f32,
    measurement_ready: bool,
}

impl FrequencyMeter {
    fn new(gate_time_ms: u32) -> Self {
        Self {
            gate_time_ms,
            pulse_count: 0,
            last_measurement: 0.0,
            measurement_ready: false,
        }
    }
    
    fn start_measurement(&mut self) {
        self.pulse_count = 0;
        self.measurement_ready = false;
        
        // 配置门控时间定时器
        self.setup_gate_timer();
        
        // 配置输入捕获计数脉冲
        self.setup_pulse_counter();
    }
    
    fn setup_gate_timer(&self) {
        // 使用TIM3作为门控定时器
        let tim3 = unsafe { &*TIM3::ptr() };
        
        // 配置1ms基准
        tim3.psc.write(|w| unsafe { w.bits(83) });  // 84MHz / 84 = 1MHz
        tim3.arr.write(|w| unsafe { w.bits(self.gate_time_ms * 1000 - 1) });
        
        // 使能更新中断
        tim3.dier.modify(|_, w| w.uie().set_bit());
        
        // 启动定时器
        tim3.cr1.modify(|_, w| w.cen().set_bit());
    }
    
    fn setup_pulse_counter(&self) {
        // 使用TIM2作为脉冲计数器
        let tim2 = unsafe { &*TIM2::ptr() };
        
        // 配置外部时钟模式1
        tim2.smcr.modify(|_, w| unsafe {
            w.sms().bits(0b111)    // 外部时钟模式1
             .ts().bits(0b101)     // TI1FP1作为触发
        });
        
        // 配置输入捕获
        tim2.ccmr1.modify(|_, w| unsafe {
            w.cc1s().bits(0b01)    // IC1映射到TI1
             .ic1f().bits(0b0000)  // 无滤波
        });
        
        tim2.ccer.modify(|_, w| {
            w.cc1e().set_bit()     // 使能捕获
             .cc1p().clear_bit()   // 上升沿
        });
        
        // 启动计数器
        tim2.cr1.modify(|_, w| w.cen().set_bit());
    }
    
    fn get_frequency(&self) -> Option<f32> {
        if self.measurement_ready {
            Some(self.last_measurement)
        } else {
            None
        }
    }
}

// 门控定时器中断
#[interrupt]
fn TIM3() {
    let tim3 = unsafe { &*TIM3::ptr() };
    let tim2 = unsafe { &*TIM2::ptr() };
    
    if tim3.sr.read().uif().bit_is_set() {
        tim3.sr.modify(|_, w| w.uif().clear_bit());
        
        // 停止计数器
        tim2.cr1.modify(|_, w| w.cen().clear_bit());
        
        // 读取脉冲计数
        let pulse_count = tim2.cnt.read().bits();
        
        // 计算频率
        let frequency = pulse_count as f32 / (GATE_TIME_MS as f32 / 1000.0);
        
        // 存储结果
        // 这里需要通过全局变量或其他方式传递结果
        
        // 重置计数器
        tim2.cnt.write(|w| unsafe { w.bits(0) });
        
        // 停止门控定时器
        tim3.cr1.modify(|_, w| w.cen().clear_bit());
    }
}
```

### 2. 超声波测距

```rust
// 超声波测距传感器
struct UltrasonicSensor {
    trigger_pin: u8,
    echo_start_time: u32,
    echo_end_time: u32,
    distance_cm: f32,
    measurement_state: MeasurementState,
}

#[derive(PartialEq)]
enum MeasurementState {
    Idle,
    TriggerSent,
    EchoReceiving,
    MeasurementComplete,
}

impl UltrasonicSensor {
    fn new(trigger_pin: u8) -> Self {
        Self {
            trigger_pin,
            echo_start_time: 0,
            echo_end_time: 0,
            distance_cm: 0.0,
            measurement_state: MeasurementState::Idle,
        }
    }
    
    fn start_measurement(&mut self) {
        if self.measurement_state == MeasurementState::Idle {
            self.send_trigger_pulse();
            self.measurement_state = MeasurementState::TriggerSent;
        }
    }
    
    fn send_trigger_pulse(&self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        
        // 发送10us高电平触发脉冲
        gpioa.bsrr.write(|w| unsafe { w.bits(1 << self.trigger_pin) });
        delay_us(10);
        gpioa.bsrr.write(|w| unsafe { w.bits(1 << (self.trigger_pin + 16)) });
    }
    
    fn process_echo_edge(&mut self, capture_time: u32, is_rising_edge: bool) {
        match self.measurement_state {
            MeasurementState::TriggerSent => {
                if is_rising_edge {
                    self.echo_start_time = capture_time;
                    self.measurement_state = MeasurementState::EchoReceiving;
                }
            },
            MeasurementState::EchoReceiving => {
                if !is_rising_edge {
                    self.echo_end_time = capture_time;
                    self.calculate_distance();
                    self.measurement_state = MeasurementState::MeasurementComplete;
                }
            },
            _ => {}
        }
    }
    
    fn calculate_distance(&mut self) {
        // 计算回声时间（处理溢出）
        let echo_time = if self.echo_end_time >= self.echo_start_time {
            self.echo_end_time - self.echo_start_time
        } else {
            (0xFFFFFFFF - self.echo_start_time) + self.echo_end_time + 1
        };
        
        // 转换为微秒（假设定时器频率为1MHz）
        let echo_time_us = echo_time as f32;
        
        // 计算距离：距离 = (时间 * 声速) / 2
        // 声速约为343m/s = 0.0343cm/us
        self.distance_cm = (echo_time_us * 0.0343) / 2.0;
    }
    
    fn get_distance(&mut self) -> Option<f32> {
        if self.measurement_state == MeasurementState::MeasurementComplete {
            self.measurement_state = MeasurementState::Idle;
            Some(self.distance_cm)
        } else {
            None
        }
    }
}

// 配置超声波传感器的输入捕获
fn configure_ultrasonic_capture() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 配置双边沿捕获
    tim2.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // IC1映射到TI1
         .ic1f().bits(0b0011)  // 适度滤波
    });
    
    // 配置双边沿触发
    tim2.ccer.modify(|_, w| {
        w.cc1e().set_bit()     // 使能捕获
         .cc1p().set_bit()     // 下降沿
         .cc1np().set_bit()    // 上升沿
    });
    
    // 使能中断
    tim2.dier.modify(|_, w| w.cc1ie().set_bit());
}
```

### 3. 编码器读取

```rust
// 增量式编码器接口
struct IncrementalEncoder {
    resolution: u32,        // 编码器分辨率（PPR）
    position: i32,          // 当前位置
    last_count: u32,        // 上次计数值
    velocity: f32,          // 角速度 (RPM)
    last_time: u32,         // 上次测量时间
}

impl IncrementalEncoder {
    fn new(resolution: u32) -> Self {
        Self {
            resolution,
            position: 0,
            last_count: 0,
            velocity: 0.0,
            last_time: 0,
        }
    }
    
    fn update(&mut self) {
        let tim2 = unsafe { &*TIM2::ptr() };
        let current_count = tim2.cnt.read().bits();
        let current_time = get_system_time_ms();
        
        // 计算位置变化
        let count_diff = current_count.wrapping_sub(self.last_count) as i32;
        self.position += count_diff;
        
        // 计算速度
        if current_time > self.last_time {
            let time_diff = current_time - self.last_time;
            let rps = count_diff as f32 / (self.resolution as f32 * time_diff as f32 / 1000.0);
            self.velocity = rps * 60.0; // 转换为RPM
        }
        
        self.last_count = current_count;
        self.last_time = current_time;
    }
    
    fn get_position(&self) -> i32 {
        self.position
    }
    
    fn get_angle_degrees(&self) -> f32 {
        (self.position as f32 / self.resolution as f32) * 360.0
    }
    
    fn get_velocity_rpm(&self) -> f32 {
        self.velocity
    }
    
    fn reset_position(&mut self) {
        self.position = 0;
        let tim2 = unsafe { &*TIM2::ptr() };
        tim2.cnt.write(|w| unsafe { w.bits(0) });
    }
}
```

### 4. 信号质量分析

```rust
// 信号质量分析器
struct SignalQualityAnalyzer {
    samples: [u32; 1000],
    sample_index: usize,
    min_period: u32,
    max_period: u32,
    average_period: f32,
    jitter: f32,
    sample_count: u32,
}

impl SignalQualityAnalyzer {
    fn new() -> Self {
        Self {
            samples: [0; 1000],
            sample_index: 0,
            min_period: u32::MAX,
            max_period: 0,
            average_period: 0.0,
            jitter: 0.0,
            sample_count: 0,
        }
    }
    
    fn add_sample(&mut self, period: u32) {
        // 存储样本
        self.samples[self.sample_index] = period;
        self.sample_index = (self.sample_index + 1) % self.samples.len();
        
        // 更新统计信息
        self.min_period = self.min_period.min(period);
        self.max_period = self.max_period.max(period);
        self.sample_count += 1;
        
        // 计算平均值
        if self.sample_count <= self.samples.len() as u32 {
            let sum: u64 = self.samples[..self.sample_count as usize].iter()
                .map(|&x| x as u64).sum();
            self.average_period = sum as f32 / self.sample_count as f32;
        } else {
            let sum: u64 = self.samples.iter().map(|&x| x as u64).sum();
            self.average_period = sum as f32 / self.samples.len() as f32;
        }
        
        // 计算抖动
        self.jitter = (self.max_period - self.min_period) as f32;
    }
    
    fn get_statistics(&self) -> SignalStatistics {
        SignalStatistics {
            min_period: self.min_period,
            max_period: self.max_period,
            average_period: self.average_period,
            jitter_ns: self.jitter * 1000.0, // 转换为纳秒
            frequency_hz: 1_000_000.0 / self.average_period,
            stability_percent: (1.0 - self.jitter / self.average_period) * 100.0,
        }
    }
    
    fn reset(&mut self) {
        self.sample_index = 0;
        self.min_period = u32::MAX;
        self.max_period = 0;
        self.average_period = 0.0;
        self.jitter = 0.0;
        self.sample_count = 0;
        self.samples.fill(0);
    }
}

#[derive(Debug)]
struct SignalStatistics {
    min_period: u32,
    max_period: u32,
    average_period: f32,
    jitter_ns: f32,
    frequency_hz: f32,
    stability_percent: f32,
}
```

## 性能优化

### 高速捕获优化

```rust
// 高速输入捕获优化
fn optimize_high_speed_capture() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 使用最高时钟频率
    tim2.psc.write(|w| unsafe { w.bits(0) }); // 无预分频，84MHz
    
    // 最小滤波以减少延迟
    tim2.ccmr1.modify(|_, w| unsafe {
        w.ic1f().bits(0b0001)  // 最小滤波
    });
    
    // 使用DMA减少中断开销
    setup_input_capture_dma();
    
    // 配置高优先级中断
    unsafe {
        let mut nvic = cortex_m::peripheral::NVIC::steal();
        nvic.set_priority(stm32f4xx_hal::pac::Interrupt::TIM2, 0);
    }
}
```

### 多通道同步捕获

```rust
// 多通道同步捕获
fn configure_synchronized_capture() {
    let tim1 = unsafe { &*TIM1::ptr() };
    
    // 配置所有通道同时捕获
    tim1.ccmr1.modify(|_, w| unsafe {
        w.cc1s().bits(0b01)    // IC1映射到TI1
         .cc2s().bits(0b01)    // IC2映射到TI2
    });
    
    tim1.ccmr2.modify(|_, w| unsafe {
        w.cc3s().bits(0b01)    // IC3映射到TI3
         .cc4s().bits(0b01)    // IC4映射到TI4
    });
    
    // 使用外部触发同步所有通道
    tim1.smcr.modify(|_, w| unsafe {
        w.sms().bits(0b110)    // 触发模式
         .ts().bits(0b000)     // ITR0作为触发源
    });
    
    // 使能所有通道
    tim1.ccer.modify(|_, w| {
        w.cc1e().set_bit()
         .cc2e().set_bit()
         .cc3e().set_bit()
         .cc4e().set_bit()
    });
}
```

## 调试和测试

### 捕获数据验证

```rust
// 输入捕获数据验证
fn validate_capture_data(captures: &[u32]) -> bool {
    if captures.len() < 2 {
        return false;
    }
    
    // 检查数据合理性
    for window in captures.windows(2) {
        let period = if window[1] >= window[0] {
            window[1] - window[0]
        } else {
            (0xFFFFFFFF - window[0]) + window[1] + 1
        };
        
        // 检查周期是否在合理范围内
        if period < 100 || period > 1_000_000 {
            return false;
        }
    }
    
    true
}

// 捕获精度测试
fn test_capture_accuracy() {
    // 生成已知频率的测试信号
    // 比较测量结果与期望值
    let expected_frequency = 1000.0; // 1kHz
    let measured_frequency = get_frequency_measurement();
    let error_percent = ((measured_frequency - expected_frequency) / expected_frequency).abs() * 100.0;
    
    assert!(error_percent < 0.1, "Capture accuracy error: {}%", error_percent);
}
```

## 总结

输入捕获技术是定时器的重要功能，通过精确捕获外部信号的时间特性，可以实现频率测量、脉宽测量、编码器读取、超声波测距等多种应用。

掌握输入捕获的配置方法、中断处理、DMA优化等技术，是开发高精度测量和控制系统的基础。在实际应用中，需要根据信号特性选择合适的滤波、触发边沿和处理策略，以获得最佳的测量精度和系统性能。

## 参考资料

- STM32F4xx Reference Manual
- AN4776: General-purpose Timer Cookbook  
- AN4013: STM32 Cross-series Timer Overview
- STM32F4xx HAL Driver User Manual