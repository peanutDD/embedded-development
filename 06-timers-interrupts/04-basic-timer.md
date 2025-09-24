# 基础定时器

## 概述

基础定时器（Basic Timer）是STM32中最简单的定时器类型，主要包括TIM6和TIM7。虽然功能相对简单，但在系统时基生成、DAC/ADC触发、精确延时等应用中发挥着重要作用。本章将详细介绍基础定时器的特性、配置方法和实际应用。

## 基础定时器特性

### 硬件特性

- **16位向上计数器**
- **16位可编程预分频器**
- **自动重载功能**
- **更新事件生成**
- **DMA请求生成**
- **用作其他定时器的触发源**

### 功能限制

基础定时器不支持：
- 输入捕获
- 输出比较
- PWM生成
- 编码器接口
- 互补输出

### 寄存器结构

```rust
// 基础定时器寄存器映射
pub struct TIM6 {
    pub cr1: CR1,      // 控制寄存器1
    pub cr2: CR2,      // 控制寄存器2
    pub dier: DIER,    // DMA/中断使能寄存器
    pub sr: SR,        // 状态寄存器
    pub egr: EGR,      // 事件生成寄存器
    pub cnt: CNT,      // 计数器
    pub psc: PSC,      // 预分频器
    pub arr: ARR,      // 自动重载寄存器
}
```

## 基础定时器配置

### 时钟使能

```rust
use stm32f4xx_hal::pac::{RCC, TIM6, TIM7};

fn enable_basic_timer_clocks() {
    let rcc = unsafe { &*RCC::ptr() };
    
    // 使能TIM6和TIM7时钟
    rcc.apb1enr.modify(|_, w| {
        w.tim6en().set_bit()
         .tim7en().set_bit()
    });
}
```

### 基本配置

```rust
fn configure_basic_timer(frequency_hz: u32) {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    // 计算预分频器和自动重载值
    // 假设APB1时钟为84MHz
    const APB1_TIMER_CLOCK: u32 = 84_000_000;
    
    let (prescaler, auto_reload) = calculate_timer_values(
        APB1_TIMER_CLOCK, 
        frequency_hz
    );
    
    // 配置预分频器
    tim6.psc.write(|w| unsafe { w.bits(prescaler) });
    
    // 配置自动重载值
    tim6.arr.write(|w| unsafe { w.bits(auto_reload) });
    
    // 生成更新事件以加载预分频器值
    tim6.egr.write(|w| w.ug().set_bit());
    
    // 清除更新标志
    tim6.sr.modify(|_, w| w.uif().clear_bit());
}

fn calculate_timer_values(timer_clock: u32, target_freq: u32) -> (u16, u16) {
    let total_count = timer_clock / target_freq;
    
    // 寻找最佳的预分频器和自动重载值组合
    for prescaler in 1..=65536u32 {
        let arr = total_count / prescaler;
        if arr <= 65536 && arr > 0 {
            return ((prescaler - 1) as u16, (arr - 1) as u16);
        }
    }
    
    // 默认值（1Hz）
    (83999, 999)
}
```

### 中断配置

```rust
use cortex_m_rt::interrupt;

fn enable_timer_interrupt() {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    // 使能更新中断
    tim6.dier.modify(|_, w| w.uie().set_bit());
    
    // 在NVIC中使能TIM6中断
    unsafe {
        cortex_m::peripheral::NVIC::unmask(
            stm32f4xx_hal::pac::Interrupt::TIM6_DAC
        );
    }
}

// 中断处理程序
#[interrupt]
fn TIM6_DAC() {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    // 检查更新中断标志
    if tim6.sr.read().uif().bit_is_set() {
        // 清除中断标志
        tim6.sr.modify(|_, w| w.uif().clear_bit());
        
        // 执行定时任务
        handle_timer_tick();
    }
}

static mut TICK_COUNTER: u32 = 0;

fn handle_timer_tick() {
    unsafe {
        TICK_COUNTER += 1;
        
        // 每1000次tick执行一次任务（假设1ms tick）
        if TICK_COUNTER % 1000 == 0 {
            // 1秒任务
            perform_second_task();
        }
        
        if TICK_COUNTER % 100 == 0 {
            // 100ms任务
            perform_100ms_task();
        }
    }
}
```

## 应用场景

### 1. 系统时基生成

```rust
// 系统时基管理
static mut SYSTEM_TICKS: u32 = 0;

fn init_system_timebase() {
    // 配置1ms时基
    configure_basic_timer(1000); // 1kHz = 1ms
    enable_timer_interrupt();
    start_timer();
}

#[interrupt]
fn TIM6_DAC() {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    if tim6.sr.read().uif().bit_is_set() {
        tim6.sr.modify(|_, w| w.uif().clear_bit());
        
        unsafe {
            SYSTEM_TICKS += 1;
        }
    }
}

// 获取系统时间（毫秒）
fn get_system_time_ms() -> u32 {
    unsafe { SYSTEM_TICKS }
}

// 延时函数
fn delay_ms(ms: u32) {
    let start_time = get_system_time_ms();
    while get_system_time_ms() - start_time < ms {
        // 等待
    }
}
```

### 2. DAC触发源

```rust
use stm32f4xx_hal::pac::DAC;

fn setup_dac_with_timer_trigger() {
    let tim6 = unsafe { &*TIM6::ptr() };
    let dac = unsafe { &*DAC::ptr() };
    
    // 配置定时器为DAC触发源
    // TIM6更新事件 -> TRGO
    tim6.cr2.modify(|_, w| unsafe {
        w.mms().bits(0b010)  // 更新事件作为触发输出
    });
    
    // 配置DAC使用TIM6作为触发源
    dac.cr.modify(|_, w| unsafe {
        w.ten1().set_bit()      // 使能触发
         .tsel1().bits(0b000)   // 选择TIM6 TRGO
    });
    
    // 配置定时器频率（例如：44.1kHz音频采样率）
    configure_basic_timer(44100);
    start_timer();
}
```

### 3. ADC定时转换

```rust
use stm32f4xx_hal::pac::ADC1;

fn setup_adc_with_timer_trigger() {
    let tim7 = unsafe { &*TIM7::ptr() };
    let adc1 = unsafe { &*ADC1::ptr() };
    
    // 配置TIM7为ADC触发源
    tim7.cr2.modify(|_, w| unsafe {
        w.mms().bits(0b010)  // 更新事件作为触发输出
    });
    
    // 配置ADC使用TIM7作为触发源
    adc1.cr2.modify(|_, w| unsafe {
        w.exten().bits(0b01)    // 上升沿触发
         .extsel().bits(0b1011) // TIM7 TRGO
    });
    
    // 配置采样频率（例如：1kHz）
    configure_timer_7(1000);
    start_timer_7();
}
```

### 4. 精确延时

```rust
// 阻塞式精确延时
fn precise_delay_us(microseconds: u32) {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    // 停止定时器
    tim6.cr1.modify(|_, w| w.cen().clear_bit());
    
    // 配置为微秒计数
    // 84MHz / 84 = 1MHz = 1us per count
    tim6.psc.write(|w| unsafe { w.bits(83) });
    tim6.arr.write(|w| unsafe { w.bits(microseconds - 1) });
    
    // 重置计数器
    tim6.cnt.write(|w| unsafe { w.bits(0) });
    
    // 清除更新标志
    tim6.sr.modify(|_, w| w.uif().clear_bit());
    
    // 启动定时器
    tim6.cr1.modify(|_, w| w.cen().set_bit());
    
    // 等待计数完成
    while !tim6.sr.read().uif().bit_is_set() {
        // 等待
    }
    
    // 停止定时器
    tim6.cr1.modify(|_, w| w.cen().clear_bit());
}

// 非阻塞式延时
static mut DELAY_ACTIVE: bool = false;
static mut DELAY_CALLBACK: Option<fn()> = None;

fn start_delay_us(microseconds: u32, callback: fn()) {
    unsafe {
        if DELAY_ACTIVE {
            return; // 延时已在进行中
        }
        
        DELAY_ACTIVE = true;
        DELAY_CALLBACK = Some(callback);
    }
    
    let tim6 = unsafe { &*TIM6::ptr() };
    
    // 配置定时器
    tim6.psc.write(|w| unsafe { w.bits(83) }); // 1us per count
    tim6.arr.write(|w| unsafe { w.bits(microseconds - 1) });
    tim6.cnt.write(|w| unsafe { w.bits(0) });
    
    // 使能中断
    tim6.dier.modify(|_, w| w.uie().set_bit());
    tim6.sr.modify(|_, w| w.uif().clear_bit());
    
    // 启动定时器
    tim6.cr1.modify(|_, w| w.cen().set_bit());
}

#[interrupt]
fn TIM6_DAC() {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    if tim6.sr.read().uif().bit_is_set() {
        tim6.sr.modify(|_, w| w.uif().clear_bit());
        tim6.cr1.modify(|_, w| w.cen().clear_bit());
        
        unsafe {
            DELAY_ACTIVE = false;
            if let Some(callback) = DELAY_CALLBACK {
                callback();
                DELAY_CALLBACK = None;
            }
        }
    }
}
```

### 5. 看门狗喂狗定时器

```rust
use stm32f4xx_hal::pac::IWDG;

fn setup_watchdog_timer() {
    let tim7 = unsafe { &*TIM7::ptr() };
    let iwdg = unsafe { &*IWDG::ptr() };
    
    // 配置看门狗
    iwdg.kr.write(|w| unsafe { w.bits(0x5555) }); // 解锁
    iwdg.pr.write(|w| unsafe { w.bits(0x06) });   // 预分频器 /256
    iwdg.rlr.write(|w| unsafe { w.bits(1250) });  // 重载值 (约10秒)
    iwdg.kr.write(|w| unsafe { w.bits(0xCCCC) }); // 启动看门狗
    
    // 配置定时器每5秒喂狗一次
    configure_timer_7(1); // 1Hz
    enable_timer_7_interrupt();
    start_timer_7();
}

static mut WATCHDOG_COUNTER: u32 = 0;

#[interrupt]
fn TIM7() {
    let tim7 = unsafe { &*TIM7::ptr() };
    
    if tim7.sr.read().uif().bit_is_set() {
        tim7.sr.modify(|_, w| w.uif().clear_bit());
        
        unsafe {
            WATCHDOG_COUNTER += 1;
            
            // 每5秒喂狗一次
            if WATCHDOG_COUNTER >= 5 {
                WATCHDOG_COUNTER = 0;
                feed_watchdog();
            }
        }
    }
}

fn feed_watchdog() {
    let iwdg = unsafe { &*IWDG::ptr() };
    iwdg.kr.write(|w| unsafe { w.bits(0xAAAA) }); // 喂狗
}
```

## DMA支持

### DMA触发配置

```rust
use stm32f4xx_hal::pac::DMA1;

fn setup_timer_dma_trigger() {
    let tim6 = unsafe { &*TIM6::ptr() };
    let dma1 = unsafe { &*DMA1::ptr() };
    
    // 使能定时器DMA请求
    tim6.dier.modify(|_, w| w.ude().set_bit());
    
    // 配置DMA通道（以DMA1 Stream1为例）
    dma1.s1cr.modify(|_, w| unsafe {
        w.chsel().bits(0b000)   // 通道0
         .mburst().bits(0b00)   // 单次传输
         .pburst().bits(0b00)   // 单次传输
         .pl().bits(0b01)       // 中等优先级
         .msize().bits(0b10)    // 32位内存数据大小
         .psize().bits(0b10)    // 32位外设数据大小
         .minc().set_bit()      // 内存地址递增
         .pinc().clear_bit()    // 外设地址不递增
         .circ().set_bit()      // 循环模式
         .dir().bits(0b01)      // 内存到外设
    });
    
    // 设置DMA地址和数据长度
    // dma1.s1par.write(...);  // 外设地址
    // dma1.s1m0ar.write(...); // 内存地址
    // dma1.s1ndtr.write(...); // 数据长度
    
    // 使能DMA流
    dma1.s1cr.modify(|_, w| w.en().set_bit());
}
```

## 定时器控制函数

### 启动和停止

```rust
fn start_timer() {
    let tim6 = unsafe { &*TIM6::ptr() };
    tim6.cr1.modify(|_, w| w.cen().set_bit());
}

fn stop_timer() {
    let tim6 = unsafe { &*TIM6::ptr() };
    tim6.cr1.modify(|_, w| w.cen().clear_bit());
}

fn reset_timer() {
    let tim6 = unsafe { &*TIM6::ptr() };
    tim6.cnt.write(|w| unsafe { w.bits(0) });
}

fn is_timer_running() -> bool {
    let tim6 = unsafe { &*TIM6::ptr() };
    tim6.cr1.read().cen().bit_is_set()
}
```

### 单次模式

```rust
fn configure_one_shot_mode() {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    // 使能单次模式
    tim6.cr1.modify(|_, w| w.opm().set_bit());
    
    // 配置定时时间
    tim6.psc.write(|w| unsafe { w.bits(83999) }); // 1ms per count
    tim6.arr.write(|w| unsafe { w.bits(999) });   // 1秒延时
    
    // 清除标志并启动
    tim6.sr.modify(|_, w| w.uif().clear_bit());
    tim6.cr1.modify(|_, w| w.cen().set_bit());
}
```

## 调试和监控

### 定时器状态监控

```rust
#[derive(Debug)]
struct TimerStatus {
    enabled: bool,
    counter_value: u16,
    prescaler: u16,
    auto_reload: u16,
    interrupt_flag: bool,
}

fn get_timer_status() -> TimerStatus {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    TimerStatus {
        enabled: tim6.cr1.read().cen().bit_is_set(),
        counter_value: tim6.cnt.read().bits() as u16,
        prescaler: tim6.psc.read().bits() as u16,
        auto_reload: tim6.arr.read().bits() as u16,
        interrupt_flag: tim6.sr.read().uif().bit_is_set(),
    }
}

fn print_timer_status() {
    let status = get_timer_status();
    rprintln!("Timer Status: {:?}", status);
}
```

### 性能测量

```rust
static mut TIMER_PERFORMANCE: TimerPerformance = TimerPerformance::new();

struct TimerPerformance {
    interrupt_count: u32,
    max_latency: u32,
    min_latency: u32,
    total_latency: u32,
}

impl TimerPerformance {
    const fn new() -> Self {
        Self {
            interrupt_count: 0,
            max_latency: 0,
            min_latency: u32::MAX,
            total_latency: 0,
        }
    }
    
    fn record_interrupt(&mut self, latency: u32) {
        self.interrupt_count += 1;
        self.total_latency += latency;
        
        if latency > self.max_latency {
            self.max_latency = latency;
        }
        if latency < self.min_latency {
            self.min_latency = latency;
        }
    }
    
    fn get_average_latency(&self) -> u32 {
        if self.interrupt_count > 0 {
            self.total_latency / self.interrupt_count
        } else {
            0
        }
    }
}
```

## 最佳实践

### 1. 时钟配置优化

```rust
// 根据应用需求选择合适的时钟频率
fn optimize_timer_clock() {
    // 对于低功耗应用，使用较低的时钟频率
    // 对于高精度应用，使用较高的时钟频率
    
    let target_resolution_us = 10; // 10微秒分辨率
    let timer_clock = 84_000_000;  // 84MHz
    
    let prescaler = timer_clock / (1_000_000 / target_resolution_us);
    
    // 配置预分频器以获得所需分辨率
    let tim6 = unsafe { &*TIM6::ptr() };
    tim6.psc.write(|w| unsafe { w.bits((prescaler - 1) as u16) });
}
```

### 2. 中断优化

```rust
// 最小化中断处理时间
#[interrupt]
fn TIM6_DAC() {
    let tim6 = unsafe { &*TIM6::ptr() };
    
    // 立即清除中断标志
    if tim6.sr.read().uif().bit_is_set() {
        tim6.sr.modify(|_, w| w.uif().clear_bit());
        
        // 只做必要的处理
        increment_tick_counter();
        
        // 复杂处理推迟到主循环
        set_pending_work_flag();
    }
}
```

### 3. 资源管理

```rust
// 使用RAII模式管理定时器资源
struct BasicTimer {
    _timer: TIM6,
}

impl BasicTimer {
    fn new(frequency: u32) -> Self {
        enable_basic_timer_clocks();
        configure_basic_timer(frequency);
        
        Self {
            _timer: unsafe { core::ptr::read(TIM6::ptr()) },
        }
    }
    
    fn start(&self) {
        start_timer();
    }
    
    fn stop(&self) {
        stop_timer();
    }
}

impl Drop for BasicTimer {
    fn drop(&mut self) {
        // 清理资源
        stop_timer();
        disable_timer_interrupt();
    }
}
```

## 总结

基础定时器虽然功能简单，但在嵌入式系统中扮演着重要角色。通过合理的配置和使用，可以实现精确的时基生成、外设触发、延时控制等功能。掌握基础定时器的使用方法，为后续学习更复杂的定时器功能奠定了基础。

## 参考资料

- STM32F4xx Reference Manual
- STM32F4xx HAL Driver User Manual
- ARM Cortex-M4 Programming Guide
- STM32 Timer Cookbook