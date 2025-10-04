# 中断系统原理

## 概述

中断系统是嵌入式系统中最重要的机制之一，它允许处理器暂停当前执行的程序，转而处理更紧急的事件。理解中断系统的工作原理对于开发高效、可靠的嵌入式应用至关重要。

## 学习目标

完成本章节后，你将掌握：
- 中断的基本概念和工作原理
- ARM Cortex-M中断控制器(NVIC)的使用
- 中断优先级和嵌套机制
- 中断服务程序的设计原则
- 中断延迟和响应时间的优化

## 1. 中断基础概念

### 1.1 什么是中断

中断是一种异步事件处理机制，当特定事件发生时，处理器会：
1. 保存当前程序状态
2. 跳转到中断服务程序(ISR)
3. 执行中断处理代码
4. 恢复原程序状态并继续执行

### 1.2 中断的分类

#### 按来源分类
- **外部中断**: 来自外部设备的信号
- **内部中断**: 来自处理器内部的事件
- **软件中断**: 由软件指令触发

#### 按优先级分类
- **不可屏蔽中断(NMI)**: 最高优先级，不能被禁用
- **可屏蔽中断**: 可以通过中断屏蔽寄存器禁用

#### 按处理方式分类
- **向量中断**: 每个中断有固定的服务程序地址
- **查询中断**: 通过查询状态寄存器确定中断源

### 1.3 中断的优势

```rust
// 轮询方式 - 浪费CPU资源
loop {
    if gpio_pin_is_high() {
        handle_button_press();
    }
    // CPU一直在检查状态
}

// 中断方式 - 高效利用CPU
#[interrupt]
fn EXTI0() {
    handle_button_press();
    // 只在事件发生时执行
}
```

## 2. ARM Cortex-M中断控制器(NVIC)

### 2.1 NVIC架构

ARM Cortex-M处理器使用嵌套向量中断控制器(NVIC)来管理中断：

```rust
// NVIC寄存器结构
pub struct NVIC {
    pub iser: [u32; 8],    // 中断使能寄存器
    pub icer: [u32; 8],    // 中断清除寄存器
    pub ispr: [u32; 8],    // 中断挂起寄存器
    pub icpr: [u32; 8],    // 中断清除挂起寄存器
    pub iabr: [u32; 8],    // 中断活动寄存器
    pub ipr: [u8; 240],    // 中断优先级寄存器
}
```

### 2.2 中断向量表

```rust
// 向量表定义
#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static RESET_VECTOR: unsafe extern "C" fn() -> ! = Reset;

#[link_section = ".vector_table.exceptions"]
#[no_mangle]
pub static EXCEPTIONS: [Vector; 14] = [
    Vector { handler: NMI },
    Vector { handler: HardFault },
    Vector { handler: MemManage },
    Vector { handler: BusFault },
    Vector { handler: UsageFault },
    // ... 更多异常处理程序
];
```

### 2.3 中断配置

```rust
use cortex_m::peripheral::NVIC;
use stm32f4xx_hal::pac::interrupt;

// 配置中断
fn configure_interrupt() {
    // 设置中断优先级
    unsafe {
        NVIC::unmask(interrupt::TIM2);
        let mut nvic = cortex_m::Peripherals::take().unwrap().NVIC;
        nvic.set_priority(interrupt::TIM2, 1);
    }
}
```

## 3. 中断优先级和嵌套

### 3.1 优先级系统

ARM Cortex-M使用数值越小优先级越高的系统：

```rust
// 优先级配置示例
const HIGH_PRIORITY: u8 = 0;    // 最高优先级
const MEDIUM_PRIORITY: u8 = 5;  // 中等优先级
const LOW_PRIORITY: u8 = 15;    // 最低优先级

fn setup_priorities() {
    unsafe {
        let mut nvic = cortex_m::Peripherals::take().unwrap().NVIC;
        
        // 关键系统中断 - 最高优先级
        nvic.set_priority(interrupt::SysTick, HIGH_PRIORITY);
        
        // 通信中断 - 中等优先级
        nvic.set_priority(interrupt::USART1, MEDIUM_PRIORITY);
        
        // 用户界面中断 - 低优先级
        nvic.set_priority(interrupt::EXTI0, LOW_PRIORITY);
    }
}
```

### 3.2 中断嵌套

```rust
// 中断嵌套示例
#[interrupt]
fn TIM2() {
    // 低优先级中断
    cortex_m::interrupt::free(|_| {
        // 临界区 - 禁用中断
        critical_section_code();
    });
    // 高优先级中断可以在这里打断执行
    non_critical_code();
}

#[interrupt]
fn SysTick() {
    // 高优先级中断 - 可以打断TIM2
    system_tick_handler();
}
```

### 3.3 优先级分组

```rust
use cortex_m::peripheral::SCB;

fn configure_priority_grouping() {
    let mut scb = unsafe { cortex_m::Peripherals::steal().SCB };
    
    // 设置优先级分组
    // 4位抢占优先级，0位子优先级
    scb.set_priority_grouping(0);
}
```

## 4. 中断服务程序设计

### 4.1 ISR设计原则

1. **保持简短**: ISR应该尽可能短小
2. **避免阻塞**: 不要在ISR中使用延时或等待
3. **最小化共享数据访问**: 使用原子操作或临界区
4. **快速响应**: 优先处理紧急任务

### 4.2 ISR实现模式

#### 最小化ISR模式
```rust
use heapless::spsc::{Producer, Consumer, Queue};

static mut QUEUE: Queue<Event, 32> = Queue::new();
static mut PRODUCER: Option<Producer<Event, 32>> = None;
static mut CONSUMER: Option<Consumer<Event, 32>> = None;

#[derive(Clone, Copy)]
enum Event {
    ButtonPress,
    TimerExpired,
    DataReceived(u8),
}

#[interrupt]
fn EXTI0() {
    // 最小化处理 - 只记录事件
    if let Some(ref mut producer) = unsafe { &mut PRODUCER } {
        let _ = producer.enqueue(Event::ButtonPress);
    }
    
    // 清除中断标志
    clear_interrupt_flag();
}

// 主循环中处理事件
fn main_loop() {
    if let Some(ref mut consumer) = unsafe { &mut CONSUMER } {
        while let Some(event) = consumer.dequeue() {
            match event {
                Event::ButtonPress => handle_button_press(),
                Event::TimerExpired => handle_timer(),
                Event::DataReceived(data) => process_data(data),
            }
        }
    }
}
```

#### 分层处理模式
```rust
// 底层ISR - 硬件相关
#[interrupt]
fn USART1() {
    let usart = unsafe { &*USART1::ptr() };
    
    if usart.sr.read().rxne().bit_is_set() {
        let data = usart.dr.read().dr().bits() as u8;
        // 调用中层处理函数
        uart_rx_handler(data);
    }
}

// 中层处理 - 协议处理
fn uart_rx_handler(data: u8) {
    static mut BUFFER: [u8; 256] = [0; 256];
    static mut INDEX: usize = 0;
    
    unsafe {
        BUFFER[INDEX] = data;
        INDEX += 1;
        
        if data == b'\n' || INDEX >= BUFFER.len() {
            // 完整消息接收完成
            process_message(&BUFFER[..INDEX]);
            INDEX = 0;
        }
    }
}

// 高层处理 - 应用逻辑
fn process_message(message: &[u8]) {
    // 解析和处理消息
    match parse_command(message) {
        Ok(cmd) => execute_command(cmd),
        Err(e) => log_error(e),
    }
}
```

### 4.3 中断安全的数据结构

```rust
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

// 原子类型 - 中断安全
static INTERRUPT_COUNT: AtomicU32 = AtomicU32::new(0);
static SYSTEM_READY: AtomicBool = AtomicBool::new(false);

// 互斥锁保护的共享数据
static SHARED_DATA: Mutex<RefCell<Option<SharedState>>> = 
    Mutex::new(RefCell::new(None));

struct SharedState {
    counter: u32,
    buffer: [u8; 64],
}

#[interrupt]
fn TIM2() {
    // 原子操作 - 无需禁用中断
    INTERRUPT_COUNT.fetch_add(1, Ordering::Relaxed);
    
    // 访问共享数据 - 需要临界区
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut shared) = SHARED_DATA.borrow(cs).borrow_mut().as_mut() {
            shared.counter += 1;
        }
    });
}
```

## 5. 中断延迟和响应时间

### 5.1 中断延迟的组成

```rust
// 中断延迟 = 硬件延迟 + 软件延迟
// 硬件延迟: 中断信号到CPU响应
// 软件延迟: 上下文切换 + ISR执行时间

fn measure_interrupt_latency() {
    // 使用GPIO翻转测量延迟
    #[interrupt]
    fn EXTI0() {
        // 立即设置GPIO - 测量硬件延迟
        set_gpio_high();
        
        // ISR处理代码
        handle_interrupt();
        
        // 清除GPIO - 测量总延迟
        set_gpio_low();
    }
}
```

### 5.2 优化中断响应时间

```rust
// 1. 使用内联函数减少调用开销
#[inline(always)]
fn critical_interrupt_handler() {
    // 关键处理代码
}

// 2. 预分配资源避免动态分配
static mut PREALLOCATED_BUFFER: [u8; 1024] = [0; 1024];

// 3. 使用查表法替代复杂计算
const LOOKUP_TABLE: [u16; 256] = [/* 预计算的值 */];

#[interrupt]
fn ADC1() {
    let raw_value = read_adc_value();
    // 使用查表而不是实时计算
    let processed_value = LOOKUP_TABLE[raw_value as usize];
    store_result(processed_value);
}
```

### 5.3 中断抖动分析

```rust
use cortex_m::peripheral::DWT;

struct InterruptStats {
    min_latency: u32,
    max_latency: u32,
    avg_latency: u32,
    jitter: u32,
}

static mut STATS: InterruptStats = InterruptStats {
    min_latency: u32::MAX,
    max_latency: 0,
    avg_latency: 0,
    jitter: 0,
};

#[interrupt]
fn TIM2() {
    let start_time = DWT::cycle_count();
    
    // 中断处理代码
    handle_timer_interrupt();
    
    let end_time = DWT::cycle_count();
    let latency = end_time.wrapping_sub(start_time);
    
    // 更新统计信息
    unsafe {
        STATS.min_latency = STATS.min_latency.min(latency);
        STATS.max_latency = STATS.max_latency.max(latency);
        STATS.jitter = STATS.max_latency - STATS.min_latency;
    }
}
```

## 6. 高级中断技术

### 6.1 中断链和共享中断

```rust
// 中断处理程序链
type InterruptHandler = fn();

struct InterruptChain {
    handlers: [Option<InterruptHandler>; 8],
    count: usize,
}

impl InterruptChain {
    const fn new() -> Self {
        Self {
            handlers: [None; 8],
            count: 0,
        }
    }
    
    fn register(&mut self, handler: InterruptHandler) -> Result<(), &'static str> {
        if self.count >= self.handlers.len() {
            return Err("Too many handlers");
        }
        
        self.handlers[self.count] = Some(handler);
        self.count += 1;
        Ok(())
    }
    
    fn execute(&self) {
        for i in 0..self.count {
            if let Some(handler) = self.handlers[i] {
                handler();
            }
        }
    }
}

static mut TIMER_CHAIN: InterruptChain = InterruptChain::new();

#[interrupt]
fn TIM2() {
    unsafe {
        TIMER_CHAIN.execute();
    }
}
```

### 6.2 软件中断和事件系统

```rust
use heapless::pool::{Pool, Node};

// 事件池管理
static mut MEMORY: [Node<Event>; 16] = [Node::new(); 16];
static mut EVENT_POOL: Pool<Event> = Pool::new();

#[derive(Clone, Copy)]
struct Event {
    event_type: EventType,
    data: u32,
    timestamp: u32,
}

#[derive(Clone, Copy)]
enum EventType {
    Timer,
    Gpio,
    Uart,
    Adc,
}

// 软件中断触发
fn trigger_software_interrupt(event: Event) {
    if let Some(mut node) = unsafe { EVENT_POOL.alloc() } {
        *node = event;
        // 触发软件中断处理事件
        cortex_m::peripheral::SCB::set_pendsv();
    }
}

#[exception]
fn PendSV() {
    // 处理软件中断事件
    while let Some(node) = unsafe { EVENT_POOL.free() } {
        let event = *node;
        process_event(event);
    }
}
```

## 7. 调试和测试

### 7.1 中断调试技术

```rust
// 中断调试辅助宏
macro_rules! debug_interrupt {
    ($name:expr) => {
        #[cfg(debug_assertions)]
        {
            static mut ENTRY_COUNT: u32 = 0;
            unsafe {
                ENTRY_COUNT += 1;
                if ENTRY_COUNT % 1000 == 0 {
                    defmt::info!("{} called {} times", $name, ENTRY_COUNT);
                }
            }
        }
    };
}

#[interrupt]
fn USART1() {
    debug_interrupt!("USART1");
    
    // 中断处理代码
    handle_uart_interrupt();
}
```

### 7.2 中断测试框架

```rust
#[cfg(test)]
mod interrupt_tests {
    use super::*;
    
    #[test]
    fn test_interrupt_priority() {
        // 模拟中断优先级测试
        let mut execution_order = Vec::new();
        
        // 模拟低优先级中断
        simulate_interrupt(interrupt::TIM2, || {
            execution_order.push("TIM2_START");
            // 模拟高优先级中断打断
            simulate_interrupt(interrupt::SysTick, || {
                execution_order.push("SysTick");
            });
            execution_order.push("TIM2_END");
        });
        
        assert_eq!(execution_order, vec!["TIM2_START", "SysTick", "TIM2_END"]);
    }
    
    #[test]
    fn test_interrupt_latency() {
        let start = std::time::Instant::now();
        simulate_interrupt(interrupt::EXTI0, || {
            // 模拟中断处理
        });
        let latency = start.elapsed();
        
        assert!(latency.as_micros() < 10); // 延迟应小于10μs
    }
}
```

## 8. 最佳实践

### 8.1 中断设计原则

1. **最小化ISR**: 只做必要的处理
2. **避免阻塞操作**: 不要在ISR中等待
3. **合理设置优先级**: 根据实时性要求分配
4. **保护共享资源**: 使用适当的同步机制
5. **测量和优化**: 监控中断性能

### 8.2 常见错误和解决方案

```rust
// 错误：在ISR中使用阻塞操作
#[interrupt]
fn BAD_EXAMPLE() {
    // 错误：延时操作
    delay_ms(100);
    
    // 错误：等待操作
    while !ready_flag() {}
    
    // 错误：复杂计算
    let result = complex_calculation();
}

// 正确：最小化ISR
#[interrupt]
fn GOOD_EXAMPLE() {
    // 只记录事件
    record_event(EventType::Interrupt);
    
    // 设置标志供主循环处理
    set_processing_flag();
    
    // 清除中断标志
    clear_interrupt_pending();
}
```

## 9. 性能优化

### 9.1 中断开销分析

```rust
// 测量中断开销
fn measure_interrupt_overhead() {
    let mut measurements = [0u32; 1000];
    
    for i in 0..measurements.len() {
        let start = DWT::cycle_count();
        
        // 触发软件中断
        cortex_m::peripheral::SCB::set_pendsv();
        
        // 等待中断完成
        while cortex_m::peripheral::SCB::icsr.read().pendsvset().bit_is_set() {}
        
        let end = DWT::cycle_count();
        measurements[i] = end.wrapping_sub(start);
    }
    
    // 分析结果
    let avg = measurements.iter().sum::<u32>() / measurements.len() as u32;
    let min = *measurements.iter().min().unwrap();
    let max = *measurements.iter().max().unwrap();
    
    defmt::info!("Interrupt overhead: avg={}, min={}, max={}", avg, min, max);
}
```

### 9.2 中断负载均衡

```rust
// 中断负载监控
struct InterruptLoadMonitor {
    total_time: u32,
    interrupt_time: u32,
    load_percentage: u32,
}

impl InterruptLoadMonitor {
    fn update(&mut self, interrupt_cycles: u32, total_cycles: u32) {
        self.interrupt_time += interrupt_cycles;
        self.total_time += total_cycles;
        
        if self.total_time > 0 {
            self.load_percentage = (self.interrupt_time * 100) / self.total_time;
        }
        
        // 重置计数器防止溢出
        if self.total_time > 1_000_000 {
            self.interrupt_time /= 2;
            self.total_time /= 2;
        }
    }
    
    fn get_load(&self) -> u32 {
        self.load_percentage
    }
}
```

## 10. 总结

中断系统是嵌入式系统的核心机制，正确理解和使用中断对于开发高性能、实时的嵌入式应用至关重要。

### 关键要点

1. **中断原理**: 理解中断的工作机制和分类
2. **NVIC使用**: 掌握ARM Cortex-M中断控制器
3. **优先级管理**: 合理设置中断优先级和嵌套
4. **ISR设计**: 遵循最小化、非阻塞的设计原则
5. **性能优化**: 测量和优化中断响应时间
6. **调试技术**: 使用适当的工具和方法调试中断

### 实践建议

- 从简单的中断开始，逐步增加复杂性
- 使用逻辑分析仪测量中断时序
- 建立中断性能基准和监控机制
- 定期审查和优化中断处理代码
- 学习和应用中断设计模式

### 下一步

完成中断系统学习后，建议继续学习：
- [实时系统基础](./04-realtime-systems.md)
- [定时器与中断实践](../../06-timers-interrupts/README.md)
- [GPIO中断应用](../../04-gpio-control/README.md)