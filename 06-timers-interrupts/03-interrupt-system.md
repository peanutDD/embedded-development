# 中断系统

## 概述

中断系统是嵌入式系统实现实时响应的核心机制。ARM Cortex-M4处理器集成了嵌套向量中断控制器(NVIC)，提供了高效的中断管理和处理能力。本章将深入介绍中断系统的工作原理、配置方法以及在Rust中的安全实现。

## ARM Cortex-M4中断架构

### 中断控制器概述

NVIC (Nested Vectored Interrupt Controller) 特性：
- 支持多达240个中断源
- 16个可编程优先级（4位优先级字段）
- 硬件优先级排序和嵌套
- 低延迟中断响应（12个时钟周期）
- 尾链优化减少上下文切换开销

### 异常和中断分类

```rust
// Cortex-M4异常向量表
#[repr(C)]
pub struct VectorTable {
    pub initial_stack_pointer: unsafe extern "C" fn(),
    pub reset: unsafe extern "C" fn(),
    pub nmi: unsafe extern "C" fn(),
    pub hard_fault: unsafe extern "C" fn(),
    pub mem_manage: unsafe extern "C" fn(),
    pub bus_fault: unsafe extern "C" fn(),
    pub usage_fault: unsafe extern "C" fn(),
    pub reserved1: [u32; 4],
    pub sv_call: unsafe extern "C" fn(),
    pub debug_monitor: unsafe extern "C" fn(),
    pub reserved2: u32,
    pub pend_sv: unsafe extern "C" fn(),
    pub sys_tick: unsafe extern "C" fn(),
    // 外部中断向量 (IRQ0-IRQ239)
    pub external_interrupts: [unsafe extern "C" fn(); 240],
}
```

### 异常优先级

| 异常类型 | 异常号 | 优先级 | 描述 |
|----------|--------|--------|------|
| Reset | -3 | -3 (最高) | 复位异常 |
| NMI | -2 | -2 | 不可屏蔽中断 |
| HardFault | -1 | -1 | 硬件故障 |
| MemManage | 4 | 可配置 | 内存管理故障 |
| BusFault | 5 | 可配置 | 总线故障 |
| UsageFault | 6 | 可配置 | 用法故障 |
| SVCall | 11 | 可配置 | 系统服务调用 |
| PendSV | 14 | 可配置 | 可挂起系统调用 |
| SysTick | 15 | 可配置 | 系统定时器 |
| IRQ0-IRQ239 | 16-255 | 可配置 | 外部中断 |

## NVIC寄存器详解

### 中断使能寄存器 (ISER)

```rust
// 使能中断
fn enable_interrupt(irq: u8) {
    let nvic = unsafe { &*cortex_m::peripheral::NVIC::ptr() };
    
    if irq < 32 {
        nvic.iser[0].write(1 << irq);
    } else if irq < 64 {
        nvic.iser[1].write(1 << (irq - 32));
    } else if irq < 96 {
        nvic.iser[2].write(1 << (irq - 64));
    }
    // ... 继续处理更多中断
}
```

### 中断优先级寄存器 (IPR)

```rust
// 设置中断优先级
fn set_interrupt_priority(irq: u8, priority: u8) {
    let nvic = unsafe { &*cortex_m::peripheral::NVIC::ptr() };
    
    // STM32F4使用4位优先级，左对齐到8位
    let priority_shifted = (priority & 0x0F) << 4;
    
    unsafe {
        let ipr_index = (irq / 4) as usize;
        let bit_offset = (irq % 4) * 8;
        
        let current = nvic.ipr[ipr_index].read();
        let mask = !(0xFF << bit_offset);
        let new_value = (current & mask) | ((priority_shifted as u32) << bit_offset);
        
        nvic.ipr[ipr_index].write(new_value);
    }
}
```

### 中断挂起寄存器 (ISPR/ICPR)

```rust
// 软件触发中断
fn trigger_interrupt(irq: u8) {
    let nvic = unsafe { &*cortex_m::peripheral::NVIC::ptr() };
    
    if irq < 32 {
        nvic.ispr[0].write(1 << irq);
    } else if irq < 64 {
        nvic.ispr[1].write(1 << (irq - 32));
    }
    // ...
}

// 清除挂起中断
fn clear_pending_interrupt(irq: u8) {
    let nvic = unsafe { &*cortex_m::peripheral::NVIC::ptr() };
    
    if irq < 32 {
        nvic.icpr[0].write(1 << irq);
    } else if irq < 64 {
        nvic.icpr[1].write(1 << (irq - 32));
    }
    // ...
}
```

## 优先级分组和抢占

### 优先级分组配置

```rust
// 配置优先级分组
fn configure_priority_grouping() {
    let mut scb = unsafe { cortex_m::peripheral::SCB::steal() };
    
    // 设置优先级分组：4位抢占优先级，0位子优先级
    scb.set_priority_grouping(0b011);
    
    // 其他分组选项：
    // 0b011: 4位抢占，0位子优先级 (推荐)
    // 0b100: 3位抢占，1位子优先级
    // 0b101: 2位抢占，2位子优先级
    // 0b110: 1位抢占，3位子优先级
    // 0b111: 0位抢占，4位子优先级
}
```

### 中断优先级最佳实践

```rust
// 定义中断优先级常量
const PRIORITY_CRITICAL: u8 = 0;    // 最高优先级（关键系统功能）
const PRIORITY_HIGH: u8 = 1;        // 高优先级（实时任务）
const PRIORITY_MEDIUM: u8 = 2;      // 中等优先级（一般任务）
const PRIORITY_LOW: u8 = 3;         // 低优先级（后台任务）

fn setup_interrupt_priorities() {
    // 系统定时器 - 最高优先级
    set_interrupt_priority(15, PRIORITY_CRITICAL);  // SysTick
    
    // 定时器中断 - 高优先级
    set_interrupt_priority(28, PRIORITY_HIGH);      // TIM2
    set_interrupt_priority(29, PRIORITY_HIGH);      // TIM3
    
    // 串口中断 - 中等优先级
    set_interrupt_priority(37, PRIORITY_MEDIUM);    // USART1
    
    // GPIO中断 - 低优先级
    set_interrupt_priority(23, PRIORITY_LOW);       // EXTI9_5
}
```

## 中断处理程序实现

### 基本中断处理程序

```rust
use cortex_m_rt::interrupt;
use stm32f4xx_hal::pac::TIM2;

// 定时器2中断处理程序
#[interrupt]
fn TIM2() {
    // 获取定时器外设
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 检查并清除中断标志
    if tim2.sr.read().uif().bit_is_set() {
        tim2.sr.modify(|_, w| w.uif().clear_bit());
        
        // 执行中断处理逻辑
        handle_timer_interrupt();
    }
}

fn handle_timer_interrupt() {
    // 中断处理逻辑应该尽可能简短
    // 复杂处理应该推迟到主循环或其他任务
    static mut COUNTER: u32 = 0;
    unsafe {
        COUNTER += 1;
        if COUNTER % 1000 == 0 {
            // 每秒执行一次的操作
            toggle_led();
        }
    }
}
```

### 中断安全的数据共享

```rust
use cortex_m::interrupt::{self, Mutex};
use core::cell::RefCell;

// 使用Mutex保护共享数据
static SHARED_DATA: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

// 在中断中安全地访问共享数据
#[interrupt]
fn TIM2() {
    interrupt::free(|cs| {
        let mut data = SHARED_DATA.borrow(cs).borrow_mut();
        *data += 1;
    });
}

// 在主程序中安全地访问共享数据
fn main_loop() {
    loop {
        let current_value = interrupt::free(|cs| {
            *SHARED_DATA.borrow(cs).borrow()
        });
        
        // 使用current_value...
    }
}
```

### 原子操作

```rust
use core::sync::atomic::{AtomicU32, Ordering};

// 使用原子类型避免临界区
static ATOMIC_COUNTER: AtomicU32 = AtomicU32::new(0);

#[interrupt]
fn TIM2() {
    // 原子递增
    ATOMIC_COUNTER.fetch_add(1, Ordering::Relaxed);
}

fn get_counter_value() -> u32 {
    ATOMIC_COUNTER.load(Ordering::Relaxed)
}
```

## 中断延迟和性能分析

### 中断响应时间

```rust
// 测量中断响应时间
static mut INTERRUPT_START_TIME: u32 = 0;
static mut INTERRUPT_RESPONSE_TIME: u32 = 0;

#[interrupt]
fn TIM2() {
    // 记录中断开始时间
    let start_time = get_cycle_count();
    
    // 中断处理逻辑
    handle_interrupt();
    
    // 计算响应时间
    let end_time = get_cycle_count();
    unsafe {
        INTERRUPT_RESPONSE_TIME = end_time - start_time;
    }
}

// 获取CPU周期计数
fn get_cycle_count() -> u32 {
    let dwt = unsafe { &*cortex_m::peripheral::DWT::ptr() };
    dwt.cyccnt.read()
}
```

### 中断统计

```rust
// 中断统计结构
#[derive(Default)]
struct InterruptStats {
    count: u32,
    max_duration: u32,
    min_duration: u32,
    total_duration: u32,
}

static mut TIM2_STATS: InterruptStats = InterruptStats {
    count: 0,
    max_duration: 0,
    min_duration: u32::MAX,
    total_duration: 0,
};

#[interrupt]
fn TIM2() {
    let start = get_cycle_count();
    
    // 中断处理逻辑
    handle_timer_interrupt();
    
    let duration = get_cycle_count() - start;
    
    unsafe {
        TIM2_STATS.count += 1;
        TIM2_STATS.total_duration += duration;
        
        if duration > TIM2_STATS.max_duration {
            TIM2_STATS.max_duration = duration;
        }
        if duration < TIM2_STATS.min_duration {
            TIM2_STATS.min_duration = duration;
        }
    }
}
```

## 高级中断技术

### 中断链和尾链优化

```rust
// 利用尾链优化减少上下文切换
#[interrupt]
fn TIM2() {
    // 处理TIM2中断
    handle_timer2();
    
    // 如果有其他挂起的中断，处理器会自动进行尾链
    // 无需完整的上下文切换
}

#[interrupt]
fn TIM3() {
    // 处理TIM3中断
    handle_timer3();
}
```

### 中断优先级继承

```rust
// 实现简单的优先级继承机制
static mut RESOURCE_OWNER: Option<u8> = None;  // 资源拥有者的优先级
static mut ORIGINAL_PRIORITY: Option<u8> = None;

fn acquire_resource(current_priority: u8) {
    interrupt::free(|_| {
        unsafe {
            if let Some(owner_priority) = RESOURCE_OWNER {
                if current_priority < owner_priority {
                    // 提升资源拥有者的优先级
                    ORIGINAL_PRIORITY = Some(owner_priority);
                    set_interrupt_priority(get_current_irq(), current_priority);
                }
            }
            RESOURCE_OWNER = Some(current_priority);
        }
    });
}

fn release_resource() {
    interrupt::free(|_| {
        unsafe {
            if let Some(original) = ORIGINAL_PRIORITY {
                // 恢复原始优先级
                set_interrupt_priority(get_current_irq(), original);
                ORIGINAL_PRIORITY = None;
            }
            RESOURCE_OWNER = None;
        }
    });
}
```

### 软件中断

```rust
// 使用PendSV实现软件中断
fn trigger_software_interrupt() {
    let mut scb = unsafe { cortex_m::peripheral::SCB::steal() };
    scb.set_pendsv();
}

#[interrupt]
fn PendSV() {
    // 软件中断处理
    // 通常用于任务切换或延迟处理
    handle_deferred_work();
}
```

## 中断调试技术

### 中断跟踪

```rust
// 中断跟踪缓冲区
const TRACE_BUFFER_SIZE: usize = 256;

#[derive(Copy, Clone)]
struct InterruptTrace {
    irq_number: u8,
    timestamp: u32,
    duration: u32,
}

static mut TRACE_BUFFER: [InterruptTrace; TRACE_BUFFER_SIZE] = 
    [InterruptTrace { irq_number: 0, timestamp: 0, duration: 0 }; TRACE_BUFFER_SIZE];
static mut TRACE_INDEX: usize = 0;

fn trace_interrupt_entry(irq: u8) -> u32 {
    let timestamp = get_cycle_count();
    unsafe {
        TRACE_BUFFER[TRACE_INDEX] = InterruptTrace {
            irq_number: irq,
            timestamp,
            duration: 0,
        };
    }
    timestamp
}

fn trace_interrupt_exit(irq: u8, start_time: u32) {
    let duration = get_cycle_count() - start_time;
    unsafe {
        TRACE_BUFFER[TRACE_INDEX].duration = duration;
        TRACE_INDEX = (TRACE_INDEX + 1) % TRACE_BUFFER_SIZE;
    }
}
```

### 中断监控

```rust
// 中断监控和报告
fn print_interrupt_statistics() {
    unsafe {
        rprintln!("TIM2 Interrupt Statistics:");
        rprintln!("  Count: {}", TIM2_STATS.count);
        rprintln!("  Max Duration: {} cycles", TIM2_STATS.max_duration);
        rprintln!("  Min Duration: {} cycles", TIM2_STATS.min_duration);
        rprintln!("  Avg Duration: {} cycles", 
                 TIM2_STATS.total_duration / TIM2_STATS.count.max(1));
    }
}
```

## 故障处理和恢复

### 硬件故障处理

```rust
#[interrupt]
fn HardFault() {
    // 硬件故障处理
    // 记录故障信息
    log_fault_information();
    
    // 尝试恢复或安全关机
    safe_shutdown();
}

#[interrupt]
fn MemoryManagement() {
    // 内存管理故障
    handle_memory_fault();
}

#[interrupt]
fn BusFault() {
    // 总线故障
    handle_bus_fault();
}

#[interrupt]
fn UsageFault() {
    // 用法故障
    handle_usage_fault();
}
```

### 看门狗中断

```rust
#[interrupt]
fn WWDG() {
    // 窗口看门狗中断
    // 最后的机会进行清理工作
    emergency_cleanup();
    
    // 清除看门狗标志（如果可能）
    let wwdg = unsafe { &*WWDG::ptr() };
    wwdg.sr.modify(|_, w| w.ewif().clear_bit());
}
```

## 实时性能优化

### 中断延迟优化

```rust
// 优化中断处理程序
#[interrupt]
fn TIM2() {
    // 1. 立即清除中断标志
    let tim2 = unsafe { &*TIM2::ptr() };
    tim2.sr.modify(|_, w| w.uif().clear_bit());
    
    // 2. 最小化中断处理时间
    // 只做必要的紧急处理
    critical_task();
    
    // 3. 将复杂处理推迟到主循环
    set_deferred_work_flag();
}

// 在主循环中处理延迟工作
fn main_loop() {
    loop {
        if check_deferred_work_flag() {
            clear_deferred_work_flag();
            handle_deferred_work();
        }
        
        // 其他主循环任务
    }
}
```

### 中断负载均衡

```rust
// 动态调整中断优先级
fn balance_interrupt_load() {
    // 根据中断统计调整优先级
    unsafe {
        if TIM2_STATS.count > HIGH_LOAD_THRESHOLD {
            // 降低优先级以减少系统负载
            set_interrupt_priority(28, PRIORITY_MEDIUM);
        } else if TIM2_STATS.count < LOW_LOAD_THRESHOLD {
            // 提高优先级以改善响应性
            set_interrupt_priority(28, PRIORITY_HIGH);
        }
    }
}
```

## 总结

中断系统是嵌入式系统实现实时性的关键技术。理解NVIC的工作原理、掌握中断优先级配置、实现安全的中断处理程序，以及进行性能优化，是开发高质量嵌入式应用的重要技能。

在Rust中，通过类型系统和所有权机制，我们可以实现更安全的中断处理，避免传统C语言中常见的竞态条件和内存安全问题。

## 参考资料

- ARM Cortex-M4 Technical Reference Manual
- STM32F4xx Programming Manual
- Cortex-M4 Devices Generic User Guide
- ARM Architecture Reference Manual