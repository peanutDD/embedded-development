#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::{self, interrupt, peripheral::NVIC};
use cortex_m_rt::{entry, exception, ExceptionFrame};
use panic_halt as _;
use stm32f4xx_hal::{
    gpio::{gpioa::PA0, gpioc::PC13, Edge, ExtiPin, Input, Output, PullUp, PushPull},
    interrupt,
    pac::{self, EXTI, TIM2},
    prelude::*,
    timer::{CounterUs, Event},
};

// 全局共享资源
static BUTTON_PRESSED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static TIMER_COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static LED_STATE: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static INTERRUPT_STATS: Mutex<RefCell<InterruptStatistics>> = 
    Mutex::new(RefCell::new(InterruptStatistics::new()));

// 中断统计结构
#[derive(Clone, Copy)]
struct InterruptStatistics {
    exti0_count: u32,
    tim2_count: u32,
    systick_count: u32,
    max_interrupt_latency: u32,
    total_interrupt_time: u32,
}

impl InterruptStatistics {
    const fn new() -> Self {
        Self {
            exti0_count: 0,
            tim2_count: 0,
            systick_count: 0,
            max_interrupt_latency: 0,
            total_interrupt_time: 0,
        }
    }
}

// 全局外设句柄
static TIMER: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));
static LED: Mutex<RefCell<Option<PC13<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static BUTTON: Mutex<RefCell<Option<PA0<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 获取外设句柄
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

    // 初始化DWT用于性能测量
    let mut dwt = cp.DWT;
    let mut dcb = cp.DCB;
    dcb.enable_trace();
    dwt.enable_cycle_counter();

    // 配置 GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    // 配置 LED (PC13)
    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_high();

    // 配置按钮 (PA0) 带上拉电阻
    let mut button = gpioa.pa0.into_pull_up_input();

    // 配置按钮外部中断
    let mut syscfg = dp.SYSCFG.constrain();
    button.make_interrupt_source(&mut syscfg);
    button.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
    button.enable_interrupt(&mut dp.EXTI);

    // 配置定时器
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(1.secs()).unwrap();
    timer.listen(Event::Update);

    // 将外设移动到全局变量中
    interrupt::free(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
        LED.borrow(cs).replace(Some(led));
        BUTTON.borrow(cs).replace(Some(button));
    });

    // 配置中断优先级
    configure_interrupt_priorities(&cp.NVIC);

    // 启用中断
    unsafe {
        NVIC::unmask(pac::Interrupt::EXTI0);
        NVIC::unmask(pac::Interrupt::TIM2);
    }

    // 演示不同类型的中断
    demonstrate_interrupt_features();

    // 主循环
    let mut counter = 0u32;
    loop {
        // 检查按钮状态
        let button_pressed = interrupt::free(|cs| *BUTTON_PRESSED.borrow(cs).borrow());

        if button_pressed {
            // 处理按钮按下事件
            handle_button_press();

            // 清除标志
            interrupt::free(|cs| {
                BUTTON_PRESSED.borrow(cs).replace(false);
            });
        }

        // 主循环计数
        counter = counter.wrapping_add(1);

        // 模拟主循环工作
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }

        // 每 100000 次循环显示一次状态
        if counter % 100000 == 0 {
            show_system_status();
        }

        // 进入低功耗模式等待中断
        cortex_m::asm::wfi();
    }
}

fn configure_interrupt_priorities(nvic: &cortex_m::peripheral::NVIC) {
    unsafe {
        let mut nvic = nvic;
        
        // 设置优先级分组 (4位抢占优先级，0位子优先级)
        cortex_m::peripheral::SCB::set_priority_grouping(3);
        
        // 配置中断优先级 (数值越小优先级越高)
        nvic.set_priority(pac::Interrupt::EXTI0, 1);  // 按钮中断 - 高优先级
        nvic.set_priority(pac::Interrupt::TIM2, 2);   // 定时器中断 - 中等优先级
        
        // 演示优先级设置
        let exti0_priority = nvic.get_priority(pac::Interrupt::EXTI0);
        let tim2_priority = nvic.get_priority(pac::Interrupt::TIM2);
        let _ = (exti0_priority, tim2_priority);
    }
}

fn demonstrate_interrupt_features() {
    // 1. 中断优先级演示
    demonstrate_interrupt_priorities();
    
    // 2. 临界区演示
    demonstrate_critical_sections();
    
    // 3. 中断嵌套演示
    demonstrate_interrupt_nesting();
    
    // 4. 软件中断演示
    demonstrate_software_interrupts();
}

fn handle_button_press() {
    // 测量按钮响应延迟
    let start_time = cortex_m::peripheral::DWT::cycle_count();
    
    // 按钮按下处理逻辑
    interrupt::free(|cs| {
        if let Some(ref mut led) = LED.borrow(cs).borrow_mut().as_mut() {
            let current_state = LED_STATE.borrow(cs).borrow().clone();
            if current_state {
                led.set_high();
            } else {
                led.set_low();
            }
            LED_STATE.borrow(cs).replace(!current_state);
        }
    });
    
    let end_time = cortex_m::peripheral::DWT::cycle_count();
    let processing_time = end_time.wrapping_sub(start_time);
    
    // 更新统计信息
    interrupt::free(|cs| {
        let mut stats = INTERRUPT_STATS.borrow(cs).borrow_mut();
        stats.total_interrupt_time = stats.total_interrupt_time.wrapping_add(processing_time);
        if processing_time > stats.max_interrupt_latency {
            stats.max_interrupt_latency = processing_time;
        }
    });
}

fn show_system_status() {
    let (timer_count, led_state, stats) = interrupt::free(|cs| {
        let timer_count = *TIMER_COUNTER.borrow(cs).borrow();
        let led_state = *LED_STATE.borrow(cs).borrow();
        let stats = *INTERRUPT_STATS.borrow(cs).borrow();
        (timer_count, led_state, stats)
    });

    // 在实际应用中，可以通过串口或 RTT 输出状态信息
    let _ = (timer_count, led_state, stats);
}

// 按钮外部中断处理函数
#[interrupt]
fn EXTI0() {
    let entry_time = cortex_m::peripheral::DWT::cycle_count();
    
    // 清除中断标志
    unsafe {
        (*EXTI::ptr()).pr.write(|w| w.pr0().set_bit());
    }

    // 防抖处理
    static mut LAST_PRESS_TIME: u32 = 0;
    static mut DEBOUNCE_COUNTER: u32 = 0;

    unsafe {
        DEBOUNCE_COUNTER += 1;

        // 简单的软件防抖 (约50ms)
        if DEBOUNCE_COUNTER.wrapping_sub(LAST_PRESS_TIME) > 4200000 { // 84MHz * 0.05s
            LAST_PRESS_TIME = DEBOUNCE_COUNTER;

            // 设置按钮按下标志
            interrupt::free(|cs| {
                BUTTON_PRESSED.borrow(cs).replace(true);
                
                // 更新统计
                let mut stats = INTERRUPT_STATS.borrow(cs).borrow_mut();
                stats.exti0_count = stats.exti0_count.wrapping_add(1);
            });
        }
    }
    
    // 测量中断处理时间
    let exit_time = cortex_m::peripheral::DWT::cycle_count();
    let interrupt_duration = exit_time.wrapping_sub(entry_time);
    
    interrupt::free(|cs| {
        let mut stats = INTERRUPT_STATS.borrow(cs).borrow_mut();
        stats.total_interrupt_time = stats.total_interrupt_time.wrapping_add(interrupt_duration);
        if interrupt_duration > stats.max_interrupt_latency {
            stats.max_interrupt_latency = interrupt_duration;
        }
    });
}

// 定时器中断处理函数
#[interrupt]
fn TIM2() {
    let entry_time = cortex_m::peripheral::DWT::cycle_count();
    
    interrupt::free(|cs| {
        if let Some(ref mut timer) = TIMER.borrow(cs).borrow_mut().as_mut() {
            // 清除中断标志
            timer.clear_interrupt(Event::Update);

            // 更新计数器
            let mut counter = TIMER_COUNTER.borrow(cs).borrow_mut();
            *counter = counter.wrapping_add(1);

            // 更新统计
            let mut stats = INTERRUPT_STATS.borrow(cs).borrow_mut();
            stats.tim2_count = stats.tim2_count.wrapping_add(1);

            // 每 5 秒闪烁一次 LED (如果没有按钮控制)
            if *counter % 5 == 0 {
                if let Some(ref mut led) = LED.borrow(cs).borrow_mut().as_mut() {
                    let current_state = LED_STATE.borrow(cs).borrow().clone();
                    if current_state {
                        led.set_high();
                    } else {
                        led.set_low();
                    }
                    LED_STATE.borrow(cs).replace(!current_state);
                }
            }
        }
    });
    
    // 测量中断处理时间
    let exit_time = cortex_m::peripheral::DWT::cycle_count();
    let interrupt_duration = exit_time.wrapping_sub(entry_time);
    
    interrupt::free(|cs| {
        let mut stats = INTERRUPT_STATS.borrow(cs).borrow_mut();
        stats.total_interrupt_time = stats.total_interrupt_time.wrapping_add(interrupt_duration);
    });
}

// SysTick 中断处理函数
#[exception]
fn SysTick() {
    static mut SYSTICK_COUNTER: u32 = 0;

    unsafe {
        SYSTICK_COUNTER = SYSTICK_COUNTER.wrapping_add(1);

        // 更新统计
        interrupt::free(|cs| {
            let mut stats = INTERRUPT_STATS.borrow(cs).borrow_mut();
            stats.systick_count = stats.systick_count.wrapping_add(1);
        });

        // 每 1000 次 SysTick 中断执行一次操作
        if SYSTICK_COUNTER % 1000 == 0 {
            // 可以在这里执行周期性任务
            // 例如：系统监控、看门狗喂狗等
        }
    }
}

// 中断优先级演示
fn demonstrate_interrupt_priorities() {
    unsafe {
        let nvic = &*cortex_m::peripheral::NVIC::ptr();

        // 获取当前优先级设置
        let exti0_priority = nvic.get_priority(pac::Interrupt::EXTI0);
        let tim2_priority = nvic.get_priority(pac::Interrupt::TIM2);
        
        // 验证优先级设置
        assert!(exti0_priority < tim2_priority); // EXTI0 应该有更高优先级

        // 演示中断屏蔽
        NVIC::mask(pac::Interrupt::TIM2);   // 暂时禁用定时器中断
        NVIC::unmask(pac::Interrupt::TIM2); // 重新启用

        // 检查中断挂起状态
        let is_tim2_pending = NVIC::is_pending(pac::Interrupt::TIM2);
        let _ = is_tim2_pending;
    }
}

// 临界区演示
fn demonstrate_critical_sections() {
    // 方法 1: 使用 interrupt::free (推荐)
    let shared_data = interrupt::free(|cs| {
        // 在这个块中，所有中断都被禁用
        // 可以安全地访问共享资源
        *TIMER_COUNTER.borrow(cs).borrow()
    });

    // 方法 2: 手动控制中断 (需要小心使用)
    let primask = cortex_m::register::primask::read();
    unsafe {
        cortex_m::interrupt::disable();
        
        // 执行临界区代码
        let _data = shared_data;
        
        // 恢复之前的中断状态
        if !primask.is_active() {
            cortex_m::interrupt::enable();
        }
    }

    // 方法 3: 使用原子操作 (适用于简单数据类型)
    use core::sync::atomic::{AtomicU32, Ordering};
    static ATOMIC_COUNTER: AtomicU32 = AtomicU32::new(0);
    
    let _current_value = ATOMIC_COUNTER.load(Ordering::Relaxed);
    ATOMIC_COUNTER.store(42, Ordering::Relaxed);
    let _incremented = ATOMIC_COUNTER.fetch_add(1, Ordering::Relaxed);
}

// 中断嵌套演示
fn demonstrate_interrupt_nesting() {
    unsafe {
        // 设置中断优先级分组
        cortex_m::peripheral::SCB::set_priority_grouping(3); // 4 位抢占，0 位子优先级

        let nvic = &*cortex_m::peripheral::NVIC::ptr();
        
        // 配置不同的抢占优先级
        nvic.set_priority(pac::Interrupt::EXTI0, 0);  // 最高优先级
        nvic.set_priority(pac::Interrupt::TIM2, 1);   // 中等优先级
        
        // 在低优先级中断执行时，高优先级中断可以打断它
        // 这需要仔细的资源管理和重入性考虑
    }
}

// 软件中断演示
fn demonstrate_software_interrupts() {
    unsafe {
        // 手动触发中断 (用于测试)
        NVIC::pend(pac::Interrupt::TIM2);
        
        // 检查是否挂起
        let is_pending = NVIC::is_pending(pac::Interrupt::TIM2);
        
        if is_pending {
            // 清除挂起状态
            NVIC::unpend(pac::Interrupt::TIM2);
        }
        
        // 使用 PendSV 进行上下文切换 (RTOS 常用)
        cortex_m::peripheral::SCB::set_pendsv();
    }
}

// PendSV 异常处理 (用于上下文切换)
#[exception]
fn PendSV() {
    // PendSV 通常用于 RTOS 的任务切换
    // 这里只是演示如何处理
    
    // 清除 PendSV 挂起位会自动完成
    // 实际的上下文切换代码会在这里实现
}

// 错误处理和异常
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    // 硬件故障异常处理
    let scb = &*cortex_m::peripheral::SCB::ptr();
    
    // 获取故障信息
    let hfsr = scb.hfsr.read();
    let cfsr = scb.cfsr.read();
    let bfar = scb.bfar.read();
    let mmfar = scb.mmfar.read();

    // 分析故障原因
    let forced_fault = (hfsr & (1 << 30)) != 0;
    let vector_table_fault = (hfsr & (1 << 1)) != 0;
    
    // 内存管理故障
    let mem_fault_valid = (cfsr & (1 << 7)) != 0;
    let bus_fault_valid = (cfsr & (1 << 15)) != 0;
    
    let _ = (ef, hfsr, cfsr, bfar, mmfar, forced_fault, vector_table_fault, 
             mem_fault_valid, bus_fault_valid);

    // 在调试模式下停止执行
    #[cfg(debug_assertions)]
    loop {
        cortex_m::asm::bkpt();
    }
    
    // 在发布模式下重启系统
    #[cfg(not(debug_assertions))]
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    // 未处理的中断处理程序
    // irqn 是中断号 (负数表示系统异常，正数表示外部中断)
    
    // 记录未处理的中断
    static mut UNHANDLED_IRQ_COUNT: u32 = 0;
    UNHANDLED_IRQ_COUNT = UNHANDLED_IRQ_COUNT.wrapping_add(1);
    
    let _ = (irqn, UNHANDLED_IRQ_COUNT);

    // 在调试模式下停止
    #[cfg(debug_assertions)]
    cortex_m::asm::bkpt();
    
    // 在发布模式下继续执行
}
