#![no_std]
#![no_main]

use cortex_m::{self, interrupt, peripheral::NVIC};
use cortex_m_rt::{entry, exception, ExceptionFrame};
use stm32f4xx_hal::{
    gpio::{gpioa::PA0, gpioc::PC13, Edge, ExtiPin, Input, Output, PullUp, PushPull},
    interrupt,
    pac::{self, EXTI, TIM2},
    prelude::*,
    timer::{CounterUs, Event},
};
use panic_halt as _;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

// 全局共享资源
static BUTTON_PRESSED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static TIMER_COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static LED_STATE: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

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
    
    // 启用中断
    unsafe {
        NVIC::unmask(pac::Interrupt::EXTI0);
        NVIC::unmask(pac::Interrupt::TIM2);
    }
    
    // 设置中断优先级
    unsafe {
        let mut nvic = cp.NVIC;
        nvic.set_priority(pac::Interrupt::EXTI0, 1); // 按钮中断高优先级
        nvic.set_priority(pac::Interrupt::TIM2, 2);  // 定时器中断低优先级
    }
    
    // 演示不同类型的中断
    demonstrate_interrupt_types();
    
    // 主循环
    let mut counter = 0u32;
    loop {
        // 检查按钮状态
        let button_pressed = interrupt::free(|cs| {
            *BUTTON_PRESSED.borrow(cs).borrow()
        });
        
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

fn demonstrate_interrupt_types() {
    // 1. 外部中断 (EXTI) - 已配置按钮
    // 2. 定时器中断 - 已配置 TIM2
    // 3. 系统中断 - SysTick (可选)
    
    // 配置 SysTick 中断 (可选)
    // let mut syst = cp.SYST;
    // syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    // syst.set_reload(84_000_000 / 10); // 100ms
    // syst.enable_counter();
    // syst.enable_interrupt();
}

fn handle_button_press() {
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
}

fn show_system_status() {
    let timer_count = interrupt::free(|cs| {
        *TIMER_COUNTER.borrow(cs).borrow()
    });
    
    let led_state = interrupt::free(|cs| {
        *LED_STATE.borrow(cs).borrow()
    });
    
    // 在实际应用中，可以通过串口或 RTT 输出状态信息
    let _ = (timer_count, led_state);
}

// 按钮外部中断处理函数
#[interrupt]
fn EXTI0() {
    // 清除中断标志
    unsafe {
        (*EXTI::ptr()).pr.write(|w| w.pr0().set_bit());
    }
    
    // 防抖处理
    static mut LAST_PRESS_TIME: u32 = 0;
    static mut DEBOUNCE_COUNTER: u32 = 0;
    
    unsafe {
        DEBOUNCE_COUNTER += 1;
        
        // 简单的软件防抖
        if DEBOUNCE_COUNTER - LAST_PRESS_TIME > 1000 {
            LAST_PRESS_TIME = DEBOUNCE_COUNTER;
            
            // 设置按钮按下标志
            interrupt::free(|cs| {
                BUTTON_PRESSED.borrow(cs).replace(true);
            });
        }
    }
}

// 定时器中断处理函数
#[interrupt]
fn TIM2() {
    interrupt::free(|cs| {
        if let Some(ref mut timer) = TIMER.borrow(cs).borrow_mut().as_mut() {
            // 清除中断标志
            timer.clear_interrupt(Event::Update);
            
            // 更新计数器
            let mut counter = TIMER_COUNTER.borrow(cs).borrow_mut();
            *counter = counter.wrapping_add(1);
            
            // 每 10 秒切换一次 LED (如果没有按钮按下)
            if *counter % 10 == 0 {
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
}

// SysTick 中断处理函数 (可选)
#[exception]
fn SysTick() {
    // SysTick 中断处理
    // 通常用于系统时钟或 RTOS 调度
    static mut SYSTICK_COUNTER: u32 = 0;
    
    unsafe {
        SYSTICK_COUNTER = SYSTICK_COUNTER.wrapping_add(1);
        
        // 每 100 次 SysTick 中断 (1秒) 执行一次操作
        if SYSTICK_COUNTER % 100 == 0 {
            // 可以在这里执行周期性任务
        }
    }
}

// 中断优先级演示
fn demonstrate_interrupt_priorities() {
    unsafe {
        let nvic = &mut *cortex_m::peripheral::NVIC::ptr();
        
        // 设置不同的中断优先级
        // 数值越小，优先级越高
        nvic.set_priority(pac::Interrupt::EXTI0, 0);  // 最高优先级
        nvic.set_priority(pac::Interrupt::TIM2, 1);   // 中等优先级
        nvic.set_priority(pac::Interrupt::USART1, 2); // 较低优先级
        
        // 启用/禁用特定中断
        NVIC::mask(pac::Interrupt::USART1);   // 禁用 USART1 中断
        NVIC::unmask(pac::Interrupt::EXTI0);  // 启用 EXTI0 中断
        
        // 检查中断是否挂起
        let is_pending = NVIC::is_pending(pac::Interrupt::TIM2);
        let _ = is_pending;
        
        // 手动触发中断 (软件中断)
        NVIC::pend(pac::Interrupt::TIM2);
        
        // 清除挂起的中断
        NVIC::unpend(pac::Interrupt::TIM2);
    }
}

// 临界区演示
fn demonstrate_critical_sections() {
    // 方法 1: 使用 interrupt::free
    let shared_data = interrupt::free(|cs| {
        // 在这个块中，所有中断都被禁用
        // 可以安全地访问共享资源
        *TIMER_COUNTER.borrow(cs).borrow()
    });
    
    // 方法 2: 手动控制中断
    unsafe {
        // 禁用中断
        cortex_m::interrupt::disable();
        
        // 访问共享资源
        let data = core::ptr::read_volatile(&shared_data);
        let _ = data;
        
        // 重新启用中断
        cortex_m::interrupt::enable();
    }
    
    // 方法 3: 使用 PRIMASK 寄存器
    let primask = cortex_m::register::primask::read();
    unsafe {
        cortex_m::interrupt::disable();
        
        // 执行临界区代码
        let _ = shared_data;
        
        // 恢复之前的中断状态
        if primask.is_active() {
            cortex_m::interrupt::enable();
        }
    }
}

// 中断嵌套演示
fn demonstrate_interrupt_nesting() {
    // 在高优先级中断中可以被更高优先级的中断打断
    // 这需要仔细的优先级设计和资源管理
    
    unsafe {
        let nvic = &mut *cortex_m::peripheral::NVIC::ptr();
        
        // 设置中断优先级分组
        // 这决定了抢占优先级和子优先级的位数分配
        cortex_m::peripheral::SCB::set_priority_grouping(3); // 4 位抢占，0 位子优先级
        
        // 配置不同的抢占优先级
        nvic.set_priority(pac::Interrupt::EXTI0, 0 << 4);  // 抢占优先级 0 (最高)
        nvic.set_priority(pac::Interrupt::TIM2, 1 << 4);   // 抢占优先级 1
        nvic.set_priority(pac::Interrupt::USART1, 2 << 4); // 抢占优先级 2
    }
}

// 错误处理和异常
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    // 硬件故障异常处理
    // 通常由于非法内存访问、除零等引起
    
    // 获取故障信息
    let hfsr = (*cortex_m::peripheral::SCB::ptr()).hfsr.read();
    let cfsr = (*cortex_m::peripheral::SCB::ptr()).cfsr.read();
    
    // 分析故障原因
    if hfsr & (1 << 30) != 0 {
        // 强制硬件故障
    }
    if hfsr & (1 << 1) != 0 {
        // 向量表硬件故障
    }
    
    let _ = (ef, hfsr, cfsr);
    
    // 在调试模式下停止执行
    loop {
        cortex_m::asm::bkpt();
    }
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    // 未处理的中断
    // irqn 是中断号
    let _ = irqn;
    
    // 记录未处理的中断并继续执行
    loop {
        cortex_m::asm::bkpt();
    }
}