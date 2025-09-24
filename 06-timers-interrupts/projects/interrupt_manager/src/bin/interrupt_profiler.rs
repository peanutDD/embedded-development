#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Event},
    gpio::{gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7}, Output, PushPull},
    gpio::{gpioc::PC13, Input, PullUp},
    interrupt,
};
use cortex_m::peripheral::{NVIC, DWT, CorePeripherals};
use interrupt_manager::{
    InterruptManager, InterruptId, Priority, InterruptProfiler, ProfileSample,
    init_global_interrupt_manager, get_global_interrupt_manager,
    interrupt_handler
};
use heapless::spsc::{Queue, Producer, Consumer};
use core::sync::atomic::{AtomicU32, AtomicU8, AtomicBool, Ordering};
use micromath::F32Ext;

// LED类型定义
type StatusLed = PB0<Output<PushPull>>;
type ProfilingLed = PB1<Output<PushPull>>;
type HighLoadLed = PB2<Output<PushPull>>;
type MediumLoadLed = PB3<Output<PushPull>>;
type LowLoadLed = PB4<Output<PushPull>>;
type OverloadLed = PB5<Output<PushPull>>;
type ErrorLed = PB6<Output<PushPull>>;
type SamplingLed = PB7<Output<PushPull>>;

type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut PROFILE_QUEUE: Queue<ProfileSample, 128> = Queue::new();
static mut PROFILE_PRODUCER: Option<Producer<ProfileSample, 128>> = None;
static mut PROFILE_CONSUMER: Option<Consumer<ProfileSample, 128>> = None;

static SYSTEM_TIME_US: AtomicU32 = AtomicU32::new(0);
static PROFILING_MODE: AtomicU8 = AtomicU8::new(0);
static PROFILING_ENABLED: AtomicBool = AtomicBool::new(true);
static CPU_LOAD_PERCENT: AtomicU32 = AtomicU32::new(0);

// DWT计数器用于精确计时
static mut DWT_CYCLES_START: u32 = 0;
static DWT_FREQUENCY: u32 = 84_000_000; // 84MHz

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 启用DWT计数器用于精确计时
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    // 配置GPIO
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    let mut status_led = gpiob.pb0.into_push_pull_output();
    let mut profiling_led = gpiob.pb1.into_push_pull_output();
    let mut high_load_led = gpiob.pb2.into_push_pull_output();
    let mut medium_load_led = gpiob.pb3.into_push_pull_output();
    let mut low_load_led = gpiob.pb4.into_push_pull_output();
    let mut overload_led = gpiob.pb5.into_push_pull_output();
    let mut error_led = gpiob.pb6.into_push_pull_output();
    let mut sampling_led = gpiob.pb7.into_push_pull_output();
    
    let mode_button = gpioc.pc13.into_pull_up_input();

    // 初始化全局中断管理器
    init_global_interrupt_manager();

    // 初始化性能分析队列
    let (producer, consumer) = unsafe { PROFILE_QUEUE.split() };
    unsafe {
        PROFILE_PRODUCER = Some(producer);
        PROFILE_CONSUMER = Some(consumer);
    }

    // 配置定时器 - 不同负载的中断源
    let mut timer2 = Timer::tim2(dp.TIM2, &clocks);
    let mut timer3 = Timer::tim3(dp.TIM3, &clocks);
    let mut timer4 = Timer::tim4(dp.TIM4, &clocks);
    let mut timer5 = Timer::tim5(dp.TIM5, &clocks);

    // 设置不同频率和负载的定时器
    timer2.start(2000.hz()); // 2kHz - 轻负载
    timer3.start(1000.hz()); // 1kHz - 中等负载
    timer4.start(500.hz());  // 500Hz - 重负载
    timer5.start(100.hz());  // 100Hz - 超重负载

    // 启用定时器中断
    timer2.listen(Event::TimeOut);
    timer3.listen(Event::TimeOut);
    timer4.listen(Event::TimeOut);
    timer5.listen(Event::TimeOut);

    // 配置中断管理器
    if let Some(manager) = get_global_interrupt_manager() {
        // 注册中断
        manager.register_interrupt(InterruptId::Timer2, Priority::High).ok();
        manager.register_interrupt(InterruptId::Timer3, Priority::High).ok();
        manager.register_interrupt(InterruptId::Timer4, Priority::Medium).ok();
        manager.register_interrupt(InterruptId::Timer5, Priority::Low).ok();

        // 启用中断
        manager.enable_interrupt(InterruptId::Timer2).ok();
        manager.enable_interrupt(InterruptId::Timer3).ok();
        manager.enable_interrupt(InterruptId::Timer4).ok();
        manager.enable_interrupt(InterruptId::Timer5).ok();
    }

    // 启用NVIC中断
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM2);
        NVIC::unmask(stm32::Interrupt::TIM3);
        NVIC::unmask(stm32::Interrupt::TIM4);
        NVIC::unmask(stm32::Interrupt::TIM5);
    }

    // 配置延时定时器
    let mut delay_timer = Timer::tim6(dp.TIM6, &clocks).delay_ms();

    // 系统启动指示
    status_led.set_high();
    delay_timer.delay_ms(1000u32);

    // 性能分析器
    let mut profiler = InterruptProfiler::new(1000); // 1ms采样间隔
    profiler.start_sampling();
    
    let mut button_debouncer = ButtonDebouncer::new();
    let mut demo_counter = 0u32;
    let mut last_report_time = 0u32;

    loop {
        // 更新系统时间
        demo_counter += 1;
        let current_time = demo_counter * 10; // 10ms per loop
        SYSTEM_TIME_US.store(current_time * 1000, Ordering::Relaxed);
        
        if let Some(manager) = get_global_interrupt_manager() {
            manager.update_system_time(current_time * 1000);
        }

        // 按钮防抖处理
        let button_pressed = button_debouncer.update(mode_button.is_low());

        if button_pressed {
            // 切换性能分析模式
            let current_mode = PROFILING_MODE.load(Ordering::Relaxed);
            let new_mode = (current_mode + 1) % 4;
            PROFILING_MODE.store(new_mode, Ordering::Relaxed);

            // 重置分析器
            profiler.clear_samples();
            profiler.start_sampling();

            // 模式切换指示
            for _ in 0..3 {
                profiling_led.set_low();
                delay_timer.delay_ms(100u32);
                profiling_led.set_high();
                delay_timer.delay_ms(100u32);
            }
        }

        // 处理性能分析样本
        if let Some(consumer) = unsafe { PROFILE_CONSUMER.as_mut() } {
            while let Some(sample) = consumer.dequeue() {
                sampling_led.set_high();
                profiler.add_sample(sample).ok();
                delay_timer.delay_ms(1u32);
                sampling_led.set_low();
            }
        }

        // 每秒生成性能报告
        if current_time - last_report_time >= 1000 {
            let report = profiler.get_performance_report();
            
            // 计算CPU负载
            let cpu_load = calculate_cpu_load(&report, current_time);
            CPU_LOAD_PERCENT.store(cpu_load as u32, Ordering::Relaxed);
            
            last_report_time = current_time;
        }

        // 显示性能分析结果
        display_profiling_results(
            &profiler,
            &mut high_load_led,
            &mut medium_load_led,
            &mut low_load_led,
            &mut overload_led,
            &mut error_led,
            demo_counter,
        );

        // 状态LED心跳
        if demo_counter % 100 == 0 {
            status_led.toggle();
        }

        // 性能分析LED指示
        if PROFILING_ENABLED.load(Ordering::Relaxed) {
            profiling_led.set_state(((demo_counter / 20) % 2 == 0).into()).ok();
        } else {
            profiling_led.set_low().ok();
        }

        delay_timer.delay_ms(10u32);
    }
}

/// 计算CPU负载
fn calculate_cpu_load(report: &interrupt_manager::PerformanceReport, current_time: u32) -> f32 {
    if current_time == 0 || report.sample_count == 0 {
        return 0.0;
    }
    
    let total_interrupt_time = report.total_execution_time_us;
    let total_time = current_time * 1000; // 转换为微秒
    
    if total_time > 0 {
        (total_interrupt_time as f32 / total_time as f32) * 100.0
    } else {
        0.0
    }
}

/// 显示性能分析结果
fn display_profiling_results(
    profiler: &InterruptProfiler,
    high_load_led: &mut impl embedded_hal::digital::v2::OutputPin,
    medium_load_led: &mut impl embedded_hal::digital::v2::OutputPin,
    low_load_led: &mut impl embedded_hal::digital::v2::OutputPin,
    overload_led: &mut impl embedded_hal::digital::v2::OutputPin,
    error_led: &mut impl embedded_hal::digital::v2::OutputPin,
    counter: u32,
) {
    let report = profiler.get_performance_report();
    let cpu_load = CPU_LOAD_PERCENT.load(Ordering::Relaxed);
    
    // 根据CPU负载显示LED
    low_load_led.set_state((cpu_load > 10).into()).ok();
    medium_load_led.set_state((cpu_load > 30).into()).ok();
    high_load_led.set_state((cpu_load > 60).into()).ok();
    overload_led.set_state((cpu_load > 80).into()).ok();
    
    // 错误指示 - 如果平均执行时间过长
    if report.avg_execution_time_us > 1000.0 {
        error_led.set_state(((counter / 5) % 2 == 0).into()).ok();
    } else {
        error_led.set_low().ok();
    }
}

/// 按钮防抖器
struct ButtonDebouncer {
    last_state: bool,
    stable_state: bool,
    counter: u8,
}

impl ButtonDebouncer {
    fn new() -> Self {
        Self {
            last_state: false,
            stable_state: false,
            counter: 0,
        }
    }
    
    fn update(&mut self, current_state: bool) -> bool {
        if current_state != self.last_state {
            self.counter = 0;
            self.last_state = current_state;
        } else {
            if self.counter < 5 {
                self.counter += 1;
            }
        }
        
        if self.counter >= 5 && current_state != self.stable_state {
            self.stable_state = current_state;
            return current_state;
        }
        
        false
    }
}

/// 获取DWT周期计数
fn get_dwt_cycles() -> u32 {
    unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.read() }
}

/// 将DWT周期转换为微秒
fn dwt_cycles_to_us(cycles: u32) -> u32 {
    (cycles as u64 * 1_000_000 / DWT_FREQUENCY as u64) as u32
}

// 中断处理函数
#[interrupt]
fn TIM2() {
    let start_cycles = get_dwt_cycles();
    
    interrupt_handler!(InterruptId::Timer2, {
        // 轻负载处理 - 快速执行
        for _ in 0..50 {
            cortex_m::asm::nop();
        }
    });
    
    let end_cycles = get_dwt_cycles();
    let execution_time_us = dwt_cycles_to_us(end_cycles.wrapping_sub(start_cycles));
    
    // 创建性能样本
    let sample = ProfileSample {
        timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
        interrupt_id: InterruptId::Timer2,
        execution_time_us,
        nested_level: 0,
    };
    
    if let Some(producer) = unsafe { PROFILE_PRODUCER.as_mut() } {
        producer.enqueue(sample).ok();
    }
    
    // 清除中断标志
    unsafe {
        let tim2 = &(*stm32::TIM2::ptr());
        tim2.sr.modify(|_, w| w.uif().clear_bit());
    }
}

#[interrupt]
fn TIM3() {
    let start_cycles = get_dwt_cycles();
    
    interrupt_handler!(InterruptId::Timer3, {
        // 中等负载处理
        for _ in 0..200 {
            cortex_m::asm::nop();
        }
        
        // 模拟一些计算
        let mut sum = 0u32;
        for i in 0..10 {
            sum = sum.wrapping_add(i * i);
        }
    });
    
    let end_cycles = get_dwt_cycles();
    let execution_time_us = dwt_cycles_to_us(end_cycles.wrapping_sub(start_cycles));
    
    // 创建性能样本
    let sample = ProfileSample {
        timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
        interrupt_id: InterruptId::Timer3,
        execution_time_us,
        nested_level: 0,
    };
    
    if let Some(producer) = unsafe { PROFILE_PRODUCER.as_mut() } {
        producer.enqueue(sample).ok();
    }
    
    // 清除中断标志
    unsafe {
        let tim3 = &(*stm32::TIM3::ptr());
        tim3.sr.modify(|_, w| w.uif().clear_bit());
    }
}

#[interrupt]
fn TIM4() {
    let start_cycles = get_dwt_cycles();
    
    interrupt_handler!(InterruptId::Timer4, {
        // 重负载处理
        for _ in 0..500 {
            cortex_m::asm::nop();
        }
        
        // 模拟复杂计算
        let mut result = 1.0f32;
        for i in 1..20 {
            result = result * (i as f32).sin();
        }
    });
    
    let end_cycles = get_dwt_cycles();
    let execution_time_us = dwt_cycles_to_us(end_cycles.wrapping_sub(start_cycles));
    
    // 创建性能样本
    let sample = ProfileSample {
        timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
        interrupt_id: InterruptId::Timer4,
        execution_time_us,
        nested_level: 0,
    };
    
    if let Some(producer) = unsafe { PROFILE_PRODUCER.as_mut() } {
        producer.enqueue(sample).ok();
    }
    
    // 清除中断标志
    unsafe {
        let tim4 = &(*stm32::TIM4::ptr());
        tim4.sr.modify(|_, w| w.uif().clear_bit());
    }
}

#[interrupt]
fn TIM5() {
    let start_cycles = get_dwt_cycles();
    
    interrupt_handler!(InterruptId::Timer5, {
        // 超重负载处理
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
        
        // 模拟非常复杂的计算
        let mut matrix = [[0.0f32; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                matrix[i][j] = (i as f32 + j as f32).sqrt();
            }
        }
        
        // 矩阵运算
        let mut sum = 0.0f32;
        for i in 0..4 {
            for j in 0..4 {
                sum += matrix[i][j];
            }
        }
    });
    
    let end_cycles = get_dwt_cycles();
    let execution_time_us = dwt_cycles_to_us(end_cycles.wrapping_sub(start_cycles));
    
    // 创建性能样本
    let sample = ProfileSample {
        timestamp: SYSTEM_TIME_US.load(Ordering::Relaxed),
        interrupt_id: InterruptId::Timer5,
        execution_time_us,
        nested_level: 0,
    };
    
    if let Some(producer) = unsafe { PROFILE_PRODUCER.as_mut() } {
        producer.enqueue(sample).ok();
    }
    
    // 清除中断标志
    unsafe {
        let tim5 = &(*stm32::TIM5::ptr());
        tim5.sr.modify(|_, w| w.uif().clear_bit());
    }
}