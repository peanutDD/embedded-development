#![no_std]
#![no_main]

// 导入必要的库
use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac::{self, interrupt, Interrupt, TIM2},
    prelude::*,
    gpio::{Output, PushPull, Pin},
    timer::{Timer, Event},
};
use rtt_target::{rprintln, rtt_init_print};
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;
use heapless::String;
use core::fmt::Write;

// 类型别名
type LedPin = Pin<'D', 12, Output<PushPull>>;
type TimerType = Timer<TIM2>;

// 全局变量，用于在中断中访问
static G_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static G_TIMER: Mutex<RefCell<Option<TimerType>>> = Mutex::new(RefCell::new(None));

// 统计变量
static mut INTERRUPT_COUNT: u32 = 0;
static mut SYSTEM_TICKS: u64 = 0;
static mut LED_STATE: bool = false;

/// 定时器配置结构
#[derive(Debug, Clone, Copy)]
struct TimerConfig {
    frequency_hz: u32,
    description: &'static str,
}

/// 预定义的定时器配置
const TIMER_CONFIGS: [TimerConfig; 5] = [
    TimerConfig { frequency_hz: 1, description: "1Hz - 慢闪 (1秒周期)" },
    TimerConfig { frequency_hz: 2, description: "2Hz - 中等闪烁 (0.5秒周期)" },
    TimerConfig { frequency_hz: 5, description: "5Hz - 快闪 (0.2秒周期)" },
    TimerConfig { frequency_hz: 10, description: "10Hz - 高频闪烁 (0.1秒周期)" },
    TimerConfig { frequency_hz: 100, description: "100Hz - 超高频 (10ms周期)" },
];

/// 主程序入口点
#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    rprintln!("🚀 基础定时器中断示例启动");
    rprintln!("硬件平台: STM32F407VG Discovery");
    rprintln!("定时器: TIM2 (32位通用定时器)");
    rprintln!("LED引脚: PD12 (板载绿色LED)");
    
    // 获取外设访问权限
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置系统时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())  // 使用外部8MHz晶振
        .sysclk(168.MHz()) // 系统时钟168MHz
        .freeze();
    
    rprintln!("⚡ 系统时钟配置完成");
    rprintln!("   SYSCLK: {}MHz", clocks.sysclk().raw() / 1_000_000);
    rprintln!("   HCLK: {}MHz", clocks.hclk().raw() / 1_000_000);
    rprintln!("   PCLK1: {}MHz", clocks.pclk1().raw() / 1_000_000);
    rprintln!("   PCLK2: {}MHz", clocks.pclk2().raw() / 1_000_000);
    
    // 配置GPIO
    let gpiod = dp.GPIOD.split();
    let led = gpiod.pd12.into_push_pull_output();
    
    // 配置TIM2定时器
    let mut timer = Timer::new(dp.TIM2, &clocks);
    
    rprintln!("💡 GPIO和定时器初始化完成");
    
    // 将LED和定时器移动到全局变量中
    cortex_m::interrupt::free(|cs| {
        G_LED.borrow(cs).replace(Some(led));
        G_TIMER.borrow(cs).replace(Some(timer));
    });
    
    // 演示不同频率的定时器配置
    for (index, config) in TIMER_CONFIGS.iter().enumerate() {
        rprintln!("\n🔄 配置 #{}: {}", index + 1, config.description);
        
        // 配置定时器频率
        configure_timer(config.frequency_hz);
        
        // 运行一段时间
        let run_duration_ms = if config.frequency_hz >= 10 { 3000 } else { 5000 };
        run_timer_demo(run_duration_ms);
        
        // 停止定时器
        stop_timer();
        
        // 显示统计信息
        display_statistics(config);
        
        // 短暂延时
        delay_ms(1000);
    }
    
    rprintln!("\n✅ 所有定时器配置演示完成");
    rprintln!("🔄 开始连续运行模式 (2Hz)");
    
    // 最终配置：2Hz连续运行
    configure_timer(2);
    
    // 主循环 - 监控和统计
    let mut last_report_time = 0u64;
    loop {
        // 每5秒报告一次统计信息
        let current_ticks = unsafe { SYSTEM_TICKS };
        if current_ticks >= last_report_time + 10 { // 2Hz * 5秒 = 10 ticks
            last_report_time = current_ticks;
            
            let interrupt_count = unsafe { INTERRUPT_COUNT };
            let uptime_seconds = current_ticks / 2; // 2Hz转换为秒
            
            rprintln!("📊 运行统计 ({}秒):", uptime_seconds);
            rprintln!("   中断次数: {}", interrupt_count);
            rprintln!("   系统滴答: {}", current_ticks);
            rprintln!("   平均频率: {:.2}Hz", 
                     interrupt_count as f32 / uptime_seconds as f32);
            rprintln!("   LED状态: {}", 
                     if unsafe { LED_STATE } { "亮" } else { "灭" });
        }
        
        // 低功耗等待
        cortex_m::asm::wfi();
    }
}

/// 配置定时器频率
fn configure_timer(frequency_hz: u32) {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut timer) = G_TIMER.borrow(cs).borrow_mut().as_mut() {
            // 停止定时器
            timer.pause();
            
            // 配置频率
            timer.start(frequency_hz.Hz());
            
            // 启用定时器中断
            timer.listen(Event::Update);
            
            rprintln!("   ⚙️  定时器配置: {}Hz", frequency_hz);
            rprintln!("   📏 周期: {:.2}ms", 1000.0 / frequency_hz as f32);
        }
    });
    
    // 启用TIM2中断
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }
    
    // 重置统计计数器
    unsafe {
        INTERRUPT_COUNT = 0;
        SYSTEM_TICKS = 0;
        LED_STATE = false;
    }
}

/// 运行定时器演示
fn run_timer_demo(duration_ms: u32) {
    rprintln!("   ▶️  开始运行 {}ms...", duration_ms);
    
    let start_ticks = unsafe { SYSTEM_TICKS };
    
    // 等待指定时间
    loop {
        let current_ticks = unsafe { SYSTEM_TICKS };
        let elapsed_ticks = current_ticks - start_ticks;
        
        // 根据频率计算经过的毫秒数
        // 这里需要知道当前频率，简化处理
        if elapsed_ticks >= (duration_ms as u64 * 2 / 1000) { // 假设2Hz
            break;
        }
        
        cortex_m::asm::wfi();
    }
    
    rprintln!("   ⏹️  运行结束");
}

/// 停止定时器
fn stop_timer() {
    // 禁用TIM2中断
    cortex_m::peripheral::NVIC::mask(Interrupt::TIM2);
    
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut timer) = G_TIMER.borrow(cs).borrow_mut().as_mut() {
            timer.pause();
            timer.unlisten(Event::Update);
        }
        
        // 关闭LED
        if let Some(ref mut led) = G_LED.borrow(cs).borrow_mut().as_mut() {
            led.set_high();
        }
    });
    
    unsafe {
        LED_STATE = false;
    }
}

/// 显示统计信息
fn display_statistics(config: &TimerConfig) {
    let interrupt_count = unsafe { INTERRUPT_COUNT };
    let system_ticks = unsafe { SYSTEM_TICKS };
    
    rprintln!("   📈 统计结果:");
    rprintln!("      中断次数: {}", interrupt_count);
    rprintln!("      系统滴答: {}", system_ticks);
    
    if system_ticks > 0 {
        let actual_frequency = interrupt_count as f32 / (system_ticks as f32 / config.frequency_hz as f32);
        rprintln!("      实际频率: {:.2}Hz (目标: {}Hz)", actual_frequency, config.frequency_hz);
        
        let error_percent = ((actual_frequency - config.frequency_hz as f32) / config.frequency_hz as f32) * 100.0;
        rprintln!("      频率误差: {:.2}%", error_percent);
    }
}

/// 简单延时函数 (毫秒)
fn delay_ms(ms: u32) {
    // 使用系统时钟进行粗略延时
    let cycles = ms * 168_000; // 168MHz时钟
    for _ in 0..cycles {
        cortex_m::asm::nop();
    }
}

/// TIM2中断服务程序
#[interrupt]
fn TIM2() {
    // 更新统计计数器
    unsafe {
        INTERRUPT_COUNT += 1;
        SYSTEM_TICKS += 1;
        LED_STATE = !LED_STATE;
    }
    
    cortex_m::interrupt::free(|cs| {
        // 清除中断标志
        if let Some(ref mut timer) = G_TIMER.borrow(cs).borrow_mut().as_mut() {
            timer.clear_interrupt(Event::Update);
        }
        
        // 切换LED状态
        if let Some(ref mut led) = G_LED.borrow(cs).borrow_mut().as_mut() {
            if unsafe { LED_STATE } {
                led.set_low(); // 点亮LED
            } else {
                led.set_high(); // 熄灭LED
            }
        }
    });
    
    // 每100次中断输出一次调试信息
    unsafe {
        if INTERRUPT_COUNT % 100 == 0 {
            // 注意：在中断中使用RTT输出可能影响实时性
            // 在生产代码中应避免在ISR中进行复杂操作
        }
    }
}

/// 错误处理函数
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("💥 程序崩溃: {:?}", info);
    
    // 禁用所有中断
    cortex_m::interrupt::disable();
    
    // 关闭LED
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = G_LED.borrow(cs).borrow_mut().as_mut() {
            led.set_high();
        }
    });
    
    loop {
        cortex_m::asm::wfi();
    }
}