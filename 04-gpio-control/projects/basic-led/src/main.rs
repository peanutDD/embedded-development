#![no_std]
#![no_main]

// 导入必要的库
use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, PushPull, Pin},
    timer::Timer,
};
use rtt_target::{rprintln, rtt_init_print};

// 类型别名，提高代码可读性
type LedPin = Pin<'C', 13, Output<PushPull>>;

/// 主程序入口点
#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    rprintln!("🚀 基础LED控制示例启动");
    rprintln!("硬件平台: STM32F407VG Discovery");
    rprintln!("LED引脚: PC13 (板载LED)");
    
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
    
    rprintln!("⚡ 系统时钟配置完成: {}MHz", clocks.sysclk().raw() / 1_000_000);
    
    // 配置GPIO
    let gpioc = dp.GPIOC.split();
    let mut led: LedPin = gpioc.pc13.into_push_pull_output();
    
    // 配置系统定时器用于精确延时
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap(); // 1Hz = 1秒周期
    
    rprintln!("💡 GPIO配置完成，开始LED闪烁循环");
    rprintln!("📊 闪烁频率: 1Hz (每秒1次)");
    
    let mut counter = 0u32;
    let mut led_state = false;
    
    // 主循环
    loop {
        // 等待定时器事件
        nb::block!(timer.wait()).unwrap();
        
        // 切换LED状态
        led_state = !led_state;
        
        if led_state {
            // 点亮LED (STM32F4 Discovery板载LED是低电平点亮)
            led.set_low();
            rprintln!("🔆 LED开启 - 计数: {} - 时间戳: {}ms", 
                     counter, 
                     counter * 1000);
        } else {
            // 熄灭LED
            led.set_high();
            rprintln!("🔅 LED关闭 - 计数: {} - 时间戳: {}ms", 
                     counter, 
                     counter * 1000);
        }
        
        counter = counter.wrapping_add(1);
        
        // 每10次闪烁输出一次统计信息
        if counter % 10 == 0 {
            rprintln!("📈 运行统计: 已完成{}次LED切换，运行时间{}秒", 
                     counter, counter / 2);
        }
    }
}

// 使用 panic-halt 库处理 panic，无需自定义 panic 处理函数