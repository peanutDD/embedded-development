#![no_std]
#![no_main]

// 导入必要的库
use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Input, Output, PushPull, Pin},
    timer::Timer,
};
use rtt_target::{rprintln, rtt_init_print};
use debouncr::{debounce_3, Debouncer, Repeat3};

// 移除void依赖以避免与panic_halt的冲突

// 类型别名，提高代码可读性
type LedPin = Pin<'C', 13, Output<PushPull>>;
type ButtonPin = Pin<'A', 0, Input>;
type ButtonDebouncer = Debouncer<u8, Repeat3>;

// Debouncer的Edge类型别名
type Edge = bool;

/// LED控制模式枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum LedMode {
    Off,           // 关闭
    On,            // 常亮
    SlowBlink,     // 慢闪 (1Hz)
    FastBlink,     // 快闪 (5Hz)
    Breathing,     // 呼吸灯效果
}

impl LedMode {
    /// 获取下一个模式
    fn next(self) -> Self {
        match self {
            LedMode::Off => LedMode::On,
            LedMode::On => LedMode::SlowBlink,
            LedMode::SlowBlink => LedMode::FastBlink,
            LedMode::FastBlink => LedMode::Breathing,
            LedMode::Breathing => LedMode::Off,
        }
    }
    
    /// 获取模式描述
    fn description(self) -> &'static str {
        match self {
            LedMode::Off => "关闭",
            LedMode::On => "常亮",
            LedMode::SlowBlink => "慢闪(1Hz)",
            LedMode::FastBlink => "快闪(5Hz)",
            LedMode::Breathing => "呼吸灯",
        }
    }
}

/// 应用程序状态结构
struct AppState {
    led_mode: LedMode,
    button_debouncer: ButtonDebouncer,
    blink_counter: u32,
    breathing_counter: u32,
    mode_change_count: u32,
    last_button_state: bool,
}

impl AppState {
    fn new() -> Self {
        Self {
            led_mode: LedMode::Off,
            button_debouncer: debounce_3(false),
            blink_counter: 0,
            breathing_counter: 0,
            mode_change_count: 0,
            last_button_state: false,
        }
    }
}

/// 主程序入口点
#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    rprintln!("🚀 按键控制LED示例启动");
    rprintln!("硬件平台: STM32F407VG Discovery");
    rprintln!("LED引脚: PC13 (板载LED)");
    rprintln!("按键引脚: PA0 (用户按键)");
    
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
    let gpioa = dp.GPIOA.split();
    
    let mut led = gpioc.pc13.into_push_pull_output();
    let button = gpioa.pa0.into_pull_up_input();
    
    // 配置定时器 - 使用较高频率以支持不同的闪烁模式
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(50.Hz()).unwrap(); // 50Hz = 20ms周期
    
    rprintln!("💡 GPIO配置完成");
    rprintln!("🔘 按键配置: 上拉输入，低电平触发");
    rprintln!("⏱️  定时器频率: 50Hz (20ms周期)");
    rprintln!("📋 LED控制模式:");
    rprintln!("   1. 关闭 → 2. 常亮 → 3. 慢闪 → 4. 快闪 → 5. 呼吸灯 → 循环");
    rprintln!("🎮 按下用户按键切换模式");
    
    // 初始化应用状态
    let mut app_state = AppState::new();
    
    // 显示初始状态
    rprintln!("🔄 当前模式: {} ({})", 
             app_state.led_mode.description(), 
             app_state.mode_change_count);
    
    // 主循环
    loop {
        // 等待定时器事件
        nb::block!(timer.wait()).unwrap();
        
        // 读取按键状态并进行防抖处理
        let button_pressed = button.is_low();
        
        // 使用debounce_3函数检测按键状态变化（忽略具体的Edge类型）
        if app_state.button_debouncer.update(button_pressed).is_some() {
            if button_pressed && !app_state.last_button_state {
                // 按键被按下，切换到下一个模式
                app_state.led_mode = app_state.led_mode.next();
                app_state.mode_change_count += 1;
                app_state.blink_counter = 0;
                app_state.breathing_counter = 0;
                
                rprintln!("🔄 模式切换: {} (第{}次切换)", 
                         app_state.led_mode.description(),
                         app_state.mode_change_count);
            }
            app_state.last_button_state = button_pressed;
        }
        
        // 根据当前模式控制LED
        match app_state.led_mode {
            LedMode::Off => {
                // LED关闭
                led.set_high();
            },
            
            LedMode::On => {
                // LED常亮
                led.set_low();
            },
            
            LedMode::SlowBlink => {
                // 慢闪：1Hz (50个周期 = 1秒)
                app_state.blink_counter += 1;
                if app_state.blink_counter >= 25 {
                    if app_state.blink_counter == 25 {
                        led.set_low(); // 点亮
                    } else if app_state.blink_counter >= 50 {
                        led.set_high(); // 熄灭
                        app_state.blink_counter = 0;
                    }
                }
            },
            
            LedMode::FastBlink => {
                // 快闪：5Hz (10个周期 = 0.2秒)
                app_state.blink_counter += 1;
                if app_state.blink_counter >= 5 {
                    if app_state.blink_counter == 5 {
                        led.set_low(); // 点亮
                    } else if app_state.blink_counter >= 10 {
                        led.set_high(); // 熄灭
                        app_state.blink_counter = 0;
                    }
                }
            },
            
            LedMode::Breathing => {
                // 呼吸灯效果：使用简单的开关模拟
                // 实际应用中可以使用PWM实现真正的呼吸效果
                app_state.breathing_counter += 1;
                
                // 呼吸周期：2秒 (100个周期)
                let cycle_pos = app_state.breathing_counter % 100;
                
                if cycle_pos < 50 {
                    // 渐亮阶段：快速闪烁模拟渐亮
                    if cycle_pos % 3 == 0 {
                        led.set_low();
                    } else {
                        led.set_high();
                    }
                } else {
                    // 渐暗阶段：慢速闪烁模拟渐暗
                    if cycle_pos % 8 == 0 {
                        led.set_low();
                    } else {
                        led.set_high();
                    }
                }
            },
        }
        
        // 每秒输出一次状态信息
        static mut COUNTER: u32 = 0;
        unsafe {
            COUNTER += 1;
            if COUNTER >= 50 { // 50 * 20ms = 1秒
                COUNTER = 0;
                rprintln!("📊 状态报告 - 模式: {}, 按键: {}, 运行时间: {}秒",
                         app_state.led_mode.description(),
                         if button_pressed { "按下" } else { "释放" },
                         app_state.mode_change_count * 50 / 50);
            }
        }
    }
}

// 已经通过panic_halt库提供了panic处理，无需自定义