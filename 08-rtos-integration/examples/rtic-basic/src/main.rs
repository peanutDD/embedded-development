#![no_main]
#![no_std]

// RTIC基础示例
// 功能：多任务LED控制、按键处理、定时器任务
// 硬件：STM32F407VG Discovery
// 演示：任务调度、资源共享、中断处理

use panic_halt as _;

use rtic::app;
use rtic_monotonics::systick::*;
use rtt_target::{rprintln, rtt_init_print};

use stm32f4xx_hal::{
    gpio::{
        gpioa::{PA0},
        gpiod::{PD12, PD13, PD14, PD15},
        Input, Output, PushPull, PullUp,
    },
    pac,
    prelude::*,
};

use fugit::ExtU32;
use heapless::spsc::{Consumer, Producer, Queue};

// 系统配置常量
const SYSTICK_RATE_HZ: u32 = 1000; // 1kHz系统时钟
const LED_BLINK_PERIOD_MS: u32 = 500; // LED闪烁周期
const BUTTON_DEBOUNCE_MS: u32 = 50; // 按键消抖时间
const HEARTBEAT_PERIOD_MS: u32 = 1000; // 心跳周期
const STATS_REPORT_PERIOD_MS: u32 = 5000; // 统计报告周期

// 任务间通信消息类型
#[derive(Clone, Copy, Debug)]
enum TaskMessage {
    ButtonPressed,
    ButtonReleased,
    LedToggle(u8), // LED编号
    SystemReset,
    StatsRequest,
}

// LED模式枚举
#[derive(Clone, Copy, Debug, PartialEq)]
enum LedMode {
    Off,
    On,
    Blink,
    FastBlink,
    Breathing,
}

// 系统统计数据
#[derive(Default)]
struct SystemStats {
    button_presses: u32,
    led_toggles: u32,
    task_switches: u32,
    uptime_seconds: u32,
    max_queue_usage: usize,
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, USART2, USART3])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        // 共享资源
        system_stats: SystemStats,
        led_modes: [LedMode; 4],
        button_state: bool,
        message_producer: Producer<'static, TaskMessage, 16>,
    }

    #[local]
    struct Local {
        // 本地资源
        led_green: PD12<Output<PushPull>>,
        led_orange: PD13<Output<PushPull>>,
        led_red: PD14<Output<PushPull>>,
        led_blue: PD15<Output<PushPull>>,
        button: PA0<Input<PullUp>>,
        message_consumer: Consumer<'static, TaskMessage, 16>,
        led_states: [bool; 4],
        blink_counter: u32,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // 初始化RTT调试输出
        rtt_init_print!();
        rprintln!("\n=== RTIC基础示例启动 ===");
        rprintln!("功能: 多任务LED控制和按键处理");
        rprintln!("硬件: STM32F407VG Discovery\n");

        let dp = ctx.device;

        // 配置时钟
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(168.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .freeze();

        rprintln!("时钟配置:");
        rprintln!("  SYSCLK: {} MHz", clocks.sysclk().to_MHz());
        rprintln!("  PCLK1:  {} MHz", clocks.pclk1().to_MHz());
        rprintln!("  PCLK2:  {} MHz", clocks.pclk2().to_MHz());

        // 初始化SysTick单调时钟
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, clocks.sysclk().to_Hz(), systick_token);

        // 配置GPIO
        let gpioa = dp.GPIOA.split();
        let gpiod = dp.GPIOD.split();

        // 配置LED引脚
        let led_green = gpiod.pd12.into_push_pull_output();
        let led_orange = gpiod.pd13.into_push_pull_output();
        let led_red = gpiod.pd14.into_push_pull_output();
        let led_blue = gpiod.pd15.into_push_pull_output();

        // 配置按键引脚
        let button = gpioa.pa0.into_pull_up_input();

        rprintln!("\nGPIO配置:");
        rprintln!("  PD12: 绿色LED");
        rprintln!("  PD13: 橙色LED");
        rprintln!("  PD14: 红色LED");
        rprintln!("  PD15: 蓝色LED");
        rprintln!("  PA0:  用户按键 (上拉输入)");

        // 创建任务间通信队列
        static mut QUEUE: Queue<TaskMessage, 16> = Queue::new();
        let (message_producer, message_consumer) = unsafe { QUEUE.split() };

        rprintln!("\n任务配置:");
        rprintln!("  消息队列大小: 16");
        rprintln!("  SysTick频率: {} Hz", SYSTICK_RATE_HZ);

        // 启动周期性任务
        led_blink_task::spawn().ok();
        heartbeat_task::spawn().ok();
        stats_report_task::spawn().ok();
        button_monitor_task::spawn().ok();

        rprintln!("\n任务启动完成，开始运行...\n");

        (
            Shared {
                system_stats: SystemStats::default(),
                led_modes: [LedMode::Blink, LedMode::FastBlink, LedMode::On, LedMode::Breathing],
                button_state: false,
                message_producer,
            },
            Local {
                led_green,
                led_orange,
                led_red,
                led_blue,
                button,
                message_consumer,
                led_states: [false; 4],
                blink_counter: 0,
            },
            init::Monotonics(Systick),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // 进入低功耗模式等待中断
            cortex_m::asm::wfi();
        }
    }

    // LED闪烁任务 - 高优先级
    #[task(priority = 3, shared = [led_modes, system_stats], local = [led_green, led_orange, led_red, led_blue, led_states, blink_counter])]
    async fn led_blink_task(mut ctx: led_blink_task::Context) {
        let mut interval = Systick::delay(LED_BLINK_PERIOD_MS.millis());
        
        loop {
            interval.next().await;
            
            let led_modes = ctx.shared.led_modes.lock(|modes| *modes);
            
            // 更新闪烁计数器
            *ctx.local.blink_counter = ctx.local.blink_counter.wrapping_add(1);
            let counter = *ctx.local.blink_counter;
            
            // 控制每个LED
            for (i, &mode) in led_modes.iter().enumerate() {
                let new_state = match mode {
                    LedMode::Off => false,
                    LedMode::On => true,
                    LedMode::Blink => (counter / 2) % 2 == 0,
                    LedMode::FastBlink => counter % 2 == 0,
                    LedMode::Breathing => {
                        // 简单的呼吸灯效果
                        let phase = (counter / 4) % 8;
                        phase < 4
                    },
                };
                
                if ctx.local.led_states[i] != new_state {
                    ctx.local.led_states[i] = new_state;
                    
                    // 更新LED状态
                    match i {
                        0 => if new_state { ctx.local.led_green.set_high() } else { ctx.local.led_green.set_low() },
                        1 => if new_state { ctx.local.led_orange.set_high() } else { ctx.local.led_orange.set_low() },
                        2 => if new_state { ctx.local.led_red.set_high() } else { ctx.local.led_red.set_low() },
                        3 => if new_state { ctx.local.led_blue.set_high() } else { ctx.local.led_blue.set_low() },
                        _ => {},
                    }
                    
                    // 更新统计
                    ctx.shared.system_stats.lock(|stats| {
                        stats.led_toggles += 1;
                    });
                }
            }
        }
    }

    // 按键监控任务 - 中等优先级
    #[task(priority = 2, shared = [button_state, message_producer, system_stats], local = [button])]
    async fn button_monitor_task(mut ctx: button_monitor_task::Context) {
        let mut interval = Systick::delay(BUTTON_DEBOUNCE_MS.millis());
        let mut last_state = false;
        
        loop {
            interval.next().await;
            
            let current_state = ctx.local.button.is_low();
            
            if current_state != last_state {
                // 按键状态发生变化
                ctx.shared.button_state.lock(|state| *state = current_state);
                
                let message = if current_state {
                    TaskMessage::ButtonPressed
                } else {
                    TaskMessage::ButtonReleased
                };
                
                // 发送消息到消息处理任务
                ctx.shared.message_producer.lock(|producer| {
                    if producer.enqueue(message).is_err() {
                        rprintln!("警告: 消息队列已满");
                    }
                });
                
                if current_state {
                    ctx.shared.system_stats.lock(|stats| {
                        stats.button_presses += 1;
                    });
                }
                
                last_state = current_state;
            }
        }
    }

    // 消息处理任务 - 中等优先级
    #[task(priority = 2, shared = [led_modes, system_stats], local = [message_consumer])]
    async fn message_handler_task(mut ctx: message_handler_task::Context) {
        loop {
            if let Some(message) = ctx.local.message_consumer.dequeue() {
                match message {
                    TaskMessage::ButtonPressed => {
                        rprintln!("[MSG] 按键按下");
                        
                        // 切换LED模式
                        ctx.shared.led_modes.lock(|modes| {
                            for mode in modes.iter_mut() {
                                *mode = match *mode {
                                    LedMode::Off => LedMode::On,
                                    LedMode::On => LedMode::Blink,
                                    LedMode::Blink => LedMode::FastBlink,
                                    LedMode::FastBlink => LedMode::Breathing,
                                    LedMode::Breathing => LedMode::Off,
                                };
                            }
                        });
                    },
                    TaskMessage::ButtonReleased => {
                        rprintln!("[MSG] 按键释放");
                    },
                    TaskMessage::LedToggle(led_num) => {
                        rprintln!("[MSG] LED {} 切换", led_num);
                    },
                    TaskMessage::SystemReset => {
                        rprintln!("[MSG] 系统重置请求");
                        ctx.shared.system_stats.lock(|stats| {
                            *stats = SystemStats::default();
                        });
                    },
                    TaskMessage::StatsRequest => {
                        rprintln!("[MSG] 统计信息请求");
                        stats_display_task::spawn().ok();
                    },
                }
                
                ctx.shared.system_stats.lock(|stats| {
                    stats.task_switches += 1;
                });
            } else {
                // 没有消息时等待一小段时间
                Systick::delay(10.millis()).await;
            }
        }
    }

    // 心跳任务 - 低优先级
    #[task(priority = 1, shared = [system_stats])]
    async fn heartbeat_task(mut ctx: heartbeat_task::Context) {
        let mut interval = Systick::delay(HEARTBEAT_PERIOD_MS.millis());
        
        loop {
            interval.next().await;
            
            ctx.shared.system_stats.lock(|stats| {
                stats.uptime_seconds += 1;
            });
            
            rprintln!("[心跳] 系统运行正常 - {} 秒", 
                ctx.shared.system_stats.lock(|stats| stats.uptime_seconds));
        }
    }

    // 统计报告任务 - 低优先级
    #[task(priority = 1, shared = [system_stats, led_modes, button_state])]
    async fn stats_report_task(mut ctx: stats_report_task::Context) {
        let mut interval = Systick::delay(STATS_REPORT_PERIOD_MS.millis());
        
        loop {
            interval.next().await;
            
            let (stats, led_modes, button_state) = (
                ctx.shared.system_stats.lock(|s| *s),
                ctx.shared.led_modes.lock(|m| *m),
                ctx.shared.button_state.lock(|b| *b),
            );
            
            rprintln!("\n=== 系统统计报告 ===");
            rprintln!("  运行时间: {} 秒", stats.uptime_seconds);
            rprintln!("  按键按下次数: {}", stats.button_presses);
            rprintln!("  LED切换次数: {}", stats.led_toggles);
            rprintln!("  任务切换次数: {}", stats.task_switches);
            rprintln!("  当前按键状态: {}", if button_state { "按下" } else { "释放" });
            
            rprintln!("  LED模式:");
            for (i, &mode) in led_modes.iter().enumerate() {
                let mode_str = match mode {
                    LedMode::Off => "关闭",
                    LedMode::On => "常亮",
                    LedMode::Blink => "慢闪",
                    LedMode::FastBlink => "快闪",
                    LedMode::Breathing => "呼吸",
                };
                rprintln!("    LED{}: {}", i, mode_str);
            }
            rprintln!("");
        }
    }

    // 统计显示任务 - 按需触发
    #[task(priority = 1, shared = [system_stats])]
    async fn stats_display_task(mut ctx: stats_display_task::Context) {
        let stats = ctx.shared.system_stats.lock(|s| *s);
        
        rprintln!("\n--- 即时统计信息 ---");
        rprintln!("运行时间: {} 秒", stats.uptime_seconds);
        rprintln!("按键次数: {}", stats.button_presses);
        rprintln!("LED切换: {}", stats.led_toggles);
        rprintln!("任务切换: {}", stats.task_switches);
        rprintln!("");
    }

    // 外部中断处理 - 最高优先级
    #[task(binds = EXTI0, priority = 4, shared = [message_producer])]
    fn exti0_interrupt(mut ctx: exti0_interrupt::Context) {
        // 处理外部中断（如果配置了的话）
        rprintln!("[中断] EXTI0 触发");
        
        // 可以发送紧急消息
        ctx.shared.message_producer.lock(|producer| {
            let _ = producer.enqueue(TaskMessage::SystemReset);
        });
    }
}

// 启动消息处理任务
#[rtic::task(priority = 2, shared = [led_modes, system_stats], local = [message_consumer])]
async fn start_message_handler(ctx: start_message_handler::Context) {
    app::message_handler_task::spawn().ok();
}