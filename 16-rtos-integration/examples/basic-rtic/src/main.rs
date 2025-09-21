#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm;
use stm32f4xx_hal::{
    gpio::{gpioa::PA5, gpioc::PC13, Input, Output, PullUp, PushPull},
    pac::{Peripherals, TIM2},
    prelude::*,
    timer::{CounterUs, Event},
};

use rtic::app;
use rtic_monotonics::{systick::*, Monotonic};
use heapless::{pool::{Pool, Node}, spsc::Queue};

// 消息类型定义
#[derive(Clone, Copy, Debug)]
pub enum Message {
    ButtonPressed,
    TimerTick,
    LedToggle,
    SystemStatus(u32),
}

// 系统状态
#[derive(Clone, Copy, Debug)]
pub struct SystemState {
    pub led_state: bool,
    pub button_count: u32,
    pub timer_count: u32,
    pub uptime_seconds: u32,
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            led_state: false,
            button_count: 0,
            timer_count: 0,
            uptime_seconds: 0,
        }
    }
}

// 内存池配置
const POOL_SIZE: usize = 16;
static mut MEMORY: [Node<Message>; POOL_SIZE] = [Node::new(); POOL_SIZE];

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        system_state: SystemState,
        message_pool: Pool<Message>,
    }

    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
        button: PC13<Input<PullUp>>,
        timer: CounterUs<TIM2>,
        message_queue: Queue<Message, 32>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // 初始化系统时钟
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

        // 初始化GPIO
        let gpioa = ctx.device.GPIOA.split();
        let gpioc = ctx.device.GPIOC.split();
        
        let led = gpioa.pa5.into_push_pull_output();
        let button = gpioc.pc13.into_pull_up_input();

        // 初始化定时器
        let mut timer = ctx.device.TIM2.counter_us(&clocks);
        timer.start(1.secs()).unwrap();
        timer.listen(Event::Update);

        // 初始化SysTick单调时钟
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 84_000_000, systick_token);

        // 初始化内存池
        let message_pool = Pool::new(unsafe { &mut MEMORY });
        let message_queue = Queue::new();

        // 启动周期性任务
        heartbeat_task::spawn().ok();
        system_monitor::spawn().ok();

        (
            Shared {
                system_state: SystemState::default(),
                message_pool,
            },
            Local {
                led,
                button,
                timer,
                message_queue,
            },
            init::Monotonics(Systick::new(ctx.core.SYST, 84_000_000)),
        )
    }

    // 空闲任务
    #[idle(shared = [system_state])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            // 在空闲时执行低优先级任务
            ctx.shared.system_state.lock(|state| {
                // 可以在这里执行一些后台任务
                asm::wfi(); // 等待中断
            });
        }
    }

    // 定时器中断处理
    #[task(binds = TIM2, local = [timer], shared = [system_state], priority = 3)]
    fn timer_interrupt(mut ctx: timer_interrupt::Context) {
        ctx.local.timer.clear_interrupt(Event::Update);
        
        ctx.shared.system_state.lock(|state| {
            state.timer_count = state.timer_count.wrapping_add(1);
        });

        // 发送定时器消息
        message_handler::spawn(Message::TimerTick).ok();
    }

    // 按钮中断处理
    #[task(binds = EXTI15_10, local = [button], shared = [system_state], priority = 2)]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        if ctx.local.button.is_low() {
            ctx.shared.system_state.lock(|state| {
                state.button_count = state.button_count.wrapping_add(1);
            });

            // 发送按钮消息
            message_handler::spawn(Message::ButtonPressed).ok();
        }
    }

    // 消息处理任务
    #[task(shared = [system_state], priority = 1)]
    fn message_handler(mut ctx: message_handler::Context, msg: Message) {
        match msg {
            Message::ButtonPressed => {
                // 处理按钮按下事件
                led_control::spawn(true).ok();
                
                ctx.shared.system_state.lock(|state| {
                    cortex_m_log::println!("Button pressed! Count: {}", state.button_count);
                });
            }
            Message::TimerTick => {
                // 处理定时器事件
                led_control::spawn(false).ok();
                
                ctx.shared.system_state.lock(|state| {
                    if state.timer_count % 10 == 0 {
                        cortex_m_log::println!("Timer tick: {}", state.timer_count);
                    }
                });
            }
            Message::LedToggle => {
                // 处理LED切换事件
                ctx.shared.system_state.lock(|state| {
                    state.led_state = !state.led_state;
                });
            }
            Message::SystemStatus(status) => {
                // 处理系统状态消息
                cortex_m_log::println!("System status: 0x{:08X}", status);
            }
        }
    }

    // LED控制任务
    #[task(local = [led], shared = [system_state], priority = 1)]
    fn led_control(mut ctx: led_control::Context, turn_on: bool) {
        if turn_on {
            ctx.local.led.set_high();
        } else {
            ctx.local.led.set_low();
        }

        ctx.shared.system_state.lock(|state| {
            state.led_state = turn_on;
        });

        message_handler::spawn(Message::LedToggle).ok();
    }

    // 心跳任务 - 每秒执行一次
    #[task(shared = [system_state], priority = 1)]
    fn heartbeat_task(mut ctx: heartbeat_task::Context) {
        ctx.shared.system_state.lock(|state| {
            state.uptime_seconds = state.uptime_seconds.wrapping_add(1);
            
            // 每10秒输出一次心跳信息
            if state.uptime_seconds % 10 == 0 {
                cortex_m_log::println!("Heartbeat - Uptime: {}s", state.uptime_seconds);
            }
        });

        // 切换LED状态作为心跳指示
        led_control::spawn(true).ok();
        
        // 延时后关闭LED
        led_off_delayed::spawn_after(100.millis()).ok();

        // 重新调度下一次心跳
        heartbeat_task::spawn_after(1.secs()).ok();
    }

    // 延时关闭LED任务
    #[task(local = [led], priority = 1)]
    fn led_off_delayed(ctx: led_off_delayed::Context) {
        ctx.local.led.set_low();
    }

    // 系统监控任务 - 每5秒执行一次
    #[task(shared = [system_state], priority = 1)]
    fn system_monitor(mut ctx: system_monitor::Context) {
        ctx.shared.system_state.lock(|state| {
            let status = (state.button_count << 16) | (state.timer_count & 0xFFFF);
            
            cortex_m_log::println!(
                "System Monitor - LED: {}, Buttons: {}, Timers: {}, Uptime: {}s",
                if state.led_state { "ON" } else { "OFF" },
                state.button_count,
                state.timer_count,
                state.uptime_seconds
            );

            // 发送系统状态消息
            message_handler::spawn(Message::SystemStatus(status)).ok();
        });

        // 重新调度下一次监控
        system_monitor::spawn_after(5.secs()).ok();
    }

    // 高优先级紧急任务
    #[task(shared = [system_state], priority = 4)]
    fn emergency_handler(mut ctx: emergency_handler::Context, error_code: u32) {
        ctx.shared.system_state.lock(|state| {
            cortex_m_log::println!("EMERGENCY: Error code 0x{:08X}", error_code);
            
            // 重置系统状态
            *state = SystemState::default();
        });

        // 快速闪烁LED表示紧急状态
        for _ in 0..5 {
            led_control::spawn(true).ok();
            // 在实际应用中，这里需要使用适当的延时机制
            led_control::spawn(false).ok();
        }
    }

    // 数据处理任务
    #[task(shared = [system_state], priority = 2)]
    fn data_processor(mut ctx: data_processor::Context, data: [u8; 16]) {
        let checksum: u32 = data.iter().map(|&x| x as u32).sum();
        
        ctx.shared.system_state.lock(|state| {
            cortex_m_log::println!("Processing data, checksum: 0x{:08X}", checksum);
        });

        // 如果校验和异常，触发紧急处理
        if checksum > 0x1000 {
            emergency_handler::spawn(checksum).ok();
        }
    }

    // 通信任务
    #[task(shared = [system_state], priority = 2)]
    fn communication_task(mut ctx: communication_task::Context, command: u8) {
        match command {
            0x01 => {
                // LED开启命令
                led_control::spawn(true).ok();
            }
            0x02 => {
                // LED关闭命令
                led_control::spawn(false).ok();
            }
            0x03 => {
                // 状态查询命令
                ctx.shared.system_state.lock(|state| {
                    let status = (state.uptime_seconds << 8) | (state.button_count & 0xFF);
                    message_handler::spawn(Message::SystemStatus(status)).ok();
                });
            }
            0xFF => {
                // 系统重置命令
                emergency_handler::spawn(0xDEADBEEF).ok();
            }
            _ => {
                cortex_m_log::println!("Unknown command: 0x{:02X}", command);
            }
        }
    }
}

// 测试函数
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_system_state() {
        let mut state = SystemState::default();
        assert_eq!(state.led_state, false);
        assert_eq!(state.button_count, 0);
        
        state.button_count += 1;
        assert_eq!(state.button_count, 1);
    }

    #[test]
    fn test_message_types() {
        let msg1 = Message::ButtonPressed;
        let msg2 = Message::TimerTick;
        let msg3 = Message::SystemStatus(0x12345678);
        
        match msg3 {
            Message::SystemStatus(status) => assert_eq!(status, 0x12345678),
            _ => panic!("Wrong message type"),
        }
    }
}