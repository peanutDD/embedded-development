#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm;
use rtic::app;
use stm32f4xx_hal::{
    gpio::{gpioa::PA5, gpioc::PC13, Input, Output, PullUp, PushPull},
    pac,
    prelude::*,
    timer::{CounterUs, Event},
};
use dwt_systick_monotonic::{DwtSystick, ExtU32};
use heapless::{
    pool::{Pool, Node},
    spsc::{Queue, Producer, Consumer},
    Vec,
};
use rtt_target::{rprintln, rtt_init_print};

// 任务优先级定义
const HIGH_PRIORITY: u8 = 3;
const MEDIUM_PRIORITY: u8 = 2;
const LOW_PRIORITY: u8 = 1;

// 内存池配置
const POOL_SIZE: usize = 16;
const QUEUE_SIZE: usize = 8;

// 任务统计信息
#[derive(Debug, Clone, Copy)]
pub struct TaskStats {
    pub execution_count: u32,
    pub total_execution_time: u32,
    pub max_execution_time: u32,
    pub missed_deadlines: u32,
}

impl Default for TaskStats {
    fn default() -> Self {
        Self {
            execution_count: 0,
            total_execution_time: 0,
            max_execution_time: 0,
            missed_deadlines: 0,
        }
    }
}

// 任务消息类型
#[derive(Debug, Clone, Copy)]
pub enum TaskMessage {
    SensorData(u16),
    ControlCommand(u8),
    StatusUpdate(bool),
    EmergencyStop,
}

// 系统状态
#[derive(Debug, Clone, Copy)]
pub struct SystemState {
    pub sensor_value: u16,
    pub control_output: u8,
    pub system_active: bool,
    pub error_count: u32,
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            sensor_value: 0,
            control_output: 0,
            system_active: true,
            error_count: 0,
        }
    }
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<84_000_000>; // 84 MHz

    #[shared]
    struct Shared {
        system_state: SystemState,
        task_stats: [TaskStats; 4], // 4个任务的统计信息
        message_queue: Queue<TaskMessage, QUEUE_SIZE>,
    }

    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
        button: PC13<Input<PullUp>>,
        timer: CounterUs<pac::TIM2>,
        producer: Producer<'static, TaskMessage, QUEUE_SIZE>,
        consumer: Consumer<'static, TaskMessage, QUEUE_SIZE>,
        memory_pool: Pool<Node<[u8; 64]>>,
    }

    #[init(local = [
        queue: Queue<TaskMessage, QUEUE_SIZE> = Queue::new(),
        memory: [Node<[u8; 64]>; POOL_SIZE] = [Node::new(); POOL_SIZE]
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("Multi-Task Scheduler Starting...");

        let dp = ctx.device;
        let cp = ctx.core;

        // 时钟配置
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(84.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .freeze();

        // GPIO配置
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();
        
        let led = gpioa.pa5.into_push_pull_output();
        let button = gpioc.pc13.into_pull_up_input();

        // 定时器配置
        let mut timer = dp.TIM2.counter_us(&clocks);
        timer.start(1.secs()).unwrap();
        timer.listen(Event::Update);

        // 单调时钟初始化
        let mono = DwtSystick::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.hclk().raw());

        // 消息队列初始化
        let (producer, consumer) = ctx.local.queue.split();

        // 内存池初始化
        let memory_pool: Pool<Node<[u8; 64]>> = Pool::new();
        for node in ctx.local.memory {
            memory_pool.manage(node);
        }

        // 启动周期性任务
        sensor_task::spawn().ok();
        control_task::spawn().ok();
        monitoring_task::spawn().ok();
        communication_task::spawn().ok();

        rprintln!("System initialized successfully");

        (
            Shared {
                system_state: SystemState::default(),
                task_stats: [TaskStats::default(); 4],
                message_queue: Queue::new(),
            },
            Local {
                led,
                button,
                timer,
                producer,
                consumer,
                memory_pool,
            },
            init::Monotonics(mono),
        )
    }

    // 高优先级传感器任务 - 100Hz
    #[task(shared = [system_state, task_stats], priority = 3)]
    fn sensor_task(mut ctx: sensor_task::Context) {
        let start_time = monotonics::MyMono::now();
        
        // 模拟传感器读取
        let sensor_value = simulate_sensor_reading();
        
        ctx.shared.system_state.lock(|state| {
            state.sensor_value = sensor_value;
        });

        // 更新任务统计
        let execution_time = (monotonics::MyMono::now() - start_time).ticks();
        ctx.shared.task_stats.lock(|stats| {
            stats[0].execution_count += 1;
            stats[0].total_execution_time += execution_time;
            if execution_time > stats[0].max_execution_time {
                stats[0].max_execution_time = execution_time;
            }
        });

        // 检查传感器值是否异常
        if sensor_value > 4000 {
            emergency_handler::spawn().ok();
        }

        // 重新调度
        sensor_task::spawn_after(10.millis()).ok();
    }

    // 中优先级控制任务 - 50Hz
    #[task(shared = [system_state, task_stats], priority = 2)]
    fn control_task(mut ctx: control_task::Context) {
        let start_time = monotonics::MyMono::now();
        
        let (sensor_value, system_active) = ctx.shared.system_state.lock(|state| {
            (state.sensor_value, state.system_active)
        });

        if system_active {
            // 简单的PID控制算法
            let control_output = calculate_control_output(sensor_value);
            
            ctx.shared.system_state.lock(|state| {
                state.control_output = control_output;
            });
        }

        // 更新任务统计
        let execution_time = (monotonics::MyMono::now() - start_time).ticks();
        ctx.shared.task_stats.lock(|stats| {
            stats[1].execution_count += 1;
            stats[1].total_execution_time += execution_time;
            if execution_time > stats[1].max_execution_time {
                stats[1].max_execution_time = execution_time;
            }
        });

        // 重新调度
        control_task::spawn_after(20.millis()).ok();
    }

    // 低优先级监控任务 - 10Hz
    #[task(shared = [system_state, task_stats], local = [led], priority = 1)]
    fn monitoring_task(mut ctx: monitoring_task::Context) {
        let start_time = monotonics::MyMono::now();
        
        let system_state = ctx.shared.system_state.lock(|state| *state);
        
        // LED状态指示
        if system_state.system_active {
            ctx.local.led.set_high();
        } else {
            ctx.local.led.set_low();
        }

        // 系统健康检查
        if system_state.error_count > 10 {
            rprintln!("Warning: High error count detected!");
        }

        // 打印系统状态
        if system_state.sensor_value % 100 == 0 {
            rprintln!("System Status - Sensor: {}, Control: {}, Active: {}", 
                     system_state.sensor_value, 
                     system_state.control_output, 
                     system_state.system_active);
        }

        // 更新任务统计
        let execution_time = (monotonics::MyMono::now() - start_time).ticks();
        ctx.shared.task_stats.lock(|stats| {
            stats[2].execution_count += 1;
            stats[2].total_execution_time += execution_time;
            if execution_time > stats[2].max_execution_time {
                stats[2].max_execution_time = execution_time;
            }
        });

        // 重新调度
        monitoring_task::spawn_after(100.millis()).ok();
    }

    // 通信任务 - 处理消息队列
    #[task(shared = [system_state, task_stats], local = [consumer], priority = 2)]
    fn communication_task(mut ctx: communication_task::Context) {
        let start_time = monotonics::MyMono::now();
        
        // 处理消息队列中的消息
        while let Some(message) = ctx.local.consumer.dequeue() {
            match message {
                TaskMessage::SensorData(value) => {
                    rprintln!("Received sensor data: {}", value);
                }
                TaskMessage::ControlCommand(cmd) => {
                    rprintln!("Received control command: {}", cmd);
                }
                TaskMessage::StatusUpdate(status) => {
                    ctx.shared.system_state.lock(|state| {
                        state.system_active = status;
                    });
                }
                TaskMessage::EmergencyStop => {
                    ctx.shared.system_state.lock(|state| {
                        state.system_active = false;
                        state.error_count += 1;
                    });
                    rprintln!("Emergency stop activated!");
                }
            }
        }

        // 更新任务统计
        let execution_time = (monotonics::MyMono::now() - start_time).ticks();
        ctx.shared.task_stats.lock(|stats| {
            stats[3].execution_count += 1;
            stats[3].total_execution_time += execution_time;
            if execution_time > stats[3].max_execution_time {
                stats[3].max_execution_time = execution_time;
            }
        });

        // 重新调度
        communication_task::spawn_after(50.millis()).ok();
    }

    // 紧急处理任务 - 最高优先级
    #[task(shared = [system_state], priority = 4)]
    fn emergency_handler(mut ctx: emergency_handler::Context) {
        rprintln!("Emergency condition detected!");
        
        ctx.shared.system_state.lock(|state| {
            state.system_active = false;
            state.error_count += 1;
        });

        // 执行紧急停止程序
        // 在实际应用中，这里会关闭所有输出，进入安全状态
    }

    // 按钮中断处理
    #[task(binds = EXTI15_10, shared = [system_state], local = [button], priority = 3)]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        if ctx.local.button.is_low() {
            ctx.shared.system_state.lock(|state| {
                state.system_active = !state.system_active;
            });
            rprintln!("System state toggled via button");
        }
    }

    // 定时器中断 - 用于性能统计
    #[task(binds = TIM2, shared = [task_stats], local = [timer], priority = 1)]
    fn timer_interrupt(mut ctx: timer_interrupt::Context) {
        ctx.local.timer.clear_interrupt(Event::Update);
        
        // 每秒打印一次任务统计信息
        ctx.shared.task_stats.lock(|stats| {
            rprintln!("=== Task Statistics ===");
            rprintln!("Sensor Task: {} executions, avg: {} us, max: {} us", 
                     stats[0].execution_count, 
                     if stats[0].execution_count > 0 { stats[0].total_execution_time / stats[0].execution_count } else { 0 },
                     stats[0].max_execution_time);
            rprintln!("Control Task: {} executions, avg: {} us, max: {} us", 
                     stats[1].execution_count,
                     if stats[1].execution_count > 0 { stats[1].total_execution_time / stats[1].execution_count } else { 0 },
                     stats[1].max_execution_time);
            rprintln!("Monitor Task: {} executions, avg: {} us, max: {} us", 
                     stats[2].execution_count,
                     if stats[2].execution_count > 0 { stats[2].total_execution_time / stats[2].execution_count } else { 0 },
                     stats[2].max_execution_time);
            rprintln!("Comm Task: {} executions, avg: {} us, max: {} us", 
                     stats[3].execution_count,
                     if stats[3].execution_count > 0 { stats[3].total_execution_time / stats[3].execution_count } else { 0 },
                     stats[3].max_execution_time);
            
            // 重置统计信息
            for stat in stats.iter_mut() {
                *stat = TaskStats::default();
            }
        });
    }

    // 空闲任务
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::wfi(); // 等待中断
        }
    }
}

// 辅助函数
fn simulate_sensor_reading() -> u16 {
    // 模拟传感器读取，返回0-4095范围的值
    static mut COUNTER: u16 = 0;
    unsafe {
        COUNTER = COUNTER.wrapping_add(1);
        // 生成一个伪随机值
        (COUNTER * 17 + 42) % 4096
    }
}

fn calculate_control_output(sensor_value: u16) -> u8 {
    // 简单的比例控制
    let setpoint = 2048u16; // 目标值
    let error = if sensor_value > setpoint {
        sensor_value - setpoint
    } else {
        setpoint - sensor_value
    };
    
    // 将误差映射到0-255范围
    ((error as u32 * 255) / 2048) as u8
}

// 单元测试
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_control_output_calculation() {
        assert_eq!(calculate_control_output(2048), 0);
        assert_eq!(calculate_control_output(0), 255);
        assert_eq!(calculate_control_output(4095), 255);
    }

    #[test]
    fn test_task_stats_default() {
        let stats = TaskStats::default();
        assert_eq!(stats.execution_count, 0);
        assert_eq!(stats.total_execution_time, 0);
        assert_eq!(stats.max_execution_time, 0);
        assert_eq!(stats.missed_deadlines, 0);
    }

    #[test]
    fn test_system_state_default() {
        let state = SystemState::default();
        assert_eq!(state.sensor_value, 0);
        assert_eq!(state.control_output, 0);
        assert_eq!(state.system_active, true);
        assert_eq!(state.error_count, 0);
    }
}