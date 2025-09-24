#![no_main]
#![no_std]

// 高级任务调度器
// 功能：实时系统任务管理、优先级调度、截止时间调度
// 硬件：STM32F407VG Discovery
// 演示：多级优先级、抢占式调度、时间片轮转、负载均衡

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
    timer::{Timer, Event},
};

use fugit::ExtU32;
use heapless::{spsc::{Consumer, Producer, Queue}, Vec, FnvIndexMap};
use micromath::F32Ext;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

// 系统配置常量
const SYSTICK_RATE_HZ: u32 = 1000; // 1kHz系统时钟
const MAX_TASKS: usize = 16; // 最大任务数
const MAX_PRIORITY_LEVELS: u8 = 8; // 最大优先级级别
const TIME_SLICE_MS: u32 = 10; // 时间片长度
const SCHEDULER_PERIOD_MS: u32 = 1; // 调度器周期
const LOAD_BALANCE_PERIOD_MS: u32 = 100; // 负载均衡周期
const PERFORMANCE_MONITOR_PERIOD_MS: u32 = 1000; // 性能监控周期

// 全局性能计数器
static TASK_SWITCHES: AtomicU32 = AtomicU32::new(0);
static PREEMPTIONS: AtomicU32 = AtomicU32::new(0);
static DEADLINE_MISSES: AtomicU32 = AtomicU32::new(0);
static SCHEDULER_INVOCATIONS: AtomicU32 = AtomicU32::new(0);
static CURRENT_CPU_LOAD: AtomicU8 = AtomicU8::new(0);

// 任务状态枚举
#[derive(Clone, Copy, Debug, PartialEq)]
enum TaskState {
    Ready,
    Running,
    Blocked,
    Suspended,
    Terminated,
}

// 调度策略枚举
#[derive(Clone, Copy, Debug, PartialEq)]
enum SchedulingPolicy {
    FixedPriority,
    RoundRobin,
    EarliestDeadlineFirst,
    RateMonotonic,
    LoadBalanced,
}

// 任务控制块
#[derive(Clone, Copy, Debug)]
struct TaskControlBlock {
    id: u8,
    priority: u8,
    state: TaskState,
    policy: SchedulingPolicy,
    period_ms: u32,
    deadline_ms: u32,
    execution_time_us: u32,
    remaining_time_slice_ms: u32,
    last_execution_time: u32,
    total_execution_time_us: u64,
    deadline_miss_count: u32,
    preemption_count: u32,
    cpu_usage_percent: u8,
}

impl Default for TaskControlBlock {
    fn default() -> Self {
        Self {
            id: 0,
            priority: 0,
            state: TaskState::Ready,
            policy: SchedulingPolicy::FixedPriority,
            period_ms: 100,
            deadline_ms: 100,
            execution_time_us: 1000,
            remaining_time_slice_ms: TIME_SLICE_MS,
            last_execution_time: 0,
            total_execution_time_us: 0,
            deadline_miss_count: 0,
            preemption_count: 0,
            cpu_usage_percent: 0,
        }
    }
}

// 调度器统计信息
#[derive(Default, Debug)]
struct SchedulerStats {
    total_tasks: u8,
    active_tasks: u8,
    blocked_tasks: u8,
    average_response_time_us: u32,
    max_response_time_us: u32,
    cpu_utilization_percent: u8,
    memory_usage_bytes: u32,
    context_switch_overhead_us: u32,
    scheduler_overhead_percent: u8,
}

// 任务消息类型
#[derive(Clone, Copy, Debug)]
enum TaskMessage {
    TaskCreate { id: u8, priority: u8, policy: SchedulingPolicy },
    TaskTerminate { id: u8 },
    TaskSuspend { id: u8 },
    TaskResume { id: u8 },
    PriorityChange { id: u8, new_priority: u8 },
    DeadlineMiss { id: u8, miss_time_ms: u32 },
    LoadBalanceRequest,
    PerformanceReport,
    SchedulerReset,
}

// 系统事件类型
#[derive(Clone, Copy, Debug)]
enum SystemEvent {
    ButtonPress,
    TimerExpired { timer_id: u8 },
    InterruptReceived { irq_num: u8 },
    ResourceAvailable { resource_id: u8 },
    EmergencyStop,
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, USART2, USART3, UART4])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        // 共享资源
        task_table: Vec<TaskControlBlock, MAX_TASKS>,
        scheduler_stats: SchedulerStats,
        current_task_id: u8,
        system_time_ms: u32,
        task_message_producer: Producer<'static, TaskMessage, 32>,
        system_event_producer: Producer<'static, SystemEvent, 16>,
        scheduling_policy: SchedulingPolicy,
    }

    #[local]
    struct Local {
        // 本地资源
        led_green: PD12<Output<PushPull>>,
        led_orange: PD13<Output<PushPull>>,
        led_red: PD14<Output<PushPull>>,
        led_blue: PD15<Output<PushPull>>,
        button: PA0<Input<PullUp>>,
        task_message_consumer: Consumer<'static, TaskMessage, 32>,
        system_event_consumer: Consumer<'static, SystemEvent, 16>,
        ready_queue: [Vec<u8, MAX_TASKS>; MAX_PRIORITY_LEVELS as usize],
        performance_counters: FnvIndexMap<u8, u32, MAX_TASKS>,
        last_button_state: bool,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // 初始化RTT调试输出
        rtt_init_print!();
        rprintln!("\n=== 高级任务调度器启动 ===");
        rprintln!("功能: 实时系统任务管理和优先级调度");
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
        rprintln!("  PD12: 绿色LED (系统状态)");
        rprintln!("  PD13: 橙色LED (调度活动)");
        rprintln!("  PD14: 红色LED (错误指示)");
        rprintln!("  PD15: 蓝色LED (性能指示)");
        rprintln!("  PA0:  用户按键 (任务控制)");

        // 创建任务间通信队列
        static mut TASK_QUEUE: Queue<TaskMessage, 32> = Queue::new();
        static mut EVENT_QUEUE: Queue<SystemEvent, 16> = Queue::new();
        let (task_message_producer, task_message_consumer) = unsafe { TASK_QUEUE.split() };
        let (system_event_producer, system_event_consumer) = unsafe { EVENT_QUEUE.split() };

        // 初始化任务表
        let mut task_table = Vec::new();
        
        // 创建系统任务
        let system_tasks = [
            (1, 7, SchedulingPolicy::FixedPriority, 10, 10),   // 高优先级系统任务
            (2, 6, SchedulingPolicy::EarliestDeadlineFirst, 20, 15), // EDF任务
            (3, 5, SchedulingPolicy::RoundRobin, 50, 40),      // 轮转任务
            (4, 4, SchedulingPolicy::RateMonotonic, 100, 80),  // RM任务
            (5, 3, SchedulingPolicy::LoadBalanced, 200, 150),  // 负载均衡任务
            (6, 2, SchedulingPolicy::FixedPriority, 500, 400), // 低优先级任务
        ];

        for &(id, priority, policy, period, deadline) in &system_tasks {
            let mut tcb = TaskControlBlock::default();
            tcb.id = id;
            tcb.priority = priority;
            tcb.policy = policy;
            tcb.period_ms = period;
            tcb.deadline_ms = deadline;
            tcb.execution_time_us = (period * 100) as u32; // 10% CPU使用率估算
            task_table.push(tcb).ok();
        }

        rprintln!("\n任务配置:");
        rprintln!("  最大任务数: {}", MAX_TASKS);
        rprintln!("  优先级级别: {}", MAX_PRIORITY_LEVELS);
        rprintln!("  时间片长度: {} ms", TIME_SLICE_MS);
        rprintln!("  调度器频率: {} Hz", 1000 / SCHEDULER_PERIOD_MS);
        rprintln!("  已创建任务: {}", task_table.len());

        // 初始化就绪队列
        let ready_queue: [Vec<u8, MAX_TASKS>; MAX_PRIORITY_LEVELS as usize] = [
            Vec::new(), Vec::new(), Vec::new(), Vec::new(),
            Vec::new(), Vec::new(), Vec::new(), Vec::new(),
        ];

        // 初始化性能计数器
        let mut performance_counters = FnvIndexMap::new();
        for tcb in &task_table {
            performance_counters.insert(tcb.id, 0).ok();
        }

        // 启动调度器和监控任务
        scheduler_task::spawn().ok();
        performance_monitor_task::spawn().ok();
        load_balancer_task::spawn().ok();
        button_handler_task::spawn().ok();
        system_status_task::spawn().ok();

        rprintln!("\n调度器启动完成，开始任务调度...\n");

        (
            Shared {
                task_table,
                scheduler_stats: SchedulerStats::default(),
                current_task_id: 0,
                system_time_ms: 0,
                task_message_producer,
                system_event_producer,
                scheduling_policy: SchedulingPolicy::FixedPriority,
            },
            Local {
                led_green,
                led_orange,
                led_red,
                led_blue,
                button,
                task_message_consumer,
                system_event_consumer,
                ready_queue,
                performance_counters,
                last_button_state: false,
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

    // 主调度器任务 - 最高优先级
    #[task(priority = 8, shared = [task_table, scheduler_stats, current_task_id, system_time_ms, scheduling_policy], local = [ready_queue])]
    async fn scheduler_task(mut ctx: scheduler_task::Context) {
        let mut interval = Systick::delay(SCHEDULER_PERIOD_MS.millis());
        let mut last_schedule_time = Systick::now();
        
        loop {
            interval.next().await;
            
            let schedule_start = Systick::now();
            SCHEDULER_INVOCATIONS.fetch_add(1, Ordering::Relaxed);
            
            // 更新系统时间
            ctx.shared.system_time_ms.lock(|time| *time += SCHEDULER_PERIOD_MS);
            
            // 获取当前调度策略
            let policy = ctx.shared.scheduling_policy.lock(|p| *p);
            
            // 执行调度算法
            let next_task_id = match policy {
                SchedulingPolicy::FixedPriority => {
                    schedule_fixed_priority(&mut ctx, ctx.local.ready_queue)
                },
                SchedulingPolicy::RoundRobin => {
                    schedule_round_robin(&mut ctx, ctx.local.ready_queue)
                },
                SchedulingPolicy::EarliestDeadlineFirst => {
                    schedule_edf(&mut ctx, ctx.local.ready_queue)
                },
                SchedulingPolicy::RateMonotonic => {
                    schedule_rate_monotonic(&mut ctx, ctx.local.ready_queue)
                },
                SchedulingPolicy::LoadBalanced => {
                    schedule_load_balanced(&mut ctx, ctx.local.ready_queue)
                },
            };
            
            // 执行任务切换
            if let Some(task_id) = next_task_id {
                let current_id = ctx.shared.current_task_id.lock(|id| *id);
                if task_id != current_id {
                    perform_task_switch(&mut ctx, task_id);
                }
            }
            
            // 更新调度器统计
            let schedule_time = schedule_start.elapsed();
            ctx.shared.scheduler_stats.lock(|stats| {
                stats.context_switch_overhead_us = schedule_time.to_micros() as u32;
            });
            
            last_schedule_time = schedule_start;
        }
    }

    // 性能监控任务 - 高优先级
    #[task(priority = 7, shared = [task_table, scheduler_stats, system_time_ms], local = [performance_counters, led_blue])]
    async fn performance_monitor_task(mut ctx: performance_monitor_task::Context) {
        let mut interval = Systick::delay(PERFORMANCE_MONITOR_PERIOD_MS.millis());
        let mut led_state = false;
        
        loop {
            interval.next().await;
            
            // 切换性能指示LED
            led_state = !led_state;
            if led_state {
                ctx.local.led_blue.set_high();
            } else {
                ctx.local.led_blue.set_low();
            }
            
            // 计算系统性能指标
            let (total_tasks, active_tasks, cpu_utilization) = ctx.shared.task_table.lock(|tasks| {
                let total = tasks.len() as u8;
                let active = tasks.iter().filter(|t| t.state == TaskState::Running || t.state == TaskState::Ready).count() as u8;
                let cpu_util = tasks.iter().map(|t| t.cpu_usage_percent as u32).sum::<u32>() / tasks.len().max(1) as u32;
                (total, active, cpu_util as u8)
            });
            
            // 更新全局CPU负载
            CURRENT_CPU_LOAD.store(cpu_utilization, Ordering::Relaxed);
            
            // 更新统计信息
            ctx.shared.scheduler_stats.lock(|stats| {
                stats.total_tasks = total_tasks;
                stats.active_tasks = active_tasks;
                stats.cpu_utilization_percent = cpu_utilization;
                stats.memory_usage_bytes = estimate_memory_usage(total_tasks);
            });
            
            // 检查性能警告
            if cpu_utilization > 90 {
                rprintln!("[警告] CPU使用率过高: {}%", cpu_utilization);
            }
            
            let deadline_misses = DEADLINE_MISSES.load(Ordering::Relaxed);
            if deadline_misses > 0 {
                rprintln!("[警告] 检测到 {} 次截止时间错过", deadline_misses);
            }
        }
    }

    // 负载均衡任务 - 中等优先级
    #[task(priority = 5, shared = [task_table, scheduling_policy], local = [led_orange])]
    async fn load_balancer_task(mut ctx: load_balancer_task::Context) {
        let mut interval = Systick::delay(LOAD_BALANCE_PERIOD_MS.millis());
        let mut balance_counter = 0u32;
        
        loop {
            interval.next().await;
            
            balance_counter += 1;
            
            // 切换负载均衡指示LED
            if balance_counter % 10 == 0 {
                ctx.local.led_orange.toggle();
            }
            
            // 分析任务负载分布
            let load_analysis = ctx.shared.task_table.lock(|tasks| {
                analyze_task_load_distribution(tasks)
            });
            
            // 如果检测到负载不均衡，调整调度策略
            if load_analysis.imbalance_detected {
                ctx.shared.scheduling_policy.lock(|policy| {
                    match *policy {
                        SchedulingPolicy::FixedPriority => {
                            *policy = SchedulingPolicy::LoadBalanced;
                            rprintln!("[负载均衡] 切换到负载均衡调度");
                        },
                        SchedulingPolicy::LoadBalanced => {
                            *policy = SchedulingPolicy::EarliestDeadlineFirst;
                            rprintln!("[负载均衡] 切换到EDF调度");
                        },
                        _ => {
                            *policy = SchedulingPolicy::FixedPriority;
                            rprintln!("[负载均衡] 恢复固定优先级调度");
                        }
                    }
                });
            }
            
            // 执行任务优先级调整
            if balance_counter % 50 == 0 {
                ctx.shared.task_table.lock(|tasks| {
                    adjust_task_priorities_for_balance(tasks);
                });
            }
        }
    }

    // 按键处理任务 - 中等优先级
    #[task(priority = 4, shared = [task_message_producer, system_event_producer], local = [button, last_button_state])]
    async fn button_handler_task(mut ctx: button_handler_task::Context) {
        let mut interval = Systick::delay(10.millis());
        
        loop {
            interval.next().await;
            
            let current_state = ctx.local.button.is_low();
            
            if current_state != *ctx.local.last_button_state {
                if current_state {
                    rprintln!("[按键] 按键按下 - 触发任务管理操作");
                    
                    // 发送系统事件
                    ctx.shared.system_event_producer.lock(|producer| {
                        let _ = producer.enqueue(SystemEvent::ButtonPress);
                    });
                    
                    // 发送任务消息（创建新任务或改变优先级）
                    ctx.shared.task_message_producer.lock(|producer| {
                        let _ = producer.enqueue(TaskMessage::TaskCreate {
                            id: 10,
                            priority: 3,
                            policy: SchedulingPolicy::RoundRobin,
                        });
                    });
                }
                
                *ctx.local.last_button_state = current_state;
            }
        }
    }

    // 系统状态任务 - 低优先级
    #[task(priority = 3, shared = [scheduler_stats, system_time_ms], local = [led_green, led_red])]
    async fn system_status_task(mut ctx: system_status_task::Context) {
        let mut interval = Systick::delay(500.millis());
        let mut status_counter = 0u32;
        
        loop {
            interval.next().await;
            
            status_counter += 1;
            
            // 系统状态LED指示
            ctx.local.led_green.toggle(); // 心跳指示
            
            // 检查系统健康状态
            let (cpu_util, active_tasks, uptime) = (
                ctx.shared.scheduler_stats.lock(|stats| stats.cpu_utilization_percent),
                ctx.shared.scheduler_stats.lock(|stats| stats.active_tasks),
                ctx.shared.system_time_ms.lock(|time| *time),
            );
            
            // 错误指示
            if cpu_util > 95 || DEADLINE_MISSES.load(Ordering::Relaxed) > 10 {
                ctx.local.led_red.set_high();
            } else {
                ctx.local.led_red.set_low();
            }
            
            // 每10秒报告一次系统状态
            if status_counter % 20 == 0 {
                let task_switches = TASK_SWITCHES.load(Ordering::Relaxed);
                let preemptions = PREEMPTIONS.load(Ordering::Relaxed);
                let scheduler_calls = SCHEDULER_INVOCATIONS.load(Ordering::Relaxed);
                
                rprintln!("\n=== 系统状态报告 ===");
                rprintln!("  运行时间: {:.1} 秒", uptime as f32 / 1000.0);
                rprintln!("  活跃任务: {}", active_tasks);
                rprintln!("  CPU使用率: {}%", cpu_util);
                rprintln!("  任务切换: {}", task_switches);
                rprintln!("  抢占次数: {}", preemptions);
                rprintln!("  调度器调用: {}", scheduler_calls);
                rprintln!("  截止时间错过: {}", DEADLINE_MISSES.load(Ordering::Relaxed));
                
                if uptime > 0 {
                    rprintln!("  平均切换频率: {:.1} 次/秒", task_switches as f32 * 1000.0 / uptime as f32);
                }
                rprintln!("");
            }
        }
    }

    // 任务消息处理任务 - 中等优先级
    #[task(priority = 4, shared = [task_table, task_message_producer], local = [task_message_consumer])]
    async fn message_handler_task(mut ctx: message_handler_task::Context) {
        loop {
            if let Some(message) = ctx.local.task_message_consumer.dequeue() {
                match message {
                    TaskMessage::TaskCreate { id, priority, policy } => {
                        rprintln!("[消息] 创建任务 ID={}, 优先级={}, 策略={:?}", id, priority, policy);
                        
                        ctx.shared.task_table.lock(|tasks| {
                            if let Some(existing) = tasks.iter_mut().find(|t| t.id == id) {
                                existing.priority = priority;
                                existing.policy = policy;
                                existing.state = TaskState::Ready;
                            } else if tasks.len() < MAX_TASKS {
                                let mut new_task = TaskControlBlock::default();
                                new_task.id = id;
                                new_task.priority = priority;
                                new_task.policy = policy;
                                new_task.state = TaskState::Ready;
                                let _ = tasks.push(new_task);
                            }
                        });
                    },
                    TaskMessage::TaskTerminate { id } => {
                        rprintln!("[消息] 终止任务 ID={}", id);
                        
                        ctx.shared.task_table.lock(|tasks| {
                            if let Some(task) = tasks.iter_mut().find(|t| t.id == id) {
                                task.state = TaskState::Terminated;
                            }
                        });
                    },
                    TaskMessage::PriorityChange { id, new_priority } => {
                        rprintln!("[消息] 任务 {} 优先级变更为 {}", id, new_priority);
                        
                        ctx.shared.task_table.lock(|tasks| {
                            if let Some(task) = tasks.iter_mut().find(|t| t.id == id) {
                                task.priority = new_priority;
                            }
                        });
                    },
                    TaskMessage::DeadlineMiss { id, miss_time_ms } => {
                        rprintln!("[警告] 任务 {} 截止时间错过 {}ms", id, miss_time_ms);
                        DEADLINE_MISSES.fetch_add(1, Ordering::Relaxed);
                        
                        ctx.shared.task_table.lock(|tasks| {
                            if let Some(task) = tasks.iter_mut().find(|t| t.id == id) {
                                task.deadline_miss_count += 1;
                            }
                        });
                    },
                    TaskMessage::PerformanceReport => {
                        performance_report_task::spawn().ok();
                    },
                    _ => {}
                }
            } else {
                Systick::delay(5.millis()).await;
            }
        }
    }

    // 性能报告任务 - 按需触发
    #[task(priority = 2, shared = [task_table, scheduler_stats])]
    async fn performance_report_task(mut ctx: performance_report_task::Context) {
        rprintln!("\n=== 详细性能报告 ===");
        
        let stats = ctx.shared.scheduler_stats.lock(|s| *s);
        
        rprintln!("调度器统计:");
        rprintln!("  总任务数: {}", stats.total_tasks);
        rprintln!("  活跃任务: {}", stats.active_tasks);
        rprintln!("  阻塞任务: {}", stats.blocked_tasks);
        rprintln!("  CPU利用率: {}%", stats.cpu_utilization_percent);
        rprintln!("  内存使用: {} 字节", stats.memory_usage_bytes);
        rprintln!("  上下文切换开销: {} μs", stats.context_switch_overhead_us);
        rprintln!("  调度器开销: {}%", stats.scheduler_overhead_percent);
        rprintln!("  平均响应时间: {} μs", stats.average_response_time_us);
        rprintln!("  最大响应时间: {} μs", stats.max_response_time_us);
        
        rprintln!("\n任务详情:");
        ctx.shared.task_table.lock(|tasks| {
            for task in tasks.iter() {
                rprintln!("  任务 {}: 优先级={}, 状态={:?}, CPU={}%, 错过={}", 
                    task.id, task.priority, task.state, task.cpu_usage_percent, task.deadline_miss_count);
            }
        });
        rprintln!("");
    }
}

// 调度算法实现
fn schedule_fixed_priority(
    ctx: &mut scheduler_task::Context,
    ready_queue: &mut [Vec<u8, MAX_TASKS>; MAX_PRIORITY_LEVELS as usize],
) -> Option<u8> {
    // 固定优先级调度：选择最高优先级的就绪任务
    for (priority, queue) in ready_queue.iter_mut().enumerate().rev() {
        if let Some(&task_id) = queue.first() {
            return Some(task_id);
        }
    }
    None
}

fn schedule_round_robin(
    ctx: &mut scheduler_task::Context,
    ready_queue: &mut [Vec<u8, MAX_TASKS>; MAX_PRIORITY_LEVELS as usize],
) -> Option<u8> {
    // 轮转调度：在同优先级任务间轮转
    for queue in ready_queue.iter_mut().rev() {
        if !queue.is_empty() {
            let task_id = queue.remove(0);
            let _ = queue.push(task_id); // 移到队尾
            return Some(task_id);
        }
    }
    None
}

fn schedule_edf(
    ctx: &mut scheduler_task::Context,
    ready_queue: &mut [Vec<u8, MAX_TASKS>; MAX_PRIORITY_LEVELS as usize],
) -> Option<u8> {
    // 最早截止时间优先调度
    ctx.shared.task_table.lock(|tasks| {
        tasks.iter()
            .filter(|t| t.state == TaskState::Ready)
            .min_by_key(|t| t.deadline_ms)
            .map(|t| t.id)
    })
}

fn schedule_rate_monotonic(
    ctx: &mut scheduler_task::Context,
    ready_queue: &mut [Vec<u8, MAX_TASKS>; MAX_PRIORITY_LEVELS as usize],
) -> Option<u8> {
    // 速率单调调度：周期越短优先级越高
    ctx.shared.task_table.lock(|tasks| {
        tasks.iter()
            .filter(|t| t.state == TaskState::Ready)
            .min_by_key(|t| t.period_ms)
            .map(|t| t.id)
    })
}

fn schedule_load_balanced(
    ctx: &mut scheduler_task::Context,
    ready_queue: &mut [Vec<u8, MAX_TASKS>; MAX_PRIORITY_LEVELS as usize],
) -> Option<u8> {
    // 负载均衡调度：选择CPU使用率最低的任务
    ctx.shared.task_table.lock(|tasks| {
        tasks.iter()
            .filter(|t| t.state == TaskState::Ready)
            .min_by_key(|t| t.cpu_usage_percent)
            .map(|t| t.id)
    })
}

fn perform_task_switch(ctx: &mut scheduler_task::Context, new_task_id: u8) {
    TASK_SWITCHES.fetch_add(1, Ordering::Relaxed);
    
    ctx.shared.current_task_id.lock(|current_id| {
        if *current_id != 0 {
            // 当前任务被抢占
            PREEMPTIONS.fetch_add(1, Ordering::Relaxed);
        }
        *current_id = new_task_id;
    });
    
    // 更新任务状态
    ctx.shared.task_table.lock(|tasks| {
        for task in tasks.iter_mut() {
            if task.id == new_task_id {
                task.state = TaskState::Running;
                task.last_execution_time = ctx.shared.system_time_ms.lock(|t| *t);
            } else if task.state == TaskState::Running {
                task.state = TaskState::Ready;
            }
        }
    });
}

// 负载分析结构
struct LoadAnalysis {
    imbalance_detected: bool,
    max_load_task: u8,
    min_load_task: u8,
    average_load: u8,
}

fn analyze_task_load_distribution(tasks: &Vec<TaskControlBlock, MAX_TASKS>) -> LoadAnalysis {
    if tasks.is_empty() {
        return LoadAnalysis {
            imbalance_detected: false,
            max_load_task: 0,
            min_load_task: 0,
            average_load: 0,
        };
    }
    
    let max_task = tasks.iter().max_by_key(|t| t.cpu_usage_percent).unwrap();
    let min_task = tasks.iter().min_by_key(|t| t.cpu_usage_percent).unwrap();
    let avg_load = tasks.iter().map(|t| t.cpu_usage_percent as u32).sum::<u32>() / tasks.len() as u32;
    
    LoadAnalysis {
        imbalance_detected: max_task.cpu_usage_percent > min_task.cpu_usage_percent + 30,
        max_load_task: max_task.id,
        min_load_task: min_task.id,
        average_load: avg_load as u8,
    }
}

fn adjust_task_priorities_for_balance(tasks: &mut Vec<TaskControlBlock, MAX_TASKS>) {
    // 根据CPU使用率调整任务优先级
    for task in tasks.iter_mut() {
        if task.cpu_usage_percent > 80 {
            // 高负载任务降低优先级
            if task.priority > 1 {
                task.priority -= 1;
            }
        } else if task.cpu_usage_percent < 20 {
            // 低负载任务提高优先级
            if task.priority < MAX_PRIORITY_LEVELS - 1 {
                task.priority += 1;
            }
        }
    }
}

fn estimate_memory_usage(task_count: u8) -> u32 {
    // 估算内存使用量
    let base_usage = 1024u32; // 基础系统内存
    let per_task_usage = 256u32; // 每个任务的内存开销
    base_usage + (task_count as u32 * per_task_usage)
}

// 启动消息处理任务
#[rtic::task(priority = 4, shared = [task_table, task_message_producer], local = [task_message_consumer])]
async fn start_message_handler(ctx: start_message_handler::Context) {
    app::message_handler_task::spawn().ok();
}