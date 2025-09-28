#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm;
use stm32f4xx_hal::{
  gpio::{gpioa::PA5, gpioc::PC13, Input, Output, PullUp, PushPull},
  pac::{Peripherals, TIM2, TIM3, TIM4, TIM5},
  prelude::*,
  timer::{CounterUs, Event},
};

use heapless::{spsc::Queue, FnvIndexMap, Vec};
use rtic::app;
use rtic_monotonics::{systick::*, Monotonic};

// 任务状态枚举
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TaskState {
  Ready,
  Running,
  Blocked,
  Suspended,
  Terminated,
}

// 调度算法类型
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SchedulingAlgorithm {
  Preemptive,
  Cooperative,
  RoundRobin,
  PriorityBased,
  EarliestDeadlineFirst,
  RateMonotonic,
}

// 任务控制块
#[derive(Clone, Copy, Debug)]
pub struct TaskControlBlock {
  pub task_id: u8,
  pub priority: u8,
  pub state: TaskState,
  pub deadline: u32,
  pub period: u32,
  pub execution_time: u32,
  pub remaining_time: u32,
  pub last_execution: u32,
  pub cpu_usage: u32,
}

impl TaskControlBlock {
  pub fn new(task_id: u8, priority: u8, period: u32, execution_time: u32) -> Self {
    Self {
      task_id,
      priority,
      state: TaskState::Ready,
      deadline: 0,
      period,
      execution_time,
      remaining_time: execution_time,
      last_execution: 0,
      cpu_usage: 0,
    }
  }

  pub fn update_deadline(&mut self, current_time: u32) {
    self.deadline = current_time + self.period;
  }

  pub fn reset_execution_time(&mut self) {
    self.remaining_time = self.execution_time;
  }
}

// 调度器统计信息
#[derive(Clone, Copy, Debug, Default)]
pub struct SchedulerStats {
  pub context_switches: u32,
  pub missed_deadlines: u32,
  pub total_cpu_time: u32,
  pub idle_time: u32,
  pub system_load: u8, // 百分比
}

// 性能监控数据
#[derive(Clone, Copy, Debug)]
pub struct PerformanceMetrics {
  pub response_time: u32,
  pub throughput: u32,
  pub utilization: u8,
  pub jitter: u32,
}

impl Default for PerformanceMetrics {
  fn default() -> Self {
    Self {
      response_time: 0,
      throughput: 0,
      utilization: 0,
      jitter: 0,
    }
  }
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4, EXTI9_5, EXTI15_10])]
mod app {
  use super::*;

  #[shared]
  struct Shared {
    scheduler_stats: SchedulerStats,
    current_algorithm: SchedulingAlgorithm,
    system_time: u32,
    task_queue: Vec<TaskControlBlock, 16>,
    performance_metrics: PerformanceMetrics,
  }

  #[local]
  struct Local {
    led: PA5<Output<PushPull>>,
    button: PC13<Input<PullUp>>,
    timer2: CounterUs<TIM2>,    // 系统时钟
    timer3: CounterUs<TIM3>,    // 任务1定时器
    timer4: CounterUs<TIM4>,    // 任务2定时器
    timer5: CounterUs<TIM5>,    // 任务3定时器
    ready_queue: Queue<u8, 16>, // 就绪队列
    current_task: Option<u8>,
    time_slice: u32,
    quantum_counter: u32,
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
    let mut timer2 = ctx.device.TIM2.counter_us(&clocks);
    timer2.start(1.millis()).unwrap(); // 系统时钟 1ms
    timer2.listen(Event::Update);

    let mut timer3 = ctx.device.TIM3.counter_us(&clocks);
    timer3.start(10.millis()).unwrap(); // 任务1周期
    timer3.listen(Event::Update);

    let mut timer4 = ctx.device.TIM4.counter_us(&clocks);
    timer4.start(20.millis()).unwrap(); // 任务2周期
    timer4.listen(Event::Update);

    let mut timer5 = ctx.device.TIM5.counter_us(&clocks);
    timer5.start(50.millis()).unwrap(); // 任务3周期
    timer5.listen(Event::Update);

    // 初始化SysTick单调时钟
    let systick_token = rtic_monotonics::create_systick_token!();
    Systick::start(ctx.core.SYST, 84_000_000, systick_token);

    // 初始化任务队列
    let mut task_queue = Vec::new();

    // 创建示例任务
    let task1 = TaskControlBlock::new(1, 3, 10, 2); // 高优先级，短周期
    let task2 = TaskControlBlock::new(2, 2, 20, 5); // 中优先级，中周期
    let task3 = TaskControlBlock::new(3, 1, 50, 8); // 低优先级，长周期

    task_queue.push(task1).ok();
    task_queue.push(task2).ok();
    task_queue.push(task3).ok();

    let ready_queue = Queue::new();

    // 启动调度器
    scheduler_tick::spawn().ok();
    performance_monitor::spawn().ok();

    cortex_m_log::println!("RTOS Scheduler System Initialized");
    cortex_m_log::println!("Default Algorithm: Priority-Based Preemptive");

    (
      Shared {
        scheduler_stats: SchedulerStats::default(),
        current_algorithm: SchedulingAlgorithm::PriorityBased,
        system_time: 0,
        task_queue,
        performance_metrics: PerformanceMetrics::default(),
      },
      Local {
        led,
        button,
        timer2,
        timer3,
        timer4,
        timer5,
        ready_queue,
        current_task: None,
        time_slice: 10, // 10ms时间片
        quantum_counter: 0,
      },
      init::Monotonics(Systick::new(ctx.core.SYST, 84_000_000)),
    )
  }

  // 空闲任务
  #[idle(shared = [scheduler_stats])]
  fn idle(mut ctx: idle::Context) -> ! {
    loop {
      ctx.shared.scheduler_stats.lock(|stats| {
        stats.idle_time = stats.idle_time.wrapping_add(1);
        asm::wfi(); // 等待中断
      });
    }
  }

  // 系统时钟中断 - 1ms
  #[task(binds = TIM2, local = [timer2, quantum_counter, time_slice], shared = [system_time, scheduler_stats], priority = 5)]
  fn system_tick(mut ctx: system_tick::Context) {
    ctx.local.timer2.clear_interrupt(Event::Update);

    ctx.shared.system_time.lock(|time| {
      *time = time.wrapping_add(1);
    });

    ctx.shared.scheduler_stats.lock(|stats| {
      stats.total_cpu_time = stats.total_cpu_time.wrapping_add(1);
    });

    // 时间片管理
    *ctx.local.quantum_counter += 1;
    if *ctx.local.quantum_counter >= *ctx.local.time_slice {
      *ctx.local.quantum_counter = 0;
      // 时间片用完，触发调度
      scheduler_tick::spawn().ok();
    }
  }

  // 任务1定时器中断
  #[task(binds = TIM3, local = [timer3], shared = [task_queue, system_time], priority = 4)]
  fn task1_timer(mut ctx: task1_timer::Context) {
    ctx.local.timer3.clear_interrupt(Event::Update);

    // 激活任务1
    (ctx.shared.task_queue, ctx.shared.system_time).lock(|queue, time| {
      if let Some(task) = queue.iter_mut().find(|t| t.task_id == 1) {
        task.state = TaskState::Ready;
        task.update_deadline(*time);
        task.reset_execution_time();
      }
    });

    task_activator::spawn(1).ok();
  }

  // 任务2定时器中断
  #[task(binds = TIM4, local = [timer4], shared = [task_queue, system_time], priority = 4)]
  fn task2_timer(mut ctx: task2_timer::Context) {
    ctx.local.timer4.clear_interrupt(Event::Update);

    // 激活任务2
    (ctx.shared.task_queue, ctx.shared.system_time).lock(|queue, time| {
      if let Some(task) = queue.iter_mut().find(|t| t.task_id == 2) {
        task.state = TaskState::Ready;
        task.update_deadline(*time);
        task.reset_execution_time();
      }
    });

    task_activator::spawn(2).ok();
  }

  // 任务3定时器中断
  #[task(binds = TIM5, local = [timer5], shared = [task_queue, system_time], priority = 4)]
  fn task3_timer(mut ctx: task3_timer::Context) {
    ctx.local.timer5.clear_interrupt(Event::Update);

    // 激活任务3
    (ctx.shared.task_queue, ctx.shared.system_time).lock(|queue, time| {
      if let Some(task) = queue.iter_mut().find(|t| t.task_id == 3) {
        task.state = TaskState::Ready;
        task.update_deadline(*time);
        task.reset_execution_time();
      }
    });

    task_activator::spawn(3).ok();
  }

  // 按钮中断 - 切换调度算法
  #[task(binds = EXTI15_10, local = [button], shared = [current_algorithm], priority = 3)]
  fn button_interrupt(mut ctx: button_interrupt::Context) {
    if ctx.local.button.is_low() {
      ctx.shared.current_algorithm.lock(|algorithm| {
        *algorithm = match *algorithm {
          SchedulingAlgorithm::PriorityBased => SchedulingAlgorithm::RoundRobin,
          SchedulingAlgorithm::RoundRobin => SchedulingAlgorithm::EarliestDeadlineFirst,
          SchedulingAlgorithm::EarliestDeadlineFirst => SchedulingAlgorithm::RateMonotonic,
          SchedulingAlgorithm::RateMonotonic => SchedulingAlgorithm::Preemptive,
          SchedulingAlgorithm::Preemptive => SchedulingAlgorithm::Cooperative,
          SchedulingAlgorithm::Cooperative => SchedulingAlgorithm::PriorityBased,
        };

        cortex_m_log::println!("Scheduling algorithm changed to: {:?}", *algorithm);
      });

      // 重新调度
      scheduler_tick::spawn().ok();
    }
  }

  // 任务激活器
  #[task(local = [ready_queue], shared = [task_queue], priority = 2)]
  fn task_activator(mut ctx: task_activator::Context, task_id: u8) {
    ctx.shared.task_queue.lock(|queue| {
      if let Some(task) = queue.iter_mut().find(|t| t.task_id == task_id) {
        if task.state == TaskState::Ready {
          // 将任务添加到就绪队列
          ctx.local.ready_queue.enqueue(task_id).ok();
          cortex_m_log::println!("Task {} activated", task_id);
        }
      }
    });

    // 触发调度
    scheduler_tick::spawn().ok();
  }

  // 调度器主循环
  #[task(local = [ready_queue, current_task], shared = [task_queue, current_algorithm, scheduler_stats, system_time], priority = 2)]
  fn scheduler_tick(mut ctx: scheduler_tick::Context) {
    let algorithm = *ctx.shared.current_algorithm.lock(|alg| *alg);

    // 根据调度算法选择下一个任务
    let next_task = match algorithm {
      SchedulingAlgorithm::PriorityBased => priority_based_scheduler(&mut ctx),
      SchedulingAlgorithm::RoundRobin => round_robin_scheduler(&mut ctx),
      SchedulingAlgorithm::EarliestDeadlineFirst => edf_scheduler(&mut ctx),
      SchedulingAlgorithm::RateMonotonic => rate_monotonic_scheduler(&mut ctx),
      SchedulingAlgorithm::Preemptive => preemptive_scheduler(&mut ctx),
      SchedulingAlgorithm::Cooperative => cooperative_scheduler(&mut ctx),
    };

    // 执行上下文切换
    if let Some(task_id) = next_task {
      if *ctx.local.current_task != Some(task_id) {
        ctx.shared.scheduler_stats.lock(|stats| {
          stats.context_switches = stats.context_switches.wrapping_add(1);
        });

        *ctx.local.current_task = Some(task_id);
        task_executor::spawn(task_id).ok();
      }
    }
  }

  // 任务执行器
  #[task(shared = [task_queue, system_time, performance_metrics], priority = 1)]
  fn task_executor(mut ctx: task_executor::Context, task_id: u8) {
    let start_time = *ctx.shared.system_time.lock(|time| *time);

    ctx.shared.task_queue.lock(|queue| {
      if let Some(task) = queue.iter_mut().find(|t| t.task_id == task_id) {
        task.state = TaskState::Running;
        task.last_execution = start_time;

        cortex_m_log::println!(
          "Executing Task {} - Priority: {}, Deadline: {}, Remaining: {}",
          task.task_id,
          task.priority,
          task.deadline,
          task.remaining_time
        );
      }
    });

    // 模拟任务执行
    match task_id {
      1 => high_priority_task::spawn().ok(),
      2 => medium_priority_task::spawn().ok(),
      3 => low_priority_task::spawn().ok(),
      _ => {}
    }

    // 更新性能指标
    let end_time = *ctx.shared.system_time.lock(|time| *time);
    let execution_time = end_time - start_time;

    ctx.shared.performance_metrics.lock(|metrics| {
      metrics.response_time = execution_time;
      metrics.throughput = metrics.throughput.wrapping_add(1);
    });
  }

  // 高优先级任务
  #[task(local = [led], shared = [task_queue], priority = 3)]
  fn high_priority_task(mut ctx: high_priority_task::Context) {
    // 模拟高优先级任务工作
    ctx.local.led.set_high();

    // 模拟计算工作
    for _ in 0..1000 {
      asm::nop();
    }

    ctx.local.led.set_low();

    // 更新任务状态
    ctx.shared.task_queue.lock(|queue| {
      if let Some(task) = queue.iter_mut().find(|t| t.task_id == 1) {
        task.remaining_time = task.remaining_time.saturating_sub(1);
        if task.remaining_time == 0 {
          task.state = TaskState::Terminated;
          cortex_m_log::println!("Task 1 completed");
        }
      }
    });
  }

  // 中优先级任务
  #[task(shared = [task_queue], priority = 2)]
  fn medium_priority_task(mut ctx: medium_priority_task::Context) {
    // 模拟中优先级任务工作
    cortex_m_log::println!("Medium priority task executing");

    // 模拟I/O操作
    for _ in 0..2000 {
      asm::nop();
    }

    // 更新任务状态
    ctx.shared.task_queue.lock(|queue| {
      if let Some(task) = queue.iter_mut().find(|t| t.task_id == 2) {
        task.remaining_time = task.remaining_time.saturating_sub(1);
        if task.remaining_time == 0 {
          task.state = TaskState::Terminated;
          cortex_m_log::println!("Task 2 completed");
        }
      }
    });
  }

  // 低优先级任务
  #[task(shared = [task_queue], priority = 1)]
  fn low_priority_task(mut ctx: low_priority_task::Context) {
    // 模拟低优先级任务工作
    cortex_m_log::println!("Low priority task executing");

    // 模拟长时间计算
    for _ in 0..5000 {
      asm::nop();
    }

    // 更新任务状态
    ctx.shared.task_queue.lock(|queue| {
      if let Some(task) = queue.iter_mut().find(|t| t.task_id == 3) {
        task.remaining_time = task.remaining_time.saturating_sub(1);
        if task.remaining_time == 0 {
          task.state = TaskState::Terminated;
          cortex_m_log::println!("Task 3 completed");
        }
      }
    });
  }

  // 性能监控任务
  #[task(shared = [scheduler_stats, performance_metrics, task_queue], priority = 1)]
  fn performance_monitor(mut ctx: performance_monitor::Context) {
    (
      ctx.shared.scheduler_stats,
      ctx.shared.performance_metrics,
      ctx.shared.task_queue,
    )
      .lock(|stats, metrics, queue| {
        // 计算系统负载
        let total_time = stats.total_cpu_time;
        let idle_percentage = if total_time > 0 {
          ((stats.idle_time * 100) / total_time) as u8
        } else {
          100
        };
        stats.system_load = 100 - idle_percentage;

        // 计算CPU利用率
        metrics.utilization = stats.system_load;

        cortex_m_log::println!(
          "Performance Monitor - Load: {}%, Switches: {}, Missed Deadlines: {}",
          stats.system_load,
          stats.context_switches,
          stats.missed_deadlines
        );

        // 检查任务状态
        for task in queue.iter() {
          cortex_m_log::println!(
            "Task {} - State: {:?}, Priority: {}, CPU Usage: {}",
            task.task_id,
            task.state,
            task.priority,
            task.cpu_usage
          );
        }
      });

    // 每5秒重新调度
    performance_monitor::spawn_after(5.secs()).ok();
  }

  // 死锁检测任务
  #[task(shared = [task_queue, scheduler_stats], priority = 1)]
  fn deadlock_detector(mut ctx: deadlock_detector::Context) {
    let mut blocked_tasks = 0;

    ctx.shared.task_queue.lock(|queue| {
      for task in queue.iter() {
        if task.state == TaskState::Blocked {
          blocked_tasks += 1;
        }
      }
    });

    if blocked_tasks > 2 {
      cortex_m_log::println!(
        "WARNING: Potential deadlock detected - {} blocked tasks",
        blocked_tasks
      );

      ctx.shared.scheduler_stats.lock(|stats| {
        stats.missed_deadlines = stats.missed_deadlines.wrapping_add(1);
      });
    }

    // 每10秒检查一次
    deadlock_detector::spawn_after(10.secs()).ok();
  }
}

// 调度算法实现
fn priority_based_scheduler(ctx: &mut scheduler_tick::Context) -> Option<u8> {
  let mut highest_priority = 0;
  let mut selected_task = None;

  while let Some(task_id) = ctx.local.ready_queue.dequeue() {
    ctx.shared.task_queue.lock(|queue| {
      if let Some(task) = queue.iter().find(|t| t.task_id == task_id) {
        if task.priority > highest_priority && task.state == TaskState::Ready {
          highest_priority = task.priority;
          selected_task = Some(task_id);
        }
      }
    });
  }

  selected_task
}

fn round_robin_scheduler(ctx: &mut scheduler_tick::Context) -> Option<u8> {
  // 简单的轮转调度
  ctx.local.ready_queue.dequeue()
}

fn edf_scheduler(ctx: &mut scheduler_tick::Context) -> Option<u8> {
  let mut earliest_deadline = u32::MAX;
  let mut selected_task = None;

  while let Some(task_id) = ctx.local.ready_queue.dequeue() {
    ctx.shared.task_queue.lock(|queue| {
      if let Some(task) = queue.iter().find(|t| t.task_id == task_id) {
        if task.deadline < earliest_deadline && task.state == TaskState::Ready {
          earliest_deadline = task.deadline;
          selected_task = Some(task_id);
        }
      }
    });
  }

  selected_task
}

fn rate_monotonic_scheduler(ctx: &mut scheduler_tick::Context) -> Option<u8> {
  let mut shortest_period = u32::MAX;
  let mut selected_task = None;

  while let Some(task_id) = ctx.local.ready_queue.dequeue() {
    ctx.shared.task_queue.lock(|queue| {
      if let Some(task) = queue.iter().find(|t| t.task_id == task_id) {
        if task.period < shortest_period && task.state == TaskState::Ready {
          shortest_period = task.period;
          selected_task = Some(task_id);
        }
      }
    });
  }

  selected_task
}

fn preemptive_scheduler(ctx: &mut scheduler_tick::Context) -> Option<u8> {
  // 抢占式调度，类似优先级调度但允许抢占
  priority_based_scheduler(ctx)
}

fn cooperative_scheduler(ctx: &mut scheduler_tick::Context) -> Option<u8> {
  // 协作式调度，只有当前任务完成才切换
  if ctx.local.current_task.is_none() {
    ctx.local.ready_queue.dequeue()
  } else {
    *ctx.local.current_task
  }
}

// 测试函数
#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_task_control_block() {
    let mut task = TaskControlBlock::new(1, 3, 100, 10);
    assert_eq!(task.task_id, 1);
    assert_eq!(task.priority, 3);
    assert_eq!(task.state, TaskState::Ready);

    task.update_deadline(50);
    assert_eq!(task.deadline, 150);

    task.reset_execution_time();
    assert_eq!(task.remaining_time, 10);
  }

  #[test]
  fn test_scheduler_stats() {
    let mut stats = SchedulerStats::default();
    stats.context_switches += 1;
    stats.missed_deadlines += 1;

    assert_eq!(stats.context_switches, 1);
    assert_eq!(stats.missed_deadlines, 1);
  }
}
