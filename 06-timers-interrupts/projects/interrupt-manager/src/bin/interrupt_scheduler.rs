#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use interrupt_manager::{
  get_global_interrupt_manager, init_global_interrupt_manager, interrupt_handler, InterruptId,
  InterruptManager, Priority,
};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{
    gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7},
    Output, PushPull,
  },
  gpio::{gpioc::PC13, Input, PullUp},
  interrupt,
  prelude::*,
  stm32,
  timer::{Event, Timer},
};

// LED类型定义
type StatusLed = PB0<Output<PushPull>>;
type SchedulerLed = PB1<Output<PushPull>>;
type HighPrioLed = PB2<Output<PushPull>>;
type MediumPrioLed = PB3<Output<PushPull>>;
type LowPrioLed = PB4<Output<PushPull>>;
type QueueLed = PB5<Output<PushPull>>;
type OverloadLed = PB6<Output<PushPull>>;
type ErrorLed = PB7<Output<PushPull>>;

type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut SCHEDULER_QUEUE: Queue<ScheduledTask, 64> = Queue::new();
static mut SCHEDULER_PRODUCER: Option<Producer<ScheduledTask, 64>> = None;
static mut SCHEDULER_CONSUMER: Option<Consumer<ScheduledTask, 64>> = None;

static SYSTEM_TIME_US: AtomicU32 = AtomicU32::new(0);
static SCHEDULER_MODE: AtomicU8 = AtomicU8::new(0);
static ACTIVE_TASKS: AtomicU16 = AtomicU16::new(0);
static COMPLETED_TASKS: AtomicU32 = AtomicU32::new(0);
static DROPPED_TASKS: AtomicU32 = AtomicU32::new(0);
static SCHEDULER_ENABLED: AtomicBool = AtomicBool::new(true);

// 任务优先级队列计数器
static TASK_COUNTERS: [AtomicU32; 4] = [
  AtomicU32::new(0), // Critical
  AtomicU32::new(0), // High
  AtomicU32::new(0), // Medium
  AtomicU32::new(0), // Low
];

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  let mut status_led = gpiob.pb0.into_push_pull_output();
  let mut scheduler_led = gpiob.pb1.into_push_pull_output();
  let mut high_prio_led = gpiob.pb2.into_push_pull_output();
  let mut medium_prio_led = gpiob.pb3.into_push_pull_output();
  let mut low_prio_led = gpiob.pb4.into_push_pull_output();
  let mut queue_led = gpiob.pb5.into_push_pull_output();
  let mut overload_led = gpiob.pb6.into_push_pull_output();
  let mut error_led = gpiob.pb7.into_push_pull_output();

  let mode_button = gpioc.pc13.into_pull_up_input();

  // 初始化全局中断管理器
  init_global_interrupt_manager();

  // 初始化调度器队列
  let (producer, consumer) = unsafe { SCHEDULER_QUEUE.split() };
  unsafe {
    SCHEDULER_PRODUCER = Some(producer);
    SCHEDULER_CONSUMER = Some(consumer);
  }

  // 配置定时器 - 不同类型的任务源
  let mut timer2 = Timer::tim2(dp.TIM2, &clocks);
  let mut timer3 = Timer::tim3(dp.TIM3, &clocks);
  let mut timer4 = Timer::tim4(dp.TIM4, &clocks);
  let mut timer5 = Timer::tim5(dp.TIM5, &clocks);

  // 设置不同频率的定时器产生不同优先级的任务
  timer2.start(200.hz()); // 200Hz - 产生关键任务
  timer3.start(150.hz()); // 150Hz - 产生高优先级任务
  timer4.start(100.hz()); // 100Hz - 产生中等优先级任务
  timer5.start(50.hz()); // 50Hz - 产生低优先级任务

  // 启用定时器中断
  timer2.listen(Event::TimeOut);
  timer3.listen(Event::TimeOut);
  timer4.listen(Event::TimeOut);
  timer5.listen(Event::TimeOut);

  // 配置中断管理器
  if let Some(manager) = get_global_interrupt_manager() {
    // 注册中断
    manager
      .register_interrupt(InterruptId::Timer2, Priority::Highest)
      .ok();
    manager
      .register_interrupt(InterruptId::Timer3, Priority::High)
      .ok();
    manager
      .register_interrupt(InterruptId::Timer4, Priority::Medium)
      .ok();
    manager
      .register_interrupt(InterruptId::Timer5, Priority::Low)
      .ok();

    // 启用中断
    manager.enable_interrupt(InterruptId::Timer2).ok();
    manager.enable_interrupt(InterruptId::Timer3).ok();
    manager.enable_interrupt(InterruptId::Timer4).ok();
    manager.enable_interrupt(InterruptId::Timer5).ok();
  }

  // 配置NVIC
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

  // 中断调度器
  let mut interrupt_scheduler = InterruptScheduler::new();
  let mut button_debouncer = ButtonDebouncer::new();
  let mut demo_counter = 0u32;

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
      // 切换调度模式
      let current_mode = SCHEDULER_MODE.load(Ordering::Relaxed);
      let new_mode = (current_mode + 1) % 4;
      SCHEDULER_MODE.store(new_mode, Ordering::Relaxed);

      // 应用新的调度策略
      interrupt_scheduler.apply_scheduling_mode(new_mode);

      // 重置统计
      interrupt_scheduler.reset_statistics();

      // 模式切换指示
      for _ in 0..3 {
        scheduler_led.set_low();
        delay_timer.delay_ms(100u32);
        scheduler_led.set_high();
        delay_timer.delay_ms(100u32);
      }
    }

    // 处理调度任务
    if let Some(consumer) = unsafe { SCHEDULER_CONSUMER.as_mut() } {
      while let Some(task) = consumer.dequeue() {
        interrupt_scheduler.schedule_task(task);
      }
    }

    // 执行调度器
    interrupt_scheduler.run_scheduler();

    // 显示调度状态
    display_scheduler_status(
      &interrupt_scheduler,
      &mut high_prio_led,
      &mut medium_prio_led,
      &mut low_prio_led,
      &mut queue_led,
      &mut overload_led,
      &mut error_led,
      demo_counter,
    );

    // 状态LED心跳
    if demo_counter % 100 == 0 {
      status_led.toggle();
    }

    // 调度器活动指示
    let active_tasks = ACTIVE_TASKS.load(Ordering::Relaxed);
    scheduler_led.set_state((active_tasks > 0).into()).ok();

    delay_timer.delay_ms(10u32);
  }
}

/// 显示调度器状态
fn display_scheduler_status(
  scheduler: &InterruptScheduler,
  high_prio_led: &mut impl embedded_hal::digital::v2::OutputPin,
  medium_prio_led: &mut impl embedded_hal::digital::v2::OutputPin,
  low_prio_led: &mut impl embedded_hal::digital::v2::OutputPin,
  queue_led: &mut impl embedded_hal::digital::v2::OutputPin,
  overload_led: &mut impl embedded_hal::digital::v2::OutputPin,
  error_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  let stats = scheduler.get_scheduler_statistics();

  // 根据任务优先级计数显示LED
  let high_count = TASK_COUNTERS[1].load(Ordering::Relaxed);
  let medium_count = TASK_COUNTERS[2].load(Ordering::Relaxed);
  let low_count = TASK_COUNTERS[3].load(Ordering::Relaxed);

  high_prio_led
    .set_state((high_count > 0 && (counter / 8) % 2 == 0).into())
    .ok();
  medium_prio_led
    .set_state((medium_count > 0 && (counter / 12) % 2 == 0).into())
    .ok();
  low_prio_led
    .set_state((low_count > 0 && (counter / 16) % 2 == 0).into())
    .ok();

  // 队列状态指示
  let queue_utilization = stats.queue_utilization;
  if queue_utilization > 80 {
    queue_led.set_state(((counter / 3) % 2 == 0).into()).ok();
  } else if queue_utilization > 50 {
    queue_led.set_state(((counter / 6) % 2 == 0).into()).ok();
  } else if queue_utilization > 0 {
    queue_led.set_state(((counter / 10) % 2 == 0).into()).ok();
  } else {
    queue_led.set_low().ok();
  }

  // 过载指示
  let dropped_tasks = DROPPED_TASKS.load(Ordering::Relaxed);
  if dropped_tasks > 0 {
    overload_led.set_state(((counter / 4) % 2 == 0).into()).ok();
  } else {
    overload_led.set_low().ok();
  }

  // 错误指示
  if stats.error_count > 0 {
    error_led.set_state(((counter / 7) % 2 == 0).into()).ok();
  } else {
    error_led.set_low().ok();
  }
}

/// 中断调度器
struct InterruptScheduler {
  current_mode: u8,
  scheduler_stats: SchedulerStatistics,
  task_queues: [heapless::Vec<ScheduledTask, 16>; 4], // 4个优先级队列
  time_slice_us: u32,
  max_execution_time_us: u32,
}

impl InterruptScheduler {
  fn new() -> Self {
    Self {
      current_mode: 0,
      scheduler_stats: SchedulerStatistics::default(),
      task_queues: [
        heapless::Vec::new(), // Critical
        heapless::Vec::new(), // High
        heapless::Vec::new(), // Medium
        heapless::Vec::new(), // Low
      ],
      time_slice_us: 1000,         // 1ms time slice
      max_execution_time_us: 5000, // 5ms max execution
    }
  }

  fn apply_scheduling_mode(&mut self, mode: u8) {
    self.current_mode = mode;

    match mode {
      0 => {
        // 优先级调度 - 严格按优先级执行
        self.time_slice_us = 0; // 不使用时间片
        self.max_execution_time_us = 10000;
      }
      1 => {
        // 时间片轮转 - 每个任务固定时间片
        self.time_slice_us = 1000; // 1ms时间片
        self.max_execution_time_us = 1000;
      }
      2 => {
        // 混合调度 - 优先级+时间片
        self.time_slice_us = 2000; // 2ms时间片
        self.max_execution_time_us = 5000;
      }
      3 => {
        // 自适应调度 - 根据负载动态调整
        let cpu_load = self.estimate_cpu_load();
        if cpu_load > 80 {
          self.time_slice_us = 500; // 短时间片
          self.max_execution_time_us = 2000;
        } else if cpu_load > 50 {
          self.time_slice_us = 1500;
          self.max_execution_time_us = 4000;
        } else {
          self.time_slice_us = 3000; // 长时间片
          self.max_execution_time_us = 8000;
        }
      }
      _ => {}
    }
  }

  fn schedule_task(&mut self, task: ScheduledTask) {
    let priority_index = match task.priority {
      TaskPriority::Critical => 0,
      TaskPriority::High => 1,
      TaskPriority::Medium => 2,
      TaskPriority::Low => 3,
    };

    // 尝试添加到对应优先级队列
    if self.task_queues[priority_index].push(task).is_err() {
      // 队列满，丢弃任务
      DROPPED_TASKS.fetch_add(1, Ordering::Relaxed);
      self.scheduler_stats.dropped_tasks += 1;
    } else {
      TASK_COUNTERS[priority_index].fetch_add(1, Ordering::Relaxed);
      ACTIVE_TASKS.fetch_add(1, Ordering::Relaxed);
    }
  }

  fn run_scheduler(&mut self) {
    let start_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

    match self.current_mode {
      0 => self.run_priority_scheduler(),
      1 => self.run_round_robin_scheduler(),
      2 => self.run_hybrid_scheduler(),
      3 => self.run_adaptive_scheduler(),
      _ => {}
    }

    let end_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
    let execution_time = end_time.wrapping_sub(start_time);

    self.scheduler_stats.total_scheduler_time += execution_time;
    self.scheduler_stats.scheduler_runs += 1;
  }

  fn run_priority_scheduler(&mut self) {
    // 严格优先级调度 - 从最高优先级开始执行
    for priority_index in 0..4 {
      if let Some(task) = self.task_queues[priority_index].pop() {
        self.execute_task(task);
        break; // 只执行一个任务
      }
    }
  }

  fn run_round_robin_scheduler(&mut self) {
    // 时间片轮转 - 在所有非空队列中轮转
    static mut CURRENT_PRIORITY: usize = 0;

    let start_priority = unsafe { CURRENT_PRIORITY };

    for _ in 0..4 {
      let priority_index = unsafe { CURRENT_PRIORITY };

      if let Some(task) = self.task_queues[priority_index].pop() {
        self.execute_task_with_time_slice(task);
        break;
      }

      unsafe {
        CURRENT_PRIORITY = (CURRENT_PRIORITY + 1) % 4;
      }
    }
  }

  fn run_hybrid_scheduler(&mut self) {
    // 混合调度 - 优先级+时间片
    // 高优先级任务优先，但有时间片限制
    for priority_index in 0..4 {
      if let Some(task) = self.task_queues[priority_index].pop() {
        if priority_index <= 1 {
          // 高优先级任务，较长时间片
          self.execute_task_with_time_slice(task);
        } else {
          // 低优先级任务，较短时间片
          self.execute_task_with_limited_time(task, self.time_slice_us / 2);
        }
        break;
      }
    }
  }

  fn run_adaptive_scheduler(&mut self) {
    // 自适应调度 - 根据系统状态动态调整
    let cpu_load = self.estimate_cpu_load();
    let queue_pressure = self.calculate_queue_pressure();

    if cpu_load > 90 || queue_pressure > 80 {
      // 高负载模式 - 快速处理
      self.run_priority_scheduler();
    } else if queue_pressure > 50 {
      // 中等负载 - 混合调度
      self.run_hybrid_scheduler();
    } else {
      // 低负载 - 轮转调度
      self.run_round_robin_scheduler();
    }
  }

  fn execute_task(&mut self, task: ScheduledTask) {
    let start_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

    // 模拟任务执行
    self.simulate_task_execution(&task);

    let end_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
    let execution_time = end_time.wrapping_sub(start_time);

    // 更新统计
    self.update_task_statistics(task, execution_time);

    ACTIVE_TASKS.fetch_sub(1, Ordering::Relaxed);
    COMPLETED_TASKS.fetch_add(1, Ordering::Relaxed);
  }

  fn execute_task_with_time_slice(&mut self, task: ScheduledTask) {
    let start_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
    let time_limit = start_time + self.time_slice_us;

    // 模拟带时间片的任务执行
    self.simulate_task_execution_with_limit(&task, time_limit);

    let end_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
    let execution_time = end_time.wrapping_sub(start_time);

    self.update_task_statistics(task, execution_time);

    ACTIVE_TASKS.fetch_sub(1, Ordering::Relaxed);
    COMPLETED_TASKS.fetch_add(1, Ordering::Relaxed);
  }

  fn execute_task_with_limited_time(&mut self, task: ScheduledTask, time_limit_us: u32) {
    let start_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
    let time_limit = start_time + time_limit_us;

    self.simulate_task_execution_with_limit(&task, time_limit);

    let end_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
    let execution_time = end_time.wrapping_sub(start_time);

    self.update_task_statistics(task, execution_time);

    ACTIVE_TASKS.fetch_sub(1, Ordering::Relaxed);
    COMPLETED_TASKS.fetch_add(1, Ordering::Relaxed);
  }

  fn simulate_task_execution(&self, task: &ScheduledTask) {
    // 根据任务类型模拟不同的执行时间
    let cycles = match task.task_type {
      TaskType::Critical => 50,
      TaskType::DataProcessing => 200,
      TaskType::Communication => 150,
      TaskType::Background => 300,
    };

    for _ in 0..cycles {
      cortex_m::asm::nop();
    }
  }

  fn simulate_task_execution_with_limit(&self, task: &ScheduledTask, time_limit: u32) {
    let cycles = match task.task_type {
      TaskType::Critical => 50,
      TaskType::DataProcessing => 200,
      TaskType::Communication => 150,
      TaskType::Background => 300,
    };

    for i in 0..cycles {
      cortex_m::asm::nop();

      // 检查时间限制
      if i % 10 == 0 {
        let current_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
        if current_time >= time_limit {
          break;
        }
      }
    }
  }

  fn update_task_statistics(&mut self, task: ScheduledTask, execution_time: u32) {
    self.scheduler_stats.total_tasks += 1;
    self.scheduler_stats.total_execution_time += execution_time;

    if execution_time > self.scheduler_stats.max_task_time {
      self.scheduler_stats.max_task_time = execution_time;
    }

    // 更新平均执行时间
    self.scheduler_stats.avg_task_time =
      self.scheduler_stats.total_execution_time / self.scheduler_stats.total_tasks;
  }

  fn get_scheduler_statistics(&self) -> &SchedulerStatistics {
    &self.scheduler_stats
  }

  fn reset_statistics(&mut self) {
    self.scheduler_stats = SchedulerStatistics::default();

    // 重置计数器
    for counter in &TASK_COUNTERS {
      counter.store(0, Ordering::Relaxed);
    }

    COMPLETED_TASKS.store(0, Ordering::Relaxed);
    DROPPED_TASKS.store(0, Ordering::Relaxed);
    ACTIVE_TASKS.store(0, Ordering::Relaxed);

    // 清空队列
    for queue in &mut self.task_queues {
      queue.clear();
    }
  }

  fn estimate_cpu_load(&self) -> u32 {
    let total_time =
      self.scheduler_stats.total_execution_time + self.scheduler_stats.total_scheduler_time;
    let elapsed_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

    if elapsed_time > 0 {
      ((total_time as f32 / elapsed_time as f32) * 100.0) as u32
    } else {
      0
    }
  }

  fn calculate_queue_pressure(&self) -> u32 {
    let total_tasks: usize = self.task_queues.iter().map(|q| q.len()).sum();
    let total_capacity = self.task_queues.len() * 16; // 每个队列容量16

    ((total_tasks as f32 / total_capacity as f32) * 100.0) as u32
  }
}

/// 调度器统计信息
#[derive(Default)]
struct SchedulerStatistics {
  total_tasks: u32,
  dropped_tasks: u32,
  total_execution_time: u32,
  total_scheduler_time: u32,
  avg_task_time: u32,
  max_task_time: u32,
  scheduler_runs: u32,
  queue_utilization: u32,
  error_count: u32,
}

/// 调度任务
#[derive(Clone, Copy)]
struct ScheduledTask {
  task_id: u16,
  task_type: TaskType,
  priority: TaskPriority,
  created_time: u32,
  deadline: u32,
  estimated_duration: u32,
  interrupt_source: InterruptId,
}

/// 任务类型
#[derive(Clone, Copy)]
enum TaskType {
  Critical,
  DataProcessing,
  Communication,
  Background,
}

/// 任务优先级
#[derive(Clone, Copy)]
enum TaskPriority {
  Critical,
  High,
  Medium,
  Low,
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

// 任务ID计数器
static TASK_ID_COUNTER: AtomicU16 = AtomicU16::new(0);

/// 创建新任务
fn create_task(
  task_type: TaskType,
  priority: TaskPriority,
  interrupt_source: InterruptId,
) -> ScheduledTask {
  let task_id = TASK_ID_COUNTER.fetch_add(1, Ordering::Relaxed);
  let current_time = SYSTEM_TIME_US.load(Ordering::Relaxed);

  let estimated_duration = match task_type {
    TaskType::Critical => 100,
    TaskType::DataProcessing => 500,
    TaskType::Communication => 300,
    TaskType::Background => 1000,
  };

  let deadline = current_time + estimated_duration * 10; // 10倍估算时间作为截止时间

  ScheduledTask {
    task_id,
    task_type,
    priority,
    created_time: current_time,
    deadline,
    estimated_duration,
    interrupt_source,
  }
}

// 中断处理函数
#[interrupt]
fn TIM2() {
  interrupt_handler!(InterruptId::Timer2, {
    // 产生关键任务
    let task = create_task(
      TaskType::Critical,
      TaskPriority::Critical,
      InterruptId::Timer2,
    );

    if let Some(producer) = unsafe { SCHEDULER_PRODUCER.as_mut() } {
      producer.enqueue(task).ok();
    }
  });

  // 清除中断标志
  unsafe {
    let tim2 = &(*stm32::TIM2::ptr());
    tim2.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM3() {
  interrupt_handler!(InterruptId::Timer3, {
    // 产生高优先级任务
    let task = create_task(
      TaskType::DataProcessing,
      TaskPriority::High,
      InterruptId::Timer3,
    );

    if let Some(producer) = unsafe { SCHEDULER_PRODUCER.as_mut() } {
      producer.enqueue(task).ok();
    }
  });

  // 清除中断标志
  unsafe {
    let tim3 = &(*stm32::TIM3::ptr());
    tim3.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM4() {
  interrupt_handler!(InterruptId::Timer4, {
    // 产生中等优先级任务
    let task = create_task(
      TaskType::Communication,
      TaskPriority::Medium,
      InterruptId::Timer4,
    );

    if let Some(producer) = unsafe { SCHEDULER_PRODUCER.as_mut() } {
      producer.enqueue(task).ok();
    }
  });

  // 清除中断标志
  unsafe {
    let tim4 = &(*stm32::TIM4::ptr());
    tim4.sr.modify(|_, w| w.uif().clear_bit());
  }
}

#[interrupt]
fn TIM5() {
  interrupt_handler!(InterruptId::Timer5, {
    // 产生低优先级任务
    let task = create_task(TaskType::Background, TaskPriority::Low, InterruptId::Timer5);

    if let Some(producer) = unsafe { SCHEDULER_PRODUCER.as_mut() } {
      producer.enqueue(task).ok();
    }
  });

  // 清除中断标志
  unsafe {
    let tim5 = &(*stm32::TIM5::ptr());
    tim5.sr.modify(|_, w| w.uif().clear_bit());
  }
}
