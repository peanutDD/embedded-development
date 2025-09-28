#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless::Vec;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{
    gpiod::{PD12, PD13, PD14, PD15},
    Output, PushPull,
  },
  interrupt,
  prelude::*,
  stm32,
  timer::{Event, Timer},
};

type LedPin = Output<PushPull>;

// 全局变量
static SCHEDULER: Mutex<RefCell<Option<CooperativeScheduler>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM6>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
  // 获取外设
  let dp = stm32::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpiod = dp.GPIOD.split();
  let led1 = gpiod.pd12.into_push_pull_output().erase();
  let led2 = gpiod.pd13.into_push_pull_output().erase();
  let led3 = gpiod.pd14.into_push_pull_output().erase();
  let led4 = gpiod.pd15.into_push_pull_output().erase();

  // 配置定时器6作为系统时基 (1kHz)
  let mut timer = Timer::tim6(dp.TIM6, &clocks);
  timer.start(1000.hz()); // 1ms tick
  timer.listen(Event::TimeOut);

  // 创建调度器
  let mut scheduler = CooperativeScheduler::new();

  // 创建任务上下文
  let mut task_contexts = [
    TaskContext::new(led1),
    TaskContext::new(led2),
    TaskContext::new(led3),
    TaskContext::new(led4),
  ];

  // 添加任务
  scheduler.add_task(Task::new(0, 500, task1)); // 500ms周期
  scheduler.add_task(Task::new(1, 1000, task2)); // 1s周期
  scheduler.add_task(Task::new(2, 2000, task3)); // 2s周期
  scheduler.add_task(Task::new(3, 100, task4)); // 100ms周期

  // 将对象移动到全局变量
  cortex_m::interrupt::free(|cs| {
    SCHEDULER.borrow(cs).replace(Some(scheduler));
    TIMER.borrow(cs).replace(Some(timer));
  });

  // 启用定时器中断
  unsafe {
    NVIC::unmask(stm32::Interrupt::TIM6_DAC);
  }

  // 主循环
  loop {
    // 运行调度器
    cortex_m::interrupt::free(|cs| {
      if let Some(ref mut scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
        scheduler.run(&mut task_contexts);
      }
    });

    // 等待下一个时钟周期
    cortex_m::asm::wfi();
  }
}

/// 任务上下文
pub struct TaskContext {
  led: LedPin,
  counter: u32,
}

impl TaskContext {
  pub fn new(led: LedPin) -> Self {
    Self { led, counter: 0 }
  }
}

/// 任务函数类型
type TaskFunction = fn(&mut TaskContext);

/// 任务结构体
#[derive(Clone, Copy)]
pub struct Task {
  id: usize,
  period_ms: u32,
  last_run: u32,
  function: TaskFunction,
  enabled: bool,
}

impl Task {
  /// 创建新任务
  pub fn new(id: usize, period_ms: u32, function: TaskFunction) -> Self {
    Self {
      id,
      period_ms,
      last_run: 0,
      function,
      enabled: true,
    }
  }

  /// 检查任务是否应该运行
  pub fn should_run(&self, current_time: u32) -> bool {
    self.enabled && (current_time.wrapping_sub(self.last_run) >= self.period_ms)
  }

  /// 运行任务
  pub fn run(&mut self, context: &mut TaskContext, current_time: u32) {
    if self.should_run(current_time) {
      (self.function)(context);
      self.last_run = current_time;
    }
  }

  /// 启用任务
  pub fn enable(&mut self) {
    self.enabled = true;
  }

  /// 禁用任务
  pub fn disable(&mut self) {
    self.enabled = false;
  }
}

/// 协作式调度器
pub struct CooperativeScheduler {
  tasks: Vec<Task, 16>,
  current_time: u32,
  tick_count: u32,
}

impl CooperativeScheduler {
  /// 创建新的调度器
  pub fn new() -> Self {
    Self {
      tasks: Vec::new(),
      current_time: 0,
      tick_count: 0,
    }
  }

  /// 添加任务
  pub fn add_task(&mut self, task: Task) -> Result<(), ()> {
    self.tasks.push(task).map_err(|_| ())
  }

  /// 移除任务
  pub fn remove_task(&mut self, task_id: usize) -> Result<(), ()> {
    if let Some(pos) = self.tasks.iter().position(|t| t.id == task_id) {
      self.tasks.remove(pos);
      Ok(())
    } else {
      Err(())
    }
  }

  /// 启用任务
  pub fn enable_task(&mut self, task_id: usize) -> Result<(), ()> {
    if let Some(task) = self.tasks.iter_mut().find(|t| t.id == task_id) {
      task.enable();
      Ok(())
    } else {
      Err(())
    }
  }

  /// 禁用任务
  pub fn disable_task(&mut self, task_id: usize) -> Result<(), ()> {
    if let Some(task) = self.tasks.iter_mut().find(|t| t.id == task_id) {
      task.disable();
      Ok(())
    } else {
      Err(())
    }
  }

  /// 系统时钟滴答
  pub fn tick(&mut self) {
    self.tick_count += 1;
    self.current_time += 1; // 1ms per tick
  }

  /// 运行调度器
  pub fn run(&mut self, contexts: &mut [TaskContext]) {
    for task in &mut self.tasks {
      if task.id < contexts.len() {
        task.run(&mut contexts[task.id], self.current_time);
      }
    }
  }

  /// 获取当前时间
  pub fn get_current_time(&self) -> u32 {
    self.current_time
  }

  /// 获取滴答计数
  pub fn get_tick_count(&self) -> u32 {
    self.tick_count
  }

  /// 获取任务数量
  pub fn get_task_count(&self) -> usize {
    self.tasks.len()
  }

  /// 获取活动任务数量
  pub fn get_active_task_count(&self) -> usize {
    self.tasks.iter().filter(|t| t.enabled).count()
  }
}

/// 任务统计信息
pub struct TaskStatistics {
  execution_count: u32,
  total_execution_time: u64,
  max_execution_time: u32,
  missed_deadlines: u32,
}

impl TaskStatistics {
  pub fn new() -> Self {
    Self {
      execution_count: 0,
      total_execution_time: 0,
      max_execution_time: 0,
      missed_deadlines: 0,
    }
  }

  pub fn record_execution(&mut self, execution_time: u32) {
    self.execution_count += 1;
    self.total_execution_time += execution_time as u64;

    if execution_time > self.max_execution_time {
      self.max_execution_time = execution_time;
    }
  }

  pub fn record_missed_deadline(&mut self) {
    self.missed_deadlines += 1;
  }

  pub fn get_average_execution_time(&self) -> u32 {
    if self.execution_count > 0 {
      (self.total_execution_time / self.execution_count as u64) as u32
    } else {
      0
    }
  }

  pub fn reset(&mut self) {
    self.execution_count = 0;
    self.total_execution_time = 0;
    self.max_execution_time = 0;
    self.missed_deadlines = 0;
  }
}

// 任务函数定义
fn task1(context: &mut TaskContext) {
  // LED1闪烁任务
  context.led.toggle();
  context.counter += 1;
}

fn task2(context: &mut TaskContext) {
  // LED2慢闪任务
  context.led.toggle();
  context.counter += 1;
}

fn task3(context: &mut TaskContext) {
  // LED3超慢闪任务
  context.led.toggle();
  context.counter += 1;
}

fn task4(context: &mut TaskContext) {
  // LED4快闪任务
  context.led.toggle();
  context.counter += 1;
}

/// 定时器中断处理程序
#[interrupt]
fn TIM6_DAC() {
  cortex_m::interrupt::free(|cs| {
    // 清除中断标志
    if let Some(ref mut timer) = TIMER.borrow(cs).borrow_mut().as_mut() {
      timer.clear_interrupt(Event::TimeOut);
    }

    // 调度器时钟滴答
    if let Some(ref mut scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
      scheduler.tick();
    }
  });
}

/// 延时任务调度器
pub struct DelayedTaskScheduler {
  scheduler: CooperativeScheduler,
  delayed_tasks: Vec<DelayedTask, 8>,
}

#[derive(Clone, Copy)]
pub struct DelayedTask {
  task: Task,
  delay_ms: u32,
  scheduled_time: u32,
}

impl DelayedTaskScheduler {
  pub fn new() -> Self {
    Self {
      scheduler: CooperativeScheduler::new(),
      delayed_tasks: Vec::new(),
    }
  }

  /// 添加延时任务
  pub fn schedule_delayed_task(&mut self, task: Task, delay_ms: u32) -> Result<(), ()> {
    let delayed_task = DelayedTask {
      task,
      delay_ms,
      scheduled_time: self.scheduler.get_current_time() + delay_ms,
    };

    self.delayed_tasks.push(delayed_task).map_err(|_| ())
  }

  /// 处理延时任务
  pub fn process_delayed_tasks(&mut self) {
    let current_time = self.scheduler.get_current_time();
    let mut i = 0;

    while i < self.delayed_tasks.len() {
      if current_time >= self.delayed_tasks[i].scheduled_time {
        let delayed_task = self.delayed_tasks.remove(i);
        let _ = self.scheduler.add_task(delayed_task.task);
      } else {
        i += 1;
      }
    }
  }

  /// 运行调度器
  pub fn run(&mut self, contexts: &mut [TaskContext]) {
    self.process_delayed_tasks();
    self.scheduler.run(contexts);
  }

  /// 时钟滴答
  pub fn tick(&mut self) {
    self.scheduler.tick();
  }
}
