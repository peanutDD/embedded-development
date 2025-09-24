#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Event},
    gpio::{gpiob::*, Output, PushPull},
    interrupt,
};
use cortex_m::peripheral::NVIC;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use heapless::{Vec, binary_heap::{BinaryHeap, Max}};

type StatusLed = PB0<Output<PushPull>>;
type TaskLed1 = PB1<Output<PushPull>>;
type TaskLed2 = PB2<Output<PushPull>>;

// 全局变量
static SCHEDULER: Mutex<RefCell<Option<PreemptiveScheduler>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM6>>>> = Mutex::new(RefCell::new(None));
static STATUS_LED: Mutex<RefCell<Option<StatusLed>>> = Mutex::new(RefCell::new(None));
static TASK_LED1: Mutex<RefCell<Option<TaskLed1>>> = Mutex::new(RefCell::new(None));
static TASK_LED2: Mutex<RefCell<Option<TaskLed2>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 获取外设
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 配置GPIO
    let gpiob = dp.GPIOB.split();
    let status_led = gpiob.pb0.into_push_pull_output();
    let task_led1 = gpiob.pb1.into_push_pull_output();
    let task_led2 = gpiob.pb2.into_push_pull_output();

    // 配置定时器6作为系统时基 (1ms tick)
    let mut timer = Timer::tim6(dp.TIM6, &clocks);
    timer.start(1.khz());
    timer.listen(Event::Update);
    
    // 创建抢占式调度器
    let mut scheduler = PreemptiveScheduler::new();
    
    // 添加任务
    scheduler.add_task(Task::new(
        1,
        TaskPriority::High,
        100,  // 100ms周期
        high_priority_task,
        TaskState::Ready,
    ));
    
    scheduler.add_task(Task::new(
        2,
        TaskPriority::Medium,
        200,  // 200ms周期
        medium_priority_task,
        TaskState::Ready,
    ));
    
    scheduler.add_task(Task::new(
        3,
        TaskPriority::Low,
        500,  // 500ms周期
        low_priority_task,
        TaskState::Ready,
    ));
    
    scheduler.add_task(Task::new(
        4,
        TaskPriority::Idle,
        1000, // 1000ms周期
        idle_task,
        TaskState::Ready,
    ));
    
    // 将对象移动到全局变量
    cortex_m::interrupt::free(|cs| {
        SCHEDULER.borrow(cs).replace(Some(scheduler));
        TIMER.borrow(cs).replace(Some(timer));
        STATUS_LED.borrow(cs).replace(Some(status_led));
        TASK_LED1.borrow(cs).replace(Some(task_led1));
        TASK_LED2.borrow(cs).replace(Some(task_led2));
    });
    
    // 启用定时器中断
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM6_DAC);
    }
    
    // 主循环 - 运行调度器
    loop {
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
                scheduler.run_scheduler();
            }
        });
        
        // 让出CPU给中断
        cortex_m::asm::wfi();
    }
}

/// 任务优先级
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TaskPriority {
    Idle = 0,
    Low = 1,
    Medium = 2,
    High = 3,
    Critical = 4,
}

/// 任务状态
#[derive(Clone, Copy, PartialEq)]
pub enum TaskState {
    Ready,      // 就绪
    Running,    // 运行中
    Blocked,    // 阻塞
    Suspended,  // 挂起
    Terminated, // 终止
}

/// 任务控制块
#[derive(Clone)]
pub struct Task {
    pub id: u32,
    pub priority: TaskPriority,
    pub period_ms: u32,
    pub last_run_time: u32,
    pub next_run_time: u32,
    pub execution_time: u32,
    pub max_execution_time: u32,
    pub run_count: u32,
    pub state: TaskState,
    pub handler: fn(),
    pub stack_usage: u32,
    pub deadline_miss_count: u32,
}

impl Task {
    /// 创建新任务
    pub fn new(
        id: u32,
        priority: TaskPriority,
        period_ms: u32,
        handler: fn(),
        state: TaskState,
    ) -> Self {
        Self {
            id,
            priority,
            period_ms,
            last_run_time: 0,
            next_run_time: 0,
            execution_time: 0,
            max_execution_time: 0,
            run_count: 0,
            state,
            handler,
            stack_usage: 0,
            deadline_miss_count: 0,
        }
    }
    
    /// 检查任务是否就绪
    pub fn is_ready(&self, current_time: u32) -> bool {
        self.state == TaskState::Ready && current_time >= self.next_run_time
    }
    
    /// 执行任务
    pub fn execute(&mut self, current_time: u32) {
        let start_time = current_time;
        self.state = TaskState::Running;
        self.last_run_time = current_time;
        
        // 执行任务处理函数
        (self.handler)();
        
        // 更新统计信息
        let execution_time = get_current_time().wrapping_sub(start_time);
        self.execution_time = execution_time;
        if execution_time > self.max_execution_time {
            self.max_execution_time = execution_time;
        }
        
        self.run_count += 1;
        self.next_run_time = current_time + self.period_ms;
        self.state = TaskState::Ready;
        
        // 检查截止时间错过
        if execution_time > self.period_ms {
            self.deadline_miss_count += 1;
        }
    }
    
    /// 挂起任务
    pub fn suspend(&mut self) {
        self.state = TaskState::Suspended;
    }
    
    /// 恢复任务
    pub fn resume(&mut self) {
        if self.state == TaskState::Suspended {
            self.state = TaskState::Ready;
        }
    }
    
    /// 阻塞任务
    pub fn block(&mut self) {
        self.state = TaskState::Blocked;
    }
    
    /// 解除阻塞
    pub fn unblock(&mut self) {
        if self.state == TaskState::Blocked {
            self.state = TaskState::Ready;
        }
    }
}

/// 任务优先级包装器，用于二叉堆
#[derive(Clone)]
pub struct PriorityTask {
    pub task: Task,
}

impl PartialEq for PriorityTask {
    fn eq(&self, other: &Self) -> bool {
        self.task.priority == other.task.priority
    }
}

impl Eq for PriorityTask {}

impl PartialOrd for PriorityTask {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PriorityTask {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.task.priority.cmp(&other.task.priority)
    }
}

/// 抢占式调度器
pub struct PreemptiveScheduler {
    tasks: Vec<Task, 16>,
    ready_queue: BinaryHeap<PriorityTask, Max, 16>,
    current_task_id: Option<u32>,
    system_time: u32,
    context_switches: u32,
    total_cpu_time: u32,
    idle_time: u32,
    scheduler_overhead: u32,
}

impl PreemptiveScheduler {
    /// 创建新的抢占式调度器
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            ready_queue: BinaryHeap::new(),
            current_task_id: None,
            system_time: 0,
            context_switches: 0,
            total_cpu_time: 0,
            idle_time: 0,
            scheduler_overhead: 0,
        }
    }
    
    /// 添加任务
    pub fn add_task(&mut self, task: Task) -> Result<(), &'static str> {
        if self.tasks.len() >= 16 {
            return Err("Task queue full");
        }
        
        self.tasks.push(task).map_err(|_| "Failed to add task")?;
        Ok(())
    }
    
    /// 移除任务
    pub fn remove_task(&mut self, task_id: u32) -> Result<(), &'static str> {
        if let Some(pos) = self.tasks.iter().position(|t| t.id == task_id) {
            self.tasks.swap_remove(pos);
            Ok(())
        } else {
            Err("Task not found")
        }
    }
    
    /// 系统时钟节拍
    pub fn tick(&mut self) {
        self.system_time += 1;
        
        // 更新就绪队列
        self.update_ready_queue();
        
        // 检查抢占
        self.check_preemption();
    }
    
    /// 更新就绪队列
    fn update_ready_queue(&mut self) {
        // 清空就绪队列
        self.ready_queue.clear();
        
        // 将就绪任务添加到队列
        for task in &self.tasks {
            if task.is_ready(self.system_time) {
                let _ = self.ready_queue.push(PriorityTask { task: task.clone() });
            }
        }
    }
    
    /// 检查是否需要抢占
    fn check_preemption(&mut self) {
        if let Some(highest_priority_task) = self.ready_queue.peek() {
            let should_preempt = match self.current_task_id {
                Some(current_id) => {
                    if let Some(current_task) = self.tasks.iter().find(|t| t.id == current_id) {
                        highest_priority_task.task.priority > current_task.priority
                    } else {
                        true
                    }
                }
                None => true,
            };
            
            if should_preempt {
                self.preempt_current_task();
            }
        }
    }
    
    /// 抢占当前任务
    fn preempt_current_task(&mut self) {
        // 保存当前任务上下文
        if let Some(current_id) = self.current_task_id {
            if let Some(current_task) = self.tasks.iter_mut().find(|t| t.id == current_id) {
                if current_task.state == TaskState::Running {
                    current_task.state = TaskState::Ready;
                }
            }
        }
        
        // 切换到最高优先级任务
        if let Some(next_task) = self.ready_queue.pop() {
            self.current_task_id = Some(next_task.task.id);
            self.context_switches += 1;
            
            // 更新任务状态
            if let Some(task) = self.tasks.iter_mut().find(|t| t.id == next_task.task.id) {
                task.execute(self.system_time);
            }
        } else {
            self.current_task_id = None;
        }
    }
    
    /// 运行调度器
    pub fn run_scheduler(&mut self) {
        let start_time = get_current_time();
        
        // 如果没有当前任务，选择下一个任务
        if self.current_task_id.is_none() {
            self.schedule_next_task();
        }
        
        // 更新调度器开销
        let scheduler_time = get_current_time().wrapping_sub(start_time);
        self.scheduler_overhead += scheduler_time;
    }
    
    /// 调度下一个任务
    fn schedule_next_task(&mut self) {
        if let Some(next_task) = self.ready_queue.pop() {
            self.current_task_id = Some(next_task.task.id);
            
            // 执行任务
            if let Some(task) = self.tasks.iter_mut().find(|t| t.id == next_task.task.id) {
                task.execute(self.system_time);
            }
        } else {
            // 没有就绪任务，进入空闲状态
            self.idle_time += 1;
        }
    }
    
    /// 挂起任务
    pub fn suspend_task(&mut self, task_id: u32) -> Result<(), &'static str> {
        if let Some(task) = self.tasks.iter_mut().find(|t| t.id == task_id) {
            task.suspend();
            
            // 如果挂起的是当前任务，需要重新调度
            if self.current_task_id == Some(task_id) {
                self.current_task_id = None;
                self.schedule_next_task();
            }
            
            Ok(())
        } else {
            Err("Task not found")
        }
    }
    
    /// 恢复任务
    pub fn resume_task(&mut self, task_id: u32) -> Result<(), &'static str> {
        if let Some(task) = self.tasks.iter_mut().find(|t| t.id == task_id) {
            task.resume();
            Ok(())
        } else {
            Err("Task not found")
        }
    }
    
    /// 获取调度器统计信息
    pub fn get_statistics(&self) -> SchedulerStatistics {
        let total_tasks = self.tasks.len() as u32;
        let ready_tasks = self.tasks.iter().filter(|t| t.state == TaskState::Ready).count() as u32;
        let running_tasks = self.tasks.iter().filter(|t| t.state == TaskState::Running).count() as u32;
        let blocked_tasks = self.tasks.iter().filter(|t| t.state == TaskState::Blocked).count() as u32;
        let suspended_tasks = self.tasks.iter().filter(|t| t.state == TaskState::Suspended).count() as u32;
        
        let cpu_utilization = if self.system_time > 0 {
            ((self.total_cpu_time * 100) / self.system_time) as u8
        } else {
            0
        };
        
        SchedulerStatistics {
            system_time: self.system_time,
            context_switches: self.context_switches,
            total_tasks,
            ready_tasks,
            running_tasks,
            blocked_tasks,
            suspended_tasks,
            cpu_utilization,
            idle_time: self.idle_time,
            scheduler_overhead: self.scheduler_overhead,
        }
    }
    
    /// 获取任务统计信息
    pub fn get_task_statistics(&self, task_id: u32) -> Option<TaskStatistics> {
        if let Some(task) = self.tasks.iter().find(|t| t.id == task_id) {
            Some(TaskStatistics {
                task_id: task.id,
                priority: task.priority,
                state: task.state,
                run_count: task.run_count,
                execution_time: task.execution_time,
                max_execution_time: task.max_execution_time,
                deadline_miss_count: task.deadline_miss_count,
                cpu_usage_percent: if self.system_time > 0 {
                    ((task.execution_time * task.run_count * 100) / self.system_time) as u8
                } else {
                    0
                },
            })
        } else {
            None
        }
    }
}

/// 调度器统计信息
#[derive(Clone, Copy)]
pub struct SchedulerStatistics {
    pub system_time: u32,
    pub context_switches: u32,
    pub total_tasks: u32,
    pub ready_tasks: u32,
    pub running_tasks: u32,
    pub blocked_tasks: u32,
    pub suspended_tasks: u32,
    pub cpu_utilization: u8,
    pub idle_time: u32,
    pub scheduler_overhead: u32,
}

/// 任务统计信息
#[derive(Clone, Copy)]
pub struct TaskStatistics {
    pub task_id: u32,
    pub priority: TaskPriority,
    pub state: TaskState,
    pub run_count: u32,
    pub execution_time: u32,
    pub max_execution_time: u32,
    pub deadline_miss_count: u32,
    pub cpu_usage_percent: u8,
}

/// 任务处理函数
fn high_priority_task() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = TASK_LED1.borrow(cs).borrow_mut().as_mut() {
            led.toggle();
        }
    });
    
    // 模拟高优先级任务工作负载
    for _ in 0..1000 {
        cortex_m::asm::nop();
    }
}

fn medium_priority_task() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = TASK_LED2.borrow(cs).borrow_mut().as_mut() {
            led.toggle();
        }
    });
    
    // 模拟中等优先级任务工作负载
    for _ in 0..2000 {
        cortex_m::asm::nop();
    }
}

fn low_priority_task() {
    // 模拟低优先级任务工作负载
    for _ in 0..5000 {
        cortex_m::asm::nop();
    }
}

fn idle_task() {
    // 空闲任务 - 可以进行功耗管理
    cortex_m::asm::wfi(); // 等待中断
}

/// 获取当前时间 (简化实现)
fn get_current_time() -> u32 {
    // 这里应该返回系统时钟计数
    // 在实际实现中可以使用DWT计数器或系统定时器
    0
}

/// 定时器中断处理程序
#[interrupt]
fn TIM6_DAC() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut timer) = TIMER.borrow(cs).borrow_mut().as_mut() {
            if timer.is_update_interrupt_pending() {
                timer.clear_update_interrupt();
                
                // 调度器时钟节拍
                if let Some(ref mut scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
                    scheduler.tick();
                }
                
                // 切换状态LED
                if let Some(ref mut led) = STATUS_LED.borrow(cs).borrow_mut().as_mut() {
                    led.toggle();
                }
            }
        }
    });
}