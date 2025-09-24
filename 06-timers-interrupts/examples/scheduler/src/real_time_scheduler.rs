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
use heapless::{Vec, binary_heap::{BinaryHeap, Min}};

type StatusLed = PB0<Output<PushPull>>;
type HardRtLed = PB1<Output<PushPull>>;
type SoftRtLed = PB2<Output<PushPull>>;
type DeadlineLed = PB3<Output<PushPull>>;

// 全局变量
static RT_SCHEDULER: Mutex<RefCell<Option<RealTimeScheduler>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM6>>>> = Mutex::new(RefCell::new(None));
static STATUS_LED: Mutex<RefCell<Option<StatusLed>>> = Mutex::new(RefCell::new(None));
static HARD_RT_LED: Mutex<RefCell<Option<HardRtLed>>> = Mutex::new(RefCell::new(None));
static SOFT_RT_LED: Mutex<RefCell<Option<SoftRtLed>>> = Mutex::new(RefCell::new(None));
static DEADLINE_LED: Mutex<RefCell<Option<DeadlineLed>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 获取外设
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.mhz()).freeze(); // 使用最高频率以获得最佳实时性能

    // 配置GPIO
    let gpiob = dp.GPIOB.split();
    let status_led = gpiob.pb0.into_push_pull_output();
    let hard_rt_led = gpiob.pb1.into_push_pull_output();
    let soft_rt_led = gpiob.pb2.into_push_pull_output();
    let deadline_led = gpiob.pb3.into_push_pull_output();

    // 配置定时器6作为系统时基 (100μs tick，高精度)
    let mut timer = Timer::tim6(dp.TIM6, &clocks);
    timer.start(10.khz()); // 100μs精度
    timer.listen(Event::Update);
    
    // 创建实时调度器
    let mut scheduler = RealTimeScheduler::new();
    
    // 添加硬实时任务
    scheduler.add_task(RealTimeTask::new(
        1,
        TaskType::HardRealTime,
        Priority::Critical,
        1000,  // 1ms周期
        800,   // 0.8ms截止时间
        200,   // 0.2ms最坏执行时间
        hard_real_time_task,
    )).unwrap();
    
    scheduler.add_task(RealTimeTask::new(
        2,
        TaskType::HardRealTime,
        Priority::High,
        2000,  // 2ms周期
        1800,  // 1.8ms截止时间
        500,   // 0.5ms最坏执行时间
        control_loop_task,
    )).unwrap();
    
    // 添加软实时任务
    scheduler.add_task(RealTimeTask::new(
        3,
        TaskType::SoftRealTime,
        Priority::Medium,
        5000,  // 5ms周期
        8000,  // 8ms截止时间
        1000,  // 1ms最坏执行时间
        soft_real_time_task,
    )).unwrap();
    
    scheduler.add_task(RealTimeTask::new(
        4,
        TaskType::SoftRealTime,
        Priority::Low,
        10000, // 10ms周期
        15000, // 15ms截止时间
        2000,  // 2ms最坏执行时间
        data_processing_task,
    )).unwrap();
    
    // 添加非实时任务
    scheduler.add_task(RealTimeTask::new(
        5,
        TaskType::NonRealTime,
        Priority::Idle,
        50000, // 50ms周期
        100000, // 100ms截止时间
        5000,  // 5ms最坏执行时间
        background_task,
    )).unwrap();
    
    // 将对象移动到全局变量
    cortex_m::interrupt::free(|cs| {
        RT_SCHEDULER.borrow(cs).replace(Some(scheduler));
        TIMER.borrow(cs).replace(Some(timer));
        STATUS_LED.borrow(cs).replace(Some(status_led));
        HARD_RT_LED.borrow(cs).replace(Some(hard_rt_led));
        SOFT_RT_LED.borrow(cs).replace(Some(soft_rt_led));
        DEADLINE_LED.borrow(cs).replace(Some(deadline_led));
    });
    
    // 启用定时器中断
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM6_DAC);
    }
    
    // 主循环
    loop {
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut scheduler) = RT_SCHEDULER.borrow(cs).borrow_mut().as_mut() {
                scheduler.schedule();
            }
        });
        
        // 让出CPU
        cortex_m::asm::wfi();
    }
}

/// 任务类型
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum TaskType {
    HardRealTime,  // 硬实时任务
    SoftRealTime,  // 软实时任务
    NonRealTime,   // 非实时任务
}

/// 任务优先级
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum Priority {
    Idle = 0,
    Low = 1,
    Medium = 2,
    High = 3,
    Critical = 4,
}

/// 任务状态
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum TaskState {
    Ready,
    Running,
    Blocked,
    Suspended,
    DeadlineMissed,
    Terminated,
}

/// 实时任务
#[derive(Clone)]
pub struct RealTimeTask {
    pub id: u32,
    pub task_type: TaskType,
    pub priority: Priority,
    pub period_us: u32,        // 周期 (微秒)
    pub deadline_us: u32,      // 相对截止时间 (微秒)
    pub wcet_us: u32,          // 最坏执行时间 (微秒)
    pub release_time: u32,     // 释放时间
    pub absolute_deadline: u32, // 绝对截止时间
    pub remaining_time: u32,   // 剩余执行时间
    pub execution_time: u32,   // 实际执行时间
    pub max_execution_time: u32,
    pub state: TaskState,
    pub handler: fn(),
    pub run_count: u32,
    pub deadline_miss_count: u32,
    pub response_time: u32,    // 响应时间
    pub max_response_time: u32,
    pub jitter: u32,           // 抖动
    pub utilization: f32,      // CPU利用率
}

impl RealTimeTask {
    /// 创建新的实时任务
    pub fn new(
        id: u32,
        task_type: TaskType,
        priority: Priority,
        period_us: u32,
        deadline_us: u32,
        wcet_us: u32,
        handler: fn(),
    ) -> Self {
        Self {
            id,
            task_type,
            priority,
            period_us,
            deadline_us,
            wcet_us,
            release_time: 0,
            absolute_deadline: deadline_us,
            remaining_time: wcet_us,
            execution_time: 0,
            max_execution_time: 0,
            state: TaskState::Ready,
            handler,
            run_count: 0,
            deadline_miss_count: 0,
            response_time: 0,
            max_response_time: 0,
            jitter: 0,
            utilization: (wcet_us as f32) / (period_us as f32),
        }
    }
    
    /// 检查任务是否就绪
    pub fn is_ready(&self, current_time: u32) -> bool {
        self.state == TaskState::Ready && current_time >= self.release_time
    }
    
    /// 检查是否错过截止时间
    pub fn is_deadline_missed(&self, current_time: u32) -> bool {
        current_time > self.absolute_deadline
    }
    
    /// 执行任务
    pub fn execute(&mut self, current_time: u32) -> TaskExecutionResult {
        let start_time = current_time;
        self.state = TaskState::Running;
        
        // 检查截止时间
        if self.is_deadline_missed(current_time) {
            self.deadline_miss_count += 1;
            self.state = TaskState::DeadlineMissed;
            return TaskExecutionResult::DeadlineMissed;
        }
        
        // 执行任务处理函数
        (self.handler)();
        
        // 更新统计信息
        let execution_time = get_current_time().wrapping_sub(start_time);
        self.execution_time = execution_time;
        if execution_time > self.max_execution_time {
            self.max_execution_time = execution_time;
        }
        
        // 计算响应时间
        self.response_time = current_time.wrapping_sub(self.release_time);
        if self.response_time > self.max_response_time {
            self.max_response_time = self.response_time;
        }
        
        self.run_count += 1;
        self.remaining_time = 0;
        
        // 设置下一个周期
        self.release_time = current_time + self.period_us;
        self.absolute_deadline = self.release_time + self.deadline_us;
        self.remaining_time = self.wcet_us;
        self.state = TaskState::Ready;
        
        // 检查执行时间是否超过WCET
        if execution_time > self.wcet_us {
            TaskExecutionResult::WcetExceeded
        } else {
            TaskExecutionResult::Success
        }
    }
    
    /// 抢占任务
    pub fn preempt(&mut self, current_time: u32) {
        if self.state == TaskState::Running {
            let elapsed = current_time.wrapping_sub(self.release_time);
            self.remaining_time = self.remaining_time.saturating_sub(elapsed);
            self.state = TaskState::Ready;
        }
    }
    
    /// 计算抖动
    pub fn calculate_jitter(&mut self, expected_release_time: u32, actual_release_time: u32) {
        let jitter = if actual_release_time > expected_release_time {
            actual_release_time - expected_release_time
        } else {
            expected_release_time - actual_release_time
        };
        
        if jitter > self.jitter {
            self.jitter = jitter;
        }
    }
}

/// 任务执行结果
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum TaskExecutionResult {
    Success,
    DeadlineMissed,
    WcetExceeded,
    Preempted,
}

/// 截止时间任务包装器，用于EDF调度
#[derive(Clone)]
pub struct DeadlineTask {
    pub task: RealTimeTask,
}

impl PartialEq for DeadlineTask {
    fn eq(&self, other: &Self) -> bool {
        self.task.absolute_deadline == other.task.absolute_deadline
    }
}

impl Eq for DeadlineTask {}

impl PartialOrd for DeadlineTask {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for DeadlineTask {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        // 最小堆：截止时间越早，优先级越高
        self.task.absolute_deadline.cmp(&other.task.absolute_deadline)
    }
}

/// 实时调度器
pub struct RealTimeScheduler {
    tasks: Vec<RealTimeTask, 16>,
    edf_queue: BinaryHeap<DeadlineTask, Min, 16>, // EDF调度队列
    current_task_id: Option<u32>,
    system_time: u32,
    context_switches: u32,
    total_deadline_misses: u32,
    hard_rt_deadline_misses: u32,
    soft_rt_deadline_misses: u32,
    total_utilization: f32,
    schedulability_test_passed: bool,
    hyperperiod: u32,
}

impl RealTimeScheduler {
    /// 创建新的实时调度器
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            edf_queue: BinaryHeap::new(),
            current_task_id: None,
            system_time: 0,
            context_switches: 0,
            total_deadline_misses: 0,
            hard_rt_deadline_misses: 0,
            soft_rt_deadline_misses: 0,
            total_utilization: 0.0,
            schedulability_test_passed: false,
            hyperperiod: 0,
        }
    }
    
    /// 添加任务
    pub fn add_task(&mut self, task: RealTimeTask) -> Result<(), &'static str> {
        if self.tasks.len() >= 16 {
            return Err("Task queue full");
        }
        
        // 更新总利用率
        self.total_utilization += task.utilization;
        
        self.tasks.push(task).map_err(|_| "Failed to add task")?;
        
        // 执行可调度性测试
        self.schedulability_test();
        
        Ok(())
    }
    
    /// 可调度性测试
    fn schedulability_test(&mut self) {
        // Liu & Layland可调度性测试
        // 对于EDF：U ≤ 1
        // 对于RM：U ≤ n(2^(1/n) - 1)
        
        let n = self.tasks.len() as f32;
        let rm_bound = n * (2.0_f32.powf(1.0 / n) - 1.0);
        
        // EDF可调度性测试
        let edf_schedulable = self.total_utilization <= 1.0;
        
        // RM可调度性测试
        let rm_schedulable = self.total_utilization <= rm_bound;
        
        self.schedulability_test_passed = edf_schedulable;
        
        // 计算超周期
        self.calculate_hyperperiod();
    }
    
    /// 计算超周期 (所有任务周期的最小公倍数)
    fn calculate_hyperperiod(&mut self) {
        if self.tasks.is_empty() {
            self.hyperperiod = 0;
            return;
        }
        
        let mut lcm = self.tasks[0].period_us;
        for task in &self.tasks[1..] {
            lcm = self.lcm(lcm, task.period_us);
        }
        self.hyperperiod = lcm;
    }
    
    /// 计算最小公倍数
    fn lcm(&self, a: u32, b: u32) -> u32 {
        (a * b) / self.gcd(a, b)
    }
    
    /// 计算最大公约数
    fn gcd(&self, mut a: u32, mut b: u32) -> u32 {
        while b != 0 {
            let temp = b;
            b = a % b;
            a = temp;
        }
        a
    }
    
    /// 系统时钟节拍
    pub fn tick(&mut self) {
        self.system_time += 100; // 100μs增量
        
        // 更新EDF队列
        self.update_edf_queue();
        
        // 检查截止时间
        self.check_deadlines();
    }
    
    /// 更新EDF队列
    fn update_edf_queue(&mut self) {
        // 清空队列
        self.edf_queue.clear();
        
        // 添加就绪任务到EDF队列
        for task in &self.tasks {
            if task.is_ready(self.system_time) {
                let _ = self.edf_queue.push(DeadlineTask { task: task.clone() });
            }
        }
    }
    
    /// 检查截止时间
    fn check_deadlines(&mut self) {
        for task in &mut self.tasks {
            if task.is_deadline_missed(self.system_time) && task.state != TaskState::DeadlineMissed {
                self.total_deadline_misses += 1;
                
                match task.task_type {
                    TaskType::HardRealTime => {
                        self.hard_rt_deadline_misses += 1;
                        // 硬实时任务错过截止时间是严重错误
                        self.handle_hard_deadline_miss(task.id);
                    }
                    TaskType::SoftRealTime => {
                        self.soft_rt_deadline_misses += 1;
                        // 软实时任务错过截止时间可以容忍
                    }
                    TaskType::NonRealTime => {
                        // 非实时任务不关心截止时间
                    }
                }
                
                task.state = TaskState::DeadlineMissed;
            }
        }
    }
    
    /// 处理硬实时任务截止时间错过
    fn handle_hard_deadline_miss(&mut self, task_id: u32) {
        // 点亮截止时间错过LED
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut led) = DEADLINE_LED.borrow(cs).borrow_mut().as_mut() {
                led.set_high();
            }
        });
        
        // 可以实施其他错误处理策略：
        // 1. 系统重启
        // 2. 降级模式
        // 3. 任务重新调度
        // 4. 错误日志记录
    }
    
    /// EDF调度
    pub fn schedule(&mut self) {
        // 获取截止时间最早的任务
        if let Some(earliest_deadline_task) = self.edf_queue.peek() {
            let next_task_id = earliest_deadline_task.task.id;
            
            // 检查是否需要抢占
            let should_preempt = match self.current_task_id {
                Some(current_id) => {
                    if let Some(current_task) = self.tasks.iter().find(|t| t.id == current_id) {
                        earliest_deadline_task.task.absolute_deadline < current_task.absolute_deadline
                    } else {
                        true
                    }
                }
                None => true,
            };
            
            if should_preempt && self.current_task_id != Some(next_task_id) {
                self.context_switch(next_task_id);
            }
        }
        
        // 执行当前任务
        if let Some(current_id) = self.current_task_id {
            if let Some(task) = self.tasks.iter_mut().find(|t| t.id == current_id) {
                if task.is_ready(self.system_time) {
                    let result = task.execute(self.system_time);
                    
                    match result {
                        TaskExecutionResult::DeadlineMissed => {
                            // 处理截止时间错过
                        }
                        TaskExecutionResult::WcetExceeded => {
                            // 处理WCET超出
                        }
                        _ => {}
                    }
                }
            }
        }
    }
    
    /// 上下文切换
    fn context_switch(&mut self, new_task_id: u32) {
        // 抢占当前任务
        if let Some(current_id) = self.current_task_id {
            if let Some(current_task) = self.tasks.iter_mut().find(|t| t.id == current_id) {
                current_task.preempt(self.system_time);
            }
        }
        
        // 切换到新任务
        self.current_task_id = Some(new_task_id);
        self.context_switches += 1;
    }
    
    /// 获取调度器统计信息
    pub fn get_statistics(&self) -> RealTimeSchedulerStatistics {
        let hard_rt_tasks = self.tasks.iter().filter(|t| t.task_type == TaskType::HardRealTime).count() as u32;
        let soft_rt_tasks = self.tasks.iter().filter(|t| t.task_type == TaskType::SoftRealTime).count() as u32;
        let non_rt_tasks = self.tasks.iter().filter(|t| t.task_type == TaskType::NonRealTime).count() as u32;
        
        RealTimeSchedulerStatistics {
            system_time: self.system_time,
            context_switches: self.context_switches,
            total_tasks: self.tasks.len() as u32,
            hard_rt_tasks,
            soft_rt_tasks,
            non_rt_tasks,
            total_deadline_misses: self.total_deadline_misses,
            hard_rt_deadline_misses: self.hard_rt_deadline_misses,
            soft_rt_deadline_misses: self.soft_rt_deadline_misses,
            total_utilization: self.total_utilization,
            schedulability_test_passed: self.schedulability_test_passed,
            hyperperiod: self.hyperperiod,
        }
    }
}

/// 实时调度器统计信息
#[derive(Clone, Copy)]
pub struct RealTimeSchedulerStatistics {
    pub system_time: u32,
    pub context_switches: u32,
    pub total_tasks: u32,
    pub hard_rt_tasks: u32,
    pub soft_rt_tasks: u32,
    pub non_rt_tasks: u32,
    pub total_deadline_misses: u32,
    pub hard_rt_deadline_misses: u32,
    pub soft_rt_deadline_misses: u32,
    pub total_utilization: f32,
    pub schedulability_test_passed: bool,
    pub hyperperiod: u32,
}

/// 任务处理函数
fn hard_real_time_task() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = HARD_RT_LED.borrow(cs).borrow_mut().as_mut() {
            led.set_high();
        }
    });
    
    // 模拟硬实时任务 - 必须在截止时间内完成
    for _ in 0..200 {
        cortex_m::asm::nop();
    }
    
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = HARD_RT_LED.borrow(cs).borrow_mut().as_mut() {
            led.set_low();
        }
    });
}

fn control_loop_task() {
    // 模拟控制回路任务
    for _ in 0..500 {
        cortex_m::asm::nop();
    }
}

fn soft_real_time_task() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = SOFT_RT_LED.borrow(cs).borrow_mut().as_mut() {
            led.toggle();
        }
    });
    
    // 模拟软实时任务 - 可以容忍偶尔错过截止时间
    for _ in 0..1000 {
        cortex_m::asm::nop();
    }
}

fn data_processing_task() {
    // 模拟数据处理任务
    for _ in 0..2000 {
        cortex_m::asm::nop();
    }
}

fn background_task() {
    // 模拟后台任务
    for _ in 0..5000 {
        cortex_m::asm::nop();
    }
}

/// 获取当前时间 (简化实现)
fn get_current_time() -> u32 {
    // 这里应该返回高精度时间戳
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
                if let Some(ref mut scheduler) = RT_SCHEDULER.borrow(cs).borrow_mut().as_mut() {
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