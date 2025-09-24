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
use heapless::{Vec, FnvIndexMap};

type StatusLed = PB0<Output<PushPull>>;
type PriorityLed1 = PB1<Output<PushPull>>;
type PriorityLed2 = PB2<Output<PushPull>>;
type PriorityLed3 = PB3<Output<PushPull>>;

// 全局变量
static PRIORITY_SCHEDULER: Mutex<RefCell<Option<PriorityScheduler>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM6>>>> = Mutex::new(RefCell::new(None));
static STATUS_LED: Mutex<RefCell<Option<StatusLed>>> = Mutex::new(RefCell::new(None));
static PRIORITY_LED1: Mutex<RefCell<Option<PriorityLed1>>> = Mutex::new(RefCell::new(None));
static PRIORITY_LED2: Mutex<RefCell<Option<PriorityLed2>>> = Mutex::new(RefCell::new(None));
static PRIORITY_LED3: Mutex<RefCell<Option<PriorityLed3>>> = Mutex::new(RefCell::new(None));

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
    let priority_led1 = gpiob.pb1.into_push_pull_output();
    let priority_led2 = gpiob.pb2.into_push_pull_output();
    let priority_led3 = gpiob.pb3.into_push_pull_output();

    // 配置定时器6作为系统时基 (1ms tick)
    let mut timer = Timer::tim6(dp.TIM6, &clocks);
    timer.start(1.khz());
    timer.listen(Event::Update);
    
    // 创建优先级调度器
    let mut scheduler = PriorityScheduler::new();
    
    // 创建互斥锁
    let mutex1 = scheduler.create_mutex(1).unwrap();
    let mutex2 = scheduler.create_mutex(2).unwrap();
    
    // 添加不同优先级的任务
    scheduler.add_task(PriorityTask::new(
        1,
        Priority::Critical,
        50,   // 50ms周期
        critical_priority_task,
        TaskState::Ready,
    )).unwrap();
    
    scheduler.add_task(PriorityTask::new(
        2,
        Priority::High,
        100,  // 100ms周期
        high_priority_task,
        TaskState::Ready,
    )).unwrap();
    
    scheduler.add_task(PriorityTask::new(
        3,
        Priority::Medium,
        200,  // 200ms周期
        medium_priority_task,
        TaskState::Ready,
    )).unwrap();
    
    scheduler.add_task(PriorityTask::new(
        4,
        Priority::Low,
        500,  // 500ms周期
        low_priority_task,
        TaskState::Ready,
    )).unwrap();
    
    // 添加使用互斥锁的任务
    scheduler.add_task(PriorityTask::new(
        5,
        Priority::High,
        300,  // 300ms周期
        mutex_user_task1,
        TaskState::Ready,
    )).unwrap();
    
    scheduler.add_task(PriorityTask::new(
        6,
        Priority::Medium,
        400,  // 400ms周期
        mutex_user_task2,
        TaskState::Ready,
    )).unwrap();
    
    // 将对象移动到全局变量
    cortex_m::interrupt::free(|cs| {
        PRIORITY_SCHEDULER.borrow(cs).replace(Some(scheduler));
        TIMER.borrow(cs).replace(Some(timer));
        STATUS_LED.borrow(cs).replace(Some(status_led));
        PRIORITY_LED1.borrow(cs).replace(Some(priority_led1));
        PRIORITY_LED2.borrow(cs).replace(Some(priority_led2));
        PRIORITY_LED3.borrow(cs).replace(Some(priority_led3));
    });
    
    // 启用定时器中断
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM6_DAC);
    }
    
    // 主循环
    loop {
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut scheduler) = PRIORITY_SCHEDULER.borrow(cs).borrow_mut().as_mut() {
                scheduler.schedule();
            }
        });
        
        // 让出CPU
        cortex_m::asm::wfi();
    }
}

/// 任务优先级
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum Priority {
    Idle = 0,
    Low = 1,
    Medium = 2,
    High = 3,
    Critical = 4,
    Interrupt = 5,
}

/// 任务状态
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum TaskState {
    Ready,
    Running,
    Blocked,
    Suspended,
    WaitingForMutex,
    Terminated,
}

/// 优先级任务
#[derive(Clone)]
pub struct PriorityTask {
    pub id: u32,
    pub priority: Priority,
    pub original_priority: Priority,  // 用于优先级继承
    pub period_ms: u32,
    pub last_run_time: u32,
    pub next_run_time: u32,
    pub execution_time: u32,
    pub max_execution_time: u32,
    pub run_count: u32,
    pub state: TaskState,
    pub handler: fn(),
    pub blocked_on_mutex: Option<u32>,
    pub owned_mutexes: Vec<u32, 4>,
    pub priority_inheritance_count: u32,
    pub deadline_miss_count: u32,
}

impl PriorityTask {
    /// 创建新的优先级任务
    pub fn new(
        id: u32,
        priority: Priority,
        period_ms: u32,
        handler: fn(),
        state: TaskState,
    ) -> Self {
        Self {
            id,
            priority,
            original_priority: priority,
            period_ms,
            last_run_time: 0,
            next_run_time: 0,
            execution_time: 0,
            max_execution_time: 0,
            run_count: 0,
            state,
            handler,
            blocked_on_mutex: None,
            owned_mutexes: Vec::new(),
            priority_inheritance_count: 0,
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
    
    /// 提升优先级 (优先级继承)
    pub fn inherit_priority(&mut self, new_priority: Priority) {
        if new_priority > self.priority {
            self.priority = new_priority;
            self.priority_inheritance_count += 1;
        }
    }
    
    /// 恢复原始优先级
    pub fn restore_original_priority(&mut self) {
        self.priority = self.original_priority;
    }
    
    /// 添加拥有的互斥锁
    pub fn add_owned_mutex(&mut self, mutex_id: u32) -> Result<(), &'static str> {
        self.owned_mutexes.push(mutex_id).map_err(|_| "Too many owned mutexes")
    }
    
    /// 移除拥有的互斥锁
    pub fn remove_owned_mutex(&mut self, mutex_id: u32) {
        if let Some(pos) = self.owned_mutexes.iter().position(|&id| id == mutex_id) {
            self.owned_mutexes.swap_remove(pos);
        }
    }
}

/// 互斥锁
#[derive(Clone)]
pub struct PriorityMutex {
    pub id: u32,
    pub owner: Option<u32>,
    pub waiting_tasks: Vec<u32, 8>,
    pub priority_ceiling: Priority,
    pub lock_count: u32,
    pub max_wait_time: u32,
    pub total_wait_time: u32,
}

impl PriorityMutex {
    /// 创建新的互斥锁
    pub fn new(id: u32, priority_ceiling: Priority) -> Self {
        Self {
            id,
            owner: None,
            waiting_tasks: Vec::new(),
            priority_ceiling,
            lock_count: 0,
            max_wait_time: 0,
            total_wait_time: 0,
        }
    }
    
    /// 尝试获取锁
    pub fn try_lock(&mut self, task_id: u32) -> bool {
        if self.owner.is_none() {
            self.owner = Some(task_id);
            self.lock_count += 1;
            true
        } else {
            false
        }
    }
    
    /// 释放锁
    pub fn unlock(&mut self, task_id: u32) -> Option<u32> {
        if self.owner == Some(task_id) {
            self.owner = None;
            // 返回下一个等待的任务
            if !self.waiting_tasks.is_empty() {
                Some(self.waiting_tasks.swap_remove(0))
            } else {
                None
            }
        } else {
            None
        }
    }
    
    /// 添加等待任务
    pub fn add_waiting_task(&mut self, task_id: u32) -> Result<(), &'static str> {
        self.waiting_tasks.push(task_id).map_err(|_| "Too many waiting tasks")
    }
    
    /// 移除等待任务
    pub fn remove_waiting_task(&mut self, task_id: u32) {
        if let Some(pos) = self.waiting_tasks.iter().position(|&id| id == task_id) {
            self.waiting_tasks.swap_remove(pos);
        }
    }
}

/// 优先级调度器
pub struct PriorityScheduler {
    tasks: Vec<PriorityTask, 16>,
    mutexes: FnvIndexMap<u32, PriorityMutex, 8>,
    current_task_id: Option<u32>,
    system_time: u32,
    context_switches: u32,
    priority_inversions: u32,
    priority_inheritances: u32,
    mutex_contentions: u32,
}

impl PriorityScheduler {
    /// 创建新的优先级调度器
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            mutexes: FnvIndexMap::new(),
            current_task_id: None,
            system_time: 0,
            context_switches: 0,
            priority_inversions: 0,
            priority_inheritances: 0,
            mutex_contentions: 0,
        }
    }
    
    /// 添加任务
    pub fn add_task(&mut self, task: PriorityTask) -> Result<(), &'static str> {
        if self.tasks.len() >= 16 {
            return Err("Task queue full");
        }
        
        self.tasks.push(task).map_err(|_| "Failed to add task")?;
        Ok(())
    }
    
    /// 创建互斥锁
    pub fn create_mutex(&mut self, mutex_id: u32) -> Result<u32, &'static str> {
        if self.mutexes.len() >= 8 {
            return Err("Too many mutexes");
        }
        
        let mutex = PriorityMutex::new(mutex_id, Priority::Critical);
        self.mutexes.insert(mutex_id, mutex).map_err(|_| "Failed to create mutex")?;
        Ok(mutex_id)
    }
    
    /// 系统时钟节拍
    pub fn tick(&mut self) {
        self.system_time += 1;
        
        // 检查任务截止时间
        self.check_deadlines();
        
        // 检查优先级反转
        self.detect_priority_inversion();
    }
    
    /// 调度任务
    pub fn schedule(&mut self) {
        // 找到最高优先级的就绪任务
        let mut highest_priority = Priority::Idle;
        let mut next_task_id = None;
        
        for task in &self.tasks {
            if task.is_ready(self.system_time) && task.priority > highest_priority {
                highest_priority = task.priority;
                next_task_id = Some(task.id);
            }
        }
        
        // 如果找到更高优先级的任务，进行上下文切换
        if let Some(task_id) = next_task_id {
            if self.current_task_id != Some(task_id) {
                self.context_switch(task_id);
            }
        }
        
        // 执行当前任务
        if let Some(current_id) = self.current_task_id {
            if let Some(task) = self.tasks.iter_mut().find(|t| t.id == current_id) {
                if task.is_ready(self.system_time) {
                    task.execute(self.system_time);
                }
            }
        }
    }
    
    /// 上下文切换
    fn context_switch(&mut self, new_task_id: u32) {
        // 保存当前任务状态
        if let Some(current_id) = self.current_task_id {
            if let Some(current_task) = self.tasks.iter_mut().find(|t| t.id == current_id) {
                if current_task.state == TaskState::Running {
                    current_task.state = TaskState::Ready;
                }
            }
        }
        
        // 切换到新任务
        self.current_task_id = Some(new_task_id);
        self.context_switches += 1;
        
        // 设置新任务状态
        if let Some(new_task) = self.tasks.iter_mut().find(|t| t.id == new_task_id) {
            new_task.state = TaskState::Running;
        }
    }
    
    /// 获取互斥锁
    pub fn lock_mutex(&mut self, task_id: u32, mutex_id: u32) -> Result<(), &'static str> {
        if let Some(mutex) = self.mutexes.get_mut(&mutex_id) {
            if mutex.try_lock(task_id) {
                // 成功获取锁
                if let Some(task) = self.tasks.iter_mut().find(|t| t.id == task_id) {
                    task.add_owned_mutex(mutex_id)?;
                }
                Ok(())
            } else {
                // 锁被占用，需要等待
                self.mutex_contentions += 1;
                mutex.add_waiting_task(task_id)?;
                
                // 阻塞当前任务
                if let Some(task) = self.tasks.iter_mut().find(|t| t.id == task_id) {
                    task.state = TaskState::WaitingForMutex;
                    task.blocked_on_mutex = Some(mutex_id);
                }
                
                // 实施优先级继承
                self.apply_priority_inheritance(task_id, mutex_id);
                
                Err("Mutex is locked")
            }
        } else {
            Err("Mutex not found")
        }
    }
    
    /// 释放互斥锁
    pub fn unlock_mutex(&mut self, task_id: u32, mutex_id: u32) -> Result<(), &'static str> {
        if let Some(mutex) = self.mutexes.get_mut(&mutex_id) {
            if let Some(next_task_id) = mutex.unlock(task_id) {
                // 唤醒下一个等待的任务
                if let Some(next_task) = self.tasks.iter_mut().find(|t| t.id == next_task_id) {
                    next_task.state = TaskState::Ready;
                    next_task.blocked_on_mutex = None;
                    next_task.add_owned_mutex(mutex_id)?;
                }
                
                // 获取锁
                mutex.owner = Some(next_task_id);
            }
            
            // 移除任务拥有的锁
            if let Some(task) = self.tasks.iter_mut().find(|t| t.id == task_id) {
                task.remove_owned_mutex(mutex_id);
                
                // 如果没有其他锁，恢复原始优先级
                if task.owned_mutexes.is_empty() {
                    task.restore_original_priority();
                }
            }
            
            Ok(())
        } else {
            Err("Mutex not found")
        }
    }
    
    /// 实施优先级继承
    fn apply_priority_inheritance(&mut self, waiting_task_id: u32, mutex_id: u32) {
        if let Some(mutex) = self.mutexes.get(&mutex_id) {
            if let Some(owner_id) = mutex.owner {
                // 获取等待任务的优先级
                let waiting_priority = self.tasks.iter()
                    .find(|t| t.id == waiting_task_id)
                    .map(|t| t.priority)
                    .unwrap_or(Priority::Idle);
                
                // 提升拥有者的优先级
                if let Some(owner_task) = self.tasks.iter_mut().find(|t| t.id == owner_id) {
                    if waiting_priority > owner_task.priority {
                        owner_task.inherit_priority(waiting_priority);
                        self.priority_inheritances += 1;
                    }
                }
            }
        }
    }
    
    /// 检查任务截止时间
    fn check_deadlines(&mut self) {
        for task in &mut self.tasks {
            if task.state == TaskState::Running {
                let elapsed = self.system_time.wrapping_sub(task.last_run_time);
                if elapsed > task.period_ms {
                    task.deadline_miss_count += 1;
                }
            }
        }
    }
    
    /// 检测优先级反转
    fn detect_priority_inversion(&mut self) {
        // 检查是否有高优先级任务被低优先级任务阻塞
        for task in &self.tasks {
            if task.state == TaskState::WaitingForMutex {
                if let Some(mutex_id) = task.blocked_on_mutex {
                    if let Some(mutex) = self.mutexes.get(&mutex_id) {
                        if let Some(owner_id) = mutex.owner {
                            if let Some(owner_task) = self.tasks.iter().find(|t| t.id == owner_id) {
                                if task.original_priority > owner_task.original_priority {
                                    self.priority_inversions += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    /// 获取调度器统计信息
    pub fn get_statistics(&self) -> PrioritySchedulerStatistics {
        let total_tasks = self.tasks.len() as u32;
        let ready_tasks = self.tasks.iter().filter(|t| t.state == TaskState::Ready).count() as u32;
        let running_tasks = self.tasks.iter().filter(|t| t.state == TaskState::Running).count() as u32;
        let blocked_tasks = self.tasks.iter().filter(|t| t.state == TaskState::Blocked || t.state == TaskState::WaitingForMutex).count() as u32;
        
        PrioritySchedulerStatistics {
            system_time: self.system_time,
            context_switches: self.context_switches,
            total_tasks,
            ready_tasks,
            running_tasks,
            blocked_tasks,
            priority_inversions: self.priority_inversions,
            priority_inheritances: self.priority_inheritances,
            mutex_contentions: self.mutex_contentions,
            total_mutexes: self.mutexes.len() as u32,
        }
    }
}

/// 优先级调度器统计信息
#[derive(Clone, Copy)]
pub struct PrioritySchedulerStatistics {
    pub system_time: u32,
    pub context_switches: u32,
    pub total_tasks: u32,
    pub ready_tasks: u32,
    pub running_tasks: u32,
    pub blocked_tasks: u32,
    pub priority_inversions: u32,
    pub priority_inheritances: u32,
    pub mutex_contentions: u32,
    pub total_mutexes: u32,
}

/// 任务处理函数
fn critical_priority_task() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = PRIORITY_LED1.borrow(cs).borrow_mut().as_mut() {
            led.set_high();
        }
    });
    
    // 模拟关键任务工作负载
    for _ in 0..500 {
        cortex_m::asm::nop();
    }
    
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = PRIORITY_LED1.borrow(cs).borrow_mut().as_mut() {
            led.set_low();
        }
    });
}

fn high_priority_task() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = PRIORITY_LED2.borrow(cs).borrow_mut().as_mut() {
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
        if let Some(ref mut led) = PRIORITY_LED3.borrow(cs).borrow_mut().as_mut() {
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

fn mutex_user_task1() {
    // 使用互斥锁的任务1
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut scheduler) = PRIORITY_SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            // 尝试获取互斥锁1
            let _ = scheduler.lock_mutex(5, 1);
            
            // 临界区工作
            for _ in 0..1000 {
                cortex_m::asm::nop();
            }
            
            // 释放互斥锁1
            let _ = scheduler.unlock_mutex(5, 1);
        }
    });
}

fn mutex_user_task2() {
    // 使用互斥锁的任务2
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut scheduler) = PRIORITY_SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            // 尝试获取互斥锁1
            let _ = scheduler.lock_mutex(6, 1);
            
            // 临界区工作
            for _ in 0..2000 {
                cortex_m::asm::nop();
            }
            
            // 释放互斥锁1
            let _ = scheduler.unlock_mutex(6, 1);
        }
    });
}

/// 获取当前时间 (简化实现)
fn get_current_time() -> u32 {
    // 这里应该返回系统时钟计数
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
                if let Some(ref mut scheduler) = PRIORITY_SCHEDULER.borrow(cs).borrow_mut().as_mut() {
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