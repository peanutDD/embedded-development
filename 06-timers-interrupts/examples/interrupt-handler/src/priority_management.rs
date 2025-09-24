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
use heapless::spsc::{Queue, Producer, Consumer};

// 全局变量
static TIMER2: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static TIMER3: Mutex<RefCell<Option<Timer<stm32::TIM3>>>> = Mutex::new(RefCell::new(None));
static TIMER4: Mutex<RefCell<Option<Timer<stm32::TIM4>>>> = Mutex::new(RefCell::new(None));
static PRIORITY_MANAGER: Mutex<RefCell<Option<InterruptPriorityManager>>> = Mutex::new(RefCell::new(None));
static LEDS: Mutex<RefCell<Option<[PB0<Output<PushPull>>; 4]>>> = Mutex::new(RefCell::new(None));

// 中断事件队列
static mut HIGH_PRIORITY_QUEUE: Queue<PriorityEvent, 8> = Queue::new();
static mut MEDIUM_PRIORITY_QUEUE: Queue<PriorityEvent, 16> = Queue::new();
static mut LOW_PRIORITY_QUEUE: Queue<PriorityEvent, 32> = Queue::new();

static mut HIGH_PRODUCER: Option<Producer<PriorityEvent, 8>> = None;
static mut MEDIUM_PRODUCER: Option<Producer<PriorityEvent, 16>> = None;
static mut LOW_PRODUCER: Option<Producer<PriorityEvent, 32>> = None;

static mut HIGH_CONSUMER: Option<Consumer<PriorityEvent, 8>> = None;
static mut MEDIUM_CONSUMER: Option<Consumer<PriorityEvent, 16>> = None;
static mut LOW_CONSUMER: Option<Consumer<PriorityEvent, 32>> = None;

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
    let led1 = gpiob.pb0.into_push_pull_output(); // 高优先级指示
    let led2 = gpiob.pb1.into_push_pull_output(); // 中优先级指示
    let led3 = gpiob.pb2.into_push_pull_output(); // 低优先级指示
    let led4 = gpiob.pb3.into_push_pull_output(); // 状态指示

    // 配置定时器
    let mut timer2 = Timer::tim2(dp.TIM2, &clocks); // 高优先级 - 1kHz
    let mut timer3 = Timer::tim3(dp.TIM3, &clocks); // 中优先级 - 100Hz
    let mut timer4 = Timer::tim4(dp.TIM4, &clocks); // 低优先级 - 10Hz
    
    timer2.start(1.khz());
    timer3.start(100.hz());
    timer4.start(10.hz());
    
    timer2.listen(Event::Update);
    timer3.listen(Event::Update);
    timer4.listen(Event::Update);

    // 初始化事件队列
    let (high_prod, high_cons) = unsafe { HIGH_PRIORITY_QUEUE.split() };
    let (med_prod, med_cons) = unsafe { MEDIUM_PRIORITY_QUEUE.split() };
    let (low_prod, low_cons) = unsafe { LOW_PRIORITY_QUEUE.split() };
    
    unsafe {
        HIGH_PRODUCER = Some(high_prod);
        MEDIUM_PRODUCER = Some(med_prod);
        LOW_PRODUCER = Some(low_prod);
        HIGH_CONSUMER = Some(high_cons);
        MEDIUM_CONSUMER = Some(med_cons);
        LOW_CONSUMER = Some(low_cons);
    }
    
    // 创建优先级管理器
    let priority_manager = InterruptPriorityManager::new();
    
    // 将对象移动到全局变量
    cortex_m::interrupt::free(|cs| {
        TIMER2.borrow(cs).replace(Some(timer2));
        TIMER3.borrow(cs).replace(Some(timer3));
        TIMER4.borrow(cs).replace(Some(timer4));
        PRIORITY_MANAGER.borrow(cs).replace(Some(priority_manager));
        // LEDS.borrow(cs).replace(Some([led1, led2, led3, led4])); // 需要修改类型
    });
    
    // 配置中断优先级
    unsafe {
        let mut nvic = cp.NVIC;
        
        // TIM2 - 最高优先级 (0)
        nvic.set_priority(stm32::Interrupt::TIM2, 0);
        NVIC::unmask(stm32::Interrupt::TIM2);
        
        // TIM3 - 中等优先级 (1)
        nvic.set_priority(stm32::Interrupt::TIM3, 1);
        NVIC::unmask(stm32::Interrupt::TIM3);
        
        // TIM4 - 低优先级 (2)
        nvic.set_priority(stm32::Interrupt::TIM4, 2);
        NVIC::unmask(stm32::Interrupt::TIM4);
    }
    
    // 配置系统定时器用于延时
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);
    let mut led1 = led1;
    let mut led2 = led2;
    let mut led3 = led3;
    let mut led4 = led4;

    // 主循环
    loop {
        // 处理高优先级事件
        if let Some(ref mut consumer) = unsafe { HIGH_CONSUMER.as_mut() } {
            while let Some(event) = consumer.dequeue() {
                led1.toggle();
                // 处理高优先级任务
                cortex_m::interrupt::free(|cs| {
                    if let Some(ref mut manager) = PRIORITY_MANAGER.borrow(cs).borrow_mut().as_mut() {
                        manager.handle_high_priority_event(event);
                    }
                });
            }
        }
        
        // 处理中等优先级事件
        if let Some(ref mut consumer) = unsafe { MEDIUM_CONSUMER.as_mut() } {
            while let Some(event) = consumer.dequeue() {
                led2.toggle();
                // 处理中等优先级任务
                cortex_m::interrupt::free(|cs| {
                    if let Some(ref mut manager) = PRIORITY_MANAGER.borrow(cs).borrow_mut().as_mut() {
                        manager.handle_medium_priority_event(event);
                    }
                });
            }
        }
        
        // 处理低优先级事件
        if let Some(ref mut consumer) = unsafe { LOW_CONSUMER.as_mut() } {
            while let Some(event) = consumer.dequeue() {
                led3.toggle();
                // 处理低优先级任务
                cortex_m::interrupt::free(|cs| {
                    if let Some(ref mut manager) = PRIORITY_MANAGER.borrow(cs).borrow_mut().as_mut() {
                        manager.handle_low_priority_event(event);
                    }
                });
            }
        }
        
        // 状态指示
        led4.toggle();
        delay.delay_ms(100u32);
        
        // 输出统计信息
        cortex_m::interrupt::free(|cs| {
            if let Some(ref manager) = PRIORITY_MANAGER.borrow(cs).borrow().as_ref() {
                let stats = manager.get_statistics();
                // 这里可以通过串口输出统计信息
            }
        });
    }
}

/// 中断优先级级别
#[derive(Clone, Copy, PartialEq)]
pub enum InterruptPriority {
    High = 0,
    Medium = 1,
    Low = 2,
    Background = 3,
}

/// 优先级事件
#[derive(Clone, Copy)]
pub struct PriorityEvent {
    pub priority: InterruptPriority,
    pub source_id: u8,
    pub timestamp: u32,
    pub data: u32,
}

impl PriorityEvent {
    pub fn new(priority: InterruptPriority, source_id: u8, timestamp: u32, data: u32) -> Self {
        Self {
            priority,
            source_id,
            timestamp,
            data,
        }
    }
}

/// 中断统计信息
#[derive(Clone, Copy)]
pub struct InterruptPriorityStatistics {
    pub high_priority_count: u32,
    pub medium_priority_count: u32,
    pub low_priority_count: u32,
    pub preemption_count: u32,
    pub max_nesting_level: u8,
    pub current_nesting_level: u8,
    pub total_interrupt_time: u32,
    pub max_interrupt_time: u32,
    pub priority_inversions: u32,
}

impl InterruptPriorityStatistics {
    pub fn new() -> Self {
        Self {
            high_priority_count: 0,
            medium_priority_count: 0,
            low_priority_count: 0,
            preemption_count: 0,
            max_nesting_level: 0,
            current_nesting_level: 0,
            total_interrupt_time: 0,
            max_interrupt_time: 0,
            priority_inversions: 0,
        }
    }
}

/// 中断优先级管理器
pub struct InterruptPriorityManager {
    statistics: InterruptPriorityStatistics,
    interrupt_stack: [InterruptPriority; 8],
    stack_pointer: usize,
    last_interrupt_time: u32,
    high_priority_handler: HighPriorityHandler,
    medium_priority_handler: MediumPriorityHandler,
    low_priority_handler: LowPriorityHandler,
}

impl InterruptPriorityManager {
    /// 创建新的中断优先级管理器
    pub fn new() -> Self {
        Self {
            statistics: InterruptPriorityStatistics::new(),
            interrupt_stack: [InterruptPriority::Background; 8],
            stack_pointer: 0,
            last_interrupt_time: 0,
            high_priority_handler: HighPriorityHandler::new(),
            medium_priority_handler: MediumPriorityHandler::new(),
            low_priority_handler: LowPriorityHandler::new(),
        }
    }
    
    /// 进入中断
    pub fn enter_interrupt(&mut self, priority: InterruptPriority, timestamp: u32) {
        // 检查是否发生抢占
        if self.stack_pointer > 0 {
            let current_priority = self.interrupt_stack[self.stack_pointer - 1];
            if priority as u8 < current_priority as u8 {
                self.statistics.preemption_count += 1;
            } else if priority as u8 > current_priority as u8 {
                self.statistics.priority_inversions += 1;
            }
        }
        
        // 压入中断栈
        if self.stack_pointer < 8 {
            self.interrupt_stack[self.stack_pointer] = priority;
            self.stack_pointer += 1;
            
            if self.stack_pointer > self.statistics.max_nesting_level as usize {
                self.statistics.max_nesting_level = self.stack_pointer as u8;
            }
        }
        
        self.statistics.current_nesting_level = self.stack_pointer as u8;
        self.last_interrupt_time = timestamp;
        
        // 更新优先级计数
        match priority {
            InterruptPriority::High => self.statistics.high_priority_count += 1,
            InterruptPriority::Medium => self.statistics.medium_priority_count += 1,
            InterruptPriority::Low => self.statistics.low_priority_count += 1,
            _ => {}
        }
    }
    
    /// 退出中断
    pub fn exit_interrupt(&mut self, timestamp: u32) {
        if self.stack_pointer > 0 {
            self.stack_pointer -= 1;
            self.statistics.current_nesting_level = self.stack_pointer as u8;
            
            // 计算中断执行时间
            let interrupt_time = timestamp.wrapping_sub(self.last_interrupt_time);
            self.statistics.total_interrupt_time += interrupt_time;
            
            if interrupt_time > self.statistics.max_interrupt_time {
                self.statistics.max_interrupt_time = interrupt_time;
            }
        }
    }
    
    /// 处理高优先级事件
    pub fn handle_high_priority_event(&mut self, event: PriorityEvent) {
        self.high_priority_handler.handle_event(event);
    }
    
    /// 处理中等优先级事件
    pub fn handle_medium_priority_event(&mut self, event: PriorityEvent) {
        self.medium_priority_handler.handle_event(event);
    }
    
    /// 处理低优先级事件
    pub fn handle_low_priority_event(&mut self, event: PriorityEvent) {
        self.low_priority_handler.handle_event(event);
    }
    
    /// 获取统计信息
    pub fn get_statistics(&self) -> InterruptPriorityStatistics {
        self.statistics
    }
    
    /// 重置统计信息
    pub fn reset_statistics(&mut self) {
        self.statistics = InterruptPriorityStatistics::new();
    }
    
    /// 获取当前中断嵌套级别
    pub fn get_nesting_level(&self) -> u8 {
        self.statistics.current_nesting_level
    }
    
    /// 检查是否可以抢占
    pub fn can_preempt(&self, new_priority: InterruptPriority) -> bool {
        if self.stack_pointer == 0 {
            return true;
        }
        
        let current_priority = self.interrupt_stack[self.stack_pointer - 1];
        (new_priority as u8) < (current_priority as u8)
    }
}

/// 高优先级处理器
pub struct HighPriorityHandler {
    event_count: u32,
    max_execution_time: u32,
    total_execution_time: u32,
}

impl HighPriorityHandler {
    pub fn new() -> Self {
        Self {
            event_count: 0,
            max_execution_time: 0,
            total_execution_time: 0,
        }
    }
    
    pub fn handle_event(&mut self, event: PriorityEvent) {
        let start_time = self.get_timestamp();
        
        // 高优先级任务处理 - 必须快速完成
        self.critical_task();
        
        let execution_time = self.get_timestamp().wrapping_sub(start_time);
        self.event_count += 1;
        self.total_execution_time += execution_time;
        
        if execution_time > self.max_execution_time {
            self.max_execution_time = execution_time;
        }
    }
    
    fn critical_task(&self) {
        // 关键任务处理 - 例如安全检查、紧急停止等
        // 保持处理时间最短
        for _ in 0..10 {
            cortex_m::asm::nop();
        }
    }
    
    fn get_timestamp(&self) -> u32 {
        // 获取高精度时间戳
        0 // 实际应用中应该使用系统定时器
    }
}

/// 中等优先级处理器
pub struct MediumPriorityHandler {
    event_count: u32,
    processing_queue: [u32; 8],
    queue_head: usize,
    queue_tail: usize,
}

impl MediumPriorityHandler {
    pub fn new() -> Self {
        Self {
            event_count: 0,
            processing_queue: [0; 8],
            queue_head: 0,
            queue_tail: 0,
        }
    }
    
    pub fn handle_event(&mut self, event: PriorityEvent) {
        self.event_count += 1;
        
        // 中等优先级任务处理 - 可以有一定的处理时间
        self.data_processing_task(event.data);
        
        // 将数据加入处理队列
        self.enqueue_data(event.data);
    }
    
    fn data_processing_task(&self, data: u32) {
        // 数据处理任务 - 例如传感器数据处理、通信协议处理等
        for _ in 0..100 {
            cortex_m::asm::nop();
        }
    }
    
    fn enqueue_data(&mut self, data: u32) {
        let next_tail = (self.queue_tail + 1) % 8;
        if next_tail != self.queue_head {
            self.processing_queue[self.queue_tail] = data;
            self.queue_tail = next_tail;
        }
    }
    
    pub fn process_queue(&mut self) -> Option<u32> {
        if self.queue_head != self.queue_tail {
            let data = self.processing_queue[self.queue_head];
            self.queue_head = (self.queue_head + 1) % 8;
            Some(data)
        } else {
            None
        }
    }
}

/// 低优先级处理器
pub struct LowPriorityHandler {
    event_count: u32,
    background_tasks: [BackgroundTask; 4],
    current_task: usize,
}

impl LowPriorityHandler {
    pub fn new() -> Self {
        Self {
            event_count: 0,
            background_tasks: [
                BackgroundTask::new(0),
                BackgroundTask::new(1),
                BackgroundTask::new(2),
                BackgroundTask::new(3),
            ],
            current_task: 0,
        }
    }
    
    pub fn handle_event(&mut self, event: PriorityEvent) {
        self.event_count += 1;
        
        // 低优先级任务处理 - 可以被高优先级中断抢占
        self.background_processing_task();
        
        // 轮转执行后台任务
        self.background_tasks[self.current_task].execute();
        self.current_task = (self.current_task + 1) % 4;
    }
    
    fn background_processing_task(&self) {
        // 后台处理任务 - 例如日志记录、统计计算、非关键数据处理等
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
    }
}

/// 后台任务
pub struct BackgroundTask {
    id: u8,
    execution_count: u32,
    last_execution_time: u32,
}

impl BackgroundTask {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            execution_count: 0,
            last_execution_time: 0,
        }
    }
    
    pub fn execute(&mut self) {
        self.execution_count += 1;
        self.last_execution_time = self.get_timestamp();
        
        // 执行具体的后台任务
        match self.id {
            0 => self.housekeeping_task(),
            1 => self.logging_task(),
            2 => self.statistics_task(),
            3 => self.maintenance_task(),
            _ => {}
        }
    }
    
    fn housekeeping_task(&self) {
        // 系统维护任务
    }
    
    fn logging_task(&self) {
        // 日志记录任务
    }
    
    fn statistics_task(&self) {
        // 统计计算任务
    }
    
    fn maintenance_task(&self) {
        // 维护任务
    }
    
    fn get_timestamp(&self) -> u32 {
        0 // 实际应用中应该使用系统定时器
    }
}

/// TIM2中断处理程序 (高优先级)
#[interrupt]
fn TIM2() {
    let timestamp = 0; // 获取时间戳
    
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut timer) = TIMER2.borrow(cs).borrow_mut().as_mut() {
            if timer.is_update_interrupt_pending() {
                timer.clear_update_interrupt();
                
                if let Some(ref mut manager) = PRIORITY_MANAGER.borrow(cs).borrow_mut().as_mut() {
                    manager.enter_interrupt(InterruptPriority::High, timestamp);
                    
                    // 发送高优先级事件
                    let event = PriorityEvent::new(InterruptPriority::High, 2, timestamp, 0);
                    unsafe {
                        if let Some(ref mut producer) = HIGH_PRODUCER.as_mut() {
                            producer.enqueue(event).ok();
                        }
                    }
                    
                    manager.exit_interrupt(timestamp);
                }
            }
        }
    });
}

/// TIM3中断处理程序 (中等优先级)
#[interrupt]
fn TIM3() {
    let timestamp = 0; // 获取时间戳
    
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut timer) = TIMER3.borrow(cs).borrow_mut().as_mut() {
            if timer.is_update_interrupt_pending() {
                timer.clear_update_interrupt();
                
                if let Some(ref mut manager) = PRIORITY_MANAGER.borrow(cs).borrow_mut().as_mut() {
                    manager.enter_interrupt(InterruptPriority::Medium, timestamp);
                    
                    // 发送中等优先级事件
                    let event = PriorityEvent::new(InterruptPriority::Medium, 3, timestamp, 0);
                    unsafe {
                        if let Some(ref mut producer) = MEDIUM_PRODUCER.as_mut() {
                            producer.enqueue(event).ok();
                        }
                    }
                    
                    manager.exit_interrupt(timestamp);
                }
            }
        }
    });
}

/// TIM4中断处理程序 (低优先级)
#[interrupt]
fn TIM4() {
    let timestamp = 0; // 获取时间戳
    
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut timer) = TIMER4.borrow(cs).borrow_mut().as_mut() {
            if timer.is_update_interrupt_pending() {
                timer.clear_update_interrupt();
                
                if let Some(ref mut manager) = PRIORITY_MANAGER.borrow(cs).borrow_mut().as_mut() {
                    manager.enter_interrupt(InterruptPriority::Low, timestamp);
                    
                    // 发送低优先级事件
                    let event = PriorityEvent::new(InterruptPriority::Low, 4, timestamp, 0);
                    unsafe {
                        if let Some(ref mut producer) = LOW_PRODUCER.as_mut() {
                            producer.enqueue(event).ok();
                        }
                    }
                    
                    manager.exit_interrupt(timestamp);
                }
            }
        }
    });
}