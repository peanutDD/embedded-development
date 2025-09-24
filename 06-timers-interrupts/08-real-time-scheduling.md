# 实时调度系统

## 概述

实时调度系统是嵌入式系统的核心组件，负责管理多个任务的执行时序，确保关键任务能够在规定的时间内完成。基于定时器的实时调度系统通过精确的时间控制，实现任务的周期性执行、优先级管理、资源分配等功能。本章将详细介绍如何使用STM32定时器构建高效的实时调度系统。

## 实时系统基础

### 实时系统分类

1. **硬实时系统（Hard Real-time）**
   - 任务必须在截止时间内完成
   - 超时会导致系统失效
   - 例：飞行控制、医疗设备

2. **软实时系统（Soft Real-time）**
   - 任务最好在截止时间内完成
   - 偶尔超时可以容忍
   - 例：多媒体播放、用户界面

3. **固实时系统（Firm Real-time）**
   - 任务必须周期性执行
   - 超时的结果会被丢弃
   - 例：视频编码、数据采集

### 调度算法

常见的实时调度算法包括：
- **速率单调调度（RMS）**：周期越短，优先级越高
- **最早截止时间优先（EDF）**：截止时间越早，优先级越高
- **固定优先级抢占式调度**：静态优先级，支持抢占
- **时间片轮转调度**：每个任务分配固定时间片

## 基于定时器的调度器

### 系统时基配置

```rust
use stm32f4xx_hal::pac::{RCC, TIM2, NVIC};
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

// 系统时基：1ms
const SYSTEM_TICK_MS: u32 = 1;
const TIMER_FREQUENCY: u32 = 84_000_000; // 84MHz

// 全局系统时间
static SYSTEM_TIME: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

fn configure_system_timebase() {
    let rcc = unsafe { &*RCC::ptr() };
    let tim2 = unsafe { &*TIM2::ptr() };
    
    // 使能TIM2时钟
    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
    
    // 配置1ms时基
    // 84MHz / 84 = 1MHz, 1MHz / 1000 = 1kHz (1ms)
    tim2.psc.write(|w| unsafe { w.bits(83) });  // 预分频器
    tim2.arr.write(|w| unsafe { w.bits(999) }); // 自动重载值
    
    // 使能更新中断
    tim2.dier.modify(|_, w| w.uie().set_bit());
    
    // 配置中断优先级（最高优先级）
    unsafe {
        let mut nvic = NVIC::steal();
        nvic.set_priority(stm32f4xx_hal::pac::Interrupt::TIM2, 0);
        nvic.enable(stm32f4xx_hal::pac::Interrupt::TIM2);
    }
    
    // 启动定时器
    tim2.cr1.modify(|_, w| w.cen().set_bit());
}

// 系统时基中断
#[interrupt]
fn TIM2() {
    let tim2 = unsafe { &*TIM2::ptr() };
    
    if tim2.sr.read().uif().bit_is_set() {
        tim2.sr.modify(|_, w| w.uif().clear_bit());
        
        // 更新系统时间
        cortex_m::interrupt::free(|cs| {
            let mut time = SYSTEM_TIME.borrow(cs).borrow_mut();
            *time = time.wrapping_add(SYSTEM_TICK_MS);
        });
        
        // 调用调度器
        scheduler_tick();
    }
}

// 获取系统时间
fn get_system_time() -> u32 {
    cortex_m::interrupt::free(|cs| {
        *SYSTEM_TIME.borrow(cs).borrow()
    })
}
```

### 任务控制块（TCB）

```rust
use heapless::Vec;

// 任务状态
#[derive(Clone, Copy, PartialEq)]
enum TaskState {
    Ready,      // 就绪
    Running,    // 运行
    Blocked,    // 阻塞
    Suspended,  // 挂起
}

// 任务优先级
type Priority = u8;
const MAX_PRIORITY: Priority = 255;
const MIN_PRIORITY: Priority = 0;

// 任务控制块
#[derive(Clone)]
struct TaskControlBlock {
    id: u8,
    name: &'static str,
    priority: Priority,
    state: TaskState,
    
    // 时间相关
    period_ms: u32,         // 任务周期
    deadline_ms: u32,       // 相对截止时间
    next_run_time: u32,     // 下次运行时间
    last_run_time: u32,     // 上次运行时间
    execution_time_us: u32, // 执行时间
    
    // 统计信息
    run_count: u32,         // 运行次数
    miss_count: u32,        // 错过截止时间次数
    max_execution_time: u32,// 最大执行时间
    
    // 任务函数
    task_function: fn(),
}

impl TaskControlBlock {
    fn new(
        id: u8,
        name: &'static str,
        priority: Priority,
        period_ms: u32,
        deadline_ms: u32,
        task_function: fn(),
    ) -> Self {
        Self {
            id,
            name,
            priority,
            state: TaskState::Ready,
            period_ms,
            deadline_ms,
            next_run_time: 0,
            last_run_time: 0,
            execution_time_us: 0,
            run_count: 0,
            miss_count: 0,
            max_execution_time: 0,
            task_function,
        }
    }
    
    fn is_ready_to_run(&self, current_time: u32) -> bool {
        self.state == TaskState::Ready && current_time >= self.next_run_time
    }
    
    fn update_next_run_time(&mut self, current_time: u32) {
        self.next_run_time = current_time + self.period_ms;
    }
    
    fn check_deadline_miss(&mut self, current_time: u32) -> bool {
        let deadline = self.last_run_time + self.deadline_ms;
        if current_time > deadline && self.state == TaskState::Ready {
            self.miss_count += 1;
            true
        } else {
            false
        }
    }
}
```

### 调度器实现

```rust
// 调度器
struct RealTimeScheduler {
    tasks: Vec<TaskControlBlock, 16>,  // 最多16个任务
    current_task: Option<u8>,          // 当前运行任务ID
    scheduling_algorithm: SchedulingAlgorithm,
    scheduler_enabled: bool,
}

#[derive(Clone, Copy)]
enum SchedulingAlgorithm {
    FixedPriority,  // 固定优先级
    RateMonotonic,  // 速率单调
    EarliestDeadlineFirst, // 最早截止时间优先
}

static mut SCHEDULER: Option<RealTimeScheduler> = None;

impl RealTimeScheduler {
    fn new(algorithm: SchedulingAlgorithm) -> Self {
        Self {
            tasks: Vec::new(),
            current_task: None,
            scheduling_algorithm: algorithm,
            scheduler_enabled: false,
        }
    }
    
    fn add_task(&mut self, task: TaskControlBlock) -> Result<(), &'static str> {
        if self.tasks.len() >= 16 {
            return Err("Task list full");
        }
        
        // 检查任务ID唯一性
        if self.tasks.iter().any(|t| t.id == task.id) {
            return Err("Task ID already exists");
        }
        
        self.tasks.push(task).map_err(|_| "Failed to add task")?;
        
        // 根据调度算法排序
        self.sort_tasks();
        
        Ok(())
    }
    
    fn sort_tasks(&mut self) {
        match self.scheduling_algorithm {
            SchedulingAlgorithm::FixedPriority => {
                // 按优先级排序（高优先级在前）
                self.tasks.sort_by(|a, b| b.priority.cmp(&a.priority));
            },
            SchedulingAlgorithm::RateMonotonic => {
                // 按周期排序（短周期在前）
                self.tasks.sort_by(|a, b| a.period_ms.cmp(&b.period_ms));
            },
            SchedulingAlgorithm::EarliestDeadlineFirst => {
                // 运行时动态排序
            }
        }
    }
    
    fn schedule(&mut self) {
        if !self.scheduler_enabled {
            return;
        }
        
        let current_time = get_system_time();
        
        // 检查截止时间错过
        self.check_deadline_misses(current_time);
        
        // 选择下一个要运行的任务
        let next_task_id = self.select_next_task(current_time);
        
        // 执行任务切换
        if let Some(task_id) = next_task_id {
            self.execute_task(task_id, current_time);
        }
    }
    
    fn select_next_task(&mut self, current_time: u32) -> Option<u8> {
        match self.scheduling_algorithm {
            SchedulingAlgorithm::FixedPriority | SchedulingAlgorithm::RateMonotonic => {
                // 选择优先级最高的就绪任务
                self.tasks.iter()
                    .find(|task| task.is_ready_to_run(current_time))
                    .map(|task| task.id)
            },
            SchedulingAlgorithm::EarliestDeadlineFirst => {
                // 选择截止时间最早的就绪任务
                let mut earliest_task: Option<&TaskControlBlock> = None;
                let mut earliest_deadline = u32::MAX;
                
                for task in &self.tasks {
                    if task.is_ready_to_run(current_time) {
                        let deadline = task.last_run_time + task.deadline_ms;
                        if deadline < earliest_deadline {
                            earliest_deadline = deadline;
                            earliest_task = Some(task);
                        }
                    }
                }
                
                earliest_task.map(|task| task.id)
            }
        }
    }
    
    fn execute_task(&mut self, task_id: u8, current_time: u32) {
        if let Some(task) = self.tasks.iter_mut().find(|t| t.id == task_id) {
            // 记录开始时间
            let start_time = get_microsecond_timestamp();
            task.last_run_time = current_time;
            task.state = TaskState::Running;
            self.current_task = Some(task_id);
            
            // 执行任务函数
            (task.task_function)();
            
            // 记录执行时间
            let end_time = get_microsecond_timestamp();
            let execution_time = end_time - start_time;
            task.execution_time_us = execution_time;
            task.max_execution_time = task.max_execution_time.max(execution_time);
            task.run_count += 1;
            
            // 更新任务状态
            task.state = TaskState::Ready;
            task.update_next_run_time(current_time);
            self.current_task = None;
        }
    }
    
    fn check_deadline_misses(&mut self, current_time: u32) {
        for task in &mut self.tasks {
            if task.check_deadline_miss(current_time) {
                // 记录截止时间错过事件
                log_deadline_miss(task.id, task.name);
            }
        }
    }
    
    fn start(&mut self) {
        self.scheduler_enabled = true;
        
        // 初始化所有任务的下次运行时间
        let current_time = get_system_time();
        for task in &mut self.tasks {
            task.next_run_time = current_time + task.period_ms;
        }
    }
    
    fn stop(&mut self) {
        self.scheduler_enabled = false;
    }
    
    fn get_task_statistics(&self, task_id: u8) -> Option<TaskStatistics> {
        self.tasks.iter()
            .find(|t| t.id == task_id)
            .map(|task| TaskStatistics {
                id: task.id,
                name: task.name,
                run_count: task.run_count,
                miss_count: task.miss_count,
                max_execution_time: task.max_execution_time,
                average_execution_time: if task.run_count > 0 {
                    task.execution_time_us / task.run_count
                } else {
                    0
                },
                cpu_utilization: self.calculate_cpu_utilization(task),
            })
    }
    
    fn calculate_cpu_utilization(&self, task: &TaskControlBlock) -> f32 {
        if task.period_ms == 0 {
            return 0.0;
        }
        
        let period_us = task.period_ms * 1000;
        task.execution_time_us as f32 / period_us as f32 * 100.0
    }
}

#[derive(Debug)]
struct TaskStatistics {
    id: u8,
    name: &'static str,
    run_count: u32,
    miss_count: u32,
    max_execution_time: u32,
    average_execution_time: u32,
    cpu_utilization: f32,
}

// 调度器时钟节拍
fn scheduler_tick() {
    unsafe {
        if let Some(ref mut scheduler) = SCHEDULER {
            scheduler.schedule();
        }
    }
}
```

### 任务同步机制

```rust
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

// 信号量
struct Semaphore {
    count: AtomicU32,
    max_count: u32,
}

impl Semaphore {
    fn new(initial_count: u32, max_count: u32) -> Self {
        Self {
            count: AtomicU32::new(initial_count),
            max_count,
        }
    }
    
    fn acquire(&self) -> bool {
        loop {
            let current = self.count.load(Ordering::Acquire);
            if current == 0 {
                return false; // 无法获取
            }
            
            if self.count.compare_exchange_weak(
                current,
                current - 1,
                Ordering::Release,
                Ordering::Relaxed
            ).is_ok() {
                return true;
            }
        }
    }
    
    fn release(&self) -> bool {
        loop {
            let current = self.count.load(Ordering::Acquire);
            if current >= self.max_count {
                return false; // 已达最大值
            }
            
            if self.count.compare_exchange_weak(
                current,
                current + 1,
                Ordering::Release,
                Ordering::Relaxed
            ).is_ok() {
                return true;
            }
        }
    }
}

// 互斥锁
struct Mutex {
    locked: AtomicBool,
    owner_task: AtomicU32,
}

impl Mutex {
    fn new() -> Self {
        Self {
            locked: AtomicBool::new(false),
            owner_task: AtomicU32::new(0),
        }
    }
    
    fn lock(&self, task_id: u32) -> bool {
        if self.locked.compare_exchange(
            false,
            true,
            Ordering::Acquire,
            Ordering::Relaxed
        ).is_ok() {
            self.owner_task.store(task_id, Ordering::Release);
            true
        } else {
            false
        }
    }
    
    fn unlock(&self, task_id: u32) -> bool {
        if self.owner_task.load(Ordering::Acquire) == task_id {
            self.owner_task.store(0, Ordering::Release);
            self.locked.store(false, Ordering::Release);
            true
        } else {
            false
        }
    }
}

// 事件标志组
struct EventFlags {
    flags: AtomicU32,
}

impl EventFlags {
    fn new() -> Self {
        Self {
            flags: AtomicU32::new(0),
        }
    }
    
    fn set(&self, mask: u32) {
        self.flags.fetch_or(mask, Ordering::Release);
    }
    
    fn clear(&self, mask: u32) {
        self.flags.fetch_and(!mask, Ordering::Release);
    }
    
    fn wait_any(&self, mask: u32) -> bool {
        (self.flags.load(Ordering::Acquire) & mask) != 0
    }
    
    fn wait_all(&self, mask: u32) -> bool {
        (self.flags.load(Ordering::Acquire) & mask) == mask
    }
}
```

## 应用实例

### 1. 数据采集系统

```rust
// 数据采集任务
static ADC_SEMAPHORE: Semaphore = Semaphore::new(1, 1);
static DATA_BUFFER: Mutex<RefCell<[f32; 1000]>> = Mutex::new(RefCell::new([0.0; 1000]));
static mut BUFFER_INDEX: usize = 0;

fn data_acquisition_task() {
    // 获取ADC信号量
    if !ADC_SEMAPHORE.acquire() {
        return; // ADC忙，跳过本次采集
    }
    
    // 执行ADC转换
    let adc_value = read_adc_channel(0);
    let voltage = adc_value as f32 * 3.3 / 4095.0;
    
    // 存储数据
    cortex_m::interrupt::free(|cs| {
        if let Ok(mut buffer) = DATA_BUFFER.try_borrow_mut() {
            unsafe {
                buffer[BUFFER_INDEX] = voltage;
                BUFFER_INDEX = (BUFFER_INDEX + 1) % buffer.len();
            }
        }
    });
    
    // 释放ADC信号量
    ADC_SEMAPHORE.release();
}

fn data_processing_task() {
    static mut LAST_PROCESS_INDEX: usize = 0;
    
    cortex_m::interrupt::free(|cs| {
        if let Ok(buffer) = DATA_BUFFER.try_borrow() {
            unsafe {
                let current_index = BUFFER_INDEX;
                
                // 处理新数据
                while LAST_PROCESS_INDEX != current_index {
                    let value = buffer[LAST_PROCESS_INDEX];
                    
                    // 执行数据处理（滤波、分析等）
                    process_data_sample(value);
                    
                    LAST_PROCESS_INDEX = (LAST_PROCESS_INDEX + 1) % buffer.len();
                }
            }
        }
    });
}

fn communication_task() {
    // 发送处理后的数据
    send_processed_data();
}

// 配置数据采集系统
fn setup_data_acquisition_system() {
    unsafe {
        SCHEDULER = Some(RealTimeScheduler::new(SchedulingAlgorithm::RateMonotonic));
        
        if let Some(ref mut scheduler) = SCHEDULER {
            // 添加数据采集任务（1kHz，高优先级）
            let acquisition_task = TaskControlBlock::new(
                1,
                "DataAcquisition",
                200,
                1,    // 1ms周期
                1,    // 1ms截止时间
                data_acquisition_task,
            );
            scheduler.add_task(acquisition_task).unwrap();
            
            // 添加数据处理任务（100Hz，中优先级）
            let processing_task = TaskControlBlock::new(
                2,
                "DataProcessing",
                150,
                10,   // 10ms周期
                8,    // 8ms截止时间
                data_processing_task,
            );
            scheduler.add_task(processing_task).unwrap();
            
            // 添加通信任务（10Hz，低优先级）
            let comm_task = TaskControlBlock::new(
                3,
                "Communication",
                100,
                100,  // 100ms周期
                80,   // 80ms截止时间
                communication_task,
            );
            scheduler.add_task(comm_task).unwrap();
            
            scheduler.start();
        }
    }
}
```

### 2. 电机控制系统

```rust
// 电机控制相关全局变量
static MOTOR_CONTROL_FLAGS: EventFlags = EventFlags::new();
static POSITION_MUTEX: Mutex = Mutex::new();

const FLAG_POSITION_UPDATE: u32 = 0x01;
const FLAG_VELOCITY_UPDATE: u32 = 0x02;
const FLAG_CURRENT_UPDATE: u32 = 0x04;

// 位置控制任务（最高优先级，1kHz）
fn position_control_task() {
    static mut TARGET_POSITION: f32 = 0.0;
    static mut CURRENT_POSITION: f32 = 0.0;
    static mut POSITION_ERROR: f32 = 0.0;
    
    // 读取编码器位置
    unsafe {
        CURRENT_POSITION = read_encoder_position();
        POSITION_ERROR = TARGET_POSITION - CURRENT_POSITION;
    }
    
    // 位置PID控制
    let velocity_command = position_pid_controller(unsafe { POSITION_ERROR });
    
    // 设置速度指令
    set_velocity_command(velocity_command);
    
    // 设置位置更新标志
    MOTOR_CONTROL_FLAGS.set(FLAG_POSITION_UPDATE);
}

// 速度控制任务（高优先级，2kHz）
fn velocity_control_task() {
    // 等待位置更新标志
    if !MOTOR_CONTROL_FLAGS.wait_any(FLAG_POSITION_UPDATE) {
        return;
    }
    
    let velocity_command = get_velocity_command();
    let current_velocity = read_current_velocity();
    let velocity_error = velocity_command - current_velocity;
    
    // 速度PID控制
    let current_command = velocity_pid_controller(velocity_error);
    
    // 设置电流指令
    set_current_command(current_command);
    
    // 设置速度更新标志
    MOTOR_CONTROL_FLAGS.set(FLAG_VELOCITY_UPDATE);
    MOTOR_CONTROL_FLAGS.clear(FLAG_POSITION_UPDATE);
}

// 电流控制任务（最高优先级，10kHz）
fn current_control_task() {
    // 等待速度更新标志
    if !MOTOR_CONTROL_FLAGS.wait_any(FLAG_VELOCITY_UPDATE) {
        return;
    }
    
    let current_command = get_current_command();
    let actual_current = read_motor_current();
    let current_error = current_command - actual_current;
    
    // 电流PID控制
    let pwm_duty = current_pid_controller(current_error);
    
    // 更新PWM输出
    update_motor_pwm(pwm_duty);
    
    // 设置电流更新标志
    MOTOR_CONTROL_FLAGS.set(FLAG_CURRENT_UPDATE);
    MOTOR_CONTROL_FLAGS.clear(FLAG_VELOCITY_UPDATE);
}

// 监控任务（低优先级，10Hz）
fn motor_monitoring_task() {
    // 检查系统状态
    check_motor_temperature();
    check_power_supply();
    check_encoder_status();
    
    // 发送状态信息
    send_motor_status();
}

// 配置电机控制系统
fn setup_motor_control_system() {
    unsafe {
        SCHEDULER = Some(RealTimeScheduler::new(SchedulingAlgorithm::FixedPriority));
        
        if let Some(ref mut scheduler) = SCHEDULER {
            // 电流控制任务（最高优先级，10kHz）
            let current_task = TaskControlBlock::new(
                1,
                "CurrentControl",
                255,
                0,    // 100μs周期 (10kHz)
                0,    // 80μs截止时间
                current_control_task,
            );
            scheduler.add_task(current_task).unwrap();
            
            // 速度控制任务（高优先级，2kHz）
            let velocity_task = TaskControlBlock::new(
                2,
                "VelocityControl",
                200,
                0,    // 500μs周期 (2kHz)
                0,    // 400μs截止时间
                velocity_control_task,
            );
            scheduler.add_task(velocity_task).unwrap();
            
            // 位置控制任务（中优先级，1kHz）
            let position_task = TaskControlBlock::new(
                3,
                "PositionControl",
                150,
                1,    // 1ms周期 (1kHz)
                0,    // 800μs截止时间
                position_control_task,
            );
            scheduler.add_task(position_task).unwrap();
            
            // 监控任务（低优先级，10Hz）
            let monitor_task = TaskControlBlock::new(
                4,
                "MotorMonitoring",
                50,
                100,  // 100ms周期 (10Hz)
                80,   // 80ms截止时间
                motor_monitoring_task,
            );
            scheduler.add_task(monitor_task).unwrap();
            
            scheduler.start();
        }
    }
}
```

### 3. 通信协议栈

```rust
// 通信协议栈任务
static PROTOCOL_SEMAPHORE: Semaphore = Semaphore::new(1, 1);
static RX_BUFFER: Mutex<RefCell<[u8; 1024]>> = Mutex::new(RefCell::new([0; 1024]));
static TX_BUFFER: Mutex<RefCell<[u8; 1024]>> = Mutex::new(RefCell::new([0; 1024]));

// 物理层接收任务（高优先级，由中断触发）
fn physical_layer_rx_task() {
    if !PROTOCOL_SEMAPHORE.acquire() {
        return;
    }
    
    // 从UART接收数据
    let received_data = uart_receive_data();
    
    // 存储到接收缓冲区
    cortex_m::interrupt::free(|cs| {
        if let Ok(mut buffer) = RX_BUFFER.try_borrow_mut() {
            // 处理接收数据
            process_received_data(&mut buffer, &received_data);
        }
    });
    
    PROTOCOL_SEMAPHORE.release();
}

// 数据链路层处理任务（中优先级）
fn data_link_layer_task() {
    cortex_m::interrupt::free(|cs| {
        if let Ok(mut rx_buffer) = RX_BUFFER.try_borrow_mut() {
            // 帧同步和错误检测
            if let Some(frame) = extract_valid_frame(&mut rx_buffer) {
                // 处理有效帧
                process_data_frame(&frame);
            }
        }
    });
}

// 应用层处理任务（低优先级）
fn application_layer_task() {
    // 处理应用层协议
    process_application_protocol();
    
    // 准备发送数据
    prepare_transmission_data();
}

// 物理层发送任务（中优先级）
fn physical_layer_tx_task() {
    cortex_m::interrupt::free(|cs| {
        if let Ok(buffer) = TX_BUFFER.try_borrow() {
            // 发送数据
            uart_transmit_data(&buffer);
        }
    });
}

// 配置通信协议栈
fn setup_communication_stack() {
    unsafe {
        SCHEDULER = Some(RealTimeScheduler::new(SchedulingAlgorithm::FixedPriority));
        
        if let Some(ref mut scheduler) = SCHEDULER {
            // 物理层接收任务（最高优先级）
            let phy_rx_task = TaskControlBlock::new(
                1,
                "PhysicalRx",
                250,
                1,    // 1ms周期
                0,    // 500μs截止时间
                physical_layer_rx_task,
            );
            scheduler.add_task(phy_rx_task).unwrap();
            
            // 数据链路层任务（高优先级）
            let dll_task = TaskControlBlock::new(
                2,
                "DataLinkLayer",
                200,
                5,    // 5ms周期
                4,    // 4ms截止时间
                data_link_layer_task,
            );
            scheduler.add_task(dll_task).unwrap();
            
            // 物理层发送任务（中优先级）
            let phy_tx_task = TaskControlBlock::new(
                3,
                "PhysicalTx",
                150,
                10,   // 10ms周期
                8,    // 8ms截止时间
                physical_layer_tx_task,
            );
            scheduler.add_task(phy_tx_task).unwrap();
            
            // 应用层任务（低优先级）
            let app_task = TaskControlBlock::new(
                4,
                "ApplicationLayer",
                100,
                50,   // 50ms周期
                40,   // 40ms截止时间
                application_layer_task,
            );
            scheduler.add_task(app_task).unwrap();
            
            scheduler.start();
        }
    }
}
```

## 性能分析和优化

### 调度性能监控

```rust
// 调度器性能监控
struct SchedulerPerformanceMonitor {
    total_schedule_calls: u32,
    max_schedule_time: u32,
    total_schedule_time: u64,
    context_switches: u32,
    idle_time: u64,
    last_idle_start: u32,
}

impl SchedulerPerformanceMonitor {
    fn new() -> Self {
        Self {
            total_schedule_calls: 0,
            max_schedule_time: 0,
            total_schedule_time: 0,
            context_switches: 0,
            idle_time: 0,
            last_idle_start: 0,
        }
    }
    
    fn start_schedule_measurement(&mut self) -> u32 {
        get_microsecond_timestamp()
    }
    
    fn end_schedule_measurement(&mut self, start_time: u32) {
        let end_time = get_microsecond_timestamp();
        let schedule_time = end_time - start_time;
        
        self.total_schedule_calls += 1;
        self.total_schedule_time += schedule_time as u64;
        self.max_schedule_time = self.max_schedule_time.max(schedule_time);
    }
    
    fn record_context_switch(&mut self) {
        self.context_switches += 1;
    }
    
    fn start_idle(&mut self) {
        self.last_idle_start = get_microsecond_timestamp();
    }
    
    fn end_idle(&mut self) {
        let idle_duration = get_microsecond_timestamp() - self.last_idle_start;
        self.idle_time += idle_duration as u64;
    }
    
    fn get_performance_report(&self) -> PerformanceReport {
        PerformanceReport {
            average_schedule_time: if self.total_schedule_calls > 0 {
                (self.total_schedule_time / self.total_schedule_calls as u64) as u32
            } else {
                0
            },
            max_schedule_time: self.max_schedule_time,
            context_switches_per_second: self.context_switches, // 需要基于时间计算
            cpu_utilization: self.calculate_cpu_utilization(),
        }
    }
    
    fn calculate_cpu_utilization(&self) -> f32 {
        let total_time = get_system_time() as u64 * 1000; // 转换为微秒
        if total_time > 0 {
            (1.0 - (self.idle_time as f32 / total_time as f32)) * 100.0
        } else {
            0.0
        }
    }
}

#[derive(Debug)]
struct PerformanceReport {
    average_schedule_time: u32,
    max_schedule_time: u32,
    context_switches_per_second: u32,
    cpu_utilization: f32,
}
```

### 可调度性分析

```rust
// 可调度性分析
impl RealTimeScheduler {
    fn analyze_schedulability(&self) -> SchedulabilityAnalysis {
        match self.scheduling_algorithm {
            SchedulingAlgorithm::RateMonotonic => self.rms_analysis(),
            SchedulingAlgorithm::EarliestDeadlineFirst => self.edf_analysis(),
            SchedulingAlgorithm::FixedPriority => self.response_time_analysis(),
        }
    }
    
    fn rms_analysis(&self) -> SchedulabilityAnalysis {
        let n = self.tasks.len() as f32;
        let utilization_bound = n * (2.0_f32.powf(1.0 / n) - 1.0);
        
        let total_utilization: f32 = self.tasks.iter()
            .map(|task| {
                let period_us = task.period_ms * 1000;
                task.execution_time_us as f32 / period_us as f32
            })
            .sum();
        
        SchedulabilityAnalysis {
            is_schedulable: total_utilization <= utilization_bound,
            total_utilization,
            utilization_bound,
            analysis_type: "Rate Monotonic Scheduling",
        }
    }
    
    fn edf_analysis(&self) -> SchedulabilityAnalysis {
        let total_utilization: f32 = self.tasks.iter()
            .map(|task| {
                let period_us = task.period_ms * 1000;
                task.execution_time_us as f32 / period_us as f32
            })
            .sum();
        
        SchedulabilityAnalysis {
            is_schedulable: total_utilization <= 1.0,
            total_utilization,
            utilization_bound: 1.0,
            analysis_type: "Earliest Deadline First",
        }
    }
    
    fn response_time_analysis(&self) -> SchedulabilityAnalysis {
        let mut all_schedulable = true;
        
        for task in &self.tasks {
            let response_time = self.calculate_response_time(task);
            if response_time > task.deadline_ms * 1000 {
                all_schedulable = false;
                break;
            }
        }
        
        let total_utilization: f32 = self.tasks.iter()
            .map(|task| {
                let period_us = task.period_ms * 1000;
                task.execution_time_us as f32 / period_us as f32
            })
            .sum();
        
        SchedulabilityAnalysis {
            is_schedulable: all_schedulable,
            total_utilization,
            utilization_bound: 1.0, // 理论上限
            analysis_type: "Response Time Analysis",
        }
    }
    
    fn calculate_response_time(&self, target_task: &TaskControlBlock) -> u32 {
        let mut response_time = target_task.execution_time_us;
        let mut prev_response_time;
        
        // 迭代计算响应时间
        loop {
            prev_response_time = response_time;
            
            // 计算高优先级任务的干扰
            let interference: u32 = self.tasks.iter()
                .filter(|task| task.priority > target_task.priority)
                .map(|task| {
                    let period_us = task.period_ms * 1000;
                    let interference_count = (response_time + period_us - 1) / period_us;
                    interference_count * task.execution_time_us
                })
                .sum();
            
            response_time = target_task.execution_time_us + interference;
            
            // 收敛检查
            if response_time == prev_response_time {
                break;
            }
            
            // 防止无限循环
            if response_time > target_task.deadline_ms * 1000 * 10 {
                response_time = u32::MAX;
                break;
            }
        }
        
        response_time
    }
}

#[derive(Debug)]
struct SchedulabilityAnalysis {
    is_schedulable: bool,
    total_utilization: f32,
    utilization_bound: f32,
    analysis_type: &'static str,
}
```

## 调试和测试

### 调度器调试工具

```rust
// 调度器调试工具
struct SchedulerDebugger {
    trace_buffer: Vec<ScheduleEvent, 1000>,
    trace_enabled: bool,
    current_index: usize,
}

#[derive(Clone, Copy)]
struct ScheduleEvent {
    timestamp: u32,
    event_type: EventType,
    task_id: u8,
    additional_data: u32,
}

#[derive(Clone, Copy)]
enum EventType {
    TaskStart,
    TaskEnd,
    TaskMissDeadline,
    ContextSwitch,
    InterruptEntry,
    InterruptExit,
}

impl SchedulerDebugger {
    fn new() -> Self {
        Self {
            trace_buffer: Vec::new(),
            trace_enabled: false,
            current_index: 0,
        }
    }
    
    fn enable_trace(&mut self) {
        self.trace_enabled = true;
    }
    
    fn disable_trace(&mut self) {
        self.trace_enabled = false;
    }
    
    fn log_event(&mut self, event_type: EventType, task_id: u8, additional_data: u32) {
        if !self.trace_enabled {
            return;
        }
        
        let event = ScheduleEvent {
            timestamp: get_microsecond_timestamp(),
            event_type,
            task_id,
            additional_data,
        };
        
        if self.trace_buffer.len() < 1000 {
            self.trace_buffer.push(event).ok();
        } else {
            self.trace_buffer[self.current_index] = event;
            self.current_index = (self.current_index + 1) % 1000;
        }
    }
    
    fn export_trace(&self) -> &[ScheduleEvent] {
        &self.trace_buffer
    }
    
    fn analyze_timing(&self) -> TimingAnalysis {
        let mut task_timings: heapless::FnvIndexMap<u8, TaskTiming, 16> = 
            heapless::FnvIndexMap::new();
        
        for event in &self.trace_buffer {
            match event.event_type {
                EventType::TaskStart => {
                    task_timings.entry(event.task_id)
                        .or_insert(TaskTiming::new())
                        .start_time = Some(event.timestamp);
                },
                EventType::TaskEnd => {
                    if let Some(timing) = task_timings.get_mut(&event.task_id) {
                        if let Some(start_time) = timing.start_time {
                            let execution_time = event.timestamp - start_time;
                            timing.total_execution_time += execution_time as u64;
                            timing.execution_count += 1;
                            timing.max_execution_time = 
                                timing.max_execution_time.max(execution_time);
                        }
                    }
                },
                _ => {}
            }
        }
        
        TimingAnalysis {
            task_timings,
        }
    }
}

#[derive(Clone)]
struct TaskTiming {
    start_time: Option<u32>,
    total_execution_time: u64,
    execution_count: u32,
    max_execution_time: u32,
}

impl TaskTiming {
    fn new() -> Self {
        Self {
            start_time: None,
            total_execution_time: 0,
            execution_count: 0,
            max_execution_time: 0,
        }
    }
    
    fn average_execution_time(&self) -> u32 {
        if self.execution_count > 0 {
            (self.total_execution_time / self.execution_count as u64) as u32
        } else {
            0
        }
    }
}

struct TimingAnalysis {
    task_timings: heapless::FnvIndexMap<u8, TaskTiming, 16>,
}
```

## 总结

基于定时器的实时调度系统是嵌入式系统的核心技术，通过精确的时间控制和合理的任务调度策略，可以确保系统的实时性和可靠性。

本章介绍了实时调度系统的基本概念、调度算法、任务同步机制以及具体的实现方法。通过数据采集、电机控制、通信协议栈等实际应用实例，展示了如何构建高效的实时系统。

在实际开发中，需要根据系统需求选择合适的调度算法，进行可调度性分析，并通过性能监控和调试工具确保系统的正确性和效率。

## 参考资料

- Real-Time Systems: Design Principles for Distributed Embedded Applications
- STM32F4xx Reference Manual
- ARM Cortex-M4 Technical Reference Manual
- Rate Monotonic Analysis for Real-Time Systems
- FreeRTOS Real Time Kernel Reference Manual