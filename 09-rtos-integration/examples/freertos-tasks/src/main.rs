#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{gpioa::PA5, gpioc::PC13, Output, PushPull},
    pac,
    prelude::*,
};

use freertos_rust::*;
use heapless::spsc::{Consumer, Producer, Queue};
use core::mem::MaybeUninit;

// 全局队列用于任务间通信
static mut QUEUE: Queue<u32, 16> = Queue::new();
static mut PRODUCER: MaybeUninit<Producer<u32, 16>> = MaybeUninit::uninit();
static mut CONSUMER: MaybeUninit<Consumer<u32, 16>> = MaybeUninit::uninit();

// 全局GPIO资源
static mut LED_USER: Option<PA5<Output<PushPull>>> = None;
static mut LED_BUILTIN: Option<PC13<Output<PushPull>>> = None;

#[entry]
fn main() -> ! {
    // 初始化硬件
    let dp = pac::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    
    let led_user = gpioa.pa5.into_push_pull_output();
    let led_builtin = gpioc.pc13.into_push_pull_output();
    
    // 初始化队列
    let (producer, consumer) = unsafe { QUEUE.split() };
    unsafe {
        PRODUCER.write(producer);
        CONSUMER.write(consumer);
        LED_USER = Some(led_user);
        LED_BUILTIN = Some(led_builtin);
    }
    
    // 创建FreeRTOS任务
    
    // 高优先级任务 - 数据生产者
    Task::new()
        .name("DataProducer")
        .stack_size(512)
        .priority(TaskPriority(3))
        .start(data_producer_task)
        .unwrap();
    
    // 中优先级任务 - 数据消费者
    Task::new()
        .name("DataConsumer")
        .stack_size(512)
        .priority(TaskPriority(2))
        .start(data_consumer_task)
        .unwrap();
    
    // 低优先级任务 - LED闪烁
    Task::new()
        .name("LedBlinker")
        .stack_size(256)
        .priority(TaskPriority(1))
        .start(led_blink_task)
        .unwrap();
    
    // 监控任务 - 系统状态监控
    Task::new()
        .name("SystemMonitor")
        .stack_size(512)
        .priority(TaskPriority(4))
        .start(system_monitor_task)
        .unwrap();
    
    // 启动FreeRTOS调度器
    FreeRtosUtils::start_scheduler();
}

// 数据生产者任务
fn data_producer_task(_: FreeRtosTaskHandle) {
    let mut counter = 0u32;
    
    loop {
        // 生产数据
        counter = counter.wrapping_add(1);
        
        unsafe {
            if let Some(ref mut producer) = PRODUCER.assume_init_mut() {
                if producer.enqueue(counter).is_ok() {
                    // 数据成功入队
                } else {
                    // 队列满，处理溢出
                }
            }
        }
        
        // 每100ms生产一次数据
        CurrentTask::delay(Duration::ms(100));
    }
}

// 数据消费者任务
fn data_consumer_task(_: FreeRtosTaskHandle) {
    loop {
        unsafe {
            if let Some(ref mut consumer) = CONSUMER.assume_init_mut() {
                if let Some(data) = consumer.dequeue() {
                    // 处理接收到的数据
                    process_data(data);
                }
            }
        }
        
        // 每50ms检查一次队列
        CurrentTask::delay(Duration::ms(50));
    }
}

// LED闪烁任务
fn led_blink_task(_: FreeRtosTaskHandle) {
    loop {
        unsafe {
            if let Some(ref mut led) = LED_BUILTIN {
                led.toggle();
            }
        }
        
        // 每500ms闪烁一次
        CurrentTask::delay(Duration::ms(500));
    }
}

// 系统监控任务
fn system_monitor_task(_: FreeRtosTaskHandle) {
    let mut cycle_count = 0u32;
    
    loop {
        cycle_count += 1;
        
        // 每10个周期检查一次系统状态
        if cycle_count % 10 == 0 {
            check_system_health();
            
            // 用户LED快闪表示系统正常
            unsafe {
                if let Some(ref mut led) = LED_USER {
                    for _ in 0..3 {
                        led.set_high();
                        CurrentTask::delay(Duration::ms(50));
                        led.set_low();
                        CurrentTask::delay(Duration::ms(50));
                    }
                }
            }
        }
        
        // 每1秒监控一次
        CurrentTask::delay(Duration::ms(1000));
    }
}

// 数据处理函数
fn process_data(data: u32) {
    // 模拟数据处理
    let _processed = data * 2 + 1;
    
    // 如果数据是特殊值，触发用户LED
    if data % 10 == 0 {
        unsafe {
            if let Some(ref mut led) = LED_USER {
                led.set_high();
                CurrentTask::delay(Duration::ms(100));
                led.set_low();
            }
        }
    }
}

// 系统健康检查
fn check_system_health() {
    // 检查任务状态
    let _task_count = get_task_count();
    
    // 检查内存使用
    let _free_heap = get_free_heap_size();
    
    // 检查队列状态
    unsafe {
        if let Some(ref consumer) = CONSUMER.assume_init_ref() {
            let _queue_len = consumer.len();
        }
    }
}

// 获取任务数量 (模拟)
fn get_task_count() -> u32 {
    4 // 我们创建了4个任务
}

// 获取空闲堆大小 (模拟)
fn get_free_heap_size() -> u32 {
    1024 // 模拟值
}

// FreeRTOS钩子函数

#[no_mangle]
pub extern "C" fn vApplicationStackOverflowHook(
    _task_handle: FreeRtosTaskHandle,
    _task_name: *const i8,
) {
    // 栈溢出处理
    panic!("Stack overflow detected!");
}

#[no_mangle]
pub extern "C" fn vApplicationMallocFailedHook() {
    // 内存分配失败处理
    panic!("Memory allocation failed!");
}

#[no_mangle]
pub extern "C" fn vApplicationIdleHook() {
    // 空闲任务钩子
    // 可以在这里实现低功耗模式
}

#[no_mangle]
pub extern "C" fn vApplicationTickHook() {
    // 系统滴答钩子
    // 每个系统滴答都会调用
}

// 内存分配器配置
#[global_allocator]
static ALLOCATOR: linked_list_allocator::LockedHeap = linked_list_allocator::LockedHeap::empty();

// 堆内存配置
const HEAP_SIZE: usize = 8192;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[no_mangle]
pub extern "C" fn __freertos_heap_init() {
    unsafe {
        ALLOCATOR.lock().init(HEAP.as_mut_ptr(), HEAP_SIZE);
    }
}