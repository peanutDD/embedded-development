#![no_std]
#![no_main]

//! # 中断驱动编程主程序
//! 
//! 演示完整的中断驱动系统，包括：
//! - 多源中断处理
//! - 事件优先级管理
//! - 异步任务处理
//! - 状态机控制
//! - 性能监控

use panic_probe as _;
use defmt_rtt as _;

// 平台特定导入
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;
#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal;
#[cfg(feature = "nrf52")]
use nrf52840_hal as hal;

use hal::{
    gpio::{Input, Output, PushPull, PullUp},
    pac::{interrupt, Interrupt, NVIC},
    prelude::*,
    timer::{Timer, Event},
};

use cortex_m::{
    interrupt::{free, Mutex},
    peripheral::NVIC as CortexNVIC,
};
use cortex_m_rt::entry;

use interrupt_driven::{
    EventProcessor, AsyncInterruptHandler, InterruptStateMachine,
    RingBufferManager, SystemEvent, Priority, EdgeType,
    AsyncTask, TaskType, ProcessorConfig, BufferConfig,
};

use core::cell::RefCell;
use heapless::Vec;

/// 系统状态
#[derive(Debug, Clone, Copy, PartialEq)]
enum SystemState {
    Initializing,
    Running,
    Processing,
    Monitoring,
    Error,
}

/// 中断驱动系统控制器
struct InterruptDrivenController<LED, BTN> {
    /// 状态LED
    status_led: LED,
    /// 用户按钮
    user_button: BTN,
    /// 事件处理器
    event_processor: EventProcessor<32>,
    /// 异步处理器
    async_handler: AsyncInterruptHandler<LED, 16>,
    /// 状态机
    state_machine: InterruptStateMachine,
    /// 数据缓冲区
    data_buffer: RingBufferManager<u8, 64>,
    /// 系统状态
    system_state: SystemState,
    /// 统计计数器
    interrupt_count: u32,
    /// 性能指标
    performance_metrics: PerformanceMetrics,
}

/// 性能指标
#[derive(Debug, Default)]
struct PerformanceMetrics {
    total_interrupts: u32,
    gpio_interrupts: u32,
    timer_interrupts: u32,
    uart_interrupts: u32,
    max_response_time: u32,
    avg_response_time: u32,
}

// 全局静态变量
static SYSTEM_CONTROLLER: Mutex<RefCell<Option<InterruptDrivenController<
    hal::gpio::Pin<Output<PushPull>>,
    hal::gpio::Pin<Input<PullUp>>
>>>> = Mutex::new(RefCell::new(None));

static TIMER: Mutex<RefCell<Option<Timer<hal::pac::TIM2>>>> = Mutex::new(RefCell::new(None));

impl<LED, BTN> InterruptDrivenController<LED, BTN>
where
    LED: embedded_hal::digital::OutputPin + Clone,
    BTN: embedded_hal::digital::InputPin,
{
    /// 创建新的控制器
    fn new(
        status_led: LED,
        user_button: BTN,
    ) -> Self {
        let processor_config = ProcessorConfig {
            max_queue_size: 32,
            enable_stats: true,
            timeout_ms: 1000,
        };
        
        let buffer_config = BufferConfig {
            watermark_high: 48,
            watermark_low: 16,
            enable_overwrite: true,
        };
        
        Self {
            async_handler: AsyncInterruptHandler::new(status_led.clone()),
            status_led,
            user_button,
            event_processor: EventProcessor::new(processor_config),
            state_machine: InterruptStateMachine::new(),
            data_buffer: RingBufferManager::new(buffer_config),
            system_state: SystemState::Initializing,
            interrupt_count: 0,
            performance_metrics: PerformanceMetrics::default(),
        }
    }
    
    /// 初始化系统
    fn initialize(&mut self) -> Result<(), ()> {
        defmt::info!("初始化中断驱动系统...");
        
        // 设置初始状态
        self.system_state = SystemState::Running;
        let _ = self.status_led.set_low();
        
        // 添加初始化任务
        let init_task = AsyncTask {
            id: 1,
            task_type: TaskType::DataProcessing,
            data: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            timestamp: 0,
            retry_count: 0,
        };
        
        self.async_handler.add_task(init_task).map_err(|_| ())?;
        
        defmt::info!("系统初始化完成");
        Ok(())
    }
    
    /// 处理GPIO中断
    fn handle_gpio_interrupt(&mut self, pin: u8, edge: EdgeType) {
        self.interrupt_count += 1;
        self.performance_metrics.gpio_interrupts += 1;
        
        let event = SystemEvent::GpioInterrupt { pin, edge };
        
        // 添加到事件队列
        if let Err(_) = self.event_processor.push_event(event, Priority::High) {
            defmt::warn!("事件队列已满，丢弃GPIO中断事件");
        }
        
        // 状态机处理
        let _ = self.state_machine.handle_event(event);
        
        // 创建异步任务
        let task = AsyncTask {
            id: self.interrupt_count as u16,
            task_type: TaskType::DataProcessing,
            data: [pin, edge as u8, 0, 0, 0, 0, 0, 0],
            timestamp: self.get_current_time(),
            retry_count: 0,
        };
        
        let _ = self.async_handler.add_task(task);
        
        defmt::debug!("处理GPIO中断: pin={}, edge={:?}", pin, edge);
    }
    
    /// 处理定时器中断
    fn handle_timer_interrupt(&mut self, timer_id: u8) {
        self.interrupt_count += 1;
        self.performance_metrics.timer_interrupts += 1;
        
        let event = SystemEvent::TimerInterrupt { timer_id };
        
        // 添加到事件队列
        if let Err(_) = self.event_processor.push_event(event, Priority::Medium) {
            defmt::warn!("事件队列已满，丢弃定时器中断事件");
        }
        
        // 状态机处理
        let _ = self.state_machine.handle_event(event);
        
        // 更新系统状态
        self.update_system_status();
        
        defmt::debug!("处理定时器中断: timer_id={}", timer_id);
    }
    
    /// 处理UART中断
    fn handle_uart_interrupt(&mut self, data: u8) {
        self.interrupt_count += 1;
        self.performance_metrics.uart_interrupts += 1;
        
        let event = SystemEvent::UartReceive { data };
        
        // 添加到事件队列
        if let Err(_) = self.event_processor.push_event(event, Priority::High) {
        }
        
        // 存储到缓冲区
        if let Err(_) = self.data_buffer.write(data) {
            defmt::warn!("数据缓冲区已满，丢弃数据: 0x{:02x}", data);
        }
        
        // 创建通信任务
        let task = AsyncTask {
            id: (self.interrupt_count + 1000) as u16,
            task_type: TaskType::Communication,
            data: [data, 0, 0, 0, 0, 0, 0, 0],
            timestamp: self.get_current_time(),
            retry_count: 0,
        };
        
        let _ = self.async_handler.add_task(task);
        
        defmt::debug!("处理UART中断: data=0x{:02x}", data);
    }
    
    /// 主循环处理
    fn run_main_loop(&mut self) {
        loop {
            // 处理事件队列
            self.process_events();
            
            // 处理异步任务
            self.process_async_tasks();
            
            // 处理数据缓冲区
            self.process_data_buffer();
            
            // 更新性能指标
            self.update_performance_metrics();
            
            // 系统监控
            self.monitor_system_health();
            
            // 短暂延时
            self.delay_ms(10);
        }
    }
    
    /// 处理事件队列
    fn process_events(&mut self) {
        while let Some(event) = self.event_processor.process_next_event() {
            match event {
                SystemEvent::GpioInterrupt { pin, edge } => {
                    self.handle_gpio_event(pin, edge);
                },
                SystemEvent::TimerInterrupt { timer_id } => {
                    self.handle_timer_event(timer_id);
                },
                SystemEvent::UartReceive { data } => {
                    self.handle_uart_event(data);
                },
                SystemEvent::AdcComplete { channel, value } => {
                    self.handle_adc_event(channel, value);
                },
                SystemEvent::UserEvent { id, data } => {
                    self.handle_user_event(id, data);
                },
            }
        }
    }
    
    /// 处理异步任务
    fn process_async_tasks(&mut self) {
        // 处理多个任务
        for _ in 0..5 {
            match self.async_handler.process_next_task() {
                Ok(true) => {
                    // 任务处理成功
                },
                Ok(false) => {
                    // 没有更多任务
                    break;
                },
                Err(_) => {
                    // 任务处理失败
                    defmt::warn!("异步任务处理失败");
                    break;
                },
            }
        }
    }
    
    /// 处理数据缓冲区
    fn process_data_buffer(&mut self) {
        // 检查水位线
        let (high_water, low_water) = self.data_buffer.check_watermarks();
        
        if high_water {
            defmt::info!("数据缓冲区高水位告警");
            self.system_state = SystemState::Processing;
            let _ = self.status_led.set_high();
            
            // 批量处理数据
            let mut batch_data = Vec::<u8, 16>::new();
            for _ in 0..16 {
                if let Some(data) = self.data_buffer.read() {
                    let _ = batch_data.push(data);
                } else {
                    break;
                }
            }
            
            if !batch_data.is_empty() {
                self.process_data_batch(&batch_data);
            }
        } else if low_water && self.system_state == SystemState::Processing {
            defmt::info!("数据缓冲区低水位，恢复正常");
            self.system_state = SystemState::Running;
            let _ = self.status_led.set_low();
        }
    }
    
    /// 批量处理数据
    fn process_data_batch(&mut self, data: &[u8]) {
        defmt::info!("批量处理数据: {} 字节", data.len());
        
        // 计算校验和
        let checksum: u8 = data.iter().fold(0, |acc, &x| acc.wrapping_add(x));
        
        // 创建计算任务
        let task = AsyncTask {
            id: 9999,
            task_type: TaskType::Calculation,
            data: [checksum, data.len() as u8, 0, 0, 0, 0, 0, 0],
            timestamp: self.get_current_time(),
            retry_count: 0,
        };
        
        let _ = self.async_handler.add_task(task);
    }
    
    /// 更新性能指标
    fn update_performance_metrics(&mut self) {
        self.performance_metrics.total_interrupts = self.interrupt_count;
        
        // 计算平均响应时间（模拟）
        if self.performance_metrics.total_interrupts > 0 {
            self.performance_metrics.avg_response_time = 
                self.performance_metrics.total_interrupts / 100; // 模拟值
        }
    }
    
    /// 监控系统健康状态
    fn monitor_system_health(&mut self) {
        let queue_length = self.event_processor.queue_length();
        let async_queue_length = self.async_handler.queue_length();
        let buffer_fill = self.data_buffer.fill_level();
        
        // 检查系统负载
        if queue_length > 24 || async_queue_length > 12 || buffer_fill > 48 {
            if self.system_state != SystemState::Error {
                defmt::warn!("系统负载过高: queue={}, async={}, buffer={}", 
                    queue_length, async_queue_length, buffer_fill);
                self.system_state = SystemState::Error;
            }
        } else if self.system_state == SystemState::Error {
            defmt::info!("系统负载恢复正常");
            self.system_state = SystemState::Running;
        }
        
        // 定期输出统计信息
        if self.interrupt_count % 1000 == 0 && self.interrupt_count > 0 {
            self.print_statistics();
        }
    }
    
    /// 输出统计信息
    fn print_statistics(&self) {
        let stats = self.event_processor.get_stats();
        let perf = self.async_handler.get_performance();
        let buffer_stats = self.data_buffer.get_stats();
        
        defmt::info!("=== 系统统计信息 ===");
        defmt::info!("总中断数: {}", self.performance_metrics.total_interrupts);
        defmt::info!("GPIO中断: {}", self.performance_metrics.gpio_interrupts);
        defmt::info!("定时器中断: {}", self.performance_metrics.timer_interrupts);
        defmt::info!("UART中断: {}", self.performance_metrics.uart_interrupts);
        defmt::info!("事件处理: {}/{}", stats.processed_events, stats.total_events);
        defmt::info!("异步任务: {}", perf.task_count);
        defmt::info!("缓冲区读写: {}/{}", buffer_stats.total_reads, buffer_stats.total_writes);
        defmt::info!("系统状态: {:?}", self.system_state);
    }
    
    /// 事件处理函数
    fn handle_gpio_event(&mut self, pin: u8, edge: EdgeType) {
        defmt::debug!("处理GPIO事件: pin={}, edge={:?}", pin, edge);
    }
    
    fn handle_timer_event(&mut self, timer_id: u8) {
        defmt::debug!("处理定时器事件: timer_id={}", timer_id);
    }
    
    fn handle_uart_event(&mut self, data: u8) {
        defmt::debug!("处理UART事件: data=0x{:02x}", data);
    }
    
    fn handle_adc_event(&mut self, channel: u8, value: u16) {
        defmt::debug!("处理ADC事件: channel={}, value={}", channel, value);
    }
    
    fn handle_user_event(&mut self, id: u16, data: u32) {
        defmt::debug!("处理用户事件: id={}, data=0x{:08x}", id, data);
    }
    
    /// 更新系统状态
    fn update_system_status(&mut self) {
        // 切换状态LED
        match self.system_state {
            SystemState::Running => {
                // LED慢闪
                if self.interrupt_count % 100 == 0 {
                    let _ = self.status_led.set_high();
                } else if self.interrupt_count % 100 == 50 {
                    let _ = self.status_led.set_low();
                }
            },
            SystemState::Processing => {
                // LED快闪
                if self.interrupt_count % 10 == 0 {
                    let _ = self.status_led.set_high();
                } else if self.interrupt_count % 10 == 5 {
                    let _ = self.status_led.set_low();
                }
            },
            SystemState::Error => {
                // LED常亮
                let _ = self.status_led.set_high();
            },
            _ => {
                let _ = self.status_led.set_low();
            }
        }
    }
    
    /// 获取当前时间（模拟）
    fn get_current_time(&self) -> u32 {
        // 在实际应用中，这里应该返回系统时钟
        self.interrupt_count
    }
    
    /// 延时函数（模拟）
    fn delay_ms(&self, _ms: u32) {
        // 在实际应用中，这里应该使用HAL的延时函数
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
    }
}

#[entry]
fn main() -> ! {
    defmt::info!("启动中断驱动编程演示程序");
    
    // 初始化外设
    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    
    // 状态LED (PA5)
    let status_led = gpioa.pa5.into_push_pull_output();
    
    // 用户按钮 (PC13)
    let user_button = gpioc.pc13.into_pull_up_input();
    
    // 配置定时器
    let mut timer = Timer::tim2(dp.TIM2, 1.khz(), clocks);
    timer.listen(Event::TimeOut);
    
    // 创建系统控制器
    let mut controller = InterruptDrivenController::new(status_led, user_button);
    
    // 初始化系统
    if let Err(_) = controller.initialize() {
        defmt::error!("系统初始化失败");
        panic!("System initialization failed");
    }
    
    // 存储全局引用
    free(|cs| {
        SYSTEM_CONTROLLER.borrow(cs).replace(Some(controller));
        TIMER.borrow(cs).replace(Some(timer));
    });
    
    // 启用中断
    unsafe {
        NVIC::unmask(Interrupt::TIM2);
        NVIC::unmask(Interrupt::EXTI15_10);
    }
    
    defmt::info!("系统启动完成，开始主循环");
    
    // 主循环
    loop {
        free(|cs| {
            if let Some(ref mut controller) = SYSTEM_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                controller.run_main_loop();
            }
        });
    }
}

/// 定时器中断处理程序
#[interrupt]
fn TIM2() {
    free(|cs| {
        if let Some(ref mut timer) = TIMER.borrow(cs).borrow_mut().as_mut() {
            timer.clear_interrupt(Event::TimeOut);
        }
        
        if let Some(ref mut controller) = SYSTEM_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
            controller.handle_timer_interrupt(2);
        }
    });
}

/// GPIO中断处理程序
#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        if let Some(ref mut controller) = SYSTEM_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
            controller.handle_gpio_interrupt(13, EdgeType::Falling);
        }
    });
}

/// UART中断处理程序（模拟）
fn simulate_uart_interrupt(data: u8) {
    free(|cs| {
        if let Some(ref mut controller) = SYSTEM_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
            controller.handle_uart_interrupt(data);
        }
    });
}