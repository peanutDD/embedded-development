#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::asm;

// 平台特定的HAL导入
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;

use hal::{
    prelude::*,
    timer::{Timer, Event},
    gpio::{Input, Output, PushPull, PullUp},
    watchdog::IndependentWatchdog as HalWatchdog,
};

use watchdog_timer::{
    SystemMonitor, IndependentWatchdog, WindowWatchdog, FaultRecoveryManager,
    WatchdogTimer, WatchdogConfig, RecoveryStrategy, ResetReason
};
use heapless::{Vec, String};
use nb::block;

/// 系统任务ID
const MAIN_TASK_ID: u8 = 1;
const COMM_TASK_ID: u8 = 2;
const SENSOR_TASK_ID: u8 = 3;
const ACTUATOR_TASK_ID: u8 = 4;

/// 系统状态
#[derive(Debug, Clone, Copy)]
enum SystemState {
    Normal,
    SafeMode,
    Recovery,
    Shutdown,
}

/// 任务状态
#[derive(Debug, Clone)]
struct TaskState {
    id: u8,
    name: String<32>,
    last_execution: u32,
    execution_count: u32,
    error_count: u32,
    is_active: bool,
}

/// 系统控制器
struct SystemController {
    monitor: SystemMonitor,
    watchdog: IndependentWatchdog,
    recovery_manager: FaultRecoveryManager,
    tasks: Vec<TaskState, 8>,
    system_state: SystemState,
    system_time_ms: u32,
    last_watchdog_feed: u32,
    error_count: u32,
}

impl SystemController {
    fn new() -> Self {
        let config = WatchdogConfig {
            timeout_ms: 2000,  // 2秒超时
            window_ms: None,
            prescaler: 4,
            auto_reload: true,
            debug_freeze: true,
            standby_freeze: true,
        };

        let mut monitor = SystemMonitor::new(100); // 100ms检查间隔
        let watchdog = IndependentWatchdog::new(config);
        let mut recovery_manager = FaultRecoveryManager::new(5); // 最多5次恢复尝试

        // 添加任务监控
        monitor.add_task(MAIN_TASK_ID, "main_task", 1000, true).ok();
        monitor.add_task(COMM_TASK_ID, "comm_task", 2000, false).ok();
        monitor.add_task(SENSOR_TASK_ID, "sensor_task", 500, true).ok();
        monitor.add_task(ACTUATOR_TASK_ID, "actuator_task", 1500, false).ok();

        // 设置恢复策略
        recovery_manager.set_recovery_strategy(MAIN_TASK_ID, RecoveryStrategy::Restart).ok();
        recovery_manager.set_recovery_strategy(COMM_TASK_ID, RecoveryStrategy::SafeMode).ok();
        recovery_manager.set_recovery_strategy(SENSOR_TASK_ID, RecoveryStrategy::Restart).ok();
        recovery_manager.set_recovery_strategy(ACTUATOR_TASK_ID, RecoveryStrategy::Ignore).ok();

        Self {
            monitor,
            watchdog,
            recovery_manager,
            tasks: Vec::new(),
            system_state: SystemState::Normal,
            system_time_ms: 0,
            last_watchdog_feed: 0,
            error_count: 0,
        }
    }

    fn initialize_tasks(&mut self) {
        let task_configs = [
            (MAIN_TASK_ID, "main_task"),
            (COMM_TASK_ID, "comm_task"),
            (SENSOR_TASK_ID, "sensor_task"),
            (ACTUATOR_TASK_ID, "actuator_task"),
        ];

        for (id, name) in &task_configs {
            let mut task_name = String::new();
            task_name.push_str(name).ok();
            
            let task = TaskState {
                id: *id,
                name: task_name,
                last_execution: 0,
                execution_count: 0,
                error_count: 0,
                is_active: true,
            };
            
            self.tasks.push(task).ok();
        }
    }

    fn update_system_time(&mut self) {
        self.system_time_ms += 1; // 简化的时间更新
    }

    fn execute_task(&mut self, task_id: u8) -> Result<(), ()> {
        // 查找任务
        let task_index = self.tasks.iter().position(|t| t.id == task_id);
        if let Some(index) = task_index {
            let task = &mut self.tasks[index];
            
            if !task.is_active {
                return Err(());
            }

            // 模拟任务执行
            let execution_result = match task_id {
                MAIN_TASK_ID => self.execute_main_task(),
                COMM_TASK_ID => self.execute_comm_task(),
                SENSOR_TASK_ID => self.execute_sensor_task(),
                ACTUATOR_TASK_ID => self.execute_actuator_task(),
                _ => Err(()),
            };

            // 更新任务状态
            task.last_execution = self.system_time_ms;
            task.execution_count += 1;

            if execution_result.is_err() {
                task.error_count += 1;
                self.error_count += 1;
            }

            // 更新监控器
            self.monitor.update_task(task_id, self.system_time_ms).ok();

            execution_result
        } else {
            Err(())
        }
    }

    fn execute_main_task(&mut self) -> Result<(), ()> {
        // 主任务：系统状态检查和协调
        match self.system_state {
            SystemState::Normal => {
                // 正常运行逻辑
                Ok(())
            },
            SystemState::SafeMode => {
                // 安全模式逻辑
                Ok(())
            },
            SystemState::Recovery => {
                // 恢复模式逻辑
                self.system_state = SystemState::Normal;
                Ok(())
            },
            SystemState::Shutdown => {
                // 关闭模式
                Err(())
            },
        }
    }

    fn execute_comm_task(&mut self) -> Result<(), ()> {
        // 通信任务：模拟通信操作
        // 10%的概率出现通信错误
        if self.system_time_ms % 10 == 7 {
            Err(())
        } else {
            Ok(())
        }
    }

    fn execute_sensor_task(&mut self) -> Result<(), ()> {
        // 传感器任务：读取传感器数据
        // 5%的概率出现传感器错误
        if self.system_time_ms % 20 == 13 {
            Err(())
        } else {
            // 更新系统健康状态
            let cpu_usage = 45.0 + (self.system_time_ms % 100) as f32 * 0.1;
            let memory_usage = 60.0 + (self.system_time_ms % 50) as f32 * 0.2;
            let temperature = 25.0 + (self.system_time_ms % 30) as f32 * 0.5;
            let voltage = 3.3 + (self.system_time_ms % 10) as f32 * 0.01;
            
            self.monitor.update_health(cpu_usage, memory_usage, temperature, voltage);
            Ok(())
        }
    }

    fn execute_actuator_task(&mut self) -> Result<(), ()> {
        // 执行器任务：控制执行器
        // 3%的概率出现执行器错误
        if self.system_time_ms % 33 == 17 {
            Err(())
        } else {
            Ok(())
        }
    }

    fn check_system_health(&mut self) {
        // 检查任务超时
        let timed_out_tasks = self.monitor.check_system(self.system_time_ms);
        
        for task in timed_out_tasks {
            // 处理超时任务
            let recovery_strategy = self.recovery_manager.handle_fault(task.task_id, self.system_time_ms);
            
            match recovery_strategy {
                RecoveryStrategy::Restart => {
                    self.restart_task(task.task_id);
                },
                RecoveryStrategy::SafeMode => {
                    self.enter_safe_mode();
                },
                RecoveryStrategy::Ignore => {
                    // 忽略错误，继续运行
                },
                RecoveryStrategy::Shutdown => {
                    self.shutdown_system();
                },
            }
        }

        // 检查系统健康指标
        let health = self.monitor.get_health();
        
        if health.cpu_usage > 90.0 || health.memory_usage > 95.0 {
            self.enter_safe_mode();
        }
        
        if health.temperature > 85.0 {
            self.shutdown_system();
        }
        
        if health.voltage < 2.8 || health.voltage > 3.6 {
            self.enter_safe_mode();
        }
    }

    fn restart_task(&mut self, task_id: u8) {
        for task in &mut self.tasks {
            if task.id == task_id {
                task.is_active = true;
                task.error_count = 0;
                task.last_execution = self.system_time_ms;
                break;
            }
        }
        
        self.recovery_manager.mark_recovery_success(task_id);
    }

    fn enter_safe_mode(&mut self) {
        self.system_state = SystemState::SafeMode;
        
        // 禁用非关键任务
        for task in &mut self.tasks {
            if task.id == COMM_TASK_ID || task.id == ACTUATOR_TASK_ID {
                task.is_active = false;
            }
        }
    }

    fn shutdown_system(&mut self) {
        self.system_state = SystemState::Shutdown;
        
        // 禁用所有任务
        for task in &mut self.tasks {
            task.is_active = false;
        }
        
        // 停止看门狗
        self.watchdog.stop().ok();
    }

    fn feed_watchdog(&mut self) -> Result<(), ()> {
        if self.system_time_ms > self.last_watchdog_feed + 1000 { // 每秒喂一次狗
            self.watchdog.feed()?;
            self.last_watchdog_feed = self.system_time_ms;
        }
        Ok(())
    }

    fn get_system_status(&self) -> String<256> {
        let mut status = String::new();
        
        // 系统状态
        match self.system_state {
            SystemState::Normal => status.push_str("NORMAL").ok(),
            SystemState::SafeMode => status.push_str("SAFE_MODE").ok(),
            SystemState::Recovery => status.push_str("RECOVERY").ok(),
            SystemState::Shutdown => status.push_str("SHUTDOWN").ok(),
        };
        
        status.push_str(" | ").ok();
        
        // 任务状态
        let active_tasks = self.tasks.iter().filter(|t| t.is_active).count();
        status.push_str("Tasks: ").ok();
        // 这里简化了数字转字符串的过程
        
        status
    }
}

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 配置状态LED
    let mut system_led = gpioc.pc13.into_push_pull_output();
    let mut error_led = gpioc.pc14.into_push_pull_output();
    let mut watchdog_led = gpioc.pc15.into_push_pull_output();
    
    // 配置按钮
    let reset_button = gpioa.pa0.into_pull_up_input();
    let safe_mode_button = gpioa.pa1.into_pull_up_input();
    let test_button = gpioa.pa2.into_pull_up_input();

    // 初始化系统控制器
    let mut system_controller = SystemController::new();
    system_controller.initialize_tasks();
    
    // 启动看门狗
    system_controller.watchdog.start(2000).unwrap(); // 2秒超时
    system_controller.monitor.enable_monitoring();

    // 主循环
    loop {
        // 更新系统时间
        system_controller.update_system_time();
        
        // 检查按钮
        if reset_button.is_low().unwrap() {
            // 重置系统
            system_controller = SystemController::new();
            system_controller.initialize_tasks();
            system_controller.watchdog.start(2000).unwrap();
            system_controller.monitor.enable_monitoring();
        }
        
        if safe_mode_button.is_low().unwrap() {
            system_controller.enter_safe_mode();
        }
        
        if test_button.is_low().unwrap() {
            // 模拟故障测试
            system_controller.error_count += 10;
        }

        // 执行任务调度
        match system_controller.system_state {
            SystemState::Normal | SystemState::Recovery => {
                // 执行所有活动任务
                let task_ids = [MAIN_TASK_ID, COMM_TASK_ID, SENSOR_TASK_ID, ACTUATOR_TASK_ID];
                
                for &task_id in &task_ids {
                    if system_controller.system_time_ms % get_task_period(task_id) == 0 {
                        system_controller.execute_task(task_id).ok();
                    }
                }
            },
            SystemState::SafeMode => {
                // 只执行关键任务
                if system_controller.system_time_ms % 100 == 0 {
                    system_controller.execute_task(MAIN_TASK_ID).ok();
                }
                if system_controller.system_time_ms % 500 == 0 {
                    system_controller.execute_task(SENSOR_TASK_ID).ok();
                }
            },
            SystemState::Shutdown => {
                // 系统关闭，只保持最基本的监控
            },
        }

        // 检查系统健康状态
        system_controller.check_system_health();

        // 喂看门狗
        if system_controller.system_state != SystemState::Shutdown {
            system_controller.feed_watchdog().ok();
        }

        // 更新LED状态
        update_led_status(
            &mut system_led,
            &mut error_led,
            &mut watchdog_led,
            &system_controller
        );

        // 短暂延时
        for _ in 0..10000 {
            asm::nop();
        }
    }
}

/// 获取任务执行周期
fn get_task_period(task_id: u8) -> u32 {
    match task_id {
        MAIN_TASK_ID => 100,     // 100ms
        COMM_TASK_ID => 200,     // 200ms
        SENSOR_TASK_ID => 50,    // 50ms
        ACTUATOR_TASK_ID => 150, // 150ms
        _ => 1000,
    }
}

/// 更新LED状态指示
fn update_led_status(
    system_led: &mut hal::gpio::gpioc::PC13<Output<PushPull>>,
    error_led: &mut hal::gpio::gpioc::PC14<Output<PushPull>>,
    watchdog_led: &mut hal::gpio::gpioc::PC15<Output<PushPull>>,
    controller: &SystemController,
) {
    // 系统状态LED
    match controller.system_state {
        SystemState::Normal => {
            system_led.set_high().ok();
        },
        SystemState::SafeMode => {
            // 慢速闪烁
            if (controller.system_time_ms / 500) % 2 == 0 {
                system_led.set_high().ok();
            } else {
                system_led.set_low().ok();
            }
        },
        SystemState::Recovery => {
            // 快速闪烁
            if (controller.system_time_ms / 100) % 2 == 0 {
                system_led.set_high().ok();
            } else {
                system_led.set_low().ok();
            }
        },
        SystemState::Shutdown => {
            system_led.set_low().ok();
        },
    }

    // 错误LED
    if controller.error_count > 0 {
        if (controller.system_time_ms / 200) % 2 == 0 {
            error_led.set_high().ok();
        } else {
            error_led.set_low().ok();
        }
    } else {
        error_led.set_low().ok();
    }

    // 看门狗LED：心跳指示
    if controller.watchdog.get_remaining_time() > 0 {
        if (controller.system_time_ms / 1000) % 2 == 0 {
            watchdog_led.set_high().ok();
        } else {
            watchdog_led.set_low().ok();
        }
    } else {
        watchdog_led.set_low().ok();
    }
}

/// 演示看门狗功能
fn demo_watchdog_features() {
    // 演示独立看门狗
    let config = WatchdogConfig::default();
    let mut iwdg = IndependentWatchdog::new(config);
    
    iwdg.start(1000).unwrap();
    
    // 正常喂狗
    for _ in 0..5 {
        iwdg.feed().unwrap();
        // 延时500ms
    }
    
    // 演示窗口看门狗
    let mut window_config = WatchdogConfig::default();
    window_config.window_ms = Some(500);
    let mut wwdg = WindowWatchdog::new(window_config);
    
    wwdg.start(1000).unwrap();
    
    // 在窗口内喂狗
    // 这里需要精确的时间控制
}

/// 系统复位检查
fn check_reset_reason() -> ResetReason {
    // 这里应该读取复位状态寄存器
    // 根据不同的复位标志位确定复位原因
    ResetReason::PowerOn
}