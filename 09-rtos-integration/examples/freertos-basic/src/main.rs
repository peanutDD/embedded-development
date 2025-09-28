#![no_main]
#![no_std]

// FreeRTOS基础示例
// 功能：多任务管理、同步机制、资源共享
// 硬件：STM32F407VG Discovery
// 演示：任务创建、队列通信、信号量、互斥锁

use panic_halt as _;

use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};

use stm32f4xx_hal::{
  gpio::{
    gpioa::PA0,
    gpiod::{PD12, PD13, PD14, PD15},
    Input, Output, PullUp, PushPull,
  },
  pac,
  prelude::*,
  timer::{Event, Timer},
};

use freertos_rust::{
  BinarySemaphore, CountingSemaphore, CurrentTask, Duration, FreeRtosError, Mutex, Queue, Task,
  TaskHandle, TaskPriority,
};

use core::sync::atomic::{AtomicU32, Ordering};
use heapless::String;

// 系统配置常量
const TASK_STACK_SIZE: u16 = 512;
const QUEUE_SIZE: usize = 10;
const MAX_TASKS: usize = 8;
const SYSTEM_TICK_RATE_HZ: u32 = 1000;

// 全局统计计数器
static TASK_SWITCHES: AtomicU32 = AtomicU32::new(0);
static MESSAGE_COUNT: AtomicU32 = AtomicU32::new(0);
static BUTTON_PRESSES: AtomicU32 = AtomicU32::new(0);

// 任务间通信消息类型
#[derive(Clone, Copy, Debug)]
enum TaskMessage {
  LedControl { led_id: u8, state: bool },
  ButtonEvent { pressed: bool, timestamp: u32 },
  SystemStatus { uptime: u32, free_heap: u32 },
  TaskNotification { task_id: u8, priority: u8 },
  EmergencyStop,
}

// LED控制结构
struct LedController {
  green: PD12<Output<PushPull>>,
  orange: PD13<Output<PushPull>>,
  red: PD14<Output<PushPull>>,
  blue: PD15<Output<PushPull>>,
}

impl LedController {
  fn new(
    green: PD12<Output<PushPull>>,
    orange: PD13<Output<PushPull>>,
    red: PD14<Output<PushPull>>,
    blue: PD15<Output<PushPull>>,
  ) -> Self {
    Self {
      green,
      orange,
      red,
      blue,
    }
  }

  fn set_led(&mut self, led_id: u8, state: bool) {
    match led_id {
      0 => {
        if state {
          self.green.set_high()
        } else {
          self.green.set_low()
        }
      }
      1 => {
        if state {
          self.orange.set_high()
        } else {
          self.orange.set_low()
        }
      }
      2 => {
        if state {
          self.red.set_high()
        } else {
          self.red.set_low()
        }
      }
      3 => {
        if state {
          self.blue.set_high()
        } else {
          self.blue.set_low()
        }
      }
      _ => {}
    }
  }

  fn set_all(&mut self, state: bool) {
    for i in 0..4 {
      self.set_led(i, state);
    }
  }
}

// 系统状态结构
#[derive(Default)]
struct SystemStatus {
  uptime_seconds: u32,
  task_count: u8,
  queue_usage: u8,
  heap_usage: u32,
  cpu_usage: u8,
}

// 全局资源（使用静态变量模拟）
static mut LED_CONTROLLER: Option<LedController> = None;
static mut BUTTON_PIN: Option<PA0<Input>> = None;
static mut SYSTEM_STATUS: SystemStatus = SystemStatus {
  uptime_seconds: 0,
  task_count: 0,
  queue_usage: 0,
  heap_usage: 0,
  cpu_usage: 0,
};

#[entry]
fn main() -> ! {
  // 初始化RTT调试输出
  rtt_init_print!();
  rprintln!("\n=== FreeRTOS基础示例启动 ===");
  rprintln!("功能: 多任务管理和同步机制");
  rprintln!("硬件: STM32F407VG Discovery\n");

  // 获取外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(8.MHz())
    .sysclk(168.MHz())
    .pclk1(42.MHz())
    .pclk2(84.MHz())
    .freeze();

  rprintln!("时钟配置:");
  rprintln!("  SYSCLK: {} MHz", clocks.sysclk().to_MHz());
  rprintln!("  PCLK1:  {} MHz", clocks.pclk1().to_MHz());
  rprintln!("  PCLK2:  {} MHz", clocks.pclk2().to_MHz());

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiod = dp.GPIOD.split();

  // 配置LED引脚
  let led_green = gpiod.pd12.into_push_pull_output();
  let led_orange = gpiod.pd13.into_push_pull_output();
  let led_red = gpiod.pd14.into_push_pull_output();
  let led_blue = gpiod.pd15.into_push_pull_output();

  // 配置按键引脚
  let button = gpioa.pa0.into_pull_up_input();

  rprintln!("\nGPIO配置:");
  rprintln!("  PD12: 绿色LED");
  rprintln!("  PD13: 橙色LED");
  rprintln!("  PD14: 红色LED");
  rprintln!("  PD15: 蓝色LED");
  rprintln!("  PA0:  用户按键 (上拉输入)");

  // 初始化全局资源
  unsafe {
    LED_CONTROLLER = Some(LedController::new(led_green, led_orange, led_red, led_blue));
    BUTTON_PIN = Some(button);
  }

  // 创建FreeRTOS同步对象
  let message_queue = Queue::<TaskMessage>::new(QUEUE_SIZE).unwrap();
  let led_mutex = Mutex::new(()).unwrap();
  let button_semaphore = BinarySemaphore::new().unwrap();
  let system_semaphore = CountingSemaphore::new(MAX_TASKS, 0).unwrap();

  rprintln!("\n同步对象创建:");
  rprintln!("  消息队列: {} 项", QUEUE_SIZE);
  rprintln!("  LED互斥锁: 已创建");
  rprintln!("  按键信号量: 已创建");
  rprintln!("  系统计数信号量: {} 最大计数", MAX_TASKS);

  // 创建任务
  rprintln!("\n创建任务:");

  // 1. LED控制任务 - 高优先级
  let led_task_handle = Task::new()
    .name("LED_Task")
    .stack_size(TASK_STACK_SIZE)
    .priority(TaskPriority(5))
    .start(move || {
      led_control_task(message_queue.clone(), led_mutex.clone());
    })
    .unwrap();
  rprintln!("  LED控制任务: 优先级 5");

  // 2. 按键监控任务 - 中等优先级
  let button_task_handle = Task::new()
    .name("Button_Task")
    .stack_size(TASK_STACK_SIZE)
    .priority(TaskPriority(4))
    .start(move || {
      button_monitor_task(message_queue.clone(), button_semaphore.clone());
    })
    .unwrap();
  rprintln!("  按键监控任务: 优先级 4");

  // 3. 系统监控任务 - 中等优先级
  let system_task_handle = Task::new()
    .name("System_Task")
    .stack_size(TASK_STACK_SIZE)
    .priority(TaskPriority(3))
    .start(move || {
      system_monitor_task(message_queue.clone(), system_semaphore.clone());
    })
    .unwrap();
  rprintln!("  系统监控任务: 优先级 3");

  // 4. 消息处理任务 - 中等优先级
  let message_task_handle = Task::new()
    .name("Message_Task")
    .stack_size(TASK_STACK_SIZE)
    .priority(TaskPriority(3))
    .start(move || {
      message_handler_task(message_queue.clone());
    })
    .unwrap();
  rprintln!("  消息处理任务: 优先级 3");

  // 5. 统计任务 - 低优先级
  let stats_task_handle = Task::new()
    .name("Stats_Task")
    .stack_size(TASK_STACK_SIZE)
    .priority(TaskPriority(2))
    .start(move || {
      statistics_task();
    })
    .unwrap();
  rprintln!("  统计任务: 优先级 2");

  // 6. 空闲任务监控 - 最低优先级
  let idle_task_handle = Task::new()
    .name("Idle_Monitor")
    .stack_size(TASK_STACK_SIZE)
    .priority(TaskPriority(1))
    .start(move || {
      idle_monitor_task();
    })
    .unwrap();
  rprintln!("  空闲监控任务: 优先级 1");

  rprintln!("\n所有任务创建完成，启动FreeRTOS调度器...\n");

  // 启动FreeRTOS调度器
  freertos_rust::start_scheduler();

  // 不应该到达这里
  panic!("FreeRTOS调度器意外退出!");
}

// LED控制任务
fn led_control_task(message_queue: Queue<TaskMessage>, led_mutex: Mutex<()>) {
  rprintln!("[LED] LED控制任务启动");

  let mut led_states = [false; 4];
  let mut blink_counter = 0u32;

  loop {
    // 等待消息
    if let Ok(message) = message_queue.receive(Duration::ms(100)) {
      match message {
        TaskMessage::LedControl { led_id, state } => {
          // 获取LED互斥锁
          if let Ok(_guard) = led_mutex.lock(Duration::ms(10)) {
            unsafe {
              if let Some(ref mut controller) = LED_CONTROLLER {
                controller.set_led(led_id, state);
                led_states[led_id as usize] = state;
                rprintln!(
                  "[LED] LED {} 设置为 {}",
                  led_id,
                  if state { "开" } else { "关" }
                );
              }
            }
          }
        }
        TaskMessage::EmergencyStop => {
          rprintln!("[LED] 紧急停止 - 关闭所有LED");
          if let Ok(_guard) = led_mutex.lock(Duration::ms(10)) {
            unsafe {
              if let Some(ref mut controller) = LED_CONTROLLER {
                controller.set_all(false);
                led_states = [false; 4];
              }
            }
          }
        }
        _ => {}
      }
    } else {
      // 超时 - 执行LED闪烁逻辑
      blink_counter = blink_counter.wrapping_add(1);

      if blink_counter % 10 == 0 {
        // 每秒切换一次LED状态（假设100ms周期）
        if let Ok(_guard) = led_mutex.lock(Duration::ms(5)) {
          unsafe {
            if let Some(ref mut controller) = LED_CONTROLLER {
              let new_state = (blink_counter / 10) % 2 == 0;
              controller.set_led(0, new_state); // 绿色LED心跳
            }
          }
        }
      }
    }

    TASK_SWITCHES.fetch_add(1, Ordering::Relaxed);
  }
}

// 按键监控任务
fn button_monitor_task(message_queue: Queue<TaskMessage>, button_semaphore: BinarySemaphore) {
  rprintln!("[BTN] 按键监控任务启动");

  let mut last_state = false;
  let mut debounce_counter = 0u32;

  loop {
    let current_state = unsafe { BUTTON_PIN.as_ref().map_or(false, |pin| pin.is_low()) };

    if current_state != last_state {
      debounce_counter += 1;
      if debounce_counter >= 5 {
        // 50ms消抖（假设10ms周期）
        let timestamp = unsafe { SYSTEM_STATUS.uptime_seconds };

        let message = TaskMessage::ButtonEvent {
          pressed: current_state,
          timestamp,
        };

        if message_queue.send(message, Duration::ms(10)).is_ok() {
          rprintln!(
            "[BTN] 按键 {} - 时间戳: {}",
            if current_state { "按下" } else { "释放" },
            timestamp
          );

          if current_state {
            BUTTON_PRESSES.fetch_add(1, Ordering::Relaxed);
            button_semaphore.give(); // 释放信号量
          }
        }

        last_state = current_state;
        debounce_counter = 0;
      }
    } else {
      debounce_counter = 0;
    }

    CurrentTask::delay(Duration::ms(10));
    TASK_SWITCHES.fetch_add(1, Ordering::Relaxed);
  }
}

// 系统监控任务
fn system_monitor_task(message_queue: Queue<TaskMessage>, system_semaphore: CountingSemaphore) {
  rprintln!("[SYS] 系统监控任务启动");

  let mut tick_counter = 0u32;

  loop {
    tick_counter += 1;

    // 每秒更新系统状态
    if tick_counter % 100 == 0 {
      // 假设10ms周期
      unsafe {
        SYSTEM_STATUS.uptime_seconds += 1;
        SYSTEM_STATUS.task_count = 6; // 固定任务数
        SYSTEM_STATUS.queue_usage = 50; // 模拟队列使用率
        SYSTEM_STATUS.heap_usage = 1024; // 模拟堆使用量
        SYSTEM_STATUS.cpu_usage = 75; // 模拟CPU使用率
      }

      let status_message = TaskMessage::SystemStatus {
        uptime: unsafe { SYSTEM_STATUS.uptime_seconds },
        free_heap: 32768 - unsafe { SYSTEM_STATUS.heap_usage },
      };

      let _ = message_queue.send(status_message, Duration::ms(5));
    }

    // 每5秒发送任务通知
    if tick_counter % 500 == 0 {
      for task_id in 0..6 {
        let notification = TaskMessage::TaskNotification {
          task_id,
          priority: task_id + 1,
        };
        let _ = message_queue.send(notification, Duration::ms(1));
        system_semaphore.give();
      }
    }

    CurrentTask::delay(Duration::ms(10));
    TASK_SWITCHES.fetch_add(1, Ordering::Relaxed);
  }
}

// 消息处理任务
fn message_handler_task(message_queue: Queue<TaskMessage>) {
  rprintln!("[MSG] 消息处理任务启动");

  loop {
    if let Ok(message) = message_queue.receive(Duration::ms(50)) {
      MESSAGE_COUNT.fetch_add(1, Ordering::Relaxed);

      match message {
        TaskMessage::ButtonEvent { pressed, timestamp } => {
          rprintln!(
            "[MSG] 处理按键事件: {} @ {}",
            if pressed { "按下" } else { "释放" },
            timestamp
          );

          if pressed {
            // 按键按下时切换LED状态
            let led_message = TaskMessage::LedControl {
              led_id: (timestamp % 4) as u8,
              state: true,
            };
            let _ = message_queue.send(led_message, Duration::ms(5));
          }
        }
        TaskMessage::SystemStatus { uptime, free_heap } => {
          if uptime % 10 == 0 {
            // 每10秒报告一次
            rprintln!(
              "[MSG] 系统状态: 运行时间 {}s, 可用堆 {} 字节",
              uptime,
              free_heap
            );
          }
        }
        TaskMessage::TaskNotification { task_id, priority } => {
          rprintln!("[MSG] 任务通知: ID={}, 优先级={}", task_id, priority);
        }
        TaskMessage::EmergencyStop => {
          rprintln!("[MSG] 处理紧急停止请求");
        }
        _ => {}
      }
    }

    TASK_SWITCHES.fetch_add(1, Ordering::Relaxed);
  }
}

// 统计任务
fn statistics_task() {
  rprintln!("[STAT] 统计任务启动");

  loop {
    CurrentTask::delay(Duration::ms(5000)); // 每5秒报告一次

    let task_switches = TASK_SWITCHES.load(Ordering::Relaxed);
    let message_count = MESSAGE_COUNT.load(Ordering::Relaxed);
    let button_presses = BUTTON_PRESSES.load(Ordering::Relaxed);

    let uptime = unsafe { SYSTEM_STATUS.uptime_seconds };
    let heap_usage = unsafe { SYSTEM_STATUS.heap_usage };
    let cpu_usage = unsafe { SYSTEM_STATUS.cpu_usage };

    rprintln!("\n=== 系统统计报告 ===");
    rprintln!("  运行时间: {} 秒", uptime);
    rprintln!("  任务切换次数: {}", task_switches);
    rprintln!("  消息处理次数: {}", message_count);
    rprintln!("  按键按下次数: {}", button_presses);
    rprintln!("  堆使用量: {} 字节", heap_usage);
    rprintln!("  CPU使用率: {}%", cpu_usage);

    if uptime > 0 {
      rprintln!("  平均任务切换频率: {} 次/秒", task_switches / uptime);
      rprintln!("  平均消息频率: {} 次/秒", message_count / uptime);
    }
    rprintln!("");

    TASK_SWITCHES.fetch_add(1, Ordering::Relaxed);
  }
}

// 空闲监控任务
fn idle_monitor_task() {
  rprintln!("[IDLE] 空闲监控任务启动");

  let mut idle_counter = 0u32;

  loop {
    idle_counter += 1;

    // 每分钟报告一次空闲状态
    if idle_counter % 6000 == 0 {
      // 假设10ms周期
      rprintln!("[IDLE] 空闲监控: 计数器 = {}", idle_counter);
    }

    CurrentTask::delay(Duration::ms(10));
    TASK_SWITCHES.fetch_add(1, Ordering::Relaxed);
  }
}

// FreeRTOS钩子函数
#[no_mangle]
extern "C" fn vApplicationStackOverflowHook(
  task_handle: *mut freertos_sys::TaskHandle_t,
  task_name: *const i8,
) {
  rprintln!("[ERROR] 栈溢出检测: 任务 {:?}", task_name);
  panic!("栈溢出!");
}

#[no_mangle]
extern "C" fn vApplicationMallocFailedHook() {
  rprintln!("[ERROR] 内存分配失败");
  panic!("内存分配失败!");
}

#[no_mangle]
extern "C" fn vApplicationIdleHook() {
  // 空闲钩子 - 可以在这里执行低功耗操作
  cortex_m::asm::wfi();
}

#[no_mangle]
extern "C" fn vApplicationTickHook() {
  // 系统滴答钩子 - 每个系统滴答调用一次
  // 可以在这里执行时间相关的操作
}

#[no_mangle]
extern "C" fn vApplicationDaemonTaskStartupHook() {
  rprintln!("[HOOK] FreeRTOS守护任务启动");
}
