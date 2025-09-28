#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm;
use stm32f4xx_hal::{
  gpio::{gpioa::PA5, gpioc::PC13, Input, Output, PullUp, PushPull},
  pac::{Peripherals, TIM2, TIM3},
  prelude::*,
  timer::{CounterUs, Event},
};

use heapless::{
  pool::{Node, Pool},
  spsc::{Consumer, Producer, Queue},
  Vec,
};
use rtic::app;
use rtic_monotonics::{systick::*, Monotonic};

// 消息类型定义
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MessageType {
  SensorData,
  UserInput,
  SystemEvent,
  NetworkPacket,
  ErrorReport,
}

#[derive(Clone, Copy, Debug)]
pub struct Message {
  pub msg_type: MessageType,
  pub priority: u8,
  pub timestamp: u32,
  pub data: [u8; 8],
}

impl Message {
  pub fn new(msg_type: MessageType, priority: u8, data: [u8; 8]) -> Self {
    Self {
      msg_type,
      priority,
      timestamp: 0, // 将在发送时设置
      data,
    }
  }
}

// 传感器数据结构
#[derive(Clone, Copy, Debug)]
pub struct SensorReading {
  pub sensor_id: u8,
  pub value: u16,
  pub status: u8,
}

// 事件类型
#[derive(Clone, Copy, Debug)]
pub enum SystemEvent {
  ButtonPressed,
  TimerExpired,
  DataReady,
  ErrorOccurred(u8),
  NetworkConnected,
  NetworkDisconnected,
}

// 统计信息
#[derive(Clone, Copy, Debug, Default)]
pub struct Statistics {
  pub messages_sent: u32,
  pub messages_received: u32,
  pub errors_count: u32,
  pub uptime_seconds: u32,
}

// 内存池配置
const POOL_SIZE: usize = 32;
static mut MESSAGE_MEMORY: [Node<Message>; POOL_SIZE] = [Node::new(); POOL_SIZE];

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4])]
mod app {
  use super::*;

  #[shared]
  struct Shared {
    statistics: Statistics,
    message_pool: Pool<Message>,
    system_time: u32,
  }

  #[local]
  struct Local {
    led: PA5<Output<PushPull>>,
    button: PC13<Input<PullUp>>,
    timer2: CounterUs<TIM2>,
    timer3: CounterUs<TIM3>,
    high_priority_queue: Queue<Message, 16>,
    low_priority_queue: Queue<Message, 32>,
    sensor_data_buffer: Vec<SensorReading, 16>,
  }

  #[init]
  fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
    // 初始化系统时钟
    let rcc = ctx.device.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

    // 初始化GPIO
    let gpioa = ctx.device.GPIOA.split();
    let gpioc = ctx.device.GPIOC.split();

    let led = gpioa.pa5.into_push_pull_output();
    let button = gpioc.pc13.into_pull_up_input();

    // 初始化定时器
    let mut timer2 = ctx.device.TIM2.counter_us(&clocks);
    timer2.start(100.millis()).unwrap(); // 高频定时器
    timer2.listen(Event::Update);

    let mut timer3 = ctx.device.TIM3.counter_us(&clocks);
    timer3.start(1.secs()).unwrap(); // 低频定时器
    timer3.listen(Event::Update);

    // 初始化SysTick单调时钟
    let systick_token = rtic_monotonics::create_systick_token!();
    Systick::start(ctx.core.SYST, 84_000_000, systick_token);

    // 初始化内存池和队列
    let message_pool = Pool::new(unsafe { &mut MESSAGE_MEMORY });
    let high_priority_queue = Queue::new();
    let low_priority_queue = Queue::new();
    let sensor_data_buffer = Vec::new();

    // 启动系统任务
    system_monitor::spawn().ok();
    message_dispatcher::spawn().ok();

    cortex_m_log::println!("Task Communication System Initialized");

    (
      Shared {
        statistics: Statistics::default(),
        message_pool,
        system_time: 0,
      },
      Local {
        led,
        button,
        timer2,
        timer3,
        high_priority_queue,
        low_priority_queue,
        sensor_data_buffer,
      },
      init::Monotonics(Systick::new(ctx.core.SYST, 84_000_000)),
    )
  }

  // 空闲任务
  #[idle(shared = [statistics])]
  fn idle(mut ctx: idle::Context) -> ! {
    loop {
      ctx.shared.statistics.lock(|stats| {
        // 在空闲时可以执行一些低优先级的后台任务
        asm::wfi(); // 等待中断
      });
    }
  }

  // 高频定时器中断 - 传感器数据采集
  #[task(binds = TIM2, local = [timer2], shared = [system_time], priority = 3)]
  fn sensor_timer(mut ctx: sensor_timer::Context) {
    ctx.local.timer2.clear_interrupt(Event::Update);

    ctx.shared.system_time.lock(|time| {
      *time = time.wrapping_add(1);
    });

    // 模拟传感器数据
    let sensor_data = [
      (*ctx.shared.system_time.lock(|t| *t) & 0xFF) as u8,
      0x12,
      0x34,
      0x56,
      0x78,
      0x9A,
      0xBC,
      0xDE,
    ];

    let msg = Message::new(MessageType::SensorData, 2, sensor_data);
    message_producer::spawn(msg).ok();
  }

  // 低频定时器中断 - 系统事件
  #[task(binds = TIM3, local = [timer3], shared = [system_time, statistics], priority = 2)]
  fn system_timer(mut ctx: system_timer::Context) {
    ctx.local.timer3.clear_interrupt(Event::Update);

    ctx.shared.statistics.lock(|stats| {
      stats.uptime_seconds = stats.uptime_seconds.wrapping_add(1);
    });

    // 发送系统心跳事件
    let heartbeat_data = [0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    let msg = Message::new(MessageType::SystemEvent, 1, heartbeat_data);
    message_producer::spawn(msg).ok();
  }

  // 按钮中断处理
  #[task(binds = EXTI15_10, local = [button], shared = [statistics], priority = 4)]
  fn button_interrupt(mut ctx: button_interrupt::Context) {
    if ctx.local.button.is_low() {
      ctx.shared.statistics.lock(|stats| {
        stats.messages_sent = stats.messages_sent.wrapping_add(1);
      });

      // 发送用户输入消息
      let user_data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
      let msg = Message::new(MessageType::UserInput, 3, user_data);
      message_producer::spawn(msg).ok();

      cortex_m_log::println!("Button pressed - User input message sent");
    }
  }

  // 消息生产者任务
  #[task(local = [high_priority_queue, low_priority_queue], shared = [system_time, statistics], priority = 2)]
  fn message_producer(mut ctx: message_producer::Context, mut msg: Message) {
    // 设置时间戳
    ctx.shared.system_time.lock(|time| {
      msg.timestamp = *time;
    });

    // 根据优先级选择队列
    let result = if msg.priority >= 3 {
      ctx.local.high_priority_queue.enqueue(msg)
    } else {
      ctx.local.low_priority_queue.enqueue(msg)
    };

    match result {
      Ok(_) => {
        ctx.shared.statistics.lock(|stats| {
          stats.messages_sent = stats.messages_sent.wrapping_add(1);
        });

        // 通知消息调度器
        message_dispatcher::spawn().ok();
      }
      Err(_) => {
        ctx.shared.statistics.lock(|stats| {
          stats.errors_count = stats.errors_count.wrapping_add(1);
        });
        cortex_m_log::println!("Queue full - message dropped");
      }
    }
  }

  // 消息调度器任务
  #[task(local = [high_priority_queue, low_priority_queue], shared = [statistics], priority = 1)]
  fn message_dispatcher(mut ctx: message_dispatcher::Context) {
    // 优先处理高优先级队列
    while let Some(msg) = ctx.local.high_priority_queue.dequeue() {
      message_consumer::spawn(msg).ok();
    }

    // 处理低优先级队列
    if let Some(msg) = ctx.local.low_priority_queue.dequeue() {
      message_consumer::spawn(msg).ok();
    }
  }

  // 消息消费者任务
  #[task(shared = [statistics], priority = 1)]
  fn message_consumer(mut ctx: message_consumer::Context, msg: Message) {
    ctx.shared.statistics.lock(|stats| {
      stats.messages_received = stats.messages_received.wrapping_add(1);
    });

    match msg.msg_type {
      MessageType::SensorData => {
        sensor_data_processor::spawn(msg).ok();
      }
      MessageType::UserInput => {
        user_input_handler::spawn(msg).ok();
      }
      MessageType::SystemEvent => {
        system_event_handler::spawn(msg).ok();
      }
      MessageType::NetworkPacket => {
        network_packet_handler::spawn(msg).ok();
      }
      MessageType::ErrorReport => {
        error_handler::spawn(msg).ok();
      }
    }
  }

  // 传感器数据处理任务
  #[task(local = [sensor_data_buffer], priority = 1)]
  fn sensor_data_processor(ctx: sensor_data_processor::Context, msg: Message) {
    let sensor_reading = SensorReading {
      sensor_id: msg.data[0],
      value: ((msg.data[1] as u16) << 8) | (msg.data[2] as u16),
      status: msg.data[3],
    };

    // 添加到缓冲区
    if ctx.local.sensor_data_buffer.push(sensor_reading).is_err() {
      cortex_m_log::println!("Sensor buffer full");
      // 移除最老的数据
      ctx.local.sensor_data_buffer.remove(0);
      ctx.local.sensor_data_buffer.push(sensor_reading).ok();
    }

    cortex_m_log::println!(
      "Sensor data: ID={}, Value={}, Status={}",
      sensor_reading.sensor_id,
      sensor_reading.value,
      sensor_reading.status
    );

    // 如果检测到异常值，发送错误报告
    if sensor_reading.value > 1000 {
      let error_data = [
        0xFF,
        sensor_reading.sensor_id,
        0x01,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
      ];
      let error_msg = Message::new(MessageType::ErrorReport, 4, error_data);
      message_producer::spawn(error_msg).ok();
    }
  }

  // 用户输入处理任务
  #[task(local = [led], priority = 1)]
  fn user_input_handler(ctx: user_input_handler::Context, msg: Message) {
    match msg.data[0] {
      0x01 => {
        // 切换LED状态
        ctx.local.led.toggle();
        cortex_m_log::println!("LED toggled by user input");
      }
      _ => {
        cortex_m_log::println!("Unknown user input: 0x{:02X}", msg.data[0]);
      }
    }
  }

  // 系统事件处理任务
  #[task(priority = 1)]
  fn system_event_handler(_ctx: system_event_handler::Context, msg: Message) {
    match msg.data[0] {
      0xAA => {
        // 心跳事件
        cortex_m_log::println!("System heartbeat at timestamp: {}", msg.timestamp);
      }
      _ => {
        cortex_m_log::println!("System event: 0x{:02X}", msg.data[0]);
      }
    }
  }

  // 网络数据包处理任务
  #[task(priority = 1)]
  fn network_packet_handler(_ctx: network_packet_handler::Context, msg: Message) {
    cortex_m_log::println!("Processing network packet, size: {}", msg.data.len());

    // 模拟网络数据包处理
    let checksum: u16 = msg.data.iter().map(|&x| x as u16).sum();
    cortex_m_log::println!("Packet checksum: 0x{:04X}", checksum);
  }

  // 错误处理任务
  #[task(shared = [statistics], priority = 4)]
  fn error_handler(mut ctx: error_handler::Context, msg: Message) {
    ctx.shared.statistics.lock(|stats| {
      stats.errors_count = stats.errors_count.wrapping_add(1);
    });

    let error_code = msg.data[0];
    let error_source = msg.data[1];

    cortex_m_log::println!(
      "ERROR: Code=0x{:02X}, Source=0x{:02X}, Timestamp={}",
      error_code,
      error_source,
      msg.timestamp
    );

    // 根据错误类型采取相应措施
    match error_code {
      0xFF => {
        // 严重错误，需要立即处理
        emergency_response::spawn().ok();
      }
      _ => {
        // 一般错误，记录日志
        cortex_m_log::println!("Error logged and handled");
      }
    }
  }

  // 紧急响应任务
  #[task(local = [led], priority = 5)]
  fn emergency_response(ctx: emergency_response::Context) {
    cortex_m_log::println!("EMERGENCY RESPONSE ACTIVATED");

    // 快速闪烁LED表示紧急状态
    for _ in 0..10 {
      ctx.local.led.set_high();
      // 在实际应用中需要适当的延时
      ctx.local.led.set_low();
    }
  }

  // 系统监控任务
  #[task(shared = [statistics], priority = 1)]
  fn system_monitor(mut ctx: system_monitor::Context) {
    ctx.shared.statistics.lock(|stats| {
      cortex_m_log::println!(
        "System Stats - Sent: {}, Received: {}, Errors: {}, Uptime: {}s",
        stats.messages_sent,
        stats.messages_received,
        stats.errors_count,
        stats.uptime_seconds
      );
    });

    // 每10秒重新调度
    system_monitor::spawn_after(10.secs()).ok();
  }

  // 数据分析任务
  #[task(local = [sensor_data_buffer], priority = 1)]
  fn data_analyzer(ctx: data_analyzer::Context) {
    if !ctx.local.sensor_data_buffer.is_empty() {
      let avg_value: u32 = ctx
        .local
        .sensor_data_buffer
        .iter()
        .map(|reading| reading.value as u32)
        .sum::<u32>()
        / ctx.local.sensor_data_buffer.len() as u32;

      cortex_m_log::println!(
        "Data Analysis - Samples: {}, Average: {}",
        ctx.local.sensor_data_buffer.len(),
        avg_value
      );

      // 清空缓冲区为下一轮分析做准备
      ctx.local.sensor_data_buffer.clear();
    }

    // 每30秒重新调度
    data_analyzer::spawn_after(30.secs()).ok();
  }

  // 通信测试任务
  #[task(priority = 1)]
  fn communication_test(_ctx: communication_test::Context) {
    // 发送测试消息到各个队列
    let test_messages = [
      Message::new(
        MessageType::NetworkPacket,
        2,
        [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
      ),
      Message::new(
        MessageType::SystemEvent,
        1,
        [0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
      ),
      Message::new(
        MessageType::SensorData,
        2,
        [0xFF, 0x03, 0xE8, 0x01, 0x00, 0x00, 0x00, 0x00],
      ),
    ];

    for msg in test_messages.iter() {
      message_producer::spawn(*msg).ok();
    }

    cortex_m_log::println!("Communication test messages sent");

    // 每60秒重新调度
    communication_test::spawn_after(60.secs()).ok();
  }
}

// 测试函数
#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_message_creation() {
    let data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    let msg = Message::new(MessageType::SensorData, 2, data);

    assert_eq!(msg.msg_type, MessageType::SensorData);
    assert_eq!(msg.priority, 2);
    assert_eq!(msg.data, data);
  }

  #[test]
  fn test_sensor_reading() {
    let reading = SensorReading {
      sensor_id: 1,
      value: 1024,
      status: 0,
    };

    assert_eq!(reading.sensor_id, 1);
    assert_eq!(reading.value, 1024);
    assert_eq!(reading.status, 0);
  }

  #[test]
  fn test_statistics() {
    let mut stats = Statistics::default();
    stats.messages_sent += 1;
    stats.messages_received += 1;

    assert_eq!(stats.messages_sent, 1);
    assert_eq!(stats.messages_received, 1);
    assert_eq!(stats.errors_count, 0);
  }
}
