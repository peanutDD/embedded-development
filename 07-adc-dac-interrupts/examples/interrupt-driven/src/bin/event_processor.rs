#![no_std]
#![no_main]

//! # 事件处理器演示程序
//!
//! 专门展示事件驱动架构的实现，包括：
//! - 多优先级事件队列
//! - 事件分发和路由
//! - 负载均衡处理
//! - 性能监控和统计

use defmt_rtt as _;
use panic_probe as _;

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;

use hal::{
  gpio::{Input, Output, PullUp, PushPull},
  pac::{interrupt, Interrupt, NVIC},
  prelude::*,
  timer::{Event, Timer},
};

use cortex_m::{
  interrupt::{free, Mutex},
  peripheral::NVIC as CortexNVIC,
};
use cortex_m_rt::entry;

use interrupt_driven::{EdgeType, EventProcessor, Priority, ProcessorConfig, SystemEvent};

use core::cell::RefCell;
use heapless::Vec;

/// 事件处理器配置
#[derive(Debug)]
struct EventProcessorConfig {
  /// 队列大小
  queue_size: usize,
  /// 批处理大小
  batch_size: usize,
  /// 处理间隔 (ms)
  processing_interval: u32,
  /// 统计报告间隔
  stats_interval: u32,
}

impl Default for EventProcessorConfig {
  fn default() -> Self {
    Self {
      queue_size: 64,
      batch_size: 8,
      processing_interval: 10,
      stats_interval: 5000,
    }
  }
}

/// 事件生成器
struct EventGenerator {
  /// 事件计数器
  counter: u32,
  /// 生成模式
  mode: GeneratorMode,
  /// 配置参数
  config: GeneratorConfig,
}

/// 生成模式
#[derive(Debug, Clone, Copy)]
enum GeneratorMode {
  Sequential,
  Random,
  Burst,
  Mixed,
}

/// 生成器配置
#[derive(Debug)]
struct GeneratorConfig {
  gpio_event_rate: u32,
  timer_event_rate: u32,
  uart_event_rate: u32,
  adc_event_rate: u32,
}

impl Default for GeneratorConfig {
  fn default() -> Self {
    Self {
      gpio_event_rate: 100,
      timer_event_rate: 50,
      uart_event_rate: 200,
      adc_event_rate: 25,
    }
  }
}

impl EventGenerator {
  /// 创建新的事件生成器
  fn new(mode: GeneratorMode, config: GeneratorConfig) -> Self {
    Self {
      counter: 0,
      mode,
      config,
    }
  }

  /// 生成下一个事件
  fn generate_next_event(&mut self) -> Option<(SystemEvent, Priority)> {
    self.counter += 1;

    match self.mode {
      GeneratorMode::Sequential => self.generate_sequential_event(),
      GeneratorMode::Random => self.generate_random_event(),
      GeneratorMode::Burst => self.generate_burst_event(),
      GeneratorMode::Mixed => self.generate_mixed_event(),
    }
  }

  /// 生成顺序事件
  fn generate_sequential_event(&self) -> Option<(SystemEvent, Priority)> {
    let event_type = self.counter % 4;

    match event_type {
      0 => Some((
        SystemEvent::GpioInterrupt {
          pin: (self.counter % 8) as u8,
          edge: EdgeType::Rising,
        },
        Priority::High,
      )),
      1 => Some((
        SystemEvent::TimerInterrupt {
          timer_id: (self.counter % 4) as u8,
        },
        Priority::Medium,
      )),
      2 => Some((
        SystemEvent::UartReceive {
          data: (self.counter % 256) as u8,
        },
        Priority::High,
      )),
      3 => Some((
        SystemEvent::AdcComplete {
          channel: (self.counter % 8) as u8,
          value: (self.counter % 4096) as u16,
        },
        Priority::Low,
      )),
      _ => None,
    }
  }

  /// 生成随机事件
  fn generate_random_event(&self) -> Option<(SystemEvent, Priority)> {
    // 简单的伪随机数生成
    let rand = (self.counter * 1103515245 + 12345) % (1 << 31);
    let event_type = rand % 4;

    match event_type {
      0 => Some((
        SystemEvent::GpioInterrupt {
          pin: (rand % 16) as u8,
          edge: if rand % 2 == 0 {
            EdgeType::Rising
          } else {
            EdgeType::Falling
          },
        },
        if rand % 3 == 0 {
          Priority::Critical
        } else {
          Priority::High
        },
      )),
      1 => Some((
        SystemEvent::TimerInterrupt {
          timer_id: (rand % 8) as u8,
        },
        Priority::Medium,
      )),
      2 => Some((
        SystemEvent::UartReceive {
          data: (rand % 256) as u8,
        },
        Priority::High,
      )),
      3 => Some((
        SystemEvent::UserEvent {
          id: (rand % 1000) as u16,
          data: rand,
        },
        Priority::Low,
      )),
      _ => None,
    }
  }

  /// 生成突发事件
  fn generate_burst_event(&self) -> Option<(SystemEvent, Priority)> {
    // 每100个事件产生一次突发
    if self.counter % 100 < 10 {
      Some((
        SystemEvent::GpioInterrupt {
          pin: 0,
          edge: EdgeType::Both,
        },
        Priority::Critical,
      ))
    } else {
      self.generate_sequential_event()
    }
  }

  /// 生成混合事件
  fn generate_mixed_event(&self) -> Option<(SystemEvent, Priority)> {
    let pattern = self.counter % 20;

    if pattern < 5 {
      self.generate_sequential_event()
    } else if pattern < 15 {
      self.generate_random_event()
    } else {
      self.generate_burst_event()
    }
  }
}

/// 事件处理器控制器
struct EventProcessorController<LED> {
  /// 状态LED
  status_led: LED,
  /// 事件处理器
  processor: EventProcessor<64>,
  /// 事件生成器
  generator: EventGenerator,
  /// 配置参数
  config: EventProcessorConfig,
  /// 统计计数器
  stats: ProcessingStats,
  /// 处理模式
  processing_mode: ProcessingMode,
}

/// 处理统计
#[derive(Debug, Default)]
struct ProcessingStats {
  total_generated: u32,
  total_processed: u32,
  total_dropped: u32,
  processing_cycles: u32,
  max_queue_depth: usize,
  avg_processing_time: u32,
}

/// 处理模式
#[derive(Debug, Clone, Copy)]
enum ProcessingMode {
  RealTime,
  Batch,
  Adaptive,
  LoadBalanced,
}

// 全局静态变量
static EVENT_CONTROLLER: Mutex<
  RefCell<Option<EventProcessorController<hal::gpio::Pin<Output<PushPull>>>>>,
> = Mutex::new(RefCell::new(None));

static TIMER: Mutex<RefCell<Option<Timer<hal::pac::TIM2>>>> = Mutex::new(RefCell::new(None));

impl<LED> EventProcessorController<LED>
where
  LED: embedded_hal::digital::OutputPin,
{
  /// 创建新的控制器
  fn new(status_led: LED, generator_mode: GeneratorMode, processing_mode: ProcessingMode) -> Self {
    let processor_config = ProcessorConfig {
      max_queue_size: 64,
      enable_stats: true,
      timeout_ms: 100,
    };

    let generator_config = GeneratorConfig::default();
    let config = EventProcessorConfig::default();

    Self {
      status_led,
      processor: EventProcessor::new(processor_config),
      generator: EventGenerator::new(generator_mode, generator_config),
      config,
      stats: ProcessingStats::default(),
      processing_mode,
    }
  }

  /// 初始化系统
  fn initialize(&mut self) -> Result<(), ()> {
    defmt::info!("初始化事件处理器系统...");

    let _ = self.status_led.set_low();

    defmt::info!("事件处理器初始化完成");
    defmt::info!("处理模式: {:?}", self.processing_mode);
    defmt::info!("队列大小: {}", self.config.queue_size);
    defmt::info!("批处理大小: {}", self.config.batch_size);

    Ok(())
  }

  /// 运行事件处理循环
  fn run_processing_loop(&mut self) {
    loop {
      // 生成事件
      self.generate_events();

      // 处理事件
      match self.processing_mode {
        ProcessingMode::RealTime => self.process_realtime(),
        ProcessingMode::Batch => self.process_batch(),
        ProcessingMode::Adaptive => self.process_adaptive(),
        ProcessingMode::LoadBalanced => self.process_load_balanced(),
      }

      // 更新统计
      self.update_statistics();

      // 监控系统状态
      self.monitor_system();

      // 延时
      self.delay_ms(self.config.processing_interval);
    }
  }

  /// 生成事件
  fn generate_events(&mut self) {
    // 根据配置生成多个事件
    let events_to_generate = match self.processing_mode {
      ProcessingMode::RealTime => 1,
      ProcessingMode::Batch => self.config.batch_size / 2,
      ProcessingMode::Adaptive => self.calculate_adaptive_rate(),
      ProcessingMode::LoadBalanced => self.calculate_balanced_rate(),
    };

    for _ in 0..events_to_generate {
      if let Some((event, priority)) = self.generator.generate_next_event() {
        match self.processor.push_event(event, priority) {
          Ok(_) => {
            self.stats.total_generated += 1;
          }
          Err(_) => {
            self.stats.total_dropped += 1;
            defmt::warn!("事件队列已满，丢弃事件");
          }
        }
      }
    }
  }

  /// 实时处理模式
  fn process_realtime(&mut self) {
    if let Some(event) = self.processor.process_next_event() {
      self.handle_event(event);
      self.stats.total_processed += 1;
    }
  }

  /// 批处理模式
  fn process_batch(&mut self) {
    let mut processed_count = 0;

    while processed_count < self.config.batch_size {
      if let Some(event) = self.processor.process_next_event() {
        self.handle_event(event);
        self.stats.total_processed += 1;
        processed_count += 1;
      } else {
        break;
      }
    }

    if processed_count > 0 {
      defmt::debug!("批处理完成: {} 个事件", processed_count);
    }
  }

  /// 自适应处理模式
  fn process_adaptive(&mut self) {
    let queue_length = self.processor.queue_length();
    let processing_rate = if queue_length > 32 {
      self.config.batch_size * 2 // 高负载时加速处理
    } else if queue_length > 16 {
      self.config.batch_size // 中等负载正常处理
    } else {
      self.config.batch_size / 2 // 低负载时减速处理
    };

    let mut processed_count = 0;
    while processed_count < processing_rate {
      if let Some(event) = self.processor.process_next_event() {
        self.handle_event(event);
        self.stats.total_processed += 1;
        processed_count += 1;
      } else {
        break;
      }
    }
  }

  /// 负载均衡处理模式
  fn process_load_balanced(&mut self) {
    let queue_length = self.processor.queue_length();

    // 根据队列长度动态调整处理策略
    if queue_length > 48 {
      // 紧急模式：快速处理
      self.emergency_processing();
    } else if queue_length > 24 {
      // 加速模式：批量处理
      self.process_batch();
    } else {
      // 正常模式：实时处理
      self.process_realtime();
    }
  }

  /// 紧急处理模式
  fn emergency_processing(&mut self) {
    defmt::warn!(
      "进入紧急处理模式，队列长度: {}",
      self.processor.queue_length()
    );
    let _ = self.status_led.set_high();

    // 快速处理所有事件
    let mut processed_count = 0;
    while let Some(event) = self.processor.process_next_event() {
      self.handle_event_fast(event);
      self.stats.total_processed += 1;
      processed_count += 1;

      if processed_count >= 32 {
        break; // 防止无限循环
      }
    }

    let _ = self.status_led.set_low();
    defmt::info!("紧急处理完成: {} 个事件", processed_count);
  }

  /// 处理单个事件
  fn handle_event(&mut self, event: SystemEvent) {
    match event {
      SystemEvent::GpioInterrupt { pin, edge } => {
        defmt::debug!("处理GPIO中断: pin={}, edge={:?}", pin, edge);
        self.simulate_gpio_processing(pin, edge);
      }
      SystemEvent::TimerInterrupt { timer_id } => {
        defmt::debug!("处理定时器中断: timer_id={}", timer_id);
        self.simulate_timer_processing(timer_id);
      }
      SystemEvent::UartReceive { data } => {
        defmt::debug!("处理UART数据: 0x{:02x}", data);
        self.simulate_uart_processing(data);
      }
      SystemEvent::AdcComplete { channel, value } => {
        defmt::debug!("处理ADC数据: channel={}, value={}", channel, value);
        self.simulate_adc_processing(channel, value);
      }
      SystemEvent::UserEvent { id, data } => {
        defmt::debug!("处理用户事件: id={}, data=0x{:08x}", id, data);
        self.simulate_user_processing(id, data);
      }
    }
  }

  /// 快速处理事件（紧急模式）
  fn handle_event_fast(&mut self, event: SystemEvent) {
    // 简化处理逻辑，只记录关键信息
    match event {
      SystemEvent::GpioInterrupt { pin, .. } => {
        if pin == 0 {
          // 只处理关键GPIO
          self.simulate_gpio_processing(pin, EdgeType::Rising);
        }
      }
      SystemEvent::TimerInterrupt { timer_id } => {
        if timer_id < 2 {
          // 只处理关键定时器
          self.simulate_timer_processing(timer_id);
        }
      }
      _ => {
        // 其他事件跳过
      }
    }
  }

  /// 模拟GPIO处理
  fn simulate_gpio_processing(&mut self, pin: u8, edge: EdgeType) {
    // 模拟处理延时
    for _ in 0..100 {
      cortex_m::asm::nop();
    }

    // 根据引脚执行不同操作
    match pin {
      0 => {
        // 关键GPIO，切换LED状态
        let _ = self.status_led.set_high();
        self.delay_us(100);
        let _ = self.status_led.set_low();
      }
      _ => {
        // 普通GPIO处理
      }
    }
  }

  /// 模拟定时器处理
  fn simulate_timer_processing(&mut self, timer_id: u8) {
    // 模拟处理延时
    for _ in 0..50 {
      cortex_m::asm::nop();
    }

    // 根据定时器ID执行操作
    if timer_id == 0 {
      // 系统定时器，更新统计
      self.stats.processing_cycles += 1;
    }
  }

  /// 模拟UART处理
  fn simulate_uart_processing(&mut self, data: u8) {
    // 模拟处理延时
    for _ in 0..200 {
      cortex_m::asm::nop();
    }

    // 处理特殊命令
    match data {
      0xFF => {
        // 复位命令
        defmt::info!("收到复位命令");
      }
      0xAA => {
        // 状态查询命令
        self.print_status();
      }
      _ => {
        // 普通数据处理
      }
    }
  }

  /// 模拟ADC处理
  fn simulate_adc_processing(&mut self, channel: u8, value: u16) {
    // 模拟处理延时
    for _ in 0..150 {
      cortex_m::asm::nop();
    }

    // 检查阈值
    if value > 3000 {
      defmt::warn!("ADC通道{}值过高: {}", channel, value);
    }
  }

  /// 模拟用户事件处理
  fn simulate_user_processing(&mut self, id: u16, data: u32) {
    // 模拟处理延时
    for _ in 0..300 {
      cortex_m::asm::nop();
    }

    // 处理特定用户事件
    if id == 1000 {
      defmt::info!("处理特殊用户事件: data=0x{:08x}", data);
    }
  }

  /// 计算自适应处理速率
  fn calculate_adaptive_rate(&self) -> usize {
    let queue_length = self.processor.queue_length();
    let base_rate = self.config.batch_size;

    if queue_length > 40 {
      base_rate * 3
    } else if queue_length > 20 {
      base_rate * 2
    } else if queue_length > 10 {
      base_rate
    } else {
      base_rate / 2
    }
  }

  /// 计算负载均衡速率
  fn calculate_balanced_rate(&self) -> usize {
    let queue_length = self.processor.queue_length();
    let processing_cycles = self.stats.processing_cycles;

    // 根据历史负载调整
    let load_factor = if processing_cycles > 0 {
      (self.stats.total_processed / processing_cycles).min(10)
    } else {
      1
    };

    let base_rate = self.config.batch_size;

    if queue_length > 32 {
      base_rate * load_factor as usize
    } else {
      base_rate
    }
  }

  /// 更新统计信息
  fn update_statistics(&mut self) {
    let queue_length = self.processor.queue_length();
    self.stats.max_queue_depth = self.stats.max_queue_depth.max(queue_length);

    // 计算平均处理时间
    if self.stats.total_processed > 0 {
      self.stats.avg_processing_time =
        self.stats.processing_cycles * 1000 / self.stats.total_processed;
    }
  }

  /// 监控系统状态
  fn monitor_system(&mut self) {
    let queue_length = self.processor.queue_length();

    // 队列长度告警
    if queue_length > 50 {
      defmt::warn!("事件队列接近满载: {}/64", queue_length);
    }

    // 定期输出统计信息
    if self.stats.processing_cycles % (self.config.stats_interval / self.config.processing_interval)
      == 0
      && self.stats.processing_cycles > 0
    {
      self.print_statistics();
    }
  }

  /// 输出统计信息
  fn print_statistics(&self) {
    let processor_stats = self.processor.get_stats();

    defmt::info!("=== 事件处理器统计 ===");
    defmt::info!("生成事件: {}", self.stats.total_generated);
    defmt::info!("处理事件: {}", self.stats.total_processed);
    defmt::info!("丢弃事件: {}", self.stats.total_dropped);
    defmt::info!("处理周期: {}", self.stats.processing_cycles);
    defmt::info!("最大队列深度: {}", self.stats.max_queue_depth);
    defmt::info!("当前队列长度: {}", self.processor.queue_length());
    defmt::info!("平均处理时间: {} μs", self.stats.avg_processing_time);
    defmt::info!(
      "处理效率: {:.1}%",
      if self.stats.total_generated > 0 {
        (self.stats.total_processed * 100) as f32 / self.stats.total_generated as f32
      } else {
        0.0
      }
    );
  }

  /// 输出状态信息
  fn print_status(&self) {
    defmt::info!("=== 系统状态 ===");
    defmt::info!("处理模式: {:?}", self.processing_mode);
    defmt::info!(
      "队列长度: {}/{}",
      self.processor.queue_length(),
      self.config.queue_size
    );
    defmt::info!("批处理大小: {}", self.config.batch_size);
    defmt::info!("处理间隔: {} ms", self.config.processing_interval);
  }

  /// 延时函数
  fn delay_ms(&self, ms: u32) {
    for _ in 0..(ms * 1000) {
      cortex_m::asm::nop();
    }
  }

  /// 微秒延时
  fn delay_us(&self, us: u32) {
    for _ in 0..us {
      cortex_m::asm::nop();
    }
  }
}

#[entry]
fn main() -> ! {
  defmt::info!("启动事件处理器演示程序");

  // 初始化外设
  let dp = hal::pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();

  // 状态LED (PA5)
  let status_led = gpioa.pa5.into_push_pull_output();

  // 配置定时器
  let mut timer = Timer::tim2(dp.TIM2, 10.hz(), clocks);
  timer.listen(Event::TimeOut);

  // 创建事件处理器控制器
  let mut controller =
    EventProcessorController::new(status_led, GeneratorMode::Mixed, ProcessingMode::Adaptive);

  // 初始化系统
  if let Err(_) = controller.initialize() {
    defmt::error!("事件处理器初始化失败");
    panic!("Event processor initialization failed");
  }

  // 存储全局引用
  free(|cs| {
    EVENT_CONTROLLER.borrow(cs).replace(Some(controller));
    TIMER.borrow(cs).replace(Some(timer));
  });

  // 启用定时器中断
  unsafe {
    NVIC::unmask(Interrupt::TIM2);
  }

  defmt::info!("事件处理器启动完成");

  // 主循环
  loop {
    free(|cs| {
      if let Some(ref mut controller) = EVENT_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
        controller.run_processing_loop();
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

    // 定时器中断可以触发事件生成或处理
    if let Some(ref mut controller) = EVENT_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
      // 在中断中可以添加高优先级事件
    }
  });
}
