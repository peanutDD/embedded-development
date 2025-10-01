#![no_std]
#![no_main]

use core::{
  cell::RefCell,
  sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::{
  asm, interrupt,
  peripheral::{DWT, NVIC, SCB},
};
use cortex_m_rt::entry;
use critical_section::Mutex;
use fugit::Duration;
use heapless::{
  pool::{Node, Pool},
  FnvIndexMap, Vec,
};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{Edge, Input, Output, Pin, PushPull},
  pac::{self, EXTI, TIM2, TIM3, TIM4, USART1},
  prelude::*,
  rcc::Clocks,
  serial::{Event as SerialEvent, Serial},
  timer::{Event, Timer},
};

/// 中断优先级定义
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum InterruptPriority {
  Critical = 0, // 最高优先级
  High = 1,
  Normal = 2,
  Low = 3,
  Background = 4, // 最低优先级
}

/// 中断类型枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InterruptType {
  Timer2,
  Timer3,
  Timer4,
  ExternalInt0,
  ExternalInt1,
  Usart1,
  SystemTick,
}

/// 中断统计信息
#[derive(Debug, Clone)]
pub struct InterruptStatistics {
  pub interrupt_type: InterruptType,
  pub count: u32,
  pub total_execution_time: u64,
  pub max_execution_time: u32,
  pub min_execution_time: u32,
  pub last_timestamp: u64,
  pub priority: InterruptPriority,
  pub preemption_count: u32,
  pub nested_count: u32,
}

/// 中断管理器
pub struct InterruptManager {
  statistics: FnvIndexMap<InterruptType, InterruptStatistics, 16>,
  current_interrupt: Option<InterruptType>,
  interrupt_stack: Vec<InterruptType, 8>,
  total_interrupts: AtomicU32,
  max_nesting_level: AtomicU32,
  system_start_time: u64,
}

/// 中断处理器特征
pub trait InterruptHandler {
  fn handle(&mut self) -> Duration<u32, 1, 1_000_000>;
  fn priority(&self) -> InterruptPriority;
  fn interrupt_type(&self) -> InterruptType;
}

/// 定时器中断处理器
pub struct TimerInterruptHandler {
  interrupt_type: InterruptType,
  priority: InterruptPriority,
  execution_count: AtomicU32,
  last_execution_time: AtomicU32,
}

/// 外部中断处理器
pub struct ExternalInterruptHandler {
  interrupt_type: InterruptType,
  priority: InterruptPriority,
  debounce_time: u32,
  last_trigger_time: AtomicU32,
}

/// 串口中断处理器
pub struct SerialInterruptHandler {
  interrupt_type: InterruptType,
  priority: InterruptPriority,
  rx_buffer: Vec<u8, 256>,
  tx_buffer: Vec<u8, 256>,
}

/// 全局中断管理器实例
static INTERRUPT_MANAGER: Mutex<RefCell<Option<InterruptManager>>> = Mutex::new(RefCell::new(None));

/// 中断执行时间测量
static INTERRUPT_START_TIME: AtomicU32 = AtomicU32::new(0);
static INTERRUPT_ACTIVE: AtomicBool = AtomicBool::new(false);

impl InterruptStatistics {
  pub fn new(interrupt_type: InterruptType, priority: InterruptPriority) -> Self {
    Self {
      interrupt_type,
      count: 0,
      total_execution_time: 0,
      max_execution_time: 0,
      min_execution_time: u32::MAX,
      last_timestamp: 0,
      priority,
      preemption_count: 0,
      nested_count: 0,
    }
  }

  pub fn update_execution_time(&mut self, execution_time: u32, timestamp: u64) {
    self.count += 1;
    self.total_execution_time += execution_time as u64;
    self.max_execution_time = self.max_execution_time.max(execution_time);
    self.min_execution_time = self.min_execution_time.min(execution_time);
    self.last_timestamp = timestamp;
  }

  pub fn average_execution_time(&self) -> f32 {
    if self.count == 0 {
      0.0
    } else {
      self.total_execution_time as f32 / self.count as f32
    }
  }

  pub fn interrupt_rate(&self, elapsed_time: u64) -> f32 {
    if elapsed_time == 0 {
      0.0
    } else {
      (self.count as f64 / (elapsed_time as f64 / 1_000_000.0)) as f32
    }
  }
}

impl InterruptManager {
  pub fn new() -> Self {
    Self {
      statistics: FnvIndexMap::new(),
      current_interrupt: None,
      interrupt_stack: Vec::new(),
      total_interrupts: AtomicU32::new(0),
      max_nesting_level: AtomicU32::new(0),
      system_start_time: DWT::cycle_count() as u64,
    }
  }

  pub fn register_interrupt(&mut self, interrupt_type: InterruptType, priority: InterruptPriority) {
    let stats = InterruptStatistics::new(interrupt_type, priority);
    self.statistics.insert(interrupt_type, stats).ok();
  }

  pub fn enter_interrupt(&mut self, interrupt_type: InterruptType) {
    // 记录中断开始时间
    let start_time = DWT::cycle_count();
    INTERRUPT_START_TIME.store(start_time, Ordering::SeqCst);
    INTERRUPT_ACTIVE.store(true, Ordering::SeqCst);

    // 检查是否为嵌套中断
    if let Some(current) = self.current_interrupt {
      if let Some(stats) = self.statistics.get_mut(&current) {
        stats.preemption_count += 1;
      }
      self.interrupt_stack.push(current).ok();
    }

    // 更新嵌套级别
    let nesting_level = self.interrupt_stack.len() as u32 + 1;
    let max_nesting = self.max_nesting_level.load(Ordering::SeqCst);
    if nesting_level > max_nesting {
      self
        .max_nesting_level
        .store(nesting_level, Ordering::SeqCst);
    }

    self.current_interrupt = Some(interrupt_type);
    self.total_interrupts.fetch_add(1, Ordering::SeqCst);

    // 更新嵌套计数
    if let Some(stats) = self.statistics.get_mut(&interrupt_type) {
      if !self.interrupt_stack.is_empty() {
        stats.nested_count += 1;
      }
    }
  }

  pub fn exit_interrupt(&mut self, interrupt_type: InterruptType) {
    // 计算执行时间
    let end_time = DWT::cycle_count();
    let start_time = INTERRUPT_START_TIME.load(Ordering::SeqCst);
    let execution_cycles = end_time.wrapping_sub(start_time);
    let execution_time_us = (execution_cycles as u64 * 1_000_000) / 168_000_000; // 168MHz

    // 更新统计信息
    if let Some(stats) = self.statistics.get_mut(&interrupt_type) {
      let timestamp = (end_time as u64 * 1_000_000) / 168_000_000;
      stats.update_execution_time(execution_time_us as u32, timestamp);
    }

    // 恢复上一个中断上下文
    self.current_interrupt = self.interrupt_stack.pop();
    INTERRUPT_ACTIVE.store(false, Ordering::SeqCst);
  }

  pub fn get_statistics(&self, interrupt_type: InterruptType) -> Option<&InterruptStatistics> {
    self.statistics.get(&interrupt_type)
  }

  pub fn get_all_statistics(&self) -> &FnvIndexMap<InterruptType, InterruptStatistics, 16> {
    &self.statistics
  }

  pub fn get_system_statistics(&self) -> SystemStatistics {
    let current_time = (DWT::cycle_count() as u64 * 1_000_000) / 168_000_000;
    let elapsed_time = current_time - self.system_start_time;

    SystemStatistics {
      total_interrupts: self.total_interrupts.load(Ordering::SeqCst),
      max_nesting_level: self.max_nesting_level.load(Ordering::SeqCst),
      elapsed_time_us: elapsed_time,
      interrupt_rate: self.total_interrupts.load(Ordering::SeqCst) as f32
        / (elapsed_time as f32 / 1_000_000.0),
    }
  }

  pub fn reset_statistics(&mut self) {
    for (_, stats) in self.statistics.iter_mut() {
      stats.count = 0;
      stats.total_execution_time = 0;
      stats.max_execution_time = 0;
      stats.min_execution_time = u32::MAX;
      stats.preemption_count = 0;
      stats.nested_count = 0;
    }
    self.total_interrupts.store(0, Ordering::SeqCst);
    self.max_nesting_level.store(0, Ordering::SeqCst);
    self.system_start_time = (DWT::cycle_count() as u64 * 1_000_000) / 168_000_000;
  }
}

/// 系统级统计信息
#[derive(Debug)]
pub struct SystemStatistics {
  pub total_interrupts: u32,
  pub max_nesting_level: u32,
  pub elapsed_time_us: u64,
  pub interrupt_rate: f32,
}

impl TimerInterruptHandler {
  pub fn new(interrupt_type: InterruptType, priority: InterruptPriority) -> Self {
    Self {
      interrupt_type,
      priority,
      execution_count: AtomicU32::new(0),
      last_execution_time: AtomicU32::new(0),
    }
  }
}

impl InterruptHandler for TimerInterruptHandler {
  fn handle(&mut self) -> Duration<u32, 1, 1_000_000> {
    let start = DWT::cycle_count();

    // 模拟定时器中断处理
    self.execution_count.fetch_add(1, Ordering::SeqCst);

    // 执行一些计算密集型任务
    let mut sum = 0u32;
    for i in 0..100 {
      sum = sum.wrapping_add(i);
    }

    let end = DWT::cycle_count();
    let execution_time = ((end.wrapping_sub(start)) as u64 * 1_000_000) / 168_000_000;

    self
      .last_execution_time
      .store(execution_time as u32, Ordering::SeqCst);
    Duration::from_ticks(execution_time as u32)
  }

  fn priority(&self) -> InterruptPriority {
    self.priority
  }

  fn interrupt_type(&self) -> InterruptType {
    self.interrupt_type
  }
}

impl ExternalInterruptHandler {
  pub fn new(
    interrupt_type: InterruptType,
    priority: InterruptPriority,
    debounce_time: u32,
  ) -> Self {
    Self {
      interrupt_type,
      priority,
      debounce_time,
      last_trigger_time: AtomicU32::new(0),
    }
  }
}

impl InterruptHandler for ExternalInterruptHandler {
  fn handle(&mut self) -> Duration<u32, 1, 1_000_000> {
    let start = DWT::cycle_count();
    let current_time = (start as u64 * 1_000_000) / 168_000_000;

    // 防抖处理
    let last_time = self.last_trigger_time.load(Ordering::SeqCst) as u64;
    if current_time - last_time < self.debounce_time as u64 {
      return Duration::from_ticks(0);
    }

    self
      .last_trigger_time
      .store(current_time as u32, Ordering::SeqCst);

    // 模拟外部中断处理
    asm::delay(1000); // 模拟处理延时

    let end = DWT::cycle_count();
    let execution_time = ((end.wrapping_sub(start)) as u64 * 1_000_000) / 168_000_000;

    Duration::from_ticks(execution_time as u32)
  }

  fn priority(&self) -> InterruptPriority {
    self.priority
  }

  fn interrupt_type(&self) -> InterruptType {
    self.interrupt_type
  }
}

/// 中断性能分析器
pub struct InterruptProfiler {
  samples: Vec<u32, 1000>,
  histogram: [u32; 10], // 执行时间直方图
}

impl InterruptProfiler {
  pub fn new() -> Self {
    Self {
      samples: Vec::new(),
      histogram: [0; 10],
    }
  }

  pub fn add_sample(&mut self, execution_time_us: u32) {
    if self.samples.push(execution_time_us).is_err() {
      // 缓冲区满时，移除最旧的样本
      self.samples.remove(0);
      self.samples.push(execution_time_us).ok();
    }

    // 更新直方图 (0-9μs, 10-19μs, ..., 90+μs)
    let bucket = (execution_time_us / 10).min(9) as usize;
    self.histogram[bucket] += 1;
  }

  pub fn get_percentile(&mut self, percentile: f32) -> u32 {
    if self.samples.is_empty() {
      return 0;
    }

    self.samples.sort();
    let index =
      ((self.samples.len() as f32 * percentile / 100.0) as usize).min(self.samples.len() - 1);
    self.samples[index]
  }

  pub fn get_histogram(&self) -> &[u32; 10] {
    &self.histogram
  }
}

// 中断服务程序
#[interrupt]
fn TIM2() {
  critical_section::with(|cs| {
    if let Some(ref mut manager) = INTERRUPT_MANAGER.borrow(cs).borrow_mut().as_mut() {
      manager.enter_interrupt(InterruptType::Timer2);

      // 清除中断标志
      unsafe {
        let tim2 = &*TIM2::ptr();
        tim2.sr.modify(|_, w| w.uif().clear_bit());
      }

      // 执行中断处理逻辑
      asm::delay(500); // 模拟处理时间

      manager.exit_interrupt(InterruptType::Timer2);
    }
  });
}

#[interrupt]
fn TIM3() {
  critical_section::with(|cs| {
    if let Some(ref mut manager) = INTERRUPT_MANAGER.borrow(cs).borrow_mut().as_mut() {
      manager.enter_interrupt(InterruptType::Timer3);

      // 清除中断标志
      unsafe {
        let tim3 = &*TIM3::ptr();
        tim3.sr.modify(|_, w| w.uif().clear_bit());
      }

      // 执行中断处理逻辑
      asm::delay(300); // 模拟处理时间

      manager.exit_interrupt(InterruptType::Timer3);
    }
  });
}

#[interrupt]
fn TIM4() {
  critical_section::with(|cs| {
    if let Some(ref mut manager) = INTERRUPT_MANAGER.borrow(cs).borrow_mut().as_mut() {
      manager.enter_interrupt(InterruptType::Timer4);

      // 清除中断标志
      unsafe {
        let tim4 = &*TIM4::ptr();
        tim4.sr.modify(|_, w| w.uif().clear_bit());
      }

      // 执行中断处理逻辑
      asm::delay(200); // 模拟处理时间

      manager.exit_interrupt(InterruptType::Timer4);
    }
  });
}

#[entry]
fn main() -> ! {
  // 初始化RTT日志
  rtt_target::rtt_init_print!();
  rtt_target::rprintln!("中断管理器系统启动");

  // 初始化外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟到168MHz
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(8.MHz())
    .sysclk(168.MHz())
    .pclk1(42.MHz())
    .pclk2(84.MHz())
    .freeze();

  // 启用DWT周期计数器
  let mut dwt = cp.DWT;
  let dcb = cp.DCB;
  dwt.enable_cycle_counter(&dcb);

  // 创建中断管理器
  let mut interrupt_manager = InterruptManager::new();

  // 注册中断
  interrupt_manager.register_interrupt(InterruptType::Timer2, InterruptPriority::High);
  interrupt_manager.register_interrupt(InterruptType::Timer3, InterruptPriority::Normal);
  interrupt_manager.register_interrupt(InterruptType::Timer4, InterruptPriority::Low);

  // 配置定时器
  let mut tim2 = Timer::new(dp.TIM2, &clocks);
  let mut tim3 = Timer::new(dp.TIM3, &clocks);
  let mut tim4 = Timer::new(dp.TIM4, &clocks);

  // 配置定时器中断
  tim2.start(Duration::<u32, 1, 1000>::from_ticks(10)); // 100Hz
  tim2.listen(Event::Update);

  tim3.start(Duration::<u32, 1, 1000>::from_ticks(20)); // 50Hz
  tim3.listen(Event::Update);

  tim4.start(Duration::<u32, 1, 1000>::from_ticks(50)); // 20Hz
  tim4.listen(Event::Update);

  // 配置NVIC优先级
  unsafe {
    NVIC::unmask(pac::Interrupt::TIM2);
    NVIC::unmask(pac::Interrupt::TIM3);
    NVIC::unmask(pac::Interrupt::TIM4);

    // 设置优先级 (数值越小优先级越高)
    cp.NVIC.set_priority(pac::Interrupt::TIM2, 1 << 4); // 高优先级
    cp.NVIC.set_priority(pac::Interrupt::TIM3, 2 << 4); // 中优先级
    cp.NVIC.set_priority(pac::Interrupt::TIM4, 3 << 4); // 低优先级
  }

  // 将中断管理器设置为全局实例
  critical_section::with(|cs| {
    *INTERRUPT_MANAGER.borrow(cs).borrow_mut() = Some(interrupt_manager);
  });

  rtt_target::rprintln!("中断系统配置完成，开始监控...");

  // 创建性能分析器
  let mut profiler = InterruptProfiler::new();
  let mut report_counter = 0u32;

  // 主循环
  loop {
    // 每5秒生成一次报告
    asm::delay(168_000_000 * 5); // 5秒延时
    report_counter += 1;

    critical_section::with(|cs| {
      if let Some(ref manager) = INTERRUPT_MANAGER.borrow(cs).borrow().as_ref() {
        let system_stats = manager.get_system_statistics();

        rtt_target::rprintln!("=== 中断系统报告 #{} ===", report_counter);
        rtt_target::rprintln!("总中断数: {}", system_stats.total_interrupts);
        rtt_target::rprintln!("最大嵌套级别: {}", system_stats.max_nesting_level);
        rtt_target::rprintln!("中断频率: {:.2} Hz", system_stats.interrupt_rate);
        rtt_target::rprintln!(
          "运行时间: {:.2} 秒",
          system_stats.elapsed_time_us as f32 / 1_000_000.0
        );

        // 打印各中断统计
        for (int_type, stats) in manager.get_all_statistics() {
          rtt_target::rprintln!(
            "{:?}: 计数={}, 平均时间={:.2}μs, 最大时间={}μs, 抢占次数={}, 嵌套次数={}",
            int_type,
            stats.count,
            stats.average_execution_time(),
            stats.max_execution_time,
            stats.preemption_count,
            stats.nested_count
          );
        }
        rtt_target::rprintln!("========================");
      }
    });

    // 每分钟重置统计
    if report_counter % 12 == 0 {
      critical_section::with(|cs| {
        if let Some(ref mut manager) = INTERRUPT_MANAGER.borrow(cs).borrow_mut().as_mut() {
          manager.reset_statistics();
          rtt_target::rprintln!("统计信息已重置");
        }
      });
    }
  }
}
