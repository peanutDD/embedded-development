#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{gpiod::PD12, Output, PushPull},
  interrupt,
  prelude::*,
  stm32,
  timer::{Event, Timer},
};

type LedPin = PD12<Output<PushPull>>;

// 全局变量用于中断处理
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM6>>>> = Mutex::new(RefCell::new(None));
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

#[entry]
fn main() -> ! {
  // 获取外设
  let dp = stm32::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpiod = dp.GPIOD.split();
  let led = gpiod.pd12.into_push_pull_output();

  // 配置定时器6 (基础定时器)
  let mut timer = Timer::tim6(dp.TIM6, &clocks);
  timer.start(1.hz()); // 1Hz，每秒触发一次
  timer.listen(Event::TimeOut);

  // 将对象移动到全局变量
  cortex_m::interrupt::free(|cs| {
    TIMER.borrow(cs).replace(Some(timer));
    LED.borrow(cs).replace(Some(led));
  });

  // 启用定时器6中断
  unsafe {
    NVIC::unmask(stm32::Interrupt::TIM6_DAC);
  }

  // 主循环
  loop {
    // 主循环可以执行其他任务
    cortex_m::asm::wfi(); // 等待中断

    // 读取计数器值
    let count = cortex_m::interrupt::free(|cs| *COUNTER.borrow(cs).borrow());

    // 这里可以根据计数器值执行不同的操作
    if count % 10 == 0 && count > 0 {
      // 每10秒执行一次的操作
    }
  }
}

/// 定时器中断处理程序
#[interrupt]
fn TIM6_DAC() {
  cortex_m::interrupt::free(|cs| {
    // 获取定时器和LED的引用
    if let (Some(ref mut timer), Some(ref mut led)) = (
      TIMER.borrow(cs).borrow_mut().as_mut(),
      LED.borrow(cs).borrow_mut().as_mut(),
    ) {
      // 清除中断标志
      timer.clear_interrupt(Event::TimeOut);

      // 切换LED状态
      led.toggle();

      // 增加计数器
      let mut counter = COUNTER.borrow(cs).borrow_mut();
      *counter += 1;
    }
  });
}

/// 定时器中断管理器
pub struct TimerInterruptManager {
  timer_handlers: [Option<fn()>; 8],
  active_timers: u8,
}

impl TimerInterruptManager {
  /// 创建新的定时器中断管理器
  pub fn new() -> Self {
    Self {
      timer_handlers: [None; 8],
      active_timers: 0,
    }
  }

  /// 注册定时器处理程序
  pub fn register_handler(&mut self, timer_id: usize, handler: fn()) -> Result<(), ()> {
    if timer_id >= 8 {
      return Err(());
    }

    self.timer_handlers[timer_id] = Some(handler);
    self.active_timers |= 1 << timer_id;
    Ok(())
  }

  /// 注销定时器处理程序
  pub fn unregister_handler(&mut self, timer_id: usize) -> Result<(), ()> {
    if timer_id >= 8 {
      return Err(());
    }

    self.timer_handlers[timer_id] = None;
    self.active_timers &= !(1 << timer_id);
    Ok(())
  }

  /// 处理定时器中断
  pub fn handle_timer_interrupt(&self, timer_id: usize) {
    if timer_id < 8 && (self.active_timers & (1 << timer_id)) != 0 {
      if let Some(handler) = self.timer_handlers[timer_id] {
        handler();
      }
    }
  }

  /// 检查定时器是否活动
  pub fn is_timer_active(&self, timer_id: usize) -> bool {
    if timer_id >= 8 {
      return false;
    }

    (self.active_timers & (1 << timer_id)) != 0
  }

  /// 获取活动定时器数量
  pub fn get_active_timer_count(&self) -> u8 {
    self.active_timers.count_ones() as u8
  }
}

/// 中断统计信息
pub struct InterruptStatistics {
  interrupt_count: [u32; 16],
  max_execution_time: [u32; 16],
  total_execution_time: [u64; 16],
  last_interrupt_time: [u32; 16],
}

impl InterruptStatistics {
  /// 创建新的中断统计
  pub fn new() -> Self {
    Self {
      interrupt_count: [0; 16],
      max_execution_time: [0; 16],
      total_execution_time: [0; 16],
      last_interrupt_time: [0; 16],
    }
  }

  /// 记录中断开始
  pub fn interrupt_start(&mut self, interrupt_id: usize, timestamp: u32) {
    if interrupt_id < 16 {
      self.last_interrupt_time[interrupt_id] = timestamp;
      self.interrupt_count[interrupt_id] += 1;
    }
  }

  /// 记录中断结束
  pub fn interrupt_end(&mut self, interrupt_id: usize, timestamp: u32) {
    if interrupt_id < 16 {
      let execution_time = timestamp.wrapping_sub(self.last_interrupt_time[interrupt_id]);

      // 更新最大执行时间
      if execution_time > self.max_execution_time[interrupt_id] {
        self.max_execution_time[interrupt_id] = execution_time;
      }

      // 更新总执行时间
      self.total_execution_time[interrupt_id] += execution_time as u64;
    }
  }

  /// 获取中断次数
  pub fn get_interrupt_count(&self, interrupt_id: usize) -> u32 {
    if interrupt_id < 16 {
      self.interrupt_count[interrupt_id]
    } else {
      0
    }
  }

  /// 获取平均执行时间
  pub fn get_average_execution_time(&self, interrupt_id: usize) -> u32 {
    if interrupt_id < 16 && self.interrupt_count[interrupt_id] > 0 {
      (self.total_execution_time[interrupt_id] / self.interrupt_count[interrupt_id] as u64) as u32
    } else {
      0
    }
  }

  /// 获取最大执行时间
  pub fn get_max_execution_time(&self, interrupt_id: usize) -> u32 {
    if interrupt_id < 16 {
      self.max_execution_time[interrupt_id]
    } else {
      0
    }
  }

  /// 重置统计信息
  pub fn reset(&mut self) {
    self.interrupt_count = [0; 16];
    self.max_execution_time = [0; 16];
    self.total_execution_time = [0; 16];
    self.last_interrupt_time = [0; 16];
  }

  /// 重置指定中断的统计信息
  pub fn reset_interrupt(&mut self, interrupt_id: usize) {
    if interrupt_id < 16 {
      self.interrupt_count[interrupt_id] = 0;
      self.max_execution_time[interrupt_id] = 0;
      self.total_execution_time[interrupt_id] = 0;
      self.last_interrupt_time[interrupt_id] = 0;
    }
  }
}

/// 中断负载监控器
pub struct InterruptLoadMonitor {
  statistics: InterruptStatistics,
  monitoring_period: u32,
  last_monitor_time: u32,
  cpu_load_threshold: u8,
  overload_callback: Option<fn(usize, u8)>,
}

impl InterruptLoadMonitor {
  /// 创建中断负载监控器
  pub fn new(monitoring_period_ms: u32, load_threshold: u8) -> Self {
    Self {
      statistics: InterruptStatistics::new(),
      monitoring_period: monitoring_period_ms,
      last_monitor_time: 0,
      cpu_load_threshold: load_threshold,
      overload_callback: None,
    }
  }

  /// 设置过载回调
  pub fn set_overload_callback(&mut self, callback: fn(usize, u8)) {
    self.overload_callback = Some(callback);
  }

  /// 监控中断负载
  pub fn monitor(&mut self, current_time: u32) {
    if current_time.wrapping_sub(self.last_monitor_time) >= self.monitoring_period {
      // 检查每个中断的CPU负载
      for i in 0..16 {
        let count = self.statistics.get_interrupt_count(i);
        if count > 0 {
          let avg_time = self.statistics.get_average_execution_time(i);
          let load_percentage = ((avg_time * count) * 100) / self.monitoring_period;

          if load_percentage > self.cpu_load_threshold as u32 {
            if let Some(callback) = self.overload_callback {
              callback(i, load_percentage as u8);
            }
          }
        }
      }

      // 重置统计信息
      self.statistics.reset();
      self.last_monitor_time = current_time;
    }
  }

  /// 记录中断执行
  pub fn record_interrupt(&mut self, interrupt_id: usize, start_time: u32, end_time: u32) {
    self.statistics.interrupt_start(interrupt_id, start_time);
    self.statistics.interrupt_end(interrupt_id, end_time);
  }
}
