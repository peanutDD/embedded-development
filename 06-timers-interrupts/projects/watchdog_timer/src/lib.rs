#![no_std]

use core::fmt::Write;
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use heapless::{String, Vec};

/// 看门狗定时器特征
pub trait WatchdogTimer {
  type Error;

  /// 启动看门狗
  fn start(&mut self, timeout_ms: u32) -> Result<(), Self::Error>;

  /// 喂狗操作
  fn feed(&mut self) -> Result<(), Self::Error>;

  /// 停止看门狗
  fn stop(&mut self) -> Result<(), Self::Error>;

  /// 获取剩余时间
  fn get_remaining_time(&self) -> u32;

  /// 检查是否由看门狗复位
  fn is_watchdog_reset(&self) -> bool;
}

/// 系统监控器
#[derive(Debug, Clone)]
pub struct SystemMonitor {
  task_timeouts: Vec<TaskTimeout, 16>,
  system_health: SystemHealth,
  monitoring_enabled: bool,
  last_check_time: u32,
  check_interval_ms: u32,
}

/// 任务超时监控
#[derive(Debug, Clone)]
pub struct TaskTimeout {
  task_id: u8,
  name: String<32>,
  timeout_ms: u32,
  last_update: u32,
  is_critical: bool,
  timeout_count: u32,
}

/// 系统健康状态
#[derive(Debug, Clone)]
pub struct SystemHealth {
  cpu_usage: f32,
  memory_usage: f32,
  temperature: f32,
  voltage: f32,
  error_count: u32,
  uptime_ms: u32,
  reset_count: u32,
  last_reset_reason: ResetReason,
}

/// 复位原因
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ResetReason {
  PowerOn,
  Watchdog,
  Software,
  External,
  BrownOut,
  Unknown,
}

/// 故障恢复策略
#[derive(Debug, Clone, Copy)]
pub enum RecoveryStrategy {
  Restart,  // 重启系统
  SafeMode, // 进入安全模式
  Ignore,   // 忽略错误
  Shutdown, // 关闭系统
}

/// 看门狗配置
#[derive(Debug, Clone)]
pub struct WatchdogConfig {
  timeout_ms: u32,
  window_ms: Option<u32>, // 窗口看门狗
  prescaler: u16,
  auto_reload: bool,
  debug_freeze: bool,
  standby_freeze: bool,
}

impl Default for WatchdogConfig {
  fn default() -> Self {
    Self {
      timeout_ms: 1000, // 1秒超时
      window_ms: None,  // 无窗口限制
      prescaler: 4,     // 4分频
      auto_reload: true,
      debug_freeze: true,   // 调试时冻结
      standby_freeze: true, // 待机时冻结
    }
  }
}

impl SystemMonitor {
  /// 创建新的系统监控器
  pub fn new(check_interval_ms: u32) -> Self {
    Self {
      task_timeouts: Vec::new(),
      system_health: SystemHealth::default(),
      monitoring_enabled: false,
      last_check_time: 0,
      check_interval_ms,
    }
  }

  /// 添加任务监控
  pub fn add_task(
    &mut self,
    task_id: u8,
    name: &str,
    timeout_ms: u32,
    is_critical: bool,
  ) -> Result<(), ()> {
    let mut task_name = String::new();
    task_name.push_str(name).map_err(|_| ())?;

    let task = TaskTimeout {
      task_id,
      name: task_name,
      timeout_ms,
      last_update: 0,
      is_critical,
      timeout_count: 0,
    };

    self.task_timeouts.push(task).map_err(|_| ())
  }

  /// 更新任务状态
  pub fn update_task(&mut self, task_id: u8, current_time: u32) -> Result<(), ()> {
    for task in &mut self.task_timeouts {
      if task.task_id == task_id {
        task.last_update = current_time;
        return Ok(());
      }
    }
    Err(())
  }

  /// 检查系统状态
  pub fn check_system(&mut self, current_time: u32) -> Vec<TaskTimeout, 16> {
    let mut timed_out_tasks = Vec::new();

    if !self.monitoring_enabled {
      return timed_out_tasks;
    }

    if current_time < self.last_check_time + self.check_interval_ms {
      return timed_out_tasks;
    }

    for task in &mut self.task_timeouts {
      if current_time > task.last_update + task.timeout_ms {
        task.timeout_count += 1;
        timed_out_tasks.push(task.clone()).ok();
      }
    }

    self.last_check_time = current_time;
    timed_out_tasks
  }

  /// 启用监控
  pub fn enable_monitoring(&mut self) {
    self.monitoring_enabled = true;
  }

  /// 禁用监控
  pub fn disable_monitoring(&mut self) {
    self.monitoring_enabled = false;
  }

  /// 更新系统健康状态
  pub fn update_health(
    &mut self,
    cpu_usage: f32,
    memory_usage: f32,
    temperature: f32,
    voltage: f32,
  ) {
    self.system_health.cpu_usage = cpu_usage;
    self.system_health.memory_usage = memory_usage;
    self.system_health.temperature = temperature;
    self.system_health.voltage = voltage;
  }

  /// 获取系统健康状态
  pub fn get_health(&self) -> &SystemHealth {
    &self.system_health
  }

  /// 重置任务超时计数
  pub fn reset_task_timeout(&mut self, task_id: u8) {
    for task in &mut self.task_timeouts {
      if task.task_id == task_id {
        task.timeout_count = 0;
        break;
      }
    }
  }
}

impl Default for SystemHealth {
  fn default() -> Self {
    Self {
      cpu_usage: 0.0,
      memory_usage: 0.0,
      temperature: 25.0,
      voltage: 3.3,
      error_count: 0,
      uptime_ms: 0,
      reset_count: 0,
      last_reset_reason: ResetReason::PowerOn,
    }
  }
}

/// 独立看门狗实现
pub struct IndependentWatchdog {
  timeout_ms: u32,
  last_feed_time: u32,
  is_running: bool,
  config: WatchdogConfig,
}

impl IndependentWatchdog {
  /// 创建新的独立看门狗
  pub fn new(config: WatchdogConfig) -> Self {
    Self {
      timeout_ms: config.timeout_ms,
      last_feed_time: 0,
      is_running: false,
      config,
    }
  }

  /// 配置看门狗参数
  pub fn configure(&mut self, config: WatchdogConfig) {
    self.config = config;
    self.timeout_ms = config.timeout_ms;
  }

  /// 检查是否需要喂狗
  pub fn needs_feeding(&self, current_time: u32) -> bool {
    if !self.is_running {
      return false;
    }

    current_time > self.last_feed_time + (self.timeout_ms / 2)
  }

  /// 获取看门狗状态
  pub fn get_status(&self) -> (bool, u32, u32) {
    (self.is_running, self.timeout_ms, self.last_feed_time)
  }
}

impl WatchdogTimer for IndependentWatchdog {
  type Error = ();

  fn start(&mut self, timeout_ms: u32) -> Result<(), Self::Error> {
    self.timeout_ms = timeout_ms;
    self.is_running = true;
    self.last_feed_time = 0; // 应该从系统时钟获取
    Ok(())
  }

  fn feed(&mut self) -> Result<(), Self::Error> {
    if self.is_running {
      self.last_feed_time = 0; // 应该从系统时钟获取
    }
    Ok(())
  }

  fn stop(&mut self) -> Result<(), Self::Error> {
    self.is_running = false;
    Ok(())
  }

  fn get_remaining_time(&self) -> u32 {
    if self.is_running {
      let elapsed = 0; // 应该计算实际经过的时间
      if elapsed < self.timeout_ms {
        self.timeout_ms - elapsed
      } else {
        0
      }
    } else {
      0
    }
  }

  fn is_watchdog_reset(&self) -> bool {
    // 这里应该检查复位状态寄存器
    false
  }
}

/// 窗口看门狗实现
pub struct WindowWatchdog {
  timeout_ms: u32,
  window_ms: u32,
  last_feed_time: u32,
  is_running: bool,
  config: WatchdogConfig,
}

impl WindowWatchdog {
  /// 创建新的窗口看门狗
  pub fn new(config: WatchdogConfig) -> Self {
    Self {
      timeout_ms: config.timeout_ms,
      window_ms: config.window_ms.unwrap_or(config.timeout_ms / 2),
      last_feed_time: 0,
      is_running: false,
      config,
    }
  }

  /// 检查是否在喂狗窗口内
  pub fn is_in_window(&self, current_time: u32) -> bool {
    if !self.is_running {
      return false;
    }

    let elapsed = current_time - self.last_feed_time;
    elapsed >= self.window_ms && elapsed < self.timeout_ms
  }

  /// 获取窗口状态
  pub fn get_window_status(&self, current_time: u32) -> (bool, u32, u32) {
    let elapsed = current_time - self.last_feed_time;
    let in_window = self.is_in_window(current_time);
    let remaining = if elapsed < self.timeout_ms {
      self.timeout_ms - elapsed
    } else {
      0
    };

    (in_window, elapsed, remaining)
  }
}

impl WatchdogTimer for WindowWatchdog {
  type Error = ();

  fn start(&mut self, timeout_ms: u32) -> Result<(), Self::Error> {
    self.timeout_ms = timeout_ms;
    self.window_ms = timeout_ms / 2; // 默认窗口为超时时间的一半
    self.is_running = true;
    self.last_feed_time = 0;
    Ok(())
  }

  fn feed(&mut self) -> Result<(), Self::Error> {
    let current_time = 0; // 应该从系统时钟获取

    if !self.is_running {
      return Err(());
    }

    if !self.is_in_window(current_time) {
      return Err(()); // 不在窗口内，喂狗失败
    }

    self.last_feed_time = current_time;
    Ok(())
  }

  fn stop(&mut self) -> Result<(), Self::Error> {
    self.is_running = false;
    Ok(())
  }

  fn get_remaining_time(&self) -> u32 {
    if self.is_running {
      let elapsed = 0; // 应该计算实际经过的时间
      if elapsed < self.timeout_ms {
        self.timeout_ms - elapsed
      } else {
        0
      }
    } else {
      0
    }
  }

  fn is_watchdog_reset(&self) -> bool {
    // 这里应该检查复位状态寄存器
    false
  }
}

/// 故障恢复管理器
pub struct FaultRecoveryManager {
  recovery_strategies: Vec<(u8, RecoveryStrategy), 16>,
  fault_history: Vec<FaultRecord, 32>,
  recovery_count: u32,
  max_recovery_attempts: u32,
}

/// 故障记录
#[derive(Debug, Clone)]
pub struct FaultRecord {
  task_id: u8,
  fault_time: u32,
  recovery_strategy: RecoveryStrategy,
  recovery_success: bool,
}

impl FaultRecoveryManager {
  /// 创建新的故障恢复管理器
  pub fn new(max_recovery_attempts: u32) -> Self {
    Self {
      recovery_strategies: Vec::new(),
      fault_history: Vec::new(),
      recovery_count: 0,
      max_recovery_attempts,
    }
  }

  /// 设置任务的恢复策略
  pub fn set_recovery_strategy(
    &mut self,
    task_id: u8,
    strategy: RecoveryStrategy,
  ) -> Result<(), ()> {
    // 检查是否已存在该任务的策略
    for (id, existing_strategy) in &mut self.recovery_strategies {
      if *id == task_id {
        *existing_strategy = strategy;
        return Ok(());
      }
    }

    // 添加新的策略
    self
      .recovery_strategies
      .push((task_id, strategy))
      .map_err(|_| ())
  }

  /// 执行故障恢复
  pub fn handle_fault(&mut self, task_id: u8, current_time: u32) -> RecoveryStrategy {
    let strategy = self.get_recovery_strategy(task_id);

    // 记录故障
    let fault_record = FaultRecord {
      task_id,
      fault_time: current_time,
      recovery_strategy: strategy,
      recovery_success: false, // 初始设为失败，后续更新
    };

    self.fault_history.push(fault_record).ok();
    self.recovery_count += 1;

    strategy
  }

  /// 获取任务的恢复策略
  pub fn get_recovery_strategy(&self, task_id: u8) -> RecoveryStrategy {
    for (id, strategy) in &self.recovery_strategies {
      if *id == task_id {
        return *strategy;
      }
    }

    // 默认策略：如果恢复次数过多，则关闭系统
    if self.recovery_count >= self.max_recovery_attempts {
      RecoveryStrategy::Shutdown
    } else {
      RecoveryStrategy::Restart
    }
  }

  /// 标记恢复成功
  pub fn mark_recovery_success(&mut self, task_id: u8) {
    if let Some(record) = self
      .fault_history
      .iter_mut()
      .rev()
      .find(|r| r.task_id == task_id)
    {
      record.recovery_success = true;
    }
  }

  /// 获取故障统计
  pub fn get_fault_statistics(&self) -> (u32, u32, u32) {
    let total_faults = self.fault_history.len() as u32;
    let successful_recoveries = self
      .fault_history
      .iter()
      .filter(|r| r.recovery_success)
      .count() as u32;
    let failed_recoveries = total_faults - successful_recoveries;

    (total_faults, successful_recoveries, failed_recoveries)
  }

  /// 重置恢复计数
  pub fn reset_recovery_count(&mut self) {
    self.recovery_count = 0;
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_system_monitor() {
    let mut monitor = SystemMonitor::new(100);

    // 添加任务
    monitor.add_task(1, "main_task", 1000, true).unwrap();
    monitor.add_task(2, "comm_task", 500, false).unwrap();

    // 启用监控
    monitor.enable_monitoring();

    // 更新任务状态
    monitor.update_task(1, 0).unwrap();
    monitor.update_task(2, 0).unwrap();

    // 检查系统状态（未超时）
    let timed_out = monitor.check_system(500);
    assert_eq!(timed_out.len(), 0);

    // 检查系统状态（任务2超时）
    let timed_out = monitor.check_system(600);
    assert_eq!(timed_out.len(), 1);
    assert_eq!(timed_out[0].task_id, 2);
  }

  #[test]
  fn test_independent_watchdog() {
    let config = WatchdogConfig::default();
    let mut watchdog = IndependentWatchdog::new(config);

    // 启动看门狗
    watchdog.start(1000).unwrap();
    assert!(watchdog.is_running);

    // 喂狗
    watchdog.feed().unwrap();

    // 停止看门狗
    watchdog.stop().unwrap();
    assert!(!watchdog.is_running);
  }

  #[test]
  fn test_window_watchdog() {
    let mut config = WatchdogConfig::default();
    config.window_ms = Some(500);
    let mut watchdog = WindowWatchdog::new(config);

    // 启动看门狗
    watchdog.start(1000).unwrap();

    // 在窗口外喂狗应该失败
    assert!(watchdog.feed().is_err());

    // 模拟时间经过，进入窗口
    watchdog.last_feed_time = 0;
    // 这里需要模拟时间经过到窗口内
  }

  #[test]
  fn test_fault_recovery_manager() {
    let mut manager = FaultRecoveryManager::new(3);

    // 设置恢复策略
    manager
      .set_recovery_strategy(1, RecoveryStrategy::Restart)
      .unwrap();
    manager
      .set_recovery_strategy(2, RecoveryStrategy::SafeMode)
      .unwrap();

    // 处理故障
    let strategy1 = manager.handle_fault(1, 1000);
    assert_eq!(strategy1, RecoveryStrategy::Restart);

    let strategy2 = manager.handle_fault(2, 2000);
    assert_eq!(strategy2, RecoveryStrategy::SafeMode);

    // 标记恢复成功
    manager.mark_recovery_success(1);

    // 获取统计信息
    let (total, success, failed) = manager.get_fault_statistics();
    assert_eq!(total, 2);
    assert_eq!(success, 1);
    assert_eq!(failed, 1);
  }
}
