//! # GPIO中断处理抽象
//!
//! 提供GPIO中断的抽象接口和管理功能

use crate::error::HalError;
use crate::gpio::{GpioPin, InputPin};
use heapless::Vec;

/// 中断触发模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptMode {
  /// 上升沿触发
  RisingEdge,
  /// 下降沿触发
  FallingEdge,
  /// 双边沿触发
  BothEdges,
  /// 低电平触发
  LowLevel,
  /// 高电平触发
  HighLevel,
}

/// 中断优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum InterruptPriority {
  /// 最低优先级
  Lowest = 0,
  /// 低优先级
  Low = 1,
  /// 中等优先级
  Medium = 2,
  /// 高优先级
  High = 3,
  /// 最高优先级
  Highest = 4,
}

/// GPIO中断特征
pub trait InterruptPin: InputPin {
  /// 中断模式类型
  type InterruptMode;

  /// 启用中断
  fn enable_interrupt(&mut self, mode: Self::InterruptMode) -> Result<(), Self::Error>;

  /// 禁用中断
  fn disable_interrupt(&mut self) -> Result<(), Self::Error>;

  /// 清除中断标志
  fn clear_interrupt(&mut self) -> Result<(), Self::Error>;

  /// 检查中断是否挂起
  fn is_interrupt_pending(&self) -> Result<bool, Self::Error>;

  /// 设置中断优先级
  fn set_interrupt_priority(&mut self, priority: InterruptPriority) -> Result<(), Self::Error>;

  /// 获取中断优先级
  fn interrupt_priority(&self) -> InterruptPriority;
}

/// 中断回调特征
pub trait InterruptCallback: Send {
  /// 中断处理函数
  fn on_interrupt(&mut self);

  /// 获取回调ID
  fn callback_id(&self) -> u32;

  /// 检查回调是否启用
  fn is_enabled(&self) -> bool;

  /// 启用/禁用回调
  fn set_enabled(&mut self, enabled: bool);
}

/// 简单的中断回调实现
pub struct SimpleCallback<F>
where
  F: FnMut() + Send,
{
  id: u32,
  enabled: bool,
  callback: F,
}

impl<F> SimpleCallback<F>
where
  F: FnMut() + Send,
{
  /// 创建新的简单回调
  pub fn new(id: u32, callback: F) -> Self {
    Self {
      id,
      enabled: true,
      callback,
    }
  }
}

impl<F> InterruptCallback for SimpleCallback<F>
where
  F: FnMut() + Send,
{
  fn on_interrupt(&mut self) {
    if self.enabled {
      (self.callback)();
    }
  }

  fn callback_id(&self) -> u32 {
    self.id
  }

  fn is_enabled(&self) -> bool {
    self.enabled
  }

  fn set_enabled(&mut self, enabled: bool) {
    self.enabled = enabled;
  }
}

/// 中断统计信息
#[derive(Debug, Clone, Copy)]
pub struct InterruptStatistics {
  /// 中断总数
  pub total_interrupts: u32,
  /// 处理的中断数
  pub handled_interrupts: u32,
  /// 丢失的中断数
  pub missed_interrupts: u32,
  /// 平均处理时间 (微秒)
  pub average_handling_time_us: u32,
  /// 最大处理时间 (微秒)
  pub max_handling_time_us: u32,
  /// 最小处理时间 (微秒)
  pub min_handling_time_us: u32,
}

impl Default for InterruptStatistics {
  fn default() -> Self {
    Self {
      total_interrupts: 0,
      handled_interrupts: 0,
      missed_interrupts: 0,
      average_handling_time_us: 0,
      max_handling_time_us: 0,
      min_handling_time_us: u32::MAX,
    }
  }
}

impl InterruptStatistics {
  /// 记录中断处理
  pub fn record_interrupt(&mut self, handling_time_us: u32) {
    self.total_interrupts += 1;
    self.handled_interrupts += 1;

    // 更新时间统计
    if handling_time_us > self.max_handling_time_us {
      self.max_handling_time_us = handling_time_us;
    }

    if handling_time_us < self.min_handling_time_us {
      self.min_handling_time_us = handling_time_us;
    }

    // 计算平均时间 (简化版本)
    self.average_handling_time_us = (self.average_handling_time_us * (self.handled_interrupts - 1)
      + handling_time_us)
      / self.handled_interrupts;
  }

  /// 记录丢失的中断
  pub fn record_missed_interrupt(&mut self) {
    self.total_interrupts += 1;
    self.missed_interrupts += 1;
  }

  /// 重置统计信息
  pub fn reset(&mut self) {
    *self = Self::default();
  }

  /// 获取中断处理成功率 (百分比)
  pub fn success_rate(&self) -> u32 {
    if self.total_interrupts > 0 {
      (self.handled_interrupts * 100) / self.total_interrupts
    } else {
      100
    }
  }
}

/// 中断管理器
pub struct InterruptManager<const N: usize> {
  /// 中断回调数组
  callbacks: [Option<Box<dyn InterruptCallback>>; N],
  /// 中断统计信息
  statistics: [InterruptStatistics; N],
  /// 全局中断启用状态
  global_enabled: bool,
}

impl<const N: usize> InterruptManager<N> {
  /// 创建新的中断管理器
  pub fn new() -> Self {
    Self {
      callbacks: [const { None }; N],
      statistics: [InterruptStatistics::default(); N],
      global_enabled: true,
    }
  }

  /// 注册中断回调
  pub fn register_callback(
    &mut self,
    pin: u8,
    callback: Box<dyn InterruptCallback>,
  ) -> Result<(), HalError> {
    if (pin as usize) < N {
      self.callbacks[pin as usize] = Some(callback);
      log::info!("为引脚 {} 注册中断回调", pin);
      Ok(())
    } else {
      Err(HalError::Configuration(
        crate::error::ConfigError::OutOfRange,
      ))
    }
  }

  /// 注销中断回调
  pub fn unregister_callback(&mut self, pin: u8) -> Result<(), HalError> {
    if (pin as usize) < N {
      self.callbacks[pin as usize] = None;
      log::info!("注销引脚 {} 的中断回调", pin);
      Ok(())
    } else {
      Err(HalError::Configuration(
        crate::error::ConfigError::OutOfRange,
      ))
    }
  }

  /// 处理中断
  pub fn handle_interrupt(&mut self, pin: u8) {
    if !self.global_enabled {
      return;
    }

    let pin_index = pin as usize;
    if pin_index < N {
      let start_time = self.get_current_time_us();

      if let Some(callback) = &mut self.callbacks[pin_index] {
        callback.on_interrupt();

        let handling_time = self.get_current_time_us() - start_time;
        self.statistics[pin_index].record_interrupt(handling_time);

        log::debug!("处理引脚 {} 中断，耗时 {} 微秒", pin, handling_time);
      } else {
        self.statistics[pin_index].record_missed_interrupt();
        log::warn!("引脚 {} 中断无回调处理", pin);
      }
    }
  }

  /// 启用全局中断
  pub fn enable_global_interrupts(&mut self) {
    self.global_enabled = true;
    log::info!("启用全局中断");
  }

  /// 禁用全局中断
  pub fn disable_global_interrupts(&mut self) {
    self.global_enabled = false;
    log::info!("禁用全局中断");
  }

  /// 检查全局中断是否启用
  pub fn is_global_enabled(&self) -> bool {
    self.global_enabled
  }

  /// 获取引脚中断统计信息
  pub fn get_statistics(&self, pin: u8) -> Option<InterruptStatistics> {
    if (pin as usize) < N {
      Some(self.statistics[pin as usize])
    } else {
      None
    }
  }

  /// 重置引脚统计信息
  pub fn reset_statistics(&mut self, pin: u8) -> Result<(), HalError> {
    if (pin as usize) < N {
      self.statistics[pin as usize].reset();
      Ok(())
    } else {
      Err(HalError::Configuration(
        crate::error::ConfigError::OutOfRange,
      ))
    }
  }

  /// 重置所有统计信息
  pub fn reset_all_statistics(&mut self) {
    for stat in &mut self.statistics {
      stat.reset();
    }
  }

  /// 获取所有活跃的中断引脚
  pub fn get_active_pins(&self) -> Vec<u8, N> {
    let mut active_pins = Vec::new();

    for (i, callback) in self.callbacks.iter().enumerate() {
      if callback.is_some() {
        let _ = active_pins.push(i as u8);
      }
    }

    active_pins
  }

  /// 获取当前时间 (微秒) - 这里是模拟实现
  fn get_current_time_us(&self) -> u32 {
    // 在实际实现中，这里应该使用系统定时器
    // 这里返回一个模拟值
    0
  }
}

/// 中断配置结构体
#[derive(Debug, Clone)]
pub struct InterruptConfig {
  /// 中断模式
  pub mode: InterruptMode,
  /// 中断优先级
  pub priority: InterruptPriority,
  /// 是否启用去抖动
  pub debounce_enabled: bool,
  /// 去抖动时间 (毫秒)
  pub debounce_time_ms: u32,
  /// 是否启用边沿检测
  pub edge_detection: bool,
}

impl Default for InterruptConfig {
  fn default() -> Self {
    Self {
      mode: InterruptMode::RisingEdge,
      priority: InterruptPriority::Medium,
      debounce_enabled: false,
      debounce_time_ms: 50,
      edge_detection: true,
    }
  }
}

/// 去抖动处理器
pub struct DebounceHandler {
  /// 最后状态
  last_state: bool,
  /// 最后变化时间
  last_change_time: u32,
  /// 去抖动时间
  debounce_time_ms: u32,
  /// 稳定状态
  stable_state: bool,
}

impl DebounceHandler {
  /// 创建新的去抖动处理器
  pub fn new(debounce_time_ms: u32) -> Self {
    Self {
      last_state: false,
      last_change_time: 0,
      debounce_time_ms,
      stable_state: false,
    }
  }

  /// 处理输入状态
  pub fn process(&mut self, current_state: bool, current_time_ms: u32) -> Option<bool> {
    if current_state != self.last_state {
      // 状态发生变化，记录时间
      self.last_state = current_state;
      self.last_change_time = current_time_ms;
      None
    } else if current_time_ms - self.last_change_time >= self.debounce_time_ms {
      // 状态稳定超过去抖动时间
      if current_state != self.stable_state {
        self.stable_state = current_state;
        Some(current_state)
      } else {
        None
      }
    } else {
      None
    }
  }

  /// 获取当前稳定状态
  pub fn stable_state(&self) -> bool {
    self.stable_state
  }

  /// 重置去抖动状态
  pub fn reset(&mut self) {
    self.last_state = false;
    self.last_change_time = 0;
    self.stable_state = false;
  }
}

/// 中断事件类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptEvent {
  /// 上升沿事件
  RisingEdge,
  /// 下降沿事件
  FallingEdge,
  /// 高电平事件
  HighLevel,
  /// 低电平事件
  LowLevel,
}

/// 中断事件处理器
pub trait InterruptEventHandler {
  /// 处理中断事件
  fn handle_event(&mut self, pin: u8, event: InterruptEvent);

  /// 获取处理器名称
  fn handler_name(&self) -> &'static str;
}

/// 简单的事件处理器实现
pub struct SimpleEventHandler<F>
where
  F: FnMut(u8, InterruptEvent),
{
  name: &'static str,
  handler: F,
}

impl<F> SimpleEventHandler<F>
where
  F: FnMut(u8, InterruptEvent),
{
  /// 创建新的事件处理器
  pub fn new(name: &'static str, handler: F) -> Self {
    Self { name, handler }
  }
}

impl<F> InterruptEventHandler for SimpleEventHandler<F>
where
  F: FnMut(u8, InterruptEvent),
{
  fn handle_event(&mut self, pin: u8, event: InterruptEvent) {
    (self.handler)(pin, event);
  }

  fn handler_name(&self) -> &'static str {
    self.name
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_interrupt_statistics() {
    let mut stats = InterruptStatistics::default();

    stats.record_interrupt(100);
    stats.record_interrupt(200);
    stats.record_interrupt(150);

    assert_eq!(stats.total_interrupts, 3);
    assert_eq!(stats.handled_interrupts, 3);
    assert_eq!(stats.missed_interrupts, 0);
    assert_eq!(stats.max_handling_time_us, 200);
    assert_eq!(stats.min_handling_time_us, 100);
    assert_eq!(stats.success_rate(), 100);

    stats.record_missed_interrupt();
    assert_eq!(stats.total_interrupts, 4);
    assert_eq!(stats.success_rate(), 75);
  }

  #[test]
  fn test_interrupt_manager() {
    let mut manager = InterruptManager::<16>::new();

    let callback = Box::new(SimpleCallback::new(1, || {
      // 测试回调
    }));

    assert!(manager.register_callback(5, callback).is_ok());

    let active_pins = manager.get_active_pins();
    assert_eq!(active_pins.len(), 1);
    assert_eq!(active_pins[0], 5);

    manager.handle_interrupt(5);

    let stats = manager.get_statistics(5).unwrap();
    assert_eq!(stats.handled_interrupts, 1);
  }

  #[test]
  fn test_debounce_handler() {
    let mut debounce = DebounceHandler::new(50);

    // 初始状态
    assert_eq!(debounce.process(false, 0), None);

    // 状态变化
    assert_eq!(debounce.process(true, 10), None);

    // 状态稳定但时间不够
    assert_eq!(debounce.process(true, 40), None);

    // 状态稳定且时间足够
    assert_eq!(debounce.process(true, 70), Some(true));

    // 再次相同状态
    assert_eq!(debounce.process(true, 80), None);
  }

  #[test]
  fn test_interrupt_config() {
    let config = InterruptConfig::default();

    assert_eq!(config.mode, InterruptMode::RisingEdge);
    assert_eq!(config.priority, InterruptPriority::Medium);
    assert!(!config.debounce_enabled);
    assert_eq!(config.debounce_time_ms, 50);
    assert!(config.edge_detection);
  }
}
