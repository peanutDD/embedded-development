#![no_std]

//! # 数字IO控制库
//!
//! 本库提供了完整的数字IO控制功能，包括：
//! - 高级数字输出控制
//! - 智能数字输入处理
//! - 矩阵键盘和LED控制
//! - 移位寄存器扩展
//! - IO扩展器管理

use bitfield::bitfield;
use core::cell::RefCell;
use critical_section::Mutex;
use embedded_hal::digital::{InputPin, OutputPin};
use heapless::{Deque, Vec};

/// 数字IO控制特征
pub trait DigitalIoController {
  type Error;

  /// 设置输出状态
  fn set_output(&mut self, pin: u8, state: bool) -> Result<(), Self::Error>;

  /// 读取输入状态
  fn read_input(&mut self, pin: u8) -> Result<bool, Self::Error>;

  /// 批量设置输出
  fn set_outputs(&mut self, mask: u32, values: u32) -> Result<(), Self::Error>;

  /// 批量读取输入
  fn read_inputs(&mut self, mask: u32) -> Result<u32, Self::Error>;
}

/// 引脚状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PinState {
  Low,
  High,
  HighZ, // 高阻态
}

/// 引脚模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PinMode {
  Input,
  Output,
  InputPullUp,
  InputPullDown,
  OpenDrain,
  PushPull,
}

/// 边沿检测类型
#[derive(Debug, Clone, Copy, PartEq)]
pub enum EdgeDetection {
  None,
  Rising,
  Falling,
  Both,
}

/// 数字输出控制器
pub struct DigitalOutputController<const N: usize> {
  /// 输出引脚状态
  pin_states: [PinState; N],
  /// 引脚配置
  pin_configs: [OutputConfig; N],
  /// 输出模式
  output_mode: OutputMode,
  /// 统计信息
  stats: OutputStats,
}

/// 输出配置
#[derive(Debug, Clone, Copy)]
pub struct OutputConfig {
  pub mode: PinMode,
  pub initial_state: PinState,
  pub drive_strength: DriveStrength,
  pub slew_rate: SlewRate,
}

/// 驱动强度
#[derive(Debug, Clone, Copy)]
pub enum DriveStrength {
  Low,
  Medium,
  High,
  Maximum,
}

/// 转换速率
#[derive(Debug, Clone, Copy)]
pub enum SlewRate {
  Slow,
  Medium,
  Fast,
  Maximum,
}

/// 输出模式
#[derive(Debug, Clone, Copy)]
pub enum OutputMode {
  Individual, // 单独控制
  Group,      // 组控制
  Pattern,    // 模式输出
  PWM,        // PWM输出
}

/// 输出统计
#[derive(Debug, Default)]
pub struct OutputStats {
  pub total_writes: u32,
  pub state_changes: u32,
  pub error_count: u32,
  pub last_update_time: u32,
}

impl Default for OutputConfig {
  fn default() -> Self {
    Self {
      mode: PinMode::PushPull,
      initial_state: PinState::Low,
      drive_strength: DriveStrength::Medium,
      slew_rate: SlewRate::Medium,
    }
  }
}

impl<const N: usize> DigitalOutputController<N> {
  /// 创建新的输出控制器
  pub fn new(configs: [OutputConfig; N]) -> Self {
    let mut pin_states = [PinState::Low; N];

    // 设置初始状态
    for (i, config) in configs.iter().enumerate() {
      pin_states[i] = config.initial_state;
    }

    Self {
      pin_states,
      pin_configs: configs,
      output_mode: OutputMode::Individual,
      stats: OutputStats::default(),
    }
  }

  /// 设置单个引脚状态
  pub fn set_pin(&mut self, pin: usize, state: PinState) -> Result<(), ()> {
    if pin >= N {
      return Err(());
    }

    if self.pin_states[pin] != state {
      self.pin_states[pin] = state;
      self.stats.state_changes += 1;
    }

    self.stats.total_writes += 1;
    Ok(())
  }

  /// 获取引脚状态
  pub fn get_pin(&self, pin: usize) -> Result<PinState, ()> {
    if pin >= N {
      return Err(());
    }

    Ok(self.pin_states[pin])
  }

  /// 批量设置引脚状态
  pub fn set_pins(&mut self, mask: u32, states: u32) -> Result<(), ()> {
    for i in 0..N.min(32) {
      if (mask & (1 << i)) != 0 {
        let state = if (states & (1 << i)) != 0 {
          PinState::High
        } else {
          PinState::Low
        };

        self.set_pin(i, state)?;
      }
    }

    Ok(())
  }

  /// 切换引脚状态
  pub fn toggle_pin(&mut self, pin: usize) -> Result<(), ()> {
    if pin >= N {
      return Err(());
    }

    let new_state = match self.pin_states[pin] {
      PinState::Low => PinState::High,
      PinState::High => PinState::Low,
      PinState::HighZ => PinState::Low,
    };

    self.set_pin(pin, new_state)
  }

  /// 设置输出模式
  pub fn set_output_mode(&mut self, mode: OutputMode) {
    self.output_mode = mode;
  }

  /// 获取统计信息
  pub fn get_stats(&self) -> &OutputStats {
    &self.stats
  }

  /// 重置统计信息
  pub fn reset_stats(&mut self) {
    self.stats = OutputStats::default();
  }
}

/// 数字输入控制器
pub struct DigitalInputController<const N: usize> {
  /// 输入引脚状态
  pin_states: [PinState; N],
  /// 引脚配置
  pin_configs: [InputConfig; N],
  /// 边沿检测历史
  edge_history: [EdgeHistory; N],
  /// 防抖配置
  debounce_config: DebounceConfig,
  /// 统计信息
  stats: InputStats,
}

/// 输入配置
#[derive(Debug, Clone, Copy)]
pub struct InputConfig {
  pub mode: PinMode,
  pub edge_detection: EdgeDetection,
  pub enable_debounce: bool,
  pub enable_interrupt: bool,
}

/// 边沿历史
#[derive(Debug, Clone, Copy)]
pub struct EdgeHistory {
  pub last_state: PinState,
  pub edge_count: u16,
  pub last_edge_time: u32,
  pub stable_time: u32,
}

/// 防抖配置
#[derive(Debug, Clone, Copy)]
pub struct DebounceConfig {
  pub debounce_time_ms: u16,
  pub stable_count: u8,
  pub enable_hysteresis: bool,
}

/// 输入统计
#[derive(Debug, Default)]
pub struct InputStats {
  pub total_reads: u32,
  pub state_changes: u32,
  pub edge_detections: u32,
  pub debounce_events: u32,
}

impl Default for InputConfig {
  fn default() -> Self {
    Self {
      mode: PinMode::InputPullUp,
      edge_detection: EdgeDetection::Both,
      enable_debounce: true,
      enable_interrupt: false,
    }
  }
}

impl Default for DebounceConfig {
  fn default() -> Self {
    Self {
      debounce_time_ms: 50,
      stable_count: 3,
      enable_hysteresis: true,
    }
  }
}

impl Default for EdgeHistory {
  fn default() -> Self {
    Self {
      last_state: PinState::Low,
      edge_count: 0,
      last_edge_time: 0,
      stable_time: 0,
    }
  }
}

impl<const N: usize> DigitalInputController<N> {
  /// 创建新的输入控制器
  pub fn new(configs: [InputConfig; N], debounce_config: DebounceConfig) -> Self {
    Self {
      pin_states: [PinState::Low; N],
      pin_configs: configs,
      edge_history: [EdgeHistory::default(); N],
      debounce_config,
      stats: InputStats::default(),
    }
  }

  /// 读取单个引脚状态
  pub fn read_pin(&mut self, pin: usize) -> Result<PinState, ()> {
    if pin >= N {
      return Err(());
    }

    self.stats.total_reads += 1;
    Ok(self.pin_states[pin])
  }

  /// 更新引脚状态（由硬件层调用）
  pub fn update_pin_state(
    &mut self,
    pin: usize,
    new_state: PinState,
    timestamp: u32,
  ) -> Result<bool, ()> {
    if pin >= N {
      return Err(());
    }

    let old_state = self.pin_states[pin];
    let config = self.pin_configs[pin];
    let mut edge_detected = false;

    // 检测边沿
    if old_state != new_state {
      let edge_type = match (old_state, new_state) {
        (PinState::Low, PinState::High) => Some(EdgeDetection::Rising),
        (PinState::High, PinState::Low) => Some(EdgeDetection::Falling),
        _ => None,
      };

      if let Some(edge) = edge_type {
        if self.should_detect_edge(config.edge_detection, edge) {
          edge_detected = true;
          self.stats.edge_detections += 1;

          // 更新边沿历史
          self.edge_history[pin].edge_count += 1;
          self.edge_history[pin].last_edge_time = timestamp;
        }
      }
    }

    // 防抖处理
    if config.enable_debounce {
      if self.process_debounce(pin, new_state, timestamp) {
        self.pin_states[pin] = new_state;
        self.stats.state_changes += 1;
      }
    } else {
      self.pin_states[pin] = new_state;
      if old_state != new_state {
        self.stats.state_changes += 1;
      }
    }

    self.edge_history[pin].last_state = new_state;

    Ok(edge_detected)
  }

  /// 处理防抖
  fn process_debounce(&mut self, pin: usize, new_state: PinState, timestamp: u32) -> bool {
    let history = &mut self.edge_history[pin];
    let config = &self.debounce_config;

    // 检查状态稳定时间
    let time_since_last_edge = timestamp.saturating_sub(history.last_edge_time);

    if time_since_last_edge >= config.debounce_time_ms as u32 {
      // 状态已稳定足够时间
      history.stable_time = time_since_last_edge;
      self.stats.debounce_events += 1;
      true
    } else {
      // 状态还在抖动
      false
    }
  }

  /// 检查是否应该检测边沿
  fn should_detect_edge(&self, config_edge: EdgeDetection, detected_edge: EdgeDetection) -> bool {
    match config_edge {
      EdgeDetection::None => false,
      EdgeDetection::Rising => detected_edge == EdgeDetection::Rising,
      EdgeDetection::Falling => detected_edge == EdgeDetection::Falling,
      EdgeDetection::Both => true,
    }
  }

  /// 批量读取引脚状态
  pub fn read_pins(&mut self, mask: u32) -> Result<u32, ()> {
    let mut result = 0u32;

    for i in 0..N.min(32) {
      if (mask & (1 << i)) != 0 {
        if self.read_pin(i)? == PinState::High {
          result |= 1 << i;
        }
      }
    }

    Ok(result)
  }

  /// 获取边沿历史
  pub fn get_edge_history(&self, pin: usize) -> Result<&EdgeHistory, ()> {
    if pin >= N {
      return Err(());
    }

    Ok(&self.edge_history[pin])
  }

  /// 获取统计信息
  pub fn get_stats(&self) -> &InputStats {
    &self.stats
  }

  /// 重置统计信息
  pub fn reset_stats(&mut self) {
    self.stats = InputStats::default();
  }
}

/// 矩阵键盘控制器
pub struct MatrixKeypadController<const ROWS: usize, const COLS: usize> {
  /// 按键状态矩阵
  key_states: [[bool; COLS]; ROWS],
  /// 按键历史
  key_history: [[KeyHistory; COLS]; ROWS],
  /// 扫描配置
  scan_config: ScanConfig,
  /// 防抖配置
  debounce_config: DebounceConfig,
  /// 统计信息
  stats: KeypadStats,
}

/// 按键历史
#[derive(Debug, Clone, Copy)]
pub struct KeyHistory {
  pub press_count: u16,
  pub last_press_time: u32,
  pub last_release_time: u32,
  pub hold_time: u32,
}

/// 扫描配置
#[derive(Debug, Clone, Copy)]
pub struct ScanConfig {
  pub scan_interval_ms: u16,
  pub hold_threshold_ms: u16,
  pub repeat_interval_ms: u16,
  pub enable_ghost_detection: bool,
}

/// 键盘统计
#[derive(Debug, Default)]
pub struct KeypadStats {
  pub total_scans: u32,
  pub key_presses: u32,
  pub key_releases: u32,
  pub ghost_detections: u32,
}

impl Default for KeyHistory {
  fn default() -> Self {
    Self {
      press_count: 0,
      last_press_time: 0,
      last_release_time: 0,
      hold_time: 0,
    }
  }
}

impl Default for ScanConfig {
  fn default() -> Self {
    Self {
      scan_interval_ms: 10,
      hold_threshold_ms: 500,
      repeat_interval_ms: 100,
      enable_ghost_detection: true,
    }
  }
}

impl<const ROWS: usize, const COLS: usize> MatrixKeypadController<ROWS, COLS> {
  /// 创建新的矩阵键盘控制器
  pub fn new(scan_config: ScanConfig, debounce_config: DebounceConfig) -> Self {
    Self {
      key_states: [[false; COLS]; ROWS],
      key_history: [[KeyHistory::default(); COLS]; ROWS],
      scan_config,
      debounce_config,
      stats: KeypadStats::default(),
    }
  }

  /// 扫描键盘矩阵
  pub fn scan_matrix(&mut self, row_states: [bool; ROWS], timestamp: u32) -> Vec<KeyEvent, 16> {
    let mut events = Vec::new();
    self.stats.total_scans += 1;

    for row in 0..ROWS {
      if row_states[row] {
        // 该行有按键按下，扫描列
        for col in 0..COLS {
          let key_pressed = self.read_key_state(row, col);
          let old_state = self.key_states[row][col];

          if key_pressed != old_state {
            // 状态发生变化
            if self.process_key_debounce(row, col, key_pressed, timestamp) {
              self.key_states[row][col] = key_pressed;

              let event = if key_pressed {
                self.stats.key_presses += 1;
                self.key_history[row][col].press_count += 1;
                self.key_history[row][col].last_press_time = timestamp;
                KeyEvent::Press { row, col }
              } else {
                self.stats.key_releases += 1;
                self.key_history[row][col].last_release_time = timestamp;
                self.key_history[row][col].hold_time =
                  timestamp.saturating_sub(self.key_history[row][col].last_press_time);
                KeyEvent::Release { row, col }
              };

              let _ = events.push(event);
            }
          } else if key_pressed {
            // 按键持续按下
            let hold_time = timestamp.saturating_sub(self.key_history[row][col].last_press_time);
            if hold_time >= self.scan_config.hold_threshold_ms as u32 {
              let _ = events.push(KeyEvent::Hold {
                row,
                col,
                duration: hold_time,
              });
            }
          }
        }
      }
    }

    // 检测幽灵按键
    if self.scan_config.enable_ghost_detection {
      if self.detect_ghost_keys() {
        self.stats.ghost_detections += 1;
        let _ = events.push(KeyEvent::Ghost);
      }
    }

    events
  }

  /// 读取按键状态（模拟）
  fn read_key_state(&self, _row: usize, _col: usize) -> bool {
    // 在实际应用中，这里应该读取硬件状态
    false
  }

  /// 处理按键防抖
  fn process_key_debounce(
    &mut self,
    row: usize,
    col: usize,
    new_state: bool,
    timestamp: u32,
  ) -> bool {
    let history = &self.key_history[row][col];
    let last_change_time = if new_state {
      history.last_release_time
    } else {
      history.last_press_time
    };

    let time_since_change = timestamp.saturating_sub(last_change_time);
    time_since_change >= self.debounce_config.debounce_time_ms as u32
  }

  /// 检测幽灵按键
  fn detect_ghost_keys(&self) -> bool {
    let mut pressed_rows = Vec::<usize, ROWS>::new();
    let mut pressed_cols = Vec::<usize, COLS>::new();

    // 收集按下的行和列
    for row in 0..ROWS {
      for col in 0..COLS {
        if self.key_states[row][col] {
          if !pressed_rows.contains(&row) {
            let _ = pressed_rows.push(row);
          }
          if !pressed_cols.contains(&col) {
            let _ = pressed_cols.push(col);
          }
        }
      }
    }

    // 如果按下的行数和列数都大于1，可能存在幽灵按键
    pressed_rows.len() > 1 && pressed_cols.len() > 1
  }

  /// 获取按键状态
  pub fn get_key_state(&self, row: usize, col: usize) -> Result<bool, ()> {
    if row >= ROWS || col >= COLS {
      return Err(());
    }

    Ok(self.key_states[row][col])
  }

  /// 获取按键历史
  pub fn get_key_history(&self, row: usize, col: usize) -> Result<&KeyHistory, ()> {
    if row >= ROWS || col >= COLS {
      return Err(());
    }

    Ok(&self.key_history[row][col])
  }

  /// 获取统计信息
  pub fn get_stats(&self) -> &KeypadStats {
    &self.stats
  }
}

/// 按键事件
#[derive(Debug, Clone, Copy)]
pub enum KeyEvent {
  Press {
    row: usize,
    col: usize,
  },
  Release {
    row: usize,
    col: usize,
  },
  Hold {
    row: usize,
    col: usize,
    duration: u32,
  },
  Ghost,
}

/// 移位寄存器控制器
pub struct ShiftRegisterController<const N: usize> {
  /// 数据缓冲区
  data_buffer: [u8; N],
  /// 配置参数
  config: ShiftRegisterConfig,
  /// 统计信息
  stats: ShiftRegisterStats,
}

/// 移位寄存器配置
#[derive(Debug, Clone, Copy)]
pub struct ShiftRegisterConfig {
  pub clock_polarity: ClockPolarity,
  pub data_order: DataOrder,
  pub latch_mode: LatchMode,
  pub enable_output: bool,
}

/// 时钟极性
#[derive(Debug, Clone, Copy)]
pub enum ClockPolarity {
  IdleLow,
  IdleHigh,
}

/// 数据顺序
#[derive(Debug, Clone, Copy)]
pub enum DataOrder {
  MsbFirst,
  LsbFirst,
}

/// 锁存模式
#[derive(Debug, Clone, Copy)]
pub enum LatchMode {
  Transparent,
  Latched,
}

/// 移位寄存器统计
#[derive(Debug, Default)]
pub struct ShiftRegisterStats {
  pub bytes_shifted: u32,
  pub latch_operations: u32,
  pub error_count: u32,
}

impl Default for ShiftRegisterConfig {
  fn default() -> Self {
    Self {
      clock_polarity: ClockPolarity::IdleLow,
      data_order: DataOrder::MsbFirst,
      latch_mode: LatchMode::Latched,
      enable_output: true,
    }
  }
}

impl<const N: usize> ShiftRegisterController<N> {
  /// 创建新的移位寄存器控制器
  pub fn new(config: ShiftRegisterConfig) -> Self {
    Self {
      data_buffer: [0; N],
      config,
      stats: ShiftRegisterStats::default(),
    }
  }

  /// 设置数据
  pub fn set_data(&mut self, data: &[u8]) -> Result<(), ()> {
    if data.len() > N {
      return Err(());
    }

    self.data_buffer[..data.len()].copy_from_slice(data);
    Ok(())
  }

  /// 移位输出数据
  pub fn shift_out(&mut self) -> Result<(), ()> {
    // 模拟移位输出过程
    for &byte in &self.data_buffer {
      self.shift_byte(byte)?;
    }

    self.stats.bytes_shifted += N as u32;
    Ok(())
  }

  /// 移位单个字节
  fn shift_byte(&mut self, byte: u8) -> Result<(), ()> {
    let bits = match self.config.data_order {
      DataOrder::MsbFirst => (0..8).rev().collect::<Vec<_, 8>>(),
      DataOrder::LsbFirst => (0..8).collect::<Vec<_, 8>>(),
    };

    for bit_pos in bits {
      let bit_value = (byte >> bit_pos) & 1 != 0;
      self.output_bit(bit_value)?;
      self.clock_pulse()?;
    }

    Ok(())
  }

  /// 输出单个位
  fn output_bit(&self, _bit: bool) -> Result<(), ()> {
    // 在实际应用中，这里应该控制数据引脚
    Ok(())
  }

  /// 时钟脉冲
  fn clock_pulse(&self) -> Result<(), ()> {
    // 在实际应用中，这里应该控制时钟引脚
    Ok(())
  }

  /// 锁存数据
  pub fn latch(&mut self) -> Result<(), ()> {
    if matches!(self.config.latch_mode, LatchMode::Latched) {
      // 在实际应用中，这里应该控制锁存引脚
      self.stats.latch_operations += 1;
    }

    Ok(())
  }

  /// 获取统计信息
  pub fn get_stats(&self) -> &ShiftRegisterStats {
    &self.stats
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_digital_output_controller() {
    let configs = [OutputConfig::default(); 8];
    let mut controller = DigitalOutputController::new(configs);

    // 测试设置引脚
    assert!(controller.set_pin(0, PinState::High).is_ok());
    assert_eq!(controller.get_pin(0).unwrap(), PinState::High);

    // 测试切换引脚
    assert!(controller.toggle_pin(0).is_ok());
    assert_eq!(controller.get_pin(0).unwrap(), PinState::Low);

    // 测试批量设置
    assert!(controller.set_pins(0xFF, 0xAA).is_ok());
    assert_eq!(controller.get_pin(1).unwrap(), PinState::High);
    assert_eq!(controller.get_pin(2).unwrap(), PinState::Low);
  }

  #[test]
  fn test_digital_input_controller() {
    let configs = [InputConfig::default(); 8];
    let debounce_config = DebounceConfig::default();
    let mut controller = DigitalInputController::new(configs, debounce_config);

    // 测试状态更新
    assert!(controller.update_pin_state(0, PinState::High, 100).is_ok());

    // 测试读取
    assert_eq!(controller.read_pin(0).unwrap(), PinState::High);
  }

  #[test]
  fn test_matrix_keypad() {
    let scan_config = ScanConfig::default();
    let debounce_config = DebounceConfig::default();
    let mut keypad = MatrixKeypadController::<4, 4>::new(scan_config, debounce_config);

    // 测试扫描
    let row_states = [true, false, false, false];
    let events = keypad.scan_matrix(row_states, 1000);

    // 验证没有按键事件（因为是模拟）
    assert!(events.is_empty());
  }

  #[test]
  fn test_shift_register() {
    let config = ShiftRegisterConfig::default();
    let mut controller = ShiftRegisterController::<4>::new(config);

    // 测试设置数据
    let data = [0xAA, 0x55, 0xFF, 0x00];
    assert!(controller.set_data(&data).is_ok());

    // 测试移位输出
    assert!(controller.shift_out().is_ok());
    assert!(controller.latch().is_ok());

    // 验证统计
    assert_eq!(controller.get_stats().bytes_shifted, 4);
    assert_eq!(controller.get_stats().latch_operations, 1);
  }
}
