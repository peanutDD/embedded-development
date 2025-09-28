#![no_std]
#![no_main]

//! # 矩阵键盘扫描示例
//!
//! 演示4x4矩阵键盘的扫描和处理：
//! - 行列扫描算法
//! - 按键防抖
//! - 多键检测
//! - 按键映射

use core::cell::RefCell;
use cortex_m_rt::entry;
use critical_section::Mutex;
use heapless::{String, Vec};
use panic_probe as _;
use stm32f4xx_hal::{
  gpio::{
    gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
    gpioc::PC13,
    Input, Output, PullUp, PushPull,
  },
  pac,
  prelude::*,
};

// 矩阵键盘配置
const ROWS: usize = 4;
const COLS: usize = 4;
const DEBOUNCE_TIME: u32 = 20; // 20ms
const LONG_PRESS_TIME: u32 = 1000; // 1秒
const REPEAT_TIME: u32 = 200; // 200ms重复间隔

// 全局键盘状态
static KEYPAD_STATE: Mutex<RefCell<KeypadState>> = Mutex::new(RefCell::new(KeypadState::new()));

/// 键盘状态
#[derive(Debug, Clone)]
pub struct KeypadState {
  pub current_keys: [[bool; COLS]; ROWS],
  pub last_keys: [[bool; COLS]; ROWS],
  pub stable_keys: [[bool; COLS]; ROWS],
  pub key_press_time: [[u32; COLS]; ROWS],
  pub last_scan_time: u32,
  pub pressed_keys: Vec<KeyEvent, 16>,
  pub key_sequence: String<64>,
}

impl KeypadState {
  pub const fn new() -> Self {
    Self {
      current_keys: [[false; COLS]; ROWS],
      last_keys: [[false; COLS]; ROWS],
      stable_keys: [[false; COLS]; ROWS],
      key_press_time: [[0; COLS]; ROWS],
      last_scan_time: 0,
      pressed_keys: Vec::new(),
      key_sequence: String::new(),
    }
  }

  pub fn update_key(&mut self, row: usize, col: usize, pressed: bool) {
    if row < ROWS && col < COLS {
      self.current_keys[row][col] = pressed;
    }
  }

  pub fn get_pressed_count(&self) -> usize {
    let mut count = 0;
    for row in 0..ROWS {
      for col in 0..COLS {
        if self.stable_keys[row][col] {
          count += 1;
        }
      }
    }
    count
  }

  pub fn get_pressed_keys(&self) -> Vec<(usize, usize), 16> {
    let mut keys = Vec::new();
    for row in 0..ROWS {
      for col in 0..COLS {
        if self.stable_keys[row][col] {
          keys.push((row, col)).ok();
        }
      }
    }
    keys
  }

  pub fn add_to_sequence(&mut self, key_char: char) {
    if self.key_sequence.len() < self.key_sequence.capacity() - 1 {
      self.key_sequence.push(key_char).ok();
    }
  }

  pub fn clear_sequence(&mut self) {
    self.key_sequence.clear();
  }
}

/// 按键事件
#[derive(Debug, Clone, Copy)]
pub struct KeyEvent {
  pub row: u8,
  pub col: u8,
  pub event_type: KeyEventType,
  pub timestamp: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KeyEventType {
  Pressed,
  Released,
  LongPress,
  Repeat,
}

/// 矩阵键盘控制器
pub struct MatrixKeypad {
  // 行输出引脚 (驱动)
  pub row_pins: RowPins,
  // 列输入引脚 (读取)
  pub col_pins: ColPins,
  // 键盘映射
  pub key_map: [[char; COLS]; ROWS],
  // 扫描状态
  pub current_row: usize,
  pub scan_counter: u32,
}

pub struct RowPins {
  pub row0: PA0<Output<PushPull>>,
  pub row1: PA1<Output<PushPull>>,
  pub row2: PA2<Output<PushPull>>,
  pub row3: PA3<Output<PushPull>>,
}

pub struct ColPins {
  pub col0: PA4<Input<PullUp>>,
  pub col1: PA5<Input<PullUp>>,
  pub col2: PA6<Input<PullUp>>,
  pub col3: PA7<Input<PullUp>>,
}

impl MatrixKeypad {
  pub fn new(row_pins: RowPins, col_pins: ColPins) -> Self {
    // 标准4x4键盘布局
    let key_map = [
      ['1', '2', '3', 'A'],
      ['4', '5', '6', 'B'],
      ['7', '8', '9', 'C'],
      ['*', '0', '#', 'D'],
    ];

    Self {
      row_pins,
      col_pins,
      key_map,
      current_row: 0,
      scan_counter: 0,
    }
  }

  /// 扫描键盘
  pub fn scan(&mut self) -> Vec<KeyEvent, 16> {
    let mut events = Vec::new();
    self.scan_counter += 1;

    // 扫描所有行
    for row in 0..ROWS {
      // 设置当前行为低电平，其他行为高电平
      self.set_row_state(row, false);
      self.set_other_rows_state(row, true);

      // 短暂延时让信号稳定
      delay_us(10);

      // 读取所有列
      for col in 0..COLS {
        let pressed = !self.read_column(col); // 按键按下时为低电平

        // 更新键盘状态
        critical_section::with(|cs| {
          KEYPAD_STATE
            .borrow(cs)
            .borrow_mut()
            .update_key(row, col, pressed);
        });

        // 处理按键事件
        if let Some(event) = self.process_key_change(row, col, pressed) {
          events.push(event).ok();
        }
      }
    }

    // 所有行设为高电平
    self.set_all_rows_state(true);

    events
  }

  /// 处理按键状态变化
  fn process_key_change(&mut self, row: usize, col: usize, pressed: bool) -> Option<KeyEvent> {
    let current_time = get_system_time();

    critical_section::with(|cs| {
      let mut state = KEYPAD_STATE.borrow(cs).borrow_mut();

      // 防抖处理
      if pressed != state.last_keys[row][col] {
        state.last_keys[row][col] = pressed;
        state.last_scan_time = current_time;
        return None;
      }

      // 检查防抖时间
      if current_time - state.last_scan_time < DEBOUNCE_TIME {
        return None;
      }

      // 状态稳定且发生变化
      if pressed != state.stable_keys[row][col] {
        state.stable_keys[row][col] = pressed;

        if pressed {
          // 按键按下
          state.key_press_time[row][col] = current_time;

          // 添加到按键序列
          let key_char = self.key_map[row][col];
          state.add_to_sequence(key_char);

          return Some(KeyEvent {
            row: row as u8,
            col: col as u8,
            event_type: KeyEventType::Pressed,
            timestamp: current_time,
          });
        } else {
          // 按键释放
          return Some(KeyEvent {
            row: row as u8,
            col: col as u8,
            event_type: KeyEventType::Released,
            timestamp: current_time,
          });
        }
      }

      // 检查长按
      if pressed && state.stable_keys[row][col] {
        let press_duration = current_time - state.key_press_time[row][col];

        if press_duration > LONG_PRESS_TIME && press_duration % REPEAT_TIME == 0 {
          return Some(KeyEvent {
            row: row as u8,
            col: col as u8,
            event_type: if press_duration == LONG_PRESS_TIME {
              KeyEventType::LongPress
            } else {
              KeyEventType::Repeat
            },
            timestamp: current_time,
          });
        }
      }

      None
    })
  }

  /// 设置指定行的状态
  fn set_row_state(&mut self, row: usize, high: bool) {
    match row {
      0 => {
        if high {
          self.row_pins.row0.set_high();
        } else {
          self.row_pins.row0.set_low();
        }
      }
      1 => {
        if high {
          self.row_pins.row1.set_high();
        } else {
          self.row_pins.row1.set_low();
        }
      }
      2 => {
        if high {
          self.row_pins.row2.set_high();
        } else {
          self.row_pins.row2.set_low();
        }
      }
      3 => {
        if high {
          self.row_pins.row3.set_high();
        } else {
          self.row_pins.row3.set_low();
        }
      }
      _ => {}
    }
  }

  /// 设置除指定行外的其他行状态
  fn set_other_rows_state(&mut self, exclude_row: usize, high: bool) {
    for row in 0..ROWS {
      if row != exclude_row {
        self.set_row_state(row, high);
      }
    }
  }

  /// 设置所有行的状态
  fn set_all_rows_state(&mut self, high: bool) {
    for row in 0..ROWS {
      self.set_row_state(row, high);
    }
  }

  /// 读取指定列的状态
  fn read_column(&self, col: usize) -> bool {
    match col {
      0 => self.col_pins.col0.is_high(),
      1 => self.col_pins.col1.is_high(),
      2 => self.col_pins.col2.is_high(),
      3 => self.col_pins.col3.is_high(),
      _ => true,
    }
  }

  /// 获取按键字符
  pub fn get_key_char(&self, row: usize, col: usize) -> char {
    if row < ROWS && col < COLS {
      self.key_map[row][col]
    } else {
      '\0'
    }
  }

  /// 设置自定义键盘映射
  pub fn set_key_map(&mut self, key_map: [[char; COLS]; ROWS]) {
    self.key_map = key_map;
  }
}

/// LED显示控制器
pub struct LedDisplay {
  pub status_led: PC13<Output<PushPull>>,
  pub blink_counter: u32,
  pub display_mode: DisplayMode,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DisplayMode {
  Idle,
  KeyPressed,
  MultiKey,
  Sequence,
  Error,
}

impl LedDisplay {
  pub fn new(status_led: PC13<Output<PushPull>>) -> Self {
    Self {
      status_led,
      blink_counter: 0,
      display_mode: DisplayMode::Idle,
    }
  }

  pub fn set_mode(&mut self, mode: DisplayMode) {
    self.display_mode = mode;
    self.blink_counter = 0;
  }

  pub fn update(&mut self) {
    self.blink_counter += 1;

    match self.display_mode {
      DisplayMode::Idle => {
        // 慢速闪烁
        if self.blink_counter % 2000 < 100 {
          self.status_led.set_low();
        } else {
          self.status_led.set_high();
        }
      }
      DisplayMode::KeyPressed => {
        // 快速闪烁
        if self.blink_counter % 200 < 100 {
          self.status_led.set_low();
        } else {
          self.status_led.set_high();
        }
      }
      DisplayMode::MultiKey => {
        // 双闪模式
        let phase = self.blink_counter % 800;
        if phase < 100 || (phase >= 200 && phase < 300) {
          self.status_led.set_low();
        } else {
          self.status_led.set_high();
        }
      }
      DisplayMode::Sequence => {
        // 常亮
        self.status_led.set_low();
      }
      DisplayMode::Error => {
        // 快速三闪
        let phase = self.blink_counter % 1000;
        if phase < 100 || (phase >= 150 && phase < 250) || (phase >= 300 && phase < 400) {
          self.status_led.set_low();
        } else {
          self.status_led.set_high();
        }
      }
    }
  }
}

/// 键盘应用处理器
pub struct KeypadApplication {
  pub password: String<16>,
  pub input_buffer: String<16>,
  pub correct_password: &'static str,
  pub attempt_count: u8,
  pub locked: bool,
  pub lock_time: u32,
}

impl KeypadApplication {
  pub fn new(correct_password: &'static str) -> Self {
    Self {
      password: String::new(),
      input_buffer: String::new(),
      correct_password,
      attempt_count: 0,
      locked: false,
      lock_time: 0,
    }
  }

  pub fn process_key(&mut self, key_char: char) -> AppEvent {
    if self.locked {
      let current_time = get_system_time();
      if current_time - self.lock_time > 30000 {
        // 30秒锁定
        self.locked = false;
        self.attempt_count = 0;
      } else {
        return AppEvent::Locked;
      }
    }

    match key_char {
      '0'..='9' | 'A'..='D' => {
        // 数字和字母键
        if self.input_buffer.len() < self.input_buffer.capacity() - 1 {
          self.input_buffer.push(key_char).ok();
          AppEvent::InputChar(key_char)
        } else {
          AppEvent::BufferFull
        }
      }
      '#' => {
        // 确认键
        self.check_password()
      }
      '*' => {
        // 清除键
        self.input_buffer.clear();
        AppEvent::Cleared
      }
      _ => AppEvent::InvalidKey,
    }
  }

  fn check_password(&mut self) -> AppEvent {
    if self.input_buffer.as_str() == self.correct_password {
      self.input_buffer.clear();
      self.attempt_count = 0;
      AppEvent::PasswordCorrect
    } else {
      self.input_buffer.clear();
      self.attempt_count += 1;

      if self.attempt_count >= 3 {
        self.locked = true;
        self.lock_time = get_system_time();
        AppEvent::Locked
      } else {
        AppEvent::PasswordIncorrect(self.attempt_count)
      }
    }
  }

  pub fn get_input_length(&self) -> usize {
    self.input_buffer.len()
  }

  pub fn is_locked(&self) -> bool {
    self.locked
  }
}

#[derive(Debug, Clone, Copy)]
pub enum AppEvent {
  InputChar(char),
  PasswordCorrect,
  PasswordIncorrect(u8),
  Cleared,
  BufferFull,
  InvalidKey,
  Locked,
}

/// 键盘映射
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KeyCode {
  // 数字键
  Digit0,
  Digit1,
  Digit2,
  Digit3,
  Digit4,
  Digit5,
  Digit6,
  Digit7,
  Digit8,
  Digit9,

  // 功能键
  Star,
  Hash,
  Up,
  Down,
  Left,
  Right,
  Enter,
  Escape,

  // 自定义功能键
  Function1,
  Function2,
  Function3,
  Function4,
}

/// 按键动作
#[derive(Debug, Clone, Copy)]
pub enum KeyAction {
  /// 单次按下
  Press,
  /// 长按
  Hold,
  /// 释放
  Release,
  /// 双击
  DoubleClick,
  /// 组合键
  Combination(u8),
}

/// 键盘功能
#[derive(Debug, Clone, Copy)]
pub enum KeyboardFunction {
  /// 数字输入
  DigitInput(u8),
  /// 导航
  Navigation(NavigationDirection),
  /// 系统控制
  SystemControl(SystemCommand),
  /// 用户自定义
  UserDefined(u8),
}

/// 导航方向
#[derive(Debug, Clone, Copy)]
pub enum NavigationDirection {
  Up,
  Down,
  Left,
  Right,
}

/// 系统命令
#[derive(Debug, Clone, Copy)]
pub enum SystemCommand {
  Enter,
  Escape,
  Menu,
  Back,
  VolumeUp,
  VolumeDown,
  BrightnessUp,
  BrightnessDown,
}

/// 键盘控制器
pub struct KeyboardController {
  /// 矩阵键盘控制器
  keypad: MatrixKeypadController<4, 4>,
  /// 键盘映射
  key_mapping: KeyMapping,
  /// 组合键处理器
  combo_handler: ComboKeyHandler,
  /// 功能处理器
  function_handler: FunctionHandler,
  /// 统计信息
  stats: KeyboardStats,
}

/// 键盘映射
pub struct KeyMapping {
  /// 位置到键码的映射
  position_map: [[KeyCode; 4]; 4],
  /// 键码到功能的映射
  function_map: FnvIndexMap<KeyCode, KeyboardFunction, 16>,
}

/// 组合键处理器
pub struct ComboKeyHandler {
  /// 当前按下的键
  pressed_keys: Vec<(usize, usize), 8>,
  /// 组合键定义
  combo_definitions: Vec<ComboDefinition, 16>,
  /// 组合键状态
  combo_state: ComboState,
}

/// 组合键定义
#[derive(Debug, Clone)]
pub struct ComboDefinition {
  pub keys: Vec<KeyCode, 4>,
  pub function: KeyboardFunction,
  pub timeout_ms: u32,
}

/// 组合键状态
#[derive(Debug, Clone, Copy)]
pub enum ComboState {
  Idle,
  Collecting,
  Matched,
  Timeout,
}

/// 功能处理器
pub struct FunctionHandler {
  /// 当前模式
  current_mode: InputMode,
  /// 输入缓冲区
  input_buffer: String<32>,
  /// 光标位置
  cursor_position: usize,
  /// 处理状态
  processing_state: ProcessingState,
}

/// 输入模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InputMode {
  Normal,
  Numeric,
  Navigation,
  Menu,
}

/// 处理状态
#[derive(Debug, Clone, Copy)]
pub struct ProcessingState {
  pub last_key_time: u32,
  pub repeat_count: u16,
  pub modifier_active: bool,
}

/// 键盘统计
#[derive(Debug, Default)]
pub struct KeyboardStats {
  pub total_key_presses: u32,
  pub combo_activations: u32,
  pub function_calls: u32,
  pub input_characters: u32,
  pub navigation_commands: u32,
}

impl Default for KeyMapping {
  fn default() -> Self {
    // 标准4x4键盘布局
    let position_map = [
      [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Up,
      ],
      [
        KeyCode::Digit4,
        KeyCode::Digit5,
        KeyCode::Digit6,
        KeyCode::Down,
      ],
      [
        KeyCode::Digit7,
        KeyCode::Digit8,
        KeyCode::Digit9,
        KeyCode::Left,
      ],
      [
        KeyCode::Star,
        KeyCode::Digit0,
        KeyCode::Hash,
        KeyCode::Right,
      ],
    ];

    let mut function_map = FnvIndexMap::new();

    // 数字键映射
    let _ = function_map.insert(KeyCode::Digit0, KeyboardFunction::DigitInput(0));
    let _ = function_map.insert(KeyCode::Digit1, KeyboardFunction::DigitInput(1));
    let _ = function_map.insert(KeyCode::Digit2, KeyboardFunction::DigitInput(2));
    let _ = function_map.insert(KeyCode::Digit3, KeyboardFunction::DigitInput(3));
    let _ = function_map.insert(KeyCode::Digit4, KeyboardFunction::DigitInput(4));
    let _ = function_map.insert(KeyCode::Digit5, KeyboardFunction::DigitInput(5));
    let _ = function_map.insert(KeyCode::Digit6, KeyboardFunction::DigitInput(6));
    let _ = function_map.insert(KeyCode::Digit7, KeyboardFunction::DigitInput(7));
    let _ = function_map.insert(KeyCode::Digit8, KeyboardFunction::DigitInput(8));
    let _ = function_map.insert(KeyCode::Digit9, KeyboardFunction::DigitInput(9));

    // 导航键映射
    let _ = function_map.insert(
      KeyCode::Up,
      KeyboardFunction::Navigation(NavigationDirection::Up),
    );
    let _ = function_map.insert(
      KeyCode::Down,
      KeyboardFunction::Navigation(NavigationDirection::Down),
    );
    let _ = function_map.insert(
      KeyCode::Left,
      KeyboardFunction::Navigation(NavigationDirection::Left),
    );
    let _ = function_map.insert(
      KeyCode::Right,
      KeyboardFunction::Navigation(NavigationDirection::Right),
    );

    // 功能键映射
    let _ = function_map.insert(
      KeyCode::Star,
      KeyboardFunction::SystemControl(SystemCommand::Menu),
    );
    let _ = function_map.insert(
      KeyCode::Hash,
      KeyboardFunction::SystemControl(SystemCommand::Enter),
    );

    Self {
      position_map,
      function_map,
    }
  }
}

impl ComboKeyHandler {
  /// 创建新的组合键处理器
  pub fn new() -> Self {
    let mut combo_definitions = Vec::new();

    // 定义一些常用组合键
    // * + # = 菜单
    let mut menu_combo = Vec::new();
    let _ = menu_combo.push(KeyCode::Star);
    let _ = menu_combo.push(KeyCode::Hash);

    let menu_def = ComboDefinition {
      keys: menu_combo,
      function: KeyboardFunction::SystemControl(SystemCommand::Menu),
      timeout_ms: 1000,
    };
    let _ = combo_definitions.push(menu_def);

    // 1 + 3 = 音量增加
    let mut vol_up_combo = Vec::new();
    let _ = vol_up_combo.push(KeyCode::Digit1);
    let _ = vol_up_combo.push(KeyCode::Digit3);

    let vol_up_def = ComboDefinition {
      keys: vol_up_combo,
      function: KeyboardFunction::SystemControl(SystemCommand::VolumeUp),
      timeout_ms: 500,
    };
    let _ = combo_definitions.push(vol_up_def);

    Self {
      pressed_keys: Vec::new(),
      combo_definitions,
      combo_state: ComboState::Idle,
    }
  }

  /// 处理按键按下
  pub fn handle_key_press(
    &mut self,
    row: usize,
    col: usize,
    key_code: KeyCode,
  ) -> Option<KeyboardFunction> {
    // 添加到按下的键列表
    if self.pressed_keys.push((row, col)).is_err() {
      // 列表满，清除最旧的
      self.pressed_keys.remove(0);
      let _ = self.pressed_keys.push((row, col));
    }

    // 检查组合键匹配
    self.check_combo_match()
  }

  /// 处理按键释放
  pub fn handle_key_release(&mut self, row: usize, col: usize) {
    // 从按下的键列表中移除
    self.pressed_keys.retain(|&(r, c)| r != row || c != col);

    // 如果没有按键按下，重置状态
    if self.pressed_keys.is_empty() {
      self.combo_state = ComboState::Idle;
    }
  }

  /// 检查组合键匹配
  fn check_combo_match(&mut self) -> Option<KeyboardFunction> {
    for combo_def in &self.combo_definitions {
      if self.pressed_keys.len() == combo_def.keys.len() {
        // 检查是否所有键都匹配
        let mut all_match = true;
        for &key_code in &combo_def.keys {
          let mut found = false;
          for &(row, col) in &self.pressed_keys {
            // 这里需要从位置获取键码，简化处理
            found = true;
            break;
          }
          if !found {
            all_match = false;
            break;
          }
        }

        if all_match {
          self.combo_state = ComboState::Matched;
          return Some(combo_def.function);
        }
      }
    }

    None
  }

  /// 处理超时
  pub fn handle_timeout(&mut self) {
    if matches!(self.combo_state, ComboState::Collecting) {
      self.combo_state = ComboState::Timeout;
      self.pressed_keys.clear();
    }
  }
}

impl FunctionHandler {
  /// 创建新的功能处理器
  pub fn new() -> Self {
    Self {
      current_mode: InputMode::Normal,
      input_buffer: String::new(),
      cursor_position: 0,
      processing_state: ProcessingState {
        last_key_time: 0,
        repeat_count: 0,
        modifier_active: false,
      },
    }
  }

  /// 处理键盘功能
  pub fn handle_function(&mut self, function: KeyboardFunction, timestamp: u32) -> Result<(), ()> {
    self.processing_state.last_key_time = timestamp;

    match function {
      KeyboardFunction::DigitInput(digit) => {
        self.handle_digit_input(digit)?;
      }
      KeyboardFunction::Navigation(direction) => {
        self.handle_navigation(direction)?;
      }
      KeyboardFunction::SystemControl(command) => {
        self.handle_system_command(command)?;
      }
      KeyboardFunction::UserDefined(code) => {
        self.handle_user_defined(code)?;
      }
    }

    Ok(())
  }

  /// 处理数字输入
  fn handle_digit_input(&mut self, digit: u8) -> Result<(), ()> {
    match self.current_mode {
      InputMode::Numeric | InputMode::Normal => {
        // 添加数字到输入缓冲区
        let digit_char = (b'0' + digit) as char;
        if self.input_buffer.push(digit_char).is_err() {
          // 缓冲区满，移除第一个字符
          self.input_buffer.remove(0);
          let _ = self.input_buffer.push(digit_char);
        }
        self.cursor_position = self.input_buffer.len();
      }
      _ => {
        // 其他模式下数字键可能有不同功能
      }
    }

    Ok(())
  }

  /// 处理导航
  fn handle_navigation(&mut self, direction: NavigationDirection) -> Result<(), ()> {
    match self.current_mode {
      InputMode::Navigation | InputMode::Normal => {
        match direction {
          NavigationDirection::Left => {
            if self.cursor_position > 0 {
              self.cursor_position -= 1;
            }
          }
          NavigationDirection::Right => {
            if self.cursor_position < self.input_buffer.len() {
              self.cursor_position += 1;
            }
          }
          NavigationDirection::Up => {
            // 在菜单模式下可能是上一项
          }
          NavigationDirection::Down => {
            // 在菜单模式下可能是下一项
          }
        }
      }
      _ => {}
    }

    Ok(())
  }

  /// 处理系统命令
  fn handle_system_command(&mut self, command: SystemCommand) -> Result<(), ()> {
    match command {
      SystemCommand::Enter => {
        // 确认当前输入或选择
        self.process_input()?;
      }
      SystemCommand::Escape => {
        // 取消或返回
        self.clear_input();
        self.current_mode = InputMode::Normal;
      }
      SystemCommand::Menu => {
        // 切换到菜单模式
        self.current_mode = InputMode::Menu;
      }
      SystemCommand::Back => {
        // 删除字符
        if self.cursor_position > 0 {
          self.input_buffer.remove(self.cursor_position - 1);
          self.cursor_position -= 1;
        }
      }
      SystemCommand::VolumeUp
      | SystemCommand::VolumeDown
      | SystemCommand::BrightnessUp
      | SystemCommand::BrightnessDown => {
        // 系统控制命令
        self.execute_system_control(command)?;
      }
    }

    Ok(())
  }

  /// 处理用户自定义功能
  fn handle_user_defined(&mut self, code: u8) -> Result<(), ()> {
    // 用户自定义功能处理
    match code {
      1 => {
        // 切换输入模式
        self.current_mode = match self.current_mode {
          InputMode::Normal => InputMode::Numeric,
          InputMode::Numeric => InputMode::Navigation,
          InputMode::Navigation => InputMode::Menu,
          InputMode::Menu => InputMode::Normal,
        };
      }
      2 => {
        // 清除输入
        self.clear_input();
      }
      _ => {}
    }

    Ok(())
  }

  /// 处理输入
  fn process_input(&mut self) -> Result<(), ()> {
    match self.current_mode {
      InputMode::Numeric => {
        // 处理数字输入
        if let Ok(number) = self.input_buffer.parse::<u32>() {
          // 处理数字
          self.clear_input();
        }
      }
      InputMode::Menu => {
        // 处理菜单选择
      }
      _ => {}
    }

    Ok(())
  }

  /// 执行系统控制
  fn execute_system_control(&mut self, command: SystemCommand) -> Result<(), ()> {
    // 在实际应用中，这里会调用相应的系统功能
    match command {
      SystemCommand::VolumeUp => {
        // 增加音量
      }
      SystemCommand::VolumeDown => {
        // 减少音量
      }
      SystemCommand::BrightnessUp => {
        // 增加亮度
      }
      SystemCommand::BrightnessDown => {
        // 减少亮度
      }
      _ => {}
    }

    Ok(())
  }

  /// 清除输入
  fn clear_input(&mut self) {
    self.input_buffer.clear();
    self.cursor_position = 0;
  }

  /// 获取当前输入
  pub fn get_current_input(&self) -> &str {
    &self.input_buffer
  }

  /// 获取当前模式
  pub fn get_current_mode(&self) -> InputMode {
    self.current_mode
  }
}

impl KeyboardController {
  /// 创建新的键盘控制器
  pub fn new() -> Self {
    let scan_config = ScanConfig {
      scan_interval_ms: 10,
      hold_threshold_ms: 500,
      repeat_interval_ms: 100,
      enable_ghost_detection: true,
    };

    let debounce_config = DebounceConfig {
      debounce_time_ms: 50,
      stable_count: 3,
      enable_hysteresis: true,
    };

    Self {
      keypad: MatrixKeypadController::new(scan_config, debounce_config),
      key_mapping: KeyMapping::default(),
      combo_handler: ComboKeyHandler::new(),
      function_handler: FunctionHandler::new(),
      stats: KeyboardStats::default(),
    }
  }

  /// 处理键盘扫描
  pub fn process(&mut self, timestamp: u32) -> Result<(), ()> {
    // 模拟行扫描
    let row_states = self.scan_keyboard_rows(timestamp);
    let events = self.keypad.scan_matrix(row_states, timestamp);

    // 处理键盘事件
    for event in events {
      self.handle_keypad_event(event, timestamp)?;
    }

    // 处理组合键超时
    self.combo_handler.handle_timeout();

    Ok(())
  }

  /// 扫描键盘行（模拟）
  fn scan_keyboard_rows(&self, timestamp: u32) -> [bool; 4] {
    // 模拟键盘活动
    let time = timestamp / 1000;
    [
      (time % 10) < 2, // 第0行有20%的时间有按键
      (time % 15) < 1, // 第1行有6.7%的时间有按键
      (time % 20) < 1, // 第2行有5%的时间有按键
      (time % 25) < 1, // 第3行有4%的时间有按键
    ]
  }

  /// 处理键盘事件
  fn handle_keypad_event(&mut self, event: KeyEvent, timestamp: u32) -> Result<(), ()> {
    match event {
      KeyEvent::Press { row, col } => {
        self.handle_key_press(row, col, timestamp)?;
      }
      KeyEvent::Release { row, col } => {
        self.handle_key_release(row, col, timestamp)?;
      }
      KeyEvent::Hold { row, col, duration } => {
        self.handle_key_hold(row, col, duration, timestamp)?;
      }
      KeyEvent::Ghost => {
        self.handle_ghost_detection(timestamp)?;
      }
    }

    Ok(())
  }

  /// 处理按键按下
  fn handle_key_press(&mut self, row: usize, col: usize, timestamp: u32) -> Result<(), ()> {
    if row >= 4 || col >= 4 {
      return Err(());
    }

    let key_code = self.key_mapping.position_map[row][col];
    self.stats.total_key_presses += 1;

    // 检查组合键
    if let Some(combo_function) = self.combo_handler.handle_key_press(row, col, key_code) {
      self
        .function_handler
        .handle_function(combo_function, timestamp)?;
      self.stats.combo_activations += 1;
      return Ok(());
    }

    // 处理单键功能
    if let Some(&function) = self.key_mapping.function_map.get(&key_code) {
      self.function_handler.handle_function(function, timestamp)?;
      self.stats.function_calls += 1;
    }

    Ok(())
  }

  /// 处理按键释放
  fn handle_key_release(&mut self, row: usize, col: usize, timestamp: u32) -> Result<(), ()> {
    self.combo_handler.handle_key_release(row, col);
    Ok(())
  }

  /// 处理按键长按
  fn handle_key_hold(
    &mut self,
    row: usize,
    col: usize,
    duration: u32,
    timestamp: u32,
  ) -> Result<(), ()> {
    if row >= 4 || col >= 4 {
      return Err(());
    }

    let key_code = self.key_mapping.position_map[row][col];

    // 长按可能触发不同的功能
    match key_code {
      KeyCode::Star => {
        // 长按*键进入设置模式
        self.function_handler.handle_function(
          KeyboardFunction::SystemControl(SystemCommand::Menu),
          timestamp,
        )?;
      }
      KeyCode::Hash => {
        // 长按#键清除输入
        self.function_handler.handle_function(
          KeyboardFunction::SystemControl(SystemCommand::Back),
          timestamp,
        )?;
      }
      _ => {
        // 其他键的长按可能是重复输入
        if let Some(&function) = self.key_mapping.function_map.get(&key_code) {
          self.function_handler.handle_function(function, timestamp)?;
        }
      }
    }

    Ok(())
  }

  /// 处理幽灵按键检测
  fn handle_ghost_detection(&mut self, timestamp: u32) -> Result<(), ()> {
    // 幽灵按键检测到时，可能需要忽略当前的按键输入
    // 或者发出警告
    Ok(())
  }

  /// 获取当前输入
  pub fn get_current_input(&self) -> &str {
    self.function_handler.get_current_input()
  }

  /// 获取当前模式
  pub fn get_current_mode(&self) -> InputMode {
    self.function_handler.get_current_mode()
  }

  /// 获取统计信息
  pub fn get_stats(&self) -> &KeyboardStats {
    &self.stats
  }

  /// 获取详细统计
  pub fn get_detailed_stats(&self) -> DetailedKeyboardStats {
    DetailedKeyboardStats {
      keyboard: self.stats,
      keypad: *self.keypad.get_stats(),
      input_length: self.function_handler.input_buffer.len(),
      current_mode: self.function_handler.current_mode,
    }
  }
}

/// 详细键盘统计
#[derive(Debug)]
pub struct DetailedKeyboardStats {
  pub keyboard: KeyboardStats,
  pub keypad: digital_io::KeypadStats,
  pub input_length: usize,
  pub current_mode: InputMode,
}

/// 全局键盘控制器
static KEYBOARD_CONTROLLER: Mutex<RefCell<Option<KeyboardController>>> =
  Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
  // 初始化键盘控制器
  let mut controller = KeyboardController::new();

  // 存储到全局变量
  critical_section::with(|cs| {
    KEYBOARD_CONTROLLER.borrow(cs).replace(Some(controller));
  });

  let mut timestamp = 0u32;

  loop {
    timestamp = timestamp.wrapping_add(1);

    // 处理键盘
    critical_section::with(|cs| {
      if let Some(ref mut controller) = KEYBOARD_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
        let _ = controller.process(timestamp);

        // 每1000次循环输出状态
        if timestamp % 1000 == 0 {
          let stats = controller.get_detailed_stats();
          let input = controller.get_current_input();
          let mode = controller.get_current_mode();

          // 在实际应用中，这里可以通过显示器或串口输出状态
          // 例如：显示当前输入、模式、统计信息等
        }
      }
    });

    // 简单延时
    for _ in 0..1000 {
      cortex_m::asm::nop();
    }
  }
}
