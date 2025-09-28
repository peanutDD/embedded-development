#![no_std]
#![no_main]

//! # 数字输出控制程序
//!
//! 专门演示数字输出控制的各种功能：
//! - 单独引脚控制
//! - 批量输出控制
//! - 输出模式切换
//! - 驱动强度调节

use core::cell::RefCell;
use cortex_m_rt::entry;
use critical_section::Mutex;
use heapless::{String, Vec};
use panic_halt as _;

use digital_io::{
  DigitalOutputController, DriveStrength, OutputConfig, OutputMode, PinMode, PinState, SlewRate,
};

/// 输出控制器配置
pub struct OutputControllerConfig {
  /// 输出模式
  pub mode: OutputMode,
  /// 更新频率
  pub update_frequency: u32,
  /// 模式参数
  pub pattern_params: PatternParams,
}

/// 模式参数
#[derive(Debug, Clone, Copy)]
pub struct PatternParams {
  pub speed: u32,
  pub direction: Direction,
  pub intensity: u8,
  pub repeat_count: u16,
}

/// 方向
#[derive(Debug, Clone, Copy)]
pub enum Direction {
  Forward,
  Backward,
  Bidirectional,
}

/// 输出控制器
pub struct OutputController {
  /// 数字输出控制器
  digital_output: DigitalOutputController<32>,
  /// 配置
  config: OutputControllerConfig,
  /// 当前模式状态
  mode_state: ModeState,
  /// 统计信息
  stats: ControllerStats,
}

/// 模式状态
#[derive(Debug)]
pub struct ModeState {
  pub current_pattern: u32,
  pub step_counter: u32,
  pub direction_flag: bool,
  pub repeat_counter: u16,
  pub last_update_time: u32,
}

/// 控制器统计
#[derive(Debug, Default)]
pub struct ControllerStats {
  pub total_updates: u32,
  pub pattern_cycles: u32,
  pub mode_switches: u32,
  pub error_count: u32,
}

impl Default for PatternParams {
  fn default() -> Self {
    Self {
      speed: 100,
      direction: Direction::Forward,
      intensity: 128,
      repeat_count: 0,
    }
  }
}

impl Default for OutputControllerConfig {
  fn default() -> Self {
    Self {
      mode: OutputMode::Pattern,
      update_frequency: 50,
      pattern_params: PatternParams::default(),
    }
  }
}

impl Default for ModeState {
  fn default() -> Self {
    Self {
      current_pattern: 0,
      step_counter: 0,
      direction_flag: true,
      repeat_counter: 0,
      last_update_time: 0,
    }
  }
}

impl OutputController {
  /// 创建新的输出控制器
  pub fn new(config: OutputControllerConfig) -> Self {
    // 配置输出引脚
    let output_configs = [OutputConfig {
      mode: PinMode::PushPull,
      initial_state: PinState::Low,
      drive_strength: DriveStrength::Medium,
      slew_rate: SlewRate::Medium,
    }; 32];

    Self {
      digital_output: DigitalOutputController::new(output_configs),
      config,
      mode_state: ModeState::default(),
      stats: ControllerStats::default(),
    }
  }

  /// 初始化控制器
  pub fn init(&mut self) -> Result<(), ()> {
    // 设置输出模式
    self.digital_output.set_output_mode(self.config.mode);

    // 清除所有输出
    self.digital_output.set_pins(0xFFFFFFFF, 0x00000000)?;

    // 重置统计
    self.digital_output.reset_stats();

    Ok(())
  }

  /// 更新输出
  pub fn update(&mut self, timestamp: u32) -> Result<(), ()> {
    let time_since_update = timestamp.saturating_sub(self.mode_state.last_update_time);
    let update_interval = 1000 / self.config.update_frequency;

    if time_since_update >= update_interval {
      self.mode_state.last_update_time = timestamp;

      match self.config.mode {
        OutputMode::Individual => self.update_individual_mode()?,
        OutputMode::Group => self.update_group_mode()?,
        OutputMode::Pattern => self.update_pattern_mode()?,
        OutputMode::PWM => self.update_pwm_mode()?,
      }

      self.stats.total_updates += 1;
    }

    Ok(())
  }

  /// 更新单独模式
  fn update_individual_mode(&mut self) -> Result<(), ()> {
    // 单独控制每个引脚
    for i in 0..32 {
      let state = if (self.mode_state.step_counter + i) % 2 == 0 {
        PinState::High
      } else {
        PinState::Low
      };

      self.digital_output.set_pin(i, state)?;
    }

    self.mode_state.step_counter += 1;
    Ok(())
  }

  /// 更新组模式
  fn update_group_mode(&mut self) -> Result<(), ()> {
    // 按组控制引脚
    let group_size = 4;
    let num_groups = 32 / group_size;

    for group in 0..num_groups {
      let group_pattern = match (self.mode_state.step_counter + group) % 4 {
        0 => 0b0001,
        1 => 0b0011,
        2 => 0b0111,
        3 => 0b1111,
        _ => 0b0000,
      };

      let mask = 0xF << (group * group_size);
      let pattern = group_pattern << (group * group_size);

      self.digital_output.set_pins(mask, pattern)?;
    }

    self.mode_state.step_counter += 1;
    Ok(())
  }

  /// 更新模式输出
  fn update_pattern_mode(&mut self) -> Result<(), ()> {
    let pattern = self.generate_pattern();
    self.digital_output.set_pins(0xFFFFFFFF, pattern)?;

    self.advance_pattern_state();
    Ok(())
  }

  /// 生成输出模式
  fn generate_pattern(&self) -> u32 {
    let step = self.mode_state.step_counter;
    let params = &self.config.pattern_params;

    match step % 8 {
      0 => self.generate_chase_pattern(),
      1 => self.generate_wave_pattern(),
      2 => self.generate_blink_pattern(),
      3 => self.generate_fade_pattern(),
      4 => self.generate_random_pattern(),
      5 => self.generate_spiral_pattern(),
      6 => self.generate_checkerboard_pattern(),
      7 => self.generate_binary_counter_pattern(),
      _ => 0x00000000,
    }
  }

  /// 生成追逐模式
  fn generate_chase_pattern(&self) -> u32 {
    let pos = self.mode_state.step_counter % 32;
    let width = 4;

    let mut pattern = 0u32;
    for i in 0..width {
      let bit_pos = (pos + i) % 32;
      pattern |= 1 << bit_pos;
    }

    pattern
  }

  /// 生成波浪模式
  fn generate_wave_pattern(&self) -> u32 {
    let step = self.mode_state.step_counter;
    let mut pattern = 0u32;

    for i in 0..32 {
      let phase = (step + i * 2) % 64;
      if phase < 32 {
        pattern |= 1 << i;
      }
    }

    pattern
  }

  /// 生成闪烁模式
  fn generate_blink_pattern(&self) -> u32 {
    let step = self.mode_state.step_counter;

    match step % 4 {
      0 => 0xFFFFFFFF,
      1 => 0x00000000,
      2 => 0xAAAAAAAA,
      3 => 0x55555555,
      _ => 0x00000000,
    }
  }

  /// 生成渐变模式
  fn generate_fade_pattern(&self) -> u32 {
    let step = self.mode_state.step_counter % 64;
    let intensity = if step < 32 { step } else { 64 - step };

    let mut pattern = 0u32;
    for i in 0..32 {
      if i < intensity {
        pattern |= 1 << i;
      }
    }

    pattern
  }

  /// 生成随机模式
  fn generate_random_pattern(&self) -> u32 {
    let step = self.mode_state.step_counter;
    // 简单的伪随机数生成
    step.wrapping_mul(1103515245).wrapping_add(12345)
  }

  /// 生成螺旋模式
  fn generate_spiral_pattern(&self) -> u32 {
    let step = self.mode_state.step_counter;
    let mut pattern = 0u32;

    // 模拟8x4矩阵的螺旋模式
    let positions = [
      0, 1, 2, 3, 7, 11, 15, 19, 23, 27, 31, 30, 29, 28, 24, 20, 16, 12, 8, 4, 5, 6, 10, 14, 18,
      22, 26, 25, 21, 17, 13, 9,
    ];

    let pos_index = step % positions.len() as u32;
    let bit_pos = positions[pos_index as usize];

    // 创建拖尾效果
    for i in 0..8 {
      let trail_pos = (pos_index + positions.len() as u32 - i) % positions.len() as u32;
      let trail_bit = positions[trail_pos as usize];
      if i < 4 {
        pattern |= 1 << trail_bit;
      }
    }

    pattern
  }

  /// 生成棋盘模式
  fn generate_checkerboard_pattern(&self) -> u32 {
    let step = self.mode_state.step_counter;

    if step % 2 == 0 {
      0xAAAAAAAA
    } else {
      0x55555555
    }
  }

  /// 生成二进制计数器模式
  fn generate_binary_counter_pattern(&self) -> u32 {
    self.mode_state.step_counter
  }

  /// 推进模式状态
  fn advance_pattern_state(&mut self) {
    self.mode_state.step_counter += 1;

    // 检查是否完成一个周期
    if self.mode_state.step_counter % 100 == 0 {
      self.stats.pattern_cycles += 1;

      // 检查重复次数
      if self.config.pattern_params.repeat_count > 0 {
        self.mode_state.repeat_counter += 1;
        if self.mode_state.repeat_counter >= self.config.pattern_params.repeat_count {
          self.mode_state.repeat_counter = 0;
          self.mode_state.step_counter = 0;
        }
      }
    }

    // 处理方向
    match self.config.pattern_params.direction {
      Direction::Bidirectional => {
        if self.mode_state.step_counter % 50 == 0 {
          self.mode_state.direction_flag = !self.mode_state.direction_flag;
        }
      }
      _ => {}
    }
  }

  /// 更新PWM模式
  fn update_pwm_mode(&mut self) -> Result<(), ()> {
    // 软件PWM实现
    let pwm_period = 100;
    let duty_cycle = self.config.pattern_params.intensity as u32 * pwm_period / 255;
    let phase = self.mode_state.step_counter % pwm_period;

    let pattern = if phase < duty_cycle {
      0xFFFFFFFF
    } else {
      0x00000000
    };

    self.digital_output.set_pins(0xFFFFFFFF, pattern)?;
    self.mode_state.step_counter += 1;

    Ok(())
  }

  /// 设置输出模式
  pub fn set_mode(&mut self, mode: OutputMode) -> Result<(), ()> {
    if self.config.mode != mode {
      self.config.mode = mode;
      self.digital_output.set_output_mode(mode);
      self.mode_state = ModeState::default();
      self.stats.mode_switches += 1;
    }

    Ok(())
  }

  /// 设置模式参数
  pub fn set_pattern_params(&mut self, params: PatternParams) {
    self.config.pattern_params = params;
  }

  /// 手动设置输出
  pub fn set_manual_output(&mut self, mask: u32, pattern: u32) -> Result<(), ()> {
    self.digital_output.set_pins(mask, pattern)
  }

  /// 切换单个引脚
  pub fn toggle_pin(&mut self, pin: usize) -> Result<(), ()> {
    self.digital_output.toggle_pin(pin)
  }

  /// 获取统计信息
  pub fn get_stats(&self) -> &ControllerStats {
    &self.stats
  }

  /// 获取详细统计
  pub fn get_detailed_stats(&self) -> DetailedOutputStats {
    DetailedOutputStats {
      controller: self.stats,
      digital_output: *self.digital_output.get_stats(),
      mode_state: self.mode_state.current_pattern,
    }
  }
}

/// 详细输出统计
#[derive(Debug)]
pub struct DetailedOutputStats {
  pub controller: ControllerStats,
  pub digital_output: digital_io::OutputStats,
  pub mode_state: u32,
}

/// 全局输出控制器
static OUTPUT_CONTROLLER: Mutex<RefCell<Option<OutputController>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
  // 创建配置
  let config = OutputControllerConfig {
    mode: OutputMode::Pattern,
    update_frequency: 50,
    pattern_params: PatternParams {
      speed: 100,
      direction: Direction::Forward,
      intensity: 200,
      repeat_count: 0,
    },
  };

  // 初始化输出控制器
  let mut controller = OutputController::new(config);
  controller.init().unwrap();

  // 存储到全局变量
  critical_section::with(|cs| {
    OUTPUT_CONTROLLER.borrow(cs).replace(Some(controller));
  });

  let mut timestamp = 0u32;
  let mut mode_switch_counter = 0u32;

  loop {
    timestamp = timestamp.wrapping_add(1);

    // 更新输出控制器
    critical_section::with(|cs| {
      if let Some(ref mut controller) = OUTPUT_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
        let _ = controller.update(timestamp);

        // 每5000次循环切换一次模式
        if timestamp % 5000 == 0 {
          mode_switch_counter += 1;
          let new_mode = match mode_switch_counter % 4 {
            0 => OutputMode::Pattern,
            1 => OutputMode::Individual,
            2 => OutputMode::Group,
            3 => OutputMode::PWM,
            _ => OutputMode::Pattern,
          };

          let _ = controller.set_mode(new_mode);
        }

        // 每1000次循环输出统计信息
        if timestamp % 1000 == 0 {
          let stats = controller.get_detailed_stats();
          // 在实际应用中，这里可以通过串口输出统计信息
        }
      }
    });

    // 简单延时
    for _ in 0..1000 {
      cortex_m::asm::nop();
    }
  }
}
