#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{gpioa::*, Alternate, Input, PullUp, AF1, AF2},
  interrupt,
  prelude::*,
  stm32,
  timer::{Channel, Event, Timer},
};

type EncoderPinA = PA8<Alternate<AF1>>;
type EncoderPinB = PA9<Alternate<AF1>>;

// 全局变量
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM1>>>> = Mutex::new(RefCell::new(None));
static ENCODER: Mutex<RefCell<Option<QuadratureEncoder>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
  // 获取外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();

  let encoder_a = gpioa.pa8.into_alternate_af1();
  let encoder_b = gpioa.pa9.into_alternate_af1();
  let mut led = gpiob.pb0.into_push_pull_output();

  // 配置定时器1用于编码器模式
  let mut timer = Timer::tim1(dp.TIM1, &clocks);
  timer.configure_encoder_mode(encoder_a, encoder_b);
  timer.listen(Event::Update);

  // 创建正交编码器
  let encoder = QuadratureEncoder::new(1000); // 1000 PPR编码器

  // 将对象移动到全局变量
  cortex_m::interrupt::free(|cs| {
    TIMER.borrow(cs).replace(Some(timer));
    ENCODER.borrow(cs).replace(Some(encoder));
  });

  // 启用中断
  unsafe {
    NVIC::unmask(stm32::Interrupt::TIM1_UP_TIM10);
  }

  // 配置系统定时器用于延时
  let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);
  let mut last_position = 0i32;

  // 主循环
  loop {
    delay.delay_ms(100u32);

    // 读取编码器位置和速度
    cortex_m::interrupt::free(|cs| {
      if let (Some(ref timer), Some(ref mut encoder)) = (
        TIMER.borrow(cs).borrow().as_ref(),
        ENCODER.borrow(cs).borrow_mut().as_mut(),
      ) {
        let current_count = timer.get_counter() as i32;
        encoder.update_position(current_count);

        let position = encoder.get_position();
        let velocity = encoder.get_velocity_rpm();

        // 根据旋转方向控制LED
        if position != last_position {
          if position > last_position {
            led.set_high(); // 顺时针
          } else {
            led.set_low(); // 逆时针
          }
          last_position = position;
        }
      }
    });
  }
}

/// 编码器方向
#[derive(Clone, Copy, PartialEq)]
pub enum EncoderDirection {
  Clockwise,
  CounterClockwise,
  Stopped,
}

/// 正交编码器
pub struct QuadratureEncoder {
  ppr: u32,                    // 每转脉冲数
  position: i32,               // 当前位置 (脉冲数)
  last_count: i32,             // 上次计数值
  velocity_buffer: [i32; 8],   // 速度计算缓冲区
  velocity_index: usize,       // 缓冲区索引
  last_update_time: u32,       // 上次更新时间
  direction: EncoderDirection, // 旋转方向
}

impl QuadratureEncoder {
  /// 创建新的正交编码器
  pub fn new(ppr: u32) -> Self {
    Self {
      ppr,
      position: 0,
      last_count: 0,
      velocity_buffer: [0; 8],
      velocity_index: 0,
      last_update_time: 0,
      direction: EncoderDirection::Stopped,
    }
  }

  /// 更新编码器位置
  pub fn update_position(&mut self, current_count: i32) {
    let delta = current_count.wrapping_sub(self.last_count);

    if delta != 0 {
      self.position = self.position.wrapping_add(delta);

      // 更新方向
      self.direction = if delta > 0 {
        EncoderDirection::Clockwise
      } else {
        EncoderDirection::CounterClockwise
      };

      // 更新速度缓冲区
      self.velocity_buffer[self.velocity_index] = delta;
      self.velocity_index = (self.velocity_index + 1) % 8;
    } else {
      self.direction = EncoderDirection::Stopped;
    }

    self.last_count = current_count;
  }

  /// 获取当前位置 (脉冲数)
  pub fn get_position(&self) -> i32 {
    self.position
  }

  /// 获取角度 (度)
  pub fn get_angle_degrees(&self) -> f32 {
    (self.position as f32 * 360.0) / (self.ppr as f32 * 4.0) // 4倍频
  }

  /// 获取转数
  pub fn get_revolutions(&self) -> f32 {
    self.position as f32 / (self.ppr as f32 * 4.0)
  }

  /// 获取速度 (RPM)
  pub fn get_velocity_rpm(&self) -> f32 {
    let mut total_delta = 0i32;
    for &delta in &self.velocity_buffer {
      total_delta += delta;
    }

    // 假设更新频率为10Hz (100ms间隔)
    let delta_per_second = total_delta as f32 * 10.0 / 8.0;
    let revolutions_per_second = delta_per_second / (self.ppr as f32 * 4.0);
    revolutions_per_second * 60.0 // 转换为RPM
  }

  /// 获取旋转方向
  pub fn get_direction(&self) -> EncoderDirection {
    self.direction
  }

  /// 重置编码器
  pub fn reset(&mut self) {
    self.position = 0;
    self.last_count = 0;
    self.velocity_buffer = [0; 8];
    self.velocity_index = 0;
    self.direction = EncoderDirection::Stopped;
  }

  /// 设置当前位置
  pub fn set_position(&mut self, position: i32) {
    self.position = position;
  }
}

/// 编码器统计信息
pub struct EncoderStatistics {
  total_pulses: u32,
  max_velocity: f32,
  min_velocity: f32,
  direction_changes: u32,
  last_direction: EncoderDirection,
}

impl EncoderStatistics {
  /// 创建编码器统计
  pub fn new() -> Self {
    Self {
      total_pulses: 0,
      max_velocity: 0.0,
      min_velocity: f32::MAX,
      direction_changes: 0,
      last_direction: EncoderDirection::Stopped,
    }
  }

  /// 更新统计信息
  pub fn update(&mut self, encoder: &QuadratureEncoder) {
    let velocity = encoder.get_velocity_rpm().abs();
    let direction = encoder.get_direction();

    // 更新脉冲计数
    self.total_pulses = encoder.get_position().abs() as u32;

    // 更新速度统计
    if velocity > self.max_velocity {
      self.max_velocity = velocity;
    }

    if velocity < self.min_velocity && velocity > 0.0 {
      self.min_velocity = velocity;
    }

    // 更新方向变化
    if direction != self.last_direction && direction != EncoderDirection::Stopped {
      self.direction_changes += 1;
    }
    self.last_direction = direction;
  }

  /// 获取总脉冲数
  pub fn get_total_pulses(&self) -> u32 {
    self.total_pulses
  }

  /// 获取最大速度
  pub fn get_max_velocity(&self) -> f32 {
    self.max_velocity
  }

  /// 获取最小速度
  pub fn get_min_velocity(&self) -> f32 {
    if self.min_velocity == f32::MAX {
      0.0
    } else {
      self.min_velocity
    }
  }

  /// 获取方向变化次数
  pub fn get_direction_changes(&self) -> u32 {
    self.direction_changes
  }
}

/// 双编码器管理器 (用于差速驱动)
pub struct DualEncoderManager {
  left_encoder: QuadratureEncoder,
  right_encoder: QuadratureEncoder,
  wheel_diameter: f32, // 轮子直径 (mm)
  wheel_base: f32,     // 轮距 (mm)
}

impl DualEncoderManager {
  /// 创建双编码器管理器
  pub fn new(ppr: u32, wheel_diameter: f32, wheel_base: f32) -> Self {
    Self {
      left_encoder: QuadratureEncoder::new(ppr),
      right_encoder: QuadratureEncoder::new(ppr),
      wheel_diameter,
      wheel_base,
    }
  }

  /// 更新左编码器
  pub fn update_left_encoder(&mut self, count: i32) {
    self.left_encoder.update_position(count);
  }

  /// 更新右编码器
  pub fn update_right_encoder(&mut self, count: i32) {
    self.right_encoder.update_position(count);
  }

  /// 获取左轮距离 (mm)
  pub fn get_left_distance_mm(&self) -> f32 {
    let revolutions = self.left_encoder.get_revolutions();
    revolutions * core::f32::consts::PI * self.wheel_diameter
  }

  /// 获取右轮距离 (mm)
  pub fn get_right_distance_mm(&self) -> f32 {
    let revolutions = self.right_encoder.get_revolutions();
    revolutions * core::f32::consts::PI * self.wheel_diameter
  }

  /// 获取机器人位移 (mm)
  pub fn get_robot_distance_mm(&self) -> f32 {
    (self.get_left_distance_mm() + self.get_right_distance_mm()) / 2.0
  }

  /// 获取机器人角度变化 (弧度)
  pub fn get_robot_angle_rad(&self) -> f32 {
    let left_distance = self.get_left_distance_mm();
    let right_distance = self.get_right_distance_mm();
    (right_distance - left_distance) / self.wheel_base
  }

  /// 获取机器人线速度 (mm/s)
  pub fn get_robot_linear_velocity(&self) -> f32 {
    let left_velocity =
      self.left_encoder.get_velocity_rpm() * core::f32::consts::PI * self.wheel_diameter / 60.0;
    let right_velocity =
      self.right_encoder.get_velocity_rpm() * core::f32::consts::PI * self.wheel_diameter / 60.0;
    (left_velocity + right_velocity) / 2.0
  }

  /// 获取机器人角速度 (rad/s)
  pub fn get_robot_angular_velocity(&self) -> f32 {
    let left_velocity =
      self.left_encoder.get_velocity_rpm() * core::f32::consts::PI * self.wheel_diameter / 60.0;
    let right_velocity =
      self.right_encoder.get_velocity_rpm() * core::f32::consts::PI * self.wheel_diameter / 60.0;
    (right_velocity - left_velocity) / self.wheel_base
  }

  /// 重置所有编码器
  pub fn reset(&mut self) {
    self.left_encoder.reset();
    self.right_encoder.reset();
  }
}

/// 编码器校准器
pub struct EncoderCalibrator {
  reference_angle: f32,    // 参考角度
  measured_pulses: i32,    // 测量脉冲数
  calibration_factor: f32, // 校准因子
  is_calibrated: bool,     // 是否已校准
}

impl EncoderCalibrator {
  /// 创建编码器校准器
  pub fn new() -> Self {
    Self {
      reference_angle: 0.0,
      measured_pulses: 0,
      calibration_factor: 1.0,
      is_calibrated: false,
    }
  }

  /// 开始校准
  pub fn start_calibration(&mut self, reference_angle: f32) {
    self.reference_angle = reference_angle;
    self.measured_pulses = 0;
    self.is_calibrated = false;
  }

  /// 完成校准
  pub fn finish_calibration(&mut self, encoder: &QuadratureEncoder) {
    self.measured_pulses = encoder.get_position();

    if self.measured_pulses != 0 {
      let measured_angle = encoder.get_angle_degrees();
      self.calibration_factor = self.reference_angle / measured_angle;
      self.is_calibrated = true;
    }
  }

  /// 获取校准后的角度
  pub fn get_calibrated_angle(&self, encoder: &QuadratureEncoder) -> f32 {
    if self.is_calibrated {
      encoder.get_angle_degrees() * self.calibration_factor
    } else {
      encoder.get_angle_degrees()
    }
  }

  /// 检查是否已校准
  pub fn is_calibrated(&self) -> bool {
    self.is_calibrated
  }

  /// 获取校准因子
  pub fn get_calibration_factor(&self) -> f32 {
    self.calibration_factor
  }
}

/// 定时器1更新中断处理程序
#[interrupt]
fn TIM1_UP_TIM10() {
  cortex_m::interrupt::free(|cs| {
    if let (Some(ref timer), Some(ref mut encoder)) = (
      TIMER.borrow(cs).borrow().as_ref(),
      ENCODER.borrow(cs).borrow_mut().as_mut(),
    ) {
      if timer.is_update_interrupt_pending() {
        // 处理定时器溢出
        let current_count = timer.get_counter() as i32;
        encoder.update_position(current_count);

        timer.clear_update_interrupt();
      }
    }
  });
}
