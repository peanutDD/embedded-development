//! # 基础LED控制库
//!
//! 提供LED控制的基础功能和抽象，支持多种LED控制模式和效果

#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/// LED控制器特征
pub trait LedController {
  /// 错误类型
  type Error;

  /// 开启LED
  fn turn_on(&mut self) -> Result<(), Self::Error>;

  /// 关闭LED
  fn turn_off(&mut self) -> Result<(), Self::Error>;

  /// 切换LED状态
  fn toggle(&mut self) -> Result<(), Self::Error>;

  /// 检查LED是否开启
  fn is_on(&self) -> bool;

  /// 设置LED状态
  fn set_state(&mut self, on: bool) -> Result<(), Self::Error> {
    if on {
      self.turn_on()
    } else {
      self.turn_off()
    }
  }
}

/// 基础LED实现
pub struct BasicLed<P> {
  pin: P,
  active_low: bool,
  state: bool,
}

impl<P> BasicLed<P>
where
  P: OutputPin,
{
  /// 创建新的LED控制器
  ///
  /// # 参数
  /// - `pin`: GPIO输出引脚
  /// - `active_low`: 是否为低电平有效 (true表示低电平点亮LED)
  pub fn new(pin: P, active_low: bool) -> Self {
    Self {
      pin,
      active_low,
      state: false,
    }
  }

  /// 创建高电平有效的LED
  pub fn active_high(pin: P) -> Self {
    Self::new(pin, false)
  }

  /// 创建低电平有效的LED
  pub fn active_low(pin: P) -> Self {
    Self::new(pin, true)
  }

  /// 获取引脚的可变引用
  pub fn pin_mut(&mut self) -> &mut P {
    &mut self.pin
  }

  /// 获取引脚的不可变引用
  pub fn pin(&self) -> &P {
    &self.pin
  }

  /// 释放引脚资源
  pub fn release(self) -> P {
    self.pin
  }
}

impl<P> LedController for BasicLed<P>
where
  P: OutputPin,
{
  type Error = P::Error;

  fn turn_on(&mut self) -> Result<(), Self::Error> {
    self.state = true;
    if self.active_low {
      self.pin.set_low()
    } else {
      self.pin.set_high()
    }
  }

  fn turn_off(&mut self) -> Result<(), Self::Error> {
    self.state = false;
    if self.active_low {
      self.pin.set_high()
    } else {
      self.pin.set_low()
    }
  }

  fn toggle(&mut self) -> Result<(), Self::Error> {
    if self.state {
      self.turn_off()
    } else {
      self.turn_on()
    }
  }

  fn is_on(&self) -> bool {
    self.state
  }
}

/// LED闪烁模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlinkPattern {
  /// 均匀闪烁
  Uniform,
  /// 快速闪烁
  Fast,
  /// 慢速闪烁
  Slow,
  /// 心跳模式 (两次快闪后暂停)
  Heartbeat,
  /// SOS模式 (摩尔斯电码)
  SOS,
  /// 自定义模式
  Custom(&'static [u32]),
}

impl BlinkPattern {
  /// 获取闪烁时序 (毫秒)
  pub fn get_timing(&self) -> &[u32] {
    match self {
      BlinkPattern::Uniform => &[500, 500],
      BlinkPattern::Fast => &[100, 100],
      BlinkPattern::Slow => &[1000, 1000],
      BlinkPattern::Heartbeat => &[100, 100, 100, 700],
      BlinkPattern::SOS => &[
        100, 100, 100, 100, 100, 300, // S
        300, 100, 300, 100, 300, 300, // O
        100, 100, 100, 100, 100, 1000, // S
      ],
      BlinkPattern::Custom(timing) => timing,
    }
  }
}

/// LED闪烁器
pub struct LedBlinker<L, D> {
  led: L,
  delay: D,
  pattern: BlinkPattern,
  pattern_index: usize,
  led_on: bool,
}

impl<L, D> LedBlinker<L, D>
where
  L: LedController,
  D: DelayNs,
{
  /// 创建新的闪烁器
  pub fn new(led: L, delay: D, pattern: BlinkPattern) -> Self {
    Self {
      led,
      delay,
      pattern,
      pattern_index: 0,
      led_on: false,
    }
  }

  /// 设置闪烁模式
  pub fn set_pattern(&mut self, pattern: BlinkPattern) {
    self.pattern = pattern;
    self.pattern_index = 0;
  }

  /// 执行一次闪烁步骤
  pub fn step(&mut self) -> Result<(), L::Error> {
    let timing = self.pattern.get_timing();

    if timing.is_empty() {
      return Ok(());
    }

    // 获取当前延时时间
    let delay_ms = timing[self.pattern_index];

    // 切换LED状态
    if self.led_on {
      self.led.turn_off()?;
      self.led_on = false;
    } else {
      self.led.turn_on()?;
      self.led_on = true;
    }

    // 延时
    self.delay.delay_ms(delay_ms);

    // 更新模式索引
    self.pattern_index = (self.pattern_index + 1) % timing.len();

    Ok(())
  }

  /// 连续闪烁指定次数
  pub fn blink_times(&mut self, times: u32) -> Result<(), L::Error> {
    for _ in 0..times {
      let timing = self.pattern.get_timing();
      for _ in 0..timing.len() {
        self.step()?;
      }
    }

    // 确保LED最终关闭
    self.led.turn_off()?;
    self.led_on = false;

    Ok(())
  }

  /// 无限闪烁 (需要外部中断)
  pub fn blink_forever(&mut self) -> Result<(), L::Error> {
    loop {
      self.step()?;
    }
  }

  /// 获取LED控制器的引用
  pub fn led(&self) -> &L {
    &self.led
  }

  /// 获取LED控制器的可变引用
  pub fn led_mut(&mut self) -> &mut L {
    &mut self.led
  }

  /// 释放资源
  pub fn release(self) -> (L, D) {
    (self.led, self.delay)
  }
}

/// LED序列控制器 - 控制多个LED
pub struct LedSequence<L, D, const N: usize> {
  leds: [L; N],
  delay: D,
  current_index: usize,
}

impl<L, D, const N: usize> LedSequence<L, D, N>
where
  L: LedController,
  D: DelayNs,
{
  /// 创建新的LED序列控制器
  pub fn new(leds: [L; N], delay: D) -> Self {
    Self {
      leds,
      delay,
      current_index: 0,
    }
  }

  /// 关闭所有LED
  pub fn turn_off_all(&mut self) -> Result<(), L::Error> {
    for led in &mut self.leds {
      led.turn_off()?;
    }
    Ok(())
  }

  /// 开启所有LED
  pub fn turn_on_all(&mut self) -> Result<(), L::Error> {
    for led in &mut self.leds {
      led.turn_on()?;
    }
    Ok(())
  }

  /// 流水灯效果
  pub fn running_light(&mut self, delay_ms: u32, cycles: u32) -> Result<(), L::Error> {
    for _ in 0..cycles {
      for i in 0..N {
        // 关闭所有LED
        self.turn_off_all()?;

        // 开启当前LED
        self.leds[i].turn_on()?;

        // 延时
        self.delay.delay_ms(delay_ms);
      }
    }

    // 最后关闭所有LED
    self.turn_off_all()?;
    Ok(())
  }

  /// 呼吸灯效果 (简化版本，需要PWM支持完整效果)
  pub fn breathing_effect(&mut self, delay_ms: u32, cycles: u32) -> Result<(), L::Error> {
    for _ in 0..cycles {
      // 渐亮效果 (简化为快速闪烁)
      for _ in 0..10 {
        self.turn_on_all()?;
        self.delay.delay_ms(delay_ms / 20);
        self.turn_off_all()?;
        self.delay.delay_ms(delay_ms / 10);
      }

      // 渐暗效果
      for _ in 0..10 {
        self.turn_on_all()?;
        self.delay.delay_ms(delay_ms / 10);
        self.turn_off_all()?;
        self.delay.delay_ms(delay_ms / 20);
      }
    }

    Ok(())
  }

  /// 随机闪烁效果
  pub fn random_blink(&mut self, delay_ms: u32, duration_ms: u32) -> Result<(), L::Error> {
    let steps = duration_ms / delay_ms;
    let mut lfsr = 0xACE1u16; // 线性反馈移位寄存器作为伪随机数生成器

    for _ in 0..steps {
      // 生成伪随机数
      let bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
      lfsr = (lfsr >> 1) | (bit << 15);

      // 根据随机数控制LED
      for (i, led) in self.leds.iter_mut().enumerate() {
        if (lfsr >> i) & 1 == 1 {
          led.turn_on()?;
        } else {
          led.turn_off()?;
        }
      }

      self.delay.delay_ms(delay_ms);
    }

    self.turn_off_all()?;
    Ok(())
  }

  /// 获取LED数量
  pub fn led_count(&self) -> usize {
    N
  }

  /// 获取指定LED的引用
  pub fn get_led(&self, index: usize) -> Option<&L> {
    self.leds.get(index)
  }

  /// 获取指定LED的可变引用
  pub fn get_led_mut(&mut self, index: usize) -> Option<&mut L> {
    self.leds.get_mut(index)
  }
}

/// LED状态指示器
pub struct LedIndicator<L> {
  led: L,
  state: IndicatorState,
}

/// 指示器状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IndicatorState {
  /// 关闭
  Off,
  /// 开启
  On,
  /// 错误状态 (快速闪烁)
  Error,
  /// 警告状态 (慢速闪烁)
  Warning,
  /// 就绪状态 (常亮)
  Ready,
  /// 忙碌状态 (中速闪烁)
  Busy,
}

impl<L> LedIndicator<L>
where
  L: LedController,
{
  /// 创建新的状态指示器
  pub fn new(led: L) -> Self {
    Self {
      led,
      state: IndicatorState::Off,
    }
  }

  /// 设置指示器状态
  pub fn set_state(&mut self, state: IndicatorState) -> Result<(), L::Error> {
    self.state = state;
    match state {
      IndicatorState::Off => self.led.turn_off(),
      IndicatorState::On | IndicatorState::Ready => self.led.turn_on(),
      _ => {
        // 其他状态需要在主循环中处理闪烁
        Ok(())
      }
    }
  }

  /// 获取当前状态
  pub fn get_state(&self) -> IndicatorState {
    self.state
  }

  /// 获取LED控制器的引用
  pub fn led(&self) -> &L {
    &self.led
  }

  /// 获取LED控制器的可变引用
  pub fn led_mut(&mut self) -> &mut L {
    &mut self.led
  }

  /// 释放LED控制器
  pub fn release(self) -> L {
    self.led
  }

  /// 更新指示器 (需要在主循环中定期调用)
  pub fn update(&mut self, delay_provider: &mut dyn DelayNs) -> Result<(), L::Error> {
    match self.state {
      IndicatorState::Error => {
        // 快速闪烁 (100ms)
        self.led.toggle()?;
        delay_provider.delay_ms(100);
      }
      IndicatorState::Warning => {
        // 慢速闪烁 (1000ms)
        self.led.toggle()?;
        delay_provider.delay_ms(1000);
      }
      IndicatorState::Busy => {
        // 中速闪烁 (300ms)
        self.led.toggle()?;
        delay_provider.delay_ms(300);
      }
      _ => {
        // 其他状态不需要更新
      }
    }
    Ok(())
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use embedded_hal::digital::ErrorType;

  // 模拟GPIO引脚用于测试
  struct MockPin {
    state: bool,
  }

  impl MockPin {
    fn new() -> Self {
      Self { state: false }
    }
  }

  impl ErrorType for MockPin {
    type Error = core::convert::Infallible;
  }

  impl OutputPin for MockPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
      self.state = false;
      Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
      self.state = true;
      Ok(())
    }
  }

  #[test]
  fn test_basic_led() {
    let pin = MockPin::new();
    let mut led = BasicLed::active_high(pin);

    assert!(!led.is_on());

    led.turn_on().unwrap();
    assert!(led.is_on());

    led.turn_off().unwrap();
    assert!(!led.is_on());

    led.toggle().unwrap();
    assert!(led.is_on());
  }

  #[test]
  fn test_blink_pattern() {
    let uniform = BlinkPattern::Uniform;
    assert_eq!(uniform.get_timing(), &[500, 500]);

    let fast = BlinkPattern::Fast;
    assert_eq!(fast.get_timing(), &[100, 100]);

    let custom = BlinkPattern::Custom(&[200, 300]);
    assert_eq!(custom.get_timing(), &[200, 300]);
  }

  #[test]
  fn test_indicator_state() {
    let pin = MockPin::new();
    let mut indicator = LedIndicator::new(BasicLed::active_high(pin));

    assert_eq!(indicator.get_state(), IndicatorState::Off);

    indicator.set_state(IndicatorState::Ready).unwrap();
    assert_eq!(indicator.get_state(), IndicatorState::Ready);
  }
}
