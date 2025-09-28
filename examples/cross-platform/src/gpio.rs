//! GPIO控制模块

use crate::traits::GpioAbstraction;
use crate::*;
use heapless::{FnvIndexMap, Vec};

/// 通用GPIO控制器
pub struct UniversalGpio<T: GpioAbstraction> {
  inner: T,
  pin_states: FnvIndexMap<u8, bool, 32>,
}

impl<T: GpioAbstraction> UniversalGpio<T> {
  pub fn new(gpio: T) -> Self {
    Self {
      inner: gpio,
      pin_states: FnvIndexMap::new(),
    }
  }

  pub fn configure_pin(&mut self, pin_id: u8, initial_state: bool) -> Result<()> {
    if initial_state {
      self.inner.set_pin_high(pin_id)?;
    } else {
      self.inner.set_pin_low(pin_id)?;
    }
    self
      .pin_states
      .insert(pin_id, initial_state)
      .map_err(|_| Error::ResourceExhausted)?;
    Ok(())
  }

  pub fn set_pin(&mut self, pin_id: u8, state: bool) -> Result<()> {
    if state {
      self.inner.set_pin_high(pin_id)?;
    } else {
      self.inner.set_pin_low(pin_id)?;
    }
    self
      .pin_states
      .insert(pin_id, state)
      .map_err(|_| Error::ResourceExhausted)?;
    Ok(())
  }

  pub fn get_pin(&self, pin_id: u8) -> Result<bool> {
    self.inner.read_pin(pin_id)
  }

  pub fn toggle_pin(&mut self, pin_id: u8) -> Result<()> {
    self.inner.toggle_pin(pin_id)?;
    if let Some(state) = self.pin_states.get_mut(&pin_id) {
      *state = !*state;
    }
    Ok(())
  }

  pub fn get_pin_count(&self) -> usize {
    self.pin_states.len()
  }
}

/// LED控制器
pub struct LedController<T: GpioAbstraction> {
  gpio: UniversalGpio<T>,
  led_pins: Vec<u8, 8>,
}

impl<T: GpioAbstraction> LedController<T> {
  pub fn new(gpio: T, led_pins: &[u8]) -> Result<Self> {
    let mut controller = Self {
      gpio: UniversalGpio::new(gpio),
      led_pins: Vec::new(),
    };

    for &pin in led_pins {
      controller
        .led_pins
        .push(pin)
        .map_err(|_| Error::ResourceExhausted)?;
      controller.gpio.configure_pin(pin, false)?;
    }

    Ok(controller)
  }

  pub fn set_led(&mut self, index: usize, state: bool) -> Result<()> {
    if let Some(&pin) = self.led_pins.get(index) {
      self.gpio.set_pin(pin, state)
    } else {
      Err(Error::InvalidParameter)
    }
  }

  pub fn toggle_led(&mut self, index: usize) -> Result<()> {
    if let Some(&pin) = self.led_pins.get(index) {
      self.gpio.toggle_pin(pin)
    } else {
      Err(Error::InvalidParameter)
    }
  }

  pub fn set_pattern(&mut self, pattern: u8) -> Result<()> {
    for (i, &pin) in self.led_pins.iter().enumerate() {
      let state = (pattern & (1 << i)) != 0;
      self.gpio.set_pin(pin, state)?;
    }
    Ok(())
  }

  pub fn clear_all(&mut self) -> Result<()> {
    for &pin in &self.led_pins {
      self.gpio.set_pin(pin, false)?;
    }
    Ok(())
  }
}
