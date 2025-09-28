//! 时间模块

use crate::*;

/// 时间戳
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Timestamp(pub u64);

impl Timestamp {
  pub fn now() -> Self {
    // 这里需要平台特定的实现
    Self(0)
  }

  pub fn elapsed_since(&self, other: Timestamp) -> Duration {
    Duration(self.0.saturating_sub(other.0))
  }
}

/// 持续时间
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Duration(pub u64);

impl Duration {
  pub fn from_millis(ms: u64) -> Self {
    Self(ms * 1000) // 假设内部使用微秒
  }

  pub fn from_secs(secs: u64) -> Self {
    Self(secs * 1_000_000)
  }

  pub fn as_millis(&self) -> u64 {
    self.0 / 1000
  }

  pub fn as_secs(&self) -> u64 {
    self.0 / 1_000_000
  }
}

/// 定时器
pub struct Timer {
  start_time: Timestamp,
  duration: Duration,
  is_running: bool,
}

impl Timer {
  pub fn new(duration: Duration) -> Self {
    Self {
      start_time: Timestamp::now(),
      duration,
      is_running: false,
    }
  }

  pub fn start(&mut self) {
    self.start_time = Timestamp::now();
    self.is_running = true;
  }

  pub fn stop(&mut self) {
    self.is_running = false;
  }

  pub fn is_expired(&self) -> bool {
    if !self.is_running {
      return false;
    }

    let elapsed = Timestamp::now().elapsed_since(self.start_time);
    elapsed >= self.duration
  }

  pub fn reset(&mut self) {
    self.start_time = Timestamp::now();
  }

  pub fn remaining(&self) -> Duration {
    if !self.is_running {
      return self.duration;
    }

    let elapsed = Timestamp::now().elapsed_since(self.start_time);
    if elapsed >= self.duration {
      Duration(0)
    } else {
      Duration(self.duration.0 - elapsed.0)
    }
  }
}
