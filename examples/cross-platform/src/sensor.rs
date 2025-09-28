//! 传感器模块

use crate::traits::SensorAbstraction;
use crate::*;
use heapless::{String, Vec};

/// 传感器信息
#[derive(Debug, Clone)]
pub struct SensorInfo {
  pub name: String<32>,
  pub sensor_type: SensorType,
  pub range: (f32, f32),
  pub resolution: f32,
  pub accuracy: f32,
}

/// 传感器类型
#[derive(Debug, Clone, Copy)]
pub enum SensorType {
  Temperature,
  Humidity,
  Pressure,
  Light,
  Accelerometer,
  Gyroscope,
  Magnetometer,
  Custom(u8),
}

/// 传感器读数
#[derive(Debug, Clone)]
pub struct SensorReading {
  pub value: f32,
  pub timestamp: u32,
  pub unit: String<16>,
  pub quality: ReadingQuality,
}

/// 读数质量
#[derive(Debug, Clone, Copy)]
pub enum ReadingQuality {
  Excellent,
  Good,
  Fair,
  Poor,
  Invalid,
}

/// 传感器管理器
pub struct SensorManager<T: SensorAbstraction> {
  sensors: Vec<T, 8>,
  readings: Vec<SensorReading, 64>,
}

impl<T: SensorAbstraction> SensorManager<T> {
  pub fn new() -> Self {
    Self {
      sensors: Vec::new(),
      readings: Vec::new(),
    }
  }

  pub fn add_sensor(&mut self, sensor: T) -> Result<usize> {
    let index = self.sensors.len();
    self
      .sensors
      .push(sensor)
      .map_err(|_| Error::ResourceExhausted)?;
    Ok(index)
  }

  pub fn read_sensor(&mut self, index: usize) -> Result<SensorReading> {
    if index < self.sensors.len() {
      // 简化实现，返回默认读数
      Ok(SensorReading {
        value: 0.0,
        timestamp: 0,
        unit: String::from("unit"),
        quality: ReadingQuality::Good,
      })
    } else {
      Err(Error::InvalidParameter)
    }
  }

  pub fn calibrate_sensor(&mut self, index: usize) -> Result<()> {
    if let Some(sensor) = self.sensors.get_mut(index) {
      sensor.calibrate()
    } else {
      Err(Error::InvalidParameter)
    }
  }

  pub fn get_sensor_info(&self, index: usize) -> Result<SensorInfo> {
    if let Some(sensor) = self.sensors.get(index) {
      Ok(sensor.get_info())
    } else {
      Err(Error::InvalidParameter)
    }
  }

  pub fn get_sensor_count(&self) -> usize {
    self.sensors.len()
  }
}
