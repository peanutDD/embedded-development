//! 数据结构模块

use crate::*;
use heapless::{String, Vec};

/// 数据点
#[derive(Debug, Clone)]
pub struct DataPoint {
  pub timestamp: u32,
  pub value: f32,
  pub quality: DataQuality,
}

/// 数据质量
#[derive(Debug, Clone, Copy)]
pub enum DataQuality {
  Good,
  Uncertain,
  Bad,
}

/// 时间序列数据
pub struct TimeSeries<const N: usize> {
  data: Vec<DataPoint, N>,
  name: String<32>,
}

impl<const N: usize> TimeSeries<N> {
  pub fn new(name: &str) -> Result<Self> {
    let mut series_name = String::new();
    series_name
      .push_str(name)
      .map_err(|_| Error::ResourceExhausted)?;

    Ok(Self {
      data: Vec::new(),
      name: series_name,
    })
  }

  pub fn add_point(&mut self, point: DataPoint) -> Result<()> {
    self.data.push(point).map_err(|_| Error::ResourceExhausted)
  }

  pub fn get_latest(&self) -> Option<&DataPoint> {
    self.data.last()
  }

  pub fn get_range(&self, start_time: u32, end_time: u32) -> Vec<&DataPoint, N> {
    let mut result = Vec::new();
    for point in &self.data {
      if point.timestamp >= start_time && point.timestamp <= end_time {
        let _ = result.push(point);
      }
    }
    result
  }

  pub fn len(&self) -> usize {
    self.data.len()
  }

  pub fn is_empty(&self) -> bool {
    self.data.is_empty()
  }

  pub fn clear(&mut self) {
    self.data.clear();
  }
}

/// 数据缓存
pub struct DataCache<const N: usize> {
  series: Vec<TimeSeries<N>, 8>,
}

impl<const N: usize> DataCache<N> {
  pub fn new() -> Self {
    Self { series: Vec::new() }
  }

  pub fn add_series(&mut self, series: TimeSeries<N>) -> Result<usize> {
    let index = self.series.len();
    self
      .series
      .push(series)
      .map_err(|_| Error::ResourceExhausted)?;
    Ok(index)
  }

  pub fn get_series(&self, index: usize) -> Option<&TimeSeries<N>> {
    self.series.get(index)
  }

  pub fn get_series_mut(&mut self, index: usize) -> Option<&mut TimeSeries<N>> {
    self.series.get_mut(index)
  }

  pub fn series_count(&self) -> usize {
    self.series.len()
  }
}
