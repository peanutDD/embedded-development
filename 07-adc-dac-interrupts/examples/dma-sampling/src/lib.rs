#![no_std]

use heapless::Vec;
use micromath::F32Ext;
use stm32f4xx_hal::{
  adc::{Adc, AdcConfig, SampleTime},
  dma::{DmaFlag, MemoryToPeripheral, PeripheralToMemory, Stream, StreamsTuple, Transfer},
  gpio::{Analog, Pin},
  pac::{ADC1, DMA2},
  prelude::*,
};

pub const MAX_BUFFER_SIZE: usize = 4096;
pub const MAX_CHANNELS: usize = 16;

/// DMA采样配置
#[derive(Debug, Clone)]
pub struct DmaSamplingConfig {
  pub buffer_size: usize,
  pub sample_rate: u32,
  pub channels: Vec<u8, MAX_CHANNELS>,
  pub circular_mode: bool,
  pub priority: DmaPriority,
  pub sample_time: SampleTime,
}

#[derive(Debug, Clone, Copy)]
pub enum DmaPriority {
  Low,
  Medium,
  High,
  VeryHigh,
}

impl Default for DmaSamplingConfig {
  fn default() -> Self {
    let mut channels = Vec::new();
    channels.push(0).ok(); // 默认通道0

    Self {
      buffer_size: 1024,
      sample_rate: 1000,
      channels,
      circular_mode: true,
      priority: DmaPriority::High,
      sample_time: SampleTime::Cycles_480,
    }
  }
}

/// DMA采样管理器
pub struct DmaSamplingManager {
  config: DmaSamplingConfig,
  primary_buffer: Vec<u16, MAX_BUFFER_SIZE>,
  secondary_buffer: Option<Vec<u16, MAX_BUFFER_SIZE>>,
  current_buffer: BufferIndex,
  samples_collected: usize,
  buffer_overruns: u32,
  transfer_complete_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BufferIndex {
  Primary,
  Secondary,
}

impl DmaSamplingManager {
  pub fn new(config: DmaSamplingConfig) -> Self {
    Self {
      config,
      primary_buffer: Vec::new(),
      secondary_buffer: None,
      current_buffer: BufferIndex::Primary,
      samples_collected: 0,
      buffer_overruns: 0,
      transfer_complete_count: 0,
    }
  }

  /// 启用双缓冲模式
  pub fn enable_dual_buffer(&mut self) {
    self.secondary_buffer = Some(Vec::new());
  }

  /// 配置DMA传输
  pub fn configure_dma_transfer(&mut self) -> Result<(), &'static str> {
    // 配置DMA流
    // 设置源地址为ADC数据寄存器
    // 设置目标地址为缓冲区
    // 配置传输大小和模式
    Ok(())
  }

  /// 启动DMA采样
  pub fn start_sampling(&mut self) -> Result<(), &'static str> {
    // 启动DMA传输
    // 启动ADC转换
    self.samples_collected = 0;
    self.buffer_overruns = 0;
    Ok(())
  }

  /// 停止DMA采样
  pub fn stop_sampling(&mut self) -> Result<(), &'static str> {
    // 停止DMA传输
    // 停止ADC转换
    Ok(())
  }

  /// 处理DMA传输完成中断
  pub fn handle_transfer_complete(&mut self) -> Result<(), &'static str> {
    self.transfer_complete_count += 1;

    if self.config.circular_mode {
      // 循环模式：切换缓冲区
      self.switch_buffer();
    } else {
      // 单次模式：停止采样
      self.stop_sampling()?;
    }

    Ok(())
  }

  /// 处理DMA半传输完成中断
  pub fn handle_half_transfer(&mut self) -> Result<(), &'static str> {
    // 在循环模式下，半传输完成时可以处理前半部分数据
    if self.config.circular_mode {
      self.process_half_buffer();
    }
    Ok(())
  }

  /// 切换缓冲区
  fn switch_buffer(&mut self) {
    if self.secondary_buffer.is_some() {
      self.current_buffer = match self.current_buffer {
        BufferIndex::Primary => BufferIndex::Secondary,
        BufferIndex::Secondary => BufferIndex::Primary,
      };
    }
  }

  /// 处理半缓冲区数据
  fn process_half_buffer(&mut self) {
    let buffer = self.get_current_buffer();
    let half_size = buffer.len() / 2;

    // 处理前半部分数据
    for i in 0..half_size {
      if let Some(&sample) = buffer.get(i) {
        self.process_sample(sample, i);
      }
    }
  }

  /// 获取当前缓冲区
  pub fn get_current_buffer(&self) -> &Vec<u16, MAX_BUFFER_SIZE> {
    match self.current_buffer {
      BufferIndex::Primary => &self.primary_buffer,
      BufferIndex::Secondary => self
        .secondary_buffer
        .as_ref()
        .unwrap_or(&self.primary_buffer),
    }
  }

  /// 获取可变当前缓冲区
  pub fn get_current_buffer_mut(&mut self) -> &mut Vec<u16, MAX_BUFFER_SIZE> {
    match self.current_buffer {
      BufferIndex::Primary => &mut self.primary_buffer,
      BufferIndex::Secondary => self
        .secondary_buffer
        .as_mut()
        .unwrap_or(&mut self.primary_buffer),
    }
  }

  /// 处理单个样本
  fn process_sample(&mut self, sample: u16, index: usize) {
    self.samples_collected += 1;

    // 这里可以添加实时处理逻辑
    // 例如：滤波、阈值检测、统计计算等
  }

  /// 检查缓冲区溢出
  pub fn check_buffer_overrun(&mut self) -> bool {
    // 检查DMA溢出标志
    // 如果发生溢出，增加计数器
    false // 模拟返回
  }

  /// 获取采样统计信息
  pub fn get_sampling_stats(&self) -> SamplingStats {
    SamplingStats {
      samples_collected: self.samples_collected,
      buffer_overruns: self.buffer_overruns,
      transfer_complete_count: self.transfer_complete_count,
      current_buffer: self.current_buffer,
      buffer_utilization: self.calculate_buffer_utilization(),
    }
  }

  /// 计算缓冲区利用率
  fn calculate_buffer_utilization(&self) -> f32 {
    let buffer = self.get_current_buffer();
    if self.config.buffer_size == 0 {
      return 0.0;
    }
    (buffer.len() as f32 / self.config.buffer_size as f32) * 100.0
  }

  /// 重置统计信息
  pub fn reset_stats(&mut self) {
    self.samples_collected = 0;
    self.buffer_overruns = 0;
    self.transfer_complete_count = 0;
  }
}

/// 采样统计信息
#[derive(Debug, Clone)]
pub struct SamplingStats {
  pub samples_collected: usize,
  pub buffer_overruns: u32,
  pub transfer_complete_count: u32,
  pub current_buffer: BufferIndex,
  pub buffer_utilization: f32,
}

/// 循环缓冲区管理器
pub struct CircularBuffer<T, const N: usize> {
  buffer: [T; N],
  head: usize,
  tail: usize,
  full: bool,
}

impl<T: Copy + Default, const N: usize> CircularBuffer<T, N> {
  pub fn new() -> Self {
    Self {
      buffer: [T::default(); N],
      head: 0,
      tail: 0,
      full: false,
    }
  }

  /// 写入数据
  pub fn write(&mut self, item: T) -> Result<(), &'static str> {
    if self.full && self.head == self.tail {
      return Err("Buffer overflow");
    }

    self.buffer[self.head] = item;
    self.head = (self.head + 1) % N;

    if self.head == self.tail {
      self.full = true;
    }

    Ok(())
  }

  /// 读取数据
  pub fn read(&mut self) -> Option<T> {
    if self.is_empty() {
      return None;
    }

    let item = self.buffer[self.tail];
    self.tail = (self.tail + 1) % N;
    self.full = false;

    Some(item)
  }

  /// 批量写入
  pub fn write_slice(&mut self, data: &[T]) -> Result<usize, &'static str> {
    let mut written = 0;

    for &item in data {
      match self.write(item) {
        Ok(()) => written += 1,
        Err(_) => break,
      }
    }

    Ok(written)
  }

  /// 批量读取
  pub fn read_slice(&mut self, buffer: &mut [T]) -> usize {
    let mut read_count = 0;

    for slot in buffer.iter_mut() {
      if let Some(item) = self.read() {
        *slot = item;
        read_count += 1;
      } else {
        break;
      }
    }

    read_count
  }

  /// 检查是否为空
  pub fn is_empty(&self) -> bool {
    !self.full && self.head == self.tail
  }

  /// 检查是否已满
  pub fn is_full(&self) -> bool {
    self.full
  }

  /// 获取可用空间
  pub fn available_space(&self) -> usize {
    if self.full {
      0
    } else if self.head >= self.tail {
      N - (self.head - self.tail)
    } else {
      self.tail - self.head
    }
  }

  /// 获取已用空间
  pub fn used_space(&self) -> usize {
    N - self.available_space()
  }

  /// 清空缓冲区
  pub fn clear(&mut self) {
    self.head = 0;
    self.tail = 0;
    self.full = false;
  }

  /// 获取容量
  pub fn capacity(&self) -> usize {
    N
  }
}

/// 高速采样控制器
pub struct HighSpeedSampler {
  sample_rate: u32,
  max_sample_rate: u32,
  buffer_manager: DmaSamplingManager,
  oversampling_ratio: u8,
  decimation_filter: DecimationFilter,
}

impl HighSpeedSampler {
  pub fn new(config: DmaSamplingConfig, max_rate: u32) -> Self {
    Self {
      sample_rate: config.sample_rate,
      max_sample_rate: max_rate,
      buffer_manager: DmaSamplingManager::new(config),
      oversampling_ratio: 1,
      decimation_filter: DecimationFilter::new(),
    }
  }

  /// 设置采样率
  pub fn set_sample_rate(&mut self, rate: u32) -> Result<(), &'static str> {
    if rate > self.max_sample_rate {
      return Err("Sample rate exceeds maximum");
    }

    self.sample_rate = rate;
    self.configure_timer_for_rate(rate)?;
    Ok(())
  }

  /// 启用过采样
  pub fn enable_oversampling(&mut self, ratio: u8) -> Result<(), &'static str> {
    if ratio == 0 || ratio > 16 {
      return Err("Invalid oversampling ratio");
    }

    self.oversampling_ratio = ratio;
    let actual_rate = self.sample_rate * ratio as u32;

    if actual_rate > self.max_sample_rate {
      return Err("Oversampled rate exceeds maximum");
    }

    self.configure_timer_for_rate(actual_rate)?;
    Ok(())
  }

  /// 配置定时器采样率
  fn configure_timer_for_rate(&mut self, rate: u32) -> Result<(), &'static str> {
    // 配置定时器以指定的频率触发ADC
    // 计算定时器预分频和重载值
    Ok(())
  }

  /// 处理高速采样数据
  pub fn process_high_speed_data(&mut self, data: &[u16]) -> Vec<u16, MAX_BUFFER_SIZE> {
    let mut processed = Vec::new();

    if self.oversampling_ratio > 1 {
      // 应用抽取滤波
      for chunk in data.chunks(self.oversampling_ratio as usize) {
        let decimated = self.decimation_filter.process(chunk);
        processed.push(decimated).ok();
      }
    } else {
      // 直接复制数据
      for &sample in data {
        processed.push(sample).ok();
      }
    }

    processed
  }

  /// 获取有效采样率
  pub fn get_effective_sample_rate(&self) -> u32 {
    self.sample_rate
  }

  /// 获取实际硬件采样率
  pub fn get_hardware_sample_rate(&self) -> u32 {
    self.sample_rate * self.oversampling_ratio as u32
  }
}

/// 抽取滤波器
pub struct DecimationFilter {
  coefficients: Vec<f32, 16>,
  delay_line: Vec<f32, 16>,
  index: usize,
}

impl DecimationFilter {
  pub fn new() -> Self {
    let mut filter = Self {
      coefficients: Vec::new(),
      delay_line: Vec::new(),
      index: 0,
    };

    // 初始化低通滤波器系数 (简单的移动平均)
    for _ in 0..8 {
      filter.coefficients.push(1.0 / 8.0).ok();
      filter.delay_line.push(0.0).ok();
    }

    filter
  }

  /// 处理数据块
  pub fn process(&mut self, samples: &[u16]) -> u16 {
    if samples.is_empty() {
      return 0;
    }

    // 计算平均值作为简单的抽取
    let sum: u32 = samples.iter().map(|&x| x as u32).sum();
    (sum / samples.len() as u32) as u16
  }

  /// 应用FIR滤波
  pub fn apply_fir_filter(&mut self, input: f32) -> f32 {
    // 更新延迟线
    self.delay_line[self.index] = input;
    self.index = (self.index + 1) % self.delay_line.len();

    // 计算滤波输出
    let mut output = 0.0;
    for i in 0..self.coefficients.len() {
      let delay_index = (self.index + i) % self.delay_line.len();
      output += self.coefficients[i] * self.delay_line[delay_index];
    }

    output
  }

  /// 重置滤波器
  pub fn reset(&mut self) {
    for sample in &mut self.delay_line {
      *sample = 0.0;
    }
    self.index = 0;
  }
}

/// DMA传输状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaTransferState {
  Idle,
  Active,
  HalfComplete,
  Complete,
  Error,
}

/// DMA传输监控器
pub struct DmaTransferMonitor {
  state: DmaTransferState,
  transfer_count: u32,
  error_count: u32,
  last_transfer_time: u32,
  performance_metrics: TransferMetrics,
}

#[derive(Debug, Clone)]
pub struct TransferMetrics {
  pub average_transfer_time: f32,
  pub max_transfer_time: u32,
  pub min_transfer_time: u32,
  pub throughput_mbps: f32,
}

impl DmaTransferMonitor {
  pub fn new() -> Self {
    Self {
      state: DmaTransferState::Idle,
      transfer_count: 0,
      error_count: 0,
      last_transfer_time: 0,
      performance_metrics: TransferMetrics {
        average_transfer_time: 0.0,
        max_transfer_time: 0,
        min_transfer_time: u32::MAX,
        throughput_mbps: 0.0,
      },
    }
  }

  /// 更新传输状态
  pub fn update_state(&mut self, new_state: DmaTransferState) {
    self.state = new_state;

    match new_state {
      DmaTransferState::Complete => {
        self.transfer_count += 1;
        self.update_performance_metrics();
      }
      DmaTransferState::Error => {
        self.error_count += 1;
      }
      _ => {}
    }
  }

  /// 更新性能指标
  fn update_performance_metrics(&mut self) {
    let current_time = self.get_current_time();
    let transfer_time = current_time - self.last_transfer_time;

    // 更新最大最小传输时间
    if transfer_time > self.performance_metrics.max_transfer_time {
      self.performance_metrics.max_transfer_time = transfer_time;
    }
    if transfer_time < self.performance_metrics.min_transfer_time {
      self.performance_metrics.min_transfer_time = transfer_time;
    }

    // 更新平均传输时间
    let alpha = 0.1; // 指数移动平均系数
    self.performance_metrics.average_transfer_time =
      alpha * transfer_time as f32 + (1.0 - alpha) * self.performance_metrics.average_transfer_time;

    self.last_transfer_time = current_time;
  }

  /// 获取当前时间 (模拟)
  fn get_current_time(&self) -> u32 {
    // 实际实现中应该使用系统定时器
    static mut TIME: u32 = 0;
    unsafe {
      TIME += 1;
      TIME
    }
  }

  /// 计算吞吐量
  pub fn calculate_throughput(&mut self, bytes_transferred: u32) {
    if self.performance_metrics.average_transfer_time > 0.0 {
      let throughput_bps =
        bytes_transferred as f32 / (self.performance_metrics.average_transfer_time / 1000000.0); // 转换为秒
      self.performance_metrics.throughput_mbps = throughput_bps / 1_000_000.0;
    }
  }

  /// 获取错误率
  pub fn get_error_rate(&self) -> f32 {
    if self.transfer_count + self.error_count == 0 {
      return 0.0;
    }
    self.error_count as f32 / (self.transfer_count + self.error_count) as f32
  }

  /// 重置统计信息
  pub fn reset_stats(&mut self) {
    self.transfer_count = 0;
    self.error_count = 0;
    self.performance_metrics = TransferMetrics {
      average_transfer_time: 0.0,
      max_transfer_time: 0,
      min_transfer_time: u32::MAX,
      throughput_mbps: 0.0,
    };
  }
}
