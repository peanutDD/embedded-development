#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use heapless::{String, Vec};
use nb::block;
use panic_halt as _;
use stm32f4xx_hal::{
  adc::{Adc, AdcConfig, SampleTime},
  gpio::Analog,
  pac::{self, interrupt, Interrupt},
  prelude::*,
  serial::{Config, Serial},
  timer::{Event, Timer},
};

use dma_sampling::CircularBuffer;

const BUFFER_SIZE: usize = 2048;
const PROCESSING_CHUNK_SIZE: usize = 256;

// 全局循环缓冲区
static CIRCULAR_BUFFER: Mutex<RefCell<CircularBuffer<u16, BUFFER_SIZE>>> =
  Mutex::new(RefCell::new(CircularBuffer::new()));
static BUFFER_HALF_FULL: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static BUFFER_OVERRUN: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = pac::Peripherals::take().unwrap();
  let mut cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();

  // 配置串口
  let tx_pin = gpioa.pa2.into_alternate();
  let rx_pin = gpioa.pa3.into_alternate();
  let mut serial = Serial::new(
    dp.USART2,
    (tx_pin, rx_pin),
    Config::default().baudrate(115200.bps()),
    &clocks,
  )
  .unwrap();

  // 配置ADC引脚
  let adc_pin = gpioa.pa0.into_analog(); // ADC1_IN0

  // 配置ADC
  let adc_config = AdcConfig::default().sample_time(SampleTime::Cycles_480);
  let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

  // 配置定时器用于模拟DMA采样
  let mut sample_timer = Timer::new(dp.TIM2, &clocks).counter_hz();
  sample_timer.start(20000.Hz()).unwrap(); // 20kHz采样率

  // 配置处理定时器
  let mut process_timer = Timer::new(dp.TIM3, &clocks).counter_hz();
  process_timer.start(100.Hz()).unwrap(); // 100Hz处理频率

  // 启用定时器中断
  cp.NVIC.enable(Interrupt::TIM2);

  writeln!(serial, "循环缓冲区DMA采样示例启动").ok();
  writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
  writeln!(serial, "采样率: 20kHz").ok();
  writeln!(serial, "缓冲区大小: {} 样本", BUFFER_SIZE).ok();
  writeln!(serial, "处理块大小: {} 样本", PROCESSING_CHUNK_SIZE).ok();
  writeln!(serial, "处理频率: 100Hz").ok();
  writeln!(serial, "").ok();

  let mut sample_count = 0u32;
  let mut process_count = 0u32;
  let vref = 3.3f32;

  // 数据处理缓冲区
  let mut processing_buffer = [0u16; PROCESSING_CHUNK_SIZE];

  loop {
    // 模拟ADC采样 (实际中由DMA中断处理)
    if sample_timer.wait().is_ok() {
      sample_count += 1;

      // 模拟ADC读取
      let adc_value = simulate_adc_reading(sample_count);

      // 写入循环缓冲区
      free(|cs| {
        let mut buffer = CIRCULAR_BUFFER.borrow(cs).borrow_mut();
        match buffer.write(adc_value) {
          Ok(()) => {
            // 检查缓冲区是否半满
            if buffer.used_space() >= BUFFER_SIZE / 2 {
              *BUFFER_HALF_FULL.borrow(cs).borrow_mut() = true;
            }
          }
          Err(_) => {
            *BUFFER_OVERRUN.borrow(cs).borrow_mut() = true;
          }
        }
      });
    }

    // 数据处理
    if process_timer.wait().is_ok() {
      process_count += 1;

      // 从循环缓冲区读取数据进行处理
      let samples_read = free(|cs| {
        let mut buffer = CIRCULAR_BUFFER.borrow(cs).borrow_mut();
        buffer.read_slice(&mut processing_buffer)
      });

      if samples_read > 0 {
        process_data_chunk(
          &mut serial,
          &processing_buffer[..samples_read],
          process_count,
          vref,
        );
      }

      // 每10次处理输出缓冲区状态
      if process_count % 10 == 0 {
        output_buffer_status(&mut serial, process_count);
      }
    }

    // 检查缓冲区溢出
    free(|cs| {
      if *BUFFER_OVERRUN.borrow(cs).borrow() {
        *BUFFER_OVERRUN.borrow(cs).borrow_mut() = false;
        writeln!(serial, "警告: 循环缓冲区溢出! (样本#{})", sample_count).ok();
      }
    });

    // 检查缓冲区半满状态
    free(|cs| {
      if *BUFFER_HALF_FULL.borrow(cs).borrow() {
        *BUFFER_HALF_FULL.borrow(cs).borrow_mut() = false;
        // 可以在这里触发额外的处理逻辑
      }
    });
  }
}

/// 处理数据块
fn process_data_chunk(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  data: &[u16],
  process_count: u32,
  vref: f32,
) {
  if data.is_empty() {
    return;
  }

  // 计算基本统计信息
  let stats = calculate_chunk_statistics(data, vref);

  // 每20次处理输出详细信息
  if process_count % 20 == 0 {
    writeln!(
      serial,
      "处理#{:04}: 样本数={}, 均值={:.3}V, 峰峰值={:.3}V, RMS={:.3}V",
      process_count,
      data.len(),
      stats.mean,
      stats.peak_to_peak,
      stats.rms
    )
    .ok();
  }

  // 执行实时信号分析
  perform_signal_analysis(serial, data, &stats, process_count);

  // 应用数字滤波
  let filtered_data = apply_digital_filter(data);

  // 检测信号特征
  detect_signal_features(serial, &filtered_data, process_count);
}

/// 数据块统计信息
#[derive(Debug)]
struct ChunkStatistics {
  mean: f32,
  max: f32,
  min: f32,
  peak_to_peak: f32,
  rms: f32,
  std_dev: f32,
}

/// 计算数据块统计信息
fn calculate_chunk_statistics(data: &[u16], vref: f32) -> ChunkStatistics {
  if data.is_empty() {
    return ChunkStatistics {
      mean: 0.0,
      max: 0.0,
      min: 0.0,
      peak_to_peak: 0.0,
      rms: 0.0,
      std_dev: 0.0,
    };
  }

  // 转换为电压
  let voltages: Vec<f32, PROCESSING_CHUNK_SIZE> = data
    .iter()
    .map(|&adc_val| adc_to_voltage(adc_val, vref, 12))
    .collect();

  // 基本统计
  let sum: f32 = voltages.iter().sum();
  let mean = sum / voltages.len() as f32;

  let max = voltages.iter().fold(0.0f32, |a, &b| a.max(b));
  let min = voltages.iter().fold(vref, |a, &b| a.min(b));
  let peak_to_peak = max - min;

  // RMS计算
  let sum_squares: f32 = voltages.iter().map(|&v| v * v).sum();
  use micromath::F32Ext;
  let rms = (sum_squares / voltages.len() as f32).sqrt();

  // 标准差
  let variance: f32 = voltages
    .iter()
    .map(|&v| {
      let diff = v - mean;
      diff * diff
    })
    .sum::<f32>()
    / voltages.len() as f32;
  let std_dev = variance.sqrt();

  ChunkStatistics {
    mean,
    max,
    min,
    peak_to_peak,
    rms,
    std_dev,
  }
}

/// 执行信号分析
fn perform_signal_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  data: &[u16],
  stats: &ChunkStatistics,
  process_count: u32,
) {
  // 信号质量评估
  let signal_quality = assess_signal_quality(stats);

  // 频率估计 (简单的过零检测)
  let estimated_freq = estimate_frequency(data, 20000.0); // 20kHz采样率

  // 每50次处理输出分析结果
  if process_count % 50 == 0 {
    writeln!(serial, "\n=== 信号分析 (处理#{}) ===", process_count).ok();
    writeln!(serial, "信号质量: {:.1}%", signal_quality * 100.0).ok();
    writeln!(serial, "估计频率: {:.1} Hz", estimated_freq).ok();
    writeln!(serial, "信噪比: {:.1} dB", calculate_snr(stats)).ok();
    writeln!(serial, "").ok();
  }

  // 异常检测
  if signal_quality < 0.5 {
    writeln!(
      serial,
      "警告: 信号质量低 {:.1}% (处理#{})",
      signal_quality * 100.0,
      process_count
    )
    .ok();
  }

  if estimated_freq > 1000.0 {
    writeln!(
      serial,
      "检测到高频信号: {:.1} Hz (处理#{})",
      estimated_freq, process_count
    )
    .ok();
  }
}

/// 评估信号质量
fn assess_signal_quality(stats: &ChunkStatistics) -> f32 {
  // 基于信噪比和稳定性的简单质量评估
  let snr_factor = if stats.mean > 0.0 {
    (stats.mean / (stats.std_dev + 0.001)).min(10.0) / 10.0
  } else {
    0.0
  };

  let amplitude_factor = if stats.peak_to_peak > 0.1 && stats.peak_to_peak < 3.0 {
    1.0
  } else {
    0.5
  };

  snr_factor * amplitude_factor
}

/// 估计频率 (过零检测)
fn estimate_frequency(data: &[u16], sample_rate: f32) -> f32 {
  if data.len() < 4 {
    return 0.0;
  }

  // 计算平均值作为阈值
  let sum: u32 = data.iter().map(|&x| x as u32).sum();
  let threshold = (sum / data.len() as u32) as u16;

  // 计算过零次数
  let mut zero_crossings = 0;
  let mut last_above = data[0] > threshold;

  for &sample in data.iter().skip(1) {
    let current_above = sample > threshold;
    if current_above != last_above {
      zero_crossings += 1;
    }
    last_above = current_above;
  }

  // 频率 = 过零次数 / 2 / 时间
  let time_duration = data.len() as f32 / sample_rate;
  (zero_crossings as f32 / 2.0) / time_duration
}

/// 计算信噪比
fn calculate_snr(stats: &ChunkStatistics) -> f32 {
  if stats.std_dev > 0.0 {
    use micromath::F32Ext;
    20.0 * (stats.mean / stats.std_dev).log10()
  } else {
    100.0 // 无噪声情况
  }
}

/// 应用数字滤波器
fn apply_digital_filter(data: &[u16]) -> Vec<u16, PROCESSING_CHUNK_SIZE> {
  let mut filtered = Vec::new();

  if data.len() < 5 {
    // 数据太少，直接返回
    for &sample in data {
      filtered.push(sample).ok();
    }
    return filtered;
  }

  // 简单的5点移动平均滤波器
  for i in 2..data.len() - 2 {
    let sum: u32 = (i - 2..=i + 2).map(|idx| data[idx] as u32).sum();
    let avg = (sum / 5) as u16;
    filtered.push(avg).ok();
  }

  filtered
}

/// 检测信号特征
fn detect_signal_features(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  data: &Vec<u16, PROCESSING_CHUNK_SIZE>,
  process_count: u32,
) {
  // 峰值检测
  let peaks = detect_peaks(data);

  // 边沿检测
  let edges = detect_edges(data);

  // 每100次处理输出特征检测结果
  if process_count % 100 == 0 && (!peaks.is_empty() || !edges.is_empty()) {
    writeln!(
      serial,
      "特征检测 (处理#{}): 峰值={}, 边沿={}",
      process_count,
      peaks.len(),
      edges.len()
    )
    .ok();
  }
}

/// 峰值检测
fn detect_peaks(data: &Vec<u16, PROCESSING_CHUNK_SIZE>) -> Vec<usize, 32> {
  let mut peaks = Vec::new();

  if data.len() < 3 {
    return peaks;
  }

  for i in 1..data.len() - 1 {
    if data[i] > data[i - 1] && data[i] > data[i + 1] {
      // 简单的局部最大值检测
      if data[i] > 2048 + 200 {
        // 阈值检测
        peaks.push(i).ok();
      }
    }
  }

  peaks
}

/// 边沿检测
fn detect_edges(data: &Vec<u16, PROCESSING_CHUNK_SIZE>) -> Vec<usize, 32> {
  let mut edges = Vec::new();

  if data.len() < 2 {
    return edges;
  }

  const EDGE_THRESHOLD: u16 = 100;

  for i in 1..data.len() {
    let diff = if data[i] > data[i - 1] {
      data[i] - data[i - 1]
    } else {
      data[i - 1] - data[i]
    };

    if diff > EDGE_THRESHOLD {
      edges.push(i).ok();
    }
  }

  edges
}

/// 输出缓冲区状态
fn output_buffer_status(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  process_count: u32,
) {
  free(|cs| {
    let buffer = CIRCULAR_BUFFER.borrow(cs).borrow();

    writeln!(
      serial,
      "缓冲区状态 (处理#{}): 已用={}/{}, 利用率={:.1}%",
      process_count,
      buffer.used_space(),
      buffer.capacity(),
      (buffer.used_space() as f32 / buffer.capacity() as f32) * 100.0
    )
    .ok();

    if buffer.is_full() {
      writeln!(serial, "警告: 缓冲区已满!").ok();
    }
  });
}

/// 定时器中断处理 (模拟DMA采样)
#[interrupt]
fn TIM2() {
  // 在实际应用中，这里会由DMA中断处理
  // 这里只是清除中断标志
  // TIM2.sr.modify(|_, w| w.uif().clear_bit());
}

/// ADC值转换为电压
fn adc_to_voltage(adc_value: u16, vref: f32, resolution: u8) -> f32 {
  let max_value = (1 << resolution) - 1;
  (adc_value as f32 / max_value as f32) * vref
}

/// 模拟ADC读取
fn simulate_adc_reading(sample_count: u32) -> u16 {
  use micromath::F32Ext;

  // 生成复合信号：直流 + 正弦波 + 噪声
  let dc_offset = 2048.0; // 1.65V直流偏移
  let sine_amplitude = 800.0;
  let sine_freq = 100.0; // 100Hz正弦波
  let sample_rate = 20000.0; // 20kHz采样率

  let t = sample_count as f32 / sample_rate;
  let sine_component = sine_amplitude * (2.0 * core::f32::consts::PI * sine_freq * t).sin();

  // 添加噪声
  let noise = ((sample_count * 7) % 21) as f32 - 10.0;

  let result = dc_offset + sine_component + noise;
  result.max(0.0).min(4095.0) as u16
}

/// 循环缓冲区性能监控
struct BufferPerformanceMonitor {
  max_usage: usize,
  total_writes: u32,
  total_reads: u32,
  overrun_count: u32,
  underrun_count: u32,
}

impl BufferPerformanceMonitor {
  fn new() -> Self {
    Self {
      max_usage: 0,
      total_writes: 0,
      total_reads: 0,
      overrun_count: 0,
      underrun_count: 0,
    }
  }

  fn record_write(&mut self, success: bool, current_usage: usize) {
    self.total_writes += 1;
    if !success {
      self.overrun_count += 1;
    }
    if current_usage > self.max_usage {
      self.max_usage = current_usage;
    }
  }

  fn record_read(&mut self, samples_read: usize) {
    self.total_reads += 1;
    if samples_read == 0 {
      self.underrun_count += 1;
    }
  }

  fn get_efficiency(&self) -> f32 {
    if self.total_writes == 0 {
      return 0.0;
    }
    (self.total_writes - self.overrun_count) as f32 / self.total_writes as f32
  }
}
