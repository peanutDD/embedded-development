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
  dma::{PeripheralToMemory, Stream0, StreamsTuple, Transfer},
  gpio::Analog,
  pac::{self, interrupt, Interrupt},
  prelude::*,
  serial::{Config, Serial},
  timer::{Event, Timer},
};

const SYNC_CHANNELS: usize = 6; // ADC1: 2通道, ADC2: 2通道, ADC3: 2通道
const BUFFER_SIZE: usize = 128;

// 同步采样结果结构
#[derive(Debug, Clone)]
struct SyncSampleResult {
  adc1_ch1: u16,
  adc1_ch2: u16,
  adc2_ch1: u16,
  adc2_ch2: u16,
  adc3_ch1: u16,
  adc3_ch2: u16,
  timestamp: u32,
}

// 全局变量用于中断处理
static SYNC_RESULTS: Mutex<RefCell<Option<SyncSampleResult>>> = Mutex::new(RefCell::new(None));
static SYNC_COMPLETE: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

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
  let gpioc = dp.GPIOC.split();

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
  // ADC1通道
  let _adc1_pin0 = gpioa.pa0.into_analog(); // ADC1_IN0
  let _adc1_pin1 = gpioa.pa1.into_analog(); // ADC1_IN1

  // ADC2通道
  let _adc2_pin2 = gpioa.pa2.into_analog(); // ADC2_IN2
  let _adc2_pin3 = gpioa.pa3.into_analog(); // ADC2_IN3

  // ADC3通道
  let _adc3_pin0 = gpioa.pa0.into_analog(); // ADC3_IN0 (共享引脚)
  let _adc3_pin1 = gpioa.pa1.into_analog(); // ADC3_IN1 (共享引脚)

  // 配置ADC
  let adc_config = AdcConfig::default().sample_time(SampleTime::Cycles_480);

  let mut adc1 = Adc::adc1(dp.ADC1, true, adc_config);
  // let mut adc2 = Adc::adc2(dp.ADC2, true, adc_config);
  // let mut adc3 = Adc::adc3(dp.ADC3, true, adc_config);

  // 配置同步触发定时器
  let mut sync_timer = Timer::new(dp.TIM1, &clocks).counter_hz();
  sync_timer.start(1000.Hz()).unwrap(); // 1kHz同步采样

  // 启用ADC中断
  cp.NVIC.enable(Interrupt::ADC);

  writeln!(serial, "多ADC同步采样示例启动").ok();
  writeln!(serial, "ADC1: PA0, PA1").ok();
  writeln!(serial, "ADC2: PA2, PA3").ok();
  writeln!(serial, "ADC3: PA0, PA1 (共享)").ok();
  writeln!(serial, "同步采样频率: 1kHz").ok();
  writeln!(serial, "同步模式: 三ADC常规同时模式").ok();
  writeln!(serial, "").ok();

  // 配置多ADC同步模式
  configure_multi_adc_sync_mode();

  let mut sample_count = 0u32;
  let vref = 3.3f32;

  // 数据分析缓冲区
  let mut analysis_buffer: Vec<SyncSampleResult, BUFFER_SIZE> = Vec::new();

  loop {
    // 等待同步触发
    if sync_timer.wait().is_ok() {
      // 触发同步转换
      trigger_sync_conversion();
      sample_count += 1;
    }

    // 检查同步转换完成
    free(|cs| {
      if *SYNC_COMPLETE.borrow(cs).borrow() {
        *SYNC_COMPLETE.borrow(cs).borrow_mut() = false;

        if let Some(result) = SYNC_RESULTS.borrow(cs).borrow_mut().take() {
          // 处理同步采样结果
          process_sync_result(&mut serial, &result, sample_count, vref);

          // 添加到分析缓冲区
          if analysis_buffer.len() >= BUFFER_SIZE {
            analysis_buffer.remove(0);
          }
          analysis_buffer.push(result).ok();

          // 每1000个样本进行分析
          if sample_count % 1000 == 0 {
            perform_sync_analysis(&mut serial, &analysis_buffer, sample_count);
          }
        }
      }
    });

    // 检测同步性能
    if sample_count % 5000 == 0 {
      check_sync_performance(&mut serial, sample_count);
    }
  }
}

/// 配置多ADC同步模式
fn configure_multi_adc_sync_mode() {
  // 配置ADC_CCR寄存器
  // MULTI[4:0] = 10110 (三ADC常规同时模式)
  // DELAY[3:0] = 0000 (无延迟)
  // DDS = 0 (DMA禁用数据打包)
  // DMA[1:0] = 01 (DMA模式1)
  // ADCPRE[1:0] = 00 (PCLK2/2)
  // VBATE = 0, TSVREFE = 0

  // 配置ADC1为主ADC
  // 配置ADC2和ADC3为从ADC

  // 启用转换完成中断
}

/// 触发同步转换
fn trigger_sync_conversion() {
  // 设置ADC1的SWSTART位
  // 这将同时触发所有配置的ADC开始转换
}

/// 处理同步采样结果
fn process_sync_result(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  result: &SyncSampleResult,
  count: u32,
  vref: f32,
) {
  // 每100个样本输出一次详细数据
  if count % 100 == 0 {
    let mut output: String<512> = String::new();
    write!(output, "同步#{:05}: ", count).ok();

    // ADC1数据
    let v1_ch1 = adc_to_voltage(result.adc1_ch1, vref, 12);
    let v1_ch2 = adc_to_voltage(result.adc1_ch2, vref, 12);
    write!(output, "ADC1[{:.3}V,{:.3}V] ", v1_ch1, v1_ch2).ok();

    // ADC2数据
    let v2_ch1 = adc_to_voltage(result.adc2_ch1, vref, 12);
    let v2_ch2 = adc_to_voltage(result.adc2_ch2, vref, 12);
    write!(output, "ADC2[{:.3}V,{:.3}V] ", v2_ch1, v2_ch2).ok();

    // ADC3数据
    let v3_ch1 = adc_to_voltage(result.adc3_ch1, vref, 12);
    let v3_ch2 = adc_to_voltage(result.adc3_ch2, vref, 12);
    write!(output, "ADC3[{:.3}V,{:.3}V]", v3_ch1, v3_ch2).ok();

    writeln!(serial, "{}", output).ok();
  }

  // 实时监控关键参数
  monitor_critical_parameters(serial, result, count, vref);
}

/// 监控关键参数
fn monitor_critical_parameters(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  result: &SyncSampleResult,
  count: u32,
  vref: f32,
) {
  // 检查电压范围
  let voltages = [
    adc_to_voltage(result.adc1_ch1, vref, 12),
    adc_to_voltage(result.adc1_ch2, vref, 12),
    adc_to_voltage(result.adc2_ch1, vref, 12),
    adc_to_voltage(result.adc2_ch2, vref, 12),
    adc_to_voltage(result.adc3_ch1, vref, 12),
    adc_to_voltage(result.adc3_ch2, vref, 12),
  ];

  for (i, &voltage) in voltages.iter().enumerate() {
    if voltage > 3.1 || voltage < 0.2 {
      writeln!(
        serial,
        "警告: 通道{}电压异常 {:.3}V (样本#{})",
        i, voltage, count
      )
      .ok();
    }
  }

  // 检查通道间相关性 (假设某些通道应该相关)
  let correlation = calculate_simple_correlation(
    adc_to_voltage(result.adc1_ch1, vref, 12),
    adc_to_voltage(result.adc3_ch1, vref, 12),
  );

  if correlation < 0.8 && count > 100 {
    writeln!(
      serial,
      "警告: ADC1_CH1与ADC3_CH1相关性低 {:.3} (样本#{})",
      correlation, count
    )
    .ok();
  }
}

/// 执行同步分析
fn perform_sync_analysis(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  buffer: &Vec<SyncSampleResult, BUFFER_SIZE>,
  count: u32,
) {
  writeln!(serial, "\n=== 同步采样分析 (样本#{}) ===", count).ok();

  if buffer.is_empty() {
    return;
  }

  // 计算各通道统计信息
  let mut channel_stats = Vec::<ChannelStatistics, SYNC_CHANNELS>::new();

  for i in 0..SYNC_CHANNELS {
    let values: Vec<u16, BUFFER_SIZE> = buffer
      .iter()
      .map(|sample| get_channel_value(sample, i))
      .collect();

    let stats = calculate_channel_statistics(&values);
    channel_stats.push(stats).ok();

    writeln!(
      serial,
      "通道{}: 均值={:.1}, 标准差={:.1}, 范围=[{}-{}]",
      i, stats.mean, stats.std_dev, stats.min, stats.max
    )
    .ok();
  }

  // 计算同步性能指标
  let sync_jitter = calculate_sync_jitter(buffer);
  writeln!(serial, "同步抖动: {:.2}μs", sync_jitter).ok();

  // 计算通道间相关性
  calculate_channel_correlations(serial, buffer);

  writeln!(serial, "").ok();
}

/// 检查同步性能
fn check_sync_performance(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  count: u32,
) {
  writeln!(serial, "\n=== 同步性能检查 (样本#{}) ===", count).ok();

  // 检查采样率稳定性
  let expected_rate = 1000.0; // 1kHz
  let actual_rate = count as f32 / (count as f32 / expected_rate);
  let rate_error = ((actual_rate - expected_rate) / expected_rate * 100.0).abs();

  writeln!(serial, "期望采样率: {:.1}Hz", expected_rate).ok();
  writeln!(serial, "实际采样率: {:.1}Hz", actual_rate).ok();
  writeln!(serial, "采样率误差: {:.2}%", rate_error).ok();

  if rate_error > 1.0 {
    writeln!(serial, "警告: 采样率误差过大!").ok();
  }

  // 检查ADC同步延迟
  let sync_delay = measure_adc_sync_delay();
  writeln!(serial, "ADC同步延迟: {:.1}ns", sync_delay).ok();

  if sync_delay > 100.0 {
    writeln!(serial, "警告: ADC同步延迟过大!").ok();
  }

  writeln!(serial, "").ok();
}

/// ADC中断处理函数
#[interrupt]
fn ADC() {
  free(|cs| {
    // 检查转换完成标志
    // if ADC1.SR.read().eoc().bit_is_set() {
    // 读取所有ADC的数据寄存器
    let result = SyncSampleResult {
      adc1_ch1: read_adc_data_register(1, 1),
      adc1_ch2: read_adc_data_register(1, 2),
      adc2_ch1: read_adc_data_register(2, 1),
      adc2_ch2: read_adc_data_register(2, 2),
      adc3_ch1: read_adc_data_register(3, 1),
      adc3_ch2: read_adc_data_register(3, 2),
      timestamp: get_timestamp(),
    };

    // 存储结果
    *SYNC_RESULTS.borrow(cs).borrow_mut() = Some(result);
    *SYNC_COMPLETE.borrow(cs).borrow_mut() = true;

    // 清除中断标志
    // ADC1.SR.modify(|_, w| w.eoc().clear_bit());
    // }
  });
}

/// 读取ADC数据寄存器
fn read_adc_data_register(adc_num: u8, channel: u8) -> u16 {
  // 模拟读取ADC数据寄存器
  simulate_sync_adc_reading(adc_num, channel)
}

/// 获取时间戳
fn get_timestamp() -> u32 {
  // 返回当前时间戳 (微秒)
  static mut TIMESTAMP: u32 = 0;
  unsafe {
    TIMESTAMP += 1000; // 假设1ms递增
    TIMESTAMP
  }
}

/// 获取通道值
fn get_channel_value(sample: &SyncSampleResult, channel: usize) -> u16 {
  match channel {
    0 => sample.adc1_ch1,
    1 => sample.adc1_ch2,
    2 => sample.adc2_ch1,
    3 => sample.adc2_ch2,
    4 => sample.adc3_ch1,
    5 => sample.adc3_ch2,
    _ => 0,
  }
}

/// 通道统计信息
#[derive(Debug, Clone)]
struct ChannelStatistics {
  mean: f32,
  std_dev: f32,
  min: u16,
  max: u16,
}

/// 计算通道统计信息
fn calculate_channel_statistics(values: &Vec<u16, BUFFER_SIZE>) -> ChannelStatistics {
  if values.is_empty() {
    return ChannelStatistics {
      mean: 0.0,
      std_dev: 0.0,
      min: 0,
      max: 0,
    };
  }

  let sum: u32 = values.iter().map(|&x| x as u32).sum();
  let mean = sum as f32 / values.len() as f32;

  let variance: f32 = values
    .iter()
    .map(|&x| {
      let diff = x as f32 - mean;
      diff * diff
    })
    .sum::<f32>()
    / values.len() as f32;

  use micromath::F32Ext;
  let std_dev = variance.sqrt();

  let min = *values.iter().min().unwrap();
  let max = *values.iter().max().unwrap();

  ChannelStatistics {
    mean,
    std_dev,
    min,
    max,
  }
}

/// 计算同步抖动
fn calculate_sync_jitter(buffer: &Vec<SyncSampleResult, BUFFER_SIZE>) -> f32 {
  if buffer.len() < 2 {
    return 0.0;
  }

  let mut intervals = Vec::<u32, BUFFER_SIZE>::new();

  for i in 1..buffer.len() {
    let interval = buffer[i].timestamp - buffer[i - 1].timestamp;
    intervals.push(interval).ok();
  }

  // 计算间隔的标准差作为抖动指标
  if intervals.is_empty() {
    return 0.0;
  }

  let sum: u32 = intervals.iter().sum();
  let mean = sum as f32 / intervals.len() as f32;

  let variance: f32 = intervals
    .iter()
    .map(|&x| {
      let diff = x as f32 - mean;
      diff * diff
    })
    .sum::<f32>()
    / intervals.len() as f32;

  use micromath::F32Ext;
  variance.sqrt() // 返回微秒单位的抖动
}

/// 计算通道间相关性
fn calculate_channel_correlations(
  serial: &mut Serial<
    pac::USART2,
    (
      stm32f4xx_hal::gpio::Pin<'A', 2>,
      stm32f4xx_hal::gpio::Pin<'A', 3>,
    ),
  >,
  buffer: &Vec<SyncSampleResult, BUFFER_SIZE>,
) {
  // 计算ADC1_CH1与ADC3_CH1的相关性 (共享引脚)
  let correlation_1_3 = calculate_correlation(
    buffer.iter().map(|s| s.adc1_ch1 as f32),
    buffer.iter().map(|s| s.adc3_ch1 as f32),
  );

  writeln!(
    serial,
    "ADC1_CH1 vs ADC3_CH1 相关性: {:.3}",
    correlation_1_3
  )
  .ok();

  // 计算ADC1_CH2与ADC3_CH2的相关性
  let correlation_2_3 = calculate_correlation(
    buffer.iter().map(|s| s.adc1_ch2 as f32),
    buffer.iter().map(|s| s.adc3_ch2 as f32),
  );

  writeln!(
    serial,
    "ADC1_CH2 vs ADC3_CH2 相关性: {:.3}",
    correlation_2_3
  )
  .ok();
}

/// 计算相关系数
fn calculate_correlation<I1, I2>(x_iter: I1, y_iter: I2) -> f32
where
  I1: Iterator<Item = f32>,
  I2: Iterator<Item = f32>,
{
  let x_values: Vec<f32, BUFFER_SIZE> = x_iter.collect();
  let y_values: Vec<f32, BUFFER_SIZE> = y_iter.collect();

  if x_values.len() != y_values.len() || x_values.is_empty() {
    return 0.0;
  }

  let n = x_values.len() as f32;
  let sum_x: f32 = x_values.iter().sum();
  let sum_y: f32 = y_values.iter().sum();
  let sum_xy: f32 = x_values
    .iter()
    .zip(y_values.iter())
    .map(|(&x, &y)| x * y)
    .sum();
  let sum_x2: f32 = x_values.iter().map(|&x| x * x).sum();
  let sum_y2: f32 = y_values.iter().map(|&y| y * y).sum();

  let numerator = n * sum_xy - sum_x * sum_y;
  let denominator_x = n * sum_x2 - sum_x * sum_x;
  let denominator_y = n * sum_y2 - sum_y * sum_y;

  use micromath::F32Ext;
  let denominator = (denominator_x * denominator_y).sqrt();

  if denominator == 0.0 {
    0.0
  } else {
    numerator / denominator
  }
}

/// 简单相关性计算
fn calculate_simple_correlation(x: f32, y: f32) -> f32 {
  // 简化的相关性计算，实际应用中需要更复杂的算法
  1.0 - (x - y).abs() / (x + y + 0.001)
}

/// 测量ADC同步延迟
fn measure_adc_sync_delay() -> f32 {
  // 模拟测量ADC之间的同步延迟
  // 实际实现需要使用高精度定时器
  50.0 // 返回50ns的模拟延迟
}

/// ADC值转换为电压
fn adc_to_voltage(adc_value: u16, vref: f32, resolution: u8) -> f32 {
  let max_value = (1 << resolution) - 1;
  (adc_value as f32 / max_value as f32) * vref
}

/// 模拟同步ADC读取
fn simulate_sync_adc_reading(adc_num: u8, channel: u8) -> u16 {
  use micromath::F32Ext;

  static mut COUNTER: u32 = 0;

  unsafe {
    COUNTER += 1;

    let base_value = match (adc_num, channel) {
      (1, 1) => 2048,                         // ADC1_CH1
      (1, 2) => 1024,                         // ADC1_CH2
      (2, 1) => 3072,                         // ADC2_CH1
      (2, 2) => 1536,                         // ADC2_CH2
      (3, 1) => 2048 + (COUNTER % 10) as i16, // ADC3_CH1 (与ADC1_CH1相似但有小差异)
      (3, 2) => 1024 + (COUNTER % 8) as i16,  // ADC3_CH2 (与ADC1_CH2相似但有小差异)
      _ => 2048,
    };

    // 添加同步噪声
    let sync_noise = ((COUNTER * 7) % 5) as i16 - 2;
    (base_value + sync_noise).max(0).min(4095) as u16
  }
}
