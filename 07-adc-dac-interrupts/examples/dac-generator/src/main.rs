#![no_std]
#![no_main]

// DAC信号生成器示例
// 功能：生成正弦波、方波、三角波、锯齿波等多种波形
// 硬件：STM32F407VG Discovery
// 输出：PA4 (DAC1_OUT), PA5 (DAC2_OUT)
// 连接：PA4 -> 示波器/LED指示电路, PA5 -> 示波器/LED指示电路

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};

use stm32f4xx_hal::{
  dac::{DacExt, DacOut, DacPin, C1, C2},
  gpio::{Analog, Pin},
  pac,
  prelude::*,
  timer::TimerExt,
};

use libm::sinf;
// use micromath::F32Ext;
// use num_traits::float::Float;

// DAC配置常量
const DAC_RESOLUTION: u16 = 4095; // 12位DAC最大值
const VREF_VOLTAGE: f32 = 3.3; // 参考电压 (V)
const UPDATE_FREQUENCY_HZ: u32 = 10000; // DAC更新频率 (Hz)
const WAVE_FREQUENCY_HZ: f32 = 100.0; // 波形频率 (Hz)
const SAMPLES_PER_CYCLE: usize = (UPDATE_FREQUENCY_HZ as f32 / WAVE_FREQUENCY_HZ) as usize;

// 波形类型枚举
#[derive(Clone, Copy, Debug)]
enum WaveformType {
  Sine,     // 正弦波
  Square,   // 方波
  Triangle, // 三角波
  Sawtooth, // 锯齿波
  Noise,    // 伪随机噪声
  DC,       // 直流
}

// 波形生成器结构
struct WaveformGenerator {
  waveform_type: WaveformType,
  amplitude: f32,      // 幅度 (0.0 - 1.0)
  offset: f32,         // 直流偏移 (0.0 - 1.0)
  phase: f32,          // 相位 (0.0 - 2π)
  frequency: f32,      // 频率 (Hz)
  sample_index: usize, // 当前样本索引
  lfsr: u16,           // 线性反馈移位寄存器 (用于噪声生成)
}

impl WaveformGenerator {
  fn new(waveform_type: WaveformType) -> Self {
    Self {
      waveform_type,
      amplitude: 0.8, // 默认80%幅度
      offset: 0.5,    // 默认中心偏移
      phase: 0.0,
      frequency: WAVE_FREQUENCY_HZ,
      sample_index: 0,
      lfsr: 0xACE1, // 噪声生成种子
    }
  }

  fn set_amplitude(&mut self, amplitude: f32) {
    self.amplitude = amplitude.max(0.0).min(1.0);
  }

  fn set_offset(&mut self, offset: f32) {
    self.offset = offset.max(0.0).min(1.0);
  }

  fn set_frequency(&mut self, frequency: f32) {
    self.frequency = frequency.max(0.1).min(5000.0);
  }

  fn next_sample(&mut self) -> f32 {
    let samples_per_cycle = UPDATE_FREQUENCY_HZ as f32 / self.frequency;
    let normalized_phase = (self.sample_index as f32 / samples_per_cycle) % 1.0;
    let phase_radians = normalized_phase * 2.0 * core::f32::consts::PI + self.phase;

    let raw_value = match self.waveform_type {
      WaveformType::Sine => sinf(phase_radians),
      WaveformType::Square => {
        if sinf(phase_radians) >= 0.0 {
          1.0
        } else {
          -1.0
        }
      }
      WaveformType::Triangle => {
        let t = normalized_phase;
        if t < 0.5 {
          4.0 * t - 1.0
        } else {
          3.0 - 4.0 * t
        }
      }
      WaveformType::Sawtooth => 2.0 * normalized_phase - 1.0,
      WaveformType::Noise => {
        // 线性反馈移位寄存器生成伪随机数
        let bit = ((self.lfsr >> 0) ^ (self.lfsr >> 2) ^ (self.lfsr >> 3) ^ (self.lfsr >> 5)) & 1;
        self.lfsr = (self.lfsr >> 1) | (bit << 15);
        (self.lfsr as f32 / 32768.0) - 1.0
      }
      WaveformType::DC => {
        0.0 // 直流分量由offset控制
      }
    };

    self.sample_index = (self.sample_index + 1) % (samples_per_cycle as usize).max(1);

    // 应用幅度和偏移
    self.offset + raw_value * self.amplitude * 0.5
  }
}

// 电压转换为DAC值
fn voltage_to_dac(voltage: f32) -> u16 {
  let normalized = (voltage / VREF_VOLTAGE).max(0.0).min(1.0);
  (normalized * DAC_RESOLUTION as f32) as u16
}

// DAC值转换为电压
fn dac_to_voltage(dac_value: u16) -> f32 {
  (dac_value as f32 / DAC_RESOLUTION as f32) * VREF_VOLTAGE
}

// 格式化电压显示
fn format_voltage(voltage: f32) -> (u32, u32) {
  let rounded = (voltage * 1000.0) as u32;
  (rounded / 1000, rounded % 1000)
}

// 波形类型名称
fn waveform_name(waveform: WaveformType) -> &'static str {
  match waveform {
    WaveformType::Sine => "正弦波",
    WaveformType::Square => "方波",
    WaveformType::Triangle => "三角波",
    WaveformType::Sawtooth => "锯齿波",
    WaveformType::Noise => "噪声",
    WaveformType::DC => "直流",
  }
}

#[entry]
fn main() -> ! {
  // 初始化RTT调试输出
  rtt_init_print!();
  rprintln!("\n=== STM32F4 DAC信号生成器示例 ===");
  rprintln!("功能: 多种波形输出和实时控制");
  rprintln!("硬件: STM32F407VG Discovery");
  rprintln!("输出: PA4 (DAC1), PA5 (DAC2)\n");

  // 获取外设句柄
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(8.MHz()) // 使用外部8MHz晶振
    .sysclk(168.MHz()) // 系统时钟168MHz
    .pclk1(42.MHz()) // APB1时钟42MHz
    .pclk2(84.MHz()) // APB2时钟84MHz
    .freeze();

  rprintln!("时钟配置:");
  rprintln!("  SYSCLK: {} MHz", clocks.sysclk().to_MHz());
  rprintln!("  PCLK1:  {} MHz", clocks.pclk1().to_MHz());
  rprintln!("  PCLK2:  {} MHz", clocks.pclk2().to_MHz());

  // 配置GPIO
  let gpioa = dp.GPIOA.split();

  // 配置DAC输出引脚
  let dac1_pin: Pin<'A', 4, Analog> = gpioa.pa4.into_analog();
  let dac2_pin: Pin<'A', 5, Analog> = gpioa.pa5.into_analog();

  rprintln!("\nGPIO配置:");
  rprintln!("  PA4: DAC1_OUT (模拟输出)");
  rprintln!("  PA5: DAC2_OUT (模拟输出)");

  // 配置DAC
  let (mut dac1, mut dac2) = dp.DAC.constrain((dac1_pin, dac2_pin));
  dac1.enable();
  dac2.enable();

  rprintln!("\nDAC配置:");
  rprintln!("  分辨率: 12位 ({} 级)", DAC_RESOLUTION + 1);
  rprintln!(
    "  参考电压: {}.{}V",
    (VREF_VOLTAGE as u32),
    ((VREF_VOLTAGE * 10.0) as u32) % 10
  );
  rprintln!(
    "  输出范围: 0V - {}.{}V",
    (VREF_VOLTAGE as u32),
    ((VREF_VOLTAGE * 10.0) as u32) % 10
  );

  // 配置定时器用于DAC更新
  let timer_period_us = 1_000_000 / UPDATE_FREQUENCY_HZ;
  let mut timer = dp.TIM2.counter_us(&clocks);
  timer.start(timer_period_us.micros()).unwrap();

  rprintln!("\n定时器配置:");
  rprintln!("  更新频率: {} Hz", UPDATE_FREQUENCY_HZ);
  rprintln!("  更新周期: {} μs", timer_period_us);
  rprintln!("  波形频率: {} Hz", WAVE_FREQUENCY_HZ);
  rprintln!("  每周期样本: {}", SAMPLES_PER_CYCLE);

  // 创建波形生成器
  let mut gen1 = WaveformGenerator::new(WaveformType::Sine);
  let mut gen2 = WaveformGenerator::new(WaveformType::Square);

  // 设置不同的参数
  gen1.set_amplitude(0.8);
  gen1.set_offset(0.5);
  gen1.set_frequency(100.0);

  gen2.set_amplitude(0.6);
  gen2.set_offset(0.5);
  gen2.set_frequency(200.0);

  rprintln!("\n波形生成器配置:");
  rprintln!(
    "  DAC1: {} - 幅度: {:.1}%, 偏移: {:.1}V, 频率: {:.1}Hz",
    waveform_name(gen1.waveform_type),
    gen1.amplitude * 100.0,
    gen1.offset * VREF_VOLTAGE,
    gen1.frequency
  );
  rprintln!(
    "  DAC2: {} - 幅度: {:.1}%, 偏移: {:.1}V, 频率: {:.1}Hz",
    waveform_name(gen2.waveform_type),
    gen2.amplitude * 100.0,
    gen2.offset * VREF_VOLTAGE,
    gen2.frequency
  );

  // 波形切换序列
  let waveforms = [
    WaveformType::Sine,
    WaveformType::Square,
    WaveformType::Triangle,
    WaveformType::Sawtooth,
    WaveformType::Noise,
    WaveformType::DC,
  ];

  let mut waveform_index = 0;
  let mut sample_count = 0u32;
  let mut switch_count = 0u32;
  let mut last_dac1_voltage = 0.0f32;
  let mut last_dac2_voltage = 0.0f32;

  rprintln!("\n开始DAC输出...");
  rprintln!("每5秒自动切换波形类型\n");

  loop {
    // 等待定时器超时
    nb::block!(timer.wait()).unwrap();

    // 生成下一个样本
    let sample1 = gen1.next_sample();
    let sample2 = gen2.next_sample();

    // 转换为DAC值
    let dac1_value = voltage_to_dac(sample1 * VREF_VOLTAGE);
    let dac2_value = voltage_to_dac(sample2 * VREF_VOLTAGE);

    // 输出到DAC
    dac1.set_value(dac1_value);
    dac2.set_value(dac2_value);

    // 计算实际输出电压
    let dac1_voltage = dac_to_voltage(dac1_value);
    let dac2_voltage = dac_to_voltage(dac2_value);

    sample_count += 1;

    // 每1000个样本输出一次状态
    if sample_count % 1000 == 0 {
      let (v1_int, v1_frac) = format_voltage(dac1_voltage);
      let (v2_int, v2_frac) = format_voltage(dac2_voltage);

      // 计算电压变化
      let delta1 = dac1_voltage - last_dac1_voltage;
      let delta2 = dac2_voltage - last_dac2_voltage;

      rprintln!(
        "[{:6}] DAC1: {:4} ({}.{:03}V) | DAC2: {:4} ({}.{:03}V) | Δ: {:+.3}V, {:+.3}V",
        sample_count / 1000,
        dac1_value,
        v1_int,
        v1_frac,
        dac2_value,
        v2_int,
        v2_frac,
        delta1,
        delta2
      );

      last_dac1_voltage = dac1_voltage;
      last_dac2_voltage = dac2_voltage;
    }

    // 每5秒切换波形 (50000个样本 @ 10kHz)
    if sample_count % 50000 == 0 {
      switch_count += 1;

      // 切换DAC1波形
      waveform_index = (waveform_index + 1) % waveforms.len();
      gen1.waveform_type = waveforms[waveform_index];
      gen1.sample_index = 0; // 重置相位

      // 切换DAC2波形 (延迟一个波形)
      let dac2_index = (waveform_index + 3) % waveforms.len();
      gen2.waveform_type = waveforms[dac2_index];
      gen2.sample_index = 0; // 重置相位

      rprintln!("\n=== 波形切换 #{} ===", switch_count);
      rprintln!(
        "  DAC1: {} ({:.1}Hz)",
        waveform_name(gen1.waveform_type),
        gen1.frequency
      );
      rprintln!(
        "  DAC2: {} ({:.1}Hz)",
        waveform_name(gen2.waveform_type),
        gen2.frequency
      );
      rprintln!("  运行时间: {} 秒\n", sample_count / UPDATE_FREQUENCY_HZ);

      // 每次切换时调整参数
      match switch_count % 4 {
        0 => {
          gen1.set_amplitude(0.8);
          gen2.set_amplitude(0.6);
        }
        1 => {
          gen1.set_amplitude(0.9);
          gen2.set_amplitude(0.7);
        }
        2 => {
          gen1.set_amplitude(0.7);
          gen2.set_amplitude(0.8);
        }
        _ => {
          gen1.set_amplitude(0.6);
          gen2.set_amplitude(0.9);
        }
      }
    }

    // 每30秒输出系统状态
    if sample_count % 300000 == 0 {
      rprintln!("\n=== 系统状态 ===");
      rprintln!("  总样本数: {}", sample_count);
      rprintln!(
        "  运行时间: {} 分钟",
        sample_count / UPDATE_FREQUENCY_HZ / 60
      );
      rprintln!("  波形切换次数: {}", switch_count);
      rprintln!("  DAC更新频率: {} Hz", UPDATE_FREQUENCY_HZ);
      rprintln!("  内存使用: 正常");
      rprintln!("  DAC状态: 正常输出\n");
    }

    // 防止计数器溢出
    if sample_count >= 0xFFFFF000 {
      sample_count = 0;
      switch_count = 0;
      rprintln!("\n计数器重置\n");
    }
  }
}
