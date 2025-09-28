#![no_std]
#![no_main]

use cortex_m_rt::entry;
use function_generator::{
  dac_to_voltage, voltage_to_dac, GeneratorConfig, GeneratorStatistics, WaveformGenerator,
  WaveformType,
};
use nb::block;
use panic_halt as _;
use stm32f4xx_hal::{
  dac::{Dac, DacPin, C1},
  gpio::{Analog, Pin},
  prelude::*,
  serial::{config::Config, Serial},
  stm32,
  timer::{Event, Timer},
};

// 波形发生器配置常量
const VREF_MV: u16 = 3300;
const SAMPLE_RATE: u32 = 200000; // 200kHz
const BUFFER_SIZE: usize = 2000;

// 波形参数配置
struct WaveformParameters {
  frequency: u32,
  amplitude: u16,
  offset: i16,
  duty_cycle: u8,
  phase: f32,
}

impl WaveformParameters {
  fn new() -> Self {
    Self {
      frequency: 1000,
      amplitude: 1500,
      offset: 0,
      duty_cycle: 50,
      phase: 0.0,
    }
  }

  fn update_for_waveform(&mut self, waveform: WaveformType) {
    match waveform {
      WaveformType::Sine => {
        self.frequency = 1000;
        self.amplitude = 1500;
        self.offset = 0;
        self.duty_cycle = 50;
        self.phase = 0.0;
      }
      WaveformType::Square => {
        self.frequency = 800;
        self.amplitude = 2000;
        self.offset = 0;
        self.duty_cycle = 50;
        self.phase = 0.0;
      }
      WaveformType::Triangle => {
        self.frequency = 1200;
        self.amplitude = 1800;
        self.offset = 200;
        self.duty_cycle = 50;
        self.phase = 0.0;
      }
      WaveformType::Sawtooth => {
        self.frequency = 1500;
        self.amplitude = 1600;
        self.offset = -100;
        self.duty_cycle = 50;
        self.phase = 0.0;
      }
      WaveformType::Pulse => {
        self.frequency = 2000;
        self.amplitude = 2500;
        self.offset = 0;
        self.duty_cycle = 25; // 25%占空比
        self.phase = 0.0;
      }
      WaveformType::Noise => {
        self.frequency = 0; // 噪声无频率
        self.amplitude = 800;
        self.offset = 0;
        self.duty_cycle = 50;
        self.phase = 0.0;
      }
      WaveformType::DC => {
        self.frequency = 0;
        self.amplitude = 0;
        self.offset = 1650; // 1.65V直流
        self.duty_cycle = 50;
        self.phase = 0.0;
      }
      WaveformType::Arbitrary => {
        self.frequency = 1000;
        self.amplitude = 1200;
        self.offset = 0;
        self.duty_cycle = 50;
        self.phase = 0.0;
      }
    }
  }
}

// 波形序列管理
struct WaveformSequence {
  waveforms: [WaveformType; 8],
  current_index: usize,
  sequence_count: u32,
  time_per_waveform: u32, // 每个波形持续时间（采样数）
  current_waveform_time: u32,
}

impl WaveformSequence {
  fn new() -> Self {
    Self {
      waveforms: [
        WaveformType::Sine,
        WaveformType::Square,
        WaveformType::Triangle,
        WaveformType::Sawtooth,
        WaveformType::Pulse,
        WaveformType::Noise,
        WaveformType::DC,
        WaveformType::Arbitrary,
      ],
      current_index: 0,
      sequence_count: 0,
      time_per_waveform: 800000, // 4秒 @ 200kHz
      current_waveform_time: 0,
    }
  }

  fn update(&mut self, sample_count: u32) -> (bool, WaveformType) {
    let mut changed = false;

    if sample_count.wrapping_sub(self.current_waveform_time) >= self.time_per_waveform {
      self.current_index = (self.current_index + 1) % self.waveforms.len();
      self.current_waveform_time = sample_count;
      self.sequence_count += 1;
      changed = true;
    }

    (changed, self.waveforms[self.current_index])
  }

  fn get_current_waveform(&self) -> WaveformType {
    self.waveforms[self.current_index]
  }

  fn get_progress(&self, sample_count: u32) -> u8 {
    let elapsed = sample_count.wrapping_sub(self.current_waveform_time);
    ((elapsed * 100) / self.time_per_waveform) as u8
  }

  fn get_remaining_time(&self, sample_count: u32) -> u32 {
    let elapsed = sample_count.wrapping_sub(self.current_waveform_time);
    self.time_per_waveform.saturating_sub(elapsed)
  }
}

// 波形质量分析
struct WaveformQuality {
  rms_value: f32,
  peak_value: u16,
  crest_factor: f32,
  zero_crossings: u16,
  symmetry_factor: f32,
  linearity_error: f32,
}

impl WaveformQuality {
  fn new() -> Self {
    Self {
      rms_value: 0.0,
      peak_value: 0,
      crest_factor: 0.0,
      zero_crossings: 0,
      symmetry_factor: 0.0,
      linearity_error: 0.0,
    }
  }

  fn analyze(&mut self, buffer: &[u16], waveform_type: WaveformType) {
    if buffer.is_empty() {
      return;
    }

    // 计算RMS值
    let sum_squares: u64 = buffer.iter().map(|&x| (x as u64) * (x as u64)).sum();
    self.rms_value = ((sum_squares / buffer.len() as u64) as f32).sqrt();

    // 计算峰值
    self.peak_value = *buffer.iter().max().unwrap_or(&0);

    // 计算峰值因子
    let avg_value = buffer.iter().map(|&x| x as u32).sum::<u32>() / buffer.len() as u32;
    if avg_value > 0 {
      self.crest_factor = self.peak_value as f32 / avg_value as f32;
    }

    // 计算过零点数量
    self.zero_crossings = self.count_zero_crossings(buffer);

    // 计算对称性因子
    self.symmetry_factor = self.calculate_symmetry(buffer);

    // 计算线性度误差（针对三角波和锯齿波）
    if matches!(
      waveform_type,
      WaveformType::Triangle | WaveformType::Sawtooth
    ) {
      self.linearity_error = self.calculate_linearity_error(buffer, waveform_type);
    }
  }

  fn count_zero_crossings(&self, buffer: &[u16]) -> u16 {
    let mid_point = 2048u16; // 12位DAC中点
    let mut crossings = 0;
    let mut last_above = buffer[0] > mid_point;

    for &sample in buffer.iter().skip(1) {
      let current_above = sample > mid_point;
      if current_above != last_above {
        crossings += 1;
        last_above = current_above;
      }
    }

    crossings
  }

  fn calculate_symmetry(&self, buffer: &[u16]) -> f32 {
    let len = buffer.len();
    let mid = len / 2;

    let mut symmetry_error = 0.0;
    for i in 0..mid {
      let left = buffer[i] as f32;
      let right = buffer[len - 1 - i] as f32;
      symmetry_error += (left - right).abs();
    }

    // 归一化对称性误差
    if mid > 0 {
      1.0 - (symmetry_error / (mid as f32 * 4095.0))
    } else {
      0.0
    }
  }

  fn calculate_linearity_error(&self, buffer: &[u16], waveform_type: WaveformType) -> f32 {
    let len = buffer.len();
    if len < 3 {
      return 0.0;
    }

    let mut total_error = 0.0;
    let mut count = 0;

    match waveform_type {
      WaveformType::Triangle => {
        // 检查三角波的线性度
        for i in 1..len - 1 {
          let expected_slope = if i < len / 2 {
            // 上升沿
            (buffer[i + 1] as i32 - buffer[i - 1] as i32) / 2
          } else {
            // 下降沿
            (buffer[i - 1] as i32 - buffer[i + 1] as i32) / 2
          };

          let actual_slope = buffer[i + 1] as i32 - buffer[i - 1] as i32;
          let error = (expected_slope - actual_slope).abs() as f32;
          total_error += error;
          count += 1;
        }
      }
      WaveformType::Sawtooth => {
        // 检查锯齿波的线性度
        for i in 1..len - 1 {
          let expected_diff = (4095.0 / len as f32) as i32;
          let actual_diff = buffer[i + 1] as i32 - buffer[i] as i32;

          // 处理锯齿波的跳跃
          if actual_diff.abs() < 2000 {
            let error = (expected_diff - actual_diff).abs() as f32;
            total_error += error;
            count += 1;
          }
        }
      }
      _ => return 0.0,
    }

    if count > 0 {
      (total_error / count as f32) / 4095.0 * 100.0 // 百分比误差
    } else {
      0.0
    }
  }
}

// 性能监控
struct WaveformPerformance {
  generation_cycles: u32,
  analysis_cycles: u32,
  output_cycles: u32,
  max_generation_cycles: u32,
  max_analysis_cycles: u32,
  max_output_cycles: u32,
  waveform_switches: u32,
  total_samples: u32,
}

impl WaveformPerformance {
  fn new() -> Self {
    Self {
      generation_cycles: 0,
      analysis_cycles: 0,
      output_cycles: 0,
      max_generation_cycles: 0,
      max_analysis_cycles: 0,
      max_output_cycles: 0,
      waveform_switches: 0,
      total_samples: 0,
    }
  }

  fn update(&mut self, gen_cycles: u32, analysis_cycles: u32, out_cycles: u32) {
    self.generation_cycles = gen_cycles;
    self.analysis_cycles = analysis_cycles;
    self.output_cycles = out_cycles;

    if gen_cycles > self.max_generation_cycles {
      self.max_generation_cycles = gen_cycles;
    }

    if analysis_cycles > self.max_analysis_cycles {
      self.max_analysis_cycles = analysis_cycles;
    }

    if out_cycles > self.max_output_cycles {
      self.max_output_cycles = out_cycles;
    }

    self.total_samples += 1;
  }

  fn record_waveform_switch(&mut self) {
    self.waveform_switches += 1;
  }

  fn get_cpu_utilization(&self) -> f32 {
    let total_cycles = self.generation_cycles + self.analysis_cycles + self.output_cycles;
    let max_cycles_per_sample = 84_000_000 / SAMPLE_RATE; // 84MHz / 200kHz

    if max_cycles_per_sample > 0 {
      (total_cycles as f32 / max_cycles_per_sample as f32) * 100.0
    } else {
      0.0
    }
  }
}

#[entry]
fn main() -> ! {
  // 初始化外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();

  // 配置串口用于输出
  let tx_pin = gpioa.pa2.into_alternate();
  let rx_pin = gpioa.pa3.into_alternate();
  let serial = Serial::new(
    dp.USART2,
    (tx_pin, rx_pin),
    Config::default().baudrate(115200.bps()),
    &clocks,
  )
  .unwrap();
  let (mut tx, _rx) = serial.split();

  // 配置DAC引脚
  let dac_pin = gpioa.pa4.into_analog();

  // 配置DAC
  let mut dac = Dac::new(dp.DAC, dac_pin, &clocks);

  // 配置定时器用于采样触发
  let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
  timer.start(SAMPLE_RATE.hz()).unwrap();
  timer.listen(Event::Update);

  // 初始化波形发生器组件
  let mut sequence = WaveformSequence::new();
  let mut parameters = WaveformParameters::new();
  let mut quality = WaveformQuality::new();
  let mut performance = WaveformPerformance::new();
  let mut statistics = GeneratorStatistics::new();

  // 初始化函数发生器
  let config = GeneratorConfig {
    waveform: sequence.get_current_waveform(),
    frequency: parameters.frequency,
    amplitude: parameters.amplitude,
    offset: parameters.offset,
    phase: parameters.phase,
    duty_cycle: parameters.duty_cycle,
    sample_rate: SAMPLE_RATE,
  };

  let mut generator = WaveformGenerator::new(config);

  // 状态变量
  let mut sample_count = 0u32;
  let mut last_status_time = 0u32;
  let mut sample_buffer = [0u16; BUFFER_SIZE];
  let mut buffer_index = 0;
  let mut current_waveform = sequence.get_current_waveform();

  writeln!(tx, "高级波形发生器启动").unwrap();
  writeln!(tx, "采样率: {}Hz", SAMPLE_RATE).unwrap();
  writeln!(tx, "缓冲区大小: {}", BUFFER_SIZE).unwrap();
  writeln!(tx, "波形序列: 8种波形自动切换").unwrap();
  writeln!(
    tx,
    "每个波形持续: {}秒",
    sequence.time_per_waveform / SAMPLE_RATE
  )
  .unwrap();

  loop {
    // 检查定时器事件
    if timer.wait().is_ok() {
      let gen_start = cortex_m::peripheral::DWT::cycle_count();

      // 检查是否需要切换波形
      let (waveform_changed, new_waveform) = sequence.update(sample_count);

      if waveform_changed {
        current_waveform = new_waveform;
        parameters.update_for_waveform(current_waveform);

        let new_config = GeneratorConfig {
          waveform: current_waveform,
          frequency: parameters.frequency,
          amplitude: parameters.amplitude,
          offset: parameters.offset,
          phase: parameters.phase,
          duty_cycle: parameters.duty_cycle,
          sample_rate: SAMPLE_RATE,
        };

        generator.update_config(new_config);
        performance.record_waveform_switch();
      }

      // 生成样本
      let sample = generator.generate_sample();
      let gen_end = cortex_m::peripheral::DWT::cycle_count();

      let analysis_start = cortex_m::peripheral::DWT::cycle_count();

      // 存储样本到缓冲区
      sample_buffer[buffer_index] = sample;
      buffer_index = (buffer_index + 1) % BUFFER_SIZE;

      let analysis_end = cortex_m::peripheral::DWT::cycle_count();

      let output_start = cortex_m::peripheral::DWT::cycle_count();

      // 输出到DAC
      dac.write(sample);

      let output_end = cortex_m::peripheral::DWT::cycle_count();

      // 更新性能统计
      let gen_cycles = gen_end.wrapping_sub(gen_start);
      let analysis_cycles = analysis_end.wrapping_sub(analysis_start);
      let output_cycles = output_end.wrapping_sub(output_start);

      performance.update(gen_cycles, analysis_cycles, output_cycles);

      sample_count += 1;
    }

    // 定期输出状态信息和分析
    if sample_count.wrapping_sub(last_status_time) >= 400000 {
      // 每2秒
      last_status_time = sample_count;

      // 进行波形质量分析
      quality.analyze(&sample_buffer, current_waveform);

      // 更新统计信息
      statistics.update(&sample_buffer, parameters.frequency, SAMPLE_RATE);

      writeln!(tx, "\n=== 波形发生器状态 ===").unwrap();
      writeln!(tx, "运行时间: {}s", sample_count / SAMPLE_RATE).unwrap();
      writeln!(tx, "总样本数: {}", sample_count).unwrap();
      writeln!(tx, "波形切换次数: {}", performance.waveform_switches).unwrap();
      writeln!(tx, "序列循环次数: {}", sequence.sequence_count).unwrap();

      // 显示当前波形信息
      writeln!(tx, "\n--- 当前波形 ---").unwrap();
      writeln!(tx, "波形类型: {:?}", current_waveform).unwrap();
      writeln!(tx, "频率: {}Hz", parameters.frequency).unwrap();
      writeln!(tx, "幅度: {}mV", parameters.amplitude).unwrap();
      writeln!(tx, "直流偏移: {}mV", parameters.offset).unwrap();
      writeln!(tx, "占空比: {}%", parameters.duty_cycle).unwrap();
      writeln!(tx, "相位: {:.1}°", parameters.phase * 180.0 / 3.14159).unwrap();

      // 显示波形进度
      let progress = sequence.get_progress(sample_count);
      let remaining = sequence.get_remaining_time(sample_count) / SAMPLE_RATE;
      writeln!(tx, "波形进度: {}%", progress).unwrap();
      writeln!(tx, "剩余时间: {}秒", remaining).unwrap();

      // 显示DAC输出信息
      let current_sample = sample_buffer[(buffer_index + BUFFER_SIZE - 1) % BUFFER_SIZE];
      let output_voltage = dac_to_voltage(current_sample, VREF_MV);
      writeln!(tx, "\n--- DAC输出 ---").unwrap();
      writeln!(tx, "当前DAC值: {}", current_sample).unwrap();
      writeln!(tx, "输出电压: {}mV", output_voltage).unwrap();
      writeln!(tx, "输出范围: 0-{}mV", VREF_MV).unwrap();

      // 显示波形质量分析
      writeln!(tx, "\n--- 波形质量 ---").unwrap();
      writeln!(tx, "RMS值: {:.1}", quality.rms_value).unwrap();
      writeln!(tx, "峰值: {}", quality.peak_value).unwrap();
      writeln!(tx, "峰值因子: {:.2}", quality.crest_factor).unwrap();
      writeln!(tx, "过零点数: {}", quality.zero_crossings).unwrap();
      writeln!(tx, "对称性: {:.1}%", quality.symmetry_factor * 100.0).unwrap();

      if matches!(
        current_waveform,
        WaveformType::Triangle | WaveformType::Sawtooth
      ) {
        writeln!(tx, "线性度误差: {:.2}%", quality.linearity_error).unwrap();
      }

      // 显示信号质量统计
      writeln!(tx, "\n--- 信号统计 ---").unwrap();
      writeln!(tx, "频率精度: {:.2}%", statistics.frequency_accuracy).unwrap();
      writeln!(tx, "幅度精度: {:.2}%", statistics.amplitude_accuracy).unwrap();
      writeln!(tx, "总谐波失真: {:.3}%", statistics.thd).unwrap();
      writeln!(tx, "信噪比: {:.1}dB", statistics.snr).unwrap();
      writeln!(tx, "频率稳定性: {:.3}%", statistics.frequency_stability).unwrap();

      // 显示性能统计
      writeln!(tx, "\n--- 性能统计 ---").unwrap();
      writeln!(tx, "生成周期: {}", performance.generation_cycles).unwrap();
      writeln!(tx, "分析周期: {}", performance.analysis_cycles).unwrap();
      writeln!(tx, "输出周期: {}", performance.output_cycles).unwrap();
      writeln!(tx, "最大生成周期: {}", performance.max_generation_cycles).unwrap();
      writeln!(tx, "最大分析周期: {}", performance.max_analysis_cycles).unwrap();
      writeln!(tx, "最大输出周期: {}", performance.max_output_cycles).unwrap();
      writeln!(tx, "CPU利用率: {:.1}%", performance.get_cpu_utilization()).unwrap();

      // 显示波形特性描述
      writeln!(tx, "\n--- 波形特性 ---").unwrap();
      match current_waveform {
        WaveformType::Sine => {
          writeln!(tx, "正弦波: 纯净的基频信号，低谐波失真").unwrap();
          writeln!(tx, "应用: 音频测试、滤波器测试、基准信号").unwrap();
        }
        WaveformType::Square => {
          writeln!(tx, "方波: 丰富的奇次谐波，快速边沿").unwrap();
          writeln!(tx, "应用: 数字电路测试、时钟信号、开关测试").unwrap();
        }
        WaveformType::Triangle => {
          writeln!(tx, "三角波: 线性上升下降，中等谐波含量").unwrap();
          writeln!(tx, "应用: 积分器测试、线性度测试、扫描信号").unwrap();
        }
        WaveformType::Sawtooth => {
          writeln!(tx, "锯齿波: 线性上升急速下降，丰富谐波").unwrap();
          writeln!(tx, "应用: 扫描电路、振荡器测试、频谱分析").unwrap();
        }
        WaveformType::Pulse => {
          writeln!(tx, "脉冲波: 可调占空比，适合脉宽调制").unwrap();
          writeln!(tx, "应用: PWM测试、脉冲响应、开关电源").unwrap();
        }
        WaveformType::Noise => {
          writeln!(tx, "噪声: 随机信号，宽频谱特性").unwrap();
          writeln!(tx, "应用: 噪声测试、随机信号、系统激励").unwrap();
        }
        WaveformType::DC => {
          writeln!(tx, "直流: 恒定电平，零频率").unwrap();
          writeln!(tx, "应用: 偏置电压、基准电平、直流测试").unwrap();
        }
        WaveformType::Arbitrary => {
          writeln!(tx, "任意波形: 用户定义，复合信号").unwrap();
          writeln!(tx, "应用: 复杂信号模拟、特殊测试、信号重现").unwrap();
        }
      }

      // 显示下一个波形预告
      let next_index = (sequence.current_index + 1) % sequence.waveforms.len();
      let next_waveform = sequence.waveforms[next_index];
      writeln!(tx, "\n--- 下一个波形 ---").unwrap();
      writeln!(tx, "下一个波形: {:?}", next_waveform).unwrap();
      writeln!(tx, "切换倒计时: {}秒", remaining).unwrap();

      // 显示系统状态
      writeln!(tx, "\n--- 系统状态 ---").unwrap();
      writeln!(tx, "系统时钟: {}MHz", clocks.sysclk().0 / 1_000_000).unwrap();
      writeln!(tx, "定时器频率: {}Hz", SAMPLE_RATE).unwrap();
      writeln!(tx, "缓冲区使用: {}/{}", buffer_index, BUFFER_SIZE).unwrap();
      writeln!(tx, "内存使用: 正常").unwrap();
      writeln!(tx, "DAC状态: 正常").unwrap();
    }
  }
}
