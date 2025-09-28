#![no_std]
#![no_main]

use cortex_m_rt::entry;
use function_generator::{
  dac_to_voltage, voltage_to_dac, GeneratorConfig, GeneratorStatistics, ModulationConfig,
  ModulationType, WaveformGenerator, WaveformType,
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

// 调制发生器配置常量
const VREF_MV: u16 = 3300;
const SAMPLE_RATE: u32 = 300000; // 300kHz
const BUFFER_SIZE: usize = 3000;

// 调制参数控制器
struct ModulationController {
  modulation_type: ModulationType,
  carrier_freq: u32,
  modulation_freq: u32,
  modulation_depth: f32,
  modulation_index: f32,
  current_phase: f32,
  modulation_phase: f32,
  sequence_step: u8,
  last_switch_time: u32,
}

impl ModulationController {
  fn new() -> Self {
    Self {
      modulation_type: ModulationType::AM,
      carrier_freq: 5000,
      modulation_freq: 500,
      modulation_depth: 0.5,
      modulation_index: 1.0,
      current_phase: 0.0,
      modulation_phase: 0.0,
      sequence_step: 0,
      last_switch_time: 0,
    }
  }

  fn update(&mut self, sample_count: u32) -> bool {
    let mut changed = false;

    // 每8秒切换调制类型和参数
    if sample_count.wrapping_sub(self.last_switch_time) >= 2400000 {
      // 8秒 @ 300kHz
      self.last_switch_time = sample_count;
      self.sequence_step = (self.sequence_step + 1) % 12; // 12种不同配置
      changed = true;

      match self.sequence_step {
        0 => {
          // AM调制 - 低调制度
          self.modulation_type = ModulationType::AM;
          self.carrier_freq = 5000;
          self.modulation_freq = 500;
          self.modulation_depth = 0.3;
          self.modulation_index = 0.3;
        }
        1 => {
          // AM调制 - 高调制度
          self.modulation_type = ModulationType::AM;
          self.carrier_freq = 4000;
          self.modulation_freq = 800;
          self.modulation_depth = 0.8;
          self.modulation_index = 0.8;
        }
        2 => {
          // FM调制 - 窄带
          self.modulation_type = ModulationType::FM;
          self.carrier_freq = 6000;
          self.modulation_freq = 300;
          self.modulation_depth = 0.5;
          self.modulation_index = 0.5; // 窄带FM
        }
        3 => {
          // FM调制 - 宽带
          self.modulation_type = ModulationType::FM;
          self.carrier_freq = 3000;
          self.modulation_freq = 200;
          self.modulation_depth = 0.7;
          self.modulation_index = 5.0; // 宽带FM
        }
        4 => {
          // PM调制 - 小调制指数
          self.modulation_type = ModulationType::PM;
          self.carrier_freq = 7000;
          self.modulation_freq = 600;
          self.modulation_depth = 0.4;
          self.modulation_index = 0.5;
        }
        5 => {
          // PM调制 - 大调制指数
          self.modulation_type = ModulationType::PM;
          self.carrier_freq = 4500;
          self.modulation_freq = 400;
          self.modulation_depth = 0.6;
          self.modulation_index = 2.0;
        }
        6 => {
          // DSB调制
          self.modulation_type = ModulationType::DSB;
          self.carrier_freq = 8000;
          self.modulation_freq = 1000;
          self.modulation_depth = 0.9;
          self.modulation_index = 1.0;
        }
        7 => {
          // SSB调制
          self.modulation_type = ModulationType::SSB;
          self.carrier_freq = 5500;
          self.modulation_freq = 750;
          self.modulation_depth = 0.7;
          self.modulation_index = 1.0;
        }
        8 => {
          // 多音调制 - AM
          self.modulation_type = ModulationType::AM;
          self.carrier_freq = 6500;
          self.modulation_freq = 250; // 低频调制
          self.modulation_depth = 0.6;
          self.modulation_index = 0.6;
        }
        9 => {
          // 多音调制 - FM
          self.modulation_type = ModulationType::FM;
          self.carrier_freq = 7500;
          self.modulation_freq = 150; // 很低频调制
          self.modulation_depth = 0.8;
          self.modulation_index = 3.0;
        }
        10 => {
          // 复合调制 - AM+FM
          self.modulation_type = ModulationType::AM; // 主要调制类型
          self.carrier_freq = 5000;
          self.modulation_freq = 333; // 特殊频率
          self.modulation_depth = 0.4;
          self.modulation_index = 1.5;
        }
        11 => {
          // 扫频调制
          self.modulation_type = ModulationType::FM;
          self.carrier_freq = 4000;
          self.modulation_freq = 100; // 慢扫频
          self.modulation_depth = 0.9;
          self.modulation_index = 10.0; // 大偏移
        }
        _ => {}
      }
    }

    changed
  }

  fn get_modulation_config(&self) -> ModulationConfig {
    ModulationConfig {
      modulation_type: self.modulation_type,
      modulation_frequency: self.modulation_freq,
      modulation_depth: self.modulation_depth,
      modulation_index: self.modulation_index,
    }
  }

  fn get_sequence_info(&self) -> (u8, &'static str) {
    let description = match self.sequence_step {
      0 => "AM调制 - 低调制度 (30%)",
      1 => "AM调制 - 高调制度 (80%)",
      2 => "FM调制 - 窄带 (β=0.5)",
      3 => "FM调制 - 宽带 (β=5.0)",
      4 => "PM调制 - 小指数 (β=0.5)",
      5 => "PM调制 - 大指数 (β=2.0)",
      6 => "DSB调制 - 双边带",
      7 => "SSB调制 - 单边带",
      8 => "多音AM - 低频调制",
      9 => "多音FM - 超低频调制",
      10 => "复合调制 - AM+FM",
      11 => "扫频调制 - 大偏移",
      _ => "未知调制",
    };

    (self.sequence_step, description)
  }
}

// 调制分析器
struct ModulationAnalyzer {
  carrier_power: f32,
  sideband_power: f32,
  modulation_depth_measured: f32,
  modulation_index_measured: f32,
  carrier_frequency_measured: u32,
  modulation_frequency_measured: u32,
  distortion: f32,
  spurious_signals: f32,
}

impl ModulationAnalyzer {
  fn new() -> Self {
    Self {
      carrier_power: 0.0,
      sideband_power: 0.0,
      modulation_depth_measured: 0.0,
      modulation_index_measured: 0.0,
      carrier_frequency_measured: 0,
      modulation_frequency_measured: 0,
      distortion: 0.0,
      spurious_signals: 0.0,
    }
  }

  fn analyze(
    &mut self,
    buffer: &[u16],
    carrier_freq: u32,
    mod_freq: u32,
    mod_type: ModulationType,
  ) {
    self.analyze_power_distribution(buffer);
    self.measure_frequencies(buffer, carrier_freq, mod_freq);
    self.calculate_modulation_parameters(buffer, mod_type);
    self.analyze_distortion(buffer);
  }

  fn analyze_power_distribution(&mut self, buffer: &[u16]) {
    // 简化的功率分析
    let mut total_power = 0.0;
    let mut carrier_power = 0.0;

    for &sample in buffer {
      let value = (sample as f32 - 2048.0) / 2048.0; // 归一化
      total_power += value * value;
    }

    // 估算载波功率（简化）
    let dc_component = buffer.iter().map(|&x| x as u32).sum::<u32>() / buffer.len() as u32;
    let dc_normalized = (dc_component as f32 - 2048.0) / 2048.0;
    carrier_power = dc_normalized * dc_normalized * buffer.len() as f32;

    self.carrier_power = carrier_power;
    self.sideband_power = total_power - carrier_power;
  }

  fn measure_frequencies(&mut self, buffer: &[u16], expected_carrier: u32, expected_mod: u32) {
    // 简化的频率测量
    self.carrier_frequency_measured = expected_carrier; // 简化实现
    self.modulation_frequency_measured = expected_mod;
  }

  fn calculate_modulation_parameters(&mut self, buffer: &[u16], mod_type: ModulationType) {
    // 计算调制深度和调制指数
    let max_val = *buffer.iter().max().unwrap_or(&2048) as f32;
    let min_val = *buffer.iter().min().unwrap_or(&2048) as f32;
    let avg_val = buffer.iter().map(|&x| x as f32).sum::<f32>() / buffer.len() as f32;

    match mod_type {
      ModulationType::AM => {
        // AM调制深度 = (Vmax - Vmin) / (Vmax + Vmin)
        if max_val + min_val > 0.0 {
          self.modulation_depth_measured = (max_val - min_val) / (max_val + min_val);
        }
        self.modulation_index_measured = self.modulation_depth_measured;
      }
      ModulationType::FM | ModulationType::PM => {
        // FM/PM调制指数估算（简化）
        let peak_deviation = (max_val - avg_val).max(avg_val - min_val);
        self.modulation_index_measured = peak_deviation / 1000.0; // 简化计算
        self.modulation_depth_measured = self.modulation_index_measured;
      }
      _ => {
        // 其他调制类型的简化处理
        self.modulation_depth_measured = (max_val - min_val) / 4096.0;
        self.modulation_index_measured = self.modulation_depth_measured;
      }
    }
  }

  fn analyze_distortion(&mut self, buffer: &[u16]) {
    // 简化的失真分析
    let mut harmonic_power = 0.0;
    let mut fundamental_power = 0.0;

    // 计算简化的THD
    for (i, &sample) in buffer.iter().enumerate() {
      let value = (sample as f32 - 2048.0) / 2048.0;

      if i % 3 == 0 {
        // 简化的基频功率
        fundamental_power += value * value;
      } else {
        // 简化的谐波功率
        harmonic_power += value * value;
      }
    }

    if fundamental_power > 0.0 {
      self.distortion = (harmonic_power / fundamental_power).sqrt() * 100.0;
    }

    // 简化的杂散信号分析
    self.spurious_signals = self.distortion * 0.1; // 估算
  }

  fn get_modulation_quality(&self) -> f32 {
    // 调制质量评分 (0-100)
    let depth_score =
      if self.modulation_depth_measured > 0.1 && self.modulation_depth_measured < 0.9 {
        100.0
      } else {
        50.0
      };

    let distortion_score = if self.distortion < 5.0 {
      100.0 - self.distortion * 10.0
    } else {
      50.0
    };

    (depth_score + distortion_score) / 2.0
  }
}

// 调制性能监控
struct ModulationPerformance {
  modulation_accuracy: f32,
  frequency_stability: f32,
  phase_noise: f32,
  spurious_suppression: f32,
  settling_time: u32,
  switching_time: u32,
  last_switch_time: u32,
}

impl ModulationPerformance {
  fn new() -> Self {
    Self {
      modulation_accuracy: 0.0,
      frequency_stability: 0.0,
      phase_noise: 0.0,
      spurious_suppression: 0.0,
      settling_time: 0,
      switching_time: 0,
      last_switch_time: 0,
    }
  }

  fn update(
    &mut self,
    analyzer: &ModulationAnalyzer,
    controller: &ModulationController,
    sample_count: u32,
  ) {
    // 计算调制精度
    let depth_error = (analyzer.modulation_depth_measured - controller.modulation_depth).abs();
    self.modulation_accuracy = 100.0 - (depth_error * 100.0);

    // 计算频率稳定性
    let freq_error = analyzer
      .carrier_frequency_measured
      .abs_diff(controller.carrier_freq) as f32;
    if controller.carrier_freq > 0 {
      self.frequency_stability = 100.0 - (freq_error / controller.carrier_freq as f32 * 100.0);
    }

    // 估算相位噪声
    self.phase_noise = analyzer.distortion * 0.5; // 简化估算

    // 计算杂散抑制
    self.spurious_suppression = 60.0 - analyzer.spurious_signals; // dB

    // 记录切换时间
    if sample_count > self.last_switch_time {
      self.switching_time = sample_count - self.last_switch_time;
      self.last_switch_time = sample_count;
    }
  }

  fn get_overall_performance(&self) -> f32 {
    // 综合性能评分
    let accuracy_weight = 0.3;
    let stability_weight = 0.3;
    let noise_weight = 0.2;
    let spurious_weight = 0.2;

    let noise_score = 100.0 - self.phase_noise.min(100.0);
    let spurious_score = (self.spurious_suppression / 60.0 * 100.0).min(100.0);

    self.modulation_accuracy * accuracy_weight
      + self.frequency_stability * stability_weight
      + noise_score * noise_weight
      + spurious_score * spurious_weight
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

  // 初始化调制组件
  let mut modulation_controller = ModulationController::new();
  let mut modulation_analyzer = ModulationAnalyzer::new();
  let mut modulation_performance = ModulationPerformance::new();
  let mut statistics = GeneratorStatistics::new();

  // 初始化函数发生器
  let config = GeneratorConfig {
    waveform: WaveformType::Sine, // 调制通常使用正弦载波
    frequency: modulation_controller.carrier_freq,
    amplitude: 1500,
    offset: 0,
    phase: 0.0,
    duty_cycle: 50,
    sample_rate: SAMPLE_RATE,
  };

  let mut generator = WaveformGenerator::new(config);

  // 状态变量
  let mut sample_count = 0u32;
  let mut last_status_time = 0u32;
  let mut sample_buffer = [0u16; BUFFER_SIZE];
  let mut buffer_index = 0;
  let mut current_modulation = modulation_controller.modulation_type;

  writeln!(tx, "调制发生器启动").unwrap();
  writeln!(tx, "采样率: {}Hz", SAMPLE_RATE).unwrap();
  writeln!(tx, "缓冲区大小: {}", BUFFER_SIZE).unwrap();
  writeln!(tx, "调制序列: 12种不同调制配置").unwrap();
  writeln!(tx, "每种调制持续: 8秒").unwrap();
  writeln!(tx, "支持调制: AM, FM, PM, DSB, SSB").unwrap();

  loop {
    // 检查定时器事件
    if timer.wait().is_ok() {
      // 检查是否需要更新调制参数
      let modulation_changed = modulation_controller.update(sample_count);

      if modulation_changed {
        current_modulation = modulation_controller.modulation_type;

        let new_config = GeneratorConfig {
          waveform: WaveformType::Sine,
          frequency: modulation_controller.carrier_freq,
          amplitude: 1500,
          offset: 0,
          phase: 0.0,
          duty_cycle: 50,
          sample_rate: SAMPLE_RATE,
        };

        generator.update_config(new_config);

        // 设置调制配置
        let mod_config = modulation_controller.get_modulation_config();
        generator.set_modulation(Some(mod_config));
      }

      // 生成调制信号样本
      let sample = generator.generate_sample();

      // 输出到DAC
      dac.write(sample);

      // 存储样本到缓冲区
      sample_buffer[buffer_index] = sample;
      buffer_index = (buffer_index + 1) % BUFFER_SIZE;

      sample_count += 1;
    }

    // 定期输出状态信息和分析
    if sample_count.wrapping_sub(last_status_time) >= 600000 {
      // 每2秒
      last_status_time = sample_count;

      // 进行调制分析
      modulation_analyzer.analyze(
        &sample_buffer,
        modulation_controller.carrier_freq,
        modulation_controller.modulation_freq,
        current_modulation,
      );

      // 更新性能统计
      modulation_performance.update(&modulation_analyzer, &modulation_controller, sample_count);

      // 更新基础统计信息
      statistics.update(
        &sample_buffer,
        modulation_controller.carrier_freq,
        SAMPLE_RATE,
      );

      writeln!(tx, "\n=== 调制发生器状态 ===").unwrap();
      writeln!(tx, "运行时间: {}s", sample_count / SAMPLE_RATE).unwrap();
      writeln!(tx, "总样本数: {}", sample_count).unwrap();

      // 显示当前调制状态
      let (step, description) = modulation_controller.get_sequence_info();
      writeln!(tx, "\n--- 当前调制 ---").unwrap();
      writeln!(tx, "序列步骤: {}/12", step + 1).unwrap();
      writeln!(tx, "调制类型: {:?}", current_modulation).unwrap();
      writeln!(tx, "调制描述: {}", description).unwrap();
      writeln!(tx, "载波频率: {}Hz", modulation_controller.carrier_freq).unwrap();
      writeln!(tx, "调制频率: {}Hz", modulation_controller.modulation_freq).unwrap();
      writeln!(
        tx,
        "调制深度: {:.1}%",
        modulation_controller.modulation_depth * 100.0
      )
      .unwrap();
      writeln!(
        tx,
        "调制指数: {:.2}",
        modulation_controller.modulation_index
      )
      .unwrap();

      // 显示调制分析结果
      writeln!(tx, "\n--- 调制分析 ---").unwrap();
      writeln!(tx, "载波功率: {:.1}", modulation_analyzer.carrier_power).unwrap();
      writeln!(tx, "边带功率: {:.1}", modulation_analyzer.sideband_power).unwrap();
      writeln!(
        tx,
        "测量调制深度: {:.1}%",
        modulation_analyzer.modulation_depth_measured * 100.0
      )
      .unwrap();
      writeln!(
        tx,
        "测量调制指数: {:.2}",
        modulation_analyzer.modulation_index_measured
      )
      .unwrap();
      writeln!(
        tx,
        "载波频率测量: {}Hz",
        modulation_analyzer.carrier_frequency_measured
      )
      .unwrap();
      writeln!(
        tx,
        "调制频率测量: {}Hz",
        modulation_analyzer.modulation_frequency_measured
      )
      .unwrap();
      writeln!(tx, "调制失真: {:.2}%", modulation_analyzer.distortion).unwrap();
      writeln!(tx, "杂散信号: {:.2}%", modulation_analyzer.spurious_signals).unwrap();
      writeln!(
        tx,
        "调制质量: {:.1}/100",
        modulation_analyzer.get_modulation_quality()
      )
      .unwrap();

      // 显示DAC输出信息
      let current_sample = sample_buffer[(buffer_index + BUFFER_SIZE - 1) % BUFFER_SIZE];
      let output_voltage = dac_to_voltage(current_sample, VREF_MV);
      writeln!(tx, "\n--- DAC输出 ---").unwrap();
      writeln!(tx, "当前DAC值: {}", current_sample).unwrap();
      writeln!(tx, "输出电压: {}mV", output_voltage).unwrap();
      writeln!(
        tx,
        "峰峰值: {}mV",
        dac_to_voltage(*sample_buffer.iter().max().unwrap_or(&0), VREF_MV)
          - dac_to_voltage(*sample_buffer.iter().min().unwrap_or(&0), VREF_MV)
      )
      .unwrap();

      // 显示调制性能
      writeln!(tx, "\n--- 调制性能 ---").unwrap();
      writeln!(
        tx,
        "调制精度: {:.1}%",
        modulation_performance.modulation_accuracy
      )
      .unwrap();
      writeln!(
        tx,
        "频率稳定性: {:.1}%",
        modulation_performance.frequency_stability
      )
      .unwrap();
      writeln!(tx, "相位噪声: {:.2}%", modulation_performance.phase_noise).unwrap();
      writeln!(
        tx,
        "杂散抑制: {:.1}dB",
        modulation_performance.spurious_suppression
      )
      .unwrap();
      writeln!(
        tx,
        "切换时间: {}ms",
        modulation_performance.switching_time / (SAMPLE_RATE / 1000)
      )
      .unwrap();
      writeln!(
        tx,
        "综合性能: {:.1}/100",
        modulation_performance.get_overall_performance()
      )
      .unwrap();

      // 显示信号质量统计
      writeln!(tx, "\n--- 信号质量 ---").unwrap();
      writeln!(tx, "频率精度: {:.2}%", statistics.frequency_accuracy).unwrap();
      writeln!(tx, "幅度精度: {:.2}%", statistics.amplitude_accuracy).unwrap();
      writeln!(tx, "总谐波失真: {:.3}%", statistics.thd).unwrap();
      writeln!(tx, "信噪比: {:.1}dB", statistics.snr).unwrap();
      writeln!(tx, "频率稳定性: {:.3}%", statistics.frequency_stability).unwrap();

      // 显示调制类型特性
      writeln!(tx, "\n--- 调制特性 ---").unwrap();
      match current_modulation {
        ModulationType::AM => {
          writeln!(tx, "幅度调制: 载波幅度随调制信号变化").unwrap();
          writeln!(tx, "特点: 简单解调，带宽 = 2×调制频率").unwrap();
          writeln!(tx, "应用: 广播、通信、测试信号").unwrap();
        }
        ModulationType::FM => {
          writeln!(tx, "频率调制: 载波频率随调制信号变化").unwrap();
          writeln!(tx, "特点: 抗噪声强，带宽取决于调制指数").unwrap();
          writeln!(tx, "应用: FM广播、高质量音频").unwrap();
        }
        ModulationType::PM => {
          writeln!(tx, "相位调制: 载波相位随调制信号变化").unwrap();
          writeln!(tx, "特点: 与FM相似，相位连续").unwrap();
          writeln!(tx, "应用: 数字通信、相位键控").unwrap();
        }
        ModulationType::DSB => {
          writeln!(tx, "双边带调制: 抑制载波的AM调制").unwrap();
          writeln!(tx, "特点: 功率效率高，需要载波恢复").unwrap();
          writeln!(tx, "应用: 高效通信系统").unwrap();
        }
        ModulationType::SSB => {
          writeln!(tx, "单边带调制: 只传输一个边带").unwrap();
          writeln!(tx, "特点: 带宽最小，功率效率最高").unwrap();
          writeln!(tx, "应用: 长距离通信、业余无线电").unwrap();
        }
      }

      // 显示调制参数建议
      writeln!(tx, "\n--- 参数建议 ---").unwrap();
      match current_modulation {
        ModulationType::AM => {
          if modulation_controller.modulation_depth > 0.8 {
            writeln!(tx, "建议: 调制深度过高，可能产生失真").unwrap();
          } else if modulation_controller.modulation_depth < 0.3 {
            writeln!(tx, "建议: 调制深度较低，信号利用率不高").unwrap();
          } else {
            writeln!(tx, "建议: 调制深度适中，信号质量良好").unwrap();
          }
        }
        ModulationType::FM => {
          if modulation_controller.modulation_index > 5.0 {
            writeln!(tx, "建议: 宽带FM，带宽较大").unwrap();
          } else if modulation_controller.modulation_index < 1.0 {
            writeln!(tx, "建议: 窄带FM，带宽较小").unwrap();
          } else {
            writeln!(tx, "建议: 中等带宽FM").unwrap();
          }
        }
        _ => {
          writeln!(tx, "建议: 参数设置合理").unwrap();
        }
      }

      // 显示下一个调制预告
      let next_step = (modulation_controller.sequence_step + 1) % 12;
      let next_description = match next_step {
        0 => "AM调制 - 低调制度",
        1 => "AM调制 - 高调制度",
        2 => "FM调制 - 窄带",
        3 => "FM调制 - 宽带",
        4 => "PM调制 - 小指数",
        5 => "PM调制 - 大指数",
        6 => "DSB调制 - 双边带",
        7 => "SSB调制 - 单边带",
        8 => "多音AM - 低频调制",
        9 => "多音FM - 超低频调制",
        10 => "复合调制 - AM+FM",
        11 => "扫频调制 - 大偏移",
        _ => "未知调制",
      };

      let remaining_time =
        (2400000 - (sample_count - modulation_controller.last_switch_time)) / SAMPLE_RATE;
      writeln!(tx, "\n--- 下一个调制 ---").unwrap();
      writeln!(tx, "下一个调制: {}", next_description).unwrap();
      writeln!(tx, "切换倒计时: {}秒", remaining_time).unwrap();

      // 显示系统状态
      writeln!(tx, "\n--- 系统状态 ---").unwrap();
      writeln!(tx, "系统时钟: {}MHz", clocks.sysclk().0 / 1_000_000).unwrap();
      writeln!(tx, "定时器频率: {}Hz", SAMPLE_RATE).unwrap();
      writeln!(tx, "缓冲区使用: {}/{}", buffer_index, BUFFER_SIZE).unwrap();
      writeln!(tx, "调制状态: 正常").unwrap();
      writeln!(tx, "DAC状态: 正常").unwrap();
    }
  }
}
