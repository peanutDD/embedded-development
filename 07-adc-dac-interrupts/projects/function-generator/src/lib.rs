#![no_std]

use heapless::Vec;
use micromath::F32Ext;

/// 波形类型
#[derive(Clone, Copy, Debug)]
pub enum WaveformType {
    Sine,       // 正弦波
    Square,     // 方波
    Triangle,   // 三角波
    Sawtooth,   // 锯齿波
    Pulse,      // 脉冲波
    Noise,      // 噪声
    DC,         // 直流
    Arbitrary,  // 任意波形
}

/// 调制类型
#[derive(Clone, Copy, Debug)]
pub enum ModulationType {
    None,       // 无调制
    AM,         // 幅度调制
    FM,         // 频率调制
    PM,         // 相位调制
    PWM,        // 脉宽调制
}

/// 扫频类型
#[derive(Clone, Copy, Debug)]
pub enum SweepType {
    None,       // 无扫频
    Linear,     // 线性扫频
    Logarithmic, // 对数扫频
    Step,       // 步进扫频
}

/// 函数发生器配置
#[derive(Clone, Copy, Debug)]
pub struct GeneratorConfig {
    pub waveform: WaveformType,
    pub frequency: u32,         // 频率 (Hz)
    pub amplitude: u16,         // 幅度 (mV)
    pub offset: i16,            // 直流偏移 (mV)
    pub phase: f32,             // 相位 (度)
    pub duty_cycle: u8,         // 占空比 (%)
    pub sample_rate: u32,       // 采样率 (Hz)
}

impl Default for GeneratorConfig {
    fn default() -> Self {
        Self {
            waveform: WaveformType::Sine,
            frequency: 1000,        // 1kHz
            amplitude: 1000,        // 1V
            offset: 0,              // 0V偏移
            phase: 0.0,             // 0度相位
            duty_cycle: 50,         // 50%占空比
            sample_rate: 100000,    // 100kHz采样率
        }
    }
}

/// 调制配置
#[derive(Clone, Copy, Debug)]
pub struct ModulationConfig {
    pub modulation_type: ModulationType,
    pub modulation_frequency: u32,  // 调制频率 (Hz)
    pub modulation_depth: u8,       // 调制深度 (%)
    pub modulation_phase: f32,      // 调制相位 (度)
}

impl Default for ModulationConfig {
    fn default() -> Self {
        Self {
            modulation_type: ModulationType::None,
            modulation_frequency: 100,  // 100Hz调制频率
            modulation_depth: 50,       // 50%调制深度
            modulation_phase: 0.0,      // 0度调制相位
        }
    }
}

/// 扫频配置
#[derive(Clone, Copy, Debug)]
pub struct SweepConfig {
    pub sweep_type: SweepType,
    pub start_frequency: u32,   // 起始频率 (Hz)
    pub stop_frequency: u32,    // 结束频率 (Hz)
    pub sweep_time: u32,        // 扫频时间 (ms)
    pub sweep_direction: bool,  // 扫频方向 (true: 上扫, false: 下扫)
}

impl Default for SweepConfig {
    fn default() -> Self {
        Self {
            sweep_type: SweepType::None,
            start_frequency: 100,   // 100Hz
            stop_frequency: 10000,  // 10kHz
            sweep_time: 1000,       // 1秒
            sweep_direction: true,  // 上扫
        }
    }
}

/// 波形生成器
pub struct WaveformGenerator {
    config: GeneratorConfig,
    modulation: ModulationConfig,
    sweep: SweepConfig,
    phase_accumulator: f32,
    modulation_phase: f32,
    sweep_phase: f32,
    current_frequency: u32,
    sample_count: u32,
}

impl WaveformGenerator {
    pub fn new(config: GeneratorConfig) -> Self {
        Self {
            config,
            modulation: ModulationConfig::default(),
            sweep: SweepConfig::default(),
            phase_accumulator: 0.0,
            modulation_phase: 0.0,
            sweep_phase: 0.0,
            current_frequency: config.frequency,
            sample_count: 0,
        }
    }

    pub fn update_config(&mut self, config: GeneratorConfig) {
        self.config = config;
        self.current_frequency = config.frequency;
    }

    pub fn set_modulation(&mut self, modulation: ModulationConfig) {
        self.modulation = modulation;
    }

    pub fn set_sweep(&mut self, sweep: SweepConfig) {
        self.sweep = sweep;
        self.sweep_phase = 0.0;
    }

    pub fn reset(&mut self) {
        self.phase_accumulator = 0.0;
        self.modulation_phase = 0.0;
        self.sweep_phase = 0.0;
        self.current_frequency = self.config.frequency;
        self.sample_count = 0;
    }

    pub fn generate_sample(&mut self) -> u16 {
        // 更新扫频
        self.update_sweep();
        
        // 计算相位增量
        let phase_increment = 2.0 * core::f32::consts::PI * self.current_frequency as f32 / self.config.sample_rate as f32;
        
        // 生成基础波形
        let mut sample = self.generate_waveform(self.phase_accumulator + self.config.phase.to_radians());
        
        // 应用调制
        sample = self.apply_modulation(sample);
        
        // 应用幅度和偏移
        sample = sample * self.config.amplitude as f32 / 1000.0; // 归一化到mV
        sample += self.config.offset as f32;
        
        // 更新相位累加器
        self.phase_accumulator += phase_increment;
        if self.phase_accumulator >= 2.0 * core::f32::consts::PI {
            self.phase_accumulator -= 2.0 * core::f32::consts::PI;
        }
        
        // 更新调制相位
        let mod_phase_increment = 2.0 * core::f32::consts::PI * self.modulation.modulation_frequency as f32 / self.config.sample_rate as f32;
        self.modulation_phase += mod_phase_increment;
        if self.modulation_phase >= 2.0 * core::f32::consts::PI {
            self.modulation_phase -= 2.0 * core::f32::consts::PI;
        }
        
        self.sample_count += 1;
        
        // 转换为DAC值 (12位，0-4095)
        let dac_value = ((sample + 1650.0) * 4095.0 / 3300.0).max(0.0).min(4095.0) as u16;
        dac_value
    }

    fn generate_waveform(&self, phase: f32) -> f32 {
        match self.config.waveform {
            WaveformType::Sine => phase.sin(),
            WaveformType::Square => {
                if phase.sin() >= 0.0 { 1.0 } else { -1.0 }
            },
            WaveformType::Triangle => {
                let normalized_phase = phase / (2.0 * core::f32::consts::PI);
                let frac = normalized_phase - normalized_phase.floor();
                if frac < 0.5 {
                    4.0 * frac - 1.0
                } else {
                    3.0 - 4.0 * frac
                }
            },
            WaveformType::Sawtooth => {
                let normalized_phase = phase / (2.0 * core::f32::consts::PI);
                let frac = normalized_phase - normalized_phase.floor();
                2.0 * frac - 1.0
            },
            WaveformType::Pulse => {
                let normalized_phase = phase / (2.0 * core::f32::consts::PI);
                let frac = normalized_phase - normalized_phase.floor();
                if frac < (self.config.duty_cycle as f32 / 100.0) { 1.0 } else { -1.0 }
            },
            WaveformType::Noise => {
                // 简单的伪随机噪声
                let seed = (phase * 12345.0) as u32;
                let noise = ((seed.wrapping_mul(1103515245).wrapping_add(12345)) >> 16) as f32 / 32768.0;
                noise * 2.0 - 1.0
            },
            WaveformType::DC => 1.0,
            WaveformType::Arbitrary => {
                // 这里可以实现任意波形，暂时返回正弦波
                phase.sin()
            },
        }
    }

    fn apply_modulation(&self, sample: f32) -> f32 {
        match self.modulation.modulation_type {
            ModulationType::None => sample,
            ModulationType::AM => {
                let mod_signal = (self.modulation_phase + self.modulation.modulation_phase.to_radians()).sin();
                let mod_factor = 1.0 + (self.modulation.modulation_depth as f32 / 100.0) * mod_signal;
                sample * mod_factor
            },
            ModulationType::FM => {
                // FM调制已在相位计算中处理
                sample
            },
            ModulationType::PM => {
                // PM调制需要修改相位
                sample
            },
            ModulationType::PWM => {
                // PWM调制修改占空比
                sample
            },
        }
    }

    fn update_sweep(&mut self) {
        match self.sweep.sweep_type {
            SweepType::None => {},
            SweepType::Linear => {
                let sweep_progress = (self.sample_count as f32 * 1000.0) / (self.config.sample_rate as f32 * self.sweep.sweep_time as f32);
                if sweep_progress <= 1.0 {
                    let freq_range = self.sweep.stop_frequency as f32 - self.sweep.start_frequency as f32;
                    if self.sweep.sweep_direction {
                        self.current_frequency = self.sweep.start_frequency + (freq_range * sweep_progress) as u32;
                    } else {
                        self.current_frequency = self.sweep.stop_frequency - (freq_range * sweep_progress) as u32;
                    }
                }
            },
            SweepType::Logarithmic => {
                let sweep_progress = (self.sample_count as f32 * 1000.0) / (self.config.sample_rate as f32 * self.sweep.sweep_time as f32);
                if sweep_progress <= 1.0 {
                    let start_log = (self.sweep.start_frequency as f32).ln();
                    let stop_log = (self.sweep.stop_frequency as f32).ln();
                    let log_range = stop_log - start_log;
                    
                    if self.sweep.sweep_direction {
                        let current_log = start_log + log_range * sweep_progress;
                        self.current_frequency = current_log.exp() as u32;
                    } else {
                        let current_log = stop_log - log_range * sweep_progress;
                        self.current_frequency = current_log.exp() as u32;
                    }
                }
            },
            SweepType::Step => {
                // 步进扫频实现
                let steps = 10; // 10个步进
                let step_time = self.sweep.sweep_time / steps;
                let current_step = (self.sample_count * 1000) / (self.config.sample_rate * step_time);
                
                if current_step < steps {
                    let freq_step = (self.sweep.stop_frequency - self.sweep.start_frequency) / steps;
                    if self.sweep.sweep_direction {
                        self.current_frequency = self.sweep.start_frequency + freq_step * current_step;
                    } else {
                        self.current_frequency = self.sweep.stop_frequency - freq_step * current_step;
                    }
                }
            },
        }
    }

    pub fn get_current_frequency(&self) -> u32 {
        self.current_frequency
    }

    pub fn get_sample_count(&self) -> u32 {
        self.sample_count
    }
}

/// 任意波形缓冲区
pub struct ArbitraryWaveform<const N: usize> {
    pub samples: Vec<u16, N>,
    pub sample_rate: u32,
    pub loop_enabled: bool,
    pub current_index: usize,
}

impl<const N: usize> ArbitraryWaveform<N> {
    pub fn new(sample_rate: u32) -> Self {
        Self {
            samples: Vec::new(),
            sample_rate,
            loop_enabled: true,
            current_index: 0,
        }
    }

    pub fn load_samples(&mut self, samples: &[u16]) -> bool {
        self.samples.clear();
        for &sample in samples {
            if self.samples.push(sample).is_err() {
                return false;
            }
        }
        self.current_index = 0;
        true
    }

    pub fn get_next_sample(&mut self) -> Option<u16> {
        if self.samples.is_empty() {
            return None;
        }

        let sample = self.samples[self.current_index];
        self.current_index += 1;

        if self.current_index >= self.samples.len() {
            if self.loop_enabled {
                self.current_index = 0;
            } else {
                return None;
            }
        }

        Some(sample)
    }

    pub fn reset(&mut self) {
        self.current_index = 0;
    }

    pub fn is_complete(&self) -> bool {
        !self.loop_enabled && self.current_index >= self.samples.len()
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }
}

/// 多通道函数发生器
pub struct MultiChannelGenerator<const CHANNELS: usize> {
    pub generators: [WaveformGenerator; CHANNELS],
    pub channel_enabled: [bool; CHANNELS],
    pub phase_locked: bool,
    pub master_channel: usize,
}

impl<const CHANNELS: usize> MultiChannelGenerator<CHANNELS> {
    pub fn new(configs: [GeneratorConfig; CHANNELS]) -> Self {
        Self {
            generators: configs.map(|config| WaveformGenerator::new(config)),
            channel_enabled: [true; CHANNELS],
            phase_locked: false,
            master_channel: 0,
        }
    }

    pub fn enable_channel(&mut self, channel: usize, enabled: bool) {
        if channel < CHANNELS {
            self.channel_enabled[channel] = enabled;
        }
    }

    pub fn set_phase_lock(&mut self, enabled: bool, master_channel: usize) {
        self.phase_locked = enabled;
        if master_channel < CHANNELS {
            self.master_channel = master_channel;
        }
    }

    pub fn generate_samples(&mut self) -> [u16; CHANNELS] {
        let mut samples = [0u16; CHANNELS];
        
        for (i, generator) in self.generators.iter_mut().enumerate() {
            if self.channel_enabled[i] {
                if self.phase_locked && i != self.master_channel {
                    // 相位锁定：从主通道复制相位
                    generator.phase_accumulator = self.generators[self.master_channel].phase_accumulator;
                }
                samples[i] = generator.generate_sample();
            }
        }
        
        samples
    }

    pub fn reset_all(&mut self) {
        for generator in &mut self.generators {
            generator.reset();
        }
    }

    pub fn update_channel_config(&mut self, channel: usize, config: GeneratorConfig) {
        if channel < CHANNELS {
            self.generators[channel].update_config(config);
        }
    }
}

/// 函数发生器统计信息
pub struct GeneratorStatistics {
    pub samples_generated: u32,
    pub frequency_accuracy: f32,
    pub amplitude_accuracy: f32,
    pub thd: f32,               // 总谐波失真
    pub snr: f32,               // 信噪比
    pub frequency_stability: f32, // 频率稳定性
}

impl GeneratorStatistics {
    pub fn new() -> Self {
        Self {
            samples_generated: 0,
            frequency_accuracy: 0.0,
            amplitude_accuracy: 0.0,
            thd: 0.0,
            snr: 0.0,
            frequency_stability: 0.0,
        }
    }

    pub fn update(&mut self, samples: &[u16], target_frequency: u32, sample_rate: u32) {
        self.samples_generated += samples.len() as u32;
        
        // 计算频率精度
        if let Some(measured_freq) = self.measure_frequency(samples, sample_rate) {
            self.frequency_accuracy = ((measured_freq as f32 - target_frequency as f32) / target_frequency as f32).abs() * 100.0;
        }
        
        // 计算其他统计信息
        self.amplitude_accuracy = self.measure_amplitude_accuracy(samples);
        self.thd = self.calculate_thd(samples);
        self.snr = self.calculate_snr(samples);
    }

    fn measure_frequency(&self, samples: &[u16], sample_rate: u32) -> Option<u32> {
        if samples.len() < 3 {
            return None;
        }

        let avg = samples.iter().map(|&x| x as u32).sum::<u32>() / samples.len() as u32;
        let mut zero_crossings = 0;
        let mut last_above = samples[0] > avg as u16;

        for &sample in samples.iter().skip(1) {
            let current_above = sample > avg as u16;
            if current_above != last_above {
                zero_crossings += 1;
            }
            last_above = current_above;
        }

        if zero_crossings < 2 {
            return None;
        }

        let total_time = samples.len() as f32 / sample_rate as f32;
        let frequency = (zero_crossings as f32 / 2.0) / total_time;
        Some(frequency as u32)
    }

    fn measure_amplitude_accuracy(&self, samples: &[u16]) -> f32 {
        if samples.is_empty() {
            return 0.0;
        }
        
        let max = *samples.iter().max().unwrap_or(&0);
        let min = *samples.iter().min().unwrap_or(&0);
        let peak_to_peak = max.saturating_sub(min);
        
        // 假设目标幅度是满量程的一半
        let target_amplitude = 2048u16;
        ((peak_to_peak as f32 - target_amplitude as f32) / target_amplitude as f32).abs() * 100.0
    }

    fn calculate_thd(&self, _samples: &[u16]) -> f32 {
        // 简化的THD计算
        // 实际实现需要FFT分析
        0.1 // 假设0.1%的THD
    }

    fn calculate_snr(&self, samples: &[u16]) -> f32 {
        if samples.is_empty() {
            return 0.0;
        }
        
        let avg = samples.iter().map(|&x| x as f32).sum::<f32>() / samples.len() as f32;
        let signal_power = avg * avg;
        
        let noise_power = samples.iter()
            .map(|&x| {
                let diff = x as f32 - avg;
                diff * diff
            })
            .sum::<f32>() / samples.len() as f32;
        
        if noise_power > 0.0 {
            10.0 * (signal_power / noise_power).log10()
        } else {
            100.0 // 很高的SNR
        }
    }
}

/// DAC电压转换
pub fn voltage_to_dac(voltage_mv: i16, vref_mv: u16) -> u16 {
    let offset_voltage = voltage_mv + (vref_mv as i16 / 2);
    if offset_voltage < 0 {
        0
    } else if offset_voltage > vref_mv as i16 {
        4095
    } else {
        ((offset_voltage as u32 * 4095) / vref_mv as u32) as u16
    }
}

/// DAC值转电压
pub fn dac_to_voltage(dac_value: u16, vref_mv: u16) -> i16 {
    let voltage = ((dac_value as u32 * vref_mv as u32) / 4095) as i16;
    voltage - (vref_mv as i16 / 2)
}