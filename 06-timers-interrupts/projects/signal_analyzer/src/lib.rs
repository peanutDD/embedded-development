#![no_std]

use heapless::{Vec, FnvIndexMap};
use libm::{sqrt, sin, cos, atan2, log10};
use micromath::F32Ext;
use core::marker::PhantomData;

/// 信号分析器特征
pub trait SignalAnalyzer {
    type Error;
    
    /// 添加样本数据
    fn add_sample(&mut self, sample: f32, timestamp_us: u64) -> Result<(), Self::Error>;
    
    /// 分析信号
    fn analyze(&mut self) -> Result<SignalAnalysisResult, Self::Error>;
    
    /// 重置分析器
    fn reset(&mut self);
    
    /// 获取采样率
    fn get_sample_rate(&self) -> u32;
}

/// 信号分析结果
#[derive(Debug, Clone)]
pub struct SignalAnalysisResult {
    pub frequency: f32,           // 基频 (Hz)
    pub amplitude: f32,           // 幅度
    pub dc_offset: f32,          // 直流偏移
    pub rms_value: f32,          // RMS值
    pub peak_to_peak: f32,       // 峰峰值
    pub duty_cycle: f32,         // 占空比 (0.0-1.0)
    pub rise_time_us: f32,       // 上升时间 (μs)
    pub fall_time_us: f32,       // 下降时间 (μs)
    pub pulse_width_us: f32,     // 脉宽 (μs)
    pub period_us: f32,          // 周期 (μs)
    pub thd: f32,                // 总谐波失真
    pub snr_db: f32,             // 信噪比 (dB)
}

/// 频率计数器
pub struct FrequencyCounter {
    edge_timestamps: Vec<u64, 1000>,
    last_sample: f32,
    threshold: f32,
    hysteresis: f32,
    edge_type: EdgeType,
    sample_rate: u32,
    min_frequency: f32,
    max_frequency: f32,
}

/// 脉宽分析器
pub struct PulseWidthAnalyzer {
    samples: Vec<(f32, u64), 2000>,
    threshold_high: f32,
    threshold_low: f32,
    current_state: PulseState,
    pulse_start: u64,
    pulse_end: u64,
    pulse_measurements: Vec<PulseMeasurement, 100>,
    sample_rate: u32,
}

/// 波形分析器
pub struct WaveformAnalyzer {
    samples: Vec<f32, 2048>,
    timestamps: Vec<u64, 2048>,
    sample_rate: u32,
    window_type: WindowType,
    analysis_result: Option<SignalAnalysisResult>,
}

/// 频谱分析器
#[cfg(feature = "dsp")]
pub struct SpectrumAnalyzer {
    samples: Vec<f32, 1024>,
    fft_buffer: Vec<microfft::Complex32, 512>,
    magnitude_spectrum: Vec<f32, 512>,
    sample_rate: u32,
    window_function: WindowType,
}

/// 实时信号监视器
pub struct RealTimeMonitor {
    frequency_counter: FrequencyCounter,
    pulse_analyzer: PulseWidthAnalyzer,
    waveform_analyzer: WaveformAnalyzer,
    statistics: SignalStatistics,
    update_interval_ms: u32,
    last_update: u64,
}

/// 边沿类型
#[derive(Debug, Clone, Copy)]
pub enum EdgeType {
    Rising,
    Falling,
    Both,
}

/// 脉冲状态
#[derive(Debug, Clone, Copy, PartialEq)]
enum PulseState {
    Low,
    High,
    Rising,
    Falling,
}

/// 脉冲测量结果
#[derive(Debug, Clone)]
pub struct PulseMeasurement {
    pub pulse_width_us: f32,
    pub period_us: f32,
    pub duty_cycle: f32,
    pub rise_time_us: f32,
    pub fall_time_us: f32,
    pub timestamp: u64,
}

/// 窗函数类型
#[derive(Debug, Clone, Copy)]
pub enum WindowType {
    Rectangle,
    Hanning,
    Hamming,
    Blackman,
    Kaiser,
}

/// 信号统计信息
#[derive(Debug, Clone)]
pub struct SignalStatistics {
    pub sample_count: u32,
    pub min_value: f32,
    pub max_value: f32,
    pub mean_value: f32,
    pub std_deviation: f32,
    pub zero_crossings: u32,
    pub peak_count: u32,
}

impl Default for SignalAnalysisResult {
    fn default() -> Self {
        Self {
            frequency: 0.0,
            amplitude: 0.0,
            dc_offset: 0.0,
            rms_value: 0.0,
            peak_to_peak: 0.0,
            duty_cycle: 0.0,
            rise_time_us: 0.0,
            fall_time_us: 0.0,
            pulse_width_us: 0.0,
            period_us: 0.0,
            thd: 0.0,
            snr_db: 0.0,
        }
    }
}

impl FrequencyCounter {
    /// 创建新的频率计数器
    pub fn new(sample_rate: u32, threshold: f32) -> Self {
        Self {
            edge_timestamps: Vec::new(),
            last_sample: 0.0,
            threshold,
            hysteresis: threshold * 0.1,
            edge_type: EdgeType::Rising,
            sample_rate,
            min_frequency: 0.1,
            max_frequency: sample_rate as f32 / 4.0,
        }
    }

    /// 设置边沿检测类型
    pub fn set_edge_type(&mut self, edge_type: EdgeType) {
        self.edge_type = edge_type;
    }

    /// 设置频率范围
    pub fn set_frequency_range(&mut self, min_freq: f32, max_freq: f32) {
        self.min_frequency = min_freq;
        self.max_frequency = max_freq;
    }

    /// 检测边沿
    fn detect_edge(&self, current: f32, previous: f32) -> Option<EdgeType> {
        match self.edge_type {
            EdgeType::Rising => {
                if previous < (self.threshold - self.hysteresis) && current > (self.threshold + self.hysteresis) {
                    Some(EdgeType::Rising)
                } else {
                    None
                }
            },
            EdgeType::Falling => {
                if previous > (self.threshold + self.hysteresis) && current < (self.threshold - self.hysteresis) {
                    Some(EdgeType::Falling)
                } else {
                    None
                }
            },
            EdgeType::Both => {
                if previous < (self.threshold - self.hysteresis) && current > (self.threshold + self.hysteresis) {
                    Some(EdgeType::Rising)
                } else if previous > (self.threshold + self.hysteresis) && current < (self.threshold - self.hysteresis) {
                    Some(EdgeType::Falling)
                } else {
                    None
                }
            },
        }
    }

    /// 计算频率
    pub fn calculate_frequency(&self) -> f32 {
        if self.edge_timestamps.len() < 2 {
            return 0.0;
        }

        let mut periods = Vec::<f32, 100>::new();
        
        for i in 1..self.edge_timestamps.len() {
            let period_us = (self.edge_timestamps[i] - self.edge_timestamps[i-1]) as f32;
            if period_us > 0.0 {
                let frequency = 1_000_000.0 / period_us;
                if frequency >= self.min_frequency && frequency <= self.max_frequency {
                    periods.push(period_us).ok();
                }
            }
        }

        if periods.is_empty() {
            return 0.0;
        }

        // 计算平均周期
        let avg_period: f32 = periods.iter().sum::<f32>() / periods.len() as f32;
        1_000_000.0 / avg_period
    }
}

impl SignalAnalyzer for FrequencyCounter {
    type Error = &'static str;

    fn add_sample(&mut self, sample: f32, timestamp_us: u64) -> Result<(), Self::Error> {
        if let Some(_edge) = self.detect_edge(sample, self.last_sample) {
            self.edge_timestamps.push(timestamp_us).map_err(|_| "Edge buffer full")?;
            
            // 保持缓冲区大小
            if self.edge_timestamps.len() > 500 {
                self.edge_timestamps.remove(0);
            }
        }
        
        self.last_sample = sample;
        Ok(())
    }

    fn analyze(&mut self) -> Result<SignalAnalysisResult, Self::Error> {
        let frequency = self.calculate_frequency();
        
        Ok(SignalAnalysisResult {
            frequency,
            period_us: if frequency > 0.0 { 1_000_000.0 / frequency } else { 0.0 },
            ..Default::default()
        })
    }

    fn reset(&mut self) {
        self.edge_timestamps.clear();
        self.last_sample = 0.0;
    }

    fn get_sample_rate(&self) -> u32 {
        self.sample_rate
    }
}

impl PulseWidthAnalyzer {
    /// 创建新的脉宽分析器
    pub fn new(sample_rate: u32, threshold_low: f32, threshold_high: f32) -> Self {
        Self {
            samples: Vec::new(),
            threshold_high,
            threshold_low,
            current_state: PulseState::Low,
            pulse_start: 0,
            pulse_end: 0,
            pulse_measurements: Vec::new(),
            sample_rate,
        }
    }

    /// 分析脉冲特性
    fn analyze_pulse(&mut self, timestamp: u64) -> Option<PulseMeasurement> {
        if self.pulse_start == 0 || self.pulse_end == 0 {
            return None;
        }

        let pulse_width_us = (self.pulse_end - self.pulse_start) as f32;
        
        // 查找下一个脉冲开始来计算周期
        let mut period_us = 0.0;
        if let Some(last_measurement) = self.pulse_measurements.last() {
            period_us = (timestamp - last_measurement.timestamp) as f32;
        }

        let duty_cycle = if period_us > 0.0 {
            pulse_width_us / period_us
        } else {
            0.0
        };

        // 简化的上升/下降时间计算
        let rise_time_us = self.sample_rate as f32 / 1_000_000.0; // 基于采样率的估算
        let fall_time_us = rise_time_us;

        Some(PulseMeasurement {
            pulse_width_us,
            period_us,
            duty_cycle,
            rise_time_us,
            fall_time_us,
            timestamp,
        })
    }
}

impl SignalAnalyzer for PulseWidthAnalyzer {
    type Error = &'static str;

    fn add_sample(&mut self, sample: f32, timestamp_us: u64) -> Result<(), Self::Error> {
        self.samples.push((sample, timestamp_us)).map_err(|_| "Sample buffer full")?;

        // 状态机处理
        let new_state = match self.current_state {
            PulseState::Low => {
                if sample > self.threshold_high {
                    self.pulse_start = timestamp_us;
                    PulseState::Rising
                } else {
                    PulseState::Low
                }
            },
            PulseState::Rising => {
                if sample > self.threshold_high {
                    PulseState::High
                } else if sample < self.threshold_low {
                    PulseState::Low
                } else {
                    PulseState::Rising
                }
            },
            PulseState::High => {
                if sample < self.threshold_low {
                    self.pulse_end = timestamp_us;
                    PulseState::Falling
                } else {
                    PulseState::High
                }
            },
            PulseState::Falling => {
                if sample < self.threshold_low {
                    // 脉冲结束，分析脉冲特性
                    if let Some(measurement) = self.analyze_pulse(timestamp_us) {
                        self.pulse_measurements.push(measurement).map_err(|_| "Measurement buffer full")?;
                    }
                    PulseState::Low
                } else if sample > self.threshold_high {
                    PulseState::High
                } else {
                    PulseState::Falling
                }
            },
        };

        self.current_state = new_state;

        // 保持缓冲区大小
        if self.samples.len() > 1000 {
            self.samples.remove(0);
        }

        Ok(())
    }

    fn analyze(&mut self) -> Result<SignalAnalysisResult, Self::Error> {
        if self.pulse_measurements.is_empty() {
            return Ok(SignalAnalysisResult::default());
        }

        // 计算平均值
        let count = self.pulse_measurements.len() as f32;
        let avg_pulse_width: f32 = self.pulse_measurements.iter()
            .map(|m| m.pulse_width_us)
            .sum::<f32>() / count;
        
        let avg_period: f32 = self.pulse_measurements.iter()
            .map(|m| m.period_us)
            .filter(|&p| p > 0.0)
            .sum::<f32>() / count;

        let avg_duty_cycle: f32 = self.pulse_measurements.iter()
            .map(|m| m.duty_cycle)
            .sum::<f32>() / count;

        let frequency = if avg_period > 0.0 {
            1_000_000.0 / avg_period
        } else {
            0.0
        };

        Ok(SignalAnalysisResult {
            frequency,
            pulse_width_us: avg_pulse_width,
            period_us: avg_period,
            duty_cycle: avg_duty_cycle,
            rise_time_us: 1.0, // 简化值
            fall_time_us: 1.0, // 简化值
            ..Default::default()
        })
    }

    fn reset(&mut self) {
        self.samples.clear();
        self.pulse_measurements.clear();
        self.current_state = PulseState::Low;
        self.pulse_start = 0;
        self.pulse_end = 0;
    }

    fn get_sample_rate(&self) -> u32 {
        self.sample_rate
    }
}

impl WaveformAnalyzer {
    /// 创建新的波形分析器
    pub fn new(sample_rate: u32) -> Self {
        Self {
            samples: Vec::new(),
            timestamps: Vec::new(),
            sample_rate,
            window_type: WindowType::Hanning,
            analysis_result: None,
        }
    }

    /// 设置窗函数类型
    pub fn set_window_type(&mut self, window_type: WindowType) {
        self.window_type = window_type;
    }

    /// 应用窗函数
    fn apply_window(&self, samples: &mut [f32]) {
        let n = samples.len();
        
        for (i, sample) in samples.iter_mut().enumerate() {
            let window_value = match self.window_type {
                WindowType::Rectangle => 1.0,
                WindowType::Hanning => 0.5 * (1.0 - cos(2.0 * core::f32::consts::PI * i as f32 / (n - 1) as f32)),
                WindowType::Hamming => 0.54 - 0.46 * cos(2.0 * core::f32::consts::PI * i as f32 / (n - 1) as f32),
                WindowType::Blackman => {
                    let a0 = 0.42;
                    let a1 = 0.5;
                    let a2 = 0.08;
                    a0 - a1 * cos(2.0 * core::f32::consts::PI * i as f32 / (n - 1) as f32) +
                    a2 * cos(4.0 * core::f32::consts::PI * i as f32 / (n - 1) as f32)
                },
                WindowType::Kaiser => 1.0, // 简化实现
            };
            
            *sample *= window_value;
        }
    }

    /// 计算信号统计
    fn calculate_statistics(&self) -> SignalStatistics {
        if self.samples.is_empty() {
            return SignalStatistics {
                sample_count: 0,
                min_value: 0.0,
                max_value: 0.0,
                mean_value: 0.0,
                std_deviation: 0.0,
                zero_crossings: 0,
                peak_count: 0,
            };
        }

        let sample_count = self.samples.len() as u32;
        let min_value = self.samples.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        let max_value = self.samples.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        let mean_value = self.samples.iter().sum::<f32>() / sample_count as f32;

        // 标准差
        let variance = self.samples.iter()
            .map(|&x| (x - mean_value).powi(2))
            .sum::<f32>() / sample_count as f32;
        let std_deviation = sqrt(variance);

        // 零交叉点计数
        let mut zero_crossings = 0;
        for i in 1..self.samples.len() {
            if (self.samples[i] >= 0.0) != (self.samples[i-1] >= 0.0) {
                zero_crossings += 1;
            }
        }

        // 峰值计数（简化实现）
        let mut peak_count = 0;
        for i in 1..self.samples.len()-1 {
            if self.samples[i] > self.samples[i-1] && self.samples[i] > self.samples[i+1] {
                peak_count += 1;
            }
        }

        SignalStatistics {
            sample_count,
            min_value,
            max_value,
            mean_value,
            std_deviation,
            zero_crossings,
            peak_count,
        }
    }
}

impl SignalAnalyzer for WaveformAnalyzer {
    type Error = &'static str;

    fn add_sample(&mut self, sample: f32, timestamp_us: u64) -> Result<(), Self::Error> {
        self.samples.push(sample).map_err(|_| "Sample buffer full")?;
        self.timestamps.push(timestamp_us).map_err(|_| "Timestamp buffer full")?;

        // 保持缓冲区大小
        if self.samples.len() > 1024 {
            self.samples.remove(0);
            self.timestamps.remove(0);
        }

        Ok(())
    }

    fn analyze(&mut self) -> Result<SignalAnalysisResult, Self::Error> {
        if self.samples.len() < 32 {
            return Err("Insufficient samples for analysis");
        }

        let statistics = self.calculate_statistics();
        
        // 基本信号参数
        let dc_offset = statistics.mean_value;
        let peak_to_peak = statistics.max_value - statistics.min_value;
        let amplitude = peak_to_peak / 2.0;
        
        // RMS值计算
        let rms_value = sqrt(self.samples.iter()
            .map(|&x| x * x)
            .sum::<f32>() / self.samples.len() as f32);

        // 频率估算（基于零交叉点）
        let duration_us = if self.timestamps.len() >= 2 {
            (self.timestamps[self.timestamps.len()-1] - self.timestamps[0]) as f32
        } else {
            0.0
        };
        
        let frequency = if duration_us > 0.0 {
            (statistics.zero_crossings as f32 / 2.0) * 1_000_000.0 / duration_us
        } else {
            0.0
        };

        let result = SignalAnalysisResult {
            frequency,
            amplitude,
            dc_offset,
            rms_value,
            peak_to_peak,
            period_us: if frequency > 0.0 { 1_000_000.0 / frequency } else { 0.0 },
            duty_cycle: 0.5, // 简化值
            rise_time_us: 0.0,
            fall_time_us: 0.0,
            pulse_width_us: 0.0,
            thd: 0.0, // 需要FFT计算
            snr_db: 0.0, // 需要噪声分析
        };

        self.analysis_result = Some(result.clone());
        Ok(result)
    }

    fn reset(&mut self) {
        self.samples.clear();
        self.timestamps.clear();
        self.analysis_result = None;
    }

    fn get_sample_rate(&self) -> u32 {
        self.sample_rate
    }
}

impl RealTimeMonitor {
    /// 创建新的实时监视器
    pub fn new(sample_rate: u32, update_interval_ms: u32) -> Self {
        Self {
            frequency_counter: FrequencyCounter::new(sample_rate, 0.5),
            pulse_analyzer: PulseWidthAnalyzer::new(sample_rate, 0.3, 0.7),
            waveform_analyzer: WaveformAnalyzer::new(sample_rate),
            statistics: SignalStatistics {
                sample_count: 0,
                min_value: 0.0,
                max_value: 0.0,
                mean_value: 0.0,
                std_deviation: 0.0,
                zero_crossings: 0,
                peak_count: 0,
            },
            update_interval_ms,
            last_update: 0,
        }
    }

    /// 添加样本到所有分析器
    pub fn add_sample(&mut self, sample: f32, timestamp_us: u64) -> Result<(), &'static str> {
        self.frequency_counter.add_sample(sample, timestamp_us)?;
        self.pulse_analyzer.add_sample(sample, timestamp_us)?;
        self.waveform_analyzer.add_sample(sample, timestamp_us)?;
        Ok(())
    }

    /// 更新分析结果
    pub fn update(&mut self, current_time_us: u64) -> Result<Option<SignalAnalysisResult>, &'static str> {
        if (current_time_us - self.last_update) < (self.update_interval_ms as u64 * 1000) {
            return Ok(None);
        }

        // 合并所有分析器的结果
        let freq_result = self.frequency_counter.analyze()?;
        let pulse_result = self.pulse_analyzer.analyze()?;
        let waveform_result = self.waveform_analyzer.analyze()?;

        let combined_result = SignalAnalysisResult {
            frequency: freq_result.frequency,
            amplitude: waveform_result.amplitude,
            dc_offset: waveform_result.dc_offset,
            rms_value: waveform_result.rms_value,
            peak_to_peak: waveform_result.peak_to_peak,
            duty_cycle: pulse_result.duty_cycle,
            rise_time_us: pulse_result.rise_time_us,
            fall_time_us: pulse_result.fall_time_us,
            pulse_width_us: pulse_result.pulse_width_us,
            period_us: freq_result.period_us,
            thd: 0.0,
            snr_db: 0.0,
        };

        self.last_update = current_time_us;
        Ok(Some(combined_result))
    }

    /// 获取统计信息
    pub fn get_statistics(&self) -> &SignalStatistics {
        &self.statistics
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frequency_counter() {
        let mut counter = FrequencyCounter::new(10000, 0.5);
        
        // 模拟1kHz方波信号
        for i in 0..1000 {
            let time_us = i * 100; // 100μs间隔
            let sample = if (i / 5) % 2 == 0 { 0.0 } else { 1.0 }; // 1kHz方波
            counter.add_sample(sample, time_us).unwrap();
        }
        
        let result = counter.analyze().unwrap();
        assert!(result.frequency > 900.0 && result.frequency < 1100.0);
    }

    #[test]
    fn test_pulse_analyzer() {
        let mut analyzer = PulseWidthAnalyzer::new(10000, 0.3, 0.7);
        
        // 模拟PWM信号
        for i in 0..1000 {
            let time_us = i * 100;
            let sample = if (i % 10) < 3 { 1.0 } else { 0.0 }; // 30%占空比
            analyzer.add_sample(sample, time_us).unwrap();
        }
        
        let result = analyzer.analyze().unwrap();
        assert!(result.duty_cycle > 0.2 && result.duty_cycle < 0.4);
    }

    #[test]
    fn test_waveform_analyzer() {
        let mut analyzer = WaveformAnalyzer::new(10000);
        
        // 模拟正弦波
        for i in 0..1000 {
            let time_us = i * 100;
            let sample = sin(2.0 * core::f32::consts::PI * i as f32 / 100.0);
            analyzer.add_sample(sample, time_us).unwrap();
        }
        
        let result = analyzer.analyze().unwrap();
        assert!(result.amplitude > 0.8 && result.amplitude < 1.2);
        assert!(result.dc_offset.abs() < 0.1);
    }
}