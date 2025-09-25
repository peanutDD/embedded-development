# 噪声源分析技术文档

## 概述

本文档详细分析STM32F4系列微控制器ADC/DAC系统中的各种噪声源，包括噪声的产生机制、特性分析、测量方法和抑制技术。

## 噪声分类和特性

### 1. 噪声类型分类

#### 1.1 按频率特性分类
- **白噪声**: 功率谱密度在所有频率上均匀分布
- **粉红噪声(1/f噪声)**: 功率谱密度与频率成反比
- **带限噪声**: 在特定频率范围内的噪声
- **脉冲噪声**: 瞬态高幅度噪声

#### 1.2 按产生机制分类
- **热噪声**: 由电阻中电子热运动产生
- **散粒噪声**: 由载流子随机通过势垒产生
- **闪烁噪声**: 由载流子数量随机变化产生
- **量化噪声**: 由ADC/DAC量化过程产生
- **开关噪声**: 由数字开关活动产生

### 2. 噪声源识别

```rust
#[derive(Debug, PartialEq)]
pub enum RecommendationPriority {
    High,
    Medium,
    Low,
}

#[derive(Debug)]
pub struct DominantFrequency {
    pub frequency: f32,
    pub amplitude: f32,
    pub rank: usize,
    pub possible_source: String,
}

#[derive(Debug)]
pub struct ReportHeader {
    pub title: String,
    pub date: chrono::DateTime<chrono::Utc>,
    pub version: String,
    pub author: String,
}

#[derive(Debug)]
pub struct SystemConfiguration {
    pub adc_resolution: u8,
    pub dac_resolution: u8,
    pub sampling_rate: f32,
    pub reference_voltage: f32,
    pub temperature: f32,
    pub supply_voltage: f32,
}

#[derive(Debug)]
pub struct NoiseMeasurements {
    pub input_referred_noise: Option<NoiseSpectrum>,
    pub output_noise: Option<NoiseSpectrum>,
    pub snr_measurements: Vec<SnrMeasurement>,
    pub total_noise_analysis: NoiseAnalysisResult,
}

#[derive(Debug)]
pub struct NoiseSourcesAnalysis {
    pub primary_sources: Vec<String>,
    pub secondary_sources: Vec<String>,
    pub mitigation_effectiveness: Vec<f32>,
}

#[derive(Debug)]
pub struct FrequencyAnalysis {
    pub frequency_response: Vec<FrequencyNoiseResponse>,
    pub dominant_noise_frequencies: Vec<DominantFrequency>,
    pub noise_floor_analysis: NoiseFloorAnalysis,
}

#[derive(Debug)]
pub struct NoiseFloorAnalysis {
    pub white_noise_floor: f32,
    pub corner_frequency: f32,
    pub low_frequency_noise: f32,
}

#[derive(Debug)]
pub struct PerformanceMetrics {
    pub snr_db: f32,
    pub thd_db: f32,
    pub sinad_db: f32,
    pub enob: f32,
    pub noise_figure: f32,
}
```

## 最佳实践和建议

### 1. 噪声测量最佳实践

1. **测量环境控制**
   - 使用屏蔽室或法拉第笼
   - 控制环境温度和湿度
   - 避免强电磁干扰源

2. **测量设备选择**
   - 使用低噪声测量设备
   - 确保测量带宽足够
   - 校准测量系统

3. **测量方法**
   - 多次测量取平均
   - 使用适当的窗函数
   - 考虑测量不确定度

### 2. 噪声抑制策略

1. **系统级策略**
   - 合理的系统架构设计
   - 信号链优化
   - 功率预算管理

2. **电路级策略**
   - 低噪声器件选择
   - 合理的增益分配
   - 滤波器设计

3. **PCB级策略**
   - 优化布局布线
   - 良好的接地设计
   - 适当的屏蔽

### 3. 故障排除指南

1. **噪声过高**
   - 检查电源质量
   - 验证接地系统
   - 排查干扰源

2. **特定频率噪声**
   - 识别噪声源
   - 设计针对性滤波
   - 改善屏蔽

3. **随机噪声**
   - 检查热噪声源
   - 优化信号链
   - 增加平均处理

## 结论

噪声分析是ADC/DAC系统设计中的关键环节。通过系统性的噪声源识别、测量和分析，可以有效提高系统的信噪比和测量精度。本文档提供的分析方法和工具可以帮助工程师快速定位噪声问题并制定有效的解决方案。

在实际应用中，应根据具体的应用需求和性能指标，选择合适的噪声分析和抑制技术，以达到最佳的系统性能。Debug, Clone)]
pub enum NoiseSource {
    Thermal {
        resistance: f32,        // 电阻值(Ω)
        temperature: f32,       // 温度(K)
        bandwidth: f32,         // 带宽(Hz)
    },
    Shot {
        current: f32,           // 电流(A)
        bandwidth: f32,         // 带宽(Hz)
    },
    Flicker {
        corner_frequency: f32,  // 拐角频率(Hz)
        amplitude: f32,         // 幅度
    },
    Quantization {
        resolution: u8,         // 分辨率(位)
        full_scale: f32,        // 满量程(V)
        sampling_rate: f32,     // 采样率(Hz)
    },
    Switching {
        frequency: f32,         // 开关频率(Hz)
        amplitude: f32,         // 幅度(V)
        harmonics: Vec<f32>,    // 谐波分量
    },
    PowerSupply {
        ripple_frequency: f32,  // 纹波频率(Hz)
        ripple_amplitude: f32,  // 纹波幅度(V)
        psrr: f32,             // 电源抑制比(dB)
    },
    Ground {
        impedance: f32,         // 地阻抗(Ω)
        current: f32,           // 地电流(A)
    },
    Crosstalk {
        coupling_coefficient: f32, // 耦合系数
        aggressor_amplitude: f32,  // 干扰源幅度(V)
        frequency: f32,            // 频率(Hz)
    },
}

impl NoiseSource {
    pub fn calculate_noise_power(&self) -> f32 {
        match self {
            NoiseSource::Thermal { resistance, temperature, bandwidth } => {
                // 热噪声功率: P = 4kTRB
                4.0 * BOLTZMANN_CONSTANT * temperature * resistance * bandwidth
            },
            NoiseSource::Shot { current, bandwidth } => {
                // 散粒噪声功率: P = 2qIB
                2.0 * ELECTRON_CHARGE * current * bandwidth
            },
            NoiseSource::Flicker { corner_frequency, amplitude } => {
                // 闪烁噪声功率密度在拐角频率处
                amplitude * amplitude / corner_frequency
            },
            NoiseSource::Quantization { resolution, full_scale, sampling_rate } => {
                // 量化噪声功率: P = (LSB)²/12
                let lsb = full_scale / (1 << resolution) as f32;
                (lsb * lsb) / 12.0
            },
            NoiseSource::Switching { amplitude, .. } => {
                // 开关噪声功率（简化模型）
                (amplitude * amplitude) / 2.0
            },
            NoiseSource::PowerSupply { ripple_amplitude, psrr, .. } => {
                // 电源噪声功率
                let noise_amplitude = ripple_amplitude / (10.0_f32.powf(psrr / 20.0));
                (noise_amplitude * noise_amplitude) / 2.0
            },
            NoiseSource::Ground { impedance, current } => {
                // 地噪声功率: P = I²R
                current * current * impedance
            },
            NoiseSource::Crosstalk { coupling_coefficient, aggressor_amplitude, .. } => {
                // 串扰噪声功率
                let coupled_amplitude = coupling_coefficient * aggressor_amplitude;
                (coupled_amplitude * coupled_amplitude) / 2.0
            },
        }
    }

    pub fn calculate_noise_voltage_rms(&self) -> f32 {
        self.calculate_noise_power().sqrt()
    }

    pub fn get_frequency_characteristic(&self, frequency: f32) -> f32 {
        match self {
            NoiseSource::Thermal { .. } => 1.0, // 白噪声
            NoiseSource::Shot { .. } => 1.0,    // 白噪声
            NoiseSource::Flicker { corner_frequency, .. } => {
                // 1/f噪声
                (corner_frequency / frequency).sqrt()
            },
            NoiseSource::Quantization { .. } => 1.0, // 白噪声
            NoiseSource::Switching { frequency: switch_freq, harmonics, .. } => {
                // 开关噪声在基频和谐波处有峰值
                let mut response = 0.0;
                response += if (frequency - switch_freq).abs() < 10.0 { 1.0 } else { 0.1 };
                for (i, &harmonic_amp) in harmonics.iter().enumerate() {
                    let harmonic_freq = switch_freq * (i + 2) as f32;
                    if (frequency - harmonic_freq).abs() < 10.0 {
                        response += harmonic_amp;
                    }
                }
                response
            },
            NoiseSource::PowerSupply { ripple_frequency, .. } => {
                // 电源噪声主要在纹波频率及其谐波
                if (frequency - ripple_frequency).abs() < 1.0 ||
                   (frequency - 2.0 * ripple_frequency).abs() < 1.0 {
                    1.0
                } else {
                    0.1
                }
            },
            NoiseSource::Ground { .. } => 1.0, // 宽带噪声
            NoiseSource::Crosstalk { frequency: crosstalk_freq, .. } => {
                // 串扰噪声主要在特定频率
                if (frequency - crosstalk_freq).abs() < 10.0 {
                    1.0
                } else {
                    0.1
                }
            },
        }
    }
}

const BOLTZMANN_CONSTANT: f32 = 1.38e-23; // J/K
const ELECTRON_CHARGE: f32 = 1.602e-19;   // C
```

### 3. 系统噪声模型

```rust
pub struct SystemNoiseModel {
    noise_sources: Vec<NoiseSource>,
    system_bandwidth: f32,
    temperature: f32,
    supply_voltage: f32,
    ground_impedance: f32,
}

impl SystemNoiseModel {
    pub fn new(bandwidth: f32, temperature: f32) -> Self {
        Self {
            noise_sources: Vec::new(),
            system_bandwidth: bandwidth,
            temperature,
            supply_voltage: 3.3,
            ground_impedance: 0.01, // 10mΩ
        }
    }

    pub fn add_adc_noise_sources(&mut self, adc_config: &AdcConfig) {
        // ADC内部噪声源
        
        // 1. 输入缓冲器热噪声
        self.noise_sources.push(NoiseSource::Thermal {
            resistance: 1000.0, // 1kΩ等效输入阻抗
            temperature: self.temperature,
            bandwidth: self.system_bandwidth,
        });

        // 2. 采样保持电路噪声
        self.noise_sources.push(NoiseSource::Thermal {
            resistance: 500.0, // 500Ω开关阻抗
            temperature: self.temperature,
            bandwidth: self.system_bandwidth,
        });

        // 3. 比较器噪声
        self.noise_sources.push(NoiseSource::Shot {
            current: 1e-9, // 1nA偏置电流
            bandwidth: self.system_bandwidth,
        });

        // 4. 参考电压噪声
        self.noise_sources.push(NoiseSource::Flicker {
            corner_frequency: 100.0, // 100Hz拐角频率
            amplitude: 10e-6, // 10μV
        });

        // 5. 量化噪声
        self.noise_sources.push(NoiseSource::Quantization {
            resolution: adc_config.resolution,
            full_scale: adc_config.reference_voltage,
            sampling_rate: adc_config.sampling_rate,
        });

        // 6. 时钟抖动噪声
        let aperture_jitter = 1e-12; // 1ps RMS抖动
        let max_input_frequency = self.system_bandwidth;
        let jitter_noise_amplitude = 2.0 * PI * max_input_frequency * aperture_jitter * adc_config.reference_voltage;
        
        self.noise_sources.push(NoiseSource::Switching {
            frequency: adc_config.sampling_rate,
            amplitude: jitter_noise_amplitude,
            harmonics: vec![0.5, 0.25, 0.125], // 谐波衰减
        });
    }

    pub fn add_dac_noise_sources(&mut self, dac_config: &DacConfig) {
        // DAC内部噪声源

        // 1. 电流源噪声
        self.noise_sources.push(NoiseSource::Shot {
            current: 10e-6, // 10μA满量程电流
            bandwidth: self.system_bandwidth,
        });

        // 2. 输出缓冲器噪声
        self.noise_sources.push(NoiseSource::Thermal {
            resistance: 100.0, // 100Ω输出阻抗
            temperature: self.temperature,
            bandwidth: self.system_bandwidth,
        });

        // 3. 参考电压噪声
        self.noise_sources.push(NoiseSource::Flicker {
            corner_frequency: 50.0, // 50Hz拐角频率
            amplitude: 5e-6, // 5μV
        });

        // 4. 量化噪声
        self.noise_sources.push(NoiseSource::Quantization {
            resolution: dac_config.resolution,
            full_scale: dac_config.reference_voltage,
            sampling_rate: dac_config.update_rate,
        });

        // 5. 开关噪声
        self.noise_sources.push(NoiseSource::Switching {
            frequency: dac_config.update_rate,
            amplitude: 1e-3, // 1mV开关噪声
            harmonics: vec![0.3, 0.1, 0.05],
        });
    }

    pub fn add_external_noise_sources(&mut self) {
        // 外部噪声源

        // 1. 电源噪声
        self.noise_sources.push(NoiseSource::PowerSupply {
            ripple_frequency: 100.0, // 100Hz纹波
            ripple_amplitude: 10e-3,  // 10mV纹波
            psrr: 60.0,              // 60dB PSRR
        });

        // 2. 地噪声
        self.noise_sources.push(NoiseSource::Ground {
            impedance: self.ground_impedance,
            current: 100e-3, // 100mA地电流
        });

        // 3. 数字开关噪声
        self.noise_sources.push(NoiseSource::Switching {
            frequency: 72e6, // 72MHz系统时钟
            amplitude: 5e-3, // 5mV耦合噪声
            harmonics: vec![0.5, 0.25, 0.125, 0.0625],
        });

        // 4. 通道间串扰
        self.noise_sources.push(NoiseSource::Crosstalk {
            coupling_coefficient: 0.01, // -40dB串扰
            aggressor_amplitude: 1.0,    // 1V干扰信号
            frequency: 1000.0,           // 1kHz测试频率
        });
    }

    pub fn calculate_total_noise(&self) -> NoiseAnalysisResult {
        let mut total_power = 0.0;
        let mut noise_breakdown = Vec::new();

        for (i, source) in self.noise_sources.iter().enumerate() {
            let power = source.calculate_noise_power();
            let voltage_rms = power.sqrt();
            
            total_power += power;
            
            noise_breakdown.push(NoiseContribution {
                source_index: i,
                source_type: format!("{:?}", source),
                power_watts: power,
                voltage_rms: voltage_rms,
                percentage: 0.0, // 稍后计算
            });
        }

        let total_voltage_rms = total_power.sqrt();
        
        // 计算各噪声源的贡献百分比
        for contribution in &mut noise_breakdown {
            contribution.percentage = (contribution.power_watts / total_power) * 100.0;
        }

        // 按贡献大小排序
        noise_breakdown.sort_by(|a, b| b.percentage.partial_cmp(&a.percentage).unwrap());

        NoiseAnalysisResult {
            total_noise_power: total_power,
            total_noise_voltage_rms: total_voltage_rms,
            total_noise_voltage_pp: total_voltage_rms * 6.6, // 假设高斯分布，99.9%概率
            noise_breakdown,
            snr_db: self.calculate_snr(total_voltage_rms),
            enob: self.calculate_enob(total_voltage_rms),
        }
    }

    pub fn calculate_frequency_response(&self, frequencies: &[f32]) -> Vec<FrequencyNoiseResponse> {
        let mut response = Vec::new();

        for &freq in frequencies {
            let mut total_power_at_freq = 0.0;
            let mut source_contributions = Vec::new();

            for source in &self.noise_sources {
                let freq_response = source.get_frequency_characteristic(freq);
                let base_power = source.calculate_noise_power();
                let power_at_freq = base_power * freq_response * freq_response;
                
                total_power_at_freq += power_at_freq;
                source_contributions.push(power_at_freq);
            }

            response.push(FrequencyNoiseResponse {
                frequency: freq,
                total_noise_power: total_power_at_freq,
                total_noise_voltage_rms: total_power_at_freq.sqrt(),
                source_contributions,
            });
        }

        response
    }

    fn calculate_snr(&self, noise_voltage_rms: f32) -> f32 {
        let signal_amplitude = self.supply_voltage / 2.0; // 假设满量程信号
        20.0 * (signal_amplitude / noise_voltage_rms).log10()
    }

    fn calculate_enob(&self, noise_voltage_rms: f32) -> f32 {
        let snr_db = self.calculate_snr(noise_voltage_rms);
        (snr_db - 1.76) / 6.02 // ENOB = (SNR - 1.76) / 6.02
    }
}

#[derive(Debug)]
pub struct NoiseAnalysisResult {
    pub total_noise_power: f32,
    pub total_noise_voltage_rms: f32,
    pub total_noise_voltage_pp: f32,
    pub noise_breakdown: Vec<NoiseContribution>,
    pub snr_db: f32,
    pub enob: f32,
}

#[derive(Debug)]
pub struct NoiseContribution {
    pub source_index: usize,
    pub source_type: String,
    pub power_watts: f32,
    pub voltage_rms: f32,
    pub percentage: f32,
}

#[derive(Debug)]
pub struct FrequencyNoiseResponse {
    pub frequency: f32,
    pub total_noise_power: f32,
    pub total_noise_voltage_rms: f32,
    pub source_contributions: Vec<f32>,
}
```

## 噪声测量技术

### 1. 噪声测量系统

```rust
pub struct NoiseMeasurementSystem {
    adc: Adc<ADC1>,
    dac: Dac<DAC1>,
    signal_generator: SignalGenerator,
    spectrum_analyzer: SpectrumAnalyzer,
    low_noise_amplifier: LowNoiseAmplifier,
    measurement_config: MeasurementConfig,
}

#[derive(Clone)]
pub struct MeasurementConfig {
    pub sampling_rate: f32,
    pub measurement_duration: f32,
    pub frequency_resolution: f32,
    pub averaging_count: u32,
    pub window_function: WindowFunction,
    pub input_coupling: InputCoupling,
    pub input_impedance: f32,
}

#[derive(Clone)]
pub enum WindowFunction {
    Rectangular,
    Hanning,
    Hamming,
    Blackman,
    Kaiser(f32), // β参数
}

#[derive(Clone)]
pub enum InputCoupling {
    DC,
    AC,
}

impl NoiseMeasurementSystem {
    pub fn measure_input_referred_noise(&mut self) -> Result<NoiseSpectrum, MeasurementError> {
        println!("测量输入参考噪声...");

        // 1. 短路输入
        self.short_circuit_input()?;
        
        // 2. 设置测量参数
        self.configure_for_noise_measurement()?;
        
        // 3. 采集噪声数据
        let noise_samples = self.acquire_noise_samples()?;
        
        // 4. 计算功率谱密度
        let psd = self.calculate_power_spectral_density(&noise_samples)?;
        
        // 5. 计算噪声指标
        let noise_metrics = self.calculate_noise_metrics(&psd)?;
        
        Ok(NoiseSpectrum {
            frequency_bins: self.generate_frequency_bins(),
            power_spectral_density: psd,
            metrics: noise_metrics,
        })
    }

    pub fn measure_output_noise(&mut self) -> Result<NoiseSpectrum, MeasurementError> {
        println!("测量输出噪声...");

        // 1. 设置DAC为中间值
        let mid_scale = (1 << 11) as u16; // 12位DAC的中间值
        self.dac.set_value(mid_scale);
        
        // 2. 测量输出噪声
        let noise_samples = self.acquire_output_noise_samples()?;
        
        // 3. 计算功率谱密度
        let psd = self.calculate_power_spectral_density(&noise_samples)?;
        
        // 4. 计算噪声指标
        let noise_metrics = self.calculate_noise_metrics(&psd)?;
        
        Ok(NoiseSpectrum {
            frequency_bins: self.generate_frequency_bins(),
            power_spectral_density: psd,
            metrics: noise_metrics,
        })
    }

    pub fn measure_snr(&mut self, signal_frequency: f32, signal_amplitude: f32) -> Result<SnrMeasurement, MeasurementError> {
        println!("测量信噪比 @{}Hz", signal_frequency);

        // 1. 生成测试信号
        self.signal_generator.generate_sine_wave(signal_frequency, signal_amplitude)?;
        
        // 2. 采集信号+噪声
        let signal_plus_noise = self.acquire_signal_samples()?;
        
        // 3. 计算频谱
        let spectrum = self.calculate_fft(&signal_plus_noise)?;
        
        // 4. 识别信号峰值
        let signal_bin = self.find_signal_bin(signal_frequency);
        let signal_power = spectrum[signal_bin].norm_sqr();
        
        // 5. 计算噪声功率（排除信号峰值和谐波）
        let noise_power = self.calculate_noise_power_excluding_signal(&spectrum, signal_frequency)?;
        
        // 6. 计算SNR
        let snr_linear = signal_power / noise_power;
        let snr_db = 10.0 * snr_linear.log10();
        
        // 7. 计算THD
        let thd = self.calculate_thd(&spectrum, signal_frequency)?;
        
        // 8. 计算SINAD
        let sinad = self.calculate_sinad(&spectrum, signal_frequency)?;
        
        Ok(SnrMeasurement {
            signal_frequency,
            signal_amplitude,
            signal_power,
            noise_power,
            snr_db,
            thd_db: thd,
            sinad_db: sinad,
            enob: (sinad - 1.76) / 6.02,
        })
    }

    fn acquire_noise_samples(&mut self) -> Result<Vec<f32>, MeasurementError> {
        let sample_count = (self.measurement_config.sampling_rate * 
                           self.measurement_config.measurement_duration) as usize;
        let mut samples = Vec::with_capacity(sample_count);
        
        // 多次平均以提高测量精度
        for avg_count in 0..self.measurement_config.averaging_count {
            let mut current_samples = Vec::with_capacity(sample_count);
            
            for _ in 0..sample_count {
                let sample = self.adc.read()? as f32;
                // 转换为电压值
                let voltage = (sample / 4095.0) * 3.3 - 1.65; // 假设3.3V参考，AC耦合
                current_samples.push(voltage);
            }
            
            // 累加到总样本中
            if avg_count == 0 {
                samples = current_samples;
            } else {
                for (i, &sample) in current_samples.iter().enumerate() {
                    samples[i] += sample;
                }
            }
            
            println!("完成第{}/{}次平均", avg_count + 1, self.measurement_config.averaging_count);
        }
        
        // 计算平均值
        for sample in &mut samples {
            *sample /= self.measurement_config.averaging_count as f32;
        }
        
        Ok(samples)
    }

    fn calculate_power_spectral_density(&self, samples: &[f32]) -> Result<Vec<f32>, MeasurementError> {
        // 1. 应用窗函数
        let windowed_samples = self.apply_window_function(samples);
        
        // 2. 计算FFT
        let fft_result = self.calculate_fft(&windowed_samples)?;
        
        // 3. 计算功率谱密度
        let mut psd = Vec::with_capacity(fft_result.len() / 2);
        let window_power = self.calculate_window_power();
        let fs = self.measurement_config.sampling_rate;
        let n = samples.len() as f32;
        
        for i in 0..fft_result.len() / 2 {
            let magnitude_squared = fft_result[i].norm_sqr();
            // PSD = |X(f)|² / (fs * N * window_power)
            let psd_value = magnitude_squared / (fs * n * window_power);
            psd.push(psd_value);
        }
        
        Ok(psd)
    }

    fn apply_window_function(&self, samples: &[f32]) -> Vec<f32> {
        let n = samples.len();
        let mut windowed = Vec::with_capacity(n);
        
        for (i, &sample) in samples.iter().enumerate() {
            let window_value = match &self.measurement_config.window_function {
                WindowFunction::Rectangular => 1.0,
                WindowFunction::Hanning => {
                    0.5 * (1.0 - (2.0 * PI * i as f32 / (n - 1) as f32).cos())
                },
                WindowFunction::Hamming => {
                    0.54 - 0.46 * (2.0 * PI * i as f32 / (n - 1) as f32).cos()
                },
                WindowFunction::Blackman => {
                    0.42 - 0.5 * (2.0 * PI * i as f32 / (n - 1) as f32).cos() +
                    0.08 * (4.0 * PI * i as f32 / (n - 1) as f32).cos()
                },
                WindowFunction::Kaiser(beta) => {
                    let alpha = (n - 1) as f32 / 2.0;
                    let x = (i as f32 - alpha) / alpha;
                    self.modified_bessel_i0(*beta * (1.0 - x * x).sqrt()) / 
                    self.modified_bessel_i0(*beta)
                },
            };
            
            windowed.push(sample * window_value);
        }
        
        windowed
    }

    fn calculate_noise_metrics(&self, psd: &[f32]) -> Result<NoiseMetrics, MeasurementError> {
        let fs = self.measurement_config.sampling_rate;
        let df = fs / (2.0 * psd.len() as f32);
        
        // 计算总噪声功率
        let total_noise_power: f32 = psd.iter().map(|&p| p * df).sum();
        let total_noise_voltage_rms = total_noise_power.sqrt();
        
        // 计算噪声密度
        let noise_density = psd.iter().map(|&p| p.sqrt()).collect::<Vec<f32>>();
        
        // 查找峰值噪声频率
        let (peak_frequency_index, &peak_noise_density) = noise_density
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();
        let peak_frequency = peak_frequency_index as f32 * df;
        
        // 计算1/f噪声拐角频率
        let corner_frequency = self.estimate_corner_frequency(psd, df)?;
        
        // 计算白噪声底限
        let white_noise_floor = self.estimate_white_noise_floor(psd)?;
        
        Ok(NoiseMetrics {
            total_noise_power,
            total_noise_voltage_rms,
            total_noise_voltage_pp: total_noise_voltage_rms * 6.6, // 99.9%概率
            noise_density,
            peak_frequency,
            peak_noise_density,
            corner_frequency,
            white_noise_floor,
            frequency_resolution: df,
        })
    }

    fn estimate_corner_frequency(&self, psd: &[f32], df: f32) -> Result<f32, MeasurementError> {
        // 寻找1/f噪声的拐角频率
        // 在低频段拟合1/f特性，找到与白噪声底限的交点
        
        let low_freq_end = (100.0 / df) as usize; // 分析到100Hz
        if low_freq_end >= psd.len() {
            return Ok(0.0); // 频率分辨率不够
        }
        
        // 估算白噪声底限（高频段平均值）
        let high_freq_start = psd.len() * 3 / 4;
        let white_noise_level: f32 = psd[high_freq_start..].iter().sum::<f32>() / 
                                    (psd.len() - high_freq_start) as f32;
        
        // 从低频开始查找，找到噪声密度降到白噪声底限2倍的频率
        for i in 10..low_freq_end { // 跳过DC附近
            if psd[i] <= white_noise_level * 2.0 {
                return Ok(i as f32 * df);
            }
        }
        
        Ok(100.0) // 默认100Hz
    }

    fn estimate_white_noise_floor(&self, psd: &[f32]) -> Result<f32, MeasurementError> {
        // 估算白噪声底限（高频段的平均值）
        let high_freq_start = psd.len() * 3 / 4;
        let white_noise_floor = psd[high_freq_start..].iter().sum::<f32>() / 
                               (psd.len() - high_freq_start) as f32;
        Ok(white_noise_floor.sqrt()) // 转换为电压密度
    }
}

#[derive(Debug)]
pub struct NoiseSpectrum {
    pub frequency_bins: Vec<f32>,
    pub power_spectral_density: Vec<f32>,
    pub metrics: NoiseMetrics,
}

#[derive(Debug)]
pub struct NoiseMetrics {
    pub total_noise_power: f32,
    pub total_noise_voltage_rms: f32,
    pub total_noise_voltage_pp: f32,
    pub noise_density: Vec<f32>,
    pub peak_frequency: f32,
    pub peak_noise_density: f32,
    pub corner_frequency: f32,
    pub white_noise_floor: f32,
    pub frequency_resolution: f32,
}

#[derive(Debug)]
pub struct SnrMeasurement {
    pub signal_frequency: f32,
    pub signal_amplitude: f32,
    pub signal_power: f32,
    pub noise_power: f32,
    pub snr_db: f32,
    pub thd_db: f32,
    pub sinad_db: f32,
    pub enob: f32,
}
```

## 噪声抑制技术

### 1. 硬件噪声抑制

```rust
pub struct NoiseSuppressionTechniques {
    pub filtering: FilteringTechniques,
    pub shielding: ShieldingTechniques,
    pub grounding: GroundingTechniques,
    pub power_supply: PowerSupplyTechniques,
    pub layout: LayoutTechniques,
}

#[derive(Debug)]
pub struct FilteringTechniques {
    pub anti_aliasing_filter: AntiAliasingFilter,
    pub power_supply_filter: PowerSupplyFilter,
    pub common_mode_filter: CommonModeFilter,
    pub differential_mode_filter: DifferentialModeFilter,
}

#[derive(Debug)]
pub struct AntiAliasingFilter {
    pub filter_type: FilterType,
    pub cutoff_frequency: f32,
    pub order: u8,
    pub attenuation_at_fs_2: f32, // dB
}

#[derive(Debug)]
pub enum FilterType {
    Butterworth,
    Chebyshev1(f32), // 通带纹波dB
    Chebyshev2(f32), // 阻带纹波dB
    Elliptic(f32, f32), // 通带纹波, 阻带纹波
    Bessel,
}

impl FilteringTechniques {
    pub fn design_anti_aliasing_filter(&self, sampling_rate: f32, signal_bandwidth: f32) -> AntiAliasingFilter {
        // 设计抗混叠滤波器
        let cutoff_frequency = signal_bandwidth * 1.2; // 留20%余量
        let required_attenuation = 60.0; // 60dB衰减
        let transition_band = sampling_rate / 2.0 - cutoff_frequency;
        
        // 根据过渡带宽度选择滤波器阶数
        let order = if transition_band > cutoff_frequency * 0.5 {
            4 // 宽过渡带，低阶滤波器
        } else if transition_band > cutoff_frequency * 0.2 {
            6 // 中等过渡带
        } else {
            8 // 窄过渡带，高阶滤波器
        };
        
        AntiAliasingFilter {
            filter_type: FilterType::Butterworth, // 平坦通带
            cutoff_frequency,
            order,
            attenuation_at_fs_2: required_attenuation,
        }
    }

    pub fn calculate_filter_response(&self, filter: &AntiAliasingFilter, frequency: f32) -> f32 {
        let normalized_freq = frequency / filter.cutoff_frequency;
        
        match &filter.filter_type {
            FilterType::Butterworth => {
                // |H(jω)|² = 1 / (1 + (ω/ωc)^(2n))
                let magnitude_squared = 1.0 / (1.0 + normalized_freq.powi(2 * filter.order as i32));
                magnitude_squared.sqrt()
            },
            FilterType::Chebyshev1(ripple_db) => {
                // 切比雪夫I型滤波器响应（简化）
                let epsilon = (10.0_f32.powf(ripple_db / 10.0) - 1.0).sqrt();
                let chebyshev_poly = self.chebyshev_polynomial(filter.order, normalized_freq);
                let magnitude_squared = 1.0 / (1.0 + epsilon * epsilon * chebyshev_poly * chebyshev_poly);
                magnitude_squared.sqrt()
            },
            _ => {
                // 其他滤波器类型的简化实现
                let magnitude_squared = 1.0 / (1.0 + normalized_freq.powi(2 * filter.order as i32));
                magnitude_squared.sqrt()
            }
        }
    }

    fn chebyshev_polynomial(&self, order: u8, x: f32) -> f32 {
        // 切比雪夫多项式的递归计算（简化版本）
        match order {
            0 => 1.0,
            1 => x,
            2 => 2.0 * x * x - 1.0,
            3 => 4.0 * x * x * x - 3.0 * x,
            _ => 2.0 * x * self.chebyshev_polynomial(order - 1, x) - 
                 self.chebyshev_polynomial(order - 2, x)
        }
    }
}

#[derive(Debug)]
pub struct GroundingTechniques {
    pub ground_plane_area: f32,      // 地平面面积 (mm²)
    pub ground_impedance: f32,       // 地阻抗 (Ω)
    pub star_grounding: bool,        // 星形接地
    pub separate_analog_digital: bool, // 模拟数字地分离
    pub ground_guard_rings: bool,    // 地保护环
}

impl GroundingTechniques {
    pub fn calculate_ground_noise(&self, ground_current: f32) -> f32 {
        // 地噪声 = 地电流 × 地阻抗
        ground_current * self.ground_impedance
    }

    pub fn optimize_ground_impedance(&mut self, target_impedance: f32) {
        // 根据目标阻抗优化地平面设计
        if self.ground_impedance > target_impedance {
            println!("当前地阻抗 {:.3}mΩ 超过目标 {:.3}mΩ", 
                    self.ground_impedance * 1000.0, 
                    target_impedance * 1000.0);
            
            // 建议增加地平面面积
            let required_area = self.ground_plane_area * (self.ground_impedance / target_impedance);
            println!("建议增加地平面面积到 {:.1}mm²", required_area);
            
            // 建议使用多层地平面
            if !self.separate_analog_digital {
                println!("建议分离模拟和数字地平面");
                self.separate_analog_digital = true;
            }
            
            // 建议添加地保护环
            if !self.ground_guard_rings {
                println!("建议添加地保护环");
                self.ground_guard_rings = true;
            }
        }
    }
}
```

### 2. 软件噪声抑制

```rust
pub struct SoftwareNoiseReduction {
    pub digital_filters: Vec<DigitalFilter>,
    pub averaging_filters: AveragingFilters,
    pub adaptive_filters: AdaptiveFilters,
    pub noise_cancellation: NoiseCancellation,
}

#[derive(Debug, Clone)]
pub struct DigitalFilter {
    pub filter_type: DigitalFilterType,
    pub coefficients: FilterCoefficients,
    pub delay_line: Vec<f32>,
    pub output_history: Vec<f32>,
}

#[derive(Debug, Clone)]
pub enum DigitalFilterType {
    FIR,
    IIR,
    Notch(f32), // 陷波频率
    HighPass(f32), // 截止频率
    LowPass(f32),  // 截止频率
    BandPass(f32, f32), // 下截止频率, 上截止频率
}

#[derive(Debug, Clone)]
pub struct FilterCoefficients {
    pub numerator: Vec<f32>,   // b系数
    pub denominator: Vec<f32>, // a系数
}

impl DigitalFilter {
    pub fn new_low_pass(cutoff_freq: f32, sampling_rate: f32, order: usize) -> Self {
        let normalized_cutoff = cutoff_freq / (sampling_rate / 2.0);
        let coefficients = Self::design_butterworth_lowpass(normalized_cutoff, order);
        
        Self {
            filter_type: DigitalFilterType::LowPass(cutoff_freq),
            coefficients,
            delay_line: vec![0.0; order + 1],
            output_history: vec![0.0; order],
        }
    }

    pub fn new_notch(notch_freq: f32, sampling_rate: f32, q_factor: f32) -> Self {
        let omega = 2.0 * PI * notch_freq / sampling_rate;
        let alpha = omega.sin() / (2.0 * q_factor);
        
        // 陷波滤波器系数
        let b0 = 1.0;
        let b1 = -2.0 * omega.cos();
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * omega.cos();
        let a2 = 1.0 - alpha;
        
        let coefficients = FilterCoefficients {
            numerator: vec![b0/a0, b1/a0, b2/a0],
            denominator: vec![1.0, a1/a0, a2/a0],
        };
        
        Self {
            filter_type: DigitalFilterType::Notch(notch_freq),
            coefficients,
            delay_line: vec![0.0; 3],
            output_history: vec![0.0; 2],
        }
    }

    pub fn process(&mut self, input: f32) -> f32 {
        // 更新延迟线
        self.delay_line.rotate_right(1);
        self.delay_line[0] = input;
        
        // 计算输出（直接II型结构）
        let mut output = 0.0;
        
        // FIR部分（前馈）
        for (i, &coeff) in self.coefficients.numerator.iter().enumerate() {
            if i < self.delay_line.len() {
                output += coeff * self.delay_line[i];
            }
        }
        
        // IIR部分（反馈）
        for (i, &coeff) in self.coefficients.denominator.iter().skip(1).enumerate() {
            if i < self.output_history.len() {
                output -= coeff * self.output_history[i];
            }
        }
        
        // 更新输出历史
        self.output_history.rotate_right(1);
        self.output_history[0] = output;
        
        output
    }

    fn design_butterworth_lowpass(normalized_cutoff: f32, order: usize) -> FilterCoefficients {
        // 简化的巴特沃斯低通滤波器设计
        // 实际应用中应使用更精确的设计方法
        
        match order {
            1 => {
                let k = (PI * normalized_cutoff).tan();
                let norm = 1.0 + k;
                FilterCoefficients {
                    numerator: vec![k / norm, k / norm],
                    denominator: vec![1.0, (k - 1.0) / norm],
                }
            },
            2 => {
                let k = (PI * normalized_cutoff).tan();
                let norm = 1.0 + k / 0.7071 + k * k;
                FilterCoefficients {
                    numerator: vec![k * k / norm, 2.0 * k * k / norm, k * k / norm],
                    denominator: vec![1.0, (2.0 * (k * k - 1.0)) / norm, (1.0 - k / 0.7071 + k * k) / norm],
                }
            },
            _ => {
                // 高阶滤波器需要更复杂的设计
                FilterCoefficients {
                    numerator: vec![normalized_cutoff, normalized_cutoff],
                    denominator: vec![1.0, normalized_cutoff - 1.0],
                }
            }
        }
    }
}

#[derive(Debug)]
pub struct AveragingFilters {
    pub moving_average: MovingAverageFilter,
    pub exponential_average: ExponentialAverageFilter,
    pub median_filter: MedianFilter,
}

#[derive(Debug)]
pub struct MovingAverageFilter {
    pub window_size: usize,
    pub buffer: Vec<f32>,
    pub index: usize,
    pub sum: f32,
}

impl MovingAverageFilter {
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size,
            buffer: vec![0.0; window_size],
            index: 0,
            sum: 0.0,
        }
    }

    pub fn process(&mut self, input: f32) -> f32 {
        // 移除最老的样本
        self.sum -= self.buffer[self.index];
        
        // 添加新样本
        self.buffer[self.index] = input;
        self.sum += input;
        
        // 更新索引
        self.index = (self.index + 1) % self.window_size;
        
        // 返回平均值
        self.sum / self.window_size as f32
    }
}

#[derive(Debug)]
pub struct AdaptiveFilters {
    pub lms_filter: LMSFilter,
    pub rls_filter: RLSFilter,
    pub kalman_filter: KalmanFilter,
}

#[derive(Debug)]
pub struct LMSFilter {
    pub weights: Vec<f32>,
    pub delay_line: Vec<f32>,
    pub step_size: f32,
    pub leakage_factor: f32,
}

impl LMSFilter {
    pub fn new(filter_length: usize, step_size: f32) -> Self {
        Self {
            weights: vec![0.0; filter_length],
            delay_line: vec![0.0; filter_length],
            step_size,
            leakage_factor: 0.9999, // 防止权重发散
        }
    }

    pub fn adapt(&mut self, input: f32, desired: f32) -> f32 {
        // 更新延迟线
        self.delay_line.rotate_right(1);
        self.delay_line[0] = input;
        
        // 计算滤波器输出
        let output: f32 = self.weights.iter()
            .zip(self.delay_line.iter())
            .map(|(w, x)| w * x)
            .sum();
        
        // 计算误差
        let error = desired - output;
        
        // 更新权重（LMS算法）
        for (weight, &x) in self.weights.iter_mut().zip(self.delay_line.iter()) {
            *weight = *weight * self.leakage_factor + self.step_size * error * x;
        }
        
        output
    }
}
```

## 噪声分析报告

### 1. 噪声分析报告生成

```rust
pub struct NoiseAnalysisReport {
    pub header: ReportHeader,
    pub system_configuration: SystemConfiguration,
    pub noise_measurements: NoiseMeasurements,
    pub noise_sources_analysis: NoiseSourcesAnalysis,
    pub frequency_analysis: FrequencyAnalysis,
    pub performance_metrics: PerformanceMetrics,
    pub recommendations: Vec<NoiseReductionRecommendation>,
    pub conclusions: String,
}

impl NoiseAnalysisReport {
    pub fn generate_comprehensive_report(
        system_model: &SystemNoiseModel,
        measurements: &[NoiseSpectrum],
        snr_measurements: &[SnrMeasurement],
    ) -> Self {
        let header = ReportHeader {
            title: "ADC/DAC系统噪声分析报告".to_string(),
            date: chrono::Utc::now(),
            version: "1.0".to_string(),
            author: "自动噪声分析系统".to_string(),
        };

        let system_config = SystemConfiguration {
            adc_resolution: 12,
            dac_resolution: 12,
            sampling_rate: 100000.0,
            reference_voltage: 3.3,
            temperature: 25.0,
            supply_voltage: 3.3,
        };

        let noise_analysis = system_model.calculate_total_noise();
        
        let noise_measurements = NoiseMeasurements {
            input_referred_noise: measurements.get(0).cloned(),
            output_noise: measurements.get(1).cloned(),
            snr_measurements: snr_measurements.to_vec(),
            total_noise_analysis: noise_analysis,
        };

        let frequency_response = system_model.calculate_frequency_response(
            &Self::generate_analysis_frequencies()
        );

        let frequency_analysis = FrequencyAnalysis {
            frequency_response,
            dominant_noise_frequencies: Self::identify_dominant_frequencies(&measurements),
            noise_floor_analysis: Self::analyze_noise_floor(&measurements),
        };

        let performance_metrics = Self::calculate_performance_metrics(&noise_measurements);
        
        let recommendations = Self::generate_recommendations(&noise_measurements, &frequency_analysis);

        let conclusions = Self::generate_conclusions(&noise_measurements, &performance_metrics);

        Self {
            header,
            system_configuration: system_config,
            noise_measurements,
            noise_sources_analysis: Self::analyze_noise_sources(&noise_analysis),
            frequency_analysis,
            performance_metrics,
            recommendations,
            conclusions,
        }
    }

    fn generate_analysis_frequencies() -> Vec<f32> {
        let mut frequencies = Vec::new();
        
        // 对数分布的频率点
        for i in 0..100 {
            let freq = 10.0_f32.powf(i as f32 / 33.0); // 1Hz到1MHz
            frequencies.push(freq);
        }
        
        frequencies
    }

    fn identify_dominant_frequencies(measurements: &[NoiseSpectrum]) -> Vec<DominantFrequency> {
        let mut dominant_frequencies = Vec::new();
        
        for measurement in measurements {
            let mut peaks = Vec::new();
            
            // 寻找噪声谱中的峰值
            for i in 1..measurement.power_spectral_density.len() - 1 {
                let current = measurement.power_spectral_density[i];
                let prev = measurement.power_spectral_density[i - 1];
                let next = measurement.power_spectral_density[i + 1];
                
                // 检查是否为局部最大值且显著高于周围
                if current > prev && current > next && current > prev * 2.0 && current > next * 2.0 {
                    peaks.push((i, current));
                }
            }
            
            // 按幅度排序，取前5个峰值
            peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
            
            for (i, (bin_index, amplitude)) in peaks.iter().take(5).enumerate() {
                let frequency = measurement.frequency_bins[*bin_index];
                dominant_frequencies.push(DominantFrequency {
                    frequency,
                    amplitude: *amplitude,
                    rank: i + 1,
                    possible_source: Self::identify_noise_source(frequency),
                });
            }
        }
        
        dominant_frequencies
    }

    fn identify_noise_source(frequency: f32) -> String {
        match frequency {
            f if f < 10.0 => "1/f噪声或温度漂移".to_string(),
            f if (f - 50.0).abs() < 1.0 => "工频干扰".to_string(),
            f if (f - 100.0).abs() < 2.0 => "电源纹波".to_string(),
            f if f > 1e6 => "数字开关噪声".to_string(),
            f if f > 1e5 => "时钟噪声".to_string(),
            f if f > 1e3 => "开关电源噪声".to_string(),
            _ => "未知噪声源".to_string(),
        }
    }

    fn generate_recommendations(
        noise_measurements: &NoiseMeasurements,
        frequency_analysis: &FrequencyAnalysis,
    ) -> Vec<NoiseReductionRecommendation> {
        let mut recommendations = Vec::new();
        
        // 基于总噪声水平的建议
        let total_noise_db = 20.0 * noise_measurements.total_noise_analysis.total_noise_voltage_rms.log10();
        
        if total_noise_db > -60.0 {
            recommendations.push(NoiseReductionRecommendation {
                priority: RecommendationPriority::High,
                category: "总体噪声".to_string(),
                description: "系统总噪声过高，建议全面优化".to_string(),
                specific_actions: vec![
                    "检查电源质量和滤波".to_string(),
                    "优化PCB布局和接地".to_string(),
                    "增加屏蔽措施".to_string(),
                ],
                expected_improvement: "10-20dB噪声降低".to_string(),
            });
        }
        
        // 基于主要噪声源的建议
        for contribution in &noise_measurements.total_noise_analysis.noise_breakdown {
            if contribution.percentage > 30.0 {
                let recommendation = match contribution.source_type.as_str() {
                    s if s.contains("PowerSupply") => NoiseReductionRecommendation {
                        priority: RecommendationPriority::High,
                        category: "电源噪声".to_string(),
                        description: format!("电源噪声占总噪声的{:.1}%", contribution.percentage),
                        specific_actions: vec![
                            "增加电源去耦电容".to_string(),
                            "使用低噪声线性稳压器".to_string(),
                            "改善电源滤波".to_string(),
                        ],
                        expected_improvement: "5-15dB噪声降低".to_string(),
                    },
                    s if s.contains("Switching") => NoiseReductionRecommendation {
                        priority: RecommendationPriority::Medium,
                        category: "开关噪声".to_string(),
                        description: format!("开关噪声占总噪声的{:.1}%", contribution.percentage),
                        specific_actions: vec![
                            "优化数字信号布线".to_string(),
                            "增加数字地和模拟地隔离".to_string(),
                            "使用差分信号传输".to_string(),
                        ],
                        expected_improvement: "3-10dB噪声降低".to_string(),
                    },
                    s if s.contains("Thermal") => NoiseReductionRecommendation {
                        priority: RecommendationPriority::Low,
                        category: "热噪声".to_string(),
                        description: format!("热噪声占总噪声的{:.1}%", contribution.percentage),
                        specific_actions: vec![
                            "降低信号源阻抗".to_string(),
                            "使用低噪声运放".to_string(),
                            "优化输入电路设计".to_string(),
                        ],
                        expected_improvement: "2-5dB噪声降低".to_string(),
                    },
                    _ => continue,
                };
                recommendations.push(recommendation);
            }
        }
        
        // 基于频率分析的建议
        for dominant_freq in &frequency_analysis.dominant_noise_frequencies {
            if dominant_freq.rank <= 3 { // 前3个主要频率
                let recommendation = match dominant_freq.possible_source.as_str() {
                    "工频干扰" => NoiseReductionRecommendation {
                        priority: RecommendationPriority::High,
                        category: "工频干扰".to_string(),
                        description: format!("{}Hz处有显著噪声峰值", dominant_freq.frequency),
                        specific_actions: vec![
                            "检查接地系统".to_string(),
                            "增加50Hz陷波滤波器".to_string(),
                            "改善屏蔽".to_string(),
                        ],
                        expected_improvement: "10-30dB特定频率噪声降低".to_string(),
                    },
                    "电源纹波" => NoiseReductionRecommendation {
                        priority: RecommendationPriority::Medium,
                        category: "电源纹波".to_string(),
                        description: format!("{}Hz处有电源纹波", dominant_freq.frequency),
                        specific_actions: vec![
                            "增加电源滤波电容".to_string(),
                            "使用LC滤波器".to_string(),
                            "检查电源稳压器性能".to_string(),
                        ],
                        expected_improvement: "5-20dB纹波降低".to_string(),
                    },
                    _ => continue,
                };
                recommendations.push(recommendation);
            }
        }
        
        recommendations
    }

    pub fn print_report(&self) {
        println!("=== {} ===", self.header.title);
        println!("生成时间: {}", self.header.date.format("%Y-%m-%d %H:%M:%S"));
        println!("版本: {}", self.header.version);
        println!();

        println!("=== 系统配置 ===");
        println!("ADC分辨率: {}位", self.system_configuration.adc_resolution);
        println!("DAC分辨率: {}位", self.system_configuration.dac_resolution);
        println!("采样率: {:.0}Hz", self.system_configuration.sampling_rate);
        println!("参考电压: {:.1}V", self.system_configuration.reference_voltage);
        println!("工作温度: {:.1}°C", self.system_configuration.temperature);
        println!();

        println!("=== 噪声测量结果 ===");
        println!("总噪声电压(RMS): {:.1}μV", 
                self.noise_measurements.total_noise_analysis.total_noise_voltage_rms * 1e6);
        println!("总噪声电压(P-P): {:.1}μV", 
                self.noise_measurements.total_noise_analysis.total_noise_voltage_pp * 1e6);
        println!("信噪比: {:.1}dB", self.noise_measurements.total_noise_analysis.snr_db);
        println!("有效位数: {:.1}位", self.noise_measurements.total_noise_analysis.enob);
        println!();

        println!("=== 主要噪声源分析 ===");
        for (i, contribution) in self.noise_measurements.total_noise_analysis.noise_breakdown.iter().take(5).enumerate() {
            println!("{}. {} - {:.1}% ({:.1}μV RMS)", 
                    i + 1, 
                    contribution.source_type,
                    contribution.percentage,
                    contribution.voltage_rms * 1e6);
        }
        println!();

        println!("=== 主要噪声频率 ===");
        for freq in &self.frequency_analysis.dominant_noise_frequencies {
            println!("{}Hz: {:.1}dB - {}", 
                    freq.frequency,
                    20.0 * freq.amplitude.log10(),
                    freq.possible_source);
        }
        println!();

        println!("=== 改进建议 ===");
        for (i, rec) in self.recommendations.iter().enumerate() {
            println!("{}. [{}] {} - {}", 
                    i + 1,
                    match rec.priority {
                        RecommendationPriority::High => "高优先级",
                        RecommendationPriority::Medium => "中优先级",
                        RecommendationPriority::Low => "低优先级",
                    },
                    rec.category,
                    rec.description);
            
            for action in &rec.specific_actions {
                println!("   - {}", action);
            }
            println!("   预期改善: {}", rec.expected_improvement);
            println!();
        }

        println!("=== 结论 ===");
        println!("{}", self.conclusions);
    }
}

#[derive(Debug)]
pub struct NoiseReductionRecommendation {
    pub priority: RecommendationPriority,
    pub category: String,
    pub description: String,
    pub specific_actions: Vec<String>,
    pub expected_improvement: String,
}

#[derive(