#![no_std]

use micromath::F32Ext;

/// 复数结构体
#[derive(Debug, Clone, Copy)]
pub struct Complex {
    pub real: f32,
    pub imag: f32,
}

impl Complex {
    pub fn new(real: f32, imag: f32) -> Self {
        Self { real, imag }
    }
    
    pub fn from_polar(magnitude: f32, phase: f32) -> Self {
        Self {
            real: magnitude * phase.cos(),
            imag: magnitude * phase.sin(),
        }
    }
    
    pub fn magnitude(&self) -> f32 {
        (self.real * self.real + self.imag * self.imag).sqrt()
    }
    
    pub fn phase(&self) -> f32 {
        self.imag.atan2(self.real)
    }
    
    pub fn conjugate(&self) -> Self {
        Self {
            real: self.real,
            imag: -self.imag,
        }
    }
}

impl core::ops::Add for Complex {
    type Output = Self;
    
    fn add(self, other: Self) -> Self {
        Self {
            real: self.real + other.real,
            imag: self.imag + other.imag,
        }
    }
}

impl core::ops::Sub for Complex {
    type Output = Self;
    
    fn sub(self, other: Self) -> Self {
        Self {
            real: self.real - other.real,
            imag: self.imag - other.imag,
        }
    }
}

impl core::ops::Mul for Complex {
    type Output = Self;
    
    fn mul(self, other: Self) -> Self {
        Self {
            real: self.real * other.real - self.imag * other.imag,
            imag: self.real * other.imag + self.imag * other.real,
        }
    }
}

/// FFT处理器
pub struct FFTProcessor<const N: usize> {
    twiddle_factors: [Complex; N],
    bit_reverse_table: [usize; N],
    window: [f32; N],
}

impl<const N: usize> FFTProcessor<N> {
    pub fn new() -> Self {
        let mut processor = Self {
            twiddle_factors: [Complex::new(0.0, 0.0); N],
            bit_reverse_table: [0; N],
            window: [1.0; N],
        };
        
        processor.init_twiddle_factors();
        processor.init_bit_reverse_table();
        processor.init_hanning_window();
        
        processor
    }
    
    fn init_twiddle_factors(&mut self) {
        for k in 0..N {
            let angle = -2.0 * core::f32::consts::PI * k as f32 / N as f32;
            self.twiddle_factors[k] = Complex::new(angle.cos(), angle.sin());
        }
    }
    
    fn init_bit_reverse_table(&mut self) {
        let log2n = (N as f32).log2() as usize;
        for i in 0..N {
            let mut reversed = 0;
            let mut temp = i;
            for _ in 0..log2n {
                reversed = (reversed << 1) | (temp & 1);
                temp >>= 1;
            }
            self.bit_reverse_table[i] = reversed;
        }
    }
    
    fn init_hanning_window(&mut self) {
        for i in 0..N {
            let angle = 2.0 * core::f32::consts::PI * i as f32 / (N - 1) as f32;
            self.window[i] = 0.5 * (1.0 - angle.cos());
        }
    }
    
    pub fn apply_window(&self, data: &mut [f32; N]) {
        for i in 0..N {
            data[i] *= self.window[i];
        }
    }
    
    pub fn fft(&self, input: &[f32; N], output: &mut [Complex; N]) {
        // 位反转重排
        for i in 0..N {
            let j = self.bit_reverse_table[i];
            output[i] = Complex::new(input[j], 0.0);
        }
        
        // Cooley-Tukey FFT算法
        let mut size = 2;
        while size <= N {
            let half_size = size / 2;
            let step = N / size;
            
            for i in (0..N).step_by(size) {
                for j in 0..half_size {
                    let u = output[i + j];
                    let v = output[i + j + half_size] * self.twiddle_factors[j * step];
                    
                    output[i + j] = u + v;
                    output[i + j + half_size] = u - v;
                }
            }
            
            size *= 2;
        }
    }
    
    pub fn ifft(&self, input: &[Complex; N], output: &mut [f32; N]) {
        let mut temp = [Complex::new(0.0, 0.0); N];
        
        // 共轭输入
        for i in 0..N {
            temp[i] = input[i].conjugate();
        }
        
        // 执行FFT
        let mut fft_output = [Complex::new(0.0, 0.0); N];
        self.fft_complex(&temp, &mut fft_output);
        
        // 共轭结果并归一化
        for i in 0..N {
            output[i] = fft_output[i].conjugate().real / N as f32;
        }
    }
    
    fn fft_complex(&self, input: &[Complex; N], output: &mut [Complex; N]) {
        // 位反转重排
        for i in 0..N {
            let j = self.bit_reverse_table[i];
            output[i] = input[j];
        }
        
        // Cooley-Tukey FFT算法
        let mut size = 2;
        while size <= N {
            let half_size = size / 2;
            let step = N / size;
            
            for i in (0..N).step_by(size) {
                for j in 0..half_size {
                    let u = output[i + j];
                    let v = output[i + j + half_size] * self.twiddle_factors[j * step];
                    
                    output[i + j] = u + v;
                    output[i + j + half_size] = u - v;
                }
            }
            
            size *= 2;
        }
    }
    
    pub fn power_spectrum(&self, fft_output: &[Complex; N], power: &mut [f32; N]) {
        for i in 0..N {
            power[i] = fft_output[i].magnitude().powi(2);
        }
    }
    
    pub fn magnitude_spectrum(&self, fft_output: &[Complex; N], magnitude: &mut [f32; N]) {
        for i in 0..N {
            magnitude[i] = fft_output[i].magnitude();
        }
    }
    
    pub fn phase_spectrum(&self, fft_output: &[Complex; N], phase: &mut [f32; N]) {
        for i in 0..N {
            phase[i] = fft_output[i].phase();
        }
    }
}

/// 频谱分析器
pub struct SpectrumAnalyzer<const N: usize> {
    fft_processor: FFTProcessor<N>,
    sample_rate: f32,
    frequency_bins: [f32; N],
    peak_threshold: f32,
    noise_floor: f32,
}

impl<const N: usize> SpectrumAnalyzer<N> {
    pub fn new(sample_rate: f32) -> Self {
        let mut analyzer = Self {
            fft_processor: FFTProcessor::new(),
            sample_rate,
            frequency_bins: [0.0; N],
            peak_threshold: 0.1,
            noise_floor: 0.001,
        };
        
        analyzer.init_frequency_bins();
        analyzer
    }
    
    fn init_frequency_bins(&mut self) {
        let freq_resolution = self.sample_rate / N as f32;
        for i in 0..N {
            self.frequency_bins[i] = i as f32 * freq_resolution;
        }
    }
    
    pub fn set_peak_threshold(&mut self, threshold: f32) {
        self.peak_threshold = threshold;
    }
    
    pub fn set_noise_floor(&mut self, noise_floor: f32) {
        self.noise_floor = noise_floor;
    }
    
    pub fn analyze(&self, signal: &mut [f32; N]) -> SpectrumResult<N> {
        // 应用窗函数
        self.fft_processor.apply_window(signal);
        
        // 执行FFT
        let mut fft_output = [Complex::new(0.0, 0.0); N];
        self.fft_processor.fft(signal, &mut fft_output);
        
        // 计算功率谱
        let mut power_spectrum = [0.0f32; N];
        self.fft_processor.power_spectrum(&fft_output, &mut power_spectrum);
        
        // 计算幅度谱
        let mut magnitude_spectrum = [0.0f32; N];
        self.fft_processor.magnitude_spectrum(&fft_output, &mut magnitude_spectrum);
        
        // 查找峰值
        let peaks = self.find_peaks(&magnitude_spectrum);
        
        // 计算总功率
        let total_power = power_spectrum.iter().sum::<f32>();
        
        // 计算信噪比
        let snr = self.calculate_snr(&magnitude_spectrum);
        
        // 计算总谐波失真
        let thd = self.calculate_thd(&peaks);
        
        SpectrumResult {
            power_spectrum,
            magnitude_spectrum,
            frequency_bins: self.frequency_bins,
            peaks,
            total_power,
            snr,
            thd,
            fundamental_frequency: self.find_fundamental_frequency(&peaks),
        }
    }
    
    fn find_peaks(&self, magnitude: &[f32; N]) -> heapless::Vec<Peak, 32> {
        let mut peaks = heapless::Vec::new();
        
        // 只分析前半部分频谱（奈奎斯特频率以下）
        let half_n = N / 2;
        
        for i in 1..half_n-1 {
            if magnitude[i] > self.peak_threshold &&
               magnitude[i] > magnitude[i-1] &&
               magnitude[i] > magnitude[i+1] {
                
                let peak = Peak {
                    frequency: self.frequency_bins[i],
                    magnitude: magnitude[i],
                    bin_index: i,
                };
                
                if peaks.push(peak).is_err() {
                    break; // 峰值数组已满
                }
            }
        }
        
        // 按幅度排序
        peaks.sort_by(|a, b| b.magnitude.partial_cmp(&a.magnitude).unwrap_or(core::cmp::Ordering::Equal));
        
        peaks
    }
    
    fn calculate_snr(&self, magnitude: &[f32; N]) -> f32 {
        let half_n = N / 2;
        
        // 找到最大信号
        let max_signal = magnitude[1..half_n].iter()
            .fold(0.0f32, |max, &val| max.max(val));
        
        // 估算噪声功率（排除峰值）
        let mut noise_samples = heapless::Vec::<f32, 256>::new();
        for i in 1..half_n {
            if magnitude[i] < max_signal * 0.1 { // 假设小于最大信号10%的为噪声
                let _ = noise_samples.push(magnitude[i]);
            }
        }
        
        if noise_samples.is_empty() {
            return 100.0; // 无噪声情况
        }
        
        let noise_power: f32 = noise_samples.iter().sum::<f32>() / noise_samples.len() as f32;
        
        if noise_power > 0.0 {
            20.0 * (max_signal / noise_power).log10()
        } else {
            100.0
        }
    }
    
    fn calculate_thd(&self, peaks: &heapless::Vec<Peak, 32>) -> f32 {
        if peaks.len() < 2 {
            return 0.0;
        }
        
        let fundamental = peaks[0].magnitude;
        let mut harmonic_power = 0.0f32;
        
        // 计算谐波功率
        for i in 1..peaks.len() {
            // 检查是否为基频的谐波
            let freq_ratio = peaks[i].frequency / peaks[0].frequency;
            if (freq_ratio - freq_ratio.round()).abs() < 0.1 {
                harmonic_power += peaks[i].magnitude * peaks[i].magnitude;
            }
        }
        
        if fundamental > 0.0 {
            (harmonic_power.sqrt() / fundamental * 100.0).min(100.0)
        } else {
            0.0
        }
    }
    
    fn find_fundamental_frequency(&self, peaks: &heapless::Vec<Peak, 32>) -> f32 {
        if peaks.is_empty() {
            return 0.0;
        }
        peaks[0].frequency
    }
}

/// 频谱分析结果
#[derive(Debug)]
pub struct SpectrumResult<const N: usize> {
    pub power_spectrum: [f32; N],
    pub magnitude_spectrum: [f32; N],
    pub frequency_bins: [f32; N],
    pub peaks: heapless::Vec<Peak, 32>,
    pub total_power: f32,
    pub snr: f32,
    pub thd: f32,
    pub fundamental_frequency: f32,
}

/// 峰值信息
#[derive(Debug, Clone, Copy)]
pub struct Peak {
    pub frequency: f32,
    pub magnitude: f32,
    pub bin_index: usize,
}

/// 数字滤波器
pub struct DigitalFilter {
    filter_type: FilterType,
    coefficients: FilterCoefficients,
    delay_line: [f32; 8], // 最大8阶滤波器
    order: usize,
}

#[derive(Debug, Clone, Copy)]
pub enum FilterType {
    LowPass,
    HighPass,
    BandPass,
    BandStop,
    Notch,
}

#[derive(Debug, Clone)]
pub struct FilterCoefficients {
    pub b: heapless::Vec<f32, 8>, // 前向系数
    pub a: heapless::Vec<f32, 8>, // 反馈系数
}

impl DigitalFilter {
    pub fn new_butterworth_lowpass(cutoff_freq: f32, sample_rate: f32, order: usize) -> Self {
        let mut filter = Self {
            filter_type: FilterType::LowPass,
            coefficients: FilterCoefficients {
                b: heapless::Vec::new(),
                a: heapless::Vec::new(),
            },
            delay_line: [0.0; 8],
            order: order.min(8),
        };
        
        filter.design_butterworth_lowpass(cutoff_freq, sample_rate);
        filter
    }
    
    pub fn new_butterworth_highpass(cutoff_freq: f32, sample_rate: f32, order: usize) -> Self {
        let mut filter = Self {
            filter_type: FilterType::HighPass,
            coefficients: FilterCoefficients {
                b: heapless::Vec::new(),
                a: heapless::Vec::new(),
            },
            delay_line: [0.0; 8],
            order: order.min(8),
        };
        
        filter.design_butterworth_highpass(cutoff_freq, sample_rate);
        filter
    }
    
    pub fn new_notch(notch_freq: f32, sample_rate: f32, q_factor: f32) -> Self {
        let mut filter = Self {
            filter_type: FilterType::Notch,
            coefficients: FilterCoefficients {
                b: heapless::Vec::new(),
                a: heapless::Vec::new(),
            },
            delay_line: [0.0; 8],
            order: 2,
        };
        
        filter.design_notch(notch_freq, sample_rate, q_factor);
        filter
    }
    
    fn design_butterworth_lowpass(&mut self, cutoff_freq: f32, sample_rate: f32) {
        let omega = 2.0 * core::f32::consts::PI * cutoff_freq / sample_rate;
        let omega_tan = (omega / 2.0).tan();
        
        // 简化的一阶巴特沃斯低通滤波器
        let k = omega_tan;
        let norm = 1.0 + k;
        
        let _ = self.coefficients.b.push(k / norm);
        let _ = self.coefficients.b.push(k / norm);
        let _ = self.coefficients.a.push(1.0);
        let _ = self.coefficients.a.push((k - 1.0) / norm);
    }
    
    fn design_butterworth_highpass(&mut self, cutoff_freq: f32, sample_rate: f32) {
        let omega = 2.0 * core::f32::consts::PI * cutoff_freq / sample_rate;
        let omega_tan = (omega / 2.0).tan();
        
        // 简化的一阶巴特沃斯高通滤波器
        let k = omega_tan;
        let norm = 1.0 + k;
        
        let _ = self.coefficients.b.push(1.0 / norm);
        let _ = self.coefficients.b.push(-1.0 / norm);
        let _ = self.coefficients.a.push(1.0);
        let _ = self.coefficients.a.push((k - 1.0) / norm);
    }
    
    fn design_notch(&mut self, notch_freq: f32, sample_rate: f32, q_factor: f32) {
        let omega = 2.0 * core::f32::consts::PI * notch_freq / sample_rate;
        let alpha = omega.sin() / (2.0 * q_factor);
        let cos_omega = omega.cos();
        
        // 陷波滤波器系数
        let norm = 1.0 + alpha;
        
        let _ = self.coefficients.b.push(1.0 / norm);
        let _ = self.coefficients.b.push(-2.0 * cos_omega / norm);
        let _ = self.coefficients.b.push(1.0 / norm);
        let _ = self.coefficients.a.push(1.0);
        let _ = self.coefficients.a.push(-2.0 * cos_omega / norm);
        let _ = self.coefficients.a.push((1.0 - alpha) / norm);
    }
    
    pub fn process(&mut self, input: f32) -> f32 {
        // 移动延迟线
        for i in (1..self.delay_line.len()).rev() {
            self.delay_line[i] = self.delay_line[i-1];
        }
        self.delay_line[0] = input;
        
        // 计算输出
        let mut output = 0.0f32;
        
        // 前向路径
        for (i, &coeff) in self.coefficients.b.iter().enumerate() {
            if i < self.delay_line.len() {
                output += coeff * self.delay_line[i];
            }
        }
        
        // 反馈路径（跳过a[0]，因为它总是1.0）
        for (i, &coeff) in self.coefficients.a.iter().skip(1).enumerate() {
            let delay_index = i + 1;
            if delay_index < self.delay_line.len() {
                output -= coeff * self.delay_line[delay_index];
            }
        }
        
        output
    }
    
    pub fn process_block(&mut self, input: &[f32], output: &mut [f32]) {
        for (i, &sample) in input.iter().enumerate() {
            if i < output.len() {
                output[i] = self.process(sample);
            }
        }
    }
    
    pub fn reset(&mut self) {
        self.delay_line.fill(0.0);
    }
    
    pub fn get_frequency_response(&self, frequency: f32, sample_rate: f32) -> Complex {
        let omega = 2.0 * core::f32::consts::PI * frequency / sample_rate;
        let z = Complex::new(omega.cos(), -omega.sin());
        
        // 计算分子多项式
        let mut numerator = Complex::new(0.0, 0.0);
        let mut z_power = Complex::new(1.0, 0.0);
        for &coeff in self.coefficients.b.iter() {
            numerator = numerator + Complex::new(coeff, 0.0) * z_power;
            z_power = z_power * z;
        }
        
        // 计算分母多项式
        let mut denominator = Complex::new(0.0, 0.0);
        z_power = Complex::new(1.0, 0.0);
        for &coeff in self.coefficients.a.iter() {
            denominator = denominator + Complex::new(coeff, 0.0) * z_power;
            z_power = z_power * z;
        }
        
        // 返回频率响应
        if denominator.magnitude() > 1e-10 {
            Complex::new(
                (numerator.real * denominator.real + numerator.imag * denominator.imag) / 
                (denominator.real * denominator.real + denominator.imag * denominator.imag),
                (numerator.imag * denominator.real - numerator.real * denominator.imag) / 
                (denominator.real * denominator.real + denominator.imag * denominator.imag)
            )
        } else {
            Complex::new(0.0, 0.0)
        }
    }
}

/// 谐波分析器
pub struct HarmonicAnalyzer<const N: usize> {
    spectrum_analyzer: SpectrumAnalyzer<N>,
    harmonic_threshold: f32,
    max_harmonics: usize,
}

impl<const N: usize> HarmonicAnalyzer<N> {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            spectrum_analyzer: SpectrumAnalyzer::new(sample_rate),
            harmonic_threshold: 0.01,
            max_harmonics: 10,
        }
    }
    
    pub fn set_harmonic_threshold(&mut self, threshold: f32) {
        self.harmonic_threshold = threshold;
    }
    
    pub fn set_max_harmonics(&mut self, max_harmonics: usize) {
        self.max_harmonics = max_harmonics;
    }
    
    pub fn analyze_harmonics(&self, signal: &mut [f32; N]) -> HarmonicResult {
        let spectrum_result = self.spectrum_analyzer.analyze(signal);
        
        if spectrum_result.peaks.is_empty() {
            return HarmonicResult {
                fundamental_frequency: 0.0,
                fundamental_magnitude: 0.0,
                harmonics: heapless::Vec::new(),
                thd: 0.0,
                thd_plus_noise: 0.0,
            };
        }
        
        let fundamental = spectrum_result.peaks[0];
        let mut harmonics = heapless::Vec::new();
        let mut harmonic_power = 0.0f32;
        
        // 查找谐波
        for harmonic_order in 2..=self.max_harmonics {
            let expected_freq = fundamental.frequency * harmonic_order as f32;
            let tolerance = fundamental.frequency * 0.1; // 10%容差
            
            // 在峰值中查找匹配的谐波
            for peak in spectrum_result.peaks.iter().skip(1) {
                if (peak.frequency - expected_freq).abs() < tolerance &&
                   peak.magnitude > self.harmonic_threshold {
                    
                    let harmonic = Harmonic {
                        order: harmonic_order,
                        frequency: peak.frequency,
                        magnitude: peak.magnitude,
                        phase: 0.0, // 简化版本，不计算相位
                        relative_magnitude: peak.magnitude / fundamental.magnitude,
                    };
                    
                    harmonic_power += peak.magnitude * peak.magnitude;
                    
                    if harmonics.push(harmonic).is_err() {
                        break;
                    }
                    break;
                }
            }
        }
        
        // 计算THD
        let thd = if fundamental.magnitude > 0.0 {
            (harmonic_power.sqrt() / fundamental.magnitude * 100.0).min(100.0)
        } else {
            0.0
        };
        
        HarmonicResult {
            fundamental_frequency: fundamental.frequency,
            fundamental_magnitude: fundamental.magnitude,
            harmonics,
            thd,
            thd_plus_noise: spectrum_result.thd,
        }
    }
}

/// 谐波分析结果
#[derive(Debug)]
pub struct HarmonicResult {
    pub fundamental_frequency: f32,
    pub fundamental_magnitude: f32,
    pub harmonics: heapless::Vec<Harmonic, 16>,
    pub thd: f32,
    pub thd_plus_noise: f32,
}

/// 谐波信息
#[derive(Debug, Clone, Copy)]
pub struct Harmonic {
    pub order: usize,
    pub frequency: f32,
    pub magnitude: f32,
    pub phase: f32,
    pub relative_magnitude: f32,
}

/// 实时分析器
pub struct RealTimeAnalyzer<const N: usize> {
    spectrum_analyzer: SpectrumAnalyzer<N>,
    harmonic_analyzer: HarmonicAnalyzer<N>,
    buffer: [f32; N],
    buffer_index: usize,
    analysis_ready: bool,
    overlap_factor: usize,
    hop_size: usize,
}

impl<const N: usize> RealTimeAnalyzer<N> {
    pub fn new(sample_rate: f32, overlap_factor: usize) -> Self {
        let hop_size = N / overlap_factor;
        
        Self {
            spectrum_analyzer: SpectrumAnalyzer::new(sample_rate),
            harmonic_analyzer: HarmonicAnalyzer::new(sample_rate),
            buffer: [0.0; N],
            buffer_index: 0,
            analysis_ready: false,
            overlap_factor,
            hop_size,
        }
    }
    
    pub fn add_sample(&mut self, sample: f32) -> bool {
        self.buffer[self.buffer_index] = sample;
        self.buffer_index += 1;
        
        if self.buffer_index >= self.hop_size {
            // 移动缓冲区
            for i in 0..(N - self.hop_size) {
                self.buffer[i] = self.buffer[i + self.hop_size];
            }
            
            self.buffer_index = N - self.hop_size;
            self.analysis_ready = true;
            return true;
        }
        
        false
    }
    
    pub fn is_analysis_ready(&self) -> bool {
        self.analysis_ready
    }
    
    pub fn analyze(&mut self) -> Option<RealTimeResult<N>> {
        if !self.analysis_ready {
            return None;
        }
        
        let mut analysis_buffer = self.buffer;
        let spectrum_result = self.spectrum_analyzer.analyze(&mut analysis_buffer);
        
        let mut harmonic_buffer = self.buffer;
        let harmonic_result = self.harmonic_analyzer.analyze_harmonics(&mut harmonic_buffer);
        
        self.analysis_ready = false;
        
        Some(RealTimeResult {
            spectrum: spectrum_result,
            harmonics: harmonic_result,
            timestamp: 0, // 简化版本，不包含时间戳
        })
    }
    
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.buffer_index = 0;
        self.analysis_ready = false;
    }
}

/// 实时分析结果
#[derive(Debug)]
pub struct RealTimeResult<const N: usize> {
    pub spectrum: SpectrumResult<N>,
    pub harmonics: HarmonicResult,
    pub timestamp: u32,
}

/// ADC值转换为电压
pub fn adc_to_voltage(adc_value: u16, vref: f32, resolution: u8) -> f32 {
    let max_value = (1 << resolution) - 1;
    (adc_value as f32 / max_value as f32) * vref
}

/// 电压转换为ADC值
pub fn voltage_to_adc(voltage: f32, vref: f32, resolution: u8) -> u16 {
    let max_value = (1 << resolution) - 1;
    let adc_value = (voltage / vref * max_value as f32).round() as u16;
    adc_value.min(max_value)
}