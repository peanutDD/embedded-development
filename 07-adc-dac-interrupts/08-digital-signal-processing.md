# 第8章：数字信号处理基础

## 概述

数字信号处理(DSP)是现代嵌入式系统中处理和分析数字信号的核心技术。本章将深入探讨在STM32F4微控制器上实现的各种DSP算法，包括数字滤波器、快速傅里叶变换(FFT)、信号分析和实时处理技术。

## 学习目标

通过本章学习，您将掌握：

1. **DSP基础理论**：采样定理、Z变换、频域分析、数字滤波器设计
2. **实时滤波器实现**：FIR/IIR滤波器、自适应滤波、多速率处理
3. **频域分析技术**：FFT/IFFT、功率谱密度、频谱分析
4. **信号检测与估计**：相关分析、谱估计、信号分离
5. **STM32F4 DSP优化**：ARM DSP库、硬件加速、内存优化

## DSP基础理论

### 数字信号基础

```rust
use core::f32::consts::PI;
use micromath::F32Ext;

// 数字信号基础结构
#[derive(Debug, Clone)]
pub struct DigitalSignal {
    pub samples: Vec<f32>,
    pub sample_rate: f32,
    pub length: usize,
    pub duration: f32,
}

impl DigitalSignal {
    pub fn new(samples: Vec<f32>, sample_rate: f32) -> Self {
        let length = samples.len();
        let duration = length as f32 / sample_rate;
        
        Self {
            samples,
            sample_rate,
            length,
            duration,
        }
    }
    
    pub fn from_function<F>(sample_rate: f32, duration: f32, func: F) -> Self 
    where
        F: Fn(f32) -> f32,
    {
        let length = (sample_rate * duration) as usize;
        let mut samples = Vec::with_capacity(length);
        
        for i in 0..length {
            let t = i as f32 / sample_rate;
            samples.push(func(t));
        }
        
        Self::new(samples, sample_rate)
    }
    
    pub fn sine_wave(frequency: f32, amplitude: f32, phase: f32, sample_rate: f32, duration: f32) -> Self {
        Self::from_function(sample_rate, duration, |t| {
            amplitude * (2.0 * PI * frequency * t + phase).sin()
        })
    }
    
    pub fn cosine_wave(frequency: f32, amplitude: f32, phase: f32, sample_rate: f32, duration: f32) -> Self {
        Self::from_function(sample_rate, duration, |t| {
            amplitude * (2.0 * PI * frequency * t + phase).cos()
        })
    }
    
    pub fn square_wave(frequency: f32, amplitude: f32, sample_rate: f32, duration: f32) -> Self {
        Self::from_function(sample_rate, duration, |t| {
            let phase = (2.0 * PI * frequency * t) % (2.0 * PI);
            if phase < PI { amplitude } else { -amplitude }
        })
    }
    
    pub fn sawtooth_wave(frequency: f32, amplitude: f32, sample_rate: f32, duration: f32) -> Self {
        Self::from_function(sample_rate, duration, |t| {
            let phase = (frequency * t) % 1.0;
            amplitude * (2.0 * phase - 1.0)
        })
    }
    
    pub fn white_noise(amplitude: f32, sample_rate: f32, duration: f32, seed: u32) -> Self {
        let mut rng = SimpleRng::new(seed);
        Self::from_function(sample_rate, duration, |_| {
            amplitude * (rng.next_f32() * 2.0 - 1.0)
        })
    }
    
    pub fn chirp(start_freq: f32, end_freq: f32, amplitude: f32, sample_rate: f32, duration: f32) -> Self {
        Self::from_function(sample_rate, duration, |t| {
            let freq = start_freq + (end_freq - start_freq) * t / duration;
            amplitude * (2.0 * PI * freq * t).sin()
        })
    }
    
    // 信号基本操作
    pub fn add(&self, other: &DigitalSignal) -> Result<DigitalSignal, DSPError> {
        if self.length != other.length {
            return Err(DSPError::LengthMismatch);
        }
        
        let samples: Vec<f32> = self.samples.iter()
            .zip(other.samples.iter())
            .map(|(&a, &b)| a + b)
            .collect();
        
        Ok(DigitalSignal::new(samples, self.sample_rate))
    }
    
    pub fn multiply(&self, other: &DigitalSignal) -> Result<DigitalSignal, DSPError> {
        if self.length != other.length {
            return Err(DSPError::LengthMismatch);
        }
        
        let samples: Vec<f32> = self.samples.iter()
            .zip(other.samples.iter())
            .map(|(&a, &b)| a * b)
            .collect();
        
        Ok(DigitalSignal::new(samples, self.sample_rate))
    }
    
    pub fn scale(&self, factor: f32) -> DigitalSignal {
        let samples: Vec<f32> = self.samples.iter()
            .map(|&x| x * factor)
            .collect();
        
        DigitalSignal::new(samples, self.sample_rate)
    }
    
    pub fn shift(&self, offset: f32) -> DigitalSignal {
        let samples: Vec<f32> = self.samples.iter()
            .map(|&x| x + offset)
            .collect();
        
        DigitalSignal::new(samples, self.sample_rate)
    }
    
    // 信号统计特性
    pub fn mean(&self) -> f32 {
        self.samples.iter().sum::<f32>() / self.length as f32
    }
    
    pub fn variance(&self) -> f32 {
        let mean = self.mean();
        let sum_squares: f32 = self.samples.iter()
            .map(|&x| (x - mean).powi(2))
            .sum();
        sum_squares / self.length as f32
    }
    
    pub fn std_deviation(&self) -> f32 {
        self.variance().sqrt()
    }
    
    pub fn rms(&self) -> f32 {
        let sum_squares: f32 = self.samples.iter()
            .map(|&x| x * x)
            .sum();
        (sum_squares / self.length as f32).sqrt()
    }
    
    pub fn peak(&self) -> f32 {
        self.samples.iter()
            .fold(0.0f32, |acc, &x| acc.max(x.abs()))
    }
    
    pub fn peak_to_peak(&self) -> f32 {
        let min = self.samples.iter().fold(f32::INFINITY, |acc, &x| acc.min(x));
        let max = self.samples.iter().fold(f32::NEG_INFINITY, |acc, &x| acc.max(x));
        max - min
    }
    
    pub fn crest_factor(&self) -> f32 {
        self.peak() / self.rms()
    }
    
    // 信号能量和功率
    pub fn energy(&self) -> f32 {
        self.samples.iter()
            .map(|&x| x * x)
            .sum()
    }
    
    pub fn power(&self) -> f32 {
        self.energy() / self.length as f32
    }
    
    // 信号窗函数
    pub fn apply_window(&mut self, window_type: WindowType) {
        let window = self.generate_window(window_type);
        for (sample, &window_val) in self.samples.iter_mut().zip(window.iter()) {
            *sample *= window_val;
        }
    }
    
    fn generate_window(&self, window_type: WindowType) -> Vec<f32> {
        let n = self.length;
        let mut window = vec![0.0; n];
        
        match window_type {
            WindowType::Rectangular => {
                window.fill(1.0);
            }
            WindowType::Hanning => {
                for i in 0..n {
                    window[i] = 0.5 * (1.0 - (2.0 * PI * i as f32 / (n - 1) as f32).cos());
                }
            }
            WindowType::Hamming => {
                for i in 0..n {
                    window[i] = 0.54 - 0.46 * (2.0 * PI * i as f32 / (n - 1) as f32).cos();
                }
            }
            WindowType::Blackman => {
                for i in 0..n {
                    let arg = 2.0 * PI * i as f32 / (n - 1) as f32;
                    window[i] = 0.42 - 0.5 * arg.cos() + 0.08 * (2.0 * arg).cos();
                }
            }
            WindowType::Kaiser(beta) => {
                let i0_beta = modified_bessel_i0(beta);
                for i in 0..n {
                    let arg = 2.0 * i as f32 / (n - 1) as f32 - 1.0;
                    let bessel_arg = beta * (1.0 - arg * arg).sqrt();
                    window[i] = modified_bessel_i0(bessel_arg) / i0_beta;
                }
            }
        }
        
        window
    }
}

#[derive(Debug, Clone, Copy)]
pub enum WindowType {
    Rectangular,
    Hanning,
    Hamming,
    Blackman,
    Kaiser(f32), // beta parameter
}

// 简单随机数生成器
struct SimpleRng {
    state: u32,
}

impl SimpleRng {
    fn new(seed: u32) -> Self {
        Self { state: seed }
    }
    
    fn next_u32(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(1103515245).wrapping_add(12345);
        self.state
    }
    
    fn next_f32(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }
}

// 修正贝塞尔函数I0的近似计算
fn modified_bessel_i0(x: f32) -> f32 {
    let ax = x.abs();
    if ax < 3.75 {
        let y = (x / 3.75).powi(2);
        1.0 + y * (3.5156229 + y * (3.0899424 + y * (1.2067492 + 
            y * (0.2659732 + y * (0.0360768 + y * 0.0045813)))))
    } else {
        let y = 3.75 / ax;
        (ax.exp() / ax.sqrt()) * (0.39894228 + y * (0.01328592 + 
            y * (0.00225319 + y * (-0.00157565 + y * (0.00916281 + 
            y * (-0.02057706 + y * (0.02635537 + y * (-0.01647633 + 
            y * 0.00392377))))))))
    }
}

#[derive(Debug)]
pub enum DSPError {
    LengthMismatch,
    InvalidParameter,
    InsufficientData,
    ComputationError,
}
```

## 数字滤波器设计与实现

### FIR滤波器

```rust
// FIR (有限冲激响应) 滤波器
pub struct FIRFilter {
    coefficients: Vec<f32>,
    delay_line: Vec<f32>,
    tap_count: usize,
    index: usize,
}

impl FIRFilter {
    pub fn new(coefficients: Vec<f32>) -> Self {
        let tap_count = coefficients.len();
        let delay_line = vec![0.0; tap_count];
        
        Self {
            coefficients,
            delay_line,
            tap_count,
            index: 0,
        }
    }
    
    // 低通滤波器设计 (窗函数法)
    pub fn lowpass(cutoff_freq: f32, sample_rate: f32, num_taps: usize, window: WindowType) -> Self {
        let mut coefficients = vec![0.0; num_taps];
        let fc = cutoff_freq / sample_rate;
        let center = (num_taps - 1) as f32 / 2.0;
        
        // 理想低通滤波器的冲激响应
        for i in 0..num_taps {
            let n = i as f32 - center;
            coefficients[i] = if n == 0.0 {
                2.0 * fc
            } else {
                (2.0 * PI * fc * n).sin() / (PI * n)
            };
        }
        
        // 应用窗函数
        let window_values = Self::generate_window(num_taps, window);
        for (coeff, &window_val) in coefficients.iter_mut().zip(window_values.iter()) {
            *coeff *= window_val;
        }
        
        // 归一化
        let sum: f32 = coefficients.iter().sum();
        for coeff in &mut coefficients {
            *coeff /= sum;
        }
        
        Self::new(coefficients)
    }
    
    // 高通滤波器设计
    pub fn highpass(cutoff_freq: f32, sample_rate: f32, num_taps: usize, window: WindowType) -> Self {
        let mut lowpass = Self::lowpass(cutoff_freq, sample_rate, num_taps, window);
        
        // 频谱反转：h_hp[n] = δ[n] - h_lp[n]
        for (i, coeff) in lowpass.coefficients.iter_mut().enumerate() {
            if i == (num_taps - 1) / 2 {
                *coeff = 1.0 - *coeff;
            } else {
                *coeff = -*coeff;
            }
        }
        
        lowpass
    }
    
    // 带通滤波器设计
    pub fn bandpass(low_freq: f32, high_freq: f32, sample_rate: f32, num_taps: usize, window: WindowType) -> Self {
        let mut coefficients = vec![0.0; num_taps];
        let fc1 = low_freq / sample_rate;
        let fc2 = high_freq / sample_rate;
        let center = (num_taps - 1) as f32 / 2.0;
        
        for i in 0..num_taps {
            let n = i as f32 - center;
            coefficients[i] = if n == 0.0 {
                2.0 * (fc2 - fc1)
            } else {
                ((2.0 * PI * fc2 * n).sin() - (2.0 * PI * fc1 * n).sin()) / (PI * n)
            };
        }
        
        // 应用窗函数
        let window_values = Self::generate_window(num_taps, window);
        for (coeff, &window_val) in coefficients.iter_mut().zip(window_values.iter()) {
            *coeff *= window_val;
        }
        
        Self::new(coefficients)
    }
    
    // 带阻滤波器设计
    pub fn bandstop(low_freq: f32, high_freq: f32, sample_rate: f32, num_taps: usize, window: WindowType) -> Self {
        let mut bandpass = Self::bandpass(low_freq, high_freq, sample_rate, num_taps, window);
        
        // 频谱反转
        for (i, coeff) in bandpass.coefficients.iter_mut().enumerate() {
            if i == (num_taps - 1) / 2 {
                *coeff = 1.0 - *coeff;
            } else {
                *coeff = -*coeff;
            }
        }
        
        bandpass
    }
    
    fn generate_window(length: usize, window_type: WindowType) -> Vec<f32> {
        let mut window = vec![0.0; length];
        
        match window_type {
            WindowType::Rectangular => {
                window.fill(1.0);
            }
            WindowType::Hanning => {
                for i in 0..length {
                    window[i] = 0.5 * (1.0 - (2.0 * PI * i as f32 / (length - 1) as f32).cos());
                }
            }
            WindowType::Hamming => {
                for i in 0..length {
                    window[i] = 0.54 - 0.46 * (2.0 * PI * i as f32 / (length - 1) as f32).cos();
                }
            }
            WindowType::Blackman => {
                for i in 0..length {
                    let arg = 2.0 * PI * i as f32 / (length - 1) as f32;
                    window[i] = 0.42 - 0.5 * arg.cos() + 0.08 * (2.0 * arg).cos();
                }
            }
            WindowType::Kaiser(beta) => {
                let i0_beta = modified_bessel_i0(beta);
                for i in 0..length {
                    let arg = 2.0 * i as f32 / (length - 1) as f32 - 1.0;
                    let bessel_arg = beta * (1.0 - arg * arg).sqrt();
                    window[i] = modified_bessel_i0(bessel_arg) / i0_beta;
                }
            }
        }
        
        window
    }
    
    // 滤波处理
    pub fn process(&mut self, input: f32) -> f32 {
        // 更新延迟线
        self.delay_line[self.index] = input;
        
        // 计算输出
        let mut output = 0.0;
        for i in 0..self.tap_count {
            let delay_index = (self.index + self.tap_count - i) % self.tap_count;
            output += self.coefficients[i] * self.delay_line[delay_index];
        }
        
        // 更新索引
        self.index = (self.index + 1) % self.tap_count;
        
        output
    }
    
    pub fn process_buffer(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), DSPError> {
        if input.len() != output.len() {
            return Err(DSPError::LengthMismatch);
        }
        
        for (i, &sample) in input.iter().enumerate() {
            output[i] = self.process(sample);
        }
        
        Ok(())
    }
    
    pub fn reset(&mut self) {
        self.delay_line.fill(0.0);
        self.index = 0;
    }
    
    // 频率响应计算
    pub fn frequency_response(&self, frequencies: &[f32], sample_rate: f32) -> Vec<(f32, f32)> {
        let mut response = Vec::with_capacity(frequencies.len());
        
        for &freq in frequencies {
            let omega = 2.0 * PI * freq / sample_rate;
            let mut real = 0.0;
            let mut imag = 0.0;
            
            for (n, &coeff) in self.coefficients.iter().enumerate() {
                let phase = -(n as f32) * omega;
                real += coeff * phase.cos();
                imag += coeff * phase.sin();
            }
            
            let magnitude = (real * real + imag * imag).sqrt();
            let phase = imag.atan2(real);
            
            response.push((magnitude, phase));
        }
        
        response
    }
    
    // 群延迟计算
    pub fn group_delay(&self, frequencies: &[f32], sample_rate: f32) -> Vec<f32> {
        let mut delays = Vec::with_capacity(frequencies.len());
        
        for &freq in frequencies {
            let omega = 2.0 * PI * freq / sample_rate;
            let mut real = 0.0;
            let mut imag = 0.0;
            let mut d_real = 0.0;
            let mut d_imag = 0.0;
            
            for (n, &coeff) in self.coefficients.iter().enumerate() {
                let phase = -(n as f32) * omega;
                let cos_phase = phase.cos();
                let sin_phase = phase.sin();
                
                real += coeff * cos_phase;
                imag += coeff * sin_phase;
                d_real += coeff * (n as f32) * sin_phase;
                d_imag -= coeff * (n as f32) * cos_phase;
            }
            
            let h_mag_sq = real * real + imag * imag;
            let group_delay = if h_mag_sq > 1e-10 {
                (real * d_real + imag * d_imag) / h_mag_sq
            } else {
                0.0
            };
            
            delays.push(group_delay);
        }
        
        delays
    }
}
```

### IIR滤波器

```rust
// IIR (无限冲激响应) 滤波器
pub struct IIRFilter {
    b_coeffs: Vec<f32>, // 前向系数
    a_coeffs: Vec<f32>, // 反馈系数
    x_history: Vec<f32>, // 输入历史
    y_history: Vec<f32>, // 输出历史
    order: usize,
}

impl IIRFilter {
    pub fn new(b_coeffs: Vec<f32>, a_coeffs: Vec<f32>) -> Result<Self, DSPError> {
        if a_coeffs.is_empty() || a_coeffs[0] == 0.0 {
            return Err(DSPError::InvalidParameter);
        }
        
        let order = a_coeffs.len().max(b_coeffs.len()) - 1;
        let x_history = vec![0.0; b_coeffs.len()];
        let y_history = vec![0.0; a_coeffs.len()];
        
        Ok(Self {
            b_coeffs,
            a_coeffs,
            x_history,
            y_history,
            order,
        })
    }
    
    // 一阶低通滤波器 (RC滤波器)
    pub fn first_order_lowpass(cutoff_freq: f32, sample_rate: f32) -> Result<Self, DSPError> {
        let rc = 1.0 / (2.0 * PI * cutoff_freq);
        let dt = 1.0 / sample_rate;
        let alpha = dt / (rc + dt);
        
        let b_coeffs = vec![alpha];
        let a_coeffs = vec![1.0, -(1.0 - alpha)];
        
        Self::new(b_coeffs, a_coeffs)
    }
    
    // 一阶高通滤波器
    pub fn first_order_highpass(cutoff_freq: f32, sample_rate: f32) -> Result<Self, DSPError> {
        let rc = 1.0 / (2.0 * PI * cutoff_freq);
        let dt = 1.0 / sample_rate;
        let alpha = rc / (rc + dt);
        
        let b_coeffs = vec![alpha, -alpha];
        let a_coeffs = vec![1.0, -(1.0 - alpha)];
        
        Self::new(b_coeffs, a_coeffs)
    }
    
    // 二阶低通滤波器 (Butterworth)
    pub fn butterworth_lowpass(cutoff_freq: f32, sample_rate: f32, q: f32) -> Result<Self, DSPError> {
        let omega = 2.0 * PI * cutoff_freq / sample_rate;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega / (2.0 * q);
        
        let b0 = (1.0 - cos_omega) / 2.0;
        let b1 = 1.0 - cos_omega;
        let b2 = (1.0 - cos_omega) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;
        
        let b_coeffs = vec![b0 / a0, b1 / a0, b2 / a0];
        let a_coeffs = vec![1.0, a1 / a0, a2 / a0];
        
        Self::new(b_coeffs, a_coeffs)
    }
    
    // 二阶高通滤波器 (Butterworth)
    pub fn butterworth_highpass(cutoff_freq: f32, sample_rate: f32, q: f32) -> Result<Self, DSPError> {
        let omega = 2.0 * PI * cutoff_freq / sample_rate;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega / (2.0 * q);
        
        let b0 = (1.0 + cos_omega) / 2.0;
        let b1 = -(1.0 + cos_omega);
        let b2 = (1.0 + cos_omega) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;
        
        let b_coeffs = vec![b0 / a0, b1 / a0, b2 / a0];
        let a_coeffs = vec![1.0, a1 / a0, a2 / a0];
        
        Self::new(b_coeffs, a_coeffs)
    }
    
    // 二阶带通滤波器
    pub fn butterworth_bandpass(center_freq: f32, bandwidth: f32, sample_rate: f32) -> Result<Self, DSPError> {
        let omega = 2.0 * PI * center_freq / sample_rate;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega * (bandwidth * (2.0_f32).ln() / 2.0).sinh();
        
        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;
        
        let b_coeffs = vec![b0 / a0, b1 / a0, b2 / a0];
        let a_coeffs = vec![1.0, a1 / a0, a2 / a0];
        
        Self::new(b_coeffs, a_coeffs)
    }
    
    // 陷波滤波器 (Notch Filter)
    pub fn notch_filter(notch_freq: f32, q: f32, sample_rate: f32) -> Result<Self, DSPError> {
        let omega = 2.0 * PI * notch_freq / sample_rate;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega / (2.0 * q);
        
        let b0 = 1.0;
        let b1 = -2.0 * cos_omega;
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;
        
        let b_coeffs = vec![b0 / a0, b1 / a0, b2 / a0];
        let a_coeffs = vec![1.0, a1 / a0, a2 / a0];
        
        Self::new(b_coeffs, a_coeffs)
    }
    
    // 峰值滤波器 (Peaking EQ)
    pub fn peaking_eq(center_freq: f32, gain_db: f32, q: f32, sample_rate: f32) -> Result<Self, DSPError> {
        let omega = 2.0 * PI * center_freq / sample_rate;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let a_gain = (gain_db / 40.0 * (10.0_f32).ln()).exp(); // 10^(gain_db/40)
        let alpha = sin_omega / (2.0 * q);
        
        let b0 = 1.0 + alpha * a_gain;
        let b1 = -2.0 * cos_omega;
        let b2 = 1.0 - alpha * a_gain;
        let a0 = 1.0 + alpha / a_gain;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha / a_gain;
        
        let b_coeffs = vec![b0 / a0, b1 / a0, b2 / a0];
        let a_coeffs = vec![1.0, a1 / a0, a2 / a0];
        
        Self::new(b_coeffs, a_coeffs)
    }
    
    // 滤波处理
    pub fn process(&mut self, input: f32) -> f32 {
        // 更新输入历史
        self.x_history.rotate_right(1);
        self.x_history[0] = input;
        
        // 计算输出
        let mut output = 0.0;
        
        // 前向路径 (FIR部分)
        for (i, &b_coeff) in self.b_coeffs.iter().enumerate() {
            if i < self.x_history.len() {
                output += b_coeff * self.x_history[i];
            }
        }
        
        // 反馈路径 (IIR部分)
        for (i, &a_coeff) in self.a_coeffs.iter().enumerate().skip(1) {
            if i - 1 < self.y_history.len() {
                output -= a_coeff * self.y_history[i - 1];
            }
        }
        
        // 更新输出历史
        self.y_history.rotate_right(1);
        self.y_history[0] = output;
        
        output
    }
    
    pub fn process_buffer(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), DSPError> {
        if input.len() != output.len() {
            return Err(DSPError::LengthMismatch);
        }
        
        for (i, &sample) in input.iter().enumerate() {
            output[i] = self.process(sample);
        }
        
        Ok(())
    }
    
    pub fn reset(&mut self) {
        self.x_history.fill(0.0);
        self.y_history.fill(0.0);
    }
    
    // 稳定性检查
    pub fn is_stable(&self) -> bool {
        // 简化的稳定性检查：检查极点是否在单位圆内
        // 对于二阶系统，检查系数条件
        if self.a_coeffs.len() == 3 {
            let a1 = self.a_coeffs[1];
            let a2 = self.a_coeffs[2];
            
            // 稳定性条件
            a2.abs() < 1.0 && (a1 + a2).abs() < 1.0 && (a1 - a2).abs() < 1.0
        } else {
            // 对于高阶系统，需要更复杂的分析
            true // 简化处理
        }
    }
    
    // 频率响应
    pub fn frequency_response(&self, frequencies: &[f32], sample_rate: f32) -> Vec<(f32, f32)> {
        let mut response = Vec::with_capacity(frequencies.len());
        
        for &freq in frequencies {
            let omega = 2.0 * PI * freq / sample_rate;
            let z_real = omega.cos();
            let z_imag = omega.sin();
            
            // 计算分子 (B(z))
            let mut num_real = 0.0;
            let mut num_imag = 0.0;
            for (k, &b_coeff) in self.b_coeffs.iter().enumerate() {
                let phase = -(k as f32) * omega;
                num_real += b_coeff * phase.cos();
                num_imag += b_coeff * phase.sin();
            }
            
            // 计算分母 (A(z))
            let mut den_real = 0.0;
            let mut den_imag = 0.0;
            for (k, &a_coeff) in self.a_coeffs.iter().enumerate() {
                let phase = -(k as f32) * omega;
                den_real += a_coeff * phase.cos();
                den_imag += a_coeff * phase.sin();
            }
            
            // H(z) = B(z) / A(z)
            let den_mag_sq = den_real * den_real + den_imag * den_imag;
            let h_real = (num_real * den_real + num_imag * den_imag) / den_mag_sq;
            let h_imag = (num_imag * den_real - num_real * den_imag) / den_mag_sq;
            
            let magnitude = (h_real * h_real + h_imag * h_imag).sqrt();
            let phase = h_imag.atan2(h_real);
            
            response.push((magnitude, phase));
        }
        
        response
    }
}
```

## 快速傅里叶变换(FFT)

### FFT实现

```rust
use core::f32::consts::PI;

// 复数结构
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

impl core::ops::Mul<f32> for Complex {
    type Output = Self;
    
    fn mul(self, scalar: f32) -> Self {
        Self {
            real: self.real * scalar,
            imag: self.imag * scalar,
        }
    }
}

// FFT处理器
pub struct FFTProcessor {
    size: usize,
    log2_size: usize,
    twiddle_factors: Vec<Complex>,
    bit_reverse_table: Vec<usize>,
    window: Vec<f32>,
}

impl FFTProcessor {
    pub fn new(size: usize) -> Result<Self, DSPError> {
        if !size.is_power_of_two() || size < 2 {
            return Err(DSPError::InvalidParameter);
        }
        
        let log2_size = size.trailing_zeros() as usize;
        let twiddle_factors = Self::generate_twiddle_factors(size);
        let bit_reverse_table = Self::generate_bit_reverse_table(size, log2_size);
        let window = vec![1.0; size]; // 默认矩形窗
        
        Ok(Self {
            size,
            log2_size,
            twiddle_factors,
            bit_reverse_table,
            window,
        })
    }
    
    fn generate_twiddle_factors(size: usize) -> Vec<Complex> {
        let mut twiddle_factors = Vec::with_capacity(size / 2);
        
        for k in 0..size / 2 {
            let angle = -2.0 * PI * k as f32 / size as f32;
            twiddle_factors.push(Complex::from_polar(1.0, angle));
        }
        
        twiddle_factors
    }
    
    fn generate_bit_reverse_table(size: usize, log2_size: usize) -> Vec<usize> {
        let mut table = vec![0; size];
        
        for i in 0..size {
            let mut reversed = 0;
            let mut temp = i;
            
            for _ in 0..log2_size {
                reversed = (reversed << 1) | (temp & 1);
                temp >>= 1;
            }
            
            table[i] = reversed;
        }
        
        table
    }
    
    pub fn set_window(&mut self, window_type: WindowType) {
        match window_type {
            WindowType::Rectangular => {
                self.window.fill(1.0);
            }
            WindowType::Hanning => {
                for i in 0..self.size {
                    self.window[i] = 0.5 * (1.0 - (2.0 * PI * i as f32 / (self.size - 1) as f32).cos());
                }
            }
            WindowType::Hamming => {
                for i in 0..self.size {
                    self.window[i] = 0.54 - 0.46 * (2.0 * PI * i as f32 / (self.size - 1) as f32).cos();
                }
            }
            WindowType::Blackman => {
                for i in 0..self.size {
                    let arg = 2.0 * PI * i as f32 / (self.size - 1) as f32;
                    self.window[i] = 0.42 - 0.5 * arg.cos() + 0.08 * (2.0 * arg).cos();
                }
            }
            WindowType::Kaiser(beta) => {
                let i0_beta = modified_bessel_i0(beta);
                for i in 0..self.size {
                    let arg = 2.0 * i as f32 / (self.size - 1) as f32 - 1.0;
                    let bessel_arg = beta * (1.0 - arg * arg).sqrt();
                    self.window[i] = modified_bessel_i0(bessel_arg) / i0_beta;
                }
            }
        }
    }
    
    // 前向FFT
    pub fn fft(&self, input: &[f32]) -> Result<Vec<Complex>, DSPError> {
        if input.len() != self.size {
            return Err(DSPError::LengthMismatch);
        }
        
        // 应用窗函数并转换为复数
        let mut data: Vec<Complex> = input.iter()
            .zip(self.window.iter())
            .map(|(&sample, &window_val)| Complex::new(sample * window_val, 0.0))
            .collect();
        
        // 位反转
        for i in 0..self.size {
            let j = self.bit_reverse_table[i];
            if i < j {
                data.swap(i, j);
            }
        }
        
        // Cooley-Tukey FFT算法
        let mut step = 2;
        for _ in 0..self.log2_size {
            let half_step = step / 2;
            
            for i in (0..self.size).step_by(step) {
                for j in 0..half_step {
                    let twiddle_index = j * self.size / step;
                    let twiddle = self.twiddle_factors[twiddle_index];
                    
                    let u = data[i + j];
                    let v = data[i + j + half_step] * twiddle;
                    
                    data[i + j] = u + v;
                    data[i + j + half_step] = u - v;
                }
            }
            
            step *= 2;
        }
        
        Ok(data)
    }
    
    // 逆FFT
    pub fn ifft(&self, input: &[Complex]) -> Result<Vec<f32>, DSPError> {
        if input.len() != self.size {
            return Err(DSPError::LengthMismatch);
        }
        
        // 共轭输入
        let mut data: Vec<Complex> = input.iter()
            .map(|&c| c.conjugate())
            .collect();
        
        // 位反转
        for i in 0..self.size {
            let j = self.bit_reverse_table[i];
            if i < j {
                data.swap(i, j);
            }
        }
        
        // FFT算法 (与前向FFT相同)
        let mut step = 2;
        for _ in 0..self.log2_size {
            let half_step = step / 2;
            
            for i in (0..self.size).step_by(step) {
                for j in 0..half_step {
                    let twiddle_index = j * self.size / step;
                    let twiddle = self.twiddle_factors[twiddle_index];
                    
                    let u = data[i + j];
                    let v = data[i + j + half_step] * twiddle;
                    
                    data[i + j] = u + v;
                    data[i + j + half_step] = u - v;
                }
            }
            
            step *= 2;
        }
        
        // 共轭并归一化
        let scale = 1.0 / self.size as f32;
        let result: Vec<f32> = data.iter()
            .map(|&c| c.conjugate().real * scale)
            .collect();
        
        Ok(result)
    }
    
    // 功率谱密度
    pub fn power_spectrum(&self, input: &[f32]) -> Result<Vec<f32>, DSPError> {
        let fft_result = self.fft(input)?;
        let power_spectrum: Vec<f32> = fft_result.iter()
            .map(|&c| c.magnitude().powi(2))
            .collect();
        
        Ok(power_spectrum)
    }
    
    // 幅度谱
    pub fn magnitude_spectrum(&self, input: &[f32]) -> Result<Vec<f32>, DSPError> {
        let fft_result = self.fft(input)?;
        let magnitude_spectrum: Vec<f32> = fft_result.iter()
            .map(|&c| c.magnitude())
            .collect();
        
        Ok(magnitude_spectrum)
    }
    
    // 相位谱
    pub fn phase_spectrum(&self, input: &[f32]) -> Result<Vec<f32>, DSPError> {
        let fft_result = self.fft(input)?;
        let phase_spectrum: Vec<f32> = fft_result.iter()
            .map(|&c| c.phase())
            .collect();
        
        Ok(phase_spectrum)
    }
    
    // 频率轴生成
    pub fn frequency_axis(&self, sample_rate: f32) -> Vec<f32> {
        (0..self.size)
            .map(|k| k as f32 * sample_rate / self.size as f32)
            .collect()
    }
    
    // 重叠相加卷积
    pub fn overlap_add_convolution(&self, signal: &[f32], impulse_response: &[f32]) -> Result<Vec<f32>, DSPError> {
        if impulse_response.len() > self.size {
            return Err(DSPError::InvalidParameter);
        }
        
        let block_size = self.size - impulse_response.len() + 1;
        let mut result = vec![0.0; signal.len() + impulse_response.len() - 1];
        
        // 计算冲激响应的FFT
        let mut ir_padded = impulse_response.to_vec();
        ir_padded.resize(self.size, 0.0);
        let ir_fft = self.fft(&ir_padded)?;
        
        // 处理每个块
        for (block_start, chunk) in signal.chunks(block_size).enumerate() {
            let mut block_padded = chunk.to_vec();
            block_padded.resize(self.size, 0.0);
            
            // 频域卷积
            let block_fft = self.fft(&block_padded)?;
            let conv_fft: Vec<Complex> = block_fft.iter()
                .zip(ir_fft.iter())
                .map(|(&a, &b)| a * b)
                .collect();
            
            // 逆FFT
            let conv_result = self.ifft(&conv_fft)?;
            
            // 重叠相加
            let output_start = block_start * block_size;
            for (i, &sample) in conv_result.iter().enumerate() {
                if output_start + i < result.len() {
                    result[output_start + i] += sample;
                }
            }
        }
        
        Ok(result)
    }
}
```

## 频谱分析

### 频谱分析器

```rust
// 频谱分析器
pub struct SpectrumAnalyzer {
    fft_processor: FFTProcessor,
    sample_rate: f32,
    window_type: WindowType,
    overlap_ratio: f32,
    averaging_type: AveragingType,
    num_averages: usize,
    spectrum_history: Vec<Vec<f32>>,
    current_average: usize,
}

#[derive(Debug, Clone, Copy)]
pub enum AveragingType {
    None,
    Linear,
    Exponential(f32), // 平滑因子
    PeakHold,
}

impl SpectrumAnalyzer {
    pub fn new(fft_size: usize, sample_rate: f32) -> Result<Self, DSPError> {
        let fft_processor = FFTProcessor::new(fft_size)?;
        
        Ok(Self {
            fft_processor,
            sample_rate,
            window_type: WindowType::Hanning,
            overlap_ratio: 0.5,
            averaging_type: AveragingType::Linear,
            num_averages: 10,
            spectrum_history: Vec::new(),
            current_average: 0,
        })
    }
    
    pub fn set_window(&mut self, window_type: WindowType) {
        self.window_type = window_type;
        self.fft_processor.set_window(window_type);
    }
    
    pub fn set_averaging(&mut self, averaging_type: AveragingType, num_averages: usize) {
        self.averaging_type = averaging_type;
        self.num_averages = num_averages;
        self.spectrum_history.clear();
        self.current_average = 0;
    }
    
    pub fn analyze(&mut self, signal: &[f32]) -> Result<SpectrumResult, DSPError> {
        let fft_size = self.fft_processor.size;
        let hop_size = (fft_size as f32 * (1.0 - self.overlap_ratio)) as usize;
        
        let mut power_spectra = Vec::new();
        
        // 处理重叠的窗口
        for window_start in (0..signal.len().saturating_sub(fft_size)).step_by(hop_size) {
            let window_end = window_start + fft_size;
            if window_end <= signal.len() {
                let window_data = &signal[window_start..window_end];
                let power_spectrum = self.fft_processor.power_spectrum(window_data)?;
                power_spectra.push(power_spectrum);
            }
        }
        
        if power_spectra.is_empty() {
            return Err(DSPError::InsufficientData);
        }
        
        // 平均功率谱
        let averaged_spectrum = self.average_spectra(&power_spectra)?;
        
        // 转换为dB
        let spectrum_db: Vec<f32> = averaged_spectrum.iter()
            .map(|&power| {
                if power > 1e-10 {
                    10.0 * power.log10()
                } else {
                    -100.0 // 最小值限制
                }
            })
            .collect();
        
        // 生成频率轴
        let frequencies = self.fft_processor.frequency_axis(self.sample_rate);
        
        // 查找峰值
        let peaks = self.find_peaks(&spectrum_db, 3.0, 5)?; // 3dB阈值，最小间隔5个bin
        
        // 计算总谐波失真 (THD)
        let thd = self.calculate_thd(&averaged_spectrum, &frequencies)?;
        
        // 计算信噪比 (SNR)
        let snr = self.calculate_snr(&averaged_spectrum, &peaks)?;
        
        Ok(SpectrumResult {
            frequencies,
            magnitude_db: spectrum_db,
            power_spectrum: averaged_spectrum,
            peaks,
            thd_percent: thd * 100.0,
            snr_db: snr,
            fundamental_frequency: self.find_fundamental_frequency(&peaks),
        })
    }
    
    fn average_spectra(&mut self, spectra: &[Vec<f32>]) -> Result<Vec<f32>, DSPError> {
        if spectra.is_empty() {
            return Err(DSPError::InsufficientData);
        }
        
        let spectrum_length = spectra[0].len();
        let mut averaged = vec![0.0; spectrum_length];
        
        match self.averaging_type {
            AveragingType::None => {
                // 使用最后一个频谱
                averaged = spectra.last().unwrap().clone();
            }
            AveragingType::Linear => {
                // 线性平均
                for spectrum in spectra {
                    for (i, &value) in spectrum.iter().enumerate() {
                        averaged[i] += value;
                    }
                }
                
                let scale = 1.0 / spectra.len() as f32;
                for value in &mut averaged {
                    *value *= scale;
                }
                
                // 更新历史记录
                self.spectrum_history.extend_from_slice(spectra);
                if self.spectrum_history.len() > self.num_averages {
                    let excess = self.spectrum_history.len() - self.num_averages;
                    self.spectrum_history.drain(0..excess);
                }
            }
            AveragingType::Exponential(alpha) => {
                // 指数平均
                if self.spectrum_history.is_empty() {
                    self.spectrum_history.push(spectra[0].clone());
                }
                
                let mut current_avg = self.spectrum_history[0].clone();
                for spectrum in spectra {
                    for (i, &new_value) in spectrum.iter().enumerate() {
                        current_avg[i] = alpha * new_value + (1.0 - alpha) * current_avg[i];
                    }
                }
                
                self.spectrum_history[0] = current_avg.clone();
                averaged = current_avg;
            }
            AveragingType::PeakHold => {
                // 峰值保持
                if self.spectrum_history.is_empty() {
                    self.spectrum_history.push(spectra[0].clone());
                }
                
                let mut peak_spectrum = self.spectrum_history[0].clone();
                for spectrum in spectra {
                    for (i, &value) in spectrum.iter().enumerate() {
                        if value > peak_spectrum[i] {
                            peak_spectrum[i] = value;
                        }
                    }
                }
                
                self.spectrum_history[0] = peak_spectrum.clone();
                averaged = peak_spectrum;
            }
        }
        
        Ok(averaged)
    }
    
    fn find_peaks(&self, spectrum_db: &[f32], threshold_db: f32, min_distance: usize) -> Result<Vec<Peak>, DSPError> {
        let mut peaks = Vec::new();
        
        for i in min_distance..spectrum_db.len() - min_distance {
            let current = spectrum_db[i];
            
            // 检查是否为局部最大值
            let mut is_peak = true;
            for j in (i - min_distance)..=(i + min_distance) {
                if j != i && spectrum_db[j] >= current {
                    is_peak = false;
                    break;
                }
            }
            
            // 检查幅度阈值
            if is_peak && current > threshold_db {
                let frequency = i as f32 * self.sample_rate / (2.0 * spectrum_db.len() as f32);
                peaks.push(Peak {
                    frequency,
                    magnitude_db: current,
                    bin_index: i,
                });
            }
        }
        
        // 按幅度排序
        peaks.sort_by(|a, b| b.magnitude_db.partial_cmp(&a.magnitude_db).unwrap());
        
        Ok(peaks)
    }
    
    fn calculate_thd(&self, power_spectrum: &[f32], frequencies: &[f32]) -> Result<f32, DSPError> {
        // 简化的THD计算：假设基频是最强的分量
        if power_spectrum.len() < 10 {
            return Ok(0.0);
        }
        
        // 找到基频
        let mut fundamental_bin = 1; // 跳过DC分量
        let mut max_power = power_spectrum[1];
        
        for (i, &power) in power_spectrum.iter().enumerate().skip(2).take(power_spectrum.len() / 4) {
            if power > max_power {
                max_power = power;
                fundamental_bin = i;
            }
        }
        
        let fundamental_freq = frequencies[fundamental_bin];
        let mut harmonic_power = 0.0;
        
        // 计算谐波功率 (2次到5次谐波)
        for harmonic in 2..=5 {
            let harmonic_freq = fundamental_freq * harmonic as f32;
            let harmonic_bin = (harmonic_freq * power_spectrum.len() as f32 / (self.sample_rate / 2.0)) as usize;
            
            if harmonic_bin < power_spectrum.len() {
                harmonic_power += power_spectrum[harmonic_bin];
            }
        }
        
        if max_power > 1e-10 {
            Ok((harmonic_power / max_power).sqrt())
        } else {
            Ok(0.0)
        }
    }
    
    fn calculate_snr(&self, power_spectrum: &[f32], peaks: &[Peak]) -> Result<f32, DSPError> {
        if peaks.is_empty() {
            return Ok(0.0);
        }
        
        // 信号功率 (最强峰值)
        let signal_power = power_spectrum[peaks[0].bin_index];
        
        // 噪声功率 (总功率减去峰值功率)
        let total_power: f32 = power_spectrum.iter().sum();
        let peak_power: f32 = peaks.iter()
            .take(10) // 考虑前10个峰值
            .map(|peak| power_spectrum[peak.bin_index])
            .sum();
        
        let noise_power = total_power - peak_power;
        
        if noise_power > 1e-10 && signal_power > 1e-10 {
            Ok(10.0 * (signal_power / noise_power).log10())
        } else {
            Ok(100.0) // 高SNR
        }
    }
    
    fn find_fundamental_frequency(&self, peaks: &[Peak]) -> Option<f32> {
        if peaks.is_empty() {
            None
        } else {
            Some(peaks[0].frequency)
        }
    }
    
    // 实时频谱分析
    pub fn real_time_analyze(&mut self, new_samples: &[f32], buffer: &mut Vec<f32>) -> Result<Option<SpectrumResult>, DSPError> {
        let fft_size = self.fft_processor.size;
        
        // 添加新样本到缓冲区
        buffer.extend_from_slice(new_samples);
        
        // 检查是否有足够的数据进行分析
        if buffer.len() >= fft_size {
            let analysis_data = &buffer[buffer.len() - fft_size..];
            let result = self.analyze(analysis_data)?;
            
            // 移除旧数据，保持缓冲区大小
            let hop_size = (fft_size as f32 * (1.0 - self.overlap_ratio)) as usize;
            if buffer.len() > fft_size + hop_size {
                buffer.drain(0..hop_size);
            }
            
            Ok(Some(result))
        } else {
            Ok(None)
        }
    }
}

#[derive(Debug, Clone)]
pub struct SpectrumResult {
    pub frequencies: Vec<f32>,
    pub magnitude_db: Vec<f32>,
    pub power_spectrum: Vec<f32>,
    pub peaks: Vec<Peak>,
    pub thd_percent: f32,
    pub snr_db: f32,
    pub fundamental_frequency: Option<f32>,
}

#[derive(Debug, Clone)]
pub struct Peak {
    pub frequency: f32,
    pub magnitude_db: f32,
    pub bin_index: usize,
}

## 相关分析与信号检测

### 相关分析器

```rust
// 相关分析器
pub struct CorrelationAnalyzer {
    max_lag: usize,
    fft_processor: Option<FFTProcessor>,
}

impl CorrelationAnalyzer {
    pub fn new(max_lag: usize) -> Self {
        // 为快速相关计算准备FFT处理器
        let fft_size = (2 * max_lag).next_power_of_two() * 2;
        let fft_processor = FFTProcessor::new(fft_size).ok();
        
        Self {
            max_lag,
            fft_processor,
        }
    }
    
    // 自相关函数
    pub fn autocorrelation(&self, signal: &[f32]) -> Result<Vec<f32>, DSPError> {
        if signal.len() < self.max_lag {
            return Err(DSPError::InsufficientData);
        }
        
        let mut autocorr = vec![0.0; 2 * self.max_lag + 1];
        let signal_len = signal.len();
        
        // 计算自相关
        for lag in 0..=self.max_lag {
            let mut sum = 0.0;
            let mut count = 0;
            
            for i in 0..(signal_len - lag) {
                sum += signal[i] * signal[i + lag];
                count += 1;
            }
            
            if count > 0 {
                autocorr[self.max_lag + lag] = sum / count as f32;
                if lag > 0 {
                    autocorr[self.max_lag - lag] = autocorr[self.max_lag + lag]; // 对称性
                }
            }
        }
        
        Ok(autocorr)
    }
    
    // 互相关函数
    pub fn cross_correlation(&self, signal1: &[f32], signal2: &[f32]) -> Result<Vec<f32>, DSPError> {
        let min_len = signal1.len().min(signal2.len());
        if min_len < self.max_lag {
            return Err(DSPError::InsufficientData);
        }
        
        let mut crosscorr = vec![0.0; 2 * self.max_lag + 1];
        
        for lag in 0..=self.max_lag {
            // 正延迟
            let mut sum_pos = 0.0;
            let mut count_pos = 0;
            
            for i in 0..(min_len - lag) {
                sum_pos += signal1[i] * signal2[i + lag];
                count_pos += 1;
            }
            
            if count_pos > 0 {
                crosscorr[self.max_lag + lag] = sum_pos / count_pos as f32;
            }
            
            // 负延迟
            if lag > 0 {
                let mut sum_neg = 0.0;
                let mut count_neg = 0;
                
                for i in lag..min_len {
                    sum_neg += signal1[i] * signal2[i - lag];
                    count_neg += 1;
                }
                
                if count_neg > 0 {
                    crosscorr[self.max_lag - lag] = sum_neg / count_neg as f32;
                }
            }
        }
        
        Ok(crosscorr)
    }
    
    // 快速相关计算 (使用FFT)
    pub fn fast_correlation(&self, signal1: &[f32], signal2: &[f32]) -> Result<Vec<f32>, DSPError> {
        let fft_processor = self.fft_processor.as_ref()
            .ok_or(DSPError::ComputationError)?;
        
        let fft_size = fft_processor.size;
        let mut padded1 = signal1.to_vec();
        let mut padded2 = signal2.to_vec();
        
        // 零填充到FFT大小
        padded1.resize(fft_size, 0.0);
        padded2.resize(fft_size, 0.0);
        
        // 计算FFT
        let fft1 = fft_processor.fft(&padded1)?;
        let fft2 = fft_processor.fft(&padded2)?;
        
        // 频域相关：FFT1 * conj(FFT2)
        let correlation_fft: Vec<Complex> = fft1.iter()
            .zip(fft2.iter())
            .map(|(&a, &b)| a * b.conjugate())
            .collect();
        
        // 逆FFT得到相关结果
        let correlation_result = fft_processor.ifft(&correlation_fft)?;
        
        // 提取所需的延迟范围
        let mut result = vec![0.0; 2 * self.max_lag + 1];
        for i in 0..=self.max_lag {
            if i < correlation_result.len() {
                result[self.max_lag + i] = correlation_result[i];
            }
            if i > 0 && fft_size - i < correlation_result.len() {
                result[self.max_lag - i] = correlation_result[fft_size - i];
            }
        }
        
        Ok(result)
    }
    
    // 归一化相关
    pub fn normalized_correlation(&self, signal1: &[f32], signal2: &[f32]) -> Result<Vec<f32>, DSPError> {
        let crosscorr = self.cross_correlation(signal1, signal2)?;
        
        // 计算信号的能量
        let energy1: f32 = signal1.iter().map(|&x| x * x).sum();
        let energy2: f32 = signal2.iter().map(|&x| x * x).sum();
        let normalization = (energy1 * energy2).sqrt();
        
        if normalization > 1e-10 {
            Ok(crosscorr.iter().map(|&x| x / normalization).collect())
        } else {
            Ok(crosscorr)
        }
    }
    
    // 寻找最大相关延迟
    pub fn find_max_correlation_lag(&self, correlation: &[f32]) -> (usize, f32) {
        let mut max_value = correlation[0];
        let mut max_index = 0;
        
        for (i, &value) in correlation.iter().enumerate() {
            if value > max_value {
                max_value = value;
                max_index = i;
            }
        }
        
        let lag = if max_index >= self.max_lag {
            max_index - self.max_lag
        } else {
            self.max_lag - max_index
        };
        
        (lag, max_value)
    }
    
    // 基音检测 (基于自相关)
    pub fn pitch_detection(&self, signal: &[f32], sample_rate: f32, min_freq: f32, max_freq: f32) -> Result<Option<f32>, DSPError> {
        let autocorr = self.autocorrelation(signal)?;
        
        // 计算频率对应的延迟范围
        let min_lag = (sample_rate / max_freq) as usize;
        let max_lag = (sample_rate / min_freq) as usize;
        
        if max_lag >= autocorr.len() / 2 {
            return Ok(None);
        }
        
        // 在指定范围内寻找最大值
        let search_start = self.max_lag + min_lag;
        let search_end = (self.max_lag + max_lag).min(autocorr.len() - 1);
        
        let mut max_value = autocorr[search_start];
        let mut max_lag_index = min_lag;
        
        for i in search_start..=search_end {
            if autocorr[i] > max_value {
                max_value = autocorr[i];
                max_lag_index = i - self.max_lag;
            }
        }
        
        // 检查是否找到有效的基音
        if max_value > 0.3 { // 阈值可调
            let pitch_frequency = sample_rate / max_lag_index as f32;
            Ok(Some(pitch_frequency))
        } else {
            Ok(None)
        }
    }
}
```

## 自适应滤波器

### LMS自适应滤波器

```rust
// LMS (最小均方) 自适应滤波器
pub struct LMSFilter {
    coefficients: Vec<f32>,
    delay_line: Vec<f32>,
    step_size: f32,
    tap_count: usize,
    index: usize,
}

impl LMSFilter {
    pub fn new(tap_count: usize, step_size: f32) -> Self {
        Self {
            coefficients: vec![0.0; tap_count],
            delay_line: vec![0.0; tap_count],
            step_size,
            tap_count,
            index: 0,
        }
    }
    
    // 自适应滤波处理
    pub fn adapt(&mut self, input: f32, desired: f32) -> (f32, f32) {
        // 更新延迟线
        self.delay_line[self.index] = input;
        
        // 计算滤波器输出
        let mut output = 0.0;
        for i in 0..self.tap_count {
            let delay_index = (self.index + self.tap_count - i) % self.tap_count;
            output += self.coefficients[i] * self.delay_line[delay_index];
        }
        
        // 计算误差
        let error = desired - output;
        
        // 更新系数 (LMS算法)
        for i in 0..self.tap_count {
            let delay_index = (self.index + self.tap_count - i) % self.tap_count;
            self.coefficients[i] += self.step_size * error * self.delay_line[delay_index];
        }
        
        // 更新索引
        self.index = (self.index + 1) % self.tap_count;
        
        (output, error)
    }
    
    // 批量处理
    pub fn adapt_batch(&mut self, input: &[f32], desired: &[f32]) -> Result<(Vec<f32>, Vec<f32>), DSPError> {
        if input.len() != desired.len() {
            return Err(DSPError::LengthMismatch);
        }
        
        let mut outputs = Vec::with_capacity(input.len());
        let mut errors = Vec::with_capacity(input.len());
        
        for (&inp, &des) in input.iter().zip(desired.iter()) {
            let (output, error) = self.adapt(inp, des);
            outputs.push(output);
            errors.push(error);
        }
        
        Ok((outputs, errors))
    }
    
    // 重置滤波器
    pub fn reset(&mut self) {
        self.coefficients.fill(0.0);
        self.delay_line.fill(0.0);
        self.index = 0;
    }
    
    // 设置步长
    pub fn set_step_size(&mut self, step_size: f32) {
        self.step_size = step_size;
    }
    
    // 获取系数
    pub fn get_coefficients(&self) -> &[f32] {
        &self.coefficients
    }
    
    // 计算均方误差
    pub fn mean_square_error(&self, input: &[f32], desired: &[f32]) -> Result<f32, DSPError> {
        if input.len() != desired.len() {
            return Err(DSPError::LengthMismatch);
        }
        
        let mut mse = 0.0;
        let mut temp_filter = self.clone();
        
        for (&inp, &des) in input.iter().zip(desired.iter()) {
            let (output, _) = temp_filter.adapt(inp, des);
            let error = des - output;
            mse += error * error;
        }
        
        Ok(mse / input.len() as f32)
    }
}

impl Clone for LMSFilter {
    fn clone(&self) -> Self {
        Self {
            coefficients: self.coefficients.clone(),
            delay_line: self.delay_line.clone(),
            step_size: self.step_size,
            tap_count: self.tap_count,
            index: self.index,
        }
    }
}
```

## 实时DSP处理系统

### 实时处理器

```rust
// 实时DSP处理器
pub struct RealTimeDSPProcessor {
    sample_rate: f32,
    buffer_size: usize,
    input_buffer: Vec<f32>,
    output_buffer: Vec<f32>,
    processing_chain: Vec<Box<dyn DSPProcessor>>,
    latency_samples: usize,
}

pub trait DSPProcessor {
    fn process(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), DSPError>;
    fn reset(&mut self);
    fn get_latency(&self) -> usize;
}

impl RealTimeDSPProcessor {
    pub fn new(sample_rate: f32, buffer_size: usize) -> Self {
        Self {
            sample_rate,
            buffer_size,
            input_buffer: vec![0.0; buffer_size],
            output_buffer: vec![0.0; buffer_size],
            processing_chain: Vec::new(),
            latency_samples: 0,
        }
    }
    
    pub fn add_processor(&mut self, processor: Box<dyn DSPProcessor>) {
        self.latency_samples += processor.get_latency();
        self.processing_chain.push(processor);
    }
    
    pub fn process_block(&mut self, input: &[f32]) -> Result<Vec<f32>, DSPError> {
        if input.len() != self.buffer_size {
            return Err(DSPError::LengthMismatch);
        }
        
        // 复制输入到工作缓冲区
        self.input_buffer.copy_from_slice(input);
        
        // 依次通过处理链
        for processor in &mut self.processing_chain {
            processor.process(&self.input_buffer, &mut self.output_buffer)?;
            self.input_buffer.copy_from_slice(&self.output_buffer);
        }
        
        Ok(self.output_buffer.clone())
    }
    
    pub fn get_total_latency_ms(&self) -> f32 {
        (self.latency_samples as f32 / self.sample_rate) * 1000.0
    }
    
    pub fn reset_all(&mut self) {
        for processor in &mut self.processing_chain {
            processor.reset();
        }
    }
}

// FIR滤波器处理器实现
impl DSPProcessor for FIRFilter {
    fn process(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), DSPError> {
        self.process_buffer(input, output)
    }
    
    fn reset(&mut self) {
        self.reset();
    }
    
    fn get_latency(&self) -> usize {
        self.tap_count / 2 // 群延迟
    }
}

// IIR滤波器处理器实现
impl DSPProcessor for IIRFilter {
    fn process(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), DSPError> {
        self.process_buffer(input, output)
    }
    
    fn reset(&mut self) {
        self.reset();
    }
    
    fn get_latency(&self) -> usize {
        0 // IIR滤波器没有固定延迟
    }
}
```

## 性能优化与最佳实践

### 内存优化

```rust
// 内存池管理器
pub struct DSPMemoryPool {
    pools: Vec<Vec<Vec<f32>>>,
    pool_sizes: Vec<usize>,
}

impl DSPMemoryPool {
    pub fn new() -> Self {
        let pool_sizes = vec![64, 128, 256, 512, 1024, 2048, 4096];
        let mut pools = Vec::with_capacity(pool_sizes.len());
        
        for &size in &pool_sizes {
            let mut pool = Vec::new();
            // 预分配一些缓冲区
            for _ in 0..4 {
                pool.push(vec![0.0; size]);
            }
            pools.push(pool);
        }
        
        Self { pools, pool_sizes }
    }
    
    pub fn get_buffer(&mut self, size: usize) -> Option<Vec<f32>> {
        for (i, &pool_size) in self.pool_sizes.iter().enumerate() {
            if size <= pool_size {
                if let Some(buffer) = self.pools[i].pop() {
                    return Some(buffer);
                }
                break;
            }
        }
        
        // 如果池中没有可用缓冲区，创建新的
        Some(vec![0.0; size])
    }
    
    pub fn return_buffer(&mut self, mut buffer: Vec<f32>) {
        let size = buffer.len();
        
        for (i, &pool_size) in self.pool_sizes.iter().enumerate() {
            if size <= pool_size {
                buffer.clear();
                buffer.resize(pool_size, 0.0);
                
                if self.pools[i].len() < 8 { // 限制池大小
                    self.pools[i].push(buffer);
                }
                break;
            }
        }
    }
}
```

## 总结

本章深入介绍了数字信号处理的核心概念和实现技术：

1. **DSP基础理论**：数字信号表示、基本操作、统计特性分析
2. **数字滤波器**：FIR/IIR滤波器设计与实现，各种滤波器类型
3. **频域分析**：FFT算法实现、频谱分析、功率谱密度计算
4. **信号检测**：相关分析、基音检测、信号特征提取
5. **自适应处理**：LMS自适应滤波器、实时参数调整
6. **实时系统**：实时DSP处理架构、延迟管理、性能优化

通过这些技术的掌握，您可以在STM32F4平台上实现复杂的信号处理算法，为各种应用场景提供强大的数字信号处理能力。

## 参考资料

### 技术文档
- STM32F4 DSP库参考手册
- ARM CMSIS-DSP库文档
- 数字信号处理理论与应用

### 学术资源
- "Digital Signal Processing" by Alan V. Oppenheim
- "Understanding Digital Signal Processing" by Richard G. Lyons
- IEEE信号处理学会期刊

### 开源项目
- CMSIS-DSP库源码
- SciPy信号处理模块
- GNU Radio项目