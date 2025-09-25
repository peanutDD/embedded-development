# 滤波技术分析文档

## 概述

滤波技术是ADC/DAC系统中降低噪声、提高信号质量的重要手段。本文档详细介绍STM32F4系列微控制器ADC/DAC系统中的各种滤波技术，包括模拟滤波器、数字滤波器的设计、实现和优化方法。

## 滤波器基础理论

### 1. 滤波器分类和特性

```rust
use core::f32::consts::PI;

#[derive(Debug, Clone)]
pub enum FilterType {
    LowPass,
    HighPass,
    BandPass,
    BandStop,
    AllPass,
}

#[derive(Debug, Clone)]
pub enum FilterResponse {
    Butterworth,    // 最平坦通带
    Chebyshev1(f32), // 通带纹波 (dB)
    Chebyshev2(f32), // 阻带纹波 (dB)
    Elliptic(f32, f32), // 通带纹波, 阻带纹波 (dB)
    Bessel,         // 线性相位
    Gaussian(f32),  // 高斯响应，参数为BT积
}

#[derive(Debug, Clone)]
pub struct FilterSpecification {
    pub filter_type: FilterType,
    pub response_type: FilterResponse,
    pub cutoff_frequency: f32,      // Hz
    pub stopband_frequency: f32,    // Hz
    pub passband_ripple: f32,       // dB
    pub stopband_attenuation: f32,  // dB
    pub sampling_rate: f32,         // Hz (数字滤波器)
    pub order: Option<u8>,          // 滤波器阶数
}

impl FilterSpecification {
    pub fn calculate_minimum_order(&self) -> u8 {
        match &self.response_type {
            FilterResponse::Butterworth => {
                let wp = 2.0 * PI * self.cutoff_frequency;
                let ws = 2.0 * PI * self.stopband_frequency;
                let ratio = ws / wp;
                
                let numerator = (10.0_f32.powf(self.stopband_attenuation / 10.0) - 1.0).log10();
                let denominator = 2.0 * ratio.log10();
                
                (numerator / denominator).ceil() as u8
            },
            FilterResponse::Chebyshev1(ripple) => {
                let epsilon = (10.0_f32.powf(ripple / 10.0) - 1.0).sqrt();
                let wp = 2.0 * PI * self.cutoff_frequency;
                let ws = 2.0 * PI * self.stopband_frequency;
                let ratio = ws / wp;
                
                let numerator = (10.0_f32.powf(self.stopband_attenuation / 10.0) / epsilon).acosh();
                let denominator = ratio.acosh();
                
                (numerator / denominator).ceil() as u8
            },
            FilterResponse::Chebyshev2(ripple) => {
                let epsilon = 1.0 / (10.0_f32.powf(ripple / 10.0) - 1.0).sqrt();
                let wp = 2.0 * PI * self.cutoff_frequency;
                let ws = 2.0 * PI * self.stopband_frequency;
                let ratio = wp / ws;
                
                let numerator = (10.0_f32.powf(self.stopband_attenuation / 10.0) * epsilon).acosh();
                let denominator = ratio.acosh();
                
                (numerator / denominator).ceil() as u8
            },
            _ => 4, // 默认4阶
        }
    }

    pub fn calculate_group_delay(&self, frequency: f32) -> f32 {
        match &self.response_type {
            FilterResponse::Bessel => {
                // 贝塞尔滤波器具有近似恒定的群延迟
                let order = self.order.unwrap_or(4) as f32;
                order / (2.0 * PI * self.cutoff_frequency)
            },
            FilterResponse::Butterworth => {
                // 巴特沃斯滤波器的群延迟
                let order = self.order.unwrap_or(4) as f32;
                let omega = 2.0 * PI * frequency;
                let omega_c = 2.0 * PI * self.cutoff_frequency;
                let ratio = omega / omega_c;
                
                (order - 1.0) / (2.0 * omega_c) * (1.0 + ratio.powi(2 * order as i32)).sqrt()
            },
            _ => {
                // 其他类型的近似计算
                let order = self.order.unwrap_or(4) as f32;
                order / (4.0 * PI * self.cutoff_frequency)
            }
        }
    }
}
```

### 2. 模拟滤波器设计

```rust
#[derive(Debug)]
pub struct AnalogFilter {
    pub specification: FilterSpecification,
    pub topology: FilterTopology,
    pub components: Vec<FilterComponent>,
    pub transfer_function: TransferFunction,
}

#[derive(Debug, Clone)]
pub enum FilterTopology {
    SallenKey,          // Sallen-Key拓扑
    Multiplefeedback,   // 多重反馈拓扑
    StateVariable,      // 状态变量拓扑
    Biquad,            // 双二次拓扑
    Ladder,            // 梯形拓扑
    Switched,          // 开关电容滤波器
}

#[derive(Debug, Clone)]
pub struct FilterComponent {
    pub component_type: ComponentType,
    pub value: f32,
    pub tolerance: f32,
    pub temperature_coefficient: f32,
    pub parasitic_effects: ParasiticEffects,
}

#[derive(Debug, Clone)]
pub enum ComponentType {
    Resistor(f32),      // 阻值 (Ω)
    Capacitor(f32),     // 容值 (F)
    Inductor(f32),      // 感值 (H)
    OpAmp(OpAmpModel),  // 运放模型
}

#[derive(Debug, Clone)]
pub struct OpAmpModel {
    pub gain_bandwidth_product: f32,  // GBW (Hz)
    pub slew_rate: f32,              // 转换速率 (V/μs)
    pub input_offset_voltage: f32,    // 输入失调电压 (V)
    pub input_bias_current: f32,      // 输入偏置电流 (A)
    pub noise_voltage_density: f32,   // 电压噪声密度 (nV/√Hz)
    pub noise_current_density: f32,   // 电流噪声密度 (pA/√Hz)
}

#[derive(Debug, Clone)]
pub struct ParasiticEffects {
    pub series_resistance: f32,    // 串联电阻 (Ω)
    pub parallel_capacitance: f32, // 并联电容 (F)
    pub series_inductance: f32,    // 串联电感 (H)
}

#[derive(Debug, Clone)]
pub struct TransferFunction {
    pub numerator: Vec<f32>,    // 分子多项式系数
    pub denominator: Vec<f32>,  // 分母多项式系数
}

impl AnalogFilter {
    pub fn design_sallen_key_lowpass(spec: &FilterSpecification) -> Self {
        let order = spec.order.unwrap_or(2);
        let mut components = Vec::new();
        
        match order {
            2 => {
                // 二阶Sallen-Key低通滤波器
                let fc = spec.cutoff_frequency;
                let q = match &spec.response_type {
                    FilterResponse::Butterworth => 0.707,
                    FilterResponse::Chebyshev1(ripple) => {
                        (10.0_f32.powf(ripple / 10.0) - 1.0).sqrt() / 2.0
                    },
                    _ => 0.707,
                };
                
                // 选择标准电容值
                let c1 = 10e-9; // 10nF
                let c2 = c1;    // 相等电容简化设计
                
                // 计算电阻值
                let k = 1.0; // 增益
                let r1 = 1.0 / (2.0 * PI * fc * c1 * q);
                let r2 = q / (2.0 * PI * fc * c1);
                
                components.push(FilterComponent {
                    component_type: ComponentType::Resistor(r1),
                    value: r1,
                    tolerance: 0.01, // 1%
                    temperature_coefficient: 100e-6, // 100ppm/°C
                    parasitic_effects: ParasiticEffects {
                        series_resistance: 0.0,
                        parallel_capacitance: 0.1e-12, // 0.1pF
                        series_inductance: 1e-9, // 1nH
                    },
                });
                
                components.push(FilterComponent {
                    component_type: ComponentType::Resistor(r2),
                    value: r2,
                    tolerance: 0.01,
                    temperature_coefficient: 100e-6,
                    parasitic_effects: ParasiticEffects {
                        series_resistance: 0.0,
                        parallel_capacitance: 0.1e-12,
                        series_inductance: 1e-9,
                    },
                });
                
                components.push(FilterComponent {
                    component_type: ComponentType::Capacitor(c1),
                    value: c1,
                    tolerance: 0.05, // 5%
                    temperature_coefficient: 30e-6, // 30ppm/°C (C0G)
                    parasitic_effects: ParasiticEffects {
                        series_resistance: 0.01, // 10mΩ ESR
                        parallel_capacitance: 0.0,
                        series_inductance: 0.5e-9, // 0.5nH ESL
                    },
                });
                
                components.push(FilterComponent {
                    component_type: ComponentType::Capacitor(c2),
                    value: c2,
                    tolerance: 0.05,
                    temperature_coefficient: 30e-6,
                    parasitic_effects: ParasiticEffects {
                        series_resistance: 0.01,
                        parallel_capacitance: 0.0,
                        series_inductance: 0.5e-9,
                    },
                });
                
                // 运放
                components.push(FilterComponent {
                    component_type: ComponentType::OpAmp(OpAmpModel {
                        gain_bandwidth_product: 10e6, // 10MHz
                        slew_rate: 10.0, // 10V/μs
                        input_offset_voltage: 1e-3, // 1mV
                        input_bias_current: 100e-12, // 100pA
                        noise_voltage_density: 10e-9, // 10nV/√Hz
                        noise_current_density: 1e-12, // 1pA/√Hz
                    }),
                    value: 1.0,
                    tolerance: 0.0,
                    temperature_coefficient: 0.0,
                    parasitic_effects: ParasiticEffects {
                        series_resistance: 0.0,
                        parallel_capacitance: 5e-12, // 5pF输入电容
                        series_inductance: 0.0,
                    },
                });
                
                // 传递函数 H(s) = K / (s²/(ωc²) + s/(Q*ωc) + 1)
                let wc = 2.0 * PI * fc;
                let transfer_function = TransferFunction {
                    numerator: vec![k * wc * wc],
                    denominator: vec![1.0, wc / q, wc * wc],
                };
                
                Self {
                    specification: spec.clone(),
                    topology: FilterTopology::SallenKey,
                    components,
                    transfer_function,
                }
            },
            _ => {
                // 高阶滤波器需要级联多个二阶段
                panic!("高阶Sallen-Key滤波器需要级联实现");
            }
        }
    }

    pub fn calculate_frequency_response(&self, frequencies: &[f32]) -> FrequencyResponse {
        let mut magnitude = Vec::new();
        let mut phase = Vec::new();
        let mut group_delay = Vec::new();
        
        for &freq in frequencies {
            let s = Complex::new(0.0, 2.0 * PI * freq);
            let h = self.evaluate_transfer_function(s);
            
            magnitude.push(20.0 * h.norm().log10());
            phase.push(h.arg() * 180.0 / PI);
            
            // 群延迟计算（数值微分）
            let df = freq * 0.001; // 0.1%的频率增量
            let s_plus = Complex::new(0.0, 2.0 * PI * (freq + df));
            let h_plus = self.evaluate_transfer_function(s_plus);
            let phase_derivative = (h_plus.arg() - h.arg()) / (2.0 * PI * df);
            group_delay.push(-phase_derivative);
        }
        
        FrequencyResponse {
            frequencies: frequencies.to_vec(),
            magnitude,
            phase,
            group_delay,
        }
    }

    fn evaluate_transfer_function(&self, s: Complex<f32>) -> Complex<f32> {
        let mut numerator = Complex::new(0.0, 0.0);
        let mut denominator = Complex::new(0.0, 0.0);
        
        // 计算分子多项式
        for (i, &coeff) in self.transfer_function.numerator.iter().enumerate() {
            numerator += Complex::new(coeff, 0.0) * s.powi(i as i32);
        }
        
        // 计算分母多项式
        for (i, &coeff) in self.transfer_function.denominator.iter().enumerate() {
            denominator += Complex::new(coeff, 0.0) * s.powi(i as i32);
        }
        
        numerator / denominator
    }

    pub fn analyze_sensitivity(&self) -> SensitivityAnalysis {
        let mut component_sensitivities = Vec::new();
        
        for (i, component) in self.components.iter().enumerate() {
            let nominal_response = self.calculate_frequency_response(&[self.specification.cutoff_frequency]);
            
            // 计算组件变化对频率响应的影响
            let delta = component.value * 0.01; // 1%变化
            let mut modified_filter = self.clone();
            modified_filter.components[i].value += delta;
            
            let modified_response = modified_filter.calculate_frequency_response(&[self.specification.cutoff_frequency]);
            
            let sensitivity = (modified_response.magnitude[0] - nominal_response.magnitude[0]) / 
                             (delta / component.value);
            
            component_sensitivities.push(ComponentSensitivity {
                component_index: i,
                component_type: component.component_type.clone(),
                magnitude_sensitivity: sensitivity,
                phase_sensitivity: (modified_response.phase[0] - nominal_response.phase[0]) / 
                                  (delta / component.value),
            });
        }
        
        SensitivityAnalysis {
            component_sensitivities,
            worst_case_deviation: self.calculate_worst_case_deviation(),
            monte_carlo_results: self.run_monte_carlo_analysis(1000),
        }
    }

    fn calculate_worst_case_deviation(&self) -> f32 {
        // 最坏情况偏差分析
        let mut max_deviation = 0.0;
        
        for component in &self.components {
            let tolerance_effect = match &component.component_type {
                ComponentType::Resistor(_) | ComponentType::Capacitor(_) => {
                    component.tolerance * 100.0 // 转换为百分比
                },
                _ => 0.0,
            };
            
            max_deviation += tolerance_effect;
        }
        
        max_deviation
    }

    fn run_monte_carlo_analysis(&self, iterations: usize) -> MonteCarloResults {
        let mut cutoff_variations = Vec::new();
        let mut gain_variations = Vec::new();
        
        for _ in 0..iterations {
            let mut modified_filter = self.clone();
            
            // 随机变化组件值
            for component in &mut modified_filter.components {
                let random_factor = 1.0 + (rand::random::<f32>() - 0.5) * 2.0 * component.tolerance;
                component.value *= random_factor;
            }
            
            // 计算变化后的响应
            let response = modified_filter.calculate_frequency_response(&[self.specification.cutoff_frequency]);
            cutoff_variations.push(self.specification.cutoff_frequency); // 简化
            gain_variations.push(response.magnitude[0]);
        }
        
        MonteCarloResults {
            cutoff_mean: cutoff_variations.iter().sum::<f32>() / cutoff_variations.len() as f32,
            cutoff_std: self.calculate_std_dev(&cutoff_variations),
            gain_mean: gain_variations.iter().sum::<f32>() / gain_variations.len() as f32,
            gain_std: self.calculate_std_dev(&gain_variations),
            yield_estimate: self.calculate_yield(&gain_variations),
        }
    }

    fn calculate_std_dev(&self, data: &[f32]) -> f32 {
        let mean = data.iter().sum::<f32>() / data.len() as f32;
        let variance = data.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / data.len() as f32;
        variance.sqrt()
    }

    fn calculate_yield(&self, data: &[f32]) -> f32 {
        let spec_min = -3.0; // -3dB最小增益
        let spec_max = 1.0;   // +1dB最大增益
        
        let within_spec = data.iter().filter(|&&x| x >= spec_min && x <= spec_max).count();
        within_spec as f32 / data.len() as f32 * 100.0
    }
}

// 复数类型定义（简化版本）
#[derive(Debug, Clone, Copy)]
pub struct Complex<T> {
    pub re: T,
    pub im: T,
}

impl<T> Complex<T>
where
    T: Copy + core::ops::Add<Output = T> + core::ops::Sub<Output = T> + 
       core::ops::Mul<Output = T> + core::ops::Div<Output = T>,
{
    pub fn new(re: T, im: T) -> Self {
        Self { re, im }
    }
}

impl Complex<f32> {
    pub fn norm(&self) -> f32 {
        (self.re * self.re + self.im * self.im).sqrt()
    }
    
    pub fn arg(&self) -> f32 {
        self.im.atan2(self.re)
    }
    
    pub fn powi(&self, n: i32) -> Self {
        // 简化的幂运算实现
        let r = self.norm();
        let theta = self.arg();
        let r_n = r.powi(n);
        let theta_n = theta * n as f32;
        
        Self {
            re: r_n * theta_n.cos(),
            im: r_n * theta_n.sin(),
        }
    }
}

impl core::ops::Add for Complex<f32> {
    type Output = Self;
    
    fn add(self, other: Self) -> Self {
        Self {
            re: self.re + other.re,
            im: self.im + other.im,
        }
    }
}

impl core::ops::AddAssign for Complex<f32> {
    fn add_assign(&mut self, other: Self) {
        self.re += other.re;
        self.im += other.im;
    }
}

impl core::ops::Mul for Complex<f32> {
    type Output = Self;
    
    fn mul(self, other: Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
}

impl core::ops::Div for Complex<f32> {
    type Output = Self;
    
    fn div(self, other: Self) -> Self {
        let denom = other.re * other.re + other.im * other.im;
        Self {
            re: (self.re * other.re + self.im * other.im) / denom,
            im: (self.im * other.re - self.re * other.im) / denom,
        }
    }
}
```

### 3. 数字滤波器设计

```rust
#[derive(Debug)]
pub struct DigitalFilter {
    pub specification: FilterSpecification,
    pub implementation: DigitalImplementation,
    pub coefficients: FilterCoefficients,
    pub state_variables: Vec<f32>,
    pub performance_metrics: PerformanceMetrics,
}

#[derive(Debug, Clone)]
pub enum DigitalImplementation {
    DirectForm1,        // 直接I型
    DirectForm2,        // 直接II型
    TransposedForm2,    // 转置II型
    CascadedBiquads,    // 级联双二次
    ParallelForm,       // 并联形式
    LatticeForm,        // 格型结构
}

#[derive(Debug, Clone)]
pub struct FilterCoefficients {
    pub numerator: Vec<f32>,    // b系数 (FIR部分)
    pub denominator: Vec<f32>,  // a系数 (IIR部分)
    pub quantization_bits: u8,  // 系数量化位数
    pub scaling_factors: Vec<f32>, // 缩放因子
}

#[derive(Debug)]
pub struct PerformanceMetrics {
    pub passband_ripple: f32,      // 通带纹波 (dB)
    pub stopband_attenuation: f32, // 阻带衰减 (dB)
    pub transition_width: f32,     // 过渡带宽度 (Hz)
    pub group_delay_variation: f32, // 群延迟变化 (samples)
    pub computational_complexity: u32, // 计算复杂度 (MIPS)
    pub memory_usage: u32,         // 内存使用 (bytes)
}

impl DigitalFilter {
    pub fn design_fir_filter(spec: &FilterSpecification) -> Self {
        let fs = spec.sampling_rate;
        let fc = spec.cutoff_frequency;
        let transition_width = spec.stopband_frequency - spec.cutoff_frequency;
        
        // 估算滤波器长度
        let filter_length = Self::estimate_fir_length(
            spec.stopband_attenuation,
            transition_width / fs
        );
        
        // 设计滤波器系数
        let coefficients = match spec.filter_type {
            FilterType::LowPass => Self::design_fir_lowpass(fc, fs, filter_length),
            FilterType::HighPass => Self::design_fir_highpass(fc, fs, filter_length),
            FilterType::BandPass => Self::design_fir_bandpass(fc, spec.stopband_frequency, fs, filter_length),
            FilterType::BandStop => Self::design_fir_bandstop(fc, spec.stopband_frequency, fs, filter_length),
            _ => panic!("不支持的FIR滤波器类型"),
        };
        
        // 应用窗函数
        let windowed_coefficients = Self::apply_window_function(
            &coefficients,
            &WindowFunction::Kaiser(Self::calculate_kaiser_beta(spec.stopband_attenuation))
        );
        
        let filter_coeffs = FilterCoefficients {
            numerator: windowed_coefficients,
            denominator: vec![1.0], // FIR滤波器分母为1
            quantization_bits: 16,
            scaling_factors: vec![1.0],
        };
        
        Self {
            specification: spec.clone(),
            implementation: DigitalImplementation::DirectForm1,
            coefficients: filter_coeffs,
            state_variables: vec![0.0; filter_length],
            performance_metrics: PerformanceMetrics {
                passband_ripple: 0.1,
                stopband_attenuation: spec.stopband_attenuation,
                transition_width,
                group_delay_variation: 0.5,
                computational_complexity: filter_length as u32,
                memory_usage: filter_length as u32 * 4, // 4 bytes per float
            },
        }
    }

    pub fn design_iir_filter(spec: &FilterSpecification) -> Self {
        let order = spec.calculate_minimum_order();
        
        // 设计模拟原型滤波器
        let analog_poles = Self::calculate_analog_poles(&spec.response_type, order);
        let analog_zeros = Self::calculate_analog_zeros(&spec.response_type, order);
        
        // 双线性变换到数字域
        let (digital_poles, digital_zeros) = Self::bilinear_transform(
            &analog_poles,
            &analog_zeros,
            spec.cutoff_frequency,
            spec.sampling_rate,
        );
        
        // 计算滤波器系数
        let coefficients = Self::poles_zeros_to_coefficients(&digital_poles, &digital_zeros);
        
        // 选择实现结构
        let implementation = if order <= 2 {
            DigitalImplementation::DirectForm2
        } else {
            DigitalImplementation::CascadedBiquads
        };
        
        Self {
            specification: spec.clone(),
            implementation,
            coefficients,
            state_variables: vec![0.0; order as usize * 2],
            performance_metrics: PerformanceMetrics {
                passband_ripple: spec.passband_ripple,
                stopband_attenuation: spec.stopband_attenuation,
                transition_width: spec.stopband_frequency - spec.cutoff_frequency,
                group_delay_variation: 2.0,
                computational_complexity: order as u32 * 5, // 每个双二次段5次乘法
                memory_usage: order as u32 * 8, // 每个双二次段8个状态变量
            },
        }
    }

    pub fn process_sample(&mut self, input: f32) -> f32 {
        match self.implementation {
            DigitalImplementation::DirectForm1 => self.process_direct_form1(input),
            DigitalImplementation::DirectForm2 => self.process_direct_form2(input),
            DigitalImplementation::CascadedBiquads => self.process_cascaded_biquads(input),
            _ => input, // 其他实现形式的占位符
        }
    }

    fn process_direct_form1(&mut self, input: f32) -> f32 {
        // 直接I型结构实现
        // y[n] = Σ(b[k] * x[n-k]) - Σ(a[k] * y[n-k])
        
        // 更新输入延迟线
        self.state_variables.rotate_right(1);
        self.state_variables[0] = input;
        
        // 计算FIR部分
        let mut output = 0.0;
        for (i, &coeff) in self.coefficients.numerator.iter().enumerate() {
            if i < self.state_variables.len() {
                output += coeff * self.state_variables[i];
            }
        }
        
        // IIR部分（如果存在）
        if self.coefficients.denominator.len() > 1 {
            // 这里需要额外的输出历史状态
            // 简化实现，实际需要分离输入和输出状态
        }
        
        output
    }

    fn process_direct_form2(&mut self, input: f32) -> f32 {
        // 直接II型结构实现（更节省内存）
        let n = self.coefficients.denominator.len() - 1;
        
        // 计算中间变量 w[n]
        let mut w = input;
        for i in 1..=n {
            if i <= self.state_variables.len() {
                w -= self.coefficients.denominator[i] * self.state_variables[i - 1];
            }
        }
        
        // 计算输出
        let mut output = self.coefficients.numerator[0] * w;
        for i in 1..self.coefficients.numerator.len() {
            if i <= self.state_variables.len() {
                output += self.coefficients.numerator[i] * self.state_variables[i - 1];
            }
        }
        
        // 更新状态变量
        self.state_variables.rotate_right(1);
        self.state_variables[0] = w;
        
        output
    }

    fn process_cascaded_biquads(&mut self, input: f32) -> f32 {
        // 级联双二次段实现
        let mut signal = input;
        let biquad_count = self.coefficients.numerator.len() / 3;
        
        for i in 0..biquad_count {
            let b0 = self.coefficients.numerator[i * 3];
            let b1 = self.coefficients.numerator[i * 3 + 1];
            let b2 = self.coefficients.numerator[i * 3 + 2];
            let a1 = self.coefficients.denominator[i * 3 + 1];
            let a2 = self.coefficients.denominator[i * 3 + 2];
            
            let state_offset = i * 2;
            let w1 = self.state_variables[state_offset];
            let w2 = self.state_variables[state_offset + 1];
            
            // 双二次段计算
            let w0 = signal - a1 * w1 - a2 * w2;
            signal = b0 * w0 + b1 * w1 + b2 * w2;
            
            // 更新状态
            self.state_variables[state_offset + 1] = w1;
            self.state_variables[state_offset] = w0;
        }
        
        signal
    }

    fn estimate_fir_length(stopband_attenuation: f32, normalized_transition_width: f32) -> usize {
        // Kaiser窗FIR滤波器长度估算
        let beta = Self::calculate_kaiser_beta(stopband_attenuation);
        let length = ((stopband_attenuation - 7.95) / (2.285 * 2.0 * PI * normalized_transition_width)).ceil();
        (length as usize).max(3)
    }

    fn calculate_kaiser_beta(stopband_attenuation: f32) -> f32 {
        if stopband_attenuation > 50.0 {
            0.1102 * (stopband_attenuation - 8.7)
        } else if stopband_attenuation >= 21.0 {
            0.5842 * (stopband_attenuation - 21.0).powf(0.4) + 0.07886 * (stopband_attenuation - 21.0)
        } else {
            0.0
        }
    }

    fn design_fir_lowpass(cutoff: f32, sampling_rate: f32, length: usize) -> Vec<f32> {
        let mut coefficients = Vec::with_capacity(length);
        let wc = 2.0 * PI * cutoff / sampling_rate;
        let center = (length - 1) as f32 / 2.0;
        
        for n in 0..length {
            let index = n as f32 - center;
            let coeff = if index == 0.0 {
                wc / PI
            } else {
                (wc * index).sin() / (PI * index)
            };
            coefficients.push(coeff);
        }
        
        coefficients
    }

    fn design_fir_highpass(cutoff: f32, sampling_rate: f32, length: usize) -> Vec<f32> {
        let mut lowpass = Self::design_fir_lowpass(cutoff, sampling_rate, length);
        
        // 高通 = 全通 - 低通
        let center = (length - 1) / 2;
        for (i, coeff) in lowpass.iter_mut().enumerate() {
            if i == center {
                *coeff = 1.0 - *coeff;
            } else {
                *coeff = -*coeff;
            }
        }
        
        lowpass
    }

    fn design_fir_bandpass(low_cutoff: f32, high_cutoff: f32, sampling_rate: f32, length: usize) -> Vec<f32> {
        let mut coefficients = Vec::with_capacity(length);
        let wc1 = 2.0 * PI * low_cutoff / sampling_rate;
        let wc2 = 2.0 * PI * high_cutoff / sampling_rate;
        let center = (length - 1) as f32 / 2.0;
        
        for n in 0..length {
            let index = n as f32 - center;
            let coeff = if index == 0.0 {
                (wc2 - wc1) / PI
            } else {
                ((wc2 * index).sin() - (wc1 * index).sin()) / (PI * index)
            };
            coefficients.push(coeff);
        }
        
        coefficients
    }

    fn design_fir_bandstop(low_cutoff: f32, high_cutoff: f32, sampling_rate: f32, length: usize) -> Vec<f32> {
        let mut bandpass = Self::design_fir_bandpass(low_cutoff, high_cutoff, sampling_rate, length);
        
        // 带阻 = 全通 - 带通
        let center = (length - 1) / 2;
        for (i, coeff) in bandpass.iter_mut().enumerate() {
            if i == center {
                *coeff = 1.0 - *coeff;
            } else {
                *coeff = -*coeff;
            }
        }
        
        bandpass
    }

    fn apply_window_function(coefficients: &[f32], window: &WindowFunction) -> Vec<f32> {
        let length = coefficients.len();
        let mut windowed = Vec::with_capacity(length);
        
        for (i, &coeff) in coefficients.iter().enumerate() {
            let window_value = match window {
                WindowFunction::Rectangular => 1.0,
                WindowFunction::Hanning => {
                    0.5 * (1.0 - (2.0 * PI * i as f32 / (length - 1) as f32).cos())
                },
                WindowFunction::Hamming => {
                    0.54 - 0.46 * (2.0 * PI * i as f32 / (length - 1) as f32).cos()
                },
                WindowFunction::Blackman => {
                    0.42 - 0.5 * (2.0 * PI * i as f32 / (length - 1) as f32).cos() +
                    0.08 * (4.0 * PI * i as f32 / (length - 1) as f32).cos()
                },
                WindowFunction::Kaiser(beta) => {
                    let alpha = (length - 1) as f32 / 2.0;
                    let x = (i as f32 - alpha) / alpha;
                    Self::modified_bessel_i0(*beta * (1.0 - x * x).sqrt()) / 
                    Self::modified_bessel_i0(*beta)
                },
            };
            
            windowed.push(coeff * window_value);
        }
        
        windowed
    }

    fn modified_bessel_i0(x: f32) -> f32 {
        // 修正贝塞尔函数I0的近似计算
        let mut sum = 1.0;
        let mut term = 1.0;
        let x_half_squared = (x / 2.0) * (x / 2.0);
        
        for k in 1..=20 {
            term *= x_half_squared / (k * k) as f32;
            sum += term;
            if term < 1e-8 {
                break;
            }
        }
        
        sum
    }

    // 其他辅助函数的简化实现
    fn calculate_analog_poles(_response_type: &FilterResponse, _order: u8) -> Vec<Complex<f32>> {
        // 简化实现，返回空向量
        Vec::new()
    }

    fn calculate_analog_zeros(_response_type: &FilterResponse, _order: u8) -> Vec<Complex<f32>> {
        // 简化实现，返回空向量
        Vec::new()
    }

    fn bilinear_transform(
        _poles: &[Complex<f32>],
        _zeros: &[Complex<f32>],
        _cutoff: f32,
        _sampling_rate: f32,
    ) -> (Vec<Complex<f32>>, Vec<Complex<f32>>) {
        // 简化实现
        (Vec::new(), Vec::new())
    }

    fn poles_zeros_to_coefficients(
        _poles: &[Complex<f32>],
        _zeros: &[Complex<f32>],
    ) -> FilterCoefficients {
        // 简化实现
        FilterCoefficients {
            numerator: vec![1.0],
            denominator: vec![1.0],
            quantization_bits: 16,
            scaling_factors: vec![1.0],
        }
    }
}

#[derive(Debug, Clone)]
pub enum WindowFunction {
    Rectangular,
    Hanning,
    Hamming,
    Blackman,
    Kaiser(f32), // β参数
}
```

### 4. 数据结构定义

```rust
#[derive(Debug)]
pub struct FrequencyResponse {
    pub frequencies: Vec<f32>,
    pub magnitude: Vec<f32>,    // dB
    pub phase: Vec<f32>,        // degrees
    pub group_delay: Vec<f32>,  // seconds
}

#[derive(Debug)]
pub struct SensitivityAnalysis {
    pub component_sensitivities: Vec<ComponentSensitivity>,
    pub worst_case_deviation: f32,
    pub monte_carlo_results: MonteCarloResults,
}

#[derive(Debug)]
pub struct ComponentSensitivity {
    pub component_index: usize,
    pub component_type: ComponentType,
    pub magnitude_sensitivity: f32,
    pub phase_sensitivity: f32,
}

#[derive(Debug)]
pub struct MonteCarloResults {
    pub cutoff_mean: f32,
    pub cutoff_std: f32,
    pub gain_mean: f32,
    pub gain_std: f32,
    pub yield_estimate: f32,
}
```

## 自适应滤波技术

### 1. 自适应滤波器实现

```rust
#[derive(Debug)]
pub struct AdaptiveFilter {
    pub filter_type: AdaptiveFilterType,
    pub weights: Vec<f32>,
    pub delay_line: Vec<f32>,
    pub adaptation_algorithm: AdaptationAlgorithm,
    pub performance_monitor: PerformanceMonitor,
}

#[derive(Debug, Clone)]
pub enum AdaptiveFilterType {
    LMS,        // 最小均方
    NLMS,       // 归一化LMS
    RLS,        // 递归最小二乘
    Kalman,     // 卡尔曼滤波
}

#[derive(Debug)]
pub struct AdaptationAlgorithm {
    pub step_size: f32,
    pub forgetting_factor: f32,
    pub regularization: f32,
    pub convergence_threshold: f32,
}

#[derive(Debug)]
pub struct PerformanceMonitor {
    pub mse_history: Vec<f32>,
    pub weight_history: Vec<Vec<f32>>,
    pub convergence_time: f32,
    pub steady_state_error: f32,
}

impl AdaptiveFilter {
    pub fn new_lms(filter_length: usize, step_size: f32) -> Self {
        Self {
            filter_type: AdaptiveFilterType::LMS,
            weights: vec![0.0; filter_length],
            delay_line: vec![0.0; filter_length],
            adaptation_algorithm: AdaptationAlgorithm {
                step_size,
                forgetting_factor: 1.0,
                regularization: 0.0,
                convergence_threshold: 1e-6,
            },
            performance_monitor: PerformanceMonitor {
                mse_history: Vec::new(),
                weight_history: Vec::new(),
                convergence_time: 0.0,
                steady_state_error: 0.0,
            },
        }
    }

    pub fn adapt(&mut self, input: f32, desired: f32) -> f32 {
        // 更新延迟线
        self.delay_line.rotate_right(1);
        self.delay_line[0] = input;
        
        // 计算滤波器输出
        let output = self.compute_output();
        
        // 计算误差
        let error = desired - output;
        
        // 更新权重
        match self.filter_type {
            AdaptiveFilterType::LMS => self.update_weights_lms(error),
            AdaptiveFilterType::NLMS => self.update_weights_nlms(error),
            AdaptiveFilterType::RLS => self.update_weights_rls(error),
            _ => {},
        }
        
        // 更新性能监控
        self.update_performance_monitor(error);
        
        output
    }

    fn compute_output(&self) -> f32 {
        self.weights.iter()
            .zip(self.delay_line.iter())
            .map(|(w, x)| w * x)
            .sum()
    }

    fn update_weights_lms(&mut self, error: f32) {
        for (weight, &input) in self.weights.iter_mut().zip(self.delay_line.iter()) {
            *weight += self.adaptation_algorithm.step_size * error * input;
        }
    }

    fn update_weights_nlms(&mut self, error: f32) {
        let input_power: f32 = self.delay_line.iter().map(|x| x * x).sum();
        let normalized_step = self.adaptation_algorithm.step_size / 
                             (input_power + self.adaptation_algorithm.regularization);
        
        for (weight, &input) in self.weights.iter_mut().zip(self.delay_line.iter()) {
            *weight += normalized_step * error * input;
        }
    }

    fn update_weights_rls(&mut self, error: f32) {
        // RLS算法的简化实现
        // 实际实现需要维护逆相关矩阵
        let lambda = self.adaptation_algorithm.forgetting_factor;
        let step_size = self.adaptation_algorithm.step_size * (1.0 - lambda);
        
        for (weight, &input) in self.weights.iter_mut().zip(self.delay_line.iter()) {
            *weight += step_size * error * input;
        }
    }

    fn update_performance_monitor(&mut self, error: f32) {
        let mse = error * error;
        self.performance_monitor.mse_history.push(mse);
        
        // 检查收敛
        if self.performance_monitor.mse_history.len() > 100 {
            let recent_mse: f32 = self.performance_monitor.mse_history
                .iter()
                .rev()
                .take(10)
                .sum::<f32>() / 10.0;
            
            if recent_mse < self.adaptation_algorithm.convergence_threshold {
                self.performance_monitor.steady_state_error = recent_mse;
            }
        }
    }

    pub fn analyze_convergence(&self) -> ConvergenceAnalysis {
        let mse_history = &self.performance_monitor.mse_history;
        
        if mse_history.len() < 10 {
            return ConvergenceAnalysis {
                converged: false,
                convergence_time: 0.0,
                final_mse: 0.0,
                learning_curve_slope: 0.0,
            };
        }
        
        // 检测收敛点
        let mut convergence_point = None;
        let threshold = self.adaptation_algorithm.convergence_threshold;
        
        for (i, &mse) in mse_history.iter().enumerate() {
            if mse < threshold {
                convergence_point = Some(i);
                break;
            }
        }
        
        let converged = convergence_point.is_some();
        let convergence_time = convergence_point.unwrap_or(0) as f32;
        let final_mse = mse_history.last().copied().unwrap_or(0.0);
        
        // 计算学习曲线斜率
        let learning_curve_slope = if mse_history.len() > 1 {
            let start_mse = mse_history[0];
            let end_mse = mse_history[mse_history.len() - 1];
            (end_mse - start_mse) / mse_history.len() as f32
        } else {
            0.0
        };
        
        ConvergenceAnalysis {
            converged,
            convergence_time,
            final_mse,
            learning_curve_slope,
        }
    }
}

#[derive(Debug)]
pub struct ConvergenceAnalysis {
    pub converged: bool,
    pub convergence_time: f32,
    pub final_mse: f32,
    pub learning_curve_slope: f32,
}
```

## 滤波器优化和调试

### 1. 滤波器性能分析

```rust
pub struct FilterPerformanceAnalyzer {
    pub test_signals: Vec<TestSignal>,
    pub noise_sources: Vec<NoiseSource>,
    pub analysis_config: AnalysisConfig,
}

#[derive(Debug, Clone)]
pub struct TestSignal {
    pub signal_type: SignalType,
    pub frequency: f32,
    pub amplitude: f32,
    pub phase: f32,
    pub duration: f32,
}

#[derive(Debug, Clone)]
pub enum SignalType {
    Sine,
    Square,
    Triangle,
    Sawtooth,
    Chirp(f32, f32), // 起始频率, 结束频率
    WhiteNoise,
    PinkNoise,
    Impulse,
}

impl FilterPerformanceAnalyzer {
    pub fn analyze_filter_performance(&self, filter: &mut DigitalFilter) -> FilterPerformanceReport {
        let mut test_results = Vec::new();
        
        for test_signal in &self.test_signals {
            let input_samples = self.generate_test_signal(test_signal);
            let mut output_samples = Vec::new();
            
            // 重置滤波器状态
            filter.state_variables.fill(0.0);
            
            // 处理测试信号
            for &sample in &input_samples {
                let output = filter.process_sample(sample);
                output_samples.push(output);
            }
            
            // 分析结果
            let result = self.analyze_signal_response(
                &input_samples,
                &output_samples,
                test_signal,
            );
            
            test_results.push(result);
        }
        
        FilterPerformanceReport {
            test_results,
            overall_score: self.calculate_overall_score(&test_results),
            recommendations: self.generate_recommendations(&test_results),
        }
    }

    fn generate_test_signal(&self, signal: &TestSignal) -> Vec<f32> {
        let sample_count = (signal.duration * self.analysis_config.sampling_rate) as usize;
        let mut samples = Vec::with_capacity(sample_count);
        
        for n in 0..sample_count {
            let t = n as f32 / self.analysis_config.sampling_rate;
            let sample = match signal.signal_type {
                SignalType::Sine => {
                    signal.amplitude * (2.0 * PI * signal.frequency * t + signal.phase).sin()
                },
                SignalType::Square => {
                    signal.amplitude * if (2.0 * PI * signal.frequency * t + signal.phase).sin() > 0.0 { 1.0 } else { -1.0 }
                },
                SignalType::Triangle => {
                    signal.amplitude * (2.0 / PI) * (2.0 * PI * signal.frequency * t + signal.phase).sin().asin()
                },
                SignalType::Chirp(f_start, f_end) => {
                    let instantaneous_freq = f_start + (f_end - f_start) * t / signal.duration;
                    signal.amplitude * (2.0 * PI * instantaneous_freq * t + signal.phase).sin()
                },
                SignalType::WhiteNoise => {
                    signal.amplitude * (rand::random::<f32>() - 0.5) * 2.0
                },
                SignalType::Impulse => {
                    if n == 0 { signal.amplitude } else { 0.0 }
                },
                _ => 0.0,
            };
            samples.push(sample);
        }
        
        samples
    }

    fn analyze_signal_response(
        &self,
        input: &[f32],
        output: &[f32],
        test_signal: &TestSignal,
    ) -> TestResult {
        // 计算频率响应
        let input_spectrum = self.calculate_fft(input);
        let output_spectrum = self.calculate_fft(output);
        
        let magnitude_response = self.calculate_magnitude_response(&input_spectrum, &output_spectrum);
        let phase_response = self.calculate_phase_response(&input_spectrum, &output_spectrum);
        
        // 计算失真
        let thd = self.calculate_thd(&output_spectrum, test_signal.frequency);
        let snr = self.calculate_snr(output);
        
        // 计算群延迟
        let group_delay = self.calculate_group_delay(&phase_response);
        
        TestResult {
            test_signal: test_signal.clone(),
            magnitude_response,
            phase_response,
            group_delay,
            thd,
            snr,
            passband_ripple: self.calculate_passband_ripple(&magnitude_response),
            stopband_attenuation: self.calculate_stopband_attenuation(&magnitude_response),
        }
    }

    fn calculate_fft(&self, signal: &[f32]) -> Vec<Complex<f32>> {
        // 简化的FFT实现（实际应使用优化的FFT库）
        let n = signal.len();
        let mut spectrum = Vec::with_capacity(n);
        
        for k in 0..n {
            let mut sum = Complex::new(0.0, 0.0);
            for (n_idx, &sample) in signal.iter().enumerate() {
                let angle = -2.0 * PI * k as f32 * n_idx as f32 / n as f32;
                sum += Complex::new(sample * angle.cos(), sample * angle.sin());
            }
            spectrum.push(sum);
        }
        
        spectrum
    }

    fn calculate_magnitude_response(
        &self,
        input_spectrum: &[Complex<f32>],
        output_spectrum: &[Complex<f32>],
    ) -> Vec<f32> {
        input_spectrum.iter()
            .zip(output_spectrum.iter())
            .map(|(input, output)| {
                let input_mag = input.norm();
                let output_mag = output.norm();
                if input_mag > 1e-10 {
                    20.0 * (output_mag / input_mag).log10()
                } else {
                    -100.0 // 很小的输入，设为-100dB
                }
            })
            .collect()
    }

    fn calculate_phase_response(
        &self,
        input_spectrum: &[Complex<f32>],
        output_spectrum: &[Complex<f32>],
    ) -> Vec<f32> {
        input_spectrum.iter()
            .zip(output_spectrum.iter())
            .map(|(input, output)| {
                (output.arg() - input.arg()) * 180.0 / PI
            })
            .collect()
    }

    fn calculate_thd(&self, spectrum: &[Complex<f32>], fundamental_freq: f32) -> f32 {
        let fs = self.analysis_config.sampling_rate;
        let fundamental_bin = (fundamental_freq * spectrum.len() as f32 / fs) as usize;
        
        if fundamental_bin >= spectrum.len() {
            return 0.0;
        }
        
        let fundamental_power = spectrum[fundamental_bin].norm().powi(2);
        let mut harmonic_power = 0.0;
        
        // 计算2到10次谐波的功率
        for harmonic in 2..=10 {
            let harmonic_bin = fundamental_bin * harmonic;
            if harmonic_bin < spectrum.len() {
                harmonic_power += spectrum[harmonic_bin].norm().powi(2);
            }
        }
        
        if fundamental_power > 0.0 {
            10.0 * (harmonic_power / fundamental_power).log10()
        } else {
            -100.0
        }
    }

    fn calculate_snr(&self, signal: &[f32]) -> f32 {
        let signal_power: f32 = signal.iter().map(|x| x * x).sum();
        let mean = signal.iter().sum::<f32>() / signal.len() as f32;
        let noise_power: f32 = signal.iter().map(|x| (x - mean).powi(2)).sum();
        
        if noise_power > 0.0 {
            10.0 * (signal_power / noise_power).log10()
        } else {
            100.0 // 无噪声情况
        }
    }

    fn calculate_group_delay(&self, phase_response: &[f32]) -> Vec<f32> {
        let mut group_delay = Vec::with_capacity(phase_response.len());
        
        for i in 0..phase_response.len() {
            let delay = if i == 0 {
                0.0
            } else {
                let phase_diff = phase_response[i] - phase_response[i - 1];
                let freq_diff = self.analysis_config.sampling_rate / phase_response.len() as f32;
                -phase_diff / (2.0 * PI * freq_diff)
            };
            group_delay.push(delay);
        }
        
        group_delay
    }

    fn calculate_passband_ripple(&self, magnitude_response: &[f32]) -> f32 {
        // 简化的通带纹波计算
        let passband_end = magnitude_response.len() / 4; // 假设前1/4为通带
        let passband = &magnitude_response[0..passband_end];
        
        let max_gain = passband.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        let min_gain = passband.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        
        max_gain - min_gain
    }

    fn calculate_stopband_attenuation(&self, magnitude_response: &[f32]) -> f32 {
        // 简化的阻带衰减计算
        let stopband_start = magnitude_response.len() / 2; // 假设后1/2为阻带
        let stopband = &magnitude_response[stopband_start..];
        
        stopband.iter().fold(f32::INFINITY, |a, &b| a.min(b)).abs()
    }

    fn calculate_overall_score(&self, results: &[TestResult]) -> f32 {
        if results.is_empty() {
            return 0.0;
        }
        
        let mut total_score = 0.0;
        
        for result in results {
            let mut test_score = 100.0;
            
            // 基于各项指标扣分
            if result.thd > -40.0 {
                test_score -= 20.0;
            }
            if result.snr < 60.0 {
                test_score -= 15.0;
            }
            if result.passband_ripple > 1.0 {
                test_score -= 10.0;
            }
            if result.stopband_attenuation < 40.0 {
                test_score -= 25.0;
            }
            
            total_score += test_score.max(0.0);
        }
        
        total_score / results.len() as f32
    }

    fn generate_recommendations(&self, results: &[TestResult]) -> Vec<String> {
        let mut recommendations = Vec::new();
        
        for result in results {
            if result.thd > -40.0 {
                recommendations.push("考虑增加滤波器阶数以降低谐波失真".to_string());
            }
            if result.snr < 60.0 {
                recommendations.push("优化滤波器设计以提高信噪比".to_string());
            }
            if result.passband_ripple > 1.0 {
                recommendations.push("调整滤波器参数以减少通带纹波".to_string());
            }
            if result.stopband_attenuation < 40.0 {
                recommendations.push("增加滤波器阶数以提高阻带衰减".to_string());
            }
        }
        
        recommendations.sort();
        recommendations.dedup();
        recommendations
    }
}

#[derive(Debug)]
pub struct FilterPerformanceReport {
    pub test_results: Vec<TestResult>,
    pub overall_score: f32,
    pub recommendations: Vec<String>,
}

#[derive(Debug)]
pub struct TestResult {
    pub test_signal: TestSignal,
    pub magnitude_response: Vec<f32>,
    pub phase_response: Vec<f32>,
    pub group_delay: Vec<f32>,
    pub thd: f32,
    pub snr: f32,
    pub passband_ripple: f32,
    pub stopband_attenuation: f32,
}

#[derive(Debug)]
pub struct AnalysisConfig {
    pub sampling_rate: f32,
    pub analysis_duration: f32,
    pub frequency_resolution: f32,
}
```

## 最佳实践和设计指南

### 1. 滤波器选择指南

1. **应用需求分析**
   - 确定滤波器类型（低通、高通、带通、带阻）
   - 明确性能指标（截止频率、阻带衰减、通带纹波）
   - 考虑实时性要求（群延迟、相位线性度）
   - 评估资源限制（计算复杂度、内存使用）

2. **模拟vs数字滤波器选择**
   - **模拟滤波器优势**：
     - 无采样率限制
     - 低功耗
     - 实时处理
     - 无量化噪声
   - **数字滤波器优势**：
     - 精确的频率响应
     - 稳定性好
     - 易于修改和调试
     - 可实现复杂算法

3. **滤波器拓扑选择**
   - **Sallen-Key**：简单、低噪声、适合低阶滤波器
   - **多重反馈**：高Q值、良好的增益控制
   - **状态变量**：同时提供低通、高通、带通输出
   - **开关电容**：精确的频率响应、集成度高

### 2. 设计优化策略

1. **性能优化**
   - 选择合适的运放（GBW、噪声、失调）
   - 优化组件值（标准值、容差匹配）
   - 考虑寄生效应（PCB布局、组件寄生参数）
   - 实施温度补偿

2. **稳定性保证**
   - 相位裕度分析
   - 增益裕度检查
   - 极点零点分布优化
   - 反馈环路稳定性

3. **噪声优化**
   - 选择低噪声组件
   - 优化信号路径
   - 实施屏蔽和接地
   - 电源去耦

### 3. 实现注意事项

1. **硬件实现**
   - PCB布局优化
   - 电源设计
   - 信号完整性
   - EMI/EMC考虑

2. **软件实现**
   - 数值精度选择
   - 溢出保护
   - 实时性保证
   - 内存管理

3. **测试验证**
   - 频率响应测试
   - 失真测量
   - 噪声分析
   - 稳定性验证

## 结论

滤波技术是ADC/DAC系统中的关键技术，正确的滤波器设计和实现对系统性能至关重要。通过合理选择滤波器类型、优化设计参数、实施适当的实现策略，可以显著提高系统的信号质量和整体性能。

本文档提供的设计方法和实现代码为STM32F4系列微控制器的滤波器设计提供了全面的技术支持，有助于工程师快速开发高性能的信号处理系统。