# 信号完整性分析技术文档

## 概述

信号完整性(Signal Integrity, SI)是指信号在传输过程中保持其时域和频域特性的能力。在高速ADC/DAC系统中，信号完整性问题会直接影响系统的测量精度和性能。本文档详细分析STM32F4系列微控制器ADC/DAC系统中的信号完整性问题及其解决方案。

## 信号完整性基础理论

### 1. 传输线理论

```rust
use core::f32::consts::PI;

#[derive(Debug, Clone)]
pub struct TransmissionLine {
    pub length: f32,           // 传输线长度 (mm)
    pub width: f32,            // 线宽 (mm)
    pub thickness: f32,        // 铜厚 (μm)
    pub dielectric_constant: f32, // 介电常数
    pub dielectric_thickness: f32, // 介质厚度 (mm)
    pub loss_tangent: f32,     // 损耗角正切
    pub characteristic_impedance: f32, // 特征阻抗 (Ω)
    pub propagation_delay: f32, // 传播延迟 (ps/mm)
}

impl TransmissionLine {
    pub fn new_microstrip(
        length: f32,
        width: f32,
        thickness: f32,
        dielectric_constant: f32,
        dielectric_thickness: f32,
        loss_tangent: f32,
    ) -> Self {
        let z0 = Self::calculate_microstrip_impedance(
            width, thickness, dielectric_constant, dielectric_thickness
        );
        let td = Self::calculate_propagation_delay(dielectric_constant);
        
        Self {
            length,
            width,
            thickness,
            dielectric_constant,
            dielectric_thickness,
            loss_tangent,
            characteristic_impedance: z0,
            propagation_delay: td,
        }
    }

    fn calculate_microstrip_impedance(
        width: f32,
        thickness: f32,
        er: f32,
        height: f32,
    ) -> f32 {
        // 微带线特征阻抗计算（IPC-2141A公式）
        let w_eff = width + thickness * (1.25 * (1.0 + (height / thickness).ln()) / PI);
        let er_eff = (er + 1.0) / 2.0 + (er - 1.0) / 2.0 * (1.0 + 12.0 * height / w_eff).powf(-0.5);
        
        let z0 = if w_eff / height < 1.0 {
            // 窄线情况
            60.0 / er_eff.sqrt() * (8.0 * height / w_eff + w_eff / (4.0 * height)).ln()
        } else {
            // 宽线情况
            120.0 * PI / er_eff.sqrt() / (w_eff / height + 1.393 + 0.667 * (w_eff / height + 1.444).ln())
        };
        
        z0
    }

    fn calculate_propagation_delay(er: f32) -> f32 {
        // 传播延迟 (ps/mm)
        let c = 2.998e11; // 光速 (mm/s)
        1e12 * er.sqrt() / c
    }

    pub fn calculate_electrical_length(&self, frequency: f32) -> f32 {
        // 电长度（以波长为单位）
        let wavelength = 2.998e11 / (frequency * self.dielectric_constant.sqrt()); // mm
        self.length / wavelength
    }

    pub fn calculate_insertion_loss(&self, frequency: f32) -> f32 {
        // 插入损耗计算 (dB)
        let alpha_c = self.calculate_conductor_loss(frequency);
        let alpha_d = self.calculate_dielectric_loss(frequency);
        (alpha_c + alpha_d) * self.length
    }

    fn calculate_conductor_loss(&self, frequency: f32) -> f32 {
        // 导体损耗 (dB/mm)
        let rs = (PI * frequency * 4e-7 * 1.7e-8).sqrt(); // 表面电阻
        let k = 1.0 + (2.0 / PI) * (self.thickness / (2.0 * rs)).atan();
        
        8.686 * rs / (self.characteristic_impedance * self.width * k)
    }

    fn calculate_dielectric_loss(&self, frequency: f32) -> f32 {
        // 介质损耗 (dB/mm)
        27.3 * self.dielectric_constant * self.loss_tangent * frequency / 
        (2.998e11 * self.dielectric_constant.sqrt())
    }

    pub fn calculate_reflection_coefficient(&self, load_impedance: f32) -> f32 {
        // 反射系数
        (load_impedance - self.characteristic_impedance) / 
        (load_impedance + self.characteristic_impedance)
    }

    pub fn calculate_return_loss(&self, load_impedance: f32) -> f32 {
        // 回波损耗 (dB)
        let gamma = self.calculate_reflection_coefficient(load_impedance);
        -20.0 * gamma.abs().log10()
    }
}
```

### 2. 信号完整性分析器

```rust
#[derive(Debug)]
pub struct SignalIntegrityAnalyzer {
    pub transmission_lines: Vec<TransmissionLine>,
    pub components: Vec<Component>,
    pub frequency_range: FrequencyRange,
    pub analysis_config: AnalysisConfig,
}

#[derive(Debug, Clone)]
pub struct Component {
    pub component_type: ComponentType,
    pub value: f32,
    pub parasitic_l: f32,  // 寄生电感 (nH)
    pub parasitic_c: f32,  // 寄生电容 (pF)
    pub parasitic_r: f32,  // 寄生电阻 (Ω)
    pub package_type: PackageType,
}

#[derive(Debug, Clone)]
pub enum ComponentType {
    Resistor(f32),      // 阻值 (Ω)
    Capacitor(f32),     // 容值 (pF)
    Inductor(f32),      // 感值 (nH)
    IC(ICModel),        // 集成电路模型
}

#[derive(Debug, Clone)]
pub struct ICModel {
    pub input_capacitance: f32,  // 输入电容 (pF)
    pub output_resistance: f32,  // 输出电阻 (Ω)
    pub switching_current: f32,  // 开关电流 (mA)
    pub rise_time: f32,         // 上升时间 (ps)
    pub fall_time: f32,         // 下降时间 (ps)
}

#[derive(Debug, Clone)]
pub enum PackageType {
    SMD0402,
    SMD0603,
    SMD0805,
    SMD1206,
    BGA,
    QFP,
    SOIC,
}

#[derive(Debug, Clone)]
pub struct FrequencyRange {
    pub start_freq: f32,    // Hz
    pub stop_freq: f32,     // Hz
    pub points: usize,
}

#[derive(Debug, Clone)]
pub struct AnalysisConfig {
    pub time_step: f32,         // 时间步长 (ps)
    pub simulation_time: f32,   // 仿真时间 (ns)
    pub impedance_tolerance: f32, // 阻抗容差 (%)
    pub crosstalk_threshold: f32, // 串扰阈值 (%)
}

impl SignalIntegrityAnalyzer {
    pub fn new() -> Self {
        Self {
            transmission_lines: Vec::new(),
            components: Vec::new(),
            frequency_range: FrequencyRange {
                start_freq: 1e3,
                stop_freq: 1e9,
                points: 1000,
            },
            analysis_config: AnalysisConfig {
                time_step: 1.0,
                simulation_time: 100.0,
                impedance_tolerance: 10.0,
                crosstalk_threshold: 5.0,
            },
        }
    }

    pub fn add_transmission_line(&mut self, tline: TransmissionLine) {
        self.transmission_lines.push(tline);
    }

    pub fn add_component(&mut self, component: Component) {
        self.components.push(component);
    }

    pub fn analyze_impedance_profile(&self) -> ImpedanceProfile {
        let mut impedance_points = Vec::new();
        let mut discontinuities = Vec::new();
        
        let mut current_position = 0.0;
        let mut current_impedance = 50.0; // 默认50Ω
        
        for tline in &self.transmission_lines {
            // 检查阻抗不连续点
            let impedance_change = (tline.characteristic_impedance - current_impedance).abs();
            let impedance_change_percent = impedance_change / current_impedance * 100.0;
            
            if impedance_change_percent > self.analysis_config.impedance_tolerance {
                discontinuities.push(ImpedanceDiscontinuity {
                    position: current_position,
                    impedance_before: current_impedance,
                    impedance_after: tline.characteristic_impedance,
                    reflection_coefficient: tline.calculate_reflection_coefficient(current_impedance),
                    severity: if impedance_change_percent > 20.0 {
                        DiscontinuitySeverity::High
                    } else if impedance_change_percent > 10.0 {
                        DiscontinuitySeverity::Medium
                    } else {
                        DiscontinuitySeverity::Low
                    },
                });
            }
            
            // 添加阻抗点
            impedance_points.push(ImpedancePoint {
                position: current_position,
                impedance: tline.characteristic_impedance,
                line_width: tline.width,
                dielectric_constant: tline.dielectric_constant,
            });
            
            current_position += tline.length;
            current_impedance = tline.characteristic_impedance;
        }
        
        ImpedanceProfile {
            impedance_points,
            discontinuities,
            total_length: current_position,
            average_impedance: self.calculate_average_impedance(),
        }
    }

    pub fn analyze_frequency_response(&self) -> FrequencyResponse {
        let frequencies = self.generate_frequency_points();
        let mut s_parameters = Vec::new();
        let mut insertion_loss = Vec::new();
        let mut return_loss = Vec::new();
        let mut group_delay = Vec::new();
        
        for &freq in &frequencies {
            let mut total_insertion_loss = 0.0;
            let mut total_reflection = 0.0;
            let mut total_delay = 0.0;
            
            for tline in &self.transmission_lines {
                total_insertion_loss += tline.calculate_insertion_loss(freq);
                total_reflection += tline.calculate_reflection_coefficient(50.0).abs();
                total_delay += tline.length * tline.propagation_delay;
            }
            
            insertion_loss.push(-total_insertion_loss); // 负值表示损耗
            return_loss.push(-20.0 * total_reflection.log10());
            group_delay.push(total_delay);
            
            // S参数计算（简化）
            let s11 = total_reflection;
            let s21 = (1.0 - total_reflection) * (-total_insertion_loss / 20.0).exp();
            
            s_parameters.push(SParameter {
                frequency: freq,
                s11_magnitude: s11,
                s11_phase: 0.0, // 简化
                s21_magnitude: s21,
                s21_phase: -2.0 * PI * freq * total_delay / 1e12, // 相位延迟
            });
        }
        
        FrequencyResponse {
            frequencies,
            s_parameters,
            insertion_loss,
            return_loss,
            group_delay,
            bandwidth_3db: self.calculate_3db_bandwidth(&insertion_loss, &frequencies),
        }
    }

    pub fn analyze_eye_diagram(&self, data_rate: f32) -> EyeDiagram {
        let bit_period = 1.0 / data_rate; // 秒
        let samples_per_bit = (bit_period / (self.analysis_config.time_step * 1e-12)) as usize;
        
        // 生成伪随机数据序列
        let data_sequence = self.generate_prbs_sequence(1024);
        
        // 计算眼图参数
        let mut eye_traces = Vec::new();
        let mut jitter_measurements = Vec::new();
        
        for window_start in 0..data_sequence.len() - 2 {
            let trace = self.simulate_signal_response(
                &data_sequence[window_start..window_start + 3],
                samples_per_bit,
            );
            eye_traces.push(trace);
        }
        
        // 分析眼图质量
        let eye_height = self.calculate_eye_height(&eye_traces);
        let eye_width = self.calculate_eye_width(&eye_traces, samples_per_bit);
        let jitter_rms = self.calculate_timing_jitter(&eye_traces);
        let noise_rms = self.calculate_amplitude_noise(&eye_traces);
        
        EyeDiagram {
            data_rate,
            bit_period,
            eye_height,
            eye_width,
            jitter_rms,
            noise_rms,
            eye_traces,
            quality_factor: eye_height / (2.0 * noise_rms),
            timing_margin: eye_width / bit_period * 100.0, // 百分比
        }
    }

    pub fn analyze_crosstalk(&self) -> CrosstalkAnalysis {
        let mut crosstalk_results = Vec::new();
        
        // 分析相邻传输线之间的串扰
        for i in 0..self.transmission_lines.len() {
            for j in (i + 1)..self.transmission_lines.len() {
                let aggressor = &self.transmission_lines[i];
                let victim = &self.transmission_lines[j];
                
                // 计算耦合系数（简化模型）
                let coupling = self.calculate_coupling_coefficient(aggressor, victim);
                
                if coupling.abs() > self.analysis_config.crosstalk_threshold / 100.0 {
                    let crosstalk = CrosstalkResult {
                        aggressor_line: i,
                        victim_line: j,
                        near_end_crosstalk: coupling * 0.5, // NEXT
                        far_end_crosstalk: coupling * 0.3,  // FEXT
                        coupling_length: aggressor.length.min(victim.length),
                        frequency_response: self.calculate_crosstalk_frequency_response(coupling),
                        severity: if coupling.abs() > 0.2 {
                            CrosstalkSeverity::High
                        } else if coupling.abs() > 0.1 {
                            CrosstalkSeverity::Medium
                        } else {
                            CrosstalkSeverity::Low
                        },
                    };
                    crosstalk_results.push(crosstalk);
                }
            }
        }
        
        CrosstalkAnalysis {
            crosstalk_results,
            worst_case_next: crosstalk_results.iter()
                .map(|c| c.near_end_crosstalk.abs())
                .fold(0.0, f32::max),
            worst_case_fext: crosstalk_results.iter()
                .map(|c| c.far_end_crosstalk.abs())
                .fold(0.0, f32::max),
            total_crosstalk_budget: self.calculate_total_crosstalk_budget(&crosstalk_results),
        }
    }

    fn calculate_coupling_coefficient(&self, line1: &TransmissionLine, line2: &TransmissionLine) -> f32 {
        // 简化的耦合系数计算
        // 实际应该考虑线间距、介质厚度等因素
        let spacing = 0.2; // 假设0.2mm间距
        let height = line1.dielectric_thickness;
        
        // 电容耦合
        let c_mutual = 8.854e-12 * line1.dielectric_constant * line1.length / 
                      (PI * spacing / height).ln();
        let c_self = 8.854e-12 * line1.dielectric_constant * line1.length / 
                    (PI * line1.width / height).ln();
        
        c_mutual / c_self
    }

    fn generate_frequency_points(&self) -> Vec<f32> {
        let mut frequencies = Vec::new();
        let log_start = self.frequency_range.start_freq.log10();
        let log_stop = self.frequency_range.stop_freq.log10();
        let log_step = (log_stop - log_start) / (self.frequency_range.points - 1) as f32;
        
        for i in 0..self.frequency_range.points {
            let log_freq = log_start + i as f32 * log_step;
            frequencies.push(10.0_f32.powf(log_freq));
        }
        
        frequencies
    }

    fn calculate_average_impedance(&self) -> f32 {
        if self.transmission_lines.is_empty() {
            return 50.0;
        }
        
        let total_length: f32 = self.transmission_lines.iter().map(|tl| tl.length).sum();
        let weighted_impedance: f32 = self.transmission_lines.iter()
            .map(|tl| tl.characteristic_impedance * tl.length)
            .sum();
        
        weighted_impedance / total_length
    }

    fn calculate_3db_bandwidth(&self, insertion_loss: &[f32], frequencies: &[f32]) -> f32 {
        // 查找-3dB点
        let max_response = insertion_loss.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        let target_level = max_response - 3.0;
        
        for (i, &loss) in insertion_loss.iter().enumerate() {
            if loss <= target_level {
                return frequencies[i];
            }
        }
        
        frequencies.last().copied().unwrap_or(0.0)
    }

    fn generate_prbs_sequence(&self, length: usize) -> Vec<u8> {
        // 生成PRBS-7序列
        let mut sequence = Vec::with_capacity(length);
        let mut shift_register = 0x7F_u8; // 初始值
        
        for _ in 0..length {
            let output = shift_register & 1;
            sequence.push(output);
            
            // PRBS-7多项式: x^7 + x^6 + 1
            let feedback = ((shift_register >> 6) ^ (shift_register >> 5)) & 1;
            shift_register = (shift_register >> 1) | (feedback << 6);
        }
        
        sequence
    }

    fn simulate_signal_response(&self, data_bits: &[u8], samples_per_bit: usize) -> Vec<f32> {
        let mut response = Vec::new();
        
        for &bit in data_bits {
            let amplitude = if bit == 1 { 1.0 } else { 0.0 };
            
            // 简化的阶跃响应
            for sample in 0..samples_per_bit {
                let t = sample as f32 / samples_per_bit as f32;
                let rise_time = 0.1; // 10%的位周期
                
                let signal = if t < rise_time {
                    amplitude * t / rise_time
                } else {
                    amplitude
                };
                
                response.push(signal);
            }
        }
        
        response
    }

    fn calculate_eye_height(&self, traces: &[Vec<f32>]) -> f32 {
        if traces.is_empty() {
            return 0.0;
        }
        
        let mid_point = traces[0].len() / 2;
        let mut high_levels = Vec::new();
        let mut low_levels = Vec::new();
        
        for trace in traces {
            if trace.len() > mid_point {
                let level = trace[mid_point];
                if level > 0.5 {
                    high_levels.push(level);
                } else {
                    low_levels.push(level);
                }
            }
        }
        
        let avg_high = high_levels.iter().sum::<f32>() / high_levels.len() as f32;
        let avg_low = low_levels.iter().sum::<f32>() / low_levels.len() as f32;
        
        avg_high - avg_low
    }

    fn calculate_eye_width(&self, traces: &[Vec<f32>], samples_per_bit: usize) -> f32 {
        // 简化的眼宽计算
        samples_per_bit as f32 * 0.8 * self.analysis_config.time_step // 80%的位周期
    }

    fn calculate_timing_jitter(&self, traces: &[Vec<f32>]) -> f32 {
        // 简化的时序抖动计算
        0.05 // 5%的位周期RMS抖动
    }

    fn calculate_amplitude_noise(&self, traces: &[Vec<f32>]) -> f32 {
        // 简化的幅度噪声计算
        0.02 // 2%的信号幅度RMS噪声
    }
}
```

### 3. 数据结构定义

```rust
#[derive(Debug)]
pub struct ImpedanceProfile {
    pub impedance_points: Vec<ImpedancePoint>,
    pub discontinuities: Vec<ImpedanceDiscontinuity>,
    pub total_length: f32,
    pub average_impedance: f32,
}

#[derive(Debug)]
pub struct ImpedancePoint {
    pub position: f32,
    pub impedance: f32,
    pub line_width: f32,
    pub dielectric_constant: f32,
}

#[derive(Debug)]
pub struct ImpedanceDiscontinuity {
    pub position: f32,
    pub impedance_before: f32,
    pub impedance_after: f32,
    pub reflection_coefficient: f32,
    pub severity: DiscontinuitySeverity,
}

#[derive(Debug, PartialEq)]
pub enum DiscontinuitySeverity {
    Low,
    Medium,
    High,
}

#[derive(Debug)]
pub struct FrequencyResponse {
    pub frequencies: Vec<f32>,
    pub s_parameters: Vec<SParameter>,
    pub insertion_loss: Vec<f32>,
    pub return_loss: Vec<f32>,
    pub group_delay: Vec<f32>,
    pub bandwidth_3db: f32,
}

#[derive(Debug)]
pub struct SParameter {
    pub frequency: f32,
    pub s11_magnitude: f32,
    pub s11_phase: f32,
    pub s21_magnitude: f32,
    pub s21_phase: f32,
}

#[derive(Debug)]
pub struct EyeDiagram {
    pub data_rate: f32,
    pub bit_period: f32,
    pub eye_height: f32,
    pub eye_width: f32,
    pub jitter_rms: f32,
    pub noise_rms: f32,
    pub eye_traces: Vec<Vec<f32>>,
    pub quality_factor: f32,
    pub timing_margin: f32,
}

#[derive(Debug)]
pub struct CrosstalkAnalysis {
    pub crosstalk_results: Vec<CrosstalkResult>,
    pub worst_case_next: f32,
    pub worst_case_fext: f32,
    pub total_crosstalk_budget: f32,
}

#[derive(Debug)]
pub struct CrosstalkResult {
    pub aggressor_line: usize,
    pub victim_line: usize,
    pub near_end_crosstalk: f32,
    pub far_end_crosstalk: f32,
    pub coupling_length: f32,
    pub frequency_response: Vec<f32>,
    pub severity: CrosstalkSeverity,
}

#[derive(Debug, PartialEq)]
pub enum CrosstalkSeverity {
    Low,
    Medium,
    High,
}
```

## 信号完整性优化技术

### 1. 阻抗控制

```rust
pub struct ImpedanceController {
    pub target_impedance: f32,
    pub tolerance: f32,
    pub stackup: PCBStackup,
}

#[derive(Debug)]
pub struct PCBStackup {
    pub layers: Vec<PCBLayer>,
    pub total_thickness: f32,
}

#[derive(Debug)]
pub struct PCBLayer {
    pub layer_type: LayerType,
    pub thickness: f32,
    pub material: Material,
}

#[derive(Debug)]
pub enum LayerType {
    Signal,
    Ground,
    Power,
    Dielectric,
}

#[derive(Debug)]
pub struct Material {
    pub name: String,
    pub dielectric_constant: f32,
    pub loss_tangent: f32,
    pub thermal_coefficient: f32,
}

impl ImpedanceController {
    pub fn optimize_trace_geometry(&self, target_z0: f32) -> TraceGeometry {
        // 优化走线几何尺寸以达到目标阻抗
        let mut width = 0.1; // 初始宽度 (mm)
        let mut best_width = width;
        let mut min_error = f32::INFINITY;
        
        // 迭代优化
        for _ in 0..100 {
            let calculated_z0 = self.calculate_impedance(width);
            let error = (calculated_z0 - target_z0).abs();
            
            if error < min_error {
                min_error = error;
                best_width = width;
            }
            
            if error < target_z0 * self.tolerance / 100.0 {
                break;
            }
            
            // 调整宽度
            if calculated_z0 > target_z0 {
                width += 0.01; // 增加宽度降低阻抗
            } else {
                width -= 0.01; // 减少宽度提高阻抗
            }
            
            if width <= 0.05 {
                width = 0.05; // 最小宽度限制
                break;
            }
        }
        
        TraceGeometry {
            width: best_width,
            thickness: 0.035, // 标准铜厚
            spacing_to_plane: self.get_dielectric_thickness(),
            calculated_impedance: self.calculate_impedance(best_width),
            impedance_error: min_error,
        }
    }

    fn calculate_impedance(&self, width: f32) -> f32 {
        // 简化的阻抗计算
        let h = self.get_dielectric_thickness();
        let er = self.get_effective_dielectric_constant();
        let t = 0.035; // 铜厚
        
        TransmissionLine::calculate_microstrip_impedance(width, t, er, h)
    }

    fn get_dielectric_thickness(&self) -> f32 {
        // 获取信号层到参考平面的距离
        for layer in &self.stackup.layers {
            if matches!(layer.layer_type, LayerType::Dielectric) {
                return layer.thickness;
            }
        }
        0.2 // 默认值
    }

    fn get_effective_dielectric_constant(&self) -> f32 {
        // 获取有效介电常数
        for layer in &self.stackup.layers {
            if matches!(layer.layer_type, LayerType::Dielectric) {
                return layer.material.dielectric_constant;
            }
        }
        4.3 // FR4默认值
    }
}

#[derive(Debug)]
pub struct TraceGeometry {
    pub width: f32,
    pub thickness: f32,
    pub spacing_to_plane: f32,
    pub calculated_impedance: f32,
    pub impedance_error: f32,
}
```

### 2. 信号完整性优化建议

```rust
pub struct SIOptimizationEngine {
    pub rules: Vec<DesignRule>,
    pub constraints: Vec<DesignConstraint>,
}

#[derive(Debug)]
pub struct DesignRule {
    pub rule_type: RuleType,
    pub description: String,
    pub severity: RuleSeverity,
    pub check_function: fn(&SignalIntegrityAnalyzer) -> bool,
}

#[derive(Debug)]
pub enum RuleType {
    ImpedanceControl,
    LengthMatching,
    ViaCount,
    CrosstalkSpacing,
    RiseTimeControl,
}

#[derive(Debug)]
pub enum RuleSeverity {
    Error,
    Warning,
    Info,
}

#[derive(Debug)]
pub struct DesignConstraint {
    pub constraint_type: ConstraintType,
    pub value: f32,
    pub tolerance: f32,
    pub units: String,
}

#[derive(Debug)]
pub enum ConstraintType {
    MaxSkew,
    MinSpacing,
    MaxVias,
    MaxLength,
    MinRiseTime,
}

impl SIOptimizationEngine {
    pub fn generate_optimization_report(&self, analyzer: &SignalIntegrityAnalyzer) -> OptimizationReport {
        let mut violations = Vec::new();
        let mut recommendations = Vec::new();
        
        // 检查设计规则
        for rule in &self.rules {
            if !(rule.check_function)(analyzer) {
                violations.push(RuleViolation {
                    rule_type: rule.rule_type.clone(),
                    description: rule.description.clone(),
                    severity: rule.severity.clone(),
                });
                
                // 生成相应的优化建议
                let recommendation = self.generate_recommendation(&rule.rule_type, analyzer);
                recommendations.push(recommendation);
            }
        }
        
        OptimizationReport {
            violations,
            recommendations,
            overall_score: self.calculate_si_score(analyzer),
            critical_issues: self.identify_critical_issues(analyzer),
        }
    }

    fn generate_recommendation(&self, rule_type: &RuleType, analyzer: &SignalIntegrityAnalyzer) -> SIRecommendation {
        match rule_type {
            RuleType::ImpedanceControl => SIRecommendation {
                category: "阻抗控制".to_string(),
                priority: RecommendationPriority::High,
                description: "检测到阻抗不匹配问题".to_string(),
                actions: vec![
                    "调整走线宽度".to_string(),
                    "优化叠层设计".to_string(),
                    "检查参考平面完整性".to_string(),
                ],
                expected_improvement: "减少反射，提高信号质量".to_string(),
            },
            RuleType::CrosstalkSpacing => SIRecommendation {
                category: "串扰控制".to_string(),
                priority: RecommendationPriority::Medium,
                description: "检测到串扰超标".to_string(),
                actions: vec![
                    "增加走线间距".to_string(),
                    "使用差分信号".to_string(),
                    "添加保护走线".to_string(),
                ],
                expected_improvement: "降低串扰，提高信号完整性".to_string(),
            },
            RuleType::LengthMatching => SIRecommendation {
                category: "长度匹配".to_string(),
                priority: RecommendationPriority::Medium,
                description: "检测到长度不匹配".to_string(),
                actions: vec![
                    "调整走线长度".to_string(),
                    "使用蛇形走线".to_string(),
                    "优化布线路径".to_string(),
                ],
                expected_improvement: "减少时序偏差".to_string(),
            },
            _ => SIRecommendation {
                category: "通用优化".to_string(),
                priority: RecommendationPriority::Low,
                description: "一般性信号完整性问题".to_string(),
                actions: vec!["请参考设计指南".to_string()],
                expected_improvement: "提高整体性能".to_string(),
            },
        }
    }

    fn calculate_si_score(&self, analyzer: &SignalIntegrityAnalyzer) -> f32 {
        let mut score = 100.0;
        
        // 基于各种指标计算得分
        let impedance_profile = analyzer.analyze_impedance_profile();
        let frequency_response = analyzer.analyze_frequency_response();
        let crosstalk_analysis = analyzer.analyze_crosstalk();
        
        // 阻抗控制得分
        for discontinuity in &impedance_profile.discontinuities {
            match discontinuity.severity {
                DiscontinuitySeverity::High => score -= 20.0,
                DiscontinuitySeverity::Medium => score -= 10.0,
                DiscontinuitySeverity::Low => score -= 5.0,
            }
        }
        
        // 频率响应得分
        if frequency_response.bandwidth_3db < 100e6 {
            score -= 15.0;
        }
        
        // 串扰得分
        if crosstalk_analysis.worst_case_next > 0.1 {
            score -= 25.0;
        }
        
        score.max(0.0)
    }

    fn identify_critical_issues(&self, analyzer: &SignalIntegrityAnalyzer) -> Vec<CriticalIssue> {
        let mut issues = Vec::new();
        
        let impedance_profile = analyzer.analyze_impedance_profile();
        let crosstalk_analysis = analyzer.analyze_crosstalk();
        
        // 检查严重的阻抗不连续
        for discontinuity in &impedance_profile.discontinuities {
            if discontinuity.severity == DiscontinuitySeverity::High {
                issues.push(CriticalIssue {
                    issue_type: IssueType::ImpedanceDiscontinuity,
                    location: discontinuity.position,
                    severity: IssueSeverity::Critical,
                    description: format!(
                        "严重阻抗不连续: {:.1}Ω -> {:.1}Ω",
                        discontinuity.impedance_before,
                        discontinuity.impedance_after
                    ),
                    impact: "可能导致严重反射和信号失真".to_string(),
                });
            }
        }
        
        // 检查严重串扰
        for crosstalk in &crosstalk_analysis.crosstalk_results {
            if crosstalk.severity == CrosstalkSeverity::High {
                issues.push(CriticalIssue {
                    issue_type: IssueType::ExcessiveCrosstalk,
                    location: 0.0, // 简化
                    severity: IssueSeverity::Critical,
                    description: format!(
                        "严重串扰: NEXT={:.1}%, FEXT={:.1}%",
                        crosstalk.near_end_crosstalk * 100.0,
                        crosstalk.far_end_crosstalk * 100.0
                    ),
                    impact: "可能导致信号错误和系统不稳定".to_string(),
                });
            }
        }
        
        issues
    }
}

#[derive(Debug)]
pub struct OptimizationReport {
    pub violations: Vec<RuleViolation>,
    pub recommendations: Vec<SIRecommendation>,
    pub overall_score: f32,
    pub critical_issues: Vec<CriticalIssue>,
}

#[derive(Debug)]
pub struct RuleViolation {
    pub rule_type: RuleType,
    pub description: String,
    pub severity: RuleSeverity,
}

#[derive(Debug)]
pub struct SIRecommendation {
    pub category: String,
    pub priority: RecommendationPriority,
    pub description: String,
    pub actions: Vec<String>,
    pub expected_improvement: String,
}

#[derive(Debug, PartialEq)]
pub enum RecommendationPriority {
    High,
    Medium,
    Low,
}

#[derive(Debug)]
pub struct CriticalIssue {
    pub issue_type: IssueType,
    pub location: f32,
    pub severity: IssueSeverity,
    pub description: String,
    pub impact: String,
}

#[derive(Debug)]
pub enum IssueType {
    ImpedanceDiscontinuity,
    ExcessiveCrosstalk,
    InsufficientBandwidth,
    TimingViolation,
}

#[derive(Debug)]
pub enum IssueSeverity {
    Critical,
    Major,
    Minor,
}
```

## 最佳实践和设计指南

### 1. 高速信号设计原则

1. **阻抗控制**
   - 保持50Ω或100Ω差分阻抗
   - 避免阻抗突变
   - 使用连续的参考平面

2. **长度匹配**
   - 关键信号长度匹配在±0.1mm内
   - 使用蛇形走线进行长度调整
   - 考虑过孔的等效长度

3. **串扰控制**
   - 保持3W规则（线间距≥3倍线宽）
   - 使用差分信号传输
   - 添加保护走线

4. **过孔优化**
   - 最小化过孔数量
   - 使用盲孔和埋孔
   - 优化过孔尺寸

### 2. PCB布局指南

1. **叠层设计**
   - 信号层紧邻参考平面
   - 控制介质厚度
   - 合理安排电源和地平面

2. **走线规划**
   - 避免直角转弯
   - 使用45°或圆弧转弯
   - 保持走线连续性

3. **电源完整性**
   - 充足的去耦电容
   - 低阻抗电源分配网络
   - 避免电源平面分割

### 3. 仿真和验证

1. **前仿真**
   - 阻抗仿真
   - 串扰分析
   - 时序分析

2. **后仿真**
   - 提取寄生参数
   - 全波仿真
   - 眼图分析

3. **测试验证**
   - TDR测试
   - 网络分析
   - 眼图测量

## 结论

信号完整性是高速ADC/DAC系统设计的关键因素。通过系统性的分析、仿真和优化，可以确保信号在传输过程中保持良好的完整性，从而提高系统的整体性能和可靠性。

本文档提供的分析工具和设计指南可以帮助工程师在设计阶段就识别和解决潜在的信号完整性问题，避免在后期调试中遇到困难。在实际应用中，应结合具体的系统要求和约束条件，选择合适的优化策略。