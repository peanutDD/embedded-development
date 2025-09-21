//! UART信号分析器 - 深度分析串口通信特性
//! 
//! 本项目实现了一个全面的UART信号分析工具，用于：
//! - 硬件电气特性分析
//! - 时序参数测量
//! - 协议层面解析
//! - 错误检测与统计

#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::asm;
use heapless::{Vec, String};
use nb::block;

#[cfg(feature = "stm32")]
use stm32f4xx_hal as hal;
#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal;

/// UART电气特性分析器
pub struct UartElectricalAnalyzer {
    /// 电压电平测量
    voltage_levels: VoltageAnalysis,
    /// 电流消耗分析
    current_analysis: CurrentAnalysis,
    /// 阻抗特性
    impedance_characteristics: ImpedanceAnalysis,
}

/// 电压电平分析
#[derive(Debug, Clone)]
pub struct VoltageAnalysis {
    /// 逻辑高电平范围
    pub voh_range: (f32, f32),  // (min, max) in volts
    /// 逻辑低电平范围
    pub vol_range: (f32, f32),  // (min, max) in volts
    /// 输入阈值电压
    pub vih_threshold: f32,     // Input high threshold
    pub vil_threshold: f32,     // Input low threshold
    /// 噪声容限
    pub noise_margin_high: f32,
    pub noise_margin_low: f32,
}

/// 电流分析
#[derive(Debug, Clone)]
pub struct CurrentAnalysis {
    /// 静态电流消耗
    pub quiescent_current: f32,  // μA
    /// 动态电流消耗
    pub dynamic_current: f32,    // μA
    /// 短路保护电流
    pub short_circuit_current: f32, // mA
    /// 驱动能力
    pub drive_capability: f32,   // mA
}

/// 阻抗分析
#[derive(Debug, Clone)]
pub struct ImpedanceAnalysis {
    /// 输出阻抗
    pub output_impedance: f32,   // Ohms
    /// 输入阻抗
    pub input_impedance: f32,    // Ohms
    /// 传输线阻抗匹配
    pub transmission_line_impedance: f32, // Ohms (typically 50Ω or 75Ω)
}

impl UartElectricalAnalyzer {
    pub fn new() -> Self {
        Self {
            voltage_levels: VoltageAnalysis {
                voh_range: (2.4, 5.0),  // TTL/CMOS levels
                vol_range: (0.0, 0.4),
                vih_threshold: 2.0,
                vil_threshold: 0.8,
                noise_margin_high: 0.4,
                noise_margin_low: 0.4,
            },
            current_analysis: CurrentAnalysis {
                quiescent_current: 1.0,
                dynamic_current: 5.0,
                short_circuit_current: 25.0,
                drive_capability: 12.0,
            },
            impedance_characteristics: ImpedanceAnalysis {
                output_impedance: 50.0,
                input_impedance: 10000.0,
                transmission_line_impedance: 50.0,
            },
        }
    }

    /// 分析电气特性兼容性
    pub fn analyze_compatibility(&self, target_specs: &VoltageAnalysis) -> CompatibilityReport {
        let voltage_compatible = self.check_voltage_compatibility(target_specs);
        let noise_margin_adequate = self.check_noise_margins(target_specs);
        
        CompatibilityReport {
            voltage_compatible,
            noise_margin_adequate,
            recommended_termination: self.calculate_termination(),
        }
    }

    fn check_voltage_compatibility(&self, target: &VoltageAnalysis) -> bool {
        // 检查输出电平是否满足输入要求
        self.voltage_levels.voh_range.0 >= target.vih_threshold &&
        self.voltage_levels.vol_range.1 <= target.vil_threshold
    }

    fn check_noise_margins(&self, target: &VoltageAnalysis) -> bool {
        let high_margin = self.voltage_levels.voh_range.0 - target.vih_threshold;
        let low_margin = target.vil_threshold - self.voltage_levels.vol_range.1;
        
        high_margin >= 0.4 && low_margin >= 0.4  // 最小400mV噪声容限
    }

    fn calculate_termination(&self) -> TerminationRecommendation {
        TerminationRecommendation {
            series_resistance: 22.0,  // Ohms
            parallel_resistance: 120.0, // Ohms
            use_ac_termination: true,
        }
    }
}

/// 兼容性报告
#[derive(Debug)]
pub struct CompatibilityReport {
    pub voltage_compatible: bool,
    pub noise_margin_adequate: bool,
    pub recommended_termination: TerminationRecommendation,
}

/// 终端匹配推荐
#[derive(Debug)]
pub struct TerminationRecommendation {
    pub series_resistance: f32,
    pub parallel_resistance: f32,
    pub use_ac_termination: bool,
}

/// UART时序分析器
pub struct UartTimingAnalyzer {
    /// 波特率精度分析
    baud_rate_accuracy: BaudRateAnalysis,
    /// 位时序分析
    bit_timing: BitTimingAnalysis,
    /// 抖动分析
    jitter_analysis: JitterAnalysis,
}

/// 波特率分析
#[derive(Debug, Clone)]
pub struct BaudRateAnalysis {
    /// 标称波特率
    pub nominal_baud_rate: u32,
    /// 实际波特率
    pub actual_baud_rate: f32,
    /// 波特率误差 (%)
    pub baud_rate_error: f32,
    /// 累积误差
    pub cumulative_error: f32,
    /// 最大可容忍误差
    pub max_tolerable_error: f32,
}

/// 位时序分析
#[derive(Debug, Clone)]
pub struct BitTimingAnalysis {
    /// 位周期
    pub bit_period: f32,        // microseconds
    /// 起始位时长
    pub start_bit_duration: f32,
    /// 数据位时长
    pub data_bit_duration: f32,
    /// 停止位时长
    pub stop_bit_duration: f32,
    /// 建立时间
    pub setup_time: f32,        // nanoseconds
    /// 保持时间
    pub hold_time: f32,         // nanoseconds
}

/// 抖动分析
#[derive(Debug, Clone)]
pub struct JitterAnalysis {
    /// 周期抖动
    pub period_jitter: f32,     // picoseconds RMS
    /// 周期到周期抖动
    pub cycle_to_cycle_jitter: f32,
    /// 累积抖动
    pub accumulated_jitter: f32,
    /// 抖动传递函数
    pub jitter_transfer_function: Vec<f32, 64>,
}

impl UartTimingAnalyzer {
    pub fn new(baud_rate: u32) -> Self {
        let bit_period = 1_000_000.0 / baud_rate as f32; // microseconds
        
        Self {
            baud_rate_accuracy: BaudRateAnalysis {
                nominal_baud_rate: baud_rate,
                actual_baud_rate: baud_rate as f32,
                baud_rate_error: 0.0,
                cumulative_error: 0.0,
                max_tolerable_error: 3.0, // 3% for UART
            },
            bit_timing: BitTimingAnalysis {
                bit_period,
                start_bit_duration: bit_period,
                data_bit_duration: bit_period,
                stop_bit_duration: bit_period,
                setup_time: bit_period * 1000.0 * 0.1, // 10% of bit period in ns
                hold_time: bit_period * 1000.0 * 0.1,
            },
            jitter_analysis: JitterAnalysis {
                period_jitter: 100.0,  // 100ps RMS typical
                cycle_to_cycle_jitter: 50.0,
                accumulated_jitter: 200.0,
                jitter_transfer_function: Vec::new(),
            },
        }
    }

    /// 测量实际波特率
    pub fn measure_baud_rate(&mut self, bit_transitions: &[u32]) -> f32 {
        if bit_transitions.len() < 2 {
            return 0.0;
        }

        // 计算位周期的平均值
        let mut total_period = 0u32;
        for i in 1..bit_transitions.len() {
            total_period += bit_transitions[i] - bit_transitions[i-1];
        }
        
        let avg_period = total_period as f32 / (bit_transitions.len() - 1) as f32;
        let measured_baud_rate = 1_000_000.0 / avg_period; // 假设时间戳单位为微秒
        
        // 计算误差
        self.baud_rate_accuracy.actual_baud_rate = measured_baud_rate;
        self.baud_rate_accuracy.baud_rate_error = 
            ((measured_baud_rate - self.baud_rate_accuracy.nominal_baud_rate as f32) / 
             self.baud_rate_accuracy.nominal_baud_rate as f32) * 100.0;
        
        measured_baud_rate
    }

    /// 分析抖动特性
    pub fn analyze_jitter(&mut self, edge_times: &[u32]) -> JitterReport {
        let mut period_variations = Vec::<f32, 32>::new();
        
        // 计算周期变化
        for i in 2..edge_times.len().min(33) {
            let period1 = edge_times[i-1] - edge_times[i-2];
            let period2 = edge_times[i] - edge_times[i-1];
            let variation = (period2 as f32 - period1 as f32).abs();
            let _ = period_variations.push(variation);
        }

        // 计算RMS抖动
        let mean_variation: f32 = period_variations.iter().sum::<f32>() / period_variations.len() as f32;
        let variance: f32 = period_variations.iter()
            .map(|&x| (x - mean_variation).powi(2))
            .sum::<f32>() / period_variations.len() as f32;
        let rms_jitter = variance.sqrt();

        self.jitter_analysis.period_jitter = rms_jitter;

        JitterReport {
            rms_jitter,
            peak_to_peak_jitter: period_variations.iter().cloned().fold(0.0f32, f32::max),
            jitter_frequency_spectrum: self.calculate_jitter_spectrum(&period_variations),
        }
    }

    fn calculate_jitter_spectrum(&self, variations: &[f32]) -> Vec<f32, 16> {
        // 简化的频谱分析 - 实际应用中会使用FFT
        let mut spectrum = Vec::new();
        for i in 0..16 {
            let freq_component = variations.iter()
                .enumerate()
                .map(|(idx, &val)| val * (2.0 * core::f32::consts::PI * i as f32 * idx as f32 / variations.len() as f32).cos())
                .sum::<f32>() / variations.len() as f32;
            let _ = spectrum.push(freq_component.abs());
        }
        spectrum
    }
}

/// 抖动报告
#[derive(Debug)]
pub struct JitterReport {
    pub rms_jitter: f32,
    pub peak_to_peak_jitter: f32,
    pub jitter_frequency_spectrum: Vec<f32, 16>,
}

/// UART协议分析器
pub struct UartProtocolAnalyzer {
    /// 帧格式配置
    frame_format: FrameFormat,
    /// 错误统计
    error_statistics: ErrorStatistics,
    /// 数据完整性检查
    data_integrity: DataIntegrityChecker,
}

/// 帧格式
#[derive(Debug, Clone)]
pub struct FrameFormat {
    pub data_bits: u8,      // 5, 6, 7, 8, 9
    pub parity: ParityType,
    pub stop_bits: StopBits,
    pub flow_control: FlowControl,
}

#[derive(Debug, Clone)]
pub enum ParityType {
    None,
    Even,
    Odd,
    Mark,   // Always 1
    Space,  // Always 0
}

#[derive(Debug, Clone)]
pub enum StopBits {
    One,
    OneAndHalf,
    Two,
}

#[derive(Debug, Clone)]
pub enum FlowControl {
    None,
    Hardware,  // RTS/CTS
    Software,  // XON/XOFF
}

/// 错误统计
#[derive(Debug, Clone)]
pub struct ErrorStatistics {
    pub framing_errors: u32,
    pub parity_errors: u32,
    pub overrun_errors: u32,
    pub break_conditions: u32,
    pub noise_errors: u32,
    pub total_frames: u32,
}

/// 数据完整性检查器
pub struct DataIntegrityChecker {
    /// CRC校验
    crc_enabled: bool,
    /// 校验和
    checksum_enabled: bool,
    /// 序列号检查
    sequence_check: bool,
}

impl UartProtocolAnalyzer {
    pub fn new(frame_format: FrameFormat) -> Self {
        Self {
            frame_format,
            error_statistics: ErrorStatistics {
                framing_errors: 0,
                parity_errors: 0,
                overrun_errors: 0,
                break_conditions: 0,
                noise_errors: 0,
                total_frames: 0,
            },
            data_integrity: DataIntegrityChecker {
                crc_enabled: false,
                checksum_enabled: false,
                sequence_check: false,
            },
        }
    }

    /// 解析UART帧
    pub fn parse_frame(&mut self, raw_data: &[u8]) -> Result<UartFrame, FrameError> {
        if raw_data.is_empty() {
            return Err(FrameError::InsufficientData);
        }

        self.error_statistics.total_frames += 1;

        // 检查起始位
        if raw_data[0] != 0 {
            self.error_statistics.framing_errors += 1;
            return Err(FrameError::InvalidStartBit);
        }

        // 提取数据位
        let data_byte = self.extract_data_bits(&raw_data[1..])?;

        // 校验奇偶位
        if let Some(parity_bit) = self.extract_parity_bit(raw_data) {
            if !self.verify_parity(data_byte, parity_bit) {
                self.error_statistics.parity_errors += 1;
                return Err(FrameError::ParityError);
            }
        }

        // 检查停止位
        if !self.verify_stop_bits(raw_data) {
            self.error_statistics.framing_errors += 1;
            return Err(FrameError::InvalidStopBit);
        }

        Ok(UartFrame {
            data: data_byte,
            timestamp: 0, // 实际应用中会记录时间戳
            frame_format: self.frame_format.clone(),
        })
    }

    fn extract_data_bits(&self, data: &[u8]) -> Result<u8, FrameError> {
        if data.len() < self.frame_format.data_bits as usize {
            return Err(FrameError::InsufficientData);
        }

        let mut result = 0u8;
        for i in 0..self.frame_format.data_bits {
            if data[i as usize] != 0 {
                result |= 1 << i;
            }
        }

        Ok(result)
    }

    fn extract_parity_bit(&self, data: &[u8]) -> Option<u8> {
        match self.frame_format.parity {
            ParityType::None => None,
            _ => {
                let parity_index = 1 + self.frame_format.data_bits as usize;
                if data.len() > parity_index {
                    Some(data[parity_index])
                } else {
                    None
                }
            }
        }
    }

    fn verify_parity(&self, data: u8, parity_bit: u8) -> bool {
        let data_parity = data.count_ones() % 2;
        
        match self.frame_format.parity {
            ParityType::None => true,
            ParityType::Even => (data_parity + parity_bit as u32) % 2 == 0,
            ParityType::Odd => (data_parity + parity_bit as u32) % 2 == 1,
            ParityType::Mark => parity_bit == 1,
            ParityType::Space => parity_bit == 0,
        }
    }

    fn verify_stop_bits(&self, data: &[u8]) -> bool {
        let stop_bit_start = 1 + self.frame_format.data_bits as usize + 
                           if matches!(self.frame_format.parity, ParityType::None) { 0 } else { 1 };
        
        let required_stop_bits = match self.frame_format.stop_bits {
            StopBits::One => 1,
            StopBits::OneAndHalf => 2, // 简化处理
            StopBits::Two => 2,
        };

        if data.len() < stop_bit_start + required_stop_bits {
            return false;
        }

        // 检查停止位都为1
        for i in 0..required_stop_bits {
            if data[stop_bit_start + i] == 0 {
                return false;
            }
        }

        true
    }

    /// 获取错误率统计
    pub fn get_error_rate(&self) -> ErrorRateReport {
        let total = self.error_statistics.total_frames;
        if total == 0 {
            return ErrorRateReport::default();
        }

        ErrorRateReport {
            framing_error_rate: (self.error_statistics.framing_errors as f32 / total as f32) * 100.0,
            parity_error_rate: (self.error_statistics.parity_errors as f32 / total as f32) * 100.0,
            overrun_error_rate: (self.error_statistics.overrun_errors as f32 / total as f32) * 100.0,
            total_error_rate: ((self.error_statistics.framing_errors + 
                               self.error_statistics.parity_errors + 
                               self.error_statistics.overrun_errors) as f32 / total as f32) * 100.0,
        }
    }
}

/// UART帧结构
#[derive(Debug)]
pub struct UartFrame {
    pub data: u8,
    pub timestamp: u32,
    pub frame_format: FrameFormat,
}

/// 帧错误类型
#[derive(Debug)]
pub enum FrameError {
    InsufficientData,
    InvalidStartBit,
    InvalidStopBit,
    ParityError,
    OverrunError,
}

/// 错误率报告
#[derive(Debug, Default)]
pub struct ErrorRateReport {
    pub framing_error_rate: f32,
    pub parity_error_rate: f32,
    pub overrun_error_rate: f32,
    pub total_error_rate: f32,
}

/// 综合UART分析器
pub struct ComprehensiveUartAnalyzer {
    electrical: UartElectricalAnalyzer,
    timing: UartTimingAnalyzer,
    protocol: UartProtocolAnalyzer,
}

impl ComprehensiveUartAnalyzer {
    pub fn new(baud_rate: u32, frame_format: FrameFormat) -> Self {
        Self {
            electrical: UartElectricalAnalyzer::new(),
            timing: UartTimingAnalyzer::new(baud_rate),
            protocol: UartProtocolAnalyzer::new(frame_format),
        }
    }

    /// 执行全面分析
    pub fn comprehensive_analysis(&mut self, signal_data: &SignalData) -> AnalysisReport {
        // 电气特性分析
        let electrical_report = self.electrical.analyze_compatibility(&VoltageAnalysis {
            voh_range: (3.0, 3.3),
            vol_range: (0.0, 0.3),
            vih_threshold: 2.0,
            vil_threshold: 0.8,
            noise_margin_high: 0.4,
            noise_margin_low: 0.4,
        });

        // 时序分析
        let measured_baud_rate = self.timing.measure_baud_rate(&signal_data.bit_transitions);
        let jitter_report = self.timing.analyze_jitter(&signal_data.edge_times);

        // 协议分析
        let mut frame_results = Vec::<Result<UartFrame, FrameError>, 32>::new();
        for frame_data in &signal_data.frame_data {
            let result = self.protocol.parse_frame(frame_data);
            let _ = frame_results.push(result);
        }

        let error_rate = self.protocol.get_error_rate();

        AnalysisReport {
            electrical_compatibility: electrical_report,
            measured_baud_rate,
            jitter_analysis: jitter_report,
            error_rate_analysis: error_rate,
            frame_analysis_results: frame_results,
            recommendations: self.generate_recommendations(),
        }
    }

    fn generate_recommendations(&self) -> Vec<String<64>, 8> {
        let mut recommendations = Vec::new();
        
        let _ = recommendations.push(String::from("使用适当的终端电阻以减少信号反射"));
        let _ = recommendations.push(String::from("确保时钟源稳定性以减少抖动"));
        let _ = recommendations.push(String::from("实施错误检测和重传机制"));
        let _ = recommendations.push(String::from("考虑使用硬件流控制以防止数据丢失"));
        
        recommendations
    }
}

/// 信号数据结构
pub struct SignalData {
    pub bit_transitions: Vec<u32, 128>,
    pub edge_times: Vec<u32, 128>,
    pub frame_data: Vec<Vec<u8, 16>, 32>,
}

/// 综合分析报告
pub struct AnalysisReport {
    pub electrical_compatibility: CompatibilityReport,
    pub measured_baud_rate: f32,
    pub jitter_analysis: JitterReport,
    pub error_rate_analysis: ErrorRateReport,
    pub frame_analysis_results: Vec<Result<UartFrame, FrameError>, 32>,
    pub recommendations: Vec<String<64>, 8>,
}

#[entry]
fn main() -> ! {
    // 初始化系统
    let mut analyzer = ComprehensiveUartAnalyzer::new(
        115200,
        FrameFormat {
            data_bits: 8,
            parity: ParityType::None,
            stop_bits: StopBits::One,
            flow_control: FlowControl::None,
        }
    );

    // 模拟信号数据
    let mut signal_data = SignalData {
        bit_transitions: Vec::new(),
        edge_times: Vec::new(),
        frame_data: Vec::new(),
    };

    // 添加测试数据
    for i in 0..10 {
        let _ = signal_data.bit_transitions.push(i * 87); // ~115200 baud timing
        let _ = signal_data.edge_times.push(i * 87);
    }

    // 添加测试帧数据
    let test_frame = vec![0, 1, 0, 1, 0, 1, 0, 1, 0, 1]; // 起始位 + 数据位 + 停止位
    let mut frame_vec = Vec::new();
    for &bit in &test_frame {
        let _ = frame_vec.push(bit);
    }
    let _ = signal_data.frame_data.push(frame_vec);

    // 执行分析
    let analysis_report = analyzer.comprehensive_analysis(&signal_data);

    // 输出分析结果（在实际硬件上会通过串口或其他方式输出）
    loop {
        // 在实际应用中，这里会：
        // 1. 持续采集UART信号
        // 2. 实时分析信号特性
        // 3. 更新统计信息
        // 4. 输出分析报告
        
        asm::wfi(); // 等待中断
    }
}