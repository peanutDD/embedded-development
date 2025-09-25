# I2C/SPI 调试指南

## 概述

通信协议调试是嵌入式开发中的重要技能。本指南提供了系统性的I2C和SPI协议调试方法、工具使用和常见问题解决方案。

## 调试工具

### 硬件调试工具

#### 逻辑分析仪
```rust
pub struct LogicAnalyzer {
    pub channels: u8,
    pub sample_rate: u32,
    pub memory_depth: u32,
    pub trigger_conditions: Vec<TriggerCondition>,
}

pub enum TriggerCondition {
    I2cStart,
    I2cStop,
    I2cAddress(u8),
    SpiChipSelect,
    SpiClockEdge,
    DataPattern(Vec<u8>),
}

impl LogicAnalyzer {
    pub fn setup_i2c_capture(&mut self, frequency: u32) -> Result<(), DebugError> {
        self.sample_rate = frequency * 10; // 10倍过采样
        self.trigger_conditions.push(TriggerCondition::I2cStart);
        Ok(())
    }
    
    pub fn setup_spi_capture(&mut self, frequency: u32) -> Result<(), DebugError> {
        self.sample_rate = frequency * 4; // 4倍过采样
        self.trigger_conditions.push(TriggerCondition::SpiChipSelect);
        Ok(())
    }
}
```

#### 示波器
```rust
pub struct Oscilloscope {
    pub channels: u8,
    pub bandwidth: u32,
    pub sample_rate: u32,
    pub trigger_level: f32,
}

impl Oscilloscope {
    pub fn measure_signal_quality(&self, signal: &[f32]) -> SignalQuality {
        SignalQuality {
            rise_time: self.calculate_rise_time(signal),
            fall_time: self.calculate_fall_time(signal),
            overshoot: self.calculate_overshoot(signal),
            noise_level: self.calculate_noise_level(signal),
        }
    }
    
    pub fn check_timing(&self, clock: &[f32], data: &[f32]) -> TimingAnalysis {
        TimingAnalysis {
            setup_time: self.calculate_setup_time(clock, data),
            hold_time: self.calculate_hold_time(clock, data),
            clock_jitter: self.calculate_jitter(clock),
        }
    }
}

pub struct SignalQuality {
    pub rise_time: f32,
    pub fall_time: f32,
    pub overshoot: f32,
    pub noise_level: f32,
}

pub struct TimingAnalysis {
    pub setup_time: f32,
    pub hold_time: f32,
    pub clock_jitter: f32,
}
```

### 软件调试工具

#### 协议分析器
```rust
pub struct ProtocolAnalyzer {
    pub i2c_decoder: I2cDecoder,
    pub spi_decoder: SpiDecoder,
    pub error_detector: ErrorDetector,
}

pub struct I2cDecoder {
    pub state: I2cState,
    pub current_address: Option<u8>,
    pub data_buffer: Vec<u8>,
}

pub enum I2cState {
    Idle,
    Start,
    Address,
    Data,
    Ack,
    Stop,
}

impl I2cDecoder {
    pub fn decode_frame(&mut self, sda: bool, scl: bool) -> Option<I2cEvent> {
        match self.state {
            I2cState::Idle => {
                if !sda && scl {
                    self.state = I2cState::Start;
                    Some(I2cEvent::Start)
                } else {
                    None
                }
            },
            I2cState::Start => {
                self.state = I2cState::Address;
                None
            },
            // ... 其他状态处理
            _ => None,
        }
    }
}

pub enum I2cEvent {
    Start,
    Stop,
    Address(u8, bool), // 地址和R/W位
    Data(u8),
    Ack,
    Nack,
    Error(I2cError),
}
```

#### 实时监控
```rust
pub struct RealTimeMonitor {
    pub bus_utilization: f32,
    pub error_count: u32,
    pub transaction_count: u32,
    pub average_latency: f32,
}

impl RealTimeMonitor {
    pub fn update_statistics(&mut self, event: &ProtocolEvent) {
        match event {
            ProtocolEvent::I2c(i2c_event) => {
                self.update_i2c_stats(i2c_event);
            },
            ProtocolEvent::Spi(spi_event) => {
                self.update_spi_stats(spi_event);
            },
        }
    }
    
    pub fn generate_report(&self) -> MonitoringReport {
        MonitoringReport {
            bus_utilization: self.bus_utilization,
            error_rate: self.error_count as f32 / self.transaction_count as f32,
            average_latency: self.average_latency,
            recommendations: self.generate_recommendations(),
        }
    }
}
```

## I2C 调试

### 常见问题诊断

#### 通信失败诊断
```rust
pub struct I2cDiagnostic {
    pub bus_analyzer: BusAnalyzer,
    pub signal_checker: SignalChecker,
}

impl I2cDiagnostic {
    pub fn diagnose_communication_failure(&self) -> DiagnosticResult {
        let mut issues = Vec::new();
        
        // 检查上拉电阻
        if !self.check_pullup_resistors() {
            issues.push(DiagnosticIssue::MissingPullupResistors);
        }
        
        // 检查信号电平
        if !self.check_signal_levels() {
            issues.push(DiagnosticIssue::InvalidSignalLevels);
        }
        
        // 检查时钟频率
        if !self.check_clock_frequency() {
            issues.push(DiagnosticIssue::InvalidClockFrequency);
        }
        
        // 检查设备地址
        if !self.check_device_addresses() {
            issues.push(DiagnosticIssue::AddressConflict);
        }
        
        DiagnosticResult { issues }
    }
    
    pub fn check_pullup_resistors(&self) -> bool {
        // 检查SDA和SCL的上拉电阻
        let sda_high = self.signal_checker.measure_high_level("SDA");
        let scl_high = self.signal_checker.measure_high_level("SCL");
        
        sda_high > 0.7 * 3.3 && scl_high > 0.7 * 3.3
    }
    
    pub fn scan_bus(&self) -> Vec<u8> {
        let mut found_devices = Vec::new();
        
        for addr in 0x08..=0x77 {
            if self.probe_address(addr) {
                found_devices.push(addr);
            }
        }
        
        found_devices
    }
}

pub enum DiagnosticIssue {
    MissingPullupResistors,
    InvalidSignalLevels,
    InvalidClockFrequency,
    AddressConflict,
    TimingViolation,
    NoiseInterference,
}
```

#### 时序分析
```rust
pub struct I2cTimingAnalyzer {
    pub setup_time_min: f32,
    pub hold_time_min: f32,
    pub clock_low_min: f32,
    pub clock_high_min: f32,
}

impl I2cTimingAnalyzer {
    pub fn analyze_timing(&self, capture_data: &CaptureData) -> TimingReport {
        let mut violations = Vec::new();
        
        for transaction in &capture_data.transactions {
            // 检查建立时间
            if transaction.setup_time < self.setup_time_min {
                violations.push(TimingViolation::SetupTime {
                    measured: transaction.setup_time,
                    required: self.setup_time_min,
                });
            }
            
            // 检查保持时间
            if transaction.hold_time < self.hold_time_min {
                violations.push(TimingViolation::HoldTime {
                    measured: transaction.hold_time,
                    required: self.hold_time_min,
                });
            }
        }
        
        TimingReport { violations }
    }
}

pub enum TimingViolation {
    SetupTime { measured: f32, required: f32 },
    HoldTime { measured: f32, required: f32 },
    ClockLow { measured: f32, required: f32 },
    ClockHigh { measured: f32, required: f32 },
}
```

### 调试步骤

#### 基础连接检查
```rust
pub struct I2cConnectionChecker {
    pub multimeter: Multimeter,
    pub signal_generator: SignalGenerator,
}

impl I2cConnectionChecker {
    pub fn check_basic_connections(&self) -> ConnectionReport {
        let mut report = ConnectionReport::new();
        
        // 1. 检查电源连接
        let vcc_voltage = self.multimeter.measure_voltage("VCC");
        report.add_check("VCC", vcc_voltage > 3.0 && vcc_voltage < 3.6);
        
        // 2. 检查地线连接
        let gnd_resistance = self.multimeter.measure_resistance("GND");
        report.add_check("GND", gnd_resistance < 1.0);
        
        // 3. 检查SDA连接
        let sda_continuity = self.multimeter.check_continuity("SDA");
        report.add_check("SDA", sda_continuity);
        
        // 4. 检查SCL连接
        let scl_continuity = self.multimeter.check_continuity("SCL");
        report.add_check("SCL", scl_continuity);
        
        report
    }
    
    pub fn test_pullup_resistors(&self) -> PullupTestResult {
        // 测试上拉电阻值
        let sda_pullup = self.measure_pullup_resistance("SDA");
        let scl_pullup = self.measure_pullup_resistance("SCL");
        
        PullupTestResult {
            sda_resistance: sda_pullup,
            scl_resistance: scl_pullup,
            sda_ok: sda_pullup > 1000.0 && sda_pullup < 10000.0,
            scl_ok: scl_pullup > 1000.0 && scl_pullup < 10000.0,
        }
    }
}
```

#### 协议级调试
```rust
pub struct I2cProtocolDebugger {
    pub transaction_logger: TransactionLogger,
    pub error_analyzer: ErrorAnalyzer,
}

impl I2cProtocolDebugger {
    pub fn debug_transaction(&mut self, address: u8, data: &[u8]) -> DebugResult {
        let mut debug_info = DebugInfo::new();
        
        // 1. 记录事务开始
        debug_info.add_event("Transaction Start", format!("Address: 0x{:02X}", address));
        
        // 2. 发送起始条件
        match self.send_start_condition() {
            Ok(_) => debug_info.add_event("Start Condition", "OK"),
            Err(e) => {
                debug_info.add_error("Start Condition", e);
                return DebugResult::Failed(debug_info);
            }
        }
        
        // 3. 发送地址
        match self.send_address(address, false) {
            Ok(ack) => {
                if ack {
                    debug_info.add_event("Address", "ACK received");
                } else {
                    debug_info.add_error("Address", "NACK received");
                    return DebugResult::Failed(debug_info);
                }
            },
            Err(e) => {
                debug_info.add_error("Address", e);
                return DebugResult::Failed(debug_info);
            }
        }
        
        // 4. 发送数据
        for (i, &byte) in data.iter().enumerate() {
            match self.send_data_byte(byte) {
                Ok(ack) => {
                    if ack {
                        debug_info.add_event(&format!("Data[{}]", i), "ACK received");
                    } else {
                        debug_info.add_error(&format!("Data[{}]", i), "NACK received");
                        break;
                    }
                },
                Err(e) => {
                    debug_info.add_error(&format!("Data[{}]", i), e);
                    break;
                }
            }
        }
        
        // 5. 发送停止条件
        match self.send_stop_condition() {
            Ok(_) => debug_info.add_event("Stop Condition", "OK"),
            Err(e) => debug_info.add_error("Stop Condition", e),
        }
        
        DebugResult::Success(debug_info)
    }
}
```

## SPI 调试

### 常见问题诊断

#### 模式配置问题
```rust
pub struct SpiModeDebugger {
    pub mode_tester: ModeCompatibilityTester,
    pub timing_analyzer: SpiTimingAnalyzer,
}

impl SpiModeDebugger {
    pub fn diagnose_mode_issues(&self, master_mode: SpiMode, slave_mode: SpiMode) -> ModeReport {
        let mut issues = Vec::new();
        
        // 检查CPOL匹配
        if master_mode.cpol != slave_mode.cpol {
            issues.push(ModeIssue::CpolMismatch {
                master: master_mode.cpol,
                slave: slave_mode.cpol,
            });
        }
        
        // 检查CPHA匹配
        if master_mode.cpha != slave_mode.cpha {
            issues.push(ModeIssue::CphaMismatch {
                master: master_mode.cpha,
                slave: slave_mode.cpha,
            });
        }
        
        // 检查位序
        if master_mode.bit_order != slave_mode.bit_order {
            issues.push(ModeIssue::BitOrderMismatch {
                master: master_mode.bit_order,
                slave: slave_mode.bit_order,
            });
        }
        
        ModeReport { issues }
    }
    
    pub fn test_all_modes(&self) -> Vec<ModeTestResult> {
        let mut results = Vec::new();
        
        for mode in 0..4 {
            let test_result = self.test_mode(SpiMode::from_mode_number(mode));
            results.push(test_result);
        }
        
        results
    }
}

pub enum ModeIssue {
    CpolMismatch { master: bool, slave: bool },
    CphaMismatch { master: bool, slave: bool },
    BitOrderMismatch { master: BitOrder, slave: BitOrder },
}
```

#### 信号完整性分析
```rust
pub struct SpiSignalAnalyzer {
    pub oscilloscope: Oscilloscope,
    pub spectrum_analyzer: SpectrumAnalyzer,
}

impl SpiSignalAnalyzer {
    pub fn analyze_signal_integrity(&self, frequency: u32) -> SignalIntegrityReport {
        let mut report = SignalIntegrityReport::new();
        
        // 分析时钟信号
        let clock_analysis = self.analyze_clock_signal(frequency);
        report.clock_quality = clock_analysis;
        
        // 分析数据信号
        let data_analysis = self.analyze_data_signals();
        report.data_quality = data_analysis;
        
        // 分析片选信号
        let cs_analysis = self.analyze_chip_select();
        report.cs_quality = cs_analysis;
        
        // 检查串扰
        let crosstalk = self.measure_crosstalk();
        report.crosstalk_level = crosstalk;
        
        report
    }
    
    pub fn measure_eye_diagram(&self, data_signal: &[f32]) -> EyeDiagram {
        // 生成眼图分析
        EyeDiagram {
            eye_height: self.calculate_eye_height(data_signal),
            eye_width: self.calculate_eye_width(data_signal),
            jitter: self.calculate_jitter(data_signal),
            noise_margin: self.calculate_noise_margin(data_signal),
        }
    }
}

pub struct SignalIntegrityReport {
    pub clock_quality: ClockQuality,
    pub data_quality: DataQuality,
    pub cs_quality: ChipSelectQuality,
    pub crosstalk_level: f32,
}
```

### 调试工具使用

#### 逻辑分析仪配置
```rust
pub struct SpiLogicAnalyzerSetup {
    pub channel_mapping: ChannelMapping,
    pub trigger_setup: TriggerSetup,
    pub decode_settings: DecodeSettings,
}

pub struct ChannelMapping {
    pub sck_channel: u8,
    pub mosi_channel: u8,
    pub miso_channel: u8,
    pub cs_channel: u8,
}

impl SpiLogicAnalyzerSetup {
    pub fn configure_for_spi(&mut self, frequency: u32) -> Result<(), ConfigError> {
        // 设置采样率
        let sample_rate = frequency * 10; // 10倍过采样
        
        // 配置触发条件
        self.trigger_setup = TriggerSetup {
            trigger_channel: self.channel_mapping.cs_channel,
            trigger_edge: TriggerEdge::Falling,
            pre_trigger_samples: 100,
            post_trigger_samples: 1000,
        };
        
        // 配置解码器
        self.decode_settings = DecodeSettings {
            protocol: Protocol::Spi,
            clock_channel: self.channel_mapping.sck_channel,
            data_channels: vec![
                self.channel_mapping.mosi_channel,
                self.channel_mapping.miso_channel,
            ],
            chip_select_channel: Some(self.channel_mapping.cs_channel),
            mode: SpiMode::default(),
        };
        
        Ok(())
    }
}
```

#### 自动化测试框架
```rust
pub struct SpiTestFramework {
    pub test_patterns: Vec<TestPattern>,
    pub expected_responses: Vec<Vec<u8>>,
    pub error_injection: ErrorInjection,
}

impl SpiTestFramework {
    pub fn run_comprehensive_test(&mut self) -> TestReport {
        let mut report = TestReport::new();
        
        // 基础功能测试
        report.add_section(self.run_basic_functionality_tests());
        
        // 边界条件测试
        report.add_section(self.run_boundary_tests());
        
        // 错误注入测试
        report.add_section(self.run_error_injection_tests());
        
        // 性能测试
        report.add_section(self.run_performance_tests());
        
        report
    }
    
    pub fn run_loopback_test(&self) -> TestResult {
        let test_data = vec![0x55, 0xAA, 0xFF, 0x00];
        let mut received_data = vec![0; test_data.len()];
        
        // 配置回环模式
        self.configure_loopback();
        
        // 发送测试数据
        match self.transfer(&test_data, &mut received_data) {
            Ok(_) => {
                if test_data == received_data {
                    TestResult::Passed
                } else {
                    TestResult::Failed(format!(
                        "Data mismatch: sent {:?}, received {:?}",
                        test_data, received_data
                    ))
                }
            },
            Err(e) => TestResult::Error(e.to_string()),
        }
    }
}
```

## 调试最佳实践

### 系统性调试方法

#### 分层调试策略
```rust
pub struct LayeredDebugging {
    pub physical_layer: PhysicalLayerDebug,
    pub protocol_layer: ProtocolLayerDebug,
    pub application_layer: ApplicationLayerDebug,
}

impl LayeredDebugging {
    pub fn debug_communication_issue(&self) -> DebugStrategy {
        let mut strategy = DebugStrategy::new();
        
        // 1. 物理层检查
        strategy.add_step(DebugStep {
            layer: Layer::Physical,
            description: "检查硬件连接和信号质量".to_string(),
            actions: vec![
                "验证电源和地线连接".to_string(),
                "检查信号线连续性".to_string(),
                "测量信号电平和时序".to_string(),
            ],
        });
        
        // 2. 协议层检查
        strategy.add_step(DebugStep {
            layer: Layer::Protocol,
            description: "验证协议配置和时序".to_string(),
            actions: vec![
                "确认协议参数配置".to_string(),
                "检查时序要求".to_string(),
                "验证数据格式".to_string(),
            ],
        });
        
        // 3. 应用层检查
        strategy.add_step(DebugStep {
            layer: Layer::Application,
            description: "检查应用逻辑和数据处理".to_string(),
            actions: vec![
                "验证数据处理逻辑".to_string(),
                "检查错误处理机制".to_string(),
                "确认状态机正确性".to_string(),
            ],
        });
        
        strategy
    }
}
```

### 调试工具集成

#### 多工具协同调试
```rust
pub struct IntegratedDebugEnvironment {
    pub logic_analyzer: LogicAnalyzer,
    pub oscilloscope: Oscilloscope,
    pub protocol_analyzer: ProtocolAnalyzer,
    pub software_debugger: SoftwareDebugger,
}

impl IntegratedDebugEnvironment {
    pub fn synchronized_capture(&mut self, duration_ms: u32) -> CaptureResult {
        // 同步触发所有工具
        let trigger_time = self.get_synchronized_trigger();
        
        // 启动捕获
        let logic_capture = self.logic_analyzer.start_capture(trigger_time, duration_ms);
        let scope_capture = self.oscilloscope.start_capture(trigger_time, duration_ms);
        let protocol_capture = self.protocol_analyzer.start_capture(trigger_time, duration_ms);
        
        // 等待捕获完成
        let results = CaptureResult {
            logic_data: logic_capture.wait_for_completion(),
            analog_data: scope_capture.wait_for_completion(),
            protocol_data: protocol_capture.wait_for_completion(),
            timestamp: trigger_time,
        };
        
        results
    }
    
    pub fn correlate_data(&self, capture: &CaptureResult) -> CorrelationReport {
        // 关联不同工具的数据
        CorrelationReport {
            timing_correlation: self.correlate_timing(&capture.logic_data, &capture.analog_data),
            protocol_correlation: self.correlate_protocol(&capture.logic_data, &capture.protocol_data),
            anomaly_detection: self.detect_anomalies(capture),
        }
    }
}
```

## 常见问题解决方案

### I2C 问题解决

#### 地址冲突解决
```rust
pub struct I2cAddressManager {
    pub device_registry: HashMap<u8, DeviceInfo>,
    pub address_scanner: AddressScanner,
}

impl I2cAddressManager {
    pub fn resolve_address_conflicts(&mut self) -> ConflictResolution {
        let conflicts = self.detect_conflicts();
        let mut resolutions = Vec::new();
        
        for conflict in conflicts {
            match self.resolve_single_conflict(conflict) {
                Some(resolution) => resolutions.push(resolution),
                None => {
                    // 无法自动解决的冲突
                    resolutions.push(ConflictResolution::ManualIntervention(conflict));
                }
            }
        }
        
        ConflictResolution::Multiple(resolutions)
    }
    
    pub fn suggest_alternative_addresses(&self, conflicted_address: u8) -> Vec<u8> {
        let mut alternatives = Vec::new();
        
        for addr in 0x08..=0x77 {
            if !self.device_registry.contains_key(&addr) && 
               self.is_address_available(addr) {
                alternatives.push(addr);
            }
        }
        
        alternatives
    }
}
```

#### 时钟延展处理
```rust
pub struct ClockStretchingHandler {
    pub timeout_ms: u32,
    pub retry_count: u8,
}

impl ClockStretchingHandler {
    pub fn handle_clock_stretching(&self) -> Result<(), I2cError> {
        let start_time = get_timestamp();
        
        while self.is_clock_stretched() {
            if get_timestamp() - start_time > self.timeout_ms {
                return Err(I2cError::ClockStretchTimeout);
            }
            
            // 短暂延时后重新检查
            delay_us(10);
        }
        
        Ok(())
    }
    
    pub fn configure_clock_stretching(&mut self, enable: bool, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
        // 配置硬件时钟延展支持
    }
}
```

### SPI 问题解决

#### 时序问题修复
```rust
pub struct SpiTimingFixer {
    pub timing_analyzer: TimingAnalyzer,
    pub parameter_optimizer: ParameterOptimizer,
}

impl SpiTimingFixer {
    pub fn fix_timing_violations(&mut self, violations: &[TimingViolation]) -> FixResult {
        let mut fixes = Vec::new();
        
        for violation in violations {
            match violation {
                TimingViolation::SetupTime { measured, required } => {
                    let fix = self.fix_setup_time(*measured, *required);
                    fixes.push(fix);
                },
                TimingViolation::HoldTime { measured, required } => {
                    let fix = self.fix_hold_time(*measured, *required);
                    fixes.push(fix);
                },
                // 处理其他时序违规
                _ => {}
            }
        }
        
        FixResult { fixes }
    }
    
    pub fn optimize_clock_frequency(&self, target_frequency: u32) -> OptimizationResult {
        let constraints = self.analyze_timing_constraints();
        let max_safe_frequency = self.calculate_max_frequency(&constraints);
        
        OptimizationResult {
            recommended_frequency: max_safe_frequency.min(target_frequency),
            timing_margins: self.calculate_margins(max_safe_frequency),
            warnings: self.generate_warnings(&constraints),
        }
    }
}
```

## 调试报告生成

### 自动化报告系统
```rust
pub struct DebugReportGenerator {
    pub template_engine: TemplateEngine,
    pub data_formatter: DataFormatter,
    pub chart_generator: ChartGenerator,
}

impl DebugReportGenerator {
    pub fn generate_comprehensive_report(&self, debug_session: &DebugSession) -> Report {
        let mut report = Report::new();
        
        // 执行摘要
        report.add_section(self.generate_executive_summary(debug_session));
        
        // 问题分析
        report.add_section(self.generate_problem_analysis(debug_session));
        
        // 测试结果
        report.add_section(self.generate_test_results(debug_session));
        
        // 信号分析
        report.add_section(self.generate_signal_analysis(debug_session));
        
        // 建议和解决方案
        report.add_section(self.generate_recommendations(debug_session));
        
        // 附录
        report.add_section(self.generate_appendix(debug_session));
        
        report
    }
    
    pub fn export_report(&self, report: &Report, format: ReportFormat) -> Result<Vec<u8>, ExportError> {
        match format {
            ReportFormat::Pdf => self.export_to_pdf(report),
            ReportFormat::Html => self.export_to_html(report),
            ReportFormat::Json => self.export_to_json(report),
            ReportFormat::Csv => self.export_to_csv(report),
        }
    }
}
```

## 结论

有效的调试需要：

1. **系统性方法**: 从物理层到应用层的分层调试
2. **合适的工具**: 选择和正确使用调试工具
3. **经验积累**: 建立问题模式识别能力
4. **文档记录**: 记录调试过程和解决方案

通过遵循本指南的方法和最佳实践，可以显著提高I2C和SPI协议调试的效率和成功率。