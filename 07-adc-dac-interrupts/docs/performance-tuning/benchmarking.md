# STM32F4 ADC/DAC 系统基准测试指南

## 概述

基准测试是性能优化的基础，通过系统性的测试和评估，可以准确了解ADC/DAC系统的性能特征，识别瓶颈，并验证优化效果。本文档提供了全面的基准测试方法、工具和最佳实践。

## 基准测试框架

### 1. 测试框架架构

```rust
#[derive(Debug)]
pub struct BenchmarkFramework {
    pub test_suite: TestSuite,
    pub measurement_engine: MeasurementEngine,
    pub data_collector: DataCollector,
    pub report_generator: ReportGenerator,
    pub configuration: BenchmarkConfig,
}

#[derive(Debug)]
pub struct TestSuite {
    pub adc_tests: Vec<AdcBenchmark>,
    pub dac_tests: Vec<DacBenchmark>,
    pub system_tests: Vec<SystemBenchmark>,
    pub stress_tests: Vec<StressTest>,
    pub regression_tests: Vec<RegressionTest>,
}

#[derive(Debug)]
pub struct BenchmarkConfig {
    pub test_duration: u32,
    pub sample_count: usize,
    pub warmup_time: u32,
    pub cooldown_time: u32,
    pub repetitions: u32,
    pub statistical_confidence: f32,
    pub output_format: OutputFormat,
}

#[derive(Debug)]
pub enum OutputFormat {
    Console,
    Json,
    Csv,
    Html,
    Binary,
}

impl BenchmarkFramework {
    pub fn new(config: BenchmarkConfig) -> Self {
        Self {
            test_suite: TestSuite::new(),
            measurement_engine: MeasurementEngine::new(),
            data_collector: DataCollector::new(),
            report_generator: ReportGenerator::new(),
            configuration: config,
        }
    }

    pub fn run_all_benchmarks(&mut self) -> BenchmarkResults {
        let mut results = BenchmarkResults::new();
        
        // 运行ADC基准测试
        for test in &self.test_suite.adc_tests {
            let result = self.run_adc_benchmark(test);
            results.adc_results.push(result);
        }
        
        // 运行DAC基准测试
        for test in &self.test_suite.dac_tests {
            let result = self.run_dac_benchmark(test);
            results.dac_results.push(result);
        }
        
        // 运行系统基准测试
        for test in &self.test_suite.system_tests {
            let result = self.run_system_benchmark(test);
            results.system_results.push(result);
        }
        
        // 运行压力测试
        for test in &self.test_suite.stress_tests {
            let result = self.run_stress_test(test);
            results.stress_results.push(result);
        }
        
        results
    }

    fn run_adc_benchmark(&mut self, test: &AdcBenchmark) -> AdcBenchmarkResult {
        // 预热
        self.warmup(test.warmup_duration);
        
        let mut measurements = Vec::new();
        
        for _ in 0..self.configuration.repetitions {
            let measurement = self.measure_adc_performance(test);
            measurements.push(measurement);
            
            // 冷却时间
            self.cooldown(self.configuration.cooldown_time);
        }
        
        AdcBenchmarkResult {
            test_name: test.name.clone(),
            measurements,
            statistics: self.calculate_statistics(&measurements),
            timestamp: self.get_timestamp(),
        }
    }

    fn measure_adc_performance(&mut self, test: &AdcBenchmark) -> AdcMeasurement {
        let start_time = self.get_high_precision_time();
        
        // 配置ADC
        self.configure_adc(&test.adc_config);
        
        // 执行测试
        let mut samples = Vec::new();
        let mut conversion_times = Vec::new();
        
        for _ in 0..test.sample_count {
            let conversion_start = self.get_high_precision_time();
            let sample = self.read_adc_sample();
            let conversion_end = self.get_high_precision_time();
            
            samples.push(sample);
            conversion_times.push(conversion_end - conversion_start);
        }
        
        let end_time = self.get_high_precision_time();
        let total_time = end_time - start_time;
        
        AdcMeasurement {
            total_time,
            sample_count: test.sample_count,
            throughput: test.sample_count as f32 / total_time,
            conversion_times,
            samples,
            cpu_usage: self.measure_cpu_usage(),
            memory_usage: self.measure_memory_usage(),
            power_consumption: self.measure_power_consumption(),
        }
    }
}
```

### 2. ADC基准测试

```rust
#[derive(Debug)]
pub struct AdcBenchmark {
    pub name: String,
    pub description: String,
    pub adc_config: AdcTestConfig,
    pub sample_count: usize,
    pub warmup_duration: u32,
    pub test_type: AdcTestType,
    pub expected_performance: PerformanceExpectation,
}

#[derive(Debug)]
pub struct AdcTestConfig {
    pub resolution: AdcResolution,
    pub sampling_rate: u32,
    pub channel_count: u8,
    pub conversion_mode: ConversionMode,
    pub trigger_source: TriggerSource,
    pub dma_enabled: bool,
    pub oversampling: Option<OversamplingConfig>,
}

#[derive(Debug)]
pub enum AdcTestType {
    SingleChannel,
    MultiChannel,
    ContinuousConversion,
    TriggeredConversion,
    DmaTransfer,
    InterruptDriven,
    HighSpeed,
    HighPrecision,
    LowPower,
}

#[derive(Debug)]
pub struct AdcMeasurement {
    pub total_time: f32,
    pub sample_count: usize,
    pub throughput: f32,
    pub conversion_times: Vec<f32>,
    pub samples: Vec<u16>,
    pub cpu_usage: f32,
    pub memory_usage: usize,
    pub power_consumption: f32,
}

#[derive(Debug)]
pub struct AdcBenchmarkResult {
    pub test_name: String,
    pub measurements: Vec<AdcMeasurement>,
    pub statistics: BenchmarkStatistics,
    pub timestamp: u32,
}

impl AdcBenchmark {
    pub fn single_channel_speed_test() -> Self {
        Self {
            name: "单通道速度测试".to_string(),
            description: "测试单通道ADC的最大转换速度".to_string(),
            adc_config: AdcTestConfig {
                resolution: AdcResolution::Bits12,
                sampling_rate: 2_000_000, // 2 MSPS
                channel_count: 1,
                conversion_mode: ConversionMode::Continuous,
                trigger_source: TriggerSource::Software,
                dma_enabled: true,
                oversampling: None,
            },
            sample_count: 10000,
            warmup_duration: 100,
            test_type: AdcTestType::SingleChannel,
            expected_performance: PerformanceExpectation {
                min_throughput: 1_800_000.0,
                max_latency: 1.0,
                max_cpu_usage: 20.0,
            },
        }
    }

    pub fn multi_channel_test() -> Self {
        Self {
            name: "多通道测试".to_string(),
            description: "测试多通道ADC的性能".to_string(),
            adc_config: AdcTestConfig {
                resolution: AdcResolution::Bits12,
                sampling_rate: 1_000_000,
                channel_count: 8,
                conversion_mode: ConversionMode::Scan,
                trigger_source: TriggerSource::Timer,
                dma_enabled: true,
                oversampling: None,
            },
            sample_count: 8000,
            warmup_duration: 200,
            test_type: AdcTestType::MultiChannel,
            expected_performance: PerformanceExpectation {
                min_throughput: 800_000.0,
                max_latency: 2.0,
                max_cpu_usage: 30.0,
            },
        }
    }

    pub fn precision_test() -> Self {
        Self {
            name: "精度测试".to_string(),
            description: "测试ADC的转换精度和稳定性".to_string(),
            adc_config: AdcTestConfig {
                resolution: AdcResolution::Bits12,
                sampling_rate: 100_000,
                channel_count: 1,
                conversion_mode: ConversionMode::Single,
                trigger_source: TriggerSource::Software,
                dma_enabled: false,
                oversampling: Some(OversamplingConfig {
                    ratio: 16,
                    shift: 4,
                }),
            },
            sample_count: 1000,
            warmup_duration: 500,
            test_type: AdcTestType::HighPrecision,
            expected_performance: PerformanceExpectation {
                min_throughput: 90_000.0,
                max_latency: 50.0,
                max_cpu_usage: 10.0,
            },
        }
    }
}
```

### 3. DAC基准测试

```rust
#[derive(Debug)]
pub struct DacBenchmark {
    pub name: String,
    pub description: String,
    pub dac_config: DacTestConfig,
    pub sample_count: usize,
    pub test_type: DacTestType,
    pub signal_config: SignalConfig,
    pub expected_performance: PerformanceExpectation,
}

#[derive(Debug)]
pub struct DacTestConfig {
    pub resolution: DacResolution,
    pub update_rate: u32,
    pub channel_count: u8,
    pub output_buffer: bool,
    pub trigger_source: TriggerSource,
    pub dma_enabled: bool,
    pub waveform_generation: bool,
}

#[derive(Debug)]
pub enum DacTestType {
    StaticOutput,
    WaveformGeneration,
    HighSpeed,
    HighPrecision,
    DualChannel,
    TriggeredOutput,
}

#[derive(Debug)]
pub struct SignalConfig {
    pub waveform_type: WaveformType,
    pub frequency: f32,
    pub amplitude: f32,
    pub offset: f32,
    pub phase: f32,
}

#[derive(Debug)]
pub enum WaveformType {
    Sine,
    Square,
    Triangle,
    Sawtooth,
    Noise,
    Custom(Vec<u16>),
}

#[derive(Debug)]
pub struct DacMeasurement {
    pub update_rate: f32,
    pub settling_time: f32,
    pub linearity_error: f32,
    pub thd: f32, // Total Harmonic Distortion
    pub snr: f32, // Signal-to-Noise Ratio
    pub cpu_usage: f32,
    pub memory_usage: usize,
    pub power_consumption: f32,
}

impl DacBenchmark {
    pub fn waveform_generation_test() -> Self {
        Self {
            name: "波形生成测试".to_string(),
            description: "测试DAC波形生成性能".to_string(),
            dac_config: DacTestConfig {
                resolution: DacResolution::Bits12,
                update_rate: 1_000_000,
                channel_count: 1,
                output_buffer: true,
                trigger_source: TriggerSource::Timer,
                dma_enabled: true,
                waveform_generation: true,
            },
            sample_count: 10000,
            test_type: DacTestType::WaveformGeneration,
            signal_config: SignalConfig {
                waveform_type: WaveformType::Sine,
                frequency: 1000.0,
                amplitude: 1.0,
                offset: 0.0,
                phase: 0.0,
            },
            expected_performance: PerformanceExpectation {
                min_throughput: 900_000.0,
                max_latency: 2.0,
                max_cpu_usage: 25.0,
            },
        }
    }

    pub fn precision_test() -> Self {
        Self {
            name: "DAC精度测试".to_string(),
            description: "测试DAC输出精度和线性度".to_string(),
            dac_config: DacTestConfig {
                resolution: DacResolution::Bits12,
                update_rate: 100_000,
                channel_count: 1,
                output_buffer: true,
                trigger_source: TriggerSource::Software,
                dma_enabled: false,
                waveform_generation: false,
            },
            sample_count: 4096, // 全量程测试
            test_type: DacTestType::HighPrecision,
            signal_config: SignalConfig {
                waveform_type: WaveformType::Custom(Self::generate_ramp_pattern()),
                frequency: 0.0,
                amplitude: 1.0,
                offset: 0.0,
                phase: 0.0,
            },
            expected_performance: PerformanceExpectation {
                min_throughput: 95_000.0,
                max_latency: 10.0,
                max_cpu_usage: 15.0,
            },
        }
    }

    fn generate_ramp_pattern() -> Vec<u16> {
        (0..4096).collect()
    }
}
```

### 4. 系统级基准测试

```rust
#[derive(Debug)]
pub struct SystemBenchmark {
    pub name: String,
    pub description: String,
    pub test_scenario: TestScenario,
    pub duration: u32,
    pub load_profile: LoadProfile,
    pub metrics: Vec<SystemMetric>,
}

#[derive(Debug)]
pub enum TestScenario {
    NormalOperation,
    PeakLoad,
    MixedWorkload,
    RealTimeResponse,
    PowerEfficiency,
    ThermalStress,
}

#[derive(Debug)]
pub struct LoadProfile {
    pub adc_load: AdcLoadConfig,
    pub dac_load: DacLoadConfig,
    pub cpu_load: CpuLoadConfig,
    pub memory_load: MemoryLoadConfig,
}

#[derive(Debug)]
pub struct AdcLoadConfig {
    pub active_channels: u8,
    pub sampling_rate: u32,
    pub conversion_mode: ConversionMode,
    pub interrupt_frequency: u32,
}

#[derive(Debug)]
pub struct DacLoadConfig {
    pub active_channels: u8,
    pub update_rate: u32,
    pub waveform_complexity: WaveformComplexity,
    pub buffer_size: usize,
}

#[derive(Debug)]
pub enum WaveformComplexity {
    Simple,    // 简单波形
    Moderate,  // 中等复杂度
    Complex,   // 复杂波形
    Maximum,   // 最大复杂度
}

#[derive(Debug)]
pub struct SystemMeasurement {
    pub timestamp: u32,
    pub cpu_utilization: f32,
    pub memory_usage: MemoryUsage,
    pub interrupt_statistics: InterruptStatistics,
    pub thermal_data: ThermalData,
    pub power_metrics: PowerMetrics,
    pub performance_counters: PerformanceCounters,
}

#[derive(Debug)]
pub struct MemoryUsage {
    pub heap_used: usize,
    pub heap_free: usize,
    pub stack_used: usize,
    pub stack_free: usize,
    pub fragmentation: f32,
}

#[derive(Debug)]
pub struct InterruptStatistics {
    pub total_interrupts: u32,
    pub adc_interrupts: u32,
    pub dac_interrupts: u32,
    pub timer_interrupts: u32,
    pub dma_interrupts: u32,
    pub average_latency: f32,
    pub max_latency: f32,
    pub missed_interrupts: u32,
}

#[derive(Debug)]
pub struct ThermalData {
    pub core_temperature: f32,
    pub ambient_temperature: f32,
    pub thermal_throttling: bool,
    pub cooling_efficiency: f32,
}

#[derive(Debug)]
pub struct PowerMetrics {
    pub total_power: f32,
    pub core_power: f32,
    pub peripheral_power: f32,
    pub voltage_levels: VoltageLevel,
    pub current_consumption: f32,
    pub efficiency: f32,
}

#[derive(Debug)]
pub struct VoltageLevel {
    pub vdd: f32,
    pub vdda: f32,
    pub vref: f32,
    pub vbat: f32,
}

impl SystemBenchmark {
    pub fn mixed_workload_test() -> Self {
        Self {
            name: "混合工作负载测试".to_string(),
            description: "模拟真实应用场景的混合工作负载".to_string(),
            test_scenario: TestScenario::MixedWorkload,
            duration: 60000, // 60秒
            load_profile: LoadProfile {
                adc_load: AdcLoadConfig {
                    active_channels: 4,
                    sampling_rate: 500_000,
                    conversion_mode: ConversionMode::Scan,
                    interrupt_frequency: 1000,
                },
                dac_load: DacLoadConfig {
                    active_channels: 2,
                    update_rate: 100_000,
                    waveform_complexity: WaveformComplexity::Moderate,
                    buffer_size: 1024,
                },
                cpu_load: CpuLoadConfig {
                    background_tasks: 3,
                    computation_intensity: 0.7,
                    context_switches: 100,
                },
                memory_load: MemoryLoadConfig {
                    allocation_rate: 1000,
                    deallocation_rate: 950,
                    fragmentation_target: 0.1,
                },
            },
            metrics: vec![
                SystemMetric::CpuUtilization,
                SystemMetric::MemoryUsage,
                SystemMetric::InterruptLatency,
                SystemMetric::Throughput,
                SystemMetric::PowerConsumption,
                SystemMetric::Temperature,
            ],
        }
    }

    pub fn real_time_response_test() -> Self {
        Self {
            name: "实时响应测试".to_string(),
            description: "测试系统实时响应能力".to_string(),
            test_scenario: TestScenario::RealTimeResponse,
            duration: 30000,
            load_profile: LoadProfile {
                adc_load: AdcLoadConfig {
                    active_channels: 1,
                    sampling_rate: 1_000_000,
                    conversion_mode: ConversionMode::Continuous,
                    interrupt_frequency: 10000,
                },
                dac_load: DacLoadConfig {
                    active_channels: 1,
                    update_rate: 1_000_000,
                    waveform_complexity: WaveformComplexity::Simple,
                    buffer_size: 512,
                },
                cpu_load: CpuLoadConfig {
                    background_tasks: 1,
                    computation_intensity: 0.3,
                    context_switches: 50,
                },
                memory_load: MemoryLoadConfig {
                    allocation_rate: 100,
                    deallocation_rate: 100,
                    fragmentation_target: 0.05,
                },
            },
            metrics: vec![
                SystemMetric::InterruptLatency,
                SystemMetric::ResponseTime,
                SystemMetric::Jitter,
                SystemMetric::Determinism,
            ],
        }
    }
}
```

### 5. 压力测试

```rust
#[derive(Debug)]
pub struct StressTest {
    pub name: String,
    pub description: String,
    pub stress_type: StressType,
    pub duration: u32,
    pub intensity: StressIntensity,
    pub failure_criteria: FailureCriteria,
}

#[derive(Debug)]
pub enum StressType {
    CpuStress,
    MemoryStress,
    InterruptStorm,
    ThermalStress,
    PowerStress,
    CombinedStress,
}

#[derive(Debug)]
pub enum StressIntensity {
    Low,
    Medium,
    High,
    Extreme,
}

#[derive(Debug)]
pub struct FailureCriteria {
    pub max_error_rate: f32,
    pub max_response_time: f32,
    pub min_throughput: f32,
    pub max_temperature: f32,
    pub max_power_consumption: f32,
}

#[derive(Debug)]
pub struct StressTestResult {
    pub test_name: String,
    pub duration: u32,
    pub peak_metrics: SystemMeasurement,
    pub failure_points: Vec<FailurePoint>,
    pub recovery_time: Option<u32>,
    pub stability_score: f32,
}

#[derive(Debug)]
pub struct FailurePoint {
    pub timestamp: u32,
    pub failure_type: FailureType,
    pub severity: FailureSeverity,
    pub description: String,
    pub recovery_action: Option<String>,
}

#[derive(Debug)]
pub enum FailureType {
    PerformanceDegradation,
    SystemHang,
    DataCorruption,
    MemoryLeak,
    ThermalThrottling,
    PowerFailure,
}

#[derive(Debug)]
pub enum FailureSeverity {
    Minor,
    Major,
    Critical,
    Catastrophic,
}

impl StressTest {
    pub fn interrupt_storm_test() -> Self {
        Self {
            name: "中断风暴测试".to_string(),
            description: "测试系统在高频中断下的稳定性".to_string(),
            stress_type: StressType::InterruptStorm,
            duration: 10000,
            intensity: StressIntensity::High,
            failure_criteria: FailureCriteria {
                max_error_rate: 0.01,
                max_response_time: 100.0,
                min_throughput: 500_000.0,
                max_temperature: 85.0,
                max_power_consumption: 200.0,
            },
        }
    }

    pub fn thermal_stress_test() -> Self {
        Self {
            name: "热应力测试".to_string(),
            description: "测试系统在高温环境下的性能".to_string(),
            stress_type: StressType::ThermalStress,
            duration: 30000,
            intensity: StressIntensity::Extreme,
            failure_criteria: FailureCriteria {
                max_error_rate: 0.001,
                max_response_time: 50.0,
                min_throughput: 800_000.0,
                max_temperature: 90.0,
                max_power_consumption: 250.0,
            },
        }
    }
}
```

## 测量引擎

### 1. 高精度时间测量

```rust
#[derive(Debug)]
pub struct MeasurementEngine {
    pub timer_config: TimerConfig,
    pub calibration_data: CalibrationData,
    pub measurement_cache: MeasurementCache,
}

#[derive(Debug)]
pub struct TimerConfig {
    pub high_resolution_timer: bool,
    pub timer_frequency: u32,
    pub calibration_enabled: bool,
    pub drift_compensation: bool,
}

impl MeasurementEngine {
    pub fn get_high_precision_time(&self) -> f32 {
        // 使用DWT CYCCNT寄存器获取高精度时间
        let cycles = self.read_cycle_counter();
        cycles as f32 / self.timer_config.timer_frequency as f32
    }

    pub fn measure_execution_time<F, R>(&self, func: F) -> (R, f32)
    where
        F: FnOnce() -> R,
    {
        let start = self.get_high_precision_time();
        let result = func();
        let end = self.get_high_precision_time();
        (result, end - start)
    }

    pub fn measure_cpu_usage(&self) -> f32 {
        // 测量CPU使用率
        let idle_time = self.measure_idle_time();
        let total_time = self.get_measurement_window();
        (1.0 - idle_time / total_time) * 100.0
    }

    pub fn measure_memory_usage(&self) -> MemoryUsage {
        // 测量内存使用情况
        MemoryUsage {
            heap_used: self.get_heap_used(),
            heap_free: self.get_heap_free(),
            stack_used: self.get_stack_used(),
            stack_free: self.get_stack_free(),
            fragmentation: self.calculate_fragmentation(),
        }
    }

    pub fn measure_power_consumption(&self) -> f32 {
        // 测量功耗（需要外部功耗测量电路）
        self.read_power_sensor()
    }

    fn read_cycle_counter(&self) -> u32 {
        // 读取DWT CYCCNT寄存器
        unsafe {
            core::ptr::read_volatile(0xE0001004 as *const u32)
        }
    }

    fn measure_idle_time(&self) -> f32 {
        // 测量空闲时间
        0.0 // 简化实现
    }

    fn get_measurement_window(&self) -> f32 {
        // 获取测量窗口时间
        1.0 // 简化实现
    }

    fn get_heap_used(&self) -> usize {
        // 获取堆使用量
        0 // 简化实现
    }

    fn get_heap_free(&self) -> usize {
        // 获取堆空闲量
        0 // 简化实现
    }

    fn get_stack_used(&self) -> usize {
        // 获取栈使用量
        0 // 简化实现
    }

    fn get_stack_free(&self) -> usize {
        // 获取栈空闲量
        0 // 简化实现
    }

    fn calculate_fragmentation(&self) -> f32 {
        // 计算内存碎片率
        0.0 // 简化实现
    }

    fn read_power_sensor(&self) -> f32 {
        // 读取功耗传感器
        0.0 // 简化实现
    }
}
```

### 2. 统计分析

```rust
#[derive(Debug, Clone)]
pub struct BenchmarkStatistics {
    pub mean: f32,
    pub median: f32,
    pub std_deviation: f32,
    pub min: f32,
    pub max: f32,
    pub percentiles: Percentiles,
    pub confidence_interval: ConfidenceInterval,
    pub outliers: Vec<f32>,
}

#[derive(Debug, Clone)]
pub struct Percentiles {
    pub p25: f32,
    pub p50: f32,
    pub p75: f32,
    pub p90: f32,
    pub p95: f32,
    pub p99: f32,
}

#[derive(Debug, Clone)]
pub struct ConfidenceInterval {
    pub confidence_level: f32,
    pub lower_bound: f32,
    pub upper_bound: f32,
}

impl BenchmarkStatistics {
    pub fn calculate(data: &[f32]) -> Self {
        if data.is_empty() {
            return Self::default();
        }

        let mut sorted_data = data.to_vec();
        sorted_data.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let mean = data.iter().sum::<f32>() / data.len() as f32;
        let median = Self::calculate_median(&sorted_data);
        let std_deviation = Self::calculate_std_deviation(data, mean);
        let min = sorted_data[0];
        let max = sorted_data[sorted_data.len() - 1];
        let percentiles = Self::calculate_percentiles(&sorted_data);
        let confidence_interval = Self::calculate_confidence_interval(data, mean, std_deviation, 0.95);
        let outliers = Self::detect_outliers(data, mean, std_deviation);

        Self {
            mean,
            median,
            std_deviation,
            min,
            max,
            percentiles,
            confidence_interval,
            outliers,
        }
    }

    fn calculate_median(sorted_data: &[f32]) -> f32 {
        let len = sorted_data.len();
        if len % 2 == 0 {
            (sorted_data[len / 2 - 1] + sorted_data[len / 2]) / 2.0
        } else {
            sorted_data[len / 2]
        }
    }

    fn calculate_std_deviation(data: &[f32], mean: f32) -> f32 {
        let variance = data.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>() / data.len() as f32;
        variance.sqrt()
    }

    fn calculate_percentiles(sorted_data: &[f32]) -> Percentiles {
        let len = sorted_data.len();
        
        Percentiles {
            p25: sorted_data[len / 4],
            p50: Self::calculate_median(sorted_data),
            p75: sorted_data[3 * len / 4],
            p90: sorted_data[9 * len / 10],
            p95: sorted_data[95 * len / 100],
            p99: sorted_data[99 * len / 100],
        }
    }

    fn calculate_confidence_interval(data: &[f32], mean: f32, std_dev: f32, confidence: f32) -> ConfidenceInterval {
        let n = data.len() as f32;
        let t_value = Self::get_t_value(confidence, n - 1.0);
        let margin_of_error = t_value * std_dev / n.sqrt();
        
        ConfidenceInterval {
            confidence_level: confidence,
            lower_bound: mean - margin_of_error,
            upper_bound: mean + margin_of_error,
        }
    }

    fn get_t_value(confidence: f32, df: f32) -> f32 {
        // 简化的t值计算，实际应用中应使用更精确的方法
        match confidence {
            0.90 => 1.645,
            0.95 => 1.96,
            0.99 => 2.576,
            _ => 1.96,
        }
    }

    fn detect_outliers(data: &[f32], mean: f32, std_dev: f32) -> Vec<f32> {
        let threshold = 2.0 * std_dev;
        data.iter()
            .filter(|&&x| (x - mean).abs() > threshold)
            .cloned()
            .collect()
    }
}

impl Default for BenchmarkStatistics {
    fn default() -> Self {
        Self {
            mean: 0.0,
            median: 0.0,
            std_deviation: 0.0,
            min: 0.0,
            max: 0.0,
            percentiles: Percentiles {
                p25: 0.0,
                p50: 0.0,
                p75: 0.0,
                p90: 0.0,
                p95: 0.0,
                p99: 0.0,
            },
            confidence_interval: ConfidenceInterval {
                confidence_level: 0.95,
                lower_bound: 0.0,
                upper_bound: 0.0,
            },
            outliers: Vec::new(),
        }
    }
}
```

## 报告生成

### 1. 基准测试报告

```rust
#[derive(Debug)]
pub struct ReportGenerator {
    pub template_engine: TemplateEngine,
    pub export_formats: Vec<ExportFormat>,
    pub visualization_engine: VisualizationEngine,
}

#[derive(Debug)]
pub enum ExportFormat {
    Html,
    Pdf,
    Json,
    Csv,
    Markdown,
}

#[derive(Debug)]
pub struct BenchmarkReport {
    pub header: ReportHeader,
    pub executive_summary: ExecutiveSummary,
    pub test_results: TestResults,
    pub performance_analysis: PerformanceAnalysis,
    pub recommendations: Vec<Recommendation>,
    pub appendices: Vec<Appendix>,
}

#[derive(Debug)]
pub struct ReportHeader {
    pub title: String,
    pub version: String,
    pub generation_time: String,
    pub test_environment: TestEnvironment,
    pub system_configuration: SystemConfiguration,
}

#[derive(Debug)]
pub struct TestEnvironment {
    pub hardware_platform: String,
    pub firmware_version: String,
    pub compiler_version: String,
    pub test_conditions: TestConditions,
}

#[derive(Debug)]
pub struct TestConditions {
    pub temperature: f32,
    pub humidity: f32,
    pub supply_voltage: f32,
    pub clock_frequency: u32,
}

#[derive(Debug)]
pub struct ExecutiveSummary {
    pub overall_performance: OverallPerformance,
    pub key_findings: Vec<KeyFinding>,
    pub performance_highlights: Vec<PerformanceHighlight>,
    pub areas_for_improvement: Vec<ImprovementArea>,
}

#[derive(Debug)]
pub struct OverallPerformance {
    pub performance_score: f32,
    pub efficiency_rating: EfficiencyRating,
    pub stability_rating: StabilityRating,
    pub power_efficiency: f32,
}

#[derive(Debug)]
pub enum EfficiencyRating {
    Excellent,
    Good,
    Fair,
    Poor,
}

#[derive(Debug)]
pub enum StabilityRating {
    VeryStable,
    Stable,
    Moderate,
    Unstable,
}

impl ReportGenerator {
    pub fn generate_report(&self, results: &BenchmarkResults) -> BenchmarkReport {
        BenchmarkReport {
            header: self.generate_header(),
            executive_summary: self.generate_executive_summary(results),
            test_results: self.format_test_results(results),
            performance_analysis: self.analyze_performance(results),
            recommendations: self.generate_recommendations(results),
            appendices: self.generate_appendices(results),
        }
    }

    fn generate_header(&self) -> ReportHeader {
        ReportHeader {
            title: "STM32F4 ADC/DAC 基准测试报告".to_string(),
            version: "1.0".to_string(),
            generation_time: self.get_current_time_string(),
            test_environment: TestEnvironment {
                hardware_platform: "STM32F407VGT6".to_string(),
                firmware_version: "1.0.0".to_string(),
                compiler_version: "rustc 1.70.0".to_string(),
                test_conditions: TestConditions {
                    temperature: 25.0,
                    humidity: 45.0,
                    supply_voltage: 3.3,
                    clock_frequency: 168_000_000,
                },
            },
            system_configuration: SystemConfiguration {
                adc_config: "12-bit, 2.4 MSPS".to_string(),
                dac_config: "12-bit, 1 MSPS".to_string(),
                memory_config: "192KB RAM, 1MB Flash".to_string(),
                optimization_level: "Release (-O3)".to_string(),
            },
        }
    }

    fn generate_executive_summary(&self, results: &BenchmarkResults) -> ExecutiveSummary {
        let overall_performance = self.calculate_overall_performance(results);
        let key_findings = self.extract_key_findings(results);
        let performance_highlights = self.identify_performance_highlights(results);
        let areas_for_improvement = self.identify_improvement_areas(results);

        ExecutiveSummary {
            overall_performance,
            key_findings,
            performance_highlights,
            areas_for_improvement,
        }
    }

    fn calculate_overall_performance(&self, results: &BenchmarkResults) -> OverallPerformance {
        // 计算综合性能评分
        let adc_score = self.calculate_adc_performance_score(&results.adc_results);
        let dac_score = self.calculate_dac_performance_score(&results.dac_results);
        let system_score = self.calculate_system_performance_score(&results.system_results);
        
        let performance_score = (adc_score + dac_score + system_score) / 3.0;
        
        OverallPerformance {
            performance_score,
            efficiency_rating: self.rate_efficiency(performance_score),
            stability_rating: self.rate_stability(results),
            power_efficiency: self.calculate_power_efficiency(results),
        }
    }

    fn rate_efficiency(&self, score: f32) -> EfficiencyRating {
        match score {
            s if s >= 90.0 => EfficiencyRating::Excellent,
            s if s >= 75.0 => EfficiencyRating::Good,
            s if s >= 60.0 => EfficiencyRating::Fair,
            _ => EfficiencyRating::Poor,
        }
    }

    fn rate_stability(&self, results: &BenchmarkResults) -> StabilityRating {
        // 基于测试结果评估稳定性
        let error_rate = self.calculate_error_rate(results);
        match error_rate {
            r if r < 0.001 => StabilityRating::VeryStable,
            r if r < 0.01 => StabilityRating::Stable,
            r if r < 0.1 => StabilityRating::Moderate,
            _ => StabilityRating::Unstable,
        }
    }

    pub fn export_report(&self, report: &BenchmarkReport, format: ExportFormat) -> Result<Vec<u8>, ExportError> {
        match format {
            ExportFormat::Html => self.export_html(report),
            ExportFormat::Json => self.export_json(report),
            ExportFormat::Csv => self.export_csv(report),
            ExportFormat::Markdown => self.export_markdown(report),
            ExportFormat::Pdf => self.export_pdf(report),
        }
    }

    fn export_html(&self, report: &BenchmarkReport) -> Result<Vec<u8>, ExportError> {
        // 生成HTML报告
        let html_content = self.template_engine.render_html_template(report)?;
        Ok(html_content.into_bytes())
    }

    fn export_json(&self, report: &BenchmarkReport) -> Result<Vec<u8>, ExportError> {
        // 生成JSON报告
        let json_content = serde_json::to_string_pretty(report)
            .map_err(|e| ExportError::SerializationError(e.to_string()))?;
        Ok(json_content.into_bytes())
    }
}
```

## 最佳实践

### 1. 测试设计原则

1. **可重复性**: 确保测试结果可重复
2. **隔离性**: 每个测试应该独立运行
3. **代表性**: 测试场景应该代表真实使用情况
4. **全面性**: 覆盖所有重要的性能指标

### 2. 测试环境控制

- **温度控制**: 保持稳定的测试温度
- **电源稳定**: 使用稳定的电源供应
- **时钟精度**: 确保时钟源的精度和稳定性
- **负载隔离**: 避免其他任务干扰测试

### 3. 数据分析指南

- **统计显著性**: 确保足够的样本量
- **异常值处理**: 正确识别和处理异常值
- **趋势分析**: 关注性能趋势变化
- **对比分析**: 与基准值和历史数据对比

## 结论

基准测试是性能优化的重要工具，通过系统性的测试和分析，可以：

1. **建立性能基准**: 为系统优化提供参考标准
2. **识别性能瓶颈**: 发现系统的薄弱环节
3. **验证优化效果**: 量化优化带来的改进
4. **指导设计决策**: 为系统设计提供数据支持

通过遵循本文档的方法和最佳实践，开发者可以建立完善的基准测试体系，为STM32F4 ADC/DAC系统的性能优化提供有力支持。