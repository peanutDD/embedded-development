# 监控与维护

## 概述

监控与维护是确保嵌入式系统长期稳定运行的关键环节。本文档介绍系统监控、性能分析、故障诊断、预防性维护等方面的技术和实践。

## 系统监控

### 实时监控系统

```rust
// 系统监控框架
use embedded_hal::digital::v2::{InputPin, OutputPin};
use heapless::{Vec, String};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SystemMetrics {
    pub timestamp: u64,
    pub cpu_usage: f32,
    pub memory_usage: MemoryUsage,
    pub temperature: i16,
    pub voltage: f32,
    pub network_stats: NetworkStats,
    pub error_count: u32,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct MemoryUsage {
    pub total: u32,
    pub used: u32,
    pub free: u32,
    pub heap_used: u32,
    pub stack_used: u32,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct NetworkStats {
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub packets_sent: u32,
    pub packets_received: u32,
    pub errors: u32,
    pub dropped: u32,
}

pub struct SystemMonitor {
    metrics_history: Vec<SystemMetrics, 100>,
    alert_thresholds: AlertThresholds,
    alert_handlers: Vec<AlertHandler, 16>,
}

#[derive(Debug, Clone)]
pub struct AlertThresholds {
    pub cpu_usage_critical: f32,
    pub cpu_usage_warning: f32,
    pub memory_usage_critical: f32,
    pub memory_usage_warning: f32,
    pub temperature_critical: i16,
    pub temperature_warning: i16,
    pub voltage_min: f32,
    pub voltage_max: f32,
    pub error_rate_threshold: u32,
}

#[derive(Debug, Clone)]
pub struct AlertHandler {
    pub alert_type: AlertType,
    pub handler: fn(&SystemMetrics, &AlertThresholds) -> AlertAction,
}

#[derive(Debug, Clone, Copy)]
pub enum AlertType {
    CpuUsage,
    MemoryUsage,
    Temperature,
    Voltage,
    NetworkError,
    SystemError,
}

#[derive(Debug, Clone)]
pub enum AlertAction {
    Log(String<128>),
    Notify(String<128>),
    Restart,
    Shutdown,
    ThrottleCpu,
    ClearCache,
}

impl SystemMonitor {
    pub fn new(thresholds: AlertThresholds) -> Self {
        Self {
            metrics_history: Vec::new(),
            alert_thresholds: thresholds,
            alert_handlers: Vec::new(),
        }
    }

    pub fn add_alert_handler(&mut self, handler: AlertHandler) -> Result<(), ()> {
        self.alert_handlers.push(handler).map_err(|_| ())
    }

    pub fn collect_metrics(&mut self) -> SystemMetrics {
        let metrics = SystemMetrics {
            timestamp: self.get_timestamp(),
            cpu_usage: self.get_cpu_usage(),
            memory_usage: self.get_memory_usage(),
            temperature: self.get_temperature(),
            voltage: self.get_voltage(),
            network_stats: self.get_network_stats(),
            error_count: self.get_error_count(),
        };

        // 存储历史数据
        if self.metrics_history.len() >= self.metrics_history.capacity() {
            self.metrics_history.remove(0);
        }
        self.metrics_history.push(metrics.clone()).ok();

        // 检查告警条件
        self.check_alerts(&metrics);

        metrics
    }

    fn check_alerts(&self, metrics: &SystemMetrics) {
        for handler in &self.alert_handlers {
            let action = (handler.handler)(metrics, &self.alert_thresholds);
            self.execute_alert_action(action);
        }
    }

    fn execute_alert_action(&self, action: AlertAction) {
        match action {
            AlertAction::Log(message) => {
                // 记录日志
                self.log_message(&message);
            }
            AlertAction::Notify(message) => {
                // 发送通知
                self.send_notification(&message);
            }
            AlertAction::Restart => {
                // 重启系统
                self.restart_system();
            }
            AlertAction::Shutdown => {
                // 关闭系统
                self.shutdown_system();
            }
            AlertAction::ThrottleCpu => {
                // CPU降频
                self.throttle_cpu();
            }
            AlertAction::ClearCache => {
                // 清理缓存
                self.clear_cache();
            }
        }
    }

    pub fn get_metrics_summary(&self, duration_minutes: u32) -> MetricsSummary {
        let cutoff_time = self.get_timestamp() - (duration_minutes as u64 * 60);
        let recent_metrics: Vec<&SystemMetrics, 100> = self.metrics_history
            .iter()
            .filter(|m| m.timestamp >= cutoff_time)
            .collect();

        if recent_metrics.is_empty() {
            return MetricsSummary::default();
        }

        let mut cpu_sum = 0.0;
        let mut memory_sum = 0.0;
        let mut temp_sum = 0i32;
        let mut voltage_sum = 0.0;
        let mut error_sum = 0u32;

        for metrics in &recent_metrics {
            cpu_sum += metrics.cpu_usage;
            memory_sum += (metrics.memory_usage.used as f32 / metrics.memory_usage.total as f32) * 100.0;
            temp_sum += metrics.temperature as i32;
            voltage_sum += metrics.voltage;
            error_sum += metrics.error_count;
        }

        let count = recent_metrics.len() as f32;
        MetricsSummary {
            avg_cpu_usage: cpu_sum / count,
            avg_memory_usage: memory_sum / count,
            avg_temperature: (temp_sum as f32 / count) as i16,
            avg_voltage: voltage_sum / count,
            total_errors: error_sum,
            sample_count: recent_metrics.len(),
        }
    }

    // 系统指标获取函数
    fn get_cpu_usage(&self) -> f32 {
        // 模拟CPU使用率获取
        25.5
    }

    fn get_memory_usage(&self) -> MemoryUsage {
        // 模拟内存使用情况获取
        MemoryUsage {
            total: 1024 * 1024, // 1MB
            used: 512 * 1024,   // 512KB
            free: 512 * 1024,   // 512KB
            heap_used: 256 * 1024, // 256KB
            stack_used: 64 * 1024,  // 64KB
        }
    }

    fn get_temperature(&self) -> i16 {
        // 模拟温度获取
        45 // 45°C
    }

    fn get_voltage(&self) -> f32 {
        // 模拟电压获取
        3.3 // 3.3V
    }

    fn get_network_stats(&self) -> NetworkStats {
        // 模拟网络统计获取
        NetworkStats {
            bytes_sent: 1024 * 1024,
            bytes_received: 2 * 1024 * 1024,
            packets_sent: 1000,
            packets_received: 2000,
            errors: 5,
            dropped: 2,
        }
    }

    fn get_error_count(&self) -> u32 {
        // 模拟错误计数获取
        10
    }

    fn get_timestamp(&self) -> u64 {
        // 模拟时间戳获取
        1640995200
    }

    // 告警动作执行函数
    fn log_message(&self, message: &str) {
        // 实际实现中会写入日志系统
    }

    fn send_notification(&self, message: &str) {
        // 实际实现中会发送网络通知
    }

    fn restart_system(&self) {
        // 实际实现中会重启系统
    }

    fn shutdown_system(&self) {
        // 实际实现中会关闭系统
    }

    fn throttle_cpu(&self) {
        // 实际实现中会降低CPU频率
    }

    fn clear_cache(&self) {
        // 实际实现中会清理系统缓存
    }
}

#[derive(Debug, Default)]
pub struct MetricsSummary {
    pub avg_cpu_usage: f32,
    pub avg_memory_usage: f32,
    pub avg_temperature: i16,
    pub avg_voltage: f32,
    pub total_errors: u32,
    pub sample_count: usize,
}

// 告警处理函数
pub fn cpu_usage_handler(metrics: &SystemMetrics, thresholds: &AlertThresholds) -> AlertAction {
    if metrics.cpu_usage > thresholds.cpu_usage_critical {
        AlertAction::ThrottleCpu
    } else if metrics.cpu_usage > thresholds.cpu_usage_warning {
        AlertAction::Log(String::from("CPU usage warning"))
    } else {
        AlertAction::Log(String::from("CPU usage normal"))
    }
}

pub fn memory_usage_handler(metrics: &SystemMetrics, thresholds: &AlertThresholds) -> AlertAction {
    let usage_percent = (metrics.memory_usage.used as f32 / metrics.memory_usage.total as f32) * 100.0;
    
    if usage_percent > thresholds.memory_usage_critical {
        AlertAction::ClearCache
    } else if usage_percent > thresholds.memory_usage_warning {
        AlertAction::Log(String::from("Memory usage warning"))
    } else {
        AlertAction::Log(String::from("Memory usage normal"))
    }
}

pub fn temperature_handler(metrics: &SystemMetrics, thresholds: &AlertThresholds) -> AlertAction {
    if metrics.temperature > thresholds.temperature_critical {
        AlertAction::Shutdown
    } else if metrics.temperature > thresholds.temperature_warning {
        AlertAction::Notify(String::from("Temperature warning"))
    } else {
        AlertAction::Log(String::from("Temperature normal"))
    }
}
```

### 性能分析

```rust
// 性能分析工具
use heapless::{Vec, String};
use core::mem;

pub struct PerformanceProfiler {
    profiles: Vec<PerformanceProfile, 32>,
    current_profile: Option<PerformanceProfile>,
}

#[derive(Debug, Clone)]
pub struct PerformanceProfile {
    pub name: String<32>,
    pub start_time: u64,
    pub end_time: u64,
    pub duration_us: u64,
    pub cpu_cycles: u64,
    pub memory_peak: u32,
    pub function_calls: Vec<FunctionCall, 16>,
}

#[derive(Debug, Clone)]
pub struct FunctionCall {
    pub name: String<32>,
    pub call_count: u32,
    pub total_time_us: u64,
    pub avg_time_us: u64,
    pub max_time_us: u64,
    pub min_time_us: u64,
}

impl PerformanceProfiler {
    pub fn new() -> Self {
        Self {
            profiles: Vec::new(),
            current_profile: None,
        }
    }

    pub fn start_profile(&mut self, name: &str) {
        let profile = PerformanceProfile {
            name: String::from(name),
            start_time: self.get_timestamp(),
            end_time: 0,
            duration_us: 0,
            cpu_cycles: self.get_cpu_cycles(),
            memory_peak: self.get_memory_usage(),
            function_calls: Vec::new(),
        };

        self.current_profile = Some(profile);
    }

    pub fn end_profile(&mut self) -> Option<PerformanceProfile> {
        if let Some(mut profile) = self.current_profile.take() {
            profile.end_time = self.get_timestamp();
            profile.duration_us = profile.end_time - profile.start_time;
            profile.cpu_cycles = self.get_cpu_cycles() - profile.cpu_cycles;

            self.profiles.push(profile.clone()).ok();
            Some(profile)
        } else {
            None
        }
    }

    pub fn record_function_call(&mut self, function_name: &str, duration_us: u64) {
        if let Some(ref mut profile) = self.current_profile {
            // 查找现有函数调用记录
            for call in &mut profile.function_calls {
                if call.name == function_name {
                    call.call_count += 1;
                    call.total_time_us += duration_us;
                    call.avg_time_us = call.total_time_us / call.call_count as u64;
                    call.max_time_us = call.max_time_us.max(duration_us);
                    call.min_time_us = call.min_time_us.min(duration_us);
                    return;
                }
            }

            // 创建新的函数调用记录
            let new_call = FunctionCall {
                name: String::from(function_name),
                call_count: 1,
                total_time_us: duration_us,
                avg_time_us: duration_us,
                max_time_us: duration_us,
                min_time_us: duration_us,
            };

            profile.function_calls.push(new_call).ok();
        }
    }

    pub fn get_performance_report(&self) -> PerformanceReport {
        let mut report = PerformanceReport {
            total_profiles: self.profiles.len(),
            total_duration_us: 0,
            avg_duration_us: 0,
            hotspots: Vec::new(),
            memory_analysis: MemoryAnalysis::default(),
        };

        if self.profiles.is_empty() {
            return report;
        }

        // 计算总时间和平均时间
        for profile in &self.profiles {
            report.total_duration_us += profile.duration_us;
        }
        report.avg_duration_us = report.total_duration_us / self.profiles.len() as u64;

        // 分析热点函数
        let mut function_stats: Vec<(String<32>, u64, u32), 32> = Vec::new();
        
        for profile in &self.profiles {
            for call in &profile.function_calls {
                // 查找现有统计
                let mut found = false;
                for (name, total_time, call_count) in &mut function_stats {
                    if *name == call.name {
                        *total_time += call.total_time_us;
                        *call_count += call.call_count;
                        found = true;
                        break;
                    }
                }

                if !found {
                    function_stats.push((call.name.clone(), call.total_time_us, call.call_count)).ok();
                }
            }
        }

        // 按总时间排序，找出热点
        function_stats.sort_by(|a, b| b.1.cmp(&a.1));
        
        for (name, total_time, call_count) in function_stats.iter().take(5) {
            let hotspot = Hotspot {
                function_name: name.clone(),
                total_time_us: *total_time,
                call_count: *call_count,
                avg_time_us: *total_time / *call_count as u64,
                percentage: (*total_time as f32 / report.total_duration_us as f32) * 100.0,
            };
            report.hotspots.push(hotspot).ok();
        }

        // 内存分析
        let mut peak_memory = 0;
        let mut avg_memory = 0u64;
        for profile in &self.profiles {
            peak_memory = peak_memory.max(profile.memory_peak);
            avg_memory += profile.memory_peak as u64;
        }
        avg_memory /= self.profiles.len() as u64;

        report.memory_analysis = MemoryAnalysis {
            peak_usage: peak_memory,
            avg_usage: avg_memory as u32,
            current_usage: self.get_memory_usage(),
            fragmentation: self.calculate_fragmentation(),
        };

        report
    }

    fn get_timestamp(&self) -> u64 {
        // 模拟高精度时间戳获取（微秒）
        1640995200000000
    }

    fn get_cpu_cycles(&self) -> u64 {
        // 模拟CPU周期计数获取
        1000000
    }

    fn get_memory_usage(&self) -> u32 {
        // 模拟内存使用量获取
        512 * 1024 // 512KB
    }

    fn calculate_fragmentation(&self) -> f32 {
        // 模拟内存碎片率计算
        15.5 // 15.5%
    }
}

#[derive(Debug)]
pub struct PerformanceReport {
    pub total_profiles: usize,
    pub total_duration_us: u64,
    pub avg_duration_us: u64,
    pub hotspots: Vec<Hotspot, 8>,
    pub memory_analysis: MemoryAnalysis,
}

#[derive(Debug, Clone)]
pub struct Hotspot {
    pub function_name: String<32>,
    pub total_time_us: u64,
    pub call_count: u32,
    pub avg_time_us: u64,
    pub percentage: f32,
}

#[derive(Debug, Default)]
pub struct MemoryAnalysis {
    pub peak_usage: u32,
    pub avg_usage: u32,
    pub current_usage: u32,
    pub fragmentation: f32,
}

// 性能分析宏
#[macro_export]
macro_rules! profile_function {
    ($profiler:expr, $func_name:expr, $code:block) => {
        {
            let start_time = $profiler.get_timestamp();
            let result = $code;
            let end_time = $profiler.get_timestamp();
            $profiler.record_function_call($func_name, end_time - start_time);
            result
        }
    };
}
```

## 故障诊断

### 诊断系统

```rust
// 故障诊断系统
use heapless::{Vec, String};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DiagnosticResult {
    pub test_name: String<32>,
    pub status: DiagnosticStatus,
    pub error_code: Option<u32>,
    pub description: String<128>,
    pub timestamp: u64,
    pub severity: Severity,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub enum DiagnosticStatus {
    Pass,
    Fail,
    Warning,
    NotTested,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub enum Severity {
    Critical,
    High,
    Medium,
    Low,
    Info,
}

pub struct DiagnosticEngine {
    tests: Vec<DiagnosticTest, 32>,
    results: Vec<DiagnosticResult, 64>,
    error_patterns: Vec<ErrorPattern, 16>,
}

#[derive(Debug, Clone)]
pub struct DiagnosticTest {
    pub name: String<32>,
    pub test_fn: fn() -> DiagnosticResult,
    pub interval_ms: u32,
    pub enabled: bool,
    pub dependencies: Vec<String<32>, 4>,
}

#[derive(Debug, Clone)]
pub struct ErrorPattern {
    pub pattern: String<64>,
    pub error_code: u32,
    pub description: String<128>,
    pub suggested_action: String<128>,
}

impl DiagnosticEngine {
    pub fn new() -> Self {
        Self {
            tests: Vec::new(),
            results: Vec::new(),
            error_patterns: Vec::new(),
        }
    }

    pub fn add_test(&mut self, test: DiagnosticTest) -> Result<(), ()> {
        self.tests.push(test).map_err(|_| ())
    }

    pub fn add_error_pattern(&mut self, pattern: ErrorPattern) -> Result<(), ()> {
        self.error_patterns.push(pattern).map_err(|_| ())
    }

    pub fn run_diagnostics(&mut self) -> DiagnosticReport {
        let mut report = DiagnosticReport {
            timestamp: self.get_timestamp(),
            total_tests: self.tests.len(),
            passed: 0,
            failed: 0,
            warnings: 0,
            critical_issues: Vec::new(),
            recommendations: Vec::new(),
        };

        // 运行所有启用的测试
        for test in &self.tests {
            if test.enabled {
                let result = (test.test_fn)();
                
                match result.status {
                    DiagnosticStatus::Pass => report.passed += 1,
                    DiagnosticStatus::Fail => {
                        report.failed += 1;
                        if matches!(result.severity, Severity::Critical | Severity::High) {
                            report.critical_issues.push(result.clone()).ok();
                        }
                    }
                    DiagnosticStatus::Warning => report.warnings += 1,
                    DiagnosticStatus::NotTested => {}
                }

                // 存储结果
                if self.results.len() >= self.results.capacity() {
                    self.results.remove(0);
                }
                self.results.push(result).ok();
            }
        }

        // 生成建议
        self.generate_recommendations(&mut report);

        report
    }

    pub fn analyze_error(&self, error_code: u32, context: &str) -> Option<ErrorAnalysis> {
        // 查找匹配的错误模式
        for pattern in &self.error_patterns {
            if pattern.error_code == error_code || context.contains(pattern.pattern.as_str()) {
                return Some(ErrorAnalysis {
                    error_code,
                    pattern: pattern.pattern.clone(),
                    description: pattern.description.clone(),
                    suggested_action: pattern.suggested_action.clone(),
                    confidence: 0.9,
                });
            }
        }

        None
    }

    pub fn get_system_health(&self) -> SystemHealth {
        let recent_results: Vec<&DiagnosticResult, 32> = self.results
            .iter()
            .rev()
            .take(32)
            .collect();

        if recent_results.is_empty() {
            return SystemHealth::Unknown;
        }

        let mut critical_count = 0;
        let mut high_count = 0;
        let mut fail_count = 0;

        for result in &recent_results {
            match result.status {
                DiagnosticStatus::Fail => {
                    fail_count += 1;
                    match result.severity {
                        Severity::Critical => critical_count += 1,
                        Severity::High => high_count += 1,
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        if critical_count > 0 {
            SystemHealth::Critical
        } else if high_count > 2 {
            SystemHealth::Degraded
        } else if fail_count > recent_results.len() / 4 {
            SystemHealth::Warning
        } else {
            SystemHealth::Healthy
        }
    }

    fn generate_recommendations(&self, report: &mut DiagnosticReport) {
        // 基于测试结果生成建议
        if report.failed > 0 {
            let recommendation = String::from("检查失败的测试项目并采取相应措施");
            report.recommendations.push(recommendation).ok();
        }

        if report.warnings > report.total_tests / 2 {
            let recommendation = String::from("系统存在多个警告，建议进行预防性维护");
            report.recommendations.push(recommendation).ok();
        }

        // 基于历史数据生成建议
        let health = self.get_system_health();
        match health {
            SystemHealth::Critical => {
                let recommendation = String::from("系统处于关键状态，需要立即干预");
                report.recommendations.push(recommendation).ok();
            }
            SystemHealth::Degraded => {
                let recommendation = String::from("系统性能下降，建议安排维护窗口");
                report.recommendations.push(recommendation).ok();
            }
            _ => {}
        }
    }

    fn get_timestamp(&self) -> u64 {
        // 模拟时间戳获取
        1640995200
    }
}

#[derive(Debug)]
pub struct DiagnosticReport {
    pub timestamp: u64,
    pub total_tests: usize,
    pub passed: usize,
    pub failed: usize,
    pub warnings: usize,
    pub critical_issues: Vec<DiagnosticResult, 8>,
    pub recommendations: Vec<String<128>, 8>,
}

#[derive(Debug)]
pub struct ErrorAnalysis {
    pub error_code: u32,
    pub pattern: String<64>,
    pub description: String<128>,
    pub suggested_action: String<128>,
    pub confidence: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum SystemHealth {
    Healthy,
    Warning,
    Degraded,
    Critical,
    Unknown,
}

// 具体诊断测试函数
pub fn test_hardware_connectivity() -> DiagnosticResult {
    // 硬件连接测试
    DiagnosticResult {
        test_name: String::from("Hardware Connectivity"),
        status: DiagnosticStatus::Pass,
        error_code: None,
        description: String::from("All hardware interfaces are responding"),
        timestamp: 1640995200,
        severity: Severity::High,
    }
}

pub fn test_memory_integrity() -> DiagnosticResult {
    // 内存完整性测试
    DiagnosticResult {
        test_name: String::from("Memory Integrity"),
        status: DiagnosticStatus::Pass,
        error_code: None,
        description: String::from("Memory test passed"),
        timestamp: 1640995200,
        severity: Severity::Critical,
    }
}

pub fn test_network_connectivity() -> DiagnosticResult {
    // 网络连接测试
    DiagnosticResult {
        test_name: String::from("Network Connectivity"),
        status: DiagnosticStatus::Warning,
        error_code: Some(0x2001),
        description: String::from("Network latency is higher than normal"),
        timestamp: 1640995200,
        severity: Severity::Medium,
    }
}

pub fn test_sensor_calibration() -> DiagnosticResult {
    // 传感器校准测试
    DiagnosticResult {
        test_name: String::from("Sensor Calibration"),
        status: DiagnosticStatus::Pass,
        error_code: None,
        description: String::from("All sensors are within calibration range"),
        timestamp: 1640995200,
        severity: Severity::High,
    }
}
```

## 预防性维护

### 维护调度系统

```rust
// 预防性维护系统
use heapless::{Vec, String};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct MaintenanceTask {
    pub id: u32,
    pub name: String<64>,
    pub description: String<128>,
    pub task_type: MaintenanceType,
    pub schedule: MaintenanceSchedule,
    pub priority: Priority,
    pub estimated_duration: u32, // 分钟
    pub last_executed: Option<u64>,
    pub next_due: u64,
    pub enabled: bool,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub enum MaintenanceType {
    Calibration,
    Cleaning,
    Inspection,
    Update,
    Backup,
    Performance,
    Security,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum MaintenanceSchedule {
    Interval(u64), // 秒
    Daily(u8),     // 小时
    Weekly(u8),    // 星期几 (0-6)
    Monthly(u8),   // 日期 (1-31)
    OnCondition(ConditionTrigger),
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ConditionTrigger {
    pub metric: String<32>,
    pub threshold: f32,
    pub operator: ComparisonOperator,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub enum ComparisonOperator {
    GreaterThan,
    LessThan,
    Equal,
    NotEqual,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub enum Priority {
    Critical,
    High,
    Medium,
    Low,
}

pub struct MaintenanceScheduler {
    tasks: Vec<MaintenanceTask, 32>,
    execution_history: Vec<MaintenanceExecution, 64>,
    current_time: u64,
}

#[derive(Debug, Clone)]
pub struct MaintenanceExecution {
    pub task_id: u32,
    pub start_time: u64,
    pub end_time: u64,
    pub status: ExecutionStatus,
    pub notes: String<128>,
}

#[derive(Debug, Clone, Copy)]
pub enum ExecutionStatus {
    Completed,
    Failed,
    Skipped,
    InProgress,
}

impl MaintenanceScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            execution_history: Vec::new(),
            current_time: 0,
        }
    }

    pub fn add_task(&mut self, mut task: MaintenanceTask) -> Result<(), ()> {
        // 计算下次执行时间
        task.next_due = self.calculate_next_due(&task.schedule);
        self.tasks.push(task).map_err(|_| ())
    }

    pub fn update_current_time(&mut self, timestamp: u64) {
        self.current_time = timestamp;
    }

    pub fn get_due_tasks(&self) -> Vec<&MaintenanceTask, 16> {
        self.tasks
            .iter()
            .filter(|task| task.enabled && task.next_due <= self.current_time)
            .collect()
    }

    pub fn execute_task(&mut self, task_id: u32) -> Result<MaintenanceExecution, MaintenanceError> {
        // 查找任务
        let task = self.tasks
            .iter_mut()
            .find(|t| t.id == task_id)
            .ok_or(MaintenanceError::TaskNotFound)?;

        let start_time = self.current_time;
        
        // 执行任务
        let status = match task.task_type {
            MaintenanceType::Calibration => self.execute_calibration_task(task),
            MaintenanceType::Cleaning => self.execute_cleaning_task(task),
            MaintenanceType::Inspection => self.execute_inspection_task(task),
            MaintenanceType::Update => self.execute_update_task(task),
            MaintenanceType::Backup => self.execute_backup_task(task),
            MaintenanceType::Performance => self.execute_performance_task(task),
            MaintenanceType::Security => self.execute_security_task(task),
        };

        let end_time = self.current_time + (task.estimated_duration as u64 * 60);
        
        // 更新任务状态
        task.last_executed = Some(start_time);
        task.next_due = self.calculate_next_due(&task.schedule);

        // 记录执行历史
        let execution = MaintenanceExecution {
            task_id,
            start_time,
            end_time,
            status,
            notes: String::from("Task executed successfully"),
        };

        if self.execution_history.len() >= self.execution_history.capacity() {
            self.execution_history.remove(0);
        }
        self.execution_history.push(execution.clone()).ok();

        Ok(execution)
    }

    pub fn get_maintenance_report(&self, days: u32) -> MaintenanceReport {
        let cutoff_time = self.current_time - (days as u64 * 24 * 3600);
        let recent_executions: Vec<&MaintenanceExecution, 32> = self.execution_history
            .iter()
            .filter(|e| e.start_time >= cutoff_time)
            .collect();

        let mut completed = 0;
        let mut failed = 0;
        let mut skipped = 0;
        let mut total_duration = 0u64;

        for execution in &recent_executions {
            match execution.status {
                ExecutionStatus::Completed => completed += 1,
                ExecutionStatus::Failed => failed += 1,
                ExecutionStatus::Skipped => skipped += 1,
                ExecutionStatus::InProgress => {}
            }
            total_duration += execution.end_time - execution.start_time;
        }

        let overdue_tasks = self.tasks
            .iter()
            .filter(|task| task.enabled && task.next_due < self.current_time)
            .count();

        MaintenanceReport {
            period_days: days,
            total_executions: recent_executions.len(),
            completed,
            failed,
            skipped,
            total_duration_minutes: total_duration / 60,
            overdue_tasks,
            success_rate: if recent_executions.is_empty() { 
                0.0 
            } else { 
                (completed as f32 / recent_executions.len() as f32) * 100.0 
            },
        }
    }

    fn calculate_next_due(&self, schedule: &MaintenanceSchedule) -> u64 {
        match schedule {
            MaintenanceSchedule::Interval(seconds) => self.current_time + seconds,
            MaintenanceSchedule::Daily(hour) => {
                // 计算下一个指定小时的时间
                let seconds_in_day = 24 * 3600;
                let current_day_start = (self.current_time / seconds_in_day) * seconds_in_day;
                let target_time = current_day_start + (*hour as u64 * 3600);
                
                if target_time > self.current_time {
                    target_time
                } else {
                    target_time + seconds_in_day
                }
            }
            MaintenanceSchedule::Weekly(day) => {
                // 计算下一个指定星期几的时间
                let seconds_in_week = 7 * 24 * 3600;
                self.current_time + seconds_in_week // 简化实现
            }
            MaintenanceSchedule::Monthly(day) => {
                // 计算下一个指定日期的时间
                let seconds_in_month = 30 * 24 * 3600; // 简化为30天
                self.current_time + seconds_in_month
            }
            MaintenanceSchedule::OnCondition(_) => {
                // 条件触发的任务暂时设置为1小时后检查
                self.current_time + 3600
            }
        }
    }

    // 具体任务执行函数
    fn execute_calibration_task(&self, task: &MaintenanceTask) -> ExecutionStatus {
        // 执行校准任务
        ExecutionStatus::Completed
    }

    fn execute_cleaning_task(&self, task: &MaintenanceTask) -> ExecutionStatus {
        // 执行清理任务
        ExecutionStatus::Completed
    }

    fn execute_inspection_task(&self, task: &MaintenanceTask) -> ExecutionStatus {
        // 执行检查任务
        ExecutionStatus::Completed
    }

    fn execute_update_task(&self, task: &MaintenanceTask) -> ExecutionStatus {
        // 执行更新任务
        ExecutionStatus::Completed
    }

    fn execute_backup_task(&self, task: &MaintenanceTask) -> ExecutionStatus {
        // 执行备份任务
        ExecutionStatus::Completed
    }

    fn execute_performance_task(&self, task: &MaintenanceTask) -> ExecutionStatus {
        // 执行性能优化任务
        ExecutionStatus::Completed
    }

    fn execute_security_task(&self, task: &MaintenanceTask) -> ExecutionStatus {
        // 执行安全检查任务
        ExecutionStatus::Completed
    }
}

#[derive(Debug)]
pub struct MaintenanceReport {
    pub period_days: u32,
    pub total_executions: usize,
    pub completed: usize,
    pub failed: usize,
    pub skipped: usize,
    pub total_duration_minutes: u64,
    pub overdue_tasks: usize,
    pub success_rate: f32,
}

#[derive(Debug)]
pub enum MaintenanceError {
    TaskNotFound,
    ExecutionFailed,
    ScheduleError,
}
```

## 总结

监控与维护是嵌入式系统运维的核心，包括：

1. **系统监控**：实时收集和分析系统指标，及时发现问题
2. **性能分析**：深入分析系统性能，识别瓶颈和优化机会
3. **故障诊断**：快速定位和分析系统故障，提供解决方案
4. **预防性维护**：主动维护系统，预防故障发生

通过完善的监控维护体系，可以确保嵌入式系统长期稳定可靠地运行。