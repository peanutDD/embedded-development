# 定时分析技术文档

## 概述

定时分析是嵌入式实时系统设计中的关键技术，用于确保系统能够满足时间约束要求。本文档详细介绍了STM32定时器系统的时序分析方法、工具和最佳实践，帮助开发者构建可靠的实时系统。

## 时序分析基础

### 时间概念定义

1. **执行时间（Execution Time）**
   - **最好情况执行时间（BCET）**：任务在最理想条件下的执行时间
   - **最坏情况执行时间（WCET）**：任务在最不利条件下的执行时间
   - **平均执行时间（ACET）**：任务的统计平均执行时间

2. **响应时间（Response Time）**
   - 从任务激活到任务完成的总时间
   - 包括等待时间、抢占时间和执行时间

3. **截止时间（Deadline）**
   - 任务必须完成的时间点
   - 分为硬截止时间和软截止时间

4. **周期（Period）**
   - 周期性任务两次连续激活之间的时间间隔

### 时序约束类型

```rust
// 时序约束定义
#[derive(Debug, Clone)]
pub struct TimingConstraint {
    pub task_id: u8,
    pub constraint_type: ConstraintType,
    pub value_us: u32,
    pub tolerance_us: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ConstraintType {
    MaxExecutionTime,    // 最大执行时间
    MaxResponseTime,     // 最大响应时间
    MinPeriod,          // 最小周期
    MaxJitter,          // 最大抖动
    MaxLatency,         // 最大延迟
}

impl TimingConstraint {
    pub fn new(task_id: u8, constraint_type: ConstraintType, value_us: u32) -> Self {
        Self {
            task_id,
            constraint_type,
            value_us,
            tolerance_us: 0,
        }
    }
    
    pub fn with_tolerance(mut self, tolerance_us: u32) -> Self {
        self.tolerance_us = tolerance_us;
        self
    }
    
    pub fn is_violated(&self, measured_value: u32) -> bool {
        measured_value > (self.value_us + self.tolerance_us)
    }
}
```

## 定时器精度分析

### 时钟源精度

```rust
// 时钟源配置和精度分析
pub struct ClockSourceAnalysis {
    pub source_type: ClockSourceType,
    pub nominal_frequency: u32,
    pub actual_frequency: u32,
    pub frequency_tolerance: f32,
    pub temperature_coefficient: f32,
    pub aging_rate: f32,
}

#[derive(Debug, Clone)]
pub enum ClockSourceType {
    HSI,        // 内部高速振荡器
    HSE,        // 外部高速振荡器
    LSI,        // 内部低速振荡器
    LSE,        // 外部低速振荡器
    PLL,        // 锁相环
}

impl ClockSourceAnalysis {
    pub fn calculate_frequency_error(&self) -> f32 {
        let error = (self.actual_frequency as f32 - self.nominal_frequency as f32) 
                   / self.nominal_frequency as f32;
        error * 100.0 // 转换为百分比
    }
    
    pub fn calculate_timing_error(&self, target_period_us: u32) -> f32 {
        let frequency_error = self.calculate_frequency_error();
        target_period_us as f32 * frequency_error / 100.0
    }
    
    pub fn predict_long_term_drift(&self, time_hours: f32) -> f32 {
        // 考虑温度和老化影响的长期漂移
        let temperature_drift = self.temperature_coefficient * time_hours;
        let aging_drift = self.aging_rate * time_hours;
        temperature_drift + aging_drift
    }
}

// 时钟精度测量
pub fn measure_clock_accuracy() -> ClockSourceAnalysis {
    // 使用外部参考时钟测量内部时钟精度
    let reference_period = 1000; // 1ms参考周期
    let measured_cycles = measure_timer_cycles_for_reference_period(reference_period);
    let expected_cycles = calculate_expected_cycles(reference_period);
    
    let actual_frequency = (expected_cycles as f32 / measured_cycles as f32) 
                          * get_timer_input_frequency() as f32;
    
    ClockSourceAnalysis {
        source_type: ClockSourceType::HSI,
        nominal_frequency: get_timer_input_frequency(),
        actual_frequency: actual_frequency as u32,
        frequency_tolerance: 1.0, // 1% HSI典型精度
        temperature_coefficient: 0.02, // 温度系数
        aging_rate: 0.001, // 老化率
    }
}
```

### 定时器分辨率分析

```rust
// 定时器分辨率分析
pub struct TimerResolutionAnalysis {
    pub timer_frequency: u32,
    pub prescaler: u32,
    pub counter_width: u8,
    pub resolution_ns: u32,
    pub max_period_ms: u32,
}

impl TimerResolutionAnalysis {
    pub fn new(timer_frequency: u32, prescaler: u32, counter_width: u8) -> Self {
        let effective_frequency = timer_frequency / (prescaler + 1);
        let resolution_ns = 1_000_000_000 / effective_frequency;
        let max_count = (1u64 << counter_width) - 1;
        let max_period_ms = (max_count * resolution_ns as u64) / 1_000_000;
        
        Self {
            timer_frequency,
            prescaler,
            counter_width,
            resolution_ns,
            max_period_ms: max_period_ms as u32,
        }
    }
    
    pub fn calculate_optimal_prescaler(&self, target_resolution_us: u32) -> u32 {
        let target_resolution_ns = target_resolution_us * 1000;
        let optimal_prescaler = (self.timer_frequency * target_resolution_ns) 
                               / 1_000_000_000 - 1;
        optimal_prescaler.max(0)
    }
    
    pub fn quantization_error(&self, target_period_us: u32) -> f32 {
        let target_period_ns = target_period_us * 1000;
        let timer_ticks = target_period_ns / self.resolution_ns;
        let actual_period_ns = timer_ticks * self.resolution_ns;
        let error_ns = (actual_period_ns as i32 - target_period_ns as i32).abs();
        
        error_ns as f32 / target_period_ns as f32 * 100.0
    }
}

// 分辨率优化建议
pub fn optimize_timer_resolution(
    target_periods: &[u32], 
    timer_freq: u32
) -> Vec<TimerConfiguration> {
    let mut configurations = Vec::new();
    
    for &period_us in target_periods {
        let mut best_config = None;
        let mut min_error = f32::MAX;
        
        // 尝试不同的预分频值
        for prescaler in 0..=65535 {
            let analysis = TimerResolutionAnalysis::new(timer_freq, prescaler, 16);
            let error = analysis.quantization_error(period_us);
            
            if error < min_error && analysis.max_period_ms * 1000 >= period_us {
                min_error = error;
                best_config = Some(TimerConfiguration {
                    prescaler,
                    auto_reload: (period_us * 1000) / analysis.resolution_ns,
                    resolution_ns: analysis.resolution_ns,
                    quantization_error: error,
                });
            }
        }
        
        if let Some(config) = best_config {
            configurations.push(config);
        }
    }
    
    configurations
}

#[derive(Debug, Clone)]
pub struct TimerConfiguration {
    pub prescaler: u32,
    pub auto_reload: u32,
    pub resolution_ns: u32,
    pub quantization_error: f32,
}
```

## 中断延迟分析

### 中断响应时间模型

```rust
// 中断延迟分析
pub struct InterruptLatencyAnalysis {
    pub interrupt_priority: u8,
    pub cpu_frequency: u32,
    pub pipeline_depth: u8,
    pub cache_enabled: bool,
    pub fpu_enabled: bool,
}

impl InterruptLatencyAnalysis {
    pub fn calculate_best_case_latency(&self) -> u32 {
        // 最好情况：无抢占，无缓存缺失
        let pipeline_flush_cycles = self.pipeline_depth as u32;
        let context_save_cycles = if self.fpu_enabled { 34 } else { 12 };
        let vector_fetch_cycles = 2;
        
        let total_cycles = pipeline_flush_cycles + context_save_cycles + vector_fetch_cycles;
        self.cycles_to_nanoseconds(total_cycles)
    }
    
    pub fn calculate_worst_case_latency(&self) -> u32 {
        // 最坏情况：被低优先级中断抢占，缓存缺失
        let best_case = self.calculate_best_case_latency();
        let cache_miss_penalty = if self.cache_enabled { 
            self.cycles_to_nanoseconds(50) // 典型缓存缺失惩罚
        } else { 
            0 
        };
        let preemption_delay = self.calculate_max_preemption_delay();
        
        best_case + cache_miss_penalty + preemption_delay
    }
    
    fn calculate_max_preemption_delay(&self) -> u32 {
        // 计算可能的最大抢占延迟
        // 这需要分析所有更高优先级的中断
        let max_higher_priority_isr_cycles = 1000; // 假设值
        self.cycles_to_nanoseconds(max_higher_priority_isr_cycles)
    }
    
    fn cycles_to_nanoseconds(&self, cycles: u32) -> u32 {
        (cycles * 1_000_000_000) / self.cpu_frequency
    }
    
    pub fn analyze_interrupt_jitter(&self, measurements: &[u32]) -> JitterAnalysis {
        if measurements.is_empty() {
            return JitterAnalysis::default();
        }
        
        let min_latency = *measurements.iter().min().unwrap();
        let max_latency = *measurements.iter().max().unwrap();
        let avg_latency = measurements.iter().sum::<u32>() / measurements.len() as u32;
        
        let variance: f32 = measurements.iter()
            .map(|&x| (x as f32 - avg_latency as f32).powi(2))
            .sum::<f32>() / measurements.len() as f32;
        let std_deviation = variance.sqrt();
        
        JitterAnalysis {
            min_latency,
            max_latency,
            avg_latency,
            jitter: max_latency - min_latency,
            std_deviation,
            measurements_count: measurements.len(),
        }
    }
}

#[derive(Debug, Default)]
pub struct JitterAnalysis {
    pub min_latency: u32,
    pub max_latency: u32,
    pub avg_latency: u32,
    pub jitter: u32,
    pub std_deviation: f32,
    pub measurements_count: usize,
}
```

### 中断优先级分析

```rust
// 中断优先级分析工具
pub struct InterruptPriorityAnalyzer {
    pub interrupts: Vec<InterruptDescriptor>,
    pub priority_groups: u8,
}

#[derive(Debug, Clone)]
pub struct InterruptDescriptor {
    pub id: u8,
    pub name: &'static str,
    pub priority: u8,
    pub sub_priority: u8,
    pub max_execution_time_us: u32,
    pub frequency_hz: f32,
    pub deadline_us: u32,
}

impl InterruptPriorityAnalyzer {
    pub fn new(priority_groups: u8) -> Self {
        Self {
            interrupts: Vec::new(),
            priority_groups,
        }
    }
    
    pub fn add_interrupt(&mut self, interrupt: InterruptDescriptor) {
        self.interrupts.push(interrupt);
        self.interrupts.sort_by_key(|i| i.priority);
    }
    
    pub fn analyze_priority_assignment(&self) -> PriorityAnalysisResult {
        let mut violations = Vec::new();
        let mut recommendations = Vec::new();
        
        // 检查优先级分配的合理性
        for interrupt in &self.interrupts {
            // 检查是否满足截止时间
            let response_time = self.calculate_response_time(interrupt);
            if response_time > interrupt.deadline_us {
                violations.push(PriorityViolation {
                    interrupt_id: interrupt.id,
                    violation_type: ViolationType::DeadlineMiss,
                    expected: interrupt.deadline_us,
                    actual: response_time,
                });
            }
            
            // 检查优先级倒置
            if let Some(inversion) = self.detect_priority_inversion(interrupt) {
                violations.push(inversion);
            }
        }
        
        // 生成优化建议
        if !violations.is_empty() {
            recommendations.extend(self.generate_priority_recommendations());
        }
        
        PriorityAnalysisResult {
            violations,
            recommendations,
            total_cpu_utilization: self.calculate_total_utilization(),
        }
    }
    
    fn calculate_response_time(&self, target: &InterruptDescriptor) -> u32 {
        let mut response_time = target.max_execution_time_us;
        
        // 计算高优先级中断的干扰
        for interrupt in &self.interrupts {
            if interrupt.priority < target.priority { // 更高优先级（数值更小）
                let interference_count = (response_time as f32 * interrupt.frequency_hz / 1_000_000.0).ceil() as u32;
                response_time += interference_count * interrupt.max_execution_time_us;
            }
        }
        
        response_time
    }
    
    fn detect_priority_inversion(&self, interrupt: &InterruptDescriptor) -> Option<PriorityViolation> {
        // 检查是否存在优先级倒置
        for other in &self.interrupts {
            if other.id != interrupt.id 
               && other.priority > interrupt.priority 
               && other.deadline_us < interrupt.deadline_us {
                return Some(PriorityViolation {
                    interrupt_id: interrupt.id,
                    violation_type: ViolationType::PriorityInversion,
                    expected: interrupt.deadline_us,
                    actual: other.deadline_us,
                });
            }
        }
        None
    }
    
    fn generate_priority_recommendations(&self) -> Vec<PriorityRecommendation> {
        let mut recommendations = Vec::new();
        
        // 基于截止时间单调分配优先级
        let mut sorted_by_deadline = self.interrupts.clone();
        sorted_by_deadline.sort_by_key(|i| i.deadline_us);
        
        for (index, interrupt) in sorted_by_deadline.iter().enumerate() {
            let recommended_priority = index as u8;
            if recommended_priority != interrupt.priority {
                recommendations.push(PriorityRecommendation {
                    interrupt_id: interrupt.id,
                    current_priority: interrupt.priority,
                    recommended_priority,
                    reason: "Deadline Monotonic Assignment".to_string(),
                });
            }
        }
        
        recommendations
    }
    
    fn calculate_total_utilization(&self) -> f32 {
        self.interrupts.iter()
            .map(|i| (i.max_execution_time_us as f32 / 1_000_000.0) * i.frequency_hz)
            .sum::<f32>() * 100.0
    }
}

#[derive(Debug)]
pub struct PriorityAnalysisResult {
    pub violations: Vec<PriorityViolation>,
    pub recommendations: Vec<PriorityRecommendation>,
    pub total_cpu_utilization: f32,
}

#[derive(Debug)]
pub struct PriorityViolation {
    pub interrupt_id: u8,
    pub violation_type: ViolationType,
    pub expected: u32,
    pub actual: u32,
}

#[derive(Debug)]
pub enum ViolationType {
    DeadlineMiss,
    PriorityInversion,
    UtilizationOverflow,
}

#[derive(Debug)]
pub struct PriorityRecommendation {
    pub interrupt_id: u8,
    pub current_priority: u8,
    pub recommended_priority: u8,
    pub reason: String,
}
```

## 任务调度分析

### 可调度性分析

```rust
// 可调度性分析工具
pub struct SchedulabilityAnalyzer {
    pub tasks: Vec<TaskDescriptor>,
    pub scheduling_policy: SchedulingPolicy,
}

#[derive(Debug, Clone)]
pub struct TaskDescriptor {
    pub id: u8,
    pub name: &'static str,
    pub priority: u8,
    pub period_us: u32,
    pub deadline_us: u32,
    pub wcet_us: u32,
    pub bcet_us: u32,
    pub jitter_us: u32,
}

#[derive(Debug, Clone)]
pub enum SchedulingPolicy {
    FixedPriority,
    RateMonotonic,
    DeadlineMonotonic,
    EarliestDeadlineFirst,
}

impl SchedulabilityAnalyzer {
    pub fn new(scheduling_policy: SchedulingPolicy) -> Self {
        Self {
            tasks: Vec::new(),
            scheduling_policy,
        }
    }
    
    pub fn add_task(&mut self, task: TaskDescriptor) {
        self.tasks.push(task);
        self.sort_tasks();
    }
    
    fn sort_tasks(&mut self) {
        match self.scheduling_policy {
            SchedulingPolicy::RateMonotonic => {
                self.tasks.sort_by_key(|t| t.period_us);
            },
            SchedulingPolicy::DeadlineMonotonic => {
                self.tasks.sort_by_key(|t| t.deadline_us);
            },
            SchedulingPolicy::FixedPriority => {
                self.tasks.sort_by_key(|t| t.priority);
            },
            SchedulingPolicy::EarliestDeadlineFirst => {
                // EDF是动态调度，不需要静态排序
            }
        }
    }
    
    pub fn analyze_schedulability(&self) -> SchedulabilityResult {
        match self.scheduling_policy {
            SchedulingPolicy::RateMonotonic => self.rate_monotonic_analysis(),
            SchedulingPolicy::DeadlineMonotonic => self.deadline_monotonic_analysis(),
            SchedulingPolicy::EarliestDeadlineFirst => self.edf_analysis(),
            SchedulingPolicy::FixedPriority => self.response_time_analysis(),
        }
    }
    
    fn rate_monotonic_analysis(&self) -> SchedulabilityResult {
        let n = self.tasks.len() as f32;
        let utilization_bound = n * (2.0_f32.powf(1.0 / n) - 1.0);
        
        let total_utilization = self.calculate_total_utilization();
        
        let is_schedulable = total_utilization <= utilization_bound;
        
        SchedulabilityResult {
            is_schedulable,
            analysis_method: "Rate Monotonic Analysis".to_string(),
            total_utilization,
            utilization_bound: Some(utilization_bound),
            task_results: self.tasks.iter().map(|task| {
                TaskSchedulabilityResult {
                    task_id: task.id,
                    is_schedulable,
                    response_time_us: self.calculate_response_time_rms(task),
                    utilization: (task.wcet_us as f32 / task.period_us as f32) * 100.0,
                }
            }).collect(),
        }
    }
    
    fn edf_analysis(&self) -> SchedulabilityResult {
        let total_utilization = self.calculate_total_utilization();
        let is_schedulable = total_utilization <= 100.0;
        
        SchedulabilityResult {
            is_schedulable,
            analysis_method: "Earliest Deadline First Analysis".to_string(),
            total_utilization,
            utilization_bound: Some(100.0),
            task_results: self.tasks.iter().map(|task| {
                TaskSchedulabilityResult {
                    task_id: task.id,
                    is_schedulable,
                    response_time_us: task.wcet_us, // EDF下的最好情况
                    utilization: (task.wcet_us as f32 / task.period_us as f32) * 100.0,
                }
            }).collect(),
        }
    }
    
    fn response_time_analysis(&self) -> SchedulabilityResult {
        let mut all_schedulable = true;
        let mut task_results = Vec::new();
        
        for task in &self.tasks {
            let response_time = self.calculate_exact_response_time(task);
            let is_task_schedulable = response_time <= task.deadline_us;
            all_schedulable &= is_task_schedulable;
            
            task_results.push(TaskSchedulabilityResult {
                task_id: task.id,
                is_schedulable: is_task_schedulable,
                response_time_us: response_time,
                utilization: (task.wcet_us as f32 / task.period_us as f32) * 100.0,
            });
        }
        
        SchedulabilityResult {
            is_schedulable: all_schedulable,
            analysis_method: "Exact Response Time Analysis".to_string(),
            total_utilization: self.calculate_total_utilization(),
            utilization_bound: None,
            task_results,
        }
    }
    
    fn deadline_monotonic_analysis(&self) -> SchedulabilityResult {
        // 类似于RMS，但基于截止时间而非周期
        self.response_time_analysis()
    }
    
    fn calculate_total_utilization(&self) -> f32 {
        self.tasks.iter()
            .map(|task| (task.wcet_us as f32 / task.period_us as f32) * 100.0)
            .sum()
    }
    
    fn calculate_response_time_rms(&self, target_task: &TaskDescriptor) -> u32 {
        let mut response_time = target_task.wcet_us;
        
        // 计算高优先级任务的干扰（周期更短的任务）
        for task in &self.tasks {
            if task.period_us < target_task.period_us {
                let interference_count = (response_time + task.period_us - 1) / task.period_us;
                response_time += interference_count * task.wcet_us;
            }
        }
        
        response_time
    }
    
    fn calculate_exact_response_time(&self, target_task: &TaskDescriptor) -> u32 {
        let mut response_time = target_task.wcet_us;
        let mut prev_response_time;
        
        // 迭代计算精确响应时间
        for _ in 0..100 { // 最多迭代100次防止无限循环
            prev_response_time = response_time;
            
            // 重新计算响应时间
            response_time = target_task.wcet_us;
            
            // 加上高优先级任务的干扰
            for task in &self.tasks {
                if task.priority < target_task.priority { // 更高优先级
                    let interference_count = (prev_response_time + task.period_us - 1) / task.period_us;
                    response_time += interference_count * task.wcet_us;
                }
            }
            
            // 检查收敛
            if response_time == prev_response_time {
                break;
            }
            
            // 检查是否超过截止时间（提前终止）
            if response_time > target_task.deadline_us * 2 {
                response_time = u32::MAX;
                break;
            }
        }
        
        response_time
    }
}

#[derive(Debug)]
pub struct SchedulabilityResult {
    pub is_schedulable: bool,
    pub analysis_method: String,
    pub total_utilization: f32,
    pub utilization_bound: Option<f32>,
    pub task_results: Vec<TaskSchedulabilityResult>,
}

#[derive(Debug)]
pub struct TaskSchedulabilityResult {
    pub task_id: u8,
    pub is_schedulable: bool,
    pub response_time_us: u32,
    pub utilization: f32,
}
```

## 性能测量工具

### 高精度时间戳

```rust
// 高精度时间戳测量
pub struct HighPrecisionTimer {
    timer_base: *mut stm32f4xx_hal::pac::tim2::RegisterBlock,
    frequency: u32,
    overflow_count: core::sync::atomic::AtomicU32,
}

impl HighPrecisionTimer {
    pub fn new() -> Self {
        // 配置TIM2为32位高精度计数器
        let rcc = unsafe { &*stm32f4xx_hal::pac::RCC::ptr() };
        let tim2 = unsafe { &*stm32f4xx_hal::pac::TIM2::ptr() };
        
        // 使能时钟
        rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
        
        // 配置为最高精度（无预分频）
        tim2.psc.write(|w| unsafe { w.bits(0) });
        tim2.arr.write(|w| unsafe { w.bits(0xFFFFFFFF) });
        
        // 使能溢出中断
        tim2.dier.modify(|_, w| w.uie().set_bit());
        
        // 启动定时器
        tim2.cr1.modify(|_, w| w.cen().set_bit());
        
        Self {
            timer_base: tim2,
            frequency: 84_000_000, // 84MHz
            overflow_count: core::sync::atomic::AtomicU32::new(0),
        }
    }
    
    pub fn get_timestamp_ns(&self) -> u64 {
        let timer = unsafe { &*self.timer_base };
        let overflow_count = self.overflow_count.load(core::sync::atomic::Ordering::Relaxed);
        let counter_value = timer.cnt.read().bits();
        
        let total_ticks = (overflow_count as u64) << 32 | counter_value as u64;
        (total_ticks * 1_000_000_000) / self.frequency as u64
    }
    
    pub fn measure_execution_time<F, R>(&self, func: F) -> (R, u32)
    where
        F: FnOnce() -> R,
    {
        let start_time = self.get_timestamp_ns();
        let result = func();
        let end_time = self.get_timestamp_ns();
        
        let execution_time_ns = (end_time - start_time) as u32;
        (result, execution_time_ns)
    }
    
    pub fn handle_overflow_interrupt(&self) {
        self.overflow_count.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        
        // 清除中断标志
        let timer = unsafe { &*self.timer_base };
        timer.sr.modify(|_, w| w.uif().clear_bit());
    }
}

// 全局高精度定时器实例
static mut HIGH_PRECISION_TIMER: Option<HighPrecisionTimer> = None;

pub fn init_high_precision_timer() {
    unsafe {
        HIGH_PRECISION_TIMER = Some(HighPrecisionTimer::new());
    }
}

pub fn get_high_precision_timestamp() -> u64 {
    unsafe {
        HIGH_PRECISION_TIMER.as_ref()
            .map(|timer| timer.get_timestamp_ns())
            .unwrap_or(0)
    }
}

// TIM2溢出中断处理
#[cortex_m_rt::interrupt]
fn TIM2() {
    unsafe {
        if let Some(ref timer) = HIGH_PRECISION_TIMER {
            timer.handle_overflow_interrupt();
        }
    }
}
```

### 统计分析工具

```rust
// 统计分析工具
pub struct TimingStatistics {
    measurements: Vec<u32>,
    max_samples: usize,
    current_index: usize,
    is_full: bool,
}

impl TimingStatistics {
    pub fn new(max_samples: usize) -> Self {
        Self {
            measurements: vec![0; max_samples],
            max_samples,
            current_index: 0,
            is_full: false,
        }
    }
    
    pub fn add_measurement(&mut self, value: u32) {
        self.measurements[self.current_index] = value;
        self.current_index = (self.current_index + 1) % self.max_samples;
        
        if self.current_index == 0 {
            self.is_full = true;
        }
    }
    
    pub fn calculate_statistics(&self) -> StatisticsResult {
        let valid_samples = if self.is_full { 
            self.max_samples 
        } else { 
            self.current_index 
        };
        
        if valid_samples == 0 {
            return StatisticsResult::default();
        }
        
        let samples = &self.measurements[0..valid_samples];
        
        let min = *samples.iter().min().unwrap();
        let max = *samples.iter().max().unwrap();
        let sum: u64 = samples.iter().map(|&x| x as u64).sum();
        let mean = (sum / valid_samples as u64) as u32;
        
        // 计算方差和标准差
        let variance: f64 = samples.iter()
            .map(|&x| (x as f64 - mean as f64).powi(2))
            .sum::<f64>() / valid_samples as f64;
        let std_dev = variance.sqrt() as u32;
        
        // 计算百分位数
        let mut sorted_samples = samples.to_vec();
        sorted_samples.sort();
        
        let p50 = sorted_samples[valid_samples * 50 / 100];
        let p90 = sorted_samples[valid_samples * 90 / 100];
        let p95 = sorted_samples[valid_samples * 95 / 100];
        let p99 = sorted_samples[valid_samples * 99 / 100];
        
        StatisticsResult {
            sample_count: valid_samples,
            min,
            max,
            mean,
            std_dev,
            p50,
            p90,
            p95,
            p99,
            range: max - min,
        }
    }
    
    pub fn detect_outliers(&self, threshold_sigma: f32) -> Vec<u32> {
        let stats = self.calculate_statistics();
        let threshold = stats.mean as f32 + threshold_sigma * stats.std_dev as f32;
        
        let valid_samples = if self.is_full { 
            self.max_samples 
        } else { 
            self.current_index 
        };
        
        self.measurements[0..valid_samples].iter()
            .filter(|&&x| x as f32 > threshold)
            .cloned()
            .collect()
    }
}

#[derive(Debug, Default)]
pub struct StatisticsResult {
    pub sample_count: usize,
    pub min: u32,
    pub max: u32,
    pub mean: u32,
    pub std_dev: u32,
    pub p50: u32,  // 中位数
    pub p90: u32,
    pub p95: u32,
    pub p99: u32,
    pub range: u32,
}

impl StatisticsResult {
    pub fn coefficient_of_variation(&self) -> f32 {
        if self.mean > 0 {
            (self.std_dev as f32 / self.mean as f32) * 100.0
        } else {
            0.0
        }
    }
    
    pub fn is_stable(&self, cv_threshold: f32) -> bool {
        self.coefficient_of_variation() < cv_threshold
    }
}
```

## 实时性能监控

### 运行时监控系统

```rust
// 实时性能监控系统
pub struct RealTimeMonitor {
    task_monitors: heapless::FnvIndexMap<u8, TaskMonitor, 16>,
    system_monitor: SystemMonitor,
    alert_thresholds: AlertThresholds,
    monitoring_enabled: bool,
}

#[derive(Debug)]
struct TaskMonitor {
    task_id: u8,
    execution_times: TimingStatistics,
    response_times: TimingStatistics,
    deadline_misses: u32,
    last_execution_start: u64,
    max_execution_time: u32,
    total_executions: u32,
}

#[derive(Debug)]
struct SystemMonitor {
    cpu_utilization: f32,
    interrupt_load: f32,
    context_switches_per_second: u32,
    memory_usage: u32,
    stack_high_water_mark: u32,
}

#[derive(Debug)]
struct AlertThresholds {
    max_execution_time_us: u32,
    max_response_time_us: u32,
    max_cpu_utilization: f32,
    max_deadline_miss_rate: f32,
}

impl RealTimeMonitor {
    pub fn new() -> Self {
        Self {
            task_monitors: heapless::FnvIndexMap::new(),
            system_monitor: SystemMonitor {
                cpu_utilization: 0.0,
                interrupt_load: 0.0,
                context_switches_per_second: 0,
                memory_usage: 0,
                stack_high_water_mark: 0,
            },
            alert_thresholds: AlertThresholds {
                max_execution_time_us: 10000,
                max_response_time_us: 50000,
                max_cpu_utilization: 80.0,
                max_deadline_miss_rate: 1.0,
            },
            monitoring_enabled: false,
        }
    }
    
    pub fn register_task(&mut self, task_id: u8) -> Result<(), &'static str> {
        if self.task_monitors.len() >= 16 {
            return Err("Too many tasks registered");
        }
        
        let monitor = TaskMonitor {
            task_id,
            execution_times: TimingStatistics::new(100),
            response_times: TimingStatistics::new(100),
            deadline_misses: 0,
            last_execution_start: 0,
            max_execution_time: 0,
            total_executions: 0,
        };
        
        self.task_monitors.insert(task_id, monitor)
            .map_err(|_| "Failed to register task")?;
        
        Ok(())
    }
    
    pub fn task_start(&mut self, task_id: u8) {
        if !self.monitoring_enabled {
            return;
        }
        
        let timestamp = get_high_precision_timestamp();
        
        if let Some(monitor) = self.task_monitors.get_mut(&task_id) {
            monitor.last_execution_start = timestamp;
        }
    }
    
    pub fn task_end(&mut self, task_id: u8) {
        if !self.monitoring_enabled {
            return;
        }
        
        let timestamp = get_high_precision_timestamp();
        
        if let Some(monitor) = self.task_monitors.get_mut(&task_id) {
            if monitor.last_execution_start > 0 {
                let execution_time = ((timestamp - monitor.last_execution_start) / 1000) as u32;
                
                monitor.execution_times.add_measurement(execution_time);
                monitor.max_execution_time = monitor.max_execution_time.max(execution_time);
                monitor.total_executions += 1;
                
                // 检查是否超过阈值
                if execution_time > self.alert_thresholds.max_execution_time_us {
                    self.trigger_alert(AlertType::ExecutionTimeExceeded, task_id, execution_time);
                }
            }
        }
    }
    
    pub fn record_deadline_miss(&mut self, task_id: u8) {
        if let Some(monitor) = self.task_monitors.get_mut(&task_id) {
            monitor.deadline_misses += 1;
            
            let miss_rate = (monitor.deadline_misses as f32 / monitor.total_executions as f32) * 100.0;
            if miss_rate > self.alert_thresholds.max_deadline_miss_rate {
                self.trigger_alert(AlertType::DeadlineMissRateHigh, task_id, miss_rate as u32);
            }
        }
    }
    
    pub fn update_system_metrics(&mut self) {
        // 更新CPU利用率
        self.system_monitor.cpu_utilization = self.calculate_cpu_utilization();
        
        // 更新中断负载
        self.system_monitor.interrupt_load = self.calculate_interrupt_load();
        
        // 检查系统级阈值
        if self.system_monitor.cpu_utilization > self.alert_thresholds.max_cpu_utilization {
            self.trigger_alert(AlertType::CpuUtilizationHigh, 0, 
                              self.system_monitor.cpu_utilization as u32);
        }
    }
    
    fn calculate_cpu_utilization(&self) -> f32 {
        // 基于任务执行时间计算CPU利用率
        let total_utilization: f32 = self.task_monitors.values()
            .map(|monitor| {
                let stats = monitor.execution_times.calculate_statistics();
                if stats.sample_count > 0 {
                    stats.mean as f32 / 1000.0 // 假设1ms周期
                } else {
                    0.0
                }
            })
            .sum();
        
        total_utilization.min(100.0)
    }
    
    fn calculate_interrupt_load(&self) -> f32 {
        // 计算中断负载（需要硬件支持）
        // 这里返回模拟值
        25.0
    }
    
    fn trigger_alert(&self, alert_type: AlertType, task_id: u8, value: u32) {
        // 触发告警（可以通过UART、LED等方式）
        match alert_type {
            AlertType::ExecutionTimeExceeded => {
                // 记录执行时间超限告警
            },
            AlertType::DeadlineMissRateHigh => {
                // 记录截止时间错过率高告警
            },
            AlertType::CpuUtilizationHigh => {
                // 记录CPU利用率高告警
            },
        }
    }
    
    pub fn get_task_report(&self, task_id: u8) -> Option<TaskPerformanceReport> {
        self.task_monitors.get(&task_id).map(|monitor| {
            let exec_stats = monitor.execution_times.calculate_statistics();
            let resp_stats = monitor.response_times.calculate_statistics();
            
            TaskPerformanceReport {
                task_id,
                total_executions: monitor.total_executions,
                deadline_misses: monitor.deadline_misses,
                deadline_miss_rate: if monitor.total_executions > 0 {
                    (monitor.deadline_misses as f32 / monitor.total_executions as f32) * 100.0
                } else {
                    0.0
                },
                execution_time_stats: exec_stats,
                response_time_stats: resp_stats,
                max_execution_time: monitor.max_execution_time,
            }
        })
    }
    
    pub fn enable_monitoring(&mut self) {
        self.monitoring_enabled = true;
    }
    
    pub fn disable_monitoring(&mut self) {
        self.monitoring_enabled = false;
    }
}

#[derive(Debug)]
enum AlertType {
    ExecutionTimeExceeded,
    DeadlineMissRateHigh,
    CpuUtilizationHigh,
}

#[derive(Debug)]
pub struct TaskPerformanceReport {
    pub task_id: u8,
    pub total_executions: u32,
    pub deadline_misses: u32,
    pub deadline_miss_rate: f32,
    pub execution_time_stats: StatisticsResult,
    pub response_time_stats: StatisticsResult,
    pub max_execution_time: u32,
}
```

## 分析报告生成

### 自动化报告生成

```rust
// 分析报告生成器
pub struct TimingAnalysisReporter {
    system_info: SystemInfo,
    analysis_results: Vec<AnalysisResult>,
    recommendations: Vec<Recommendation>,
}

#[derive(Debug)]
struct SystemInfo {
    cpu_frequency: u32,
    timer_frequency: u32,
    memory_size: u32,
    flash_size: u32,
    compiler_version: &'static str,
    optimization_level: &'static str,
}

#[derive(Debug)]
enum AnalysisResult {
    SchedulabilityAnalysis(SchedulabilityResult),
    InterruptLatencyAnalysis(JitterAnalysis),
    TimerResolutionAnalysis(TimerResolutionAnalysis),
    PerformanceMonitoring(Vec<TaskPerformanceReport>),
}

#[derive(Debug)]
struct Recommendation {
    category: RecommendationCategory,
    priority: RecommendationPriority,
    description: String,
    implementation_effort: ImplementationEffort,
}

#[derive(Debug)]
enum RecommendationCategory {
    TimerConfiguration,
    InterruptPriority,
    TaskScheduling,
    SystemOptimization,
}

#[derive(Debug)]
enum RecommendationPriority {
    Critical,
    High,
    Medium,
    Low,
}

#[derive(Debug)]
enum ImplementationEffort {
    Low,
    Medium,
    High,
}

impl TimingAnalysisReporter {
    pub fn new() -> Self {
        Self {
            system_info: SystemInfo {
                cpu_frequency: 168_000_000,
                timer_frequency: 84_000_000,
                memory_size: 192 * 1024,
                flash_size: 1024 * 1024,
                compiler_version: "rustc 1.70.0",
                optimization_level: "release",
            },
            analysis_results: Vec::new(),
            recommendations: Vec::new(),
        }
    }
    
    pub fn add_analysis_result(&mut self, result: AnalysisResult) {
        self.analysis_results.push(result);
        self.generate_recommendations_for_result(&result);
    }
    
    fn generate_recommendations_for_result(&mut self, result: &AnalysisResult) {
        match result {
            AnalysisResult::SchedulabilityAnalysis(sched_result) => {
                if !sched_result.is_schedulable {
                    self.recommendations.push(Recommendation {
                        category: RecommendationCategory::TaskScheduling,
                        priority: RecommendationPriority::Critical,
                        description: "System is not schedulable. Consider reducing task execution times or increasing periods.".to_string(),
                        implementation_effort: ImplementationEffort::High,
                    });
                }
                
                if sched_result.total_utilization > 80.0 {
                    self.recommendations.push(Recommendation {
                        category: RecommendationCategory::SystemOptimization,
                        priority: RecommendationPriority::High,
                        description: "CPU utilization is high. Consider optimizing critical tasks.".to_string(),
                        implementation_effort: ImplementationEffort::Medium,
                    });
                }
            },
            
            AnalysisResult::InterruptLatencyAnalysis(jitter_analysis) => {
                if jitter_analysis.jitter > 10000 { // 10ms
                    self.recommendations.push(Recommendation {
                        category: RecommendationCategory::InterruptPriority,
                        priority: RecommendationPriority::High,
                        description: "High interrupt jitter detected. Review interrupt priorities and ISR execution times.".to_string(),
                        implementation_effort: ImplementationEffort::Medium,
                    });
                }
            },
            
            AnalysisResult::TimerResolutionAnalysis(timer_analysis) => {
                if timer_analysis.resolution_ns > 1000000 { // 1ms
                    self.recommendations.push(Recommendation {
                        category: RecommendationCategory::TimerConfiguration,
                        priority: RecommendationPriority::Medium,
                        description: "Timer resolution is low. Consider reducing prescaler value for better precision.".to_string(),
                        implementation_effort: ImplementationEffort::Low,
                    });
                }
            },
            
            AnalysisResult::PerformanceMonitoring(reports) => {
                for report in reports {
                    if report.deadline_miss_rate > 1.0 {
                        self.recommendations.push(Recommendation {
                            category: RecommendationCategory::TaskScheduling,
                            priority: RecommendationPriority::Critical,
                            description: format!("Task {} has high deadline miss rate: {:.2}%", 
                                               report.task_id, report.deadline_miss_rate),
                            implementation_effort: ImplementationEffort::High,
                        });
                    }
                }
            }
        }
    }
    
    pub fn generate_report(&self) -> String {
        let mut report = String::new();
        
        // 报告头部
        report.push_str("# 定时分析报告\n\n");
        report.push_str(&format!("生成时间: {}\n", "2024-01-01 12:00:00")); // 实际应用中使用RTC
        report.push_str(&format!("分析工具版本: {}\n\n", "1.0.0"));
        
        // 系统信息
        report.push_str("## 系统信息\n\n");
        report.push_str(&format!("- CPU频率: {} MHz\n", self.system_info.cpu_frequency / 1_000_000));
        report.push_str(&format!("- 定时器频率: {} MHz\n", self.system_info.timer_frequency / 1_000_000));
        report.push_str(&format!("- 内存大小: {} KB\n", self.system_info.memory_size / 1024));
        report.push_str(&format!("- Flash大小: {} KB\n", self.system_info.flash_size / 1024));
        report.push_str(&format!("- 编译器: {}\n", self.system_info.compiler_version));
        report.push_str(&format!("- 优化级别: {}\n\n", self.system_info.optimization_level));
        
        // 分析结果
        report.push_str("## 分析结果\n\n");
        for (index, result) in self.analysis_results.iter().enumerate() {
            report.push_str(&format!("### 分析 {}\n\n", index + 1));
            report.push_str(&self.format_analysis_result(result));
            report.push_str("\n");
        }
        
        // 建议
        report.push_str("## 优化建议\n\n");
        let mut critical_recommendations: Vec<_> = self.recommendations.iter()
            .filter(|r| matches!(r.priority, RecommendationPriority::Critical))
            .collect();
        let mut high_recommendations: Vec<_> = self.recommendations.iter()
            .filter(|r| matches!(r.priority, RecommendationPriority::High))
            .collect();
        let mut medium_recommendations: Vec<_> = self.recommendations.iter()
            .filter(|r| matches!(r.priority, RecommendationPriority::Medium))
            .collect();
        
        if !critical_recommendations.is_empty() {
            report.push_str("### 关键建议\n\n");
            for rec in critical_recommendations {
                report.push_str(&format!("- **{}**: {}\n", 
                    self.format_category(&rec.category), rec.description));
            }
            report.push_str("\n");
        }
        
        if !high_recommendations.is_empty() {
            report.push_str("### 高优先级建议\n\n");
            for rec in high_recommendations {
                report.push_str(&format!("- **{}**: {}\n", 
                    self.format_category(&rec.category), rec.description));
            }
            report.push_str("\n");
        }
        
        if !medium_recommendations.is_empty() {
            report.push_str("### 中优先级建议\n\n");
            for rec in medium_recommendations {
                report.push_str(&format!("- **{}**: {}\n", 
                    self.format_category(&rec.category), rec.description));
            }
            report.push_str("\n");
        }
        
        // 总结
        report.push_str("## 总结\n\n");
        let critical_count = self.recommendations.iter()
            .filter(|r| matches!(r.priority, RecommendationPriority::Critical))
            .count();
        
        if critical_count > 0 {
            report.push_str(&format!("⚠️ 发现 {} 个关键问题需要立即处理。\n", critical_count));
        } else {
            report.push_str("✅ 系统时序分析通过，未发现关键问题。\n");
        }
        
        report.push_str("\n---\n");
        report.push_str("*此报告由STM32定时分析工具自动生成*\n");
        
        report
    }
    
    fn format_analysis_result(&self, result: &AnalysisResult) -> String {
        match result {
            AnalysisResult::SchedulabilityAnalysis(sched) => {
                format!("**可调度性分析 ({})**\n\n- 可调度: {}\n- 总利用率: {:.2}%\n- 利用率上限: {:.2}%\n",
                    sched.analysis_method,
                    if sched.is_schedulable { "是" } else { "否" },
                    sched.total_utilization,
                    sched.utilization_bound.unwrap_or(100.0))
            },
            AnalysisResult::InterruptLatencyAnalysis(jitter) => {
                format!("**中断延迟分析**\n\n- 最小延迟: {} μs\n- 最大延迟: {} μs\n- 平均延迟: {} μs\n- 抖动: {} μs\n",
                    jitter.min_latency, jitter.max_latency, jitter.avg_latency, jitter.jitter)
            },
            AnalysisResult::TimerResolutionAnalysis(timer) => {
                format!("**定时器分辨率分析**\n\n- 定时器频率: {} Hz\n- 预分频器: {}\n- 分辨率: {} ns\n- 最大周期: {} ms\n",
                    timer.timer_frequency, timer.prescaler, timer.resolution_ns, timer.max_period_ms)
            },
            AnalysisResult::PerformanceMonitoring(reports) => {
                let mut result = String::from("**性能监控结果**\n\n");
                for report in reports {
                    result.push_str(&format!("任务 {}: 执行 {} 次, 错过截止时间 {} 次 ({:.2}%)\n",
                        report.task_id, report.total_executions, 
                        report.deadline_misses, report.deadline_miss_rate));
                }
                result
            }
        }
    }
    
    fn format_category(&self, category: &RecommendationCategory) -> &'static str {
        match category {
            RecommendationCategory::TimerConfiguration => "定时器配置",
            RecommendationCategory::InterruptPriority => "中断优先级",
            RecommendationCategory::TaskScheduling => "任务调度",
            RecommendationCategory::SystemOptimization => "系统优化",
        }
    }
}
```

## 总结

定时分析是确保嵌入式实时系统正确性和可靠性的关键技术。本文档详细介绍了：

1. **时序分析基础**：时间概念定义、约束类型、分析方法
2. **定时器精度分析**：时钟源精度、分辨率分析、配置优化
3. **中断延迟分析**：响应时间模型、优先级分析、抖动分析
4. **任务调度分析**：可调度性分析、不同调度算法的分析方法
5. **性能测量工具**：高精度时间戳、统计分析、实时监控
6. **报告生成**：自动化分析报告、优化建议生成

通过系统性的定时分析，开发者可以：
- 验证系统的实时性要求
- 识别潜在的时序问题
- 优化系统配置和任务调度
- 提供量化的性能指标
- 生成详细的分析报告

在实际应用中，建议结合静态分析和动态测量，使用多种分析方法交叉验证，确保系统的时序正确性和可靠性。