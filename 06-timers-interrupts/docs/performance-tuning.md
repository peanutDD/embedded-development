# 性能调优技术文档

## 概述

性能调优是嵌入式实时系统开发中的关键环节，直接影响系统的响应速度、功耗和可靠性。本文档针对STM32定时器和中断系统，提供全面的性能优化策略、实施方法和最佳实践，帮助开发者构建高效的实时系统。

## 性能优化原则

### 优化目标层次

1. **硬实时性能**
   - 确保关键任务在截止时间内完成
   - 最小化中断响应延迟
   - 减少任务调度开销

2. **系统吞吐量**
   - 最大化CPU利用率
   - 优化内存访问模式
   - 减少上下文切换开销

3. **功耗效率**
   - 动态频率调节
   - 智能休眠管理
   - 外设功耗优化

4. **资源利用率**
   - 内存使用优化
   - 定时器资源复用
   - 中断向量优化

### 性能分析方法

```rust
// 性能分析框架
pub struct PerformanceProfiler {
    profiling_enabled: bool,
    sample_buffer: heapless::Vec<ProfileSample, 1000>,
    current_sample_index: usize,
    profiling_overhead: u32,
}

#[derive(Debug, Clone)]
pub struct ProfileSample {
    timestamp: u64,
    event_type: ProfileEventType,
    task_id: u8,
    cpu_cycles: u32,
    memory_usage: u32,
    stack_usage: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ProfileEventType {
    TaskStart,
    TaskEnd,
    InterruptEntry,
    InterruptExit,
    ContextSwitch,
    MemoryAllocation,
    CacheEvent,
    PowerStateChange,
}

impl PerformanceProfiler {
    pub fn new() -> Self {
        Self {
            profiling_enabled: false,
            sample_buffer: heapless::Vec::new(),
            current_sample_index: 0,
            profiling_overhead: 0,
        }
    }
    
    pub fn start_profiling(&mut self) {
        self.profiling_enabled = true;
        self.measure_profiling_overhead();
    }
    
    pub fn stop_profiling(&mut self) {
        self.profiling_enabled = false;
    }
    
    fn measure_profiling_overhead(&mut self) {
        let start_cycles = get_cpu_cycle_count();
        
        // 模拟一次采样操作
        let dummy_sample = ProfileSample {
            timestamp: get_high_precision_timestamp(),
            event_type: ProfileEventType::TaskStart,
            task_id: 0,
            cpu_cycles: 0,
            memory_usage: 0,
            stack_usage: 0,
        };
        
        let end_cycles = get_cpu_cycle_count();
        self.profiling_overhead = end_cycles - start_cycles;
    }
    
    pub fn record_event(&mut self, event_type: ProfileEventType, task_id: u8) {
        if !self.profiling_enabled {
            return;
        }
        
        let sample = ProfileSample {
            timestamp: get_high_precision_timestamp(),
            event_type,
            task_id,
            cpu_cycles: get_cpu_cycle_count(),
            memory_usage: get_memory_usage(),
            stack_usage: get_stack_usage(),
        };
        
        if self.sample_buffer.len() < 1000 {
            self.sample_buffer.push(sample).ok();
        } else {
            self.sample_buffer[self.current_sample_index] = sample;
            self.current_sample_index = (self.current_sample_index + 1) % 1000;
        }
    }
    
    pub fn analyze_performance(&self) -> PerformanceAnalysis {
        let mut task_performance: heapless::FnvIndexMap<u8, TaskPerformanceMetrics, 16> = 
            heapless::FnvIndexMap::new();
        
        let mut current_task_starts: heapless::FnvIndexMap<u8, u64, 16> = 
            heapless::FnvIndexMap::new();
        
        for sample in &self.sample_buffer {
            match sample.event_type {
                ProfileEventType::TaskStart => {
                    current_task_starts.insert(sample.task_id, sample.timestamp).ok();
                },
                ProfileEventType::TaskEnd => {
                    if let Some(&start_time) = current_task_starts.get(&sample.task_id) {
                        let execution_time = sample.timestamp - start_time;
                        
                        let metrics = task_performance.entry(sample.task_id)
                            .or_insert(TaskPerformanceMetrics::new());
                        
                        metrics.update_execution_time(execution_time as u32);
                        metrics.update_cpu_cycles(sample.cpu_cycles);
                        metrics.update_memory_usage(sample.memory_usage);
                    }
                },
                _ => {}
            }
        }
        
        PerformanceAnalysis {
            task_metrics: task_performance,
            profiling_overhead: self.profiling_overhead,
            total_samples: self.sample_buffer.len(),
        }
    }
}

#[derive(Debug)]
pub struct TaskPerformanceMetrics {
    pub execution_count: u32,
    pub total_execution_time: u64,
    pub min_execution_time: u32,
    pub max_execution_time: u32,
    pub total_cpu_cycles: u64,
    pub peak_memory_usage: u32,
    pub cache_misses: u32,
}

impl TaskPerformanceMetrics {
    pub fn new() -> Self {
        Self {
            execution_count: 0,
            total_execution_time: 0,
            min_execution_time: u32::MAX,
            max_execution_time: 0,
            total_cpu_cycles: 0,
            peak_memory_usage: 0,
            cache_misses: 0,
        }
    }
    
    pub fn update_execution_time(&mut self, time: u32) {
        self.execution_count += 1;
        self.total_execution_time += time as u64;
        self.min_execution_time = self.min_execution_time.min(time);
        self.max_execution_time = self.max_execution_time.max(time);
    }
    
    pub fn update_cpu_cycles(&mut self, cycles: u32) {
        self.total_cpu_cycles += cycles as u64;
    }
    
    pub fn update_memory_usage(&mut self, usage: u32) {
        self.peak_memory_usage = self.peak_memory_usage.max(usage);
    }
    
    pub fn average_execution_time(&self) -> u32 {
        if self.execution_count > 0 {
            (self.total_execution_time / self.execution_count as u64) as u32
        } else {
            0
        }
    }
    
    pub fn cycles_per_execution(&self) -> u32 {
        if self.execution_count > 0 {
            (self.total_cpu_cycles / self.execution_count as u64) as u32
        } else {
            0
        }
    }
}

#[derive(Debug)]
pub struct PerformanceAnalysis {
    pub task_metrics: heapless::FnvIndexMap<u8, TaskPerformanceMetrics, 16>,
    pub profiling_overhead: u32,
    pub total_samples: usize,
}
```

## 定时器性能优化

### 定时器配置优化

```rust
// 定时器配置优化器
pub struct TimerOptimizer {
    available_timers: Vec<TimerResource>,
    optimization_goals: OptimizationGoals,
}

#[derive(Debug, Clone)]
pub struct TimerResource {
    pub timer_id: u8,
    pub timer_type: TimerType,
    pub max_frequency: u32,
    pub resolution_bits: u8,
    pub features: TimerFeatures,
    pub power_consumption: u32, // μA
    pub is_available: bool,
}

#[derive(Debug, Clone)]
pub enum TimerType {
    Basic,      // TIM6, TIM7
    General,    // TIM2-TIM5, TIM9-TIM14
    Advanced,   // TIM1, TIM8
}

#[derive(Debug, Clone)]
pub struct TimerFeatures {
    pub pwm_channels: u8,
    pub input_capture: bool,
    pub encoder_interface: bool,
    pub complementary_outputs: bool,
    pub break_input: bool,
    pub dma_support: bool,
}

#[derive(Debug, Clone)]
pub struct OptimizationGoals {
    pub minimize_power: bool,
    pub maximize_precision: bool,
    pub minimize_jitter: bool,
    pub maximize_frequency: bool,
    pub minimize_resource_usage: bool,
}

impl TimerOptimizer {
    pub fn new() -> Self {
        Self {
            available_timers: Self::initialize_timer_resources(),
            optimization_goals: OptimizationGoals {
                minimize_power: false,
                maximize_precision: true,
                minimize_jitter: true,
                maximize_frequency: false,
                minimize_resource_usage: true,
            },
        }
    }
    
    fn initialize_timer_resources() -> Vec<TimerResource> {
        vec![
            TimerResource {
                timer_id: 1,
                timer_type: TimerType::Advanced,
                max_frequency: 168_000_000,
                resolution_bits: 16,
                features: TimerFeatures {
                    pwm_channels: 4,
                    input_capture: true,
                    encoder_interface: true,
                    complementary_outputs: true,
                    break_input: true,
                    dma_support: true,
                },
                power_consumption: 150,
                is_available: true,
            },
            TimerResource {
                timer_id: 2,
                timer_type: TimerType::General,
                max_frequency: 84_000_000,
                resolution_bits: 32,
                features: TimerFeatures {
                    pwm_channels: 4,
                    input_capture: true,
                    encoder_interface: true,
                    complementary_outputs: false,
                    break_input: false,
                    dma_support: true,
                },
                power_consumption: 100,
                is_available: true,
            },
            TimerResource {
                timer_id: 6,
                timer_type: TimerType::Basic,
                max_frequency: 84_000_000,
                resolution_bits: 16,
                features: TimerFeatures {
                    pwm_channels: 0,
                    input_capture: false,
                    encoder_interface: false,
                    complementary_outputs: false,
                    break_input: false,
                    dma_support: true,
                },
                power_consumption: 50,
                is_available: true,
            },
        ]
    }
    
    pub fn optimize_timer_allocation(&self, requirements: &[TimerRequirement]) -> Vec<TimerAllocation> {
        let mut allocations = Vec::new();
        let mut available_timers = self.available_timers.clone();
        
        // 按优先级排序需求
        let mut sorted_requirements = requirements.to_vec();
        sorted_requirements.sort_by_key(|req| req.priority);
        
        for requirement in &sorted_requirements {
            if let Some(allocation) = self.find_optimal_timer(&available_timers, requirement) {
                // 标记定时器为已使用
                if let Some(timer) = available_timers.iter_mut()
                    .find(|t| t.timer_id == allocation.timer_id) {
                    timer.is_available = false;
                }
                allocations.push(allocation);
            }
        }
        
        allocations
    }
    
    fn find_optimal_timer(&self, available_timers: &[TimerResource], 
                         requirement: &TimerRequirement) -> Option<TimerAllocation> {
        let mut best_timer: Option<&TimerResource> = None;
        let mut best_score = f32::MIN;
        
        for timer in available_timers.iter().filter(|t| t.is_available) {
            if self.timer_meets_requirements(timer, requirement) {
                let score = self.calculate_timer_score(timer, requirement);
                if score > best_score {
                    best_score = score;
                    best_timer = Some(timer);
                }
            }
        }
        
        best_timer.map(|timer| {
            let config = self.generate_optimal_config(timer, requirement);
            TimerAllocation {
                timer_id: timer.timer_id,
                requirement_id: requirement.id,
                configuration: config,
                estimated_power: timer.power_consumption,
                estimated_precision: self.calculate_precision(timer, requirement),
            }
        })
    }
    
    fn timer_meets_requirements(&self, timer: &TimerResource, req: &TimerRequirement) -> bool {
        // 检查基本要求
        if req.required_frequency > timer.max_frequency {
            return false;
        }
        
        if req.pwm_channels > timer.features.pwm_channels {
            return false;
        }
        
        if req.input_capture && !timer.features.input_capture {
            return false;
        }
        
        if req.encoder_interface && !timer.features.encoder_interface {
            return false;
        }
        
        if req.dma_required && !timer.features.dma_support {
            return false;
        }
        
        true
    }
    
    fn calculate_timer_score(&self, timer: &TimerResource, req: &TimerRequirement) -> f32 {
        let mut score = 0.0;
        
        // 功耗评分
        if self.optimization_goals.minimize_power {
            score += (1000.0 - timer.power_consumption as f32) / 1000.0 * 30.0;
        }
        
        // 精度评分
        if self.optimization_goals.maximize_precision {
            let precision_score = timer.resolution_bits as f32 / 32.0 * 25.0;
            score += precision_score;
        }
        
        // 资源利用率评分
        if self.optimization_goals.minimize_resource_usage {
            let utilization = self.calculate_resource_utilization(timer, req);
            score += (1.0 - utilization) * 20.0;
        }
        
        // 频率匹配评分
        let frequency_match = req.required_frequency as f32 / timer.max_frequency as f32;
        score += frequency_match * 25.0;
        
        score
    }
    
    fn calculate_resource_utilization(&self, timer: &TimerResource, req: &TimerRequirement) -> f32 {
        let mut utilization = 0.0;
        
        // PWM通道利用率
        if timer.features.pwm_channels > 0 {
            utilization += req.pwm_channels as f32 / timer.features.pwm_channels as f32 * 0.4;
        }
        
        // 频率利用率
        utilization += req.required_frequency as f32 / timer.max_frequency as f32 * 0.6;
        
        utilization.min(1.0)
    }
    
    fn generate_optimal_config(&self, timer: &TimerResource, req: &TimerRequirement) -> TimerConfiguration {
        let prescaler = self.calculate_optimal_prescaler(timer, req);
        let auto_reload = self.calculate_auto_reload(timer, req, prescaler);
        
        TimerConfiguration {
            prescaler,
            auto_reload,
            clock_division: 0,
            counter_mode: CounterMode::Up,
            repetition_counter: 0,
            auto_reload_preload: true,
        }
    }
    
    fn calculate_optimal_prescaler(&self, timer: &TimerResource, req: &TimerRequirement) -> u32 {
        let target_frequency = req.required_frequency;
        let timer_clock = timer.max_frequency;
        
        // 计算最优预分频值以获得所需频率
        let prescaler = (timer_clock / target_frequency).saturating_sub(1);
        prescaler.min(65535) // 16位预分频器最大值
    }
    
    fn calculate_auto_reload(&self, timer: &TimerResource, req: &TimerRequirement, prescaler: u32) -> u32 {
        let effective_clock = timer.max_frequency / (prescaler + 1);
        let period_ticks = effective_clock / req.required_frequency;
        
        let max_value = (1u64 << timer.resolution_bits) - 1;
        period_ticks.min(max_value as u32)
    }
    
    fn calculate_precision(&self, timer: &TimerResource, req: &TimerRequirement) -> f32 {
        let prescaler = self.calculate_optimal_prescaler(timer, req);
        let effective_clock = timer.max_frequency / (prescaler + 1);
        let resolution_ns = 1_000_000_000.0 / effective_clock as f32;
        
        // 返回纳秒级精度
        resolution_ns
    }
}

#[derive(Debug, Clone)]
pub struct TimerRequirement {
    pub id: u8,
    pub priority: u8,
    pub required_frequency: u32,
    pub pwm_channels: u8,
    pub input_capture: bool,
    pub encoder_interface: bool,
    pub dma_required: bool,
    pub power_constraint: Option<u32>,
    pub precision_requirement: Option<f32>,
}

#[derive(Debug)]
pub struct TimerAllocation {
    pub timer_id: u8,
    pub requirement_id: u8,
    pub configuration: TimerConfiguration,
    pub estimated_power: u32,
    pub estimated_precision: f32,
}

#[derive(Debug)]
pub struct TimerConfiguration {
    pub prescaler: u32,
    pub auto_reload: u32,
    pub clock_division: u8,
    pub counter_mode: CounterMode,
    pub repetition_counter: u8,
    pub auto_reload_preload: bool,
}

#[derive(Debug)]
pub enum CounterMode {
    Up,
    Down,
    CenterAligned1,
    CenterAligned2,
    CenterAligned3,
}
```

### 高精度定时优化

```rust
// 高精度定时优化
pub struct HighPrecisionTimingOptimizer {
    calibration_data: CalibrationData,
    compensation_enabled: bool,
}

#[derive(Debug)]
pub struct CalibrationData {
    pub temperature_coefficient: f32,
    pub voltage_coefficient: f32,
    pub aging_coefficient: f32,
    pub reference_temperature: f32,
    pub reference_voltage: f32,
    pub calibration_timestamp: u64,
}

impl HighPrecisionTimingOptimizer {
    pub fn new() -> Self {
        Self {
            calibration_data: CalibrationData {
                temperature_coefficient: -40.0e-6, // ppm/°C
                voltage_coefficient: 0.1e-6,       // ppm/mV
                aging_coefficient: 1.0e-6,         // ppm/year
                reference_temperature: 25.0,       // °C
                reference_voltage: 3300.0,         // mV
                calibration_timestamp: 0,
            },
            compensation_enabled: false,
        }
    }
    
    pub fn calibrate_timing(&mut self, reference_frequency: u32, measurement_duration_ms: u32) -> CalibrationResult {
        let start_time = get_high_precision_timestamp();
        let start_counter = get_timer_counter_value();
        
        // 等待测量周期
        delay_ms(measurement_duration_ms);
        
        let end_time = get_high_precision_timestamp();
        let end_counter = get_timer_counter_value();
        
        let measured_duration = end_time - start_time;
        let counter_ticks = end_counter - start_counter;
        
        let measured_frequency = (counter_ticks as f64 * 1_000_000_000.0) / measured_duration as f64;
        let frequency_error = (measured_frequency - reference_frequency as f64) / reference_frequency as f64;
        
        CalibrationResult {
            reference_frequency,
            measured_frequency: measured_frequency as u32,
            frequency_error_ppm: (frequency_error * 1_000_000.0) as f32,
            measurement_duration_ms,
            calibration_valid: frequency_error.abs() < 0.001, // 1000ppm限制
        }
    }
    
    pub fn enable_temperature_compensation(&mut self, enable: bool) {
        self.compensation_enabled = enable;
    }
    
    pub fn calculate_frequency_correction(&self, current_temperature: f32, current_voltage: f32) -> f32 {
        if !self.compensation_enabled {
            return 1.0;
        }
        
        let temp_delta = current_temperature - self.calibration_data.reference_temperature;
        let voltage_delta = current_voltage - self.calibration_data.reference_voltage;
        
        let temp_correction = 1.0 + (self.calibration_data.temperature_coefficient * temp_delta);
        let voltage_correction = 1.0 + (self.calibration_data.voltage_coefficient * voltage_delta);
        
        temp_correction * voltage_correction
    }
    
    pub fn apply_timing_correction(&self, target_period_us: u32, 
                                  current_temp: f32, current_voltage: f32) -> u32 {
        let correction_factor = self.calculate_frequency_correction(current_temp, current_voltage);
        (target_period_us as f32 * correction_factor) as u32
    }
    
    pub fn optimize_timer_for_precision(&self, timer_config: &mut TimerConfiguration, 
                                       target_frequency: u32) -> PrecisionOptimizationResult {
        let original_config = timer_config.clone();
        
        // 尝试不同的预分频值以获得最佳精度
        let mut best_error = f32::MAX;
        let mut best_prescaler = timer_config.prescaler;
        let mut best_auto_reload = timer_config.auto_reload;
        
        let timer_clock = 84_000_000u32; // 假设84MHz时钟
        
        for prescaler in 0..=65535u32 {
            let effective_clock = timer_clock / (prescaler + 1);
            if effective_clock < target_frequency {
                break;
            }
            
            let auto_reload = effective_clock / target_frequency;
            if auto_reload == 0 || auto_reload > 65535 {
                continue;
            }
            
            let actual_frequency = effective_clock / auto_reload;
            let error = ((actual_frequency as f32 - target_frequency as f32) / target_frequency as f32).abs();
            
            if error < best_error {
                best_error = error;
                best_prescaler = prescaler;
                best_auto_reload = auto_reload;
            }
        }
        
        timer_config.prescaler = best_prescaler;
        timer_config.auto_reload = best_auto_reload;
        
        PrecisionOptimizationResult {
            original_config,
            optimized_config: timer_config.clone(),
            frequency_error_ppm: best_error * 1_000_000.0,
            improvement_factor: if best_error > 0.0 { 
                self.calculate_original_error(&original_config, target_frequency) / best_error 
            } else { 
                1.0 
            },
        }
    }
    
    fn calculate_original_error(&self, config: &TimerConfiguration, target_frequency: u32) -> f32 {
        let timer_clock = 84_000_000u32;
        let effective_clock = timer_clock / (config.prescaler + 1);
        let actual_frequency = effective_clock / config.auto_reload;
        
        ((actual_frequency as f32 - target_frequency as f32) / target_frequency as f32).abs()
    }
}

#[derive(Debug)]
pub struct CalibrationResult {
    pub reference_frequency: u32,
    pub measured_frequency: u32,
    pub frequency_error_ppm: f32,
    pub measurement_duration_ms: u32,
    pub calibration_valid: bool,
}

#[derive(Debug)]
pub struct PrecisionOptimizationResult {
    pub original_config: TimerConfiguration,
    pub optimized_config: TimerConfiguration,
    pub frequency_error_ppm: f32,
    pub improvement_factor: f32,
}
```

## 中断系统优化

### 中断优先级优化

```rust
// 中断优先级优化器
pub struct InterruptPriorityOptimizer {
    interrupt_descriptors: Vec<InterruptDescriptor>,
    optimization_strategy: PriorityOptimizationStrategy,
    system_constraints: SystemConstraints,
}

#[derive(Debug, Clone)]
pub struct InterruptDescriptor {
    pub interrupt_id: u8,
    pub name: &'static str,
    pub current_priority: u8,
    pub max_execution_time_us: u32,
    pub average_execution_time_us: u32,
    pub frequency_hz: f32,
    pub deadline_us: u32,
    pub criticality: CriticalityLevel,
    pub can_be_nested: bool,
    pub resource_dependencies: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum CriticalityLevel {
    Critical,    // 安全关键
    High,        // 高重要性
    Medium,      // 中等重要性
    Low,         // 低重要性
}

#[derive(Debug, Clone)]
pub enum PriorityOptimizationStrategy {
    DeadlineMonotonic,      // 截止时间单调
    RateMonotonic,          // 速率单调
    CriticalityBased,       // 基于关键性
    ResponseTimeOptimal,    // 响应时间最优
    Hybrid,                 // 混合策略
}

#[derive(Debug)]
pub struct SystemConstraints {
    pub max_interrupt_nesting_depth: u8,
    pub max_total_interrupt_load: f32,
    pub priority_groups: u8,
    pub reserved_priorities: Vec<u8>,
}

impl InterruptPriorityOptimizer {
    pub fn new(strategy: PriorityOptimizationStrategy) -> Self {
        Self {
            interrupt_descriptors: Vec::new(),
            optimization_strategy: strategy,
            system_constraints: SystemConstraints {
                max_interrupt_nesting_depth: 4,
                max_total_interrupt_load: 50.0,
                priority_groups: 4,
                reserved_priorities: vec![0, 1], // 保留最高优先级
            },
        }
    }
    
    pub fn add_interrupt(&mut self, descriptor: InterruptDescriptor) {
        self.interrupt_descriptors.push(descriptor);
    }
    
    pub fn optimize_priorities(&mut self) -> PriorityOptimizationResult {
        let original_assignment = self.get_current_priority_assignment();
        
        let optimized_assignment = match self.optimization_strategy {
            PriorityOptimizationStrategy::DeadlineMonotonic => {
                self.deadline_monotonic_assignment()
            },
            PriorityOptimizationStrategy::RateMonotonic => {
                self.rate_monotonic_assignment()
            },
            PriorityOptimizationStrategy::CriticalityBased => {
                self.criticality_based_assignment()
            },
            PriorityOptimizationStrategy::ResponseTimeOptimal => {
                self.response_time_optimal_assignment()
            },
            PriorityOptimizationStrategy::Hybrid => {
                self.hybrid_assignment()
            },
        };
        
        let analysis = self.analyze_priority_assignment(&optimized_assignment);
        
        PriorityOptimizationResult {
            original_assignment,
            optimized_assignment: optimized_assignment.clone(),
            analysis,
            improvements: self.calculate_improvements(&original_assignment, &optimized_assignment),
        }
    }
    
    fn deadline_monotonic_assignment(&self) -> Vec<PriorityAssignment> {
        let mut assignments = Vec::new();
        let mut sorted_interrupts = self.interrupt_descriptors.clone();
        
        // 按截止时间排序（短截止时间优先级高）
        sorted_interrupts.sort_by(|a, b| a.deadline_us.cmp(&b.deadline_us));
        
        let mut current_priority = self.get_highest_available_priority();
        
        for interrupt in sorted_interrupts {
            if !self.system_constraints.reserved_priorities.contains(&current_priority) {
                assignments.push(PriorityAssignment {
                    interrupt_id: interrupt.interrupt_id,
                    assigned_priority: current_priority,
                    rationale: format!("Deadline Monotonic: {} μs deadline", interrupt.deadline_us),
                });
                current_priority += 1;
            }
        }
        
        assignments
    }
    
    fn rate_monotonic_assignment(&self) -> Vec<PriorityAssignment> {
        let mut assignments = Vec::new();
        let mut sorted_interrupts = self.interrupt_descriptors.clone();
        
        // 按频率排序（高频率优先级高）
        sorted_interrupts.sort_by(|a, b| b.frequency_hz.partial_cmp(&a.frequency_hz).unwrap());
        
        let mut current_priority = self.get_highest_available_priority();
        
        for interrupt in sorted_interrupts {
            if !self.system_constraints.reserved_priorities.contains(&current_priority) {
                assignments.push(PriorityAssignment {
                    interrupt_id: interrupt.interrupt_id,
                    assigned_priority: current_priority,
                    rationale: format!("Rate Monotonic: {:.1} Hz frequency", interrupt.frequency_hz),
                });
                current_priority += 1;
            }
        }
        
        assignments
    }
    
    fn criticality_based_assignment(&self) -> Vec<PriorityAssignment> {
        let mut assignments = Vec::new();
        let mut sorted_interrupts = self.interrupt_descriptors.clone();
        
        // 按关键性排序
        sorted_interrupts.sort_by(|a, b| {
            let a_level = match a.criticality {
                CriticalityLevel::Critical => 0,
                CriticalityLevel::High => 1,
                CriticalityLevel::Medium => 2,
                CriticalityLevel::Low => 3,
            };
            let b_level = match b.criticality {
                CriticalityLevel::Critical => 0,
                CriticalityLevel::High => 1,
                CriticalityLevel::Medium => 2,
                CriticalityLevel::Low => 3,
            };
            a_level.cmp(&b_level)
        });
        
        let mut current_priority = self.get_highest_available_priority();
        
        for interrupt in sorted_interrupts {
            if !self.system_constraints.reserved_priorities.contains(&current_priority) {
                assignments.push(PriorityAssignment {
                    interrupt_id: interrupt.interrupt_id,
                    assigned_priority: current_priority,
                    rationale: format!("Criticality Based: {:?} level", interrupt.criticality),
                });
                current_priority += 1;
            }
        }
        
        assignments
    }
    
    fn response_time_optimal_assignment(&self) -> Vec<PriorityAssignment> {
        // 使用启发式算法寻找最优响应时间分配
        let mut best_assignment = self.deadline_monotonic_assignment();
        let mut best_max_response_time = self.calculate_max_response_time(&best_assignment);
        
        // 尝试不同的优先级排列
        let mut attempts = 0;
        const MAX_ATTEMPTS: usize = 1000;
        
        while attempts < MAX_ATTEMPTS {
            let mut test_assignment = best_assignment.clone();
            
            // 随机交换两个中断的优先级
            if test_assignment.len() >= 2 {
                let idx1 = (get_random_number() as usize) % test_assignment.len();
                let idx2 = (get_random_number() as usize) % test_assignment.len();
                
                if idx1 != idx2 {
                    let temp_priority = test_assignment[idx1].assigned_priority;
                    test_assignment[idx1].assigned_priority = test_assignment[idx2].assigned_priority;
                    test_assignment[idx2].assigned_priority = temp_priority;
                    
                    let max_response_time = self.calculate_max_response_time(&test_assignment);
                    if max_response_time < best_max_response_time {
                        best_assignment = test_assignment;
                        best_max_response_time = max_response_time;
                    }
                }
            }
            
            attempts += 1;
        }
        
        // 更新rationale
        for assignment in &mut best_assignment {
            assignment.rationale = "Response Time Optimal".to_string();
        }
        
        best_assignment
    }
    
    fn hybrid_assignment(&self) -> Vec<PriorityAssignment> {
        // 混合策略：首先按关键性分组，然后在组内按截止时间排序
        let mut assignments = Vec::new();
        
        // 按关键性分组
        let critical_interrupts: Vec<_> = self.interrupt_descriptors.iter()
            .filter(|i| i.criticality == CriticalityLevel::Critical)
            .cloned()
            .collect();
        
        let high_interrupts: Vec<_> = self.interrupt_descriptors.iter()
            .filter(|i| i.criticality == CriticalityLevel::High)
            .cloned()
            .collect();
        
        let medium_interrupts: Vec<_> = self.interrupt_descriptors.iter()
            .filter(|i| i.criticality == CriticalityLevel::Medium)
            .cloned()
            .collect();
        
        let low_interrupts: Vec<_> = self.interrupt_descriptors.iter()
            .filter(|i| i.criticality == CriticalityLevel::Low)
            .cloned()
            .collect();
        
        let mut current_priority = self.get_highest_available_priority();
        
        // 处理每个关键性级别
        for group in [critical_interrupts, high_interrupts, medium_interrupts, low_interrupts] {
            let mut sorted_group = group;
            sorted_group.sort_by(|a, b| a.deadline_us.cmp(&b.deadline_us));
            
            for interrupt in sorted_group {
                if !self.system_constraints.reserved_priorities.contains(&current_priority) {
                    assignments.push(PriorityAssignment {
                        interrupt_id: interrupt.interrupt_id,
                        assigned_priority: current_priority,
                        rationale: format!("Hybrid: {:?} + {} μs deadline", 
                                         interrupt.criticality, interrupt.deadline_us),
                    });
                    current_priority += 1;
                }
            }
        }
        
        assignments
    }
    
    fn get_current_priority_assignment(&self) -> Vec<PriorityAssignment> {
        self.interrupt_descriptors.iter()
            .map(|interrupt| PriorityAssignment {
                interrupt_id: interrupt.interrupt_id,
                assigned_priority: interrupt.current_priority,
                rationale: "Current Assignment".to_string(),
            })
            .collect()
    }
    
    fn get_highest_available_priority(&self) -> u8 {
        // 找到第一个未保留的优先级
        for priority in 0..=255u8 {
            if !self.system_constraints.reserved_priorities.contains(&priority) {
                return priority;
            }
        }
        255 // 默认最低优先级
    }
    
    fn analyze_priority_assignment(&self, assignment: &[PriorityAssignment]) -> PriorityAnalysis {
        let mut schedulable_interrupts = 0;
        let mut total_response_time = 0u32;
        let mut max_response_time = 0u32;
        let mut violations = Vec::new();
        
        for interrupt in &self.interrupt_descriptors {
            if let Some(assignment) = assignment.iter()
                .find(|a| a.interrupt_id == interrupt.interrupt_id) {
                
                let response_time = self.calculate_interrupt_response_time(interrupt, assignment, assignment);
                total_response_time += response_time;
                max_response_time = max_response_time.max(response_time);
                
                if response_time <= interrupt.deadline_us {
                    schedulable_interrupts += 1;
                } else {
                    violations.push(PriorityViolation {
                        interrupt_id: interrupt.interrupt_id,
                        deadline_us: interrupt.deadline_us,
                        response_time_us: response_time,
                        violation_margin_us: response_time - interrupt.deadline_us,
                    });
                }
            }
        }
        
        let total_interrupt_load = self.calculate_total_interrupt_load();
        
        PriorityAnalysis {
            total_interrupts: self.interrupt_descriptors.len(),
            schedulable_interrupts,
            average_response_time: if self.interrupt_descriptors.len() > 0 {
                total_response_time / self.interrupt_descriptors.len() as u32
            } else {
                0
            },
            max_response_time,
            total_interrupt_load,
            violations,
            is_schedulable: violations.is_empty() && total_interrupt_load <= self.system_constraints.max_total_interrupt_load,
        }
    }
    
    fn calculate_interrupt_response_time(&self, target: &InterruptDescriptor, 
                                       target_assignment: &PriorityAssignment,
                                       all_assignments: &[PriorityAssignment]) -> u32 {
        let mut response_time = target.max_execution_time_us;
        
        // 计算高优先级中断的干扰
        for interrupt in &self.interrupt_descriptors {
            if let Some(assignment) = all_assignments.iter()
                .find(|a| a.interrupt_id == interrupt.interrupt_id) {
                
                if assignment.assigned_priority < target_assignment.assigned_priority {
                    // 更高优先级的中断
                    let interference_count = ((response_time as f32 * interrupt.frequency_hz) / 1_000_000.0).ceil() as u32;
                    response_time += interference_count * interrupt.max_execution_time_us;
                }
            }
        }
        
        response_time
    }
    
    fn calculate_max_response_time(&self, assignment: &[PriorityAssignment]) -> u32 {
        let mut max_response_time = 0u32;
        
        for interrupt in &self.interrupt_descriptors {
            if let Some(target_assignment) = assignment.iter()
                .find(|a| a.interrupt_id == interrupt.interrupt_id) {
                
                let response_time = self.calculate_interrupt_response_time(interrupt, target_assignment, assignment);
                max_response_time = max_response_time.max(response_time);
            }
        }
        
        max_response_time
    }
    
    fn calculate_total_interrupt_load(&self) -> f32 {
        self.interrupt_descriptors.iter()
            .map(|interrupt| {
                (interrupt.average_execution_time_us as f32 / 1_000_000.0) * interrupt.frequency_hz
            })
            .sum::<f32>() * 100.0
    }
    
    fn calculate_improvements(&self, original: &[PriorityAssignment], 
                            optimized: &[PriorityAssignment]) -> Vec<Improvement> {
        let original_analysis = self.analyze_priority_assignment(original);
        let optimized_analysis = self.analyze_priority_assignment(optimized);
        
        let mut improvements = Vec::new();
        
        if optimized_analysis.max_response_time < original_analysis.max_response_time {
            improvements.push(Improvement {
                metric: "Max Response Time".to_string(),
                original_value: original_analysis.max_response_time as f32,
                optimized_value: optimized_analysis.max_response_time as f32,
                improvement_percent: ((original_analysis.max_response_time - optimized_analysis.max_response_time) as f32 
                                    / original_analysis.max_response_time as f32) * 100.0,
            });
        }
        
        if optimized_analysis.schedulable_interrupts > original_analysis.schedulable_interrupts {
            improvements.push(Improvement {
                metric: "Schedulable Interrupts".to_string(),
                original_value: original_analysis.schedulable_interrupts as f32,
                optimized_value: optimized_analysis.schedulable_interrupts as f32,
                improvement_percent: ((optimized_analysis.schedulable_interrupts - original_analysis.schedulable_interrupts) as f32 
                                    / original_analysis.schedulable_interrupts as f32) * 100.0,
            });
        }
        
        improvements
    }
}

#[derive(Debug, Clone)]
pub struct PriorityAssignment {
    pub interrupt_id: u8,
    pub assigned_priority: u8,
    pub rationale: String,
}

#[derive(Debug)]
pub struct PriorityOptimizationResult {
    pub original_assignment: Vec<PriorityAssignment>,
    pub optimized_assignment: Vec<PriorityAssignment>,
    pub analysis: PriorityAnalysis,
    pub improvements: Vec<Improvement>,
}

#[derive(Debug)]
pub struct PriorityAnalysis {
    pub total_interrupts: usize,
    pub schedulable_interrupts: usize,
    pub average_response_time: u32,
    pub max_response_time: u32,
    pub total_interrupt_load: f32,
    pub violations: Vec<PriorityViolation>,
    pub is_schedulable: bool,
}

#[derive(Debug)]
pub struct PriorityViolation {
    pub interrupt_id: u8,
    pub deadline_us: u32,
    pub response_time_us: u32,
    pub violation_margin_us: u32,
}

#[derive(Debug)]
pub struct Improvement {
    pub metric: String,
    pub original_value: f32,
    pub optimized_value: f32,
    pub improvement_percent: f32,
}

// 辅助函数
fn get_random_number() -> u32 {
    // 简单的线性同余生成器
    static mut SEED: u32 = 1;
    unsafe {
        SEED = SEED.wrapping_mul(1103515245).wrapping_add(12345);
        SEED
    }
}
```

### 中断处理优化

```rust
// 中断处理优化器
pub struct InterruptHandlerOptimizer {
    optimization_techniques: Vec<OptimizationTechnique>,
    performance_metrics: HandlerPerformanceMetrics,
}

#[derive(Debug, Clone)]
pub enum OptimizationTechnique {
    MinimizeRegisterSaves,      // 最小化寄存器保存
    UseInlineAssembly,          // 使用内联汇编
    OptimizeDataAccess,         // 优化数据访问
    ReduceBranchPredictionMiss, // 减少分支预测失误
    EnableTailChaining,         // 启用尾链
    UseFastInterruptReturn,     // 使用快速中断返回
}

#[derive(Debug)]
pub struct HandlerPerformanceMetrics {
    pub entry_latency_cycles: u32,
    pub exit_latency_cycles: u32,
    pub context_save_cycles: u32,
    pub context_restore_cycles: u32,
    pub handler_execution_cycles: u32,
    pub total_overhead_cycles: u32,
}

impl InterruptHandlerOptimizer {
    pub fn new() -> Self {
        Self {
            optimization_techniques: vec![
                OptimizationTechnique::MinimizeRegisterSaves,
                OptimizationTechnique::OptimizeDataAccess,
                OptimizationTechnique::EnableTailChaining,
            ],
            performance_metrics: HandlerPerformanceMetrics {
                entry_latency_cycles: 12,
                exit_latency_cycles: 12,
                context_save_cycles: 34,
                context_restore_cycles: 34,
                handler_execution_cycles: 0,
                total_overhead_cycles: 92,
            },
        }
    }
    
    pub fn optimize_handler(&self, handler_code: &str) -> OptimizedHandler {
        let mut optimized_code = handler_code.to_string();
        let mut applied_optimizations = Vec::new();
        let mut estimated_improvement = 0u32;
        
        for technique in &self.optimization_techniques {
            match technique {
                OptimizationTechnique::MinimizeRegisterSaves => {
                    if let Some((code, improvement)) = self.apply_minimal_register_saves(&optimized_code) {
                        optimized_code = code;
                        estimated_improvement += improvement;
                        applied_optimizations.push(technique.clone());
                    }
                },
                OptimizationTechnique::OptimizeDataAccess => {
                    if let Some((code, improvement)) = self.optimize_data_access(&optimized_code) {
                        optimized_code = code;
                        estimated_improvement += improvement;
                        applied_optimizations.push(technique.clone());
                    }
                },
                OptimizationTechnique::EnableTailChaining => {
                    if let Some((code, improvement)) = self.enable_tail_chaining(&optimized_code) {
                        optimized_code = code;
                        estimated_improvement += improvement;
                        applied_optimizations.push(technique.clone());
                    }
                },
                _ => {}
            }
        }
        
        OptimizedHandler {
            original_code: handler_code.to_string(),
            optimized_code,
            applied_optimizations,
            estimated_cycle_reduction: estimated_improvement,
            optimization_notes: self.generate_optimization_notes(&applied_optimizations),
        }
    }
    
    fn apply_minimal_register_saves(&self, code: &str) -> Option<(String, u32)> {
        // 分析代码中使用的寄存器，只保存必要的寄存器
        if code.contains("// Full context save") {
            let optimized = code.replace(
                "// Full context save",
                "// Minimal context save - only used registers"
            );
            Some((optimized, 20)) // 估计节省20个周期
        } else {
            None
        }
    }
    
    fn optimize_data_access(&self, code: &str) -> Option<(String, u32)> {
        // 优化内存访问模式
        if code.contains("volatile read") {
            let optimized = code.replace(
                "volatile read",
                "cached read with memory barrier"
            );
            Some((optimized, 5)) // 估计节省5个周期
        } else {
            None
        }
    }
    
    fn enable_tail_chaining(&self, code: &str) -> Option<(String, u32)> {
        // 启用中断尾链优化
        if !code.contains("tail_chaining_enabled") {
            let optimized = format!("{}\n// tail_chaining_enabled", code);
            Some((optimized, 12)) // 节省上下文恢复/保存开销
        } else {
            None
        }
    }
    
    fn generate_optimization_notes(&self, optimizations: &[OptimizationTechnique]) -> Vec<String> {
        optimizations.iter().map(|opt| {
            match opt {
                OptimizationTechnique::MinimizeRegisterSaves => {
                    "Reduced context save/restore overhead by only saving used registers".to_string()
                },
                OptimizationTechnique::OptimizeDataAccess => {
                    "Optimized memory access patterns to reduce cache misses".to_string()
                },
                OptimizationTechnique::EnableTailChaining => {
                    "Enabled interrupt tail chaining to reduce context switch overhead".to_string()
                },
                _ => format!("Applied optimization: {:?}", opt),
            }
        }).collect()
    }
    
    pub fn measure_handler_performance(&mut self, handler_function: fn()) -> HandlerPerformanceMetrics {
        let start_cycles = get_cpu_cycle_count();
        
        // 模拟中断入口开销
        let entry_start = get_cpu_cycle_count();
        // 中断入口处理...
        let entry_end = get_cpu_cycle_count();
        
        // 执行处理函数
        let handler_start = get_cpu_cycle_count();
        handler_function();
        let handler_end = get_cpu_cycle_count();
        
        // 模拟中断出口开销
        let exit_start = get_cpu_cycle_count();
        // 中断出口处理...
        let exit_end = get_cpu_cycle_count();
        
        let metrics = HandlerPerformanceMetrics {
            entry_latency_cycles: entry_end - entry_start,
            exit_latency_cycles: exit_end - exit_start,
            context_save_cycles: 34, // 固定值，基于ARM Cortex-M4
            context_restore_cycles: 34,
            handler_execution_cycles: handler_end - handler_start,
            total_overhead_cycles: (entry_end - entry_start) + (exit_end - exit_start) + 68,
        };
        
        self.performance_metrics = metrics.clone();
        metrics
    }
    
    pub fn generate_optimized_handler_template(&self, handler_name: &str, 
                                             optimization_level: OptimizationLevel) -> String {
        let mut template = String::new();
        
        template.push_str(&format!("// Optimized interrupt handler: {}\n", handler_name));
        template.push_str(&format!("// Optimization level: {:?}\n", optimization_level));
        template.push_str("// Generated by InterruptHandlerOptimizer\n\n");
        
        match optimization_level {
            OptimizationLevel::Minimal => {
                template.push_str(&self.generate_minimal_template(handler_name));
            },
            OptimizationLevel::Balanced => {
                template.push_str(&self.generate_balanced_template(handler_name));
            },
            OptimizationLevel::Aggressive => {
                template.push_str(&self.generate_aggressive_template(handler_name));
            },
        }
        
        template
    }
    
    fn generate_minimal_template(&self, handler_name: &str) -> String {
        format!(r#"
#[interrupt]
fn {}() {{
    // Minimal optimization - basic handler structure
    
    // Clear interrupt flag
    clear_interrupt_flag();
    
    // Handler logic here
    handle_interrupt_minimal();
}}

fn handle_interrupt_minimal() {{
    // Keep handler logic simple and fast
    // Avoid complex operations
    // Use direct register access where possible
}}
"#, handler_name)
    }
    
    fn generate_balanced_template(&self, handler_name: &str) -> String {
        format!(r#"
#[interrupt]
fn {}() {{
    // Balanced optimization - good performance with maintainability
    
    // Fast path for common cases
    if is_common_case() {{
        handle_fast_path();
        return;
    }}
    
    // Full processing for complex cases
    handle_complex_case();
}}

#[inline(always)]
fn handle_fast_path() {{
    // Optimized for most frequent interrupt causes
    // Minimal branching
    // Direct hardware access
}}

fn handle_complex_case() {{
    // Handle less common but more complex scenarios
    // Can use more sophisticated logic here
}}

#[inline(always)]
fn is_common_case() -> bool {{
    // Quick check for common interrupt conditions
    // Should be very fast (1-2 cycles)
    check_status_register() & COMMON_MASK != 0
}}
"#, handler_name)
    }
    
    fn generate_aggressive_template(&self, handler_name: &str) -> String {
        format!(r#"
// Aggressive optimization - maximum performance
// May sacrifice some code readability for speed

#[naked]
#[no_mangle]
pub unsafe extern "C" fn {}() {{
    // Naked function for complete control over prologue/epilogue
    asm!(
        // Custom context save - only save what we use
        "push {{r0, r1, r2, r3}}",
        
        // Fast interrupt processing
        "ldr r0, =INTERRUPT_STATUS_REG",
        "ldr r1, [r0]",
        "ldr r2, =INTERRUPT_CLEAR_REG", 
        "str r1, [r2]",
        
        // Call optimized handler
        "bl {}_optimized_handler",
        
        // Custom context restore
        "pop {{r0, r1, r2, r3}}",
        "bx lr",
        
        options(noreturn)
    );
}}

#[inline(always)]
fn {}_optimized_handler() {{
    // Ultra-fast handler implementation
    // Use only stack variables
    // Minimize memory access
    // Unroll critical loops
    
    let status = unsafe {{ read_volatile(INTERRUPT_STATUS_REG) }};
    
    // Unrolled bit checking for maximum speed
    if status & 0x01 != 0 {{ handle_bit_0(); }}
    if status & 0x02 != 0 {{ handle_bit_1(); }}
    if status & 0x04 != 0 {{ handle_bit_2(); }}
    if status & 0x08 != 0 {{ handle_bit_3(); }}
}}

#[inline(always)]
fn handle_bit_0() {{
    // Bit 0 handler - keep it minimal
    unsafe {{ write_volatile(ACTION_REG_0, 1); }}
}}

#[inline(always)]
fn handle_bit_1() {{
    // Bit 1 handler - keep it minimal  
    unsafe {{ write_volatile(ACTION_REG_1, 1); }}
}}

// Continue for other bits...
"#, handler_name, handler_name, handler_name)
    }
}

#[derive(Debug, Clone)]
pub enum OptimizationLevel {
    Minimal,    // 基本优化
    Balanced,   // 平衡优化
    Aggressive, // 激进优化
}

#[derive(Debug)]
pub struct OptimizedHandler {
    pub original_code: String,
    pub optimized_code: String,
    pub applied_optimizations: Vec<OptimizationTechnique>,
    pub estimated_cycle_reduction: u32,
    pub optimization_notes: Vec<String>,
}

// 辅助函数（需要根据实际硬件实现）
fn get_cpu_cycle_count() -> u32 {
    // 使用DWT CYCCNT寄存器
    unsafe {
        core::ptr::read_volatile(0xE0001004 as *const u32)
    }
}

fn clear_interrupt_flag() {
    // 清除中断标志的实现
}

fn check_status_register() -> u32 {
    // 读取状态寄存器
    0
}

const COMMON_MASK: u32 = 0xFF;
const INTERRUPT_STATUS_REG: *mut u32 = 0x40000000 as *mut u32;
const INTERRUPT_CLEAR_REG: *mut u32 = 0x40000004 as *mut u32;
const ACTION_REG_0: *mut u32 = 0x40000008 as *mut u32;
const ACTION_REG_1: *mut u32 = 0x4000000C as *mut u32;

fn read_volatile<T>(addr: *const T) -> T {
    unsafe { core::ptr::read_volatile(addr) }
}

fn write_volatile<T>(addr: *mut T, value: T) {
    unsafe { core::ptr::write_volatile(addr, value) }
}
```

## 系统级优化

### 功耗优化

```rust
// 功耗优化管理器
pub struct PowerOptimizationManager {
    power_modes: Vec<PowerMode>,
    current_mode: PowerModeId,
    optimization_strategy: PowerOptimizationStrategy,
    power_budget: PowerBudget,
}

#[derive(Debug, Clone)]
pub struct PowerMode {
    pub id: PowerModeId,
    pub name: &'static str,
    pub cpu_frequency: u32,
    pub peripheral_states: PeripheralStates,
    pub estimated_power_mw: f32,
    pub wakeup_latency_us: u32,
    pub entry_conditions: Vec<PowerModeCondition>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PowerModeId {
    HighPerformance,
    Balanced,
    PowerSaver,
    Sleep,
    DeepSleep,
    Standby,
}

#[derive(Debug, Clone)]
pub struct PeripheralStates {
    pub timers_enabled: Vec<u8>,
    pub uart_enabled: bool,
    pub spi_enabled: bool,
    pub i2c_enabled: bool,
    pub adc_enabled: bool,
    pub dac_enabled: bool,
    pub gpio_states: u32,
}

#[derive(Debug, Clone)]
pub enum PowerModeCondition {
    CpuUtilizationBelow(f32),
    NoActiveTimers,
    NoActiveCommunication,
    BatteryLevelAbove(f32),
    TemperatureBelow(f32),
    IdleTimeAbove(u32),
}

#[derive(Debug, Clone)]
pub enum PowerOptimizationStrategy {
    MaximizeBatteryLife,
    BalancePerformancePower,
    MinimizeLatency,
    AdaptiveDynamic,
}

#[derive(Debug)]
pub struct PowerBudget {
    pub total_budget_mw: f32,
    pub cpu_allocation_mw: f32,
    pub peripheral_allocation_mw: f32,
    pub communication_allocation_mw: f32,
    pub reserve_allocation_mw: f32,
}

impl PowerOptimizationManager {
    pub fn new(strategy: PowerOptimizationStrategy) -> Self {
        Self {
            power_modes: Self::initialize_power_modes(),
            current_mode: PowerModeId::Balanced,
            optimization_strategy: strategy,
            power_budget: PowerBudget {
                total_budget_mw: 100.0,
                cpu_allocation_mw: 50.0,
                peripheral_allocation_mw: 30.0,
                communication_allocation_mw: 15.0,
                reserve_allocation_mw: 5.0,
            },
        }
    }
    
    fn initialize_power_modes() -> Vec<PowerMode> {
        vec![
            PowerMode {
                id: PowerModeId::HighPerformance,
                name: "High Performance",
                cpu_frequency: 168_000_000,
                peripheral_states: PeripheralStates {
                    timers_enabled: vec![1, 2, 3, 4, 5, 6, 7, 8],
                    uart_enabled: true,
                    spi_enabled: true,
                    i2c_enabled: true,
                    adc_enabled: true,
                    dac_enabled: true,
                    gpio_states: 0xFFFFFFFF,
                },