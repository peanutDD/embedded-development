#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use rtic_monotonics::systick::*;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, PushPull, Pin},
    adc::{Adc, config::AdcConfig},
    timer::Timer,
};
use heapless::{Vec, FnvIndexMap};
use micromath::F32Ext;

// 性能指标类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MetricType {
    CpuUsage,
    MemoryUsage,
    StackUsage,
    InterruptLatency,
    TaskExecutionTime,
    PowerConsumption,
    Temperature,
    Frequency,
}

// 性能数据点
#[derive(Debug, Clone, Copy)]
pub struct MetricData {
    pub metric_type: MetricType,
    pub timestamp: u64,
    pub value: f32,
    pub min_value: f32,
    pub max_value: f32,
    pub average: f32,
    pub sample_count: u32,
}

impl MetricData {
    fn new(metric_type: MetricType, value: f32) -> Self {
        Self {
            metric_type,
            timestamp: 0,
            value,
            min_value: value,
            max_value: value,
            average: value,
            sample_count: 1,
        }
    }
    
    fn update(&mut self, new_value: f32, timestamp: u64) {
        self.timestamp = timestamp;
        self.value = new_value;
        
        if new_value < self.min_value {
            self.min_value = new_value;
        }
        if new_value > self.max_value {
            self.max_value = new_value;
        }
        
        // 更新移动平均
        let alpha = 0.1; // 平滑因子
        self.average = self.average * (1.0 - alpha) + new_value * alpha;
        self.sample_count += 1;
    }
}

// CPU使用率监控器
#[derive(Debug)]
pub struct CpuMonitor {
    idle_cycles: u64,
    total_cycles: u64,
    last_measurement: u64,
    usage_history: Vec<f32, 64>,
}

impl CpuMonitor {
    fn new() -> Self {
        Self {
            idle_cycles: 0,
            total_cycles: 0,
            last_measurement: 0,
            usage_history: Vec::new(),
        }
    }
    
    fn start_idle(&mut self) {
        self.last_measurement = cortex_m::peripheral::DWT::cycle_count() as u64;
    }
    
    fn end_idle(&mut self) {
        let current_cycles = cortex_m::peripheral::DWT::cycle_count() as u64;
        let idle_duration = current_cycles.wrapping_sub(self.last_measurement);
        self.idle_cycles = self.idle_cycles.wrapping_add(idle_duration);
    }
    
    fn update_total_cycles(&mut self) {
        let current_cycles = cortex_m::peripheral::DWT::cycle_count() as u64;
        self.total_cycles = current_cycles;
    }
    
    fn calculate_usage(&mut self) -> f32 {
        if self.total_cycles == 0 {
            return 0.0;
        }
        
        let usage = 1.0 - (self.idle_cycles as f32 / self.total_cycles as f32);
        let usage_percent = (usage * 100.0).max(0.0).min(100.0);
        
        // 记录历史数据
        if self.usage_history.len() >= self.usage_history.capacity() {
            self.usage_history.remove(0);
        }
        self.usage_history.push(usage_percent).ok();
        
        // 重置计数器
        self.idle_cycles = 0;
        self.total_cycles = 0;
        
        usage_percent
    }
    
    fn get_average_usage(&self) -> f32 {
        if self.usage_history.is_empty() {
            0.0
        } else {
            let sum: f32 = self.usage_history.iter().sum();
            sum / self.usage_history.len() as f32
        }
    }
    
    fn get_peak_usage(&self) -> f32 {
        self.usage_history.iter().fold(0.0, |max, &val| max.max(val))
    }
}

// 内存使用监控器
#[derive(Debug)]
pub struct MemoryMonitor {
    heap_start: usize,
    heap_size: usize,
    stack_start: usize,
    stack_size: usize,
    allocated_bytes: usize,
    peak_allocated: usize,
    allocation_count: u32,
    deallocation_count: u32,
}

impl MemoryMonitor {
    fn new(heap_start: usize, heap_size: usize, stack_start: usize, stack_size: usize) -> Self {
        Self {
            heap_start,
            heap_size,
            stack_start,
            stack_size,
            allocated_bytes: 0,
            peak_allocated: 0,
            allocation_count: 0,
            deallocation_count: 0,
        }
    }
    
    fn record_allocation(&mut self, size: usize) {
        self.allocated_bytes += size;
        self.allocation_count += 1;
        
        if self.allocated_bytes > self.peak_allocated {
            self.peak_allocated = self.allocated_bytes;
        }
    }
    
    fn record_deallocation(&mut self, size: usize) {
        self.allocated_bytes = self.allocated_bytes.saturating_sub(size);
        self.deallocation_count += 1;
    }
    
    fn get_heap_usage_percent(&self) -> f32 {
        if self.heap_size == 0 {
            0.0
        } else {
            (self.allocated_bytes as f32 / self.heap_size as f32) * 100.0
        }
    }
    
    fn get_stack_usage_percent(&self) -> f32 {
        // 简化的栈使用率计算（实际实现需要栈指针检测）
        let current_sp = cortex_m::register::msp::read() as usize;
        let stack_used = self.stack_start.saturating_sub(current_sp);
        
        if self.stack_size == 0 {
            0.0
        } else {
            (stack_used as f32 / self.stack_size as f32) * 100.0
        }
    }
    
    fn get_fragmentation_ratio(&self) -> f32 {
        // 简化的碎片率计算
        if self.allocation_count == 0 {
            0.0
        } else {
            let avg_allocation_size = self.allocated_bytes as f32 / self.allocation_count as f32;
            let fragmentation = 1.0 - (avg_allocation_size / 1024.0).min(1.0);
            fragmentation * 100.0
        }
    }
}

// 中断延迟监控器
#[derive(Debug)]
pub struct InterruptLatencyMonitor {
    interrupt_start_time: u32,
    latency_samples: Vec<u32, 32>,
    max_latency: u32,
    total_interrupts: u32,
}

impl InterruptLatencyMonitor {
    fn new() -> Self {
        Self {
            interrupt_start_time: 0,
            latency_samples: Vec::new(),
            max_latency: 0,
            total_interrupts: 0,
        }
    }
    
    fn start_interrupt(&mut self) {
        self.interrupt_start_time = cortex_m::peripheral::DWT::cycle_count();
    }
    
    fn end_interrupt(&mut self) {
        let end_time = cortex_m::peripheral::DWT::cycle_count();
        let latency = end_time.wrapping_sub(self.interrupt_start_time);
        
        if latency > self.max_latency {
            self.max_latency = latency;
        }
        
        if self.latency_samples.len() >= self.latency_samples.capacity() {
            self.latency_samples.remove(0);
        }
        self.latency_samples.push(latency).ok();
        
        self.total_interrupts += 1;
    }
    
    fn get_average_latency_us(&self, cpu_freq_hz: u32) -> f32 {
        if self.latency_samples.is_empty() {
            0.0
        } else {
            let sum: u32 = self.latency_samples.iter().sum();
            let avg_cycles = sum as f32 / self.latency_samples.len() as f32;
            (avg_cycles / cpu_freq_hz as f32) * 1_000_000.0
        }
    }
    
    fn get_max_latency_us(&self, cpu_freq_hz: u32) -> f32 {
        (self.max_latency as f32 / cpu_freq_hz as f32) * 1_000_000.0
    }
}

// 任务执行时间监控器
#[derive(Debug)]
pub struct TaskMonitor {
    task_start_times: FnvIndexMap<u8, u32, 16>,
    execution_times: FnvIndexMap<u8, Vec<u32, 16>, 16>,
    task_counts: FnvIndexMap<u8, u32, 16>,
}

impl TaskMonitor {
    fn new() -> Self {
        Self {
            task_start_times: FnvIndexMap::new(),
            execution_times: FnvIndexMap::new(),
            task_counts: FnvIndexMap::new(),
        }
    }
    
    fn start_task(&mut self, task_id: u8) {
        let start_time = cortex_m::peripheral::DWT::cycle_count();
        self.task_start_times.insert(task_id, start_time).ok();
    }
    
    fn end_task(&mut self, task_id: u8) {
        if let Some(&start_time) = self.task_start_times.get(&task_id) {
            let end_time = cortex_m::peripheral::DWT::cycle_count();
            let execution_time = end_time.wrapping_sub(start_time);
            
            // 记录执行时间
            let times = self.execution_times.entry(task_id).or_insert(Vec::new());
            if times.len() >= times.capacity() {
                times.remove(0);
            }
            times.push(execution_time).ok();
            
            // 更新任务计数
            let count = self.task_counts.entry(task_id).or_insert(0);
            *count += 1;
            
            self.task_start_times.remove(&task_id);
        }
    }
    
    fn get_average_execution_time_us(&self, task_id: u8, cpu_freq_hz: u32) -> f32 {
        if let Some(times) = self.execution_times.get(&task_id) {
            if !times.is_empty() {
                let sum: u32 = times.iter().sum();
                let avg_cycles = sum as f32 / times.len() as f32;
                (avg_cycles / cpu_freq_hz as f32) * 1_000_000.0
            } else {
                0.0
            }
        } else {
            0.0
        }
    }
    
    fn get_max_execution_time_us(&self, task_id: u8, cpu_freq_hz: u32) -> f32 {
        if let Some(times) = self.execution_times.get(&task_id) {
            if let Some(&max_cycles) = times.iter().max() {
                (max_cycles as f32 / cpu_freq_hz as f32) * 1_000_000.0
            } else {
                0.0
            }
        } else {
            0.0
        }
    }
    
    fn get_task_frequency(&self, task_id: u8, measurement_period_ms: u32) -> f32 {
        if let Some(&count) = self.task_counts.get(&task_id) {
            (count as f32 * 1000.0) / measurement_period_ms as f32
        } else {
            0.0
        }
    }
}

// 功耗监控器
#[derive(Debug)]
pub struct PowerMonitor {
    voltage_samples: Vec<f32, 64>,
    current_samples: Vec<f32, 64>,
    power_samples: Vec<f32, 64>,
    energy_consumed: f32,
    last_measurement_time: u64,
}

impl PowerMonitor {
    fn new() -> Self {
        Self {
            voltage_samples: Vec::new(),
            current_samples: Vec::new(),
            power_samples: Vec::new(),
            energy_consumed: 0.0,
            last_measurement_time: 0,
        }
    }
    
    fn add_measurement(&mut self, voltage: f32, current: f32, timestamp: u64) {
        let power = voltage * current;
        
        // 记录样本
        if self.voltage_samples.len() >= self.voltage_samples.capacity() {
            self.voltage_samples.remove(0);
        }
        self.voltage_samples.push(voltage).ok();
        
        if self.current_samples.len() >= self.current_samples.capacity() {
            self.current_samples.remove(0);
        }
        self.current_samples.push(current).ok();
        
        if self.power_samples.len() >= self.power_samples.capacity() {
            self.power_samples.remove(0);
        }
        self.power_samples.push(power).ok();
        
        // 计算能耗
        if self.last_measurement_time > 0 {
            let time_delta = (timestamp - self.last_measurement_time) as f32 / 1000.0; // 转换为秒
            self.energy_consumed += power * time_delta;
        }
        
        self.last_measurement_time = timestamp;
    }
    
    fn get_average_power(&self) -> f32 {
        if self.power_samples.is_empty() {
            0.0
        } else {
            let sum: f32 = self.power_samples.iter().sum();
            sum / self.power_samples.len() as f32
        }
    }
    
    fn get_peak_power(&self) -> f32 {
        self.power_samples.iter().fold(0.0, |max, &val| max.max(val))
    }
    
    fn get_total_energy(&self) -> f32 {
        self.energy_consumed
    }
    
    fn get_efficiency_ratio(&self) -> f32 {
        let avg_power = self.get_average_power();
        let peak_power = self.get_peak_power();
        
        if peak_power > 0.0 {
            avg_power / peak_power
        } else {
            0.0
        }
    }
}

// 系统性能监控器
#[derive(Debug)]
pub struct SystemPerformanceMonitor {
    cpu_monitor: CpuMonitor,
    memory_monitor: MemoryMonitor,
    interrupt_monitor: InterruptLatencyMonitor,
    task_monitor: TaskMonitor,
    power_monitor: PowerMonitor,
    metrics: FnvIndexMap<MetricType, MetricData, 16>,
    cpu_frequency: u32,
    measurement_interval: u32,
    last_update: u64,
}

impl SystemPerformanceMonitor {
    fn new(cpu_frequency: u32, measurement_interval: u32) -> Self {
        Self {
            cpu_monitor: CpuMonitor::new(),
            memory_monitor: MemoryMonitor::new(0x20000000, 128 * 1024, 0x20020000, 8 * 1024),
            interrupt_monitor: InterruptLatencyMonitor::new(),
            task_monitor: TaskMonitor::new(),
            power_monitor: PowerMonitor::new(),
            metrics: FnvIndexMap::new(),
            cpu_frequency,
            measurement_interval,
            last_update: 0,
        }
    }
    
    fn update_metrics(&mut self, timestamp: u64) {
        if timestamp.wrapping_sub(self.last_update) < self.measurement_interval as u64 {
            return;
        }
        
        // 更新CPU使用率
        let cpu_usage = self.cpu_monitor.calculate_usage();
        self.update_metric(MetricType::CpuUsage, cpu_usage, timestamp);
        
        // 更新内存使用率
        let memory_usage = self.memory_monitor.get_heap_usage_percent();
        self.update_metric(MetricType::MemoryUsage, memory_usage, timestamp);
        
        // 更新栈使用率
        let stack_usage = self.memory_monitor.get_stack_usage_percent();
        self.update_metric(MetricType::StackUsage, stack_usage, timestamp);
        
        // 更新中断延迟
        let interrupt_latency = self.interrupt_monitor.get_average_latency_us(self.cpu_frequency);
        self.update_metric(MetricType::InterruptLatency, interrupt_latency, timestamp);
        
        // 更新功耗
        let power_consumption = self.power_monitor.get_average_power();
        self.update_metric(MetricType::PowerConsumption, power_consumption, timestamp);
        
        self.last_update = timestamp;
    }
    
    fn update_metric(&mut self, metric_type: MetricType, value: f32, timestamp: u64) {
        if let Some(metric) = self.metrics.get_mut(&metric_type) {
            metric.update(value, timestamp);
        } else {
            let mut metric = MetricData::new(metric_type, value);
            metric.timestamp = timestamp;
            self.metrics.insert(metric_type, metric).ok();
        }
    }
    
    fn get_metric(&self, metric_type: MetricType) -> Option<&MetricData> {
        self.metrics.get(&metric_type)
    }
    
    fn get_system_health_score(&self) -> f32 {
        let mut score = 100.0;
        let mut factors = 0;
        
        // CPU使用率影响
        if let Some(cpu_metric) = self.get_metric(MetricType::CpuUsage) {
            if cpu_metric.value > 90.0 {
                score -= 30.0;
            } else if cpu_metric.value > 70.0 {
                score -= 15.0;
            }
            factors += 1;
        }
        
        // 内存使用率影响
        if let Some(memory_metric) = self.get_metric(MetricType::MemoryUsage) {
            if memory_metric.value > 90.0 {
                score -= 25.0;
            } else if memory_metric.value > 75.0 {
                score -= 10.0;
            }
            factors += 1;
        }
        
        // 中断延迟影响
        if let Some(latency_metric) = self.get_metric(MetricType::InterruptLatency) {
            if latency_metric.value > 100.0 { // 100us
                score -= 20.0;
            } else if latency_metric.value > 50.0 {
                score -= 10.0;
            }
            factors += 1;
        }
        
        if factors == 0 {
            0.0
        } else {
            score.max(0.0)
        }
    }
    
    fn detect_performance_issues(&self) -> Vec<&'static str, 8> {
        let mut issues = Vec::new();
        
        if let Some(cpu_metric) = self.get_metric(MetricType::CpuUsage) {
            if cpu_metric.value > 95.0 {
                issues.push("Critical CPU usage").ok();
            } else if cpu_metric.value > 85.0 {
                issues.push("High CPU usage").ok();
            }
        }
        
        if let Some(memory_metric) = self.get_metric(MetricType::MemoryUsage) {
            if memory_metric.value > 95.0 {
                issues.push("Critical memory usage").ok();
            } else if memory_metric.value > 85.0 {
                issues.push("High memory usage").ok();
            }
        }
        
        if let Some(stack_metric) = self.get_metric(MetricType::StackUsage) {
            if stack_metric.value > 90.0 {
                issues.push("Stack overflow risk").ok();
            }
        }
        
        if let Some(latency_metric) = self.get_metric(MetricType::InterruptLatency) {
            if latency_metric.value > 200.0 {
                issues.push("High interrupt latency").ok();
            }
        }
        
        issues
    }
    
    fn generate_performance_report(&self) -> PerformanceReport {
        PerformanceReport {
            timestamp: self.last_update,
            cpu_usage: self.get_metric(MetricType::CpuUsage).map(|m| m.value).unwrap_or(0.0),
            memory_usage: self.get_metric(MetricType::MemoryUsage).map(|m| m.value).unwrap_or(0.0),
            stack_usage: self.get_metric(MetricType::StackUsage).map(|m| m.value).unwrap_or(0.0),
            interrupt_latency: self.get_metric(MetricType::InterruptLatency).map(|m| m.value).unwrap_or(0.0),
            power_consumption: self.get_metric(MetricType::PowerConsumption).map(|m| m.value).unwrap_or(0.0),
            health_score: self.get_system_health_score(),
            peak_cpu_usage: self.cpu_monitor.get_peak_usage(),
            average_cpu_usage: self.cpu_monitor.get_average_usage(),
            memory_fragmentation: self.memory_monitor.get_fragmentation_ratio(),
            total_energy: self.power_monitor.get_total_energy(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PerformanceReport {
    pub timestamp: u64,
    pub cpu_usage: f32,
    pub memory_usage: f32,
    pub stack_usage: f32,
    pub interrupt_latency: f32,
    pub power_consumption: f32,
    pub health_score: f32,
    pub peak_cpu_usage: f32,
    pub average_cpu_usage: f32,
    pub memory_fragmentation: f32,
    pub total_energy: f32,
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, USART2, USART3])]
mod app {
    use super::*;
    
    #[shared]
    struct Shared {
        performance_monitor: SystemPerformanceMonitor,
        led: Pin<'A', 5, Output<PushPull>>,
    }
    
    #[local]
    struct Local {
        adc: Adc<pac::ADC1>,
        timer: Timer<pac::TIM2>,
    }
    
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;
        let cp = ctx.core;
        
        // 配置系统时钟
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();
        
        // 启用DWT用于性能测试
        let mut dwt = cp.DWT;
        dwt.enable_cycle_counter();
        
        // 配置GPIO
        let gpioa = dp.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();
        
        // 配置ADC用于功耗监控
        let adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());
        
        // 配置定时器
        let timer = Timer::new(dp.TIM2, &clocks).counter_hz();
        
        // 初始化系统时钟
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cp.SYST, 168_000_000, systick_token);
        
        // 创建性能监控器
        let performance_monitor = SystemPerformanceMonitor::new(168_000_000, 1000); // 1秒更新间隔
        
        // 启动监控任务
        monitor_system::spawn().ok();
        update_display::spawn().ok();
        
        cortex_m_log::println!("Performance monitor initialized");
        
        (
            Shared {
                performance_monitor,
                led,
            },
            Local {
                adc,
                timer,
            },
        )
    }
    
    #[idle]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            // 记录空闲时间用于CPU使用率计算
            ctx.shared.performance_monitor.lock(|monitor| {
                monitor.cpu_monitor.start_idle();
            });
            
            cortex_m::asm::wfi();
            
            ctx.shared.performance_monitor.lock(|monitor| {
                monitor.cpu_monitor.end_idle();
            });
        }
    }
    
    #[task(shared = [performance_monitor], priority = 1)]
    async fn monitor_system(mut ctx: monitor_system::Context) {
        loop {
            let timestamp = Systick::now().duration_since_epoch().to_millis();
            
            ctx.shared.performance_monitor.lock(|monitor| {
                monitor.cpu_monitor.update_total_cycles();
                monitor.update_metrics(timestamp);
            });
            
            // 检查性能问题
            let issues = ctx.shared.performance_monitor.lock(|monitor| {
                monitor.detect_performance_issues()
            });
            
            if !issues.is_empty() {
                cortex_m_log::println!("Performance issues detected:");
                for issue in &issues {
                    cortex_m_log::println!("  - {}", issue);
                }
            }
            
            // 每秒监控一次
            Systick::delay(1000.millis()).await;
        }
    }
    
    #[task(shared = [performance_monitor, led], priority = 1)]
    async fn update_display(mut ctx: update_display::Context) {
        loop {
            let report = ctx.shared.performance_monitor.lock(|monitor| {
                monitor.generate_performance_report()
            });
            
            // 根据系统健康状态控制LED
            ctx.shared.led.lock(|led| {
                if report.health_score > 80.0 {
                    led.set_low(); // 绿色：系统健康
                } else if report.health_score > 60.0 {
                    // 黄色：警告（通过闪烁实现）
                    led.toggle();
                } else {
                    led.set_high(); // 红色：严重问题
                }
            });
            
            // 打印性能报告
            cortex_m_log::println!("\n=== Performance Report ===");
            cortex_m_log::println!("Health Score: {:.1}%", report.health_score);
            cortex_m_log::println!("CPU Usage: {:.1}% (Peak: {:.1}%, Avg: {:.1}%)", 
                report.cpu_usage, report.peak_cpu_usage, report.average_cpu_usage);
            cortex_m_log::println!("Memory Usage: {:.1}% (Fragmentation: {:.1}%)", 
                report.memory_usage, report.memory_fragmentation);
            cortex_m_log::println!("Stack Usage: {:.1}%", report.stack_usage);
            cortex_m_log::println!("Interrupt Latency: {:.1} μs", report.interrupt_latency);
            cortex_m_log::println!("Power Consumption: {:.2} W", report.power_consumption);
            cortex_m_log::println!("Total Energy: {:.3} Wh", report.total_energy);
            
            // 每5秒更新一次显示
            Systick::delay(5000.millis()).await;
        }
    }
    
    #[task(shared = [performance_monitor], priority = 2)]
    async fn high_priority_task(mut ctx: high_priority_task::Context) {
        const TASK_ID: u8 = 1;
        
        ctx.shared.performance_monitor.lock(|monitor| {
            monitor.task_monitor.start_task(TASK_ID);
        });
        
        // 模拟高优先级任务工作
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
        
        ctx.shared.performance_monitor.lock(|monitor| {
            monitor.task_monitor.end_task(TASK_ID);
        });
    }
    
    #[task(shared = [performance_monitor], priority = 1)]
    async fn medium_priority_task(mut ctx: medium_priority_task::Context) {
        const TASK_ID: u8 = 2;
        
        ctx.shared.performance_monitor.lock(|monitor| {
            monitor.task_monitor.start_task(TASK_ID);
        });
        
        // 模拟中等优先级任务工作
        Systick::delay(10.millis()).await;
        
        ctx.shared.performance_monitor.lock(|monitor| {
            monitor.task_monitor.end_task(TASK_ID);
        });
    }
    
    #[task(shared = [performance_monitor], local = [adc], priority = 3)]
    async fn power_measurement_task(mut ctx: power_measurement_task::Context) {
        loop {
            let timestamp = Systick::now().duration_since_epoch().to_millis();
            
            // 模拟电压和电流测量
            let voltage = 3.3; // V
            let current = 0.1; // A (模拟值)
            
            ctx.shared.performance_monitor.lock(|monitor| {
                monitor.power_monitor.add_measurement(voltage, current, timestamp);
            });
            
            // 每100ms测量一次功耗
            Systick::delay(100.millis()).await;
        }
    }
    
    // 中断处理程序示例
    #[task(binds = EXTI0, shared = [performance_monitor], priority = 4)]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        ctx.shared.performance_monitor.lock(|monitor| {
            monitor.interrupt_monitor.start_interrupt();
        });
        
        // 处理按钮中断
        // ...
        
        ctx.shared.performance_monitor.lock(|monitor| {
            monitor.interrupt_monitor.end_interrupt();
        });
        
        // 触发任务执行
        high_priority_task::spawn().ok();
        medium_priority_task::spawn().ok();
    }
}