#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::asm;

// 平台特定的HAL导入
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;

use hal::{
    prelude::*,
    timer::{Timer, Event},
    gpio::{Input, Output, PushPull, PullUp},
    adc::{Adc, config::AdcConfig},
};

use watchdog_timer::{
    SystemMonitor, SystemHealth, WatchdogTimer, IndependentWatchdog, 
    WatchdogConfig, RecoveryStrategy, ResetReason
};
use heapless::{Vec, String, FnvIndexMap};
use nb::block;
use libm::{sqrtf, fabsf};

/// 监控配置
#[derive(Debug, Clone)]
struct MonitorConfig {
    check_interval_ms: u32,
    health_update_ms: u32,
    display_update_ms: u32,
    alert_thresholds: AlertThresholds,
    logging_enabled: bool,
}

/// 告警阈值
#[derive(Debug, Clone)]
struct AlertThresholds {
    cpu_usage_warning: f32,
    cpu_usage_critical: f32,
    memory_usage_warning: f32,
    memory_usage_critical: f32,
    temperature_warning: f32,
    temperature_critical: f32,
    voltage_low_warning: f32,
    voltage_low_critical: f32,
    voltage_high_warning: f32,
    voltage_high_critical: f32,
}

/// 系统指标
#[derive(Debug, Clone)]
struct SystemMetrics {
    cpu_usage: f32,
    memory_usage: f32,
    temperature: f32,
    voltage: f32,
    uptime_ms: u32,
    task_count: u32,
    error_count: u32,
    interrupt_count: u32,
    context_switches: u32,
}

/// 性能统计
#[derive(Debug, Clone)]
struct PerformanceStats {
    avg_cpu_usage: f32,
    peak_cpu_usage: f32,
    avg_memory_usage: f32,
    peak_memory_usage: f32,
    min_free_memory: u32,
    max_temperature: f32,
    min_voltage: f32,
    max_voltage: f32,
    total_errors: u32,
    uptime_hours: f32,
}

/// 告警级别
#[derive(Debug, Clone, Copy, PartialEq)]
enum AlertLevel {
    Info,
    Warning,
    Critical,
    Emergency,
}

/// 告警记录
#[derive(Debug, Clone)]
struct AlertRecord {
    level: AlertLevel,
    message: String<128>,
    timestamp: u32,
    metric_name: String<32>,
    metric_value: f32,
    threshold: f32,
}

/// 系统监控器
struct SystemMonitorController {
    monitor: SystemMonitor,
    config: MonitorConfig,
    metrics: SystemMetrics,
    stats: PerformanceStats,
    alerts: Vec<AlertRecord, 32>,
    last_health_update: u32,
    last_display_update: u32,
    system_time_ms: u32,
    measurement_count: u32,
    cpu_samples: Vec<f32, 100>,
    memory_samples: Vec<f32, 100>,
    temp_samples: Vec<f32, 50>,
    voltage_samples: Vec<f32, 50>,
}

impl Default for MonitorConfig {
    fn default() -> Self {
        Self {
            check_interval_ms: 100,
            health_update_ms: 1000,
            display_update_ms: 500,
            alert_thresholds: AlertThresholds::default(),
            logging_enabled: true,
        }
    }
}

impl Default for AlertThresholds {
    fn default() -> Self {
        Self {
            cpu_usage_warning: 70.0,
            cpu_usage_critical: 90.0,
            memory_usage_warning: 80.0,
            memory_usage_critical: 95.0,
            temperature_warning: 70.0,
            temperature_critical: 85.0,
            voltage_low_warning: 3.0,
            voltage_low_critical: 2.8,
            voltage_high_warning: 3.6,
            voltage_high_critical: 3.8,
        }
    }
}

impl Default for SystemMetrics {
    fn default() -> Self {
        Self {
            cpu_usage: 0.0,
            memory_usage: 0.0,
            temperature: 25.0,
            voltage: 3.3,
            uptime_ms: 0,
            task_count: 0,
            error_count: 0,
            interrupt_count: 0,
            context_switches: 0,
        }
    }
}

impl Default for PerformanceStats {
    fn default() -> Self {
        Self {
            avg_cpu_usage: 0.0,
            peak_cpu_usage: 0.0,
            avg_memory_usage: 0.0,
            peak_memory_usage: 0.0,
            min_free_memory: u32::MAX,
            max_temperature: 0.0,
            min_voltage: f32::INFINITY,
            max_voltage: 0.0,
            total_errors: 0,
            uptime_hours: 0.0,
        }
    }
}

impl SystemMonitorController {
    fn new(config: MonitorConfig) -> Self {
        let monitor = SystemMonitor::new(config.check_interval_ms);
        
        Self {
            monitor,
            config,
            metrics: SystemMetrics::default(),
            stats: PerformanceStats::default(),
            alerts: Vec::new(),
            last_health_update: 0,
            last_display_update: 0,
            system_time_ms: 0,
            measurement_count: 0,
            cpu_samples: Vec::new(),
            memory_samples: Vec::new(),
            temp_samples: Vec::new(),
            voltage_samples: Vec::new(),
        }
    }

    fn initialize(&mut self) {
        // 添加系统任务监控
        self.monitor.add_task(1, "main_loop", 1000, true).ok();
        self.monitor.add_task(2, "sensor_read", 500, true).ok();
        self.monitor.add_task(3, "data_process", 2000, false).ok();
        self.monitor.add_task(4, "communication", 3000, false).ok();
        
        self.monitor.enable_monitoring();
    }

    fn update_system_time(&mut self) {
        self.system_time_ms += 1;
        self.metrics.uptime_ms = self.system_time_ms;
    }

    fn collect_system_metrics(&mut self, adc_values: &[u16; 4]) {
        // 模拟CPU使用率计算
        let base_cpu = 20.0;
        let variation = (self.system_time_ms % 1000) as f32 * 0.05;
        let load_factor = if self.system_time_ms % 5000 < 1000 { 30.0 } else { 0.0 };
        self.metrics.cpu_usage = base_cpu + variation + load_factor;

        // 模拟内存使用率
        let base_memory = 45.0;
        let growth = (self.system_time_ms / 10000) as f32 * 0.1;
        self.metrics.memory_usage = base_memory + growth + (self.system_time_ms % 100) as f32 * 0.2;

        // 从ADC读取温度 (假设ADC通道0连接温度传感器)
        let temp_raw = adc_values[0];
        self.metrics.temperature = 25.0 + (temp_raw as f32 - 2048.0) * 0.01;

        // 从ADC读取电压 (假设ADC通道1连接电压分压器)
        let voltage_raw = adc_values[1];
        self.metrics.voltage = (voltage_raw as f32 / 4096.0) * 3.3 * 2.0; // 假设2:1分压

        // 更新其他指标
        self.metrics.task_count = 4; // 固定任务数
        self.metrics.interrupt_count += (self.system_time_ms % 10) as u32;
        self.metrics.context_switches += (self.system_time_ms % 7) as u32;

        // 添加样本到历史数据
        self.add_samples();
        
        // 更新统计信息
        self.update_statistics();
    }

    fn add_samples(&mut self) {
        // 添加CPU使用率样本
        if self.cpu_samples.len() >= 100 {
            self.cpu_samples.remove(0);
        }
        self.cpu_samples.push(self.metrics.cpu_usage).ok();

        // 添加内存使用率样本
        if self.memory_samples.len() >= 100 {
            self.memory_samples.remove(0);
        }
        self.memory_samples.push(self.metrics.memory_usage).ok();

        // 添加温度样本
        if self.temp_samples.len() >= 50 {
            self.temp_samples.remove(0);
        }
        self.temp_samples.push(self.metrics.temperature).ok();

        // 添加电压样本
        if self.voltage_samples.len() >= 50 {
            self.voltage_samples.remove(0);
        }
        self.voltage_samples.push(self.metrics.voltage).ok();
    }

    fn update_statistics(&mut self) {
        self.measurement_count += 1;

        // 计算CPU统计
        if !self.cpu_samples.is_empty() {
            let sum: f32 = self.cpu_samples.iter().sum();
            self.stats.avg_cpu_usage = sum / self.cpu_samples.len() as f32;
            self.stats.peak_cpu_usage = self.cpu_samples.iter().fold(0.0f32, |a, &b| a.max(b));
        }

        // 计算内存统计
        if !self.memory_samples.is_empty() {
            let sum: f32 = self.memory_samples.iter().sum();
            self.stats.avg_memory_usage = sum / self.memory_samples.len() as f32;
            self.stats.peak_memory_usage = self.memory_samples.iter().fold(0.0f32, |a, &b| a.max(b));
        }

        // 更新温度统计
        if !self.temp_samples.is_empty() {
            self.stats.max_temperature = self.temp_samples.iter().fold(0.0f32, |a, &b| a.max(*b));
        }

        // 更新电压统计
        if !self.voltage_samples.is_empty() {
            self.stats.min_voltage = self.voltage_samples.iter().fold(f32::INFINITY, |a, &b| a.min(*b));
            self.stats.max_voltage = self.voltage_samples.iter().fold(0.0f32, |a, &b| a.max(*b));
        }

        // 更新运行时间
        self.stats.uptime_hours = self.metrics.uptime_ms as f32 / 3600000.0;
        self.stats.total_errors = self.metrics.error_count;
    }

    fn check_alerts(&mut self) {
        let thresholds = &self.config.alert_thresholds;

        // 检查CPU使用率
        self.check_metric_alert(
            "CPU Usage",
            self.metrics.cpu_usage,
            thresholds.cpu_usage_warning,
            thresholds.cpu_usage_critical,
        );

        // 检查内存使用率
        self.check_metric_alert(
            "Memory Usage",
            self.metrics.memory_usage,
            thresholds.memory_usage_warning,
            thresholds.memory_usage_critical,
        );

        // 检查温度
        self.check_metric_alert(
            "Temperature",
            self.metrics.temperature,
            thresholds.temperature_warning,
            thresholds.temperature_critical,
        );

        // 检查电压（低电压）
        if self.metrics.voltage < thresholds.voltage_low_critical {
            self.add_alert(AlertLevel::Critical, "Voltage", self.metrics.voltage, thresholds.voltage_low_critical);
        } else if self.metrics.voltage < thresholds.voltage_low_warning {
            self.add_alert(AlertLevel::Warning, "Voltage", self.metrics.voltage, thresholds.voltage_low_warning);
        }

        // 检查电压（高电压）
        if self.metrics.voltage > thresholds.voltage_high_critical {
            self.add_alert(AlertLevel::Critical, "Voltage", self.metrics.voltage, thresholds.voltage_high_critical);
        } else if self.metrics.voltage > thresholds.voltage_high_warning {
            self.add_alert(AlertLevel::Warning, "Voltage", self.metrics.voltage, thresholds.voltage_high_warning);
        }
    }

    fn check_metric_alert(&mut self, metric_name: &str, value: f32, warning_threshold: f32, critical_threshold: f32) {
        if value > critical_threshold {
            self.add_alert(AlertLevel::Critical, metric_name, value, critical_threshold);
        } else if value > warning_threshold {
            self.add_alert(AlertLevel::Warning, metric_name, value, warning_threshold);
        }
    }

    fn add_alert(&mut self, level: AlertLevel, metric_name: &str, value: f32, threshold: f32) {
        let mut message = String::new();
        let mut name = String::new();
        
        name.push_str(metric_name).ok();
        
        match level {
            AlertLevel::Warning => message.push_str("WARNING: ").ok(),
            AlertLevel::Critical => message.push_str("CRITICAL: ").ok(),
            AlertLevel::Emergency => message.push_str("EMERGENCY: ").ok(),
            AlertLevel::Info => message.push_str("INFO: ").ok(),
        }
        
        message.push_str(metric_name).ok();
        message.push_str(" exceeded threshold").ok();

        let alert = AlertRecord {
            level,
            message,
            timestamp: self.system_time_ms,
            metric_name: name,
            metric_value: value,
            threshold,
        };

        // 保持告警历史大小
        if self.alerts.len() >= 32 {
            self.alerts.remove(0);
        }
        
        self.alerts.push(alert).ok();
    }

    fn update_display(&self) {
        // 在实际应用中，这里会更新LCD显示或发送到串口
        // 这里只是演示显示逻辑的结构
    }

    fn get_system_status(&self) -> String<256> {
        let mut status = String::new();
        
        status.push_str("SYS: ").ok();
        // 添加系统状态信息
        status.push_str(" | CPU: ").ok();
        // 添加CPU使用率
        status.push_str(" | MEM: ").ok();
        // 添加内存使用率
        status.push_str(" | TEMP: ").ok();
        // 添加温度
        status.push_str(" | VOLT: ").ok();
        // 添加电压
        
        status
    }

    fn calculate_system_health_score(&self) -> f32 {
        let mut score = 100.0;
        
        // CPU使用率影响 (权重: 25%)
        let cpu_penalty = if self.metrics.cpu_usage > 80.0 {
            (self.metrics.cpu_usage - 80.0) * 1.25
        } else {
            0.0
        };
        score -= cpu_penalty * 0.25;
        
        // 内存使用率影响 (权重: 25%)
        let memory_penalty = if self.metrics.memory_usage > 85.0 {
            (self.metrics.memory_usage - 85.0) * 2.0
        } else {
            0.0
        };
        score -= memory_penalty * 0.25;
        
        // 温度影响 (权重: 30%)
        let temp_penalty = if self.metrics.temperature > 60.0 {
            (self.metrics.temperature - 60.0) * 2.0
        } else {
            0.0
        };
        score -= temp_penalty * 0.30;
        
        // 电压影响 (权重: 20%)
        let voltage_penalty = if self.metrics.voltage < 3.0 || self.metrics.voltage > 3.6 {
            fabsf(self.metrics.voltage - 3.3) * 10.0
        } else {
            0.0
        };
        score -= voltage_penalty * 0.20;
        
        score.max(0.0).min(100.0)
    }

    fn get_trend_analysis(&self) -> (f32, f32, f32, f32) {
        // 计算各指标的趋势 (简化的线性趋势)
        let cpu_trend = self.calculate_trend(&self.cpu_samples);
        let memory_trend = self.calculate_trend(&self.memory_samples);
        let temp_trend = self.calculate_trend(&self.temp_samples);
        let voltage_trend = self.calculate_trend(&self.voltage_samples);
        
        (cpu_trend, memory_trend, temp_trend, voltage_trend)
    }

    fn calculate_trend(&self, samples: &Vec<f32, impl heapless::ArrayLength<Item = f32>>) -> f32 {
        if samples.len() < 2 {
            return 0.0;
        }
        
        let n = samples.len() as f32;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_x2 = 0.0;
        
        for (i, &value) in samples.iter().enumerate() {
            let x = i as f32;
            sum_x += x;
            sum_y += value;
            sum_xy += x * value;
            sum_x2 += x * x;
        }
        
        // 计算线性回归斜率
        let slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
        slope
    }
}

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 配置ADC用于系统监控
    let adc_pins = (
        gpioa.pa0.into_analog(),  // 温度传感器
        gpioa.pa1.into_analog(),  // 电压监控
        gpioa.pa2.into_analog(),  // 电流监控
        gpioa.pa3.into_analog(),  // 备用通道
    );
    
    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());

    // 配置状态LED
    let mut status_led = gpioc.pc13.into_push_pull_output();
    let mut warning_led = gpioc.pc14.into_push_pull_output();
    let mut critical_led = gpioc.pc15.into_push_pull_output();
    
    // 配置按钮
    let reset_button = gpioa.pa4.into_pull_up_input();
    let display_button = gpioa.pa5.into_pull_up_input();

    // 初始化系统监控器
    let config = MonitorConfig::default();
    let mut monitor_controller = SystemMonitorController::new(config);
    monitor_controller.initialize();

    // 主循环
    loop {
        // 更新系统时间
        monitor_controller.update_system_time();
        
        // 检查按钮
        if reset_button.is_low().unwrap() {
            // 重置统计信息
            monitor_controller.stats = PerformanceStats::default();
            monitor_controller.alerts.clear();
            monitor_controller.measurement_count = 0;
        }
        
        if display_button.is_low().unwrap() {
            // 切换显示模式或输出详细信息
        }

        // 定期收集系统指标
        if monitor_controller.system_time_ms % monitor_controller.config.health_update_ms == 0 {
            // 读取ADC值
            let adc_values = [
                adc.read(&adc_pins.0).unwrap_or(2048),
                adc.read(&adc_pins.1).unwrap_or(2048),
                adc.read(&adc_pins.2).unwrap_or(2048),
                adc.read(&adc_pins.3).unwrap_or(2048),
            ];
            
            monitor_controller.collect_system_metrics(&adc_values);
            monitor_controller.check_alerts();
        }

        // 更新任务监控
        monitor_controller.monitor.update_task(1, monitor_controller.system_time_ms).ok();
        if monitor_controller.system_time_ms % 500 == 0 {
            monitor_controller.monitor.update_task(2, monitor_controller.system_time_ms).ok();
        }
        if monitor_controller.system_time_ms % 2000 == 0 {
            monitor_controller.monitor.update_task(3, monitor_controller.system_time_ms).ok();
        }
        if monitor_controller.system_time_ms % 3000 == 0 {
            monitor_controller.monitor.update_task(4, monitor_controller.system_time_ms).ok();
        }

        // 检查任务超时
        let timed_out_tasks = monitor_controller.monitor.check_system(monitor_controller.system_time_ms);
        for task in timed_out_tasks {
            monitor_controller.metrics.error_count += 1;
            monitor_controller.add_alert(
                AlertLevel::Warning,
                "Task Timeout",
                task.task_id as f32,
                0.0
            );
        }

        // 更新LED状态
        update_monitoring_leds(
            &mut status_led,
            &mut warning_led,
            &mut critical_led,
            &monitor_controller
        );

        // 更新显示
        if monitor_controller.system_time_ms % monitor_controller.config.display_update_ms == 0 {
            monitor_controller.update_display();
        }

        // 短暂延时
        for _ in 0..1000 {
            asm::nop();
        }
    }
}

/// 更新监控LED状态
fn update_monitoring_leds(
    status_led: &mut hal::gpio::gpioc::PC13<Output<PushPull>>,
    warning_led: &mut hal::gpio::gpioc::PC14<Output<PushPull>>,
    critical_led: &mut hal::gpio::gpioc::PC15<Output<PushPull>>,
    controller: &SystemMonitorController,
) {
    let health_score = controller.calculate_system_health_score();
    
    // 状态LED：根据健康分数控制
    if health_score > 80.0 {
        status_led.set_high().ok(); // 常亮表示健康
    } else if health_score > 60.0 {
        // 慢速闪烁表示一般
        if (controller.system_time_ms / 1000) % 2 == 0 {
            status_led.set_high().ok();
        } else {
            status_led.set_low().ok();
        }
    } else {
        // 快速闪烁表示不健康
        if (controller.system_time_ms / 200) % 2 == 0 {
            status_led.set_high().ok();
        } else {
            status_led.set_low().ok();
        }
    }

    // 警告LED：有警告级别告警时亮起
    let has_warning = controller.alerts.iter().any(|a| a.level == AlertLevel::Warning);
    if has_warning {
        if (controller.system_time_ms / 500) % 2 == 0 {
            warning_led.set_high().ok();
        } else {
            warning_led.set_low().ok();
        }
    } else {
        warning_led.set_low().ok();
    }

    // 严重告警LED：有严重告警时亮起
    let has_critical = controller.alerts.iter().any(|a| 
        a.level == AlertLevel::Critical || a.level == AlertLevel::Emergency
    );
    if has_critical {
        if (controller.system_time_ms / 100) % 2 == 0 {
            critical_led.set_high().ok();
        } else {
            critical_led.set_low().ok();
        }
    } else {
        critical_led.set_low().ok();
    }
}