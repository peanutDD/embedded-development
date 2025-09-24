#![no_std]

//! # 高精度定时器库
//! 
//! 这个库提供了微秒级精度的定时器实现，支持精确的时间测量、
//! 频率生成和定时分析功能。

use core::sync::atomic::{AtomicU32, AtomicU64, AtomicBool, Ordering};
use heapless::{Vec, spsc::{Queue, Producer, Consumer}};

/// 高精度定时器错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PrecisionTimerError {
    /// 定时器未初始化
    NotInitialized,
    /// 定时器已经在运行
    AlreadyRunning,
    /// 定时器未运行
    NotRunning,
    /// 无效的定时器配置
    InvalidConfiguration,
    /// 队列已满
    QueueFull,
    /// 队列为空
    QueueEmpty,
    /// 超出范围
    OutOfRange,
    /// 硬件错误
    HardwareError,
}

/// 定时器精度级别
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimerPrecision {
    /// 微秒级精度 (1μs)
    Microsecond,
    /// 纳秒级精度 (100ns)
    Nanosecond,
    /// 系统时钟精度
    SystemClock,
    /// 自定义精度
    Custom(u32),
}

/// 定时器模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimerMode {
    /// 单次定时
    OneShot,
    /// 周期性定时
    Periodic,
    /// PWM模式
    Pwm,
    /// 输入捕获模式
    InputCapture,
}

/// 时间测量类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MeasurementType {
    /// 执行时间测量
    ExecutionTime,
    /// 间隔时间测量
    IntervalTime,
    /// 频率测量
    Frequency,
    /// 占空比测量
    DutyCycle,
}

/// 定时器配置
#[derive(Debug, Clone, Copy)]
pub struct TimerConfig {
    /// 定时器精度
    pub precision: TimerPrecision,
    /// 定时器模式
    pub mode: TimerMode,
    /// 预分频值
    pub prescaler: u32,
    /// 自动重载值
    pub auto_reload: u32,
    /// 是否启用中断
    pub interrupt_enabled: bool,
    /// 是否启用DMA
    pub dma_enabled: bool,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            precision: TimerPrecision::Microsecond,
            mode: TimerMode::OneShot,
            prescaler: 84,  // 对于84MHz系统时钟，得到1MHz定时器时钟
            auto_reload: 1000,
            interrupt_enabled: true,
            dma_enabled: false,
        }
    }
}

/// 时间戳结构
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timestamp {
    /// 微秒部分
    pub microseconds: u64,
    /// 纳秒部分 (0-999)
    pub nanoseconds: u16,
}

impl Timestamp {
    /// 创建新的时间戳
    pub fn new(microseconds: u64, nanoseconds: u16) -> Self {
        Self {
            microseconds,
            nanoseconds: nanoseconds % 1000,
        }
    }
    
    /// 从微秒创建时间戳
    pub fn from_microseconds(microseconds: u64) -> Self {
        Self {
            microseconds,
            nanoseconds: 0,
        }
    }
    
    /// 从纳秒创建时间戳
    pub fn from_nanoseconds(nanoseconds: u64) -> Self {
        Self {
            microseconds: nanoseconds / 1000,
            nanoseconds: (nanoseconds % 1000) as u16,
        }
    }
    
    /// 转换为总纳秒数
    pub fn to_nanoseconds(&self) -> u64 {
        self.microseconds * 1000 + self.nanoseconds as u64
    }
    
    /// 转换为总微秒数
    pub fn to_microseconds(&self) -> u64 {
        self.microseconds + (self.nanoseconds as u64 + 500) / 1000
    }
    
    /// 计算时间差
    pub fn duration_since(&self, earlier: Timestamp) -> Option<Timestamp> {
        let self_ns = self.to_nanoseconds();
        let earlier_ns = earlier.to_nanoseconds();
        
        if self_ns >= earlier_ns {
            Some(Timestamp::from_nanoseconds(self_ns - earlier_ns))
        } else {
            None
        }
    }
}

/// 时间测量结果
#[derive(Debug, Clone, Copy)]
pub struct MeasurementResult {
    /// 测量类型
    pub measurement_type: MeasurementType,
    /// 开始时间戳
    pub start_time: Timestamp,
    /// 结束时间戳
    pub end_time: Timestamp,
    /// 测量值 (根据类型不同含义不同)
    pub value: f32,
    /// 测量精度 (纳秒)
    pub precision_ns: u32,
    /// 是否有效
    pub valid: bool,
}

impl MeasurementResult {
    /// 获取持续时间
    pub fn duration(&self) -> Option<Timestamp> {
        self.end_time.duration_since(self.start_time)
    }
    
    /// 获取持续时间 (微秒)
    pub fn duration_microseconds(&self) -> Option<u64> {
        self.duration().map(|d| d.to_microseconds())
    }
    
    /// 获取持续时间 (纳秒)
    pub fn duration_nanoseconds(&self) -> Option<u64> {
        self.duration().map(|d| d.to_nanoseconds())
    }
}

/// 频率生成器配置
#[derive(Debug, Clone, Copy)]
pub struct FrequencyConfig {
    /// 目标频率 (Hz)
    pub frequency_hz: f32,
    /// 占空比 (0.0-1.0)
    pub duty_cycle: f32,
    /// 相位偏移 (度)
    pub phase_offset: f32,
    /// 波形类型
    pub waveform: WaveformType,
}

/// 波形类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WaveformType {
    /// 方波
    Square,
    /// 正弦波
    Sine,
    /// 三角波
    Triangle,
    /// 锯齿波
    Sawtooth,
}

/// 定时分析结果
#[derive(Debug, Clone)]
pub struct TimingAnalysis {
    /// 样本数量
    pub sample_count: u32,
    /// 最小值 (纳秒)
    pub min_ns: u64,
    /// 最大值 (纳秒)
    pub max_ns: u64,
    /// 平均值 (纳秒)
    pub avg_ns: f32,
    /// 标准差 (纳秒)
    pub std_dev_ns: f32,
    /// 抖动 (纳秒)
    pub jitter_ns: u64,
    /// 直方图数据
    pub histogram: Vec<u32, 32>,
}

impl TimingAnalysis {
    /// 创建新的分析结果
    pub fn new() -> Self {
        Self {
            sample_count: 0,
            min_ns: u64::MAX,
            max_ns: 0,
            avg_ns: 0.0,
            std_dev_ns: 0.0,
            jitter_ns: 0,
            histogram: Vec::new(),
        }
    }
    
    /// 添加样本
    pub fn add_sample(&mut self, value_ns: u64) {
        self.sample_count += 1;
        
        if value_ns < self.min_ns {
            self.min_ns = value_ns;
        }
        
        if value_ns > self.max_ns {
            self.max_ns = value_ns;
        }
        
        // 更新平均值 (增量计算)
        let delta = value_ns as f32 - self.avg_ns;
        self.avg_ns += delta / self.sample_count as f32;
        
        // 更新抖动
        self.jitter_ns = self.max_ns - self.min_ns;
        
        // 更新直方图
        self.update_histogram(value_ns);
    }
    
    fn update_histogram(&mut self, value_ns: u64) {
        if self.sample_count == 1 {
            // 初始化直方图
            for _ in 0..32 {
                self.histogram.push(0).ok();
            }
        }
        
        if self.histogram.len() == 32 && self.max_ns > self.min_ns {
            let range = self.max_ns - self.min_ns;
            let bin_size = range / 32;
            
            if bin_size > 0 {
                let bin_index = ((value_ns - self.min_ns) / bin_size).min(31) as usize;
                if let Some(bin) = self.histogram.get_mut(bin_index) {
                    *bin += 1;
                }
            }
        }
    }
    
    /// 计算标准差
    pub fn calculate_std_dev(&mut self, samples: &[u64]) {
        if samples.len() < 2 {
            return;
        }
        
        let mut sum_sq_diff = 0.0f32;
        for &sample in samples {
            let diff = sample as f32 - self.avg_ns;
            sum_sq_diff += diff * diff;
        }
        
        self.std_dev_ns = (sum_sq_diff / (samples.len() - 1) as f32).sqrt();
    }
}

/// 高精度定时器
pub struct PrecisionTimer {
    /// 定时器配置
    config: TimerConfig,
    /// 是否已初始化
    initialized: AtomicBool,
    /// 是否正在运行
    running: AtomicBool,
    /// 系统时钟频率
    system_clock_hz: u32,
    /// 定时器时钟频率
    timer_clock_hz: u32,
    /// 当前计数值
    current_count: AtomicU32,
    /// 溢出计数
    overflow_count: AtomicU32,
    /// 基准时间戳
    base_timestamp: AtomicU64,
}

impl PrecisionTimer {
    /// 创建新的高精度定时器
    pub fn new(config: TimerConfig, system_clock_hz: u32) -> Self {
        let timer_clock_hz = system_clock_hz / (config.prescaler + 1);
        
        Self {
            config,
            initialized: AtomicBool::new(false),
            running: AtomicBool::new(false),
            system_clock_hz,
            timer_clock_hz,
            current_count: AtomicU32::new(0),
            overflow_count: AtomicU32::new(0),
            base_timestamp: AtomicU64::new(0),
        }
    }
    
    /// 初始化定时器
    pub fn init(&self) -> Result<(), PrecisionTimerError> {
        if self.initialized.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::AlreadyRunning);
        }
        
        // 这里应该包含硬件初始化代码
        // 由于是抽象实现，我们只设置标志
        
        self.initialized.store(true, Ordering::Relaxed);
        Ok(())
    }
    
    /// 启动定时器
    pub fn start(&self) -> Result<(), PrecisionTimerError> {
        if !self.initialized.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::NotInitialized);
        }
        
        if self.running.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::AlreadyRunning);
        }
        
        self.running.store(true, Ordering::Relaxed);
        self.base_timestamp.store(self.get_system_time_ns(), Ordering::Relaxed);
        
        Ok(())
    }
    
    /// 停止定时器
    pub fn stop(&self) -> Result<(), PrecisionTimerError> {
        if !self.running.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::NotRunning);
        }
        
        self.running.store(false, Ordering::Relaxed);
        Ok(())
    }
    
    /// 获取当前时间戳
    pub fn get_timestamp(&self) -> Timestamp {
        if !self.running.load(Ordering::Relaxed) {
            return Timestamp::new(0, 0);
        }
        
        let current_ns = self.get_system_time_ns();
        let base_ns = self.base_timestamp.load(Ordering::Relaxed);
        let elapsed_ns = current_ns.wrapping_sub(base_ns);
        
        Timestamp::from_nanoseconds(elapsed_ns)
    }
    
    /// 获取定时器精度 (纳秒)
    pub fn get_precision_ns(&self) -> u32 {
        match self.config.precision {
            TimerPrecision::Microsecond => 1000,
            TimerPrecision::Nanosecond => 100,
            TimerPrecision::SystemClock => 1_000_000_000 / self.system_clock_hz,
            TimerPrecision::Custom(ns) => ns,
        }
    }
    
    /// 延时 (微秒)
    pub fn delay_us(&self, microseconds: u32) -> Result<(), PrecisionTimerError> {
        if !self.running.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::NotRunning);
        }
        
        let start = self.get_timestamp();
        let target_us = microseconds as u64;
        
        loop {
            let current = self.get_timestamp();
            if let Some(elapsed) = current.duration_since(start) {
                if elapsed.to_microseconds() >= target_us {
                    break;
                }
            }
            
            // 让出CPU时间
            core::hint::spin_loop();
        }
        
        Ok(())
    }
    
    /// 延时 (纳秒)
    pub fn delay_ns(&self, nanoseconds: u32) -> Result<(), PrecisionTimerError> {
        if !self.running.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::NotRunning);
        }
        
        let start = self.get_timestamp();
        let target_ns = nanoseconds as u64;
        
        loop {
            let current = self.get_timestamp();
            if let Some(elapsed) = current.duration_since(start) {
                if elapsed.to_nanoseconds() >= target_ns {
                    break;
                }
            }
            
            core::hint::spin_loop();
        }
        
        Ok(())
    }
    
    /// 获取系统时间 (纳秒)
    fn get_system_time_ns(&self) -> u64 {
        // 这里应该读取硬件计数器
        // 由于是抽象实现，我们使用模拟值
        let count = self.current_count.load(Ordering::Relaxed) as u64;
        let overflow = self.overflow_count.load(Ordering::Relaxed) as u64;
        
        let total_ticks = overflow * (self.config.auto_reload as u64 + 1) + count;
        let ns_per_tick = 1_000_000_000u64 / self.timer_clock_hz as u64;
        
        total_ticks * ns_per_tick
    }
    
    /// 处理定时器溢出
    pub fn handle_overflow(&self) {
        self.overflow_count.fetch_add(1, Ordering::Relaxed);
    }
    
    /// 更新计数值
    pub fn update_count(&self, count: u32) {
        self.current_count.store(count, Ordering::Relaxed);
    }
}

/// 时间测量器
pub struct TimeMeasurement {
    /// 高精度定时器
    timer: PrecisionTimer,
    /// 测量结果队列
    results: Queue<MeasurementResult, 64>,
    /// 当前测量
    current_measurement: Option<(MeasurementType, Timestamp)>,
}

impl TimeMeasurement {
    /// 创建新的时间测量器
    pub fn new(timer: PrecisionTimer) -> Self {
        Self {
            timer,
            results: Queue::new(),
            current_measurement: None,
        }
    }
    
    /// 开始测量
    pub fn start_measurement(&mut self, measurement_type: MeasurementType) -> Result<(), PrecisionTimerError> {
        if self.current_measurement.is_some() {
            return Err(PrecisionTimerError::AlreadyRunning);
        }
        
        let start_time = self.timer.get_timestamp();
        self.current_measurement = Some((measurement_type, start_time));
        
        Ok(())
    }
    
    /// 结束测量
    pub fn end_measurement(&mut self) -> Result<MeasurementResult, PrecisionTimerError> {
        let (measurement_type, start_time) = self.current_measurement
            .take()
            .ok_or(PrecisionTimerError::NotRunning)?;
        
        let end_time = self.timer.get_timestamp();
        
        let value = match measurement_type {
            MeasurementType::ExecutionTime => {
                if let Some(duration) = end_time.duration_since(start_time) {
                    duration.to_microseconds() as f32
                } else {
                    0.0
                }
            }
            MeasurementType::IntervalTime => {
                if let Some(duration) = end_time.duration_since(start_time) {
                    duration.to_microseconds() as f32
                } else {
                    0.0
                }
            }
            MeasurementType::Frequency => {
                if let Some(duration) = end_time.duration_since(start_time) {
                    let period_us = duration.to_microseconds();
                    if period_us > 0 {
                        1_000_000.0 / period_us as f32
                    } else {
                        0.0
                    }
                } else {
                    0.0
                }
            }
            MeasurementType::DutyCycle => {
                // 这里需要额外的逻辑来计算占空比
                50.0 // 默认值
            }
        };
        
        let result = MeasurementResult {
            measurement_type,
            start_time,
            end_time,
            value,
            precision_ns: self.timer.get_precision_ns(),
            valid: true,
        };
        
        // 尝试将结果加入队列
        if self.results.enqueue(result).is_err() {
            return Err(PrecisionTimerError::QueueFull);
        }
        
        Ok(result)
    }
    
    /// 获取测量结果
    pub fn get_result(&mut self) -> Option<MeasurementResult> {
        self.results.dequeue()
    }
    
    /// 获取所有测量结果
    pub fn get_all_results(&mut self) -> Vec<MeasurementResult, 64> {
        let mut results = Vec::new();
        
        while let Some(result) = self.results.dequeue() {
            results.push(result).ok();
        }
        
        results
    }
}

/// 频率生成器
pub struct FrequencyGenerator {
    /// 高精度定时器
    timer: PrecisionTimer,
    /// 频率配置
    config: FrequencyConfig,
    /// 当前相位
    current_phase: AtomicU32,
    /// 是否正在生成
    generating: AtomicBool,
}

impl FrequencyGenerator {
    /// 创建新的频率生成器
    pub fn new(timer: PrecisionTimer, config: FrequencyConfig) -> Self {
        Self {
            timer,
            config,
            current_phase: AtomicU32::new(0),
            generating: AtomicBool::new(false),
        }
    }
    
    /// 开始生成频率
    pub fn start_generation(&self) -> Result<(), PrecisionTimerError> {
        if self.generating.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::AlreadyRunning);
        }
        
        self.timer.start()?;
        self.generating.store(true, Ordering::Relaxed);
        self.current_phase.store(0, Ordering::Relaxed);
        
        Ok(())
    }
    
    /// 停止生成频率
    pub fn stop_generation(&self) -> Result<(), PrecisionTimerError> {
        if !self.generating.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::NotRunning);
        }
        
        self.generating.store(false, Ordering::Relaxed);
        self.timer.stop()
    }
    
    /// 获取当前输出值
    pub fn get_output_value(&self) -> f32 {
        if !self.generating.load(Ordering::Relaxed) {
            return 0.0;
        }
        
        let phase = self.current_phase.load(Ordering::Relaxed) as f32;
        let phase_rad = (phase + self.config.phase_offset) * core::f32::consts::PI / 180.0;
        
        match self.config.waveform {
            WaveformType::Square => {
                if (phase_rad.sin() >= 0.0) {
                    1.0
                } else {
                    0.0
                }
            }
            WaveformType::Sine => {
                (phase_rad.sin() + 1.0) / 2.0
            }
            WaveformType::Triangle => {
                let normalized_phase = (phase % 360.0) / 360.0;
                if normalized_phase < 0.5 {
                    normalized_phase * 2.0
                } else {
                    2.0 - normalized_phase * 2.0
                }
            }
            WaveformType::Sawtooth => {
                (phase % 360.0) / 360.0
            }
        }
    }
    
    /// 更新相位
    pub fn update_phase(&self) {
        if !self.generating.load(Ordering::Relaxed) {
            return;
        }
        
        let timestamp = self.timer.get_timestamp();
        let time_s = timestamp.to_microseconds() as f32 / 1_000_000.0;
        let new_phase = (time_s * self.config.frequency_hz * 360.0) % 360.0;
        
        self.current_phase.store(new_phase as u32, Ordering::Relaxed);
    }
}

/// 定时分析器
pub struct TimingAnalyzer {
    /// 分析结果
    analysis: TimingAnalysis,
    /// 样本缓冲区
    samples: Vec<u64, 1024>,
    /// 是否正在分析
    analyzing: AtomicBool,
}

impl TimingAnalyzer {
    /// 创建新的定时分析器
    pub fn new() -> Self {
        Self {
            analysis: TimingAnalysis::new(),
            samples: Vec::new(),
            analyzing: AtomicBool::new(false),
        }
    }
    
    /// 开始分析
    pub fn start_analysis(&mut self) {
        self.analyzing.store(true, Ordering::Relaxed);
        self.analysis = TimingAnalysis::new();
        self.samples.clear();
    }
    
    /// 停止分析
    pub fn stop_analysis(&mut self) -> TimingAnalysis {
        self.analyzing.store(false, Ordering::Relaxed);
        
        // 计算标准差
        let samples_slice = self.samples.as_slice();
        self.analysis.calculate_std_dev(samples_slice);
        
        self.analysis.clone()
    }
    
    /// 添加样本
    pub fn add_sample(&mut self, value_ns: u64) -> Result<(), PrecisionTimerError> {
        if !self.analyzing.load(Ordering::Relaxed) {
            return Err(PrecisionTimerError::NotRunning);
        }
        
        self.analysis.add_sample(value_ns);
        
        if self.samples.push(value_ns).is_err() {
            return Err(PrecisionTimerError::QueueFull);
        }
        
        Ok(())
    }
    
    /// 获取当前分析结果
    pub fn get_analysis(&self) -> &TimingAnalysis {
        &self.analysis
    }
}

/// 全局系统时间 (纳秒)
static GLOBAL_SYSTEM_TIME_NS: AtomicU64 = AtomicU64::new(0);

/// 更新全局系统时间
pub fn update_global_time(time_ns: u64) {
    GLOBAL_SYSTEM_TIME_NS.store(time_ns, Ordering::Relaxed);
}

/// 获取全局系统时间
pub fn get_global_time() -> u64 {
    GLOBAL_SYSTEM_TIME_NS.load(Ordering::Relaxed)
}

/// 高精度延时 (微秒)
pub fn delay_us_precise(microseconds: u32) {
    let start_time = get_global_time();
    let target_ns = microseconds as u64 * 1000;
    
    loop {
        let current_time = get_global_time();
        if current_time.wrapping_sub(start_time) >= target_ns {
            break;
        }
        core::hint::spin_loop();
    }
}

/// 高精度延时 (纳秒)
pub fn delay_ns_precise(nanoseconds: u32) {
    let start_time = get_global_time();
    let target_ns = nanoseconds as u64;
    
    loop {
        let current_time = get_global_time();
        if current_time.wrapping_sub(start_time) >= target_ns {
            break;
        }
        core::hint::spin_loop();
    }
}