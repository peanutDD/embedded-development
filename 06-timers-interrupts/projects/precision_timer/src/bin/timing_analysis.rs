#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Event},
    gpio::{gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7}, Output, PushPull},
    gpio::{gpioc::PC13, Input, PullUp},
    interrupt,
};
use cortex_m::peripheral::{NVIC, DWT};
use precision_timer::{
    TimingAnalyzer, TimingAnalysis, MeasurementResult,
    update_global_time, get_global_time
};
use heapless::spsc::{Queue, Producer, Consumer};
use core::sync::atomic::{AtomicU32, AtomicU8, AtomicU16, AtomicBool, Ordering};

// LED类型定义
type StatusLed = PB0<Output<PushPull>>;
type AnalysisLed = PB1<Output<PushPull>>;
type JitterLed = PB2<Output<PushPull>>;
type DriftLed = PB3<Output<PushPull>>;
type StabilityLed = PB4<Output<PushPull>>;
type AccuracyLed = PB5<Output<PushPull>>;
type PerformanceLed = PB6<Output<PushPull>>;
type ErrorLed = PB7<Output<PushPull>>;

type ModeButton = PC13<Input<PullUp>>;

// 全局变量
static mut ANALYSIS_QUEUE: Queue<AnalysisEvent, 128> = Queue::new();
static mut ANALYSIS_PRODUCER: Option<Producer<AnalysisEvent, 128>> = None;
static mut ANALYSIS_CONSUMER: Option<Consumer<AnalysisEvent, 128>> = None;

static SYSTEM_TIME_US: AtomicU32 = AtomicU32::new(0);
static ANALYSIS_MODE: AtomicU8 = AtomicU8::new(0);
static ANALYSIS_SAMPLES: AtomicU32 = AtomicU32::new(0);
static ANALYSIS_ACTIVE: AtomicBool = AtomicBool::new(false);

// 分析统计
static TIMING_STATS: [AtomicU32; 8] = [
    AtomicU32::new(0), // 平均抖动(ns)
    AtomicU32::new(0), // 最大抖动(ns)
    AtomicU32::new(0), // 时钟漂移(ppb)
    AtomicU32::new(0), // 稳定性指数(0-100)
    AtomicU32::new(0), // 精度百分比(0-100)
    AtomicU32::new(0), // 性能评分(0-100)
    AtomicU32::new(0), // 异常事件数
    AtomicU32::new(0), // 分析周期数
];

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // 启用DWT计数器用于高精度时间测量
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.mhz()).freeze();

    // 配置GPIO
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    let mut status_led = gpiob.pb0.into_push_pull_output();
    let mut analysis_led = gpiob.pb1.into_push_pull_output();
    let mut jitter_led = gpiob.pb2.into_push_pull_output();
    let mut drift_led = gpiob.pb3.into_push_pull_output();
    let mut stability_led = gpiob.pb4.into_push_pull_output();
    let mut accuracy_led = gpiob.pb5.into_push_pull_output();
    let mut performance_led = gpiob.pb6.into_push_pull_output();
    let mut error_led = gpiob.pb7.into_push_pull_output();
    
    let mode_button = gpioc.pc13.into_pull_up_input();

    // 初始化分析事件队列
    let (producer, consumer) = unsafe { ANALYSIS_QUEUE.split() };
    unsafe {
        ANALYSIS_PRODUCER = Some(producer);
        ANALYSIS_CONSUMER = Some(consumer);
    }

    // 配置定时器用于生成测试信号
    let mut timer2 = Timer::tim2(dp.TIM2, &clocks);
    let mut timer3 = Timer::tim3(dp.TIM3, &clocks);
    let mut timer4 = Timer::tim4(dp.TIM4, &clocks);
    let mut timer5 = Timer::tim5(dp.TIM5, &clocks);

    // 设置不同精度的定时器用于分析测试
    timer2.start(1000.hz()); // 1kHz - 基准信号
    timer3.start(10000.hz()); // 10kHz - 高频测试
    timer4.start(100.hz()); // 100Hz - 低频测试
    timer5.start(1.hz()); // 1Hz - 系统时间更新

    // 启用定时器中断
    timer2.listen(Event::TimeOut);
    timer3.listen(Event::TimeOut);
    timer4.listen(Event::TimeOut);
    timer5.listen(Event::TimeOut);

    // 配置NVIC
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM2);
        NVIC::unmask(stm32::Interrupt::TIM3);
        NVIC::unmask(stm32::Interrupt::TIM4);
        NVIC::unmask(stm32::Interrupt::TIM5);
    }

    // 创建定时分析器实例
    let mut timing_analyzer = TimingAnalyzer::new(168_000_000); // 168MHz时钟
    timing_analyzer.init().ok();

    // 配置延时定时器
    let mut delay_timer = Timer::tim6(dp.TIM6, &clocks).delay_ms();

    // 系统启动指示
    status_led.set_high();
    delay_timer.delay_ms(1000u32);

    // 定时分析管理器
    let mut analysis_manager = TimingAnalysisManager::new(timing_analyzer);
    let mut button_debouncer = ButtonDebouncer::new();
    let mut demo_counter = 0u32;

    loop {
        // 更新系统时间
        demo_counter += 1;
        let current_time_us = demo_counter * 100; // 100μs per loop
        SYSTEM_TIME_US.store(current_time_us, Ordering::Relaxed);
        update_global_time(current_time_us as u64 * 1000);

        // 按钮防抖处理
        let button_pressed = button_debouncer.update(mode_button.is_low());

        if button_pressed {
            // 切换分析模式
            let current_mode = ANALYSIS_MODE.load(Ordering::Relaxed);
            let new_mode = (current_mode + 1) % 4;
            ANALYSIS_MODE.store(new_mode, Ordering::Relaxed);

            // 应用新的分析模式
            analysis_manager.set_analysis_mode(new_mode);

            // 重置统计
            analysis_manager.reset_statistics();

            // 模式切换指示
            for _ in 0..3 {
                analysis_led.set_low();
                delay_timer.delay_ms(100u32);
                analysis_led.set_high();
                delay_timer.delay_ms(100u32);
            }
        }

        // 处理分析事件
        if let Some(consumer) = unsafe { ANALYSIS_CONSUMER.as_mut() } {
            while let Some(event) = consumer.dequeue() {
                analysis_manager.handle_analysis_event(event);
            }
        }

        // 执行定时分析
        analysis_manager.run_timing_analysis();

        // 显示分析状态
        display_analysis_status(
            &analysis_manager,
            &mut jitter_led,
            &mut drift_led,
            &mut stability_led,
            &mut accuracy_led,
            &mut performance_led,
            &mut error_led,
            demo_counter,
        );

        // 状态LED心跳
        if demo_counter % 100 == 0 {
            status_led.toggle();
        }

        // 分析活动指示
        analysis_led.set_state(ANALYSIS_ACTIVE.load(Ordering::Relaxed).into()).ok();

        delay_timer.delay_ms(100u32);
    }
}

/// 显示分析状态
fn display_analysis_status(
    manager: &TimingAnalysisManager,
    jitter_led: &mut impl embedded_hal::digital::v2::OutputPin,
    drift_led: &mut impl embedded_hal::digital::v2::OutputPin,
    stability_led: &mut impl embedded_hal::digital::v2::OutputPin,
    accuracy_led: &mut impl embedded_hal::digital::v2::OutputPin,
    performance_led: &mut impl embedded_hal::digital::v2::OutputPin,
    error_led: &mut impl embedded_hal::digital::v2::OutputPin,
    counter: u32,
) {
    let analysis = manager.get_timing_analysis();
    
    // 抖动指示
    let avg_jitter = TIMING_STATS[0].load(Ordering::Relaxed);
    if avg_jitter < 100 { // < 100ns
        jitter_led.set_high().ok();
    } else if avg_jitter < 1000 { // < 1μs
        jitter_led.set_state(((counter / 10) % 2 == 0).into()).ok();
    } else {
        jitter_led.set_state(((counter / 5) % 2 == 0).into()).ok();
    }
    
    // 漂移指示
    let drift_ppb = TIMING_STATS[2].load(Ordering::Relaxed);
    if drift_ppb < 100 { // < 100ppb
        drift_led.set_high().ok();
    } else if drift_ppb < 1000 { // < 1ppm
        drift_led.set_state(((counter / 12) % 2 == 0).into()).ok();
    } else {
        drift_led.set_state(((counter / 6) % 2 == 0).into()).ok();
    }
    
    // 稳定性指示
    let stability = TIMING_STATS[3].load(Ordering::Relaxed);
    if stability > 95 {
        stability_led.set_high().ok();
    } else if stability > 80 {
        stability_led.set_state(((counter / 15) % 2 == 0).into()).ok();
    } else {
        stability_led.set_low().ok();
    }
    
    // 精度指示
    let accuracy = TIMING_STATS[4].load(Ordering::Relaxed);
    if accuracy > 99 {
        accuracy_led.set_high().ok();
    } else if accuracy > 95 {
        accuracy_led.set_state(((counter / 8) % 2 == 0).into()).ok();
    } else {
        accuracy_led.set_low().ok();
    }
    
    // 性能指示
    let performance = TIMING_STATS[5].load(Ordering::Relaxed);
    if performance > 90 {
        performance_led.set_high().ok();
    } else if performance > 70 {
        performance_led.set_state(((counter / 20) % 2 == 0).into()).ok();
    } else {
        performance_led.set_low().ok();
    }
    
    // 错误指示
    let error_count = TIMING_STATS[6].load(Ordering::Relaxed);
    if error_count > 0 {
        error_led.set_state(((counter / 7) % 2 == 0).into()).ok();
    } else {
        error_led.set_low().ok();
    }
}

/// 定时分析管理器
struct TimingAnalysisManager {
    timing_analyzer: TimingAnalyzer,
    current_mode: u8,
    analysis_results: TimingAnalysis,
    measurement_buffer: [MeasurementResult; 256],
    buffer_index: usize,
    buffer_full: bool,
    reference_timestamps: [u64; 4],
    last_analysis_time: u64,
}

impl TimingAnalysisManager {
    fn new(timing_analyzer: TimingAnalyzer) -> Self {
        Self {
            timing_analyzer,
            current_mode: 0,
            analysis_results: TimingAnalysis::default(),
            measurement_buffer: [MeasurementResult::default(); 256],
            buffer_index: 0,
            buffer_full: false,
            reference_timestamps: [0; 4],
            last_analysis_time: 0,
        }
    }
    
    fn set_analysis_mode(&mut self, mode: u8) {
        self.current_mode = mode;
        
        // 清空缓冲区
        self.buffer_index = 0;
        self.buffer_full = false;
        
        // 重置参考时间戳
        let current_time = get_global_time();
        for timestamp in &mut self.reference_timestamps {
            *timestamp = current_time;
        }
        
        self.last_analysis_time = current_time;
        
        ANALYSIS_ACTIVE.store(true, Ordering::Relaxed);
    }
    
    fn handle_analysis_event(&mut self, event: AnalysisEvent) {
        // 创建测量结果
        let measurement = MeasurementResult {
            measurement_id: event.timer_id,
            start_time: self.reference_timestamps[event.timer_id as usize],
            end_time: event.timestamp,
            duration_ns: event.timestamp - self.reference_timestamps[event.timer_id as usize],
            frequency_hz: if event.period_ns > 0 { 1_000_000_000.0 / event.period_ns as f32 } else { 0.0 },
            duty_cycle_percent: 50.0, // 假设50%占空比
            phase_degrees: 0.0,
            accuracy_percent: 99.0,
            jitter_ns: event.jitter_ns,
            is_valid: true,
        };
        
        // 添加到缓冲区
        self.add_measurement(measurement);
        
        // 更新参考时间戳
        self.reference_timestamps[event.timer_id as usize] = event.timestamp;
        
        ANALYSIS_SAMPLES.fetch_add(1, Ordering::Relaxed);
    }
    
    fn add_measurement(&mut self, measurement: MeasurementResult) {
        self.measurement_buffer[self.buffer_index] = measurement;
        self.buffer_index = (self.buffer_index + 1) % self.measurement_buffer.len();
        
        if self.buffer_index == 0 {
            self.buffer_full = true;
        }
    }
    
    fn run_timing_analysis(&mut self) {
        let current_time = get_global_time();
        
        // 每秒执行一次完整分析
        if current_time - self.last_analysis_time >= 1_000_000 {
            self.perform_comprehensive_analysis();
            self.last_analysis_time = current_time;
        }
        
        // 根据模式执行特定分析
        match self.current_mode {
            0 => self.analyze_jitter(),
            1 => self.analyze_drift(),
            2 => self.analyze_stability(),
            3 => self.analyze_comprehensive(),
            _ => {}
        }
    }
    
    fn analyze_jitter(&mut self) {
        let sample_count = if self.buffer_full { self.measurement_buffer.len() } else { self.buffer_index };
        
        if sample_count < 2 {
            return;
        }
        
        let mut jitter_values = [0u32; 256];
        let mut jitter_count = 0;
        
        // 计算相邻测量之间的抖动
        for i in 1..sample_count {
            let prev_duration = self.measurement_buffer[i - 1].duration_ns;
            let curr_duration = self.measurement_buffer[i].duration_ns;
            
            if prev_duration > 0 && curr_duration > 0 {
                let jitter = curr_duration.abs_diff(prev_duration);
                if jitter_count < jitter_values.len() {
                    jitter_values[jitter_count] = jitter as u32;
                    jitter_count += 1;
                }
            }
        }
        
        if jitter_count > 0 {
            // 计算平均抖动
            let total_jitter: u64 = jitter_values[..jitter_count].iter().map(|&x| x as u64).sum();
            let avg_jitter = (total_jitter / jitter_count as u64) as u32;
            
            // 计算最大抖动
            let max_jitter = jitter_values[..jitter_count].iter().max().copied().unwrap_or(0);
            
            TIMING_STATS[0].store(avg_jitter, Ordering::Relaxed);
            TIMING_STATS[1].store(max_jitter, Ordering::Relaxed);
            
            self.analysis_results.avg_jitter_ns = avg_jitter as f32;
            self.analysis_results.max_jitter_ns = max_jitter as f32;
        }
    }
    
    fn analyze_drift(&mut self) {
        let sample_count = if self.buffer_full { self.measurement_buffer.len() } else { self.buffer_index };
        
        if sample_count < 10 {
            return;
        }
        
        // 计算时钟漂移 - 比较期望频率和实际频率
        let mut frequency_sum = 0.0f32;
        let mut valid_samples = 0;
        
        for i in 0..sample_count {
            let measurement = &self.measurement_buffer[i];
            if measurement.frequency_hz > 0.0 {
                frequency_sum += measurement.frequency_hz;
                valid_samples += 1;
            }
        }
        
        if valid_samples > 0 {
            let avg_frequency = frequency_sum / valid_samples as f32;
            let expected_frequency = match self.current_mode {
                0 => 1000.0, // 1kHz
                1 => 10000.0, // 10kHz
                2 => 100.0, // 100Hz
                _ => 1000.0,
            };
            
            // 计算漂移（以ppb为单位）
            let drift_ratio = (avg_frequency - expected_frequency) / expected_frequency;
            let drift_ppb = (drift_ratio * 1_000_000_000.0).abs() as u32;
            
            TIMING_STATS[2].store(drift_ppb, Ordering::Relaxed);
            self.analysis_results.clock_drift_ppb = drift_ppb as f32;
        }
    }
    
    fn analyze_stability(&mut self) {
        let sample_count = if self.buffer_full { self.measurement_buffer.len() } else { self.buffer_index };
        
        if sample_count < 20 {
            return;
        }
        
        // 计算Allan方差来评估稳定性
        let mut durations = [0u64; 256];
        let mut duration_count = 0;
        
        for i in 0..sample_count {
            if self.measurement_buffer[i].duration_ns > 0 {
                durations[duration_count] = self.measurement_buffer[i].duration_ns;
                duration_count += 1;
                if duration_count >= durations.len() {
                    break;
                }
            }
        }
        
        if duration_count > 10 {
            // 计算平均值
            let mean: f64 = durations[..duration_count].iter().map(|&x| x as f64).sum::<f64>() / duration_count as f64;
            
            // 计算方差
            let variance: f64 = durations[..duration_count].iter()
                .map(|&x| {
                    let diff = x as f64 - mean;
                    diff * diff
                })
                .sum::<f64>() / duration_count as f64;
            
            // 计算稳定性指数（0-100）
            let stability_index = if mean > 0.0 {
                let cv = (variance.sqrt() / mean) * 100.0; // 变异系数
                (100.0 - cv.min(100.0)).max(0.0) as u32
            } else {
                0
            };
            
            TIMING_STATS[3].store(stability_index, Ordering::Relaxed);
            self.analysis_results.stability_index = stability_index as f32;
        }
    }
    
    fn analyze_comprehensive(&mut self) {
        // 执行所有分析
        self.analyze_jitter();
        self.analyze_drift();
        self.analyze_stability();
        
        // 计算综合精度
        let jitter_score = if TIMING_STATS[0].load(Ordering::Relaxed) < 100 { 100 } else { 50 };
        let drift_score = if TIMING_STATS[2].load(Ordering::Relaxed) < 1000 { 100 } else { 50 };
        let stability_score = TIMING_STATS[3].load(Ordering::Relaxed);
        
        let accuracy = (jitter_score + drift_score + stability_score) / 3;
        TIMING_STATS[4].store(accuracy, Ordering::Relaxed);
        
        // 计算性能评分
        let sample_count = ANALYSIS_SAMPLES.load(Ordering::Relaxed);
        let performance = if sample_count > 1000 {
            (accuracy * 90 / 100).min(100)
        } else {
            (accuracy * sample_count / 1000).min(100)
        };
        TIMING_STATS[5].store(performance, Ordering::Relaxed);
        
        // 更新分析周期数
        TIMING_STATS[7].fetch_add(1, Ordering::Relaxed);
    }
    
    fn perform_comprehensive_analysis(&mut self) {
        if let Ok(analysis) = self.timing_analyzer.analyze_measurements(&self.measurement_buffer[..]) {
            self.analysis_results = analysis;
            
            // 更新统计数组
            TIMING_STATS[0].store(analysis.avg_jitter_ns as u32, Ordering::Relaxed);
            TIMING_STATS[1].store(analysis.max_jitter_ns as u32, Ordering::Relaxed);
            TIMING_STATS[2].store(analysis.clock_drift_ppb as u32, Ordering::Relaxed);
            TIMING_STATS[3].store(analysis.stability_index as u32, Ordering::Relaxed);
            TIMING_STATS[4].store((analysis.overall_accuracy * 100.0) as u32, Ordering::Relaxed);
            TIMING_STATS[5].store((analysis.performance_score * 100.0) as u32, Ordering::Relaxed);
        }
    }
    
    fn get_timing_analysis(&self) -> &TimingAnalysis {
        &self.analysis_results
    }
    
    fn reset_statistics(&mut self) {
        self.analysis_results = TimingAnalysis::default();
        
        // 重置缓冲区
        self.buffer_index = 0;
        self.buffer_full = false;
        
        // 重置计数器
        ANALYSIS_SAMPLES.store(0, Ordering::Relaxed);
        
        // 重置统计数组
        for stat in &TIMING_STATS {
            stat.store(0, Ordering::Relaxed);
        }
        
        // 重置参考时间戳
        let current_time = get_global_time();
        for timestamp in &mut self.reference_timestamps {
            *timestamp = current_time;
        }
        
        self.last_analysis_time = current_time;
    }
}

/// 分析事件
#[derive(Clone, Copy)]
struct AnalysisEvent {
    timer_id: u8,
    timestamp: u64,
    period_ns: u32,
    jitter_ns: u32,
}

/// 按钮防抖器
struct ButtonDebouncer {
    last_state: bool,
    stable_state: bool,
    counter: u8,
}

impl ButtonDebouncer {
    fn new() -> Self {
        Self {
            last_state: false,
            stable_state: false,
            counter: 0,
        }
    }
    
    fn update(&mut self, current_state: bool) -> bool {
        if current_state != self.last_state {
            self.counter = 0;
            self.last_state = current_state;
        } else {
            if self.counter < 5 {
                self.counter += 1;
            }
        }
        
        if self.counter >= 5 && current_state != self.stable_state {
            self.stable_state = current_state;
            return current_state;
        }
        
        false
    }
}

// 中断处理函数
#[interrupt]
fn TIM2() {
    // 1kHz基准信号
    let current_time = get_global_time();
    static mut LAST_TIM2_TIME: u64 = 0;
    
    let period_ns = if unsafe { LAST_TIM2_TIME } > 0 {
        (current_time - unsafe { LAST_TIM2_TIME }) as u32
    } else {
        1_000_000 // 1ms
    };
    
    let expected_period = 1_000_000u32; // 1ms
    let jitter = period_ns.abs_diff(expected_period);
    
    let event = AnalysisEvent {
        timer_id: 0,
        timestamp: current_time,
        period_ns,
        jitter_ns: jitter,
    };
    
    if let Some(producer) = unsafe { ANALYSIS_PRODUCER.as_mut() } {
        producer.enqueue(event).ok();
    }
    
    unsafe { LAST_TIM2_TIME = current_time; }
    
    // 清除中断标志
    unsafe {
        let tim2 = &(*stm32::TIM2::ptr());
        tim2.sr.modify(|_, w| w.uif().clear_bit());
    }
}

#[interrupt]
fn TIM3() {
    // 10kHz高频测试信号
    let current_time = get_global_time();
    static mut LAST_TIM3_TIME: u64 = 0;
    
    let period_ns = if unsafe { LAST_TIM3_TIME } > 0 {
        (current_time - unsafe { LAST_TIM3_TIME }) as u32
    } else {
        100_000 // 100μs
    };
    
    let expected_period = 100_000u32; // 100μs
    let jitter = period_ns.abs_diff(expected_period);
    
    let event = AnalysisEvent {
        timer_id: 1,
        timestamp: current_time,
        period_ns,
        jitter_ns: jitter,
    };
    
    if let Some(producer) = unsafe { ANALYSIS_PRODUCER.as_mut() } {
        producer.enqueue(event).ok();
    }
    
    unsafe { LAST_TIM3_TIME = current_time; }
    
    // 清除中断标志
    unsafe {
        let tim3 = &(*stm32::TIM3::ptr());
        tim3.sr.modify(|_, w| w.uif().clear_bit());
    }
}

#[interrupt]
fn TIM4() {
    // 100Hz低频测试信号
    let current_time = get_global_time();
    static mut LAST_TIM4_TIME: u64 = 0;
    
    let period_ns = if unsafe { LAST_TIM4_TIME } > 0 {
        (current_time - unsafe { LAST_TIM4_TIME }) as u32
    } else {
        10_000_000 // 10ms
    };
    
    let expected_period = 10_000_000u32; // 10ms
    let jitter = period_ns.abs_diff(expected_period);
    
    let event = AnalysisEvent {
        timer_id: 2,
        timestamp: current_time,
        period_ns,
        jitter_ns: jitter,
    };
    
    if let Some(producer) = unsafe { ANALYSIS_PRODUCER.as_mut() } {
        producer.enqueue(event).ok();
    }
    
    unsafe { LAST_TIM4_TIME = current_time; }
    
    // 清除中断标志
    unsafe {
        let tim4 = &(*stm32::TIM4::ptr());
        tim4.sr.modify(|_, w| w.uif().clear_bit());
    }
}

#[interrupt]
fn TIM5() {
    // 系统时间更新
    let current_time = SYSTEM_TIME_US.load(Ordering::Relaxed);
    update_global_time(current_time as u64 * 1000);
    
    // 清除中断标志
    unsafe {
        let tim5 = &(*stm32::TIM5::ptr());
        tim5.sr.modify(|_, w| w.uif().clear_bit());
    }
}