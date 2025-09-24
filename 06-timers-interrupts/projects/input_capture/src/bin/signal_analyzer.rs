#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, PushPull, Pin, Input, PullUp},
    timer::{Timer, Event, CounterUs},
    rcc::Clocks,
    interrupt,
};
use heapless::spsc::{Queue, Producer, Consumer};
use heapless::{Vec, Deque};
use rtt_target::{rprintln, rtt_init_print};
use input_capture::{CaptureEvent, EdgeType, CaptureMode};
use libm::{fabsf, sqrtf, fminf, fmaxf, sinf, cosf, atan2f, logf, powf};
use cortex_m::peripheral::NVIC;

// 信号分析事件类型
#[derive(Debug, Clone, Copy)]
enum SignalEvent {
    FrequencyDetected(f32),      // 频率检测
    AmplitudeUpdate(f32),        // 幅度更新
    PhaseShift(f32),             // 相位偏移
    HarmonicDetected(u8, f32),   // 谐波检测（次数，幅度）
    NoiseLevel(f32),             // 噪声水平
    SignalQuality(f32),          // 信号质量
    AnalysisComplete(SignalAnalysisResult), // 分析完成
    ModeChange(AnalysisMode),    // 模式切换
    TriggerEvent(TriggerType),   // 触发事件
}

#[derive(Debug, Clone, Copy)]
enum AnalysisMode {
    FrequencyAnalysis,  // 频率分析
    SpectrumAnalysis,   // 频谱分析
    HarmonicAnalysis,   // 谐波分析
    PhaseAnalysis,      // 相位分析
    NoiseAnalysis,      // 噪声分析
    QualityAnalysis,    // 质量分析
    RealTimeAnalysis,   // 实时分析
}

#[derive(Debug, Clone, Copy)]
enum TriggerType {
    RisingEdge,     // 上升沿触发
    FallingEdge,    // 下降沿触发
    LevelHigh,      // 高电平触发
    LevelLow,       // 低电平触发
    FrequencyMatch, // 频率匹配触发
    AmplitudeMatch, // 幅度匹配触发
}

#[derive(Debug, Clone, Copy)]
struct SignalAnalysisResult {
    fundamental_freq: f32,    // 基频
    peak_amplitude: f32,      // 峰值幅度
    rms_amplitude: f32,       // RMS幅度
    thd_percent: f32,         // 总谐波失真
    snr_db: f32,             // 信噪比
    phase_deg: f32,          // 相位
    duty_cycle: f32,         // 占空比
    rise_time_us: f32,       // 上升时间
    fall_time_us: f32,       // 下降时间
    jitter_us: f32,          // 抖动
}

// 快速傅里叶变换（简化版）
struct SimpleDFT {
    sample_buffer: Deque<f32, 256>,
    frequency_bins: [f32; 128],
    magnitude_bins: [f32; 128],
    sample_rate: f32,
    window_function: WindowType,
}

#[derive(Debug, Clone, Copy)]
enum WindowType {
    Rectangular,
    Hamming,
    Hanning,
    Blackman,
}

impl SimpleDFT {
    fn new(sample_rate: f32) -> Self {
        Self {
            sample_buffer: Deque::new(),
            frequency_bins: [0.0; 128],
            magnitude_bins: [0.0; 128],
            sample_rate,
            window_function: WindowType::Hamming,
        }
    }

    fn add_sample(&mut self, sample: f32) {
        if self.sample_buffer.len() >= 256 {
            self.sample_buffer.pop_front();
        }
        let _ = self.sample_buffer.push_back(sample);
    }

    fn compute_spectrum(&mut self) -> Result<(), &'static str> {
        if self.sample_buffer.len() < 128 {
            return Err("样本数量不足");
        }

        let n = 128; // DFT点数
        
        // 应用窗函数并计算DFT
        for k in 0..n {
            let mut real_sum = 0.0;
            let mut imag_sum = 0.0;
            
            for (i, &sample) in self.sample_buffer.iter().take(n).enumerate() {
                let windowed_sample = sample * self.window_function(i, n);
                let angle = -2.0 * core::f32::consts::PI * (k as f32) * (i as f32) / (n as f32);
                
                real_sum += windowed_sample * cosf(angle);
                imag_sum += windowed_sample * sinf(angle);
            }
            
            // 计算幅度
            self.magnitude_bins[k] = sqrtf(real_sum * real_sum + imag_sum * imag_sum);
            
            // 计算频率
            self.frequency_bins[k] = (k as f32) * self.sample_rate / (n as f32);
        }

        Ok(())
    }

    fn window_function(&self, n: usize, N: usize) -> f32 {
        let n_f = n as f32;
        let N_f = N as f32;
        
        match self.window_function {
            WindowType::Rectangular => 1.0,
            WindowType::Hamming => 0.54 - 0.46 * cosf(2.0 * core::f32::consts::PI * n_f / (N_f - 1.0)),
            WindowType::Hanning => 0.5 * (1.0 - cosf(2.0 * core::f32::consts::PI * n_f / (N_f - 1.0))),
            WindowType::Blackman => {
                0.42 - 0.5 * cosf(2.0 * core::f32::consts::PI * n_f / (N_f - 1.0))
                    + 0.08 * cosf(4.0 * core::f32::consts::PI * n_f / (N_f - 1.0))
            }
        }
    }

    fn find_peak_frequency(&self) -> (f32, f32) {
        let mut max_magnitude = 0.0;
        let mut peak_frequency = 0.0;
        
        // 跳过DC分量（k=0）
        for k in 1..self.magnitude_bins.len() {
            if self.magnitude_bins[k] > max_magnitude {
                max_magnitude = self.magnitude_bins[k];
                peak_frequency = self.frequency_bins[k];
            }
        }
        
        (peak_frequency, max_magnitude)
    }

    fn calculate_thd(&self, fundamental_freq: f32) -> f32 {
        let fundamental_bin = (fundamental_freq * 128.0 / self.sample_rate) as usize;
        if fundamental_bin >= self.magnitude_bins.len() {
            return 0.0;
        }
        
        let fundamental_magnitude = self.magnitude_bins[fundamental_bin];
        let mut harmonic_power = 0.0;
        
        // 计算前5个谐波的功率
        for harmonic in 2..=5 {
            let harmonic_bin = fundamental_bin * harmonic;
            if harmonic_bin < self.magnitude_bins.len() {
                let harmonic_magnitude = self.magnitude_bins[harmonic_bin];
                harmonic_power += harmonic_magnitude * harmonic_magnitude;
            }
        }
        
        if fundamental_magnitude > 0.0 {
            sqrtf(harmonic_power) / fundamental_magnitude * 100.0
        } else {
            0.0
        }
    }
}

// 信号质量分析器
struct SignalQualityAnalyzer {
    amplitude_buffer: Deque<f32, 64>,
    frequency_buffer: Deque<f32, 32>,
    noise_buffer: Deque<f32, 32>,
    last_snr: f32,
    last_thd: f32,
    last_jitter: f32,
}

impl SignalQualityAnalyzer {
    fn new() -> Self {
        Self {
            amplitude_buffer: Deque::new(),
            frequency_buffer: Deque::new(),
            noise_buffer: Deque::new(),
            last_snr: 0.0,
            last_thd: 0.0,
            last_jitter: 0.0,
        }
    }

    fn add_measurement(&mut self, amplitude: f32, frequency: f32, noise: f32) {
        // 添加幅度测量
        if self.amplitude_buffer.len() >= 64 {
            self.amplitude_buffer.pop_front();
        }
        let _ = self.amplitude_buffer.push_back(amplitude);

        // 添加频率测量
        if self.frequency_buffer.len() >= 32 {
            self.frequency_buffer.pop_front();
        }
        let _ = self.frequency_buffer.push_back(frequency);

        // 添加噪声测量
        if self.noise_buffer.len() >= 32 {
            self.noise_buffer.pop_front();
        }
        let _ = self.noise_buffer.push_back(noise);
    }

    fn calculate_snr(&mut self) -> f32 {
        if self.amplitude_buffer.is_empty() || self.noise_buffer.is_empty() {
            return 0.0;
        }

        // 计算信号功率（RMS）
        let signal_power: f32 = self.amplitude_buffer.iter()
            .map(|&a| a * a)
            .sum::<f32>() / self.amplitude_buffer.len() as f32;

        // 计算噪声功率（RMS）
        let noise_power: f32 = self.noise_buffer.iter()
            .map(|&n| n * n)
            .sum::<f32>() / self.noise_buffer.len() as f32;

        if noise_power > 0.0 {
            self.last_snr = 10.0 * logf(signal_power / noise_power) / logf(10.0);
        } else {
            self.last_snr = 100.0; // 无噪声时的高SNR
        }

        self.last_snr
    }

    fn calculate_jitter(&mut self) -> f32 {
        if self.frequency_buffer.len() < 2 {
            return 0.0;
        }

        // 计算频率标准差作为抖动指标
        let mean_freq: f32 = self.frequency_buffer.iter().sum::<f32>() / self.frequency_buffer.len() as f32;
        
        let variance: f32 = self.frequency_buffer.iter()
            .map(|&f| (f - mean_freq) * (f - mean_freq))
            .sum::<f32>() / (self.frequency_buffer.len() - 1) as f32;

        self.last_jitter = sqrtf(variance);
        self.last_jitter
    }

    fn get_quality_score(&self) -> f32 {
        // 综合质量评分（0-100）
        let snr_score = if self.last_snr > 40.0 { 100.0 } else { self.last_snr * 2.5 };
        let thd_score = if self.last_thd < 1.0 { 100.0 } else { 100.0 / self.last_thd };
        let jitter_score = if self.last_jitter < 0.1 { 100.0 } else { 10.0 / self.last_jitter };

        (snr_score + thd_score + jitter_score) / 3.0
    }
}

// 信号分析器管理器
struct SignalAnalyzerManager {
    dft_analyzer: SimpleDFT,
    quality_analyzer: SignalQualityAnalyzer,
    current_mode: AnalysisMode,
    analysis_active: bool,
    trigger_type: TriggerType,
    trigger_level: f32,
    sample_count: u32,
    analysis_results: SignalAnalysisResult,
    statistics: AnalysisStatistics,
    event_producer: Producer<'static, SignalEvent, 64>,
}

#[derive(Debug, Default)]
struct AnalysisStatistics {
    total_samples: u32,
    analysis_cycles: u32,
    frequency_detections: u32,
    harmonic_detections: u32,
    trigger_events: u32,
    quality_measurements: u32,
    processing_time_ms: u32,
    mode_changes: u32,
}

impl SignalAnalyzerManager {
    fn new(
        sample_rate: f32,
        event_producer: Producer<'static, SignalEvent, 64>,
    ) -> Self {
        Self {
            dft_analyzer: SimpleDFT::new(sample_rate),
            quality_analyzer: SignalQualityAnalyzer::new(),
            current_mode: AnalysisMode::FrequencyAnalysis,
            analysis_active: false,
            trigger_type: TriggerType::RisingEdge,
            trigger_level: 2.5, // 2.5V触发电平
            sample_count: 0,
            analysis_results: SignalAnalysisResult {
                fundamental_freq: 0.0,
                peak_amplitude: 0.0,
                rms_amplitude: 0.0,
                thd_percent: 0.0,
                snr_db: 0.0,
                phase_deg: 0.0,
                duty_cycle: 0.0,
                rise_time_us: 0.0,
                fall_time_us: 0.0,
                jitter_us: 0.0,
            },
            statistics: AnalysisStatistics::default(),
            event_producer,
        }
    }

    fn set_mode(&mut self, mode: AnalysisMode) {
        self.current_mode = mode;
        self.statistics.mode_changes += 1;
        rprintln!("信号分析模式切换到: {:?}", mode);
    }

    fn start_analysis(&mut self) {
        self.analysis_active = true;
        self.sample_count = 0;
        rprintln!("开始信号分析");
    }

    fn stop_analysis(&mut self) {
        self.analysis_active = false;
        rprintln!("停止信号分析");
    }

    fn process_sample(&mut self, timestamp: u32, voltage: f32) -> Result<(), &'static str> {
        if !self.analysis_active {
            return Ok(());
        }

        self.sample_count += 1;
        self.statistics.total_samples += 1;

        // 添加样本到DFT分析器
        self.dft_analyzer.add_sample(voltage);

        // 检查触发条件
        self.check_trigger(voltage)?;

        // 根据模式进行不同的分析
        match self.current_mode {
            AnalysisMode::FrequencyAnalysis => {
                self.perform_frequency_analysis()?;
            }
            AnalysisMode::SpectrumAnalysis => {
                if self.sample_count % 64 == 0 {
                    self.perform_spectrum_analysis()?;
                }
            }
            AnalysisMode::HarmonicAnalysis => {
                if self.sample_count % 128 == 0 {
                    self.perform_harmonic_analysis()?;
                }
            }
            AnalysisMode::PhaseAnalysis => {
                self.perform_phase_analysis(voltage)?;
            }
            AnalysisMode::NoiseAnalysis => {
                self.perform_noise_analysis(voltage)?;
            }
            AnalysisMode::QualityAnalysis => {
                if self.sample_count % 32 == 0 {
                    self.perform_quality_analysis()?;
                }
            }
            AnalysisMode::RealTimeAnalysis => {
                self.perform_realtime_analysis(voltage)?;
            }
        }

        Ok(())
    }

    fn check_trigger(&mut self, voltage: f32) -> Result<(), &'static str> {
        let triggered = match self.trigger_type {
            TriggerType::RisingEdge => voltage > self.trigger_level,
            TriggerType::FallingEdge => voltage < self.trigger_level,
            TriggerType::LevelHigh => voltage > self.trigger_level,
            TriggerType::LevelLow => voltage < self.trigger_level,
            TriggerType::FrequencyMatch => {
                // 简化的频率匹配检测
                self.analysis_results.fundamental_freq > (self.trigger_level - 1.0) &&
                self.analysis_results.fundamental_freq < (self.trigger_level + 1.0)
            }
            TriggerType::AmplitudeMatch => {
                self.analysis_results.peak_amplitude > (self.trigger_level - 0.1) &&
                self.analysis_results.peak_amplitude < (self.trigger_level + 0.1)
            }
        };

        if triggered {
            self.statistics.trigger_events += 1;
            self.send_event(SignalEvent::TriggerEvent(self.trigger_type));
        }

        Ok(())
    }

    fn perform_frequency_analysis(&mut self) -> Result<(), &'static str> {
        if self.sample_count % 32 == 0 {
            if let Err(_) = self.dft_analyzer.compute_spectrum() {
                return Err("频谱计算失败");
            }

            let (peak_freq, peak_magnitude) = self.dft_analyzer.find_peak_frequency();
            
            if peak_freq > 0.0 {
                self.analysis_results.fundamental_freq = peak_freq;
                self.analysis_results.peak_amplitude = peak_magnitude;
                self.statistics.frequency_detections += 1;
                
                self.send_event(SignalEvent::FrequencyDetected(peak_freq));
                self.send_event(SignalEvent::AmplitudeUpdate(peak_magnitude));
            }
        }
        Ok(())
    }

    fn perform_spectrum_analysis(&mut self) -> Result<(), &'static str> {
        if let Err(_) = self.dft_analyzer.compute_spectrum() {
            return Err("频谱分析失败");
        }

        let (peak_freq, peak_magnitude) = self.dft_analyzer.find_peak_frequency();
        self.analysis_results.fundamental_freq = peak_freq;
        self.analysis_results.peak_amplitude = peak_magnitude;

        // 计算RMS幅度
        let mut rms_sum = 0.0;
        for &magnitude in &self.dft_analyzer.magnitude_bins[1..64] {
            rms_sum += magnitude * magnitude;
        }
        self.analysis_results.rms_amplitude = sqrtf(rms_sum / 63.0);

        self.statistics.analysis_cycles += 1;
        Ok(())
    }

    fn perform_harmonic_analysis(&mut self) -> Result<(), &'static str> {
        if let Err(_) = self.dft_analyzer.compute_spectrum() {
            return Err("谐波分析失败");
        }

        let (fundamental_freq, _) = self.dft_analyzer.find_peak_frequency();
        
        if fundamental_freq > 0.0 {
            // 计算总谐波失真
            let thd = self.dft_analyzer.calculate_thd(fundamental_freq);
            self.analysis_results.thd_percent = thd;
            
            // 检测谐波
            for harmonic in 2..=5 {
                let harmonic_freq = fundamental_freq * harmonic as f32;
                let harmonic_bin = (harmonic_freq * 128.0 / self.dft_analyzer.sample_rate) as usize;
                
                if harmonic_bin < self.dft_analyzer.magnitude_bins.len() {
                    let harmonic_magnitude = self.dft_analyzer.magnitude_bins[harmonic_bin];
                    if harmonic_magnitude > 0.1 { // 阈值检测
                        self.statistics.harmonic_detections += 1;
                        self.send_event(SignalEvent::HarmonicDetected(harmonic, harmonic_magnitude));
                    }
                }
            }
        }

        Ok(())
    }

    fn perform_phase_analysis(&mut self, voltage: f32) -> Result<(), &'static str> {
        // 简化的相位分析
        // 实际实现需要参考信号或更复杂的算法
        if self.sample_count % 100 == 0 {
            // 模拟相位计算
            let phase = (self.sample_count as f32 * 0.1) % 360.0;
            self.analysis_results.phase_deg = phase;
            self.send_event(SignalEvent::PhaseShift(phase));
        }
        Ok(())
    }

    fn perform_noise_analysis(&mut self, voltage: f32) -> Result<(), &'static str> {
        // 简单的噪声分析
        static mut LAST_VOLTAGE: f32 = 0.0;
        
        unsafe {
            let noise_level = fabsf(voltage - LAST_VOLTAGE);
            LAST_VOLTAGE = voltage;
            
            if self.sample_count % 50 == 0 {
                self.send_event(SignalEvent::NoiseLevel(noise_level));
            }
        }
        
        Ok(())
    }

    fn perform_quality_analysis(&mut self) -> Result<(), &'static str> {
        // 更新质量分析器
        self.quality_analyzer.add_measurement(
            self.analysis_results.peak_amplitude,
            self.analysis_results.fundamental_freq,
            0.1 // 简化的噪声值
        );

        // 计算质量指标
        let snr = self.quality_analyzer.calculate_snr();
        let jitter = self.quality_analyzer.calculate_jitter();
        let quality_score = self.quality_analyzer.get_quality_score();

        self.analysis_results.snr_db = snr;
        self.analysis_results.jitter_us = jitter;

        self.statistics.quality_measurements += 1;
        self.send_event(SignalEvent::SignalQuality(quality_score));

        Ok(())
    }

    fn perform_realtime_analysis(&mut self, voltage: f32) -> Result<(), &'static str> {
        // 实时分析：快速更新基本参数
        if self.sample_count % 10 == 0 {
            if let Err(_) = self.dft_analyzer.compute_spectrum() {
                return Ok(()); // 忽略错误，继续处理
            }

            let (freq, amplitude) = self.dft_analyzer.find_peak_frequency();
            
            if freq > 0.0 {
                self.send_event(SignalEvent::FrequencyDetected(freq));
                self.send_event(SignalEvent::AmplitudeUpdate(amplitude));
            }
        }
        Ok(())
    }

    fn update_analysis(&mut self, dt_ms: u32) -> Result<(), &'static str> {
        self.statistics.processing_time_ms += dt_ms;

        // 定期完成分析
        if self.sample_count > 0 && self.sample_count % 1000 == 0 {
            self.send_event(SignalEvent::AnalysisComplete(self.analysis_results));
        }

        Ok(())
    }

    fn set_trigger(&mut self, trigger_type: TriggerType, level: f32) {
        self.trigger_type = trigger_type;
        self.trigger_level = level;
        rprintln!("触发设置: {:?}, 电平: {:.2} V", trigger_type, level);
    }

    fn get_current_results(&self) -> SignalAnalysisResult {
        self.analysis_results
    }

    fn send_event(&mut self, event: SignalEvent) {
        if self.event_producer.enqueue(event).is_err() {
            rprintln!("警告: 事件队列已满");
        }
    }
}

static mut SIGNAL_EVENT_QUEUE: Queue<SignalEvent, 64> = Queue::new();
static mut SIGNAL_TIMESTAMP: u32 = 0;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("信号分析器系统启动");

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 配置按钮
    let button1 = gpioc.pc13.into_pull_up_input(); // 模式切换
    let button2 = gpioc.pc14.into_pull_up_input(); // 分析控制
    let button3 = gpioc.pc15.into_pull_up_input(); // 触发设置

    // 配置LED指示灯
    let mut status_led = gpiob.pb0.into_push_pull_output();      // 系统状态
    let mut analysis_led = gpiob.pb1.into_push_pull_output();    // 分析活动
    let mut frequency_led = gpiob.pb2.into_push_pull_output();   // 频率检测
    let mut spectrum_led = gpiob.pb3.into_push_pull_output();    // 频谱分析
    let mut harmonic_led = gpiob.pb4.into_push_pull_output();    // 谐波检测
    let mut quality_led = gpiob.pb5.into_push_pull_output();     // 信号质量
    let mut trigger_led = gpiob.pb6.into_push_pull_output();     // 触发指示
    let mut error_led = gpiob.pb7.into_push_pull_output();       // 错误指示

    // 配置信号输入引脚（模拟ADC输入）
    let signal_input = gpioa.pa0.into_analog(); // ADC输入

    // 配置定时器
    let mut timer2 = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer2.start(10.kHz()).unwrap(); // 10kHz采样率
    timer2.listen(Event::Update);

    // 配置系统定时器
    let mut delay = cp.SYST.delay(&clocks);

    // 创建事件队列
    let (event_producer, mut event_consumer) = unsafe {
        SIGNAL_EVENT_QUEUE.split()
    };

    // 创建信号分析器管理器
    let mut analyzer_manager = SignalAnalyzerManager::new(
        10000.0, // 10kHz采样率
        event_producer
    );

    // 启用中断
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }

    // 系统变量
    let mut last_button1_state = button1.is_high();
    let mut last_button2_state = button2.is_high();
    let mut last_button3_state = button3.is_high();
    let mut button1_debounce = 0u32;
    let mut button2_debounce = 0u32;
    let mut button3_debounce = 0u32;
    let mut system_tick = 0u32;
    let mut trigger_level_index = 0u8;

    rprintln!("信号分析器系统就绪");

    loop {
        let current_time = system_tick;
        system_tick = system_tick.wrapping_add(1);

        // 模拟ADC读取（实际应用中需要配置ADC）
        let simulated_voltage = 2.5 + 1.0 * sinf(2.0 * core::f32::consts::PI * 1000.0 * (current_time as f32) / 10000.0);
        
        // 处理信号样本
        if let Err(e) = analyzer_manager.process_sample(current_time, simulated_voltage) {
            rprintln!("信号处理错误: {}", e);
            error_led.set_high();
        } else {
            error_led.set_low();
        }

        // 按钮1处理（模式切换）
        let button1_state = button1.is_high();
        if button1_state != last_button1_state {
            button1_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button1_debounce) > 50 && button1_state && !last_button1_state {
            let new_mode = match analyzer_manager.current_mode {
                AnalysisMode::FrequencyAnalysis => AnalysisMode::SpectrumAnalysis,
                AnalysisMode::SpectrumAnalysis => AnalysisMode::HarmonicAnalysis,
                AnalysisMode::HarmonicAnalysis => AnalysisMode::PhaseAnalysis,
                AnalysisMode::PhaseAnalysis => AnalysisMode::NoiseAnalysis,
                AnalysisMode::NoiseAnalysis => AnalysisMode::QualityAnalysis,
                AnalysisMode::QualityAnalysis => AnalysisMode::RealTimeAnalysis,
                AnalysisMode::RealTimeAnalysis => AnalysisMode::FrequencyAnalysis,
            };
            analyzer_manager.send_event(SignalEvent::ModeChange(new_mode));
        }
        last_button1_state = button1_state;

        // 按钮2处理（分析控制）
        let button2_state = button2.is_high();
        if button2_state != last_button2_state {
            button2_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button2_debounce) > 50 && button2_state && !last_button2_state {
            if analyzer_manager.analysis_active {
                analyzer_manager.stop_analysis();
            } else {
                analyzer_manager.start_analysis();
            }
        }
        last_button2_state = button2_state;

        // 按钮3处理（触发设置）
        let button3_state = button3.is_high();
        if button3_state != last_button3_state {
            button3_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button3_debounce) > 50 && button3_state && !last_button3_state {
            trigger_level_index = (trigger_level_index + 1) % 5;
            let trigger_levels = [1.0, 1.5, 2.0, 2.5, 3.0];
            analyzer_manager.set_trigger(TriggerType::RisingEdge, trigger_levels[trigger_level_index as usize]);
        }
        last_button3_state = button3_state;

        // 处理事件队列
        while let Some(event) = event_consumer.dequeue() {
            match event {
                SignalEvent::FrequencyDetected(freq) => {
                    frequency_led.set_high();
                    rprintln!("检测到频率: {:.2} Hz", freq);
                }
                SignalEvent::AmplitudeUpdate(amplitude) => {
                    // 根据幅度调整LED亮度（简化）
                    if amplitude > 1.0 {
                        spectrum_led.set_high();
                    } else {
                        spectrum_led.set_low();
                    }
                }
                SignalEvent::PhaseShift(phase) => {
                    rprintln!("相位偏移: {:.1}°", phase);
                }
                SignalEvent::HarmonicDetected(harmonic, magnitude) => {
                    harmonic_led.set_high();
                    rprintln!("检测到{}次谐波，幅度: {:.3}", harmonic, magnitude);
                }
                SignalEvent::NoiseLevel(noise) => {
                    rprintln!("噪声水平: {:.3}", noise);
                }
                SignalEvent::SignalQuality(quality) => {
                    if quality > 80.0 {
                        quality_led.set_high();
                    } else if quality > 50.0 {
                        if system_tick % 500 < 250 {
                            quality_led.set_high();
                        } else {
                            quality_led.set_low();
                        }
                    } else {
                        quality_led.set_low();
                    }
                    rprintln!("信号质量: {:.1}%", quality);
                }
                SignalEvent::AnalysisComplete(result) => {
                    rprintln!("分析完成:");
                    rprintln!("  基频: {:.2} Hz", result.fundamental_freq);
                    rprintln!("  峰值幅度: {:.3}", result.peak_amplitude);
                    rprintln!("  RMS幅度: {:.3}", result.rms_amplitude);
                    rprintln!("  THD: {:.2}%", result.thd_percent);
                    rprintln!("  SNR: {:.1} dB", result.snr_db);
                    rprintln!("  相位: {:.1}°", result.phase_deg);
                    rprintln!("  抖动: {:.3} us", result.jitter_us);
                }
                SignalEvent::ModeChange(mode) => {
                    analyzer_manager.set_mode(mode);
                }
                SignalEvent::TriggerEvent(trigger_type) => {
                    trigger_led.set_high();
                    rprintln!("触发事件: {:?}", trigger_type);
                }
            }
        }

        // 更新分析
        if let Err(e) = analyzer_manager.update_analysis(10) {
            rprintln!("更新分析错误: {}", e);
            error_led.set_high();
        }

        // LED指示更新
        // 系统状态LED（心跳）
        if system_tick % 100 == 0 {
            status_led.toggle();
        }

        // 分析活动指示
        if analyzer_manager.analysis_active {
            analysis_led.set_high();
        } else {
            analysis_led.set_low();
        }

        // 频率LED渐暗
        if system_tick % 200 == 0 {
            frequency_led.set_low();
        }

        // 谐波LED渐暗
        if system_tick % 300 == 0 {
            harmonic_led.set_low();
        }

        // 触发LED渐暗
        if system_tick % 100 == 0 {
            trigger_led.set_low();
        }

        // 定期输出统计信息
        if system_tick % 10000 == 0 {
            rprintln!("=== 信号分析器统计信息 ===");
            rprintln!("总样本数: {}", analyzer_manager.statistics.total_samples);
            rprintln!("分析周期: {}", analyzer_manager.statistics.analysis_cycles);
            rprintln!("频率检测: {}", analyzer_manager.statistics.frequency_detections);
            rprintln!("谐波检测: {}", analyzer_manager.statistics.harmonic_detections);
            rprintln!("触发事件: {}", analyzer_manager.statistics.trigger_events);
            rprintln!("质量测量: {}", analyzer_manager.statistics.quality_measurements);
            rprintln!("处理时间: {} ms", analyzer_manager.statistics.processing_time_ms);
            rprintln!("模式切换: {}", analyzer_manager.statistics.mode_changes);
            rprintln!("当前模式: {:?}", analyzer_manager.current_mode);
            rprintln!("触发类型: {:?}", analyzer_manager.trigger_type);
            rprintln!("触发电平: {:.2} V", analyzer_manager.trigger_level);
            
            let results = analyzer_manager.get_current_results();
            rprintln!("当前分析结果:");
            rprintln!("  基频: {:.2} Hz", results.fundamental_freq);
            rprintln!("  峰值幅度: {:.3}", results.peak_amplitude);
            rprintln!("  RMS幅度: {:.3}", results.rms_amplitude);
            rprintln!("  THD: {:.2}%", results.thd_percent);
            rprintln!("  SNR: {:.1} dB", results.snr_db);
        }

        delay.delay_ms(1u32);
    }
}

// 定时器中断处理
#[interrupt]
fn TIM2() {
    unsafe {
        SIGNAL_TIMESTAMP = SIGNAL_TIMESTAMP.wrapping_add(1);
    }
}