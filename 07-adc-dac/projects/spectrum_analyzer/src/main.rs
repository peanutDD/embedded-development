#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::{asm, peripheral::DWT};
use stm32f4xx_hal::{
    pac::{self, ADC1, DMA2, TIM2},
    prelude::*,
    adc::{Adc, config::{AdcConfig, SampleTime, Sequence, Eoc, Scan, Resolution}},
    dma::{Stream0, Channel0, StreamsTuple, Transfer, MemoryToPeripheral, config::DmaConfig},
    timer::{Timer, Event},
    gpio::{Analog, Pin},
    rcc::Clocks,
};
use heapless::{Vec, FnvIndexMap};
use microfft::{real::rfft_1024, Complex32};
use num_complex::Complex;
use libm::{sqrt, log10, atan2, sin, cos};
use critical_section::Mutex;
use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use fugit::Duration;

/// FFT大小常量
const FFT_SIZE: usize = 1024;
const SAMPLE_RATE: u32 = 100_000; // 100kHz采样率
const FREQUENCY_RESOLUTION: f32 = SAMPLE_RATE as f32 / FFT_SIZE as f32; // ~97.66 Hz

/// 频谱分析仪主结构
pub struct SpectrumAnalyzer {
    adc: Adc<ADC1>,
    sample_buffer: [u16; FFT_SIZE],
    fft_input: [f32; FFT_SIZE],
    fft_output: [Complex32; FFT_SIZE / 2 + 1],
    magnitude_spectrum: [f32; FFT_SIZE / 2 + 1],
    power_spectrum: [f32; FFT_SIZE / 2 + 1],
    phase_spectrum: [f32; FFT_SIZE / 2 + 1],
    window_function: [f32; FFT_SIZE],
    peak_detector: PeakDetector,
    spectrum_statistics: SpectrumStatistics,
    sampling_active: bool,
}

/// 峰值检测器
pub struct PeakDetector {
    peaks: Vec<Peak, 32>,
    threshold: f32,
    min_distance: usize,
}

/// 峰值信息
#[derive(Debug, Clone, Copy)]
pub struct Peak {
    pub frequency: f32,
    pub magnitude: f32,
    pub phase: f32,
    pub bin_index: usize,
}

/// 频谱统计信息
#[derive(Debug, Clone)]
pub struct SpectrumStatistics {
    pub total_power: f32,
    pub peak_frequency: f32,
    pub peak_magnitude: f32,
    pub centroid_frequency: f32,
    pub bandwidth: f32,
    pub snr: f32,
    pub thd: f32, // 总谐波失真
}

/// 窗函数类型
#[derive(Debug, Clone, Copy)]
pub enum WindowFunction {
    Rectangular,
    Hanning,
    Hamming,
    Blackman,
    Kaiser(f32), // beta参数
}

/// 频谱显示模式
#[derive(Debug, Clone, Copy)]
pub enum DisplayMode {
    Magnitude,
    Power,
    Phase,
    Waterfall,
}

/// DMA传输状态
static DMA_TRANSFER_COMPLETE: AtomicBool = AtomicBool::new(false);
static SAMPLE_BUFFER: Mutex<RefCell<[u16; FFT_SIZE]>> = Mutex::new(RefCell::new([0; FFT_SIZE]));

impl SpectrumAnalyzer {
    pub fn new(adc: Adc<ADC1>) -> Self {
        let mut analyzer = Self {
            adc,
            sample_buffer: [0; FFT_SIZE],
            fft_input: [0.0; FFT_SIZE],
            fft_output: [Complex32::new(0.0, 0.0); FFT_SIZE / 2 + 1],
            magnitude_spectrum: [0.0; FFT_SIZE / 2 + 1],
            power_spectrum: [0.0; FFT_SIZE / 2 + 1],
            phase_spectrum: [0.0; FFT_SIZE / 2 + 1],
            window_function: [0.0; FFT_SIZE],
            peak_detector: PeakDetector::new(0.1, 10),
            spectrum_statistics: SpectrumStatistics::new(),
            sampling_active: false,
        };

        // 初始化汉宁窗
        analyzer.generate_window(WindowFunction::Hanning);
        analyzer
    }

    /// 生成窗函数
    pub fn generate_window(&mut self, window_type: WindowFunction) {
        match window_type {
            WindowFunction::Rectangular => {
                for i in 0..FFT_SIZE {
                    self.window_function[i] = 1.0;
                }
            }
            WindowFunction::Hanning => {
                for i in 0..FFT_SIZE {
                    let n = i as f32;
                    let N = FFT_SIZE as f32;
                    self.window_function[i] = 0.5 * (1.0 - cos(2.0 * core::f32::consts::PI * n / (N - 1.0)));
                }
            }
            WindowFunction::Hamming => {
                for i in 0..FFT_SIZE {
                    let n = i as f32;
                    let N = FFT_SIZE as f32;
                    self.window_function[i] = 0.54 - 0.46 * cos(2.0 * core::f32::consts::PI * n / (N - 1.0));
                }
            }
            WindowFunction::Blackman => {
                for i in 0..FFT_SIZE {
                    let n = i as f32;
                    let N = FFT_SIZE as f32;
                    let a0 = 0.42;
                    let a1 = 0.5;
                    let a2 = 0.08;
                    self.window_function[i] = a0 
                        - a1 * cos(2.0 * core::f32::consts::PI * n / (N - 1.0))
                        + a2 * cos(4.0 * core::f32::consts::PI * n / (N - 1.0));
                }
            }
            WindowFunction::Kaiser(beta) => {
                // 简化的Kaiser窗实现
                for i in 0..FFT_SIZE {
                    let n = i as f32;
                    let N = FFT_SIZE as f32;
                    let alpha = (N - 1.0) / 2.0;
                    let x = (n - alpha) / alpha;
                    // 简化的修正贝塞尔函数近似
                    let bessel_i0 = |x: f32| -> f32 {
                        let mut sum = 1.0;
                        let mut term = 1.0;
                        for k in 1..20 {
                            term *= (x * x) / (4.0 * k as f32 * k as f32);
                            sum += term;
                        }
                        sum
                    };
                    
                    self.window_function[i] = bessel_i0(beta * sqrt(1.0 - x * x)) / bessel_i0(beta);
                }
            }
        }
    }

    /// 开始采样
    pub fn start_sampling(&mut self) {
        self.sampling_active = true;
        DMA_TRANSFER_COMPLETE.store(false, Ordering::SeqCst);
    }

    /// 停止采样
    pub fn stop_sampling(&mut self) {
        self.sampling_active = false;
    }

    /// 处理采样数据并执行FFT
    pub fn process_samples(&mut self) -> bool {
        if !DMA_TRANSFER_COMPLETE.load(Ordering::SeqCst) {
            return false;
        }

        // 从DMA缓冲区复制数据
        critical_section::with(|cs| {
            let buffer = SAMPLE_BUFFER.borrow(cs).borrow();
            self.sample_buffer.copy_from_slice(&*buffer);
        });

        // 转换为浮点数并应用窗函数
        for i in 0..FFT_SIZE {
            let sample = (self.sample_buffer[i] as f32 - 2048.0) / 2048.0; // 转换为-1到1范围
            self.fft_input[i] = sample * self.window_function[i];
        }

        // 执行实数FFT
        let mut fft_buffer = self.fft_input.clone();
        let spectrum = rfft_1024(&mut fft_buffer);
        
        // 复制FFT结果
        for (i, &complex_val) in spectrum.iter().enumerate() {
            self.fft_output[i] = complex_val;
        }

        // 计算幅度谱、功率谱和相位谱
        self.compute_spectra();

        // 检测峰值
        self.peak_detector.detect_peaks(&self.magnitude_spectrum);

        // 计算统计信息
        self.compute_statistics();

        DMA_TRANSFER_COMPLETE.store(false, Ordering::SeqCst);
        true
    }

    /// 计算各种频谱
    fn compute_spectra(&mut self) {
        for i in 0..self.fft_output.len() {
            let real = self.fft_output[i].re;
            let imag = self.fft_output[i].im;
            
            // 幅度谱
            self.magnitude_spectrum[i] = sqrt(real * real + imag * imag);
            
            // 功率谱
            self.power_spectrum[i] = self.magnitude_spectrum[i] * self.magnitude_spectrum[i];
            
            // 相位谱
            self.phase_spectrum[i] = atan2(imag, real);
        }

        // 归一化处理
        let max_magnitude = self.magnitude_spectrum.iter()
            .fold(0.0f32, |acc, &x| acc.max(x));
        
        if max_magnitude > 0.0 {
            for i in 0..self.magnitude_spectrum.len() {
                self.magnitude_spectrum[i] /= max_magnitude;
            }
        }
    }

    /// 计算频谱统计信息
    fn compute_statistics(&mut self) {
        let mut total_power = 0.0;
        let mut weighted_freq_sum = 0.0;
        let mut peak_magnitude = 0.0;
        let mut peak_frequency = 0.0;

        for i in 1..self.power_spectrum.len() {
            let power = self.power_spectrum[i];
            let frequency = i as f32 * FREQUENCY_RESOLUTION;
            
            total_power += power;
            weighted_freq_sum += frequency * power;
            
            if self.magnitude_spectrum[i] > peak_magnitude {
                peak_magnitude = self.magnitude_spectrum[i];
                peak_frequency = frequency;
            }
        }

        self.spectrum_statistics.total_power = total_power;
        self.spectrum_statistics.peak_magnitude = peak_magnitude;
        self.spectrum_statistics.peak_frequency = peak_frequency;
        
        // 质心频率
        self.spectrum_statistics.centroid_frequency = if total_power > 0.0 {
            weighted_freq_sum / total_power
        } else {
            0.0
        };

        // 计算带宽 (3dB带宽)
        self.compute_bandwidth();

        // 计算信噪比
        self.compute_snr();

        // 计算总谐波失真
        self.compute_thd();
    }

    /// 计算3dB带宽
    fn compute_bandwidth(&mut self) {
        let peak_power = self.spectrum_statistics.peak_magnitude * self.spectrum_statistics.peak_magnitude;
        let threshold = peak_power / 2.0; // -3dB点

        let mut lower_freq = 0.0;
        let mut upper_freq = 0.0;
        let mut found_lower = false;
        let mut found_upper = false;

        for i in 1..self.power_spectrum.len() {
            let power = self.power_spectrum[i];
            let frequency = i as f32 * FREQUENCY_RESOLUTION;

            if !found_lower && power >= threshold {
                lower_freq = frequency;
                found_lower = true;
            }

            if found_lower && power >= threshold {
                upper_freq = frequency;
                found_upper = true;
            }
        }

        self.spectrum_statistics.bandwidth = if found_upper && found_lower {
            upper_freq - lower_freq
        } else {
            0.0
        };
    }

    /// 计算信噪比
    fn compute_snr(&mut self) {
        let signal_power = self.spectrum_statistics.peak_magnitude * self.spectrum_statistics.peak_magnitude;
        
        // 估算噪声功率（排除峰值附近的频率）
        let peak_bin = (self.spectrum_statistics.peak_frequency / FREQUENCY_RESOLUTION) as usize;
        let exclude_range = 10; // 排除峰值附近10个频率点
        
        let mut noise_power = 0.0;
        let mut noise_samples = 0;
        
        for i in 1..self.power_spectrum.len() {
            if (i < peak_bin.saturating_sub(exclude_range)) || (i > peak_bin + exclude_range) {
                noise_power += self.power_spectrum[i];
                noise_samples += 1;
            }
        }
        
        if noise_samples > 0 {
            noise_power /= noise_samples as f32;
            self.spectrum_statistics.snr = if noise_power > 0.0 {
                10.0 * log10(signal_power / noise_power)
            } else {
                100.0 // 很高的SNR
            };
        } else {
            self.spectrum_statistics.snr = 0.0;
        }
    }

    /// 计算总谐波失真
    fn compute_thd(&mut self) {
        let fundamental_bin = (self.spectrum_statistics.peak_frequency / FREQUENCY_RESOLUTION) as usize;
        let fundamental_power = self.power_spectrum[fundamental_bin];
        
        let mut harmonic_power = 0.0;
        
        // 计算前5次谐波的功率
        for harmonic in 2..=5 {
            let harmonic_bin = fundamental_bin * harmonic;
            if harmonic_bin < self.power_spectrum.len() {
                harmonic_power += self.power_spectrum[harmonic_bin];
            }
        }
        
        self.spectrum_statistics.thd = if fundamental_power > 0.0 {
            100.0 * sqrt(harmonic_power / fundamental_power)
        } else {
            0.0
        };
    }

    /// 获取频谱数据
    pub fn get_magnitude_spectrum(&self) -> &[f32] {
        &self.magnitude_spectrum
    }

    pub fn get_power_spectrum(&self) -> &[f32] {
        &self.power_spectrum
    }

    pub fn get_phase_spectrum(&self) -> &[f32] {
        &self.phase_spectrum
    }

    pub fn get_peaks(&self) -> &[Peak] {
        &self.peak_detector.peaks
    }

    pub fn get_statistics(&self) -> &SpectrumStatistics {
        &self.spectrum_statistics
    }

    /// 频率到频率点索引的转换
    pub fn frequency_to_bin(&self, frequency: f32) -> usize {
        (frequency / FREQUENCY_RESOLUTION) as usize
    }

    /// 频率点索引到频率的转换
    pub fn bin_to_frequency(&self, bin: usize) -> f32 {
        bin as f32 * FREQUENCY_RESOLUTION
    }
}

impl PeakDetector {
    pub fn new(threshold: f32, min_distance: usize) -> Self {
        Self {
            peaks: Vec::new(),
            threshold,
            min_distance,
        }
    }

    pub fn detect_peaks(&mut self, spectrum: &[f32]) {
        self.peaks.clear();

        for i in self.min_distance..(spectrum.len() - self.min_distance) {
            if spectrum[i] > self.threshold {
                let mut is_peak = true;
                
                // 检查是否为局部最大值
                for j in (i - self.min_distance)..=(i + self.min_distance) {
                    if j != i && spectrum[j] >= spectrum[i] {
                        is_peak = false;
                        break;
                    }
                }
                
                if is_peak {
                    let frequency = i as f32 * FREQUENCY_RESOLUTION;
                    let peak = Peak {
                        frequency,
                        magnitude: spectrum[i],
                        phase: 0.0, // 需要从相位谱中获取
                        bin_index: i,
                    };
                    
                    if self.peaks.push(peak).is_err() {
                        break; // 峰值缓冲区已满
                    }
                }
            }
        }

        // 按幅度排序
        self.peaks.sort_by(|a, b| b.magnitude.partial_cmp(&a.magnitude).unwrap());
    }

    pub fn set_threshold(&mut self, threshold: f32) {
        self.threshold = threshold;
    }

    pub fn set_min_distance(&mut self, min_distance: usize) {
        self.min_distance = min_distance;
    }
}

impl SpectrumStatistics {
    pub fn new() -> Self {
        Self {
            total_power: 0.0,
            peak_frequency: 0.0,
            peak_magnitude: 0.0,
            centroid_frequency: 0.0,
            bandwidth: 0.0,
            snr: 0.0,
            thd: 0.0,
        }
    }
}

/// 频谱显示器
pub struct SpectrumDisplay {
    display_mode: DisplayMode,
    frequency_range: (f32, f32),
    magnitude_range: (f32, f32),
    log_scale: bool,
}

impl SpectrumDisplay {
    pub fn new() -> Self {
        Self {
            display_mode: DisplayMode::Magnitude,
            frequency_range: (0.0, SAMPLE_RATE as f32 / 2.0),
            magnitude_range: (-80.0, 0.0), // dB范围
            log_scale: true,
        }
    }

    pub fn set_display_mode(&mut self, mode: DisplayMode) {
        self.display_mode = mode;
    }

    pub fn set_frequency_range(&mut self, min_freq: f32, max_freq: f32) {
        self.frequency_range = (min_freq, max_freq);
    }

    pub fn set_magnitude_range(&mut self, min_mag: f32, max_mag: f32) {
        self.magnitude_range = (min_mag, max_mag);
    }

    /// 将线性幅度转换为dB
    pub fn linear_to_db(&self, linear: f32) -> f32 {
        if linear > 0.0 {
            20.0 * log10(linear)
        } else {
            -100.0 // 最小dB值
        }
    }

    /// 格式化频率显示
    pub fn format_frequency(&self, frequency: f32) -> heapless::String<16> {
        if frequency >= 1000.0 {
            heapless::String::from(format!("{:.1}kHz", frequency / 1000.0).as_str())
        } else {
            heapless::String::from(format!("{:.0}Hz", frequency).as_str())
        }
    }
}

#[entry]
fn main() -> ! {
    // 初始化RTT日志
    rtt_target::rtt_init_print!();
    rtt_target::rprintln!("频谱分析仪系统启动");

    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟到168MHz
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

    // 启用DWT周期计数器
    let mut dwt = cp.DWT;
    let dcb = cp.DCB;
    dwt.enable_cycle_counter(&dcb);

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let adc_pin = gpioa.pa0.into_analog(); // ADC输入引脚

    // 配置ADC
    let adc_config = AdcConfig::default()
        .resolution(Resolution::Twelve)
        .end_of_conversion_interrupt(Eoc::Conversion)
        .scan(Scan::Enabled);
    
    let mut adc = Adc::adc1(dp.ADC1, true, adc_config);
    adc.set_sample_time(SampleTime::Cycles_480);

    // 创建频谱分析仪
    let mut spectrum_analyzer = SpectrumAnalyzer::new(adc);
    let mut display = SpectrumDisplay::new();

    // 配置采样定时器
    let mut timer = Timer::new(dp.TIM2, &clocks);
    timer.start(Duration::<u32, 1, 1_000_000>::from_ticks(10)); // 100kHz采样率
    timer.listen(Event::Update);

    rtt_target::rprintln!("频谱分析仪配置完成");
    rtt_target::rprintln!("采样率: {} Hz", SAMPLE_RATE);
    rtt_target::rprintln!("FFT大小: {}", FFT_SIZE);
    rtt_target::rprintln!("频率分辨率: {:.2} Hz", FREQUENCY_RESOLUTION);

    // 开始采样
    spectrum_analyzer.start_sampling();

    let mut frame_counter = 0u32;
    let mut last_report_time = DWT::cycle_count();

    // 主循环
    loop {
        // 处理采样数据
        if spectrum_analyzer.process_samples() {
            frame_counter += 1;

            // 每秒生成一次报告
            let current_time = DWT::cycle_count();
            let elapsed_cycles = current_time.wrapping_sub(last_report_time);
            let elapsed_seconds = elapsed_cycles as f32 / 168_000_000.0;

            if elapsed_seconds >= 1.0 {
                let stats = spectrum_analyzer.get_statistics();
                let peaks = spectrum_analyzer.get_peaks();

                rtt_target::rprintln!("=== 频谱分析报告 ===");
                rtt_target::rprintln!("帧数: {} | 帧率: {:.1} FPS", frame_counter, frame_counter as f32 / elapsed_seconds);
                rtt_target::rprintln!("峰值频率: {:.1} Hz | 峰值幅度: {:.3}", stats.peak_frequency, stats.peak_magnitude);
                rtt_target::rprintln!("质心频率: {:.1} Hz | 带宽: {:.1} Hz", stats.centroid_frequency, stats.bandwidth);
                rtt_target::rprintln!("信噪比: {:.1} dB | 总谐波失真: {:.2}%", stats.snr, stats.thd);
                rtt_target::rprintln!("总功率: {:.6} | 检测到峰值: {}", stats.total_power, peaks.len());

                // 显示前5个峰值
                rtt_target::rprintln!("主要峰值:");
                for (i, peak) in peaks.iter().take(5).enumerate() {
                    rtt_target::rprintln!("  {}. {:.1} Hz: {:.3} ({:.1} dB)", 
                        i + 1, 
                        peak.frequency, 
                        peak.magnitude,
                        display.linear_to_db(peak.magnitude)
                    );
                }

                frame_counter = 0;
                last_report_time = current_time;
            }
        }

        // 短暂延时以避免过度占用CPU
        asm::delay(1000);
    }
}