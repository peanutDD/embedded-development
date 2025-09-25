#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac::{self, interrupt, Interrupt},
    prelude::*,
    adc::{Adc, AdcConfig, SampleTime},
    gpio::Analog,
    timer::{Timer, Event},
    serial::{Serial, Config},
};
use nb::block;
use heapless::{String, Vec};
use core::fmt::Write;
use cortex_m::interrupt::{free, Mutex};
use core::cell::RefCell;
use signal_analyzer::{DigitalFilter, FilterType, SpectrumAnalyzer, adc_to_voltage};
use micromath::F32Ext;

const FILTER_SIZE: usize = 256;
const SAMPLE_RATE: f32 = 16000.0; // 16kHz采样率

// 全局滤波器缓冲区
static FILTER_BUFFER: Mutex<RefCell<[f32; FILTER_SIZE]>> = 
    Mutex::new(RefCell::new([0.0; FILTER_SIZE]));
static mut BUFFER_INDEX: usize = 0;
static mut BUFFER_READY: bool = false;

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    
    // 配置串口
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    let mut serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &clocks,
    ).unwrap();
    
    // 配置ADC引脚
    let adc_pin = gpioa.pa0.into_analog(); // ADC1_IN0
    
    // 配置ADC
    let adc_config = AdcConfig::default()
        .sample_time(SampleTime::Cycles_480);
    let mut adc = Adc::adc1(dp.ADC1, true, adc_config);
    
    // 配置采样定时器
    let mut sample_timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    sample_timer.start((SAMPLE_RATE as u32).Hz()).unwrap();
    
    // 配置分析定时器
    let mut analysis_timer = Timer::new(dp.TIM3, &clocks).counter_hz();
    analysis_timer.start(4.Hz()).unwrap(); // 4Hz分析频率
    
    // 配置状态输出定时器
    let mut status_timer = Timer::new(dp.TIM4, &clocks).counter_hz();
    status_timer.start(1.Hz()).unwrap(); // 1Hz状态输出
    
    // 配置滤波器切换定时器
    let mut filter_switch_timer = Timer::new(dp.TIM5, &clocks).counter_hz();
    filter_switch_timer.start(0.2.Hz()).unwrap(); // 每5秒切换滤波器
    
    // 启用定时器中断
    cp.NVIC.enable(Interrupt::TIM2);
    
    // 创建多种滤波器
    let mut filters = FilterBank::new();
    let mut spectrum_analyzer = SpectrumAnalyzer::<FILTER_SIZE>::new(SAMPLE_RATE);
    
    writeln!(serial, "滤波器分析器示例启动").ok();
    writeln!(serial, "缓冲区大小: {}", FILTER_SIZE).ok();
    writeln!(serial, "采样率: {:.0} Hz", SAMPLE_RATE).ok();
    writeln!(serial, "分析频率范围: 0 - {:.0} Hz", SAMPLE_RATE / 2.0).ok();
    writeln!(serial, "可用滤波器:").ok();
    writeln!(serial, "  1. 低通滤波器 (1kHz)").ok();
    writeln!(serial, "  2. 高通滤波器 (2kHz)").ok();
    writeln!(serial, "  3. 陷波滤波器 (50Hz)").ok();
    writeln!(serial, "  4. 陷波滤波器 (1kHz)").ok();
    writeln!(serial, "  5. 无滤波器 (原始信号)").ok();
    writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
    writeln!(serial, "").ok();
    
    let mut sample_count = 0u32;
    let mut analysis_count = 0u32;
    let mut current_filter_index = 0usize;
    let vref = 3.3f32;
    
    // 滤波器分析统计
    let mut filter_stats = FilterAnalysisStatistics::new();
    
    loop {
        // ADC采样
        if sample_timer.wait().is_ok() {
            sample_count += 1;
            
            // 模拟ADC读取
            let adc_value = simulate_noisy_signal(sample_count);
            let voltage = adc_to_voltage(adc_value, vref, 12);
            
            // 应用当前滤波器
            let filtered_voltage = filters.process(current_filter_index, voltage);
            
            // 存储到滤波器缓冲区
            let buffer_full = free(|cs| {
                let mut buffer = FILTER_BUFFER.borrow(cs).borrow_mut();
                unsafe {
                    buffer[BUFFER_INDEX] = filtered_voltage;
                    BUFFER_INDEX += 1;
                    
                    if BUFFER_INDEX >= FILTER_SIZE {
                        BUFFER_INDEX = 0;
                        BUFFER_READY = true;
                        true
                    } else {
                        false
                    }
                }
            });
            
            if buffer_full {
                filter_stats.record_buffer_fill();
            }
        }
        
        // 滤波器分析
        if analysis_timer.wait().is_ok() {
            analysis_count += 1;
            
            let analysis_ready = unsafe { BUFFER_READY };
            if analysis_ready {
                // 复制缓冲区进行分析
                let mut analysis_buffer = [0.0f32; FILTER_SIZE];
                free(|cs| {
                    let buffer = FILTER_BUFFER.borrow(cs).borrow();
                    analysis_buffer.copy_from_slice(&*buffer);
                });
                
                // 执行滤波器分析
                perform_filter_analysis(&mut serial, &spectrum_analyzer, &mut analysis_buffer, 
                                      current_filter_index, analysis_count, &mut filter_stats);
                
                unsafe { BUFFER_READY = false; }
            }
        }
        
        // 切换滤波器
        if filter_switch_timer.wait().is_ok() {
            current_filter_index = (current_filter_index + 1) % 5;
            filters.reset_all();
            
            let filter_name = match current_filter_index {
                0 => "低通滤波器 (1kHz)",
                1 => "高通滤波器 (2kHz)",
                2 => "陷波滤波器 (50Hz)",
                3 => "陷波滤波器 (1kHz)",
                4 => "无滤波器",
                _ => "未知",
            };
            
            writeln!(serial, "\n>>> 切换到: {} <<<", filter_name).ok();
            filter_stats.record_filter_switch(current_filter_index);
        }
        
        // 输出系统状态
        if status_timer.wait().is_ok() {
            output_filter_status(&mut serial, sample_count, analysis_count, 
                                current_filter_index, &filter_stats);
        }
    }
}

/// 滤波器组
struct FilterBank {
    lowpass_filter: DigitalFilter,
    highpass_filter: DigitalFilter,
    notch_50hz_filter: DigitalFilter,
    notch_1khz_filter: DigitalFilter,
}

impl FilterBank {
    fn new() -> Self {
        Self {
            lowpass_filter: DigitalFilter::new_butterworth_lowpass(1000.0, SAMPLE_RATE, 2),
            highpass_filter: DigitalFilter::new_butterworth_highpass(2000.0, SAMPLE_RATE, 2),
            notch_50hz_filter: DigitalFilter::new_notch(50.0, SAMPLE_RATE, 10.0),
            notch_1khz_filter: DigitalFilter::new_notch(1000.0, SAMPLE_RATE, 5.0),
        }
    }
    
    fn process(&mut self, filter_index: usize, input: f32) -> f32 {
        match filter_index {
            0 => self.lowpass_filter.process(input),
            1 => self.highpass_filter.process(input),
            2 => self.notch_50hz_filter.process(input),
            3 => self.notch_1khz_filter.process(input),
            4 => input, // 无滤波器
            _ => input,
        }
    }
    
    fn reset_all(&mut self) {
        self.lowpass_filter.reset();
        self.highpass_filter.reset();
        self.notch_50hz_filter.reset();
        self.notch_1khz_filter.reset();
    }
    
    fn get_filter_name(index: usize) -> &'static str {
        match index {
            0 => "低通 (1kHz)",
            1 => "高通 (2kHz)",
            2 => "陷波 (50Hz)",
            3 => "陷波 (1kHz)",
            4 => "无滤波器",
            _ => "未知",
        }
    }
}

/// 滤波器分析统计
struct FilterAnalysisStatistics {
    total_analyses: u32,
    buffer_fills: u32,
    filter_switch_count: u32,
    filter_usage_count: [u32; 5],
    filter_performance: [FilterPerformance; 5],
}

#[derive(Debug, Clone, Copy)]
struct FilterPerformance {
    avg_attenuation: f32,
    avg_snr_improvement: f32,
    avg_thd_reduction: f32,
    noise_reduction_factor: f32,
    frequency_response_flatness: f32,
}

impl FilterPerformance {
    fn new() -> Self {
        Self {
            avg_attenuation: 0.0,
            avg_snr_improvement: 0.0,
            avg_thd_reduction: 0.0,
            noise_reduction_factor: 0.0,
            frequency_response_flatness: 0.0,
        }
    }
}

impl FilterAnalysisStatistics {
    fn new() -> Self {
        Self {
            total_analyses: 0,
            buffer_fills: 0,
            filter_switch_count: 0,
            filter_usage_count: [0; 5],
            filter_performance: [FilterPerformance::new(); 5],
        }
    }
    
    fn record_buffer_fill(&mut self) {
        self.buffer_fills += 1;
    }
    
    fn record_filter_switch(&mut self, filter_index: usize) {
        self.filter_switch_count += 1;
        if filter_index < 5 {
            self.filter_usage_count[filter_index] += 1;
        }
    }
    
    fn update_filter_performance(
        &mut self, 
        filter_index: usize, 
        result: &signal_analyzer::SpectrumResult<FILTER_SIZE>,
        original_power: f32,
        original_snr: f32
    ) {
        if filter_index >= 5 {
            return;
        }
        
        self.total_analyses += 1;
        
        let performance = &mut self.filter_performance[filter_index];
        let alpha = 0.1f32;
        
        // 计算衰减
        let attenuation = if original_power > 0.0 {
            -20.0 * (result.total_power / original_power).log10()
        } else {
            0.0
        };
        performance.avg_attenuation = performance.avg_attenuation * (1.0 - alpha) + attenuation * alpha;
        
        // 计算SNR改善
        let snr_improvement = result.snr - original_snr;
        performance.avg_snr_improvement = performance.avg_snr_improvement * (1.0 - alpha) + 
                                        snr_improvement * alpha;
        
        // 计算THD减少
        let thd_reduction = if result.thd < 50.0 { // 合理的THD值
            result.thd
        } else {
            0.0
        };
        performance.avg_thd_reduction = performance.avg_thd_reduction * (1.0 - alpha) + 
                                      thd_reduction * alpha;
        
        // 计算噪声减少因子
        let noise_reduction = if original_power > result.total_power {
            (original_power - result.total_power) / original_power
        } else {
            0.0
        };
        performance.noise_reduction_factor = performance.noise_reduction_factor * (1.0 - alpha) + 
                                           noise_reduction * alpha;
    }
}

/// 执行滤波器分析
fn perform_filter_analysis(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    analyzer: &SpectrumAnalyzer<FILTER_SIZE>,
    signal: &mut [f32; FILTER_SIZE],
    filter_index: usize,
    analysis_count: u32,
    stats: &mut FilterAnalysisStatistics
) {
    // 计算原始信号统计（用于比较）
    let original_power = calculate_signal_power(signal);
    let original_snr = estimate_signal_snr(signal);
    
    // 执行频谱分析
    let result = analyzer.analyze(signal);
    
    // 更新滤波器性能统计
    stats.update_filter_performance(filter_index, &result, original_power, original_snr);
    
    // 输出分析结果
    if analysis_count % 4 == 0 { // 每4次分析输出一次详细结果
        output_detailed_filter_analysis(serial, filter_index, analysis_count, &result, 
                                      original_power, original_snr);
    }
    
    // 分析滤波器效果
    analyze_filter_effectiveness(serial, filter_index, &result, original_power, 
                                original_snr, analysis_count);
    
    // 频率响应分析
    if analysis_count % 8 == 0 { // 每8次分析进行一次频率响应分析
        analyze_frequency_response(serial, filter_index, &result, analysis_count);
    }
}

/// 计算信号功率
fn calculate_signal_power(signal: &[f32; FILTER_SIZE]) -> f32 {
    signal.iter().map(|&x| x * x).sum::<f32>() / signal.len() as f32
}

/// 估算信号SNR
fn estimate_signal_snr(signal: &[f32; FILTER_SIZE]) -> f32 {
    let mean = signal.iter().sum::<f32>() / signal.len() as f32;
    let signal_power = mean * mean;
    
    let noise_power = signal.iter()
        .map(|&x| {
            let diff = x - mean;
            diff * diff
        })
        .sum::<f32>() / signal.len() as f32;
    
    if noise_power > 0.0 {
        10.0 * (signal_power / noise_power).log10()
    } else {
        100.0
    }
}

/// 输出详细滤波器分析结果
fn output_detailed_filter_analysis(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    filter_index: usize,
    analysis_count: u32,
    result: &signal_analyzer::SpectrumResult<FILTER_SIZE>,
    original_power: f32,
    original_snr: f32
) {
    let filter_name = FilterBank::get_filter_name(filter_index);
    
    writeln!(serial, "\n=== 滤波器分析 #{} - {} ===", analysis_count, filter_name).ok();
    
    // 滤波器效果
    let power_reduction = if original_power > 0.0 {
        (1.0 - result.total_power / original_power) * 100.0
    } else {
        0.0
    };
    
    let snr_improvement = result.snr - original_snr;
    
    writeln!(serial, "滤波器效果:").ok();
    writeln!(serial, "  功率减少: {:.1}%", power_reduction).ok();
    writeln!(serial, "  SNR改善: {:.1} dB", snr_improvement).ok();
    writeln!(serial, "  当前SNR: {:.1} dB", result.snr).ok();
    writeln!(serial, "  当前THD: {:.2}%", result.thd).ok();
    
    // 频谱信息
    writeln!(serial, "频谱信息:").ok();
    writeln!(serial, "  总功率: {:.6}", result.total_power).ok();
    writeln!(serial, "  基频: {:.1} Hz", result.fundamental_frequency).ok();
    writeln!(serial, "  峰值数量: {}", result.peaks.len()).ok();
    
    // 主要频率分量
    writeln!(serial, "主要频率分量:").ok();
    for (i, peak) in result.peaks.iter().take(5).enumerate() {
        writeln!(serial, "  {}: {:.1} Hz ({:.4})", 
                i + 1, peak.frequency, peak.magnitude).ok();
    }
    
    writeln!(serial, "").ok();
}

/// 分析滤波器有效性
fn analyze_filter_effectiveness(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    filter_index: usize,
    result: &signal_analyzer::SpectrumResult<FILTER_SIZE>,
    original_power: f32,
    original_snr: f32,
    analysis_count: u32
) {
    match filter_index {
        0 => { // 低通滤波器
            // 检查高频衰减
            let high_freq_power = calculate_band_power(&result.power_spectrum, 
                                                     &result.frequency_bins, 1000.0, 8000.0);
            let total_power = result.total_power;
            let high_freq_ratio = if total_power > 0.0 {
                high_freq_power / total_power * 100.0
            } else {
                0.0
            };
            
            if high_freq_ratio < 10.0 {
                writeln!(serial, "低通滤波器效果良好: 高频能量占比 {:.1}% (分析#{})", 
                        high_freq_ratio, analysis_count).ok();
            } else {
                writeln!(serial, "低通滤波器效果不佳: 高频能量占比 {:.1}% (分析#{})", 
                        high_freq_ratio, analysis_count).ok();
            }
        }
        1 => { // 高通滤波器
            // 检查低频衰减
            let low_freq_power = calculate_band_power(&result.power_spectrum, 
                                                    &result.frequency_bins, 0.0, 2000.0);
            let total_power = result.total_power;
            let low_freq_ratio = if total_power > 0.0 {
                low_freq_power / total_power * 100.0
            } else {
                0.0
            };
            
            if low_freq_ratio < 20.0 {
                writeln!(serial, "高通滤波器效果良好: 低频能量占比 {:.1}% (分析#{})", 
                        low_freq_ratio, analysis_count).ok();
            } else {
                writeln!(serial, "高通滤波器效果不佳: 低频能量占比 {:.1}% (分析#{})", 
                        low_freq_ratio, analysis_count).ok();
            }
        }
        2 => { // 50Hz陷波滤波器
            // 检查50Hz附近的衰减
            let notch_power = calculate_band_power(&result.power_spectrum, 
                                                 &result.frequency_bins, 45.0, 55.0);
            let total_power = result.total_power;
            let notch_ratio = if total_power > 0.0 {
                notch_power / total_power * 100.0
            } else {
                0.0
            };
            
            if notch_ratio < 5.0 {
                writeln!(serial, "50Hz陷波滤波器效果良好: 50Hz能量占比 {:.1}% (分析#{})", 
                        notch_ratio, analysis_count).ok();
            }
        }
        3 => { // 1kHz陷波滤波器
            // 检查1kHz附近的衰减
            let notch_power = calculate_band_power(&result.power_spectrum, 
                                                 &result.frequency_bins, 950.0, 1050.0);
            let total_power = result.total_power;
            let notch_ratio = if total_power > 0.0 {
                notch_power / total_power * 100.0
            } else {
                0.0
            };
            
            if notch_ratio < 10.0 {
                writeln!(serial, "1kHz陷波滤波器效果良好: 1kHz能量占比 {:.1}% (分析#{})", 
                        notch_ratio, analysis_count).ok();
            }
        }
        4 => { // 无滤波器
            // 分析原始信号特征
            if result.peaks.len() > 5 {
                writeln!(serial, "原始信号复杂: {} 个频率峰值 (分析#{})", 
                        result.peaks.len(), analysis_count).ok();
            }
        }
        _ => {}
    }
}

/// 计算频段功率
fn calculate_band_power(
    power_spectrum: &[f32; FILTER_SIZE], 
    frequency_bins: &[f32; FILTER_SIZE],
    low_freq: f32, 
    high_freq: f32
) -> f32 {
    let mut band_power = 0.0f32;
    
    for (i, &power) in power_spectrum.iter().enumerate() {
        if i < frequency_bins.len() {
            let freq = frequency_bins[i];
            if freq >= low_freq && freq <= high_freq {
                band_power += power;
            }
        }
    }
    
    band_power
}

/// 分析频率响应
fn analyze_frequency_response(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    filter_index: usize,
    result: &signal_analyzer::SpectrumResult<FILTER_SIZE>,
    analysis_count: u32
) {
    let filter_name = FilterBank::get_filter_name(filter_index);
    
    writeln!(serial, "频率响应分析 - {} (分析#{}):", filter_name, analysis_count).ok();
    
    // 分析不同频段的响应
    let bands = [
        ("0-500Hz", 0.0, 500.0),
        ("500Hz-1kHz", 500.0, 1000.0),
        ("1-2kHz", 1000.0, 2000.0),
        ("2-4kHz", 2000.0, 4000.0),
        ("4-8kHz", 4000.0, 8000.0),
    ];
    
    let total_power = result.total_power;
    
    for (band_name, low_freq, high_freq) in bands.iter() {
        let band_power = calculate_band_power(&result.power_spectrum, 
                                            &result.frequency_bins, *low_freq, *high_freq);
        let band_percentage = if total_power > 0.0 {
            band_power / total_power * 100.0
        } else {
            0.0
        };
        
        writeln!(serial, "  {}: {:.1}%", band_name, band_percentage).ok();
    }
    
    writeln!(serial, "").ok();
}

/// 输出滤波器状态
fn output_filter_status(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    sample_count: u32,
    analysis_count: u32,
    current_filter_index: usize,
    stats: &FilterAnalysisStatistics
) {
    let current_filter_name = FilterBank::get_filter_name(current_filter_index);
    
    writeln!(serial, "\n=== 滤波器系统状态 ===").ok();
    writeln!(serial, "采样数: {}", sample_count).ok();
    writeln!(serial, "分析数: {}", analysis_count).ok();
    writeln!(serial, "缓冲区填充: {}", stats.buffer_fills).ok();
    writeln!(serial, "当前滤波器: {}", current_filter_name).ok();
    writeln!(serial, "滤波器切换次数: {}", stats.filter_switch_count).ok();
    
    // 输出各滤波器使用统计
    writeln!(serial, "滤波器使用统计:").ok();
    for i in 0..5 {
        let filter_name = FilterBank::get_filter_name(i);
        writeln!(serial, "  {}: {} 次", filter_name, stats.filter_usage_count[i]).ok();
    }
    
    // 输出当前滤波器性能
    if current_filter_index < 5 {
        let perf = &stats.filter_performance[current_filter_index];
        writeln!(serial, "当前滤波器性能:").ok();
        writeln!(serial, "  平均衰减: {:.1} dB", perf.avg_attenuation).ok();
        writeln!(serial, "  平均SNR改善: {:.1} dB", perf.avg_snr_improvement).ok();
        writeln!(serial, "  噪声减少因子: {:.3}", perf.noise_reduction_factor).ok();
    }
    
    writeln!(serial, "").ok();
}

/// 模拟含噪声信号
fn simulate_noisy_signal(sample_count: u32) -> u16 {
    let t = sample_count as f32 / SAMPLE_RATE;
    let dc_offset = 2048.0;
    
    // 基础信号
    let signal = 
        // 主信号 1kHz
        600.0 * (2.0 * core::f32::consts::PI * 1000.0 * t).sin() +
        // 低频信号 200Hz
        300.0 * (2.0 * core::f32::consts::PI * 200.0 * t).sin() +
        // 高频信号 3kHz
        200.0 * (2.0 * core::f32::consts::PI * 3000.0 * t).sin();
    
    // 50Hz工频干扰
    let power_line_noise = 150.0 * (2.0 * core::f32::consts::PI * 50.0 * t).sin();
    
    // 高频噪声
    let high_freq_noise = 100.0 * (2.0 * core::f32::consts::PI * 5000.0 * t).sin() * 
                         ((sample_count * 3) % 7) as f32 / 7.0;
    
    // 白噪声
    let white_noise = ((sample_count * 17 + 29) % 61) as f32 - 30.0;
    let scaled_white_noise = white_noise * 25.0;
    
    // 脉冲噪声
    let impulse_noise = if (sample_count % 1000) < 10 {
        200.0 * ((sample_count % 10) as f32 / 10.0 - 0.5)
    } else {
        0.0
    };
    
    let result = dc_offset + signal + power_line_noise + high_freq_noise + 
                scaled_white_noise + impulse_noise;
    result.max(0.0).min(4095.0) as u16
}

/// 定时器中断处理
#[interrupt]
fn TIM2() {
    // 在实际应用中，这里会处理ADC采样中断
}