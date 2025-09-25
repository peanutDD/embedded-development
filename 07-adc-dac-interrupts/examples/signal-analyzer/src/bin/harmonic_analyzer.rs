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
use signal_analyzer::{HarmonicAnalyzer, SpectrumAnalyzer, adc_to_voltage};
use micromath::F32Ext;

const HARMONIC_SIZE: usize = 512;
const SAMPLE_RATE: f32 = 20000.0; // 20kHz采样率

// 全局谐波缓冲区
static HARMONIC_BUFFER: Mutex<RefCell<[f32; HARMONIC_SIZE]>> = 
    Mutex::new(RefCell::new([0.0; HARMONIC_SIZE]));
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
    analysis_timer.start(2.Hz()).unwrap(); // 2Hz分析频率
    
    // 配置状态输出定时器
    let mut status_timer = Timer::new(dp.TIM4, &clocks).counter_hz();
    status_timer.start(0.5.Hz()).unwrap(); // 0.5Hz状态输出
    
    // 配置信号切换定时器
    let mut signal_switch_timer = Timer::new(dp.TIM5, &clocks).counter_hz();
    signal_switch_timer.start(0.1.Hz()).unwrap(); // 每10秒切换信号
    
    // 启用定时器中断
    cp.NVIC.enable(Interrupt::TIM2);
    
    // 创建谐波分析器
    let mut harmonic_analyzer = HarmonicAnalyzer::<HARMONIC_SIZE>::new(SAMPLE_RATE);
    harmonic_analyzer.set_harmonic_threshold(0.01); // 1%阈值
    harmonic_analyzer.set_max_harmonics(20); // 最多20个谐波
    
    let spectrum_analyzer = SpectrumAnalyzer::<HARMONIC_SIZE>::new(SAMPLE_RATE);
    
    writeln!(serial, "谐波分析器示例启动").ok();
    writeln!(serial, "缓冲区大小: {}", HARMONIC_SIZE).ok();
    writeln!(serial, "采样率: {:.0} Hz", SAMPLE_RATE).ok();
    writeln!(serial, "分析频率范围: 0 - {:.0} Hz", SAMPLE_RATE / 2.0).ok();
    writeln!(serial, "最大谐波数: 20").ok();
    writeln!(serial, "谐波阈值: 1%").ok();
    writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
    writeln!(serial, "").ok();
    
    let mut sample_count = 0u32;
    let mut analysis_count = 0u32;
    let mut current_signal_type = 0usize;
    let vref = 3.3f32;
    
    // 谐波分析统计
    let mut harmonic_stats = HarmonicAnalysisStatistics::new();
    
    loop {
        // ADC采样
        if sample_timer.wait().is_ok() {
            sample_count += 1;
            
            // 模拟ADC读取
            let adc_value = simulate_harmonic_signal(sample_count, current_signal_type);
            let voltage = adc_to_voltage(adc_value, vref, 12);
            
            // 存储到谐波缓冲区
            let buffer_full = free(|cs| {
                let mut buffer = HARMONIC_BUFFER.borrow(cs).borrow_mut();
                unsafe {
                    buffer[BUFFER_INDEX] = voltage;
                    BUFFER_INDEX += 1;
                    
                    if BUFFER_INDEX >= HARMONIC_SIZE {
                        BUFFER_INDEX = 0;
                        BUFFER_READY = true;
                        true
                    } else {
                        false
                    }
                }
            });
            
            if buffer_full {
                harmonic_stats.record_buffer_fill();
            }
        }
        
        // 谐波分析
        if analysis_timer.wait().is_ok() {
            analysis_count += 1;
            
            let analysis_ready = unsafe { BUFFER_READY };
            if analysis_ready {
                // 复制缓冲区进行分析
                let mut analysis_buffer = [0.0f32; HARMONIC_SIZE];
                free(|cs| {
                    let buffer = HARMONIC_BUFFER.borrow(cs).borrow();
                    analysis_buffer.copy_from_slice(&*buffer);
                });
                
                // 执行谐波分析
                perform_harmonic_analysis(&mut serial, &harmonic_analyzer, &spectrum_analyzer,
                                        &mut analysis_buffer, current_signal_type, 
                                        analysis_count, &mut harmonic_stats);
                
                unsafe { BUFFER_READY = false; }
            }
        }
        
        // 切换信号类型
        if signal_switch_timer.wait().is_ok() {
            current_signal_type = (current_signal_type + 1) % 5;
            
            let signal_name = match current_signal_type {
                0 => "纯正弦波",
                1 => "方波 (丰富谐波)",
                2 => "锯齿波 (奇偶谐波)",
                3 => "三角波 (奇次谐波)",
                4 => "失真正弦波",
                _ => "未知",
            };
            
            writeln!(serial, "\n>>> 切换到: {} <<<", signal_name).ok();
            harmonic_stats.record_signal_switch(current_signal_type);
        }
        
        // 输出系统状态
        if status_timer.wait().is_ok() {
            output_harmonic_status(&mut serial, sample_count, analysis_count, 
                                 current_signal_type, &harmonic_stats);
        }
    }
}

/// 谐波分析统计
struct HarmonicAnalysisStatistics {
    total_analyses: u32,
    buffer_fills: u32,
    signal_switch_count: u32,
    signal_usage_count: [u32; 5],
    harmonic_distribution: [HarmonicDistribution; 5],
    thd_history: [f32; 10],
    thd_index: usize,
}

#[derive(Debug, Clone, Copy)]
struct HarmonicDistribution {
    avg_fundamental_power: f32,
    avg_second_harmonic: f32,
    avg_third_harmonic: f32,
    avg_total_harmonics: f32,
    avg_thd: f32,
    max_harmonic_order: u32,
    harmonic_pattern: [f32; 10], // 前10个谐波的平均功率
}

impl HarmonicDistribution {
    fn new() -> Self {
        Self {
            avg_fundamental_power: 0.0,
            avg_second_harmonic: 0.0,
            avg_third_harmonic: 0.0,
            avg_total_harmonics: 0.0,
            avg_thd: 0.0,
            max_harmonic_order: 0,
            harmonic_pattern: [0.0; 10],
        }
    }
}

impl HarmonicAnalysisStatistics {
    fn new() -> Self {
        Self {
            total_analyses: 0,
            buffer_fills: 0,
            signal_switch_count: 0,
            signal_usage_count: [0; 5],
            harmonic_distribution: [HarmonicDistribution::new(); 5],
            thd_history: [0.0; 10],
            thd_index: 0,
        }
    }
    
    fn record_buffer_fill(&mut self) {
        self.buffer_fills += 1;
    }
    
    fn record_signal_switch(&mut self, signal_type: usize) {
        self.signal_switch_count += 1;
        if signal_type < 5 {
            self.signal_usage_count[signal_type] += 1;
        }
    }
    
    fn update_harmonic_distribution(
        &mut self, 
        signal_type: usize, 
        result: &signal_analyzer::HarmonicResult<HARMONIC_SIZE>
    ) {
        if signal_type >= 5 {
            return;
        }
        
        self.total_analyses += 1;
        
        let distribution = &mut self.harmonic_distribution[signal_type];
        let alpha = 0.1f32;
        
        // 更新基频功率
        distribution.avg_fundamental_power = distribution.avg_fundamental_power * (1.0 - alpha) + 
                                           result.fundamental_power * alpha;
        
        // 更新THD
        distribution.avg_thd = distribution.avg_thd * (1.0 - alpha) + result.thd * alpha;
        
        // 更新THD历史
        self.thd_history[self.thd_index] = result.thd;
        self.thd_index = (self.thd_index + 1) % self.thd_history.len();
        
        // 更新谐波模式
        for (i, harmonic) in result.harmonics.iter().take(10).enumerate() {
            distribution.harmonic_pattern[i] = distribution.harmonic_pattern[i] * (1.0 - alpha) + 
                                             harmonic.power * alpha;
        }
        
        // 更新最大谐波阶数
        if !result.harmonics.is_empty() {
            let max_order = result.harmonics.iter()
                .map(|h| h.order)
                .max()
                .unwrap_or(0);
            distribution.max_harmonic_order = distribution.max_harmonic_order.max(max_order);
        }
        
        // 更新特定谐波
        if result.harmonics.len() > 0 {
            distribution.avg_second_harmonic = result.harmonics.iter()
                .find(|h| h.order == 2)
                .map(|h| h.power)
                .unwrap_or(0.0);
                
            distribution.avg_third_harmonic = result.harmonics.iter()
                .find(|h| h.order == 3)
                .map(|h| h.power)
                .unwrap_or(0.0);
        }
        
        // 计算总谐波功率
        let total_harmonic_power: f32 = result.harmonics.iter()
            .filter(|h| h.order > 1)
            .map(|h| h.power)
            .sum();
        distribution.avg_total_harmonics = distribution.avg_total_harmonics * (1.0 - alpha) + 
                                         total_harmonic_power * alpha;
    }
    
    fn get_thd_trend(&self) -> f32 {
        if self.thd_history.iter().all(|&x| x == 0.0) {
            return 0.0;
        }
        
        let recent_avg = self.thd_history[..5].iter().sum::<f32>() / 5.0;
        let older_avg = self.thd_history[5..].iter().sum::<f32>() / 5.0;
        
        recent_avg - older_avg
    }
}

/// 执行谐波分析
fn perform_harmonic_analysis(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    harmonic_analyzer: &HarmonicAnalyzer<HARMONIC_SIZE>,
    spectrum_analyzer: &SpectrumAnalyzer<HARMONIC_SIZE>,
    signal: &mut [f32; HARMONIC_SIZE],
    signal_type: usize,
    analysis_count: u32,
    stats: &mut HarmonicAnalysisStatistics
) {
    // 执行谐波分析
    let harmonic_result = harmonic_analyzer.analyze(signal);
    
    // 执行频谱分析以获得更多信息
    let spectrum_result = spectrum_analyzer.analyze(signal);
    
    // 更新统计信息
    stats.update_harmonic_distribution(signal_type, &harmonic_result);
    
    // 输出分析结果
    if analysis_count % 2 == 0 { // 每2次分析输出一次详细结果
        output_detailed_harmonic_analysis(serial, signal_type, analysis_count, 
                                        &harmonic_result, &spectrum_result);
    }
    
    // 分析谐波特征
    analyze_harmonic_characteristics(serial, signal_type, &harmonic_result, analysis_count);
    
    // 谐波质量评估
    if analysis_count % 4 == 0 { // 每4次分析进行一次质量评估
        assess_harmonic_quality(serial, signal_type, &harmonic_result, &spectrum_result, 
                               analysis_count);
    }
    
    // 谐波异常检测
    detect_harmonic_anomalies(serial, &harmonic_result, analysis_count);
}

/// 输出详细谐波分析结果
fn output_detailed_harmonic_analysis(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    signal_type: usize,
    analysis_count: u32,
    harmonic_result: &signal_analyzer::HarmonicResult<HARMONIC_SIZE>,
    spectrum_result: &signal_analyzer::SpectrumResult<HARMONIC_SIZE>
) {
    let signal_name = get_signal_name(signal_type);
    
    writeln!(serial, "\n=== 谐波分析 #{} - {} ===", analysis_count, signal_name).ok();
    
    // 基本信息
    writeln!(serial, "基本信息:").ok();
    writeln!(serial, "  基频: {:.1} Hz", harmonic_result.fundamental_frequency).ok();
    writeln!(serial, "  基频功率: {:.6}", harmonic_result.fundamental_power).ok();
    writeln!(serial, "  总谐波失真 (THD): {:.2}%", harmonic_result.thd).ok();
    writeln!(serial, "  谐波数量: {}", harmonic_result.harmonics.len()).ok();
    writeln!(serial, "  信噪比: {:.1} dB", spectrum_result.snr).ok();
    
    // 谐波详情
    writeln!(serial, "谐波分量:").ok();
    for (i, harmonic) in harmonic_result.harmonics.iter().take(10).enumerate() {
        let percentage = if harmonic_result.fundamental_power > 0.0 {
            (harmonic.power / harmonic_result.fundamental_power) * 100.0
        } else {
            0.0
        };
        
        writeln!(serial, "  H{}: {:.1} Hz, 功率: {:.6} ({:.1}%)", 
                harmonic.order, harmonic.frequency, harmonic.power, percentage).ok();
    }
    
    // 谐波分布分析
    analyze_harmonic_distribution(serial, &harmonic_result);
    
    writeln!(serial, "").ok();
}

/// 分析谐波特征
fn analyze_harmonic_characteristics(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    signal_type: usize,
    result: &signal_analyzer::HarmonicResult<HARMONIC_SIZE>,
    analysis_count: u32
) {
    match signal_type {
        0 => { // 纯正弦波
            if result.thd < 1.0 {
                writeln!(serial, "纯正弦波质量优秀: THD {:.2}% (分析#{})", 
                        result.thd, analysis_count).ok();
            } else if result.thd < 5.0 {
                writeln!(serial, "纯正弦波质量良好: THD {:.2}% (分析#{})", 
                        result.thd, analysis_count).ok();
            } else {
                writeln!(serial, "纯正弦波质量较差: THD {:.2}% (分析#{})", 
                        result.thd, analysis_count).ok();
            }
        }
        1 => { // 方波
            let expected_harmonics = count_odd_harmonics(&result.harmonics);
            writeln!(serial, "方波特征: {} 个奇次谐波, THD {:.1}% (分析#{})", 
                    expected_harmonics, result.thd, analysis_count).ok();
            
            // 检查方波的理论谐波比例
            check_square_wave_harmonics(serial, &result.harmonics);
        }
        2 => { // 锯齿波
            let total_harmonics = result.harmonics.len();
            writeln!(serial, "锯齿波特征: {} 个谐波, THD {:.1}% (分析#{})", 
                    total_harmonics, result.thd, analysis_count).ok();
            
            // 检查锯齿波的谐波衰减
            check_sawtooth_harmonics(serial, &result.harmonics);
        }
        3 => { // 三角波
            let odd_harmonics = count_odd_harmonics(&result.harmonics);
            writeln!(serial, "三角波特征: {} 个奇次谐波, THD {:.1}% (分析#{})", 
                    odd_harmonics, result.thd, analysis_count).ok();
            
            // 检查三角波的谐波衰减
            check_triangle_wave_harmonics(serial, &result.harmonics);
        }
        4 => { // 失真正弦波
            writeln!(serial, "失真正弦波: THD {:.1}%, {} 个谐波 (分析#{})", 
                    result.thd, result.harmonics.len(), analysis_count).ok();
            
            // 分析失真类型
            analyze_distortion_type(serial, &result.harmonics);
        }
        _ => {}
    }
}

/// 分析谐波分布
fn analyze_harmonic_distribution(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    result: &signal_analyzer::HarmonicResult<HARMONIC_SIZE>
) {
    if result.harmonics.is_empty() {
        return;
    }
    
    // 计算奇偶谐波比例
    let odd_power: f32 = result.harmonics.iter()
        .filter(|h| h.order % 2 == 1 && h.order > 1)
        .map(|h| h.power)
        .sum();
    
    let even_power: f32 = result.harmonics.iter()
        .filter(|h| h.order % 2 == 0)
        .map(|h| h.power)
        .sum();
    
    let total_harmonic_power = odd_power + even_power;
    
    if total_harmonic_power > 0.0 {
        let odd_percentage = (odd_power / total_harmonic_power) * 100.0;
        let even_percentage = (even_power / total_harmonic_power) * 100.0;
        
        writeln!(serial, "谐波分布:").ok();
        writeln!(serial, "  奇次谐波: {:.1}%", odd_percentage).ok();
        writeln!(serial, "  偶次谐波: {:.1}%", even_percentage).ok();
    }
    
    // 分析谐波衰减
    if result.harmonics.len() >= 3 {
        let h2_power = result.harmonics.iter()
            .find(|h| h.order == 2)
            .map(|h| h.power)
            .unwrap_or(0.0);
        
        let h3_power = result.harmonics.iter()
            .find(|h| h.order == 3)
            .map(|h| h.power)
            .unwrap_or(0.0);
        
        if result.fundamental_power > 0.0 {
            let h2_ratio = (h2_power / result.fundamental_power) * 100.0;
            let h3_ratio = (h3_power / result.fundamental_power) * 100.0;
            
            writeln!(serial, "主要谐波比例:").ok();
            writeln!(serial, "  二次谐波: {:.2}%", h2_ratio).ok();
            writeln!(serial, "  三次谐波: {:.2}%", h3_ratio).ok();
        }
    }
}

/// 计算奇次谐波数量
fn count_odd_harmonics(harmonics: &heapless::Vec<signal_analyzer::Harmonic, HARMONIC_SIZE>) -> usize {
    harmonics.iter()
        .filter(|h| h.order % 2 == 1 && h.order > 1)
        .count()
}

/// 检查方波谐波特征
fn check_square_wave_harmonics(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    harmonics: &heapless::Vec<signal_analyzer::Harmonic, HARMONIC_SIZE>
) {
    // 方波理论上只有奇次谐波，且幅度按1/n衰减
    let odd_harmonics: heapless::Vec<_, 10> = harmonics.iter()
        .filter(|h| h.order % 2 == 1 && h.order > 1)
        .take(5)
        .collect();
    
    if odd_harmonics.len() >= 2 {
        writeln!(serial, "方波谐波检查:").ok();
        for harmonic in odd_harmonics.iter() {
            let theoretical_ratio = 1.0 / harmonic.order as f32;
            writeln!(serial, "  H{}: 理论比例 1/{} = {:.3}", 
                    harmonic.order, harmonic.order, theoretical_ratio).ok();
        }
    }
}

/// 检查锯齿波谐波特征
fn check_sawtooth_harmonics(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    harmonics: &heapless::Vec<signal_analyzer::Harmonic, HARMONIC_SIZE>
) {
    // 锯齿波有所有谐波，幅度按1/n衰减
    if harmonics.len() >= 3 {
        writeln!(serial, "锯齿波谐波衰减检查:").ok();
        for (i, harmonic) in harmonics.iter().take(5).enumerate() {
            if harmonic.order > 1 {
                let expected_decay = 1.0 / harmonic.order as f32;
                writeln!(serial, "  H{}: 期望衰减 1/{} = {:.3}", 
                        harmonic.order, harmonic.order, expected_decay).ok();
            }
        }
    }
}

/// 检查三角波谐波特征
fn check_triangle_wave_harmonics(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    harmonics: &heapless::Vec<signal_analyzer::Harmonic, HARMONIC_SIZE>
) {
    // 三角波只有奇次谐波，幅度按1/n²衰减
    let odd_harmonics: heapless::Vec<_, 10> = harmonics.iter()
        .filter(|h| h.order % 2 == 1 && h.order > 1)
        .take(5)
        .collect();
    
    if odd_harmonics.len() >= 2 {
        writeln!(serial, "三角波谐波衰减检查:").ok();
        for harmonic in odd_harmonics.iter() {
            let expected_decay = 1.0 / (harmonic.order * harmonic.order) as f32;
            writeln!(serial, "  H{}: 期望衰减 1/{}² = {:.4}", 
                    harmonic.order, harmonic.order, expected_decay).ok();
        }
    }
}

/// 分析失真类型
fn analyze_distortion_type(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    harmonics: &heapless::Vec<signal_analyzer::Harmonic, HARMONIC_SIZE>
) {
    if harmonics.is_empty() {
        return;
    }
    
    let even_count = harmonics.iter().filter(|h| h.order % 2 == 0).count();
    let odd_count = harmonics.iter().filter(|h| h.order % 2 == 1 && h.order > 1).count();
    
    writeln!(serial, "失真分析:").ok();
    
    if even_count > odd_count {
        writeln!(serial, "  主要为偶次谐波失真 (可能是非对称失真)").ok();
    } else if odd_count > even_count * 2 {
        writeln!(serial, "  主要为奇次谐波失真 (可能是对称失真)").ok();
    } else {
        writeln!(serial, "  混合谐波失真").ok();
    }
    
    // 检查特定谐波
    let h2_present = harmonics.iter().any(|h| h.order == 2);
    let h3_present = harmonics.iter().any(|h| h.order == 3);
    
    if h2_present && h3_present {
        writeln!(serial, "  检测到二次和三次谐波 (可能是放大器失真)").ok();
    } else if h3_present {
        writeln!(serial, "  主要为三次谐波 (可能是软限幅失真)").ok();
    } else if h2_present {
        writeln!(serial, "  主要为二次谐波 (可能是非线性失真)").ok();
    }
}

/// 评估谐波质量
fn assess_harmonic_quality(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    signal_type: usize,
    harmonic_result: &signal_analyzer::HarmonicResult<HARMONIC_SIZE>,
    spectrum_result: &signal_analyzer::SpectrumResult<HARMONIC_SIZE>,
    analysis_count: u32
) {
    let signal_name = get_signal_name(signal_type);
    
    writeln!(serial, "信号质量评估 - {} (分析#{}):", signal_name, analysis_count).ok();
    
    // THD评级
    let thd_grade = if harmonic_result.thd < 1.0 {
        "优秀"
    } else if harmonic_result.thd < 5.0 {
        "良好"
    } else if harmonic_result.thd < 10.0 {
        "一般"
    } else {
        "较差"
    };
    
    writeln!(serial, "  THD评级: {} ({:.2}%)", thd_grade, harmonic_result.thd).ok();
    
    // SNR评级
    let snr_grade = if spectrum_result.snr > 60.0 {
        "优秀"
    } else if spectrum_result.snr > 40.0 {
        "良好"
    } else if spectrum_result.snr > 20.0 {
        "一般"
    } else {
        "较差"
    };
    
    writeln!(serial, "  SNR评级: {} ({:.1} dB)", snr_grade, spectrum_result.snr).ok();
    
    // 谐波丰富度
    let harmonic_richness = if harmonic_result.harmonics.len() > 10 {
        "非常丰富"
    } else if harmonic_result.harmonics.len() > 5 {
        "丰富"
    } else if harmonic_result.harmonics.len() > 2 {
        "中等"
    } else {
        "简单"
    };
    
    writeln!(serial, "  谐波丰富度: {} ({} 个谐波)", 
            harmonic_richness, harmonic_result.harmonics.len()).ok();
    
    // 基频稳定性
    let freq_stability = if (harmonic_result.fundamental_frequency - get_expected_frequency(signal_type)).abs() < 5.0 {
        "稳定"
    } else {
        "不稳定"
    };
    
    writeln!(serial, "  基频稳定性: {} ({:.1} Hz)", 
            freq_stability, harmonic_result.fundamental_frequency).ok();
    
    writeln!(serial, "").ok();
}

/// 检测谐波异常
fn detect_harmonic_anomalies(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    result: &signal_analyzer::HarmonicResult<HARMONIC_SIZE>,
    analysis_count: u32
) {
    let mut anomalies = heapless::Vec::<&str, 10>::new();
    
    // 检查异常高的THD
    if result.thd > 50.0 {
        anomalies.push("THD异常高").ok();
    }
    
    // 检查基频异常
    if result.fundamental_frequency < 50.0 || result.fundamental_frequency > 5000.0 {
        anomalies.push("基频异常").ok();
    }
    
    // 检查谐波数量异常
    if result.harmonics.len() > 15 {
        anomalies.push("谐波数量异常多").ok();
    }
    
    // 检查基频功率异常低
    if result.fundamental_power < 0.001 {
        anomalies.push("基频功率异常低").ok();
    }
    
    // 检查高次谐波异常强
    let high_order_harmonics = result.harmonics.iter()
        .filter(|h| h.order > 10)
        .count();
    
    if high_order_harmonics > 5 {
        anomalies.push("高次谐波异常强").ok();
    }
    
    if !anomalies.is_empty() {
        writeln!(serial, "谐波异常检测 (分析#{}):", analysis_count).ok();
        for anomaly in anomalies.iter() {
            writeln!(serial, "  ⚠️ {}", anomaly).ok();
        }
        writeln!(serial, "").ok();
    }
}

/// 输出谐波状态
fn output_harmonic_status(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    sample_count: u32,
    analysis_count: u32,
    current_signal_type: usize,
    stats: &HarmonicAnalysisStatistics
) {
    let current_signal_name = get_signal_name(current_signal_type);
    
    writeln!(serial, "\n=== 谐波分析系统状态 ===").ok();
    writeln!(serial, "采样数: {}", sample_count).ok();
    writeln!(serial, "分析数: {}", analysis_count).ok();
    writeln!(serial, "缓冲区填充: {}", stats.buffer_fills).ok();
    writeln!(serial, "当前信号: {}", current_signal_name).ok();
    writeln!(serial, "信号切换次数: {}", stats.signal_switch_count).ok();
    
    // 输出各信号使用统计
    writeln!(serial, "信号使用统计:").ok();
    for i in 0..5 {
        let signal_name = get_signal_name(i);
        writeln!(serial, "  {}: {} 次", signal_name, stats.signal_usage_count[i]).ok();
    }
    
    // 输出当前信号的谐波特征
    if current_signal_type < 5 {
        let dist = &stats.harmonic_distribution[current_signal_type];
        writeln!(serial, "当前信号谐波特征:").ok();
        writeln!(serial, "  平均THD: {:.2}%", dist.avg_thd).ok();
        writeln!(serial, "  平均基频功率: {:.6}", dist.avg_fundamental_power).ok();
        writeln!(serial, "  最大谐波阶数: {}", dist.max_harmonic_order).ok();
    }
    
    // THD趋势
    let thd_trend = stats.get_thd_trend();
    let trend_desc = if thd_trend > 1.0 {
        "上升"
    } else if thd_trend < -1.0 {
        "下降"
    } else {
        "稳定"
    };
    
    writeln!(serial, "THD趋势: {} ({:.2}%)", trend_desc, thd_trend).ok();
    writeln!(serial, "").ok();
}

/// 获取信号名称
fn get_signal_name(signal_type: usize) -> &'static str {
    match signal_type {
        0 => "纯正弦波",
        1 => "方波",
        2 => "锯齿波",
        3 => "三角波",
        4 => "失真正弦波",
        _ => "未知信号",
    }
}

/// 获取期望频率
fn get_expected_frequency(signal_type: usize) -> f32 {
    match signal_type {
        0 => 1000.0, // 纯正弦波 1kHz
        1 => 500.0,  // 方波 500Hz
        2 => 800.0,  // 锯齿波 800Hz
        3 => 600.0,  // 三角波 600Hz
        4 => 1000.0, // 失真正弦波 1kHz
        _ => 1000.0,
    }
}

/// 模拟谐波信号
fn simulate_harmonic_signal(sample_count: u32, signal_type: usize) -> u16 {
    let t = sample_count as f32 / SAMPLE_RATE;
    let dc_offset = 2048.0;
    
    let signal = match signal_type {
        0 => { // 纯正弦波
            800.0 * (2.0 * core::f32::consts::PI * 1000.0 * t).sin()
        }
        1 => { // 方波 (富含奇次谐波)
            let fundamental = 600.0 * (2.0 * core::f32::consts::PI * 500.0 * t).sin();
            let h3 = 200.0 * (2.0 * core::f32::consts::PI * 1500.0 * t).sin();
            let h5 = 120.0 * (2.0 * core::f32::consts::PI * 2500.0 * t).sin();
            let h7 = 85.0 * (2.0 * core::f32::consts::PI * 3500.0 * t).sin();
            fundamental + h3 + h5 + h7
        }
        2 => { // 锯齿波 (所有谐波)
            let fundamental = 600.0 * (2.0 * core::f32::consts::PI * 800.0 * t).sin();
            let h2 = 300.0 * (2.0 * core::f32::consts::PI * 1600.0 * t).sin();
            let h3 = 200.0 * (2.0 * core::f32::consts::PI * 2400.0 * t).sin();
            let h4 = 150.0 * (2.0 * core::f32::consts::PI * 3200.0 * t).sin();
            let h5 = 120.0 * (2.0 * core::f32::consts::PI * 4000.0 * t).sin();
            fundamental + h2 + h3 + h4 + h5
        }
        3 => { // 三角波 (奇次谐波，快速衰减)
            let fundamental = 700.0 * (2.0 * core::f32::consts::PI * 600.0 * t).sin();
            let h3 = 77.0 * (2.0 * core::f32::consts::PI * 1800.0 * t).sin(); // 1/9
            let h5 = 28.0 * (2.0 * core::f32::consts::PI * 3000.0 * t).sin(); // 1/25
            let h7 = 14.0 * (2.0 * core::f32::consts::PI * 4200.0 * t).sin(); // 1/49
            fundamental + h3 + h5 + h7
        }
        4 => { // 失真正弦波
            let fundamental = 700.0 * (2.0 * core::f32::consts::PI * 1000.0 * t).sin();
            let h2 = 140.0 * (2.0 * core::f32::consts::PI * 2000.0 * t).sin(); // 20%
            let h3 = 105.0 * (2.0 * core::f32::consts::PI * 3000.0 * t).sin(); // 15%
            let h4 = 70.0 * (2.0 * core::f32::consts::PI * 4000.0 * t).sin();  // 10%
            let h5 = 35.0 * (2.0 * core::f32::consts::PI * 5000.0 * t).sin();  // 5%
            fundamental + h2 + h3 + h4 + h5
        }
        _ => 0.0,
    };
    
    // 添加少量噪声
    let noise = ((sample_count * 13 + 17) % 31) as f32 - 15.0;
    let scaled_noise = noise * 10.0;
    
    let result = dc_offset + signal + scaled_noise;
    result.max(0.0).min(4095.0) as u16
}

/// 定时器中断处理
#[interrupt]
fn TIM2() {
    // 在实际应用中，这里会处理ADC采样中断
}