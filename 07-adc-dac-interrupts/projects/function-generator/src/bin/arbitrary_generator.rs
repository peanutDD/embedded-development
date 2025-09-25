#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    dac::{Dac, DacPin, C1},
    gpio::{Analog, Pin},
    serial::{config::Config, Serial},
    timer::{Timer, Event},
};
use nb::block;
use function_generator::{
    WaveformGenerator, GeneratorConfig, WaveformType, ArbitraryWaveform,
    MultiChannelGenerator, GeneratorStatistics, voltage_to_dac, dac_to_voltage
};

// 任意波形发生器配置常量
const VREF_MV: u16 = 3300;
const SAMPLE_RATE: u32 = 400000; // 400kHz
const BUFFER_SIZE: usize = 4000;
const WAVEFORM_SIZE: usize = 1000;

// 预定义波形库
struct WaveformLibrary {
    current_waveform: usize,
    waveform_count: usize,
    waveform_names: [&'static str; 10],
}

impl WaveformLibrary {
    fn new() -> Self {
        Self {
            current_waveform: 0,
            waveform_count: 10,
            waveform_names: [
                "心电图波形 (ECG)",
                "音频波形 (Audio)",
                "脉冲串 (Pulse Train)",
                "调制包络 (AM Envelope)",
                "啁啾信号 (Chirp)",
                "高斯脉冲 (Gaussian)",
                "指数衰减 (Exponential)",
                "阶跃响应 (Step Response)",
                "复合谐波 (Multi-Harmonic)",
                "随机游走 (Random Walk)",
            ],
        }
    }

    fn generate_waveform(&self, waveform_id: usize, buffer: &mut [u16]) {
        let len = buffer.len();
        
        match waveform_id {
            0 => self.generate_ecg_waveform(buffer),
            1 => self.generate_audio_waveform(buffer),
            2 => self.generate_pulse_train(buffer),
            3 => self.generate_am_envelope(buffer),
            4 => self.generate_chirp_signal(buffer),
            5 => self.generate_gaussian_pulse(buffer),
            6 => self.generate_exponential_decay(buffer),
            7 => self.generate_step_response(buffer),
            8 => self.generate_multi_harmonic(buffer),
            9 => self.generate_random_walk(buffer),
            _ => self.generate_sine_wave(buffer),
        }
    }

    fn generate_ecg_waveform(&self, buffer: &mut [u16]) {
        // 模拟心电图波形
        let len = buffer.len();
        for (i, sample) in buffer.iter_mut().enumerate() {
            let t = (i as f32) / (len as f32) * 2.0 * 3.14159;
            
            // P波、QRS复合波、T波的组合
            let p_wave = 0.1 * (t * 0.5).sin();
            let qrs_complex = if t > 1.0 && t < 2.0 {
                0.8 * (t * 10.0).sin() * (-((t - 1.5) * 5.0).powi(2)).exp()
            } else {
                0.0
            };
            let t_wave = 0.2 * ((t - 4.0) * 0.8).sin() * (-((t - 4.0) * 2.0).powi(2)).exp();
            
            let ecg_value = p_wave + qrs_complex + t_wave;
            *sample = ((ecg_value + 1.0) * 2047.5) as u16;
        }
    }

    fn generate_audio_waveform(&self, buffer: &mut [u16]) {
        // 模拟音频波形（多频率混合）
        let len = buffer.len();
        for (i, sample) in buffer.iter_mut().enumerate() {
            let t = (i as f32) / (len as f32) * 4.0 * 3.14159;
            
            let fundamental = 0.5 * t.sin();
            let second_harmonic = 0.3 * (2.0 * t).sin();
            let third_harmonic = 0.2 * (3.0 * t).sin();
            let noise = 0.05 * ((t * 17.0).sin() + (t * 23.0).sin());
            
            let audio_value = fundamental + second_harmonic + third_harmonic + noise;
            *sample = ((audio_value + 1.0) * 2047.5) as u16;
        }
    }

    fn generate_pulse_train(&self, buffer: &mut [u16]) {
        // 脉冲串波形
        let len = buffer.len();
        let pulse_width = len / 20; // 5%占空比
        let period = len / 5; // 5个脉冲
        
        for (i, sample) in buffer.iter_mut().enumerate() {
            let position_in_period = i % period;
            
            if position_in_period < pulse_width {
                *sample = 4000; // 高电平
            } else {
                *sample = 500; // 低电平
            }
        }
    }

    fn generate_am_envelope(&self, buffer: &mut [u16]) {
        // AM调制包络
        let len = buffer.len();
        for (i, sample) in buffer.iter_mut().enumerate() {
            let t = (i as f32) / (len as f32) * 4.0 * 3.14159;
            
            let carrier = (t * 10.0).sin();
            let modulation = 0.5 * (t * 0.5).sin();
            let envelope = 1.0 + modulation;
            
            let am_value = carrier * envelope * 0.8;
            *sample = ((am_value + 1.0) * 2047.5) as u16;
        }
    }

    fn generate_chirp_signal(&self, buffer: &mut [u16]) {
        // 啁啾信号（频率扫描）
        let len = buffer.len();
        for (i, sample) in buffer.iter_mut().enumerate() {
            let t = (i as f32) / (len as f32);
            let freq = 1.0 + t * 10.0; // 频率从1到11
            let phase = 2.0 * 3.14159 * freq * t;
            
            let chirp_value = 0.8 * phase.sin();
            *sample = ((chirp_value + 1.0) * 2047.5) as u16;
        }
    }

    fn generate_gaussian_pulse(&self, buffer: &mut [u16]) {
        // 高斯脉冲
        let len = buffer.len();
        let center = len as f32 / 2.0;
        let sigma = len as f32 / 8.0;
        
        for (i, sample) in buffer.iter_mut().enumerate() {
            let x = i as f32 - center;
            let gaussian = (-0.5 * (x / sigma).powi(2)).exp();
            
            *sample = (gaussian * 4095.0) as u16;
        }
    }

    fn generate_exponential_decay(&self, buffer: &mut [u16]) {
        // 指数衰减
        let len = buffer.len();
        for (i, sample) in buffer.iter_mut().enumerate() {
            let t = (i as f32) / (len as f32);
            let decay_constant = 3.0;
            
            let exp_value = (-decay_constant * t).exp();
            *sample = (exp_value * 4095.0) as u16;
        }
    }

    fn generate_step_response(&self, buffer: &mut [u16]) {
        // 阶跃响应
        let len = buffer.len();
        let step_point = len / 4;
        
        for (i, sample) in buffer.iter_mut().enumerate() {
            if i < step_point {
                *sample = 500; // 低电平
            } else {
                let t = (i - step_point) as f32 / (len - step_point) as f32;
                let response = 1.0 - (-5.0 * t).exp();
                *sample = (500.0 + response * 3500.0) as u16;
            }
        }
    }

    fn generate_multi_harmonic(&self, buffer: &mut [u16]) {
        // 复合谐波
        let len = buffer.len();
        for (i, sample) in buffer.iter_mut().enumerate() {
            let t = (i as f32) / (len as f32) * 2.0 * 3.14159;
            
            let mut harmonic_sum = 0.0;
            for n in 1..=7 {
                let amplitude = 1.0 / (n as f32);
                harmonic_sum += amplitude * (n as f32 * t).sin();
            }
            
            *sample = ((harmonic_sum * 0.5 + 1.0) * 2047.5) as u16;
        }
    }

    fn generate_random_walk(&self, buffer: &mut [u16]) {
        // 随机游走
        let len = buffer.len();
        let mut current_value = 2048.0;
        let step_size = 50.0;
        
        for (i, sample) in buffer.iter_mut().enumerate() {
            // 简单的伪随机数生成
            let seed = (i * 1103515245 + 12345) as u32;
            let random = ((seed >> 16) & 0x7fff) as f32 / 32768.0 - 0.5;
            
            current_value += random * step_size;
            current_value = current_value.max(0.0).min(4095.0);
            
            *sample = current_value as u16;
        }
    }

    fn generate_sine_wave(&self, buffer: &mut [u16]) {
        // 默认正弦波
        let len = buffer.len();
        for (i, sample) in buffer.iter_mut().enumerate() {
            let t = (i as f32) / (len as f32) * 2.0 * 3.14159;
            let sine_value = 0.8 * t.sin();
            *sample = ((sine_value + 1.0) * 2047.5) as u16;
        }
    }

    fn get_current_name(&self) -> &'static str {
        self.waveform_names[self.current_waveform]
    }

    fn next_waveform(&mut self) {
        self.current_waveform = (self.current_waveform + 1) % self.waveform_count;
    }
}

// 波形分析器
struct WaveformAnalyzer {
    peak_value: u16,
    min_value: u16,
    rms_value: f32,
    average_value: f32,
    peak_to_peak: u16,
    crest_factor: f32,
    form_factor: f32,
    rise_time: u32,
    fall_time: u32,
    pulse_width: u32,
    duty_cycle: f32,
    zero_crossings: u16,
}

impl WaveformAnalyzer {
    fn new() -> Self {
        Self {
            peak_value: 0,
            min_value: 4095,
            rms_value: 0.0,
            average_value: 0.0,
            peak_to_peak: 0,
            crest_factor: 0.0,
            form_factor: 0.0,
            rise_time: 0,
            fall_time: 0,
            pulse_width: 0,
            duty_cycle: 0.0,
            zero_crossings: 0,
        }
    }

    fn analyze(&mut self, buffer: &[u16]) {
        if buffer.is_empty() {
            return;
        }

        self.calculate_basic_parameters(buffer);
        self.calculate_timing_parameters(buffer);
        self.calculate_derived_parameters();
    }

    fn calculate_basic_parameters(&mut self, buffer: &[u16]) {
        // 基本参数计算
        self.peak_value = *buffer.iter().max().unwrap_or(&0);
        self.min_value = *buffer.iter().min().unwrap_or(&4095);
        self.peak_to_peak = self.peak_value.saturating_sub(self.min_value);
        
        // 平均值
        let sum: u64 = buffer.iter().map(|&x| x as u64).sum();
        self.average_value = sum as f32 / buffer.len() as f32;
        
        // RMS值
        let sum_squares: u64 = buffer.iter().map(|&x| (x as u64) * (x as u64)).sum();
        self.rms_value = ((sum_squares as f32) / (buffer.len() as f32)).sqrt();
        
        // 过零点计算
        self.zero_crossings = self.count_zero_crossings(buffer);
    }

    fn calculate_timing_parameters(&mut self, buffer: &[u16]) {
        let threshold_high = (self.peak_value as f32 * 0.9) as u16;
        let threshold_low = (self.min_value as f32 + (self.peak_value - self.min_value) as f32 * 0.1) as u16;
        let threshold_mid = (self.peak_value + self.min_value) / 2;
        
        // 上升时间 (10% to 90%)
        let mut rise_start = None;
        let mut rise_end = None;
        
        for (i, &sample) in buffer.iter().enumerate() {
            if rise_start.is_none() && sample > threshold_low {
                rise_start = Some(i);
            }
            if rise_start.is_some() && rise_end.is_none() && sample > threshold_high {
                rise_end = Some(i);
                break;
            }
        }
        
        if let (Some(start), Some(end)) = (rise_start, rise_end) {
            self.rise_time = (end - start) as u32;
        }
        
        // 脉冲宽度和占空比
        let mut high_samples = 0;
        for &sample in buffer {
            if sample > threshold_mid {
                high_samples += 1;
            }
        }
        
        self.pulse_width = high_samples;
        self.duty_cycle = (high_samples as f32 / buffer.len() as f32) * 100.0;
    }

    fn calculate_derived_parameters(&mut self) {
        // 峰值因子
        if self.rms_value > 0.0 {
            self.crest_factor = self.peak_value as f32 / self.rms_value;
        }
        
        // 波形因子
        if self.average_value > 0.0 {
            self.form_factor = self.rms_value / self.average_value;
        }
    }

    fn count_zero_crossings(&self, buffer: &[u16]) -> u16 {
        let mid_point = 2048u16;
        let mut crossings = 0;
        let mut last_above = buffer[0] > mid_point;

        for &sample in buffer.iter().skip(1) {
            let current_above = sample > mid_point;
            if current_above != last_above {
                crossings += 1;
                last_above = current_above;
            }
        }

        crossings
    }

    fn get_waveform_classification(&self) -> &'static str {
        // 基于分析参数的波形分类
        if self.duty_cycle > 80.0 {
            "高电平信号"
        } else if self.duty_cycle < 20.0 {
            "脉冲信号"
        } else if self.crest_factor > 2.0 {
            "尖峰信号"
        } else if self.form_factor > 1.5 {
            "非正弦信号"
        } else if self.zero_crossings > 10 {
            "振荡信号"
        } else {
            "复杂信号"
        }
    }
}

// 任意波形性能监控
struct ArbitraryPerformance {
    waveform_fidelity: f32,
    update_rate: f32,
    memory_usage: f32,
    generation_time: u32,
    switching_time: u32,
    error_rate: f32,
    last_switch_time: u32,
    waveform_switches: u32,
}

impl ArbitraryPerformance {
    fn new() -> Self {
        Self {
            waveform_fidelity: 0.0,
            update_rate: 0.0,
            memory_usage: 0.0,
            generation_time: 0,
            switching_time: 0,
            error_rate: 0.0,
            last_switch_time: 0,
            waveform_switches: 0,
        }
    }

    fn update(&mut self, analyzer: &WaveformAnalyzer, sample_count: u32) {
        // 计算波形保真度
        self.waveform_fidelity = self.calculate_fidelity(analyzer);
        
        // 计算更新速率
        self.update_rate = SAMPLE_RATE as f32;
        
        // 估算内存使用率
        self.memory_usage = (WAVEFORM_SIZE as f32 / 4096.0) * 100.0; // 假设4KB总内存
        
        // 记录切换时间
        if sample_count > self.last_switch_time {
            self.switching_time = sample_count - self.last_switch_time;
            self.last_switch_time = sample_count;
            self.waveform_switches += 1;
        }
    }

    fn calculate_fidelity(&self, analyzer: &WaveformAnalyzer) -> f32 {
        // 基于波形参数计算保真度
        let peak_score = if analyzer.peak_value > 100 && analyzer.peak_value < 4000 { 100.0 } else { 80.0 };
        let rms_score = if analyzer.rms_value > 100.0 && analyzer.rms_value < 3000.0 { 100.0 } else { 80.0 };
        let noise_score = 100.0 - (analyzer.zero_crossings as f32 * 2.0).min(50.0);
        
        (peak_score + rms_score + noise_score) / 3.0
    }

    fn get_performance_rating(&self) -> &'static str {
        if self.waveform_fidelity > 90.0 {
            "优秀"
        } else if self.waveform_fidelity > 80.0 {
            "良好"
        } else if self.waveform_fidelity > 70.0 {
            "一般"
        } else {
            "需要改进"
        }
    }
}

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();

    // 配置串口用于输出
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    let serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &clocks,
    ).unwrap();
    let (mut tx, _rx) = serial.split();

    // 配置DAC引脚
    let dac_pin = gpioa.pa4.into_analog();

    // 配置DAC
    let mut dac = Dac::new(dp.DAC, dac_pin, &clocks);

    // 配置定时器用于采样触发
    let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer.start(SAMPLE_RATE.hz()).unwrap();
    timer.listen(Event::Update);

    // 初始化任意波形组件
    let mut waveform_library = WaveformLibrary::new();
    let mut waveform_analyzer = WaveformAnalyzer::new();
    let mut arbitrary_performance = ArbitraryPerformance::new();
    let mut statistics = GeneratorStatistics::new();

    // 创建任意波形数据
    let mut arbitrary_data = [0u16; WAVEFORM_SIZE];
    waveform_library.generate_waveform(0, &mut arbitrary_data);
    
    let arbitrary_waveform = ArbitraryWaveform::new(&arbitrary_data);

    // 初始化函数发生器
    let config = GeneratorConfig {
        waveform: WaveformType::Arbitrary,
        frequency: 1000,
        amplitude: 2000,
        offset: 0,
        phase: 0.0,
        duty_cycle: 50,
        sample_rate: SAMPLE_RATE,
    };
    
    let mut generator = WaveformGenerator::new(config);
    generator.set_arbitrary_waveform(Some(arbitrary_waveform));

    // 状态变量
    let mut sample_count = 0u32;
    let mut last_status_time = 0u32;
    let mut last_waveform_switch = 0u32;
    let mut sample_buffer = [0u16; BUFFER_SIZE];
    let mut buffer_index = 0;
    let waveform_switch_interval = 2000000; // 5秒 @ 400kHz

    writeln!(tx, "任意波形发生器启动").unwrap();
    writeln!(tx, "采样率: {}Hz", SAMPLE_RATE).unwrap();
    writeln!(tx, "缓冲区大小: {}", BUFFER_SIZE).unwrap();
    writeln!(tx, "波形库: {}种预定义波形", waveform_library.waveform_count).unwrap();
    writeln!(tx, "波形大小: {}点", WAVEFORM_SIZE).unwrap();
    writeln!(tx, "切换间隔: {}秒", waveform_switch_interval / SAMPLE_RATE).unwrap();

    loop {
        // 检查定时器事件
        if timer.wait().is_ok() {
            // 检查是否需要切换波形
            if sample_count.wrapping_sub(last_waveform_switch) >= waveform_switch_interval {
                last_waveform_switch = sample_count;
                
                // 切换到下一个波形
                waveform_library.next_waveform();
                waveform_library.generate_waveform(waveform_library.current_waveform, &mut arbitrary_data);
                
                let new_arbitrary_waveform = ArbitraryWaveform::new(&arbitrary_data);
                generator.set_arbitrary_waveform(Some(new_arbitrary_waveform));
            }
            
            // 生成任意波形样本
            let sample = generator.generate_sample();
            
            // 输出到DAC
            dac.write(sample);
            
            // 存储样本到缓冲区
            sample_buffer[buffer_index] = sample;
            buffer_index = (buffer_index + 1) % BUFFER_SIZE;
            
            sample_count += 1;
        }

        // 定期输出状态信息和分析
        if sample_count.wrapping_sub(last_status_time) >= 800000 { // 每2秒
            last_status_time = sample_count;
            
            // 进行波形分析
            waveform_analyzer.analyze(&sample_buffer);
            
            // 更新性能统计
            arbitrary_performance.update(&waveform_analyzer, sample_count);
            
            // 更新基础统计信息
            statistics.update(&sample_buffer, 1000, SAMPLE_RATE);
            
            writeln!(tx, "\n=== 任意波形发生器状态 ===").unwrap();
            writeln!(tx, "运行时间: {}s", sample_count / SAMPLE_RATE).unwrap();
            writeln!(tx, "总样本数: {}", sample_count).unwrap();
            writeln!(tx, "波形切换次数: {}", arbitrary_performance.waveform_switches).unwrap();
            
            // 显示当前波形信息
            writeln!(tx, "\n--- 当前波形 ---").unwrap();
            writeln!(tx, "波形名称: {}", waveform_library.get_current_name()).unwrap();
            writeln!(tx, "波形编号: {}/{}", waveform_library.current_waveform + 1, waveform_library.waveform_count).unwrap();
            writeln!(tx, "波形大小: {}点", WAVEFORM_SIZE).unwrap();
            writeln!(tx, "输出频率: 1000Hz").unwrap();
            writeln!(tx, "输出幅度: 2000mV").unwrap();
            
            // 显示波形分析结果
            writeln!(tx, "\n--- 波形分析 ---").unwrap();
            writeln!(tx, "峰值: {} ({}mV)", waveform_analyzer.peak_value, 
                     dac_to_voltage(waveform_analyzer.peak_value, VREF_MV)).unwrap();
            writeln!(tx, "最小值: {} ({}mV)", waveform_analyzer.min_value,
                     dac_to_voltage(waveform_analyzer.min_value, VREF_MV)).unwrap();
            writeln!(tx, "峰峰值: {} ({}mV)", waveform_analyzer.peak_to_peak,
                     dac_to_voltage(waveform_analyzer.peak_to_peak, VREF_MV).abs()).unwrap();
            writeln!(tx, "RMS值: {:.1}", waveform_analyzer.rms_value).unwrap();
            writeln!(tx, "平均值: {:.1}", waveform_analyzer.average_value).unwrap();
            writeln!(tx, "峰值因子: {:.2}", waveform_analyzer.crest_factor).unwrap();
            writeln!(tx, "波形因子: {:.2}", waveform_analyzer.form_factor).unwrap();
            writeln!(tx, "过零点数: {}", waveform_analyzer.zero_crossings).unwrap();
            
            // 显示时序参数
            writeln!(tx, "\n--- 时序参数 ---").unwrap();
            writeln!(tx, "上升时间: {}us", waveform_analyzer.rise_time * 1000000 / SAMPLE_RATE).unwrap();
            writeln!(tx, "脉冲宽度: {}us", waveform_analyzer.pulse_width * 1000000 / SAMPLE_RATE).unwrap();
            writeln!(tx, "占空比: {:.1}%", waveform_analyzer.duty_cycle).unwrap();
            writeln!(tx, "波形分类: {}", waveform_analyzer.get_waveform_classification()).unwrap();
            
            // 显示DAC输出信息
            let current_sample = sample_buffer[(buffer_index + BUFFER_SIZE - 1) % BUFFER_SIZE];
            let output_voltage = dac_to_voltage(current_sample, VREF_MV);
            writeln!(tx, "\n--- DAC输出 ---").unwrap();
            writeln!(tx, "当前DAC值: {}", current_sample).unwrap();
            writeln!(tx, "输出电压: {}mV", output_voltage).unwrap();
            writeln!(tx, "输出范围: 0-{}mV", VREF_MV).unwrap();
            writeln!(tx, "分辨率: 12位 (4096级)").unwrap();
            
            // 显示任意波形性能
            writeln!(tx, "\n--- 性能指标 ---").unwrap();
            writeln!(tx, "波形保真度: {:.1}%", arbitrary_performance.waveform_fidelity).unwrap();
            writeln!(tx, "更新速率: {:.0}Hz", arbitrary_performance.update_rate).unwrap();
            writeln!(tx, "内存使用: {:.1}%", arbitrary_performance.memory_usage).unwrap();
            writeln!(tx, "切换时间: {}ms", arbitrary_performance.switching_time / (SAMPLE_RATE / 1000)).unwrap();
            writeln!(tx, "错误率: {:.3}%", arbitrary_performance.error_rate).unwrap();
            writeln!(tx, "性能评级: {}", arbitrary_performance.get_performance_rating()).unwrap();
            
            // 显示信号质量统计
            writeln!(tx, "\n--- 信号质量 ---").unwrap();
            writeln!(tx, "频率精度: {:.2}%", statistics.frequency_accuracy).unwrap();
            writeln!(tx, "幅度精度: {:.2}%", statistics.amplitude_accuracy).unwrap();
            writeln!(tx, "总谐波失真: {:.3}%", statistics.thd).unwrap();
            writeln!(tx, "信噪比: {:.1}dB", statistics.snr).unwrap();
            writeln!(tx, "频率稳定性: {:.3}%", statistics.frequency_stability).unwrap();
            
            // 显示当前波形特性描述
            writeln!(tx, "\n--- 波形特性 ---").unwrap();
            match waveform_library.current_waveform {
                0 => {
                    writeln!(tx, "心电图波形: 模拟心脏电活动").unwrap();
                    writeln!(tx, "特点: P波、QRS复合波、T波").unwrap();
                    writeln!(tx, "应用: 医疗设备测试、生物信号模拟").unwrap();
                },
                1 => {
                    writeln!(tx, "音频波形: 多频率混合信号").unwrap();
                    writeln!(tx, "特点: 基频+谐波+噪声").unwrap();
                    writeln!(tx, "应用: 音频设备测试、声学分析").unwrap();
                },
                2 => {
                    writeln!(tx, "脉冲串: 周期性脉冲序列").unwrap();
                    writeln!(tx, "特点: 固定占空比、快速边沿").unwrap();
                    writeln!(tx, "应用: 数字电路测试、时序分析").unwrap();
                },
                3 => {
                    writeln!(tx, "AM包络: 幅度调制包络信号").unwrap();
                    writeln!(tx, "特点: 载波被调制信号调制").unwrap();
                    writeln!(tx, "应用: 通信系统测试、调制分析").unwrap();
                },
                4 => {
                    writeln!(tx, "啁啾信号: 频率扫描信号").unwrap();
                    writeln!(tx, "特点: 频率随时间线性变化").unwrap();
                    writeln!(tx, "应用: 雷达测试、频响测量").unwrap();
                },
                5 => {
                    writeln!(tx, "高斯脉冲: 钟形脉冲信号").unwrap();
                    writeln!(tx, "特点: 平滑边沿、最小频谱扩展").unwrap();
                    writeln!(tx, "应用: 脉冲响应测试、滤波器测试").unwrap();
                },
                6 => {
                    writeln!(tx, "指数衰减: 指数衰减信号").unwrap();
                    writeln!(tx, "特点: 快速衰减、单调下降").unwrap();
                    writeln!(tx, "应用: 瞬态响应测试、RC电路分析").unwrap();
                },
                7 => {
                    writeln!(tx, "阶跃响应: 阶跃输入响应").unwrap();
                    writeln!(tx, "特点: 突变+指数趋近").unwrap();
                    writeln!(tx, "应用: 系统响应测试、控制系统").unwrap();
                },
                8 => {
                    writeln!(tx, "复合谐波: 多次谐波叠加").unwrap();
                    writeln!(tx, "特点: 丰富的频谱成分").unwrap();
                    writeln!(tx, "应用: 谐波分析、失真测试").unwrap();
                },
                9 => {
                    writeln!(tx, "随机游走: 随机变化信号").unwrap();
                    writeln!(tx, "特点: 随机性、连续性").unwrap();
                    writeln!(tx, "应用: 噪声模拟、随机过程测试").unwrap();
                },
                _ => {
                    writeln!(tx, "未知波形").unwrap();
                }
            }
            
            // 显示下一个波形预告
            let next_waveform = (waveform_library.current_waveform + 1) % waveform_library.waveform_count;
            let next_name = waveform_library.waveform_names[next_waveform];
            let remaining_time = (waveform_switch_interval - (sample_count - last_waveform_switch)) / SAMPLE_RATE;
            
            writeln!(tx, "\n--- 下一个波形 ---").unwrap();
            writeln!(tx, "下一个波形: {}", next_name).unwrap();
            writeln!(tx, "切换倒计时: {}秒", remaining_time).unwrap();
            
            // 显示系统状态
            writeln!(tx, "\n--- 系统状态 ---").unwrap();
            writeln!(tx, "系统时钟: {}MHz", clocks.sysclk().0 / 1_000_000).unwrap();
            writeln!(tx, "定时器频率: {}Hz", SAMPLE_RATE).unwrap();
            writeln!(tx, "缓冲区使用: {}/{}", buffer_index, BUFFER_SIZE).unwrap();
            writeln!(tx, "波形内存: {}字节", WAVEFORM_SIZE * 2).unwrap();
            writeln!(tx, "任意波形状态: 正常").unwrap();
            writeln!(tx, "DAC状态: 正常").unwrap();
        }
    }
}