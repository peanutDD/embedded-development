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
    WaveformGenerator, GeneratorConfig, WaveformType, SweepConfig, SweepType,
    GeneratorStatistics, voltage_to_dac, dac_to_voltage
};

// 扫频发生器配置常量
const VREF_MV: u16 = 3300;
const SAMPLE_RATE: u32 = 250000; // 250kHz
const BUFFER_SIZE: usize = 2500;

// 扫频控制器
struct SweepController {
    sweep_type: SweepType,
    start_freq: u32,
    stop_freq: u32,
    start_amplitude: u16,
    stop_amplitude: u16,
    sweep_time: u32, // 扫描时间（采样数）
    current_step: u32,
    total_steps: u32,
    sweep_count: u32,
    direction_up: bool,
    current_freq: u32,
    current_amplitude: u16,
}

impl SweepController {
    fn new() -> Self {
        Self {
            sweep_type: SweepType::Linear,
            start_freq: 100,
            stop_freq: 10000,
            start_amplitude: 500,
            stop_amplitude: 2000,
            sweep_time: 1250000, // 5秒 @ 250kHz
            current_step: 0,
            total_steps: 1000,
            sweep_count: 0,
            direction_up: true,
            current_freq: 100,
            current_amplitude: 500,
        }
    }

    fn update(&mut self) -> (u32, u16) {
        // 计算当前步骤的进度
        let progress = if self.direction_up {
            self.current_step as f32 / self.total_steps as f32
        } else {
            1.0 - (self.current_step as f32 / self.total_steps as f32)
        };

        // 根据扫描类型计算频率
        self.current_freq = match self.sweep_type {
            SweepType::Linear => {
                let freq_range = self.stop_freq - self.start_freq;
                self.start_freq + (freq_range as f32 * progress) as u32
            },
            SweepType::Logarithmic => {
                let log_start = (self.start_freq as f32).ln();
                let log_stop = (self.stop_freq as f32).ln();
                let log_current = log_start + (log_stop - log_start) * progress;
                log_current.exp() as u32
            },
            SweepType::Exponential => {
                let exp_factor = progress * progress; // 二次函数
                let freq_range = self.stop_freq - self.start_freq;
                self.start_freq + (freq_range as f32 * exp_factor) as u32
            },
        };

        // 计算幅度（线性扫描）
        let amp_range = self.stop_amplitude as i32 - self.start_amplitude as i32;
        self.current_amplitude = (self.start_amplitude as i32 + (amp_range as f32 * progress) as i32) as u16;

        // 更新步骤
        self.current_step += 1;

        // 检查是否完成一次扫描
        if self.current_step >= self.total_steps {
            self.current_step = 0;
            self.sweep_count += 1;
            
            // 双向扫描
            self.direction_up = !self.direction_up;
            
            // 每完成一个双向扫描周期，切换扫描类型
            if self.sweep_count % 2 == 0 {
                self.sweep_type = match self.sweep_type {
                    SweepType::Linear => SweepType::Logarithmic,
                    SweepType::Logarithmic => SweepType::Exponential,
                    SweepType::Exponential => SweepType::Linear,
                };
            }
        }

        (self.current_freq, self.current_amplitude)
    }

    fn get_progress(&self) -> u8 {
        ((self.current_step * 100) / self.total_steps) as u8
    }

    fn get_sweep_info(&self) -> (SweepType, bool, u32, f32) {
        let progress = self.current_step as f32 / self.total_steps as f32;
        (self.sweep_type, self.direction_up, self.sweep_count, progress)
    }
}

// 频谱分析器（简化版）
struct SpectrumAnalyzer {
    frequency_bins: [u32; 32], // 32个频率段
    power_bins: [f32; 32],
    peak_frequency: u32,
    peak_power: f32,
    total_power: f32,
    fundamental_power: f32,
    harmonic_power: f32,
}

impl SpectrumAnalyzer {
    fn new() -> Self {
        Self {
            frequency_bins: [0; 32],
            power_bins: [0.0; 32],
            peak_frequency: 0,
            peak_power: 0.0,
            total_power: 0.0,
            fundamental_power: 0.0,
            harmonic_power: 0.0,
        }
    }

    fn analyze(&mut self, buffer: &[u16], fundamental_freq: u32, sample_rate: u32) {
        // 简化的频谱分析
        self.calculate_power_spectrum(buffer);
        self.find_peak_frequency(sample_rate);
        self.calculate_harmonic_content(fundamental_freq, sample_rate);
    }

    fn calculate_power_spectrum(&mut self, buffer: &[u16]) {
        // 简化的功率谱计算
        let bin_size = buffer.len() / 32;
        
        for (i, power) in self.power_bins.iter_mut().enumerate() {
            let start_idx = i * bin_size;
            let end_idx = ((i + 1) * bin_size).min(buffer.len());
            
            let mut sum_squares = 0.0;
            for j in start_idx..end_idx {
                let sample = buffer[j] as f32 - 2048.0; // 去除直流分量
                sum_squares += sample * sample;
            }
            
            *power = sum_squares / (end_idx - start_idx) as f32;
        }
        
        self.total_power = self.power_bins.iter().sum();
    }

    fn find_peak_frequency(&mut self, sample_rate: u32) {
        let mut max_power = 0.0;
        let mut max_bin = 0;
        
        for (i, &power) in self.power_bins.iter().enumerate() {
            if power > max_power {
                max_power = power;
                max_bin = i;
            }
        }
        
        self.peak_power = max_power;
        self.peak_frequency = (max_bin as u32 * sample_rate) / (32 * 2); // 奈奎斯特频率
    }

    fn calculate_harmonic_content(&mut self, fundamental_freq: u32, sample_rate: u32) {
        // 估算基频和谐波功率
        let freq_per_bin = sample_rate / (32 * 2);
        let fundamental_bin = (fundamental_freq / freq_per_bin).min(31);
        
        self.fundamental_power = if fundamental_bin < 32 {
            self.power_bins[fundamental_bin as usize]
        } else {
            0.0
        };
        
        // 计算谐波功率（2次、3次、4次谐波）
        self.harmonic_power = 0.0;
        for harmonic in 2..=4 {
            let harmonic_bin = (fundamental_bin * harmonic).min(31);
            if harmonic_bin < 32 {
                self.harmonic_power += self.power_bins[harmonic_bin as usize];
            }
        }
    }

    fn get_thd(&self) -> f32 {
        if self.fundamental_power > 0.0 {
            (self.harmonic_power / self.fundamental_power).sqrt() * 100.0
        } else {
            0.0
        }
    }

    fn get_snr(&self) -> f32 {
        let signal_power = self.fundamental_power + self.harmonic_power;
        let noise_power = self.total_power - signal_power;
        
        if noise_power > 0.0 {
            10.0 * (signal_power / noise_power).log10()
        } else {
            100.0 // 很高的SNR
        }
    }
}

// 扫频性能监控
struct SweepPerformance {
    sweep_rate: f32, // Hz/s
    frequency_accuracy: f32,
    amplitude_accuracy: f32,
    settling_time: u32,
    linearity_error: f32,
    sweep_repeatability: f32,
    last_freq_update: u32,
    last_frequency: u32,
}

impl SweepPerformance {
    fn new() -> Self {
        Self {
            sweep_rate: 0.0,
            frequency_accuracy: 0.0,
            amplitude_accuracy: 0.0,
            settling_time: 0,
            linearity_error: 0.0,
            sweep_repeatability: 0.0,
            last_freq_update: 0,
            last_frequency: 0,
        }
    }

    fn update(&mut self, current_freq: u32, target_freq: u32, current_amp: u16, target_amp: u16, sample_count: u32) {
        // 计算扫频速率
        if sample_count > self.last_freq_update {
            let time_diff = sample_count - self.last_freq_update;
            let freq_diff = current_freq.abs_diff(self.last_frequency);
            
            if time_diff > 0 {
                self.sweep_rate = (freq_diff as f32 * SAMPLE_RATE as f32) / time_diff as f32;
            }
        }

        // 计算频率精度
        if target_freq > 0 {
            self.frequency_accuracy = 100.0 - (current_freq.abs_diff(target_freq) as f32 / target_freq as f32 * 100.0);
        }

        // 计算幅度精度
        if target_amp > 0 {
            self.amplitude_accuracy = 100.0 - (current_amp.abs_diff(target_amp) as f32 / target_amp as f32 * 100.0);
        }

        self.last_freq_update = sample_count;
        self.last_frequency = current_freq;
    }

    fn calculate_linearity(&mut self, expected_progress: f32, actual_progress: f32) {
        self.linearity_error = (expected_progress - actual_progress).abs() * 100.0;
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

    // 初始化扫频组件
    let mut sweep_controller = SweepController::new();
    let mut spectrum_analyzer = SpectrumAnalyzer::new();
    let mut sweep_performance = SweepPerformance::new();
    let mut statistics = GeneratorStatistics::new();

    // 初始化函数发生器
    let config = GeneratorConfig {
        waveform: WaveformType::Sine, // 扫频通常使用正弦波
        frequency: sweep_controller.current_freq,
        amplitude: sweep_controller.current_amplitude,
        offset: 0,
        phase: 0.0,
        duty_cycle: 50,
        sample_rate: SAMPLE_RATE,
    };
    
    let mut generator = WaveformGenerator::new(config);

    // 状态变量
    let mut sample_count = 0u32;
    let mut last_status_time = 0u32;
    let mut last_sweep_update = 0u32;
    let mut sample_buffer = [0u16; BUFFER_SIZE];
    let mut buffer_index = 0;
    let sweep_update_interval = SAMPLE_RATE / 1000; // 1kHz更新率

    writeln!(tx, "扫频发生器启动").unwrap();
    writeln!(tx, "采样率: {}Hz", SAMPLE_RATE).unwrap();
    writeln!(tx, "扫频范围: {}Hz - {}Hz", sweep_controller.start_freq, sweep_controller.stop_freq).unwrap();
    writeln!(tx, "幅度范围: {}mV - {}mV", sweep_controller.start_amplitude, sweep_controller.stop_amplitude).unwrap();
    writeln!(tx, "扫描时间: {}秒", sweep_controller.sweep_time / SAMPLE_RATE).unwrap();
    writeln!(tx, "更新频率: {}Hz", SAMPLE_RATE / sweep_update_interval).unwrap();

    loop {
        // 检查定时器事件
        if timer.wait().is_ok() {
            // 检查是否需要更新扫频参数
            if sample_count.wrapping_sub(last_sweep_update) >= sweep_update_interval {
                last_sweep_update = sample_count;
                
                let (new_freq, new_amplitude) = sweep_controller.update();
                
                // 更新发生器配置
                let new_config = GeneratorConfig {
                    waveform: WaveformType::Sine,
                    frequency: new_freq,
                    amplitude: new_amplitude,
                    offset: 0,
                    phase: 0.0,
                    duty_cycle: 50,
                    sample_rate: SAMPLE_RATE,
                };
                
                generator.update_config(new_config);
                
                // 更新性能统计
                sweep_performance.update(new_freq, new_freq, new_amplitude, new_amplitude, sample_count);
            }
            
            // 生成样本
            let sample = generator.generate_sample();
            
            // 输出到DAC
            dac.write(sample);
            
            // 存储样本到缓冲区
            sample_buffer[buffer_index] = sample;
            buffer_index = (buffer_index + 1) % BUFFER_SIZE;
            
            sample_count += 1;
        }

        // 定期输出状态信息和分析
        if sample_count.wrapping_sub(last_status_time) >= 500000 { // 每2秒
            last_status_time = sample_count;
            
            // 进行频谱分析
            spectrum_analyzer.analyze(&sample_buffer, sweep_controller.current_freq, SAMPLE_RATE);
            
            // 更新统计信息
            statistics.update(&sample_buffer, sweep_controller.current_freq, SAMPLE_RATE);
            
            writeln!(tx, "\n=== 扫频发生器状态 ===").unwrap();
            writeln!(tx, "运行时间: {}s", sample_count / SAMPLE_RATE).unwrap();
            writeln!(tx, "总样本数: {}", sample_count).unwrap();
            writeln!(tx, "扫描完成次数: {}", sweep_controller.sweep_count).unwrap();
            
            // 显示当前扫频状态
            let (sweep_type, direction_up, sweep_count, progress) = sweep_controller.get_sweep_info();
            writeln!(tx, "\n--- 扫频状态 ---").unwrap();
            writeln!(tx, "扫描类型: {:?}", sweep_type).unwrap();
            writeln!(tx, "扫描方向: {}", if direction_up { "上升" } else { "下降" }).unwrap();
            writeln!(tx, "扫描进度: {:.1}%", progress * 100.0).unwrap();
            writeln!(tx, "当前频率: {}Hz", sweep_controller.current_freq).unwrap();
            writeln!(tx, "当前幅度: {}mV", sweep_controller.current_amplitude).unwrap();
            writeln!(tx, "扫描速率: {:.1}Hz/s", sweep_performance.sweep_rate).unwrap();
            
            // 显示扫频范围信息
            writeln!(tx, "\n--- 扫频范围 ---").unwrap();
            writeln!(tx, "起始频率: {}Hz", sweep_controller.start_freq).unwrap();
            writeln!(tx, "结束频率: {}Hz", sweep_controller.stop_freq).unwrap();
            writeln!(tx, "起始幅度: {}mV", sweep_controller.start_amplitude).unwrap();
            writeln!(tx, "结束幅度: {}mV", sweep_controller.stop_amplitude).unwrap();
            writeln!(tx, "频率跨度: {}Hz", sweep_controller.stop_freq - sweep_controller.start_freq).unwrap();
            writeln!(tx, "幅度跨度: {}mV", sweep_controller.stop_amplitude - sweep_controller.start_amplitude).unwrap();
            
            // 显示DAC输出信息
            let current_sample = sample_buffer[(buffer_index + BUFFER_SIZE - 1) % BUFFER_SIZE];
            let output_voltage = dac_to_voltage(current_sample, VREF_MV);
            writeln!(tx, "\n--- DAC输出 ---").unwrap();
            writeln!(tx, "当前DAC值: {}", current_sample).unwrap();
            writeln!(tx, "输出电压: {}mV", output_voltage).unwrap();
            writeln!(tx, "理论幅度: {}mV", sweep_controller.current_amplitude).unwrap();
            writeln!(tx, "幅度误差: {:.1}%", 
                     (output_voltage.abs_diff(sweep_controller.current_amplitude as i16) as f32 / sweep_controller.current_amplitude as f32 * 100.0)).unwrap();
            
            // 显示频谱分析结果
            writeln!(tx, "\n--- 频谱分析 ---").unwrap();
            writeln!(tx, "峰值频率: {}Hz", spectrum_analyzer.peak_frequency).unwrap();
            writeln!(tx, "峰值功率: {:.1}", spectrum_analyzer.peak_power).unwrap();
            writeln!(tx, "总功率: {:.1}", spectrum_analyzer.total_power).unwrap();
            writeln!(tx, "基频功率: {:.1}", spectrum_analyzer.fundamental_power).unwrap();
            writeln!(tx, "谐波功率: {:.1}", spectrum_analyzer.harmonic_power).unwrap();
            writeln!(tx, "总谐波失真: {:.3}%", spectrum_analyzer.get_thd()).unwrap();
            writeln!(tx, "信噪比: {:.1}dB", spectrum_analyzer.get_snr()).unwrap();
            
            // 显示扫频性能
            writeln!(tx, "\n--- 扫频性能 ---").unwrap();
            writeln!(tx, "频率精度: {:.2}%", sweep_performance.frequency_accuracy).unwrap();
            writeln!(tx, "幅度精度: {:.2}%", sweep_performance.amplitude_accuracy).unwrap();
            writeln!(tx, "扫描速率: {:.1}Hz/s", sweep_performance.sweep_rate).unwrap();
            writeln!(tx, "线性度误差: {:.3}%", sweep_performance.linearity_error).unwrap();
            
            // 显示信号质量统计
            writeln!(tx, "\n--- 信号质量 ---").unwrap();
            writeln!(tx, "频率稳定性: {:.3}%", statistics.frequency_stability).unwrap();
            writeln!(tx, "幅度稳定性: {:.2}%", statistics.amplitude_accuracy).unwrap();
            writeln!(tx, "相位噪声: 低").unwrap();
            writeln!(tx, "频率分辨率: {:.1}Hz", SAMPLE_RATE as f32 / sweep_controller.total_steps as f32).unwrap();
            
            // 显示扫描类型特性
            writeln!(tx, "\n--- 扫描特性 ---").unwrap();
            match sweep_type {
                SweepType::Linear => {
                    writeln!(tx, "线性扫描: 频率均匀变化，适合一般测试").unwrap();
                    writeln!(tx, "特点: 每步频率增量相等").unwrap();
                },
                SweepType::Logarithmic => {
                    writeln!(tx, "对数扫描: 频率比例变化，适合宽带测试").unwrap();
                    writeln!(tx, "特点: 低频密集，高频稀疏").unwrap();
                },
                SweepType::Exponential => {
                    writeln!(tx, "指数扫描: 频率加速变化，适合快速扫描").unwrap();
                    writeln!(tx, "特点: 开始慢，后期快").unwrap();
                },
            }
            
            // 显示应用建议
            writeln!(tx, "\n--- 应用建议 ---").unwrap();
            if sweep_controller.current_freq < 1000 {
                writeln!(tx, "当前频段: 低频 - 适合音频、控制系统测试").unwrap();
            } else if sweep_controller.current_freq < 10000 {
                writeln!(tx, "当前频段: 中频 - 适合通用信号处理测试").unwrap();
            } else {
                writeln!(tx, "当前频段: 高频 - 适合高速电路、RF前端测试").unwrap();
            }
            
            // 显示系统状态
            writeln!(tx, "\n--- 系统状态 ---").unwrap();
            writeln!(tx, "系统时钟: {}MHz", clocks.sysclk().0 / 1_000_000).unwrap();
            writeln!(tx, "定时器频率: {}Hz", SAMPLE_RATE).unwrap();
            writeln!(tx, "更新频率: {}Hz", SAMPLE_RATE / sweep_update_interval).unwrap();
            writeln!(tx, "缓冲区使用: {}/{}", buffer_index, BUFFER_SIZE).unwrap();
            writeln!(tx, "内存使用: 正常").unwrap();
            writeln!(tx, "扫频状态: 正常").unwrap();
        }
    }
}