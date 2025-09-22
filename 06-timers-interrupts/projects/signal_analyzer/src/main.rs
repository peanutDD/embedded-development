#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::asm;

// 平台特定的HAL导入
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;
#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal;
#[cfg(feature = "nrf52840")]
use nrf52840_hal as hal;

use hal::{
    prelude::*,
    timer::{Timer, Event},
    gpio::{Input, Output, PushPull, PullUp},
    adc::{Adc, AdcPin},
};

use signal_analyzer::{
    SignalAnalyzer, RealTimeMonitor, FrequencyCounter, PulseWidthAnalyzer, 
    WaveformAnalyzer, EdgeType, WindowType, SignalAnalysisResult
};

use heapless::{Vec, String};
use nb::block;

/// 系统状态
#[derive(Debug, Clone, Copy)]
enum SystemState {
    Idle,
    FrequencyMeasurement,
    PulseAnalysis,
    WaveformAnalysis,
    RealTimeMonitoring,
    CalibrationMode,
}

/// 分析模式
#[derive(Debug, Clone, Copy)]
enum AnalysisMode {
    SingleShot,
    Continuous,
    Triggered,
}

/// 触发配置
#[derive(Debug, Clone)]
struct TriggerConfig {
    enabled: bool,
    level: f32,
    edge: EdgeType,
    holdoff_us: u32,
}

/// 测量配置
#[derive(Debug, Clone)]
struct MeasurementConfig {
    sample_rate: u32,
    buffer_size: u16,
    analysis_mode: AnalysisMode,
    trigger: TriggerConfig,
    auto_range: bool,
    input_coupling: InputCoupling,
}

/// 输入耦合方式
#[derive(Debug, Clone, Copy)]
enum InputCoupling {
    DC,
    AC,
}

/// 显示信息
#[derive(Debug, Clone)]
struct DisplayInfo {
    frequency: f32,
    amplitude: f32,
    duty_cycle: f32,
    pulse_width_us: f32,
    rms_value: f32,
    peak_to_peak: f32,
    status: String<64>,
}

/// 校准数据
#[derive(Debug, Clone)]
struct CalibrationData {
    adc_offset: f32,
    adc_gain: f32,
    frequency_correction: f32,
    time_base_error: f32,
}

/// 信号发生器（用于测试）
struct SignalGenerator {
    frequency: f32,
    amplitude: f32,
    offset: f32,
    waveform_type: WaveformType,
    phase: f32,
    sample_rate: u32,
}

/// 波形类型
#[derive(Debug, Clone, Copy)]
enum WaveformType {
    Sine,
    Square,
    Triangle,
    Sawtooth,
    Noise,
}

impl Default for MeasurementConfig {
    fn default() -> Self {
        Self {
            sample_rate: 10000,
            buffer_size: 1024,
            analysis_mode: AnalysisMode::Continuous,
            trigger: TriggerConfig {
                enabled: false,
                level: 0.5,
                edge: EdgeType::Rising,
                holdoff_us: 1000,
            },
            auto_range: true,
            input_coupling: InputCoupling::DC,
        }
    }
}

impl Default for CalibrationData {
    fn default() -> Self {
        Self {
            adc_offset: 0.0,
            adc_gain: 1.0,
            frequency_correction: 1.0,
            time_base_error: 0.0,
        }
    }
}

impl SignalGenerator {
    fn new(sample_rate: u32) -> Self {
        Self {
            frequency: 1000.0,
            amplitude: 1.0,
            offset: 0.0,
            waveform_type: WaveformType::Sine,
            phase: 0.0,
            sample_rate,
        }
    }

    fn set_frequency(&mut self, frequency: f32) {
        self.frequency = frequency;
    }

    fn set_amplitude(&mut self, amplitude: f32) {
        self.amplitude = amplitude;
    }

    fn set_waveform(&mut self, waveform_type: WaveformType) {
        self.waveform_type = waveform_type;
    }

    fn generate_sample(&mut self) -> f32 {
        let sample = match self.waveform_type {
            WaveformType::Sine => {
                libm::sinf(2.0 * core::f32::consts::PI * self.phase)
            },
            WaveformType::Square => {
                if libm::sinf(2.0 * core::f32::consts::PI * self.phase) >= 0.0 { 1.0 } else { -1.0 }
            },
            WaveformType::Triangle => {
                let normalized_phase = self.phase - libm::floorf(self.phase);
                if normalized_phase < 0.5 {
                    4.0 * normalized_phase - 1.0
                } else {
                    3.0 - 4.0 * normalized_phase
                }
            },
            WaveformType::Sawtooth => {
                2.0 * (self.phase - libm::floorf(self.phase)) - 1.0
            },
            WaveformType::Noise => {
                // 简单的伪随机噪声
                let mut x = (self.phase * 12345.0) as u32;
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                (x as f32 / u32::MAX as f32) * 2.0 - 1.0
            },
        };

        // 更新相位
        self.phase += self.frequency / self.sample_rate as f32;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }

        self.amplitude * sample + self.offset
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

    // 配置ADC输入引脚
    let adc_pin = gpioa.pa0.into_analog();
    let mut adc = Adc::adc1(dp.ADC1, true, hal::adc::config::AdcConfig::default());

    // 配置定时器
    let mut timer = Timer::tim2(dp.TIM2, 10.khz(), clocks);
    
    // 配置状态LED
    let mut status_led = gpioc.pc13.into_push_pull_output();
    
    // 配置按钮
    let mode_button = gpioa.pa1.into_pull_up_input();
    let trigger_button = gpioa.pa2.into_pull_up_input();
    let calibrate_button = gpioa.pa3.into_pull_up_input();

    // 初始化分析器
    let mut config = MeasurementConfig::default();
    let mut frequency_counter = FrequencyCounter::new(config.sample_rate, 0.5);
    let mut pulse_analyzer = PulseWidthAnalyzer::new(config.sample_rate, 0.3, 0.7);
    let mut waveform_analyzer = WaveformAnalyzer::new(config.sample_rate);
    let mut real_time_monitor = RealTimeMonitor::new(config.sample_rate, 100);

    // 初始化信号发生器（测试用）
    let mut signal_generator = SignalGenerator::new(config.sample_rate);
    
    // 系统状态
    let mut system_state = SystemState::RealTimeMonitoring;
    let mut calibration_data = CalibrationData::default();
    let mut display_info = DisplayInfo {
        frequency: 0.0,
        amplitude: 0.0,
        duty_cycle: 0.0,
        pulse_width_us: 0.0,
        rms_value: 0.0,
        peak_to_peak: 0.0,
        status: String::new(),
    };

    // 采样缓冲区
    let mut sample_buffer: Vec<(f32, u64), 2048> = Vec::new();
    let mut timestamp_counter = 0u64;
    let mut last_analysis_time = 0u64;
    let mut button_debounce_time = 0u64;

    // 演示不同的分析模式
    demo_frequency_measurement(&mut frequency_counter, &mut signal_generator);
    demo_pulse_analysis(&mut pulse_analyzer, &mut signal_generator);
    demo_waveform_analysis(&mut waveform_analyzer, &mut signal_generator);

    loop {
        // 获取当前时间戳
        timestamp_counter += 100; // 模拟100μs时间步进
        
        // 读取ADC值
        let adc_value: u16 = block!(adc.read(&mut adc_pin)).unwrap_or(0);
        let voltage = (adc_value as f32 / 4095.0) * 3.3; // 转换为电压
        
        // 应用校准
        let calibrated_sample = (voltage - calibration_data.adc_offset) * calibration_data.adc_gain;
        
        // 检查按钮状态
        if timestamp_counter > button_debounce_time + 50000 { // 50ms防抖
            if mode_button.is_low().unwrap() {
                system_state = match system_state {
                    SystemState::Idle => SystemState::FrequencyMeasurement,
                    SystemState::FrequencyMeasurement => SystemState::PulseAnalysis,
                    SystemState::PulseAnalysis => SystemState::WaveformAnalysis,
                    SystemState::WaveformAnalysis => SystemState::RealTimeMonitoring,
                    SystemState::RealTimeMonitoring => SystemState::CalibrationMode,
                    SystemState::CalibrationMode => SystemState::Idle,
                };
                button_debounce_time = timestamp_counter;
                display_info.status.clear();
                display_info.status.push_str("Mode changed").ok();
            }
            
            if trigger_button.is_low().unwrap() {
                config.trigger.enabled = !config.trigger.enabled;
                button_debounce_time = timestamp_counter;
            }
            
            if calibrate_button.is_low().unwrap() {
                perform_calibration(&mut calibration_data, &mut adc, &mut adc_pin);
                button_debounce_time = timestamp_counter;
            }
        }

        // 根据系统状态执行相应的分析
        match system_state {
            SystemState::Idle => {
                status_led.set_low().ok();
                asm::wfi(); // 等待中断
            },
            
            SystemState::FrequencyMeasurement => {
                status_led.set_high().ok();
                
                if let Ok(_) = frequency_counter.add_sample(calibrated_sample, timestamp_counter) {
                    if timestamp_counter > last_analysis_time + 100000 { // 每100ms分析一次
                        if let Ok(result) = frequency_counter.analyze() {
                            display_info.frequency = result.frequency;
                            display_info.status.clear();
                            display_info.status.push_str("Freq measurement").ok();
                        }
                        last_analysis_time = timestamp_counter;
                    }
                }
            },
            
            SystemState::PulseAnalysis => {
                status_led.set_high().ok();
                
                if let Ok(_) = pulse_analyzer.add_sample(calibrated_sample, timestamp_counter) {
                    if timestamp_counter > last_analysis_time + 200000 { // 每200ms分析一次
                        if let Ok(result) = pulse_analyzer.analyze() {
                            display_info.duty_cycle = result.duty_cycle;
                            display_info.pulse_width_us = result.pulse_width_us;
                            display_info.frequency = result.frequency;
                            display_info.status.clear();
                            display_info.status.push_str("Pulse analysis").ok();
                        }
                        last_analysis_time = timestamp_counter;
                    }
                }
            },
            
            SystemState::WaveformAnalysis => {
                status_led.set_high().ok();
                
                if let Ok(_) = waveform_analyzer.add_sample(calibrated_sample, timestamp_counter) {
                    if timestamp_counter > last_analysis_time + 500000 { // 每500ms分析一次
                        if let Ok(result) = waveform_analyzer.analyze() {
                            display_info.amplitude = result.amplitude;
                            display_info.rms_value = result.rms_value;
                            display_info.peak_to_peak = result.peak_to_peak;
                            display_info.frequency = result.frequency;
                            display_info.status.clear();
                            display_info.status.push_str("Waveform analysis").ok();
                        }
                        last_analysis_time = timestamp_counter;
                    }
                }
            },
            
            SystemState::RealTimeMonitoring => {
                // LED闪烁表示实时监控
                if (timestamp_counter / 250000) % 2 == 0 {
                    status_led.set_high().ok();
                } else {
                    status_led.set_low().ok();
                }
                
                if let Ok(_) = real_time_monitor.add_sample(calibrated_sample, timestamp_counter) {
                    if let Ok(Some(result)) = real_time_monitor.update(timestamp_counter) {
                        display_info.frequency = result.frequency;
                        display_info.amplitude = result.amplitude;
                        display_info.duty_cycle = result.duty_cycle;
                        display_info.pulse_width_us = result.pulse_width_us;
                        display_info.rms_value = result.rms_value;
                        display_info.peak_to_peak = result.peak_to_peak;
                        display_info.status.clear();
                        display_info.status.push_str("Real-time monitor").ok();
                    }
                }
            },
            
            SystemState::CalibrationMode => {
                // 快速闪烁表示校准模式
                if (timestamp_counter / 100000) % 2 == 0 {
                    status_led.set_high().ok();
                } else {
                    status_led.set_low().ok();
                }
                
                display_info.status.clear();
                display_info.status.push_str("Calibration mode").ok();
            },
        }

        // 等待定时器事件
        block!(timer.wait()).ok();
    }
}

/// 演示频率测量功能
fn demo_frequency_measurement(counter: &mut FrequencyCounter, generator: &mut SignalGenerator) {
    // 测试不同频率
    let test_frequencies = [100.0, 1000.0, 5000.0, 10000.0];
    
    for &freq in &test_frequencies {
        generator.set_frequency(freq);
        generator.set_waveform(WaveformType::Square);
        counter.reset();
        
        // 生成测试信号
        for i in 0..1000 {
            let sample = generator.generate_sample();
            let timestamp = i * 100; // 100μs间隔
            counter.add_sample(sample, timestamp).ok();
        }
        
        if let Ok(result) = counter.analyze() {
            // 在实际应用中，这里会输出到显示器或串口
            // 这里只是演示分析过程
        }
    }
}

/// 演示脉冲分析功能
fn demo_pulse_analysis(analyzer: &mut PulseWidthAnalyzer, generator: &mut SignalGenerator) {
    // 测试不同占空比
    let test_duty_cycles = [0.1, 0.25, 0.5, 0.75, 0.9];
    
    for &duty_cycle in &test_duty_cycles {
        generator.set_frequency(1000.0);
        generator.set_waveform(WaveformType::Square);
        analyzer.reset();
        
        // 生成PWM信号
        for i in 0..2000 {
            let phase = (i as f32 / 100.0) % 1.0; // 100个样本一个周期
            let sample = if phase < duty_cycle { 1.0 } else { 0.0 };
            let timestamp = i * 100;
            analyzer.add_sample(sample, timestamp).ok();
        }
        
        if let Ok(result) = analyzer.analyze() {
            // 验证占空比测量精度
        }
    }
}

/// 演示波形分析功能
fn demo_waveform_analysis(analyzer: &mut WaveformAnalyzer, generator: &mut SignalGenerator) {
    let waveform_types = [
        WaveformType::Sine,
        WaveformType::Square,
        WaveformType::Triangle,
        WaveformType::Sawtooth,
    ];
    
    for &waveform_type in &waveform_types {
        generator.set_frequency(1000.0);
        generator.set_amplitude(1.0);
        generator.set_waveform(waveform_type);
        analyzer.reset();
        
        // 设置不同的窗函数
        analyzer.set_window_type(WindowType::Hanning);
        
        // 生成测试信号
        for i in 0..1024 {
            let sample = generator.generate_sample();
            let timestamp = i * 100;
            analyzer.add_sample(sample, timestamp).ok();
        }
        
        if let Ok(result) = analyzer.analyze() {
            // 分析不同波形的特征
        }
    }
}

/// 执行系统校准
fn perform_calibration(
    calibration: &mut CalibrationData,
    adc: &mut Adc<hal::pac::ADC1>,
    adc_pin: &mut impl AdcPin<hal::pac::ADC1>
) {
    // 零点校准
    let mut zero_samples = Vec::<u16, 100>::new();
    for _ in 0..100 {
        if let Ok(sample) = block!(adc.read(adc_pin)) {
            zero_samples.push(sample).ok();
        }
    }
    
    if !zero_samples.is_empty() {
        let zero_average = zero_samples.iter().sum::<u16>() as f32 / zero_samples.len() as f32;
        calibration.adc_offset = (zero_average / 4095.0) * 3.3;
    }
    
    // 增益校准（需要已知参考电压）
    // 这里简化为默认值
    calibration.adc_gain = 1.0;
    calibration.frequency_correction = 1.0;
    calibration.time_base_error = 0.0;
}