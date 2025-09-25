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
    WaveformGenerator, GeneratorConfig, WaveformType, GeneratorStatistics,
    voltage_to_dac, dac_to_voltage
};

// 基础函数发生器配置常量
const VREF_MV: u16 = 3300;
const SAMPLE_RATE: u32 = 100000; // 100kHz
const BUFFER_SIZE: usize = 1000;

// 函数发生器状态
struct GeneratorState {
    current_waveform: WaveformType,
    frequency: u32,
    amplitude: u16,
    offset: i16,
    samples_generated: u32,
    last_switch_time: u32,
    waveform_cycle: u8,
}

impl GeneratorState {
    fn new() -> Self {
        Self {
            current_waveform: WaveformType::Sine,
            frequency: 1000,
            amplitude: 1000,
            offset: 0,
            samples_generated: 0,
            last_switch_time: 0,
            waveform_cycle: 0,
        }
    }

    fn next_waveform(&mut self) {
        self.current_waveform = match self.current_waveform {
            WaveformType::Sine => WaveformType::Square,
            WaveformType::Square => WaveformType::Triangle,
            WaveformType::Triangle => WaveformType::Sawtooth,
            WaveformType::Sawtooth => WaveformType::Pulse,
            WaveformType::Pulse => WaveformType::Noise,
            WaveformType::Noise => WaveformType::DC,
            WaveformType::DC => WaveformType::Sine,
            WaveformType::Arbitrary => WaveformType::Sine,
        };
        self.waveform_cycle += 1;
    }

    fn update_parameters(&mut self, time: u32) {
        // 每10秒切换波形
        if time.wrapping_sub(self.last_switch_time) >= 1000000 { // 10秒 @ 100kHz
            self.last_switch_time = time;
            self.next_waveform();
            
            // 根据波形类型调整参数
            match self.current_waveform {
                WaveformType::Sine => {
                    self.frequency = 1000;
                    self.amplitude = 1000;
                },
                WaveformType::Square => {
                    self.frequency = 500;
                    self.amplitude = 1500;
                },
                WaveformType::Triangle => {
                    self.frequency = 2000;
                    self.amplitude = 800;
                },
                WaveformType::Sawtooth => {
                    self.frequency = 1500;
                    self.amplitude = 1200;
                },
                WaveformType::Pulse => {
                    self.frequency = 1000;
                    self.amplitude = 2000;
                },
                WaveformType::Noise => {
                    self.frequency = 0; // 噪声无频率
                    self.amplitude = 500;
                },
                WaveformType::DC => {
                    self.frequency = 0;
                    self.amplitude = 0;
                    self.offset = 1000; // 1V直流偏移
                },
                WaveformType::Arbitrary => {
                    self.frequency = 1000;
                    self.amplitude = 1000;
                },
            }
        }
    }
}

// 性能监控
struct GeneratorPerformance {
    generation_time: u32,
    output_time: u32,
    total_cycles: u32,
    max_generation_time: u32,
    max_output_time: u32,
    waveform_switch_count: u32,
}

impl GeneratorPerformance {
    fn new() -> Self {
        Self {
            generation_time: 0,
            output_time: 0,
            total_cycles: 0,
            max_generation_time: 0,
            max_output_time: 0,
            waveform_switch_count: 0,
        }
    }

    fn update_performance(&mut self, gen_time: u32, out_time: u32) {
        self.generation_time = gen_time;
        self.output_time = out_time;
        
        if gen_time > self.max_generation_time {
            self.max_generation_time = gen_time;
        }
        
        if out_time > self.max_output_time {
            self.max_output_time = out_time;
        }
        
        self.total_cycles += 1;
    }

    fn record_waveform_switch(&mut self) {
        self.waveform_switch_count += 1;
    }

    fn get_average_generation_time(&self) -> u32 {
        if self.total_cycles > 0 {
            self.generation_time / self.total_cycles
        } else {
            0
        }
    }

    fn get_average_output_time(&self) -> u32 {
        if self.total_cycles > 0 {
            self.output_time / self.total_cycles
        } else {
            0
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

    // 初始化函数发生器
    let config = GeneratorConfig {
        waveform: WaveformType::Sine,
        frequency: 1000,
        amplitude: 1000,
        offset: 0,
        phase: 0.0,
        duty_cycle: 50,
        sample_rate: SAMPLE_RATE,
    };
    
    let mut generator = WaveformGenerator::new(config);
    let mut statistics = GeneratorStatistics::new();
    let mut state = GeneratorState::new();
    let mut performance = GeneratorPerformance::new();

    // 状态变量
    let mut sample_count = 0u32;
    let mut last_status_time = 0u32;
    let mut sample_buffer = [0u16; BUFFER_SIZE];
    let mut buffer_index = 0;
    let mut last_waveform = WaveformType::Sine;

    writeln!(tx, "基础函数发生器启动").unwrap();
    writeln!(tx, "采样率: {}Hz", SAMPLE_RATE).unwrap();
    writeln!(tx, "初始波形: {:?}", config.waveform).unwrap();
    writeln!(tx, "初始频率: {}Hz", config.frequency).unwrap();
    writeln!(tx, "初始幅度: {}mV", config.amplitude).unwrap();

    loop {
        // 检查定时器事件
        if timer.wait().is_ok() {
            let current_time = sample_count;
            let generation_start = current_time;
            
            // 更新状态参数
            state.update_parameters(current_time);
            
            // 检查是否需要更新配置
            if state.current_waveform != last_waveform {
                let new_config = GeneratorConfig {
                    waveform: state.current_waveform,
                    frequency: state.frequency,
                    amplitude: state.amplitude,
                    offset: state.offset,
                    phase: 0.0,
                    duty_cycle: match state.current_waveform {
                        WaveformType::Pulse => 25, // 25%占空比
                        _ => 50,
                    },
                    sample_rate: SAMPLE_RATE,
                };
                
                generator.update_config(new_config);
                last_waveform = state.current_waveform;
                performance.record_waveform_switch();
            }
            
            // 生成样本
            let sample = generator.generate_sample();
            let generation_time = current_time - generation_start;
            
            let output_start = current_time;
            
            // 输出到DAC
            dac.write(sample);
            
            let output_time = current_time - output_start;
            
            // 更新性能统计
            performance.update_performance(generation_time, output_time);
            
            // 存储样本到缓冲区用于分析
            sample_buffer[buffer_index] = sample;
            buffer_index = (buffer_index + 1) % BUFFER_SIZE;
            
            state.samples_generated += 1;
            sample_count += 1;
        }

        // 定期输出状态信息
        if sample_count.wrapping_sub(last_status_time) >= 500000 { // 每5秒
            last_status_time = sample_count;
            
            // 更新统计信息
            statistics.update(&sample_buffer, state.frequency, SAMPLE_RATE);
            
            writeln!(tx, "\n=== 函数发生器状态 ===").unwrap();
            writeln!(tx, "运行时间: {}s", sample_count / SAMPLE_RATE).unwrap();
            writeln!(tx, "生成样本数: {}", state.samples_generated).unwrap();
            writeln!(tx, "波形切换次数: {}", performance.waveform_switch_count).unwrap();
            
            // 显示当前波形信息
            writeln!(tx, "\n--- 当前波形 ---").unwrap();
            writeln!(tx, "波形类型: {:?}", state.current_waveform).unwrap();
            writeln!(tx, "频率: {}Hz", state.frequency).unwrap();
            writeln!(tx, "幅度: {}mV", state.amplitude).unwrap();
            writeln!(tx, "直流偏移: {}mV", state.offset).unwrap();
            writeln!(tx, "波形周期: {}", state.waveform_cycle).unwrap();
            
            // 显示DAC输出信息
            let current_sample = sample_buffer[(buffer_index + BUFFER_SIZE - 1) % BUFFER_SIZE];
            let output_voltage = dac_to_voltage(current_sample, VREF_MV);
            writeln!(tx, "\n--- DAC输出 ---").unwrap();
            writeln!(tx, "当前DAC值: {}", current_sample).unwrap();
            writeln!(tx, "输出电压: {}mV", output_voltage).unwrap();
            writeln!(tx, "DAC分辨率: 12位 (0-4095)").unwrap();
            writeln!(tx, "参考电压: {}mV", VREF_MV).unwrap();
            
            // 显示信号质量统计
            writeln!(tx, "\n--- 信号质量 ---").unwrap();
            writeln!(tx, "频率精度: {:.2}%", statistics.frequency_accuracy).unwrap();
            writeln!(tx, "幅度精度: {:.2}%", statistics.amplitude_accuracy).unwrap();
            writeln!(tx, "总谐波失真: {:.3}%", statistics.thd).unwrap();
            writeln!(tx, "信噪比: {:.1}dB", statistics.snr).unwrap();
            writeln!(tx, "频率稳定性: {:.3}%", statistics.frequency_stability).unwrap();
            
            // 显示性能统计
            writeln!(tx, "\n--- 性能统计 ---").unwrap();
            writeln!(tx, "平均生成时间: {}us", performance.get_average_generation_time() * 10).unwrap();
            writeln!(tx, "平均输出时间: {}us", performance.get_average_output_time() * 10).unwrap();
            writeln!(tx, "最大生成时间: {}us", performance.max_generation_time * 10).unwrap();
            writeln!(tx, "最大输出时间: {}us", performance.max_output_time * 10).unwrap();
            writeln!(tx, "总处理周期: {}", performance.total_cycles).unwrap();
            
            // 显示波形特性分析
            writeln!(tx, "\n--- 波形分析 ---").unwrap();
            let max_sample = *sample_buffer.iter().max().unwrap_or(&0);
            let min_sample = *sample_buffer.iter().min().unwrap_or(&0);
            let peak_to_peak = max_sample.saturating_sub(min_sample);
            let avg_sample = sample_buffer.iter().map(|&x| x as u32).sum::<u32>() / BUFFER_SIZE as u32;
            
            writeln!(tx, "峰峰值: {} ({}mV)", peak_to_peak, 
                     dac_to_voltage(peak_to_peak, VREF_MV).abs()).unwrap();
            writeln!(tx, "平均值: {} ({}mV)", avg_sample, 
                     dac_to_voltage(avg_sample as u16, VREF_MV)).unwrap();
            writeln!(tx, "最大值: {} ({}mV)", max_sample, 
                     dac_to_voltage(max_sample, VREF_MV)).unwrap();
            writeln!(tx, "最小值: {} ({}mV)", min_sample, 
                     dac_to_voltage(min_sample, VREF_MV)).unwrap();
            
            // 显示波形特征
            match state.current_waveform {
                WaveformType::Sine => {
                    writeln!(tx, "波形特征: 平滑正弦波，适合音频测试").unwrap();
                },
                WaveformType::Square => {
                    writeln!(tx, "波形特征: 方波，适合数字电路测试").unwrap();
                },
                WaveformType::Triangle => {
                    writeln!(tx, "波形特征: 三角波，线性上升下降").unwrap();
                },
                WaveformType::Sawtooth => {
                    writeln!(tx, "波形特征: 锯齿波，适合扫描应用").unwrap();
                },
                WaveformType::Pulse => {
                    writeln!(tx, "波形特征: 脉冲波，可调占空比").unwrap();
                },
                WaveformType::Noise => {
                    writeln!(tx, "波形特征: 随机噪声，适合噪声测试").unwrap();
                },
                WaveformType::DC => {
                    writeln!(tx, "波形特征: 直流电平，恒定输出").unwrap();
                },
                WaveformType::Arbitrary => {
                    writeln!(tx, "波形特征: 任意波形，用户定义").unwrap();
                },
            }
            
            // 显示系统状态
            writeln!(tx, "\n--- 系统状态 ---").unwrap();
            writeln!(tx, "定时器频率: {}Hz", SAMPLE_RATE).unwrap();
            writeln!(tx, "缓冲区大小: {}", BUFFER_SIZE).unwrap();
            writeln!(tx, "缓冲区索引: {}", buffer_index).unwrap();
            writeln!(tx, "系统时钟: {}MHz", clocks.sysclk().0 / 1_000_000).unwrap();
            
            // 显示下一个波形预告
            let next_waveform = match state.current_waveform {
                WaveformType::Sine => WaveformType::Square,
                WaveformType::Square => WaveformType::Triangle,
                WaveformType::Triangle => WaveformType::Sawtooth,
                WaveformType::Sawtooth => WaveformType::Pulse,
                WaveformType::Pulse => WaveformType::Noise,
                WaveformType::Noise => WaveformType::DC,
                WaveformType::DC => WaveformType::Sine,
                WaveformType::Arbitrary => WaveformType::Sine,
            };
            
            let time_to_switch = 1000000 - (sample_count - state.last_switch_time);
            let seconds_to_switch = time_to_switch / SAMPLE_RATE;
            
            writeln!(tx, "\n--- 下一个波形 ---").unwrap();
            writeln!(tx, "下一个波形: {:?}", next_waveform).unwrap();
            writeln!(tx, "切换倒计时: {}秒", seconds_to_switch).unwrap();
        }
    }
}

// 模拟复杂波形生成（用于测试）
fn generate_complex_waveform(time: u32, waveform_type: WaveformType) -> u16 {
    let t = (time as f32) * 0.0001; // 时间缩放
    
    match waveform_type {
        WaveformType::Sine => {
            let sine = (2.0 * 3.14159 * t).sin();
            ((sine + 1.0) * 2047.5) as u16
        },
        WaveformType::Square => {
            let square = if (t * 1000.0) as u32 % 2 == 0 { 1.0 } else { -1.0 };
            ((square + 1.0) * 2047.5) as u16
        },
        WaveformType::Triangle => {
            let triangle = 2.0 * ((t * 1000.0) % 1.0) - 1.0;
            ((triangle + 1.0) * 2047.5) as u16
        },
        WaveformType::Sawtooth => {
            let sawtooth = (t * 1000.0) % 1.0;
            (sawtooth * 4095.0) as u16
        },
        WaveformType::Pulse => {
            let pulse = if ((t * 1000.0) % 1.0) < 0.25 { 1.0 } else { -1.0 };
            ((pulse + 1.0) * 2047.5) as u16
        },
        WaveformType::Noise => {
            let seed = (t * 54321.0) as u32;
            let noise = ((seed.wrapping_mul(1103515245).wrapping_add(12345)) >> 16) as f32 / 32768.0;
            (noise * 4095.0) as u16
        },
        WaveformType::DC => 2048, // 中点直流
        WaveformType::Arbitrary => {
            // 复合波形：基频 + 3次谐波
            let fundamental = (2.0 * 3.14159 * t).sin();
            let harmonic = 0.3 * (6.0 * 3.14159 * t).sin();
            let composite = fundamental + harmonic;
            ((composite + 1.0) * 2047.5) as u16
        },
    }
}