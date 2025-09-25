#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    adc::{Adc, config::AdcConfig},
    gpio::{Analog, Pin},
    serial::{config::Config, Serial},
    timer::{Timer, Event},
};
use nb::block;
use oscilloscope::{
    OscilloscopeBuffer, TriggerDetector, TriggerConfig, TriggerType,
    TimebaseConfig, ChannelConfig, OscilloscopeMeasurement, DisplayBuffer,
    adc_to_voltage
};

// 示波器配置常量
const BUFFER_SIZE: usize = 1000;
const DISPLAY_WIDTH: usize = 128;
const DISPLAY_HEIGHT: usize = 64;
const VREF_MV: u16 = 3300;

// 示波器统计信息
struct OscilloscopeStatistics {
    samples_captured: u32,
    trigger_events: u32,
    measurement_cycles: u32,
    last_frequency: Option<u32>,
    last_amplitude: u16,
    last_dc_offset: u16,
}

impl OscilloscopeStatistics {
    fn new() -> Self {
        Self {
            samples_captured: 0,
            trigger_events: 0,
            measurement_cycles: 0,
            last_frequency: None,
            last_amplitude: 0,
            last_dc_offset: 0,
        }
    }

    fn update_measurements(&mut self, buffer: &OscilloscopeBuffer<BUFFER_SIZE>) {
        if buffer.len() > 0 {
            self.last_amplitude = OscilloscopeMeasurement::peak_to_peak(&buffer.data);
            self.last_dc_offset = OscilloscopeMeasurement::average(&buffer.data);
            self.last_frequency = OscilloscopeMeasurement::frequency(&buffer.data, buffer.sample_rate);
            self.measurement_cycles += 1;
        }
    }
}

// 示波器性能监控
struct OscilloscopePerformance {
    acquisition_time: u32,
    processing_time: u32,
    display_time: u32,
    total_cycles: u32,
    max_acquisition_time: u32,
    max_processing_time: u32,
}

impl OscilloscopePerformance {
    fn new() -> Self {
        Self {
            acquisition_time: 0,
            processing_time: 0,
            display_time: 0,
            total_cycles: 0,
            max_acquisition_time: 0,
            max_processing_time: 0,
        }
    }

    fn update_acquisition_time(&mut self, time: u32) {
        self.acquisition_time = time;
        if time > self.max_acquisition_time {
            self.max_acquisition_time = time;
        }
    }

    fn update_processing_time(&mut self, time: u32) {
        self.processing_time = time;
        if time > self.max_processing_time {
            self.max_processing_time = time;
        }
        self.total_cycles += 1;
    }

    fn get_average_acquisition_time(&self) -> u32 {
        if self.total_cycles > 0 {
            self.acquisition_time / self.total_cycles
        } else {
            0
        }
    }

    fn get_average_processing_time(&self) -> u32 {
        if self.total_cycles > 0 {
            self.processing_time / self.total_cycles
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
    let gpiob = dp.GPIOB.split();

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

    // 配置ADC引脚
    let adc_pin = gpioa.pa0.into_analog();

    // 配置ADC
    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());

    // 配置定时器用于采样触发
    let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer.start(100.khz()).unwrap(); // 100kHz采样率
    timer.listen(Event::Update);

    // 初始化示波器组件
    let mut buffer = OscilloscopeBuffer::<BUFFER_SIZE>::new();
    buffer.sample_rate = 100000; // 100kHz

    let trigger_config = TriggerConfig {
        trigger_type: TriggerType::Rising,
        threshold: 2048, // 中点触发
        hysteresis: 100,
        channel: 0,
        pre_trigger: 100,
        post_trigger: 900,
    };
    let mut trigger_detector = TriggerDetector::new(trigger_config);

    let timebase_config = TimebaseConfig::default();
    let channel_config = ChannelConfig::default();
    
    let mut display_buffer = DisplayBuffer::<DISPLAY_WIDTH, DISPLAY_HEIGHT>::new();
    let mut statistics = OscilloscopeStatistics::new();
    let mut performance = OscilloscopePerformance::new();

    // 状态变量
    let mut sample_count = 0u32;
    let mut last_status_time = 0u32;
    let mut acquisition_active = false;
    let mut acquisition_start_time = 0u32;

    writeln!(tx, "基础示波器启动").unwrap();
    writeln!(tx, "采样率: {}Hz", buffer.sample_rate).unwrap();
    writeln!(tx, "缓冲区大小: {}", BUFFER_SIZE).unwrap();
    writeln!(tx, "触发类型: {:?}", trigger_config.trigger_type).unwrap();

    loop {
        // 检查定时器事件
        if timer.wait().is_ok() {
            let current_time = sample_count;
            
            // 开始数据采集
            if !acquisition_active {
                acquisition_start_time = current_time;
                acquisition_active = true;
                buffer.clear();
                trigger_detector.reset();
            }

            // 读取ADC值
            let adc_value: u16 = adc.read(&adc_pin).unwrap();
            
            // 处理触发
            let trigger_result = trigger_detector.process_sample(adc_value);
            
            match trigger_result {
                oscilloscope::TriggerResult::Triggered => {
                    statistics.trigger_events += 1;
                    buffer.trigger_position = trigger_detector.get_trigger_position();
                },
                oscilloscope::TriggerResult::Complete => {
                    // 采集完成，进行数据处理
                    let acquisition_time = current_time - acquisition_start_time;
                    performance.update_acquisition_time(acquisition_time);
                    
                    let processing_start = current_time;
                    
                    // 更新统计信息
                    statistics.update_measurements(&buffer);
                    statistics.samples_captured += buffer.len() as u32;
                    
                    // 更新显示缓冲区
                    display_buffer.clear();
                    display_buffer.draw_grid();
                    display_buffer.draw_waveform(&buffer.data, 0);
                    display_buffer.draw_cursor();
                    
                    let processing_time = current_time - processing_start;
                    performance.update_processing_time(processing_time);
                    
                    acquisition_active = false;
                },
                oscilloscope::TriggerResult::Continue => {
                    // 继续采集数据
                    if !buffer.add_sample(adc_value) {
                        // 缓冲区满，强制完成采集
                        acquisition_active = false;
                    }
                },
            }

            sample_count += 1;
        }

        // 定期输出状态信息
        if sample_count.wrapping_sub(last_status_time) >= 500000 { // 每5秒
            last_status_time = sample_count;
            
            writeln!(tx, "\n=== 示波器状态 ===").unwrap();
            writeln!(tx, "运行时间: {}s", sample_count / 100000).unwrap();
            writeln!(tx, "采样计数: {}", statistics.samples_captured).unwrap();
            writeln!(tx, "触发事件: {}", statistics.trigger_events).unwrap();
            writeln!(tx, "测量周期: {}", statistics.measurement_cycles).unwrap();
            
            // 显示测量结果
            writeln!(tx, "\n--- 信号测量 ---").unwrap();
            let amplitude_mv = adc_to_voltage(statistics.last_amplitude, VREF_MV);
            let dc_offset_mv = adc_to_voltage(statistics.last_dc_offset, VREF_MV);
            
            writeln!(tx, "幅度: {}mV (ADC: {})", amplitude_mv, statistics.last_amplitude).unwrap();
            writeln!(tx, "直流偏移: {}mV (ADC: {})", dc_offset_mv, statistics.last_dc_offset).unwrap();
            
            if let Some(freq) = statistics.last_frequency {
                writeln!(tx, "频率: {}Hz", freq).unwrap();
            } else {
                writeln!(tx, "频率: 无法测量").unwrap();
            }
            
            // 显示性能信息
            writeln!(tx, "\n--- 性能统计 ---").unwrap();
            writeln!(tx, "平均采集时间: {}us", performance.get_average_acquisition_time() * 10).unwrap();
            writeln!(tx, "平均处理时间: {}us", performance.get_average_processing_time() * 10).unwrap();
            writeln!(tx, "最大采集时间: {}us", performance.max_acquisition_time * 10).unwrap();
            writeln!(tx, "最大处理时间: {}us", performance.max_processing_time * 10).unwrap();
            
            // 显示触发状态
            writeln!(tx, "\n--- 触发状态 ---").unwrap();
            writeln!(tx, "触发类型: {:?}", trigger_config.trigger_type).unwrap();
            writeln!(tx, "触发阈值: {}mV", adc_to_voltage(trigger_config.threshold, VREF_MV)).unwrap();
            writeln!(tx, "触发完成: {}", trigger_detector.is_complete()).unwrap();
            
            // 显示时基信息
            writeln!(tx, "\n--- 时基设置 ---").unwrap();
            writeln!(tx, "采样率: {}Hz", timebase_config.sample_rate).unwrap();
            writeln!(tx, "时间/格: {}us", timebase_config.time_per_div).unwrap();
            writeln!(tx, "记录长度: {}", timebase_config.record_length).unwrap();
            
            // 显示通道信息
            writeln!(tx, "\n--- 通道设置 ---").unwrap();
            writeln!(tx, "通道使能: {}", channel_config.enabled).unwrap();
            writeln!(tx, "电压/格: {}mV", channel_config.voltage_per_div).unwrap();
            writeln!(tx, "耦合方式: {:?}", channel_config.coupling).unwrap();
            writeln!(tx, "探头比例: {}x", channel_config.probe_ratio).unwrap();
            
            // 显示缓冲区状态
            writeln!(tx, "\n--- 缓冲区状态 ---").unwrap();
            writeln!(tx, "缓冲区大小: {}/{}", buffer.len(), BUFFER_SIZE).unwrap();
            writeln!(tx, "缓冲区满: {}", buffer.is_full()).unwrap();
            if let Some(pos) = buffer.trigger_position {
                writeln!(tx, "触发位置: {}", pos).unwrap();
            }
            
            // 显示显示缓冲区信息
            writeln!(tx, "\n--- 显示信息 ---").unwrap();
            writeln!(tx, "显示分辨率: {}x{}", DISPLAY_WIDTH, DISPLAY_HEIGHT).unwrap();
            writeln!(tx, "网格使能: {}", display_buffer.grid_enabled).unwrap();
            writeln!(tx, "光标位置: ({}, {})", display_buffer.cursor_x, display_buffer.cursor_y).unwrap();
        }
    }
}

// 模拟复杂信号生成（用于测试）
fn generate_test_signal(time: u32, signal_type: u8) -> u16 {
    let t = (time as f32) * 0.001; // 时间缩放
    
    match signal_type {
        0 => {
            // 正弦波 + 噪声
            let sine = (2.0 * 3.14159 * 1000.0 * t).sin();
            let noise = ((time * 17) % 100) as f32 / 100.0 - 0.5;
            ((sine * 0.8 + noise * 0.1 + 1.0) * 2047.0) as u16
        },
        1 => {
            // 方波
            let square = if (t * 1000.0) as u32 % 2 == 0 { 1.0 } else { -1.0 };
            ((square + 1.0) * 2047.0) as u16
        },
        2 => {
            // 三角波
            let triangle = 2.0 * ((t * 1000.0) % 1.0) - 1.0;
            ((triangle + 1.0) * 2047.0) as u16
        },
        _ => {
            // 锯齿波
            let sawtooth = (t * 1000.0) % 1.0;
            (sawtooth * 4095.0) as u16
        }
    }
}