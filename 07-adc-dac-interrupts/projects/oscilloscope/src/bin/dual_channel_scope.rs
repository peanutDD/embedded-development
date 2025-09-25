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
    MultiChannelBuffer, TriggerDetector, TriggerConfig, TriggerType,
    TimebaseConfig, ChannelConfig, OscilloscopeMeasurement, DisplayBuffer,
    adc_to_voltage
};

// 双通道示波器配置常量
const CHANNELS: usize = 2;
const BUFFER_SIZE: usize = 1000;
const DISPLAY_WIDTH: usize = 128;
const DISPLAY_HEIGHT: usize = 64;
const VREF_MV: u16 = 3300;

// 双通道统计信息
struct DualChannelStatistics {
    channel_stats: [ChannelStats; CHANNELS],
    cross_channel_correlation: f32,
    phase_difference: f32,
    amplitude_ratio: f32,
    measurement_cycles: u32,
}

struct ChannelStats {
    samples_captured: u32,
    last_frequency: Option<u32>,
    last_amplitude: u16,
    last_dc_offset: u16,
    last_rms: u16,
    last_duty_cycle: u8,
    peak_count: u16,
}

impl ChannelStats {
    fn new() -> Self {
        Self {
            samples_captured: 0,
            last_frequency: None,
            last_amplitude: 0,
            last_dc_offset: 0,
            last_rms: 0,
            last_duty_cycle: 0,
            peak_count: 0,
        }
    }

    fn update_measurements(&mut self, data: &[u16], sample_rate: u32) {
        if !data.is_empty() {
            self.last_amplitude = OscilloscopeMeasurement::peak_to_peak(data);
            self.last_dc_offset = OscilloscopeMeasurement::average(data);
            self.last_rms = OscilloscopeMeasurement::rms(data);
            self.last_frequency = OscilloscopeMeasurement::frequency(data, sample_rate);
            self.last_duty_cycle = OscilloscopeMeasurement::duty_cycle(data);
            self.samples_captured += data.len() as u32;
            
            // 计算峰值数量
            let avg = self.last_dc_offset;
            let threshold = avg + self.last_amplitude / 4;
            self.peak_count = count_peaks(data, threshold);
        }
    }
}

impl DualChannelStatistics {
    fn new() -> Self {
        Self {
            channel_stats: [ChannelStats::new(), ChannelStats::new()],
            cross_channel_correlation: 0.0,
            phase_difference: 0.0,
            amplitude_ratio: 0.0,
            measurement_cycles: 0,
        }
    }

    fn update_measurements(&mut self, buffer: &MultiChannelBuffer<CHANNELS, BUFFER_SIZE>) {
        // 更新各通道统计
        for (i, channel) in buffer.channels.iter().enumerate() {
            if buffer.is_channel_enabled(i) && !channel.data.is_empty() {
                self.channel_stats[i].update_measurements(&channel.data, channel.sample_rate);
            }
        }

        // 计算跨通道分析
        if buffer.is_channel_enabled(0) && buffer.is_channel_enabled(1) {
            let ch0_data = &buffer.channels[0].data;
            let ch1_data = &buffer.channels[1].data;
            
            if !ch0_data.is_empty() && !ch1_data.is_empty() {
                self.cross_channel_correlation = calculate_correlation(ch0_data, ch1_data);
                self.phase_difference = calculate_phase_difference(ch0_data, ch1_data);
                
                let amp0 = self.channel_stats[0].last_amplitude;
                let amp1 = self.channel_stats[1].last_amplitude;
                if amp1 > 0 {
                    self.amplitude_ratio = amp0 as f32 / amp1 as f32;
                }
            }
        }

        self.measurement_cycles += 1;
    }
}

// 双通道性能监控
struct DualChannelPerformance {
    acquisition_time: u32,
    processing_time: u32,
    channel_switch_time: u32,
    total_cycles: u32,
    max_acquisition_time: u32,
    channel_balance: [u32; CHANNELS], // 每个通道的处理时间
}

impl DualChannelPerformance {
    fn new() -> Self {
        Self {
            acquisition_time: 0,
            processing_time: 0,
            channel_switch_time: 0,
            total_cycles: 0,
            max_acquisition_time: 0,
            channel_balance: [0; CHANNELS],
        }
    }

    fn update_performance(&mut self, acq_time: u32, proc_time: u32, switch_time: u32) {
        self.acquisition_time = acq_time;
        self.processing_time = proc_time;
        self.channel_switch_time = switch_time;
        
        if acq_time > self.max_acquisition_time {
            self.max_acquisition_time = acq_time;
        }
        
        self.total_cycles += 1;
    }

    fn update_channel_balance(&mut self, channel: usize, time: u32) {
        if channel < CHANNELS {
            self.channel_balance[channel] += time;
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

    // 配置双通道ADC引脚
    let adc_pin0 = gpioa.pa0.into_analog(); // 通道0
    let adc_pin1 = gpioa.pa1.into_analog(); // 通道1

    // 配置ADC
    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());

    // 配置定时器用于采样触发
    let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer.start(200.khz()).unwrap(); // 200kHz采样率，每通道100kHz
    timer.listen(Event::Update);

    // 初始化双通道示波器组件
    let mut buffer = MultiChannelBuffer::<CHANNELS, BUFFER_SIZE>::new();
    buffer.enable_channel(0, true);
    buffer.enable_channel(1, true);
    
    // 设置采样率
    for channel in &mut buffer.channels {
        channel.sample_rate = 100000; // 每通道100kHz
    }

    let trigger_config = TriggerConfig {
        trigger_type: TriggerType::Rising,
        threshold: 2048,
        hysteresis: 100,
        channel: 0, // 通道0作为触发源
        pre_trigger: 100,
        post_trigger: 900,
    };
    let mut trigger_detector = TriggerDetector::new(trigger_config);

    let timebase_config = TimebaseConfig::default();
    let mut channel_configs = [ChannelConfig::default(); CHANNELS];
    
    // 配置通道参数
    channel_configs[0].voltage_per_div = 1000; // 1V/div
    channel_configs[1].voltage_per_div = 500;  // 0.5V/div
    
    let mut display_buffer = DisplayBuffer::<DISPLAY_WIDTH, DISPLAY_HEIGHT>::new();
    let mut statistics = DualChannelStatistics::new();
    let mut performance = DualChannelPerformance::new();

    // 状态变量
    let mut sample_count = 0u32;
    let mut last_status_time = 0u32;
    let mut current_channel = 0usize;
    let mut acquisition_active = false;
    let mut acquisition_start_time = 0u32;

    writeln!(tx, "双通道示波器启动").unwrap();
    writeln!(tx, "通道数: {}", CHANNELS).unwrap();
    writeln!(tx, "每通道采样率: {}Hz", buffer.channels[0].sample_rate).unwrap();
    writeln!(tx, "缓冲区大小: {}", BUFFER_SIZE).unwrap();

    loop {
        // 检查定时器事件
        if timer.wait().is_ok() {
            let current_time = sample_count;
            let channel_start_time = current_time;
            
            // 开始数据采集
            if !acquisition_active {
                acquisition_start_time = current_time;
                acquisition_active = true;
                buffer.clear_all();
                trigger_detector.reset();
            }

            // 读取当前通道的ADC值
            let adc_value: u16 = match current_channel {
                0 => adc.read(&adc_pin0).unwrap(),
                1 => adc.read(&adc_pin1).unwrap(),
                _ => 0,
            };

            // 处理触发（仅在触发通道）
            let trigger_result = if current_channel == trigger_config.channel as usize {
                trigger_detector.process_sample(adc_value)
            } else {
                oscilloscope::TriggerResult::Continue
            };
            
            match trigger_result {
                oscilloscope::TriggerResult::Triggered => {
                    // 设置所有通道的触发位置
                    for channel in &mut buffer.channels {
                        channel.trigger_position = trigger_detector.get_trigger_position();
                    }
                },
                oscilloscope::TriggerResult::Complete => {
                    // 采集完成，进行数据处理
                    let acquisition_time = current_time - acquisition_start_time;
                    let processing_start = current_time;
                    
                    // 更新统计信息
                    statistics.update_measurements(&buffer);
                    
                    // 更新显示缓冲区
                    display_buffer.clear();
                    display_buffer.draw_grid();
                    
                    // 绘制两个通道的波形（不同颜色/样式）
                    if buffer.is_channel_enabled(0) {
                        display_buffer.draw_waveform(&buffer.channels[0].data, 0);
                    }
                    if buffer.is_channel_enabled(1) {
                        // 可以在这里实现不同的绘制样式
                        display_buffer.draw_waveform(&buffer.channels[1].data, 1);
                    }
                    
                    display_buffer.draw_cursor();
                    
                    let processing_time = current_time - processing_start;
                    let switch_time = current_time - channel_start_time;
                    
                    performance.update_performance(acquisition_time, processing_time, switch_time);
                    
                    acquisition_active = false;
                },
                oscilloscope::TriggerResult::Continue => {
                    // 继续采集数据
                    let samples = [adc_value, 0]; // 只有当前通道有数据
                    let mut channel_samples = [0u16; CHANNELS];
                    channel_samples[current_channel] = adc_value;
                    
                    if !buffer.add_samples(&channel_samples) {
                        // 缓冲区满，强制完成采集
                        acquisition_active = false;
                    }
                },
            }

            // 更新通道平衡统计
            let channel_time = current_time - channel_start_time;
            performance.update_channel_balance(current_channel, channel_time);

            // 切换到下一个通道
            current_channel = (current_channel + 1) % CHANNELS;
            sample_count += 1;
        }

        // 定期输出状态信息
        if sample_count.wrapping_sub(last_status_time) >= 1000000 { // 每10秒
            last_status_time = sample_count;
            
            writeln!(tx, "\n=== 双通道示波器状态 ===").unwrap();
            writeln!(tx, "运行时间: {}s", sample_count / 200000).unwrap();
            writeln!(tx, "测量周期: {}", statistics.measurement_cycles).unwrap();
            
            // 显示各通道测量结果
            for (i, channel_stat) in statistics.channel_stats.iter().enumerate() {
                if buffer.is_channel_enabled(i) {
                    writeln!(tx, "\n--- 通道 {} ---", i).unwrap();
                    
                    let amplitude_mv = adc_to_voltage(channel_stat.last_amplitude, VREF_MV);
                    let dc_offset_mv = adc_to_voltage(channel_stat.last_dc_offset, VREF_MV);
                    let rms_mv = adc_to_voltage(channel_stat.last_rms, VREF_MV);
                    
                    writeln!(tx, "采样数: {}", channel_stat.samples_captured).unwrap();
                    writeln!(tx, "幅度: {}mV", amplitude_mv).unwrap();
                    writeln!(tx, "直流偏移: {}mV", dc_offset_mv).unwrap();
                    writeln!(tx, "RMS: {}mV", rms_mv).unwrap();
                    writeln!(tx, "占空比: {}%", channel_stat.last_duty_cycle).unwrap();
                    writeln!(tx, "峰值数: {}", channel_stat.peak_count).unwrap();
                    
                    if let Some(freq) = channel_stat.last_frequency {
                        writeln!(tx, "频率: {}Hz", freq).unwrap();
                    } else {
                        writeln!(tx, "频率: 无法测量").unwrap();
                    }
                    
                    // 显示通道配置
                    writeln!(tx, "电压/格: {}mV", channel_configs[i].voltage_per_div).unwrap();
                    writeln!(tx, "耦合: {:?}", channel_configs[i].coupling).unwrap();
                    writeln!(tx, "探头比例: {}x", channel_configs[i].probe_ratio).unwrap();
                }
            }
            
            // 显示跨通道分析
            if buffer.is_channel_enabled(0) && buffer.is_channel_enabled(1) {
                writeln!(tx, "\n--- 跨通道分析 ---").unwrap();
                writeln!(tx, "相关性: {:.3}", statistics.cross_channel_correlation).unwrap();
                writeln!(tx, "相位差: {:.1}°", statistics.phase_difference).unwrap();
                writeln!(tx, "幅度比 (CH0/CH1): {:.2}", statistics.amplitude_ratio).unwrap();
                
                // 信号关系分析
                if statistics.cross_channel_correlation > 0.8 {
                    writeln!(tx, "信号关系: 强正相关").unwrap();
                } else if statistics.cross_channel_correlation < -0.8 {
                    writeln!(tx, "信号关系: 强负相关").unwrap();
                } else if statistics.cross_channel_correlation.abs() < 0.2 {
                    writeln!(tx, "信号关系: 无相关性").unwrap();
                } else {
                    writeln!(tx, "信号关系: 弱相关").unwrap();
                }
            }
            
            // 显示性能信息
            writeln!(tx, "\n--- 性能统计 ---").unwrap();
            writeln!(tx, "平均采集时间: {}us", performance.acquisition_time * 5).unwrap();
            writeln!(tx, "平均处理时间: {}us", performance.processing_time * 5).unwrap();
            writeln!(tx, "通道切换时间: {}us", performance.channel_switch_time * 5).unwrap();
            writeln!(tx, "最大采集时间: {}us", performance.max_acquisition_time * 5).unwrap();
            
            // 显示通道平衡
            writeln!(tx, "通道平衡:").unwrap();
            for (i, &balance) in performance.channel_balance.iter().enumerate() {
                let avg_time = if performance.total_cycles > 0 {
                    balance / performance.total_cycles
                } else {
                    0
                };
                writeln!(tx, "  CH{}: {}us", i, avg_time * 5).unwrap();
            }
            
            // 显示触发状态
            writeln!(tx, "\n--- 触发状态 ---").unwrap();
            writeln!(tx, "触发通道: {}", trigger_config.channel).unwrap();
            writeln!(tx, "触发类型: {:?}", trigger_config.trigger_type).unwrap();
            writeln!(tx, "触发阈值: {}mV", adc_to_voltage(trigger_config.threshold, VREF_MV)).unwrap();
            writeln!(tx, "触发完成: {}", trigger_detector.is_complete()).unwrap();
            
            // 显示缓冲区状态
            writeln!(tx, "\n--- 缓冲区状态 ---").unwrap();
            for (i, channel) in buffer.channels.iter().enumerate() {
                if buffer.is_channel_enabled(i) {
                    writeln!(tx, "CH{}: {}/{} 样本", i, channel.len(), BUFFER_SIZE).unwrap();
                    if let Some(pos) = channel.trigger_position {
                        writeln!(tx, "CH{} 触发位置: {}", i, pos).unwrap();
                    }
                }
            }
        }
    }
}

// 计算两个信号的相关性
fn calculate_correlation(data1: &[u16], data2: &[u16]) -> f32 {
    if data1.len() != data2.len() || data1.is_empty() {
        return 0.0;
    }

    let len = data1.len();
    let mean1 = data1.iter().map(|&x| x as f32).sum::<f32>() / len as f32;
    let mean2 = data2.iter().map(|&x| x as f32).sum::<f32>() / len as f32;

    let mut numerator = 0.0;
    let mut sum_sq1 = 0.0;
    let mut sum_sq2 = 0.0;

    for i in 0..len {
        let diff1 = data1[i] as f32 - mean1;
        let diff2 = data2[i] as f32 - mean2;
        
        numerator += diff1 * diff2;
        sum_sq1 += diff1 * diff1;
        sum_sq2 += diff2 * diff2;
    }

    let denominator = (sum_sq1 * sum_sq2).sqrt();
    if denominator > 0.0 {
        numerator / denominator
    } else {
        0.0
    }
}

// 计算两个信号的相位差
fn calculate_phase_difference(data1: &[u16], data2: &[u16]) -> f32 {
    if data1.len() != data2.len() || data1.len() < 4 {
        return 0.0;
    }

    // 简化的相位差计算：找到过零点的时间差
    let avg1 = OscilloscopeMeasurement::average(data1);
    let avg2 = OscilloscopeMeasurement::average(data2);

    let mut zero_cross1 = None;
    let mut zero_cross2 = None;

    // 找到第一个上升沿过零点
    for i in 1..data1.len() {
        if zero_cross1.is_none() && data1[i-1] < avg1 && data1[i] >= avg1 {
            zero_cross1 = Some(i);
        }
        if zero_cross2.is_none() && data2[i-1] < avg2 && data2[i] >= avg2 {
            zero_cross2 = Some(i);
        }
        if zero_cross1.is_some() && zero_cross2.is_some() {
            break;
        }
    }

    if let (Some(cross1), Some(cross2)) = (zero_cross1, zero_cross2) {
        let time_diff = cross1 as i32 - cross2 as i32;
        // 假设一个周期大约是数据长度的1/4，转换为角度
        let phase_diff = (time_diff as f32 / data1.len() as f32) * 360.0 * 4.0;
        phase_diff.abs().min(180.0) // 限制在±180度内
    } else {
        0.0
    }
}

// 计算峰值数量
fn count_peaks(data: &[u16], threshold: u16) -> u16 {
    if data.len() < 3 {
        return 0;
    }

    let mut peaks = 0;
    let mut last_above = data[0] > threshold;

    for i in 1..data.len() {
        let current_above = data[i] > threshold;
        if !last_above && current_above {
            peaks += 1;
        }
        last_above = current_above;
    }

    peaks
}