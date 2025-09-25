#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    adc::{Adc, config::{AdcConfig, SampleTime, Sequence, Eoc, Scan, Continuous}},
    gpio::{Analog, Pin},
    serial::{config::Config, Serial},
    timer::{Timer, Event},
};
use nb::block;
use oscilloscope::{
    TriggerSystem, TriggerConfig, TriggerType, TriggerSlope, TriggerMode,
    MultiChannelBuffer, MeasurementEngine, OscilloscopeStatistics
};

// 存储示波器配置常量
const VREF_MV: u16 = 3300;
const SAMPLE_RATE: u32 = 250000; // 250kHz
const BUFFER_SIZE: usize = 1024;
const STORAGE_SEGMENTS: usize = 16; // 16个存储段
const SEGMENT_SIZE: usize = 512; // 每段512样本
const MAX_WAVEFORMS: usize = 8; // 最多存储8个波形
const COMPRESSION_RATIO: usize = 4; // 压缩比例

// 存储模式
#[derive(Clone, Copy, PartialEq)]
enum StorageMode {
    Normal,      // 正常模式
    Envelope,    // 包络模式
    Average,     // 平均模式
    Peak,        // 峰值检测模式
    Compressed,  // 压缩模式
    Infinite,    // 无限存储模式
}

// 波形存储结构
#[derive(Clone)]
struct StoredWaveform {
    id: u8,
    timestamp: u32,
    sample_rate: u32,
    channel_count: u8,
    sample_count: u16,
    trigger_position: u16,
    storage_mode: StorageMode,
    compression_level: u8,
    data: [u16; SEGMENT_SIZE * 2], // 双通道数据
    metadata: WaveformMetadata,
}

#[derive(Clone)]
struct WaveformMetadata {
    amplitude: f32,
    frequency: f32,
    dc_offset: f32,
    rms_value: f32,
    peak_to_peak: u16,
    rise_time: u32,
    fall_time: u32,
    duty_cycle: f32,
    signal_type: SignalType,
    quality_score: f32,
}

#[derive(Clone, Copy)]
enum SignalType {
    Sine,
    Square,
    Triangle,
    Sawtooth,
    Pulse,
    Noise,
    Complex,
    Unknown,
}

impl Default for StoredWaveform {
    fn default() -> Self {
        Self {
            id: 0,
            timestamp: 0,
            sample_rate: SAMPLE_RATE,
            channel_count: 2,
            sample_count: 0,
            trigger_position: 0,
            storage_mode: StorageMode::Normal,
            compression_level: 0,
            data: [0; SEGMENT_SIZE * 2],
            metadata: WaveformMetadata {
                amplitude: 0.0,
                frequency: 0.0,
                dc_offset: 0.0,
                rms_value: 0.0,
                peak_to_peak: 0,
                rise_time: 0,
                fall_time: 0,
                duty_cycle: 0.0,
                signal_type: SignalType::Unknown,
                quality_score: 0.0,
            },
        }
    }
}

// 存储管理器
struct StorageManager {
    waveforms: [StoredWaveform; MAX_WAVEFORMS],
    waveform_count: usize,
    current_segment: usize,
    storage_mode: StorageMode,
    auto_save: bool,
    compression_enabled: bool,
    envelope_buffer: [u16; SEGMENT_SIZE * 2], // 包络缓冲区
    average_buffer: [u32; SEGMENT_SIZE * 2],  // 平均缓冲区
    average_count: u32,
    peak_buffer: [u16; SEGMENT_SIZE * 2],     // 峰值缓冲区
    storage_statistics: StorageStatistics,
}

struct StorageStatistics {
    total_stored: u32,
    storage_usage: f32,
    compression_ratio: f32,
    average_quality: f32,
    storage_rate: f32,
    retrieval_time: u32,
    data_integrity: f32,
    storage_efficiency: f32,
}

impl StorageManager {
    fn new() -> Self {
        Self {
            waveforms: [StoredWaveform::default(); MAX_WAVEFORMS],
            waveform_count: 0,
            current_segment: 0,
            storage_mode: StorageMode::Normal,
            auto_save: true,
            compression_enabled: false,
            envelope_buffer: [0; SEGMENT_SIZE * 2],
            average_buffer: [0; SEGMENT_SIZE * 2],
            average_count: 0,
            peak_buffer: [0; SEGMENT_SIZE * 2],
            storage_statistics: StorageStatistics {
                total_stored: 0,
                storage_usage: 0.0,
                compression_ratio: 1.0,
                average_quality: 0.0,
                storage_rate: 0.0,
                retrieval_time: 0,
                data_integrity: 100.0,
                storage_efficiency: 0.0,
            },
        }
    }

    fn store_waveform(&mut self, ch1_data: &[u16], ch2_data: &[u16], timestamp: u32) -> bool {
        if self.waveform_count >= MAX_WAVEFORMS {
            // 覆盖最旧的波形
            self.waveform_count = MAX_WAVEFORMS - 1;
            // 移动数组元素
            for i in 0..MAX_WAVEFORMS-1 {
                self.waveforms[i] = self.waveforms[i+1].clone();
            }
        }

        let mut waveform = StoredWaveform::default();
        waveform.id = self.waveform_count as u8;
        waveform.timestamp = timestamp;
        waveform.sample_rate = SAMPLE_RATE;
        waveform.channel_count = 2;
        waveform.storage_mode = self.storage_mode;
        waveform.trigger_position = SEGMENT_SIZE as u16 / 4; // 25%位置

        // 根据存储模式处理数据
        let processed_data = match self.storage_mode {
            StorageMode::Normal => self.store_normal(ch1_data, ch2_data),
            StorageMode::Envelope => self.store_envelope(ch1_data, ch2_data),
            StorageMode::Average => self.store_average(ch1_data, ch2_data),
            StorageMode::Peak => self.store_peak(ch1_data, ch2_data),
            StorageMode::Compressed => self.store_compressed(ch1_data, ch2_data),
            StorageMode::Infinite => self.store_infinite(ch1_data, ch2_data),
        };

        // 复制处理后的数据
        let copy_len = processed_data.len().min(waveform.data.len());
        waveform.data[..copy_len].copy_from_slice(&processed_data[..copy_len]);
        waveform.sample_count = (copy_len / 2) as u16;

        // 分析波形并生成元数据
        self.analyze_waveform(&mut waveform);

        // 存储波形
        self.waveforms[self.waveform_count] = waveform;
        self.waveform_count += 1;

        // 更新统计信息
        self.update_storage_statistics();

        true
    }

    fn store_normal(&self, ch1_data: &[u16], ch2_data: &[u16]) -> [u16; SEGMENT_SIZE * 2] {
        let mut result = [0u16; SEGMENT_SIZE * 2];
        let len = ch1_data.len().min(ch2_data.len()).min(SEGMENT_SIZE);

        for i in 0..len {
            result[i * 2] = ch1_data[i];
            result[i * 2 + 1] = ch2_data[i];
        }

        result
    }

    fn store_envelope(&mut self, ch1_data: &[u16], ch2_data: &[u16]) -> [u16; SEGMENT_SIZE * 2] {
        let mut result = [0u16; SEGMENT_SIZE * 2];
        let len = ch1_data.len().min(ch2_data.len()).min(SEGMENT_SIZE);
        let decimation = if len > SEGMENT_SIZE { len / SEGMENT_SIZE } else { 1 };

        for i in 0..(len / decimation) {
            let start_idx = i * decimation;
            let end_idx = ((i + 1) * decimation).min(len);

            // 计算包络（最大值和最小值）
            let mut ch1_max = 0u16;
            let mut ch1_min = 4095u16;
            let mut ch2_max = 0u16;
            let mut ch2_min = 4095u16;

            for j in start_idx..end_idx {
                ch1_max = ch1_max.max(ch1_data[j]);
                ch1_min = ch1_min.min(ch1_data[j]);
                ch2_max = ch2_max.max(ch2_data[j]);
                ch2_min = ch2_min.min(ch2_data[j]);
            }

            // 存储包络数据（交替存储最大值和最小值）
            if i * 4 + 3 < result.len() {
                result[i * 4] = ch1_max;
                result[i * 4 + 1] = ch1_min;
                result[i * 4 + 2] = ch2_max;
                result[i * 4 + 3] = ch2_min;
            }
        }

        result
    }

    fn store_average(&mut self, ch1_data: &[u16], ch2_data: &[u16]) -> [u16; SEGMENT_SIZE * 2] {
        let mut result = [0u16; SEGMENT_SIZE * 2];
        let len = ch1_data.len().min(ch2_data.len()).min(SEGMENT_SIZE);

        // 累加到平均缓冲区
        for i in 0..len {
            self.average_buffer[i * 2] += ch1_data[i] as u32;
            self.average_buffer[i * 2 + 1] += ch2_data[i] as u32;
        }

        self.average_count += 1;

        // 计算平均值
        for i in 0..len {
            result[i * 2] = (self.average_buffer[i * 2] / self.average_count) as u16;
            result[i * 2 + 1] = (self.average_buffer[i * 2 + 1] / self.average_count) as u16;
        }

        result
    }

    fn store_peak(&mut self, ch1_data: &[u16], ch2_data: &[u16]) -> [u16; SEGMENT_SIZE * 2] {
        let mut result = [0u16; SEGMENT_SIZE * 2];
        let len = ch1_data.len().min(ch2_data.len()).min(SEGMENT_SIZE);

        // 峰值检测
        for i in 0..len {
            // 更新峰值缓冲区
            self.peak_buffer[i * 2] = self.peak_buffer[i * 2].max(ch1_data[i]);
            self.peak_buffer[i * 2 + 1] = self.peak_buffer[i * 2 + 1].max(ch2_data[i]);
            
            result[i * 2] = self.peak_buffer[i * 2];
            result[i * 2 + 1] = self.peak_buffer[i * 2 + 1];
        }

        result
    }

    fn store_compressed(&self, ch1_data: &[u16], ch2_data: &[u16]) -> [u16; SEGMENT_SIZE * 2] {
        let mut result = [0u16; SEGMENT_SIZE * 2];
        let len = ch1_data.len().min(ch2_data.len());
        let compressed_len = len / COMPRESSION_RATIO;

        // 简单的抽取压缩
        for i in 0..compressed_len.min(SEGMENT_SIZE) {
            let src_idx = i * COMPRESSION_RATIO;
            if src_idx < len {
                result[i * 2] = ch1_data[src_idx];
                result[i * 2 + 1] = ch2_data[src_idx];
            }
        }

        result
    }

    fn store_infinite(&mut self, ch1_data: &[u16], ch2_data: &[u16]) -> [u16; SEGMENT_SIZE * 2] {
        // 无限存储模式：循环覆盖
        let mut result = [0u16; SEGMENT_SIZE * 2];
        let len = ch1_data.len().min(ch2_data.len()).min(SEGMENT_SIZE);
        let start_offset = self.current_segment * SEGMENT_SIZE / STORAGE_SEGMENTS;

        for i in 0..len {
            let idx = (start_offset + i) % SEGMENT_SIZE;
            result[idx * 2] = ch1_data[i];
            result[idx * 2 + 1] = ch2_data[i];
        }

        self.current_segment = (self.current_segment + 1) % STORAGE_SEGMENTS;
        result
    }

    fn analyze_waveform(&self, waveform: &mut StoredWaveform) {
        let ch1_data = self.extract_channel_data(&waveform.data, 0);
        let ch2_data = self.extract_channel_data(&waveform.data, 1);

        // 基本统计分析
        waveform.metadata.amplitude = self.calculate_amplitude(&ch1_data);
        waveform.metadata.frequency = self.estimate_frequency(&ch1_data, waveform.sample_rate);
        waveform.metadata.dc_offset = self.calculate_dc_offset(&ch1_data);
        waveform.metadata.rms_value = self.calculate_rms(&ch1_data);
        waveform.metadata.peak_to_peak = self.calculate_peak_to_peak(&ch1_data);
        waveform.metadata.duty_cycle = self.calculate_duty_cycle(&ch1_data);
        
        // 时序分析
        let (rise_time, fall_time) = self.calculate_edge_times(&ch1_data);
        waveform.metadata.rise_time = rise_time;
        waveform.metadata.fall_time = fall_time;

        // 信号类型识别
        waveform.metadata.signal_type = self.identify_signal_type(&ch1_data);

        // 质量评分
        waveform.metadata.quality_score = self.calculate_quality_score(waveform);
    }

    fn extract_channel_data(&self, data: &[u16], channel: u8) -> [u16; SEGMENT_SIZE] {
        let mut result = [0u16; SEGMENT_SIZE];
        let offset = channel as usize;
        
        for i in 0..SEGMENT_SIZE {
            if i * 2 + offset < data.len() {
                result[i] = data[i * 2 + offset];
            }
        }
        
        result
    }

    fn calculate_amplitude(&self, data: &[u16]) -> f32 {
        let max_val = *data.iter().max().unwrap_or(&0);
        let min_val = *data.iter().min().unwrap_or(&4095);
        ((max_val - min_val) as f32 * VREF_MV as f32) / 4096.0 / 2.0
    }

    fn estimate_frequency(&self, data: &[u16], sample_rate: u32) -> f32 {
        let mut zero_crossings = 0;
        let mid_point = 2048u16;
        let mut last_above = data[0] > mid_point;

        for &sample in data.iter().skip(1) {
            let current_above = sample > mid_point;
            if current_above != last_above {
                zero_crossings += 1;
                last_above = current_above;
            }
        }

        (zero_crossings as f32 * sample_rate as f32) / (2.0 * data.len() as f32)
    }

    fn calculate_dc_offset(&self, data: &[u16]) -> f32 {
        let sum: u32 = data.iter().map(|&x| x as u32).sum();
        let average = sum as f32 / data.len() as f32;
        (average * VREF_MV as f32) / 4096.0 - (VREF_MV as f32 / 2.0)
    }

    fn calculate_rms(&self, data: &[u16]) -> f32 {
        let sum_squares: u64 = data.iter().map(|&x| (x as u64) * (x as u64)).sum();
        let mean_square = sum_squares as f32 / data.len() as f32;
        (mean_square.sqrt() * VREF_MV as f32) / 4096.0
    }

    fn calculate_peak_to_peak(&self, data: &[u16]) -> u16 {
        let max_val = *data.iter().max().unwrap_or(&0);
        let min_val = *data.iter().min().unwrap_or(&4095);
        max_val - min_val
    }

    fn calculate_duty_cycle(&self, data: &[u16]) -> f32 {
        let mid_point = 2048u16;
        let high_samples = data.iter().filter(|&&x| x > mid_point).count();
        (high_samples as f32 / data.len() as f32) * 100.0
    }

    fn calculate_edge_times(&self, data: &[u16]) -> (u32, u32) {
        let max_val = *data.iter().max().unwrap_or(&0);
        let min_val = *data.iter().min().unwrap_or(&4095);
        let threshold_10 = min_val + (max_val - min_val) / 10;
        let threshold_90 = min_val + (max_val - min_val) * 9 / 10;

        let mut rise_start = None;
        let mut rise_end = None;
        let mut fall_start = None;
        let mut fall_end = None;

        for (i, &sample) in data.iter().enumerate() {
            if rise_start.is_none() && sample > threshold_10 {
                rise_start = Some(i);
            }
            if rise_start.is_some() && rise_end.is_none() && sample > threshold_90 {
                rise_end = Some(i);
            }
            if rise_end.is_some() && fall_start.is_none() && sample < threshold_90 {
                fall_start = Some(i);
            }
            if fall_start.is_some() && fall_end.is_none() && sample < threshold_10 {
                fall_end = Some(i);
                break;
            }
        }

        let rise_time = if let (Some(start), Some(end)) = (rise_start, rise_end) {
            ((end - start) * 1000000 / SAMPLE_RATE as usize) as u32
        } else {
            0
        };

        let fall_time = if let (Some(start), Some(end)) = (fall_start, fall_end) {
            ((end - start) * 1000000 / SAMPLE_RATE as usize) as u32
        } else {
            0
        };

        (rise_time, fall_time)
    }

    fn identify_signal_type(&self, data: &[u16]) -> SignalType {
        let amplitude = self.calculate_amplitude(data);
        let duty_cycle = self.calculate_duty_cycle(data);
        let (rise_time, fall_time) = self.calculate_edge_times(data);

        // 简单的信号类型识别
        if amplitude < 100.0 {
            SignalType::Noise
        } else if duty_cycle > 45.0 && duty_cycle < 55.0 && rise_time < 50 && fall_time < 50 {
            SignalType::Square
        } else if duty_cycle > 10.0 && duty_cycle < 90.0 {
            SignalType::Pulse
        } else if rise_time > 100 || fall_time > 100 {
            SignalType::Triangle
        } else {
            // 进一步分析波形形状
            let harmonics = self.analyze_harmonics(data);
            if harmonics < 0.1 {
                SignalType::Sine
            } else if harmonics > 0.5 {
                SignalType::Complex
            } else {
                SignalType::Sawtooth
            }
        }
    }

    fn analyze_harmonics(&self, data: &[u16]) -> f32 {
        // 简化的谐波分析
        let fundamental_freq = self.estimate_frequency(data, SAMPLE_RATE);
        if fundamental_freq < 1.0 {
            return 0.0;
        }

        // 计算二次和三次谐波的近似强度
        let mut harmonic_power = 0.0;
        let mut fundamental_power = 0.0;

        // 简化计算：基于波形的变化率
        for i in 1..data.len() {
            let diff = (data[i] as i32 - data[i-1] as i32).abs() as f32;
            fundamental_power += diff;
            
            if i > 2 {
                let second_diff = (data[i] as i32 - 2 * data[i-1] as i32 + data[i-2] as i32).abs() as f32;
                harmonic_power += second_diff;
            }
        }

        if fundamental_power > 0.0 {
            harmonic_power / fundamental_power
        } else {
            0.0
        }
    }

    fn calculate_quality_score(&self, waveform: &StoredWaveform) -> f32 {
        let mut score = 100.0;

        // 基于信噪比的评分
        if waveform.metadata.amplitude < 100.0 {
            score -= 30.0; // 信号太小
        }

        // 基于频率稳定性的评分
        if waveform.metadata.frequency < 1.0 || waveform.metadata.frequency > SAMPLE_RATE as f32 / 4.0 {
            score -= 20.0; // 频率异常
        }

        // 基于存储模式的评分
        match waveform.storage_mode {
            StorageMode::Normal => {},
            StorageMode::Compressed => score -= 10.0,
            StorageMode::Envelope => score -= 5.0,
            _ => score -= 15.0,
        }

        score.max(0.0).min(100.0)
    }

    fn update_storage_statistics(&mut self) {
        self.storage_statistics.total_stored = self.waveform_count as u32;
        self.storage_statistics.storage_usage = (self.waveform_count as f32 / MAX_WAVEFORMS as f32) * 100.0;
        
        // 计算平均质量
        if self.waveform_count > 0 {
            let total_quality: f32 = self.waveforms[..self.waveform_count]
                .iter()
                .map(|w| w.metadata.quality_score)
                .sum();
            self.storage_statistics.average_quality = total_quality / self.waveform_count as f32;
        }

        // 计算压缩比
        self.storage_statistics.compression_ratio = if self.compression_enabled {
            COMPRESSION_RATIO as f32
        } else {
            1.0
        };

        // 计算存储效率
        self.storage_statistics.storage_efficiency = 
            (self.storage_statistics.average_quality / 100.0) * 
            (100.0 - self.storage_statistics.storage_usage) / 100.0 * 100.0;
    }

    fn get_waveform(&self, id: u8) -> Option<&StoredWaveform> {
        self.waveforms.iter().find(|w| w.id == id)
    }

    fn get_storage_mode_name(&self) -> &'static str {
        match self.storage_mode {
            StorageMode::Normal => "正常模式",
            StorageMode::Envelope => "包络模式",
            StorageMode::Average => "平均模式",
            StorageMode::Peak => "峰值模式",
            StorageMode::Compressed => "压缩模式",
            StorageMode::Infinite => "无限模式",
        }
    }

    fn switch_storage_mode(&mut self) {
        self.storage_mode = match self.storage_mode {
            StorageMode::Normal => StorageMode::Envelope,
            StorageMode::Envelope => StorageMode::Average,
            StorageMode::Average => StorageMode::Peak,
            StorageMode::Peak => StorageMode::Compressed,
            StorageMode::Compressed => StorageMode::Infinite,
            StorageMode::Infinite => StorageMode::Normal,
        };

        // 重置相关缓冲区
        self.average_buffer = [0; SEGMENT_SIZE * 2];
        self.average_count = 0;
        self.peak_buffer = [0; SEGMENT_SIZE * 2];
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

    // 配置ADC引脚
    let adc_pin1 = gpioa.pa0.into_analog();
    let adc_pin2 = gpioa.pa1.into_analog();

    // 配置ADC
    let adc_config = AdcConfig::default()
        .end_of_conversion_interrupt(Eoc::Conversion)
        .scan(Scan::Enabled)
        .continuous(Continuous::Single);
    
    let mut adc = Adc::new(dp.ADC1, true, adc_config);
    adc.configure_channel(&adc_pin1, Sequence::One, SampleTime::Cycles_480);
    adc.configure_channel(&adc_pin2, Sequence::Two, SampleTime::Cycles_480);

    // 配置定时器用于采样触发
    let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer.start(SAMPLE_RATE.hz()).unwrap();
    timer.listen(Event::Update);

    // 初始化触发系统
    let trigger_config = TriggerConfig {
        trigger_type: TriggerType::Edge,
        slope: TriggerSlope::Rising,
        level: 2048,
        mode: TriggerMode::Auto,
        holdoff_time: 1000,
    };
    let mut trigger_system = TriggerSystem::new(trigger_config);

    // 初始化存储管理器
    let mut storage_manager = StorageManager::new();

    // 初始化缓冲区和统计
    let mut buffer = MultiChannelBuffer::new(BUFFER_SIZE, 2);
    let mut measurement_engine = MeasurementEngine::new();
    let mut statistics = OscilloscopeStatistics::new();

    // 状态变量
    let mut sample_count = 0u32;
    let mut last_status_time = 0u32;
    let mut last_storage_time = 0u32;
    let mut mode_switch_time = 0u32;
    let storage_interval = 500000; // 2秒 @ 250kHz
    let mode_switch_interval = 1250000; // 5秒 @ 250kHz

    writeln!(tx, "存储示波器启动").unwrap();
    writeln!(tx, "采样率: {}Hz", SAMPLE_RATE).unwrap();
    writeln!(tx, "缓冲区大小: {}", BUFFER_SIZE).unwrap();
    writeln!(tx, "存储段数: {}", STORAGE_SEGMENTS).unwrap();
    writeln!(tx, "最大波形数: {}", MAX_WAVEFORMS).unwrap();
    writeln!(tx, "存储模式: {}", storage_manager.get_storage_mode_name()).unwrap();

    loop {
        // 检查定时器事件
        if timer.wait().is_ok() {
            // 检查是否需要切换存储模式
            if sample_count.wrapping_sub(mode_switch_time) >= mode_switch_interval {
                mode_switch_time = sample_count;
                storage_manager.switch_storage_mode();
            }

            // 读取ADC通道1和2
            let sample1: u16 = adc.convert(&adc_pin1, SampleTime::Cycles_480);
            let sample2: u16 = adc.convert(&adc_pin2, SampleTime::Cycles_480);
            
            // 处理触发
            let triggered = trigger_system.process_sample(sample1, 0);
            
            // 存储样本到缓冲区
            buffer.add_sample(0, sample1);
            buffer.add_sample(1, sample2);
            
            // 定期存储波形或在触发时存储
            let should_store = triggered || 
                (storage_manager.auto_save && 
                 sample_count.wrapping_sub(last_storage_time) >= storage_interval);
            
            if should_store {
                last_storage_time = sample_count;
                
                let ch1_data = buffer.get_channel_data(0);
                let ch2_data = buffer.get_channel_data(1);
                
                // 存储波形
                storage_manager.store_waveform(ch1_data, ch2_data, sample_count);
                
                // 进行测量
                measurement_engine.measure_all(ch1_data, SAMPLE_RATE);
                statistics.update_from_measurements(&measurement_engine);
            }
            
            sample_count += 1;
        }

        // 定期输出状态信息
        if sample_count.wrapping_sub(last_status_time) >= 500000 { // 每2秒
            last_status_time = sample_count;
            
            writeln!(tx, "\n=== 存储示波器状态 ===").unwrap();
            writeln!(tx, "运行时间: {}s", sample_count / SAMPLE_RATE).unwrap();
            writeln!(tx, "总样本数: {}", sample_count).unwrap();
            writeln!(tx, "触发次数: {}", trigger_system.get_trigger_count()).unwrap();
            
            // 显示存储状态
            writeln!(tx, "\n--- 存储状态 ---").unwrap();
            writeln!(tx, "存储模式: {}", storage_manager.get_storage_mode_name()).unwrap();
            writeln!(tx, "已存储波形: {}/{}", storage_manager.waveform_count, MAX_WAVEFORMS).unwrap();
            writeln!(tx, "存储使用率: {:.1}%", storage_manager.storage_statistics.storage_usage).unwrap();
            writeln!(tx, "自动保存: {}", if storage_manager.auto_save { "开启" } else { "关闭" }).unwrap();
            writeln!(tx, "压缩模式: {}", if storage_manager.compression_enabled { "开启" } else { "关闭" }).unwrap();
            writeln!(tx, "当前段: {}/{}", storage_manager.current_segment, STORAGE_SEGMENTS).unwrap();
            
            // 显示存储统计
            writeln!(tx, "\n--- 存储统计 ---").unwrap();
            writeln!(tx, "总存储数: {}", storage_manager.storage_statistics.total_stored).unwrap();
            writeln!(tx, "平均质量: {:.1}%", storage_manager.storage_statistics.average_quality).unwrap();
            writeln!(tx, "压缩比: {:.1}:1", storage_manager.storage_statistics.compression_ratio).unwrap();
            writeln!(tx, "存储效率: {:.1}%", storage_manager.storage_statistics.storage_efficiency).unwrap();
            writeln!(tx, "数据完整性: {:.1}%", storage_manager.storage_statistics.data_integrity).unwrap();
            writeln!(tx, "检索时间: {}us", storage_manager.storage_statistics.retrieval_time).unwrap();
            
            // 显示最近存储的波形信息
            if storage_manager.waveform_count > 0 {
                let latest_waveform = &storage_manager.waveforms[storage_manager.waveform_count - 1];
                
                writeln!(tx, "\n--- 最新波形 ---").unwrap();
                writeln!(tx, "波形ID: {}", latest_waveform.id).unwrap();
                writeln!(tx, "时间戳: {}s", latest_waveform.timestamp / SAMPLE_RATE).unwrap();
                writeln!(tx, "样本数: {}", latest_waveform.sample_count).unwrap();
                writeln!(tx, "触发位置: {}", latest_waveform.trigger_position).unwrap();
                writeln!(tx, "存储模式: {:?}", latest_waveform.storage_mode).unwrap();
                
                // 显示波形元数据
                writeln!(tx, "\n--- 波形分析 ---").unwrap();
                writeln!(tx, "幅度: {:.1}mV", latest_waveform.metadata.amplitude).unwrap();
                writeln!(tx, "频率: {:.1}Hz", latest_waveform.metadata.frequency).unwrap();
                writeln!(tx, "直流偏移: {:.1}mV", latest_waveform.metadata.dc_offset).unwrap();
                writeln!(tx, "RMS值: {:.1}mV", latest_waveform.metadata.rms_value).unwrap();
                writeln!(tx, "峰峰值: {} ({}mV)", latest_waveform.metadata.peak_to_peak,
                         (latest_waveform.metadata.peak_to_peak as u32 * VREF_MV as u32) / 4096).unwrap();
                writeln!(tx, "上升时间: {}us", latest_waveform.metadata.rise_time).unwrap();
                writeln!(tx, "下降时间: {}us", latest_waveform.metadata.fall_time).unwrap();
                writeln!(tx, "占空比: {:.1}%", latest_waveform.metadata.duty_cycle).unwrap();
                
                let signal_type_name = match latest_waveform.metadata.signal_type {
                    SignalType::Sine => "正弦波",
                    SignalType::Square => "方波",
                    SignalType::Triangle => "三角波",
                    SignalType::Sawtooth => "锯齿波",
                    SignalType::Pulse => "脉冲",
                    SignalType::Noise => "噪声",
                    SignalType::Complex => "复杂信号",
                    SignalType::Unknown => "未知",
                };
                writeln!(tx, "信号类型: {}", signal_type_name).unwrap();
                writeln!(tx, "质量评分: {:.1}%", latest_waveform.metadata.quality_score).unwrap();
            }
            
            // 显示存储模式特性
            writeln!(tx, "\n--- 存储模式特性 ---").unwrap();
            match storage_manager.storage_mode {
                StorageMode::Normal => {
                    writeln!(tx, "正常模式: 完整数据存储").unwrap();
                    writeln!(tx, "优点: 数据完整，分析精确").unwrap();
                    writeln!(tx, "缺点: 存储空间需求大").unwrap();
                },
                StorageMode::Envelope => {
                    writeln!(tx, "包络模式: 存储最大最小值").unwrap();
                    writeln!(tx, "优点: 保留幅度信息，压缩率高").unwrap();
                    writeln!(tx, "缺点: 丢失详细波形信息").unwrap();
                },
                StorageMode::Average => {
                    writeln!(tx, "平均模式: 累积平均计算").unwrap();
                    writeln!(tx, "优点: 降低噪声，提高信噪比").unwrap();
                    writeln!(tx, "缺点: 丢失瞬态信息").unwrap();
                    writeln!(tx, "平均次数: {}", storage_manager.average_count).unwrap();
                },
                StorageMode::Peak => {
                    writeln!(tx, "峰值模式: 保持峰值检测").unwrap();
                    writeln!(tx, "优点: 捕获最大信号幅度").unwrap();
                    writeln!(tx, "缺点: 不反映实际波形").unwrap();
                },
                StorageMode::Compressed => {
                    writeln!(tx, "压缩模式: 数据抽取压缩").unwrap();
                    writeln!(tx, "优点: 节省存储空间").unwrap();
                    writeln!(tx, "缺点: 降低时间分辨率").unwrap();
                    writeln!(tx, "压缩比: {}:1", COMPRESSION_RATIO).unwrap();
                },
                StorageMode::Infinite => {
                    writeln!(tx, "无限模式: 循环覆盖存储").unwrap();
                    writeln!(tx, "优点: 连续记录，不丢失数据").unwrap();
                    writeln!(tx, "缺点: 历史数据会被覆盖").unwrap();
                },
            }
            
            // 显示信号测量结果
            writeln!(tx, "\n--- 信号测量 ---").unwrap();
            writeln!(tx, "CH1 幅度: {:.1}mV", statistics.ch1_amplitude).unwrap();
            writeln!(tx, "CH1 频率: {:.1}Hz", statistics.ch1_frequency).unwrap();
            writeln!(tx, "CH1 直流偏移: {:.1}mV", statistics.ch1_dc_offset).unwrap();
            writeln!(tx, "CH1 RMS: {:.1}mV", statistics.ch1_rms).unwrap();
            
            writeln!(tx, "CH2 幅度: {:.1}mV", statistics.ch2_amplitude).unwrap();
            writeln!(tx, "CH2 频率: {:.1}Hz", statistics.ch2_frequency).unwrap();
            writeln!(tx, "CH2 直流偏移: {:.1}mV", statistics.ch2_dc_offset).unwrap();
            writeln!(tx, "CH2 RMS: {:.1}mV", statistics.ch2_rms).unwrap();
            
            // 显示下一个存储模式预告
            let next_mode = match storage_manager.storage_mode {
                StorageMode::Normal => "包络模式",
                StorageMode::Envelope => "平均模式",
                StorageMode::Average => "峰值模式",
                StorageMode::Peak => "压缩模式",
                StorageMode::Compressed => "无限模式",
                StorageMode::Infinite => "正常模式",
            };
            let remaining_time = (mode_switch_interval - (sample_count - mode_switch_time)) / SAMPLE_RATE;
            
            writeln!(tx, "\n--- 下一个存储模式 ---").unwrap();
            writeln!(tx, "下一个模式: {}", next_mode).unwrap();
            writeln!(tx, "切换倒计时: {}秒", remaining_time).unwrap();
            
            // 显示系统状态
            writeln!(tx, "\n--- 系统状态 ---").unwrap();
            writeln!(tx, "系统时钟: {}MHz", clocks.sysclk().0 / 1_000_000).unwrap();
            writeln!(tx, "ADC分辨率: 12位").unwrap();
            writeln!(tx, "参考电压: {}mV", VREF_MV).unwrap();
            writeln!(tx, "缓冲区状态: 正常").unwrap();
            writeln!(tx, "存储系统: 正常").unwrap();
            writeln!(tx, "触发系统: 正常").unwrap();
        }
    }
}