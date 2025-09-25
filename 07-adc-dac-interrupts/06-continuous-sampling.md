# 第6章：连续采样技术

## 概述

连续采样是现代数据采集系统的核心技术，它允许系统以固定的时间间隔持续获取模拟信号的数字表示。本章将深入探讨STM32F4微控制器中连续采样的实现方法、优化策略和实际应用。

## 学习目标

通过本章学习，您将掌握：

1. **连续采样基础理论**：采样定理、奈奎斯特频率、混叠现象
2. **STM32F4连续采样配置**：定时器触发、DMA传输、循环缓冲区
3. **高速数据流处理**：实时处理、缓冲区管理、数据完整性
4. **性能优化技术**：中断优化、内存管理、CPU负载平衡
5. **实际应用案例**：音频采集、振动监测、实时控制系统

## 连续采样基础理论

### 采样定理与奈奎斯特频率

```rust
// 采样定理验证工具
pub struct SamplingTheoremValidator {
    sample_rate: f32,
    signal_frequency: f32,
    samples: Vec<f32>,
}

impl SamplingTheoremValidator {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            sample_rate,
            signal_frequency: 0.0,
            samples: Vec::new(),
        }
    }
    
    pub fn validate_nyquist_criterion(&self, signal_freq: f32) -> ValidationResult {
        let nyquist_freq = self.sample_rate / 2.0;
        
        ValidationResult {
            signal_frequency: signal_freq,
            sample_rate: self.sample_rate,
            nyquist_frequency: nyquist_freq,
            is_valid: signal_freq < nyquist_freq,
            aliasing_frequency: if signal_freq > nyquist_freq {
                Some(self.calculate_aliasing_frequency(signal_freq))
            } else {
                None
            },
            recommendation: self.get_recommendation(signal_freq, nyquist_freq),
        }
    }
    
    fn calculate_aliasing_frequency(&self, signal_freq: f32) -> f32 {
        let nyquist = self.sample_rate / 2.0;
        let k = (signal_freq / nyquist).floor() as i32;
        
        if k % 2 == 0 {
            signal_freq - k as f32 * nyquist
        } else {
            (k + 1) as f32 * nyquist - signal_freq
        }
    }
    
    fn get_recommendation(&self, signal_freq: f32, nyquist_freq: f32) -> String {
        if signal_freq < nyquist_freq * 0.4 {
            "采样率充足，信号可以完美重建".to_string()
        } else if signal_freq < nyquist_freq * 0.8 {
            "采样率适中，建议使用抗混叠滤波器".to_string()
        } else if signal_freq < nyquist_freq {
            "采样率接近极限，必须使用高质量抗混叠滤波器".to_string()
        } else {
            format!("采样率不足，建议提高到至少 {:.1} Hz", signal_freq * 2.5)
        }
    }
}

#[derive(Debug)]
pub struct ValidationResult {
    pub signal_frequency: f32,
    pub sample_rate: f32,
    pub nyquist_frequency: f32,
    pub is_valid: bool,
    pub aliasing_frequency: Option<f32>,
    pub recommendation: String,
}
```

### 抗混叠滤波器设计

```rust
// 抗混叠滤波器设计工具
pub struct AntiAliasingFilter {
    cutoff_frequency: f32,
    sample_rate: f32,
    filter_order: usize,
    coefficients: FilterCoefficients,
}

#[derive(Debug, Clone)]
pub struct FilterCoefficients {
    pub a: Vec<f32>, // 分母系数
    pub b: Vec<f32>, // 分子系数
}

impl AntiAliasingFilter {
    pub fn design_butterworth(sample_rate: f32, cutoff_freq: f32, order: usize) -> Self {
        let normalized_freq = cutoff_freq / (sample_rate / 2.0);
        let coefficients = Self::calculate_butterworth_coefficients(normalized_freq, order);
        
        Self {
            cutoff_frequency: cutoff_freq,
            sample_rate,
            filter_order: order,
            coefficients,
        }
    }
    
    fn calculate_butterworth_coefficients(wc: f32, order: usize) -> FilterCoefficients {
        // 简化的巴特沃斯滤波器系数计算
        // 实际应用中需要更精确的计算
        let mut a = vec![1.0];
        let mut b = vec![1.0];
        
        match order {
            1 => {
                let k = (std::f32::consts::PI * wc).tan();
                let norm = 1.0 + k;
                b = vec![k / norm, k / norm];
                a = vec![1.0, (k - 1.0) / norm];
            }
            2 => {
                let k = (std::f32::consts::PI * wc).tan();
                let norm = 1.0 + k / 0.7071 + k * k;
                b = vec![k * k / norm, 2.0 * k * k / norm, k * k / norm];
                a = vec![1.0, (2.0 * (k * k - 1.0)) / norm, (1.0 - k / 0.7071 + k * k) / norm];
            }
            _ => {
                // 高阶滤波器需要更复杂的计算
                b = vec![wc; order + 1];
                a = vec![1.0; order + 1];
            }
        }
        
        FilterCoefficients { a, b }
    }
    
    pub fn apply_filter(&self, input: &[f32]) -> Vec<f32> {
        let mut output = vec![0.0; input.len()];
        let mut x_history = vec![0.0; self.coefficients.b.len()];
        let mut y_history = vec![0.0; self.coefficients.a.len()];
        
        for (i, &sample) in input.iter().enumerate() {
            // 更新输入历史
            x_history.rotate_right(1);
            x_history[0] = sample;
            
            // 计算输出
            let mut y = 0.0;
            for (j, &b_coeff) in self.coefficients.b.iter().enumerate() {
                if j < x_history.len() {
                    y += b_coeff * x_history[j];
                }
            }
            
            for (j, &a_coeff) in self.coefficients.a.iter().skip(1).enumerate() {
                if j < y_history.len() {
                    y -= a_coeff * y_history[j];
                }
            }
            
            // 更新输出历史
            y_history.rotate_right(1);
            y_history[0] = y;
            
            output[i] = y;
        }
        
        output
    }
    
    pub fn get_frequency_response(&self, frequencies: &[f32]) -> Vec<FrequencyResponse> {
        frequencies.iter().map(|&freq| {
            let omega = 2.0 * std::f32::consts::PI * freq / self.sample_rate;
            let z = Complex::new(omega.cos(), omega.sin());
            
            let mut numerator = Complex::new(0.0, 0.0);
            let mut denominator = Complex::new(0.0, 0.0);
            
            for (i, &b_coeff) in self.coefficients.b.iter().enumerate() {
                numerator += b_coeff * z.powf(-(i as f32));
            }
            
            for (i, &a_coeff) in self.coefficients.a.iter().enumerate() {
                denominator += a_coeff * z.powf(-(i as f32));
            }
            
            let h = numerator / denominator;
            
            FrequencyResponse {
                frequency: freq,
                magnitude: h.norm(),
                magnitude_db: 20.0 * h.norm().log10(),
                phase: h.arg(),
                phase_degrees: h.arg() * 180.0 / std::f32::consts::PI,
            }
        }).collect()
    }
}

#[derive(Debug)]
pub struct FrequencyResponse {
    pub frequency: f32,
    pub magnitude: f32,
    pub magnitude_db: f32,
    pub phase: f32,
    pub phase_degrees: f32,
}

// 简化的复数结构
#[derive(Debug, Clone, Copy)]
struct Complex {
    real: f32,
    imag: f32,
}

impl Complex {
    fn new(real: f32, imag: f32) -> Self {
        Self { real, imag }
    }
    
    fn norm(&self) -> f32 {
        (self.real * self.real + self.imag * self.imag).sqrt()
    }
    
    fn arg(&self) -> f32 {
        self.imag.atan2(self.real)
    }
    
    fn powf(&self, exp: f32) -> Self {
        let r = self.norm();
        let theta = self.arg();
        let new_r = r.powf(exp);
        let new_theta = theta * exp;
        
        Self {
            real: new_r * new_theta.cos(),
            imag: new_r * new_theta.sin(),
        }
    }
}

impl std::ops::Add for Complex {
    type Output = Self;
    
    fn add(self, other: Self) -> Self {
        Self {
            real: self.real + other.real,
            imag: self.imag + other.imag,
        }
    }
}

impl std::ops::AddAssign for Complex {
    fn add_assign(&mut self, other: Self) {
        *self = *self + other;
    }
}

impl std::ops::Mul<f32> for Complex {
    type Output = Self;
    
    fn mul(self, scalar: f32) -> Self {
        Self {
            real: self.real * scalar,
            imag: self.imag * scalar,
        }
    }
}

impl std::ops::Div for Complex {
    type Output = Self;
    
    fn div(self, other: Self) -> Self {
        let denom = other.real * other.real + other.imag * other.imag;
        Self {
            real: (self.real * other.real + self.imag * other.imag) / denom,
            imag: (self.imag * other.real - self.real * other.imag) / denom,
        }
    }
}
```

## STM32F4连续采样实现

### 定时器触发的连续采样

```rust
use stm32f4xx_hal::{
    adc::{Adc, config::AdcConfig},
    gpio::{Analog, Pin},
    pac::{ADC1, TIM2},
    timer::{Timer, Event},
    dma::{Stream0, Channel0, StreamsTuple, Transfer, MemoryToPeripheral},
    prelude::*,
};

// 连续采样系统
pub struct ContinuousSamplingSystem {
    adc: Adc<ADC1>,
    timer: Timer<TIM2>,
    dma_transfer: Option<Transfer<Stream0<stm32f4xx_hal::pac::DMA2>, Channel0, Adc<ADC1>, MemoryToPeripheral, &'static mut [u16]>>,
    sample_buffer: &'static mut [u16],
    sample_rate: u32,
    channels: Vec<Pin<'A', 0, Analog>>,
    buffer_manager: CircularBufferManager,
    statistics: SamplingStatistics,
}

impl ContinuousSamplingSystem {
    pub fn new(
        adc: ADC1,
        timer: TIM2,
        dma_streams: StreamsTuple<stm32f4xx_hal::pac::DMA2>,
        sample_buffer: &'static mut [u16],
        sample_rate: u32,
    ) -> Self {
        // 配置ADC
        let adc_config = AdcConfig::default()
            .continuous(true)
            .dma(true)
            .scan(true);
        
        let mut adc = Adc::adc1(adc, true, adc_config);
        
        // 配置定时器
        let mut timer = Timer::new(timer, &clocks);
        timer.start(sample_rate.Hz());
        timer.listen(Event::Update);
        
        // 配置DMA
        let dma_stream = dma_streams.0;
        let dma_transfer = Transfer::init_memory_to_peripheral(
            dma_stream,
            adc,
            sample_buffer,
            None,
            dma::config::DmaConfig::default()
                .memory_increment(true)
                .circular_buffer(true)
                .transfer_complete_interrupt(true)
                .half_transfer_interrupt(true),
        );
        
        Self {
            adc,
            timer,
            dma_transfer: Some(dma_transfer),
            sample_buffer,
            sample_rate,
            channels: Vec::new(),
            buffer_manager: CircularBufferManager::new(sample_buffer.len()),
            statistics: SamplingStatistics::new(),
        }
    }
    
    pub fn add_channel(&mut self, channel: Pin<'A', 0, Analog>) -> Result<(), SamplingError> {
        if self.channels.len() >= 16 {
            return Err(SamplingError::TooManyChannels);
        }
        
        self.channels.push(channel);
        Ok(())
    }
    
    pub fn start_continuous_sampling(&mut self) -> Result<(), SamplingError> {
        // 启动DMA传输
        if let Some(transfer) = self.dma_transfer.take() {
            transfer.start(|_adc| {
                // ADC启动回调
            });
        }
        
        // 启动定时器
        self.timer.resume();
        
        self.statistics.start_time = get_timestamp_ms();
        self.statistics.is_running = true;
        
        Ok(())
    }
    
    pub fn stop_continuous_sampling(&mut self) -> Result<(), SamplingError> {
        // 停止定时器
        self.timer.pause();
        
        // 停止DMA传输
        if let Some(mut transfer) = self.dma_transfer.take() {
            let (stream, adc, buffer, _) = transfer.free();
            self.dma_transfer = Some(Transfer::init_memory_to_peripheral(
                stream, adc, buffer, None,
                dma::config::DmaConfig::default(),
            ));
        }
        
        self.statistics.is_running = false;
        self.statistics.total_runtime_ms = get_timestamp_ms() - self.statistics.start_time;
        
        Ok(())
    }
    
    pub fn handle_dma_interrupt(&mut self) -> Result<ProcessedData, SamplingError> {
        self.statistics.total_samples += self.channels.len() as u64;
        self.statistics.interrupt_count += 1;
        
        // 处理半传输完成中断
        if self.is_half_transfer_complete() {
            let half_size = self.sample_buffer.len() / 2;
            let data = &self.sample_buffer[0..half_size];
            return self.process_buffer_data(data);
        }
        
        // 处理传输完成中断
        if self.is_transfer_complete() {
            let half_size = self.sample_buffer.len() / 2;
            let data = &self.sample_buffer[half_size..];
            return self.process_buffer_data(data);
        }
        
        Err(SamplingError::NoDataAvailable)
    }
    
    fn process_buffer_data(&mut self, data: &[u16]) -> Result<ProcessedData, SamplingError> {
        let mut processed = ProcessedData::new(self.channels.len());
        
        // 按通道分离数据
        for (i, &sample) in data.iter().enumerate() {
            let channel_index = i % self.channels.len();
            processed.channels[channel_index].push(sample);
        }
        
        // 转换为电压值
        for channel_data in &mut processed.channels {
            for sample in channel_data {
                *sample = self.adc_to_voltage(*sample);
            }
        }
        
        // 更新统计信息
        self.update_statistics(&processed);
        
        Ok(processed)
    }
    
    fn adc_to_voltage(&self, adc_value: u16) -> u16 {
        // 简化的ADC到电压转换
        // 实际应用中需要考虑参考电压和校准
        ((adc_value as u32 * 3300) / 4095) as u16 // mV
    }
    
    fn update_statistics(&mut self, data: &ProcessedData) {
        for (i, channel_data) in data.channels.iter().enumerate() {
            if !channel_data.is_empty() {
                let sum: u32 = channel_data.iter().map(|&x| x as u32).sum();
                let avg = sum / channel_data.len() as u32;
                
                self.statistics.channel_averages[i] = avg as u16;
                
                // 计算标准差
                let variance: u32 = channel_data.iter()
                    .map(|&x| {
                        let diff = x as i32 - avg as i32;
                        (diff * diff) as u32
                    })
                    .sum::<u32>() / channel_data.len() as u32;
                
                self.statistics.channel_std_dev[i] = (variance as f32).sqrt() as u16;
            }
        }
    }
    
    fn is_half_transfer_complete(&self) -> bool {
        // 检查DMA半传输完成标志
        // 实际实现需要检查DMA寄存器
        true // 简化实现
    }
    
    fn is_transfer_complete(&self) -> bool {
        // 检查DMA传输完成标志
        // 实际实现需要检查DMA寄存器
        true // 简化实现
    }
    
    pub fn get_sampling_statistics(&self) -> &SamplingStatistics {
        &self.statistics
    }
    
    pub fn get_effective_sample_rate(&self) -> f32 {
        if self.statistics.total_runtime_ms > 0 {
            (self.statistics.total_samples as f32 * 1000.0) / self.statistics.total_runtime_ms as f32
        } else {
            0.0
        }
    }
}

#[derive(Debug)]
pub struct ProcessedData {
    pub channels: Vec<Vec<u16>>,
    pub timestamp_ms: u32,
    pub sample_count: usize,
}

impl ProcessedData {
    fn new(channel_count: usize) -> Self {
        Self {
            channels: vec![Vec::new(); channel_count],
            timestamp_ms: get_timestamp_ms(),
            sample_count: 0,
        }
    }
}

#[derive(Debug)]
pub struct SamplingStatistics {
    pub is_running: bool,
    pub start_time: u32,
    pub total_runtime_ms: u32,
    pub total_samples: u64,
    pub interrupt_count: u64,
    pub channel_averages: [u16; 16],
    pub channel_std_dev: [u16; 16],
    pub buffer_overruns: u32,
    pub missed_samples: u32,
}

impl SamplingStatistics {
    fn new() -> Self {
        Self {
            is_running: false,
            start_time: 0,
            total_runtime_ms: 0,
            total_samples: 0,
            interrupt_count: 0,
            channel_averages: [0; 16],
            channel_std_dev: [0; 16],
            buffer_overruns: 0,
            missed_samples: 0,
        }
    }
}

#[derive(Debug)]
pub enum SamplingError {
    TooManyChannels,
    InvalidSampleRate,
    DmaError,
    TimerError,
    BufferOverrun,
    NoDataAvailable,
}
```

### 循环缓冲区管理

```rust
// 高效的循环缓冲区管理器
pub struct CircularBufferManager {
    buffer_size: usize,
    write_index: usize,
    read_index: usize,
    data_count: usize,
    overrun_count: u32,
}

impl CircularBufferManager {
    pub fn new(buffer_size: usize) -> Self {
        Self {
            buffer_size,
            write_index: 0,
            read_index: 0,
            data_count: 0,
            overrun_count: 0,
        }
    }
    
    pub fn write_data(&mut self, buffer: &mut [u16], data: &[u16]) -> Result<usize, BufferError> {
        if data.len() > self.available_write_space() {
            self.overrun_count += 1;
            return Err(BufferError::BufferFull);
        }
        
        let mut written = 0;
        for &sample in data {
            buffer[self.write_index] = sample;
            self.write_index = (self.write_index + 1) % self.buffer_size;
            written += 1;
        }
        
        self.data_count += written;
        Ok(written)
    }
    
    pub fn read_data(&mut self, buffer: &[u16], output: &mut [u16]) -> Result<usize, BufferError> {
        let read_count = output.len().min(self.data_count);
        
        for i in 0..read_count {
            output[i] = buffer[self.read_index];
            self.read_index = (self.read_index + 1) % self.buffer_size;
        }
        
        self.data_count -= read_count;
        Ok(read_count)
    }
    
    pub fn available_write_space(&self) -> usize {
        self.buffer_size - self.data_count
    }
    
    pub fn available_read_data(&self) -> usize {
        self.data_count
    }
    
    pub fn is_full(&self) -> bool {
        self.data_count == self.buffer_size
    }
    
    pub fn is_empty(&self) -> bool {
        self.data_count == 0
    }
    
    pub fn get_overrun_count(&self) -> u32 {
        self.overrun_count
    }
    
    pub fn reset(&mut self) {
        self.write_index = 0;
        self.read_index = 0;
        self.data_count = 0;
        self.overrun_count = 0;
    }
    
    pub fn get_buffer_utilization(&self) -> f32 {
        (self.data_count as f32 / self.buffer_size as f32) * 100.0
    }
}

#[derive(Debug)]
pub enum BufferError {
    BufferFull,
    BufferEmpty,
    InvalidSize,
}
```

## 实时数据流处理

### 流式数据处理器

```rust
// 实时流式数据处理器
pub struct StreamProcessor {
    processors: Vec<Box<dyn DataProcessor>>,
    input_buffer: Vec<f32>,
    output_buffer: Vec<f32>,
    processing_stats: ProcessingStatistics,
}

pub trait DataProcessor {
    fn process(&mut self, input: &[f32], output: &mut [f32]) -> Result<usize, ProcessingError>;
    fn get_latency_samples(&self) -> usize;
    fn reset(&mut self);
}

impl StreamProcessor {
    pub fn new(buffer_size: usize) -> Self {
        Self {
            processors: Vec::new(),
            input_buffer: vec![0.0; buffer_size],
            output_buffer: vec![0.0; buffer_size],
            processing_stats: ProcessingStatistics::new(),
        }
    }
    
    pub fn add_processor(&mut self, processor: Box<dyn DataProcessor>) {
        self.processors.push(processor);
    }
    
    pub fn process_stream(&mut self, input: &[f32]) -> Result<Vec<f32>, ProcessingError> {
        let start_time = get_timestamp_us();
        
        // 复制输入数据到内部缓冲区
        if input.len() > self.input_buffer.len() {
            return Err(ProcessingError::BufferTooSmall);
        }
        
        self.input_buffer[..input.len()].copy_from_slice(input);
        let mut current_input = &self.input_buffer[..input.len()];
        let mut current_output = &mut self.output_buffer[..input.len()];
        
        // 依次应用所有处理器
        for processor in &mut self.processors {
            let processed_samples = processor.process(current_input, current_output)?;
            
            // 交换输入输出缓冲区
            std::mem::swap(&mut current_input, &mut current_output);
            
            self.processing_stats.total_samples_processed += processed_samples as u64;
        }
        
        let end_time = get_timestamp_us();
        self.processing_stats.total_processing_time_us += end_time - start_time;
        self.processing_stats.processing_cycles += 1;
        
        Ok(current_input[..input.len()].to_vec())
    }
    
    pub fn get_total_latency_samples(&self) -> usize {
        self.processors.iter().map(|p| p.get_latency_samples()).sum()
    }
    
    pub fn get_processing_stats(&self) -> &ProcessingStatistics {
        &self.processing_stats
    }
    
    pub fn reset_all_processors(&mut self) {
        for processor in &mut self.processors {
            processor.reset();
        }
        self.processing_stats.reset();
    }
}

#[derive(Debug)]
pub struct ProcessingStatistics {
    pub total_samples_processed: u64,
    pub total_processing_time_us: u64,
    pub processing_cycles: u64,
    pub average_processing_time_us: f32,
    pub max_processing_time_us: u64,
    pub cpu_utilization_percent: f32,
}

impl ProcessingStatistics {
    fn new() -> Self {
        Self {
            total_samples_processed: 0,
            total_processing_time_us: 0,
            processing_cycles: 0,
            average_processing_time_us: 0.0,
            max_processing_time_us: 0,
            cpu_utilization_percent: 0.0,
        }
    }
    
    fn reset(&mut self) {
        *self = Self::new();
    }
    
    pub fn update_stats(&mut self) {
        if self.processing_cycles > 0 {
            self.average_processing_time_us = 
                self.total_processing_time_us as f32 / self.processing_cycles as f32;
        }
    }
}

#[derive(Debug)]
pub enum ProcessingError {
    BufferTooSmall,
    ProcessorError,
    InvalidInput,
}
```

### 实时滤波器实现

```rust
// 实时FIR滤波器
pub struct RealtimeFirFilter {
    coefficients: Vec<f32>,
    delay_line: Vec<f32>,
    index: usize,
}

impl RealtimeFirFilter {
    pub fn new(coefficients: Vec<f32>) -> Self {
        let delay_line = vec![0.0; coefficients.len()];
        Self {
            coefficients,
            delay_line,
            index: 0,
        }
    }
    
    pub fn design_lowpass(sample_rate: f32, cutoff_freq: f32, num_taps: usize) -> Self {
        let mut coefficients = Vec::with_capacity(num_taps);
        let fc = cutoff_freq / sample_rate;
        let center = (num_taps - 1) as f32 / 2.0;
        
        for i in 0..num_taps {
            let n = i as f32 - center;
            let coeff = if n == 0.0 {
                2.0 * fc
            } else {
                (2.0 * std::f32::consts::PI * fc * n).sin() / (std::f32::consts::PI * n)
            };
            
            // 应用汉明窗
            let window = 0.54 - 0.46 * (2.0 * std::f32::consts::PI * i as f32 / (num_taps - 1) as f32).cos();
            coefficients.push(coeff * window);
        }
        
        // 归一化系数
        let sum: f32 = coefficients.iter().sum();
        for coeff in &mut coefficients {
            *coeff /= sum;
        }
        
        Self::new(coefficients)
    }
}

impl DataProcessor for RealtimeFirFilter {
    fn process(&mut self, input: &[f32], output: &mut [f32]) -> Result<usize, ProcessingError> {
        if output.len() < input.len() {
            return Err(ProcessingError::BufferTooSmall);
        }
        
        for (i, &sample) in input.iter().enumerate() {
            // 更新延迟线
            self.delay_line[self.index] = sample;
            self.index = (self.index + 1) % self.delay_line.len();
            
            // 计算滤波器输出
            let mut result = 0.0;
            for (j, &coeff) in self.coefficients.iter().enumerate() {
                let delay_index = (self.index + self.delay_line.len() - 1 - j) % self.delay_line.len();
                result += coeff * self.delay_line[delay_index];
            }
            
            output[i] = result;
        }
        
        Ok(input.len())
    }
    
    fn get_latency_samples(&self) -> usize {
        self.coefficients.len() / 2
    }
    
    fn reset(&mut self) {
        self.delay_line.fill(0.0);
        self.index = 0;
    }
}

// 实时IIR滤波器
pub struct RealtimeIirFilter {
    a_coeffs: Vec<f32>, // 分母系数
    b_coeffs: Vec<f32>, // 分子系数
    x_history: Vec<f32>, // 输入历史
    y_history: Vec<f32>, // 输出历史
}

impl RealtimeIirFilter {
    pub fn new(a_coeffs: Vec<f32>, b_coeffs: Vec<f32>) -> Self {
        let x_history = vec![0.0; b_coeffs.len()];
        let y_history = vec![0.0; a_coeffs.len()];
        
        Self {
            a_coeffs,
            b_coeffs,
            x_history,
            y_history,
        }
    }
    
    pub fn design_butterworth_lowpass(sample_rate: f32, cutoff_freq: f32, order: usize) -> Self {
        // 简化的巴特沃斯滤波器设计
        match order {
            1 => {
                let wc = 2.0 * std::f32::consts::PI * cutoff_freq / sample_rate;
                let k = wc.tan();
                let norm = 1.0 + k;
                
                let b_coeffs = vec![k / norm, k / norm];
                let a_coeffs = vec![1.0, (k - 1.0) / norm];
                
                Self::new(a_coeffs, b_coeffs)
            }
            2 => {
                let wc = 2.0 * std::f32::consts::PI * cutoff_freq / sample_rate;
                let k = wc.tan();
                let sqrt2 = 2.0_f32.sqrt();
                let norm = 1.0 + k / sqrt2 + k * k;
                
                let b_coeffs = vec![k * k / norm, 2.0 * k * k / norm, k * k / norm];
                let a_coeffs = vec![
                    1.0,
                    (2.0 * (k * k - 1.0)) / norm,
                    (1.0 - k / sqrt2 + k * k) / norm
                ];
                
                Self::new(a_coeffs, b_coeffs)
            }
            _ => {
                // 高阶滤波器需要更复杂的设计
                let b_coeffs = vec![1.0, 0.0];
                let a_coeffs = vec![1.0, 0.0];
                Self::new(a_coeffs, b_coeffs)
            }
        }
    }
}

impl DataProcessor for RealtimeIirFilter {
    fn process(&mut self, input: &[f32], output: &mut [f32]) -> Result<usize, ProcessingError> {
        if output.len() < input.len() {
            return Err(ProcessingError::BufferTooSmall);
        }
        
        for (i, &sample) in input.iter().enumerate() {
            // 更新输入历史
            self.x_history.rotate_right(1);
            self.x_history[0] = sample;
            
            // 计算输出
            let mut y = 0.0;
            
            // 分子部分 (前馈)
            for (j, &b_coeff) in self.b_coeffs.iter().enumerate() {
                if j < self.x_history.len() {
                    y += b_coeff * self.x_history[j];
                }
            }
            
            // 分母部分 (反馈)
            for (j, &a_coeff) in self.a_coeffs.iter().skip(1).enumerate() {
                if j < self.y_history.len() {
                    y -= a_coeff * self.y_history[j];
                }
            }
            
            // 更新输出历史
            self.y_history.rotate_right(1);
            self.y_history[0] = y;
            
            output[i] = y;
        }
        
        Ok(input.len())
    }
    
    fn get_latency_samples(&self) -> usize {
        1 // IIR滤波器通常只有1个样本的延迟
    }
    
    fn reset(&mut self) {
        self.x_history.fill(0.0);
        self.y_history.fill(0.0);
    }
}
```

## 性能优化技术

### 中断优化策略

```rust
// 中断优化管理器
pub struct InterruptOptimizer {
    interrupt_load: f32,
    max_interrupt_time_us: u32,
    interrupt_frequency: u32,
    optimization_strategies: Vec<OptimizationStrategy>,
}

#[derive(Debug, Clone)]
pub enum OptimizationStrategy {
    BatchProcessing { batch_size: usize },
    PriorityAdjustment { priority: u8 },
    BufferSizeOptimization { buffer_size: usize },
    ProcessingOffload { use_dma: bool },
}

impl InterruptOptimizer {
    pub fn new() -> Self {
        Self {
            interrupt_load: 0.0,
            max_interrupt_time_us: 100,
            interrupt_frequency: 1000,
            optimization_strategies: Vec::new(),
        }
    }
    
    pub fn analyze_interrupt_performance(&mut self, stats: &SamplingStatistics) -> OptimizationReport {
        // 计算中断负载
        let interrupt_period_us = 1_000_000 / self.interrupt_frequency;
        let avg_interrupt_time_us = if stats.interrupt_count > 0 {
            stats.total_runtime_ms * 1000 / stats.interrupt_count as u32
        } else {
            0
        };
        
        self.interrupt_load = (avg_interrupt_time_us as f32 / interrupt_period_us as f32) * 100.0;
        
        let mut recommendations = Vec::new();
        
        // 分析并生成优化建议
        if self.interrupt_load > 80.0 {
            recommendations.push(OptimizationStrategy::BatchProcessing { batch_size: 64 });
            recommendations.push(OptimizationStrategy::ProcessingOffload { use_dma: true });
        }
        
        if avg_interrupt_time_us > self.max_interrupt_time_us {
            recommendations.push(OptimizationStrategy::BufferSizeOptimization { buffer_size: 512 });
        }
        
        if stats.buffer_overruns > 0 {
            recommendations.push(OptimizationStrategy::PriorityAdjustment { priority: 1 });
        }
        
        OptimizationReport {
            interrupt_load_percent: self.interrupt_load,
            average_interrupt_time_us: avg_interrupt_time_us,
            max_interrupt_time_us: self.max_interrupt_time_us,
            buffer_overruns: stats.buffer_overruns,
            recommendations,
        }
    }
    
    pub fn apply_optimization(&mut self, strategy: OptimizationStrategy) -> Result<(), OptimizationError> {
        match strategy {
            OptimizationStrategy::BatchProcessing { batch_size } => {
                if batch_size > 1024 {
                    return Err(OptimizationError::InvalidBatchSize);
                }
                // 实施批处理优化
                println!("Applied batch processing with size: {}", batch_size);
            }
            OptimizationStrategy::PriorityAdjustment { priority } => {
                if priority > 15 {
                    return Err(OptimizationError::InvalidPriority);
                }
                // 调整中断优先级
                println!("Adjusted interrupt priority to: {}", priority);
            }
            OptimizationStrategy::BufferSizeOptimization { buffer_size } => {
                if buffer_size < 64 || buffer_size > 4096 {
                    return Err(OptimizationError::InvalidBufferSize);
                }
                // 优化缓冲区大小
                println!("Optimized buffer size to: {}", buffer_size);
            }
            OptimizationStrategy::ProcessingOffload { use_dma } => {
                if use_dma {
                    // 启用DMA处理
                    println!("Enabled DMA processing offload");
                }
            }
        }
        
        self.optimization_strategies.push(strategy);
        Ok(())
    }
}

#[derive(Debug)]
pub struct OptimizationReport {
    pub interrupt_load_percent: f32,
    pub average_interrupt_time_us: u32,
    pub max_interrupt_time_us: u32,
    pub buffer_overruns: u32,
    pub recommendations: Vec<OptimizationStrategy>,
}

#[derive(Debug)]
pub enum OptimizationError {
    InvalidBatchSize,
    InvalidPriority,
    InvalidBufferSize,
    OptimizationFailed,
}
```

### 内存管理优化

```rust
// 内存池管理器
pub struct MemoryPoolManager {
    pools: Vec<MemoryPool>,
    allocation_stats: AllocationStatistics,
}

struct MemoryPool {
    block_size: usize,
    block_count: usize,
    free_blocks: Vec<*mut u8>,
    memory: Vec<u8>,
}

impl MemoryPool {
    fn new(block_size: usize, block_count: usize) -> Self {
        let total_size = block_size * block_count;
        let mut memory = vec![0u8; total_size];
        let mut free_blocks = Vec::with_capacity(block_count);
        
        // 初始化空闲块列表
        for i in 0..block_count {
            let block_ptr = unsafe { memory.as_mut_ptr().add(i * block_size) };
            free_blocks.push(block_ptr);
        }
        
        Self {
            block_size,
            block_count,
            free_blocks,
            memory,
        }
    }
    
    fn allocate(&mut self) -> Option<*mut u8> {
        self.free_blocks.pop()
    }
    
    fn deallocate(&mut self, ptr: *mut u8) -> Result<(), MemoryError> {
        // 验证指针是否属于这个池
        let pool_start = self.memory.as_ptr() as usize;
        let pool_end = pool_start + self.memory.len();
        let ptr_addr = ptr as usize;
        
        if ptr_addr < pool_start || ptr_addr >= pool_end {
            return Err(MemoryError::InvalidPointer);
        }
        
        // 检查对齐
        if (ptr_addr - pool_start) % self.block_size != 0 {
            return Err(MemoryError::InvalidAlignment);
        }
        
        self.free_blocks.push(ptr);
        Ok(())
    }
    
    fn available_blocks(&self) -> usize {
        self.free_blocks.len()
    }
    
    fn utilization(&self) -> f32 {
        let used_blocks = self.block_count - self.free_blocks.len();
        (used_blocks as f32 / self.block_count as f32) * 100.0
    }
}

impl MemoryPoolManager {
    pub fn new() -> Self {
        let mut pools = Vec::new();
        
        // 创建不同大小的内存池
        pools.push(MemoryPool::new(64, 100));    // 小块
        pools.push(MemoryPool::new(256, 50));    // 中块
        pools.push(MemoryPool::new(1024, 20));   // 大块
        pools.push(MemoryPool::new(4096, 5));    // 超大块
        
        Self {
            pools,
            allocation_stats: AllocationStatistics::new(),
        }
    }
    
    pub fn allocate(&mut self, size: usize) -> Option<*mut u8> {
        // 找到合适的内存池
        for pool in &mut self.pools {
            if size <= pool.block_size {
                if let Some(ptr) = pool.allocate() {
                    self.allocation_stats.total_allocations += 1;
                    self.allocation_stats.bytes_allocated += pool.block_size;
                    return Some(ptr);
                }
            }
        }
        
        self.allocation_stats.failed_allocations += 1;
        None
    }
    
    pub fn deallocate(&mut self, ptr: *mut u8, size: usize) -> Result<(), MemoryError> {
        // 找到对应的内存池
        for pool in &mut self.pools {
            if size <= pool.block_size {
                if pool.deallocate(ptr).is_ok() {
                    self.allocation_stats.total_deallocations += 1;
                    self.allocation_stats.bytes_deallocated += pool.block_size;
                    return Ok(());
                }
            }
        }
        
        Err(MemoryError::PoolNotFound)
    }
    
    pub fn get_memory_report(&self) -> MemoryReport {
        let mut pool_reports = Vec::new();
        
        for (i, pool) in self.pools.iter().enumerate() {
            pool_reports.push(PoolReport {
                pool_id: i,
                block_size: pool.block_size,
                total_blocks: pool.block_count,
                available_blocks: pool.available_blocks(),
                utilization_percent: pool.utilization(),
            });
        }
        
        MemoryReport {
            pool_reports,
            allocation_stats: self.allocation_stats.clone(),
        }
    }
    
    pub fn defragment(&mut self) -> DefragmentationResult {
        // 内存池通常不需要碎片整理，但可以重新组织空闲列表
        let mut total_freed = 0;
        
        for pool in &mut self.pools {
            // 重新排序空闲块列表以提高局部性
            pool.free_blocks.sort_by_key(|&ptr| ptr as usize);
            total_freed += pool.available_blocks();
        }
        
        DefragmentationResult {
            blocks_reorganized: total_freed,
            memory_freed_bytes: 0, // 内存池不释放内存
            fragmentation_reduced_percent: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct AllocationStatistics {
    pub total_allocations: u64,
    pub total_deallocations: u64,
    pub failed_allocations: u64,
    pub bytes_allocated: u64,
    pub bytes_deallocated: u64,
}

impl AllocationStatistics {
    fn new() -> Self {
        Self {
            total_allocations: 0,
            total_deallocations: 0,
            failed_allocations: 0,
            bytes_allocated: 0,
            bytes_deallocated: 0,
        }
    }
}

#[derive(Debug)]
pub struct MemoryReport {
    pub pool_reports: Vec<PoolReport>,
    pub allocation_stats: AllocationStatistics,
}

#[derive(Debug)]
pub struct PoolReport {
    pub pool_id: usize,
    pub block_size: usize,
    pub total_blocks: usize,
    pub available_blocks: usize,
    pub utilization_percent: f32,
}

#[derive(Debug)]
pub struct DefragmentationResult {
    pub blocks_reorganized: usize,
    pub memory_freed_bytes: usize,
    pub fragmentation_reduced_percent: f32,
}

#[derive(Debug)]
pub enum MemoryError {
    InvalidPointer,
    InvalidAlignment,
    PoolNotFound,
    OutOfMemory,
}
```

## 实际应用案例

### 音频采集系统

```rust
// 高质量音频采集系统
pub struct AudioAcquisitionSystem {
    sampling_system: ContinuousSamplingSystem,
    audio_processor: AudioProcessor,
    format_converter: AudioFormatConverter,
    quality_analyzer: AudioQualityAnalyzer,
}

impl AudioAcquisitionSystem {
    pub fn new(sample_rate: u32, bit_depth: u8, channels: u8) -> Self {
        let sampling_system = ContinuousSamplingSystem::new(
            /* ADC, Timer, DMA parameters */
        );
        
        Self {
            sampling_system,
            audio_processor: AudioProcessor::new(sample_rate, bit_depth, channels),
            format_converter: AudioFormatConverter::new(),
            quality_analyzer: AudioQualityAnalyzer::new(),
        }
    }
    
    pub fn start_recording(&mut self) -> Result<(), AudioError> {
        // 配置音频参数
        self.audio_processor.configure_for_recording()?;
        
        // 启动连续采样
        self.sampling_system.start_continuous_sampling()
            .map_err(|_| AudioError::SamplingStartFailed)?;
        
        Ok(())
    }
    
    pub fn process_audio_data(&mut self, raw_data: &[u16]) -> Result<AudioData, AudioError> {
        // 转换为浮点音频数据
        let audio_samples = self.format_converter.convert_to_float(raw_data)?;
        
        // 应用音频处理
        let processed_samples = self.audio_processor.process(&audio_samples)?;
        
        // 分析音频质量
        let quality_metrics = self.quality_analyzer.analyze(&processed_samples)?;
        
        Ok(AudioData {
            samples: processed_samples,
            sample_rate: self.audio_processor.sample_rate,
            channels: self.audio_processor.channels,
            quality_metrics,
            timestamp_ms: get_timestamp_ms(),
        })
    }
}

pub struct AudioProcessor {
    sample_rate: u32,
    bit_depth: u8,
    channels: u8,
    dc_blocker: DcBlockingFilter,
    noise_gate: NoiseGate,
    compressor: AudioCompressor,
}

impl AudioProcessor {
    pub fn new(sample_rate: u32, bit_depth: u8, channels: u8) -> Self {
        Self {
            sample_rate,
            bit_depth,
            channels,
            dc_blocker: DcBlockingFilter::new(sample_rate as f32),
            noise_gate: NoiseGate::new(-60.0), // -60dB threshold
            compressor: AudioCompressor::new(4.0, 10.0), // 4:1 ratio, 10ms attack
        }
    }
    
    pub fn configure_for_recording(&mut self) -> Result<(), AudioError> {
        // 配置DC阻断滤波器
        self.dc_blocker.set_cutoff_frequency(20.0); // 20Hz高通
        
        // 配置噪声门
        self.noise_gate.set_threshold(-50.0);
        self.noise_gate.set_ratio(10.0);
        
        // 配置压缩器
        self.compressor.set_threshold(-20.0);
        self.compressor.set_attack_time(5.0);
        self.compressor.set_release_time(100.0);
        
        Ok(())
    }
    
    pub fn process(&mut self, input: &[f32]) -> Result<Vec<f32>, AudioError> {
        let mut output = input.to_vec();
        
        // 应用DC阻断
        self.dc_blocker.process(&mut output)?;
        
        // 应用噪声门
        self.noise_gate.process(&mut output)?;
        
        // 应用压缩
        self.compressor.process(&mut output)?;
        
        Ok(output)
    }
}

// DC阻断滤波器
pub struct DcBlockingFilter {
    sample_rate: f32,
    cutoff_freq: f32,
    prev_input: f32,
    prev_output: f32,
    alpha: f32,
}

impl DcBlockingFilter {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            sample_rate,
            cutoff_freq: 20.0,
            prev_input: 0.0,
            prev_output: 0.0,
            alpha: 0.0,
        }
    }
    
    pub fn set_cutoff_frequency(&mut self, freq: f32) {
        self.cutoff_freq = freq;
        let rc = 1.0 / (2.0 * std::f32::consts::PI * freq);
        let dt = 1.0 / self.sample_rate;
        self.alpha = rc / (rc + dt);
    }
    
    pub fn process(&mut self, samples: &mut [f32]) -> Result<(), AudioError> {
        for sample in samples {
            let output = self.alpha * (self.prev_output + *sample - self.prev_input);
            self.prev_input = *sample;
            self.prev_output = output;
            *sample = output;
        }
        Ok(())
    }
}

// 噪声门
pub struct NoiseGate {
    threshold_db: f32,
    ratio: f32,
    attack_samples: usize,
    release_samples: usize,
    envelope: f32,
    gate_state: f32,
}

impl NoiseGate {
    pub fn new(threshold_db: f32) -> Self {
        Self {
            threshold_db,
            ratio: 10.0,
            attack_samples: 220, // ~5ms at 44.1kHz
            release_samples: 4410, // ~100ms at 44.1kHz
            envelope: 0.0,
            gate_state: 0.0,
        }
    }
    
    pub fn set_threshold(&mut self, threshold_db: f32) {
        self.threshold_db = threshold_db;
    }
    
    pub fn set_ratio(&mut self, ratio: f32) {
        self.ratio = ratio;
    }
    
    pub fn process(&mut self, samples: &mut [f32]) -> Result<(), AudioError> {
        let threshold_linear = db_to_linear(self.threshold_db);
        
        for sample in samples {
            let input_level = sample.abs();
            
            // 更新包络
            if input_level > self.envelope {
                self.envelope = input_level;
            } else {
                self.envelope *= 0.999; // 慢速衰减
            }
            
            // 计算门控状态
            let target_gate = if self.envelope > threshold_linear {
                1.0
            } else {
                1.0 / self.ratio
            };
            
            // 平滑门控状态变化
            if target_gate > self.gate_state {
                self.gate_state += (target_gate - self.gate_state) / self.attack_samples as f32;
            } else {
                self.gate_state += (target_gate - self.gate_state) / self.release_samples as f32;
            }
            
            *sample *= self.gate_state;
        }
        
        Ok(())
    }
}

// 音频压缩器
pub struct AudioCompressor {
    threshold_db: f32,
    ratio: f32,
    attack_time_ms: f32,
    release_time_ms: f32,
    envelope: f32,
    gain_reduction: f32,
}

impl AudioCompressor {
    pub fn new(ratio: f32, attack_time_ms: f32) -> Self {
        Self {
            threshold_db: -20.0,
            ratio,
            attack_time_ms,
            release_time_ms: 100.0,
            envelope: 0.0,
            gain_reduction: 1.0,
        }
    }
    
    pub fn set_threshold(&mut self, threshold_db: f32) {
        self.threshold_db = threshold_db;
    }
    
    pub fn set_attack_time(&mut self, attack_ms: f32) {
        self.attack_time_ms = attack_ms;
    }
    
    pub fn set_release_time(&mut self, release_ms: f32) {
        self.release_time_ms = release_ms;
    }
    
    pub fn process(&mut self, samples: &mut [f32]) -> Result<(), AudioError> {
        let threshold_linear = db_to_linear(self.threshold_db);
        
        for sample in samples {
            let input_level = sample.abs();
            
            // 更新包络跟踪
            if input_level > self.envelope {
                self.envelope = input_level;
            } else {
                self.envelope *= 0.9999; // 慢速释放
            }
            
            // 计算增益衰减
            if self.envelope > threshold_linear {
                let over_threshold_db = linear_to_db(self.envelope) - self.threshold_db;
                let compressed_db = over_threshold_db / self.ratio;
                let target_gain = db_to_linear(compressed_db - over_threshold_db);
                
                // 平滑增益变化
                if target_gain < self.gain_reduction {
                    self.gain_reduction = target_gain; // 快速攻击
                } else {
                    self.gain_reduction += (target_gain - self.gain_reduction) * 0.001; // 慢速释放
                }
            } else {
                self.gain_reduction += (1.0 - self.gain_reduction) * 0.001; // 释放到1.0
            }
            
            *sample *= self.gain_reduction;
        }
        
        Ok(())
    }
}

// 辅助函数
fn db_to_linear(db: f32) -> f32 {
    10.0_f32.powf(db / 20.0)
}

fn linear_to_db(linear: f32) -> f32 {
    20.0 * linear.log10()
}

#[derive(Debug)]
pub struct AudioData {
    pub samples: Vec<f32>,
    pub sample_rate: u32,
    pub channels: u8,
    pub quality_metrics: AudioQualityMetrics,
    pub timestamp_ms: u32,
}

#[derive(Debug)]
pub struct AudioQualityMetrics {
    pub snr_db: f32,
    pub thd_percent: f32,
    pub dynamic_range_db: f32,
    pub peak_level_db: f32,
    pub rms_level_db: f32,
}

#[derive(Debug)]
pub enum AudioError {
    SamplingStartFailed,
    ProcessingFailed,
    FormatConversionFailed,
    QualityAnalysisFailed,
}

// 辅助函数
fn get_timestamp_ms() -> u32 {
    // 返回当前时间戳（毫秒）
    0 // 简化实现
}

fn get_timestamp_us() -> u64 {
    // 返回当前时间戳（微秒）
    0 // 简化实现
}
```

## 最佳实践

### 设计原则

1. **实时性保证**：
   - 确定性的处理时间
   - 优先级合理分配
   - 避免阻塞操作

2. **数据完整性**：
   - 双缓冲机制
   - 错误检测和恢复
   - 数据校验

3. **资源优化**：
   - 内存使用最小化
   - CPU负载平衡
   - 功耗管理

### 性能优化建议

1. **硬件优化**：
   - 选择合适的采样率
   - 优化时钟配置
   - 使用硬件加速

2. **软件优化**：
   - 算法优化
   - 数据结构优化
   - 编译器优化

3. **系统优化**：
   - 任务调度优化
   - 中断管理
   - 内存管理

## 总结

连续采样技术是现代嵌入式系统中数据采集的核心技术。通过本章的学习，您应该掌握：

1. 连续采样的理论基础和实现方法
2. STM32F4平台的高效采样配置
3. 实时数据流处理技术
4. 性能优化和资源管理策略
5. 实际应用系统的设计和实现

关键要点：
- 遵循采样定理，避免混叠现象
- 使用DMA和中断优化数据传输
- 实施有效的缓冲区管理
- 进行全面的性能分析和优化
- 确保系统的实时性和可靠性

## 参考资料

1. **技术文档**：
   - STM32F4 Reference Manual
   - ADC Application Notes
   - DMA Programming Guide

2. **学术资源**：
   - Digital Signal Processing Theory
   - Real-time Systems Design
   - Embedded Systems Programming

3. **开源项目**：
   - STM32 HAL Examples
   - Real-time Audio Processing
   - Industrial Data Acquisition Systems