# 第7章：DAC波形生成技术

## 概述

数字模拟转换器(DAC)波形生成是现代嵌入式系统中信号生成和控制的核心技术。本章将深入探讨STM32F4微控制器中DAC的高级应用，包括任意波形生成、多通道同步输出、频率调制和相位控制等技术。

## 学习目标

通过本章学习，您将掌握：

1. **DAC波形生成原理**：波形数学模型、采样重建理论、频谱分析
2. **STM32F4 DAC高级配置**：双通道同步、触发模式、DMA波形输出
3. **任意波形生成**：查表法、实时计算、波形合成技术
4. **频率和相位控制**：数字频率合成、相位累加器、调制技术
5. **实际应用案例**：函数发生器、音频合成器、控制信号生成

## DAC波形生成基础理论

### 数字波形合成原理

```rust
use core::f32::consts::PI;

// 波形生成器基础结构
pub struct WaveformGenerator {
    sample_rate: f32,
    phase_accumulator: f32,
    frequency: f32,
    amplitude: f32,
    offset: f32,
    waveform_type: WaveformType,
    lookup_table: Option<Vec<f32>>,
    phase_increment: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum WaveformType {
    Sine,
    Cosine,
    Triangle,
    Sawtooth,
    Square,
    Noise,
    Custom,
}

impl WaveformGenerator {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            sample_rate,
            phase_accumulator: 0.0,
            frequency: 1000.0,
            amplitude: 1.0,
            offset: 0.0,
            waveform_type: WaveformType::Sine,
            lookup_table: None,
            phase_increment: 0.0,
        }
    }
    
    pub fn set_frequency(&mut self, frequency: f32) -> Result<(), WaveformError> {
        if frequency <= 0.0 || frequency > self.sample_rate / 2.0 {
            return Err(WaveformError::InvalidFrequency);
        }
        
        self.frequency = frequency;
        self.phase_increment = 2.0 * PI * frequency / self.sample_rate;
        Ok(())
    }
    
    pub fn set_amplitude(&mut self, amplitude: f32) -> Result<(), WaveformError> {
        if amplitude < 0.0 || amplitude > 1.0 {
            return Err(WaveformError::InvalidAmplitude);
        }
        
        self.amplitude = amplitude;
        Ok(())
    }
    
    pub fn set_offset(&mut self, offset: f32) -> Result<(), WaveformError> {
        if offset.abs() > 1.0 {
            return Err(WaveformError::InvalidOffset);
        }
        
        self.offset = offset;
        Ok(())
    }
    
    pub fn set_waveform_type(&mut self, waveform_type: WaveformType) {
        self.waveform_type = waveform_type;
        
        // 为某些波形类型预生成查找表
        match waveform_type {
            WaveformType::Sine | WaveformType::Cosine => {
                self.generate_sinusoidal_table();
            }
            WaveformType::Custom => {
                // 用户需要提供自定义查找表
            }
            _ => {
                self.lookup_table = None;
            }
        }
    }
    
    fn generate_sinusoidal_table(&mut self) {
        const TABLE_SIZE: usize = 1024;
        let mut table = Vec::with_capacity(TABLE_SIZE);
        
        for i in 0..TABLE_SIZE {
            let phase = 2.0 * PI * i as f32 / TABLE_SIZE as f32;
            let value = match self.waveform_type {
                WaveformType::Sine => phase.sin(),
                WaveformType::Cosine => phase.cos(),
                _ => 0.0,
            };
            table.push(value);
        }
        
        self.lookup_table = Some(table);
    }
    
    pub fn set_custom_waveform(&mut self, samples: Vec<f32>) -> Result<(), WaveformError> {
        if samples.is_empty() || samples.len() > 4096 {
            return Err(WaveformError::InvalidTableSize);
        }
        
        // 验证样本值范围
        for &sample in &samples {
            if sample.abs() > 1.0 {
                return Err(WaveformError::InvalidSampleValue);
            }
        }
        
        self.waveform_type = WaveformType::Custom;
        self.lookup_table = Some(samples);
        Ok(())
    }
    
    pub fn generate_sample(&mut self) -> f32 {
        let sample = match self.waveform_type {
            WaveformType::Sine => self.generate_sine(),
            WaveformType::Cosine => self.generate_cosine(),
            WaveformType::Triangle => self.generate_triangle(),
            WaveformType::Sawtooth => self.generate_sawtooth(),
            WaveformType::Square => self.generate_square(),
            WaveformType::Noise => self.generate_noise(),
            WaveformType::Custom => self.generate_custom(),
        };
        
        // 更新相位累加器
        self.phase_accumulator += self.phase_increment;
        if self.phase_accumulator >= 2.0 * PI {
            self.phase_accumulator -= 2.0 * PI;
        }
        
        // 应用幅度和偏移
        self.amplitude * sample + self.offset
    }
    
    fn generate_sine(&self) -> f32 {
        if let Some(ref table) = self.lookup_table {
            let index = (self.phase_accumulator / (2.0 * PI) * table.len() as f32) as usize % table.len();
            table[index]
        } else {
            self.phase_accumulator.sin()
        }
    }
    
    fn generate_cosine(&self) -> f32 {
        if let Some(ref table) = self.lookup_table {
            let index = (self.phase_accumulator / (2.0 * PI) * table.len() as f32) as usize % table.len();
            table[index]
        } else {
            self.phase_accumulator.cos()
        }
    }
    
    fn generate_triangle(&self) -> f32 {
        let normalized_phase = self.phase_accumulator / (2.0 * PI);
        if normalized_phase < 0.5 {
            4.0 * normalized_phase - 1.0
        } else {
            3.0 - 4.0 * normalized_phase
        }
    }
    
    fn generate_sawtooth(&self) -> f32 {
        let normalized_phase = self.phase_accumulator / (2.0 * PI);
        2.0 * normalized_phase - 1.0
    }
    
    fn generate_square(&self) -> f32 {
        if self.phase_accumulator < PI {
            1.0
        } else {
            -1.0
        }
    }
    
    fn generate_noise(&self) -> f32 {
        // 简单的线性同余随机数生成器
        static mut SEED: u32 = 1;
        unsafe {
            SEED = SEED.wrapping_mul(1103515245).wrapping_add(12345);
            ((SEED >> 16) as f32 / 32768.0) - 1.0
        }
    }
    
    fn generate_custom(&self) -> f32 {
        if let Some(ref table) = self.lookup_table {
            let index = (self.phase_accumulator / (2.0 * PI) * table.len() as f32) as usize % table.len();
            table[index]
        } else {
            0.0
        }
    }
    
    pub fn generate_buffer(&mut self, buffer: &mut [f32]) {
        for sample in buffer {
            *sample = self.generate_sample();
        }
    }
    
    pub fn reset_phase(&mut self) {
        self.phase_accumulator = 0.0;
    }
    
    pub fn get_phase(&self) -> f32 {
        self.phase_accumulator
    }
    
    pub fn set_phase(&mut self, phase: f32) {
        self.phase_accumulator = phase % (2.0 * PI);
    }
}

#[derive(Debug)]
pub enum WaveformError {
    InvalidFrequency,
    InvalidAmplitude,
    InvalidOffset,
    InvalidTableSize,
    InvalidSampleValue,
}
```

### 多通道波形合成器

```rust
// 多通道波形合成器
pub struct MultiChannelWaveformSynthesizer {
    channels: Vec<WaveformGenerator>,
    sample_rate: f32,
    sync_mode: SyncMode,
    master_phase: f32,
    output_mixer: OutputMixer,
}

#[derive(Debug, Clone, Copy)]
pub enum SyncMode {
    Independent,    // 独立运行
    PhaseLocked,   // 相位锁定
    FrequencyRatio, // 频率比例
}

impl MultiChannelWaveformSynthesizer {
    pub fn new(channel_count: usize, sample_rate: f32) -> Self {
        let mut channels = Vec::with_capacity(channel_count);
        for _ in 0..channel_count {
            channels.push(WaveformGenerator::new(sample_rate));
        }
        
        Self {
            channels,
            sample_rate,
            sync_mode: SyncMode::Independent,
            master_phase: 0.0,
            output_mixer: OutputMixer::new(channel_count),
        }
    }
    
    pub fn set_sync_mode(&mut self, sync_mode: SyncMode) {
        self.sync_mode = sync_mode;
    }
    
    pub fn configure_channel(&mut self, channel: usize, config: ChannelConfig) -> Result<(), SynthesizerError> {
        if channel >= self.channels.len() {
            return Err(SynthesizerError::InvalidChannel);
        }
        
        let generator = &mut self.channels[channel];
        generator.set_frequency(config.frequency)?;
        generator.set_amplitude(config.amplitude)?;
        generator.set_offset(config.offset)?;
        generator.set_waveform_type(config.waveform_type);
        
        if let Some(phase_offset) = config.phase_offset {
            generator.set_phase(phase_offset);
        }
        
        Ok(())
    }
    
    pub fn generate_samples(&mut self, output: &mut [Vec<f32>]) -> Result<(), SynthesizerError> {
        if output.len() != self.channels.len() {
            return Err(SynthesizerError::BufferSizeMismatch);
        }
        
        let buffer_size = output[0].len();
        
        match self.sync_mode {
            SyncMode::Independent => {
                for (i, channel) in self.channels.iter_mut().enumerate() {
                    channel.generate_buffer(&mut output[i]);
                }
            }
            SyncMode::PhaseLocked => {
                self.generate_phase_locked_samples(output, buffer_size)?;
            }
            SyncMode::FrequencyRatio => {
                self.generate_frequency_ratio_samples(output, buffer_size)?;
            }
        }
        
        Ok(())
    }
    
    fn generate_phase_locked_samples(&mut self, output: &mut [Vec<f32>], buffer_size: usize) -> Result<(), SynthesizerError> {
        // 所有通道使用相同的主相位，但可以有相位偏移
        for sample_idx in 0..buffer_size {
            for (channel_idx, channel) in self.channels.iter_mut().enumerate() {
                // 同步相位到主相位
                let phase_offset = channel.get_phase() - self.master_phase;
                channel.set_phase(self.master_phase + phase_offset);
                
                output[channel_idx][sample_idx] = channel.generate_sample();
            }
            
            // 更新主相位
            let master_frequency = self.channels[0].frequency;
            let phase_increment = 2.0 * PI * master_frequency / self.sample_rate;
            self.master_phase += phase_increment;
            if self.master_phase >= 2.0 * PI {
                self.master_phase -= 2.0 * PI;
            }
        }
        
        Ok(())
    }
    
    fn generate_frequency_ratio_samples(&mut self, output: &mut [Vec<f32>], buffer_size: usize) -> Result<(), SynthesizerError> {
        // 基于主频率的整数倍关系
        let master_frequency = self.channels[0].frequency;
        
        for sample_idx in 0..buffer_size {
            for (channel_idx, channel) in self.channels.iter_mut().enumerate() {
                // 计算频率比例
                let frequency_ratio = channel.frequency / master_frequency;
                let synced_phase = self.master_phase * frequency_ratio;
                channel.set_phase(synced_phase);
                
                output[channel_idx][sample_idx] = channel.generate_sample();
            }
            
            // 更新主相位
            let phase_increment = 2.0 * PI * master_frequency / self.sample_rate;
            self.master_phase += phase_increment;
            if self.master_phase >= 2.0 * PI {
                self.master_phase -= 2.0 * PI;
            }
        }
        
        Ok(())
    }
    
    pub fn generate_mixed_output(&mut self, buffer_size: usize) -> Result<Vec<f32>, SynthesizerError> {
        let mut channel_outputs = vec![vec![0.0; buffer_size]; self.channels.len()];
        self.generate_samples(&mut channel_outputs)?;
        
        self.output_mixer.mix_channels(&channel_outputs)
    }
    
    pub fn set_channel_mix_level(&mut self, channel: usize, level: f32) -> Result<(), SynthesizerError> {
        self.output_mixer.set_channel_level(channel, level)
    }
    
    pub fn reset_all_phases(&mut self) {
        self.master_phase = 0.0;
        for channel in &mut self.channels {
            channel.reset_phase();
        }
    }
}

#[derive(Debug, Clone)]
pub struct ChannelConfig {
    pub frequency: f32,
    pub amplitude: f32,
    pub offset: f32,
    pub waveform_type: WaveformType,
    pub phase_offset: Option<f32>,
}

// 输出混合器
pub struct OutputMixer {
    channel_levels: Vec<f32>,
    master_level: f32,
    pan_positions: Vec<f32>, // -1.0 (左) 到 1.0 (右)
}

impl OutputMixer {
    pub fn new(channel_count: usize) -> Self {
        Self {
            channel_levels: vec![1.0; channel_count],
            master_level: 1.0,
            pan_positions: vec![0.0; channel_count], // 居中
        }
    }
    
    pub fn set_channel_level(&mut self, channel: usize, level: f32) -> Result<(), SynthesizerError> {
        if channel >= self.channel_levels.len() {
            return Err(SynthesizerError::InvalidChannel);
        }
        
        if level < 0.0 || level > 1.0 {
            return Err(SynthesizerError::InvalidLevel);
        }
        
        self.channel_levels[channel] = level;
        Ok(())
    }
    
    pub fn set_master_level(&mut self, level: f32) -> Result<(), SynthesizerError> {
        if level < 0.0 || level > 1.0 {
            return Err(SynthesizerError::InvalidLevel);
        }
        
        self.master_level = level;
        Ok(())
    }
    
    pub fn set_channel_pan(&mut self, channel: usize, pan: f32) -> Result<(), SynthesizerError> {
        if channel >= self.pan_positions.len() {
            return Err(SynthesizerError::InvalidChannel);
        }
        
        if pan < -1.0 || pan > 1.0 {
            return Err(SynthesizerError::InvalidPan);
        }
        
        self.pan_positions[channel] = pan;
        Ok(())
    }
    
    pub fn mix_channels(&self, channel_outputs: &[Vec<f32>]) -> Result<Vec<f32>, SynthesizerError> {
        if channel_outputs.is_empty() {
            return Err(SynthesizerError::NoChannels);
        }
        
        let buffer_size = channel_outputs[0].len();
        let mut mixed_output = vec![0.0; buffer_size];
        
        for sample_idx in 0..buffer_size {
            let mut sum = 0.0;
            
            for (channel_idx, channel_output) in channel_outputs.iter().enumerate() {
                if channel_idx < self.channel_levels.len() {
                    sum += channel_output[sample_idx] * self.channel_levels[channel_idx];
                }
            }
            
            mixed_output[sample_idx] = sum * self.master_level;
            
            // 应用软限幅防止溢出
            if mixed_output[sample_idx] > 1.0 {
                mixed_output[sample_idx] = 1.0;
            } else if mixed_output[sample_idx] < -1.0 {
                mixed_output[sample_idx] = -1.0;
            }
        }
        
        Ok(mixed_output)
    }
    
    pub fn mix_channels_stereo(&self, channel_outputs: &[Vec<f32>]) -> Result<(Vec<f32>, Vec<f32>), SynthesizerError> {
        if channel_outputs.is_empty() {
            return Err(SynthesizerError::NoChannels);
        }
        
        let buffer_size = channel_outputs[0].len();
        let mut left_output = vec![0.0; buffer_size];
        let mut right_output = vec![0.0; buffer_size];
        
        for sample_idx in 0..buffer_size {
            let mut left_sum = 0.0;
            let mut right_sum = 0.0;
            
            for (channel_idx, channel_output) in channel_outputs.iter().enumerate() {
                if channel_idx < self.channel_levels.len() && channel_idx < self.pan_positions.len() {
                    let sample = channel_output[sample_idx] * self.channel_levels[channel_idx];
                    let pan = self.pan_positions[channel_idx];
                    
                    // 计算左右声道增益
                    let left_gain = (1.0 - pan) / 2.0;
                    let right_gain = (1.0 + pan) / 2.0;
                    
                    left_sum += sample * left_gain;
                    right_sum += sample * right_gain;
                }
            }
            
            left_output[sample_idx] = (left_sum * self.master_level).clamp(-1.0, 1.0);
            right_output[sample_idx] = (right_sum * self.master_level).clamp(-1.0, 1.0);
        }
        
        Ok((left_output, right_output))
    }
}

#[derive(Debug)]
pub enum SynthesizerError {
    InvalidChannel,
    InvalidLevel,
    InvalidPan,
    BufferSizeMismatch,
    NoChannels,
}
```

## STM32F4 DAC高级配置

### 双通道同步DAC控制

```rust
use stm32f4xx_hal::{
    pac::{DAC, TIM6, DMA1},
    dac::{Dac, DacPin, C1, C2},
    timer::{Timer, Event},
    dma::{Stream5, Stream6, Channel7, Transfer, MemoryToPeripheral},
    gpio::{Pin, Analog},
    prelude::*,
};

// STM32F4双通道DAC控制器
pub struct DualChannelDacController {
    dac: Dac<DAC>,
    timer: Timer<TIM6>,
    dma_ch1: Option<Transfer<Stream5<DMA1>, Channel7, Dac<DAC, C1>, MemoryToPeripheral, &'static mut [u16]>>,
    dma_ch2: Option<Transfer<Stream6<DMA1>, Channel7, Dac<DAC, C2>, MemoryToPeripheral, &'static mut [u16]>>,
    waveform_buffer_ch1: &'static mut [u16],
    waveform_buffer_ch2: &'static mut [u16],
    sample_rate: u32,
    sync_mode: DacSyncMode,
}

#[derive(Debug, Clone, Copy)]
pub enum DacSyncMode {
    Independent,
    Synchronized,
    PhaseShifted(f32), // 相位偏移（弧度）
}

impl DualChannelDacController {
    pub fn new(
        dac: DAC,
        timer: TIM6,
        dma1_streams: (Stream5<DMA1>, Stream6<DMA1>),
        buffer_ch1: &'static mut [u16],
        buffer_ch2: &'static mut [u16],
        sample_rate: u32,
        clocks: &Clocks,
    ) -> Self {
        // 初始化DAC
        let dac = Dac::dac(dac, (Pin::new(), Pin::new()));
        
        // 配置定时器
        let mut timer = Timer::new(timer, clocks);
        timer.start(sample_rate.Hz());
        timer.listen(Event::Update);
        
        // 配置DMA传输
        let dma_config = dma::config::DmaConfig::default()
            .memory_increment(true)
            .circular_buffer(true)
            .transfer_complete_interrupt(true);
        
        let dma_ch1 = Transfer::init_memory_to_peripheral(
            dma1_streams.0,
            dac.ch1,
            buffer_ch1,
            None,
            dma_config,
        );
        
        let dma_ch2 = Transfer::init_memory_to_peripheral(
            dma1_streams.1,
            dac.ch2,
            buffer_ch2,
            None,
            dma_config,
        );
        
        Self {
            dac,
            timer,
            dma_ch1: Some(dma_ch1),
            dma_ch2: Some(dma_ch2),
            waveform_buffer_ch1: buffer_ch1,
            waveform_buffer_ch2: buffer_ch2,
            sample_rate,
            sync_mode: DacSyncMode::Independent,
        }
    }
    
    pub fn set_sync_mode(&mut self, sync_mode: DacSyncMode) {
        self.sync_mode = sync_mode;
    }
    
    pub fn load_waveform_ch1(&mut self, waveform: &WaveformData) -> Result<(), DacError> {
        if waveform.samples.len() > self.waveform_buffer_ch1.len() {
            return Err(DacError::BufferTooSmall);
        }
        
        // 转换浮点样本到DAC值
        for (i, &sample) in waveform.samples.iter().enumerate() {
            self.waveform_buffer_ch1[i] = self.float_to_dac_value(sample);
        }
        
        // 如果波形较短，重复填充缓冲区
        let waveform_len = waveform.samples.len();
        for i in waveform_len..self.waveform_buffer_ch1.len() {
            self.waveform_buffer_ch1[i] = self.waveform_buffer_ch1[i % waveform_len];
        }
        
        Ok(())
    }
    
    pub fn load_waveform_ch2(&mut self, waveform: &WaveformData) -> Result<(), DacError> {
        if waveform.samples.len() > self.waveform_buffer_ch2.len() {
            return Err(DacError::BufferTooSmall);
        }
        
        // 应用同步模式
        let mut samples = waveform.samples.clone();
        match self.sync_mode {
            DacSyncMode::Independent => {
                // 不做修改
            }
            DacSyncMode::Synchronized => {
                // 确保两个通道完全同步
                samples = self.waveform_buffer_ch1.iter()
                    .map(|&val| self.dac_value_to_float(val))
                    .collect();
            }
            DacSyncMode::PhaseShifted(phase_shift) => {
                // 应用相位偏移
                samples = self.apply_phase_shift(&samples, phase_shift);
            }
        }
        
        // 转换并加载到缓冲区
        for (i, &sample) in samples.iter().enumerate() {
            self.waveform_buffer_ch2[i] = self.float_to_dac_value(sample);
        }
        
        let waveform_len = samples.len();
        for i in waveform_len..self.waveform_buffer_ch2.len() {
            self.waveform_buffer_ch2[i] = self.waveform_buffer_ch2[i % waveform_len];
        }
        
        Ok(())
    }
    
    fn apply_phase_shift(&self, samples: &[f32], phase_shift: f32) -> Vec<f32> {
        let len = samples.len();
        let shift_samples = ((phase_shift / (2.0 * PI)) * len as f32) as usize;
        
        let mut shifted = vec![0.0; len];
        for i in 0..len {
            let src_index = (i + shift_samples) % len;
            shifted[i] = samples[src_index];
        }
        
        shifted
    }
    
    fn float_to_dac_value(&self, sample: f32) -> u16 {
        // 将-1.0到1.0的浮点值转换为0到4095的DAC值
        let normalized = (sample + 1.0) / 2.0; // 转换到0.0-1.0范围
        let clamped = normalized.clamp(0.0, 1.0);
        (clamped * 4095.0) as u16
    }
    
    fn dac_value_to_float(&self, dac_value: u16) -> f32 {
        // 将0到4095的DAC值转换为-1.0到1.0的浮点值
        let normalized = dac_value as f32 / 4095.0;
        normalized * 2.0 - 1.0
    }
    
    pub fn start_output(&mut self) -> Result<(), DacError> {
        // 启动DMA传输
        if let Some(transfer_ch1) = self.dma_ch1.take() {
            transfer_ch1.start(|_dac| {
                // DAC通道1启动回调
            });
        }
        
        if let Some(transfer_ch2) = self.dma_ch2.take() {
            transfer_ch2.start(|_dac| {
                // DAC通道2启动回调
            });
        }
        
        // 启动定时器
        self.timer.resume();
        
        Ok(())
    }
    
    pub fn stop_output(&mut self) -> Result<(), DacError> {
        // 停止定时器
        self.timer.pause();
        
        // 停止DMA传输
        if let Some(mut transfer_ch1) = self.dma_ch1.take() {
            let (stream, dac_ch1, buffer, _) = transfer_ch1.free();
            self.dma_ch1 = Some(Transfer::init_memory_to_peripheral(
                stream, dac_ch1, buffer, None,
                dma::config::DmaConfig::default(),
            ));
        }
        
        if let Some(mut transfer_ch2) = self.dma_ch2.take() {
            let (stream, dac_ch2, buffer, _) = transfer_ch2.free();
            self.dma_ch2 = Some(Transfer::init_memory_to_peripheral(
                stream, dac_ch2, buffer, None,
                dma::config::DmaConfig::default(),
            ));
        }
        
        Ok(())
    }
    
    pub fn set_sample_rate(&mut self, sample_rate: u32) -> Result<(), DacError> {
        if sample_rate < 1 || sample_rate > 1_000_000 {
            return Err(DacError::InvalidSampleRate);
        }
        
        self.sample_rate = sample_rate;
        
        // 重新配置定时器
        self.timer.pause();
        self.timer.start(sample_rate.Hz());
        
        Ok(())
    }
    
    pub fn get_output_frequency(&self, waveform_length: usize) -> f32 {
        self.sample_rate as f32 / waveform_length as f32
    }
    
    pub fn set_dc_output(&mut self, ch1_voltage: f32, ch2_voltage: f32) -> Result<(), DacError> {
        // 设置直流输出
        let dac_value_ch1 = self.voltage_to_dac_value(ch1_voltage)?;
        let dac_value_ch2 = self.voltage_to_dac_value(ch2_voltage)?;
        
        // 填充缓冲区为常数值
        self.waveform_buffer_ch1.fill(dac_value_ch1);
        self.waveform_buffer_ch2.fill(dac_value_ch2);
        
        Ok(())
    }
    
    fn voltage_to_dac_value(&self, voltage: f32) -> Result<u16, DacError> {
        const VREF: f32 = 3.3; // 参考电压
        
        if voltage < 0.0 || voltage > VREF {
            return Err(DacError::VoltageOutOfRange);
        }
        
        let dac_value = (voltage / VREF * 4095.0) as u16;
        Ok(dac_value)
    }
}

#[derive(Debug, Clone)]
pub struct WaveformData {
    pub samples: Vec<f32>,
    pub sample_rate: f32,
    pub frequency: f32,
}

#[derive(Debug)]
pub enum DacError {
    BufferTooSmall,
    InvalidSampleRate,
    VoltageOutOfRange,
    DmaError,
    TimerError,
}
```

### 高精度波形生成

```rust
// 高精度数字频率合成器
pub struct HighPrecisionDDS {
    phase_accumulator: u64,
    frequency_word: u64,
    phase_resolution: u8, // 相位分辨率位数
    amplitude_word: u16,
    lookup_table: Vec<u16>,
    sample_rate: f32,
    output_filter: AntiAliasingFilter,
}

impl HighPrecisionDDS {
    pub fn new(phase_resolution: u8, sample_rate: f32) -> Self {
        let table_size = 1 << (phase_resolution.min(16)); // 最大64K点
        let lookup_table = Self::generate_sine_table(table_size);
        
        Self {
            phase_accumulator: 0,
            frequency_word: 0,
            phase_resolution,
            amplitude_word: 0xFFFF, // 最大幅度
            lookup_table,
            sample_rate,
            output_filter: AntiAliasingFilter::new(sample_rate),
        }
    }
    
    fn generate_sine_table(size: usize) -> Vec<u16> {
        let mut table = Vec::with_capacity(size);
        
        for i in 0..size {
            let phase = 2.0 * PI * i as f32 / size as f32;
            let sine_value = phase.sin();
            // 转换为无符号16位值 (0-65535)
            let dac_value = ((sine_value + 1.0) / 2.0 * 65535.0) as u16;
            table.push(dac_value);
        }
        
        table
    }
    
    pub fn set_frequency(&mut self, frequency: f32) -> Result<(), DDSError> {
        if frequency < 0.0 || frequency > self.sample_rate / 2.0 {
            return Err(DDSError::FrequencyOutOfRange);
        }
        
        // 计算频率控制字
        let max_phase = 1u64 << (self.phase_resolution as u64);
        self.frequency_word = ((frequency as f64 / self.sample_rate as f64) * max_phase as f64) as u64;
        
        Ok(())
    }
    
    pub fn set_amplitude(&mut self, amplitude: f32) -> Result<(), DDSError> {
        if amplitude < 0.0 || amplitude > 1.0 {
            return Err(DDSError::AmplitudeOutOfRange);
        }
        
        self.amplitude_word = (amplitude * 65535.0) as u16;
        Ok(())
    }
    
    pub fn set_phase_offset(&mut self, phase_radians: f32) {
        let max_phase = 1u64 << (self.phase_resolution as u64);
        let phase_offset = ((phase_radians / (2.0 * PI)) * max_phase as f32) as u64;
        self.phase_accumulator = phase_offset;
    }
    
    pub fn generate_sample(&mut self) -> u16 {
        // 获取查找表索引
        let table_size = self.lookup_table.len();
        let phase_to_index_shift = self.phase_resolution - (table_size.trailing_zeros() as u8);
        let table_index = (self.phase_accumulator >> phase_to_index_shift) as usize % table_size;
        
        // 获取基础波形值
        let base_value = self.lookup_table[table_index];
        
        // 应用幅度控制
        let amplitude_scaled = ((base_value as u32 * self.amplitude_word as u32) >> 16) as u16;
        
        // 更新相位累加器
        self.phase_accumulator = self.phase_accumulator.wrapping_add(self.frequency_word);
        
        amplitude_scaled
    }
    
    pub fn generate_buffer(&mut self, buffer: &mut [u16]) {
        for sample in buffer {
            *sample = self.generate_sample();
        }
    }
    
    pub fn generate_filtered_buffer(&mut self, buffer: &mut [u16]) -> Result<(), DDSError> {
        // 生成原始样本
        let mut raw_samples = vec![0.0f32; buffer.len()];
        for (i, sample) in raw_samples.iter_mut().enumerate() {
            let dac_value = self.generate_sample();
            *sample = (dac_value as f32 / 65535.0) * 2.0 - 1.0; // 转换为-1到1范围
        }
        
        // 应用抗混叠滤波
        let filtered_samples = self.output_filter.apply_filter(&raw_samples);
        
        // 转换回DAC值
        for (i, &filtered_sample) in filtered_samples.iter().enumerate() {
            let normalized = (filtered_sample + 1.0) / 2.0;
            buffer[i] = (normalized.clamp(0.0, 1.0) * 65535.0) as u16;
        }
        
        Ok(())
    }
    
    pub fn reset(&mut self) {
        self.phase_accumulator = 0;
    }
    
    pub fn get_frequency_resolution(&self) -> f32 {
        self.sample_rate / (1u64 << self.phase_resolution) as f32
    }
    
    pub fn get_spurious_free_dynamic_range(&self) -> f32 {
        // 估算无杂散动态范围 (SFDR)
        // 基于相位分辨率和查找表大小
        let phase_noise_db = -6.02 * self.phase_resolution as f32;
        let quantization_noise_db = -6.02 * 16.0; // 16位DAC
        
        phase_noise_db.min(quantization_noise_db)
    }
}

// 抗混叠滤波器（简化版本）
pub struct AntiAliasingFilter {
    sample_rate: f32,
    cutoff_frequency: f32,
    filter_coefficients: Vec<f32>,
    delay_line: Vec<f32>,
}

impl AntiAliasingFilter {
    pub fn new(sample_rate: f32) -> Self {
        let cutoff_frequency = sample_rate * 0.4; // 设置为采样率的40%
        let filter_coefficients = Self::design_lowpass_filter(sample_rate, cutoff_frequency, 64);
        let delay_line = vec![0.0; filter_coefficients.len()];
        
        Self {
            sample_rate,
            cutoff_frequency,
            filter_coefficients,
            delay_line,
        }
    }
    
    fn design_lowpass_filter(sample_rate: f32, cutoff_freq: f32, num_taps: usize) -> Vec<f32> {
        let mut coefficients = Vec::with_capacity(num_taps);
        let fc = cutoff_freq / sample_rate;
        let center = (num_taps - 1) as f32 / 2.0;
        
        for i in 0..num_taps {
            let n = i as f32 - center;
            let coeff = if n == 0.0 {
                2.0 * fc
            } else {
                (2.0 * PI * fc * n).sin() / (PI * n)
            };
            
            // 应用汉明窗
            let window = 0.54 - 0.46 * (2.0 * PI * i as f32 / (num_taps - 1) as f32).cos();
            coefficients.push(coeff * window);
        }
        
        // 归一化
        let sum: f32 = coefficients.iter().sum();
        for coeff in &mut coefficients {
            *coeff /= sum;
        }
        
        coefficients
    }
    
    pub fn apply_filter(&mut self, input: &[f32]) -> Vec<f32> {
        let mut output = Vec::with_capacity(input.len());
        
        for &sample in input {
            // 更新延迟线
            self.delay_line.rotate_right(1);
            self.delay_line[0] = sample;
            
            // 计算滤波器输出
            let mut result = 0.0;
            for (i, &coeff) in self.filter_coefficients.iter().enumerate() {
                if i < self.delay_line.len() {
                    result += coeff * self.delay_line[i];
                }
            }
            
            output.push(result);
        }
        
        output
    }
}

#[derive(Debug)]
pub enum DDSError {
    FrequencyOutOfRange,
    AmplitudeOutOfRange,
    PhaseOutOfRange,
    FilterError,
}
```

## 频率调制和相位控制

### 频率调制器

```rust
// 频率调制器
pub struct FrequencyModulator {
    carrier_generator: WaveformGenerator,
    modulation_generator: WaveformGenerator,
    modulation_depth: f32,
    modulation_type: ModulationType,
    fm_sensitivity: f32, // Hz/V
}

#[derive(Debug, Clone, Copy)]
pub enum ModulationType {
    FrequencyModulation,  // FM
    PhaseModulation,      // PM
    AmplitudeModulation,  // AM
}

impl FrequencyModulator {
    pub fn new(sample_rate: f32, carrier_freq: f32, modulation_freq: f32) -> Self {
        let mut carrier_generator = WaveformGenerator::new(sample_rate);
        carrier_generator.set_frequency(carrier_freq).unwrap();
        carrier_generator.set_waveform_type(WaveformType::Sine);
        
        let mut modulation_generator = WaveformGenerator::new(sample_rate);
        modulation_generator.set_frequency(modulation_freq).unwrap();
        modulation_generator.set_waveform_type(WaveformType::Sine);
        
        Self {
            carrier_generator,
            modulation_generator,
            modulation_depth: 0.1,
            modulation_type: ModulationType::FrequencyModulation,
            fm_sensitivity: 1000.0, // 1kHz/V
        }
    }
    
    pub fn set_carrier_frequency(&mut self, frequency: f32) -> Result<(), WaveformError> {
        self.carrier_generator.set_frequency(frequency)
    }
    
    pub fn set_modulation_frequency(&mut self, frequency: f32) -> Result<(), WaveformError> {
        self.modulation_generator.set_frequency(frequency)
    }
    
    pub fn set_modulation_depth(&mut self, depth: f32) -> Result<(), ModulationError> {
        if depth < 0.0 || depth > 1.0 {
            return Err(ModulationError::InvalidDepth);
        }
        self.modulation_depth = depth;
        Ok(())
    }
    
    pub fn set_modulation_type(&mut self, mod_type: ModulationType) {
        self.modulation_type = mod_type;
    }
    
    pub fn set_fm_sensitivity(&mut self, sensitivity: f32) {
        self.fm_sensitivity = sensitivity;
    }
    
    pub fn generate_modulated_sample(&mut self) -> f32 {
        let modulation_signal = self.modulation_generator.generate_sample();
        
        match self.modulation_type {
            ModulationType::FrequencyModulation => {
                self.generate_fm_sample(modulation_signal)
            }
            ModulationType::PhaseModulation => {
                self.generate_pm_sample(modulation_signal)
            }
            ModulationType::AmplitudeModulation => {
                self.generate_am_sample(modulation_signal)
            }
        }
    }
    
    fn generate_fm_sample(&mut self, modulation_signal: f32) -> f32 {
        // 频率调制：瞬时频率 = 载波频率 + 调制深度 * 调制信号 * FM灵敏度
        let frequency_deviation = self.modulation_depth * modulation_signal * self.fm_sensitivity;
        let instantaneous_frequency = self.carrier_generator.frequency + frequency_deviation;
        
        // 临时修改载波频率
        let original_frequency = self.carrier_generator.frequency;
        self.carrier_generator.set_frequency(instantaneous_frequency).unwrap_or(());
        let sample = self.carrier_generator.generate_sample();
        self.carrier_generator.set_frequency(original_frequency).unwrap_or(());
        
        sample
    }
    
    fn generate_pm_sample(&mut self, modulation_signal: f32) -> f32 {
        // 相位调制：瞬时相位 = 载波相位 + 调制深度 * 调制信号
        let phase_deviation = self.modulation_depth * modulation_signal * PI;
        let original_phase = self.carrier_generator.get_phase();
        
        self.carrier_generator.set_phase(original_phase + phase_deviation);
        let sample = self.carrier_generator.generate_sample();
        
        sample
    }
    
    fn generate_am_sample(&mut self, modulation_signal: f32) -> f32 {
        // 幅度调制：瞬时幅度 = 载波幅度 * (1 + 调制深度 * 调制信号)
        let carrier_sample = self.carrier_generator.generate_sample();
        let modulated_amplitude = 1.0 + self.modulation_depth * modulation_signal;
        
        carrier_sample * modulated_amplitude
    }
    
    pub fn generate_modulated_buffer(&mut self, buffer: &mut [f32]) {
        for sample in buffer {
            *sample = self.generate_modulated_sample();
        }
    }
    
    pub fn get_modulation_index(&self) -> f32 {
        // 调制指数 = 频率偏移 / 调制频率
        let max_frequency_deviation = self.modulation_depth * self.fm_sensitivity;
        max_frequency_deviation / self.modulation_generator.frequency
    }
    
    pub fn get_bandwidth(&self) -> f32 {
        // 卡森定律：带宽 ≈ 2 * (频率偏移 + 调制频率)
        let max_frequency_deviation = self.modulation_depth * self.fm_sensitivity;
        2.0 * (max_frequency_deviation + self.modulation_generator.frequency)
    }
}

#[derive(Debug)]
pub enum ModulationError {
    InvalidDepth,
    InvalidFrequency,
    InvalidSensitivity,
}
```

### 相位锁定环(PLL)

```rust
// 数字相位锁定环
pub struct DigitalPLL {
    reference_frequency: f32,
    vco_frequency: f32,
    phase_detector: PhaseDetector,
    loop_filter: LoopFilter,
    vco: VoltageControlledOscillator,
    frequency_divider: FrequencyDivider,
    lock_status: bool,
    lock_threshold: f32,
}

impl DigitalPLL {
    pub fn new(reference_freq: f32, target_freq: f32, sample_rate: f32) -> Self {
        let division_ratio = (target_freq / reference_freq).round() as u32;
        
        Self {
            reference_frequency: reference_freq,
            vco_frequency: target_freq,
            phase_detector: PhaseDetector::new(),
            loop_filter: LoopFilter::new(0.1, 0.01), // Kp=0.1, Ki=0.01
            vco: VoltageControlledOscillator::new(sample_rate, target_freq),
            frequency_divider: FrequencyDivider::new(division_ratio),
            lock_status: false,
            lock_threshold: 0.1, // 10% 频率误差阈值
        }
    }
    
    pub fn process_sample(&mut self, reference_signal: f32) -> f32 {
        // VCO输出
        let vco_output = self.vco.generate_sample();
        
        // 分频器输出
        let divided_signal = self.frequency_divider.process(vco_output);
        
        // 相位检测
        let phase_error = self.phase_detector.detect_phase_error(reference_signal, divided_signal);
        
        // 环路滤波
        let control_voltage = self.loop_filter.filter(phase_error);
        
        // 控制VCO频率
        self.vco.set_control_voltage(control_voltage);
        
        // 更新锁定状态
        self.update_lock_status(phase_error);
        
        vco_output
    }
    
    fn update_lock_status(&mut self, phase_error: f32) {
        let frequency_error = phase_error.abs() / (2.0 * PI) * self.reference_frequency;
        let relative_error = frequency_error / self.vco_frequency;
        
        self.lock_status = relative_error < self.lock_threshold;
    }
    
    pub fn is_locked(&self) -> bool {
        self.lock_status
    }
    
    pub fn get_vco_frequency(&self) -> f32 {
        self.vco.get_frequency()
    }
    
    pub fn set_loop_bandwidth(&mut self, bandwidth: f32) {
        self.loop_filter.set_bandwidth(bandwidth);
    }
}

// 相位检测器
pub struct PhaseDetector {
    prev_ref_phase: f32,
    prev_fb_phase: f32,
}

impl PhaseDetector {
    pub fn new() -> Self {
        Self {
            prev_ref_phase: 0.0,
            prev_fb_phase: 0.0,
        }
    }
    
    pub fn detect_phase_error(&mut self, reference: f32, feedback: f32) -> f32 {
        // 简单的相位检测器（乘法器型）
        let phase_error = reference * feedback;
        
        // 更新相位历史
        self.prev_ref_phase = reference.atan2(1.0);
        self.prev_fb_phase = feedback.atan2(1.0);
        
        phase_error
    }
}

// 环路滤波器
pub struct LoopFilter {
    kp: f32, // 比例增益
    ki: f32, // 积分增益
    integrator: f32,
    prev_error: f32,
}

impl LoopFilter {
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            kp,
            ki,
            integrator: 0.0,
            prev_error: 0.0,
        }
    }
    
    pub fn filter(&mut self, error: f32) -> f32 {
        // PI控制器
        self.integrator += error * self.ki;
        
        // 防止积分饱和
        self.integrator = self.integrator.clamp(-1.0, 1.0);
        
        let output = self.kp * error + self.integrator;
        self.prev_error = error;
        
        output.clamp(-1.0, 1.0)
    }
    
    pub fn set_bandwidth(&mut self, bandwidth: f32) {
        // 根据带宽调整增益
        self.kp = bandwidth * 2.0;
        self.ki = bandwidth * bandwidth;
    }
    
    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.prev_error = 0.0;
    }
}

// 压控振荡器
pub struct VoltageControlledOscillator {
    generator: WaveformGenerator,
    center_frequency: f32,
    sensitivity: f32, // Hz/V
    control_voltage: f32,
}

impl VoltageControlledOscillator {
    pub fn new(sample_rate: f32, center_frequency: f32) -> Self {
        let mut generator = WaveformGenerator::new(sample_rate);
        generator.set_frequency(center_frequency).unwrap();
        generator.set_waveform_type(WaveformType::Sine);
        
        Self {
            generator,
            center_frequency,
            sensitivity: center_frequency * 0.1, // 10% 调谐范围
            control_voltage: 0.0,
        }
    }
    
    pub fn set_control_voltage(&mut self, voltage: f32) {
        self.control_voltage = voltage.clamp(-1.0, 1.0);
        
        let frequency = self.center_frequency + self.sensitivity * self.control_voltage;
        self.generator.set_frequency(frequency.max(1.0)).unwrap_or(());
    }
    
    pub fn generate_sample(&mut self) -> f32 {
        self.generator.generate_sample()
    }
    
    pub fn get_frequency(&self) -> f32 {
        self.center_frequency + self.sensitivity * self.control_voltage
    }
    
    pub fn set_sensitivity(&mut self, sensitivity: f32) {
        self.sensitivity = sensitivity;
    }
}

// 分频器
pub struct FrequencyDivider {
    division_ratio: u32,
    counter: u32,
    output_state: bool,
}

impl FrequencyDivider {
    pub fn new(division_ratio: u32) -> Self {
        Self {
            division_ratio,
            counter: 0,
            output_state: false,
        }
    }
    
    pub fn process(&mut self, input: f32) -> f32 {
        // 检测输入信号的上升沿
        if input > 0.0 && !self.output_state {
            self.counter += 1;
            
            if self.counter >= self.division_ratio {
                self.counter = 0;
                self.output_state = !self.output_state;
            }
        }
        
        if self.output_state { 1.0 } else { -1.0 }
    }
    
    pub fn set_division_ratio(&mut self, ratio: u32) {
        self.division_ratio = ratio;
        self.counter = 0;
    }
    
    pub fn reset(&mut self) {
        self.counter = 0;
        self.output_state = false;
    }
}
```

## 实际应用案例

### 函数发生器实现

```rust
// 多功能函数发生器
pub struct FunctionGenerator {
    dac_controller: DualChannelDacController,
    waveform_synthesizer: MultiChannelWaveformSynthesizer,
    modulator: FrequencyModulator,
    sweep_generator: SweepGenerator,
    burst_generator: BurstGenerator,
    current_mode: GeneratorMode,
    output_settings: OutputSettings,
}

#[derive(Debug, Clone, Copy)]
pub enum GeneratorMode {
    ContinuousWave,
    Modulated,
    Sweep,
    Burst,
    Arbitrary,
}

#[derive(Debug, Clone)]
pub struct OutputSettings {
    pub frequency: f32,
    pub amplitude: f32,
    pub offset: f32,
    pub phase: f32,
    pub waveform_type: WaveformType,
    pub channel_enabled: [bool; 2],
}

impl FunctionGenerator {
    pub fn new(
        dac_controller: DualChannelDacController,
        sample_rate: f32,
    ) -> Self {
        let waveform_synthesizer = MultiChannelWaveformSynthesizer::new(2, sample_rate);
        let modulator = FrequencyModulator::new(sample_rate, 1000.0, 100.0);
        let sweep_generator = SweepGenerator::new(sample_rate);
        let burst_generator = BurstGenerator::new(sample_rate);
        
        let output_settings = OutputSettings {
            frequency: 1000.0,
            amplitude: 1.0,
            offset: 0.0,
            phase: 0.0,
            waveform_type: WaveformType::Sine,
            channel_enabled: [true, true],
        };
        
        Self {
            dac_controller,
            waveform_synthesizer,
            modulator,
            sweep_generator,
            burst_generator,
            current_mode: GeneratorMode::ContinuousWave,
            output_settings,
        }
    }
    
    pub fn set_mode(&mut self, mode: GeneratorMode) -> Result<(), GeneratorError> {
        self.current_mode = mode;
        
        match mode {
            GeneratorMode::ContinuousWave => {
                self.configure_continuous_wave()?;
            }
            GeneratorMode::Modulated => {
                self.configure_modulated_output()?;
            }
            GeneratorMode::Sweep => {
                self.configure_sweep_output()?;
            }
            GeneratorMode::Burst => {
                self.configure_burst_output()?;
            }
            GeneratorMode::Arbitrary => {
                self.configure_arbitrary_waveform()?;
            }
        }
        
        Ok(())
    }
    
    fn configure_continuous_wave(&mut self) -> Result<(), GeneratorError> {
        // 配置连续波输出
        for channel in 0..2 {
            if self.output_settings.channel_enabled[channel] {
                let config = ChannelConfig {
                    frequency: self.output_settings.frequency,
                    amplitude: self.output_settings.amplitude,
                    offset: self.output_settings.offset,
                    waveform_type: self.output_settings.waveform_type,
                    phase_offset: Some(self.output_settings.phase + channel as f32 * PI / 2.0),
                };
                
                self.waveform_synthesizer.configure_channel(channel, config)
                    .map_err(|_| GeneratorError::ConfigurationFailed)?;
            }
        }
        
        Ok(())
    }
    
    fn configure_modulated_output(&mut self) -> Result<(), GeneratorError> {
        // 配置调制输出
        self.modulator.set_carrier_frequency(self.output_settings.frequency)
            .map_err(|_| GeneratorError::ConfigurationFailed)?;
        
        Ok(())
    }
    
    fn configure_sweep_output(&mut self) -> Result<(), GeneratorError> {
        // 配置扫频输出
        self.sweep_generator.configure(
            self.output_settings.frequency * 0.1, // 起始频率
            self.output_settings.frequency * 10.0, // 结束频率
            1.0, // 扫描时间（秒）
            SweepType::Linear,
        )?;
        
        Ok(())
    }
    
    fn configure_burst_output(&mut self) -> Result<(), GeneratorError> {
        // 配置突发输出
        self.burst_generator.configure(
            self.output_settings.frequency,
            100, // 突发周期数
            1000.0, // 重复频率
        )?;
        
        Ok(())
    }
    
    fn configure_arbitrary_waveform(&mut self) -> Result<(), GeneratorError> {
        // 配置任意波形输出
        // 这里可以加载用户定义的波形数据
        Ok(())
    }
    
    pub fn generate_output_buffer(&mut self, buffer_size: usize) -> Result<(Vec<u16>, Vec<u16>), GeneratorError> {
        let mut ch1_buffer = vec![0u16; buffer_size];
        let mut ch2_buffer = vec![0u16; buffer_size];
        
        match self.current_mode {
            GeneratorMode::ContinuousWave => {
                let mut float_buffers = vec![vec![0.0f32; buffer_size]; 2];
                self.waveform_synthesizer.generate_samples(&mut float_buffers)
                    .map_err(|_| GeneratorError::GenerationFailed)?;
                
                // 转换为DAC值
                for i in 0..buffer_size {
                    ch1_buffer[i] = self.float_to_dac_value(float_buffers[0][i]);
                    ch2_buffer[i] = self.float_to_dac_value(float_buffers[1][i]);
                }
            }
            GeneratorMode::Modulated => {
                let mut float_buffer = vec![0.0f32; buffer_size];
                self.modulator.generate_modulated_buffer(&mut float_buffer);
                
                for i in 0..buffer_size {
                    let dac_value = self.float_to_dac_value(float_buffer[i]);
                    ch1_buffer[i] = dac_value;
                    ch2_buffer[i] = dac_value; // 两通道相同
                }
            }
            GeneratorMode::Sweep => {
                let mut float_buffer = vec![0.0f32; buffer_size];
                self.sweep_generator.generate_buffer(&mut float_buffer)?;
                
                for i in 0..buffer_size {
                    let dac_value = self.float_to_dac_value(float_buffer[i]);
                    ch1_buffer[i] = dac_value;
                    ch2_buffer[i] = dac_value;
                }
            }
            GeneratorMode::Burst => {
                let mut float_buffer = vec![0.0f32; buffer_size];
                self.burst_generator.generate_buffer(&mut float_buffer)?;
                
                for i in 0..buffer_size {
                    let dac_value = self.float_to_dac_value(float_buffer[i]);
                    ch1_buffer[i] = dac_value;
                    ch2_buffer[i] = dac_value;
                }
            }
            GeneratorMode::Arbitrary => {
                // 任意波形生成逻辑
                for i in 0..buffer_size {
                    ch1_buffer[i] = 2048; // 中点值
                    ch2_buffer[i] = 2048;
                }
            }
        }
        
        Ok((ch1_buffer, ch2_buffer))
    }
    
    fn float_to_dac_value(&self, sample: f32) -> u16 {
        let normalized = (sample + 1.0) / 2.0;
        (normalized.clamp(0.0, 1.0) * 4095.0) as u16
    }
    
    pub fn start_output(&mut self) -> Result<(), GeneratorError> {
        // 生成初始缓冲区
        let (ch1_buffer, ch2_buffer) = self.generate_output_buffer(1024)?;
        
        // 加载到DAC控制器
        let waveform_ch1 = WaveformData {
            samples: ch1_buffer.iter().map(|&v| (v as f32 / 4095.0) * 2.0 - 1.0).collect(),
            sample_rate: self.dac_controller.sample_rate as f32,
            frequency: self.output_settings.frequency,
        };
        
        let waveform_ch2 = WaveformData {
            samples: ch2_buffer.iter().map(|&v| (v as f32 / 4095.0) * 2.0 - 1.0).collect(),
            sample_rate: self.dac_controller.sample_rate as f32,
            frequency: self.output_settings.frequency,
        };
        
        self.dac_controller.load_waveform_ch1(&waveform_ch1)
            .map_err(|_| GeneratorError::DacError)?;
        self.dac_controller.load_waveform_ch2(&waveform_ch2)
            .map_err(|_| GeneratorError::DacError)?;
        
        // 启动输出
        self.dac_controller.start_output()
            .map_err(|_| GeneratorError::DacError)?;
        
        Ok(())
    }
    
    pub fn stop_output(&mut self) -> Result<(), GeneratorError> {
        self.dac_controller.stop_output()
            .map_err(|_| GeneratorError::DacError)
    }
    
    pub fn set_frequency(&mut self, frequency: f32) -> Result<(), GeneratorError> {
        if frequency <= 0.0 || frequency > 100_000.0 {
            return Err(GeneratorError::InvalidParameter);
        }
        
        self.output_settings.frequency = frequency;
        self.configure_current_mode()
    }
    
    pub fn set_amplitude(&mut self, amplitude: f32) -> Result<(), GeneratorError> {
        if amplitude < 0.0 || amplitude > 1.0 {
            return Err(GeneratorError::InvalidParameter);
        }
        
        self.output_settings.amplitude = amplitude;
        self.configure_current_mode()
    }
    
    pub fn set_offset(&mut self, offset: f32) -> Result<(), GeneratorError> {
        if offset.abs() > 1.0 {
            return Err(GeneratorError::InvalidParameter);
        }
        
        self.output_settings.offset = offset;
        self.configure_current_mode()
    }
    
    pub fn set_waveform_type(&mut self, waveform_type: WaveformType) -> Result<(), GeneratorError> {
        self.output_settings.waveform_type = waveform_type;
        self.configure_current_mode()
    }
    
    fn configure_current_mode(&mut self) -> Result<(), GeneratorError> {
        self.set_mode(self.current_mode)
    }
    
    pub fn enable_channel(&mut self, channel: usize, enabled: bool) -> Result<(), GeneratorError> {
        if channel >= 2 {
            return Err(GeneratorError::InvalidChannel);
        }
        
        self.output_settings.channel_enabled[channel] = enabled;
        Ok(())
    }
    
    pub fn get_output_info(&self) -> OutputInfo {
        OutputInfo {
            mode: self.current_mode,
            settings: self.output_settings.clone(),
            is_running: true, // 简化实现
        }
    }
}

// 扫频发生器
pub struct SweepGenerator {
    start_frequency: f32,
    stop_frequency: f32,
    sweep_time: f32,
    sweep_type: SweepType,
    current_frequency: f32,
    sample_rate: f32,
    time_counter: f32,
    generator: WaveformGenerator,
}

#[derive(Debug, Clone, Copy)]
pub enum SweepType {
    Linear,
    Logarithmic,
}

impl SweepGenerator {
    pub fn new(sample_rate: f32) -> Self {
        let mut generator = WaveformGenerator::new(sample_rate);
        generator.set_waveform_type(WaveformType::Sine);
        
        Self {
            start_frequency: 100.0,
            stop_frequency: 10000.0,
            sweep_time: 1.0,
            sweep_type: SweepType::Linear,
            current_frequency: 100.0,
            sample_rate,
            time_counter: 0.0,
            generator,
        }
    }
    
    pub fn configure(
        &mut self,
        start_freq: f32,
        stop_freq: f32,
        sweep_time: f32,
        sweep_type: SweepType,
    ) -> Result<(), GeneratorError> {
        if start_freq <= 0.0 || stop_freq <= 0.0 || sweep_time <= 0.0 {
            return Err(GeneratorError::InvalidParameter);
        }
        
        self.start_frequency = start_freq;
        self.stop_frequency = stop_freq;
        self.sweep_time = sweep_time;
        self.sweep_type = sweep_type;
        self.current_frequency = start_freq;
        self.time_counter = 0.0;
        
        self.generator.set_frequency(start_freq)
            .map_err(|_| GeneratorError::ConfigurationFailed)?;
        
        Ok(())
    }
    
    pub fn generate_buffer(&mut self, buffer: &mut [f32]) -> Result<(), GeneratorError> {
        let time_step = 1.0 / self.sample_rate;
        
        for sample in buffer {
            // 计算当前频率
            let progress = self.time_counter / self.sweep_time;
            
            if progress >= 1.0 {
                // 重置扫频
                self.time_counter = 0.0;
                self.current_frequency = self.start_frequency;
            } else {
                self.current_frequency = match self.sweep_type {
                    SweepType::Linear => {
                        self.start_frequency + progress * (self.stop_frequency - self.start_frequency)
                    }
                    SweepType::Logarithmic => {
                        self.start_frequency * (self.stop_frequency / self.start_frequency).powf(progress)
                    }
                };
            }
            
            // 更新发生器频率
            self.generator.set_frequency(self.current_frequency)
                .map_err(|_| GeneratorError::ConfigurationFailed)?;
            
            // 生成样本
            *sample = self.generator.generate_sample();
            
            // 更新时间计数器
            self.time_counter += time_step;
        }
        
        Ok(())
    }
}

// 突发发生器
pub struct BurstGenerator {
    frequency: f32,
    burst_count: u32,
    repeat_rate: f32,
    sample_rate: f32,
    generator: WaveformGenerator,
    burst_counter: u32,
    repeat_counter: f32,
    in_burst: bool,
}

impl BurstGenerator {
    pub fn new(sample_rate: f32) -> Self {
        let mut generator = WaveformGenerator::new(sample_rate);
        generator.set_waveform_type(WaveformType::Sine);
        
        Self {
            frequency: 1000.0,
            burst_count: 10,
            repeat_rate: 100.0,
            sample_rate,
            generator,
            burst_counter: 0,
            repeat_counter: 0.0,
            in_burst: false,
        }
    }
    
    pub fn configure(
        &mut self,
        frequency: f32,
        burst_count: u32,
        repeat_rate: f32,
    ) -> Result<(), GeneratorError> {
        if frequency <= 0.0 || burst_count == 0 || repeat_rate <= 0.0 {
            return Err(GeneratorError::InvalidParameter);
        }
        
        self.frequency = frequency;
        self.burst_count = burst_count;
        self.repeat_rate = repeat_rate;
        
        self.generator.set_frequency(frequency)
            .map_err(|_| GeneratorError::ConfigurationFailed)?;
        
        Ok(())
    }
    
    pub fn generate_buffer(&mut self, buffer: &mut [f32]) -> Result<(), GeneratorError> {
        let time_step = 1.0 / self.sample_rate;
        let repeat_period = 1.0 / self.repeat_rate;
        let cycles_per_burst = self.burst_count;
        let samples_per_cycle = (self.sample_rate / self.frequency) as u32;
        
        for sample in buffer {
            if !self.in_burst {
                // 检查是否开始新的突发
                if self.repeat_counter >= repeat_period {
                    self.in_burst = true;
                    self.burst_counter = 0;
                    self.repeat_counter = 0.0;
                    self.generator.reset_phase();
                }
            }
            
            if self.in_burst {
                // 生成突发信号
                *sample = self.generator.generate_sample();
                
                // 检查是否完成当前突发
                if self.burst_counter >= cycles_per_burst * samples_per_cycle {
                    self.in_burst = false;
                    self.burst_counter = 0;
                } else {
                    self.burst_counter += 1;
                }
            } else {
                // 静默期间
                *sample = 0.0;
            }
            
            self.repeat_counter += time_step;
        }
        
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct OutputInfo {
    pub mode: GeneratorMode,
    pub settings: OutputSettings,
    pub is_running: bool,
}

#[derive(Debug)]
pub enum GeneratorError {
    ConfigurationFailed,
    GenerationFailed,
    DacError,
    InvalidParameter,
    InvalidChannel,
}
```

## 性能优化和最佳实践

### 内存优化技术

```rust
// 内存优化的波形缓冲区管理
pub struct OptimizedWaveformBuffer {
    buffers: [Vec<u16>; 2], // 双缓冲
    active_buffer: usize,
    buffer_size: usize,
    sample_rate: f32,
    dma_half_complete: bool,
}

impl OptimizedWaveformBuffer {
    pub fn new(buffer_size: usize, sample_rate: f32) -> Self {
        Self {
            buffers: [
                vec![0u16; buffer_size],
                vec![0u16; buffer_size],
            ],
            active_buffer: 0,
            buffer_size,
            sample_rate,
            dma_half_complete: false,
        }
    }
    
    pub fn get_active_buffer(&mut self) -> &mut [u16] {
        &mut self.buffers[self.active_buffer]
    }
    
    pub fn get_inactive_buffer(&mut self) -> &mut [u16] {
        &mut self.buffers[1 - self.active_buffer]
    }
    
    pub fn swap_buffers(&mut self) {
        self.active_buffer = 1 - self.active_buffer;
    }
    
    pub fn handle_dma_interrupt(&mut self, half_complete: bool) {
        self.dma_half_complete = half_complete;
        
        if half_complete {
            // DMA传输完成一半，可以更新后半部分
            self.update_buffer_half(false);
        } else {
            // DMA传输完全完成，可以更新前半部分
            self.update_buffer_half(true);
        }
    }
    
    fn update_buffer_half(&mut self, first_half: bool) {
        let buffer = self.get_active_buffer();
        let start_idx = if first_half { 0 } else { self.buffer_size / 2 };
        let end_idx = if first_half { self.buffer_size / 2 } else { self.buffer_size };
        
        // 在这里更新缓冲区的指定部分
        for i in start_idx..end_idx {
            // 实时计算或从预计算表中获取值
            buffer[i] = self.calculate_sample(i);
        }
    }
    
    fn calculate_sample(&self, index: usize) -> u16 {
        // 简化的样本计算
        let phase = 2.0 * PI * index as f32 / self.buffer_size as f32;
        let sample = phase.sin();
        ((sample + 1.0) / 2.0 * 4095.0) as u16
    }
}
```

### 实时性能监控

```rust
// 性能监控器
pub struct PerformanceMonitor {
    sample_count: u64,
    error_count: u64,
    max_processing_time: u32,
    avg_processing_time: f32,
    cpu_usage: f32,
    memory_usage: usize,
    start_time: u64,
}

impl PerformanceMonitor {
    pub fn new() -> Self {
        Self {
            sample_count: 0,
            error_count: 0,
            max_processing_time: 0,
            avg_processing_time: 0.0,
            cpu_usage: 0.0,
            memory_usage: 0,
            start_time: 0, // 应该使用系统时钟
        }
    }
    
    pub fn record_processing_time(&mut self, processing_time: u32) {
        self.sample_count += 1;
        
        if processing_time > self.max_processing_time {
            self.max_processing_time = processing_time;
        }
        
        // 计算移动平均
        let alpha = 0.1; // 平滑因子
        self.avg_processing_time = alpha * processing_time as f32 + 
                                  (1.0 - alpha) * self.avg_processing_time;
    }
    
    pub fn record_error(&mut self) {
        self.error_count += 1;
    }
    
    pub fn get_statistics(&self) -> PerformanceStats {
        PerformanceStats {
            sample_count: self.sample_count,
            error_count: self.error_count,
            error_rate: self.error_count as f32 / self.sample_count as f32,
            max_processing_time: self.max_processing_time,
            avg_processing_time: self.avg_processing_time,
            cpu_usage: self.cpu_usage,
            memory_usage: self.memory_usage,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PerformanceStats {
    pub sample_count: u64,
    pub error_count: u64,
    pub error_rate: f32,
    pub max_processing_time: u32,
    pub avg_processing_time: f32,
    pub cpu_usage: f32,
    pub memory_usage: usize,
}
```

## 调试和测试

### 波形质量分析

```rust
// 波形质量分析器
pub struct WaveformAnalyzer {
    fft_processor: FFTProcessor,
    thd_analyzer: THDAnalyzer,
    snr_analyzer: SNRAnalyzer,
}

impl WaveformAnalyzer {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            fft_processor: FFTProcessor::new(1024),
            thd_analyzer: THDAnalyzer::new(),
            snr_analyzer: SNRAnalyzer::new(sample_rate),
        }
    }
    
    pub fn analyze_waveform(&mut self, samples: &[f32]) -> WaveformAnalysis {
        let spectrum = self.fft_processor.compute_fft(samples);
        let thd = self.thd_analyzer.calculate_thd(&spectrum);
        let snr = self.snr_analyzer.calculate_snr(&spectrum);
        
        WaveformAnalysis {
            thd_percent: thd * 100.0,
            snr_db: 20.0 * snr.log10(),
            fundamental_frequency: self.find_fundamental_frequency(&spectrum),
            peak_amplitude: samples.iter().fold(0.0f32, |acc, &x| acc.max(x.abs())),
            rms_amplitude: self.calculate_rms(samples),
            spectrum: spectrum.clone(),
        }
    }
    
    fn find_fundamental_frequency(&self, spectrum: &[f32]) -> f32 {
        // 简化的基频检测
        let mut max_magnitude = 0.0;
        let mut fundamental_bin = 0;
        
        for (i, &magnitude) in spectrum.iter().enumerate().skip(1) {
            if magnitude > max_magnitude {
                max_magnitude = magnitude;
                fundamental_bin = i;
            }
        }
        
        fundamental_bin as f32 * self.snr_analyzer.sample_rate / spectrum.len() as f32
    }
    
    fn calculate_rms(&self, samples: &[f32]) -> f32 {
        let sum_squares: f32 = samples.iter().map(|&x| x * x).sum();
        (sum_squares / samples.len() as f32).sqrt()
    }
}

#[derive(Debug, Clone)]
pub struct WaveformAnalysis {
    pub thd_percent: f32,
    pub snr_db: f32,
    pub fundamental_frequency: f32,
    pub peak_amplitude: f32,
    pub rms_amplitude: f32,
    pub spectrum: Vec<f32>,
}
```

## 总结

本章详细介绍了DAC波形生成的核心技术和实际应用。通过学习本章内容，您应该能够：

1. **理解波形生成原理**：掌握数字波形合成、查找表技术和实时计算方法
2. **实现多通道同步**：配置STM32F4双通道DAC，实现相位锁定和频率同步
3. **应用调制技术**：实现频率调制、相位调制和幅度调制
4. **优化系统性能**：使用高精度DDS、内存优化和实时监控技术
5. **构建实际应用**：开发函数发生器、音频合成器等复杂系统

## 最佳实践

1. **精度优化**：使用高分辨率相位累加器和优化的查找表
2. **实时性保证**：采用双缓冲技术和中断驱动的数据更新
3. **噪声控制**：实施抗混叠滤波和适当的采样率选择
4. **系统监控**：集成性能监控和质量分析功能
5. **模块化设计**：构建可重用的波形生成组件

## 参考资料

### 技术文档
- STM32F4xx DAC参考手册
- 数字信号处理理论与应用
- 高精度频率合成技术

### 学术资源
- IEEE信号处理期刊相关论文
- 数字频率合成(DDS)技术研究
- 波形生成算法优化方法

### 开源项目
- GNU Radio波形生成模块
- SigGen开源信号发生器
- STM32 DAC应用示例