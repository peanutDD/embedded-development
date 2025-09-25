# 多通道ADC采样

## 概述

多通道ADC采样允许系统同时监测多个模拟信号，是复杂嵌入式系统的核心功能。本章详细介绍STM32F4多通道ADC的配置和使用，包括扫描模式、DMA传输、同步采样、通道管理和性能优化等关键技术。

## 多通道ADC基础

### 工作模式

**扫描模式（Scan Mode）**：
- 自动依次转换多个通道
- 可配置转换顺序
- 支持不同采样时间

**连续模式（Continuous Mode）**：
- 完成一轮扫描后自动开始下一轮
- 适合实时监测应用

**单次模式（Single Mode）**：
- 完成一轮扫描后停止
- 适合按需采样应用

### 通道配置

```rust
use stm32f4xx_hal::{
    gpio::{Analog, Pin},
    pac::{ADC1, DMA2},
    adc::{Adc, config::*},
    dma::{Stream0, Channel0, MemoryToPeripheral, StreamsTuple},
};

// 多通道GPIO配置
pub struct MultiChannelPins {
    pub ch0: Pin<'A', 0, Analog>, // PA0 - ADC1_IN0
    pub ch1: Pin<'A', 1, Analog>, // PA1 - ADC1_IN1
    pub ch2: Pin<'A', 2, Analog>, // PA2 - ADC1_IN2
    pub ch3: Pin<'A', 3, Analog>, // PA3 - ADC1_IN3
    pub ch4: Pin<'A', 4, Analog>, // PA4 - ADC1_IN4
    pub ch5: Pin<'A', 5, Analog>, // PA5 - ADC1_IN5
}

impl MultiChannelPins {
    pub fn new(gpioa: gpio::gpioa::Parts) -> Self {
        Self {
            ch0: gpioa.pa0.into_analog(),
            ch1: gpioa.pa1.into_analog(),
            ch2: gpioa.pa2.into_analog(),
            ch3: gpioa.pa3.into_analog(),
            ch4: gpioa.pa4.into_analog(),
            ch5: gpioa.pa5.into_analog(),
        }
    }
}
```

## 扫描模式配置

### 基本扫描配置

```rust
// 多通道扫描ADC配置
pub struct MultiChannelAdc {
    adc: Adc<ADC1>,
    pins: MultiChannelPins,
    channel_count: usize,
    sequence: [u8; 16], // 转换序列
}

impl MultiChannelAdc {
    pub fn new(adc_peripheral: ADC1, pins: MultiChannelPins) -> Self {
        // ADC配置
        let config = AdcConfig::default()
            .clock(Clock::Pclk2_div_4)
            .resolution(Resolution::Twelve)
            .align(Align::Right)
            .scan(Scan::Enabled)           // 启用扫描模式
            .continuous(Continuous::Single) // 单次扫描
            .external_trigger(ExternalTrigger::Disable)
            .end_of_conversion(Eoc::Sequence) // 序列结束中断
            .dma(Dma::Enabled);            // 启用DMA
        
        let mut adc = Adc::adc1(adc_peripheral, true, config);
        
        // 配置转换序列
        let sequence = [0, 1, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let channel_count = 6;
        
        Self {
            adc,
            pins,
            channel_count,
            sequence,
        }
    }
    
    pub fn configure_sequence(&mut self, channels: &[u8]) {
        self.channel_count = channels.len().min(16);
        self.sequence[..self.channel_count].copy_from_slice(&channels[..self.channel_count]);
        
        // 配置ADC序列寄存器
        self.adc.set_regular_sequence(&self.sequence[..self.channel_count]);
    }
    
    pub fn configure_sampling_times(&mut self) {
        // 为每个通道配置采样时间
        self.adc.set_channel_sample_time(0, SampleTime::Cycles_84);
        self.adc.set_channel_sample_time(1, SampleTime::Cycles_84);
        self.adc.set_channel_sample_time(2, SampleTime::Cycles_84);
        self.adc.set_channel_sample_time(3, SampleTime::Cycles_84);
        self.adc.set_channel_sample_time(4, SampleTime::Cycles_84);
        self.adc.set_channel_sample_time(5, SampleTime::Cycles_84);
    }
}
```

### 手动扫描实现

```rust
// 手动多通道扫描
impl MultiChannelAdc {
    pub fn scan_all_channels(&mut self) -> Result<[u16; 6], AdcError> {
        let mut results = [0u16; 6];
        
        // 依次读取每个通道
        results[0] = self.adc.convert(&mut self.pins.ch0, SampleTime::Cycles_84)?;
        results[1] = self.adc.convert(&mut self.pins.ch1, SampleTime::Cycles_84)?;
        results[2] = self.adc.convert(&mut self.pins.ch2, SampleTime::Cycles_84)?;
        results[3] = self.adc.convert(&mut self.pins.ch3, SampleTime::Cycles_84)?;
        results[4] = self.adc.convert(&mut self.pins.ch4, SampleTime::Cycles_84)?;
        results[5] = self.adc.convert(&mut self.pins.ch5, SampleTime::Cycles_84)?;
        
        Ok(results)
    }
    
    pub fn scan_selected_channels(&mut self, channels: &[usize]) -> Result<Vec<u16>, AdcError> {
        let mut results = Vec::new();
        
        for &channel in channels {
            let value = match channel {
                0 => self.adc.convert(&mut self.pins.ch0, SampleTime::Cycles_84)?,
                1 => self.adc.convert(&mut self.pins.ch1, SampleTime::Cycles_84)?,
                2 => self.adc.convert(&mut self.pins.ch2, SampleTime::Cycles_84)?,
                3 => self.adc.convert(&mut self.pins.ch3, SampleTime::Cycles_84)?,
                4 => self.adc.convert(&mut self.pins.ch4, SampleTime::Cycles_84)?,
                5 => self.adc.convert(&mut self.pins.ch5, SampleTime::Cycles_84)?,
                _ => return Err(AdcError::InvalidChannel),
            };
            results.push(value);
        }
        
        Ok(results)
    }
    
    pub fn scan_with_averaging(&mut self, samples_per_channel: usize) -> Result<[u16; 6], AdcError> {
        let mut sums = [0u32; 6];
        
        for _ in 0..samples_per_channel {
            let scan_result = self.scan_all_channels()?;
            for (i, &value) in scan_result.iter().enumerate() {
                sums[i] += value as u32;
            }
            delay_us(10); // 采样间隔
        }
        
        let mut averages = [0u16; 6];
        for (i, &sum) in sums.iter().enumerate() {
            averages[i] = (sum / samples_per_channel as u32) as u16;
        }
        
        Ok(averages)
    }
}
```

## DMA多通道采样

### DMA配置

```rust
use stm32f4xx_hal::dma::{
    Stream0, Channel0, PeripheralToMemory, 
    Transfer, StreamsTuple, DmaFlag
};

// DMA多通道ADC
pub struct DmaMultiChannelAdc {
    transfer: Option<Transfer<Stream0<DMA2>, Channel0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16]>>,
    buffer: &'static mut [u16],
    channel_count: usize,
}

impl DmaMultiChannelAdc {
    pub fn new(
        adc: Adc<ADC1>,
        dma_stream: Stream0<DMA2>,
        buffer: &'static mut [u16],
        channel_count: usize,
    ) -> Self {
        // 配置DMA传输
        let transfer = Transfer::init_peripheral_to_memory(
            dma_stream,
            adc,
            buffer,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .peripheral_increment(false)
                .circular_buffer(true)
                .transfer_complete_interrupt(true)
                .half_transfer_interrupt(true),
        );
        
        Self {
            transfer: Some(transfer),
            buffer,
            channel_count,
        }
    }
    
    pub fn start_continuous_scan(&mut self) -> Result<(), DmaError> {
        if let Some(transfer) = self.transfer.take() {
            let (stream, adc, buffer, _) = transfer.free();
            
            // 重新初始化传输
            let transfer = Transfer::init_peripheral_to_memory(
                stream,
                adc,
                buffer,
                None,
                DmaConfig::default()
                    .memory_increment(true)
                    .peripheral_increment(false)
                    .circular_buffer(true)
                    .transfer_complete_interrupt(true),
            );
            
            // 启动传输
            transfer.start(|adc| {
                adc.start_conversion();
            });
            
            self.transfer = Some(transfer);
            Ok(())
        } else {
            Err(DmaError::TransferInProgress)
        }
    }
    
    pub fn get_latest_data(&self) -> &[u16] {
        &self.buffer[..self.channel_count]
    }
    
    pub fn get_channel_data(&self, channel: usize) -> Option<&[u16]> {
        if channel < self.channel_count {
            let start = channel;
            let step = self.channel_count;
            let mut data = Vec::new();
            
            let mut index = start;
            while index < self.buffer.len() {
                data.push(self.buffer[index]);
                index += step;
            }
            
            Some(&data)
        } else {
            None
        }
    }
}
```

### 循环缓冲区管理

```rust
// 循环缓冲区管理器
pub struct CircularBuffer<T, const N: usize> {
    buffer: [T; N],
    write_index: usize,
    read_index: usize,
    full: bool,
}

impl<T: Copy + Default, const N: usize> CircularBuffer<T, N> {
    pub fn new() -> Self {
        Self {
            buffer: [T::default(); N],
            write_index: 0,
            read_index: 0,
            full: false,
        }
    }
    
    pub fn write(&mut self, data: &[T]) {
        for &item in data {
            self.buffer[self.write_index] = item;
            self.write_index = (self.write_index + 1) % N;
            
            if self.write_index == self.read_index {
                self.full = true;
                self.read_index = (self.read_index + 1) % N;
            }
        }
    }
    
    pub fn read(&mut self, data: &mut [T]) -> usize {
        let mut count = 0;
        
        while count < data.len() && !self.is_empty() {
            data[count] = self.buffer[self.read_index];
            self.read_index = (self.read_index + 1) % N;
            self.full = false;
            count += 1;
        }
        
        count
    }
    
    pub fn is_empty(&self) -> bool {
        !self.full && self.read_index == self.write_index
    }
    
    pub fn is_full(&self) -> bool {
        self.full
    }
    
    pub fn available_data(&self) -> usize {
        if self.full {
            N
        } else if self.write_index >= self.read_index {
            self.write_index - self.read_index
        } else {
            N - self.read_index + self.write_index
        }
    }
}

// 多通道循环缓冲区
pub struct MultiChannelBuffer {
    buffers: [CircularBuffer<u16, 1024>; 8],
    channel_count: usize,
}

impl MultiChannelBuffer {
    pub fn new(channel_count: usize) -> Self {
        Self {
            buffers: [
                CircularBuffer::new(), CircularBuffer::new(),
                CircularBuffer::new(), CircularBuffer::new(),
                CircularBuffer::new(), CircularBuffer::new(),
                CircularBuffer::new(), CircularBuffer::new(),
            ],
            channel_count: channel_count.min(8),
        }
    }
    
    pub fn write_scan_data(&mut self, data: &[u16]) {
        if data.len() == self.channel_count {
            for (i, &value) in data.iter().enumerate() {
                self.buffers[i].write(&[value]);
            }
        }
    }
    
    pub fn read_channel_data(&mut self, channel: usize, data: &mut [u16]) -> usize {
        if channel < self.channel_count {
            self.buffers[channel].read(data)
        } else {
            0
        }
    }
    
    pub fn get_channel_stats(&self, channel: usize) -> Option<ChannelStats> {
        if channel < self.channel_count {
            Some(ChannelStats {
                available_samples: self.buffers[channel].available_data(),
                is_full: self.buffers[channel].is_full(),
            })
        } else {
            None
        }
    }
}

#[derive(Debug)]
pub struct ChannelStats {
    pub available_samples: usize,
    pub is_full: bool,
}
```

## 中断驱动采样

### 中断配置

```rust
// 中断驱动多通道ADC
pub struct InterruptDrivenAdc {
    adc: Adc<ADC1>,
    current_channel: usize,
    channel_sequence: [usize; 8],
    channel_count: usize,
    results: [u16; 8],
    scan_complete: bool,
}

impl InterruptDrivenAdc {
    pub fn new(adc: Adc<ADC1>, channels: &[usize]) -> Self {
        let mut channel_sequence = [0; 8];
        let channel_count = channels.len().min(8);
        channel_sequence[..channel_count].copy_from_slice(&channels[..channel_count]);
        
        Self {
            adc,
            current_channel: 0,
            channel_sequence,
            channel_count,
            results: [0; 8],
            scan_complete: false,
        }
    }
    
    pub fn start_scan(&mut self) {
        self.current_channel = 0;
        self.scan_complete = false;
        
        // 启用转换完成中断
        self.adc.listen(Event::EndOfConversion);
        
        // 开始第一个通道的转换
        self.start_next_channel();
    }
    
    fn start_next_channel(&mut self) {
        if self.current_channel < self.channel_count {
            let channel = self.channel_sequence[self.current_channel];
            // 启动指定通道的转换
            self.adc.start_conversion_channel(channel);
        }
    }
    
    // 在ADC中断处理函数中调用
    pub fn handle_conversion_complete(&mut self) {
        if self.current_channel < self.channel_count {
            // 读取转换结果
            self.results[self.current_channel] = self.adc.current_sample();
            self.current_channel += 1;
            
            if self.current_channel < self.channel_count {
                // 继续下一个通道
                self.start_next_channel();
            } else {
                // 扫描完成
                self.scan_complete = true;
                self.adc.unlisten(Event::EndOfConversion);
            }
        }
    }
    
    pub fn is_scan_complete(&self) -> bool {
        self.scan_complete
    }
    
    pub fn get_results(&self) -> &[u16] {
        &self.results[..self.channel_count]
    }
}

// 中断处理函数示例
#[interrupt]
fn ADC() {
    cortex_m::interrupt::free(|cs| {
        if let Some(adc) = ADC_INSTANCE.borrow(cs).borrow_mut().as_mut() {
            adc.handle_conversion_complete();
        }
    });
}
```

### 定时器触发扫描

```rust
use stm32f4xx_hal::timer::{Timer, Event as TimerEvent};

// 定时器触发的多通道采样
pub struct TimerTriggeredSampling {
    adc: InterruptDrivenAdc,
    timer: Timer<TIM2>,
    sampling_rate_hz: u32,
    auto_restart: bool,
}

impl TimerTriggeredSampling {
    pub fn new(
        adc: InterruptDrivenAdc,
        timer: Timer<TIM2>,
        sampling_rate_hz: u32,
    ) -> Self {
        Self {
            adc,
            timer,
            sampling_rate_hz,
            auto_restart: false,
        }
    }
    
    pub fn start_periodic_sampling(&mut self, auto_restart: bool) {
        self.auto_restart = auto_restart;
        
        // 配置定时器
        let period = 1_000_000 / self.sampling_rate_hz; // 微秒
        self.timer.start(period.micros());
        self.timer.listen(TimerEvent::TimeOut);
        
        // 开始第一次扫描
        self.adc.start_scan();
    }
    
    pub fn handle_timer_interrupt(&mut self) {
        if self.adc.is_scan_complete() && self.auto_restart {
            // 开始新的扫描
            self.adc.start_scan();
        }
        
        // 清除定时器中断标志
        self.timer.clear_interrupt(TimerEvent::TimeOut);
    }
    
    pub fn stop_sampling(&mut self) {
        self.timer.unlisten(TimerEvent::TimeOut);
        self.auto_restart = false;
    }
    
    pub fn get_latest_results(&self) -> &[u16] {
        self.adc.get_results()
    }
}
```

## 通道管理和配置

### 动态通道配置

```rust
// 动态通道管理器
pub struct ChannelManager {
    active_channels: heapless::Vec<ChannelConfig, 16>,
    channel_map: [Option<usize>; 16], // 物理通道到逻辑通道的映射
}

#[derive(Debug, Clone)]
pub struct ChannelConfig {
    pub physical_channel: u8,
    pub logical_channel: usize,
    pub sample_time: SampleTime,
    pub calibration: ChannelCalibration,
    pub enabled: bool,
    pub name: heapless::String<32>,
}

#[derive(Debug, Clone)]
pub struct ChannelCalibration {
    pub offset: i16,
    pub gain: f32,
    pub reference_voltage: f32,
}

impl ChannelManager {
    pub fn new() -> Self {
        Self {
            active_channels: heapless::Vec::new(),
            channel_map: [None; 16],
        }
    }
    
    pub fn add_channel(&mut self, config: ChannelConfig) -> Result<(), ChannelError> {
        if config.physical_channel >= 16 {
            return Err(ChannelError::InvalidPhysicalChannel);
        }
        
        if self.active_channels.len() >= 16 {
            return Err(ChannelError::TooManyChannels);
        }
        
        // 检查物理通道是否已被使用
        if self.channel_map[config.physical_channel as usize].is_some() {
            return Err(ChannelError::ChannelAlreadyInUse);
        }
        
        let logical_index = self.active_channels.len();
        self.channel_map[config.physical_channel as usize] = Some(logical_index);
        
        let mut config = config;
        config.logical_channel = logical_index;
        
        self.active_channels.push(config).map_err(|_| ChannelError::TooManyChannels)?;
        
        Ok(())
    }
    
    pub fn remove_channel(&mut self, physical_channel: u8) -> Result<(), ChannelError> {
        if let Some(logical_index) = self.channel_map[physical_channel as usize] {
            self.active_channels.remove(logical_index);
            self.channel_map[physical_channel as usize] = None;
            
            // 更新映射
            for (phys_ch, logical_opt) in self.channel_map.iter_mut().enumerate() {
                if let Some(logical) = logical_opt {
                    if *logical > logical_index {
                        *logical -= 1;
                    }
                }
            }
            
            Ok(())
        } else {
            Err(ChannelError::ChannelNotFound)
        }
    }
    
    pub fn get_channel_config(&self, physical_channel: u8) -> Option<&ChannelConfig> {
        self.channel_map[physical_channel as usize]
            .and_then(|logical| self.active_channels.get(logical))
    }
    
    pub fn get_active_channels(&self) -> &[ChannelConfig] {
        &self.active_channels
    }
    
    pub fn get_enabled_channels(&self) -> impl Iterator<Item = &ChannelConfig> {
        self.active_channels.iter().filter(|ch| ch.enabled)
    }
    
    pub fn enable_channel(&mut self, physical_channel: u8, enabled: bool) -> Result<(), ChannelError> {
        if let Some(logical_index) = self.channel_map[physical_channel as usize] {
            if let Some(config) = self.active_channels.get_mut(logical_index) {
                config.enabled = enabled;
                Ok(())
            } else {
                Err(ChannelError::ChannelNotFound)
            }
        } else {
            Err(ChannelError::ChannelNotFound)
        }
    }
}

#[derive(Debug)]
pub enum ChannelError {
    InvalidPhysicalChannel,
    TooManyChannels,
    ChannelAlreadyInUse,
    ChannelNotFound,
}
```

### 通道校准管理

```rust
// 多通道校准管理
pub struct MultiChannelCalibration {
    calibrations: heapless::Vec<ChannelCalibration, 16>,
    temperature_compensation: bool,
    reference_temperature: f32,
}

impl MultiChannelCalibration {
    pub fn new() -> Self {
        Self {
            calibrations: heapless::Vec::new(),
            temperature_compensation: false,
            reference_temperature: 25.0,
        }
    }
    
    pub fn calibrate_channel(
        &mut self,
        channel: usize,
        zero_point_adc: u16,
        full_scale_adc: u16,
        full_scale_voltage: f32,
        reference_voltage: f32,
    ) -> Result<(), CalibrationError> {
        if channel >= self.calibrations.len() {
            return Err(CalibrationError::InvalidChannel);
        }
        
        // 计算偏移和增益
        let offset = zero_point_adc as i16;
        let gain = if full_scale_adc > zero_point_adc {
            let expected_adc = (full_scale_voltage / reference_voltage * 4095.0) as u16;
            let actual_range = full_scale_adc - zero_point_adc;
            let expected_range = expected_adc - zero_point_adc;
            expected_range as f32 / actual_range as f32
        } else {
            1.0
        };
        
        let calibration = ChannelCalibration {
            offset,
            gain,
            reference_voltage,
        };
        
        if let Some(cal) = self.calibrations.get_mut(channel) {
            *cal = calibration;
        } else {
            self.calibrations.push(calibration).map_err(|_| CalibrationError::TooManyChannels)?;
        }
        
        Ok(())
    }
    
    pub fn apply_calibration(
        &self,
        channel: usize,
        raw_values: &[u16],
        temperature: f32,
    ) -> Result<Vec<f32>, CalibrationError> {
        if channel >= self.calibrations.len() {
            return Err(CalibrationError::InvalidChannel);
        }
        
        let cal = &self.calibrations[channel];
        let mut calibrated_values = Vec::new();
        
        for &raw_value in raw_values {
            // 偏移校准
            let offset_corrected = (raw_value as i32 - cal.offset as i32).clamp(0, 4095) as u16;
            
            // 增益校准
            let gain_corrected = offset_corrected as f32 * cal.gain;
            
            // 转换为电压
            let voltage = (gain_corrected * cal.reference_voltage) / 4095.0;
            
            // 温度补偿
            let final_voltage = if self.temperature_compensation {
                let temp_coeff = 0.001; // 0.1%/°C
                let temp_error = (temperature - self.reference_temperature) * temp_coeff;
                voltage * (1.0 - temp_error)
            } else {
                voltage
            };
            
            calibrated_values.push(final_voltage);
        }
        
        Ok(calibrated_values)
    }
    
    pub fn enable_temperature_compensation(&mut self, enabled: bool, reference_temp: f32) {
        self.temperature_compensation = enabled;
        self.reference_temperature = reference_temp;
    }
}

#[derive(Debug)]
pub enum CalibrationError {
    InvalidChannel,
    TooManyChannels,
}
```

## 同步采样技术

### 多ADC同步

```rust
// 多ADC同步采样
pub struct SynchronizedMultiAdc {
    adc1: Adc<ADC1>,
    adc2: Adc<ADC2>,
    adc3: Adc<ADC3>,
    sync_mode: SyncMode,
}

#[derive(Debug, Clone, Copy)]
pub enum SyncMode {
    Independent,
    DualRegularSimultaneous,
    DualRegularAlternate,
    TripleRegularSimultaneous,
}

impl SynchronizedMultiAdc {
    pub fn new(
        adc1: Adc<ADC1>,
        adc2: Adc<ADC2>,
        adc3: Adc<ADC3>,
        sync_mode: SyncMode,
    ) -> Self {
        let mut instance = Self {
            adc1,
            adc2,
            adc3,
            sync_mode,
        };
        
        instance.configure_sync_mode();
        instance
    }
    
    fn configure_sync_mode(&mut self) {
        match self.sync_mode {
            SyncMode::DualRegularSimultaneous => {
                // 配置ADC1和ADC2同步采样
                self.adc1.set_dual_mode(DualMode::RegularSimultaneous);
                self.adc2.set_dual_mode(DualMode::RegularSimultaneous);
            }
            SyncMode::TripleRegularSimultaneous => {
                // 配置三个ADC同步采样
                self.adc1.set_triple_mode(TripleMode::RegularSimultaneous);
                self.adc2.set_triple_mode(TripleMode::RegularSimultaneous);
                self.adc3.set_triple_mode(TripleMode::RegularSimultaneous);
            }
            _ => {
                // 独立模式，无需特殊配置
            }
        }
    }
    
    pub fn synchronized_sample(&mut self) -> Result<SyncSampleResult, AdcError> {
        match self.sync_mode {
            SyncMode::DualRegularSimultaneous => {
                // 启动同步转换
                self.adc1.start_conversion();
                
                // 等待转换完成
                while !self.adc1.is_conversion_complete() {}
                
                let adc1_result = self.adc1.current_sample();
                let adc2_result = self.adc2.current_sample();
                
                Ok(SyncSampleResult::Dual(adc1_result, adc2_result))
            }
            SyncMode::TripleRegularSimultaneous => {
                // 启动同步转换
                self.adc1.start_conversion();
                
                // 等待转换完成
                while !self.adc1.is_conversion_complete() {}
                
                let adc1_result = self.adc1.current_sample();
                let adc2_result = self.adc2.current_sample();
                let adc3_result = self.adc3.current_sample();
                
                Ok(SyncSampleResult::Triple(adc1_result, adc2_result, adc3_result))
            }
            _ => {
                Err(AdcError::UnsupportedMode)
            }
        }
    }
}

#[derive(Debug)]
pub enum SyncSampleResult {
    Single(u16),
    Dual(u16, u16),
    Triple(u16, u16, u16),
}
```

### 时间同步采样

```rust
// 精确时间同步采样
pub struct TimeSynchronizedSampling {
    adc: MultiChannelAdc,
    timer: Timer<TIM2>,
    sample_buffer: CircularBuffer<TimestampedSample, 1024>,
    time_base_us: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct TimestampedSample {
    pub timestamp_us: u32,
    pub channel: u8,
    pub value: u16,
}

impl TimeSynchronizedSampling {
    pub fn new(adc: MultiChannelAdc, timer: Timer<TIM2>) -> Self {
        Self {
            adc,
            timer,
            sample_buffer: CircularBuffer::new(),
            time_base_us: 0,
        }
    }
    
    pub fn start_synchronized_sampling(&mut self, sample_rate_hz: u32) {
        // 配置定时器作为时间基准
        let period_us = 1_000_000 / sample_rate_hz;
        self.timer.start(period_us.micros());
        self.timer.listen(TimerEvent::TimeOut);
        
        self.time_base_us = 0;
    }
    
    pub fn handle_timer_interrupt(&mut self) {
        self.time_base_us += 1000; // 假设1ms中断
        
        // 执行同步采样
        if let Ok(results) = self.adc.scan_all_channels() {
            for (channel, &value) in results.iter().enumerate() {
                let sample = TimestampedSample {
                    timestamp_us: self.time_base_us,
                    channel: channel as u8,
                    value,
                };
                
                self.sample_buffer.write(&[sample]);
            }
        }
        
        self.timer.clear_interrupt(TimerEvent::TimeOut);
    }
    
    pub fn get_samples_in_time_range(
        &mut self,
        start_time_us: u32,
        end_time_us: u32,
    ) -> Vec<TimestampedSample> {
        let mut samples = Vec::new();
        let mut temp_buffer = [TimestampedSample {
            timestamp_us: 0,
            channel: 0,
            value: 0,
        }; 1024];
        
        let count = self.sample_buffer.read(&mut temp_buffer);
        
        for sample in &temp_buffer[..count] {
            if sample.timestamp_us >= start_time_us && sample.timestamp_us <= end_time_us {
                samples.push(*sample);
            }
        }
        
        samples
    }
    
    pub fn get_channel_samples_since(
        &mut self,
        channel: u8,
        since_time_us: u32,
    ) -> Vec<TimestampedSample> {
        let mut samples = Vec::new();
        let mut temp_buffer = [TimestampedSample {
            timestamp_us: 0,
            channel: 0,
            value: 0,
        }; 1024];
        
        let count = self.sample_buffer.read(&mut temp_buffer);
        
        for sample in &temp_buffer[..count] {
            if sample.channel == channel && sample.timestamp_us >= since_time_us {
                samples.push(*sample);
            }
        }
        
        samples
    }
}
```

## 数据处理和分析

### 多通道数据分析

```rust
// 多通道数据分析器
pub struct MultiChannelAnalyzer {
    channel_count: usize,
    window_size: usize,
    data_windows: Vec<CircularBuffer<f32, 256>>,
    statistics: Vec<ChannelStatistics>,
}

#[derive(Debug, Clone)]
pub struct ChannelStatistics {
    pub mean: f32,
    pub std_dev: f32,
    pub min: f32,
    pub max: f32,
    pub rms: f32,
    pub peak_to_peak: f32,
    pub sample_count: u32,
}

impl MultiChannelAnalyzer {
    pub fn new(channel_count: usize, window_size: usize) -> Self {
        let mut data_windows = Vec::new();
        let mut statistics = Vec::new();
        
        for _ in 0..channel_count {
            data_windows.push(CircularBuffer::new());
            statistics.push(ChannelStatistics {
                mean: 0.0,
                std_dev: 0.0,
                min: f32::MAX,
                max: f32::MIN,
                rms: 0.0,
                peak_to_peak: 0.0,
                sample_count: 0,
            });
        }
        
        Self {
            channel_count,
            window_size,
            data_windows,
            statistics,
        }
    }
    
    pub fn update_data(&mut self, channel_data: &[f32]) {
        if channel_data.len() != self.channel_count {
            return;
        }
        
        for (channel, &value) in channel_data.iter().enumerate() {
            self.data_windows[channel].write(&[value]);
            self.update_channel_statistics(channel, value);
        }
    }
    
    fn update_channel_statistics(&mut self, channel: usize, new_value: f32) {
        let stats = &mut self.statistics[channel];
        
        // 更新计数
        stats.sample_count += 1;
        
        // 更新最小值和最大值
        stats.min = stats.min.min(new_value);
        stats.max = stats.max.max(new_value);
        stats.peak_to_peak = stats.max - stats.min;
        
        // 增量更新均值
        let delta = new_value - stats.mean;
        stats.mean += delta / stats.sample_count as f32;
        
        // 更新RMS（简化版本）
        stats.rms = (stats.rms.powi(2) * (stats.sample_count - 1) as f32 + new_value.powi(2)) / stats.sample_count as f32;
        stats.rms = stats.rms.sqrt();
        
        // 计算标准差（需要窗口数据）
        if stats.sample_count >= 2 {
            stats.std_dev = self.calculate_std_dev(channel);
        }
    }
    
    fn calculate_std_dev(&self, channel: usize) -> f32 {
        let mut temp_data = [0.0f32; 256];
        let count = self.data_windows[channel].available_data().min(256);
        
        if count < 2 {
            return 0.0;
        }
        
        // 这里需要实现从循环缓冲区读取数据的方法
        // 简化实现
        let mean = self.statistics[channel].mean;
        let mut variance = 0.0;
        
        for i in 0..count {
            let diff = temp_data[i] - mean;
            variance += diff * diff;
        }
        
        (variance / (count - 1) as f32).sqrt()
    }
    
    pub fn get_channel_statistics(&self, channel: usize) -> Option<&ChannelStatistics> {
        self.statistics.get(channel)
    }
    
    pub fn get_all_statistics(&self) -> &[ChannelStatistics] {
        &self.statistics
    }
    
    pub fn detect_anomalies(&self, threshold_sigma: f32) -> Vec<ChannelAnomaly> {
        let mut anomalies = Vec::new();
        
        for (channel, stats) in self.statistics.iter().enumerate() {
            // 检测异常值
            let upper_threshold = stats.mean + threshold_sigma * stats.std_dev;
            let lower_threshold = stats.mean - threshold_sigma * stats.std_dev;
            
            // 获取最新值进行检查
            let mut latest_data = [0.0f32; 1];
            if self.data_windows[channel].available_data() > 0 {
                // 这里需要实现获取最新数据的方法
                let latest_value = latest_data[0]; // 简化
                
                if latest_value > upper_threshold {
                    anomalies.push(ChannelAnomaly {
                        channel,
                        anomaly_type: AnomalyType::HighValue,
                        value: latest_value,
                        threshold: upper_threshold,
                    });
                } else if latest_value < lower_threshold {
                    anomalies.push(ChannelAnomaly {
                        channel,
                        anomaly_type: AnomalyType::LowValue,
                        value: latest_value,
                        threshold: lower_threshold,
                    });
                }
            }
        }
        
        anomalies
    }
}

#[derive(Debug)]
pub struct ChannelAnomaly {
    pub channel: usize,
    pub anomaly_type: AnomalyType,
    pub value: f32,
    pub threshold: f32,
}

#[derive(Debug)]
pub enum AnomalyType {
    HighValue,
    LowValue,
    RapidChange,
    NoSignal,
}
```

### 通道间相关性分析

```rust
// 通道间相关性分析
pub struct ChannelCorrelationAnalyzer {
    channel_count: usize,
    correlation_matrix: Vec<Vec<f32>>,
    data_buffer: Vec<Vec<f32>>,
    buffer_size: usize,
}

impl ChannelCorrelationAnalyzer {
    pub fn new(channel_count: usize, buffer_size: usize) -> Self {
        let mut correlation_matrix = Vec::new();
        let mut data_buffer = Vec::new();
        
        for _ in 0..channel_count {
            correlation_matrix.push(vec![0.0; channel_count]);
            data_buffer.push(Vec::with_capacity(buffer_size));
        }
        
        Self {
            channel_count,
            correlation_matrix,
            data_buffer,
            buffer_size,
        }
    }
    
    pub fn update_data(&mut self, channel_data: &[f32]) {
        if channel_data.len() != self.channel_count {
            return;
        }
        
        for (channel, &value) in channel_data.iter().enumerate() {
            if self.data_buffer[channel].len() >= self.buffer_size {
                self.data_buffer[channel].remove(0);
            }
            self.data_buffer[channel].push(value);
        }
        
        // 更新相关性矩阵
        self.calculate_correlations();
    }
    
    fn calculate_correlations(&mut self) {
        for i in 0..self.channel_count {
            for j in 0..self.channel_count {
                if i == j {
                    self.correlation_matrix[i][j] = 1.0;
                } else {
                    self.correlation_matrix[i][j] = self.calculate_correlation(i, j);
                }
            }
        }
    }
    
    fn calculate_correlation(&self, channel1: usize, channel2: usize) -> f32 {
        let data1 = &self.data_buffer[channel1];
        let data2 = &self.data_buffer[channel2];
        
        let min_len = data1.len().min(data2.len());
        if min_len < 2 {
            return 0.0;
        }
        
        // 计算均值
        let mean1: f32 = data1.iter().take(min_len).sum::<f32>() / min_len as f32;
        let mean2: f32 = data2.iter().take(min_len).sum::<f32>() / min_len as f32;
        
        // 计算协方差和方差
        let mut covariance = 0.0;
        let mut var1 = 0.0;
        let mut var2 = 0.0;
        
        for i in 0..min_len {
            let diff1 = data1[i] - mean1;
            let diff2 = data2[i] - mean2;
            
            covariance += diff1 * diff2;
            var1 += diff1 * diff1;
            var2 += diff2 * diff2;
        }
        
        let denominator = (var1 * var2).sqrt();
        if denominator > 0.0 {
            covariance / denominator
        } else {
            0.0
        }
    }
    
    pub fn get_correlation(&self, channel1: usize, channel2: usize) -> Option<f32> {
        if channel1 < self.channel_count && channel2 < self.channel_count {
            Some(self.correlation_matrix[channel1][channel2])
        } else {
            None
        }
    }
    
    pub fn get_correlation_matrix(&self) -> &Vec<Vec<f32>> {
        &self.correlation_matrix
    }
    
    pub fn find_highly_correlated_pairs(&self, threshold: f32) -> Vec<CorrelatedPair> {
        let mut pairs = Vec::new();
        
        for i in 0..self.channel_count {
            for j in (i + 1)..self.channel_count {
                let correlation = self.correlation_matrix[i][j].abs();
                if correlation >= threshold {
                    pairs.push(CorrelatedPair {
                        channel1: i,
                        channel2: j,
                        correlation: self.correlation_matrix[i][j],
                    });
                }
            }
        }
        
        pairs
    }
}

#[derive(Debug)]
pub struct CorrelatedPair {
    pub channel1: usize,
    pub channel2: usize,
    pub correlation: f32,
}
```

## 性能优化

### 采样速度优化

```rust
// 高速多通道采样优化
pub struct HighSpeedMultiChannelAdc {
    adc: Adc<ADC1>,
    dma_transfer: Option<Transfer<Stream0<DMA2>, Channel0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16]>>,
    channel_count: usize,
    sample_rate_hz: u32,
    optimization_level: OptimizationLevel,
}

#[derive(Debug, Clone, Copy)]
pub enum OptimizationLevel {
    Balanced,
    MaxSpeed,
    MaxPrecision,
}

impl HighSpeedMultiChannelAdc {
    pub fn new(
        adc: Adc<ADC1>,
        dma_stream: Stream0<DMA2>,
        buffer: &'static mut [u16],
        channel_count: usize,
        optimization: OptimizationLevel,
    ) -> Self {
        let mut instance = Self {
            adc,
            dma_transfer: None,
            channel_count,
            sample_rate_hz: 1000,
            optimization_level: optimization,
        };
        
        instance.configure_for_optimization();
        instance.setup_dma(dma_stream, buffer);
        instance
    }
    
    fn configure_for_optimization(&mut self) {
        match self.optimization_level {
            OptimizationLevel::MaxSpeed => {
                // 最高速度配置
                self.adc.set_clock(Clock::Pclk2_div_2); // 最高时钟
                self.adc.set_resolution(Resolution::Six); // 降低分辨率提高速度
                self.adc.set_sample_time_all(SampleTime::Cycles_3); // 最短采样时间
                self.adc.set_continuous_mode(true);
                self.adc.set_scan_mode(true);
            }
            OptimizationLevel::MaxPrecision => {
                // 最高精度配置
                self.adc.set_clock(Clock::Pclk2_div_8); // 较低时钟减少噪声
                self.adc.set_resolution(Resolution::Twelve); // 最高分辨率
                self.adc.set_sample_time_all(SampleTime::Cycles_480); // 最长采样时间
                self.adc.set_continuous_mode(false);
                self.adc.set_scan_mode(true);
            }
            OptimizationLevel::Balanced => {
                // 平衡配置
                self.adc.set_clock(Clock::Pclk2_div_4);
                self.adc.set_resolution(Resolution::Twelve);
                self.adc.set_sample_time_all(SampleTime::Cycles_84);
                self.adc.set_continuous_mode(true);
                self.adc.set_scan_mode(true);
            }
        }
    }
    
    fn setup_dma(&mut self, dma_stream: Stream0<DMA2>, buffer: &'static mut [u16]) {
        let transfer = Transfer::init_peripheral_to_memory(
            dma_stream,
            self.adc,
            buffer,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .peripheral_increment(false)
                .circular_buffer(true)
                .transfer_complete_interrupt(true)
                .half_transfer_interrupt(true)
                .priority(Priority::VeryHigh), // 最高DMA优先级
        );
        
        self.dma_transfer = Some(transfer);
    }
    
    pub fn start_high_speed_sampling(&mut self, sample_rate_hz: u32) -> Result<(), AdcError> {
        self.sample_rate_hz = sample_rate_hz;
        
        if let Some(transfer) = self.dma_transfer.take() {
            transfer.start(|adc| {
                adc.start_conversion();
            });
            
            self.dma_transfer = Some(transfer);
            Ok(())
        } else {
            Err(AdcError::DmaNotConfigured)
        }
    }
    
    pub fn get_throughput_stats(&self) -> ThroughputStats {
        let samples_per_second = self.sample_rate_hz * self.channel_count as u32;
        let bytes_per_second = samples_per_second * 2; // 16位采样
        let conversion_time_us = match self.optimization_level {
            OptimizationLevel::MaxSpeed => 1,
            OptimizationLevel::Balanced => 5,
            OptimizationLevel::MaxPrecision => 20,
        };
        
        ThroughputStats {
            samples_per_second,
            bytes_per_second,
            conversion_time_us,
            theoretical_max_rate: 84_000_000 / (conversion_time_us * self.channel_count as u32),
        }
    }
}

#[derive(Debug)]
pub struct ThroughputStats {
    pub samples_per_second: u32,
    pub bytes_per_second: u32,
    pub conversion_time_us: u32,
    pub theoretical_max_rate: u32,
}
```

### 内存优化

```rust
// 内存优化的多通道数据管理
pub struct MemoryOptimizedChannelData<const CHANNELS: usize, const BUFFER_SIZE: usize> {
    // 使用固定大小数组避免堆分配
    raw_data: [[u16; BUFFER_SIZE]; CHANNELS],
    processed_data: [[f32; BUFFER_SIZE]; CHANNELS],
    write_indices: [usize; CHANNELS],
    data_counts: [usize; CHANNELS],
    compression_enabled: bool,
}

impl<const CHANNELS: usize, const BUFFER_SIZE: usize> MemoryOptimizedChannelData<CHANNELS, BUFFER_SIZE> {
    pub fn new() -> Self {
        Self {
            raw_data: [[0; BUFFER_SIZE]; CHANNELS],
            processed_data: [[0.0; BUFFER_SIZE]; CHANNELS],
            write_indices: [0; CHANNELS],
            data_counts: [0; CHANNELS],
            compression_enabled: false,
        }
    }
    
    pub fn add_sample(&mut self, channel: usize, value: u16) -> Result<(), DataError> {
        if channel >= CHANNELS {
            return Err(DataError::InvalidChannel);
        }
        
        let write_idx = self.write_indices[channel];
        self.raw_data[channel][write_idx] = value;
        
        // 更新写入索引（循环缓冲区）
        self.write_indices[channel] = (write_idx + 1) % BUFFER_SIZE;
        
        // 更新数据计数
        if self.data_counts[channel] < BUFFER_SIZE {
            self.data_counts[channel] += 1;
        }
        
        Ok(())
    }
    
    pub fn add_batch_samples(&mut self, channel_data: &[u16]) -> Result<(), DataError> {
        if channel_data.len() != CHANNELS {
            return Err(DataError::InvalidDataSize);
        }
        
        for (channel, &value) in channel_data.iter().enumerate() {
            self.add_sample(channel, value)?;
        }
        
        Ok(())
    }
    
    pub fn get_latest_samples(&self, channel: usize, count: usize) -> Result<&[u16], DataError> {
        if channel >= CHANNELS {
            return Err(DataError::InvalidChannel);
        }
        
        let available = self.data_counts[channel].min(count);
        let write_idx = self.write_indices[channel];
        
        if available == 0 {
            return Ok(&[]);
        }
        
        // 简化实现：返回最新的连续数据
        let start_idx = if write_idx >= available {
            write_idx - available
        } else {
            BUFFER_SIZE - (available - write_idx)
        };
        
        Ok(&self.raw_data[channel][start_idx..start_idx + available])
    }
    
    pub fn process_channel_data(&mut self, channel: usize, calibration: &ChannelCalibration) -> Result<(), DataError> {
        if channel >= CHANNELS {
            return Err(DataError::InvalidChannel);
        }
        
        let data_count = self.data_counts[channel];
        for i in 0..data_count {
            let raw_value = self.raw_data[channel][i];
            
            // 应用校准
            let offset_corrected = (raw_value as i32 - calibration.offset as i32).clamp(0, 4095) as u16;
            let gain_corrected = offset_corrected as f32 * calibration.gain;
            let voltage = (gain_corrected * calibration.reference_voltage) / 4095.0;
            
            self.processed_data[channel][i] = voltage;
        }
        
        Ok(())
    }
    
    pub fn get_memory_usage(&self) -> MemoryUsage {
        let raw_data_bytes = CHANNELS * BUFFER_SIZE * 2; // u16 = 2 bytes
        let processed_data_bytes = CHANNELS * BUFFER_SIZE * 4; // f32 = 4 bytes
        let metadata_bytes = CHANNELS * 2 * 4; // indices and counts, usize = 4 bytes
        
        MemoryUsage {
            total_bytes: raw_data_bytes + processed_data_bytes + metadata_bytes,
            raw_data_bytes,
            processed_data_bytes,
            metadata_bytes,
            utilization_percent: self.calculate_utilization(),
        }
    }
    
    fn calculate_utilization(&self) -> f32 {
        let total_slots = CHANNELS * BUFFER_SIZE;
        let used_slots: usize = self.data_counts.iter().sum();
        (used_slots as f32 / total_slots as f32) * 100.0
    }
    
    pub fn compress_data(&mut self) -> Result<CompressionStats, DataError> {
        if !self.compression_enabled {
            return Err(DataError::CompressionNotEnabled);
        }
        
        // 简单的差分压缩
        let mut compression_stats = CompressionStats {
            original_size: 0,
            compressed_size: 0,
            compression_ratio: 1.0,
        };
        
        for channel in 0..CHANNELS {
            let data_count = self.data_counts[channel];
            if data_count < 2 {
                continue;
            }
            
            compression_stats.original_size += data_count * 2; // u16 size
            
            // 差分编码
            let mut compressed_count = 1; // 第一个值不压缩
            for i in 1..data_count {
                let diff = self.raw_data[channel][i] as i32 - self.raw_data[channel][i-1] as i32;
                if diff.abs() < 128 {
                    compressed_count += 1; // 可以用8位存储
                } else {
                    compressed_count += 2; // 需要16位存储
                }
            }
            
            compression_stats.compressed_size += compressed_count;
        }
        
        compression_stats.compression_ratio = 
            compression_stats.original_size as f32 / compression_stats.compressed_size as f32;
        
        Ok(compression_stats)
    }
}

#[derive(Debug)]
pub struct MemoryUsage {
    pub total_bytes: usize,
    pub raw_data_bytes: usize,
    pub processed_data_bytes: usize,
    pub metadata_bytes: usize,
    pub utilization_percent: f32,
}

#[derive(Debug)]
pub struct CompressionStats {
    pub original_size: usize,
    pub compressed_size: usize,
    pub compression_ratio: f32,
}

#[derive(Debug)]
pub enum DataError {
    InvalidChannel,
    InvalidDataSize,
    CompressionNotEnabled,
}
```

## 实际应用示例

### 多传感器数据采集系统

```rust
// 多传感器数据采集系统
pub struct MultiSensorDataAcquisition {
    adc: HighSpeedMultiChannelAdc,
    sensors: Vec<SensorConfig>,
    data_manager: MemoryOptimizedChannelData<8, 1024>,
    analyzer: MultiChannelAnalyzer,
    calibration: MultiChannelCalibration,
}

#[derive(Debug, Clone)]
pub struct SensorConfig {
    pub channel: usize,
    pub sensor_type: SensorType,
    pub name: String,
    pub unit: String,
    pub range: (f32, f32),
    pub calibration: ChannelCalibration,
}

#[derive(Debug, Clone)]
pub enum SensorType {
    Temperature,
    Pressure,
    Humidity,
    LightIntensity,
    Voltage,
    Current,
    Acceleration,
    Custom(String),
}

impl MultiSensorDataAcquisition {
    pub fn new(adc: HighSpeedMultiChannelAdc) -> Self {
        Self {
            adc,
            sensors: Vec::new(),
            data_manager: MemoryOptimizedChannelData::new(),
            analyzer: MultiChannelAnalyzer::new(8, 256),
            calibration: MultiChannelCalibration::new(),
        }
    }
    
    pub fn add_sensor(&mut self, sensor: SensorConfig) -> Result<(), SystemError> {
        if sensor.channel >= 8 {
            return Err(SystemError::InvalidChannel);
        }
        
        // 检查通道是否已被使用
        if self.sensors.iter().any(|s| s.channel == sensor.channel) {
            return Err(SystemError::ChannelInUse);
        }
        
        self.sensors.push(sensor);
        Ok(())
    }
    
    pub fn start_acquisition(&mut self, sample_rate_hz: u32) -> Result<(), SystemError> {
        // 启动高速采样
        self.adc.start_high_speed_sampling(sample_rate_hz)
            .map_err(|_| SystemError::AdcStartFailed)?;
        
        Ok(())
    }
    
    pub fn process_new_data(&mut self, raw_data: &[u16]) -> Result<ProcessingResult, SystemError> {
        // 添加原始数据
        self.data_manager.add_batch_samples(raw_data)
            .map_err(|_| SystemError::DataProcessingFailed)?;
        
        // 处理每个传感器的数据
        let mut sensor_readings = Vec::new();
        
        for sensor in &self.sensors {
            // 获取最新数据
            let latest_samples = self.data_manager.get_latest_samples(sensor.channel, 1)
                .map_err(|_| SystemError::DataRetrievalFailed)?;
            
            if let Some(&raw_value) = latest_samples.first() {
                // 应用校准
                let calibrated_voltage = self.calibration.apply_calibration(
                    sensor.channel, 
                    &[raw_value], 
                    25.0 // 假设温度
                ).map_err(|_| SystemError::CalibrationFailed)?;
                
                if let Some(&voltage) = calibrated_voltage.first() {
                    // 转换为传感器读数
                    let sensor_value = self.convert_voltage_to_sensor_value(sensor, voltage);
                    
                    sensor_readings.push(SensorReading {
                        channel: sensor.channel,
                        sensor_type: sensor.sensor_type.clone(),
                        name: sensor.name.clone(),
                        raw_value,
                        voltage,
                        sensor_value,
                        unit: sensor.unit.clone(),
                        timestamp_ms: get_timestamp_ms(),
                    });
                }
            }
        }
        
        // 更新分析器
        let voltage_data: Vec<f32> = sensor_readings.iter().map(|r| r.voltage).collect();
        self.analyzer.update_data(&voltage_data);
        
        // 检测异常
        let anomalies = self.analyzer.detect_anomalies(3.0);
        
        Ok(ProcessingResult {
            sensor_readings,
            anomalies,
            memory_usage: self.data_manager.get_memory_usage(),
        })
    }
    
    fn convert_voltage_to_sensor_value(&self, sensor: &SensorConfig, voltage: f32) -> f32 {
        match sensor.sensor_type {
            SensorType::Temperature => {
                // LM35: 10mV/°C
                voltage / 0.01
            }
            SensorType::Pressure => {
                // 假设0-5V对应0-100 PSI
                (voltage / 5.0) * 100.0
            }
            SensorType::Humidity => {
                // 假设0-3.3V对应0-100% RH
                (voltage / 3.3) * 100.0
            }
            SensorType::LightIntensity => {
                // 光敏电阻，非线性转换
                let resistance = (3.3 - voltage) / voltage * 10000.0; // 10k上拉
                1000000.0 / resistance // 简化的光强公式
            }
            SensorType::Voltage => voltage,
            SensorType::Current => {
                // 假设通过分流电阻测量，1V = 1A
                voltage
            }
            SensorType::Acceleration => {
                // ADXL335: 330mV/g, 1.65V为0g
                (voltage - 1.65) / 0.33
            }
            SensorType::Custom(_) => voltage, // 直接返回电压
        }
    }
    
    pub fn get_system_status(&self) -> SystemStatus {
        let throughput = self.adc.get_throughput_stats();
        let memory = self.data_manager.get_memory_usage();
        
        SystemStatus {
            active_sensors: self.sensors.len(),
            throughput_stats: throughput,
            memory_usage: memory,
            anomaly_count: 0, // 需要从最近的处理结果获取
        }
    }
}

#[derive(Debug)]
pub struct SensorReading {
    pub channel: usize,
    pub sensor_type: SensorType,
    pub name: String,
    pub raw_value: u16,
    pub voltage: f32,
    pub sensor_value: f32,
    pub unit: String,
    pub timestamp_ms: u32,
}

#[derive(Debug)]
pub struct ProcessingResult {
    pub sensor_readings: Vec<SensorReading>,
    pub anomalies: Vec<ChannelAnomaly>,
    pub memory_usage: MemoryUsage,
}

#[derive(Debug)]
pub struct SystemStatus {
    pub active_sensors: usize,
    pub throughput_stats: ThroughputStats,
    pub memory_usage: MemoryUsage,
    pub anomaly_count: usize,
}

#[derive(Debug)]
pub enum SystemError {
    InvalidChannel,
    ChannelInUse,
    AdcStartFailed,
    DataProcessingFailed,
    DataRetrievalFailed,
    CalibrationFailed,
}
```

### 工业监测系统

```rust
// 工业多参数监测系统
pub struct IndustrialMonitoringSystem {
    data_acquisition: MultiSensorDataAcquisition,
    alarm_manager: AlarmManager,
    data_logger: DataLogger,
    communication: CommunicationInterface,
}

#[derive(Debug, Clone)]
pub struct AlarmConfig {
    pub channel: usize,
    pub alarm_type: AlarmType,
    pub threshold: f32,
    pub hysteresis: f32,
    pub enabled: bool,
    pub priority: AlarmPriority,
}

#[derive(Debug, Clone)]
pub enum AlarmType {
    HighLimit,
    LowLimit,
    RateOfChange,
    Deviation,
}

#[derive(Debug, Clone)]
pub enum AlarmPriority {
    Critical,
    High,
    Medium,
    Low,
}

pub struct AlarmManager {
    alarms: Vec<AlarmConfig>,
    active_alarms: Vec<ActiveAlarm>,
}

#[derive(Debug, Clone)]
pub struct ActiveAlarm {
    pub config: AlarmConfig,
    pub triggered_at: u32,
    pub current_value: f32,
    pub acknowledged: bool,
}

impl AlarmManager {
    pub fn new() -> Self {
        Self {
            alarms: Vec::new(),
            active_alarms: Vec::new(),
        }
    }
    
    pub fn add_alarm(&mut self, alarm: AlarmConfig) {
        self.alarms.push(alarm);
    }
    
    pub fn check_alarms(&mut self, readings: &[SensorReading]) {
        for reading in readings {
            for alarm in &self.alarms {
                if alarm.channel == reading.channel && alarm.enabled {
                    let triggered = match alarm.alarm_type {
                        AlarmType::HighLimit => reading.sensor_value > alarm.threshold,
                        AlarmType::LowLimit => reading.sensor_value < alarm.threshold,
                        AlarmType::RateOfChange => {
                            // 需要历史数据来计算变化率
                            false // 简化实现
                        }
                        AlarmType::Deviation => {
                            // 需要设定点来计算偏差
                            false // 简化实现
                        }
                    };
                    
                    if triggered {
                        self.trigger_alarm(alarm.clone(), reading.sensor_value, reading.timestamp_ms);
                    } else {
                        self.clear_alarm(alarm.channel, &alarm.alarm_type);
                    }
                }
            }
        }
    }
    
    fn trigger_alarm(&mut self, config: AlarmConfig, value: f32, timestamp: u32) {
        // 检查是否已经存在相同的告警
        if !self.active_alarms.iter().any(|a| 
            a.config.channel == config.channel && 
            std::mem::discriminant(&a.config.alarm_type) == std::mem::discriminant(&config.alarm_type)
        ) {
            self.active_alarms.push(ActiveAlarm {
                config,
                triggered_at: timestamp,
                current_value: value,
                acknowledged: false,
            });
        }
    }
    
    fn clear_alarm(&mut self, channel: usize, alarm_type: &AlarmType) {
        self.active_alarms.retain(|alarm| 
            !(alarm.config.channel == channel && 
              std::mem::discriminant(&alarm.config.alarm_type) == std::mem::discriminant(alarm_type))
        );
    }
    
    pub fn get_active_alarms(&self) -> &[ActiveAlarm] {
        &self.active_alarms
    }
    
    pub fn acknowledge_alarm(&mut self, channel: usize, alarm_type: &AlarmType) {
        for alarm in &mut self.active_alarms {
            if alarm.config.channel == channel && 
               std::mem::discriminant(&alarm.config.alarm_type) == std::mem::discriminant(alarm_type) {
                alarm.acknowledged = true;
            }
        }
    }
}
```

## 调试和测试

### 多通道测试工具

```rust
// 多通道ADC测试工具
pub struct MultiChannelTestSuite {
    adc: MultiChannelAdc,
    test_results: Vec<ChannelTestResult>,
}

#[derive(Debug)]
pub struct ChannelTestResult {
    pub channel: usize,
    pub noise_level: f32,
    pub linearity_error: f32,
    pub offset_error: f32,
    pub gain_error: f32,
    pub crosstalk: Vec<f32>, // 与其他通道的串扰
    pub passed: bool,
}

impl MultiChannelTestSuite {
    pub fn new(adc: MultiChannelAdc) -> Self {
        Self {
            adc,
            test_results: Vec::new(),
        }
    }
    
    pub fn run_comprehensive_test(&mut self) -> Result<TestSummary, TestError> {
        self.test_results.clear();
        
        // 测试每个通道
        for channel in 0..6 {
            let result = self.test_single_channel(channel)?;
            self.test_results.push(result);
        }
        
        // 测试通道间串扰
        self.test_channel_crosstalk()?;
        
        Ok(self.generate_test_summary())
    }
    
    fn test_single_channel(&mut self, channel: usize) -> Result<ChannelTestResult, TestError> {
        println!("Testing channel {}", channel);
        
        // 噪声测试
        let noise_level = self.test_noise_level(channel)?;
        
        // 线性度测试
        let linearity_error = self.test_linearity(channel)?;
        
        // 偏移测试
        let offset_error = self.test_offset(channel)?;
        
        // 增益测试
        let gain_error = self.test_gain(channel)?;
        
        let passed = noise_level < 2.0 && 
                    linearity_error < 0.5 && 
                    offset_error.abs() < 10.0 && 
                    gain_error.abs() < 1.0;
        
        Ok(ChannelTestResult {
            channel,
            noise_level,
            linearity_error,
            offset_error,
            gain_error,
            crosstalk: Vec::new(), // 稍后填充
            passed,
        })
    }
    
    fn test_noise_level(&mut self, channel: usize) -> Result<f32, TestError> {
        println!("Connect channel {} to a stable voltage source and press enter", channel);
        wait_for_input();
        
        let samples = 1000;
        let mut values = Vec::new();
        
        for _ in 0..samples {
            let scan_result = self.adc.scan_all_channels()?;
            values.push(scan_result[channel]);
            delay_ms(1);
        }
        
        // 计算标准差作为噪声水平
        let mean: f32 = values.iter().map(|&x| x as f32).sum::<f32>() / values.len() as f32;
        let variance: f32 = values.iter()
            .map(|&x| {
                let diff = x as f32 - mean;
                diff * diff
            })
            .sum::<f32>() / values.len() as f32;
        
        Ok(variance.sqrt())
    }
    
    fn test_linearity(&mut self, channel: usize) -> Result<f32, TestError> {
        let test_voltages = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.3];
        let mut linearity_errors = Vec::new();
        
        for &voltage in &test_voltages {
            println!("Set channel {} to {:.1}V and press enter", channel, voltage);
            wait_for_input();
            
            let scan_result = self.adc.scan_with_averaging(100)?;
            let measured_adc = scan_result[channel];
            let measured_voltage = (measured_adc as f32 * 3.3) / 4095.0;
            
            let error = (measured_voltage - voltage).abs();
            linearity_errors.push(error);
        }
        
        // 返回最大线性度误差
        Ok(linearity_errors.iter().fold(0.0f32, |a, &b| a.max(b)))
    }
    
    fn test_offset(&mut self, channel: usize) -> Result<f32, TestError> {
        println!("Connect channel {} to ground and press enter", channel);
        wait_for_input();
        
        let scan_result = self.adc.scan_with_averaging(100)?;
        let zero_reading = scan_result[channel];
        
        // 偏移误差（以ADC码为单位）
        Ok(zero_reading as f32)
    }
    
    fn test_gain(&mut self, channel: usize) -> Result<f32, TestError> {
        println!("Connect channel {} to 3.3V reference and press enter", channel);
        wait_for_input();
        
        let scan_result = self.adc.scan_with_averaging(100)?;
        let full_scale_reading = scan_result[channel];
        
        let expected_reading = 4095.0;
        let gain_error = ((full_scale_reading as f32 - expected_reading) / expected_reading) * 100.0;
        
        Ok(gain_error)
    }
    
    fn test_channel_crosstalk(&mut self) -> Result<(), TestError> {
        // 测试通道间串扰
        for test_channel in 0..6 {
            println!("Connect channel {} to 3.3V, others to ground", test_channel);
            wait_for_input();
            
            let scan_result = self.adc.scan_with_averaging(100)?;
            
            for (channel, &value) in scan_result.iter().enumerate() {
                if channel != test_channel {
                    let crosstalk_voltage = (value as f32 * 3.3) / 4095.0;
                    let crosstalk_percent = (crosstalk_voltage / 3.3) * 100.0;
                    
                    // 将串扰结果添加到测试结果中
                    if let Some(result) = self.test_results.get_mut(channel) {
                        result.crosstalk.push(crosstalk_percent);
                    }
                }
            }
        }
        
        Ok(())
    }
    
    fn generate_test_summary(&self) -> TestSummary {
        let passed_channels = self.test_results.iter().filter(|r| r.passed).count();
        let total_channels = self.test_results.len();
        
        let avg_noise = self.test_results.iter()
            .map(|r| r.noise_level)
            .sum::<f32>() / total_channels as f32;
        
        let max_crosstalk = self.test_results.iter()
            .flat_map(|r| &r.crosstalk)
            .fold(0.0f32, |a, &b| a.max(b));
        
        TestSummary {
            total_channels,
            passed_channels,
            overall_passed: passed_channels == total_channels,
            average_noise_level: avg_noise,
            maximum_crosstalk: max_crosstalk,
            test_results: self.test_results.clone(),
        }
    }
}

#[derive(Debug)]
pub struct TestSummary {
    pub total_channels: usize,
    pub passed_channels: usize,
    pub overall_passed: bool,
    pub average_noise_level: f32,
    pub maximum_crosstalk: f32,
    pub test_results: Vec<ChannelTestResult>,
}

#[derive(Debug)]
pub enum TestError {
    AdcError(AdcError),
    InvalidChannel,
    TestSetupError,
}

impl From<AdcError> for TestError {
    fn from(error: AdcError) -> Self {
        TestError::AdcError(error)
    }
}

// 辅助函数
fn wait_for_input() {
    // 在实际实现中，这里会等待用户输入
    delay_ms(1000); // 简化实现
}

fn get_timestamp_ms() -> u32 {
    // 返回当前时间戳（毫秒）
    0 // 简化实现
}
```

## 最佳实践

### 设计原则

1. **通道隔离**：
   - 使用适当的模拟开关
   - 避免通道间串扰
   - 合理的PCB布局

2. **时序管理**：
   - 合理配置采样时间
   - 避免转换冲突
   - 优化扫描顺序

3. **数据完整性**：
   - 实施校验和检查
   - 异常值检测
   - 数据备份机制

### 性能优化建议

1. **硬件优化**：
   - 选择合适的参考电压
   - 使用低噪声电源
   - 优化信号调理电路

2. **软件优化**：
   - 使用DMA减少CPU负载
   - 实施高效的数据结构
   - 优化中断处理

3. **系统优化**：
   - 平衡采样速度和精度
   - 合理分配内存资源
   - 实施有效的错误处理

## 总结

多通道ADC采样是复杂嵌入式系统的核心功能，需要综合考虑硬件配置、软件实现、性能优化和可靠性设计。通过本章的学习，您应该能够：

1. 配置和使用STM32F4的多通道ADC功能
2. 实现高效的DMA数据传输
3. 设计可靠的通道管理系统
4. 实施有效的数据处理和分析算法
5. 优化系统性能和资源使用
6. 进行全面的测试和验证

关键要点：
- 合理配置扫描序列和采样时间
- 使用DMA提高数据传输效率
- 实施有效的校准和滤波算法
- 设计完善的异常检测机制
- 进行充分的测试和验证

## 参考资料

1. STM32F4 Reference Manual - Multi-channel ADC
2. "Multi-channel Data Acquisition Systems" - Design Guide
3. "ADC Performance Optimization" - Application Note
4. "Industrial Monitoring Systems" - Best Practices Guide