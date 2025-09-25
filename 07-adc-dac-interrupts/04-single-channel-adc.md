# 单通道ADC采样

## 概述

单通道ADC采样是最基础的模拟信号采集方式，适用于简单的传感器读取、电压监测、温度测量等应用。本章详细介绍如何在STM32F4上实现高质量的单通道ADC采样，包括硬件配置、软件实现、精度优化和实际应用。

## 基本概念

### 单通道采样特点

**优势**：
- 配置简单
- 资源占用少
- 延迟最小
- 精度最高

**适用场景**：
- 电池电压监测
- 温度传感器读取
- 光强检测
- 压力测量
- 简单的用户输入（电位器）

### 采样流程

1. **初始化**：配置GPIO、ADC、时钟
2. **校准**：偏移和增益校准
3. **采样**：触发转换并读取结果
4. **处理**：数据转换和滤波
5. **输出**：显示或传输结果

## 硬件配置

### GPIO配置

```rust
use stm32f4xx_hal::{
    gpio::{Analog, Pin},
    pac::ADC1,
    adc::{Adc, config::AdcConfig},
};

// 配置ADC输入引脚
fn configure_adc_pin() -> Pin<'A', 0, Analog> {
    let dp = pac::Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split();
    
    // PA0配置为模拟输入
    gpioa.pa0.into_analog()
}
```

### ADC基本配置

```rust
// ADC基本配置
fn configure_adc(adc_peripheral: ADC1) -> Adc<ADC1> {
    let config = AdcConfig::default()
        .clock(Clock::Pclk2_div_4)      // ADC时钟21MHz
        .resolution(Resolution::Twelve) // 12位分辨率
        .align(Align::Right)           // 右对齐
        .scan(Scan::Disabled)          // 禁用扫描模式
        .external_trigger(ExternalTrigger::Disable) // 软件触发
        .continuous(Continuous::Single) // 单次转换
        .dma(Dma::Disabled);           // 禁用DMA
    
    Adc::adc1(adc_peripheral, true, config)
}
```

### 采样时间配置

```rust
use stm32f4xx_hal::adc::config::SampleTime;

// 配置采样时间
fn configure_sampling_time(adc: &mut Adc<ADC1>, pin: &mut Pin<'A', 0, Analog>) {
    // 根据源阻抗选择采样时间
    // 高阻抗源：使用较长采样时间
    // 低阻抗源：可使用较短采样时间
    
    adc.configure_channel(pin, Sequence::One, SampleTime::Cycles_480);
}
```

## 基本采样实现

### 单次采样

```rust
// 单次ADC采样
fn single_sample(adc: &mut Adc<ADC1>, pin: &mut Pin<'A', 0, Analog>) -> Result<u16, AdcError> {
    // 启动转换并等待完成
    adc.convert(pin, SampleTime::Cycles_480)
}

// 带超时的采样
fn single_sample_with_timeout(
    adc: &mut Adc<ADC1>, 
    pin: &mut Pin<'A', 0, Analog>,
    timeout_ms: u32
) -> Result<u16, AdcError> {
    let start_time = get_time_ms();
    
    loop {
        if let Ok(value) = adc.convert(pin, SampleTime::Cycles_480) {
            return Ok(value);
        }
        
        if get_time_ms() - start_time > timeout_ms {
            return Err(AdcError::Timeout);
        }
    }
}
```

### 多次采样平均

```rust
// 多次采样求平均
fn averaged_sample(
    adc: &mut Adc<ADC1>, 
    pin: &mut Pin<'A', 0, Analog>,
    samples: usize
) -> Result<u16, AdcError> {
    let mut sum = 0u32;
    let mut valid_samples = 0;
    
    for _ in 0..samples {
        match adc.convert(pin, SampleTime::Cycles_480) {
            Ok(value) => {
                sum += value as u32;
                valid_samples += 1;
            }
            Err(_) => continue, // 忽略错误的采样
        }
        
        // 采样间隔，避免过快采样
        delay_us(10);
    }
    
    if valid_samples > 0 {
        Ok((sum / valid_samples) as u16)
    } else {
        Err(AdcError::NoValidSamples)
    }
}
```

### 过采样技术

```rust
// 过采样提高分辨率
fn oversampled_reading(
    adc: &mut Adc<ADC1>, 
    pin: &mut Pin<'A', 0, Analog>,
    oversample_ratio: u32
) -> Result<u32, AdcError> {
    let mut sum = 0u64;
    
    for _ in 0..oversample_ratio {
        let sample = adc.convert(pin, SampleTime::Cycles_480)?;
        sum += sample as u64;
        delay_us(1); // 短暂延迟
    }
    
    // 过采样可以提高分辨率
    // 4倍过采样提高1位分辨率
    // 16倍过采样提高2位分辨率
    // 64倍过采样提高3位分辨率
    Ok((sum / oversample_ratio as u64) as u32)
}

// 计算有效分辨率
fn calculate_effective_resolution(oversample_ratio: u32) -> f32 {
    12.0 + 0.5 * (oversample_ratio as f32).log2()
}
```

## 数据转换和校准

### 原始值转换

```rust
// ADC原始值转换为电压
fn adc_to_voltage(adc_value: u16, vref: f32) -> f32 {
    (adc_value as f32 * vref) / 4095.0
}

// 电压转换为ADC值
fn voltage_to_adc(voltage: f32, vref: f32) -> u16 {
    ((voltage * 4095.0) / vref).clamp(0.0, 4095.0) as u16
}

// 考虑分辨率的转换
fn adc_to_voltage_precise(adc_value: u16, vref: f32, resolution_bits: u8) -> f32 {
    let max_value = (1 << resolution_bits) - 1;
    (adc_value as f32 * vref) / max_value as f32
}
```

### 校准实现

```rust
// ADC校准结构
#[derive(Debug, Clone)]
pub struct AdcCalibration {
    pub offset: i16,        // 偏移校准值
    pub gain: f32,          // 增益校准系数
    pub vref: f32,          // 参考电压
    pub temp_coeff: f32,    // 温度系数
}

impl AdcCalibration {
    pub fn new() -> Self {
        Self {
            offset: 0,
            gain: 1.0,
            vref: 3.3,
            temp_coeff: 0.0,
        }
    }
    
    // 应用校准
    pub fn apply(&self, raw_value: u16, temperature: f32) -> f32 {
        // 偏移校准
        let offset_corrected = raw_value as i32 - self.offset as i32;
        let offset_corrected = offset_corrected.clamp(0, 4095) as u16;
        
        // 增益校准
        let gain_corrected = offset_corrected as f32 * self.gain;
        
        // 转换为电压
        let voltage = (gain_corrected * self.vref) / 4095.0;
        
        // 温度补偿
        let temp_error = (temperature - 25.0) * self.temp_coeff;
        voltage * (1.0 - temp_error)
    }
}

// 偏移校准
fn calibrate_offset(
    adc: &mut Adc<ADC1>, 
    pin: &mut Pin<'A', 0, Analog>
) -> Result<i16, AdcError> {
    // 将输入接地进行偏移校准
    // 在实际应用中，需要硬件支持或已知零点
    
    let samples = 100;
    let zero_reading = averaged_sample(adc, pin, samples)?;
    
    // 理想零点应该是0，实际读数就是偏移
    Ok(zero_reading as i16)
}

// 增益校准
fn calibrate_gain(
    adc: &mut Adc<ADC1>, 
    pin: &mut Pin<'A', 0, Analog>,
    reference_voltage: f32,
    vref: f32
) -> Result<f32, AdcError> {
    // 输入已知精确电压进行增益校准
    
    let samples = 100;
    let measured_raw = averaged_sample(adc, pin, samples)?;
    
    // 计算理论ADC值
    let expected_adc = voltage_to_adc(reference_voltage, vref);
    
    // 计算增益系数
    let gain = expected_adc as f32 / measured_raw as f32;
    
    Ok(gain)
}
```

## 数字滤波

### 移动平均滤波器

```rust
// 移动平均滤波器
pub struct MovingAverageFilter {
    buffer: heapless::Vec<u16, 32>,
    index: usize,
    sum: u32,
    initialized: bool,
}

impl MovingAverageFilter {
    pub fn new(size: usize) -> Self {
        Self {
            buffer: heapless::Vec::new(),
            index: 0,
            sum: 0,
            initialized: false,
        }
    }
    
    pub fn update(&mut self, value: u16) -> u16 {
        if !self.initialized {
            // 初始化阶段，填充缓冲区
            if self.buffer.push(value).is_ok() {
                self.sum += value as u32;
                if self.buffer.len() == self.buffer.capacity() {
                    self.initialized = true;
                }
                return value;
            }
        }
        
        // 更新循环缓冲区
        let old_value = self.buffer[self.index];
        self.buffer[self.index] = value;
        
        self.sum = self.sum - old_value as u32 + value as u32;
        self.index = (self.index + 1) % self.buffer.len();
        
        (self.sum / self.buffer.len() as u32) as u16
    }
    
    pub fn get_average(&self) -> u16 {
        if self.buffer.is_empty() {
            0
        } else {
            (self.sum / self.buffer.len() as u32) as u16
        }
    }
}
```

### 指数移动平均滤波器

```rust
// 指数移动平均滤波器（EMA）
pub struct ExponentialMovingAverage {
    alpha: f32,
    value: f32,
    initialized: bool,
}

impl ExponentialMovingAverage {
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            value: 0.0,
            initialized: false,
        }
    }
    
    pub fn update(&mut self, new_value: u16) -> u16 {
        let new_value_f = new_value as f32;
        
        if !self.initialized {
            self.value = new_value_f;
            self.initialized = true;
        } else {
            self.value = self.alpha * new_value_f + (1.0 - self.alpha) * self.value;
        }
        
        self.value as u16
    }
    
    pub fn get_value(&self) -> u16 {
        self.value as u16
    }
}
```

### 中值滤波器

```rust
// 中值滤波器
pub struct MedianFilter {
    buffer: heapless::Vec<u16, 16>,
    index: usize,
}

impl MedianFilter {
    pub fn new(size: usize) -> Self {
        Self {
            buffer: heapless::Vec::new(),
            index: 0,
        }
    }
    
    pub fn update(&mut self, value: u16) -> u16 {
        if self.buffer.len() < self.buffer.capacity() {
            let _ = self.buffer.push(value);
        } else {
            self.buffer[self.index] = value;
            self.index = (self.index + 1) % self.buffer.len();
        }
        
        self.get_median()
    }
    
    fn get_median(&self) -> u16 {
        if self.buffer.is_empty() {
            return 0;
        }
        
        let mut sorted = self.buffer.clone();
        sorted.sort_unstable();
        
        let len = sorted.len();
        if len % 2 == 0 {
            (sorted[len / 2 - 1] + sorted[len / 2]) / 2
        } else {
            sorted[len / 2]
        }
    }
}
```

## 异常检测和处理

### 异常值检测

```rust
// 异常值检测器
pub struct OutlierDetector {
    mean: f32,
    variance: f32,
    sample_count: u32,
    threshold_sigma: f32,
}

impl OutlierDetector {
    pub fn new(threshold_sigma: f32) -> Self {
        Self {
            mean: 0.0,
            variance: 0.0,
            sample_count: 0,
            threshold_sigma,
        }
    }
    
    pub fn is_outlier(&mut self, value: u16) -> bool {
        let value_f = value as f32;
        
        if self.sample_count < 10 {
            // 初始阶段，收集统计信息
            self.update_statistics(value_f);
            return false;
        }
        
        let std_dev = self.variance.sqrt();
        let deviation = (value_f - self.mean).abs();
        
        if deviation > self.threshold_sigma * std_dev {
            return true; // 检测到异常值
        }
        
        self.update_statistics(value_f);
        false
    }
    
    fn update_statistics(&mut self, value: f32) {
        self.sample_count += 1;
        let delta = value - self.mean;
        self.mean += delta / self.sample_count as f32;
        let delta2 = value - self.mean;
        self.variance += delta * delta2;
        
        if self.sample_count > 1 {
            self.variance /= (self.sample_count - 1) as f32;
        }
    }
}
```

### 范围检查

```rust
// 范围检查器
pub struct RangeChecker {
    min_value: u16,
    max_value: u16,
    error_count: u32,
    max_errors: u32,
}

impl RangeChecker {
    pub fn new(min_value: u16, max_value: u16, max_errors: u32) -> Self {
        Self {
            min_value,
            max_value,
            error_count: 0,
            max_errors,
        }
    }
    
    pub fn check(&mut self, value: u16) -> Result<u16, RangeError> {
        if value < self.min_value || value > self.max_value {
            self.error_count += 1;
            
            if self.error_count > self.max_errors {
                return Err(RangeError::TooManyErrors);
            }
            
            return Err(RangeError::OutOfRange { value, min: self.min_value, max: self.max_value });
        }
        
        self.error_count = 0; // 重置错误计数
        Ok(value)
    }
}

#[derive(Debug)]
pub enum RangeError {
    OutOfRange { value: u16, min: u16, max: u16 },
    TooManyErrors,
}
```

## 实际应用示例

### 电池电压监测

```rust
// 电池电压监测器
pub struct BatteryMonitor {
    adc: Adc<ADC1>,
    pin: Pin<'A', 0, Analog>,
    calibration: AdcCalibration,
    filter: MovingAverageFilter,
    voltage_divider_ratio: f32, // 分压比
}

impl BatteryMonitor {
    pub fn new(
        adc: Adc<ADC1>, 
        pin: Pin<'A', 0, Analog>,
        voltage_divider_ratio: f32
    ) -> Self {
        Self {
            adc,
            pin,
            calibration: AdcCalibration::new(),
            filter: MovingAverageFilter::new(16),
            voltage_divider_ratio,
        }
    }
    
    pub fn read_voltage(&mut self) -> Result<f32, AdcError> {
        // 读取ADC值
        let raw_value = self.adc.convert(&mut self.pin, SampleTime::Cycles_480)?;
        
        // 应用滤波
        let filtered_value = self.filter.update(raw_value);
        
        // 应用校准
        let voltage = self.calibration.apply(filtered_value, 25.0); // 假设25°C
        
        // 考虑分压比
        let actual_voltage = voltage * self.voltage_divider_ratio;
        
        Ok(actual_voltage)
    }
    
    pub fn get_battery_percentage(&mut self) -> Result<f32, AdcError> {
        let voltage = self.read_voltage()?;
        
        // 锂电池电压范围：3.0V - 4.2V
        let min_voltage = 3.0;
        let max_voltage = 4.2;
        
        let percentage = ((voltage - min_voltage) / (max_voltage - min_voltage) * 100.0)
            .clamp(0.0, 100.0);
        
        Ok(percentage)
    }
}
```

### 温度传感器读取

```rust
// LM35温度传感器读取
pub struct TemperatureSensor {
    adc: Adc<ADC1>,
    pin: Pin<'A', 1, Analog>,
    calibration: AdcCalibration,
    filter: ExponentialMovingAverage,
}

impl TemperatureSensor {
    pub fn new(adc: Adc<ADC1>, pin: Pin<'A', 1, Analog>) -> Self {
        Self {
            adc,
            pin,
            calibration: AdcCalibration::new(),
            filter: ExponentialMovingAverage::new(0.1), // 较慢的响应
        }
    }
    
    pub fn read_temperature(&mut self) -> Result<f32, AdcError> {
        // 多次采样求平均
        let raw_value = averaged_sample(&mut self.adc, &mut self.pin, 10)?;
        
        // 应用滤波
        let filtered_value = self.filter.update(raw_value);
        
        // 转换为电压
        let voltage = adc_to_voltage(filtered_value, self.calibration.vref);
        
        // LM35: 10mV/°C, 0°C时输出0V
        let temperature = voltage / 0.01; // 转换为摄氏度
        
        Ok(temperature)
    }
    
    pub fn read_temperature_with_compensation(&mut self) -> Result<f32, AdcError> {
        let raw_temp = self.read_temperature()?;
        
        // 简单的非线性补偿
        let compensated = if raw_temp < 0.0 {
            raw_temp * 1.02 // 低温补偿
        } else if raw_temp > 50.0 {
            raw_temp * 0.98 // 高温补偿
        } else {
            raw_temp
        };
        
        Ok(compensated)
    }
}
```

### 光强检测器

```rust
// 光敏电阻光强检测
pub struct LightSensor {
    adc: Adc<ADC1>,
    pin: Pin<'A', 2, Analog>,
    calibration: AdcCalibration,
    filter: MedianFilter,
    dark_value: u16,    // 暗电流值
    bright_value: u16,  // 强光值
}

impl LightSensor {
    pub fn new(adc: Adc<ADC1>, pin: Pin<'A', 2, Analog>) -> Self {
        Self {
            adc,
            pin,
            calibration: AdcCalibration::new(),
            filter: MedianFilter::new(8),
            dark_value: 100,   // 需要校准
            bright_value: 3900, // 需要校准
        }
    }
    
    pub fn read_light_intensity(&mut self) -> Result<f32, AdcError> {
        // 读取ADC值
        let raw_value = self.adc.convert(&mut self.pin, SampleTime::Cycles_480)?;
        
        // 应用中值滤波去除脉冲干扰
        let filtered_value = self.filter.update(raw_value);
        
        // 转换为光强百分比
        let intensity = if filtered_value <= self.dark_value {
            0.0
        } else if filtered_value >= self.bright_value {
            100.0
        } else {
            let range = self.bright_value - self.dark_value;
            let offset = filtered_value - self.dark_value;
            (offset as f32 / range as f32) * 100.0
        };
        
        Ok(intensity)
    }
    
    pub fn calibrate_dark(&mut self) -> Result<(), AdcError> {
        // 在暗环境中校准暗电流
        let samples = 50;
        self.dark_value = averaged_sample(&mut self.adc, &mut self.pin, samples)?;
        Ok(())
    }
    
    pub fn calibrate_bright(&mut self) -> Result<(), AdcError> {
        // 在强光环境中校准
        let samples = 50;
        self.bright_value = averaged_sample(&mut self.adc, &mut self.pin, samples)?;
        Ok(())
    }
}
```

## 性能优化

### 采样速度优化

```rust
// 快速采样配置
fn configure_fast_sampling(adc: &mut Adc<ADC1>) {
    // 使用最高ADC时钟
    adc.set_clock(Clock::Pclk2_div_2); // 42MHz
    
    // 使用最短采样时间（适用于低阻抗源）
    adc.set_sample_time(SampleTime::Cycles_3);
    
    // 禁用不必要的功能
    adc.set_scan_mode(false);
    adc.set_continuous_mode(false);
}

// 批量快速采样
fn fast_batch_sampling(
    adc: &mut Adc<ADC1>, 
    pin: &mut Pin<'A', 0, Analog>,
    buffer: &mut [u16]
) -> Result<(), AdcError> {
    for sample in buffer.iter_mut() {
        *sample = adc.convert(pin, SampleTime::Cycles_3)?;
    }
    Ok(())
}
```

### 精度优化

```rust
// 高精度采样配置
fn configure_high_precision(adc: &mut Adc<ADC1>) {
    // 使用较低ADC时钟减少噪声
    adc.set_clock(Clock::Pclk2_div_8); // 10.5MHz
    
    // 使用最长采样时间
    adc.set_sample_time(SampleTime::Cycles_480);
}

// 高精度采样with统计
fn precision_sampling_with_stats(
    adc: &mut Adc<ADC1>, 
    pin: &mut Pin<'A', 0, Analog>,
    samples: usize
) -> Result<SamplingStats, AdcError> {
    let mut values = heapless::Vec::<u16, 256>::new();
    
    for _ in 0..samples {
        let value = adc.convert(pin, SampleTime::Cycles_480)?;
        let _ = values.push(value);
        delay_us(100); // 采样间隔
    }
    
    Ok(calculate_stats(&values))
}

#[derive(Debug)]
pub struct SamplingStats {
    pub mean: f32,
    pub std_dev: f32,
    pub min: u16,
    pub max: u16,
    pub range: u16,
}

fn calculate_stats(values: &[u16]) -> SamplingStats {
    let sum: u32 = values.iter().map(|&x| x as u32).sum();
    let mean = sum as f32 / values.len() as f32;
    
    let variance: f32 = values.iter()
        .map(|&x| {
            let diff = x as f32 - mean;
            diff * diff
        })
        .sum::<f32>() / values.len() as f32;
    
    let std_dev = variance.sqrt();
    let min = *values.iter().min().unwrap();
    let max = *values.iter().max().unwrap();
    
    SamplingStats {
        mean,
        std_dev,
        min,
        max,
        range: max - min,
    }
}
```

## 调试和测试

### 采样质量评估

```rust
// ADC采样质量测试
pub struct AdcQualityTest {
    adc: Adc<ADC1>,
    pin: Pin<'A', 0, Analog>,
}

impl AdcQualityTest {
    pub fn test_noise_level(&mut self, samples: usize) -> Result<f32, AdcError> {
        // 输入固定电压，测量噪声水平
        let stats = precision_sampling_with_stats(&mut self.adc, &mut self.pin, samples)?;
        
        // 噪声水平用标准差表示
        Ok(stats.std_dev)
    }
    
    pub fn test_linearity(&mut self, test_voltages: &[f32]) -> Result<Vec<f32>, AdcError> {
        let mut linearity_errors = Vec::new();
        
        for &voltage in test_voltages {
            // 需要外部可调电压源
            println!("Set input voltage to {:.3}V and press enter", voltage);
            wait_for_input();
            
            let measured_adc = averaged_sample(&mut self.adc, &mut self.pin, 100)?;
            let measured_voltage = adc_to_voltage(measured_adc, 3.3);
            let error = measured_voltage - voltage;
            
            linearity_errors.push(error);
        }
        
        Ok(linearity_errors)
    }
    
    pub fn test_repeatability(&mut self, iterations: usize) -> Result<f32, AdcError> {
        let mut measurements = Vec::new();
        
        for _ in 0..iterations {
            let value = averaged_sample(&mut self.adc, &mut self.pin, 10)?;
            measurements.push(value);
            delay_ms(100);
        }
        
        let stats = calculate_stats(&measurements);
        Ok(stats.std_dev / stats.mean * 100.0) // 相对标准差百分比
    }
}
```

### 实时监测

```rust
// 实时ADC监测
pub struct AdcMonitor {
    adc: Adc<ADC1>,
    pin: Pin<'A', 0, Analog>,
    last_values: heapless::Vec<u16, 100>,
    alert_thresholds: (u16, u16), // (min, max)
}

impl AdcMonitor {
    pub fn new(adc: Adc<ADC1>, pin: Pin<'A', 0, Analog>) -> Self {
        Self {
            adc,
            pin,
            last_values: heapless::Vec::new(),
            alert_thresholds: (100, 3900),
        }
    }
    
    pub fn update(&mut self) -> Result<MonitorResult, AdcError> {
        let value = self.adc.convert(&mut self.pin, SampleTime::Cycles_480)?;
        
        // 更新历史记录
        if self.last_values.len() >= self.last_values.capacity() {
            self.last_values.remove(0);
        }
        let _ = self.last_values.push(value);
        
        // 检查告警条件
        let mut alerts = Vec::new();
        
        if value < self.alert_thresholds.0 {
            alerts.push(Alert::UnderRange(value));
        }
        if value > self.alert_thresholds.1 {
            alerts.push(Alert::OverRange(value));
        }
        
        // 检查变化率
        if self.last_values.len() >= 2 {
            let prev_value = self.last_values[self.last_values.len() - 2];
            let change_rate = (value as i32 - prev_value as i32).abs();
            if change_rate > 500 { // 快速变化阈值
                alerts.push(Alert::RapidChange { from: prev_value, to: value });
            }
        }
        
        Ok(MonitorResult {
            current_value: value,
            voltage: adc_to_voltage(value, 3.3),
            alerts,
            history_size: self.last_values.len(),
        })
    }
}

#[derive(Debug)]
pub struct MonitorResult {
    pub current_value: u16,
    pub voltage: f32,
    pub alerts: Vec<Alert>,
    pub history_size: usize,
}

#[derive(Debug)]
pub enum Alert {
    UnderRange(u16),
    OverRange(u16),
    RapidChange { from: u16, to: u16 },
}
```

## 最佳实践

### 硬件设计建议

1. **输入保护**：
   ```rust
   // 软件输入范围检查
   fn validate_input_range(adc_value: u16) -> Result<u16, InputError> {
       if adc_value > 4000 { // 接近满量程
           Err(InputError::NearOverload)
       } else if adc_value < 50 { // 接近零点
           Err(InputError::NearZero)
       } else {
           Ok(adc_value)
       }
   }
   ```

2. **信号调理**：
   - 使用运放缓冲器降低源阻抗
   - 添加抗混叠滤波器
   - 考虑信号范围匹配

3. **PCB布局**：
   - 模拟信号远离数字信号
   - 使用地平面
   - 最短信号路径

### 软件设计建议

1. **错误处理**：
   ```rust
   // 完整的错误处理
   fn robust_adc_reading(adc: &mut Adc<ADC1>, pin: &mut Pin<'A', 0, Analog>) -> Result<u16, AdcError> {
       const MAX_RETRIES: usize = 3;
       
       for attempt in 0..MAX_RETRIES {
           match adc.convert(pin, SampleTime::Cycles_480) {
               Ok(value) => {
                   // 验证读数合理性
                   if value > 10 && value < 4090 {
                       return Ok(value);
                   }
               }
               Err(e) => {
                   if attempt == MAX_RETRIES - 1 {
                       return Err(e);
                   }
                   delay_ms(1); // 短暂延迟后重试
               }
           }
       }
       
       Err(AdcError::MaxRetriesExceeded)
   }
   ```

2. **性能监测**：
   ```rust
   // 性能统计
   pub struct PerformanceStats {
       pub total_samples: u32,
       pub error_count: u32,
       pub avg_sample_time_us: f32,
       pub max_sample_time_us: u32,
   }
   
   impl PerformanceStats {
       pub fn error_rate(&self) -> f32 {
           if self.total_samples > 0 {
               self.error_count as f32 / self.total_samples as f32 * 100.0
           } else {
               0.0
           }
       }
   }
   ```

## 总结

单通道ADC采样是嵌入式系统中最基础也是最重要的功能之一。通过合理的硬件配置、软件实现和优化技术，可以实现高质量的模拟信号采集。

关键要点：
1. 根据应用需求选择合适的采样时间和精度
2. 实施有效的校准和滤波技术
3. 添加异常检测和错误处理机制
4. 平衡采样速度和精度要求
5. 进行充分的测试和验证

通过本章的学习，您应该能够：
- 配置和使用STM32F4的ADC进行单通道采样
- 实现各种滤波和校准算法
- 处理实际应用中的各种问题
- 优化采样性能和精度

## 参考资料

1. STM32F4 Reference Manual - ADC章节
2. "ADC Noise Analysis and Reduction" - Application Note
3. "Improving ADC Resolution by Oversampling" - AN2668
4. "STM32F4 ADC Calibration" - Application Note