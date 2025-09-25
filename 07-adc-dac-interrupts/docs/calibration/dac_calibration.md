# DAC校准技术文档

## 概述

本文档详细介绍了STM32F4系列微控制器DAC的校准方法和技术，确保输出精度和波形质量。

## DAC校准原理

### 1. DAC特性分析

#### 1.1 主要误差源
- **偏移误差**: 零码输出时的非零电压
- **增益误差**: 满量程输出与理想值的偏差
- **积分非线性(INL)**: 实际传输特性与理想直线的偏差
- **微分非线性(DNL)**: 相邻码值间电压差的变化
- **温度漂移**: 温度变化引起的输出变化

#### 1.2 校准目标
- 消除偏移和增益误差
- 改善线性度
- 补偿温度漂移
- 提高输出精度和稳定性

### 2. 校准类型

#### 2.1 静态校准
- **零点校准**: 校准DAC码值0时的输出电压
- **满量程校准**: 校准DAC最大码值时的输出电压
- **多点线性校准**: 使用多个校准点改善线性度

#### 2.2 动态校准
- **频率响应校准**: 校准不同频率下的幅度和相位响应
- **谐波失真校准**: 减少输出信号的谐波失真
- **噪声校准**: 优化输出信号的信噪比

## 实现方法

### 1. 硬件要求

#### 1.1 测量设备
- **高精度数字万用表**: 6.5位或更高精度
- **示波器**: 带宽≥100MHz，垂直分辨率≥8位
- **频谱分析仪**: 用于谐波和噪声分析
- **精密负载**: 阻抗匹配的负载电阻

#### 1.2 参考标准
- **电压参考**: 精度±0.01%，温度系数<10ppm/°C
- **频率参考**: 晶振精度±1ppm
- **温度传感器**: 精度±0.1°C

### 2. 软件实现

#### 2.1 校准数据结构

```rust
#[derive(Clone, Copy)]
pub struct DacCalibrationData {
    pub offset_correction: i16,      // 偏移校正值
    pub gain_correction: f32,        // 增益校正系数
    pub linearity_table: [f32; 32],  // 线性度校正表
    pub frequency_response: [f32; 16], // 频率响应校正
    pub temperature_coeff: f32,       // 温度系数
    pub thd_correction: [f32; 8],    // 谐波失真校正
    pub noise_floor: f32,            // 噪声底限
    pub settling_time: u16,          // 建立时间(μs)
    pub timestamp: u32,              // 校准时间戳
    pub checksum: u16,               // 数据校验和
}

impl Default for DacCalibrationData {
    fn default() -> Self {
        Self {
            offset_correction: 0,
            gain_correction: 1.0,
            linearity_table: [1.0; 32],
            frequency_response: [1.0; 16],
            temperature_coeff: 0.0,
            thd_correction: [0.0; 8],
            noise_floor: -90.0, // dB
            settling_time: 10,   // μs
            timestamp: 0,
            checksum: 0,
        }
    }
}
```

#### 2.2 DAC校准器实现

```rust
pub struct DacCalibrator {
    calibration_data: DacCalibrationData,
    reference_voltage: f32,
    temperature: f32,
    dac_resolution: u8,
}

impl DacCalibrator {
    pub fn new(reference_voltage: f32, resolution: u8) -> Self {
        Self {
            calibration_data: DacCalibrationData::default(),
            reference_voltage,
            temperature: 25.0,
            dac_resolution: resolution,
        }
    }

    // 偏移校准
    pub fn calibrate_offset(&mut self, dac: &mut Dac<DAC1>, 
                           voltmeter: &mut Voltmeter) -> Result<(), CalibrationError> {
        // 设置DAC输出为0
        dac.set_value(0);
        delay_ms(100); // 等待稳定
        
        // 测量实际输出电压
        let measured_voltage = voltmeter.read_voltage()?;
        
        // 计算偏移校正值
        let expected_voltage = 0.0;
        let error_voltage = measured_voltage - expected_voltage;
        let max_dac_value = (1 << self.dac_resolution) - 1;
        
        self.calibration_data.offset_correction = 
            ((error_voltage / self.reference_voltage) * max_dac_value as f32) as i16;
        
        println!("偏移校准完成: {}mV -> 校正值: {}", 
                measured_voltage * 1000.0, 
                self.calibration_data.offset_correction);
        
        Ok(())
    }

    // 增益校准
    pub fn calibrate_gain(&mut self, dac: &mut Dac<DAC1>, 
                         voltmeter: &mut Voltmeter) -> Result<(), CalibrationError> {
        let max_dac_value = (1 << self.dac_resolution) - 1;
        
        // 设置DAC为满量程输出
        dac.set_value(max_dac_value);
        delay_ms(100);
        
        // 测量实际输出电压
        let measured_voltage = voltmeter.read_voltage()?;
        
        // 计算增益校正系数
        let expected_voltage = self.reference_voltage;
        self.calibration_data.gain_correction = expected_voltage / measured_voltage;
        
        println!("增益校准完成: {:.3}V -> 校正系数: {:.6}", 
                measured_voltage, 
                self.calibration_data.gain_correction);
        
        Ok(())
    }

    // 线性度校准
    pub fn calibrate_linearity(&mut self, dac: &mut Dac<DAC1>, 
                              voltmeter: &mut Voltmeter) -> Result<(), CalibrationError> {
        let max_dac_value = (1 << self.dac_resolution) - 1;
        let segments = self.calibration_data.linearity_table.len();
        
        for i in 0..segments {
            let dac_value = (i * max_dac_value / (segments - 1)) as u16;
            
            // 应用已有的偏移和增益校正
            let corrected_value = self.apply_offset_gain_correction(dac_value);
            dac.set_value(corrected_value);
            delay_ms(50);
            
            // 测量输出电压
            let measured_voltage = voltmeter.read_voltage()?;
            
            // 计算期望电压
            let expected_voltage = (dac_value as f32 / max_dac_value as f32) * self.reference_voltage;
            
            // 计算线性度校正系数
            if measured_voltage > 0.001 { // 避免除零
                self.calibration_data.linearity_table[i] = expected_voltage / measured_voltage;
            }
            
            println!("线性度校准点{}: DAC={}, 测量={:.3}V, 期望={:.3}V, 系数={:.6}", 
                    i, dac_value, measured_voltage, expected_voltage, 
                    self.calibration_data.linearity_table[i]);
        }
        
        Ok(())
    }

    // 频率响应校准
    pub fn calibrate_frequency_response(&mut self, dac: &mut Dac<DAC1>, 
                                       oscilloscope: &mut Oscilloscope) -> Result<(), CalibrationError> {
        let test_frequencies = [
            100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 
            10000.0, 20000.0, 50000.0, 100000.0
        ];
        
        let amplitude = (1 << (self.dac_resolution - 2)) as u16; // 25%幅度
        
        for (i, &frequency) in test_frequencies.iter().enumerate() {
            if i >= self.calibration_data.frequency_response.len() {
                break;
            }
            
            // 生成正弦波
            self.generate_sine_wave(dac, frequency, amplitude);
            delay_ms(100); // 等待稳定
            
            // 测量输出幅度
            let measured_amplitude = oscilloscope.measure_amplitude()?;
            let expected_amplitude = self.dac_value_to_voltage(amplitude);
            
            // 计算频率响应校正
            if measured_amplitude > 0.001 {
                self.calibration_data.frequency_response[i] = expected_amplitude / measured_amplitude;
            }
            
            println!("频率响应校准: {}Hz, 测量={:.3}V, 期望={:.3}V, 校正={:.6}", 
                    frequency, measured_amplitude, expected_amplitude, 
                    self.calibration_data.frequency_response[i]);
        }
        
        Ok(())
    }

    // 谐波失真校准
    pub fn calibrate_thd(&mut self, dac: &mut Dac<DAC1>, 
                        spectrum_analyzer: &mut SpectrumAnalyzer) -> Result<(), CalibrationError> {
        let test_frequency = 1000.0; // 1kHz测试信号
        let amplitude = (1 << (self.dac_resolution - 2)) as u16;
        
        // 生成测试信号
        self.generate_sine_wave(dac, test_frequency, amplitude);
        delay_ms(200);
        
        // 测量谐波
        let harmonics = spectrum_analyzer.measure_harmonics(test_frequency, 8)?;
        
        for (i, &harmonic_level) in harmonics.iter().enumerate() {
            if i < self.calibration_data.thd_correction.len() {
                // 计算谐波抑制校正
                self.calibration_data.thd_correction[i] = -harmonic_level; // dB
                
                println!("谐波{}校准: {:.1}dB -> 校正: {:.1}dB", 
                        i + 2, harmonic_level, self.calibration_data.thd_correction[i]);
            }
        }
        
        Ok(())
    }

    // 应用校准
    pub fn apply_calibration(&self, dac_value: u16) -> u16 {
        // 1. 偏移校正
        let offset_corrected = (dac_value as i32 - self.calibration_data.offset_correction as i32)
            .max(0) as u16;
        
        // 2. 增益校正
        let gain_corrected = (offset_corrected as f32 * self.calibration_data.gain_correction) as u16;
        
        // 3. 线性度校正
        let max_value = (1 << self.dac_resolution) - 1;
        let segment = (gain_corrected * self.calibration_data.linearity_table.len() as u16 / max_value)
            .min(self.calibration_data.linearity_table.len() as u16 - 1) as usize;
        
        let linearity_corrected = (gain_corrected as f32 * 
            self.calibration_data.linearity_table[segment]) as u16;
        
        // 4. 温度补偿
        let temp_correction = 1.0 + self.calibration_data.temperature_coeff * (self.temperature - 25.0);
        let final_value = (linearity_corrected as f32 * temp_correction) as u16;
        
        final_value.min(max_value)
    }

    fn apply_offset_gain_correction(&self, dac_value: u16) -> u16 {
        let offset_corrected = (dac_value as i32 - self.calibration_data.offset_correction as i32)
            .max(0) as u16;
        (offset_corrected as f32 * self.calibration_data.gain_correction) as u16
    }

    fn dac_value_to_voltage(&self, dac_value: u16) -> f32 {
        let max_value = (1 << self.dac_resolution) - 1;
        (dac_value as f32 / max_value as f32) * self.reference_voltage
    }

    fn generate_sine_wave(&self, dac: &mut Dac<DAC1>, frequency: f32, amplitude: u16) {
        // 简化的正弦波生成（实际应用中需要更复杂的实现）
        let samples_per_cycle = 100;
        let sample_rate = frequency * samples_per_cycle as f32;
        
        for i in 0..samples_per_cycle {
            let phase = 2.0 * core::f32::consts::PI * i as f32 / samples_per_cycle as f32;
            let sine_value = phase.sin();
            let dac_value = ((sine_value + 1.0) * amplitude as f32 / 2.0) as u16;
            
            let calibrated_value = self.apply_calibration(dac_value);
            dac.set_value(calibrated_value);
            
            // 延时以控制采样率
            delay_us((1_000_000.0 / sample_rate) as u32);
        }
    }
}
```

### 3. 校准验证

#### 3.1 静态精度验证
```rust
pub fn verify_static_accuracy(&self, test_points: &[u16]) -> StaticAccuracyReport {
    let mut errors = Vec::new();
    let mut max_error = 0.0f32;
    
    for &dac_value in test_points {
        let calibrated_value = self.apply_calibration(dac_value);
        let expected_voltage = self.dac_value_to_voltage(dac_value);
        
        // 这里需要实际测量输出电压
        let measured_voltage = measure_dac_output(calibrated_value);
        
        let error = (measured_voltage - expected_voltage).abs();
        let error_percent = (error / expected_voltage) * 100.0;
        
        errors.push(error_percent);
        max_error = max_error.max(error_percent);
    }
    
    let rms_error = (errors.iter().map(|e| e * e).sum::<f32>() / errors.len() as f32).sqrt();
    
    StaticAccuracyReport {
        max_error_percent: max_error,
        rms_error_percent: rms_error,
        linearity_error: self.calculate_inl(&errors),
        offset_error: errors[0], // 零点误差
        gain_error: errors[errors.len() - 1], // 满量程误差
        passed: max_error < 0.1, // 0.1%精度要求
    }
}
```

#### 3.2 动态性能验证
```rust
pub fn verify_dynamic_performance(&self, test_frequencies: &[f32]) -> DynamicPerformanceReport {
    let mut frequency_errors = Vec::new();
    let mut thd_values = Vec::new();
    let mut snr_values = Vec::new();
    
    for &frequency in test_frequencies {
        // 生成测试信号并测量
        let amplitude_error = self.measure_amplitude_error(frequency);
        let thd = self.measure_thd(frequency);
        let snr = self.measure_snr(frequency);
        
        frequency_errors.push(amplitude_error);
        thd_values.push(thd);
        snr_values.push(snr);
    }
    
    DynamicPerformanceReport {
        max_amplitude_error: frequency_errors.iter().fold(0.0f32, |a, &b| a.max(b)),
        average_thd: thd_values.iter().sum::<f32>() / thd_values.len() as f32,
        min_snr: snr_values.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
        bandwidth_3db: self.calculate_3db_bandwidth(),
        settling_time: self.calibration_data.settling_time,
        passed: self.check_dynamic_specs(&frequency_errors, &thd_values, &snr_values),
    }
}
```

## 校准程序

### 1. 完整校准流程

```rust
pub fn full_calibration_sequence() -> Result<DacCalibrationData, CalibrationError> {
    let mut calibrator = DacCalibrator::new(3.3, 12); // 3.3V, 12位DAC
    let mut dac = setup_dac();
    let mut voltmeter = setup_voltmeter();
    let mut oscilloscope = setup_oscilloscope();
    let mut spectrum_analyzer = setup_spectrum_analyzer();
    
    println!("开始DAC完整校准流程...");
    
    // 1. 系统预热
    println!("系统预热中...");
    delay_ms(10000); // 10秒预热
    
    // 2. 偏移校准
    println!("执行偏移校准...");
    calibrator.calibrate_offset(&mut dac, &mut voltmeter)?;
    
    // 3. 增益校准
    println!("执行增益校准...");
    calibrator.calibrate_gain(&mut dac, &mut voltmeter)?;
    
    // 4. 线性度校准
    println!("执行线性度校准...");
    calibrator.calibrate_linearity(&mut dac, &mut voltmeter)?;
    
    // 5. 频率响应校准
    println!("执行频率响应校准...");
    calibrator.calibrate_frequency_response(&mut dac, &mut oscilloscope)?;
    
    // 6. 谐波失真校准
    println!("执行谐波失真校准...");
    calibrator.calibrate_thd(&mut dac, &mut spectrum_analyzer)?;
    
    // 7. 温度系数测量
    println!("测量温度系数...");
    calibrator.measure_temperature_coefficient(&mut dac, &mut voltmeter)?;
    
    // 8. 验证校准结果
    println!("验证校准结果...");
    let static_report = calibrator.verify_static_accuracy(&[0, 1024, 2048, 3072, 4095]);
    let dynamic_report = calibrator.verify_dynamic_performance(&[100.0, 1000.0, 10000.0]);
    
    if static_report.passed && dynamic_report.passed {
        println!("校准成功完成！");
        println!("静态精度: {:.3}%", static_report.max_error_percent);
        println!("动态THD: {:.1}dB", dynamic_report.average_thd);
        println!("信噪比: {:.1}dB", dynamic_report.min_snr);
        
        Ok(calibrator.calibration_data)
    } else {
        Err(CalibrationError::VerificationFailed)
    }
}
```

### 2. 快速校准程序

```rust
pub fn quick_calibration() -> Result<DacCalibrationData, CalibrationError> {
    let mut calibrator = DacCalibrator::new(3.3, 12);
    let mut dac = setup_dac();
    let mut voltmeter = setup_voltmeter();
    
    println!("执行快速校准...");
    
    // 仅执行偏移和增益校准
    calibrator.calibrate_offset(&mut dac, &mut voltmeter)?;
    calibrator.calibrate_gain(&mut dac, &mut voltmeter)?;
    
    // 简单验证
    let test_points = [0, 2048, 4095]; // 0%, 50%, 100%
    let report = calibrator.verify_static_accuracy(&test_points);
    
    if report.max_error_percent < 0.5 { // 0.5%精度要求
        println!("快速校准完成，精度: {:.3}%", report.max_error_percent);
        Ok(calibrator.calibration_data)
    } else {
        Err(CalibrationError::AccuracyNotMet)
    }
}
```

## 校准数据管理

### 1. 数据存储格式

```rust
// Flash存储结构
#[repr(C, packed)]
struct DacCalibrationFlashData {
    header: CalibrationHeader,
    data: DacCalibrationData,
    crc32: u32,
}

struct CalibrationHeader {
    magic: u32,           // 0xDAC12345
    version: u16,         // 校准数据版本
    length: u16,          // 数据长度
    device_id: u32,       // 设备ID
    calibration_date: u32, // 校准日期
}
```

### 2. 数据备份和恢复

```rust
pub fn backup_calibration_data(data: &DacCalibrationData) -> Result<(), StorageError> {
    // 主存储区
    save_to_flash(CALIBRATION_PRIMARY_ADDR, data)?;
    
    // 备份存储区
    save_to_flash(CALIBRATION_BACKUP_ADDR, data)?;
    
    // 外部EEPROM备份
    save_to_eeprom(CALIBRATION_EEPROM_ADDR, data)?;
    
    Ok(())
}

pub fn restore_calibration_data() -> Result<DacCalibrationData, StorageError> {
    // 尝试从主存储区加载
    if let Ok(data) = load_from_flash(CALIBRATION_PRIMARY_ADDR) {
        if verify_calibration_data(&data) {
            return Ok(data);
        }
    }
    
    // 尝试从备份存储区加载
    if let Ok(data) = load_from_flash(CALIBRATION_BACKUP_ADDR) {
        if verify_calibration_data(&data) {
            // 恢复主存储区
            save_to_flash(CALIBRATION_PRIMARY_ADDR, &data)?;
            return Ok(data);
        }
    }
    
    // 尝试从EEPROM加载
    if let Ok(data) = load_from_eeprom(CALIBRATION_EEPROM_ADDR) {
        if verify_calibration_data(&data) {
            // 恢复Flash存储
            backup_calibration_data(&data)?;
            return Ok(data);
        }
    }
    
    // 所有存储都失败，使用默认值
    Err(StorageError::AllBackupsFailed)
}
```

## 性能指标

### 1. 校准精度目标

| 参数 | 目标值 | 测试条件 |
|------|--------|----------|
| 偏移误差 | ±0.05% FS | 25°C |
| 增益误差 | ±0.1% FS | 25°C |
| 积分非线性 | ±0.5 LSB | 全量程 |
| 微分非线性 | ±0.25 LSB | 全量程 |
| 温度漂移 | ±50 ppm/°C | -40°C~+85°C |
| THD | <-60dB | 1kHz, -1dBFS |
| SNR | >70dB | 1kHz, -1dBFS |

### 2. 校准时间预算

```rust
pub struct CalibrationTimeBudget {
    pub offset_calibration: u32,    // 30秒
    pub gain_calibration: u32,      // 30秒
    pub linearity_calibration: u32, // 5分钟
    pub frequency_response: u32,    // 3分钟
    pub thd_calibration: u32,       // 2分钟
    pub verification: u32,          // 2分钟
    pub total_time: u32,           // 约13分钟
}

impl Default for CalibrationTimeBudget {
    fn default() -> Self {
        Self {
            offset_calibration: 30,
            gain_calibration: 30,
            linearity_calibration: 300,
            frequency_response: 180,
            thd_calibration: 120,
            verification: 120,
            total_time: 780, // 13分钟
        }
    }
}
```

## 故障排除

### 1. 常见问题及解决方案

#### 1.1 校准不收敛
- **症状**: 多次校准结果差异很大
- **原因**: 温度不稳定、电源噪声、负载变化
- **解决**: 延长预热时间、改善电源滤波、使用恒定负载

#### 1.2 线性度差
- **症状**: INL/DNL超出规格
- **原因**: DAC内部特性、参考电压不稳定
- **解决**: 增加校准点数、使用高精度参考、分段校准

#### 1.3 温度漂移大
- **症状**: 温度变化时输出偏差大
- **原因**: 温度补偿不足、器件特性
- **解决**: 重新测量温度系数、使用温度传感器实时补偿

### 2. 调试工具

```rust
pub fn calibration_diagnostics(calibrator: &DacCalibrator) {
    println!("=== DAC校准诊断信息 ===");
    
    // 基本参数
    println!("参考电压: {:.3}V", calibrator.reference_voltage);
    println!("DAC分辨率: {}位", calibrator.dac_resolution);
    println!("当前温度: {:.1}°C", calibrator.temperature);
    
    // 校准参数
    println!("\n校准参数:");
    println!("偏移校正: {}", calibrator.calibration_data.offset_correction);
    println!("增益校正: {:.6}", calibrator.calibration_data.gain_correction);
    println!("温度系数: {:.2} ppm/°C", calibrator.calibration_data.temperature_coeff);
    
    // 线性度表
    println!("\n线性度校正表:");
    for (i, &coeff) in calibrator.calibration_data.linearity_table.iter().enumerate() {
        if i % 8 == 0 { println!(); }
        print!("{:.4} ", coeff);
    }
    println!();
    
    // 频率响应
    println!("\n频率响应校正:");
    for (i, &coeff) in calibrator.calibration_data.frequency_response.iter().enumerate() {
        println!("  {}Hz: {:.4}", 100 * (1 << i), coeff);
    }
    
    // 谐波失真校正
    println!("\n谐波失真校正:");
    for (i, &correction) in calibrator.calibration_data.thd_correction.iter().enumerate() {
        println!("  H{}: {:.1}dB", i + 2, correction);
    }
}
```

## 最佳实践

1. **环境控制**: 在温度稳定的环境中进行校准
2. **设备预热**: 充分的预热时间确保稳定性
3. **负载一致**: 校准和使用时保持相同的负载条件
4. **多次测量**: 每个校准点进行多次测量取平均
5. **分段校准**: 对于高精度要求，采用分段校准方法
6. **定期验证**: 定期验证校准有效性
7. **数据备份**: 多重备份校准数据
8. **版本管理**: 记录校准数据版本和变更历史

## 总结

DAC校准是确保输出精度和信号质量的关键环节。通过系统的校准流程、精确的测量设备和合适的校准算法，可以显著提高DAC的性能指标。在实际应用中，应根据具体的精度要求和应用场景选择合适的校准方法和验证标准。