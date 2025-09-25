# ADC校准技术文档

## 概述

本文档详细介绍了STM32F4系列微控制器ADC的校准方法和技术，确保测量精度和系统可靠性。

## ADC校准原理

### 1. 校准类型

#### 1.1 偏移校准（Offset Calibration）
- **目的**: 消除ADC的零点偏移误差
- **方法**: 在输入为0V时测量ADC输出，记录偏移值
- **公式**: `校准值 = 测量值 - 偏移值`

#### 1.2 增益校准（Gain Calibration）
- **目的**: 修正ADC的满量程误差
- **方法**: 使用已知参考电压进行校准
- **公式**: `实际电压 = (ADC值 × 参考电压) / (满量程值 × 增益系数)`

#### 1.3 线性度校准（Linearity Calibration）
- **目的**: 修正ADC的非线性误差
- **方法**: 多点校准，建立校准表或拟合曲线

### 2. 校准流程

```
开始校准
    ↓
设置ADC参数
    ↓
偏移校准
    ↓
增益校准
    ↓
线性度校准
    ↓
验证校准结果
    ↓
保存校准参数
    ↓
校准完成
```

## 实现方法

### 1. 硬件要求

#### 1.1 参考电压源
- **精度**: ±0.1% 或更高
- **稳定性**: 温度系数 < 50ppm/°C
- **噪声**: < 10μVrms

#### 1.2 校准设备
- 精密电压源
- 数字万用表（6.5位或更高）
- 温度传感器
- 屏蔽盒（减少干扰）

### 2. 软件实现

#### 2.1 校准数据结构

```rust
#[derive(Clone, Copy)]
pub struct CalibrationData {
    pub offset: i16,           // 偏移校准值
    pub gain: f32,             // 增益校准系数
    pub linearity: [f32; 16],  // 线性度校准表
    pub temperature_coeff: f32, // 温度系数
    pub timestamp: u32,        // 校准时间戳
    pub checksum: u16,         // 校验和
}

impl Default for CalibrationData {
    fn default() -> Self {
        Self {
            offset: 0,
            gain: 1.0,
            linearity: [1.0; 16],
            temperature_coeff: 0.0,
            timestamp: 0,
            checksum: 0,
        }
    }
}
```

#### 2.2 校准函数实现

```rust
pub struct AdcCalibrator {
    calibration_data: CalibrationData,
    reference_voltage: f32,
    temperature: f32,
}

impl AdcCalibrator {
    pub fn new(reference_voltage: f32) -> Self {
        Self {
            calibration_data: CalibrationData::default(),
            reference_voltage,
            temperature: 25.0,
        }
    }

    // 偏移校准
    pub fn calibrate_offset(&mut self, adc: &mut Adc<ADC1>) -> Result<(), CalibrationError> {
        // 1. 设置输入为0V（短接到地）
        // 2. 多次采样求平均
        let mut sum = 0i32;
        const SAMPLES: usize = 1000;
        
        for _ in 0..SAMPLES {
            let sample = adc.read_raw();
            sum += sample as i32;
        }
        
        let average = sum / SAMPLES as i32;
        self.calibration_data.offset = average as i16;
        
        Ok(())
    }

    // 增益校准
    pub fn calibrate_gain(&mut self, adc: &mut Adc<ADC1>, 
                         reference_voltage: f32) -> Result<(), CalibrationError> {
        // 1. 输入已知参考电压
        // 2. 测量ADC输出
        let mut sum = 0u32;
        const SAMPLES: usize = 1000;
        
        for _ in 0..SAMPLES {
            let sample = adc.read_raw();
            sum += sample as u32;
        }
        
        let average = sum / SAMPLES as u32;
        let corrected_value = (average as i32 - self.calibration_data.offset as i32) as u32;
        
        // 计算增益系数
        let expected_value = (reference_voltage * 4096.0 / self.reference_voltage) as u32;
        self.calibration_data.gain = expected_value as f32 / corrected_value as f32;
        
        Ok(())
    }

    // 线性度校准
    pub fn calibrate_linearity(&mut self, adc: &mut Adc<ADC1>, 
                              test_voltages: &[f32]) -> Result<(), CalibrationError> {
        for (i, &voltage) in test_voltages.iter().enumerate() {
            if i >= self.calibration_data.linearity.len() {
                break;
            }
            
            // 设置测试电压
            // 测量ADC输出
            let mut sum = 0u32;
            const SAMPLES: usize = 100;
            
            for _ in 0..SAMPLES {
                let sample = adc.read_raw();
                sum += sample as u32;
            }
            
            let average = sum / SAMPLES as u32;
            let corrected_value = self.apply_offset_gain_correction(average);
            
            // 计算线性度系数
            let expected_value = (voltage * 4096.0 / self.reference_voltage) as u32;
            self.calibration_data.linearity[i] = expected_value as f32 / corrected_value as f32;
        }
        
        Ok(())
    }

    // 应用校准
    pub fn apply_calibration(&self, raw_value: u16) -> f32 {
        // 1. 偏移校准
        let offset_corrected = (raw_value as i32 - self.calibration_data.offset as i32) as u32;
        
        // 2. 增益校准
        let gain_corrected = offset_corrected as f32 * self.calibration_data.gain;
        
        // 3. 线性度校准
        let segment = (gain_corrected / 256.0) as usize;
        let linearity_factor = if segment < self.calibration_data.linearity.len() {
            self.calibration_data.linearity[segment]
        } else {
            1.0
        };
        
        let linearity_corrected = gain_corrected * linearity_factor;
        
        // 4. 温度补偿
        let temp_correction = 1.0 + self.calibration_data.temperature_coeff * (self.temperature - 25.0);
        let final_value = linearity_corrected * temp_correction;
        
        // 5. 转换为电压
        (final_value * self.reference_voltage) / 4096.0
    }

    fn apply_offset_gain_correction(&self, raw_value: u32) -> u32 {
        let offset_corrected = (raw_value as i32 - self.calibration_data.offset as i32) as u32;
        (offset_corrected as f32 * self.calibration_data.gain) as u32
    }
}
```

### 3. 校准验证

#### 3.1 精度验证
```rust
pub fn verify_calibration(&self, test_points: &[(f32, u16)]) -> CalibrationReport {
    let mut errors = Vec::new();
    let mut max_error = 0.0f32;
    let mut rms_error = 0.0f32;
    
    for &(expected_voltage, raw_adc) in test_points {
        let measured_voltage = self.apply_calibration(raw_adc);
        let error = (measured_voltage - expected_voltage).abs();
        let error_percent = (error / expected_voltage) * 100.0;
        
        errors.push(error_percent);
        max_error = max_error.max(error_percent);
        rms_error += error_percent * error_percent;
    }
    
    rms_error = (rms_error / errors.len() as f32).sqrt();
    
    CalibrationReport {
        max_error_percent: max_error,
        rms_error_percent: rms_error,
        linearity_error: self.calculate_linearity_error(&errors),
        temperature_drift: self.calculate_temperature_drift(),
        passed: max_error < 0.1, // 0.1% 精度要求
    }
}
```

## 校准程序

### 1. 自动校准程序

```rust
pub fn auto_calibration_sequence() -> Result<CalibrationData, CalibrationError> {
    let mut calibrator = AdcCalibrator::new(3.3);
    
    // 1. 系统预热
    delay_ms(5000);
    
    // 2. 偏移校准
    println!("开始偏移校准...");
    calibrator.calibrate_offset(&mut adc)?;
    
    // 3. 增益校准
    println!("开始增益校准...");
    calibrator.calibrate_gain(&mut adc, 3.0)?; // 使用3.0V参考
    
    // 4. 线性度校准
    println!("开始线性度校准...");
    let test_voltages = [0.2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
    calibrator.calibrate_linearity(&mut adc, &test_voltages)?;
    
    // 5. 验证校准结果
    println!("验证校准结果...");
    let verification_points = [(0.5, 620), (1.0, 1241), (2.0, 2482), (3.0, 3723)];
    let report = calibrator.verify_calibration(&verification_points);
    
    if report.passed {
        println!("校准成功！最大误差: {:.3}%", report.max_error_percent);
        Ok(calibrator.calibration_data)
    } else {
        Err(CalibrationError::VerificationFailed)
    }
}
```

### 2. 校准数据存储

```rust
// EEPROM存储
pub fn save_calibration_to_eeprom(data: &CalibrationData) -> Result<(), StorageError> {
    let mut buffer = [0u8; 64];
    
    // 序列化校准数据
    buffer[0..2].copy_from_slice(&data.offset.to_le_bytes());
    buffer[2..6].copy_from_slice(&data.gain.to_le_bytes());
    
    // 存储线性度表
    for (i, &coeff) in data.linearity.iter().enumerate() {
        let start = 6 + i * 4;
        buffer[start..start+4].copy_from_slice(&coeff.to_le_bytes());
    }
    
    // 计算校验和
    let checksum = calculate_checksum(&buffer[0..62]);
    buffer[62..64].copy_from_slice(&checksum.to_le_bytes());
    
    // 写入EEPROM
    eeprom_write(CALIBRATION_ADDRESS, &buffer)
}

// 从EEPROM加载
pub fn load_calibration_from_eeprom() -> Result<CalibrationData, StorageError> {
    let mut buffer = [0u8; 64];
    eeprom_read(CALIBRATION_ADDRESS, &mut buffer)?;
    
    // 验证校验和
    let stored_checksum = u16::from_le_bytes([buffer[62], buffer[63]]);
    let calculated_checksum = calculate_checksum(&buffer[0..62]);
    
    if stored_checksum != calculated_checksum {
        return Err(StorageError::ChecksumMismatch);
    }
    
    // 反序列化数据
    let mut data = CalibrationData::default();
    data.offset = i16::from_le_bytes([buffer[0], buffer[1]]);
    data.gain = f32::from_le_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]);
    
    for i in 0..16 {
        let start = 6 + i * 4;
        data.linearity[i] = f32::from_le_bytes([
            buffer[start], buffer[start+1], buffer[start+2], buffer[start+3]
        ]);
    }
    
    Ok(data)
}
```

## 校准周期和维护

### 1. 校准周期建议

| 应用场景 | 校准周期 | 精度要求 |
|----------|----------|----------|
| 实验室测量 | 6个月 | ±0.05% |
| 工业控制 | 1年 | ±0.1% |
| 消费电子 | 2年 | ±0.5% |
| 教学演示 | 按需 | ±1% |

### 2. 校准触发条件

- 温度变化超过±10°C
- 系统重启后首次使用
- 测量结果异常
- 定期维护时间到达
- 硬件更换后

### 3. 校准质量评估

```rust
pub struct CalibrationQuality {
    pub repeatability: f32,    // 重复性 (%)
    pub stability: f32,        // 稳定性 (ppm/°C)
    pub linearity: f32,        // 线性度 (%)
    pub accuracy: f32,         // 准确度 (%)
    pub drift: f32,           // 漂移 (ppm/month)
}

impl CalibrationQuality {
    pub fn evaluate(&self) -> QualityLevel {
        let score = (100.0 - self.repeatability) * 0.3 +
                   (100.0 - self.linearity) * 0.3 +
                   (100.0 - self.accuracy) * 0.4;
        
        match score {
            s if s >= 95.0 => QualityLevel::Excellent,
            s if s >= 90.0 => QualityLevel::Good,
            s if s >= 80.0 => QualityLevel::Acceptable,
            _ => QualityLevel::Poor,
        }
    }
}
```

## 故障排除

### 1. 常见问题

#### 1.1 校准失败
- **原因**: 参考电压不稳定、温度变化、电磁干扰
- **解决**: 检查参考源、等待温度稳定、改善屏蔽

#### 1.2 精度不达标
- **原因**: 校准点选择不当、非线性严重、噪声过大
- **解决**: 增加校准点、使用高阶校准、改善信号质量

#### 1.3 温度漂移
- **原因**: 温度补偿不足、器件老化
- **解决**: 重新测量温度系数、更换器件

### 2. 调试工具

```rust
pub fn calibration_debug_info(calibrator: &AdcCalibrator) {
    println!("=== ADC校准调试信息 ===");
    println!("偏移值: {}", calibrator.calibration_data.offset);
    println!("增益系数: {:.6}", calibrator.calibration_data.gain);
    println!("温度系数: {:.6} ppm/°C", calibrator.calibration_data.temperature_coeff);
    
    println!("线性度校准表:");
    for (i, &coeff) in calibrator.calibration_data.linearity.iter().enumerate() {
        println!("  段{}: {:.6}", i, coeff);
    }
    
    println!("参考电压: {:.3}V", calibrator.reference_voltage);
    println!("当前温度: {:.1}°C", calibrator.temperature);
}
```

## 最佳实践

1. **环境控制**: 在稳定的温度和湿度环境中进行校准
2. **预热时间**: 系统上电后等待至少5分钟再开始校准
3. **多次测量**: 每个校准点进行多次测量并取平均值
4. **参考标准**: 使用高精度、可溯源的参考标准
5. **记录保存**: 详细记录校准过程和结果
6. **定期验证**: 定期验证校准有效性
7. **版本管理**: 对校准参数进行版本管理和备份

## 总结

ADC校准是确保测量精度的关键步骤。通过系统的校准流程、合适的校准算法和定期的维护，可以显著提高ADC的测量精度和长期稳定性。在实际应用中，应根据具体的精度要求和使用环境选择合适的校准方法和周期。