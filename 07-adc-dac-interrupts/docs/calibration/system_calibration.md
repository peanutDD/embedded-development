# 系统级校准技术文档

## 概述

本文档介绍STM32F4系列微控制器ADC/DAC系统的整体校准策略，包括系统级校准流程、多通道校准、交叉校准验证和自动化校准管理。

## 系统校准架构

### 1. 校准系统组成

#### 1.1 硬件组件
- **STM32F4微控制器**: 主控制器
- **多通道ADC**: 同时采样多个信号
- **多通道DAC**: 同时输出多个信号
- **精密参考源**: 电压和频率参考
- **校准设备**: 万用表、示波器、信号发生器
- **温度传感器**: 环境温度监控
- **校准开关矩阵**: 自动切换测试信号

#### 1.2 软件架构
```rust
pub struct SystemCalibrator {
    adc_calibrators: [AdcCalibrator; 3],    // 多个ADC校准器
    dac_calibrators: [DacCalibrator; 2],    // 多个DAC校准器
    reference_manager: ReferenceManager,     // 参考源管理
    switch_matrix: SwitchMatrix,            // 开关矩阵控制
    temperature_monitor: TemperatureMonitor, // 温度监控
    calibration_scheduler: CalibrationScheduler, // 校准调度器
    data_manager: CalibrationDataManager,   // 数据管理
    verification_engine: VerificationEngine, // 验证引擎
}
```

### 2. 校准数据结构

#### 2.1 系统校准数据
```rust
#[derive(Clone)]
pub struct SystemCalibrationData {
    pub header: CalibrationHeader,
    pub adc_calibrations: [AdcCalibrationData; 3],
    pub dac_calibrations: [DacCalibrationData; 2],
    pub cross_calibration: CrossCalibrationData,
    pub system_parameters: SystemParameters,
    pub environmental_data: EnvironmentalData,
    pub verification_results: VerificationResults,
    pub metadata: CalibrationMetadata,
}

#[derive(Clone)]
pub struct CalibrationHeader {
    pub magic: u32,              // 0x53594343 ("SYCC")
    pub version: u16,            // 校准数据版本
    pub device_serial: u32,      // 设备序列号
    pub calibration_id: u32,     // 校准批次ID
    pub timestamp: u64,          // 校准时间戳
    pub operator_id: u16,        // 操作员ID
    pub checksum: u32,           // 数据校验和
}

#[derive(Clone)]
pub struct CrossCalibrationData {
    pub adc_dac_correlation: [[f32; 2]; 3],  // ADC-DAC相关性矩阵
    pub channel_crosstalk: [f32; 16],        // 通道串扰系数
    pub timing_skew: [i16; 8],               // 时序偏差(ns)
    pub common_mode_rejection: f32,          // 共模抑制比
    pub power_supply_rejection: f32,         // 电源抑制比
    pub temperature_correlation: [f32; 4],   // 温度相关性
}

#[derive(Clone)]
pub struct SystemParameters {
    pub reference_voltage: f32,      // 系统参考电压
    pub reference_frequency: f32,    // 系统参考频率
    pub supply_voltage: f32,         // 电源电压
    pub operating_temperature: f32,  // 工作温度
    pub humidity: f32,               // 湿度
    pub pressure: f32,               // 气压
    pub vibration_level: f32,        // 振动水平
}
```

## 校准流程

### 1. 系统级校准序列

```rust
impl SystemCalibrator {
    pub fn full_system_calibration(&mut self) -> Result<SystemCalibrationData, CalibrationError> {
        println!("开始系统级校准流程...");
        
        // 1. 系统初始化和预热
        self.initialize_system()?;
        self.preheat_system(Duration::from_secs(300))?; // 5分钟预热
        
        // 2. 环境参数记录
        let environmental_data = self.record_environmental_conditions()?;
        
        // 3. 参考源校准
        self.calibrate_reference_sources()?;
        
        // 4. 单独校准各个组件
        self.calibrate_individual_components()?;
        
        // 5. 交叉校准
        let cross_calibration = self.perform_cross_calibration()?;
        
        // 6. 系统级验证
        let verification_results = self.verify_system_performance()?;
        
        // 7. 数据整合和存储
        let calibration_data = SystemCalibrationData {
            header: self.create_calibration_header(),
            adc_calibrations: self.get_adc_calibrations(),
            dac_calibrations: self.get_dac_calibrations(),
            cross_calibration,
            system_parameters: self.get_system_parameters(),
            environmental_data,
            verification_results,
            metadata: self.create_metadata(),
        };
        
        // 8. 数据验证和存储
        self.validate_and_store_calibration_data(&calibration_data)?;
        
        println!("系统校准完成！");
        Ok(calibration_data)
    }

    fn initialize_system(&mut self) -> Result<(), CalibrationError> {
        println!("初始化校准系统...");
        
        // 初始化硬件
        self.reference_manager.initialize()?;
        self.switch_matrix.initialize()?;
        self.temperature_monitor.initialize()?;
        
        // 初始化校准器
        for calibrator in &mut self.adc_calibrators {
            calibrator.initialize()?;
        }
        
        for calibrator in &mut self.dac_calibrators {
            calibrator.initialize()?;
        }
        
        // 系统自检
        self.perform_system_self_test()?;
        
        Ok(())
    }

    fn preheat_system(&mut self, duration: Duration) -> Result<(), CalibrationError> {
        println!("系统预热中... ({:?})", duration);
        
        let start_time = Instant::now();
        let mut last_temp = 0.0f32;
        let mut stable_count = 0u32;
        
        while start_time.elapsed() < duration {
            let current_temp = self.temperature_monitor.read_temperature()?;
            
            // 检查温度稳定性
            if (current_temp - last_temp).abs() < 0.1 {
                stable_count += 1;
            } else {
                stable_count = 0;
            }
            
            // 如果温度已稳定5分钟，可以提前结束预热
            if stable_count > 300 && start_time.elapsed() > Duration::from_secs(180) {
                println!("温度已稳定，预热完成");
                break;
            }
            
            last_temp = current_temp;
            delay_ms(1000); // 每秒检查一次
            
            // 显示进度
            if start_time.elapsed().as_secs() % 30 == 0 {
                println!("预热进度: {:.1}°C, 剩余时间: {:?}", 
                        current_temp, 
                        duration - start_time.elapsed());
            }
        }
        
        Ok(())
    }

    fn calibrate_reference_sources(&mut self) -> Result<(), CalibrationError> {
        println!("校准参考源...");
        
        // 校准电压参考
        let voltage_ref_error = self.reference_manager.calibrate_voltage_reference()?;
        println!("电压参考校准: 误差 {:.3}%", voltage_ref_error * 100.0);
        
        // 校准频率参考
        let frequency_ref_error = self.reference_manager.calibrate_frequency_reference()?;
        println!("频率参考校准: 误差 {:.1}ppm", frequency_ref_error * 1e6);
        
        // 验证参考源稳定性
        self.verify_reference_stability()?;
        
        Ok(())
    }

    fn calibrate_individual_components(&mut self) -> Result<(), CalibrationError> {
        println!("校准各个组件...");
        
        // 并行校准ADC（如果硬件支持）
        for (i, calibrator) in self.adc_calibrators.iter_mut().enumerate() {
            println!("校准ADC{}...", i + 1);
            calibrator.full_calibration()?;
        }
        
        // 并行校准DAC
        for (i, calibrator) in self.dac_calibrators.iter_mut().enumerate() {
            println!("校准DAC{}...", i + 1);
            calibrator.full_calibration()?;
        }
        
        Ok(())
    }

    fn perform_cross_calibration(&mut self) -> Result<CrossCalibrationData, CalibrationError> {
        println!("执行交叉校准...");
        
        let mut cross_data = CrossCalibrationData::default();
        
        // 1. ADC-DAC相关性校准
        cross_data.adc_dac_correlation = self.calibrate_adc_dac_correlation()?;
        
        // 2. 通道串扰测量
        cross_data.channel_crosstalk = self.measure_channel_crosstalk()?;
        
        // 3. 时序偏差校准
        cross_data.timing_skew = self.calibrate_timing_skew()?;
        
        // 4. 共模抑制比测量
        cross_data.common_mode_rejection = self.measure_cmrr()?;
        
        // 5. 电源抑制比测量
        cross_data.power_supply_rejection = self.measure_psrr()?;
        
        // 6. 温度相关性分析
        cross_data.temperature_correlation = self.analyze_temperature_correlation()?;
        
        Ok(cross_data)
    }

    fn calibrate_adc_dac_correlation(&mut self) -> Result<[[f32; 2]; 3], CalibrationError> {
        println!("校准ADC-DAC相关性...");
        
        let mut correlation_matrix = [[0.0f32; 2]; 3];
        let test_voltages = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]; // 测试电压点
        
        for adc_idx in 0..3 {
            for dac_idx in 0..2 {
                let mut correlation_sum = 0.0f32;
                let mut count = 0;
                
                for &test_voltage in &test_voltages {
                    // DAC输出测试电压
                    let dac_value = self.voltage_to_dac_value(test_voltage, dac_idx);
                    self.set_dac_output(dac_idx, dac_value)?;
                    
                    // 通过开关矩阵连接DAC输出到ADC输入
                    self.switch_matrix.connect_dac_to_adc(dac_idx, adc_idx)?;
                    delay_ms(10); // 稳定时间
                    
                    // ADC测量
                    let measured_voltage = self.read_adc_voltage(adc_idx)?;
                    
                    // 计算相关性
                    let correlation = measured_voltage / test_voltage;
                    correlation_sum += correlation;
                    count += 1;
                    
                    println!("ADC{}-DAC{}: 设定={:.3}V, 测量={:.3}V, 相关性={:.6}", 
                            adc_idx + 1, dac_idx + 1, test_voltage, measured_voltage, correlation);
                }
                
                correlation_matrix[adc_idx][dac_idx] = correlation_sum / count as f32;
            }
        }
        
        Ok(correlation_matrix)
    }

    fn measure_channel_crosstalk(&mut self) -> Result<[f32; 16], CalibrationError> {
        println!("测量通道串扰...");
        
        let mut crosstalk = [0.0f32; 16];
        let test_amplitude = 2.0; // 2V测试信号
        
        // 测试每个通道对其他通道的影响
        for source_ch in 0..4 { // 假设4个输出通道
            // 在源通道输出信号
            self.set_channel_output(source_ch, test_amplitude)?;
            
            for victim_ch in 0..4 { // 测量4个输入通道
                if source_ch != victim_ch {
                    let crosstalk_voltage = self.read_channel_input(victim_ch)?;
                    let crosstalk_ratio = crosstalk_voltage / test_amplitude;
                    
                    let index = source_ch * 4 + victim_ch;
                    if index < 16 {
                        crosstalk[index] = crosstalk_ratio;
                    }
                    
                    println!("通道{}->通道{}: {:.1}dB", 
                            source_ch, victim_ch, 20.0 * crosstalk_ratio.log10());
                }
            }
            
            // 关闭源通道输出
            self.set_channel_output(source_ch, 0.0)?;
        }
        
        Ok(crosstalk)
    }

    fn calibrate_timing_skew(&mut self) -> Result<[i16; 8], CalibrationError> {
        println!("校准时序偏差...");
        
        let mut timing_skew = [0i16; 8];
        let test_frequency = 10000.0; // 10kHz测试信号
        
        // 生成同步测试信号
        self.generate_sync_test_signals(test_frequency)?;
        
        // 测量各通道的相位差
        for ch in 0..8 {
            let phase_delay = self.measure_channel_phase_delay(ch, test_frequency)?;
            // 转换为纳秒
            timing_skew[ch] = (phase_delay / (2.0 * PI * test_frequency) * 1e9) as i16;
            
            println!("通道{} 时序偏差: {}ns", ch, timing_skew[ch]);
        }
        
        Ok(timing_skew)
    }

    fn measure_cmrr(&mut self) -> Result<f32, CalibrationError> {
        println!("测量共模抑制比...");
        
        let common_mode_voltage = 1.5; // 1.5V共模电压
        let differential_voltage = 0.01; // 10mV差分信号
        
        // 施加共模电压
        self.apply_common_mode_voltage(common_mode_voltage)?;
        let common_mode_output = self.measure_differential_output()?;
        
        // 施加差分信号
        self.apply_differential_voltage(differential_voltage)?;
        let differential_output = self.measure_differential_output()?;
        
        // 计算CMRR
        let cmrr = 20.0 * (differential_output / common_mode_output).log10();
        println!("共模抑制比: {:.1}dB", cmrr);
        
        Ok(cmrr)
    }

    fn measure_psrr(&mut self) -> Result<f32, CalibrationError> {
        println!("测量电源抑制比...");
        
        let nominal_supply = 3.3;
        let supply_variation = 0.1; // 100mV电源变化
        
        // 测量标称电源电压下的输出
        self.set_supply_voltage(nominal_supply)?;
        let nominal_output = self.measure_system_output()?;
        
        // 测量电源变化时的输出
        self.set_supply_voltage(nominal_supply + supply_variation)?;
        let varied_output = self.measure_system_output()?;
        
        // 计算PSRR
        let output_change = (varied_output - nominal_output).abs();
        let psrr = 20.0 * (supply_variation / output_change).log10();
        println!("电源抑制比: {:.1}dB", psrr);
        
        // 恢复标称电源电压
        self.set_supply_voltage(nominal_supply)?;
        
        Ok(psrr)
    }
}
```

### 2. 自动化校准管理

```rust
pub struct CalibrationScheduler {
    schedule: Vec<CalibrationTask>,
    current_task: Option<CalibrationTask>,
    completion_status: HashMap<String, CalibrationStatus>,
}

#[derive(Clone)]
pub struct CalibrationTask {
    pub id: String,
    pub task_type: CalibrationTaskType,
    pub priority: Priority,
    pub estimated_duration: Duration,
    pub dependencies: Vec<String>,
    pub retry_count: u8,
    pub max_retries: u8,
}

#[derive(Clone)]
pub enum CalibrationTaskType {
    SystemInitialization,
    ReferenceCalibration,
    AdcCalibration(usize),
    DacCalibration(usize),
    CrossCalibration,
    SystemVerification,
    DataStorage,
}

impl CalibrationScheduler {
    pub fn create_full_calibration_schedule() -> Self {
        let mut schedule = Vec::new();
        
        // 1. 系统初始化
        schedule.push(CalibrationTask {
            id: "sys_init".to_string(),
            task_type: CalibrationTaskType::SystemInitialization,
            priority: Priority::Critical,
            estimated_duration: Duration::from_secs(60),
            dependencies: vec![],
            retry_count: 0,
            max_retries: 3,
        });
        
        // 2. 参考源校准
        schedule.push(CalibrationTask {
            id: "ref_cal".to_string(),
            task_type: CalibrationTaskType::ReferenceCalibration,
            priority: Priority::High,
            estimated_duration: Duration::from_secs(300),
            dependencies: vec!["sys_init".to_string()],
            retry_count: 0,
            max_retries: 2,
        });
        
        // 3. ADC校准
        for i in 0..3 {
            schedule.push(CalibrationTask {
                id: format!("adc_cal_{}", i),
                task_type: CalibrationTaskType::AdcCalibration(i),
                priority: Priority::High,
                estimated_duration: Duration::from_secs(600),
                dependencies: vec!["ref_cal".to_string()],
                retry_count: 0,
                max_retries: 2,
            });
        }
        
        // 4. DAC校准
        for i in 0..2 {
            schedule.push(CalibrationTask {
                id: format!("dac_cal_{}", i),
                task_type: CalibrationTaskType::DacCalibration(i),
                priority: Priority::High,
                estimated_duration: Duration::from_secs(600),
                dependencies: vec!["ref_cal".to_string()],
                retry_count: 0,
                max_retries: 2,
            });
        }
        
        // 5. 交叉校准
        let mut cross_dependencies = vec![];
        for i in 0..3 {
            cross_dependencies.push(format!("adc_cal_{}", i));
        }
        for i in 0..2 {
            cross_dependencies.push(format!("dac_cal_{}", i));
        }
        
        schedule.push(CalibrationTask {
            id: "cross_cal".to_string(),
            task_type: CalibrationTaskType::CrossCalibration,
            priority: Priority::Medium,
            estimated_duration: Duration::from_secs(900),
            dependencies: cross_dependencies,
            retry_count: 0,
            max_retries: 2,
        });
        
        // 6. 系统验证
        schedule.push(CalibrationTask {
            id: "sys_verify".to_string(),
            task_type: CalibrationTaskType::SystemVerification,
            priority: Priority::High,
            estimated_duration: Duration::from_secs(300),
            dependencies: vec!["cross_cal".to_string()],
            retry_count: 0,
            max_retries: 1,
        });
        
        // 7. 数据存储
        schedule.push(CalibrationTask {
            id: "data_store".to_string(),
            task_type: CalibrationTaskType::DataStorage,
            priority: Priority::Critical,
            estimated_duration: Duration::from_secs(30),
            dependencies: vec!["sys_verify".to_string()],
            retry_count: 0,
            max_retries: 5,
        });
        
        Self {
            schedule,
            current_task: None,
            completion_status: HashMap::new(),
        }
    }

    pub fn execute_schedule(&mut self, calibrator: &mut SystemCalibrator) -> Result<(), CalibrationError> {
        println!("开始执行校准计划...");
        
        let total_tasks = self.schedule.len();
        let mut completed_tasks = 0;
        
        while let Some(task) = self.get_next_ready_task() {
            println!("执行任务: {} ({}/{})", task.id, completed_tasks + 1, total_tasks);
            
            self.current_task = Some(task.clone());
            
            let start_time = Instant::now();
            let result = self.execute_task(&task, calibrator);
            let execution_time = start_time.elapsed();
            
            match result {
                Ok(_) => {
                    println!("任务 {} 完成，耗时: {:?}", task.id, execution_time);
                    self.completion_status.insert(task.id.clone(), CalibrationStatus::Completed);
                    completed_tasks += 1;
                }
                Err(e) => {
                    println!("任务 {} 失败: {:?}", task.id, e);
                    
                    if task.retry_count < task.max_retries {
                        println!("重试任务 {} ({}/{})", task.id, task.retry_count + 1, task.max_retries);
                        let mut retry_task = task.clone();
                        retry_task.retry_count += 1;
                        self.schedule.push(retry_task);
                        self.completion_status.insert(task.id.clone(), CalibrationStatus::Retrying);
                    } else {
                        println!("任务 {} 达到最大重试次数，标记为失败", task.id);
                        self.completion_status.insert(task.id.clone(), CalibrationStatus::Failed);
                        
                        if task.priority == Priority::Critical {
                            return Err(CalibrationError::CriticalTaskFailed(task.id));
                        }
                    }
                }
            }
            
            self.current_task = None;
        }
        
        println!("校准计划执行完成！");
        self.print_completion_summary();
        
        Ok(())
    }

    fn execute_task(&self, task: &CalibrationTask, calibrator: &mut SystemCalibrator) -> Result<(), CalibrationError> {
        match &task.task_type {
            CalibrationTaskType::SystemInitialization => {
                calibrator.initialize_system()
            }
            CalibrationTaskType::ReferenceCalibration => {
                calibrator.calibrate_reference_sources()
            }
            CalibrationTaskType::AdcCalibration(index) => {
                calibrator.adc_calibrators[*index].full_calibration()
            }
            CalibrationTaskType::DacCalibration(index) => {
                calibrator.dac_calibrators[*index].full_calibration()
            }
            CalibrationTaskType::CrossCalibration => {
                calibrator.perform_cross_calibration().map(|_| ())
            }
            CalibrationTaskType::SystemVerification => {
                calibrator.verify_system_performance().map(|_| ())
            }
            CalibrationTaskType::DataStorage => {
                calibrator.save_calibration_data()
            }
        }
    }
}
```

## 验证和质量控制

### 1. 系统级验证

```rust
pub struct VerificationEngine {
    test_specifications: TestSpecifications,
    measurement_equipment: MeasurementEquipment,
    test_results: Vec<TestResult>,
}

impl VerificationEngine {
    pub fn verify_system_performance(&mut self, calibrator: &SystemCalibrator) -> Result<VerificationResults, CalibrationError> {
        println!("开始系统性能验证...");
        
        let mut results = VerificationResults::new();
        
        // 1. 静态精度验证
        results.static_accuracy = self.verify_static_accuracy(calibrator)?;
        
        // 2. 动态性能验证
        results.dynamic_performance = self.verify_dynamic_performance(calibrator)?;
        
        // 3. 温度特性验证
        results.temperature_performance = self.verify_temperature_performance(calibrator)?;
        
        // 4. 长期稳定性验证
        results.stability_performance = self.verify_stability_performance(calibrator)?;
        
        // 5. 系统级指标验证
        results.system_metrics = self.verify_system_metrics(calibrator)?;
        
        // 6. 生成验证报告
        self.generate_verification_report(&results)?;
        
        Ok(results)
    }

    fn verify_static_accuracy(&mut self, calibrator: &SystemCalibrator) -> Result<StaticAccuracyResults, CalibrationError> {
        println!("验证静态精度...");
        
        let mut results = StaticAccuracyResults::new();
        
        // 测试点定义
        let test_points = [
            0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0
        ]; // 相对于满量程的比例
        
        // ADC精度验证
        for (adc_idx, adc_cal) in calibrator.adc_calibrators.iter().enumerate() {
            let mut adc_errors = Vec::new();
            
            for &point in &test_points {
                let test_voltage = point * calibrator.reference_manager.get_reference_voltage();
                
                // 应用测试电压
                self.measurement_equipment.set_precision_voltage(test_voltage)?;
                delay_ms(100); // 稳定时间
                
                // ADC测量
                let measured_voltage = adc_cal.read_calibrated_voltage()?;
                let error = (measured_voltage - test_voltage) / test_voltage * 100.0;
                adc_errors.push(error);
                
                println!("ADC{} @{:.1}%: 设定={:.4}V, 测量={:.4}V, 误差={:.3}%", 
                        adc_idx + 1, point * 100.0, test_voltage, measured_voltage, error);
            }
            
            results.adc_accuracy[adc_idx] = AdcAccuracyResult {
                max_error: adc_errors.iter().fold(0.0f32, |a, &b| a.max(b.abs())),
                rms_error: (adc_errors.iter().map(|e| e * e).sum::<f32>() / adc_errors.len() as f32).sqrt(),
                linearity_error: self.calculate_linearity_error(&adc_errors),
                passed: adc_errors.iter().all(|&e| e.abs() < self.test_specifications.adc_accuracy_limit),
            };
        }
        
        // DAC精度验证
        for (dac_idx, dac_cal) in calibrator.dac_calibrators.iter().enumerate() {
            let mut dac_errors = Vec::new();
            
            for &point in &test_points {
                let target_voltage = point * calibrator.reference_manager.get_reference_voltage();
                let dac_value = calibrator.voltage_to_dac_value(target_voltage, dac_idx);
                
                // DAC输出
                dac_cal.set_calibrated_output(dac_value)?;
                delay_ms(100);
                
                // 精密测量
                let measured_voltage = self.measurement_equipment.measure_precision_voltage()?;
                let error = (measured_voltage - target_voltage) / target_voltage * 100.0;
                dac_errors.push(error);
                
                println!("DAC{} @{:.1}%: 目标={:.4}V, 测量={:.4}V, 误差={:.3}%", 
                        dac_idx + 1, point * 100.0, target_voltage, measured_voltage, error);
            }
            
            results.dac_accuracy[dac_idx] = DacAccuracyResult {
                max_error: dac_errors.iter().fold(0.0f32, |a, &b| a.max(b.abs())),
                rms_error: (dac_errors.iter().map(|e| e * e).sum::<f32>() / dac_errors.len() as f32).sqrt(),
                linearity_error: self.calculate_linearity_error(&dac_errors),
                passed: dac_errors.iter().all(|&e| e.abs() < self.test_specifications.dac_accuracy_limit),
            };
        }
        
        Ok(results)
    }

    fn verify_dynamic_performance(&mut self, calibrator: &SystemCalibrator) -> Result<DynamicPerformanceResults, CalibrationError> {
        println!("验证动态性能...");
        
        let mut results = DynamicPerformanceResults::new();
        
        let test_frequencies = [100.0, 1000.0, 10000.0, 50000.0, 100000.0];
        
        // 频率响应测试
        for &frequency in &test_frequencies {
            // 生成测试信号
            self.measurement_equipment.generate_sine_wave(frequency, 1.0)?;
            
            // 测量系统响应
            let amplitude_response = self.measure_amplitude_response(calibrator, frequency)?;
            let phase_response = self.measure_phase_response(calibrator, frequency)?;
            
            results.frequency_response.push(FrequencyResponsePoint {
                frequency,
                amplitude_db: 20.0 * amplitude_response.log10(),
                phase_deg: phase_response * 180.0 / PI,
            });
            
            println!("频率响应 @{}Hz: 幅度={:.2}dB, 相位={:.1}°", 
                    frequency, 20.0 * amplitude_response.log10(), phase_response * 180.0 / PI);
        }
        
        // THD+N测试
        for &frequency in &[1000.0, 10000.0] {
            let thd_n = self.measure_thd_n(calibrator, frequency)?;
            results.thd_n_measurements.push(ThdNMeasurement {
                frequency,
                thd_n_db: thd_n,
                passed: thd_n < self.test_specifications.thd_n_limit,
            });
            
            println!("THD+N @{}Hz: {:.1}dB", frequency, thd_n);
        }
        
        // SNR测试
        let snr = self.measure_snr(calibrator)?;
        results.snr_db = snr;
        results.snr_passed = snr > self.test_specifications.snr_limit;
        
        println!("信噪比: {:.1}dB", snr);
        
        Ok(results)
    }
}
```

## 数据管理和追溯

### 1. 校准数据管理

```rust
pub struct CalibrationDataManager {
    storage_backend: StorageBackend,
    encryption_key: [u8; 32],
    compression_enabled: bool,
    backup_locations: Vec<BackupLocation>,
}

impl CalibrationDataManager {
    pub fn store_calibration_data(&mut self, data: &SystemCalibrationData) -> Result<String, StorageError> {
        // 1. 数据验证
        self.validate_calibration_data(data)?;
        
        // 2. 数据序列化
        let serialized_data = self.serialize_data(data)?;
        
        // 3. 数据压缩（可选）
        let compressed_data = if self.compression_enabled {
            self.compress_data(&serialized_data)?
        } else {
            serialized_data
        };
        
        // 4. 数据加密
        let encrypted_data = self.encrypt_data(&compressed_data)?;
        
        // 5. 生成存储ID
        let storage_id = self.generate_storage_id(data);
        
        // 6. 主存储
        self.storage_backend.store(&storage_id, &encrypted_data)?;
        
        // 7. 备份存储
        for backup_location in &self.backup_locations {
            backup_location.store(&storage_id, &encrypted_data)?;
        }
        
        // 8. 更新索引
        self.update_calibration_index(&storage_id, data)?;
        
        println!("校准数据已存储，ID: {}", storage_id);
        Ok(storage_id)
    }

    pub fn retrieve_calibration_data(&mut self, storage_id: &str) -> Result<SystemCalibrationData, StorageError> {
        // 1. 从主存储加载
        let encrypted_data = match self.storage_backend.load(storage_id) {
            Ok(data) => data,
            Err(_) => {
                // 2. 尝试从备份加载
                println!("主存储失败，尝试从备份恢复...");
                self.restore_from_backup(storage_id)?
            }
        };
        
        // 3. 数据解密
        let compressed_data = self.decrypt_data(&encrypted_data)?;
        
        // 4. 数据解压缩
        let serialized_data = if self.compression_enabled {
            self.decompress_data(&compressed_data)?
        } else {
            compressed_data
        };
        
        // 5. 数据反序列化
        let calibration_data = self.deserialize_data(&serialized_data)?;
        
        // 6. 数据验证
        self.validate_calibration_data(&calibration_data)?;
        
        Ok(calibration_data)
    }

    pub fn create_calibration_report(&self, data: &SystemCalibrationData) -> Result<CalibrationReport, ReportError> {
        let mut report = CalibrationReport::new();
        
        // 基本信息
        report.header = ReportHeader {
            title: "系统校准报告".to_string(),
            device_serial: data.header.device_serial,
            calibration_date: data.header.timestamp,
            operator_id: data.header.operator_id,
            report_version: "1.0".to_string(),
        };
        
        // 校准摘要
        report.summary = CalibrationSummary {
            total_components: data.adc_calibrations.len() + data.dac_calibrations.len(),
            successful_calibrations: self.count_successful_calibrations(data),
            overall_status: self.determine_overall_status(data),
            calibration_duration: self.calculate_calibration_duration(data),
        };
        
        // 详细结果
        report.adc_results = self.format_adc_results(&data.adc_calibrations);
        report.dac_results = self.format_dac_results(&data.dac_calibrations);
        report.cross_calibration_results = self.format_cross_calibration_results(&data.cross_calibration);
        
        // 验证结果
        report.verification_results = self.format_verification_results(&data.verification_results);
        
        // 环境条件
        report.environmental_conditions = self.format_environmental_data(&data.environmental_data);
        
        // 建议和注意事项
        report.recommendations = self.generate_recommendations(data);
        
        Ok(report)
    }
}
```

## 最佳实践

### 1. 校准策略

1. **分层校准**: 先校准参考源，再校准各组件，最后进行系统级校准
2. **并行处理**: 在硬件允许的情况下并行校准多个组件
3. **温度管理**: 确保校准过程中温度稳定
4. **多点校准**: 使用足够的校准点确保精度
5. **交叉验证**: 使用独立的测量设备验证校准结果

### 2. 质量控制

1. **统计过程控制**: 监控校准参数的统计分布
2. **趋势分析**: 跟踪校准参数随时间的变化
3. **异常检测**: 自动识别异常的校准结果
4. **可追溯性**: 完整记录校准过程和结果
5. **定期复校**: 建立定期重新校准的机制

### 3. 数据安全

1. **数据加密**: 对敏感的校准数据进行加密存储
2. **多重备份**: 在多个位置备份校准数据
3. **完整性检查**: 使用校验和验证数据完整性
4. **访问控制**: 限制对校准数据的访问权限
5. **审计日志**: 记录所有数据访问和修改操作

## 总结

系统级校准是确保整个ADC/DAC系统性能的关键环节。通过系统化的校准流程、自动化的管理工具和完善的质量控制机制，可以实现高精度、高可靠性的测量和输出系统。在实际应用中，应根据具体的性能要求和应用场景制定合适的校准策略和验证标准。