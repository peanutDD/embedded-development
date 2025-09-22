use micromath::F32Ext;

#[cfg(feature = "rtt-log")]
use rtt_target::rprintln;

/// 电气特性测试器
pub struct ElectricalTester {
    system_voltage: f32,
    temperature: f32,
    test_frequency: f32,
}

/// 电压测量结果
#[derive(Debug, Clone)]
pub struct VoltageMeasurement {
    pub voltage: f32,
    pub timestamp_us: u32,
    pub temperature: f32,
    pub accuracy: f32,
}

/// 电流测量结果
#[derive(Debug, Clone)]
pub struct CurrentMeasurement {
    pub current: f32,
    pub voltage_drop: f32,
    pub power: f32,
    pub efficiency: f32,
}

/// 阻抗测量结果
#[derive(Debug, Clone)]
pub struct ImpedanceMeasurement {
    pub resistance: f32,
    pub reactance: f32,
    pub impedance_magnitude: f32,
    pub phase_angle: f32,
}

impl ElectricalTester {
    /// 创建新的电气测试器
    pub fn new(system_voltage: f32) -> Self {
        Self {
            system_voltage,
            temperature: 25.0, // 默认室温
            test_frequency: 1000.0, // 默认1kHz测试频率
        }
    }

    /// 设置测试温度
    pub fn set_temperature(&mut self, temperature: f32) {
        self.temperature = temperature;
        rprintln!("测试温度设置为: {:.1}°C", temperature);
    }

    /// 设置测试频率
    pub fn set_test_frequency(&mut self, frequency: f32) {
        self.test_frequency = frequency;
        rprintln!("测试频率设置为: {:.1}Hz", frequency);
    }

    /// 测量输出高电平电压
    pub fn measure_voh(&self, load_current: f32) -> VoltageMeasurement {
        rprintln!("测量输出高电平电压 (负载电流: {:.1}mA)", load_current * 1000.0);
        
        // 考虑温度系数和负载效应
        let temp_coefficient = 1.0 - (self.temperature - 25.0) * 0.001;
        let load_drop = load_current * 10.0; // 假设内阻10Ω
        let voltage = (self.system_voltage - 0.1 - load_drop) * temp_coefficient;
        
        VoltageMeasurement {
            voltage,
            timestamp_us: self.get_timestamp(),
            temperature: self.temperature,
            accuracy: 0.01, // 1%精度
        }
    }

    /// 测量输出低电平电压
    pub fn measure_vol(&self, sink_current: f32) -> VoltageMeasurement {
        rprintln!("测量输出低电平电压 (灌电流: {:.1}mA)", sink_current * 1000.0);
        
        // 考虑温度系数和负载效应
        let temp_coefficient = 1.0 + (self.temperature - 25.0) * 0.001;
        let voltage_rise = sink_current * 8.0; // 假设导通电阻8Ω
        let voltage = (0.05 + voltage_rise) * temp_coefficient;
        
        VoltageMeasurement {
            voltage,
            timestamp_us: self.get_timestamp(),
            temperature: self.temperature,
            accuracy: 0.01,
        }
    }

    /// 测量输入阈值电压
    pub fn measure_input_threshold(&self, rising_edge: bool) -> VoltageMeasurement {
        let threshold_type = if rising_edge { "上升沿" } else { "下降沿" };
        rprintln!("测量输入阈值电压 ({})", threshold_type);
        
        let base_threshold = if rising_edge {
            self.system_voltage * 0.7 // VIH
        } else {
            self.system_voltage * 0.3 // VIL
        };
        
        // 考虑温度漂移
        let temp_drift = (self.temperature - 25.0) * 0.002;
        let voltage = base_threshold + temp_drift;
        
        VoltageMeasurement {
            voltage,
            timestamp_us: self.get_timestamp(),
            temperature: self.temperature,
            accuracy: 0.02, // 2%精度
        }
    }

    /// 测量最大输出电流
    pub fn measure_max_output_current(&self) -> CurrentMeasurement {
        rprintln!("测量最大输出电流");
        
        // 模拟负载测试
        let mut max_current = 0.0;
        let mut test_voltage = self.system_voltage;
        
        // 逐步增加负载，直到电压下降到不可接受的水平
        for load_resistance in (50..=1000).step_by(50) {
            let current = test_voltage / load_resistance as f32;
            let voltage_drop = current * 10.0; // 内阻压降
            test_voltage = self.system_voltage - voltage_drop;
            
            if test_voltage >= self.system_voltage * 0.8 {
                max_current = current;
            } else {
                break;
            }
        }
        
        CurrentMeasurement {
            current: max_current,
            voltage_drop: self.system_voltage - test_voltage,
            power: max_current * test_voltage,
            efficiency: test_voltage / self.system_voltage,
        }
    }

    /// 测量输入漏电流
    pub fn measure_input_leakage(&self, input_voltage: f32) -> CurrentMeasurement {
        rprintln!("测量输入漏电流 (输入电压: {:.2}V)", input_voltage);
        
        // 模拟漏电流测量
        let base_leakage = 1e-9; // 1nA基础漏电流
        
        // 考虑温度效应（每10°C翻倍）
        let temp_factor = 2.0_f32.powf((self.temperature - 25.0) / 10.0);
        
        // 考虑电压效应
        let voltage_factor = if input_voltage > self.system_voltage {
            (input_voltage / self.system_voltage).powf(2.0)
        } else {
            1.0
        };
        
        let leakage_current = base_leakage * temp_factor * voltage_factor;
        
        CurrentMeasurement {
            current: leakage_current,
            voltage_drop: 0.0,
            power: leakage_current * input_voltage,
            efficiency: 1.0,
        }
    }

    /// 测量输入阻抗
    pub fn measure_input_impedance(&self) -> ImpedanceMeasurement {
        rprintln!("测量输入阻抗 (频率: {:.1}Hz)", self.test_frequency);
        
        // 模拟阻抗测量
        let base_resistance = 1e12; // 1TΩ基础电阻
        let input_capacitance = 5e-12; // 5pF输入电容
        
        // 计算频率相关的阻抗
        let omega = 2.0 * core::f32::consts::PI * self.test_frequency;
        let capacitive_reactance = 1.0 / (omega * input_capacitance);
        
        let impedance_magnitude = (base_resistance * base_resistance + 
                                  capacitive_reactance * capacitive_reactance).sqrt();
        let phase_angle = (-capacitive_reactance / base_resistance).atan();
        
        ImpedanceMeasurement {
            resistance: base_resistance,
            reactance: -capacitive_reactance,
            impedance_magnitude,
            phase_angle: phase_angle * 180.0 / core::f32::consts::PI,
        }
    }

    /// 测量驱动强度特性
    pub fn characterize_drive_strength(&self) -> DriveStrengthCharacteristics {
        rprintln!("表征驱动强度特性");
        
        let mut characteristics = DriveStrengthCharacteristics::new();
        
        // 测试不同负载下的性能
        for load_pf in [10, 50, 100, 200, 500] {
            let load_capacitance = load_pf as f32 * 1e-12;
            let rise_time = self.calculate_rise_time(load_capacitance);
            let fall_time = self.calculate_fall_time(load_capacitance);
            
            characteristics.add_measurement(LoadMeasurement {
                load_capacitance_pf: load_pf as f32,
                rise_time_ns: rise_time,
                fall_time_ns: fall_time,
                power_consumption_uw: self.calculate_switching_power(load_capacitance),
            });
        }
        
        characteristics
    }

    /// 温度特性测试
    pub fn temperature_characterization(&mut self, temp_range: (f32, f32)) -> TemperatureCharacteristics {
        rprintln!("温度特性测试 ({:.1}°C 到 {:.1}°C)", temp_range.0, temp_range.1);
        
        let mut characteristics = TemperatureCharacteristics::new();
        
        for temp in ((temp_range.0 as i32)..=(temp_range.1 as i32)).step_by(10) {
            self.set_temperature(temp as f32);
            
            let voh = self.measure_voh(0.01); // 10mA负载
            let vol = self.measure_vol(0.01); // 10mA负载
            let leakage = self.measure_input_leakage(self.system_voltage);
            
            characteristics.add_measurement(TemperatureMeasurement {
                temperature: temp as f32,
                voh: voh.voltage,
                vol: vol.voltage,
                leakage_current: leakage.current,
                threshold_drift: self.calculate_threshold_drift(temp as f32),
            });
        }
        
        characteristics
    }

    // 私有辅助方法
    fn get_timestamp(&self) -> u32 {
        // 模拟时间戳
        42000 // 微秒
    }

    fn calculate_rise_time(&self, load_capacitance: f32) -> f32 {
        // 简化的RC时间常数计算
        let drive_resistance = 50.0; // 50Ω驱动电阻
        let time_constant = drive_resistance * load_capacitance;
        time_constant * 2.2 * 1e9 // 转换为纳秒
    }

    fn calculate_fall_time(&self, load_capacitance: f32) -> f32 {
        // 下降时间通常比上升时间快
        self.calculate_rise_time(load_capacitance) * 0.8
    }

    fn calculate_switching_power(&self, load_capacitance: f32) -> f32 {
        // P = C * V² * f
        load_capacitance * self.system_voltage * self.system_voltage * 
        self.test_frequency * 1e6 // 转换为微瓦
    }

    fn calculate_threshold_drift(&self, temperature: f32) -> f32 {
        // 温度漂移系数：2mV/°C
        (temperature - 25.0) * 0.002
    }
}

/// 驱动强度特性
#[derive(Debug)]
pub struct DriveStrengthCharacteristics {
    measurements: heapless::Vec<LoadMeasurement, 8>,
}

#[derive(Debug, Clone)]
pub struct LoadMeasurement {
    pub load_capacitance_pf: f32,
    pub rise_time_ns: f32,
    pub fall_time_ns: f32,
    pub power_consumption_uw: f32,
}

/// 温度特性
#[derive(Debug)]
pub struct TemperatureCharacteristics {
    measurements: heapless::Vec<TemperatureMeasurement, 16>,
}

#[derive(Debug, Clone)]
pub struct TemperatureMeasurement {
    pub temperature: f32,
    pub voh: f32,
    pub vol: f32,
    pub leakage_current: f32,
    pub threshold_drift: f32,
}

impl DriveStrengthCharacteristics {
    fn new() -> Self {
        Self {
            measurements: heapless::Vec::new(),
        }
    }

    fn add_measurement(&mut self, measurement: LoadMeasurement) {
        let _ = self.measurements.push(measurement);
    }

    pub fn get_measurements(&self) -> &[LoadMeasurement] {
        &self.measurements
    }

    pub fn find_max_load(&self) -> Option<f32> {
        self.measurements.iter()
            .filter(|m| m.rise_time_ns < 50.0) // 50ns限制
            .map(|m| m.load_capacitance_pf)
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    }
}

impl TemperatureCharacteristics {
    fn new() -> Self {
        Self {
            measurements: heapless::Vec::new(),
        }
    }

    fn add_measurement(&mut self, measurement: TemperatureMeasurement) {
        let _ = self.measurements.push(measurement);
    }

    pub fn get_measurements(&self) -> &[TemperatureMeasurement] {
        &self.measurements
    }

    pub fn calculate_temp_coefficient(&self) -> f32 {
        if self.measurements.len() < 2 {
            return 0.0;
        }

        let first = &self.measurements[0];
        let last = &self.measurements[self.measurements.len() - 1];
        
        let voltage_change = last.voh - first.voh;
        let temp_change = last.temperature - first.temperature;
        
        if temp_change != 0.0 {
            voltage_change / temp_change
        } else {
            0.0
        }
    }
}