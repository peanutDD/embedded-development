use micromath::F32Ext;

#[cfg(feature = "rtt-log")]
use rtt_target::rprintln;

/// 时序测试器
pub struct TimingTester {
    system_clock_mhz: f32,
    measurement_resolution_ps: f32,
    temperature: f32,
}

/// 时序测量结果
#[derive(Debug, Clone)]
pub struct TimingMeasurement {
    pub delay_ns: f32,
    pub jitter_ps: f32,
    pub accuracy_percent: f32,
    pub temperature: f32,
    pub load_condition: LoadCondition,
}

/// 负载条件
#[derive(Debug, Clone)]
pub struct LoadCondition {
    pub capacitance_pf: f32,
    pub resistance_ohm: f32,
    pub voltage_swing_v: f32,
}

/// 频率响应测试结果
#[derive(Debug, Clone)]
pub struct FrequencyResponse {
    pub frequency_mhz: f32,
    pub amplitude_db: f32,
    pub phase_deg: f32,
    pub distortion_percent: f32,
}

/// 眼图分析结果
#[derive(Debug, Clone)]
pub struct EyeDiagramAnalysis {
    pub eye_height_v: f32,
    pub eye_width_ns: f32,
    pub jitter_rms_ps: f32,
    pub crossing_percentage: f32,
    pub quality_factor: f32,
}

impl TimingTester {
    /// 创建新的时序测试器
    pub fn new(system_clock_mhz: f32) -> Self {
        Self {
            system_clock_mhz,
            measurement_resolution_ps: 10.0, // 10ps分辨率
            temperature: 25.0,
        }
    }

    /// 设置测量分辨率
    pub fn set_resolution(&mut self, resolution_ps: f32) {
        self.measurement_resolution_ps = resolution_ps;
        rprintln!("时序测量分辨率设置为: {:.1}ps", resolution_ps);
    }

    /// 测量传播延迟
    pub fn measure_propagation_delay(&self, load: &LoadCondition) -> TimingMeasurement {
        rprintln!("测量传播延迟 (负载: {:.0}pF, {:.0}Ω)", 
                 load.capacitance_pf, load.resistance_ohm);
        
        // 基础传播延迟（内部延迟）
        let internal_delay = 2.0; // 2ns内部延迟
        
        // 负载相关延迟
        let load_delay = self.calculate_rc_delay(load);
        
        // 温度系数影响
        let temp_factor = 1.0 + (self.temperature - 25.0) * 0.001;
        
        let total_delay = (internal_delay + load_delay) * temp_factor;
        
        // 计算抖动
        let jitter = self.calculate_jitter(total_delay);
        
        TimingMeasurement {
            delay_ns: total_delay,
            jitter_ps: jitter,
            accuracy_percent: 5.0, // 5%精度
            temperature: self.temperature,
            load_condition: load.clone(),
        }
    }

    /// 测量上升时间
    pub fn measure_rise_time(&self, load: &LoadCondition) -> TimingMeasurement {
        rprintln!("测量上升时间 (10%-90%)");
        
        // 基于RC时间常数计算
        let time_constant = self.calculate_time_constant(load);
        let rise_time = time_constant * 2.2; // 10%-90%上升时间
        
        // 考虑驱动器的压摆率限制
        let slew_rate_limit = load.voltage_swing_v / 0.5; // V/ns
        let slew_limited_time = load.voltage_swing_v * 0.8 / slew_rate_limit;
        
        let actual_rise_time = rise_time.max(slew_limited_time);
        
        TimingMeasurement {
            delay_ns: actual_rise_time,
            jitter_ps: self.calculate_jitter(actual_rise_time),
            accuracy_percent: 3.0,
            temperature: self.temperature,
            load_condition: load.clone(),
        }
    }

    /// 测量下降时间
    pub fn measure_fall_time(&self, load: &LoadCondition) -> TimingMeasurement {
        rprintln!("测量下降时间 (90%-10%)");
        
        // 下降时间通常比上升时间快（NMOS更强）
        let rise_time = self.measure_rise_time(load);
        let fall_time = rise_time.delay_ns * 0.7;
        
        TimingMeasurement {
            delay_ns: fall_time,
            jitter_ps: self.calculate_jitter(fall_time),
            accuracy_percent: 3.0,
            temperature: self.temperature,
            load_condition: load.clone(),
        }
    }

    /// 测量建立时间
    pub fn measure_setup_time(&self) -> TimingMeasurement {
        rprintln!("测量建立时间");
        
        // 建立时间主要由内部时序决定
        let setup_time = 1.0 + (self.system_clock_mhz / 100.0); // 基础1ns + 频率相关
        
        let load = LoadCondition {
            capacitance_pf: 5.0, // 内部负载
            resistance_ohm: 1000.0,
            voltage_swing_v: 3.3,
        };
        
        TimingMeasurement {
            delay_ns: setup_time,
            jitter_ps: setup_time * 50.0, // 5%抖动
            accuracy_percent: 10.0,
            temperature: self.temperature,
            load_condition: load,
        }
    }

    /// 测量保持时间
    pub fn measure_hold_time(&self) -> TimingMeasurement {
        rprintln!("测量保持时间");
        
        // 保持时间通常很小或为负值
        let hold_time = 0.5;
        
        let load = LoadCondition {
            capacitance_pf: 5.0,
            resistance_ohm: 1000.0,
            voltage_swing_v: 3.3,
        };
        
        TimingMeasurement {
            delay_ns: hold_time,
            jitter_ps: 100.0, // 固定抖动
            accuracy_percent: 20.0,
            temperature: self.temperature,
            load_condition: load,
        }
    }

    /// 频率响应测试
    pub fn frequency_response_test(&self, frequencies: &[f32]) -> heapless::Vec<FrequencyResponse, 16> {
        rprintln!("频率响应测试");
        
        let mut responses = heapless::Vec::new();
        
        for &freq_mhz in frequencies {
            let response = self.measure_frequency_response(freq_mhz);
            let _ = responses.push(response);
        }
        
        responses
    }

    /// 眼图分析
    pub fn eye_diagram_analysis(&self, data_rate_mbps: f32, pattern: &[u8]) -> EyeDiagramAnalysis {
        rprintln!("眼图分析 (数据速率: {:.1}Mbps)", data_rate_mbps);
        
        // 模拟眼图分析
        let bit_period_ns = 1000.0 / data_rate_mbps;
        let rise_time = self.estimate_rise_time_for_rate(data_rate_mbps);
        
        // 眼图参数计算
        let eye_width = bit_period_ns - 2.0 * rise_time;
        let eye_height = 3.3 * 0.8; // 80%的电压摆幅
        
        // 抖动分析
        let pattern_jitter = self.analyze_pattern_jitter(pattern);
        let thermal_jitter = self.calculate_thermal_jitter();
        let total_jitter = (pattern_jitter * pattern_jitter + thermal_jitter * thermal_jitter).sqrt();
        
        // 质量因子
        let quality_factor = eye_height / (total_jitter * 1e-3); // 转换ps到ns
        
        EyeDiagramAnalysis {
            eye_height_v: eye_height,
            eye_width_ns: eye_width,
            jitter_rms_ps: total_jitter,
            crossing_percentage: 50.0, // 理想50%交叉点
            quality_factor,
        }
    }

    /// 时钟抖动分析
    pub fn clock_jitter_analysis(&self, clock_freq_mhz: f32) -> ClockJitterAnalysis {
        rprintln!("时钟抖动分析 (频率: {:.1}MHz)", clock_freq_mhz);
        
        // 不同类型的抖动源
        let thermal_jitter = self.calculate_thermal_jitter();
        let phase_noise_jitter = self.calculate_phase_noise_jitter(clock_freq_mhz);
        let supply_noise_jitter = self.calculate_supply_noise_jitter();
        
        // RMS抖动合成
        let total_rms_jitter = (thermal_jitter * thermal_jitter + 
                               phase_noise_jitter * phase_noise_jitter + 
                               supply_noise_jitter * supply_noise_jitter).sqrt();
        
        // 峰峰值抖动（通常是RMS的6-8倍）
        let peak_to_peak_jitter = total_rms_jitter * 7.0;
        
        ClockJitterAnalysis {
            rms_jitter_ps: total_rms_jitter,
            peak_to_peak_jitter_ps: peak_to_peak_jitter,
            thermal_component_ps: thermal_jitter,
            phase_noise_component_ps: phase_noise_jitter,
            supply_noise_component_ps: supply_noise_jitter,
            frequency_mhz: clock_freq_mhz,
        }
    }

    /// 时序裕量分析
    pub fn timing_margin_analysis(&self, target_frequency_mhz: f32) -> TimingMarginAnalysis {
        rprintln!("时序裕量分析 (目标频率: {:.1}MHz)", target_frequency_mhz);
        
        let period_ns = 1000.0 / target_frequency_mhz;
        
        // 关键路径延迟
        let logic_delay = self.estimate_logic_delay();
        let routing_delay = self.estimate_routing_delay();
        let setup_time = self.measure_setup_time().delay_ns;
        
        let total_delay = logic_delay + routing_delay + setup_time;
        
        // 时钟偏斜和抖动
        let clock_skew = self.estimate_clock_skew();
        let clock_jitter = self.clock_jitter_analysis(target_frequency_mhz).rms_jitter_ps / 1000.0;
        
        // 计算裕量
        let setup_margin = period_ns - total_delay - clock_skew - clock_jitter;
        let hold_margin = self.measure_hold_time().delay_ns - clock_skew - clock_jitter;
        
        TimingMarginAnalysis {
            target_period_ns: period_ns,
            total_delay_ns: total_delay,
            setup_margin_ns: setup_margin,
            hold_margin_ns: hold_margin,
            clock_skew_ns: clock_skew,
            clock_jitter_ns: clock_jitter,
            passes_timing: setup_margin > 0.0 && hold_margin > 0.0,
        }
    }

    // 私有辅助方法
    fn calculate_rc_delay(&self, load: &LoadCondition) -> f32 {
        let time_constant = load.resistance_ohm * load.capacitance_pf * 1e-12;
        time_constant * 1e9 // 转换为纳秒
    }

    fn calculate_time_constant(&self, load: &LoadCondition) -> f32 {
        // 考虑驱动器输出阻抗
        let total_resistance = load.resistance_ohm + 50.0; // 50Ω驱动阻抗
        total_resistance * load.capacitance_pf * 1e-3 // ns
    }

    fn calculate_jitter(&self, delay_ns: f32) -> f32 {
        // 抖动通常与延迟成正比
        let base_jitter = 50.0; // 50ps基础抖动
        let delay_dependent = delay_ns * 10.0; // 每ns增加10ps抖动
        base_jitter + delay_dependent
    }

    fn measure_frequency_response(&self, frequency_mhz: f32) -> FrequencyResponse {
        // 简化的频率响应模型
        let cutoff_freq = 100.0; // 100MHz截止频率
        let amplitude_db = -20.0 * (frequency_mhz / cutoff_freq).log10();
        let phase_deg = -90.0 * (frequency_mhz / cutoff_freq).atan() / (core::f32::consts::PI / 2.0);
        
        // 失真计算
        let distortion = if frequency_mhz > cutoff_freq {
            (frequency_mhz / cutoff_freq - 1.0) * 5.0
        } else {
            0.1
        };
        
        FrequencyResponse {
            frequency_mhz,
            amplitude_db,
            phase_deg,
            distortion_percent: distortion,
        }
    }

    fn estimate_rise_time_for_rate(&self, data_rate_mbps: f32) -> f32 {
        // 经验公式：上升时间约为位周期的20%
        let bit_period_ns = 1000.0 / data_rate_mbps;
        bit_period_ns * 0.2
    }

    fn analyze_pattern_jitter(&self, pattern: &[u8]) -> f32 {
        // 分析数据模式引起的抖动
        let mut transitions = 0;
        for i in 1..pattern.len() {
            if pattern[i] != pattern[i-1] {
                transitions += 1;
            }
        }
        
        // 更多跳变导致更多抖动
        let transition_ratio = transitions as f32 / pattern.len() as f32;
        50.0 + transition_ratio * 100.0 // ps
    }

    fn calculate_thermal_jitter(&self) -> f32 {
        // 热噪声抖动
        let base_thermal = 20.0; // 20ps @ 25°C
        let temp_factor = ((self.temperature + 273.15) / 298.15).sqrt();
        base_thermal * temp_factor
    }

    fn calculate_phase_noise_jitter(&self, frequency_mhz: f32) -> f32 {
        // 相位噪声转换为时域抖动
        let phase_noise_dbc = -100.0; // -100dBc/Hz @ 1kHz offset
        let jitter_ps = 10.0_f32.powf(phase_noise_dbc / 20.0) / (2.0 * core::f32::consts::PI * frequency_mhz) * 1e6;
        jitter_ps
    }

    fn calculate_supply_noise_jitter(&self) -> f32 {
        // 电源噪声引起的抖动
        let supply_noise_mv = 10.0; // 10mV电源噪声
        let sensitivity_ps_per_mv = 5.0; // 5ps/mV敏感度
        supply_noise_mv * sensitivity_ps_per_mv
    }

    fn estimate_logic_delay(&self) -> f32 {
        // 估算逻辑延迟
        5.0 // 5ns典型逻辑延迟
    }

    fn estimate_routing_delay(&self) -> f32 {
        // 估算布线延迟
        2.0 // 2ns典型布线延迟
    }

    fn estimate_clock_skew(&self) -> f32 {
        // 估算时钟偏斜
        0.5 // 500ps时钟偏斜
    }
}

/// 时钟抖动分析结果
#[derive(Debug, Clone)]
pub struct ClockJitterAnalysis {
    pub rms_jitter_ps: f32,
    pub peak_to_peak_jitter_ps: f32,
    pub thermal_component_ps: f32,
    pub phase_noise_component_ps: f32,
    pub supply_noise_component_ps: f32,
    pub frequency_mhz: f32,
}

/// 时序裕量分析结果
#[derive(Debug, Clone)]
pub struct TimingMarginAnalysis {
    pub target_period_ns: f32,
    pub total_delay_ns: f32,
    pub setup_margin_ns: f32,
    pub hold_margin_ns: f32,
    pub clock_skew_ns: f32,
    pub clock_jitter_ns: f32,
    pub passes_timing: bool,
}

impl ClockJitterAnalysis {
    pub fn jitter_budget_analysis(&self) -> JitterBudget {
        let total_budget = self.peak_to_peak_jitter_ps;
        
        JitterBudget {
            total_budget_ps: total_budget,
            thermal_allocation_ps: self.thermal_component_ps * 7.0, // 峰峰值
            phase_noise_allocation_ps: self.phase_noise_component_ps * 7.0,
            supply_noise_allocation_ps: self.supply_noise_component_ps * 7.0,
            margin_ps: total_budget * 0.2, // 20%裕量
        }
    }
}

/// 抖动预算分析
#[derive(Debug, Clone)]
pub struct JitterBudget {
    pub total_budget_ps: f32,
    pub thermal_allocation_ps: f32,
    pub phase_noise_allocation_ps: f32,
    pub supply_noise_allocation_ps: f32,
    pub margin_ps: f32,
}