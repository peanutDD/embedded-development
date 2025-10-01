use micromath::F32Ext;

#[cfg(feature = "rtt-log")]
use rtt_target::rprintln;

/// 功耗测试器
pub struct PowerTester {
  supply_voltage: f32,
  temperature: f32,
  measurement_duration_ms: u32,
}

/// 功耗测量结果
#[derive(Debug, Clone)]
pub struct PowerMeasurement {
  pub current_ua: f32,
  pub power_uw: f32,
  pub voltage: f32,
  pub duration_ms: u32,
  pub temperature: f32,
  pub test_condition: TestCondition,
}

/// 测试条件
#[derive(Debug, Clone)]
pub struct TestCondition {
  pub pin_state: PinState,
  pub load_type: LoadType,
  pub switching_frequency_hz: f32,
  pub duty_cycle_percent: f32,
}

/// 引脚状态
#[derive(Debug, Clone, Copy)]
pub enum PinState {
  Static(bool),  // 静态高/低
  Switching,     // 切换状态
  HighImpedance, // 高阻态
  Analog,        // 模拟模式
}

/// 负载类型
#[derive(Debug, Clone, Copy)]
pub enum LoadType {
  NoLoad,                        // 无负载
  Resistive(f32),                // 阻性负载 (Ω)
  Capacitive(f32),               // 容性负载 (pF)
  ResistiveCapacitive(f32, f32), // RC负载 (Ω, pF)
  Led(f32),                      // LED负载 (mA)
}

/// 功耗分析结果
#[derive(Debug, Clone)]
pub struct PowerAnalysis {
  pub static_power_uw: f32,
  pub dynamic_power_uw: f32,
  pub leakage_power_nw: f32,
  pub switching_power_uw: f32,
  pub total_power_uw: f32,
  pub efficiency_percent: f32,
}

/// 温度功耗特性
#[derive(Debug, Clone)]
pub struct TemperaturePowerProfile {
  pub temperature: f32,
  pub leakage_current_na: f32,
  pub threshold_voltage_v: f32,
  pub switching_energy_pj: f32,
  pub thermal_resistance_c_per_w: f32,
}

impl PowerTester {
  /// 创建新的功耗测试器
  pub fn new(supply_voltage: f32) -> Self {
    Self {
      supply_voltage,
      temperature: 25.0,
      measurement_duration_ms: 1000, // 默认1秒测量
    }
  }

  /// 设置测试温度
  pub fn set_temperature(&mut self, temperature: f32) {
    self.temperature = temperature;
    rprintln!("功耗测试温度设置为: {:.1}°C", temperature);
  }

  /// 设置测量持续时间
  pub fn set_measurement_duration(&mut self, duration_ms: u32) {
    self.measurement_duration_ms = duration_ms;
    rprintln!("测量持续时间设置为: {}ms", duration_ms);
  }

  /// 测量静态功耗
  pub fn measure_static_power(&self, pin_state: bool) -> PowerMeasurement {
    rprintln!(
      "测量静态功耗 (引脚状态: {})",
      if pin_state { "高" } else { "低" }
    );

    // 基础漏电流
    let base_leakage = self.calculate_leakage_current();

    // 输出级静态电流
    let output_current = if pin_state {
      // 高电平时的静态电流
      0.1e-6 // 100nA
    } else {
      // 低电平时的静态电流
      0.05e-6 // 50nA
    };

    let total_current = base_leakage + output_current;
    let power = total_current * self.supply_voltage * 1e6; // 转换为μW

    PowerMeasurement {
      current_ua: total_current * 1e6,
      power_uw: power,
      voltage: self.supply_voltage,
      duration_ms: self.measurement_duration_ms,
      temperature: self.temperature,
      test_condition: TestCondition {
        pin_state: PinState::Static(pin_state),
        load_type: LoadType::NoLoad,
        switching_frequency_hz: 0.0,
        duty_cycle_percent: if pin_state { 100.0 } else { 0.0 },
      },
    }
  }

  /// 测量动态功耗
  pub fn measure_dynamic_power(&self, frequency_hz: f32, load: LoadType) -> PowerMeasurement {
    rprintln!("测量动态功耗 (频率: {:.1}Hz)", frequency_hz);

    // 计算切换功耗
    let switching_power = self.calculate_switching_power(frequency_hz, &load);

    // 计算短路功耗
    let short_circuit_power = self.calculate_short_circuit_power(frequency_hz);

    // 静态功耗基线
    let static_power = self.measure_static_power(false).power_uw;

    let total_power = static_power + switching_power + short_circuit_power;
    let total_current = total_power / self.supply_voltage;

    PowerMeasurement {
      current_ua: total_current,
      power_uw: total_power,
      voltage: self.supply_voltage,
      duration_ms: self.measurement_duration_ms,
      temperature: self.temperature,
      test_condition: TestCondition {
        pin_state: PinState::Switching,
        load_type: load,
        switching_frequency_hz: frequency_hz,
        duty_cycle_percent: 50.0,
      },
    }
  }

  /// 测量不同负载下的功耗
  pub fn measure_load_power(&self, load: LoadType) -> PowerMeasurement {
    rprintln!("测量负载功耗");

    let load_current = match load {
      LoadType::NoLoad => 0.0,
      LoadType::Resistive(r) => self.supply_voltage / r,
      LoadType::Led(i_ma) => i_ma * 1e-3,
      LoadType::Capacitive(_) => 0.0, // 静态时无电流
      LoadType::ResistiveCapacitive(r, _) => self.supply_voltage / r,
    };

    // 驱动器损耗
    let driver_loss = self.calculate_driver_loss(load_current);

    // 负载功耗
    let load_power = load_current * self.supply_voltage * 1e6; // μW

    // 总功耗
    let total_power = load_power + driver_loss;
    let total_current = total_power / self.supply_voltage;

    PowerMeasurement {
      current_ua: total_current,
      power_uw: total_power,
      voltage: self.supply_voltage,
      duration_ms: self.measurement_duration_ms,
      temperature: self.temperature,
      test_condition: TestCondition {
        pin_state: PinState::Static(true),
        load_type: load,
        switching_frequency_hz: 0.0,
        duty_cycle_percent: 100.0,
      },
    }
  }

  /// 功耗分析
  pub fn analyze_power_consumption(&self, frequency_hz: f32, load: LoadType) -> PowerAnalysis {
    rprintln!("功耗分析 (频率: {:.1}Hz)", frequency_hz);

    // 各种功耗组件
    let static_measurement = self.measure_static_power(false);
    let dynamic_measurement = self.measure_dynamic_power(frequency_hz, load);
    let load_measurement = self.measure_load_power(load);

    let static_power = static_measurement.power_uw;
    let switching_power = self.calculate_switching_power(frequency_hz, &load);
    let leakage_power = self.calculate_leakage_current() * self.supply_voltage * 1e9; // nW

    let total_power = dynamic_measurement.power_uw;
    let dynamic_power = total_power - static_power;

    // 效率计算
    let useful_power = match load {
      LoadType::Led(i_ma) => i_ma * 1e-3 * 2.0 * 1e6, // 假设LED压降2V
      LoadType::Resistive(r) => (self.supply_voltage * self.supply_voltage / r) * 1e6,
      _ => 0.0,
    };

    let efficiency = if total_power > 0.0 {
      (useful_power / total_power) * 100.0
    } else {
      0.0
    };

    PowerAnalysis {
      static_power_uw: static_power,
      dynamic_power_uw: dynamic_power,
      leakage_power_nw: leakage_power,
      switching_power_uw: switching_power,
      total_power_uw: total_power,
      efficiency_percent: efficiency,
    }
  }

  /// 温度功耗特性分析
  pub fn temperature_power_profile(
    &mut self,
    temp_range: (f32, f32),
  ) -> heapless::Vec<TemperaturePowerProfile, 16> {
    rprintln!(
      "温度功耗特性分析 ({:.1}°C 到 {:.1}°C)",
      temp_range.0,
      temp_range.1
    );

    let mut profiles = heapless::Vec::new();

    for temp in ((temp_range.0 as i32)..=(temp_range.1 as i32)).step_by(10) {
      self.set_temperature(temp as f32);

      let profile = TemperaturePowerProfile {
        temperature: temp as f32,
        leakage_current_na: self.calculate_leakage_current() * 1e9,
        threshold_voltage_v: self.calculate_threshold_voltage(),
        switching_energy_pj: self.calculate_switching_energy() * 1e12,
        thermal_resistance_c_per_w: self.calculate_thermal_resistance(),
      };

      let _ = profiles.push(profile);
    }

    profiles
  }

  /// 功耗优化建议
  pub fn power_optimization_recommendations(
    &self,
    analysis: &PowerAnalysis,
  ) -> PowerOptimizationReport {
    rprintln!("生成功耗优化建议");

    let mut recommendations = heapless::Vec::new();

    // 静态功耗优化
    if analysis.static_power_uw > 10.0 {
      let _ = recommendations.push(OptimizationRecommendation {
        category: OptimizationCategory::Static,
        description: "考虑使用低功耗模式或关闭未使用的引脚",
        potential_savings_percent: 30.0,
        implementation_difficulty: DifficultyLevel::Easy,
      });
    }

    // 动态功耗优化
    if analysis.dynamic_power_uw > 100.0 {
      let _ = recommendations.push(OptimizationRecommendation {
        category: OptimizationCategory::Dynamic,
        description: "降低切换频率或使用较低的驱动强度",
        potential_savings_percent: 50.0,
        implementation_difficulty: DifficultyLevel::Medium,
      });
    }

    // 负载优化
    if analysis.efficiency_percent < 50.0 {
      let _ = recommendations.push(OptimizationRecommendation {
        category: OptimizationCategory::Load,
        description: "优化负载匹配或使用更高效的驱动方式",
        potential_savings_percent: 25.0,
        implementation_difficulty: DifficultyLevel::Hard,
      });
    }

    PowerOptimizationReport {
      current_power_uw: analysis.total_power_uw,
      estimated_optimized_power_uw: analysis.total_power_uw * 0.6, // 假设40%优化
      recommendations,
    }
  }

  /// 电池寿命估算
  pub fn battery_life_estimation(
    &self,
    battery_capacity_mah: f32,
    analysis: &PowerAnalysis,
  ) -> BatteryLifeEstimate {
    rprintln!("电池寿命估算 (容量: {:.0}mAh)", battery_capacity_mah);

    let average_current_ma = analysis.total_power_uw / (self.supply_voltage * 1000.0);

    // 考虑电池效率和自放电
    let battery_efficiency = 0.85; // 85%效率
    let self_discharge_ma = 0.01; // 10μA自放电

    let effective_current = average_current_ma + self_discharge_ma;
    let effective_capacity = battery_capacity_mah * battery_efficiency;

    let life_hours = effective_capacity / effective_current;
    let life_days = life_hours / 24.0;

    BatteryLifeEstimate {
      average_current_ma,
      effective_capacity_mah: effective_capacity,
      estimated_life_hours: life_hours,
      estimated_life_days: life_days,
      confidence_level: if average_current_ma < 1.0 {
        ConfidenceLevel::High
      } else {
        ConfidenceLevel::Medium
      },
    }
  }

  // 私有计算方法
  fn calculate_leakage_current(&self) -> f32 {
    // 温度相关的漏电流
    let base_leakage = 1e-9; // 1nA @ 25°C
    let temp_factor = 2.0_f32.powf((self.temperature - 25.0) / 10.0);
    base_leakage * temp_factor
  }

  fn calculate_switching_power(&self, frequency_hz: f32, load: &LoadType) -> f32 {
    let capacitance = match load {
      LoadType::Capacitive(c) => *c * 1e-12,
      LoadType::ResistiveCapacitive(_, c) => *c * 1e-12,
      _ => 10e-12, // 默认10pF
    };

    // P = C * V² * f
    capacitance * self.supply_voltage * self.supply_voltage * frequency_hz * 1e6
    // μW
  }

  fn calculate_short_circuit_power(&self, frequency_hz: f32) -> f32 {
    // 短路功耗在切换过程中产生
    let short_circuit_current = 2e-3; // 2mA短路电流
    let short_circuit_time = 1e-9; // 1ns短路时间
    let duty_cycle = short_circuit_time * frequency_hz;

    short_circuit_current * self.supply_voltage * duty_cycle * 1e6 // μW
  }

  fn calculate_driver_loss(&self, load_current: f32) -> f32 {
    // 驱动器内阻损耗
    let driver_resistance = 50.0; // 50Ω内阻
    load_current * load_current * driver_resistance * 1e6 // μW
  }

  fn calculate_threshold_voltage(&self) -> f32 {
    // 温度相关的阈值电压
    let base_threshold = self.supply_voltage * 0.5;
    let temp_coefficient = -0.002; // -2mV/°C
    base_threshold + (self.temperature - 25.0) * temp_coefficient
  }

  fn calculate_switching_energy(&self) -> f32 {
    // 单次切换能量
    let capacitance = 10e-12; // 10pF
    0.5 * capacitance * self.supply_voltage * self.supply_voltage // J
  }

  fn calculate_thermal_resistance(&self) -> f32 {
    // 热阻计算
    150.0 // 150°C/W典型值
  }
}

/// 功耗优化报告
#[derive(Debug)]
pub struct PowerOptimizationReport {
  pub current_power_uw: f32,
  pub estimated_optimized_power_uw: f32,
  pub recommendations: heapless::Vec<OptimizationRecommendation, 8>,
}

/// 优化建议
#[derive(Debug, Clone)]
pub struct OptimizationRecommendation {
  pub category: OptimizationCategory,
  pub description: &'static str,
  pub potential_savings_percent: f32,
  pub implementation_difficulty: DifficultyLevel,
}

/// 优化类别
#[derive(Debug, Clone, Copy)]
pub enum OptimizationCategory {
  Static,
  Dynamic,
  Load,
  Thermal,
}

/// 实现难度
#[derive(Debug, Clone, Copy)]
pub enum DifficultyLevel {
  Easy,
  Medium,
  Hard,
}

/// 电池寿命估算
#[derive(Debug, Clone)]
pub struct BatteryLifeEstimate {
  pub average_current_ma: f32,
  pub effective_capacity_mah: f32,
  pub estimated_life_hours: f32,
  pub estimated_life_days: f32,
  pub confidence_level: ConfidenceLevel,
}

/// 置信度等级
#[derive(Debug, Clone, Copy)]
pub enum ConfidenceLevel {
  High,
  Medium,
  Low,
}

impl PowerAnalysis {
  /// 功耗等级评估
  pub fn power_grade(&self) -> PowerGrade {
    if self.total_power_uw < 10.0 {
      PowerGrade::Excellent
    } else if self.total_power_uw < 50.0 {
      PowerGrade::Good
    } else if self.total_power_uw < 200.0 {
      PowerGrade::Average
    } else {
      PowerGrade::Poor
    }
  }

  /// 功耗分布分析
  pub fn power_distribution(&self) -> PowerDistribution {
    let total = self.total_power_uw;

    PowerDistribution {
      static_percentage: (self.static_power_uw / total) * 100.0,
      dynamic_percentage: (self.dynamic_power_uw / total) * 100.0,
      switching_percentage: (self.switching_power_uw / total) * 100.0,
      leakage_percentage: (self.leakage_power_nw * 1e-3 / total) * 100.0,
    }
  }
}

/// 功耗等级
#[derive(Debug, Clone, Copy)]
pub enum PowerGrade {
  Excellent,
  Good,
  Average,
  Poor,
}

/// 功耗分布
#[derive(Debug, Clone)]
pub struct PowerDistribution {
  pub static_percentage: f32,
  pub dynamic_percentage: f32,
  pub switching_percentage: f32,
  pub leakage_percentage: f32,
}
