# 电池优化技术

电池供电的嵌入式设备需要特殊的优化策略来最大化电池寿命。本文档详细介绍了各种电池优化技术和实现方法。

## 电池基础知识

### 常见电池类型

#### 锂离子电池 (Li-ion)
- **标称电压**: 3.7V
- **充电电压**: 4.2V
- **放电截止电压**: 3.0V
- **能量密度**: 高
- **自放电率**: 低 (2-3%/月)
- **循环寿命**: 500-1000次

#### 锂聚合物电池 (LiPo)
- **标称电压**: 3.7V
- **充电电压**: 4.2V
- **放电截止电压**: 3.0V
- **特点**: 轻薄、形状灵活
- **应用**: 便携设备、无人机

#### 镍氢电池 (NiMH)
- **标称电压**: 1.2V
- **充电电压**: 1.4V
- **放电截止电压**: 1.0V
- **自放电率**: 较高 (15-20%/月)
- **环保性**: 好

### 电池特性曲线

```rust
// 电池放电曲线数据
pub struct BatteryProfile {
    pub battery_type: BatteryType,
    pub nominal_voltage: u16,    // mV
    pub max_voltage: u16,        // mV
    pub min_voltage: u16,        // mV
    pub capacity: u16,           // mAh
    pub discharge_curve: [(u8, u16); 11], // (百分比, 电压mV)
}

#[derive(Debug, Clone, Copy)]
pub enum BatteryType {
    LiIon,
    LiPo,
    NiMH,
    Alkaline,
}

impl BatteryProfile {
    // 锂离子电池配置
    pub fn li_ion_18650() -> Self {
        Self {
            battery_type: BatteryType::LiIon,
            nominal_voltage: 3700,
            max_voltage: 4200,
            min_voltage: 3000,
            capacity: 2500,
            discharge_curve: [
                (100, 4200), (90, 4100), (80, 4000), (70, 3900), (60, 3800),
                (50, 3750), (40, 3700), (30, 3650), (20, 3500), (10, 3300), (0, 3000)
            ],
        }
    }
    
    // 锂聚合物电池配置
    pub fn lipo_1s() -> Self {
        Self {
            battery_type: BatteryType::LiPo,
            nominal_voltage: 3700,
            max_voltage: 4200,
            min_voltage: 3000,
            capacity: 1000,
            discharge_curve: [
                (100, 4200), (90, 4150), (80, 4050), (70, 3950), (60, 3850),
                (50, 3800), (40, 3750), (30, 3700), (20, 3600), (10, 3400), (0, 3000)
            ],
        }
    }
    
    // 根据电压估算电量
    pub fn voltage_to_percentage(&self, voltage_mv: u16) -> u8 {
        if voltage_mv >= self.max_voltage {
            return 100;
        }
        if voltage_mv <= self.min_voltage {
            return 0;
        }
        
        // 线性插值计算电量百分比
        for i in 0..self.discharge_curve.len() - 1 {
            let (percent_high, voltage_high) = self.discharge_curve[i];
            let (percent_low, voltage_low) = self.discharge_curve[i + 1];
            
            if voltage_mv <= voltage_high && voltage_mv >= voltage_low {
                let voltage_range = voltage_high - voltage_low;
                let percent_range = percent_high - percent_low;
                let voltage_offset = voltage_mv - voltage_low;
                
                return percent_low + ((voltage_offset * percent_range) / voltage_range) as u8;
            }
        }
        
        0
    }
    
    // 估算剩余运行时间
    pub fn estimate_runtime(&self, current_ma: u16, voltage_mv: u16) -> u32 {
        let remaining_percentage = self.voltage_to_percentage(voltage_mv);
        let remaining_capacity = (self.capacity * remaining_percentage as u16) / 100;
        
        if current_ma == 0 {
            return u32::MAX; // 无限时间
        }
        
        // 返回剩余时间（分钟）
        (remaining_capacity as u32 * 60) / current_ma as u32
    }
}
```

## 电流测量和监控

### 高精度电流测量

```rust
use stm32f4xx_hal::{
    adc::{Adc, config::AdcConfig},
    gpio::{gpioa::PA1, Analog},
};

pub struct CurrentMonitor {
    adc: Adc<stm32::ADC1>,
    current_sense_pin: PA1<Analog>,
    shunt_resistance: f32,  // 分流电阻值 (欧姆)
    amplifier_gain: f32,    // 放大器增益
    calibration_offset: i16, // 校准偏移
}

impl CurrentMonitor {
    pub fn new(
        adc: Adc<stm32::ADC1>,
        current_sense_pin: PA1<Analog>,
        shunt_resistance: f32,
        amplifier_gain: f32,
    ) -> Self {
        Self {
            adc,
            current_sense_pin,
            shunt_resistance,
            amplifier_gain,
            calibration_offset: 0,
        }
    }
    
    // 校准零点偏移
    pub fn calibrate_zero_offset(&mut self) {
        let mut sum = 0i32;
        const SAMPLES: u16 = 100;
        
        for _ in 0..SAMPLES {
            let sample: u16 = self.adc.read(&mut self.current_sense_pin).unwrap();
            sum += sample as i32;
            cortex_m::asm::delay(1000); // 1ms延时
        }
        
        self.calibration_offset = (sum / SAMPLES as i32) as i16;
    }
    
    // 读取瞬时电流 (mA)
    pub fn read_current_ma(&mut self) -> i16 {
        let raw_sample: u16 = self.adc.read(&mut self.current_sense_pin).unwrap();
        let calibrated_sample = raw_sample as i16 - self.calibration_offset;
        
        // 转换为电压 (mV)
        let voltage_mv = (calibrated_sample as f32 * 3300.0) / 4095.0;
        
        // 计算电流 (mA)
        let current_ma = (voltage_mv / self.amplifier_gain) / self.shunt_resistance;
        
        current_ma as i16
    }
    
    // 测量平均电流
    pub fn measure_average_current(&mut self, duration_ms: u32) -> i16 {
        let samples = duration_ms / 10; // 每10ms采样一次
        let mut sum = 0i32;
        
        for _ in 0..samples {
            sum += self.read_current_ma() as i32;
            cortex_m::asm::delay(10000); // 10ms延时
        }
        
        (sum / samples as i32) as i16
    }
    
    // 测量峰值电流
    pub fn measure_peak_current(&mut self, duration_ms: u32) -> i16 {
        let samples = duration_ms; // 每1ms采样一次
        let mut peak_current = 0i16;
        
        for _ in 0..samples {
            let current = self.read_current_ma();
            if current > peak_current {
                peak_current = current;
            }
            cortex_m::asm::delay(1000); // 1ms延时
        }
        
        peak_current
    }
}
```

### 电量计算器

```rust
pub struct BatteryGauge {
    battery_profile: BatteryProfile,
    current_monitor: CurrentMonitor,
    accumulated_charge: i32,    // 累积电荷 (mAs)
    last_update_time: u32,      // 上次更新时间
    state_of_charge: u8,        // 电量状态 (0-100%)
    voltage_history: [u16; 10], // 电压历史记录
    current_history: [i16; 10], // 电流历史记录
    history_index: usize,
}

impl BatteryGauge {
    pub fn new(
        battery_profile: BatteryProfile,
        current_monitor: CurrentMonitor,
    ) -> Self {
        Self {
            battery_profile,
            current_monitor,
            accumulated_charge: 0,
            last_update_time: 0,
            state_of_charge: 100,
            voltage_history: [0; 10],
            current_history: [0; 10],
            history_index: 0,
        }
    }
    
    // 更新电量状态
    pub fn update(&mut self, voltage_mv: u16, current_time_ms: u32) {
        let current_ma = self.current_monitor.read_current_ma();
        
        // 计算时间差
        let time_delta_ms = if self.last_update_time == 0 {
            0
        } else {
            current_time_ms - self.last_update_time
        };
        
        // 累积电荷变化 (库仑计数法)
        if time_delta_ms > 0 {
            let charge_delta = (current_ma as i32 * time_delta_ms as i32) / 1000; // mAs
            self.accumulated_charge += charge_delta;
        }
        
        // 更新历史记录
        self.voltage_history[self.history_index] = voltage_mv;
        self.current_history[self.history_index] = current_ma;
        self.history_index = (self.history_index + 1) % 10;
        
        // 结合电压法和库仑计数法估算电量
        let voltage_soc = self.battery_profile.voltage_to_percentage(voltage_mv);
        let coulomb_soc = self.calculate_coulomb_soc();
        
        // 加权平均
        self.state_of_charge = ((voltage_soc as u16 * 3 + coulomb_soc as u16 * 7) / 10) as u8;
        
        self.last_update_time = current_time_ms;
    }
    
    fn calculate_coulomb_soc(&self) -> u8 {
        // 基于累积电荷计算电量
        let total_capacity_mas = self.battery_profile.capacity as i32 * 3600; // mAs
        let remaining_capacity = total_capacity_mas + self.accumulated_charge;
        
        let soc = (remaining_capacity * 100) / total_capacity_mas;
        soc.max(0).min(100) as u8
    }
    
    // 获取当前电量
    pub fn get_state_of_charge(&self) -> u8 {
        self.state_of_charge
    }
    
    // 获取剩余容量 (mAh)
    pub fn get_remaining_capacity(&self) -> u16 {
        (self.battery_profile.capacity * self.state_of_charge as u16) / 100
    }
    
    // 估算剩余时间
    pub fn estimate_remaining_time(&self) -> u32 {
        let avg_current = self.get_average_current();
        if avg_current <= 0 {
            return u32::MAX; // 充电或无消耗
        }
        
        let remaining_capacity = self.get_remaining_capacity();
        (remaining_capacity as u32 * 60) / avg_current as u32 // 分钟
    }
    
    fn get_average_current(&self) -> i16 {
        let sum: i32 = self.current_history.iter().map(|&x| x as i32).sum();
        (sum / 10) as i16
    }
    
    // 获取电池健康状态
    pub fn get_battery_health(&self) -> BatteryHealth {
        let voltage_variance = self.calculate_voltage_variance();
        let current_variance = self.calculate_current_variance();
        
        if voltage_variance > 100 || current_variance > 50 {
            BatteryHealth::Poor
        } else if voltage_variance > 50 || current_variance > 25 {
            BatteryHealth::Fair
        } else {
            BatteryHealth::Good
        }
    }
    
    fn calculate_voltage_variance(&self) -> u16 {
        let avg: u32 = self.voltage_history.iter().map(|&x| x as u32).sum::<u32>() / 10;
        let variance: u32 = self.voltage_history.iter()
            .map(|&x| {
                let diff = if x as u32 > avg { x as u32 - avg } else { avg - x as u32 };
                diff * diff
            })
            .sum::<u32>() / 10;
        
        (variance as f32).sqrt() as u16
    }
    
    fn calculate_current_variance(&self) -> u16 {
        let avg: i32 = self.current_history.iter().map(|&x| x as i32).sum::<i32>() / 10;
        let variance: u32 = self.current_history.iter()
            .map(|&x| {
                let diff = (x as i32 - avg).abs() as u32;
                diff * diff
            })
            .sum::<u32>() / 10;
        
        (variance as f32).sqrt() as u16
    }
}

#[derive(Debug, PartialEq)]
pub enum BatteryHealth {
    Good,
    Fair,
    Poor,
}
```

## 动态功耗管理

### 自适应采样率

```rust
pub struct AdaptiveSampling {
    base_interval: u32,      // 基础采样间隔 (ms)
    current_interval: u32,   // 当前采样间隔 (ms)
    battery_level: u8,       // 电池电量
    activity_level: u8,      // 活动水平 (0-100)
    min_interval: u32,       // 最小间隔
    max_interval: u32,       // 最大间隔
}

impl AdaptiveSampling {
    pub fn new(base_interval: u32) -> Self {
        Self {
            base_interval,
            current_interval: base_interval,
            battery_level: 100,
            activity_level: 50,
            min_interval: base_interval / 4,
            max_interval: base_interval * 16,
        }
    }
    
    // 更新采样策略
    pub fn update_strategy(&mut self, battery_level: u8, activity_level: u8) {
        self.battery_level = battery_level;
        self.activity_level = activity_level;
        
        // 根据电池电量调整
        let battery_factor = match battery_level {
            0..=10 => 8.0,      // 极低电量：大幅降低采样率
            11..=25 => 4.0,     // 低电量：显著降低采样率
            26..=50 => 2.0,     // 中等电量：适度降低采样率
            51..=75 => 1.5,     // 较高电量：轻微降低采样率
            _ => 1.0,           // 高电量：正常采样率
        };
        
        // 根据活动水平调整
        let activity_factor = match activity_level {
            0..=20 => 4.0,      // 低活动：降低采样率
            21..=40 => 2.0,     // 中低活动：适度降低
            41..=60 => 1.0,     // 中等活动：正常采样
            61..=80 => 0.7,     // 高活动：提高采样率
            _ => 0.5,           // 极高活动：大幅提高采样率
        };
        
        // 计算新的采样间隔
        let new_interval = (self.base_interval as f32 * battery_factor * activity_factor) as u32;
        
        // 限制在合理范围内
        self.current_interval = new_interval.max(self.min_interval).min(self.max_interval);
    }
    
    // 获取当前采样间隔
    pub fn get_sampling_interval(&self) -> u32 {
        self.current_interval
    }
    
    // 获取功耗节省比例
    pub fn get_power_savings(&self) -> f32 {
        if self.current_interval > self.base_interval {
            1.0 - (self.base_interval as f32 / self.current_interval as f32)
        } else {
            0.0
        }
    }
}
```

### 智能外设管理

```rust
pub struct SmartPeripheralManager {
    peripheral_usage: [PeripheralUsage; 8],
    power_budget: u32,      // 功耗预算 (mW)
    current_consumption: u32, // 当前功耗 (mW)
}

#[derive(Clone, Copy)]
struct PeripheralUsage {
    peripheral: Peripheral,
    power_consumption: u32,  // 功耗 (mW)
    usage_frequency: u32,    // 使用频率
    last_used: u32,          // 上次使用时间
    priority: Priority,      // 优先级
    enabled: bool,           // 是否启用
}

#[derive(Clone, Copy, PartialEq)]
enum Priority {
    Critical = 0,
    High = 1,
    Medium = 2,
    Low = 3,
}

impl SmartPeripheralManager {
    pub fn new(power_budget: u32) -> Self {
        Self {
            peripheral_usage: [
                PeripheralUsage {
                    peripheral: Peripheral::USART1,
                    power_consumption: 5,
                    usage_frequency: 0,
                    last_used: 0,
                    priority: Priority::High,
                    enabled: false,
                },
                PeripheralUsage {
                    peripheral: Peripheral::I2C1,
                    power_consumption: 3,
                    usage_frequency: 0,
                    last_used: 0,
                    priority: Priority::Medium,
                    enabled: false,
                },
                PeripheralUsage {
                    peripheral: Peripheral::SPI1,
                    power_consumption: 4,
                    usage_frequency: 0,
                    last_used: 0,
                    priority: Priority::Medium,
                    enabled: false,
                },
                PeripheralUsage {
                    peripheral: Peripheral::ADC1,
                    power_consumption: 8,
                    usage_frequency: 0,
                    last_used: 0,
                    priority: Priority::High,
                    enabled: false,
                },
                PeripheralUsage {
                    peripheral: Peripheral::TIM2,
                    power_consumption: 2,
                    usage_frequency: 0,
                    last_used: 0,
                    priority: Priority::Critical,
                    enabled: true,
                },
                // ... 其他外设
                PeripheralUsage {
                    peripheral: Peripheral::GPIOA,
                    power_consumption: 1,
                    usage_frequency: 0,
                    last_used: 0,
                    priority: Priority::Critical,
                    enabled: true,
                },
                PeripheralUsage {
                    peripheral: Peripheral::GPIOB,
                    power_consumption: 1,
                    usage_frequency: 0,
                    last_used: 0,
                    priority: Priority::Low,
                    enabled: false,
                },
                PeripheralUsage {
                    peripheral: Peripheral::USART2,
                    power_consumption: 5,
                    usage_frequency: 0,
                    last_used: 0,
                    priority: Priority::Low,
                    enabled: false,
                },
            ],
            power_budget,
            current_consumption: 0,
        }
    }
    
    // 请求使用外设
    pub fn request_peripheral(&mut self, peripheral: Peripheral, current_time: u32) -> bool {
        if let Some(usage) = self.find_peripheral_mut(peripheral) {
            if usage.enabled {
                usage.usage_frequency += 1;
                usage.last_used = current_time;
                return true;
            }
            
            // 检查功耗预算
            if self.current_consumption + usage.power_consumption <= self.power_budget {
                usage.enabled = true;
                usage.usage_frequency += 1;
                usage.last_used = current_time;
                self.current_consumption += usage.power_consumption;
                return true;
            }
            
            // 尝试释放低优先级外设
            if self.try_free_low_priority_peripherals(usage.power_consumption) {
                usage.enabled = true;
                usage.usage_frequency += 1;
                usage.last_used = current_time;
                self.current_consumption += usage.power_consumption;
                return true;
            }
        }
        
        false
    }
    
    // 释放外设
    pub fn release_peripheral(&mut self, peripheral: Peripheral) {
        if let Some(usage) = self.find_peripheral_mut(peripheral) {
            if usage.enabled && usage.priority != Priority::Critical {
                usage.enabled = false;
                self.current_consumption -= usage.power_consumption;
            }
        }
    }
    
    // 自动管理外设（定期调用）
    pub fn auto_manage(&mut self, current_time: u32) {
        const IDLE_TIMEOUT: u32 = 30000; // 30秒超时
        
        for usage in &mut self.peripheral_usage {
            if usage.enabled && usage.priority != Priority::Critical {
                // 检查是否长时间未使用
                if current_time - usage.last_used > IDLE_TIMEOUT {
                    usage.enabled = false;
                    self.current_consumption -= usage.power_consumption;
                }
            }
        }
    }
    
    // 调整功耗预算
    pub fn adjust_power_budget(&mut self, new_budget: u32) {
        self.power_budget = new_budget;
        
        // 如果当前功耗超出预算，关闭低优先级外设
        while self.current_consumption > self.power_budget {
            if !self.disable_lowest_priority_peripheral() {
                break; // 无法进一步降低功耗
            }
        }
    }
    
    fn find_peripheral_mut(&mut self, peripheral: Peripheral) -> Option<&mut PeripheralUsage> {
        self.peripheral_usage.iter_mut()
            .find(|usage| usage.peripheral as u32 == peripheral as u32)
    }
    
    fn try_free_low_priority_peripherals(&mut self, required_power: u32) -> bool {
        let mut freed_power = 0;
        
        // 按优先级从低到高排序
        let mut indices: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
        indices.sort_by(|&a, &b| {
            self.peripheral_usage[b].priority.cmp(&self.peripheral_usage[a].priority)
        });
        
        for &i in &indices {
            let usage = &mut self.peripheral_usage[i];
            if usage.enabled && usage.priority != Priority::Critical {
                usage.enabled = false;
                freed_power += usage.power_consumption;
                self.current_consumption -= usage.power_consumption;
                
                if freed_power >= required_power {
                    return true;
                }
            }
        }
        
        false
    }
    
    fn disable_lowest_priority_peripheral(&mut self) -> bool {
        let mut lowest_priority = Priority::Critical;
        let mut target_index = None;
        
        for (i, usage) in self.peripheral_usage.iter().enumerate() {
            if usage.enabled && usage.priority > lowest_priority {
                lowest_priority = usage.priority;
                target_index = Some(i);
            }
        }
        
        if let Some(index) = target_index {
            let usage = &mut self.peripheral_usage[index];
            usage.enabled = false;
            self.current_consumption -= usage.power_consumption;
            return true;
        }
        
        false
    }
    
    // 获取功耗统计
    pub fn get_power_statistics(&self) -> PowerStatistics {
        PowerStatistics {
            total_budget: self.power_budget,
            current_consumption: self.current_consumption,
            utilization: (self.current_consumption * 100) / self.power_budget,
            active_peripherals: self.peripheral_usage.iter()
                .filter(|usage| usage.enabled)
                .count() as u8,
        }
    }
}

#[derive(Debug)]
pub struct PowerStatistics {
    pub total_budget: u32,
    pub current_consumption: u32,
    pub utilization: u32,
    pub active_peripherals: u8,
}
```

## 电池保护机制

### 过放保护

```rust
pub struct BatteryProtection {
    voltage_monitor: VoltageMonitor,
    protection_levels: ProtectionLevels,
    protection_state: ProtectionState,
    recovery_hysteresis: u16, // 恢复滞后电压 (mV)
}

#[derive(Clone, Copy)]
struct ProtectionLevels {
    critical_voltage: u16,    // 临界电压 (mV)
    warning_voltage: u16,     // 警告电压 (mV)
    shutdown_voltage: u16,    // 关机电压 (mV)
    recovery_voltage: u16,    // 恢复电压 (mV)
}

#[derive(Debug, PartialEq)]
enum ProtectionState {
    Normal,
    Warning,
    Critical,
    Shutdown,
}

impl BatteryProtection {
    pub fn new(voltage_monitor: VoltageMonitor) -> Self {
        Self {
            voltage_monitor,
            protection_levels: ProtectionLevels {
                critical_voltage: 3200,  // 3.2V
                warning_voltage: 3400,   // 3.4V
                shutdown_voltage: 3000,  // 3.0V
                recovery_voltage: 3600,  // 3.6V
            },
            protection_state: ProtectionState::Normal,
            recovery_hysteresis: 200, // 200mV滞后
        }
    }
    
    // 检查电池保护状态
    pub fn check_protection(&mut self) -> ProtectionAction {
        let voltage = self.voltage_monitor.read_supply_voltage();
        let previous_state = self.protection_state;
        
        // 状态转换逻辑
        self.protection_state = match self.protection_state {
            ProtectionState::Normal => {
                if voltage <= self.protection_levels.warning_voltage {
                    ProtectionState::Warning
                } else {
                    ProtectionState::Normal
                }
            }
            ProtectionState::Warning => {
                if voltage <= self.protection_levels.critical_voltage {
                    ProtectionState::Critical
                } else if voltage >= self.protection_levels.warning_voltage + self.recovery_hysteresis {
                    ProtectionState::Normal
                } else {
                    ProtectionState::Warning
                }
            }
            ProtectionState::Critical => {
                if voltage <= self.protection_levels.shutdown_voltage {
                    ProtectionState::Shutdown
                } else if voltage >= self.protection_levels.critical_voltage + self.recovery_hysteresis {
                    ProtectionState::Warning
                } else {
                    ProtectionState::Critical
                }
            }
            ProtectionState::Shutdown => {
                if voltage >= self.protection_levels.recovery_voltage {
                    ProtectionState::Normal
                } else {
                    ProtectionState::Shutdown
                }
            }
        };
        
        // 根据状态变化返回相应动作
        match (previous_state, self.protection_state) {
            (_, ProtectionState::Warning) if previous_state != ProtectionState::Warning => {
                ProtectionAction::EnablePowerSaving
            }
            (_, ProtectionState::Critical) if previous_state != ProtectionState::Critical => {
                ProtectionAction::EnterEmergencyMode
            }
            (_, ProtectionState::Shutdown) if previous_state != ProtectionState::Shutdown => {
                ProtectionAction::ForceShutdown
            }
            (ProtectionState::Shutdown, ProtectionState::Normal) => {
                ProtectionAction::SystemRecovery
            }
            _ => ProtectionAction::None,
        }
    }
    
    // 获取当前保护状态
    pub fn get_protection_state(&self) -> &ProtectionState {
        &self.protection_state
    }
    
    // 设置保护参数
    pub fn set_protection_levels(&mut self, levels: ProtectionLevels) {
        self.protection_levels = levels;
    }
}

#[derive(Debug, PartialEq)]
pub enum ProtectionAction {
    None,
    EnablePowerSaving,
    EnterEmergencyMode,
    ForceShutdown,
    SystemRecovery,
}
```

### 温度保护

```rust
pub struct ThermalProtection {
    temperature_sensor: TemperatureSensor,
    thermal_limits: ThermalLimits,
    thermal_state: ThermalState,
}

#[derive(Clone, Copy)]
struct ThermalLimits {
    warning_temp: i16,      // 警告温度 (°C * 10)
    critical_temp: i16,     // 临界温度 (°C * 10)
    shutdown_temp: i16,     // 关机温度 (°C * 10)
    recovery_temp: i16,     // 恢复温度 (°C * 10)
}

#[derive(Debug, PartialEq)]
enum ThermalState {
    Normal,
    Warning,
    Critical,
    Shutdown,
}

pub struct TemperatureSensor {
    adc: Adc<stm32::ADC1>,
}

impl TemperatureSensor {
    pub fn new(adc: Adc<stm32::ADC1>) -> Self {
        Self { adc }
    }
    
    // 读取内部温度传感器
    pub fn read_temperature(&mut self) -> i16 {
        // 启用内部温度传感器
        unsafe {
            let adc = &(*stm32::ADC1::ptr());
            adc.ccr.modify(|_, w| w.tsvrefe().set_bit());
        }
        
        // 读取温度传感器ADC值
        let temp_sample = 1800u16; // 模拟读取值
        
        // 转换为温度 (°C * 10)
        // STM32F4内部温度传感器转换公式
        let v25 = 760; // 25°C时的电压 (mV)
        let avg_slope = 25; // 平均斜率 (mV/°C * 10)
        
        let voltage_mv = (temp_sample * 3300) / 4095;
        let temperature = 250 + ((v25 as i32 - voltage_mv as i32) * 10) / avg_slope as i32;
        
        temperature as i16
    }
}

impl ThermalProtection {
    pub fn new(temperature_sensor: TemperatureSensor) -> Self {
        Self {
            temperature_sensor,
            thermal_limits: ThermalLimits {
                warning_temp: 600,   // 60°C
                critical_temp: 750,  // 75°C
                shutdown_temp: 850,  // 85°C
                recovery_temp: 500,  // 50°C
            },
            thermal_state: ThermalState::Normal,
        }
    }
    
    // 检查热保护状态
    pub fn check_thermal_protection(&mut self) -> ThermalAction {
        let temperature = self.temperature_sensor.read_temperature();
        let previous_state = self.thermal_state;
        
        // 状态转换逻辑
        self.thermal_state = match self.thermal_state {
            ThermalState::Normal => {
                if temperature >= self.thermal_limits.warning_temp {
                    ThermalState::Warning
                } else {
                    ThermalState::Normal
                }
            }
            ThermalState::Warning => {
                if temperature >= self.thermal_limits.critical_temp {
                    ThermalState::Critical
                } else if temperature <= self.thermal_limits.warning_temp - 50 {
                    ThermalState::Normal
                } else {
                    ThermalState::Warning
                }
            }
            ThermalState::Critical => {
                if temperature >= self.thermal_limits.shutdown_temp {
                    ThermalState::Shutdown
                } else if temperature <= self.thermal_limits.critical_temp - 50 {
                    ThermalState::Warning
                } else {
                    ThermalState::Critical
                }
            }
            ThermalState::Shutdown => {
                if temperature <= self.thermal_limits.recovery_temp {
                    ThermalState::Normal
                } else {
                    ThermalState::Shutdown
                }
            }
        };
        
        // 根据状态变化返回相应动作
        match (previous_state, self.thermal_state) {
            (_, ThermalState::Warning) if previous_state != ThermalState::Warning => {
                ThermalAction::ReducePerformance
            }
            (_, ThermalState::Critical) if previous_state != ThermalState::Critical => {
                ThermalAction::ThrottleCPU
            }
            (_, ThermalState::Shutdown) if previous_state != ThermalState::Shutdown => {
                ThermalAction::EmergencyShutdown
            }
            _ => ThermalAction::None,
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum ThermalAction {
    None,
    ReducePerformance,
    ThrottleCPU,
    EmergencyShutdown,
}
```

## 综合电池管理系统

### 完整的电池管理系统

```rust
pub struct ComprehensiveBatteryManager {
    battery_gauge: BatteryGauge,
    battery_protection: BatteryProtection,
    thermal_protection: ThermalProtection,
    adaptive_sampling: AdaptiveSampling,
    peripheral_manager: SmartPeripheralManager,
    system_state: SystemState,
    emergency_data: EmergencyData,
}

#[derive(Debug, PartialEq)]
enum SystemState {
    Normal,
    PowerSaving,
    Emergency,
    Shutdown,
}

struct EmergencyData {
    last_voltage: u16,
    last_temperature: i16,
    last_soc: u8,
    shutdown_reason: ShutdownReason,
}

#[derive(Debug, Clone, Copy)]
enum ShutdownReason {
    None,
    LowVoltage,
    HighTemperature,
    UserRequest,
    SystemError,
}

impl ComprehensiveBatteryManager {
    pub fn new(
        battery_gauge: BatteryGauge,
        battery_protection: BatteryProtection,
        thermal_protection: ThermalProtection,
        adaptive_sampling: AdaptiveSampling,
        peripheral_manager: SmartPeripheralManager,
    ) -> Self {
        Self {
            battery_gauge,
            battery_protection,
            thermal_protection,
            adaptive_sampling,
            peripheral_manager,
            system_state: SystemState::Normal,
            emergency_data: EmergencyData {
                last_voltage: 0,
                last_temperature: 0,
                last_soc: 0,
                shutdown_reason: ShutdownReason::None,
            },
        }
    }
    
    // 主更新循环
    pub fn update(&mut self, voltage_mv: u16, current_time_ms: u32) -> SystemAction {
        // 更新电池状态
        self.battery_gauge.update(voltage_mv, current_time_ms);
        let soc = self.battery_gauge.get_state_of_charge();
        
        // 检查电池保护
        let protection_action = self.battery_protection.check_protection();
        
        // 检查热保护
        let thermal_action = self.thermal_protection.check_thermal_protection();
        
        // 更新自适应采样
        let activity_level = self.estimate_activity_level();
        self.adaptive_sampling.update_strategy(soc, activity_level);
        
        // 自动管理外设
        self.peripheral_manager.auto_manage(current_time_ms);
        
        // 保存紧急数据
        self.emergency_data.last_voltage = voltage_mv;
        self.emergency_data.last_soc = soc;
        
        // 综合决策
        self.make_system_decision(protection_action, thermal_action)
    }
    
    fn estimate_activity_level(&self) -> u8 {
        // 基于外设使用情况估算活动水平
        let stats = self.peripheral_manager.get_power_statistics();
        (stats.utilization as u8).min(100)
    }
    
    fn make_system_decision(
        &mut self,
        protection_action: ProtectionAction,
        thermal_action: ThermalAction,
    ) -> SystemAction {
        // 优先处理紧急情况
        match (protection_action, thermal_action) {
            (ProtectionAction::ForceShutdown, _) => {
                self.emergency_data.shutdown_reason = ShutdownReason::LowVoltage;
                self.system_state = SystemState::Shutdown;
                return SystemAction::EmergencyShutdown;
            }
            (_, ThermalAction::EmergencyShutdown) => {
                self.emergency_data.shutdown_reason = ShutdownReason::HighTemperature;
                self.system_state = SystemState::Shutdown;
                return SystemAction::EmergencyShutdown;
            }
            _ => {}
        }
        
        // 处理其他情况
        let new_state = match (&self.system_state, protection_action, thermal_action) {
            (SystemState::Normal, ProtectionAction::EnablePowerSaving, _) |
            (SystemState::Normal, _, ThermalAction::ReducePerformance) => {
                SystemState::PowerSaving
            }
            (SystemState::PowerSaving, ProtectionAction::EnterEmergencyMode, _) |
            (SystemState::PowerSaving, _, ThermalAction::ThrottleCPU) => {
                SystemState::Emergency
            }
            (SystemState::Emergency, ProtectionAction::None, ThermalAction::None) => {
                SystemState::PowerSaving
            }
            (SystemState::PowerSaving, ProtectionAction::None, ThermalAction::None) => {
                SystemState::Normal
            }
            _ => self.system_state.clone(),
        };
        
        let action = if new_state != self.system_state {
            match new_state {
                SystemState::PowerSaving => SystemAction::EnterPowerSaving,
                SystemState::Emergency => SystemAction::EnterEmergencyMode,
                SystemState::Normal => SystemAction::ResumeNormal,
                SystemState::Shutdown => SystemAction::EmergencyShutdown,
            }
        } else {
            SystemAction::None
        };
        
        self.system_state = new_state;
        action
    }
    
    // 获取系统状态报告
    pub fn get_status_report(&self) -> BatteryStatusReport {
        BatteryStatusReport {
            state_of_charge: self.battery_gauge.get_state_of_charge(),
            remaining_time: self.battery_gauge.estimate_remaining_time(),
            battery_health: self.battery_gauge.get_battery_health(),
            system_state: self.system_state.clone(),
            power_statistics: self.peripheral_manager.get_power_statistics(),
            sampling_interval: self.adaptive_sampling.get_sampling_interval(),
            power_savings: self.adaptive_sampling.get_power_savings(),
        }
    }
}

#[derive(Debug)]
pub struct BatteryStatusReport {
    pub state_of_charge: u8,
    pub remaining_time: u32,
    pub battery_health: BatteryHealth,
    pub system_state: SystemState,
    pub power_statistics: PowerStatistics,
    pub sampling_interval: u32,
    pub power_savings: f32,
}

#[derive(Debug, PartialEq)]
pub enum SystemAction {
    None,
    EnterPowerSaving,
    EnterEmergencyMode,
    ResumeNormal,
    EmergencyShutdown,
}
```

## 总结

电池优化是一个综合性的系统工程，需要从硬件设计到软件算法的全方位考虑：

1. **精确监控**: 实时监控电压、电流、温度等关键参数
2. **智能管理**: 根据电池状态动态调整系统行为
3. **保护机制**: 实现多层次的电池保护策略
4. **自适应优化**: 根据使用模式自动优化功耗
5. **紧急处理**: 在极端情况下保护电池和数据安全

通过这些技术的综合应用，可以显著延长电池寿命，提高系统可靠性，为用户提供更好的使用体验。