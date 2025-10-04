# 功耗管理技术

## 概述

功耗管理是嵌入式系统设计中的关键技术，特别是在电池供电的便携设备、物联网节点和长期部署的传感器网络中。有效的功耗管理不仅能延长设备的工作时间，还能减少发热、提高可靠性并降低运营成本。

## 学习目标

完成本章节后，你将掌握：
- 功耗的来源和分类
- 低功耗设计的基本原则
- 各种睡眠模式和唤醒机制
- 动态电压频率调节技术
- 功耗测量和分析方法
- 软硬件协同的功耗优化策略

## 1. 功耗基础理论

### 1.1 功耗的来源

```rust
// 功耗组成分析
#[derive(Debug, Clone, Copy)]
struct PowerConsumption {
    static_power: f32,      // 静态功耗 (漏电流)
    dynamic_power: f32,     // 动态功耗 (开关活动)
    short_circuit_power: f32, // 短路功耗
}

impl PowerConsumption {
    fn total_power(&self) -> f32 {
        self.static_power + self.dynamic_power + self.short_circuit_power
    }
    
    // 动态功耗计算: P = α × C × V² × f
    fn calculate_dynamic_power(
        activity_factor: f32,  // α: 活动因子
        capacitance: f32,      // C: 负载电容
        voltage: f32,          // V: 供电电压
        frequency: f32,        // f: 工作频率
    ) -> f32 {
        activity_factor * capacitance * voltage * voltage * frequency
    }
    
    // 静态功耗计算: P = V × I_leak
    fn calculate_static_power(voltage: f32, leakage_current: f32) -> f32 {
        voltage * leakage_current
    }
}

// 系统功耗模型
struct SystemPowerModel {
    cpu_power: PowerConsumption,
    memory_power: PowerConsumption,
    peripheral_power: PowerConsumption,
    io_power: PowerConsumption,
}

impl SystemPowerModel {
    fn total_system_power(&self) -> f32 {
        self.cpu_power.total_power() +
        self.memory_power.total_power() +
        self.peripheral_power.total_power() +
        self.io_power.total_power()
    }
    
    fn power_breakdown(&self) -> PowerBreakdown {
        let total = self.total_system_power();
        PowerBreakdown {
            cpu_percentage: (self.cpu_power.total_power() / total) * 100.0,
            memory_percentage: (self.memory_power.total_power() / total) * 100.0,
            peripheral_percentage: (self.peripheral_power.total_power() / total) * 100.0,
            io_percentage: (self.io_power.total_power() / total) * 100.0,
        }
    }
}

#[derive(Debug)]
struct PowerBreakdown {
    cpu_percentage: f32,
    memory_percentage: f32,
    peripheral_percentage: f32,
    io_percentage: f32,
}
```

### 1.2 功耗分类

```rust
#[derive(Debug, Clone, Copy)]
enum PowerState {
    Active,      // 活动状态
    Idle,        // 空闲状态
    Sleep,       // 睡眠状态
    DeepSleep,   // 深度睡眠
    Standby,     // 待机状态
    Shutdown,    // 关机状态
}

#[derive(Debug, Clone, Copy)]
struct PowerStateCharacteristics {
    state: PowerState,
    current_consumption: f32,  // 电流消耗 (μA)
    wakeup_time: u32,         // 唤醒时间 (μs)
    context_retained: bool,    // 是否保持上下文
    ram_retained: bool,        // 是否保持RAM内容
    rtc_active: bool,         // RTC是否工作
}

const POWER_STATES: [PowerStateCharacteristics; 6] = [
    PowerStateCharacteristics {
        state: PowerState::Active,
        current_consumption: 25000.0,  // 25mA
        wakeup_time: 0,
        context_retained: true,
        ram_retained: true,
        rtc_active: true,
    },
    PowerStateCharacteristics {
        state: PowerState::Idle,
        current_consumption: 15000.0,  // 15mA
        wakeup_time: 1,
        context_retained: true,
        ram_retained: true,
        rtc_active: true,
    },
    PowerStateCharacteristics {
        state: PowerState::Sleep,
        current_consumption: 500.0,    // 500μA
        wakeup_time: 10,
        context_retained: true,
        ram_retained: true,
        rtc_active: true,
    },
    PowerStateCharacteristics {
        state: PowerState::DeepSleep,
        current_consumption: 50.0,     // 50μA
        wakeup_time: 100,
        context_retained: false,
        ram_retained: true,
        rtc_active: true,
    },
    PowerStateCharacteristics {
        state: PowerState::Standby,
        current_consumption: 2.0,      // 2μA
        wakeup_time: 1000,
        context_retained: false,
        ram_retained: false,
        rtc_active: true,
    },
    PowerStateCharacteristics {
        state: PowerState::Shutdown,
        current_consumption: 0.1,      // 0.1μA
        wakeup_time: 10000,
        context_retained: false,
        ram_retained: false,
        rtc_active: false,
    },
];
```

## 2. 低功耗设计原则

### 2.1 硬件设计原则

```rust
// 时钟管理策略
struct ClockManager {
    system_clock: u32,
    peripheral_clocks: heapless::FnvIndexMap<PeripheralId, u32, 16>,
    clock_gates: u32,  // 时钟门控寄存器
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum PeripheralId {
    UART1, UART2, SPI1, SPI2, I2C1, I2C2,
    TIM1, TIM2, TIM3, ADC1, DAC1, GPIO,
}

impl ClockManager {
    fn new(system_clock: u32) -> Self {
        Self {
            system_clock,
            peripheral_clocks: heapless::FnvIndexMap::new(),
            clock_gates: 0,
        }
    }
    
    fn enable_peripheral_clock(&mut self, peripheral: PeripheralId) {
        let bit_position = peripheral as u32;
        self.clock_gates |= 1 << bit_position;
        
        // 设置外设时钟频率
        let clock_freq = match peripheral {
            PeripheralId::UART1 | PeripheralId::UART2 => self.system_clock / 4,
            PeripheralId::SPI1 | PeripheralId::SPI2 => self.system_clock / 2,
            PeripheralId::I2C1 | PeripheralId::I2C2 => self.system_clock / 8,
            _ => self.system_clock,
        };
        
        let _ = self.peripheral_clocks.insert(peripheral, clock_freq);
    }
    
    fn disable_peripheral_clock(&mut self, peripheral: PeripheralId) {
        let bit_position = peripheral as u32;
        self.clock_gates &= !(1 << bit_position);
        self.peripheral_clocks.remove(&peripheral);
    }
    
    fn scale_system_clock(&mut self, scale_factor: f32) {
        self.system_clock = (self.system_clock as f32 * scale_factor) as u32;
        
        // 重新计算所有外设时钟
        for (peripheral, clock) in self.peripheral_clocks.iter_mut() {
            *clock = (*clock as f32 * scale_factor) as u32;
        }
    }
    
    fn get_active_power_estimate(&self) -> f32 {
        // 简化的功耗估算模型
        let base_power = (self.system_clock as f32 / 1_000_000.0) * 0.5; // 0.5mA per MHz
        let peripheral_power: f32 = self.peripheral_clocks.values()
            .map(|&freq| (freq as f32 / 1_000_000.0) * 0.1) // 0.1mA per MHz for peripherals
            .sum();
        
        base_power + peripheral_power
    }
}
```

### 2.2 软件设计原则

```rust
// 任务功耗管理
struct PowerAwareTask {
    id: u32,
    priority: u8,
    power_budget: f32,        // 功耗预算 (mW)
    execution_time: u32,      // 执行时间 (ms)
    deadline: u32,            // 截止期 (ms)
    power_state: PowerState,  // 期望的功耗状态
}

impl PowerAwareTask {
    fn energy_consumption(&self) -> f32 {
        // 能耗 = 功率 × 时间
        self.power_budget * (self.execution_time as f32 / 1000.0)
    }
    
    fn power_efficiency(&self) -> f32 {
        // 功耗效率 = 执行时间 / 功耗预算
        self.execution_time as f32 / self.power_budget
    }
    
    fn can_use_low_power_mode(&self) -> bool {
        let slack_time = self.deadline - self.execution_time;
        slack_time > 100  // 如果有超过100ms的空闲时间，可以进入低功耗模式
    }
}

// 功耗感知调度器
struct PowerAwareScheduler {
    tasks: heapless::Vec<PowerAwareTask, 16>,
    total_power_budget: f32,
    current_power_consumption: f32,
    power_manager: PowerManager,
}

impl PowerAwareScheduler {
    fn schedule_next_task(&mut self) -> Option<PowerAwareTask> {
        // 按功耗效率排序任务
        self.tasks.sort_by(|a, b| {
            b.power_efficiency().partial_cmp(&a.power_efficiency()).unwrap()
        });
        
        // 选择功耗效率最高且在功耗预算内的任务
        for (i, task) in self.tasks.iter().enumerate() {
            if self.current_power_consumption + task.power_budget <= self.total_power_budget {
                return Some(self.tasks.swap_remove(i));
            }
        }
        
        None
    }
    
    fn optimize_power_consumption(&mut self) {
        // 动态调整系统时钟
        let utilization = self.calculate_cpu_utilization();
        if utilization < 0.5 {
            self.power_manager.scale_frequency(0.8);
        } else if utilization > 0.9 {
            self.power_manager.scale_frequency(1.2);
        }
        
        // 关闭未使用的外设
        self.power_manager.disable_unused_peripherals();
    }
    
    fn calculate_cpu_utilization(&self) -> f32 {
        let total_execution_time: u32 = self.tasks.iter()
            .map(|task| task.execution_time)
            .sum();
        
        let total_period = 1000; // 假设1秒的调度周期
        total_execution_time as f32 / total_period as f32
    }
}
```

## 3. 睡眠模式和唤醒机制

### 3.1 睡眠模式实现

```rust
use cortex_m::peripheral::{SCB, NVIC};
use stm32f4xx_hal::pac::{PWR, RCC};

struct SleepModeManager {
    power_state: PowerState,
    wakeup_sources: heapless::Vec<WakeupSource, 8>,
    context_backup: Option<SystemContext>,
}

#[derive(Debug, Clone, Copy)]
enum WakeupSource {
    ExternalInterrupt(u8),  // 外部中断引脚
    Timer(u32),            // 定时器
    RTC,                   // 实时时钟
    UART,                  // 串口数据
    Watchdog,              // 看门狗
}

#[derive(Debug, Clone)]
struct SystemContext {
    cpu_registers: [u32; 16],
    peripheral_states: heapless::Vec<(PeripheralId, u32), 16>,
    gpio_states: u32,
    clock_config: u32,
}

impl SleepModeManager {
    fn new() -> Self {
        Self {
            power_state: PowerState::Active,
            wakeup_sources: heapless::Vec::new(),
            context_backup: None,
        }
    }
    
    fn add_wakeup_source(&mut self, source: WakeupSource) -> Result<(), &'static str> {
        self.wakeup_sources.push(source).map_err(|_| "Wakeup source list full")
    }
    
    fn enter_sleep_mode(&mut self, target_state: PowerState) -> Result<(), &'static str> {
        match target_state {
            PowerState::Sleep => self.enter_sleep(),
            PowerState::DeepSleep => self.enter_deep_sleep(),
            PowerState::Standby => self.enter_standby(),
            _ => Err("Invalid sleep state"),
        }
    }
    
    fn enter_sleep(&mut self) -> Result<(), &'static str> {
        // 配置唤醒源
        self.configure_wakeup_sources()?;
        
        // 进入睡眠模式
        unsafe {
            let scb = &*SCB::ptr();
            scb.scr.modify(|r| r & !0x04); // 清除SLEEPDEEP位
            cortex_m::asm::wfi(); // 等待中断
        }
        
        self.power_state = PowerState::Sleep;
        Ok(())
    }
    
    fn enter_deep_sleep(&mut self) -> Result<(), &'static str> {
        // 保存系统上下文
        self.save_context();
        
        // 配置唤醒源
        self.configure_wakeup_sources()?;
        
        // 关闭不必要的时钟
        self.disable_clocks();
        
        // 进入深度睡眠
        unsafe {
            let scb = &*SCB::ptr();
            let pwr = &*PWR::ptr();
            
            // 设置深度睡眠模式
            scb.scr.modify(|r| r | 0x04); // 设置SLEEPDEEP位
            pwr.cr.modify(|_, w| w.pdds().clear_bit()); // 进入Stop模式
            
            cortex_m::asm::wfi();
        }
        
        self.power_state = PowerState::DeepSleep;
        Ok(())
    }
    
    fn enter_standby(&mut self) -> Result<(), &'static str> {
        // 保存关键数据到备份寄存器
        self.save_critical_data();
        
        // 配置唤醒源（通常只有RTC和外部引脚）
        self.configure_standby_wakeup()?;
        
        // 进入待机模式
        unsafe {
            let scb = &*SCB::ptr();
            let pwr = &*PWR::ptr();
            
            scb.scr.modify(|r| r | 0x04); // 设置SLEEPDEEP位
            pwr.cr.modify(|_, w| w.pdds().set_bit()); // 进入Standby模式
            
            cortex_m::asm::wfi();
        }
        
        self.power_state = PowerState::Standby;
        Ok(())
    }
    
    fn wakeup_from_sleep(&mut self) -> Result<(), &'static str> {
        match self.power_state {
            PowerState::Sleep => {
                // 从睡眠模式唤醒，上下文自动恢复
                self.power_state = PowerState::Active;
            }
            PowerState::DeepSleep => {
                // 从深度睡眠唤醒，需要恢复时钟和上下文
                self.restore_clocks();
                self.restore_context();
                self.power_state = PowerState::Active;
            }
            PowerState::Standby => {
                // 从待机模式唤醒，系统重启
                self.restore_critical_data();
                self.system_reinitialize();
                self.power_state = PowerState::Active;
            }
            _ => return Err("Not in sleep state"),
        }
        
        Ok(())
    }
    
    fn configure_wakeup_sources(&self) -> Result<(), &'static str> {
        for source in &self.wakeup_sources {
            match source {
                WakeupSource::ExternalInterrupt(pin) => {
                    self.configure_external_interrupt(*pin)?;
                }
                WakeupSource::Timer(period) => {
                    self.configure_timer_wakeup(*period)?;
                }
                WakeupSource::RTC => {
                    self.configure_rtc_wakeup()?;
                }
                WakeupSource::UART => {
                    self.configure_uart_wakeup()?;
                }
                WakeupSource::Watchdog => {
                    self.configure_watchdog_wakeup()?;
                }
            }
        }
        Ok(())
    }
    
    fn configure_external_interrupt(&self, pin: u8) -> Result<(), &'static str> {
        // 配置外部中断作为唤醒源
        unsafe {
            let nvic = &*NVIC::ptr();
            match pin {
                0 => nvic.enable(stm32f4xx_hal::pac::Interrupt::EXTI0),
                1 => nvic.enable(stm32f4xx_hal::pac::Interrupt::EXTI1),
                // ... 其他引脚
                _ => return Err("Unsupported interrupt pin"),
            }
        }
        Ok(())
    }
    
    fn configure_timer_wakeup(&self, period: u32) -> Result<(), &'static str> {
        // 配置定时器唤醒
        // 实现定时器配置逻辑
        Ok(())
    }
    
    fn configure_rtc_wakeup(&self) -> Result<(), &'static str> {
        // 配置RTC唤醒
        // 实现RTC配置逻辑
        Ok(())
    }
    
    fn configure_uart_wakeup(&self) -> Result<(), &'static str> {
        // 配置UART唤醒
        // 实现UART唤醒配置逻辑
        Ok(())
    }
    
    fn configure_watchdog_wakeup(&self) -> Result<(), &'static str> {
        // 配置看门狗唤醒
        // 实现看门狗配置逻辑
        Ok(())
    }
    
    fn configure_standby_wakeup(&self) -> Result<(), &'static str> {
        // 待机模式只支持特定的唤醒源
        for source in &self.wakeup_sources {
            match source {
                WakeupSource::RTC => self.configure_rtc_wakeup()?,
                WakeupSource::ExternalInterrupt(0) => {
                    // 通常只有WKUP引脚可以从待机模式唤醒
                    self.configure_wkup_pin()?;
                }
                _ => return Err("Unsupported wakeup source for standby mode"),
            }
        }
        Ok(())
    }
    
    fn configure_wkup_pin(&self) -> Result<(), &'static str> {
        // 配置WKUP引脚
        unsafe {
            let pwr = &*PWR::ptr();
            pwr.csr.modify(|_, w| w.ewup().set_bit());
        }
        Ok(())
    }
    
    fn save_context(&mut self) {
        // 保存系统上下文
        let context = SystemContext {
            cpu_registers: [0; 16], // 简化实现
            peripheral_states: heapless::Vec::new(),
            gpio_states: 0,
            clock_config: 0,
        };
        self.context_backup = Some(context);
    }
    
    fn restore_context(&mut self) {
        // 恢复系统上下文
        if let Some(_context) = &self.context_backup {
            // 恢复寄存器状态
            // 恢复外设配置
            // 恢复GPIO状态
        }
    }
    
    fn save_critical_data(&self) {
        // 保存关键数据到备份寄存器或EEPROM
    }
    
    fn restore_critical_data(&self) {
        // 从备份寄存器或EEPROM恢复关键数据
    }
    
    fn disable_clocks(&self) {
        // 关闭不必要的时钟
        unsafe {
            let rcc = &*RCC::ptr();
            // 关闭外设时钟
            rcc.ahb1enr.modify(|_, w| w.gpioaen().clear_bit());
            // ... 关闭其他时钟
        }
    }
    
    fn restore_clocks(&self) {
        // 恢复时钟配置
        unsafe {
            let rcc = &*RCC::ptr();
            // 恢复外设时钟
            rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
            // ... 恢复其他时钟
        }
    }
    
    fn system_reinitialize(&self) {
        // 系统重新初始化（从待机模式唤醒后）
        // 重新配置时钟
        // 重新初始化外设
        // 恢复应用状态
    }
}
```

### 3.2 智能唤醒策略

```rust
struct IntelligentWakeupManager {
    sleep_manager: SleepModeManager,
    activity_predictor: ActivityPredictor,
    energy_harvester: Option<EnergyHarvester>,
    wakeup_scheduler: WakeupScheduler,
}

struct ActivityPredictor {
    historical_data: heapless::Vec<ActivityRecord, 100>,
    prediction_model: PredictionModel,
}

#[derive(Debug, Clone, Copy)]
struct ActivityRecord {
    timestamp: u32,
    activity_type: ActivityType,
    duration: u32,
    power_consumption: f32,
}

#[derive(Debug, Clone, Copy)]
enum ActivityType {
    SensorReading,
    DataTransmission,
    UserInteraction,
    Maintenance,
    Emergency,
}

struct PredictionModel {
    weights: [f32; 5],
    bias: f32,
}

impl ActivityPredictor {
    fn predict_next_activity(&self, current_time: u32) -> Option<(ActivityType, u32)> {
        // 简化的活动预测算法
        let recent_activities: heapless::Vec<ActivityRecord, 10> = self.historical_data
            .iter()
            .filter(|record| current_time - record.timestamp < 3600) // 最近1小时
            .cloned()
            .collect();
        
        if recent_activities.is_empty() {
            return None;
        }
        
        // 找到最常见的活动类型
        let mut activity_counts = [0u32; 5];
        for record in &recent_activities {
            activity_counts[record.activity_type as usize] += 1;
        }
        
        let max_count_index = activity_counts.iter()
            .enumerate()
            .max_by_key(|(_, &count)| count)
            .map(|(index, _)| index)?;
        
        let predicted_activity = match max_count_index {
            0 => ActivityType::SensorReading,
            1 => ActivityType::DataTransmission,
            2 => ActivityType::UserInteraction,
            3 => ActivityType::Maintenance,
            4 => ActivityType::Emergency,
            _ => ActivityType::SensorReading,
        };
        
        // 预测下次活动时间
        let avg_interval = recent_activities.windows(2)
            .map(|window| window[1].timestamp - window[0].timestamp)
            .sum::<u32>() / (recent_activities.len() - 1) as u32;
        
        Some((predicted_activity, current_time + avg_interval))
    }
    
    fn record_activity(&mut self, activity: ActivityRecord) {
        if self.historical_data.len() >= 100 {
            self.historical_data.remove(0);
        }
        let _ = self.historical_data.push(activity);
    }
    
    fn update_prediction_model(&mut self) {
        // 使用历史数据更新预测模型
        // 这里可以实现简单的线性回归或其他机器学习算法
    }
}

struct WakeupScheduler {
    scheduled_wakeups: heapless::Vec<ScheduledWakeup, 16>,
    current_time: u32,
}

#[derive(Debug, Clone, Copy)]
struct ScheduledWakeup {
    wakeup_time: u32,
    activity_type: ActivityType,
    power_budget: f32,
    priority: u8,
}

impl WakeupScheduler {
    fn schedule_wakeup(&mut self, wakeup: ScheduledWakeup) -> Result<(), &'static str> {
        // 按时间顺序插入唤醒事件
        let insert_pos = self.scheduled_wakeups
            .iter()
            .position(|w| w.wakeup_time > wakeup.wakeup_time)
            .unwrap_or(self.scheduled_wakeups.len());
        
        self.scheduled_wakeups.insert(insert_pos, wakeup)
            .map_err(|_| "Wakeup schedule full")
    }
    
    fn get_next_wakeup(&mut self) -> Option<ScheduledWakeup> {
        if !self.scheduled_wakeups.is_empty() && 
           self.scheduled_wakeups[0].wakeup_time <= self.current_time {
            Some(self.scheduled_wakeups.remove(0))
        } else {
            None
        }
    }
    
    fn optimize_schedule(&mut self, available_energy: f32) {
        // 根据可用能量优化唤醒调度
        let mut total_energy_needed = 0.0;
        
        for wakeup in &self.scheduled_wakeups {
            total_energy_needed += wakeup.power_budget;
        }
        
        if total_energy_needed > available_energy {
            // 能量不足，需要优化调度
            self.scheduled_wakeups.sort_by(|a, b| b.priority.cmp(&a.priority));
            
            let mut cumulative_energy = 0.0;
            let mut keep_count = 0;
            
            for wakeup in &self.scheduled_wakeups {
                if cumulative_energy + wakeup.power_budget <= available_energy {
                    cumulative_energy += wakeup.power_budget;
                    keep_count += 1;
                } else {
                    break;
                }
            }
            
            self.scheduled_wakeups.truncate(keep_count);
        }
    }
}

struct EnergyHarvester {
    harvesting_type: HarvestingType,
    current_power: f32,      // 当前收集功率 (mW)
    stored_energy: f32,      // 存储的能量 (mWh)
    max_storage: f32,        // 最大存储容量 (mWh)
}

#[derive(Debug, Clone, Copy)]
enum HarvestingType {
    Solar,
    Thermal,
    Vibration,
    RF,
    Wind,
}

impl EnergyHarvester {
    fn update_harvesting_power(&mut self, environmental_conditions: &EnvironmentalConditions) {
        self.current_power = match self.harvesting_type {
            HarvestingType::Solar => {
                environmental_conditions.light_intensity * 0.1 // 简化模型
            }
            HarvestingType::Thermal => {
                environmental_conditions.temperature_gradient * 0.05
            }
            HarvestingType::Vibration => {
                environmental_conditions.vibration_level * 0.02
            }
            HarvestingType::RF => {
                environmental_conditions.rf_power_density * 0.01
            }
            HarvestingType::Wind => {
                environmental_conditions.wind_speed.powi(3) * 0.001
            }
        };
    }
    
    fn harvest_energy(&mut self, duration_ms: u32) {
        let harvested = self.current_power * (duration_ms as f32 / 3600000.0); // 转换为mWh
        self.stored_energy = (self.stored_energy + harvested).min(self.max_storage);
    }
    
    fn consume_energy(&mut self, energy_mwh: f32) -> bool {
        if self.stored_energy >= energy_mwh {
            self.stored_energy -= energy_mwh;
            true
        } else {
            false
        }
    }
    
    fn get_available_energy(&self) -> f32 {
        self.stored_energy
    }
}

#[derive(Debug, Clone, Copy)]
struct EnvironmentalConditions {
    light_intensity: f32,      // 光照强度 (lux)
    temperature_gradient: f32,  // 温度梯度 (°C)
    vibration_level: f32,      // 振动水平 (g)
    rf_power_density: f32,     // RF功率密度 (mW/cm²)
    wind_speed: f32,           // 风速 (m/s)
}

impl IntelligentWakeupManager {
    fn optimize_power_strategy(&mut self, environmental_conditions: &EnvironmentalConditions) {
        // 更新能量收集状态
        if let Some(ref mut harvester) = self.energy_harvester {
            harvester.update_harvesting_power(environmental_conditions);
        }
        
        // 预测下次活动
        let current_time = self.get_current_time();
        if let Some((activity_type, predicted_time)) = self.activity_predictor.predict_next_activity(current_time) {
            let power_budget = self.estimate_activity_power(activity_type);
            
            let scheduled_wakeup = ScheduledWakeup {
                wakeup_time: predicted_time,
                activity_type,
                power_budget,
                priority: self.get_activity_priority(activity_type),
            };
            
            let _ = self.wakeup_scheduler.schedule_wakeup(scheduled_wakeup);
        }
        
        // 根据可用能量优化调度
        let available_energy = self.energy_harvester
            .as_ref()
            .map(|h| h.get_available_energy())
            .unwrap_or(1000.0); // 默认1Wh
        
        self.wakeup_scheduler.optimize_schedule(available_energy);
        
        // 选择最优的睡眠模式
        let optimal_sleep_mode = self.select_optimal_sleep_mode();
        let _ = self.sleep_manager.enter_sleep_mode(optimal_sleep_mode);
    }
    
    fn select_optimal_sleep_mode(&self) -> PowerState {
        let next_wakeup_time = self.wakeup_scheduler.scheduled_wakeups
            .first()
            .map(|w| w.wakeup_time)
            .unwrap_or(u32::MAX);
        
        let current_time = self.get_current_time();
        let sleep_duration = next_wakeup_time - current_time;
        
        // 根据睡眠时间选择最优模式
        if sleep_duration < 100 {
            PowerState::Idle
        } else if sleep_duration < 1000 {
            PowerState::Sleep
        } else if sleep_duration < 10000 {
            PowerState::DeepSleep
        } else {
            PowerState::Standby
        }
    }
    
    fn estimate_activity_power(&self, activity_type: ActivityType) -> f32 {
        match activity_type {
            ActivityType::SensorReading => 5.0,      // 5mW
            ActivityType::DataTransmission => 50.0,  // 50mW
            ActivityType::UserInteraction => 20.0,   // 20mW
            ActivityType::Maintenance => 10.0,       // 10mW
            ActivityType::Emergency => 100.0,        // 100mW
        }
    }
    
    fn get_activity_priority(&self, activity_type: ActivityType) -> u8 {
        match activity_type {
            ActivityType::Emergency => 255,
            ActivityType::SensorReading => 200,
            ActivityType::DataTransmission => 150,
            ActivityType::UserInteraction => 100,
            ActivityType::Maintenance => 50,
        }
    }
    
    fn get_current_time(&self) -> u32 {
        // 返回当前时间戳
        0 // 简化实现
    }
}
```

## 4. 动态电压频率调节(DVFS)

### 4.1 DVFS基础实现

```rust
struct DVFSController {
    voltage_levels: heapless::Vec<VoltageLevel, 8>,
    frequency_levels: heapless::Vec<FrequencyLevel, 8>,
    current_operating_point: OperatingPoint,
    performance_monitor: PerformanceMonitor,
    power_monitor: PowerMonitor,
}

#[derive(Debug, Clone, Copy)]
struct VoltageLevel {
    voltage: f32,      // 电压 (V)
    power_overhead: f32, // 功耗开销 (mW)
    switching_time: u32, // 切换时间 (μs)
}

#[derive(Debug, Clone, Copy)]
struct FrequencyLevel {
    frequency: u32,    // 频率 (Hz)
    performance_factor: f32, // 性能因子
    switching_time: u32, // 切换时间 (μs)
}

#[derive(Debug, Clone, Copy)]
struct OperatingPoint {
    voltage_index: usize,
    frequency_index: usize,
    power_consumption: f32,
    performance_level: f32,
}

struct PerformanceMonitor {
    cpu_utilization: f32,
    task_completion_rate: f32,
    deadline_miss_rate: f32,
    response_time: u32,
}

struct PowerMonitor {
    current_power: f32,
    average_power: f32,
    peak_power: f32,
    energy_consumed: f32,
}

impl DVFSController {
    fn new() -> Self {
        let voltage_levels = heapless::Vec::from_slice(&[
            VoltageLevel { voltage: 1.8, power_overhead: 10.0, switching_time: 100 },
            VoltageLevel { voltage: 1.5, power_overhead: 7.0, switching_time: 80 },
            VoltageLevel { voltage: 1.2, power_overhead: 5.0, switching_time: 60 },
            VoltageLevel { voltage: 1.0, power_overhead: 3.0, switching_time: 40 },
        ]).unwrap();
        
        let frequency_levels = heapless::Vec::from_slice(&[
            FrequencyLevel { frequency: 168_000_000, performance_factor: 1.0, switching_time: 50 },
            FrequencyLevel { frequency: 120_000_000, performance_factor: 0.8, switching_time: 40 },
            FrequencyLevel { frequency: 84_000_000, performance_factor: 0.6, switching_time: 30 },
            FrequencyLevel { frequency: 48_000_000, performance_factor: 0.4, switching_time: 20 },
            FrequencyLevel { frequency: 16_000_000, performance_factor: 0.2, switching_time: 10 },
        ]).unwrap();
        
        Self {
            voltage_levels,
            frequency_levels,
            current_operating_point: OperatingPoint {
                voltage_index: 0,
                frequency_index: 0,
                power_consumption: 0.0,
                performance_level: 1.0,
            },
            performance_monitor: PerformanceMonitor {
                cpu_utilization: 0.0,
                task_completion_rate: 1.0,
                deadline_miss_rate: 0.0,
                response_time: 0,
            },
            power_monitor: PowerMonitor {
                current_power: 0.0,
                average_power: 0.0,
                peak_power: 0.0,
                energy_consumed: 0.0,
            },
        }
    }
    
    fn update_operating_point(&mut self, target_performance: f32, power_budget: f32) -> Result<(), &'static str> {
        let optimal_point = self.find_optimal_operating_point(target_performance, power_budget)?;
        
        if optimal_point.voltage_index != self.current_operating_point.voltage_index ||
           optimal_point.frequency_index != self.current_operating_point.frequency_index {
            self.transition_to_operating_point(optimal_point)?;
        }
        
        Ok(())
    }
    
    fn find_optimal_operating_point(&self, target_performance: f32, power_budget: f32) -> Result<OperatingPoint, &'static str> {
        let mut best_point = None;
        let mut best_efficiency = 0.0;
        
        for (v_idx, voltage) in self.voltage_levels.iter().enumerate() {
            for (f_idx, frequency) in self.frequency_levels.iter().enumerate() {
                let power = self.calculate_power_consumption(v_idx, f_idx);
                let performance = frequency.performance_factor;
                
                // 检查是否满足约束条件
                if power <= power_budget && performance >= target_performance {
                    let efficiency = performance / power;
                    
                    if efficiency > best_efficiency {
                        best_efficiency = efficiency;
                        best_point = Some(OperatingPoint {
                            voltage_index: v_idx,
                            frequency_index: f_idx,
                            power_consumption: power,
                            performance_level: performance,
                        });
                    }
                }
            }
        }
        
        best_point.ok_or("No suitable operating point found")
    }
    
    fn calculate_power_consumption(&self, voltage_index: usize, frequency_index: usize) -> f32 {
        let voltage = self.voltage_levels[voltage_index].voltage;
        let frequency = self.frequency_levels[frequency_index].frequency as f32;
        
        // 简化的功耗模型: P = C * V² * f + P_static
        let capacitance = 1e-12; // 1pF
        let dynamic_power = capacitance * voltage * voltage * frequency;
        let static_power = self.voltage_levels[voltage_index].power_overhead;
        
        dynamic_power + static_power
    }
    
    fn transition_to_operating_point(&mut self, new_point: OperatingPoint) -> Result<(), &'static str> {
        let current_voltage = self.voltage_levels[self.current_operating_point.voltage_index].voltage;
        let new_voltage = self.voltage_levels[new_point.voltage_index].voltage;
        let current_frequency = self.frequency_levels[self.current_operating_point.frequency_index].frequency;
        let new_frequency = self.frequency_levels[new_point.frequency_index].frequency;
        
        // DVFS切换顺序：
        // 1. 如果提高性能：先提高电压，再提高频率
        // 2. 如果降低性能：先降低频率，再降低电压
        
        if new_frequency > current_frequency {
            // 提高性能
            if new_voltage > current_voltage {
                self.set_voltage(new_point.voltage_index)?;
                self.wait_voltage_stable();
            }
            self.set_frequency(new_point.frequency_index)?;
        } else {
            // 降低性能
            self.set_frequency(new_point.frequency_index)?;
            if new_voltage < current_voltage {
                self.set_voltage(new_point.voltage_index)?;
            }
        }
        
        self.current_operating_point = new_point;
        Ok(())
    }
    
    fn set_voltage(&self, voltage_index: usize) -> Result<(), &'static str> {
        // 设置电压调节器
        let voltage = self.voltage_levels[voltage_index].voltage;
        
        // 这里需要调用具体的电压调节器API
        // 例如：通过I2C控制PMIC
        self.configure_voltage_regulator(voltage)
    }
    
    fn set_frequency(&self, frequency_index: usize) -> Result<(), &'static str> {
        // 设置系统时钟频率
        let frequency = self.frequency_levels[frequency_index].frequency;
        
        // 这里需要调用具体的时钟配置API
        self.configure_system_clock(frequency)
    }
    
    fn wait_voltage_stable(&self) {
        // 等待电压稳定
        let switching_time = self.voltage_levels[self.current_operating_point.voltage_index].switching_time;
        self.delay_microseconds(switching_time);
    }
    
    fn configure_voltage_regulator(&self, voltage: f32) -> Result<(), &'static str> {
        // 实际的电压调节器配置
        // 这里是简化实现
        Ok(())
    }
    
    fn configure_system_clock(&self, frequency: u32) -> Result<(), &'static str> {
        // 实际的系统时钟配置
        // 这里是简化实现
        Ok(())
    }
    
    fn delay_microseconds(&self, microseconds: u32) {
        // 微秒级延时
        for _ in 0..(microseconds * 168) { // 假设168MHz时钟
            cortex_m::asm::nop();
        }
    }
}
```

### 4.2 自适应DVFS算法

```rust
struct AdaptiveDVFSController {
    dvfs_controller: DVFSController,
    workload_predictor: WorkloadPredictor,
    thermal_monitor: ThermalMonitor,
    battery_monitor: BatteryMonitor,
    control_algorithm: DVFSAlgorithm,
}

struct WorkloadPredictor {
    historical_workload: heapless::Vec<WorkloadSample, 64>,
    prediction_window: u32,
    prediction_accuracy: f32,
}

#[derive(Debug, Clone, Copy)]
struct WorkloadSample {
    timestamp: u32,
    cpu_utilization: f32,
    memory_utilization: f32,
    io_activity: f32,
    task_count: u32,
}

struct ThermalMonitor {
    current_temperature: f32,
    max_temperature: f32,
    thermal_throttling_threshold: f32,
    cooling_rate: f32,
}

struct BatteryMonitor {
    current_voltage: f32,
    remaining_capacity: f32,
    discharge_rate: f32,
    estimated_lifetime: u32,
}

#[derive(Debug, Clone, Copy)]
enum DVFSAlgorithm {
    Conservative,    // 保守算法：缓慢调整
    Ondemand,       // 按需算法：快速响应负载变化
    Performance,    // 性能算法：优先性能
    Powersave,      // 省电算法：优先功耗
    Adaptive,       // 自适应算法：综合考虑多种因素
}

impl AdaptiveDVFSController {
    fn update_dvfs_policy(&mut self) -> Result<(), &'static str> {
        // 收集系统状态信息
        let current_workload = self.measure_current_workload();
        let predicted_workload = self.workload_predictor.predict_workload()?;
        let thermal_state = self.thermal_monitor.get_thermal_state();
        let battery_state = self.battery_monitor.get_battery_state();
        
        // 根据系统状态选择DVFS策略
        let target_performance = self.calculate_target_performance(
            predicted_workload,
            thermal_state,
            battery_state,
        );
        
        let power_budget = self.calculate_power_budget(thermal_state, battery_state);
        
        // 更新操作点
        self.dvfs_controller.update_operating_point(target_performance, power_budget)?;
        
        // 记录工作负载样本
        self.workload_predictor.record_workload(current_workload);
        
        Ok(())
    }
    
    fn measure_current_workload(&self) -> WorkloadSample {
        WorkloadSample {
            timestamp: self.get_current_time(),
            cpu_utilization: self.measure_cpu_utilization(),
            memory_utilization: self.measure_memory_utilization(),
            io_activity: self.measure_io_activity(),
            task_count: self.get_active_task_count(),
        }
    }
    
    fn calculate_target_performance(&self, workload: WorkloadSample, thermal: ThermalState, battery: BatteryState) -> f32 {
        let mut target_performance = workload.cpu_utilization;
        
        // 根据算法类型调整目标性能
        match self.control_algorithm {
            DVFSAlgorithm::Conservative => {
                // 保守算法：只有在高负载时才提高性能
                if workload.cpu_utilization > 0.8 {
                    target_performance = 1.0;
                } else if workload.cpu_utilization < 0.3 {
                    target_performance = 0.5;
                }
            }
            DVFSAlgorithm::Ondemand => {
                // 按需算法：快速响应负载变化
                target_performance = workload.cpu_utilization * 1.2; // 留20%余量
            }
            DVFSAlgorithm::Performance => {
                // 性能算法：始终保持高性能
                target_performance = 1.0;
            }
            DVFSAlgorithm::Powersave => {
                // 省电算法：尽可能降低性能
                target_performance = workload.cpu_utilization * 0.8;
            }
            DVFSAlgorithm::Adaptive => {
                // 自适应算法：综合考虑多种因素
                target_performance = self.adaptive_performance_calculation(workload, thermal, battery);
            }
        }
        
        // 应用热管理约束
        if thermal.is_overheating() {
            target_performance *= 0.7; // 降低30%性能
        }
        
        // 应用电池管理约束
        if battery.is_low_power() {
            target_performance *= 0.8; // 降低20%性能
        }
        
        target_performance.clamp(0.1, 1.0)
    }
    
    fn adaptive_performance_calculation(&self, workload: WorkloadSample, thermal: ThermalState, battery: BatteryState) -> f32 {
        // 多因子自适应算法
        let workload_factor = workload.cpu_utilization;
        let thermal_factor = if thermal.temperature > thermal.throttling_threshold {
            0.5
        } else {
            1.0
        };
        let battery_factor = battery.remaining_capacity / 100.0;
        
        // 加权平均
        let weights = [0.5, 0.3, 0.2]; // 工作负载、热管理、电池状态的权重
        workload_factor * weights[0] + thermal_factor * weights[1] + battery_factor * weights[2]
    }
    
    fn calculate_power_budget(&self, thermal: ThermalState, battery: BatteryState) -> f32 {
        let mut base_budget = 1000.0; // 基础功耗预算 1W
        
        // 热管理约束
        if thermal.is_overheating() {
            base_budget *= 0.6; // 降低40%功耗预算
        } else if thermal.temperature > thermal.throttling_threshold * 0.8 {
            base_budget *= 0.8; // 降低20%功耗预算
        }
        
        // 电池管理约束
        if battery.remaining_capacity < 20.0 {
            base_budget *= 0.5; // 低电量时降低50%功耗预算
        } else if battery.remaining_capacity < 50.0 {
            base_budget *= 0.7; // 中等电量时降低30%功耗预算
        }
        
        base_budget
    }
    
    fn measure_cpu_utilization(&self) -> f32 {
        // 测量CPU利用率
        // 这里是简化实现
        0.5
    }
    
    fn measure_memory_utilization(&self) -> f32 {
        // 测量内存利用率
        0.3
    }
    
    fn measure_io_activity(&self) -> f32 {
        // 测量I/O活动
        0.2
    }
    
    fn get_active_task_count(&self) -> u32 {
        // 获取活动任务数量
        5
    }
    
    fn get_current_time(&self) -> u32 {
        // 获取当前时间戳
        0
    }
}

#[derive(Debug, Clone, Copy)]
struct ThermalState {
    temperature: f32,
    throttling_threshold: f32,
    critical_threshold: f32,
}

impl ThermalState {
    fn is_overheating(&self) -> bool {
        self.temperature > self.throttling_threshold
    }
    
    fn is_critical(&self) -> bool {
        self.temperature > self.critical_threshold
    }
}

#[derive(Debug, Clone, Copy)]
struct BatteryState {
    remaining_capacity: f32, // 剩余电量百分比
    voltage: f32,
    discharge_rate: f32,
}

impl BatteryState {
    fn is_low_power(&self) -> bool {
        self.remaining_capacity < 20.0
    }
    
    fn estimated_runtime(&self) -> u32 {
        if self.discharge_rate > 0.0 {
            (self.remaining_capacity / self.discharge_rate * 3600.0) as u32
        } else {
            u32::MAX
        }
    }
}

impl WorkloadPredictor {
    fn predict_workload(&self) -> Result<WorkloadSample, &'static str> {
        if self.historical_workload.len() < 3 {
            return Err("Insufficient historical data");
        }
        
        // 简单的移动平均预测
        let recent_samples = &self.historical_workload[self.historical_workload.len()-3..];
        let avg_cpu = recent_samples.iter().map(|s| s.cpu_utilization).sum::<f32>() / 3.0;
        let avg_memory = recent_samples.iter().map(|s| s.memory_utilization).sum::<f32>() / 3.0;
        let avg_io = recent_samples.iter().map(|s| s.io_activity).sum::<f32>() / 3.0;
        let avg_tasks = recent_samples.iter().map(|s| s.task_count).sum::<u32>() / 3;
        
        Ok(WorkloadSample {
            timestamp: self.get_predicted_time(),
            cpu_utilization: avg_cpu,
            memory_utilization: avg_memory,
            io_activity: avg_io,
            task_count: avg_tasks,
        })
    }
    
    fn record_workload(&mut self, sample: WorkloadSample) {
        if self.historical_workload.len() >= 64 {
            self.historical_workload.remove(0);
        }
        let _ = self.historical_workload.push(sample);
    }
    
    fn get_predicted_time(&self) -> u32 {
        // 返回预测时间点
        0
    }
}

impl ThermalMonitor {
    fn get_thermal_state(&self) -> ThermalState {
        ThermalState {
            temperature: self.current_temperature,
            throttling_threshold: self.thermal_throttling_threshold,
            critical_threshold: self.max_temperature,
        }
    }
    
    fn update_temperature(&mut self) {
        // 读取温度传感器
        self.current_temperature = self.read_temperature_sensor();
    }
    
    fn read_temperature_sensor(&self) -> f32 {
        // 读取实际温度传感器
        25.0 // 简化实现
    }
}

impl BatteryMonitor {
    fn get_battery_state(&self) -> BatteryState {
        BatteryState {
            remaining_capacity: self.remaining_capacity,
            voltage: self.current_voltage,
            discharge_rate: self.discharge_rate,
        }
    }
    
    fn update_battery_status(&mut self) {
        // 更新电池状态
        self.current_voltage = self.read_battery_voltage();
        self.remaining_capacity = self.calculate_remaining_capacity();
        self.discharge_rate = self.calculate_discharge_rate();
    }
    
    fn read_battery_voltage(&self) -> f32 {
        // 读取电池电压
        3.7 // 简化实现
    }
    
    fn calculate_remaining_capacity(&self) -> f32 {
        // 根据电压计算剩余电量
        ((self.current_voltage - 3.0) / (4.2 - 3.0) * 100.0).clamp(0.0, 100.0)
    }
    
    fn calculate_discharge_rate(&self) -> f32 {
        // 计算放电速率
        1.0 // 简化实现：1%/小时
    }
}
```

## 5. 功耗测量和分析

### 5.1 功耗测量系统

```rust
struct PowerMeasurementSystem {
    adc: AdcInterface,
    current_sensor: CurrentSensor,
    voltage_sensor: VoltageSensor,
    measurement_buffer: heapless::Vec<PowerSample, 1000>,
    sampling_rate: u32,
    calibration_data: CalibrationData,
}

#[derive(Debug, Clone, Copy)]
struct PowerSample {
    timestamp: u32,
    voltage: f32,      // 电压 (V)
    current: f32,      // 电流 (A)
    power: f32,        // 功率 (W)
    temperature: f32,  // 温度 (°C)
}

struct CalibrationData {
    voltage_offset: f32,
    voltage_scale: f32,
    current_offset: f32,
    current_scale: f32,
    temperature_offset: f32,
}

trait AdcInterface {
    fn read_channel(&self, channel: u8) -> Result<u16, &'static str>;
    fn start_continuous_conversion(&mut self, channels: &[u8]) -> Result<(), &'static str>;
    fn stop_conversion(&mut self) -> Result<(), &'static str>;
}

trait CurrentSensor {
    fn read_current(&self) -> Result<f32, &'static str>;
    fn set_range(&mut self, max_current: f32) -> Result<(), &'static str>;
    fn calibrate(&mut self, known_current: f32) -> Result<(), &'static str>;
}

trait VoltageSensor {
    fn read_voltage(&self) -> Result<f32, &'static str>;
    fn set_range(&mut self, max_voltage: f32) -> Result<(), &'static str>;
}

impl PowerMeasurementSystem {
    fn new(adc: impl AdcInterface, current_sensor: impl CurrentSensor, voltage_sensor: impl VoltageSensor) -> Self {
        Self {
            adc: Box::new(adc),
            current_sensor: Box::new(current_sensor),
            voltage_sensor: Box::new(voltage_sensor),
            measurement_buffer: heapless::Vec::new(),
            sampling_rate: 1000, // 1kHz
            calibration_data: CalibrationData {
                voltage_offset: 0.0,
                voltage_scale: 1.0,
                current_offset: 0.0,
                current_scale: 1.0,
                temperature_offset: 0.0,
            },
        }
    }
    
    fn start_measurement(&mut self) -> Result<(), &'static str> {
        // 配置ADC连续转换
        self.adc.start_continuous_conversion(&[0, 1, 2])?; // 电压、电流、温度通道
        
        // 设置传感器量程
        self.current_sensor.set_range(1.0)?; // 最大1A
        self.voltage_sensor.set_range(5.0)?; // 最大5V
        
        Ok(())
    }
    
    fn stop_measurement(&mut self) -> Result<(), &'static str> {
        self.adc.stop_conversion()
    }
    
    fn sample_power(&mut self) -> Result<PowerSample, &'static str> {
        let voltage_raw = self.adc.read_channel(0)?;
        let current_raw = self.adc.read_channel(1)?;
        let temperature_raw = self.adc.read_channel(2)?;
        
        // 应用校准数据
        let voltage = self.calibrate_voltage(voltage_raw);
        let current = self.calibrate_current(current_raw);
        let temperature = self.calibrate_temperature(temperature_raw);
        
        let power = voltage * current;
        
        let sample = PowerSample {
            timestamp: self.get_timestamp(),
            voltage,
            current,
            power,
            temperature,
        };
        
        // 存储样本
        if self.measurement_buffer.len() >= 1000 {
            self.measurement_buffer.remove(0);
        }
        let _ = self.measurement_buffer.push(sample);
        
        Ok(sample)
    }
    
    fn calibrate_voltage(&self, raw_value: u16) -> f32 {
        let normalized = raw_value as f32 / 4095.0; // 12位ADC
        (normalized * self.calibration_data.voltage_scale) + self.calibration_data.voltage_offset
    }
    
    fn calibrate_current(&self, raw_value: u16) -> f32 {
        let normalized = raw_value as f32 / 4095.0;
        (normalized * self.calibration_data.current_scale) + self.calibration_data.current_offset
    }
    
    fn calibrate_temperature(&self, raw_value: u16) -> f32 {
        let normalized = raw_value as f32 / 4095.0;
        (normalized * 100.0) + self.calibration_data.temperature_offset // 简化的温度转换
    }
    
    fn get_timestamp(&self) -> u32 {
        // 返回当前时间戳
        0 // 简化实现
    }
    
    fn calculate_statistics(&self) -> PowerStatistics {
        if self.measurement_buffer.is_empty() {
            return PowerStatistics::default();
        }
        
        let powers: heapless::Vec<f32, 1000> = self.measurement_buffer
            .iter()
            .map(|sample| sample.power)
            .collect();
        
        let min_power = powers.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        let max_power = powers.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        let avg_power = powers.iter().sum::<f32>() / powers.len() as f32;
        
        // 计算能耗
        let total_energy = self.measurement_buffer.windows(2)
            .map(|window| {
                let dt = (window[1].timestamp - window[0].timestamp) as f32 / 1000.0; // 转换为秒
                window[0].power * dt
            })
            .sum::<f32>();
        
        PowerStatistics {
            min_power,
            max_power,
            avg_power,
            total_energy,
            sample_count: self.measurement_buffer.len(),
            measurement_duration: self.get_measurement_duration(),
        }
    }
    
    fn get_measurement_duration(&self) -> u32 {
        if self.measurement_buffer.len() < 2 {
            0
        } else {
            let first = self.measurement_buffer.first().unwrap();
            let last = self.measurement_buffer.last().unwrap();
            last.timestamp - first.timestamp
        }
    }
}

#[derive(Debug, Default)]
struct PowerStatistics {
    min_power: f32,
    max_power: f32,
    avg_power: f32,
    total_energy: f32,
    sample_count: usize,
    measurement_duration: u32,
}

// 功耗分析器
struct PowerAnalyzer {
    measurement_system: PowerMeasurementSystem,
    analysis_results: heapless::Vec<PowerAnalysisResult, 32>,
}

#[derive(Debug, Clone)]
struct PowerAnalysisResult {
    analysis_type: AnalysisType,
    result_data: AnalysisData,
    timestamp: u32,
}

#[derive(Debug, Clone, Copy)]
enum AnalysisType {
    StateAnalysis,      // 状态功耗分析
    FrequencyAnalysis,  // 频域分析
    EfficiencyAnalysis, // 效率分析
    TrendAnalysis,      // 趋势分析
}

#[derive(Debug, Clone)]
enum AnalysisData {
    StateData(StateAnalysisData),
    FrequencyData(FrequencyAnalysisData),
    EfficiencyData(EfficiencyAnalysisData),
    TrendData(TrendAnalysisData),
}

#[derive(Debug, Clone)]
struct StateAnalysisData {
    active_power: f32,
    idle_power: f32,
    sleep_power: f32,
    state_transitions: u32,
}

#[derive(Debug, Clone)]
struct FrequencyAnalysisData {
    dominant_frequency: f32,
    power_spectral_density: heapless::Vec<f32, 64>,
    frequency_bins: heapless::Vec<f32, 64>,
}

#[derive(Debug, Clone)]
struct EfficiencyAnalysisData {
    power_efficiency: f32,
    energy_efficiency: f32,
    performance_per_watt: f32,
}

#[derive(Debug, Clone)]
struct TrendAnalysisData {
    power_trend: f32,        // 功耗趋势 (W/s)
    efficiency_trend: f32,   // 效率趋势
    predicted_lifetime: u32, // 预测续航时间 (秒)
}

impl PowerAnalyzer {
    fn analyze_power_states(&mut self) -> Result<StateAnalysisData, &'static str> {
        let samples = &self.measurement_system.measurement_buffer;
        if samples.len() < 10 {
            return Err("Insufficient data for state analysis");
        }
        
        // 简化的状态检测：基于功耗阈值
        let mut active_samples = heapless::Vec::<f32, 1000>::new();
        let mut idle_samples = heapless::Vec::<f32, 1000>::new();
        let mut sleep_samples = heapless::Vec::<f32, 1000>::new();
        let mut state_transitions = 0;
        let mut last_state = PowerState::Active;
        
        for sample in samples {
            let current_state = if sample.power > 0.1 {
                PowerState::Active
            } else if sample.power > 0.01 {
                PowerState::Idle
            } else {
                PowerState::Sleep
            };
            
            if current_state != last_state {
                state_transitions += 1;
                last_state = current_state;
            }
            
            match current_state {
                PowerState::Active => { let _ = active_samples.push(sample.power); }
                PowerState::Idle => { let _ = idle_samples.push(sample.power); }
                PowerState::Sleep => { let _ = sleep_samples.push(sample.power); }
                _ => {}
            }
        }
        
        let active_power = if active_samples.is_empty() { 0.0 } else {
            active_samples.iter().sum::<f32>() / active_samples.len() as f32
        };
        
        let idle_power = if idle_samples.is_empty() { 0.0 } else {
            idle_samples.iter().sum::<f32>() / idle_samples.len() as f32
        };
        
        let sleep_power = if sleep_samples.is_empty() { 0.0 } else {
            sleep_samples.iter().sum::<f32>() / sleep_samples.len() as f32
        };
        
        Ok(StateAnalysisData {
            active_power,
            idle_power,
            sleep_power,
            state_transitions,
        })
    }
    
    fn analyze_efficiency(&mut self) -> Result<EfficiencyAnalysisData, &'static str> {
        let stats = self.measurement_system.calculate_statistics();
        
        if stats.sample_count == 0 {
            return Err("No measurement data available");
        }
        
        // 计算功耗效率（简化模型）
        let power_efficiency = if stats.max_power > 0.0 {
            stats.avg_power / stats.max_power
        } else {
            0.0
        };
        
        // 计算能耗效率
        let energy_efficiency = if stats.total_energy > 0.0 {
            (stats.measurement_duration as f32 / 1000.0) / stats.total_energy
        } else {
            0.0
        };
        
        // 计算性能功耗比（需要性能指标）
        let performance_per_watt = self.calculate_performance_per_watt();
        
        Ok(EfficiencyAnalysisData {
            power_efficiency,
            energy_efficiency,
            performance_per_watt,
        })
    }
    
    fn calculate_performance_per_watt(&self) -> f32 {
        // 简化的性能功耗比计算
        // 实际实现需要结合具体的性能指标
        1.0
    }
    
    fn predict_battery_lifetime(&self, battery_capacity_wh: f32) -> u32 {
        let stats = self.measurement_system.calculate_statistics();
        
        if stats.avg_power <= 0.0 {
            return u32::MAX;
        }
        
        // 预测续航时间 = 电池容量 / 平均功耗
        let lifetime_hours = battery_capacity_wh / stats.avg_power;
        (lifetime_hours * 3600.0) as u32 // 转换为秒
    }
}

## 6. 总结

功耗管理是嵌入式系统设计中的关键技术，涉及硬件设计、软件优化、系统架构等多个层面。

### 关键要点

1. **功耗理论**: 理解静态功耗和动态功耗的来源和计算方法
2. **设计原则**: 掌握低功耗设计的基本原则和最佳实践
3. **睡眠管理**: 合理使用各种睡眠模式和唤醒机制
4. **DVFS技术**: 动态调节电压和频率以优化功耗
5. **测量分析**: 建立完整的功耗测量和分析体系
6. **智能优化**: 应用预测算法和自适应策略

### 设计策略

- **分层优化**: 从硬件到软件的全栈功耗优化
- **动态管理**: 根据工作负载动态调整功耗策略
- **预测控制**: 利用历史数据预测和优化功耗
- **多目标平衡**: 在性能、功耗、实时性之间找到平衡
- **系统级思考**: 考虑整个系统的功耗优化

### 实践建议

- 建立功耗测量基准和监控体系
- 使用专业的功耗分析工具
- 在设计早期就考虑功耗优化
- 定期评估和优化功耗策略
- 学习和应用最新的低功耗技术

### 下一步

完成功耗管理学习后，建议继续学习：
- [启动过程和引导](./06-boot-process.md)
- [功耗管理实践](../../11-power-management/README.md)
- [无线通信功耗优化](../../14-wireless-communication/README.md)