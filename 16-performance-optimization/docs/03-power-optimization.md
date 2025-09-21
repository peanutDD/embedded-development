# 功耗优化

本章详细介绍嵌入式Rust开发中的功耗优化技术，包括低功耗模式、时钟管理、外设控制等。

## 1. 低功耗模式

### 1.1 STM32低功耗模式

```rust
use stm32f4xx_hal::{
    pac::{PWR, RCC, SCB},
    prelude::*,
    rcc::Clocks,
};
use cortex_m::peripheral::NVIC;

#[derive(Debug, Clone, Copy)]
enum PowerMode {
    Run,        // 运行模式
    Sleep,      // 睡眠模式
    Stop,       // 停止模式
    Standby,    // 待机模式
}

struct PowerManager {
    pwr: PWR,
    rcc: RCC,
    scb: SCB,
    current_mode: PowerMode,
    wakeup_sources: u32,
}

impl PowerManager {
    fn new(pwr: PWR, rcc: RCC, scb: SCB) -> Self {
        Self {
            pwr,
            rcc,
            scb,
            current_mode: PowerMode::Run,
            wakeup_sources: 0,
        }
    }
    
    fn enter_sleep_mode(&mut self) {
        // 配置睡眠模式
        self.scb.scr.modify(|_, w| w.sleepdeep().clear_bit());
        
        // 等待中断
        cortex_m::asm::wfi();
        
        self.current_mode = PowerMode::Sleep;
    }
    
    fn enter_stop_mode(&mut self) {
        // 使能PWR时钟
        self.rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
        
        // 配置停止模式
        self.pwr.cr.modify(|_, w| {
            w.pdds().clear_bit()    // 进入停止模式而非待机模式
             .lpds().set_bit()      // 低功耗深度睡眠
        });
        
        // 配置深度睡眠
        self.scb.scr.modify(|_, w| w.sleepdeep().set_bit());
        
        // 等待中断
        cortex_m::asm::wfi();
        
        self.current_mode = PowerMode::Stop;
    }
    
    fn enter_standby_mode(&mut self) {
        // 使能PWR时钟
        self.rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
        
        // 清除唤醒标志
        self.pwr.cr.modify(|_, w| w.cwuf().set_bit());
        
        // 配置待机模式
        self.pwr.cr.modify(|_, w| {
            w.pdds().set_bit()      // 进入待机模式
             .cwuf().set_bit()      // 清除唤醒标志
        });
        
        // 配置深度睡眠
        self.scb.scr.modify(|_, w| w.sleepdeep().set_bit());
        
        // 等待中断
        cortex_m::asm::wfi();
        
        self.current_mode = PowerMode::Standby;
    }
    
    fn configure_wakeup_pin(&mut self) {
        // 使能PWR时钟
        self.rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
        
        // 使能唤醒引脚
        self.pwr.csr.modify(|_, w| w.ewup().set_bit());
    }
    
    fn get_wakeup_reason(&self) -> WakeupReason {
        let csr = self.pwr.csr.read();
        
        if csr.wuf().bit_is_set() {
            WakeupReason::WakeupPin
        } else {
            WakeupReason::Other
        }
    }
    
    fn restore_from_stop(&mut self, clocks: &Clocks) {
        // 重新配置系统时钟（停止模式会关闭HSE和PLL）
        self.configure_clocks_after_stop(clocks);
        
        self.current_mode = PowerMode::Run;
    }
    
    fn configure_clocks_after_stop(&mut self, clocks: &Clocks) {
        // 重新启动HSE
        self.rcc.cr.modify(|_, w| w.hseon().set_bit());
        while self.rcc.cr.read().hserdy().bit_is_clear() {}
        
        // 重新配置PLL
        // 这里需要根据具体的时钟配置来设置
    }
}

#[derive(Debug, Clone, Copy)]
enum WakeupReason {
    WakeupPin,
    RTC,
    Watchdog,
    Other,
}
```

### 1.2 动态功耗管理

```rust
use heapless::Vec;

struct DynamicPowerManager {
    power_states: Vec<PowerState, 8>,
    current_state: usize,
    idle_counter: u32,
    activity_threshold: u32,
}

#[derive(Debug, Clone)]
struct PowerState {
    name: &'static str,
    cpu_freq: u32,
    voltage: f32,
    power_consumption: f32,
    wakeup_latency: u32,
    conditions: PowerConditions,
}

#[derive(Debug, Clone)]
struct PowerConditions {
    min_idle_time: u32,
    max_cpu_load: u8,
    required_peripherals: u32,
}

impl DynamicPowerManager {
    fn new() -> Self {
        let mut power_states = Vec::new();
        
        // 定义不同的功耗状态
        power_states.push(PowerState {
            name: "High Performance",
            cpu_freq: 168_000_000,
            voltage: 3.3,
            power_consumption: 100.0,
            wakeup_latency: 0,
            conditions: PowerConditions {
                min_idle_time: 0,
                max_cpu_load: 100,
                required_peripherals: 0xFFFFFFFF,
            },
        }).ok();
        
        power_states.push(PowerState {
            name: "Normal",
            cpu_freq: 84_000_000,
            voltage: 3.0,
            power_consumption: 50.0,
            wakeup_latency: 10,
            conditions: PowerConditions {
                min_idle_time: 100,
                max_cpu_load: 70,
                required_peripherals: 0x0000FFFF,
            },
        }).ok();
        
        power_states.push(PowerState {
            name: "Low Power",
            cpu_freq: 16_000_000,
            voltage: 2.7,
            power_consumption: 10.0,
            wakeup_latency: 100,
            conditions: PowerConditions {
                min_idle_time: 1000,
                max_cpu_load: 30,
                required_peripherals: 0x000000FF,
            },
        }).ok();
        
        Self {
            power_states,
            current_state: 0,
            idle_counter: 0,
            activity_threshold: 1000,
        }
    }
    
    fn update(&mut self, system_load: SystemLoad) {
        // 更新空闲计数器
        if system_load.cpu_usage < 10 {
            self.idle_counter += 1;
        } else {
            self.idle_counter = 0;
        }
        
        // 选择最佳功耗状态
        let optimal_state = self.select_optimal_state(&system_load);
        
        if optimal_state != self.current_state {
            self.transition_to_state(optimal_state);
        }
    }
    
    fn select_optimal_state(&self, load: &SystemLoad) -> usize {
        for (index, state) in self.power_states.iter().enumerate().rev() {
            if self.can_enter_state(state, load) {
                return index;
            }
        }
        0 // 默认返回最高性能状态
    }
    
    fn can_enter_state(&self, state: &PowerState, load: &SystemLoad) -> bool {
        self.idle_counter >= state.conditions.min_idle_time &&
        load.cpu_usage <= state.conditions.max_cpu_load &&
        (load.active_peripherals & state.conditions.required_peripherals) == load.active_peripherals
    }
    
    fn transition_to_state(&mut self, new_state: usize) {
        if new_state < self.power_states.len() {
            cortex_m_log::println!(
                "Power transition: {} -> {}",
                self.power_states[self.current_state].name,
                self.power_states[new_state].name
            );
            
            self.current_state = new_state;
            self.apply_power_state();
        }
    }
    
    fn apply_power_state(&self) {
        let state = &self.power_states[self.current_state];
        
        // 调整CPU频率
        self.set_cpu_frequency(state.cpu_freq);
        
        // 调整电压（如果支持）
        self.set_voltage(state.voltage);
        
        // 禁用不需要的外设
        self.configure_peripherals(state.conditions.required_peripherals);
    }
    
    fn set_cpu_frequency(&self, freq: u32) {
        // 实际的频率调整代码
        cortex_m_log::println!("Setting CPU frequency to {} Hz", freq);
    }
    
    fn set_voltage(&self, voltage: f32) {
        // 实际的电压调整代码（需要硬件支持）
        cortex_m_log::println!("Setting voltage to {} V", voltage);
    }
    
    fn configure_peripherals(&self, required: u32) {
        // 禁用不需要的外设时钟
        cortex_m_log::println!("Configuring peripherals: 0x{:08X}", required);
    }
    
    fn get_current_power_consumption(&self) -> f32 {
        self.power_states[self.current_state].power_consumption
    }
}

#[derive(Debug)]
struct SystemLoad {
    cpu_usage: u8,
    active_peripherals: u32,
    pending_tasks: u8,
    network_activity: bool,
}
```

### 1.3 外设功耗管理

```rust
use stm32f4xx_hal::pac::{RCC, GPIOA, GPIOB, GPIOC};

struct PeripheralPowerManager {
    rcc: RCC,
    enabled_peripherals: u32,
    power_domains: [PowerDomain; 8],
}

#[derive(Debug, Clone)]
struct PowerDomain {
    name: &'static str,
    peripherals: u32,
    power_consumption: f32,
    enabled: bool,
    reference_count: u8,
}

impl PeripheralPowerManager {
    fn new(rcc: RCC) -> Self {
        let power_domains = [
            PowerDomain {
                name: "GPIO",
                peripherals: 0x0000000F, // GPIOA-D
                power_consumption: 5.0,
                enabled: false,
                reference_count: 0,
            },
            PowerDomain {
                name: "UART",
                peripherals: 0x000000F0, // UART1-4
                power_consumption: 3.0,
                enabled: false,
                reference_count: 0,
            },
            PowerDomain {
                name: "SPI",
                peripherals: 0x00000F00, // SPI1-4
                power_consumption: 2.0,
                enabled: false,
                reference_count: 0,
            },
            PowerDomain {
                name: "I2C",
                peripherals: 0x0000F000, // I2C1-4
                power_consumption: 1.5,
                enabled: false,
                reference_count: 0,
            },
            PowerDomain {
                name: "ADC",
                peripherals: 0x000F0000, // ADC1-3
                power_consumption: 8.0,
                enabled: false,
                reference_count: 0,
            },
            PowerDomain {
                name: "DAC",
                peripherals: 0x00F00000, // DAC
                power_consumption: 4.0,
                enabled: false,
                reference_count: 0,
            },
            PowerDomain {
                name: "Timer",
                peripherals: 0x0F000000, // TIM1-4
                power_consumption: 2.5,
                enabled: false,
                reference_count: 0,
            },
            PowerDomain {
                name: "DMA",
                peripherals: 0xF0000000, // DMA1-2
                power_consumption: 6.0,
                enabled: false,
                reference_count: 0,
            },
        ];
        
        Self {
            rcc,
            enabled_peripherals: 0,
            power_domains,
        }
    }
    
    fn enable_peripheral(&mut self, peripheral_id: u32) -> Result<(), ()> {
        for domain in &mut self.power_domains {
            if domain.peripherals & peripheral_id != 0 {
                domain.reference_count += 1;
                
                if !domain.enabled {
                    self.enable_power_domain(domain)?;
                }
                
                self.enabled_peripherals |= peripheral_id;
                return Ok(());
            }
        }
        Err(())
    }
    
    fn disable_peripheral(&mut self, peripheral_id: u32) -> Result<(), ()> {
        for domain in &mut self.power_domains {
            if domain.peripherals & peripheral_id != 0 {
                if domain.reference_count > 0 {
                    domain.reference_count -= 1;
                    
                    if domain.reference_count == 0 {
                        self.disable_power_domain(domain)?;
                    }
                    
                    self.enabled_peripherals &= !peripheral_id;
                    return Ok(());
                }
            }
        }
        Err(())
    }
    
    fn enable_power_domain(&mut self, domain: &mut PowerDomain) -> Result<(), ()> {
        cortex_m_log::println!("Enabling power domain: {}", domain.name);
        
        // 根据外设类型启用相应的时钟
        match domain.name {
            "GPIO" => {
                self.rcc.ahb1enr.modify(|_, w| {
                    w.gpioaen().set_bit()
                     .gpioben().set_bit()
                     .gpiocen().set_bit()
                     .gpioden().set_bit()
                });
            },
            "UART" => {
                self.rcc.apb2enr.modify(|_, w| w.usart1en().set_bit());
                self.rcc.apb1enr.modify(|_, w| {
                    w.usart2en().set_bit()
                     .usart3en().set_bit()
                });
            },
            "SPI" => {
                self.rcc.apb2enr.modify(|_, w| w.spi1en().set_bit());
                self.rcc.apb1enr.modify(|_, w| {
                    w.spi2en().set_bit()
                     .spi3en().set_bit()
                });
            },
            "I2C" => {
                self.rcc.apb1enr.modify(|_, w| {
                    w.i2c1en().set_bit()
                     .i2c2en().set_bit()
                     .i2c3en().set_bit()
                });
            },
            "ADC" => {
                self.rcc.apb2enr.modify(|_, w| {
                    w.adc1en().set_bit()
                     .adc2en().set_bit()
                     .adc3en().set_bit()
                });
            },
            "DAC" => {
                self.rcc.apb1enr.modify(|_, w| w.dacen().set_bit());
            },
            "Timer" => {
                self.rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
                self.rcc.apb1enr.modify(|_, w| {
                    w.tim2en().set_bit()
                     .tim3en().set_bit()
                     .tim4en().set_bit()
                });
            },
            "DMA" => {
                self.rcc.ahb1enr.modify(|_, w| {
                    w.dma1en().set_bit()
                     .dma2en().set_bit()
                });
            },
            _ => return Err(()),
        }
        
        domain.enabled = true;
        Ok(())
    }
    
    fn disable_power_domain(&mut self, domain: &mut PowerDomain) -> Result<(), ()> {
        cortex_m_log::println!("Disabling power domain: {}", domain.name);
        
        // 根据外设类型禁用相应的时钟
        match domain.name {
            "GPIO" => {
                self.rcc.ahb1enr.modify(|_, w| {
                    w.gpioaen().clear_bit()
                     .gpioben().clear_bit()
                     .gpiocen().clear_bit()
                     .gpioden().clear_bit()
                });
            },
            "UART" => {
                self.rcc.apb2enr.modify(|_, w| w.usart1en().clear_bit());
                self.rcc.apb1enr.modify(|_, w| {
                    w.usart2en().clear_bit()
                     .usart3en().clear_bit()
                });
            },
            // 其他外设类似...
            _ => return Err(()),
        }
        
        domain.enabled = false;
        Ok(())
    }
    
    fn get_total_power_consumption(&self) -> f32 {
        self.power_domains
            .iter()
            .filter(|d| d.enabled)
            .map(|d| d.power_consumption)
            .sum()
    }
    
    fn optimize_power_usage(&mut self) {
        // 检查未使用的外设并禁用它们
        for domain in &mut self.power_domains {
            if domain.enabled && domain.reference_count == 0 {
                self.disable_power_domain(domain).ok();
            }
        }
    }
}
```

## 2. 时钟管理

### 2.1 动态时钟调整

```rust
use stm32f4xx_hal::rcc::{Clocks, CFGR};

struct ClockManager {
    cfgr: CFGR,
    current_config: ClockConfig,
    available_configs: [ClockConfig; 4],
}

#[derive(Debug, Clone, Copy)]
struct ClockConfig {
    name: &'static str,
    sysclk: u32,
    hclk: u32,
    pclk1: u32,
    pclk2: u32,
    power_consumption: f32,
    performance_level: u8,
}

impl ClockManager {
    fn new(cfgr: CFGR) -> Self {
        let available_configs = [
            ClockConfig {
                name: "High Performance",
                sysclk: 168_000_000,
                hclk: 168_000_000,
                pclk1: 42_000_000,
                pclk2: 84_000_000,
                power_consumption: 100.0,
                performance_level: 100,
            },
            ClockConfig {
                name: "Balanced",
                sysclk: 84_000_000,
                hclk: 84_000_000,
                pclk1: 42_000_000,
                pclk2: 84_000_000,
                power_consumption: 60.0,
                performance_level: 70,
            },
            ClockConfig {
                name: "Power Save",
                sysclk: 42_000_000,
                hclk: 42_000_000,
                pclk1: 21_000_000,
                pclk2: 42_000_000,
                power_consumption: 30.0,
                performance_level: 40,
            },
            ClockConfig {
                name: "Ultra Low Power",
                sysclk: 16_000_000,
                hclk: 16_000_000,
                pclk1: 16_000_000,
                pclk2: 16_000_000,
                power_consumption: 10.0,
                performance_level: 20,
            },
        ];
        
        Self {
            cfgr,
            current_config: available_configs[0],
            available_configs,
        }
    }
    
    fn set_clock_config(&mut self, config_index: usize) -> Result<(), ()> {
        if config_index >= self.available_configs.len() {
            return Err(());
        }
        
        let new_config = self.available_configs[config_index];
        
        cortex_m_log::println!(
            "Switching clock config: {} -> {}",
            self.current_config.name,
            new_config.name
        );
        
        // 实际的时钟配置切换
        self.apply_clock_config(&new_config)?;
        
        self.current_config = new_config;
        Ok(())
    }
    
    fn apply_clock_config(&mut self, config: &ClockConfig) -> Result<(), ()> {
        // 这里需要实际的时钟配置代码
        // 注意：动态时钟切换需要小心处理，避免系统不稳定
        
        cortex_m_log::println!("Applying clock config: {}", config.name);
        cortex_m_log::println!("  SYSCLK: {} Hz", config.sysclk);
        cortex_m_log::println!("  HCLK: {} Hz", config.hclk);
        cortex_m_log::println!("  PCLK1: {} Hz", config.pclk1);
        cortex_m_log::println!("  PCLK2: {} Hz", config.pclk2);
        
        Ok(())
    }
    
    fn select_optimal_config(&self, required_performance: u8) -> usize {
        for (index, config) in self.available_configs.iter().enumerate() {
            if config.performance_level >= required_performance {
                return index;
            }
        }
        0 // 默认返回最高性能配置
    }
    
    fn get_current_power_consumption(&self) -> f32 {
        self.current_config.power_consumption
    }
}

// 自适应时钟管理
struct AdaptiveClockManager {
    clock_manager: ClockManager,
    performance_monitor: PerformanceMonitor,
    adjustment_timer: u32,
}

impl AdaptiveClockManager {
    fn new(cfgr: CFGR) -> Self {
        Self {
            clock_manager: ClockManager::new(cfgr),
            performance_monitor: PerformanceMonitor::new(),
            adjustment_timer: 0,
        }
    }
    
    fn update(&mut self) {
        self.adjustment_timer += 1;
        
        // 每100ms检查一次性能需求
        if self.adjustment_timer >= 100 {
            self.adjustment_timer = 0;
            
            let performance_metrics = self.performance_monitor.get_metrics();
            let required_performance = self.calculate_required_performance(&performance_metrics);
            
            let optimal_config = self.clock_manager.select_optimal_config(required_performance);
            
            // 如果当前配置不是最优的，则切换
            if optimal_config != self.get_current_config_index() {
                self.clock_manager.set_clock_config(optimal_config).ok();
            }
        }
    }
    
    fn calculate_required_performance(&self, metrics: &PerformanceMetrics) -> u8 {
        let mut required = 20; // 基础性能需求
        
        // 根据CPU使用率调整
        if metrics.cpu_usage > 80 {
            required = 100;
        } else if metrics.cpu_usage > 60 {
            required = 70;
        } else if metrics.cpu_usage > 40 {
            required = 40;
        }
        
        // 根据任务优先级调整
        if metrics.high_priority_tasks > 0 {
            required = core::cmp::max(required, 70);
        }
        
        // 根据实时性要求调整
        if metrics.realtime_deadline_misses > 0 {
            required = 100;
        }
        
        required
    }
    
    fn get_current_config_index(&self) -> usize {
        for (index, config) in self.clock_manager.available_configs.iter().enumerate() {
            if config.name == self.clock_manager.current_config.name {
                return index;
            }
        }
        0
    }
}

struct PerformanceMonitor {
    cpu_usage: u8,
    task_count: u8,
    interrupt_count: u32,
    deadline_misses: u8,
}

impl PerformanceMonitor {
    fn new() -> Self {
        Self {
            cpu_usage: 0,
            task_count: 0,
            interrupt_count: 0,
            deadline_misses: 0,
        }
    }
    
    fn get_metrics(&self) -> PerformanceMetrics {
        PerformanceMetrics {
            cpu_usage: self.cpu_usage,
            high_priority_tasks: self.task_count,
            realtime_deadline_misses: self.deadline_misses,
        }
    }
    
    fn update_cpu_usage(&mut self, usage: u8) {
        self.cpu_usage = usage;
    }
    
    fn record_deadline_miss(&mut self) {
        self.deadline_misses = self.deadline_misses.saturating_add(1);
    }
}

struct PerformanceMetrics {
    cpu_usage: u8,
    high_priority_tasks: u8,
    realtime_deadline_misses: u8,
}
```

### 2.2 外设时钟门控

```rust
struct PeripheralClockGating {
    rcc: RCC,
    gating_policy: GatingPolicy,
    usage_counters: [u32; 32],
    last_access_time: [u32; 32],
}

#[derive(Debug, Clone, Copy)]
enum GatingPolicy {
    Aggressive,  // 立即关闭未使用的时钟
    Conservative, // 延迟关闭时钟
    Manual,      // 手动控制
}

impl PeripheralClockGating {
    fn new(rcc: RCC, policy: GatingPolicy) -> Self {
        Self {
            rcc,
            gating_policy: policy,
            usage_counters: [0; 32],
            last_access_time: [0; 32],
        }
    }
    
    fn enable_peripheral_clock(&mut self, peripheral_id: u8) {
        if peripheral_id < 32 {
            self.usage_counters[peripheral_id as usize] += 1;
            self.last_access_time[peripheral_id as usize] = self.get_current_time();
            
            self.apply_clock_enable(peripheral_id);
        }
    }
    
    fn disable_peripheral_clock(&mut self, peripheral_id: u8) {
        if peripheral_id < 32 {
            if self.usage_counters[peripheral_id as usize] > 0 {
                self.usage_counters[peripheral_id as usize] -= 1;
                
                if self.usage_counters[peripheral_id as usize] == 0 {
                    match self.gating_policy {
                        GatingPolicy::Aggressive => {
                            self.apply_clock_disable(peripheral_id);
                        },
                        GatingPolicy::Conservative => {
                            // 延迟禁用，在update()中处理
                        },
                        GatingPolicy::Manual => {
                            // 不自动禁用
                        },
                    }
                }
            }
        }
    }
    
    fn update(&mut self) {
        if matches!(self.gating_policy, GatingPolicy::Conservative) {
            let current_time = self.get_current_time();
            const IDLE_TIMEOUT: u32 = 1000; // 1秒超时
            
            for (id, &last_access) in self.last_access_time.iter().enumerate() {
                if self.usage_counters[id] == 0 && 
                   current_time.saturating_sub(last_access) > IDLE_TIMEOUT {
                    self.apply_clock_disable(id as u8);
                }
            }
        }
    }
    
    fn apply_clock_enable(&mut self, peripheral_id: u8) {
        match peripheral_id {
            0 => self.rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit()),
            1 => self.rcc.ahb1enr.modify(|_, w| w.gpioben().set_bit()),
            2 => self.rcc.ahb1enr.modify(|_, w| w.gpiocen().set_bit()),
            3 => self.rcc.ahb1enr.modify(|_, w| w.gpioden().set_bit()),
            4 => self.rcc.apb2enr.modify(|_, w| w.usart1en().set_bit()),
            5 => self.rcc.apb1enr.modify(|_, w| w.usart2en().set_bit()),
            6 => self.rcc.apb2enr.modify(|_, w| w.spi1en().set_bit()),
            7 => self.rcc.apb1enr.modify(|_, w| w.spi2en().set_bit()),
            8 => self.rcc.apb1enr.modify(|_, w| w.i2c1en().set_bit()),
            9 => self.rcc.apb1enr.modify(|_, w| w.i2c2en().set_bit()),
            10 => self.rcc.apb2enr.modify(|_, w| w.adc1en().set_bit()),
            11 => self.rcc.apb1enr.modify(|_, w| w.dacen().set_bit()),
            12 => self.rcc.apb2enr.modify(|_, w| w.tim1en().set_bit()),
            13 => self.rcc.apb1enr.modify(|_, w| w.tim2en().set_bit()),
            14 => self.rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit()),
            15 => self.rcc.ahb1enr.modify(|_, w| w.dma2en().set_bit()),
            _ => {},
        }
        
        cortex_m_log::println!("Enabled clock for peripheral {}", peripheral_id);
    }
    
    fn apply_clock_disable(&mut self, peripheral_id: u8) {
        match peripheral_id {
            0 => self.rcc.ahb1enr.modify(|_, w| w.gpioaen().clear_bit()),
            1 => self.rcc.ahb1enr.modify(|_, w| w.gpioben().clear_bit()),
            2 => self.rcc.ahb1enr.modify(|_, w| w.gpiocen().clear_bit()),
            3 => self.rcc.ahb1enr.modify(|_, w| w.gpioden().clear_bit()),
            4 => self.rcc.apb2enr.modify(|_, w| w.usart1en().clear_bit()),
            5 => self.rcc.apb1enr.modify(|_, w| w.usart2en().clear_bit()),
            6 => self.rcc.apb2enr.modify(|_, w| w.spi1en().clear_bit()),
            7 => self.rcc.apb1enr.modify(|_, w| w.spi2en().clear_bit()),
            8 => self.rcc.apb1enr.modify(|_, w| w.i2c1en().clear_bit()),
            9 => self.rcc.apb1enr.modify(|_, w| w.i2c2en().clear_bit()),
            10 => self.rcc.apb2enr.modify(|_, w| w.adc1en().clear_bit()),
            11 => self.rcc.apb1enr.modify(|_, w| w.dacen().clear_bit()),
            12 => self.rcc.apb2enr.modify(|_, w| w.tim1en().clear_bit()),
            13 => self.rcc.apb1enr.modify(|_, w| w.tim2en().clear_bit()),
            14 => self.rcc.ahb1enr.modify(|_, w| w.dma1en().clear_bit()),
            15 => self.rcc.ahb1enr.modify(|_, w| w.dma2en().clear_bit()),
            _ => {},
        }
        
        cortex_m_log::println!("Disabled clock for peripheral {}", peripheral_id);
    }
    
    fn get_current_time(&self) -> u32 {
        // 返回当前时间戳（毫秒）
        cortex_m::peripheral::DWT::cycle_count() / 168_000 // 假设168MHz时钟
    }
    
    fn get_power_savings(&self) -> f32 {
        let disabled_count = self.usage_counters.iter().filter(|&&count| count == 0).count();
        disabled_count as f32 * 2.0 // 每个禁用的外设节省约2mA
    }
}
```

## 3. 电源管理单元(PMU)

### 3.1 电压调节

```rust
struct VoltageRegulator {
    target_voltage: f32,
    current_voltage: f32,
    regulation_mode: RegulationMode,
    load_current: f32,
}

#[derive(Debug, Clone, Copy)]
enum RegulationMode {
    HighEfficiency,  // 高效率模式
    LowNoise,       // 低噪声模式
    FastTransient,  // 快速瞬态响应
    PowerSave,      // 省电模式
}

impl VoltageRegulator {
    fn new() -> Self {
        Self {
            target_voltage: 3.3,
            current_voltage: 3.3,
            regulation_mode: RegulationMode::HighEfficiency,
            load_current: 0.0,
        }
    }
    
    fn set_voltage(&mut self, voltage: f32) -> Result<(), ()> {
        if voltage < 1.8 || voltage > 3.6 {
            return Err(());
        }
        
        cortex_m_log::println!("Setting voltage: {} V -> {} V", self.current_voltage, voltage);
        
        // 渐进式电压调整，避免突变
        let step = if voltage > self.current_voltage { 0.1 } else { -0.1 };
        
        while (self.current_voltage - voltage).abs() > 0.05 {
            self.current_voltage += step;
            self.apply_voltage_setting(self.current_voltage);
            
            // 等待电压稳定
            cortex_m::asm::delay(1000);
        }
        
        self.current_voltage = voltage;
        self.target_voltage = voltage;
        
        Ok(())
    }
    
    fn set_regulation_mode(&mut self, mode: RegulationMode) {
        self.regulation_mode = mode;
        self.apply_regulation_mode();
    }
    
    fn apply_voltage_setting(&self, voltage: f32) {
        // 实际的电压设置代码（通过DAC或PWM控制）
        let dac_value = ((voltage - 1.8) / (3.6 - 1.8) * 4095.0) as u16;
        cortex_m_log::println!("DAC value for {} V: {}", voltage, dac_value);
    }
    
    fn apply_regulation_mode(&self) {
        match self.regulation_mode {
            RegulationMode::HighEfficiency => {
                cortex_m_log::println!("Switching to high efficiency mode");
                // 配置为高效率模式
            },
            RegulationMode::LowNoise => {
                cortex_m_log::println!("Switching to low noise mode");
                // 配置为低噪声模式
            },
            RegulationMode::FastTransient => {
                cortex_m_log::println!("Switching to fast transient mode");
                // 配置为快速瞬态响应模式
            },
            RegulationMode::PowerSave => {
                cortex_m_log::println!("Switching to power save mode");
                // 配置为省电模式
            },
        }
    }
    
    fn monitor_load(&mut self, current: f32) {
        self.load_current = current;
        
        // 根据负载自动调整调节模式
        if current > 500.0 {
            // 高负载，使用快速瞬态响应
            self.set_regulation_mode(RegulationMode::FastTransient);
        } else if current < 50.0 {
            // 低负载，使用省电模式
            self.set_regulation_mode(RegulationMode::PowerSave);
        } else {
            // 中等负载，使用高效率模式
            self.set_regulation_mode(RegulationMode::HighEfficiency);
        }
    }
    
    fn get_efficiency(&self) -> f32 {
        // 计算调节器效率
        match self.regulation_mode {
            RegulationMode::HighEfficiency => 0.95,
            RegulationMode::LowNoise => 0.85,
            RegulationMode::FastTransient => 0.90,
            RegulationMode::PowerSave => 0.98,
        }
    }
    
    fn get_power_consumption(&self) -> f32 {
        let output_power = self.current_voltage * self.load_current;
        let efficiency = self.get_efficiency();
        
        output_power / efficiency
    }
}
```

### 3.2 电池管理

```rust
struct BatteryManager {
    voltage: f32,
    current: f32,
    capacity: f32,
    temperature: f32,
    charge_state: ChargeState,
    protection_flags: u8,
}

#[derive(Debug, Clone, Copy)]
enum ChargeState {
    Charging,
    Discharging,
    Full,
    Empty,
    Error,
}

impl BatteryManager {
    fn new() -> Self {
        Self {
            voltage: 3.7,
            current: 0.0,
            capacity: 100.0,
            temperature: 25.0,
            charge_state: ChargeState::Discharging,
            protection_flags: 0,
        }
    }
    
    fn update_measurements(&mut self, voltage: f32, current: f32, temperature: f32) {
        self.voltage = voltage;
        self.current = current;
        self.temperature = temperature;
        
        self.update_charge_state();
        self.check_protection();
        self.estimate_capacity();
    }
    
    fn update_charge_state(&mut self) {
        if self.voltage > 4.1 && self.current.abs() < 0.1 {
            self.charge_state = ChargeState::Full;
        } else if self.voltage < 3.0 {
            self.charge_state = ChargeState::Empty;
        } else if self.current > 0.1 {
            self.charge_state = ChargeState::Charging;
        } else if self.current < -0.1 {
            self.charge_state = ChargeState::Discharging;
        }
    }
    
    fn check_protection(&mut self) {
        self.protection_flags = 0;
        
        // 过压保护
        if self.voltage > 4.3 {
            self.protection_flags |= 0x01;
        }
        
        // 欠压保护
        if self.voltage < 2.5 {
            self.protection_flags |= 0x02;
        }
        
        // 过流保护
        if self.current.abs() > 2.0 {
            self.protection_flags |= 0x04;
        }
        
        // 过温保护
        if self.temperature > 60.0 || self.temperature < -10.0 {
            self.protection_flags |= 0x08;
        }
        
        if self.protection_flags != 0 {
            self.charge_state = ChargeState::Error;
            cortex_m_log::println!("Battery protection triggered: 0x{:02X}", self.protection_flags);
        }
    }
    
    fn estimate_capacity(&mut self) {
        // 基于电压的容量估算（简化模型）
        if self.voltage > 4.0 {
            self.capacity = 90.0 + (self.voltage - 4.0) * 100.0;
        } else if self.voltage > 3.7 {
            self.capacity = 50.0 + (self.voltage - 3.7) * 133.3;
        } else if self.voltage > 3.4 {
            self.capacity = 20.0 + (self.voltage - 3.4) * 100.0;
        } else if self.voltage > 3.0 {
            self.capacity = (self.voltage - 3.0) * 50.0;
        } else {
            self.capacity = 0.0;
        }
        
        // 限制在0-100%范围内
        self.capacity = self.capacity.clamp(0.0, 100.0);
    }
    
    fn get_remaining_time(&self) -> Option<u32> {
        if matches!(self.charge_state, ChargeState::Discharging) && self.current < 0.0 {
            let remaining_capacity = self.capacity / 100.0 * 2000.0; // 假设2000mAh电池
            let discharge_rate = -self.current;
            
            if discharge_rate > 0.0 {
                Some((remaining_capacity / discharge_rate * 3600.0) as u32) // 秒
            } else {
                None
            }
        } else {
            None
        }
    }
    
    fn should_enter_low_power(&self) -> bool {
        self.capacity < 20.0 || matches!(self.charge_state, ChargeState::Empty)
    }
    
    fn get_recommended_power_mode(&self) -> PowerMode {
        match self.capacity as u8 {
            0..=10 => PowerMode::Standby,
            11..=30 => PowerMode::Stop,
            31..=60 => PowerMode::Sleep,
            _ => PowerMode::Run,
        }
    }
}
```

## 4. 功耗监控

### 4.1 实时功耗测量

```rust
struct PowerMonitor {
    voltage_adc: u16,
    current_adc: u16,
    power_history: heapless::Vec<PowerSample, 100>,
    calibration: CalibrationData,
}

#[derive(Debug, Clone, Copy)]
struct PowerSample {
    timestamp: u32,
    voltage: f32,
    current: f32,
    power: f32,
}

#[derive(Debug, Clone, Copy)]
struct CalibrationData {
    voltage_offset: f32,
    voltage_scale: f32,
    current_offset: f32,
    current_scale: f32,
}

impl PowerMonitor {
    fn new() -> Self {
        Self {
            voltage_adc: 0,
            current_adc: 0,
            power_history: heapless::Vec::new(),
            calibration: CalibrationData {
                voltage_offset: 0.0,
                voltage_scale: 3.3 / 4095.0,
                current_offset: 0.0,
                current_scale: 1.0 / 4095.0,
            },
        }
    }
    
    fn update_measurements(&mut self, voltage_adc: u16, current_adc: u16) {
        self.voltage_adc = voltage_adc;
        self.current_adc = current_adc;
        
        let voltage = self.convert_voltage(voltage_adc);
        let current = self.convert_current(current_adc);
        let power = voltage * current;
        
        let sample = PowerSample {
            timestamp: self.get_timestamp(),
            voltage,
            current,
            power,
        };
        
        // 添加到历史记录
        if self.power_history.is_full() {
            self.power_history.pop();
        }
        self.power_history.push(sample).ok();
    }
    
    fn convert_voltage(&self, adc_value: u16) -> f32 {
        (adc_value as f32 * self.calibration.voltage_scale) + self.calibration.voltage_offset
    }
    
    fn convert_current(&self, adc_value: u16) -> f32 {
        (adc_value as f32 * self.calibration.current_scale) + self.calibration.current_offset
    }
    
    fn get_current_power(&self) -> f32 {
        if let Some(latest) = self.power_history.last() {
            latest.power
        } else {
            0.0
        }
    }
    
    fn get_average_power(&self, duration_ms: u32) -> f32 {
        let current_time = self.get_timestamp();
        let start_time = current_time.saturating_sub(duration_ms);
        
        let samples: heapless::Vec<f32, 100> = self.power_history
            .iter()
            .filter(|sample| sample.timestamp >= start_time)
            .map(|sample| sample.power)
            .collect();
        
        if samples.is_empty() {
            0.0
        } else {
            samples.iter().sum::<f32>() / samples.len() as f32
        }
    }
    
    fn get_peak_power(&self, duration_ms: u32) -> f32 {
        let current_time = self.get_timestamp();
        let start_time = current_time.saturating_sub(duration_ms);
        
        self.power_history
            .iter()
            .filter(|sample| sample.timestamp >= start_time)
            .map(|sample| sample.power)
            .fold(0.0, f32::max)
    }
    
    fn get_energy_consumption(&self, duration_ms: u32) -> f32 {
        let current_time = self.get_timestamp();
        let start_time = current_time.saturating_sub(duration_ms);
        
        let mut energy = 0.0;
        let mut last_sample: Option<&PowerSample> = None;
        
        for sample in self.power_history.iter() {
            if sample.timestamp >= start_time {
                if let Some(prev) = last_sample {
                    let dt = (sample.timestamp - prev.timestamp) as f32 / 1000.0; // 转换为秒
                    let avg_power = (sample.power + prev.power) / 2.0;
                    energy += avg_power * dt;
                }
                last_sample = Some(sample);
            }
        }
        
        energy
    }
    
    fn calibrate(&mut self, known_voltage: f32, known_current: f32) {
        let measured_voltage = self.convert_voltage(self.voltage_adc);
        let measured_current = self.convert_current(self.current_adc);
        
        // 简单的线性校准
        self.calibration.voltage_offset += known_voltage - measured_voltage;
        self.calibration.current_offset += known_current - measured_current;
        
        cortex_m_log::println!("Calibration updated:");
        cortex_m_log::println!("  Voltage offset: {}", self.calibration.voltage_offset);
        cortex_m_log::println!("  Current offset: {}", self.calibration.current_offset);
    }
    
    fn get_timestamp(&self) -> u32 {
        cortex_m::peripheral::DWT::cycle_count() / 168_000 // 转换为毫秒
    }
    
    fn generate_report(&self) -> PowerReport {
        let current_power = self.get_current_power();
        let avg_1s = self.get_average_power(1000);
        let avg_10s = self.get_average_power(10000);
        let peak_10s = self.get_peak_power(10000);
        let energy_1min = self.get_energy_consumption(60000);
        
        PowerReport {
            current_power,
            average_1s: avg_1s,
            average_10s: avg_10s,
            peak_10s,
            energy_1min,
        }
    }
}

#[derive(Debug)]
struct PowerReport {
    current_power: f32,
    average_1s: f32,
    average_10s: f32,
    peak_10s: f32,
    energy_1min: f32,
}

impl PowerReport {
    fn print(&self) {
        cortex_m_log::println!("Power Report:");
        cortex_m_log::println!("  Current: {:.2} mW", self.current_power * 1000.0);
        cortex_m_log::println!("  Average (1s): {:.2} mW", self.average_1s * 1000.0);
        cortex_m_log::println!("  Average (10s): {:.2} mW", self.average_10s * 1000.0);
        cortex_m_log::println!("  Peak (10s): {:.2} mW", self.peak_10s * 1000.0);
        cortex_m_log::println!("  Energy (1min): {:.2} mJ", self.energy_1min * 1000.0);
    }
}
```

### 4.2 功耗预算管理

```rust
struct PowerBudgetManager {
    total_budget: f32,
    allocated_budget: f32,
    components: heapless::Vec<PowerComponent, 16>,
    violations: u8,
}

#[derive(Debug, Clone)]
struct PowerComponent {
    name: &'static str,
    allocated_power: f32,
    actual_power: f32,
    priority: u8,
    enabled: bool,
}

impl PowerBudgetManager {
    fn new(total_budget: f32) -> Self {
        Self {
            total_budget,
            allocated_budget: 0.0,
            components: heapless::Vec::new(),
            violations: 0,
        }
    }
    
    fn add_component(&mut self, name: &'static str, power: f32, priority: u8) -> Result<(), ()> {
        if self.allocated_budget + power > self.total_budget {
            return Err(());
        }
        
        let component = PowerComponent {
            name,
            allocated_power: power,
            actual_power: 0.0,
            priority,
            enabled: true,
        };
        
        self.components.push(component).map_err(|_| ())?;
        self.allocated_budget += power;
        
        cortex_m_log::println!("Added component '{}': {:.2} mW (priority {})", name, power * 1000.0, priority);
        
        Ok(())
    }
    
    fn update_component_power(&mut self, name: &'static str, actual_power: f32) {
        for component in &mut self.components {
            if component.name == name {
                component.actual_power = actual_power;
                
                // 检查是否超出预算
                if actual_power > component.allocated_power * 1.1 { // 10%容差
                    cortex_m_log::println!("Power budget violation: {} ({:.2} mW > {:.2} mW)", 
                        name, actual_power * 1000.0, component.allocated_power * 1000.0);
                    self.violations += 1;
                }
                break;
            }
        }
    }
    
    fn get_total_actual_power(&self) -> f32 {
        self.components
            .iter()
            .filter(|c| c.enabled)
            .map(|c| c.actual_power)
            .sum()
    }
    
    fn get_remaining_budget(&self) -> f32 {
        self.total_budget - self.get_total_actual_power()
    }
    
    fn optimize_power_allocation(&mut self) -> Result<(), ()> {
        let total_actual = self.get_total_actual_power();
        
        if total_actual > self.total_budget {
            // 超出预算，需要优化
            cortex_m_log::println!("Power budget exceeded: {:.2} mW > {:.2} mW", 
                total_actual * 1000.0, self.total_budget * 1000.0);
            
            // 按优先级排序，禁用低优先级组件
            let mut sorted_indices: heapless::Vec<usize, 16> = (0..self.components.len()).collect();
            sorted_indices.sort_by_key(|&i| self.components[i].priority);
            
            let mut current_power = total_actual;
            
            for &index in sorted_indices.iter() {
                if current_power <= self.total_budget {
                    break;
                }
                
                if self.components[index].enabled {
                    cortex_m_log::println!("Disabling component '{}' to save power", 
                        self.components[index].name);
                    
                    current_power -= self.components[index].actual_power;
                    self.components[index].enabled = false;
                }
            }
            
            if current_power > self.total_budget {
                return Err(()); // 无法满足功耗预算
            }
        }
        
        Ok(())
    }
    
    fn enable_component(&mut self, name: &'static str) -> Result<(), ()> {
        for component in &mut self.components {
            if component.name == name {
                if !component.enabled {
                    let new_total = self.get_total_actual_power() + component.actual_power;
                    
                    if new_total <= self.total_budget {
                        component.enabled = true;
                        cortex_m_log::println!("Enabled component '{}'", name);
                        return Ok(());
                    } else {
                        return Err(()); // 会超出预算
                    }
                }
                return Ok(());
            }
        }
        Err(())
    }
    
    fn disable_component(&mut self, name: &'static str) -> Result<(), ()> {
        for component in &mut self.components {
            if component.name == name {
                if component.enabled {
                    component.enabled = false;
                    cortex_m_log::println!("Disabled component '{}'", name);
                }
                return Ok(());
            }
        }
        Err(())
    }
    
    fn generate_budget_report(&self) -> BudgetReport {
        let total_allocated = self.components.iter().map(|c| c.allocated_power).sum();
        let total_actual = self.get_total_actual_power();
        let efficiency = if total_allocated > 0.0 { total_actual / total_allocated } else { 0.0 };
        
        BudgetReport {
            total_budget: self.total_budget,
            total_allocated,
            total_actual,
            remaining: self.get_remaining_budget(),
            efficiency,
            violations: self.violations,
            enabled_components: self.components.iter().filter(|c| c.enabled).count() as u8,
        }
    }
}

#[derive(Debug)]
struct BudgetReport {
    total_budget: f32,
    total_allocated: f32,
    total_actual: f32,
    remaining: f32,
    efficiency: f32,
    violations: u8,
    enabled_components: u8,
}

impl BudgetReport {
    fn print(&self) {
        cortex_m_log::println!("Power Budget Report:");
        cortex_m_log::println!("  Total Budget: {:.2} mW", self.total_budget * 1000.0);
        cortex_m_log::println!("  Allocated: {:.2} mW", self.total_allocated * 1000.0);
        cortex_m_log::println!("  Actual: {:.2} mW", self.total_actual * 1000.0);
        cortex_m_log::println!("  Remaining: {:.2} mW", self.remaining * 1000.0);
        cortex_m_log::println!("  Efficiency: {:.1}%", self.efficiency * 100.0);
        cortex_m_log::println!("  Violations: {}", self.violations);
        cortex_m_log::println!("  Enabled Components: {}", self.enabled_components);
    }
}
```

## 5. 功耗优化测试

### 5.1 功耗基准测试

```rust
struct PowerBenchmark;

impl PowerBenchmark {
    fn run_all_tests(power_monitor: &mut PowerMonitor) {
        cortex_m_log::println!("Starting power consumption benchmarks...");
        
        Self::test_idle_power(power_monitor);
        Self::test_cpu_load_power(power_monitor);
        Self::test_peripheral_power(power_monitor);
        Self::test_sleep_modes(power_monitor);
    }
    
    fn test_idle_power(power_monitor: &mut PowerMonitor) {
        cortex_m_log::println!("Testing idle power consumption...");
        
        // 禁用所有非必要外设
        // 测量基础功耗
        
        let start_time = power_monitor.get_timestamp();
        
        // 空闲1秒
        while power_monitor.get_timestamp() - start_time < 1000 {
            cortex_m::asm::nop();
        }
        
        let idle_power = power_monitor.get_average_power(1000);
        cortex_m_log::println!("Idle power: {:.2} mW", idle_power * 1000.0);
    }
    
    fn test_cpu_load_power(power_monitor: &mut PowerMonitor) {
        cortex_m_log::println!("Testing CPU load power consumption...");
        
        for load_percent in [25, 50, 75, 100].iter() {
            let start_time = power_monitor.get_timestamp();
            
            // 模拟不同的CPU负载
            while power_monitor.get_timestamp() - start_time < 1000 {
                // 工作时间
                let work_cycles = *load_percent * 1000;
                for _ in 0..work_cycles {
                    cortex_m::asm::nop();
                }
                
                // 空闲时间
                let idle_cycles = (100 - load_percent) * 1000;
                for _ in 0..idle_cycles {
                    cortex_m::asm::wfi();
                }
            }
            
            let load_power = power_monitor.get_average_power(1000);
            cortex_m_log::println!("{}% CPU load power: {:.2} mW", load_percent, load_power * 1000.0);
        }
    }
    
    fn test_peripheral_power(power_monitor: &mut PowerMonitor) {
        cortex_m_log::println!("Testing peripheral power consumption...");
        
        // 测试各个外设的功耗
        let peripherals = ["GPIO", "UART", "SPI", "I2C", "ADC", "Timer"];
        
        for peripheral in peripherals.iter() {
            cortex_m_log::println!("Testing {} power...", peripheral);
            
            // 启用外设
            // enable_peripheral(peripheral);
            
            let start_time = power_monitor.get_timestamp();
            
            // 运行外设1秒
            while power_monitor.get_timestamp() - start_time < 1000 {
                // 模拟外设活动
                cortex_m::asm::nop();
            }
            
            let peripheral_power = power_monitor.get_average_power(1000);
            cortex_m_log::println!("{} power: {:.2} mW", peripheral, peripheral_power * 1000.0);
            
            // 禁用外设
            // disable_peripheral(peripheral);
        }
    }
    
    fn test_sleep_modes(power_monitor: &mut PowerMonitor) {
        cortex_m_log::println!("Testing sleep mode power consumption...");
        
        // 注意：实际测试需要外部唤醒源
        cortex_m_log::println!("Sleep mode testing requires external wakeup source");
        
        // 模拟不同睡眠模式的功耗
        let sleep_modes = [
            ("Sleep", 10.0),
            ("Stop", 2.0),
            ("Standby", 0.1),
        ];
        
        for (mode, estimated_power) in sleep_modes.iter() {
            cortex_m_log::println!("{} mode estimated power: {:.2} mW", mode, estimated_power);
        }
    }
}
```

## 总结

功耗优化是嵌入式系统设计的关键要素，特别是对于电池供电的设备。通过合理的低功耗模式管理、动态时钟调整、外设功耗控制和实时功耗监控，可以显著延长设备的工作时间。

关键要点：
1. 合理使用不同的低功耗模式
2. 实施动态功耗管理策略
3. 优化时钟配置和外设使用
4. 实时监控和预算管理功耗
5. 进行全面的功耗基准测试
6. 根据应用需求平衡性能和功耗