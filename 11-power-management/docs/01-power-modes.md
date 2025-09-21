# 电源管理模式

在嵌入式系统中，电源管理是延长电池寿命和降低功耗的关键技术。STM32F4系列微控制器提供了多种电源管理模式，可以根据应用需求灵活选择。

## 电源管理概述

### 功耗优化的重要性
- **电池寿命**: 延长电池供电设备的工作时间
- **热管理**: 降低系统发热，提高可靠性
- **成本控制**: 减少电源系统的复杂度和成本
- **环保要求**: 符合绿色电子产品标准

### STM32F4电源架构
```
VDD (1.8V-3.6V) ──┬── 内核电源域 (1.2V)
                  ├── I/O电源域 (VDD)
                  ├── ADC电源域 (VDDA)
                  ├── 备份域 (VBAT)
                  └── 时钟域
```

## STM32F4电源模式

### 1. 运行模式 (Run Mode)
正常工作模式，CPU和所有外设都可以工作。

```rust
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    rcc::{Clocks, RccExt},
};

// 高性能运行模式配置
pub fn configure_high_performance_mode(rcc: stm32::RCC) -> Clocks {
    let rcc = rcc.constrain();
    
    // 配置为最高频率 168MHz
    rcc.cfgr
        .use_hse(8.mhz())           // 外部晶振8MHz
        .sysclk(168.mhz())          // 系统时钟168MHz
        .hclk(168.mhz())            // AHB时钟168MHz
        .pclk1(42.mhz())            // APB1时钟42MHz
        .pclk2(84.mhz())            // APB2时钟84MHz
        .freeze()
}

// 低功耗运行模式配置
pub fn configure_low_power_run_mode(rcc: stm32::RCC) -> Clocks {
    let rcc = rcc.constrain();
    
    // 配置为较低频率以降低功耗
    rcc.cfgr
        .use_hsi()                  // 使用内部RC振荡器
        .sysclk(16.mhz())           // 系统时钟16MHz
        .hclk(16.mhz())             // AHB时钟16MHz
        .pclk1(16.mhz())            // APB1时钟16MHz
        .pclk2(16.mhz())            // APB2时钟16MHz
        .freeze()
}
```

### 2. 睡眠模式 (Sleep Mode)
CPU停止工作，但外设继续运行，可以通过中断唤醒。

```rust
use cortex_m;
use stm32f4xx_hal::{
    stm32,
    gpio::{gpioa::PA0, Input, PullUp},
    interrupt,
};

pub struct SleepManager {
    wakeup_pin: PA0<Input<PullUp>>,
}

impl SleepManager {
    pub fn new(wakeup_pin: PA0<Input<PullUp>>) -> Self {
        Self { wakeup_pin }
    }
    
    // 进入睡眠模式
    pub fn enter_sleep_mode(&self) {
        // 配置唤醒源
        self.configure_wakeup_sources();
        
        // 清除待处理的中断
        cortex_m::peripheral::SCB::clear_sleepdeep();
        
        // 进入睡眠模式
        cortex_m::asm::wfi(); // Wait For Interrupt
        
        // 从睡眠模式唤醒后继续执行
        self.handle_wakeup();
    }
    
    fn configure_wakeup_sources(&self) {
        // 配置外部中断作为唤醒源
        // 这里简化处理，实际应用中需要配置EXTI
    }
    
    fn handle_wakeup(&self) {
        // 唤醒后的处理逻辑
        // 可以检查唤醒原因并执行相应操作
    }
}

// 定时器唤醒示例
pub struct TimerWakeup {
    timer: stm32::TIM2,
}

impl TimerWakeup {
    pub fn new(timer: stm32::TIM2) -> Self {
        Self { timer }
    }
    
    // 配置定时器唤醒
    pub fn configure_timer_wakeup(&mut self, seconds: u32) {
        // 启用定时器时钟
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
        }
        
        // 配置定时器
        self.timer.psc.write(|w| unsafe { w.psc().bits(16000 - 1) }); // 预分频器
        self.timer.arr.write(|w| unsafe { w.arr().bits(1000 * seconds - 1) }); // 自动重载值
        
        // 启用定时器中断
        self.timer.dier.write(|w| w.uie().set_bit());
        
        // 启动定时器
        self.timer.cr1.write(|w| w.cen().set_bit());
    }
}
```

### 3. 停止模式 (Stop Mode)
CPU和大部分外设停止工作，只保留SRAM和寄存器内容。

```rust
use stm32f4xx_hal::{
    stm32,
    rcc::Clocks,
};

pub struct StopModeManager {
    pwr: stm32::PWR,
    scb: cortex_m::peripheral::SCB,
}

impl StopModeManager {
    pub fn new(pwr: stm32::PWR, scb: cortex_m::peripheral::SCB) -> Self {
        // 启用PWR时钟
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
        }
        
        Self { pwr, scb }
    }
    
    // 进入停止模式
    pub fn enter_stop_mode(&mut self) {
        // 配置停止模式参数
        self.configure_stop_mode();
        
        // 配置唤醒源
        self.configure_wakeup_sources();
        
        // 清除唤醒标志
        self.pwr.cr.modify(|_, w| w.cwuf().set_bit());
        
        // 设置SLEEPDEEP位
        self.scb.set_sleepdeep();
        
        // 进入停止模式
        cortex_m::asm::wfi();
        
        // 唤醒后恢复系统时钟
        self.restore_system_clock();
    }
    
    fn configure_stop_mode(&mut self) {
        // 配置电压调节器为低功耗模式
        self.pwr.cr.modify(|_, w| w.lpds().set_bit());
        
        // 配置Flash在停止模式下的行为
        self.pwr.cr.modify(|_, w| w.fpds().set_bit());
    }
    
    fn configure_wakeup_sources(&self) {
        // 配置RTC作为唤醒源
        self.configure_rtc_wakeup();
        
        // 配置外部中断作为唤醒源
        self.configure_exti_wakeup();
    }
    
    fn configure_rtc_wakeup(&self) {
        // RTC唤醒配置
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
            
            // 启用备份域访问
            self.pwr.cr.modify(|_, w| w.dbp().set_bit());
            
            // 配置RTC时钟源
            rcc.bdcr.modify(|_, w| w.rtcsel().lsi().rtcen().set_bit());
        }
    }
    
    fn configure_exti_wakeup(&self) {
        // 配置外部中断线作为唤醒源
        unsafe {
            let exti = &(*stm32::EXTI::ptr());
            
            // 启用EXTI线0中断
            exti.imr.modify(|_, w| w.mr0().set_bit());
            
            // 配置为下降沿触发
            exti.ftsr.modify(|_, w| w.tr0().set_bit());
        }
    }
    
    fn restore_system_clock(&self) {
        // 停止模式唤醒后，系统时钟会切换到HSI
        // 需要重新配置为原来的时钟源
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            
            // 启用HSE
            rcc.cr.modify(|_, w| w.hseon().set_bit());
            while rcc.cr.read().hserdy().bit_is_clear() {}
            
            // 重新配置PLL
            rcc.pllcfgr.modify(|_, w| {
                w.pllsrc().hse()
                 .pllm().bits(8)
                 .plln().bits(336)
                 .pllp().div4()
                 .pllq().bits(7)
            });
            
            // 启用PLL
            rcc.cr.modify(|_, w| w.pllon().set_bit());
            while rcc.cr.read().pllrdy().bit_is_clear() {}
            
            // 切换系统时钟到PLL
            rcc.cfgr.modify(|_, w| w.sw().pll());
            while !rcc.cfgr.read().sws().is_pll() {}
        }
    }
}
```

### 4. 待机模式 (Standby Mode)
最低功耗模式，除了备份域外所有内容都会丢失。

```rust
pub struct StandbyModeManager {
    pwr: stm32::PWR,
    rtc: stm32::RTC,
}

impl StandbyModeManager {
    pub fn new(pwr: stm32::PWR, rtc: stm32::RTC) -> Self {
        // 启用PWR和RTC时钟
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb1enr.modify(|_, w| w.pwren().set_bit().rtcen().set_bit());
        }
        
        Self { pwr, rtc }
    }
    
    // 进入待机模式
    pub fn enter_standby_mode(&mut self, wakeup_seconds: u32) {
        // 配置RTC唤醒
        self.configure_rtc_alarm(wakeup_seconds);
        
        // 清除唤醒标志
        self.pwr.cr.modify(|_, w| w.cwuf().set_bit());
        
        // 启用唤醒引脚
        self.pwr.csr.modify(|_, w| w.ewup().set_bit());
        
        // 设置待机模式
        self.pwr.cr.modify(|_, w| w.pdds().set_bit());
        
        // 设置SLEEPDEEP位
        unsafe {
            let scb = &(*cortex_m::peripheral::SCB::ptr());
            scb.scr.modify(|v| v | (1 << 2));
        }
        
        // 进入待机模式
        cortex_m::asm::wfi();
        
        // 注意：从待机模式唤醒后会重新启动系统
        // 不会执行到这里
    }
    
    fn configure_rtc_alarm(&mut self, seconds: u32) {
        // 启用备份域访问
        self.pwr.cr.modify(|_, w| w.dbp().set_bit());
        
        // 配置RTC闹钟
        unsafe {
            // 禁用闹钟
            self.rtc.cr.modify(|_, w| w.alrae().clear_bit());
            
            // 等待闹钟配置就绪
            while self.rtc.isr.read().alrawf().bit_is_clear() {}
            
            // 设置闹钟时间
            let alarm_time = self.calculate_alarm_time(seconds);
            self.rtc.alrmar.write(|w| w.bits(alarm_time));
            
            // 启用闹钟中断
            self.rtc.cr.modify(|_, w| w.alraie().set_bit());
            
            // 启用闹钟
            self.rtc.cr.modify(|_, w| w.alrae().set_bit());
        }
    }
    
    fn calculate_alarm_time(&self, seconds: u32) -> u32 {
        // 简化的时间计算
        // 实际应用中需要考虑当前时间和日期
        let hours = (seconds / 3600) % 24;
        let minutes = (seconds % 3600) / 60;
        let secs = seconds % 60;
        
        // RTC闹钟寄存器格式 (BCD编码)
        let bcd_hours = ((hours / 10) << 4) | (hours % 10);
        let bcd_minutes = ((minutes / 10) << 4) | (minutes % 10);
        let bcd_seconds = ((secs / 10) << 4) | (secs % 10);
        
        (bcd_hours << 16) | (bcd_minutes << 8) | bcd_seconds
    }
    
    // 检查是否从待机模式唤醒
    pub fn check_standby_wakeup(&self) -> bool {
        self.pwr.csr.read().wuf().bit_is_set()
    }
}
```

## 动态电源管理

### 动态频率调节 (DVFS)
根据系统负载动态调整CPU频率。

```rust
pub struct DynamicFrequencyScaling {
    rcc: stm32::RCC,
    current_frequency: u32,
    load_monitor: SystemLoadMonitor,
}

impl DynamicFrequencyScaling {
    pub fn new(rcc: stm32::RCC) -> Self {
        Self {
            rcc,
            current_frequency: 168_000_000, // 168MHz
            load_monitor: SystemLoadMonitor::new(),
        }
    }
    
    // 动态调整频率
    pub fn adjust_frequency(&mut self) {
        let cpu_load = self.load_monitor.get_cpu_load();
        
        let target_frequency = match cpu_load {
            0..=20 => 16_000_000,   // 低负载：16MHz
            21..=50 => 84_000_000,  // 中负载：84MHz
            51..=80 => 120_000_000, // 高负载：120MHz
            _ => 168_000_000,       // 满负载：168MHz
        };
        
        if target_frequency != self.current_frequency {
            self.set_system_frequency(target_frequency);
            self.current_frequency = target_frequency;
        }
    }
    
    fn set_system_frequency(&mut self, frequency: u32) {
        unsafe {
            match frequency {
                16_000_000 => {
                    // 切换到HSI (16MHz)
                    self.rcc.cfgr.modify(|_, w| w.sw().hsi());
                    while !self.rcc.cfgr.read().sws().is_hsi() {}
                    
                    // 关闭PLL节省功耗
                    self.rcc.cr.modify(|_, w| w.pllon().clear_bit());
                }
                84_000_000 => {
                    // 配置PLL为84MHz
                    self.configure_pll(84_000_000);
                }
                120_000_000 => {
                    // 配置PLL为120MHz
                    self.configure_pll(120_000_000);
                }
                168_000_000 => {
                    // 配置PLL为168MHz
                    self.configure_pll(168_000_000);
                }
                _ => {}
            }
        }
    }
    
    fn configure_pll(&mut self, frequency: u32) {
        unsafe {
            // 关闭PLL
            self.rcc.cr.modify(|_, w| w.pllon().clear_bit());
            while self.rcc.cr.read().pllrdy().bit_is_set() {}
            
            // 配置PLL参数
            let (plln, pllp) = match frequency {
                84_000_000 => (336, 4),   // 8MHz * 336 / 4 / 8 = 84MHz
                120_000_000 => (240, 2),  // 8MHz * 240 / 2 / 8 = 120MHz
                168_000_000 => (336, 2),  // 8MHz * 336 / 2 / 8 = 168MHz
                _ => (336, 2),
            };
            
            self.rcc.pllcfgr.modify(|_, w| {
                w.pllsrc().hse()
                 .pllm().bits(8)
                 .plln().bits(plln)
                 .pllp().bits((pllp / 2 - 1) as u8)
                 .pllq().bits(7)
            });
            
            // 启用PLL
            self.rcc.cr.modify(|_, w| w.pllon().set_bit());
            while self.rcc.cr.read().pllrdy().bit_is_clear() {}
            
            // 切换系统时钟到PLL
            self.rcc.cfgr.modify(|_, w| w.sw().pll());
            while !self.rcc.cfgr.read().sws().is_pll() {}
        }
    }
}

// 系统负载监控
pub struct SystemLoadMonitor {
    idle_counter: u32,
    total_counter: u32,
    last_measurement: u32,
}

impl SystemLoadMonitor {
    pub fn new() -> Self {
        Self {
            idle_counter: 0,
            total_counter: 0,
            last_measurement: 0,
        }
    }
    
    // 在空闲循环中调用
    pub fn idle_tick(&mut self) {
        self.idle_counter += 1;
        self.total_counter += 1;
    }
    
    // 在任务执行时调用
    pub fn active_tick(&mut self) {
        self.total_counter += 1;
    }
    
    // 获取CPU负载百分比
    pub fn get_cpu_load(&mut self) -> u8 {
        if self.total_counter == 0 {
            return 0;
        }
        
        let idle_percentage = (self.idle_counter * 100) / self.total_counter;
        let cpu_load = 100 - idle_percentage;
        
        // 重置计数器
        self.idle_counter = 0;
        self.total_counter = 0;
        
        cpu_load as u8
    }
}
```

### 外设电源管理
动态控制外设的电源状态。

```rust
pub struct PeripheralPowerManager {
    rcc: stm32::RCC,
    active_peripherals: u32,
}

impl PeripheralPowerManager {
    pub fn new(rcc: stm32::RCC) -> Self {
        Self {
            rcc,
            active_peripherals: 0,
        }
    }
    
    // 启用外设时钟
    pub fn enable_peripheral(&mut self, peripheral: Peripheral) {
        if !self.is_peripheral_enabled(peripheral) {
            unsafe {
                match peripheral {
                    Peripheral::GPIOA => {
                        self.rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
                    }
                    Peripheral::GPIOB => {
                        self.rcc.ahb1enr.modify(|_, w| w.gpioben().set_bit());
                    }
                    Peripheral::USART1 => {
                        self.rcc.apb2enr.modify(|_, w| w.usart1en().set_bit());
                    }
                    Peripheral::USART2 => {
                        self.rcc.apb1enr.modify(|_, w| w.usart2en().set_bit());
                    }
                    Peripheral::I2C1 => {
                        self.rcc.apb1enr.modify(|_, w| w.i2c1en().set_bit());
                    }
                    Peripheral::SPI1 => {
                        self.rcc.apb2enr.modify(|_, w| w.spi1en().set_bit());
                    }
                    Peripheral::TIM2 => {
                        self.rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
                    }
                    Peripheral::ADC1 => {
                        self.rcc.apb2enr.modify(|_, w| w.adc1en().set_bit());
                    }
                }
            }
            
            self.active_peripherals |= peripheral as u32;
        }
    }
    
    // 禁用外设时钟
    pub fn disable_peripheral(&mut self, peripheral: Peripheral) {
        if self.is_peripheral_enabled(peripheral) {
            unsafe {
                match peripheral {
                    Peripheral::GPIOA => {
                        self.rcc.ahb1enr.modify(|_, w| w.gpioaen().clear_bit());
                    }
                    Peripheral::GPIOB => {
                        self.rcc.ahb1enr.modify(|_, w| w.gpioben().clear_bit());
                    }
                    Peripheral::USART1 => {
                        self.rcc.apb2enr.modify(|_, w| w.usart1en().clear_bit());
                    }
                    Peripheral::USART2 => {
                        self.rcc.apb1enr.modify(|_, w| w.usart2en().clear_bit());
                    }
                    Peripheral::I2C1 => {
                        self.rcc.apb1enr.modify(|_, w| w.i2c1en().clear_bit());
                    }
                    Peripheral::SPI1 => {
                        self.rcc.apb2enr.modify(|_, w| w.spi1en().clear_bit());
                    }
                    Peripheral::TIM2 => {
                        self.rcc.apb1enr.modify(|_, w| w.tim2en().clear_bit());
                    }
                    Peripheral::ADC1 => {
                        self.rcc.apb2enr.modify(|_, w| w.adc1en().clear_bit());
                    }
                }
            }
            
            self.active_peripherals &= !(peripheral as u32);
        }
    }
    
    fn is_peripheral_enabled(&self, peripheral: Peripheral) -> bool {
        (self.active_peripherals & (peripheral as u32)) != 0
    }
    
    // 获取当前活跃的外设数量
    pub fn get_active_peripheral_count(&self) -> u8 {
        self.active_peripherals.count_ones() as u8
    }
}

#[derive(Clone, Copy)]
pub enum Peripheral {
    GPIOA = 1 << 0,
    GPIOB = 1 << 1,
    USART1 = 1 << 2,
    USART2 = 1 << 3,
    I2C1 = 1 << 4,
    SPI1 = 1 << 5,
    TIM2 = 1 << 6,
    ADC1 = 1 << 7,
}
```

## 电源监控和管理

### 电压监控
监控系统供电电压，在电压异常时采取保护措施。

```rust
use stm32f4xx_hal::{
    adc::{Adc, config::AdcConfig},
    gpio::{gpioa::PA0, Analog},
};

pub struct VoltageMonitor {
    adc: Adc<stm32::ADC1>,
    vref_pin: PA0<Analog>,
    voltage_threshold: u16,
}

impl VoltageMonitor {
    pub fn new(adc: Adc<stm32::ADC1>, vref_pin: PA0<Analog>) -> Self {
        Self {
            adc,
            vref_pin,
            voltage_threshold: 2800, // 2.8V阈值
        }
    }
    
    // 读取供电电压
    pub fn read_supply_voltage(&mut self) -> u16 {
        // 读取内部参考电压
        let vrefint_sample: u16 = self.adc.read(&mut self.vref_pin).unwrap();
        
        // 计算实际电压 (mV)
        // VREFINT典型值为1.21V，ADC满量程对应VDD
        let vdd_mv = (1210 * 4095) / vrefint_sample;
        
        vdd_mv
    }
    
    // 检查电压是否正常
    pub fn is_voltage_normal(&mut self) -> bool {
        let voltage = self.read_supply_voltage();
        voltage >= self.voltage_threshold
    }
    
    // 低电压保护
    pub fn low_voltage_protection(&mut self) -> PowerAction {
        let voltage = self.read_supply_voltage();
        
        match voltage {
            0..=2500 => PowerAction::EmergencyShutdown,
            2501..=2800 => PowerAction::EnterLowPowerMode,
            2801..=3000 => PowerAction::ReducePerformance,
            _ => PowerAction::Normal,
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum PowerAction {
    Normal,
    ReducePerformance,
    EnterLowPowerMode,
    EmergencyShutdown,
}
```

### 电池管理
针对电池供电系统的专门管理。

```rust
pub struct BatteryManager {
    voltage_monitor: VoltageMonitor,
    charge_level: u8,
    discharge_curve: [u16; 11], // 0-100%对应的电压值
}

impl BatteryManager {
    pub fn new(voltage_monitor: VoltageMonitor) -> Self {
        Self {
            voltage_monitor,
            charge_level: 100,
            // 锂电池放电曲线 (3.0V-4.2V)
            discharge_curve: [
                3000, 3200, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200
            ],
        }
    }
    
    // 更新电池电量
    pub fn update_battery_level(&mut self) {
        let voltage = self.voltage_monitor.read_supply_voltage();
        self.charge_level = self.voltage_to_percentage(voltage);
    }
    
    fn voltage_to_percentage(&self, voltage: u16) -> u8 {
        // 根据放电曲线计算电量百分比
        for (i, &curve_voltage) in self.discharge_curve.iter().enumerate() {
            if voltage <= curve_voltage {
                return (i * 10) as u8;
            }
        }
        100
    }
    
    // 获取电池状态
    pub fn get_battery_status(&self) -> BatteryStatus {
        match self.charge_level {
            0..=5 => BatteryStatus::Critical,
            6..=15 => BatteryStatus::Low,
            16..=30 => BatteryStatus::Medium,
            _ => BatteryStatus::Good,
        }
    }
    
    // 电池保护策略
    pub fn get_power_strategy(&self) -> PowerStrategy {
        match self.get_battery_status() {
            BatteryStatus::Critical => PowerStrategy::EmergencyMode,
            BatteryStatus::Low => PowerStrategy::PowerSaving,
            BatteryStatus::Medium => PowerStrategy::Balanced,
            BatteryStatus::Good => PowerStrategy::Performance,
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum BatteryStatus {
    Good,
    Medium,
    Low,
    Critical,
}

#[derive(Debug, PartialEq)]
pub enum PowerStrategy {
    Performance,    // 高性能模式
    Balanced,       // 平衡模式
    PowerSaving,    // 省电模式
    EmergencyMode,  // 紧急模式
}
```

## 实际应用示例

### 智能传感器节点
结合多种电源管理技术的实际应用。

```rust
pub struct SmartSensorNode {
    power_manager: PeripheralPowerManager,
    battery_manager: BatteryManager,
    sleep_manager: SleepManager,
    frequency_scaler: DynamicFrequencyScaling,
    sensor_active: bool,
    measurement_interval: u32,
}

impl SmartSensorNode {
    pub fn new(
        power_manager: PeripheralPowerManager,
        battery_manager: BatteryManager,
        sleep_manager: SleepManager,
        frequency_scaler: DynamicFrequencyScaling,
    ) -> Self {
        Self {
            power_manager,
            battery_manager,
            sleep_manager,
            frequency_scaler,
            sensor_active: false,
            measurement_interval: 60, // 默认60秒间隔
        }
    }
    
    // 主运行循环
    pub fn run(&mut self) {
        loop {
            // 更新电池状态
            self.battery_manager.update_battery_level();
            
            // 根据电池状态调整策略
            self.adjust_power_strategy();
            
            // 执行传感器测量
            if self.should_measure() {
                self.perform_measurement();
            }
            
            // 进入低功耗模式
            self.enter_low_power_mode();
        }
    }
    
    fn adjust_power_strategy(&mut self) {
        let strategy = self.battery_manager.get_power_strategy();
        
        match strategy {
            PowerStrategy::Performance => {
                self.measurement_interval = 30;  // 30秒间隔
                self.frequency_scaler.adjust_frequency();
            }
            PowerStrategy::Balanced => {
                self.measurement_interval = 60;  // 60秒间隔
                self.frequency_scaler.adjust_frequency();
            }
            PowerStrategy::PowerSaving => {
                self.measurement_interval = 300; // 5分钟间隔
                // 降低CPU频率
                // 关闭非必要外设
                self.power_manager.disable_peripheral(Peripheral::USART1);
            }
            PowerStrategy::EmergencyMode => {
                self.measurement_interval = 3600; // 1小时间隔
                // 最低功耗配置
                self.disable_non_essential_peripherals();
            }
        }
    }
    
    fn should_measure(&self) -> bool {
        // 简化的测量时机判断
        // 实际应用中会基于定时器或RTC
        true
    }
    
    fn perform_measurement(&mut self) {
        // 启用传感器外设
        self.power_manager.enable_peripheral(Peripheral::I2C1);
        self.power_manager.enable_peripheral(Peripheral::ADC1);
        
        // 执行测量
        self.sensor_active = true;
        
        // 模拟测量过程
        cortex_m::asm::delay(1000);
        
        // 测量完成后关闭外设
        self.power_manager.disable_peripheral(Peripheral::I2C1);
        self.power_manager.disable_peripheral(Peripheral::ADC1);
        
        self.sensor_active = false;
    }
    
    fn enter_low_power_mode(&mut self) {
        if !self.sensor_active {
            // 根据电池状态选择低功耗模式
            match self.battery_manager.get_battery_status() {
                BatteryStatus::Good | BatteryStatus::Medium => {
                    // 睡眠模式
                    self.sleep_manager.enter_sleep_mode();
                }
                BatteryStatus::Low => {
                    // 停止模式
                    // self.stop_manager.enter_stop_mode();
                }
                BatteryStatus::Critical => {
                    // 待机模式
                    // self.standby_manager.enter_standby_mode(3600);
                }
            }
        }
    }
    
    fn disable_non_essential_peripherals(&mut self) {
        self.power_manager.disable_peripheral(Peripheral::USART1);
        self.power_manager.disable_peripheral(Peripheral::SPI1);
        self.power_manager.disable_peripheral(Peripheral::TIM2);
    }
}
```

## 总结

电源管理是嵌入式系统设计的重要组成部分，需要综合考虑：

1. **硬件设计**: 选择合适的电源方案和低功耗器件
2. **软件优化**: 实现智能的电源管理策略
3. **系统集成**: 协调各个子系统的电源需求
4. **用户体验**: 在功耗和性能之间找到平衡

通过合理的电源管理，可以显著延长电池寿命，提高系统可靠性，降低运行成本。