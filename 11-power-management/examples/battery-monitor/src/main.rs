#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    gpio::{gpioa::*, gpiob::*, gpioc::*, Output, PushPull, Analog},
    adc::{Adc, config::AdcConfig},
    timer::{Timer, Event},
    i2c::I2c,
    serial::{Serial, config::Config as SerialConfig},
};

use heapless::{Vec, String, pool::{Pool, Node}};
use micromath::F32Ext;

/// 电池监控系统
pub struct BatteryMonitorSystem {
    // 硬件接口
    voltage_adc: Adc<stm32::ADC1>,
    current_adc: Adc<stm32::ADC2>,
    voltage_pin: PA1<Analog>,
    current_pin: PA2<Analog>,
    
    // 状态指示
    status_led: PC13<Output<PushPull>>,
    warning_led: PB0<Output<PushPull>>,
    error_led: PB1<Output<PushPull>>,
    
    // 通信接口
    serial: Serial<stm32::USART1, (PA9<stm32f4xx_hal::gpio::Alternate<7>>, PA10<stm32f4xx_hal::gpio::Alternate<7>>)>,
    
    // 电池配置
    battery_config: BatteryConfig,
    
    // 监控状态
    monitor_state: MonitorState,
    
    // 数据缓冲
    voltage_buffer: Vec<u16, 100>,
    current_buffer: Vec<i16, 100>,
    
    // 统计数据
    statistics: BatteryStatistics,
    
    // 校准参数
    calibration: CalibrationData,
}

/// 电池配置
#[derive(Clone, Copy)]
pub struct BatteryConfig {
    pub battery_type: BatteryType,
    pub nominal_voltage: u16,    // mV
    pub capacity: u16,           // mAh
    pub max_voltage: u16,        // mV
    pub min_voltage: u16,        // mV
    pub max_current: u16,        // mA
    pub internal_resistance: u16, // mΩ
}

/// 电池类型
#[derive(Clone, Copy, PartialEq)]
pub enum BatteryType {
    LiIon,
    LiPo,
    NiMH,
    LeadAcid,
}

/// 监控状态
#[derive(Clone, Copy)]
pub struct MonitorState {
    pub voltage: u16,           // 当前电压 (mV)
    pub current: i16,           // 当前电流 (mA, 正值为放电)
    pub power: i16,             // 当前功率 (mW)
    pub state_of_charge: u8,    // 电量百分比
    pub remaining_capacity: u16, // 剩余容量 (mAh)
    pub remaining_time: u32,    // 剩余时间 (分钟)
    pub temperature: i16,       // 温度 (°C * 10)
    pub cycle_count: u16,       // 循环次数
    pub health: BatteryHealth,  // 电池健康状态
}

/// 电池健康状态
#[derive(Clone, Copy, PartialEq)]
pub enum BatteryHealth {
    Excellent,  // 优秀 (>90%)
    Good,       // 良好 (70-90%)
    Fair,       // 一般 (50-70%)
    Poor,       // 差 (30-50%)
    Critical,   // 临界 (<30%)
}

/// 电池统计数据
#[derive(Clone, Copy)]
pub struct BatteryStatistics {
    pub total_charge_in: u32,   // 总充入电量 (mAh)
    pub total_charge_out: u32,  // 总放出电量 (mAh)
    pub max_voltage_seen: u16,  // 历史最高电压
    pub min_voltage_seen: u16,  // 历史最低电压
    pub max_current_seen: u16,  // 历史最大电流
    pub average_voltage: u16,   // 平均电压
    pub average_current: u16,   // 平均电流
    pub uptime: u32,           // 运行时间 (秒)
}

/// 校准数据
#[derive(Clone, Copy)]
pub struct CalibrationData {
    pub voltage_offset: i16,    // 电压偏移 (mV)
    pub voltage_scale: f32,     // 电压比例
    pub current_offset: i16,    // 电流偏移 (mA)
    pub current_scale: f32,     // 电流比例
    pub shunt_resistance: f32,  // 分流电阻 (mΩ)
}

impl BatteryMonitorSystem {
    pub fn new(
        voltage_adc: Adc<stm32::ADC1>,
        current_adc: Adc<stm32::ADC2>,
        voltage_pin: PA1<Analog>,
        current_pin: PA2<Analog>,
        status_led: PC13<Output<PushPull>>,
        warning_led: PB0<Output<PushPull>>,
        error_led: PB1<Output<PushPull>>,
        serial: Serial<stm32::USART1, (PA9<stm32f4xx_hal::gpio::Alternate<7>>, PA10<stm32f4xx_hal::gpio::Alternate<7>>)>,
    ) -> Self {
        Self {
            voltage_adc,
            current_adc,
            voltage_pin,
            current_pin,
            status_led,
            warning_led,
            error_led,
            serial,
            battery_config: BatteryConfig {
                battery_type: BatteryType::LiIon,
                nominal_voltage: 3700,
                capacity: 2500,
                max_voltage: 4200,
                min_voltage: 3000,
                max_current: 2500,
                internal_resistance: 100,
            },
            monitor_state: MonitorState {
                voltage: 0,
                current: 0,
                power: 0,
                state_of_charge: 0,
                remaining_capacity: 0,
                remaining_time: 0,
                temperature: 250,
                cycle_count: 0,
                health: BatteryHealth::Good,
            },
            voltage_buffer: Vec::new(),
            current_buffer: Vec::new(),
            statistics: BatteryStatistics {
                total_charge_in: 0,
                total_charge_out: 0,
                max_voltage_seen: 0,
                min_voltage_seen: 5000,
                max_current_seen: 0,
                average_voltage: 0,
                average_current: 0,
                uptime: 0,
            },
            calibration: CalibrationData {
                voltage_offset: 0,
                voltage_scale: 1.0,
                current_offset: 0,
                current_scale: 1.0,
                shunt_resistance: 10.0, // 10mΩ分流电阻
            },
        }
    }
    
    /// 初始化监控系统
    pub fn initialize(&mut self) -> Result<(), &'static str> {
        // 初始化LED
        self.status_led.set_high();
        self.warning_led.set_low();
        self.error_led.set_low();
        
        // 校准ADC
        self.calibrate_adc()?;
        
        // 初始化电池状态
        self.initialize_battery_state()?;
        
        // 发送初始化消息
        self.send_message("Battery Monitor Initialized\r\n");
        
        Ok(())
    }
    
    /// 主监控循环
    pub fn run_monitoring(&mut self) -> ! {
        let mut last_update = 0u32;
        let mut sample_count = 0u32;
        
        loop {
            // 读取传感器数据
            let voltage = self.read_voltage();
            let current = self.read_current();
            
            // 更新缓冲区
            self.update_buffers(voltage, current);
            
            // 计算派生值
            self.calculate_derived_values();
            
            // 更新统计数据
            self.update_statistics();
            
            // 检查报警条件
            self.check_alarms();
            
            // 更新LED状态
            self.update_led_status();
            
            // 定期发送数据
            sample_count += 1;
            if sample_count % 100 == 0 {
                self.send_status_report();
            }
            
            // 延时
            cortex_m::asm::delay(10000); // 10ms
        }
    }
    
    /// 读取电压
    fn read_voltage(&mut self) -> u16 {
        let raw_sample: u16 = self.voltage_adc.read(&mut self.voltage_pin).unwrap_or(0);
        
        // 转换为毫伏
        let voltage_mv = ((raw_sample as f32 * 3300.0) / 4095.0) as u16;
        
        // 应用校准
        let calibrated_voltage = ((voltage_mv as i32 + self.calibration.voltage_offset as i32) as f32 
            * self.calibration.voltage_scale) as u16;
        
        calibrated_voltage
    }
    
    /// 读取电流
    fn read_current(&mut self) -> i16 {
        let raw_sample: u16 = self.current_adc.read(&mut self.current_pin).unwrap_or(2048);
        
        // 转换为电压 (mV)
        let voltage_mv = ((raw_sample as f32 * 3300.0) / 4095.0) - 1650.0; // 1.65V为零点
        
        // 转换为电流 (mA)
        let current_ma = (voltage_mv / self.calibration.shunt_resistance) as i16;
        
        // 应用校准
        let calibrated_current = ((current_ma as i32 + self.calibration.current_offset as i32) as f32 
            * self.calibration.current_scale) as i16;
        
        calibrated_current
    }
    
    /// 更新数据缓冲区
    fn update_buffers(&mut self, voltage: u16, current: i16) {
        // 电压缓冲区
        if self.voltage_buffer.len() >= 100 {
            self.voltage_buffer.remove(0);
        }
        self.voltage_buffer.push(voltage).ok();
        
        // 电流缓冲区
        if self.current_buffer.len() >= 100 {
            self.current_buffer.remove(0);
        }
        self.current_buffer.push(current).ok();
        
        // 更新当前状态
        self.monitor_state.voltage = voltage;
        self.monitor_state.current = current;
    }
    
    /// 计算派生值
    fn calculate_derived_values(&mut self) {
        // 计算功率
        self.monitor_state.power = ((self.monitor_state.voltage as i32 * self.monitor_state.current as i32) / 1000) as i16;
        
        // 计算电量百分比
        self.monitor_state.state_of_charge = self.calculate_state_of_charge();
        
        // 计算剩余容量
        self.monitor_state.remaining_capacity = 
            (self.battery_config.capacity * self.monitor_state.state_of_charge as u16) / 100;
        
        // 计算剩余时间
        self.monitor_state.remaining_time = self.calculate_remaining_time();
        
        // 评估电池健康状态
        self.monitor_state.health = self.assess_battery_health();
    }
    
    /// 计算电量百分比
    fn calculate_state_of_charge(&self) -> u8 {
        let voltage = self.monitor_state.voltage;
        
        // 基于电压的SOC估算 (简化的线性模型)
        match self.battery_config.battery_type {
            BatteryType::LiIon | BatteryType::LiPo => {
                if voltage >= 4200 { 100 }
                else if voltage <= 3000 { 0 }
                else {
                    let range = 4200 - 3000;
                    let offset = voltage - 3000;
                    ((offset * 100) / range) as u8
                }
            }
            BatteryType::NiMH => {
                if voltage >= 1400 { 100 }
                else if voltage <= 1000 { 0 }
                else {
                    let range = 1400 - 1000;
                    let offset = voltage - 1000;
                    ((offset * 100) / range) as u8
                }
            }
            BatteryType::LeadAcid => {
                if voltage >= 2400 { 100 }
                else if voltage <= 2000 { 0 }
                else {
                    let range = 2400 - 2000;
                    let offset = voltage - 2000;
                    ((offset * 100) / range) as u8
                }
            }
        }
    }
    
    /// 计算剩余时间
    fn calculate_remaining_time(&self) -> u32 {
        if self.monitor_state.current <= 0 {
            return u32::MAX; // 充电或无消耗
        }
        
        let remaining_capacity = self.monitor_state.remaining_capacity;
        let current_ma = self.monitor_state.current as u32;
        
        // 剩余时间 (分钟)
        (remaining_capacity as u32 * 60) / current_ma
    }
    
    /// 评估电池健康状态
    fn assess_battery_health(&self) -> BatteryHealth {
        // 基于多个因素评估健康状态
        let voltage_health = self.assess_voltage_health();
        let capacity_health = self.assess_capacity_health();
        let resistance_health = self.assess_resistance_health();
        
        // 取最差的健康状态
        let health_scores = [voltage_health, capacity_health, resistance_health];
        *health_scores.iter().min().unwrap_or(&BatteryHealth::Good)
    }
    
    fn assess_voltage_health(&self) -> BatteryHealth {
        let voltage_variance = self.calculate_voltage_variance();
        
        if voltage_variance < 50 { BatteryHealth::Excellent }
        else if voltage_variance < 100 { BatteryHealth::Good }
        else if voltage_variance < 200 { BatteryHealth::Fair }
        else if voltage_variance < 300 { BatteryHealth::Poor }
        else { BatteryHealth::Critical }
    }
    
    fn assess_capacity_health(&self) -> BatteryHealth {
        // 基于实际容量与标称容量的比较
        let capacity_ratio = (self.monitor_state.remaining_capacity * 100) / self.battery_config.capacity;
        
        if capacity_ratio > 90 { BatteryHealth::Excellent }
        else if capacity_ratio > 70 { BatteryHealth::Good }
        else if capacity_ratio > 50 { BatteryHealth::Fair }
        else if capacity_ratio > 30 { BatteryHealth::Poor }
        else { BatteryHealth::Critical }
    }
    
    fn assess_resistance_health(&self) -> BatteryHealth {
        // 基于内阻变化评估
        let estimated_resistance = self.estimate_internal_resistance();
        let resistance_increase = if estimated_resistance > self.battery_config.internal_resistance {
            ((estimated_resistance - self.battery_config.internal_resistance) * 100) / self.battery_config.internal_resistance
        } else { 0 };
        
        if resistance_increase < 20 { BatteryHealth::Excellent }
        else if resistance_increase < 50 { BatteryHealth::Good }
        else if resistance_increase < 100 { BatteryHealth::Fair }
        else if resistance_increase < 200 { BatteryHealth::Poor }
        else { BatteryHealth::Critical }
    }
    
    /// 计算电压方差
    fn calculate_voltage_variance(&self) -> u16 {
        if self.voltage_buffer.len() < 10 {
            return 0;
        }
        
        let sum: u32 = self.voltage_buffer.iter().map(|&x| x as u32).sum();
        let avg = sum / self.voltage_buffer.len() as u32;
        
        let variance: u32 = self.voltage_buffer.iter()
            .map(|&x| {
                let diff = if x as u32 > avg { x as u32 - avg } else { avg - x as u32 };
                diff * diff
            })
            .sum::<u32>() / self.voltage_buffer.len() as u32;
        
        (variance as f32).sqrt() as u16
    }
    
    /// 估算内阻
    fn estimate_internal_resistance(&self) -> u16 {
        if self.voltage_buffer.len() < 10 || self.current_buffer.len() < 10 {
            return self.battery_config.internal_resistance;
        }
        
        // 简化的内阻估算：基于电压和电流的变化
        let voltage_change = self.voltage_buffer.iter().max().unwrap() - self.voltage_buffer.iter().min().unwrap();
        let current_change = self.current_buffer.iter().max().unwrap() - self.current_buffer.iter().min().unwrap();
        
        if current_change > 100 {
            (voltage_change * 1000) / current_change as u16
        } else {
            self.battery_config.internal_resistance
        }
    }
    
    /// 更新统计数据
    fn update_statistics(&mut self) {
        // 更新最大最小值
        if self.monitor_state.voltage > self.statistics.max_voltage_seen {
            self.statistics.max_voltage_seen = self.monitor_state.voltage;
        }
        if self.monitor_state.voltage < self.statistics.min_voltage_seen {
            self.statistics.min_voltage_seen = self.monitor_state.voltage;
        }
        
        let current_abs = self.monitor_state.current.abs() as u16;
        if current_abs > self.statistics.max_current_seen {
            self.statistics.max_current_seen = current_abs;
        }
        
        // 更新累积电量
        if self.monitor_state.current > 0 {
            // 放电
            self.statistics.total_charge_out += (self.monitor_state.current as u32) / 3600; // mAs to mAh
        } else if self.monitor_state.current < 0 {
            // 充电
            self.statistics.total_charge_in += ((-self.monitor_state.current) as u32) / 3600;
        }
        
        // 更新平均值
        self.update_averages();
        
        // 更新运行时间
        self.statistics.uptime += 1; // 假设每次调用间隔1秒
    }
    
    fn update_averages(&mut self) {
        if !self.voltage_buffer.is_empty() {
            let sum: u32 = self.voltage_buffer.iter().map(|&x| x as u32).sum();
            self.statistics.average_voltage = (sum / self.voltage_buffer.len() as u32) as u16;
        }
        
        if !self.current_buffer.is_empty() {
            let sum: i32 = self.current_buffer.iter().map(|&x| x as i32).sum();
            self.statistics.average_current = ((sum / self.current_buffer.len() as i32).abs()) as u16;
        }
    }
    
    /// 检查报警条件
    fn check_alarms(&mut self) {
        let mut alarm_active = false;
        
        // 低电压报警
        if self.monitor_state.voltage < self.battery_config.min_voltage + 200 {
            alarm_active = true;
        }
        
        // 过电压报警
        if self.monitor_state.voltage > self.battery_config.max_voltage {
            alarm_active = true;
        }
        
        // 过流报警
        if self.monitor_state.current.abs() as u16 > self.battery_config.max_current {
            alarm_active = true;
        }
        
        // 健康状态报警
        if self.monitor_state.health == BatteryHealth::Critical {
            alarm_active = true;
        }
        
        // 设置报警LED
        if alarm_active {
            self.error_led.set_high();
        } else {
            self.error_led.set_low();
        }
    }
    
    /// 更新LED状态
    fn update_led_status(&mut self) {
        // 状态LED：根据电量闪烁
        match self.monitor_state.state_of_charge {
            80..=100 => {
                self.status_led.set_high();
                self.warning_led.set_low();
            }
            20..=79 => {
                // 正常闪烁
                static mut BLINK_COUNTER: u32 = 0;
                unsafe {
                    BLINK_COUNTER += 1;
                    if BLINK_COUNTER % 100 == 0 {
                        self.status_led.toggle();
                    }
                }
                self.warning_led.set_low();
            }
            0..=19 => {
                // 快速闪烁
                static mut FAST_BLINK_COUNTER: u32 = 0;
                unsafe {
                    FAST_BLINK_COUNTER += 1;
                    if FAST_BLINK_COUNTER % 25 == 0 {
                        self.status_led.toggle();
                    }
                }
                self.warning_led.set_high();
            }
        }
    }
    
    /// 发送状态报告
    fn send_status_report(&mut self) {
        let mut report = String::<256>::new();
        
        // 格式化状态报告
        report.push_str("=== Battery Status Report ===\r\n").ok();
        
        // 基本状态
        let voltage_str = format_voltage(self.monitor_state.voltage);
        let current_str = format_current(self.monitor_state.current);
        let power_str = format_power(self.monitor_state.power);
        
        report.push_str(&format!("Voltage: {}V\r\n", voltage_str)).ok();
        report.push_str(&format!("Current: {}A\r\n", current_str)).ok();
        report.push_str(&format!("Power: {}W\r\n", power_str)).ok();
        report.push_str(&format!("SOC: {}%\r\n", self.monitor_state.state_of_charge)).ok();
        report.push_str(&format!("Remaining: {}mAh\r\n", self.monitor_state.remaining_capacity)).ok();
        
        // 健康状态
        let health_str = match self.monitor_state.health {
            BatteryHealth::Excellent => "Excellent",
            BatteryHealth::Good => "Good",
            BatteryHealth::Fair => "Fair",
            BatteryHealth::Poor => "Poor",
            BatteryHealth::Critical => "Critical",
        };
        report.push_str(&format!("Health: {}\r\n", health_str)).ok();
        
        // 统计数据
        report.push_str(&format!("Uptime: {}s\r\n", self.statistics.uptime)).ok();
        report.push_str(&format!("Charge In: {}mAh\r\n", self.statistics.total_charge_in)).ok();
        report.push_str(&format!("Charge Out: {}mAh\r\n", self.statistics.total_charge_out)).ok();
        
        report.push_str("=============================\r\n").ok();
        
        // 发送报告
        self.send_message(&report);
    }
    
    /// 发送消息
    fn send_message(&mut self, message: &str) {
        for byte in message.bytes() {
            nb::block!(self.serial.write(byte)).ok();
        }
    }
    
    /// 校准ADC
    fn calibrate_adc(&mut self) -> Result<(), &'static str> {
        // 简化的校准过程
        // 在实际应用中，这里应该包含更复杂的校准算法
        
        // 零点校准
        let mut voltage_sum = 0u32;
        let mut current_sum = 0i32;
        
        for _ in 0..100 {
            voltage_sum += self.voltage_adc.read(&mut self.voltage_pin).unwrap_or(0) as u32;
            current_sum += (self.current_adc.read(&mut self.current_pin).unwrap_or(2048) as i32) - 2048;
            cortex_m::asm::delay(1000);
        }
        
        // 计算偏移
        let voltage_offset = (voltage_sum / 100) as i16;
        let current_offset = (current_sum / 100) as i16;
        
        self.calibration.voltage_offset = -voltage_offset;
        self.calibration.current_offset = -current_offset;
        
        Ok(())
    }
    
    /// 初始化电池状态
    fn initialize_battery_state(&mut self) -> Result<(), &'static str> {
        // 读取初始电压
        let initial_voltage = self.read_voltage();
        self.monitor_state.voltage = initial_voltage;
        
        // 估算初始电量
        self.monitor_state.state_of_charge = self.calculate_state_of_charge();
        self.monitor_state.remaining_capacity = 
            (self.battery_config.capacity * self.monitor_state.state_of_charge as u16) / 100;
        
        // 初始化统计数据
        self.statistics.min_voltage_seen = initial_voltage;
        self.statistics.max_voltage_seen = initial_voltage;
        
        Ok(())
    }
}

/// 格式化电压显示
fn format_voltage(voltage_mv: u16) -> &'static str {
    // 简化的格式化，实际应用中应该返回格式化的字符串
    match voltage_mv {
        0..=3000 => "Low",
        3001..=3700 => "Normal",
        3701..=4200 => "High",
        _ => "Over",
    }
}

/// 格式化电流显示
fn format_current(current_ma: i16) -> &'static str {
    match current_ma {
        i16::MIN..=-100 => "Charging",
        -99..=99 => "Idle",
        100..=i16::MAX => "Discharging",
    }
}

/// 格式化功率显示
fn format_power(power_mw: i16) -> &'static str {
    match power_mw.abs() {
        0..=1000 => "Low",
        1001..=5000 => "Medium",
        5001..=i16::MAX => "High",
    }
}

#[entry]
fn main() -> ! {
    // 获取外设
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr
        .use_hse(8.mhz())
        .sysclk(84.mhz())
        .pclk1(42.mhz())
        .pclk2(84.mhz())
        .freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // ADC配置
    let adc1_config = AdcConfig::default();
    let adc2_config = AdcConfig::default();
    let voltage_adc = Adc::adc1(dp.ADC1, true, adc1_config);
    let current_adc = Adc::adc2(dp.ADC2, true, adc2_config);
    
    let voltage_pin = gpioa.pa1.into_analog();
    let current_pin = gpioa.pa2.into_analog();
    
    // LED配置
    let status_led = gpioc.pc13.into_push_pull_output();
    let warning_led = gpiob.pb0.into_push_pull_output();
    let error_led = gpiob.pb1.into_push_pull_output();
    
    // 串口配置
    let tx = gpioa.pa9.into_alternate_af7();
    let rx = gpioa.pa10.into_alternate_af7();
    let serial_config = SerialConfig::default().baudrate(115200.bps());
    let serial = Serial::usart1(dp.USART1, (tx, rx), serial_config, clocks).unwrap();
    
    // 创建监控系统
    let mut monitor = BatteryMonitorSystem::new(
        voltage_adc,
        current_adc,
        voltage_pin,
        current_pin,
        status_led,
        warning_led,
        error_led,
        serial,
    );
    
    // 初始化
    monitor.initialize().unwrap();
    
    // 运行监控循环
    monitor.run_monitoring()
}