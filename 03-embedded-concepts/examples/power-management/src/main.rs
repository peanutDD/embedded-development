#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{gpioa::PA0, gpioc::PC13, Input, Output, PullUp, PushPull},
    timer::{CounterUs, Event},
    adc::{Adc, config::AdcConfig},
};
use cortex_m::{interrupt, peripheral::NVIC};
use heapless::Vec;

// 功耗管理状态
#[derive(Debug, Clone, Copy, PartialEq)]
enum PowerState {
    Active,
    LowPower,
    Sleep,
    DeepSleep,
    Standby,
}

// 功耗管理器
struct PowerManager {
    current_state: PowerState,
    target_state: PowerState,
    activity_counter: u32,
    idle_threshold: u32,
    wake_sources: u8,
    power_statistics: PowerStatistics,
}

#[derive(Debug, Clone, Copy)]
struct PowerStatistics {
    active_time_ms: u32,
    low_power_time_ms: u32,
    sleep_time_ms: u32,
    deep_sleep_time_ms: u32,
    standby_time_ms: u32,
    state_transitions: u32,
    wake_events: u32,
}

impl PowerManager {
    fn new() -> Self {
        Self {
            current_state: PowerState::Active,
            target_state: PowerState::Active,
            activity_counter: 0,
            idle_threshold: 1000, // 1秒无活动后进入低功耗模式
            wake_sources: 0,
            power_statistics: PowerStatistics {
                active_time_ms: 0,
                low_power_time_ms: 0,
                sleep_time_ms: 0,
                deep_sleep_time_ms: 0,
                standby_time_ms: 0,
                state_transitions: 0,
                wake_events: 0,
            },
        }
    }
    
    fn update_activity(&mut self) {
        self.activity_counter = 0; // 重置活动计数器
        if self.current_state != PowerState::Active {
            self.target_state = PowerState::Active;
        }
    }
    
    fn tick(&mut self) {
        self.activity_counter += 1;
        
        // 根据活动情况决定目标状态
        match self.current_state {
            PowerState::Active => {
                if self.activity_counter > self.idle_threshold {
                    self.target_state = PowerState::LowPower;
                }
            }
            PowerState::LowPower => {
                if self.activity_counter > self.idle_threshold * 2 {
                    self.target_state = PowerState::Sleep;
                }
            }
            PowerState::Sleep => {
                if self.activity_counter > self.idle_threshold * 5 {
                    self.target_state = PowerState::DeepSleep;
                }
            }
            _ => {}
        }
    }
    
    fn transition_to_target_state(&mut self) {
        if self.current_state != self.target_state {
            self.exit_current_state();
            self.current_state = self.target_state;
            self.enter_current_state();
            self.power_statistics.state_transitions += 1;
        }
    }
    
    fn exit_current_state(&mut self) {
        match self.current_state {
            PowerState::Active => {
                // 保存活动状态的上下文
            }
            PowerState::LowPower => {
                // 恢复一些外设
                self.enable_non_critical_peripherals();
            }
            PowerState::Sleep => {
                // 恢复时钟和外设
                self.restore_clocks();
            }
            PowerState::DeepSleep => {
                // 完全恢复系统状态
                self.full_system_restore();
            }
            PowerState::Standby => {
                // 从待机模式恢复（通常是系统重启）
            }
        }
    }
    
    fn enter_current_state(&mut self) {
        match self.current_state {
            PowerState::Active => {
                // 启用所有外设和最高性能
                self.enable_all_peripherals();
                self.set_high_performance_mode();
            }
            PowerState::LowPower => {
                // 降低时钟频率，禁用非关键外设
                self.reduce_clock_frequency();
                self.disable_non_critical_peripherals();
            }
            PowerState::Sleep => {
                // 进入睡眠模式
                self.enter_sleep_mode();
            }
            PowerState::DeepSleep => {
                // 进入深度睡眠模式
                self.enter_deep_sleep_mode();
            }
            PowerState::Standby => {
                // 进入待机模式
                self.enter_standby_mode();
            }
        }
    }
    
    fn enable_all_peripherals(&self) {
        unsafe {
            let rcc = &*pac::RCC::ptr();
            
            // 启用所有GPIO时钟
            rcc.ahb1enr.modify(|_, w| {
                w.gpioaen().set_bit()
                 .gpioben().set_bit()
                 .gpiocen().set_bit()
                 .gpioden().set_bit()
                 .gpioeen().set_bit()
            });
            
            // 启用定时器时钟
            rcc.apb1enr.modify(|_, w| {
                w.tim2en().set_bit()
                 .tim3en().set_bit()
                 .tim4en().set_bit()
            });
            
            // 启用ADC时钟
            rcc.apb2enr.modify(|_, w| w.adc1en().set_bit());
        }
    }
    
    fn disable_non_critical_peripherals(&self) {
        unsafe {
            let rcc = &*pac::RCC::ptr();
            
            // 禁用非关键外设时钟
            rcc.apb1enr.modify(|_, w| {
                w.tim3en().clear_bit()
                 .tim4en().clear_bit()
            });
            
            // 禁用部分GPIO
            rcc.ahb1enr.modify(|_, w| {
                w.gpioden().clear_bit()
                 .gpioeen().clear_bit()
            });
        }
    }
    
    fn enable_non_critical_peripherals(&self) {
        unsafe {
            let rcc = &*pac::RCC::ptr();
            
            rcc.apb1enr.modify(|_, w| {
                w.tim3en().set_bit()
                 .tim4en().set_bit()
            });
            
            rcc.ahb1enr.modify(|_, w| {
                w.gpioden().set_bit()
                 .gpioeen().set_bit()
            });
        }
    }
    
    fn set_high_performance_mode(&self) {
        unsafe {
            let rcc = &*pac::RCC::ptr();
            let flash = &*pac::FLASH::ptr();
            
            // 设置最高时钟频率 (84MHz)
            // 配置Flash等待状态
            flash.acr.modify(|_, w| w.latency().bits(2));
            
            // 启用HSE
            rcc.cr.modify(|_, w| w.hseon().set_bit());
            while rcc.cr.read().hserdy().bit_is_clear() {}
            
            // 配置PLL
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
            
            // 切换到PLL
            rcc.cfgr.modify(|_, w| w.sw().pll());
            while !rcc.cfgr.read().sws().is_pll() {}
        }
    }
    
    fn reduce_clock_frequency(&self) {
        unsafe {
            let rcc = &*pac::RCC::ptr();
            let flash = &*pac::FLASH::ptr();
            
            // 降低到中等频率 (42MHz)
            flash.acr.modify(|_, w| w.latency().bits(1));
            
            // 重新配置PLL为较低频率
            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            while rcc.cr.read().pllrdy().bit_is_set() {}
            
            rcc.pllcfgr.modify(|_, w| {
                w.pllsrc().hse()
                 .pllm().bits(8)
                 .plln().bits(168)
                 .pllp().div4()
                 .pllq().bits(7)
            });
            
            rcc.cr.modify(|_, w| w.pllon().set_bit());
            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }
    }
    
    fn restore_clocks(&self) {
        // 恢复到高性能模式的时钟设置
        self.set_high_performance_mode();
    }
    
    fn full_system_restore(&self) {
        // 完全恢复系统状态
        self.restore_clocks();
        self.enable_all_peripherals();
    }
    
    fn enter_sleep_mode(&self) {
        unsafe {
            let scb = &*cortex_m::peripheral::SCB::ptr();
            
            // 配置睡眠模式
            scb.scr.modify(|v| v & !0x04); // 清除SLEEPDEEP位
            
            // 进入睡眠模式
            cortex_m::asm::wfi();
        }
    }
    
    fn enter_deep_sleep_mode(&self) {
        unsafe {
            let scb = &*cortex_m::peripheral::SCB::ptr();
            let pwr = &*pac::PWR::ptr();
            let rcc = &*pac::RCC::ptr();
            
            // 启用PWR时钟
            rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
            
            // 配置深度睡眠模式
            scb.scr.modify(|v| v | 0x04); // 设置SLEEPDEEP位
            
            // 配置电源控制
            pwr.cr.modify(|_, w| w.pdds().clear_bit()); // 进入Stop模式而不是Standby
            
            // 进入深度睡眠
            cortex_m::asm::wfi();
        }
    }
    
    fn enter_standby_mode(&self) {
        unsafe {
            let pwr = &*pac::PWR::ptr();
            let rcc = &*pac::RCC::ptr();
            
            // 启用PWR时钟
            rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
            
            // 清除唤醒标志
            pwr.cr.modify(|_, w| w.cwuf().set_bit());
            
            // 配置待机模式
            pwr.cr.modify(|_, w| w.pdds().set_bit());
            
            // 设置SLEEPDEEP位
            let scb = &*cortex_m::peripheral::SCB::ptr();
            scb.scr.modify(|v| v | 0x04);
            
            // 进入待机模式
            cortex_m::asm::wfi();
        }
    }
    
    fn configure_wake_sources(&mut self, sources: u8) {
        self.wake_sources = sources;
        
        unsafe {
            let pwr = &*pac::PWR::ptr();
            let rcc = &*pac::RCC::ptr();
            
            rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
            
            // 配置唤醒引脚
            if sources & 0x01 != 0 {
                pwr.csr.modify(|_, w| w.ewup().set_bit());
            }
            
            // 配置RTC唤醒
            if sources & 0x02 != 0 {
                // 配置RTC唤醒（需要RTC初始化）
            }
            
            // 配置外部中断唤醒
            if sources & 0x04 != 0 {
                // 配置EXTI唤醒
            }
        }
    }
    
    fn handle_wake_event(&mut self) {
        self.power_statistics.wake_events += 1;
        self.activity_counter = 0;
        self.target_state = PowerState::Active;
    }
    
    fn get_power_consumption_estimate(&self) -> u32 {
        // 返回估计的功耗（微安）
        match self.current_state {
            PowerState::Active => 50000,      // 50mA
            PowerState::LowPower => 20000,    // 20mA
            PowerState::Sleep => 2000,        // 2mA
            PowerState::DeepSleep => 200,     // 0.2mA
            PowerState::Standby => 10,        // 0.01mA
        }
    }
}

// 动态电压频率调节 (DVFS)
struct DvfsController {
    current_frequency: u32,
    current_voltage: u32,
    target_frequency: u32,
    load_threshold_high: u8,
    load_threshold_low: u8,
    cpu_load_history: Vec<u8, 16>,
}

impl DvfsController {
    fn new() -> Self {
        Self {
            current_frequency: 84_000_000, // 84MHz
            current_voltage: 3300,         // 3.3V (mV)
            target_frequency: 84_000_000,
            load_threshold_high: 80,       // 80%
            load_threshold_low: 30,        // 30%
            cpu_load_history: Vec::new(),
        }
    }
    
    fn update_cpu_load(&mut self, load_percent: u8) {
        if self.cpu_load_history.len() >= 16 {
            self.cpu_load_history.remove(0);
        }
        let _ = self.cpu_load_history.push(load_percent);
        
        // 计算平均负载
        let avg_load = self.calculate_average_load();
        
        // 根据负载调整频率
        if avg_load > self.load_threshold_high {
            self.increase_frequency();
        } else if avg_load < self.load_threshold_low {
            self.decrease_frequency();
        }
    }
    
    fn calculate_average_load(&self) -> u8 {
        if self.cpu_load_history.is_empty() {
            return 0;
        }
        
        let sum: u32 = self.cpu_load_history.iter().map(|&x| x as u32).sum();
        (sum / self.cpu_load_history.len() as u32) as u8
    }
    
    fn increase_frequency(&mut self) {
        let new_freq = core::cmp::min(self.current_frequency * 2, 84_000_000);
        if new_freq != self.current_frequency {
            self.target_frequency = new_freq;
            self.apply_frequency_change();
        }
    }
    
    fn decrease_frequency(&mut self) {
        let new_freq = core::cmp::max(self.current_frequency / 2, 8_000_000);
        if new_freq != self.current_frequency {
            self.target_frequency = new_freq;
            self.apply_frequency_change();
        }
    }
    
    fn apply_frequency_change(&mut self) {
        // 实际的频率切换实现
        unsafe {
            let rcc = &*pac::RCC::ptr();
            let flash = &*pac::FLASH::ptr();
            
            // 根据目标频率调整Flash等待状态
            let wait_states = if self.target_frequency > 60_000_000 {
                2
            } else if self.target_frequency > 30_000_000 {
                1
            } else {
                0
            };
            
            flash.acr.modify(|_, w| w.latency().bits(wait_states));
            
            // 重新配置PLL
            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            while rcc.cr.read().pllrdy().bit_is_set() {}
            
            let pll_n = (self.target_frequency / 1_000_000) * 8 / 8; // 简化计算
            rcc.pllcfgr.modify(|_, w| {
                w.pllsrc().hse()
                 .pllm().bits(8)
                 .plln().bits(pll_n as u16)
                 .pllp().div4()
                 .pllq().bits(7)
            });
            
            rcc.cr.modify(|_, w| w.pllon().set_bit());
            while rcc.cr.read().pllrdy().bit_is_clear() {}
            
            self.current_frequency = self.target_frequency;
        }
    }
    
    fn get_power_efficiency(&self) -> f32 {
        // 计算功耗效率 (MIPS/Watt)
        let mips = self.current_frequency / 1_000_000;
        let power_mw = self.estimate_power_consumption();
        
        if power_mw > 0 {
            mips as f32 / power_mw as f32 * 1000.0
        } else {
            0.0
        }
    }
    
    fn estimate_power_consumption(&self) -> u32 {
        // 估计功耗 (mW)
        // 简化的功耗模型：P = C * V^2 * f
        let voltage_factor = (self.current_voltage * self.current_voltage) / (3300 * 3300);
        let freq_factor = self.current_frequency / 84_000_000;
        
        (100 * voltage_factor * freq_factor) as u32 // 基础功耗100mW
    }
}

// 全局变量
static mut POWER_MANAGER: Option<PowerManager> = None;
static mut DVFS_CONTROLLER: Option<DvfsController> = None;

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    
    let mut led = gpioc.pc13.into_push_pull_output();
    let button = gpioa.pa0.into_pull_up_input();
    
    // 配置定时器
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(1.secs()).unwrap();
    timer.listen(Event::Update);
    
    // 配置ADC用于监控电压
    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());
    
    // 初始化功耗管理器
    unsafe {
        POWER_MANAGER = Some(PowerManager::new());
        DVFS_CONTROLLER = Some(DvfsController::new());
    }
    
    // 配置唤醒源
    unsafe {
        if let Some(ref mut pm) = POWER_MANAGER {
            pm.configure_wake_sources(0x07); // 启用所有唤醒源
        }
    }
    
    // 启用中断
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }
    
    let mut tick_counter = 0u32;
    let mut cpu_load_counter = 0u32;
    
    loop {
        // 模拟CPU负载计算
        let cpu_load = calculate_cpu_load();
        
        // 更新DVFS控制器
        unsafe {
            if let Some(ref mut dvfs) = DVFS_CONTROLLER {
                dvfs.update_cpu_load(cpu_load);
            }
        }
        
        // 检查按钮状态（模拟用户活动）
        if button.is_low() {
            unsafe {
                if let Some(ref mut pm) = POWER_MANAGER {
                    pm.update_activity();
                }
            }
            led.set_low();
        } else {
            led.set_high();
        }
        
        // 更新功耗管理器
        unsafe {
            if let Some(ref mut pm) = POWER_MANAGER {
                pm.tick();
                pm.transition_to_target_state();
            }
        }
        
        // 每秒打印一次状态信息
        tick_counter += 1;
        if tick_counter >= 1000 {
            print_power_status();
            tick_counter = 0;
        }
        
        // 模拟工作负载
        for _ in 0..1000 {
            cpu_load_counter = cpu_load_counter.wrapping_add(1);
        }
        
        // 短暂延时
        cortex_m::asm::delay(10000);
    }
}

fn calculate_cpu_load() -> u8 {
    // 简化的CPU负载计算
    // 在实际应用中，这应该基于实际的CPU使用情况
    static mut LOAD_COUNTER: u32 = 0;
    
    unsafe {
        LOAD_COUNTER = LOAD_COUNTER.wrapping_add(1);
        ((LOAD_COUNTER % 100) as u8).saturating_add(20)
    }
}

fn print_power_status() {
    // 在实际应用中，这里会通过串口或RTT输出状态信息
    unsafe {
        if let Some(ref pm) = POWER_MANAGER {
            let consumption = pm.get_power_consumption_estimate();
            // 这里可以输出功耗状态信息
            let _ = consumption; // 避免未使用变量警告
        }
        
        if let Some(ref dvfs) = DVFS_CONTROLLER {
            let efficiency = dvfs.get_power_efficiency();
            let power = dvfs.estimate_power_consumption();
            let _ = (efficiency, power); // 避免未使用变量警告
        }
    }
}

// 定时器中断处理
#[interrupt]
fn TIM2() {
    unsafe {
        let timer = &*pac::TIM2::ptr();
        timer.sr.modify(|_, w| w.uif().clear_bit());
        
        // 处理定时器中断
        if let Some(ref mut pm) = POWER_MANAGER {
            // 更新功耗统计
            match pm.current_state {
                PowerState::Active => pm.power_statistics.active_time_ms += 1,
                PowerState::LowPower => pm.power_statistics.low_power_time_ms += 1,
                PowerState::Sleep => pm.power_statistics.sleep_time_ms += 1,
                PowerState::DeepSleep => pm.power_statistics.deep_sleep_time_ms += 1,
                PowerState::Standby => pm.power_statistics.standby_time_ms += 1,
            }
        }
    }
}