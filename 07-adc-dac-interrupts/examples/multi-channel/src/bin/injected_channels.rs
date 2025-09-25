#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac::{self, interrupt, Interrupt},
    prelude::*,
    adc::{Adc, AdcConfig, SampleTime},
    gpio::Analog,
    timer::{Timer, Event},
    serial::{Serial, Config},
};
use nb::block;
use heapless::{String, Vec};
use core::fmt::Write;
use cortex_m::interrupt::{free, Mutex};
use core::cell::RefCell;

const INJECTED_CHANNELS: usize = 4;
const REGULAR_CHANNELS: usize = 4;

// 全局变量用于中断处理
static INJECTED_RESULTS: Mutex<RefCell<Option<Vec<u16, INJECTED_CHANNELS>>>> = 
    Mutex::new(RefCell::new(None));
static CONVERSION_COMPLETE: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置串口
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    let mut serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &clocks,
    ).unwrap();
    
    // 配置ADC引脚
    // 常规通道
    let _regular_pin0 = gpioa.pa0.into_analog(); // ADC1_IN0
    let _regular_pin1 = gpioa.pa1.into_analog(); // ADC1_IN1
    let _regular_pin4 = gpioa.pa4.into_analog(); // ADC1_IN4
    let _regular_pin5 = gpioa.pa5.into_analog(); // ADC1_IN5
    
    // 注入通道 (高优先级)
    let _injected_pin6 = gpioa.pa6.into_analog(); // ADC1_IN6
    let _injected_pin7 = gpioa.pa7.into_analog(); // ADC1_IN7
    let _injected_pin8 = gpiob.pb0.into_analog(); // ADC1_IN8
    let _injected_pin9 = gpiob.pb1.into_analog(); // ADC1_IN9
    
    // 配置ADC
    let adc_config = AdcConfig::default()
        .sample_time(SampleTime::Cycles_480);
    let mut adc = Adc::adc1(dp.ADC1, true, adc_config);
    
    // 配置定时器用于触发注入转换
    let mut timer = Timer::new(dp.TIM3, &clocks).counter_hz();
    timer.start(100.Hz()).unwrap(); // 100Hz注入转换触发
    
    // 配置常规转换定时器
    let mut regular_timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    regular_timer.start(10.Hz()).unwrap(); // 10Hz常规转换
    
    // 启用ADC中断
    cp.NVIC.enable(Interrupt::ADC);
    
    writeln!(serial, "ADC注入通道示例启动").ok();
    writeln!(serial, "常规通道: PA0, PA1, PA4, PA5 (10Hz)").ok();
    writeln!(serial, "注入通道: PA6, PA7, PB0, PB1 (100Hz)").ok();
    writeln!(serial, "注入通道具有更高优先级，可以中断常规转换").ok();
    writeln!(serial, "").ok();
    
    let mut regular_count = 0u32;
    let mut injected_count = 0u32;
    let vref = 3.3f32;
    
    // 配置注入通道序列
    configure_injected_channels();
    
    loop {
        // 检查注入转换完成
        free(|cs| {
            if *CONVERSION_COMPLETE.borrow(cs).borrow() {
                *CONVERSION_COMPLETE.borrow(cs).borrow_mut() = false;
                
                if let Some(results) = INJECTED_RESULTS.borrow(cs).borrow_mut().take() {
                    injected_count += 1;
                    process_injected_results(&mut serial, &results, injected_count, vref);
                }
            }
        });
        
        // 常规转换处理
        if regular_timer.wait().is_ok() {
            regular_count += 1;
            
            // 执行常规转换
            let regular_results = perform_regular_conversion();
            process_regular_results(&mut serial, &regular_results, regular_count, vref);
        }
        
        // 触发注入转换
        if timer.wait().is_ok() {
            trigger_injected_conversion();
        }
    }
}

/// 配置注入通道
fn configure_injected_channels() {
    // 配置注入通道序列
    // 设置ADC_JSQR寄存器
    // JL[1:0] = 11 (4个注入通道)
    // JSQ4[4:0] = 6 (通道6)
    // JSQ3[4:0] = 7 (通道7)  
    // JSQ2[4:0] = 8 (通道8)
    // JSQ1[4:0] = 9 (通道9)
    
    // 启用注入转换完成中断
    // 设置ADC_CR1的JEOCIE位
}

/// 触发注入转换
fn trigger_injected_conversion() {
    // 设置ADC_CR2的JSWSTART位启动注入转换
}

/// 执行常规转换
fn perform_regular_conversion() -> Vec<u16, REGULAR_CHANNELS> {
    let mut results = Vec::new();
    
    // 模拟常规通道转换
    for i in 0..REGULAR_CHANNELS {
        let adc_value = simulate_regular_adc(i as u8);
        results.push(adc_value).ok();
    }
    
    results
}

/// 处理注入转换结果
fn process_injected_results(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    results: &Vec<u16, INJECTED_CHANNELS>,
    count: u32,
    vref: f32
) {
    // 每10次输出一次注入结果
    if count % 10 == 0 {
        let mut output: String<256> = String::new();
        write!(output, "注入#{:04}: ", count).ok();
        
        for (i, &adc_value) in results.iter().enumerate() {
            let voltage = adc_to_voltage(adc_value, vref, 12);
            write!(output, "INJ{}: {:.3}V ", i, voltage).ok();
        }
        
        writeln!(serial, "{}", output).ok();
    }
    
    // 检查关键信号 (假设注入通道用于监控关键参数)
    check_critical_signals(serial, results, count, vref);
}

/// 处理常规转换结果
fn process_regular_results(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    results: &Vec<u16, REGULAR_CHANNELS>,
    count: u32,
    vref: f32
) {
    // 每5次输出一次常规结果
    if count % 5 == 0 {
        let mut output: String<256> = String::new();
        write!(output, "常规#{:04}: ", count).ok();
        
        for (i, &adc_value) in results.iter().enumerate() {
            let voltage = adc_to_voltage(adc_value, vref, 12);
            write!(output, "REG{}: {:.3}V ", i, voltage).ok();
        }
        
        writeln!(serial, "{}", output).ok();
    }
}

/// 检查关键信号
fn check_critical_signals(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    results: &Vec<u16, INJECTED_CHANNELS>,
    count: u32,
    vref: f32
) {
    // 假设注入通道0监控电源电压
    if let Some(&power_adc) = results.get(0) {
        let power_voltage = adc_to_voltage(power_adc, vref, 12);
        if power_voltage < 2.8 || power_voltage > 3.5 {
            writeln!(serial, "警告: 电源电压异常 {:.3}V (注入#{:04})", 
                    power_voltage, count).ok();
        }
    }
    
    // 假设注入通道1监控温度传感器
    if let Some(&temp_adc) = results.get(1) {
        let temp_voltage = adc_to_voltage(temp_adc, vref, 12);
        // 简单的温度转换 (假设线性传感器)
        let temperature = (temp_voltage - 0.5) * 100.0; // 假设500mV = 0°C, 10mV/°C
        
        if temperature > 85.0 || temperature < -10.0 {
            writeln!(serial, "警告: 温度异常 {:.1}°C (注入#{:04})", 
                    temperature, count).ok();
        }
    }
    
    // 假设注入通道2和3监控差分信号
    if let (Some(&sig_p), Some(&sig_n)) = (results.get(2), results.get(3)) {
        let voltage_p = adc_to_voltage(sig_p, vref, 12);
        let voltage_n = adc_to_voltage(sig_n, vref, 12);
        let diff_voltage = voltage_p - voltage_n;
        
        if diff_voltage.abs() > 2.0 {
            writeln!(serial, "警告: 差分信号异常 {:.3}V (注入#{:04})", 
                    diff_voltage, count).ok();
        }
    }
}

/// ADC中断处理函数
#[interrupt]
fn ADC() {
    free(|cs| {
        // 检查注入转换完成标志
        // if ADC1.SR.read().jeoc().bit_is_set() {
            // 读取注入数据寄存器
            let mut results = Vec::new();
            
            // 读取JDR1-JDR4寄存器
            for i in 0..INJECTED_CHANNELS {
                let adc_value = read_injected_data_register(i);
                results.push(adc_value).ok();
            }
            
            // 存储结果
            *INJECTED_RESULTS.borrow(cs).borrow_mut() = Some(results);
            *CONVERSION_COMPLETE.borrow(cs).borrow_mut() = true;
            
            // 清除中断标志
            // ADC1.SR.modify(|_, w| w.jeoc().clear_bit());
        // }
    });
}

/// 读取注入数据寄存器
fn read_injected_data_register(channel: usize) -> u16 {
    // 模拟读取JDR寄存器
    simulate_injected_adc(channel as u8)
}

/// ADC值转换为电压
fn adc_to_voltage(adc_value: u16, vref: f32, resolution: u8) -> f32 {
    let max_value = (1 << resolution) - 1;
    (adc_value as f32 / max_value as f32) * vref
}

/// 模拟常规ADC读取
fn simulate_regular_adc(channel: u8) -> u16 {
    use micromath::F32Ext;
    
    match channel {
        0 => 1650,  // 1.35V
        1 => 2048,  // 1.65V  
        2 => 2450,  // 1.98V
        3 => 1024,  // 0.825V
        _ => 2048,
    }
}

/// 模拟注入ADC读取
fn simulate_injected_adc(channel: u8) -> u16 {
    use micromath::F32Ext;
    
    static mut COUNTER: u32 = 0;
    
    unsafe {
        COUNTER += 1;
        
        match channel {
            0 => 4000 + ((COUNTER % 100) as i16 - 50), // 电源电压 (~3.23V + 噪声)
            1 => 2048 + (50.0 * (COUNTER as f32 * 0.1).sin()) as i16, // 温度传感器
            2 => 2500 + (100.0 * (COUNTER as f32 * 0.05).sin()) as i16, // 差分信号+
            3 => 1500 + (100.0 * (COUNTER as f32 * 0.05).cos()) as i16, // 差分信号-
            _ => 2048,
        }
    }.max(0).min(4095) as u16
}

/// 注入通道配置结构
struct InjectedChannelConfig {
    channels: Vec<u8, INJECTED_CHANNELS>,
    sample_times: Vec<SampleTime, INJECTED_CHANNELS>,
    trigger_source: InjectedTrigger,
    auto_injection: bool,
}

#[derive(Debug, Clone, Copy)]
enum InjectedTrigger {
    Software,
    Timer1CC4,
    Timer2TRGO,
    Timer3CC1,
    Timer4CC4,
    External,
}

impl Default for InjectedChannelConfig {
    fn default() -> Self {
        let mut channels = Vec::new();
        let mut sample_times = Vec::new();
        
        // 默认配置4个注入通道
        for i in 6..10 {
            channels.push(i).ok();
            sample_times.push(SampleTime::Cycles_480).ok();
        }
        
        Self {
            channels,
            sample_times,
            trigger_source: InjectedTrigger::Software,
            auto_injection: false,
        }
    }
}

/// 注入通道管理器
struct InjectedChannelManager {
    config: InjectedChannelConfig,
    priority_levels: Vec<u8, INJECTED_CHANNELS>,
    interrupt_enabled: bool,
}

impl InjectedChannelManager {
    fn new(config: InjectedChannelConfig) -> Self {
        let mut priority_levels = Vec::new();
        
        // 设置默认优先级
        for i in 0..INJECTED_CHANNELS {
            priority_levels.push(i as u8).ok();
        }
        
        Self {
            config,
            priority_levels,
            interrupt_enabled: true,
        }
    }
    
    fn configure_trigger(&mut self, trigger: InjectedTrigger) {
        self.config.trigger_source = trigger;
        // 配置ADC_CR2的JEXTSEL位
    }
    
    fn enable_auto_injection(&mut self, enable: bool) {
        self.config.auto_injection = enable;
        // 配置ADC_CR1的JAUTO位
    }
    
    fn set_channel_priority(&mut self, channel: usize, priority: u8) {
        if channel < self.priority_levels.len() {
            self.priority_levels[channel] = priority;
        }
    }
}