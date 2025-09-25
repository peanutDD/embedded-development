#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    adc::{Adc, AdcConfig, SampleTime},
    gpio::Analog,
    timer::{Timer, Event},
    serial::{Serial, Config},
};
use nb::block;
use heapless::{String, Vec};
use core::fmt::Write;

use multi_channel::{MultiChannelADC, MultiChannelConfig, ChannelStats};

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置串口用于输出
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    let mut serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &clocks,
    ).unwrap();
    
    // 配置ADC引脚
    let adc_pin0 = gpioa.pa0.into_analog(); // ADC1_IN0
    let adc_pin1 = gpioa.pa1.into_analog(); // ADC1_IN1
    let adc_pin4 = gpioa.pa4.into_analog(); // ADC1_IN4
    let adc_pin5 = gpioa.pa5.into_analog(); // ADC1_IN5
    let adc_pin6 = gpioa.pa6.into_analog(); // ADC1_IN6
    let adc_pin7 = gpioa.pa7.into_analog(); // ADC1_IN7
    
    // 配置ADC
    let adc_config = AdcConfig::default()
        .sample_time(SampleTime::Cycles_480);
    let mut adc = Adc::adc1(dp.ADC1, true, adc_config);
    
    // 创建多通道ADC配置
    let mut config = MultiChannelConfig::default();
    config.sample_time = SampleTime::Cycles_480;
    config.resolution = 12;
    config.scan_mode = true;
    config.continuous_mode = false;
    
    // 添加通道
    config.channels.push(0).ok(); // PA0
    config.channels.push(1).ok(); // PA1
    config.channels.push(4).ok(); // PA4
    config.channels.push(5).ok(); // PA5
    config.channels.push(6).ok(); // PA6
    config.channels.push(7).ok(); // PA7
    
    // 创建多通道ADC管理器
    let mut multi_adc = MultiChannelADC::new(adc, config);
    
    // 配置定时器用于定时采样
    let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer.start(10.Hz()).unwrap(); // 10Hz采样率
    
    writeln!(serial, "多通道ADC采样示例启动").ok();
    writeln!(serial, "采样通道: PA0, PA1, PA4, PA5, PA6, PA7").ok();
    writeln!(serial, "采样率: 10Hz").ok();
    writeln!(serial, "分辨率: 12位").ok();
    writeln!(serial, "参考电压: 3.3V").ok();
    writeln!(serial, "").ok();
    
    let mut sample_count = 0u32;
    let vref = 3.3f32;
    
    loop {
        // 等待定时器事件
        block!(timer.wait()).ok();
        
        // 读取所有通道
        match multi_adc.read_all_channels() {
            Ok(values) => {
                sample_count += 1;
                
                // 输出采样数据
                let mut output: String<256> = String::new();
                write!(output, "Sample #{:04}: ", sample_count).ok();
                
                for (i, &adc_value) in values.iter().enumerate() {
                    let voltage = multi_adc.adc_to_voltage(adc_value, vref);
                    write!(output, "CH{}: {:.3}V ", i, voltage).ok();
                }
                
                writeln!(serial, "{}", output).ok();
                
                // 每100个样本输出统计信息
                if sample_count % 100 == 0 {
                    writeln!(serial, "\n=== 统计信息 (最近100个样本) ===").ok();
                    
                    for i in 0..values.len() {
                        if let Some(stats) = multi_adc.get_channel_stats(i) {
                            let mean_voltage = multi_adc.adc_to_voltage(stats.mean as u16, vref);
                            let min_voltage = multi_adc.adc_to_voltage(stats.min, vref);
                            let max_voltage = multi_adc.adc_to_voltage(stats.max, vref);
                            
                            writeln!(serial, 
                                "CH{}: 均值={:.3}V, 最小={:.3}V, 最大={:.3}V, 标准差={:.1}",
                                i, mean_voltage, min_voltage, max_voltage, stats.std_dev
                            ).ok();
                        }
                    }
                    writeln!(serial, "").ok();
                }
            }
            Err(e) => {
                writeln!(serial, "ADC读取错误: {}", e).ok();
            }
        }
        
        // 简单的LED指示
        if sample_count % 10 == 0 {
            // 可以添加LED闪烁指示
        }
    }
}

/// 简单的延时函数
fn delay_ms(ms: u32) {
    for _ in 0..(ms * 1000) {
        cortex_m::asm::nop();
    }
}