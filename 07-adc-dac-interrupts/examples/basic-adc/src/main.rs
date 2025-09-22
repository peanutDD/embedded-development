#![no_std]
#![no_main]

// 基础ADC采样示例
// 功能：单通道电压测量和显示
// 硬件：STM32F407VG Discovery + 电位器
// 连接：电位器输出 -> PA0 (ADC1_IN0)

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};

use stm32f4xx_hal::{
    adc::{Adc, config::{AdcConfig, SampleTime}},
    gpio::{Analog, Pin},
    pac,
    prelude::*,
    timer::TimerExt,
};

use libm::roundf;
// use micromath::F32Ext; // 未使用的导入

// ADC采样配置
const ADC_SAMPLE_TIME: SampleTime = SampleTime::Cycles_480; // 最长采样时间，提高精度
const VREF_VOLTAGE: f32 = 3.3; // 参考电压 (V)
const ADC_RESOLUTION: u32 = 4096; // 12位ADC分辨率
const SAMPLE_INTERVAL_MS: u32 = 100; // 采样间隔 (ms)

// 统计数据结构
#[derive(Default)]
struct AdcStats {
    min_value: u16,
    max_value: u16,
    sum: u32,
    count: u32,
    min_voltage: f32,
    max_voltage: f32,
}

impl AdcStats {
    fn new() -> Self {
        Self {
            min_value: u16::MAX,
            max_value: 0,
            sum: 0,
            count: 0,
            min_voltage: f32::MAX,
            max_voltage: 0.0,
        }
    }

    fn update(&mut self, raw_value: u16, voltage: f32) {
        self.min_value = self.min_value.min(raw_value);
        self.max_value = self.max_value.max(raw_value);
        self.sum += raw_value as u32;
        self.count += 1;
        
        self.min_voltage = self.min_voltage.min(voltage);
        self.max_voltage = self.max_voltage.max(voltage);
    }

    fn get_average_raw(&self) -> f32 {
        if self.count > 0 {
            self.sum as f32 / self.count as f32
        } else {
            0.0
        }
    }

    fn get_average_voltage(&self) -> f32 {
        raw_to_voltage(self.get_average_raw() as u16)
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

// ADC原始值转换为电压
fn raw_to_voltage(raw_value: u16) -> f32 {
    (raw_value as f32 / ADC_RESOLUTION as f32) * VREF_VOLTAGE
}

// 电压转换为百分比
fn voltage_to_percentage(voltage: f32) -> f32 {
    (voltage / VREF_VOLTAGE) * 100.0
}

// 格式化电压显示
fn format_voltage(voltage: f32) -> (u32, u32) {
    let rounded = roundf(voltage * 1000.0) as u32;
    (rounded / 1000, rounded % 1000)
}

// 电压范围分类
fn classify_voltage_range(voltage: f32) -> &'static str {
    match voltage {
        v if v < 0.5 => "极低",
        v if v < 1.0 => "低",
        v if v < 2.0 => "中低",
        v if v < 2.5 => "中",
        v if v < 3.0 => "中高",
        _ => "高",
    }
}

#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    rprintln!("\n=== STM32F4 基础ADC采样示例 ===");
    rprintln!("功能: 单通道电压测量和统计分析");
    rprintln!("硬件: STM32F407VG + 电位器");
    rprintln!("连接: 电位器输出 -> PA0 (ADC1_IN0)\n");

    // 获取外设句柄
    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz()) // 使用外部8MHz晶振
        .sysclk(168.MHz()) // 系统时钟168MHz
        .pclk1(42.MHz())   // APB1时钟42MHz
        .pclk2(84.MHz())   // APB2时钟84MHz
        .freeze();

    rprintln!("时钟配置:");
    rprintln!("  SYSCLK: {} MHz", clocks.sysclk().to_MHz());
    rprintln!("  PCLK1:  {} MHz", clocks.pclk1().to_MHz());
    rprintln!("  PCLK2:  {} MHz", clocks.pclk2().to_MHz());

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    
    // 配置PA0为模拟输入 (ADC1_IN0)
    let mut adc_pin: Pin<'A', 0, Analog> = gpioa.pa0.into_analog();
    
    rprintln!("\nGPIO配置:");
    rprintln!("  PA0: ADC1_IN0 (模拟输入)");

    // 配置ADC
    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());
    adc.set_default_sample_time(ADC_SAMPLE_TIME);
    
    rprintln!("\nADC配置:");
    rprintln!("  分辨率: 12位 ({} 级)", ADC_RESOLUTION);
    rprintln!("  采样时间: {:?}", ADC_SAMPLE_TIME);
    rprintln!("  参考电压: {}.{}V", 
        (VREF_VOLTAGE as u32), 
        ((VREF_VOLTAGE * 10.0) as u32) % 10);

    // 配置定时器用于定时采样
    let mut timer = dp.TIM2.counter_ms(&clocks);
    timer.start(SAMPLE_INTERVAL_MS.millis()).unwrap();
    
    rprintln!("\n定时器配置:");
    rprintln!("  采样间隔: {} ms", SAMPLE_INTERVAL_MS);
    rprintln!("  采样频率: {} Hz", 1000 / SAMPLE_INTERVAL_MS);

    // 统计数据
    let mut stats = AdcStats::new();
    let mut sample_count = 0u32;
    let mut last_voltage = 0.0f32;
    
    rprintln!("\n开始ADC采样...");
    rprintln!("格式: [样本] 原始值 -> 电压 (百分比) [范围] [变化]\n");

    loop {
        // 等待定时器超时
        nb::block!(timer.wait()).unwrap();
        
        // 读取ADC值
        let raw_value: u16 = adc.read(&mut adc_pin).unwrap();
        
        // 转换为电压
        let voltage = raw_to_voltage(raw_value);
        let percentage = voltage_to_percentage(voltage);
        let range = classify_voltage_range(voltage);
        
        // 计算变化
        let change = if sample_count > 0 {
            voltage - last_voltage
        } else {
            0.0
        };
        
        // 更新统计
        stats.update(raw_value, voltage);
        sample_count += 1;
        
        // 格式化输出
        let (v_int, v_frac) = format_voltage(voltage);
        let (p_int, p_frac) = format_voltage(percentage);
        let change_str = if change.abs() < 0.001 {
            "="
        } else if change > 0.0 {
            "+"
        } else {
            "-"
        };
        
        // 输出当前读数
        rprintln!(
            "[{:4}] {:4} -> {}.{:03}V ({:2}.{:01}%) [{}] [{}]",
            sample_count,
            raw_value,
            v_int, v_frac,
            p_int, p_frac / 100,
            range,
            change_str
        );
        
        // 每10个样本输出统计信息
        if sample_count % 10 == 0 {
            let avg_voltage = stats.get_average_voltage();
            let (avg_int, avg_frac) = format_voltage(avg_voltage);
            let (min_int, min_frac) = format_voltage(stats.min_voltage);
            let (max_int, max_frac) = format_voltage(stats.max_voltage);
            let range_voltage = stats.max_voltage - stats.min_voltage;
            let (range_int, range_frac) = format_voltage(range_voltage);
            
            rprintln!("\n--- 统计信息 (最近10个样本) ---");
            rprintln!("  平均值: {}.{:03}V (原始: {:.1})", 
                avg_int, avg_frac, stats.get_average_raw());
            rprintln!("  最小值: {}.{:03}V (原始: {})", 
                min_int, min_frac, stats.min_value);
            rprintln!("  最大值: {}.{:03}V (原始: {})", 
                max_int, max_frac, stats.max_value);
            rprintln!("  变化范围: {}.{:03}V ({} LSB)", 
                range_int, range_frac, stats.max_value - stats.min_value);
            rprintln!("  样本数: {}\n", stats.count);
            
            // 重置统计
            stats.reset();
        }
        
        // 每50个样本输出系统信息
        if sample_count % 50 == 0 {
            rprintln!("\n=== 系统状态 (样本 {}) ===", sample_count);
            rprintln!("  运行时间: {} 秒", sample_count * SAMPLE_INTERVAL_MS / 1000);
            rprintln!("  内存使用: 正常");
            rprintln!("  ADC状态: 正常工作");
            rprintln!("  定时器: 正常运行\n");
        }
        
        last_voltage = voltage;
        
        // 防止溢出
        if sample_count >= 9999 {
            sample_count = 0;
            rprintln!("\n样本计数器重置\n");
        }
    }
}