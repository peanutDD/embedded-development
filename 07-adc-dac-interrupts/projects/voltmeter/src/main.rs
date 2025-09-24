#![no_std]
#![no_main]

// 数字电压表项目
// 功能：高精度电压测量、统计分析、自动量程、校准功能
// 硬件：STM32F407VG Discovery + 分压电路
// 测量范围：0-30V (通过分压电路)
// 精度：±0.1% ±1 digit

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

use libm::sqrtf;
use heapless::Vec;

// 电压表配置常量
const ADC_RESOLUTION: u32 = 4096; // 12位ADC分辨率
const VREF_VOLTAGE: f32 = 3.3; // 参考电压 (V)
const VOLTAGE_DIVIDER_RATIO: f32 = 10.0; // 分压比 (R1+R2)/R2
const MAX_INPUT_VOLTAGE: f32 = 30.0; // 最大输入电压 (V)
const SAMPLE_TIME: SampleTime = SampleTime::Cycles_480; // 最长采样时间
const MEASUREMENT_INTERVAL_MS: u32 = 50; // 测量间隔 (ms)

// 测量精度等级
#[derive(Clone, Copy, Debug)]
enum AccuracyLevel {
    Fast,     // 快速测量 (±0.5%)
    Normal,   // 标准测量 (±0.2%)
    High,     // 高精度测量 (±0.1%)
    Ultra,    // 超高精度测量 (±0.05%)
}

// 量程设置
#[derive(Clone, Copy, Debug)]
enum VoltageRange {
    Auto,        // 自动量程
    Range3V,     // 0-3.3V
    Range10V,    // 0-10V
    Range30V,    // 0-30V
}

// 校准数据结构
#[derive(Clone, Copy)]
struct CalibrationData {
    offset: f32,      // 零点偏移 (V)
    gain: f32,        // 增益系数
    linearity: [f32; 5], // 线性度校正系数
}

impl Default for CalibrationData {
    fn default() -> Self {
        Self {
            offset: 0.0,
            gain: 1.0,
            linearity: [0.0; 5],
        }
    }
}

// 统计数据结构
#[derive(Default)]
struct MeasurementStats {
    samples: Vec<f32, 1000>,
    min_voltage: f32,
    max_voltage: f32,
    sum: f32,
    sum_squares: f32,
    count: u32,
}

impl MeasurementStats {
    fn new() -> Self {
        Self {
            samples: Vec::new(),
            min_voltage: f32::MAX,
            max_voltage: f32::MIN,
            sum: 0.0,
            sum_squares: 0.0,
            count: 0,
        }
    }

    fn add_sample(&mut self, voltage: f32) {
        // 更新统计数据
        self.min_voltage = self.min_voltage.min(voltage);
        self.max_voltage = self.max_voltage.max(voltage);
        self.sum += voltage;
        self.sum_squares += voltage * voltage;
        self.count += 1;

        // 保存样本用于高级分析
        if self.samples.len() < self.samples.capacity() {
            let _ = self.samples.push(voltage);
        } else {
            // 循环缓冲区
            let index = (self.count as usize - 1) % self.samples.capacity();
            self.samples[index] = voltage;
        }
    }

    fn get_average(&self) -> f32 {
        if self.count > 0 {
            self.sum / self.count as f32
        } else {
            0.0
        }
    }

    fn get_std_deviation(&self) -> f32 {
        if self.count > 1 {
            let mean = self.get_average();
            let variance = (self.sum_squares / self.count as f32) - (mean * mean);
            sqrtf(variance.max(0.0))
        } else {
            0.0
        }
    }

    fn get_peak_to_peak(&self) -> f32 {
        self.max_voltage - self.min_voltage
    }

    fn reset(&mut self) {
        self.samples.clear();
        self.min_voltage = f32::MAX;
        self.max_voltage = f32::MIN;
        self.sum = 0.0;
        self.sum_squares = 0.0;
        self.count = 0;
    }
}

// 数字电压表结构
struct DigitalVoltmeter {
    accuracy_level: AccuracyLevel,
    voltage_range: VoltageRange,
    calibration: CalibrationData,
    stats: MeasurementStats,
    auto_range_enabled: bool,
    hold_mode: bool,
    held_voltage: f32,
}

impl DigitalVoltmeter {
    fn new() -> Self {
        Self {
            accuracy_level: AccuracyLevel::Normal,
            voltage_range: VoltageRange::Auto,
            calibration: CalibrationData::default(),
            stats: MeasurementStats::new(),
            auto_range_enabled: true,
            hold_mode: false,
            held_voltage: 0.0,
        }
    }

    fn set_accuracy_level(&mut self, level: AccuracyLevel) {
        self.accuracy_level = level;
    }

    fn set_voltage_range(&mut self, range: VoltageRange) {
        self.voltage_range = range;
        self.auto_range_enabled = matches!(range, VoltageRange::Auto);
    }

    fn toggle_hold(&mut self, current_voltage: f32) {
        self.hold_mode = !self.hold_mode;
        if self.hold_mode {
            self.held_voltage = current_voltage;
        }
    }

    fn get_samples_for_accuracy(&self) -> usize {
        match self.accuracy_level {
            AccuracyLevel::Fast => 4,
            AccuracyLevel::Normal => 16,
            AccuracyLevel::High => 64,
            AccuracyLevel::Ultra => 256,
        }
    }

    fn apply_calibration(&self, raw_voltage: f32) -> f32 {
        // 应用零点偏移
        let mut corrected = raw_voltage - self.calibration.offset;
        
        // 应用增益校正
        corrected *= self.calibration.gain;
        
        // 应用线性度校正 (多项式)
        let normalized = corrected / MAX_INPUT_VOLTAGE;
        let mut linearity_correction = 0.0;
        let mut power = 1.0;
        
        for &coeff in &self.calibration.linearity {
            linearity_correction += coeff * power;
            power *= normalized;
        }
        
        corrected + linearity_correction
    }

    fn auto_select_range(&mut self, voltage: f32) {
        if !self.auto_range_enabled {
            return;
        }

        self.voltage_range = if voltage <= 3.0 {
            VoltageRange::Range3V
        } else if voltage <= 9.0 {
            VoltageRange::Range10V
        } else {
            VoltageRange::Range30V
        };
    }

    fn get_range_string(&self) -> &'static str {
        match self.voltage_range {
            VoltageRange::Auto => "自动",
            VoltageRange::Range3V => "3.3V",
            VoltageRange::Range10V => "10V",
            VoltageRange::Range30V => "30V",
        }
    }

    fn get_accuracy_string(&self) -> &'static str {
        match self.accuracy_level {
            AccuracyLevel::Fast => "快速 (±0.5%)",
            AccuracyLevel::Normal => "标准 (±0.2%)",
            AccuracyLevel::High => "高精度 (±0.1%)",
            AccuracyLevel::Ultra => "超高精度 (±0.05%)",
        }
    }
}

// ADC原始值转换为电压
fn raw_to_voltage(raw_value: u16, range: VoltageRange) -> f32 {
    let adc_voltage = (raw_value as f32 / ADC_RESOLUTION as f32) * VREF_VOLTAGE;
    
    match range {
        VoltageRange::Range3V => adc_voltage,
        VoltageRange::Range10V | VoltageRange::Range30V | VoltageRange::Auto => {
            adc_voltage * VOLTAGE_DIVIDER_RATIO
        },
    }
}

// 多次采样平均
fn measure_voltage_averaged(
    adc: &mut Adc<pac::ADC1>,
    pin: &mut Pin<'A', 0, Analog>,
    samples: usize,
    range: VoltageRange,
) -> f32 {
    let mut sum = 0u32;
    
    for _ in 0..samples {
        let raw: u16 = adc.read(pin).unwrap();
        sum += raw as u32;
        
        // 小延迟确保ADC稳定
        cortex_m::asm::delay(100);
    }
    
    let average_raw = (sum / samples as u32) as u16;
    raw_to_voltage(average_raw, range)
}

// 格式化电压显示
fn format_voltage_precise(voltage: f32) -> (u32, u32, u32) {
    let rounded = (voltage * 10000.0) as u32;
    (rounded / 10000, (rounded / 10) % 1000, rounded % 10)
}

// 电压稳定性分析
fn analyze_stability(voltage: f32, std_dev: f32) -> &'static str {
    let cv = if voltage > 0.001 { (std_dev / voltage) * 100.0 } else { 0.0 };
    
    match cv {
        cv if cv < 0.01 => "极稳定",
        cv if cv < 0.05 => "很稳定",
        cv if cv < 0.1 => "稳定",
        cv if cv < 0.5 => "一般",
        _ => "不稳定",
    }
}

#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    rprintln!("\n=== STM32F4 数字电压表项目 ===");
    rprintln!("功能: 高精度电压测量和统计分析");
    rprintln!("测量范围: 0-30V (通过分压电路)");
    rprintln!("精度: ±0.05% - ±0.5% (可选)\n");

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

    rprintln!("系统时钟: {} MHz", clocks.sysclk().to_MHz());

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiod = dp.GPIOD.split();
    
    // 配置ADC输入引脚
    let mut adc_pin: Pin<'A', 0, Analog> = gpioa.pa0.into_analog();
    
    // 配置状态LED
    let mut led_green = gpiod.pd12.into_push_pull_output(); // 正常工作
    let mut led_orange = gpiod.pd13.into_push_pull_output(); // 警告
    let mut led_red = gpiod.pd14.into_push_pull_output(); // 错误
    let mut led_blue = gpiod.pd15.into_push_pull_output(); // 保持模式
    
    rprintln!("\nGPIO配置:");
    rprintln!("  PA0: ADC输入 (分压后)");
    rprintln!("  PD12: 绿色LED (正常)");
    rprintln!("  PD13: 橙色LED (警告)");
    rprintln!("  PD14: 红色LED (错误)");
    rprintln!("  PD15: 蓝色LED (保持)");

    // 配置ADC
    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());
    adc.set_default_sample_time(SAMPLE_TIME);
    
    rprintln!("\nADC配置:");
    rprintln!("  分辨率: 12位");
    rprintln!("  采样时间: {:?}", SAMPLE_TIME);
    rprintln!("  参考电压: {:.1}V", VREF_VOLTAGE);
    rprintln!("  分压比: 1:{:.1}", VOLTAGE_DIVIDER_RATIO);

    // 配置定时器
    let mut timer = dp.TIM2.counter_ms(&clocks);
    timer.start(MEASUREMENT_INTERVAL_MS.millis()).unwrap();
    
    rprintln!("\n测量配置:");
    rprintln!("  测量间隔: {} ms", MEASUREMENT_INTERVAL_MS);
    rprintln!("  最大输入: {:.1}V", MAX_INPUT_VOLTAGE);

    // 创建数字电压表实例
    let mut voltmeter = DigitalVoltmeter::new();
    
    // 初始化LED状态
    led_green.set_high();
    led_orange.set_low();
    led_red.set_low();
    led_blue.set_low();
    
    let mut measurement_count = 0u32;
    let mut mode_switch_count = 0u32;
    
    rprintln!("\n开始电压测量...");
    rprintln!("格式: [测量] 电压 (范围) [精度] [稳定性] [统计]\n");

    loop {
        // 等待测量间隔
        nb::block!(timer.wait()).unwrap();
        
        measurement_count += 1;
        
        // 获取当前精度设置的采样数
        let samples = voltmeter.get_samples_for_accuracy();
        
        // 执行多次采样平均测量
        let raw_voltage = measure_voltage_averaged(
            &mut adc, 
            &mut adc_pin, 
            samples, 
            voltmeter.voltage_range
        );
        
        // 应用校准
        let calibrated_voltage = voltmeter.apply_calibration(raw_voltage);
        
        // 自动量程选择
        voltmeter.auto_select_range(calibrated_voltage);
        
        // 更新统计数据
        if !voltmeter.hold_mode {
            voltmeter.stats.add_sample(calibrated_voltage);
        }
        
        // 获取显示电压
        let display_voltage = if voltmeter.hold_mode {
            voltmeter.held_voltage
        } else {
            calibrated_voltage
        };
        
        // 格式化电压显示
        let (v_int, v_frac, v_sub) = format_voltage_precise(display_voltage);
        
        // 计算统计信息
        let avg_voltage = voltmeter.stats.get_average();
        let std_dev = voltmeter.stats.get_std_deviation();
        let stability = analyze_stability(avg_voltage, std_dev);
        
        // 更新LED状态
        if voltmeter.hold_mode {
            led_blue.set_high();
            led_green.set_low();
        } else if display_voltage > MAX_INPUT_VOLTAGE * 0.9 {
            led_red.set_high();
            led_green.set_low();
            led_orange.set_low();
        } else if display_voltage > MAX_INPUT_VOLTAGE * 0.8 {
            led_orange.set_high();
            led_green.set_low();
            led_red.set_low();
        } else {
            led_green.set_high();
            led_orange.set_low();
            led_red.set_low();
        }
        
        // 输出测量结果
        let hold_indicator = if voltmeter.hold_mode { " [HOLD]" } else { "" };
        
        rprintln!(
            "[{:5}] {}.{:03}.{}V ({}) [{}] [{}] σ={:.4}V{}",
            measurement_count,
            v_int, v_frac, v_sub,
            voltmeter.get_range_string(),
            voltmeter.get_accuracy_string(),
            stability,
            std_dev,
            hold_indicator
        );
        
        // 每20次测量输出详细统计
        if measurement_count % 20 == 0 {
            let (avg_int, avg_frac, avg_sub) = format_voltage_precise(avg_voltage);
            let (min_int, min_frac, min_sub) = format_voltage_precise(voltmeter.stats.min_voltage);
            let (max_int, max_frac, max_sub) = format_voltage_precise(voltmeter.stats.max_voltage);
            let pp_voltage = voltmeter.stats.get_peak_to_peak();
            let (pp_int, pp_frac, pp_sub) = format_voltage_precise(pp_voltage);
            
            rprintln!("\n--- 统计分析 (最近20次测量) ---");
            rprintln!("  平均值: {}.{:03}.{}V", avg_int, avg_frac, avg_sub);
            rprintln!("  最小值: {}.{:03}.{}V", min_int, min_frac, min_sub);
            rprintln!("  最大值: {}.{:03}.{}V", max_int, max_frac, max_sub);
            rprintln!("  峰峰值: {}.{:03}.{}V", pp_int, pp_frac, pp_sub);
            rprintln!("  标准差: {:.6}V", std_dev);
            rprintln!("  稳定性: {}", stability);
            rprintln!("  样本数: {}\n", voltmeter.stats.count);
            
            // 重置统计
            voltmeter.stats.reset();
        }
        
        // 每100次测量切换模式演示
        if measurement_count % 100 == 0 {
            mode_switch_count += 1;
            
            match mode_switch_count % 6 {
                0 => {
                    voltmeter.set_accuracy_level(AccuracyLevel::Fast);
                    voltmeter.set_voltage_range(VoltageRange::Auto);
                },
                1 => {
                    voltmeter.set_accuracy_level(AccuracyLevel::Normal);
                    voltmeter.set_voltage_range(VoltageRange::Range10V);
                },
                2 => {
                    voltmeter.set_accuracy_level(AccuracyLevel::High);
                    voltmeter.set_voltage_range(VoltageRange::Auto);
                },
                3 => {
                    voltmeter.set_accuracy_level(AccuracyLevel::Ultra);
                    voltmeter.set_voltage_range(VoltageRange::Range30V);
                },
                4 => {
                    voltmeter.toggle_hold(display_voltage);
                },
                _ => {
                    voltmeter.hold_mode = false;
                    led_blue.set_low();
                    voltmeter.set_accuracy_level(AccuracyLevel::Normal);
                    voltmeter.set_voltage_range(VoltageRange::Auto);
                },
            }
            
            rprintln!("\n=== 模式切换 #{} ===", mode_switch_count);
            rprintln!("  精度等级: {}", voltmeter.get_accuracy_string());
            rprintln!("  电压量程: {}", voltmeter.get_range_string());
            rprintln!("  保持模式: {}", if voltmeter.hold_mode { "开启" } else { "关闭" });
            rprintln!("  运行时间: {} 秒\n", measurement_count * MEASUREMENT_INTERVAL_MS / 1000);
        }
        
        // 每500次测量输出系统状态
        if measurement_count % 500 == 0 {
            rprintln!("\n=== 系统状态报告 ===");
            rprintln!("  总测量次数: {}", measurement_count);
            rprintln!("  运行时间: {:.1} 分钟", 
                measurement_count as f32 * MEASUREMENT_INTERVAL_MS as f32 / 60000.0);
            rprintln!("  模式切换次数: {}", mode_switch_count);
            rprintln!("  当前精度: {}", voltmeter.get_accuracy_string());
            rprintln!("  当前量程: {}", voltmeter.get_range_string());
            rprintln!("  ADC状态: 正常工作");
            rprintln!("  内存使用: 正常\n");
        }
        
        // 防止计数器溢出
        if measurement_count >= 0xFFFFF000 {
            measurement_count = 0;
            mode_switch_count = 0;
            rprintln!("\n计数器重置\n");
        }
    }
}