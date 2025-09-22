#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::asm;

// 平台特定的HAL导入
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;

use hal::{
    prelude::*,
    timer::{Timer, Event},
    gpio::{Input, Output, PushPull, PullUp, Alternate, AF1},
};

use input_capture::{FrequencyMeter, CaptureEvent, EdgeType, MeasurementResult};
use heapless::{Vec, String};
use nb::block;

/// 频率计配置
#[derive(Debug, Clone)]
struct FrequencyMeterConfig {
    timer_frequency: u32,
    measurement_window_ms: u32,
    input_filter: u8,
    trigger_level: f32,
    auto_range: bool,
    display_update_ms: u32,
}

/// 测量范围
#[derive(Debug, Clone, Copy)]
enum MeasurementRange {
    LowFreq,    // 0.1Hz - 1kHz
    MidFreq,    // 1kHz - 100kHz
    HighFreq,   // 100kHz - 10MHz
    Auto,       // 自动选择
}

/// 显示模式
#[derive(Debug, Clone, Copy)]
enum DisplayMode {
    Frequency,
    Period,
    Both,
    Statistics,
}

/// 频率计状态
#[derive(Debug, Clone)]
struct FrequencyMeterState {
    current_frequency: f32,
    current_period_us: f32,
    measurement_count: u32,
    min_frequency: f32,
    max_frequency: f32,
    avg_frequency: f32,
    frequency_stability: f32,
    measurement_active: bool,
    range: MeasurementRange,
    display_mode: DisplayMode,
}

impl Default for FrequencyMeterConfig {
    fn default() -> Self {
        Self {
            timer_frequency: 84_000_000, // 84MHz
            measurement_window_ms: 1000,  // 1秒测量窗口
            input_filter: 0,              // 无滤波
            trigger_level: 1.65,          // 3.3V/2
            auto_range: true,
            display_update_ms: 100,       // 100ms更新显示
        }
    }
}

impl Default for FrequencyMeterState {
    fn default() -> Self {
        Self {
            current_frequency: 0.0,
            current_period_us: 0.0,
            measurement_count: 0,
            min_frequency: f32::INFINITY,
            max_frequency: 0.0,
            avg_frequency: 0.0,
            frequency_stability: 0.0,
            measurement_active: false,
            range: MeasurementRange::Auto,
            display_mode: DisplayMode::Frequency,
        }
    }
}

/// 输入捕获通道包装器
struct InputCaptureChannel {
    last_capture: u32,
    capture_ready: bool,
    overflow_count: u32,
}

impl InputCaptureChannel {
    fn new() -> Self {
        Self {
            last_capture: 0,
            capture_ready: false,
            overflow_count: 0,
        }
    }

    fn process_capture(&mut self, capture_value: u32) -> Option<u32> {
        if self.capture_ready {
            let period = if capture_value >= self.last_capture {
                capture_value - self.last_capture
            } else {
                // 处理定时器溢出
                (u32::MAX - self.last_capture) + capture_value + 1
            };
            
            self.last_capture = capture_value;
            Some(period)
        } else {
            self.last_capture = capture_value;
            self.capture_ready = true;
            None
        }
    }

    fn reset(&mut self) {
        self.capture_ready = false;
        self.overflow_count = 0;
    }
}

/// 频率统计计算器
struct FrequencyStatistics {
    measurements: Vec<f32, 1000>,
    sum: f64,
    sum_squares: f64,
    count: u32,
}

impl FrequencyStatistics {
    fn new() -> Self {
        Self {
            measurements: Vec::new(),
            sum: 0.0,
            sum_squares: 0.0,
            count: 0,
        }
    }

    fn add_measurement(&mut self, frequency: f32) {
        if frequency > 0.0 {
            self.measurements.push(frequency).ok();
            self.sum += frequency as f64;
            self.sum_squares += (frequency as f64).powi(2);
            self.count += 1;

            // 保持缓冲区大小
            if self.measurements.len() > 500 {
                let removed = self.measurements.remove(0);
                self.sum -= removed as f64;
                self.sum_squares -= (removed as f64).powi(2);
                self.count -= 1;
            }
        }
    }

    fn get_average(&self) -> f32 {
        if self.count > 0 {
            (self.sum / self.count as f64) as f32
        } else {
            0.0
        }
    }

    fn get_standard_deviation(&self) -> f32 {
        if self.count > 1 {
            let mean = self.sum / self.count as f64;
            let variance = (self.sum_squares / self.count as f64) - mean.powi(2);
            (variance.sqrt()) as f32
        } else {
            0.0
        }
    }

    fn get_min_max(&self) -> (f32, f32) {
        if self.measurements.is_empty() {
            return (0.0, 0.0);
        }

        let mut min_val = f32::INFINITY;
        let mut max_val = 0.0;

        for &freq in &self.measurements {
            if freq < min_val {
                min_val = freq;
            }
            if freq > max_val {
                max_val = freq;
            }
        }

        (min_val, max_val)
    }

    fn reset(&mut self) {
        self.measurements.clear();
        self.sum = 0.0;
        self.sum_squares = 0.0;
        self.count = 0;
    }
}

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 配置输入捕获引脚 (TIM2_CH1 - PA15)
    let input_pin = gpioa.pa15.into_alternate_af1();
    
    // 配置状态LED
    let mut status_led = gpioc.pc13.into_push_pull_output();
    let mut range_led = gpioc.pc14.into_push_pull_output();
    let mut activity_led = gpioc.pc15.into_push_pull_output();
    
    // 配置按钮
    let start_button = gpioa.pa0.into_pull_up_input();
    let range_button = gpioa.pa1.into_pull_up_input();
    let display_button = gpioa.pa2.into_pull_up_input();
    let reset_button = gpioa.pa3.into_pull_up_input();

    // 配置定时器
    let config = FrequencyMeterConfig::default();
    let mut timer = Timer::tim2(dp.TIM2, config.timer_frequency.hz(), clocks);
    
    // 初始化频率计
    let mut frequency_meter = FrequencyMeter::new(config.timer_frequency);
    let mut input_capture = InputCaptureChannel::new();
    let mut statistics = FrequencyStatistics::new();
    
    // 系统状态
    let mut state = FrequencyMeterState::default();
    let mut last_display_update = 0u64;
    let mut last_measurement_time = 0u64;
    let mut button_debounce_time = 0u64;
    let mut system_time_ms = 0u64;

    // 演示不同频率范围的测量
    demo_frequency_ranges(&mut frequency_meter);

    loop {
        system_time_ms += 1; // 简化的时间递增
        
        // 检查按钮状态
        if system_time_ms > button_debounce_time + 50 { // 50ms防抖
            if start_button.is_low().unwrap() {
                state.measurement_active = !state.measurement_active;
                if state.measurement_active {
                    frequency_meter.reset();
                    statistics.reset();
                    state.measurement_count = 0;
                }
                button_debounce_time = system_time_ms;
            }
            
            if range_button.is_low().unwrap() {
                state.range = match state.range {
                    MeasurementRange::Auto => MeasurementRange::LowFreq,
                    MeasurementRange::LowFreq => MeasurementRange::MidFreq,
                    MeasurementRange::MidFreq => MeasurementRange::HighFreq,
                    MeasurementRange::HighFreq => MeasurementRange::Auto,
                };
                configure_range(&mut frequency_meter, state.range);
                button_debounce_time = system_time_ms;
            }
            
            if display_button.is_low().unwrap() {
                state.display_mode = match state.display_mode {
                    DisplayMode::Frequency => DisplayMode::Period,
                    DisplayMode::Period => DisplayMode::Both,
                    DisplayMode::Both => DisplayMode::Statistics,
                    DisplayMode::Statistics => DisplayMode::Frequency,
                };
                button_debounce_time = system_time_ms;
            }
            
            if reset_button.is_low().unwrap() {
                frequency_meter.reset();
                statistics.reset();
                state = FrequencyMeterState::default();
                input_capture.reset();
                button_debounce_time = system_time_ms;
            }
        }

        // 模拟输入捕获中断
        if state.measurement_active {
            // 这里应该是实际的输入捕获中断处理
            // 为了演示，我们模拟一个1kHz信号
            if system_time_ms % 1 == 0 { // 每1ms一个捕获事件
                let capture_value = (system_time_ms * config.timer_frequency as u64 / 1000) as u32;
                
                if let Some(period_ticks) = input_capture.process_capture(capture_value) {
                    // 将定时器周期转换为时间
                    let period_us = (period_ticks as f64 * 1_000_000.0 / config.timer_frequency as f64) as u32;
                    frequency_meter.add_capture(capture_value).ok();
                    
                    // 计算频率
                    let frequency = frequency_meter.calculate_frequency();
                    if frequency > 0.0 {
                        state.current_frequency = frequency;
                        state.current_period_us = 1_000_000.0 / frequency;
                        state.measurement_count += 1;
                        
                        // 更新统计
                        statistics.add_measurement(frequency);
                        let (min_freq, max_freq) = statistics.get_min_max();
                        state.min_frequency = min_freq;
                        state.max_frequency = max_freq;
                        state.avg_frequency = statistics.get_average();
                        state.frequency_stability = statistics.get_standard_deviation();
                    }
                }
            }
        }

        // 更新LED状态
        update_led_indicators(
            &mut status_led,
            &mut range_led,
            &mut activity_led,
            &state,
            system_time_ms
        );

        // 更新显示
        if system_time_ms > last_display_update + config.display_update_ms as u64 {
            update_display(&state, &config);
            last_display_update = system_time_ms;
        }

        // 短暂延时
        for _ in 0..1000 {
            asm::nop();
        }
    }
}

/// 配置测量范围
fn configure_range(frequency_meter: &mut FrequencyMeter, range: MeasurementRange) {
    match range {
        MeasurementRange::LowFreq => {
            frequency_meter.set_measurement_window(10000); // 10秒窗口
        },
        MeasurementRange::MidFreq => {
            frequency_meter.set_measurement_window(1000);  // 1秒窗口
        },
        MeasurementRange::HighFreq => {
            frequency_meter.set_measurement_window(100);   // 100ms窗口
        },
        MeasurementRange::Auto => {
            frequency_meter.set_measurement_window(1000);  // 默认1秒窗口
        },
    }
}

/// 更新LED指示器
fn update_led_indicators(
    status_led: &mut hal::gpio::gpioc::PC13<Output<PushPull>>,
    range_led: &mut hal::gpio::gpioc::PC14<Output<PushPull>>,
    activity_led: &mut hal::gpio::gpioc::PC15<Output<PushPull>>,
    state: &FrequencyMeterState,
    system_time_ms: u64,
) {
    // 状态LED：测量活动时常亮
    if state.measurement_active {
        status_led.set_high().ok();
    } else {
        status_led.set_low().ok();
    }

    // 范围LED：根据测量范围闪烁
    let blink_period = match state.range {
        MeasurementRange::LowFreq => 1000,   // 1秒闪烁
        MeasurementRange::MidFreq => 500,    // 0.5秒闪烁
        MeasurementRange::HighFreq => 200,   // 0.2秒闪烁
        MeasurementRange::Auto => 100,       // 0.1秒闪烁
    };

    if (system_time_ms / blink_period) % 2 == 0 {
        range_led.set_high().ok();
    } else {
        range_led.set_low().ok();
    }

    // 活动LED：有频率信号时快速闪烁
    if state.current_frequency > 0.0 {
        if (system_time_ms / 50) % 2 == 0 {
            activity_led.set_high().ok();
        } else {
            activity_led.set_low().ok();
        }
    } else {
        activity_led.set_low().ok();
    }
}

/// 更新显示信息
fn update_display(state: &FrequencyMeterState, config: &FrequencyMeterConfig) {
    // 在实际应用中，这里会输出到LCD显示器或串口
    // 这里只是演示显示逻辑
    
    match state.display_mode {
        DisplayMode::Frequency => {
            // 显示频率：1234.56 Hz
        },
        DisplayMode::Period => {
            // 显示周期：812.34 μs
        },
        DisplayMode::Both => {
            // 同时显示频率和周期
        },
        DisplayMode::Statistics => {
            // 显示统计信息：平均值、最小值、最大值、标准差
        },
    }
}

/// 演示不同频率范围的测量
fn demo_frequency_ranges(frequency_meter: &mut FrequencyMeter) {
    // 测试低频范围 (0.1Hz - 1kHz)
    frequency_meter.set_measurement_window(10000); // 10秒窗口
    
    // 模拟1Hz信号测量
    for i in 0..10 {
        let timestamp = i * 1_000_000; // 1秒间隔
        frequency_meter.add_capture(timestamp).ok();
    }
    
    let low_freq = frequency_meter.calculate_frequency();
    frequency_meter.reset();
    
    // 测试中频范围 (1kHz - 100kHz)
    frequency_meter.set_measurement_window(1000); // 1秒窗口
    
    // 模拟10kHz信号测量
    for i in 0..10000 {
        let timestamp = i * 100; // 100μs间隔
        frequency_meter.add_capture(timestamp).ok();
    }
    
    let mid_freq = frequency_meter.calculate_frequency();
    frequency_meter.reset();
    
    // 测试高频范围 (100kHz - 10MHz)
    frequency_meter.set_measurement_window(100); // 100ms窗口
    
    // 模拟1MHz信号测量
    for i in 0..100000 {
        let timestamp = i * 1; // 1μs间隔
        frequency_meter.add_capture(timestamp).ok();
    }
    
    let high_freq = frequency_meter.calculate_frequency();
    frequency_meter.reset();
}

/// 格式化频率显示
fn format_frequency(frequency: f32) -> String<32> {
    let mut result = String::new();
    
    if frequency >= 1_000_000.0 {
        // MHz
        let mhz = frequency / 1_000_000.0;
        result.push_str("MHz").ok();
    } else if frequency >= 1_000.0 {
        // kHz
        let khz = frequency / 1_000.0;
        result.push_str("kHz").ok();
    } else {
        // Hz
        result.push_str("Hz").ok();
    }
    
    result
}

/// 格式化周期显示
fn format_period(period_us: f32) -> String<32> {
    let mut result = String::new();
    
    if period_us >= 1_000_000.0 {
        // 秒
        let seconds = period_us / 1_000_000.0;
        result.push_str("s").ok();
    } else if period_us >= 1_000.0 {
        // 毫秒
        let ms = period_us / 1_000.0;
        result.push_str("ms").ok();
    } else {
        // 微秒
        result.push_str("μs").ok();
    }
    
    result
}