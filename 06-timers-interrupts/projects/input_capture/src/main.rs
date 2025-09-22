#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::asm;

// 平台特定的HAL导入
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;
#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal;
#[cfg(feature = "nrf52840")]
use nrf52840_hal as hal;

use hal::{
    prelude::*,
    timer::{Timer, Event, CountDownTimer},
    gpio::{Input, Output, PushPull, PullUp, Alternate, AF1},
    interrupt,
};

use input_capture::{
    FrequencyMeter, PulseCounter, RotaryEncoder, UltrasonicSensor,
    CaptureEvent, EdgeType, CountMode, RotationDirection,
    MeasurementResult, CaptureStatistics
};

use heapless::{Vec, String};
use nb::block;
use cortex_m::interrupt::{Mutex, free};
use core::cell::RefCell;

/// 系统状态
#[derive(Debug, Clone, Copy)]
enum SystemMode {
    FrequencyMeter,
    PulseCounter,
    RotaryEncoder,
    UltrasonicSensor,
    MultiChannel,
}

/// 测量状态
#[derive(Debug, Clone)]
struct MeasurementState {
    frequency_hz: f32,
    pulse_count: u64,
    encoder_position: i32,
    encoder_velocity: f32,
    distance_cm: f32,
    measurement_active: bool,
}

/// 显示信息
#[derive(Debug, Clone)]
struct DisplayInfo {
    mode: SystemMode,
    measurement: MeasurementState,
    status: String<64>,
    error_count: u32,
}

/// 全局共享数据
static SHARED_DATA: Mutex<RefCell<Option<SharedData>>> = Mutex::new(RefCell::new(None));

struct SharedData {
    frequency_meter: FrequencyMeter,
    pulse_counter: PulseCounter,
    rotary_encoder: RotaryEncoder,
    ultrasonic_sensor: UltrasonicSensor,
    system_time_us: u64,
    capture_events: Vec<CaptureEvent, 1000>,
}

/// 定时器包装器
struct TimerWrapper {
    timer: CountDownTimer<hal::pac::TIM2>,
    frequency: u32,
}

/// 输入捕获包装器
struct InputCaptureWrapper {
    channel1_pin: hal::gpio::gpioa::PA8<Alternate<AF1>>,
    channel2_pin: hal::gpio::gpioa::PA9<Alternate<AF1>>,
    last_capture1: u32,
    last_capture2: u32,
}

impl Default for MeasurementState {
    fn default() -> Self {
        Self {
            frequency_hz: 0.0,
            pulse_count: 0,
            encoder_position: 0,
            encoder_velocity: 0.0,
            distance_cm: 0.0,
            measurement_active: false,
        }
    }
}

impl TimerWrapper {
    fn new(timer: CountDownTimer<hal::pac::TIM2>, frequency: u32) -> Self {
        Self { timer, frequency }
    }

    fn get_counter(&self) -> u32 {
        // 获取定时器计数值的简化实现
        0 // 实际实现需要访问定时器寄存器
    }

    fn reset_counter(&mut self) {
        // 重置定时器计数器
    }
}

impl InputCaptureWrapper {
    fn new(
        channel1_pin: hal::gpio::gpioa::PA8<Alternate<AF1>>,
        channel2_pin: hal::gpio::gpioa::PA9<Alternate<AF1>>,
    ) -> Self {
        Self {
            channel1_pin,
            channel2_pin,
            last_capture1: 0,
            last_capture2: 0,
        }
    }

    fn read_capture_channel1(&mut self) -> Option<u32> {
        // 读取通道1捕获值的简化实现
        None
    }

    fn read_capture_channel2(&mut self) -> Option<u32> {
        // 读取通道2捕获值的简化实现
        None
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

    // 配置输入捕获引脚
    let capture_pin1 = gpioa.pa8.into_alternate_af1();
    let capture_pin2 = gpioa.pa9.into_alternate_af1();
    
    // 配置编码器引脚
    let encoder_a = gpiob.pb6.into_pull_up_input();
    let encoder_b = gpiob.pb7.into_pull_up_input();
    
    // 配置超声波传感器引脚
    let mut ultrasonic_trigger = gpioc.pc8.into_push_pull_output();
    let ultrasonic_echo = gpioc.pc9.into_pull_up_input();
    
    // 配置状态LED
    let mut status_led = gpioc.pc13.into_push_pull_output();
    
    // 配置按钮
    let mode_button = gpioa.pa0.into_pull_up_input();
    let start_button = gpioa.pa1.into_pull_up_input();
    let reset_button = gpioa.pa2.into_pull_up_input();

    // 配置定时器
    let timer_frequency = 1_000_000; // 1MHz
    let mut timer = Timer::tim2(dp.TIM2, timer_frequency.hz(), clocks);
    let timer_wrapper = TimerWrapper::new(timer, timer_frequency);
    
    // 配置输入捕获
    let mut input_capture = InputCaptureWrapper::new(capture_pin1, capture_pin2);

    // 初始化测量设备
    let frequency_meter = FrequencyMeter::new(timer_frequency);
    let pulse_counter = PulseCounter::new(timer_frequency);
    let rotary_encoder = RotaryEncoder::new(400, timer_frequency); // 400步/转编码器
    let ultrasonic_sensor = UltrasonicSensor::new(timer_frequency);

    // 初始化共享数据
    free(|cs| {
        SHARED_DATA.borrow(cs).replace(Some(SharedData {
            frequency_meter,
            pulse_counter,
            rotary_encoder,
            ultrasonic_sensor,
            system_time_us: 0,
            capture_events: Vec::new(),
        }));
    });

    // 系统状态
    let mut current_mode = SystemMode::FrequencyMeter;
    let mut display_info = DisplayInfo {
        mode: current_mode,
        measurement: MeasurementState::default(),
        status: String::new(),
        error_count: 0,
    };

    let mut button_debounce_time = 0u64;
    let mut last_encoder_a = false;
    let mut last_encoder_b = false;
    let mut ultrasonic_measurement_start = 0u64;
    let mut ultrasonic_trigger_sent = false;

    // 演示不同的测量功能
    demo_frequency_measurement();
    demo_pulse_counting();
    demo_encoder_reading();
    demo_ultrasonic_measurement();

    loop {
        let current_time = get_system_time_us();
        
        // 检查按钮状态
        if current_time > button_debounce_time + 50000 { // 50ms防抖
            if mode_button.is_low().unwrap() {
                current_mode = match current_mode {
                    SystemMode::FrequencyMeter => SystemMode::PulseCounter,
                    SystemMode::PulseCounter => SystemMode::RotaryEncoder,
                    SystemMode::RotaryEncoder => SystemMode::UltrasonicSensor,
                    SystemMode::UltrasonicSensor => SystemMode::MultiChannel,
                    SystemMode::MultiChannel => SystemMode::FrequencyMeter,
                };
                display_info.mode = current_mode;
                button_debounce_time = current_time;
                
                display_info.status.clear();
                display_info.status.push_str("Mode changed").ok();
            }
            
            if start_button.is_low().unwrap() {
                display_info.measurement.measurement_active = !display_info.measurement.measurement_active;
                button_debounce_time = current_time;
            }
            
            if reset_button.is_low().unwrap() {
                reset_measurements();
                display_info.measurement = MeasurementState::default();
                button_debounce_time = current_time;
                
                display_info.status.clear();
                display_info.status.push_str("Reset complete").ok();
            }
        }

        // 根据当前模式执行相应的测量
        match current_mode {
            SystemMode::FrequencyMeter => {
                status_led.set_high().ok();
                
                // 读取输入捕获
                if let Some(capture_value) = input_capture.read_capture_channel1() {
                    process_frequency_capture(capture_value, current_time);
                }
                
                // 更新频率测量结果
                if let Some(frequency) = get_frequency_measurement() {
                    display_info.measurement.frequency_hz = frequency;
                    display_info.status.clear();
                    display_info.status.push_str("Freq measurement").ok();
                }
            },
            
            SystemMode::PulseCounter => {
                status_led.set_high().ok();
                
                // 检测脉冲边沿
                let current_level = ultrasonic_echo.is_high().unwrap();
                static mut LAST_LEVEL: bool = false;
                
                unsafe {
                    if current_level != LAST_LEVEL && current_level {
                        // 检测到上升沿
                        process_pulse_event(current_time);
                    }
                    LAST_LEVEL = current_level;
                }
                
                // 更新脉冲计数结果
                if let Some(count) = get_pulse_count() {
                    display_info.measurement.pulse_count = count;
                    display_info.status.clear();
                    display_info.status.push_str("Pulse counting").ok();
                }
            },
            
            SystemMode::RotaryEncoder => {
                status_led.set_high().ok();
                
                // 读取编码器状态
                let encoder_a = encoder_a.is_high().unwrap();
                let encoder_b = encoder_b.is_high().unwrap();
                
                if encoder_a != last_encoder_a || encoder_b != last_encoder_b {
                    process_encoder_update(encoder_a, encoder_b, current_time);
                    last_encoder_a = encoder_a;
                    last_encoder_b = encoder_b;
                }
                
                // 更新编码器结果
                if let Some((position, velocity)) = get_encoder_measurement() {
                    display_info.measurement.encoder_position = position;
                    display_info.measurement.encoder_velocity = velocity;
                    display_info.status.clear();
                    display_info.status.push_str("Encoder reading").ok();
                }
            },
            
            SystemMode::UltrasonicSensor => {
                // LED闪烁表示超声波测量
                if (current_time / 250000) % 2 == 0 {
                    status_led.set_high().ok();
                } else {
                    status_led.set_low().ok();
                }
                
                // 超声波测量逻辑
                if display_info.measurement.measurement_active {
                    if !ultrasonic_trigger_sent {
                        // 发送触发脉冲
                        ultrasonic_trigger.set_high().ok();
                        ultrasonic_measurement_start = current_time;
                        ultrasonic_trigger_sent = true;
                    } else if current_time > ultrasonic_measurement_start + 10 {
                        // 10μs后关闭触发
                        ultrasonic_trigger.set_low().ok();
                        
                        // 检测回声
                        let echo_state = ultrasonic_echo.is_high().unwrap();
                        process_ultrasonic_echo(echo_state, current_time);
                        
                        // 1秒后重新测量
                        if current_time > ultrasonic_measurement_start + 1_000_000 {
                            ultrasonic_trigger_sent = false;
                        }
                    }
                }
                
                // 更新距离测量结果
                if let Some(distance) = get_distance_measurement() {
                    display_info.measurement.distance_cm = distance;
                    display_info.status.clear();
                    display_info.status.push_str("Distance measurement").ok();
                }
            },
            
            SystemMode::MultiChannel => {
                // 快速闪烁表示多通道模式
                if (current_time / 100000) % 2 == 0 {
                    status_led.set_high().ok();
                } else {
                    status_led.set_low().ok();
                }
                
                // 同时处理多个输入
                if let Some(capture1) = input_capture.read_capture_channel1() {
                    process_frequency_capture(capture1, current_time);
                }
                
                if let Some(capture2) = input_capture.read_capture_channel2() {
                    process_pulse_event(current_time);
                }
                
                // 编码器更新
                let encoder_a = encoder_a.is_high().unwrap();
                let encoder_b = encoder_b.is_high().unwrap();
                if encoder_a != last_encoder_a || encoder_b != last_encoder_b {
                    process_encoder_update(encoder_a, encoder_b, current_time);
                    last_encoder_a = encoder_a;
                    last_encoder_b = encoder_b;
                }
                
                // 更新所有测量结果
                update_all_measurements(&mut display_info);
                display_info.status.clear();
                display_info.status.push_str("Multi-channel mode").ok();
            },
        }

        // 短暂延时
        for _ in 0..1000 {
            asm::nop();
        }
    }
}

/// 获取系统时间（微秒）
fn get_system_time_us() -> u64 {
    free(|cs| {
        if let Some(ref mut data) = SHARED_DATA.borrow(cs).borrow_mut().as_mut() {
            data.system_time_us += 100; // 简化的时间递增
            data.system_time_us
        } else {
            0
        }
    })
}

/// 处理频率捕获事件
fn process_frequency_capture(capture_value: u32, timestamp: u64) {
    free(|cs| {
        if let Some(ref mut data) = SHARED_DATA.borrow(cs).borrow_mut().as_mut() {
            data.frequency_meter.add_capture(capture_value).ok();
        }
    });
}

/// 处理脉冲事件
fn process_pulse_event(timestamp: u64) {
    free(|cs| {
        if let Some(ref mut data) = SHARED_DATA.borrow(cs).borrow_mut().as_mut() {
            data.pulse_counter.add_pulse(timestamp as u32, EdgeType::Rising).ok();
        }
    });
}

/// 处理编码器更新
fn process_encoder_update(a_state: bool, b_state: bool, timestamp: u64) {
    free(|cs| {
        if let Some(ref mut data) = SHARED_DATA.borrow(cs).borrow_mut().as_mut() {
            data.rotary_encoder.update(a_state, b_state, timestamp as u32);
        }
    });
}

/// 处理超声波回声
fn process_ultrasonic_echo(echo_state: bool, timestamp: u64) {
    free(|cs| {
        if let Some(ref mut data) = SHARED_DATA.borrow(cs).borrow_mut().as_mut() {
            if echo_state {
                data.ultrasonic_sensor.echo_start(timestamp as u32);
            } else {
                data.ultrasonic_sensor.echo_end(timestamp as u32);
            }
        }
    });
}

/// 获取频率测量结果
fn get_frequency_measurement() -> Option<f32> {
    free(|cs| {
        if let Some(ref mut data) = SHARED_DATA.borrow(cs).borrow_mut().as_mut() {
            Some(data.frequency_meter.calculate_frequency())
        } else {
            None
        }
    })
}

/// 获取脉冲计数结果
fn get_pulse_count() -> Option<u64> {
    free(|cs| {
        if let Some(ref data) = SHARED_DATA.borrow(cs).borrow().as_ref() {
            Some(data.pulse_counter.get_pulse_count())
        } else {
            None
        }
    })
}

/// 获取编码器测量结果
fn get_encoder_measurement() -> Option<(i32, f32)> {
    free(|cs| {
        if let Some(ref data) = SHARED_DATA.borrow(cs).borrow().as_ref() {
            Some((
                data.rotary_encoder.get_position(),
                data.rotary_encoder.get_velocity_rpm()
            ))
        } else {
            None
        }
    })
}

/// 获取距离测量结果
fn get_distance_measurement() -> Option<f32> {
    free(|cs| {
        if let Some(ref data) = SHARED_DATA.borrow(cs).borrow().as_ref() {
            Some(data.ultrasonic_sensor.get_distance_cm())
        } else {
            None
        }
    })
}

/// 重置所有测量
fn reset_measurements() {
    free(|cs| {
        if let Some(ref mut data) = SHARED_DATA.borrow(cs).borrow_mut().as_mut() {
            data.frequency_meter.reset();
            data.pulse_counter.reset();
            data.rotary_encoder.reset_position();
            data.capture_events.clear();
        }
    });
}

/// 更新所有测量结果
fn update_all_measurements(display_info: &mut DisplayInfo) {
    if let Some(frequency) = get_frequency_measurement() {
        display_info.measurement.frequency_hz = frequency;
    }
    
    if let Some(count) = get_pulse_count() {
        display_info.measurement.pulse_count = count;
    }
    
    if let Some((position, velocity)) = get_encoder_measurement() {
        display_info.measurement.encoder_position = position;
        display_info.measurement.encoder_velocity = velocity;
    }
    
    if let Some(distance) = get_distance_measurement() {
        display_info.measurement.distance_cm = distance;
    }
}

/// 演示频率测量功能
fn demo_frequency_measurement() {
    // 在实际应用中，这里会配置输入捕获中断
    // 并处理捕获事件来测量频率
}

/// 演示脉冲计数功能
fn demo_pulse_counting() {
    // 在实际应用中，这里会配置外部中断
    // 来计数脉冲事件
}

/// 演示编码器读取功能
fn demo_encoder_reading() {
    // 在实际应用中，这里会配置正交编码器接口
    // 或使用外部中断来读取编码器信号
}

/// 演示超声波测量功能
fn demo_ultrasonic_measurement() {
    // 在实际应用中，这里会配置定时器和输入捕获
    // 来精确测量超声波往返时间
}