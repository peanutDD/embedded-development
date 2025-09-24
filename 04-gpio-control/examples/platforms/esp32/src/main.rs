#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;

use esp32_hal::{
    clock::ClockControl,
    gpio::{Gpio2, Gpio4, Input, Output, PullUp, PushPull, IO},
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    peripherals::Peripherals,
    prelude::*,
    timer::{Timer, Timer0, TimerGroup},
    Delay,
};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer as AsyncTimer};
use log::info;

// GPIO引脚类型定义
type LedPin = Gpio2<Output<PushPull>>;
type ButtonPin = Gpio4<Input<PullUp>>;

/// ESP32 GPIO控制示例
/// 
/// 功能特性：
/// - LED闪烁控制
/// - 按钮输入检测  
/// - PWM呼吸灯效果
/// - 异步任务处理
/// - WiFi状态指示
/// - 温度传感器读取
#[main]
async fn main(spawner: Spawner) {
    println!("ESP32 GPIO控制示例启动");

    // 初始化外设
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // 初始化GPIO
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    
    // LED引脚 (GPIO2 - 内置LED)
    let led = io.pins.gpio2.into_push_pull_output();
    
    // 按钮引脚 (GPIO4)
    let button = io.pins.gpio4.into_pull_up_input();

    // PWM配置 (GPIO5)
    let pwm_pin = io.pins.gpio5.into_push_pull_output();
    
    // 配置LEDC (LED控制器用于PWM)
    let mut ledc = Ledc::new(peripherals.LEDC, &clocks);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    
    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty13Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 1000u32.Hz(),
        })
        .unwrap();

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, pwm_pin);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // 定时器配置
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    timer0.start(1u64.secs());

    // 延时器
    let mut delay = Delay::new(&clocks);

    info!("ESP32 GPIO和定时器配置完成");

    // 启动异步任务
    spawner.spawn(led_blink_task(led)).ok();
    spawner.spawn(button_monitor_task(button)).ok();
    spawner.spawn(pwm_breathing_task(channel0)).ok();
    spawner.spawn(system_monitor_task()).ok();

    // 主循环
    loop {
        AsyncTimer::after(Duration::from_millis(1000)).await;
        info!("主循环运行中...");
    }
}

/// LED闪烁任务
#[embassy_executor::task]
async fn led_blink_task(mut led: LedPin) {
    let mut state = false;
    loop {
        if state {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
        state = !state;
        
        AsyncTimer::after(Duration::from_millis(500)).await;
    }
}

/// 按钮监控任务
#[embassy_executor::task]
async fn button_monitor_task(button: ButtonPin) {
    let mut last_state = button.is_high().unwrap();
    let mut press_count = 0u32;
    
    loop {
        let current_state = button.is_high().unwrap();
        
        if !current_state && last_state {
            // 按钮按下
            press_count += 1;
            info!("按钮按下 - 次数: {}", press_count);
        }
        
        last_state = current_state;
        AsyncTimer::after(Duration::from_millis(50)).await;
    }
}

/// PWM呼吸灯任务
#[embassy_executor::task]
async fn pwm_breathing_task(mut pwm_channel: impl ChannelIFace<LowSpeed>) {
    let mut duty = 0u8;
    let mut direction = true;
    
    loop {
        // 更新PWM占空比
        if direction {
            if duty >= 100 {
                direction = false;
            } else {
                duty += 2;
            }
        } else {
            if duty <= 2 {
                direction = true;
            } else {
                duty -= 2;
            }
        }
        
        pwm_channel.set_duty_pct(duty).unwrap();
        AsyncTimer::after(Duration::from_millis(50)).await;
    }
}

/// 系统监控任务
#[embassy_executor::task]
async fn system_monitor_task() {
    let mut cycle_count = 0u32;
    
    loop {
        cycle_count += 1;
        
        // 系统状态报告
        if cycle_count % 10 == 0 {
            info!("系统监控 - 周期: {}, 堆内存: {}KB", 
                cycle_count, get_free_heap_size() / 1024);
        }
        
        AsyncTimer::after(Duration::from_secs(1)).await;
    }
}

/// 获取空闲堆内存大小 (模拟)
fn get_free_heap_size() -> u32 {
    // ESP32实际实现需要调用ESP-IDF函数
    // 这里返回模拟值
    200 * 1024 // 200KB
}

/// GPIO控制器
pub struct GpioController {
    led_state: bool,
    button_state: bool,
    pwm_duty: u8,
    event_count: u32,
}

impl GpioController {
    pub fn new() -> Self {
        Self {
            led_state: false,
            button_state: false,
            pwm_duty: 0,
            event_count: 0,
        }
    }

    pub fn update_led(&mut self, state: bool) {
        self.led_state = state;
        self.event_count += 1;
        info!("LED状态更新: {}", state);
    }

    pub fn update_button(&mut self, pressed: bool) {
        if pressed != self.button_state {
            self.button_state = pressed;
            self.event_count += 1;
            info!("按钮状态变化: {}", pressed);
        }
    }

    pub fn update_pwm(&mut self, duty: u8) {
        self.pwm_duty = duty;
        info!("PWM占空比更新: {}%", duty);
    }

    pub fn get_stats(&self) -> (bool, bool, u8, u32) {
        (self.led_state, self.button_state, self.pwm_duty, self.event_count)
    }
}

/// WiFi状态管理器
pub struct WiFiManager {
    connected: bool,
    signal_strength: i8,
    connection_count: u32,
}

impl WiFiManager {
    pub fn new() -> Self {
        Self {
            connected: false,
            signal_strength: -50,
            connection_count: 0,
        }
    }

    pub fn connect(&mut self, ssid: &str) -> Result<(), &'static str> {
        // 模拟WiFi连接
        info!("连接到WiFi: {}", ssid);
        self.connected = true;
        self.connection_count += 1;
        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.connected = false;
        info!("WiFi已断开");
    }

    pub fn get_status(&self) -> (bool, i8, u32) {
        (self.connected, self.signal_strength, self.connection_count)
    }
}

/// 传感器数据管理器
pub struct SensorManager {
    temperature: f32,
    humidity: f32,
    pressure: f32,
    last_update: u32,
}

impl SensorManager {
    pub fn new() -> Self {
        Self {
            temperature: 25.0,
            humidity: 60.0,
            pressure: 1013.25,
            last_update: 0,
        }
    }

    pub fn read_sensors(&mut self) {
        // 模拟传感器读取
        self.temperature = 25.0 + (self.last_update as f32 * 0.1) % 10.0;
        self.humidity = 60.0 + (self.last_update as f32 * 0.2) % 20.0;
        self.pressure = 1013.25 + (self.last_update as f32 * 0.05) % 5.0;
        self.last_update += 1;
        
        info!("传感器数据 - 温度: {:.1}°C, 湿度: {:.1}%, 气压: {:.2}hPa", 
            self.temperature, self.humidity, self.pressure);
    }

    pub fn get_data(&self) -> (f32, f32, f32) {
        (self.temperature, self.humidity, self.pressure)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpio_controller() {
        let mut controller = GpioController::new();
        controller.update_led(true);
        assert_eq!(controller.led_state, true);
        assert_eq!(controller.event_count, 1);
    }

    #[test]
    fn test_wifi_manager() {
        let mut wifi = WiFiManager::new();
        assert_eq!(wifi.connected, false);
        
        wifi.connect("TestNetwork").unwrap();
        assert_eq!(wifi.connected, true);
        assert_eq!(wifi.connection_count, 1);
    }

    #[test]
    fn test_sensor_manager() {
        let mut sensors = SensorManager::new();
        let initial_temp = sensors.temperature;
        
        sensors.read_sensors();
        // 温度应该有变化
        assert_ne!(sensors.temperature, initial_temp);
    }
}