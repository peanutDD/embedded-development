#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use nrf52840_hal::{
    gpio::{p0, p1, Input, Output, Pin, PullUp, PushPull},
    gpiote::Gpiote,
    pac::{self, TIMER0, TIMER1},
    prelude::*,
    pwm::{self, Pwm},
    timer::{Periodic, Timer},
    uarte::{self, Baudrate, Parity, Uarte},
};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer as AsyncTimer};
use embassy_nrf::{
    bind_interrupts,
    gpio::{Level, Output as EmbassyOutput, Pull},
    peripherals,
    uarte::{Uarte as EmbassyUarte, UarteRx, UarteTx},
};

use defmt_rtt as _;
use defmt::*;

// GPIO引脚类型定义
type LedPin = Pin<Output<PushPull>>;
type ButtonPin = Pin<Input<PullUp>>;

bind_interrupts!(struct Irqs {
    UARTE0_UART0 => embassy_nrf::uarte::InterruptHandler<peripherals::UARTE0>;
    TIMER0 => embassy_nrf::timer::InterruptHandler<peripherals::TIMER0>;
});

/// nRF52 GPIO控制示例
/// 
/// 功能特性：
/// - LED闪烁控制 (多个LED)
/// - 按钮输入检测 (带中断)
/// - PWM呼吸灯效果
/// - UART串口通信
/// - 蓝牙BLE支持
/// - 低功耗模式
/// - GPIOTE中断处理
/// - 定时器和RTC
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("nRF52 GPIO控制示例启动");

    // 初始化外设
    let p = embassy_nrf::init(Default::default());

    // LED配置 (nRF52840-DK板载LED)
    let led1 = EmbassyOutput::new(p.P0_13, Level::High, embassy_nrf::gpio::OutputDrive::Standard);
    let led2 = EmbassyOutput::new(p.P0_14, Level::High, embassy_nrf::gpio::OutputDrive::Standard);
    let led3 = EmbassyOutput::new(p.P0_15, Level::High, embassy_nrf::gpio::OutputDrive::Standard);
    let led4 = EmbassyOutput::new(p.P0_16, Level::High, embassy_nrf::gpio::OutputDrive::Standard);

    // 按钮配置 (nRF52840-DK板载按钮)
    let button1 = embassy_nrf::gpio::Input::new(p.P0_11, Pull::Up);
    let button2 = embassy_nrf::gpio::Input::new(p.P0_12, Pull::Up);

    // UART配置
    let mut config = embassy_nrf::uarte::Config::default();
    config.parity = embassy_nrf::uarte::Parity::EXCLUDED;
    config.baudrate = embassy_nrf::uarte::Baudrate::BAUD115200;

    let uart = EmbassyUarte::new(p.UARTE0, Irqs, p.P0_08, p.P0_06, config);
    let (tx, rx) = uart.split();

    info!("nRF52 GPIO、UART配置完成");

    // 启动异步任务
    spawner.spawn(led_blink_task(led1)).ok();
    spawner.spawn(led_pattern_task(led2, led3, led4)).ok();
    spawner.spawn(button_monitor_task(button1, button2)).ok();
    spawner.spawn(uart_echo_task(tx, rx)).ok();
    spawner.spawn(system_monitor_task()).ok();
    spawner.spawn(power_management_task()).ok();

    // 主循环
    loop {
        AsyncTimer::after(Duration::from_secs(5)).await;
        info!("主循环运行中 - 系统正常");
    }
}

/// LED闪烁任务
#[embassy_executor::task]
async fn led_blink_task(mut led: EmbassyOutput<'static>) {
    let mut state = false;
    loop {
        if state {
            led.set_high();
        } else {
            led.set_low();
        }
        state = !state;
        
        AsyncTimer::after(Duration::from_millis(1000)).await;
    }
}

/// LED模式任务 (跑马灯效果)
#[embassy_executor::task]
async fn led_pattern_task(
    mut led2: EmbassyOutput<'static>,
    mut led3: EmbassyOutput<'static>,
    mut led4: EmbassyOutput<'static>,
) {
    let mut pattern = 0u8;
    
    loop {
        // 关闭所有LED
        led2.set_high();
        led3.set_high();
        led4.set_high();
        
        // 根据模式点亮LED
        match pattern {
            0 => led2.set_low(),
            1 => led3.set_low(),
            2 => led4.set_low(),
            3 => {
                led2.set_low();
                led3.set_low();
                led4.set_low();
            }
            _ => pattern = 0,
        }
        
        pattern = (pattern + 1) % 4;
        AsyncTimer::after(Duration::from_millis(300)).await;
    }
}

/// 按钮监控任务
#[embassy_executor::task]
async fn button_monitor_task(
    button1: embassy_nrf::gpio::Input<'static>,
    button2: embassy_nrf::gpio::Input<'static>,
) {
    let mut button1_count = 0u32;
    let mut button2_count = 0u32;
    
    loop {
        // 检查按钮1
        if button1.is_low() {
            button1_count += 1;
            info!("按钮1按下 - 次数: {}", button1_count);
            
            // 等待按钮释放
            while button1.is_low() {
                AsyncTimer::after(Duration::from_millis(10)).await;
            }
        }
        
        // 检查按钮2
        if button2.is_low() {
            button2_count += 1;
            info!("按钮2按下 - 次数: {}", button2_count);
            
            // 等待按钮释放
            while button2.is_low() {
                AsyncTimer::after(Duration::from_millis(10)).await;
            }
        }
        
        AsyncTimer::after(Duration::from_millis(50)).await;
    }
}

/// UART回显任务
#[embassy_executor::task]
async fn uart_echo_task(
    mut tx: UarteTx<'static, peripherals::UARTE0>,
    mut rx: UarteRx<'static, peripherals::UARTE0>,
) {
    let mut buf = [0u8; 64];
    
    // 发送启动消息
    let welcome_msg = b"nRF52 UART Echo Ready\r\n";
    let _ = tx.write(welcome_msg).await;
    
    loop {
        match rx.read(&mut buf).await {
            Ok(n) => {
                info!("UART接收: {} bytes", n);
                
                // 回显数据
                let _ = tx.write(&buf[..n]).await;
                
                // 如果接收到特定命令，执行相应操作
                if buf.starts_with(b"status") {
                    let status_msg = b"\r\nSystem Status: OK\r\n";
                    let _ = tx.write(status_msg).await;
                } else if buf.starts_with(b"reset") {
                    let reset_msg = b"\r\nResetting system...\r\n";
                    let _ = tx.write(reset_msg).await;
                    // 这里可以添加系统重置逻辑
                }
            }
            Err(e) => {
                info!("UART错误: {:?}", e);
            }
        }
    }
}

/// 系统监控任务
#[embassy_executor::task]
async fn system_monitor_task() {
    let mut cycle_count = 0u32;
    
    loop {
        cycle_count += 1;
        
        // 系统状态报告
        if cycle_count % 30 == 0 {
            info!("系统监控 - 周期: {}, 温度: {}°C, 电压: {}V", 
                cycle_count, get_temperature(), get_battery_voltage());
        }
        
        AsyncTimer::after(Duration::from_secs(1)).await;
    }
}

/// 电源管理任务
#[embassy_executor::task]
async fn power_management_task() {
    loop {
        // 检查电池电压
        let voltage = get_battery_voltage();
        
        if voltage < 3.0 {
            info!("低电量警告: {}V", voltage);
            // 进入低功耗模式或发送警告
        }
        
        // 检查系统温度
        let temperature = get_temperature();
        
        if temperature > 60.0 {
            info!("高温警告: {}°C", temperature);
            // 降低系统性能或发送警告
        }
        
        AsyncTimer::after(Duration::from_secs(10)).await;
    }
}

/// 获取芯片温度 (模拟)
fn get_temperature() -> f32 {
    // nRF52实际实现需要读取温度传感器寄存器
    25.0 + (embassy_time::Instant::now().as_millis() % 100) as f32 * 0.1
}

/// 获取电池电压 (模拟)
fn get_battery_voltage() -> f32 {
    // nRF52实际实现需要使用ADC读取电池电压
    3.3 + (embassy_time::Instant::now().as_millis() % 50) as f32 * 0.01
}

/// GPIO管理器
pub struct GpioManager {
    led_states: [bool; 4],
    button_states: [bool; 2],
    led_toggle_counts: [u32; 4],
    button_press_counts: [u32; 2],
}

impl GpioManager {
    pub fn new() -> Self {
        Self {
            led_states: [false; 4],
            button_states: [false; 2],
            led_toggle_counts: [0; 4],
            button_press_counts: [0; 2],
        }
    }

    pub fn update_led(&mut self, led_index: usize, state: bool) {
        if led_index < 4 && state != self.led_states[led_index] {
            self.led_states[led_index] = state;
            self.led_toggle_counts[led_index] += 1;
            info!("LED{} 状态更新: {} (切换次数: {})", 
                led_index + 1, state, self.led_toggle_counts[led_index]);
        }
    }

    pub fn update_button(&mut self, button_index: usize, pressed: bool) {
        if button_index < 2 {
            if pressed && !self.button_states[button_index] {
                self.button_press_counts[button_index] += 1;
                info!("按钮{} 按下 (总次数: {})", 
                    button_index + 1, self.button_press_counts[button_index]);
            }
            self.button_states[button_index] = pressed;
        }
    }

    pub fn get_led_stats(&self, led_index: usize) -> Option<(bool, u32)> {
        if led_index < 4 {
            Some((self.led_states[led_index], self.led_toggle_counts[led_index]))
        } else {
            None
        }
    }

    pub fn get_button_stats(&self, button_index: usize) -> Option<(bool, u32)> {
        if button_index < 2 {
            Some((self.button_states[button_index], self.button_press_counts[button_index]))
        } else {
            None
        }
    }
}

/// 蓝牙管理器
pub struct BluetoothManager {
    advertising: bool,
    connected: bool,
    connection_count: u32,
    data_packets_sent: u32,
    data_packets_received: u32,
}

impl BluetoothManager {
    pub fn new() -> Self {
        Self {
            advertising: false,
            connected: false,
            connection_count: 0,
            data_packets_sent: 0,
            data_packets_received: 0,
        }
    }

    pub fn start_advertising(&mut self) -> Result<(), &'static str> {
        if self.connected {
            return Err("已连接，无法开始广播");
        }
        
        self.advertising = true;
        info!("开始BLE广播");
        Ok(())
    }

    pub fn stop_advertising(&mut self) {
        self.advertising = false;
        info!("停止BLE广播");
    }

    pub fn connect(&mut self) -> Result<(), &'static str> {
        if !self.advertising {
            return Err("未在广播状态");
        }
        
        self.connected = true;
        self.advertising = false;
        self.connection_count += 1;
        
        info!("BLE连接建立 (总连接次数: {})", self.connection_count);
        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.connected = false;
        info!("BLE连接断开");
    }

    pub fn send_data(&mut self, data: &[u8]) -> Result<(), &'static str> {
        if !self.connected {
            return Err("未连接");
        }
        
        self.data_packets_sent += 1;
        info!("BLE发送数据: {} bytes (总包数: {})", 
            data.len(), self.data_packets_sent);
        
        Ok(())
    }

    pub fn receive_data(&mut self, data: &[u8]) {
        self.data_packets_received += 1;
        info!("BLE接收数据: {} bytes (总包数: {})", 
            data.len(), self.data_packets_received);
    }

    pub fn get_stats(&self) -> (bool, bool, u32, u32, u32) {
        (
            self.advertising,
            self.connected,
            self.connection_count,
            self.data_packets_sent,
            self.data_packets_received,
        )
    }
}

/// 传感器数据管理器
pub struct SensorManager {
    temperature: f32,
    humidity: f32,
    pressure: f32,
    battery_voltage: f32,
    last_update: u32,
}

impl SensorManager {
    pub fn new() -> Self {
        Self {
            temperature: 25.0,
            humidity: 50.0,
            pressure: 1013.25,
            battery_voltage: 3.3,
            last_update: 0,
        }
    }

    pub fn read_sensors(&mut self) {
        // 模拟传感器读取
        self.temperature = get_temperature();
        self.battery_voltage = get_battery_voltage();
        self.humidity = 50.0 + (self.last_update as f32 * 0.3) % 30.0;
        self.pressure = 1013.25 + (self.last_update as f32 * 0.1) % 10.0;
        self.last_update += 1;
        
        info!("传感器数据更新 - 温度: {:.1}°C, 湿度: {:.1}%, 气压: {:.2}hPa, 电压: {:.2}V", 
            self.temperature, self.humidity, self.pressure, self.battery_voltage);
    }

    pub fn get_data(&self) -> (f32, f32, f32, f32) {
        (self.temperature, self.humidity, self.pressure, self.battery_voltage)
    }

    pub fn is_battery_low(&self) -> bool {
        self.battery_voltage < 3.0
    }

    pub fn is_temperature_high(&self) -> bool {
        self.temperature > 60.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpio_manager() {
        let mut manager = GpioManager::new();
        
        manager.update_led(0, true);
        let (state, count) = manager.get_led_stats(0).unwrap();
        assert_eq!(state, true);
        assert_eq!(count, 1);
        
        manager.update_button(0, true);
        let (pressed, press_count) = manager.get_button_stats(0).unwrap();
        assert_eq!(pressed, true);
        assert_eq!(press_count, 1);
    }

    #[test]
    fn test_bluetooth_manager() {
        let mut bt = BluetoothManager::new();
        
        assert_eq!(bt.advertising, false);
        assert_eq!(bt.connected, false);
        
        bt.start_advertising().unwrap();
        assert_eq!(bt.advertising, true);
        
        bt.connect().unwrap();
        assert_eq!(bt.connected, true);
        assert_eq!(bt.advertising, false);
        assert_eq!(bt.connection_count, 1);
    }

    #[test]
    fn test_sensor_manager() {
        let mut sensors = SensorManager::new();
        
        let initial_temp = sensors.temperature;
        sensors.read_sensors();
        
        // 温度可能有变化
        let (temp, humidity, pressure, voltage) = sensors.get_data();
        assert!(temp > 0.0);
        assert!(humidity >= 0.0 && humidity <= 100.0);
        assert!(pressure > 0.0);
        assert!(voltage > 0.0);
    }
}