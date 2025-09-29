#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed, Input, Pull},
    peripherals::*,
    time::Hertz,
    Config,
    i2c::{I2c, TimeoutError},
    spi::{Spi, Config as SpiConfig, Phase, Polarity},
    usart::{Uart, Config as UartConfig},
    adc::{Adc, SampleTime},
    timer::simple_pwm::{PwmPin, SimplePwm},
    bind_interrupts,
};
use embassy_time::{Duration, Timer, Instant, Ticker};
use embassy_net::{Stack, StackResources, Ipv4Address, Ipv4Cidr};
use heapless::{Vec, String};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// 绑定中断
bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<USART2>;
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C1>;
    ADC => embassy_stm32::adc::InterruptHandler<ADC1>;
});

// 传感器数据结构
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct SensorData {
    pub temperature: f32,
    pub humidity: f32,
    pub light_level: u16,
    pub timestamp: u64,
}

// 系统状态
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct SystemStatus {
    pub uptime: u64,
    pub task_count: u8,
    pub error_count: u16,
    pub memory_usage: u32,
}

// 异步传感器接口
pub struct AsyncSensorManager {
    i2c: I2c<'static, I2C1>,
    adc: Adc<'static, ADC1>,
}

impl AsyncSensorManager {
    pub fn new(i2c: I2c<'static, I2C1>, adc: Adc<'static, ADC1>) -> Self {
        Self { i2c, adc }
    }

    pub async fn read_temperature(&mut self) -> Result<f32, TimeoutError> {
        // 模拟I2C温度传感器读取
        let mut buffer = [0u8; 2];
        
        // 发送读取命令到温度传感器 (地址0x48)
        match self.i2c.write_read(0x48, &[0x00], &mut buffer).await {
            Ok(_) => {
                let raw_temp = ((buffer[0] as u16) << 8) | (buffer[1] as u16);
                let temperature = (raw_temp as f32) * 0.0625; // 转换为摄氏度
                Ok(temperature)
            }
            Err(e) => {
                warn!("Temperature sensor read failed: {:?}", e);
                // 返回模拟值
                Ok(25.0 + (embassy_time::Instant::now().as_millis() % 100) as f32 * 0.1)
            }
        }
    }

    pub async fn read_humidity(&mut self) -> Result<f32, TimeoutError> {
        // 模拟I2C湿度传感器读取
        let mut buffer = [0u8; 2];
        
        match self.i2c.write_read(0x40, &[0xE5], &mut buffer).await {
            Ok(_) => {
                let raw_humidity = ((buffer[0] as u16) << 8) | (buffer[1] as u16);
                let humidity = ((raw_humidity as f32) * 125.0 / 65536.0) - 6.0;
                Ok(humidity.max(0.0).min(100.0))
            }
            Err(e) => {
                warn!("Humidity sensor read failed: {:?}", e);
                // 返回模拟值
                Ok(50.0 + (embassy_time::Instant::now().as_millis() % 50) as f32 * 0.2)
            }
        }
    }

    pub async fn read_light_level(&mut self) -> u16 {
        // 模拟ADC光照传感器读取
        // 在实际应用中，这里会读取ADC通道
        (embassy_time::Instant::now().as_millis() % 4096) as u16
    }
}

// 异步网络管理器
pub struct AsyncNetworkManager {
    // 网络栈将在实际实现中添加
}

impl AsyncNetworkManager {
    pub fn new() -> Self {
        Self {}
    }

    pub async fn send_data(&mut self, data: &SensorData) -> Result<(), ()> {
        // 模拟网络发送
        info!("Sending sensor data: {:?}", data);
        Timer::after(Duration::from_millis(100)).await; // 模拟网络延迟
        Ok(())
    }

    pub async fn check_connection(&mut self) -> bool {
        // 模拟网络连接检查
        Timer::after(Duration::from_millis(50)).await;
        true // 假设连接正常
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Embassy Async Example Starting...");

    // 系统配置
    let mut config = Config::default();
    config.rcc.hse = Some(Hertz(8_000_000));
    config.rcc.sysclk = Some(Hertz(84_000_000));
    config.rcc.pclk1 = Some(Hertz(42_000_000));
    config.rcc.pclk2 = Some(Hertz(84_000_000));

    let p = embassy_stm32::init(config);

    // GPIO配置
    let led_status = Output::new(p.PA5, Level::Low, Speed::Low);
    let led_network = Output::new(p.PA6, Level::Low, Speed::Low);
    let button = Input::new(p.PC13, Pull::Up);

    // I2C配置 (用于传感器)
    let i2c = I2c::new(
        p.I2C1,
        p.PB8, // SCL
        p.PB9, // SDA
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz(400_000),
        Default::default(),
    );

    // ADC配置
    let adc = Adc::new(p.ADC1, Irqs);

    // UART配置 (用于调试)
    let uart = Uart::new(
        p.USART2,
        p.PA3, // RX
        p.PA2, // TX
        Irqs,
        p.DMA1_CH5,
        p.DMA1_CH6,
        UartConfig::default(),
    ).unwrap();

    // 创建传感器管理器
    let sensor_manager = AsyncSensorManager::new(i2c, adc);
    let network_manager = AsyncNetworkManager::new();

    // 启动异步任务
    spawner.spawn(sensor_task(sensor_manager)).unwrap();
    spawner.spawn(network_task(network_manager)).unwrap();
    spawner.spawn(led_blink_task(led_status)).unwrap();
    spawner.spawn(network_status_task(led_network)).unwrap();
    spawner.spawn(button_monitor_task(button)).unwrap();
    spawner.spawn(system_monitor_task()).unwrap();
    spawner.spawn(data_logger_task(uart)).unwrap();

    info!("All async tasks spawned successfully");

    // 主循环 - 在Embassy中，main函数会一直运行
    loop {
        Timer::after(Duration::from_secs(1)).await;
        debug!("Main loop heartbeat");
    }
}

// 传感器数据采集任务
#[embassy_executor::task]
async fn sensor_task(mut sensor_manager: AsyncSensorManager) {
    info!("Sensor task started");
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut sample_count = 0u32;

    loop {
        ticker.next().await;
        
        let start_time = Instant::now();
        
        // 并发读取多个传感器
        let (temperature_result, humidity_result, light_level) = embassy_futures::join::join3(
            sensor_manager.read_temperature(),
            sensor_manager.read_humidity(),
            sensor_manager.read_light_level(),
        ).await;

        let sensor_data = SensorData {
            temperature: temperature_result.unwrap_or(0.0),
            humidity: humidity_result.unwrap_or(0.0),
            light_level,
            timestamp: start_time.as_millis(),
        };

        sample_count += 1;
        info!("Sensor reading #{}: {:?}", sample_count, sensor_data);

        // 检查传感器数据异常
        if sensor_data.temperature > 50.0 || sensor_data.temperature < -10.0 {
            warn!("Temperature out of range: {:.1}°C", sensor_data.temperature);
        }

        if sensor_data.humidity > 95.0 {
            warn!("High humidity detected: {:.1}%", sensor_data.humidity);
        }

        let read_duration = start_time.elapsed();
        debug!("Sensor read completed in {} ms", read_duration.as_millis());
    }
}

// 网络通信任务
#[embassy_executor::task]
async fn network_task(mut network_manager: AsyncNetworkManager) {
    info!("Network task started");
    let mut ticker = Ticker::every(Duration::from_secs(5));
    let mut transmission_count = 0u32;

    loop {
        ticker.next().await;

        // 检查网络连接
        if network_manager.check_connection().await {
            // 创建模拟传感器数据
            let sensor_data = SensorData {
                temperature: 25.0,
                humidity: 60.0,
                light_level: 512,
                timestamp: Instant::now().as_millis(),
            };

            // 发送数据到云端
            match network_manager.send_data(&sensor_data).await {
                Ok(_) => {
                    transmission_count += 1;
                    info!("Data transmission #{} successful", transmission_count);
                }
                Err(_) => {
                    warn!("Data transmission failed");
                }
            }
        } else {
            warn!("Network connection lost");
        }
    }
}

// LED状态指示任务
#[embassy_executor::task]
async fn led_blink_task(mut led: Output<'static, PA5>) {
    info!("LED blink task started");
    let mut ticker = Ticker::every(Duration::from_millis(500));

    loop {
        ticker.next().await;
        led.toggle();
        debug!("Status LED toggled");
    }
}

// 网络状态LED任务
#[embassy_executor::task]
async fn network_status_task(mut led: Output<'static, PA6>) {
    info!("Network status task started");
    let mut ticker = Ticker::every(Duration::from_millis(200));
    let mut blink_count = 0u8;

    loop {
        ticker.next().await;
        
        // 快速闪烁表示网络活动
        led.toggle();
        blink_count += 1;
        
        if blink_count >= 10 {
            // 每2秒暂停一次
            Timer::after(Duration::from_secs(1)).await;
            blink_count = 0;
        }
    }
}

// 按钮监控任务
#[embassy_executor::task]
async fn button_monitor_task(mut button: Input<'static, PC13>) {
    info!("Button monitor task started");
    let mut last_state = button.is_high();
    let mut press_count = 0u32;

    loop {
        button.wait_for_any_edge().await;
        
        // 防抖动延迟
        Timer::after(Duration::from_millis(50)).await;
        
        let current_state = button.is_high();
        if current_state != last_state {
            if !current_state {
                // 按钮按下
                press_count += 1;
                info!("Button pressed #{}", press_count);
                
                // 触发系统状态报告
                // 在实际应用中，可以通过信号或共享状态来通知其他任务
            }
            last_state = current_state;
        }
    }
}

// 系统监控任务
#[embassy_executor::task]
async fn system_monitor_task() {
    info!("System monitor task started");
    let mut ticker = Ticker::every(Duration::from_secs(10));
    let start_time = Instant::now();

    loop {
        ticker.next().await;
        
        let uptime = start_time.elapsed().as_secs();
        let system_status = SystemStatus {
            uptime,
            task_count: 7, // 当前运行的任务数量
            error_count: 0, // 错误计数
            memory_usage: 0, // 内存使用情况 (简化)
        };

        info!("System Status: {:?}", system_status);
        
        // 系统健康检查
        if uptime % 60 == 0 {
            info!("System running for {} minutes", uptime / 60);
        }
    }
}

// 数据记录任务
#[embassy_executor::task]
async fn data_logger_task(mut uart: Uart<'static, USART2>) {
    info!("Data logger task started");
    let mut ticker = Ticker::every(Duration::from_secs(30));
    let mut log_count = 0u32;

    loop {
        ticker.next().await;
        
        log_count += 1;
        let log_message = format!("LOG#{}: System operational\r\n", log_count);
        
        // 通过UART发送日志
        if let Err(e) = uart.write(log_message.as_bytes()).await {
            warn!("UART write failed: {:?}", e);
        } else {
            debug!("Log message sent via UART");
        }
    }
}

// 辅助函数
fn format(args: core::fmt::Arguments) -> String<256> {
    let mut string = String::new();
    core::fmt::write(&mut string, args).unwrap();
    string
}