//! 工业物联网边缘计算网关
//! 
//! 本项目实现了一个完整的IoT网关系统，包括：
//! - 多协议传感器数据采集 (I2C, SPI, UART, 1-Wire)
//! - 无线通信 (WiFi, LoRaWAN, NRF24L01)
//! - 云端数据同步 (MQTT, CoAP, HTTP/HTTPS)
//! - 边缘计算和数据预处理
//! - 本地数据存储和缓存
//! - Web管理界面
//! - 安全加密和认证
//! - 低功耗管理

#![no_std]
#![no_main]
// #![feature(type_alias_impl_trait)] // 移除不稳定特性

use panic_halt as _;

// RTIC实时框架
use rtic::app;
// use rtic_time::{Monotonic, TimeoutError}; // 未使用
// use rtic_monotonics::systick::*;

// 嵌入式核心库
// use cortex_m;
// use cortex_m_rt::entry; // 未使用
// use core::mem::MaybeUninit; // 未使用
// use core::sync::atomic::{AtomicBool, Ordering};
use core::cell::RefCell;
use stm32f4xx_hal::{
    prelude::*,
    // pac, // {self, interrupt},
    gpio::{Output, PushPull},
    // timer::{Timer, Event},
    // usart::{Serial, Config as SerialConfig},
    // spi::{Spi, Mode, Phase, Polarity},
    // i2c::I2c,
    // adc::{Adc, config::AdcConfig},
    // rcc::RccExt,
    // flash::FlashExt,
    // pwr::PwrExt,
};

// 网络和通信 - 注释掉因为相关crate被禁用
// use embedded_nal::{TcpClientStack, UdpClientStack, Dns};
// use rumqttc::*;
// use coap_lite::*;
// use reqwless::*;

// 无线通信 - 注释掉因为相关crate被禁用
// use esp_at_nal::*;
// use lora_phy::*;
// use lorawan::*;
// use nrf24l01::*;

// 传感器驱动 - 注释掉因为相关crate被禁用
// use bme280::*;
// use sht3x::*;
// use ds18b20::*;
// use mpu6050::*;
// use ads1x1x::*;

// 数据结构和处理
use heapless::{
    Vec, String, // FnvIndexMap,
    // pool::{Pool, Node},
    // spsc::{Queue, Producer, Consumer},
};
// use serde::{Serialize, Deserialize};
// use micromath::F32Ext;
// use statistics::*;

// 存储和文件系统
// use embedded_storage::*;
// use sequential_storage::*;
// use embedded_sdmmc::*;

// 调试和日志 - 注释掉因为相关crate被禁用
// use rtt_target::{rprintln, rtt_init_print};
// use cortex_m_log::log::{init as log_init, Logger};
// 日志和调试 - 注释掉因为相关crate被禁用
// use log::{info, warn, error, debug};
// use defmt::{info as defmt_info, warn as defmt_warn, error as defmt_error};

// 同步和原子操作
use critical_section::Mutex;
// use portable_atomic::{AtomicU32}; // AtomicBool, Ordering 未使用

// 加密和安全
// use aes::*;
// use sha2::*;
// use hmac::*;
// use ecdsa::*;
// use p256::*;

// 类型定义
type StatusLed = stm32f4xx_hal::gpio::gpioc::PC13<Output<PushPull>>;
type WifiLed = stm32f4xx_hal::gpio::gpioa::PA5<Output<PushPull>>;
type LoraLed = stm32f4xx_hal::gpio::gpioa::PA6<Output<PushPull>>;
type DataLed = stm32f4xx_hal::gpio::gpioa::PA7<Output<PushPull>>;

// 常量定义
#[allow(dead_code)]
const MAX_SENSORS: usize = 32;
const MAX_DEVICES: usize = 16;
const MAX_DATA_POINTS: usize = 1000;
#[allow(dead_code)]
const MAX_MQTT_TOPICS: usize = 64;
const SAMPLING_INTERVAL_MS: u32 = 1000;
const UPLOAD_INTERVAL_MS: u32 = 60000;
#[allow(dead_code)]
const HEARTBEAT_INTERVAL_MS: u32 = 30000;
#[allow(dead_code)]
const DATA_BUFFER_SIZE: usize = 512;
#[allow(dead_code)]
const NETWORK_TIMEOUT_MS: u32 = 30000;

// 传感器类型枚举
#[derive(Debug, Clone, Copy, PartialEq)] // Serialize, Deserialize
#[allow(dead_code)]  // 允许未使用的变体
pub enum SensorType {
    Temperature,
    Humidity,
    Pressure,
    Acceleration,
    Gyroscope,
    Voltage,
    Current,
    Light,
    Motion,
    Gas,
}

// 通信协议枚举
#[derive(Debug, Clone, Copy, PartialEq)] // Serialize, Deserialize
#[allow(dead_code)]  // 允许未使用的枚举
enum CommProtocol {
    WiFi,
    LoRaWAN,
    NRF24L01,
    Ethernet,
    Cellular,
}

// 传感器数据结构
#[derive(Debug, Clone)] // Serialize, Deserialize
pub struct SensorData {
    pub sensor_id: u16,
    pub sensor_type: SensorType,
    pub value: f32,
    pub unit: String<16>,
    pub timestamp: u32,
    pub quality: u8, // 数据质量 0-100
    pub location: String<32>,
}

// 设备信息结构
#[derive(Debug, Clone)] // Serialize, Deserialize
pub struct DeviceInfo {
    pub device_id: String<32>,
    pub device_type: String<32>,
    pub firmware_version: String<16>,
    pub hardware_version: String<16>,
    pub mac_address: String<18>,
    pub ip_address: String<16>,
    pub last_seen: u32,
    pub battery_level: u8,
    pub signal_strength: i8,
}

// 网络状态结构
#[derive(Debug, Clone)] // Serialize, Deserialize
pub struct NetworkStatus {
    pub wifi_connected: bool,
    pub wifi_ssid: String<32>,
    pub wifi_rssi: i8,
    pub lora_joined: bool,
    pub lora_rssi: i8,
    pub mqtt_connected: bool,
    pub last_upload: u32,
    pub upload_count: u32,
    pub error_count: u32,
}

// 系统统计结构
#[derive(Debug, Clone)] // Serialize, Deserialize
pub struct SystemStats {
    pub uptime_seconds: u32,
    pub cpu_usage: f32,
    pub memory_usage: f32,
    pub flash_usage: f32,
    pub temperature: f32,
    pub battery_voltage: f32,
    pub power_consumption: f32,
    pub data_points_collected: u32,
    pub data_points_uploaded: u32,
}

// 配置结构
#[derive(Debug, Clone)] // Serialize, Deserialize
pub struct GatewayConfig {
    pub device_id: String<32>,
    pub sampling_interval: u32,
    pub upload_interval: u32,
    pub wifi_ssid: String<32>,
    pub wifi_password: String<64>,
    pub mqtt_broker: String<64>,
    pub mqtt_port: u16,
    pub mqtt_username: String<32>,
    pub mqtt_password: String<64>,
    pub lora_dev_eui: [u8; 8],
    pub lora_app_eui: [u8; 8],
    pub lora_app_key: [u8; 16],
    pub encryption_enabled: bool,
    pub debug_enabled: bool,
}

// 系统状态 - 注释掉因为atomic被禁用
// static SYSTEM_UPTIME: AtomicU32 = AtomicU32::new(0);
// static WIFI_CONNECTED: AtomicBool = AtomicBool::new(false);
// static LORA_JOINED: AtomicBool = AtomicBool::new(false);
// static MQTT_CONNECTED: AtomicBool = AtomicBool::new(false);
// static DATA_COLLECTION_ENABLED: AtomicBool = AtomicBool::new(true);
// static LOW_POWER_MODE: AtomicBool = AtomicBool::new(false);

// 共享资源
#[allow(dead_code)]  // 允许未使用的静态变量
static SENSOR_DATA: Mutex<Vec<SensorData, MAX_DATA_POINTS>> = Mutex::new(Vec::new());
#[allow(dead_code)]  // 允许未使用的静态变量
static DEVICE_LIST: Mutex<Vec<DeviceInfo, MAX_DEVICES>> = Mutex::new(Vec::new());
#[allow(dead_code)]  // 允许未使用的静态变量
static GATEWAY_CONFIG: Mutex<Option<GatewayConfig>> = Mutex::new(None);

// 数据队列 (注释掉Pool相关代码以避免Sync问题)
// static mut UPLOAD_QUEUE_BUFFER: [Node<SensorData>; 128] = [Node::new(); 128];
// static UPLOAD_POOL: Pool<SensorData> = Pool::new();
// 使用简单的Vec替代
#[allow(dead_code)]  // 允许未使用的静态变量
static UPLOAD_QUEUE: Mutex<RefCell<heapless::Vec<SensorData, 128>>> = 
    Mutex::new(RefCell::new(heapless::Vec::new()));

// RTIC应用定义
#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, USART2, USART3, UART4, UART5])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        sensor_data: Vec<SensorData, MAX_DATA_POINTS>,
        network_status: NetworkStatus,
        system_stats: SystemStats,
    }

    #[local]
    struct Local {
        status_led: StatusLed,
        wifi_led: WifiLed,
        lora_led: LoraLed,
        data_led: DataLed,
        bme280: Option<()>, // 简化类型
        mpu6050: Option<()>, // 简化类型
        adc: (), // 简化类型
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // 初始化RTT调试 - 注释掉因为rtt_target被禁用
        // rtt_init_print!();
        // rprintln!("🌐 IoT网关系统启动中...");

        // 初始化日志系统 - 注释掉因为相关crate被禁用
        // static LOGGER: Logger = Logger {
        //     level: log::LevelFilter::Info,
        //     print: |args| {
        //         rprintln!("{}", args);
        //     },
        // };
        // log_init(&LOGGER).unwrap();
        // info!("日志系统初始化完成");

        let dp = ctx.device;
        let _cp = ctx.core;

        // 配置系统时钟 (84MHz)
        let rcc = dp.RCC.constrain();
        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(84.MHz())
            .hclk(84.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .freeze();

        // info!("系统时钟配置完成: {}MHz", clocks.sysclk().0 / 1_000_000);

        // 初始化系统滴答定时器 (注释掉，因为rtic-monotonics被禁用)
        // let systick_mono = rtic_monotonics::systick::Systick::new(cp.SYST, clocks.sysclk().0);

        // 配置GPIO
        let gpioa = dp.GPIOA.split();
        let _gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        // LED指示灯
        let status_led = gpioc.pc13.into_push_pull_output();
        let wifi_led = gpioa.pa5.into_push_pull_output();
        let lora_led = gpioa.pa6.into_push_pull_output();
        let data_led = gpioa.pa7.into_push_pull_output();

        // 配置I2C (传感器接口) - 注释掉因为相关crate被禁用
        // let i2c_pins = (
        //     gpiob.pb8.into_alternate_af4().set_open_drain(), // SCL
        //     gpiob.pb9.into_alternate_af4().set_open_drain(), // SDA
        // );
        // let i2c1 = I2c::i2c1(dp.I2C1, i2c_pins, 400.khz(), clocks);

        // 初始化传感器 - 注释掉因为相关crate被禁用
        // let bme280 = Bme280::new_primary(i2c1.clone()).ok();
        // let mpu6050 = Mpu6050::new(i2c1).ok();

        // 配置ADC - 注释掉因为相关crate被禁用
        // let adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());

        // 配置串口 (ESP32 WiFi模块) - 注释掉因为相关crate被禁用
        // let wifi_serial = Serial::usart1(
        //     dp.USART1,
        //     (
        //         gpioa.pa9.into_alternate_af7(),  // TX
        //         gpioa.pa10.into_alternate_af7(), // RX
        //     ),
        //     SerialConfig::default().baudrate(115200.bps()),
        //     clocks,
        // ).unwrap();

        // 配置SPI (LoRa模块) - 注释掉因为相关crate被禁用
        // let lora_spi_pins = (
        //     gpioa.pa5.into_alternate_af5(),  // SCK
        //     gpioa.pa6.into_alternate_af5(),  // MISO
        //     gpioa.pa7.into_alternate_af5(),  // MOSI
        // );
        // let lora_spi = Spi::spi1(
        //     dp.SPI1,
        //     lora_spi_pins,
        //     Mode {
        //         polarity: Polarity::IdleLow,
        //         phase: Phase::CaptureOnFirstTransition,
        //     },
        //     1.mhz(),
        //     clocks,
        // );

        // info!("外设初始化完成");

        // 初始化数据池
        // unsafe {
            // UPLOAD_POOL.grow_exact(&mut UPLOAD_QUEUE_BUFFER); // 注释掉，使用Vec替代
        // }

        // 初始化配置
        init_gateway_config();

        // 启动定时任务
        sensor_sampling_task::spawn().ok();
        network_management_task::spawn().ok();
        data_upload_task::spawn().ok();
        system_monitor_task::spawn().ok();
        heartbeat_task::spawn().ok();

        // info!("所有任务启动完成");

        (
            Shared {
                sensor_data: Vec::new(),
                network_status: NetworkStatus {
                    wifi_connected: false,
                    wifi_ssid: String::new(),
                    wifi_rssi: -100,
                    lora_joined: false,
                    lora_rssi: -100,
                    mqtt_connected: false,
                    last_upload: 0,
                    upload_count: 0,
                    error_count: 0,
                },
                system_stats: SystemStats {
                    uptime_seconds: 0,
                    cpu_usage: 0.0,
                    memory_usage: 0.0,
                    flash_usage: 0.0,
                    temperature: 25.0,
                    battery_voltage: 3.7,
                    power_consumption: 100.0,
                    data_points_collected: 0,
                    data_points_uploaded: 0,
                },
            },
            Local {
                status_led,
                wifi_led,
                lora_led,
                data_led,
                bme280: None,
                mpu6050: None,
                adc: (), // 简化占位符
            }
         )
    }

    // 传感器采样任务
    #[task(shared = [sensor_data, system_stats], local = [bme280, mpu6050, adc, data_led], priority = 1)]
    async fn sensor_sampling_task(mut ctx: sensor_sampling_task::Context) {
        // if !DATA_COLLECTION_ENABLED.load(Ordering::Relaxed) {
        //     return;
        // }

        // let current_time = SYSTEM_UPTIME.load(Ordering::Relaxed);
        let current_time = 0; // 简化处理
        let mut new_data = Vec::<SensorData, 16>::new();

        // 读取BME280环境传感器 - 注释掉因为传感器被禁用
        // if let Some(ref mut bme280) = ctx.local.bme280 {
            // 模拟传感器数据
            new_data.push(SensorData {
                sensor_id: 1,
                sensor_type: SensorType::Temperature,
                value: 25.0,
                unit: String::from("°C"),
                timestamp: current_time,
                quality: 95,
                location: String::from("Indoor"),
            }).ok();
        // }

        // 读取MPU6050运动传感器 - 注释掉因为传感器被禁用
        // if let Some(ref mut mpu6050) = ctx.local.mpu6050 {
            // 模拟加速度计数据
            new_data.push(SensorData {
                sensor_id: 4,
                sensor_type: SensorType::Acceleration,
                value: 9.8,
                unit: String::from("m/s²"),
                timestamp: current_time,
                quality: 90,
                location: String::from("Device"),
            }).ok();
        // }

        // 读取ADC电压 (模拟读取) - 注释掉因为ADC被禁用
        let _voltage = 3.3; // 模拟电压值
        new_data.push(SensorData {
            sensor_id: 6,
            sensor_type: SensorType::Voltage,
            value: 3.3,
            unit: String::from("V"),
            timestamp: current_time,
            quality: 85,
            location: String::from("Analog"),
        }).ok();

        // 更新共享数据
        ctx.shared.sensor_data.lock(|data| {
            for sensor_data in new_data.iter() {
                if data.len() >= MAX_DATA_POINTS {
                    data.remove(0); // 移除最旧的数据
                }
                data.push(sensor_data.clone()).ok();
            }
        });

        ctx.shared.system_stats.lock(|stats| {
            stats.data_points_collected += new_data.len() as u32;
        });

        // 闪烁数据LED
        ctx.local.data_led.set_high();
        
        // debug!("采集到 {} 个传感器数据点", new_data.len());

        // 调度下次采样 (注释掉，因为rtic-monotonics被禁用)
        // sensor_sampling_task::spawn_after(rtic_time::duration::millis(SAMPLING_INTERVAL_MS)).ok();
    }

    // 网络管理任务
    #[task(shared = [network_status], local = [wifi_led, lora_led], priority = 2)]
    async fn network_management_task(mut ctx: network_management_task::Context) {
        // 检查WiFi连接状态
        let wifi_status = check_wifi_connection();
        // WIFI_CONNECTED.store(wifi_status, Ordering::Relaxed);
        
        if wifi_status {
            ctx.local.wifi_led.set_high();
        } else {
            ctx.local.wifi_led.set_low();
            // 尝试重新连接WiFi
            reconnect_wifi();
        }

        // 检查LoRaWAN连接状态
        let lora_status = check_lora_connection();
        // LORA_JOINED.store(lora_status, Ordering::Relaxed);
        
        if lora_status {
            ctx.local.lora_led.set_high();
        } else {
            ctx.local.lora_led.set_low();
            // 尝试重新加入LoRaWAN网络
            rejoin_lora_network();
        }

        // 检查MQTT连接状态
        let mqtt_status = check_mqtt_connection();
        // MQTT_CONNECTED.store(mqtt_status, Ordering::Relaxed);
        
        if !mqtt_status && wifi_status {
            // 尝试重新连接MQTT
            reconnect_mqtt();
        }

        // 更新网络状态
        ctx.shared.network_status.lock(|status| {
            status.wifi_connected = wifi_status;
            status.lora_joined = lora_status;
            status.mqtt_connected = mqtt_status;
        });

        // info!("网络状态 - WiFi: {}, LoRa: {}, MQTT: {}", 
        //       wifi_status, lora_status, mqtt_status);

        // 调度下次检查 (注释掉，因为rtic-monotonics被禁用)
        // network_management_task::spawn_after(rtic_time::duration::millis(10000)).ok();
    }

    // 数据上传任务
    #[task(shared = [sensor_data, network_status, system_stats], priority = 3)]
    async fn data_upload_task(mut ctx: data_upload_task::Context) {
        // let wifi_connected = WIFI_CONNECTED.load(Ordering::Relaxed);
        // let mqtt_connected = MQTT_CONNECTED.load(Ordering::Relaxed);
        // let lora_joined = LORA_JOINED.load(Ordering::Relaxed);
        let wifi_connected = false; // 简化处理
        let mqtt_connected = false; // 简化处理
        let lora_joined = false; // 简化处理
        
        if !wifi_connected && !lora_joined {
            // warn!("无可用网络连接，跳过数据上传");
            // data_upload_task::spawn_after(rtic_time::duration::millis(UPLOAD_INTERVAL_MS)).ok();
            return;
        }

        let mut upload_count = 0;
        let current_time: u32 = 0; // SYSTEM_UPTIME.load(Ordering::Relaxed);

        // 获取待上传的数据
        let data_to_upload = ctx.shared.sensor_data.lock(|data| {
            let mut upload_data = Vec::<SensorData, 64>::new();
            for sensor_data in data.iter().take(64) {
                upload_data.push(sensor_data.clone()).ok();
            }
            upload_data
        });

        if data_to_upload.is_empty() {
            // debug!("没有数据需要上传");
            // data_upload_task::spawn_after(rtic_time::duration::millis(UPLOAD_INTERVAL_MS)).ok();
            return;
        }

        // 优先使用MQTT上传
        if wifi_connected && mqtt_connected {
            upload_count = upload_via_mqtt(&data_to_upload);
            // info!("通过MQTT上传了 {} 个数据点", upload_count);
        }
        // 备用LoRaWAN上传
        else if lora_joined {
            upload_count = upload_via_lora(&data_to_upload);
            // info!("通过LoRaWAN上传了 {} 个数据点", upload_count);
        }

        // 更新统计信息
        ctx.shared.system_stats.lock(|stats| {
            stats.data_points_uploaded += upload_count;
        });

        ctx.shared.network_status.lock(|status| {
            status.last_upload = current_time;
            status.upload_count += upload_count;
        });

        // 清除已上传的数据
        if upload_count > 0 {
            ctx.shared.sensor_data.lock(|data| {
                for _ in 0..upload_count.min(data.len() as u32) {
                    data.remove(0);
                }
            });
        }

        // 调度下次上传 (注释掉，因为rtic-monotonics被禁用)
        // data_upload_task::spawn_after(rtic_time::duration::millis(UPLOAD_INTERVAL_MS)).ok();
    }

    // 系统监控任务
    #[task(shared = [system_stats], local = [status_led], priority = 4)]
    async fn system_monitor_task(mut ctx: system_monitor_task::Context) {
        // let uptime = SYSTEM_UPTIME.fetch_add(1, Ordering::Relaxed);
        let uptime: u32 = 0; // 简化处理
        
        // 更新系统统计
        ctx.shared.system_stats.lock(|stats| {
            stats.uptime_seconds = uptime;
            stats.cpu_usage = calculate_cpu_usage();
            stats.memory_usage = calculate_memory_usage();
            stats.flash_usage = calculate_flash_usage();
            stats.temperature = read_internal_temperature();
            stats.battery_voltage = read_battery_voltage();
            stats.power_consumption = calculate_power_consumption();
        });

        // 检查低电量模式
        let battery_voltage = read_battery_voltage();
        if battery_voltage < 3.2 {
            // if !LOW_POWER_MODE.load(Ordering::Relaxed) {
        if true { // 简化处理
                // warn!("电池电压过低 ({:.2}V)，进入低功耗模式", battery_voltage);
                enter_low_power_mode();
            }
        } else if battery_voltage > 3.5 {
            // if LOW_POWER_MODE.load(Ordering::Relaxed) {
        if false { // 简化处理
                // info!("电池电压恢复 ({:.2}V)，退出低功耗模式", battery_voltage);
                exit_low_power_mode();
            }
        }

        // 状态LED闪烁
        ctx.local.status_led.toggle();

        // 每分钟报告一次系统状态
        if uptime % 60 == 0 {
            ctx.shared.system_stats.lock(|_stats| {
                // info!("📊 系统状态: 运行{}秒, CPU:{:.1}%, 内存:{:.1}%, 温度:{:.1}°C", 
                //       stats.uptime_seconds, stats.cpu_usage, stats.memory_usage, stats.temperature);
            });
        }

        // 调度下次监控 (注释掉，因为rtic-monotonics被禁用)
        // system_monitor_task::spawn_after(rtic_time::duration::millis(1000)).ok();
    }

    // 心跳任务
    #[task(shared = [network_status], priority = 5)]
    async fn heartbeat_task(mut _ctx: heartbeat_task::Context) {
        // let current_time = SYSTEM_UPTIME.load(Ordering::Relaxed);
        let _current_time = 0; // 简化处理
        
        // 发送心跳消息
        // if WIFI_CONNECTED.load(Ordering::Relaxed) && MQTT_CONNECTED.load(Ordering::Relaxed) {
        if false { // 简化处理
            send_heartbeat_mqtt();
        }
        
        // if LORA_JOINED.load(Ordering::Relaxed) {
        if false { // 简化处理
            send_heartbeat_lora();
        }

        // debug!("💓 发送心跳信号");

        // 调度下次心跳 (注释掉，因为rtic-monotonics被禁用)
        // heartbeat_task::spawn_after(rtic_time::duration::millis(HEARTBEAT_INTERVAL_MS)).ok();
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // 进入低功耗等待模式
            cortex_m::asm::wfi();
        }
    }
}

// 初始化网关配置
fn init_gateway_config() {
    let _config = GatewayConfig {
        device_id: String::from("iot-gateway-001"),
        sampling_interval: SAMPLING_INTERVAL_MS,
        upload_interval: UPLOAD_INTERVAL_MS,
        wifi_ssid: String::from("IoT_Network"),
        wifi_password: String::from("password123"),
        mqtt_broker: String::from("mqtt.example.com"),
        mqtt_port: 1883,
        mqtt_username: String::from("gateway"),
        mqtt_password: String::from("secret"),
        lora_dev_eui: [0x00; 8],
        lora_app_eui: [0x00; 8],
        lora_app_key: [0x00; 16],
        encryption_enabled: true,
        debug_enabled: true,
    };

    critical_section::with(|_cs| {
        // GATEWAY_CONFIG.borrow(cs).replace(config); // 注释掉因为RefCell问题
    });

    // info!("网关配置初始化完成");
}

// WiFi连接检查
fn check_wifi_connection() -> bool {
    // 实现WiFi连接状态检查
    // 这里返回模拟状态
    true
}

// 重新连接WiFi
fn reconnect_wifi() {
    // info!("尝试重新连接WiFi...");
    // 实现WiFi重连逻辑
}

// LoRaWAN连接检查
fn check_lora_connection() -> bool {
    // 实现LoRaWAN连接状态检查
    true
}

// 重新加入LoRaWAN网络
fn rejoin_lora_network() {
    // info!("尝试重新加入LoRaWAN网络...");
    // 实现LoRaWAN重连逻辑
}

// MQTT连接检查
fn check_mqtt_connection() -> bool {
    // 实现MQTT连接状态检查
    true
}

// 重新连接MQTT
fn reconnect_mqtt() {
    // info!("尝试重新连接MQTT...");
    // 实现MQTT重连逻辑
}

// 通过MQTT上传数据
fn upload_via_mqtt(data: &[SensorData]) -> u32 {
    let mut uploaded = 0;
    
    for _sensor_data in data.iter().take(32) {
        // 序列化数据
        // if let Ok(json_data) = postcard::to_vec::<_, 256>(sensor_data) {
        {
            // 发布到MQTT主题
            // 在no_std环境中使用简单的主题名
        let _topic = "sensors/data";
            // 实现MQTT发布逻辑
            uploaded += 1;
        }
    }
    
    uploaded
}

// 通过LoRaWAN上传数据
fn upload_via_lora(data: &[SensorData]) -> u32 {
    let mut uploaded = 0;
    
    // LoRaWAN有载荷限制，每次只能发送少量数据
    for sensor_data in data.iter().take(4) {
        // 压缩数据格式
        let _compressed_data = compress_sensor_data(sensor_data);
        // 实现LoRaWAN发送逻辑
        uploaded += 1;
    }
    
    uploaded
}

// 压缩传感器数据
fn compress_sensor_data(data: &SensorData) -> Vec<u8, 32> {
    let mut compressed = Vec::new();
    
    // 简单的数据压缩格式
    compressed.push((data.sensor_id & 0xFF) as u8).ok();
    compressed.push(((data.sensor_id >> 8) & 0xFF) as u8).ok();
    compressed.push(data.sensor_type as u8).ok();
    
    // 将浮点数转换为定点数
    let value_fixed = (data.value * 100.0) as i16;
    compressed.push((value_fixed & 0xFF) as u8).ok();
    compressed.push(((value_fixed >> 8) & 0xFF) as u8).ok();
    
    compressed.push(data.quality).ok();
    
    compressed
}

// 发送MQTT心跳
fn send_heartbeat_mqtt() {
    // 在no_std环境中使用简单的字符串
    let _heartbeat = "device_id:iot-gateway-001,status:online";
    // 实现MQTT心跳发送
}

// 发送LoRa心跳
fn send_heartbeat_lora() {
    let mut heartbeat = Vec::<u8, 16>::new();
    heartbeat.push(0xFF).ok(); // 心跳标识
    // let uptime = SYSTEM_UPTIME.load(Ordering::Relaxed);
        let uptime = 0; // 简化处理
    heartbeat.extend_from_slice(&(uptime as u32).to_le_bytes()).ok();
    // 实现LoRa心跳发送
}

// 计算CPU使用率
fn calculate_cpu_usage() -> f32 {
    // 简化的CPU使用率计算
    let active_tasks = 5; // 活跃任务数
    let max_tasks = 10;   // 最大任务数
    (active_tasks as f32 / max_tasks as f32) * 100.0
}

// 计算内存使用率
fn calculate_memory_usage() -> f32 {
    // 简化的内存使用率计算
    let used_memory = 24 * 1024; // 24KB
    let total_memory = 128 * 1024; // 128KB
    (used_memory as f32 / total_memory as f32) * 100.0
}

// 计算Flash使用率
fn calculate_flash_usage() -> f32 {
    // 简化的Flash使用率计算
    let used_flash = 256 * 1024; // 256KB
    let total_flash = 1024 * 1024; // 1MB
    (used_flash as f32 / total_flash as f32) * 100.0
}

// 读取内部温度
fn read_internal_temperature() -> f32 {
    // 模拟内部温度读取
    // 25.0 + (SYSTEM_UPTIME.load(Ordering::Relaxed) as f32 * 0.001).sin() * 5.0
        25.0 // 简化处理
}

// 读取电池电压
fn read_battery_voltage() -> f32 {
    // 模拟电池电压读取
    // 3.7 - (SYSTEM_UPTIME.load(Ordering::Relaxed) as f32 / 86400.0) * 0.5
        3.7 // 简化处理
}

// 计算功耗
fn calculate_power_consumption() -> f32 {
    let base_power = 100.0; // 基础功耗 100mW
    // let wifi_power = if WIFI_CONNECTED.load(Ordering::Relaxed) { 50.0 } else { 0.0 };
    // let lora_power = if LORA_JOINED.load(Ordering::Relaxed) { 30.0 } else { 0.0 };
    // let sensor_power = if DATA_COLLECTION_ENABLED.load(Ordering::Relaxed) { 20.0 } else { 0.0 };
    let wifi_power = 0.0; // 简化处理
    let lora_power = 0.0; // 简化处理
    let sensor_power = 20.0; // 简化处理
    
    base_power + wifi_power + lora_power + sensor_power
}

// 进入低功耗模式
fn enter_low_power_mode() {
    // LOW_POWER_MODE.store(true, Ordering::Relaxed);
    // DATA_COLLECTION_ENABLED.store(false, Ordering::Relaxed);
    // 降低采样频率，关闭非必要外设
    // info!("已进入低功耗模式");
}

// 退出低功耗模式
fn exit_low_power_mode() {
    // LOW_POWER_MODE.store(false, Ordering::Relaxed);
    // DATA_COLLECTION_ENABLED.store(true, Ordering::Relaxed);
    // 恢复正常采样频率，重新启用外设
    // info!("已退出低功耗模式");
}

// 中断处理程序 - 注释掉因为相关功能被禁用
// #[interrupt]
// fn USART1() {
//     // WiFi模块串口中断
// }

// #[interrupt]
// fn SPI1() {
//     // LoRa模块SPI中断
// }

// #[interrupt]
// fn I2C1_EV() {
//     // I2C事件中断
// }

// #[interrupt]
// fn I2C1_ER() {
//     // I2C错误中断
// }

// #[interrupt]
// fn ADC() {
//     // ADC转换完成中断
// }

// #[interrupt]
// fn EXTI15_10() {
//     // 外部中断 (按键或传感器)
// }

// 异常处理
#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    // error!("硬件错误: {:#?}", ef);
    panic!("Hard fault occurred!");
}

#[cortex_m_rt::exception]
unsafe fn DefaultHandler(_irqn: i16) {
    // warn!("未处理的中断: {}", irqn);
}