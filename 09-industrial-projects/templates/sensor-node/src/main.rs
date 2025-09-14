//! 传感器节点模板
//!
//! 这个模板提供了一个通用的传感器节点框架，支持：
//! - 多种传感器接口（I2C、SPI、ADC等）
//! - 数据采集和处理
//! - 无线通信（LoRa、WiFi、蓝牙等）
//! - 低功耗管理
//! - 数据缓存和传输

#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use stm32f4xx_hal::{prelude::*, pac, i2c::I2c, spi::Spi, adc::Adc};
use heapless::{Vec, FnvIndexMap};
use serde::{Serialize, Deserialize};
use fugit::RateExtU32;
use libm::{sinf, cosf};

/// 传感器类型枚举
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
enum SensorType {
    Temperature,
    Humidity,
    Pressure,
    Light,
    Motion,
    Gas,
    Voltage,
    Current,
}

/// 传感器读数
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SensorReading {
    sensor_type: SensorType,
    value: f32,
    unit: &'static str,
    timestamp: u32,
    quality: DataQuality,
}

/// 数据质量指示
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
enum DataQuality {
    Good,
    Uncertain,
    Bad,
}

/// 传感器配置
#[derive(Debug, Clone)]
struct SensorConfig {
    sensor_type: SensorType,
    sample_rate: u32,  // Hz
    enabled: bool,
    calibration_offset: f32,
    calibration_scale: f32,
}

/// 通信配置
#[derive(Debug, Clone)]
struct CommunicationConfig {
    transmission_interval: u32,  // 秒
    max_retries: u8,
    power_level: u8,
}

/// 系统状态
#[derive(Debug, Clone, Copy)]
enum SystemState {
    Initializing,
    Sampling,
    Transmitting,
    Sleeping,
    Error,
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {
    use super::*;
    use systick_monotonic::{Systick, ExtU32};
    
    #[shared]
    struct Shared {
        sensor_readings: Vec<SensorReading, 64>,
        system_state: SystemState,
        error_count: u32,
    }
    
    #[local]
    struct Local {
        sensor_configs: Vec<SensorConfig, 8>,
        comm_config: CommunicationConfig,
        led: stm32f4xx_hal::gpio::gpioc::PC13<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>,
        timestamp_counter: u32,
    }
    
    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;
    
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;
        let cp = ctx.core;
        
        // 配置时钟
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
        
        // 配置GPIO
        let gpioc = dp.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output();
        
        // 初始化系统定时器
        let mono = Systick::new(cp.SYST, 84_000_000);
        
        // 初始化传感器配置
        let mut sensor_configs = Vec::new();
        
        // 添加默认传感器配置
        let _ = sensor_configs.push(SensorConfig {
            sensor_type: SensorType::Temperature,
            sample_rate: 1, // 1 Hz
            enabled: true,
            calibration_offset: 0.0,
            calibration_scale: 1.0,
        });
        
        let _ = sensor_configs.push(SensorConfig {
            sensor_type: SensorType::Humidity,
            sample_rate: 1, // 1 Hz
            enabled: true,
            calibration_offset: 0.0,
            calibration_scale: 1.0,
        });
        
        // 通信配置
        let comm_config = CommunicationConfig {
            transmission_interval: 60, // 每60秒传输一次
            max_retries: 3,
            power_level: 5,
        };
        
        // 启动定期任务
        sensor_sampling::spawn().ok();
        data_transmission::spawn().ok();
        system_monitor::spawn().ok();
        
        (
            Shared {
                sensor_readings: Vec::new(),
                system_state: SystemState::Initializing,
                error_count: 0,
            },
            Local {
                sensor_configs,
                comm_config,
                led,
                timestamp_counter: 0,
            },
        )
    }
    
    /// 传感器采样任务
    #[task(shared = [sensor_readings, system_state], local = [sensor_configs, timestamp_counter])]
    async fn sensor_sampling(mut ctx: sensor_sampling::Context) {
        ctx.shared.system_state.lock(|state| *state = SystemState::Sampling);
        
        // 更新时间戳
        *ctx.local.timestamp_counter += 1;
        let timestamp = *ctx.local.timestamp_counter;
        
        // 遍历所有启用的传感器
        for config in ctx.local.sensor_configs.iter().filter(|c| c.enabled) {
            let reading = match config.sensor_type {
                SensorType::Temperature => read_temperature_sensor(config, timestamp),
                SensorType::Humidity => read_humidity_sensor(config, timestamp),
                SensorType::Pressure => read_pressure_sensor(config, timestamp),
                SensorType::Light => read_light_sensor(config, timestamp),
                SensorType::Motion => read_motion_sensor(config, timestamp),
                SensorType::Gas => read_gas_sensor(config, timestamp),
                SensorType::Voltage => read_voltage_sensor(config, timestamp),
                SensorType::Current => read_current_sensor(config, timestamp),
            };
            
            // 存储读数
            ctx.shared.sensor_readings.lock(|readings| {
                if readings.push(reading).is_err() {
                    // 缓冲区满，移除最旧的读数
                    readings.remove(0);
                    let _ = readings.push(reading);
                }
            });
        }
        
        // 调度下一次采样
        sensor_sampling::spawn_after(1.secs()).ok();
    }
    
    /// 数据传输任务
    #[task(shared = [sensor_readings, system_state], local = [comm_config])]
    async fn data_transmission(mut ctx: data_transmission::Context) {
        ctx.shared.system_state.lock(|state| *state = SystemState::Transmitting);
        
        let readings_to_send = ctx.shared.sensor_readings.lock(|readings| {
            let data = readings.clone();
            readings.clear();
            data
        });
        
        if !readings_to_send.is_empty() {
            // 序列化数据
            if let Ok(serialized) = postcard::to_vec::<_, 512>(&readings_to_send) {
                // 发送数据（这里是模拟实现）
                transmit_data(&serialized, ctx.local.comm_config);
            }
        }
        
        // 调度下一次传输
        let interval = ctx.local.comm_config.transmission_interval;
        data_transmission::spawn_after((interval as u64).secs()).ok();
    }
    
    /// 系统监控任务
    #[task(shared = [system_state, error_count], local = [led])]
    async fn system_monitor(mut ctx: system_monitor::Context) {
        let state = ctx.shared.system_state.lock(|s| *s);
        let error_count = ctx.shared.error_count.lock(|c| *c);
        
        // 根据系统状态控制LED
        match state {
            SystemState::Initializing => {
                // 快速闪烁
                ctx.local.led.toggle();
            }
            SystemState::Sampling | SystemState::Transmitting => {
                // 慢速闪烁
                ctx.local.led.set_high();
            }
            SystemState::Sleeping => {
                // LED关闭
                ctx.local.led.set_low();
            }
            SystemState::Error => {
                // 持续亮起
                ctx.local.led.set_high();
            }
        }
        
        // 检查错误计数
        if error_count > 10 {
            ctx.shared.system_state.lock(|s| *s = SystemState::Error);
        }
        
        // 调度下一次监控
        system_monitor::spawn_after(500.millis()).ok();
    }
}

// 传感器读取函数（模拟实现）
fn read_temperature_sensor(config: &SensorConfig, timestamp: u32) -> SensorReading {
    // 模拟温度读数 (20-30°C)
    let raw_value = 25.0 + sinf(timestamp as f32 * 0.1) * 5.0;
    let calibrated_value = raw_value * config.calibration_scale + config.calibration_offset;
    
    SensorReading {
        sensor_type: SensorType::Temperature,
        value: calibrated_value,
        unit: "°C",
        timestamp,
        quality: DataQuality::Good,
    }
}

fn read_humidity_sensor(config: &SensorConfig, timestamp: u32) -> SensorReading {
    // 模拟湿度读数 (40-80%)
    let raw_value = 60.0 + cosf(timestamp as f32 * 0.05) * 20.0;
    let calibrated_value = raw_value * config.calibration_scale + config.calibration_offset;
    
    SensorReading {
        sensor_type: SensorType::Humidity,
        value: calibrated_value,
        unit: "%RH",
        timestamp,
        quality: DataQuality::Good,
    }
}

fn read_pressure_sensor(config: &SensorConfig, timestamp: u32) -> SensorReading {
    // 模拟压力读数 (1000-1020 hPa)
    let raw_value = 1013.25 + sinf(timestamp as f32 * 0.02) * 7.0;
    let calibrated_value = raw_value * config.calibration_scale + config.calibration_offset;
    
    SensorReading {
        sensor_type: SensorType::Pressure,
        value: calibrated_value,
        unit: "hPa",
        timestamp,
        quality: DataQuality::Good,
    }
}

fn read_light_sensor(config: &SensorConfig, timestamp: u32) -> SensorReading {
    // 模拟光照读数 (0-1000 lux)
    let raw_value: f32 = 500.0 + sinf(timestamp as f32 * 0.1) * 400.0;
    let calibrated_value = raw_value.max(0.0) * config.calibration_scale + config.calibration_offset;
    
    SensorReading {
        sensor_type: SensorType::Light,
        value: calibrated_value,
        unit: "lux",
        timestamp,
        quality: DataQuality::Good,
    }
}

fn read_motion_sensor(config: &SensorConfig, timestamp: u32) -> SensorReading {
    // 模拟运动检测 (0或1)
    let raw_value = if (timestamp % 10) < 3 { 1.0 } else { 0.0 };
    let calibrated_value = raw_value * config.calibration_scale + config.calibration_offset;
    
    SensorReading {
        sensor_type: SensorType::Motion,
        value: calibrated_value,
        unit: "bool",
        timestamp,
        quality: DataQuality::Good,
    }
}

fn read_gas_sensor(config: &SensorConfig, timestamp: u32) -> SensorReading {
    // 模拟气体浓度读数 (0-500 ppm)
    let raw_value: f32 = 100.0 + sinf(timestamp as f32 * 0.03) * 50.0;
    let calibrated_value = raw_value.max(0.0) * config.calibration_scale + config.calibration_offset;
    
    SensorReading {
        sensor_type: SensorType::Gas,
        value: calibrated_value,
        unit: "ppm",
        timestamp,
        quality: DataQuality::Good,
    }
}

fn read_voltage_sensor(config: &SensorConfig, timestamp: u32) -> SensorReading {
    // 模拟电压读数 (3.0-3.6V)
    let raw_value = 3.3 + sinf(timestamp as f32 * 0.01) * 0.3;
    let calibrated_value = raw_value * config.calibration_scale + config.calibration_offset;
    
    SensorReading {
        sensor_type: SensorType::Voltage,
        value: calibrated_value,
        unit: "V",
        timestamp,
        quality: DataQuality::Good,
    }
}

fn read_current_sensor(config: &SensorConfig, timestamp: u32) -> SensorReading {
    // 模拟电流读数 (0-100 mA)
    let raw_value: f32 = 50.0 + cosf(timestamp as f32 * 0.05) * 30.0;
    let calibrated_value = raw_value.max(0.0) * config.calibration_scale + config.calibration_offset;
    
    SensorReading {
        sensor_type: SensorType::Current,
        value: calibrated_value,
        unit: "mA",
        timestamp,
        quality: DataQuality::Good,
    }
}

/// 数据传输函数（模拟实现）
fn transmit_data(data: &[u8], _config: &CommunicationConfig) {
    // 这里应该实现实际的无线传输逻辑
    // 例如：LoRa、WiFi、蓝牙等
    
    // 模拟传输延时
    cortex_m::asm::delay(1000);
    
    // 在实际实现中，这里会：
    // 1. 配置无线模块
    // 2. 建立连接
    // 3. 发送数据
    // 4. 等待确认
    // 5. 处理重传逻辑
}