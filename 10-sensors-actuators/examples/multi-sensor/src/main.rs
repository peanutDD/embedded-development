#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    gpio::{Pin, Output, PushPull, Input, PullUp, Alternate, AF4},
    serial::{Serial, Config},
    timer::Timer,
    delay::Delay,
    i2c::I2c,
    spi::Spi,
    adc::{Adc, config::AdcConfig},
};

use heapless::{Vec, String, FnvIndexMap};
use nb::block;
use serde::{Serialize, Deserialize};

// 传感器数据类型
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SensorReading {
    pub sensor_id: u8,
    pub sensor_type: SensorType,
    pub timestamp: u32,
    pub value: f32,
    pub unit: Unit,
    pub quality: DataQuality,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum SensorType {
    Temperature,
    Humidity,
    Pressure,
    Light,
    Acceleration,
    Gyroscope,
    Magnetometer,
    Voltage,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Unit {
    Celsius,
    Fahrenheit,
    Percent,
    Pascal,
    Lux,
    MeterPerSecondSquared,
    DegreesPerSecond,
    Gauss,
    Volt,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum DataQuality {
    Excellent,
    Good,
    Fair,
    Poor,
    Invalid,
}

// 传感器接口抽象
pub trait Sensor {
    type Error;
    
    fn initialize(&mut self) -> Result<(), Self::Error>;
    fn read(&mut self) -> Result<Vec<SensorReading, 8>, Self::Error>;
    fn get_sensor_info(&self) -> SensorInfo;
    fn set_configuration(&mut self, config: &SensorConfig) -> Result<(), Self::Error>;
    fn get_status(&mut self) -> Result<SensorStatus, Self::Error>;
}

#[derive(Debug, Clone)]
pub struct SensorInfo {
    pub id: u8,
    pub name: &'static str,
    pub sensor_types: Vec<SensorType, 8>,
    pub sample_rate_hz: f32,
    pub resolution: u16,
    pub range: (f32, f32),
}

#[derive(Debug, Clone)]
pub struct SensorConfig {
    pub sample_rate: u32,
    pub resolution: u8,
    pub filter_enabled: bool,
    pub calibration_enabled: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct SensorStatus {
    pub is_online: bool,
    pub last_error: Option<u8>,
    pub sample_count: u32,
    pub error_count: u32,
}

// BMP280 环境传感器实现
pub struct BMP280Sensor<I2C> {
    i2c: I2C,
    address: u8,
    info: SensorInfo,
    calibration: BMP280Calibration,
    config: SensorConfig,
    status: SensorStatus,
}

#[derive(Default)]
struct BMP280Calibration {
    dig_t1: u16, dig_t2: i16, dig_t3: i16,
    dig_p1: u16, dig_p2: i16, dig_p3: i16,
    dig_p4: i16, dig_p5: i16, dig_p6: i16,
    dig_p7: i16, dig_p8: i16, dig_p9: i16,
}

impl<I2C> BMP280Sensor<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
    pub fn new(i2c: I2C, sensor_id: u8) -> Self {
        let mut sensor_types = Vec::new();
        sensor_types.push(SensorType::Temperature).ok();
        sensor_types.push(SensorType::Pressure).ok();
        
        let info = SensorInfo {
            id: sensor_id,
            name: "BMP280",
            sensor_types,
            sample_rate_hz: 1.0,
            resolution: 20,
            range: (-40.0, 85.0),
        };
        
        Self {
            i2c,
            address: 0x76,
            info,
            calibration: BMP280Calibration::default(),
            config: SensorConfig {
                sample_rate: 1,
                resolution: 16,
                filter_enabled: true,
                calibration_enabled: true,
            },
            status: SensorStatus {
                is_online: false,
                last_error: None,
                sample_count: 0,
                error_count: 0,
            },
        }
    }
    
    fn read_register(&mut self, reg: u8) -> Result<u8, u8> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buffer)
            .map_err(|_| 1)?;
        Ok(buffer[0])
    }
    
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), u8> {
        self.i2c.write(self.address, &[reg, value])
            .map_err(|_| 2)
    }
    
    fn read_calibration(&mut self) -> Result<(), u8> {
        let mut buffer = [0u8; 24];
        self.i2c.write_read(self.address, &[0x88], &mut buffer)
            .map_err(|_| 3)?;
        
        self.calibration.dig_t1 = ((buffer[1] as u16) << 8) | (buffer[0] as u16);
        self.calibration.dig_t2 = ((buffer[3] as i16) << 8) | (buffer[2] as i16);
        self.calibration.dig_t3 = ((buffer[5] as i16) << 8) | (buffer[4] as i16);
        
        self.calibration.dig_p1 = ((buffer[7] as u16) << 8) | (buffer[6] as u16);
        self.calibration.dig_p2 = ((buffer[9] as i16) << 8) | (buffer[8] as i16);
        self.calibration.dig_p3 = ((buffer[11] as i16) << 8) | (buffer[10] as i16);
        self.calibration.dig_p4 = ((buffer[13] as i16) << 8) | (buffer[12] as i16);
        self.calibration.dig_p5 = ((buffer[15] as i16) << 8) | (buffer[14] as i16);
        self.calibration.dig_p6 = ((buffer[17] as i16) << 8) | (buffer[16] as i16);
        self.calibration.dig_p7 = ((buffer[19] as i16) << 8) | (buffer[18] as i16);
        self.calibration.dig_p8 = ((buffer[21] as i16) << 8) | (buffer[20] as i16);
        self.calibration.dig_p9 = ((buffer[23] as i16) << 8) | (buffer[22] as i16);
        
        Ok(())
    }
    
    fn compensate_temperature(&self, raw_temp: i32) -> (f32, i32) {
        let var1 = (((raw_temp >> 3) - ((self.calibration.dig_t1 as i32) << 1)) * 
                   (self.calibration.dig_t2 as i32)) >> 11;
        
        let var2 = (((((raw_temp >> 4) - (self.calibration.dig_t1 as i32)) * 
                     ((raw_temp >> 4) - (self.calibration.dig_t1 as i32))) >> 12) * 
                   (self.calibration.dig_t3 as i32)) >> 14;
        
        let t_fine = var1 + var2;
        let temperature = (t_fine * 5 + 128) >> 8;
        
        (temperature as f32 / 100.0, t_fine)
    }
    
    fn compensate_pressure(&self, raw_pressure: i32, t_fine: i32) -> f32 {
        let mut var1 = (t_fine as i64) - 128000;
        let mut var2 = var1 * var1 * (self.calibration.dig_p6 as i64);
        var2 = var2 + ((var1 * (self.calibration.dig_p5 as i64)) << 17);
        var2 = var2 + (((self.calibration.dig_p4 as i64) << 35));
        var1 = ((var1 * var1 * (self.calibration.dig_p3 as i64)) >> 8) + 
               ((var1 * (self.calibration.dig_p2 as i64)) << 12);
        var1 = (((1i64 << 47) + var1)) * (self.calibration.dig_p1 as i64) >> 33;
        
        if var1 == 0 {
            return 0.0;
        }
        
        let mut p = 1048576 - raw_pressure as i64;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = ((self.calibration.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        var2 = ((self.calibration.dig_p8 as i64) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + ((self.calibration.dig_p7 as i64) << 4);
        
        (p as f32) / 256.0
    }
}

impl<I2C> Sensor for BMP280Sensor<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
    type Error = u8;
    
    fn initialize(&mut self) -> Result<(), Self::Error> {
        // 验证芯片ID
        let chip_id = self.read_register(0xD0)?;
        if chip_id != 0x58 {
            self.status.last_error = Some(4);
            return Err(4);
        }
        
        // 读取校准数据
        self.read_calibration()?;
        
        // 配置传感器
        self.write_register(0xF4, 0x27)?; // 温度和压力过采样
        self.write_register(0xF5, 0xA0)?; // 配置寄存器
        
        self.status.is_online = true;
        Ok(())
    }
    
    fn read(&mut self) -> Result<Vec<SensorReading, 8>, Self::Error> {
        let mut readings = Vec::new();
        
        // 读取原始温度数据
        let mut temp_buffer = [0u8; 3];
        self.i2c.write_read(self.address, &[0xFA], &mut temp_buffer)
            .map_err(|_| 5)?;
        
        let raw_temp = ((temp_buffer[0] as i32) << 12) | 
                       ((temp_buffer[1] as i32) << 4) | 
                       ((temp_buffer[2] as i32) >> 4);
        
        // 读取原始压力数据
        let mut press_buffer = [0u8; 3];
        self.i2c.write_read(self.address, &[0xF7], &mut press_buffer)
            .map_err(|_| 6)?;
        
        let raw_pressure = ((press_buffer[0] as i32) << 12) | 
                          ((press_buffer[1] as i32) << 4) | 
                          ((press_buffer[2] as i32) >> 4);
        
        // 补偿计算
        let (temperature, t_fine) = self.compensate_temperature(raw_temp);
        let pressure = self.compensate_pressure(raw_pressure, t_fine);
        
        // 创建温度读数
        let temp_reading = SensorReading {
            sensor_id: self.info.id,
            sensor_type: SensorType::Temperature,
            timestamp: 0, // 将由调用者设置
            value: temperature,
            unit: Unit::Celsius,
            quality: DataQuality::Good,
        };
        
        // 创建压力读数
        let pressure_reading = SensorReading {
            sensor_id: self.info.id,
            sensor_type: SensorType::Pressure,
            timestamp: 0,
            value: pressure,
            unit: Unit::Pascal,
            quality: DataQuality::Good,
        };
        
        readings.push(temp_reading).map_err(|_| 7)?;
        readings.push(pressure_reading).map_err(|_| 8)?;
        
        self.status.sample_count += 1;
        Ok(readings)
    }
    
    fn get_sensor_info(&self) -> SensorInfo {
        self.info.clone()
    }
    
    fn set_configuration(&mut self, config: &SensorConfig) -> Result<(), Self::Error> {
        self.config = config.clone();
        
        // 根据配置更新寄存器
        let ctrl_meas = match config.resolution {
            16 => 0x27, // 标准分辨率
            20 => 0x57, // 高分辨率
            _ => 0x27,
        };
        
        self.write_register(0xF4, ctrl_meas)?;
        
        Ok(())
    }
    
    fn get_status(&mut self) -> Result<SensorStatus, Self::Error> {
        Ok(self.status)
    }
}

// MPU6050 惯性测量单元
pub struct MPU6050Sensor<I2C> {
    i2c: I2C,
    address: u8,
    info: SensorInfo,
    config: SensorConfig,
    status: SensorStatus,
}

impl<I2C> MPU6050Sensor<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
    pub fn new(i2c: I2C, sensor_id: u8) -> Self {
        let mut sensor_types = Vec::new();
        sensor_types.push(SensorType::Acceleration).ok();
        sensor_types.push(SensorType::Gyroscope).ok();
        sensor_types.push(SensorType::Temperature).ok();
        
        let info = SensorInfo {
            id: sensor_id,
            name: "MPU6050",
            sensor_types,
            sample_rate_hz: 100.0,
            resolution: 16,
            range: (-16.0, 16.0),
        };
        
        Self {
            i2c,
            address: 0x68,
            info,
            config: SensorConfig {
                sample_rate: 100,
                resolution: 16,
                filter_enabled: true,
                calibration_enabled: true,
            },
            status: SensorStatus {
                is_online: false,
                last_error: None,
                sample_count: 0,
                error_count: 0,
            },
        }
    }
    
    fn read_register(&mut self, reg: u8) -> Result<u8, u8> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buffer)
            .map_err(|_| 1)?;
        Ok(buffer[0])
    }
    
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), u8> {
        self.i2c.write(self.address, &[reg, value])
            .map_err(|_| 2)
    }
    
    fn read_i16(&mut self, reg: u8) -> Result<i16, u8> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(self.address, &[reg], &mut buffer)
            .map_err(|_| 3)?;
        
        Ok(((buffer[0] as i16) << 8) | (buffer[1] as i16))
    }
}

impl<I2C> Sensor for MPU6050Sensor<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
    type Error = u8;
    
    fn initialize(&mut self) -> Result<(), Self::Error> {
        // 验证芯片ID
        let who_am_i = self.read_register(0x75)?;
        if who_am_i != 0x68 {
            self.status.last_error = Some(4);
            return Err(4);
        }
        
        // 复位设备
        self.write_register(0x6B, 0x80)?;
        
        // 等待复位完成
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
        
        // 配置电源管理
        self.write_register(0x6B, 0x01)?; // 使用X轴陀螺仪作为时钟源
        
        // 配置采样率
        self.write_register(0x19, 0x07)?; // 采样率分频器
        
        // 配置数字低通滤波器
        self.write_register(0x1A, 0x06)?; // 5Hz低通滤波器
        
        // 配置陀螺仪量程 (±250°/s)
        self.write_register(0x1B, 0x00)?;
        
        // 配置加速度计量程 (±2g)
        self.write_register(0x1C, 0x00)?;
        
        self.status.is_online = true;
        Ok(())
    }
    
    fn read(&mut self) -> Result<Vec<SensorReading, 8>, Self::Error> {
        let mut readings = Vec::new();
        
        // 读取加速度数据
        let accel_x = self.read_i16(0x3B)? as f32 / 16384.0; // ±2g量程
        let accel_y = self.read_i16(0x3D)? as f32 / 16384.0;
        let accel_z = self.read_i16(0x3F)? as f32 / 16384.0;
        
        // 读取温度数据
        let temp_raw = self.read_i16(0x41)?;
        let temperature = (temp_raw as f32) / 340.0 + 36.53;
        
        // 读取陀螺仪数据
        let gyro_x = self.read_i16(0x43)? as f32 / 131.0; // ±250°/s量程
        let gyro_y = self.read_i16(0x45)? as f32 / 131.0;
        let gyro_z = self.read_i16(0x47)? as f32 / 131.0;
        
        // 创建读数
        let readings_data = [
            (SensorType::Acceleration, accel_x, Unit::MeterPerSecondSquared),
            (SensorType::Acceleration, accel_y, Unit::MeterPerSecondSquared),
            (SensorType::Acceleration, accel_z, Unit::MeterPerSecondSquared),
            (SensorType::Temperature, temperature, Unit::Celsius),
            (SensorType::Gyroscope, gyro_x, Unit::DegreesPerSecond),
            (SensorType::Gyroscope, gyro_y, Unit::DegreesPerSecond),
            (SensorType::Gyroscope, gyro_z, Unit::DegreesPerSecond),
        ];
        
        for (sensor_type, value, unit) in readings_data.iter() {
            let reading = SensorReading {
                sensor_id: self.info.id,
                sensor_type: *sensor_type,
                timestamp: 0,
                value: *value,
                unit: *unit,
                quality: DataQuality::Good,
            };
            
            readings.push(reading).map_err(|_| 5)?;
        }
        
        self.status.sample_count += 1;
        Ok(readings)
    }
    
    fn get_sensor_info(&self) -> SensorInfo {
        self.info.clone()
    }
    
    fn set_configuration(&mut self, config: &SensorConfig) -> Result<(), Self::Error> {
        self.config = config.clone();
        
        // 根据采样率配置分频器
        let sample_rate_div = (1000 / config.sample_rate).saturating_sub(1) as u8;
        self.write_register(0x19, sample_rate_div)?;
        
        Ok(())
    }
    
    fn get_status(&mut self) -> Result<SensorStatus, Self::Error> {
        Ok(self.status)
    }
}

// 数据融合和处理
pub struct SensorFusion {
    readings_buffer: Vec<SensorReading, 256>,
    filters: FnvIndexMap<u8, KalmanFilter, 16>,
    calibration_data: FnvIndexMap<u8, CalibrationData, 16>,
}

pub struct KalmanFilter {
    q: f32,  // 过程噪声协方差
    r: f32,  // 测量噪声协方差
    x: f32,  // 状态估计
    p: f32,  // 估计误差协方差
    k: f32,  // 卡尔曼增益
}

impl KalmanFilter {
    pub fn new(q: f32, r: f32, initial_value: f32) -> Self {
        Self {
            q,
            r,
            x: initial_value,
            p: 1.0,
            k: 0.0,
        }
    }
    
    pub fn update(&mut self, measurement: f32) -> f32 {
        // 预测步骤
        self.p += self.q;
        
        // 更新步骤
        self.k = self.p / (self.p + self.r);
        self.x += self.k * (measurement - self.x);
        self.p *= 1.0 - self.k;
        
        self.x
    }
}

#[derive(Clone)]
pub struct CalibrationData {
    pub offset: f32,
    pub scale: f32,
    pub temperature_coefficient: f32,
}

impl CalibrationData {
    pub fn apply(&self, value: f32, temperature: f32) -> f32 {
        let temp_corrected = value + self.temperature_coefficient * (temperature - 25.0);
        (temp_corrected + self.offset) * self.scale
    }
}

impl SensorFusion {
    pub fn new() -> Self {
        Self {
            readings_buffer: Vec::new(),
            filters: FnvIndexMap::new(),
            calibration_data: FnvIndexMap::new(),
        }
    }
    
    pub fn add_reading(&mut self, mut reading: SensorReading) -> Result<(), &'static str> {
        // 应用校准
        if let Some(cal_data) = self.calibration_data.get(&reading.sensor_id) {
            // 假设温度为25°C (实际应用中应该从温度传感器获取)
            reading.value = cal_data.apply(reading.value, 25.0);
        }
        
        // 应用卡尔曼滤波
        let filter_key = (reading.sensor_id << 4) | (reading.sensor_type as u8);
        if let Some(filter) = self.filters.get_mut(&filter_key) {
            reading.value = filter.update(reading.value);
        } else {
            // 创建新的滤波器
            let filter = KalmanFilter::new(0.01, 0.1, reading.value);
            self.filters.insert(filter_key, filter).map_err(|_| "Filter map full")?;
        }
        
        // 添加到缓冲区
        if self.readings_buffer.is_full() {
            self.readings_buffer.remove(0);
        }
        
        self.readings_buffer.push(reading).map_err(|_| "Buffer full")?;
        
        Ok(())
    }
    
    pub fn get_latest_readings(&self, sensor_type: SensorType) -> Vec<&SensorReading, 16> {
        let mut results = Vec::new();
        
        for reading in self.readings_buffer.iter().rev() {
            if reading.sensor_type == sensor_type {
                if results.push(reading).is_err() {
                    break;
                }
            }
        }
        
        results
    }
    
    pub fn calculate_statistics(&self, sensor_type: SensorType) -> Option<SensorStatistics> {
        let readings = self.get_latest_readings(sensor_type);
        
        if readings.is_empty() {
            return None;
        }
        
        let mut sum = 0.0f32;
        let mut min_val = f32::INFINITY;
        let mut max_val = f32::NEG_INFINITY;
        
        for reading in &readings {
            sum += reading.value;
            min_val = min_val.min(reading.value);
            max_val = max_val.max(reading.value);
        }
        
        let mean = sum / readings.len() as f32;
        
        // 计算标准差
        let variance: f32 = readings.iter()
            .map(|r| (r.value - mean).powi(2))
            .sum::<f32>() / readings.len() as f32;
        
        let std_dev = variance.sqrt();
        
        Some(SensorStatistics {
            count: readings.len() as u32,
            mean,
            std_dev,
            min: min_val,
            max: max_val,
            range: max_val - min_val,
        })
    }
}

#[derive(Debug, Clone)]
pub struct SensorStatistics {
    pub count: u32,
    pub mean: f32,
    pub std_dev: f32,
    pub min: f32,
    pub max: f32,
    pub range: f32,
}

// 多传感器管理系统
pub struct MultiSensorSystem {
    sensors: Vec<Box<dyn Sensor<Error = u8>>, 8>,
    fusion: SensorFusion,
    data_logger: DataLogger,
    alert_manager: AlertManager,
    system_time: u32,
}

pub struct DataLogger {
    log_buffer: Vec<LogEntry, 1000>,
    current_index: usize,
}

#[derive(Clone)]
pub struct LogEntry {
    pub timestamp: u32,
    pub sensor_id: u8,
    pub sensor_type: SensorType,
    pub value: f32,
    pub quality: DataQuality,
}

pub struct AlertManager {
    thresholds: FnvIndexMap<SensorType, AlertThreshold, 16>,
    active_alerts: Vec<Alert, 32>,
}

#[derive(Clone)]
pub struct AlertThreshold {
    pub min_value: f32,
    pub max_value: f32,
    pub rate_limit: f32, // 最大变化率
}

#[derive(Clone)]
pub struct Alert {
    pub alert_type: AlertType,
    pub sensor_id: u8,
    pub sensor_type: SensorType,
    pub value: f32,
    pub timestamp: u32,
}

#[derive(Clone, Copy)]
pub enum AlertType {
    ValueTooLow,
    ValueTooHigh,
    RateOfChangeExceeded,
    SensorOffline,
}

impl MultiSensorSystem {
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            fusion: SensorFusion::new(),
            data_logger: DataLogger::new(),
            alert_manager: AlertManager::new(),
            system_time: 0,
        }
    }
    
    pub fn add_sensor(&mut self, sensor: Box<dyn Sensor<Error = u8>>) -> Result<(), &'static str> {
        self.sensors.push(sensor).map_err(|_| "Too many sensors")
    }
    
    pub fn initialize_all_sensors(&mut self) -> Result<(), &'static str> {
        for sensor in &mut self.sensors {
            sensor.initialize().map_err(|_| "Sensor initialization failed")?;
        }
        Ok(())
    }
    
    pub fn update(&mut self) -> Result<(), &'static str> {
        self.system_time += 1;
        
        // 读取所有传感器
        for sensor in &mut self.sensors {
            match sensor.read() {
                Ok(readings) => {
                    for mut reading in readings {
                        reading.timestamp = self.system_time;
                        
                        // 添加到数据融合
                        self.fusion.add_reading(reading)?;
                        
                        // 记录数据
                        self.data_logger.log_reading(&reading);
                        
                        // 检查警报
                        self.alert_manager.check_alerts(&reading);
                    }
                }
                Err(_) => {
                    // 处理传感器错误
                    let sensor_info = sensor.get_sensor_info();
                    let alert = Alert {
                        alert_type: AlertType::SensorOffline,
                        sensor_id: sensor_info.id,
                        sensor_type: SensorType::Temperature, // 默认类型
                        value: 0.0,
                        timestamp: self.system_time,
                    };
                    
                    self.alert_manager.add_alert(alert);
                }
            }
        }
        
        Ok(())
    }
    
    pub fn get_sensor_statistics(&self, sensor_type: SensorType) -> Option<SensorStatistics> {
        self.fusion.calculate_statistics(sensor_type)
    }
    
    pub fn get_active_alerts(&self) -> &Vec<Alert, 32> {
        &self.alert_manager.active_alerts
    }
    
    pub fn export_data(&self) -> Result<Vec<u8, 1024>, &'static str> {
        self.data_logger.export_recent_data(100)
    }
}

impl DataLogger {
    pub fn new() -> Self {
        Self {
            log_buffer: Vec::new(),
            current_index: 0,
        }
    }
    
    pub fn log_reading(&mut self, reading: &SensorReading) {
        let entry = LogEntry {
            timestamp: reading.timestamp,
            sensor_id: reading.sensor_id,
            sensor_type: reading.sensor_type,
            value: reading.value,
            quality: reading.quality,
        };
        
        if self.log_buffer.is_full() {
            // 覆盖最旧的条目
            self.log_buffer[self.current_index] = entry;
            self.current_index = (self.current_index + 1) % self.log_buffer.capacity();
        } else {
            self.log_buffer.push(entry).ok();
        }
    }
    
    pub fn export_recent_data(&self, count: usize) -> Result<Vec<u8, 1024>, &'static str> {
        let mut export_data = Vec::new();
        let actual_count = count.min(self.log_buffer.len());
        
        for i in 0..actual_count {
            let index = if self.log_buffer.is_full() {
                (self.current_index + self.log_buffer.len() - actual_count + i) % self.log_buffer.len()
            } else {
                self.log_buffer.len() - actual_count + i
            };
            
            export_data.push(&self.log_buffer[index]).map_err(|_| "Export buffer full")?;
        }
        
        // 序列化数据 (简化版本)
        let mut result = Vec::new();
        for entry in export_data {
            // 简单的二进制格式: timestamp(4) + sensor_id(1) + type(1) + value(4)
            let timestamp_bytes = entry.timestamp.to_le_bytes();
            let value_bytes = entry.value.to_le_bytes();
            
            for &byte in &timestamp_bytes {
                result.push(byte).map_err(|_| "Result buffer full")?;
            }
            result.push(entry.sensor_id).map_err(|_| "Result buffer full")?;
            result.push(entry.sensor_type as u8).map_err(|_| "Result buffer full")?;
            for &byte in &value_bytes {
                result.push(byte).map_err(|_| "Result buffer full")?;
            }
        }
        
        Ok(result)
    }
}

impl AlertManager {
    pub fn new() -> Self {
        Self {
            thresholds: FnvIndexMap::new(),
            active_alerts: Vec::new(),
        }
    }
    
    pub fn set_threshold(&mut self, sensor_type: SensorType, threshold: AlertThreshold) -> Result<(), &'static str> {
        self.thresholds.insert(sensor_type, threshold).map_err(|_| "Threshold map full")?;
        Ok(())
    }
    
    pub fn check_alerts(&mut self, reading: &SensorReading) {
        if let Some(threshold) = self.thresholds.get(&reading.sensor_type) {
            let mut alert_type = None;
            
            if reading.value < threshold.min_value {
                alert_type = Some(AlertType::ValueTooLow);
            } else if reading.value > threshold.max_value {
                alert_type = Some(AlertType::ValueTooHigh);
            }
            
            if let Some(alert_type) = alert_type {
                let alert = Alert {
                    alert_type,
                    sensor_id: reading.sensor_id,
                    sensor_type: reading.sensor_type,
                    value: reading.value,
                    timestamp: reading.timestamp,
                };
                
                self.add_alert(alert);
            }
        }
    }
    
    pub fn add_alert(&mut self, alert: Alert) {
        if self.active_alerts.is_full() {
            self.active_alerts.remove(0);
        }
        
        self.active_alerts.push(alert).ok();
    }
    
    pub fn clear_alerts(&mut self) {
        self.active_alerts.clear();
    }
}

#[entry]
fn main() -> ! {
    // 获取外设
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 状态LED
    let mut status_led = gpioc.pc13.into_push_pull_output();
    let mut error_led = gpioa.pa5.into_push_pull_output();
    let mut activity_led = gpiob.pb0.into_push_pull_output();
    
    // 配置串口
    let tx = gpioa.pa2.into_alternate_af7();
    let rx = gpioa.pa3.into_alternate_af7();
    let serial = Serial::usart2(
        dp.USART2,
        (tx, rx),
        Config::default().baudrate(115200.bps()),
        clocks,
    ).unwrap();
    let (mut tx, mut rx) = serial.split();
    
    // 配置I2C
    let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
    let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
    let i2c1 = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);
    
    let scl2 = gpiob.pb10.into_alternate_af4().set_open_drain();
    let sda2 = gpiob.pb11.into_alternate_af4().set_open_drain();
    let i2c2 = I2c::i2c2(dp.I2C2, (scl2, sda2), 400.khz(), clocks);
    
    // 配置定时器
    let mut timer = Timer::tim2(dp.TIM2, 10.hz(), clocks); // 10Hz采样率
    
    // 初始化多传感器系统
    let mut sensor_system = MultiSensorSystem::new();
    
    // 添加BMP280传感器
    let bmp280 = BMP280Sensor::new(i2c1, 1);
    sensor_system.add_sensor(Box::new(bmp280)).unwrap();
    
    // 添加MPU6050传感器
    let mpu6050 = MPU6050Sensor::new(i2c2, 2);
    sensor_system.add_sensor(Box::new(mpu6050)).unwrap();
    
    // 设置警报阈值
    sensor_system.alert_manager.set_threshold(
        SensorType::Temperature,
        AlertThreshold {
            min_value: -10.0,
            max_value: 60.0,
            rate_limit: 5.0,
        }
    ).unwrap();
    
    sensor_system.alert_manager.set_threshold(
        SensorType::Pressure,
        AlertThreshold {
            min_value: 80000.0,
            max_value: 110000.0,
            rate_limit: 1000.0,
        }
    ).unwrap();
    
    // 初始化所有传感器
    match sensor_system.initialize_all_sensors() {
        Ok(()) => {
            for byte in b"Multi-Sensor System Initialized\r\n" {
                block!(tx.write(*byte)).ok();
            }
        }
        Err(_) => {
            error_led.set_high().ok();
            for byte in b"Sensor Initialization Failed\r\n" {
                block!(tx.write(*byte)).ok();
            }
        }
    }
    
    let mut cycle_count = 0u32;
    
    loop {
        // 等待定时器
        block!(timer.wait()).ok();
        
        // 更新传感器系统
        match sensor_system.update() {
            Ok(()) => {
                activity_led.toggle().ok();
                
                // 每10个周期输出统计信息
                if cycle_count % 10 == 0 {
                    // 输出温度统计
                    if let Some(temp_stats) = sensor_system.get_sensor_statistics(SensorType::Temperature) {
                        let mut buffer = [0u8; 128];
                        if let Ok(len) = format_statistics(&mut buffer, "TEMP", &temp_stats, cycle_count) {
                            for &byte in &buffer[..len] {
                                block!(tx.write(byte)).ok();
                            }
                        }
                    }
                    
                    // 输出压力统计
                    if let Some(pressure_stats) = sensor_system.get_sensor_statistics(SensorType::Pressure) {
                        let mut buffer = [0u8; 128];
                        if let Ok(len) = format_statistics(&mut buffer, "PRESS", &pressure_stats, cycle_count) {
                            for &byte in &buffer[..len] {
                                block!(tx.write(byte)).ok();
                            }
                        }
                    }
                    
                    // 输出警报信息
                    let alerts = sensor_system.get_active_alerts();
                    if !alerts.is_empty() {
                        let mut buffer = [0u8; 64];
                        if let Ok(len) = format_alerts(&mut buffer, alerts.len(), cycle_count) {
                            for &byte in &buffer[..len] {
                                block!(tx.write(byte)).ok();
                            }
                        }
                        error_led.set_high().ok();
                    } else {
                        error_led.set_low().ok();
                    }
                }
                
                // 每100个周期导出数据
                if cycle_count % 100 == 0 && cycle_count > 0 {
                    match sensor_system.export_data() {
                        Ok(data) => {
                            for byte in b"DATA_EXPORT:" {
                                block!(tx.write(*byte)).ok();
                            }
                            
                            // 发送数据长度
                            let len_bytes = (data.len() as u16).to_le_bytes();
                            for &byte in &len_bytes {
                                block!(tx.write(byte)).ok();
                            }
                            
                            // 发送数据
                            for &byte in &data {
                                block!(tx.write(byte)).ok();
                            }
                            
                            for byte in b"\r\n" {
                                block!(tx.write(*byte)).ok();
                            }
                        }
                        Err(_) => {
                            for byte in b"DATA_EXPORT_FAILED\r\n" {
                                block!(tx.write(*byte)).ok();
                            }
                        }
                    }
                }
            }
            Err(_) => {
                error_led.set_high().ok();
            }
        }
        
        // 切换状态LED
        status_led.toggle().ok();
        
        cycle_count += 1;
    }
}

// 格式化统计信息
fn format_statistics(buffer: &mut [u8], sensor_type: &str, stats: &SensorStatistics, timestamp: u32) -> Result<usize, ()> {
    use heapless::String;
    use core::fmt::Write;
    
    let mut output = String::<128>::new();
    write!(output, "STATS,{},MEAN={:.2},STD={:.2},MIN={:.2},MAX={:.2},COUNT={},TIME={}\r\n",
           sensor_type, stats.mean, stats.std_dev, stats.min, stats.max, stats.count, timestamp)
        .map_err(|_| ())?;
    
    let bytes = output.as_bytes();
    if bytes.len() > buffer.len() {
        return Err(());
    }
    
    buffer[..bytes.len()].copy_from_slice(bytes);
    Ok(bytes.len())
}

// 格式化警报信息
fn format_alerts(buffer: &mut [u8], alert_count: usize, timestamp: u32) -> Result<usize, ()> {
    use heapless::String;
    use core::fmt::Write;
    
    let mut output = String::<64>::new();
    write!(output, "ALERTS,COUNT={},TIME={}\r\n", alert_count, timestamp)
        .map_err(|_| ())?;
    
    let bytes = output.as_bytes();
    if bytes.len() > buffer.len() {
        return Err(());
    }
    
    buffer[..bytes.len()].copy_from_slice(bytes);
    Ok(bytes.len())
}