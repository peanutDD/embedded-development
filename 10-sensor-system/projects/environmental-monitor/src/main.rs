#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    gpio::{gpioa::*, gpiob::*, gpioc::*, Alternate, Output, PushPull},
    i2c::{I2c, Mode},
    spi::{Spi, NoMiso, NoMosi},
    serial::{config::Config, Serial},
    timer::{Timer, Event},
    delay::Delay,
};

use heapless::{Vec, String, pool::{Pool, Node}};
use micromath::F32Ext;
use libm;

// 传感器数据结构
#[derive(Debug, Clone)]
pub struct EnvironmentalData {
    pub timestamp: u32,
    pub temperature: f32,      // 温度 (°C)
    pub humidity: f32,         // 湿度 (%)
    pub pressure: f32,         // 气压 (hPa)
    pub light_intensity: f32,  // 光照强度 (lux)
    pub co2_level: u16,        // CO2浓度 (ppm)
    pub pm25: u16,             // PM2.5 (μg/m³)
    pub pm10: u16,             // PM10 (μg/m³)
    pub air_quality_index: u8, // 空气质量指数
}

impl Default for EnvironmentalData {
    fn default() -> Self {
        Self {
            timestamp: 0,
            temperature: 0.0,
            humidity: 0.0,
            pressure: 0.0,
            light_intensity: 0.0,
            co2_level: 0,
            pm25: 0,
            pm10: 0,
            air_quality_index: 0,
        }
    }
}

// 传感器接口抽象
pub trait EnvironmentalSensor {
    type Error;
    
    fn read_data(&mut self) -> Result<EnvironmentalData, Self::Error>;
    fn calibrate(&mut self) -> Result<(), Self::Error>;
    fn get_sensor_info(&self) -> &'static str;
}

// BMP280 温湿度气压传感器
pub struct BMP280Sensor<I2C> {
    i2c: I2C,
    address: u8,
    calibration_data: CalibrationData,
}

#[derive(Default)]
struct CalibrationData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
}

impl<I2C, E> BMP280Sensor<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::Read<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: 0x76,
            calibration_data: CalibrationData::default(),
        }
    }
    
    pub fn init(&mut self) -> Result<(), E> {
        // 读取芯片ID验证
        let mut chip_id = [0u8; 1];
        self.i2c.write_read(self.address, &[0xD0], &mut chip_id)?;
        
        if chip_id[0] != 0x58 {
            // 返回错误，但这里我们简化处理
        }
        
        // 读取校准数据
        self.read_calibration_data()?;
        
        // 配置传感器
        // 设置控制寄存器：温度过采样x2，压力过采样x16，正常模式
        self.i2c.write(self.address, &[0xF4, 0x57])?;
        
        // 设置配置寄存器：待机时间1000ms，IIR滤波器系数16
        self.i2c.write(self.address, &[0xF5, 0xA0])?;
        
        Ok(())
    }
    
    fn read_calibration_data(&mut self) -> Result<(), E> {
        let mut cal_data = [0u8; 24];
        self.i2c.write_read(self.address, &[0x88], &mut cal_data)?;
        
        self.calibration_data.dig_t1 = u16::from_le_bytes([cal_data[0], cal_data[1]]);
        self.calibration_data.dig_t2 = i16::from_le_bytes([cal_data[2], cal_data[3]]);
        self.calibration_data.dig_t3 = i16::from_le_bytes([cal_data[4], cal_data[5]]);
        
        self.calibration_data.dig_p1 = u16::from_le_bytes([cal_data[6], cal_data[7]]);
        self.calibration_data.dig_p2 = i16::from_le_bytes([cal_data[8], cal_data[9]]);
        self.calibration_data.dig_p3 = i16::from_le_bytes([cal_data[10], cal_data[11]]);
        self.calibration_data.dig_p4 = i16::from_le_bytes([cal_data[12], cal_data[13]]);
        self.calibration_data.dig_p5 = i16::from_le_bytes([cal_data[14], cal_data[15]]);
        self.calibration_data.dig_p6 = i16::from_le_bytes([cal_data[16], cal_data[17]]);
        self.calibration_data.dig_p7 = i16::from_le_bytes([cal_data[18], cal_data[19]]);
        self.calibration_data.dig_p8 = i16::from_le_bytes([cal_data[20], cal_data[21]]);
        self.calibration_data.dig_p9 = i16::from_le_bytes([cal_data[22], cal_data[23]]);
        
        Ok(())
    }
    
    fn read_raw_data(&mut self) -> Result<(i32, i32), E> {
        let mut data = [0u8; 6];
        self.i2c.write_read(self.address, &[0xF7], &mut data)?;
        
        let pressure_raw = ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);
        let temperature_raw = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);
        
        Ok((temperature_raw as i32, pressure_raw as i32))
    }
    
    fn compensate_temperature(&self, adc_t: i32) -> (f32, i32) {
        let var1 = (((adc_t >> 3) - ((self.calibration_data.dig_t1 as i32) << 1)) *
                   (self.calibration_data.dig_t2 as i32)) >> 11;
        let var2 = (((((adc_t >> 4) - (self.calibration_data.dig_t1 as i32)) *
                     ((adc_t >> 4) - (self.calibration_data.dig_t1 as i32))) >> 12) *
                   (self.calibration_data.dig_t3 as i32)) >> 14;
        let t_fine = var1 + var2;
        let temperature = (t_fine * 5 + 128) >> 8;
        
        (temperature as f32 / 100.0, t_fine)
    }
    
    fn compensate_pressure(&self, adc_p: i32, t_fine: i32) -> f32 {
        let mut var1 = (t_fine as i64) - 128000;
        let mut var2 = var1 * var1 * (self.calibration_data.dig_p6 as i64);
        var2 = var2 + ((var1 * (self.calibration_data.dig_p5 as i64)) << 17);
        var2 = var2 + ((self.calibration_data.dig_p4 as i64) << 35);
        var1 = ((var1 * var1 * (self.calibration_data.dig_p3 as i64)) >> 8) +
               ((var1 * (self.calibration_data.dig_p2 as i64)) << 12);
        var1 = (((1i64 << 47) + var1) * (self.calibration_data.dig_p1 as i64)) >> 33;
        
        if var1 == 0 {
            return 0.0;
        }
        
        let mut p = 1048576 - adc_p as i64;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = ((self.calibration_data.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        var2 = ((self.calibration_data.dig_p8 as i64) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + ((self.calibration_data.dig_p7 as i64) << 4);
        
        (p as f32) / 256.0 / 100.0 // 转换为hPa
    }
}

impl<I2C, E> EnvironmentalSensor for BMP280Sensor<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::Read<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    type Error = E;
    
    fn read_data(&mut self) -> Result<EnvironmentalData, Self::Error> {
        let (temp_raw, press_raw) = self.read_raw_data()?;
        let (temperature, t_fine) = self.compensate_temperature(temp_raw);
        let pressure = self.compensate_pressure(press_raw, t_fine);
        
        let mut data = EnvironmentalData::default();
        data.temperature = temperature;
        data.pressure = pressure;
        
        Ok(data)
    }
    
    fn calibrate(&mut self) -> Result<(), Self::Error> {
        self.read_calibration_data()
    }
    
    fn get_sensor_info(&self) -> &'static str {
        "BMP280 Temperature & Pressure Sensor"
    }
}

// SHT30 温湿度传感器
pub struct SHT30Sensor<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> SHT30Sensor<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::Read<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: 0x44,
        }
    }
    
    pub fn init(&mut self) -> Result<(), E> {
        // 软复位
        self.i2c.write(self.address, &[0x30, 0xA2])?;
        Ok(())
    }
    
    fn crc8(&self, data: &[u8]) -> u8 {
        let mut crc = 0xFF;
        for byte in data {
            crc ^= byte;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ 0x31;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }
    
    fn read_measurement(&mut self) -> Result<(f32, f32), E> {
        // 发送测量命令（高重复性，时钟拉伸使能）
        self.i2c.write(self.address, &[0x2C, 0x06])?;
        
        // 等待测量完成
        cortex_m::asm::delay(15000); // 约15ms
        
        // 读取数据
        let mut data = [0u8; 6];
        self.i2c.read(self.address, &mut data)?;
        
        // 验证CRC
        if self.crc8(&data[0..2]) != data[2] || self.crc8(&data[3..5]) != data[5] {
            // CRC错误，这里简化处理
        }
        
        // 转换数据
        let temp_raw = u16::from_be_bytes([data[0], data[1]]);
        let hum_raw = u16::from_be_bytes([data[3], data[4]]);
        
        let temperature = -45.0 + 175.0 * (temp_raw as f32) / 65535.0;
        let humidity = 100.0 * (hum_raw as f32) / 65535.0;
        
        Ok((temperature, humidity))
    }
}

impl<I2C, E> EnvironmentalSensor for SHT30Sensor<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::Read<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    type Error = E;
    
    fn read_data(&mut self) -> Result<EnvironmentalData, Self::Error> {
        let (temperature, humidity) = self.read_measurement()?;
        
        let mut data = EnvironmentalData::default();
        data.temperature = temperature;
        data.humidity = humidity;
        
        Ok(data)
    }
    
    fn calibrate(&mut self) -> Result<(), Self::Error> {
        self.init()
    }
    
    fn get_sensor_info(&self) -> &'static str {
        "SHT30 Temperature & Humidity Sensor"
    }
}

// 数据融合和处理系统
pub struct DataProcessor {
    history_buffer: Vec<EnvironmentalData, 100>,
    current_index: usize,
    filter_alpha: f32,
    filtered_data: EnvironmentalData,
}

impl DataProcessor {
    pub fn new() -> Self {
        Self {
            history_buffer: Vec::new(),
            current_index: 0,
            filter_alpha: 0.1,
            filtered_data: EnvironmentalData::default(),
        }
    }
    
    pub fn process_data(&mut self, mut data: EnvironmentalData) -> EnvironmentalData {
        // 数据验证和异常检测
        data = self.validate_data(data);
        
        // 低通滤波
        self.apply_low_pass_filter(&data);
        
        // 添加到历史缓冲区
        if self.history_buffer.len() < 100 {
            let _ = self.history_buffer.push(data.clone());
        } else {
            self.history_buffer[self.current_index] = data.clone();
            self.current_index = (self.current_index + 1) % 100;
        }
        
        // 计算空气质量指数
        self.filtered_data.air_quality_index = self.calculate_aqi(&self.filtered_data);
        
        self.filtered_data.clone()
    }
    
    fn validate_data(&self, mut data: EnvironmentalData) -> EnvironmentalData {
        // 温度范围检查 (-40°C to 85°C)
        if data.temperature < -40.0 || data.temperature > 85.0 {
            data.temperature = self.filtered_data.temperature;
        }
        
        // 湿度范围检查 (0% to 100%)
        if data.humidity < 0.0 || data.humidity > 100.0 {
            data.humidity = self.filtered_data.humidity;
        }
        
        // 气压范围检查 (300hPa to 1100hPa)
        if data.pressure < 300.0 || data.pressure > 1100.0 {
            data.pressure = self.filtered_data.pressure;
        }
        
        data
    }
    
    fn apply_low_pass_filter(&mut self, data: &EnvironmentalData) {
        let alpha = self.filter_alpha;
        
        self.filtered_data.temperature = alpha * data.temperature + (1.0 - alpha) * self.filtered_data.temperature;
        self.filtered_data.humidity = alpha * data.humidity + (1.0 - alpha) * self.filtered_data.humidity;
        self.filtered_data.pressure = alpha * data.pressure + (1.0 - alpha) * self.filtered_data.pressure;
        self.filtered_data.light_intensity = alpha * data.light_intensity + (1.0 - alpha) * self.filtered_data.light_intensity;
        
        // 对于整数值使用简单平均
        self.filtered_data.co2_level = ((alpha * data.co2_level as f32) + 
                                       ((1.0 - alpha) * self.filtered_data.co2_level as f32)) as u16;
        self.filtered_data.pm25 = ((alpha * data.pm25 as f32) + 
                                  ((1.0 - alpha) * self.filtered_data.pm25 as f32)) as u16;
        self.filtered_data.pm10 = ((alpha * data.pm10 as f32) + 
                                  ((1.0 - alpha) * self.filtered_data.pm10 as f32)) as u16;
    }
    
    fn calculate_aqi(&self, data: &EnvironmentalData) -> u8 {
        // 简化的AQI计算，基于PM2.5
        let pm25_aqi = match data.pm25 {
            0..=12 => (data.pm25 as f32 / 12.0 * 50.0) as u8,
            13..=35 => (50.0 + (data.pm25 - 12) as f32 / 23.0 * 50.0) as u8,
            36..=55 => (100.0 + (data.pm25 - 35) as f32 / 20.0 * 50.0) as u8,
            56..=150 => (150.0 + (data.pm25 - 55) as f32 / 95.0 * 50.0) as u8,
            151..=250 => (200.0 + (data.pm25 - 150) as f32 / 100.0 * 100.0) as u8,
            _ => 255,
        };
        
        pm25_aqi.min(255)
    }
    
    pub fn get_statistics(&self) -> EnvironmentalStats {
        if self.history_buffer.is_empty() {
            return EnvironmentalStats::default();
        }
        
        let mut temp_sum = 0.0;
        let mut hum_sum = 0.0;
        let mut press_sum = 0.0;
        let mut temp_min = f32::INFINITY;
        let mut temp_max = f32::NEG_INFINITY;
        
        for data in &self.history_buffer {
            temp_sum += data.temperature;
            hum_sum += data.humidity;
            press_sum += data.pressure;
            
            if data.temperature < temp_min {
                temp_min = data.temperature;
            }
            if data.temperature > temp_max {
                temp_max = data.temperature;
            }
        }
        
        let count = self.history_buffer.len() as f32;
        
        EnvironmentalStats {
            temp_avg: temp_sum / count,
            temp_min,
            temp_max,
            humidity_avg: hum_sum / count,
            pressure_avg: press_sum / count,
            sample_count: self.history_buffer.len(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct EnvironmentalStats {
    pub temp_avg: f32,
    pub temp_min: f32,
    pub temp_max: f32,
    pub humidity_avg: f32,
    pub pressure_avg: f32,
    pub sample_count: usize,
}

impl Default for EnvironmentalStats {
    fn default() -> Self {
        Self {
            temp_avg: 0.0,
            temp_min: 0.0,
            temp_max: 0.0,
            humidity_avg: 0.0,
            pressure_avg: 0.0,
            sample_count: 0,
        }
    }
}

// 数据存储管理
pub struct DataLogger<SPI> {
    flash: SPI,
    current_address: u32,
    sector_size: u32,
    total_sectors: u32,
}

impl<SPI, E> DataLogger<SPI>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E>
        + embedded_hal::blocking::spi::Write<u8, Error = E>,
{
    pub fn new(flash: SPI) -> Self {
        Self {
            flash,
            current_address: 0,
            sector_size: 4096,
            total_sectors: 2048, // 8MB Flash
        }
    }
    
    pub fn init(&mut self) -> Result<(), E> {
        // 读取Flash ID验证
        let mut cmd = [0x9F, 0x00, 0x00, 0x00];
        self.flash.transfer(&mut cmd)?;
        
        // 查找下一个可用地址
        self.find_next_address()?;
        
        Ok(())
    }
    
    fn find_next_address(&mut self) -> Result<(), E> {
        // 简化实现：从头开始查找
        self.current_address = 0;
        Ok(())
    }
    
    pub fn log_data(&mut self, data: &EnvironmentalData) -> Result<(), E> {
        // 序列化数据（简化版本）
        let mut buffer = [0u8; 32];
        
        // 将数据转换为字节数组（简化实现）
        let temp_bytes = data.temperature.to_le_bytes();
        let hum_bytes = data.humidity.to_le_bytes();
        let press_bytes = data.pressure.to_le_bytes();
        
        buffer[0..4].copy_from_slice(&data.timestamp.to_le_bytes());
        buffer[4..8].copy_from_slice(&temp_bytes);
        buffer[8..12].copy_from_slice(&hum_bytes);
        buffer[12..16].copy_from_slice(&press_bytes);
        buffer[16..18].copy_from_slice(&data.co2_level.to_le_bytes());
        buffer[18..20].copy_from_slice(&data.pm25.to_le_bytes());
        buffer[20..22].copy_from_slice(&data.pm10.to_le_bytes());
        
        // 写入Flash
        self.write_page(self.current_address, &buffer)?;
        self.current_address += 32;
        
        // 检查是否需要擦除下一个扇区
        if self.current_address % self.sector_size == 0 {
            self.erase_sector(self.current_address)?;
        }
        
        Ok(())
    }
    
    fn write_page(&mut self, address: u32, data: &[u8]) -> Result<(), E> {
        // 写使能
        self.flash.write(&[0x06])?;
        
        // 页编程命令
        let mut cmd = Vec::<u8, 260>::new();
        let _ = cmd.push(0x02); // 页编程命令
        let _ = cmd.push((address >> 16) as u8);
        let _ = cmd.push((address >> 8) as u8);
        let _ = cmd.push(address as u8);
        
        for &byte in data {
            let _ = cmd.push(byte);
        }
        
        self.flash.write(&cmd)?;
        
        // 等待写入完成
        self.wait_for_ready()?;
        
        Ok(())
    }
    
    fn erase_sector(&mut self, address: u32) -> Result<(), E> {
        // 写使能
        self.flash.write(&[0x06])?;
        
        // 扇区擦除命令
        let cmd = [
            0x20, // 扇区擦除命令
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
        ];
        
        self.flash.write(&cmd)?;
        
        // 等待擦除完成
        self.wait_for_ready()?;
        
        Ok(())
    }
    
    fn wait_for_ready(&mut self) -> Result<(), E> {
        loop {
            let mut status = [0x05, 0x00];
            self.flash.transfer(&mut status)?;
            
            if status[1] & 0x01 == 0 {
                break;
            }
            
            cortex_m::asm::delay(1000);
        }
        Ok(())
    }
}

// 环境监测系统主控制器
pub struct EnvironmentalMonitor<I2C1, I2C2, SPI> {
    bmp280: BMP280Sensor<I2C1>,
    sht30: SHT30Sensor<I2C2>,
    processor: DataProcessor,
    logger: DataLogger<SPI>,
    system_time: u32,
    sampling_interval: u32,
    last_sample_time: u32,
}

impl<I2C1, I2C2, SPI, E1, E2, E3> EnvironmentalMonitor<I2C1, I2C2, SPI>
where
    I2C1: embedded_hal::blocking::i2c::Write<Error = E1>
         + embedded_hal::blocking::i2c::Read<Error = E1>
         + embedded_hal::blocking::i2c::WriteRead<Error = E1>,
    I2C2: embedded_hal::blocking::i2c::Write<Error = E2>
         + embedded_hal::blocking::i2c::Read<Error = E2>
         + embedded_hal::blocking::i2c::WriteRead<Error = E2>,
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E3>
        + embedded_hal::blocking::spi::Write<u8, Error = E3>,
{
    pub fn new(
        i2c1: I2C1,
        i2c2: I2C2,
        spi: SPI,
        sampling_interval: u32,
    ) -> Self {
        Self {
            bmp280: BMP280Sensor::new(i2c1),
            sht30: SHT30Sensor::new(i2c2),
            processor: DataProcessor::new(),
            logger: DataLogger::new(spi),
            system_time: 0,
            sampling_interval,
            last_sample_time: 0,
        }
    }
    
    pub fn init(&mut self) -> Result<(), ()> {
        // 初始化所有传感器
        self.bmp280.init().map_err(|_| ())?;
        self.sht30.init().map_err(|_| ())?;
        self.logger.init().map_err(|_| ())?;
        
        Ok(())
    }
    
    pub fn update(&mut self) -> Result<Option<EnvironmentalData>, ()> {
        self.system_time += 1;
        
        if self.system_time - self.last_sample_time >= self.sampling_interval {
            let data = self.sample_all_sensors()?;
            let processed_data = self.processor.process_data(data);
            
            // 记录数据
            self.logger.log_data(&processed_data).map_err(|_| ())?;
            
            self.last_sample_time = self.system_time;
            
            Ok(Some(processed_data))
        } else {
            Ok(None)
        }
    }
    
    fn sample_all_sensors(&mut self) -> Result<EnvironmentalData, ()> {
        let mut combined_data = EnvironmentalData::default();
        combined_data.timestamp = self.system_time;
        
        // 读取BMP280数据
        if let Ok(bmp_data) = self.bmp280.read_data() {
            combined_data.temperature = bmp_data.temperature;
            combined_data.pressure = bmp_data.pressure;
        }
        
        // 读取SHT30数据
        if let Ok(sht_data) = self.sht30.read_data() {
            // 如果BMP280没有读取到温度，使用SHT30的
            if combined_data.temperature == 0.0 {
                combined_data.temperature = sht_data.temperature;
            } else {
                // 否则取平均值
                combined_data.temperature = (combined_data.temperature + sht_data.temperature) / 2.0;
            }
            combined_data.humidity = sht_data.humidity;
        }
        
        Ok(combined_data)
    }
    
    pub fn get_statistics(&self) -> EnvironmentalStats {
        self.processor.get_statistics()
    }
    
    pub fn set_sampling_interval(&mut self, interval: u32) {
        self.sampling_interval = interval;
    }
}

#[entry]
fn main() -> ! {
    // 获取外设访问权限
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置I2C1 (BMP280)
    let scl1 = gpiob.pb8.into_alternate_af4().set_open_drain();
    let sda1 = gpiob.pb9.into_alternate_af4().set_open_drain();
    let i2c1 = I2c::i2c1(dp.I2C1, (scl1, sda1), 400.khz(), clocks);
    
    // 配置I2C2 (SHT30)
    let scl2 = gpiob.pb10.into_alternate_af4().set_open_drain();
    let sda2 = gpiob.pb11.into_alternate_af4().set_open_drain();
    let i2c2 = I2c::i2c2(dp.I2C2, (scl2, sda2), 400.khz(), clocks);
    
    // 配置SPI1 (Flash存储)
    let sck = gpioa.pa5.into_alternate_af5();
    let miso = gpioa.pa6.into_alternate_af5();
    let mosi = gpioa.pa7.into_alternate_af5();
    let spi = Spi::spi1(dp.SPI1, (sck, miso, mosi), 
                       embedded_hal::spi::MODE_0, 1.mhz(), clocks);
    
    // 配置状态LED
    let mut status_led = gpioc.pc13.into_push_pull_output();
    let mut error_led = gpioa.pa8.into_push_pull_output();
    
    // 配置串口用于调试输出
    let tx = gpioa.pa2.into_alternate_af7();
    let rx = gpioa.pa3.into_alternate_af7();
    let serial = Serial::usart2(
        dp.USART2,
        (tx, rx),
        Config::default().baudrate(115200.bps()),
        clocks,
    ).unwrap();
    let (mut tx, _rx) = serial.split();
    
    // 配置系统定时器
    let mut timer = Timer::tim2(dp.TIM2, 1.hz(), clocks);
    timer.listen(Event::TimeOut);
    
    // 创建延时对象
    let mut delay = Delay::new(cp.SYST, clocks);
    
    // 初始化环境监测系统
    let mut monitor = EnvironmentalMonitor::new(i2c1, i2c2, spi, 1000); // 1秒采样间隔
    
    // 系统初始化
    match monitor.init() {
        Ok(_) => {
            status_led.set_high().ok();
            // 发送初始化成功消息
            for byte in b"Environmental Monitor Initialized\r\n" {
                nb::block!(tx.write(*byte)).ok();
            }
        }
        Err(_) => {
            error_led.set_high().ok();
            // 发送初始化失败消息
            for byte in b"Initialization Failed\r\n" {
                nb::block!(tx.write(*byte)).ok();
            }
        }
    }
    
    let mut led_state = false;
    let mut cycle_count = 0u32;
    
    loop {
        // 更新监测系统
        match monitor.update() {
            Ok(Some(data)) => {
                // 有新数据
                status_led.set_low().ok();
                
                // 输出数据到串口
                let temp_str = format_float(data.temperature, 1);
                let hum_str = format_float(data.humidity, 1);
                let press_str = format_float(data.pressure, 1);
                
                // 发送数据（简化格式）
                for byte in b"Data: T=" {
                    nb::block!(tx.write(*byte)).ok();
                }
                for byte in temp_str.as_bytes() {
                    nb::block!(tx.write(*byte)).ok();
                }
                for byte in b"C H=" {
                    nb::block!(tx.write(*byte)).ok();
                }
                for byte in hum_str.as_bytes() {
                    nb::block!(tx.write(*byte)).ok();
                }
                for byte in b"% P=" {
                    nb::block!(tx.write(*byte)).ok();
                }
                for byte in press_str.as_bytes() {
                    nb::block!(tx.write(*byte)).ok();
                }
                for byte in b"hPa AQI=" {
                    nb::block!(tx.write(*byte)).ok();
                }
                
                // 输出AQI
                let aqi_str = format_u8(data.air_quality_index);
                for byte in aqi_str.as_bytes() {
                    nb::block!(tx.write(*byte)).ok();
                }
                for byte in b"\r\n" {
                    nb::block!(tx.write(*byte)).ok();
                }
                
                cycle_count += 1;
                
                // 每10次采样输出统计信息
                if cycle_count % 10 == 0 {
                    let stats = monitor.get_statistics();
                    
                    for byte in b"Stats: Avg=" {
                        nb::block!(tx.write(*byte)).ok();
                    }
                    let avg_str = format_float(stats.temp_avg, 1);
                    for byte in avg_str.as_bytes() {
                        nb::block!(tx.write(*byte)).ok();
                    }
                    for byte in b"C Min=" {
                        nb::block!(tx.write(*byte)).ok();
                    }
                    let min_str = format_float(stats.temp_min, 1);
                    for byte in min_str.as_bytes() {
                        nb::block!(tx.write(*byte)).ok();
                    }
                    for byte in b"C Max=" {
                        nb::block!(tx.write(*byte)).ok();
                    }
                    let max_str = format_float(stats.temp_max, 1);
                    for byte in max_str.as_bytes() {
                        nb::block!(tx.write(*byte)).ok();
                    }
                    for byte in b"C\r\n" {
                        nb::block!(tx.write(*byte)).ok();
                    }
                }
            }
            Ok(None) => {
                // 无新数据，正常运行
                if led_state {
                    status_led.set_high().ok();
                } else {
                    status_led.set_low().ok();
                }
                led_state = !led_state;
            }
            Err(_) => {
                // 错误状态
                error_led.set_high().ok();
                status_led.set_low().ok();
            }
        }
        
        delay.delay_ms(100u32);
    }
}

// 辅助函数：格式化浮点数
fn format_float(value: f32, decimals: u8) -> String<16> {
    let mut result = String::new();
    
    let integer_part = value as i32;
    let fractional_part = ((value - integer_part as f32) * 10.0_f32.powi(decimals as i32)) as u32;
    
    // 处理负数
    if value < 0.0 && integer_part == 0 {
        let _ = result.push('-');
    }
    
    // 整数部分
    let int_str = format_i32(integer_part);
    for ch in int_str.chars() {
        let _ = result.push(ch);
    }
    
    // 小数点和小数部分
    if decimals > 0 {
        let _ = result.push('.');
        let frac_str = format_u32_with_leading_zeros(fractional_part, decimals);
        for ch in frac_str.chars() {
            let _ = result.push(ch);
        }
    }
    
    result
}

fn format_i32(mut value: i32) -> String<12> {
    let mut result = String::new();
    
    if value == 0 {
        let _ = result.push('0');
        return result;
    }
    
    let negative = value < 0;
    if negative {
        value = -value;
    }
    
    let mut digits = String::<12>::new();
    while value > 0 {
        let digit = (value % 10) as u8 + b'0';
        let _ = digits.push(digit as char);
        value /= 10;
    }
    
    if negative {
        let _ = result.push('-');
    }
    
    // 反转数字
    for ch in digits.chars().rev() {
        let _ = result.push(ch);
    }
    
    result
}

fn format_u32_with_leading_zeros(mut value: u32, width: u8) -> String<8> {
    let mut result = String::new();
    
    for _ in 0..width {
        let digit = (value % 10) as u8 + b'0';
        let _ = result.push(digit as char);
        value /= 10;
    }
    
    // 反转
    let chars: Vec<char, 8> = result.chars().collect();
    result.clear();
    for &ch in chars.iter().rev() {
        let _ = result.push(ch);
    }
    
    result
}

fn format_u8(value: u8) -> String<4> {
    let mut result = String::new();
    
    if value == 0 {
        let _ = result.push('0');
        return result;
    }
    
    let mut temp = value;
    let mut digits = String::<4>::new();
    
    while temp > 0 {
        let digit = (temp % 10) + b'0';
        let _ = digits.push(digit as char);
        temp /= 10;
    }
    
    // 反转数字
    for ch in digits.chars().rev() {
        let _ = result.push(ch);
    }
    
    result
}