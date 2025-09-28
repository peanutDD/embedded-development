//! Temperature sensor implementations
//!
//! This module provides implementations for various temperature sensors
//! including DS18B20 (1-Wire), BME280 (I2C/SPI), and SHT30 (I2C).

use super::*;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_async::digital::Wait;

/// Temperature sensor trait
#[async_trait::async_trait]
pub trait TemperatureSensor: SensorTrait {
    /// Read temperature in Celsius
    async fn read_celsius(&mut self) -> Result<f32, Self::Error>;
    
    /// Read temperature in Fahrenheit
    async fn read_fahrenheit(&mut self) -> Result<f32, Self::Error> {
        let celsius = self.read_celsius().await?;
        Ok(super::utils::celsius_to_fahrenheit(celsius))
    }
    
    /// Read temperature in Kelvin
    async fn read_kelvin(&mut self) -> Result<f32, Self::Error> {
        let celsius = self.read_celsius().await?;
        Ok(super::utils::celsius_to_kelvin(celsius))
    }
    
    /// Get temperature resolution
    fn get_resolution(&self) -> f32;
    
    /// Set temperature resolution (if supported)
    async fn set_resolution(&mut self, _resolution: f32) -> Result<(), Self::Error> {
        Err(SensorError::NotSupported.into())
    }
}

/// DS18B20 1-Wire temperature sensor
pub struct DS18B20<P> {
    pin: P,
    resolution: u8,
    rom_code: Option<[u8; 8]>,
    info: SensorInfo,
}

impl<P> DS18B20<P>
where
    P: Wait,
{
    /// Create a new DS18B20 instance
    pub fn new(pin: P) -> Self {
        Self {
            pin,
            resolution: 12, // 12-bit resolution by default
            rom_code: None,
            info: SensorInfo {
                id: 0,
                name: "DS18B20",
                sensor_type: SensorType::Temperature,
                protocol: Protocol::OneWire,
                address: None,
                resolution: Some(0.0625), // 12-bit resolution
                range: Some((-55.0, 125.0)),
                accuracy: Some(0.5),
            },
        }
    }
    
    /// Set resolution (9-12 bits)
    pub fn set_resolution_bits(&mut self, bits: u8) -> Result<(), SensorError> {
        if bits < 9 || bits > 12 {
            return Err(SensorError::InvalidData);
        }
        
        self.resolution = bits;
        self.info.resolution = Some(match bits {
            9 => 0.5,
            10 => 0.25,
            11 => 0.125,
            12 => 0.0625,
            _ => unreachable!(),
        });
        
        Ok(())
    }
    
    /// Read temperature from DS18B20
    pub async fn read_temperature(&mut self) -> Result<f32, SensorError> {
        // Reset pulse
        self.reset_pulse().await?;
        
        // Skip ROM command (for single device)
        self.write_byte(0xCC).await?;
        
        // Convert T command
        self.write_byte(0x44).await?;
        
        // Wait for conversion (depends on resolution)
        let conversion_time = match self.resolution {
            9 => Duration::from_millis(94),
            10 => Duration::from_millis(188),
            11 => Duration::from_millis(375),
            12 => Duration::from_millis(750),
            _ => Duration::from_millis(750),
        };
        Timer::after(conversion_time).await;
        
        // Reset pulse
        self.reset_pulse().await?;
        
        // Skip ROM command
        self.write_byte(0xCC).await?;
        
        // Read scratchpad command
        self.write_byte(0xBE).await?;
        
        // Read 9 bytes of scratchpad
        let mut scratchpad = [0u8; 9];
        for byte in &mut scratchpad {
            *byte = self.read_byte().await?;
        }
        
        // Verify CRC
        if !self.verify_crc(&scratchpad) {
            return Err(SensorError::InvalidData);
        }
        
        // Convert temperature
        let temp_raw = ((scratchpad[1] as u16) << 8) | (scratchpad[0] as u16);
        let temperature = (temp_raw as i16) as f32 / 16.0;
        
        Ok(temperature)
    }
    
    /// Reset pulse for 1-Wire communication
    async fn reset_pulse(&mut self) -> Result<(), SensorError> {
        // Implementation would depend on the specific GPIO traits
        // This is a placeholder for the 1-Wire reset sequence
        Timer::after(Duration::from_micros(480)).await;
        Ok(())
    }
    
    /// Write a byte to 1-Wire bus
    async fn write_byte(&mut self, byte: u8) -> Result<(), SensorError> {
        for i in 0..8 {
            let bit = (byte >> i) & 0x01;
            self.write_bit(bit != 0).await?;
        }
        Ok(())
    }
    
    /// Read a byte from 1-Wire bus
    async fn read_byte(&mut self) -> Result<u8, SensorError> {
        let mut byte = 0u8;
        for i in 0..8 {
            if self.read_bit().await? {
                byte |= 1 << i;
            }
        }
        Ok(byte)
    }
    
    /// Write a bit to 1-Wire bus
    async fn write_bit(&mut self, bit: bool) -> Result<(), SensorError> {
        // Implementation would depend on the specific GPIO traits
        // This is a placeholder for the 1-Wire bit write sequence
        if bit {
            Timer::after(Duration::from_micros(6)).await;
        } else {
            Timer::after(Duration::from_micros(60)).await;
        }
        Ok(())
    }
    
    /// Read a bit from 1-Wire bus
    async fn read_bit(&mut self) -> Result<bool, SensorError> {
        // Implementation would depend on the specific GPIO traits
        // This is a placeholder for the 1-Wire bit read sequence
        Timer::after(Duration::from_micros(6)).await;
        Ok(true) // Placeholder
    }
    
    /// Verify CRC of scratchpad data
    fn verify_crc(&self, data: &[u8; 9]) -> bool {
        let mut crc = 0u8;
        for &byte in &data[0..8] {
            crc = self.crc8_update(crc, byte);
        }
        crc == data[8]
    }
    
    /// Update CRC8 calculation
    fn crc8_update(&self, crc: u8, data: u8) -> u8 {
        let mut crc = crc ^ data;
        for _ in 0..8 {
            if crc & 0x01 != 0 {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
        }
        crc
    }
}

#[async_trait::async_trait]
impl<P> SensorTrait for DS18B20<P>
where
    P: Wait,
{
    type Error = SensorError;
    type Data = f32;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        // Perform initial reset to check if sensor is present
        self.reset_pulse().await?;
        Ok(())
    }
    
    async fn read(&mut self) -> Result<Self::Data, Self::Error> {
        self.read_temperature().await
    }
    
    fn get_info(&self) -> SensorInfo {
        self.info.clone()
    }
}

#[async_trait::async_trait]
impl<P> TemperatureSensor for DS18B20<P>
where
    P: Wait,
{
    async fn read_celsius(&mut self) -> Result<f32, Self::Error> {
        self.read_temperature().await
    }
    
    fn get_resolution(&self) -> f32 {
        self.info.resolution.unwrap_or(0.0625)
    }
    
    async fn set_resolution(&mut self, resolution: f32) -> Result<(), Self::Error> {
        let bits = match resolution {
            r if r >= 0.5 => 9,
            r if r >= 0.25 => 10,
            r if r >= 0.125 => 11,
            _ => 12,
        };
        self.set_resolution_bits(bits)
    }
}

/// BME280 combined sensor (temperature, humidity, pressure)
pub struct BME280<I> {
    i2c: I,
    address: u8,
    calibration: Option<BME280Calibration>,
    info: SensorInfo,
}

/// BME280 calibration data
#[derive(Debug, Clone)]
struct BME280Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    t_fine: i32,
}

/// BME280 measurement data
#[derive(Debug, Clone)]
pub struct BME280Data {
    pub temperature: f32,
    pub humidity: f32,
    pub pressure: f32,
}

impl<I> BME280<I>
where
    I: I2c,
{
    /// Create new BME280 instance
    pub fn new(i2c: I, address: u8) -> Self {
        Self {
            i2c,
            address,
            calibration: None,
            info: SensorInfo {
                id: 0,
                name: "BME280",
                sensor_type: SensorType::Combined,
                protocol: Protocol::I2C,
                address: Some(address),
                resolution: Some(0.01),
                range: Some((-40.0, 85.0)),
                accuracy: Some(1.0),
            },
        }
    }
    
    /// Initialize BME280 sensor
    pub async fn initialize(&mut self) -> Result<(), SensorError> {
        // Check chip ID
        let chip_id = self.read_register(0xD0).await?;
        if chip_id != 0x60 {
            return Err(SensorError::SensorNotFound);
        }
        
        // Reset sensor
        self.write_register(0xE0, 0xB6).await?;
        Timer::after(Duration::from_millis(10)).await;
        
        // Read calibration data
        self.read_calibration().await?;
        
        // Configure sensor
        // Humidity oversampling x1
        self.write_register(0xF2, 0x01).await?;
        
        // Temperature oversampling x2, Pressure oversampling x16, Normal mode
        self.write_register(0xF4, 0x57).await?;
        
        // Config: standby 1000ms, filter off
        self.write_register(0xF5, 0xA0).await?;
        
        Ok(())
    }
    
    /// Read all measurements
    pub async fn read_all(&mut self) -> Result<BME280Data, SensorError> {
        // Read raw data
        let mut data = [0u8; 8];
        self.read_registers(0xF7, &mut data).await?;
        
        // Extract raw values
        let press_raw = ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);
        let temp_raw = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);
        let hum_raw = ((data[6] as u32) << 8) | (data[7] as u32);
        
        let cal = self.calibration.as_mut().ok_or(SensorError::CalibrationError)?;
        
        // Calculate temperature
        let temperature = self.compensate_temperature(temp_raw, cal)?;
        
        // Calculate pressure
        let pressure = self.compensate_pressure(press_raw, cal)?;
        
        // Calculate humidity
        let humidity = self.compensate_humidity(hum_raw, cal)?;
        
        Ok(BME280Data {
            temperature,
            humidity,
            pressure,
        })
    }
    
    /// Read temperature only
    pub async fn read_temperature(&mut self) -> Result<f32, SensorError> {
        let data = self.read_all().await?;
        Ok(data.temperature)
    }
    
    /// Read calibration data from sensor
    async fn read_calibration(&mut self) -> Result<(), SensorError> {
        let mut cal_data = [0u8; 26];
        self.read_registers(0x88, &mut cal_data).await?;
        
        let mut hum_cal = [0u8; 7];
        self.read_registers(0xE1, &mut hum_cal).await?;
        
        let calibration = BME280Calibration {
            dig_t1: u16::from_le_bytes([cal_data[0], cal_data[1]]),
            dig_t2: i16::from_le_bytes([cal_data[2], cal_data[3]]),
            dig_t3: i16::from_le_bytes([cal_data[4], cal_data[5]]),
            dig_p1: u16::from_le_bytes([cal_data[6], cal_data[7]]),
            dig_p2: i16::from_le_bytes([cal_data[8], cal_data[9]]),
            dig_p3: i16::from_le_bytes([cal_data[10], cal_data[11]]),
            dig_p4: i16::from_le_bytes([cal_data[12], cal_data[13]]),
            dig_p5: i16::from_le_bytes([cal_data[14], cal_data[15]]),
            dig_p6: i16::from_le_bytes([cal_data[16], cal_data[17]]),
            dig_p7: i16::from_le_bytes([cal_data[18], cal_data[19]]),
            dig_p8: i16::from_le_bytes([cal_data[20], cal_data[21]]),
            dig_p9: i16::from_le_bytes([cal_data[22], cal_data[23]]),
            dig_h1: cal_data[25],
            dig_h2: i16::from_le_bytes([hum_cal[0], hum_cal[1]]),
            dig_h3: hum_cal[2],
            dig_h4: ((hum_cal[3] as i16) << 4) | ((hum_cal[4] as i16) & 0x0F),
            dig_h5: ((hum_cal[5] as i16) << 4) | ((hum_cal[4] as i16) >> 4),
            dig_h6: hum_cal[6] as i8,
            t_fine: 0,
        };
        
        self.calibration = Some(calibration);
        Ok(())
    }
    
    /// Compensate temperature reading
    fn compensate_temperature(&self, adc_t: u32, cal: &mut BME280Calibration) -> Result<f32, SensorError> {
        let var1 = (((adc_t as i32) >> 3) - ((cal.dig_t1 as i32) << 1)) * (cal.dig_t2 as i32) >> 11;
        let var2 = (((((adc_t as i32) >> 4) - (cal.dig_t1 as i32)) * 
                    (((adc_t as i32) >> 4) - (cal.dig_t1 as i32))) >> 12) * (cal.dig_t3 as i32) >> 14;
        
        cal.t_fine = var1 + var2;
        let temperature = (cal.t_fine * 5 + 128) >> 8;
        Ok(temperature as f32 / 100.0)
    }
    
    /// Compensate pressure reading
    fn compensate_pressure(&self, adc_p: u32, cal: &BME280Calibration) -> Result<f32, SensorError> {
        let mut var1 = (cal.t_fine as i64) - 128000;
        let mut var2 = var1 * var1 * (cal.dig_p6 as i64);
        var2 = var2 + ((var1 * (cal.dig_p5 as i64)) << 17);
        var2 = var2 + ((cal.dig_p4 as i64) << 35);
        var1 = ((var1 * var1 * (cal.dig_p3 as i64)) >> 8) + ((var1 * (cal.dig_p2 as i64)) << 12);
        var1 = (((1i64 << 47) + var1) * (cal.dig_p1 as i64)) >> 33;
        
        if var1 == 0 {
            return Ok(0.0);
        }
        
        let mut p = 1048576 - (adc_p as i64);
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = ((cal.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        var2 = ((cal.dig_p8 as i64) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + ((cal.dig_p7 as i64) << 4);
        
        Ok((p as f32) / 256.0 / 100.0) // Convert to hPa
    }
    
    /// Compensate humidity reading
    fn compensate_humidity(&self, adc_h: u32, cal: &BME280Calibration) -> Result<f32, SensorError> {
        let v_x1_u32r = cal.t_fine - 76800;
        let v_x1_u32r = (((((adc_h as i32) << 14) - ((cal.dig_h4 as i32) << 20) - 
                          ((cal.dig_h5 as i32) * v_x1_u32r)) + 16384) >> 15) * 
                        (((((((v_x1_u32r * (cal.dig_h6 as i32)) >> 10) * 
                             (((v_x1_u32r * (cal.dig_h3 as i32)) >> 11) + 32768)) >> 10) + 
                           2097152) * (cal.dig_h2 as i32) + 8192) >> 14);
        
        let v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * 
                                      (cal.dig_h1 as i32)) >> 4);
        
        let v_x1_u32r = if v_x1_u32r < 0 { 0 } else { v_x1_u32r };
        let v_x1_u32r = if v_x1_u32r > 419430400 { 419430400 } else { v_x1_u32r };
        
        Ok((v_x1_u32r >> 12) as f32 / 1024.0)
    }
    
    /// Read single register
    async fn read_register(&mut self, reg: u8) -> Result<u8, SensorError> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        Ok(buffer[0])
    }
    
    /// Read multiple registers
    async fn read_registers(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), SensorError> {
        self.i2c.write_read(self.address, &[reg], buffer).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Write single register
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), SensorError> {
        self.i2c.write(self.address, &[reg, value]).await
            .map_err(|_| SensorError::CommunicationError)
    }
}

#[async_trait::async_trait]
impl<I> SensorTrait for BME280<I>
where
    I: I2c,
{
    type Error = SensorError;
    type Data = BME280Data;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        self.initialize().await
    }
    
    async fn read(&mut self) -> Result<Self::Data, Self::Error> {
        self.read_all().await
    }
    
    fn get_info(&self) -> SensorInfo {
        self.info.clone()
    }
}

#[async_trait::async_trait]
impl<I> TemperatureSensor for BME280<I>
where
    I: I2c,
{
    async fn read_celsius(&mut self) -> Result<f32, Self::Error> {
        self.read_temperature().await
    }
    
    fn get_resolution(&self) -> f32 {
        0.01 // BME280 has 0.01°C resolution
    }
}