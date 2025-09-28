//! Humidity sensor implementations
//!
//! This module provides implementations for various humidity sensors
//! including SHT30 (I2C) and BME280 (I2C/SPI).

use super::*;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

/// Humidity sensor trait
#[async_trait::async_trait]
pub trait HumiditySensor: SensorTrait {
    /// Read relative humidity percentage
    async fn read_humidity(&mut self) -> Result<f32, Self::Error>;
    
    /// Read both temperature and humidity (for combined sensors)
    async fn read_temp_humidity(&mut self) -> Result<(f32, f32), Self::Error> {
        let humidity = self.read_humidity().await?;
        // Default implementation - override for sensors that can read both simultaneously
        Ok((0.0, humidity))
    }
    
    /// Calculate dew point from current readings
    async fn calculate_dew_point(&mut self) -> Result<f32, Self::Error> {
        let (temperature, humidity) = self.read_temp_humidity().await?;
        Ok(super::utils::calculate_dew_point(temperature, humidity))
    }
    
    /// Get humidity measurement range
    fn get_humidity_range(&self) -> (f32, f32) {
        (0.0, 100.0) // Default 0-100% RH
    }
    
    /// Get humidity accuracy
    fn get_humidity_accuracy(&self) -> f32 {
        2.0 // Default ±2% RH
    }
}

/// SHT30 I2C humidity and temperature sensor
pub struct SHT30<I> {
    i2c: I,
    address: u8,
    info: SensorInfo,
}

/// SHT30 measurement data
#[derive(Debug, Clone)]
pub struct SHT30Data {
    pub temperature: f32,
    pub humidity: f32,
}

/// SHT30 measurement repeatability
#[derive(Debug, Clone, Copy)]
pub enum SHT30Repeatability {
    High,   // 15.5ms measurement time
    Medium, // 6.5ms measurement time  
    Low,    // 4.5ms measurement time
}

/// SHT30 clock stretching mode
#[derive(Debug, Clone, Copy)]
pub enum SHT30ClockStretching {
    Enabled,
    Disabled,
}

impl<I> SHT30<I>
where
    I: I2c,
{
    /// Create new SHT30 instance
    pub fn new(i2c: I, address: u8) -> Self {
        Self {
            i2c,
            address,
            info: SensorInfo {
                id: 0,
                name: "SHT30",
                sensor_type: SensorType::Combined,
                protocol: Protocol::I2C,
                address: Some(address),
                resolution: Some(0.01), // 0.01% RH, 0.01°C
                range: Some((0.0, 100.0)), // 0-100% RH
                accuracy: Some(2.0), // ±2% RH, ±0.3°C
            },
        }
    }
    
    /// Initialize SHT30 sensor
    pub async fn initialize(&mut self) -> Result<(), SensorError> {
        // Soft reset
        self.soft_reset().await?;
        
        // Wait for reset to complete
        Timer::after(Duration::from_millis(2)).await;
        
        // Read status register to verify communication
        let _status = self.read_status().await?;
        
        Ok(())
    }
    
    /// Perform soft reset
    pub async fn soft_reset(&mut self) -> Result<(), SensorError> {
        let command = [0x30, 0xA2]; // Soft reset command
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Read status register
    pub async fn read_status(&mut self) -> Result<u16, SensorError> {
        let command = [0xF3, 0x2D]; // Read status command
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)?;
        
        let mut buffer = [0u8; 3];
        self.i2c.read(self.address, &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        
        // Verify CRC
        if !self.verify_crc(&buffer[0..2], buffer[2]) {
            return Err(SensorError::InvalidData);
        }
        
        Ok(u16::from_be_bytes([buffer[0], buffer[1]]))
    }
    
    /// Clear status register
    pub async fn clear_status(&mut self) -> Result<(), SensorError> {
        let command = [0x30, 0x41]; // Clear status command
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Read temperature and humidity with specified repeatability
    pub async fn read_temp_humidity_with_repeatability(
        &mut self,
        repeatability: SHT30Repeatability,
        clock_stretching: SHT30ClockStretching,
    ) -> Result<SHT30Data, SensorError> {
        let command = match (repeatability, clock_stretching) {
            (SHT30Repeatability::High, SHT30ClockStretching::Enabled) => [0x2C, 0x06],
            (SHT30Repeatability::Medium, SHT30ClockStretching::Enabled) => [0x2C, 0x0D],
            (SHT30Repeatability::Low, SHT30ClockStretching::Enabled) => [0x2C, 0x10],
            (SHT30Repeatability::High, SHT30ClockStretching::Disabled) => [0x24, 0x00],
            (SHT30Repeatability::Medium, SHT30ClockStretching::Disabled) => [0x24, 0x0B],
            (SHT30Repeatability::Low, SHT30ClockStretching::Disabled) => [0x24, 0x16],
        };
        
        // Send measurement command
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)?;
        
        // Wait for measurement to complete
        let wait_time = match repeatability {
            SHT30Repeatability::High => Duration::from_millis(16),
            SHT30Repeatability::Medium => Duration::from_millis(7),
            SHT30Repeatability::Low => Duration::from_millis(5),
        };
        Timer::after(wait_time).await;
        
        // Read measurement data
        let mut buffer = [0u8; 6];
        self.i2c.read(self.address, &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        
        // Verify CRCs
        if !self.verify_crc(&buffer[0..2], buffer[2]) {
            return Err(SensorError::InvalidData);
        }
        if !self.verify_crc(&buffer[3..5], buffer[5]) {
            return Err(SensorError::InvalidData);
        }
        
        // Convert raw data
        let temp_raw = u16::from_be_bytes([buffer[0], buffer[1]]);
        let hum_raw = u16::from_be_bytes([buffer[3], buffer[4]]);
        
        let temperature = -45.0 + 175.0 * (temp_raw as f32) / 65535.0;
        let humidity = 100.0 * (hum_raw as f32) / 65535.0;
        
        // Clamp humidity to valid range
        let humidity = humidity.max(0.0).min(100.0);
        
        Ok(SHT30Data {
            temperature,
            humidity,
        })
    }
    
    /// Read temperature and humidity with default settings (high repeatability, no clock stretching)
    pub async fn read_temp_humidity(&mut self) -> Result<SHT30Data, SensorError> {
        self.read_temp_humidity_with_repeatability(
            SHT30Repeatability::High,
            SHT30ClockStretching::Disabled,
        ).await
    }
    
    /// Read only humidity
    pub async fn read_humidity(&mut self) -> Result<f32, SensorError> {
        let data = self.read_temp_humidity().await?;
        Ok(data.humidity)
    }
    
    /// Read only temperature
    pub async fn read_temperature(&mut self) -> Result<f32, SensorError> {
        let data = self.read_temp_humidity().await?;
        Ok(data.temperature)
    }
    
    /// Start periodic measurement mode
    pub async fn start_periodic_measurement(
        &mut self,
        repeatability: SHT30Repeatability,
        frequency: SHT30MeasurementFrequency,
    ) -> Result<(), SensorError> {
        let command = match (repeatability, frequency) {
            (SHT30Repeatability::High, SHT30MeasurementFrequency::Freq0_5Hz) => [0x20, 0x32],
            (SHT30Repeatability::Medium, SHT30MeasurementFrequency::Freq0_5Hz) => [0x20, 0x24],
            (SHT30Repeatability::Low, SHT30MeasurementFrequency::Freq0_5Hz) => [0x20, 0x2F],
            (SHT30Repeatability::High, SHT30MeasurementFrequency::Freq1Hz) => [0x21, 0x30],
            (SHT30Repeatability::Medium, SHT30MeasurementFrequency::Freq1Hz) => [0x21, 0x26],
            (SHT30Repeatability::Low, SHT30MeasurementFrequency::Freq1Hz) => [0x21, 0x2D],
            (SHT30Repeatability::High, SHT30MeasurementFrequency::Freq2Hz) => [0x22, 0x36],
            (SHT30Repeatability::Medium, SHT30MeasurementFrequency::Freq2Hz) => [0x22, 0x20],
            (SHT30Repeatability::Low, SHT30MeasurementFrequency::Freq2Hz) => [0x22, 0x2B],
            (SHT30Repeatability::High, SHT30MeasurementFrequency::Freq4Hz) => [0x23, 0x34],
            (SHT30Repeatability::Medium, SHT30MeasurementFrequency::Freq4Hz) => [0x23, 0x22],
            (SHT30Repeatability::Low, SHT30MeasurementFrequency::Freq4Hz) => [0x23, 0x29],
            (SHT30Repeatability::High, SHT30MeasurementFrequency::Freq10Hz) => [0x27, 0x37],
            (SHT30Repeatability::Medium, SHT30MeasurementFrequency::Freq10Hz) => [0x27, 0x21],
            (SHT30Repeatability::Low, SHT30MeasurementFrequency::Freq10Hz) => [0x27, 0x2A],
        };
        
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Stop periodic measurement mode
    pub async fn stop_periodic_measurement(&mut self) -> Result<(), SensorError> {
        let command = [0x30, 0x93]; // Break command
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Fetch data from periodic measurement
    pub async fn fetch_periodic_data(&mut self) -> Result<SHT30Data, SensorError> {
        let command = [0xE0, 0x00]; // Fetch data command
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)?;
        
        let mut buffer = [0u8; 6];
        self.i2c.read(self.address, &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        
        // Verify CRCs
        if !self.verify_crc(&buffer[0..2], buffer[2]) {
            return Err(SensorError::InvalidData);
        }
        if !self.verify_crc(&buffer[3..5], buffer[5]) {
            return Err(SensorError::InvalidData);
        }
        
        // Convert raw data
        let temp_raw = u16::from_be_bytes([buffer[0], buffer[1]]);
        let hum_raw = u16::from_be_bytes([buffer[3], buffer[4]]);
        
        let temperature = -45.0 + 175.0 * (temp_raw as f32) / 65535.0;
        let humidity = 100.0 * (hum_raw as f32) / 65535.0;
        
        // Clamp humidity to valid range
        let humidity = humidity.max(0.0).min(100.0);
        
        Ok(SHT30Data {
            temperature,
            humidity,
        })
    }
    
    /// Enable heater
    pub async fn enable_heater(&mut self) -> Result<(), SensorError> {
        let command = [0x30, 0x6D]; // Heater enable command
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Disable heater
    pub async fn disable_heater(&mut self) -> Result<(), SensorError> {
        let command = [0x30, 0x66]; // Heater disable command
        self.i2c.write(self.address, &command).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Verify CRC-8 checksum
    fn verify_crc(&self, data: &[u8], crc: u8) -> bool {
        let mut calculated_crc = 0xFF;
        
        for &byte in data {
            calculated_crc ^= byte;
            for _ in 0..8 {
                if calculated_crc & 0x80 != 0 {
                    calculated_crc = (calculated_crc << 1) ^ 0x31;
                } else {
                    calculated_crc <<= 1;
                }
            }
        }
        
        calculated_crc == crc
    }
}

/// SHT30 measurement frequency for periodic mode
#[derive(Debug, Clone, Copy)]
pub enum SHT30MeasurementFrequency {
    Freq0_5Hz,
    Freq1Hz,
    Freq2Hz,
    Freq4Hz,
    Freq10Hz,
}

#[async_trait::async_trait]
impl<I> SensorTrait for SHT30<I>
where
    I: I2c,
{
    type Error = SensorError;
    type Data = SHT30Data;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        self.initialize().await
    }
    
    async fn read(&mut self) -> Result<Self::Data, Self::Error> {
        self.read_temp_humidity().await
    }
    
    fn get_info(&self) -> SensorInfo {
        self.info.clone()
    }
    
    async fn sleep(&mut self) -> Result<(), Self::Error> {
        // Stop any periodic measurements and enter low power mode
        let _ = self.stop_periodic_measurement().await;
        Ok(())
    }
}

#[async_trait::async_trait]
impl<I> HumiditySensor for SHT30<I>
where
    I: I2c,
{
    async fn read_humidity(&mut self) -> Result<f32, Self::Error> {
        self.read_humidity().await
    }
    
    async fn read_temp_humidity(&mut self) -> Result<(f32, f32), Self::Error> {
        let data = self.read_temp_humidity().await?;
        Ok((data.temperature, data.humidity))
    }
    
    fn get_humidity_range(&self) -> (f32, f32) {
        (0.0, 100.0)
    }
    
    fn get_humidity_accuracy(&self) -> f32 {
        2.0 // ±2% RH
    }
}

/// DHT22 (AM2302) humidity and temperature sensor
pub struct DHT22<P> {
    pin: P,
    info: SensorInfo,
}

/// DHT22 measurement data
#[derive(Debug, Clone)]
pub struct DHT22Data {
    pub temperature: f32,
    pub humidity: f32,
}

impl<P> DHT22<P> {
    /// Create new DHT22 instance
    pub fn new(pin: P) -> Self {
        Self {
            pin,
            info: SensorInfo {
                id: 0,
                name: "DHT22",
                sensor_type: SensorType::Combined,
                protocol: Protocol::Digital,
                address: None,
                resolution: Some(0.1), // 0.1% RH, 0.1°C
                range: Some((0.0, 100.0)), // 0-100% RH
                accuracy: Some(2.0), // ±2% RH, ±0.5°C
            },
        }
    }
    
    /// Read temperature and humidity from DHT22
    /// Note: This is a simplified implementation. A real implementation would
    /// require precise timing control and GPIO manipulation.
    pub async fn read_temp_humidity(&mut self) -> Result<DHT22Data, SensorError> {
        // DHT22 communication protocol:
        // 1. Send start signal (pull low for 1-10ms, then high for 20-40μs)
        // 2. Wait for DHT22 response (low for 80μs, then high for 80μs)
        // 3. Read 40 bits of data (5 bytes)
        // 4. Verify checksum
        
        // This is a placeholder implementation
        // Real implementation would require:
        // - Precise timing control
        // - GPIO pin manipulation
        // - Interrupt handling or busy waiting
        
        Timer::after(Duration::from_millis(2)).await;
        
        // Placeholder data - in real implementation, this would be read from the sensor
        let humidity_raw = 0x0234u16; // Example: 56.4% RH
        let temperature_raw = 0x0167u16; // Example: 35.9°C
        
        let humidity = (humidity_raw as f32) / 10.0;
        let temperature = (temperature_raw as f32) / 10.0;
        
        Ok(DHT22Data {
            temperature,
            humidity,
        })
    }
}

#[async_trait::async_trait]
impl<P> SensorTrait for DHT22<P> {
    type Error = SensorError;
    type Data = DHT22Data;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        // DHT22 doesn't require initialization
        Ok(())
    }
    
    async fn read(&mut self) -> Result<Self::Data, Self::Error> {
        self.read_temp_humidity().await
    }
    
    fn get_info(&self) -> SensorInfo {
        self.info.clone()
    }
}

#[async_trait::async_trait]
impl<P> HumiditySensor for DHT22<P> {
    async fn read_humidity(&mut self) -> Result<f32, Self::Error> {
        let data = self.read_temp_humidity().await?;
        Ok(data.humidity)
    }
    
    async fn read_temp_humidity(&mut self) -> Result<(f32, f32), Self::Error> {
        let data = self.read_temp_humidity().await?;
        Ok((data.temperature, data.humidity))
    }
    
    fn get_humidity_range(&self) -> (f32, f32) {
        (0.0, 100.0)
    }
    
    fn get_humidity_accuracy(&self) -> f32 {
        2.0 // ±2% RH
    }
}

/// Humidity sensor manager for handling multiple humidity sensors
pub struct HumiditySensorManager {
    sensors: heapless::Vec<SensorInfo, 8>,
    calibrations: heapless::FnvIndexMap<u8, HumidityCalibration, 8>,
}

/// Humidity calibration data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HumidityCalibration {
    pub offset: f32,
    pub scale: f32,
    pub reference_temperature: f32,
    pub temperature_coefficient: f32,
}

impl HumidityCalibration {
    pub fn new(offset: f32, scale: f32) -> Self {
        Self {
            offset,
            scale,
            reference_temperature: 25.0,
            temperature_coefficient: 0.0,
        }
    }
    
    /// Apply calibration with temperature compensation
    pub fn apply(&self, raw_humidity: f32, temperature: f32) -> f32 {
        let temp_compensation = self.temperature_coefficient * (temperature - self.reference_temperature);
        ((raw_humidity + self.offset) * self.scale + temp_compensation).max(0.0).min(100.0)
    }
}

impl HumiditySensorManager {
    pub fn new() -> Self {
        Self {
            sensors: heapless::Vec::new(),
            calibrations: heapless::FnvIndexMap::new(),
        }
    }
    
    /// Register a humidity sensor
    pub fn register_sensor(&mut self, sensor_info: SensorInfo) -> Result<(), SensorError> {
        self.sensors.push(sensor_info)
            .map_err(|_| SensorError::NotSupported)
    }
    
    /// Set calibration for a sensor
    pub fn set_calibration(&mut self, sensor_id: u8, calibration: HumidityCalibration) -> Result<(), SensorError> {
        self.calibrations.insert(sensor_id, calibration)
            .map_err(|_| SensorError::NotSupported)?;
        Ok(())
    }
    
    /// Get calibration for a sensor
    pub fn get_calibration(&self, sensor_id: u8) -> Option<&HumidityCalibration> {
        self.calibrations.get(&sensor_id)
    }
    
    /// Apply calibration to a reading
    pub fn apply_calibration(&self, sensor_id: u8, raw_humidity: f32, temperature: f32) -> f32 {
        if let Some(cal) = self.get_calibration(sensor_id) {
            cal.apply(raw_humidity, temperature)
        } else {
            raw_humidity
        }
    }
    
    /// List all registered humidity sensors
    pub fn list_sensors(&self) -> &[SensorInfo] {
        &self.sensors
    }
}

/// Humidity data processing utilities
pub mod humidity_utils {
    use super::*;
    
    /// Calculate absolute humidity from relative humidity and temperature
    pub fn calculate_absolute_humidity(relative_humidity: f32, temperature_celsius: f32) -> f32 {
        // Saturation vapor pressure (hPa)
        let svp = 6.112 * (17.67 * temperature_celsius / (temperature_celsius + 243.5)).exp();
        
        // Actual vapor pressure (hPa)
        let avp = relative_humidity / 100.0 * svp;
        
        // Absolute humidity (g/m³)
        216.7 * avp / (temperature_celsius + 273.15)
    }
    
    /// Calculate mixing ratio from relative humidity and pressure
    pub fn calculate_mixing_ratio(relative_humidity: f32, temperature_celsius: f32, pressure_hpa: f32) -> f32 {
        let svp = 6.112 * (17.67 * temperature_celsius / (temperature_celsius + 243.5)).exp();
        let avp = relative_humidity / 100.0 * svp;
        
        // Mixing ratio (g/kg)
        621.97 * avp / (pressure_hpa - avp)
    }
    
    /// Calculate vapor pressure deficit
    pub fn calculate_vpd(relative_humidity: f32, temperature_celsius: f32) -> f32 {
        let svp = 6.112 * (17.67 * temperature_celsius / (temperature_celsius + 243.5)).exp();
        let avp = relative_humidity / 100.0 * svp;
        
        svp - avp // VPD in hPa
    }
    
    /// Validate humidity reading
    pub fn validate_humidity(humidity: f32) -> bool {
        humidity >= 0.0 && humidity <= 100.0
    }
    
    /// Convert relative humidity at one temperature to another
    pub fn convert_rh_temperature(rh: f32, from_temp: f32, to_temp: f32) -> f32 {
        let svp_from = 6.112 * (17.67 * from_temp / (from_temp + 243.5)).exp();
        let svp_to = 6.112 * (17.67 * to_temp / (to_temp + 243.5)).exp();
        
        let avp = rh / 100.0 * svp_from;
        let new_rh = (avp / svp_to * 100.0).min(100.0);
        
        new_rh
    }
}