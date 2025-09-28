//! Sensor modules for the Sensor Hub
//!
//! This module provides implementations for various types of sensors
//! including temperature, humidity, pressure, and light sensors.
//!
//! # Supported Sensors
//!
//! - **Temperature**: DS18B20, BME280, SHT30
//! - **Humidity**: BME280, SHT30
//! - **Pressure**: BME280, BMP280
//! - **Light**: BH1750, TSL2561
//!
//! # Example
//!
//! ```rust
//! use sensor_hub::sensors::*;
//!
//! // Initialize a temperature sensor
//! let mut temp_sensor = temperature::DS18B20::new(one_wire_pin);
//! let temperature = temp_sensor.read_celsius().await?;
//! ```

pub mod temperature;
pub mod humidity;
pub mod pressure;
pub mod light;

use core::fmt::Debug;
use embassy_time::{Duration, Instant};
use serde::{Deserialize, Serialize};

/// Base trait for all sensors
#[async_trait::async_trait]
pub trait SensorTrait {
    type Error: Debug;
    type Data: Debug + Clone;
    
    /// Initialize the sensor
    async fn init(&mut self) -> Result<(), Self::Error>;
    
    /// Read data from the sensor
    async fn read(&mut self) -> Result<Self::Data, Self::Error>;
    
    /// Get sensor information
    fn get_info(&self) -> SensorInfo;
    
    /// Check if sensor is available
    async fn is_available(&mut self) -> bool {
        self.read().await.is_ok()
    }
    
    /// Enter low power mode
    async fn sleep(&mut self) -> Result<(), Self::Error> {
        // Default implementation - no-op
        Ok(())
    }
    
    /// Wake up from low power mode
    async fn wake(&mut self) -> Result<(), Self::Error> {
        // Default implementation - no-op
        Ok(())
    }
}

/// Sensor information structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorInfo {
    pub id: u8,
    pub name: &'static str,
    pub sensor_type: SensorType,
    pub protocol: Protocol,
    pub address: Option<u8>,
    pub resolution: Option<f32>,
    pub range: Option<(f32, f32)>,
    pub accuracy: Option<f32>,
}

/// Types of sensors supported
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SensorType {
    Temperature,
    Humidity,
    Pressure,
    Light,
    Combined, // For sensors like BME280 that measure multiple parameters
}

/// Communication protocols supported
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Protocol {
    I2C,
    SPI,
    OneWire,
    Analog,
    Digital,
}

/// Unified sensor data structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorData {
    Temperature(f32),  // Celsius
    Humidity(f32),     // %RH
    Pressure(f32),     // hPa
    Light(f32),        // lux
    Combined {
        temperature: Option<f32>,
        humidity: Option<f32>,
        pressure: Option<f32>,
        light: Option<f32>,
    },
    Error,
}

/// Sensor reading with metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorReading {
    pub sensor_id: u8,
    pub timestamp: u64,
    pub data: SensorData,
    pub quality: DataQuality,
}

/// Data quality indicator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DataQuality {
    Good,
    Warning(heapless::String<64>),
    Error(heapless::String<64>),
}

/// Common sensor errors
#[derive(Debug, Clone)]
pub enum SensorError {
    CommunicationError,
    InvalidData,
    SensorNotFound,
    CalibrationError,
    TimeoutError,
    InitializationFailed,
    NotSupported,
}

impl core::fmt::Display for SensorError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SensorError::CommunicationError => write!(f, "Communication error"),
            SensorError::InvalidData => write!(f, "Invalid data received"),
            SensorError::SensorNotFound => write!(f, "Sensor not found"),
            SensorError::CalibrationError => write!(f, "Calibration error"),
            SensorError::TimeoutError => write!(f, "Timeout error"),
            SensorError::InitializationFailed => write!(f, "Initialization failed"),
            SensorError::NotSupported => write!(f, "Operation not supported"),
        }
    }
}

/// Power management trait for sensors
#[async_trait::async_trait]
pub trait PowerManagement {
    type Error;
    
    /// Enter sleep mode
    async fn enter_sleep_mode(&mut self) -> Result<(), Self::Error>;
    
    /// Wake up from sleep mode
    async fn wake_up(&mut self) -> Result<(), Self::Error>;
    
    /// Get power consumption information
    fn get_power_consumption(&self) -> PowerConsumption;
}

/// Power consumption information
#[derive(Debug, Clone)]
pub struct PowerConsumption {
    pub active_current_ua: u32,
    pub sleep_current_ua: u32,
    pub startup_time_ms: u32,
}

/// Calibration trait for sensors
pub trait Calibration {
    type Data;
    type Error;
    
    /// Apply calibration to raw data
    fn apply(&self, raw_data: Self::Data) -> Result<Self::Data, Self::Error>;
    
    /// Load calibration from storage
    fn load_calibration(&mut self) -> Result<(), Self::Error>;
    
    /// Save calibration to storage
    fn save_calibration(&self) -> Result<(), Self::Error>;
}

/// Temperature calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemperatureCalibration {
    pub offset: f32,
    pub scale: f32,
}

impl TemperatureCalibration {
    pub fn new(offset: f32, scale: f32) -> Self {
        Self { offset, scale }
    }
    
    pub fn apply(&self, raw_temp: f32) -> f32 {
        (raw_temp + self.offset) * self.scale
    }
}

/// Pressure calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PressureCalibration {
    pub sea_level_pressure: f32,  // hPa
    pub altitude_offset: f32,     // meters
}

impl PressureCalibration {
    pub fn new(sea_level_pressure: f32, altitude_offset: f32) -> Self {
        Self {
            sea_level_pressure,
            altitude_offset,
        }
    }
    
    pub fn calculate_altitude(&self, pressure: f32) -> f32 {
        let ratio = pressure / self.sea_level_pressure;
        44330.0 * (1.0 - ratio.powf(0.1903)) + self.altitude_offset
    }
}

/// Sensor registry for managing multiple sensors
pub struct SensorRegistry {
    sensors: heapless::Vec<SensorInfo, 16>,
    next_id: u8,
}

impl SensorRegistry {
    pub fn new() -> Self {
        Self {
            sensors: heapless::Vec::new(),
            next_id: 0,
        }
    }
    
    /// Register a new sensor
    pub fn register_sensor(&mut self, mut info: SensorInfo) -> Result<u8, SensorError> {
        info.id = self.next_id;
        self.sensors.push(info)
            .map_err(|_| SensorError::NotSupported)?;
        
        let id = self.next_id;
        self.next_id += 1;
        Ok(id)
    }
    
    /// Get sensor information by ID
    pub fn get_sensor_info(&self, id: u8) -> Option<&SensorInfo> {
        self.sensors.iter().find(|s| s.id == id)
    }
    
    /// List all registered sensors
    pub fn list_sensors(&self) -> &[SensorInfo] {
        &self.sensors
    }
    
    /// Find sensors by type
    pub fn find_by_type(&self, sensor_type: SensorType) -> impl Iterator<Item = &SensorInfo> {
        self.sensors.iter().filter(move |s| s.sensor_type == sensor_type)
    }
    
    /// Find sensors by protocol
    pub fn find_by_protocol(&self, protocol: Protocol) -> impl Iterator<Item = &SensorInfo> {
        self.sensors.iter().filter(move |s| s.protocol == protocol)
    }
}

/// Utility functions
pub mod utils {
    use super::*;
    
    /// Convert Celsius to Fahrenheit
    pub fn celsius_to_fahrenheit(celsius: f32) -> f32 {
        celsius * 9.0 / 5.0 + 32.0
    }
    
    /// Convert Celsius to Kelvin
    pub fn celsius_to_kelvin(celsius: f32) -> f32 {
        celsius + 273.15
    }
    
    /// Convert Fahrenheit to Celsius
    pub fn fahrenheit_to_celsius(fahrenheit: f32) -> f32 {
        (fahrenheit - 32.0) * 5.0 / 9.0
    }
    
    /// Calculate dew point from temperature and humidity
    pub fn calculate_dew_point(temperature: f32, humidity: f32) -> f32 {
        let a = 17.27;
        let b = 237.7;
        let alpha = ((a * temperature) / (b + temperature)) + (humidity / 100.0).ln();
        (b * alpha) / (a - alpha)
    }
    
    /// Calculate heat index from temperature and humidity
    pub fn calculate_heat_index(temperature_f: f32, humidity: f32) -> f32 {
        if temperature_f < 80.0 {
            return temperature_f;
        }
        
        let t = temperature_f;
        let h = humidity;
        
        let hi = -42.379 + 2.04901523 * t + 10.14333127 * h
            - 0.22475541 * t * h - 0.00683783 * t * t
            - 0.05481717 * h * h + 0.00122874 * t * t * h
            + 0.00085282 * t * h * h - 0.00000199 * t * t * h * h;
        
        hi
    }
    
    /// Validate sensor reading
    pub fn validate_reading(reading: &SensorReading) -> bool {
        match &reading.data {
            SensorData::Temperature(temp) => *temp > -273.15 && *temp < 1000.0,
            SensorData::Humidity(hum) => *hum >= 0.0 && *hum <= 100.0,
            SensorData::Pressure(press) => *press > 0.0 && *press < 2000.0,
            SensorData::Light(lux) => *lux >= 0.0,
            SensorData::Combined { .. } => true, // More complex validation needed
            SensorData::Error => false,
        }
    }
    
    /// Get current timestamp (placeholder - implement based on your RTC)
    pub fn get_timestamp() -> u64 {
        // This should be implemented based on your RTC or system timer
        // For now, return a placeholder
        0
    }
}

/// Re-export commonly used types
pub use temperature::*;
pub use humidity::*;
pub use pressure::*;
pub use light::*;