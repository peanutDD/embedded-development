//! Pressure sensor implementations
//!
//! This module provides implementations for various pressure sensors
//! including BMP280, BMP388, and MS5611.

use super::*;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;

/// Pressure sensor trait
#[async_trait::async_trait]
pub trait PressureSensor: SensorTrait {
    /// Read atmospheric pressure in hPa (hectopascals)
    async fn read_pressure(&mut self) -> Result<f32, Self::Error>;
    
    /// Read temperature from pressure sensor (if available)
    async fn read_temperature(&mut self) -> Result<f32, Self::Error> {
        Err(SensorError::NotSupported)
    }
    
    /// Read both pressure and temperature (for combined sensors)
    async fn read_pressure_temperature(&mut self) -> Result<(f32, f32), Self::Error> {
        let pressure = self.read_pressure().await?;
        let temperature = self.read_temperature().await.unwrap_or(25.0);
        Ok((pressure, temperature))
    }
    
    /// Calculate altitude from pressure (using standard atmosphere model)
    async fn calculate_altitude(&mut self, sea_level_pressure: f32) -> Result<f32, Self::Error> {
        let pressure = self.read_pressure().await?;
        Ok(super::utils::calculate_altitude_from_pressure(pressure, sea_level_pressure))
    }
    
    /// Get pressure measurement range in hPa
    fn get_pressure_range(&self) -> (f32, f32) {
        (300.0, 1100.0) // Default range for atmospheric pressure
    }
    
    /// Get pressure accuracy in hPa
    fn get_pressure_accuracy(&self) -> f32 {
        1.0 // Default ±1 hPa
    }
    
    /// Get pressure resolution in hPa
    fn get_pressure_resolution(&self) -> f32 {
        0.01 // Default 0.01 hPa
    }
}

/// BMP280 I2C/SPI pressure and temperature sensor
pub struct BMP280<I> {
    interface: I,
    calibration: BMP280Calibration,
    config: BMP280Config,
    info: SensorInfo,
}

/// BMP280 measurement data
#[derive(Debug, Clone)]
pub struct BMP280Data {
    pub pressure: f32,    // hPa
    pub temperature: f32, // °C
    pub altitude: Option<f32>, // meters (if sea level pressure is known)
}

/// BMP280 calibration coefficients
#[derive(Debug, Clone, Default)]
pub struct BMP280Calibration {
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,
}

/// BMP280 configuration
#[derive(Debug, Clone)]
pub struct BMP280Config {
    pub temperature_oversampling: BMP280Oversampling,
    pub pressure_oversampling: BMP280Oversampling,
    pub iir_filter: BMP280IIRFilter,
    pub standby_time: BMP280StandbyTime,
    pub mode: BMP280Mode,
}

/// BMP280 oversampling settings
#[derive(Debug, Clone, Copy)]
pub enum BMP280Oversampling {
    Skip = 0,
    X1 = 1,
    X2 = 2,
    X4 = 3,
    X8 = 4,
    X16 = 5,
}

/// BMP280 IIR filter settings
#[derive(Debug, Clone, Copy)]
pub enum BMP280IIRFilter {
    Off = 0,
    X2 = 1,
    X4 = 2,
    X8 = 3,
    X16 = 4,
}

/// BMP280 standby time settings
#[derive(Debug, Clone, Copy)]
pub enum BMP280StandbyTime {
    Ms0_5 = 0,
    Ms62_5 = 1,
    Ms125 = 2,
    Ms250 = 3,
    Ms500 = 4,
    Ms1000 = 5,
    Ms2000 = 6,
    Ms4000 = 7,
}

/// BMP280 operating mode
#[derive(Debug, Clone, Copy)]
pub enum BMP280Mode {
    Sleep = 0,
    Forced = 1,
    Normal = 3,
}

// BMP280 register addresses
const BMP280_REG_DIG_T1: u8 = 0x88;
const BMP280_REG_DIG_P1: u8 = 0x8E;
const BMP280_REG_CHIPID: u8 = 0xD0;
const BMP280_REG_RESET: u8 = 0xE0;
const BMP280_REG_STATUS: u8 = 0xF3;
const BMP280_REG_CTRL_MEAS: u8 = 0xF4;
const BMP280_REG_CONFIG: u8 = 0xF5;
const BMP280_REG_PRESSUREDATA: u8 = 0xF7;
const BMP280_REG_TEMPDATA: u8 = 0xFA;

const BMP280_CHIPID: u8 = 0x58;
const BMP280_RESET_VALUE: u8 = 0xB6;

impl Default for BMP280Config {
    fn default() -> Self {
        Self {
            temperature_oversampling: BMP280Oversampling::X2,
            pressure_oversampling: BMP280Oversampling::X16,
            iir_filter: BMP280IIRFilter::X4,
            standby_time: BMP280StandbyTime::Ms125,
            mode: BMP280Mode::Normal,
        }
    }
}

impl<I> BMP280<I>
where
    I: I2c,
{
    /// Create new BMP280 instance with I2C interface
    pub fn new_i2c(i2c: I, address: u8) -> Self {
        Self {
            interface: i2c,
            calibration: BMP280Calibration::default(),
            config: BMP280Config::default(),
            info: SensorInfo {
                id: 0,
                name: "BMP280",
                sensor_type: SensorType::Combined,
                protocol: Protocol::I2C,
                address: Some(address),
                resolution: Some(0.01), // 0.01 hPa
                range: Some((300.0, 1100.0)), // 300-1100 hPa
                accuracy: Some(1.0), // ±1 hPa
            },
        }
    }
    
    /// Initialize BMP280 sensor
    pub async fn initialize(&mut self) -> Result<(), SensorError> {
        // Check chip ID
        let chip_id = self.read_register(BMP280_REG_CHIPID).await?;
        if chip_id != BMP280_CHIPID {
            return Err(SensorError::InvalidData);
        }
        
        // Soft reset
        self.write_register(BMP280_REG_RESET, BMP280_RESET_VALUE).await?;
        Timer::after(Duration::from_millis(10)).await;
        
        // Read calibration coefficients
        self.read_calibration().await?;
        
        // Configure sensor
        self.configure().await?;
        
        Ok(())
    }
    
    /// Read calibration coefficients from sensor
    async fn read_calibration(&mut self) -> Result<(), SensorError> {
        // Read temperature calibration coefficients
        let mut temp_cal = [0u8; 6];
        self.read_registers(BMP280_REG_DIG_T1, &mut temp_cal).await?;
        
        self.calibration.dig_t1 = u16::from_le_bytes([temp_cal[0], temp_cal[1]]);
        self.calibration.dig_t2 = i16::from_le_bytes([temp_cal[2], temp_cal[3]]);
        self.calibration.dig_t3 = i16::from_le_bytes([temp_cal[4], temp_cal[5]]);
        
        // Read pressure calibration coefficients
        let mut press_cal = [0u8; 18];
        self.read_registers(BMP280_REG_DIG_P1, &mut press_cal).await?;
        
        self.calibration.dig_p1 = u16::from_le_bytes([press_cal[0], press_cal[1]]);
        self.calibration.dig_p2 = i16::from_le_bytes([press_cal[2], press_cal[3]]);
        self.calibration.dig_p3 = i16::from_le_bytes([press_cal[4], press_cal[5]]);
        self.calibration.dig_p4 = i16::from_le_bytes([press_cal[6], press_cal[7]]);
        self.calibration.dig_p5 = i16::from_le_bytes([press_cal[8], press_cal[9]]);
        self.calibration.dig_p6 = i16::from_le_bytes([press_cal[10], press_cal[11]]);
        self.calibration.dig_p7 = i16::from_le_bytes([press_cal[12], press_cal[13]]);
        self.calibration.dig_p8 = i16::from_le_bytes([press_cal[14], press_cal[15]]);
        self.calibration.dig_p9 = i16::from_le_bytes([press_cal[16], press_cal[17]]);
        
        Ok(())
    }
    
    /// Configure sensor with current settings
    async fn configure(&mut self) -> Result<(), SensorError> {
        // Configure measurement control register
        let ctrl_meas = (self.config.temperature_oversampling as u8) << 5
            | (self.config.pressure_oversampling as u8) << 2
            | (self.config.mode as u8);
        
        self.write_register(BMP280_REG_CTRL_MEAS, ctrl_meas).await?;
        
        // Configure config register
        let config = (self.config.standby_time as u8) << 5
            | (self.config.iir_filter as u8) << 2;
        
        self.write_register(BMP280_REG_CONFIG, config).await?;
        
        Ok(())
    }
    
    /// Read raw temperature and pressure data
    async fn read_raw_data(&mut self) -> Result<(i32, i32), SensorError> {
        // Check if measurement is ready
        let status = self.read_register(BMP280_REG_STATUS).await?;
        if status & 0x08 != 0 { // measuring bit
            return Err(SensorError::NotReady);
        }
        
        // Read pressure and temperature data (6 bytes)
        let mut data = [0u8; 6];
        self.read_registers(BMP280_REG_PRESSUREDATA, &mut data).await?;
        
        // Convert to 20-bit values
        let pressure_raw = ((data[0] as i32) << 12) | ((data[1] as i32) << 4) | ((data[2] as i32) >> 4);
        let temperature_raw = ((data[3] as i32) << 12) | ((data[4] as i32) << 4) | ((data[5] as i32) >> 4);
        
        Ok((pressure_raw, temperature_raw))
    }
    
    /// Compensate temperature reading using calibration data
    fn compensate_temperature(&self, raw_temp: i32) -> (f32, i32) {
        let var1 = (raw_temp / 16384 - (self.calibration.dig_t1 as i32) / 1024) * (self.calibration.dig_t2 as i32);
        let var2 = ((raw_temp / 131072 - (self.calibration.dig_t1 as i32) / 8192)
            * (raw_temp / 131072 - (self.calibration.dig_t1 as i32) / 8192)) / 4096
            * (self.calibration.dig_t3 as i32);
        
        let t_fine = var1 + var2;
        let temperature = (t_fine * 5 + 128) / 256;
        
        (temperature as f32 / 100.0, t_fine)
    }
    
    /// Compensate pressure reading using calibration data
    fn compensate_pressure(&self, raw_pressure: i32, t_fine: i32) -> f32 {
        let mut var1 = (t_fine as i64) / 2 - 64000;
        let mut var2 = var1 * var1 * (self.calibration.dig_p6 as i64) / 32768;
        var2 = var2 + var1 * (self.calibration.dig_p5 as i64) * 2;
        var2 = var2 / 4 + (self.calibration.dig_p4 as i64) * 65536;
        var1 = ((self.calibration.dig_p3 as i64) * var1 * var1 / 524288 + (self.calibration.dig_p2 as i64) * var1) / 524288;
        var1 = (1 + var1 / 32768) * (self.calibration.dig_p1 as i64);
        
        if var1 == 0 {
            return 0.0; // Avoid division by zero
        }
        
        let mut pressure = 1048576 - raw_pressure as i64;
        pressure = (pressure - var2 / 4096) * 6250 / var1;
        var1 = (self.calibration.dig_p9 as i64) * pressure * pressure / 2147483648;
        var2 = pressure * (self.calibration.dig_p8 as i64) / 32768;
        pressure = pressure + (var1 + var2 + (self.calibration.dig_p7 as i64)) / 16;
        
        pressure as f32 / 100.0 // Convert Pa to hPa
    }
    
    /// Read compensated temperature and pressure
    pub async fn read_pressure_temperature(&mut self) -> Result<BMP280Data, SensorError> {
        // For forced mode, trigger measurement
        if matches!(self.config.mode, BMP280Mode::Forced) {
            let ctrl_meas = (self.config.temperature_oversampling as u8) << 5
                | (self.config.pressure_oversampling as u8) << 2
                | (BMP280Mode::Forced as u8);
            self.write_register(BMP280_REG_CTRL_MEAS, ctrl_meas).await?;
            
            // Wait for measurement to complete
            Timer::after(Duration::from_millis(50)).await;
        }
        
        let (raw_pressure, raw_temperature) = self.read_raw_data().await?;
        
        let (temperature, t_fine) = self.compensate_temperature(raw_temperature);
        let pressure = self.compensate_pressure(raw_pressure, t_fine);
        
        Ok(BMP280Data {
            pressure,
            temperature,
            altitude: None,
        })
    }
    
    /// Read only pressure
    pub async fn read_pressure(&mut self) -> Result<f32, SensorError> {
        let data = self.read_pressure_temperature().await?;
        Ok(data.pressure)
    }
    
    /// Read only temperature
    pub async fn read_temperature(&mut self) -> Result<f32, SensorError> {
        let data = self.read_pressure_temperature().await?;
        Ok(data.temperature)
    }
    
    /// Set sensor configuration
    pub async fn set_config(&mut self, config: BMP280Config) -> Result<(), SensorError> {
        self.config = config;
        self.configure().await
    }
    
    /// Get current configuration
    pub fn get_config(&self) -> &BMP280Config {
        &self.config
    }
    
    /// Read single register
    async fn read_register(&mut self, reg: u8) -> Result<u8, SensorError> {
        let mut buffer = [0u8; 1];
        self.interface.write_read(self.info.address.unwrap(), &[reg], &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        Ok(buffer[0])
    }
    
    /// Write single register
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), SensorError> {
        self.interface.write(self.info.address.unwrap(), &[reg, value]).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Read multiple registers
    async fn read_registers(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), SensorError> {
        self.interface.write_read(self.info.address.unwrap(), &[reg], buffer).await
            .map_err(|_| SensorError::CommunicationError)
    }
}

#[async_trait::async_trait]
impl<I> SensorTrait for BMP280<I>
where
    I: I2c,
{
    type Error = SensorError;
    type Data = BMP280Data;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        self.initialize().await
    }
    
    async fn read(&mut self) -> Result<Self::Data, Self::Error> {
        self.read_pressure_temperature().await
    }
    
    fn get_info(&self) -> SensorInfo {
        self.info.clone()
    }
    
    async fn sleep(&mut self) -> Result<(), Self::Error> {
        // Set to sleep mode
        let mut config = self.config.clone();
        config.mode = BMP280Mode::Sleep;
        self.set_config(config).await
    }
}

#[async_trait::async_trait]
impl<I> PressureSensor for BMP280<I>
where
    I: I2c,
{
    async fn read_pressure(&mut self) -> Result<f32, Self::Error> {
        self.read_pressure().await
    }
    
    async fn read_temperature(&mut self) -> Result<f32, Self::Error> {
        self.read_temperature().await
    }
    
    async fn read_pressure_temperature(&mut self) -> Result<(f32, f32), Self::Error> {
        let data = self.read_pressure_temperature().await?;
        Ok((data.pressure, data.temperature))
    }
    
    fn get_pressure_range(&self) -> (f32, f32) {
        (300.0, 1100.0)
    }
    
    fn get_pressure_accuracy(&self) -> f32 {
        1.0 // ±1 hPa
    }
    
    fn get_pressure_resolution(&self) -> f32 {
        0.01 // 0.01 hPa
    }
}

/// MS5611 SPI/I2C pressure sensor
pub struct MS5611<I> {
    interface: I,
    calibration: MS5611Calibration,
    info: SensorInfo,
}

/// MS5611 calibration coefficients
#[derive(Debug, Clone, Default)]
pub struct MS5611Calibration {
    pub c1: u16, // Pressure sensitivity
    pub c2: u16, // Pressure offset
    pub c3: u16, // Temperature coefficient of pressure sensitivity
    pub c4: u16, // Temperature coefficient of pressure offset
    pub c5: u16, // Reference temperature
    pub c6: u16, // Temperature coefficient of temperature
}

/// MS5611 measurement data
#[derive(Debug, Clone)]
pub struct MS5611Data {
    pub pressure: f32,    // hPa
    pub temperature: f32, // °C
}

/// MS5611 oversampling ratio
#[derive(Debug, Clone, Copy)]
pub enum MS5611OSR {
    OSR256 = 0x40,
    OSR512 = 0x42,
    OSR1024 = 0x44,
    OSR2048 = 0x46,
    OSR4096 = 0x48,
}

// MS5611 commands
const MS5611_CMD_RESET: u8 = 0x1E;
const MS5611_CMD_CONVERT_D1: u8 = 0x40; // Pressure
const MS5611_CMD_CONVERT_D2: u8 = 0x50; // Temperature
const MS5611_CMD_ADC_READ: u8 = 0x00;
const MS5611_CMD_PROM_READ: u8 = 0xA0;

impl<I> MS5611<I>
where
    I: I2c,
{
    /// Create new MS5611 instance
    pub fn new(interface: I, address: u8) -> Self {
        Self {
            interface,
            calibration: MS5611Calibration::default(),
            info: SensorInfo {
                id: 0,
                name: "MS5611",
                sensor_type: SensorType::Combined,
                protocol: Protocol::I2C,
                address: Some(address),
                resolution: Some(0.012), // 0.012 hPa
                range: Some((10.0, 1200.0)), // 10-1200 hPa
                accuracy: Some(1.5), // ±1.5 hPa
            },
        }
    }
    
    /// Initialize MS5611 sensor
    pub async fn initialize(&mut self) -> Result<(), SensorError> {
        // Reset sensor
        self.send_command(MS5611_CMD_RESET).await?;
        Timer::after(Duration::from_millis(3)).await;
        
        // Read calibration coefficients
        self.read_calibration().await?;
        
        Ok(())
    }
    
    /// Read calibration coefficients from PROM
    async fn read_calibration(&mut self) -> Result<(), SensorError> {
        let mut coefficients = [0u16; 6];
        
        for i in 0..6 {
            let cmd = MS5611_CMD_PROM_READ + ((i + 1) << 1) as u8;
            let data = self.read_prom(cmd).await?;
            coefficients[i] = data;
        }
        
        self.calibration.c1 = coefficients[0];
        self.calibration.c2 = coefficients[1];
        self.calibration.c3 = coefficients[2];
        self.calibration.c4 = coefficients[3];
        self.calibration.c5 = coefficients[4];
        self.calibration.c6 = coefficients[5];
        
        Ok(())
    }
    
    /// Read PROM data
    async fn read_prom(&mut self, cmd: u8) -> Result<u16, SensorError> {
        let mut buffer = [0u8; 2];
        self.interface.write_read(self.info.address.unwrap(), &[cmd], &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        Ok(u16::from_be_bytes(buffer))
    }
    
    /// Send command to sensor
    async fn send_command(&mut self, cmd: u8) -> Result<(), SensorError> {
        self.interface.write(self.info.address.unwrap(), &[cmd]).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Read ADC result
    async fn read_adc(&mut self) -> Result<u32, SensorError> {
        let mut buffer = [0u8; 3];
        self.interface.write_read(self.info.address.unwrap(), &[MS5611_CMD_ADC_READ], &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        
        Ok(((buffer[0] as u32) << 16) | ((buffer[1] as u32) << 8) | (buffer[2] as u32))
    }
    
    /// Convert and read pressure
    async fn convert_pressure(&mut self, osr: MS5611OSR) -> Result<u32, SensorError> {
        self.send_command(MS5611_CMD_CONVERT_D1 | osr as u8).await?;
        
        // Wait for conversion to complete
        let wait_time = match osr {
            MS5611OSR::OSR256 => Duration::from_millis(1),
            MS5611OSR::OSR512 => Duration::from_millis(2),
            MS5611OSR::OSR1024 => Duration::from_millis(3),
            MS5611OSR::OSR2048 => Duration::from_millis(5),
            MS5611OSR::OSR4096 => Duration::from_millis(10),
        };
        Timer::after(wait_time).await;
        
        self.read_adc().await
    }
    
    /// Convert and read temperature
    async fn convert_temperature(&mut self, osr: MS5611OSR) -> Result<u32, SensorError> {
        self.send_command(MS5611_CMD_CONVERT_D2 | osr as u8).await?;
        
        // Wait for conversion to complete
        let wait_time = match osr {
            MS5611OSR::OSR256 => Duration::from_millis(1),
            MS5611OSR::OSR512 => Duration::from_millis(2),
            MS5611OSR::OSR1024 => Duration::from_millis(3),
            MS5611OSR::OSR2048 => Duration::from_millis(5),
            MS5611OSR::OSR4096 => Duration::from_millis(10),
        };
        Timer::after(wait_time).await;
        
        self.read_adc().await
    }
    
    /// Read compensated pressure and temperature
    pub async fn read_pressure_temperature(&mut self) -> Result<MS5611Data, SensorError> {
        let osr = MS5611OSR::OSR4096; // Use highest resolution
        
        // Read raw values
        let d1 = self.convert_pressure(osr).await?;
        let d2 = self.convert_temperature(osr).await?;
        
        // Calculate temperature
        let dt = d2 as i64 - (self.calibration.c5 as i64) << 8;
        let temp = 2000 + dt * (self.calibration.c6 as i64) / (1i64 << 23);
        
        // Calculate pressure
        let off = (self.calibration.c2 as i64) << 16 + (self.calibration.c4 as i64) * dt / (1i64 << 7);
        let sens = (self.calibration.c1 as i64) << 15 + (self.calibration.c3 as i64) * dt / (1i64 << 8);
        
        // Second order temperature compensation
        let (temp, off, sens) = if temp < 2000 {
            let t2 = dt * dt / (1i64 << 31);
            let off2 = 5 * (temp - 2000) * (temp - 2000) / 2;
            let sens2 = 5 * (temp - 2000) * (temp - 2000) / 4;
            
            let (off2, sens2) = if temp < -1500 {
                let off2 = off2 + 7 * (temp + 1500) * (temp + 1500);
                let sens2 = sens2 + 11 * (temp + 1500) * (temp + 1500) / 2;
                (off2, sens2)
            } else {
                (off2, sens2)
            };
            
            (temp - t2, off - off2, sens - sens2)
        } else {
            (temp, off, sens)
        };
        
        let pressure = (d1 as i64 * sens / (1i64 << 21) - off) / (1i64 << 15);
        
        Ok(MS5611Data {
            pressure: pressure as f32 / 100.0, // Convert Pa to hPa
            temperature: temp as f32 / 100.0,  // Convert to °C
        })
    }
}

#[async_trait::async_trait]
impl<I> SensorTrait for MS5611<I>
where
    I: I2c,
{
    type Error = SensorError;
    type Data = MS5611Data;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        self.initialize().await
    }
    
    async fn read(&mut self) -> Result<Self::Data, Self::Error> {
        self.read_pressure_temperature().await
    }
    
    fn get_info(&self) -> SensorInfo {
        self.info.clone()
    }
}

#[async_trait::async_trait]
impl<I> PressureSensor for MS5611<I>
where
    I: I2c,
{
    async fn read_pressure(&mut self) -> Result<f32, Self::Error> {
        let data = self.read_pressure_temperature().await?;
        Ok(data.pressure)
    }
    
    async fn read_temperature(&mut self) -> Result<f32, Self::Error> {
        let data = self.read_pressure_temperature().await?;
        Ok(data.temperature)
    }
    
    async fn read_pressure_temperature(&mut self) -> Result<(f32, f32), Self::Error> {
        let data = self.read_pressure_temperature().await?;
        Ok((data.pressure, data.temperature))
    }
    
    fn get_pressure_range(&self) -> (f32, f32) {
        (10.0, 1200.0)
    }
    
    fn get_pressure_accuracy(&self) -> f32 {
        1.5 // ±1.5 hPa
    }
    
    fn get_pressure_resolution(&self) -> f32 {
        0.012 // 0.012 hPa
    }
}

/// Pressure sensor manager
pub struct PressureSensorManager {
    sensors: heapless::Vec<SensorInfo, 8>,
    calibrations: heapless::FnvIndexMap<u8, PressureCalibration, 8>,
    sea_level_pressure: f32,
}

/// Pressure calibration data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PressureCalibration {
    pub offset: f32,
    pub scale: f32,
    pub temperature_coefficient: f32,
}

impl PressureCalibration {
    pub fn new(offset: f32, scale: f32) -> Self {
        Self {
            offset,
            scale,
            temperature_coefficient: 0.0,
        }
    }
    
    /// Apply calibration with temperature compensation
    pub fn apply(&self, raw_pressure: f32, temperature: f32) -> f32 {
        let temp_compensation = self.temperature_coefficient * (temperature - 25.0);
        (raw_pressure + self.offset) * self.scale + temp_compensation
    }
}

impl PressureSensorManager {
    pub fn new() -> Self {
        Self {
            sensors: heapless::Vec::new(),
            calibrations: heapless::FnvIndexMap::new(),
            sea_level_pressure: 1013.25, // Standard sea level pressure
        }
    }
    
    /// Set sea level pressure for altitude calculations
    pub fn set_sea_level_pressure(&mut self, pressure: f32) {
        self.sea_level_pressure = pressure;
    }
    
    /// Get current sea level pressure setting
    pub fn get_sea_level_pressure(&self) -> f32 {
        self.sea_level_pressure
    }
    
    /// Register a pressure sensor
    pub fn register_sensor(&mut self, sensor_info: SensorInfo) -> Result<(), SensorError> {
        self.sensors.push(sensor_info)
            .map_err(|_| SensorError::NotSupported)
    }
    
    /// Set calibration for a sensor
    pub fn set_calibration(&mut self, sensor_id: u8, calibration: PressureCalibration) -> Result<(), SensorError> {
        self.calibrations.insert(sensor_id, calibration)
            .map_err(|_| SensorError::NotSupported)?;
        Ok(())
    }
    
    /// Apply calibration to a reading
    pub fn apply_calibration(&self, sensor_id: u8, raw_pressure: f32, temperature: f32) -> f32 {
        if let Some(cal) = self.calibrations.get(&sensor_id) {
            cal.apply(raw_pressure, temperature)
        } else {
            raw_pressure
        }
    }
    
    /// Calculate altitude from pressure
    pub fn calculate_altitude(&self, pressure: f32) -> f32 {
        super::utils::calculate_altitude_from_pressure(pressure, self.sea_level_pressure)
    }
}

/// Pressure data processing utilities
pub mod pressure_utils {
    use super::*;
    
    /// Calculate altitude from pressure using barometric formula
    pub fn calculate_altitude_from_pressure(pressure: f32, sea_level_pressure: f32) -> f32 {
        44330.0 * (1.0 - (pressure / sea_level_pressure).powf(0.1903))
    }
    
    /// Calculate sea level pressure from altitude and current pressure
    pub fn calculate_sea_level_pressure(pressure: f32, altitude: f32) -> f32 {
        pressure / (1.0 - altitude / 44330.0).powf(5.255)
    }
    
    /// Convert pressure units
    pub fn hpa_to_mmhg(hpa: f32) -> f32 {
        hpa * 0.750062
    }
    
    pub fn hpa_to_inhg(hpa: f32) -> f32 {
        hpa * 0.02953
    }
    
    pub fn hpa_to_psi(hpa: f32) -> f32 {
        hpa * 0.0145038
    }
    
    /// Validate pressure reading
    pub fn validate_pressure(pressure: f32) -> bool {
        pressure >= 10.0 && pressure <= 1200.0
    }
    
    /// Calculate pressure trend
    pub fn calculate_pressure_trend(readings: &[f32], time_window_hours: f32) -> PressureTrend {
        if readings.len() < 2 {
            return PressureTrend::Stable;
        }
        
        let first = readings[0];
        let last = readings[readings.len() - 1];
        let change = last - first;
        let rate = change / time_window_hours;
        
        match rate {
            r if r > 1.0 => PressureTrend::Rising,
            r if r < -1.0 => PressureTrend::Falling,
            _ => PressureTrend::Stable,
        }
    }
}

/// Pressure trend indication
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PressureTrend {
    Rising,
    Stable,
    Falling,
}