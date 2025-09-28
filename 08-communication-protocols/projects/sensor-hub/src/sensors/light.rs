//! Light sensor implementations
//!
//! This module provides implementations for various light sensors
//! including BH1750, TSL2561, and VEML7700.

use super::*;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

/// Light sensor trait
#[async_trait::async_trait]
pub trait LightSensor: SensorTrait {
    /// Read illuminance in lux
    async fn read_lux(&mut self) -> Result<f32, Self::Error>;
    
    /// Read raw light intensity (sensor-specific units)
    async fn read_raw(&mut self) -> Result<u16, Self::Error> {
        Err(SensorError::NotSupported)
    }
    
    /// Get measurement range in lux
    fn get_lux_range(&self) -> (f32, f32) {
        (0.0, 65535.0) // Default range
    }
    
    /// Get measurement resolution in lux
    fn get_lux_resolution(&self) -> f32 {
        1.0 // Default 1 lux
    }
    
    /// Get measurement accuracy in percentage
    fn get_accuracy(&self) -> f32 {
        5.0 // Default ±5%
    }
    
    /// Set measurement time/integration time
    async fn set_integration_time(&mut self, _time_ms: u16) -> Result<(), Self::Error> {
        Err(SensorError::NotSupported)
    }
    
    /// Set gain/sensitivity
    async fn set_gain(&mut self, _gain: u8) -> Result<(), Self::Error> {
        Err(SensorError::NotSupported)
    }
    
    /// Check if sensor supports automatic gain control
    fn supports_auto_gain(&self) -> bool {
        false
    }
    
    /// Enable/disable automatic gain control
    async fn set_auto_gain(&mut self, _enable: bool) -> Result<(), Self::Error> {
        Err(SensorError::NotSupported)
    }
}

/// BH1750 I2C ambient light sensor
pub struct BH1750<I> {
    interface: I,
    mode: BH1750Mode,
    info: SensorInfo,
}

/// BH1750 measurement modes
#[derive(Debug, Clone, Copy)]
pub enum BH1750Mode {
    /// Continuously H-Resolution Mode (1 lx resolution, 120ms measurement time)
    ContinuousHighRes = 0x10,
    /// Continuously H-Resolution Mode2 (0.5 lx resolution, 120ms measurement time)
    ContinuousHighRes2 = 0x11,
    /// Continuously L-Resolution Mode (4 lx resolution, 16ms measurement time)
    ContinuousLowRes = 0x13,
    /// One Time H-Resolution Mode (1 lx resolution, 120ms measurement time)
    OneTimeHighRes = 0x20,
    /// One Time H-Resolution Mode2 (0.5 lx resolution, 120ms measurement time)
    OneTimeHighRes2 = 0x21,
    /// One Time L-Resolution Mode (4 lx resolution, 16ms measurement time)
    OneTimeLowRes = 0x23,
}

/// BH1750 commands
const BH1750_POWER_DOWN: u8 = 0x00;
const BH1750_POWER_ON: u8 = 0x01;
const BH1750_RESET: u8 = 0x07;

/// BH1750 measurement data
#[derive(Debug, Clone)]
pub struct BH1750Data {
    pub lux: f32,
    pub raw: u16,
    pub mode: BH1750Mode,
}

impl<I> BH1750<I>
where
    I: I2c,
{
    /// Create new BH1750 instance
    pub fn new(interface: I, address: u8) -> Self {
        Self {
            interface,
            mode: BH1750Mode::ContinuousHighRes,
            info: SensorInfo {
                id: 0,
                name: "BH1750",
                sensor_type: SensorType::Light,
                protocol: Protocol::I2C,
                address: Some(address),
                resolution: Some(1.0), // 1 lux
                range: Some((0.0, 65535.0)), // 0-65535 lux
                accuracy: Some(20.0), // ±20%
            },
        }
    }
    
    /// Initialize BH1750 sensor
    pub async fn initialize(&mut self) -> Result<(), SensorError> {
        // Power on
        self.send_command(BH1750_POWER_ON).await?;
        Timer::after(Duration::from_millis(10)).await;
        
        // Reset
        self.send_command(BH1750_RESET).await?;
        Timer::after(Duration::from_millis(10)).await;
        
        // Set default mode
        self.set_mode(self.mode).await?;
        
        Ok(())
    }
    
    /// Set measurement mode
    pub async fn set_mode(&mut self, mode: BH1750Mode) -> Result<(), SensorError> {
        self.mode = mode;
        self.send_command(mode as u8).await?;
        
        // Wait for measurement to be ready
        let wait_time = match mode {
            BH1750Mode::ContinuousLowRes | BH1750Mode::OneTimeLowRes => Duration::from_millis(24),
            _ => Duration::from_millis(180), // High resolution modes
        };
        Timer::after(wait_time).await;
        
        Ok(())
    }
    
    /// Read light measurement
    pub async fn read_light(&mut self) -> Result<BH1750Data, SensorError> {
        // For one-time modes, send measurement command
        if matches!(self.mode, BH1750Mode::OneTimeHighRes | BH1750Mode::OneTimeHighRes2 | BH1750Mode::OneTimeLowRes) {
            self.send_command(self.mode as u8).await?;
            
            let wait_time = match self.mode {
                BH1750Mode::OneTimeLowRes => Duration::from_millis(24),
                _ => Duration::from_millis(180),
            };
            Timer::after(wait_time).await;
        }
        
        // Read 2 bytes of data
        let mut buffer = [0u8; 2];
        self.interface.read(self.info.address.unwrap(), &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        
        let raw = u16::from_be_bytes(buffer);
        
        // Convert to lux based on mode
        let lux = match self.mode {
            BH1750Mode::ContinuousHighRes2 | BH1750Mode::OneTimeHighRes2 => {
                raw as f32 / 2.4 // 0.5 lx resolution
            }
            BH1750Mode::ContinuousLowRes | BH1750Mode::OneTimeLowRes => {
                raw as f32 / 3.0 // 4 lx resolution (approximately)
            }
            _ => raw as f32 / 1.2, // 1 lx resolution
        };
        
        Ok(BH1750Data {
            lux,
            raw,
            mode: self.mode,
        })
    }
    
    /// Read only lux value
    pub async fn read_lux(&mut self) -> Result<f32, SensorError> {
        let data = self.read_light().await?;
        Ok(data.lux)
    }
    
    /// Send command to sensor
    async fn send_command(&mut self, command: u8) -> Result<(), SensorError> {
        self.interface.write(self.info.address.unwrap(), &[command]).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Power down sensor
    pub async fn power_down(&mut self) -> Result<(), SensorError> {
        self.send_command(BH1750_POWER_DOWN).await
    }
    
    /// Power up sensor
    pub async fn power_up(&mut self) -> Result<(), SensorError> {
        self.send_command(BH1750_POWER_ON).await?;
        Timer::after(Duration::from_millis(10)).await;
        Ok(())
    }
}

#[async_trait::async_trait]
impl<I> SensorTrait for BH1750<I>
where
    I: I2c,
{
    type Error = SensorError;
    type Data = BH1750Data;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        self.initialize().await
    }
    
    async fn read(&mut self) -> Result<Self::Data, Self::Error> {
        self.read_light().await
    }
    
    fn get_info(&self) -> SensorInfo {
        self.info.clone()
    }
    
    async fn sleep(&mut self) -> Result<(), Self::Error> {
        self.power_down().await
    }
}

#[async_trait::async_trait]
impl<I> LightSensor for BH1750<I>
where
    I: I2c,
{
    async fn read_lux(&mut self) -> Result<f32, Self::Error> {
        self.read_lux().await
    }
    
    async fn read_raw(&mut self) -> Result<u16, Self::Error> {
        let data = self.read_light().await?;
        Ok(data.raw)
    }
    
    fn get_lux_range(&self) -> (f32, f32) {
        (0.0, 65535.0)
    }
    
    fn get_lux_resolution(&self) -> f32 {
        match self.mode {
            BH1750Mode::ContinuousHighRes2 | BH1750Mode::OneTimeHighRes2 => 0.5,
            BH1750Mode::ContinuousLowRes | BH1750Mode::OneTimeLowRes => 4.0,
            _ => 1.0,
        }
    }
    
    fn get_accuracy(&self) -> f32 {
        20.0 // ±20%
    }
}

/// TSL2561 I2C light-to-digital converter
pub struct TSL2561<I> {
    interface: I,
    gain: TSL2561Gain,
    integration_time: TSL2561IntegrationTime,
    info: SensorInfo,
}

/// TSL2561 gain settings
#[derive(Debug, Clone, Copy)]
pub enum TSL2561Gain {
    Low = 0x00,  // 1x gain
    High = 0x10, // 16x gain
}

/// TSL2561 integration time settings
#[derive(Debug, Clone, Copy)]
pub enum TSL2561IntegrationTime {
    Ms13 = 0x00,  // 13.7ms
    Ms101 = 0x01, // 101ms
    Ms402 = 0x02, // 402ms
}

/// TSL2561 measurement data
#[derive(Debug, Clone)]
pub struct TSL2561Data {
    pub lux: f32,
    pub ch0: u16, // Broadband (visible + IR)
    pub ch1: u16, // IR only
    pub ratio: f32, // CH1/CH0 ratio
}

// TSL2561 register addresses
const TSL2561_REG_CONTROL: u8 = 0x00;
const TSL2561_REG_TIMING: u8 = 0x01;
const TSL2561_REG_ID: u8 = 0x0A;
const TSL2561_REG_DATA0LOW: u8 = 0x0C;
const TSL2561_REG_DATA0HIGH: u8 = 0x0D;
const TSL2561_REG_DATA1LOW: u8 = 0x0E;
const TSL2561_REG_DATA1HIGH: u8 = 0x0F;

// TSL2561 commands
const TSL2561_CMD: u8 = 0x80;
const TSL2561_CMD_WORD: u8 = 0xA0;

// TSL2561 control values
const TSL2561_POWER_UP: u8 = 0x03;
const TSL2561_POWER_DOWN: u8 = 0x00;

impl<I> TSL2561<I>
where
    I: I2c,
{
    /// Create new TSL2561 instance
    pub fn new(interface: I, address: u8) -> Self {
        Self {
            interface,
            gain: TSL2561Gain::Low,
            integration_time: TSL2561IntegrationTime::Ms402,
            info: SensorInfo {
                id: 0,
                name: "TSL2561",
                sensor_type: SensorType::Light,
                protocol: Protocol::I2C,
                address: Some(address),
                resolution: Some(0.1), // 0.1 lux
                range: Some((0.1, 40000.0)), // 0.1-40000 lux
                accuracy: Some(3.0), // ±3%
            },
        }
    }
    
    /// Initialize TSL2561 sensor
    pub async fn initialize(&mut self) -> Result<(), SensorError> {
        // Check device ID
        let id = self.read_register(TSL2561_REG_ID).await?;
        if (id & 0xF0) != 0x10 { // TSL2561 ID should be 0x1X
            return Err(SensorError::InvalidData);
        }
        
        // Power up
        self.write_register(TSL2561_REG_CONTROL, TSL2561_POWER_UP).await?;
        Timer::after(Duration::from_millis(1)).await;
        
        // Set timing (gain and integration time)
        self.set_timing().await?;
        
        Ok(())
    }
    
    /// Set timing register (gain and integration time)
    async fn set_timing(&mut self) -> Result<(), SensorError> {
        let timing = self.gain as u8 | self.integration_time as u8;
        self.write_register(TSL2561_REG_TIMING, timing).await
    }
    
    /// Set gain
    pub async fn set_gain(&mut self, gain: TSL2561Gain) -> Result<(), SensorError> {
        self.gain = gain;
        self.set_timing().await
    }
    
    /// Set integration time
    pub async fn set_integration_time(&mut self, time: TSL2561IntegrationTime) -> Result<(), SensorError> {
        self.integration_time = time;
        self.set_timing().await
    }
    
    /// Read both channels and calculate lux
    pub async fn read_light(&mut self) -> Result<TSL2561Data, SensorError> {
        // Wait for integration to complete
        let wait_time = match self.integration_time {
            TSL2561IntegrationTime::Ms13 => Duration::from_millis(15),
            TSL2561IntegrationTime::Ms101 => Duration::from_millis(110),
            TSL2561IntegrationTime::Ms402 => Duration::from_millis(450),
        };
        Timer::after(wait_time).await;
        
        // Read channel 0 (broadband)
        let ch0 = self.read_word(TSL2561_REG_DATA0LOW).await?;
        
        // Read channel 1 (IR)
        let ch1 = self.read_word(TSL2561_REG_DATA1LOW).await?;
        
        // Calculate ratio
        let ratio = if ch0 == 0 {
            0.0
        } else {
            ch1 as f32 / ch0 as f32
        };
        
        // Calculate lux using TSL2561 algorithm
        let lux = self.calculate_lux(ch0, ch1);
        
        Ok(TSL2561Data {
            lux,
            ch0,
            ch1,
            ratio,
        })
    }
    
    /// Calculate lux from channel readings using TSL2561 algorithm
    fn calculate_lux(&self, ch0: u16, ch1: u16) -> f32 {
        if ch0 == 0 {
            return 0.0;
        }
        
        // Scale values based on integration time and gain
        let ch_scale = match self.integration_time {
            TSL2561IntegrationTime::Ms13 => 0.034, // 13.7ms / 402ms
            TSL2561IntegrationTime::Ms101 => 0.252, // 101ms / 402ms
            TSL2561IntegrationTime::Ms402 => 1.0,
        };
        
        let gain_scale = match self.gain {
            TSL2561Gain::Low => 16.0,
            TSL2561Gain::High => 1.0,
        };
        
        let scale = ch_scale * gain_scale;
        let channel0 = (ch0 as f32) * scale;
        let channel1 = (ch1 as f32) * scale;
        
        // Calculate ratio
        let ratio = if channel0 == 0.0 {
            0.0
        } else {
            channel1 / channel0
        };
        
        // Calculate lux based on ratio
        let lux = if ratio <= 0.50 {
            0.0304 * channel0 - 0.062 * channel0 * ratio.powf(1.4)
        } else if ratio <= 0.61 {
            0.0224 * channel0 - 0.031 * channel1
        } else if ratio <= 0.80 {
            0.0128 * channel0 - 0.0153 * channel1
        } else if ratio <= 1.30 {
            0.00146 * channel0 - 0.00112 * channel1
        } else {
            0.0
        };
        
        lux.max(0.0)
    }
    
    /// Read single register
    async fn read_register(&mut self, reg: u8) -> Result<u8, SensorError> {
        let mut buffer = [0u8; 1];
        self.interface.write_read(self.info.address.unwrap(), &[TSL2561_CMD | reg], &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        Ok(buffer[0])
    }
    
    /// Write single register
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), SensorError> {
        self.interface.write(self.info.address.unwrap(), &[TSL2561_CMD | reg, value]).await
            .map_err(|_| SensorError::CommunicationError)
    }
    
    /// Read word (16-bit) from register
    async fn read_word(&mut self, reg: u8) -> Result<u16, SensorError> {
        let mut buffer = [0u8; 2];
        self.interface.write_read(self.info.address.unwrap(), &[TSL2561_CMD_WORD | reg], &mut buffer).await
            .map_err(|_| SensorError::CommunicationError)?;
        Ok(u16::from_le_bytes(buffer))
    }
    
    /// Power down sensor
    pub async fn power_down(&mut self) -> Result<(), SensorError> {
        self.write_register(TSL2561_REG_CONTROL, TSL2561_POWER_DOWN).await
    }
    
    /// Power up sensor
    pub async fn power_up(&mut self) -> Result<(), SensorError> {
        self.write_register(TSL2561_REG_CONTROL, TSL2561_POWER_UP).await?;
        Timer::after(Duration::from_millis(1)).await;
        Ok(())
    }
}

#[async_trait::async_trait]
impl<I> SensorTrait for TSL2561<I>
where
    I: I2c,
{
    type Error = SensorError;
    type Data = TSL2561Data;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        self.initialize().await
    }
    
    async fn read(&mut self) -> Result<Self::Data, Self::Error> {
        self.read_light().await
    }
    
    fn get_info(&self) -> SensorInfo {
        self.info.clone()
    }
    
    async fn sleep(&mut self) -> Result<(), Self::Error> {
        self.power_down().await
    }
}

#[async_trait::async_trait]
impl<I> LightSensor for TSL2561<I>
where
    I: I2c,
{
    async fn read_lux(&mut self) -> Result<f32, Self::Error> {
        let data = self.read_light().await?;
        Ok(data.lux)
    }
    
    async fn read_raw(&mut self) -> Result<u16, Self::Error> {
        let data = self.read_light().await?;
        Ok(data.ch0) // Return broadband channel
    }
    
    fn get_lux_range(&self) -> (f32, f32) {
        (0.1, 40000.0)
    }
    
    fn get_lux_resolution(&self) -> f32 {
        0.1
    }
    
    fn get_accuracy(&self) -> f32 {
        3.0 // ±3%
    }
    
    async fn set_integration_time(&mut self, time_ms: u16) -> Result<(), Self::Error> {
        let integration_time = match time_ms {
            0..=50 => TSL2561IntegrationTime::Ms13,
            51..=200 => TSL2561IntegrationTime::Ms101,
            _ => TSL2561IntegrationTime::Ms402,
        };
        self.set_integration_time(integration_time).await
    }
    
    async fn set_gain(&mut self, gain: u8) -> Result<(), Self::Error> {
        let tsl_gain = match gain {
            0..=1 => TSL2561Gain::Low,
            _ => TSL2561Gain::High,
        };
        self.set_gain(tsl_gain).await
    }
}

/// Light sensor manager
pub struct LightSensorManager {
    sensors: heapless::Vec<SensorInfo, 8>,
    calibrations: heapless::FnvIndexMap<u8, LightCalibration, 8>,
    auto_gain_enabled: bool,
}

/// Light calibration data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightCalibration {
    pub offset: f32,
    pub scale: f32,
    pub dark_current: f32, // Dark current compensation
}

impl LightCalibration {
    pub fn new(offset: f32, scale: f32) -> Self {
        Self {
            offset,
            scale,
            dark_current: 0.0,
        }
    }
    
    /// Apply calibration
    pub fn apply(&self, raw_lux: f32) -> f32 {
        ((raw_lux - self.dark_current) + self.offset) * self.scale
    }
}

impl LightSensorManager {
    pub fn new() -> Self {
        Self {
            sensors: heapless::Vec::new(),
            calibrations: heapless::FnvIndexMap::new(),
            auto_gain_enabled: false,
        }
    }
    
    /// Register a light sensor
    pub fn register_sensor(&mut self, sensor_info: SensorInfo) -> Result<(), SensorError> {
        self.sensors.push(sensor_info)
            .map_err(|_| SensorError::NotSupported)
    }
    
    /// Set calibration for a sensor
    pub fn set_calibration(&mut self, sensor_id: u8, calibration: LightCalibration) -> Result<(), SensorError> {
        self.calibrations.insert(sensor_id, calibration)
            .map_err(|_| SensorError::NotSupported)?;
        Ok(())
    }
    
    /// Apply calibration to a reading
    pub fn apply_calibration(&self, sensor_id: u8, raw_lux: f32) -> f32 {
        if let Some(cal) = self.calibrations.get(&sensor_id) {
            cal.apply(raw_lux)
        } else {
            raw_lux
        }
    }
    
    /// Enable/disable automatic gain control
    pub fn set_auto_gain(&mut self, enabled: bool) {
        self.auto_gain_enabled = enabled;
    }
    
    /// Check if auto gain is enabled
    pub fn is_auto_gain_enabled(&self) -> bool {
        self.auto_gain_enabled
    }
}

/// Light measurement utilities
pub mod light_utils {
    use super::*;
    
    /// Light level categories
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum LightLevel {
        Dark,      // < 1 lux
        VeryDim,   // 1-10 lux
        Dim,       // 10-100 lux
        Normal,    // 100-1000 lux
        Bright,    // 1000-10000 lux
        VeryBright, // > 10000 lux
    }
    
    /// Categorize light level
    pub fn categorize_light_level(lux: f32) -> LightLevel {
        match lux {
            l if l < 1.0 => LightLevel::Dark,
            l if l < 10.0 => LightLevel::VeryDim,
            l if l < 100.0 => LightLevel::Dim,
            l if l < 1000.0 => LightLevel::Normal,
            l if l < 10000.0 => LightLevel::Bright,
            _ => LightLevel::VeryBright,
        }
    }
    
    /// Convert lux to foot-candles
    pub fn lux_to_foot_candles(lux: f32) -> f32 {
        lux / 10.764
    }
    
    /// Convert foot-candles to lux
    pub fn foot_candles_to_lux(fc: f32) -> f32 {
        fc * 10.764
    }
    
    /// Validate light reading
    pub fn validate_lux(lux: f32) -> bool {
        lux >= 0.0 && lux <= 100000.0
    }
    
    /// Calculate optimal integration time based on light level
    pub fn calculate_optimal_integration_time(lux: f32) -> u16 {
        match lux {
            l if l < 1.0 => 400,    // Long integration for low light
            l if l < 100.0 => 100,  // Medium integration
            _ => 13,                // Short integration for bright light
        }
    }
    
    /// Calculate optimal gain based on light level
    pub fn calculate_optimal_gain(lux: f32) -> u8 {
        match lux {
            l if l < 10.0 => 16,  // High gain for low light
            _ => 1,               // Low gain for normal/bright light
        }
    }
    
    /// Apply gamma correction for display
    pub fn apply_gamma_correction(lux: f32, gamma: f32) -> f32 {
        lux.powf(1.0 / gamma)
    }
    
    /// Calculate circadian lighting metrics
    pub fn calculate_circadian_stimulus(lux: f32, color_temperature: f32) -> f32 {
        // Simplified circadian stimulus calculation
        // Real implementation would require spectral data
        let melanopic_factor = if color_temperature > 5000.0 {
            1.2 // Blue-rich light has higher circadian impact
        } else {
            0.8 // Warm light has lower circadian impact
        };
        
        (lux * melanopic_factor / 1000.0).min(1.0)
    }
}

/// Light measurement statistics
#[derive(Debug, Clone)]
pub struct LightStatistics {
    pub min: f32,
    pub max: f32,
    pub average: f32,
    pub median: f32,
    pub std_deviation: f32,
    pub sample_count: usize,
}

impl LightStatistics {
    /// Calculate statistics from a series of light measurements
    pub fn from_measurements(measurements: &[f32]) -> Self {
        if measurements.is_empty() {
            return Self {
                min: 0.0,
                max: 0.0,
                average: 0.0,
                median: 0.0,
                std_deviation: 0.0,
                sample_count: 0,
            };
        }
        
        let mut sorted = measurements.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        
        let min = sorted[0];
        let max = sorted[sorted.len() - 1];
        let sum: f32 = measurements.iter().sum();
        let average = sum / measurements.len() as f32;
        
        let median = if sorted.len() % 2 == 0 {
            (sorted[sorted.len() / 2 - 1] + sorted[sorted.len() / 2]) / 2.0
        } else {
            sorted[sorted.len() / 2]
        };
        
        let variance: f32 = measurements.iter()
            .map(|x| (x - average).powi(2))
            .sum::<f32>() / measurements.len() as f32;
        let std_deviation = variance.sqrt();
        
        Self {
            min,
            max,
            average,
            median,
            std_deviation,
            sample_count: measurements.len(),
        }
    }
}