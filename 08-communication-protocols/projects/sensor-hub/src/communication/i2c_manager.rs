//! I2C Communication Manager
//!
//! This module provides a comprehensive I2C communication manager with
//! features like device discovery, error handling, statistics, and retry logic.

use super::*;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c;
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

/// I2C Manager for handling multiple I2C devices
pub struct I2cManager<I2C> {
    i2c: I2C,
    devices: FnvIndexMap<u8, I2cDeviceInfo, 16>,
    stats: CommunicationStats,
    config: I2cConfig,
    scanner: BusScanner,
    diagnostics: CommunicationDiagnostics,
}

/// I2C device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct I2cDeviceInfo {
    pub address: u8,
    pub name: Option<heapless::String<32>>,
    pub device_type: I2cDeviceType,
    pub config: DeviceConfig,
    pub last_seen: Option<u64>,
    pub stats: CommunicationStats,
    pub status: DeviceStatus,
}

/// I2C device types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum I2cDeviceType {
    Sensor,
    Display,
    Memory,
    RTC,
    ADC,
    DAC,
    GPIO,
    Unknown,
}

/// I2C configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct I2cConfig {
    pub clock_speed_hz: u32,
    pub timeout_ms: u32,
    pub retry_count: u8,
    pub scan_on_init: bool,
    pub auto_recovery: bool,
    pub pullup_enabled: bool,
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self {
            clock_speed_hz: 100_000, // 100kHz standard mode
            timeout_ms: 1000,
            retry_count: 3,
            scan_on_init: true,
            auto_recovery: true,
            pullup_enabled: true,
        }
    }
}

/// I2C specific errors
#[derive(Debug, Clone, PartialEq)]
pub enum I2cError {
    /// Communication error
    Communication(CommunicationError),
    /// No ACK received
    NoAck,
    /// Arbitration lost
    ArbitrationLost,
    /// Bus busy
    BusBusy,
    /// Clock stretching timeout
    ClockStretchTimeout,
    /// Invalid address format
    InvalidAddress,
    /// Device buffer full
    DeviceBufferFull,
}

impl From<CommunicationError> for I2cError {
    fn from(error: CommunicationError) -> Self {
        I2cError::Communication(error)
    }
}

impl<I2C> I2cManager<I2C>
where
    I2C: I2c,
{
    /// Create new I2C manager
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            devices: FnvIndexMap::new(),
            stats: CommunicationStats::new(),
            config: I2cConfig::default(),
            scanner: BusScanner::new(),
            diagnostics: CommunicationDiagnostics::new(),
        }
    }
    
    /// Create I2C manager with custom configuration
    pub fn with_config(i2c: I2C, config: I2cConfig) -> Self {
        let mut manager = Self::new(i2c);
        manager.config = config;
        manager
    }
    
    /// Initialize the I2C manager
    pub async fn init(&mut self) -> Result<(), I2cError> {
        // Perform initial bus scan if enabled
        if self.config.scan_on_init {
            self.scan_bus().await?;
        }
        
        // Initialize diagnostics
        self.diagnostics = CommunicationDiagnostics::new();
        
        Ok(())
    }
    
    /// Scan I2C bus for devices
    pub async fn scan_bus(&mut self) -> Result<Vec<u8, 16>, I2cError> {
        let found_devices = self.scanner.scan_i2c(&mut self.i2c).await;
        
        // Update device registry
        for &address in &found_devices {
            if !self.devices.contains_key(&address) {
                let device_info = I2cDeviceInfo {
                    address,
                    name: None,
                    device_type: I2cDeviceType::Unknown,
                    config: DeviceConfig::i2c(address, self.config.clock_speed_hz),
                    last_seen: Some(self.get_timestamp()),
                    stats: CommunicationStats::new(),
                    status: DeviceStatus::Online,
                };
                
                let _ = self.devices.insert(address, device_info);
                self.diagnostics.update_device_status(address, DeviceStatus::Online);
            }
        }
        
        Ok(found_devices)
    }
    
    /// Register a device manually
    pub fn register_device(
        &mut self,
        address: u8,
        name: Option<&str>,
        device_type: I2cDeviceType,
    ) -> Result<(), I2cError> {
        if !comm_utils::is_valid_i2c_address(address) {
            return Err(I2cError::InvalidAddress);
        }
        
        let device_info = I2cDeviceInfo {
            address,
            name: name.map(|s| heapless::String::from(s)).unwrap_or(None),
            device_type,
            config: DeviceConfig::i2c(address, self.config.clock_speed_hz),
            last_seen: None,
            stats: CommunicationStats::new(),
            status: DeviceStatus::Offline,
        };
        
        self.devices.insert(address, device_info)
            .map_err(|_| I2cError::Communication(CommunicationError::BufferOverflow))?;
        
        Ok(())
    }
    
    /// Unregister a device
    pub fn unregister_device(&mut self, address: u8) -> Result<(), I2cError> {
        self.devices.remove(&address)
            .ok_or(I2cError::Communication(CommunicationError::DeviceNotFound))?;
        Ok(())
    }
    
    /// Write data to I2C device with retry logic
    pub async fn write_with_retry(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), I2cError> {
        let start_time = Instant::now();
        let mut last_error = None;
        
        for attempt in 0..=self.config.retry_count {
            match self.write_raw(address, data).await {
                Ok(()) => {
                    let duration = start_time.elapsed();
                    self.record_success(address, duration.as_micros() as u32);
                    return Ok(());
                }
                Err(e) => {
                    last_error = Some(e.clone());
                    self.record_failure(address, e.clone().into());
                    
                    if attempt < self.config.retry_count {
                        self.stats.record_retry();
                        // Exponential backoff
                        let delay_ms = 10 * (1 << attempt);
                        Timer::after(Duration::from_millis(delay_ms as u64)).await;
                    }
                }
            }
        }
        
        Err(last_error.unwrap())
    }
    
    /// Read data from I2C device with retry logic
    pub async fn read_with_retry(
        &mut self,
        address: u8,
        buffer: &mut [u8],
    ) -> Result<(), I2cError> {
        let start_time = Instant::now();
        let mut last_error = None;
        
        for attempt in 0..=self.config.retry_count {
            match self.read_raw(address, buffer).await {
                Ok(()) => {
                    let duration = start_time.elapsed();
                    self.record_success(address, duration.as_micros() as u32);
                    return Ok(());
                }
                Err(e) => {
                    last_error = Some(e.clone());
                    self.record_failure(address, e.clone().into());
                    
                    if attempt < self.config.retry_count {
                        self.stats.record_retry();
                        // Exponential backoff
                        let delay_ms = 10 * (1 << attempt);
                        Timer::after(Duration::from_millis(delay_ms as u64)).await;
                    }
                }
            }
        }
        
        Err(last_error.unwrap())
    }
    
    /// Write then read from I2C device
    pub async fn write_read_with_retry(
        &mut self,
        address: u8,
        write_data: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), I2cError> {
        let start_time = Instant::now();
        let mut last_error = None;
        
        for attempt in 0..=self.config.retry_count {
            match self.write_read_raw(address, write_data, read_buffer).await {
                Ok(()) => {
                    let duration = start_time.elapsed();
                    self.record_success(address, duration.as_micros() as u32);
                    return Ok(());
                }
                Err(e) => {
                    last_error = Some(e.clone());
                    self.record_failure(address, e.clone().into());
                    
                    if attempt < self.config.retry_count {
                        self.stats.record_retry();
                        // Exponential backoff
                        let delay_ms = 10 * (1 << attempt);
                        Timer::after(Duration::from_millis(delay_ms as u64)).await;
                    }
                }
            }
        }
        
        Err(last_error.unwrap())
    }
    
    /// Raw write operation (no retry)
    async fn write_raw(&mut self, address: u8, data: &[u8]) -> Result<(), I2cError> {
        self.i2c.write(address, data).await
            .map_err(|_| I2cError::Communication(CommunicationError::BusError))
    }
    
    /// Raw read operation (no retry)
    async fn read_raw(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), I2cError> {
        self.i2c.read(address, buffer).await
            .map_err(|_| I2cError::Communication(CommunicationError::BusError))
    }
    
    /// Raw write-read operation (no retry)
    async fn write_read_raw(
        &mut self,
        address: u8,
        write_data: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), I2cError> {
        self.i2c.write_read(address, write_data, read_buffer).await
            .map_err(|_| I2cError::Communication(CommunicationError::BusError))
    }
    
    /// Probe device (check if it responds)
    pub async fn probe_device(&mut self, address: u8) -> Result<bool, I2cError> {
        match self.i2c.write(address, &[]).await {
            Ok(()) => {
                self.update_device_last_seen(address);
                self.diagnostics.update_device_status(address, DeviceStatus::Online);
                Ok(true)
            }
            Err(_) => {
                self.diagnostics.update_device_status(address, DeviceStatus::Offline);
                Ok(false)
            }
        }
    }
    
    /// Read register from device
    pub async fn read_register(
        &mut self,
        address: u8,
        register: u8,
        buffer: &mut [u8],
    ) -> Result<(), I2cError> {
        self.write_read_with_retry(address, &[register], buffer).await
    }
    
    /// Write register to device
    pub async fn write_register(
        &mut self,
        address: u8,
        register: u8,
        data: &[u8],
    ) -> Result<(), I2cError> {
        let mut write_data = Vec::<u8, 32>::new();
        write_data.push(register).map_err(|_| I2cError::Communication(CommunicationError::BufferOverflow))?;
        
        for &byte in data {
            write_data.push(byte).map_err(|_| I2cError::Communication(CommunicationError::BufferOverflow))?;
        }
        
        self.write_with_retry(address, &write_data).await
    }
    
    /// Read single register byte
    pub async fn read_register_byte(&mut self, address: u8, register: u8) -> Result<u8, I2cError> {
        let mut buffer = [0u8; 1];
        self.read_register(address, register, &mut buffer).await?;
        Ok(buffer[0])
    }
    
    /// Write single register byte
    pub async fn write_register_byte(
        &mut self,
        address: u8,
        register: u8,
        value: u8,
    ) -> Result<(), I2cError> {
        self.write_register(address, register, &[value]).await
    }
    
    /// Read 16-bit register (big-endian)
    pub async fn read_register_u16_be(
        &mut self,
        address: u8,
        register: u8,
    ) -> Result<u16, I2cError> {
        let mut buffer = [0u8; 2];
        self.read_register(address, register, &mut buffer).await?;
        Ok(u16::from_be_bytes(buffer))
    }
    
    /// Read 16-bit register (little-endian)
    pub async fn read_register_u16_le(
        &mut self,
        address: u8,
        register: u8,
    ) -> Result<u16, I2cError> {
        let mut buffer = [0u8; 2];
        self.read_register(address, register, &mut buffer).await?;
        Ok(u16::from_le_bytes(buffer))
    }
    
    /// Write 16-bit register (big-endian)
    pub async fn write_register_u16_be(
        &mut self,
        address: u8,
        register: u8,
        value: u16,
    ) -> Result<(), I2cError> {
        let bytes = value.to_be_bytes();
        self.write_register(address, register, &bytes).await
    }
    
    /// Write 16-bit register (little-endian)
    pub async fn write_register_u16_le(
        &mut self,
        address: u8,
        register: u8,
        value: u16,
    ) -> Result<(), I2cError> {
        let bytes = value.to_le_bytes();
        self.write_register(address, register, &bytes).await
    }
    
    /// Get device information
    pub fn get_device_info(&self, address: u8) -> Option<&I2cDeviceInfo> {
        self.devices.get(&address)
    }
    
    /// Get all registered devices
    pub fn get_all_devices(&self) -> impl Iterator<Item = (&u8, &I2cDeviceInfo)> {
        self.devices.iter()
    }
    
    /// Get devices by type
    pub fn get_devices_by_type(&self, device_type: I2cDeviceType) -> Vec<u8, 16> {
        let mut devices = Vec::new();
        for (&address, info) in &self.devices {
            if info.device_type == device_type {
                let _ = devices.push(address);
            }
        }
        devices
    }
    
    /// Update device configuration
    pub fn update_device_config(
        &mut self,
        address: u8,
        config: DeviceConfig,
    ) -> Result<(), I2cError> {
        if let Some(device) = self.devices.get_mut(&address) {
            device.config = config;
            Ok(())
        } else {
            Err(I2cError::Communication(CommunicationError::DeviceNotFound))
        }
    }
    
    /// Set device name
    pub fn set_device_name(&mut self, address: u8, name: &str) -> Result<(), I2cError> {
        if let Some(device) = self.devices.get_mut(&address) {
            device.name = Some(heapless::String::from(name));
            Ok(())
        } else {
            Err(I2cError::Communication(CommunicationError::DeviceNotFound))
        }
    }
    
    /// Set device type
    pub fn set_device_type(
        &mut self,
        address: u8,
        device_type: I2cDeviceType,
    ) -> Result<(), I2cError> {
        if let Some(device) = self.devices.get_mut(&address) {
            device.device_type = device_type;
            Ok(())
        } else {
            Err(I2cError::Communication(CommunicationError::DeviceNotFound))
        }
    }
    
    /// Get communication statistics
    pub fn get_stats(&self) -> &CommunicationStats {
        &self.stats
    }
    
    /// Get device-specific statistics
    pub fn get_device_stats(&self, address: u8) -> Option<&CommunicationStats> {
        self.devices.get(&address).map(|device| &device.stats)
    }
    
    /// Reset statistics
    pub fn reset_stats(&mut self) {
        self.stats.reset();
        for device in self.devices.values_mut() {
            device.stats.reset();
        }
    }
    
    /// Get diagnostics
    pub fn get_diagnostics(&self) -> &CommunicationDiagnostics {
        &self.diagnostics
    }
    
    /// Update diagnostics
    pub fn update_diagnostics(&mut self) {
        self.diagnostics.calculate_bus_health();
    }
    
    /// Perform bus recovery
    pub async fn recover_bus(&mut self) -> Result<(), I2cError> {
        // Implementation would depend on the specific I2C peripheral
        // This is a placeholder for bus recovery logic
        
        // Reset statistics
        self.reset_stats();
        
        // Re-scan bus
        self.scan_bus().await?;
        
        Ok(())
    }
    
    /// Health check for all devices
    pub async fn health_check(&mut self) -> Result<Vec<(u8, DeviceStatus), 16>, I2cError> {
        let mut results = Vec::new();
        
        for &address in self.devices.keys() {
            let is_online = self.probe_device(address).await?;
            let status = if is_online {
                DeviceStatus::Online
            } else {
                DeviceStatus::Offline
            };
            
            let _ = results.push((address, status));
        }
        
        Ok(results)
    }
    
    /// Record successful operation
    fn record_success(&mut self, address: u8, response_time_us: u32) {
        self.stats.record_success(response_time_us);
        
        if let Some(device) = self.devices.get_mut(&address) {
            device.stats.record_success(response_time_us);
            device.status = DeviceStatus::Online;
            device.last_seen = Some(self.get_timestamp());
        }
    }
    
    /// Record failed operation
    fn record_failure(&mut self, address: u8, error: CommunicationError) {
        self.stats.record_failure(error.clone());
        
        if let Some(device) = self.devices.get_mut(&address) {
            device.stats.record_failure(error.clone());
            device.status = DeviceStatus::Error;
        }
        
        self.diagnostics.record_error(error, self.get_timestamp());
    }
    
    /// Update device last seen timestamp
    fn update_device_last_seen(&mut self, address: u8) {
        if let Some(device) = self.devices.get_mut(&address) {
            device.last_seen = Some(self.get_timestamp());
        }
    }
    
    /// Get current timestamp (placeholder - would use actual RTC)
    fn get_timestamp(&self) -> u64 {
        // This would typically use a real-time clock
        // For now, return a placeholder value
        0
    }
}

#[async_trait::async_trait]
impl<I2C> CommunicationManager for I2cManager<I2C>
where
    I2C: I2c + Send,
{
    type Error = I2cError;
    
    async fn init(&mut self) -> Result<(), Self::Error> {
        self.init().await
    }
    
    async fn write(&mut self, address: u8, data: &[u8]) -> Result<(), Self::Error> {
        self.write_with_retry(address, data).await
    }
    
    async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.read_with_retry(address, buffer).await
    }
    
    async fn write_read(
        &mut self,
        address: u8,
        write_data: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.write_read_with_retry(address, write_data, read_buffer).await
    }
    
    async fn probe_device(&mut self, address: u8) -> Result<bool, Self::Error> {
        self.probe_device(address).await
    }
    
    fn get_stats(&self) -> &CommunicationStats {
        &self.stats
    }
    
    fn reset_stats(&mut self) {
        self.reset_stats();
    }
    
    fn set_timeout(&mut self, timeout_ms: u32) {
        self.config.timeout_ms = timeout_ms;
    }
    
    fn set_retry_count(&mut self, count: u8) {
        self.config.retry_count = count;
    }
}

/// I2C utilities
pub mod i2c_utils {
    use super::*;
    
    /// Common I2C device addresses
    pub mod addresses {
        // Temperature sensors
        pub const DS1621: u8 = 0x48;
        pub const LM75: u8 = 0x48;
        pub const TMP102: u8 = 0x48;
        
        // Humidity sensors
        pub const SHT30: u8 = 0x44;
        pub const SHT31: u8 = 0x44;
        pub const HTU21D: u8 = 0x40;
        
        // Pressure sensors
        pub const BMP280: u8 = 0x76;
        pub const BMP180: u8 = 0x77;
        pub const MS5611: u8 = 0x77;
        
        // Light sensors
        pub const BH1750: u8 = 0x23;
        pub const TSL2561: u8 = 0x39;
        pub const VEML7700: u8 = 0x10;
        
        // Displays
        pub const SSD1306: u8 = 0x3C;
        pub const SH1106: u8 = 0x3C;
        
        // RTC
        pub const DS1307: u8 = 0x68;
        pub const DS3231: u8 = 0x68;
        pub const PCF8523: u8 = 0x68;
        
        // EEPROM
        pub const AT24C32: u8 = 0x50;
        pub const AT24C64: u8 = 0x50;
        
        // GPIO Expanders
        pub const PCF8574: u8 = 0x20;
        pub const MCP23017: u8 = 0x20;
    }
    
    /// Identify device type by address (heuristic)
    pub fn identify_device_type(address: u8) -> I2cDeviceType {
        match address {
            // Temperature sensors
            0x48 | 0x49 | 0x4A | 0x4B => I2cDeviceType::Sensor,
            
            // Humidity sensors
            0x40 | 0x44 => I2cDeviceType::Sensor,
            
            // Pressure sensors
            0x76 | 0x77 => I2cDeviceType::Sensor,
            
            // Light sensors
            0x10 | 0x23 | 0x29 | 0x39 => I2cDeviceType::Sensor,
            
            // Displays
            0x3C | 0x3D => I2cDeviceType::Display,
            
            // RTC
            0x68 => I2cDeviceType::RTC,
            
            // EEPROM
            0x50..=0x57 => I2cDeviceType::Memory,
            
            // GPIO Expanders
            0x20..=0x27 => I2cDeviceType::GPIO,
            
            // ADC/DAC
            0x48..=0x4B => I2cDeviceType::ADC,
            
            _ => I2cDeviceType::Unknown,
        }
    }
    
    /// Get device name by address (heuristic)
    pub fn get_device_name(address: u8) -> Option<&'static str> {
        match address {
            addresses::DS1621 => Some("DS1621"),
            addresses::SHT30 => Some("SHT30"),
            addresses::BMP280 => Some("BMP280"),
            addresses::BH1750 => Some("BH1750"),
            addresses::SSD1306 => Some("SSD1306"),
            addresses::DS1307 => Some("DS1307"),
            addresses::AT24C32 => Some("AT24C32"),
            addresses::PCF8574 => Some("PCF8574"),
            _ => None,
        }
    }
    
    /// Calculate I2C timing parameters
    pub fn calculate_timing(clock_speed_hz: u32, system_clock_hz: u32) -> I2cTiming {
        let period_ns = 1_000_000_000 / clock_speed_hz;
        let system_period_ns = 1_000_000_000 / system_clock_hz;
        
        I2cTiming {
            scl_high_period: period_ns / 2,
            scl_low_period: period_ns / 2,
            sda_hold_time: system_period_ns * 2,
            sda_setup_time: system_period_ns * 2,
        }
    }
}

/// I2C timing parameters
#[derive(Debug, Clone, Copy)]
pub struct I2cTiming {
    pub scl_high_period: u32,
    pub scl_low_period: u32,
    pub sda_hold_time: u32,
    pub sda_setup_time: u32,
}