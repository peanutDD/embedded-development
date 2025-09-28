//! SPI Communication Manager
//!
//! This module provides a comprehensive SPI communication manager with
//! features like device management, error handling, statistics, and configuration.

use super::*;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::SpiDevice;
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

/// SPI Manager for handling multiple SPI devices
pub struct SpiManager<SPI> {
    spi: SPI,
    devices: FnvIndexMap<u8, SpiDeviceInfo, 8>, // Fewer SPI devices typically
    stats: CommunicationStats,
    config: SpiConfig,
    diagnostics: CommunicationDiagnostics,
}

/// SPI device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpiDeviceInfo {
    pub chip_select: u8,
    pub name: Option<heapless::String<32>>,
    pub device_type: SpiDeviceType,
    pub config: SpiDeviceConfig,
    pub last_used: Option<u64>,
    pub stats: CommunicationStats,
    pub status: DeviceStatus,
}

/// SPI device types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SpiDeviceType {
    Sensor,
    Display,
    Memory,
    ADC,
    DAC,
    Radio,
    SD_Card,
    Flash,
    Unknown,
}

/// SPI device-specific configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpiDeviceConfig {
    pub clock_speed_hz: u32,
    pub mode: SpiMode,
    pub bit_order: BitOrder,
    pub word_size: u8,
    pub cs_active_low: bool,
    pub cs_setup_time_ns: u16,
    pub cs_hold_time_ns: u16,
    pub inter_word_delay_ns: u16,
}

/// SPI configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpiConfig {
    pub default_clock_speed_hz: u32,
    pub default_mode: SpiMode,
    pub timeout_ms: u32,
    pub retry_count: u8,
    pub auto_cs_control: bool,
    pub dma_enabled: bool,
}

/// SPI modes (CPOL, CPHA)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SpiMode {
    Mode0, // CPOL=0, CPHA=0
    Mode1, // CPOL=0, CPHA=1
    Mode2, // CPOL=1, CPHA=0
    Mode3, // CPOL=1, CPHA=1
}

/// Bit order for SPI transmission
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BitOrder {
    MsbFirst,
    LsbFirst,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            default_clock_speed_hz: 1_000_000, // 1MHz
            default_mode: SpiMode::Mode0,
            timeout_ms: 1000,
            retry_count: 3,
            auto_cs_control: true,
            dma_enabled: false,
        }
    }
}

impl Default for SpiDeviceConfig {
    fn default() -> Self {
        Self {
            clock_speed_hz: 1_000_000,
            mode: SpiMode::Mode0,
            bit_order: BitOrder::MsbFirst,
            word_size: 8,
            cs_active_low: true,
            cs_setup_time_ns: 100,
            cs_hold_time_ns: 100,
            inter_word_delay_ns: 0,
        }
    }
}

/// SPI specific errors
#[derive(Debug, Clone, PartialEq)]
pub enum SpiError {
    /// Communication error
    Communication(CommunicationError),
    /// Invalid chip select
    InvalidChipSelect,
    /// Mode configuration error
    ModeConfigError,
    /// Clock configuration error
    ClockConfigError,
    /// DMA error
    DmaError,
    /// FIFO overflow/underflow
    FifoError,
    /// Frame format error
    FrameFormatError,
}

impl From<CommunicationError> for SpiError {
    fn from(error: CommunicationError) -> Self {
        SpiError::Communication(error)
    }
}

impl<SPI> SpiManager<SPI>
where
    SPI: SpiDevice,
{
    /// Create new SPI manager
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            devices: FnvIndexMap::new(),
            stats: CommunicationStats::new(),
            config: SpiConfig::default(),
            diagnostics: CommunicationDiagnostics::new(),
        }
    }
    
    /// Create SPI manager with custom configuration
    pub fn with_config(spi: SPI, config: SpiConfig) -> Self {
        let mut manager = Self::new(spi);
        manager.config = config;
        manager
    }
    
    /// Initialize the SPI manager
    pub async fn init(&mut self) -> Result<(), SpiError> {
        // Initialize diagnostics
        self.diagnostics = CommunicationDiagnostics::new();
        Ok(())
    }
    
    /// Register a SPI device
    pub fn register_device(
        &mut self,
        chip_select: u8,
        name: Option<&str>,
        device_type: SpiDeviceType,
        config: Option<SpiDeviceConfig>,
    ) -> Result<(), SpiError> {
        let device_config = config.unwrap_or_else(|| {
            let mut cfg = SpiDeviceConfig::default();
            cfg.clock_speed_hz = self.config.default_clock_speed_hz;
            cfg.mode = self.config.default_mode;
            cfg
        });
        
        let device_info = SpiDeviceInfo {
            chip_select,
            name: name.map(|s| heapless::String::from(s)).unwrap_or(None),
            device_type,
            config: device_config,
            last_used: None,
            stats: CommunicationStats::new(),
            status: DeviceStatus::Offline,
        };
        
        self.devices.insert(chip_select, device_info)
            .map_err(|_| SpiError::Communication(CommunicationError::BufferOverflow))?;
        
        Ok(())
    }
    
    /// Unregister a device
    pub fn unregister_device(&mut self, chip_select: u8) -> Result<(), SpiError> {
        self.devices.remove(&chip_select)
            .ok_or(SpiError::Communication(CommunicationError::DeviceNotFound))?;
        Ok(())
    }
    
    /// Transfer data with retry logic
    pub async fn transfer_with_retry(
        &mut self,
        chip_select: u8,
        write_data: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), SpiError> {
        let start_time = Instant::now();
        let mut last_error = None;
        
        // Configure device before transfer
        self.configure_device(chip_select)?;
        
        for attempt in 0..=self.config.retry_count {
            match self.transfer_raw(write_data, read_buffer).await {
                Ok(()) => {
                    let duration = start_time.elapsed();
                    self.record_success(chip_select, duration.as_micros() as u32);
                    return Ok(());
                }
                Err(e) => {
                    last_error = Some(e.clone());
                    self.record_failure(chip_select, e.clone().into());
                    
                    if attempt < self.config.retry_count {
                        self.stats.record_retry();
                        // Short delay between retries
                        Timer::after(Duration::from_millis(10)).await;
                    }
                }
            }
        }
        
        Err(last_error.unwrap())
    }
    
    /// Write data with retry logic
    pub async fn write_with_retry(
        &mut self,
        chip_select: u8,
        data: &[u8],
    ) -> Result<(), SpiError> {
        let start_time = Instant::now();
        let mut last_error = None;
        
        // Configure device before write
        self.configure_device(chip_select)?;
        
        for attempt in 0..=self.config.retry_count {
            match self.write_raw(data).await {
                Ok(()) => {
                    let duration = start_time.elapsed();
                    self.record_success(chip_select, duration.as_micros() as u32);
                    return Ok(());
                }
                Err(e) => {
                    last_error = Some(e.clone());
                    self.record_failure(chip_select, e.clone().into());
                    
                    if attempt < self.config.retry_count {
                        self.stats.record_retry();
                        Timer::after(Duration::from_millis(10)).await;
                    }
                }
            }
        }
        
        Err(last_error.unwrap())
    }
    
    /// Read data with retry logic
    pub async fn read_with_retry(
        &mut self,
        chip_select: u8,
        buffer: &mut [u8],
    ) -> Result<(), SpiError> {
        let start_time = Instant::now();
        let mut last_error = None;
        
        // Configure device before read
        self.configure_device(chip_select)?;
        
        for attempt in 0..=self.config.retry_count {
            match self.read_raw(buffer).await {
                Ok(()) => {
                    let duration = start_time.elapsed();
                    self.record_success(chip_select, duration.as_micros() as u32);
                    return Ok(());
                }
                Err(e) => {
                    last_error = Some(e.clone());
                    self.record_failure(chip_select, e.clone().into());
                    
                    if attempt < self.config.retry_count {
                        self.stats.record_retry();
                        Timer::after(Duration::from_millis(10)).await;
                    }
                }
            }
        }
        
        Err(last_error.unwrap())
    }
    
    /// Raw transfer operation (no retry)
    async fn transfer_raw(&mut self, write_data: &[u8], read_buffer: &mut [u8]) -> Result<(), SpiError> {
        self.spi.transfer(read_buffer, write_data).await
            .map_err(|_| SpiError::Communication(CommunicationError::BusError))
    }
    
    /// Raw write operation (no retry)
    async fn write_raw(&mut self, data: &[u8]) -> Result<(), SpiError> {
        self.spi.write(data).await
            .map_err(|_| SpiError::Communication(CommunicationError::BusError))
    }
    
    /// Raw read operation (no retry)
    async fn read_raw(&mut self, buffer: &mut [u8]) -> Result<(), SpiError> {
        self.spi.read(buffer).await
            .map_err(|_| SpiError::Communication(CommunicationError::BusError))
    }
    
    /// Configure device settings before communication
    fn configure_device(&mut self, chip_select: u8) -> Result<(), SpiError> {
        if let Some(device) = self.devices.get(&chip_select) {
            // Here we would configure the SPI peripheral with device-specific settings
            // This is hardware-dependent and would need actual implementation
            
            // Update last used timestamp
            self.update_device_last_used(chip_select);
            Ok(())
        } else {
            Err(SpiError::Communication(CommunicationError::DeviceNotFound))
        }
    }
    
    /// Write register to device
    pub async fn write_register(
        &mut self,
        chip_select: u8,
        register: u8,
        data: &[u8],
    ) -> Result<(), SpiError> {
        let mut write_data = Vec::<u8, 32>::new();
        write_data.push(register).map_err(|_| SpiError::Communication(CommunicationError::BufferOverflow))?;
        
        for &byte in data {
            write_data.push(byte).map_err(|_| SpiError::Communication(CommunicationError::BufferOverflow))?;
        }
        
        self.write_with_retry(chip_select, &write_data).await
    }
    
    /// Read register from device
    pub async fn read_register(
        &mut self,
        chip_select: u8,
        register: u8,
        buffer: &mut [u8],
    ) -> Result<(), SpiError> {
        let write_data = [register | 0x80]; // Set read bit (common convention)
        let mut full_buffer = Vec::<u8, 33>::new();
        
        // Prepare buffer for transfer (command + data)
        full_buffer.push(0).map_err(|_| SpiError::Communication(CommunicationError::BufferOverflow))?; // Dummy byte for command
        for _ in 0..buffer.len() {
            full_buffer.push(0).map_err(|_| SpiError::Communication(CommunicationError::BufferOverflow))?;
        }
        
        self.transfer_with_retry(chip_select, &write_data, &mut full_buffer).await?;
        
        // Copy data (skip first byte which is response to command)
        buffer.copy_from_slice(&full_buffer[1..buffer.len() + 1]);
        
        Ok(())
    }
    
    /// Write single register byte
    pub async fn write_register_byte(
        &mut self,
        chip_select: u8,
        register: u8,
        value: u8,
    ) -> Result<(), SpiError> {
        self.write_register(chip_select, register, &[value]).await
    }
    
    /// Read single register byte
    pub async fn read_register_byte(
        &mut self,
        chip_select: u8,
        register: u8,
    ) -> Result<u8, SpiError> {
        let mut buffer = [0u8; 1];
        self.read_register(chip_select, register, &mut buffer).await?;
        Ok(buffer[0])
    }
    
    /// Write 16-bit register (big-endian)
    pub async fn write_register_u16_be(
        &mut self,
        chip_select: u8,
        register: u8,
        value: u16,
    ) -> Result<(), SpiError> {
        let bytes = value.to_be_bytes();
        self.write_register(chip_select, register, &bytes).await
    }
    
    /// Read 16-bit register (big-endian)
    pub async fn read_register_u16_be(
        &mut self,
        chip_select: u8,
        register: u8,
    ) -> Result<u16, SpiError> {
        let mut buffer = [0u8; 2];
        self.read_register(chip_select, register, &mut buffer).await?;
        Ok(u16::from_be_bytes(buffer))
    }
    
    /// Write 16-bit register (little-endian)
    pub async fn write_register_u16_le(
        &mut self,
        chip_select: u8,
        register: u8,
        value: u16,
    ) -> Result<(), SpiError> {
        let bytes = value.to_le_bytes();
        self.write_register(chip_select, register, &bytes).await
    }
    
    /// Read 16-bit register (little-endian)
    pub async fn read_register_u16_le(
        &mut self,
        chip_select: u8,
        register: u8,
    ) -> Result<u16, SpiError> {
        let mut buffer = [0u8; 2];
        self.read_register(chip_select, register, &mut buffer).await?;
        Ok(u16::from_le_bytes(buffer))
    }
    
    /// Bulk transfer for high-speed operations
    pub async fn bulk_transfer(
        &mut self,
        chip_select: u8,
        write_data: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), SpiError> {
        // For bulk operations, we might want different retry logic or DMA
        self.transfer_with_retry(chip_select, write_data, read_buffer).await
    }
    
    /// Test device communication
    pub async fn test_device(&mut self, chip_select: u8) -> Result<bool, SpiError> {
        // Try to read a common register or perform a known operation
        // This is device-specific and would need to be implemented per device type
        
        if let Some(device) = self.devices.get(&chip_select) {
            match device.device_type {
                SpiDeviceType::Sensor => {
                    // Try reading WHO_AM_I register (common at 0x0F)
                    match self.read_register_byte(chip_select, 0x0F).await {
                        Ok(_) => {
                            self.diagnostics.update_device_status(chip_select, DeviceStatus::Online);
                            Ok(true)
                        }
                        Err(_) => {
                            self.diagnostics.update_device_status(chip_select, DeviceStatus::Offline);
                            Ok(false)
                        }
                    }
                }
                _ => {
                    // For other device types, just try a simple write/read
                    match self.write_with_retry(chip_select, &[0x00]).await {
                        Ok(_) => {
                            self.diagnostics.update_device_status(chip_select, DeviceStatus::Online);
                            Ok(true)
                        }
                        Err(_) => {
                            self.diagnostics.update_device_status(chip_select, DeviceStatus::Offline);
                            Ok(false)
                        }
                    }
                }
            }
        } else {
            Err(SpiError::Communication(CommunicationError::DeviceNotFound))
        }
    }
    
    /// Get device information
    pub fn get_device_info(&self, chip_select: u8) -> Option<&SpiDeviceInfo> {
        self.devices.get(&chip_select)
    }
    
    /// Get all registered devices
    pub fn get_all_devices(&self) -> impl Iterator<Item = (&u8, &SpiDeviceInfo)> {
        self.devices.iter()
    }
    
    /// Get devices by type
    pub fn get_devices_by_type(&self, device_type: SpiDeviceType) -> Vec<u8, 8> {
        let mut devices = Vec::new();
        for (&chip_select, info) in &self.devices {
            if info.device_type == device_type {
                let _ = devices.push(chip_select);
            }
        }
        devices
    }
    
    /// Update device configuration
    pub fn update_device_config(
        &mut self,
        chip_select: u8,
        config: SpiDeviceConfig,
    ) -> Result<(), SpiError> {
        if let Some(device) = self.devices.get_mut(&chip_select) {
            device.config = config;
            Ok(())
        } else {
            Err(SpiError::Communication(CommunicationError::DeviceNotFound))
        }
    }
    
    /// Set device name
    pub fn set_device_name(&mut self, chip_select: u8, name: &str) -> Result<(), SpiError> {
        if let Some(device) = self.devices.get_mut(&chip_select) {
            device.name = Some(heapless::String::from(name));
            Ok(())
        } else {
            Err(SpiError::Communication(CommunicationError::DeviceNotFound))
        }
    }
    
    /// Set device type
    pub fn set_device_type(
        &mut self,
        chip_select: u8,
        device_type: SpiDeviceType,
    ) -> Result<(), SpiError> {
        if let Some(device) = self.devices.get_mut(&chip_select) {
            device.device_type = device_type;
            Ok(())
        } else {
            Err(SpiError::Communication(CommunicationError::DeviceNotFound))
        }
    }
    
    /// Get communication statistics
    pub fn get_stats(&self) -> &CommunicationStats {
        &self.stats
    }
    
    /// Get device-specific statistics
    pub fn get_device_stats(&self, chip_select: u8) -> Option<&CommunicationStats> {
        self.devices.get(&chip_select).map(|device| &device.stats)
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
    
    /// Health check for all devices
    pub async fn health_check(&mut self) -> Result<Vec<(u8, DeviceStatus), 8>, SpiError> {
        let mut results = Vec::new();
        
        for &chip_select in self.devices.keys() {
            let is_online = self.test_device(chip_select).await?;
            let status = if is_online {
                DeviceStatus::Online
            } else {
                DeviceStatus::Offline
            };
            
            let _ = results.push((chip_select, status));
        }
        
        Ok(results)
    }
    
    /// Record successful operation
    fn record_success(&mut self, chip_select: u8, response_time_us: u32) {
        self.stats.record_success(response_time_us);
        
        if let Some(device) = self.devices.get_mut(&chip_select) {
            device.stats.record_success(response_time_us);
            device.status = DeviceStatus::Online;
            device.last_used = Some(self.get_timestamp());
        }
    }
    
    /// Record failed operation
    fn record_failure(&mut self, chip_select: u8, error: CommunicationError) {
        self.stats.record_failure(error.clone());
        
        if let Some(device) = self.devices.get_mut(&chip_select) {
            device.stats.record_failure(error.clone());
            device.status = DeviceStatus::Error;
        }
        
        self.diagnostics.record_error(error, self.get_timestamp());
    }
    
    /// Update device last used timestamp
    fn update_device_last_used(&mut self, chip_select: u8) {
        if let Some(device) = self.devices.get_mut(&chip_select) {
            device.last_used = Some(self.get_timestamp());
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
impl<SPI> CommunicationManager for SpiManager<SPI>
where
    SPI: SpiDevice + Send,
{
    type Error = SpiError;
    
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
        self.transfer_with_retry(address, write_data, read_buffer).await
    }
    
    async fn probe_device(&mut self, address: u8) -> Result<bool, Self::Error> {
        self.test_device(address).await
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

/// SPI utilities
pub mod spi_utils {
    use super::*;
    
    /// Calculate SPI clock divider
    pub fn calculate_clock_divider(system_clock_hz: u32, target_clock_hz: u32) -> u32 {
        let divider = system_clock_hz / target_clock_hz;
        // Ensure divider is even and at least 2
        if divider < 2 {
            2
        } else if divider % 2 == 0 {
            divider
        } else {
            divider + 1
        }
    }
    
    /// Get maximum safe clock speed for device type
    pub fn get_max_clock_speed(device_type: SpiDeviceType) -> u32 {
        match device_type {
            SpiDeviceType::Sensor => 10_000_000,    // 10MHz
            SpiDeviceType::Display => 20_000_000,   // 20MHz
            SpiDeviceType::Memory => 25_000_000,    // 25MHz
            SpiDeviceType::Flash => 50_000_000,     // 50MHz
            SpiDeviceType::SD_Card => 25_000_000,   // 25MHz
            SpiDeviceType::ADC => 5_000_000,        // 5MHz
            SpiDeviceType::DAC => 10_000_000,       // 10MHz
            SpiDeviceType::Radio => 8_000_000,      // 8MHz
            SpiDeviceType::Unknown => 1_000_000,    // 1MHz (safe default)
        }
    }
    
    /// Get recommended SPI mode for device type
    pub fn get_recommended_mode(device_type: SpiDeviceType) -> SpiMode {
        match device_type {
            SpiDeviceType::Sensor => SpiMode::Mode0,
            SpiDeviceType::Display => SpiMode::Mode0,
            SpiDeviceType::Memory => SpiMode::Mode0,
            SpiDeviceType::Flash => SpiMode::Mode0,
            SpiDeviceType::SD_Card => SpiMode::Mode0,
            SpiDeviceType::ADC => SpiMode::Mode0,
            SpiDeviceType::DAC => SpiMode::Mode0,
            SpiDeviceType::Radio => SpiMode::Mode0,
            SpiDeviceType::Unknown => SpiMode::Mode0,
        }
    }
    
    /// Create device configuration for common sensor types
    pub fn create_sensor_config(clock_speed_hz: u32) -> SpiDeviceConfig {
        SpiDeviceConfig {
            clock_speed_hz,
            mode: SpiMode::Mode0,
            bit_order: BitOrder::MsbFirst,
            word_size: 8,
            cs_active_low: true,
            cs_setup_time_ns: 100,
            cs_hold_time_ns: 100,
            inter_word_delay_ns: 0,
        }
    }
    
    /// Create device configuration for displays
    pub fn create_display_config(clock_speed_hz: u32) -> SpiDeviceConfig {
        SpiDeviceConfig {
            clock_speed_hz,
            mode: SpiMode::Mode0,
            bit_order: BitOrder::MsbFirst,
            word_size: 8,
            cs_active_low: true,
            cs_setup_time_ns: 50,
            cs_hold_time_ns: 50,
            inter_word_delay_ns: 0,
        }
    }
    
    /// Create device configuration for memory devices
    pub fn create_memory_config(clock_speed_hz: u32) -> SpiDeviceConfig {
        SpiDeviceConfig {
            clock_speed_hz,
            mode: SpiMode::Mode0,
            bit_order: BitOrder::MsbFirst,
            word_size: 8,
            cs_active_low: true,
            cs_setup_time_ns: 200,
            cs_hold_time_ns: 200,
            inter_word_delay_ns: 0,
        }
    }
    
    /// Validate SPI configuration
    pub fn validate_config(config: &SpiDeviceConfig) -> Result<(), SpiError> {
        if config.clock_speed_hz == 0 {
            return Err(SpiError::ClockConfigError);
        }
        
        if config.word_size == 0 || config.word_size > 32 {
            return Err(SpiError::FrameFormatError);
        }
        
        Ok(())
    }
    
    /// Convert SPI mode to CPOL/CPHA values
    pub fn mode_to_cpol_cpha(mode: SpiMode) -> (bool, bool) {
        match mode {
            SpiMode::Mode0 => (false, false), // CPOL=0, CPHA=0
            SpiMode::Mode1 => (false, true),  // CPOL=0, CPHA=1
            SpiMode::Mode2 => (true, false),  // CPOL=1, CPHA=0
            SpiMode::Mode3 => (true, true),   // CPOL=1, CPHA=1
        }
    }
    
    /// Convert CPOL/CPHA values to SPI mode
    pub fn cpol_cpha_to_mode(cpol: bool, cpha: bool) -> SpiMode {
        match (cpol, cpha) {
            (false, false) => SpiMode::Mode0,
            (false, true) => SpiMode::Mode1,
            (true, false) => SpiMode::Mode2,
            (true, true) => SpiMode::Mode3,
        }
    }
}