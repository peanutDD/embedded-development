//! Communication module for sensor hub
//!
//! This module provides communication managers for different protocols
//! including I2C and SPI interfaces.

use embassy_time::{Duration, Timer};
use embedded_hal_async::{i2c::I2c, spi::SpiDevice};
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

pub mod i2c_manager;
pub mod spi_manager;

pub use i2c_manager::*;
pub use spi_manager::*;

/// Communication protocol types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Protocol {
    I2C,
    SPI,
    UART,
    OneWire,
    CAN,
}

/// Communication error types
#[derive(Debug, Clone, PartialEq)]
pub enum CommunicationError {
    /// Bus error (e.g., no ACK, arbitration lost)
    BusError,
    /// Device not found or not responding
    DeviceNotFound,
    /// Invalid address
    InvalidAddress,
    /// Timeout occurred
    Timeout,
    /// Buffer overflow
    BufferOverflow,
    /// Invalid data format
    InvalidData,
    /// Checksum/CRC error
    ChecksumError,
    /// Protocol-specific error
    ProtocolError(u8),
    /// Hardware error
    HardwareError,
    /// Configuration error
    ConfigurationError,
}

/// Communication statistics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CommunicationStats {
    pub total_transactions: u32,
    pub successful_transactions: u32,
    pub failed_transactions: u32,
    pub timeout_count: u32,
    pub retry_count: u32,
    pub average_response_time_us: u32,
    pub last_error: Option<CommunicationError>,
}

impl CommunicationStats {
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Calculate success rate as percentage
    pub fn success_rate(&self) -> f32 {
        if self.total_transactions == 0 {
            0.0
        } else {
            (self.successful_transactions as f32 / self.total_transactions as f32) * 100.0
        }
    }
    
    /// Record successful transaction
    pub fn record_success(&mut self, response_time_us: u32) {
        self.total_transactions += 1;
        self.successful_transactions += 1;
        
        // Update average response time
        if self.successful_transactions == 1 {
            self.average_response_time_us = response_time_us;
        } else {
            self.average_response_time_us = 
                (self.average_response_time_us * (self.successful_transactions - 1) + response_time_us) 
                / self.successful_transactions;
        }
    }
    
    /// Record failed transaction
    pub fn record_failure(&mut self, error: CommunicationError) {
        self.total_transactions += 1;
        self.failed_transactions += 1;
        self.last_error = Some(error.clone());
        
        match error {
            CommunicationError::Timeout => self.timeout_count += 1,
            _ => {}
        }
    }
    
    /// Record retry attempt
    pub fn record_retry(&mut self) {
        self.retry_count += 1;
    }
    
    /// Reset statistics
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

/// Device configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceConfig {
    pub address: u8,
    pub protocol: Protocol,
    pub timeout_ms: u32,
    pub retry_count: u8,
    pub clock_speed_hz: Option<u32>,
    pub data_bits: Option<u8>,
    pub stop_bits: Option<u8>,
    pub parity: Option<Parity>,
}

/// Parity settings for UART
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Parity {
    None,
    Even,
    Odd,
}

impl DeviceConfig {
    /// Create I2C device configuration
    pub fn i2c(address: u8, clock_speed_hz: u32) -> Self {
        Self {
            address,
            protocol: Protocol::I2C,
            timeout_ms: 1000,
            retry_count: 3,
            clock_speed_hz: Some(clock_speed_hz),
            data_bits: None,
            stop_bits: None,
            parity: None,
        }
    }
    
    /// Create SPI device configuration
    pub fn spi(address: u8, clock_speed_hz: u32) -> Self {
        Self {
            address,
            protocol: Protocol::SPI,
            timeout_ms: 1000,
            retry_count: 3,
            clock_speed_hz: Some(clock_speed_hz),
            data_bits: None,
            stop_bits: None,
            parity: None,
        }
    }
    
    /// Create UART device configuration
    pub fn uart(address: u8, baud_rate: u32, data_bits: u8, stop_bits: u8, parity: Parity) -> Self {
        Self {
            address,
            protocol: Protocol::UART,
            timeout_ms: 1000,
            retry_count: 3,
            clock_speed_hz: Some(baud_rate),
            data_bits: Some(data_bits),
            stop_bits: Some(stop_bits),
            parity: Some(parity),
        }
    }
}

/// Communication manager trait
#[async_trait::async_trait]
pub trait CommunicationManager {
    type Error;
    
    /// Initialize the communication interface
    async fn init(&mut self) -> Result<(), Self::Error>;
    
    /// Write data to a device
    async fn write(&mut self, address: u8, data: &[u8]) -> Result<(), Self::Error>;
    
    /// Read data from a device
    async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;
    
    /// Write then read (common for register access)
    async fn write_read(&mut self, address: u8, write_data: &[u8], read_buffer: &mut [u8]) -> Result<(), Self::Error>;
    
    /// Check if device is present/responding
    async fn probe_device(&mut self, address: u8) -> Result<bool, Self::Error>;
    
    /// Get communication statistics
    fn get_stats(&self) -> &CommunicationStats;
    
    /// Reset communication statistics
    fn reset_stats(&mut self);
    
    /// Set timeout for operations
    fn set_timeout(&mut self, timeout_ms: u32);
    
    /// Set retry count for failed operations
    fn set_retry_count(&mut self, count: u8);
}

/// Device registry for managing multiple devices
pub struct DeviceRegistry {
    devices: FnvIndexMap<u8, DeviceConfig, 16>,
    active_devices: Vec<u8, 16>,
}

impl DeviceRegistry {
    pub fn new() -> Self {
        Self {
            devices: FnvIndexMap::new(),
            active_devices: Vec::new(),
        }
    }
    
    /// Register a device
    pub fn register_device(&mut self, config: DeviceConfig) -> Result<(), CommunicationError> {
        let address = config.address;
        self.devices.insert(address, config)
            .map_err(|_| CommunicationError::BufferOverflow)?;
        Ok(())
    }
    
    /// Unregister a device
    pub fn unregister_device(&mut self, address: u8) -> Result<(), CommunicationError> {
        self.devices.remove(&address)
            .ok_or(CommunicationError::DeviceNotFound)?;
        
        // Remove from active devices if present
        if let Some(pos) = self.active_devices.iter().position(|&x| x == address) {
            self.active_devices.remove(pos);
        }
        
        Ok(())
    }
    
    /// Get device configuration
    pub fn get_device_config(&self, address: u8) -> Option<&DeviceConfig> {
        self.devices.get(&address)
    }
    
    /// Mark device as active
    pub fn mark_active(&mut self, address: u8) -> Result<(), CommunicationError> {
        if !self.devices.contains_key(&address) {
            return Err(CommunicationError::DeviceNotFound);
        }
        
        if !self.active_devices.contains(&address) {
            self.active_devices.push(address)
                .map_err(|_| CommunicationError::BufferOverflow)?;
        }
        
        Ok(())
    }
    
    /// Mark device as inactive
    pub fn mark_inactive(&mut self, address: u8) {
        if let Some(pos) = self.active_devices.iter().position(|&x| x == address) {
            self.active_devices.remove(pos);
        }
    }
    
    /// Get list of active devices
    pub fn get_active_devices(&self) -> &[u8] {
        &self.active_devices
    }
    
    /// Get list of all registered devices
    pub fn get_all_devices(&self) -> impl Iterator<Item = (&u8, &DeviceConfig)> {
        self.devices.iter()
    }
    
    /// Check if device is registered
    pub fn is_registered(&self, address: u8) -> bool {
        self.devices.contains_key(&address)
    }
    
    /// Check if device is active
    pub fn is_active(&self, address: u8) -> bool {
        self.active_devices.contains(&address)
    }
    
    /// Get device count by protocol
    pub fn count_by_protocol(&self, protocol: Protocol) -> usize {
        self.devices.values()
            .filter(|config| config.protocol == protocol)
            .count()
    }
}

/// Communication utilities
pub mod comm_utils {
    use super::*;
    
    /// Calculate CRC8 checksum
    pub fn crc8(data: &[u8]) -> u8 {
        let mut crc = 0u8;
        for &byte in data {
            crc ^= byte;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }
    
    /// Calculate CRC16 checksum
    pub fn crc16(data: &[u8]) -> u16 {
        let mut crc = 0xFFFFu16;
        for &byte in data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if crc & 0x0001 != 0 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        crc
    }
    
    /// Validate I2C address
    pub fn is_valid_i2c_address(address: u8) -> bool {
        // 7-bit I2C addresses: 0x08-0x77 (excluding reserved addresses)
        match address {
            0x00..=0x07 => false, // Reserved addresses
            0x78..=0x7F => false, // Reserved addresses
            _ => true,
        }
    }
    
    /// Convert 7-bit I2C address to 8-bit (with R/W bit)
    pub fn i2c_address_with_rw(address: u8, read: bool) -> u8 {
        (address << 1) | if read { 1 } else { 0 }
    }
    
    /// Extract 7-bit address from 8-bit I2C address
    pub fn extract_i2c_address(address_with_rw: u8) -> u8 {
        address_with_rw >> 1
    }
    
    /// Check if operation is read or write from 8-bit I2C address
    pub fn is_i2c_read(address_with_rw: u8) -> bool {
        address_with_rw & 0x01 != 0
    }
    
    /// Convert frequency to period in microseconds
    pub fn freq_to_period_us(freq_hz: u32) -> u32 {
        1_000_000 / freq_hz
    }
    
    /// Convert period in microseconds to frequency
    pub fn period_us_to_freq(period_us: u32) -> u32 {
        1_000_000 / period_us
    }
    
    /// Calculate optimal clock divider for target frequency
    pub fn calculate_clock_divider(system_clock_hz: u32, target_freq_hz: u32) -> u32 {
        (system_clock_hz + target_freq_hz - 1) / target_freq_hz // Ceiling division
    }
    
    /// Validate data buffer size
    pub fn validate_buffer_size(size: usize, max_size: usize) -> Result<(), CommunicationError> {
        if size > max_size {
            Err(CommunicationError::BufferOverflow)
        } else {
            Ok(())
        }
    }
    
    /// Create timeout duration from milliseconds
    pub fn timeout_from_ms(ms: u32) -> Duration {
        Duration::from_millis(ms as u64)
    }
    
    /// Retry operation with exponential backoff
    pub async fn retry_with_backoff<F, T, E>(
        mut operation: F,
        max_retries: u8,
        initial_delay_ms: u32,
    ) -> Result<T, E>
    where
        F: FnMut() -> Result<T, E>,
    {
        let mut delay_ms = initial_delay_ms;
        
        for attempt in 0..=max_retries {
            match operation() {
                Ok(result) => return Ok(result),
                Err(e) => {
                    if attempt == max_retries {
                        return Err(e);
                    }
                    
                    // Wait before retry with exponential backoff
                    Timer::after(Duration::from_millis(delay_ms as u64)).await;
                    delay_ms = (delay_ms * 2).min(1000); // Cap at 1 second
                }
            }
        }
        
        unreachable!()
    }
}

/// Bus scanner for discovering devices
pub struct BusScanner {
    scan_range: (u8, u8),
    timeout_ms: u32,
}

impl BusScanner {
    pub fn new() -> Self {
        Self {
            scan_range: (0x08, 0x77), // Standard I2C range
            timeout_ms: 100,
        }
    }
    
    /// Set address range to scan
    pub fn set_range(&mut self, start: u8, end: u8) {
        self.scan_range = (start, end);
    }
    
    /// Set timeout for each probe
    pub fn set_timeout(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
    }
    
    /// Scan for I2C devices
    pub async fn scan_i2c<I>(&self, i2c: &mut I) -> Vec<u8, 16>
    where
        I: I2c,
    {
        let mut found_devices = Vec::new();
        
        for address in self.scan_range.0..=self.scan_range.1 {
            if comm_utils::is_valid_i2c_address(address) {
                // Try to write to the device (0 bytes)
                if i2c.write(address, &[]).await.is_ok() {
                    let _ = found_devices.push(address);
                }
                
                // Small delay between probes
                Timer::after(Duration::from_millis(1)).await;
            }
        }
        
        found_devices
    }
    
    /// Scan for devices and return with protocol info
    pub async fn scan_with_protocol<I>(&self, i2c: &mut I, protocol: Protocol) -> Vec<(u8, Protocol), 16>
    where
        I: I2c,
    {
        let addresses = self.scan_i2c(i2c).await;
        let mut devices = Vec::new();
        
        for address in addresses {
            let _ = devices.push((address, protocol));
        }
        
        devices
    }
}

/// Communication diagnostics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommunicationDiagnostics {
    pub bus_health: BusHealth,
    pub device_status: FnvIndexMap<u8, DeviceStatus, 16>,
    pub error_history: Vec<(u64, CommunicationError), 32>, // (timestamp, error)
    pub performance_metrics: PerformanceMetrics,
}

/// Bus health status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BusHealth {
    Healthy,
    Degraded,
    Critical,
    Offline,
}

/// Device status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceStatus {
    Online,
    Offline,
    Intermittent,
    Error,
}

/// Performance metrics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    pub throughput_bps: u32,
    pub latency_us: u32,
    pub error_rate: f32,
    pub uptime_seconds: u32,
}

impl CommunicationDiagnostics {
    pub fn new() -> Self {
        Self {
            bus_health: BusHealth::Healthy,
            device_status: FnvIndexMap::new(),
            error_history: Vec::new(),
            performance_metrics: PerformanceMetrics::default(),
        }
    }
    
    /// Update device status
    pub fn update_device_status(&mut self, address: u8, status: DeviceStatus) {
        let _ = self.device_status.insert(address, status);
    }
    
    /// Record error
    pub fn record_error(&mut self, error: CommunicationError, timestamp: u64) {
        if self.error_history.len() >= 32 {
            self.error_history.remove(0);
        }
        let _ = self.error_history.push((timestamp, error));
    }
    
    /// Calculate overall bus health
    pub fn calculate_bus_health(&mut self) -> BusHealth {
        let online_devices = self.device_status.values()
            .filter(|&&status| status == DeviceStatus::Online)
            .count();
        
        let total_devices = self.device_status.len();
        
        if total_devices == 0 {
            self.bus_health = BusHealth::Offline;
        } else {
            let online_ratio = online_devices as f32 / total_devices as f32;
            
            self.bus_health = match online_ratio {
                r if r >= 0.9 => BusHealth::Healthy,
                r if r >= 0.7 => BusHealth::Degraded,
                r if r > 0.0 => BusHealth::Critical,
                _ => BusHealth::Offline,
            };
        }
        
        self.bus_health
    }
    
    /// Get recent error count
    pub fn get_recent_error_count(&self, since_timestamp: u64) -> usize {
        self.error_history.iter()
            .filter(|(timestamp, _)| *timestamp >= since_timestamp)
            .count()
    }
}