# Sensor Hub API Documentation

## Overview

The Sensor Hub provides a comprehensive API for sensor management, data collection, and system configuration. The API is designed to be type-safe, async-friendly, and suitable for embedded environments.

## Core Traits

### SensorTrait

Base trait for all sensors in the system.

```rust
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;

#[async_trait]
pub trait SensorTrait {
    type Error;
    type Data;
    
    /// Initialize the sensor
    async fn init(&mut self) -> Result<(), Self::Error>;
    
    /// Read data from the sensor
    async fn read(&mut self) -> Result<Self::Data, Self::Error>;
    
    /// Get sensor information
    fn get_info(&self) -> SensorInfo;
    
    /// Check if sensor is available
    async fn is_available(&mut self) -> bool;
    
    /// Enter low power mode
    async fn sleep(&mut self) -> Result<(), Self::Error>;
    
    /// Wake up from low power mode
    async fn wake(&mut self) -> Result<(), Self::Error>;
}
```

### CommunicationManager

Trait for managing communication protocols.

```rust
#[async_trait]
pub trait CommunicationManager {
    type Error;
    
    /// Initialize the communication interface
    async fn init(&mut self) -> Result<(), Self::Error>;
    
    /// Scan for available devices
    async fn scan_devices(&mut self) -> Result<Vec<u8>, Self::Error>;
    
    /// Check if device is responsive
    async fn ping_device(&mut self, address: u8) -> Result<bool, Self::Error>;
    
    /// Reset the communication interface
    async fn reset(&mut self) -> Result<(), Self::Error>;
}
```

## Sensor Modules

### Temperature Sensor API

```rust
pub mod temperature {
    use super::*;
    
    #[derive(Debug, Clone)]
    pub struct TemperatureReading {
        pub celsius: f32,
        pub fahrenheit: f32,
        pub kelvin: f32,
        pub timestamp: u64,
    }
    
    #[async_trait]
    pub trait TemperatureSensor: SensorTrait {
        /// Read temperature in Celsius
        async fn read_celsius(&mut self) -> Result<f32, Self::Error>;
        
        /// Read temperature in Fahrenheit
        async fn read_fahrenheit(&mut self) -> Result<f32, Self::Error> {
            let celsius = self.read_celsius().await?;
            Ok(celsius * 9.0 / 5.0 + 32.0)
        }
        
        /// Read temperature in Kelvin
        async fn read_kelvin(&mut self) -> Result<f32, Self::Error> {
            let celsius = self.read_celsius().await?;
            Ok(celsius + 273.15)
        }
        
        /// Get temperature resolution
        fn get_resolution(&self) -> f32;
        
        /// Set temperature resolution (if supported)
        async fn set_resolution(&mut self, resolution: f32) -> Result<(), Self::Error>;
    }
    
    // DS18B20 Implementation
    pub struct DS18B20<P> {
        pin: P,
        resolution: u8,
    }
    
    impl<P> DS18B20<P> {
        pub fn new(pin: P) -> Self {
            Self {
                pin,
                resolution: 12, // 12-bit resolution by default
            }
        }
        
        pub async fn read_temperature(&mut self) -> Result<f32, DS18B20Error> {
            // Implementation details...
            todo!()
        }
    }
}
```

### Humidity Sensor API

```rust
pub mod humidity {
    use super::*;
    
    #[derive(Debug, Clone)]
    pub struct HumidityReading {
        pub relative_humidity: f32, // %RH
        pub absolute_humidity: f32, // g/m³
        pub timestamp: u64,
    }
    
    #[async_trait]
    pub trait HumiditySensor: SensorTrait {
        /// Read relative humidity in %RH
        async fn read_humidity(&mut self) -> Result<f32, Self::Error>;
        
        /// Calculate absolute humidity
        async fn read_absolute_humidity(&mut self, temperature: f32) -> Result<f32, Self::Error> {
            let rh = self.read_humidity().await?;
            // Calculate absolute humidity using temperature
            let saturation_pressure = 6.112 * (17.67 * temperature / (temperature + 243.5)).exp();
            let actual_pressure = rh / 100.0 * saturation_pressure;
            Ok(actual_pressure * 2.1674 / (temperature + 273.15))
        }
        
        /// Get humidity accuracy
        fn get_accuracy(&self) -> f32;
    }
}
```

### Pressure Sensor API

```rust
pub mod pressure {
    use super::*;
    
    #[derive(Debug, Clone)]
    pub struct PressureReading {
        pub pressure_hpa: f32,
        pub pressure_pa: f32,
        pub altitude_m: f32,
        pub timestamp: u64,
    }
    
    #[async_trait]
    pub trait PressureSensor: SensorTrait {
        /// Read pressure in hPa
        async fn read_pressure(&mut self) -> Result<f32, Self::Error>;
        
        /// Calculate altitude from pressure
        async fn calculate_altitude(&mut self, sea_level_pressure: f32) -> Result<f32, Self::Error> {
            let pressure = self.read_pressure().await?;
            let ratio = pressure / sea_level_pressure;
            Ok(44330.0 * (1.0 - ratio.powf(0.1903)))
        }
        
        /// Get pressure range
        fn get_pressure_range(&self) -> (f32, f32);
    }
}
```

### Light Sensor API

```rust
pub mod light {
    use super::*;
    
    #[derive(Debug, Clone)]
    pub struct LightReading {
        pub lux: f32,
        pub ir_level: Option<f32>,
        pub visible_level: Option<f32>,
        pub timestamp: u64,
    }
    
    #[async_trait]
    pub trait LightSensor: SensorTrait {
        /// Read light level in lux
        async fn read_lux(&mut self) -> Result<f32, Self::Error>;
        
        /// Read infrared level (if supported)
        async fn read_ir(&mut self) -> Result<Option<f32>, Self::Error> {
            Ok(None)
        }
        
        /// Read visible light level (if supported)
        async fn read_visible(&mut self) -> Result<Option<f32>, Self::Error> {
            Ok(None)
        }
        
        /// Set measurement mode
        async fn set_mode(&mut self, mode: LightMode) -> Result<(), Self::Error>;
    }
    
    #[derive(Debug, Clone, Copy)]
    pub enum LightMode {
        OneTime,
        Continuous,
        HighResolution,
        LowPower,
    }
}
```

## Communication Managers

### I2C Manager API

```rust
pub mod i2c_manager {
    use super::*;
    use embedded_hal_async::i2c::I2c;
    
    pub struct I2CManager<I2C> {
        i2c: I2C,
        devices: heapless::FnvIndexMap<u8, DeviceInfo, 16>,
    }
    
    impl<I2C> I2CManager<I2C>
    where
        I2C: I2c,
    {
        pub fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                devices: heapless::FnvIndexMap::new(),
            }
        }
        
        /// Scan I2C bus for devices
        pub async fn scan_bus(&mut self) -> Result<Vec<u8>, I2CError> {
            let mut found_devices = Vec::new();
            
            for addr in 0x08..=0x77 {
                if self.ping_device(addr).await? {
                    found_devices.push(addr);
                }
            }
            
            Ok(found_devices)
        }
        
        /// Register a device
        pub fn register_device(&mut self, address: u8, info: DeviceInfo) -> Result<(), I2CError> {
            self.devices.insert(address, info)
                .map_err(|_| I2CError::DeviceRegistrationFailed)?;
            Ok(())
        }
        
        /// Read from device
        pub async fn read_from_device(
            &mut self,
            address: u8,
            buffer: &mut [u8],
        ) -> Result<(), I2CError> {
            self.i2c.read(address, buffer).await
                .map_err(|_| I2CError::CommunicationFailed)
        }
        
        /// Write to device
        pub async fn write_to_device(
            &mut self,
            address: u8,
            data: &[u8],
        ) -> Result<(), I2CError> {
            self.i2c.write(address, data).await
                .map_err(|_| I2CError::CommunicationFailed)
        }
        
        /// Write then read from device
        pub async fn write_read(
            &mut self,
            address: u8,
            write_data: &[u8],
            read_buffer: &mut [u8],
        ) -> Result<(), I2CError> {
            self.i2c.write_read(address, write_data, read_buffer).await
                .map_err(|_| I2CError::CommunicationFailed)
        }
    }
}
```

### SPI Manager API

```rust
pub mod spi_manager {
    use super::*;
    use embedded_hal_async::spi::SpiDevice;
    
    pub struct SPIManager<SPI> {
        spi: SPI,
        devices: heapless::FnvIndexMap<u8, SPIDeviceInfo, 8>,
    }
    
    impl<SPI> SPIManager<SPI>
    where
        SPI: SpiDevice,
    {
        pub fn new(spi: SPI) -> Self {
            Self {
                spi,
                devices: heapless::FnvIndexMap::new(),
            }
        }
        
        /// Transfer data with device
        pub async fn transfer(
            &mut self,
            device_id: u8,
            data: &mut [u8],
        ) -> Result<(), SPIError> {
            self.spi.transfer(data).await
                .map_err(|_| SPIError::TransferFailed)
        }
        
        /// Write data to device
        pub async fn write(
            &mut self,
            device_id: u8,
            data: &[u8],
        ) -> Result<(), SPIError> {
            self.spi.write(data).await
                .map_err(|_| SPIError::WriteFailed)
        }
        
        /// Read data from device
        pub async fn read(
            &mut self,
            device_id: u8,
            buffer: &mut [u8],
        ) -> Result<(), SPIError> {
            self.spi.read(buffer).await
                .map_err(|_| SPIError::ReadFailed)
        }
    }
}
```

## Data Processing API

### Data Collector

```rust
pub mod collector {
    use super::*;
    
    pub struct DataCollector {
        sensors: heapless::Vec<Box<dyn SensorTrait>, 16>,
        collection_interval: Duration,
        last_collection: Option<Instant>,
    }
    
    impl DataCollector {
        pub fn new(interval: Duration) -> Self {
            Self {
                sensors: heapless::Vec::new(),
                collection_interval: interval,
                last_collection: None,
            }
        }
        
        /// Add sensor to collection
        pub fn add_sensor(&mut self, sensor: Box<dyn SensorTrait>) -> Result<(), CollectorError> {
            self.sensors.push(sensor)
                .map_err(|_| CollectorError::TooManySensors)
        }
        
        /// Collect data from all sensors
        pub async fn collect_all(&mut self) -> Result<Vec<SensorReading>, CollectorError> {
            let mut readings = Vec::new();
            
            for (id, sensor) in self.sensors.iter_mut().enumerate() {
                match sensor.read().await {
                    Ok(data) => {
                        readings.push(SensorReading {
                            sensor_id: id as u8,
                            timestamp: get_timestamp(),
                            data,
                            quality: DataQuality::Good,
                        });
                    }
                    Err(e) => {
                        readings.push(SensorReading {
                            sensor_id: id as u8,
                            timestamp: get_timestamp(),
                            data: SensorData::Error,
                            quality: DataQuality::Error(format!("{:?}", e)),
                        });
                    }
                }
            }
            
            self.last_collection = Some(Instant::now());
            Ok(readings)
        }
        
        /// Check if it's time to collect data
        pub fn should_collect(&self) -> bool {
            match self.last_collection {
                None => true,
                Some(last) => last.elapsed() >= self.collection_interval,
            }
        }
    }
}
```

### Data Processor

```rust
pub mod processor {
    use super::*;
    
    pub struct DataProcessor {
        filters: heapless::Vec<Box<dyn Filter>, 8>,
        calibrations: heapless::FnvIndexMap<u8, Calibration, 16>,
    }
    
    impl DataProcessor {
        pub fn new() -> Self {
            Self {
                filters: heapless::Vec::new(),
                calibrations: heapless::FnvIndexMap::new(),
            }
        }
        
        /// Process sensor reading
        pub fn process(&mut self, reading: SensorReading) -> Result<SensorReading, ProcessorError> {
            let mut processed = reading;
            
            // Apply calibration
            if let Some(calibration) = self.calibrations.get(&processed.sensor_id) {
                processed.data = calibration.apply(processed.data)?;
            }
            
            // Apply filters
            for filter in &mut self.filters {
                processed.data = filter.apply(processed.data)?;
            }
            
            Ok(processed)
        }
        
        /// Add calibration for sensor
        pub fn add_calibration(&mut self, sensor_id: u8, calibration: Calibration) {
            self.calibrations.insert(sensor_id, calibration);
        }
        
        /// Add filter
        pub fn add_filter(&mut self, filter: Box<dyn Filter>) -> Result<(), ProcessorError> {
            self.filters.push(filter)
                .map_err(|_| ProcessorError::TooManyFilters)
        }
    }
    
    pub trait Filter {
        fn apply(&mut self, data: SensorData) -> Result<SensorData, ProcessorError>;
    }
}
```

## Error Types

```rust
#[derive(Debug, Clone)]
pub enum SensorHubError {
    I2CError(I2CError),
    SPIError(SPIError),
    SensorError(SensorError),
    CollectorError(CollectorError),
    ProcessorError(ProcessorError),
    StorageError(StorageError),
}

#[derive(Debug, Clone)]
pub enum I2CError {
    CommunicationFailed,
    DeviceNotFound,
    DeviceRegistrationFailed,
    BusError,
}

#[derive(Debug, Clone)]
pub enum SPIError {
    TransferFailed,
    WriteFailed,
    ReadFailed,
    DeviceNotSelected,
}

#[derive(Debug, Clone)]
pub enum SensorError {
    InitializationFailed,
    ReadFailed,
    InvalidData,
    NotAvailable,
    CalibrationError,
}
```

## Usage Examples

### Basic Sensor Reading

```rust
use sensor_hub::*;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize I2C
    let i2c = I2c::new(/* ... */);
    let mut i2c_manager = I2CManager::new(i2c);
    
    // Initialize BME280 sensor
    let mut bme280 = BME280::new_i2c(&mut i2c_manager, 0x76).await?;
    
    // Read sensor data
    loop {
        match bme280.read().await {
            Ok(data) => {
                info!("Temperature: {:.2}°C", data.temperature);
                info!("Humidity: {:.1}%RH", data.humidity);
                info!("Pressure: {:.1} hPa", data.pressure);
            }
            Err(e) => {
                error!("Sensor read error: {:?}", e);
            }
        }
        
        Timer::after(Duration::from_secs(1)).await;
    }
}
```

### Multi-Sensor Data Collection

```rust
use sensor_hub::*;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut collector = DataCollector::new(Duration::from_secs(5));
    let mut processor = DataProcessor::new();
    let mut storage = DataStorage::new();
    
    // Add sensors
    collector.add_sensor(Box::new(bme280))?;
    collector.add_sensor(Box::new(bh1750))?;
    
    // Main collection loop
    loop {
        if collector.should_collect() {
            let readings = collector.collect_all().await?;
            
            for reading in readings {
                let processed = processor.process(reading)?;
                storage.store(processed).await?;
            }
        }
        
        Timer::after(Duration::from_millis(100)).await;
    }
}
```