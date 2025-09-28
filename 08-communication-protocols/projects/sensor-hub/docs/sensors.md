# Sensor Documentation

## Supported Sensors

The Sensor Hub supports various types of sensors connected via I2C, SPI, and 1-Wire protocols.

## Temperature Sensors

### DS18B20 (1-Wire)
- **Protocol**: 1-Wire
- **Resolution**: 9-12 bits (0.5°C to 0.0625°C)
- **Range**: -55°C to +125°C
- **Features**: Waterproof versions available, unique 64-bit address
- **Use Case**: Outdoor temperature monitoring, liquid temperature

```rust
// Example usage
let temp_sensor = DS18B20::new(one_wire_pin);
let temperature = temp_sensor.read_temperature()?;
```

### BME280 Temperature (I2C/SPI)
- **Protocol**: I2C (0x76, 0x77) or SPI
- **Resolution**: 0.01°C
- **Range**: -40°C to +85°C
- **Features**: Combined temperature, humidity, and pressure
- **Use Case**: Indoor climate monitoring, weather stations

```rust
// I2C usage
let bme280 = BME280::new_i2c(i2c, 0x76)?;
let (temp, humidity, pressure) = bme280.read_all()?;
```

## Humidity Sensors

### BME280 Humidity (I2C/SPI)
- **Protocol**: I2C (0x76, 0x77) or SPI
- **Resolution**: 0.008% RH
- **Range**: 0-100% RH
- **Accuracy**: ±3% RH
- **Features**: Temperature compensation built-in

### SHT30 (I2C)
- **Protocol**: I2C (0x44, 0x45)
- **Resolution**: 0.01% RH
- **Range**: 0-100% RH
- **Accuracy**: ±2% RH
- **Features**: High accuracy, low power consumption

```rust
// SHT30 usage
let sht30 = SHT30::new(i2c, 0x44)?;
let (temp, humidity) = sht30.read()?;
```

## Pressure Sensors

### BME280 Pressure (I2C/SPI)
- **Protocol**: I2C (0x76, 0x77) or SPI
- **Resolution**: 0.18 Pa
- **Range**: 300-1100 hPa
- **Accuracy**: ±1 hPa
- **Features**: Sea level pressure calculation

### BMP280 (I2C/SPI)
- **Protocol**: I2C (0x76, 0x77) or SPI
- **Resolution**: 0.16 Pa
- **Range**: 300-1100 hPa
- **Accuracy**: ±1 hPa
- **Features**: Temperature compensation, altitude calculation

```rust
// BMP280 usage
let bmp280 = BMP280::new_i2c(i2c, 0x76)?;
let pressure = bmp280.read_pressure()?;
let altitude = bmp280.calculate_altitude(pressure, sea_level_pressure);
```

## Light Sensors

### BH1750 (I2C)
- **Protocol**: I2C (0x23, 0x5C)
- **Resolution**: 1 lux
- **Range**: 1-65535 lux
- **Features**: Automatic gain control, multiple measurement modes
- **Use Case**: Ambient light sensing, display brightness control

```rust
// BH1750 usage
let bh1750 = BH1750::new(i2c, 0x23)?;
let lux = bh1750.read_light_level()?;
```

### TSL2561 (I2C)
- **Protocol**: I2C (0x29, 0x39, 0x49)
- **Resolution**: Variable
- **Range**: 0.1-40,000 lux
- **Features**: Dual photodiodes, IR compensation
- **Use Case**: Precise light measurement, photography applications

```rust
// TSL2561 usage
let tsl2561 = TSL2561::new(i2c, 0x39)?;
let lux = tsl2561.read_lux()?;
```

## Sensor Configuration

### I2C Sensors Configuration

```toml
# sensors.toml
[i2c]
frequency = 400_000  # 400kHz
timeout_ms = 1000

[sensors.bme280]
address = 0x76
sampling_rate_hz = 1
oversampling_temp = "x2"
oversampling_humidity = "x1"
oversampling_pressure = "x16"
filter = "off"

[sensors.bh1750]
address = 0x23
mode = "continuous_high_res"
sampling_rate_hz = 0.5
```

### SPI Sensors Configuration

```toml
[spi]
frequency = 1_000_000  # 1MHz
mode = 0  # CPOL=0, CPHA=0

[sensors.ads1115]
cs_pin = 10
gain = "4_096v"
data_rate = "128_sps"
```

## Calibration

### Temperature Calibration

```rust
pub struct TemperatureCalibration {
    pub offset: f32,
    pub scale: f32,
}

impl TemperatureCalibration {
    pub fn apply(&self, raw_temp: f32) -> f32 {
        (raw_temp + self.offset) * self.scale
    }
}
```

### Pressure Calibration

```rust
pub struct PressureCalibration {
    pub sea_level_pressure: f32,  // hPa
    pub altitude_offset: f32,     // meters
}

impl PressureCalibration {
    pub fn calculate_altitude(&self, pressure: f32) -> f32 {
        let ratio = pressure / self.sea_level_pressure;
        44330.0 * (1.0 - ratio.powf(0.1903)) + self.altitude_offset
    }
}
```

## Error Handling

### Common Sensor Errors

```rust
#[derive(Debug, Clone)]
pub enum SensorError {
    CommunicationError,
    InvalidData,
    SensorNotFound,
    CalibrationError,
    TimeoutError,
}
```

### Retry Mechanisms

```rust
pub async fn read_with_retry<T>(
    sensor: &mut T,
    max_retries: u8,
) -> Result<SensorData, SensorError>
where
    T: SensorTrait,
{
    for attempt in 0..=max_retries {
        match sensor.read().await {
            Ok(data) => return Ok(data),
            Err(e) if attempt == max_retries => return Err(e),
            Err(_) => {
                // Wait before retry
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
    unreachable!()
}
```

## Power Management

### Low Power Modes

```rust
pub trait PowerManagement {
    async fn enter_sleep_mode(&mut self) -> Result<(), SensorError>;
    async fn wake_up(&mut self) -> Result<(), SensorError>;
    fn get_power_consumption(&self) -> PowerConsumption;
}

pub struct PowerConsumption {
    pub active_current_ua: u32,
    pub sleep_current_ua: u32,
    pub startup_time_ms: u32,
}
```

## Sensor Data Format

### Unified Data Structure

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorReading {
    pub sensor_id: u8,
    pub timestamp: u64,
    pub data: SensorData,
    pub quality: DataQuality,
}

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
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DataQuality {
    Good,
    Warning(String),
    Error(String),
}
```