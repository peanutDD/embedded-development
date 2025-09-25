# 传感器集成指南

## 概述

本指南详细介绍如何在STM32F4系统中集成各种I2C和SPI传感器，包括驱动开发、数据处理、校准和优化等关键技术。

## 传感器分类

### I2C 传感器

#### 环境传感器
```rust
pub trait EnvironmentalSensor {
    type Reading;
    type Error;
    
    fn read_data(&mut self) -> Result<Self::Reading, Self::Error>;
    fn configure(&mut self, config: &SensorConfig) -> Result<(), Self::Error>;
    fn calibrate(&mut self, calibration_data: &CalibrationData) -> Result<(), Self::Error>;
}

// 温湿度传感器 (SHT30)
pub struct Sht30Sensor<I2C> {
    i2c: I2C,
    address: u8,
    config: Sht30Config,
}

#[derive(Debug)]
pub struct Sht30Reading {
    pub temperature: f32,  // 摄氏度
    pub humidity: f32,     // 相对湿度 %
    pub timestamp: u64,
}

impl<I2C> Sht30Sensor<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            config: Sht30Config::default(),
        }
    }
    
    pub fn read_temperature_humidity(&mut self) -> Result<Sht30Reading, Sht30Error> {
        // 发送测量命令
        self.i2c.write(self.address, &[0x2C, 0x06])?;
        
        // 等待测量完成
        delay_ms(15);
        
        // 读取数据
        let mut buffer = [0u8; 6];
        self.i2c.read(self.address, &mut buffer)?;
        
        // 解析数据
        let temp_raw = ((buffer[0] as u16) << 8) | (buffer[1] as u16);
        let hum_raw = ((buffer[3] as u16) << 8) | (buffer[4] as u16);
        
        // 转换为物理值
        let temperature = -45.0 + 175.0 * (temp_raw as f32) / 65535.0;
        let humidity = 100.0 * (hum_raw as f32) / 65535.0;
        
        Ok(Sht30Reading {
            temperature,
            humidity,
            timestamp: get_timestamp(),
        })
    }
}

// 气压传感器 (BMP280)
pub struct Bmp280Sensor<I2C> {
    i2c: I2C,
    address: u8,
    calibration: Bmp280Calibration,
}

#[derive(Debug)]
pub struct Bmp280Reading {
    pub pressure: f32,     // hPa
    pub temperature: f32,  // 摄氏度
    pub altitude: f32,     // 米 (基于标准大气压)
}

impl<I2C> Bmp280Sensor<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn init(&mut self) -> Result<(), Bmp280Error> {
        // 读取校准数据
        self.read_calibration_data()?;
        
        // 配置传感器
        self.write_register(0xF4, 0x27)?; // 温度和压力过采样
        self.write_register(0xF5, 0xA0)?; // 配置寄存器
        
        Ok(())
    }
    
    pub fn read_pressure_temperature(&mut self) -> Result<Bmp280Reading, Bmp280Error> {
        // 读取原始数据
        let mut buffer = [0u8; 6];
        self.read_registers(0xF7, &mut buffer)?;
        
        // 解析原始值
        let pressure_raw = ((buffer[0] as u32) << 12) | 
                          ((buffer[1] as u32) << 4) | 
                          ((buffer[2] as u32) >> 4);
        let temperature_raw = ((buffer[3] as u32) << 12) | 
                             ((buffer[4] as u32) << 4) | 
                             ((buffer[5] as u32) >> 4);
        
        // 应用校准补偿
        let temperature = self.compensate_temperature(temperature_raw);
        let pressure = self.compensate_pressure(pressure_raw);
        let altitude = self.calculate_altitude(pressure);
        
        Ok(Bmp280Reading {
            pressure,
            temperature,
            altitude,
        })
    }
}
```

#### 运动传感器
```rust
// 加速度计和陀螺仪 (MPU6050)
pub struct Mpu6050Sensor<I2C> {
    i2c: I2C,
    address: u8,
    accel_scale: AccelScale,
    gyro_scale: GyroScale,
}

#[derive(Debug)]
pub struct Mpu6050Reading {
    pub accel: Vector3<f32>,    // m/s²
    pub gyro: Vector3<f32>,     // rad/s
    pub temperature: f32,       // 摄氏度
}

#[derive(Debug)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<I2C> Mpu6050Sensor<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn init(&mut self) -> Result<(), Mpu6050Error> {
        // 唤醒设备
        self.write_register(0x6B, 0x00)?;
        
        // 配置加速度计量程
        self.write_register(0x1C, self.accel_scale as u8)?;
        
        // 配置陀螺仪量程
        self.write_register(0x1B, self.gyro_scale as u8)?;
        
        // 配置数字低通滤波器
        self.write_register(0x1A, 0x03)?;
        
        Ok(())
    }
    
    pub fn read_all(&mut self) -> Result<Mpu6050Reading, Mpu6050Error> {
        let mut buffer = [0u8; 14];
        self.read_registers(0x3B, &mut buffer)?;
        
        // 解析加速度数据
        let accel_x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let accel_y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let accel_z = i16::from_be_bytes([buffer[4], buffer[5]]);
        
        // 解析温度数据
        let temp_raw = i16::from_be_bytes([buffer[6], buffer[7]]);
        
        // 解析陀螺仪数据
        let gyro_x = i16::from_be_bytes([buffer[8], buffer[9]]);
        let gyro_y = i16::from_be_bytes([buffer[10], buffer[11]]);
        let gyro_z = i16::from_be_bytes([buffer[12], buffer[13]]);
        
        // 转换为物理单位
        let accel_scale_factor = self.accel_scale.scale_factor();
        let gyro_scale_factor = self.gyro_scale.scale_factor();
        
        Ok(Mpu6050Reading {
            accel: Vector3 {
                x: (accel_x as f32) * accel_scale_factor,
                y: (accel_y as f32) * accel_scale_factor,
                z: (accel_z as f32) * accel_scale_factor,
            },
            gyro: Vector3 {
                x: (gyro_x as f32) * gyro_scale_factor,
                y: (gyro_y as f32) * gyro_scale_factor,
                z: (gyro_z as f32) * gyro_scale_factor,
            },
            temperature: (temp_raw as f32) / 340.0 + 36.53,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub enum AccelScale {
    G2 = 0x00,   // ±2g
    G4 = 0x08,   // ±4g
    G8 = 0x10,   // ±8g
    G16 = 0x18,  // ±16g
}

impl AccelScale {
    pub fn scale_factor(&self) -> f32 {
        match self {
            AccelScale::G2 => 2.0 / 32768.0 * 9.81,
            AccelScale::G4 => 4.0 / 32768.0 * 9.81,
            AccelScale::G8 => 8.0 / 32768.0 * 9.81,
            AccelScale::G16 => 16.0 / 32768.0 * 9.81,
        }
    }
}
```

### SPI 传感器

#### 高精度ADC
```rust
// 24位ADC (ADS1256)
pub struct Ads1256Sensor<SPI, CS> {
    spi: SPI,
    cs: CS,
    gain: PgaGain,
    data_rate: DataRate,
}

#[derive(Debug)]
pub struct Ads1256Reading {
    pub channel: u8,
    pub raw_value: i32,
    pub voltage: f32,
    pub timestamp: u64,
}

impl<SPI, CS> Ads1256Sensor<SPI, CS>
where
    SPI: embedded_hal::spi::SpiDevice,
    CS: embedded_hal::digital::OutputPin,
{
    pub fn init(&mut self) -> Result<(), Ads1256Error> {
        // 等待设备就绪
        self.wait_for_drdy()?;
        
        // 发送复位命令
        self.send_command(Command::Reset)?;
        delay_ms(1);
        
        // 配置寄存器
        self.write_register(Register::Status, 0x06)?; // 启用缓冲器
        self.write_register(Register::Mux, 0x01)?;    // 选择输入通道
        self.write_register(Register::Adcon, self.gain as u8)?; // 设置增益
        self.write_register(Register::Drate, self.data_rate as u8)?; // 设置数据率
        
        // 执行自校准
        self.send_command(Command::SelfCal)?;
        self.wait_for_drdy()?;
        
        Ok(())
    }
    
    pub fn read_channel(&mut self, channel: u8) -> Result<Ads1256Reading, Ads1256Error> {
        // 选择输入通道
        let mux_value = (channel << 4) | 0x08; // 单端输入，AINCOM为负输入
        self.write_register(Register::Mux, mux_value)?;
        
        // 同步转换
        self.send_command(Command::Sync)?;
        self.send_command(Command::Wakeup)?;
        
        // 等待转换完成
        self.wait_for_drdy()?;
        
        // 读取数据
        let raw_value = self.read_data()?;
        
        // 转换为电压
        let voltage = self.raw_to_voltage(raw_value);
        
        Ok(Ads1256Reading {
            channel,
            raw_value,
            voltage,
            timestamp: get_timestamp(),
        })
    }
    
    fn raw_to_voltage(&self, raw: i32) -> f32 {
        let vref = 2.5; // 参考电压
        let gain = self.gain.value();
        let full_scale = 0x7FFFFF as f32; // 23位有符号数的最大值
        
        (raw as f32) * vref / (gain * full_scale)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum PgaGain {
    Gain1 = 0x00,
    Gain2 = 0x01,
    Gain4 = 0x02,
    Gain8 = 0x03,
    Gain16 = 0x04,
    Gain32 = 0x05,
    Gain64 = 0x06,
}

impl PgaGain {
    pub fn value(&self) -> f32 {
        match self {
            PgaGain::Gain1 => 1.0,
            PgaGain::Gain2 => 2.0,
            PgaGain::Gain4 => 4.0,
            PgaGain::Gain8 => 8.0,
            PgaGain::Gain16 => 16.0,
            PgaGain::Gain32 => 32.0,
            PgaGain::Gain64 => 64.0,
        }
    }
}
```

#### 惯性测量单元
```rust
// 高性能IMU (BMI088)
pub struct Bmi088Sensor<SPI, CS_ACCEL, CS_GYRO> {
    spi: SPI,
    cs_accel: CS_ACCEL,
    cs_gyro: CS_GYRO,
    accel_config: AccelConfig,
    gyro_config: GyroConfig,
}

#[derive(Debug)]
pub struct Bmi088Reading {
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub accel_timestamp: u64,
    pub gyro_timestamp: u64,
}

impl<SPI, CS_ACCEL, CS_GYRO> Bmi088Sensor<SPI, CS_ACCEL, CS_GYRO>
where
    SPI: embedded_hal::spi::SpiDevice,
    CS_ACCEL: embedded_hal::digital::OutputPin,
    CS_GYRO: embedded_hal::digital::OutputPin,
{
    pub fn init(&mut self) -> Result<(), Bmi088Error> {
        // 初始化加速度计
        self.init_accelerometer()?;
        
        // 初始化陀螺仪
        self.init_gyroscope()?;
        
        Ok(())
    }
    
    fn init_accelerometer(&mut self) -> Result<(), Bmi088Error> {
        // 软复位
        self.write_accel_register(0x7E, 0xB6)?;
        delay_ms(1);
        
        // 配置量程和带宽
        self.write_accel_register(0x41, self.accel_config.range as u8)?;
        self.write_accel_register(0x40, self.accel_config.bandwidth as u8)?;
        
        // 启用加速度计
        self.write_accel_register(0x7D, 0x04)?;
        
        Ok(())
    }
    
    fn init_gyroscope(&mut self) -> Result<(), Bmi088Error> {
        // 软复位
        self.write_gyro_register(0x14, 0xB6)?;
        delay_ms(30);
        
        // 配置量程和带宽
        self.write_gyro_register(0x0F, self.gyro_config.range as u8)?;
        self.write_gyro_register(0x10, self.gyro_config.bandwidth as u8)?;
        
        Ok(())
    }
    
    pub fn read_all(&mut self) -> Result<Bmi088Reading, Bmi088Error> {
        // 读取加速度数据
        let mut accel_data = [0u8; 6];
        self.read_accel_registers(0x12, &mut accel_data)?;
        let accel_timestamp = get_timestamp();
        
        // 读取陀螺仪数据
        let mut gyro_data = [0u8; 6];
        self.read_gyro_registers(0x02, &mut gyro_data)?;
        let gyro_timestamp = get_timestamp();
        
        // 解析加速度数据
        let accel_x = i16::from_le_bytes([accel_data[0], accel_data[1]]);
        let accel_y = i16::from_le_bytes([accel_data[2], accel_data[3]]);
        let accel_z = i16::from_le_bytes([accel_data[4], accel_data[5]]);
        
        // 解析陀螺仪数据
        let gyro_x = i16::from_le_bytes([gyro_data[0], gyro_data[1]]);
        let gyro_y = i16::from_le_bytes([gyro_data[2], gyro_data[3]]);
        let gyro_z = i16::from_le_bytes([gyro_data[4], gyro_data[5]]);
        
        // 转换为物理单位
        let accel_scale = self.accel_config.scale_factor();
        let gyro_scale = self.gyro_config.scale_factor();
        
        Ok(Bmi088Reading {
            accel: Vector3 {
                x: (accel_x as f32) * accel_scale,
                y: (accel_y as f32) * accel_scale,
                z: (accel_z as f32) * accel_scale,
            },
            gyro: Vector3 {
                x: (gyro_x as f32) * gyro_scale,
                y: (gyro_y as f32) * gyro_scale,
                z: (gyro_z as f32) * gyro_scale,
            },
            accel_timestamp,
            gyro_timestamp,
        })
    }
}
```

## 传感器管理系统

### 传感器注册表
```rust
pub struct SensorRegistry {
    pub sensors: HashMap<SensorId, Box<dyn SensorTrait>>,
    pub configurations: HashMap<SensorId, SensorConfig>,
    pub calibrations: HashMap<SensorId, CalibrationData>,
}

pub trait SensorTrait {
    fn sensor_type(&self) -> SensorType;
    fn read_raw(&mut self) -> Result<RawReading, SensorError>;
    fn configure(&mut self, config: &SensorConfig) -> Result<(), SensorError>;
    fn calibrate(&mut self, data: &CalibrationData) -> Result<(), SensorError>;
    fn self_test(&mut self) -> Result<SelfTestResult, SensorError>;
}

impl SensorRegistry {
    pub fn register_sensor(&mut self, id: SensorId, sensor: Box<dyn SensorTrait>) {
        self.sensors.insert(id, sensor);
    }
    
    pub fn read_sensor(&mut self, id: &SensorId) -> Result<ProcessedReading, SensorError> {
        let sensor = self.sensors.get_mut(id)
            .ok_or(SensorError::SensorNotFound)?;
        
        let raw_reading = sensor.read_raw()?;
        let calibration = self.calibrations.get(id);
        
        let processed_reading = self.process_reading(raw_reading, calibration)?;
        
        Ok(processed_reading)
    }
    
    pub fn read_all_sensors(&mut self) -> HashMap<SensorId, Result<ProcessedReading, SensorError>> {
        let mut results = HashMap::new();
        
        for id in self.sensors.keys().cloned().collect::<Vec<_>>() {
            let result = self.read_sensor(&id);
            results.insert(id, result);
        }
        
        results
    }
}
```

### 数据融合系统
```rust
pub struct SensorFusion {
    pub imu_filter: ComplementaryFilter,
    pub position_estimator: KalmanFilter,
    pub environmental_smoother: MovingAverageFilter,
}

impl SensorFusion {
    pub fn fuse_imu_data(&mut self, accel: Vector3<f32>, gyro: Vector3<f32>, dt: f32) -> Orientation {
        self.imu_filter.update(accel, gyro, dt)
    }
    
    pub fn estimate_position(&mut self, accel: Vector3<f32>, dt: f32) -> Position {
        self.position_estimator.predict(dt);
        self.position_estimator.update(accel);
        self.position_estimator.get_state()
    }
    
    pub fn smooth_environmental_data(&mut self, reading: f32) -> f32 {
        self.environmental_smoother.update(reading)
    }
}

// 互补滤波器
pub struct ComplementaryFilter {
    pub alpha: f32,
    pub orientation: Quaternion,
}

impl ComplementaryFilter {
    pub fn update(&mut self, accel: Vector3<f32>, gyro: Vector3<f32>, dt: f32) -> Orientation {
        // 从加速度计计算倾斜角
        let accel_orientation = self.accel_to_orientation(accel);
        
        // 从陀螺仪积分得到角度变化
        let gyro_delta = self.integrate_gyro(gyro, dt);
        
        // 互补滤波融合
        self.orientation = self.orientation.multiply(&gyro_delta);
        self.orientation = self.orientation.slerp(&accel_orientation, self.alpha);
        
        self.orientation.to_euler()
    }
}

// 卡尔曼滤波器
pub struct KalmanFilter {
    pub state: Vector3<f32>,      // 位置
    pub velocity: Vector3<f32>,   // 速度
    pub covariance: Matrix3x3,    // 协方差矩阵
    pub process_noise: f32,       // 过程噪声
    pub measurement_noise: f32,   // 测量噪声
}

impl KalmanFilter {
    pub fn predict(&mut self, dt: f32) {
        // 状态预测
        self.state = self.state + self.velocity * dt + 
                    self.acceleration * (dt * dt * 0.5);
        self.velocity = self.velocity + self.acceleration * dt;
        
        // 协方差预测
        self.covariance = self.covariance + self.process_noise;
    }
    
    pub fn update(&mut self, measurement: Vector3<f32>) {
        // 卡尔曼增益
        let gain = self.covariance / (self.covariance + self.measurement_noise);
        
        // 状态更新
        let innovation = measurement - self.state;
        self.state = self.state + gain * innovation;
        
        // 协方差更新
        self.covariance = (1.0 - gain) * self.covariance;
    }
}
```

## 校准系统

### 自动校准
```rust
pub struct AutoCalibration {
    pub calibration_procedures: HashMap<SensorType, CalibrationProcedure>,
    pub calibration_data: HashMap<SensorId, CalibrationData>,
}

pub trait CalibrationProcedure {
    fn start_calibration(&mut self) -> Result<(), CalibrationError>;
    fn collect_sample(&mut self, reading: RawReading) -> Result<(), CalibrationError>;
    fn finish_calibration(&mut self) -> Result<CalibrationData, CalibrationError>;
    fn is_calibration_complete(&self) -> bool;
}

// 加速度计校准
pub struct AccelerometerCalibration {
    pub samples: Vec<Vector3<f32>>,
    pub required_orientations: Vec<Vector3<f32>>,
    pub current_orientation: usize,
    pub samples_per_orientation: usize,
}

impl CalibrationProcedure for AccelerometerCalibration {
    fn start_calibration(&mut self) -> Result<(), CalibrationError> {
        self.samples.clear();
        self.current_orientation = 0;
        
        // 定义6个标准方向 (+X, -X, +Y, -Y, +Z, -Z)
        self.required_orientations = vec![
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, -1.0),
        ];
        
        Ok(())
    }
    
    fn collect_sample(&mut self, reading: RawReading) -> Result<(), CalibrationError> {
        if let RawReading::Accelerometer(accel) = reading {
            self.samples.push(accel);
            
            if self.samples.len() >= self.samples_per_orientation {
                self.current_orientation += 1;
                
                if self.current_orientation < self.required_orientations.len() {
                    // 提示用户调整到下一个方向
                    self.prompt_next_orientation();
                }
            }
        }
        
        Ok(())
    }
    
    fn finish_calibration(&mut self) -> Result<CalibrationData, CalibrationError> {
        if !self.is_calibration_complete() {
            return Err(CalibrationError::IncompleteCalibration);
        }
        
        // 计算偏移和缩放因子
        let (offset, scale) = self.calculate_calibration_parameters();
        
        Ok(CalibrationData::Accelerometer {
            offset,
            scale_matrix: scale,
            temperature_compensation: None,
        })
    }
    
    fn is_calibration_complete(&self) -> bool {
        self.current_orientation >= self.required_orientations.len()
    }
}

// 陀螺仪校准
pub struct GyroscopeCalibration {
    pub samples: Vec<Vector3<f32>>,
    pub sample_count: usize,
    pub required_samples: usize,
    pub stability_threshold: f32,
}

impl CalibrationProcedure for GyroscopeCalibration {
    fn start_calibration(&mut self) -> Result<(), CalibrationError> {
        self.samples.clear();
        self.sample_count = 0;
        
        // 提示用户保持设备静止
        self.prompt_keep_still();
        
        Ok(())
    }
    
    fn collect_sample(&mut self, reading: RawReading) -> Result<(), CalibrationError> {
        if let RawReading::Gyroscope(gyro) = reading {
            // 检查稳定性
            if self.is_stable(&gyro) {
                self.samples.push(gyro);
                self.sample_count += 1;
            } else {
                // 如果不稳定，重新开始采样
                self.samples.clear();
                self.sample_count = 0;
            }
        }
        
        Ok(())
    }
    
    fn finish_calibration(&mut self) -> Result<CalibrationData, CalibrationError> {
        if self.samples.len() < self.required_samples {
            return Err(CalibrationError::InsufficientSamples);
        }
        
        // 计算零偏
        let bias = self.calculate_bias();
        
        Ok(CalibrationData::Gyroscope {
            bias,
            scale_factor: Vector3::new(1.0, 1.0, 1.0), // 假设缩放因子为1
            temperature_compensation: None,
        })
    }
    
    fn is_calibration_complete(&self) -> bool {
        self.sample_count >= self.required_samples
    }
}
```

### 温度补偿
```rust
pub struct TemperatureCompensation {
    pub temperature_coefficients: HashMap<SensorId, TemperatureCoefficients>,
    pub reference_temperature: f32,
}

#[derive(Debug, Clone)]
pub struct TemperatureCoefficients {
    pub offset_tc: Vector3<f32>,    // 偏移温度系数
    pub scale_tc: Vector3<f32>,     // 缩放温度系数
    pub nonlinear_tc: Vector3<f32>, // 非线性温度系数
}

impl TemperatureCompensation {
    pub fn compensate_reading(&self, 
                            sensor_id: &SensorId, 
                            reading: Vector3<f32>, 
                            temperature: f32) -> Vector3<f32> {
        if let Some(coeffs) = self.temperature_coefficients.get(sensor_id) {
            let dt = temperature - self.reference_temperature;
            
            // 线性补偿
            let offset_compensation = coeffs.offset_tc * dt;
            let scale_compensation = Vector3::new(1.0, 1.0, 1.0) + coeffs.scale_tc * dt;
            
            // 非线性补偿
            let nonlinear_compensation = coeffs.nonlinear_tc * (dt * dt);
            
            // 应用补偿
            let compensated = (reading - offset_compensation - nonlinear_compensation);
            Vector3::new(
                compensated.x * scale_compensation.x,
                compensated.y * scale_compensation.y,
                compensated.z * scale_compensation.z,
            )
        } else {
            reading // 无补偿数据时返回原始读数
        }
    }
    
    pub fn calibrate_temperature_compensation(&mut self, 
                                            sensor_id: SensorId,
                                            calibration_data: &[TemperatureCalibrationPoint]) -> Result<(), CalibrationError> {
        if calibration_data.len() < 3 {
            return Err(CalibrationError::InsufficientTemperaturePoints);
        }
        
        // 使用最小二乘法拟合温度系数
        let coefficients = self.fit_temperature_coefficients(calibration_data)?;
        
        self.temperature_coefficients.insert(sensor_id, coefficients);
        
        Ok(())
    }
}

#[derive(Debug)]
pub struct TemperatureCalibrationPoint {
    pub temperature: f32,
    pub sensor_reading: Vector3<f32>,
    pub reference_value: Vector3<f32>,
}
```

## 数据处理管道

### 实时数据处理
```rust
pub struct SensorDataPipeline {
    pub input_buffer: RingBuffer<SensorReading>,
    pub filters: Vec<Box<dyn DataFilter>>,
    pub processors: Vec<Box<dyn DataProcessor>>,
    pub output_buffer: RingBuffer<ProcessedData>,
}

pub trait DataFilter {
    fn filter(&mut self, data: &SensorReading) -> SensorReading;
    fn reset(&mut self);
}

pub trait DataProcessor {
    fn process(&mut self, data: &[SensorReading]) -> ProcessedData;
}

// 低通滤波器
pub struct LowPassFilter {
    pub cutoff_frequency: f32,
    pub sample_rate: f32,
    pub alpha: f32,
    pub previous_output: Option<Vector3<f32>>,
}

impl DataFilter for LowPassFilter {
    fn filter(&mut self, data: &SensorReading) -> SensorReading {
        if let SensorReading::Vector(input) = data {
            let output = match self.previous_output {
                Some(prev) => prev * (1.0 - self.alpha) + input * self.alpha,
                None => *input,
            };
            
            self.previous_output = Some(output);
            SensorReading::Vector(output)
        } else {
            *data
        }
    }
    
    fn reset(&mut self) {
        self.previous_output = None;
    }
}

// 高通滤波器
pub struct HighPassFilter {
    pub cutoff_frequency: f32,
    pub sample_rate: f32,
    pub alpha: f32,
    pub previous_input: Option<Vector3<f32>>,
    pub previous_output: Option<Vector3<f32>>,
}

impl DataFilter for HighPassFilter {
    fn filter(&mut self, data: &SensorReading) -> SensorReading {
        if let SensorReading::Vector(input) = data {
            let output = match (self.previous_input, self.previous_output) {
                (Some(prev_in), Some(prev_out)) => {
                    prev_out * self.alpha + (input - prev_in) * self.alpha
                },
                _ => Vector3::new(0.0, 0.0, 0.0),
            };
            
            self.previous_input = Some(*input);
            self.previous_output = Some(output);
            SensorReading::Vector(output)
        } else {
            *data
        }
    }
    
    fn reset(&mut self) {
        self.previous_input = None;
        self.previous_output = None;
    }
}

// 异常值检测
pub struct OutlierDetector {
    pub threshold_sigma: f32,
    pub window_size: usize,
    pub history: RingBuffer<f32>,
}

impl DataFilter for OutlierDetector {
    fn filter(&mut self, data: &SensorReading) -> SensorReading {
        if let SensorReading::Scalar(value) = data {
            self.history.push(*value);
            
            if self.history.len() >= self.window_size {
                let mean = self.calculate_mean();
                let std_dev = self.calculate_std_dev(mean);
                
                if (value - mean).abs() > self.threshold_sigma * std_dev {
                    // 异常值，返回均值
                    SensorReading::Scalar(mean)
                } else {
                    *data
                }
            } else {
                *data
            }
        } else {
            *data
        }
    }
    
    fn reset(&mut self) {
        self.history.clear();
    }
}
```

### 数据存储和检索
```rust
pub struct SensorDataStorage {
    pub flash_storage: FlashStorage,
    pub ram_buffer: CircularBuffer<SensorReading>,
    pub compression: DataCompression,
}

impl SensorDataStorage {
    pub fn store_reading(&mut self, reading: &SensorReading) -> Result<(), StorageError> {
        // 先存储到RAM缓冲区
        self.ram_buffer.push(*reading);
        
        // 当缓冲区满时，压缩并写入Flash
        if self.ram_buffer.is_full() {
            let compressed_data = self.compression.compress(&self.ram_buffer.data())?;
            self.flash_storage.write_block(&compressed_data)?;
            self.ram_buffer.clear();
        }
        
        Ok(())
    }
    
    pub fn retrieve_data(&mut self, 
                        start_time: u64, 
                        end_time: u64) -> Result<Vec<SensorReading>, StorageError> {
        let mut results = Vec::new();
        
        // 从RAM缓冲区检索最新数据
        for reading in self.ram_buffer.iter() {
            if reading.timestamp >= start_time && reading.timestamp <= end_time {
                results.push(*reading);
            }
        }
        
        // 从Flash存储检索历史数据
        let flash_data = self.flash_storage.read_time_range(start_time, end_time)?;
        let decompressed = self.compression.decompress(&flash_data)?;
        results.extend(decompressed);
        
        // 按时间戳排序
        results.sort_by_key(|r| r.timestamp);
        
        Ok(results)
    }
}

// 数据压缩
pub struct DataCompression {
    pub algorithm: CompressionAlgorithm,
}

pub enum CompressionAlgorithm {
    None,
    RunLength,
    Delta,
    Huffman,
}

impl DataCompression {
    pub fn compress(&self, data: &[SensorReading]) -> Result<Vec<u8>, CompressionError> {
        match self.algorithm {
            CompressionAlgorithm::None => Ok(self.serialize(data)),
            CompressionAlgorithm::Delta => self.delta_compress(data),
            CompressionAlgorithm::RunLength => self.run_length_compress(data),
            CompressionAlgorithm::Huffman => self.huffman_compress(data),
        }
    }
    
    fn delta_compress(&self, data: &[SensorReading]) -> Result<Vec<u8>, CompressionError> {
        let mut compressed = Vec::new();
        let mut previous: Option<SensorReading> = None;
        
        for reading in data {
            match previous {
                None => {
                    // 第一个值直接存储
                    compressed.extend(self.serialize_single(reading));
                },
                Some(prev) => {
                    // 存储差值
                    let delta = self.calculate_delta(&prev, reading);
                    compressed.extend(self.serialize_delta(&delta));
                }
            }
            previous = Some(*reading);
        }
        
        Ok(compressed)
    }
}
```

## 性能优化

### 采样率优化
```rust
pub struct AdaptiveSampling {
    pub base_sample_rate: f32,
    pub max_sample_rate: f32,
    pub min_sample_rate: f32,
    pub activity_detector: ActivityDetector,
    pub current_rate: f32,
}

impl AdaptiveSampling {
    pub fn update_sample_rate(&mut self, sensor_data: &SensorReading) -> f32 {
        let activity_level = self.activity_detector.detect_activity(sensor_data);
        
        self.current_rate = match activity_level {
            ActivityLevel::Low => self.min_sample_rate,
            ActivityLevel::Medium => self.base_sample_rate,
            ActivityLevel::High => self.max_sample_rate,
        };
        
        self.current_rate
    }
}

pub struct ActivityDetector {
    pub threshold_low: f32,
    pub threshold_high: f32,
    pub window_size: usize,
    pub variance_buffer: RingBuffer<f32>,
}

impl ActivityDetector {
    pub fn detect_activity(&mut self, data: &SensorReading) -> ActivityLevel {
        if let SensorReading::Vector(vec) = data {
            let magnitude = (vec.x * vec.x + vec.y * vec.y + vec.z * vec.z).sqrt();
            self.variance_buffer.push(magnitude);
            
            if self.variance_buffer.len() >= self.window_size {
                let variance = self.calculate_variance();
                
                if variance < self.threshold_low {
                    ActivityLevel::Low
                } else if variance > self.threshold_high {
                    ActivityLevel::High
                } else {
                    ActivityLevel::Medium
                }
            } else {
                ActivityLevel::Medium
            }
        } else {
            ActivityLevel::Medium
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum ActivityLevel {
    Low,
    Medium,
    High,
}
```

### 功耗管理
```rust
pub struct SensorPowerManager {
    pub power_states: HashMap<SensorId, PowerState>,
    pub wake_conditions: HashMap<SensorId, WakeCondition>,
    pub sleep_timer: SleepTimer,
}

#[derive(Debug, Clone, Copy)]
pub enum PowerState {
    Active,
    LowPower,
    Sleep,
    DeepSleep,
}

pub enum WakeCondition {
    Timer(u32),           // 定时唤醒 (ms)
    Threshold(f32),       // 阈值触发
    External,             // 外部中断
    Motion,               // 运动检测
}

impl SensorPowerManager {
    pub fn manage_power(&mut self, sensor_id: &SensorId, activity: ActivityLevel) {
        let current_state = self.power_states.get(sensor_id).copied()
            .unwrap_or(PowerState::Active);
        
        let new_state = match (current_state, activity) {
            (PowerState::Active, ActivityLevel::Low) => {
                // 活动水平低，进入低功耗模式
                self.configure_low_power_mode(sensor_id);
                PowerState::LowPower
            },
            (PowerState::LowPower, ActivityLevel::Low) => {
                // 持续低活动，进入睡眠模式
                self.configure_sleep_mode(sensor_id);
                PowerState::Sleep
            },
            (PowerState::Sleep, ActivityLevel::High) => {
                // 检测到活动，唤醒传感器
                self.wake_up_sensor(sensor_id);
                PowerState::Active
            },
            _ => current_state,
        };
        
        self.power_states.insert(*sensor_id, new_state);
    }
    
    fn configure_low_power_mode(&mut self, sensor_id: &SensorId) {
        // 降低采样率
        // 关闭不必要的功能
        // 设置唤醒条件
    }
    
    fn configure_sleep_mode(&mut self, sensor_id: &SensorId) {
        // 进入最低功耗模式
        // 只保持必要的唤醒电路
    }
    
    fn wake_up_sensor(&mut self, sensor_id: &SensorId) {
        // 恢复正常工作模式
        // 重新初始化传感器
    }
}
```

## 结论

传感器集成是一个复杂的系统工程，需要考虑：

1. **硬件接口**: 正确的电气连接和信号完整性
2. **驱动开发**: 可靠的底层驱动程序
3. **数据处理**: 有效的滤波和融合算法
4. **校准系统**: 精确的校准和补偿机制
5. **性能优化**: 功耗和实时性的平衡

通过系统性的设计和实现，可以构建出高性能、低功耗的传感器系统。