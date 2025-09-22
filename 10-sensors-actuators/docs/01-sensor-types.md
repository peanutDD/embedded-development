# 传感器类型与特性

## 概述

传感器是嵌入式系统中获取外界信息的重要组件。本文档详细介绍各种传感器的类型、工作原理、接口特性和应用场景。

## 传感器分类

### 按物理量分类

#### 1. 温度传感器
**工作原理**: 基于材料的热敏特性
**常见类型**:
- **热敏电阻 (NTC/PTC)**: 电阻随温度变化
- **热电偶**: 基于塞贝克效应
- **数字温度传感器**: 集成ADC的智能传感器

**典型器件**:
```rust
// DS18B20 数字温度传感器
struct DS18B20 {
    one_wire_pin: OneWirePin,
    resolution: u8,        // 9-12位分辨率
    conversion_time: u16,  // 转换时间 (ms)
}

impl DS18B20 {
    fn read_temperature(&mut self) -> Result<f32, Error> {
        // 启动温度转换
        self.start_conversion()?;
        
        // 等待转换完成
        delay_ms(self.conversion_time);
        
        // 读取温度数据
        let raw_data = self.read_scratchpad()?;
        let temperature = self.convert_to_celsius(raw_data);
        
        Ok(temperature)
    }
}
```

#### 2. 湿度传感器
**工作原理**: 基于材料的吸湿特性
**测量方法**:
- **电容式**: 湿度影响电容值
- **电阻式**: 湿度影响电阻值
- **热导式**: 基于湿度对热导率的影响

**典型应用**:
```rust
// DHT22 温湿度传感器
struct DHT22 {
    data_pin: DigitalPin,
    last_reading: Option<(f32, f32)>, // (温度, 湿度)
}

impl DHT22 {
    fn read_sensor(&mut self) -> Result<(f32, f32), Error> {
        // 发送启动信号
        self.send_start_signal()?;
        
        // 读取40位数据
        let data = self.read_data_bits()?;
        
        // 校验数据
        if !self.verify_checksum(&data) {
            return Err(Error::ChecksumError);
        }
        
        // 解析温湿度
        let humidity = self.parse_humidity(&data);
        let temperature = self.parse_temperature(&data);
        
        self.last_reading = Some((temperature, humidity));
        Ok((temperature, humidity))
    }
}
```

#### 3. 压力传感器
**工作原理**: 基于压阻效应或电容变化
**类型**:
- **绝对压力**: 相对于真空的压力
- **表压**: 相对于大气压的压力
- **差压**: 两个压力之间的差值

**典型器件**:
```rust
// BMP280 气压传感器
struct BMP280 {
    i2c: I2C,
    address: u8,
    calibration: CalibrationData,
}

impl BMP280 {
    fn read_pressure(&mut self) -> Result<f32, Error> {
        // 读取原始压力数据
        let raw_pressure = self.read_raw_pressure()?;
        let raw_temperature = self.read_raw_temperature()?;
        
        // 温度补偿
        let t_fine = self.compensate_temperature(raw_temperature);
        
        // 压力补偿计算
        let pressure = self.compensate_pressure(raw_pressure, t_fine);
        
        Ok(pressure / 100.0) // 转换为hPa
    }
    
    fn compensate_pressure(&self, raw: i32, t_fine: i32) -> u32 {
        // 使用校准参数进行补偿计算
        let var1 = (t_fine as i64) - 128000;
        let var2 = var1 * var1 * (self.calibration.dig_p6 as i64);
        // ... 复杂的补偿算法
        pressure as u32
    }
}
```

#### 4. 光照传感器
**工作原理**: 基于光电效应
**类型**:
- **光敏电阻 (LDR)**: 电阻随光照变化
- **光电二极管**: 产生光电流
- **光电三极管**: 放大光电流

**应用示例**:
```rust
// BH1750 数字光照传感器
struct BH1750 {
    i2c: I2C,
    address: u8,
    mode: MeasurementMode,
}

enum MeasurementMode {
    ContinuousHighRes,    // 1 lx 分辨率
    ContinuousHighRes2,   // 0.5 lx 分辨率
    ContinuousLowRes,     // 4 lx 分辨率
    OneTimeHighRes,       // 单次高分辨率
}

impl BH1750 {
    fn read_illuminance(&mut self) -> Result<f32, Error> {
        // 发送测量命令
        self.send_command(self.mode as u8)?;
        
        // 等待测量完成
        match self.mode {
            MeasurementMode::ContinuousHighRes => delay_ms(120),
            MeasurementMode::ContinuousLowRes => delay_ms(16),
            _ => delay_ms(120),
        }
        
        // 读取数据
        let raw_data = self.read_data()?;
        let illuminance = (raw_data as f32) / 1.2; // 转换为lux
        
        Ok(illuminance)
    }
}
```

### 按输出信号分类

#### 1. 模拟传感器
**特点**: 输出连续变化的电压或电流信号
**接口**: 需要ADC进行数字化

```rust
// 模拟传感器读取示例
struct AnalogSensor {
    adc_channel: u8,
    reference_voltage: f32,
    resolution: u16, // ADC位数
}

impl AnalogSensor {
    fn read_voltage(&mut self, adc: &mut ADC) -> Result<f32, Error> {
        let raw_value = adc.read_channel(self.adc_channel)?;
        let voltage = (raw_value as f32 / (1 << self.resolution) as f32) 
                     * self.reference_voltage;
        Ok(voltage)
    }
    
    fn convert_to_physical_value(&self, voltage: f32) -> f32 {
        // 根据传感器特性转换
        // 例如：温度 = (电压 - 偏移) / 灵敏度
        (voltage - 0.5) / 0.01 // 示例：10mV/°C，500mV偏移
    }
}
```

#### 2. 数字传感器
**特点**: 直接输出数字信号
**接口**: I2C, SPI, UART, 1-Wire等

```rust
// 数字传感器接口抽象
trait DigitalSensor {
    type Data;
    type Error;
    
    fn initialize(&mut self) -> Result<(), Self::Error>;
    fn read_data(&mut self) -> Result<Self::Data, Self::Error>;
    fn set_configuration(&mut self, config: &SensorConfig) -> Result<(), Self::Error>;
    fn get_device_id(&mut self) -> Result<u16, Self::Error>;
}

// I2C传感器实现
struct I2CSensor {
    i2c: I2C,
    address: u8,
    config: SensorConfig,
}

impl DigitalSensor for I2CSensor {
    type Data = SensorReading;
    type Error = I2CError;
    
    fn read_data(&mut self) -> Result<Self::Data, Self::Error> {
        let mut buffer = [0u8; 6];
        self.i2c.write_read(self.address, &[DATA_REG], &mut buffer)?;
        
        let reading = SensorReading {
            timestamp: get_timestamp(),
            value: self.parse_data(&buffer),
            status: self.check_status(&buffer),
        };
        
        Ok(reading)
    }
}
```

### 按应用领域分类

#### 1. 环境监测传感器
**用途**: 监测环境参数
**包括**: 温度、湿度、气压、空气质量、噪声等

```rust
// 环境监测站
struct EnvironmentalStation {
    temperature_sensor: DS18B20,
    humidity_sensor: DHT22,
    pressure_sensor: BMP280,
    air_quality_sensor: MQ135,
    noise_sensor: AnalogSensor,
}

impl EnvironmentalStation {
    fn read_all_parameters(&mut self) -> Result<EnvironmentalData, Error> {
        let temperature = self.temperature_sensor.read_temperature()?;
        let (temp2, humidity) = self.humidity_sensor.read_sensor()?;
        let pressure = self.pressure_sensor.read_pressure()?;
        let air_quality = self.air_quality_sensor.read_ppm()?;
        let noise_level = self.noise_sensor.read_db()?;
        
        Ok(EnvironmentalData {
            temperature: (temperature + temp2) / 2.0, // 平均值
            humidity,
            pressure,
            air_quality,
            noise_level,
            timestamp: get_timestamp(),
        })
    }
}
```

#### 2. 运动传感器
**用途**: 检测物体运动和姿态
**包括**: 加速度计、陀螺仪、磁力计、倾角传感器

```rust
// 9轴运动传感器 (3轴加速度 + 3轴陀螺仪 + 3轴磁力计)
struct IMU9DOF {
    accelerometer: Accelerometer,
    gyroscope: Gyroscope,
    magnetometer: Magnetometer,
    fusion_filter: MadgwickFilter,
}

impl IMU9DOF {
    fn read_motion_data(&mut self) -> Result<MotionData, Error> {
        let accel = self.accelerometer.read_acceleration()?;
        let gyro = self.gyroscope.read_angular_velocity()?;
        let mag = self.magnetometer.read_magnetic_field()?;
        
        // 传感器融合计算姿态
        let quaternion = self.fusion_filter.update(accel, gyro, mag);
        let euler_angles = quaternion.to_euler_angles();
        
        Ok(MotionData {
            acceleration: accel,
            angular_velocity: gyro,
            magnetic_field: mag,
            orientation: euler_angles,
            quaternion,
        })
    }
}
```

#### 3. 位置传感器
**用途**: 检测位置和距离
**包括**: 超声波、激光、霍尔传感器、编码器

```rust
// 超声波距离传感器
struct UltrasonicSensor {
    trigger_pin: OutputPin,
    echo_pin: InputPin,
    timer: Timer,
}

impl UltrasonicSensor {
    fn measure_distance(&mut self) -> Result<f32, Error> {
        // 发送触发脉冲
        self.trigger_pin.set_high();
        delay_us(10);
        self.trigger_pin.set_low();
        
        // 等待回波开始
        let start_time = self.wait_for_echo_start()?;
        
        // 等待回波结束
        let end_time = self.wait_for_echo_end()?;
        
        // 计算距离 (声速 = 343 m/s)
        let pulse_duration = end_time - start_time;
        let distance = (pulse_duration as f32 * 343.0) / (2.0 * 1_000_000.0);
        
        Ok(distance)
    }
    
    fn wait_for_echo_start(&mut self) -> Result<u32, Error> {
        let timeout = 30000; // 30ms超时
        let start = self.timer.get_counter();
        
        while self.echo_pin.is_low() {
            if self.timer.get_counter() - start > timeout {
                return Err(Error::Timeout);
            }
        }
        
        Ok(self.timer.get_counter())
    }
}
```

## 传感器接口协议

### I2C接口传感器
**特点**: 
- 两线制 (SDA, SCL)
- 多设备共享总线
- 地址寻址
- 适合短距离通信

```rust
// I2C传感器通用驱动框架
struct I2CSensorDriver<T> {
    i2c: I2C,
    address: u8,
    registers: T,
}

impl<T: RegisterMap> I2CSensorDriver<T> {
    fn read_register(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buffer)?;
        Ok(buffer[0])
    }
    
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        self.i2c.write(self.address, &[reg, value])?;
        Ok(())
    }
    
    fn read_multiple(&mut self, start_reg: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.i2c.write_read(self.address, &[start_reg], buffer)?;
        Ok(())
    }
}
```

### SPI接口传感器
**特点**:
- 四线制 (MOSI, MISO, SCK, CS)
- 全双工通信
- 高速传输
- 点对点连接

```rust
// SPI传感器驱动框架
struct SPISensorDriver {
    spi: SPI,
    cs_pin: OutputPin,
    config: SPIConfig,
}

impl SPISensorDriver {
    fn read_register(&mut self, reg: u8) -> Result<u8, Error> {
        self.cs_pin.set_low();
        
        // 发送读命令 (最高位为1表示读)
        let cmd = reg | 0x80;
        self.spi.transfer(&mut [cmd])?;
        
        // 读取数据
        let mut data = [0u8; 1];
        self.spi.transfer(&mut data)?;
        
        self.cs_pin.set_high();
        Ok(data[0])
    }
    
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        self.cs_pin.set_low();
        
        // 发送写命令和数据
        let mut data = [reg & 0x7F, value]; // 最高位为0表示写
        self.spi.transfer(&mut data)?;
        
        self.cs_pin.set_high();
        Ok(())
    }
}
```

### UART接口传感器
**特点**:
- 异步串行通信
- 简单的两线制
- 适合长距离传输
- 常用于GPS、气体传感器等

```rust
// UART传感器驱动
struct UARTSensorDriver {
    uart: UART,
    buffer: [u8; 256],
    parser: MessageParser,
}

impl UARTSensorDriver {
    fn read_sensor_data(&mut self) -> Result<SensorData, Error> {
        // 读取数据到缓冲区
        let bytes_read = self.uart.read(&mut self.buffer)?;
        
        // 解析消息
        for &byte in &self.buffer[..bytes_read] {
            if let Some(message) = self.parser.parse_byte(byte) {
                return self.decode_message(message);
            }
        }
        
        Err(Error::NoData)
    }
    
    fn send_command(&mut self, cmd: &[u8]) -> Result<(), Error> {
        self.uart.write(cmd)?;
        Ok(())
    }
}
```

## 传感器数据处理

### 数据滤波
**目的**: 减少噪声，提高数据质量

```rust
// 移动平均滤波器
struct MovingAverageFilter {
    buffer: [f32; 16],
    index: usize,
    count: usize,
}

impl MovingAverageFilter {
    fn new() -> Self {
        Self {
            buffer: [0.0; 16],
            index: 0,
            count: 0,
        }
    }
    
    fn update(&mut self, value: f32) -> f32 {
        self.buffer[self.index] = value;
        self.index = (self.index + 1) % self.buffer.len();
        
        if self.count < self.buffer.len() {
            self.count += 1;
        }
        
        let sum: f32 = self.buffer[..self.count].iter().sum();
        sum / self.count as f32
    }
}

// 卡尔曼滤波器 (简化版)
struct KalmanFilter {
    x: f32,      // 状态估计
    p: f32,      // 估计误差协方差
    q: f32,      // 过程噪声协方差
    r: f32,      // 测量噪声协方差
}

impl KalmanFilter {
    fn new(q: f32, r: f32) -> Self {
        Self {
            x: 0.0,
            p: 1.0,
            q,
            r,
        }
    }
    
    fn update(&mut self, measurement: f32) -> f32 {
        // 预测步骤
        self.p += self.q;
        
        // 更新步骤
        let k = self.p / (self.p + self.r); // 卡尔曼增益
        self.x += k * (measurement - self.x);
        self.p *= 1.0 - k;
        
        self.x
    }
}
```

### 数据校准
**目的**: 消除系统误差，提高精度

```rust
// 传感器校准数据
struct CalibrationData {
    offset: f32,     // 零点偏移
    scale: f32,      // 比例因子
    linearity: f32,  // 线性度修正
}

impl CalibrationData {
    fn apply_calibration(&self, raw_value: f32) -> f32 {
        // 基本线性校准
        let calibrated = (raw_value - self.offset) * self.scale;
        
        // 非线性修正 (二次项)
        calibrated + self.linearity * calibrated * calibrated
    }
    
    fn load_from_flash(&mut self, flash: &mut Flash, address: u32) -> Result<(), Error> {
        let mut buffer = [0u8; 12]; // 3个f32值
        flash.read(address, &mut buffer)?;
        
        self.offset = f32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
        self.scale = f32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]);
        self.linearity = f32::from_le_bytes([buffer[8], buffer[9], buffer[10], buffer[11]]);
        
        Ok(())
    }
}
```

## 传感器选型指南

### 性能参数
1. **精度**: 测量值与真实值的接近程度
2. **分辨率**: 能够分辨的最小变化量
3. **量程**: 能够测量的范围
4. **响应时间**: 从输入变化到输出稳定的时间
5. **稳定性**: 长期使用的漂移特性

### 环境要求
1. **工作温度范围**
2. **湿度要求**
3. **抗震动能力**
4. **电磁兼容性**
5. **化学兼容性**

### 接口选择
1. **I2C**: 适合多传感器系统，中等速度
2. **SPI**: 适合高速数据传输
3. **UART**: 适合长距离传输
4. **模拟**: 适合简单应用，需要ADC

### 功耗考虑
1. **工作电流**
2. **待机电流**
3. **电源电压范围**
4. **低功耗模式支持**

## 总结

传感器是嵌入式系统感知外界的重要接口。选择合适的传感器需要综合考虑应用需求、性能参数、接口特性和成本因素。通过合理的驱动设计、数据处理和校准，可以充分发挥传感器的性能，构建可靠的感知系统。