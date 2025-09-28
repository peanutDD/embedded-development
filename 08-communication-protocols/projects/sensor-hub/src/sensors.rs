// 传感器模块 - 简化版本
use embedded_hal::blocking::i2c::{Write, WriteRead};
use heapless::Vec;

// 传感器I2C地址
const BMP280_ADDR: u8 = 0x76;
const MPU6050_ADDR: u8 = 0x68;

// BMP280寄存器地址
const BMP280_REG_ID: u8 = 0xD0;
const BMP280_REG_CTRL_MEAS: u8 = 0xF4;
const BMP280_REG_CONFIG: u8 = 0xF5;
const BMP280_REG_PRESS_MSB: u8 = 0xF7;
const BMP280_REG_TEMP_MSB: u8 = 0xFA;

// MPU6050寄存器地址
const MPU6050_REG_WHO_AM_I: u8 = 0x75;
const MPU6050_REG_PWR_MGMT_1: u8 = 0x6B;
const MPU6050_REG_ACCEL_CONFIG: u8 = 0x1C;
const MPU6050_REG_GYRO_CONFIG: u8 = 0x1B;
const MPU6050_REG_ACCEL_XOUT_H: u8 = 0x3B;

// 传感器数据结构
#[derive(Debug, Clone, Copy)]
pub struct SensorData {
  pub timestamp: u32,
  pub temperature: f32,
  pub pressure: f32,
  pub humidity: f32, // 预留字段
  pub accel_x: f32,
  pub accel_y: f32,
  pub accel_z: f32,
  pub gyro_x: f32,
  pub gyro_y: f32,
  pub gyro_z: f32,
  pub checksum: u32,
}

impl SensorData {
  pub fn new() -> Self {
    Self {
      timestamp: get_system_time(),
      temperature: 0.0,
      pressure: 0.0,
      humidity: 0.0,
      accel_x: 0.0,
      accel_y: 0.0,
      accel_z: 0.0,
      gyro_x: 0.0,
      gyro_y: 0.0,
      gyro_z: 0.0,
      checksum: 0,
    }
  }

  pub fn calculate_checksum(&mut self) {
    let mut sum = 0u32;
    sum = sum.wrapping_add(self.timestamp);
    sum = sum.wrapping_add(self.temperature.to_bits());
    sum = sum.wrapping_add(self.pressure.to_bits());
    sum = sum.wrapping_add(self.humidity.to_bits());
    sum = sum.wrapping_add(self.accel_x.to_bits());
    sum = sum.wrapping_add(self.accel_y.to_bits());
    sum = sum.wrapping_add(self.accel_z.to_bits());
    sum = sum.wrapping_add(self.gyro_x.to_bits());
    sum = sum.wrapping_add(self.gyro_y.to_bits());
    sum = sum.wrapping_add(self.gyro_z.to_bits());
    self.checksum = sum;
  }

  pub fn verify_checksum(&self) -> bool {
    let mut sum = 0u32;
    sum = sum.wrapping_add(self.timestamp);
    sum = sum.wrapping_add(self.temperature.to_bits());
    sum = sum.wrapping_add(self.pressure.to_bits());
    sum = sum.wrapping_add(self.humidity.to_bits());
    sum = sum.wrapping_add(self.accel_x.to_bits());
    sum = sum.wrapping_add(self.accel_y.to_bits());
    sum = sum.wrapping_add(self.accel_z.to_bits());
    sum = sum.wrapping_add(self.gyro_x.to_bits());
    sum = sum.wrapping_add(self.gyro_y.to_bits());
    sum = sum.wrapping_add(self.gyro_z.to_bits());
    sum == self.checksum
  }

  pub fn to_bytes(&self) -> [u8; 44] {
    let mut bytes = [0u8; 44];
    bytes[0..4].copy_from_slice(&self.timestamp.to_le_bytes());
    bytes[4..8].copy_from_slice(&self.temperature.to_le_bytes());
    bytes[8..12].copy_from_slice(&self.pressure.to_le_bytes());
    bytes[12..16].copy_from_slice(&self.humidity.to_le_bytes());
    bytes[16..20].copy_from_slice(&self.accel_x.to_le_bytes());
    bytes[20..24].copy_from_slice(&self.accel_y.to_le_bytes());
    bytes[24..28].copy_from_slice(&self.accel_z.to_le_bytes());
    bytes[28..32].copy_from_slice(&self.gyro_x.to_le_bytes());
    bytes[32..36].copy_from_slice(&self.gyro_y.to_le_bytes());
    bytes[36..40].copy_from_slice(&self.gyro_z.to_le_bytes());
    bytes[40..44].copy_from_slice(&self.checksum.to_le_bytes());
    bytes
  }

  pub fn from_bytes(bytes: &[u8; 44]) -> Self {
    let timestamp = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    let temperature = f32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
    let pressure = f32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
    let humidity = f32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]);
    let accel_x = f32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
    let accel_y = f32::from_le_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]);
    let accel_z = f32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
    let gyro_x = f32::from_le_bytes([bytes[28], bytes[29], bytes[30], bytes[31]]);
    let gyro_y = f32::from_le_bytes([bytes[32], bytes[33], bytes[34], bytes[35]]);
    let gyro_z = f32::from_le_bytes([bytes[36], bytes[37], bytes[38], bytes[39]]);
    let checksum = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]);

    Self {
      timestamp,
      temperature,
      pressure,
      humidity,
      accel_x,
      accel_y,
      accel_z,
      gyro_x,
      gyro_y,
      gyro_z,
      checksum,
    }
  }
}

impl Default for SensorData {
  fn default() -> Self {
    Self::new()
  }
}

// 传感器管理器 - 简化版本
pub struct SensorManager<I2C> {
  i2c: I2C,
  bmp280_initialized: bool,
  mpu6050_initialized: bool,
}

impl<I2C, E> SensorManager<I2C>
where
  I2C: WriteRead<Error = E> + Write<Error = E>,
{
  pub fn new(i2c: I2C) -> Self {
    Self {
      i2c,
      bmp280_initialized: false,
      mpu6050_initialized: false,
    }
  }

  pub fn initialize(&mut self) -> Result<(), E> {
    // 初始化BMP280
    self.bmp280_initialized = self.init_bmp280().is_ok();

    // 初始化MPU6050
    self.mpu6050_initialized = self.init_mpu6050().is_ok();

    if self.bmp280_initialized || self.mpu6050_initialized {
      Ok(())
    } else {
      Err(self.i2c.write(0x00, &[]).unwrap_err()) // 返回一个错误
    }
  }

  pub fn read_all_sensors(&mut self) -> Result<SensorData, E> {
    let mut data = SensorData::new();

    // 读取BMP280数据
    if self.bmp280_initialized {
      if let Ok((temp, press)) = self.read_bmp280() {
        data.temperature = temp;
        data.pressure = press;
      }
    }

    // 读取MPU6050数据
    if self.mpu6050_initialized {
      if let Ok((accel, gyro)) = self.read_mpu6050() {
        data.accel_x = accel[0];
        data.accel_y = accel[1];
        data.accel_z = accel[2];
        data.gyro_x = gyro[0];
        data.gyro_y = gyro[1];
        data.gyro_z = gyro[2];
      }
    }

    data.calculate_checksum();
    Ok(data)
  }

  fn init_bmp280(&mut self) -> Result<(), E> {
    // 读取芯片ID
    let mut id_buffer = [0u8; 1];
    self
      .i2c
      .write_read(BMP280_ADDR, &[BMP280_REG_ID], &mut id_buffer)?;

    if id_buffer[0] != 0x58 {
      // BMP280 ID
      return Err(self.i2c.write(0x00, &[]).unwrap_err());
    }

    // 配置传感器
    self.i2c.write(BMP280_ADDR, &[BMP280_REG_CTRL_MEAS, 0x27])?; // 正常模式
    self.i2c.write(BMP280_ADDR, &[BMP280_REG_CONFIG, 0xA0])?; // 配置

    Ok(())
  }

  fn read_bmp280(&mut self) -> Result<(f32, f32), E> {
    // 读取原始数据
    let mut data = [0u8; 6];
    self
      .i2c
      .write_read(BMP280_ADDR, &[BMP280_REG_PRESS_MSB], &mut data)?;

    // 解析压力数据
    let press_raw = ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);

    // 解析温度数据
    let temp_raw = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);

    // 简化的温度和压力计算
    let temperature = (temp_raw as f32) / 5120.0 - 40.0;
    let pressure = (press_raw as f32) / 256.0;

    Ok((temperature, pressure))
  }

  fn init_mpu6050(&mut self) -> Result<(), E> {
    // 读取设备ID
    let mut id_buffer = [0u8; 1];
    self
      .i2c
      .write_read(MPU6050_ADDR, &[MPU6050_REG_WHO_AM_I], &mut id_buffer)?;

    if id_buffer[0] != 0x68 {
      // MPU6050 ID
      return Err(self.i2c.write(0x00, &[]).unwrap_err());
    }

    // 唤醒设备
    self
      .i2c
      .write(MPU6050_ADDR, &[MPU6050_REG_PWR_MGMT_1, 0x00])?;

    // 配置陀螺仪 (±250°/s)
    self
      .i2c
      .write(MPU6050_ADDR, &[MPU6050_REG_GYRO_CONFIG, 0x00])?;

    // 配置加速度计 (±2g)
    self
      .i2c
      .write(MPU6050_ADDR, &[MPU6050_REG_ACCEL_CONFIG, 0x00])?;

    Ok(())
  }

  fn read_mpu6050(&mut self) -> Result<([f32; 3], [f32; 3]), E> {
    // 读取加速度计和陀螺仪数据 (14字节)
    let mut data = [0u8; 14];
    self
      .i2c
      .write_read(MPU6050_ADDR, &[MPU6050_REG_ACCEL_XOUT_H], &mut data)?;

    // 解析加速度数据
    let accel_x = i16::from_be_bytes([data[0], data[1]]) as f32 / 16384.0;
    let accel_y = i16::from_be_bytes([data[2], data[3]]) as f32 / 16384.0;
    let accel_z = i16::from_be_bytes([data[4], data[5]]) as f32 / 16384.0;

    // 跳过温度数据 (data[6], data[7])

    // 解析陀螺仪数据
    let gyro_x = i16::from_be_bytes([data[8], data[9]]) as f32 / 131.0;
    let gyro_y = i16::from_be_bytes([data[10], data[11]]) as f32 / 131.0;
    let gyro_z = i16::from_be_bytes([data[12], data[13]]) as f32 / 131.0;

    Ok(([accel_x, accel_y, accel_z], [gyro_x, gyro_y, gyro_z]))
  }

  pub fn is_initialized(&self) -> bool {
    self.bmp280_initialized || self.mpu6050_initialized
  }
}

// 获取系统时间
fn get_system_time() -> u32 {
  crate::get_system_time()
}
