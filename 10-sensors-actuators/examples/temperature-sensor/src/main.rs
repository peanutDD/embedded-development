#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
  delay::Delay,
  gpio::{Input, Output, Pin, PullUp, PushPull},
  i2c::I2c,
  prelude::*,
  serial::{Config, Serial},
  spi::Spi,
  stm32,
  timer::Timer,
};

use heapless::{String, Vec};
use nb::block;

// 温度传感器抽象接口
trait TemperatureSensor {
  type Error;

  fn read_temperature(&mut self) -> Result<f32, Self::Error>;
  fn get_sensor_id(&self) -> &'static str;
  fn is_available(&mut self) -> bool;
}

// DS18B20 温度传感器 (1-Wire)
struct DS18B20Sensor<PIN> {
  pin: PIN,
  delay: Delay,
}

#[derive(Debug)]
enum DS18B20Error {
  CommunicationError,
  CrcError,
  ConversionTimeout,
}

impl<PIN> DS18B20Sensor<PIN>
where
  PIN: embedded_hal::digital::v2::OutputPin + embedded_hal::digital::v2::InputPin,
{
  fn new(pin: PIN, delay: Delay) -> Self {
    Self { pin, delay }
  }

  fn reset(&mut self) -> Result<bool, DS18B20Error> {
    // 发送复位脉冲
    self
      .pin
      .set_low()
      .map_err(|_| DS18B20Error::CommunicationError)?;
    self.delay.delay_us(480u32);

    // 释放总线并等待存在脉冲
    self
      .pin
      .set_high()
      .map_err(|_| DS18B20Error::CommunicationError)?;
    self.delay.delay_us(70u32);

    // 检查存在脉冲
    let presence = self
      .pin
      .is_low()
      .map_err(|_| DS18B20Error::CommunicationError)?;
    self.delay.delay_us(410u32);

    Ok(presence)
  }

  fn write_bit(&mut self, bit: bool) -> Result<(), DS18B20Error> {
    self
      .pin
      .set_low()
      .map_err(|_| DS18B20Error::CommunicationError)?;

    if bit {
      self.delay.delay_us(6u32);
      self
        .pin
        .set_high()
        .map_err(|_| DS18B20Error::CommunicationError)?;
      self.delay.delay_us(64u32);
    } else {
      self.delay.delay_us(60u32);
      self
        .pin
        .set_high()
        .map_err(|_| DS18B20Error::CommunicationError)?;
      self.delay.delay_us(10u32);
    }

    Ok(())
  }

  fn read_bit(&mut self) -> Result<bool, DS18B20Error> {
    self
      .pin
      .set_low()
      .map_err(|_| DS18B20Error::CommunicationError)?;
    self.delay.delay_us(6u32);

    self
      .pin
      .set_high()
      .map_err(|_| DS18B20Error::CommunicationError)?;
    self.delay.delay_us(9u32);

    let bit = self
      .pin
      .is_high()
      .map_err(|_| DS18B20Error::CommunicationError)?;
    self.delay.delay_us(55u32);

    Ok(bit)
  }

  fn write_byte(&mut self, byte: u8) -> Result<(), DS18B20Error> {
    for i in 0..8 {
      self.write_bit((byte >> i) & 1 == 1)?;
    }
    Ok(())
  }

  fn read_byte(&mut self) -> Result<u8, DS18B20Error> {
    let mut byte = 0u8;
    for i in 0..8 {
      if self.read_bit()? {
        byte |= 1 << i;
      }
    }
    Ok(byte)
  }

  fn start_conversion(&mut self) -> Result<(), DS18B20Error> {
    if !self.reset()? {
      return Err(DS18B20Error::CommunicationError);
    }

    // 跳过ROM命令 (单个设备)
    self.write_byte(0xCC)?;

    // 启动温度转换
    self.write_byte(0x44)?;

    Ok(())
  }

  fn read_scratchpad(&mut self) -> Result<[u8; 9], DS18B20Error> {
    if !self.reset()? {
      return Err(DS18B20Error::CommunicationError);
    }

    // 跳过ROM命令
    self.write_byte(0xCC)?;

    // 读取暂存器
    self.write_byte(0xBE)?;

    let mut scratchpad = [0u8; 9];
    for i in 0..9 {
      scratchpad[i] = self.read_byte()?;
    }

    // 验证CRC
    if !self.verify_crc(&scratchpad) {
      return Err(DS18B20Error::CrcError);
    }

    Ok(scratchpad)
  }

  fn verify_crc(&self, data: &[u8; 9]) -> bool {
    let mut crc = 0u8;

    for &byte in &data[0..8] {
      crc ^= byte;
      for _ in 0..8 {
        if crc & 0x01 != 0 {
          crc = (crc >> 1) ^ 0x8C;
        } else {
          crc >>= 1;
        }
      }
    }

    crc == data[8]
  }
}

impl<PIN> TemperatureSensor for DS18B20Sensor<PIN>
where
  PIN: embedded_hal::digital::v2::OutputPin + embedded_hal::digital::v2::InputPin,
{
  type Error = DS18B20Error;

  fn read_temperature(&mut self) -> Result<f32, Self::Error> {
    // 启动转换
    self.start_conversion()?;

    // 等待转换完成 (最多750ms)
    self.delay.delay_ms(750u32);

    // 读取结果
    let scratchpad = self.read_scratchpad()?;

    // 解析温度数据
    let temp_raw = ((scratchpad[1] as u16) << 8) | (scratchpad[0] as u16);
    let temperature = (temp_raw as i16) as f32 / 16.0;

    Ok(temperature)
  }

  fn get_sensor_id(&self) -> &'static str {
    "DS18B20"
  }

  fn is_available(&mut self) -> bool {
    self.reset().unwrap_or(false)
  }
}

// BMP280 温度和气压传感器 (I2C)
struct BMP280Sensor<I2C> {
  i2c: I2C,
  address: u8,
  calibration: BMP280Calibration,
}

#[derive(Default)]
struct BMP280Calibration {
  dig_t1: u16,
  dig_t2: i16,
  dig_t3: i16,
  dig_p1: u16,
  dig_p2: i16,
  dig_p3: i16,
  dig_p4: i16,
  dig_p5: i16,
  dig_p6: i16,
  dig_p7: i16,
  dig_p8: i16,
  dig_p9: i16,
}

#[derive(Debug)]
enum BMP280Error {
  I2cError,
  InvalidChipId,
  CalibrationError,
}

impl<I2C> BMP280Sensor<I2C>
where
  I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
  const CHIP_ID: u8 = 0x58;
  const DEFAULT_ADDRESS: u8 = 0x76;

  fn new(i2c: I2C) -> Self {
    Self {
      i2c,
      address: Self::DEFAULT_ADDRESS,
      calibration: BMP280Calibration::default(),
    }
  }

  fn initialize(&mut self) -> Result<(), BMP280Error> {
    // 验证芯片ID
    let chip_id = self.read_register(0xD0)?;
    if chip_id != Self::CHIP_ID {
      return Err(BMP280Error::InvalidChipId);
    }

    // 读取校准数据
    self.read_calibration_data()?;

    // 配置传感器
    self.write_register(0xF4, 0x27)?; // 温度和压力过采样
    self.write_register(0xF5, 0xA0)?; // 配置寄存器

    Ok(())
  }

  fn read_register(&mut self, reg: u8) -> Result<u8, BMP280Error> {
    let mut buffer = [0u8; 1];
    self
      .i2c
      .write_read(self.address, &[reg], &mut buffer)
      .map_err(|_| BMP280Error::I2cError)?;
    Ok(buffer[0])
  }

  fn write_register(&mut self, reg: u8, value: u8) -> Result<(), BMP280Error> {
    self
      .i2c
      .write(self.address, &[reg, value])
      .map_err(|_| BMP280Error::I2cError)
  }

  fn read_calibration_data(&mut self) -> Result<(), BMP280Error> {
    let mut buffer = [0u8; 24];
    self
      .i2c
      .write_read(self.address, &[0x88], &mut buffer)
      .map_err(|_| BMP280Error::I2cError)?;

    self.calibration.dig_t1 = ((buffer[1] as u16) << 8) | (buffer[0] as u16);
    self.calibration.dig_t2 = ((buffer[3] as i16) << 8) | (buffer[2] as i16);
    self.calibration.dig_t3 = ((buffer[5] as i16) << 8) | (buffer[4] as i16);

    self.calibration.dig_p1 = ((buffer[7] as u16) << 8) | (buffer[6] as u16);
    self.calibration.dig_p2 = ((buffer[9] as i16) << 8) | (buffer[8] as i16);
    self.calibration.dig_p3 = ((buffer[11] as i16) << 8) | (buffer[10] as i16);
    self.calibration.dig_p4 = ((buffer[13] as i16) << 8) | (buffer[12] as i16);
    self.calibration.dig_p5 = ((buffer[15] as i16) << 8) | (buffer[14] as i16);
    self.calibration.dig_p6 = ((buffer[17] as i16) << 8) | (buffer[16] as i16);
    self.calibration.dig_p7 = ((buffer[19] as i16) << 8) | (buffer[18] as i16);
    self.calibration.dig_p8 = ((buffer[21] as i16) << 8) | (buffer[20] as i16);
    self.calibration.dig_p9 = ((buffer[23] as i16) << 8) | (buffer[22] as i16);

    Ok(())
  }

  fn read_raw_temperature(&mut self) -> Result<i32, BMP280Error> {
    let mut buffer = [0u8; 3];
    self
      .i2c
      .write_read(self.address, &[0xFA], &mut buffer)
      .map_err(|_| BMP280Error::I2cError)?;

    let raw = ((buffer[0] as i32) << 12) | ((buffer[1] as i32) << 4) | ((buffer[2] as i32) >> 4);
    Ok(raw)
  }

  fn compensate_temperature(&self, raw_temp: i32) -> (f32, i32) {
    let var1 = (((raw_temp >> 3) - ((self.calibration.dig_t1 as i32) << 1))
      * (self.calibration.dig_t2 as i32))
      >> 11;

    let var2 = (((((raw_temp >> 4) - (self.calibration.dig_t1 as i32))
      * ((raw_temp >> 4) - (self.calibration.dig_t1 as i32)))
      >> 12)
      * (self.calibration.dig_t3 as i32))
      >> 14;

    let t_fine = var1 + var2;
    let temperature = (t_fine * 5 + 128) >> 8;

    (temperature as f32 / 100.0, t_fine)
  }
}

impl<I2C> TemperatureSensor for BMP280Sensor<I2C>
where
  I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
  type Error = BMP280Error;

  fn read_temperature(&mut self) -> Result<f32, Self::Error> {
    let raw_temp = self.read_raw_temperature()?;
    let (temperature, _) = self.compensate_temperature(raw_temp);
    Ok(temperature)
  }

  fn get_sensor_id(&self) -> &'static str {
    "BMP280"
  }

  fn is_available(&mut self) -> bool {
    self
      .read_register(0xD0)
      .map(|id| id == Self::CHIP_ID)
      .unwrap_or(false)
  }
}

// 温度数据滤波器
struct TemperatureFilter {
  history: Vec<f32, 10>,
  alpha: f32, // 低通滤波器系数
}

impl TemperatureFilter {
  fn new(alpha: f32) -> Self {
    Self {
      history: Vec::new(),
      alpha: alpha.clamp(0.0, 1.0),
    }
  }

  fn update(&mut self, new_value: f32) -> f32 {
    if self.history.is_empty() {
      self.history.push(new_value).ok();
      return new_value;
    }

    // 低通滤波
    let last_value = self.history[self.history.len() - 1];
    let filtered = self.alpha * new_value + (1.0 - self.alpha) * last_value;

    // 更新历史记录
    if self.history.is_full() {
      self.history.remove(0);
    }
    self.history.push(filtered).ok();

    filtered
  }

  fn get_average(&self) -> f32 {
    if self.history.is_empty() {
      return 0.0;
    }

    let sum: f32 = self.history.iter().sum();
    sum / self.history.len() as f32
  }

  fn get_variance(&self) -> f32 {
    if self.history.len() < 2 {
      return 0.0;
    }

    let avg = self.get_average();
    let sum_sq_diff: f32 = self.history.iter().map(|&x| (x - avg) * (x - avg)).sum();

    sum_sq_diff / (self.history.len() - 1) as f32
  }
}

// 温度监控系统
struct TemperatureMonitor {
  sensors: Vec<Box<dyn TemperatureSensor<Error = DS18B20Error>>, 4>,
  filters: Vec<TemperatureFilter, 4>,
  thresholds: TemperatureThresholds,
  alarm_state: AlarmState,
}

struct TemperatureThresholds {
  min_temp: f32,
  max_temp: f32,
  warning_delta: f32,
}

#[derive(PartialEq)]
enum AlarmState {
  Normal,
  Warning,
  Critical,
}

impl TemperatureMonitor {
  fn new(thresholds: TemperatureThresholds) -> Self {
    Self {
      sensors: Vec::new(),
      filters: Vec::new(),
      thresholds,
      alarm_state: AlarmState::Normal,
    }
  }

  fn add_sensor(
    &mut self,
    sensor: Box<dyn TemperatureSensor<Error = DS18B20Error>>,
  ) -> Result<(), &'static str> {
    if self.sensors.is_full() {
      return Err("Too many sensors");
    }

    self
      .sensors
      .push(sensor)
      .map_err(|_| "Failed to add sensor")?;
    self
      .filters
      .push(TemperatureFilter::new(0.1))
      .map_err(|_| "Failed to add filter")?;

    Ok(())
  }

  fn read_all_temperatures(&mut self) -> Vec<Result<f32, DS18B20Error>, 4> {
    let mut results = Vec::new();

    for (i, sensor) in self.sensors.iter_mut().enumerate() {
      match sensor.read_temperature() {
        Ok(temp) => {
          // 应用滤波
          let filtered_temp = self.filters[i].update(temp);
          results.push(Ok(filtered_temp)).ok();
        }
        Err(e) => {
          results.push(Err(e)).ok();
        }
      }
    }

    results
  }

  fn check_alarms(&mut self, temperatures: &[Result<f32, DS18B20Error>]) -> AlarmState {
    let mut max_temp = f32::NEG_INFINITY;
    let mut min_temp = f32::INFINITY;
    let mut valid_readings = 0;

    for temp_result in temperatures {
      if let Ok(temp) = temp_result {
        max_temp = max_temp.max(*temp);
        min_temp = min_temp.min(*temp);
        valid_readings += 1;
      }
    }

    if valid_readings == 0 {
      return AlarmState::Critical;
    }

    // 检查温度范围
    if max_temp > self.thresholds.max_temp || min_temp < self.thresholds.min_temp {
      self.alarm_state = AlarmState::Critical;
    } else if max_temp > self.thresholds.max_temp - self.thresholds.warning_delta
      || min_temp < self.thresholds.min_temp + self.thresholds.warning_delta
    {
      self.alarm_state = AlarmState::Warning;
    } else {
      self.alarm_state = AlarmState::Normal;
    }

    self.alarm_state
  }

  fn get_statistics(&self) -> TemperatureStatistics {
    let mut stats = TemperatureStatistics::default();

    for filter in &self.filters {
      if !filter.history.is_empty() {
        let avg = filter.get_average();
        let variance = filter.get_variance();

        stats.count += 1;
        stats.sum += avg;
        stats.sum_squares += avg * avg;
        stats.max_variance = stats.max_variance.max(variance);
      }
    }

    if stats.count > 0 {
      stats.average = stats.sum / stats.count as f32;
      stats.std_deviation =
        ((stats.sum_squares / stats.count as f32) - (stats.average * stats.average)).sqrt();
    }

    stats
  }
}

#[derive(Default)]
struct TemperatureStatistics {
  count: u32,
  sum: f32,
  sum_squares: f32,
  average: f32,
  std_deviation: f32,
  max_variance: f32,
}

// 数据记录器
struct TemperatureLogger {
  buffer: Vec<TemperatureReading, 100>,
  current_index: usize,
}

struct TemperatureReading {
  timestamp: u32,
  sensor_id: u8,
  temperature: f32,
  quality: DataQuality,
}

#[derive(Clone, Copy)]
enum DataQuality {
  Good,
  Warning,
  Error,
}

impl TemperatureLogger {
  fn new() -> Self {
    Self {
      buffer: Vec::new(),
      current_index: 0,
    }
  }

  fn log_reading(&mut self, sensor_id: u8, temperature: f32, quality: DataQuality, timestamp: u32) {
    let reading = TemperatureReading {
      timestamp,
      sensor_id,
      temperature,
      quality,
    };

    if self.buffer.is_full() {
      // 覆盖最旧的记录
      self.buffer[self.current_index] = reading;
      self.current_index = (self.current_index + 1) % self.buffer.capacity();
    } else {
      self.buffer.push(reading).ok();
    }
  }

  fn get_recent_readings(&self, count: usize) -> Vec<&TemperatureReading, 10> {
    let mut recent = Vec::new();
    let actual_count = count.min(self.buffer.len()).min(10);

    for i in 0..actual_count {
      let index = if self.buffer.is_full() {
        (self.current_index + self.buffer.len() - actual_count + i) % self.buffer.len()
      } else {
        self.buffer.len() - actual_count + i
      };

      recent.push(&self.buffer[index]).ok();
    }

    recent
  }
}

#[entry]
fn main() -> ! {
  // 获取外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 状态LED
  let mut led = gpioc.pc13.into_push_pull_output();
  let mut error_led = gpioa.pa5.into_push_pull_output();

  // 配置串口
  let tx = gpioa.pa2.into_alternate_af7();
  let rx = gpioa.pa3.into_alternate_af7();
  let serial = Serial::usart2(
    dp.USART2,
    (tx, rx),
    Config::default().baudrate(115200.bps()),
    clocks,
  )
  .unwrap();
  let (mut tx, mut rx) = serial.split();

  // 配置I2C
  let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
  let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
  let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);

  // 配置延时
  let mut delay = Delay::new(cp.SYST, clocks);

  // 配置定时器
  let mut timer = Timer::tim2(dp.TIM2, 1.hz(), clocks);

  // 初始化传感器
  let ds18b20_pin = gpioa.pa0.into_open_drain_output();
  let mut ds18b20 = DS18B20Sensor::new(ds18b20_pin, delay);

  let mut bmp280 = BMP280Sensor::new(i2c);
  if let Err(_) = bmp280.initialize() {
    error_led.set_high().ok();
  }

  // 初始化温度监控系统
  let thresholds = TemperatureThresholds {
    min_temp: -10.0,
    max_temp: 50.0,
    warning_delta: 5.0,
  };

  let mut monitor = TemperatureMonitor::new(thresholds);
  let mut logger = TemperatureLogger::new();

  // 发送启动消息
  for byte in b"Temperature Monitoring System Started\r\n" {
    block!(tx.write(*byte)).ok();
  }

  let mut sample_count = 0u32;

  loop {
    // 等待定时器
    block!(timer.wait()).ok();

    // 读取DS18B20
    match ds18b20.read_temperature() {
      Ok(temp) => {
        logger.log_reading(0, temp, DataQuality::Good, sample_count);

        // 发送数据
        let mut buffer = [0u8; 64];
        if let Ok(len) = format_temperature_data(&mut buffer, "DS18B20", temp, sample_count) {
          for &byte in &buffer[..len] {
            block!(tx.write(byte)).ok();
          }
        }
      }
      Err(_) => {
        logger.log_reading(0, 0.0, DataQuality::Error, sample_count);
        error_led.set_high().ok();
      }
    }

    // 读取BMP280
    match bmp280.read_temperature() {
      Ok(temp) => {
        logger.log_reading(1, temp, DataQuality::Good, sample_count);

        // 发送数据
        let mut buffer = [0u8; 64];
        if let Ok(len) = format_temperature_data(&mut buffer, "BMP280", temp, sample_count) {
          for &byte in &buffer[..len] {
            block!(tx.write(byte)).ok();
          }
        }
      }
      Err(_) => {
        logger.log_reading(1, 0.0, DataQuality::Error, sample_count);
      }
    }

    // 每10次采样输出统计信息
    if sample_count % 10 == 0 {
      let recent_readings = logger.get_recent_readings(10);
      let mut buffer = [0u8; 128];
      if let Ok(len) = format_statistics(&mut buffer, &recent_readings, sample_count) {
        for &byte in &buffer[..len] {
          block!(tx.write(byte)).ok();
        }
      }
    }

    // 切换状态LED
    led.toggle().ok();
    error_led.set_low().ok();

    sample_count += 1;
  }
}

// 格式化温度数据
fn format_temperature_data(
  buffer: &mut [u8],
  sensor_name: &str,
  temperature: f32,
  timestamp: u32,
) -> Result<usize, ()> {
  use heapless::String;

  let mut output = String::<64>::new();

  // 格式化为: "SENSOR,TEMP,TIMESTAMP\r\n"
  use core::fmt::Write;
  write!(
    output,
    "{},{:.2},{}\r\n",
    sensor_name, temperature, timestamp
  )
  .map_err(|_| ())?;

  let bytes = output.as_bytes();
  if bytes.len() > buffer.len() {
    return Err(());
  }

  buffer[..bytes.len()].copy_from_slice(bytes);
  Ok(bytes.len())
}

// 格式化统计信息
fn format_statistics(
  buffer: &mut [u8],
  readings: &[&TemperatureReading],
  timestamp: u32,
) -> Result<usize, ()> {
  use heapless::String;

  if readings.is_empty() {
    return Ok(0);
  }

  // 计算平均温度
  let mut sum = 0.0f32;
  let mut count = 0u32;

  for reading in readings {
    if matches!(reading.quality, DataQuality::Good) {
      sum += reading.temperature;
      count += 1;
    }
  }

  if count == 0 {
    return Ok(0);
  }

  let average = sum / count as f32;

  let mut output = String::<128>::new();
  use core::fmt::Write;
  write!(
    output,
    "STATS,AVG={:.2},COUNT={},TIME={}\r\n",
    average, count, timestamp
  )
  .map_err(|_| ())?;

  let bytes = output.as_bytes();
  if bytes.len() > buffer.len() {
    return Err(());
  }

  buffer[..bytes.len()].copy_from_slice(bytes);
  Ok(bytes.len())
}
