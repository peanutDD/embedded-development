#![no_std]
#![no_main]

//! # 高级I2C EEPROM操作示例
//!
//! 演示更复杂的EEPROM操作：
//! - 多设备管理
//! - 数据结构存储
//! - 错误恢复
//! - 性能优化

use cortex_m_rt::entry;
use crc::{Crc, CRC_16_IBM_SDLC};
use heapless::{FnvIndexMap, String, Vec};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{
    gpiob::{PB6, PB7},
    gpioc::PC13,
    AlternateOD, Output, PushPull,
  },
  i2c::{I2c, Mode},
  pac,
  prelude::*,
  timer::{Event, Timer},
};

// EEPROM设备配置
const EEPROM_24C02_ADDR: u8 = 0x50; // 24C02 (256 bytes)
const EEPROM_24C08_ADDR: u8 = 0x51; // 24C08 (1024 bytes)
const EEPROM_24C32_ADDR: u8 = 0x52; // 24C32 (4096 bytes)

// 数据结构版本
const DATA_VERSION: u8 = 1;

// CRC计算器
const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

type I2cType = I2c<pac::I2C1, (PB6<AlternateOD<4>>, PB7<AlternateOD<4>>)>;

/// EEPROM设备信息
#[derive(Debug, Clone, Copy)]
pub struct EepromDevice {
  pub address: u8,
  pub size: usize,
  pub page_size: usize,
  pub write_time_ms: u32,
}

/// 数据记录结构
#[derive(Debug, Clone)]
pub struct DataRecord {
  pub id: u16,
  pub timestamp: u32,
  pub data: Vec<u8, 32>,
  pub checksum: u16,
}

/// EEPROM管理器
pub struct EepromManager {
  devices: FnvIndexMap<u8, EepromDevice, 4>,
  current_device: u8,
  error_count: u32,
  retry_count: u8,
}

impl EepromManager {
  pub fn new() -> Self {
    let mut devices = FnvIndexMap::new();

    // 添加支持的设备
    devices
      .insert(
        EEPROM_24C02_ADDR,
        EepromDevice {
          address: EEPROM_24C02_ADDR,
          size: 256,
          page_size: 8,
          write_time_ms: 5,
        },
      )
      .ok();

    devices
      .insert(
        EEPROM_24C08_ADDR,
        EepromDevice {
          address: EEPROM_24C08_ADDR,
          size: 1024,
          page_size: 16,
          write_time_ms: 5,
        },
      )
      .ok();

    devices
      .insert(
        EEPROM_24C32_ADDR,
        EepromDevice {
          address: EEPROM_24C32_ADDR,
          size: 4096,
          page_size: 32,
          write_time_ms: 10,
        },
      )
      .ok();

    Self {
      devices,
      current_device: EEPROM_24C02_ADDR,
      error_count: 0,
      retry_count: 3,
    }
  }

  /// 扫描可用设备
  pub fn scan_devices(&mut self, i2c: &mut I2cType) -> Vec<u8, 4> {
    let mut found_devices = Vec::new();

    for (&addr, _) in self.devices.iter() {
      if self.test_device(i2c, addr) {
        found_devices.push(addr).ok();
      }
    }

    found_devices
  }

  /// 测试设备是否存在
  fn test_device(&self, i2c: &mut I2cType, address: u8) -> bool {
    // 尝试读取第一个字节
    let mut buffer = [0u8; 1];
    match i2c.write_read(address, &[0x00, 0x00], &mut buffer) {
      Ok(_) => true,
      Err(_) => false,
    }
  }

  /// 写入数据记录
  pub fn write_record(
    &mut self,
    i2c: &mut I2cType,
    address: u16,
    record: &DataRecord,
  ) -> Result<(), EepromError> {
    let device = self
      .devices
      .get(&self.current_device)
      .ok_or(EepromError::DeviceNotFound)?;

    // 序列化记录
    let serialized = self.serialize_record(record)?;

    // 分页写入
    self.write_data_with_retry(i2c, device, address, &serialized)
  }

  /// 读取数据记录
  pub fn read_record(
    &mut self,
    i2c: &mut I2cType,
    address: u16,
  ) -> Result<DataRecord, EepromError> {
    let device = self
      .devices
      .get(&self.current_device)
      .ok_or(EepromError::DeviceNotFound)?;

    // 先读取记录头部获取长度
    let mut header = [0u8; 8]; // id(2) + timestamp(4) + length(1) + version(1)
    self.read_data_with_retry(i2c, device, address, &mut header)?;

    let data_length = header[6] as usize;
    if data_length > 32 {
      return Err(EepromError::InvalidData);
    }

    // 读取完整记录
    let total_length = 8 + data_length + 2; // header + data + checksum
    let mut buffer = Vec::<u8, 64>::new();
    buffer
      .resize(total_length, 0)
      .map_err(|_| EepromError::BufferTooSmall)?;

    self.read_data_with_retry(i2c, device, address, &mut buffer)?;

    // 反序列化记录
    self.deserialize_record(&buffer)
  }

  /// 序列化数据记录
  fn serialize_record(&self, record: &DataRecord) -> Result<Vec<u8, 64>, EepromError> {
    let mut buffer = Vec::new();

    // 写入记录头
    buffer
      .extend_from_slice(&record.id.to_le_bytes())
      .map_err(|_| EepromError::BufferTooSmall)?;
    buffer
      .extend_from_slice(&record.timestamp.to_le_bytes())
      .map_err(|_| EepromError::BufferTooSmall)?;
    buffer
      .push(record.data.len() as u8)
      .map_err(|_| EepromError::BufferTooSmall)?;
    buffer
      .push(DATA_VERSION)
      .map_err(|_| EepromError::BufferTooSmall)?;

    // 写入数据
    buffer
      .extend_from_slice(&record.data)
      .map_err(|_| EepromError::BufferTooSmall)?;

    // 计算并写入校验和
    let checksum = CRC16.checksum(&buffer);
    buffer
      .extend_from_slice(&checksum.to_le_bytes())
      .map_err(|_| EepromError::BufferTooSmall)?;

    Ok(buffer)
  }

  /// 反序列化数据记录
  fn deserialize_record(&self, buffer: &[u8]) -> Result<DataRecord, EepromError> {
    if buffer.len() < 10 {
      return Err(EepromError::InvalidData);
    }

    // 解析头部
    let id = u16::from_le_bytes([buffer[0], buffer[1]]);
    let timestamp = u32::from_le_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]);
    let data_length = buffer[6] as usize;
    let version = buffer[7];

    if version != DATA_VERSION {
      return Err(EepromError::VersionMismatch);
    }

    if buffer.len() != 8 + data_length + 2 {
      return Err(EepromError::InvalidData);
    }

    // 验证校验和
    let data_end = 8 + data_length;
    let stored_checksum = u16::from_le_bytes([buffer[data_end], buffer[data_end + 1]]);
    let calculated_checksum = CRC16.checksum(&buffer[..data_end]);

    if stored_checksum != calculated_checksum {
      return Err(EepromError::ChecksumMismatch);
    }

    // 提取数据
    let mut data = Vec::new();
    data
      .extend_from_slice(&buffer[8..data_end])
      .map_err(|_| EepromError::BufferTooSmall)?;

    Ok(DataRecord {
      id,
      timestamp,
      data,
      checksum: stored_checksum,
    })
  }

  /// 带重试的数据写入
  fn write_data_with_retry(
    &mut self,
    i2c: &mut I2cType,
    device: &EepromDevice,
    address: u16,
    data: &[u8],
  ) -> Result<(), EepromError> {
    for attempt in 0..self.retry_count {
      match self.write_data_paged(i2c, device, address, data) {
        Ok(_) => return Ok(()),
        Err(e) => {
          self.error_count += 1;
          if attempt == self.retry_count - 1 {
            return Err(e);
          }
          // 等待后重试
          delay_ms(10);
        }
      }
    }
    Err(EepromError::MaxRetriesExceeded)
  }

  /// 分页写入数据
  fn write_data_paged(
    &self,
    i2c: &mut I2cType,
    device: &EepromDevice,
    start_address: u16,
    data: &[u8],
  ) -> Result<(), EepromError> {
    let mut offset = 0;
    let mut current_address = start_address;

    while offset < data.len() {
      // 计算当前页的剩余空间
      let page_start = current_address as usize & !(device.page_size - 1);
      let page_offset = current_address as usize - page_start;
      let page_remaining = device.page_size - page_offset;

      // 确定本次写入的字节数
      let bytes_to_write = core::cmp::min(page_remaining, data.len() - offset);

      // 写入数据
      let mut write_buffer = Vec::<u8, 34>::new(); // 最大页大小 + 地址
      write_buffer
        .extend_from_slice(&current_address.to_be_bytes())
        .map_err(|_| EepromError::BufferTooSmall)?;
      write_buffer
        .extend_from_slice(&data[offset..offset + bytes_to_write])
        .map_err(|_| EepromError::BufferTooSmall)?;

      i2c
        .write(device.address, &write_buffer)
        .map_err(|_| EepromError::I2cError)?;

      // 等待写入完成
      delay_ms(device.write_time_ms);

      // 更新偏移和地址
      offset += bytes_to_write;
      current_address += bytes_to_write as u16;
    }

    Ok(())
  }

  /// 带重试的数据读取
  fn read_data_with_retry(
    &mut self,
    i2c: &mut I2cType,
    device: &EepromDevice,
    address: u16,
    buffer: &mut [u8],
  ) -> Result<(), EepromError> {
    for attempt in 0..self.retry_count {
      match i2c.write_read(device.address, &address.to_be_bytes(), buffer) {
        Ok(_) => return Ok(()),
        Err(_) => {
          self.error_count += 1;
          if attempt == self.retry_count - 1 {
            return Err(EepromError::I2cError);
          }
          delay_ms(1);
        }
      }
    }
    Err(EepromError::MaxRetriesExceeded)
  }

  /// 获取错误统计
  pub fn get_error_count(&self) -> u32 {
    self.error_count
  }

  /// 重置错误计数
  pub fn reset_error_count(&mut self) {
    self.error_count = 0;
  }
}

/// EEPROM错误类型
#[derive(Debug, Clone, Copy)]
pub enum EepromError {
  DeviceNotFound,
  I2cError,
  InvalidData,
  ChecksumMismatch,
  VersionMismatch,
  BufferTooSmall,
  MaxRetriesExceeded,
}

#[entry]
fn main() -> ! {
  // 获取外设句柄
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

  // 配置 GPIO
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 配置状态 LED
  let mut led = gpioc.pc13.into_push_pull_output();
  led.set_high();

  // 配置 I2C 引脚
  let scl = gpiob.pb6.into_alternate_open_drain();
  let sda = gpiob.pb7.into_alternate_open_drain();

  // 初始化 I2C
  let mut i2c = I2c::new(
    dp.I2C1,
    (scl, sda),
    Mode::Standard {
      frequency: 100.kHz(),
    },
    &clocks,
  );

  // 初始化定时器
  let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
  timer.start(1.Hz()).unwrap();

  // 创建EEPROM管理器
  let mut eeprom_manager = EepromManager::new();

  // 启动指示
  startup_sequence(&mut led);

  // 扫描设备
  let found_devices = eeprom_manager.scan_devices(&mut i2c);
  indicate_device_count(&mut led, found_devices.len());

  if found_devices.is_empty() {
    // 没有找到设备，进入错误指示循环
    loop {
      error_indication(&mut led);
    }
  }

  // 演示高级操作
  demonstrate_advanced_operations(&mut eeprom_manager, &mut i2c, &mut led, &mut timer);

  loop {
    // 主循环
    led.set_low();
    delay_ms(1000);
    led.set_high();
    delay_ms(1000);
  }
}

fn startup_sequence(led: &mut PC13<Output<PushPull>>) {
  for _ in 0..5 {
    led.set_low();
    delay_ms(100);
    led.set_high();
    delay_ms(100);
  }
}

fn indicate_device_count(led: &mut PC13<Output<PushPull>>, count: usize) {
  delay_ms(500);
  for _ in 0..count {
    led.set_low();
    delay_ms(200);
    led.set_high();
    delay_ms(200);
  }
  delay_ms(500);
}

fn error_indication(led: &mut PC13<Output<PushPull>>) {
  // 快速闪烁表示错误
  for _ in 0..10 {
    led.set_low();
    delay_ms(50);
    led.set_high();
    delay_ms(50);
  }
  delay_ms(1000);
}

fn demonstrate_advanced_operations(
  manager: &mut EepromManager,
  i2c: &mut I2cType,
  led: &mut PC13<Output<PushPull>>,
  timer: &mut Timer<pac::TIM2>,
) {
  let mut timestamp = 0u32;

  // 1. 写入测试记录
  for i in 0..5 {
    let mut data = Vec::new();
    data
      .extend_from_slice(&format!("Test data {}", i).as_bytes()[..10])
      .ok();

    let record = DataRecord {
      id: i as u16,
      timestamp,
      data,
      checksum: 0, // 将在序列化时计算
    };

    match manager.write_record(i2c, i * 64, &record) {
      Ok(_) => {
        led.set_low();
        delay_ms(100);
        led.set_high();
        delay_ms(100);
      }
      Err(_) => {
        error_indication(led);
      }
    }

    timestamp += 1000;
  }

  delay_ms(1000);

  // 2. 读取并验证记录
  for i in 0..5 {
    match manager.read_record(i2c, i * 64) {
      Ok(record) => {
        // 验证数据
        if record.id == i as u16 {
          // 成功指示
          led.set_low();
          delay_ms(50);
          led.set_high();
          delay_ms(50);
        } else {
          error_indication(led);
        }
      }
      Err(_) => {
        error_indication(led);
      }
    }
  }

  // 3. 显示错误统计
  let error_count = manager.get_error_count();
  if error_count > 0 {
    // 通过LED闪烁次数显示错误数量
    delay_ms(1000);
    for _ in 0..error_count.min(10) {
      led.set_low();
      delay_ms(300);
      led.set_high();
      delay_ms(300);
    }
  }
}

// 简单延时函数
fn delay_ms(ms: u32) {
  for _ in 0..(ms * 8400) {
    cortex_m::asm::nop();
  }
}
