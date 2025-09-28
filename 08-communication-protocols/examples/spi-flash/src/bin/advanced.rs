#![no_std]
#![no_main]

//! # 高级SPI Flash操作示例
//!
//! 演示SPI Flash的高级功能：
//! - 多设备管理
//! - 磨损均衡
//! - 错误恢复
//! - 性能优化
//! - 数据完整性验证

use bitfield::bitfield;
use cortex_m_rt::entry;
use crc::{Crc, CRC_32_ISO_HDLC};
use defmt_rtt as _;
use heapless::{FnvIndexMap, Vec};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{
    gpioa::{PA5, PA6, PA7},
    gpiob::PB0,
    gpioc::PC13,
    Alternate, Output, PushPull,
  },
  pac,
  prelude::*,
  spi::{Mode, Phase, Polarity, Spi},
};

// Flash 命令定义
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_WRITE_DISABLE: u8 = 0x04;
const CMD_READ_STATUS: u8 = 0x05;
const CMD_WRITE_STATUS: u8 = 0x01;
const CMD_READ_DATA: u8 = 0x03;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_SECTOR_ERASE: u8 = 0x20;
const CMD_BLOCK_ERASE_32K: u8 = 0x52;
const CMD_BLOCK_ERASE_64K: u8 = 0xD8;
const CMD_CHIP_ERASE: u8 = 0xC7;
const CMD_POWER_DOWN: u8 = 0xB9;
const CMD_RELEASE_POWER_DOWN: u8 = 0xAB;
const CMD_JEDEC_ID: u8 = 0x9F;
const CMD_UNIQUE_ID: u8 = 0x4B;

// Flash 参数
const PAGE_SIZE: usize = 256;
const SECTOR_SIZE: usize = 4096;
const BLOCK_32K_SIZE: usize = 32768;
const BLOCK_64K_SIZE: usize = 65536;
const TOTAL_SIZE: usize = 8 * 1024 * 1024; // 8MB

// 状态寄存器位定义
const STATUS_BUSY: u8 = 0x01;
const STATUS_WEL: u8 = 0x02;
const STATUS_BP0: u8 = 0x04;
const STATUS_BP1: u8 = 0x08;
const STATUS_BP2: u8 = 0x10;
const STATUS_TB: u8 = 0x20;
const STATUS_SEC: u8 = 0x40;
const STATUS_SRP: u8 = 0x80;

// CRC计算器
const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

type SpiType = Spi<pac::SPI1, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>;
type CsPin = PB0<Output<PushPull>>;

bitfield! {
    /// Flash状态寄存器
    pub struct StatusRegister(u8);
    impl Debug;
    pub busy, set_busy: 0;
    pub wel, set_wel: 1;
    pub bp0, set_bp0: 2;
    pub bp1, set_bp1: 3;
    pub bp2, set_bp2: 4;
    pub tb, set_tb: 5;
    pub sec, set_sec: 6;
    pub srp, set_srp: 7;
}

/// JEDEC ID信息
#[derive(Debug, Clone, Copy)]
pub struct JedecId {
  pub manufacturer_id: u8,
  pub memory_type: u8,
  pub capacity: u8,
}

impl JedecId {
  pub fn from_bytes(bytes: &[u8; 3]) -> Self {
    Self {
      manufacturer_id: bytes[0],
      memory_type: bytes[1],
      capacity: bytes[2],
    }
  }

  pub fn get_capacity_bytes(&self) -> usize {
    1 << self.capacity as usize
  }

  pub fn is_winbond(&self) -> bool {
    self.manufacturer_id == 0xEF
  }
}

/// Flash设备信息
#[derive(Debug, Clone)]
pub struct FlashDevice {
  pub id: u8,
  pub jedec_id: JedecId,
  pub unique_id: [u8; 8],
  pub total_size: usize,
  pub page_size: usize,
  pub sector_size: usize,
  pub erase_cycles: u32,
  pub bad_blocks: Vec<u16, 32>,
}

/// 磨损均衡信息
#[derive(Debug, Clone, Copy)]
pub struct WearLevelingInfo {
  pub sector: u16,
  pub erase_count: u32,
  pub last_written: u32,
}

/// Flash管理器
pub struct FlashManager {
  devices: Vec<FlashDevice, 4>,
  wear_leveling: FnvIndexMap<u16, WearLevelingInfo, 128>,
  current_device: u8,
  error_count: u32,
  performance_stats: PerformanceStats,
}

/// 性能统计
#[derive(Debug, Clone, Copy)]
pub struct PerformanceStats {
  pub read_operations: u32,
  pub write_operations: u32,
  pub erase_operations: u32,
  pub total_bytes_read: u32,
  pub total_bytes_written: u32,
  pub average_read_time_us: u32,
  pub average_write_time_us: u32,
  pub error_count: u32,
}

impl Default for PerformanceStats {
  fn default() -> Self {
    Self {
      read_operations: 0,
      write_operations: 0,
      erase_operations: 0,
      total_bytes_read: 0,
      total_bytes_written: 0,
      average_read_time_us: 0,
      average_write_time_us: 0,
      error_count: 0,
    }
  }
}

impl FlashManager {
  pub fn new() -> Self {
    Self {
      devices: Vec::new(),
      wear_leveling: FnvIndexMap::new(),
      current_device: 0,
      error_count: 0,
      performance_stats: PerformanceStats::default(),
    }
  }

  /// 初始化Flash设备
  pub fn init_device(
    &mut self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    device_id: u8,
  ) -> Result<(), FlashError> {
    // 读取JEDEC ID
    let jedec_id = self.read_jedec_id(spi, cs)?;

    // 读取唯一ID
    let unique_id = self.read_unique_id(spi, cs)?;

    // 创建设备信息
    let device = FlashDevice {
      id: device_id,
      jedec_id,
      unique_id,
      total_size: jedec_id.get_capacity_bytes(),
      page_size: PAGE_SIZE,
      sector_size: SECTOR_SIZE,
      erase_cycles: 0,
      bad_blocks: Vec::new(),
    };

    self
      .devices
      .push(device)
      .map_err(|_| FlashError::TooManyDevices)?;

    // 初始化磨损均衡表
    self.init_wear_leveling(device_id)?;

    Ok(())
  }

  /// 高级读取操作（带重试和错误恢复）
  pub fn advanced_read(
    &mut self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    buffer: &mut [u8],
  ) -> Result<usize, FlashError> {
    let start_time = get_timestamp();
    let mut retry_count = 0;
    const MAX_RETRIES: u8 = 3;

    while retry_count < MAX_RETRIES {
      match self.read_with_verification(spi, cs, address, buffer) {
        Ok(bytes_read) => {
          // 更新性能统计
          self.performance_stats.read_operations += 1;
          self.performance_stats.total_bytes_read += bytes_read as u32;
          let elapsed = get_timestamp() - start_time;
          self.update_average_read_time(elapsed);

          return Ok(bytes_read);
        }
        Err(e) => {
          retry_count += 1;
          self.performance_stats.error_count += 1;

          if retry_count < MAX_RETRIES {
            // 尝试错误恢复
            self.recover_from_error(spi, cs, &e)?;
            delay_ms(10); // 短暂延时后重试
          } else {
            return Err(e);
          }
        }
      }
    }

    Err(FlashError::MaxRetriesExceeded)
  }

  /// 高级写入操作（带磨损均衡）
  pub fn advanced_write(
    &mut self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    data: &[u8],
  ) -> Result<(), FlashError> {
    let start_time = get_timestamp();

    // 检查磨损均衡
    let optimal_address = self.get_optimal_write_address(address)?;

    // 执行写入
    self.write_with_verification(spi, cs, optimal_address, data)?;

    // 更新磨损均衡信息
    self.update_wear_leveling(optimal_address, data.len())?;

    // 更新性能统计
    self.performance_stats.write_operations += 1;
    self.performance_stats.total_bytes_written += data.len() as u32;
    let elapsed = get_timestamp() - start_time;
    self.update_average_write_time(elapsed);

    Ok(())
  }

  /// 智能擦除操作
  pub fn smart_erase(
    &mut self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
  ) -> Result<(), FlashError> {
    let sector = (address / SECTOR_SIZE as u32) as u16;

    // 检查是否为坏块
    if self.is_bad_block(sector) {
      return Err(FlashError::BadBlock);
    }

    // 选择最优擦除策略
    let erase_type = self.select_erase_type(address);

    match erase_type {
      EraseType::Sector => self.erase_sector(spi, cs, address)?,
      EraseType::Block32K => self.erase_block_32k(spi, cs, address)?,
      EraseType::Block64K => self.erase_block_64k(spi, cs, address)?,
    }

    // 更新擦除计数
    self.increment_erase_count(sector)?;

    // 验证擦除结果
    self.verify_erase(spi, cs, address, erase_type.size())?;

    self.performance_stats.erase_operations += 1;

    Ok(())
  }

  /// 数据完整性检查
  pub fn verify_data_integrity(
    &mut self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    expected_data: &[u8],
  ) -> Result<bool, FlashError> {
    let mut read_buffer = [0u8; 1024];
    let chunk_size = core::cmp::min(expected_data.len(), read_buffer.len());

    for (i, chunk) in expected_data.chunks(chunk_size).enumerate() {
      let chunk_address = address + (i * chunk_size) as u32;
      let bytes_read = self.read_data(spi, cs, chunk_address, &mut read_buffer[..chunk.len()])?;

      if bytes_read != chunk.len() || &read_buffer[..bytes_read] != chunk {
        return Ok(false);
      }
    }

    Ok(true)
  }

  /// 获取设备健康状态
  pub fn get_device_health(&self, device_id: u8) -> Result<DeviceHealth, FlashError> {
    let device = self
      .devices
      .iter()
      .find(|d| d.id == device_id)
      .ok_or(FlashError::DeviceNotFound)?;

    let total_sectors = device.total_size / device.sector_size;
    let bad_block_percentage = (device.bad_blocks.len() * 100) / total_sectors;

    let average_erase_count = self.calculate_average_erase_count();
    let max_erase_count = self.get_max_erase_count();

    let health_score =
      self.calculate_health_score(bad_block_percentage, average_erase_count, max_erase_count);

    Ok(DeviceHealth {
      device_id,
      health_score,
      bad_block_count: device.bad_blocks.len(),
      total_erase_cycles: device.erase_cycles,
      average_erase_count,
      max_erase_count,
      error_rate: self.calculate_error_rate(),
    })
  }

  /// 执行垃圾回收
  pub fn garbage_collect(&mut self, spi: &mut SpiType, cs: &mut CsPin) -> Result<u32, FlashError> {
    let mut reclaimed_bytes = 0u32;

    // 查找需要回收的扇区
    let sectors_to_reclaim = self.find_sectors_for_gc();

    for sector in sectors_to_reclaim {
      // 读取有效数据
      let valid_data = self.extract_valid_data(spi, cs, sector)?;

      // 擦除扇区
      let sector_address = sector as u32 * SECTOR_SIZE as u32;
      self.smart_erase(spi, cs, sector_address)?;

      // 重新写入有效数据
      if !valid_data.is_empty() {
        self.advanced_write(spi, cs, sector_address, &valid_data)?;
      }

      reclaimed_bytes += SECTOR_SIZE as u32 - valid_data.len() as u32;
    }

    Ok(reclaimed_bytes)
  }

  // 私有方法实现

  fn read_jedec_id(&self, spi: &mut SpiType, cs: &mut CsPin) -> Result<JedecId, FlashError> {
    let mut buffer = [0u8; 3];

    cs.set_low();
    spi
      .write(&[CMD_JEDEC_ID])
      .map_err(|_| FlashError::SpiError)?;
    spi
      .transfer(&mut buffer)
      .map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    Ok(JedecId::from_bytes(&buffer))
  }

  fn read_unique_id(&self, spi: &mut SpiType, cs: &mut CsPin) -> Result<[u8; 8], FlashError> {
    let mut buffer = [0u8; 8];

    cs.set_low();
    spi
      .write(&[CMD_UNIQUE_ID, 0x00, 0x00, 0x00, 0x00])
      .map_err(|_| FlashError::SpiError)?;
    spi
      .transfer(&mut buffer)
      .map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    Ok(buffer)
  }

  fn read_with_verification(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    buffer: &mut [u8],
  ) -> Result<usize, FlashError> {
    // 第一次读取
    let bytes_read1 = self.read_data(spi, cs, address, buffer)?;

    // 第二次读取进行验证
    let mut verify_buffer = [0u8; 256];
    let verify_size = core::cmp::min(bytes_read1, verify_buffer.len());
    let bytes_read2 = self.read_data(spi, cs, address, &mut verify_buffer[..verify_size])?;

    // 比较两次读取结果
    if bytes_read1 != bytes_read2 || buffer[..verify_size] != verify_buffer[..verify_size] {
      return Err(FlashError::DataCorruption);
    }

    Ok(bytes_read1)
  }

  fn write_with_verification(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    data: &[u8],
  ) -> Result<(), FlashError> {
    // 执行写入
    self.write_data(spi, cs, address, data)?;

    // 读回验证
    let mut verify_buffer = [0u8; 256];
    let chunk_size = core::cmp::min(data.len(), verify_buffer.len());

    for (i, chunk) in data.chunks(chunk_size).enumerate() {
      let chunk_address = address + (i * chunk_size) as u32;
      let bytes_read = self.read_data(spi, cs, chunk_address, &mut verify_buffer[..chunk.len()])?;

      if bytes_read != chunk.len() || &verify_buffer[..bytes_read] != chunk {
        return Err(FlashError::WriteVerificationFailed);
      }
    }

    Ok(())
  }

  fn read_data(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    buffer: &mut [u8],
  ) -> Result<usize, FlashError> {
    let addr_bytes = address.to_be_bytes();

    cs.set_low();
    spi
      .write(&[
        CMD_FAST_READ,
        addr_bytes[1],
        addr_bytes[2],
        addr_bytes[3],
        0x00,
      ])
      .map_err(|_| FlashError::SpiError)?;
    spi.transfer(buffer).map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    Ok(buffer.len())
  }

  fn write_data(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    data: &[u8],
  ) -> Result<(), FlashError> {
    for (i, chunk) in data.chunks(PAGE_SIZE).enumerate() {
      let page_address = address + (i * PAGE_SIZE) as u32;
      self.write_page(spi, cs, page_address, chunk)?;
    }

    Ok(())
  }

  fn write_page(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    data: &[u8],
  ) -> Result<(), FlashError> {
    // 写使能
    self.write_enable(spi, cs)?;

    // 页编程
    let addr_bytes = address.to_be_bytes();

    cs.set_low();
    spi
      .write(&[
        CMD_PAGE_PROGRAM,
        addr_bytes[1],
        addr_bytes[2],
        addr_bytes[3],
      ])
      .map_err(|_| FlashError::SpiError)?;
    spi.write(data).map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    // 等待写入完成
    self.wait_for_ready(spi, cs)?;

    Ok(())
  }

  fn write_enable(&self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FlashError> {
    cs.set_low();
    spi
      .write(&[CMD_WRITE_ENABLE])
      .map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    Ok(())
  }

  fn wait_for_ready(&self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FlashError> {
    let mut timeout = 10000;

    while timeout > 0 {
      let status = self.read_status(spi, cs)?;
      if !status.busy() {
        return Ok(());
      }

      delay_us(100);
      timeout -= 1;
    }

    Err(FlashError::Timeout)
  }

  fn read_status(&self, spi: &mut SpiType, cs: &mut CsPin) -> Result<StatusRegister, FlashError> {
    let mut buffer = [0u8; 1];

    cs.set_low();
    spi
      .write(&[CMD_READ_STATUS])
      .map_err(|_| FlashError::SpiError)?;
    spi
      .transfer(&mut buffer)
      .map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    Ok(StatusRegister(buffer[0]))
  }

  fn erase_sector(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
  ) -> Result<(), FlashError> {
    self.write_enable(spi, cs)?;

    let addr_bytes = address.to_be_bytes();

    cs.set_low();
    spi
      .write(&[
        CMD_SECTOR_ERASE,
        addr_bytes[1],
        addr_bytes[2],
        addr_bytes[3],
      ])
      .map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    self.wait_for_ready(spi, cs)?;

    Ok(())
  }

  fn erase_block_32k(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
  ) -> Result<(), FlashError> {
    self.write_enable(spi, cs)?;

    let addr_bytes = address.to_be_bytes();

    cs.set_low();
    spi
      .write(&[
        CMD_BLOCK_ERASE_32K,
        addr_bytes[1],
        addr_bytes[2],
        addr_bytes[3],
      ])
      .map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    self.wait_for_ready(spi, cs)?;

    Ok(())
  }

  fn erase_block_64k(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
  ) -> Result<(), FlashError> {
    self.write_enable(spi, cs)?;

    let addr_bytes = address.to_be_bytes();

    cs.set_low();
    spi
      .write(&[
        CMD_BLOCK_ERASE_64K,
        addr_bytes[1],
        addr_bytes[2],
        addr_bytes[3],
      ])
      .map_err(|_| FlashError::SpiError)?;
    cs.set_high();

    self.wait_for_ready(spi, cs)?;

    Ok(())
  }

  fn init_wear_leveling(&mut self, device_id: u8) -> Result<(), FlashError> {
    let device = self
      .devices
      .iter()
      .find(|d| d.id == device_id)
      .ok_or(FlashError::DeviceNotFound)?;

    let total_sectors = device.total_size / device.sector_size;

    for sector in 0..total_sectors as u16 {
      let info = WearLevelingInfo {
        sector,
        erase_count: 0,
        last_written: 0,
      };

      self
        .wear_leveling
        .insert(sector, info)
        .map_err(|_| FlashError::InternalError)?;
    }

    Ok(())
  }

  fn get_optimal_write_address(&self, requested_address: u32) -> Result<u32, FlashError> {
    let requested_sector = (requested_address / SECTOR_SIZE as u32) as u16;

    // 查找磨损最少的扇区
    let mut min_erase_count = u32::MAX;
    let mut optimal_sector = requested_sector;

    for (&sector, &info) in &self.wear_leveling {
      if info.erase_count < min_erase_count && !self.is_bad_block(sector) {
        min_erase_count = info.erase_count;
        optimal_sector = sector;
      }
    }

    Ok(optimal_sector as u32 * SECTOR_SIZE as u32 + (requested_address % SECTOR_SIZE as u32))
  }

  fn update_wear_leveling(&mut self, address: u32, _size: usize) -> Result<(), FlashError> {
    let sector = (address / SECTOR_SIZE as u32) as u16;

    if let Some(info) = self.wear_leveling.get_mut(&sector) {
      info.last_written = get_timestamp();
    }

    Ok(())
  }

  fn increment_erase_count(&mut self, sector: u16) -> Result<(), FlashError> {
    if let Some(info) = self.wear_leveling.get_mut(&sector) {
      info.erase_count += 1;
    }

    Ok(())
  }

  fn is_bad_block(&self, sector: u16) -> bool {
    self
      .devices
      .iter()
      .any(|device| device.bad_blocks.contains(&sector))
  }

  fn select_erase_type(&self, address: u32) -> EraseType {
    // 简单的擦除类型选择逻辑
    if address % BLOCK_64K_SIZE as u32 == 0 {
      EraseType::Block64K
    } else if address % BLOCK_32K_SIZE as u32 == 0 {
      EraseType::Block32K
    } else {
      EraseType::Sector
    }
  }

  fn verify_erase(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    size: usize,
  ) -> Result<(), FlashError> {
    let mut buffer = [0u8; 256];
    let mut offset = 0;

    while offset < size {
      let chunk_size = core::cmp::min(buffer.len(), size - offset);
      let chunk_address = address + offset as u32;

      self.read_data(spi, cs, chunk_address, &mut buffer[..chunk_size])?;

      // 检查是否全为0xFF（擦除状态）
      if !buffer[..chunk_size].iter().all(|&b| b == 0xFF) {
        return Err(FlashError::EraseVerificationFailed);
      }

      offset += chunk_size;
    }

    Ok(())
  }

  fn recover_from_error(
    &mut self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    error: &FlashError,
  ) -> Result<(), FlashError> {
    match error {
      FlashError::DataCorruption => {
        // 尝试重新初始化SPI接口
        delay_ms(10);
        Ok(())
      }
      FlashError::Timeout => {
        // 发送复位命令
        cs.set_low();
        delay_us(1);
        cs.set_high();
        delay_ms(1);
        Ok(())
      }
      _ => Ok(()),
    }
  }

  fn calculate_average_erase_count(&self) -> u32 {
    if self.wear_leveling.is_empty() {
      return 0;
    }

    let total: u32 = self
      .wear_leveling
      .values()
      .map(|info| info.erase_count)
      .sum();

    total / self.wear_leveling.len() as u32
  }

  fn get_max_erase_count(&self) -> u32 {
    self
      .wear_leveling
      .values()
      .map(|info| info.erase_count)
      .max()
      .unwrap_or(0)
  }

  fn calculate_health_score(
    &self,
    bad_block_percentage: usize,
    average_erase_count: u32,
    max_erase_count: u32,
  ) -> u8 {
    let mut score = 100u8;

    // 坏块影响
    score = score.saturating_sub(bad_block_percentage as u8 * 2);

    // 擦除次数影响
    if max_erase_count > 10000 {
      score = score.saturating_sub(20);
    } else if max_erase_count > 5000 {
      score = score.saturating_sub(10);
    }

    // 磨损均匀性影响
    if max_erase_count > average_erase_count * 2 {
      score = score.saturating_sub(15);
    }

    score
  }

  fn calculate_error_rate(&self) -> f32 {
    if self.performance_stats.read_operations + self.performance_stats.write_operations == 0 {
      return 0.0;
    }

    self.performance_stats.error_count as f32
      / (self.performance_stats.read_operations + self.performance_stats.write_operations) as f32
  }

  fn update_average_read_time(&mut self, elapsed: u32) {
    let total_time =
      self.performance_stats.average_read_time_us * (self.performance_stats.read_operations - 1);
    self.performance_stats.average_read_time_us =
      (total_time + elapsed) / self.performance_stats.read_operations;
  }

  fn update_average_write_time(&mut self, elapsed: u32) {
    let total_time =
      self.performance_stats.average_write_time_us * (self.performance_stats.write_operations - 1);
    self.performance_stats.average_write_time_us =
      (total_time + elapsed) / self.performance_stats.write_operations;
  }

  fn find_sectors_for_gc(&self) -> Vec<u16, 16> {
    let mut sectors = Vec::new();

    // 简单的GC策略：选择擦除次数最高的扇区
    let mut sector_counts: Vec<(u16, u32), 128> = Vec::new();

    for (&sector, &info) in &self.wear_leveling {
      sector_counts.push((sector, info.erase_count)).ok();
    }

    // 排序并选择前几个
    sector_counts.sort_by(|a, b| b.1.cmp(&a.1));

    for (sector, _) in sector_counts.iter().take(4) {
      sectors.push(*sector).ok();
    }

    sectors
  }

  fn extract_valid_data(
    &self,
    spi: &mut SpiType,
    cs: &mut CsPin,
    sector: u16,
  ) -> Result<Vec<u8, 4096>, FlashError> {
    let mut data = Vec::new();
    let mut buffer = [0u8; SECTOR_SIZE];
    let sector_address = sector as u32 * SECTOR_SIZE as u32;

    self.read_data(spi, cs, sector_address, &mut buffer)?;

    // 简单的有效数据检测：非0xFF的数据
    for &byte in &buffer {
      if byte != 0xFF {
        data.push(byte).map_err(|_| FlashError::InternalError)?;
      }
    }

    Ok(data)
  }
}

/// 擦除类型
#[derive(Debug, Clone, Copy)]
pub enum EraseType {
  Sector,
  Block32K,
  Block64K,
}

impl EraseType {
  pub fn size(&self) -> usize {
    match self {
      EraseType::Sector => SECTOR_SIZE,
      EraseType::Block32K => BLOCK_32K_SIZE,
      EraseType::Block64K => BLOCK_64K_SIZE,
    }
  }
}

/// 设备健康状态
#[derive(Debug)]
pub struct DeviceHealth {
  pub device_id: u8,
  pub health_score: u8, // 0-100
  pub bad_block_count: usize,
  pub total_erase_cycles: u32,
  pub average_erase_count: u32,
  pub max_erase_count: u32,
  pub error_rate: f32,
}

/// Flash错误类型
#[derive(Debug, Clone, Copy)]
pub enum FlashError {
  SpiError,
  Timeout,
  DataCorruption,
  WriteVerificationFailed,
  EraseVerificationFailed,
  BadBlock,
  DeviceNotFound,
  TooManyDevices,
  MaxRetriesExceeded,
  InternalError,
}

#[entry]
fn main() -> ! {
  // 获取外设句柄
  let dp = pac::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

  // 配置 GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 配置状态 LED
  let mut led = gpioc.pc13.into_push_pull_output();
  led.set_high();

  // 配置 SPI 引脚
  let sck = gpioa.pa5.into_alternate();
  let miso = gpioa.pa6.into_alternate();
  let mosi = gpioa.pa7.into_alternate();
  let mut cs = gpiob.pb0.into_push_pull_output();
  cs.set_high();

  // 初始化 SPI
  let mut spi = Spi::new(
    dp.SPI1,
    (sck, miso, mosi),
    Mode {
      polarity: Polarity::IdleLow,
      phase: Phase::CaptureOnFirstTransition,
    },
    1.MHz(),
    &clocks,
  );

  // 创建Flash管理器
  let mut flash_manager = FlashManager::new();

  // 启动指示
  startup_sequence(&mut led);

  // 演示高级Flash操作
  demonstrate_advanced_flash(&mut flash_manager, &mut spi, &mut cs, &mut led);

  loop {
    // 主循环
    led.set_low();
    delay_ms(2000);
    led.set_high();
    delay_ms(2000);
  }
}

fn startup_sequence(led: &mut PC13<Output<PushPull>>) {
  for _ in 0..3 {
    led.set_low();
    delay_ms(200);
    led.set_high();
    delay_ms(200);
  }
}

fn demonstrate_advanced_flash(
  manager: &mut FlashManager,
  spi: &mut SpiType,
  cs: &mut CsPin,
  led: &mut PC13<Output<PushPull>>,
) {
  // 1. 初始化Flash设备
  match manager.init_device(spi, cs, 0) {
    Ok(_) => {
      // 成功指示
      for _ in 0..2 {
        led.set_low();
        delay_ms(100);
        led.set_high();
        delay_ms(100);
      }
    }
    Err(_) => {
      error_indication(led);
      return;
    }
  }

  delay_ms(500);

  // 2. 高级写入测试
  let test_data = b"Advanced Flash Test Data with Wear Leveling!";
  match manager.advanced_write(spi, cs, 0x1000, test_data) {
    Ok(_) => {
      led.set_low();
      delay_ms(200);
      led.set_high();
      delay_ms(200);
    }
    Err(_) => {
      error_indication(led);
    }
  }

  // 3. 高级读取测试
  let mut read_buffer = [0u8; 64];
  match manager.advanced_read(spi, cs, 0x1000, &mut read_buffer) {
    Ok(bytes_read) => {
      // 验证数据
      if &read_buffer[..bytes_read] == test_data {
        // 成功指示
        for _ in 0..3 {
          led.set_low();
          delay_ms(100);
          led.set_high();
          delay_ms(100);
        }
      } else {
        error_indication(led);
      }
    }
    Err(_) => {
      error_indication(led);
    }
  }

  // 4. 数据完整性验证
  match manager.verify_data_integrity(spi, cs, 0x1000, test_data) {
    Ok(true) => {
      // 完整性验证成功
      for _ in 0..4 {
        led.set_low();
        delay_ms(100);
        led.set_high();
        delay_ms(100);
      }
    }
    Ok(false) => {
      error_indication(led);
    }
    Err(_) => {
      error_indication(led);
    }
  }

  // 5. 智能擦除测试
  match manager.smart_erase(spi, cs, 0x1000) {
    Ok(_) => {
      led.set_low();
      delay_ms(300);
      led.set_high();
      delay_ms(300);
    }
    Err(_) => {
      error_indication(led);
    }
  }

  // 6. 设备健康检查
  match manager.get_device_health(0) {
    Ok(health) => {
      // 通过LED闪烁显示健康分数（十位数）
      let health_tens = health.health_score / 10;
      delay_ms(1000);
      for _ in 0..health_tens {
        led.set_low();
        delay_ms(200);
        led.set_high();
        delay_ms(200);
      }
    }
    Err(_) => {
      error_indication(led);
    }
  }
}

fn error_indication(led: &mut PC13<Output<PushPull>>) {
  for _ in 0..5 {
    led.set_low();
    delay_ms(50);
    led.set_high();
    delay_ms(50);
  }
}

// 简单的时间戳函数
fn get_timestamp() -> u32 {
  // 实际应用中应该使用系统定时器
  static mut COUNTER: u32 = 0;
  unsafe {
    COUNTER += 1;
    COUNTER
  }
}

// 简单延时函数
fn delay_ms(ms: u32) {
  for _ in 0..(ms * 8400) {
    cortex_m::asm::nop();
  }
}

fn delay_us(us: u32) {
  for _ in 0..(us * 8) {
    cortex_m::asm::nop();
  }
}
