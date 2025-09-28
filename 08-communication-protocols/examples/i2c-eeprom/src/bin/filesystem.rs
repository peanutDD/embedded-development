#![no_std]
#![no_main]

//! # EEPROM文件系统示例
//!
//! 在EEPROM上实现简单的文件系统：
//! - 文件分配表(FAT)
//! - 目录结构
//! - 文件读写操作
//! - 磨损均衡

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
};

// 文件系统常量
const SECTOR_SIZE: usize = 64; // 扇区大小
const MAX_FILES: usize = 16; // 最大文件数
const MAX_FILENAME_LEN: usize = 12; // 最大文件名长度
const FAT_SECTORS: usize = 2; // FAT占用扇区数
const ROOT_DIR_SECTORS: usize = 1; // 根目录占用扇区数
const MAGIC_NUMBER: u32 = 0x45455046; // "FEEE" 文件系统标识

// CRC计算器
const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

type I2cType = I2c<pac::I2C1, (PB6<AlternateOD<4>>, PB7<AlternateOD<4>>)>;

/// 文件系统超级块
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SuperBlock {
  pub magic: u32,             // 魔数
  pub version: u16,           // 版本号
  pub sector_size: u16,       // 扇区大小
  pub total_sectors: u16,     // 总扇区数
  pub fat_sectors: u16,       // FAT扇区数
  pub root_dir_sectors: u16,  // 根目录扇区数
  pub data_start_sector: u16, // 数据区起始扇区
  pub free_sectors: u16,      // 空闲扇区数
  pub checksum: u16,          // 校验和
}

/// 文件分配表项
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u16)]
pub enum FatEntry {
  Free = 0x0000,       // 空闲
  Reserved = 0xFFF0,   // 保留
  Bad = 0xFFF7,        // 坏扇区
  EndOfChain = 0xFFFF, // 链结束
  Used(u16),           // 指向下一个扇区
}

impl From<u16> for FatEntry {
  fn from(value: u16) -> Self {
    match value {
      0x0000 => FatEntry::Free,
      0xFFF0 => FatEntry::Reserved,
      0xFFF7 => FatEntry::Bad,
      0xFFFF => FatEntry::EndOfChain,
      n => FatEntry::Used(n),
    }
  }
}

impl From<FatEntry> for u16 {
  fn from(entry: FatEntry) -> Self {
    match entry {
      FatEntry::Free => 0x0000,
      FatEntry::Reserved => 0xFFF0,
      FatEntry::Bad => 0xFFF7,
      FatEntry::EndOfChain => 0xFFFF,
      FatEntry::Used(n) => n,
    }
  }
}

/// 目录项
#[derive(Debug, Clone)]
#[repr(C)]
pub struct DirectoryEntry {
  pub filename: String<MAX_FILENAME_LEN>, // 文件名
  pub size: u32,                          // 文件大小
  pub start_sector: u16,                  // 起始扇区
  pub attributes: FileAttributes,         // 文件属性
  pub created_time: u32,                  // 创建时间
  pub modified_time: u32,                 // 修改时间
  pub checksum: u16,                      // 校验和
}

/// 文件属性
#[derive(Debug, Clone, Copy)]
pub struct FileAttributes {
  pub read_only: bool,
  pub hidden: bool,
  pub system: bool,
  pub directory: bool,
}

impl FileAttributes {
  pub fn new() -> Self {
    Self {
      read_only: false,
      hidden: false,
      system: false,
      directory: false,
    }
  }

  pub fn to_byte(&self) -> u8 {
    let mut byte = 0u8;
    if self.read_only {
      byte |= 0x01;
    }
    if self.hidden {
      byte |= 0x02;
    }
    if self.system {
      byte |= 0x04;
    }
    if self.directory {
      byte |= 0x08;
    }
    byte
  }

  pub fn from_byte(byte: u8) -> Self {
    Self {
      read_only: (byte & 0x01) != 0,
      hidden: (byte & 0x02) != 0,
      system: (byte & 0x04) != 0,
      directory: (byte & 0x08) != 0,
    }
  }
}

/// EEPROM文件系统
pub struct EepromFileSystem {
  eeprom_addr: u8,
  super_block: SuperBlock,
  fat: Vec<FatEntry, 128>,
  directory: Vec<DirectoryEntry, MAX_FILES>,
  wear_leveling_counter: u32,
}

impl EepromFileSystem {
  pub fn new(eeprom_addr: u8, total_size: usize) -> Self {
    let total_sectors = total_size / SECTOR_SIZE;
    let data_start_sector = FAT_SECTORS + ROOT_DIR_SECTORS + 1; // +1 for super block

    let super_block = SuperBlock {
      magic: MAGIC_NUMBER,
      version: 1,
      sector_size: SECTOR_SIZE as u16,
      total_sectors: total_sectors as u16,
      fat_sectors: FAT_SECTORS as u16,
      root_dir_sectors: ROOT_DIR_SECTORS as u16,
      data_start_sector: data_start_sector as u16,
      free_sectors: (total_sectors - data_start_sector) as u16,
      checksum: 0,
    };

    let mut fat = Vec::new();
    fat.resize(total_sectors, FatEntry::Free).ok();

    // 标记系统扇区为保留
    for i in 0..data_start_sector {
      fat[i] = FatEntry::Reserved;
    }

    Self {
      eeprom_addr,
      super_block,
      fat,
      directory: Vec::new(),
      wear_leveling_counter: 0,
    }
  }

  /// 格式化文件系统
  pub fn format(&mut self, i2c: &mut I2cType) -> Result<(), FsError> {
    // 写入超级块
    self.write_super_block(i2c)?;

    // 初始化FAT
    self.write_fat(i2c)?;

    // 初始化根目录
    self.write_root_directory(i2c)?;

    Ok(())
  }

  /// 挂载文件系统
  pub fn mount(&mut self, i2c: &mut I2cType) -> Result<(), FsError> {
    // 读取超级块
    self.read_super_block(i2c)?;

    // 验证文件系统
    if self.super_block.magic != MAGIC_NUMBER {
      return Err(FsError::InvalidFileSystem);
    }

    // 读取FAT
    self.read_fat(i2c)?;

    // 读取根目录
    self.read_root_directory(i2c)?;

    Ok(())
  }

  /// 创建文件
  pub fn create_file(
    &mut self,
    i2c: &mut I2cType,
    filename: &str,
    data: &[u8],
  ) -> Result<(), FsError> {
    // 检查文件名长度
    if filename.len() > MAX_FILENAME_LEN {
      return Err(FsError::FilenameTooLong);
    }

    // 检查文件是否已存在
    if self.find_file(filename).is_some() {
      return Err(FsError::FileExists);
    }

    // 计算需要的扇区数
    let sectors_needed = (data.len() + SECTOR_SIZE - 1) / SECTOR_SIZE;

    // 分配扇区
    let sectors = self.allocate_sectors(sectors_needed)?;

    // 写入文件数据
    self.write_file_data(i2c, &sectors, data)?;

    // 创建目录项
    let mut filename_string = String::new();
    filename_string
      .push_str(filename)
      .map_err(|_| FsError::FilenameTooLong)?;

    let dir_entry = DirectoryEntry {
      filename: filename_string,
      size: data.len() as u32,
      start_sector: sectors[0],
      attributes: FileAttributes::new(),
      created_time: self.get_current_time(),
      modified_time: self.get_current_time(),
      checksum: 0,
    };

    // 添加到目录
    self
      .directory
      .push(dir_entry)
      .map_err(|_| FsError::DirectoryFull)?;

    // 更新磁盘上的目录
    self.write_root_directory(i2c)?;

    // 更新FAT
    self.write_fat(i2c)?;

    Ok(())
  }

  /// 读取文件
  pub fn read_file(
    &mut self,
    i2c: &mut I2cType,
    filename: &str,
    buffer: &mut [u8],
  ) -> Result<usize, FsError> {
    // 查找文件
    let dir_entry = self
      .find_file(filename)
      .ok_or(FsError::FileNotFound)?
      .clone();

    // 检查缓冲区大小
    if buffer.len() < dir_entry.size as usize {
      return Err(FsError::BufferTooSmall);
    }

    // 获取文件的扇区链
    let sectors = self.get_sector_chain(dir_entry.start_sector)?;

    // 读取文件数据
    let bytes_read = self.read_file_data(i2c, &sectors, buffer, dir_entry.size as usize)?;

    Ok(bytes_read)
  }

  /// 删除文件
  pub fn delete_file(&mut self, i2c: &mut I2cType, filename: &str) -> Result<(), FsError> {
    // 查找文件
    let file_index = self
      .directory
      .iter()
      .position(|entry| entry.filename == filename)
      .ok_or(FsError::FileNotFound)?;

    let dir_entry = &self.directory[file_index];

    // 释放扇区链
    self.free_sector_chain(dir_entry.start_sector)?;

    // 从目录中移除
    self.directory.swap_remove(file_index);

    // 更新磁盘上的目录和FAT
    self.write_root_directory(i2c)?;
    self.write_fat(i2c)?;

    Ok(())
  }

  /// 列出文件
  pub fn list_files(&self) -> &[DirectoryEntry] {
    &self.directory
  }

  /// 获取文件系统信息
  pub fn get_fs_info(&self) -> FileSystemInfo {
    let free_sectors = self
      .fat
      .iter()
      .skip(self.super_block.data_start_sector as usize)
      .filter(|&&entry| entry == FatEntry::Free)
      .count();

    FileSystemInfo {
      total_size: (self.super_block.total_sectors as usize) * SECTOR_SIZE,
      free_size: free_sectors * SECTOR_SIZE,
      used_size: ((self.super_block.total_sectors as usize) - free_sectors) * SECTOR_SIZE,
      file_count: self.directory.len(),
    }
  }

  // 私有方法实现

  fn write_super_block(&mut self, i2c: &mut I2cType) -> Result<(), FsError> {
    // 计算校验和
    self.super_block.checksum = self.calculate_super_block_checksum();

    // 序列化超级块
    let data = self.serialize_super_block();

    // 写入扇区0
    self.write_sector(i2c, 0, &data)
  }

  fn read_super_block(&mut self, i2c: &mut I2cType) -> Result<(), FsError> {
    let mut buffer = [0u8; SECTOR_SIZE];
    self.read_sector(i2c, 0, &mut buffer)?;

    self.super_block = self.deserialize_super_block(&buffer)?;

    // 验证校验和
    let calculated_checksum = self.calculate_super_block_checksum();
    if self.super_block.checksum != calculated_checksum {
      return Err(FsError::CorruptedData);
    }

    Ok(())
  }

  fn write_fat(&self, i2c: &mut I2cType) -> Result<(), FsError> {
    let entries_per_sector = SECTOR_SIZE / 2; // 每个FAT项2字节

    for sector in 0..FAT_SECTORS {
      let mut buffer = [0u8; SECTOR_SIZE];
      let start_entry = sector * entries_per_sector;
      let end_entry = core::cmp::min(start_entry + entries_per_sector, self.fat.len());

      for (i, &entry) in self.fat[start_entry..end_entry].iter().enumerate() {
        let bytes = u16::from(entry).to_le_bytes();
        buffer[i * 2] = bytes[0];
        buffer[i * 2 + 1] = bytes[1];
      }

      self.write_sector(i2c, 1 + sector, &buffer)?;
    }

    Ok(())
  }

  fn read_fat(&mut self, i2c: &mut I2cType) -> Result<(), FsError> {
    let entries_per_sector = SECTOR_SIZE / 2;
    self.fat.clear();

    for sector in 0..FAT_SECTORS {
      let mut buffer = [0u8; SECTOR_SIZE];
      self.read_sector(i2c, 1 + sector, &mut buffer)?;

      for i in 0..entries_per_sector {
        if self.fat.len() >= self.super_block.total_sectors as usize {
          break;
        }

        let value = u16::from_le_bytes([buffer[i * 2], buffer[i * 2 + 1]]);
        self
          .fat
          .push(FatEntry::from(value))
          .map_err(|_| FsError::InternalError)?;
      }
    }

    Ok(())
  }

  fn write_root_directory(&self, i2c: &mut I2cType) -> Result<(), FsError> {
    let mut buffer = [0u8; SECTOR_SIZE];
    let mut offset = 0;

    for entry in &self.directory {
      let serialized = self.serialize_directory_entry(entry);
      if offset + serialized.len() > SECTOR_SIZE {
        break; // 目录已满
      }

      buffer[offset..offset + serialized.len()].copy_from_slice(&serialized);
      offset += serialized.len();
    }

    let root_dir_sector = 1 + FAT_SECTORS;
    self.write_sector(i2c, root_dir_sector, &buffer)
  }

  fn read_root_directory(&mut self, i2c: &mut I2cType) -> Result<(), FsError> {
    let mut buffer = [0u8; SECTOR_SIZE];
    let root_dir_sector = 1 + FAT_SECTORS;
    self.read_sector(i2c, root_dir_sector, &mut buffer)?;

    self.directory.clear();
    let mut offset = 0;

    while offset < SECTOR_SIZE {
      if buffer[offset] == 0 {
        break; // 目录结束
      }

      match self.deserialize_directory_entry(&buffer[offset..]) {
        Ok((entry, size)) => {
          self
            .directory
            .push(entry)
            .map_err(|_| FsError::DirectoryFull)?;
          offset += size;
        }
        Err(_) => break,
      }
    }

    Ok(())
  }

  fn allocate_sectors(&mut self, count: usize) -> Result<Vec<u16, 32>, FsError> {
    let mut allocated = Vec::new();
    let mut found = 0;

    // 查找空闲扇区
    for (i, &entry) in self
      .fat
      .iter()
      .enumerate()
      .skip(self.super_block.data_start_sector as usize)
    {
      if entry == FatEntry::Free {
        allocated
          .push(i as u16)
          .map_err(|_| FsError::InternalError)?;
        found += 1;

        if found == count {
          break;
        }
      }
    }

    if found < count {
      return Err(FsError::DiskFull);
    }

    // 标记扇区为已使用并建立链
    for i in 0..allocated.len() {
      if i == allocated.len() - 1 {
        self.fat[allocated[i] as usize] = FatEntry::EndOfChain;
      } else {
        self.fat[allocated[i] as usize] = FatEntry::Used(allocated[i + 1]);
      }
    }

    Ok(allocated)
  }

  fn get_sector_chain(&self, start_sector: u16) -> Result<Vec<u16, 32>, FsError> {
    let mut chain = Vec::new();
    let mut current = start_sector;

    loop {
      chain.push(current).map_err(|_| FsError::InternalError)?;

      match self.fat[current as usize] {
        FatEntry::EndOfChain => break,
        FatEntry::Used(next) => current = next,
        _ => return Err(FsError::CorruptedData),
      }
    }

    Ok(chain)
  }

  fn free_sector_chain(&mut self, start_sector: u16) -> Result<(), FsError> {
    let chain = self.get_sector_chain(start_sector)?;

    for &sector in &chain {
      self.fat[sector as usize] = FatEntry::Free;
    }

    Ok(())
  }

  fn write_file_data(
    &self,
    i2c: &mut I2cType,
    sectors: &[u16],
    data: &[u8],
  ) -> Result<(), FsError> {
    let mut offset = 0;

    for &sector in sectors {
      let bytes_to_write = core::cmp::min(SECTOR_SIZE, data.len() - offset);
      let mut buffer = [0u8; SECTOR_SIZE];

      buffer[..bytes_to_write].copy_from_slice(&data[offset..offset + bytes_to_write]);

      self.write_sector(i2c, sector as usize, &buffer)?;
      offset += bytes_to_write;

      if offset >= data.len() {
        break;
      }
    }

    Ok(())
  }

  fn read_file_data(
    &self,
    i2c: &mut I2cType,
    sectors: &[u16],
    buffer: &mut [u8],
    file_size: usize,
  ) -> Result<usize, FsError> {
    let mut offset = 0;
    let mut bytes_read = 0;

    for &sector in sectors {
      let mut sector_buffer = [0u8; SECTOR_SIZE];
      self.read_sector(i2c, sector as usize, &mut sector_buffer)?;

      let bytes_to_copy = core::cmp::min(SECTOR_SIZE, file_size - bytes_read);
      buffer[offset..offset + bytes_to_copy].copy_from_slice(&sector_buffer[..bytes_to_copy]);

      offset += bytes_to_copy;
      bytes_read += bytes_to_copy;

      if bytes_read >= file_size {
        break;
      }
    }

    Ok(bytes_read)
  }

  fn write_sector(&self, i2c: &mut I2cType, sector: usize, data: &[u8]) -> Result<(), FsError> {
    let address = (sector * SECTOR_SIZE) as u16;
    let addr_bytes = address.to_be_bytes();

    // 分页写入
    let mut offset = 0;
    while offset < data.len() {
      let page_size = core::cmp::min(8, data.len() - offset); // 假设页大小为8字节
      let current_addr = address + offset as u16;
      let current_addr_bytes = current_addr.to_be_bytes();

      let mut write_buffer = [0u8; 10]; // 地址(2) + 数据(8)
      write_buffer[0] = current_addr_bytes[0];
      write_buffer[1] = current_addr_bytes[1];
      write_buffer[2..2 + page_size].copy_from_slice(&data[offset..offset + page_size]);

      i2c
        .write(self.eeprom_addr, &write_buffer[..2 + page_size])
        .map_err(|_| FsError::I2cError)?;

      delay_ms(5); // 写入延时
      offset += page_size;
    }

    Ok(())
  }

  fn read_sector(
    &self,
    i2c: &mut I2cType,
    sector: usize,
    buffer: &mut [u8],
  ) -> Result<(), FsError> {
    let address = (sector * SECTOR_SIZE) as u16;
    let addr_bytes = address.to_be_bytes();

    i2c
      .write_read(self.eeprom_addr, &addr_bytes, buffer)
      .map_err(|_| FsError::I2cError)
  }

  fn find_file(&self, filename: &str) -> Option<&DirectoryEntry> {
    self
      .directory
      .iter()
      .find(|entry| entry.filename == filename)
  }

  fn get_current_time(&self) -> u32 {
    // 简单的时间戳，实际应用中应该使用RTC
    self.wear_leveling_counter
  }

  fn calculate_super_block_checksum(&self) -> u16 {
    // 简化的校验和计算
    let mut sum = 0u16;
    sum = sum.wrapping_add(self.super_block.magic as u16);
    sum = sum.wrapping_add(self.super_block.version);
    sum = sum.wrapping_add(self.super_block.sector_size);
    sum = sum.wrapping_add(self.super_block.total_sectors);
    sum
  }

  fn serialize_super_block(&self) -> [u8; SECTOR_SIZE] {
    let mut buffer = [0u8; SECTOR_SIZE];

    buffer[0..4].copy_from_slice(&self.super_block.magic.to_le_bytes());
    buffer[4..6].copy_from_slice(&self.super_block.version.to_le_bytes());
    buffer[6..8].copy_from_slice(&self.super_block.sector_size.to_le_bytes());
    buffer[8..10].copy_from_slice(&self.super_block.total_sectors.to_le_bytes());
    buffer[10..12].copy_from_slice(&self.super_block.fat_sectors.to_le_bytes());
    buffer[12..14].copy_from_slice(&self.super_block.root_dir_sectors.to_le_bytes());
    buffer[14..16].copy_from_slice(&self.super_block.data_start_sector.to_le_bytes());
    buffer[16..18].copy_from_slice(&self.super_block.free_sectors.to_le_bytes());
    buffer[18..20].copy_from_slice(&self.super_block.checksum.to_le_bytes());

    buffer
  }

  fn deserialize_super_block(&self, buffer: &[u8]) -> Result<SuperBlock, FsError> {
    if buffer.len() < 20 {
      return Err(FsError::CorruptedData);
    }

    Ok(SuperBlock {
      magic: u32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]),
      version: u16::from_le_bytes([buffer[4], buffer[5]]),
      sector_size: u16::from_le_bytes([buffer[6], buffer[7]]),
      total_sectors: u16::from_le_bytes([buffer[8], buffer[9]]),
      fat_sectors: u16::from_le_bytes([buffer[10], buffer[11]]),
      root_dir_sectors: u16::from_le_bytes([buffer[12], buffer[13]]),
      data_start_sector: u16::from_le_bytes([buffer[14], buffer[15]]),
      free_sectors: u16::from_le_bytes([buffer[16], buffer[17]]),
      checksum: u16::from_le_bytes([buffer[18], buffer[19]]),
    })
  }

  fn serialize_directory_entry(&self, entry: &DirectoryEntry) -> Vec<u8, 64> {
    let mut buffer = Vec::new();

    // 文件名长度 + 文件名
    buffer.push(entry.filename.len() as u8).ok();
    buffer.extend_from_slice(entry.filename.as_bytes()).ok();

    // 文件大小
    buffer.extend_from_slice(&entry.size.to_le_bytes()).ok();

    // 起始扇区
    buffer
      .extend_from_slice(&entry.start_sector.to_le_bytes())
      .ok();

    // 属性
    buffer.push(entry.attributes.to_byte()).ok();

    // 时间戳
    buffer
      .extend_from_slice(&entry.created_time.to_le_bytes())
      .ok();
    buffer
      .extend_from_slice(&entry.modified_time.to_le_bytes())
      .ok();

    // 校验和
    buffer.extend_from_slice(&entry.checksum.to_le_bytes()).ok();

    buffer
  }

  fn deserialize_directory_entry(&self, buffer: &[u8]) -> Result<(DirectoryEntry, usize), FsError> {
    if buffer.is_empty() {
      return Err(FsError::CorruptedData);
    }

    let filename_len = buffer[0] as usize;
    if buffer.len() < 1 + filename_len + 15 {
      // 最小目录项大小
      return Err(FsError::CorruptedData);
    }

    let mut offset = 1;

    // 文件名
    let mut filename = String::new();
    filename
      .push_str(
        core::str::from_utf8(&buffer[offset..offset + filename_len])
          .map_err(|_| FsError::CorruptedData)?,
      )
      .map_err(|_| FsError::CorruptedData)?;
    offset += filename_len;

    // 文件大小
    let size = u32::from_le_bytes([
      buffer[offset],
      buffer[offset + 1],
      buffer[offset + 2],
      buffer[offset + 3],
    ]);
    offset += 4;

    // 起始扇区
    let start_sector = u16::from_le_bytes([buffer[offset], buffer[offset + 1]]);
    offset += 2;

    // 属性
    let attributes = FileAttributes::from_byte(buffer[offset]);
    offset += 1;

    // 时间戳
    let created_time = u32::from_le_bytes([
      buffer[offset],
      buffer[offset + 1],
      buffer[offset + 2],
      buffer[offset + 3],
    ]);
    offset += 4;

    let modified_time = u32::from_le_bytes([
      buffer[offset],
      buffer[offset + 1],
      buffer[offset + 2],
      buffer[offset + 3],
    ]);
    offset += 4;

    // 校验和
    let checksum = u16::from_le_bytes([buffer[offset], buffer[offset + 1]]);
    offset += 2;

    let entry = DirectoryEntry {
      filename,
      size,
      start_sector,
      attributes,
      created_time,
      modified_time,
      checksum,
    };

    Ok((entry, offset))
  }
}

/// 文件系统错误类型
#[derive(Debug, Clone, Copy)]
pub enum FsError {
  I2cError,
  InvalidFileSystem,
  FileNotFound,
  FileExists,
  DirectoryFull,
  DiskFull,
  FilenameTooLong,
  BufferTooSmall,
  CorruptedData,
  InternalError,
}

/// 文件系统信息
#[derive(Debug)]
pub struct FileSystemInfo {
  pub total_size: usize,
  pub free_size: usize,
  pub used_size: usize,
  pub file_count: usize,
}

#[entry]
fn main() -> ! {
  // 获取外设句柄
  let dp = pac::Peripherals::take().unwrap();

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

  // 创建文件系统
  let mut fs = EepromFileSystem::new(0x50, 1024); // 1KB EEPROM

  // 启动指示
  startup_sequence(&mut led);

  // 演示文件系统操作
  demonstrate_filesystem(&mut fs, &mut i2c, &mut led);

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

fn demonstrate_filesystem(
  fs: &mut EepromFileSystem,
  i2c: &mut I2cType,
  led: &mut PC13<Output<PushPull>>,
) {
  // 1. 格式化文件系统
  match fs.format(i2c) {
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

  // 2. 挂载文件系统
  match fs.mount(i2c) {
    Ok(_) => {
      led.set_low();
      delay_ms(200);
      led.set_high();
      delay_ms(200);
    }
    Err(_) => {
      error_indication(led);
      return;
    }
  }

  // 3. 创建测试文件
  let test_data = b"Hello, EEPROM File System!";
  match fs.create_file(i2c, "test.txt", test_data) {
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

  // 4. 读取文件
  let mut read_buffer = [0u8; 64];
  match fs.read_file(i2c, "test.txt", &mut read_buffer) {
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

  // 5. 显示文件系统信息
  let fs_info = fs.get_fs_info();

  // 通过LED闪烁显示文件数量
  delay_ms(1000);
  for _ in 0..fs_info.file_count {
    led.set_low();
    delay_ms(200);
    led.set_high();
    delay_ms(200);
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

// 简单延时函数
fn delay_ms(ms: u32) {
  for _ in 0..(ms * 8400) {
    cortex_m::asm::nop();
  }
}
