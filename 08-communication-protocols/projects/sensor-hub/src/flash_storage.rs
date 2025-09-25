//! W25Q64 Flash存储器驱动模块
//! 
//! 支持功能:
//! - 基本读写操作
//! - 扇区和块擦除
//! - 状态查询
//! - 传感器数据存储管理

use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use heapless::Vec;
use crate::sensors::SensorData;

// W25Q64 命令定义
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
const CMD_DEVICE_ID: u8 = 0x90;
const CMD_JEDEC_ID: u8 = 0x9F;

// Flash参数
const PAGE_SIZE: usize = 256;           // 页大小
const SECTOR_SIZE: usize = 4096;        // 扇区大小 (4KB)
const BLOCK_32K_SIZE: usize = 32768;    // 32KB块大小
const BLOCK_64K_SIZE: usize = 65536;    // 64KB块大小
const CHIP_SIZE: usize = 8 * 1024 * 1024; // 芯片总大小 (8MB)

// 状态寄存器位定义
const STATUS_BUSY: u8 = 0x01;
const STATUS_WEL: u8 = 0x02;

// 数据存储配置
const SENSOR_DATA_SIZE: usize = 44;     // 每条传感器记录的大小
const RECORDS_PER_SECTOR: usize = SECTOR_SIZE / SENSOR_DATA_SIZE; // 每扇区记录数
const DATA_START_ADDRESS: u32 = 0x1000; // 数据存储起始地址 (跳过前4KB用于元数据)

#[derive(Debug)]
pub enum FlashError {
    SpiError,
    Timeout,
    InvalidAddress,
    WriteProtected,
    DeviceNotFound,
    BufferFull,
}

// Flash设备信息
#[derive(Debug)]
pub struct FlashInfo {
    pub manufacturer_id: u8,
    pub device_id: u8,
    pub capacity: usize,
    pub page_size: usize,
    pub sector_size: usize,
}

// 数据存储元数据
#[derive(Debug, Clone, Copy)]
pub struct StorageMetadata {
    pub total_records: u32,
    pub current_address: u32,
    pub last_write_time: u32,
    pub checksum: u32,
}

impl StorageMetadata {
    pub fn new() -> Self {
        Self {
            total_records: 0,
            current_address: DATA_START_ADDRESS,
            last_write_time: 0,
            checksum: 0,
        }
    }

    pub fn calculate_checksum(&mut self) {
        let mut sum = 0u32;
        sum = sum.wrapping_add(self.total_records);
        sum = sum.wrapping_add(self.current_address);
        sum = sum.wrapping_add(self.last_write_time);
        self.checksum = sum;
    }

    pub fn verify_checksum(&self) -> bool {
        let mut sum = 0u32;
        sum = sum.wrapping_add(self.total_records);
        sum = sum.wrapping_add(self.current_address);
        sum = sum.wrapping_add(self.last_write_time);
        sum == self.checksum
    }

    pub fn to_bytes(&self) -> [u8; 16] {
        let mut bytes = [0u8; 16];
        bytes[0..4].copy_from_slice(&self.total_records.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.current_address.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.last_write_time.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.checksum.to_le_bytes());
        bytes
    }

    pub fn from_bytes(bytes: &[u8; 16]) -> Self {
        let total_records = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let current_address = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let last_write_time = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        let checksum = u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]);

        Self {
            total_records,
            current_address,
            last_write_time,
            checksum,
        }
    }
}

// W25Q64 Flash存储器驱动
pub struct FlashStorage<SPI, CS> {
    spi: SPI,
    cs: CS,
    info: Option<FlashInfo>,
    metadata: StorageMetadata,
}

impl<SPI, CS, E> FlashStorage<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            spi,
            cs,
            info: None,
            metadata: StorageMetadata::new(),
        }
    }

    pub fn initialize(&mut self) -> Result<(), FlashError> {
        // 释放CS引脚
        let _ = self.cs.set_high();

        // 释放掉电模式
        self.release_power_down()?;

        // 读取设备ID
        let device_info = self.read_device_id()?;
        
        // 验证设备
        if device_info.0 != 0xEF || device_info.1 != 0x16 {
            return Err(FlashError::DeviceNotFound);
        }

        self.info = Some(FlashInfo {
            manufacturer_id: device_info.0,
            device_id: device_info.1,
            capacity: CHIP_SIZE,
            page_size: PAGE_SIZE,
            sector_size: SECTOR_SIZE,
        });

        // 读取存储元数据
        self.load_metadata()?;

        Ok(())
    }

    fn select(&mut self) -> Result<(), FlashError> {
        self.cs.set_low().map_err(|_| FlashError::SpiError)
    }

    fn deselect(&mut self) -> Result<(), FlashError> {
        self.cs.set_high().map_err(|_| FlashError::SpiError)
    }

    fn write_enable(&mut self) -> Result<(), FlashError> {
        self.select()?;
        self.spi.write(&[CMD_WRITE_ENABLE]).map_err(|_| FlashError::SpiError)?;
        self.deselect()?;
        Ok(())
    }

    fn wait_for_ready(&mut self) -> Result<(), FlashError> {
        let mut timeout = 10000;
        while timeout > 0 {
            let status = self.read_status()?;
            if (status & STATUS_BUSY) == 0 {
                return Ok(());
            }
            timeout -= 1;
        }
        Err(FlashError::Timeout)
    }

    fn read_status(&mut self) -> Result<u8, FlashError> {
        self.select()?;
        let mut buffer = [CMD_READ_STATUS, 0x00];
        self.spi.transfer(&mut buffer).map_err(|_| FlashError::SpiError)?;
        self.deselect()?;
        Ok(buffer[1])
    }

    fn read_device_id(&mut self) -> Result<(u8, u8), FlashError> {
        self.select()?;
        let mut buffer = [CMD_DEVICE_ID, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.spi.transfer(&mut buffer).map_err(|_| FlashError::SpiError)?;
        self.deselect()?;
        Ok((buffer[4], buffer[5]))
    }

    fn release_power_down(&mut self) -> Result<(), FlashError> {
        self.select()?;
        self.spi.write(&[CMD_RELEASE_POWER_DOWN]).map_err(|_| FlashError::SpiError)?;
        self.deselect()?;
        
        // 等待设备准备就绪
        delay_us(30);
        Ok(())
    }

    pub fn read_data(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FlashError> {
        if address + buffer.len() as u32 > CHIP_SIZE as u32 {
            return Err(FlashError::InvalidAddress);
        }

        self.wait_for_ready()?;
        self.select()?;

        let cmd = [
            CMD_READ_DATA,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
        ];

        self.spi.write(&cmd).map_err(|_| FlashError::SpiError)?;

        // 读取数据
        for byte in buffer.iter_mut() {
            let mut temp = [0u8];
            self.spi.transfer(&mut temp).map_err(|_| FlashError::SpiError)?;
            *byte = temp[0];
        }

        self.deselect()?;
        Ok(())
    }

    pub fn write_page(&mut self, address: u32, data: &[u8]) -> Result<(), FlashError> {
        if data.len() > PAGE_SIZE {
            return Err(FlashError::InvalidAddress);
        }

        if address + data.len() as u32 > CHIP_SIZE as u32 {
            return Err(FlashError::InvalidAddress);
        }

        self.wait_for_ready()?;
        self.write_enable()?;

        self.select()?;

        let cmd = [
            CMD_PAGE_PROGRAM,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
        ];

        self.spi.write(&cmd).map_err(|_| FlashError::SpiError)?;
        self.spi.write(data).map_err(|_| FlashError::SpiError)?;

        self.deselect()?;
        self.wait_for_ready()?;

        Ok(())
    }

    pub fn erase_sector(&mut self, address: u32) -> Result<(), FlashError> {
        if address >= CHIP_SIZE as u32 {
            return Err(FlashError::InvalidAddress);
        }

        self.wait_for_ready()?;
        self.write_enable()?;

        self.select()?;

        let cmd = [
            CMD_SECTOR_ERASE,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
        ];

        self.spi.write(&cmd).map_err(|_| FlashError::SpiError)?;
        self.deselect()?;
        self.wait_for_ready()?;

        Ok(())
    }

    pub fn erase_block_64k(&mut self, address: u32) -> Result<(), FlashError> {
        if address >= CHIP_SIZE as u32 {
            return Err(FlashError::InvalidAddress);
        }

        self.wait_for_ready()?;
        self.write_enable()?;

        self.select()?;

        let cmd = [
            CMD_BLOCK_ERASE_64K,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
        ];

        self.spi.write(&cmd).map_err(|_| FlashError::SpiError)?;
        self.deselect()?;
        self.wait_for_ready()?;

        Ok(())
    }

    pub fn chip_erase(&mut self) -> Result<(), FlashError> {
        self.wait_for_ready()?;
        self.write_enable()?;

        self.select()?;
        self.spi.write(&[CMD_CHIP_ERASE]).map_err(|_| FlashError::SpiError)?;
        self.deselect()?;

        // 芯片擦除需要更长时间
        let mut timeout = 100000;
        while timeout > 0 {
            let status = self.read_status()?;
            if (status & STATUS_BUSY) == 0 {
                break;
            }
            timeout -= 1;
        }

        if timeout == 0 {
            return Err(FlashError::Timeout);
        }

        // 重置元数据
        self.metadata = StorageMetadata::new();
        self.save_metadata()?;

        Ok(())
    }

    fn load_metadata(&mut self) -> Result<(), FlashError> {
        let mut buffer = [0u8; 16];
        self.read_data(0x0000, &mut buffer)?;

        let metadata = StorageMetadata::from_bytes(&buffer);
        
        if metadata.verify_checksum() {
            self.metadata = metadata;
        } else {
            // 元数据损坏，使用默认值
            self.metadata = StorageMetadata::new();
            self.save_metadata()?;
        }

        Ok(())
    }

    fn save_metadata(&mut self) -> Result<(), FlashError> {
        self.metadata.calculate_checksum();
        let data = self.metadata.to_bytes();
        
        // 擦除第一个扇区的前256字节用于存储元数据
        self.erase_sector(0x0000)?;
        self.write_page(0x0000, &data)?;

        Ok(())
    }

    pub fn write_sensor_data(&mut self, data: &[SensorData]) -> Result<(), FlashError> {
        if data.is_empty() {
            return Ok(());
        }

        for sensor_data in data {
            // 检查是否需要擦除新扇区
            let sector_start = (self.metadata.current_address / SECTOR_SIZE as u32) * SECTOR_SIZE as u32;
            let next_address = self.metadata.current_address + SENSOR_DATA_SIZE as u32;
            let next_sector_start = (next_address / SECTOR_SIZE as u32) * SECTOR_SIZE as u32;

            if next_sector_start != sector_start {
                // 需要擦除新扇区
                self.erase_sector(next_sector_start)?;
            }

            // 写入传感器数据
            let data_bytes = sensor_data.to_bytes();
            
            // 分页写入 (每页256字节)
            let mut offset = 0;
            while offset < data_bytes.len() {
                let page_start = self.metadata.current_address + offset as u32;
                let page_offset = (page_start % PAGE_SIZE as u32) as usize;
                let remaining_in_page = PAGE_SIZE - page_offset;
                let bytes_to_write = core::cmp::min(remaining_in_page, data_bytes.len() - offset);

                self.write_page(
                    page_start,
                    &data_bytes[offset..offset + bytes_to_write]
                )?;

                offset += bytes_to_write;
            }

            // 更新元数据
            self.metadata.current_address += SENSOR_DATA_SIZE as u32;
            self.metadata.total_records += 1;
            self.metadata.last_write_time = get_system_time();
        }

        // 保存元数据
        self.save_metadata()?;

        Ok(())
    }

    pub fn read_sensor_data(&mut self, start_record: u32, count: u32) -> Result<Vec<SensorData, 32>, FlashError> {
        let mut result = Vec::new();

        if start_record >= self.metadata.total_records {
            return Ok(result);
        }

        let actual_count = core::cmp::min(count, self.metadata.total_records - start_record);
        let actual_count = core::cmp::min(actual_count, 32); // 限制返回数量

        for i in 0..actual_count {
            let record_address = DATA_START_ADDRESS + (start_record + i) * SENSOR_DATA_SIZE as u32;
            let mut buffer = [0u8; SENSOR_DATA_SIZE];
            
            self.read_data(record_address, &mut buffer)?;
            let sensor_data = SensorData::from_bytes(&buffer);
            
            if sensor_data.verify_checksum() {
                result.push(sensor_data).map_err(|_| FlashError::BufferFull)?;
            }
        }

        Ok(result)
    }

    pub fn get_storage_info(&self) -> (u32, u32, u32) {
        (
            self.metadata.total_records,
            self.metadata.current_address,
            (CHIP_SIZE as u32 - self.metadata.current_address) / SENSOR_DATA_SIZE as u32
        )
    }

    pub fn get_flash_info(&self) -> Option<&FlashInfo> {
        self.info.as_ref()
    }

    pub fn format_storage(&mut self) -> Result<(), FlashError> {
        // 擦除整个芯片
        self.chip_erase()?;
        
        // 重新初始化元数据
        self.metadata = StorageMetadata::new();
        self.save_metadata()?;

        Ok(())
    }

    pub fn power_down(&mut self) -> Result<(), FlashError> {
        self.wait_for_ready()?;
        self.select()?;
        self.spi.write(&[CMD_POWER_DOWN]).map_err(|_| FlashError::SpiError)?;
        self.deselect()?;
        Ok(())
    }
}

// 延时函数占位符
fn delay_us(_us: u32) {
    // 这里应该实现微秒级延时
    // 可以使用定时器或者简单的循环
    for _ in 0..100 {
        cortex_m::asm::nop();
    }
}

// 获取系统时间的占位函数
fn get_system_time() -> u32 {
    // 这里应该返回实际的系统时间戳
    // 可以使用定时器或RTC来实现
    0
}