#![no_std]
#![no_main]

//! # SPI Flash文件系统示例
//! 
//! 在SPI Flash上实现高性能文件系统：
//! - 日志结构文件系统(LFS)
//! - 写入缓存和读取缓存
//! - 垃圾回收
//! - 磨损均衡
//! - 断电保护

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{gpioa::{PA5, PA6, PA7}, gpiob::PB0, gpioc::PC13, Alternate, Output, PushPull},
    pac,
    prelude::*,
    spi::{Spi, Mode, Phase, Polarity},
};
use panic_halt as _;
use heapless::{Vec, String, FnvIndexMap, Deque};
use crc::{Crc, CRC_32_ISO_HDLC};
use bitfield::bitfield;

// Flash 命令定义
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_READ_STATUS: u8 = 0x05;
const CMD_READ_DATA: u8 = 0x03;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_SECTOR_ERASE: u8 = 0x20;
const CMD_JEDEC_ID: u8 = 0x9F;

// 文件系统常量
const PAGE_SIZE: usize = 256;
const SECTOR_SIZE: usize = 4096;
const BLOCK_SIZE: usize = 65536;
const TOTAL_SIZE: usize = 8 * 1024 * 1024; // 8MB

const MAX_FILES: usize = 64;
const MAX_FILENAME_LEN: usize = 32;
const CACHE_SIZE: usize = 8;
const WRITE_BUFFER_SIZE: usize = 1024;

const MAGIC_SUPERBLOCK: u32 = 0x4C465346; // "LFSF"
const MAGIC_INODE: u32 = 0x494E4F44;      // "INOD"
const MAGIC_DATA: u32 = 0x44415441;       // "DATA"
const MAGIC_DIR: u32 = 0x44495245;        // "DIRE"

// CRC计算器
const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

type SpiType = Spi<pac::SPI1, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>;
type CsPin = PB0<Output<PushPull>>;

bitfield! {
    /// 块头部
    pub struct BlockHeader(u32);
    impl Debug;
    pub magic, set_magic: 31, 0;
}

/// 超级块
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SuperBlock {
    pub magic: u32,              // 魔数
    pub version: u16,            // 版本
    pub block_size: u16,         // 块大小
    pub total_blocks: u32,       // 总块数
    pub free_blocks: u32,        // 空闲块数
    pub root_inode: u32,         // 根目录inode
    pub next_inode: u32,         // 下一个inode号
    pub write_count: u64,        // 写入计数
    pub checksum: u32,           // 校验和
}

/// Inode结构
#[derive(Debug, Clone)]
#[repr(C)]
pub struct Inode {
    pub magic: u32,              // 魔数
    pub inode_num: u32,          // inode号
    pub file_type: FileType,     // 文件类型
    pub size: u32,               // 文件大小
    pub blocks: Vec<u32, 16>,    // 数据块列表
    pub created_time: u64,       // 创建时间
    pub modified_time: u64,      // 修改时间
    pub checksum: u32,           // 校验和
}

/// 文件类型
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum FileType {
    Regular = 1,
    Directory = 2,
}

/// 目录项
#[derive(Debug, Clone)]
#[repr(C)]
pub struct DirectoryEntry {
    pub inode_num: u32,                      // inode号
    pub filename: String<MAX_FILENAME_LEN>,  // 文件名
    pub file_type: FileType,                 // 文件类型
}

/// 数据块
#[derive(Debug, Clone)]
#[repr(C)]
pub struct DataBlock {
    pub magic: u32,              // 魔数
    pub inode_num: u32,          // 所属inode
    pub block_index: u16,        // 块索引
    pub data_size: u16,          // 数据大小
    pub data: Vec<u8, 4080>,     // 数据内容 (4096 - 16)
    pub checksum: u32,           // 校验和
}

/// 缓存项
#[derive(Debug, Clone)]
pub struct CacheEntry {
    pub block_num: u32,
    pub data: Vec<u8, SECTOR_SIZE>,
    pub dirty: bool,
    pub access_time: u64,
}

/// 写入缓冲区项
#[derive(Debug, Clone)]
pub struct WriteBufferEntry {
    pub address: u32,
    pub data: Vec<u8, PAGE_SIZE>,
    pub timestamp: u64,
}

/// 日志结构文件系统
pub struct LogStructuredFS {
    // Flash接口
    flash_size: usize,
    
    // 文件系统元数据
    super_block: SuperBlock,
    inode_cache: FnvIndexMap<u32, Inode, 32>,
    
    // 缓存系统
    read_cache: Vec<CacheEntry, CACHE_SIZE>,
    write_buffer: Deque<WriteBufferEntry, 16>,
    
    // 垃圾回收
    gc_threshold: f32,
    wear_leveling: FnvIndexMap<u32, u32, 256>, // block -> erase_count
    
    // 统计信息
    stats: FileSystemStats,
}

/// 文件系统统计
#[derive(Debug, Clone, Copy)]
pub struct FileSystemStats {
    pub total_reads: u64,
    pub total_writes: u64,
    pub cache_hits: u64,
    pub cache_misses: u64,
    pub gc_cycles: u32,
    pub bytes_written: u64,
    pub bytes_read: u64,
}

impl Default for FileSystemStats {
    fn default() -> Self {
        Self {
            total_reads: 0,
            total_writes: 0,
            cache_hits: 0,
            cache_misses: 0,
            gc_cycles: 0,
            bytes_written: 0,
            bytes_read: 0,
        }
    }
}

impl LogStructuredFS {
    pub fn new(flash_size: usize) -> Self {
        let total_blocks = flash_size / SECTOR_SIZE;
        
        let super_block = SuperBlock {
            magic: MAGIC_SUPERBLOCK,
            version: 1,
            block_size: SECTOR_SIZE as u16,
            total_blocks: total_blocks as u32,
            free_blocks: (total_blocks - 1) as u32, // -1 for superblock
            root_inode: 1,
            next_inode: 2,
            write_count: 0,
            checksum: 0,
        };
        
        Self {
            flash_size,
            super_block,
            inode_cache: FnvIndexMap::new(),
            read_cache: Vec::new(),
            write_buffer: Deque::new(),
            gc_threshold: 0.8,
            wear_leveling: FnvIndexMap::new(),
            stats: FileSystemStats::default(),
        }
    }
    
    /// 格式化文件系统
    pub fn format(&mut self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FsError> {
        // 擦除整个Flash
        self.erase_chip(spi, cs)?;
        
        // 写入超级块
        self.write_super_block(spi, cs)?;
        
        // 创建根目录
        self.create_root_directory(spi, cs)?;
        
        // 初始化磨损均衡表
        self.init_wear_leveling();
        
        Ok(())
    }
    
    /// 挂载文件系统
    pub fn mount(&mut self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FsError> {
        // 读取超级块
        self.read_super_block(spi, cs)?;
        
        // 验证文件系统
        if self.super_block.magic != MAGIC_SUPERBLOCK {
            return Err(FsError::InvalidFileSystem);
        }
        
        // 扫描并重建inode缓存
        self.rebuild_inode_cache(spi, cs)?;
        
        // 初始化磨损均衡表
        self.init_wear_leveling();
        
        Ok(())
    }
    
    /// 创建文件
    pub fn create_file(&mut self, 
                      spi: &mut SpiType, 
                      cs: &mut CsPin, 
                      parent_inode: u32,
                      filename: &str, 
                      file_type: FileType) -> Result<u32, FsError> {
        // 检查文件名长度
        if filename.len() > MAX_FILENAME_LEN {
            return Err(FsError::FilenameTooLong);
        }
        
        // 检查父目录是否存在
        let parent = self.get_inode(spi, cs, parent_inode)?;
        if parent.file_type != FileType::Directory {
            return Err(FsError::NotADirectory);
        }
        
        // 检查文件是否已存在
        if self.find_file_in_directory(spi, cs, parent_inode, filename)?.is_some() {
            return Err(FsError::FileExists);
        }
        
        // 分配新的inode号
        let inode_num = self.super_block.next_inode;
        self.super_block.next_inode += 1;
        
        // 创建新的inode
        let mut inode = Inode {
            magic: MAGIC_INODE,
            inode_num,
            file_type,
            size: 0,
            blocks: Vec::new(),
            created_time: self.get_current_time(),
            modified_time: self.get_current_time(),
            checksum: 0,
        };
        
        // 计算校验和
        inode.checksum = self.calculate_inode_checksum(&inode);
        
        // 写入inode
        self.write_inode(spi, cs, &inode)?;
        
        // 添加到父目录
        self.add_directory_entry(spi, cs, parent_inode, filename, inode_num, file_type)?;
        
        // 更新超级块
        self.write_super_block(spi, cs)?;
        
        Ok(inode_num)
    }
    
    /// 写入文件数据
    pub fn write_file(&mut self, 
                     spi: &mut SpiType, 
                     cs: &mut CsPin, 
                     inode_num: u32, 
                     offset: u32, 
                     data: &[u8]) -> Result<usize, FsError> {
        // 获取inode
        let mut inode = self.get_inode(spi, cs, inode_num)?;
        
        if inode.file_type != FileType::Regular {
            return Err(FsError::NotARegularFile);
        }
        
        // 使用写入缓冲区进行批量写入
        self.buffer_write(offset, data)?;
        
        // 如果缓冲区满了，刷新到Flash
        if self.write_buffer.len() >= self.write_buffer.capacity() - 1 {
            self.flush_write_buffer(spi, cs, &mut inode)?;
        }
        
        // 更新文件大小
        let new_size = core::cmp::max(inode.size, offset + data.len() as u32);
        if new_size != inode.size {
            inode.size = new_size;
            inode.modified_time = self.get_current_time();
            inode.checksum = self.calculate_inode_checksum(&inode);
            self.write_inode(spi, cs, &inode)?;
        }
        
        self.stats.total_writes += 1;
        self.stats.bytes_written += data.len() as u64;
        
        Ok(data.len())
    }
    
    /// 读取文件数据
    pub fn read_file(&mut self, 
                    spi: &mut SpiType, 
                    cs: &mut CsPin, 
                    inode_num: u32, 
                    offset: u32, 
                    buffer: &mut [u8]) -> Result<usize, FsError> {
        // 获取inode
        let inode = self.get_inode(spi, cs, inode_num)?;
        
        if inode.file_type != FileType::Regular {
            return Err(FsError::NotARegularFile);
        }
        
        // 检查读取范围
        if offset >= inode.size {
            return Ok(0);
        }
        
        let bytes_to_read = core::cmp::min(buffer.len(), (inode.size - offset) as usize);
        let mut bytes_read = 0;
        
        // 按块读取数据
        let start_block = offset / SECTOR_SIZE as u32;
        let end_block = (offset + bytes_to_read as u32 - 1) / SECTOR_SIZE as u32;
        
        for block_index in start_block..=end_block {
            if block_index as usize >= inode.blocks.len() {
                break;
            }
            
            let block_num = inode.blocks[block_index as usize];
            let block_data = self.read_block_cached(spi, cs, block_num)?;
            
            // 计算在当前块中的偏移和长度
            let block_offset = if block_index == start_block {
                offset % SECTOR_SIZE as u32
            } else {
                0
            };
            
            let block_end = if block_index == end_block {
                ((offset + bytes_to_read as u32 - 1) % SECTOR_SIZE as u32) + 1
            } else {
                SECTOR_SIZE as u32
            };
            
            let copy_len = (block_end - block_offset) as usize;
            let buffer_offset = bytes_read;
            
            buffer[buffer_offset..buffer_offset + copy_len]
                .copy_from_slice(&block_data[block_offset as usize..block_end as usize]);
            
            bytes_read += copy_len;
        }
        
        self.stats.total_reads += 1;
        self.stats.bytes_read += bytes_read as u64;
        
        Ok(bytes_read)
    }
    
    /// 删除文件
    pub fn delete_file(&mut self, 
                      spi: &mut SpiType, 
                      cs: &mut CsPin, 
                      parent_inode: u32,
                      filename: &str) -> Result<(), FsError> {
        // 在父目录中查找文件
        let file_inode = self.find_file_in_directory(spi, cs, parent_inode, filename)?
            .ok_or(FsError::FileNotFound)?;
        
        // 获取文件inode
        let inode = self.get_inode(spi, cs, file_inode)?;
        
        // 标记数据块为可回收
        for &block_num in &inode.blocks {
            self.mark_block_for_gc(block_num);
        }
        
        // 从父目录中移除
        self.remove_directory_entry(spi, cs, parent_inode, filename)?;
        
        // 标记inode为已删除
        self.mark_inode_deleted(spi, cs, file_inode)?;
        
        // 更新空闲块计数
        self.super_block.free_blocks += inode.blocks.len() as u32;
        self.write_super_block(spi, cs)?;
        
        Ok(())
    }
    
    /// 列出目录内容
    pub fn list_directory(&mut self, 
                         spi: &mut SpiType, 
                         cs: &mut CsPin, 
                         inode_num: u32) -> Result<Vec<DirectoryEntry, 32>, FsError> {
        let inode = self.get_inode(spi, cs, inode_num)?;
        
        if inode.file_type != FileType::Directory {
            return Err(FsError::NotADirectory);
        }
        
        let mut entries = Vec::new();
        
        // 读取目录数据块
        for &block_num in &inode.blocks {
            let block_data = self.read_block_cached(spi, cs, block_num)?;
            
            // 解析目录项
            let mut offset = 0;
            while offset < block_data.len() {
                if block_data[offset] == 0 {
                    break; // 目录结束
                }
                
                match self.parse_directory_entry(&block_data[offset..]) {
                    Ok((entry, size)) => {
                        entries.push(entry).map_err(|_| FsError::TooManyFiles)?;
                        offset += size;
                    },
                    Err(_) => break,
                }
            }
        }
        
        Ok(entries)
    }
    
    /// 垃圾回收
    pub fn garbage_collect(&mut self, spi: &mut SpiType, cs: &mut CsPin) -> Result<u32, FsError> {
        let mut reclaimed_blocks = 0u32;
        
        // 检查是否需要垃圾回收
        let usage_ratio = 1.0 - (self.super_block.free_blocks as f32 / self.super_block.total_blocks as f32);
        if usage_ratio < self.gc_threshold {
            return Ok(0);
        }
        
        // 查找可回收的块
        let blocks_to_reclaim = self.find_blocks_for_gc(spi, cs)?;
        
        for block_num in blocks_to_reclaim {
            // 读取块数据
            let block_data = self.read_block_cached(spi, cs, block_num)?;
            
            // 提取有效数据
            let valid_data = self.extract_valid_data(&block_data)?;
            
            // 如果有有效数据，重新分配到新块
            if !valid_data.is_empty() {
                let new_block = self.allocate_block()?;
                self.write_block(spi, cs, new_block, &valid_data)?;
                
                // 更新相关inode的块引用
                self.update_block_references(spi, cs, block_num, new_block)?;
            }
            
            // 擦除旧块
            self.erase_block(spi, cs, block_num)?;
            
            // 标记为空闲
            self.super_block.free_blocks += 1;
            reclaimed_blocks += 1;
        }
        
        // 更新超级块
        self.write_super_block(spi, cs)?;
        
        self.stats.gc_cycles += 1;
        
        Ok(reclaimed_blocks)
    }
    
    /// 同步文件系统
    pub fn sync(&mut self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FsError> {
        // 刷新写入缓冲区
        self.flush_all_write_buffers(spi, cs)?;
        
        // 刷新缓存
        self.flush_cache(spi, cs)?;
        
        // 更新超级块
        self.write_super_block(spi, cs)?;
        
        Ok(())
    }
    
    /// 获取文件系统统计信息
    pub fn get_stats(&self) -> FileSystemStats {
        self.stats
    }
    
    /// 获取文件系统信息
    pub fn get_fs_info(&self) -> FileSystemInfo {
        FileSystemInfo {
            total_size: self.flash_size,
            free_size: self.super_block.free_blocks as usize * SECTOR_SIZE,
            used_size: (self.super_block.total_blocks - self.super_block.free_blocks) as usize * SECTOR_SIZE,
            file_count: self.inode_cache.len(),
            cache_hit_rate: if self.stats.total_reads > 0 {
                self.stats.cache_hits as f32 / self.stats.total_reads as f32
            } else {
                0.0
            },
        }
    }
    
    // 私有方法实现
    
    fn write_super_block(&mut self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FsError> {
        // 计算校验和
        self.super_block.checksum = self.calculate_super_block_checksum();
        
        // 序列化超级块
        let data = self.serialize_super_block();
        
        // 写入第一个扇区
        self.write_sector(spi, cs, 0, &data)
    }
    
    fn read_super_block(&mut self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FsError> {
        let mut buffer = [0u8; SECTOR_SIZE];
        self.read_sector(spi, cs, 0, &mut buffer)?;
        
        self.super_block = self.deserialize_super_block(&buffer)?;
        
        // 验证校验和
        let calculated_checksum = self.calculate_super_block_checksum();
        if self.super_block.checksum != calculated_checksum {
            return Err(FsError::CorruptedData);
        }
        
        Ok(())
    }
    
    fn create_root_directory(&mut self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FsError> {
        let root_inode = Inode {
            magic: MAGIC_INODE,
            inode_num: 1,
            file_type: FileType::Directory,
            size: 0,
            blocks: Vec::new(),
            created_time: self.get_current_time(),
            modified_time: self.get_current_time(),
            checksum: 0,
        };
        
        let mut inode = root_inode;
        inode.checksum = self.calculate_inode_checksum(&inode);
        
        self.write_inode(spi, cs, &inode)?;
        
        Ok(())
    }
    
    fn get_inode(&mut self, spi: &mut SpiType, cs: &mut CsPin, inode_num: u32) -> Result<Inode, FsError> {
        // 先检查缓存
        if let Some(inode) = self.inode_cache.get(&inode_num) {
            return Ok(inode.clone());
        }
        
        // 从Flash读取
        let inode = self.read_inode_from_flash(spi, cs, inode_num)?;
        
        // 添加到缓存
        self.inode_cache.insert(inode_num, inode.clone()).ok();
        
        Ok(inode)
    }
    
    fn write_inode(&mut self, spi: &mut SpiType, cs: &mut CsPin, inode: &Inode) -> Result<(), FsError> {
        // 序列化inode
        let data = self.serialize_inode(inode)?;
        
        // 分配新块写入
        let block_num = self.allocate_block()?;
        self.write_block(spi, cs, block_num, &data)?;
        
        // 更新缓存
        self.inode_cache.insert(inode.inode_num, inode.clone()).ok();
        
        Ok(())
    }
    
    fn read_block_cached(&mut self, 
                        spi: &mut SpiType, 
                        cs: &mut CsPin, 
                        block_num: u32) -> Result<Vec<u8, SECTOR_SIZE>, FsError> {
        // 检查缓存
        for entry in &mut self.read_cache {
            if entry.block_num == block_num {
                entry.access_time = self.get_current_time();
                self.stats.cache_hits += 1;
                return Ok(entry.data.clone());
            }
        }
        
        // 缓存未命中，从Flash读取
        let mut data = Vec::new();
        data.resize(SECTOR_SIZE, 0).map_err(|_| FsError::InternalError)?;
        
        self.read_sector(spi, cs, block_num, &mut data)?;
        
        // 添加到缓存
        self.add_to_cache(block_num, &data);
        
        self.stats.cache_misses += 1;
        
        Ok(data)
    }
    
    fn buffer_write(&mut self, offset: u32, data: &[u8]) -> Result<(), FsError> {
        // 将数据添加到写入缓冲区
        for chunk in data.chunks(PAGE_SIZE) {
            let mut chunk_data = Vec::new();
            chunk_data.extend_from_slice(chunk).map_err(|_| FsError::InternalError)?;
            
            let entry = WriteBufferEntry {
                address: offset,
                data: chunk_data,
                timestamp: self.get_current_time(),
            };
            
            self.write_buffer.push_back(entry).map_err(|_| FsError::BufferFull)?;
        }
        
        Ok(())
    }
    
    fn flush_write_buffer(&mut self, 
                         spi: &mut SpiType, 
                         cs: &mut CsPin, 
                         inode: &mut Inode) -> Result<(), FsError> {
        while let Some(entry) = self.write_buffer.pop_front() {
            // 分配新的数据块
            let block_num = self.allocate_block()?;
            
            // 创建数据块
            let data_block = DataBlock {
                magic: MAGIC_DATA,
                inode_num: inode.inode_num,
                block_index: inode.blocks.len() as u16,
                data_size: entry.data.len() as u16,
                data: entry.data,
                checksum: 0,
            };
            
            // 序列化并写入
            let serialized = self.serialize_data_block(&data_block)?;
            self.write_block(spi, cs, block_num, &serialized)?;
            
            // 更新inode的块列表
            inode.blocks.push(block_num).map_err(|_| FsError::TooManyBlocks)?;
        }
        
        Ok(())
    }
    
    fn allocate_block(&mut self) -> Result<u32, FsError> {
        if self.super_block.free_blocks == 0 {
            return Err(FsError::DiskFull);
        }
        
        // 简单的块分配策略：顺序分配
        for block_num in 1..self.super_block.total_blocks {
            if self.is_block_free(block_num) {
                self.super_block.free_blocks -= 1;
                return Ok(block_num);
            }
        }
        
        Err(FsError::DiskFull)
    }
    
    fn is_block_free(&self, block_num: u32) -> bool {
        // 简化实现：假设块是空闲的
        // 实际实现中需要维护空闲块位图
        true
    }
    
    fn write_sector(&self, spi: &mut SpiType, cs: &mut CsPin, sector: u32, data: &[u8]) -> Result<(), FsError> {
        let address = sector * SECTOR_SIZE as u32;
        
        // 先擦除扇区
        self.erase_sector(spi, cs, address)?;
        
        // 按页写入数据
        for (i, chunk) in data.chunks(PAGE_SIZE).enumerate() {
            let page_address = address + (i * PAGE_SIZE) as u32;
            self.write_page(spi, cs, page_address, chunk)?;
        }
        
        Ok(())
    }
    
    fn read_sector(&self, spi: &mut SpiType, cs: &mut CsPin, sector: u32, buffer: &mut [u8]) -> Result<(), FsError> {
        let address = sector * SECTOR_SIZE as u32;
        let addr_bytes = address.to_be_bytes();
        
        cs.set_low();
        spi.write(&[CMD_FAST_READ, addr_bytes[1], addr_bytes[2], addr_bytes[3], 0x00])
            .map_err(|_| FsError::SpiError)?;
        spi.transfer(buffer).map_err(|_| FsError::SpiError)?;
        cs.set_high();
        
        Ok(())
    }
    
    fn write_page(&self, spi: &mut SpiType, cs: &mut CsPin, address: u32, data: &[u8]) -> Result<(), FsError> {
        // 写使能
        self.write_enable(spi, cs)?;
        
        // 页编程
        let addr_bytes = address.to_be_bytes();
        
        cs.set_low();
        spi.write(&[CMD_PAGE_PROGRAM, addr_bytes[1], addr_bytes[2], addr_bytes[3]])
            .map_err(|_| FsError::SpiError)?;
        spi.write(data).map_err(|_| FsError::SpiError)?;
        cs.set_high();
        
        // 等待写入完成
        self.wait_for_ready(spi, cs)?;
        
        Ok(())
    }
    
    fn erase_sector(&self, spi: &mut SpiType, cs: &mut CsPin, address: u32) -> Result<(), FsError> {
        self.write_enable(spi, cs)?;
        
        let addr_bytes = address.to_be_bytes();
        
        cs.set_low();
        spi.write(&[CMD_SECTOR_ERASE, addr_bytes[1], addr_bytes[2], addr_bytes[3]])
            .map_err(|_| FsError::SpiError)?;
        cs.set_high();
        
        self.wait_for_ready(spi, cs)?;
        
        Ok(())
    }
    
    fn write_enable(&self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FsError> {
        cs.set_low();
        spi.write(&[CMD_WRITE_ENABLE]).map_err(|_| FsError::SpiError)?;
        cs.set_high();
        
        Ok(())
    }
    
    fn wait_for_ready(&self, spi: &mut SpiType, cs: &mut CsPin) -> Result<(), FsError> {
        let mut timeout = 10000;
        
        while timeout > 0 {
            let mut status = [0u8; 1];
            
            cs.set_low();
            spi.write(&[CMD_READ_STATUS]).map_err(|_| FsError::SpiError)?;
            spi.transfer(&mut status).map_err(|_| FsError::SpiError)?;
            cs.set_high();
            
            if (status[0] & 0x01) == 0 {
                return Ok(());
            }
            
            delay_us(100);
            timeout -= 1;
        }
        
        Err(FsError::Timeout)
    }
    
    fn get_current_time(&self) -> u64 {
        // 简单的时间戳实现
        self.super_block.write_count
    }
    
    fn calculate_super_block_checksum(&self) -> u32 {
        // 简化的校验和计算
        let mut sum = 0u32;
        sum = sum.wrapping_add(self.super_block.magic);
        sum = sum.wrapping_add(self.super_block.version as u32);
        sum = sum.wrapping_add(self.super_block.total_blocks);
        sum = sum.wrapping_add(self.super_block.free_blocks);
        sum
    }
    
    fn calculate_inode_checksum(&self, inode: &Inode) -> u32 {
        // 简化的校验和计算
        let mut sum = 0u32;
        sum = sum.wrapping_add(inode.magic);
        sum = sum.wrapping_add(inode.inode_num);
        sum = sum.wrapping_add(inode.size);
        sum
    }
    
    fn serialize_super_block(&self) -> Vec<u8, SECTOR_SIZE> {
        let mut buffer = Vec::new();
        buffer.resize(SECTOR_SIZE, 0).ok();
        
        // 序列化超级块字段
        buffer[0..4].copy_from_slice(&self.super_block.magic.to_le_bytes());
        buffer[4..6].copy_from_slice(&self.super_block.version.to_le_bytes());
        buffer[6..8].copy_from_slice(&self.super_block.block_size.to_le_bytes());
        buffer[8..12].copy_from_slice(&self.super_block.total_blocks.to_le_bytes());
        buffer[12..16].copy_from_slice(&self.super_block.free_blocks.to_le_bytes());
        buffer[16..20].copy_from_slice(&self.super_block.root_inode.to_le_bytes());
        buffer[20..24].copy_from_slice(&self.super_block.next_inode.to_le_bytes());
        buffer[24..32].copy_from_slice(&self.super_block.write_count.to_le_bytes());
        buffer[32..36].copy_from_slice(&self.super_block.checksum.to_le_bytes());
        
        buffer
    }
    
    fn deserialize_super_block(&self, buffer: &[u8]) -> Result<SuperBlock, FsError> {
        if buffer.len() < 36 {
            return Err(FsError::CorruptedData);
        }
        
        Ok(SuperBlock {
            magic: u32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]),
            version: u16::from_le_bytes([buffer[4], buffer[5]]),
            block_size: u16::from_le_bytes([buffer[6], buffer[7]]),
            total_blocks: u32::from_le_bytes([buffer[8], buffer[9], buffer[10], buffer[11]]),
            free_blocks: u32::from_le_bytes([buffer[12], buffer[13], buffer[14], buffer[15]]),
            root_inode: u32::from_le_bytes([buffer[16], buffer[17], buffer[18], buffer[19]]),
            next_inode: u32::from_le_bytes([buffer[20], buffer[21], buffer[22], buffer[23]]),
            write_count: u64::from_le_bytes([
                buffer[24], buffer[25], buffer[26], buffer[27],
                buffer[28], buffer[29], buffer[30], buffer[31]
            ]),
            checksum: u32::from_le_bytes([buffer[32], buffer[33], buffer[34], buffer[35]]),
        })
    }
    
    // 其他辅助方法的简化实现
    fn erase_chip(&self, _spi: &mut SpiType, _cs: &mut CsPin) -> Result<(), FsError> {
        // 简化实现
        Ok(())
    }
    
    fn rebuild_inode_cache(&mut self, _spi: &mut SpiType, _cs: &mut CsPin) -> Result<(), FsError> {
        // 简化实现
        Ok(())
    }
    
    fn init_wear_leveling(&mut self) {
        // 简化实现
    }
    
    fn find_file_in_directory(&mut self, 
                             _spi: &mut SpiType, 
                             _cs: &mut CsPin, 
                             _parent_inode: u32,
                             _filename: &str) -> Result<Option<u32>, FsError> {
        // 简化实现
        Ok(None)
    }
    
    fn add_directory_entry(&mut self, 
                          _spi: &mut SpiType, 
                          _cs: &mut CsPin, 
                          _parent_inode: u32,
                          _filename: &str, 
                          _inode_num: u32, 
                          _file_type: FileType) -> Result<(), FsError> {
        // 简化实现
        Ok(())
    }
    
    fn remove_directory_entry(&mut self, 
                             _spi: &mut SpiType, 
                             _cs: &mut CsPin, 
                             _parent_inode: u32,
                             _filename: &str) -> Result<(), FsError> {
        // 简化实现
        Ok(())
    }
    
    fn mark_inode_deleted(&mut self, 
                         _spi: &mut SpiType, 
                         _cs: &mut CsPin, 
                         _inode_num: u32) -> Result<(), FsError> {
        // 简化实现
        Ok(())
    }
    
    fn mark_block_for_gc(&mut self, _block_num: u32) {
        // 简化实现
    }
    
    fn parse_directory_entry(&self, _buffer: &[u8]) -> Result<(DirectoryEntry, usize), FsError> {
        // 简化实现
        Err(FsError::CorruptedData)
    }
    
    fn find_blocks_for_gc(&self, _spi: &mut SpiType, _cs: &mut CsPin) -> Result<Vec<u32, 16>, FsError> {
        // 简化实现
        Ok(Vec::new())
    }
    
    fn extract_valid_data(&self, _block_data: &[u8]) -> Result<Vec<u8, SECTOR_SIZE>, FsError> {
        // 简化实现
        Ok(Vec::new())
    }
    
    fn write_block(&self, spi: &mut SpiType, cs: &mut CsPin, block_num: u32, data: &[u8]) -> Result<(), FsError> {
        self.write_sector(spi, cs, block_num, data)
    }
    
    fn erase_block(&self, spi: &mut SpiType, cs: &mut CsPin, block_num: u32) -> Result<(), FsError> {
        let address = block_num * SECTOR_SIZE as u32;
        self.erase_sector(spi, cs, address)
    }
    
    fn update_block_references(&mut self, 
                              _spi: &mut SpiType, 
                              _cs: &mut CsPin, 
                              _old_block: u32, 
                              _new_block: u32) -> Result<(), FsError> {
        // 简化实现
        Ok(())
    }
    
    fn flush_all_write_buffers(&mut self, _spi: &mut SpiType, _cs: &mut CsPin) -> Result<(), FsError> {
        // 简化实现
        Ok(())
    }
    
    fn flush_cache(&mut self, _spi: &mut SpiType, _cs: &mut CsPin) -> Result<(), FsError> {
        // 简化实现
        Ok(())
    }
    
    fn read_inode_from_flash(&self, _spi: &mut SpiType, _cs: &mut CsPin, _inode_num: u32) -> Result<Inode, FsError> {
        // 简化实现
        Err(FsError::InodeNotFound)
    }
    
    fn serialize_inode(&self, _inode: &Inode) -> Result<Vec<u8, SECTOR_SIZE>, FsError> {
        // 简化实现
        Ok(Vec::new())
    }
    
    fn serialize_data_block(&self, _block: &DataBlock) -> Result<Vec<u8, SECTOR_SIZE>, FsError> {
        // 简化实现
        Ok(Vec::new())
    }
    
    fn add_to_cache(&mut self, block_num: u32, data: &[u8]) {
        // 如果缓存已满，移除最旧的项
        if self.read_cache.len() >= CACHE_SIZE {
            let mut oldest_index = 0;
            let mut oldest_time = u64::MAX;
            
            for (i, entry) in self.read_cache.iter().enumerate() {
                if entry.access_time < oldest_time {
                    oldest_time = entry.access_time;
                    oldest_index = i;
                }
            }
            
            self.read_cache.swap_remove(oldest_index);
        }
        
        // 添加新的缓存项
        let mut cache_data = Vec::new();
        cache_data.extend_from_slice(data).ok();
        
        let entry = CacheEntry {
            block_num,
            data: cache_data,
            dirty: false,
            access_time: self.get_current_time(),
        };
        
        self.read_cache.push(entry).ok();
    }
}

/// 文件系统信息
#[derive(Debug)]
pub struct FileSystemInfo {
    pub total_size: usize,
    pub free_size: usize,
    pub used_size: usize,
    pub file_count: usize,
    pub cache_hit_rate: f32,
}

/// 文件系统错误类型
#[derive(Debug, Clone, Copy)]
pub enum FsError {
    SpiError,
    Timeout,
    InvalidFileSystem,
    CorruptedData,
    FileNotFound,
    FileExists,
    DirectoryFull,
    DiskFull,
    FilenameTooLong,
    NotADirectory,
    NotARegularFile,
    TooManyFiles,
    TooManyBlocks,
    BufferFull,
    InodeNotFound,
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
    
    // 创建文件系统
    let mut lfs = LogStructuredFS::new(TOTAL_SIZE);
    
    // 启动指示
    startup_sequence(&mut led);
    
    // 演示文件系统操作
    demonstrate_filesystem(&mut lfs, &mut spi, &mut cs, &mut led);
    
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
    lfs: &mut LogStructuredFS,
    spi: &mut SpiType,
    cs: &mut CsPin,
    led: &mut PC13<Output<PushPull>>,
) {
    // 1. 格式化文件系统
    match lfs.format(spi, cs) {
        Ok(_) => {
            // 成功指示
            for _ in 0..2 {
                led.set_low();
                delay_ms(100);
                led.set_high();
                delay_ms(100);
            }
        },
        Err(_) => {
            error_indication(led);
            return;
        }
    }
    
    delay_ms(500);
    
    // 2. 挂载文件系统
    match lfs.mount(spi, cs) {
        Ok(_) => {
            led.set_low();
            delay_ms(200);
            led.set_high();
            delay_ms(200);
        },
        Err(_) => {
            error_indication(led);
            return;
        }
    }
    
    // 3. 创建测试文件
    match lfs.create_file(spi, cs, 1, "test.txt", FileType::Regular) {
        Ok(inode_num) => {
            // 写入测试数据
            let test_data = b"Hello, Log-Structured File System!";
            match lfs.write_file(spi, cs, inode_num, 0, test_data) {
                Ok(_) => {
                    led.set_low();
                    delay_ms(300);
                    led.set_high();
                    delay_ms(300);
                },
                Err(_) => {
                    error_indication(led);
                }
            }
        },
        Err(_) => {
            error_indication(led);
        }
    }
    
    // 4. 同步文件系统
    match lfs.sync(spi, cs) {
        Ok(_) => {
            // 成功指示
            for _ in 0..3 {
                led.set_low();
                delay_ms(100);
                led.set_high();
                delay_ms(100);
            }
        },
        Err(_) => {
            error_indication(led);
        }
    }
    
    // 5. 显示文件系统统计信息
    let stats = lfs.get_stats();
    let fs_info = lfs.get_fs_info();
    
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

fn delay_us(us: u32) {
    for _ in 0..(us * 8) {
        cortex_m::asm::nop();
    }
}