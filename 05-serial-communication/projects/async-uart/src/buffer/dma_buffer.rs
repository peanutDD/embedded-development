//! # DMA缓冲区实现
//!
//! 提供支持DMA传输的高效缓冲区，适用于高速数据传输场景。

use super::{Buffer, ReadableBuffer, WritableBuffer, BufferStats};
use crate::error::{BufferError, DmaError, Result};
use core::{mem, ptr, slice, sync::atomic::{AtomicBool, AtomicUsize, Ordering}};

/// DMA缓冲区配置
#[derive(Debug, Clone)]
pub struct DmaBufferConfig {
    /// 缓冲区大小
    pub size: usize,
    /// 内存对齐要求
    pub alignment: usize,
    /// 是否启用统计
    pub enable_stats: bool,
}

impl Default for DmaBufferConfig {
    fn default() -> Self {
        Self {
            size: 1024,
            alignment: 32, // 通常DMA需要32字节对齐
            enable_stats: true,
        }
    }
}

impl DmaBufferConfig {
    /// 创建新的配置
    pub fn new(size: usize) -> Self {
        Self {
            size,
            ..Default::default()
        }
    }
    
    /// 设置对齐要求
    pub fn alignment(mut self, alignment: usize) -> Self {
        self.alignment = alignment;
        self
    }
    
    /// 禁用统计
    pub fn disable_stats(mut self) -> Self {
        self.enable_stats = false;
        self
    }
    
    /// 验证配置
    pub fn validate(&self) -> Result<()> {
        if self.size == 0 {
            return Err(BufferError::InvalidSize.into());
        }
        
        if self.alignment == 0 || !self.alignment.is_power_of_two() {
            return Err(BufferError::Misaligned.into());
        }
        
        if self.size % self.alignment != 0 {
            return Err(BufferError::Misaligned.into());
        }
        
        Ok(())
    }
}

/// DMA缓冲区状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaBufferState {
    /// 空闲
    Idle,
    /// 准备中
    Preparing,
    /// 传输中
    Transferring,
    /// 完成
    Complete,
    /// 错误
    Error,
}

/// DMA传输方向
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaDirection {
    /// 内存到外设
    MemoryToPeripheral,
    /// 外设到内存
    PeripheralToMemory,
    /// 内存到内存
    MemoryToMemory,
}

/// DMA传输模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaMode {
    /// 单次传输
    Single,
    /// 循环传输
    Circular,
    /// 双缓冲
    DoubleBuffer,
}

/// DMA缓冲区
pub struct DmaBuffer {
    /// 数据存储（对齐的内存）
    data: Vec<u8>,
    /// 缓冲区大小
    size: usize,
    /// 内存对齐
    alignment: usize,
    /// 当前长度
    length: AtomicUsize,
    /// 读取位置
    read_pos: AtomicUsize,
    /// 写入位置
    write_pos: AtomicUsize,
    /// DMA状态
    state: AtomicUsize, // 使用usize存储DmaBufferState
    /// 是否正在传输
    transferring: AtomicBool,
    /// 统计信息
    stats: BufferStats,
    /// 是否启用统计
    enable_stats: bool,
}

impl DmaBuffer {
    /// 创建新的DMA缓冲区
    pub fn new(config: DmaBufferConfig) -> Result<Self> {
        config.validate()?;
        
        // 分配对齐的内存
        let mut data = Vec::with_capacity(config.size + config.alignment);
        data.resize(config.size + config.alignment, 0);
        
        // 确保数据指针对齐
        let ptr = data.as_mut_ptr();
        let aligned_ptr = ((ptr as usize + config.alignment - 1) & !(config.alignment - 1)) as *mut u8;
        let offset = aligned_ptr as usize - ptr as usize;
        
        // 调整Vec以使用对齐的内存
        unsafe {
            data.set_len(config.size + offset);
            ptr::copy(ptr, aligned_ptr, config.size);
            data = Vec::from_raw_parts(aligned_ptr, config.size, config.size);
        }
        
        Ok(Self {
            data,
            size: config.size,
            alignment: config.alignment,
            length: AtomicUsize::new(0),
            read_pos: AtomicUsize::new(0),
            write_pos: AtomicUsize::new(0),
            state: AtomicUsize::new(DmaBufferState::Idle as usize),
            transferring: AtomicBool::new(false),
            stats: BufferStats::new(),
            enable_stats: config.enable_stats,
        })
    }
    
    /// 获取DMA状态
    pub fn state(&self) -> DmaBufferState {
        match self.state.load(Ordering::Acquire) {
            0 => DmaBufferState::Idle,
            1 => DmaBufferState::Preparing,
            2 => DmaBufferState::Transferring,
            3 => DmaBufferState::Complete,
            4 => DmaBufferState::Error,
            _ => DmaBufferState::Error,
        }
    }
    
    /// 设置DMA状态
    pub fn set_state(&self, state: DmaBufferState) {
        self.state.store(state as usize, Ordering::Release);
    }
    
    /// 检查是否正在传输
    pub fn is_transferring(&self) -> bool {
        self.transferring.load(Ordering::Acquire)
    }
    
    /// 设置传输状态
    pub fn set_transferring(&self, transferring: bool) {
        self.transferring.store(transferring, Ordering::Release);
        if transferring {
            self.set_state(DmaBufferState::Transferring);
        }
    }
    
    /// 获取数据指针（用于DMA传输）
    pub fn as_ptr(&self) -> *const u8 {
        self.data.as_ptr()
    }
    
    /// 获取可变数据指针（用于DMA传输）
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.data.as_mut_ptr()
    }
    
    /// 获取数据切片
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.length.load(Ordering::Acquire)]
    }
    
    /// 获取可变数据切片
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        let len = self.length.load(Ordering::Acquire);
        &mut self.data[..len]
    }
    
    /// 准备DMA传输
    pub fn prepare_transfer(&mut self, direction: DmaDirection, mode: DmaMode) -> Result<()> {
        if self.is_transferring() {
            return Err(DmaError::ChannelBusy.into());
        }
        
        self.set_state(DmaBufferState::Preparing);
        
        // 根据传输方向和模式进行准备
        match direction {
            DmaDirection::MemoryToPeripheral => {
                // 确保数据已准备好发送
                if self.len() == 0 {
                    return Err(BufferError::Empty.into());
                }
            }
            DmaDirection::PeripheralToMemory => {
                // 确保有足够空间接收数据
                if self.available() == 0 {
                    return Err(BufferError::Full.into());
                }
            }
            DmaDirection::MemoryToMemory => {
                // 内存到内存传输的特殊处理
            }
        }
        
        // 清除缓存（在实际硬件上需要）
        self.invalidate_cache();
        
        Ok(())
    }
    
    /// 开始DMA传输
    pub fn start_transfer(&mut self) -> Result<()> {
        if self.state() != DmaBufferState::Preparing {
            return Err(DmaError::ConfigError.into());
        }
        
        self.set_transferring(true);
        Ok(())
    }
    
    /// 完成DMA传输
    pub fn complete_transfer(&mut self, bytes_transferred: usize) -> Result<()> {
        if !self.is_transferring() {
            return Err(DmaError::TransferError.into());
        }
        
        self.set_transferring(false);
        self.set_state(DmaBufferState::Complete);
        
        // 更新位置
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let new_write_pos = (write_pos + bytes_transferred).min(self.size);
        self.write_pos.store(new_write_pos, Ordering::Release);
        
        let new_length = self.length.load(Ordering::Acquire) + bytes_transferred;
        self.length.store(new_length.min(self.size), Ordering::Release);
        
        // 更新统计信息
        if self.enable_stats {
            self.update_stats_write(bytes_transferred);
        }
        
        // 刷新缓存（在实际硬件上需要）
        self.flush_cache();
        
        Ok(())
    }
    
    /// 中止DMA传输
    pub fn abort_transfer(&mut self) -> Result<()> {
        if !self.is_transferring() {
            return Ok(());
        }
        
        self.set_transferring(false);
        self.set_state(DmaBufferState::Error);
        
        Ok(())
    }
    
    /// 等待DMA传输完成
    pub async fn wait_for_completion(&self) -> Result<()> {
        // 在实际实现中，这里应该使用适当的异步等待机制
        while self.is_transferring() {
            // 让出CPU时间
            #[cfg(feature = "embassy")]
            embassy_time::Timer::after(embassy_time::Duration::from_micros(10)).await;
            
            #[cfg(not(feature = "embassy"))]
            {
                // 简单的忙等待，实际实现中应该使用更好的方法
                for _ in 0..1000 {
                    core::hint::spin_loop();
                }
            }
        }
        
        match self.state() {
            DmaBufferState::Complete => Ok(()),
            DmaBufferState::Error => Err(DmaError::TransferError.into()),
            _ => Err(DmaError::TransferError.into()),
        }
    }
    
    /// 重置缓冲区
    pub fn reset(&mut self) {
        if self.is_transferring() {
            let _ = self.abort_transfer();
        }
        
        self.length.store(0, Ordering::Release);
        self.read_pos.store(0, Ordering::Release);
        self.write_pos.store(0, Ordering::Release);
        self.set_state(DmaBufferState::Idle);
        
        if self.enable_stats {
            self.stats.reset();
        }
    }
    
    /// 获取内存对齐要求
    pub fn alignment(&self) -> usize {
        self.alignment
    }
    
    /// 检查地址是否对齐
    pub fn is_aligned(&self, addr: usize) -> bool {
        addr % self.alignment == 0
    }
    
    /// 获取对齐的大小
    pub fn align_size(&self, size: usize) -> usize {
        (size + self.alignment - 1) & !(self.alignment - 1)
    }
    
    /// 无效化缓存（平台相关）
    fn invalidate_cache(&self) {
        // 在实际实现中，这里应该调用平台特定的缓存无效化函数
        #[cfg(target_arch = "arm")]
        {
            // ARM Cortex-M缓存无效化
            // unsafe {
            //     cortex_m::asm::dsb();
            //     cortex_m::asm::isb();
            // }
        }
    }
    
    /// 刷新缓存（平台相关）
    fn flush_cache(&self) {
        // 在实际实现中，这里应该调用平台特定的缓存刷新函数
        #[cfg(target_arch = "arm")]
        {
            // ARM Cortex-M缓存刷新
            // unsafe {
            //     cortex_m::asm::dsb();
            //     cortex_m::asm::isb();
            // }
        }
    }
    
    /// 更新写入统计信息
    fn update_stats_write(&mut self, bytes: usize) {
        self.stats.record_write(bytes);
        self.stats.update_usage(self.len());
    }
    
    /// 更新读取统计信息
    fn update_stats_read(&mut self, bytes: usize) {
        self.stats.record_read(bytes);
        self.stats.update_usage(self.len());
    }
    
    /// 创建DMA描述符
    pub fn create_descriptor(&self, direction: DmaDirection) -> DmaDescriptor {
        DmaDescriptor {
            source_addr: match direction {
                DmaDirection::MemoryToPeripheral => self.as_ptr() as u32,
                _ => 0, // 外设地址由HAL层提供
            },
            dest_addr: match direction {
                DmaDirection::PeripheralToMemory => self.as_ptr() as u32,
                _ => 0, // 外设地址由HAL层提供
            },
            transfer_size: self.len() as u32,
            direction,
            mode: DmaMode::Single,
        }
    }
}

impl Buffer for DmaBuffer {
    fn capacity(&self) -> usize {
        self.size
    }
    
    fn len(&self) -> usize {
        self.length.load(Ordering::Acquire)
    }
    
    fn clear(&mut self) {
        self.reset();
    }
    
    fn stats(&self) -> BufferStats {
        self.stats
    }
}

impl ReadableBuffer for DmaBuffer {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        if self.is_transferring() {
            return Err(DmaError::ChannelBusy.into());
        }
        
        let available = self.len();
        if available == 0 {
            if self.enable_stats {
                self.stats.record_underflow();
            }
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let to_read = buf.len().min(available);
        
        unsafe {
            ptr::copy_nonoverlapping(
                self.data.as_ptr().add(read_pos),
                buf.as_mut_ptr(),
                to_read,
            );
        }
        
        // 更新读取位置
        let new_read_pos = read_pos + to_read;
        self.read_pos.store(new_read_pos, Ordering::Release);
        
        // 更新长度
        let new_length = available - to_read;
        self.length.store(new_length, Ordering::Release);
        
        // 如果读取完毕，重置位置
        if new_length == 0 {
            self.read_pos.store(0, Ordering::Release);
            self.write_pos.store(0, Ordering::Release);
        }
        
        if self.enable_stats {
            self.update_stats_read(to_read);
        }
        
        Ok(to_read)
    }
    
    fn read_byte(&mut self) -> Result<u8> {
        let mut buf = [0u8; 1];
        self.read(&mut buf)?;
        Ok(buf[0])
    }
    
    fn peek(&self, buf: &mut [u8]) -> Result<usize> {
        let available = self.len();
        if available == 0 {
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let to_peek = buf.len().min(available);
        
        unsafe {
            ptr::copy_nonoverlapping(
                self.data.as_ptr().add(read_pos),
                buf.as_mut_ptr(),
                to_peek,
            );
        }
        
        Ok(to_peek)
    }
    
    fn peek_byte(&self) -> Result<u8> {
        if self.is_empty() {
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        Ok(self.data[read_pos])
    }
    
    fn skip(&mut self, count: usize) -> Result<usize> {
        let available = self.len();
        let to_skip = count.min(available);
        
        if to_skip > 0 {
            let read_pos = self.read_pos.load(Ordering::Acquire);
            let new_read_pos = read_pos + to_skip;
            self.read_pos.store(new_read_pos, Ordering::Release);
            
            let new_length = available - to_skip;
            self.length.store(new_length, Ordering::Release);
            
            if new_length == 0 {
                self.read_pos.store(0, Ordering::Release);
                self.write_pos.store(0, Ordering::Release);
            }
            
            if self.enable_stats {
                self.update_stats_read(to_skip);
            }
        }
        
        Ok(to_skip)
    }
    
    fn read_until(&mut self, delimiter: u8, buf: &mut [u8]) -> Result<usize> {
        let available = self.len();
        if available == 0 {
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let mut found_pos = None;
        
        // 查找分隔符
        for i in 0..available {
            if self.data[read_pos + i] == delimiter {
                found_pos = Some(i + 1); // 包含分隔符
                break;
            }
        }
        
        let to_read = if let Some(pos) = found_pos {
            pos.min(buf.len())
        } else {
            available.min(buf.len())
        };
        
        // 读取数据
        self.read(&mut buf[..to_read])
    }
}

impl WritableBuffer for DmaBuffer {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        if self.is_transferring() {
            return Err(DmaError::ChannelBusy.into());
        }
        
        let available_space = self.available();
        if available_space == 0 {
            if self.enable_stats {
                self.stats.record_overflow();
            }
            return Err(BufferError::Full.into());
        }
        
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let to_write = buf.len().min(available_space);
        
        unsafe {
            ptr::copy_nonoverlapping(
                buf.as_ptr(),
                self.data.as_mut_ptr().add(write_pos),
                to_write,
            );
        }
        
        // 更新写入位置
        let new_write_pos = write_pos + to_write;
        self.write_pos.store(new_write_pos, Ordering::Release);
        
        // 更新长度
        let new_length = self.length.load(Ordering::Acquire) + to_write;
        self.length.store(new_length, Ordering::Release);
        
        if self.enable_stats {
            self.update_stats_write(to_write);
        }
        
        Ok(to_write)
    }
    
    fn write_byte(&mut self, byte: u8) -> Result<()> {
        let buf = [byte];
        self.write(&buf)?;
        Ok(())
    }
    
    fn flush(&mut self) -> Result<()> {
        // DMA缓冲区的刷新操作
        self.flush_cache();
        Ok(())
    }
    
    fn reserve(&mut self, additional: usize) -> Result<()> {
        if self.available() < additional {
            return Err(BufferError::Full.into());
        }
        Ok(())
    }
}

/// DMA描述符
#[derive(Debug, Clone)]
pub struct DmaDescriptor {
    /// 源地址
    pub source_addr: u32,
    /// 目标地址
    pub dest_addr: u32,
    /// 传输大小
    pub transfer_size: u32,
    /// 传输方向
    pub direction: DmaDirection,
    /// 传输模式
    pub mode: DmaMode,
}

impl DmaDescriptor {
    /// 创建新的DMA描述符
    pub fn new(source_addr: u32, dest_addr: u32, transfer_size: u32, direction: DmaDirection) -> Self {
        Self {
            source_addr,
            dest_addr,
            transfer_size,
            direction,
            mode: DmaMode::Single,
        }
    }
    
    /// 设置传输模式
    pub fn mode(mut self, mode: DmaMode) -> Self {
        self.mode = mode;
        self
    }
    
    /// 验证描述符
    pub fn validate(&self) -> Result<()> {
        if self.transfer_size == 0 {
            return Err(DmaError::SizeError.into());
        }
        
        // 检查地址对齐
        if self.source_addr % 4 != 0 || self.dest_addr % 4 != 0 {
            return Err(DmaError::AlignmentError.into());
        }
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_dma_buffer_creation() {
        let config = DmaBufferConfig::new(1024).alignment(32);
        let buffer = DmaBuffer::new(config).unwrap();
        
        assert_eq!(buffer.capacity(), 1024);
        assert_eq!(buffer.len(), 0);
        assert_eq!(buffer.alignment(), 32);
        assert_eq!(buffer.state(), DmaBufferState::Idle);
        assert!(!buffer.is_transferring());
    }
    
    #[test]
    fn test_dma_buffer_write_read() {
        let config = DmaBufferConfig::new(1024);
        let mut buffer = DmaBuffer::new(config).unwrap();
        
        // 写入数据
        let data = b"Hello, DMA World!";
        let written = buffer.write(data).unwrap();
        assert_eq!(written, data.len());
        assert_eq!(buffer.len(), data.len());
        
        // 读取数据
        let mut read_buf = [0u8; 32];
        let read = buffer.read(&mut read_buf).unwrap();
        assert_eq!(read, data.len());
        assert_eq!(&read_buf[..read], data);
        assert_eq!(buffer.len(), 0);
    }
    
    #[test]
    fn test_dma_buffer_transfer_state() {
        let config = DmaBufferConfig::new(1024);
        let mut buffer = DmaBuffer::new(config).unwrap();
        
        // 准备传输
        buffer.prepare_transfer(DmaDirection::MemoryToPeripheral, DmaMode::Single).unwrap();
        assert_eq!(buffer.state(), DmaBufferState::Preparing);
        
        // 开始传输
        buffer.start_transfer().unwrap();
        assert!(buffer.is_transferring());
        assert_eq!(buffer.state(), DmaBufferState::Transferring);
        
        // 完成传输
        buffer.complete_transfer(100).unwrap();
        assert!(!buffer.is_transferring());
        assert_eq!(buffer.state(), DmaBufferState::Complete);
    }
    
    #[test]
    fn test_dma_buffer_alignment() {
        let config = DmaBufferConfig::new(1024).alignment(64);
        let buffer = DmaBuffer::new(config).unwrap();
        
        assert_eq!(buffer.alignment(), 64);
        assert!(buffer.is_aligned(0));
        assert!(buffer.is_aligned(64));
        assert!(buffer.is_aligned(128));
        assert!(!buffer.is_aligned(63));
        assert!(!buffer.is_aligned(65));
        
        assert_eq!(buffer.align_size(100), 128);
        assert_eq!(buffer.align_size(64), 64);
        assert_eq!(buffer.align_size(65), 128);
    }
    
    #[test]
    fn test_dma_buffer_transfer_busy() {
        let config = DmaBufferConfig::new(1024);
        let mut buffer = DmaBuffer::new(config).unwrap();
        
        // 开始传输
        buffer.set_transferring(true);
        
        // 尝试读写应该失败
        let mut read_buf = [0u8; 10];
        assert!(buffer.read(&mut read_buf).is_err());
        assert!(buffer.write(b"test").is_err());
    }
    
    #[test]
    fn test_dma_descriptor() {
        let desc = DmaDescriptor::new(0x1000, 0x2000, 256, DmaDirection::MemoryToMemory)
            .mode(DmaMode::Circular);
        
        assert_eq!(desc.source_addr, 0x1000);
        assert_eq!(desc.dest_addr, 0x2000);
        assert_eq!(desc.transfer_size, 256);
        assert_eq!(desc.direction, DmaDirection::MemoryToMemory);
        assert_eq!(desc.mode, DmaMode::Circular);
        
        assert!(desc.validate().is_ok());
    }
    
    #[test]
    fn test_dma_descriptor_validation() {
        // 无效的传输大小
        let desc = DmaDescriptor::new(0x1000, 0x2000, 0, DmaDirection::MemoryToMemory);
        assert!(desc.validate().is_err());
        
        // 未对齐的地址
        let desc = DmaDescriptor::new(0x1001, 0x2000, 256, DmaDirection::MemoryToMemory);
        assert!(desc.validate().is_err());
        
        let desc = DmaDescriptor::new(0x1000, 0x2001, 256, DmaDirection::MemoryToMemory);
        assert!(desc.validate().is_err());
    }
    
    #[test]
    fn test_dma_buffer_config_validation() {
        // 有效配置
        let config = DmaBufferConfig::new(1024).alignment(32);
        assert!(config.validate().is_ok());
        
        // 无效大小
        let config = DmaBufferConfig::new(0);
        assert!(config.validate().is_err());
        
        // 无效对齐
        let config = DmaBufferConfig::new(1024).alignment(0);
        assert!(config.validate().is_err());
        
        let config = DmaBufferConfig::new(1024).alignment(3);
        assert!(config.validate().is_err());
        
        // 大小不是对齐的倍数
        let config = DmaBufferConfig::new(1000).alignment(32);
        assert!(config.validate().is_err());
    }
    
    #[test]
    fn test_dma_buffer_reset() {
        let config = DmaBufferConfig::new(1024);
        let mut buffer = DmaBuffer::new(config).unwrap();
        
        // 写入数据并设置状态
        buffer.write(b"test data").unwrap();
        buffer.set_transferring(true);
        
        // 重置缓冲区
        buffer.reset();
        
        assert_eq!(buffer.len(), 0);
        assert_eq!(buffer.state(), DmaBufferState::Idle);
        assert!(!buffer.is_transferring());
    }
}