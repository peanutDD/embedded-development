//! # 环形缓冲区实现
//!
//! 提供高效的FIFO环形缓冲区，支持并发读写操作。

use super::{Buffer, ReadableBuffer, WritableBuffer, BufferStats};
use crate::error::{BufferError, Result};
use core::sync::atomic::{AtomicUsize, Ordering};
use core::{mem, ptr, slice};

/// 环形缓冲区
pub struct RingBuffer {
    /// 数据存储
    data: Vec<u8>,
    /// 读取位置
    read_pos: AtomicUsize,
    /// 写入位置
    write_pos: AtomicUsize,
    /// 容量（必须是2的幂）
    capacity: usize,
    /// 掩码（用于快速取模）
    mask: usize,
    /// 统计信息
    stats: BufferStats,
}

impl RingBuffer {
    /// 创建新的环形缓冲区
    pub fn new(capacity: usize) -> Result<Self> {
        if capacity == 0 {
            return Err(BufferError::InvalidSize.into());
        }
        
        // 确保容量是2的幂
        let capacity = capacity.next_power_of_two();
        let mask = capacity - 1;
        
        let mut data = Vec::with_capacity(capacity);
        data.resize(capacity, 0);
        
        Ok(Self {
            data,
            read_pos: AtomicUsize::new(0),
            write_pos: AtomicUsize::new(0),
            capacity,
            mask,
            stats: BufferStats::new(),
        })
    }
    
    /// 创建指定容量的环形缓冲区
    pub fn with_capacity(capacity: usize) -> Result<Self> {
        Self::new(capacity)
    }
    
    /// 获取读取位置
    fn read_position(&self) -> usize {
        self.read_pos.load(Ordering::Acquire)
    }
    
    /// 获取写入位置
    fn write_position(&self) -> usize {
        self.write_pos.load(Ordering::Acquire)
    }
    
    /// 设置读取位置
    fn set_read_position(&self, pos: usize) {
        self.read_pos.store(pos, Ordering::Release);
    }
    
    /// 设置写入位置
    fn set_write_position(&self, pos: usize) {
        self.write_pos.store(pos, Ordering::Release);
    }
    
    /// 获取连续可读数据长度
    fn contiguous_readable(&self) -> usize {
        let read_pos = self.read_position();
        let write_pos = self.write_position();
        
        if write_pos >= read_pos {
            write_pos - read_pos
        } else {
            self.capacity - read_pos
        }
    }
    
    /// 获取连续可写空间长度
    fn contiguous_writable(&self) -> usize {
        let read_pos = self.read_position();
        let write_pos = self.write_position();
        
        if read_pos > write_pos {
            read_pos - write_pos - 1
        } else if read_pos == 0 {
            self.capacity - write_pos - 1
        } else {
            self.capacity - write_pos
        }
    }
    
    /// 获取数据指针（读取）
    fn read_ptr(&self) -> *const u8 {
        unsafe { self.data.as_ptr().add(self.read_position() & self.mask) }
    }
    
    /// 获取数据指针（写入）
    fn write_ptr(&mut self) -> *mut u8 {
        unsafe { self.data.as_mut_ptr().add(self.write_position() & self.mask) }
    }
    
    /// 推进读取位置
    fn advance_read(&mut self, count: usize) {
        let new_pos = (self.read_position() + count) & self.mask;
        self.set_read_position(new_pos);
        self.stats.record_read(count);
        self.stats.update_usage(self.len());
    }
    
    /// 推进写入位置
    fn advance_write(&mut self, count: usize) {
        let new_pos = (self.write_position() + count) & self.mask;
        self.set_write_position(new_pos);
        self.stats.record_write(count);
        self.stats.update_usage(self.len());
    }
    
    /// 创建读取器
    pub fn reader(&self) -> RingBufferReader {
        RingBufferReader::new(self)
    }
    
    /// 创建写入器
    pub fn writer(&mut self) -> RingBufferWriter {
        RingBufferWriter::new(self)
    }
    
    /// 分割为读写器
    pub fn split(&mut self) -> (RingBufferReader, RingBufferWriter) {
        // 注意：这里使用了不安全的操作，在实际实现中需要更仔细的处理
        let reader = RingBufferReader::new(unsafe { &*(self as *const Self) });
        let writer = RingBufferWriter::new(self);
        (reader, writer)
    }
    
    /// 重置缓冲区
    pub fn reset(&mut self) {
        self.set_read_position(0);
        self.set_write_position(0);
        self.stats.reset();
    }
    
    /// 获取缓冲区使用情况
    pub fn usage(&self) -> f64 {
        self.len() as f64 / self.capacity as f64
    }
    
    /// 检查是否接近满
    pub fn is_nearly_full(&self, threshold: f64) -> bool {
        self.usage() >= threshold
    }
    
    /// 检查是否接近空
    pub fn is_nearly_empty(&self, threshold: f64) -> bool {
        self.usage() <= threshold
    }
    
    /// 压缩缓冲区（移动数据到开头）
    pub fn compact(&mut self) {
        let read_pos = self.read_position();
        let write_pos = self.write_position();
        let len = self.len();
        
        if read_pos == 0 || len == 0 {
            return;
        }
        
        // 移动数据到缓冲区开头
        if write_pos > read_pos {
            // 数据是连续的
            unsafe {
                ptr::copy(
                    self.data.as_ptr().add(read_pos),
                    self.data.as_mut_ptr(),
                    len,
                );
            }
        } else {
            // 数据是分段的
            let first_part = self.capacity - read_pos;
            let second_part = write_pos;
            
            unsafe {
                // 移动第一部分
                ptr::copy(
                    self.data.as_ptr().add(read_pos),
                    self.data.as_mut_ptr(),
                    first_part,
                );
                // 移动第二部分
                ptr::copy(
                    self.data.as_ptr(),
                    self.data.as_mut_ptr().add(first_part),
                    second_part,
                );
            }
        }
        
        self.set_read_position(0);
        self.set_write_position(len);
    }
    
    /// 获取内部数据的不可变引用（用于调试）
    pub fn as_slice(&self) -> &[u8] {
        &self.data
    }
}

impl Buffer for RingBuffer {
    fn capacity(&self) -> usize {
        self.capacity - 1 // 保留一个位置用于区分满和空
    }
    
    fn len(&self) -> usize {
        let read_pos = self.read_position();
        let write_pos = self.write_position();
        
        if write_pos >= read_pos {
            write_pos - read_pos
        } else {
            self.capacity - read_pos + write_pos
        }
    }
    
    fn clear(&mut self) {
        self.reset();
    }
    
    fn stats(&self) -> BufferStats {
        self.stats
    }
}

impl ReadableBuffer for RingBuffer {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        let available = self.len();
        if available == 0 {
            self.stats.record_underflow();
            return Err(BufferError::Empty.into());
        }
        
        let to_read = buf.len().min(available);
        let read_pos = self.read_position();
        
        // 检查是否需要分两次读取
        let first_part = (self.capacity - read_pos).min(to_read);
        let second_part = to_read - first_part;
        
        unsafe {
            // 读取第一部分
            ptr::copy_nonoverlapping(
                self.data.as_ptr().add(read_pos),
                buf.as_mut_ptr(),
                first_part,
            );
            
            // 如果需要，读取第二部分
            if second_part > 0 {
                ptr::copy_nonoverlapping(
                    self.data.as_ptr(),
                    buf.as_mut_ptr().add(first_part),
                    second_part,
                );
            }
        }
        
        self.advance_read(to_read);
        Ok(to_read)
    }
    
    fn read_byte(&mut self) -> Result<u8> {
        if self.is_empty() {
            self.stats.record_underflow();
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_position();
        let byte = self.data[read_pos];
        self.advance_read(1);
        Ok(byte)
    }
    
    fn peek(&self, buf: &mut [u8]) -> Result<usize> {
        let available = self.len();
        if available == 0 {
            return Err(BufferError::Empty.into());
        }
        
        let to_peek = buf.len().min(available);
        let read_pos = self.read_position();
        
        // 检查是否需要分两次读取
        let first_part = (self.capacity - read_pos).min(to_peek);
        let second_part = to_peek - first_part;
        
        unsafe {
            // 读取第一部分
            ptr::copy_nonoverlapping(
                self.data.as_ptr().add(read_pos),
                buf.as_mut_ptr(),
                first_part,
            );
            
            // 如果需要，读取第二部分
            if second_part > 0 {
                ptr::copy_nonoverlapping(
                    self.data.as_ptr(),
                    buf.as_mut_ptr().add(first_part),
                    second_part,
                );
            }
        }
        
        Ok(to_peek)
    }
    
    fn peek_byte(&self) -> Result<u8> {
        if self.is_empty() {
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_position();
        Ok(self.data[read_pos])
    }
    
    fn skip(&mut self, count: usize) -> Result<usize> {
        let available = self.len();
        let to_skip = count.min(available);
        
        if to_skip > 0 {
            self.advance_read(to_skip);
        }
        
        Ok(to_skip)
    }
    
    fn read_until(&mut self, delimiter: u8, buf: &mut [u8]) -> Result<usize> {
        let available = self.len();
        if available == 0 {
            return Err(BufferError::Empty.into());
        }
        
        let mut found_pos = None;
        let read_pos = self.read_position();
        
        // 在第一部分中查找分隔符
        let first_part = (self.capacity - read_pos).min(available);
        for i in 0..first_part {
            if self.data[read_pos + i] == delimiter {
                found_pos = Some(i + 1); // 包含分隔符
                break;
            }
        }
        
        // 如果在第一部分没找到，在第二部分中查找
        if found_pos.is_none() && available > first_part {
            let second_part = available - first_part;
            for i in 0..second_part {
                if self.data[i] == delimiter {
                    found_pos = Some(first_part + i + 1); // 包含分隔符
                    break;
                }
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

impl WritableBuffer for RingBuffer {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        let available_space = self.available();
        if available_space == 0 {
            self.stats.record_overflow();
            return Err(BufferError::Full.into());
        }
        
        let to_write = buf.len().min(available_space);
        let write_pos = self.write_position();
        
        // 检查是否需要分两次写入
        let first_part = (self.capacity - write_pos).min(to_write);
        let second_part = to_write - first_part;
        
        unsafe {
            // 写入第一部分
            ptr::copy_nonoverlapping(
                buf.as_ptr(),
                self.data.as_mut_ptr().add(write_pos),
                first_part,
            );
            
            // 如果需要，写入第二部分
            if second_part > 0 {
                ptr::copy_nonoverlapping(
                    buf.as_ptr().add(first_part),
                    self.data.as_mut_ptr(),
                    second_part,
                );
            }
        }
        
        self.advance_write(to_write);
        Ok(to_write)
    }
    
    fn write_byte(&mut self, byte: u8) -> Result<()> {
        if self.available() == 0 {
            self.stats.record_overflow();
            return Err(BufferError::Full.into());
        }
        
        let write_pos = self.write_position();
        self.data[write_pos] = byte;
        self.advance_write(1);
        Ok(())
    }
    
    fn flush(&mut self) -> Result<()> {
        // 环形缓冲区不需要刷新操作
        Ok(())
    }
    
    fn reserve(&mut self, additional: usize) -> Result<()> {
        if self.available() < additional {
            return Err(BufferError::Full.into());
        }
        Ok(())
    }
}

/// 环形缓冲区读取器
pub struct RingBufferReader<'a> {
    buffer: &'a RingBuffer,
}

impl<'a> RingBufferReader<'a> {
    fn new(buffer: &'a RingBuffer) -> Self {
        Self { buffer }
    }
    
    /// 读取数据
    pub fn read(&self, buf: &mut [u8]) -> Result<usize> {
        // 注意：这里需要更仔细的并发控制
        // 在实际实现中，应该使用适当的同步机制
        let available = self.buffer.len();
        if available == 0 {
            return Err(BufferError::Empty.into());
        }
        
        let to_read = buf.len().min(available);
        let read_pos = self.buffer.read_position();
        
        // 检查是否需要分两次读取
        let first_part = (self.buffer.capacity - read_pos).min(to_read);
        let second_part = to_read - first_part;
        
        unsafe {
            // 读取第一部分
            ptr::copy_nonoverlapping(
                self.buffer.data.as_ptr().add(read_pos),
                buf.as_mut_ptr(),
                first_part,
            );
            
            // 如果需要，读取第二部分
            if second_part > 0 {
                ptr::copy_nonoverlapping(
                    self.buffer.data.as_ptr(),
                    buf.as_mut_ptr().add(first_part),
                    second_part,
                );
            }
        }
        
        // 更新读取位置
        let new_pos = (read_pos + to_read) & self.buffer.mask;
        self.buffer.set_read_position(new_pos);
        
        Ok(to_read)
    }
    
    /// 窥视数据
    pub fn peek(&self, buf: &mut [u8]) -> Result<usize> {
        let available = self.buffer.len();
        if available == 0 {
            return Err(BufferError::Empty.into());
        }
        
        let to_peek = buf.len().min(available);
        let read_pos = self.buffer.read_position();
        
        // 检查是否需要分两次读取
        let first_part = (self.buffer.capacity - read_pos).min(to_peek);
        let second_part = to_peek - first_part;
        
        unsafe {
            // 读取第一部分
            ptr::copy_nonoverlapping(
                self.buffer.data.as_ptr().add(read_pos),
                buf.as_mut_ptr(),
                first_part,
            );
            
            // 如果需要，读取第二部分
            if second_part > 0 {
                ptr::copy_nonoverlapping(
                    self.buffer.data.as_ptr(),
                    buf.as_mut_ptr().add(first_part),
                    second_part,
                );
            }
        }
        
        Ok(to_peek)
    }
}

/// 环形缓冲区写入器
pub struct RingBufferWriter<'a> {
    buffer: &'a mut RingBuffer,
}

impl<'a> RingBufferWriter<'a> {
    fn new(buffer: &'a mut RingBuffer) -> Self {
        Self { buffer }
    }
    
    /// 写入数据
    pub fn write(&mut self, buf: &[u8]) -> Result<usize> {
        self.buffer.write(buf)
    }
    
    /// 写入单个字节
    pub fn write_byte(&mut self, byte: u8) -> Result<()> {
        self.buffer.write_byte(byte)
    }
    
    /// 获取可用空间
    pub fn available(&self) -> usize {
        self.buffer.available()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_ring_buffer_creation() {
        let buffer = RingBuffer::new(16).unwrap();
        assert_eq!(buffer.capacity(), 15); // 实际容量是16-1
        assert_eq!(buffer.len(), 0);
        assert!(buffer.is_empty());
        assert!(!buffer.is_full());
    }
    
    #[test]
    fn test_ring_buffer_write_read() {
        let mut buffer = RingBuffer::new(16).unwrap();
        
        // 写入数据
        let data = b"Hello, World!";
        let written = buffer.write(data).unwrap();
        assert_eq!(written, data.len());
        assert_eq!(buffer.len(), data.len());
        
        // 读取数据
        let mut read_buf = [0u8; 20];
        let read = buffer.read(&mut read_buf).unwrap();
        assert_eq!(read, data.len());
        assert_eq!(&read_buf[..read], data);
        assert_eq!(buffer.len(), 0);
    }
    
    #[test]
    fn test_ring_buffer_wrap_around() {
        let mut buffer = RingBuffer::new(8).unwrap();
        
        // 写入数据直到接近满
        let data1 = b"12345";
        buffer.write(data1).unwrap();
        
        // 读取部分数据
        let mut read_buf = [0u8; 3];
        buffer.read(&mut read_buf).unwrap();
        assert_eq!(&read_buf, b"123");
        
        // 写入更多数据，触发环绕
        let data2 = b"6789";
        buffer.write(data2).unwrap();
        
        // 读取所有剩余数据
        let mut read_buf = [0u8; 10];
        let read = buffer.read(&mut read_buf).unwrap();
        assert_eq!(&read_buf[..read], b"456789");
    }
    
    #[test]
    fn test_ring_buffer_full() {
        let mut buffer = RingBuffer::new(4).unwrap();
        
        // 填满缓冲区（容量是3）
        let data = b"123";
        buffer.write(data).unwrap();
        assert!(buffer.is_full());
        
        // 尝试写入更多数据应该失败
        let result = buffer.write(b"4");
        assert!(result.is_err());
    }
    
    #[test]
    fn test_ring_buffer_empty() {
        let mut buffer = RingBuffer::new(8).unwrap();
        
        // 尝试从空缓冲区读取应该失败
        let mut read_buf = [0u8; 5];
        let result = buffer.read(&mut read_buf);
        assert!(result.is_err());
        
        // 尝试读取单个字节也应该失败
        let result = buffer.read_byte();
        assert!(result.is_err());
    }
    
    #[test]
    fn test_ring_buffer_peek() {
        let mut buffer = RingBuffer::new(16).unwrap();
        
        let data = b"Hello";
        buffer.write(data).unwrap();
        
        // 窥视数据
        let mut peek_buf = [0u8; 10];
        let peeked = buffer.peek(&mut peek_buf).unwrap();
        assert_eq!(peeked, data.len());
        assert_eq!(&peek_buf[..peeked], data);
        
        // 缓冲区长度应该不变
        assert_eq!(buffer.len(), data.len());
        
        // 窥视单个字节
        let byte = buffer.peek_byte().unwrap();
        assert_eq!(byte, b'H');
        assert_eq!(buffer.len(), data.len());
    }
    
    #[test]
    fn test_ring_buffer_skip() {
        let mut buffer = RingBuffer::new(16).unwrap();
        
        let data = b"Hello, World!";
        buffer.write(data).unwrap();
        
        // 跳过前5个字节
        let skipped = buffer.skip(5).unwrap();
        assert_eq!(skipped, 5);
        assert_eq!(buffer.len(), data.len() - 5);
        
        // 读取剩余数据
        let mut read_buf = [0u8; 20];
        let read = buffer.read(&mut read_buf).unwrap();
        assert_eq!(&read_buf[..read], b", World!");
    }
    
    #[test]
    fn test_ring_buffer_read_until() {
        let mut buffer = RingBuffer::new(32).unwrap();
        
        let data = b"Hello\nWorld\nTest";
        buffer.write(data).unwrap();
        
        // 读取到第一个换行符
        let mut read_buf = [0u8; 20];
        let read = buffer.read_until(b'\n', &mut read_buf).unwrap();
        assert_eq!(&read_buf[..read], b"Hello\n");
        
        // 读取到第二个换行符
        let read = buffer.read_until(b'\n', &mut read_buf).unwrap();
        assert_eq!(&read_buf[..read], b"World\n");
        
        // 读取剩余数据（没有找到分隔符）
        let read = buffer.read_until(b'\n', &mut read_buf).unwrap();
        assert_eq!(&read_buf[..read], b"Test");
    }
    
    #[test]
    fn test_ring_buffer_compact() {
        let mut buffer = RingBuffer::new(16).unwrap();
        
        // 写入数据
        buffer.write(b"12345").unwrap();
        
        // 读取部分数据
        let mut read_buf = [0u8; 3];
        buffer.read(&mut read_buf).unwrap();
        
        // 写入更多数据
        buffer.write(b"6789").unwrap();
        
        // 压缩缓冲区
        buffer.compact();
        
        // 验证数据仍然正确
        let mut read_buf = [0u8; 10];
        let read = buffer.read(&mut read_buf).unwrap();
        assert_eq!(&read_buf[..read], b"456789");
    }
    
    #[test]
    fn test_ring_buffer_stats() {
        let mut buffer = RingBuffer::new(16).unwrap();
        
        // 写入数据
        buffer.write(b"Hello").unwrap();
        let stats = buffer.stats();
        assert_eq!(stats.bytes_written, 5);
        assert_eq!(stats.write_operations, 1);
        
        // 读取数据
        let mut read_buf = [0u8; 10];
        buffer.read(&mut read_buf).unwrap();
        let stats = buffer.stats();
        assert_eq!(stats.bytes_read, 5);
        assert_eq!(stats.read_operations, 1);
    }
    
    #[test]
    fn test_ring_buffer_usage() {
        let mut buffer = RingBuffer::new(16).unwrap();
        
        assert_eq!(buffer.usage(), 0.0);
        assert!(buffer.is_nearly_empty(0.1));
        assert!(!buffer.is_nearly_full(0.9));
        
        // 写入数据到一半
        buffer.write(b"1234567").unwrap(); // 7 bytes out of 15 capacity
        let usage = buffer.usage();
        assert!(usage > 0.4 && usage < 0.5);
        
        // 写入更多数据
        buffer.write(b"890123456").unwrap(); // Total 16 bytes, but capacity is 15
        assert!(buffer.is_nearly_full(0.9));
    }
}