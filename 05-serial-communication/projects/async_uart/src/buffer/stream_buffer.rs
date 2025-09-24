//! # 流式缓冲区实现
//!
//! 提供支持连续数据流处理的缓冲区，适用于实时数据处理场景。

use super::{Buffer, ReadableBuffer, WritableBuffer, BufferStats, OverflowStrategy};
use crate::error::{BufferError, Result};
use core::{
    mem,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    task::{Context, Poll, Waker},
};
use futures_util::{Stream, Sink};
use pin_project_lite::pin_project;

/// 流式缓冲区配置
#[derive(Debug, Clone)]
pub struct StreamBufferConfig {
    /// 缓冲区大小
    pub size: usize,
    /// 溢出策略
    pub overflow_strategy: OverflowStrategy,
    /// 低水位标记（字节数）
    pub low_watermark: usize,
    /// 高水位标记（字节数）
    pub high_watermark: usize,
    /// 是否启用背压
    pub enable_backpressure: bool,
    /// 是否启用统计
    pub enable_stats: bool,
    /// 块大小（用于批量处理）
    pub chunk_size: usize,
}

impl Default for StreamBufferConfig {
    fn default() -> Self {
        Self {
            size: 4096,
            overflow_strategy: OverflowStrategy::Block,
            low_watermark: 1024,
            high_watermark: 3072,
            enable_backpressure: true,
            enable_stats: true,
            chunk_size: 256,
        }
    }
}

impl StreamBufferConfig {
    /// 创建新的配置
    pub fn new(size: usize) -> Self {
        let low_watermark = size / 4;
        let high_watermark = size * 3 / 4;
        
        Self {
            size,
            low_watermark,
            high_watermark,
            ..Default::default()
        }
    }
    
    /// 设置水位标记
    pub fn watermarks(mut self, low: usize, high: usize) -> Self {
        self.low_watermark = low;
        self.high_watermark = high;
        self
    }
    
    /// 设置溢出策略
    pub fn overflow_strategy(mut self, strategy: OverflowStrategy) -> Self {
        self.overflow_strategy = strategy;
        self
    }
    
    /// 设置块大小
    pub fn chunk_size(mut self, size: usize) -> Self {
        self.chunk_size = size;
        self
    }
    
    /// 禁用背压
    pub fn disable_backpressure(mut self) -> Self {
        self.enable_backpressure = false;
        self
    }
    
    /// 验证配置
    pub fn validate(&self) -> Result<()> {
        if self.size == 0 {
            return Err(BufferError::InvalidSize.into());
        }
        
        if self.low_watermark >= self.high_watermark {
            return Err(BufferError::InvalidConfiguration.into());
        }
        
        if self.high_watermark >= self.size {
            return Err(BufferError::InvalidConfiguration.into());
        }
        
        if self.chunk_size == 0 || self.chunk_size > self.size {
            return Err(BufferError::InvalidConfiguration.into());
        }
        
        Ok(())
    }
}

/// 流状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamState {
    /// 正常流动
    Flowing,
    /// 暂停（背压）
    Paused,
    /// 已关闭
    Closed,
    /// 错误状态
    Error,
}

/// 流式缓冲区
pub struct StreamBuffer {
    /// 数据存储
    data: Vec<u8>,
    /// 缓冲区大小
    size: usize,
    /// 读取位置
    read_pos: AtomicUsize,
    /// 写入位置
    write_pos: AtomicUsize,
    /// 当前长度
    length: AtomicUsize,
    /// 流状态
    state: AtomicUsize, // 使用usize存储StreamState
    /// 配置
    config: StreamBufferConfig,
    /// 统计信息
    stats: BufferStats,
    /// 等待读取的任务
    read_waker: Option<Waker>,
    /// 等待写入的任务
    write_waker: Option<Waker>,
    /// 是否已关闭
    closed: AtomicBool,
}

impl StreamBuffer {
    /// 创建新的流式缓冲区
    pub fn new(config: StreamBufferConfig) -> Result<Self> {
        config.validate()?;
        
        Ok(Self {
            data: vec![0; config.size],
            size: config.size,
            read_pos: AtomicUsize::new(0),
            write_pos: AtomicUsize::new(0),
            length: AtomicUsize::new(0),
            state: AtomicUsize::new(StreamState::Flowing as usize),
            config,
            stats: BufferStats::new(),
            read_waker: None,
            write_waker: None,
            closed: AtomicBool::new(false),
        })
    }
    
    /// 获取流状态
    pub fn state(&self) -> StreamState {
        match self.state.load(Ordering::Acquire) {
            0 => StreamState::Flowing,
            1 => StreamState::Paused,
            2 => StreamState::Closed,
            3 => StreamState::Error,
            _ => StreamState::Error,
        }
    }
    
    /// 设置流状态
    pub fn set_state(&self, state: StreamState) {
        self.state.store(state as usize, Ordering::Release);
    }
    
    /// 检查是否已关闭
    pub fn is_closed(&self) -> bool {
        self.closed.load(Ordering::Acquire)
    }
    
    /// 关闭流
    pub fn close(&mut self) {
        self.closed.store(true, Ordering::Release);
        self.set_state(StreamState::Closed);
        
        // 唤醒等待的任务
        if let Some(waker) = self.read_waker.take() {
            waker.wake();
        }
        if let Some(waker) = self.write_waker.take() {
            waker.wake();
        }
    }
    
    /// 检查是否应该暂停（背压）
    pub fn should_pause(&self) -> bool {
        if !self.config.enable_backpressure {
            return false;
        }
        
        self.len() >= self.config.high_watermark
    }
    
    /// 检查是否应该恢复
    pub fn should_resume(&self) -> bool {
        if !self.config.enable_backpressure {
            return true;
        }
        
        self.len() <= self.config.low_watermark
    }
    
    /// 更新流状态
    pub fn update_flow_state(&mut self) {
        if self.is_closed() {
            return;
        }
        
        let current_state = self.state();
        
        match current_state {
            StreamState::Flowing => {
                if self.should_pause() {
                    self.set_state(StreamState::Paused);
                    if self.config.enable_stats {
                        self.stats.record_backpressure();
                    }
                }
            }
            StreamState::Paused => {
                if self.should_resume() {
                    self.set_state(StreamState::Flowing);
                    // 唤醒等待写入的任务
                    if let Some(waker) = self.write_waker.take() {
                        waker.wake();
                    }
                }
            }
            _ => {}
        }
    }
    
    /// 异步读取数据
    pub fn poll_read(&mut self, cx: &mut Context<'_>, buf: &mut [u8]) -> Poll<Result<usize>> {
        if self.is_closed() && self.is_empty() {
            return Poll::Ready(Ok(0));
        }
        
        if self.is_empty() {
            // 保存waker以便稍后唤醒
            self.read_waker = Some(cx.waker().clone());
            return Poll::Pending;
        }
        
        match self.read(buf) {
            Ok(n) => {
                self.update_flow_state();
                Poll::Ready(Ok(n))
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }
    
    /// 异步写入数据
    pub fn poll_write(&mut self, cx: &mut Context<'_>, buf: &[u8]) -> Poll<Result<usize>> {
        if self.is_closed() {
            return Poll::Ready(Err(BufferError::Closed.into()));
        }
        
        // 检查背压
        if self.state() == StreamState::Paused {
            self.write_waker = Some(cx.waker().clone());
            return Poll::Pending;
        }
        
        match self.write(buf) {
            Ok(n) => {
                self.update_flow_state();
                // 唤醒等待读取的任务
                if let Some(waker) = self.read_waker.take() {
                    waker.wake();
                }
                Poll::Ready(Ok(n))
            }
            Err(BufferError::Full) => {
                // 根据溢出策略处理
                match self.config.overflow_strategy {
                    OverflowStrategy::Block => {
                        self.write_waker = Some(cx.waker().clone());
                        Poll::Pending
                    }
                    OverflowStrategy::DropOldest => {
                        // 丢弃最旧的数据
                        let to_drop = buf.len().min(self.len());
                        let _ = self.skip(to_drop);
                        self.write(buf).map(Poll::Ready).unwrap_or(Poll::Pending)
                    }
                    OverflowStrategy::DropNewest => {
                        // 丢弃新数据
                        if self.config.enable_stats {
                            self.stats.record_overflow();
                        }
                        Poll::Ready(Ok(0))
                    }
                    OverflowStrategy::Error => {
                        Poll::Ready(Err(BufferError::Full.into()))
                    }
                }
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }
    
    /// 异步刷新
    pub fn poll_flush(&mut self, _cx: &mut Context<'_>) -> Poll<Result<()>> {
        // 流式缓冲区的刷新操作
        Poll::Ready(Ok(()))
    }
    
    /// 异步关闭
    pub fn poll_close(&mut self, _cx: &mut Context<'_>) -> Poll<Result<()>> {
        self.close();
        Poll::Ready(Ok(()))
    }
    
    /// 读取块数据
    pub fn read_chunk(&mut self) -> Result<Vec<u8>> {
        let chunk_size = self.config.chunk_size.min(self.len());
        if chunk_size == 0 {
            return Ok(Vec::new());
        }
        
        let mut chunk = vec![0u8; chunk_size];
        let read = self.read(&mut chunk)?;
        chunk.truncate(read);
        Ok(chunk)
    }
    
    /// 写入块数据
    pub fn write_chunk(&mut self, chunk: &[u8]) -> Result<usize> {
        self.write(chunk)
    }
    
    /// 获取可读的连续数据切片
    pub fn readable_slice(&self) -> &[u8] {
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let len = self.len();
        
        if len == 0 {
            return &[];
        }
        
        if read_pos < write_pos {
            // 数据是连续的
            &self.data[read_pos..write_pos]
        } else {
            // 数据环绕，返回第一部分
            &self.data[read_pos..]
        }
    }
    
    /// 获取可写的连续空间切片
    pub fn writable_slice(&mut self) -> &mut [u8] {
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let available = self.available();
        
        if available == 0 {
            return &mut [];
        }
        
        if write_pos < read_pos {
            // 可写空间是连续的
            &mut self.data[write_pos..read_pos]
        } else {
            // 可写空间在末尾
            let end_space = self.size - write_pos;
            if end_space > 0 {
                &mut self.data[write_pos..]
            } else {
                &mut self.data[..read_pos]
            }
        }
    }
    
    /// 提交写入的字节数
    pub fn commit_write(&mut self, bytes: usize) -> Result<()> {
        let available = self.available();
        if bytes > available {
            return Err(BufferError::InvalidSize.into());
        }
        
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let new_write_pos = (write_pos + bytes) % self.size;
        self.write_pos.store(new_write_pos, Ordering::Release);
        
        let new_length = self.length.load(Ordering::Acquire) + bytes;
        self.length.store(new_length, Ordering::Release);
        
        if self.config.enable_stats {
            self.stats.record_write(bytes);
            self.stats.update_usage(self.len());
        }
        
        self.update_flow_state();
        
        // 唤醒等待读取的任务
        if let Some(waker) = self.read_waker.take() {
            waker.wake();
        }
        
        Ok(())
    }
    
    /// 提交读取的字节数
    pub fn commit_read(&mut self, bytes: usize) -> Result<()> {
        let len = self.len();
        if bytes > len {
            return Err(BufferError::InvalidSize.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let new_read_pos = (read_pos + bytes) % self.size;
        self.read_pos.store(new_read_pos, Ordering::Release);
        
        let new_length = len - bytes;
        self.length.store(new_length, Ordering::Release);
        
        if self.config.enable_stats {
            self.stats.record_read(bytes);
            self.stats.update_usage(self.len());
        }
        
        self.update_flow_state();
        
        Ok(())
    }
    
    /// 获取配置
    pub fn config(&self) -> &StreamBufferConfig {
        &self.config
    }
    
    /// 获取水位状态
    pub fn watermark_status(&self) -> (bool, bool) {
        let len = self.len();
        (
            len <= self.config.low_watermark,  // 低水位
            len >= self.config.high_watermark, // 高水位
        )
    }
}

impl Buffer for StreamBuffer {
    fn capacity(&self) -> usize {
        self.size
    }
    
    fn len(&self) -> usize {
        self.length.load(Ordering::Acquire)
    }
    
    fn clear(&mut self) {
        self.read_pos.store(0, Ordering::Release);
        self.write_pos.store(0, Ordering::Release);
        self.length.store(0, Ordering::Release);
        
        if self.config.enable_stats {
            self.stats.reset();
        }
        
        self.update_flow_state();
    }
    
    fn stats(&self) -> BufferStats {
        self.stats
    }
}

impl ReadableBuffer for StreamBuffer {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        let len = self.len();
        if len == 0 {
            if self.config.enable_stats {
                self.stats.record_underflow();
            }
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let to_read = buf.len().min(len);
        
        if read_pos < write_pos {
            // 数据是连续的
            let available = write_pos - read_pos;
            let copy_len = to_read.min(available);
            buf[..copy_len].copy_from_slice(&self.data[read_pos..read_pos + copy_len]);
            
            self.read_pos.store(read_pos + copy_len, Ordering::Release);
            self.length.store(len - copy_len, Ordering::Release);
            
            if self.config.enable_stats {
                self.stats.record_read(copy_len);
                self.stats.update_usage(self.len());
            }
            
            Ok(copy_len)
        } else {
            // 数据环绕
            let first_part = self.size - read_pos;
            let second_part = write_pos;
            
            if to_read <= first_part {
                // 只需要读取第一部分
                buf[..to_read].copy_from_slice(&self.data[read_pos..read_pos + to_read]);
                let new_read_pos = if read_pos + to_read == self.size { 0 } else { read_pos + to_read };
                self.read_pos.store(new_read_pos, Ordering::Release);
            } else {
                // 需要读取两部分
                buf[..first_part].copy_from_slice(&self.data[read_pos..]);
                let remaining = to_read - first_part;
                let second_read = remaining.min(second_part);
                buf[first_part..first_part + second_read].copy_from_slice(&self.data[..second_read]);
                self.read_pos.store(second_read, Ordering::Release);
            }
            
            self.length.store(len - to_read, Ordering::Release);
            
            if self.config.enable_stats {
                self.stats.record_read(to_read);
                self.stats.update_usage(self.len());
            }
            
            Ok(to_read)
        }
    }
    
    fn read_byte(&mut self) -> Result<u8> {
        let mut buf = [0u8; 1];
        self.read(&mut buf)?;
        Ok(buf[0])
    }
    
    fn peek(&self, buf: &mut [u8]) -> Result<usize> {
        let len = self.len();
        if len == 0 {
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let to_peek = buf.len().min(len);
        
        if read_pos < write_pos {
            // 数据是连续的
            let available = write_pos - read_pos;
            let copy_len = to_peek.min(available);
            buf[..copy_len].copy_from_slice(&self.data[read_pos..read_pos + copy_len]);
            Ok(copy_len)
        } else {
            // 数据环绕
            let first_part = self.size - read_pos;
            
            if to_peek <= first_part {
                buf[..to_peek].copy_from_slice(&self.data[read_pos..read_pos + to_peek]);
            } else {
                buf[..first_part].copy_from_slice(&self.data[read_pos..]);
                let remaining = to_peek - first_part;
                let second_peek = remaining.min(write_pos);
                buf[first_part..first_part + second_peek].copy_from_slice(&self.data[..second_peek]);
            }
            
            Ok(to_peek)
        }
    }
    
    fn peek_byte(&self) -> Result<u8> {
        if self.is_empty() {
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        Ok(self.data[read_pos])
    }
    
    fn skip(&mut self, count: usize) -> Result<usize> {
        let len = self.len();
        let to_skip = count.min(len);
        
        if to_skip > 0 {
            let read_pos = self.read_pos.load(Ordering::Acquire);
            let new_read_pos = (read_pos + to_skip) % self.size;
            self.read_pos.store(new_read_pos, Ordering::Release);
            self.length.store(len - to_skip, Ordering::Release);
            
            if self.config.enable_stats {
                self.stats.record_read(to_skip);
                self.stats.update_usage(self.len());
            }
        }
        
        Ok(to_skip)
    }
    
    fn read_until(&mut self, delimiter: u8, buf: &mut [u8]) -> Result<usize> {
        let len = self.len();
        if len == 0 {
            return Err(BufferError::Empty.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let mut found_pos = None;
        let mut search_len = 0;
        
        // 查找分隔符
        if read_pos < write_pos {
            // 数据是连续的
            for i in 0..(write_pos - read_pos) {
                if self.data[read_pos + i] == delimiter {
                    found_pos = Some(i + 1);
                    break;
                }
                search_len += 1;
            }
        } else {
            // 数据环绕，先搜索第一部分
            for i in 0..(self.size - read_pos) {
                if self.data[read_pos + i] == delimiter {
                    found_pos = Some(i + 1);
                    break;
                }
                search_len += 1;
            }
            
            // 如果没找到，搜索第二部分
            if found_pos.is_none() {
                for i in 0..write_pos {
                    if self.data[i] == delimiter {
                        found_pos = Some(search_len + i + 1);
                        break;
                    }
                }
            }
        }
        
        let to_read = if let Some(pos) = found_pos {
            pos.min(buf.len())
        } else {
            len.min(buf.len())
        };
        
        self.read(&mut buf[..to_read])
    }
}

impl WritableBuffer for StreamBuffer {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        let available = self.available();
        if available == 0 {
            if self.config.enable_stats {
                self.stats.record_overflow();
            }
            return Err(BufferError::Full.into());
        }
        
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let to_write = buf.len().min(available);
        
        if write_pos < read_pos {
            // 可写空间是连续的
            let space = read_pos - write_pos;
            let copy_len = to_write.min(space);
            self.data[write_pos..write_pos + copy_len].copy_from_slice(&buf[..copy_len]);
            
            self.write_pos.store(write_pos + copy_len, Ordering::Release);
            let new_length = self.length.load(Ordering::Acquire) + copy_len;
            self.length.store(new_length, Ordering::Release);
            
            if self.config.enable_stats {
                self.stats.record_write(copy_len);
                self.stats.update_usage(self.len());
            }
            
            Ok(copy_len)
        } else {
            // 可写空间可能环绕
            let end_space = self.size - write_pos;
            let start_space = read_pos;
            
            if to_write <= end_space {
                // 只写入末尾空间
                self.data[write_pos..write_pos + to_write].copy_from_slice(&buf[..to_write]);
                let new_write_pos = if write_pos + to_write == self.size { 0 } else { write_pos + to_write };
                self.write_pos.store(new_write_pos, Ordering::Release);
            } else {
                // 需要环绕写入
                self.data[write_pos..].copy_from_slice(&buf[..end_space]);
                let remaining = to_write - end_space;
                let start_write = remaining.min(start_space);
                self.data[..start_write].copy_from_slice(&buf[end_space..end_space + start_write]);
                self.write_pos.store(start_write, Ordering::Release);
            }
            
            let new_length = self.length.load(Ordering::Acquire) + to_write;
            self.length.store(new_length, Ordering::Release);
            
            if self.config.enable_stats {
                self.stats.record_write(to_write);
                self.stats.update_usage(self.len());
            }
            
            Ok(to_write)
        }
    }
    
    fn write_byte(&mut self, byte: u8) -> Result<()> {
        let buf = [byte];
        self.write(&buf)?;
        Ok(())
    }
    
    fn flush(&mut self) -> Result<()> {
        // 流式缓冲区的刷新操作
        Ok(())
    }
    
    fn reserve(&mut self, additional: usize) -> Result<()> {
        if self.available() < additional {
            return Err(BufferError::Full.into());
        }
        Ok(())
    }
}

// 为StreamBuffer实现Stream trait
pin_project! {
    /// 流式缓冲区的Stream包装器
    pub struct StreamBufferStream {
        #[pin]
        buffer: StreamBuffer,
    }
}

impl StreamBufferStream {
    /// 创建新的流包装器
    pub fn new(buffer: StreamBuffer) -> Self {
        Self { buffer }
    }
}

impl Stream for StreamBufferStream {
    type Item = Result<Vec<u8>>;
    
    fn poll_next(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.project();
        
        if this.buffer.is_closed() && this.buffer.is_empty() {
            return Poll::Ready(None);
        }
        
        match this.buffer.read_chunk() {
            Ok(chunk) => {
                if chunk.is_empty() {
                    this.buffer.read_waker = Some(cx.waker().clone());
                    Poll::Pending
                } else {
                    Poll::Ready(Some(Ok(chunk)))
                }
            }
            Err(e) => Poll::Ready(Some(Err(e))),
        }
    }
}

// 为StreamBuffer实现Sink trait
impl Sink<Vec<u8>> for StreamBuffer {
    type Error = crate::error::UartError;
    
    fn poll_ready(mut self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<(), Self::Error>> {
        if self.is_closed() {
            return Poll::Ready(Err(BufferError::Closed.into()));
        }
        
        if self.state() == StreamState::Paused {
            self.write_waker = Some(cx.waker().clone());
            Poll::Pending
        } else {
            Poll::Ready(Ok(()))
        }
    }
    
    fn start_send(mut self: core::pin::Pin<&mut Self>, item: Vec<u8>) -> Result<(), Self::Error> {
        self.write_chunk(&item)?;
        Ok(())
    }
    
    fn poll_flush(mut self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<(), Self::Error>> {
        self.poll_flush(cx)
    }
    
    fn poll_close(mut self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<(), Self::Error>> {
        self.poll_close(cx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_stream_buffer_creation() {
        let config = StreamBufferConfig::new(1024);
        let buffer = StreamBuffer::new(config).unwrap();
        
        assert_eq!(buffer.capacity(), 1024);
        assert_eq!(buffer.len(), 0);
        assert_eq!(buffer.state(), StreamState::Flowing);
        assert!(!buffer.is_closed());
    }
    
    #[test]
    fn test_stream_buffer_write_read() {
        let config = StreamBufferConfig::new(1024);
        let mut buffer = StreamBuffer::new(config).unwrap();
        
        // 写入数据
        let data = b"Hello, Stream World!";
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
    fn test_stream_buffer_backpressure() {
        let config = StreamBufferConfig::new(100)
            .watermarks(20, 80);
        let mut buffer = StreamBuffer::new(config).unwrap();
        
        // 写入数据直到高水位
        let data = vec![0u8; 85];
        buffer.write(&data).unwrap();
        
        // 检查是否触发背压
        assert!(buffer.should_pause());
        buffer.update_flow_state();
        assert_eq!(buffer.state(), StreamState::Paused);
        
        // 读取数据直到低水位
        let mut read_buf = vec![0u8; 70];
        buffer.read(&mut read_buf).unwrap();
        
        // 检查是否恢复
        assert!(buffer.should_resume());
        buffer.update_flow_state();
        assert_eq!(buffer.state(), StreamState::Flowing);
    }
    
    #[test]
    fn test_stream_buffer_overflow_strategies() {
        // 测试丢弃最旧数据策略
        let config = StreamBufferConfig::new(10)
            .overflow_strategy(OverflowStrategy::DropOldest);
        let mut buffer = StreamBuffer::new(config).unwrap();
        
        // 填满缓冲区
        buffer.write(b"1234567890").unwrap();
        assert_eq!(buffer.len(), 10);
        
        // 写入更多数据，应该丢弃最旧的
        buffer.write(b"ABC").unwrap();
        assert_eq!(buffer.len(), 10);
        
        // 读取数据，应该是最新的
        let mut read_buf = [0u8; 10];
        let read = buffer.read(&mut read_buf).unwrap();
        assert_eq!(&read_buf[..read], b"4567890ABC");
    }
    
    #[test]
    fn test_stream_buffer_chunks() {
        let config = StreamBufferConfig::new(1024).chunk_size(10);
        let mut buffer = StreamBuffer::new(config).unwrap();
        
        // 写入数据
        buffer.write(b"Hello, World! This is a test.").unwrap();
        
        // 读取块
        let chunk = buffer.read_chunk().unwrap();
        assert_eq!(chunk.len(), 10);
        assert_eq!(&chunk, b"Hello, Wor");
        
        // 再读取一块
        let chunk = buffer.read_chunk().unwrap();
        assert_eq!(chunk.len(), 10);
        assert_eq!(&chunk, b"ld! This i");
    }
    
    #[test]
    fn test_stream_buffer_watermarks() {
        let config = StreamBufferConfig::new(100).watermarks(25, 75);
        let buffer = StreamBuffer::new(config).unwrap();
        
        assert_eq!(buffer.config().low_watermark, 25);
        assert_eq!(buffer.config().high_watermark, 75);
        
        let (low, high) = buffer.watermark_status();
        assert!(low);  // 空缓冲区低于低水位
        assert!(!high); // 空缓冲区低于高水位
    }
    
    #[test]
    fn test_stream_buffer_close() {
        let config = StreamBufferConfig::new(1024);
        let mut buffer = StreamBuffer::new(config).unwrap();
        
        assert!(!buffer.is_closed());
        assert_eq!(buffer.state(), StreamState::Flowing);
        
        buffer.close();
        
        assert!(buffer.is_closed());
        assert_eq!(buffer.state(), StreamState::Closed);
    }
    
    #[test]
    fn test_stream_buffer_config_validation() {
        // 有效配置
        let config = StreamBufferConfig::new(1024).watermarks(256, 768);
        assert!(config.validate().is_ok());
        
        // 无效大小
        let config = StreamBufferConfig::new(0);
        assert!(config.validate().is_err());
        
        // 无效水位标记
        let config = StreamBufferConfig::new(1024).watermarks(800, 200);
        assert!(config.validate().is_err());
        
        let config = StreamBufferConfig::new(1024).watermarks(500, 1024);
        assert!(config.validate().is_err());
        
        // 无效块大小
        let config = StreamBufferConfig::new(1024).chunk_size(0);
        assert!(config.validate().is_err());
        
        let config = StreamBufferConfig::new(1024).chunk_size(2048);
        assert!(config.validate().is_err());
    }
}