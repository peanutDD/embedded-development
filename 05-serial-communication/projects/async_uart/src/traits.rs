//! # 异步UART特征定义
//!
//! 定义了异步UART通信的核心特征，提供统一的异步接口。

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embedded_hal_async::serial::{Read, Write};
use crate::{Error, Result, Statistics};

/// 异步读取特征
pub trait AsyncRead {
    /// 异步读取数据到缓冲区
    /// 
    /// # 参数
    /// 
    /// * `buf` - 目标缓冲区
    /// 
    /// # 返回值
    /// 
    /// 返回读取的字节数
    fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> impl Future<Output = Result<usize>> + 'a;
    
    /// 异步读取确切数量的字节
    /// 
    /// # 参数
    /// 
    /// * `buf` - 目标缓冲区
    /// 
    /// # 返回值
    /// 
    /// 成功时返回()，失败时返回错误
    fn read_exact<'a>(&'a mut self, buf: &'a mut [u8]) -> impl Future<Output = Result<()>> + 'a {
        async move {
            let mut pos = 0;
            while pos < buf.len() {
                let n = self.read(&mut buf[pos..]).await?;
                if n == 0 {
                    return Err(Error::UnexpectedEof);
                }
                pos += n;
            }
            Ok(())
        }
    }
    
    /// 异步读取直到遇到分隔符
    /// 
    /// # 参数
    /// 
    /// * `buf` - 目标缓冲区
    /// * `delimiter` - 分隔符字节
    /// 
    /// # 返回值
    /// 
    /// 返回读取的字节数（包含分隔符）
    fn read_until<'a>(&'a mut self, buf: &'a mut [u8], delimiter: u8) -> impl Future<Output = Result<usize>> + 'a {
        async move {
            let mut pos = 0;
            while pos < buf.len() {
                let n = self.read(&mut buf[pos..pos+1]).await?;
                if n == 0 {
                    return Err(Error::UnexpectedEof);
                }
                pos += n;
                if buf[pos-1] == delimiter {
                    break;
                }
            }
            Ok(pos)
        }
    }
    
    /// 异步读取一行（以\n结尾）
    /// 
    /// # 参数
    /// 
    /// * `buf` - 目标缓冲区
    /// 
    /// # 返回值
    /// 
    /// 返回读取的字节数（包含换行符）
    fn read_line<'a>(&'a mut self, buf: &'a mut [u8]) -> impl Future<Output = Result<usize>> + 'a {
        self.read_until(buf, b'\n')
    }
}

/// 异步写入特征
pub trait AsyncWrite {
    /// 异步写入数据
    /// 
    /// # 参数
    /// 
    /// * `buf` - 要写入的数据
    /// 
    /// # 返回值
    /// 
    /// 返回写入的字节数
    fn write<'a>(&'a mut self, buf: &'a [u8]) -> impl Future<Output = Result<usize>> + 'a;
    
    /// 异步写入所有数据
    /// 
    /// # 参数
    /// 
    /// * `buf` - 要写入的数据
    /// 
    /// # 返回值
    /// 
    /// 成功时返回()，失败时返回错误
    fn write_all<'a>(&'a mut self, buf: &'a [u8]) -> impl Future<Output = Result<()>> + 'a {
        async move {
            let mut pos = 0;
            while pos < buf.len() {
                let n = self.write(&buf[pos..]).await?;
                if n == 0 {
                    return Err(Error::WriteZero);
                }
                pos += n;
            }
            Ok(())
        }
    }
    
    /// 异步刷新写入缓冲区
    fn flush(&mut self) -> impl Future<Output = Result<()>>;
    
    /// 异步写入字符串
    /// 
    /// # 参数
    /// 
    /// * `s` - 要写入的字符串
    /// 
    /// # 返回值
    /// 
    /// 成功时返回()，失败时返回错误
    fn write_str<'a>(&'a mut self, s: &'a str) -> impl Future<Output = Result<()>> + 'a {
        self.write_all(s.as_bytes())
    }
    
    /// 异步写入格式化字符串
    /// 
    /// # 参数
    /// 
    /// * `args` - 格式化参数
    /// 
    /// # 返回值
    /// 
    /// 成功时返回()，失败时返回错误
    fn write_fmt<'a>(&'a mut self, args: core::fmt::Arguments<'_>) -> impl Future<Output = Result<()>> + 'a {
        async move {
            use heapless::String;
            let mut buf = String::<256>::new();
            core::fmt::write(&mut buf, args).map_err(|_| Error::FormatError)?;
            self.write_all(buf.as_bytes()).await
        }
    }
}

/// 异步UART特征
pub trait AsyncUart: AsyncRead + AsyncWrite {
    /// 获取统计信息
    fn statistics(&self) -> &Statistics;
    
    /// 重置统计信息
    fn reset_statistics(&mut self);
    
    /// 检查是否有数据可读
    fn poll_read_ready(&mut self, cx: &mut Context<'_>) -> Poll<Result<()>>;
    
    /// 检查是否可以写入
    fn poll_write_ready(&mut self, cx: &mut Context<'_>) -> Poll<Result<()>>;
    
    /// 设置超时时间
    fn set_timeout(&mut self, timeout_ms: u32);
    
    /// 获取超时时间
    fn timeout(&self) -> u32;
    
    /// 启用/禁用DMA
    fn set_dma_enabled(&mut self, enabled: bool) -> Result<()>;
    
    /// 检查DMA是否启用
    fn is_dma_enabled(&self) -> bool;
    
    /// 获取缓冲区使用情况
    fn buffer_usage(&self) -> (usize, usize); // (used, total)
    
    /// 清空接收缓冲区
    fn clear_rx_buffer(&mut self);
    
    /// 清空发送缓冲区
    fn clear_tx_buffer(&mut self);
    
    /// 异步等待数据可读
    fn wait_for_data<'a>(&'a mut self) -> impl Future<Output = Result<()>> + 'a {
        async move {
            futures::future::poll_fn(|cx| self.poll_read_ready(cx)).await
        }
    }
    
    /// 异步等待可以写入
    fn wait_for_write<'a>(&'a mut self) -> impl Future<Output = Result<()>> + 'a {
        async move {
            futures::future::poll_fn(|cx| self.poll_write_ready(cx)).await
        }
    }
}

/// 异步UART流特征
pub trait AsyncUartStream: AsyncUart {
    /// 分割为读写两部分
    fn split(&mut self) -> (AsyncUartReader<'_>, AsyncUartWriter<'_>);
}

/// 异步UART读取器
pub struct AsyncUartReader<'a> {
    uart: &'a mut dyn AsyncUart,
}

impl<'a> AsyncUartReader<'a> {
    /// 创建新的读取器
    pub fn new(uart: &'a mut dyn AsyncUart) -> Self {
        Self { uart }
    }
}

impl<'a> AsyncRead for AsyncUartReader<'a> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.uart.read(buf).await
    }
}

/// 异步UART写入器
pub struct AsyncUartWriter<'a> {
    uart: &'a mut dyn AsyncUart,
}

impl<'a> AsyncUartWriter<'a> {
    /// 创建新的写入器
    pub fn new(uart: &'a mut dyn AsyncUart) -> Self {
        Self { uart }
    }
}

impl<'a> AsyncWrite for AsyncUartWriter<'a> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize> {
        self.uart.write(buf).await
    }
    
    async fn flush(&mut self) -> Result<()> {
        self.uart.flush().await
    }
}

/// 带超时的异步读取特征
pub trait AsyncReadWithTimeout: AsyncRead {
    /// 带超时的异步读取
    /// 
    /// # 参数
    /// 
    /// * `buf` - 目标缓冲区
    /// * `timeout_ms` - 超时时间（毫秒）
    /// 
    /// # 返回值
    /// 
    /// 返回读取的字节数或超时错误
    fn read_timeout<'a>(&'a mut self, buf: &'a mut [u8], timeout_ms: u32) -> impl Future<Output = Result<usize>> + 'a {
        async move {
            use embassy_time::{Duration, Timer};
            
            let read_future = self.read(buf);
            let timeout_future = Timer::after(Duration::from_millis(timeout_ms as u64));
            
            match embassy_futures::select::select(read_future, timeout_future).await {
                embassy_futures::select::Either::First(result) => result,
                embassy_futures::select::Either::Second(_) => Err(Error::Timeout),
            }
        }
    }
}

/// 带超时的异步写入特征
pub trait AsyncWriteWithTimeout: AsyncWrite {
    /// 带超时的异步写入
    /// 
    /// # 参数
    /// 
    /// * `buf` - 要写入的数据
    /// * `timeout_ms` - 超时时间（毫秒）
    /// 
    /// # 返回值
    /// 
    /// 返回写入的字节数或超时错误
    fn write_timeout<'a>(&'a mut self, buf: &'a [u8], timeout_ms: u32) -> impl Future<Output = Result<usize>> + 'a {
        async move {
            use embassy_time::{Duration, Timer};
            
            let write_future = self.write(buf);
            let timeout_future = Timer::after(Duration::from_millis(timeout_ms as u64));
            
            match embassy_futures::select::select(write_future, timeout_future).await {
                embassy_futures::select::Either::First(result) => result,
                embassy_futures::select::Either::Second(_) => Err(Error::Timeout),
            }
        }
    }
}

// 为所有实现AsyncRead的类型自动实现AsyncReadWithTimeout
impl<T: AsyncRead> AsyncReadWithTimeout for T {}

// 为所有实现AsyncWrite的类型自动实现AsyncWriteWithTimeout
impl<T: AsyncWrite> AsyncWriteWithTimeout for T {}

/// 异步UART事件
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartEvent {
    /// 数据接收完成
    DataReceived(usize),
    /// 数据发送完成
    DataSent(usize),
    /// 接收错误
    ReceiveError(Error),
    /// 发送错误
    SendError(Error),
    /// 缓冲区溢出
    BufferOverflow,
    /// 超时
    Timeout,
    /// DMA传输完成
    DmaComplete,
}

/// 异步UART事件处理器
pub trait AsyncUartEventHandler {
    /// 处理UART事件
    fn handle_event(&mut self, event: UartEvent) -> impl Future<Output = ()>;
}

/// 空的事件处理器
pub struct NullEventHandler;

impl AsyncUartEventHandler for NullEventHandler {
    async fn handle_event(&mut self, _event: UartEvent) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    
    struct MockUart {
        rx_data: heapless::Vec<u8, 256>,
        tx_data: heapless::Vec<u8, 256>,
        stats: Statistics,
    }
    
    impl MockUart {
        fn new() -> Self {
            Self {
                rx_data: heapless::Vec::new(),
                tx_data: heapless::Vec::new(),
                stats: Statistics::default(),
            }
        }
        
        fn add_rx_data(&mut self, data: &[u8]) {
            self.rx_data.extend_from_slice(data).ok();
        }
    }
    
    impl AsyncRead for MockUart {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
            let len = core::cmp::min(buf.len(), self.rx_data.len());
            if len == 0 {
                return Ok(0);
            }
            
            buf[..len].copy_from_slice(&self.rx_data[..len]);
            self.rx_data.drain(..len);
            self.stats.bytes_received += len as u64;
            Ok(len)
        }
    }
    
    impl AsyncWrite for MockUart {
        async fn write(&mut self, buf: &[u8]) -> Result<usize> {
            let len = core::cmp::min(buf.len(), self.tx_data.capacity() - self.tx_data.len());
            self.tx_data.extend_from_slice(&buf[..len]).map_err(|_| Error::BufferFull)?;
            self.stats.bytes_sent += len as u64;
            Ok(len)
        }
        
        async fn flush(&mut self) -> Result<()> {
            Ok(())
        }
    }
    
    impl AsyncUart for MockUart {
        fn statistics(&self) -> &Statistics {
            &self.stats
        }
        
        fn reset_statistics(&mut self) {
            self.stats.reset();
        }
        
        fn poll_read_ready(&mut self, _cx: &mut Context<'_>) -> Poll<Result<()>> {
            if self.rx_data.is_empty() {
                Poll::Pending
            } else {
                Poll::Ready(Ok(()))
            }
        }
        
        fn poll_write_ready(&mut self, _cx: &mut Context<'_>) -> Poll<Result<()>> {
            if self.tx_data.len() < self.tx_data.capacity() {
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        }
        
        fn set_timeout(&mut self, _timeout_ms: u32) {}
        
        fn timeout(&self) -> u32 {
            1000
        }
        
        fn set_dma_enabled(&mut self, _enabled: bool) -> Result<()> {
            Ok(())
        }
        
        fn is_dma_enabled(&self) -> bool {
            false
        }
        
        fn buffer_usage(&self) -> (usize, usize) {
            (self.rx_data.len(), self.rx_data.capacity())
        }
        
        fn clear_rx_buffer(&mut self) {
            self.rx_data.clear();
        }
        
        fn clear_tx_buffer(&mut self) {
            self.tx_data.clear();
        }
    }
    
    #[tokio::test]
    async fn test_async_read_write() {
        let mut uart = MockUart::new();
        uart.add_rx_data(b"Hello, World!");
        
        // 测试读取
        let mut buf = [0u8; 13];
        let len = uart.read(&mut buf).await.unwrap();
        assert_eq!(len, 13);
        assert_eq!(&buf, b"Hello, World!");
        
        // 测试写入
        let data = b"Test data";
        let len = uart.write(data).await.unwrap();
        assert_eq!(len, 9);
        
        // 检查统计信息
        let stats = uart.statistics();
        assert_eq!(stats.bytes_received, 13);
        assert_eq!(stats.bytes_sent, 9);
    }
    
    #[tokio::test]
    async fn test_read_exact() {
        let mut uart = MockUart::new();
        uart.add_rx_data(b"Hello, World!");
        
        let mut buf = [0u8; 5];
        uart.read_exact(&mut buf).await.unwrap();
        assert_eq!(&buf, b"Hello");
    }
    
    #[tokio::test]
    async fn test_write_all() {
        let mut uart = MockUart::new();
        
        let data = b"Test data";
        uart.write_all(data).await.unwrap();
        
        let stats = uart.statistics();
        assert_eq!(stats.bytes_sent, 9);
    }
}