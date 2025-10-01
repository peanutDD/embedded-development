//! # 通用HAL适配器
//!
//! 提供通用的UART HAL适配器实现，主要用于测试和开发。

use super::{HalAdapter, Platform, UartErrors, UartInterrupts, UartPins, UartStatus};
use crate::{
    config::Config,
    error::{HardwareError, Result},
    traits::{AsyncRead, AsyncWrite},
};
use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

/// 通用HAL适配器
pub struct GenericHalAdapter {
    /// 内部状态
    state: Arc<Mutex<GenericState>>,
}

/// 通用适配器内部状态
#[derive(Debug)]
struct GenericState {
    /// 平台类型
    platform: Platform,
    /// 配置
    config: Option<Config>,
    /// 引脚配置
    pins: Option<UartPins>,
    /// UART状态
    status: UartStatus,
    /// 错误状态
    errors: UartErrors,
    /// 启用的中断
    interrupts: UartInterrupts,
    /// 接收缓冲区
    rx_buffer: VecDeque<u8>,
    /// 发送缓冲区
    tx_buffer: VecDeque<u8>,
    /// 最大缓冲区大小
    max_buffer_size: usize,
    /// 模拟延迟
    simulate_delay: bool,
    /// 延迟时间
    delay_duration: Duration,
    /// 最后操作时间
    last_operation: Option<Instant>,
    /// 错误注入
    inject_errors: bool,
    /// 错误注入概率
    error_probability: f32,
    /// 统计信息
    stats: GenericStats,
}

/// 统计信息
#[derive(Debug, Default)]
struct GenericStats {
    /// 读取字节数
    bytes_read: usize,
    /// 写入字节数
    bytes_written: usize,
    /// 读取操作次数
    read_operations: usize,
    /// 写入操作次数
    write_operations: usize,
    /// 错误次数
    error_count: usize,
    /// 初始化次数
    init_count: usize,
}

impl GenericHalAdapter {
    /// 创建新的通用适配器
    pub fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(GenericState {
                platform: Platform::Generic,
                config: None,
                pins: None,
                status: UartStatus::default(),
                errors: UartErrors::default(),
                interrupts: UartInterrupts::default(),
                rx_buffer: VecDeque::new(),
                tx_buffer: VecDeque::new(),
                max_buffer_size: 1024,
                simulate_delay: false,
                delay_duration: Duration::from_millis(1),
                last_operation: None,
                inject_errors: false,
                error_probability: 0.0,
                stats: GenericStats::default(),
            })),
        }
    }
    
    /// 设置最大缓冲区大小
    pub fn set_max_buffer_size(&self, size: usize) {
        if let Ok(mut state) = self.state.lock() {
            state.max_buffer_size = size;
        }
    }
    
    /// 启用延迟模拟
    pub fn enable_delay_simulation(&self, delay: Duration) {
        if let Ok(mut state) = self.state.lock() {
            state.simulate_delay = true;
            state.delay_duration = delay;
        }
    }
    
    /// 禁用延迟模拟
    pub fn disable_delay_simulation(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.simulate_delay = false;
        }
    }
    
    /// 启用错误注入
    pub fn enable_error_injection(&self, probability: f32) {
        if let Ok(mut state) = self.state.lock() {
            state.inject_errors = true;
            state.error_probability = probability.clamp(0.0, 1.0);
        }
    }
    
    /// 禁用错误注入
    pub fn disable_error_injection(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.inject_errors = false;
        }
    }
    
    /// 向接收缓冲区添加数据
    pub fn add_rx_data(&self, data: &[u8]) -> Result<()> {
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        
        if state.rx_buffer.len() + data.len() > state.max_buffer_size {
            state.errors.overrun_error = true;
            return Err(HardwareError::BufferOverflow.into());
        }
        
        for &byte in data {
            state.rx_buffer.push_back(byte);
        }
        
        state.status.rx_not_empty = !state.rx_buffer.is_empty();
        Ok(())
    }
    
    /// 从发送缓冲区获取数据
    pub fn get_tx_data(&self) -> Result<Vec<u8>> {
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        
        let data: Vec<u8> = state.tx_buffer.drain(..).collect();
        state.status.tx_empty = state.tx_buffer.is_empty();
        
        Ok(data)
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> Result<GenericStats> {
        let state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        Ok(GenericStats {
            bytes_read: state.stats.bytes_read,
            bytes_written: state.stats.bytes_written,
            read_operations: state.stats.read_operations,
            write_operations: state.stats.write_operations,
            error_count: state.stats.error_count,
            init_count: state.stats.init_count,
        })
    }
    
    /// 重置统计信息
    pub fn reset_stats(&self) -> Result<()> {
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        state.stats = GenericStats::default();
        Ok(())
    }
    
    /// 模拟延迟
    async fn simulate_delay(&self) -> Result<()> {
        let delay = {
            let state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
            if state.simulate_delay {
                Some(state.delay_duration)
            } else {
                None
            }
        };
        
        if let Some(delay) = delay {
            tokio::time::sleep(delay).await;
        }
        
        Ok(())
    }
    
    /// 检查是否应该注入错误
    fn should_inject_error(&self) -> bool {
        let state = match self.state.lock() {
            Ok(state) => state,
            Err(_) => return false,
        };
        
        if !state.inject_errors {
            return false;
        }
        
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen::<f32>() < state.error_probability
    }
}

impl Default for GenericHalAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl HalAdapter for GenericHalAdapter {
    fn platform(&self) -> Platform {
        Platform::Generic
    }
    
    async fn init(&mut self, config: &Config, pins: &UartPins) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        
        // 验证配置
        config.validate()?;
        pins.validate()?;
        
        // 保存配置
        state.config = Some(config.clone());
        state.pins = Some(pins.clone());
        
        // 更新状态
        state.status.initialized = true;
        state.status.has_errors = false;
        state.errors = UartErrors::default();
        
        // 更新统计
        state.stats.init_count += 1;
        
        Ok(())
    }
    
    async fn enable(&mut self) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        
        if !state.status.initialized {
            return Err(HardwareError::NotInitialized.into());
        }
        
        state.status.enabled = true;
        Ok(())
    }
    
    async fn disable(&mut self) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        
        state.status.enabled = false;
        state.status.transmitting = false;
        state.status.receiving = false;
        
        Ok(())
    }
    
    async fn reset(&mut self) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        
        // 重置状态
        state.status = UartStatus::default();
        state.errors = UartErrors::default();
        state.interrupts = UartInterrupts::default();
        
        // 清空缓冲区
        state.rx_buffer.clear();
        state.tx_buffer.clear();
        
        Ok(())
    }
    
    fn status(&self) -> UartStatus {
        self.state.lock().map(|state| state.status).unwrap_or_default()
    }
    
    async fn set_baudrate(&mut self, baudrate: u32) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        
        if let Some(ref mut config) = state.config {
            config.baudrate = baudrate.into();
        }
        
        Ok(())
    }
    
    async fn enable_dma(&mut self, _tx_channel: Option<u8>, _rx_channel: Option<u8>) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        state.status.dma_enabled = true;
        
        Ok(())
    }
    
    async fn disable_dma(&mut self) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        state.status.dma_enabled = false;
        
        Ok(())
    }
    
    async fn enable_interrupts(&mut self, interrupts: UartInterrupts) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        state.interrupts = interrupts;
        
        Ok(())
    }
    
    async fn disable_interrupts(&mut self, _interrupts: UartInterrupts) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        state.interrupts = UartInterrupts::default();
        
        Ok(())
    }
    
    async fn clear_interrupt_flags(&mut self, _flags: UartInterrupts) -> Result<()> {
        self.simulate_delay().await?;
        Ok(())
    }
    
    fn get_errors(&self) -> UartErrors {
        self.state.lock().map(|state| state.errors).unwrap_or_default()
    }
    
    async fn clear_errors(&mut self) -> Result<()> {
        self.simulate_delay().await?;
        
        let mut state = self.state.lock().map_err(|_| HardwareError::ResourceBusy)?;
        state.errors = UartErrors::default();
        state.status.has_errors = false;
        
        Ok(())
    }
}

impl AsyncRead for GenericHalAdapter {
    fn poll_read(
        self: Pin<&mut Self>,
        _cx: &mut Context<'_>,
        buf: &mut [u8],
    ) -> Poll<Result<usize>> {
        let mut state = match self.state.lock() {
            Ok(state) => state,
            Err(_) => return Poll::Ready(Err(HardwareError::ResourceBusy.into())),
        };
        
        if !state.status.enabled {
            return Poll::Ready(Err(HardwareError::NotEnabled.into()));
        }
        
        // 检查错误注入
        if self.should_inject_error() {
            state.errors.frame_error = true;
            state.stats.error_count += 1;
            return Poll::Ready(Err(HardwareError::FrameError.into()));
        }
        
        let bytes_to_read = buf.len().min(state.rx_buffer.len());
        if bytes_to_read == 0 {
            return Poll::Pending;
        }
        
        for i in 0..bytes_to_read {
            buf[i] = state.rx_buffer.pop_front().unwrap();
        }
        
        // 更新状态
        state.status.rx_not_empty = !state.rx_buffer.is_empty();
        state.status.receiving = bytes_to_read > 0;
        
        // 更新统计
        state.stats.bytes_read += bytes_to_read;
        state.stats.read_operations += 1;
        state.last_operation = Some(Instant::now());
        
        Poll::Ready(Ok(bytes_to_read))
    }
}

impl AsyncWrite for GenericHalAdapter {
    fn poll_write(
        self: Pin<&mut Self>,
        _cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<Result<usize>> {
        let mut state = match self.state.lock() {
            Ok(state) => state,
            Err(_) => return Poll::Ready(Err(HardwareError::ResourceBusy.into())),
        };
        
        if !state.status.enabled {
            return Poll::Ready(Err(HardwareError::NotEnabled.into()));
        }
        
        // 检查错误注入
        if self.should_inject_error() {
            state.errors.overrun_error = true;
            state.stats.error_count += 1;
            return Poll::Ready(Err(HardwareError::BufferOverflow.into()));
        }
        
        let available_space = state.max_buffer_size.saturating_sub(state.tx_buffer.len());
        let bytes_to_write = buf.len().min(available_space);
        
        if bytes_to_write == 0 {
            return Poll::Ready(Err(HardwareError::BufferOverflow.into()));
        }
        
        for &byte in &buf[..bytes_to_write] {
            state.tx_buffer.push_back(byte);
        }
        
        // 更新状态
        state.status.tx_empty = state.tx_buffer.is_empty();
        state.status.transmitting = bytes_to_write > 0;
        
        // 更新统计
        state.stats.bytes_written += bytes_to_write;
        state.stats.write_operations += 1;
        state.last_operation = Some(Instant::now());
        
        Poll::Ready(Ok(bytes_to_write))
    }
    
    fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Result<()>> {
        let mut state = match self.state.lock() {
            Ok(state) => state,
            Err(_) => return Poll::Ready(Err(HardwareError::ResourceBusy.into())),
        };
        
        // 模拟刷新操作
        state.tx_buffer.clear();
        state.status.tx_empty = true;
        state.status.transmitting = false;
        
        Poll::Ready(Ok(()))
    }
    
    fn poll_close(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Result<()>> {
        let mut state = match self.state.lock() {
            Ok(state) => state,
            Err(_) => return Poll::Ready(Err(HardwareError::ResourceBusy.into())),
        };
        
        // 关闭UART
        state.status.enabled = false;
        state.status.transmitting = false;
        state.status.receiving = false;
        
        Poll::Ready(Ok(()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::presets;
    use tokio_test;
    
    #[tokio::test]
    async fn test_generic_adapter_creation() {
        let adapter = GenericHalAdapter::new();
        assert_eq!(adapter.platform(), Platform::Generic);
        
        let status = adapter.status();
        assert!(!status.initialized);
        assert!(!status.enabled);
    }
    
    #[tokio::test]
    async fn test_generic_adapter_init() {
        let mut adapter = GenericHalAdapter::new();
        let config = presets::standard_115200();
        let pins = UartPins::new(1, 2);
        
        let result = adapter.init(&config, &pins).await;
        assert!(result.is_ok());
        
        let status = adapter.status();
        assert!(status.initialized);
    }
    
    #[tokio::test]
    async fn test_generic_adapter_enable_disable() {
        let mut adapter = GenericHalAdapter::new();
        let config = presets::standard_115200();
        let pins = UartPins::new(1, 2);
        
        adapter.init(&config, &pins).await.unwrap();
        
        let result = adapter.enable().await;
        assert!(result.is_ok());
        assert!(adapter.status().enabled);
        
        let result = adapter.disable().await;
        assert!(result.is_ok());
        assert!(!adapter.status().enabled);
    }
    
    #[tokio::test]
    async fn test_generic_adapter_rx_data() {
        let adapter = GenericHalAdapter::new();
        let test_data = b"Hello, World!";
        
        let result = adapter.add_rx_data(test_data);
        assert!(result.is_ok());
        
        let status = adapter.status();
        assert!(status.rx_not_empty);
    }
    
    #[tokio::test]
    async fn test_generic_adapter_tx_data() {
        let adapter = GenericHalAdapter::new();
        
        // 模拟写入数据
        {
            let mut state = adapter.state.lock().unwrap();
            state.status.enabled = true;
            for &byte in b"Test data" {
                state.tx_buffer.push_back(byte);
            }
        }
        
        let tx_data = adapter.get_tx_data().unwrap();
        assert_eq!(tx_data, b"Test data");
        
        let status = adapter.status();
        assert!(status.tx_empty);
    }
    
    #[tokio::test]
    async fn test_generic_adapter_stats() {
        let adapter = GenericHalAdapter::new();
        
        // 模拟一些操作
        {
            let mut state = adapter.state.lock().unwrap();
            state.stats.bytes_read = 100;
            state.stats.bytes_written = 200;
            state.stats.read_operations = 10;
            state.stats.write_operations = 20;
        }
        
        let stats = adapter.get_stats().unwrap();
        assert_eq!(stats.bytes_read, 100);
        assert_eq!(stats.bytes_written, 200);
        assert_eq!(stats.read_operations, 10);
        assert_eq!(stats.write_operations, 20);
        
        adapter.reset_stats().unwrap();
        let stats = adapter.get_stats().unwrap();
        assert_eq!(stats.bytes_read, 0);
        assert_eq!(stats.bytes_written, 0);
    }
    
    #[tokio::test]
    async fn test_generic_adapter_delay_simulation() {
        let adapter = GenericHalAdapter::new();
        
        adapter.enable_delay_simulation(Duration::from_millis(10));
        
        let start = Instant::now();
        adapter.simulate_delay().await.unwrap();
        let elapsed = start.elapsed();
        
        assert!(elapsed >= Duration::from_millis(10));
        
        adapter.disable_delay_simulation();
    }
    
    #[tokio::test]
    async fn test_generic_adapter_error_injection() {
        let adapter = GenericHalAdapter::new();
        
        adapter.enable_error_injection(1.0); // 100% 错误概率
        assert!(adapter.should_inject_error());
        
        adapter.enable_error_injection(0.0); // 0% 错误概率
        assert!(!adapter.should_inject_error());
        
        adapter.disable_error_injection();
    }
}