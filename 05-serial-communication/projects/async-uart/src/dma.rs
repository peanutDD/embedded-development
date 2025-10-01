//! # DMA支持模块
//!
//! 提供异步DMA传输功能，支持高效的数据传输。

use crate::{
    buffer::{DmaBuffer, DmaBufferConfig, DmaDescriptor, DmaDirection, DmaMode},
    error::{DmaError, Result, UartError},
    traits::{AsyncRead, AsyncWrite},
};
use core::{
    future::Future,
    pin::Pin,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    task::{Context, Poll, Waker},
};
use futures_util::ready;

/// DMA通道状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaChannelState {
    /// 空闲
    Idle,
    /// 配置中
    Configuring,
    /// 传输中
    Transferring,
    /// 暂停
    Paused,
    /// 完成
    Complete,
    /// 错误
    Error,
}

/// DMA传输类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaTransferType {
    /// 单次传输
    Single,
    /// 循环传输
    Circular,
    /// 双缓冲传输
    DoubleBuffer,
    /// 链式传输
    Linked,
}

/// DMA优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaPriority {
    /// 低优先级
    Low = 0,
    /// 中等优先级
    Medium = 1,
    /// 高优先级
    High = 2,
    /// 非常高优先级
    VeryHigh = 3,
}

/// DMA传输配置
#[derive(Debug, Clone)]
pub struct DmaTransferConfig {
    /// 传输方向
    pub direction: DmaDirection,
    /// 传输模式
    pub mode: DmaMode,
    /// 传输类型
    pub transfer_type: DmaTransferType,
    /// 优先级
    pub priority: DmaPriority,
    /// 数据宽度（字节）
    pub data_width: usize,
    /// 突发长度
    pub burst_length: usize,
    /// 是否启用中断
    pub enable_interrupt: bool,
    /// 是否启用半传输中断
    pub enable_half_transfer_interrupt: bool,
    /// 是否启用错误中断
    pub enable_error_interrupt: bool,
    /// 外设地址
    pub peripheral_address: Option<u32>,
    /// 外设地址递增
    pub peripheral_increment: bool,
    /// 内存地址递增
    pub memory_increment: bool,
}

impl Default for DmaTransferConfig {
    fn default() -> Self {
        Self {
            direction: DmaDirection::MemoryToPeripheral,
            mode: DmaMode::Single,
            transfer_type: DmaTransferType::Single,
            priority: DmaPriority::Medium,
            data_width: 1,
            burst_length: 1,
            enable_interrupt: true,
            enable_half_transfer_interrupt: false,
            enable_error_interrupt: true,
            peripheral_address: None,
            peripheral_increment: false,
            memory_increment: true,
        }
    }
}

impl DmaTransferConfig {
    /// 创建新的传输配置
    pub fn new(direction: DmaDirection) -> Self {
        Self {
            direction,
            ..Default::default()
        }
    }
    
    /// 设置传输模式
    pub fn mode(mut self, mode: DmaMode) -> Self {
        self.mode = mode;
        self
    }
    
    /// 设置优先级
    pub fn priority(mut self, priority: DmaPriority) -> Self {
        self.priority = priority;
        self
    }
    
    /// 设置数据宽度
    pub fn data_width(mut self, width: usize) -> Self {
        self.data_width = width;
        self
    }
    
    /// 设置外设地址
    pub fn peripheral_address(mut self, address: u32) -> Self {
        self.peripheral_address = Some(address);
        self
    }
    
    /// 启用循环模式
    pub fn circular(mut self) -> Self {
        self.mode = DmaMode::Circular;
        self.transfer_type = DmaTransferType::Circular;
        self
    }
    
    /// 验证配置
    pub fn validate(&self) -> Result<()> {
        if self.data_width == 0 || self.data_width > 4 {
            return Err(DmaError::ConfigError.into());
        }
        
        if self.burst_length == 0 || self.burst_length > 16 {
            return Err(DmaError::ConfigError.into());
        }
        
        match self.direction {
            DmaDirection::MemoryToPeripheral | DmaDirection::PeripheralToMemory => {
                if self.peripheral_address.is_none() {
                    return Err(DmaError::ConfigError.into());
                }
            }
            DmaDirection::MemoryToMemory => {
                // 内存到内存传输不需要外设地址
            }
        }
        
        Ok(())
    }
}

/// DMA传输统计
#[derive(Debug, Clone, Copy, Default)]
pub struct DmaTransferStats {
    /// 总传输次数
    pub total_transfers: usize,
    /// 成功传输次数
    pub successful_transfers: usize,
    /// 失败传输次数
    pub failed_transfers: usize,
    /// 总传输字节数
    pub total_bytes: usize,
    /// 平均传输速度（字节/秒）
    pub average_speed: f32,
    /// 最后传输时间戳
    pub last_transfer_time: u64,
}

impl DmaTransferStats {
    /// 创建新的统计
    pub fn new() -> Self {
        Self::default()
    }
    
    /// 记录成功传输
    pub fn record_success(&mut self, bytes: usize, duration_us: u64) {
        self.total_transfers += 1;
        self.successful_transfers += 1;
        self.total_bytes += bytes;
        
        if duration_us > 0 {
            let speed = (bytes as f32 * 1_000_000.0) / duration_us as f32;
            self.average_speed = (self.average_speed * (self.successful_transfers - 1) as f32 + speed) 
                / self.successful_transfers as f32;
        }
        
        self.last_transfer_time = self.get_current_time();
    }
    
    /// 记录失败传输
    pub fn record_failure(&mut self) {
        self.total_transfers += 1;
        self.failed_transfers += 1;
        self.last_transfer_time = self.get_current_time();
    }
    
    /// 获取成功率
    pub fn success_rate(&self) -> f32 {
        if self.total_transfers == 0 {
            0.0
        } else {
            self.successful_transfers as f32 / self.total_transfers as f32
        }
    }
    
    /// 重置统计
    pub fn reset(&mut self) {
        *self = Self::new();
    }
    
    /// 获取当前时间（微秒）
    fn get_current_time(&self) -> u64 {
        // 在实际实现中，这里应该使用平台特定的时间函数
        #[cfg(feature = "embassy")]
        {
            embassy_time::Instant::now().as_micros()
        }
        #[cfg(not(feature = "embassy"))]
        {
            // 简单的计数器，实际实现中应该使用更好的方法
            0
        }
    }
}

/// DMA通道
pub struct DmaChannel {
    /// 通道ID
    id: usize,
    /// 通道状态
    state: AtomicUsize, // 使用usize存储DmaChannelState
    /// 传输配置
    config: Option<DmaTransferConfig>,
    /// 当前缓冲区
    buffer: Option<DmaBuffer>,
    /// 传输统计
    stats: DmaTransferStats,
    /// 等待传输完成的任务
    completion_waker: Option<Waker>,
    /// 是否正在传输
    transferring: AtomicBool,
    /// 传输的字节数
    transferred_bytes: AtomicUsize,
    /// 总传输字节数
    total_bytes: AtomicUsize,
}

impl DmaChannel {
    /// 创建新的DMA通道
    pub fn new(id: usize) -> Self {
        Self {
            id,
            state: AtomicUsize::new(DmaChannelState::Idle as usize),
            config: None,
            buffer: None,
            stats: DmaTransferStats::new(),
            completion_waker: None,
            transferring: AtomicBool::new(false),
            transferred_bytes: AtomicUsize::new(0),
            total_bytes: AtomicUsize::new(0),
        }
    }
    
    /// 获取通道ID
    pub fn id(&self) -> usize {
        self.id
    }
    
    /// 获取通道状态
    pub fn state(&self) -> DmaChannelState {
        match self.state.load(Ordering::Acquire) {
            0 => DmaChannelState::Idle,
            1 => DmaChannelState::Configuring,
            2 => DmaChannelState::Transferring,
            3 => DmaChannelState::Paused,
            4 => DmaChannelState::Complete,
            5 => DmaChannelState::Error,
            _ => DmaChannelState::Error,
        }
    }
    
    /// 设置通道状态
    pub fn set_state(&self, state: DmaChannelState) {
        self.state.store(state as usize, Ordering::Release);
    }
    
    /// 检查是否正在传输
    pub fn is_transferring(&self) -> bool {
        self.transferring.load(Ordering::Acquire)
    }
    
    /// 配置DMA传输
    pub fn configure(&mut self, config: DmaTransferConfig, buffer: DmaBuffer) -> Result<()> {
        if self.is_transferring() {
            return Err(DmaError::ChannelBusy.into());
        }
        
        config.validate()?;
        
        self.set_state(DmaChannelState::Configuring);
        self.config = Some(config);
        self.buffer = Some(buffer);
        
        Ok(())
    }
    
    /// 开始DMA传输
    pub fn start_transfer(&mut self) -> Result<()> {
        if self.config.is_none() || self.buffer.is_none() {
            return Err(DmaError::ConfigError.into());
        }
        
        if self.is_transferring() {
            return Err(DmaError::ChannelBusy.into());
        }
        
        let buffer = self.buffer.as_mut().unwrap();
        let config = self.config.as_ref().unwrap();
        
        // 准备缓冲区
        buffer.prepare_transfer(config.direction, config.mode)?;
        buffer.start_transfer()?;
        
        // 设置传输状态
        self.transferring.store(true, Ordering::Release);
        self.set_state(DmaChannelState::Transferring);
        self.transferred_bytes.store(0, Ordering::Release);
        self.total_bytes.store(buffer.len(), Ordering::Release);
        
        // 在实际实现中，这里应该配置硬件DMA控制器
        self.configure_hardware_dma()?;
        
        Ok(())
    }
    
    /// 停止DMA传输
    pub fn stop_transfer(&mut self) -> Result<()> {
        if !self.is_transferring() {
            return Ok(());
        }
        
        // 停止硬件DMA
        self.stop_hardware_dma()?;
        
        // 中止缓冲区传输
        if let Some(buffer) = self.buffer.as_mut() {
            buffer.abort_transfer()?;
        }
        
        self.transferring.store(false, Ordering::Release);
        self.set_state(DmaChannelState::Idle);
        
        // 唤醒等待的任务
        if let Some(waker) = self.completion_waker.take() {
            waker.wake();
        }
        
        Ok(())
    }
    
    /// 暂停DMA传输
    pub fn pause_transfer(&mut self) -> Result<()> {
        if !self.is_transferring() {
            return Err(DmaError::TransferError.into());
        }
        
        // 暂停硬件DMA
        self.pause_hardware_dma()?;
        self.set_state(DmaChannelState::Paused);
        
        Ok(())
    }
    
    /// 恢复DMA传输
    pub fn resume_transfer(&mut self) -> Result<()> {
        if self.state() != DmaChannelState::Paused {
            return Err(DmaError::TransferError.into());
        }
        
        // 恢复硬件DMA
        self.resume_hardware_dma()?;
        self.set_state(DmaChannelState::Transferring);
        
        Ok(())
    }
    
    /// 等待传输完成
    pub async fn wait_for_completion(&mut self) -> Result<usize> {
        DmaTransferFuture::new(self).await
    }
    
    /// 处理DMA中断
    pub fn handle_interrupt(&mut self, interrupt_type: DmaInterruptType) -> Result<()> {
        match interrupt_type {
            DmaInterruptType::TransferComplete => {
                self.handle_transfer_complete()?;
            }
            DmaInterruptType::HalfTransfer => {
                self.handle_half_transfer()?;
            }
            DmaInterruptType::TransferError => {
                self.handle_transfer_error()?;
            }
        }
        
        Ok(())
    }
    
    /// 处理传输完成中断
    fn handle_transfer_complete(&mut self) -> Result<()> {
        let transferred = self.total_bytes.load(Ordering::Acquire);
        
        // 完成缓冲区传输
        if let Some(buffer) = self.buffer.as_mut() {
            buffer.complete_transfer(transferred)?;
        }
        
        self.transferring.store(false, Ordering::Release);
        self.set_state(DmaChannelState::Complete);
        self.transferred_bytes.store(transferred, Ordering::Release);
        
        // 更新统计
        self.stats.record_success(transferred, 0); // 实际实现中应该计算传输时间
        
        // 唤醒等待的任务
        if let Some(waker) = self.completion_waker.take() {
            waker.wake();
        }
        
        Ok(())
    }
    
    /// 处理半传输中断
    fn handle_half_transfer(&mut self) -> Result<()> {
        let half_bytes = self.total_bytes.load(Ordering::Acquire) / 2;
        self.transferred_bytes.store(half_bytes, Ordering::Release);
        
        // 在双缓冲模式下，这里可以切换缓冲区
        if let Some(config) = &self.config {
            if config.transfer_type == DmaTransferType::DoubleBuffer {
                // 切换缓冲区逻辑
            }
        }
        
        Ok(())
    }
    
    /// 处理传输错误中断
    fn handle_transfer_error(&mut self) -> Result<()> {
        // 停止传输
        if let Some(buffer) = self.buffer.as_mut() {
            buffer.abort_transfer()?;
        }
        
        self.transferring.store(false, Ordering::Release);
        self.set_state(DmaChannelState::Error);
        
        // 更新统计
        self.stats.record_failure();
        
        // 唤醒等待的任务
        if let Some(waker) = self.completion_waker.take() {
            waker.wake();
        }
        
        Ok(())
    }
    
    /// 获取传输进度
    pub fn progress(&self) -> f32 {
        let total = self.total_bytes.load(Ordering::Acquire);
        if total == 0 {
            return 0.0;
        }
        
        let transferred = self.transferred_bytes.load(Ordering::Acquire);
        transferred as f32 / total as f32
    }
    
    /// 获取传输统计
    pub fn stats(&self) -> DmaTransferStats {
        self.stats
    }
    
    /// 重置通道
    pub fn reset(&mut self) -> Result<()> {
        if self.is_transferring() {
            self.stop_transfer()?;
        }
        
        self.config = None;
        self.buffer = None;
        self.stats.reset();
        self.completion_waker = None;
        self.transferred_bytes.store(0, Ordering::Release);
        self.total_bytes.store(0, Ordering::Release);
        self.set_state(DmaChannelState::Idle);
        
        Ok(())
    }
    
    /// 配置硬件DMA（平台相关）
    fn configure_hardware_dma(&self) -> Result<()> {
        // 在实际实现中，这里应该配置具体的硬件DMA控制器
        // 例如：STM32的DMA、ESP32的DMA、RP2040的DMA等
        
        #[cfg(feature = "stm32")]
        {
            // STM32 DMA配置
            // self.configure_stm32_dma()?;
        }
        
        #[cfg(feature = "esp32")]
        {
            // ESP32 DMA配置
            // self.configure_esp32_dma()?;
        }
        
        #[cfg(feature = "rp2040")]
        {
            // RP2040 DMA配置
            // self.configure_rp2040_dma()?;
        }
        
        Ok(())
    }
    
    /// 停止硬件DMA
    fn stop_hardware_dma(&self) -> Result<()> {
        // 平台相关的DMA停止代码
        Ok(())
    }
    
    /// 暂停硬件DMA
    fn pause_hardware_dma(&self) -> Result<()> {
        // 平台相关的DMA暂停代码
        Ok(())
    }
    
    /// 恢复硬件DMA
    fn resume_hardware_dma(&self) -> Result<()> {
        // 平台相关的DMA恢复代码
        Ok(())
    }
}

/// DMA中断类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaInterruptType {
    /// 传输完成
    TransferComplete,
    /// 半传输
    HalfTransfer,
    /// 传输错误
    TransferError,
}

/// DMA传输Future
struct DmaTransferFuture<'a> {
    channel: &'a mut DmaChannel,
}

impl<'a> DmaTransferFuture<'a> {
    fn new(channel: &'a mut DmaChannel) -> Self {
        Self { channel }
    }
}

impl<'a> Future for DmaTransferFuture<'a> {
    type Output = Result<usize>;
    
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let channel = &mut self.channel;
        
        match channel.state() {
            DmaChannelState::Complete => {
                let transferred = channel.transferred_bytes.load(Ordering::Acquire);
                Poll::Ready(Ok(transferred))
            }
            DmaChannelState::Error => {
                Poll::Ready(Err(DmaError::TransferError.into()))
            }
            DmaChannelState::Transferring | DmaChannelState::Paused => {
                channel.completion_waker = Some(cx.waker().clone());
                Poll::Pending
            }
            _ => {
                Poll::Ready(Err(DmaError::TransferError.into()))
            }
        }
    }
}

/// DMA管理器
pub struct DmaManager {
    /// DMA通道
    channels: Vec<DmaChannel>,
    /// 可用通道
    available_channels: Vec<usize>,
}

impl DmaManager {
    /// 创建新的DMA管理器
    pub fn new(num_channels: usize) -> Self {
        let mut channels = Vec::with_capacity(num_channels);
        let mut available_channels = Vec::with_capacity(num_channels);
        
        for i in 0..num_channels {
            channels.push(DmaChannel::new(i));
            available_channels.push(i);
        }
        
        Self {
            channels,
            available_channels,
        }
    }
    
    /// 分配DMA通道
    pub fn allocate_channel(&mut self) -> Option<usize> {
        self.available_channels.pop()
    }
    
    /// 释放DMA通道
    pub fn release_channel(&mut self, channel_id: usize) -> Result<()> {
        if channel_id >= self.channels.len() {
            return Err(DmaError::InvalidChannel.into());
        }
        
        // 重置通道
        self.channels[channel_id].reset()?;
        
        // 添加到可用列表
        if !self.available_channels.contains(&channel_id) {
            self.available_channels.push(channel_id);
        }
        
        Ok(())
    }
    
    /// 获取通道
    pub fn get_channel(&mut self, channel_id: usize) -> Option<&mut DmaChannel> {
        self.channels.get_mut(channel_id)
    }
    
    /// 获取可用通道数量
    pub fn available_count(&self) -> usize {
        self.available_channels.len()
    }
    
    /// 获取总通道数量
    pub fn total_count(&self) -> usize {
        self.channels.len()
    }
    
    /// 处理所有通道的中断
    pub fn handle_interrupts(&mut self) -> Result<()> {
        for channel in &mut self.channels {
            if channel.is_transferring() {
                // 检查硬件中断状态并处理
                // 这里应该根据具体硬件实现
            }
        }
        
        Ok(())
    }
}

/// DMA读取器
pub struct DmaReader {
    channel: DmaChannel,
    buffer: DmaBuffer,
}

impl DmaReader {
    /// 创建新的DMA读取器
    pub fn new(channel: DmaChannel, buffer_config: DmaBufferConfig) -> Result<Self> {
        let buffer = DmaBuffer::new(buffer_config)?;
        
        Ok(Self {
            channel,
            buffer,
        })
    }
    
    /// 开始读取
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        // 配置DMA传输
        let config = DmaTransferConfig::new(DmaDirection::PeripheralToMemory);
        self.channel.configure(config, self.buffer.clone())?;
        
        // 开始传输
        self.channel.start_transfer()?;
        
        // 等待完成
        let transferred = self.channel.wait_for_completion().await?;
        
        // 从缓冲区读取数据
        self.buffer.read(buf)
    }
}

impl AsyncRead for DmaReader {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut [u8],
    ) -> Poll<Result<usize>> {
        // 实现异步读取逻辑
        match self.buffer.poll_read(cx, buf) {
            Poll::Ready(result) => Poll::Ready(result),
            Poll::Pending => Poll::Pending,
        }
    }
}

/// DMA写入器
pub struct DmaWriter {
    channel: DmaChannel,
    buffer: DmaBuffer,
}

impl DmaWriter {
    /// 创建新的DMA写入器
    pub fn new(channel: DmaChannel, buffer_config: DmaBufferConfig) -> Result<Self> {
        let buffer = DmaBuffer::new(buffer_config)?;
        
        Ok(Self {
            channel,
            buffer,
        })
    }
    
    /// 开始写入
    pub async fn write(&mut self, buf: &[u8]) -> Result<usize> {
        // 写入数据到缓冲区
        let written = self.buffer.write(buf)?;
        
        // 配置DMA传输
        let config = DmaTransferConfig::new(DmaDirection::MemoryToPeripheral);
        self.channel.configure(config, self.buffer.clone())?;
        
        // 开始传输
        self.channel.start_transfer()?;
        
        // 等待完成
        self.channel.wait_for_completion().await?;
        
        Ok(written)
    }
}

impl AsyncWrite for DmaWriter {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<Result<usize>> {
        // 实现异步写入逻辑
        match self.buffer.poll_write(cx, buf) {
            Poll::Ready(result) => Poll::Ready(result),
            Poll::Pending => Poll::Pending,
        }
    }
    
    fn poll_flush(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<()>> {
        self.buffer.poll_flush(cx)
    }
    
    fn poll_close(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<()>> {
        self.buffer.poll_close(cx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_dma_channel_creation() {
        let channel = DmaChannel::new(0);
        
        assert_eq!(channel.id(), 0);
        assert_eq!(channel.state(), DmaChannelState::Idle);
        assert!(!channel.is_transferring());
    }
    
    #[test]
    fn test_dma_transfer_config() {
        let config = DmaTransferConfig::new(DmaDirection::MemoryToPeripheral)
            .mode(DmaMode::Circular)
            .priority(DmaPriority::High)
            .data_width(2)
            .peripheral_address(0x40000000);
        
        assert_eq!(config.direction, DmaDirection::MemoryToPeripheral);
        assert_eq!(config.mode, DmaMode::Circular);
        assert_eq!(config.priority, DmaPriority::High);
        assert_eq!(config.data_width, 2);
        assert_eq!(config.peripheral_address, Some(0x40000000));
        
        assert!(config.validate().is_ok());
    }
    
    #[test]
    fn test_dma_transfer_config_validation() {
        // 有效配置
        let config = DmaTransferConfig::new(DmaDirection::MemoryToPeripheral)
            .peripheral_address(0x40000000);
        assert!(config.validate().is_ok());
        
        // 无效数据宽度
        let config = DmaTransferConfig::new(DmaDirection::MemoryToPeripheral)
            .data_width(0)
            .peripheral_address(0x40000000);
        assert!(config.validate().is_err());
        
        // 缺少外设地址
        let config = DmaTransferConfig::new(DmaDirection::MemoryToPeripheral);
        assert!(config.validate().is_err());
        
        // 内存到内存传输不需要外设地址
        let config = DmaTransferConfig::new(DmaDirection::MemoryToMemory);
        assert!(config.validate().is_ok());
    }
    
    #[test]
    fn test_dma_manager() {
        let mut manager = DmaManager::new(4);
        
        assert_eq!(manager.total_count(), 4);
        assert_eq!(manager.available_count(), 4);
        
        // 分配通道
        let channel_id = manager.allocate_channel().unwrap();
        assert_eq!(manager.available_count(), 3);
        
        // 释放通道
        manager.release_channel(channel_id).unwrap();
        assert_eq!(manager.available_count(), 4);
    }
    
    #[test]
    fn test_dma_transfer_stats() {
        let mut stats = DmaTransferStats::new();
        
        assert_eq!(stats.total_transfers, 0);
        assert_eq!(stats.success_rate(), 0.0);
        
        // 记录成功传输
        stats.record_success(1024, 1000);
        assert_eq!(stats.total_transfers, 1);
        assert_eq!(stats.successful_transfers, 1);
        assert_eq!(stats.total_bytes, 1024);
        assert_eq!(stats.success_rate(), 1.0);
        
        // 记录失败传输
        stats.record_failure();
        assert_eq!(stats.total_transfers, 2);
        assert_eq!(stats.failed_transfers, 1);
        assert_eq!(stats.success_rate(), 0.5);
    }
    
    #[test]
    fn test_dma_channel_configuration() {
        let mut channel = DmaChannel::new(0);
        let buffer_config = DmaBufferConfig::new(1024);
        let buffer = DmaBuffer::new(buffer_config).unwrap();
        let config = DmaTransferConfig::new(DmaDirection::MemoryToPeripheral)
            .peripheral_address(0x40000000);
        
        // 配置通道
        assert!(channel.configure(config, buffer).is_ok());
        assert_eq!(channel.state(), DmaChannelState::Configuring);
    }
}