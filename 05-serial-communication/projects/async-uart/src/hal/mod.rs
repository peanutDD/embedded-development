//! # HAL适配器模块
//!
//! 提供平台特定的硬件抽象层适配器，支持多种嵌入式平台。

use crate::{
    config::Config,
    error::{HardwareError, Result, UartError},
    traits::{AsyncRead, AsyncWrite, AsyncUart},
};
use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

/// 平台类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Platform {
    /// STM32系列
    Stm32,
    /// ESP32系列
    Esp32,
    /// RP2040 (Raspberry Pi Pico)
    Rp2040,
    /// nRF52系列
    Nrf52,
    /// 通用平台（用于测试）
    Generic,
}

/// UART引脚配置
#[derive(Debug, Clone)]
pub struct UartPins {
    /// TX引脚
    pub tx: Option<u8>,
    /// RX引脚
    pub rx: Option<u8>,
    /// RTS引脚（流控制）
    pub rts: Option<u8>,
    /// CTS引脚（流控制）
    pub cts: Option<u8>,
}

impl UartPins {
    /// 创建新的引脚配置
    pub fn new(tx: u8, rx: u8) -> Self {
        Self {
            tx: Some(tx),
            rx: Some(rx),
            rts: None,
            cts: None,
        }
    }
    
    /// 添加流控制引脚
    pub fn with_flow_control(mut self, rts: u8, cts: u8) -> Self {
        self.rts = Some(rts);
        self.cts = Some(cts);
        self
    }
    
    /// 验证引脚配置
    pub fn validate(&self) -> Result<()> {
        if self.tx.is_none() && self.rx.is_none() {
            return Err(HardwareError::InvalidPin.into());
        }
        
        Ok(())
    }
}

/// HAL适配器特征
pub trait HalAdapter: AsyncRead + AsyncWrite + Send + Sync {
    /// 获取平台类型
    fn platform(&self) -> Platform;
    
    /// 初始化UART
    fn init(&mut self, config: &Config, pins: &UartPins) -> impl Future<Output = Result<()>> + Send;
    
    /// 启用UART
    fn enable(&mut self) -> impl Future<Output = Result<()>> + Send;
    
    /// 禁用UART
    fn disable(&mut self) -> impl Future<Output = Result<()>> + Send;
    
    /// 重置UART
    fn reset(&mut self) -> impl Future<Output = Result<()>> + Send;
    
    /// 获取UART状态
    fn status(&self) -> UartStatus;
    
    /// 设置波特率
    fn set_baudrate(&mut self, baudrate: u32) -> impl Future<Output = Result<()>> + Send;
    
    /// 启用DMA
    fn enable_dma(&mut self, tx_channel: Option<u8>, rx_channel: Option<u8>) -> impl Future<Output = Result<()>> + Send;
    
    /// 禁用DMA
    fn disable_dma(&mut self) -> impl Future<Output = Result<()>> + Send;
    
    /// 启用中断
    fn enable_interrupts(&mut self, interrupts: UartInterrupts) -> impl Future<Output = Result<()>> + Send;
    
    /// 禁用中断
    fn disable_interrupts(&mut self, interrupts: UartInterrupts) -> impl Future<Output = Result<()>> + Send;
    
    /// 清除中断标志
    fn clear_interrupt_flags(&mut self, flags: UartInterrupts) -> impl Future<Output = Result<()>> + Send;
    
    /// 获取错误状态
    fn get_errors(&self) -> UartErrors;
    
    /// 清除错误状态
    fn clear_errors(&mut self) -> impl Future<Output = Result<()>> + Send;
}

/// UART状态
#[derive(Debug, Clone, Copy, Default)]
pub struct UartStatus {
    /// 是否已初始化
    pub initialized: bool,
    /// 是否已启用
    pub enabled: bool,
    /// 是否正在传输
    pub transmitting: bool,
    /// 是否正在接收
    pub receiving: bool,
    /// TX FIFO是否为空
    pub tx_empty: bool,
    /// RX FIFO是否有数据
    pub rx_not_empty: bool,
    /// 是否有错误
    pub has_errors: bool,
    /// DMA是否启用
    pub dma_enabled: bool,
}

/// UART中断类型
#[derive(Debug, Clone, Copy, Default)]
pub struct UartInterrupts {
    /// TX完成中断
    pub tx_complete: bool,
    /// RX完成中断
    pub rx_complete: bool,
    /// TX FIFO空中断
    pub tx_empty: bool,
    /// RX FIFO非空中断
    pub rx_not_empty: bool,
    /// 错误中断
    pub error: bool,
    /// 空闲线路中断
    pub idle: bool,
    /// DMA传输完成中断
    pub dma_complete: bool,
}

impl UartInterrupts {
    /// 创建所有中断启用的配置
    pub fn all() -> Self {
        Self {
            tx_complete: true,
            rx_complete: true,
            tx_empty: true,
            rx_not_empty: true,
            error: true,
            idle: true,
            dma_complete: true,
        }
    }
    
    /// 创建基本中断配置
    pub fn basic() -> Self {
        Self {
            tx_complete: true,
            rx_complete: true,
            error: true,
            ..Default::default()
        }
    }
    
    /// 创建DMA中断配置
    pub fn dma() -> Self {
        Self {
            dma_complete: true,
            error: true,
            ..Default::default()
        }
    }
}

/// UART错误状态
#[derive(Debug, Clone, Copy, Default)]
pub struct UartErrors {
    /// 帧错误
    pub frame_error: bool,
    /// 校验错误
    pub parity_error: bool,
    /// 噪声错误
    pub noise_error: bool,
    /// 溢出错误
    pub overrun_error: bool,
    /// DMA错误
    pub dma_error: bool,
}

impl UartErrors {
    /// 检查是否有任何错误
    pub fn has_errors(&self) -> bool {
        self.frame_error || self.parity_error || self.noise_error || self.overrun_error || self.dma_error
    }
    
    /// 清除所有错误
    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

/// HAL适配器工厂
pub struct HalAdapterFactory;

impl HalAdapterFactory {
    /// 创建平台特定的HAL适配器
    pub fn create(platform: Platform) -> Result<Box<dyn HalAdapter>> {
        match platform {
            #[cfg(feature = "stm32")]
            Platform::Stm32 => Ok(Box::new(stm32::Stm32HalAdapter::new())),
            
            #[cfg(feature = "esp32")]
            Platform::Esp32 => Ok(Box::new(esp32::Esp32HalAdapter::new())),
            
            #[cfg(feature = "rp2040")]
            Platform::Rp2040 => Ok(Box::new(rp2040::Rp2040HalAdapter::new())),
            
            #[cfg(feature = "nrf52")]
            Platform::Nrf52 => Ok(Box::new(nrf52::Nrf52HalAdapter::new())),
            
            Platform::Generic => Ok(Box::new(generic::GenericHalAdapter::new())),
            
            _ => Err(HardwareError::UnsupportedPlatform.into()),
        }
    }
    
    /// 自动检测平台
    pub fn detect_platform() -> Platform {
        #[cfg(feature = "stm32")]
        return Platform::Stm32;
        
        #[cfg(feature = "esp32")]
        return Platform::Esp32;
        
        #[cfg(feature = "rp2040")]
        return Platform::Rp2040;
        
        #[cfg(feature = "nrf52")]
        return Platform::Nrf52;
        
        Platform::Generic
    }
}

/// 通用UART适配器
pub struct GenericUartAdapter<T: HalAdapter> {
    /// HAL适配器
    adapter: T,
    /// 配置
    config: Option<Config>,
    /// 引脚配置
    pins: Option<UartPins>,
    /// 是否已初始化
    initialized: bool,
}

impl<T: HalAdapter> GenericUartAdapter<T> {
    /// 创建新的通用适配器
    pub fn new(adapter: T) -> Self {
        Self {
            adapter,
            config: None,
            pins: None,
            initialized: false,
        }
    }
    
    /// 初始化适配器
    pub async fn init(&mut self, config: Config, pins: UartPins) -> Result<()> {
        pins.validate()?;
        config.validate()?;
        
        self.adapter.init(&config, &pins).await?;
        self.config = Some(config);
        self.pins = Some(pins);
        self.initialized = true;
        
        Ok(())
    }
    
    /// 启动UART
    pub async fn start(&mut self) -> Result<()> {
        if !self.initialized {
            return Err(HardwareError::NotInitialized.into());
        }
        
        self.adapter.enable().await?;
        Ok(())
    }
    
    /// 停止UART
    pub async fn stop(&mut self) -> Result<()> {
        self.adapter.disable().await?;
        Ok(())
    }
    
    /// 重新配置UART
    pub async fn reconfigure(&mut self, config: Config) -> Result<()> {
        if !self.initialized {
            return Err(HardwareError::NotInitialized.into());
        }
        
        config.validate()?;
        
        // 停止UART
        self.adapter.disable().await?;
        
        // 重新初始化
        if let Some(pins) = &self.pins {
            self.adapter.init(&config, pins).await?;
        }
        
        // 重新启动
        self.adapter.enable().await?;
        
        self.config = Some(config);
        Ok(())
    }
    
    /// 获取适配器引用
    pub fn adapter(&self) -> &T {
        &self.adapter
    }
    
    /// 获取可变适配器引用
    pub fn adapter_mut(&mut self) -> &mut T {
        &mut self.adapter
    }
    
    /// 获取配置
    pub fn config(&self) -> Option<&Config> {
        self.config.as_ref()
    }
    
    /// 获取引脚配置
    pub fn pins(&self) -> Option<&UartPins> {
        self.pins.as_ref()
    }
    
    /// 检查是否已初始化
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}

impl<T: HalAdapter> AsyncRead for GenericUartAdapter<T> {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut [u8],
    ) -> Poll<Result<usize>> {
        Pin::new(&mut self.adapter).poll_read(cx, buf)
    }
}

impl<T: HalAdapter> AsyncWrite for GenericUartAdapter<T> {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<Result<usize>> {
        Pin::new(&mut self.adapter).poll_write(cx, buf)
    }
    
    fn poll_flush(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<()>> {
        Pin::new(&mut self.adapter).poll_flush(cx)
    }
    
    fn poll_close(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<()>> {
        Pin::new(&mut self.adapter).poll_close(cx)
    }
}

impl<T: HalAdapter> AsyncUart for GenericUartAdapter<T> {
    fn poll_read_with_timeout(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut [u8],
        timeout: core::time::Duration,
    ) -> Poll<Result<usize>> {
        // 实现带超时的读取
        Pin::new(&mut self.adapter).poll_read(cx, buf)
    }
    
    fn poll_write_with_timeout(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
        timeout: core::time::Duration,
    ) -> Poll<Result<usize>> {
        // 实现带超时的写入
        Pin::new(&mut self.adapter).poll_write(cx, buf)
    }
}

// 声明平台特定的模块
#[cfg(feature = "stm32")]
pub mod stm32;

#[cfg(feature = "esp32")]
pub mod esp32;

#[cfg(feature = "rp2040")]
pub mod rp2040;

#[cfg(feature = "nrf52")]
pub mod nrf52;

pub mod generic;

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_uart_pins_creation() {
        let pins = UartPins::new(1, 2);
        assert_eq!(pins.tx, Some(1));
        assert_eq!(pins.rx, Some(2));
        assert_eq!(pins.rts, None);
        assert_eq!(pins.cts, None);
        
        assert!(pins.validate().is_ok());
    }
    
    #[test]
    fn test_uart_pins_with_flow_control() {
        let pins = UartPins::new(1, 2).with_flow_control(3, 4);
        assert_eq!(pins.tx, Some(1));
        assert_eq!(pins.rx, Some(2));
        assert_eq!(pins.rts, Some(3));
        assert_eq!(pins.cts, Some(4));
        
        assert!(pins.validate().is_ok());
    }
    
    #[test]
    fn test_uart_pins_validation() {
        let pins = UartPins {
            tx: None,
            rx: None,
            rts: None,
            cts: None,
        };
        
        assert!(pins.validate().is_err());
    }
    
    #[test]
    fn test_uart_interrupts() {
        let interrupts = UartInterrupts::all();
        assert!(interrupts.tx_complete);
        assert!(interrupts.rx_complete);
        assert!(interrupts.error);
        
        let basic = UartInterrupts::basic();
        assert!(basic.tx_complete);
        assert!(basic.rx_complete);
        assert!(basic.error);
        assert!(!basic.tx_empty);
        
        let dma = UartInterrupts::dma();
        assert!(dma.dma_complete);
        assert!(dma.error);
        assert!(!dma.tx_complete);
    }
    
    #[test]
    fn test_uart_errors() {
        let mut errors = UartErrors::default();
        assert!(!errors.has_errors());
        
        errors.frame_error = true;
        assert!(errors.has_errors());
        
        errors.clear();
        assert!(!errors.has_errors());
    }
    
    #[test]
    fn test_platform_detection() {
        let platform = HalAdapterFactory::detect_platform();
        // 在测试环境中应该返回Generic
        assert_eq!(platform, Platform::Generic);
    }
}