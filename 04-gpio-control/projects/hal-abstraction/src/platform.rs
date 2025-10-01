//! # 平台抽象模块
//! 
//! 提供不同硬件平台的GPIO HAL实现

pub mod mock;

#[cfg(feature = "stm32f4")]
pub mod stm32;

#[cfg(feature = "esp32")]
pub mod esp32;

#[cfg(feature = "rp2040")]
pub mod rp2040;

#[cfg(feature = "nrf52")]
pub mod nrf52;

/// 平台特征
pub trait Platform {
    /// 平台名称
    const NAME: &'static str;
    
    /// 支持的引脚数量
    const MAX_PINS: usize;
    
    /// 支持的端口数量
    const MAX_PORTS: usize;
    
    /// 是否支持中断
    const SUPPORTS_INTERRUPTS: bool;
    
    /// 是否支持DMA
    const SUPPORTS_DMA: bool;
    
    /// 是否支持模拟功能
    const SUPPORTS_ANALOG: bool;
    
    /// 获取平台信息
    fn platform_info() -> PlatformInfo {
        PlatformInfo {
            name: Self::NAME,
            max_pins: Self::MAX_PINS,
            max_ports: Self::MAX_PORTS,
            supports_interrupts: Self::SUPPORTS_INTERRUPTS,
            supports_dma: Self::SUPPORTS_DMA,
            supports_analog: Self::SUPPORTS_ANALOG,
        }
    }
}

/// 平台信息结构体
#[derive(Debug, Clone)]
pub struct PlatformInfo {
    pub name: &'static str,
    pub max_pins: usize,
    pub max_ports: usize,
    pub supports_interrupts: bool,
    pub supports_dma: bool,
    pub supports_analog: bool,
}

impl core::fmt::Display for PlatformInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "平台信息:")?;
        writeln!(f, "  名称: {}", self.name)?;
        writeln!(f, "  最大引脚数: {}", self.max_pins)?;
        writeln!(f, "  最大端口数: {}", self.max_ports)?;
        writeln!(f, "  支持中断: {}", if self.supports_interrupts { "是" } else { "否" })?;
        writeln!(f, "  支持DMA: {}", if self.supports_dma { "是" } else { "否" })?;
        writeln!(f, "  支持模拟: {}", if self.supports_analog { "是" } else { "否" })?;
        Ok(())
    }
}