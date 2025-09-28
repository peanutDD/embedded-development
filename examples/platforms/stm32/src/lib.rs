#![no_std]
#![no_main]

use panic_probe as _;

// 重新导出常用类型和特征
pub use cortex_m;
pub use cortex_m_rt;
pub use embedded_hal;
pub use stm32f4xx_hal as hal;

// 平台特定的GPIO定义
pub mod gpio {
  use crate::hal::gpio::{gpioa::*, gpiob::*, gpioc::*, Alternate, Input, Output, PushPull};

  // 常用引脚类型定义
  pub type LedPin = PA5<Output<PushPull>>;
  pub type ButtonPin = PC13<Input>;
  pub type UartTxPin = PA2<Alternate<7>>;
  pub type UartRxPin = PA3<Alternate<7>>;
  pub type I2cSclPin = PB8<Alternate<4>>;
  pub type I2cSdaPin = PB9<Alternate<4>>;
  pub type SpiSckPin = PA5<Alternate<5>>;
  pub type SpiMisoPin = PA6<Alternate<5>>;
  pub type SpiMosiPin = PA7<Alternate<5>>;
  pub type AdcPin = PA0<crate::hal::gpio::Analog>;
  pub type PwmPin = PA8<Alternate<1>>;
}

// 平台特定的外设定义
pub mod peripherals {
  use crate::hal::{
    adc::Adc,
    i2c::I2c,
    pac::{ADC1, I2C1, SPI1, TIM2, USART2},
    serial::Serial,
    spi::Spi,
    timer::{CounterUs, Timer},
  };

  // UART类型定义
  pub type UartType = Serial<USART2>;

  // I2C类型定义
  pub type I2cType = I2c<I2C1>;

  // SPI类型定义
  pub type SpiType = Spi<SPI1>;

  // ADC类型定义
  pub type AdcType = Adc<ADC1>;

  // 定时器类型定义
  pub type TimerType = Timer<TIM2>;
  pub type CounterType = CounterUs<TIM2>;
}

// 系统初始化
pub mod system {
  use crate::hal::{
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    rcc::RccExt,
  };

  pub struct SystemConfig {
    pub sysclk_mhz: u32,
    pub hclk_mhz: u32,
    pub pclk1_mhz: u32,
    pub pclk2_mhz: u32,
  }

  impl Default for SystemConfig {
    fn default() -> Self {
      Self {
        sysclk_mhz: 84,
        hclk_mhz: 84,
        pclk1_mhz: 42,
        pclk2_mhz: 84,
      }
    }
  }

  pub fn init_system(config: SystemConfig) -> CorePeripherals {
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let _clocks = rcc
      .cfgr
      .sysclk(config.sysclk_mhz.MHz())
      .hclk(config.hclk_mhz.MHz())
      .pclk1(config.pclk1_mhz.MHz())
      .pclk2(config.pclk2_mhz.MHz())
      .freeze();

    cp
  }
}

// 错误处理
#[derive(Debug, Clone, Copy)]
pub enum PlatformError {
  InitializationFailed,
  CommunicationError,
  TimeoutError,
  InvalidParameter,
  HardwareFault,
}

// 实用工具
pub mod utils {
  use cortex_m::asm;

  /// 延时函数（基于CPU周期）
  pub fn delay_cycles(cycles: u32) {
    for _ in 0..cycles {
      asm::nop();
    }
  }

  /// 毫秒延时（粗略估算，基于84MHz系统时钟）
  pub fn delay_ms(ms: u32) {
    delay_cycles(ms * 84_000);
  }

  /// 微秒延时（粗略估算，基于84MHz系统时钟）
  pub fn delay_us(us: u32) {
    delay_cycles(us * 84);
  }
}

// 调试支持
pub mod debug {
  use defmt_rtt as _;

  // 调试宏
  #[macro_export]
  macro_rules! debug_print {
        ($($arg:tt)*) => {
            defmt::info!($($arg)*);
        };
    }

  #[macro_export]
  macro_rules! debug_error {
        ($($arg:tt)*) => {
            defmt::error!($($arg)*);
        };
    }
}

// 中断处理
pub mod interrupts {
  use cortex_m::interrupt;

  /// 临界区执行
  pub fn critical_section<F, R>(f: F) -> R
  where
    F: FnOnce() -> R,
  {
    interrupt::free(|_| f())
  }

  /// 禁用中断
  pub fn disable_interrupts() {
    interrupt::disable();
  }

  /// 启用中断
  pub fn enable_interrupts() {
    unsafe {
      interrupt::enable();
    }
  }
}

// 内存管理工具
pub mod memory {
  use heapless::Vec;

  // 简单的内存管理示例
  pub fn create_buffer<const N: usize>() -> Vec<u8, N> {
    Vec::new()
  }

  // 创建固定大小的缓冲区
  pub fn create_fixed_buffer() -> Vec<u8, 64> {
    Vec::new()
  }
}

// 测试支持
#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_system_config() {
    let config = system::SystemConfig::default();
    assert_eq!(config.sysclk_mhz, 84);
    assert_eq!(config.hclk_mhz, 84);
    assert_eq!(config.pclk1_mhz, 42);
    assert_eq!(config.pclk2_mhz, 84);
  }

  #[test]
  fn test_platform_error() {
    let error = PlatformError::InitializationFailed;
    assert!(matches!(error, PlatformError::InitializationFailed));
  }
}
