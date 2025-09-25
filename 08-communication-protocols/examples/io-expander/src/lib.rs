#![no_std]

//! PCF8574 I2C IO扩展器库
//! 
//! 本库提供了PCF8574 I2C IO扩展器的完整驱动实现，
//! 支持GPIO扩展、输入输出控制、中断处理等功能。

pub mod pcf8574;

pub use pcf8574::*;

/// 库版本信息
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// PCF8574默认I2C地址
pub const PCF8574_DEFAULT_ADDR: u8 = 0x20;

/// PCF8574地址范围
pub const PCF8574_ADDR_MIN: u8 = 0x20;
pub const PCF8574_ADDR_MAX: u8 = 0x27;

/// GPIO引脚掩码
pub mod pin_mask {
    pub const P0: u8 = 0x01;
    pub const P1: u8 = 0x02;
    pub const P2: u8 = 0x04;
    pub const P3: u8 = 0x08;
    pub const P4: u8 = 0x10;
    pub const P5: u8 = 0x20;
    pub const P6: u8 = 0x40;
    pub const P7: u8 = 0x80;
    pub const ALL: u8 = 0xFF;
}

/// 常用引脚组合
pub mod pin_groups {
    pub const LEDS: u8 = 0x0F;      // P0-P3 用于LED
    pub const BUTTONS: u8 = 0xF0;   // P4-P7 用于按键
    pub const LOWER_NIBBLE: u8 = 0x0F;
    pub const UPPER_NIBBLE: u8 = 0xF0;
}