//! 跨平台抽象特征定义

use crate::*;

/// GPIO抽象特征
pub trait GpioAbstraction {
    type Pin: embedded_hal::digital::OutputPin + embedded_hal::digital::InputPin;
    type Error;
    
    fn get_pin(&mut self, pin_id: u8) -> Result<&mut Self::Pin>;
    fn set_pin_high(&mut self, pin_id: u8) -> Result<()>;
    fn set_pin_low(&mut self, pin_id: u8) -> Result<()>;
    fn read_pin(&self, pin_id: u8) -> Result<bool>;
    fn toggle_pin(&mut self, pin_id: u8) -> Result<()>;
}

/// 传感器抽象特征
pub trait SensorAbstraction {
    type Reading;
    type Error;
    
    fn read(&mut self) -> nb::Result<Self::Reading, Self::Error>;
    fn init(&mut self) -> Result<()>;
    fn calibrate(&mut self) -> Result<()>;
    fn get_info(&self) -> crate::sensor::SensorInfo;
}

/// 通信抽象特征
pub trait CommunicationAbstraction {
    type Data;
    type Error;
    
    fn send(&mut self, data: &Self::Data) -> nb::Result<(), Self::Error>;
    fn receive(&mut self) -> nb::Result<Self::Data, Self::Error>;
    fn is_connected(&self) -> bool;
    fn get_status(&self) -> crate::communication::CommunicationStatus;
}

/// 定时器抽象特征
pub trait TimerAbstraction {
    type Duration;
    type Error;
    
    fn start(&mut self, duration: Self::Duration) -> Result<()>;
    fn stop(&mut self) -> Result<()>;
    fn is_expired(&self) -> bool;
    fn reset(&mut self) -> Result<()>;
}