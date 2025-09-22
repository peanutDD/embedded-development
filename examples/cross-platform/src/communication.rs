//! 通信模块

use crate::*;
use crate::traits::CommunicationAbstraction;
use heapless::Vec;

/// 通信状态
#[derive(Debug, Clone, Copy)]
pub enum CommunicationStatus {
    Disconnected,
    Connecting,
    Connected,
    Error,
}

/// 数据包
#[derive(Debug, Clone)]
pub struct DataPacket {
    pub id: u16,
    pub data: Vec<u8, 256>,
    pub timestamp: u32,
    pub checksum: u16,
}

impl DataPacket {
    pub fn new(id: u16, data: &[u8]) -> Result<Self> {
        let mut packet_data = Vec::new();
        for &byte in data {
            packet_data.push(byte).map_err(|_| Error::ResourceExhausted)?;
        }
        
        let checksum = Self::calculate_checksum(data);
        
        Ok(Self {
            id,
            data: packet_data,
            timestamp: 0, // 需要获取当前时间戳
            checksum,
        })
    }
    
    fn calculate_checksum(data: &[u8]) -> u16 {
        data.iter().map(|&b| b as u16).sum()
    }
    
    pub fn verify_checksum(&self) -> bool {
        Self::calculate_checksum(&self.data) == self.checksum
    }
}

/// 通信管理器
pub struct CommunicationManager<T: CommunicationAbstraction> {
    interface: T,
    tx_buffer: Vec<DataPacket, 16>,
    rx_buffer: Vec<DataPacket, 16>,
}

impl<T: CommunicationAbstraction> CommunicationManager<T> {
    pub fn new(interface: T) -> Self {
        Self {
            interface,
            tx_buffer: Vec::new(),
            rx_buffer: Vec::new(),
        }
    }
    
    pub fn send_packet(&mut self, packet: DataPacket) -> Result<()> {
        self.tx_buffer.push(packet).map_err(|_| Error::ResourceExhausted)
    }
    
    pub fn receive_packet(&mut self) -> Result<Option<DataPacket>> {
        if self.rx_buffer.is_empty() {
            Ok(None)
        } else {
            Ok(self.rx_buffer.pop())
        }
    }
    
    pub fn process_buffers(&mut self) -> Result<()> {
        // 处理发送缓冲区
        while let Some(_packet) = self.tx_buffer.pop() {
            // 这里需要将DataPacket转换为T::Data
            // 由于类型约束，这里提供一个示例实现
        }
        
        // 处理接收缓冲区
        // 这里需要从接口接收数据并转换为DataPacket
        
        Ok(())
    }
    
    pub fn get_status(&self) -> CommunicationStatus {
        self.interface.get_status()
    }
    
    pub fn is_connected(&self) -> bool {
        self.interface.is_connected()
    }
}