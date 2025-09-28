//! UART协议处理模块
//! 
//! 负责UART串口通信的管理，支持多种波特率和数据格式。

use heapless::{String, Vec, FnvIndexMap, Deque};
use embedded_hal::serial::{Read, Write};
use embedded_hal::blocking::delay::DelayMs;
use embassy_time::{Duration, Timer, Instant};
use nb;

use super::{
    ProtocolError, ProtocolConfig, ProtocolPacket, ProtocolStats, ProtocolManager, ProtocolDevice,
    DataDirection, TransferMode,
};

/// UART错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum UARTError {
    /// 传输错误
    TransferError,
    /// 帧错误
    FrameError,
    /// 奇偶校验错误
    ParityError,
    /// 溢出错误
    OverrunError,
    /// 噪声错误
    NoiseError,
    /// 超时
    Timeout,
    /// 缓冲区满
    BufferFull,
    /// 配置错误
    ConfigError(&'static str),
    /// 设备忙碌
    DeviceBusy,
    /// 无效参数
    InvalidParameter,
    /// 协议错误
    ProtocolError,
}

/// UART配置
#[derive(Debug, Clone)]
pub struct UARTConfig {
    /// 波特率
    pub baud_rate: u32,
    /// 数据位数
    pub data_bits: u8,
    /// 停止位数
    pub stop_bits: StopBits,
    /// 奇偶校验
    pub parity: Parity,
    /// 流控制
    pub flow_control: FlowControl,
    /// TX引脚
    pub tx_pin: u8,
    /// RX引脚
    pub rx_pin: u8,
    /// 接收缓冲区大小
    pub rx_buffer_size: usize,
    /// 发送缓冲区大小
    pub tx_buffer_size: usize,
    /// 超时时间（毫秒）
    pub timeout_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 启用回显
    pub echo_enabled: bool,
}

/// 停止位
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StopBits {
    /// 1个停止位
    One,
    /// 1.5个停止位
    OnePointFive,
    /// 2个停止位
    Two,
}

/// 奇偶校验
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Parity {
    /// 无校验
    None,
    /// 奇校验
    Odd,
    /// 偶校验
    Even,
    /// 标记校验
    Mark,
    /// 空格校验
    Space,
}

/// 流控制
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlowControl {
    /// 无流控制
    None,
    /// 软件流控制（XON/XOFF）
    Software,
    /// 硬件流控制（RTS/CTS）
    Hardware,
}

impl Default for UARTConfig {
    fn default() -> Self {
        Self {
            baud_rate: 115200,
            data_bits: 8,
            stop_bits: StopBits::One,
            parity: Parity::None,
            flow_control: FlowControl::None,
            tx_pin: 17,
            rx_pin: 16,
            rx_buffer_size: 256,
            tx_buffer_size: 256,
            timeout_ms: 1000,
            retry_count: 3,
            echo_enabled: false,
        }
    }
}

/// UART设备信息
#[derive(Debug, Clone)]
pub struct UARTDeviceInfo {
    /// 设备ID
    pub device_id: u8,
    /// 设备名称
    pub name: String<32>,
    /// 设备类型
    pub device_type: String<16>,
    /// 波特率
    pub baud_rate: u32,
    /// 是否在线
    pub online: bool,
    /// 最后通信时间
    pub last_communication: u64,
    /// 通信统计
    pub stats: UARTDeviceStats,
}

/// UART设备统计
#[derive(Debug, Clone, Default)]
pub struct UARTDeviceStats {
    /// 发送字节数
    pub bytes_sent: u64,
    /// 接收字节数
    pub bytes_received: u64,
    /// 发送消息数
    pub messages_sent: u32,
    /// 接收消息数
    pub messages_received: u32,
    /// 帧错误数
    pub frame_errors: u32,
    /// 奇偶校验错误数
    pub parity_errors: u32,
    /// 溢出错误数
    pub overrun_errors: u32,
    /// 超时次数
    pub timeouts: u32,
    /// 平均传输时间（微秒）
    pub avg_transfer_time_us: u32,
}

/// UART消息
#[derive(Debug, Clone)]
pub struct UARTMessage {
    /// 消息ID
    pub id: u16,
    /// 设备ID
    pub device_id: u8,
    /// 消息类型
    pub message_type: MessageType,
    /// 数据负载
    pub payload: Vec<u8, 256>,
    /// 时间戳
    pub timestamp: u64,
    /// 校验和
    pub checksum: Option<u16>,
}

/// 消息类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MessageType {
    /// 数据消息
    Data,
    /// 命令消息
    Command,
    /// 响应消息
    Response,
    /// 状态消息
    Status,
    /// 错误消息
    Error,
    /// 心跳消息
    Heartbeat,
}

impl Default for UARTMessage {
    fn default() -> Self {
        Self {
            id: 0,
            device_id: 0,
            message_type: MessageType::Data,
            payload: Vec::new(),
            timestamp: 0,
            checksum: None,
        }
    }
}

impl UARTMessage {
    /// 创建新消息
    pub fn new(device_id: u8, message_type: MessageType, payload: &[u8]) -> Result<Self, UARTError> {
        let mut message = Self::default();
        message.device_id = device_id;
        message.message_type = message_type;
        message.payload.extend_from_slice(payload)
            .map_err(|_| UARTError::BufferFull)?;
        message.timestamp = Self::get_current_timestamp();
        message.id = Self::generate_message_id();
        Ok(message)
    }

    /// 序列化消息
    pub fn serialize(&self) -> Result<Vec<u8, 512>, UARTError> {
        let mut buffer = Vec::new();
        
        // 消息头
        buffer.push(0xAA).map_err(|_| UARTError::BufferFull)?; // 起始标志
        buffer.push(self.device_id).map_err(|_| UARTError::BufferFull)?;
        buffer.push(self.message_type as u8).map_err(|_| UARTError::BufferFull)?;
        buffer.push(self.payload.len() as u8).map_err(|_| UARTError::BufferFull)?;
        
        // 消息ID
        buffer.push((self.id >> 8) as u8).map_err(|_| UARTError::BufferFull)?;
        buffer.push((self.id & 0xFF) as u8).map_err(|_| UARTError::BufferFull)?;
        
        // 数据负载
        buffer.extend_from_slice(&self.payload).map_err(|_| UARTError::BufferFull)?;
        
        // 校验和
        let checksum = self.calculate_checksum();
        buffer.push((checksum >> 8) as u8).map_err(|_| UARTError::BufferFull)?;
        buffer.push((checksum & 0xFF) as u8).map_err(|_| UARTError::BufferFull)?;
        
        // 结束标志
        buffer.push(0x55).map_err(|_| UARTError::BufferFull)?;
        
        Ok(buffer)
    }

    /// 反序列化消息
    pub fn deserialize(data: &[u8]) -> Result<Self, UARTError> {
        if data.len() < 9 {
            return Err(UARTError::ProtocolError);
        }

        // 检查起始和结束标志
        if data[0] != 0xAA || data[data.len() - 1] != 0x55 {
            return Err(UARTError::ProtocolError);
        }

        let device_id = data[1];
        let message_type = match data[2] {
            0 => MessageType::Data,
            1 => MessageType::Command,
            2 => MessageType::Response,
            3 => MessageType::Status,
            4 => MessageType::Error,
            5 => MessageType::Heartbeat,
            _ => return Err(UARTError::ProtocolError),
        };
        
        let payload_len = data[3] as usize;
        if data.len() != 9 + payload_len {
            return Err(UARTError::ProtocolError);
        }

        let id = ((data[4] as u16) << 8) | (data[5] as u16);
        
        let mut payload = Vec::new();
        payload.extend_from_slice(&data[6..6 + payload_len])
            .map_err(|_| UARTError::BufferFull)?;

        let received_checksum = ((data[6 + payload_len] as u16) << 8) | (data[7 + payload_len] as u16);
        
        let mut message = Self {
            id,
            device_id,
            message_type,
            payload,
            timestamp: Self::get_current_timestamp(),
            checksum: Some(received_checksum),
        };

        // 验证校验和
        if message.calculate_checksum() != received_checksum {
            return Err(UARTError::ProtocolError);
        }

        Ok(message)
    }

    /// 计算校验和
    pub fn calculate_checksum(&self) -> u16 {
        let mut checksum = 0u16;
        checksum = checksum.wrapping_add(self.device_id as u16);
        checksum = checksum.wrapping_add(self.message_type as u16);
        checksum = checksum.wrapping_add(self.payload.len() as u16);
        checksum = checksum.wrapping_add(self.id);
        
        for &byte in &self.payload {
            checksum = checksum.wrapping_add(byte as u16);
        }
        
        checksum
    }

    /// 获取当前时间戳
    fn get_current_timestamp() -> u64 {
        // 这里应该使用实际的时间获取函数
        1000000 // 模拟时间戳
    }

    /// 生成消息ID
    fn generate_message_id() -> u16 {
        // 简单的递增ID生成器
        static mut COUNTER: u16 = 0;
        unsafe {
            COUNTER = COUNTER.wrapping_add(1);
            COUNTER
        }
    }
}

/// UART管理器
pub struct UARTManager<UART, D> 
where
    UART: Read<u8> + Write<u8>,
    D: DelayMs<u32>,
{
    /// UART外设
    uart: UART,
    /// 延时提供者
    delay: D,
    /// 配置
    config: UARTConfig,
    /// 协议配置
    protocol_config: ProtocolConfig,
    /// 统计信息
    stats: ProtocolStats,
    /// 设备列表
    devices: FnvIndexMap<u8, UARTDeviceInfo, 16>,
    /// 接收缓冲区
    rx_buffer: Deque<u8, 512>,
    /// 发送缓冲区
    tx_buffer: Deque<u8, 512>,
    /// 消息队列
    message_queue: Deque<UARTMessage, 32>,
    /// 最后操作时间
    last_operation_time: Instant,
}

impl<UART, D> UARTManager<UART, D>
where
    UART: Read<u8> + Write<u8>,
    D: DelayMs<u32>,
{
    /// 创建新的UART管理器
    pub fn new(uart: UART, delay: D, config: UARTConfig) -> Self {
        let protocol_config = ProtocolConfig {
            protocol_type: super::ProtocolType::UART,
            timeout_ms: config.timeout_ms,
            retry_count: config.retry_count,
            ..Default::default()
        };

        Self {
            uart,
            delay,
            config,
            protocol_config,
            stats: ProtocolStats::default(),
            devices: FnvIndexMap::new(),
            rx_buffer: Deque::new(),
            tx_buffer: Deque::new(),
            message_queue: Deque::new(),
            last_operation_time: Instant::now(),
        }
    }

    /// 注册UART设备
    pub fn register_device(&mut self, device_info: UARTDeviceInfo) -> Result<(), UARTError> {
        self.devices.insert(device_info.device_id, device_info)
            .map_err(|_| UARTError::ConfigError("Too many devices"))?;
        Ok(())
    }

    /// 发送数据
    pub fn send_data(&mut self, device_id: u8, data: &[u8]) -> Result<(), UARTError> {
        let start_time = Instant::now();
        
        for attempt in 0..=self.config.retry_count {
            let mut success = true;
            
            for &byte in data {
                match self.uart.write(byte) {
                    Ok(()) => {},
                    Err(nb::Error::WouldBlock) => {
                        // 非阻塞模式，稍后重试
                        self.delay.delay_ms(1);
                        match self.uart.write(byte) {
                            Ok(()) => {},
                            Err(_) => {
                                success = false;
                                break;
                            }
                        }
                    },
                    Err(_) => {
                        success = false;
                        break;
                    }
                }
            }
            
            if success {
                let operation_time = start_time.elapsed().as_micros() as u32;
                self.update_stats(device_id, operation_time, true, data.len(), true);
                return Ok(());
            } else if attempt < self.config.retry_count {
                self.delay.delay_ms(10);
                self.stats.retries += 1;
            }
        }
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(device_id, operation_time, false, data.len(), true);
        
        Err(UARTError::TransferError)
    }

    /// 接收数据
    pub fn receive_data(&mut self, max_length: usize) -> Result<Vec<u8, 256>, UARTError> {
        let mut data = Vec::new();
        let start_time = Instant::now();
        
        while data.len() < max_length {
            match self.uart.read() {
                Ok(byte) => {
                    if data.push(byte).is_err() {
                        break;
                    }
                },
                Err(nb::Error::WouldBlock) => {
                    // 没有更多数据
                    break;
                },
                Err(_) => {
                    return Err(UARTError::TransferError);
                }
            }
            
            // 检查超时
            if start_time.elapsed().as_millis() > self.config.timeout_ms as u64 {
                if data.is_empty() {
                    return Err(UARTError::Timeout);
                } else {
                    break;
                }
            }
        }
        
        if !data.is_empty() {
            let operation_time = start_time.elapsed().as_micros() as u32;
            self.update_stats(0, operation_time, true, data.len(), false);
        }
        
        Ok(data)
    }

    /// 发送消息
    pub fn send_message(&mut self, message: &UARTMessage) -> Result<(), UARTError> {
        let serialized = message.serialize()?;
        self.send_data(message.device_id, &serialized)
    }

    /// 接收消息
    pub fn receive_message(&mut self) -> Result<Option<UARTMessage>, UARTError> {
        // 尝试从缓冲区读取数据
        let mut buffer = [0u8; 64];
        let mut bytes_read = 0;
        
        while bytes_read < buffer.len() {
            match self.uart.read() {
                Ok(byte) => {
                    buffer[bytes_read] = byte;
                    bytes_read += 1;
                    
                    // 检查是否收到完整消息
                    if bytes_read >= 9 && buffer[0] == 0xAA {
                        let payload_len = buffer[3] as usize;
                        let expected_len = 9 + payload_len;
                        
                        if bytes_read >= expected_len && buffer[expected_len - 1] == 0x55 {
                            // 收到完整消息
                            let message = UARTMessage::deserialize(&buffer[..expected_len])?;
                            return Ok(Some(message));
                        }
                    }
                },
                Err(nb::Error::WouldBlock) => {
                    // 没有更多数据
                    break;
                },
                Err(_) => {
                    return Err(UARTError::TransferError);
                }
            }
        }
        
        Ok(None)
    }

    /// 发送字符串
    pub fn send_string(&mut self, device_id: u8, text: &str) -> Result<(), UARTError> {
        self.send_data(device_id, text.as_bytes())
    }

    /// 接收字符串
    pub fn receive_string(&mut self, max_length: usize) -> Result<String<256>, UARTError> {
        let data = self.receive_data(max_length)?;
        
        // 转换为字符串，忽略无效的UTF-8字符
        let mut string = String::new();
        for &byte in &data {
            if byte.is_ascii() && byte >= 32 && byte <= 126 {
                if string.push(byte as char).is_err() {
                    break;
                }
            }
        }
        
        Ok(string)
    }

    /// 发送命令
    pub fn send_command(&mut self, device_id: u8, command: &str) -> Result<(), UARTError> {
        let mut cmd_data = Vec::new();
        cmd_data.extend_from_slice(command.as_bytes())
            .map_err(|_| UARTError::BufferFull)?;
        
        // 添加换行符
        cmd_data.push(b'\r').map_err(|_| UARTError::BufferFull)?;
        cmd_data.push(b'\n').map_err(|_| UARTError::BufferFull)?;
        
        self.send_data(device_id, &cmd_data)
    }

    /// 等待响应
    pub fn wait_for_response(&mut self, timeout_ms: u32) -> Result<String<256>, UARTError> {
        let start_time = Instant::now();
        let mut response = String::new();
        
        while start_time.elapsed().as_millis() < timeout_ms as u64 {
            match self.uart.read() {
                Ok(byte) => {
                    if byte == b'\n' || byte == b'\r' {
                        if !response.is_empty() {
                            return Ok(response);
                        }
                    } else if byte.is_ascii_graphic() || byte == b' ' {
                        if response.push(byte as char).is_err() {
                            return Ok(response);
                        }
                    }
                },
                Err(nb::Error::WouldBlock) => {
                    self.delay.delay_ms(1);
                },
                Err(_) => {
                    return Err(UARTError::TransferError);
                }
            }
        }
        
        if response.is_empty() {
            Err(UARTError::Timeout)
        } else {
            Ok(response)
        }
    }

    /// 清空接收缓冲区
    pub fn flush_rx_buffer(&mut self) {
        while let Ok(_) = self.uart.read() {
            // 读取并丢弃所有数据
        }
    }

    /// 检查设备是否在线
    pub fn ping_device(&mut self, device_id: u8) -> Result<bool, UARTError> {
        let ping_message = UARTMessage::new(device_id, MessageType::Heartbeat, b"PING")?;
        
        self.send_message(&ping_message)?;
        
        // 等待响应
        let start_time = Instant::now();
        while start_time.elapsed().as_millis() < 1000 {
            if let Ok(Some(response)) = self.receive_message() {
                if response.device_id == device_id && response.message_type == MessageType::Heartbeat {
                    return Ok(true);
                }
            }
            self.delay.delay_ms(10);
        }
        
        Ok(false)
    }

    /// 更新统计信息
    fn update_stats(&mut self, device_id: u8, operation_time: u32, success: bool, bytes: usize, is_send: bool) {
        // 更新全局统计
        if success {
            if is_send {
                self.stats.packets_sent += 1;
            } else {
                self.stats.packets_received += 1;
            }
            self.stats.total_bytes_transferred += bytes as u64;
        } else {
            if is_send {
                self.stats.send_failures += 1;
            } else {
                self.stats.receive_failures += 1;
            }
        }

        self.update_avg_transfer_time(operation_time);

        // 更新设备统计
        if let Some(device_info) = self.devices.get_mut(&device_id) {
            if is_send {
                device_info.stats.bytes_sent += bytes as u64;
                device_info.stats.messages_sent += 1;
            } else {
                device_info.stats.bytes_received += bytes as u64;
                device_info.stats.messages_received += 1;
            }

            // 更新设备平均传输时间
            let total_msgs = device_info.stats.messages_sent + device_info.stats.messages_received;
            if total_msgs == 1 {
                device_info.stats.avg_transfer_time_us = operation_time;
            } else {
                let alpha = 0.1;
                let new_avg = (1.0 - alpha) * device_info.stats.avg_transfer_time_us as f32 + 
                             alpha * operation_time as f32;
                device_info.stats.avg_transfer_time_us = new_avg as u32;
            }

            device_info.last_communication = self.get_current_timestamp();
        }

        self.last_operation_time = Instant::now();
    }

    /// 更新平均传输时间
    fn update_avg_transfer_time(&mut self, operation_time: u32) {
        let total_ops = self.stats.packets_sent + self.stats.packets_received;
        if total_ops == 1 {
            self.stats.avg_transfer_time_us = operation_time;
        } else {
            let alpha = 0.1;
            let new_avg = (1.0 - alpha) * self.stats.avg_transfer_time_us as f32 + 
                         alpha * operation_time as f32;
            self.stats.avg_transfer_time_us = new_avg as u32;
        }
    }

    /// 获取设备信息
    pub fn get_device_info(&self, device_id: u8) -> Option<&UARTDeviceInfo> {
        self.devices.get(&device_id)
    }

    /// 获取所有设备
    pub fn get_all_devices(&self) -> Vec<&UARTDeviceInfo, 16> {
        let mut devices = Vec::new();
        for device_info in self.devices.values() {
            if devices.push(device_info).is_err() {
                break;
            }
        }
        devices
    }

    /// 获取当前时间戳
    fn get_current_timestamp(&self) -> u64 {
        self.last_operation_time.elapsed().as_millis() as u64
    }

    /// 设置设备名称
    pub fn set_device_name(&mut self, device_id: u8, name: &str) -> Result<(), UARTError> {
        if let Some(device_info) = self.devices.get_mut(&device_id) {
            device_info.name = String::from_str(name)
                .map_err(|_| UARTError::ConfigError("Invalid device name"))?;
            Ok(())
        } else {
            Err(UARTError::InvalidParameter)
        }
    }
}

impl<UART, D> ProtocolManager for UARTManager<UART, D>
where
    UART: Read<u8> + Write<u8>,
    D: DelayMs<u32>,
{
    fn init(&mut self) -> Result<(), ProtocolError> {
        self.flush_rx_buffer();
        Ok(())
    }

    fn send_packet(&mut self, packet: &ProtocolPacket) -> Result<(), ProtocolError> {
        let device_id = packet.address as u8;
        
        match packet.direction {
            DataDirection::Write => {
                self.send_data(device_id, &packet.data)
                    .map_err(|e| ProtocolError::UART(e))
            },
            _ => Err(ProtocolError::UnsupportedProtocol),
        }
    }

    fn receive_packet(&mut self) -> Result<ProtocolPacket, ProtocolError> {
        if let Some(message) = self.receive_message().map_err(|e| ProtocolError::UART(e))? {
            let mut packet = ProtocolPacket::default();
            packet.protocol = super::ProtocolType::UART;
            packet.address = message.device_id as u32;
            packet.data.extend_from_slice(&message.payload)
                .map_err(|_| ProtocolError::BufferError)?;
            packet.direction = DataDirection::Read;
            packet.timestamp = message.timestamp;
            
            Ok(packet)
        } else {
            Err(ProtocolError::Timeout)
        }
    }

    fn read_data(&mut self, address: u32, _register: Option<u16>, length: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        self.receive_data(length)
            .map_err(|e| ProtocolError::UART(e))
    }

    fn write_data(&mut self, address: u32, _register: Option<u16>, data: &[u8]) -> Result<(), ProtocolError> {
        self.send_data(address as u8, data)
            .map_err(|e| ProtocolError::UART(e))
    }

    fn device_exists(&mut self, address: u32) -> Result<bool, ProtocolError> {
        self.ping_device(address as u8)
            .map_err(|e| ProtocolError::UART(e))
    }

    fn scan_devices(&mut self) -> Result<Vec<u32, 128>, ProtocolError> {
        let mut found_devices = Vec::new();
        
        for device_id in self.devices.keys() {
            found_devices.push(*device_id as u32)
                .map_err(|_| ProtocolError::BufferError)?;
        }
        
        Ok(found_devices)
    }

    fn get_stats(&self) -> &ProtocolStats {
        &self.stats
    }

    fn reset_stats(&mut self) {
        self.stats = ProtocolStats::default();
        for device_info in self.devices.values_mut() {
            device_info.stats = UARTDeviceStats::default();
        }
    }

    fn set_config(&mut self, config: ProtocolConfig) -> Result<(), ProtocolError> {
        self.protocol_config = config;
        Ok(())
    }

    fn get_config(&self) -> &ProtocolConfig {
        &self.protocol_config
    }
}

/// UART设备包装器
pub struct UARTDevice<UART, D>
where
    UART: Read<u8> + Write<u8>,
    D: DelayMs<u32>,
{
    /// UART管理器引用
    manager: *mut UARTManager<UART, D>,
    /// 设备ID
    device_id: u8,
}

impl<UART, D> UARTDevice<UART, D>
where
    UART: Read<u8> + Write<u8>,
    D: DelayMs<u32>,
{
    /// 创建新的UART设备
    pub fn new(manager: &mut UARTManager<UART, D>, device_id: u8) -> Self {
        Self {
            manager: manager as *mut _,
            device_id,
        }
    }

    /// 获取管理器引用
    fn get_manager(&mut self) -> &mut UARTManager<UART, D> {
        unsafe { &mut *self.manager }
    }
}

impl<UART, D> ProtocolDevice for UARTDevice<UART, D>
where
    UART: Read<u8> + Write<u8>,
    D: DelayMs<u32>,
{
    fn read_register(&mut self, _register: u16) -> Result<u8, ProtocolError> {
        // UART通常不使用寄存器概念
        Err(ProtocolError::UnsupportedProtocol)
    }

    fn write_register(&mut self, _register: u16, _value: u8) -> Result<(), ProtocolError> {
        // UART通常不使用寄存器概念
        Err(ProtocolError::UnsupportedProtocol)
    }

    fn read_registers(&mut self, _start_register: u16, count: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        self.get_manager()
            .receive_data(count)
            .map_err(|e| ProtocolError::UART(e))
    }

    fn write_registers(&mut self, _start_register: u16, data: &[u8]) -> Result<(), ProtocolError> {
        self.get_manager()
            .send_data(self.device_id, data)
            .map_err(|e| ProtocolError::UART(e))
    }

    fn read_block(&mut self, length: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        self.get_manager()
            .receive_data(length)
            .map_err(|e| ProtocolError::UART(e))
    }

    fn write_block(&mut self, data: &[u8]) -> Result<(), ProtocolError> {
        self.get_manager()
            .send_data(self.device_id, data)
            .map_err(|e| ProtocolError::UART(e))
    }

    fn get_address(&self) -> u32 {
        self.device_id as u32
    }

    fn set_address(&mut self, address: u32) {
        self.device_id = address as u8;
    }

    fn is_connected(&mut self) -> Result<bool, ProtocolError> {
        self.get_manager()
            .ping_device(self.device_id)
            .map_err(|e| ProtocolError::UART(e))
    }

    fn reset(&mut self) -> Result<(), ProtocolError> {
        // 发送复位命令
        self.get_manager()
            .send_command(self.device_id, "RESET")
            .map_err(|e| ProtocolError::UART(e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 模拟UART实现用于测试
    struct MockUART {
        tx_data: Vec<u8, 256>,
        rx_data: Vec<u8, 256>,
        rx_index: usize,
    }

    impl MockUART {
        fn new() -> Self {
            Self {
                tx_data: Vec::new(),
                rx_data: Vec::new(),
                rx_index: 0,
            }
        }

        fn add_rx_data(&mut self, data: &[u8]) {
            self.rx_data.extend_from_slice(data).ok();
        }
    }

    impl Read<u8> for MockUART {
        type Error = ();

        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            if self.rx_index < self.rx_data.len() {
                let byte = self.rx_data[self.rx_index];
                self.rx_index += 1;
                Ok(byte)
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    impl Write<u8> for MockUART {
        type Error = ();

        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.tx_data.push(word).map_err(|_| nb::Error::Other(()))?;
            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            Ok(())
        }
    }

    struct MockDelay;

    impl DelayMs<u32> for MockDelay {
        fn delay_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_uart_manager_creation() {
        let uart = MockUART::new();
        let delay = MockDelay;
        let config = UARTConfig::default();
        
        let manager = UARTManager::new(uart, delay, config);
        assert_eq!(manager.devices.len(), 0);
    }

    #[test]
    fn test_uart_message_serialization() {
        let message = UARTMessage::new(1, MessageType::Data, b"Hello").unwrap();
        let serialized = message.serialize().unwrap();
        
        assert_eq!(serialized[0], 0xAA); // 起始标志
        assert_eq!(serialized[1], 1); // 设备ID
        assert_eq!(serialized[2], MessageType::Data as u8); // 消息类型
        assert_eq!(serialized[3], 5); // 数据长度
        assert_eq!(serialized[serialized.len() - 1], 0x55); // 结束标志
    }

    #[test]
    fn test_uart_message_deserialization() {
        let original = UARTMessage::new(1, MessageType::Command, b"TEST").unwrap();
        let serialized = original.serialize().unwrap();
        let deserialized = UARTMessage::deserialize(&serialized).unwrap();
        
        assert_eq!(deserialized.device_id, original.device_id);
        assert_eq!(deserialized.message_type, original.message_type);
        assert_eq!(deserialized.payload, original.payload);
    }
}