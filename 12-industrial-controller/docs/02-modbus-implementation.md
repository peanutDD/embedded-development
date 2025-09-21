# Modbus协议实现

本文档详细介绍在STM32F4平台上实现Modbus RTU和Modbus TCP协议的方法，包括协议解析、数据处理、错误处理和性能优化等方面。

## Modbus协议概述

### 协议特点
- **主从架构**: 一个主站，多个从站
- **请求-响应模式**: 主站发送请求，从站响应
- **标准化**: 工业自动化领域广泛使用
- **简单可靠**: 协议简单，易于实现和调试

### 数据模型
```
┌─────────────────┬─────────────────┬─────────────────┐
│   数据类型      │   地址范围      │   访问权限      │
├─────────────────┼─────────────────┼─────────────────┤
│ 线圈(Coils)     │ 00001-09999     │ 读写            │
│ 离散输入        │ 10001-19999     │ 只读            │
│ 输入寄存器      │ 30001-39999     │ 只读            │
│ 保持寄存器      │ 40001-49999     │ 读写            │
└─────────────────┴─────────────────┴─────────────────┘
```

## Modbus RTU实现

### 完整的RTU协议栈
```rust
use stm32f4xx_hal::{
    prelude::*,
    serial::{Serial, Event},
    timer::{Timer, Event as TimerEvent},
    stm32,
};
use heapless::{Vec, pool::{Pool, Node}};
use nb;

/// Modbus RTU帧结构
#[derive(Clone)]
pub struct ModbusRtuFrame {
    pub slave_id: u8,
    pub function_code: u8,
    pub data: Vec<u8, 252>,
    pub crc: u16,
}

/// Modbus RTU状态机
#[derive(Clone, Copy, PartialEq)]
pub enum ModbusRtuState {
    Idle,
    Receiving,
    Processing,
    Transmitting,
    Error,
}

/// Modbus RTU主站
pub struct ModbusRtuMaster {
    serial: Serial<stm32::USART1, (stm32f4xx_hal::gpio::gpioa::PA9<stm32f4xx_hal::gpio::Alternate<7>>, 
                                   stm32f4xx_hal::gpio::gpioa::PA10<stm32f4xx_hal::gpio::Alternate<7>>)>,
    timer: Timer<stm32::TIM2>,
    state: ModbusRtuState,
    rx_buffer: Vec<u8, 256>,
    tx_buffer: Vec<u8, 256>,
    response_timeout: u32,
    frame_timeout: u32,
    current_request: Option<ModbusRtuFrame>,
    transaction_id: u16,
    statistics: ModbusStatistics,
}

/// Modbus统计信息
#[derive(Clone, Copy)]
pub struct ModbusStatistics {
    pub requests_sent: u32,
    pub responses_received: u32,
    pub timeouts: u32,
    pub crc_errors: u32,
    pub frame_errors: u32,
    pub exception_responses: u32,
}

impl ModbusRtuMaster {
    pub fn new(
        serial: Serial<stm32::USART1, (stm32f4xx_hal::gpio::gpioa::PA9<stm32f4xx_hal::gpio::Alternate<7>>, 
                                       stm32f4xx_hal::gpio::gpioa::PA10<stm32f4xx_hal::gpio::Alternate<7>>)>,
        timer: Timer<stm32::TIM2>,
    ) -> Self {
        Self {
            serial,
            timer,
            state: ModbusRtuState::Idle,
            rx_buffer: Vec::new(),
            tx_buffer: Vec::new(),
            response_timeout: 1000,  // 1秒
            frame_timeout: 35,       // 3.5个字符时间
            current_request: None,
            transaction_id: 0,
            statistics: ModbusStatistics {
                requests_sent: 0,
                responses_received: 0,
                timeouts: 0,
                crc_errors: 0,
                frame_errors: 0,
                exception_responses: 0,
            },
        }
    }
    
    /// 读取线圈
    pub fn read_coils(&mut self, slave_id: u8, start_addr: u16, quantity: u16) -> Result<(), ModbusError> {
        if self.state != ModbusRtuState::Idle {
            return Err(ModbusError::Busy);
        }
        
        if quantity == 0 || quantity > 2000 {
            return Err(ModbusError::InvalidParameter);
        }
        
        let mut frame = ModbusRtuFrame {
            slave_id,
            function_code: 0x01,
            data: Vec::new(),
            crc: 0,
        };
        
        // 构建请求数据
        frame.data.push((start_addr >> 8) as u8).ok();
        frame.data.push((start_addr & 0xFF) as u8).ok();
        frame.data.push((quantity >> 8) as u8).ok();
        frame.data.push((quantity & 0xFF) as u8).ok();
        
        self.send_request(frame)
    }
    
    /// 读取保持寄存器
    pub fn read_holding_registers(&mut self, slave_id: u8, start_addr: u16, quantity: u16) -> Result<(), ModbusError> {
        if self.state != ModbusRtuState::Idle {
            return Err(ModbusError::Busy);
        }
        
        if quantity == 0 || quantity > 125 {
            return Err(ModbusError::InvalidParameter);
        }
        
        let mut frame = ModbusRtuFrame {
            slave_id,
            function_code: 0x03,
            data: Vec::new(),
            crc: 0,
        };
        
        frame.data.push((start_addr >> 8) as u8).ok();
        frame.data.push((start_addr & 0xFF) as u8).ok();
        frame.data.push((quantity >> 8) as u8).ok();
        frame.data.push((quantity & 0xFF) as u8).ok();
        
        self.send_request(frame)
    }
    
    /// 写单个线圈
    pub fn write_single_coil(&mut self, slave_id: u8, addr: u16, value: bool) -> Result<(), ModbusError> {
        if self.state != ModbusRtuState::Idle {
            return Err(ModbusError::Busy);
        }
        
        let mut frame = ModbusRtuFrame {
            slave_id,
            function_code: 0x05,
            data: Vec::new(),
            crc: 0,
        };
        
        frame.data.push((addr >> 8) as u8).ok();
        frame.data.push((addr & 0xFF) as u8).ok();
        frame.data.push(if value { 0xFF } else { 0x00 }).ok();
        frame.data.push(0x00).ok();
        
        self.send_request(frame)
    }
    
    /// 写单个寄存器
    pub fn write_single_register(&mut self, slave_id: u8, addr: u16, value: u16) -> Result<(), ModbusError> {
        if self.state != ModbusRtuState::Idle {
            return Err(ModbusError::Busy);
        }
        
        let mut frame = ModbusRtuFrame {
            slave_id,
            function_code: 0x06,
            data: Vec::new(),
            crc: 0,
        };
        
        frame.data.push((addr >> 8) as u8).ok();
        frame.data.push((addr & 0xFF) as u8).ok();
        frame.data.push((value >> 8) as u8).ok();
        frame.data.push((value & 0xFF) as u8).ok();
        
        self.send_request(frame)
    }
    
    /// 写多个寄存器
    pub fn write_multiple_registers(&mut self, slave_id: u8, start_addr: u16, values: &[u16]) -> Result<(), ModbusError> {
        if self.state != ModbusRtuState::Idle {
            return Err(ModbusError::Busy);
        }
        
        if values.is_empty() || values.len() > 123 {
            return Err(ModbusError::InvalidParameter);
        }
        
        let mut frame = ModbusRtuFrame {
            slave_id,
            function_code: 0x10,
            data: Vec::new(),
            crc: 0,
        };
        
        let quantity = values.len() as u16;
        let byte_count = (quantity * 2) as u8;
        
        frame.data.push((start_addr >> 8) as u8).ok();
        frame.data.push((start_addr & 0xFF) as u8).ok();
        frame.data.push((quantity >> 8) as u8).ok();
        frame.data.push((quantity & 0xFF) as u8).ok();
        frame.data.push(byte_count).ok();
        
        for &value in values {
            frame.data.push((value >> 8) as u8).ok();
            frame.data.push((value & 0xFF) as u8).ok();
        }
        
        self.send_request(frame)
    }
    
    /// 发送请求
    fn send_request(&mut self, mut frame: ModbusRtuFrame) -> Result<(), ModbusError> {
        // 计算CRC
        let mut crc_data = Vec::<u8, 256>::new();
        crc_data.push(frame.slave_id).ok();
        crc_data.push(frame.function_code).ok();
        for &byte in &frame.data {
            crc_data.push(byte).ok();
        }
        
        frame.crc = calculate_crc16(&crc_data);
        
        // 构建发送缓冲区
        self.tx_buffer.clear();
        self.tx_buffer.push(frame.slave_id).ok();
        self.tx_buffer.push(frame.function_code).ok();
        for &byte in &frame.data {
            self.tx_buffer.push(byte).ok();
        }
        self.tx_buffer.push((frame.crc & 0xFF) as u8).ok();
        self.tx_buffer.push((frame.crc >> 8) as u8).ok();
        
        // 发送数据
        for &byte in &self.tx_buffer {
            nb::block!(self.serial.write(byte)).map_err(|_| ModbusError::TransmissionError)?;
        }
        
        // 设置状态和超时
        self.current_request = Some(frame);
        self.state = ModbusRtuState::Receiving;
        self.rx_buffer.clear();
        self.timer.start(self.response_timeout.millis());
        self.statistics.requests_sent += 1;
        self.transaction_id += 1;
        
        Ok(())
    }
    
    /// 处理接收数据
    pub fn handle_receive(&mut self) -> Result<Option<ModbusResponse>, ModbusError> {
        if self.state != ModbusRtuState::Receiving {
            return Ok(None);
        }
        
        // 读取串口数据
        while let Ok(byte) = self.serial.read() {
            if self.rx_buffer.push(byte).is_err() {
                self.state = ModbusRtuState::Error;
                self.statistics.frame_errors += 1;
                return Err(ModbusError::BufferOverflow);
            }
            
            // 重置帧间隔定时器
            self.timer.start(self.frame_timeout.millis());
        }
        
        // 检查帧间隔超时
        if self.timer.wait().is_ok() {
            return self.process_response();
        }
        
        // 检查响应超时
        if self.timer.wait().is_ok() {
            self.state = ModbusRtuState::Idle;
            self.statistics.timeouts += 1;
            return Err(ModbusError::Timeout);
        }
        
        Ok(None)
    }
    
    /// 处理响应
    fn process_response(&mut self) -> Result<Option<ModbusResponse>, ModbusError> {
        if self.rx_buffer.len() < 4 {
            self.state = ModbusRtuState::Idle;
            self.statistics.frame_errors += 1;
            return Err(ModbusError::InvalidFrame);
        }
        
        // 验证CRC
        let data_len = self.rx_buffer.len() - 2;
        let received_crc = (self.rx_buffer[data_len + 1] as u16) << 8 | (self.rx_buffer[data_len] as u16);
        let calculated_crc = calculate_crc16(&self.rx_buffer[..data_len]);
        
        if received_crc != calculated_crc {
            self.state = ModbusRtuState::Idle;
            self.statistics.crc_errors += 1;
            return Err(ModbusError::CrcError);
        }
        
        let slave_id = self.rx_buffer[0];
        let function_code = self.rx_buffer[1];
        
        // 检查从站ID
        if let Some(ref request) = self.current_request {
            if slave_id != request.slave_id {
                self.state = ModbusRtuState::Idle;
                self.statistics.frame_errors += 1;
                return Err(ModbusError::InvalidSlaveId);
            }
        }
        
        // 检查异常响应
        if function_code & 0x80 != 0 {
            let exception_code = self.rx_buffer[2];
            self.state = ModbusRtuState::Idle;
            self.statistics.exception_responses += 1;
            return Err(ModbusError::Exception(exception_code));
        }
        
        // 解析响应数据
        let response = self.parse_response(function_code, &self.rx_buffer[2..data_len])?;
        
        self.state = ModbusRtuState::Idle;
        self.current_request = None;
        self.statistics.responses_received += 1;
        
        Ok(Some(response))
    }
    
    /// 解析响应数据
    fn parse_response(&self, function_code: u8, data: &[u8]) -> Result<ModbusResponse, ModbusError> {
        match function_code {
            0x01 | 0x02 => {
                // 读取线圈/离散输入响应
                if data.is_empty() {
                    return Err(ModbusError::InvalidFrame);
                }
                
                let byte_count = data[0] as usize;
                if data.len() < 1 + byte_count {
                    return Err(ModbusError::InvalidFrame);
                }
                
                let mut coils = Vec::<bool, 2000>::new();
                for byte_idx in 0..byte_count {
                    let byte_value = data[1 + byte_idx];
                    for bit_idx in 0..8 {
                        let coil_state = (byte_value & (1 << bit_idx)) != 0;
                        coils.push(coil_state).ok();
                    }
                }
                
                Ok(ModbusResponse::ReadCoils(coils))
            }
            
            0x03 | 0x04 => {
                // 读取寄存器响应
                if data.is_empty() {
                    return Err(ModbusError::InvalidFrame);
                }
                
                let byte_count = data[0] as usize;
                if data.len() < 1 + byte_count || byte_count % 2 != 0 {
                    return Err(ModbusError::InvalidFrame);
                }
                
                let mut registers = Vec::<u16, 125>::new();
                for i in (1..1 + byte_count).step_by(2) {
                    let register_value = ((data[i] as u16) << 8) | (data[i + 1] as u16);
                    registers.push(register_value).ok();
                }
                
                Ok(ModbusResponse::ReadRegisters(registers))
            }
            
            0x05 => {
                // 写单个线圈响应
                if data.len() < 4 {
                    return Err(ModbusError::InvalidFrame);
                }
                
                let addr = ((data[0] as u16) << 8) | (data[1] as u16);
                let value = data[2] == 0xFF;
                
                Ok(ModbusResponse::WriteSingleCoil { addr, value })
            }
            
            0x06 => {
                // 写单个寄存器响应
                if data.len() < 4 {
                    return Err(ModbusError::InvalidFrame);
                }
                
                let addr = ((data[0] as u16) << 8) | (data[1] as u16);
                let value = ((data[2] as u16) << 8) | (data[3] as u16);
                
                Ok(ModbusResponse::WriteSingleRegister { addr, value })
            }
            
            0x0F => {
                // 写多个线圈响应
                if data.len() < 4 {
                    return Err(ModbusError::InvalidFrame);
                }
                
                let start_addr = ((data[0] as u16) << 8) | (data[1] as u16);
                let quantity = ((data[2] as u16) << 8) | (data[3] as u16);
                
                Ok(ModbusResponse::WriteMultipleCoils { start_addr, quantity })
            }
            
            0x10 => {
                // 写多个寄存器响应
                if data.len() < 4 {
                    return Err(ModbusError::InvalidFrame);
                }
                
                let start_addr = ((data[0] as u16) << 8) | (data[1] as u16);
                let quantity = ((data[2] as u16) << 8) | (data[3] as u16);
                
                Ok(ModbusResponse::WriteMultipleRegisters { start_addr, quantity })
            }
            
            _ => Err(ModbusError::UnsupportedFunction),
        }
    }
    
    /// 获取统计信息
    pub fn get_statistics(&self) -> &ModbusStatistics {
        &self.statistics
    }
    
    /// 重置统计信息
    pub fn reset_statistics(&mut self) {
        self.statistics = ModbusStatistics {
            requests_sent: 0,
            responses_received: 0,
            timeouts: 0,
            crc_errors: 0,
            frame_errors: 0,
            exception_responses: 0,
        };
    }
    
    /// 获取当前状态
    pub fn get_state(&self) -> ModbusRtuState {
        self.state
    }
}

/// Modbus响应类型
#[derive(Clone)]
pub enum ModbusResponse {
    ReadCoils(Vec<bool, 2000>),
    ReadRegisters(Vec<u16, 125>),
    WriteSingleCoil { addr: u16, value: bool },
    WriteSingleRegister { addr: u16, value: u16 },
    WriteMultipleCoils { start_addr: u16, quantity: u16 },
    WriteMultipleRegisters { start_addr: u16, quantity: u16 },
}

/// Modbus错误类型
#[derive(Clone, Copy, PartialEq)]
pub enum ModbusError {
    Busy,
    InvalidParameter,
    TransmissionError,
    Timeout,
    InvalidFrame,
    CrcError,
    InvalidSlaveId,
    Exception(u8),
    BufferOverflow,
    UnsupportedFunction,
}
```

## Modbus TCP实现

### TCP协议栈
```rust
use heapless::{Vec, FnvIndexMap};

/// Modbus TCP帧头
#[derive(Clone, Copy)]
pub struct ModbusTcpHeader {
    pub transaction_id: u16,
    pub protocol_id: u16,
    pub length: u16,
    pub unit_id: u8,
}

/// Modbus TCP帧
#[derive(Clone)]
pub struct ModbusTcpFrame {
    pub header: ModbusTcpHeader,
    pub function_code: u8,
    pub data: Vec<u8, 252>,
}

/// TCP连接状态
#[derive(Clone, Copy, PartialEq)]
pub enum TcpConnectionState {
    Closed,
    Listening,
    Connected,
    Error,
}

/// Modbus TCP服务器
pub struct ModbusTcpServer {
    connections: FnvIndexMap<u16, TcpConnection, 8>,
    next_connection_id: u16,
    server_port: u16,
    max_connections: u8,
    data_model: ModbusDataModel,
    statistics: ModbusTcpStatistics,
}

/// TCP连接
pub struct TcpConnection {
    pub id: u16,
    pub state: TcpConnectionState,
    pub remote_addr: [u8; 4],
    pub remote_port: u16,
    pub rx_buffer: Vec<u8, 512>,
    pub tx_buffer: Vec<u8, 512>,
    pub last_activity: u32,
    pub bytes_received: u32,
    pub bytes_sent: u32,
}

/// Modbus数据模型
pub struct ModbusDataModel {
    pub coils: [bool; 10000],
    pub discrete_inputs: [bool; 10000],
    pub input_registers: [u16; 10000],
    pub holding_registers: [u16; 10000],
}

/// Modbus TCP统计信息
#[derive(Clone, Copy)]
pub struct ModbusTcpStatistics {
    pub connections_accepted: u32,
    pub connections_closed: u32,
    pub requests_processed: u32,
    pub responses_sent: u32,
    pub errors: u32,
    pub bytes_received: u32,
    pub bytes_sent: u32,
}

impl ModbusTcpServer {
    pub fn new(port: u16, max_connections: u8) -> Self {
        Self {
            connections: FnvIndexMap::new(),
            next_connection_id: 1,
            server_port: port,
            max_connections,
            data_model: ModbusDataModel {
                coils: [false; 10000],
                discrete_inputs: [false; 10000],
                input_registers: [0; 10000],
                holding_registers: [0; 10000],
            },
            statistics: ModbusTcpStatistics {
                connections_accepted: 0,
                connections_closed: 0,
                requests_processed: 0,
                responses_sent: 0,
                errors: 0,
                bytes_received: 0,
                bytes_sent: 0,
            },
        }
    }
    
    /// 接受新连接
    pub fn accept_connection(&mut self, remote_addr: [u8; 4], remote_port: u16) -> Result<u16, &'static str> {
        if self.connections.len() >= self.max_connections as usize {
            return Err("Maximum connections reached");
        }
        
        let connection_id = self.next_connection_id;
        self.next_connection_id += 1;
        
        let connection = TcpConnection {
            id: connection_id,
            state: TcpConnectionState::Connected,
            remote_addr,
            remote_port,
            rx_buffer: Vec::new(),
            tx_buffer: Vec::new(),
            last_activity: 0, // 应该使用系统时钟
            bytes_received: 0,
            bytes_sent: 0,
        };
        
        self.connections.insert(connection_id, connection).ok();
        self.statistics.connections_accepted += 1;
        
        Ok(connection_id)
    }
    
    /// 关闭连接
    pub fn close_connection(&mut self, connection_id: u16) {
        if self.connections.remove(&connection_id).is_some() {
            self.statistics.connections_closed += 1;
        }
    }
    
    /// 处理接收数据
    pub fn handle_receive(&mut self, connection_id: u16, data: &[u8]) -> Result<(), ModbusError> {
        if let Some(connection) = self.connections.get_mut(&connection_id) {
            // 添加数据到接收缓冲区
            for &byte in data {
                if connection.rx_buffer.push(byte).is_err() {
                    return Err(ModbusError::BufferOverflow);
                }
            }
            
            connection.bytes_received += data.len() as u32;
            self.statistics.bytes_received += data.len() as u32;
            
            // 处理完整的帧
            while let Some(frame) = self.extract_frame(&mut connection.rx_buffer)? {
                self.process_tcp_frame(connection_id, frame)?;
            }
            
            Ok(())
        } else {
            Err(ModbusError::InvalidSlaveId)
        }
    }
    
    /// 提取完整帧
    fn extract_frame(&self, buffer: &mut Vec<u8, 512>) -> Result<Option<ModbusTcpFrame>, ModbusError> {
        if buffer.len() < 8 {
            return Ok(None); // 不够一个完整的MBAP头
        }
        
        // 解析MBAP头
        let transaction_id = ((buffer[0] as u16) << 8) | (buffer[1] as u16);
        let protocol_id = ((buffer[2] as u16) << 8) | (buffer[3] as u16);
        let length = ((buffer[4] as u16) << 8) | (buffer[5] as u16);
        let unit_id = buffer[6];
        let function_code = buffer[7];
        
        // 检查协议ID
        if protocol_id != 0 {
            return Err(ModbusError::InvalidFrame);
        }
        
        // 检查长度
        if length < 2 || length > 253 {
            return Err(ModbusError::InvalidFrame);
        }
        
        let total_length = 6 + length as usize;
        if buffer.len() < total_length {
            return Ok(None); // 帧不完整
        }
        
        // 提取数据
        let mut data = Vec::<u8, 252>::new();
        for i in 8..total_length {
            data.push(buffer[i]).ok();
        }
        
        // 从缓冲区移除已处理的数据
        for _ in 0..total_length {
            buffer.remove(0);
        }
        
        let frame = ModbusTcpFrame {
            header: ModbusTcpHeader {
                transaction_id,
                protocol_id,
                length,
                unit_id,
            },
            function_code,
            data,
        };
        
        Ok(Some(frame))
    }
    
    /// 处理TCP帧
    fn process_tcp_frame(&mut self, connection_id: u16, frame: ModbusTcpFrame) -> Result<(), ModbusError> {
        let response = match frame.function_code {
            0x01 => self.handle_read_coils(&frame.data)?,
            0x02 => self.handle_read_discrete_inputs(&frame.data)?,
            0x03 => self.handle_read_holding_registers(&frame.data)?,
            0x04 => self.handle_read_input_registers(&frame.data)?,
            0x05 => self.handle_write_single_coil(&frame.data)?,
            0x06 => self.handle_write_single_register(&frame.data)?,
            0x0F => self.handle_write_multiple_coils(&frame.data)?,
            0x10 => self.handle_write_multiple_registers(&frame.data)?,
            _ => return self.send_exception_response(connection_id, &frame, 0x01), // 非法功能码
        };
        
        self.send_response(connection_id, &frame, response)?;
        self.statistics.requests_processed += 1;
        
        Ok(())
    }
    
    /// 处理读取线圈
    fn handle_read_coils(&self, data: &[u8]) -> Result<Vec<u8, 252>, ModbusError> {
        if data.len() < 4 {
            return Err(ModbusError::InvalidFrame);
        }
        
        let start_addr = ((data[0] as u16) << 8) | (data[1] as u16);
        let quantity = ((data[2] as u16) << 8) | (data[3] as u16);
        
        if quantity == 0 || quantity > 2000 {
            return Err(ModbusError::InvalidParameter);
        }
        
        if start_addr as usize + quantity as usize > self.data_model.coils.len() {
            return Err(ModbusError::InvalidParameter);
        }
        
        let byte_count = (quantity + 7) / 8;
        let mut response = Vec::<u8, 252>::new();
        response.push(byte_count as u8).ok();
        
        for byte_idx in 0..byte_count {
            let mut byte_value = 0u8;
            for bit_idx in 0..8 {
                let coil_idx = start_addr as usize + (byte_idx * 8 + bit_idx) as usize;
                if coil_idx < start_addr as usize + quantity as usize && coil_idx < self.data_model.coils.len() {
                    if self.data_model.coils[coil_idx] {
                        byte_value |= 1 << bit_idx;
                    }
                }
            }
            response.push(byte_value).ok();
        }
        
        Ok(response)
    }
    
    /// 处理读取保持寄存器
    fn handle_read_holding_registers(&self, data: &[u8]) -> Result<Vec<u8, 252>, ModbusError> {
        if data.len() < 4 {
            return Err(ModbusError::InvalidFrame);
        }
        
        let start_addr = ((data[0] as u16) << 8) | (data[1] as u16);
        let quantity = ((data[2] as u16) << 8) | (data[3] as u16);
        
        if quantity == 0 || quantity > 125 {
            return Err(ModbusError::InvalidParameter);
        }
        
        if start_addr as usize + quantity as usize > self.data_model.holding_registers.len() {
            return Err(ModbusError::InvalidParameter);
        }
        
        let mut response = Vec::<u8, 252>::new();
        response.push((quantity * 2) as u8).ok();
        
        for i in 0..quantity {
            let reg_value = self.data_model.holding_registers[start_addr as usize + i as usize];
            response.push((reg_value >> 8) as u8).ok();
            response.push((reg_value & 0xFF) as u8).ok();
        }
        
        Ok(response)
    }
    
    /// 处理写单个线圈
    fn handle_write_single_coil(&mut self, data: &[u8]) -> Result<Vec<u8, 252>, ModbusError> {
        if data.len() < 4 {
            return Err(ModbusError::InvalidFrame);
        }
        
        let addr = ((data[0] as u16) << 8) | (data[1] as u16);
        let value = ((data[2] as u16) << 8) | (data[3] as u16);
        
        if addr as usize >= self.data_model.coils.len() {
            return Err(ModbusError::InvalidParameter);
        }
        
        let coil_value = match value {
            0x0000 => false,
            0xFF00 => true,
            _ => return Err(ModbusError::InvalidParameter),
        };
        
        self.data_model.coils[addr as usize] = coil_value;
        
        // 回显请求
        let mut response = Vec::<u8, 252>::new();
        for &byte in data.iter().take(4) {
            response.push(byte).ok();
        }
        
        Ok(response)
    }
    
    /// 发送响应
    fn send_response(&mut self, connection_id: u16, request: &ModbusTcpFrame, response_data: Vec<u8, 252>) -> Result<(), ModbusError> {
        if let Some(connection) = self.connections.get_mut(&connection_id) {
            connection.tx_buffer.clear();
            
            // MBAP头
            let length = 2 + response_data.len() as u16;
            connection.tx_buffer.push((request.header.transaction_id >> 8) as u8).ok();
            connection.tx_buffer.push((request.header.transaction_id & 0xFF) as u8).ok();
            connection.tx_buffer.push(0x00).ok(); // 协议ID高字节
            connection.tx_buffer.push(0x00).ok(); // 协议ID低字节
            connection.tx_buffer.push((length >> 8) as u8).ok();
            connection.tx_buffer.push((length & 0xFF) as u8).ok();
            connection.tx_buffer.push(request.header.unit_id).ok();
            
            // 功能码
            connection.tx_buffer.push(request.function_code).ok();
            
            // 响应数据
            for &byte in &response_data {
                connection.tx_buffer.push(byte).ok();
            }
            
            connection.bytes_sent += connection.tx_buffer.len() as u32;
            self.statistics.bytes_sent += connection.tx_buffer.len() as u32;
            self.statistics.responses_sent += 1;
            
            Ok(())
        } else {
            Err(ModbusError::InvalidSlaveId)
        }
    }
    
    /// 发送异常响应
    fn send_exception_response(&mut self, connection_id: u16, request: &ModbusTcpFrame, exception_code: u8) -> Result<(), ModbusError> {
        if let Some(connection) = self.connections.get_mut(&connection_id) {
            connection.tx_buffer.clear();
            
            // MBAP头
            connection.tx_buffer.push((request.header.transaction_id >> 8) as u8).ok();
            connection.tx_buffer.push((request.header.transaction_id & 0xFF) as u8).ok();
            connection.tx_buffer.push(0x00).ok();
            connection.tx_buffer.push(0x00).ok();
            connection.tx_buffer.push(0x00).ok();
            connection.tx_buffer.push(0x03).ok(); // 长度 = 3
            connection.tx_buffer.push(request.header.unit_id).ok();
            
            // 异常响应
            connection.tx_buffer.push(request.function_code | 0x80).ok();
            connection.tx_buffer.push(exception_code).ok();
            
            self.statistics.errors += 1;
            
            Ok(())
        } else {
            Err(ModbusError::InvalidSlaveId)
        }
    }
    
    /// 获取发送数据
    pub fn get_tx_data(&mut self, connection_id: u16) -> Option<&[u8]> {
        if let Some(connection) = self.connections.get(&connection_id) {
            if !connection.tx_buffer.is_empty() {
                return Some(&connection.tx_buffer);
            }
        }
        None
    }
    
    /// 清除发送缓冲区
    pub fn clear_tx_buffer(&mut self, connection_id: u16) {
        if let Some(connection) = self.connections.get_mut(&connection_id) {
            connection.tx_buffer.clear();
        }
    }
    
    /// 更新数据模型
    pub fn update_discrete_inputs(&mut self, start_addr: u16, values: &[bool]) {
        let start = start_addr as usize;
        let end = (start + values.len()).min(self.data_model.discrete_inputs.len());
        let len = end - start;
        
        if len > 0 {
            self.data_model.discrete_inputs[start..end].copy_from_slice(&values[..len]);
        }
    }
    
    pub fn update_input_registers(&mut self, start_addr: u16, values: &[u16]) {
        let start = start_addr as usize;
        let end = (start + values.len()).min(self.data_model.input_registers.len());
        let len = end - start;
        
        if len > 0 {
            self.data_model.input_registers[start..end].copy_from_slice(&values[..len]);
        }
    }
    
    /// 获取输出数据
    pub fn get_coils(&self, start_addr: u16, count: u16) -> &[bool] {
        let start = start_addr as usize;
        let end = (start + count as usize).min(self.data_model.coils.len());
        &self.data_model.coils[start..end]
    }
    
    pub fn get_holding_registers(&self, start_addr: u16, count: u16) -> &[u16] {
        let start = start_addr as usize;
        let end = (start + count as usize).min(self.data_model.holding_registers.len());
        &self.data_model.holding_registers[start..end]
    }
    
    // 其他功能码处理方法的简化实现
    fn handle_read_discrete_inputs(&self, _data: &[u8]) -> Result<Vec<u8, 252>, ModbusError> {
        Err(ModbusError::UnsupportedFunction)
    }
    
    fn handle_read_input_registers(&self, _data: &[u8]) -> Result<Vec<u8, 252>, ModbusError> {
        Err(ModbusError::UnsupportedFunction)
    }
    
    fn handle_write_single_register(&mut self, _data: &[u8]) -> Result<Vec<u8, 252>, ModbusError> {
        Err(ModbusError::UnsupportedFunction)
    }
    
    fn handle_write_multiple_coils(&mut self, _data: &[u8]) -> Result<Vec<u8, 252>, ModbusError> {
        Err(ModbusError::UnsupportedFunction)
    }
    
    fn handle_write_multiple_registers(&mut self, _data: &[u8]) -> Result<Vec<u8, 252>, ModbusError> {
        Err(ModbusError::UnsupportedFunction)
    }
}
```

## CRC16计算优化

### 查表法CRC计算
```rust
/// CRC16查找表
const CRC16_TABLE: [u16; 256] = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
];

/// 快速CRC16计算
pub fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    
    for &byte in data {
        let table_index = ((crc ^ (byte as u16)) & 0xFF) as usize;
        crc = (crc >> 8) ^ CRC16_TABLE[table_index];
    }
    
    crc
}

/// 增量CRC16计算
pub struct Crc16Calculator {
    crc: u16,
}

impl Crc16Calculator {
    pub fn new() -> Self {
        Self { crc: 0xFFFF }
    }
    
    pub fn update(&mut self, byte: u8) {
        let table_index = ((self.crc ^ (byte as u16)) & 0xFF) as usize;
        self.crc = (self.crc >> 8) ^ CRC16_TABLE[table_index];
    }
    
    pub fn finalize(self) -> u16 {
        self.crc
    }
    
    pub fn reset(&mut self) {
        self.crc = 0xFFFF;
    }
}
```

## 性能优化和调试

### 性能监控
```rust
/// Modbus性能监控
pub struct ModbusPerformanceMonitor {
    pub request_count: u32,
    pub response_count: u32,
    pub error_count: u32,
    pub total_response_time: u32,  // μs
    pub max_response_time: u32,    // μs
    pub min_response_time: u32,    // μs
    pub throughput_bps: u32,       // bytes per second
    pub last_reset_time: u32,
}

impl ModbusPerformanceMonitor {
    pub fn new() -> Self {
        Self {
            request_count: 0,
            response_count: 0,
            error_count: 0,
            total_response_time: 0,
            max_response_time: 0,
            min_response_time: u32::MAX,
            throughput_bps: 0,
            last_reset_time: 0,
        }
    }
    
    pub fn record_request(&mut self) {
        self.request_count += 1;
    }
    
    pub fn record_response(&mut self, response_time: u32) {
        self.response_count += 1;
        self.total_response_time += response_time;
        
        if response_time > self.max_response_time {
            self.max_response_time = response_time;
        }
        
        if response_time < self.min_response_time {
            self.min_response_time = response_time;
        }
    }
    
    pub fn record_error(&mut self) {
        self.error_count += 1;
    }
    
    pub fn get_average_response_time(&self) -> u32 {
        if self.response_count > 0 {
            self.total_response_time / self.response_count
        } else {
            0
        }
    }
    
    pub fn get_success_rate(&self) -> f32 {
        if self.request_count > 0 {
            (self.response_count as f32) / (self.request_count as f32) * 100.0
        } else {
            0.0
        }
    }
    
    pub fn reset(&mut self, current_time: u32) {
        *self = Self::new();
        self.last_reset_time = current_time;
    }
}
```

## 总结

本文档详细介绍了Modbus协议在STM32F4平台上的实现，包括：

1. **Modbus RTU实现**: 完整的主站协议栈，支持所有标准功能码
2. **Modbus TCP实现**: 基于TCP的服务器实现，支持多连接
3. **CRC计算优化**: 使用查表法提高CRC计算效率
4. **性能监控**: 实时监控协议性能和统计信息

这些实现为工业控制系统提供了可靠的通信基础，支持与各种Modbus设备的互操作。