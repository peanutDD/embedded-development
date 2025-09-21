#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    gpio::{Output, PushPull, Pin},
    timer::{Timer, Event},
    serial::{Serial, Config},
};

use heapless::{Vec, FnvIndexMap, pool::{Pool, Node}};
use smoltcp::{
    iface::{Interface, InterfaceBuilder, NeighborCache, Routes},
    socket::{TcpSocket, TcpSocketBuffer},
    time::Instant,
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
};
use nb;

/// Modbus网关配置
const MAX_RTU_SLAVES: usize = 16;
const MAX_TCP_CONNECTIONS: usize = 8;
const GATEWAY_BUFFER_SIZE: usize = 512;
const RTU_TIMEOUT_MS: u32 = 1000;
const TCP_TIMEOUT_MS: u32 = 5000;

/// Modbus功能码
#[derive(Clone, Copy, PartialEq)]
pub enum ModbusFunction {
    ReadCoils = 0x01,
    ReadDiscreteInputs = 0x02,
    ReadHoldingRegisters = 0x03,
    ReadInputRegisters = 0x04,
    WriteSingleCoil = 0x05,
    WriteSingleRegister = 0x06,
    WriteMultipleCoils = 0x0F,
    WriteMultipleRegisters = 0x10,
}

/// Modbus RTU帧
#[derive(Clone)]
pub struct ModbusRtuFrame {
    pub slave_id: u8,
    pub function_code: u8,
    pub data: Vec<u8, 252>,
    pub crc: u16,
}

/// Modbus TCP帧
#[derive(Clone)]
pub struct ModbusTcpFrame {
    pub transaction_id: u16,
    pub protocol_id: u16,
    pub length: u16,
    pub unit_id: u8,
    pub function_code: u8,
    pub data: Vec<u8, 252>,
}

/// 网关请求状态
#[derive(Clone, Copy, PartialEq)]
pub enum GatewayRequestState {
    Idle,
    WaitingRtuResponse,
    SendingTcpResponse,
    Error,
}

/// 网关请求
#[derive(Clone)]
pub struct GatewayRequest {
    pub tcp_transaction_id: u16,
    pub tcp_connection_id: u16,
    pub rtu_slave_id: u8,
    pub function_code: u8,
    pub request_data: Vec<u8, 252>,
    pub response_data: Vec<u8, 252>,
    pub state: GatewayRequestState,
    pub timestamp: u32,
    pub retry_count: u8,
}

impl GatewayRequest {
    pub fn new(tcp_transaction_id: u16, tcp_connection_id: u16, rtu_slave_id: u8, function_code: u8) -> Self {
        Self {
            tcp_transaction_id,
            tcp_connection_id,
            rtu_slave_id,
            function_code,
            request_data: Vec::new(),
            response_data: Vec::new(),
            state: GatewayRequestState::Idle,
            timestamp: 0,
            retry_count: 0,
        }
    }
}

/// RTU从站配置
#[derive(Clone, Copy)]
pub struct RtuSlaveConfig {
    pub slave_id: u8,
    pub enabled: bool,
    pub timeout_ms: u32,
    pub retry_count: u8,
    pub last_communication: u32,
    pub error_count: u32,
    pub success_count: u32,
}

impl RtuSlaveConfig {
    pub fn new(slave_id: u8) -> Self {
        Self {
            slave_id,
            enabled: true,
            timeout_ms: RTU_TIMEOUT_MS,
            retry_count: 3,
            last_communication: 0,
            error_count: 0,
            success_count: 0,
        }
    }
}

/// TCP连接状态
#[derive(Clone, Copy, PartialEq)]
pub enum TcpConnectionState {
    Closed,
    Connected,
    Error,
}

/// TCP连接
pub struct TcpConnection {
    pub id: u16,
    pub state: TcpConnectionState,
    pub socket_handle: Option<smoltcp::socket::SocketHandle>,
    pub rx_buffer: Vec<u8, GATEWAY_BUFFER_SIZE>,
    pub tx_buffer: Vec<u8, GATEWAY_BUFFER_SIZE>,
    pub last_activity: u32,
    pub bytes_received: u32,
    pub bytes_sent: u32,
}

impl TcpConnection {
    pub fn new(id: u16) -> Self {
        Self {
            id,
            state: TcpConnectionState::Closed,
            socket_handle: None,
            rx_buffer: Vec::new(),
            tx_buffer: Vec::new(),
            last_activity: 0,
            bytes_received: 0,
            bytes_sent: 0,
        }
    }
}

/// Modbus网关主结构
pub struct ModbusGateway {
    // RTU配置
    pub rtu_slaves: [RtuSlaveConfig; MAX_RTU_SLAVES],
    pub rtu_tx_buffer: Vec<u8, 256>,
    pub rtu_rx_buffer: Vec<u8, 256>,
    
    // TCP连接
    pub tcp_connections: [TcpConnection; MAX_TCP_CONNECTIONS],
    pub next_connection_id: u16,
    
    // 请求队列
    pub pending_requests: Vec<GatewayRequest, 32>,
    pub current_request: Option<GatewayRequest>,
    
    // 统计信息
    pub total_requests: u32,
    pub successful_requests: u32,
    pub failed_requests: u32,
    pub rtu_timeouts: u32,
    pub tcp_timeouts: u32,
    
    // 系统状态
    pub system_time: u32,
    pub gateway_enabled: bool,
    pub error_flags: u32,
}

impl ModbusGateway {
    pub fn new() -> Self {
        let mut gateway = Self {
            rtu_slaves: [RtuSlaveConfig::new(0); MAX_RTU_SLAVES],
            rtu_tx_buffer: Vec::new(),
            rtu_rx_buffer: Vec::new(),
            tcp_connections: [
                TcpConnection::new(0), TcpConnection::new(1), TcpConnection::new(2), TcpConnection::new(3),
                TcpConnection::new(4), TcpConnection::new(5), TcpConnection::new(6), TcpConnection::new(7),
            ],
            next_connection_id: 1,
            pending_requests: Vec::new(),
            current_request: None,
            total_requests: 0,
            successful_requests: 0,
            failed_requests: 0,
            rtu_timeouts: 0,
            tcp_timeouts: 0,
            system_time: 0,
            gateway_enabled: true,
            error_flags: 0,
        };
        
        // 初始化RTU从站配置
        for i in 0..MAX_RTU_SLAVES {
            gateway.rtu_slaves[i] = RtuSlaveConfig::new((i + 1) as u8);
        }
        
        gateway
    }
    
    /// 处理TCP连接
    pub fn handle_tcp_connection(&mut self, connection_id: u16, data: &[u8]) -> Result<(), &'static str> {
        if let Some(connection) = self.get_tcp_connection_mut(connection_id) {
            // 添加接收数据到缓冲区
            for &byte in data {
                if connection.rx_buffer.push(byte).is_err() {
                    return Err("TCP receive buffer overflow");
                }
            }
            
            connection.bytes_received += data.len() as u32;
            connection.last_activity = self.system_time;
            
            // 处理完整的Modbus TCP帧
            while let Some(tcp_frame) = self.extract_tcp_frame(&mut connection.rx_buffer)? {
                self.process_tcp_request(connection_id, tcp_frame)?;
            }
            
            Ok(())
        } else {
            Err("Invalid connection ID")
        }
    }
    
    /// 提取TCP帧
    fn extract_tcp_frame(&self, buffer: &mut Vec<u8, GATEWAY_BUFFER_SIZE>) -> Result<Option<ModbusTcpFrame>, &'static str> {
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
            return Err("Invalid protocol ID");
        }
        
        // 检查长度
        if length < 2 || length > 253 {
            return Err("Invalid frame length");
        }
        
        let total_length = 6 + length as usize;
        if buffer.len() < total_length {
            return Ok(None); // 帧不完整
        }
        
        // 提取数据
        let mut data = Vec::<u8, 252>::new();
        for i in 8..total_length {
            data.push(buffer[i]).map_err(|_| "Data buffer overflow")?;
        }
        
        // 从缓冲区移除已处理的数据
        for _ in 0..total_length {
            buffer.remove(0);
        }
        
        let frame = ModbusTcpFrame {
            transaction_id,
            protocol_id,
            length,
            unit_id,
            function_code,
            data,
        };
        
        Ok(Some(frame))
    }
    
    /// 处理TCP请求
    fn process_tcp_request(&mut self, connection_id: u16, tcp_frame: ModbusTcpFrame) -> Result<(), &'static str> {
        // 检查从站ID是否有效
        if tcp_frame.unit_id == 0 || tcp_frame.unit_id > MAX_RTU_SLAVES as u8 {
            return self.send_tcp_exception(connection_id, &tcp_frame, 0x0A); // Gateway target device failed to respond
        }
        
        // 检查从站是否启用
        let slave_index = (tcp_frame.unit_id - 1) as usize;
        if !self.rtu_slaves[slave_index].enabled {
            return self.send_tcp_exception(connection_id, &tcp_frame, 0x0B); // Gateway target device failed to respond
        }
        
        // 创建网关请求
        let mut request = GatewayRequest::new(
            tcp_frame.transaction_id,
            connection_id,
            tcp_frame.unit_id,
            tcp_frame.function_code,
        );
        
        request.request_data = tcp_frame.data;
        request.timestamp = self.system_time;
        
        // 添加到请求队列
        if self.pending_requests.push(request).is_err() {
            return self.send_tcp_exception(connection_id, &tcp_frame, 0x06); // Slave device busy
        }
        
        self.total_requests += 1;
        
        Ok(())
    }
    
    /// 发送TCP异常响应
    fn send_tcp_exception(&mut self, connection_id: u16, tcp_frame: &ModbusTcpFrame, exception_code: u8) -> Result<(), &'static str> {
        if let Some(connection) = self.get_tcp_connection_mut(connection_id) {
            connection.tx_buffer.clear();
            
            // MBAP头
            connection.tx_buffer.push((tcp_frame.transaction_id >> 8) as u8).ok();
            connection.tx_buffer.push((tcp_frame.transaction_id & 0xFF) as u8).ok();
            connection.tx_buffer.push(0x00).ok(); // 协议ID高字节
            connection.tx_buffer.push(0x00).ok(); // 协议ID低字节
            connection.tx_buffer.push(0x00).ok(); // 长度高字节
            connection.tx_buffer.push(0x03).ok(); // 长度低字节 (3字节)
            connection.tx_buffer.push(tcp_frame.unit_id).ok();
            
            // 异常响应
            connection.tx_buffer.push(tcp_frame.function_code | 0x80).ok();
            connection.tx_buffer.push(exception_code).ok();
            
            Ok(())
        } else {
            Err("Invalid connection ID")
        }
    }
    
    /// 处理RTU通信
    pub fn handle_rtu_communication(&mut self) -> Result<(), &'static str> {
        // 处理当前请求
        if let Some(mut request) = self.current_request.take() {
            match request.state {
                GatewayRequestState::WaitingRtuResponse => {
                    // 检查RTU响应超时
                    if self.system_time - request.timestamp > RTU_TIMEOUT_MS {
                        request.retry_count += 1;
                        if request.retry_count >= 3 {
                            // 发送TCP异常响应
                            self.send_tcp_exception(
                                request.tcp_connection_id,
                                &ModbusTcpFrame {
                                    transaction_id: request.tcp_transaction_id,
                                    protocol_id: 0,
                                    length: 0,
                                    unit_id: request.rtu_slave_id,
                                    function_code: request.function_code,
                                    data: Vec::new(),
                                },
                                0x0B, // Gateway target device failed to respond
                            )?;
                            
                            self.failed_requests += 1;
                            self.rtu_timeouts += 1;
                        } else {
                            // 重试RTU请求
                            request.state = GatewayRequestState::Idle;
                            request.timestamp = self.system_time;
                            self.current_request = Some(request);
                        }
                    } else {
                        // 检查RTU响应
                        if let Some(rtu_response) = self.check_rtu_response()? {
                            // 转换为TCP响应
                            self.convert_rtu_to_tcp_response(&mut request, rtu_response)?;
                            request.state = GatewayRequestState::SendingTcpResponse;
                        }
                        self.current_request = Some(request);
                    }
                }
                
                GatewayRequestState::SendingTcpResponse => {
                    // 发送TCP响应
                    self.send_tcp_response(&request)?;
                    self.successful_requests += 1;
                    
                    // 更新从站统计
                    let slave_index = (request.rtu_slave_id - 1) as usize;
                    self.rtu_slaves[slave_index].success_count += 1;
                    self.rtu_slaves[slave_index].last_communication = self.system_time;
                }
                
                _ => {
                    self.current_request = Some(request);
                }
            }
        }
        
        // 处理新请求
        if self.current_request.is_none() && !self.pending_requests.is_empty() {
            let mut request = self.pending_requests.remove(0);
            
            // 发送RTU请求
            self.send_rtu_request(&mut request)?;
            request.state = GatewayRequestState::WaitingRtuResponse;
            request.timestamp = self.system_time;
            
            self.current_request = Some(request);
        }
        
        Ok(())
    }
    
    /// 发送RTU请求
    fn send_rtu_request(&mut self, request: &mut GatewayRequest) -> Result<(), &'static str> {
        self.rtu_tx_buffer.clear();
        
        // 构建RTU帧
        self.rtu_tx_buffer.push(request.rtu_slave_id).ok();
        self.rtu_tx_buffer.push(request.function_code).ok();
        
        for &byte in &request.request_data {
            self.rtu_tx_buffer.push(byte).ok();
        }
        
        // 计算CRC
        let crc = self.calculate_crc16(&self.rtu_tx_buffer);
        self.rtu_tx_buffer.push((crc & 0xFF) as u8).ok();
        self.rtu_tx_buffer.push((crc >> 8) as u8).ok();
        
        // 发送数据（这里需要实际的串口发送实现）
        self.send_rtu_data(&self.rtu_tx_buffer)?;
        
        Ok(())
    }
    
    /// 检查RTU响应
    fn check_rtu_response(&mut self) -> Result<Option<ModbusRtuFrame>, &'static str> {
        // 读取串口数据（这里需要实际的串口读取实现）
        let received_data = self.receive_rtu_data()?;
        
        if received_data.is_empty() {
            return Ok(None);
        }
        
        // 添加到接收缓冲区
        for &byte in &received_data {
            if self.rtu_rx_buffer.push(byte).is_err() {
                self.rtu_rx_buffer.clear();
                return Err("RTU receive buffer overflow");
            }
        }
        
        // 检查是否有完整帧
        if self.rtu_rx_buffer.len() >= 4 {
            // 验证CRC
            let data_len = self.rtu_rx_buffer.len() - 2;
            let received_crc = (self.rtu_rx_buffer[data_len + 1] as u16) << 8 | (self.rtu_rx_buffer[data_len] as u16);
            let calculated_crc = self.calculate_crc16(&self.rtu_rx_buffer[..data_len]);
            
            if received_crc == calculated_crc {
                // 解析帧
                let slave_id = self.rtu_rx_buffer[0];
                let function_code = self.rtu_rx_buffer[1];
                
                let mut data = Vec::<u8, 252>::new();
                for i in 2..data_len {
                    data.push(self.rtu_rx_buffer[i]).ok();
                }
                
                let frame = ModbusRtuFrame {
                    slave_id,
                    function_code,
                    data,
                    crc: calculated_crc,
                };
                
                self.rtu_rx_buffer.clear();
                return Ok(Some(frame));
            } else {
                self.rtu_rx_buffer.clear();
                return Err("RTU CRC error");
            }
        }
        
        Ok(None)
    }
    
    /// 转换RTU响应为TCP响应
    fn convert_rtu_to_tcp_response(&mut self, request: &mut GatewayRequest, rtu_response: ModbusRtuFrame) -> Result<(), &'static str> {
        // 检查从站ID和功能码
        if rtu_response.slave_id != request.rtu_slave_id {
            return Err("RTU slave ID mismatch");
        }
        
        // 检查异常响应
        if rtu_response.function_code & 0x80 != 0 {
            // 异常响应，直接转发
            request.response_data.clear();
            if !rtu_response.data.is_empty() {
                request.response_data.push(rtu_response.data[0]).ok(); // 异常码
            }
        } else if rtu_response.function_code == request.function_code {
            // 正常响应
            request.response_data = rtu_response.data;
        } else {
            return Err("RTU function code mismatch");
        }
        
        Ok(())
    }
    
    /// 发送TCP响应
    fn send_tcp_response(&mut self, request: &GatewayRequest) -> Result<(), &'static str> {
        if let Some(connection) = self.get_tcp_connection_mut(request.tcp_connection_id) {
            connection.tx_buffer.clear();
            
            // MBAP头
            let length = 2 + request.response_data.len() as u16;
            connection.tx_buffer.push((request.tcp_transaction_id >> 8) as u8).ok();
            connection.tx_buffer.push((request.tcp_transaction_id & 0xFF) as u8).ok();
            connection.tx_buffer.push(0x00).ok(); // 协议ID高字节
            connection.tx_buffer.push(0x00).ok(); // 协议ID低字节
            connection.tx_buffer.push((length >> 8) as u8).ok();
            connection.tx_buffer.push((length & 0xFF) as u8).ok();
            connection.tx_buffer.push(request.rtu_slave_id).ok();
            
            // 功能码和数据
            connection.tx_buffer.push(request.function_code).ok();
            for &byte in &request.response_data {
                connection.tx_buffer.push(byte).ok();
            }
            
            connection.bytes_sent += connection.tx_buffer.len() as u32;
            
            Ok(())
        } else {
            Err("Invalid connection ID")
        }
    }
    
    /// 获取TCP连接（可变引用）
    fn get_tcp_connection_mut(&mut self, connection_id: u16) -> Option<&mut TcpConnection> {
        self.tcp_connections.iter_mut().find(|conn| conn.id == connection_id)
    }
    
    /// 计算CRC16
    fn calculate_crc16(&self, data: &[u8]) -> u16 {
        let mut crc = 0xFFFFu16;
        
        for &byte in data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if crc & 0x0001 != 0 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        
        crc
    }
    
    /// 发送RTU数据（需要实际实现）
    fn send_rtu_data(&self, _data: &[u8]) -> Result<(), &'static str> {
        // 这里应该通过串口发送数据
        Ok(())
    }
    
    /// 接收RTU数据（需要实际实现）
    fn receive_rtu_data(&self) -> Result<Vec<u8, 256>, &'static str> {
        // 这里应该从串口接收数据
        Ok(Vec::new())
    }
    
    /// 更新系统时间
    pub fn update_system_time(&mut self, time_ms: u32) {
        self.system_time = time_ms;
    }
    
    /// 获取网关统计信息
    pub fn get_statistics(&self) -> GatewayStatistics {
        GatewayStatistics {
            total_requests: self.total_requests,
            successful_requests: self.successful_requests,
            failed_requests: self.failed_requests,
            rtu_timeouts: self.rtu_timeouts,
            tcp_timeouts: self.tcp_timeouts,
            active_connections: self.tcp_connections.iter().filter(|conn| conn.state == TcpConnectionState::Connected).count() as u8,
            pending_requests: self.pending_requests.len() as u8,
        }
    }
    
    /// 重置统计信息
    pub fn reset_statistics(&mut self) {
        self.total_requests = 0;
        self.successful_requests = 0;
        self.failed_requests = 0;
        self.rtu_timeouts = 0;
        self.tcp_timeouts = 0;
        
        for slave in &mut self.rtu_slaves {
            slave.error_count = 0;
            slave.success_count = 0;
        }
    }
}

/// 网关统计信息
#[derive(Clone, Copy)]
pub struct GatewayStatistics {
    pub total_requests: u32,
    pub successful_requests: u32,
    pub failed_requests: u32,
    pub rtu_timeouts: u32,
    pub tcp_timeouts: u32,
    pub active_connections: u8,
    pub pending_requests: u8,
}

/// 主函数
#[entry]
fn main() -> ! {
    // 获取设备外设
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置串口（RTU通信）
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    let serial_config = Config::default().baudrate(9600.bps());
    let mut serial = Serial::new(dp.USART1, (tx_pin, rx_pin), serial_config, &clocks).unwrap();
    
    // 配置以太网（TCP通信）
    // 这里需要实际的以太网配置代码
    
    // 配置系统定时器
    let mut timer = Timer::new(dp.TIM2, &clocks);
    timer.start(10.millis()); // 10ms周期
    timer.listen(Event::TimeOut);
    
    // 创建Modbus网关
    let mut gateway = ModbusGateway::new();
    
    let mut system_time = 0u32;
    
    // 主循环
    loop {
        // 等待定时器中断
        nb::block!(timer.wait()).unwrap();
        
        system_time += 10; // 增加10ms
        gateway.update_system_time(system_time);
        
        // 处理RTU通信
        if let Err(_) = gateway.handle_rtu_communication() {
            // 处理RTU通信错误
        }
        
        // 处理TCP连接（这里需要实际的网络处理代码）
        // handle_tcp_connections(&mut gateway);
        
        // 可选：更新状态LED、处理用户界面等
    }
}