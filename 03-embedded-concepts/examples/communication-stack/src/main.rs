#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{config::Config, Serial},
    spi::{Spi, Mode, Phase, Polarity},
    i2c::I2c,
    gpio::{gpioa::*, gpiob::*, gpioc::*, Alternate, Output, PushPull},
    timer::{CounterUs, Event},
};
use cortex_m::{interrupt, peripheral::NVIC};
use heapless::{Vec, spsc::{Producer, Consumer, Queue}};

// 协议栈层次定义
#[derive(Debug, Clone, Copy, PartialEq)]
enum ProtocolLayer {
    Physical,
    DataLink,
    Network,
    Transport,
    Application,
}

// 数据包结构
#[derive(Debug, Clone)]
struct Packet {
    layer: ProtocolLayer,
    source: u8,
    destination: u8,
    packet_type: PacketType,
    sequence: u16,
    payload: Vec<u8, 256>,
    checksum: u16,
    timestamp: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum PacketType {
    Data,
    Ack,
    Nack,
    Control,
    Heartbeat,
}

impl Packet {
    fn new(layer: ProtocolLayer, source: u8, destination: u8, packet_type: PacketType) -> Self {
        Self {
            layer,
            source,
            destination,
            packet_type,
            sequence: 0,
            payload: Vec::new(),
            checksum: 0,
            timestamp: 0,
        }
    }
    
    fn calculate_checksum(&mut self) {
        // 简化的校验和计算
        let mut sum = 0u16;
        sum = sum.wrapping_add(self.source as u16);
        sum = sum.wrapping_add(self.destination as u16);
        sum = sum.wrapping_add(self.packet_type as u16);
        sum = sum.wrapping_add(self.sequence);
        
        for &byte in &self.payload {
            sum = sum.wrapping_add(byte as u16);
        }
        
        self.checksum = !sum; // 取反作为校验和
    }
    
    fn verify_checksum(&self) -> bool {
        let mut sum = 0u16;
        sum = sum.wrapping_add(self.source as u16);
        sum = sum.wrapping_add(self.destination as u16);
        sum = sum.wrapping_add(self.packet_type as u16);
        sum = sum.wrapping_add(self.sequence);
        sum = sum.wrapping_add(self.checksum);
        
        for &byte in &self.payload {
            sum = sum.wrapping_add(byte as u16);
        }
        
        sum == 0xFFFF
    }
    
    fn serialize(&self) -> Vec<u8, 512> {
        let mut buffer = Vec::new();
        
        // 添加头部
        let _ = buffer.push(0xAA); // 起始标志
        let _ = buffer.push(0x55);
        let _ = buffer.push(self.layer as u8);
        let _ = buffer.push(self.source);
        let _ = buffer.push(self.destination);
        let _ = buffer.push(self.packet_type as u8);
        
        // 添加序列号（大端序）
        let _ = buffer.push((self.sequence >> 8) as u8);
        let _ = buffer.push(self.sequence as u8);
        
        // 添加载荷长度
        let _ = buffer.push(self.payload.len() as u8);
        
        // 添加载荷
        for &byte in &self.payload {
            if buffer.push(byte).is_err() {
                break;
            }
        }
        
        // 添加校验和
        let _ = buffer.push((self.checksum >> 8) as u8);
        let _ = buffer.push(self.checksum as u8);
        
        // 添加结束标志
        let _ = buffer.push(0x55);
        let _ = buffer.push(0xAA);
        
        buffer
    }
    
    fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 13 {
            return None; // 最小包长度
        }
        
        // 检查起始标志
        if data[0] != 0xAA || data[1] != 0x55 {
            return None;
        }
        
        let layer = match data[2] {
            0 => ProtocolLayer::Physical,
            1 => ProtocolLayer::DataLink,
            2 => ProtocolLayer::Network,
            3 => ProtocolLayer::Transport,
            4 => ProtocolLayer::Application,
            _ => return None,
        };
        
        let source = data[3];
        let destination = data[4];
        let packet_type = match data[5] {
            0 => PacketType::Data,
            1 => PacketType::Ack,
            2 => PacketType::Nack,
            3 => PacketType::Control,
            4 => PacketType::Heartbeat,
            _ => return None,
        };
        
        let sequence = ((data[6] as u16) << 8) | (data[7] as u16);
        let payload_len = data[8] as usize;
        
        if data.len() < 13 + payload_len {
            return None;
        }
        
        let mut payload = Vec::new();
        for i in 0..payload_len {
            if payload.push(data[9 + i]).is_err() {
                return None;
            }
        }
        
        let checksum_offset = 9 + payload_len;
        let checksum = ((data[checksum_offset] as u16) << 8) | (data[checksum_offset + 1] as u16);
        
        // 检查结束标志
        let end_offset = checksum_offset + 2;
        if data.len() < end_offset + 2 || data[end_offset] != 0x55 || data[end_offset + 1] != 0xAA {
            return None;
        }
        
        let mut packet = Self {
            layer,
            source,
            destination,
            packet_type,
            sequence,
            payload,
            checksum,
            timestamp: 0,
        };
        
        // 验证校验和
        if !packet.verify_checksum() {
            return None;
        }
        
        Some(packet)
    }
}

// 物理层接口
trait PhysicalLayer {
    fn send(&mut self, data: &[u8]) -> Result<(), CommError>;
    fn receive(&mut self) -> Result<Option<Vec<u8, 512>>, CommError>;
    fn is_ready(&self) -> bool;
}

// UART物理层实现
struct UartPhysicalLayer {
    serial: Serial<pac::USART1, (PA9<Alternate<7>>, PA10<Alternate<7>>)>,
    tx_buffer: Vec<u8, 512>,
    rx_buffer: Vec<u8, 512>,
}

impl UartPhysicalLayer {
    fn new(
        usart: pac::USART1,
        pins: (PA9<Alternate<7>>, PA10<Alternate<7>>),
        clocks: &stm32f4xx_hal::rcc::Clocks,
    ) -> Self {
        let config = Config::default().baudrate(115200.bps());
        let serial = Serial::usart1(usart, pins, config, clocks).unwrap();
        
        Self {
            serial,
            tx_buffer: Vec::new(),
            rx_buffer: Vec::new(),
        }
    }
}

impl PhysicalLayer for UartPhysicalLayer {
    fn send(&mut self, data: &[u8]) -> Result<(), CommError> {
        for &byte in data {
            match self.serial.write(byte) {
                Ok(_) => {}
                Err(nb::Error::WouldBlock) => return Err(CommError::Busy),
                Err(_) => return Err(CommError::HardwareError),
            }
        }
        Ok(())
    }
    
    fn receive(&mut self) -> Result<Option<Vec<u8, 512>>, CommError> {
        let mut received_data = Vec::new();
        
        loop {
            match self.serial.read() {
                Ok(byte) => {
                    if received_data.push(byte).is_err() {
                        break;
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(_) => return Err(CommError::HardwareError),
            }
        }
        
        if received_data.is_empty() {
            Ok(None)
        } else {
            Ok(Some(received_data))
        }
    }
    
    fn is_ready(&self) -> bool {
        true // UART通常总是就绪的
    }
}

// SPI物理层实现
struct SpiPhysicalLayer {
    spi: Spi<pac::SPI1, (PB3<Alternate<5>>, PB4<Alternate<5>>, PB5<Alternate<5>>)>,
    cs_pin: PC0<Output<PushPull>>,
}

impl SpiPhysicalLayer {
    fn new(
        spi: pac::SPI1,
        pins: (PB3<Alternate<5>>, PB4<Alternate<5>>, PB5<Alternate<5>>),
        cs_pin: PC0<Output<PushPull>>,
        clocks: &stm32f4xx_hal::rcc::Clocks,
    ) -> Self {
        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        
        let spi = Spi::spi1(spi, pins, mode, 1.MHz(), clocks);
        
        Self {
            spi,
            cs_pin,
        }
    }
}

impl PhysicalLayer for SpiPhysicalLayer {
    fn send(&mut self, data: &[u8]) -> Result<(), CommError> {
        self.cs_pin.set_low();
        
        for &byte in data {
            match self.spi.send(byte) {
                Ok(_) => {}
                Err(_) => {
                    self.cs_pin.set_high();
                    return Err(CommError::HardwareError);
                }
            }
        }
        
        self.cs_pin.set_high();
        Ok(())
    }
    
    fn receive(&mut self) -> Result<Option<Vec<u8, 512>>, CommError> {
        // SPI是全双工的，接收通常与发送同时进行
        // 这里简化实现
        Ok(None)
    }
    
    fn is_ready(&self) -> bool {
        true
    }
}

// 数据链路层
struct DataLinkLayer {
    sequence_number: u16,
    expected_sequence: u16,
    retry_count: u8,
    max_retries: u8,
    ack_timeout_ms: u32,
}

impl DataLinkLayer {
    fn new() -> Self {
        Self {
            sequence_number: 0,
            expected_sequence: 0,
            retry_count: 0,
            max_retries: 3,
            ack_timeout_ms: 1000,
        }
    }
    
    fn send_with_ack(&mut self, mut packet: Packet, physical: &mut dyn PhysicalLayer) -> Result<(), CommError> {
        packet.sequence = self.sequence_number;
        packet.calculate_checksum();
        
        let serialized = packet.serialize();
        
        for _ in 0..=self.max_retries {
            physical.send(&serialized)?;
            
            // 等待ACK（简化实现）
            let start_time = get_timestamp();
            while get_timestamp() - start_time < self.ack_timeout_ms {
                if let Ok(Some(data)) = physical.receive() {
                    if let Some(ack_packet) = Packet::deserialize(&data) {
                        if ack_packet.packet_type == PacketType::Ack && 
                           ack_packet.sequence == self.sequence_number {
                            self.sequence_number = self.sequence_number.wrapping_add(1);
                            return Ok(());
                        }
                    }
                }
            }
        }
        
        Err(CommError::TimeoutError)
    }
    
    fn receive_with_ack(&mut self, physical: &mut dyn PhysicalLayer) -> Result<Option<Packet>, CommError> {
        if let Ok(Some(data)) = physical.receive() {
            if let Some(packet) = Packet::deserialize(&data) {
                if packet.sequence == self.expected_sequence {
                    // 发送ACK
                    let mut ack = Packet::new(
                        ProtocolLayer::DataLink,
                        packet.destination,
                        packet.source,
                        PacketType::Ack,
                    );
                    ack.sequence = packet.sequence;
                    ack.calculate_checksum();
                    
                    let ack_data = ack.serialize();
                    let _ = physical.send(&ack_data);
                    
                    self.expected_sequence = self.expected_sequence.wrapping_add(1);
                    return Ok(Some(packet));
                } else {
                    // 序列号不匹配，发送NACK
                    let mut nack = Packet::new(
                        ProtocolLayer::DataLink,
                        packet.destination,
                        packet.source,
                        PacketType::Nack,
                    );
                    nack.sequence = self.expected_sequence;
                    nack.calculate_checksum();
                    
                    let nack_data = nack.serialize();
                    let _ = physical.send(&nack_data);
                }
            }
        }
        
        Ok(None)
    }
}

// 网络层
struct NetworkLayer {
    node_id: u8,
    routing_table: Vec<RoutingEntry, 16>,
}

#[derive(Debug, Clone, Copy)]
struct RoutingEntry {
    destination: u8,
    next_hop: u8,
    metric: u8,
}

impl NetworkLayer {
    fn new(node_id: u8) -> Self {
        Self {
            node_id,
            routing_table: Vec::new(),
        }
    }
    
    fn add_route(&mut self, destination: u8, next_hop: u8, metric: u8) -> Result<(), CommError> {
        let entry = RoutingEntry {
            destination,
            next_hop,
            metric,
        };
        
        self.routing_table.push(entry).map_err(|_| CommError::BufferFull)
    }
    
    fn route_packet(&self, packet: &mut Packet) -> Result<u8, CommError> {
        if packet.destination == self.node_id {
            return Ok(self.node_id); // 本地处理
        }
        
        // 查找路由表
        for entry in &self.routing_table {
            if entry.destination == packet.destination {
                return Ok(entry.next_hop);
            }
        }
        
        Err(CommError::RouteNotFound)
    }
}

// 传输层
struct TransportLayer {
    connections: Vec<Connection, 8>,
    next_connection_id: u16,
}

#[derive(Debug, Clone)]
struct Connection {
    id: u16,
    local_port: u16,
    remote_port: u16,
    remote_address: u8,
    state: ConnectionState,
    send_window: u16,
    receive_window: u16,
    sequence_number: u32,
    ack_number: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ConnectionState {
    Closed,
    Listen,
    SynSent,
    SynReceived,
    Established,
    FinWait1,
    FinWait2,
    CloseWait,
    Closing,
    LastAck,
    TimeWait,
}

impl TransportLayer {
    fn new() -> Self {
        Self {
            connections: Vec::new(),
            next_connection_id: 1,
        }
    }
    
    fn create_connection(&mut self, local_port: u16, remote_address: u8, remote_port: u16) -> Result<u16, CommError> {
        if self.connections.len() >= 8 {
            return Err(CommError::ResourceExhausted);
        }
        
        let connection = Connection {
            id: self.next_connection_id,
            local_port,
            remote_port,
            remote_address,
            state: ConnectionState::Closed,
            send_window: 1024,
            receive_window: 1024,
            sequence_number: 0,
            ack_number: 0,
        };
        
        self.connections.push(connection).map_err(|_| CommError::ResourceExhausted)?;
        
        let id = self.next_connection_id;
        self.next_connection_id = self.next_connection_id.wrapping_add(1);
        Ok(id)
    }
    
    fn send_data(&mut self, connection_id: u16, data: &[u8]) -> Result<(), CommError> {
        // 查找连接
        let connection = self.connections.iter_mut()
            .find(|c| c.id == connection_id)
            .ok_or(CommError::InvalidConnection)?;
        
        if connection.state != ConnectionState::Established {
            return Err(CommError::ConnectionNotEstablished);
        }
        
        // 分段发送数据（如果需要）
        const MAX_SEGMENT_SIZE: usize = 128;
        
        for chunk in data.chunks(MAX_SEGMENT_SIZE) {
            let mut packet = Packet::new(
                ProtocolLayer::Transport,
                0, // 将由网络层填充
                connection.remote_address,
                PacketType::Data,
            );
            
            for &byte in chunk {
                if packet.payload.push(byte).is_err() {
                    break;
                }
            }
            
            packet.sequence = connection.sequence_number as u16;
            connection.sequence_number = connection.sequence_number.wrapping_add(chunk.len() as u32);
            
            // 这里应该将包传递给网络层
        }
        
        Ok(())
    }
}

// 应用层
struct ApplicationLayer {
    services: Vec<Service, 8>,
}

#[derive(Debug, Clone)]
struct Service {
    port: u16,
    name: &'static str,
    handler: fn(&[u8]) -> Vec<u8, 256>,
}

impl ApplicationLayer {
    fn new() -> Self {
        Self {
            services: Vec::new(),
        }
    }
    
    fn register_service(&mut self, port: u16, name: &'static str, handler: fn(&[u8]) -> Vec<u8, 256>) -> Result<(), CommError> {
        let service = Service { port, name, handler };
        self.services.push(service).map_err(|_| CommError::ResourceExhausted)
    }
    
    fn handle_request(&self, port: u16, data: &[u8]) -> Result<Vec<u8, 256>, CommError> {
        for service in &self.services {
            if service.port == port {
                return Ok((service.handler)(data));
            }
        }
        
        Err(CommError::ServiceNotFound)
    }
}

// 完整的协议栈
struct ProtocolStack {
    physical: Box<dyn PhysicalLayer>,
    data_link: DataLinkLayer,
    network: NetworkLayer,
    transport: TransportLayer,
    application: ApplicationLayer,
    packet_queue: Queue<Packet, 32>,
}

impl ProtocolStack {
    fn new(physical: Box<dyn PhysicalLayer>, node_id: u8) -> Self {
        let (packet_queue, _) = Queue::new().split();
        
        Self {
            physical,
            data_link: DataLinkLayer::new(),
            network: NetworkLayer::new(node_id),
            transport: TransportLayer::new(),
            application: ApplicationLayer::new(),
            packet_queue,
        }
    }
    
    fn send_message(&mut self, destination: u8, port: u16, data: &[u8]) -> Result<(), CommError> {
        // 创建应用层数据包
        let mut packet = Packet::new(
            ProtocolLayer::Application,
            self.network.node_id,
            destination,
            PacketType::Data,
        );
        
        // 添加端口信息到载荷
        let _ = packet.payload.push((port >> 8) as u8);
        let _ = packet.payload.push(port as u8);
        
        for &byte in data {
            if packet.payload.push(byte).is_err() {
                break;
            }
        }
        
        // 通过各层处理
        self.process_outgoing_packet(packet)
    }
    
    fn process_outgoing_packet(&mut self, mut packet: Packet) -> Result<(), CommError> {
        // 网络层路由
        let next_hop = self.network.route_packet(&mut packet)?;
        
        // 数据链路层发送
        self.data_link.send_with_ack(packet, self.physical.as_mut())
    }
    
    fn process_incoming_packets(&mut self) -> Result<(), CommError> {
        if let Ok(Some(packet)) = self.data_link.receive_with_ack(self.physical.as_mut()) {
            match packet.layer {
                ProtocolLayer::Application => {
                    self.handle_application_packet(packet)?;
                }
                ProtocolLayer::Transport => {
                    self.handle_transport_packet(packet)?;
                }
                ProtocolLayer::Network => {
                    self.handle_network_packet(packet)?;
                }
                _ => {}
            }
        }
        
        Ok(())
    }
    
    fn handle_application_packet(&mut self, packet: Packet) -> Result<(), CommError> {
        if packet.payload.len() < 2 {
            return Err(CommError::InvalidPacket);
        }
        
        let port = ((packet.payload[0] as u16) << 8) | (packet.payload[1] as u16);
        let data = &packet.payload[2..];
        
        if let Ok(response_data) = self.application.handle_request(port, data) {
            // 发送响应
            self.send_message(packet.source, port, &response_data)?;
        }
        
        Ok(())
    }
    
    fn handle_transport_packet(&mut self, _packet: Packet) -> Result<(), CommError> {
        // 处理传输层包
        Ok(())
    }
    
    fn handle_network_packet(&mut self, _packet: Packet) -> Result<(), CommError> {
        // 处理网络层包
        Ok(())
    }
}

// 错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
enum CommError {
    HardwareError,
    TimeoutError,
    BufferFull,
    InvalidPacket,
    ChecksumError,
    RouteNotFound,
    ResourceExhausted,
    InvalidConnection,
    ConnectionNotEstablished,
    ServiceNotFound,
    Busy,
}

// 全局变量
static mut PROTOCOL_STACK: Option<ProtocolStack> = None;
static mut TIMESTAMP_COUNTER: u32 = 0;

// 示例服务处理函数
fn echo_service(data: &[u8]) -> Vec<u8, 256> {
    let mut response = Vec::new();
    let _ = response.extend_from_slice(b"Echo: ");
    for &byte in data {
        if response.push(byte).is_err() {
            break;
        }
    }
    response
}

fn time_service(_data: &[u8]) -> Vec<u8, 256> {
    let mut response = Vec::new();
    let timestamp = get_timestamp();
    let time_str = format_no_std::show(&timestamp, format_no_std::DisplayStyle::Hexadecimal).unwrap();
    let _ = response.extend_from_slice(time_str.as_bytes());
    response
}

fn get_timestamp() -> u32 {
    unsafe { TIMESTAMP_COUNTER }
}

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置UART引脚
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    
    // 创建UART物理层
    let uart_physical = UartPhysicalLayer::new(dp.USART1, (tx_pin, rx_pin), &clocks);
    
    // 创建协议栈
    let mut stack = ProtocolStack::new(Box::new(uart_physical), 1); // 节点ID为1
    
    // 注册服务
    let _ = stack.application.register_service(80, "echo", echo_service);
    let _ = stack.application.register_service(123, "time", time_service);
    
    // 添加路由
    let _ = stack.network.add_route(2, 2, 1); // 到节点2的路由
    let _ = stack.network.add_route(3, 2, 2); // 到节点3通过节点2
    
    unsafe {
        PROTOCOL_STACK = Some(stack);
    }
    
    // 配置定时器用于时间戳
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(1.millis()).unwrap();
    timer.listen(Event::Update);
    
    // 启用定时器中断
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }
    
    let mut message_counter = 0u32;
    
    loop {
        // 处理接收到的包
        unsafe {
            if let Some(ref mut stack) = PROTOCOL_STACK {
                let _ = stack.process_incoming_packets();
            }
        }
        
        // 定期发送测试消息
        message_counter += 1;
        if message_counter >= 100000 {
            unsafe {
                if let Some(ref mut stack) = PROTOCOL_STACK {
                    let test_data = b"Hello from node 1";
                    let _ = stack.send_message(2, 80, test_data);
                }
            }
            message_counter = 0;
        }
        
        // 短暂延时
        cortex_m::asm::delay(1000);
    }
}

// 定时器中断处理
#[interrupt]
fn TIM2() {
    unsafe {
        let timer = &*pac::TIM2::ptr();
        timer.sr.modify(|_, w| w.uif().clear_bit());
        
        // 更新时间戳
        TIMESTAMP_COUNTER = TIMESTAMP_COUNTER.wrapping_add(1);
    }
}