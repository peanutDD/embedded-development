#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::interrupt::{free, Mutex};
use core::cell::RefCell;
use heapless::{Vec, String, pool::{Pool, Node}};
use nb::block;

// HAL imports (adjust based on your target MCU)
use stm32f4xx_hal::{
    prelude::*,
    pac::{self, interrupt, USART1, USART2, TIM2},
    serial::{config::Config, Serial, Tx, Rx},
    gpio::{gpioa::*, gpiob::*, Alternate, Output, PushPull},
    timer::{Timer, Event},
    dma::{StreamsTuple, Transfer, Stream0, Stream1, Channel4, PeripheralToMemory, MemoryToPeripheral},
    rcc::Clocks,
};

// 系统配置常量
const BUFFER_SIZE: usize = 512;
const MAX_PACKET_SIZE: usize = 256;
const BRIDGE_TIMEOUT_MS: u32 = 100;
const WIRELESS_BAUD_RATE: u32 = 115200;
const SERIAL_BAUD_RATE: u32 = 9600;
const MAX_RETRY_COUNT: u8 = 3;
const HEARTBEAT_INTERVAL_MS: u32 = 5000;

// 类型定义
type SerialTx = Tx<USART1>;
type SerialRx = Rx<USART1>;
type WirelessTx = Tx<USART2>;
type WirelessRx = Rx<USART2>;
type BridgeTimer = Timer<TIM2>;

// DMA传输类型
type DmaTransferRx = Transfer<Stream0<pac::DMA2>, Channel4, SerialRx, PeripheralToMemory, &'static mut [u8]>;
type DmaTransferTx = Transfer<Stream1<pac::DMA2>, Channel4, SerialTx, MemoryToPeripheral, &'static mut [u8]>;

// 全局变量
static SERIAL_TX: Mutex<RefCell<Option<SerialTx>>> = Mutex::new(RefCell::new(None));
static SERIAL_RX: Mutex<RefCell<Option<SerialRx>>> = Mutex::new(RefCell::new(None));
static WIRELESS_TX: Mutex<RefCell<Option<WirelessTx>>> = Mutex::new(RefCell::new(None));
static WIRELESS_RX: Mutex<RefCell<Option<WirelessRx>>> = Mutex::new(RefCell::new(None));
static BRIDGE_TIMER: Mutex<RefCell<Option<BridgeTimer>>> = Mutex::new(RefCell::new(None));

// 数据缓冲区
static mut SERIAL_RX_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut WIRELESS_RX_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut SERIAL_TX_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut WIRELESS_TX_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

// 系统状态
static mut BRIDGE_STATE: BridgeState = BridgeState::new();
static mut PACKET_POOL: [Node<Packet>; 16] = [Node::new(); 16];
static PACKET_POOL_STORAGE: Pool<Packet> = Pool::new();

#[entry]
fn main() -> ! {
    // 初始化系统
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(25.mhz())
        .sysclk(168.mhz())
        .pclk1(42.mhz())
        .pclk2(84.mhz())
        .freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // 配置串口1 (Serial)
    let serial_tx = gpioa.pa9.into_alternate_af7();
    let serial_rx = gpioa.pa10.into_alternate_af7();
    let serial = Serial::usart1(
        dp.USART1,
        (serial_tx, serial_rx),
        Config::default().baudrate(SERIAL_BAUD_RATE.bps()),
        clocks,
    ).unwrap();
    let (serial_tx, serial_rx) = serial.split();

    // 配置串口2 (Wireless)
    let wireless_tx = gpioa.pa2.into_alternate_af7();
    let wireless_rx = gpioa.pa3.into_alternate_af7();
    let wireless = Serial::usart2(
        dp.USART2,
        (wireless_tx, wireless_rx),
        Config::default().baudrate(WIRELESS_BAUD_RATE.bps()),
        clocks,
    ).unwrap();
    let (wireless_tx, wireless_rx) = wireless.split();

    // 配置定时器
    let mut timer = Timer::tim2(dp.TIM2, 1.khz(), clocks);
    timer.listen(Event::TimeOut);

    // 配置DMA
    let dma = StreamsTuple::new(dp.DMA2);

    // 存储全局变量
    free(|cs| {
        SERIAL_TX.borrow(cs).replace(Some(serial_tx));
        SERIAL_RX.borrow(cs).replace(Some(serial_rx));
        WIRELESS_TX.borrow(cs).replace(Some(wireless_tx));
        WIRELESS_RX.borrow(cs).replace(Some(wireless_rx));
        BRIDGE_TIMER.borrow(cs).replace(Some(timer));
    });

    // 初始化数据包池
    unsafe {
        PACKET_POOL_STORAGE.grow(&mut PACKET_POOL);
    }

    // 启用中断
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::USART1);
        pac::NVIC::unmask(pac::Interrupt::USART2);
        pac::NVIC::unmask(pac::Interrupt::TIM2);
    }

    // 初始化桥接器
    let mut bridge = WirelessBridge::new();
    bridge.initialize();

    // 主循环
    loop {
        // 处理串口到无线的数据传输
        bridge.process_serial_to_wireless();
        
        // 处理无线到串口的数据传输
        bridge.process_wireless_to_serial();
        
        // 处理系统维护任务
        bridge.process_maintenance();
        
        // 更新统计信息
        bridge.update_statistics();
        
        // 检查系统健康状态
        bridge.check_system_health();
        
        // 低功耗等待
        cortex_m::asm::wfi();
    }
}

// 无线桥接器结构体
struct WirelessBridge {
    serial_buffer: CircularBuffer,
    wireless_buffer: CircularBuffer,
    packet_processor: PacketProcessor,
    protocol_handler: ProtocolHandler,
    statistics: BridgeStatistics,
    config: BridgeConfig,
    last_heartbeat: u32,
    retry_count: u8,
}

impl WirelessBridge {
    fn new() -> Self {
        Self {
            serial_buffer: CircularBuffer::new(),
            wireless_buffer: CircularBuffer::new(),
            packet_processor: PacketProcessor::new(),
            protocol_handler: ProtocolHandler::new(),
            statistics: BridgeStatistics::new(),
            config: BridgeConfig::default(),
            last_heartbeat: 0,
            retry_count: 0,
        }
    }

    fn initialize(&mut self) {
        // 初始化缓冲区
        self.serial_buffer.clear();
        self.wireless_buffer.clear();
        
        // 重置统计信息
        self.statistics.reset();
        
        // 发送初始化命令
        self.send_initialization_sequence();
        
        // 启动心跳定时器
        self.last_heartbeat = get_system_time();
    }

    fn process_serial_to_wireless(&mut self) {
        // 从串口读取数据
        if let Some(data) = self.read_serial_data() {
            // 处理数据包
            if let Ok(packet) = self.packet_processor.process_incoming_data(&data) {
                // 应用协议处理
                if let Ok(processed_packet) = self.protocol_handler.process_outgoing(&packet) {
                    // 发送到无线模块
                    if self.send_wireless_data(&processed_packet).is_ok() {
                        self.statistics.serial_to_wireless_packets += 1;
                        self.statistics.serial_to_wireless_bytes += processed_packet.len() as u32;
                    } else {
                        self.statistics.transmission_errors += 1;
                        self.handle_transmission_error(&processed_packet);
                    }
                }
            }
        }
    }

    fn process_wireless_to_serial(&mut self) {
        // 从无线模块读取数据
        if let Some(data) = self.read_wireless_data() {
            // 处理数据包
            if let Ok(packet) = self.packet_processor.process_incoming_data(&data) {
                // 应用协议处理
                if let Ok(processed_packet) = self.protocol_handler.process_incoming(&packet) {
                    // 发送到串口
                    if self.send_serial_data(&processed_packet).is_ok() {
                        self.statistics.wireless_to_serial_packets += 1;
                        self.statistics.wireless_to_serial_bytes += processed_packet.len() as u32;
                    } else {
                        self.statistics.transmission_errors += 1;
                        self.handle_transmission_error(&processed_packet);
                    }
                }
            }
        }
    }

    fn process_maintenance(&mut self) {
        let current_time = get_system_time();
        
        // 检查心跳
        if current_time - self.last_heartbeat > HEARTBEAT_INTERVAL_MS {
            self.send_heartbeat();
            self.last_heartbeat = current_time;
        }
        
        // 清理过期数据
        self.cleanup_expired_data();
        
        // 优化缓冲区
        self.optimize_buffers();
    }

    fn read_serial_data(&mut self) -> Option<Vec<u8, BUFFER_SIZE>> {
        free(|cs| {
            if let Some(ref mut rx) = SERIAL_RX.borrow(cs).borrow_mut().as_mut() {
                let mut data = Vec::new();
                while let Ok(byte) = rx.read() {
                    if data.push(byte).is_err() {
                        break;
                    }
                }
                if !data.is_empty() {
                    Some(data)
                } else {
                    None
                }
            } else {
                None
            }
        })
    }

    fn read_wireless_data(&mut self) -> Option<Vec<u8, BUFFER_SIZE>> {
        free(|cs| {
            if let Some(ref mut rx) = WIRELESS_RX.borrow(cs).borrow_mut().as_mut() {
                let mut data = Vec::new();
                while let Ok(byte) = rx.read() {
                    if data.push(byte).is_err() {
                        break;
                    }
                }
                if !data.is_empty() {
                    Some(data)
                } else {
                    None
                }
            } else {
                None
            }
        })
    }

    fn send_serial_data(&mut self, data: &[u8]) -> Result<(), BridgeError> {
        free(|cs| {
            if let Some(ref mut tx) = SERIAL_TX.borrow(cs).borrow_mut().as_mut() {
                for &byte in data {
                    block!(tx.write(byte)).map_err(|_| BridgeError::TransmissionFailed)?;
                }
                Ok(())
            } else {
                Err(BridgeError::HardwareNotAvailable)
            }
        })
    }

    fn send_wireless_data(&mut self, data: &[u8]) -> Result<(), BridgeError> {
        free(|cs| {
            if let Some(ref mut tx) = WIRELESS_TX.borrow(cs).borrow_mut().as_mut() {
                for &byte in data {
                    block!(tx.write(byte)).map_err(|_| BridgeError::TransmissionFailed)?;
                }
                Ok(())
            } else {
                Err(BridgeError::HardwareNotAvailable)
            }
        })
    }

    fn send_heartbeat(&mut self) {
        let heartbeat_packet = self.protocol_handler.create_heartbeat_packet();
        let _ = self.send_wireless_data(&heartbeat_packet);
        self.statistics.heartbeat_sent += 1;
    }

    fn send_initialization_sequence(&mut self) {
        let init_packet = self.protocol_handler.create_init_packet();
        let _ = self.send_wireless_data(&init_packet);
    }

    fn handle_transmission_error(&mut self, data: &[u8]) {
        if self.retry_count < MAX_RETRY_COUNT {
            // 重试传输
            self.retry_count += 1;
            // 这里可以实现重试逻辑
        } else {
            // 记录错误并丢弃数据
            self.statistics.dropped_packets += 1;
            self.retry_count = 0;
        }
    }

    fn cleanup_expired_data(&mut self) {
        // 清理过期的缓冲区数据
        self.serial_buffer.cleanup_expired();
        self.wireless_buffer.cleanup_expired();
    }

    fn optimize_buffers(&mut self) {
        // 优化缓冲区使用
        self.serial_buffer.optimize();
        self.wireless_buffer.optimize();
    }

    fn update_statistics(&mut self) {
        self.statistics.uptime = get_system_time();
        self.statistics.buffer_usage = self.calculate_buffer_usage();
    }

    fn calculate_buffer_usage(&self) -> u8 {
        let serial_usage = self.serial_buffer.usage_percentage();
        let wireless_usage = self.wireless_buffer.usage_percentage();
        (serial_usage + wireless_usage) / 2
    }

    fn check_system_health(&mut self) -> SystemHealth {
        let mut health = SystemHealth::Good;
        
        // 检查缓冲区使用率
        if self.calculate_buffer_usage() > 80 {
            health = SystemHealth::Warning;
        }
        
        // 检查错误率
        let total_packets = self.statistics.serial_to_wireless_packets + 
                           self.statistics.wireless_to_serial_packets;
        if total_packets > 0 {
            let error_rate = (self.statistics.transmission_errors * 100) / total_packets;
            if error_rate > 5 {
                health = SystemHealth::Critical;
            }
        }
        
        health
    }
}

// 数据包处理器
struct PacketProcessor {
    current_packet: Option<Packet>,
    packet_buffer: Vec<u8, MAX_PACKET_SIZE>,
    state: PacketState,
}

impl PacketProcessor {
    fn new() -> Self {
        Self {
            current_packet: None,
            packet_buffer: Vec::new(),
            state: PacketState::WaitingForHeader,
        }
    }

    fn process_incoming_data(&mut self, data: &[u8]) -> Result<Packet, PacketError> {
        for &byte in data {
            match self.state {
                PacketState::WaitingForHeader => {
                    if byte == PACKET_START_MARKER {
                        self.packet_buffer.clear();
                        self.packet_buffer.push(byte).ok();
                        self.state = PacketState::ReadingHeader;
                    }
                }
                PacketState::ReadingHeader => {
                    self.packet_buffer.push(byte).ok();
                    if self.packet_buffer.len() >= PACKET_HEADER_SIZE {
                        self.state = PacketState::ReadingPayload;
                    }
                }
                PacketState::ReadingPayload => {
                    self.packet_buffer.push(byte).ok();
                    if self.is_packet_complete() {
                        let packet = self.build_packet()?;
                        self.state = PacketState::WaitingForHeader;
                        return Ok(packet);
                    }
                }
            }
        }
        Err(PacketError::IncompletePacket)
    }

    fn is_packet_complete(&self) -> bool {
        if self.packet_buffer.len() < PACKET_HEADER_SIZE {
            return false;
        }
        
        let payload_length = self.packet_buffer[2] as usize;
        self.packet_buffer.len() >= PACKET_HEADER_SIZE + payload_length + PACKET_CHECKSUM_SIZE
    }

    fn build_packet(&self) -> Result<Packet, PacketError> {
        if self.packet_buffer.len() < PACKET_MIN_SIZE {
            return Err(PacketError::InvalidSize);
        }

        let packet_type = PacketType::from_byte(self.packet_buffer[1])?;
        let payload_length = self.packet_buffer[2] as usize;
        
        if payload_length > MAX_PACKET_SIZE - PACKET_HEADER_SIZE - PACKET_CHECKSUM_SIZE {
            return Err(PacketError::PayloadTooLarge);
        }

        let payload_start = PACKET_HEADER_SIZE;
        let payload_end = payload_start + payload_length;
        let payload = &self.packet_buffer[payload_start..payload_end];

        let checksum_pos = payload_end;
        let expected_checksum = self.packet_buffer[checksum_pos];
        let calculated_checksum = self.calculate_checksum(&self.packet_buffer[..checksum_pos]);

        if expected_checksum != calculated_checksum {
            return Err(PacketError::ChecksumMismatch);
        }

        Ok(Packet {
            packet_type,
            payload: payload.to_vec(),
            timestamp: get_system_time(),
        })
    }

    fn calculate_checksum(&self, data: &[u8]) -> u8 {
        data.iter().fold(0u8, |acc, &x| acc.wrapping_add(x))
    }
}

// 协议处理器
struct ProtocolHandler {
    sequence_number: u16,
    session_id: u32,
}

impl ProtocolHandler {
    fn new() -> Self {
        Self {
            sequence_number: 0,
            session_id: generate_session_id(),
        }
    }

    fn process_outgoing(&mut self, packet: &Packet) -> Result<Vec<u8, BUFFER_SIZE>, ProtocolError> {
        let mut output = Vec::new();
        
        // 添加协议头
        output.push(PROTOCOL_VERSION).ok();
        output.push(packet.packet_type.to_byte()).ok();
        output.extend_from_slice(&self.sequence_number.to_le_bytes()).ok();
        output.extend_from_slice(&self.session_id.to_le_bytes()).ok();
        output.extend_from_slice(&(packet.payload.len() as u16).to_le_bytes()).ok();
        
        // 添加载荷
        output.extend_from_slice(&packet.payload).ok();
        
        // 添加校验和
        let checksum = self.calculate_protocol_checksum(&output);
        output.push(checksum).ok();
        
        self.sequence_number = self.sequence_number.wrapping_add(1);
        
        Ok(output)
    }

    fn process_incoming(&mut self, packet: &Packet) -> Result<Vec<u8, BUFFER_SIZE>, ProtocolError> {
        // 验证协议版本和会话ID
        if !self.validate_protocol_header(&packet.payload) {
            return Err(ProtocolError::InvalidHeader);
        }
        
        // 提取实际数据
        let data_start = PROTOCOL_HEADER_SIZE;
        let data = &packet.payload[data_start..];
        
        let mut output = Vec::new();
        output.extend_from_slice(data).ok();
        
        Ok(output)
    }

    fn create_heartbeat_packet(&mut self) -> Vec<u8, BUFFER_SIZE> {
        let mut packet = Vec::new();
        packet.push(PACKET_START_MARKER).ok();
        packet.push(PacketType::Heartbeat.to_byte()).ok();
        packet.push(4).ok(); // payload length
        packet.extend_from_slice(&get_system_time().to_le_bytes()).ok();
        
        let checksum = self.calculate_protocol_checksum(&packet);
        packet.push(checksum).ok();
        
        packet
    }

    fn create_init_packet(&mut self) -> Vec<u8, BUFFER_SIZE> {
        let mut packet = Vec::new();
        packet.push(PACKET_START_MARKER).ok();
        packet.push(PacketType::Initialize.to_byte()).ok();
        packet.push(8).ok(); // payload length
        packet.extend_from_slice(&self.session_id.to_le_bytes()).ok();
        packet.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes()).ok();
        
        let checksum = self.calculate_protocol_checksum(&packet);
        packet.push(checksum).ok();
        
        packet
    }

    fn validate_protocol_header(&self, data: &[u8]) -> bool {
        if data.len() < PROTOCOL_HEADER_SIZE {
            return false;
        }
        
        let version = data[0];
        version == PROTOCOL_VERSION
    }

    fn calculate_protocol_checksum(&self, data: &[u8]) -> u8 {
        data.iter().fold(0u8, |acc, &x| acc.wrapping_add(x))
    }
}

// 循环缓冲区
struct CircularBuffer {
    buffer: [u8; BUFFER_SIZE],
    head: usize,
    tail: usize,
    count: usize,
    last_access: u32,
}

impl CircularBuffer {
    fn new() -> Self {
        Self {
            buffer: [0; BUFFER_SIZE],
            head: 0,
            tail: 0,
            count: 0,
            last_access: get_system_time(),
        }
    }

    fn push(&mut self, data: &[u8]) -> Result<(), BufferError> {
        if data.len() > self.available_space() {
            return Err(BufferError::BufferFull);
        }

        for &byte in data {
            self.buffer[self.head] = byte;
            self.head = (self.head + 1) % BUFFER_SIZE;
            self.count += 1;
        }
        
        self.last_access = get_system_time();
        Ok(())
    }

    fn pop(&mut self, buffer: &mut [u8]) -> usize {
        let bytes_to_read = core::cmp::min(buffer.len(), self.count);
        
        for i in 0..bytes_to_read {
            buffer[i] = self.buffer[self.tail];
            self.tail = (self.tail + 1) % BUFFER_SIZE;
            self.count -= 1;
        }
        
        self.last_access = get_system_time();
        bytes_to_read
    }

    fn available_space(&self) -> usize {
        BUFFER_SIZE - self.count
    }

    fn usage_percentage(&self) -> u8 {
        ((self.count * 100) / BUFFER_SIZE) as u8
    }

    fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.count = 0;
        self.last_access = get_system_time();
    }

    fn cleanup_expired(&mut self) {
        let current_time = get_system_time();
        if current_time - self.last_access > BRIDGE_TIMEOUT_MS * 10 {
            self.clear();
        }
    }

    fn optimize(&mut self) {
        // 如果缓冲区使用率过高，可以实现一些优化策略
        if self.usage_percentage() > 90 {
            // 丢弃最旧的数据
            let bytes_to_drop = self.count / 4;
            self.tail = (self.tail + bytes_to_drop) % BUFFER_SIZE;
            self.count -= bytes_to_drop;
        }
    }
}

// 中断处理函数
#[interrupt]
fn USART1() {
    // 串口1接收中断
    free(|cs| {
        if let Some(ref mut rx) = SERIAL_RX.borrow(cs).borrow_mut().as_mut() {
            if let Ok(_byte) = rx.read() {
                // 数据已在主循环中处理
                unsafe {
                    BRIDGE_STATE.serial_rx_interrupts += 1;
                }
            }
        }
    });
}

#[interrupt]
fn USART2() {
    // 串口2接收中断
    free(|cs| {
        if let Some(ref mut rx) = WIRELESS_RX.borrow(cs).borrow_mut().as_mut() {
            if let Ok(_byte) = rx.read() {
                // 数据已在主循环中处理
                unsafe {
                    BRIDGE_STATE.wireless_rx_interrupts += 1;
                }
            }
        }
    });
}

#[interrupt]
fn TIM2() {
    // 定时器中断 - 用于超时检测
    free(|cs| {
        if let Some(ref mut timer) = BRIDGE_TIMER.borrow(cs).borrow_mut().as_mut() {
            timer.clear_interrupt(Event::TimeOut);
            unsafe {
                BRIDGE_STATE.timer_interrupts += 1;
            }
        }
    });
}

// 数据结构定义
#[derive(Clone)]
struct Packet {
    packet_type: PacketType,
    payload: Vec<u8, MAX_PACKET_SIZE>,
    timestamp: u32,
}

#[derive(Clone, Copy)]
enum PacketType {
    Data,
    Control,
    Heartbeat,
    Initialize,
    Acknowledge,
}

impl PacketType {
    fn from_byte(byte: u8) -> Result<Self, PacketError> {
        match byte {
            0x01 => Ok(PacketType::Data),
            0x02 => Ok(PacketType::Control),
            0x03 => Ok(PacketType::Heartbeat),
            0x04 => Ok(PacketType::Initialize),
            0x05 => Ok(PacketType::Acknowledge),
            _ => Err(PacketError::InvalidType),
        }
    }

    fn to_byte(self) -> u8 {
        match self {
            PacketType::Data => 0x01,
            PacketType::Control => 0x02,
            PacketType::Heartbeat => 0x03,
            PacketType::Initialize => 0x04,
            PacketType::Acknowledge => 0x05,
        }
    }
}

#[derive(Clone, Copy)]
enum PacketState {
    WaitingForHeader,
    ReadingHeader,
    ReadingPayload,
}

#[derive(Clone, Copy)]
enum SystemHealth {
    Good,
    Warning,
    Critical,
}

// 错误类型定义
#[derive(Debug)]
enum BridgeError {
    TransmissionFailed,
    HardwareNotAvailable,
    BufferOverflow,
    ProtocolError,
}

#[derive(Debug)]
enum PacketError {
    InvalidType,
    InvalidSize,
    PayloadTooLarge,
    ChecksumMismatch,
    IncompletePacket,
}

#[derive(Debug)]
enum ProtocolError {
    InvalidHeader,
    UnsupportedVersion,
    SessionMismatch,
}

#[derive(Debug)]
enum BufferError {
    BufferFull,
    BufferEmpty,
    InvalidSize,
}

// 统计信息结构体
struct BridgeStatistics {
    serial_to_wireless_packets: u32,
    wireless_to_serial_packets: u32,
    serial_to_wireless_bytes: u32,
    wireless_to_serial_bytes: u32,
    transmission_errors: u32,
    dropped_packets: u32,
    heartbeat_sent: u32,
    heartbeat_received: u32,
    uptime: u32,
    buffer_usage: u8,
}

impl BridgeStatistics {
    fn new() -> Self {
        Self {
            serial_to_wireless_packets: 0,
            wireless_to_serial_packets: 0,
            serial_to_wireless_bytes: 0,
            wireless_to_serial_bytes: 0,
            transmission_errors: 0,
            dropped_packets: 0,
            heartbeat_sent: 0,
            heartbeat_received: 0,
            uptime: 0,
            buffer_usage: 0,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn get_throughput(&self) -> (u32, u32) {
        let uptime_seconds = self.uptime / 1000;
        if uptime_seconds > 0 {
            (
                self.serial_to_wireless_bytes / uptime_seconds,
                self.wireless_to_serial_bytes / uptime_seconds,
            )
        } else {
            (0, 0)
        }
    }

    fn get_error_rate(&self) -> u8 {
        let total_packets = self.serial_to_wireless_packets + self.wireless_to_serial_packets;
        if total_packets > 0 {
            ((self.transmission_errors * 100) / total_packets) as u8
        } else {
            0
        }
    }
}

// 桥接配置
struct BridgeConfig {
    serial_baud_rate: u32,
    wireless_baud_rate: u32,
    buffer_size: usize,
    timeout_ms: u32,
    max_retry_count: u8,
    heartbeat_interval_ms: u32,
    enable_flow_control: bool,
    enable_error_correction: bool,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            serial_baud_rate: SERIAL_BAUD_RATE,
            wireless_baud_rate: WIRELESS_BAUD_RATE,
            buffer_size: BUFFER_SIZE,
            timeout_ms: BRIDGE_TIMEOUT_MS,
            max_retry_count: MAX_RETRY_COUNT,
            heartbeat_interval_ms: HEARTBEAT_INTERVAL_MS,
            enable_flow_control: true,
            enable_error_correction: true,
        }
    }
}

// 系统状态
struct BridgeState {
    serial_rx_interrupts: u32,
    wireless_rx_interrupts: u32,
    timer_interrupts: u32,
    last_activity: u32,
    connection_status: ConnectionStatus,
}

impl BridgeState {
    const fn new() -> Self {
        Self {
            serial_rx_interrupts: 0,
            wireless_rx_interrupts: 0,
            timer_interrupts: 0,
            last_activity: 0,
            connection_status: ConnectionStatus::Disconnected,
        }
    }
}

#[derive(Clone, Copy)]
enum ConnectionStatus {
    Connected,
    Disconnected,
    Connecting,
    Error,
}

// 协议常量
const PACKET_START_MARKER: u8 = 0xAA;
const PACKET_HEADER_SIZE: usize = 3;
const PACKET_CHECKSUM_SIZE: usize = 1;
const PACKET_MIN_SIZE: usize = PACKET_HEADER_SIZE + PACKET_CHECKSUM_SIZE;
const PROTOCOL_VERSION: u8 = 0x01;
const PROTOCOL_HEADER_SIZE: usize = 12;

// 辅助函数
fn get_system_time() -> u32 {
    // 这里应该返回系统时间戳（毫秒）
    // 实际实现中可能需要使用系统定时器
    0
}

fn generate_session_id() -> u32 {
    // 生成唯一的会话ID
    // 实际实现中可能使用硬件随机数生成器或基于时间的算法
    0x12345678
}