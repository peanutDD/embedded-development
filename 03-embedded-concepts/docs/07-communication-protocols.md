# 通信协议基础

## 概述

通信协议是嵌入式系统与外界交互的桥梁，涵盖了从简单的串行通信到复杂的网络协议。理解各种通信协议的原理、特点和应用场景，对于设计可靠的嵌入式系统至关重要。

## 学习目标

完成本章节后，你将掌握：
- 串行和并行通信的基本原理
- 常见通信协议的特点和应用
- 协议栈的设计和实现
- 通信可靠性和错误处理
- 网络通信和物联网协议

## 1. 通信基础理论

### 1.1 通信系统模型

```rust
// 通信系统基本组件
pub struct CommunicationSystem {
    transmitter: Transmitter,
    receiver: Receiver,
    channel: Channel,
    protocol_stack: ProtocolStack,
}

// 发送器
pub struct Transmitter {
    encoder: Encoder,
    modulator: Modulator,
    power_amplifier: PowerAmplifier,
}

// 接收器
pub struct Receiver {
    demodulator: Demodulator,
    decoder: Decoder,
    error_detector: ErrorDetector,
}

// 通信信道
pub struct Channel {
    channel_type: ChannelType,
    bandwidth: u32,
    noise_level: f32,
    attenuation: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum ChannelType {
    Wired,      // 有线信道
    Wireless,   // 无线信道
    Optical,    // 光纤信道
    Acoustic,   // 声学信道
}

// 编码器
pub trait Encoder {
    fn encode(&self, data: &[u8]) -> Vec<u8, 256>;
    fn get_overhead(&self) -> usize;
}

// 解码器
pub trait Decoder {
    fn decode(&self, encoded_data: &[u8]) -> Result<Vec<u8, 256>, DecodeError>;
    fn detect_errors(&self, data: &[u8]) -> bool;
}

#[derive(Debug, Clone, Copy)]
pub enum DecodeError {
    ChecksumError,
    FormatError,
    BufferOverflow,
    TimeoutError,
}

// 简单的曼彻斯特编码器
pub struct ManchesterEncoder;

impl Encoder for ManchesterEncoder {
    fn encode(&self, data: &[u8]) -> Vec<u8, 256> {
        let mut encoded = Vec::new();
        
        for &byte in data {
            for bit in 0..8 {
                let bit_value = (byte >> (7 - bit)) & 1;
                // 曼彻斯特编码：0 -> 01, 1 -> 10
                if bit_value == 0 {
                    let _ = encoded.push(0x01);
                } else {
                    let _ = encoded.push(0x10);
                }
            }
        }
        
        encoded
    }
    
    fn get_overhead(&self) -> usize {
        100 // 100% 开销
    }
}

// 曼彻斯特解码器
pub struct ManchesterDecoder;

impl Decoder for ManchesterDecoder {
    fn decode(&self, encoded_data: &[u8]) -> Result<Vec<u8, 256>, DecodeError> {
        if encoded_data.len() % 8 != 0 {
            return Err(DecodeError::FormatError);
        }
        
        let mut decoded = Vec::new();
        
        for chunk in encoded_data.chunks(8) {
            let mut byte = 0u8;
            
            for (i, &encoded_bit) in chunk.iter().enumerate() {
                let bit_value = match encoded_bit {
                    0x01 => 0,
                    0x10 => 1,
                    _ => return Err(DecodeError::FormatError),
                };
                
                byte |= bit_value << (7 - i);
            }
            
            if decoded.push(byte).is_err() {
                return Err(DecodeError::BufferOverflow);
            }
        }
        
        Ok(decoded)
    }
    
    fn detect_errors(&self, data: &[u8]) -> bool {
        // 检查曼彻斯特编码的有效性
        for &byte in data {
            if byte != 0x01 && byte != 0x10 {
                return true; // 发现错误
            }
        }
        false
    }
}
```

### 1.2 通信参数和性能指标

```rust
// 通信性能指标
#[derive(Debug, Clone)]
pub struct CommunicationMetrics {
    pub bandwidth: u32,           // 带宽 (bps)
    pub throughput: u32,          // 吞吐量 (bps)
    pub latency: u32,             // 延迟 (ms)
    pub jitter: u32,              // 抖动 (ms)
    pub packet_loss_rate: f32,    // 丢包率 (%)
    pub bit_error_rate: f32,      // 误码率
    pub signal_to_noise_ratio: f32, // 信噪比 (dB)
}

impl CommunicationMetrics {
    pub fn new() -> Self {
        Self {
            bandwidth: 0,
            throughput: 0,
            latency: 0,
            jitter: 0,
            packet_loss_rate: 0.0,
            bit_error_rate: 0.0,
            signal_to_noise_ratio: 0.0,
        }
    }
    
    pub fn calculate_efficiency(&self) -> f32 {
        if self.bandwidth == 0 {
            0.0
        } else {
            self.throughput as f32 / self.bandwidth as f32
        }
    }
    
    pub fn is_acceptable(&self, requirements: &QoSRequirements) -> bool {
        self.latency <= requirements.max_latency &&
        self.jitter <= requirements.max_jitter &&
        self.packet_loss_rate <= requirements.max_packet_loss &&
        self.bit_error_rate <= requirements.max_bit_error_rate
    }
}

// 服务质量要求
#[derive(Debug, Clone)]
pub struct QoSRequirements {
    pub max_latency: u32,
    pub max_jitter: u32,
    pub max_packet_loss: f32,
    pub max_bit_error_rate: f32,
    pub min_throughput: u32,
}

// 通信模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommunicationMode {
    Simplex,    // 单工
    HalfDuplex, // 半双工
    FullDuplex, // 全双工
}

// 同步方式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SynchronizationType {
    Synchronous,   // 同步
    Asynchronous,  // 异步
}
```

## 2. 串行通信协议

### 2.1 UART协议实现

```rust
use heapless::{Vec, spsc::{Producer, Consumer, Queue}};

// UART配置
#[derive(Debug, Clone, Copy)]
pub struct UartConfig {
    pub baud_rate: u32,
    pub data_bits: DataBits,
    pub parity: Parity,
    pub stop_bits: StopBits,
    pub flow_control: FlowControl,
}

#[derive(Debug, Clone, Copy)]
pub enum DataBits {
    Five = 5,
    Six = 6,
    Seven = 7,
    Eight = 8,
}

#[derive(Debug, Clone, Copy)]
pub enum Parity {
    None,
    Even,
    Odd,
    Mark,
    Space,
}

#[derive(Debug, Clone, Copy)]
pub enum StopBits {
    One,
    OneAndHalf,
    Two,
}

#[derive(Debug, Clone, Copy)]
pub enum FlowControl {
    None,
    RtsCts,
    XonXoff,
}

// UART驱动程序
pub struct UartDriver {
    config: UartConfig,
    tx_buffer: Vec<u8, 256>,
    rx_buffer: Vec<u8, 256>,
    tx_queue: Queue<u8, 256>,
    rx_queue: Queue<u8, 256>,
    state: UartState,
    statistics: UartStatistics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartState {
    Idle,
    Transmitting,
    Receiving,
    Error,
}

#[derive(Debug, Clone)]
pub struct UartStatistics {
    pub bytes_transmitted: u32,
    pub bytes_received: u32,
    pub transmission_errors: u32,
    pub reception_errors: u32,
    pub parity_errors: u32,
    pub framing_errors: u32,
    pub overrun_errors: u32,
}

impl UartDriver {
    pub fn new(config: UartConfig) -> Self {
        let (tx_queue, _) = Queue::new().split();
        let (rx_queue, _) = Queue::new().split();
        
        Self {
            config,
            tx_buffer: Vec::new(),
            rx_buffer: Vec::new(),
            tx_queue,
            rx_queue,
            state: UartState::Idle,
            statistics: UartStatistics {
                bytes_transmitted: 0,
                bytes_received: 0,
                transmission_errors: 0,
                reception_errors: 0,
                parity_errors: 0,
                framing_errors: 0,
                overrun_errors: 0,
            },
        }
    }
    
    pub fn configure(&mut self, config: UartConfig) -> Result<(), UartError> {
        if self.state != UartState::Idle {
            return Err(UartError::Busy);
        }
        
        self.config = config;
        self.configure_hardware()?;
        
        Ok(())
    }
    
    fn configure_hardware(&self) -> Result<(), UartError> {
        // 配置UART硬件寄存器
        // 这里是简化实现
        Ok(())
    }
    
    pub fn send(&mut self, data: &[u8]) -> Result<usize, UartError> {
        if self.state == UartState::Error {
            return Err(UartError::HardwareError);
        }
        
        let mut sent = 0;
        for &byte in data {
            if self.tx_buffer.push(byte).is_ok() {
                sent += 1;
            } else {
                break; // 缓冲区满
            }
        }
        
        if sent > 0 {
            self.start_transmission()?;
        }
        
        Ok(sent)
    }
    
    fn start_transmission(&mut self) -> Result<(), UartError> {
        if self.state == UartState::Idle && !self.tx_buffer.is_empty() {
            self.state = UartState::Transmitting;
            // 启动硬件传输
            self.enable_tx_interrupt();
        }
        
        Ok(())
    }
    
    fn enable_tx_interrupt(&self) {
        // 启用发送中断
    }
    
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, UartError> {
        let mut received = 0;
        
        for i in 0..buffer.len() {
            if let Some(byte) = self.rx_buffer.get(i).copied() {
                buffer[i] = byte;
                received += 1;
            } else {
                break;
            }
        }
        
        // 清除已读取的数据
        for _ in 0..received {
            self.rx_buffer.remove(0);
        }
        
        Ok(received)
    }
    
    // 中断处理函数
    pub fn handle_tx_interrupt(&mut self) {
        if let Some(byte) = self.tx_buffer.get(0).copied() {
            // 发送字节到硬件
            self.write_data_register(byte);
            self.tx_buffer.remove(0);
            self.statistics.bytes_transmitted += 1;
            
            if self.tx_buffer.is_empty() {
                self.state = UartState::Idle;
                self.disable_tx_interrupt();
            }
        }
    }
    
    pub fn handle_rx_interrupt(&mut self) {
        let status = self.read_status_register();
        
        if status & 0x01 != 0 { // 数据就绪
            let data = self.read_data_register();
            
            // 检查错误
            if status & 0x02 != 0 { // 奇偶校验错误
                self.statistics.parity_errors += 1;
            }
            
            if status & 0x04 != 0 { // 帧错误
                self.statistics.framing_errors += 1;
            }
            
            if status & 0x08 != 0 { // 溢出错误
                self.statistics.overrun_errors += 1;
            }
            
            if status & 0x0E == 0 { // 无错误
                if self.rx_buffer.push(data).is_err() {
                    // 接收缓冲区满
                    self.statistics.reception_errors += 1;
                } else {
                    self.statistics.bytes_received += 1;
                }
            }
        }
    }
    
    fn write_data_register(&self, data: u8) {
        // 写入硬件数据寄存器
    }
    
    fn read_data_register(&self) -> u8 {
        // 读取硬件数据寄存器
        0
    }
    
    fn read_status_register(&self) -> u8 {
        // 读取硬件状态寄存器
        0
    }
    
    fn disable_tx_interrupt(&self) {
        // 禁用发送中断
    }
    
    pub fn get_statistics(&self) -> &UartStatistics {
        &self.statistics
    }
    
    pub fn reset_statistics(&mut self) {
        self.statistics = UartStatistics {
            bytes_transmitted: 0,
            bytes_received: 0,
            transmission_errors: 0,
            reception_errors: 0,
            parity_errors: 0,
            framing_errors: 0,
            overrun_errors: 0,
        };
    }
}

#[derive(Debug, Clone, Copy)]
pub enum UartError {
    InvalidConfig,
    Busy,
    BufferFull,
    HardwareError,
    TimeoutError,
}
```

### 2.2 SPI协议实现

```rust
// SPI配置
#[derive(Debug, Clone, Copy)]
pub struct SpiConfig {
    pub clock_polarity: ClockPolarity,
    pub clock_phase: ClockPhase,
    pub bit_order: BitOrder,
    pub clock_speed: u32,
    pub data_size: DataSize,
}

#[derive(Debug, Clone, Copy)]
pub enum ClockPolarity {
    IdleLow,  // CPOL = 0
    IdleHigh, // CPOL = 1
}

#[derive(Debug, Clone, Copy)]
pub enum ClockPhase {
    FirstEdge,  // CPHA = 0
    SecondEdge, // CPHA = 1
}

#[derive(Debug, Clone, Copy)]
pub enum BitOrder {
    MsbFirst,
    LsbFirst,
}

#[derive(Debug, Clone, Copy)]
pub enum DataSize {
    Bits8 = 8,
    Bits16 = 16,
}

// SPI驱动程序
pub struct SpiDriver {
    config: SpiConfig,
    cs_pin: Option<CsPin>,
    transfer_buffer: Vec<u8, 256>,
    state: SpiState,
    statistics: SpiStatistics,
}

pub struct CsPin {
    pin_number: u8,
    active_low: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiState {
    Idle,
    Transferring,
    Error,
}

#[derive(Debug, Clone)]
pub struct SpiStatistics {
    pub transfers_completed: u32,
    pub bytes_transferred: u32,
    pub transfer_errors: u32,
    pub cs_assertions: u32,
}

impl SpiDriver {
    pub fn new(config: SpiConfig) -> Self {
        Self {
            config,
            cs_pin: None,
            transfer_buffer: Vec::new(),
            state: SpiState::Idle,
            statistics: SpiStatistics {
                transfers_completed: 0,
                bytes_transferred: 0,
                transfer_errors: 0,
                cs_assertions: 0,
            },
        }
    }
    
    pub fn set_cs_pin(&mut self, pin_number: u8, active_low: bool) {
        self.cs_pin = Some(CsPin { pin_number, active_low });
    }
    
    pub fn transfer(&mut self, tx_data: &[u8], rx_data: &mut [u8]) -> Result<(), SpiError> {
        if self.state != SpiState::Idle {
            return Err(SpiError::Busy);
        }
        
        if tx_data.len() != rx_data.len() {
            return Err(SpiError::InvalidLength);
        }
        
        self.state = SpiState::Transferring;
        
        // 断言CS信号
        self.assert_cs();
        
        // 执行传输
        for (i, &tx_byte) in tx_data.iter().enumerate() {
            rx_data[i] = self.transfer_byte(tx_byte)?;
        }
        
        // 释放CS信号
        self.deassert_cs();
        
        self.state = SpiState::Idle;
        self.statistics.transfers_completed += 1;
        self.statistics.bytes_transferred += tx_data.len() as u32;
        
        Ok(())
    }
    
    fn transfer_byte(&self, tx_byte: u8) -> Result<u8, SpiError> {
        // 写入发送数据
        self.write_data_register(tx_byte);
        
        // 等待传输完成
        let timeout = 1000;
        let mut count = 0;
        
        while !self.is_transfer_complete() {
            count += 1;
            if count > timeout {
                return Err(SpiError::TimeoutError);
            }
        }
        
        // 读取接收数据
        Ok(self.read_data_register())
    }
    
    fn assert_cs(&mut self) {
        if let Some(ref cs_pin) = self.cs_pin {
            if cs_pin.active_low {
                self.set_gpio_low(cs_pin.pin_number);
            } else {
                self.set_gpio_high(cs_pin.pin_number);
            }
            self.statistics.cs_assertions += 1;
        }
    }
    
    fn deassert_cs(&self) {
        if let Some(ref cs_pin) = self.cs_pin {
            if cs_pin.active_low {
                self.set_gpio_high(cs_pin.pin_number);
            } else {
                self.set_gpio_low(cs_pin.pin_number);
            }
        }
    }
    
    fn write_data_register(&self, data: u8) {
        // 写入SPI数据寄存器
    }
    
    fn read_data_register(&self) -> u8 {
        // 读取SPI数据寄存器
        0
    }
    
    fn is_transfer_complete(&self) -> bool {
        // 检查传输完成标志
        true
    }
    
    fn set_gpio_high(&self, pin: u8) {
        // 设置GPIO引脚为高电平
    }
    
    fn set_gpio_low(&self, pin: u8) {
        // 设置GPIO引脚为低电平
    }
    
    pub fn write_register(&mut self, register: u8, value: u8) -> Result<(), SpiError> {
        let tx_data = [register, value];
        let mut rx_data = [0u8; 2];
        
        self.transfer(&tx_data, &mut rx_data)
    }
    
    pub fn read_register(&mut self, register: u8) -> Result<u8, SpiError> {
        let tx_data = [register | 0x80, 0x00]; // 读取命令通常设置最高位
        let mut rx_data = [0u8; 2];
        
        self.transfer(&tx_data, &mut rx_data)?;
        Ok(rx_data[1])
    }
    
    pub fn get_statistics(&self) -> &SpiStatistics {
        &self.statistics
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SpiError {
    InvalidConfig,
    InvalidLength,
    Busy,
    TimeoutError,
    HardwareError,
}
```

### 2.3 I2C协议实现

```rust
// I2C配置
#[derive(Debug, Clone, Copy)]
pub struct I2cConfig {
    pub clock_speed: I2cSpeed,
    pub addressing_mode: AddressingMode,
    pub own_address: u16,
}

#[derive(Debug, Clone, Copy)]
pub enum I2cSpeed {
    Standard = 100_000,    // 100 kHz
    Fast = 400_000,        // 400 kHz
    FastPlus = 1_000_000,  // 1 MHz
    HighSpeed = 3_400_000, // 3.4 MHz
}

#[derive(Debug, Clone, Copy)]
pub enum AddressingMode {
    SevenBit,
    TenBit,
}

// I2C驱动程序
pub struct I2cDriver {
    config: I2cConfig,
    state: I2cState,
    current_transaction: Option<I2cTransaction>,
    statistics: I2cStatistics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cState {
    Idle,
    StartCondition,
    AddressPhase,
    DataPhase,
    StopCondition,
    Error,
}

#[derive(Debug, Clone)]
pub struct I2cTransaction {
    pub slave_address: u16,
    pub operation: I2cOperation,
    pub data: Vec<u8, 256>,
    pub bytes_processed: usize,
}

#[derive(Debug, Clone, Copy)]
pub enum I2cOperation {
    Write,
    Read,
    WriteRead { write_len: usize },
}

#[derive(Debug, Clone)]
pub struct I2cStatistics {
    pub transactions_completed: u32,
    pub bytes_transmitted: u32,
    pub bytes_received: u32,
    pub ack_failures: u32,
    pub arbitration_losses: u32,
    pub bus_errors: u32,
}

impl I2cDriver {
    pub fn new(config: I2cConfig) -> Self {
        Self {
            config,
            state: I2cState::Idle,
            current_transaction: None,
            statistics: I2cStatistics {
                transactions_completed: 0,
                bytes_transmitted: 0,
                bytes_received: 0,
                ack_failures: 0,
                arbitration_losses: 0,
                bus_errors: 0,
            },
        }
    }
    
    pub fn write(&mut self, slave_address: u16, data: &[u8]) -> Result<(), I2cError> {
        if self.state != I2cState::Idle {
            return Err(I2cError::Busy);
        }
        
        let mut transaction_data = Vec::new();
        for &byte in data {
            if transaction_data.push(byte).is_err() {
                return Err(I2cError::BufferOverflow);
            }
        }
        
        self.current_transaction = Some(I2cTransaction {
            slave_address,
            operation: I2cOperation::Write,
            data: transaction_data,
            bytes_processed: 0,
        });
        
        self.start_transaction()
    }
    
    pub fn read(&mut self, slave_address: u16, buffer: &mut [u8]) -> Result<(), I2cError> {
        if self.state != I2cState::Idle {
            return Err(I2cError::Busy);
        }
        
        let mut transaction_data = Vec::new();
        for _ in 0..buffer.len() {
            if transaction_data.push(0).is_err() {
                return Err(I2cError::BufferOverflow);
            }
        }
        
        self.current_transaction = Some(I2cTransaction {
            slave_address,
            operation: I2cOperation::Read,
            data: transaction_data,
            bytes_processed: 0,
        });
        
        self.start_transaction()?;
        
        // 复制读取的数据
        if let Some(ref transaction) = self.current_transaction {
            for (i, &byte) in transaction.data.iter().enumerate() {
                if i < buffer.len() {
                    buffer[i] = byte;
                }
            }
        }
        
        Ok(())
    }
    
    pub fn write_read(&mut self, slave_address: u16, write_data: &[u8], read_buffer: &mut [u8]) -> Result<(), I2cError> {
        if self.state != I2cState::Idle {
            return Err(I2cError::Busy);
        }
        
        let mut transaction_data = Vec::new();
        
        // 添加写数据
        for &byte in write_data {
            if transaction_data.push(byte).is_err() {
                return Err(I2cError::BufferOverflow);
            }
        }
        
        // 为读数据预留空间
        for _ in 0..read_buffer.len() {
            if transaction_data.push(0).is_err() {
                return Err(I2cError::BufferOverflow);
            }
        }
        
        self.current_transaction = Some(I2cTransaction {
            slave_address,
            operation: I2cOperation::WriteRead { write_len: write_data.len() },
            data: transaction_data,
            bytes_processed: 0,
        });
        
        self.start_transaction()?;
        
        // 复制读取的数据
        if let Some(ref transaction) = self.current_transaction {
            for (i, &byte) in transaction.data[write_data.len()..].iter().enumerate() {
                if i < read_buffer.len() {
                    read_buffer[i] = byte;
                }
            }
        }
        
        Ok(())
    }
    
    fn start_transaction(&mut self) -> Result<(), I2cError> {
        // 检查总线是否空闲
        if !self.is_bus_idle() {
            return Err(I2cError::BusBusy);
        }
        
        // 生成起始条件
        self.generate_start_condition();
        self.state = I2cState::StartCondition;
        
        // 启用中断
        self.enable_interrupts();
        
        Ok(())
    }
    
    fn generate_start_condition(&self) {
        // 生成I2C起始条件
    }
    
    fn generate_stop_condition(&self) {
        // 生成I2C停止条件
    }
    
    fn is_bus_idle(&self) -> bool {
        // 检查I2C总线是否空闲
        true
    }
    
    fn enable_interrupts(&self) {
        // 启用I2C中断
    }
    
    fn disable_interrupts(&self) {
        // 禁用I2C中断
    }
    
    // 中断处理函数
    pub fn handle_interrupt(&mut self) {
        let status = self.read_status_register();
        
        match self.state {
            I2cState::StartCondition => {
                if status & 0x01 != 0 { // 起始条件已发送
                    self.send_address();
                    self.state = I2cState::AddressPhase;
                }
            }
            
            I2cState::AddressPhase => {
                if status & 0x02 != 0 { // 地址已发送
                    if status & 0x04 != 0 { // 收到ACK
                        self.state = I2cState::DataPhase;
                        self.process_data_phase();
                    } else { // 收到NACK
                        self.statistics.ack_failures += 1;
                        self.handle_error(I2cError::NoAcknowledge);
                    }
                }
            }
            
            I2cState::DataPhase => {
                self.process_data_phase();
            }
            
            I2cState::StopCondition => {
                if status & 0x08 != 0 { // 停止条件已发送
                    self.complete_transaction();
                }
            }
            
            I2cState::Error => {
                self.handle_error_recovery();
            }
            
            _ => {}
        }
        
        // 检查错误条件
        if status & 0x10 != 0 { // 仲裁丢失
            self.statistics.arbitration_losses += 1;
            self.handle_error(I2cError::ArbitrationLoss);
        }
        
        if status & 0x20 != 0 { // 总线错误
            self.statistics.bus_errors += 1;
            self.handle_error(I2cError::BusError);
        }
    }
    
    fn send_address(&self) {
        if let Some(ref transaction) = self.current_transaction {
            let address = match transaction.operation {
                I2cOperation::Write | I2cOperation::WriteRead { .. } => {
                    (transaction.slave_address << 1) | 0 // 写操作
                }
                I2cOperation::Read => {
                    (transaction.slave_address << 1) | 1 // 读操作
                }
            };
            
            self.write_data_register(address as u8);
        }
    }
    
    fn process_data_phase(&mut self) {
        if let Some(ref mut transaction) = self.current_transaction {
            match transaction.operation {
                I2cOperation::Write => {
                    if transaction.bytes_processed < transaction.data.len() {
                        let byte = transaction.data[transaction.bytes_processed];
                        self.write_data_register(byte);
                        transaction.bytes_processed += 1;
                        self.statistics.bytes_transmitted += 1;
                    } else {
                        self.generate_stop_condition();
                        self.state = I2cState::StopCondition;
                    }
                }
                
                I2cOperation::Read => {
                    if transaction.bytes_processed < transaction.data.len() {
                        let byte = self.read_data_register();
                        transaction.data[transaction.bytes_processed] = byte;
                        transaction.bytes_processed += 1;
                        self.statistics.bytes_received += 1;
                        
                        // 发送ACK/NACK
                        if transaction.bytes_processed < transaction.data.len() {
                            self.send_ack();
                        } else {
                            self.send_nack();
                            self.generate_stop_condition();
                            self.state = I2cState::StopCondition;
                        }
                    }
                }
                
                I2cOperation::WriteRead { write_len } => {
                    if transaction.bytes_processed < write_len {
                        // 写阶段
                        let byte = transaction.data[transaction.bytes_processed];
                        self.write_data_register(byte);
                        transaction.bytes_processed += 1;
                        self.statistics.bytes_transmitted += 1;
                        
                        if transaction.bytes_processed == write_len {
                            // 重新开始读操作
                            self.generate_start_condition();
                            self.state = I2cState::StartCondition;
                        }
                    } else {
                        // 读阶段
                        let byte = self.read_data_register();
                        transaction.data[transaction.bytes_processed] = byte;
                        transaction.bytes_processed += 1;
                        self.statistics.bytes_received += 1;
                        
                        if transaction.bytes_processed < transaction.data.len() {
                            self.send_ack();
                        } else {
                            self.send_nack();
                            self.generate_stop_condition();
                            self.state = I2cState::StopCondition;
                        }
                    }
                }
            }
        }
    }
    
    fn complete_transaction(&mut self) {
        self.state = I2cState::Idle;
        self.current_transaction = None;
        self.statistics.transactions_completed += 1;
        self.disable_interrupts();
    }
    
    fn handle_error(&mut self, error: I2cError) {
        self.state = I2cState::Error;
        // 记录错误并尝试恢复
    }
    
    fn handle_error_recovery(&mut self) {
        // 错误恢复逻辑
        self.generate_stop_condition();
        self.state = I2cState::Idle;
        self.current_transaction = None;
        self.disable_interrupts();
    }
    
    fn write_data_register(&self, data: u8) {
        // 写入I2C数据寄存器
    }
    
    fn read_data_register(&self) -> u8 {
        // 读取I2C数据寄存器
        0
    }
    
    fn read_status_register(&self) -> u8 {
        // 读取I2C状态寄存器
        0
    }
    
    fn send_ack(&self) {
        // 发送ACK信号
    }
    
    fn send_nack(&self) {
        // 发送NACK信号
    }
    
    pub fn get_statistics(&self) -> &I2cStatistics {
        &self.statistics
    }
}

#[derive(Debug, Clone, Copy)]
pub enum I2cError {
    InvalidConfig,
    Busy,
    BusBusy,
    NoAcknowledge,
    ArbitrationLoss,
    BusError,
    BufferOverflow,
    TimeoutError,
}
```

## 3. 协议栈设计

### 3.1 分层协议栈

```rust
// 协议栈结构
pub struct ProtocolStack {
    physical_layer: PhysicalLayer,
    data_link_layer: DataLinkLayer,
    network_layer: NetworkLayer,
    transport_layer: TransportLayer,
    application_layer: ApplicationLayer,
}

// 物理层
pub struct PhysicalLayer {
    interface: PhysicalInterface,
    signal_parameters: SignalParameters,
}

pub enum PhysicalInterface {
    Uart(UartDriver),
    Spi(SpiDriver),
    I2c(I2cDriver),
    Ethernet(EthernetDriver),
    Wireless(WirelessDriver),
}

pub struct SignalParameters {
    pub voltage_levels: (f32, f32), // (low, high)
    pub timing_parameters: TimingParameters,
    pub electrical_characteristics: ElectricalCharacteristics,
}

pub struct TimingParameters {
    pub setup_time: u32,
    pub hold_time: u32,
    pub propagation_delay: u32,
}

pub struct ElectricalCharacteristics {
    pub input_impedance: f32,
    pub output_impedance: f32,
    pub drive_strength: f32,
}

// 数据链路层
pub struct DataLinkLayer {
    framing: FramingProtocol,
    error_detection: ErrorDetection,
    flow_control: FlowControl,
    access_control: AccessControl,
}

pub enum FramingProtocol {
    ByteStuffing,
    BitStuffing,
    LengthField,
    StartEndDelimiters,
}

pub enum ErrorDetection {
    Checksum,
    Crc8,
    Crc16,
    Crc32,
    Hamming,
}

pub enum AccessControl {
    None,
    Csma,      // 载波侦听多路访问
    TokenPassing,
    Polling,
}

// 网络层
pub struct NetworkLayer {
    addressing: AddressingScheme,
    routing: RoutingProtocol,
    fragmentation: FragmentationSupport,
}

pub enum AddressingScheme {
    Flat(u8),           // 平面地址
    Hierarchical(u16),  // 分层地址
    Geographic(GeoAddress),
}

pub struct GeoAddress {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
}

pub enum RoutingProtocol {
    Static,
    Dynamic(DynamicRouting),
}

pub enum DynamicRouting {
    DistanceVector,
    LinkState,
    Hybrid,
}

// 传输层
pub struct TransportLayer {
    reliability: ReliabilityLevel,
    congestion_control: CongestionControl,
    multiplexing: MultiplexingSupport,
}

pub enum ReliabilityLevel {
    BestEffort,
    ReliableDelivery,
    OrderedDelivery,
    ExactlyOnceDelivery,
}

pub enum CongestionControl {
    None,
    WindowBased,
    RateBased,
    Hybrid,
}

// 应用层
pub struct ApplicationLayer {
    protocols: Vec<ApplicationProtocol, 8>,
    services: Vec<ApplicationService, 16>,
}

pub enum ApplicationProtocol {
    Http,
    Mqtt,
    CoAP,
    Custom(CustomProtocol),
}

pub struct CustomProtocol {
    pub name: &'static str,
    pub version: u8,
    pub message_types: Vec<MessageType, 16>,
}

pub struct MessageType {
    pub id: u8,
    pub name: &'static str,
    pub format: MessageFormat,
}

pub enum MessageFormat {
    Binary,
    Text,
    Json,
    Protobuf,
}

impl ProtocolStack {
    pub fn new() -> Self {
        Self {
            physical_layer: PhysicalLayer::new(),
            data_link_layer: DataLinkLayer::new(),
            network_layer: NetworkLayer::new(),
            transport_layer: TransportLayer::new(),
            application_layer: ApplicationLayer::new(),
        }
    }
    
    pub fn send_message(&mut self, message: &ApplicationMessage) -> Result<(), ProtocolError> {
        // 应用层处理
        let app_data = self.application_layer.encode_message(message)?;
        
        // 传输层处理
        let transport_segments = self.transport_layer.segment_data(&app_data)?;
        
        for segment in transport_segments {
            // 网络层处理
            let network_packet = self.network_layer.create_packet(segment)?;
            
            // 数据链路层处理
            let link_frame = self.data_link_layer.create_frame(network_packet)?;
            
            // 物理层传输
            self.physical_layer.transmit(&link_frame)?;
        }
        
        Ok(())
    }
    
    pub fn receive_message(&mut self) -> Result<Option<ApplicationMessage>, ProtocolError> {
        // 物理层接收
        if let Some(raw_data) = self.physical_layer.receive()? {
            // 数据链路层处理
            let frame = self.data_link_layer.parse_frame(&raw_data)?;
            
            // 网络层处理
            let packet = self.network_layer.parse_packet(&frame.payload)?;
            
            // 传输层处理
            let segment = self.transport_layer.parse_segment(&packet.payload)?;
            
            // 应用层处理
            let message = self.application_layer.decode_message(&segment.payload)?;
            
            Ok(Some(message))
        } else {
            Ok(None)
        }
    }
}

// 应用消息
#[derive(Debug, Clone)]
pub struct ApplicationMessage {
    pub message_type: u8,
    pub source: u16,
    pub destination: u16,
    pub payload: Vec<u8, 256>,
    pub qos: QualityOfService,
}

#[derive(Debug, Clone, Copy)]
pub struct QualityOfService {
    pub priority: u8,
    pub reliability: bool,
    pub ordering: bool,
    pub max_latency: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum ProtocolError {
    PhysicalLayerError,
    FramingError,
    ChecksumError,
    RoutingError,
    TransportError,
    ApplicationError,
    BufferOverflow,
    TimeoutError,
}
```

## 4. 总结

通信协议是嵌入式系统的重要组成部分，正确选择和实现通信协议对系统性能和可靠性至关重要。

### 关键要点

1. **协议选择**: 根据应用需求选择合适的通信协议
2. **分层设计**: 采用分层架构简化协议实现
3. **错误处理**: 实现完善的错误检测和恢复机制
4. **性能优化**: 优化协议参数以满足性能要求
5. **可靠性**: 确保通信的可靠性和稳定性

### 设计原则

- **简单性**: 协议设计应尽可能简单
- **可扩展性**: 支持功能扩展和协议升级
- **互操作性**: 确保与其他系统的兼容性
- **效率**: 优化带宽利用率和处理效率
- **鲁棒性**: 具备良好的错误处理能力

### 实践建议

- 深入理解各种协议的特点和适用场景
- 实现完整的协议栈和测试框架
- 使用协议分析工具进行调试和优化
- 建立协议性能基准和监控机制
- 学习和应用最新的通信技术

### 下一步

完成通信协议学习后，建议继续学习：
- [嵌入式软件架构](./08-software-architecture.md)
- [串口通信实践](../../05-serial-communication/README.md)
- [通信协议实现](../../08-communication-protocols/README.md)