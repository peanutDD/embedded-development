# 蓝牙通信实现

本文档介绍在STM32F4微控制器上实现蓝牙通信的方法，包括经典蓝牙和低功耗蓝牙(BLE)的实现。

## 蓝牙技术概述

蓝牙是一种短距离无线通信技术，广泛应用于物联网设备的近场通信。

### 蓝牙版本对比
- **经典蓝牙**: 高数据传输率，功耗较高
- **低功耗蓝牙(BLE)**: 低功耗，适合传感器应用
- **蓝牙5.0**: 更长距离，更高速度，更低功耗

### 应用场景
- 智能手机与设备配对
- 传感器数据采集
- 设备状态监控
- 近场数据传输

## HC-05经典蓝牙模块

HC-05是常用的经典蓝牙模块，支持SPP(串口协议)透传。

### 硬件特性
- **蓝牙版本**: 2.0+EDR
- **工作频率**: 2.4GHz ISM频段
- **传输距离**: 10米(Class 2)
- **数据速率**: 最高2.1Mbps
- **工作电压**: 3.3V-5V
- **接口**: UART串口

### HC-05模块控制实现

```rust
use stm32f4xx_hal::{
    prelude::*,
    pac,
    serial::{Serial, Config},
    gpio::{Pin, Output, PushPull, Input, PullUp},
};
use heapless::{String, Vec};
use nb::block;

// 蓝牙模块结构
pub struct BluetoothModule<UART, EN_PIN> {
    uart: UART,
    en_pin: EN_PIN,
    buffer: Vec<u8, 256>,
    connected: bool,
    device_name: String<32>,
    pin_code: String<8>,
}

impl<UART, EN_PIN> BluetoothModule<UART, EN_PIN>
where
    UART: embedded_hal::serial::Read<u8> + embedded_hal::serial::Write<u8>,
    EN_PIN: OutputPin,
{
    pub fn new(uart: UART, en_pin: EN_PIN) -> Self {
        Self {
            uart,
            en_pin,
            buffer: Vec::new(),
            connected: false,
            device_name: String::new(),
            pin_code: String::new(),
        }
    }
    
    // 进入AT命令模式
    pub fn enter_at_mode(&mut self) -> Result<(), &'static str> {
        // 拉高EN引脚进入AT模式
        self.en_pin.set_high().map_err(|_| "Failed to set EN pin")?;
        
        // 等待模块启动
        cortex_m::asm::delay(84000 * 100); // 100ms延时
        
        Ok(())
    }
    
    // 退出AT命令模式
    pub fn exit_at_mode(&mut self) -> Result<(), &'static str> {
        // 拉低EN引脚退出AT模式
        self.en_pin.set_low().map_err(|_| "Failed to clear EN pin")?;
        
        // 等待模块重启
        cortex_m::asm::delay(84000 * 500); // 500ms延时
        
        Ok(())
    }
    
    // 发送AT命令
    pub fn send_at_command(&mut self, command: &str) -> Result<String<256>, &'static str> {
        // 清空缓冲区
        self.buffer.clear();
        
        // 发送命令
        for byte in command.bytes() {
            block!(self.uart.write(byte)).map_err(|_| "UART write error")?;
        }
        
        // 发送回车换行
        block!(self.uart.write(b'\r')).map_err(|_| "UART write error")?;
        block!(self.uart.write(b'\n')).map_err(|_| "UART write error")?;
        
        // 读取响应
        let mut response = String::new();
        for _ in 0..2000 { // 2秒超时
            if let Ok(byte) = self.uart.read() {
                if self.buffer.push(byte).is_err() {
                    break;
                }
                
                // 检查是否收到完整响应
                if self.buffer.len() >= 2 {
                    let len = self.buffer.len();
                    if self.buffer[len-2] == b'\r' && self.buffer[len-1] == b'\n' {
                        // 转换为字符串
                        for &b in &self.buffer[..len-2] {
                            if response.push(b as char).is_err() {
                                break;
                            }
                        }
                        return Ok(response);
                    }
                }
            }
            
            cortex_m::asm::delay(84000); // 1ms延时
        }
        
        Err("AT command timeout")
    }
    
    // 测试连接
    pub fn test_connection(&mut self) -> Result<bool, &'static str> {
        let response = self.send_at_command("AT")?;
        Ok(response.contains("OK"))
    }
    
    // 设置设备名称
    pub fn set_device_name(&mut self, name: &str) -> Result<(), &'static str> {
        self.device_name.clear();
        self.device_name.push_str(name).map_err(|_| "Name too long")?;
        
        let mut command = String::<64>::new();
        command.push_str("AT+NAME=").map_err(|_| "Command too long")?;
        command.push_str(name).map_err(|_| "Command too long")?;
        
        let response = self.send_at_command(&command)?;
        
        if response.contains("OK") {
            Ok(())
        } else {
            Err("Failed to set device name")
        }
    }
    
    // 设置PIN码
    pub fn set_pin_code(&mut self, pin: &str) -> Result<(), &'static str> {
        self.pin_code.clear();
        self.pin_code.push_str(pin).map_err(|_| "PIN too long")?;
        
        let mut command = String::<32>::new();
        command.push_str("AT+PSWD=").map_err(|_| "Command too long")?;
        command.push_str(pin).map_err(|_| "Command too long")?;
        
        let response = self.send_at_command(&command)?;
        
        if response.contains("OK") {
            Ok(())
        } else {
            Err("Failed to set PIN code")
        }
    }
    
    // 设置工作模式
    pub fn set_role(&mut self, role: BluetoothRole) -> Result<(), &'static str> {
        let command = match role {
            BluetoothRole::Slave => "AT+ROLE=0",
            BluetoothRole::Master => "AT+ROLE=1",
        };
        
        let response = self.send_at_command(command)?;
        
        if response.contains("OK") {
            Ok(())
        } else {
            Err("Failed to set role")
        }
    }
    
    // 查询设备地址
    pub fn get_address(&mut self) -> Result<String<32>, &'static str> {
        let response = self.send_at_command("AT+ADDR?")?;
        
        // 解析地址（简化版）
        if let Some(start) = response.find("+ADDR:") {
            let addr_start = start + 6;
            let addr_end = response.len().min(addr_start + 17); // MAC地址长度
            let addr_str = &response[addr_start..addr_end];
            
            let mut address = String::new();
            address.push_str(addr_str).map_err(|_| "Address too long")?;
            Ok(address)
        } else {
            Err("Failed to get address")
        }
    }
    
    // 搜索设备
    pub fn scan_devices(&mut self) -> Result<Vec<String<32>, 8>, &'static str> {
        let response = self.send_at_command("AT+INQ")?;
        let mut devices = Vec::new();
        
        // 解析搜索结果（简化版）
        for line in response.split('\n') {
            if line.contains("+INQ:") {
                if let Some(start) = line.find("+INQ:") {
                    let device_info = &line[start + 5..];
                    let mut device = String::new();
                    device.push_str(device_info).map_err(|_| "Device info too long")?;
                    devices.push(device).map_err(|_| "Too many devices")?;
                }
            }
        }
        
        Ok(devices)
    }
    
    // 连接设备
    pub fn connect_device(&mut self, address: &str) -> Result<(), &'static str> {
        let mut command = String::<64>::new();
        command.push_str("AT+CONN=").map_err(|_| "Command too long")?;
        command.push_str(address).map_err(|_| "Command too long")?;
        
        let response = self.send_at_command(&command)?;
        
        if response.contains("OK") {
            self.connected = true;
            Ok(())
        } else {
            Err("Failed to connect device")
        }
    }
    
    // 断开连接
    pub fn disconnect(&mut self) -> Result<(), &'static str> {
        let response = self.send_at_command("AT+DISC")?;
        
        if response.contains("OK") {
            self.connected = false;
            Ok(())
        } else {
            Err("Failed to disconnect")
        }
    }
    
    // 发送数据（透传模式）
    pub fn send_data(&mut self, data: &[u8]) -> Result<(), &'static str> {
        if !self.connected {
            return Err("Not connected");
        }
        
        for &byte in data {
            block!(self.uart.write(byte)).map_err(|_| "UART write error")?;
        }
        
        Ok(())
    }
    
    // 接收数据（透传模式）
    pub fn receive_data(&mut self) -> Result<Vec<u8, 256>, &'static str> {
        let mut data = Vec::new();
        
        // 非阻塞读取
        for _ in 0..1000 { // 1秒超时
            if let Ok(byte) = self.uart.read() {
                if data.push(byte).is_err() {
                    break;
                }
            } else {
                break;
            }
            cortex_m::asm::delay(84); // 1us延时
        }
        
        Ok(data)
    }
}

// 蓝牙工作角色
#[derive(Debug, Clone, Copy)]
pub enum BluetoothRole {
    Slave,  // 从设备
    Master, // 主设备
}
```

## ESP32 BLE实现

ESP32集成了WiFi和蓝牙功能，支持BLE 4.2和5.0标准。

### BLE服务和特征实现

```rust
use heapless::{String, Vec};

// BLE服务UUID
pub const DEVICE_INFO_SERVICE_UUID: u16 = 0x180A;
pub const BATTERY_SERVICE_UUID: u16 = 0x180F;
pub const CUSTOM_SERVICE_UUID: u128 = 0x12345678_1234_1234_1234_123456789ABC;

// BLE特征UUID
pub const MANUFACTURER_NAME_CHAR_UUID: u16 = 0x2A29;
pub const MODEL_NUMBER_CHAR_UUID: u16 = 0x2A24;
pub const BATTERY_LEVEL_CHAR_UUID: u16 = 0x2A19;

// BLE特征属性
#[derive(Debug, Clone, Copy)]
pub struct CharacteristicProperties {
    pub read: bool,
    pub write: bool,
    pub notify: bool,
    pub indicate: bool,
}

// BLE特征结构
pub struct BleCharacteristic {
    pub uuid: u128,
    pub properties: CharacteristicProperties,
    pub value: Vec<u8, 64>,
    pub descriptor: String<32>,
}

impl BleCharacteristic {
    pub fn new(uuid: u128, properties: CharacteristicProperties) -> Self {
        Self {
            uuid,
            properties,
            value: Vec::new(),
            descriptor: String::new(),
        }
    }
    
    // 设置值
    pub fn set_value(&mut self, data: &[u8]) -> Result<(), &'static str> {
        self.value.clear();
        for &byte in data {
            self.value.push(byte).map_err(|_| "Value too long")?;
        }
        Ok(())
    }
    
    // 获取值
    pub fn get_value(&self) -> &[u8] {
        &self.value
    }
    
    // 设置描述符
    pub fn set_descriptor(&mut self, desc: &str) -> Result<(), &'static str> {
        self.descriptor.clear();
        self.descriptor.push_str(desc).map_err(|_| "Descriptor too long")?;
        Ok(())
    }
}

// BLE服务结构
pub struct BleService {
    pub uuid: u128,
    pub characteristics: Vec<BleCharacteristic, 8>,
    pub name: String<32>,
}

impl BleService {
    pub fn new(uuid: u128, name: &str) -> Result<Self, &'static str> {
        let mut service_name = String::new();
        service_name.push_str(name).map_err(|_| "Name too long")?;
        
        Ok(Self {
            uuid,
            characteristics: Vec::new(),
            name: service_name,
        })
    }
    
    // 添加特征
    pub fn add_characteristic(&mut self, characteristic: BleCharacteristic) -> Result<(), &'static str> {
        self.characteristics.push(characteristic).map_err(|_| "Too many characteristics")
    }
    
    // 查找特征
    pub fn find_characteristic(&mut self, uuid: u128) -> Option<&mut BleCharacteristic> {
        self.characteristics.iter_mut().find(|c| c.uuid == uuid)
    }
}

// BLE设备结构
pub struct BleDevice<UART> {
    uart: UART,
    services: Vec<BleService, 4>,
    device_name: String<32>,
    advertising: bool,
    connected: bool,
    connection_handle: u16,
}

impl<UART> BleDevice<UART>
where
    UART: embedded_hal::serial::Read<u8> + embedded_hal::serial::Write<u8>,
{
    pub fn new(uart: UART, device_name: &str) -> Result<Self, &'static str> {
        let mut name = String::new();
        name.push_str(device_name).map_err(|_| "Name too long")?;
        
        Ok(Self {
            uart,
            services: Vec::new(),
            device_name: name,
            advertising: false,
            connected: false,
            connection_handle: 0,
        })
    }
    
    // 初始化BLE
    pub fn initialize(&mut self) -> Result<(), &'static str> {
        // 发送初始化命令（ESP32 AT命令）
        self.send_at_command("AT+BLEINIT=2")?; // BLE服务器模式
        self.send_at_command("AT+BLEADDR?")?;  // 获取BLE地址
        
        Ok(())
    }
    
    // 发送AT命令
    fn send_at_command(&mut self, command: &str) -> Result<String<256>, &'static str> {
        // 发送命令
        for byte in command.bytes() {
            block!(self.uart.write(byte)).map_err(|_| "UART write error")?;
        }
        
        block!(self.uart.write(b'\r')).map_err(|_| "UART write error")?;
        block!(self.uart.write(b'\n')).map_err(|_| "UART write error")?;
        
        // 读取响应
        let mut response = String::new();
        let mut buffer = Vec::<u8, 256>::new();
        
        for _ in 0..2000 { // 2秒超时
            if let Ok(byte) = self.uart.read() {
                if buffer.push(byte).is_err() {
                    break;
                }
                
                if buffer.len() >= 2 {
                    let len = buffer.len();
                    if buffer[len-2] == b'\r' && buffer[len-1] == b'\n' {
                        for &b in &buffer[..len-2] {
                            if response.push(b as char).is_err() {
                                break;
                            }
                        }
                        return Ok(response);
                    }
                }
            }
            
            cortex_m::asm::delay(84000); // 1ms延时
        }
        
        Err("AT command timeout")
    }
    
    // 添加服务
    pub fn add_service(&mut self, service: BleService) -> Result<(), &'static str> {
        // 创建服务AT命令
        let mut command = String::<128>::new();
        command.push_str("AT+BLEGATTSSRVCRE").map_err(|_| "Command too long")?;
        
        self.services.push(service).map_err(|_| "Too many services")?;
        
        Ok(())
    }
    
    // 开始广播
    pub fn start_advertising(&mut self) -> Result<(), &'static str> {
        // 设置广播数据
        let mut adv_data = String::<128>::new();
        adv_data.push_str("AT+BLEADVDATA=\"").map_err(|_| "Command too long")?;
        
        // 添加设备名称到广播数据
        // 简化的广播数据格式
        adv_data.push_str("02010602").map_err(|_| "Command too long")?; // 标志
        
        // 设备名称长度和类型
        let name_len = self.device_name.len() + 1;
        adv_data.push_str(&format_hex_byte(name_len as u8)).map_err(|_| "Command too long")?;
        adv_data.push_str("09").map_err(|_| "Command too long")?; // 完整本地名称
        
        // 设备名称（十六进制编码）
        for byte in self.device_name.bytes() {
            adv_data.push_str(&format_hex_byte(byte)).map_err(|_| "Command too long")?;
        }
        
        adv_data.push('"').map_err(|_| "Command too long")?;
        
        self.send_at_command(&adv_data)?;
        
        // 开始广播
        self.send_at_command("AT+BLEADVSTART")?;
        
        self.advertising = true;
        Ok(())
    }
    
    // 停止广播
    pub fn stop_advertising(&mut self) -> Result<(), &'static str> {
        self.send_at_command("AT+BLEADVSTOP")?;
        self.advertising = false;
        Ok(())
    }
    
    // 处理连接事件
    pub fn handle_connection(&mut self) -> Result<(), &'static str> {
        // 检查连接状态
        let response = self.send_at_command("AT+BLECONN?")?;
        
        if response.contains("+BLECONN:") {
            // 解析连接句柄
            if let Some(start) = response.find("+BLECONN:") {
                // 简化的连接句柄解析
                self.connected = true;
                self.connection_handle = 1; // 简化处理
            }
        }
        
        Ok(())
    }
    
    // 发送通知
    pub fn send_notification(&mut self, char_uuid: u128, data: &[u8]) -> Result<(), &'static str> {
        if !self.connected {
            return Err("Not connected");
        }
        
        // 构建通知命令
        let mut command = String::<128>::new();
        command.push_str("AT+BLEGATTSNTFY=").map_err(|_| "Command too long")?;
        
        // 连接句柄
        command.push('0').map_err(|_| "Command too long")?;
        command.push(',').map_err(|_| "Command too long")?;
        
        // 特征句柄（简化处理）
        command.push('1').map_err(|_| "Command too long")?;
        command.push(',').map_err(|_| "Command too long")?;
        
        // 数据长度
        let data_len = data.len();
        command.push((b'0' + (data_len / 10) as u8) as char).map_err(|_| "Command too long")?;
        command.push((b'0' + (data_len % 10) as u8) as char).map_err(|_| "Command too long")?;
        command.push(',').map_err(|_| "Command too long")?;
        
        // 数据（十六进制）
        for &byte in data {
            command.push_str(&format_hex_byte(byte)).map_err(|_| "Command too long")?;
        }
        
        self.send_at_command(&command)?;
        
        Ok(())
    }
    
    // 读取特征值
    pub fn read_characteristic(&mut self, service_uuid: u128, char_uuid: u128) -> Result<Vec<u8, 64>, &'static str> {
        // 查找服务和特征
        for service in &self.services {
            if service.uuid == service_uuid {
                for characteristic in &service.characteristics {
                    if characteristic.uuid == char_uuid {
                        let mut data = Vec::new();
                        for &byte in characteristic.get_value() {
                            data.push(byte).map_err(|_| "Data too long")?;
                        }
                        return Ok(data);
                    }
                }
            }
        }
        
        Err("Characteristic not found")
    }
    
    // 写入特征值
    pub fn write_characteristic(&mut self, service_uuid: u128, char_uuid: u128, data: &[u8]) -> Result<(), &'static str> {
        // 查找服务和特征
        for service in &mut self.services {
            if service.uuid == service_uuid {
                if let Some(characteristic) = service.find_characteristic(char_uuid) {
                    characteristic.set_value(data)?;
                    return Ok(());
                }
            }
        }
        
        Err("Characteristic not found")
    }
    
    // 获取连接状态
    pub fn is_connected(&self) -> bool {
        self.connected
    }
    
    // 获取广播状态
    pub fn is_advertising(&self) -> bool {
        self.advertising
    }
}

// 十六进制字节格式化
fn format_hex_byte(byte: u8) -> String<2> {
    let mut result = String::new();
    let high = (byte >> 4) & 0x0F;
    let low = byte & 0x0F;
    
    let high_char = if high < 10 { (b'0' + high) as char } else { (b'A' + high - 10) as char };
    let low_char = if low < 10 { (b'0' + low) as char } else { (b'A' + low - 10) as char };
    
    result.push(high_char).ok();
    result.push(low_char).ok();
    
    result
}
```

## 蓝牙数据传输协议

实现自定义的蓝牙数据传输协议，支持可靠的数据传输。

```rust
use heapless::{String, Vec};

// 数据包类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PacketType {
    Data = 0x01,
    Ack = 0x02,
    Nack = 0x03,
    Heartbeat = 0x04,
}

// 数据包结构
#[derive(Debug, Clone)]
pub struct BluetoothPacket {
    pub packet_type: PacketType,
    pub sequence: u8,
    pub length: u8,
    pub data: Vec<u8, 64>,
    pub checksum: u8,
}

impl BluetoothPacket {
    pub fn new(packet_type: PacketType, sequence: u8, data: &[u8]) -> Result<Self, &'static str> {
        let mut packet_data = Vec::new();
        for &byte in data {
            packet_data.push(byte).map_err(|_| "Data too long")?;
        }
        
        let checksum = Self::calculate_checksum(packet_type, sequence, data);
        
        Ok(Self {
            packet_type,
            sequence,
            length: data.len() as u8,
            data: packet_data,
            checksum,
        })
    }
    
    // 计算校验和
    fn calculate_checksum(packet_type: PacketType, sequence: u8, data: &[u8]) -> u8 {
        let mut checksum = packet_type as u8;
        checksum = checksum.wrapping_add(sequence);
        checksum = checksum.wrapping_add(data.len() as u8);
        
        for &byte in data {
            checksum = checksum.wrapping_add(byte);
        }
        
        !checksum // 取反作为校验和
    }
    
    // 序列化数据包
    pub fn serialize(&self) -> Vec<u8, 128> {
        let mut buffer = Vec::new();
        
        // 包头
        buffer.push(0xAA).ok(); // 同步字节1
        buffer.push(0x55).ok(); // 同步字节2
        
        // 包内容
        buffer.push(self.packet_type as u8).ok();
        buffer.push(self.sequence).ok();
        buffer.push(self.length).ok();
        
        // 数据
        for &byte in &self.data {
            buffer.push(byte).ok();
        }
        
        // 校验和
        buffer.push(self.checksum).ok();
        
        buffer
    }
    
    // 反序列化数据包
    pub fn deserialize(data: &[u8]) -> Result<Self, &'static str> {
        if data.len() < 6 {
            return Err("Packet too short");
        }
        
        // 检查同步字节
        if data[0] != 0xAA || data[1] != 0x55 {
            return Err("Invalid sync bytes");
        }
        
        let packet_type = match data[2] {
            0x01 => PacketType::Data,
            0x02 => PacketType::Ack,
            0x03 => PacketType::Nack,
            0x04 => PacketType::Heartbeat,
            _ => return Err("Invalid packet type"),
        };
        
        let sequence = data[3];
        let length = data[4];
        
        if data.len() < (5 + length + 1) as usize {
            return Err("Incomplete packet");
        }
        
        let mut packet_data = Vec::new();
        for i in 0..length {
            packet_data.push(data[5 + i as usize]).map_err(|_| "Data too long")?;
        }
        
        let received_checksum = data[5 + length as usize];
        let calculated_checksum = Self::calculate_checksum(packet_type, sequence, &packet_data);
        
        if received_checksum != calculated_checksum {
            return Err("Checksum mismatch");
        }
        
        Ok(Self {
            packet_type,
            sequence,
            length,
            data: packet_data,
            checksum: received_checksum,
        })
    }
    
    // 验证数据包
    pub fn is_valid(&self) -> bool {
        let calculated_checksum = Self::calculate_checksum(self.packet_type, self.sequence, &self.data);
        calculated_checksum == self.checksum
    }
}

// 蓝牙传输协议
pub struct BluetoothProtocol<BT> {
    bluetooth: BT,
    tx_sequence: u8,
    rx_sequence: u8,
    retry_count: u8,
    max_retries: u8,
}

impl<BT> BluetoothProtocol<BT>
where
    BT: BluetoothModule,
{
    pub fn new(bluetooth: BT) -> Self {
        Self {
            bluetooth,
            tx_sequence: 0,
            rx_sequence: 0,
            retry_count: 0,
            max_retries: 3,
        }
    }
    
    // 发送数据包
    pub fn send_packet(&mut self, data: &[u8]) -> Result<(), &'static str> {
        let packet = BluetoothPacket::new(PacketType::Data, self.tx_sequence, data)?;
        let serialized = packet.serialize();
        
        for retry in 0..=self.max_retries {
            // 发送数据包
            self.bluetooth.send_data(&serialized)?;
            
            // 等待ACK
            if let Ok(ack_packet) = self.receive_packet_with_timeout(1000) {
                if ack_packet.packet_type == PacketType::Ack && 
                   ack_packet.sequence == self.tx_sequence {
                    self.tx_sequence = self.tx_sequence.wrapping_add(1);
                    return Ok(());
                }
            }
            
            if retry < self.max_retries {
                // 重试延时
                cortex_m::asm::delay(84000 * 100); // 100ms
            }
        }
        
        Err("Send packet failed after retries")
    }
    
    // 接收数据包
    pub fn receive_packet(&mut self) -> Result<Vec<u8, 64>, &'static str> {
        let packet = self.receive_packet_with_timeout(5000)?;
        
        if packet.packet_type == PacketType::Data {
            // 发送ACK
            let ack_packet = BluetoothPacket::new(PacketType::Ack, packet.sequence, &[])?;
            let ack_serialized = ack_packet.serialize();
            self.bluetooth.send_data(&ack_serialized)?;
            
            // 检查序列号
            if packet.sequence == self.rx_sequence {
                self.rx_sequence = self.rx_sequence.wrapping_add(1);
                return Ok(packet.data);
            } else {
                return Err("Sequence number mismatch");
            }
        }
        
        Err("Not a data packet")
    }
    
    // 带超时的接收数据包
    fn receive_packet_with_timeout(&mut self, timeout_ms: u32) -> Result<BluetoothPacket, &'static str> {
        let mut buffer = Vec::<u8, 128>::new();
        
        for _ in 0..timeout_ms {
            let received_data = self.bluetooth.receive_data()?;
            
            for &byte in &received_data {
                if buffer.push(byte).is_err() {
                    buffer.clear(); // 缓冲区满，重新开始
                }
                
                // 查找完整的数据包
                if let Some(packet) = self.find_complete_packet(&buffer)? {
                    return Ok(packet);
                }
            }
            
            cortex_m::asm::delay(84000); // 1ms延时
        }
        
        Err("Receive timeout")
    }
    
    // 查找完整的数据包
    fn find_complete_packet(&self, buffer: &[u8]) -> Result<Option<BluetoothPacket>, &'static str> {
        // 查找同步字节
        for i in 0..buffer.len().saturating_sub(5) {
            if buffer[i] == 0xAA && buffer[i + 1] == 0x55 {
                // 检查是否有足够的数据
                if i + 5 < buffer.len() {
                    let length = buffer[i + 4];
                    let packet_size = 6 + length as usize;
                    
                    if i + packet_size <= buffer.len() {
                        // 尝试反序列化
                        if let Ok(packet) = BluetoothPacket::deserialize(&buffer[i..i + packet_size]) {
                            return Ok(Some(packet));
                        }
                    }
                }
            }
        }
        
        Ok(None)
    }
    
    // 发送心跳包
    pub fn send_heartbeat(&mut self) -> Result<(), &'static str> {
        let packet = BluetoothPacket::new(PacketType::Heartbeat, 0, &[])?;
        let serialized = packet.serialize();
        self.bluetooth.send_data(&serialized)
    }
    
    // 检查连接状态
    pub fn is_connected(&self) -> bool {
        self.bluetooth.is_connected()
    }
}
```

## 总结

蓝牙通信为嵌入式设备提供了便捷的近场无线连接能力：

1. **经典蓝牙**: 适合高数据量传输应用
2. **低功耗蓝牙**: 适合传感器和物联网应用
3. **数据协议**: 实现可靠的数据传输机制
4. **设备配对**: 支持安全的设备连接
5. **实时通信**: 低延迟的数据交换

在实际应用中，需要根据具体需求选择合适的蓝牙技术和实现方案，考虑功耗、传输距离、数据量等因素。