# WiFi通信基础

本文档介绍在STM32F4微控制器上实现WiFi通信的基础知识和实现方法。

## WiFi通信概述

WiFi（Wireless Fidelity）是一种基于IEEE 802.11标准的无线局域网技术，广泛应用于物联网设备的网络连接。

### 主要特点
- **高速传输**: 支持Mbps级别的数据传输
- **标准化协议**: 基于TCP/IP协议栈
- **广泛兼容**: 与现有网络基础设施兼容
- **灵活配置**: 支持多种网络拓扑结构

### 应用场景
- 物联网设备远程监控
- 传感器数据云端上传
- 设备固件远程更新
- 实时数据可视化

## ESP8266 WiFi模块

ESP8266是最常用的WiFi模块之一，具有成本低、功耗小、集成度高的特点。

### 硬件特性
- **处理器**: 32位RISC处理器
- **WiFi标准**: 802.11 b/g/n
- **工作频率**: 2.4GHz
- **传输速率**: 最高150Mbps
- **工作电压**: 3.0V-3.6V
- **功耗**: 待机<1mA，工作80mA

### 接口类型
- **UART接口**: AT命令控制
- **SPI接口**: 高速数据传输
- **SDIO接口**: 高性能应用

## AT命令控制

AT命令是控制ESP8266最简单的方式，通过UART串口发送标准化命令。

### 基本AT命令实现

```rust
use stm32f4xx_hal::{
    prelude::*,
    pac,
    serial::{Serial, Config},
};
use heapless::{String, Vec};
use nb::block;

// WiFi模块结构
pub struct WiFiModule<UART> {
    uart: UART,
    buffer: Vec<u8, 256>,
    connected: bool,
    ip_address: String<16>,
}

impl<UART> WiFiModule<UART>
where
    UART: embedded_hal::serial::Read<u8> + embedded_hal::serial::Write<u8>,
{
    pub fn new(uart: UART) -> Self {
        Self {
            uart,
            buffer: Vec::new(),
            connected: false,
            ip_address: String::new(),
        }
    }
    
    // 发送AT命令
    pub fn send_command(&mut self, command: &str) -> Result<(), &'static str> {
        // 发送命令
        for byte in command.bytes() {
            block!(self.uart.write(byte)).map_err(|_| "UART write error")?;
        }
        
        // 发送回车换行
        block!(self.uart.write(b'\r')).map_err(|_| "UART write error")?;
        block!(self.uart.write(b'\n')).map_err(|_| "UART write error")?;
        
        Ok(())
    }
    
    // 读取响应
    pub fn read_response(&mut self, timeout_ms: u32) -> Result<String<256>, &'static str> {
        self.buffer.clear();
        let mut response = String::new();
        
        for _ in 0..timeout_ms {
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
            
            // 简单延时1ms
            cortex_m::asm::delay(84000); // 假设84MHz时钟
        }
        
        Err("Response timeout")
    }
    
    // 测试连接
    pub fn test_connection(&mut self) -> Result<bool, &'static str> {
        self.send_command("AT")?;
        let response = self.read_response(1000)?;
        Ok(response.contains("OK"))
    }
    
    // 设置WiFi模式
    pub fn set_wifi_mode(&mut self, mode: WiFiMode) -> Result<(), &'static str> {
        let command = match mode {
            WiFiMode::Station => "AT+CWMODE=1",
            WiFiMode::AccessPoint => "AT+CWMODE=2",
            WiFiMode::Both => "AT+CWMODE=3",
        };
        
        self.send_command(command)?;
        let response = self.read_response(2000)?;
        
        if response.contains("OK") {
            Ok(())
        } else {
            Err("Failed to set WiFi mode")
        }
    }
    
    // 连接WiFi网络
    pub fn connect_wifi(&mut self, ssid: &str, password: &str) -> Result<(), &'static str> {
        // 构建连接命令
        let mut command = String::<128>::new();
        command.push_str("AT+CWJAP=\"").map_err(|_| "Command too long")?;
        command.push_str(ssid).map_err(|_| "Command too long")?;
        command.push_str("\",\"").map_err(|_| "Command too long")?;
        command.push_str(password).map_err(|_| "Command too long")?;
        command.push('"').map_err(|_| "Command too long")?;
        
        self.send_command(&command)?;
        let response = self.read_response(10000)?; // 连接可能需要较长时间
        
        if response.contains("OK") {
            self.connected = true;
            Ok(())
        } else {
            Err("Failed to connect WiFi")
        }
    }
    
    // 获取IP地址
    pub fn get_ip_address(&mut self) -> Result<String<16>, &'static str> {
        self.send_command("AT+CIFSR")?;
        let response = self.read_response(2000)?;
        
        // 解析IP地址（简化版）
        if let Some(start) = response.find("STAIP,\"") {
            let ip_start = start + 7;
            if let Some(end) = response[ip_start..].find('"') {
                let ip_str = &response[ip_start..ip_start + end];
                let mut ip = String::new();
                ip.push_str(ip_str).map_err(|_| "IP address too long")?;
                self.ip_address = ip.clone();
                return Ok(ip);
            }
        }
        
        Err("Failed to get IP address")
    }
    
    // 建立TCP连接
    pub fn connect_tcp(&mut self, host: &str, port: u16) -> Result<(), &'static str> {
        let mut command = String::<128>::new();
        command.push_str("AT+CIPSTART=\"TCP\",\"").map_err(|_| "Command too long")?;
        command.push_str(host).map_err(|_| "Command too long")?;
        command.push_str("\",").map_err(|_| "Command too long")?;
        
        // 添加端口号（简化的数字转换）
        let mut port_str = String::<8>::new();
        let mut temp_port = port;
        if temp_port == 0 {
            port_str.push('0').map_err(|_| "Port conversion error")?;
        } else {
            let mut digits = Vec::<u8, 8>::new();
            while temp_port > 0 {
                digits.push((temp_port % 10) as u8).map_err(|_| "Too many digits")?;
                temp_port /= 10;
            }
            for &digit in digits.iter().rev() {
                port_str.push((b'0' + digit) as char).map_err(|_| "Port conversion error")?;
            }
        }
        
        command.push_str(&port_str).map_err(|_| "Command too long")?;
        
        self.send_command(&command)?;
        let response = self.read_response(5000)?;
        
        if response.contains("OK") {
            Ok(())
        } else {
            Err("Failed to connect TCP")
        }
    }
    
    // 发送数据
    pub fn send_data(&mut self, data: &[u8]) -> Result<(), &'static str> {
        // 发送数据长度命令
        let mut command = String::<32>::new();
        command.push_str("AT+CIPSEND=").map_err(|_| "Command too long")?;
        
        // 添加数据长度
        let mut len_str = String::<8>::new();
        let mut temp_len = data.len();
        if temp_len == 0 {
            len_str.push('0').map_err(|_| "Length conversion error")?;
        } else {
            let mut digits = Vec::<u8, 8>::new();
            while temp_len > 0 {
                digits.push((temp_len % 10) as u8).map_err(|_| "Too many digits")?;
                temp_len /= 10;
            }
            for &digit in digits.iter().rev() {
                len_str.push((b'0' + digit) as char).map_err(|_| "Length conversion error")?;
            }
        }
        
        command.push_str(&len_str).map_err(|_| "Command too long")?;
        
        self.send_command(&command)?;
        let response = self.read_response(2000)?;
        
        if response.contains(">") {
            // 发送实际数据
            for &byte in data {
                block!(self.uart.write(byte)).map_err(|_| "UART write error")?;
            }
            
            let response = self.read_response(5000)?;
            if response.contains("SEND OK") {
                Ok(())
            } else {
                Err("Failed to send data")
            }
        } else {
            Err("Failed to enter send mode")
        }
    }
    
    // 接收数据
    pub fn receive_data(&mut self) -> Result<Vec<u8, 256>, &'static str> {
        let mut data = Vec::new();
        
        // 等待接收数据指示
        let response = self.read_response(5000)?;
        
        if response.contains("+IPD,") {
            // 解析数据长度和内容（简化版）
            if let Some(start) = response.find("+IPD,") {
                // 这里需要更复杂的解析逻辑
                // 简化处理，假设数据在响应中
                for byte in response.bytes().skip(start + 5) {
                    if data.push(byte).is_err() {
                        break;
                    }
                }
            }
        }
        
        Ok(data)
    }
    
    // 断开连接
    pub fn disconnect(&mut self) -> Result<(), &'static str> {
        self.send_command("AT+CIPCLOSE")?;
        let response = self.read_response(2000)?;
        
        if response.contains("OK") {
            Ok(())
        } else {
            Err("Failed to disconnect")
        }
    }
    
    // 重启模块
    pub fn reset(&mut self) -> Result<(), &'static str> {
        self.send_command("AT+RST")?;
        let response = self.read_response(3000)?;
        
        if response.contains("OK") {
            self.connected = false;
            self.ip_address.clear();
            Ok(())
        } else {
            Err("Failed to reset")
        }
    }
}

// WiFi工作模式
#[derive(Debug, Clone, Copy)]
pub enum WiFiMode {
    Station,      // 客户端模式
    AccessPoint,  // 接入点模式
    Both,         // 混合模式
}
```

## HTTP客户端实现

基于AT命令实现简单的HTTP客户端功能。

```rust
use heapless::{String, Vec};

// HTTP客户端
pub struct HttpClient<WIFI> {
    wifi: WIFI,
    host: String<64>,
    port: u16,
}

impl<WIFI> HttpClient<WIFI>
where
    WIFI: WiFiModule,
{
    pub fn new(wifi: WIFI) -> Self {
        Self {
            wifi,
            host: String::new(),
            port: 80,
        }
    }
    
    // 设置服务器
    pub fn set_server(&mut self, host: &str, port: u16) -> Result<(), &'static str> {
        self.host.clear();
        self.host.push_str(host).map_err(|_| "Host name too long")?;
        self.port = port;
        Ok(())
    }
    
    // 发送GET请求
    pub fn get(&mut self, path: &str) -> Result<String<512>, &'static str> {
        // 连接服务器
        self.wifi.connect_tcp(&self.host, self.port)?;
        
        // 构建HTTP请求
        let mut request = String::<256>::new();
        request.push_str("GET ").map_err(|_| "Request too long")?;
        request.push_str(path).map_err(|_| "Request too long")?;
        request.push_str(" HTTP/1.1\r\n").map_err(|_| "Request too long")?;
        request.push_str("Host: ").map_err(|_| "Request too long")?;
        request.push_str(&self.host).map_err(|_| "Request too long")?;
        request.push_str("\r\n").map_err(|_| "Request too long")?;
        request.push_str("Connection: close\r\n").map_err(|_| "Request too long")?;
        request.push_str("\r\n").map_err(|_| "Request too long")?;
        
        // 发送请求
        self.wifi.send_data(request.as_bytes())?;
        
        // 接收响应
        let response_data = self.wifi.receive_data()?;
        
        // 转换为字符串
        let mut response = String::new();
        for &byte in &response_data {
            if response.push(byte as char).is_err() {
                break;
            }
        }
        
        // 断开连接
        self.wifi.disconnect()?;
        
        Ok(response)
    }
    
    // 发送POST请求
    pub fn post(&mut self, path: &str, data: &str) -> Result<String<512>, &'static str> {
        // 连接服务器
        self.wifi.connect_tcp(&self.host, self.port)?;
        
        // 构建HTTP请求
        let mut request = String::<512>::new();
        request.push_str("POST ").map_err(|_| "Request too long")?;
        request.push_str(path).map_err(|_| "Request too long")?;
        request.push_str(" HTTP/1.1\r\n").map_err(|_| "Request too long")?;
        request.push_str("Host: ").map_err(|_| "Request too long")?;
        request.push_str(&self.host).map_err(|_| "Request too long")?;
        request.push_str("\r\n").map_err(|_| "Request too long")?;
        request.push_str("Content-Type: application/json\r\n").map_err(|_| "Request too long")?;
        request.push_str("Content-Length: ").map_err(|_| "Request too long")?;
        
        // 添加内容长度
        let mut len_str = String::<8>::new();
        let mut temp_len = data.len();
        if temp_len == 0 {
            len_str.push('0').map_err(|_| "Length conversion error")?;
        } else {
            let mut digits = Vec::<u8, 8>::new();
            while temp_len > 0 {
                digits.push((temp_len % 10) as u8).map_err(|_| "Too many digits")?;
                temp_len /= 10;
            }
            for &digit in digits.iter().rev() {
                len_str.push((b'0' + digit) as char).map_err(|_| "Length conversion error")?;
            }
        }
        
        request.push_str(&len_str).map_err(|_| "Request too long")?;
        request.push_str("\r\n").map_err(|_| "Request too long")?;
        request.push_str("Connection: close\r\n").map_err(|_| "Request too long")?;
        request.push_str("\r\n").map_err(|_| "Request too long")?;
        request.push_str(data).map_err(|_| "Request too long")?;
        
        // 发送请求
        self.wifi.send_data(request.as_bytes())?;
        
        // 接收响应
        let response_data = self.wifi.receive_data()?;
        
        // 转换为字符串
        let mut response = String::new();
        for &byte in &response_data {
            if response.push(byte as char).is_err() {
                break;
            }
        }
        
        // 断开连接
        self.wifi.disconnect()?;
        
        Ok(response)
    }
}
```

## MQTT客户端实现

MQTT是物联网应用中常用的轻量级消息传输协议。

```rust
use heapless::{String, Vec};

// MQTT客户端
pub struct MqttClient<WIFI> {
    wifi: WIFI,
    broker_host: String<64>,
    broker_port: u16,
    client_id: String<32>,
    connected: bool,
}

impl<WIFI> MqttClient<WIFI>
where
    WIFI: WiFiModule,
{
    pub fn new(wifi: WIFI, client_id: &str) -> Result<Self, &'static str> {
        let mut id = String::new();
        id.push_str(client_id).map_err(|_| "Client ID too long")?;
        
        Ok(Self {
            wifi,
            broker_host: String::new(),
            broker_port: 1883,
            client_id: id,
            connected: false,
        })
    }
    
    // 设置MQTT代理
    pub fn set_broker(&mut self, host: &str, port: u16) -> Result<(), &'static str> {
        self.broker_host.clear();
        self.broker_host.push_str(host).map_err(|_| "Host name too long")?;
        self.broker_port = port;
        Ok(())
    }
    
    // 连接MQTT代理
    pub fn connect(&mut self) -> Result<(), &'static str> {
        // 连接TCP
        self.wifi.connect_tcp(&self.broker_host, self.broker_port)?;
        
        // 构建MQTT CONNECT包（简化版）
        let mut connect_packet = Vec::<u8, 128>::new();
        
        // 固定头部
        connect_packet.push(0x10).map_err(|_| "Packet too long")?; // CONNECT
        connect_packet.push(0x00).map_err(|_| "Packet too long")?; // 长度占位符
        
        // 可变头部
        // 协议名称
        connect_packet.push(0x00).map_err(|_| "Packet too long")?;
        connect_packet.push(0x04).map_err(|_| "Packet too long")?;
        connect_packet.push(b'M').map_err(|_| "Packet too long")?;
        connect_packet.push(b'Q').map_err(|_| "Packet too long")?;
        connect_packet.push(b'T').map_err(|_| "Packet too long")?;
        connect_packet.push(b'T').map_err(|_| "Packet too long")?;
        
        // 协议版本
        connect_packet.push(0x04).map_err(|_| "Packet too long")?;
        
        // 连接标志
        connect_packet.push(0x02).map_err(|_| "Packet too long")?; // Clean session
        
        // 保持连接时间
        connect_packet.push(0x00).map_err(|_| "Packet too long")?;
        connect_packet.push(0x3C).map_err(|_| "Packet too long")?; // 60秒
        
        // 载荷 - 客户端ID
        let client_id_len = self.client_id.len() as u16;
        connect_packet.push((client_id_len >> 8) as u8).map_err(|_| "Packet too long")?;
        connect_packet.push((client_id_len & 0xFF) as u8).map_err(|_| "Packet too long")?;
        
        for byte in self.client_id.bytes() {
            connect_packet.push(byte).map_err(|_| "Packet too long")?;
        }
        
        // 更新长度字段
        let remaining_length = connect_packet.len() - 2;
        connect_packet[1] = remaining_length as u8;
        
        // 发送CONNECT包
        self.wifi.send_data(&connect_packet)?;
        
        // 等待CONNACK
        let response = self.wifi.receive_data()?;
        
        if response.len() >= 4 && response[0] == 0x20 && response[3] == 0x00 {
            self.connected = true;
            Ok(())
        } else {
            Err("MQTT connection failed")
        }
    }
    
    // 发布消息
    pub fn publish(&mut self, topic: &str, message: &str) -> Result<(), &'static str> {
        if !self.connected {
            return Err("Not connected to MQTT broker");
        }
        
        // 构建MQTT PUBLISH包（简化版）
        let mut publish_packet = Vec::<u8, 256>::new();
        
        // 固定头部
        publish_packet.push(0x30).map_err(|_| "Packet too long")?; // PUBLISH, QoS 0
        publish_packet.push(0x00).map_err(|_| "Packet too long")?; // 长度占位符
        
        // 可变头部 - 主题名称
        let topic_len = topic.len() as u16;
        publish_packet.push((topic_len >> 8) as u8).map_err(|_| "Packet too long")?;
        publish_packet.push((topic_len & 0xFF) as u8).map_err(|_| "Packet too long")?;
        
        for byte in topic.bytes() {
            publish_packet.push(byte).map_err(|_| "Packet too long")?;
        }
        
        // 载荷 - 消息内容
        for byte in message.bytes() {
            publish_packet.push(byte).map_err(|_| "Packet too long")?;
        }
        
        // 更新长度字段
        let remaining_length = publish_packet.len() - 2;
        publish_packet[1] = remaining_length as u8;
        
        // 发送PUBLISH包
        self.wifi.send_data(&publish_packet)?;
        
        Ok(())
    }
    
    // 订阅主题
    pub fn subscribe(&mut self, topic: &str) -> Result<(), &'static str> {
        if !self.connected {
            return Err("Not connected to MQTT broker");
        }
        
        // 构建MQTT SUBSCRIBE包（简化版）
        let mut subscribe_packet = Vec::<u8, 128>::new();
        
        // 固定头部
        subscribe_packet.push(0x82).map_err(|_| "Packet too long")?; // SUBSCRIBE
        subscribe_packet.push(0x00).map_err(|_| "Packet too long")?; // 长度占位符
        
        // 可变头部 - 包标识符
        subscribe_packet.push(0x00).map_err(|_| "Packet too long")?;
        subscribe_packet.push(0x01).map_err(|_| "Packet too long")?;
        
        // 载荷 - 主题过滤器
        let topic_len = topic.len() as u16;
        subscribe_packet.push((topic_len >> 8) as u8).map_err(|_| "Packet too long")?;
        subscribe_packet.push((topic_len & 0xFF) as u8).map_err(|_| "Packet too long")?;
        
        for byte in topic.bytes() {
            subscribe_packet.push(byte).map_err(|_| "Packet too long")?;
        }
        
        // QoS级别
        subscribe_packet.push(0x00).map_err(|_| "Packet too long")?;
        
        // 更新长度字段
        let remaining_length = subscribe_packet.len() - 2;
        subscribe_packet[1] = remaining_length as u8;
        
        // 发送SUBSCRIBE包
        self.wifi.send_data(&subscribe_packet)?;
        
        // 等待SUBACK
        let response = self.wifi.receive_data()?;
        
        if response.len() >= 5 && response[0] == 0x90 {
            Ok(())
        } else {
            Err("MQTT subscription failed")
        }
    }
    
    // 断开连接
    pub fn disconnect(&mut self) -> Result<(), &'static str> {
        if self.connected {
            // 发送DISCONNECT包
            let disconnect_packet = [0xE0, 0x00];
            self.wifi.send_data(&disconnect_packet)?;
            
            self.connected = false;
        }
        
        self.wifi.disconnect()
    }
}
```

## 总结

WiFi通信为嵌入式设备提供了强大的网络连接能力，通过ESP8266等WiFi模块，可以轻松实现：

1. **基础连接**: AT命令控制WiFi连接
2. **HTTP通信**: RESTful API调用和数据传输
3. **MQTT通信**: 轻量级消息传输协议
4. **实时数据**: 传感器数据云端上传
5. **远程控制**: 设备状态监控和控制

在实际应用中，需要考虑网络稳定性、功耗管理、安全性等因素，选择合适的通信协议和实现方案。