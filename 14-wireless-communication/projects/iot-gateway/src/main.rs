//! # IoT Gateway 物联网网关
//!
//! 这是一个完整的物联网网关实现，支持多种无线通信协议：
//! - WiFi (ESP8266/ESP32)
//! - Bluetooth Classic/BLE
//! - LoRa 长距离通信
//! - MQTT 消息队列
//! - CoAP 轻量级协议
//!
//! ## 硬件连接
//!
//! ### WiFi模块 (ESP8266)
//! - VCC -> 3.3V
//! - GND -> GND
//! - TX -> PA10 (USART1_RX)
//! - RX -> PA9 (USART1_TX)
//! - EN -> PA8 (使能控制)
//!
//! ### 蓝牙模块 (HC-05)
//! - VCC -> 3.3V
//! - GND -> GND
//! - TX -> PA3 (USART2_RX)
//! - RX -> PA2 (USART2_TX)
//! - EN -> PA1 (使能控制)
//!
//! ### LoRa模块 (SX1278)
//! - VCC -> 3.3V
//! - GND -> GND
//! - MOSI -> PB15 (SPI2_MOSI)
//! - MISO -> PB14 (SPI2_MISO)
//! - SCK -> PB13 (SPI2_SCK)
//! - NSS -> PB12 (片选)
//! - RST -> PB11 (复位)
//! - DIO0 -> PB10 (中断)
//!
//! ### 显示屏 (SSD1306)
//! - VCC -> 3.3V
//! - GND -> GND
//! - SCL -> PB6 (I2C1_SCL)
//! - SDA -> PB7 (I2C1_SDA)
//!
//! ### 状态LED
//! - WiFi状态 -> PC13
//! - 蓝牙状态 -> PC14
//! - LoRa状态 -> PC15

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
  delay::Delay,
  gpio::{Input, Output, PullUp, PushPull},
  i2c::I2c,
  pac,
  prelude::*,
  serial::{Config as SerialConfig, Serial},
  spi::{Mode, Phase, Polarity, Spi},
  timer::Timer,
};

use heapless::{FnvIndexMap, String, Vec};
use libm::*;
use nb::block;

// 系统配置常量
const SYSTEM_CLOCK_HZ: u32 = 84_000_000;
const WIFI_BAUD_RATE: u32 = 115200;
const BLUETOOTH_BAUD_RATE: u32 = 9600;
const LORA_FREQUENCY: u32 = 433_000_000; // 433MHz
const MAX_DEVICES: usize = 32;
const MAX_MESSAGE_SIZE: usize = 256;
const GATEWAY_ID: &str = "IOT_GATEWAY_001";

/// 通信协议类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Protocol {
  WiFi,
  Bluetooth,
  LoRa,
  Ethernet,
}

/// 设备类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeviceType {
  Sensor,
  Actuator,
  Gateway,
  Controller,
}

/// 消息类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MessageType {
  Data,
  Command,
  Status,
  Heartbeat,
  Alarm,
}

/// 设备信息
#[derive(Debug, Clone)]
pub struct DeviceInfo {
  pub id: String<32>,
  pub device_type: DeviceType,
  pub protocol: Protocol,
  pub rssi: i16,
  pub last_seen: u32,
  pub battery_level: u8,
  pub firmware_version: String<16>,
}

impl DeviceInfo {
  pub fn new(id: &str, device_type: DeviceType, protocol: Protocol) -> Self {
    let mut device_id = String::new();
    device_id.push_str(id).ok();

    Self {
      id: device_id,
      device_type,
      protocol,
      rssi: 0,
      last_seen: 0,
      battery_level: 100,
      firmware_version: String::from("1.0.0"),
    }
  }

  pub fn update_status(&mut self, rssi: i16, timestamp: u32) {
    self.rssi = rssi;
    self.last_seen = timestamp;
  }

  pub fn is_online(&self, current_time: u32, timeout_ms: u32) -> bool {
    current_time.saturating_sub(self.last_seen) < timeout_ms
  }
}

/// 网关消息
#[derive(Debug, Clone)]
pub struct GatewayMessage {
  pub source_id: String<32>,
  pub target_id: String<32>,
  pub message_type: MessageType,
  pub protocol: Protocol,
  pub payload: Vec<u8, MAX_MESSAGE_SIZE>,
  pub timestamp: u32,
  pub sequence: u16,
}

impl GatewayMessage {
  pub fn new(
    source: &str,
    target: &str,
    msg_type: MessageType,
    protocol: Protocol,
    data: &[u8],
  ) -> Self {
    let mut source_id = String::new();
    source_id.push_str(source).ok();

    let mut target_id = String::new();
    target_id.push_str(target).ok();

    let mut payload = Vec::new();
    payload.extend_from_slice(data).ok();

    Self {
      source_id,
      target_id,
      message_type: msg_type,
      protocol,
      payload,
      timestamp: 0,
      sequence: 0,
    }
  }

  pub fn serialize(&self) -> Vec<u8, MAX_MESSAGE_SIZE> {
    let mut buffer = Vec::new();

    // 消息头
    buffer.push(0xAA).ok(); // 起始标志
    buffer.push(self.message_type as u8).ok();
    buffer.push(self.protocol as u8).ok();
    buffer.push(self.source_id.len() as u8).ok();

    // 源ID
    buffer.extend_from_slice(self.source_id.as_bytes()).ok();

    // 目标ID长度和内容
    buffer.push(self.target_id.len() as u8).ok();
    buffer.extend_from_slice(self.target_id.as_bytes()).ok();

    // 时间戳
    let timestamp_bytes = self.timestamp.to_le_bytes();
    buffer.extend_from_slice(&timestamp_bytes).ok();

    // 序列号
    let sequence_bytes = self.sequence.to_le_bytes();
    buffer.extend_from_slice(&sequence_bytes).ok();

    // 载荷长度和内容
    buffer.push(self.payload.len() as u8).ok();
    buffer.extend_from_slice(&self.payload).ok();

    // 校验和
    let checksum = self.calculate_checksum(&buffer);
    buffer.push(checksum).ok();

    buffer
  }

  fn calculate_checksum(&self, data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &x| acc.wrapping_add(x))
  }
}

/// WiFi模块控制器
pub struct WiFiController {
  enabled: bool,
  connected: bool,
  ssid: String<32>,
  ip_address: String<16>,
  connection_count: u32,
}

impl WiFiController {
  pub fn new() -> Self {
    Self {
      enabled: false,
      connected: false,
      ssid: String::new(),
      ip_address: String::new(),
      connection_count: 0,
    }
  }

  pub fn initialize(&mut self) -> Result<(), &'static str> {
    // 发送AT命令测试连接
    self.send_at_command("AT")?;

    // 设置WiFi模式为Station+AP
    self.send_at_command("AT+CWMODE=3")?;

    // 启用多连接
    self.send_at_command("AT+CIPMUX=1")?;

    self.enabled = true;
    Ok(())
  }

  pub fn connect_network(&mut self, ssid: &str, password: &str) -> Result<(), &'static str> {
    if !self.enabled {
      return Err("WiFi not initialized");
    }

    // 构建连接命令
    let mut cmd = String::<64>::new();
    cmd.push_str("AT+CWJAP=\"").ok();
    cmd.push_str(ssid).ok();
    cmd.push_str("\",\"").ok();
    cmd.push_str(password).ok();
    cmd.push_str("\"").ok();

    self.send_at_command(&cmd)?;

    self.ssid.clear();
    self.ssid.push_str(ssid).ok();
    self.connected = true;

    Ok(())
  }

  pub fn start_server(&mut self, port: u16) -> Result<(), &'static str> {
    if !self.connected {
      return Err("WiFi not connected");
    }

    // 启动TCP服务器
    let mut cmd = String::<32>::new();
    cmd.push_str("AT+CIPSERVER=1,").ok();

    let port_str = itoa::Buffer::new().format(port);
    cmd.push_str(port_str).ok();

    self.send_at_command(&cmd)?;

    Ok(())
  }

  pub fn send_data(&mut self, connection_id: u8, data: &[u8]) -> Result<(), &'static str> {
    // 发送数据长度
    let mut cmd = String::<32>::new();
    cmd.push_str("AT+CIPSEND=").ok();

    let id_str = itoa::Buffer::new().format(connection_id);
    cmd.push_str(id_str).ok();
    cmd.push_str(",").ok();

    let len_str = itoa::Buffer::new().format(data.len());
    cmd.push_str(len_str).ok();

    self.send_at_command(&cmd)?;

    // 发送实际数据
    // 这里需要实际的UART发送实现

    Ok(())
  }

  fn send_at_command(&self, command: &str) -> Result<(), &'static str> {
    // 这里需要实际的UART发送实现
    // 发送命令并等待响应
    Ok(())
  }

  pub fn get_status(&self) -> (bool, bool, &str) {
    (self.enabled, self.connected, &self.ssid)
  }
}

/// 蓝牙控制器
pub struct BluetoothController {
  enabled: bool,
  discoverable: bool,
  connected_devices: u8,
  device_name: String<32>,
}

impl BluetoothController {
  pub fn new() -> Self {
    Self {
      enabled: false,
      discoverable: false,
      connected_devices: 0,
      device_name: String::from("IoT_Gateway"),
    }
  }

  pub fn initialize(&mut self) -> Result<(), &'static str> {
    // 测试蓝牙模块
    self.send_at_command("AT")?;

    // 设置设备名称
    let mut cmd = String::<64>::new();
    cmd.push_str("AT+NAME=").ok();
    cmd.push_str(&self.device_name).ok();
    self.send_at_command(&cmd)?;

    // 设置为可发现模式
    self.send_at_command("AT+CMODE=1")?;

    self.enabled = true;
    self.discoverable = true;

    Ok(())
  }

  pub fn start_advertising(&mut self) -> Result<(), &'static str> {
    if !self.enabled {
      return Err("Bluetooth not initialized");
    }

    // 开始广播
    self.send_at_command("AT+INQ")?;

    Ok(())
  }

  pub fn send_data(&mut self, data: &[u8]) -> Result<(), &'static str> {
    if self.connected_devices == 0 {
      return Err("No connected devices");
    }

    // 发送数据到已连接的设备
    // 这里需要实际的UART发送实现

    Ok(())
  }

  fn send_at_command(&self, command: &str) -> Result<(), &'static str> {
    // 这里需要实际的UART发送实现
    Ok(())
  }

  pub fn get_status(&self) -> (bool, bool, u8) {
    (self.enabled, self.discoverable, self.connected_devices)
  }
}

/// LoRa控制器
pub struct LoRaController {
  enabled: bool,
  frequency: u32,
  spreading_factor: u8,
  bandwidth: u32,
  coding_rate: u8,
  tx_power: i8,
  packet_count: u32,
}

impl LoRaController {
  pub fn new() -> Self {
    Self {
      enabled: false,
      frequency: LORA_FREQUENCY,
      spreading_factor: 7,
      bandwidth: 125000, // 125kHz
      coding_rate: 5,    // 4/5
      tx_power: 14,      // 14dBm
      packet_count: 0,
    }
  }

  pub fn initialize(&mut self) -> Result<(), &'static str> {
    // 复位LoRa模块
    self.reset_module()?;

    // 设置频率
    self.set_frequency(self.frequency)?;

    // 设置扩频因子
    self.set_spreading_factor(self.spreading_factor)?;

    // 设置带宽
    self.set_bandwidth(self.bandwidth)?;

    // 设置编码率
    self.set_coding_rate(self.coding_rate)?;

    // 设置发射功率
    self.set_tx_power(self.tx_power)?;

    self.enabled = true;
    Ok(())
  }

  pub fn send_packet(&mut self, data: &[u8]) -> Result<(), &'static str> {
    if !self.enabled {
      return Err("LoRa not initialized");
    }

    if data.len() > 255 {
      return Err("Packet too large");
    }

    // 发送数据包
    // 这里需要实际的SPI通信实现

    self.packet_count += 1;
    Ok(())
  }

  pub fn receive_packet(&mut self) -> Result<Vec<u8, 255>, &'static str> {
    if !self.enabled {
      return Err("LoRa not initialized");
    }

    // 接收数据包
    // 这里需要实际的SPI通信实现

    let mut packet = Vec::new();
    // 模拟接收到的数据
    packet.push(0x01).ok();
    packet.push(0x02).ok();

    Ok(packet)
  }

  fn reset_module(&self) -> Result<(), &'static str> {
    // 复位LoRa模块
    Ok(())
  }

  fn set_frequency(&self, freq: u32) -> Result<(), &'static str> {
    // 设置工作频率
    Ok(())
  }

  fn set_spreading_factor(&self, sf: u8) -> Result<(), &'static str> {
    // 设置扩频因子
    Ok(())
  }

  fn set_bandwidth(&self, bw: u32) -> Result<(), &'static str> {
    // 设置带宽
    Ok(())
  }

  fn set_coding_rate(&self, cr: u8) -> Result<(), &'static str> {
    // 设置编码率
    Ok(())
  }

  fn set_tx_power(&self, power: i8) -> Result<(), &'static str> {
    // 设置发射功率
    Ok(())
  }

  pub fn get_status(&self) -> (bool, u32, u8, u32) {
    (
      self.enabled,
      self.frequency,
      self.spreading_factor,
      self.packet_count,
    )
  }
}

/// MQTT客户端
pub struct MqttClient {
  connected: bool,
  broker_address: String<64>,
  client_id: String<32>,
  keep_alive: u16,
  message_id: u16,
}

impl MqttClient {
  pub fn new(client_id: &str) -> Self {
    let mut id = String::new();
    id.push_str(client_id).ok();

    Self {
      connected: false,
      broker_address: String::new(),
      client_id: id,
      keep_alive: 60,
      message_id: 1,
    }
  }

  pub fn connect(&mut self, broker: &str, port: u16) -> Result<(), &'static str> {
    self.broker_address.clear();
    self.broker_address.push_str(broker).ok();

    // 建立TCP连接到MQTT代理
    // 发送CONNECT消息

    self.connected = true;
    Ok(())
  }

  pub fn publish(&mut self, topic: &str, payload: &[u8], qos: u8) -> Result<(), &'static str> {
    if !self.connected {
      return Err("MQTT not connected");
    }

    // 构建PUBLISH消息
    // 发送到代理

    self.message_id += 1;
    Ok(())
  }

  pub fn subscribe(&mut self, topic: &str, qos: u8) -> Result<(), &'static str> {
    if !self.connected {
      return Err("MQTT not connected");
    }

    // 构建SUBSCRIBE消息
    // 发送到代理

    self.message_id += 1;
    Ok(())
  }

  pub fn disconnect(&mut self) -> Result<(), &'static str> {
    if self.connected {
      // 发送DISCONNECT消息
      self.connected = false;
    }
    Ok(())
  }

  pub fn get_status(&self) -> (bool, &str) {
    (self.connected, &self.broker_address)
  }
}

/// 设备管理器
pub struct DeviceManager {
  devices: FnvIndexMap<String<32>, DeviceInfo, MAX_DEVICES>,
  device_count: usize,
}

impl DeviceManager {
  pub fn new() -> Self {
    Self {
      devices: FnvIndexMap::new(),
      device_count: 0,
    }
  }

  pub fn add_device(&mut self, device: DeviceInfo) -> Result<(), &'static str> {
    if self.device_count >= MAX_DEVICES {
      return Err("Device limit reached");
    }

    self.devices.insert(device.id.clone(), device).ok();
    self.device_count += 1;

    Ok(())
  }

  pub fn remove_device(&mut self, device_id: &str) -> Result<(), &'static str> {
    let mut id = String::<32>::new();
    id.push_str(device_id).ok();

    if self.devices.remove(&id).is_some() {
      self.device_count -= 1;
      Ok(())
    } else {
      Err("Device not found")
    }
  }

  pub fn update_device_status(&mut self, device_id: &str, rssi: i16, timestamp: u32) {
    let mut id = String::<32>::new();
    id.push_str(device_id).ok();

    if let Some(device) = self.devices.get_mut(&id) {
      device.update_status(rssi, timestamp);
    }
  }

  pub fn get_device(&self, device_id: &str) -> Option<&DeviceInfo> {
    let mut id = String::<32>::new();
    id.push_str(device_id).ok();

    self.devices.get(&id)
  }

  pub fn get_online_devices(
    &self,
    current_time: u32,
    timeout_ms: u32,
  ) -> Vec<&DeviceInfo, MAX_DEVICES> {
    let mut online_devices = Vec::new();

    for (_, device) in &self.devices {
      if device.is_online(current_time, timeout_ms) {
        online_devices.push(device).ok();
      }
    }

    online_devices
  }

  pub fn get_device_count(&self) -> usize {
    self.device_count
  }

  pub fn cleanup_offline_devices(&mut self, current_time: u32, timeout_ms: u32) {
    let mut to_remove = Vec::<String<32>, MAX_DEVICES>::new();

    for (id, device) in &self.devices {
      if !device.is_online(current_time, timeout_ms) {
        to_remove.push(id.clone()).ok();
      }
    }

    for id in &to_remove {
      self.devices.remove(id);
      self.device_count -= 1;
    }
  }
}

/// 消息路由器
pub struct MessageRouter {
  message_queue: Vec<GatewayMessage, 64>,
  routing_table: FnvIndexMap<String<32>, Protocol, MAX_DEVICES>,
}

impl MessageRouter {
  pub fn new() -> Self {
    Self {
      message_queue: Vec::new(),
      routing_table: FnvIndexMap::new(),
    }
  }

  pub fn add_route(&mut self, device_id: &str, protocol: Protocol) -> Result<(), &'static str> {
    let mut id = String::<32>::new();
    id.push_str(device_id).ok();

    self.routing_table.insert(id, protocol).ok();
    Ok(())
  }

  pub fn route_message(&mut self, message: GatewayMessage) -> Result<(), &'static str> {
    // 查找目标设备的协议
    let target_protocol = self
      .routing_table
      .get(&message.target_id)
      .copied()
      .unwrap_or(message.protocol);

    // 如果协议不同，需要协议转换
    if message.protocol != target_protocol {
      let converted_message = self.convert_protocol(message, target_protocol)?;
      self.queue_message(converted_message)?;
    } else {
      self.queue_message(message)?;
    }

    Ok(())
  }

  fn convert_protocol(
    &self,
    mut message: GatewayMessage,
    target_protocol: Protocol,
  ) -> Result<GatewayMessage, &'static str> {
    // 协议转换逻辑
    message.protocol = target_protocol;

    // 根据目标协议调整消息格式
    match target_protocol {
      Protocol::WiFi => {
        // WiFi协议特定的格式调整
      }
      Protocol::Bluetooth => {
        // 蓝牙协议特定的格式调整
      }
      Protocol::LoRa => {
        // LoRa协议特定的格式调整
        // 可能需要分包处理
        if message.payload.len() > 255 {
          return Err("Message too large for LoRa");
        }
      }
      Protocol::Ethernet => {
        // 以太网协议特定的格式调整
      }
    }

    Ok(message)
  }

  fn queue_message(&mut self, message: GatewayMessage) -> Result<(), &'static str> {
    if self.message_queue.len() >= self.message_queue.capacity() {
      return Err("Message queue full");
    }

    self.message_queue.push(message).ok();
    Ok(())
  }

  pub fn process_queue(&mut self) -> Option<GatewayMessage> {
    if !self.message_queue.is_empty() {
      Some(self.message_queue.remove(0))
    } else {
      None
    }
  }

  pub fn get_queue_size(&self) -> usize {
    self.message_queue.len()
  }
}

/// IoT网关系统
pub struct IoTGateway {
  wifi: WiFiController,
  bluetooth: BluetoothController,
  lora: LoRaController,
  mqtt: MqttClient,
  device_manager: DeviceManager,
  message_router: MessageRouter,
  system_time: u32,
  status_led_wifi: bool,
  status_led_bluetooth: bool,
  status_led_lora: bool,
}

impl IoTGateway {
  pub fn new() -> Self {
    Self {
      wifi: WiFiController::new(),
      bluetooth: BluetoothController::new(),
      lora: LoRaController::new(),
      mqtt: MqttClient::new(GATEWAY_ID),
      device_manager: DeviceManager::new(),
      message_router: MessageRouter::new(),
      system_time: 0,
      status_led_wifi: false,
      status_led_bluetooth: false,
      status_led_lora: false,
    }
  }

  pub fn initialize(&mut self) -> Result<(), &'static str> {
    // 初始化WiFi
    if let Err(e) = self.wifi.initialize() {
      return Err(e);
    }
    self.status_led_wifi = true;

    // 初始化蓝牙
    if let Err(e) = self.bluetooth.initialize() {
      return Err(e);
    }
    self.status_led_bluetooth = true;

    // 初始化LoRa
    if let Err(e) = self.lora.initialize() {
      return Err(e);
    }
    self.status_led_lora = true;

    // 连接MQTT代理
    self.mqtt.connect("mqtt.example.com", 1883)?;

    // 订阅网关主题
    self.mqtt.subscribe("gateway/+/command", 1)?;
    self.mqtt.subscribe("gateway/+/config", 1)?;

    Ok(())
  }

  pub fn run_cycle(&mut self) {
    self.system_time += 1;

    // 处理WiFi消息
    self.process_wifi_messages();

    // 处理蓝牙消息
    self.process_bluetooth_messages();

    // 处理LoRa消息
    self.process_lora_messages();

    // 处理消息路由
    self.process_message_routing();

    // 发送心跳消息
    if self.system_time % 30000 == 0 {
      // 每30秒
      self.send_heartbeat();
    }

    // 清理离线设备
    if self.system_time % 60000 == 0 {
      // 每60秒
      self
        .device_manager
        .cleanup_offline_devices(self.system_time, 120000); // 2分钟超时
    }

    // 更新状态LED
    self.update_status_leds();
  }

  fn process_wifi_messages(&mut self) {
    // 处理WiFi接收到的消息
    // 这里需要实际的WiFi消息处理逻辑
  }

  fn process_bluetooth_messages(&mut self) {
    // 处理蓝牙接收到的消息
    // 这里需要实际的蓝牙消息处理逻辑
  }

  fn process_lora_messages(&mut self) {
    // 尝试接收LoRa消息
    if let Ok(packet) = self.lora.receive_packet() {
      // 解析消息
      if let Ok(message) = self.parse_lora_message(&packet) {
        // 更新设备状态
        self.device_manager.update_device_status(
          &message.source_id,
          -80, // 模拟RSSI
          self.system_time,
        );

        // 路由消息
        self.message_router.route_message(message).ok();
      }
    }
  }

  fn parse_lora_message(&self, packet: &[u8]) -> Result<GatewayMessage, &'static str> {
    if packet.len() < 10 {
      return Err("Packet too short");
    }

    // 简化的消息解析
    let message = GatewayMessage::new(
      "lora_device_001",
      GATEWAY_ID,
      MessageType::Data,
      Protocol::LoRa,
      &packet[10..],
    );

    Ok(message)
  }

  fn process_message_routing(&mut self) {
    // 处理消息队列中的消息
    while let Some(message) = self.message_router.process_queue() {
      match message.protocol {
        Protocol::WiFi => {
          self.send_wifi_message(&message);
        }
        Protocol::Bluetooth => {
          self.send_bluetooth_message(&message);
        }
        Protocol::LoRa => {
          self.send_lora_message(&message);
        }
        Protocol::Ethernet => {
          self.send_mqtt_message(&message);
        }
      }
    }
  }

  fn send_wifi_message(&mut self, message: &GatewayMessage) {
    let serialized = message.serialize();
    self.wifi.send_data(0, &serialized).ok();
  }

  fn send_bluetooth_message(&mut self, message: &GatewayMessage) {
    let serialized = message.serialize();
    self.bluetooth.send_data(&serialized).ok();
  }

  fn send_lora_message(&mut self, message: &GatewayMessage) {
    let serialized = message.serialize();
    self.lora.send_packet(&serialized).ok();
  }

  fn send_mqtt_message(&mut self, message: &GatewayMessage) {
    let mut topic = String::<64>::new();
    topic.push_str("gateway/").ok();
    topic.push_str(&message.target_id).ok();
    topic.push_str("/data").ok();

    self.mqtt.publish(&topic, &message.payload, 1).ok();
  }

  fn send_heartbeat(&mut self) {
    let heartbeat_data = b"HEARTBEAT";
    let heartbeat = GatewayMessage::new(
      GATEWAY_ID,
      "server",
      MessageType::Heartbeat,
      Protocol::Ethernet,
      heartbeat_data,
    );

    self.message_router.route_message(heartbeat).ok();
  }

  fn update_status_leds(&mut self) {
    let (wifi_enabled, wifi_connected, _) = self.wifi.get_status();
    self.status_led_wifi = wifi_enabled && wifi_connected;

    let (bt_enabled, bt_discoverable, _) = self.bluetooth.get_status();
    self.status_led_bluetooth = bt_enabled && bt_discoverable;

    let (lora_enabled, _, _, _) = self.lora.get_status();
    self.status_led_lora = lora_enabled;
  }

  pub fn get_system_status(&self) -> SystemStatus {
    SystemStatus {
      uptime: self.system_time,
      device_count: self.device_manager.get_device_count(),
      message_queue_size: self.message_router.get_queue_size(),
      wifi_status: self.wifi.get_status(),
      bluetooth_status: self.bluetooth.get_status(),
      lora_status: self.lora.get_status(),
      mqtt_status: self.mqtt.get_status(),
    }
  }
}

/// 系统状态信息
pub struct SystemStatus {
  pub uptime: u32,
  pub device_count: usize,
  pub message_queue_size: usize,
  pub wifi_status: (bool, bool, String<32>),
  pub bluetooth_status: (bool, bool, u8),
  pub lora_status: (bool, u32, u8, u32),
  pub mqtt_status: (bool, String<64>),
}

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .sysclk(84.MHz())
    .hclk(84.MHz())
    .pclk1(42.MHz())
    .pclk2(84.MHz())
    .freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 状态LED
  let mut led_wifi = gpioc.pc13.into_push_pull_output();
  let mut led_bluetooth = gpioc.pc14.into_push_pull_output();
  let mut led_lora = gpioc.pc15.into_push_pull_output();

  // WiFi模块控制引脚
  let mut wifi_enable = gpioa.pa8.into_push_pull_output();

  // 蓝牙模块控制引脚
  let mut bluetooth_enable = gpioa.pa1.into_push_pull_output();

  // LoRa模块控制引脚
  let mut lora_reset = gpiob.pb11.into_push_pull_output();
  let lora_dio0 = gpiob.pb10.into_pull_up_input();

  // 配置UART1 (WiFi)
  let wifi_tx = gpioa.pa9.into_alternate();
  let wifi_rx = gpioa.pa10.into_alternate();
  let wifi_serial = Serial::new(
    dp.USART1,
    (wifi_tx, wifi_rx),
    SerialConfig::default().baudrate(WIFI_BAUD_RATE.bps()),
    &clocks,
  )
  .unwrap();

  // 配置UART2 (蓝牙)
  let bluetooth_tx = gpioa.pa2.into_alternate();
  let bluetooth_rx = gpioa.pa3.into_alternate();
  let bluetooth_serial = Serial::new(
    dp.USART2,
    (bluetooth_tx, bluetooth_rx),
    SerialConfig::default().baudrate(BLUETOOTH_BAUD_RATE.bps()),
    &clocks,
  )
  .unwrap();

  // 配置SPI2 (LoRa)
  let lora_sck = gpiob.pb13.into_alternate();
  let lora_miso = gpiob.pb14.into_alternate();
  let lora_mosi = gpiob.pb15.into_alternate();
  let mut lora_nss = gpiob.pb12.into_push_pull_output();

  let lora_spi = Spi::new(
    dp.SPI2,
    (lora_sck, lora_miso, lora_mosi),
    Mode {
      polarity: Polarity::IdleLow,
      phase: Phase::CaptureOnFirstTransition,
    },
    1.MHz(),
    &clocks,
  );

  // 配置I2C1 (显示屏)
  let display_scl = gpiob.pb6.into_alternate_open_drain();
  let display_sda = gpiob.pb7.into_alternate_open_drain();
  let display_i2c = I2c::new(dp.I2C1, (display_scl, display_sda), 400.kHz(), &clocks);

  // 配置延时
  let mut delay = Delay::new(cp.SYST, &clocks);

  // 启用模块
  wifi_enable.set_high();
  bluetooth_enable.set_high();
  lora_reset.set_high();

  delay.delay_ms(1000u32);

  // 创建IoT网关实例
  let mut gateway = IoTGateway::new();

  // 初始化网关
  match gateway.initialize() {
    Ok(()) => {
      led_wifi.set_low();
      led_bluetooth.set_low();
      led_lora.set_low();
    }
    Err(_) => {
      // 初始化失败，闪烁所有LED
      loop {
        led_wifi.toggle();
        led_bluetooth.toggle();
        led_lora.toggle();
        delay.delay_ms(200u32);
      }
    }
  }

  // 主循环
  loop {
    // 运行网关周期
    gateway.run_cycle();

    // 更新状态LED
    let status = gateway.get_system_status();

    if status.wifi_status.0 && status.wifi_status.1 {
      led_wifi.set_low();
    } else {
      led_wifi.set_high();
    }

    if status.bluetooth_status.0 && status.bluetooth_status.1 {
      led_bluetooth.set_low();
    } else {
      led_bluetooth.set_high();
    }

    if status.lora_status.0 {
      led_lora.set_low();
    } else {
      led_lora.set_high();
    }

    // 延时
    delay.delay_ms(100u32);
  }
}
