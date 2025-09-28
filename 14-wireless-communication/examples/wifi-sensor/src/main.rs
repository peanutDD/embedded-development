#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
  adc::{config::AdcConfig, Adc},
  gpio::{Alternate, Input, Output, Pin, PullUp, PushPull},
  i2c::I2c,
  pac,
  prelude::*,
  rcc::RccExt,
  serial::{Config, Serial},
  timer::Timer,
};

use heapless::{String, Vec};
use nb::block;

// WiFi传感器系统配置
#[derive(Debug, Clone)]
pub struct SensorConfig {
  pub device_id: String<32>,
  pub wifi_ssid: String<64>,
  pub wifi_password: String<64>,
  pub server_url: String<128>,
  pub api_key: String<64>,
  pub sample_interval: u32, // 采样间隔(ms)
  pub upload_interval: u32, // 上传间隔(ms)
  pub sensor_count: u8,
}

impl Default for SensorConfig {
  fn default() -> Self {
    let mut config = Self {
      device_id: String::new(),
      wifi_ssid: String::new(),
      wifi_password: String::new(),
      server_url: String::new(),
      api_key: String::new(),
      sample_interval: 1000,  // 1秒采样
      upload_interval: 60000, // 1分钟上传
      sensor_count: 4,
    };

    config.device_id.push_str("SENSOR_001").ok();
    config.wifi_ssid.push_str("MyWiFi").ok();
    config.wifi_password.push_str("password123").ok();
    config
      .server_url
      .push_str("http://api.example.com/sensors")
      .ok();
    config.api_key.push_str("your_api_key_here").ok();

    config
  }
}

// 传感器数据结构
#[derive(Debug, Clone)]
pub struct SensorData {
  pub timestamp: u32,
  pub temperature: f32,
  pub humidity: f32,
  pub pressure: f32,
  pub light: u16,
  pub battery_voltage: f32,
}

impl Default for SensorData {
  fn default() -> Self {
    Self {
      timestamp: 0,
      temperature: 0.0,
      humidity: 0.0,
      pressure: 0.0,
      light: 0,
      battery_voltage: 0.0,
    }
  }
}

// WiFi模块结构
pub struct WiFiModule<UART> {
  uart: UART,
  connected: bool,
  ip_address: String<16>,
  buffer: Vec<u8, 512>,
}

impl<UART> WiFiModule<UART>
where
  UART: embedded_hal::serial::Read<u8> + embedded_hal::serial::Write<u8>,
{
  pub fn new(uart: UART) -> Self {
    Self {
      uart,
      connected: false,
      ip_address: String::new(),
      buffer: Vec::new(),
    }
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
    for _ in 0..3000 {
      // 3秒超时
      if let Ok(byte) = self.uart.read() {
        if self.buffer.push(byte).is_err() {
          break;
        }

        // 检查是否收到完整响应
        if self.buffer.len() >= 4 {
          let len = self.buffer.len();
          if self.buffer[len - 4..len] == [b'\r', b'\n', b'O', b'K']
            || self.buffer[len - 4..len] == [b'F', b'A', b'I', b'L']
          {
            // 转换为字符串
            for &b in &self.buffer {
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

  // 初始化WiFi模块
  pub fn initialize(&mut self) -> Result<(), &'static str> {
    // 测试连接
    self.send_at_command("AT")?;

    // 设置WiFi模式为Station
    self.send_at_command("AT+CWMODE=1")?;

    // 启用多连接
    self.send_at_command("AT+CIPMUX=1")?;

    Ok(())
  }

  // 连接WiFi网络
  pub fn connect_wifi(&mut self, ssid: &str, password: &str) -> Result<(), &'static str> {
    // 构建连接命令
    let mut command = String::<128>::new();
    command
      .push_str("AT+CWJAP=\"")
      .map_err(|_| "Command too long")?;
    command.push_str(ssid).map_err(|_| "Command too long")?;
    command.push_str("\",\"").map_err(|_| "Command too long")?;
    command.push_str(password).map_err(|_| "Command too long")?;
    command.push('"').map_err(|_| "Command too long")?;

    let response = self.send_at_command(&command)?;

    if response.contains("OK") {
      self.connected = true;

      // 获取IP地址
      let ip_response = self.send_at_command("AT+CIFSR")?;
      if let Some(start) = ip_response.find("STAIP,\"") {
        let ip_start = start + 7;
        if let Some(end) = ip_response[ip_start..].find('"') {
          let ip_str = &ip_response[ip_start..ip_start + end];
          self.ip_address.clear();
          self
            .ip_address
            .push_str(ip_str)
            .map_err(|_| "IP too long")?;
        }
      }

      Ok(())
    } else {
      Err("Failed to connect WiFi")
    }
  }

  // 断开WiFi连接
  pub fn disconnect_wifi(&mut self) -> Result<(), &'static str> {
    self.send_at_command("AT+CWQAP")?;
    self.connected = false;
    self.ip_address.clear();
    Ok(())
  }

  // 建立TCP连接
  pub fn connect_tcp(&mut self, host: &str, port: u16) -> Result<u8, &'static str> {
    let mut command = String::<128>::new();
    command
      .push_str("AT+CIPSTART=0,\"TCP\",\"")
      .map_err(|_| "Command too long")?;
    command.push_str(host).map_err(|_| "Command too long")?;
    command.push_str("\",").map_err(|_| "Command too long")?;

    // 添加端口号
    let port_str = format_number(port as u32);
    command
      .push_str(&port_str)
      .map_err(|_| "Command too long")?;

    let response = self.send_at_command(&command)?;

    if response.contains("OK") {
      Ok(0) // 返回连接ID
    } else {
      Err("Failed to connect TCP")
    }
  }

  // 发送TCP数据
  pub fn send_tcp_data(&mut self, conn_id: u8, data: &[u8]) -> Result<(), &'static str> {
    // 准备发送命令
    let mut command = String::<64>::new();
    command
      .push_str("AT+CIPSEND=")
      .map_err(|_| "Command too long")?;
    command
      .push((b'0' + conn_id) as char)
      .map_err(|_| "Command too long")?;
    command.push(',').map_err(|_| "Command too long")?;

    let data_len = format_number(data.len() as u32);
    command
      .push_str(&data_len)
      .map_err(|_| "Command too long")?;

    // 发送长度命令
    self.send_at_command(&command)?;

    // 等待">"提示符
    for _ in 0..1000 {
      if let Ok(byte) = self.uart.read() {
        if byte == b'>' {
          break;
        }
      }
      cortex_m::asm::delay(84000); // 1ms延时
    }

    // 发送实际数据
    for &byte in data {
      block!(self.uart.write(byte)).map_err(|_| "UART write error")?;
    }

    // 等待发送完成
    for _ in 0..3000 {
      if let Ok(byte) = self.uart.read() {
        if self.buffer.push(byte).is_err() {
          self.buffer.clear();
        }

        // 检查发送结果
        if self.buffer.len() >= 11 {
          let buffer_str = core::str::from_utf8(&self.buffer).unwrap_or("");
          if buffer_str.contains("SEND OK") {
            return Ok(());
          } else if buffer_str.contains("SEND FAIL") {
            return Err("Send failed");
          }
        }
      }
      cortex_m::asm::delay(84000); // 1ms延时
    }

    Err("Send timeout")
  }

  // 关闭TCP连接
  pub fn close_tcp(&mut self, conn_id: u8) -> Result<(), &'static str> {
    let mut command = String::<32>::new();
    command
      .push_str("AT+CIPCLOSE=")
      .map_err(|_| "Command too long")?;
    command
      .push((b'0' + conn_id) as char)
      .map_err(|_| "Command too long")?;

    self.send_at_command(&command)?;
    Ok(())
  }

  // 检查连接状态
  pub fn is_connected(&self) -> bool {
    self.connected
  }

  // 获取IP地址
  pub fn get_ip_address(&self) -> &str {
    &self.ip_address
  }
}

// HTTP客户端
pub struct HttpClient<WIFI> {
  wifi: WIFI,
  user_agent: String<64>,
}

impl<WIFI> HttpClient<WIFI>
where
  WIFI: WiFiModule,
{
  pub fn new(wifi: WIFI) -> Self {
    let mut user_agent = String::new();
    user_agent.push_str("WiFiSensor/1.0").ok();

    Self { wifi, user_agent }
  }

  // 发送GET请求
  pub fn get(&mut self, url: &str) -> Result<String<512>, &'static str> {
    // 解析URL（简化版）
    let (host, path) = self.parse_url(url)?;

    // 建立连接
    let conn_id = self.wifi.connect_tcp(&host, 80)?;

    // 构建HTTP请求
    let mut request = String::<256>::new();
    request.push_str("GET ").map_err(|_| "Request too long")?;
    request.push_str(&path).map_err(|_| "Request too long")?;
    request
      .push_str(" HTTP/1.1\r\n")
      .map_err(|_| "Request too long")?;
    request.push_str("Host: ").map_err(|_| "Request too long")?;
    request.push_str(&host).map_err(|_| "Request too long")?;
    request.push_str("\r\n").map_err(|_| "Request too long")?;
    request
      .push_str("User-Agent: ")
      .map_err(|_| "Request too long")?;
    request
      .push_str(&self.user_agent)
      .map_err(|_| "Request too long")?;
    request.push_str("\r\n").map_err(|_| "Request too long")?;
    request
      .push_str("Connection: close\r\n")
      .map_err(|_| "Request too long")?;
    request.push_str("\r\n").map_err(|_| "Request too long")?;

    // 发送请求
    self.wifi.send_tcp_data(conn_id, request.as_bytes())?;

    // 接收响应（简化版）
    let response = String::new(); // 实际实现需要接收和解析HTTP响应

    // 关闭连接
    self.wifi.close_tcp(conn_id)?;

    Ok(response)
  }

  // 发送POST请求
  pub fn post(
    &mut self,
    url: &str,
    data: &str,
    content_type: &str,
  ) -> Result<String<512>, &'static str> {
    // 解析URL
    let (host, path) = self.parse_url(url)?;

    // 建立连接
    let conn_id = self.wifi.connect_tcp(&host, 80)?;

    // 构建HTTP请求
    let mut request = String::<512>::new();
    request.push_str("POST ").map_err(|_| "Request too long")?;
    request.push_str(&path).map_err(|_| "Request too long")?;
    request
      .push_str(" HTTP/1.1\r\n")
      .map_err(|_| "Request too long")?;
    request.push_str("Host: ").map_err(|_| "Request too long")?;
    request.push_str(&host).map_err(|_| "Request too long")?;
    request.push_str("\r\n").map_err(|_| "Request too long")?;
    request
      .push_str("User-Agent: ")
      .map_err(|_| "Request too long")?;
    request
      .push_str(&self.user_agent)
      .map_err(|_| "Request too long")?;
    request.push_str("\r\n").map_err(|_| "Request too long")?;
    request
      .push_str("Content-Type: ")
      .map_err(|_| "Request too long")?;
    request
      .push_str(content_type)
      .map_err(|_| "Request too long")?;
    request.push_str("\r\n").map_err(|_| "Request too long")?;
    request
      .push_str("Content-Length: ")
      .map_err(|_| "Request too long")?;

    let data_len = format_number(data.len() as u32);
    request
      .push_str(&data_len)
      .map_err(|_| "Request too long")?;
    request.push_str("\r\n").map_err(|_| "Request too long")?;
    request
      .push_str("Connection: close\r\n")
      .map_err(|_| "Request too long")?;
    request.push_str("\r\n").map_err(|_| "Request too long")?;
    request.push_str(data).map_err(|_| "Request too long")?;

    // 发送请求
    self.wifi.send_tcp_data(conn_id, request.as_bytes())?;

    // 接收响应
    let response = String::new(); // 实际实现需要接收和解析HTTP响应

    // 关闭连接
    self.wifi.close_tcp(conn_id)?;

    Ok(response)
  }

  // 解析URL（简化版）
  fn parse_url(&self, url: &str) -> Result<(String<64>, String<128>), &'static str> {
    // 移除http://前缀
    let url_without_protocol = if url.starts_with("http://") {
      &url[7..]
    } else {
      url
    };

    // 查找第一个'/'分隔主机和路径
    if let Some(slash_pos) = url_without_protocol.find('/') {
      let host_str = &url_without_protocol[..slash_pos];
      let path_str = &url_without_protocol[slash_pos..];

      let mut host = String::new();
      let mut path = String::new();

      host.push_str(host_str).map_err(|_| "Host too long")?;
      path.push_str(path_str).map_err(|_| "Path too long")?;

      Ok((host, path))
    } else {
      let mut host = String::new();
      let mut path = String::new();

      host
        .push_str(url_without_protocol)
        .map_err(|_| "Host too long")?;
      path.push('/').map_err(|_| "Path too long")?;

      Ok((host, path))
    }
  }
}

// 传感器管理器
pub struct SensorManager<ADC> {
  adc: ADC,
  last_sample_time: u32,
  sample_count: u32,
  data_buffer: Vec<SensorData, 32>,
}

impl<ADC> SensorManager<ADC>
where
  ADC: embedded_hal::adc::OneShot<stm32f4xx_hal::pac::ADC1, u16, Pin<'A', 0, Analog>>,
{
  pub fn new(adc: ADC) -> Self {
    Self {
      adc,
      last_sample_time: 0,
      sample_count: 0,
      data_buffer: Vec::new(),
    }
  }

  // 读取传感器数据
  pub fn read_sensors(&mut self, timestamp: u32) -> Result<SensorData, &'static str> {
    let mut data = SensorData::default();
    data.timestamp = timestamp;

    // 读取温度传感器（模拟）
    data.temperature = self.read_temperature()?;

    // 读取湿度传感器（模拟）
    data.humidity = self.read_humidity()?;

    // 读取气压传感器（模拟）
    data.pressure = self.read_pressure()?;

    // 读取光照传感器
    data.light = self.read_light_sensor()?;

    // 读取电池电压
    data.battery_voltage = self.read_battery_voltage()?;

    self.sample_count += 1;
    self.last_sample_time = timestamp;

    Ok(data)
  }

  // 读取温度（模拟实现）
  fn read_temperature(&mut self) -> Result<f32, &'static str> {
    // 实际实现中应该读取真实的温度传感器
    // 这里返回模拟值
    Ok(25.0 + (self.sample_count % 10) as f32 * 0.5)
  }

  // 读取湿度（模拟实现）
  fn read_humidity(&mut self) -> Result<f32, &'static str> {
    // 实际实现中应该读取真实的湿度传感器
    Ok(60.0 + (self.sample_count % 20) as f32 * 1.0)
  }

  // 读取气压（模拟实现）
  fn read_pressure(&mut self) -> Result<f32, &'static str> {
    // 实际实现中应该读取真实的气压传感器
    Ok(1013.25 + (self.sample_count % 5) as f32 * 2.0)
  }

  // 读取光照传感器
  fn read_light_sensor(&mut self) -> Result<u16, &'static str> {
    // 实际实现中应该通过ADC读取光敏电阻
    // 这里返回模拟值
    Ok(500 + (self.sample_count % 100) as u16 * 10)
  }

  // 读取电池电压
  fn read_battery_voltage(&mut self) -> Result<f32, &'static str> {
    // 实际实现中应该通过ADC读取电池电压
    // 这里返回模拟值
    Ok(3.7 - (self.sample_count % 50) as f32 * 0.01)
  }

  // 添加数据到缓冲区
  pub fn add_data(&mut self, data: SensorData) -> Result<(), &'static str> {
    if self.data_buffer.is_full() {
      // 移除最旧的数据
      self.data_buffer.remove(0);
    }

    self.data_buffer.push(data).map_err(|_| "Buffer full")?;
    Ok(())
  }

  // 获取缓冲区数据
  pub fn get_buffered_data(&self) -> &[SensorData] {
    &self.data_buffer
  }

  // 清空缓冲区
  pub fn clear_buffer(&mut self) {
    self.data_buffer.clear();
  }

  // 获取统计信息
  pub fn get_statistics(&self) -> SensorStatistics {
    if self.data_buffer.is_empty() {
      return SensorStatistics::default();
    }

    let mut stats = SensorStatistics::default();
    stats.sample_count = self.data_buffer.len() as u32;

    // 计算平均值
    let mut temp_sum = 0.0;
    let mut humidity_sum = 0.0;
    let mut pressure_sum = 0.0;

    for data in &self.data_buffer {
      temp_sum += data.temperature;
      humidity_sum += data.humidity;
      pressure_sum += data.pressure;
    }

    let count = self.data_buffer.len() as f32;
    stats.avg_temperature = temp_sum / count;
    stats.avg_humidity = humidity_sum / count;
    stats.avg_pressure = pressure_sum / count;

    stats
  }
}

// 传感器统计信息
#[derive(Debug, Clone)]
pub struct SensorStatistics {
  pub sample_count: u32,
  pub avg_temperature: f32,
  pub avg_humidity: f32,
  pub avg_pressure: f32,
}

impl Default for SensorStatistics {
  fn default() -> Self {
    Self {
      sample_count: 0,
      avg_temperature: 0.0,
      avg_humidity: 0.0,
      avg_pressure: 0.0,
    }
  }
}

// WiFi传感器系统
pub struct WiFiSensorSystem<WIFI, SENSORS> {
  config: SensorConfig,
  wifi: WIFI,
  sensors: SENSORS,
  system_time: u32,
  last_upload_time: u32,
  status_led: Pin<'C', 13, Output<PushPull>>,
  error_count: u32,
}

impl<WIFI, SENSORS> WiFiSensorSystem<WIFI, SENSORS>
where
  WIFI: WiFiModule,
  SENSORS: SensorManager,
{
  pub fn new(
    config: SensorConfig,
    wifi: WIFI,
    sensors: SENSORS,
    status_led: Pin<'C', 13, Output<PushPull>>,
  ) -> Self {
    Self {
      config,
      wifi,
      sensors,
      system_time: 0,
      last_upload_time: 0,
      status_led,
      error_count: 0,
    }
  }

  // 系统初始化
  pub fn initialize(&mut self) -> Result<(), &'static str> {
    // 初始化WiFi模块
    self.wifi.initialize()?;

    // 连接WiFi网络
    self
      .wifi
      .connect_wifi(&self.config.wifi_ssid, &self.config.wifi_password)?;

    // 指示灯闪烁表示初始化完成
    for _ in 0..3 {
      self.status_led.set_high();
      cortex_m::asm::delay(84000 * 200); // 200ms
      self.status_led.set_low();
      cortex_m::asm::delay(84000 * 200); // 200ms
    }

    Ok(())
  }

  // 主运行循环
  pub fn run(&mut self) -> ! {
    loop {
      // 更新系统时间
      self.system_time += 1;

      // 检查是否需要采样
      if self.system_time % self.config.sample_interval == 0 {
        if let Err(_) = self.sample_sensors() {
          self.error_count += 1;
        }
      }

      // 检查是否需要上传数据
      if self.system_time - self.last_upload_time >= self.config.upload_interval {
        if let Err(_) = self.upload_data() {
          self.error_count += 1;
        } else {
          self.last_upload_time = self.system_time;
        }
      }

      // 更新状态指示
      self.update_status_led();

      // 延时1ms
      cortex_m::asm::delay(84000);
    }
  }

  // 采样传感器数据
  fn sample_sensors(&mut self) -> Result<(), &'static str> {
    let data = self.sensors.read_sensors(self.system_time)?;
    self.sensors.add_data(data)?;
    Ok(())
  }

  // 上传数据到服务器
  fn upload_data(&mut self) -> Result<(), &'static str> {
    if !self.wifi.is_connected() {
      return Err("WiFi not connected");
    }

    let buffered_data = self.sensors.get_buffered_data();
    if buffered_data.is_empty() {
      return Ok(()); // 没有数据需要上传
    }

    // 构建JSON数据
    let json_data = self.build_json_payload(buffered_data)?;

    // 创建HTTP客户端
    let mut http_client = HttpClient::new(&mut self.wifi);

    // 发送POST请求
    let response = http_client.post(&self.config.server_url, &json_data, "application/json")?;

    // 检查响应（简化版）
    if response.contains("200 OK") {
      // 上传成功，清空缓冲区
      self.sensors.clear_buffer();
      Ok(())
    } else {
      Err("Upload failed")
    }
  }

  // 构建JSON数据
  fn build_json_payload(&self, data: &[SensorData]) -> Result<String<1024>, &'static str> {
    let mut json = String::new();

    json
      .push_str("{\"device_id\":\"")
      .map_err(|_| "JSON too long")?;
    json
      .push_str(&self.config.device_id)
      .map_err(|_| "JSON too long")?;
    json
      .push_str("\",\"data\":[")
      .map_err(|_| "JSON too long")?;

    for (i, sensor_data) in data.iter().enumerate() {
      if i > 0 {
        json.push(',').map_err(|_| "JSON too long")?;
      }

      json.push('{').map_err(|_| "JSON too long")?;
      json
        .push_str("\"timestamp\":")
        .map_err(|_| "JSON too long")?;
      json
        .push_str(&format_number(sensor_data.timestamp))
        .map_err(|_| "JSON too long")?;
      json
        .push_str(",\"temperature\":")
        .map_err(|_| "JSON too long")?;
      json
        .push_str(&format_float(sensor_data.temperature))
        .map_err(|_| "JSON too long")?;
      json
        .push_str(",\"humidity\":")
        .map_err(|_| "JSON too long")?;
      json
        .push_str(&format_float(sensor_data.humidity))
        .map_err(|_| "JSON too long")?;
      json
        .push_str(",\"pressure\":")
        .map_err(|_| "JSON too long")?;
      json
        .push_str(&format_float(sensor_data.pressure))
        .map_err(|_| "JSON too long")?;
      json.push_str(",\"light\":").map_err(|_| "JSON too long")?;
      json
        .push_str(&format_number(sensor_data.light as u32))
        .map_err(|_| "JSON too long")?;
      json
        .push_str(",\"battery\":")
        .map_err(|_| "JSON too long")?;
      json
        .push_str(&format_float(sensor_data.battery_voltage))
        .map_err(|_| "JSON too long")?;
      json.push('}').map_err(|_| "JSON too long")?;
    }

    json.push_str("]}").map_err(|_| "JSON too long")?;

    Ok(json)
  }

  // 更新状态LED
  fn update_status_led(&mut self) {
    if self.wifi.is_connected() {
      // WiFi连接正常，LED慢闪
      if (self.system_time / 1000) % 2 == 0 {
        self.status_led.set_high();
      } else {
        self.status_led.set_low();
      }
    } else {
      // WiFi未连接，LED快闪
      if (self.system_time / 250) % 2 == 0 {
        self.status_led.set_high();
      } else {
        self.status_led.set_low();
      }
    }
  }

  // 获取系统状态
  pub fn get_system_status(&self) -> SystemStatus {
    SystemStatus {
      wifi_connected: self.wifi.is_connected(),
      ip_address: self.wifi.get_ip_address().into(),
      uptime: self.system_time,
      error_count: self.error_count,
      sensor_stats: self.sensors.get_statistics(),
    }
  }
}

// 系统状态
#[derive(Debug)]
pub struct SystemStatus {
  pub wifi_connected: bool,
  pub ip_address: String<16>,
  pub uptime: u32,
  pub error_count: u32,
  pub sensor_stats: SensorStatistics,
}

// 数字格式化函数
fn format_number(num: u32) -> String<16> {
  let mut result = String::new();
  let mut n = num;

  if n == 0 {
    result.push('0').ok();
    return result;
  }

  let mut digits = Vec::<u8, 16>::new();
  while n > 0 {
    digits.push((n % 10) as u8).ok();
    n /= 10;
  }

  for &digit in digits.iter().rev() {
    result.push((b'0' + digit) as char).ok();
  }

  result
}

// 浮点数格式化函数（简化版）
fn format_float(num: f32) -> String<16> {
  let integer_part = num as u32;
  let fractional_part = ((num - integer_part as f32) * 100.0) as u32;

  let mut result = format_number(integer_part);
  result.push('.').ok();

  if fractional_part < 10 {
    result.push('0').ok();
  }
  let frac_str = format_number(fractional_part);
  result.push_str(&frac_str).ok();

  result
}

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 状态LED (PC13)
  let status_led = gpioc.pc13.into_push_pull_output();

  // UART配置 (PA9/PA10 - USART1)
  let tx_pin = gpioa.pa9.into_alternate();
  let rx_pin = gpioa.pa10.into_alternate();

  let serial = Serial::new(
    dp.USART1,
    (tx_pin, rx_pin),
    Config::default().baudrate(115200.bps()),
    &clocks,
  )
  .unwrap();

  // ADC配置
  let adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());

  // 创建系统组件
  let config = SensorConfig::default();
  let wifi = WiFiModule::new(serial);
  let sensors = SensorManager::new(adc);

  // 创建系统
  let mut system = WiFiSensorSystem::new(config, wifi, sensors, status_led);

  // 初始化系统
  if let Err(_) = system.initialize() {
    // 初始化失败，进入错误状态
    loop {
      // 错误指示：快速闪烁
      cortex_m::asm::delay(84000 * 100); // 100ms
    }
  }

  // 运行主循环
  system.run()
}
