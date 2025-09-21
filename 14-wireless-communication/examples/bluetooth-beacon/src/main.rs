#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    pac,
    gpio::{Pin, Output, PushPull, Input, PullUp, Alternate},
    serial::{Serial, Config},
    timer::Timer,
    rcc::RccExt,
};

use heapless::{String, Vec};
use nb::block;

// iBeacon标准UUID
pub const IBEACON_UUID: [u8; 16] = [
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0
];

// 信标配置
#[derive(Debug, Clone)]
pub struct BeaconConfig {
    pub uuid: [u8; 16],
    pub major: u16,
    pub minor: u16,
    pub tx_power: i8,
    pub advertising_interval: u16, // ms
    pub device_name: String<32>,
}

impl Default for BeaconConfig {
    fn default() -> Self {
        let mut device_name = String::new();
        device_name.push_str("MyBeacon").ok();
        
        Self {
            uuid: IBEACON_UUID,
            major: 1,
            minor: 1,
            tx_power: -59, // dBm at 1m
            advertising_interval: 1000, // 1秒
            device_name,
        }
    }
}

// 扫描到的设备信息
#[derive(Debug, Clone)]
pub struct ScannedDevice {
    pub address: String<18>,
    pub name: String<32>,
    pub rssi: i8,
    pub beacon_data: Option<BeaconData>,
    pub last_seen: u32,
}

// 信标数据
#[derive(Debug, Clone)]
pub struct BeaconData {
    pub uuid: [u8; 16],
    pub major: u16,
    pub minor: u16,
    pub tx_power: i8,
    pub distance: f32, // 估算距离(米)
}

// 蓝牙模块结构
pub struct BluetoothModule<UART> {
    uart: UART,
    buffer: Vec<u8, 512>,
    advertising: bool,
    scanning: bool,
    connected: bool,
}

impl<UART> BluetoothModule<UART>
where
    UART: embedded_hal::serial::Read<u8> + embedded_hal::serial::Write<u8>,
{
    pub fn new(uart: UART) -> Self {
        Self {
            uart,
            buffer: Vec::new(),
            advertising: false,
            scanning: false,
            connected: false,
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
        for _ in 0..3000 { // 3秒超时
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
    
    // 初始化BLE模块
    pub fn initialize(&mut self) -> Result<(), &'static str> {
        // 测试连接
        self.send_at_command("AT")?;
        
        // 初始化BLE
        self.send_at_command("AT+BLEINIT=2")?; // BLE服务器模式
        
        // 获取BLE地址
        self.send_at_command("AT+BLEADDR?")?;
        
        Ok(())
    }
    
    // 设置信标广播数据
    pub fn set_beacon_data(&mut self, config: &BeaconConfig) -> Result<(), &'static str> {
        // 构建iBeacon广播数据
        let beacon_data = self.build_ibeacon_data(config)?;
        
        // 设置广播数据
        let mut command = String::<256>::new();
        command.push_str("AT+BLEADVDATA=\"").map_err(|_| "Command too long")?;
        
        // 转换为十六进制字符串
        for &byte in &beacon_data {
            command.push_str(&format_hex_byte(byte)).map_err(|_| "Command too long")?;
        }
        
        command.push('"').map_err(|_| "Command too long")?;
        
        self.send_at_command(&command)?;
        
        Ok(())
    }
    
    // 构建iBeacon广播数据
    fn build_ibeacon_data(&self, config: &BeaconConfig) -> Result<Vec<u8, 64>, &'static str> {
        let mut data = Vec::new();
        
        // 标志字段
        data.push(0x02).map_err(|_| "Data too long")?; // 长度
        data.push(0x01).map_err(|_| "Data too long")?; // 类型：标志
        data.push(0x06).map_err(|_| "Data too long")?; // 值：LE General Discoverable + BR/EDR Not Supported
        
        // iBeacon数据
        data.push(0x1A).map_err(|_| "Data too long")?; // 长度：26字节
        data.push(0xFF).map_err(|_| "Data too long")?; // 类型：厂商特定数据
        
        // Apple公司ID (0x004C)
        data.push(0x4C).map_err(|_| "Data too long")?;
        data.push(0x00).map_err(|_| "Data too long")?;
        
        // iBeacon类型和长度
        data.push(0x02).map_err(|_| "Data too long")?; // iBeacon类型
        data.push(0x15).map_err(|_| "Data too long")?; // 数据长度：21字节
        
        // UUID (16字节)
        for &byte in &config.uuid {
            data.push(byte).map_err(|_| "Data too long")?;
        }
        
        // Major (2字节，大端序)
        data.push((config.major >> 8) as u8).map_err(|_| "Data too long")?;
        data.push((config.major & 0xFF) as u8).map_err(|_| "Data too long")?;
        
        // Minor (2字节，大端序)
        data.push((config.minor >> 8) as u8).map_err(|_| "Data too long")?;
        data.push((config.minor & 0xFF) as u8).map_err(|_| "Data too long")?;
        
        // TX Power (1字节，有符号)
        data.push(config.tx_power as u8).map_err(|_| "Data too long")?;
        
        Ok(data)
    }
    
    // 开始信标广播
    pub fn start_beacon(&mut self, config: &BeaconConfig) -> Result<(), &'static str> {
        // 设置广播数据
        self.set_beacon_data(config)?;
        
        // 设置广播参数
        let mut adv_params = String::<64>::new();
        adv_params.push_str("AT+BLEADVPARAM=").map_err(|_| "Command too long")?;
        adv_params.push_str(&format_number(config.advertising_interval as u32)).map_err(|_| "Command too long")?;
        adv_params.push(',').map_err(|_| "Command too long")?;
        adv_params.push_str(&format_number(config.advertising_interval as u32)).map_err(|_| "Command too long")?;
        adv_params.push_str(",0,0,7,0").map_err(|_| "Command too long")?; // 其他参数
        
        self.send_at_command(&adv_params)?;
        
        // 开始广播
        self.send_at_command("AT+BLEADVSTART")?;
        
        self.advertising = true;
        Ok(())
    }
    
    // 停止信标广播
    pub fn stop_beacon(&mut self) -> Result<(), &'static str> {
        self.send_at_command("AT+BLEADVSTOP")?;
        self.advertising = false;
        Ok(())
    }
    
    // 开始扫描
    pub fn start_scan(&mut self) -> Result<(), &'static str> {
        // 设置扫描参数
        self.send_at_command("AT+BLESCANPARAM=0,0,0x10,0x10,0,0")?;
        
        // 开始扫描
        self.send_at_command("AT+BLESCAN=1")?;
        
        self.scanning = true;
        Ok(())
    }
    
    // 停止扫描
    pub fn stop_scan(&mut self) -> Result<(), &'static str> {
        self.send_at_command("AT+BLESCAN=0")?;
        self.scanning = false;
        Ok(())
    }
    
    // 解析扫描结果
    pub fn parse_scan_result(&mut self) -> Result<Option<ScannedDevice>, &'static str> {
        // 读取UART数据
        let mut scan_data = Vec::<u8, 256>::new();
        
        for _ in 0..100 { // 100ms超时
            if let Ok(byte) = self.uart.read() {
                if scan_data.push(byte).is_err() {
                    break;
                }
                
                // 检查是否收到完整的扫描结果
                if scan_data.len() >= 2 {
                    let len = scan_data.len();
                    if scan_data[len-2] == b'\r' && scan_data[len-1] == b'\n' {
                        // 解析扫描数据
                        return self.parse_scan_data(&scan_data);
                    }
                }
            }
            
            cortex_m::asm::delay(84000); // 1ms延时
        }
        
        Ok(None)
    }
    
    // 解析扫描数据
    fn parse_scan_data(&self, data: &[u8]) -> Result<Option<ScannedDevice>, &'static str> {
        // 转换为字符串
        let data_str = core::str::from_utf8(data).map_err(|_| "Invalid UTF-8")?;
        
        // 查找扫描结果标识
        if let Some(start) = data_str.find("+BLESCAN:") {
            let scan_line = &data_str[start + 9..];
            
            // 解析扫描结果（简化版）
            // 格式：address,rssi,adv_data,scan_rsp_data,addr_type
            let parts: Vec<&str, 8> = scan_line.split(',').collect();
            
            if parts.len() >= 3 {
                let mut device = ScannedDevice {
                    address: String::new(),
                    name: String::new(),
                    rssi: 0,
                    beacon_data: None,
                    last_seen: 0, // 实际应用中应该使用系统时间
                };
                
                // 解析地址
                device.address.push_str(parts[0]).map_err(|_| "Address too long")?;
                
                // 解析RSSI
                if let Ok(rssi) = parts[1].parse::<i8>() {
                    device.rssi = rssi;
                }
                
                // 解析广播数据
                if parts.len() >= 3 {
                    if let Some(beacon_data) = self.parse_beacon_data(parts[2])? {
                        device.beacon_data = Some(beacon_data);
                    }
                }
                
                return Ok(Some(device));
            }
        }
        
        Ok(None)
    }
    
    // 解析信标数据
    fn parse_beacon_data(&self, adv_data: &str) -> Result<Option<BeaconData>, &'static str> {
        // 将十六进制字符串转换为字节数组
        let mut data = Vec::<u8, 64>::new();
        
        let mut chars = adv_data.chars();
        while let (Some(high), Some(low)) = (chars.next(), chars.next()) {
            if let (Some(h), Some(l)) = (hex_char_to_u8(high), hex_char_to_u8(low)) {
                data.push((h << 4) | l).map_err(|_| "Data too long")?;
            }
        }
        
        // 查找iBeacon数据
        for i in 0..data.len().saturating_sub(25) {
            // 查找Apple公司ID和iBeacon标识
            if data.len() >= i + 25 &&
               data[i] == 0xFF &&     // 厂商特定数据
               data[i+1] == 0x4C &&  // Apple公司ID低字节
               data[i+2] == 0x00 &&  // Apple公司ID高字节
               data[i+3] == 0x02 &&  // iBeacon类型
               data[i+4] == 0x15 {   // 数据长度
                
                let mut beacon_data = BeaconData {
                    uuid: [0; 16],
                    major: 0,
                    minor: 0,
                    tx_power: 0,
                    distance: 0.0,
                };
                
                // 提取UUID
                for j in 0..16 {
                    beacon_data.uuid[j] = data[i + 5 + j];
                }
                
                // 提取Major
                beacon_data.major = ((data[i + 21] as u16) << 8) | (data[i + 22] as u16);
                
                // 提取Minor
                beacon_data.minor = ((data[i + 23] as u16) << 8) | (data[i + 24] as u16);
                
                // 提取TX Power
                beacon_data.tx_power = data[i + 25] as i8;
                
                return Ok(Some(beacon_data));
            }
        }
        
        Ok(None)
    }
    
    // 获取状态
    pub fn is_advertising(&self) -> bool {
        self.advertising
    }
    
    pub fn is_scanning(&self) -> bool {
        self.scanning
    }
    
    pub fn is_connected(&self) -> bool {
        self.connected
    }
}

// 距离计算器
pub struct DistanceCalculator;

impl DistanceCalculator {
    // 根据RSSI和TX Power计算距离
    pub fn calculate_distance(rssi: i8, tx_power: i8) -> f32 {
        if rssi == 0 {
            return -1.0; // 无法计算
        }
        
        let ratio = (tx_power as f32) - (rssi as f32);
        
        if ratio < 0.0 {
            return ratio;
        }
        
        let accuracy = (0.89976 * ratio.powf(7.7095)) + 0.111;
        accuracy
    }
    
    // 根据距离分类接近程度
    pub fn classify_proximity(distance: f32) -> ProximityLevel {
        if distance < 0.0 {
            ProximityLevel::Unknown
        } else if distance < 0.5 {
            ProximityLevel::Immediate
        } else if distance < 4.0 {
            ProximityLevel::Near
        } else {
            ProximityLevel::Far
        }
    }
}

// 接近程度分类
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProximityLevel {
    Immediate, // 立即接近 (<0.5m)
    Near,      // 近距离 (0.5-4m)
    Far,       // 远距离 (>4m)
    Unknown,   // 未知
}

// 信标管理器
pub struct BeaconManager<BT> {
    bluetooth: BT,
    config: BeaconConfig,
    scanned_devices: Vec<ScannedDevice, 16>,
    system_time: u32,
    scan_interval: u32,
    beacon_mode: bool,
}

impl<BT> BeaconManager<BT>
where
    BT: BluetoothModule,
{
    pub fn new(bluetooth: BT, config: BeaconConfig) -> Self {
        Self {
            bluetooth,
            config,
            scanned_devices: Vec::new(),
            system_time: 0,
            scan_interval: 5000, // 5秒扫描间隔
            beacon_mode: true,   // 默认信标模式
        }
    }
    
    // 初始化
    pub fn initialize(&mut self) -> Result<(), &'static str> {
        self.bluetooth.initialize()?;
        
        if self.beacon_mode {
            self.bluetooth.start_beacon(&self.config)?;
        } else {
            self.bluetooth.start_scan()?;
        }
        
        Ok(())
    }
    
    // 更新系统
    pub fn update(&mut self, delta_time: u32) -> Result<(), &'static str> {
        self.system_time += delta_time;
        
        if self.beacon_mode {
            // 信标模式：定期检查状态
            if self.system_time % 10000 == 0 { // 每10秒检查一次
                if !self.bluetooth.is_advertising() {
                    self.bluetooth.start_beacon(&self.config)?;
                }
            }
        } else {
            // 扫描模式：处理扫描结果
            if let Some(device) = self.bluetooth.parse_scan_result()? {
                self.add_or_update_device(device);
            }
            
            // 清理过期设备
            if self.system_time % 30000 == 0 { // 每30秒清理一次
                self.cleanup_old_devices();
            }
        }
        
        Ok(())
    }
    
    // 切换到信标模式
    pub fn switch_to_beacon_mode(&mut self) -> Result<(), &'static str> {
        if !self.beacon_mode {
            self.bluetooth.stop_scan()?;
            self.bluetooth.start_beacon(&self.config)?;
            self.beacon_mode = true;
        }
        Ok(())
    }
    
    // 切换到扫描模式
    pub fn switch_to_scan_mode(&mut self) -> Result<(), &'static str> {
        if self.beacon_mode {
            self.bluetooth.stop_beacon()?;
            self.bluetooth.start_scan()?;
            self.beacon_mode = false;
        }
        Ok(())
    }
    
    // 添加或更新设备
    fn add_or_update_device(&mut self, mut device: ScannedDevice) {
        device.last_seen = self.system_time;
        
        // 计算距离
        if let Some(ref mut beacon_data) = device.beacon_data {
            beacon_data.distance = DistanceCalculator::calculate_distance(
                device.rssi,
                beacon_data.tx_power
            );
        }
        
        // 查找是否已存在
        for existing_device in &mut self.scanned_devices {
            if existing_device.address == device.address {
                *existing_device = device;
                return;
            }
        }
        
        // 添加新设备
        if self.scanned_devices.push(device).is_err() {
            // 如果列表满了，移除最旧的设备
            self.scanned_devices.remove(0);
            self.scanned_devices.push(device).ok();
        }
    }
    
    // 清理过期设备
    fn cleanup_old_devices(&mut self) {
        let current_time = self.system_time;
        let timeout = 60000; // 60秒超时
        
        self.scanned_devices.retain(|device| {
            current_time - device.last_seen < timeout
        });
    }
    
    // 获取扫描到的设备
    pub fn get_scanned_devices(&self) -> &[ScannedDevice] {
        &self.scanned_devices
    }
    
    // 获取特定UUID的信标
    pub fn get_beacons_by_uuid(&self, uuid: &[u8; 16]) -> Vec<&ScannedDevice, 8> {
        let mut beacons = Vec::new();
        
        for device in &self.scanned_devices {
            if let Some(ref beacon_data) = device.beacon_data {
                if beacon_data.uuid == *uuid {
                    beacons.push(device).ok();
                }
            }
        }
        
        beacons
    }
    
    // 获取最近的信标
    pub fn get_nearest_beacon(&self) -> Option<&ScannedDevice> {
        let mut nearest: Option<&ScannedDevice> = None;
        let mut min_distance = f32::INFINITY;
        
        for device in &self.scanned_devices {
            if let Some(ref beacon_data) = device.beacon_data {
                if beacon_data.distance >= 0.0 && beacon_data.distance < min_distance {
                    min_distance = beacon_data.distance;
                    nearest = Some(device);
                }
            }
        }
        
        nearest
    }
    
    // 获取接近程度统计
    pub fn get_proximity_stats(&self) -> ProximityStats {
        let mut stats = ProximityStats::default();
        
        for device in &self.scanned_devices {
            if let Some(ref beacon_data) = device.beacon_data {
                let proximity = DistanceCalculator::classify_proximity(beacon_data.distance);
                
                match proximity {
                    ProximityLevel::Immediate => stats.immediate_count += 1,
                    ProximityLevel::Near => stats.near_count += 1,
                    ProximityLevel::Far => stats.far_count += 1,
                    ProximityLevel::Unknown => stats.unknown_count += 1,
                }
            }
        }
        
        stats.total_count = self.scanned_devices.len() as u32;
        stats
    }
    
    // 更新信标配置
    pub fn update_beacon_config(&mut self, config: BeaconConfig) -> Result<(), &'static str> {
        self.config = config;
        
        if self.beacon_mode {
            self.bluetooth.stop_beacon()?;
            self.bluetooth.start_beacon(&self.config)?;
        }
        
        Ok(())
    }
    
    // 获取当前模式
    pub fn is_beacon_mode(&self) -> bool {
        self.beacon_mode
    }
    
    // 获取系统时间
    pub fn get_system_time(&self) -> u32 {
        self.system_time
    }
}

// 接近程度统计
#[derive(Debug, Default)]
pub struct ProximityStats {
    pub total_count: u32,
    pub immediate_count: u32,
    pub near_count: u32,
    pub far_count: u32,
    pub unknown_count: u32,
}

// 工具函数
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

fn hex_char_to_u8(c: char) -> Option<u8> {
    match c {
        '0'..='9' => Some(c as u8 - b'0'),
        'A'..='F' => Some(c as u8 - b'A' + 10),
        'a'..='f' => Some(c as u8 - b'a' + 10),
        _ => None,
    }
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
    let gpioc = dp.GPIOC.split();
    
    // 状态LED (PC13)
    let mut status_led = gpioc.pc13.into_push_pull_output();
    
    // UART配置 (PA9/PA10 - USART1)
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    
    let serial = Serial::new(
        dp.USART1,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &clocks,
    ).unwrap();
    
    // 创建蓝牙模块
    let bluetooth = BluetoothModule::new(serial);
    
    // 创建信标配置
    let mut config = BeaconConfig::default();
    config.device_name.clear();
    config.device_name.push_str("STM32Beacon").ok();
    config.major = 100;
    config.minor = 1;
    
    // 创建信标管理器
    let mut beacon_manager = BeaconManager::new(bluetooth, config);
    
    // 初始化系统
    if let Err(_) = beacon_manager.initialize() {
        // 初始化失败，错误指示
        loop {
            status_led.set_high();
            cortex_m::asm::delay(84000 * 100); // 100ms
            status_led.set_low();
            cortex_m::asm::delay(84000 * 100); // 100ms
        }
    }
    
    // 主循环
    let mut loop_count = 0u32;
    let mut mode_switch_time = 0u32;
    
    loop {
        loop_count += 1;
        
        // 更新信标管理器
        if let Err(_) = beacon_manager.update(1) {
            // 处理错误
        }
        
        // 每30秒切换一次模式（演示用）
        if loop_count - mode_switch_time >= 30000 {
            mode_switch_time = loop_count;
            
            if beacon_manager.is_beacon_mode() {
                // 切换到扫描模式
                if beacon_manager.switch_to_scan_mode().is_ok() {
                    // 扫描模式指示：快速闪烁
                    for _ in 0..5 {
                        status_led.set_high();
                        cortex_m::asm::delay(84000 * 50); // 50ms
                        status_led.set_low();
                        cortex_m::asm::delay(84000 * 50); // 50ms
                    }
                }
            } else {
                // 切换到信标模式
                if beacon_manager.switch_to_beacon_mode().is_ok() {
                    // 信标模式指示：慢速闪烁
                    for _ in 0..3 {
                        status_led.set_high();
                        cortex_m::asm::delay(84000 * 200); // 200ms
                        status_led.set_low();
                        cortex_m::asm::delay(84000 * 200); // 200ms
                    }
                }
            }
        }
        
        // 状态指示
        if beacon_manager.is_beacon_mode() {
            // 信标模式：LED常亮
            if loop_count % 2000 == 0 { // 每2秒闪烁一次
                status_led.set_high();
                cortex_m::asm::delay(84000 * 100); // 100ms
                status_led.set_low();
            }
        } else {
            // 扫描模式：根据扫描到的设备数量调整闪烁频率
            let device_count = beacon_manager.get_scanned_devices().len();
            let blink_interval = if device_count > 0 { 500 } else { 1000 };
            
            if loop_count % blink_interval == 0 {
                status_led.set_high();
                cortex_m::asm::delay(84000 * 50); // 50ms
                status_led.set_low();
            }
        }
        
        // 1ms延时
        cortex_m::asm::delay(84000);
    }
}