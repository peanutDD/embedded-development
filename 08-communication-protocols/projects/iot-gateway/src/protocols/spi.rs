//! SPI协议处理模块
//! 
//! 负责SPI总线的管理和设备通信，支持多种SPI模式和片选管理。

use heapless::{String, Vec, FnvIndexMap};
use embedded_hal::blocking::{spi::{Transfer, Write}, delay::DelayMs};
use embedded_hal::digital::v2::OutputPin;
use embassy_time::{Duration, Timer, Instant};

use super::{
    ProtocolError, ProtocolConfig, ProtocolPacket, ProtocolStats, ProtocolManager, ProtocolDevice,
    DataDirection, TransferMode,
};

/// SPI错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum SPIError {
    /// 传输错误
    TransferError,
    /// 片选错误
    ChipSelectError,
    /// 配置错误
    ConfigError(&'static str),
    /// 超时
    Timeout,
    /// 设备忙碌
    DeviceBusy,
    /// 无效参数
    InvalidParameter,
    /// 缓冲区错误
    BufferError,
    /// 硬件错误
    HardwareError,
    /// 模式不匹配
    ModeMismatch,
}

/// SPI模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SPIMode {
    /// 模式0: CPOL=0, CPHA=0
    Mode0,
    /// 模式1: CPOL=0, CPHA=1
    Mode1,
    /// 模式2: CPOL=1, CPHA=0
    Mode2,
    /// 模式3: CPOL=1, CPHA=1
    Mode3,
}

/// SPI配置
#[derive(Debug, Clone)]
pub struct SPIConfig {
    /// 时钟频率（Hz）
    pub frequency: u32,
    /// SPI模式
    pub mode: SPIMode,
    /// 数据位宽
    pub data_bits: u8,
    /// 字节序（true为MSB优先）
    pub msb_first: bool,
    /// MOSI引脚
    pub mosi_pin: u8,
    /// MISO引脚
    pub miso_pin: u8,
    /// SCLK引脚
    pub sclk_pin: u8,
    /// 默认片选引脚
    pub default_cs_pin: u8,
    /// 超时时间（毫秒）
    pub timeout_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 片选激活电平（true为高电平）
    pub cs_active_high: bool,
    /// 片选保持时间（微秒）
    pub cs_hold_time_us: u32,
}

impl Default for SPIConfig {
    fn default() -> Self {
        Self {
            frequency: 1_000_000, // 1MHz
            mode: SPIMode::Mode0,
            data_bits: 8,
            msb_first: true,
            mosi_pin: 23,
            miso_pin: 19,
            sclk_pin: 18,
            default_cs_pin: 5,
            timeout_ms: 1000,
            retry_count: 3,
            cs_active_high: false,
            cs_hold_time_us: 1,
        }
    }
}

/// SPI设备信息
#[derive(Debug, Clone)]
pub struct SPIDeviceInfo {
    /// 设备ID
    pub device_id: u8,
    /// 片选引脚
    pub cs_pin: u8,
    /// 设备名称
    pub name: String<32>,
    /// 设备类型
    pub device_type: String<16>,
    /// SPI模式
    pub mode: SPIMode,
    /// 最大频率
    pub max_frequency: u32,
    /// 是否在线
    pub online: bool,
    /// 最后通信时间
    pub last_communication: u64,
    /// 通信统计
    pub stats: SPIDeviceStats,
}

/// SPI设备统计
#[derive(Debug, Clone, Default)]
pub struct SPIDeviceStats {
    /// 传输次数
    pub transfer_count: u32,
    /// 写入次数
    pub write_count: u32,
    /// 成功次数
    pub success_count: u32,
    /// 失败次数
    pub failure_count: u32,
    /// 传输字节数
    pub bytes_transferred: u64,
    /// 平均传输时间（微秒）
    pub avg_transfer_time_us: u32,
    /// 最后错误
    pub last_error: Option<SPIError>,
}

/// 片选管理器
pub struct ChipSelectManager<CS> 
where
    CS: OutputPin,
{
    /// 片选引脚映射
    cs_pins: FnvIndexMap<u8, CS, 16>,
    /// 当前激活的片选
    active_cs: Option<u8>,
    /// 片选激活电平
    active_high: bool,
}

impl<CS> ChipSelectManager<CS>
where
    CS: OutputPin,
{
    /// 创建新的片选管理器
    pub fn new(active_high: bool) -> Self {
        Self {
            cs_pins: FnvIndexMap::new(),
            active_cs: None,
            active_high,
        }
    }

    /// 添加片选引脚
    pub fn add_cs_pin(&mut self, device_id: u8, cs_pin: CS) -> Result<(), SPIError> {
        self.cs_pins.insert(device_id, cs_pin)
            .map_err(|_| SPIError::ConfigError("Too many CS pins"))?;
        Ok(())
    }

    /// 选择设备
    pub fn select_device(&mut self, device_id: u8) -> Result<(), SPIError> {
        // 先取消选择当前设备
        if let Some(current_id) = self.active_cs {
            self.deselect_device(current_id)?;
        }

        // 选择新设备
        if let Some(cs_pin) = self.cs_pins.get_mut(&device_id) {
            if self.active_high {
                cs_pin.set_high().map_err(|_| SPIError::ChipSelectError)?;
            } else {
                cs_pin.set_low().map_err(|_| SPIError::ChipSelectError)?;
            }
            self.active_cs = Some(device_id);
            Ok(())
        } else {
            Err(SPIError::InvalidParameter)
        }
    }

    /// 取消选择设备
    pub fn deselect_device(&mut self, device_id: u8) -> Result<(), SPIError> {
        if let Some(cs_pin) = self.cs_pins.get_mut(&device_id) {
            if self.active_high {
                cs_pin.set_low().map_err(|_| SPIError::ChipSelectError)?;
            } else {
                cs_pin.set_high().map_err(|_| SPIError::ChipSelectError)?;
            }
            
            if self.active_cs == Some(device_id) {
                self.active_cs = None;
            }
            Ok(())
        } else {
            Err(SPIError::InvalidParameter)
        }
    }

    /// 取消选择所有设备
    pub fn deselect_all(&mut self) -> Result<(), SPIError> {
        for (device_id, cs_pin) in self.cs_pins.iter_mut() {
            if self.active_high {
                cs_pin.set_low().map_err(|_| SPIError::ChipSelectError)?;
            } else {
                cs_pin.set_high().map_err(|_| SPIError::ChipSelectError)?;
            }
        }
        self.active_cs = None;
        Ok(())
    }

    /// 获取当前激活的设备
    pub fn get_active_device(&self) -> Option<u8> {
        self.active_cs
    }
}

/// SPI管理器
pub struct SPIManager<SPI, CS, D> 
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    D: DelayMs<u32>,
{
    /// SPI外设
    spi: SPI,
    /// 片选管理器
    cs_manager: ChipSelectManager<CS>,
    /// 延时提供者
    delay: D,
    /// 配置
    config: SPIConfig,
    /// 协议配置
    protocol_config: ProtocolConfig,
    /// 统计信息
    stats: ProtocolStats,
    /// 设备列表
    devices: FnvIndexMap<u8, SPIDeviceInfo, 16>,
    /// 总线状态
    bus_state: BusState,
    /// 最后操作时间
    last_operation_time: Instant,
}

/// 总线状态
#[derive(Debug, Clone, Copy, PartialEq)]
enum BusState {
    /// 空闲
    Idle,
    /// 忙碌
    Busy,
    /// 错误
    Error,
}

impl<SPI, CS, D> SPIManager<SPI, CS, D>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    D: DelayMs<u32>,
{
    /// 创建新的SPI管理器
    pub fn new(spi: SPI, delay: D, config: SPIConfig) -> Self {
        let protocol_config = ProtocolConfig {
            protocol_type: super::ProtocolType::SPI,
            timeout_ms: config.timeout_ms,
            retry_count: config.retry_count,
            ..Default::default()
        };

        let cs_manager = ChipSelectManager::new(config.cs_active_high);

        Self {
            spi,
            cs_manager,
            delay,
            config,
            protocol_config,
            stats: ProtocolStats::default(),
            devices: FnvIndexMap::new(),
            bus_state: BusState::Idle,
            last_operation_time: Instant::now(),
        }
    }

    /// 注册SPI设备
    pub fn register_device(&mut self, device_info: SPIDeviceInfo, cs_pin: CS) -> Result<(), SPIError> {
        // 添加片选引脚
        self.cs_manager.add_cs_pin(device_info.device_id, cs_pin)?;

        // 添加设备信息
        self.devices.insert(device_info.device_id, device_info)
            .map_err(|_| SPIError::ConfigError("Too many devices"))?;

        Ok(())
    }

    /// 传输数据（全双工）
    pub fn transfer(&mut self, device_id: u8, data: &mut [u8]) -> Result<(), SPIError> {
        self.validate_device(device_id)?;
        self.bus_state = BusState::Busy;

        let start_time = Instant::now();
        
        // 选择设备
        self.cs_manager.select_device(device_id)?;
        
        // 等待片选建立时间
        if self.config.cs_hold_time_us > 0 {
            self.delay.delay_ms((self.config.cs_hold_time_us + 999) / 1000);
        }

        let mut result = Ok(());
        
        for attempt in 0..=self.config.retry_count {
            match self.spi.transfer(data) {
                Ok(_) => {
                    result = Ok(());
                    break;
                },
                Err(_) => {
                    if attempt < self.config.retry_count {
                        self.delay.delay_ms(10);
                        self.stats.retries += 1;
                    } else {
                        result = Err(SPIError::TransferError);
                    }
                }
            }
        }

        // 取消选择设备
        self.cs_manager.deselect_device(device_id)?;
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(device_id, operation_time, result.is_ok(), data.len(), true);
        
        self.bus_state = BusState::Idle;
        result
    }

    /// 写入数据（单向）
    pub fn write(&mut self, device_id: u8, data: &[u8]) -> Result<(), SPIError> {
        self.validate_device(device_id)?;
        self.bus_state = BusState::Busy;

        let start_time = Instant::now();
        
        // 选择设备
        self.cs_manager.select_device(device_id)?;
        
        // 等待片选建立时间
        if self.config.cs_hold_time_us > 0 {
            self.delay.delay_ms((self.config.cs_hold_time_us + 999) / 1000);
        }

        let mut result = Ok(());
        
        for attempt in 0..=self.config.retry_count {
            match self.spi.write(data) {
                Ok(_) => {
                    result = Ok(());
                    break;
                },
                Err(_) => {
                    if attempt < self.config.retry_count {
                        self.delay.delay_ms(10);
                        self.stats.retries += 1;
                    } else {
                        result = Err(SPIError::TransferError);
                    }
                }
            }
        }

        // 取消选择设备
        self.cs_manager.deselect_device(device_id)?;
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(device_id, operation_time, result.is_ok(), data.len(), false);
        
        self.bus_state = BusState::Idle;
        result
    }

    /// 读取数据
    pub fn read(&mut self, device_id: u8, length: usize) -> Result<Vec<u8, 256>, SPIError> {
        let mut buffer = vec![0u8; length];
        self.transfer(device_id, &mut buffer)?;
        
        let mut result = Vec::new();
        result.extend_from_slice(&buffer)
            .map_err(|_| SPIError::BufferError)?;
        
        Ok(result)
    }

    /// 写入寄存器
    pub fn write_register(&mut self, device_id: u8, register: u8, value: u8) -> Result<(), SPIError> {
        let data = [register, value];
        self.write(device_id, &data)
    }

    /// 读取寄存器
    pub fn read_register(&mut self, device_id: u8, register: u8) -> Result<u8, SPIError> {
        let mut data = [register, 0x00];
        self.transfer(device_id, &mut data)?;
        Ok(data[1])
    }

    /// 写入多个寄存器
    pub fn write_registers(&mut self, device_id: u8, start_register: u8, data: &[u8]) -> Result<(), SPIError> {
        let mut write_data = Vec::new();
        write_data.push(start_register)
            .map_err(|_| SPIError::BufferError)?;
        write_data.extend_from_slice(data)
            .map_err(|_| SPIError::BufferError)?;
        
        self.write(device_id, &write_data)
    }

    /// 读取多个寄存器
    pub fn read_registers(&mut self, device_id: u8, start_register: u8, count: usize) -> Result<Vec<u8, 256>, SPIError> {
        let mut data = Vec::new();
        data.push(start_register)
            .map_err(|_| SPIError::BufferError)?;
        
        // 添加虚拟字节用于读取
        for _ in 0..count {
            data.push(0x00)
                .map_err(|_| SPIError::BufferError)?;
        }
        
        self.transfer(device_id, &mut data)?;
        
        // 移除第一个字节（寄存器地址）
        data.remove(0);
        Ok(data)
    }

    /// 检查设备是否存在
    pub fn device_exists(&mut self, device_id: u8) -> bool {
        self.devices.contains_key(&device_id)
    }

    /// 扫描设备
    pub fn scan_devices(&mut self) -> Vec<u8, 16> {
        let mut found_devices = Vec::new();
        
        for device_id in self.devices.keys() {
            if found_devices.push(*device_id).is_err() {
                break;
            }
        }
        
        found_devices
    }

    /// 验证设备
    fn validate_device(&self, device_id: u8) -> Result<(), SPIError> {
        if !self.devices.contains_key(&device_id) {
            return Err(SPIError::InvalidParameter);
        }

        if self.bus_state == BusState::Error {
            return Err(SPIError::HardwareError);
        }

        Ok(())
    }

    /// 更新统计信息
    fn update_stats(&mut self, device_id: u8, operation_time: u32, success: bool, bytes: usize, is_transfer: bool) {
        // 更新全局统计
        if success {
            if is_transfer {
                self.stats.packets_sent += 1;
                self.stats.packets_received += 1;
            } else {
                self.stats.packets_sent += 1;
            }
            self.stats.total_bytes_transferred += bytes as u64;
        } else {
            self.stats.send_failures += 1;
        }

        self.update_avg_transfer_time(operation_time);

        // 更新设备统计
        if let Some(device_info) = self.devices.get_mut(&device_id) {
            if is_transfer {
                device_info.stats.transfer_count += 1;
            } else {
                device_info.stats.write_count += 1;
            }

            if success {
                device_info.stats.success_count += 1;
            } else {
                device_info.stats.failure_count += 1;
            }

            device_info.stats.bytes_transferred += bytes as u64;

            // 更新设备平均传输时间
            let total_ops = device_info.stats.success_count + device_info.stats.failure_count;
            if total_ops == 1 {
                device_info.stats.avg_transfer_time_us = operation_time;
            } else {
                let alpha = 0.1;
                let new_avg = (1.0 - alpha) * device_info.stats.avg_transfer_time_us as f32 + 
                             alpha * operation_time as f32;
                device_info.stats.avg_transfer_time_us = new_avg as u32;
            }

            device_info.last_communication = self.get_current_timestamp();
        }

        self.last_operation_time = Instant::now();
    }

    /// 更新平均传输时间
    fn update_avg_transfer_time(&mut self, operation_time: u32) {
        let total_ops = self.stats.packets_sent + self.stats.packets_received;
        if total_ops == 1 {
            self.stats.avg_transfer_time_us = operation_time;
        } else {
            let alpha = 0.1;
            let new_avg = (1.0 - alpha) * self.stats.avg_transfer_time_us as f32 + 
                         alpha * operation_time as f32;
            self.stats.avg_transfer_time_us = new_avg as u32;
        }
    }

    /// 获取设备信息
    pub fn get_device_info(&self, device_id: u8) -> Option<&SPIDeviceInfo> {
        self.devices.get(&device_id)
    }

    /// 获取所有设备
    pub fn get_all_devices(&self) -> Vec<&SPIDeviceInfo, 16> {
        let mut devices = Vec::new();
        for device_info in self.devices.values() {
            if devices.push(device_info).is_err() {
                break;
            }
        }
        devices
    }

    /// 获取当前时间戳
    fn get_current_timestamp(&self) -> u64 {
        self.last_operation_time.elapsed().as_millis() as u64
    }

    /// 获取总线状态
    pub fn get_bus_state(&self) -> BusState {
        self.bus_state
    }

    /// 设置设备名称
    pub fn set_device_name(&mut self, device_id: u8, name: &str) -> Result<(), SPIError> {
        if let Some(device_info) = self.devices.get_mut(&device_id) {
            device_info.name = String::from_str(name)
                .map_err(|_| SPIError::ConfigError("Invalid device name"))?;
            Ok(())
        } else {
            Err(SPIError::InvalidParameter)
        }
    }

    /// 设置设备类型
    pub fn set_device_type(&mut self, device_id: u8, device_type: &str) -> Result<(), SPIError> {
        if let Some(device_info) = self.devices.get_mut(&device_id) {
            device_info.device_type = String::from_str(device_type)
                .map_err(|_| SPIError::ConfigError("Invalid device type"))?;
            Ok(())
        } else {
            Err(SPIError::InvalidParameter)
        }
    }
}

impl<SPI, CS, D> ProtocolManager for SPIManager<SPI, CS, D>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    D: DelayMs<u32>,
{
    fn init(&mut self) -> Result<(), ProtocolError> {
        self.bus_state = BusState::Idle;
        self.cs_manager.deselect_all()
            .map_err(|e| ProtocolError::SPI(e))?;
        Ok(())
    }

    fn send_packet(&mut self, packet: &ProtocolPacket) -> Result<(), ProtocolError> {
        let device_id = packet.address as u8;
        
        match packet.direction {
            DataDirection::Write => {
                if let Some(register) = packet.register {
                    let mut data = Vec::new();
                    data.push(register as u8)
                        .map_err(|_| ProtocolError::BufferError)?;
                    data.extend_from_slice(&packet.data)
                        .map_err(|_| ProtocolError::BufferError)?;
                    
                    self.write(device_id, &data)
                        .map_err(|e| ProtocolError::SPI(e))
                } else {
                    self.write(device_id, &packet.data)
                        .map_err(|e| ProtocolError::SPI(e))
                }
            },
            _ => Err(ProtocolError::UnsupportedProtocol),
        }
    }

    fn receive_packet(&mut self) -> Result<ProtocolPacket, ProtocolError> {
        // SPI是主从协议，主设备不会主动接收数据包
        Err(ProtocolError::UnsupportedProtocol)
    }

    fn read_data(&mut self, address: u32, register: Option<u16>, length: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        let device_id = address as u8;
        
        if let Some(reg) = register {
            self.read_registers(device_id, reg as u8, length)
                .map_err(|e| ProtocolError::SPI(e))
        } else {
            self.read(device_id, length)
                .map_err(|e| ProtocolError::SPI(e))
        }
    }

    fn write_data(&mut self, address: u32, register: Option<u16>, data: &[u8]) -> Result<(), ProtocolError> {
        let device_id = address as u8;
        
        if let Some(reg) = register {
            self.write_registers(device_id, reg as u8, data)
                .map_err(|e| ProtocolError::SPI(e))
        } else {
            self.write(device_id, data)
                .map_err(|e| ProtocolError::SPI(e))
        }
    }

    fn device_exists(&mut self, address: u32) -> Result<bool, ProtocolError> {
        Ok(self.device_exists(address as u8))
    }

    fn scan_devices(&mut self) -> Result<Vec<u32, 128>, ProtocolError> {
        let devices = self.scan_devices();
        let mut result = Vec::new();
        
        for device_id in devices {
            result.push(device_id as u32)
                .map_err(|_| ProtocolError::BufferError)?;
        }
        
        Ok(result)
    }

    fn get_stats(&self) -> &ProtocolStats {
        &self.stats
    }

    fn reset_stats(&mut self) {
        self.stats = ProtocolStats::default();
        for device_info in self.devices.values_mut() {
            device_info.stats = SPIDeviceStats::default();
        }
    }

    fn set_config(&mut self, config: ProtocolConfig) -> Result<(), ProtocolError> {
        self.protocol_config = config;
        Ok(())
    }

    fn get_config(&self) -> &ProtocolConfig {
        &self.protocol_config
    }
}

/// SPI设备包装器
pub struct SPIDevice<SPI, CS, D>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    D: DelayMs<u32>,
{
    /// SPI管理器引用
    manager: *mut SPIManager<SPI, CS, D>,
    /// 设备ID
    device_id: u8,
}

impl<SPI, CS, D> SPIDevice<SPI, CS, D>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    D: DelayMs<u32>,
{
    /// 创建新的SPI设备
    pub fn new(manager: &mut SPIManager<SPI, CS, D>, device_id: u8) -> Self {
        Self {
            manager: manager as *mut _,
            device_id,
        }
    }

    /// 获取管理器引用
    fn get_manager(&mut self) -> &mut SPIManager<SPI, CS, D> {
        unsafe { &mut *self.manager }
    }
}

impl<SPI, CS, D> ProtocolDevice for SPIDevice<SPI, CS, D>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    D: DelayMs<u32>,
{
    fn read_register(&mut self, register: u16) -> Result<u8, ProtocolError> {
        self.get_manager()
            .read_register(self.device_id, register as u8)
            .map_err(|e| ProtocolError::SPI(e))
    }

    fn write_register(&mut self, register: u16, value: u8) -> Result<(), ProtocolError> {
        self.get_manager()
            .write_register(self.device_id, register as u8, value)
            .map_err(|e| ProtocolError::SPI(e))
    }

    fn read_registers(&mut self, start_register: u16, count: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        self.get_manager()
            .read_registers(self.device_id, start_register as u8, count)
            .map_err(|e| ProtocolError::SPI(e))
    }

    fn write_registers(&mut self, start_register: u16, data: &[u8]) -> Result<(), ProtocolError> {
        self.get_manager()
            .write_registers(self.device_id, start_register as u8, data)
            .map_err(|e| ProtocolError::SPI(e))
    }

    fn read_block(&mut self, length: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        self.get_manager()
            .read(self.device_id, length)
            .map_err(|e| ProtocolError::SPI(e))
    }

    fn write_block(&mut self, data: &[u8]) -> Result<(), ProtocolError> {
        self.get_manager()
            .write(self.device_id, data)
            .map_err(|e| ProtocolError::SPI(e))
    }

    fn get_address(&self) -> u32 {
        self.device_id as u32
    }

    fn set_address(&mut self, address: u32) {
        self.device_id = address as u8;
    }

    fn is_connected(&mut self) -> Result<bool, ProtocolError> {
        Ok(self.get_manager().device_exists(self.device_id))
    }

    fn reset(&mut self) -> Result<(), ProtocolError> {
        // SPI设备通常没有标准的复位命令
        // 可以尝试发送设备特定的复位序列
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 模拟SPI实现用于测试
    struct MockSPI {
        fail_operations: bool,
    }

    impl MockSPI {
        fn new() -> Self {
            Self {
                fail_operations: false,
            }
        }
    }

    impl Transfer<u8> for MockSPI {
        type Error = ();

        fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
            if self.fail_operations {
                Err(())
            } else {
                Ok(words)
            }
        }
    }

    impl Write<u8> for MockSPI {
        type Error = ();

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            if self.fail_operations {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    struct MockCS {
        state: bool,
    }

    impl MockCS {
        fn new() -> Self {
            Self { state: false }
        }
    }

    impl OutputPin for MockCS {
        type Error = ();

        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.state = false;
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.state = true;
            Ok(())
        }
    }

    struct MockDelay;

    impl DelayMs<u32> for MockDelay {
        fn delay_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_spi_manager_creation() {
        let spi = MockSPI::new();
        let delay = MockDelay;
        let config = SPIConfig::default();
        
        let manager = SPIManager::new(spi, delay, config);
        assert_eq!(manager.get_bus_state(), BusState::Idle);
    }

    #[test]
    fn test_chip_select_manager() {
        let mut cs_manager = ChipSelectManager::new(false);
        let cs_pin = MockCS::new();
        
        assert!(cs_manager.add_cs_pin(1, cs_pin).is_ok());
        assert!(cs_manager.select_device(1).is_ok());
        assert_eq!(cs_manager.get_active_device(), Some(1));
        assert!(cs_manager.deselect_device(1).is_ok());
        assert_eq!(cs_manager.get_active_device(), None);
    }

    #[test]
    fn test_spi_mode_values() {
        // 测试SPI模式枚举
        assert_ne!(SPIMode::Mode0, SPIMode::Mode1);
        assert_ne!(SPIMode::Mode2, SPIMode::Mode3);
    }
}