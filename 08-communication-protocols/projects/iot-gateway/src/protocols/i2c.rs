//! I2C协议处理模块
//! 
//! 负责I2C总线的管理和设备通信，支持标准速度和快速模式。

use heapless::{String, Vec, FnvIndexMap};
use embedded_hal::blocking::{i2c::{Read, Write, WriteRead}, delay::DelayMs};
use embassy_time::{Duration, Timer, Instant};

use super::{
    ProtocolError, ProtocolConfig, ProtocolPacket, ProtocolStats, ProtocolManager, ProtocolDevice,
    DataDirection, TransferMode,
};

/// I2C错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum I2CError {
    /// 总线错误
    BusError,
    /// 设备无应答
    NoAcknowledge,
    /// 仲裁丢失
    ArbitrationLoss,
    /// 超时
    Timeout,
    /// 数据溢出
    Overrun,
    /// 配置错误
    ConfigError(&'static str),
    /// 设备忙碌
    DeviceBusy,
    /// 地址无效
    InvalidAddress,
    /// 数据长度错误
    InvalidDataLength,
    /// 硬件错误
    HardwareError,
}

/// I2C配置
#[derive(Debug, Clone)]
pub struct I2CConfig {
    /// 时钟频率（Hz）
    pub frequency: u32,
    /// SDA引脚
    pub sda_pin: u8,
    /// SCL引脚
    pub scl_pin: u8,
    /// 上拉电阻启用
    pub pullup_enabled: bool,
    /// 超时时间（毫秒）
    pub timeout_ms: u32,
    /// 重试次数
    pub retry_count: u8,
    /// 地址位数（7或10位）
    pub address_bits: u8,
    /// 启用时钟拉伸
    pub clock_stretching: bool,
    /// 总线恢复启用
    pub bus_recovery: bool,
}

impl Default for I2CConfig {
    fn default() -> Self {
        Self {
            frequency: 100_000, // 100kHz标准模式
            sda_pin: 21,
            scl_pin: 22,
            pullup_enabled: true,
            timeout_ms: 1000,
            retry_count: 3,
            address_bits: 7,
            clock_stretching: true,
            bus_recovery: true,
        }
    }
}

/// I2C设备信息
#[derive(Debug, Clone)]
pub struct I2CDeviceInfo {
    /// 设备地址
    pub address: u8,
    /// 设备名称
    pub name: String<32>,
    /// 设备类型
    pub device_type: String<16>,
    /// 是否在线
    pub online: bool,
    /// 最后通信时间
    pub last_communication: u64,
    /// 通信统计
    pub stats: I2CDeviceStats,
}

/// I2C设备统计
#[derive(Debug, Clone, Default)]
pub struct I2CDeviceStats {
    /// 读取次数
    pub read_count: u32,
    /// 写入次数
    pub write_count: u32,
    /// 成功次数
    pub success_count: u32,
    /// 失败次数
    pub failure_count: u32,
    /// 平均响应时间（微秒）
    pub avg_response_time_us: u32,
    /// 最后错误
    pub last_error: Option<I2CError>,
}

/// I2C管理器
pub struct I2CManager<I2C, D> 
where
    I2C: Read + Write + WriteRead,
    D: DelayMs<u32>,
{
    /// I2C外设
    i2c: I2C,
    /// 延时提供者
    delay: D,
    /// 配置
    config: I2CConfig,
    /// 协议配置
    protocol_config: ProtocolConfig,
    /// 统计信息
    stats: ProtocolStats,
    /// 设备列表
    devices: FnvIndexMap<u8, I2CDeviceInfo, 128>,
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
    /// 恢复中
    Recovering,
}

impl<I2C, D> I2CManager<I2C, D>
where
    I2C: Read + Write + WriteRead,
    D: DelayMs<u32>,
{
    /// 创建新的I2C管理器
    pub fn new(i2c: I2C, delay: D, config: I2CConfig) -> Self {
        let protocol_config = ProtocolConfig {
            protocol_type: super::ProtocolType::I2C,
            timeout_ms: config.timeout_ms,
            retry_count: config.retry_count,
            ..Default::default()
        };

        Self {
            i2c,
            delay,
            config,
            protocol_config,
            stats: ProtocolStats::default(),
            devices: FnvIndexMap::new(),
            bus_state: BusState::Idle,
            last_operation_time: Instant::now(),
        }
    }

    /// 扫描I2C总线上的设备
    pub fn scan_bus(&mut self) -> Result<Vec<u8, 128>, I2CError> {
        let mut found_devices = Vec::new();
        
        self.bus_state = BusState::Busy;
        
        // 扫描7位地址范围（0x08-0x77）
        for address in 0x08..=0x77 {
            if self.probe_device(address)? {
                if found_devices.push(address).is_err() {
                    break; // 缓冲区满
                }
                
                // 更新或创建设备信息
                self.update_device_info(address, true);
            }
        }
        
        self.bus_state = BusState::Idle;
        self.stats.packets_sent += 1;
        
        Ok(found_devices)
    }

    /// 探测设备是否存在
    pub fn probe_device(&mut self, address: u8) -> Result<bool, I2CError> {
        if !self.is_valid_address(address) {
            return Err(I2CError::InvalidAddress);
        }

        let start_time = Instant::now();
        
        // 尝试写入0字节来探测设备
        let result = self.i2c.write(address, &[]);
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_avg_response_time(operation_time);
        
        match result {
            Ok(()) => {
                self.stats.packets_sent += 1;
                Ok(true)
            },
            Err(_) => {
                self.stats.send_failures += 1;
                Ok(false)
            }
        }
    }

    /// 读取设备数据
    pub fn read_device(&mut self, address: u8, length: usize) -> Result<Vec<u8, 256>, I2CError> {
        self.validate_operation(address, length)?;
        
        let mut buffer = vec![0u8; length];
        let mut data = Vec::new();
        
        let start_time = Instant::now();
        
        for attempt in 0..=self.config.retry_count {
            match self.i2c.read(address, &mut buffer) {
                Ok(()) => {
                    data.extend_from_slice(&buffer)
                        .map_err(|_| I2CError::DataOverflow)?;
                    
                    let operation_time = start_time.elapsed().as_micros() as u32;
                    self.update_stats(address, operation_time, true, false);
                    
                    return Ok(data);
                },
                Err(_) => {
                    if attempt < self.config.retry_count {
                        self.delay.delay_ms(10); // 重试前延迟
                        self.stats.retries += 1;
                    }
                }
            }
        }
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(address, operation_time, false, false);
        
        Err(I2CError::NoAcknowledge)
    }

    /// 写入设备数据
    pub fn write_device(&mut self, address: u8, data: &[u8]) -> Result<(), I2CError> {
        self.validate_operation(address, data.len())?;
        
        let start_time = Instant::now();
        
        for attempt in 0..=self.config.retry_count {
            match self.i2c.write(address, data) {
                Ok(()) => {
                    let operation_time = start_time.elapsed().as_micros() as u32;
                    self.update_stats(address, operation_time, true, true);
                    
                    return Ok(());
                },
                Err(_) => {
                    if attempt < self.config.retry_count {
                        self.delay.delay_ms(10);
                        self.stats.retries += 1;
                    }
                }
            }
        }
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(address, operation_time, false, true);
        
        Err(I2CError::NoAcknowledge)
    }

    /// 读取寄存器
    pub fn read_register(&mut self, address: u8, register: u8) -> Result<u8, I2CError> {
        self.validate_operation(address, 1)?;
        
        let mut buffer = [0u8; 1];
        let start_time = Instant::now();
        
        for attempt in 0..=self.config.retry_count {
            match self.i2c.write_read(address, &[register], &mut buffer) {
                Ok(()) => {
                    let operation_time = start_time.elapsed().as_micros() as u32;
                    self.update_stats(address, operation_time, true, false);
                    
                    return Ok(buffer[0]);
                },
                Err(_) => {
                    if attempt < self.config.retry_count {
                        self.delay.delay_ms(10);
                        self.stats.retries += 1;
                    }
                }
            }
        }
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(address, operation_time, false, false);
        
        Err(I2CError::NoAcknowledge)
    }

    /// 写入寄存器
    pub fn write_register(&mut self, address: u8, register: u8, value: u8) -> Result<(), I2CError> {
        self.validate_operation(address, 2)?;
        
        let data = [register, value];
        let start_time = Instant::now();
        
        for attempt in 0..=self.config.retry_count {
            match self.i2c.write(address, &data) {
                Ok(()) => {
                    let operation_time = start_time.elapsed().as_micros() as u32;
                    self.update_stats(address, operation_time, true, true);
                    
                    return Ok(());
                },
                Err(_) => {
                    if attempt < self.config.retry_count {
                        self.delay.delay_ms(10);
                        self.stats.retries += 1;
                    }
                }
            }
        }
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(address, operation_time, false, true);
        
        Err(I2CError::NoAcknowledge)
    }

    /// 读取多个寄存器
    pub fn read_registers(&mut self, address: u8, start_register: u8, count: usize) -> Result<Vec<u8, 256>, I2CError> {
        self.validate_operation(address, count)?;
        
        let mut buffer = vec![0u8; count];
        let mut data = Vec::new();
        
        let start_time = Instant::now();
        
        for attempt in 0..=self.config.retry_count {
            match self.i2c.write_read(address, &[start_register], &mut buffer) {
                Ok(()) => {
                    data.extend_from_slice(&buffer)
                        .map_err(|_| I2CError::DataOverflow)?;
                    
                    let operation_time = start_time.elapsed().as_micros() as u32;
                    self.update_stats(address, operation_time, true, false);
                    
                    return Ok(data);
                },
                Err(_) => {
                    if attempt < self.config.retry_count {
                        self.delay.delay_ms(10);
                        self.stats.retries += 1;
                    }
                }
            }
        }
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(address, operation_time, false, false);
        
        Err(I2CError::NoAcknowledge)
    }

    /// 写入多个寄存器
    pub fn write_registers(&mut self, address: u8, start_register: u8, data: &[u8]) -> Result<(), I2CError> {
        self.validate_operation(address, data.len() + 1)?;
        
        let mut write_data = Vec::new();
        write_data.push(start_register)
            .map_err(|_| I2CError::DataOverflow)?;
        write_data.extend_from_slice(data)
            .map_err(|_| I2CError::DataOverflow)?;
        
        let start_time = Instant::now();
        
        for attempt in 0..=self.config.retry_count {
            match self.i2c.write(address, &write_data) {
                Ok(()) => {
                    let operation_time = start_time.elapsed().as_micros() as u32;
                    self.update_stats(address, operation_time, true, true);
                    
                    return Ok(());
                },
                Err(_) => {
                    if attempt < self.config.retry_count {
                        self.delay.delay_ms(10);
                        self.stats.retries += 1;
                    }
                }
            }
        }
        
        let operation_time = start_time.elapsed().as_micros() as u32;
        self.update_stats(address, operation_time, false, true);
        
        Err(I2CError::NoAcknowledge)
    }

    /// 总线恢复
    pub fn recover_bus(&mut self) -> Result<(), I2CError> {
        if !self.config.bus_recovery {
            return Err(I2CError::ConfigError("Bus recovery disabled"));
        }

        self.bus_state = BusState::Recovering;
        
        // 执行总线恢复序列
        // 1. 生成9个时钟脉冲
        for _ in 0..9 {
            // 这里应该手动控制SCL引脚
            self.delay.delay_ms(1);
        }
        
        // 2. 发送STOP条件
        // 这里应该手动控制SDA和SCL引脚
        
        self.delay.delay_ms(10);
        self.bus_state = BusState::Idle;
        
        Ok(())
    }

    /// 验证操作参数
    fn validate_operation(&self, address: u8, data_length: usize) -> Result<(), I2CError> {
        if !self.is_valid_address(address) {
            return Err(I2CError::InvalidAddress);
        }

        if data_length == 0 || data_length > 256 {
            return Err(I2CError::InvalidDataLength);
        }

        if self.bus_state == BusState::Error {
            return Err(I2CError::BusError);
        }

        Ok(())
    }

    /// 检查地址是否有效
    fn is_valid_address(&self, address: u8) -> bool {
        if self.config.address_bits == 7 {
            // 7位地址：0x08-0x77（排除保留地址）
            address >= 0x08 && address <= 0x77
        } else {
            // 10位地址支持
            true
        }
    }

    /// 更新设备信息
    fn update_device_info(&mut self, address: u8, online: bool) {
        let device_info = self.devices.entry(address).or_insert_with(|| {
            I2CDeviceInfo {
                address,
                name: String::from_str(&format!("I2C_0x{:02X}", address)).unwrap_or_default(),
                device_type: String::from_str("Unknown").unwrap_or_default(),
                online: false,
                last_communication: 0,
                stats: I2CDeviceStats::default(),
            }
        });

        device_info.online = online;
        device_info.last_communication = self.get_current_timestamp();
    }

    /// 更新统计信息
    fn update_stats(&mut self, address: u8, operation_time: u32, success: bool, is_write: bool) {
        // 更新全局统计
        if success {
            if is_write {
                self.stats.packets_sent += 1;
            } else {
                self.stats.packets_received += 1;
            }
        } else {
            if is_write {
                self.stats.send_failures += 1;
            } else {
                self.stats.receive_failures += 1;
            }
        }

        self.update_avg_response_time(operation_time);

        // 更新设备统计
        if let Some(device_info) = self.devices.get_mut(&address) {
            if is_write {
                device_info.stats.write_count += 1;
            } else {
                device_info.stats.read_count += 1;
            }

            if success {
                device_info.stats.success_count += 1;
            } else {
                device_info.stats.failure_count += 1;
            }

            // 更新设备平均响应时间
            let total_ops = device_info.stats.success_count + device_info.stats.failure_count;
            if total_ops == 1 {
                device_info.stats.avg_response_time_us = operation_time;
            } else {
                let alpha = 0.1;
                let new_avg = (1.0 - alpha) * device_info.stats.avg_response_time_us as f32 + 
                             alpha * operation_time as f32;
                device_info.stats.avg_response_time_us = new_avg as u32;
            }

            device_info.last_communication = self.get_current_timestamp();
        }

        self.last_operation_time = Instant::now();
    }

    /// 更新平均响应时间
    fn update_avg_response_time(&mut self, operation_time: u32) {
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
    pub fn get_device_info(&self, address: u8) -> Option<&I2CDeviceInfo> {
        self.devices.get(&address)
    }

    /// 获取所有设备
    pub fn get_all_devices(&self) -> Vec<&I2CDeviceInfo, 128> {
        let mut devices = Vec::new();
        for device_info in self.devices.values() {
            if devices.push(device_info).is_err() {
                break;
            }
        }
        devices
    }

    /// 获取在线设备
    pub fn get_online_devices(&self) -> Vec<u8, 128> {
        let mut online_devices = Vec::new();
        for (address, device_info) in &self.devices {
            if device_info.online {
                if online_devices.push(*address).is_err() {
                    break;
                }
            }
        }
        online_devices
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
    pub fn set_device_name(&mut self, address: u8, name: &str) -> Result<(), I2CError> {
        if let Some(device_info) = self.devices.get_mut(&address) {
            device_info.name = String::from_str(name)
                .map_err(|_| I2CError::ConfigError("Invalid device name"))?;
            Ok(())
        } else {
            Err(I2CError::InvalidAddress)
        }
    }

    /// 设置设备类型
    pub fn set_device_type(&mut self, address: u8, device_type: &str) -> Result<(), I2CError> {
        if let Some(device_info) = self.devices.get_mut(&address) {
            device_info.device_type = String::from_str(device_type)
                .map_err(|_| I2CError::ConfigError("Invalid device type"))?;
            Ok(())
        } else {
            Err(I2CError::InvalidAddress)
        }
    }
}

impl<I2C, D> ProtocolManager for I2CManager<I2C, D>
where
    I2C: Read + Write + WriteRead,
    D: DelayMs<u32>,
{
    fn init(&mut self) -> Result<(), ProtocolError> {
        self.bus_state = BusState::Idle;
        Ok(())
    }

    fn send_packet(&mut self, packet: &ProtocolPacket) -> Result<(), ProtocolError> {
        let address = packet.address as u8;
        
        match packet.direction {
            DataDirection::Write => {
                if let Some(register) = packet.register {
                    let mut data = Vec::new();
                    data.push(register as u8)
                        .map_err(|_| ProtocolError::BufferError)?;
                    data.extend_from_slice(&packet.data)
                        .map_err(|_| ProtocolError::BufferError)?;
                    
                    self.write_device(address, &data)
                        .map_err(|e| ProtocolError::I2C(e))
                } else {
                    self.write_device(address, &packet.data)
                        .map_err(|e| ProtocolError::I2C(e))
                }
            },
            _ => Err(ProtocolError::UnsupportedProtocol),
        }
    }

    fn receive_packet(&mut self) -> Result<ProtocolPacket, ProtocolError> {
        // I2C是主从协议，主设备不会主动接收数据包
        Err(ProtocolError::UnsupportedProtocol)
    }

    fn read_data(&mut self, address: u32, register: Option<u16>, length: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        let addr = address as u8;
        
        if let Some(reg) = register {
            self.read_registers(addr, reg as u8, length)
                .map_err(|e| ProtocolError::I2C(e))
        } else {
            self.read_device(addr, length)
                .map_err(|e| ProtocolError::I2C(e))
        }
    }

    fn write_data(&mut self, address: u32, register: Option<u16>, data: &[u8]) -> Result<(), ProtocolError> {
        let addr = address as u8;
        
        if let Some(reg) = register {
            self.write_registers(addr, reg as u8, data)
                .map_err(|e| ProtocolError::I2C(e))
        } else {
            self.write_device(addr, data)
                .map_err(|e| ProtocolError::I2C(e))
        }
    }

    fn device_exists(&mut self, address: u32) -> Result<bool, ProtocolError> {
        self.probe_device(address as u8)
            .map_err(|e| ProtocolError::I2C(e))
    }

    fn scan_devices(&mut self) -> Result<Vec<u32, 128>, ProtocolError> {
        let devices = self.scan_bus()
            .map_err(|e| ProtocolError::I2C(e))?;
        
        let mut result = Vec::new();
        for address in devices {
            result.push(address as u32)
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
            device_info.stats = I2CDeviceStats::default();
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

/// I2C设备包装器
pub struct I2CDevice<I2C, D>
where
    I2C: Read + Write + WriteRead,
    D: DelayMs<u32>,
{
    /// I2C管理器引用
    manager: *mut I2CManager<I2C, D>,
    /// 设备地址
    address: u8,
}

impl<I2C, D> I2CDevice<I2C, D>
where
    I2C: Read + Write + WriteRead,
    D: DelayMs<u32>,
{
    /// 创建新的I2C设备
    pub fn new(manager: &mut I2CManager<I2C, D>, address: u8) -> Self {
        Self {
            manager: manager as *mut _,
            address,
        }
    }

    /// 获取管理器引用
    fn get_manager(&mut self) -> &mut I2CManager<I2C, D> {
        unsafe { &mut *self.manager }
    }
}

impl<I2C, D> ProtocolDevice for I2CDevice<I2C, D>
where
    I2C: Read + Write + WriteRead,
    D: DelayMs<u32>,
{
    fn read_register(&mut self, register: u16) -> Result<u8, ProtocolError> {
        self.get_manager()
            .read_register(self.address, register as u8)
            .map_err(|e| ProtocolError::I2C(e))
    }

    fn write_register(&mut self, register: u16, value: u8) -> Result<(), ProtocolError> {
        self.get_manager()
            .write_register(self.address, register as u8, value)
            .map_err(|e| ProtocolError::I2C(e))
    }

    fn read_registers(&mut self, start_register: u16, count: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        self.get_manager()
            .read_registers(self.address, start_register as u8, count)
            .map_err(|e| ProtocolError::I2C(e))
    }

    fn write_registers(&mut self, start_register: u16, data: &[u8]) -> Result<(), ProtocolError> {
        self.get_manager()
            .write_registers(self.address, start_register as u8, data)
            .map_err(|e| ProtocolError::I2C(e))
    }

    fn read_block(&mut self, length: usize) -> Result<Vec<u8, 256>, ProtocolError> {
        self.get_manager()
            .read_device(self.address, length)
            .map_err(|e| ProtocolError::I2C(e))
    }

    fn write_block(&mut self, data: &[u8]) -> Result<(), ProtocolError> {
        self.get_manager()
            .write_device(self.address, data)
            .map_err(|e| ProtocolError::I2C(e))
    }

    fn get_address(&self) -> u32 {
        self.address as u32
    }

    fn set_address(&mut self, address: u32) {
        self.address = address as u8;
    }

    fn is_connected(&mut self) -> Result<bool, ProtocolError> {
        self.get_manager()
            .probe_device(self.address)
            .map_err(|e| ProtocolError::I2C(e))
    }

    fn reset(&mut self) -> Result<(), ProtocolError> {
        // I2C设备通常没有硬件复位，可以尝试软件复位
        // 这里可以发送设备特定的复位命令
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 模拟I2C实现用于测试
    struct MockI2C {
        fail_operations: bool,
    }

    impl MockI2C {
        fn new() -> Self {
            Self {
                fail_operations: false,
            }
        }
    }

    impl Read for MockI2C {
        type Error = ();

        fn read(&mut self, _address: u8, _buffer: &mut [u8]) -> Result<(), Self::Error> {
            if self.fail_operations {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    impl Write for MockI2C {
        type Error = ();

        fn write(&mut self, _address: u8, _data: &[u8]) -> Result<(), Self::Error> {
            if self.fail_operations {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    impl WriteRead for MockI2C {
        type Error = ();

        fn write_read(&mut self, _address: u8, _bytes: &[u8], _buffer: &mut [u8]) -> Result<(), Self::Error> {
            if self.fail_operations {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    struct MockDelay;

    impl DelayMs<u32> for MockDelay {
        fn delay_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_i2c_manager_creation() {
        let i2c = MockI2C::new();
        let delay = MockDelay;
        let config = I2CConfig::default();
        
        let manager = I2CManager::new(i2c, delay, config);
        assert_eq!(manager.get_bus_state(), BusState::Idle);
    }

    #[test]
    fn test_address_validation() {
        let i2c = MockI2C::new();
        let delay = MockDelay;
        let config = I2CConfig::default();
        
        let manager = I2CManager::new(i2c, delay, config);
        
        assert!(manager.is_valid_address(0x44)); // 有效地址
        assert!(!manager.is_valid_address(0x00)); // 保留地址
        assert!(!manager.is_valid_address(0x78)); // 超出范围
    }
}