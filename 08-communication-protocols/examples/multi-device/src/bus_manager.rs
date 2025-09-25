use embedded_hal::{i2c::I2c, spi::SpiDevice};
use heapless::{Vec, FnvIndexMap};
use crate::{
    DeviceInfo, DeviceType, DeviceStatus, Priority, BusType, 
    CommunicationError, PerformanceStats, i2c_addresses
};

/// 总线操作请求
#[derive(Debug, Clone)]
pub struct BusRequest {
    pub device_id: u8,
    pub priority: Priority,
    pub operation_type: OperationType,
    pub data: Vec<u8, 256>,
    pub timeout_ms: u32,
}

/// 操作类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperationType {
    Read,
    Write,
    WriteRead,
    Scan,
}

/// 总线管理器
pub struct BusManager<I2C, SPI> {
    /// I2C总线
    i2c: Option<I2C>,
    /// SPI总线
    spi: Option<SPI>,
    /// 设备列表
    devices: Vec<DeviceInfo, 16>,
    /// 请求队列
    request_queue: Vec<BusRequest, 32>,
    /// 总线锁定状态
    i2c_locked: bool,
    spi_locked: bool,
    /// 系统时钟计数器
    system_tick: u32,
    /// 全局性能统计
    global_stats: PerformanceStats,
}

impl<I2C, SPI, E1, E2> BusManager<I2C, SPI>
where
    I2C: I2c<Error = E1>,
    SPI: SpiDevice<Error = E2>,
{
    /// 创建新的总线管理器
    pub fn new() -> Self {
        Self {
            i2c: None,
            spi: None,
            devices: Vec::new(),
            request_queue: Vec::new(),
            i2c_locked: false,
            spi_locked: false,
            system_tick: 0,
            global_stats: PerformanceStats::new(),
        }
    }
    
    /// 设置I2C总线
    pub fn set_i2c(&mut self, i2c: I2C) {
        self.i2c = Some(i2c);
    }
    
    /// 设置SPI总线
    pub fn set_spi(&mut self, spi: SPI) {
        self.spi = Some(spi);
    }
    
    /// 更新系统时钟
    pub fn update_tick(&mut self) {
        self.system_tick = self.system_tick.wrapping_add(1);
    }
    
    /// 扫描I2C总线设备
    pub fn scan_i2c_devices(&mut self) -> Result<Vec<u8, 16>, CommunicationError> {
        if self.i2c.is_none() {
            return Err(CommunicationError::BusError);
        }
        
        let mut found_devices = Vec::new();
        let i2c = self.i2c.as_mut().unwrap();
        
        // 扫描标准I2C地址范围
        for addr in 0x08..=0x77 {
            let mut buffer = [0u8; 1];
            if i2c.read(addr, &mut buffer).is_ok() {
                found_devices.push(addr).ok();
                
                // 自动识别已知设备
                let device_name = match addr {
                    i2c_addresses::PCF8574 => "PCF8574 IO Expander",
                    i2c_addresses::AT24C256 => "AT24C256 EEPROM",
                    i2c_addresses::DS3231_RTC => "DS3231 RTC",
                    i2c_addresses::BMP280 => "BMP280 Sensor",
                    i2c_addresses::BMP280_ALT => "BMP280 Sensor (Alt)",
                    _ => "Unknown I2C Device",
                };
                
                // 添加到设备列表
                let device_id = self.devices.len() as u8;
                let device_info = DeviceInfo::new(device_id, DeviceType::I2c, addr, device_name);
                self.devices.push(device_info).ok();
            }
        }
        
        Ok(found_devices)
    }
    
    /// 注册SPI设备
    pub fn register_spi_device(&mut self, cs_pin: u8, name: &str) -> Result<u8, CommunicationError> {
        if self.devices.len() >= 16 {
            return Err(CommunicationError::BufferFull);
        }
        
        let device_id = self.devices.len() as u8;
        let device_info = DeviceInfo::new(device_id, DeviceType::Spi, cs_pin, name);
        self.devices.push(device_info)
            .map_err(|_| CommunicationError::BufferFull)?;
        
        Ok(device_id)
    }
    
    /// 获取设备信息
    pub fn get_device(&self, device_id: u8) -> Option<&DeviceInfo> {
        self.devices.iter().find(|dev| dev.id == device_id)
    }
    
    /// 获取设备信息（可变）
    pub fn get_device_mut(&mut self, device_id: u8) -> Option<&mut DeviceInfo> {
        self.devices.iter_mut().find(|dev| dev.id == device_id)
    }
    
    /// 设置设备优先级
    pub fn set_device_priority(&mut self, device_id: u8, priority: Priority) -> Result<(), CommunicationError> {
        if let Some(device) = self.get_device_mut(device_id) {
            device.priority = priority;
            Ok(())
        } else {
            Err(CommunicationError::InvalidParameter)
        }
    }
    
    /// 检查设备状态
    pub fn check_device_status(&mut self, device_id: u8) -> Result<DeviceStatus, CommunicationError> {
        let device = self.get_device(device_id)
            .ok_or(CommunicationError::InvalidParameter)?;
        
        let status = match device.device_type {
            DeviceType::I2c => self.check_i2c_device_status(device.address),
            DeviceType::Spi => self.check_spi_device_status(device.address),
            DeviceType::Unknown => DeviceStatus::Unknown,
        };
        
        // 更新设备状态
        if let Some(device) = self.get_device_mut(device_id) {
            device.status = status;
            device.last_access_time = self.system_tick;
        }
        
        Ok(status)
    }
    
    /// 检查I2C设备状态
    fn check_i2c_device_status(&mut self, address: u8) -> DeviceStatus {
        if let Some(ref mut i2c) = self.i2c {
            let mut buffer = [0u8; 1];
            match i2c.read(address, &mut buffer) {
                Ok(_) => DeviceStatus::Online,
                Err(_) => DeviceStatus::Offline,
            }
        } else {
            DeviceStatus::Error
        }
    }
    
    /// 检查SPI设备状态
    fn check_spi_device_status(&mut self, _cs_pin: u8) -> DeviceStatus {
        // SPI设备状态检查需要具体的设备协议
        // 这里简化为在线状态
        DeviceStatus::Online
    }
    
    /// 添加总线请求
    pub fn add_request(&mut self, request: BusRequest) -> Result<(), CommunicationError> {
        if self.request_queue.len() >= 32 {
            return Err(CommunicationError::BufferFull);
        }
        
        // 按优先级插入请求
        let insert_pos = self.request_queue.iter()
            .position(|req| req.priority < request.priority)
            .unwrap_or(self.request_queue.len());
        
        self.request_queue.insert(insert_pos, request)
            .map_err(|_| CommunicationError::BufferFull)?;
        
        Ok(())
    }
    
    /// 处理请求队列
    pub fn process_requests(&mut self) -> Result<u8, CommunicationError> {
        let mut processed_count = 0;
        
        while let Some(request) = self.request_queue.pop() {
            let start_time = self.system_tick;
            let result = self.execute_request(&request);
            let end_time = self.system_tick;
            
            // 更新统计信息
            self.update_stats(request.device_id, result.is_ok(), end_time - start_time);
            
            processed_count += 1;
            
            // 限制每次处理的请求数量，避免阻塞
            if processed_count >= 8 {
                break;
            }
        }
        
        Ok(processed_count)
    }
    
    /// 执行单个请求
    fn execute_request(&mut self, request: &BusRequest) -> Result<(), CommunicationError> {
        let device = self.get_device(request.device_id)
            .ok_or(CommunicationError::InvalidParameter)?;
        
        match device.device_type {
            DeviceType::I2c => self.execute_i2c_request(device.address, request),
            DeviceType::Spi => self.execute_spi_request(device.address, request),
            DeviceType::Unknown => Err(CommunicationError::InvalidParameter),
        }
    }
    
    /// 执行I2C请求
    fn execute_i2c_request(&mut self, address: u8, request: &BusRequest) -> Result<(), CommunicationError> {
        if self.i2c_locked {
            return Err(CommunicationError::BusError);
        }
        
        self.i2c_locked = true;
        
        let result = if let Some(ref mut i2c) = self.i2c {
            match request.operation_type {
                OperationType::Read => {
                    let mut buffer = [0u8; 256];
                    let len = request.data.len().min(256);
                    i2c.read(address, &mut buffer[..len])
                        .map_err(|_| CommunicationError::BusError)
                }
                OperationType::Write => {
                    i2c.write(address, &request.data)
                        .map_err(|_| CommunicationError::BusError)
                }
                OperationType::WriteRead => {
                    let mut read_buffer = [0u8; 128];
                    let write_len = request.data.len() / 2;
                    let read_len = request.data.len() - write_len;
                    
                    i2c.write_read(address, &request.data[..write_len], &mut read_buffer[..read_len])
                        .map_err(|_| CommunicationError::BusError)
                }
                OperationType::Scan => {
                    let mut buffer = [0u8; 1];
                    i2c.read(address, &mut buffer)
                        .map_err(|_| CommunicationError::NoResponse)
                }
            }
        } else {
            Err(CommunicationError::BusError)
        };
        
        self.i2c_locked = false;
        result
    }
    
    /// 执行SPI请求
    fn execute_spi_request(&mut self, _cs_pin: u8, request: &BusRequest) -> Result<(), CommunicationError> {
        if self.spi_locked {
            return Err(CommunicationError::BusError);
        }
        
        self.spi_locked = true;
        
        let result = if let Some(ref mut spi) = self.spi {
            match request.operation_type {
                OperationType::Write => {
                    spi.write(&request.data)
                        .map_err(|_| CommunicationError::BusError)
                }
                OperationType::Read => {
                    let mut buffer = vec![0u8; request.data.len()];
                    spi.read(&mut buffer)
                        .map_err(|_| CommunicationError::BusError)
                }
                OperationType::WriteRead => {
                    let mut read_buffer = vec![0u8; request.data.len()];
                    spi.transfer(&mut read_buffer, &request.data)
                        .map_err(|_| CommunicationError::BusError)
                }
                OperationType::Scan => {
                    // SPI设备扫描需要具体协议支持
                    Ok(())
                }
            }
        } else {
            Err(CommunicationError::BusError)
        };
        
        self.spi_locked = false;
        result
    }
    
    /// 更新统计信息
    fn update_stats(&mut self, device_id: u8, success: bool, response_time: u32) {
        // 更新全局统计
        self.global_stats.total_operations += 1;
        if success {
            self.global_stats.successful_operations += 1;
        } else {
            self.global_stats.failed_operations += 1;
            self.global_stats.last_error_timestamp = self.system_tick;
        }
        
        // 更新平均响应时间
        let total_ops = self.global_stats.total_operations;
        let current_avg = self.global_stats.average_response_time_us;
        self.global_stats.average_response_time_us = 
            (current_avg * (total_ops - 1) + response_time) / total_ops;
        
        // 更新设备统计
        if let Some(device) = self.get_device_mut(device_id) {
            device.stats.total_operations += 1;
            if success {
                device.stats.successful_operations += 1;
            } else {
                device.stats.failed_operations += 1;
                device.stats.last_error_timestamp = self.system_tick;
            }
            
            // 更新设备平均响应时间
            let dev_total_ops = device.stats.total_operations;
            let dev_current_avg = device.stats.average_response_time_us;
            device.stats.average_response_time_us = 
                (dev_current_avg * (dev_total_ops - 1) + response_time) / dev_total_ops;
        }
    }
    
    /// 获取全局统计信息
    pub fn get_global_stats(&self) -> &PerformanceStats {
        &self.global_stats
    }
    
    /// 获取设备列表
    pub fn get_devices(&self) -> &[DeviceInfo] {
        &self.devices
    }
    
    /// 获取在线设备数量
    pub fn get_online_device_count(&self) -> usize {
        self.devices.iter()
            .filter(|dev| dev.status == DeviceStatus::Online)
            .count()
    }
    
    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.global_stats = PerformanceStats::new();
        for device in &mut self.devices {
            device.stats = PerformanceStats::new();
        }
    }
    
    /// 清理离线设备
    pub fn cleanup_offline_devices(&mut self) {
        self.devices.retain(|dev| dev.status != DeviceStatus::Offline);
    }
    
    /// 获取总线使用率
    pub fn calculate_bus_utilization(&mut self) -> (u8, u8) {
        // 简化的总线使用率计算
        let i2c_util = if self.i2c_locked { 100 } else { 
            (self.request_queue.iter()
                .filter(|req| {
                    if let Some(dev) = self.get_device(req.device_id) {
                        dev.device_type == DeviceType::I2c
                    } else {
                        false
                    }
                })
                .count() * 10).min(100) as u8
        };
        
        let spi_util = if self.spi_locked { 100 } else {
            (self.request_queue.iter()
                .filter(|req| {
                    if let Some(dev) = self.get_device(req.device_id) {
                        dev.device_type == DeviceType::Spi
                    } else {
                        false
                    }
                })
                .count() * 10).min(100) as u8
        };
        
        (i2c_util, spi_util)
    }
}