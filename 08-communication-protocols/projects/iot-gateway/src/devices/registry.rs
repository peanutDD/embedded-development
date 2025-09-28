//! 设备注册表
//! 
//! 负责IoT网关中设备信息的存储、查询和管理。
//! 提供设备发现、注册、配置管理等功能。

use heapless::{String, Vec, FnvIndexMap};
use serde::{Deserialize, Serialize};
use embassy_time::Instant;

use super::{
    DeviceError, DeviceConfig, DeviceType, InterfaceType, DeviceState, DevicePriority,
    device_utils,
};

/// 设备注册表配置
#[derive(Debug, Clone)]
pub struct DeviceRegistryConfig {
    /// 最大设备数量
    pub max_devices: usize,
    /// 启用持久化存储
    pub persistent_storage: bool,
    /// 自动保存间隔（毫秒）
    pub auto_save_interval_ms: u32,
    /// 设备信息缓存大小
    pub cache_size: usize,
    /// 启用设备分组
    pub grouping_enabled: bool,
    /// 启用设备标签
    pub tagging_enabled: bool,
}

impl Default for DeviceRegistryConfig {
    fn default() -> Self {
        Self {
            max_devices: 64,
            persistent_storage: true,
            auto_save_interval_ms: 300000, // 5分钟
            cache_size: 32,
            grouping_enabled: true,
            tagging_enabled: true,
        }
    }
}

/// 设备信息
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceInfo {
    /// 设备名称
    pub name: String<32>,
    /// 设备ID
    pub device_id: String<32>,
    /// 设备类型
    pub device_type: DeviceType,
    /// 接口类型
    pub interface_type: InterfaceType,
    /// 设备地址
    pub address: u32,
    /// 制造商
    pub manufacturer: String<32>,
    /// 型号
    pub model: String<32>,
    /// 固件版本
    pub firmware_version: String<16>,
    /// 硬件版本
    pub hardware_version: String<16>,
    /// 序列号
    pub serial_number: String<32>,
    /// 设备描述
    pub description: String<64>,
    /// 设备位置
    pub location: String<32>,
    /// 设备标签
    pub tags: Vec<String<16>, 8>,
    /// 设备组
    pub group: String<16>,
    /// 注册时间
    pub registered_at: u64,
    /// 最后更新时间
    pub last_updated: u64,
    /// 设备配置
    pub config: DeviceConfig,
}

impl Default for DeviceInfo {
    fn default() -> Self {
        Self {
            name: String::new(),
            device_id: String::new(),
            device_type: DeviceType::Other,
            interface_type: InterfaceType::Virtual,
            address: 0,
            manufacturer: String::new(),
            model: String::new(),
            firmware_version: String::new(),
            hardware_version: String::new(),
            serial_number: String::new(),
            description: String::new(),
            location: String::new(),
            tags: Vec::new(),
            group: String::new(),
            registered_at: 0,
            last_updated: 0,
            config: DeviceConfig::default(),
        }
    }
}

impl DeviceInfo {
    /// 创建新的设备信息
    pub fn new(name: &str, device_type: DeviceType, interface_type: InterfaceType, address: u32) -> Result<Self, DeviceError> {
        device_utils::validate_device_name(name)?;
        
        let device_id = device_utils::generate_device_id(device_type, interface_type, address);
        let current_time = Self::get_current_timestamp();
        
        let mut info = Self::default();
        info.name = String::from_str(name).map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;
        info.device_id = device_id;
        info.device_type = device_type;
        info.interface_type = interface_type;
        info.address = address;
        info.registered_at = current_time;
        info.last_updated = current_time;
        
        Ok(info)
    }

    /// 添加标签
    pub fn add_tag(&mut self, tag: &str) -> Result<(), DeviceError> {
        let tag_str = String::from_str(tag)
            .map_err(|_| DeviceError::ConfigurationError("Invalid tag"))?;
        
        if !self.tags.contains(&tag_str) {
            self.tags.push(tag_str)
                .map_err(|_| DeviceError::ResourceExhausted)?;
        }
        
        self.last_updated = Self::get_current_timestamp();
        Ok(())
    }

    /// 移除标签
    pub fn remove_tag(&mut self, tag: &str) -> Result<(), DeviceError> {
        let tag_str = String::from_str(tag)
            .map_err(|_| DeviceError::ConfigurationError("Invalid tag"))?;
        
        self.tags.retain(|t| t != &tag_str);
        self.last_updated = Self::get_current_timestamp();
        Ok(())
    }

    /// 检查是否有指定标签
    pub fn has_tag(&self, tag: &str) -> bool {
        if let Ok(tag_str) = String::from_str(tag) {
            self.tags.contains(&tag_str)
        } else {
            false
        }
    }

    /// 设置设备组
    pub fn set_group(&mut self, group: &str) -> Result<(), DeviceError> {
        self.group = String::from_str(group)
            .map_err(|_| DeviceError::ConfigurationError("Invalid group name"))?;
        self.last_updated = Self::get_current_timestamp();
        Ok(())
    }

    /// 更新设备信息
    pub fn update_info(&mut self, manufacturer: &str, model: &str, firmware_version: &str) -> Result<(), DeviceError> {
        self.manufacturer = String::from_str(manufacturer)
            .map_err(|_| DeviceError::ConfigurationError("Invalid manufacturer"))?;
        self.model = String::from_str(model)
            .map_err(|_| DeviceError::ConfigurationError("Invalid model"))?;
        self.firmware_version = String::from_str(firmware_version)
            .map_err(|_| DeviceError::ConfigurationError("Invalid firmware version"))?;
        
        self.last_updated = Self::get_current_timestamp();
        Ok(())
    }

    /// 获取当前时间戳
    fn get_current_timestamp() -> u64 {
        // 这里应该使用实际的时间获取函数
        // 为了演示，返回一个模拟值
        1000000 // 模拟时间戳
    }
}

/// 设备状态信息
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceStatus {
    /// 设备名称
    pub device_name: String<32>,
    /// 当前状态
    pub state: DeviceState,
    /// 在线状态
    pub online: bool,
    /// 最后通信时间
    pub last_communication: u64,
    /// 健康评分 (0-100)
    pub health_score: u8,
    /// 错误计数
    pub error_count: u32,
    /// 运行时间（秒）
    pub uptime_seconds: u64,
    /// CPU使用率（百分比）
    pub cpu_usage: u8,
    /// 内存使用率（百分比）
    pub memory_usage: u8,
    /// 温度（摄氏度）
    pub temperature: f32,
    /// 电池电量（百分比，如果适用）
    pub battery_level: Option<u8>,
    /// 信号强度（dBm，如果适用）
    pub signal_strength: Option<i8>,
}

impl Default for DeviceStatus {
    fn default() -> Self {
        Self {
            device_name: String::new(),
            state: DeviceState::Uninitialized,
            online: false,
            last_communication: 0,
            health_score: 100,
            error_count: 0,
            uptime_seconds: 0,
            cpu_usage: 0,
            memory_usage: 0,
            temperature: 25.0,
            battery_level: None,
            signal_strength: None,
        }
    }
}

/// 设备查询条件
#[derive(Debug, Clone, Default)]
pub struct DeviceQuery {
    /// 设备类型过滤
    pub device_type: Option<DeviceType>,
    /// 接口类型过滤
    pub interface_type: Option<InterfaceType>,
    /// 设备组过滤
    pub group: Option<String<16>>,
    /// 标签过滤
    pub tags: Vec<String<16>, 8>,
    /// 制造商过滤
    pub manufacturer: Option<String<32>>,
    /// 在线状态过滤
    pub online_only: bool,
    /// 健康评分最小值
    pub min_health_score: Option<u8>,
    /// 最大返回数量
    pub limit: Option<usize>,
}

/// 设备组信息
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceGroup {
    /// 组名
    pub name: String<16>,
    /// 组描述
    pub description: String<64>,
    /// 组成员设备
    pub members: Vec<String<32>, 32>,
    /// 组配置
    pub config: GroupConfig,
    /// 创建时间
    pub created_at: u64,
    /// 最后更新时间
    pub last_updated: u64,
}

/// 设备组配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GroupConfig {
    /// 同步采样
    pub sync_sampling: bool,
    /// 采样间隔（毫秒）
    pub sample_interval_ms: u32,
    /// 数据聚合
    pub data_aggregation: bool,
    /// 故障转移
    pub failover_enabled: bool,
    /// 负载均衡
    pub load_balancing: bool,
}

impl Default for GroupConfig {
    fn default() -> Self {
        Self {
            sync_sampling: false,
            sample_interval_ms: 1000,
            data_aggregation: false,
            failover_enabled: false,
            load_balancing: false,
        }
    }
}

/// 设备注册表
pub struct DeviceRegistry {
    /// 配置
    config: DeviceRegistryConfig,
    /// 设备信息映射
    devices: FnvIndexMap<String<32>, DeviceInfo, 64>,
    /// 设备状态映射
    device_status: FnvIndexMap<String<32>, DeviceStatus, 64>,
    /// 设备组映射
    device_groups: FnvIndexMap<String<16>, DeviceGroup, 16>,
    /// 按类型索引
    type_index: FnvIndexMap<DeviceType, Vec<String<32>, 32>, 8>,
    /// 按接口索引
    interface_index: FnvIndexMap<InterfaceType, Vec<String<32>, 32>, 8>,
    /// 按组索引
    group_index: FnvIndexMap<String<16>, Vec<String<32>, 32>, 16>,
    /// 标签索引
    tag_index: FnvIndexMap<String<16>, Vec<String<32>, 32>, 32>,
    /// 最后保存时间
    last_save_time: Instant,
}

impl DeviceRegistry {
    /// 创建新的设备注册表
    pub fn new(config: DeviceRegistryConfig) -> Self {
        Self {
            config,
            devices: FnvIndexMap::new(),
            device_status: FnvIndexMap::new(),
            device_groups: FnvIndexMap::new(),
            type_index: FnvIndexMap::new(),
            interface_index: FnvIndexMap::new(),
            group_index: FnvIndexMap::new(),
            tag_index: FnvIndexMap::new(),
            last_save_time: Instant::now(),
        }
    }

    /// 注册设备
    pub fn register_device(&mut self, device_info: DeviceInfo) -> Result<(), DeviceError> {
        // 检查设备是否已存在
        if self.devices.contains_key(&device_info.name) {
            return Err(DeviceError::DeviceAlreadyExists(device_info.name.clone()));
        }

        // 检查设备数量限制
        if self.devices.len() >= self.config.max_devices {
            return Err(DeviceError::ResourceExhausted);
        }

        // 验证设备信息
        device_utils::validate_device_name(&device_info.name)?;

        // 添加到主映射
        let device_name = device_info.name.clone();
        self.devices.insert(device_name.clone(), device_info.clone())
            .map_err(|_| DeviceError::ResourceExhausted)?;

        // 创建默认状态
        let mut status = DeviceStatus::default();
        status.device_name = device_name.clone();
        self.device_status.insert(device_name.clone(), status)
            .map_err(|_| DeviceError::ResourceExhausted)?;

        // 更新索引
        self.update_indexes(&device_info)?;

        Ok(())
    }

    /// 注销设备
    pub fn unregister_device(&mut self, device_name: &str) -> Result<DeviceInfo, DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        // 获取设备信息
        let device_info = self.devices.remove(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str.clone()))?;

        // 移除状态信息
        self.device_status.remove(&device_name_str);

        // 从组中移除
        if !device_info.group.is_empty() {
            if let Some(group) = self.device_groups.get_mut(&device_info.group) {
                group.members.retain(|name| name != &device_name_str);
            }
        }

        // 更新索引
        self.remove_from_indexes(&device_info);

        Ok(device_info)
    }

    /// 更新设备信息
    pub fn update_device_info(&mut self, device_name: &str, device_info: DeviceInfo) -> Result<(), DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        // 检查设备是否存在
        let old_info = self.devices.get(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str.clone()))?
            .clone();

        // 更新设备信息
        self.devices.insert(device_name_str, device_info.clone())
            .map_err(|_| DeviceError::ResourceExhausted)?;

        // 如果索引相关信息发生变化，更新索引
        if old_info.device_type != device_info.device_type ||
           old_info.interface_type != device_info.interface_type ||
           old_info.group != device_info.group ||
           old_info.tags != device_info.tags {
            
            self.remove_from_indexes(&old_info);
            self.update_indexes(&device_info)?;
        }

        Ok(())
    }

    /// 获取设备信息
    pub fn get_device_info(&self, device_name: &str) -> Result<&DeviceInfo, DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        self.devices.get(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str))
    }

    /// 获取设备状态
    pub fn get_device_status(&self, device_name: &str) -> Result<&DeviceStatus, DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        self.device_status.get(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str))
    }

    /// 更新设备状态
    pub fn update_device_status(&mut self, device_name: &str, status: DeviceStatus) -> Result<(), DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;

        if !self.devices.contains_key(&device_name_str) {
            return Err(DeviceError::DeviceNotFound(device_name_str));
        }

        self.device_status.insert(device_name_str, status)
            .map_err(|_| DeviceError::ResourceExhausted)?;

        Ok(())
    }

    /// 查询设备
    pub fn query_devices(&self, query: &DeviceQuery) -> Vec<&DeviceInfo, 32> {
        let mut results = Vec::new();
        let mut count = 0;
        let limit = query.limit.unwrap_or(usize::MAX);

        for device_info in self.devices.values() {
            if count >= limit {
                break;
            }

            // 应用过滤条件
            if let Some(device_type) = query.device_type {
                if device_info.device_type != device_type {
                    continue;
                }
            }

            if let Some(interface_type) = query.interface_type {
                if device_info.interface_type != interface_type {
                    continue;
                }
            }

            if let Some(ref group) = query.group {
                if &device_info.group != group {
                    continue;
                }
            }

            if let Some(ref manufacturer) = query.manufacturer {
                if &device_info.manufacturer != manufacturer {
                    continue;
                }
            }

            // 检查标签
            if !query.tags.is_empty() {
                let mut has_all_tags = true;
                for tag in &query.tags {
                    if !device_info.tags.contains(tag) {
                        has_all_tags = false;
                        break;
                    }
                }
                if !has_all_tags {
                    continue;
                }
            }

            // 检查在线状态
            if query.online_only {
                if let Some(status) = self.device_status.get(&device_info.name) {
                    if !status.online {
                        continue;
                    }
                } else {
                    continue;
                }
            }

            // 检查健康评分
            if let Some(min_health) = query.min_health_score {
                if let Some(status) = self.device_status.get(&device_info.name) {
                    if status.health_score < min_health {
                        continue;
                    }
                } else {
                    continue;
                }
            }

            if results.push(device_info).is_err() {
                break;
            }
            count += 1;
        }

        results
    }

    /// 获取所有设备名称
    pub fn get_all_device_names(&self) -> Vec<String<32>, 64> {
        let mut names = Vec::new();
        
        for name in self.devices.keys() {
            if names.push(name.clone()).is_err() {
                break;
            }
        }
        
        names
    }

    /// 按类型获取设备
    pub fn get_devices_by_type(&self, device_type: DeviceType) -> Vec<&DeviceInfo, 32> {
        let mut devices = Vec::new();
        
        if let Some(device_names) = self.type_index.get(&device_type) {
            for device_name in device_names {
                if let Some(device_info) = self.devices.get(device_name) {
                    if devices.push(device_info).is_err() {
                        break;
                    }
                }
            }
        }
        
        devices
    }

    /// 按接口获取设备
    pub fn get_devices_by_interface(&self, interface_type: InterfaceType) -> Vec<&DeviceInfo, 32> {
        let mut devices = Vec::new();
        
        if let Some(device_names) = self.interface_index.get(&interface_type) {
            for device_name in device_names {
                if let Some(device_info) = self.devices.get(device_name) {
                    if devices.push(device_info).is_err() {
                        break;
                    }
                }
            }
        }
        
        devices
    }

    /// 创建设备组
    pub fn create_device_group(&mut self, name: &str, description: &str, config: GroupConfig) -> Result<(), DeviceError> {
        let group_name = String::from_str(name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid group name"))?;

        if self.device_groups.contains_key(&group_name) {
            return Err(DeviceError::DeviceAlreadyExists(String::from_str(name).unwrap_or_default()));
        }

        let group = DeviceGroup {
            name: group_name.clone(),
            description: String::from_str(description).unwrap_or_default(),
            members: Vec::new(),
            config,
            created_at: DeviceInfo::get_current_timestamp(),
            last_updated: DeviceInfo::get_current_timestamp(),
        };

        self.device_groups.insert(group_name.clone(), group)
            .map_err(|_| DeviceError::ResourceExhausted)?;

        // 初始化组索引
        self.group_index.insert(group_name, Vec::new())
            .map_err(|_| DeviceError::ResourceExhausted)?;

        Ok(())
    }

    /// 添加设备到组
    pub fn add_device_to_group(&mut self, device_name: &str, group_name: &str) -> Result<(), DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;
        let group_name_str = String::from_str(group_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid group name"))?;

        // 检查设备是否存在
        let mut device_info = self.devices.get(&device_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(device_name_str.clone()))?
            .clone();

        // 检查组是否存在
        let group = self.device_groups.get_mut(&group_name_str)
            .ok_or_else(|| DeviceError::DeviceNotFound(group_name_str.clone()))?;

        // 添加到组成员
        if !group.members.contains(&device_name_str) {
            group.members.push(device_name_str.clone())
                .map_err(|_| DeviceError::ResourceExhausted)?;
        }

        // 更新设备的组信息
        device_info.group = group_name_str.clone();
        device_info.last_updated = DeviceInfo::get_current_timestamp();
        self.devices.insert(device_name_str.clone(), device_info)
            .map_err(|_| DeviceError::ResourceExhausted)?;

        // 更新组索引
        let group_devices = self.group_index.entry(group_name_str)
            .or_insert_with(Vec::new);
        
        if !group_devices.contains(&device_name_str) {
            group_devices.push(device_name_str)
                .map_err(|_| DeviceError::ResourceExhausted)?;
        }

        Ok(())
    }

    /// 从组中移除设备
    pub fn remove_device_from_group(&mut self, device_name: &str, group_name: &str) -> Result<(), DeviceError> {
        let device_name_str = String::from_str(device_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid device name"))?;
        let group_name_str = String::from_str(group_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid group name"))?;

        // 从组中移除
        if let Some(group) = self.device_groups.get_mut(&group_name_str) {
            group.members.retain(|name| name != &device_name_str);
        }

        // 更新设备信息
        if let Some(mut device_info) = self.devices.get(&device_name_str).cloned() {
            device_info.group = String::new();
            device_info.last_updated = DeviceInfo::get_current_timestamp();
            self.devices.insert(device_name_str.clone(), device_info)
                .map_err(|_| DeviceError::ResourceExhausted)?;
        }

        // 更新组索引
        if let Some(group_devices) = self.group_index.get_mut(&group_name_str) {
            group_devices.retain(|name| name != &device_name_str);
        }

        Ok(())
    }

    /// 获取组中的设备
    pub fn get_group_devices(&self, group_name: &str) -> Result<Vec<&DeviceInfo, 32>, DeviceError> {
        let group_name_str = String::from_str(group_name)
            .map_err(|_| DeviceError::ConfigurationError("Invalid group name"))?;

        let mut devices = Vec::new();
        
        if let Some(device_names) = self.group_index.get(&group_name_str) {
            for device_name in device_names {
                if let Some(device_info) = self.devices.get(device_name) {
                    if devices.push(device_info).is_err() {
                        break;
                    }
                }
            }
        }
        
        Ok(devices)
    }

    /// 更新索引
    fn update_indexes(&mut self, device_info: &DeviceInfo) -> Result<(), DeviceError> {
        let device_name = device_info.name.clone();

        // 更新类型索引
        let type_devices = self.type_index.entry(device_info.device_type)
            .or_insert_with(Vec::new);
        if !type_devices.contains(&device_name) {
            type_devices.push(device_name.clone())
                .map_err(|_| DeviceError::ResourceExhausted)?;
        }

        // 更新接口索引
        let interface_devices = self.interface_index.entry(device_info.interface_type)
            .or_insert_with(Vec::new);
        if !interface_devices.contains(&device_name) {
            interface_devices.push(device_name.clone())
                .map_err(|_| DeviceError::ResourceExhausted)?;
        }

        // 更新组索引
        if !device_info.group.is_empty() {
            let group_devices = self.group_index.entry(device_info.group.clone())
                .or_insert_with(Vec::new);
            if !group_devices.contains(&device_name) {
                group_devices.push(device_name.clone())
                    .map_err(|_| DeviceError::ResourceExhausted)?;
            }
        }

        // 更新标签索引
        for tag in &device_info.tags {
            let tag_devices = self.tag_index.entry(tag.clone())
                .or_insert_with(Vec::new);
            if !tag_devices.contains(&device_name) {
                tag_devices.push(device_name.clone())
                    .map_err(|_| DeviceError::ResourceExhausted)?;
            }
        }

        Ok(())
    }

    /// 从索引中移除
    fn remove_from_indexes(&mut self, device_info: &DeviceInfo) {
        let device_name = &device_info.name;

        // 从类型索引中移除
        if let Some(type_devices) = self.type_index.get_mut(&device_info.device_type) {
            type_devices.retain(|name| name != device_name);
        }

        // 从接口索引中移除
        if let Some(interface_devices) = self.interface_index.get_mut(&device_info.interface_type) {
            interface_devices.retain(|name| name != device_name);
        }

        // 从组索引中移除
        if !device_info.group.is_empty() {
            if let Some(group_devices) = self.group_index.get_mut(&device_info.group) {
                group_devices.retain(|name| name != device_name);
            }
        }

        // 从标签索引中移除
        for tag in &device_info.tags {
            if let Some(tag_devices) = self.tag_index.get_mut(tag) {
                tag_devices.retain(|name| name != device_name);
            }
        }
    }

    /// 获取统计信息
    pub fn get_statistics(&self) -> RegistryStatistics {
        let mut stats = RegistryStatistics::default();
        
        stats.total_devices = self.devices.len();
        stats.total_groups = self.device_groups.len();
        
        // 按类型统计
        for device_type in [DeviceType::Sensor, DeviceType::Actuator, DeviceType::Display, 
                           DeviceType::Storage, DeviceType::Communication, DeviceType::PowerManagement, 
                           DeviceType::Other] {
            if let Some(devices) = self.type_index.get(&device_type) {
                match device_type {
                    DeviceType::Sensor => stats.sensors = devices.len(),
                    DeviceType::Actuator => stats.actuators = devices.len(),
                    DeviceType::Display => stats.displays = devices.len(),
                    DeviceType::Storage => stats.storage_devices = devices.len(),
                    DeviceType::Communication => stats.communication_devices = devices.len(),
                    _ => {}
                }
            }
        }

        // 在线设备统计
        stats.online_devices = self.device_status.values()
            .filter(|status| status.online)
            .count();

        stats
    }

    /// 清空注册表
    pub fn clear(&mut self) {
        self.devices.clear();
        self.device_status.clear();
        self.device_groups.clear();
        self.type_index.clear();
        self.interface_index.clear();
        self.group_index.clear();
        self.tag_index.clear();
    }

    /// 获取设备数量
    pub fn device_count(&self) -> usize {
        self.devices.len()
    }

    /// 检查设备是否存在
    pub fn device_exists(&self, device_name: &str) -> bool {
        if let Ok(device_name_str) = String::from_str(device_name) {
            self.devices.contains_key(&device_name_str)
        } else {
            false
        }
    }
}

/// 注册表统计信息
#[derive(Debug, Clone, Default)]
pub struct RegistryStatistics {
    /// 设备总数
    pub total_devices: usize,
    /// 在线设备数
    pub online_devices: usize,
    /// 传感器数量
    pub sensors: usize,
    /// 执行器数量
    pub actuators: usize,
    /// 显示器数量
    pub displays: usize,
    /// 存储设备数量
    pub storage_devices: usize,
    /// 通信设备数量
    pub communication_devices: usize,
    /// 设备组总数
    pub total_groups: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_info_creation() {
        let device_info = DeviceInfo::new("test_sensor", DeviceType::Sensor, InterfaceType::I2C, 0x44);
        assert!(device_info.is_ok());
        
        let info = device_info.unwrap();
        assert_eq!(info.name, "test_sensor");
        assert_eq!(info.device_type, DeviceType::Sensor);
        assert_eq!(info.interface_type, InterfaceType::I2C);
        assert_eq!(info.address, 0x44);
    }

    #[test]
    fn test_device_registration() {
        let mut registry = DeviceRegistry::new(DeviceRegistryConfig::default());
        let device_info = DeviceInfo::new("test_sensor", DeviceType::Sensor, InterfaceType::I2C, 0x44).unwrap();
        
        assert!(registry.register_device(device_info).is_ok());
        assert_eq!(registry.device_count(), 1);
        assert!(registry.device_exists("test_sensor"));
    }

    #[test]
    fn test_device_query() {
        let mut registry = DeviceRegistry::new(DeviceRegistryConfig::default());
        
        // 注册几个设备
        let sensor1 = DeviceInfo::new("sensor1", DeviceType::Sensor, InterfaceType::I2C, 0x44).unwrap();
        let sensor2 = DeviceInfo::new("sensor2", DeviceType::Sensor, InterfaceType::SPI, 0x01).unwrap();
        let actuator1 = DeviceInfo::new("actuator1", DeviceType::Actuator, InterfaceType::GPIO, 12).unwrap();
        
        registry.register_device(sensor1).unwrap();
        registry.register_device(sensor2).unwrap();
        registry.register_device(actuator1).unwrap();
        
        // 查询传感器
        let query = DeviceQuery {
            device_type: Some(DeviceType::Sensor),
            ..Default::default()
        };
        
        let results = registry.query_devices(&query);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_device_groups() {
        let mut registry = DeviceRegistry::new(DeviceRegistryConfig::default());
        
        // 创建设备组
        let group_config = GroupConfig::default();
        assert!(registry.create_device_group("sensors", "Sensor group", group_config).is_ok());
        
        // 注册设备
        let device_info = DeviceInfo::new("temp_sensor", DeviceType::Sensor, InterfaceType::I2C, 0x44).unwrap();
        registry.register_device(device_info).unwrap();
        
        // 添加设备到组
        assert!(registry.add_device_to_group("temp_sensor", "sensors").is_ok());
        
        // 获取组中的设备
        let group_devices = registry.get_group_devices("sensors").unwrap();
        assert_eq!(group_devices.len(), 1);
        assert_eq!(group_devices[0].name, "temp_sensor");
    }
}