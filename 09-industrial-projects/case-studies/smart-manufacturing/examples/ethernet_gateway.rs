//! 以太网网关示例
//! 
//! 这个示例展示了工业以太网网关的基本功能：
//! - 多协议支持（Modbus TCP, EtherNet/IP等）
//! - 数据路由和转换
//! - 网络管理
//! - 安全通信

use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime};

/// 协议类型枚举
#[derive(Debug, Clone, PartialEq, Hash, Eq)]
enum ProtocolType {
    ModbusTcp,
    EtherNetIp,
    OpcUa,
    Http,
}

/// 网络设备信息
#[derive(Debug, Clone)]
struct NetworkDevice {
    id: u32,
    name: String,
    ip_address: IpAddr,
    port: u16,
    protocol: ProtocolType,
    status: DeviceStatus,
    last_seen: SystemTime,
}

/// 设备状态枚举
#[derive(Debug, Clone, PartialEq)]
enum DeviceStatus {
    Online,
    Offline,
    Error,
    Maintenance,
}

/// 数据包结构
#[derive(Debug, Clone)]
struct DataPacket {
    source_device: u32,
    destination_device: u32,
    protocol: ProtocolType,
    data: Vec<u8>,
    timestamp: SystemTime,
    priority: PacketPriority,
}

/// 数据包优先级
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum PacketPriority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
}

/// 路由规则
#[derive(Debug, Clone)]
struct RoutingRule {
    id: u32,
    source_protocol: ProtocolType,
    destination_protocol: ProtocolType,
    source_device_filter: Option<u32>,
    destination_device: u32,
    transformation: DataTransformation,
}

/// 数据转换类型
#[derive(Debug, Clone)]
enum DataTransformation {
    None,
    ModbusToOpcUa,
    EtherNetIpToHttp,
    Custom(String),
}

/// 网络统计信息
#[derive(Debug, Default)]
struct NetworkStatistics {
    packets_received: u64,
    packets_sent: u64,
    packets_dropped: u64,
    bytes_received: u64,
    bytes_sent: u64,
    errors: u64,
}

/// 以太网网关主结构
struct EthernetGateway {
    devices: Arc<Mutex<HashMap<u32, NetworkDevice>>>,
    routing_rules: Arc<Mutex<Vec<RoutingRule>>>,
    statistics: Arc<Mutex<NetworkStatistics>>,
    packet_queue: Arc<Mutex<Vec<DataPacket>>>,
    is_running: Arc<Mutex<bool>>,
}

impl EthernetGateway {
    fn new() -> Self {
        Self {
            devices: Arc::new(Mutex::new(HashMap::new())),
            routing_rules: Arc::new(Mutex::new(Vec::new())),
            statistics: Arc::new(Mutex::new(NetworkStatistics::default())),
            packet_queue: Arc::new(Mutex::new(Vec::new())),
            is_running: Arc::new(Mutex::new(false)),
        }
    }
    
    /// 添加网络设备
    fn add_device(&self, device: NetworkDevice) {
        let mut devices = self.devices.lock().unwrap();
        devices.insert(device.id, device);
    }
    
    /// 添加路由规则
    fn add_routing_rule(&self, rule: RoutingRule) {
        let mut rules = self.routing_rules.lock().unwrap();
        rules.push(rule);
    }
    
    /// 启动网关服务
    fn start(&self) {
        {
            let mut running = self.is_running.lock().unwrap();
            *running = true;
        }
        
        println!("以太网网关启动中...");
        
        // 启动设备监控线程
        self.start_device_monitor();
        
        // 启动数据包处理线程
        self.start_packet_processor();
        
        // 启动网络监听线程
        self.start_network_listener();
        
        println!("以太网网关已启动");
    }
    
    /// 停止网关服务
    fn stop(&self) {
        let mut running = self.is_running.lock().unwrap();
        *running = false;
        println!("以太网网关已停止");
    }
    
    /// 启动设备监控线程
    fn start_device_monitor(&self) {
        let devices = Arc::clone(&self.devices);
        let is_running = Arc::clone(&self.is_running);
        
        thread::spawn(move || {
            while *is_running.lock().unwrap() {
                {
                    let mut devices = devices.lock().unwrap();
                    let now = SystemTime::now();
                    
                    for device in devices.values_mut() {
                        // 检查设备超时
                        if let Ok(elapsed) = now.duration_since(device.last_seen) {
                            if elapsed > Duration::from_secs(30) {
                                if device.status == DeviceStatus::Online {
                                    device.status = DeviceStatus::Offline;
                                    println!("设备 {} 离线", device.name);
                                }
                            }
                        }
                    }
                }
                
                thread::sleep(Duration::from_secs(5));
            }
        });
    }
    
    /// 启动数据包处理线程
    fn start_packet_processor(&self) {
        let packet_queue = Arc::clone(&self.packet_queue);
        let routing_rules = Arc::clone(&self.routing_rules);
        let statistics = Arc::clone(&self.statistics);
        let is_running = Arc::clone(&self.is_running);
        
        thread::spawn(move || {
            while *is_running.lock().unwrap() {
                let packets_to_process = {
                    let mut queue = packet_queue.lock().unwrap();
                    let packets = queue.clone();
                    queue.clear();
                    packets
                };
                
                for packet in packets_to_process {
                    Self::process_packet(&packet, &routing_rules, &statistics);
                }
                
                thread::sleep(Duration::from_millis(10));
            }
        });
    }
    
    /// 启动网络监听线程
    fn start_network_listener(&self) {
        let packet_queue = Arc::clone(&self.packet_queue);
        let devices = Arc::clone(&self.devices);
        let statistics = Arc::clone(&self.statistics);
        let is_running = Arc::clone(&self.is_running);
        
        thread::spawn(move || {
            while *is_running.lock().unwrap() {
                // 模拟接收数据包
                let packet = Self::simulate_receive_packet();
                
                {
                    let mut queue = packet_queue.lock().unwrap();
                    queue.push(packet);
                    
                    // 按优先级排序
                    queue.sort_by(|a, b| b.priority.cmp(&a.priority));
                }
                
                {
                    let mut stats = statistics.lock().unwrap();
                    stats.packets_received += 1;
                }
                
                // 更新设备最后见到时间
                {
                    let mut devices = devices.lock().unwrap();
                    if let Some(device) = devices.get_mut(&1) {
                        device.last_seen = SystemTime::now();
                        device.status = DeviceStatus::Online;
                    }
                }
                
                thread::sleep(Duration::from_millis(100));
            }
        });
    }
    
    /// 处理数据包
    fn process_packet(
        packet: &DataPacket,
        routing_rules: &Arc<Mutex<Vec<RoutingRule>>>,
        statistics: &Arc<Mutex<NetworkStatistics>>,
    ) {
        let rules = routing_rules.lock().unwrap();
        
        for rule in rules.iter() {
            if rule.source_protocol == packet.protocol {
                if let Some(filter) = rule.source_device_filter {
                    if filter != packet.source_device {
                        continue;
                    }
                }
                
                // 执行数据转换
                let transformed_data = Self::transform_data(&packet.data, &rule.transformation);
                
                println!(
                    "路由数据包: {} -> {} ({}字节)",
                    packet.source_device,
                    rule.destination_device,
                    transformed_data.len()
                );
                
                {
                    let mut stats = statistics.lock().unwrap();
                    stats.packets_sent += 1;
                    stats.bytes_sent += transformed_data.len() as u64;
                }
                
                break;
            }
        }
    }
    
    /// 数据转换
    fn transform_data(data: &[u8], transformation: &DataTransformation) -> Vec<u8> {
        match transformation {
            DataTransformation::None => data.to_vec(),
            DataTransformation::ModbusToOpcUa => {
                // 模拟Modbus到OPC UA的转换
                let mut result = Vec::new();
                result.extend_from_slice(b"OPC-UA:");
                result.extend_from_slice(data);
                result
            },
            DataTransformation::EtherNetIpToHttp => {
                // 模拟EtherNet/IP到HTTP的转换
                let json_data = format!("{{\"data\": {:?}}}", data);
                json_data.into_bytes()
            },
            DataTransformation::Custom(name) => {
                println!("执行自定义转换: {}", name);
                data.to_vec()
            },
        }
    }
    
    /// 模拟接收数据包
    fn simulate_receive_packet() -> DataPacket {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        SystemTime::now().hash(&mut hasher);
        let random = hasher.finish();
        
        let protocols = [ProtocolType::ModbusTcp, ProtocolType::EtherNetIp, ProtocolType::OpcUa];
        let protocol = protocols[(random % 3) as usize].clone();
        
        DataPacket {
            source_device: 1,
            destination_device: 2,
            protocol,
            data: vec![0x01, 0x02, 0x03, 0x04],
            timestamp: SystemTime::now(),
            priority: PacketPriority::Normal,
        }
    }
    
    /// 获取网络统计信息
    fn get_statistics(&self) -> NetworkStatistics {
        self.statistics.lock().unwrap().clone()
    }
    
    /// 获取设备列表
    fn get_devices(&self) -> Vec<NetworkDevice> {
        self.devices.lock().unwrap().values().cloned().collect()
    }
}

fn main() {
    println!("以太网网关示例");
    
    let gateway = EthernetGateway::new();
    
    // 添加网络设备
    gateway.add_device(NetworkDevice {
        id: 1,
        name: "PLC-001".to_string(),
        ip_address: IpAddr::V4(Ipv4Addr::new(192, 168, 1, 10)),
        port: 502,
        protocol: ProtocolType::ModbusTcp,
        status: DeviceStatus::Online,
        last_seen: SystemTime::now(),
    });
    
    gateway.add_device(NetworkDevice {
        id: 2,
        name: "HMI-001".to_string(),
        ip_address: IpAddr::V4(Ipv4Addr::new(192, 168, 1, 20)),
        port: 4840,
        protocol: ProtocolType::OpcUa,
        status: DeviceStatus::Online,
        last_seen: SystemTime::now(),
    });
    
    // 添加路由规则
    gateway.add_routing_rule(RoutingRule {
        id: 1,
        source_protocol: ProtocolType::ModbusTcp,
        destination_protocol: ProtocolType::OpcUa,
        source_device_filter: Some(1),
        destination_device: 2,
        transformation: DataTransformation::ModbusToOpcUa,
    });
    
    // 启动网关
    gateway.start();
    
    // 运行一段时间
    println!("\n网关运行中，按Ctrl+C停止...");
    thread::sleep(Duration::from_secs(10));
    
    // 显示统计信息
    let stats = gateway.get_statistics();
    println!("\n=== 网络统计信息 ===");
    println!("接收数据包: {}", stats.packets_received);
    println!("发送数据包: {}", stats.packets_sent);
    println!("丢弃数据包: {}", stats.packets_dropped);
    println!("接收字节数: {}", stats.bytes_received);
    println!("发送字节数: {}", stats.bytes_sent);
    
    // 显示设备状态
    let devices = gateway.get_devices();
    println!("\n=== 设备状态 ===");
    for device in devices {
        println!("{}: {} ({}:{})", device.name, 
                device.status as u8, device.ip_address, device.port);
    }
    
    // 停止网关
    gateway.stop();
    
    println!("\n以太网网关示例完成");
}