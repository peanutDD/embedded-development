# RTOS网络协议栈集成

## 概述

网络协议栈是现代嵌入式系统的重要组成部分，特别是在物联网(IoT)应用中。RTOS需要提供高效、可靠的网络支持，包括TCP/IP协议栈、网络驱动程序接口和网络应用框架。

## 网络架构设计

### 分层架构

```rust
/// 网络层次结构
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NetworkLayer {
    Physical,       // 物理层
    DataLink,       // 数据链路层
    Network,        // 网络层
    Transport,      // 传输层
    Session,        // 会话层
    Presentation,   // 表示层
    Application,    // 应用层
}

/// 网络协议类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProtocolType {
    Ethernet,
    WiFi,
    Bluetooth,
    LoRa,
    Zigbee,
    CAN,
    IPv4,
    IPv6,
    TCP,
    UDP,
    HTTP,
    MQTT,
    CoAP,
}

/// 网络接口描述符
#[derive(Debug, Clone)]
pub struct NetworkInterface {
    pub interface_id: u8,
    pub name: String<16>,
    pub mac_address: [u8; 6],
    pub ip_address: IpAddress,
    pub netmask: IpAddress,
    pub gateway: IpAddress,
    pub mtu: u16,
    pub status: InterfaceStatus,
    pub statistics: InterfaceStats,
}

/// 接口状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterfaceStatus {
    Down,
    Up,
    Connecting,
    Connected,
    Disconnecting,
    Error,
}

/// 接口统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct InterfaceStats {
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub packets_sent: u32,
    pub packets_received: u32,
    pub errors_sent: u32,
    pub errors_received: u32,
    pub dropped_packets: u32,
}

/// IP地址
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IpAddress {
    V4([u8; 4]),
    V6([u8; 16]),
}

impl IpAddress {
    pub fn new_v4(a: u8, b: u8, c: u8, d: u8) -> Self {
        Self::V4([a, b, c, d])
    }
    
    pub fn new_v6(addr: [u8; 16]) -> Self {
        Self::V6(addr)
    }
    
    pub fn is_loopback(&self) -> bool {
        match self {
            IpAddress::V4([127, 0, 0, 1]) => true,
            IpAddress::V6(addr) => {
                *addr == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
            }
        }
    }
    
    pub fn is_multicast(&self) -> bool {
        match self {
            IpAddress::V4([a, _, _, _]) => *a >= 224 && *a <= 239,
            IpAddress::V6(addr) => addr[0] == 0xFF,
        }
    }
}
```

### 网络栈管理器

```rust
/// 网络栈管理器
pub struct NetworkStackManager {
    interfaces: Vec<NetworkInterface, 8>,
    routing_table: Vec<RouteEntry, 32>,
    arp_table: Vec<ArpEntry, 64>,
    socket_manager: SocketManager,
    packet_pool: PacketPool,
    config: NetworkConfig,
    stats: NetworkStats,
}

/// 路由表项
#[derive(Debug, Clone)]
pub struct RouteEntry {
    pub destination: IpAddress,
    pub netmask: IpAddress,
    pub gateway: IpAddress,
    pub interface_id: u8,
    pub metric: u16,
    pub flags: RouteFlags,
}

/// 路由标志
#[derive(Debug, Clone, Copy, Default)]
pub struct RouteFlags {
    pub up: bool,
    pub gateway: bool,
    pub host: bool,
    pub dynamic: bool,
}

/// ARP表项
#[derive(Debug, Clone)]
pub struct ArpEntry {
    pub ip_address: IpAddress,
    pub mac_address: [u8; 6],
    pub interface_id: u8,
    pub timestamp: u64,
    pub state: ArpState,
}

/// ARP状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArpState {
    Incomplete,
    Reachable,
    Stale,
    Delay,
    Probe,
}

/// 网络配置
#[derive(Debug, Clone)]
pub struct NetworkConfig {
    pub dhcp_enabled: bool,
    pub dns_servers: Vec<IpAddress, 4>,
    pub hostname: String<32>,
    pub domain: String<64>,
    pub tcp_keepalive: bool,
    pub tcp_nodelay: bool,
    pub buffer_sizes: BufferSizes,
}

/// 缓冲区大小配置
#[derive(Debug, Clone, Copy)]
pub struct BufferSizes {
    pub rx_buffer_size: usize,
    pub tx_buffer_size: usize,
    pub tcp_window_size: usize,
    pub udp_buffer_size: usize,
}

/// 网络统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct NetworkStats {
    pub total_interfaces: u8,
    pub active_interfaces: u8,
    pub total_packets: u64,
    pub dropped_packets: u64,
    pub routing_errors: u32,
    pub arp_requests: u32,
    pub arp_replies: u32,
}

impl NetworkStackManager {
    /// 创建网络栈管理器
    pub fn new(config: NetworkConfig) -> Self {
        Self {
            interfaces: Vec::new(),
            routing_table: Vec::new(),
            arp_table: Vec::new(),
            socket_manager: SocketManager::new(),
            packet_pool: PacketPool::new(1024, 64), // 64个1KB包
            config,
            stats: NetworkStats::default(),
        }
    }
    
    /// 添加网络接口
    pub fn add_interface(
        &mut self,
        interface: NetworkInterface,
    ) -> Result<(), NetworkError> {
        // 检查接口ID是否已存在
        if self.interfaces.iter().any(|iface| iface.interface_id == interface.interface_id) {
            return Err(NetworkError::InterfaceExists);
        }
        
        self.interfaces.push(interface)
            .map_err(|_| NetworkError::OutOfMemory)?;
        
        self.stats.total_interfaces += 1;
        Ok(())
    }
    
    /// 启用接口
    pub fn enable_interface(&mut self, interface_id: u8) -> Result<(), NetworkError> {
        if let Some(interface) = self.interfaces.iter_mut()
            .find(|iface| iface.interface_id == interface_id) {
            
            interface.status = InterfaceStatus::Up;
            self.stats.active_interfaces += 1;
            
            // 添加默认路由
            self.add_default_route(interface_id, interface.gateway)?;
            
            Ok(())
        } else {
            Err(NetworkError::InterfaceNotFound)
        }
    }
    
    /// 禁用接口
    pub fn disable_interface(&mut self, interface_id: u8) -> Result<(), NetworkError> {
        if let Some(interface) = self.interfaces.iter_mut()
            .find(|iface| iface.interface_id == interface_id) {
            
            interface.status = InterfaceStatus::Down;
            self.stats.active_interfaces = self.stats.active_interfaces.saturating_sub(1);
            
            // 移除相关路由
            self.routing_table.retain(|route| route.interface_id != interface_id);
            
            Ok(())
        } else {
            Err(NetworkError::InterfaceNotFound)
        }
    }
    
    /// 添加路由
    pub fn add_route(&mut self, route: RouteEntry) -> Result<(), NetworkError> {
        // 检查接口是否存在
        if !self.interfaces.iter().any(|iface| iface.interface_id == route.interface_id) {
            return Err(NetworkError::InterfaceNotFound);
        }
        
        self.routing_table.push(route)
            .map_err(|_| NetworkError::OutOfMemory)?;
        
        Ok(())
    }
    
    /// 添加默认路由
    fn add_default_route(
        &mut self,
        interface_id: u8,
        gateway: IpAddress,
    ) -> Result<(), NetworkError> {
        let default_route = RouteEntry {
            destination: IpAddress::V4([0, 0, 0, 0]),
            netmask: IpAddress::V4([0, 0, 0, 0]),
            gateway,
            interface_id,
            metric: 1,
            flags: RouteFlags {
                up: true,
                gateway: true,
                host: false,
                dynamic: false,
            },
        };
        
        self.add_route(default_route)
    }
    
    /// 查找路由
    pub fn find_route(&self, destination: IpAddress) -> Option<&RouteEntry> {
        // 查找最匹配的路由（最长前缀匹配）
        let mut best_match: Option<&RouteEntry> = None;
        let mut best_prefix_len = 0;
        
        for route in &self.routing_table {
            if route.flags.up && self.address_matches(destination, route.destination, route.netmask) {
                let prefix_len = self.calculate_prefix_length(route.netmask);
                if prefix_len > best_prefix_len {
                    best_match = Some(route);
                    best_prefix_len = prefix_len;
                }
            }
        }
        
        best_match
    }
    
    /// 检查地址是否匹配
    fn address_matches(
        &self,
        addr: IpAddress,
        network: IpAddress,
        netmask: IpAddress,
    ) -> bool {
        match (addr, network, netmask) {
            (IpAddress::V4(a), IpAddress::V4(n), IpAddress::V4(m)) => {
                for i in 0..4 {
                    if (a[i] & m[i]) != (n[i] & m[i]) {
                        return false;
                    }
                }
                true
            }
            (IpAddress::V6(a), IpAddress::V6(n), IpAddress::V6(m)) => {
                for i in 0..16 {
                    if (a[i] & m[i]) != (n[i] & m[i]) {
                        return false;
                    }
                }
                true
            }
            _ => false,
        }
    }
    
    /// 计算前缀长度
    fn calculate_prefix_length(&self, netmask: IpAddress) -> u8 {
        match netmask {
            IpAddress::V4(mask) => {
                let mut count = 0;
                for byte in mask {
                    count += byte.count_ones() as u8;
                }
                count
            }
            IpAddress::V6(mask) => {
                let mut count = 0;
                for byte in mask {
                    count += byte.count_ones() as u8;
                }
                count
            }
        }
    }
    
    /// 获取网络统计信息
    pub fn get_stats(&self) -> NetworkStats {
        self.stats
    }
}
```

## 数据包管理

### 数据包结构

```rust
/// 网络数据包
#[derive(Debug)]
pub struct NetworkPacket {
    pub buffer: Vec<u8, 1536>, // 最大以太网帧大小
    pub length: usize,
    pub interface_id: u8,
    pub protocol: ProtocolType,
    pub timestamp: u64,
    pub flags: PacketFlags,
}

/// 数据包标志
#[derive(Debug, Clone, Copy, Default)]
pub struct PacketFlags {
    pub broadcast: bool,
    pub multicast: bool,
    pub fragmented: bool,
    pub checksum_valid: bool,
    pub priority: PacketPriority,
}

/// 数据包优先级
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PacketPriority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
}

impl NetworkPacket {
    /// 创建新数据包
    pub fn new(interface_id: u8, protocol: ProtocolType) -> Self {
        Self {
            buffer: Vec::new(),
            length: 0,
            interface_id,
            protocol,
            timestamp: get_current_time() as u64,
            flags: PacketFlags::default(),
        }
    }
    
    /// 添加数据
    pub fn push_data(&mut self, data: &[u8]) -> Result<(), NetworkError> {
        for &byte in data {
            self.buffer.push(byte).map_err(|_| NetworkError::BufferFull)?;
        }
        self.length += data.len();
        Ok(())
    }
    
    /// 获取数据
    pub fn get_data(&self) -> &[u8] {
        &self.buffer[..self.length]
    }
    
    /// 获取可变数据
    pub fn get_data_mut(&mut self) -> &mut [u8] {
        &mut self.buffer[..self.length]
    }
    
    /// 解析以太网头部
    pub fn parse_ethernet_header(&self) -> Result<EthernetHeader, NetworkError> {
        if self.length < 14 {
            return Err(NetworkError::InvalidPacket);
        }
        
        let data = self.get_data();
        Ok(EthernetHeader {
            dst_mac: [
                data[0], data[1], data[2],
                data[3], data[4], data[5]
            ],
            src_mac: [
                data[6], data[7], data[8],
                data[9], data[10], data[11]
            ],
            ethertype: u16::from_be_bytes([data[12], data[13]]),
        })
    }
    
    /// 解析IP头部
    pub fn parse_ip_header(&self, offset: usize) -> Result<IpHeader, NetworkError> {
        if self.length < offset + 20 {
            return Err(NetworkError::InvalidPacket);
        }
        
        let data = &self.get_data()[offset..];
        let version = (data[0] >> 4) & 0x0F;
        
        match version {
            4 => self.parse_ipv4_header(offset),
            6 => self.parse_ipv6_header(offset),
            _ => Err(NetworkError::UnsupportedProtocol),
        }
    }
    
    /// 解析IPv4头部
    fn parse_ipv4_header(&self, offset: usize) -> Result<IpHeader, NetworkError> {
        let data = &self.get_data()[offset..];
        
        Ok(IpHeader::V4(Ipv4Header {
            version: (data[0] >> 4) & 0x0F,
            ihl: data[0] & 0x0F,
            tos: data[1],
            total_length: u16::from_be_bytes([data[2], data[3]]),
            identification: u16::from_be_bytes([data[4], data[5]]),
            flags: (data[6] >> 5) & 0x07,
            fragment_offset: u16::from_be_bytes([data[6], data[7]]) & 0x1FFF,
            ttl: data[8],
            protocol: data[9],
            checksum: u16::from_be_bytes([data[10], data[11]]),
            src_addr: [data[12], data[13], data[14], data[15]],
            dst_addr: [data[16], data[17], data[18], data[19]],
        }))
    }
    
    /// 解析IPv6头部
    fn parse_ipv6_header(&self, offset: usize) -> Result<IpHeader, NetworkError> {
        let data = &self.get_data()[offset..];
        
        if data.len() < 40 {
            return Err(NetworkError::InvalidPacket);
        }
        
        let mut src_addr = [0u8; 16];
        let mut dst_addr = [0u8; 16];
        
        src_addr.copy_from_slice(&data[8..24]);
        dst_addr.copy_from_slice(&data[24..40]);
        
        Ok(IpHeader::V6(Ipv6Header {
            version: (data[0] >> 4) & 0x0F,
            traffic_class: ((data[0] & 0x0F) << 4) | ((data[1] >> 4) & 0x0F),
            flow_label: u32::from_be_bytes([0, data[1] & 0x0F, data[2], data[3]]),
            payload_length: u16::from_be_bytes([data[4], data[5]]),
            next_header: data[6],
            hop_limit: data[7],
            src_addr,
            dst_addr,
        }))
    }
}

/// 以太网头部
#[derive(Debug, Clone, Copy)]
pub struct EthernetHeader {
    pub dst_mac: [u8; 6],
    pub src_mac: [u8; 6],
    pub ethertype: u16,
}

/// IP头部
#[derive(Debug, Clone)]
pub enum IpHeader {
    V4(Ipv4Header),
    V6(Ipv6Header),
}

/// IPv4头部
#[derive(Debug, Clone, Copy)]
pub struct Ipv4Header {
    pub version: u8,
    pub ihl: u8,
    pub tos: u8,
    pub total_length: u16,
    pub identification: u16,
    pub flags: u8,
    pub fragment_offset: u16,
    pub ttl: u8,
    pub protocol: u8,
    pub checksum: u16,
    pub src_addr: [u8; 4],
    pub dst_addr: [u8; 4],
}

/// IPv6头部
#[derive(Debug, Clone, Copy)]
pub struct Ipv6Header {
    pub version: u8,
    pub traffic_class: u8,
    pub flow_label: u32,
    pub payload_length: u16,
    pub next_header: u8,
    pub hop_limit: u8,
    pub src_addr: [u8; 16],
    pub dst_addr: [u8; 16],
}
```

### 数据包池管理

```rust
/// 数据包池
pub struct PacketPool {
    packets: Vec<Option<NetworkPacket>, 64>,
    free_list: Vec<usize, 64>,
    packet_size: usize,
    stats: PacketPoolStats,
}

/// 数据包池统计
#[derive(Debug, Clone, Copy, Default)]
pub struct PacketPoolStats {
    pub total_packets: usize,
    pub free_packets: usize,
    pub allocated_packets: usize,
    pub allocation_failures: u32,
    pub peak_usage: usize,
}

impl PacketPool {
    /// 创建数据包池
    pub fn new(packet_size: usize, pool_size: usize) -> Self {
        let mut packets = Vec::new();
        let mut free_list = Vec::new();
        
        // 初始化数据包池
        for i in 0..pool_size {
            let _ = packets.push(None);
            let _ = free_list.push(i);
        }
        
        Self {
            packets,
            free_list,
            packet_size,
            stats: PacketPoolStats {
                total_packets: pool_size,
                free_packets: pool_size,
                allocated_packets: 0,
                allocation_failures: 0,
                peak_usage: 0,
            },
        }
    }
    
    /// 分配数据包
    pub fn allocate(
        &mut self,
        interface_id: u8,
        protocol: ProtocolType,
    ) -> Result<PacketHandle, NetworkError> {
        if let Some(index) = self.free_list.pop() {
            let packet = NetworkPacket::new(interface_id, protocol);
            self.packets[index] = Some(packet);
            
            self.stats.free_packets -= 1;
            self.stats.allocated_packets += 1;
            
            if self.stats.allocated_packets > self.stats.peak_usage {
                self.stats.peak_usage = self.stats.allocated_packets;
            }
            
            Ok(PacketHandle { index })
        } else {
            self.stats.allocation_failures += 1;
            Err(NetworkError::OutOfMemory)
        }
    }
    
    /// 释放数据包
    pub fn deallocate(&mut self, handle: PacketHandle) -> Result<(), NetworkError> {
        if handle.index >= self.packets.len() {
            return Err(NetworkError::InvalidHandle);
        }
        
        if self.packets[handle.index].is_some() {
            self.packets[handle.index] = None;
            self.free_list.push(handle.index)
                .map_err(|_| NetworkError::OutOfMemory)?;
            
            self.stats.free_packets += 1;
            self.stats.allocated_packets -= 1;
            
            Ok(())
        } else {
            Err(NetworkError::InvalidHandle)
        }
    }
    
    /// 获取数据包
    pub fn get_packet(&self, handle: &PacketHandle) -> Option<&NetworkPacket> {
        self.packets.get(handle.index)?.as_ref()
    }
    
    /// 获取可变数据包
    pub fn get_packet_mut(&mut self, handle: &PacketHandle) -> Option<&mut NetworkPacket> {
        self.packets.get_mut(handle.index)?.as_mut()
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> PacketPoolStats {
        self.stats
    }
}

/// 数据包句柄
#[derive(Debug, Clone, Copy)]
pub struct PacketHandle {
    index: usize,
}
```

## Socket管理

### Socket抽象

```rust
/// Socket类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SocketType {
    Stream,     // TCP
    Datagram,   // UDP
    Raw,        // 原始套接字
}

/// Socket状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SocketState {
    Closed,
    Listen,
    SynSent,
    SynReceived,
    Established,
    FinWait1,
    FinWait2,
    CloseWait,
    Closing,
    LastAck,
    TimeWait,
}

/// Socket地址
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SocketAddr {
    pub ip: IpAddress,
    pub port: u16,
}

/// Socket描述符
#[derive(Debug, Clone)]
pub struct Socket {
    pub socket_id: u32,
    pub socket_type: SocketType,
    pub state: SocketState,
    pub local_addr: Option<SocketAddr>,
    pub remote_addr: Option<SocketAddr>,
    pub rx_buffer: RingBuffer,
    pub tx_buffer: RingBuffer,
    pub options: SocketOptions,
    pub stats: SocketStats,
}

/// Socket选项
#[derive(Debug, Clone, Copy)]
pub struct SocketOptions {
    pub reuse_addr: bool,
    pub keep_alive: bool,
    pub no_delay: bool,
    pub broadcast: bool,
    pub receive_timeout: Option<u32>,
    pub send_timeout: Option<u32>,
    pub buffer_size: usize,
}

/// Socket统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct SocketStats {
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub packets_sent: u32,
    pub packets_received: u32,
    pub connection_attempts: u32,
    pub connection_failures: u32,
    pub retransmissions: u32,
}

/// 环形缓冲区
#[derive(Debug, Clone)]
pub struct RingBuffer {
    buffer: Vec<u8, 2048>,
    head: usize,
    tail: usize,
    size: usize,
}

impl RingBuffer {
    pub fn new(capacity: usize) -> Self {
        Self {
            buffer: Vec::new(),
            head: 0,
            tail: 0,
            size: 0,
        }
    }
    
    /// 写入数据
    pub fn write(&mut self, data: &[u8]) -> usize {
        let mut written = 0;
        
        for &byte in data {
            if self.size < self.buffer.capacity() {
                if self.buffer.len() <= self.tail {
                    let _ = self.buffer.push(byte);
                } else {
                    self.buffer[self.tail] = byte;
                }
                
                self.tail = (self.tail + 1) % self.buffer.capacity();
                self.size += 1;
                written += 1;
            } else {
                break;
            }
        }
        
        written
    }
    
    /// 读取数据
    pub fn read(&mut self, buffer: &mut [u8]) -> usize {
        let mut read = 0;
        
        for i in 0..buffer.len() {
            if self.size > 0 {
                buffer[i] = self.buffer[self.head];
                self.head = (self.head + 1) % self.buffer.capacity();
                self.size -= 1;
                read += 1;
            } else {
                break;
            }
        }
        
        read
    }
    
    /// 获取可用空间
    pub fn available_space(&self) -> usize {
        self.buffer.capacity() - self.size
    }
    
    /// 获取可读数据量
    pub fn available_data(&self) -> usize {
        self.size
    }
    
    /// 清空缓冲区
    pub fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.size = 0;
    }
}

impl Socket {
    /// 创建新Socket
    pub fn new(socket_id: u32, socket_type: SocketType) -> Self {
        Self {
            socket_id,
            socket_type,
            state: SocketState::Closed,
            local_addr: None,
            remote_addr: None,
            rx_buffer: RingBuffer::new(1024),
            tx_buffer: RingBuffer::new(1024),
            options: SocketOptions {
                reuse_addr: false,
                keep_alive: false,
                no_delay: false,
                broadcast: false,
                receive_timeout: None,
                send_timeout: None,
                buffer_size: 1024,
            },
            stats: SocketStats::default(),
        }
    }
    
    /// 绑定地址
    pub fn bind(&mut self, addr: SocketAddr) -> Result<(), NetworkError> {
        if self.state != SocketState::Closed {
            return Err(NetworkError::InvalidState);
        }
        
        self.local_addr = Some(addr);
        Ok(())
    }
    
    /// 连接到远程地址
    pub fn connect(&mut self, addr: SocketAddr) -> Result<(), NetworkError> {
        match self.socket_type {
            SocketType::Stream => {
                if self.state != SocketState::Closed {
                    return Err(NetworkError::InvalidState);
                }
                
                self.remote_addr = Some(addr);
                self.state = SocketState::SynSent;
                self.stats.connection_attempts += 1;
                
                // 发送SYN包的逻辑应该在这里实现
                Ok(())
            }
            SocketType::Datagram => {
                self.remote_addr = Some(addr);
                Ok(())
            }
            SocketType::Raw => {
                self.remote_addr = Some(addr);
                Ok(())
            }
        }
    }
    
    /// 监听连接
    pub fn listen(&mut self, backlog: u32) -> Result<(), NetworkError> {
        if self.socket_type != SocketType::Stream {
            return Err(NetworkError::UnsupportedOperation);
        }
        
        if self.local_addr.is_none() {
            return Err(NetworkError::NotBound);
        }
        
        self.state = SocketState::Listen;
        Ok(())
    }
    
    /// 发送数据
    pub fn send(&mut self, data: &[u8]) -> Result<usize, NetworkError> {
        match self.socket_type {
            SocketType::Stream => {
                if self.state != SocketState::Established {
                    return Err(NetworkError::NotConnected);
                }
            }
            SocketType::Datagram => {
                if self.remote_addr.is_none() {
                    return Err(NetworkError::NotConnected);
                }
            }
            SocketType::Raw => {}
        }
        
        let sent = self.tx_buffer.write(data);
        self.stats.bytes_sent += sent as u64;
        
        if sent > 0 {
            self.stats.packets_sent += 1;
        }
        
        Ok(sent)
    }
    
    /// 接收数据
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, NetworkError> {
        let received = self.rx_buffer.read(buffer);
        self.stats.bytes_received += received as u64;
        
        if received > 0 {
            self.stats.packets_received += 1;
        }
        
        Ok(received)
    }
    
    /// 关闭Socket
    pub fn close(&mut self) -> Result<(), NetworkError> {
        match self.socket_type {
            SocketType::Stream => {
                match self.state {
                    SocketState::Established => {
                        self.state = SocketState::FinWait1;
                        // 发送FIN包
                    }
                    SocketState::CloseWait => {
                        self.state = SocketState::LastAck;
                        // 发送FIN包
                    }
                    _ => {
                        self.state = SocketState::Closed;
                    }
                }
            }
            _ => {
                self.state = SocketState::Closed;
            }
        }
        
        Ok(())
    }
}
```

### Socket管理器

```rust
/// Socket管理器
pub struct SocketManager {
    sockets: Vec<Option<Socket>, 64>,
    next_socket_id: u32,
    port_allocator: PortAllocator,
    stats: SocketManagerStats,
}

/// 端口分配器
#[derive(Debug)]
struct PortAllocator {
    dynamic_ports: Vec<bool, 16384>, // 49152-65535
    next_port: u16,
}

/// Socket管理器统计
#[derive(Debug, Clone, Copy, Default)]
pub struct SocketManagerStats {
    pub total_sockets: u32,
    pub active_sockets: u32,
    pub tcp_sockets: u32,
    pub udp_sockets: u32,
    pub raw_sockets: u32,
    pub port_allocation_failures: u32,
}

impl PortAllocator {
    fn new() -> Self {
        Self {
            dynamic_ports: Vec::new(),
            next_port: 49152,
        }
    }
    
    /// 分配动态端口
    fn allocate_port(&mut self) -> Option<u16> {
        let start_port = self.next_port;
        
        loop {
            let index = (self.next_port - 49152) as usize;
            
            if index < self.dynamic_ports.len() {
                if !self.dynamic_ports[index] {
                    self.dynamic_ports[index] = true;
                    let port = self.next_port;
                    self.next_port = if self.next_port == 65535 { 49152 } else { self.next_port + 1 };
                    return Some(port);
                }
            } else {
                // 扩展端口数组
                while self.dynamic_ports.len() <= index {
                    if self.dynamic_ports.push(false).is_err() {
                        return None;
                    }
                }
                continue;
            }
            
            self.next_port = if self.next_port == 65535 { 49152 } else { self.next_port + 1 };
            
            if self.next_port == start_port {
                // 所有端口都已分配
                return None;
            }
        }
    }
    
    /// 释放端口
    fn deallocate_port(&mut self, port: u16) {
        if port >= 49152 {
            let index = (port - 49152) as usize;
            if index < self.dynamic_ports.len() {
                self.dynamic_ports[index] = false;
            }
        }
    }
    
    /// 检查端口是否可用
    fn is_port_available(&self, port: u16) -> bool {
        if port < 49152 {
            // 系统端口，需要特权
            false
        } else {
            let index = (port - 49152) as usize;
            index >= self.dynamic_ports.len() || !self.dynamic_ports[index]
        }
    }
}

impl SocketManager {
    /// 创建Socket管理器
    pub fn new() -> Self {
        Self {
            sockets: Vec::new(),
            next_socket_id: 1,
            port_allocator: PortAllocator::new(),
            stats: SocketManagerStats::default(),
        }
    }
    
    /// 创建Socket
    pub fn create_socket(&mut self, socket_type: SocketType) -> Result<u32, NetworkError> {
        // 查找空闲槽位
        let socket_id = self.next_socket_id;
        self.next_socket_id += 1;
        
        let socket = Socket::new(socket_id, socket_type);
        
        // 查找空闲位置
        if let Some(empty_slot) = self.sockets.iter_mut().find(|slot| slot.is_none()) {
            *empty_slot = Some(socket);
        } else {
            // 添加新槽位
            self.sockets.push(Some(socket))
                .map_err(|_| NetworkError::OutOfMemory)?;
        }
        
        // 更新统计信息
        self.stats.total_sockets += 1;
        self.stats.active_sockets += 1;
        
        match socket_type {
            SocketType::Stream => self.stats.tcp_sockets += 1,
            SocketType::Datagram => self.stats.udp_sockets += 1,
            SocketType::Raw => self.stats.raw_sockets += 1,
        }
        
        Ok(socket_id)
    }
    
    /// 关闭Socket
    pub fn close_socket(&mut self, socket_id: u32) -> Result<(), NetworkError> {
        if let Some(socket_slot) = self.sockets.iter_mut()
            .find(|slot| slot.as_ref().map_or(false, |s| s.socket_id == socket_id)) {
            
            if let Some(socket) = socket_slot.take() {
                // 释放端口
                if let Some(local_addr) = socket.local_addr {
                    self.port_allocator.deallocate_port(local_addr.port);
                }
                
                // 更新统计信息
                self.stats.active_sockets -= 1;
                
                match socket.socket_type {
                    SocketType::Stream => self.stats.tcp_sockets -= 1,
                    SocketType::Datagram => self.stats.udp_sockets -= 1,
                    SocketType::Raw => self.stats.raw_sockets -= 1,
                }
                
                Ok(())
            } else {
                Err(NetworkError::InvalidSocket)
            }
        } else {
            Err(NetworkError::InvalidSocket)
        }
    }
    
    /// 获取Socket
    pub fn get_socket(&self, socket_id: u32) -> Option<&Socket> {
        self.sockets.iter()
            .find_map(|slot| slot.as_ref())
            .filter(|socket| socket.socket_id == socket_id)
    }
    
    /// 获取可变Socket
    pub fn get_socket_mut(&mut self, socket_id: u32) -> Option<&mut Socket> {
        self.sockets.iter_mut()
            .find_map(|slot| slot.as_mut())
            .filter(|socket| socket.socket_id == socket_id)
    }
    
    /// 绑定Socket到地址
    pub fn bind_socket(
        &mut self,
        socket_id: u32,
        mut addr: SocketAddr,
    ) -> Result<(), NetworkError> {
        // 如果端口为0，自动分配
        if addr.port == 0 {
            if let Some(port) = self.port_allocator.allocate_port() {
                addr.port = port;
            } else {
                self.stats.port_allocation_failures += 1;
                return Err(NetworkError::PortUnavailable);
            }
        } else {
            // 检查端口是否可用
            if !self.port_allocator.is_port_available(addr.port) {
                return Err(NetworkError::PortUnavailable);
            }
        }
        
        if let Some(socket) = self.get_socket_mut(socket_id) {
            socket.bind(addr)
        } else {
            Err(NetworkError::InvalidSocket)
        }
    }
    
    /// 查找监听Socket
    pub fn find_listening_socket(&self, addr: SocketAddr) -> Option<&Socket> {
        self.sockets.iter()
            .filter_map(|slot| slot.as_ref())
            .find(|socket| {
                socket.state == SocketState::Listen &&
                socket.local_addr.map_or(false, |local| {
                    local.port == addr.port &&
                    (local.ip == addr.ip || matches!(local.ip, IpAddress::V4([0, 0, 0, 0])))
                })
            })
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> SocketManagerStats {
        self.stats
    }
}
```

## 网络错误处理

```rust
/// 网络错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NetworkError {
    // 通用错误
    OutOfMemory,
    InvalidParameter,
    Timeout,
    
    // 接口错误
    InterfaceNotFound,
    InterfaceExists,
    InterfaceDown,
    
    // 路由错误
    NoRoute,
    RouteExists,
    
    // 数据包错误
    InvalidPacket,
    PacketTooLarge,
    BufferFull,
    InvalidHandle,
    
    // Socket错误
    InvalidSocket,
    InvalidState,
    NotBound,
    NotConnected,
    PortUnavailable,
    UnsupportedOperation,
    
    // 协议错误
    UnsupportedProtocol,
    ProtocolError,
    ChecksumError,
    
    // 连接错误
    ConnectionRefused,
    ConnectionReset,
    ConnectionAborted,
    ConnectionTimeout,
}

impl NetworkError {
    /// 获取错误描述
    pub fn description(&self) -> &'static str {
        match self {
            NetworkError::OutOfMemory => "Out of memory",
            NetworkError::InvalidParameter => "Invalid parameter",
            NetworkError::Timeout => "Operation timed out",
            NetworkError::InterfaceNotFound => "Network interface not found",
            NetworkError::InterfaceExists => "Network interface already exists",
            NetworkError::InterfaceDown => "Network interface is down",
            NetworkError::NoRoute => "No route to destination",
            NetworkError::RouteExists => "Route already exists",
            NetworkError::InvalidPacket => "Invalid packet format",
            NetworkError::PacketTooLarge => "Packet too large",
            NetworkError::BufferFull => "Buffer is full",
            NetworkError::InvalidHandle => "Invalid packet handle",
            NetworkError::InvalidSocket => "Invalid socket",
            NetworkError::InvalidState => "Invalid socket state",
            NetworkError::NotBound => "Socket not bound",
            NetworkError::NotConnected => "Socket not connected",
            NetworkError::PortUnavailable => "Port unavailable",
            NetworkError::UnsupportedOperation => "Unsupported operation",
            NetworkError::UnsupportedProtocol => "Unsupported protocol",
            NetworkError::ProtocolError => "Protocol error",
            NetworkError::ChecksumError => "Checksum error",
            NetworkError::ConnectionRefused => "Connection refused",
            NetworkError::ConnectionReset => "Connection reset",
            NetworkError::ConnectionAborted => "Connection aborted",
            NetworkError::ConnectionTimeout => "Connection timeout",
        }
    }
    
    /// 检查是否为临时错误
    pub fn is_temporary(&self) -> bool {
        matches!(self, 
            NetworkError::Timeout |
            NetworkError::BufferFull |
            NetworkError::OutOfMemory |
            NetworkError::InterfaceDown
        )
    }
    
    /// 检查是否为致命错误
    pub fn is_fatal(&self) -> bool {
        matches!(self,
            NetworkError::InvalidParameter |
            NetworkError::UnsupportedProtocol |
            NetworkError::UnsupportedOperation
        )
    }
}
```

## 网络应用框架

### HTTP客户端

```rust
/// HTTP方法
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HttpMethod {
    Get,
    Post,
    Put,
    Delete,
    Head,
    Options,
    Patch,
}

/// HTTP状态码
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HttpStatus {
    Ok = 200,
    Created = 201,
    NoContent = 204,
    BadRequest = 400,
    Unauthorized = 401,
    Forbidden = 403,
    NotFound = 404,
    InternalServerError = 500,
}

/// HTTP请求
#[derive(Debug, Clone)]
pub struct HttpRequest {
    pub method: HttpMethod,
    pub url: String<256>,
    pub headers: Vec<(String<64>, String<256>), 16>,
    pub body: Vec<u8, 1024>,
}

/// HTTP响应
#[derive(Debug, Clone)]
pub struct HttpResponse {
    pub status: u16,
    pub headers: Vec<(String<64>, String<256>), 16>,
    pub body: Vec<u8, 2048>,
}

/// HTTP客户端
pub struct HttpClient {
    socket_manager: SocketManager,
    timeout: u32,
    user_agent: String<64>,
    stats: HttpClientStats,
}

/// HTTP客户端统计
#[derive(Debug, Clone, Copy, Default)]
pub struct HttpClientStats {
    pub requests_sent: u32,
    pub responses_received: u32,
    pub timeouts: u32,
    pub errors: u32,
    pub bytes_sent: u64,
    pub bytes_received: u64,
}

impl HttpClient {
    /// 创建HTTP客户端
    pub fn new(timeout: u32) -> Self {
        Self {
            socket_manager: SocketManager::new(),
            timeout,
            user_agent: String::from("EmbeddedHTTP/1.0"),
            stats: HttpClientStats::default(),
        }
    }
    
    /// 发送GET请求
    pub fn get(&mut self, url: &str) -> Result<HttpResponse, NetworkError> {
        let request = HttpRequest {
            method: HttpMethod::Get,
            url: String::from(url),
            headers: Vec::new(),
            body: Vec::new(),
        };
        
        self.send_request(request)
    }
    
    /// 发送POST请求
    pub fn post(&mut self, url: &str, body: &[u8]) -> Result<HttpResponse, NetworkError> {
        let mut request = HttpRequest {
            method: HttpMethod::Post,
            url: String::from(url),
            headers: Vec::new(),
            body: Vec::new(),
        };
        
        // 添加Content-Length头
        let content_length = format!("Content-Length: {}", body.len());
        let _ = request.headers.push((
            String::from("Content-Length"),
            String::from(content_length.as_str())
        ));
        
        // 复制body
        for &byte in body {
            if request.body.push(byte).is_err() {
                return Err(NetworkError::PacketTooLarge);
            }
        }
        
        self.send_request(request)
    }
    
    /// 发送请求
    fn send_request(&mut self, request: HttpRequest) -> Result<HttpResponse, NetworkError> {
        // 解析URL
        let (host, port, path) = self.parse_url(&request.url)?;
        
        // 创建TCP连接
        let socket_id = self.socket_manager.create_socket(SocketType::Stream)?;
        
        // 连接到服务器
        let server_addr = SocketAddr {
            ip: self.resolve_hostname(&host)?,
            port,
        };
        
        if let Some(socket) = self.socket_manager.get_socket_mut(socket_id) {
            socket.connect(server_addr)?;
            
            // 构建HTTP请求
            let http_request = self.build_http_request(&request, &host, &path)?;
            
            // 发送请求
            socket.send(http_request.as_bytes())?;
            self.stats.requests_sent += 1;
            self.stats.bytes_sent += http_request.len() as u64;
            
            // 接收响应
            let response = self.receive_response(socket_id)?;
            
            // 关闭连接
            self.socket_manager.close_socket(socket_id)?;
            
            self.stats.responses_received += 1;
            self.stats.bytes_received += response.body.len() as u64;
            
            Ok(response)
        } else {
            Err(NetworkError::InvalidSocket)
        }
    }
    
    /// 解析URL
    fn parse_url(&self, url: &str) -> Result<(String<64>, u16, String<128>), NetworkError> {
        // 简化的URL解析
        if url.starts_with("http://") {
            let url = &url[7..]; // 移除"http://"
            
            if let Some(slash_pos) = url.find('/') {
                let host_port = &url[..slash_pos];
                let path = &url[slash_pos..];
                
                let (host, port) = if let Some(colon_pos) = host_port.find(':') {
                    let host = &host_port[..colon_pos];
                    let port = host_port[colon_pos + 1..].parse().unwrap_or(80);
                    (host, port)
                } else {
                    (host_port, 80)
                };
                
                Ok((
                    String::from(host),
                    port,
                    String::from(path)
                ))
            } else {
                let (host, port) = if let Some(colon_pos) = url.find(':') {
                    let host = &url[..colon_pos];
                    let port = url[colon_pos + 1..].parse().unwrap_or(80);
                    (host, port)
                } else {
                    (url, 80)
                };
                
                Ok((
                    String::from(host),
                    port,
                    String::from("/")
                ))
            }
        } else {
            Err(NetworkError::InvalidParameter)
        }
    }
    
    /// 解析主机名
    fn resolve_hostname(&self, hostname: &str) -> Result<IpAddress, NetworkError> {
        // 简化的DNS解析，实际应该查询DNS服务器
        match hostname {
            "localhost" => Ok(IpAddress::V4([127, 0, 0, 1])),
            _ => {
                // 尝试解析为IP地址
                if let Ok(addr) = hostname.parse::<core::net::Ipv4Addr>() {
                    let octets = addr.octets();
                    Ok(IpAddress::V4(octets))
                } else {
                    Err(NetworkError::NoRoute)
                }
            }
        }
    }
    
    /// 构建HTTP请求
    fn build_http_request(
        &self,
        request: &HttpRequest,
        host: &str,
        path: &str,
    ) -> Result<String<1024>, NetworkError> {
        let mut http_request = String::new();
        
        // 请求行
        let method_str = match request.method {
            HttpMethod::Get => "GET",
            HttpMethod::Post => "POST",
            HttpMethod::Put => "PUT",
            HttpMethod::Delete => "DELETE",
            HttpMethod::Head => "HEAD",
            HttpMethod::Options => "OPTIONS",
            HttpMethod::Patch => "PATCH",
        };
        
        let _ = write!(http_request, "{} {} HTTP/1.1\r\n", method_str, path);
        
        // Host头
        let _ = write!(http_request, "Host: {}\r\n", host);
        
        // User-Agent头
        let _ = write!(http_request, "User-Agent: {}\r\n", self.user_agent);
        
        // 其他头部
        for (name, value) in &request.headers {
            let _ = write!(http_request, "{}: {}\r\n", name, value);
        }
        
        // 连接头
        let _ = write!(http_request, "Connection: close\r\n");
        
        // 空行
        let _ = write!(http_request, "\r\n");
        
        // 请求体
        if !request.body.is_empty() {
            for &byte in &request.body {
                let _ = write!(http_request, "{}", byte as char);
            }
        }
        
        Ok(http_request)
    }
    
    /// 接收响应
    fn receive_response(&mut self, socket_id: u32) -> Result<HttpResponse, NetworkError> {
        let mut response_data = Vec::<u8, 2048>::new();
        let mut buffer = [0u8; 256];
        
        // 接收数据
        loop {
            if let Some(socket) = self.socket_manager.get_socket_mut(socket_id) {
                match socket.receive(&mut buffer) {
                    Ok(0) => break, // 连接关闭
                    Ok(n) => {
                        for i in 0..n {
                            if response_data.push(buffer[i]).is_err() {
                                return Err(NetworkError::BufferFull);
                            }
                        }
                    }
                    Err(e) => return Err(e),
                }
            } else {
                return Err(NetworkError::InvalidSocket);
            }
        }
        
        // 解析HTTP响应
        self.parse_http_response(&response_data)
    }
    
    /// 解析HTTP响应
    fn parse_http_response(&self, data: &[u8]) -> Result<HttpResponse, NetworkError> {
        let response_str = core::str::from_utf8(data)
            .map_err(|_| NetworkError::ProtocolError)?;
        
        let mut lines = response_str.lines();
        
        // 解析状态行
        let status_line = lines.next().ok_or(NetworkError::ProtocolError)?;
        let status = self.parse_status_line(status_line)?;
        
        // 解析头部
        let mut headers = Vec::new();
        let mut body_start = 0;
        
        for (i, line) in lines.enumerate() {
            if line.is_empty() {
                body_start = i + 2; // +2 for status line and empty line
                break;
            }
            
            if let Some(colon_pos) = line.find(':') {
                let name = line[..colon_pos].trim();
                let value = line[colon_pos + 1..].trim();
                
                let _ = headers.push((
                    String::from(name),
                    String::from(value)
                ));
            }
        }
        
        // 解析响应体
        let mut body = Vec::new();
        let body_data = response_str.lines().skip(body_start).collect::<Vec<_>>().join("\n");
        
        for byte in body_data.bytes() {
            if body.push(byte).is_err() {
                break;
            }
        }
        
        Ok(HttpResponse {
            status,
            headers,
            body,
        })
    }
    
    /// 解析状态行
    fn parse_status_line(&self, status_line: &str) -> Result<u16, NetworkError> {
        let parts: Vec<&str> = status_line.split_whitespace().collect();
        if parts.len() >= 2 {
            parts[1].parse().map_err(|_| NetworkError::ProtocolError)
        } else {
            Err(NetworkError::ProtocolError)
        }
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> HttpClientStats {
        self.stats
    }
}
```

### MQTT客户端

```rust
/// MQTT消息类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MqttMessageType {
    Connect = 1,
    ConnAck = 2,
    Publish = 3,
    PubAck = 4,
    PubRec = 5,
    PubRel = 6,
    PubComp = 7,
    Subscribe = 8,
    SubAck = 9,
    Unsubscribe = 10,
    UnsubAck = 11,
    PingReq = 12,
    PingResp = 13,
    Disconnect = 14,
}

/// MQTT QoS级别
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MqttQoS {
    AtMostOnce = 0,
    AtLeastOnce = 1,
    ExactlyOnce = 2,
}

/// MQTT连接状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MqttConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Disconnecting,
}

/// MQTT消息
#[derive(Debug, Clone)]
pub struct MqttMessage {
    pub topic: String<128>,
    pub payload: Vec<u8, 256>,
    pub qos: MqttQoS,
    pub retain: bool,
    pub dup: bool,
}

/// MQTT客户端配置
#[derive(Debug, Clone)]
pub struct MqttConfig {
    pub client_id: String<64>,
    pub username: Option<String<64>>,
    pub password: Option<String<64>>,
    pub keep_alive: u16,
    pub clean_session: bool,
    pub will_topic: Option<String<128>>,
    pub will_message: Option<Vec<u8, 256>>,
    pub will_qos: MqttQoS,
    pub will_retain: bool,
}

/// MQTT客户端
pub struct MqttClient {
    socket_manager: SocketManager,
    socket_id: Option<u32>,
    state: MqttConnectionState,
    config: MqttConfig,
    packet_id: u16,
    subscriptions: Vec<String<128>, 16>,
    stats: MqttClientStats,
}

/// MQTT客户端统计
#[derive(Debug, Clone, Copy, Default)]
pub struct MqttClientStats {
    pub messages_published: u32,
    pub messages_received: u32,
    pub connection_attempts: u32,
    pub connection_failures: u32,
    pub ping_requests: u32,
    pub ping_responses: u32,
    pub bytes_sent: u64,
    pub bytes_received: u64,
}

impl MqttClient {
    /// 创建MQTT客户端
    pub fn new(config: MqttConfig) -> Self {
        Self {
            socket_manager: SocketManager::new(),
            socket_id: None,
            state: MqttConnectionState::Disconnected,
            config,
            packet_id: 1,
            subscriptions: Vec::new(),
            stats: MqttClientStats::default(),
        }
    }
    
    /// 连接到MQTT代理
    pub fn connect(&mut self, broker_addr: SocketAddr) -> Result<(), NetworkError> {
        if self.state != MqttConnectionState::Disconnected {
            return Err(NetworkError::InvalidState);
        }
        
        // 创建TCP连接
        let socket_id = self.socket_manager.create_socket(SocketType::Stream)?;
        
        if let Some(socket) = self.socket_manager.get_socket_mut(socket_id) {
            socket.connect(broker_addr)?;
            self.socket_id = Some(socket_id);
            self.state = MqttConnectionState::Connecting;
            
            // 发送CONNECT包
            self.send_connect_packet()?;
            
            self.stats.connection_attempts += 1;
            Ok(())
        } else {
            Err(NetworkError::InvalidSocket)
        }
    }
    
    /// 断开连接
    pub fn disconnect(&mut self) -> Result<(), NetworkError> {
        if self.state == MqttConnectionState::Connected {
            self.send_disconnect_packet()?;
        }
        
        if let Some(socket_id) = self.socket_id.take() {
            self.socket_manager.close_socket(socket_id)?;
        }
        
        self.state = MqttConnectionState::Disconnected;
        Ok(())
    }
    
    /// 发布消息
    pub fn publish(&mut self, message: &MqttMessage) -> Result<(), NetworkError> {
        if self.state != MqttConnectionState::Connected {
            return Err(NetworkError::NotConnected);
        }
        
        self.send_publish_packet(message)?;
        self.stats.messages_published += 1;
        Ok(())
    }
    
    /// 订阅主题
    pub fn subscribe(&mut self, topic: &str, qos: MqttQoS) -> Result<(), NetworkError> {
        if self.state != MqttConnectionState::Connected {
            return Err(NetworkError::NotConnected);
        }
        
        self.send_subscribe_packet(topic, qos)?;
        
        // 添加到订阅列表
        if !self.subscriptions.iter().any(|t| t == topic) {
            let _ = self.subscriptions.push(String::from(topic));
        }
        
        Ok(())
    }
    
    /// 取消订阅
    pub fn unsubscribe(&mut self, topic: &str) -> Result<(), NetworkError> {
        if self.state != MqttConnectionState::Connected {
            return Err(NetworkError::NotConnected);
        }
        
        self.send_unsubscribe_packet(topic)?;
        
        // 从订阅列表移除
        self.subscriptions.retain(|t| t != topic);
        
        Ok(())
    }
    
    /// 发送CONNECT包
    fn send_connect_packet(&mut self) -> Result<(), NetworkError> {
        let mut packet = Vec::<u8, 512>::new();
        
        // 固定头部
        let _ = packet.push(0x10); // CONNECT消息类型
        
        // 可变头部和载荷
        let mut variable_header = Vec::<u8, 256>::new();
        
        // 协议名称
        let protocol_name = b"MQTT";
        let _ = variable_header.push(0x00);
        let _ = variable_header.push(protocol_name.len() as u8);
        for &byte in protocol_name {
            let _ = variable_header.push(byte);
        }
        
        // 协议版本
        let _ = variable_header.push(0x04); // MQTT 3.1.1
        
        // 连接标志
        let mut connect_flags = 0u8;
        if self.config.clean_session {
            connect_flags |= 0x02;
        }
        if self.config.will_topic.is_some() {
            connect_flags |= 0x04;
            connect_flags |= (self.config.will_qos as u8) << 3;
            if self.config.will_retain {
                connect_flags |= 0x20;
            }
        }
        if self.config.password.is_some() {
            connect_flags |= 0x40;
        }
        if self.config.username.is_some() {
            connect_flags |= 0x80;
        }
        let _ = variable_header.push(connect_flags);
        
        // Keep Alive
        let _ = variable_header.push((self.config.keep_alive >> 8) as u8);
        let _ = variable_header.push((self.config.keep_alive & 0xFF) as u8);
        
        // 载荷：客户端ID
        let client_id = self.config.client_id.as_bytes();
        let _ = variable_header.push((client_id.len() >> 8) as u8);
        let _ = variable_header.push((client_id.len() & 0xFF) as u8);
        for &byte in client_id {
            let _ = variable_header.push(byte);
        }
        
        // 遗嘱主题和消息
        if let Some(ref will_topic) = self.config.will_topic {
            let topic_bytes = will_topic.as_bytes();
            let _ = variable_header.push((topic_bytes.len() >> 8) as u8);
            let _ = variable_header.push((topic_bytes.len() & 0xFF) as u8);
            for &byte in topic_bytes {
                let _ = variable_header.push(byte);
            }
            
            if let Some(ref will_message) = self.config.will_message {
                let _ = variable_header.push((will_message.len() >> 8) as u8);
                let _ = variable_header.push((will_message.len() & 0xFF) as u8);
                for &byte in will_message {
                    let _ = variable_header.push(byte);
                }
            }
        }
        
        // 用户名和密码
        if let Some(ref username) = self.config.username {
            let username_bytes = username.as_bytes();
            let _ = variable_header.push((username_bytes.len() >> 8) as u8);
            let _ = variable_header.push((username_bytes.len() & 0xFF) as u8);
            for &byte in username_bytes {
                let _ = variable_header.push(byte);
            }
        }
        
        if let Some(ref password) = self.config.password {
            let password_bytes = password.as_bytes();
            let _ = variable_header.push((password_bytes.len() >> 8) as u8);
            let _ = variable_header.push((password_bytes.len() & 0xFF) as u8);
            for &byte in password_bytes {
                let _ = variable_header.push(byte);
            }
        }
        
        // 剩余长度
        self.encode_remaining_length(&mut packet, variable_header.len())?;
        
        // 添加可变头部和载荷
        for &byte in &variable_header {
            let _ = packet.push(byte);
        }
        
        // 发送数据包
        self.send_packet(&packet)
    }
    
    /// 发送PUBLISH包
    fn send_publish_packet(&mut self, message: &MqttMessage) -> Result<(), NetworkError> {
        let mut packet = Vec::<u8, 512>::new();
        
        // 固定头部
        let mut fixed_header = 0x30u8; // PUBLISH消息类型
        if message.dup {
            fixed_header |= 0x08;
        }
        fixed_header |= (message.qos as u8) << 1;
        if message.retain {
            fixed_header |= 0x01;
        }
        let _ = packet.push(fixed_header);
        
        // 可变头部和载荷
        let mut variable_header = Vec::<u8, 256>::new();
        
        // 主题名称
        let topic_bytes = message.topic.as_bytes();
        let _ = variable_header.push((topic_bytes.len() >> 8) as u8);
        let _ = variable_header.push((topic_bytes.len() & 0xFF) as u8);
        for &byte in topic_bytes {
            let _ = variable_header.push(byte);
        }
        
        // 包标识符（QoS > 0时需要）
        if message.qos != MqttQoS::AtMostOnce {
            let _ = variable_header.push((self.packet_id >> 8) as u8);
            let _ = variable_header.push((self.packet_id & 0xFF) as u8);
            self.packet_id = self.packet_id.wrapping_add(1);
            if self.packet_id == 0 {
                self.packet_id = 1;
            }
        }
        
        // 载荷（消息内容）
        for &byte in &message.payload {
            let _ = variable_header.push(byte);
        }
        
        // 剩余长度
        self.encode_remaining_length(&mut packet, variable_header.len())?;
        
        // 添加可变头部和载荷
        for &byte in &variable_header {
            let _ = packet.push(byte);
        }
        
        // 发送数据包
        self.send_packet(&packet)
    }
    
    /// 发送SUBSCRIBE包
    fn send_subscribe_packet(&mut self, topic: &str, qos: MqttQoS) -> Result<(), NetworkError> {
        let mut packet = Vec::<u8, 256>::new();
        
        // 固定头部
        let _ = packet.push(0x82); // SUBSCRIBE消息类型
        
        // 可变头部和载荷
        let mut variable_header = Vec::<u8, 128>::new();
        
        // 包标识符
        let _ = variable_header.push((self.packet_id >> 8) as u8);
        let _ = variable_header.push((self.packet_id & 0xFF) as u8);
        self.packet_id = self.packet_id.wrapping_add(1);
        if self.packet_id == 0 {
            self.packet_id = 1;
        }
        
        // 主题过滤器
        let topic_bytes = topic.as_bytes();
        let _ = variable_header.push((topic_bytes.len() >> 8) as u8);
        let _ = variable_header.push((topic_bytes.len() & 0xFF) as u8);
        for &byte in topic_bytes {
            let _ = variable_header.push(byte);
        }
        
        // QoS
        let _ = variable_header.push(qos as u8);
        
        // 剩余长度
        self.encode_remaining_length(&mut packet, variable_header.len())?;
        
        // 添加可变头部和载荷
        for &byte in &variable_header {
            let _ = packet.push(byte);
        }
        
        // 发送数据包
        self.send_packet(&packet)
    }
    
    /// 发送UNSUBSCRIBE包
    fn send_unsubscribe_packet(&mut self, topic: &str) -> Result<(), NetworkError> {
        let mut packet = Vec::<u8, 256>::new();
        
        // 固定头部
        let _ = packet.push(0xA2); // UNSUBSCRIBE消息类型
        
        // 可变头部和载荷
        let mut variable_header = Vec::<u8, 128>::new();
        
        // 包标识符
        let _ = variable_header.push((self.packet_id >> 8) as u8);
        let _ = variable_header.push((self.packet_id & 0xFF) as u8);
        self.packet_id = self.packet_id.wrapping_add(1);
        if self.packet_id == 0 {
            self.packet_id = 1;
        }
        
        // 主题过滤器
        let topic_bytes = topic.as_bytes();
        let _ = variable_header.push((topic_bytes.len() >> 8) as u8);
        let _ = variable_header.push((topic_bytes.len() & 0xFF) as u8);
        for &byte in topic_bytes {
            let _ = variable_header.push(byte);
        }
        
        // 剩余长度
        self.encode_remaining_length(&mut packet, variable_header.len())?;
        
        // 添加可变头部和载荷
        for &byte in &variable_header {
            let _ = packet.push(byte);
        }
        
        // 发送数据包
        self.send_packet(&packet)
    }
    
    /// 发送DISCONNECT包
    fn send_disconnect_packet(&mut self) -> Result<(), NetworkError> {
        let packet = [0xE0, 0x00]; // DISCONNECT消息类型，剩余长度为0
        self.send_packet(&packet)
    }
    
    /// 编码剩余长度
    fn encode_remaining_length(
        &self,
        packet: &mut Vec<u8, 512>,
        length: usize,
    ) -> Result<(), NetworkError> {
        let mut remaining_length = length;
        
        loop {
            let mut encoded_byte = (remaining_length % 128) as u8;
            remaining_length /= 128;
            
            if remaining_length > 0 {
                encoded_byte |= 128;
            }
            
            packet.push(encoded_byte).map_err(|_| NetworkError::BufferFull)?;
            
            if remaining_length == 0 {
                break;
            }
        }
        
        Ok(())
    }
    
    /// 发送数据包
    fn send_packet(&mut self, packet: &[u8]) -> Result<(), NetworkError> {
        if let Some(socket_id) = self.socket_id {
            if let Some(socket) = self.socket_manager.get_socket_mut(socket_id) {
                socket.send(packet)?;
                self.stats.bytes_sent += packet.len() as u64;
                Ok(())
            } else {
                Err(NetworkError::InvalidSocket)
            }
        } else {
            Err(NetworkError::NotConnected)
        }
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> MqttClientStats {
        self.stats
    }
}
```

## 网络驱动接口

### 网络设备抽象

```rust
/// 网络设备特征
pub trait NetworkDevice {
    /// 初始化设备
    fn init(&mut self) -> Result<(), NetworkError>;
    
    /// 启动设备
    fn start(&mut self) -> Result<(), NetworkError>;
    
    /// 停止设备
    fn stop(&mut self) -> Result<(), NetworkError>;
    
    /// 发送数据包
    fn transmit(&mut self, packet: &NetworkPacket) -> Result<(), NetworkError>;
    
    /// 接收数据包
    fn receive(&mut self) -> Result<Option<NetworkPacket>, NetworkError>;
    
    /// 获取MAC地址
    fn get_mac_address(&self) -> [u8; 6];
    
    /// 设置MAC地址
    fn set_mac_address(&mut self, mac: [u8; 6]) -> Result<(), NetworkError>;
    
    /// 获取设备状态
    fn get_status(&self) -> InterfaceStatus;
    
    /// 获取设备统计信息
    fn get_statistics(&self) -> InterfaceStats;
    
    /// 设置接收过滤器
    fn set_rx_filter(&mut self, filter: RxFilter) -> Result<(), NetworkError>;
}

/// 接收过滤器
#[derive(Debug, Clone, Copy)]
pub struct RxFilter {
    pub promiscuous: bool,
    pub broadcast: bool,
    pub multicast: bool,
    pub unicast: bool,
}

/// 以太网设备
pub struct EthernetDevice {
    mac_address: [u8; 6],
    status: InterfaceStatus,
    statistics: InterfaceStats,
    rx_filter: RxFilter,
    tx_queue: Vec<NetworkPacket, 16>,
    rx_queue: Vec<NetworkPacket, 16>,
}

impl EthernetDevice {
    /// 创建以太网设备
    pub fn new(mac_address: [u8; 6]) -> Self {
        Self {
            mac_address,
            status: InterfaceStatus::Down,
            statistics: InterfaceStats::default(),
            rx_filter: RxFilter {
                promiscuous: false,
                broadcast: true,
                multicast: false,
                unicast: true,
            },
            tx_queue: Vec::new(),
            rx_queue: Vec::new(),
        }
    }
}

impl NetworkDevice for EthernetDevice {
    fn init(&mut self) -> Result<(), NetworkError> {
        // 初始化以太网控制器
        // 这里应该包含具体的硬件初始化代码
        self.status = InterfaceStatus::Down;
        Ok(())
    }
    
    fn start(&mut self) -> Result<(), NetworkError> {
        if self.status == InterfaceStatus::Down {
            // 启动以太网控制器
            // 配置DMA、中断等
            self.status = InterfaceStatus::Up;
        }
        Ok(())
    }
    
    fn stop(&mut self) -> Result<(), NetworkError> {
        if self.status == InterfaceStatus::Up {
            // 停止以太网控制器
            self.status = InterfaceStatus::Down;
            
            // 清空队列
            self.tx_queue.clear();
            self.rx_queue.clear();
        }
        Ok(())
    }
    
    fn transmit(&mut self, packet: &NetworkPacket) -> Result<(), NetworkError> {
        if self.status != InterfaceStatus::Up {
            return Err(NetworkError::InterfaceDown);
        }
        
        // 添加到发送队列
        self.tx_queue.push(packet.clone())
            .map_err(|_| NetworkError::BufferFull)?;
        
        // 实际的硬件发送应该在这里实现
        // 这里只是模拟
        self.statistics.packets_sent += 1;
        self.statistics.bytes_sent += packet.length as u64;
        
        Ok(())
    }
    
    fn receive(&mut self) -> Result<Option<NetworkPacket>, NetworkError> {
        if self.status != InterfaceStatus::Up {
            return Err(NetworkError::InterfaceDown);
        }
        
        // 从接收队列获取数据包
        if let Some(packet) = self.rx_queue.pop() {
            self.statistics.packets_received += 1;
            self.statistics.bytes_received += packet.length as u64;
            Ok(Some(packet))
        } else {
            Ok(None)
        }
    }
    
    fn get_mac_address(&self) -> [u8; 6] {
        self.mac_address
    }
    
    fn set_mac_address(&mut self, mac: [u8; 6]) -> Result<(), NetworkError> {
        self.mac_address = mac;
        // 更新硬件MAC地址寄存器
        Ok(())
    }
    
    fn get_status(&self) -> InterfaceStatus {
        self.status
    }
    
    fn get_statistics(&self) -> InterfaceStats {
        self.statistics
    }
    
    fn set_rx_filter(&mut self, filter: RxFilter) -> Result<(), NetworkError> {
        self.rx_filter = filter;
        // 配置硬件接收过滤器
        Ok(())
    }
}
```

## 性能优化

### 零拷贝优化

```rust
/// 零拷贝缓冲区
pub struct ZeroCopyBuffer {
    data: *mut u8,
    length: usize,
    capacity: usize,
    reference_count: u32,
}

/// 缓冲区描述符
#[derive(Debug, Clone, Copy)]
pub struct BufferDescriptor {
    pub address: usize,
    pub length: usize,
    pub flags: BufferFlags,
}

/// 缓冲区标志
#[derive(Debug, Clone, Copy, Default)]
pub struct BufferFlags {
    pub owned: bool,
    pub readonly: bool,
    pub dma_coherent: bool,
}

impl ZeroCopyBuffer {
    /// 创建零拷贝缓冲区
    pub unsafe fn new(data: *mut u8, length: usize, capacity: usize) -> Self {
        Self {
            data,
            length,
            capacity,
            reference_count: 1,
        }
    }
    
    /// 增加引用计数
    pub fn clone_ref(&mut self) -> Self {
        self.reference_count += 1;
        Self {
            data: self.data,
            length: self.length,
            capacity: self.capacity,
            reference_count: 1, // 新引用的计数
        }
    }
    
    /// 获取数据指针
    pub fn as_ptr(&self) -> *const u8 {
        self.data
    }
    
    /// 获取可变数据指针
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.data
    }
    
    /// 获取数据切片
    pub unsafe fn as_slice(&self) -> &[u8] {
        core::slice::from_raw_parts(self.data, self.length)
    }
    
    /// 获取可变数据切片
    pub unsafe fn as_mut_slice(&mut self) -> &mut [u8] {
        core::slice::from_raw_parts_mut(self.data, self.length)
    }
}

impl Drop for ZeroCopyBuffer {
    fn drop(&mut self) {
        self.reference_count -= 1;
        if self.reference_count == 0 {
            // 释放缓冲区
            // 这里应该调用适当的内存释放函数
        }
    }
}
```

### 网络中断处理

```rust
/// 网络中断处理器
pub struct NetworkInterruptHandler {
    rx_queue: Queue<NetworkPacket, 32>,
    tx_complete_queue: Queue<u32, 16>, // 发送完成的包ID
    error_queue: Queue<NetworkError, 8>,
    stats: InterruptStats,
}

/// 中断统计
#[derive(Debug, Clone, Copy, Default)]
pub struct InterruptStats {
    pub rx_interrupts: u32,
    pub tx_interrupts: u32,
    pub error_interrupts: u32,
    pub missed_interrupts: u32,
}

impl NetworkInterruptHandler {
    /// 创建中断处理器
    pub fn new() -> Self {
        Self {
            rx_queue: Queue::new(),
            tx_complete_queue: Queue::new(),
            error_queue: Queue::new(),
            stats: InterruptStats::default(),
        }
    }
    
    /// 处理接收中断
    pub fn handle_rx_interrupt(&mut self, packet: NetworkPacket) {
        if self.rx_queue.enqueue(packet).is_err() {
            self.stats.missed_interrupts += 1;
        } else {
            self.stats.rx_interrupts += 1;
        }
    }
    
    /// 处理发送完成中断
    pub fn handle_tx_complete_interrupt(&mut self, packet_id: u32) {
        if self.tx_complete_queue.enqueue(packet_id).is_err() {
            self.stats.missed_interrupts += 1;
        } else {
            self.stats.tx_interrupts += 1;
        }
    }
    
    /// 处理错误中断
    pub fn handle_error_interrupt(&mut self, error: NetworkError) {
        if self.error_queue.enqueue(error).is_err() {
            self.stats.missed_interrupts += 1;
        } else {
            self.stats.error_interrupts += 1;
        }
    }
    
    /// 获取接收的数据包
    pub fn get_received_packet(&mut self) -> Option<NetworkPacket> {
        self.rx_queue.dequeue()
    }
    
    /// 获取发送完成的包ID
    pub fn get_tx_complete(&mut self) -> Option<u32> {
        self.tx_complete_queue.dequeue()
    }
    
    /// 获取错误
    pub fn get_error(&mut self) -> Option<NetworkError> {
        self.error_queue.dequeue()
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> InterruptStats {
        self.stats
    }
}
```

## 总结

网络协议栈集成是RTOS的重要组成部分，需要考虑以下关键点：

### 设计原则

1. **分层架构**：采用标准的网络分层模型，便于维护和扩展
2. **模块化设计**：各协议层相对独立，支持灵活配置
3. **高效内存管理**：使用数据包池和零拷贝技术
4. **实时性保证**：优化中断处理和数据路径
5. **可扩展性**：支持多种网络协议和设备类型

### 性能优化

1. **零拷贝**：减少数据复制开销
2. **中断合并**：减少中断频率
3. **批量处理**：提高数据包处理效率
4. **内存预分配**：避免动态内存分配
5. **硬件加速**：利用网络处理器和DMA

### 可靠性保证

1. **错误处理**：完善的错误检测和恢复机制
2. **流量控制**：防止缓冲区溢出
3. **超时处理**：避免资源泄漏
4. **状态管理**：正确的连接状态转换
5. **资源管理**：及时释放网络资源

网络协议栈的实现需要深入理解网络协议原理和嵌入式系统特点，在功能完整性和资源效率之间找到平衡点。