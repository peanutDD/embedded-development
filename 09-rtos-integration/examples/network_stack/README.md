# 网络协议栈示例

这个示例演示了如何在STM32F4微控制器上集成和使用网络协议栈，实现TCP/UDP通信、HTTP客户端和以太网连接功能。

## 功能特性

### 网络协议支持
- **TCP协议**: 可靠的面向连接通信
- **UDP协议**: 快速的无连接通信
- **HTTP客户端**: 支持GET/POST请求
- **DHCP客户端**: 自动IP地址分配
- **ARP协议**: 地址解析协议
- **ICMP协议**: 网络控制消息协议

### 硬件支持
- **以太网控制器**: ENC28J60 SPI接口
- **网络接口**: 10Mbps以太网
- **缓冲管理**: 高效的数据包缓冲
- **中断处理**: 异步网络事件处理

### 应用功能
- **TCP客户端**: 连接远程服务器
- **UDP回显**: 数据包回显服务
- **HTTP请求**: 发送HTTP GET/POST请求
- **网络监控**: 实时网络统计信息
- **错误处理**: 完善的网络错误处理

## 硬件要求

### 开发板
- STM32F407VG开发板
- 168MHz ARM Cortex-M4处理器
- 1MB Flash存储器
- 192KB RAM (128KB + 64KB CCM)

### 网络模块
- ENC28J60以太网控制器模块
- SPI接口连接
- 10Mbps以太网PHY
- RJ45网络接口

### 连接配置
```
STM32F407    ENC28J60
---------    --------
PA5 (SCK) -> SCK
PA6 (MISO)-> SO
PA7 (MOSI)-> SI
PA4 (CS)  -> CS
PA3 (RST) -> RST
3.3V      -> VCC
GND       -> GND
```

## 项目结构

```
network_stack/
├── Cargo.toml              # 项目配置和依赖
├── README.md               # 项目说明文档
├── src/
│   └── main.rs            # 主程序文件
├── examples/              # 示例程序
│   ├── tcp_client.rs      # TCP客户端示例
│   ├── udp_echo.rs        # UDP回显示例
│   ├── http_client.rs     # HTTP客户端示例
│   └── dhcp_client.rs     # DHCP客户端示例
├── memory.x               # 内存布局配置
└── .cargo/
    └── config.toml        # Cargo配置
```

## 代码说明

### 网络设备抽象
```rust
// 以太网设备适配器
struct EthernetDevice {
    enc28j60: Enc28j60<SPI, CS, INT, RST>,
}

// 实现smoltcp Device trait
impl Device for EthernetDevice {
    type RxToken<'a> = RxToken<'a>;
    type TxToken<'a> = TxToken<'a>;
    
    fn receive(&mut self, timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)>;
    fn transmit(&mut self, timestamp: Instant) -> Option<Self::TxToken<'_>>;
    fn capabilities(&self) -> DeviceCapabilities;
}
```

### 网络接口配置
```rust
// 创建网络接口
let config = Config::new(EthernetAddress(MAC_ADDRESS).into());
let mut iface = Interface::new(config, &mut device, Instant::ZERO);

// 配置IP地址
iface.update_ip_addrs(|ip_addrs| {
    ip_addrs.push(IpCidr::new(
        IpAddress::v4(192, 168, 1, 100),
        24,
    )).unwrap();
});

// 配置默认路由
iface.routes_mut().add_default_ipv4_route(
    Ipv4Address::new(192, 168, 1, 1)
).unwrap();
```

### TCP客户端实现
```rust
// 创建TCP套接字
let tcp_rx_buffer = tcp::SocketBuffer::new(&mut rx_buffer[..]);
let tcp_tx_buffer = tcp::SocketBuffer::new(&mut tx_buffer[..]);
let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);

// 连接到远程服务器
socket.connect(context, (server_ip, 80), local_port)?;

// 发送HTTP请求
let request = "GET / HTTP/1.1\r\nHost: example.com\r\n\r\n";
socket.send_slice(request.as_bytes())?;

// 接收响应
let mut buffer = [0u8; 1024];
let received = socket.recv_slice(&mut buffer)?;
```

### UDP通信实现
```rust
// 创建UDP套接字
let udp_rx_buffer = udp::PacketBuffer::new(&mut rx_meta[..], &mut rx_buffer[..]);
let udp_tx_buffer = udp::PacketBuffer::new(&mut tx_meta[..], &mut tx_buffer[..]);
let udp_socket = udp::Socket::new(udp_rx_buffer, udp_tx_buffer);

// 绑定本地端口
socket.bind(8080)?;

// 发送UDP数据包
let data = b"Hello, UDP!";
socket.send_slice(data, (remote_ip, remote_port).into())?;

// 接收UDP数据包
let (received, remote_addr) = socket.recv_slice(&mut buffer)?;
```

## 网络配置

### 静态IP配置
```rust
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
const IP_ADDRESS: [u8; 4] = [192, 168, 1, 100];
const GATEWAY: [u8; 4] = [192, 168, 1, 1];
const SUBNET_MASK: [u8; 4] = [255, 255, 255, 0];
```

### DHCP配置
```rust
// 创建DHCP套接字
let dhcp_socket = dhcpv4::Socket::new();
let dhcp_handle = sockets.add(dhcp_socket);

// 启动DHCP客户端
let socket = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle);
socket.start_discovery(timestamp)?;
```

## 编译和烧录

### 环境准备
```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf
cargo install cargo-embed
cargo install probe-run
```

### 编译项目
```bash
# 进入项目目录
cd examples/network_stack

# 编译项目
cargo build --release

# 检查代码
cargo check

# 运行测试
cargo test --lib
```

### 烧录和调试
```bash
# 使用cargo-embed烧录
cargo embed --release

# 使用probe-run运行
cargo run --release

# 使用GDB调试
cargo embed --release --gdb
```

## 预期行为

### 启动序列
1. **硬件初始化**: 配置时钟、GPIO和SPI接口
2. **网络初始化**: 初始化ENC28J60和网络接口
3. **IP配置**: 设置静态IP或启动DHCP客户端
4. **应用启动**: 开始网络通信测试

### 网络测试
1. **TCP连接测试**: 连接到HTTP服务器
2. **HTTP请求**: 发送GET请求并接收响应
3. **UDP回显**: 发送UDP数据包并处理回显
4. **网络监控**: 显示网络统计信息

### 日志输出
```
Starting network stack example...
Network interface initialized
IP: 192.168.1.100
Gateway: 192.168.1.1
Starting TCP connection test...
TCP connection established
HTTP request sent: 65 bytes
HTTP response received: 512 bytes
UDP packet sent to 8.8.8.8:53
Network Statistics:
  Packets: TX=5, RX=3
  Bytes: TX=1024, RX=768
```

## 学习要点

### 网络协议栈概念
- **分层架构**: 物理层、数据链路层、网络层、传输层
- **协议实现**: TCP/UDP/IP/ARP/ICMP协议栈
- **套接字编程**: 网络应用程序接口
- **缓冲管理**: 高效的数据包处理

### 嵌入式网络编程
- **资源约束**: 内存和处理能力限制
- **实时性要求**: 网络延迟和响应时间
- **错误处理**: 网络故障和恢复机制
- **性能优化**: 零拷贝和批量处理

### 最佳实践
- **模块化设计**: 清晰的网络层次结构
- **异步处理**: 非阻塞网络操作
- **资源管理**: 合理的缓冲区分配
- **错误恢复**: 健壮的错误处理机制

## 性能分析

### 内存使用
- **代码大小**: ~45KB Flash
- **静态内存**: ~8KB RAM
- **网络缓冲**: ~4KB RAM
- **栈使用**: ~2KB per task

### 网络性能
- **吞吐量**: ~8Mbps (受ENC28J60限制)
- **延迟**: ~1-5ms (本地网络)
- **并发连接**: 支持多个TCP/UDP套接字
- **数据包处理**: ~1000 pps

## 扩展建议

### 功能扩展
- **TLS/SSL支持**: 加密网络通信
- **WebSocket客户端**: 实时双向通信
- **MQTT客户端**: 物联网消息传输
- **CoAP协议**: 受约束应用协议
- **NTP客户端**: 网络时间同步

### 性能优化
- **DMA传输**: 减少CPU负载
- **中断合并**: 提高网络效率
- **零拷贝**: 减少内存拷贝开销
- **硬件加速**: 利用硬件网络功能

### 安全增强
- **防火墙规则**: 网络访问控制
- **入侵检测**: 异常流量监控
- **加密通信**: 数据传输保护
- **认证机制**: 设备身份验证

## 故障排除

### 常见问题

**网络连接失败**
- 检查硬件连接和电源
- 验证网络配置和IP地址
- 确认路由器和交换机设置

**数据传输错误**
- 检查SPI通信和时序
- 验证缓冲区大小和对齐
- 确认网络协议配置

**性能问题**
- 优化缓冲区大小
- 调整网络轮询频率
- 检查CPU使用率

### 调试技巧

**网络抓包**
```bash
# 使用Wireshark抓包分析
sudo tcpdump -i eth0 host 192.168.1.100
```

**RTT日志分析**
```rust
// 启用详细网络日志
log::set_max_level(log::LevelFilter::Debug);
```

**性能监控**
```rust
// 监控网络统计
fn monitor_network_performance() {
    let stats = get_network_stats();
    info!("Throughput: {} Kbps", stats.throughput_kbps);
    info!("Packet loss: {}%", stats.packet_loss_percent);
}
```

## 相关资源

- [smoltcp文档](https://docs.rs/smoltcp/)
- [ENC28J60数据手册](https://www.microchip.com/en-us/product/ENC28J60)
- [STM32F4参考手册](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [TCP/IP协议详解](https://tools.ietf.org/rfc/rfc793.txt)
- [嵌入式网络编程指南](https://github.com/rust-embedded/book)