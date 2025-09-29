#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{gpioa::*, gpiob::*, gpioc::*, Alternate, Output, PushPull},
    pac,
    prelude::*,
    spi::{Spi, NoMiso, NoMosi, NoSck},
    timer::Timer,
};

use smoltcp::{
    iface::{Config, Interface, SocketSet},
    phy::{Device, DeviceCapabilities, Medium, RxToken, TxToken},
    socket::{tcp, udp, dhcpv4},
    time::{Duration, Instant},
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
};

use heapless::{String, Vec};
use log::info;
use rtt_target::{rprintln, rtt_init_print};
use rtt_log::RTTLogger;

use enc28j60::Enc28j60;

// 网络缓冲区大小
const RX_BUFFER_SIZE: usize = 1024;
const TX_BUFFER_SIZE: usize = 1024;
const SOCKET_BUFFER_SIZE: usize = 2048;

// 网络配置
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
const IP_ADDRESS: [u8; 4] = [192, 168, 1, 100];
const GATEWAY: [u8; 4] = [192, 168, 1, 1];
const SUBNET_MASK: [u8; 4] = [255, 255, 255, 0];

// 全局网络设备
static mut ETH_DEVICE: Option<EthernetDevice> = None;
static mut INTERFACE: Option<Interface> = None;
static mut SOCKETS: Option<SocketSet> = None;

// 缓冲区
static mut RX_BUFFER: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];
static mut TX_BUFFER: [u8; TX_BUFFER_SIZE] = [0; TX_BUFFER_SIZE];
static mut TCP_RX_BUFFER: [u8; SOCKET_BUFFER_SIZE] = [0; SOCKET_BUFFER_SIZE];
static mut TCP_TX_BUFFER: [u8; SOCKET_BUFFER_SIZE] = [0; SOCKET_BUFFER_SIZE];
static mut UDP_RX_BUFFER: [u8; SOCKET_BUFFER_SIZE] = [0; SOCKET_BUFFER_SIZE];
static mut UDP_TX_BUFFER: [u8; SOCKET_BUFFER_SIZE] = [0; SOCKET_BUFFER_SIZE];

#[entry]
fn main() -> ! {
    // 初始化RTT日志
    rtt_init_print!();
    static LOGGER: RTTLogger = RTTLogger::new(log::LevelFilter::Info);
    rtt_log::init(&LOGGER).unwrap();
    
    info!("Starting network stack example...");
    
    // 初始化硬件
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置SPI用于ENC28J60
    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    let cs = gpioa.pa4.into_push_pull_output();
    let rst = gpioa.pa3.into_push_pull_output();
    
    let spi = Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        enc28j60::MODE,
        1.mhz(),
        &clocks,
    );
    
    // 初始化ENC28J60以太网控制器
    let mut enc28j60 = Enc28j60::new(
        spi,
        cs,
        enc28j60::Unconnected,
        rst,
        &mut cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().raw()),
        7 * 1024,
        MAC_ADDRESS,
    ).unwrap();
    
    // 创建以太网设备适配器
    let eth_device = EthernetDevice::new(enc28j60);
    
    // 配置网络接口
    let config = Config::new(EthernetAddress(MAC_ADDRESS).into());
    let mut iface = Interface::new(config, &mut eth_device, Instant::ZERO);
    
    // 配置IP地址
    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs.push(IpCidr::new(
            IpAddress::v4(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]),
            24,
        )).unwrap();
    });
    
    // 配置路由
    iface.routes_mut().add_default_ipv4_route(
        Ipv4Address::new(GATEWAY[0], GATEWAY[1], GATEWAY[2], GATEWAY[3])
    ).unwrap();
    
    // 创建套接字集合
    let mut sockets = SocketSet::new(Vec::new());
    
    // 创建TCP套接字
    let tcp_rx_buffer = unsafe { tcp::SocketBuffer::new(&mut TCP_RX_BUFFER[..]) };
    let tcp_tx_buffer = unsafe { tcp::SocketBuffer::new(&mut TCP_TX_BUFFER[..]) };
    let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);
    let tcp_handle = sockets.add(tcp_socket);
    
    // 创建UDP套接字
    let udp_rx_buffer = unsafe { udp::PacketBuffer::new(&mut UDP_RX_BUFFER[..], &mut [][..]) };
    let udp_tx_buffer = unsafe { udp::PacketBuffer::new(&mut UDP_TX_BUFFER[..], &mut [][..]) };
    let udp_socket = udp::Socket::new(udp_rx_buffer, udp_tx_buffer);
    let udp_handle = sockets.add(udp_socket);
    
    // 存储全局变量
    unsafe {
        ETH_DEVICE = Some(eth_device);
        INTERFACE = Some(iface);
        SOCKETS = Some(sockets);
    }
    
    info!("Network interface initialized");
    info!("IP: {}.{}.{}.{}", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    info!("Gateway: {}.{}.{}.{}", GATEWAY[0], GATEWAY[1], GATEWAY[2], GATEWAY[3]);
    
    // 网络应用示例
    let mut app = NetworkApplication::new(tcp_handle, udp_handle);
    
    // 主循环
    let mut timestamp = Instant::ZERO;
    loop {
        // 更新时间戳
        timestamp += Duration::from_millis(10);
        
        unsafe {
            if let (Some(ref mut iface), Some(ref mut sockets), Some(ref mut device)) = 
                (INTERFACE.as_mut(), SOCKETS.as_mut(), ETH_DEVICE.as_mut()) {
                
                // 处理网络数据包
                let _ = iface.poll(timestamp, device, sockets);
                
                // 运行网络应用
                app.run(iface, sockets, timestamp);
            }
        }
        
        // 简单延时
        cortex_m::asm::delay(1680000); // ~10ms at 168MHz
    }
}

// 以太网设备适配器
struct EthernetDevice {
    enc28j60: Enc28j60<
        Spi<pac::SPI1, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>,
        PA4<Output<PushPull>>,
        enc28j60::Unconnected,
        PA3<Output<PushPull>>
    >,
}

impl EthernetDevice {
    fn new(enc28j60: Enc28j60<
        Spi<pac::SPI1, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>,
        PA4<Output<PushPull>>,
        enc28j60::Unconnected,
        PA3<Output<PushPull>>
    >) -> Self {
        Self { enc28j60 }
    }
}

impl Device for EthernetDevice {
    type RxToken<'a> = RxToken where Self: 'a;
    type TxToken<'a> = TxToken where Self: 'a;
    
    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if self.enc28j60.pending_packets().unwrap_or(0) > 0 {
            Some((RxToken(&mut self.enc28j60), TxToken(&mut self.enc28j60)))
        } else {
            None
        }
    }
    
    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        Some(TxToken(&mut self.enc28j60))
    }
    
    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1500;
        caps.max_burst_size = Some(1);
        caps.medium = Medium::Ethernet;
        caps
    }
}

// 接收令牌
struct RxToken<'a>(&'a mut Enc28j60<
    Spi<pac::SPI1, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>,
    PA4<Output<PushPull>>,
    enc28j60::Unconnected,
    PA3<Output<PushPull>>
>);

impl<'a> RxToken for RxToken<'a> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buffer = [0u8; 1518]; // 最大以太网帧大小
        let len = self.0.receive(&mut buffer).unwrap_or(0);
        f(&mut buffer[..len])
    }
}

// 发送令牌
struct TxToken<'a>(&'a mut Enc28j60<
    Spi<pac::SPI1, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>,
    PA4<Output<PushPull>>,
    enc28j60::Unconnected,
    PA3<Output<PushPull>>
>);

impl<'a> TxToken for TxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buffer = [0u8; 1518];
        let result = f(&mut buffer[..len]);
        let _ = self.0.transmit(&buffer[..len]);
        result
    }
}

// 网络应用程序
struct NetworkApplication {
    tcp_handle: tcp::Handle,
    udp_handle: udp::Handle,
    state: AppState,
    last_activity: Instant,
}

#[derive(Debug, Clone, Copy)]
enum AppState {
    Idle,
    TcpConnecting,
    TcpConnected,
    HttpRequest,
    UdpEcho,
}

impl NetworkApplication {
    fn new(tcp_handle: tcp::Handle, udp_handle: udp::Handle) -> Self {
        Self {
            tcp_handle,
            udp_handle,
            state: AppState::Idle,
            last_activity: Instant::ZERO,
        }
    }
    
    fn run(&mut self, iface: &mut Interface, sockets: &mut SocketSet, timestamp: Instant) {
        // 每5秒执行一次网络操作
        if timestamp - self.last_activity > Duration::from_secs(5) {
            self.last_activity = timestamp;
            
            match self.state {
                AppState::Idle => {
                    info!("Starting TCP connection test...");
                    self.start_tcp_connection(sockets);
                    self.state = AppState::TcpConnecting;
                }
                AppState::TcpConnecting => {
                    self.handle_tcp_connection(sockets);
                }
                AppState::TcpConnected => {
                    self.send_http_request(sockets);
                    self.state = AppState::HttpRequest;
                }
                AppState::HttpRequest => {
                    self.handle_http_response(sockets);
                    self.state = AppState::UdpEcho;
                }
                AppState::UdpEcho => {
                    self.test_udp_echo(sockets);
                    self.state = AppState::Idle;
                }
            }
        }
        
        // 处理UDP数据包
        self.handle_udp_packets(sockets);
    }
    
    fn start_tcp_connection(&mut self, sockets: &mut SocketSet) {
        let socket = sockets.get_mut::<tcp::Socket>(self.tcp_handle);
        
        // 连接到HTTP服务器 (例如: httpbin.org:80)
        let remote_addr = (Ipv4Address::new(54, 230, 97, 4), 80); // httpbin.org
        
        match socket.connect(iface.context(), remote_addr, 49152) {
            Ok(()) => {
                info!("TCP connection initiated to {}:{}", remote_addr.0, remote_addr.1);
            }
            Err(e) => {
                info!("Failed to initiate TCP connection: {:?}", e);
                self.state = AppState::Idle;
            }
        }
    }
    
    fn handle_tcp_connection(&mut self, sockets: &mut SocketSet) {
        let socket = sockets.get_mut::<tcp::Socket>(self.tcp_handle);
        
        if socket.is_active() {
            info!("TCP connection established");
            self.state = AppState::TcpConnected;
        } else if !socket.is_open() {
            info!("TCP connection failed");
            self.state = AppState::Idle;
        }
    }
    
    fn send_http_request(&mut self, sockets: &mut SocketSet) {
        let socket = sockets.get_mut::<tcp::Socket>(self.tcp_handle);
        
        if socket.can_send() {
            let request = "GET /get HTTP/1.1\r\nHost: httpbin.org\r\nConnection: close\r\n\r\n";
            
            match socket.send_slice(request.as_bytes()) {
                Ok(sent) => {
                    info!("HTTP request sent: {} bytes", sent);
                }
                Err(e) => {
                    info!("Failed to send HTTP request: {:?}", e);
                    self.state = AppState::Idle;
                }
            }
        }
    }
    
    fn handle_http_response(&mut self, sockets: &mut SocketSet) {
        let socket = sockets.get_mut::<tcp::Socket>(self.tcp_handle);
        
        if socket.can_recv() {
            let mut buffer = [0u8; 512];
            
            match socket.recv_slice(&mut buffer) {
                Ok(received) => {
                    if received > 0 {
                        info!("HTTP response received: {} bytes", received);
                        // 这里可以解析HTTP响应
                        let response = core::str::from_utf8(&buffer[..received])
                            .unwrap_or("<invalid UTF-8>");
                        info!("Response preview: {}", &response[..response.len().min(100)]);
                    }
                }
                Err(e) => {
                    info!("Failed to receive HTTP response: {:?}", e);
                }
            }
        }
        
        if !socket.is_active() {
            info!("TCP connection closed");
            socket.close();
        }
    }
    
    fn test_udp_echo(&mut self, sockets: &mut SocketSet) {
        let socket = sockets.get_mut::<udp::Socket>(self.udp_handle);
        
        if !socket.is_open() {
            // 绑定到本地端口
            match socket.bind(8080) {
                Ok(()) => {
                    info!("UDP socket bound to port 8080");
                }
                Err(e) => {
                    info!("Failed to bind UDP socket: {:?}", e);
                    return;
                }
            }
        }
        
        // 发送UDP测试数据包
        let test_data = b"Hello, UDP World!";
        let remote_addr = (Ipv4Address::new(8, 8, 8, 8), 53); // Google DNS
        
        match socket.send_slice(test_data, remote_addr.into()) {
            Ok(()) => {
                info!("UDP packet sent to {}:{}", remote_addr.0, remote_addr.1);
            }
            Err(e) => {
                info!("Failed to send UDP packet: {:?}", e);
            }
        }
    }
    
    fn handle_udp_packets(&mut self, sockets: &mut SocketSet) {
        let socket = sockets.get_mut::<udp::Socket>(self.udp_handle);
        
        if socket.can_recv() {
            let mut buffer = [0u8; 512];
            
            match socket.recv_slice(&mut buffer) {
                Ok((received, remote_addr)) => {
                    info!("UDP packet received from {}: {} bytes", remote_addr, received);
                    
                    // 回显数据包
                    let echo_data = &buffer[..received];
                    match socket.send_slice(echo_data, remote_addr) {
                        Ok(()) => {
                            info!("UDP echo sent back to {}", remote_addr);
                        }
                        Err(e) => {
                            info!("Failed to send UDP echo: {:?}", e);
                        }
                    }
                }
                Err(e) => {
                    info!("Failed to receive UDP packet: {:?}", e);
                }
            }
        }
    }
}

// HTTP客户端实用函数
struct HttpClient;

impl HttpClient {
    fn parse_response(data: &[u8]) -> Option<HttpResponse> {
        let response_str = core::str::from_utf8(data).ok()?;
        let lines: Vec<&str> = response_str.lines().collect();
        
        if lines.is_empty() {
            return None;
        }
        
        // 解析状态行
        let status_line = lines[0];
        let parts: Vec<&str> = status_line.split_whitespace().collect();
        
        if parts.len() < 3 {
            return None;
        }
        
        let status_code = parts[1].parse().ok()?;
        
        Some(HttpResponse {
            status_code,
            headers: Vec::new(), // 简化实现
            body: String::new(), // 简化实现
        })
    }
}

#[derive(Debug)]
struct HttpResponse {
    status_code: u16,
    headers: Vec<(String<64>, String<256>)>,
    body: String<1024>,
}

// 网络统计
#[derive(Debug, Default)]
struct NetworkStats {
    packets_sent: u32,
    packets_received: u32,
    bytes_sent: u32,
    bytes_received: u32,
    tcp_connections: u32,
    udp_packets: u32,
    errors: u32,
}

static mut NETWORK_STATS: NetworkStats = NetworkStats {
    packets_sent: 0,
    packets_received: 0,
    bytes_sent: 0,
    bytes_received: 0,
    tcp_connections: 0,
    udp_packets: 0,
    errors: 0,
};

// 网络监控函数
fn log_network_stats() {
    unsafe {
        info!("Network Statistics:");
        info!("  Packets: TX={}, RX={}", NETWORK_STATS.packets_sent, NETWORK_STATS.packets_received);
        info!("  Bytes: TX={}, RX={}", NETWORK_STATS.bytes_sent, NETWORK_STATS.bytes_received);
        info!("  TCP connections: {}", NETWORK_STATS.tcp_connections);
        info!("  UDP packets: {}", NETWORK_STATS.udp_packets);
        info!("  Errors: {}", NETWORK_STATS.errors);
    }
}