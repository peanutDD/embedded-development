#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm;
use rtic::app;
use stm32f4xx_hal::{
    gpio::{gpioa::*, gpiob::*, gpioc::*, Input, Output, PullUp, PushPull, Alternate},
    pac,
    prelude::*,
    spi::{Spi, NoMiso, NoMosi},
    timer::{CounterUs, Event},
    i2c::I2c,
    adc::{Adc, config::AdcConfig},
};
use dwt_systick_monotonic::{DwtSystick, ExtU32};
use heapless::{
    pool::{Pool, Node},
    spsc::{Queue, Producer, Consumer},
    Vec, String,
};
use rtt_target::{rprintln, rtt_init_print};
use serde::{Serialize, Deserialize};
use serde_json_core;

// 网络相关
use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::{tcp, udp},
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
    time::Instant,
};
use enc28j60::{Enc28j60, Unconnected};

// 加密相关
use aes::Aes128;
use aes::cipher::{BlockEncrypt, KeyInit, generic_array::GenericArray};
use sha2::{Sha256, Digest};
use hmac::{Hmac, Mac};

// 常量定义
const SENSOR_COUNT: usize = 8;
const DATA_BUFFER_SIZE: usize = 64;
const NETWORK_BUFFER_SIZE: usize = 1024;
const CLOUD_SERVER_IP: [u8; 4] = [192, 168, 1, 100];
const CLOUD_SERVER_PORT: u16 = 8080;

// 传感器数据结构
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SensorReading {
    pub sensor_id: u8,
    pub sensor_type: SensorType,
    pub value: f32,
    pub timestamp: u32,
    pub quality: u8, // 数据质量指标 0-100
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum SensorType {
    Temperature,
    Humidity,
    Pressure,
    Light,
    Motion,
    Gas,
    Voltage,
    Current,
}

// IoT数据包结构
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IoTDataPacket {
    pub device_id: String<32>,
    pub timestamp: u32,
    pub sensors: Vec<SensorReading, SENSOR_COUNT>,
    pub status: DeviceStatus,
    pub checksum: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DeviceStatus {
    pub battery_level: u8,
    pub signal_strength: i8,
    pub temperature: f32,
    pub uptime: u32,
    pub error_count: u16,
}

// 网关配置
#[derive(Debug, Clone)]
pub struct GatewayConfig {
    pub device_id: String<32>,
    pub server_address: IpAddress,
    pub server_port: u16,
    pub encryption_key: [u8; 16],
    pub sampling_interval: u32, // ms
    pub transmission_interval: u32, // ms
}

impl Default for GatewayConfig {
    fn default() -> Self {
        let mut device_id = String::new();
        device_id.push_str("IOT_GW_001").ok();
        
        Self {
            device_id,
            server_address: IpAddress::Ipv4(Ipv4Address::new(192, 168, 1, 100)),
            server_port: 8080,
            encryption_key: [0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                           0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c],
            sampling_interval: 1000,    // 1秒采样
            transmission_interval: 5000, // 5秒传输
        }
    }
}

// 网络统计
#[derive(Debug, Default)]
pub struct NetworkStats {
    pub packets_sent: u32,
    pub packets_received: u32,
    pub bytes_sent: u32,
    pub bytes_received: u32,
    pub connection_errors: u16,
    pub transmission_errors: u16,
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<84_000_000>; // 84 MHz

    #[shared]
    struct Shared {
        gateway_config: GatewayConfig,
        sensor_data: Vec<SensorReading, SENSOR_COUNT>,
        network_stats: NetworkStats,
        device_status: DeviceStatus,
        data_queue: Queue<IoTDataPacket, 8>,
    }

    #[local]
    struct Local {
        // 硬件外设
        led_status: PA5<Output<PushPull>>,
        led_network: PA6<Output<PushPull>>,
        button: PC13<Input<PullUp>>,
        
        // 传感器接口
        adc: Adc<pac::ADC1>,
        i2c: I2c<pac::I2C1>,
        
        // 网络接口
        ethernet: Enc28j60<Spi<pac::SPI1, (PA5<Alternate<5>>, NoMiso, PA7<Alternate<5>>)>, PA4<Output<PushPull>>>,
        network_interface: Interface,
        sockets: SocketSet<'static>,
        
        // 数据处理
        data_producer: Producer<'static, IoTDataPacket, 8>,
        data_consumer: Consumer<'static, IoTDataPacket, 8>,
        
        // 定时器
        sampling_timer: CounterUs<pac::TIM2>,
        transmission_timer: CounterUs<pac::TIM3>,
        
        // 缓冲区
        network_buffer: [u8; NETWORK_BUFFER_SIZE],
    }

    #[init(local = [
        data_queue: Queue<IoTDataPacket, 8> = Queue::new(),
        socket_storage: [tcp::SocketStorage<'static>; 2] = [tcp::SocketStorage::EMPTY; 2],
        network_buffer: [u8; NETWORK_BUFFER_SIZE] = [0; NETWORK_BUFFER_SIZE]
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("IoT Gateway Starting...");

        let dp = ctx.device;
        let cp = ctx.core;

        // 时钟配置
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(84.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .freeze();

        // GPIO配置
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        
        let led_status = gpioa.pa5.into_push_pull_output();
        let led_network = gpioa.pa6.into_push_pull_output();
        let button = gpioc.pc13.into_pull_up_input();

        // SPI配置 (用于以太网)
        let sck = gpioa.pa5.into_alternate();
        let mosi = gpioa.pa7.into_alternate();
        let cs = gpioa.pa4.into_push_pull_output();
        
        let spi = Spi::new(
            dp.SPI1,
            (sck, NoMiso, mosi),
            enc28j60::MODE,
            1.MHz(),
            &clocks,
        );

        // 以太网控制器初始化
        let mut ethernet = Enc28j60::new(spi, cs, Unconnected, Unconnected, &mut ctx.local.network_buffer[..])
            .expect("Failed to initialize ENC28J60");
        
        // 设置MAC地址
        let mac_addr = EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
        
        // 网络接口配置
        let config = Config::new(mac_addr.into());
        let mut network_interface = Interface::new(config, &mut ethernet, Instant::ZERO);
        
        // IP配置
        network_interface.update_ip_addrs(|ip_addrs| {
            ip_addrs.push(IpCidr::new(IpAddress::v4(192, 168, 1, 50), 24)).unwrap();
        });

        // Socket配置
        let mut sockets = SocketSet::new(&mut ctx.local.socket_storage[..]);
        
        // TCP socket for cloud communication
        let tcp_rx_buffer = tcp::SocketBuffer::new(&mut [0; 1024][..]);
        let tcp_tx_buffer = tcp::SocketBuffer::new(&mut [0; 1024][..]);
        let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);
        sockets.add(tcp_socket);

        // I2C配置 (用于传感器)
        let scl = gpiob.pb8.into_alternate_open_drain();
        let sda = gpiob.pb9.into_alternate_open_drain();
        let i2c = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

        // ADC配置 (用于模拟传感器)
        let adc_config = AdcConfig::default();
        let adc = Adc::adc1(dp.ADC1, true, adc_config);

        // 定时器配置
        let mut sampling_timer = dp.TIM2.counter_us(&clocks);
        sampling_timer.start(1.secs()).unwrap();
        sampling_timer.listen(Event::Update);

        let mut transmission_timer = dp.TIM3.counter_us(&clocks);
        transmission_timer.start(5.secs()).unwrap();
        transmission_timer.listen(Event::Update);

        // 单调时钟初始化
        let mono = DwtSystick::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.hclk().raw());

        // 数据队列初始化
        let (data_producer, data_consumer) = ctx.local.data_queue.split();

        // 启动任务
        sensor_sampling_task::spawn().ok();
        data_processing_task::spawn().ok();
        network_communication_task::spawn().ok();
        system_monitoring_task::spawn().ok();

        rprintln!("IoT Gateway initialized successfully");

        (
            Shared {
                gateway_config: GatewayConfig::default(),
                sensor_data: Vec::new(),
                network_stats: NetworkStats::default(),
                device_status: DeviceStatus {
                    battery_level: 100,
                    signal_strength: -50,
                    temperature: 25.0,
                    uptime: 0,
                    error_count: 0,
                },
                data_queue: Queue::new(),
            },
            Local {
                led_status,
                led_network,
                button,
                adc,
                i2c,
                ethernet,
                network_interface,
                sockets,
                data_producer,
                data_consumer,
                sampling_timer,
                transmission_timer,
                network_buffer: [0; NETWORK_BUFFER_SIZE],
            },
            init::Monotonics(mono),
        )
    }

    // 传感器采样任务 - 高优先级
    #[task(shared = [sensor_data, device_status], local = [adc, i2c], priority = 3)]
    fn sensor_sampling_task(mut ctx: sensor_sampling_task::Context) {
        let timestamp = monotonics::MyMono::now().ticks();
        
        // 模拟传感器读取
        let mut readings = Vec::new();
        
        // 温度传感器 (I2C)
        let temperature = read_temperature_sensor(&mut ctx.local.i2c);
        readings.push(SensorReading {
            sensor_id: 1,
            sensor_type: SensorType::Temperature,
            value: temperature,
            timestamp,
            quality: 95,
        }).ok();

        // 湿度传感器 (I2C)
        let humidity = read_humidity_sensor(&mut ctx.local.i2c);
        readings.push(SensorReading {
            sensor_id: 2,
            sensor_type: SensorType::Humidity,
            value: humidity,
            timestamp,
            quality: 90,
        }).ok();

        // 光照传感器 (ADC)
        let light = read_light_sensor(&mut ctx.local.adc);
        readings.push(SensorReading {
            sensor_id: 3,
            sensor_type: SensorType::Light,
            value: light,
            timestamp,
            quality: 85,
        }).ok();

        // 电压传感器 (ADC)
        let voltage = read_voltage_sensor(&mut ctx.local.adc);
        readings.push(SensorReading {
            sensor_id: 4,
            sensor_type: SensorType::Voltage,
            value: voltage,
            timestamp,
            quality: 98,
        }).ok();

        // 更新共享数据
        ctx.shared.sensor_data.lock(|data| {
            data.clear();
            for reading in readings {
                data.push(reading).ok();
            }
        });

        // 更新设备状态
        ctx.shared.device_status.lock(|status| {
            status.uptime += 1;
            status.temperature = temperature;
        });

        rprintln!("Sensor sampling completed: {} readings", readings.len());

        // 重新调度
        sensor_sampling_task::spawn_after(1.secs()).ok();
    }

    // 数据处理任务 - 中优先级
    #[task(shared = [sensor_data, gateway_config, device_status], local = [data_producer], priority = 2)]
    fn data_processing_task(mut ctx: data_processing_task::Context) {
        let (sensor_data, config, status) = ctx.shared.lock(|data, cfg, stat| {
            (data.clone(), cfg.clone(), *stat)
        });

        if !sensor_data.is_empty() {
            // 创建IoT数据包
            let mut packet = IoTDataPacket {
                device_id: config.device_id.clone(),
                timestamp: monotonics::MyMono::now().ticks(),
                sensors: sensor_data,
                status,
                checksum: 0,
            };

            // 计算校验和
            packet.checksum = calculate_checksum(&packet);

            // 加密数据包 (可选)
            if let Ok(encrypted_packet) = encrypt_packet(&packet, &config.encryption_key) {
                // 发送到网络队列
                if ctx.local.data_producer.enqueue(encrypted_packet).is_err() {
                    rprintln!("Warning: Data queue full, dropping packet");
                }
            }
        }

        // 重新调度
        data_processing_task::spawn_after(5.secs()).ok();
    }

    // 网络通信任务 - 中优先级
    #[task(shared = [gateway_config, network_stats], local = [data_consumer, network_interface, sockets, ethernet, led_network], priority = 2)]
    fn network_communication_task(mut ctx: network_communication_task::Context) {
        // 处理网络接口
        let timestamp = Instant::from_millis(monotonics::MyMono::now().ticks() as i64);
        
        match ctx.local.network_interface.poll(timestamp, ctx.local.ethernet, ctx.local.sockets) {
            Ok(_) => {
                ctx.local.led_network.set_high(); // 网络正常
            }
            Err(e) => {
                ctx.local.led_network.set_low(); // 网络异常
                rprintln!("Network error: {:?}", e);
                
                ctx.shared.network_stats.lock(|stats| {
                    stats.connection_errors += 1;
                });
            }
        }

        // 处理待发送的数据包
        while let Some(packet) = ctx.local.data_consumer.dequeue() {
            if let Ok(json_data) = serialize_packet(&packet) {
                // 发送到云服务器
                if send_to_cloud(&json_data, ctx.local.sockets, &mut ctx.shared.gateway_config, &mut ctx.shared.network_stats) {
                    rprintln!("Data packet sent successfully");
                } else {
                    rprintln!("Failed to send data packet");
                }
            }
        }

        // 重新调度
        network_communication_task::spawn_after(100.millis()).ok();
    }

    // 系统监控任务 - 低优先级
    #[task(shared = [device_status, network_stats], local = [led_status], priority = 1)]
    fn system_monitoring_task(mut ctx: system_monitoring_task::Context) {
        // 系统状态LED指示
        ctx.local.led_status.toggle();

        // 打印系统统计信息
        let (status, stats) = ctx.shared.lock(|dev_status, net_stats| {
            (*dev_status, *net_stats)
        });

        rprintln!("=== System Status ===");
        rprintln!("Uptime: {} s, Battery: {}%, Temp: {:.1}°C", 
                 status.uptime, status.battery_level, status.temperature);
        rprintln!("Network - Sent: {} packets, Received: {} packets", 
                 stats.packets_sent, stats.packets_received);
        rprintln!("Errors - Connection: {}, Transmission: {}", 
                 stats.connection_errors, stats.transmission_errors);

        // 重新调度
        system_monitoring_task::spawn_after(10.secs()).ok();
    }

    // 按钮中断处理
    #[task(binds = EXTI15_10, shared = [gateway_config], local = [button], priority = 3)]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        if ctx.local.button.is_low() {
            rprintln!("Button pressed - triggering manual data transmission");
            
            // 触发立即数据传输
            data_processing_task::spawn().ok();
            
            ctx.shared.gateway_config.lock(|config| {
                rprintln!("Current config - Device ID: {}, Server: {:?}:{}", 
                         config.device_id, config.server_address, config.server_port);
            });
        }
    }

    // 采样定时器中断
    #[task(binds = TIM2, local = [sampling_timer], priority = 2)]
    fn sampling_timer_interrupt(ctx: sampling_timer_interrupt::Context) {
        ctx.local.sampling_timer.clear_interrupt(Event::Update);
        sensor_sampling_task::spawn().ok();
    }

    // 传输定时器中断
    #[task(binds = TIM3, local = [transmission_timer], priority = 2)]
    fn transmission_timer_interrupt(ctx: transmission_timer_interrupt::Context) {
        ctx.local.transmission_timer.clear_interrupt(Event::Update);
        data_processing_task::spawn().ok();
    }

    // 空闲任务
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::wfi(); // 等待中断，节省功耗
        }
    }
}

// 传感器读取函数
fn read_temperature_sensor(i2c: &mut I2c<pac::I2C1>) -> f32 {
    // 模拟温度传感器读取 (实际应用中会读取真实传感器)
    static mut TEMP_COUNTER: u16 = 0;
    unsafe {
        TEMP_COUNTER = TEMP_COUNTER.wrapping_add(1);
        20.0 + (TEMP_COUNTER as f32 * 0.1) % 10.0
    }
}

fn read_humidity_sensor(i2c: &mut I2c<pac::I2C1>) -> f32 {
    // 模拟湿度传感器读取
    static mut HUM_COUNTER: u16 = 0;
    unsafe {
        HUM_COUNTER = HUM_COUNTER.wrapping_add(1);
        40.0 + (HUM_COUNTER as f32 * 0.2) % 20.0
    }
}

fn read_light_sensor(adc: &mut Adc<pac::ADC1>) -> f32 {
    // 模拟光照传感器读取
    static mut LIGHT_COUNTER: u16 = 0;
    unsafe {
        LIGHT_COUNTER = LIGHT_COUNTER.wrapping_add(1);
        (LIGHT_COUNTER % 1000) as f32
    }
}

fn read_voltage_sensor(adc: &mut Adc<pac::ADC1>) -> f32 {
    // 模拟电压传感器读取
    static mut VOLT_COUNTER: u16 = 0;
    unsafe {
        VOLT_COUNTER = VOLT_COUNTER.wrapping_add(1);
        3.3 + (VOLT_COUNTER as f32 * 0.001) % 0.5
    }
}

// 数据处理函数
fn calculate_checksum(packet: &IoTDataPacket) -> u32 {
    // 简单的校验和计算
    let mut checksum = 0u32;
    checksum = checksum.wrapping_add(packet.timestamp);
    for sensor in &packet.sensors {
        checksum = checksum.wrapping_add(sensor.sensor_id as u32);
        checksum = checksum.wrapping_add(sensor.value as u32);
    }
    checksum
}

fn encrypt_packet(packet: &IoTDataPacket, key: &[u8; 16]) -> Result<IoTDataPacket, ()> {
    // 简化的加密实现 (实际应用中需要更复杂的加密)
    // 这里只是演示，实际应该加密整个数据包
    Ok(packet.clone())
}

fn serialize_packet(packet: &IoTDataPacket) -> Result<String<512>, ()> {
    let mut buffer = [0u8; 512];
    match serde_json_core::to_slice(packet, &mut buffer) {
        Ok((_, len)) => {
            let json_str = core::str::from_utf8(&buffer[..len]).map_err(|_| ())?;
            let mut result = String::new();
            result.push_str(json_str).map_err(|_| ())?;
            Ok(result)
        }
        Err(_) => Err(()),
    }
}

fn send_to_cloud(
    data: &str,
    sockets: &mut SocketSet,
    config: &mut GatewayConfig,
    stats: &mut NetworkStats,
) -> bool {
    // 简化的云端发送实现
    // 实际应用中需要建立TCP连接并发送数据
    stats.packets_sent += 1;
    stats.bytes_sent += data.len() as u32;
    
    rprintln!("Sending to cloud: {} bytes", data.len());
    true // 模拟发送成功
}

// 单元测试
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_reading_creation() {
        let reading = SensorReading {
            sensor_id: 1,
            sensor_type: SensorType::Temperature,
            value: 25.5,
            timestamp: 1000,
            quality: 95,
        };
        
        assert_eq!(reading.sensor_id, 1);
        assert_eq!(reading.value, 25.5);
        assert_eq!(reading.quality, 95);
    }

    #[test]
    fn test_gateway_config_default() {
        let config = GatewayConfig::default();
        assert_eq!(config.device_id.as_str(), "IOT_GW_001");
        assert_eq!(config.server_port, 8080);
        assert_eq!(config.sampling_interval, 1000);
    }

    #[test]
    fn test_checksum_calculation() {
        let mut packet = IoTDataPacket {
            device_id: String::new(),
            timestamp: 1000,
            sensors: Vec::new(),
            status: DeviceStatus {
                battery_level: 100,
                signal_strength: -50,
                temperature: 25.0,
                uptime: 0,
                error_count: 0,
            },
            checksum: 0,
        };
        
        let checksum = calculate_checksum(&packet);
        assert_eq!(checksum, 1000); // 只有timestamp贡献校验和
    }
}