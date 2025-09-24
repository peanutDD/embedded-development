#![no_std]
#![no_main]

use panic_halt as _;

use rtic::app;
use stm32f4xx_hal::{
    gpio::{gpioa::PA2, gpioa::PA3, Alternate, AF7},
    pac::{USART2, TIM2},
    prelude::*,
    serial::{config::Config, Serial, Tx, Rx},
    timer::{Timer, Event},
};
use heapless::{
    spsc::{Consumer, Producer, Queue},
    String,
    Vec,
};
use serde::{Deserialize, Serialize};

// 缓冲区大小配置
const TX_BUFFER_SIZE: usize = 1024;
const RX_BUFFER_SIZE: usize = 1024;
const LOG_BUFFER_SIZE: usize = 512;
const MAX_CHANNELS: usize = 4;

// 数据包结构
#[derive(Debug, Serialize, Deserialize)]
pub struct DataPacket {
    pub timestamp: u32,
    pub channel: u8,
    pub data: Vec<u8, 64>,
    pub checksum: u16,
}

// 监控配置
#[derive(Debug, Clone)]
pub struct MonitorConfig {
    pub enabled: bool,
    pub baudrate: u32,
    pub data_bits: u8,
    pub stop_bits: u8,
    pub parity: bool,
    pub log_level: LogLevel,
    pub auto_save: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
}

// 统计信息
#[derive(Debug)]
pub struct Statistics {
    pub packets_received: u32,
    pub packets_sent: u32,
    pub bytes_received: u32,
    pub bytes_sent: u32,
    pub errors: u32,
    pub uptime_seconds: u32,
}

#[app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        config: MonitorConfig,
        statistics: Statistics,
        log_buffer: Queue<String<128>, LOG_BUFFER_SIZE>,
    }

    #[local]
    struct Local {
        uart_tx: Tx<USART2>,
        uart_rx: Rx<USART2>,
        timer: Timer<TIM2>,
        tx_producer: Producer<'static, u8, TX_BUFFER_SIZE>,
        tx_consumer: Consumer<'static, u8, TX_BUFFER_SIZE>,
        rx_producer: Producer<'static, u8, RX_BUFFER_SIZE>,
        rx_consumer: Consumer<'static, u8, RX_BUFFER_SIZE>,
        packet_buffer: Vec<u8, 256>,
        timestamp_counter: u32,
    }

    #[init(local = [
        tx_queue: Queue<u8, TX_BUFFER_SIZE> = Queue::new(),
        rx_queue: Queue<u8, RX_BUFFER_SIZE> = Queue::new(),
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;

        // 配置时钟
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

        // 配置GPIO
        let gpioa = dp.GPIOA.split();
        let tx_pin = gpioa.pa2.into_alternate::<AF7>();
        let rx_pin = gpioa.pa3.into_alternate::<AF7>();

        // 配置UART
        let serial = Serial::new(
            dp.USART2,
            (tx_pin, rx_pin),
            Config::default().baudrate(115200.bps()),
            &clocks,
        ).unwrap();

        let (mut uart_tx, mut uart_rx) = serial.split();
        uart_rx.listen();

        // 配置定时器
        let mut timer = Timer::new(dp.TIM2, &clocks);
        timer.start(1.secs());
        timer.listen(Event::Update);

        // 初始化队列
        let (tx_producer, tx_consumer) = ctx.local.tx_queue.split();
        let (rx_producer, rx_consumer) = ctx.local.rx_queue.split();

        // 发送启动消息
        let startup_msg = "Serial Monitor Started\r\n";
        for byte in startup_msg.bytes() {
            tx_producer.enqueue(byte).ok();
        }

        // 启动发送
        if let Some(byte) = tx_consumer.dequeue() {
            uart_tx.write(byte).ok();
            uart_tx.listen();
        }

        (
            Shared {
                config: MonitorConfig {
                    enabled: true,
                    baudrate: 115200,
                    data_bits: 8,
                    stop_bits: 1,
                    parity: false,
                    log_level: LogLevel::Info,
                    auto_save: false,
                },
                statistics: Statistics {
                    packets_received: 0,
                    packets_sent: 0,
                    bytes_received: 0,
                    bytes_sent: 0,
                    errors: 0,
                    uptime_seconds: 0,
                },
                log_buffer: Queue::new(),
            },
            Local {
                uart_tx,
                uart_rx,
                timer,
                tx_producer,
                tx_consumer,
                rx_producer,
                rx_consumer,
                packet_buffer: Vec::new(),
                timestamp_counter: 0,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = USART2, local = [uart_tx, uart_rx, tx_consumer, rx_producer], shared = [statistics])]
    fn uart_handler(mut ctx: uart_handler::Context) {
        let uart_tx = ctx.local.uart_tx;
        let uart_rx = ctx.local.uart_rx;
        let tx_consumer = ctx.local.tx_consumer;
        let rx_producer = ctx.local.rx_producer;

        // 处理接收中断
        if uart_rx.is_rxne() {
            if let Ok(byte) = uart_rx.read() {
                if rx_producer.enqueue(byte).is_ok() {
                    ctx.shared.statistics.lock(|stats| {
                        stats.bytes_received += 1;
                    });
                    
                    // 触发数据处理任务
                    data_processor::spawn().ok();
                } else {
                    ctx.shared.statistics.lock(|stats| {
                        stats.errors += 1;
                    });
                }
            }
        }

        // 处理发送中断
        if uart_tx.is_txe() {
            if let Some(byte) = tx_consumer.dequeue() {
                uart_tx.write(byte).ok();
                ctx.shared.statistics.lock(|stats| {
                    stats.bytes_sent += 1;
                });
            } else {
                uart_tx.unlisten();
            }
        }
    }

    #[task(local = [rx_consumer, packet_buffer, timestamp_counter], shared = [statistics, log_buffer, config])]
    fn data_processor(mut ctx: data_processor::Context) {
        let rx_consumer = ctx.local.rx_consumer;
        let packet_buffer = ctx.local.packet_buffer;
        let timestamp_counter = ctx.local.timestamp_counter;

        // 处理接收到的数据
        while let Some(byte) = rx_consumer.dequeue() {
            packet_buffer.push(byte).ok();

            // 检查是否为完整数据包（简单的换行符分隔）
            if byte == b'\n' || packet_buffer.len() >= 255 {
                if !packet_buffer.is_empty() {
                    // 创建数据包
                    let packet = DataPacket {
                        timestamp: *timestamp_counter,
                        channel: 0,
                        data: packet_buffer.clone(),
                        checksum: calculate_checksum(packet_buffer),
                    };

                    // 处理数据包
                    process_packet(&packet, &mut ctx.shared);

                    // 清空缓冲区
                    packet_buffer.clear();
                }
            }
        }
    }

    #[task(binds = TIM2, local = [timer, timestamp_counter], shared = [statistics, config])]
    fn timer_handler(mut ctx: timer_handler::Context) {
        ctx.local.timer.clear_interrupt(Event::Update);
        *ctx.local.timestamp_counter += 1;

        ctx.shared.statistics.lock(|stats| {
            stats.uptime_seconds += 1;
        });

        // 定期任务
        if *ctx.local.timestamp_counter % 10 == 0 {
            status_reporter::spawn().ok();
        }

        if *ctx.local.timestamp_counter % 60 == 0 {
            log_manager::spawn().ok();
        }
    }

    #[task(local = [tx_producer], shared = [statistics, config])]
    fn status_reporter(mut ctx: status_reporter::Context) {
        let tx_producer = ctx.local.tx_producer;

        ctx.shared.statistics.lock(|stats| {
            ctx.shared.config.lock(|config| {
                if config.enabled && config.log_level as u8 <= LogLevel::Info as u8 {
                    let mut status_msg: String<256> = String::new();
                    use core::fmt::Write;
                    
                    write!(status_msg, 
                           "\r\n[STATUS] RX: {}, TX: {}, Errors: {}, Uptime: {}s\r\n",
                           stats.bytes_received,
                           stats.bytes_sent,
                           stats.errors,
                           stats.uptime_seconds).ok();

                    // 发送状态消息
                    for byte in status_msg.bytes() {
                        if tx_producer.enqueue(byte).is_err() {
                            break;
                        }
                    }

                    // 启动发送
                    uart_send_trigger::spawn().ok();
                }
            });
        });
    }

    #[task(local = [uart_tx, tx_consumer])]
    fn uart_send_trigger(ctx: uart_send_trigger::Context) {
        let uart_tx = ctx.local.uart_tx;
        let tx_consumer = ctx.local.tx_consumer;

        if let Some(byte) = tx_consumer.dequeue() {
            uart_tx.write(byte).ok();
            uart_tx.listen();
        }
    }

    #[task(shared = [log_buffer, config])]
    fn log_manager(mut ctx: log_manager::Context) {
        ctx.shared.log_buffer.lock(|log_buffer| {
            ctx.shared.config.lock(|config| {
                if config.auto_save {
                    // 这里可以实现日志保存到存储器的逻辑
                    // 由于是嵌入式环境，可能需要使用外部存储器或EEPROM
                    
                    // 清理旧日志
                    while log_buffer.len() > LOG_BUFFER_SIZE / 2 {
                        log_buffer.dequeue();
                    }
                }
            });
        });
    }

    #[task(local = [tx_producer], shared = [config])]
    fn command_processor(mut ctx: command_processor::Context, command: String<64>) {
        let tx_producer = ctx.local.tx_producer;

        ctx.shared.config.lock(|config| {
            let response = process_command(&command, config);
            
            // 发送响应
            for byte in response.bytes() {
                if tx_producer.enqueue(byte).is_err() {
                    break;
                }
            }
            
            uart_send_trigger::spawn().ok();
        });
    }
}

fn process_packet(packet: &DataPacket, shared: &mut app::Shared) {
    // 验证数据包
    let calculated_checksum = calculate_checksum(&packet.data);
    if calculated_checksum != packet.checksum {
        shared.statistics.errors += 1;
        return;
    }

    shared.statistics.packets_received += 1;

    // 根据配置处理数据包
    if shared.config.enabled {
        // 创建日志条目
        let mut log_entry: String<128> = String::new();
        use core::fmt::Write;
        
        write!(log_entry, 
               "[{}] CH{}: {} bytes",
               packet.timestamp,
               packet.channel,
               packet.data.len()).ok();

        // 添加到日志缓冲区
        shared.log_buffer.enqueue(log_entry).ok();

        // 如果是调试模式，显示详细信息
        if shared.config.log_level == LogLevel::Debug {
            // 可以在这里添加更详细的数据包分析
        }
    }
}

fn calculate_checksum(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |acc, &byte| {
        acc.wrapping_add(byte as u16)
    })
}

fn process_command(command: &str, config: &mut MonitorConfig) -> String<256> {
    let mut response: String<256> = String::new();
    use core::fmt::Write;

    let parts: Vec<&str, 8> = command.trim().split_whitespace().collect();
    if parts.is_empty() {
        return response;
    }

    match parts[0] {
        "help" => {
            write!(response, 
                   "Commands: help, status, enable, disable, baudrate <rate>, loglevel <level>\r\n").ok();
        }
        "status" => {
            write!(response,
                   "Monitor: {}, Baudrate: {}, LogLevel: {:?}\r\n",
                   if config.enabled { "ON" } else { "OFF" },
                   config.baudrate,
                   config.log_level).ok();
        }
        "enable" => {
            config.enabled = true;
            write!(response, "Monitor enabled\r\n").ok();
        }
        "disable" => {
            config.enabled = false;
            write!(response, "Monitor disabled\r\n").ok();
        }
        "baudrate" => {
            if parts.len() > 1 {
                if let Ok(rate) = parts[1].parse::<u32>() {
                    config.baudrate = rate;
                    write!(response, "Baudrate set to {}\r\n", rate).ok();
                } else {
                    write!(response, "Invalid baudrate\r\n").ok();
                }
            } else {
                write!(response, "Current baudrate: {}\r\n", config.baudrate).ok();
            }
        }
        "loglevel" => {
            if parts.len() > 1 {
                match parts[1] {
                    "debug" => {
                        config.log_level = LogLevel::Debug;
                        write!(response, "Log level set to Debug\r\n").ok();
                    }
                    "info" => {
                        config.log_level = LogLevel::Info;
                        write!(response, "Log level set to Info\r\n").ok();
                    }
                    "warning" => {
                        config.log_level = LogLevel::Warning;
                        write!(response, "Log level set to Warning\r\n").ok();
                    }
                    "error" => {
                        config.log_level = LogLevel::Error;
                        write!(response, "Log level set to Error\r\n").ok();
                    }
                    _ => {
                        write!(response, "Invalid log level\r\n").ok();
                    }
                }
            } else {
                write!(response, "Current log level: {:?}\r\n", config.log_level).ok();
            }
        }
        _ => {
            write!(response, "Unknown command: {}\r\n", parts[0]).ok();
        }
    }

    response
}