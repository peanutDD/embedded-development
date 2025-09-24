#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{gpioa::PA2, gpioa::PA3, Alternate, AF7},
    interrupt,
    pac::{self, USART2},
    prelude::*,
    serial::{config::Config, Serial, Tx, Rx},
};
use heapless::{
    spsc::{Consumer, Producer, Queue},
    String,
    Vec,
};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

// 缓冲区大小配置
const TX_BUFFER_SIZE: usize = 256;
const RX_BUFFER_SIZE: usize = 256;
const LINE_BUFFER_SIZE: usize = 128;

// 全局变量
static mut TX_QUEUE: Queue<u8, TX_BUFFER_SIZE> = Queue::new();
static mut RX_QUEUE: Queue<u8, RX_BUFFER_SIZE> = Queue::new();
static mut TX_PRODUCER: Option<Producer<u8, TX_BUFFER_SIZE>> = None;
static mut TX_CONSUMER: Option<Consumer<u8, TX_BUFFER_SIZE>> = None;
static mut RX_PRODUCER: Option<Producer<u8, RX_BUFFER_SIZE>> = None;
static mut RX_CONSUMER: Option<Consumer<u8, RX_BUFFER_SIZE>> = None;

static mut UART_TX: Option<Tx<USART2>> = None;
static mut UART_RX: Option<Rx<USART2>> = None;

static TX_READY: AtomicBool = AtomicBool::new(true);
static RX_OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);
static TX_OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);

// 统计信息
#[derive(Debug)]
struct UartStats {
    bytes_sent: u32,
    bytes_received: u32,
    lines_processed: u32,
    tx_overflows: u32,
    rx_overflows: u32,
}

impl UartStats {
    fn new() -> Self {
        Self {
            bytes_sent: 0,
            bytes_received: 0,
            lines_processed: 0,
            tx_overflows: 0,
            rx_overflows: 0,
        }
    }
}

static mut STATS: UartStats = UartStats {
    bytes_sent: 0,
    bytes_received: 0,
    lines_processed: 0,
    tx_overflows: 0,
    rx_overflows: 0,
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

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

    let (tx, rx) = serial.split();

    // 初始化队列
    let (tx_prod, tx_cons) = unsafe { TX_QUEUE.split() };
    let (rx_prod, rx_cons) = unsafe { RX_QUEUE.split() };

    unsafe {
        TX_PRODUCER = Some(tx_prod);
        TX_CONSUMER = Some(tx_cons);
        RX_PRODUCER = Some(rx_prod);
        RX_CONSUMER = Some(rx_cons);
        UART_TX = Some(tx);
        UART_RX = Some(rx);
    }

    // 启用UART中断
    unsafe {
        if let Some(ref mut uart_rx) = UART_RX {
            uart_rx.listen();
        }
    }

    // 启用NVIC中断
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::USART2);
    }

    // 发送欢迎消息
    send_string("Buffered UART Example Started\r\n");
    send_string("Type commands and press Enter\r\n");
    send_string("Available commands: help, stats, echo <text>\r\n");

    let mut line_buffer: Vec<u8, LINE_BUFFER_SIZE> = Vec::new();
    let mut stats_counter = 0u32;

    loop {
        // 处理接收到的数据
        while let Some(byte) = receive_byte() {
            unsafe {
                STATS.bytes_received += 1;
            }

            match byte {
                b'\r' | b'\n' => {
                    if !line_buffer.is_empty() {
                        process_command(&line_buffer);
                        line_buffer.clear();
                        unsafe {
                            STATS.lines_processed += 1;
                        }
                    }
                    send_string("> ");
                }
                b'\x08' | b'\x7f' => {
                    // 退格键处理
                    if !line_buffer.is_empty() {
                        line_buffer.pop();
                        send_string("\x08 \x08");
                    }
                }
                0x20..=0x7E => {
                    // 可打印字符
                    if line_buffer.len() < LINE_BUFFER_SIZE - 1 {
                        line_buffer.push(byte).ok();
                        send_byte(byte); // 回显
                    } else {
                        send_string("\r\nLine too long!\r\n> ");
                        line_buffer.clear();
                    }
                }
                _ => {
                    // 忽略其他控制字符
                }
            }
        }

        // 定期显示统计信息
        stats_counter += 1;
        if stats_counter >= 5_000_000 {
            stats_counter = 0;
            show_buffer_status();
        }

        cortex_m::asm::nop();
    }
}

fn process_command(command: &[u8]) {
    let cmd_str = core::str::from_utf8(command).unwrap_or("");
    let parts: Vec<&str, 8> = cmd_str.split_whitespace().collect();

    if parts.is_empty() {
        return;
    }

    match parts[0] {
        "help" => {
            send_string("\r\nAvailable commands:\r\n");
            send_string("  help - Show this help\r\n");
            send_string("  stats - Show communication statistics\r\n");
            send_string("  echo <text> - Echo back the text\r\n");
            send_string("  clear - Clear statistics\r\n");
            send_string("  buffer - Show buffer status\r\n");
        }
        "stats" => {
            show_stats();
        }
        "echo" => {
            if parts.len() > 1 {
                send_string("\r\nEcho: ");
                for (i, part) in parts.iter().skip(1).enumerate() {
                    if i > 0 {
                        send_string(" ");
                    }
                    send_string(part);
                }
                send_string("\r\n");
            } else {
                send_string("\r\nEcho: (empty)\r\n");
            }
        }
        "clear" => {
            unsafe {
                STATS = UartStats::new();
            }
            RX_OVERFLOW_COUNT.store(0, Ordering::Relaxed);
            TX_OVERFLOW_COUNT.store(0, Ordering::Relaxed);
            send_string("\r\nStatistics cleared\r\n");
        }
        "buffer" => {
            show_buffer_status();
        }
        _ => {
            send_string("\r\nUnknown command: ");
            send_string(cmd_str);
            send_string("\r\nType 'help' for available commands\r\n");
        }
    }
}

fn show_stats() {
    unsafe {
        let mut stats_str: String<256> = String::new();
        
        use core::fmt::Write;
        write!(stats_str, "\r\nCommunication Statistics:\r\n").ok();
        write!(stats_str, "  Bytes sent: {}\r\n", STATS.bytes_sent).ok();
        write!(stats_str, "  Bytes received: {}\r\n", STATS.bytes_received).ok();
        write!(stats_str, "  Lines processed: {}\r\n", STATS.lines_processed).ok();
        write!(stats_str, "  TX overflows: {}\r\n", TX_OVERFLOW_COUNT.load(Ordering::Relaxed)).ok();
        write!(stats_str, "  RX overflows: {}\r\n", RX_OVERFLOW_COUNT.load(Ordering::Relaxed)).ok();
        
        send_string(&stats_str);
    }
}

fn show_buffer_status() {
    unsafe {
        if let (Some(ref tx_prod), Some(ref rx_prod)) = 
            (TX_PRODUCER.as_ref(), RX_PRODUCER.as_ref()) {
            
            let tx_free = tx_prod.capacity() - tx_prod.len();
            let rx_free = rx_prod.capacity() - rx_prod.len();
            
            let mut status_str: String<128> = String::new();
            use core::fmt::Write;
            write!(status_str, "\r\nBuffer Status - TX: {}/{}, RX: {}/{}\r\n", 
                   tx_prod.len(), tx_prod.capacity(),
                   rx_prod.len(), rx_prod.capacity()).ok();
            
            send_string(&status_str);
        }
    }
}

fn send_byte(byte: u8) -> bool {
    unsafe {
        if let Some(ref mut producer) = TX_PRODUCER {
            match producer.enqueue(byte) {
                Ok(()) => {
                    STATS.bytes_sent += 1;
                    // 如果TX空闲，立即开始发送
                    if TX_READY.load(Ordering::Relaxed) {
                        start_transmission();
                    }
                    true
                }
                Err(_) => {
                    TX_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
                    false
                }
            }
        } else {
            false
        }
    }
}

fn send_string(s: &str) -> usize {
    let mut sent_count = 0;
    for byte in s.bytes() {
        if send_byte(byte) {
            sent_count += 1;
        } else {
            break; // 缓冲区满，停止发送
        }
    }
    sent_count
}

fn receive_byte() -> Option<u8> {
    unsafe {
        if let Some(ref mut consumer) = RX_CONSUMER {
            consumer.dequeue()
        } else {
            None
        }
    }
}

fn start_transmission() {
    unsafe {
        if let (Some(ref mut consumer), Some(ref mut uart_tx)) = 
            (TX_CONSUMER.as_mut(), UART_TX.as_mut()) {
            if let Some(byte) = consumer.dequeue() {
                TX_READY.store(false, Ordering::Relaxed);
                uart_tx.write(byte).ok();
                uart_tx.listen();
            }
        }
    }
}

#[interrupt]
fn USART2() {
    unsafe {
        if let Some(ref mut uart_rx) = UART_RX {
            // 处理接收中断
            if uart_rx.is_rxne() {
                if let Ok(byte) = uart_rx.read() {
                    if let Some(ref mut producer) = RX_PRODUCER {
                        if producer.enqueue(byte).is_err() {
                            RX_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }
        }

        if let Some(ref mut uart_tx) = UART_TX {
            // 处理发送中断
            if uart_tx.is_txe() {
                if let Some(ref mut consumer) = TX_CONSUMER {
                    if let Some(byte) = consumer.dequeue() {
                        uart_tx.write(byte).ok();
                    } else {
                        // 没有更多数据要发送，停止发送中断
                        uart_tx.unlisten();
                        TX_READY.store(true, Ordering::Relaxed);
                    }
                }
            }
        }
    }
}