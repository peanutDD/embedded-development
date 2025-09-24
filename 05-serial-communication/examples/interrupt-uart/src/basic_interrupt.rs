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
use heapless::spsc::{Consumer, Producer, Queue};
use core::sync::atomic::{AtomicBool, Ordering};

// 全局变量
static mut TX_QUEUE: Queue<u8, 64> = Queue::new();
static mut RX_QUEUE: Queue<u8, 64> = Queue::new();
static mut TX_PRODUCER: Option<Producer<u8, 64>> = None;
static mut TX_CONSUMER: Option<Consumer<u8, 64>> = None;
static mut RX_PRODUCER: Option<Producer<u8, 64>> = None;
static mut RX_CONSUMER: Option<Consumer<u8, 64>> = None;

static mut UART_TX: Option<Tx<USART2>> = None;
static mut UART_RX: Option<Rx<USART2>> = None;

static TX_READY: AtomicBool = AtomicBool::new(true);

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
    send_string("UART Interrupt Example Started\r\n");

    let mut counter = 0u32;

    loop {
        // 处理接收到的数据
        if let Some(data) = receive_byte() {
            // 回显接收到的字符
            send_byte(data);
            
            // 如果接收到回车，发送换行
            if data == b'\r' {
                send_byte(b'\n');
            }
        }

        // 定期发送计数器值
        counter += 1;
        if counter >= 1_000_000 {
            counter = 0;
            send_string("Heartbeat\r\n");
        }

        // 简单延时
        cortex_m::asm::nop();
    }
}

fn send_byte(byte: u8) {
    unsafe {
        if let Some(ref mut producer) = TX_PRODUCER {
            if producer.enqueue(byte).is_ok() {
                // 如果TX空闲，立即开始发送
                if TX_READY.load(Ordering::Relaxed) {
                    start_transmission();
                }
            }
        }
    }
}

fn send_string(s: &str) {
    for byte in s.bytes() {
        send_byte(byte);
    }
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
                        producer.enqueue(byte).ok();
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