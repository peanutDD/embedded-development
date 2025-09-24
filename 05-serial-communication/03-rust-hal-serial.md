# Rust HAL串口API

## 概述

Rust嵌入式生态系统提供了强大的HAL（Hardware Abstraction Layer）抽象层，使得串口编程既类型安全又高效。本文档详细介绍如何使用Rust HAL进行串口编程，包括embedded-hal特征、STM32 HAL实现和最佳实践。

## embedded-hal特征系统

### 核心特征

embedded-hal定义了一套标准的硬件抽象接口，使代码可以在不同平台间移植。

#### 1. 串口相关特征

```rust
use embedded_hal::serial::{Read, Write};
use nb;

// 写入特征
pub trait Write<Word> {
    type Error;
    
    fn write(&mut self, word: Word) -> nb::Result<(), Self::Error>;
    fn flush(&mut self) -> nb::Result<(), Self::Error>;
}

// 读取特征
pub trait Read<Word> {
    type Error;
    
    fn read(&mut self) -> nb::Result<Word, Self::Error>;
}
```

#### 2. nb（Non-Blocking）模式

embedded-hal使用nb crate实现非阻塞操作：

```rust
use nb;

// nb::Result定义
pub enum Result<T, E> {
    Ok(T),
    Err(nb::Error<E>),
}

// nb::Error定义
pub enum Error<E> {
    Other(E),      // 实际错误
    WouldBlock,    // 操作会阻塞
}
```

### 类型安全设计

Rust HAL通过类型系统确保编译时安全：

```rust
// 状态类型示例
pub struct Uart<USART, PINS, MODE> {
    usart: USART,
    pins: PINS,
    _mode: PhantomData<MODE>,
}

// 不同状态类型
pub struct Enabled;
pub struct Disabled;

// 只有启用状态的UART才能进行通信
impl<USART, PINS> Uart<USART, PINS, Enabled> {
    pub fn write(&mut self, data: u8) -> nb::Result<(), Error> {
        // 实现写入逻辑
    }
}
```

## STM32 HAL实现

### 基础配置

#### 1. 依赖配置

```toml
[dependencies]
stm32f4xx-hal = { version = "0.14", features = ["stm32f407"] }
embedded-hal = "0.2"
nb = "1.0"
cortex-m = "0.7"
```

#### 2. 基本初始化

```rust
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{config::Config, Serial},
};

fn init_uart() -> Serial<pac::USART1, (PA9<Alternate<7>>, PA10<Alternate<7>>)> {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    
    // 创建串口配置
    let config = Config::default()
        .baudrate(115200.bps())
        .wordlength_8()
        .parity_none()
        .stopbits(stm32f4xx_hal::serial::StopBits::STOP1);
    
    // 初始化串口
    Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks)
        .unwrap()
}
```

### 配置选项

#### 1. 波特率配置

```rust
use stm32f4xx_hal::time::Bps;

// 不同波特率配置方式
let config = Config::default()
    .baudrate(9600.bps())      // 9600 bps
    .baudrate(115200.bps())    // 115200 bps
    .baudrate(Bps(460800));    // 460800 bps
```

#### 2. 数据格式配置

```rust
use stm32f4xx_hal::serial::{Parity, StopBits, WordLength};

let config = Config::default()
    .wordlength_8()                    // 8位数据
    .wordlength_9()                    // 9位数据
    .parity_none()                     // 无校验
    .parity_even()                     // 偶校验
    .parity_odd()                      // 奇校验
    .stopbits(StopBits::STOP1)         // 1个停止位
    .stopbits(StopBits::STOP2);        // 2个停止位
```

#### 3. 高级配置

```rust
let config = Config::default()
    .baudrate(115200.bps())
    .oversampling_16()                 // 16倍过采样
    .oversampling_8()                  // 8倍过采样
    .dma(stm32f4xx_hal::dma::config::DmaConfig::TxRx); // DMA配置
```

## 基础操作

### 阻塞式I/O

#### 1. 发送数据

```rust
use embedded_hal::serial::Write;
use nb::block;

fn send_byte(uart: &mut impl Write<u8>, data: u8) -> Result<(), impl std::error::Error> {
    block!(uart.write(data))?;
    block!(uart.flush())?;
    Ok(())
}

fn send_string(uart: &mut impl Write<u8>, s: &str) -> Result<(), impl std::error::Error> {
    for byte in s.bytes() {
        block!(uart.write(byte))?;
    }
    block!(uart.flush())?;
    Ok(())
}
```

#### 2. 接收数据

```rust
use embedded_hal::serial::Read;
use nb::block;

fn receive_byte(uart: &mut impl Read<u8>) -> Result<u8, impl std::error::Error> {
    block!(uart.read())
}

fn receive_line(uart: &mut impl Read<u8>, buffer: &mut [u8]) -> Result<usize, impl std::error::Error> {
    let mut count = 0;
    
    for i in 0..buffer.len() {
        let byte = block!(uart.read())?;
        buffer[i] = byte;
        count += 1;
        
        if byte == b'\n' || byte == b'\r' {
            break;
        }
    }
    
    Ok(count)
}
```

### 非阻塞式I/O

#### 1. 轮询模式

```rust
use nb;

fn try_send(uart: &mut impl Write<u8>, data: u8) -> nb::Result<(), impl std::error::Error> {
    uart.write(data)
}

fn try_receive(uart: &mut impl Read<u8>) -> nb::Result<u8, impl std::error::Error> {
    uart.read()
}

// 使用示例
loop {
    match try_receive(&mut uart) {
        Ok(data) => {
            // 处理接收到的数据
            println!("Received: {}", data);
        }
        Err(nb::Error::WouldBlock) => {
            // 没有数据可读，继续其他任务
        }
        Err(nb::Error::Other(e)) => {
            // 处理错误
            println!("Error: {:?}", e);
        }
    }
}
```

#### 2. 状态机模式

```rust
enum UartState {
    Idle,
    Sending(u8),
    Receiving,
}

struct UartStateMachine {
    state: UartState,
    tx_buffer: heapless::Vec<u8, 256>,
    rx_buffer: heapless::Vec<u8, 256>,
}

impl UartStateMachine {
    fn update(&mut self, uart: &mut impl Write<u8> + Read<u8>) {
        match self.state {
            UartState::Idle => {
                if let Some(data) = self.tx_buffer.pop() {
                    self.state = UartState::Sending(data);
                }
            }
            UartState::Sending(data) => {
                match uart.write(data) {
                    Ok(()) => {
                        self.state = UartState::Idle;
                    }
                    Err(nb::Error::WouldBlock) => {
                        // 继续等待
                    }
                    Err(nb::Error::Other(_)) => {
                        // 处理错误
                        self.state = UartState::Idle;
                    }
                }
            }
            UartState::Receiving => {
                match uart.read() {
                    Ok(data) => {
                        let _ = self.rx_buffer.push(data);
                    }
                    Err(nb::Error::WouldBlock) => {
                        // 没有数据
                    }
                    Err(nb::Error::Other(_)) => {
                        // 处理错误
                    }
                }
            }
        }
    }
}
```

## 中断驱动编程

### 中断配置

```rust
use stm32f4xx_hal::{
    interrupt,
    pac::{self, NVIC},
};
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

// 全局UART实例
type UartType = Serial<pac::USART1, (PA9<Alternate<7>>, PA10<Alternate<7>>)>;
static G_UART: Mutex<RefCell<Option<UartType>>> = Mutex::new(RefCell::new(None));

fn setup_uart_interrupt() {
    let mut uart = init_uart();
    
    // 启用接收中断
    uart.listen(stm32f4xx_hal::serial::Event::Rxne);
    
    // 存储到全局变量
    cortex_m::interrupt::free(|cs| {
        G_UART.borrow(cs).replace(Some(uart));
    });
    
    // 启用NVIC中断
    unsafe {
        NVIC::unmask(pac::Interrupt::USART1);
    }
}
```

### 中断服务程序

```rust
use heapless::spsc::{Consumer, Producer, Queue};

// 接收缓冲区
static mut RX_QUEUE: Queue<u8, 256> = Queue::new();
static mut RX_PRODUCER: Option<Producer<u8, 256>> = None;
static mut RX_CONSUMER: Option<Consumer<u8, 256>> = None;

#[interrupt]
fn USART1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut uart) = G_UART.borrow(cs).borrow_mut().as_mut() {
            match uart.read() {
                Ok(data) => {
                    // 将数据放入队列
                    unsafe {
                        if let Some(ref mut producer) = RX_PRODUCER {
                            let _ = producer.enqueue(data);
                        }
                    }
                }
                Err(nb::Error::WouldBlock) => {
                    // 正常情况，没有数据
                }
                Err(nb::Error::Other(_)) => {
                    // 处理错误
                }
            }
        }
    });
}

// 主程序中读取数据
fn main_loop() {
    unsafe {
        if let Some(ref mut consumer) = RX_CONSUMER {
            while let Some(data) = consumer.dequeue() {
                // 处理接收到的数据
                println!("Received: {}", data);
            }
        }
    }
}
```

## DMA支持

### DMA配置

```rust
use stm32f4xx_hal::dma::{config::DmaConfig, PeripheralToMemory, Stream2, StreamsTuple};

fn setup_uart_dma() {
    let dp = pac::Peripherals::take().unwrap();
    
    // 初始化DMA
    let dma2 = StreamsTuple::new(dp.DMA2);
    
    // 配置UART
    let mut uart = init_uart();
    let (tx, rx) = uart.split();
    
    // 配置RX DMA
    let rx_buffer = singleton!(: [u8; 256] = [0; 256]).unwrap();
    let rx_dma = rx.with_dma(dma2.2, rx_buffer);
    
    // 启动DMA传输
    let rx_transfer = rx_dma.read_exact();
}
```

### DMA传输管理

```rust
use stm32f4xx_hal::dma::{Transfer, Stream};

struct UartDmaManager<STREAM, PERIPHERAL, BUFFER> {
    transfer: Option<Transfer<STREAM, PERIPHERAL, BUFFER>>,
}

impl<STREAM, PERIPHERAL, BUFFER> UartDmaManager<STREAM, PERIPHERAL, BUFFER>
where
    STREAM: Stream,
    Transfer<STREAM, PERIPHERAL, BUFFER>: TransferExt<BUFFER>,
{
    fn start_transfer(&mut self, transfer: Transfer<STREAM, PERIPHERAL, BUFFER>) {
        self.transfer = Some(transfer);
    }
    
    fn check_complete(&mut self) -> Option<BUFFER> {
        if let Some(transfer) = self.transfer.take() {
            if transfer.is_complete() {
                let (buffer, _) = transfer.wait();
                Some(buffer)
            } else {
                self.transfer = Some(transfer);
                None
            }
        } else {
            None
        }
    }
}
```

## 错误处理

### 错误类型

```rust
use stm32f4xx_hal::serial::Error;

#[derive(Debug)]
enum UartError {
    Hal(Error),
    BufferFull,
    Timeout,
    InvalidData,
}

impl From<Error> for UartError {
    fn from(e: Error) -> Self {
        UartError::Hal(e)
    }
}
```

### 错误恢复

```rust
fn handle_uart_error(uart: &mut impl Write<u8> + Read<u8>, error: UartError) {
    match error {
        UartError::Hal(Error::Overrun) => {
            // 清除溢出错误
            let _ = uart.read(); // 读取并丢弃数据
        }
        UartError::Hal(Error::Framing) => {
            // 帧错误，可能需要重新同步
            // 实现重新同步逻辑
        }
        UartError::Hal(Error::Parity) => {
            // 校验错误，记录并继续
            println!("Parity error detected");
        }
        UartError::BufferFull => {
            // 缓冲区满，清空或扩大缓冲区
        }
        _ => {
            // 其他错误处理
        }
    }
}
```

## 高级特性

### 1. 格式化输出

```rust
use core::fmt::Write;

// 实现fmt::Write特征
impl<UART> Write for UartWrapper<UART>
where
    UART: embedded_hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.bytes() {
            block!(self.uart.write(byte)).map_err(|_| core::fmt::Error)?;
        }
        Ok(())
    }
}

// 使用示例
fn print_formatted(uart: &mut UartWrapper<impl Write<u8>>) {
    write!(uart, "Temperature: {:.2}°C\r\n", 25.67).unwrap();
}
```

### 2. 协议解析

```rust
use heapless::Vec;

struct ProtocolParser {
    buffer: Vec<u8, 256>,
    state: ParseState,
}

enum ParseState {
    WaitingForHeader,
    ReadingLength,
    ReadingData(usize),
    ReadingChecksum,
}

impl ProtocolParser {
    fn process_byte(&mut self, byte: u8) -> Option<Vec<u8, 256>> {
        match self.state {
            ParseState::WaitingForHeader => {
                if byte == 0xAA {
                    self.buffer.clear();
                    self.buffer.push(byte).ok()?;
                    self.state = ParseState::ReadingLength;
                }
            }
            ParseState::ReadingLength => {
                self.buffer.push(byte).ok()?;
                if byte > 0 {
                    self.state = ParseState::ReadingData(byte as usize);
                } else {
                    self.state = ParseState::ReadingChecksum;
                }
            }
            ParseState::ReadingData(remaining) => {
                self.buffer.push(byte).ok()?;
                if remaining == 1 {
                    self.state = ParseState::ReadingChecksum;
                } else {
                    self.state = ParseState::ReadingData(remaining - 1);
                }
            }
            ParseState::ReadingChecksum => {
                self.buffer.push(byte).ok()?;
                // 验证校验和
                if self.verify_checksum() {
                    let result = self.buffer.clone();
                    self.state = ParseState::WaitingForHeader;
                    return Some(result);
                }
                self.state = ParseState::WaitingForHeader;
            }
        }
        None
    }
    
    fn verify_checksum(&self) -> bool {
        // 实现校验和验证逻辑
        true
    }
}
```

### 3. 异步支持

```rust
use embedded_hal_async::serial::{Read as AsyncRead, Write as AsyncWrite};

async fn async_echo<UART>(uart: &mut UART) -> Result<(), UART::Error>
where
    UART: AsyncRead<u8> + AsyncWrite<u8>,
{
    loop {
        let byte = uart.read().await?;
        uart.write(byte).await?;
        uart.flush().await?;
    }
}
```

## 测试和调试

### 单元测试

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::serial::{Mock as SerialMock, Transaction};
    
    #[test]
    fn test_send_string() {
        let expectations = [
            Transaction::write(b'H'),
            Transaction::write(b'e'),
            Transaction::write(b'l'),
            Transaction::write(b'l'),
            Transaction::write(b'o'),
            Transaction::flush(),
        ];
        
        let mut uart = SerialMock::new(&expectations);
        send_string(&mut uart, "Hello").unwrap();
        uart.done();
    }
}
```

### 调试宏

```rust
macro_rules! uart_println {
    ($uart:expr, $($arg:tt)*) => {
        {
            use core::fmt::Write;
            write!($uart, $($arg)*).ok();
            write!($uart, "\r\n").ok();
        }
    };
}

// 使用示例
uart_println!(uart, "Debug: value = {}", 42);
```

## 性能优化

### 1. 零拷贝操作

```rust
use heapless::pool::{Pool, Node};

// 使用内存池避免动态分配
static mut MEMORY: [Node<[u8; 64]>; 16] = [Node::new(); 16];
static POOL: Pool<[u8; 64]> = Pool::new();

fn zero_copy_operation() {
    // 初始化内存池
    unsafe {
        POOL.grow(&mut MEMORY);
    }
    
    // 从池中获取缓冲区
    if let Some(buffer) = POOL.alloc() {
        // 使用缓冲区
        // ...
        
        // 归还到池中
        POOL.free(buffer);
    }
}
```

### 2. 编译时优化

```rust
// 使用const泛型优化缓冲区大小
struct UartBuffer<const N: usize> {
    data: [u8; N],
    head: usize,
    tail: usize,
}

impl<const N: usize> UartBuffer<N> {
    const fn new() -> Self {
        Self {
            data: [0; N],
            head: 0,
            tail: 0,
        }
    }
}
```

## 最佳实践

### 1. 类型安全
- 使用强类型包装原始数据
- 利用类型系统防止配置错误
- 实现适当的错误处理

### 2. 资源管理
- 使用RAII模式管理资源
- 避免动态内存分配
- 合理使用静态分配

### 3. 性能考虑
- 选择合适的I/O模式（阻塞/非阻塞/中断/DMA）
- 优化缓冲区大小
- 减少不必要的数据拷贝

### 4. 可维护性
- 保持代码模块化
- 使用清晰的接口抽象
- 编写充分的测试

## 总结

Rust HAL为串口编程提供了类型安全、高性能的抽象层。通过合理使用embedded-hal特征、STM32 HAL实现和各种编程模式，可以构建出既安全又高效的串口通信系统。关键是要理解不同I/O模式的适用场景，并根据具体需求选择合适的实现方案。

## 参考资料

- [embedded-hal Documentation](https://docs.rs/embedded-hal/)
- [stm32f4xx-hal Documentation](https://docs.rs/stm32f4xx-hal/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [nb crate Documentation](https://docs.rs/nb/)