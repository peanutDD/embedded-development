# SPI 协议详解

## 概述

SPI (Serial Peripheral Interface) 是由摩托罗拉公司开发的一种全双工同步串行通信协议。它是一种主从式通信协议，广泛用于微控制器与各种外设之间的高速数据传输。

## SPI 总线特性

### 物理特性
- **四线制**: SCLK (时钟)、MOSI (主出从入)、MISO (主入从出)、SS/CS (片选)
- **全双工**: 可以同时发送和接收数据
- **主从模式**: 明确的主机和从机角色
- **高速传输**: 支持几十 MHz 的时钟频率

### 信号线定义
- **SCLK (Serial Clock)**: 时钟信号，由主机产生
- **MOSI (Master Out Slave In)**: 主机输出，从机输入数据线
- **MISO (Master In Slave Out)**: 主机输入，从机输出数据线
- **SS/CS (Slave Select/Chip Select)**: 片选信号，选择特定从机

### 连接方式
```
主机                    从机1
SCLK  ----------------> SCLK
MOSI  ----------------> MOSI
MISO  <---------------- MISO
CS1   ----------------> CS

                        从机2
SCLK  ----------------> SCLK
MOSI  ----------------> MOSI
MISO  <---------------- MISO
CS2   ----------------> CS
```

## SPI 通信模式

### 时钟极性和相位 (CPOL/CPHA)

SPI 有四种工作模式，由时钟极性 (CPOL) 和时钟相位 (CPHA) 决定：

| 模式 | CPOL | CPHA | 时钟空闲状态 | 数据采样边沿 |
|------|------|------|-------------|-------------|
| 0    | 0    | 0    | 低电平      | 上升沿      |
| 1    | 0    | 1    | 低电平      | 下降沿      |
| 2    | 1    | 0    | 高电平      | 下降沿      |
| 3    | 1    | 1    | 高电平      | 上升沿      |

### 模式 0 (CPOL=0, CPHA=0) 时序图
```
CS    ----\___________________/----
SCLK  ____/‾\__/‾\__/‾\__/‾\______
MOSI  ----< D7 >< D6 >< D5 >< D4 >--
MISO  ----< D7 >< D6 >< D5 >< D4 >--
      采样  ↑    ↑    ↑    ↑
```

### 模式 1 (CPOL=0, CPHA=1) 时序图
```
CS    ----\___________________/----
SCLK  ____/‾\__/‾\__/‾\__/‾\______
MOSI  ----< D7 >< D6 >< D5 >< D4 >--
MISO  ----< D7 >< D6 >< D5 >< D4 >--
      采样     ↓    ↓    ↓    ↓
```

## SPI 数据传输

### 基本传输过程
```rust
fn spi_transfer_byte(data: u8) -> u8 {
    // 1. 拉低片选信号
    cs_low();
    
    // 2. 发送数据位 (MSB 先发送)
    let mut tx_data = data;
    let mut rx_data = 0u8;
    
    for bit in 0..8 {
        // 设置 MOSI 线
        if tx_data & 0x80 != 0 {
            mosi_high();
        } else {
            mosi_low();
        }
        
        // 产生时钟脉冲
        sclk_high();
        delay_half_period();
        
        // 读取 MISO 线
        rx_data <<= 1;
        if miso_read() {
            rx_data |= 0x01;
        }
        
        sclk_low();
        delay_half_period();
        
        tx_data <<= 1;
    }
    
    // 3. 拉高片选信号
    cs_high();
    
    rx_data
}
```

### 多字节传输
```rust
fn spi_transfer_multiple(tx_data: &[u8], rx_data: &mut [u8]) {
    assert_eq!(tx_data.len(), rx_data.len());
    
    cs_low();
    
    for i in 0..tx_data.len() {
        rx_data[i] = spi_transfer_byte_no_cs(tx_data[i]);
    }
    
    cs_high();
}

fn spi_transfer_byte_no_cs(data: u8) -> u8 {
    // 不操作片选信号的字节传输
    // 用于连续传输多个字节
    // ... 实现细节同上，但不操作 CS
}
```

## SPI 配置参数

### 时钟频率
```rust
enum SpiClockSpeed {
    Speed125kHz = 125_000,
    Speed250kHz = 250_000,
    Speed500kHz = 500_000,
    Speed1MHz = 1_000_000,
    Speed2MHz = 2_000_000,
    Speed4MHz = 4_000_000,
    Speed8MHz = 8_000_000,
    Speed16MHz = 16_000_000,
}

fn calculate_prescaler(system_clock: u32, target_speed: u32) -> u16 {
    let prescaler = system_clock / target_speed;
    
    // 找到最接近的 2 的幂次
    match prescaler {
        0..=2 => 2,
        3..=4 => 4,
        5..=8 => 8,
        9..=16 => 16,
        17..=32 => 32,
        33..=64 => 64,
        65..=128 => 128,
        _ => 256,
    }
}
```

### 数据格式
```rust
enum SpiDataSize {
    Bits8 = 8,
    Bits16 = 16,
}

enum SpiBitOrder {
    MsbFirst,  // 最高位先发送
    LsbFirst,  // 最低位先发送
}

struct SpiConfig {
    mode: u8,           // 0-3
    clock_speed: u32,   // Hz
    data_size: SpiDataSize,
    bit_order: SpiBitOrder,
    cs_active_low: bool,
}
```

## SPI 与常见设备通信

### Flash 存储器 (W25Q64)
```rust
// W25Q64 Flash 存储器命令
const CMD_READ_DATA: u8 = 0x03;
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_SECTOR_ERASE: u8 = 0x20;
const CMD_READ_STATUS: u8 = 0x05;

fn flash_read(address: u32, buffer: &mut [u8]) {
    cs_low();
    
    // 发送读命令
    spi_transfer_byte_no_cs(CMD_READ_DATA);
    
    // 发送 24 位地址
    spi_transfer_byte_no_cs((address >> 16) as u8);
    spi_transfer_byte_no_cs((address >> 8) as u8);
    spi_transfer_byte_no_cs(address as u8);
    
    // 读取数据
    for i in 0..buffer.len() {
        buffer[i] = spi_transfer_byte_no_cs(0x00);
    }
    
    cs_high();
}

fn flash_write_page(address: u32, data: &[u8]) {
    // 写使能
    cs_low();
    spi_transfer_byte_no_cs(CMD_WRITE_ENABLE);
    cs_high();
    
    // 页编程
    cs_low();
    spi_transfer_byte_no_cs(CMD_PAGE_PROGRAM);
    spi_transfer_byte_no_cs((address >> 16) as u8);
    spi_transfer_byte_no_cs((address >> 8) as u8);
    spi_transfer_byte_no_cs(address as u8);
    
    for &byte in data {
        spi_transfer_byte_no_cs(byte);
    }
    
    cs_high();
    
    // 等待写入完成
    wait_flash_ready();
}

fn wait_flash_ready() {
    loop {
        cs_low();
        spi_transfer_byte_no_cs(CMD_READ_STATUS);
        let status = spi_transfer_byte_no_cs(0x00);
        cs_high();
        
        if status & 0x01 == 0 { // WIP bit = 0
            break;
        }
        
        delay_ms(1);
    }
}
```

### ADC 转换器 (MCP3008)
```rust
// MCP3008 8通道 10位 ADC
fn mcp3008_read_channel(channel: u8) -> u16 {
    assert!(channel < 8);
    
    cs_low();
    
    // 发送起始位和配置
    spi_transfer_byte_no_cs(0x01); // 起始位
    
    // 发送通道选择 (单端模式)
    let config = 0x80 | (channel << 4);
    let high_byte = spi_transfer_byte_no_cs(config);
    let low_byte = spi_transfer_byte_no_cs(0x00);
    
    cs_high();
    
    // 组合 10 位结果
    let result = ((high_byte as u16 & 0x03) << 8) | (low_byte as u16);
    result
}

fn mcp3008_read_all_channels() -> [u16; 8] {
    let mut results = [0u16; 8];
    
    for channel in 0..8 {
        results[channel] = mcp3008_read_channel(channel);
    }
    
    results
}
```

### 显示屏 (ST7735)
```rust
// ST7735 TFT 显示屏控制
const CMD_SWRESET: u8 = 0x01;
const CMD_SLPOUT: u8 = 0x11;
const CMD_DISPON: u8 = 0x29;
const CMD_CASET: u8 = 0x2A;
const CMD_RASET: u8 = 0x2B;
const CMD_RAMWR: u8 = 0x2C;

fn st7735_send_command(cmd: u8) {
    dc_low();  // 命令模式
    cs_low();
    spi_transfer_byte_no_cs(cmd);
    cs_high();
}

fn st7735_send_data(data: u8) {
    dc_high(); // 数据模式
    cs_low();
    spi_transfer_byte_no_cs(data);
    cs_high();
}

fn st7735_init() {
    // 硬件复位
    reset_low();
    delay_ms(10);
    reset_high();
    delay_ms(120);
    
    // 软件复位
    st7735_send_command(CMD_SWRESET);
    delay_ms(150);
    
    // 退出睡眠模式
    st7735_send_command(CMD_SLPOUT);
    delay_ms(500);
    
    // 开启显示
    st7735_send_command(CMD_DISPON);
    delay_ms(100);
}

fn st7735_draw_pixel(x: u16, y: u16, color: u16) {
    // 设置列地址
    st7735_send_command(CMD_CASET);
    st7735_send_data((x >> 8) as u8);
    st7735_send_data(x as u8);
    st7735_send_data((x >> 8) as u8);
    st7735_send_data(x as u8);
    
    // 设置行地址
    st7735_send_command(CMD_RASET);
    st7735_send_data((y >> 8) as u8);
    st7735_send_data(y as u8);
    st7735_send_data((y >> 8) as u8);
    st7735_send_data(y as u8);
    
    // 写入像素数据
    st7735_send_command(CMD_RAMWR);
    st7735_send_data((color >> 8) as u8);
    st7735_send_data(color as u8);
}
```

## SPI DMA 传输

### DMA 配置
```rust
fn setup_spi_dma() {
    // 配置 TX DMA 通道
    let tx_dma_config = DmaConfig {
        channel: DmaChannel::Channel3,
        direction: DmaDirection::MemoryToPeripheral,
        memory_increment: true,
        peripheral_increment: false,
        data_size: DmaDataSize::Byte,
        priority: DmaPriority::High,
    };
    
    // 配置 RX DMA 通道
    let rx_dma_config = DmaConfig {
        channel: DmaChannel::Channel2,
        direction: DmaDirection::PeripheralToMemory,
        memory_increment: true,
        peripheral_increment: false,
        data_size: DmaDataSize::Byte,
        priority: DmaPriority::High,
    };
    
    configure_dma_channel(tx_dma_config);
    configure_dma_channel(rx_dma_config);
}

fn spi_dma_transfer(tx_buffer: &[u8], rx_buffer: &mut [u8]) {
    assert_eq!(tx_buffer.len(), rx_buffer.len());
    
    // 启动 DMA 传输
    start_dma_tx(tx_buffer);
    start_dma_rx(rx_buffer);
    
    // 启动 SPI 传输
    enable_spi_dma();
    
    // 等待传输完成
    wait_dma_complete();
    
    // 清理
    disable_spi_dma();
}
```

## SPI 性能优化

### 时钟频率优化
```rust
fn optimize_spi_speed(device_type: DeviceType) -> u32 {
    match device_type {
        DeviceType::Flash => {
            // Flash 存储器通常支持高速
            min(system_clock() / 2, 50_000_000) // 最高 50MHz
        }
        DeviceType::Adc => {
            // ADC 需要稳定的时钟
            min(system_clock() / 8, 5_000_000)  // 最高 5MHz
        }
        DeviceType::Display => {
            // 显示屏需要平衡速度和稳定性
            min(system_clock() / 4, 20_000_000) // 最高 20MHz
        }
        DeviceType::Sensor => {
            // 传感器通常速度要求不高
            min(system_clock() / 16, 2_000_000) // 最高 2MHz
        }
    }
}
```

### 批量传输优化
```rust
fn optimized_flash_read(address: u32, buffer: &mut [u8]) {
    const MAX_CHUNK_SIZE: usize = 256;
    
    let mut remaining = buffer.len();
    let mut current_addr = address;
    let mut buffer_offset = 0;
    
    while remaining > 0 {
        let chunk_size = min(remaining, MAX_CHUNK_SIZE);
        
        // 使用 DMA 进行大块传输
        if chunk_size >= 64 {
            flash_read_dma(
                current_addr,
                &mut buffer[buffer_offset..buffer_offset + chunk_size]
            );
        } else {
            // 小块数据使用普通传输
            flash_read_normal(
                current_addr,
                &mut buffer[buffer_offset..buffer_offset + chunk_size]
            );
        }
        
        remaining -= chunk_size;
        current_addr += chunk_size as u32;
        buffer_offset += chunk_size;
    }
}
```

## 错误处理和调试

### 常见错误类型
```rust
#[derive(Debug)]
enum SpiError {
    BusyTimeout,
    TransmissionError,
    OverrunError,
    ModeError,
    CrcError,
    DmaError,
}

fn handle_spi_error(error: SpiError) {
    match error {
        SpiError::BusyTimeout => {
            // 重置 SPI 外设
            reset_spi_peripheral();
        }
        SpiError::OverrunError => {
            // 清除接收缓冲区
            clear_rx_buffer();
        }
        SpiError::ModeError => {
            // 重新配置 SPI 模式
            reconfigure_spi_mode();
        }
        SpiError::CrcError => {
            // 重传数据
            retransmit_data();
        }
        SpiError::DmaError => {
            // 重置 DMA 通道
            reset_dma_channels();
        }
        _ => {
            // 通用错误处理
            reset_spi_system();
        }
    }
}
```

### 调试技巧
```rust
fn debug_spi_communication() {
    // 1. 检查 SPI 配置
    let config = get_spi_config();
    println!("SPI Mode: {}", config.mode);
    println!("Clock Speed: {} Hz", config.clock_speed);
    println!("Data Size: {} bits", config.data_size);
    
    // 2. 测试回环模式
    enable_loopback_mode();
    let test_data = [0x55, 0xAA, 0xFF, 0x00];
    let mut rx_data = [0u8; 4];
    spi_transfer(&test_data, &mut rx_data);
    
    if test_data == rx_data {
        println!("Loopback test passed");
    } else {
        println!("Loopback test failed");
    }
    
    // 3. 监控信号质量
    check_signal_integrity();
}

fn check_signal_integrity() {
    // 检查时钟信号
    let clock_freq = measure_clock_frequency();
    println!("Measured clock frequency: {} Hz", clock_freq);
    
    // 检查数据线
    let mosi_level = read_mosi_level();
    let miso_level = read_miso_level();
    println!("MOSI level: {}, MISO level: {}", mosi_level, miso_level);
    
    // 检查片选信号
    let cs_level = read_cs_level();
    println!("CS level: {}", cs_level);
}
```

## SPI vs I2C 比较

| 特性 | SPI | I2C |
|------|-----|-----|
| 线数 | 4 (+ 每个从机一根 CS) | 2 |
| 速度 | 高 (几十 MHz) | 中等 (400kHz - 3.4MHz) |
| 复杂度 | 简单 | 中等 |
| 多主机 | 复杂 | 支持 |
| 设备数量 | 受 CS 引脚限制 | 127 个 |
| 功耗 | 较高 | 较低 |
| 距离 | 短 | 中等 |

## 总结

SPI 是一种高速、简单的串行通信协议，具有以下特点：

**优点**:
- 高速传输能力
- 全双工通信
- 简单的协议实现
- 无地址冲突问题

**缺点**:
- 需要更多的引脚
- 不支持多主机
- 没有内置的错误检测

**适用场景**:
- 高速数据传输
- 简单的点对点通信
- 对实时性要求高的应用
- Flash 存储器、ADC、显示屏等设备

掌握 SPI 协议的原理和实现，对于嵌入式系统中的高速数据传输具有重要意义。