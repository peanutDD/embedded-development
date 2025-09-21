#![no_std]
#![no_main]

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{gpiob::{PB6, PB7}, gpioc::PC13, AlternateOD, Output, PushPull},
    i2c::{I2c, Mode},
    pac,
    prelude::*,
};
use panic_halt as _;

// EEPROM 设备地址 (24C02 系列)
const EEPROM_ADDR: u8 = 0x50;

// EEPROM 参数
const PAGE_SIZE: usize = 8;     // 24C02 页大小为 8 字节
const TOTAL_SIZE: usize = 256;  // 24C02 总容量 256 字节
const WRITE_CYCLE_TIME_MS: u32 = 5; // 写周期时间

type I2cType = I2c<pac::I2C1, (PB6<AlternateOD<4>>, PB7<AlternateOD<4>>)>;

#[entry]
fn main() -> ! {
    // 获取外设句柄
    let dp = pac::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    
    // 配置 GPIO
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置状态 LED
    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_high();
    
    // 配置 I2C 引脚
    let scl = gpiob.pb6.into_alternate_open_drain();
    let sda = gpiob.pb7.into_alternate_open_drain();
    
    // 初始化 I2C
    let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard { frequency: 100.kHz() },
        &clocks,
    );
    
    // 启动指示
    startup_sequence(&mut led);
    
    // EEPROM 功能演示
    demonstrate_eeprom_operations(&mut i2c, &mut led);
    
    loop {
        // 主循环
        led.set_low();
        delay_ms(500);
        led.set_high();
        delay_ms(500);
    }
}

fn startup_sequence(led: &mut PC13<Output<PushPull>>) {
    // 快速闪烁表示系统启动
    for _ in 0..6 {
        led.set_low();
        delay_ms(100);
        led.set_high();
        delay_ms(100);
    }
}

fn demonstrate_eeprom_operations(i2c: &mut I2cType, led: &mut PC13<Output<PushPull>>) {
    // 1. 扫描 I2C 设备
    scan_i2c_devices(i2c, led);
    
    // 2. 单字节读写测试
    test_single_byte_operations(i2c, led);
    
    // 3. 多字节读写测试
    test_multi_byte_operations(i2c, led);
    
    // 4. 页写入测试
    test_page_write_operations(i2c, led);
    
    // 5. 随机访问测试
    test_random_access(i2c, led);
    
    // 6. 数据完整性测试
    test_data_integrity(i2c, led);
    
    // 7. 性能测试
    test_performance(i2c, led);
}

fn scan_i2c_devices(i2c: &mut I2cType, led: &mut PC13<Output<PushPull>>) {
    // 扫描 I2C 总线上的设备
    let mut found_devices = 0;
    
    for addr in 0x08..=0x77 {
        // 尝试写入一个字节来检测设备
        if i2c.write(addr, &[]).is_ok() {
            found_devices += 1;
            
            // 闪烁 LED 表示找到设备
            for _ in 0..3 {
                led.set_low();
                delay_ms(50);
                led.set_high();
                delay_ms(50);
            }
            delay_ms(200);
        }
    }
    
    // 根据找到的设备数量闪烁 LED
    for _ in 0..found_devices {
        led.set_low();
        delay_ms(300);
        led.set_high();
        delay_ms(300);
    }
}

fn test_single_byte_operations(i2c: &mut I2cType, led: &mut PC13<Output<PushPull>>) {
    // 单字节写入测试
    let test_address = 0x00;
    let test_data = 0xA5;
    
    // 写入数据
    if eeprom_write_byte(i2c, test_address, test_data).is_ok() {
        // 读取数据
        match eeprom_read_byte(i2c, test_address) {
            Ok(read_data) => {
                if read_data == test_data {
                    // 测试成功 - 长亮 LED
                    led.set_low();
                    delay_ms(1000);
                    led.set_high();
                } else {
                    // 数据不匹配 - 快速闪烁
                    for _ in 0..10 {
                        led.set_low();
                        delay_ms(50);
                        led.set_high();
                        delay_ms(50);
                    }
                }
            }
            Err(_) => {
                // 读取失败 - 慢速闪烁
                for _ in 0..5 {
                    led.set_low();
                    delay_ms(200);
                    led.set_high();
                    delay_ms(200);
                }
            }
        }
    }
}

fn test_multi_byte_operations(i2c: &mut I2cType, led: &mut PC13<Output<PushPull>>) {
    let start_address = 0x10;
    let test_data = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];
    let mut read_buffer = [0u8; 8];
    
    // 写入多字节数据
    if eeprom_write_bytes(i2c, start_address, &test_data).is_ok() {
        // 读取多字节数据
        if eeprom_read_bytes(i2c, start_address, &mut read_buffer).is_ok() {
            // 验证数据
            let mut success = true;
            for i in 0..test_data.len() {
                if test_data[i] != read_buffer[i] {
                    success = false;
                    break;
                }
            }
            
            if success {
                // 测试成功 - 两次长亮
                for _ in 0..2 {
                    led.set_low();
                    delay_ms(500);
                    led.set_high();
                    delay_ms(500);
                }
            } else {
                // 数据验证失败
                error_indication(led, 3);
            }
        } else {
            // 读取失败
            error_indication(led, 2);
        }
    } else {
        // 写入失败
        error_indication(led, 1);
    }
}

fn test_page_write_operations(i2c: &mut I2cType, led: &mut PC13<Output<PushPull>>) {
    let page_address = 0x20; // 页对齐地址
    let page_data = [0x55; PAGE_SIZE]; // 填充数据
    let mut read_buffer = [0u8; PAGE_SIZE];
    
    // 页写入
    if eeprom_write_page(i2c, page_address, &page_data).is_ok() {
        // 读取页数据
        if eeprom_read_bytes(i2c, page_address, &mut read_buffer).is_ok() {
            // 验证数据
            if page_data == read_buffer {
                // 页写入测试成功 - 三次长亮
                for _ in 0..3 {
                    led.set_low();
                    delay_ms(400);
                    led.set_high();
                    delay_ms(400);
                }
            } else {
                error_indication(led, 4);
            }
        } else {
            error_indication(led, 5);
        }
    } else {
        error_indication(led, 6);
    }
}

fn test_random_access(i2c: &mut I2cType, led: &mut PC13<Output<PushPull>>) {
    // 随机地址访问测试
    let test_addresses = [0x00, 0x0F, 0x10, 0x1F, 0x80, 0xFF];
    let test_values = [0xAA, 0x55, 0xFF, 0x00, 0xF0, 0x0F];
    
    let mut success_count = 0;
    
    for i in 0..test_addresses.len() {
        // 写入测试值
        if eeprom_write_byte(i2c, test_addresses[i], test_values[i]).is_ok() {
            // 读取并验证
            if let Ok(read_value) = eeprom_read_byte(i2c, test_addresses[i]) {
                if read_value == test_values[i] {
                    success_count += 1;
                }
            }
        }
    }
    
    // 根据成功次数闪烁 LED
    for _ in 0..success_count {
        led.set_low();
        delay_ms(150);
        led.set_high();
        delay_ms(150);
    }
    delay_ms(1000);
}

fn test_data_integrity(i2c: &mut I2cType, led: &mut PC13<Output<PushPull>>) {
    // 数据完整性测试 - 写入模式数据并验证
    let start_addr = 0x30;
    let pattern_length = 16;
    
    // 创建测试模式
    let mut test_pattern = [0u8; 16];
    for i in 0..pattern_length {
        test_pattern[i] = (i as u8) ^ 0xAA;
    }
    
    // 写入模式数据
    if eeprom_write_bytes(i2c, start_addr, &test_pattern).is_ok() {
        delay_ms(10); // 等待写入完成
        
        // 读取并验证
        let mut read_pattern = [0u8; 16];
        if eeprom_read_bytes(i2c, start_addr, &mut read_pattern).is_ok() {
            let mut error_count = 0;
            for i in 0..pattern_length {
                if test_pattern[i] != read_pattern[i] {
                    error_count += 1;
                }
            }
            
            if error_count == 0 {
                // 完整性测试通过 - 长时间亮起
                led.set_low();
                delay_ms(2000);
                led.set_high();
            } else {
                // 有错误 - 根据错误数量闪烁
                for _ in 0..error_count {
                    led.set_low();
                    delay_ms(100);
                    led.set_high();
                    delay_ms(100);
                }
            }
        }
    }
}

fn test_performance(i2c: &mut I2cType, led: &mut PC13<Output<PushPull>>) {
    // 性能测试 - 测量读写速度
    let test_size = 64;
    let start_addr = 0x40;
    let test_data = [0x5A; 64];
    let mut read_buffer = [0u8; 64];
    
    // 写入性能测试
    led.set_low(); // 开始计时指示
    
    // 分页写入 (每页 8 字节)
    for page in 0..(test_size / PAGE_SIZE) {
        let page_addr = start_addr + (page * PAGE_SIZE) as u8;
        let page_start = page * PAGE_SIZE;
        let page_end = page_start + PAGE_SIZE;
        
        let _ = eeprom_write_page(i2c, page_addr, &test_data[page_start..page_end]);
    }
    
    led.set_high(); // 写入完成
    delay_ms(100);
    
    // 读取性能测试
    led.set_low(); // 开始读取计时
    
    let _ = eeprom_read_bytes(i2c, start_addr, &mut read_buffer);
    
    led.set_high(); // 读取完成
    
    // 验证数据正确性
    let mut correct = true;
    for i in 0..test_size {
        if test_data[i] != read_buffer[i] {
            correct = false;
            break;
        }
    }
    
    if correct {
        // 性能测试成功 - 快速闪烁 5 次
        for _ in 0..5 {
            led.set_low();
            delay_ms(100);
            led.set_high();
            delay_ms(100);
        }
    }
}

// EEPROM 操作函数
fn eeprom_write_byte(i2c: &mut I2cType, address: u8, data: u8) -> Result<(), ()> {
    let write_data = [address, data];
    
    match i2c.write(EEPROM_ADDR, &write_data) {
        Ok(_) => {
            // 等待写周期完成
            delay_ms(WRITE_CYCLE_TIME_MS);
            
            // 轮询写入完成状态
            wait_write_complete(i2c);
            Ok(())
        }
        Err(_) => Err(()),
    }
}

fn eeprom_read_byte(i2c: &mut I2cType, address: u8) -> Result<u8, ()> {
    let mut read_data = [0u8; 1];
    
    // 写入地址
    match i2c.write(EEPROM_ADDR, &[address]) {
        Ok(_) => {
            // 读取数据
            match i2c.read(EEPROM_ADDR, &mut read_data) {
                Ok(_) => Ok(read_data[0]),
                Err(_) => Err(()),
            }
        }
        Err(_) => Err(()),
    }
}

fn eeprom_write_bytes(i2c: &mut I2cType, start_address: u8, data: &[u8]) -> Result<(), ()> {
    // 逐字节写入 (简单实现)
    for (i, &byte) in data.iter().enumerate() {
        let addr = start_address.wrapping_add(i as u8);
        eeprom_write_byte(i2c, addr, byte)?;
    }
    Ok(())
}

fn eeprom_read_bytes(i2c: &mut I2cType, start_address: u8, buffer: &mut [u8]) -> Result<(), ()> {
    // 写入起始地址
    match i2c.write(EEPROM_ADDR, &[start_address]) {
        Ok(_) => {
            // 连续读取数据
            match i2c.read(EEPROM_ADDR, buffer) {
                Ok(_) => Ok(()),
                Err(_) => Err(()),
            }
        }
        Err(_) => Err(()),
    }
}

fn eeprom_write_page(i2c: &mut I2cType, page_address: u8, data: &[u8]) -> Result<(), ()> {
    assert!(data.len() <= PAGE_SIZE);
    assert!(page_address % PAGE_SIZE as u8 == 0); // 页对齐检查
    
    // 构建写入数据 (地址 + 数据)
    let mut write_buffer = [0u8; PAGE_SIZE + 1];
    write_buffer[0] = page_address;
    write_buffer[1..=data.len()].copy_from_slice(data);
    
    match i2c.write(EEPROM_ADDR, &write_buffer[..=data.len()]) {
        Ok(_) => {
            // 等待页写入完成
            delay_ms(WRITE_CYCLE_TIME_MS);
            wait_write_complete(i2c);
            Ok(())
        }
        Err(_) => Err(()),
    }
}

fn wait_write_complete(i2c: &mut I2cType) {
    // 轮询 ACK 来检测写入完成
    let mut retry_count = 0;
    const MAX_RETRIES: u32 = 100;
    
    while retry_count < MAX_RETRIES {
        // 尝试写入空数据，如果 ACK 则写入完成
        if i2c.write(EEPROM_ADDR, &[]).is_ok() {
            break;
        }
        
        delay_ms(1);
        retry_count += 1;
    }
}

fn error_indication(led: &mut PC13<Output<PushPull>>, error_code: u8) {
    // 根据错误代码闪烁 LED
    for _ in 0..error_code {
        led.set_low();
        delay_ms(200);
        led.set_high();
        delay_ms(200);
    }
    delay_ms(1000);
}

fn delay_ms(ms: u32) {
    // 简单的延时函数
    for _ in 0..(ms * 21000) {
        cortex_m::asm::nop();
    }
}

// EEPROM 高级功能
mod eeprom_advanced {
    use super::*;
    
    pub fn eeprom_fill(i2c: &mut I2cType, start_addr: u8, length: usize, value: u8) -> Result<(), ()> {
        // 填充指定区域
        for i in 0..length {
            let addr = start_addr.wrapping_add(i as u8);
            eeprom_write_byte(i2c, addr, value)?;
        }
        Ok(())
    }
    
    pub fn eeprom_copy(i2c: &mut I2cType, src_addr: u8, dst_addr: u8, length: usize) -> Result<(), ()> {
        // 复制数据块
        for i in 0..length {
            let src = src_addr.wrapping_add(i as u8);
            let dst = dst_addr.wrapping_add(i as u8);
            
            let data = eeprom_read_byte(i2c, src)?;
            eeprom_write_byte(i2c, dst, data)?;
        }
        Ok(())
    }
    
    pub fn eeprom_compare(i2c: &mut I2cType, addr1: u8, addr2: u8, length: usize) -> Result<bool, ()> {
        // 比较两个数据块
        for i in 0..length {
            let a1 = addr1.wrapping_add(i as u8);
            let a2 = addr2.wrapping_add(i as u8);
            
            let data1 = eeprom_read_byte(i2c, a1)?;
            let data2 = eeprom_read_byte(i2c, a2)?;
            
            if data1 != data2 {
                return Ok(false);
            }
        }
        Ok(true)
    }
    
    pub fn eeprom_checksum(i2c: &mut I2cType, start_addr: u8, length: usize) -> Result<u8, ()> {
        // 计算校验和
        let mut checksum = 0u8;
        
        for i in 0..length {
            let addr = start_addr.wrapping_add(i as u8);
            let data = eeprom_read_byte(i2c, addr)?;
            checksum = checksum.wrapping_add(data);
        }
        
        Ok(checksum)
    }
}