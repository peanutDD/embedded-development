#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{
    gpioa::{PA5, PA6, PA7},
    gpiob::PB0,
    gpioc::PC13,
    Alternate, Output, PushPull,
  },
  pac,
  prelude::*,
  spi::{Mode, Phase, Polarity, Spi},
};

// W25Q64 Flash 命令定义
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_WRITE_DISABLE: u8 = 0x04;
const CMD_READ_STATUS: u8 = 0x05;
const CMD_WRITE_STATUS: u8 = 0x01;
const CMD_READ_DATA: u8 = 0x03;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_SECTOR_ERASE: u8 = 0x20;
const CMD_BLOCK_ERASE_32K: u8 = 0x52;
const CMD_BLOCK_ERASE_64K: u8 = 0xD8;
const CMD_CHIP_ERASE: u8 = 0xC7;
const CMD_POWER_DOWN: u8 = 0xB9;
const CMD_RELEASE_POWER_DOWN: u8 = 0xAB;
const CMD_DEVICE_ID: u8 = 0x90;
const CMD_JEDEC_ID: u8 = 0x9F;

// Flash 参数
const PAGE_SIZE: usize = 256;
const SECTOR_SIZE: usize = 4096;
const BLOCK_32K_SIZE: usize = 32768;
const BLOCK_64K_SIZE: usize = 65536;

// 状态寄存器位定义
const STATUS_BUSY: u8 = 0x01;
const STATUS_WEL: u8 = 0x02;

type SpiType = Spi<pac::SPI1, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>;
type CsPin = PB0<Output<PushPull>>;

#[entry]
fn main() -> ! {
  // 获取外设句柄
  let dp = pac::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

  // 配置 GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 配置状态 LED
  let mut led = gpioc.pc13.into_push_pull_output();
  led.set_high();

  // 配置 SPI 引脚
  let sck = gpioa.pa5.into_alternate(); // SCK
  let miso = gpioa.pa6.into_alternate(); // MISO
  let mosi = gpioa.pa7.into_alternate(); // MOSI
  let mut cs = gpiob.pb0.into_push_pull_output(); // CS

  // 初始化 CS 为高电平 (未选中)
  cs.set_high();

  // 配置 SPI
  let spi_mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
  };

  let mut spi = Spi::new(dp.SPI1, (sck, miso, mosi), spi_mode, 8.MHz(), &clocks);

  // 启动指示
  startup_sequence(&mut led);

  // Flash 功能演示
  demonstrate_flash_operations(&mut spi, &mut cs, &mut led);

  loop {
    // 主循环
    led.set_low();
    delay_ms(1000);
    led.set_high();
    delay_ms(1000);
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

fn demonstrate_flash_operations(
  spi: &mut SpiType,
  cs: &mut CsPin,
  led: &mut PC13<Output<PushPull>>,
) {
  // 1. Flash 初始化和识别
  test_flash_identification(spi, cs, led);

  // 2. 状态寄存器操作
  test_status_register(spi, cs, led);

  // 3. 单字节读写测试
  test_single_byte_operations(spi, cs, led);

  // 4. 页编程测试
  test_page_program(spi, cs, led);

  // 5. 扇区擦除测试
  test_sector_erase(spi, cs, led);

  // 6. 快速读取测试
  test_fast_read(spi, cs, led);

  // 7. 大数据块操作测试
  test_large_data_operations(spi, cs, led);

  // 8. 性能测试
  test_performance(spi, cs, led);

  // 9. 电源管理测试
  test_power_management(spi, cs, led);
}

fn test_flash_identification(spi: &mut SpiType, cs: &mut CsPin, led: &mut PC13<Output<PushPull>>) {
  // 读取 JEDEC ID
  let jedec_id = flash_read_jedec_id(spi, cs);

  // 检查是否为预期的 Flash 芯片 (W25Q64: EF4017)
  if jedec_id[0] == 0xEF && jedec_id[1] == 0x40 && jedec_id[2] == 0x17 {
    // 识别成功 - 长亮 LED
    led.set_low();
    delay_ms(1000);
    led.set_high();
  } else {
    // 识别失败 - 快速闪烁
    for _ in 0..10 {
      led.set_low();
      delay_ms(50);
      led.set_high();
      delay_ms(50);
    }
  }

  // 读取设备 ID
  let device_id = flash_read_device_id(spi, cs);

  // 验证设备 ID (W25Q64: 16)
  if device_id == 0x16 {
    // 设备 ID 正确 - 两次闪烁
    for _ in 0..2 {
      led.set_low();
      delay_ms(200);
      led.set_high();
      delay_ms(200);
    }
  }
}

fn test_status_register(spi: &mut SpiType, cs: &mut CsPin, led: &mut PC13<Output<PushPull>>) {
  // 读取状态寄存器
  let status = flash_read_status(spi, cs);

  // 检查初始状态 (应该不忙且写保护禁用)
  if (status & STATUS_BUSY) == 0 {
    // Flash 不忙 - 短闪烁
    led.set_low();
    delay_ms(100);
    led.set_high();
    delay_ms(100);
  }

  // 测试写使能
  flash_write_enable(spi, cs);
  let status_after_we = flash_read_status(spi, cs);

  if (status_after_we & STATUS_WEL) != 0 {
    // 写使能成功 - 两次短闪烁
    for _ in 0..2 {
      led.set_low();
      delay_ms(100);
      led.set_high();
      delay_ms(100);
    }
  }

  // 测试写禁用
  flash_write_disable(spi, cs);
  let status_after_wd = flash_read_status(spi, cs);

  if (status_after_wd & STATUS_WEL) == 0 {
    // 写禁用成功 - 三次短闪烁
    for _ in 0..3 {
      led.set_low();
      delay_ms(100);
      led.set_high();
      delay_ms(100);
    }
  }
}

fn test_single_byte_operations(
  spi: &mut SpiType,
  cs: &mut CsPin,
  led: &mut PC13<Output<PushPull>>,
) {
  let test_address = 0x001000; // 测试地址
  let test_data = [0xA5, 0x5A, 0xFF, 0x00];

  // 先擦除扇区
  flash_sector_erase(spi, cs, test_address);

  // 写入测试数据
  flash_page_program(spi, cs, test_address, &test_data);

  // 读取数据
  let mut read_buffer = [0u8; 4];
  flash_read_data(spi, cs, test_address, &mut read_buffer);

  // 验证数据
  let mut success = true;
  for i in 0..test_data.len() {
    if test_data[i] != read_buffer[i] {
      success = false;
      break;
    }
  }

  if success {
    // 测试成功 - 长亮
    led.set_low();
    delay_ms(800);
    led.set_high();
  } else {
    // 测试失败 - 错误指示
    error_indication(led, 1);
  }
}

fn test_page_program(spi: &mut SpiType, cs: &mut CsPin, led: &mut PC13<Output<PushPull>>) {
  let page_address = 0x002000;
  let mut test_page = [0u8; PAGE_SIZE];

  // 创建测试模式
  for i in 0..PAGE_SIZE {
    test_page[i] = (i & 0xFF) as u8;
  }

  // 擦除扇区
  flash_sector_erase(spi, cs, page_address);

  // 页编程
  flash_page_program(spi, cs, page_address, &test_page);

  // 读取整页数据
  let mut read_page = [0u8; PAGE_SIZE];
  flash_read_data(spi, cs, page_address, &mut read_page);

  // 验证数据
  let mut error_count = 0;
  for i in 0..PAGE_SIZE {
    if test_page[i] != read_page[i] {
      error_count += 1;
    }
  }

  if error_count == 0 {
    // 页编程成功 - 长时间亮起
    led.set_low();
    delay_ms(1500);
    led.set_high();
  } else {
    // 有错误 - 根据错误数量闪烁
    let blink_count = if error_count > 10 { 10 } else { error_count };
    for _ in 0..blink_count {
      led.set_low();
      delay_ms(100);
      led.set_high();
      delay_ms(100);
    }
  }
}

fn test_sector_erase(spi: &mut SpiType, cs: &mut CsPin, led: &mut PC13<Output<PushPull>>) {
  let sector_address = 0x003000;

  // 先写入一些数据
  let test_data = [0xFF; 256];
  flash_page_program(spi, cs, sector_address, &test_data);

  // 擦除扇区
  flash_sector_erase(spi, cs, sector_address);

  // 读取擦除后的数据
  let mut read_buffer = [0u8; 256];
  flash_read_data(spi, cs, sector_address, &mut read_buffer);

  // 验证数据是否全为 0xFF
  let mut erased_correctly = true;
  for &byte in read_buffer.iter() {
    if byte != 0xFF {
      erased_correctly = false;
      break;
    }
  }

  if erased_correctly {
    // 擦除成功 - 两次长亮
    for _ in 0..2 {
      led.set_low();
      delay_ms(500);
      led.set_high();
      delay_ms(500);
    }
  } else {
    error_indication(led, 2);
  }
}

fn test_fast_read(spi: &mut SpiType, cs: &mut CsPin, led: &mut PC13<Output<PushPull>>) {
  let test_address = 0x004000;
  let test_data = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0];

  // 擦除并写入测试数据
  flash_sector_erase(spi, cs, test_address);
  flash_page_program(spi, cs, test_address, &test_data);

  // 使用快速读取
  let mut read_buffer = [0u8; 8];
  flash_fast_read(spi, cs, test_address, &mut read_buffer);

  // 验证数据
  let mut success = true;
  for i in 0..test_data.len() {
    if test_data[i] != read_buffer[i] {
      success = false;
      break;
    }
  }

  if success {
    // 快速读取成功 - 快速闪烁 5 次
    for _ in 0..5 {
      led.set_low();
      delay_ms(80);
      led.set_high();
      delay_ms(80);
    }
  } else {
    error_indication(led, 3);
  }
}

fn test_large_data_operations(spi: &mut SpiType, cs: &mut CsPin, led: &mut PC13<Output<PushPull>>) {
  let start_address = 0x005000;
  let data_size = 1024; // 1KB 数据

  // 创建测试数据
  let mut test_data = [0u8; 1024];
  for i in 0..data_size {
    test_data[i] = ((i * 3) & 0xFF) as u8;
  }

  // 擦除足够的扇区 (1KB 需要 1 个扇区)
  flash_sector_erase(spi, cs, start_address);

  // 分页写入数据
  let pages = data_size / PAGE_SIZE;
  for page in 0..pages {
    let page_addr = start_address + (page * PAGE_SIZE) as u32;
    let page_start = page * PAGE_SIZE;
    let page_end = page_start + PAGE_SIZE;

    flash_page_program(spi, cs, page_addr, &test_data[page_start..page_end]);
  }

  // 读取并验证数据
  let mut read_buffer = [0u8; 1024];
  flash_read_data(spi, cs, start_address, &mut read_buffer);

  let mut error_count = 0;
  for i in 0..data_size {
    if test_data[i] != read_buffer[i] {
      error_count += 1;
    }
  }

  if error_count == 0 {
    // 大数据操作成功 - 长时间亮起
    led.set_low();
    delay_ms(2000);
    led.set_high();
  } else {
    // 显示错误数量 (最多 20 次闪烁)
    let blink_count = if error_count > 20 { 20 } else { error_count };
    for _ in 0..blink_count {
      led.set_low();
      delay_ms(50);
      led.set_high();
      delay_ms(50);
    }
  }
}

fn test_performance(spi: &mut SpiType, cs: &mut CsPin, led: &mut PC13<Output<PushPull>>) {
  let test_address = 0x006000;
  let test_size = 512;
  let test_data = [0x55; 512];
  let mut read_buffer = [0u8; 512];

  // 擦除扇区
  flash_sector_erase(spi, cs, test_address);

  // 写入性能测试
  led.set_low(); // 开始计时指示

  // 分页写入
  let pages = test_size / PAGE_SIZE;
  for page in 0..pages {
    let page_addr = test_address + (page * PAGE_SIZE) as u32;
    let page_start = page * PAGE_SIZE;
    let page_end = page_start + PAGE_SIZE;

    flash_page_program(spi, cs, page_addr, &test_data[page_start..page_end]);
  }

  led.set_high(); // 写入完成
  delay_ms(100);

  // 读取性能测试
  led.set_low(); // 开始读取计时

  flash_read_data(spi, cs, test_address, &mut read_buffer);

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
    // 性能测试成功 - 快速闪烁 8 次
    for _ in 0..8 {
      led.set_low();
      delay_ms(60);
      led.set_high();
      delay_ms(60);
    }
  }
}

fn test_power_management(spi: &mut SpiType, cs: &mut CsPin, led: &mut PC13<Output<PushPull>>) {
  // 进入掉电模式
  flash_power_down(spi, cs);

  // 指示进入掉电模式
  led.set_low();
  delay_ms(500);
  led.set_high();
  delay_ms(500);

  // 退出掉电模式
  flash_release_power_down(spi, cs);

  // 测试是否正常工作
  let jedec_id = flash_read_jedec_id(spi, cs);

  if jedec_id[0] == 0xEF && jedec_id[1] == 0x40 && jedec_id[2] == 0x17 {
    // 电源管理测试成功 - 三次长亮
    for _ in 0..3 {
      led.set_low();
      delay_ms(400);
      led.set_high();
      delay_ms(400);
    }
  } else {
    error_indication(led, 4);
  }
}

// Flash 操作函数
fn flash_read_jedec_id(spi: &mut SpiType, cs: &mut CsPin) -> [u8; 3] {
  let mut id = [0u8; 3];

  cs.set_low();
  let _ = spi.send(CMD_JEDEC_ID);

  for i in 0..3 {
    id[i] = spi_transfer(spi, 0x00);
  }

  cs.set_high();
  id
}

fn flash_read_device_id(spi: &mut SpiType, cs: &mut CsPin) -> u8 {
  cs.set_low();
  let _ = spi.send(CMD_DEVICE_ID);
  let _ = spi.send(0x00); // 地址高字节
  let _ = spi.send(0x00); // 地址中字节
  let _ = spi.send(0x00); // 地址低字节

  let _ = spi_transfer(spi, 0x00); // 制造商 ID (跳过)
  let device_id = spi_transfer(spi, 0x00);

  cs.set_high();
  device_id
}

fn flash_read_status(spi: &mut SpiType, cs: &mut CsPin) -> u8 {
  cs.set_low();
  let _ = spi.send(CMD_READ_STATUS);
  let status = spi_transfer(spi, 0x00);
  cs.set_high();
  status
}

fn flash_write_enable(spi: &mut SpiType, cs: &mut CsPin) {
  cs.set_low();
  let _ = spi.send(CMD_WRITE_ENABLE);
  cs.set_high();
}

fn flash_write_disable(spi: &mut SpiType, cs: &mut CsPin) {
  cs.set_low();
  let _ = spi.send(CMD_WRITE_DISABLE);
  cs.set_high();
}

fn flash_wait_busy(spi: &mut SpiType, cs: &mut CsPin) {
  while (flash_read_status(spi, cs) & STATUS_BUSY) != 0 {
    delay_ms(1);
  }
}

fn flash_sector_erase(spi: &mut SpiType, cs: &mut CsPin, address: u32) {
  flash_write_enable(spi, cs);

  cs.set_low();
  let _ = spi.send(CMD_SECTOR_ERASE);
  let _ = spi.send((address >> 16) as u8);
  let _ = spi.send((address >> 8) as u8);
  let _ = spi.send(address as u8);
  cs.set_high();

  flash_wait_busy(spi, cs);
}

fn flash_page_program(spi: &mut SpiType, cs: &mut CsPin, address: u32, data: &[u8]) {
  assert!(data.len() <= PAGE_SIZE);

  flash_write_enable(spi, cs);

  cs.set_low();
  let _ = spi.send(CMD_PAGE_PROGRAM);
  let _ = spi.send((address >> 16) as u8);
  let _ = spi.send((address >> 8) as u8);
  let _ = spi.send(address as u8);

  for &byte in data {
    let _ = spi.send(byte);
  }

  cs.set_high();
  flash_wait_busy(spi, cs);
}

fn flash_read_data(spi: &mut SpiType, cs: &mut CsPin, address: u32, buffer: &mut [u8]) {
  cs.set_low();
  let _ = spi.send(CMD_READ_DATA);
  let _ = spi.send((address >> 16) as u8);
  let _ = spi.send((address >> 8) as u8);
  let _ = spi.send(address as u8);

  for byte in buffer.iter_mut() {
    *byte = spi_transfer(spi, 0x00);
  }

  cs.set_high();
}

fn flash_fast_read(spi: &mut SpiType, cs: &mut CsPin, address: u32, buffer: &mut [u8]) {
  cs.set_low();
  let _ = spi.send(CMD_FAST_READ);
  let _ = spi.send((address >> 16) as u8);
  let _ = spi.send((address >> 8) as u8);
  let _ = spi.send(address as u8);
  let _ = spi.send(0x00); // 虚拟字节

  for byte in buffer.iter_mut() {
    *byte = spi_transfer(spi, 0x00);
  }

  cs.set_high();
}

fn flash_power_down(spi: &mut SpiType, cs: &mut CsPin) {
  cs.set_low();
  let _ = spi.send(CMD_POWER_DOWN);
  cs.set_high();
}

fn flash_release_power_down(spi: &mut SpiType, cs: &mut CsPin) {
  cs.set_low();
  let _ = spi.send(CMD_RELEASE_POWER_DOWN);
  cs.set_high();
  delay_ms(1); // 等待唤醒
}

fn spi_transfer(spi: &mut SpiType, data: u8) -> u8 {
  nb::block!(spi.send(data)).unwrap();
  nb::block!(spi.read()).unwrap()
}

fn error_indication(led: &mut PC13<Output<PushPull>>, error_code: u8) {
  // 根据错误代码闪烁 LED
  for _ in 0..error_code {
    led.set_low();
    delay_ms(300);
    led.set_high();
    delay_ms(300);
  }
  delay_ms(1000);
}

fn delay_ms(ms: u32) {
  // 简单的延时函数
  for _ in 0..(ms * 21000) {
    cortex_m::asm::nop();
  }
}

// Flash 高级功能
mod flash_advanced {
  use super::*;

  pub fn flash_block_erase_32k(spi: &mut SpiType, cs: &mut CsPin, address: u32) {
    flash_write_enable(spi, cs);

    cs.set_low();
    let _ = spi.send(CMD_BLOCK_ERASE_32K);
    let _ = spi.send((address >> 16) as u8);
    let _ = spi.send((address >> 8) as u8);
    let _ = spi.send(address as u8);
    cs.set_high();

    flash_wait_busy(spi, cs);
  }

  pub fn flash_block_erase_64k(spi: &mut SpiType, cs: &mut CsPin, address: u32) {
    flash_write_enable(spi, cs);

    cs.set_low();
    let _ = spi.send(CMD_BLOCK_ERASE_64K);
    let _ = spi.send((address >> 16) as u8);
    let _ = spi.send((address >> 8) as u8);
    let _ = spi.send(address as u8);
    cs.set_high();

    flash_wait_busy(spi, cs);
  }

  pub fn flash_chip_erase(spi: &mut SpiType, cs: &mut CsPin) {
    flash_write_enable(spi, cs);

    cs.set_low();
    let _ = spi.send(CMD_CHIP_ERASE);
    cs.set_high();

    flash_wait_busy(spi, cs);
  }

  pub fn flash_write_status(spi: &mut SpiType, cs: &mut CsPin, status: u8) {
    flash_write_enable(spi, cs);

    cs.set_low();
    let _ = spi.send(CMD_WRITE_STATUS);
    let _ = spi.send(status);
    cs.set_high();

    flash_wait_busy(spi, cs);
  }

  pub fn flash_write_large_data(
    spi: &mut SpiType,
    cs: &mut CsPin,
    start_address: u32,
    data: &[u8],
  ) {
    let mut address = start_address;
    let mut remaining = data.len();
    let mut offset = 0;

    while remaining > 0 {
      let page_offset = (address % PAGE_SIZE as u32) as usize;
      let write_size = core::cmp::min(PAGE_SIZE - page_offset, remaining);

      flash_page_program(spi, cs, address, &data[offset..offset + write_size]);

      address += write_size as u32;
      offset += write_size;
      remaining -= write_size;
    }
  }

  pub fn flash_verify_data(
    spi: &mut SpiType,
    cs: &mut CsPin,
    address: u32,
    expected_data: &[u8],
  ) -> bool {
    let mut read_buffer = [0u8; PAGE_SIZE];
    let mut verified = true;
    let mut remaining = expected_data.len();
    let mut offset = 0;
    let mut current_address = address;

    while remaining > 0 {
      let read_size = core::cmp::min(PAGE_SIZE, remaining);
      flash_read_data(spi, cs, current_address, &mut read_buffer[..read_size]);

      for i in 0..read_size {
        if read_buffer[i] != expected_data[offset + i] {
          verified = false;
          break;
        }
      }

      if !verified {
        break;
      }

      current_address += read_size as u32;
      offset += read_size;
      remaining -= read_size;
    }

    verified
  }
}
