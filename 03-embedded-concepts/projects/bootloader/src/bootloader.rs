#![no_std]
#![no_main]

use cortex_m::{self, asm};
use cortex_m_rt::{entry, exception, ExceptionFrame};
use crc::{Crc, CRC_32_ISO_HDLC};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{gpioa::PA0, gpioc::PC13, Input, Output, PullUp, PushPull},
  pac,
  prelude::*,
};

// 应用程序起始地址
const APP_START_ADDR: u32 = 0x08008000;
const APP_MAX_SIZE: u32 = 480 * 1024; // 480KB

// 应用程序头部结构
#[repr(C)]
struct AppHeader {
  magic: u32,       // 魔数，用于验证
  version: u32,     // 应用程序版本
  size: u32,        // 应用程序大小
  crc32: u32,       // CRC32 校验和
  entry_point: u32, // 入口点地址
}

const APP_MAGIC: u32 = 0xDEADBEEF;

#[entry]
fn main() -> ! {
  // 初始化外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let _clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

  // 配置 GPIO
  let gpioa = dp.GPIOA.split();
  let gpioc = dp.GPIOC.split();

  // 配置状态 LED (PC13)
  let mut status_led = gpioc.pc13.into_push_pull_output();

  // 配置用户按钮 (PA0)
  let user_button = gpioa.pa0.into_pull_up_input();

  // Bootloader 启动指示
  bootloader_startup_sequence(&mut status_led);

  // 检查是否需要进入 bootloader 模式
  if should_enter_bootloader_mode(&user_button) {
    // 进入 bootloader 模式
    enter_bootloader_mode(&mut status_led);
  } else {
    // 尝试启动应用程序
    match try_boot_application() {
      Ok(_) => {
        // 不应该到达这里
        panic!("Application returned unexpectedly");
      }
      Err(e) => {
        // 应用程序启动失败，进入 bootloader 模式
        handle_boot_error(e, &mut status_led);
        enter_bootloader_mode(&mut status_led);
      }
    }
  }
}

fn bootloader_startup_sequence(led: &mut PC13<Output<PushPull>>) {
  // 快速闪烁 LED 表示 bootloader 启动
  for _ in 0..6 {
    led.set_low();
    delay_ms(100);
    led.set_high();
    delay_ms(100);
  }
}

fn should_enter_bootloader_mode(button: &PA0<Input<PullUp>>) -> bool {
  // 检查用户按钮是否被按下
  // 按钮按下时为低电平
  if button.is_low() {
    // 防抖延时
    delay_ms(50);
    return button.is_low();
  }

  // 检查应用程序是否有效
  !is_application_valid()
}

fn is_application_valid() -> bool {
  unsafe {
    // 检查应用程序头部
    let header_ptr = APP_START_ADDR as *const AppHeader;
    let header = core::ptr::read_volatile(header_ptr);

    // 验证魔数
    if header.magic != APP_MAGIC {
      return false;
    }

    // 验证大小
    if header.size == 0 || header.size > APP_MAX_SIZE {
      return false;
    }

    // 验证入口点
    if header.entry_point < APP_START_ADDR || header.entry_point >= APP_START_ADDR + header.size {
      return false;
    }

    // 验证 CRC32
    let app_data = core::slice::from_raw_parts(
      (APP_START_ADDR + core::mem::size_of::<AppHeader>() as u32) as *const u8,
      (header.size - core::mem::size_of::<AppHeader>() as u32) as usize,
    );

    let crc = Crc::<u32>::new(&CRC_32_ISO_HDLC);
    let calculated_crc = crc.checksum(app_data);

    header.crc32 == calculated_crc
  }
}

fn try_boot_application() -> Result<(), BootError> {
  unsafe {
    // 读取应用程序头部
    let header_ptr = APP_START_ADDR as *const AppHeader;
    let header = core::ptr::read_volatile(header_ptr);

    // 再次验证应用程序
    if !is_application_valid() {
      return Err(BootError::InvalidApplication);
    }

    // 获取应用程序的向量表
    let app_vector_table = APP_START_ADDR as *const u32;
    let app_stack_ptr = core::ptr::read_volatile(app_vector_table);
    let app_reset_vector = core::ptr::read_volatile(app_vector_table.offset(1));

    // 验证栈指针和复位向量
    if app_stack_ptr < 0x20000000 || app_stack_ptr > 0x20018000 {
      return Err(BootError::InvalidStackPointer);
    }

    if app_reset_vector < APP_START_ADDR || app_reset_vector >= APP_START_ADDR + header.size {
      return Err(BootError::InvalidResetVector);
    }

    // 禁用所有中断
    cortex_m::interrupt::disable();

    // 重新配置向量表偏移
    let scb = &mut *cortex_m::peripheral::SCB::ptr();
    scb.vtor.write(APP_START_ADDR);

    // 设置栈指针
    asm!("msr msp, {}", in(reg) app_stack_ptr);

    // 跳转到应用程序
    let app_entry: extern "C" fn() -> ! = core::mem::transmute(app_reset_vector);
    app_entry();
  }
}

fn enter_bootloader_mode(led: &mut PC13<Output<PushPull>>) -> ! {
  // 进入 bootloader 模式
  // 在实际应用中，这里会实现固件更新功能

  loop {
    // 慢速闪烁表示 bootloader 模式
    led.set_low();
    delay_ms(500);
    led.set_high();
    delay_ms(500);

    // 在这里可以实现：
    // 1. 串口通信接收新固件
    // 2. USB DFU 模式
    // 3. 网络更新
    // 4. SD 卡更新等

    // 模拟固件更新检查
    if check_for_firmware_update() {
      perform_firmware_update(led);
    }
  }
}

fn check_for_firmware_update() -> bool {
  // 模拟检查固件更新
  // 在实际应用中，这里会检查：
  // 1. 串口是否有数据
  // 2. USB 是否连接
  // 3. 网络是否有更新
  // 4. SD 卡是否插入等

  false // 暂时返回 false
}

fn perform_firmware_update(led: &mut PC13<Output<PushPull>>) {
  // 执行固件更新
  // 1. 解锁 Flash
  flash::unlock();

  // 2. 擦除应用程序区域 (扇区 2 到 7)
  for sector in 2..=7 {
    led.set_low(); // 擦除时 LED 亮
    delay_ms(50);
    if let Err(_) = flash::erase_sector(sector) {
      // 擦除失败，进入错误处理
      // 在实际应用中，这里应该有更完善的错误报告机制
      loop {
        led.set_high();
        delay_ms(100);
        led.set_low();
        delay_ms(100);
      }
    }
    led.set_high(); // 擦除完成 LED 灭
    delay_ms(50);
  }

  // 3. 模拟接收新固件数据并写入 Flash
  // 实际应用中，这里会从通信接口（如串口、USB、网络）接收数据
  // 为了演示，我们写入一些模拟数据
  let dummy_firmware_data: [u32; 10] = [
    0x11223344, 0x55667788, 0xAABBCCDD, 0xEEFF0011, 0x22334455,
    0x66778899, 0xAABBCCDD, 0xEEFF0011, 0x12345678, 0x9ABCDEF0,
  ];
  let mut current_addr = APP_START_ADDR;
  for &word in dummy_firmware_data.iter() {
    if let Err(_) = flash::program_word(current_addr, word) {
      // 写入失败，进入错误处理
      loop {
        led.set_high();
        delay_ms(100);
        led.set_low();
        delay_ms(100);
      }
    }
    current_addr += 4; // 每次写入一个字 (4 字节)
  }

  // 4. 锁定 Flash
  flash::lock();

  // 快速闪烁表示正在更新
  for _ in 0..20 {
    led.set_low();
    delay_ms(50);
    led.set_high();
    delay_ms(50);
  }

  // 模拟更新完成，重启系统
  system_reset();
}

fn handle_boot_error(error: BootError, led: &mut PC13<Output<PushPull>>) {
  // 根据错误类型闪烁不同的模式
  match error {
    BootError::InvalidApplication => {
      // 3 次短闪
      for _ in 0..3 {
        led.set_low();
        delay_ms(200);
        led.set_high();
        delay_ms(200);
      }
    }
    BootError::InvalidStackPointer => {
      // 4 次短闪
      for _ in 0..4 {
        led.set_low();
        delay_ms(200);
        led.set_high();
        delay_ms(200);
      }
    }
    BootError::InvalidResetVector => {
      // 5 次短闪
      for _ in 0..5 {
        led.set_low();
        delay_ms(200);
        led.set_high();
        delay_ms(200);
      }
    }
  }

  delay_ms(1000);
}

fn system_reset() -> ! {
  unsafe {
    // 软件复位
    let scb = &mut *cortex_m::peripheral::SCB::ptr();
    scb.aircr.write(
      (0x05FA << 16) | // VECTKEY
            (1 << 2), // SYSRESETREQ
    );
  }

  loop {
    cortex_m::asm::nop();
  }
}

fn delay_ms(ms: u32) {
  // 简单的延时函数
  // 在实际应用中应该使用定时器
  for _ in 0..(ms * 21000) {
    cortex_m::asm::nop();
  }
}

#[derive(Debug)]
enum BootError {
  InvalidApplication,
  InvalidStackPointer,
  InvalidResetVector,
}

// Flash 操作函数
mod flash {
  use stm32f4xx_hal::pac::FLASH;

  pub fn unlock() {
    unsafe {
      let flash = &*FLASH::ptr();

      // 解锁 Flash
      flash.keyr.write(|w| w.key().bits(0x45670123));
      flash.keyr.write(|w| w.key().bits(0xCDEF89AB));
    }
  }

  pub fn lock() {
    unsafe {
      let flash = &*FLASH::ptr();
      flash.cr.modify(|_, w| w.lock().set_bit());
    }
  }

  pub fn erase_sector(sector: u8) -> Result<(), FlashError> {
    unsafe {
      let flash = &*FLASH::ptr();

      // 等待 Flash 操作完成
      while flash.sr.read().bsy().bit_is_set() {}

      // 配置扇区擦除
      flash
        .cr
        .modify(|_, w| w.ser().set_bit().snb().bits(sector).strt().set_bit());

      // 等待操作完成
      while flash.sr.read().bsy().bit_is_set() {}

      // 检查错误
      let sr = flash.sr.read();
      if sr.pgaerr().bit_is_set() || sr.pgperr().bit_is_set() || sr.wrperr().bit_is_set() {
        return Err(FlashError::ProgramError);
      }

      // 清除扇区擦除位
      flash.cr.modify(|_, w| w.ser().clear_bit());

      Ok(())
    }
  }

  pub fn program_word(addr: u32, data: u32) -> Result<(), FlashError> {
    unsafe {
      let flash = &*FLASH::ptr();

      // 等待 Flash 操作完成
      while flash.sr.read().bsy().bit_is_set() {}

      // 启用编程
      flash.cr.modify(|_, w| w.pg().set_bit().psize().bits(0b10)); // 32-bit

      // 写入数据
      core::ptr::write_volatile(addr as *mut u32, data);

      // 等待操作完成
      while flash.sr.read().bsy().bit_is_set() {}

      // 检查错误
      let sr = flash.sr.read();
      if sr.pgaerr().bit_is_set() || sr.pgperr().bit_is_set() || sr.wrperr().bit_is_set() {
        return Err(FlashError::ProgramError);
      }

      // 禁用编程
      flash.cr.modify(|_, w| w.pg().clear_bit());

      Ok(())
    }
  }

  #[derive(Debug)]
  pub enum FlashError {
    ProgramError,
  }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
  // Bootloader 中的硬件故障处理
  let _ = ef;

  // 尝试重启系统
  system_reset();
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
  // 未处理的中断
  let _ = irqn;
}
