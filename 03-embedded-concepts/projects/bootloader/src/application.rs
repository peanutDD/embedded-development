#![no_std]
#![no_main]

use cortex_m_rt::{entry, exception, ExceptionFrame};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{gpioc::PC13, Output, PushPull},
  pac,
  prelude::*,
};

// 应用程序头部 (必须位于程序开始处)
#[link_section = ".app_header"]
#[no_mangle]
static APP_HEADER: AppHeader = AppHeader {
  magic: 0xDEADBEEF,
  version: 0x00010000, // 版本 1.0.0
  size: 0,             // 构建时填充
  crc32: 0,            // 构建时计算
  entry_point: 0,      // 构建时填充
};

#[repr(C)]
struct AppHeader {
  magic: u32,
  version: u32,
  size: u32,
  crc32: u32,
  entry_point: u32,
}

#[entry]
fn main() -> ! {
  // 应用程序主函数
  let dp = pac::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let _clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

  // 配置 GPIO
  let gpioc = dp.GPIOC.split();
  let mut led = gpioc.pc13.into_push_pull_output();

  // 应用程序启动指示
  application_startup_sequence(&mut led);

  // 主循环
  let mut counter = 0u32;
  loop {
    // 应用程序逻辑
    run_application_logic(&mut led, &mut counter);

    // 延时
    delay_ms(1000);
  }
}

fn application_startup_sequence(led: &mut PC13<Output<PushPull>>) {
  // 长闪烁表示应用程序启动
  for _ in 0..3 {
    led.set_low();
    delay_ms(500);
    led.set_high();
    delay_ms(500);
  }
}

fn run_application_logic(led: &mut PC13<Output<PushPull>>, counter: &mut u32) {
  *counter = counter.wrapping_add(1);

  // 根据计数器值控制 LED
  if *counter % 2 == 0 {
    led.set_low(); // LED 亮
  } else {
    led.set_high(); // LED 灭
  }

  // 模拟应用程序工作
  for _ in 0..1000 {
    cortex_m::asm::nop();
  }

  // 每 10 次循环执行特殊操作
  if *counter % 10 == 0 {
    perform_special_task(led);
  }
}

fn perform_special_task(led: &mut PC13<Output<PushPull>>) {
  // 快速闪烁表示特殊任务
  for _ in 0..5 {
    led.set_low();
    delay_ms(100);
    led.set_high();
    delay_ms(100);
  }
}

fn delay_ms(ms: u32) {
  // 简单的延时函数
  for _ in 0..(ms * 21000) {
    cortex_m::asm::nop();
  }
}

// 应用程序版本信息
#[link_section = ".app_info"]
#[no_mangle]
static APP_INFO: AppInfo = AppInfo {
  name: b"Sample Application\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
  version_major: 1,
  version_minor: 0,
  version_patch: 0,
  build_date: b"2024-01-01\0\0\0\0\0\0",
  build_time: b"12:00:00\0",
};

#[repr(C)]
struct AppInfo {
  name: &'static [u8; 32],
  version_major: u16,
  version_minor: u16,
  version_patch: u16,
  build_date: &'static [u8; 16],
  build_time: &'static [u8; 9],
}

// 应用程序配置
#[link_section = ".app_config"]
#[no_mangle]
static APP_CONFIG: AppConfig = AppConfig {
  led_blink_interval: 1000,  // ms
  special_task_interval: 10, // 每 10 次循环
  debug_enabled: true,
  reserved: [0; 13],
};

#[repr(C)]
struct AppConfig {
  led_blink_interval: u32,
  special_task_interval: u32,
  debug_enabled: bool,
  reserved: [u8; 13], // 填充到 32 字节
}

// 应用程序状态
static mut APP_STATE: AppState = AppState {
  uptime_seconds: 0,
  loop_count: 0,
  error_count: 0,
  last_error: 0,
};

#[repr(C)]
struct AppState {
  uptime_seconds: u32,
  loop_count: u32,
  error_count: u32,
  last_error: u32,
}

// 错误处理
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
  // 应用程序硬件故障处理
  APP_STATE.error_count += 1;
  APP_STATE.last_error = 0x1001; // 硬件故障错误码

  let _ = ef;

  // 尝试恢复或重启
  // 在实际应用中，可能需要保存错误信息到非易失性存储

  loop {
    cortex_m::asm::bkpt();
  }
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
  // 未处理的中断
  APP_STATE.error_count += 1;
  APP_STATE.last_error = 0x2000 | (irqn as u32 & 0xFFFF);

  // 记录未处理的中断
}
