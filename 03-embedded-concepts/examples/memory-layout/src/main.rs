#![no_std]
#![no_main]

use cortex_m_rt::{entry, exception, ExceptionFrame};
use panic_halt as _;

// 链接器符号引用
extern "C" {
  static mut _sdata: u32;
  static mut _edata: u32;
  static _sidata: u32;
  static mut _sbss: u32;
  static mut _ebss: u32;
  static _sheap: u32;
  static _eheap: u32;
  static _sstack: u32;
  static _estack: u32;
}

// 全局变量
static mut GLOBAL_COUNTER: u32 = 42;

// 常量数据
const LOOKUP_TABLE: [u16; 16] = [0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100, 121, 144, 169, 196, 225];

// 字符串常量
const DEVICE_NAME: &str = "STM32F4 Memory Layout Demo";

#[entry]
fn main() -> ! {
  // 显示内存布局信息
  show_memory_layout();

  // 演示内存使用
  demonstrate_memory_usage();

  // 检查栈使用情况
  check_stack_usage();

  // 主循环
  loop {
    unsafe {
      GLOBAL_COUNTER = GLOBAL_COUNTER.wrapping_add(1);
    }
    
    cortex_m::asm::delay(1000000);
  }
}

fn show_memory_layout() {
  unsafe {
    // Flash 区域信息
    let flash_start = 0x08000000u32;
    let flash_size = 512 * 1024; // 512KB

    // RAM 区域信息
    let ram_start = 0x20000000u32;
    let ram_size = 96 * 1024; // 96KB

    // 数据段信息
    let data_start = &_sdata as *const u32 as u32;
    let data_end = &_edata as *const u32 as u32;
    let data_size = data_end - data_start;
    let data_flash_addr = &_sidata as *const u32 as u32;

    // BSS 段信息
    let bss_start = &_sbss as *const u32 as u32;
    let bss_end = &_ebss as *const u32 as u32;
    let bss_size = bss_end - bss_start;

    // 堆信息
    let heap_start = &_sheap as *const u32 as u32;
    let heap_end = &_eheap as *const u32 as u32;
    let heap_size = heap_end - heap_start;

    // 栈信息
    let stack_start = &_sstack as *const u32 as u32;
    let stack_end = &_estack as *const u32 as u32;
    let stack_size = stack_start - stack_end;

    // 避免未使用变量警告
    let _ = (flash_start, flash_size, ram_start, ram_size);
    let _ = (data_start, data_end, data_size, data_flash_addr);
    let _ = (bss_start, bss_end, bss_size);
    let _ = (heap_start, heap_end, heap_size);
    let _ = (stack_start, stack_end, stack_size);
  }
}

fn demonstrate_memory_usage() {
  // 访问全局变量
  unsafe {
    GLOBAL_COUNTER = GLOBAL_COUNTER.wrapping_add(1);
  }

  // 访问常量数据
  let square_of_5 = LOOKUP_TABLE[5]; // 应该是 25
  let device_name_len = DEVICE_NAME.len();
  
  // 避免未使用变量警告
  let _ = (square_of_5, device_name_len);
}

fn check_stack_usage() {
  // 获取当前栈指针
  let current_sp: u32;
  unsafe {
    core::arch::asm!("mov {}, sp", out(reg) current_sp);
  }

  unsafe {
    let stack_start = &_sstack as *const u32 as u32;
    let stack_end = &_estack as *const u32 as u32;

    // 计算栈使用量
    let stack_used = stack_start - current_sp;
    let stack_total = stack_start - stack_end;
    let stack_free = current_sp - stack_end;

    // 避免未使用变量警告
    let _ = (stack_used, stack_total, stack_free);
  }
}

// 异常处理
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
  let _ = ef;
  loop {
    cortex_m::asm::bkpt();
  }
}

// 默认异常处理程序
#[exception]
unsafe fn DefaultHandler(irqn: i16) -> ! {
  let _ = irqn;
  loop {
    cortex_m::asm::bkpt();
  }
}
