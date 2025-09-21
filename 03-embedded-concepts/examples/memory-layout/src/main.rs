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

// 全局变量 (存储在 .data 段)
static mut GLOBAL_COUNTER: u32 = 42;

// 未初始化全局变量 (存储在 .bss 段)
static mut UNINITIALIZED_ARRAY: [u32; 100] = [0; 100];

// 常量数据 (存储在 .rodata 段)
const LOOKUP_TABLE: [u16; 16] = [
    0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100, 121, 144, 169, 196, 225
];

// 字符串常量 (存储在 .rodata 段)
const DEVICE_NAME: &str = "STM32F4 Memory Layout Demo";

#[entry]
fn main() -> ! {
    // 显示内存布局信息
    show_memory_layout();
    
    // 演示内存使用
    demonstrate_memory_usage();
    
    // 检查栈使用情况
    check_stack_usage();
    
    // 主循环 - 优化版本，避免过度CPU使用
    loop {
        // 执行一些轻量级操作
        unsafe {
            GLOBAL_COUNTER = GLOBAL_COUNTER.wrapping_add(1);
        }
        
        // 添加延迟以减少CPU负载
        cortex_m::asm::delay(1000000); // 约1ms延迟
        
        // 可选：进入低功耗模式
        cortex_m::asm::wfi(); // Wait For Interrupt
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
        
        // 在实际应用中，这些信息可以通过调试器查看
        // 或者通过 RTT、串口等方式输出
        
        // 这里我们只是访问这些地址以确保链接器正确设置
        let _ = (flash_start, flash_size);
        let _ = (ram_start, ram_size);
        let _ = (data_start, data_end, data_size, data_flash_addr);
        let _ = (bss_start, bss_end, bss_size);
        let _ = (heap_start, heap_end, heap_size);
        let _ = (stack_start, stack_end, stack_size);
    }
}

fn demonstrate_memory_usage() {
    // 访问全局变量 (.data 段)
    unsafe {
        GLOBAL_COUNTER += 1;
        let counter_value = GLOBAL_COUNTER;
        let _ = counter_value;
    }
    
    // 访问未初始化数组 (.bss 段)
    unsafe {
        UNINITIALIZED_ARRAY[0] = 0x12345678;
        UNINITIALIZED_ARRAY[99] = 0x87654321;
        let first = UNINITIALIZED_ARRAY[0];
        let last = UNINITIALIZED_ARRAY[99];
        let _ = (first, last);
    }
    
    // 访问常量数据 (.rodata 段)
    let square_of_5 = LOOKUP_TABLE[5]; // 应该是 25
    let device_name_len = DEVICE_NAME.len();
    let _ = (square_of_5, device_name_len);
    
    // 局部变量 (栈)
    let local_array: [u32; 50] = [0xDEADBEEF; 50];
    let local_sum: u32 = local_array.iter().sum();
    let _ = local_sum;
    
    // 递归调用测试栈使用
    recursive_function(5);
}

fn recursive_function(depth: u32) {
    if depth == 0 {
        return;
    }
    
    // 在栈上分配一些数据
    let local_data: [u32; 10] = [depth; 10];
    let sum: u32 = local_data.iter().sum();
    let _ = sum;
    
    // 递归调用
    recursive_function(depth - 1);
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
        
        // 检查栈溢出
        if current_sp < stack_end {
            // 栈溢出！在实际应用中应该触发错误处理
            panic!("Stack overflow detected!");
        }
        
        // 在实际应用中，可以通过调试接口输出这些信息
        let _ = (stack_used, stack_total, stack_free);
    }
}

// 内存访问安全检查函数
fn safe_read_u32(addr: u32) -> Result<u32, MemoryError> {
    // 检查地址对齐
    if addr % 4 != 0 {
        return Err(MemoryError::Misaligned);
    }
    
    // 检查地址范围
    if !is_valid_read_address(addr) {
        return Err(MemoryError::InvalidAddress);
    }
    
    unsafe {
        Ok(core::ptr::read_volatile(addr as *const u32))
    }
}

fn is_valid_read_address(addr: u32) -> bool {
    // Flash 区域 (只读)
    if addr >= 0x08000000 && addr <= 0x0807FFFF {
        return true;
    }
    
    // SRAM 区域 (读写)
    if addr >= 0x20000000 && addr <= 0x20017FFF {
        return true;
    }
    
    // 外设区域 (根据需要)
    if addr >= 0x40000000 && addr <= 0x5FFFFFFF {
        return true;
    }
    
    false
}

#[derive(Debug)]
enum MemoryError {
    Misaligned,
    InvalidAddress,
}

// 硬件故障异常处理
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    // 在实际应用中，这里应该记录错误信息并尝试恢复
    // 或者安全地重启系统
    
    // 获取故障地址 (如果可用)
    let cfsr = (*cortex_m::peripheral::SCB::ptr()).cfsr.read();
    let hfsr = (*cortex_m::peripheral::SCB::ptr()).hfsr.read();
    let mmfar = (*cortex_m::peripheral::SCB::ptr()).mmfar.read();
    let bfar = (*cortex_m::peripheral::SCB::ptr()).bfar.read();
    
    // 分析故障原因
    let _ = (cfsr, hfsr, mmfar, bfar, ef);
    
    // 在调试模式下可以设置断点查看这些值
    loop {
        cortex_m::asm::bkpt();
    }
}

// 内存管理故障异常处理
#[exception]
unsafe fn MemoryManagement() -> ! {
    // 内存管理单元 (MPU) 故障
    let mmfsr = (*cortex_m::peripheral::SCB::ptr()).cfsr.read() & 0xFF;
    let mmfar = (*cortex_m::peripheral::SCB::ptr()).mmfar.read();
    
    let _ = (mmfsr, mmfar);
    
    loop {
        cortex_m::asm::bkpt();
    }
}

// 总线故障异常处理
#[exception]
unsafe fn BusFault() -> ! {
    // 总线访问故障
    let bfsr = ((*cortex_m::peripheral::SCB::ptr()).cfsr.read() >> 8) & 0xFF;
    let bfar = (*cortex_m::peripheral::SCB::ptr()).bfar.read();
    
    let _ = (bfsr, bfar);
    
    loop {
        cortex_m::asm::bkpt();
    }
}

// 用法故障异常处理
#[exception]
unsafe fn UsageFault() -> ! {
    // 指令用法故障
    let ufsr = ((*cortex_m::peripheral::SCB::ptr()).cfsr.read() >> 16) & 0xFFFF;
    
    let _ = ufsr;
    
    loop {
        cortex_m::asm::bkpt();
    }
}