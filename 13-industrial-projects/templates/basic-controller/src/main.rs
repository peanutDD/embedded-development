#![no_std]
#![no_main]

// 基础控制器模板
// 
// 这是一个通用的嵌入式控制器模板，提供了常见的功能模块：
// - GPIO 控制
// - 串口通信
// - 定时器
// - ADC 采样
// - 状态机管理
// - 错误处理
// 
// 可以基于此模板快速开发各种控制器应用

use panic_halt as _;
use rtic::app;

use stm32f4xx_hal::{
    prelude::*,
};

// 全局内存分配器
extern crate linked_list_allocator;
use linked_list_allocator::LockedHeap;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

// 系统配置常量
const SYSTEM_FREQ: u32 = 84_000_000; // 84 MHz

// 控制器状态枚举
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControllerState {
    Idle,
    Running,
    Error,
    Maintenance,
}

// 系统错误类型
#[derive(Debug, Clone, Copy)]
pub enum SystemError {
    TimerError,
    InvalidState,
}

#[app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        state: ControllerState,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // 初始化内存分配器
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.lock().init(HEAP.as_ptr() as *mut u8, HEAP_SIZE) }

        let dp = ctx.device;

        // 配置时钟
        let rcc = dp.RCC.constrain();
        let _clocks = rcc.cfgr.sysclk(SYSTEM_FREQ.Hz()).freeze();

        (
            Shared {
                state: ControllerState::Idle,
            },
            Local {},
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}