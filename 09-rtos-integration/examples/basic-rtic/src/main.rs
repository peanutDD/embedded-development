#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm::delay;
use rtic::app;
use stm32f4xx_hal::{
    gpio::{gpioa::PA0, gpioc::PC13, Input, Output, PullUp, PushPull},
    pac,
    prelude::*,
};

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM2])]
mod app {
    use super::*;
    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    use rtic::time::duration::Milliseconds;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<84_000_000>; // 84 MHz

    #[shared]
    struct Shared {
        counter: u32,
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        button: PA0<Input<PullUp>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = ctx.device;
        let mut cp = ctx.core;

        // 配置时钟
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

        // 初始化单调时钟
        let mono = DwtSystick::new(&mut cp.DCB, cp.DWT, cp.SYST, clocks.hclk().0);

        // 配置GPIO
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        let led = gpioc.pc13.into_push_pull_output();
        let button = gpioa.pa0.into_pull_up_input();

        // 配置外部中断
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti = dp.EXTI;
        
        // 配置PA0为外部中断源
        syscfg.exti.select_gpio(&button, &mut exti);
        exti.imr1.modify(|_, w| w.mr0().set_bit());
        exti.ftsr1.modify(|_, w| w.tr0().set_bit()); // 下降沿触发

        // 启动LED闪烁任务
        blink_led::spawn().ok();

        (
            Shared { counter: 0 },
            Local { led, button },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led], shared = [counter])]
    fn blink_led(mut ctx: blink_led::Context) {
        ctx.local.led.toggle();
        
        ctx.shared.counter.lock(|counter| {
            *counter += 1;
        });

        // 每500ms闪烁一次
        blink_led::spawn_after(500.millis()).ok();
    }

    #[task(binds = EXTI0, local = [button], shared = [counter])]
    fn button_pressed(mut ctx: button_pressed::Context) {
        // 清除中断标志
        let exti = unsafe { &(*pac::EXTI::ptr()) };
        exti.pr1.write(|w| w.pr0().set_bit());

        // 简单的防抖动延时
        delay(1_000_000);

        // 检查按钮状态
        if ctx.local.button.is_low() {
            ctx.shared.counter.lock(|counter| {
                *counter = 0; // 重置计数器
            });
            
            // 快速闪烁3次表示按钮被按下
            rapid_blink::spawn().ok();
        }
    }

    #[task(local = [led])]
    fn rapid_blink(ctx: rapid_blink::Context) {
        // 快速闪烁3次
        for _ in 0..6 {
            ctx.local.led.toggle();
            delay(5_000_000); // 约60ms延时
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi(); // 等待中断
        }
    }
}