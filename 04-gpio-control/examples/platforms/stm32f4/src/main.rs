#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{Output, PushPull, Input, PullUp, Pin},
    pac,
    prelude::*,
    timer::{Timer, Event},
    pwm::{self, PwmChannels},
};
use nb::block;
use defmt_rtt as _;

// GPIO pin type aliases for STM32F407
type LedPin = Pin<'C', 13, Output<PushPull>>;
type ButtonPin = Pin<'A', 0, Input<PullUp>>;
type PwmPin = pwm::PwmChannel<pac::TIM2, 0>;

/// STM32F4 GPIO控制示例
/// 
/// 功能特性：
/// - LED闪烁控制
/// - 按钮输入检测
/// - PWM呼吸灯效果
/// - 定时器中断
/// - GPIO状态监控
#[entry]
fn main() -> ! {
    defmt::info!("STM32F4 GPIO控制示例启动");

    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

    defmt::info!("系统时钟配置完成: SYSCLK={}MHz", clocks.sysclk().raw() / 1_000_000);

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    // LED引脚 (PC13)
    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_high();

    // 按钮引脚 (PA0)
    let button = gpioa.pa0.into_pull_up_input();

    // PWM配置 (PA1 - TIM2_CH2)
    let pwm_pin = gpioa.pa1.into_alternate();
    let mut pwm = dp.TIM2.pwm_hz(pwm_pin, 1.kHz(), &clocks).split();
    pwm.0.enable();

    // 定时器配置
    let mut timer = Timer::new(dp.TIM3, &clocks).counter_hz();
    timer.start(2.Hz()).unwrap();

    defmt::info!("GPIO和定时器配置完成");

    // 状态变量
    let mut led_state = false;
    let mut button_pressed = false;
    let mut pwm_direction = true;
    let mut pwm_duty = 0u16;
    let mut cycle_count = 0u32;

    // 主循环
    loop {
        // 检查定时器事件
        if timer.wait().is_ok() {
            cycle_count += 1;
            
            // LED闪烁控制
            led_state = !led_state;
            if led_state {
                led.set_low();
            } else {
                led.set_high();
            }

            // 按钮状态检测
            let current_button = button.is_low();
            if current_button && !button_pressed {
                defmt::info!("按钮按下 - 周期: {}", cycle_count);
                button_pressed = true;
            } else if !current_button && button_pressed {
                defmt::info!("按钮释放 - 周期: {}", cycle_count);
                button_pressed = false;
            }

            // PWM呼吸灯效果
            update_breathing_led(&mut pwm.0, &mut pwm_duty, &mut pwm_direction);

            // 状态报告 (每10秒)
            if cycle_count % 20 == 0 {
                defmt::info!("系统状态 - LED: {}, 按钮: {}, PWM: {}, 周期: {}", 
                    led_state, button_pressed, pwm_duty, cycle_count);
            }
        }

        // 低功耗等待
        cortex_m::asm::wfi();
    }
}

/// 更新PWM呼吸灯效果
fn update_breathing_led(
    pwm: &mut PwmPin,
    duty: &mut u16,
    direction: &mut bool,
) {
    const MAX_DUTY: u16 = 1000;
    const STEP: u16 = 50;

    if *direction {
        if *duty + STEP >= MAX_DUTY {
            *duty = MAX_DUTY;
            *direction = false;
        } else {
            *duty += STEP;
        }
    } else {
        if *duty <= STEP {
            *duty = 0;
            *direction = true;
        } else {
            *duty -= STEP;
        }
    }

    pwm.set_duty(*duty);
}

/// GPIO状态监控器
pub struct GpioMonitor {
    led_pin: LedPin,
    button_pin: ButtonPin,
    led_state: bool,
    button_state: bool,
    toggle_count: u32,
}

impl GpioMonitor {
    pub fn new(led: LedPin, button: ButtonPin) -> Self {
        Self {
            led_pin: led,
            button_pin: button,
            led_state: false,
            button_state: false,
            toggle_count: 0,
        }
    }

    pub fn update(&mut self) {
        // 更新按钮状态
        let current_button = self.button_pin.is_low();
        if current_button != self.button_state {
            self.button_state = current_button;
            if current_button {
                self.toggle_led();
            }
        }
    }

    pub fn toggle_led(&mut self) {
        self.led_state = !self.led_state;
        self.toggle_count += 1;
        
        if self.led_state {
            self.led_pin.set_low();
        } else {
            self.led_pin.set_high();
        }
        
        defmt::info!("LED切换 - 状态: {}, 次数: {}", self.led_state, self.toggle_count);
    }

    pub fn get_stats(&self) -> (bool, bool, u32) {
        (self.led_state, self.button_state, self.toggle_count)
    }
}

/// PWM控制器
pub struct PwmController {
    pwm_channel: PwmPin,
    current_duty: u16,
    target_duty: u16,
    max_duty: u16,
}

impl PwmController {
    pub fn new(mut pwm: PwmPin, max_duty: u16) -> Self {
        pwm.enable();
        Self {
            pwm_channel: pwm,
            current_duty: 0,
            target_duty: 0,
            max_duty,
        }
    }

    pub fn set_duty(&mut self, duty: u16) {
        let clamped_duty = duty.min(self.max_duty);
        self.target_duty = clamped_duty;
        self.pwm_channel.set_duty(clamped_duty);
        self.current_duty = clamped_duty;
    }

    pub fn fade_to(&mut self, target: u16, step: u16) {
        let clamped_target = target.min(self.max_duty);
        self.target_duty = clamped_target;
        
        if self.current_duty < self.target_duty {
            self.current_duty = (self.current_duty + step).min(self.target_duty);
        } else if self.current_duty > self.target_duty {
            self.current_duty = self.current_duty.saturating_sub(step).max(self.target_duty);
        }
        
        self.pwm_channel.set_duty(self.current_duty);
    }

    pub fn get_duty(&self) -> u16 {
        self.current_duty
    }

    pub fn is_fading(&self) -> bool {
        self.current_duty != self.target_duty
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pwm_controller() {
        // 注意：这些测试需要在目标硬件上运行
        // 这里只是展示测试结构
    }

    #[test]
    fn test_gpio_monitor() {
        // GPIO监控器测试
    }
}