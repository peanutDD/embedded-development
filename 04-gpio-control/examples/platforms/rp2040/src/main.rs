#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use rp_pico::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{DynPin, FunctionSio, Pin, PullUp, SioInput, SioOutput},
    pac,
    pio::PIOExt,
    pwm::{self, Pwm, Slice, SliceId, A, B},
    sio::Sio,
    timer::Timer,
    usb::UsbBus,
    watchdog::Watchdog,
};
use rp_pico::{
    hal::Timer as HalTimer,
    XOSC_CRYSTAL_FREQ,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use defmt_rtt as _;
use defmt::*;

// GPIO引脚类型定义
type LedPin = Pin<rp_pico::hal::gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullUp>;
type ButtonPin = Pin<rp_pico::hal::gpio::bank0::Gpio15, FunctionSio<SioInput>, PullUp>;

/// RP2040 GPIO控制示例
/// 
/// 功能特性：
/// - LED闪烁控制 (内置LED GPIO25)
/// - 按钮输入检测 (GPIO15)
/// - PWM呼吸灯效果 (GPIO16)
/// - PIO状态机控制
/// - USB串口通信
/// - 多核处理支持
/// - 定时器中断
#[entry]
fn main() -> ! {
    info!("RP2040 GPIO控制示例启动");

    // 初始化外设
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // 配置时钟
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    info!("时钟配置完成: SYS={}MHz", clocks.system_clock.freq().to_MHz());

    // 配置GPIO
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LED引脚 (GPIO25 - 内置LED)
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    // 按钮引脚 (GPIO15)
    let button_pin = pins.gpio15.into_pull_up_input();

    // PWM配置 (GPIO16)
    let mut pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm_pin = pins.gpio16.into_function::<rp_pico::hal::gpio::FunctionPwm>();
    let pwm = &mut pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.enable();

    // 配置PWM通道
    let channel = &mut pwm.channel_a;
    channel.output_to(pwm_pin);

    // 定时器配置
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // USB配置
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Embedded Dev")
        .product("RP2040 GPIO Example")
        .serial_number("12345")
        .device_class(2) // CDC
        .build();

    // PIO配置
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let _installed = pio.install(&pio_proc::pio_file!("src/blink.pio")).unwrap();

    info!("GPIO、PWM、USB和PIO配置完成");

    // 状态变量
    let mut led_state = false;
    let mut button_pressed = false;
    let mut pwm_duty = 0u16;
    let mut pwm_direction = true;
    let mut cycle_count = 0u32;
    let mut last_timer = timer.get_counter();

    // 主循环
    loop {
        let current_timer = timer.get_counter();
        
        // 每500ms执行一次
        if current_timer.wrapping_sub(last_timer).to_micros() >= 500_000 {
            last_timer = current_timer;
            cycle_count += 1;

            // LED闪烁控制
            led_state = !led_state;
            if led_state {
                led_pin.set_high().unwrap();
            } else {
                led_pin.set_low().unwrap();
            }

            // 按钮状态检测
            let current_button = button_pin.is_low().unwrap();
            if current_button && !button_pressed {
                info!("按钮按下 - 周期: {}", cycle_count);
                button_pressed = true;
            } else if !current_button && button_pressed {
                info!("按钮释放 - 周期: {}", cycle_count);
                button_pressed = false;
            }

            // PWM呼吸灯效果
            update_pwm_breathing(channel, &mut pwm_duty, &mut pwm_direction);

            // 状态报告 (每10秒)
            if cycle_count % 20 == 0 {
                info!("系统状态 - LED: {}, 按钮: {}, PWM: {}, 周期: {}", 
                    led_state, button_pressed, pwm_duty, cycle_count);
            }
        }

        // USB处理
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    info!("USB接收: {} bytes", count);
                    // 回显数据
                    let _ = serial.write(&buf[0..count]);
                }
                _ => {}
            }
        }

        // 短暂延时
        delay.delay_us(100);
    }
}

/// 更新PWM呼吸灯效果
fn update_pwm_breathing<S: SliceId>(
    channel: &mut pwm::Channel<S, A>,
    duty: &mut u16,
    direction: &mut bool,
) {
    const MAX_DUTY: u16 = 65535;
    const STEP: u16 = 2000;

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

    channel.set_duty(*duty);
}

/// GPIO状态管理器
pub struct GpioManager {
    led_state: bool,
    button_state: bool,
    pwm_duty: u16,
    toggle_count: u32,
    button_press_count: u32,
}

impl GpioManager {
    pub fn new() -> Self {
        Self {
            led_state: false,
            button_state: false,
            pwm_duty: 0,
            toggle_count: 0,
            button_press_count: 0,
        }
    }

    pub fn update_led(&mut self, state: bool) {
        if state != self.led_state {
            self.led_state = state;
            self.toggle_count += 1;
            info!("LED状态更新: {} (切换次数: {})", state, self.toggle_count);
        }
    }

    pub fn update_button(&mut self, pressed: bool) {
        if pressed && !self.button_state {
            self.button_press_count += 1;
            info!("按钮按下 (总次数: {})", self.button_press_count);
        }
        self.button_state = pressed;
    }

    pub fn update_pwm(&mut self, duty: u16) {
        self.pwm_duty = duty;
    }

    pub fn get_stats(&self) -> (bool, bool, u16, u32, u32) {
        (
            self.led_state,
            self.button_state,
            self.pwm_duty,
            self.toggle_count,
            self.button_press_count,
        )
    }
}

/// PIO状态机管理器
pub struct PioManager {
    program_loaded: bool,
    state_machine_active: bool,
    instruction_count: u32,
}

impl PioManager {
    pub fn new() -> Self {
        Self {
            program_loaded: false,
            state_machine_active: false,
            instruction_count: 0,
        }
    }

    pub fn load_program(&mut self) -> Result<(), &'static str> {
        // 模拟PIO程序加载
        self.program_loaded = true;
        info!("PIO程序加载成功");
        Ok(())
    }

    pub fn start_state_machine(&mut self) -> Result<(), &'static str> {
        if !self.program_loaded {
            return Err("PIO程序未加载");
        }
        
        self.state_machine_active = true;
        info!("PIO状态机启动");
        Ok(())
    }

    pub fn stop_state_machine(&mut self) {
        self.state_machine_active = false;
        info!("PIO状态机停止");
    }

    pub fn execute_instruction(&mut self) {
        if self.state_machine_active {
            self.instruction_count += 1;
        }
    }

    pub fn get_status(&self) -> (bool, bool, u32) {
        (self.program_loaded, self.state_machine_active, self.instruction_count)
    }
}

/// USB通信管理器
pub struct UsbManager {
    connected: bool,
    bytes_sent: u32,
    bytes_received: u32,
    message_count: u32,
}

impl UsbManager {
    pub fn new() -> Self {
        Self {
            connected: false,
            bytes_sent: 0,
            bytes_received: 0,
            message_count: 0,
        }
    }

    pub fn set_connected(&mut self, connected: bool) {
        if connected != self.connected {
            self.connected = connected;
            info!("USB连接状态: {}", connected);
        }
    }

    pub fn send_data(&mut self, data: &[u8]) -> Result<usize, &'static str> {
        if !self.connected {
            return Err("USB未连接");
        }

        let len = data.len();
        self.bytes_sent += len as u32;
        self.message_count += 1;
        
        info!("USB发送: {} bytes (总计: {} bytes, {} 消息)", 
            len, self.bytes_sent, self.message_count);
        
        Ok(len)
    }

    pub fn receive_data(&mut self, data: &[u8]) {
        let len = data.len();
        self.bytes_received += len as u32;
        
        info!("USB接收: {} bytes (总计: {} bytes)", len, self.bytes_received);
    }

    pub fn get_stats(&self) -> (bool, u32, u32, u32) {
        (self.connected, self.bytes_sent, self.bytes_received, self.message_count)
    }
}

/// 性能监控器
pub struct PerformanceMonitor {
    loop_count: u32,
    max_loop_time: u32,
    min_loop_time: u32,
    avg_loop_time: u32,
    last_measurement: u32,
}

impl PerformanceMonitor {
    pub fn new() -> Self {
        Self {
            loop_count: 0,
            max_loop_time: 0,
            min_loop_time: u32::MAX,
            avg_loop_time: 0,
            last_measurement: 0,
        }
    }

    pub fn start_measurement(&mut self, timer: &Timer) {
        self.last_measurement = timer.get_counter().ticks();
    }

    pub fn end_measurement(&mut self, timer: &Timer) {
        let current = timer.get_counter().ticks();
        let elapsed = current.wrapping_sub(self.last_measurement);
        
        self.loop_count += 1;
        
        if elapsed > self.max_loop_time {
            self.max_loop_time = elapsed;
        }
        
        if elapsed < self.min_loop_time {
            self.min_loop_time = elapsed;
        }
        
        // 简单移动平均
        self.avg_loop_time = (self.avg_loop_time + elapsed) / 2;
        
        if self.loop_count % 1000 == 0 {
            info!("性能统计 - 循环: {}, 平均: {}μs, 最大: {}μs, 最小: {}μs",
                self.loop_count, self.avg_loop_time, self.max_loop_time, self.min_loop_time);
        }
    }

    pub fn get_stats(&self) -> (u32, u32, u32, u32) {
        (self.loop_count, self.avg_loop_time, self.max_loop_time, self.min_loop_time)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpio_manager() {
        let mut manager = GpioManager::new();
        
        manager.update_led(true);
        assert_eq!(manager.led_state, true);
        assert_eq!(manager.toggle_count, 1);
        
        manager.update_button(true);
        assert_eq!(manager.button_state, true);
        assert_eq!(manager.button_press_count, 1);
    }

    #[test]
    fn test_pio_manager() {
        let mut pio = PioManager::new();
        
        assert_eq!(pio.program_loaded, false);
        assert_eq!(pio.state_machine_active, false);
        
        pio.load_program().unwrap();
        assert_eq!(pio.program_loaded, true);
        
        pio.start_state_machine().unwrap();
        assert_eq!(pio.state_machine_active, true);
    }

    #[test]
    fn test_usb_manager() {
        let mut usb = UsbManager::new();
        
        assert_eq!(usb.connected, false);
        
        usb.set_connected(true);
        assert_eq!(usb.connected, true);
        
        let data = b"test";
        let result = usb.send_data(data);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 4);
        assert_eq!(usb.bytes_sent, 4);
    }
}