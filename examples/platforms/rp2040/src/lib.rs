#![no_std]
#![no_main]

use panic_probe as _;

// 重新导出常用类型和特征
pub use rp2040_hal as hal;
pub use rp_pico as pico;
pub use embedded_hal;
pub use cortex_m;
pub use cortex_m_rt;

// GPIO相关
pub mod gpio {
    use crate::hal::{
        gpio::{
            bank0::{Gpio25, Gpio2, Gpio3, Gpio4, Gpio5},
            FunctionSio, Pin, PullDown, PullUp, SioInput, SioOutput,
        },
        sio::Sio,
    };
    
    // 常用引脚类型定义
    pub type LedPin = Pin<Gpio25, FunctionSio<SioOutput>, PullDown>;
    pub type ButtonPin = Pin<Gpio2, FunctionSio<SioInput>, PullUp>;
    pub type SensorPowerPin = Pin<Gpio3, FunctionSio<SioOutput>, PullDown>;
    pub type StatusPin = Pin<Gpio4, FunctionSio<SioOutput>, PullDown>;
    pub type InputPin = Pin<Gpio5, FunctionSio<SioInput>, PullDown>;
    
    // GPIO控制器
    pub struct GpioController {
        led: Option<LedPin>,
        button: Option<ButtonPin>,
        sensor_power: Option<SensorPowerPin>,
        status: Option<StatusPin>,
    }
    
    impl GpioController {
        pub fn new() -> Self {
            Self {
                led: None,
                button: None,
                sensor_power: None,
                status: None,
            }
        }
        
        pub fn init_led(&mut self, pin: Pin<Gpio25, hal::gpio::FunctionNull, PullDown>) {
            self.led = Some(pin.into_push_pull_output());
        }
        
        pub fn init_button(&mut self, pin: Pin<Gpio2, hal::gpio::FunctionNull, PullDown>) {
            self.button = Some(pin.into_pull_up_input());
        }
        
        pub fn led_on(&mut self) {
            if let Some(ref mut led) = self.led {
                led.set_high().ok();
            }
        }
        
        pub fn led_off(&mut self) {
            if let Some(ref mut led) = self.led {
                led.set_low().ok();
            }
        }
        
        pub fn led_toggle(&mut self) {
            if let Some(ref mut led) = self.led {
                led.toggle().ok();
            }
        }
        
        pub fn is_button_pressed(&self) -> bool {
            if let Some(ref button) = self.button {
                button.is_low().unwrap_or(false)
            } else {
                false
            }
        }
        
        pub fn sensor_power_on(&mut self) {
            if let Some(ref mut power) = self.sensor_power {
                power.set_high().ok();
            }
        }
        
        pub fn sensor_power_off(&mut self) {
            if let Some(ref mut power) = self.sensor_power {
                power.set_low().ok();
            }
        }
    }
}

// 系统初始化
pub mod system {
    use crate::hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        sio::Sio,
        watchdog::Watchdog,
    };
    
    pub struct SystemConfig {
        pub external_xtal_freq_hz: u32,
        pub sys_freq_hz: u32,
        pub usb_freq_hz: u32,
    }
    
    impl Default for SystemConfig {
        fn default() -> Self {
            Self {
                external_xtal_freq_hz: 12_000_000,
                sys_freq_hz: 125_000_000,
                usb_freq_hz: 48_000_000,
            }
        }
    }
    
    pub struct SystemPeripherals {
        pub pac: pac::Peripherals,
        pub core: pac::CorePeripherals,
        pub sio: Sio,
        pub pins: crate::pico::Pins,
        pub clocks: crate::hal::clocks::ClocksManager,
    }
    
    pub fn init_system(config: SystemConfig) -> SystemPeripherals {
        let mut pac = pac::Peripherals::take().unwrap();
        let core = pac::CorePeripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let sio = Sio::new(pac.SIO);
        
        // 时钟配置
        let clocks = init_clocks_and_plls(
            config.external_xtal_freq_hz,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();
        
        // GPIO引脚初始化
        let pins = crate::pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        
        SystemPeripherals {
            pac,
            core,
            sio,
            pins,
            clocks,
        }
    }
}

// PIO (Programmable I/O) 支持
pub mod pio {
    use crate::hal::{
        pio::{PIOExt, StateMachine, UninitStateMachine, PIO, SM0},
        pac::PIO0,
    };
    
    pub struct PioController {
        pio: PIO<PIO0>,
        sm0: Option<StateMachine<(PIO0, SM0), hal::pio::Stopped>>,
    }
    
    impl PioController {
        pub fn new(pio0: PIO0, resets: &mut crate::hal::pac::RESETS) -> Self {
            let (pio, sm0, _, _, _) = pio0.split(resets);
            
            Self {
                pio,
                sm0: Some(sm0),
            }
        }
        
        pub fn load_program(&mut self, program: &[u16]) -> Result<(), PioError> {
            // 加载PIO程序到指令内存
            if program.len() > 32 {
                return Err(PioError::ProgramTooLarge);
            }
            
            // 这里应该实现实际的程序加载逻辑
            defmt::info!("PIO程序已加载，指令数: {}", program.len());
            Ok(())
        }
        
        pub fn start_state_machine(&mut self) -> Result<(), PioError> {
            if let Some(sm) = self.sm0.take() {
                let _running_sm = sm.start();
                defmt::info!("PIO状态机已启动");
                Ok(())
            } else {
                Err(PioError::StateMachineNotAvailable)
            }
        }
    }
    
    #[derive(Debug)]
    pub enum PioError {
        ProgramTooLarge,
        StateMachineNotAvailable,
        InvalidConfiguration,
    }
}

// 多核支持
pub mod multicore {
    use crate::hal::{
        multicore::{Multicore, Stack},
        pac::CorePeripherals,
        sio::Sio,
    };
    use cortex_m::interrupt;
    
    // Core 1栈空间
    static mut CORE1_STACK: Stack<4096> = Stack::new();
    
    pub struct MulticoreManager {
        multicore: Multicore,
    }
    
    impl MulticoreManager {
        pub fn new(pac: crate::hal::pac::Peripherals, sio: Sio) -> Self {
            let multicore = Multicore::new(&mut pac.PSM, &mut pac.PPB, sio.fifo);
            
            Self { multicore }
        }
        
        pub fn spawn_core1<F>(&mut self, entry: F) -> Result<(), MulticoreError>
        where
            F: FnOnce() -> ! + Send + 'static,
        {
            let core1 = self.multicore.cores()[1].spawn(unsafe { &mut CORE1_STACK.mem }, entry);
            
            match core1 {
                Ok(_) => {
                    defmt::info!("Core 1已启动");
                    Ok(())
                }
                Err(_) => Err(MulticoreError::SpawnFailed),
            }
        }
        
        pub fn send_to_core1(&mut self, data: u32) {
            self.multicore.cores()[0].fifo.write_blocking(data);
        }
        
        pub fn receive_from_core1(&mut self) -> Option<u32> {
            self.multicore.cores()[0].fifo.read()
        }
    }
    
    #[derive(Debug)]
    pub enum MulticoreError {
        SpawnFailed,
        CommunicationError,
    }
    
    // Core 1入口函数示例
    pub fn core1_task() -> ! {
        defmt::info!("Core 1任务启动");
        
        let core = unsafe { CorePeripherals::steal() };
        let mut counter = 0u32;
        
        loop {
            counter += 1;
            
            if counter % 1000000 == 0 {
                defmt::info!("Core 1心跳: {}", counter / 1000000);
            }
            
            cortex_m::asm::nop();
        }
    }
}

// DMA支持
pub mod dma {
    use crate::hal::{
        dma::{DMAExt, Channel, CH0},
        pac::DMA,
    };
    
    pub struct DmaController {
        dma: crate::hal::dma::DMA,
        ch0: Option<Channel<CH0>>,
    }
    
    impl DmaController {
        pub fn new(dma: DMA, resets: &mut crate::hal::pac::RESETS) -> Self {
            let dma = dma.split(resets);
            let ch0 = dma.ch0;
            
            Self {
                dma,
                ch0: Some(ch0),
            }
        }
        
        pub fn transfer_memory(&mut self, src: &[u8], dst: &mut [u8]) -> Result<(), DmaError> {
            if src.len() != dst.len() {
                return Err(DmaError::SizeMismatch);
            }
            
            if let Some(ch) = self.ch0.take() {
                // 这里应该实现实际的DMA传输
                defmt::info!("DMA传输: {} 字节", src.len());
                
                // 模拟传输
                for (i, &byte) in src.iter().enumerate() {
                    dst[i] = byte;
                }
                
                self.ch0 = Some(ch);
                Ok(())
            } else {
                Err(DmaError::ChannelNotAvailable)
            }
        }
    }
    
    #[derive(Debug)]
    pub enum DmaError {
        SizeMismatch,
        ChannelNotAvailable,
        TransferFailed,
    }
}

// 错误处理
#[derive(Debug, Clone, Copy)]
pub enum PlatformError {
    InitializationFailed,
    CommunicationError,
    TimeoutError,
    InvalidParameter,
    HardwareFault,
    PioError,
    MulticoreError,
    DmaError,
}

// 实用工具
pub mod utils {
    use cortex_m::asm;
    use crate::hal::timer::{Timer, Instant};
    
    /// 延时函数（基于CPU周期）
    pub fn delay_cycles(cycles: u32) {
        for _ in 0..cycles {
            asm::nop();
        }
    }
    
    /// 毫秒延时（基于125MHz系统时钟）
    pub fn delay_ms(ms: u32) {
        delay_cycles(ms * 125_000);
    }
    
    /// 微秒延时（基于125MHz系统时钟）
    pub fn delay_us(us: u32) {
        delay_cycles(us * 125);
    }
    
    /// 精确延时（使用定时器）
    pub struct PreciseDelay {
        timer: Timer,
    }
    
    impl PreciseDelay {
        pub fn new(timer: Timer) -> Self {
            Self { timer }
        }
        
        pub fn delay_ms(&mut self, ms: u32) {
            let start = self.timer.get_counter();
            let target = start.wrapping_add(ms * 1000);
            
            while self.timer.get_counter().wrapping_sub(start) < (ms * 1000) {
                asm::nop();
            }
        }
        
        pub fn delay_us(&mut self, us: u32) {
            let start = self.timer.get_counter();
            
            while self.timer.get_counter().wrapping_sub(start) < us {
                asm::nop();
            }
        }
    }
}

// 调试支持
pub mod debug {
    use defmt_rtt as _;
    
    // 调试宏
    #[macro_export]
    macro_rules! debug_print {
        ($($arg:tt)*) => {
            defmt::info!($($arg)*);
        };
    }
    
    #[macro_export]
    macro_rules! debug_error {
        ($($arg:tt)*) => {
            defmt::error!($($arg)*);
        };
    }
    
    #[macro_export]
    macro_rules! debug_warn {
        ($($arg:tt)*) => {
            defmt::warn!($($arg)*);
        };
    }
}

// 中断处理
pub mod interrupts {
    use cortex_m::interrupt;
    
    /// 临界区执行
    pub fn critical_section<F, R>(f: F) -> R
    where
        F: FnOnce() -> R,
    {
        interrupt::free(|_| f())
    }
    
    /// 禁用中断
    pub fn disable_interrupts() {
        interrupt::disable();
    }
    
    /// 启用中断
    pub fn enable_interrupts() {
        unsafe {
            interrupt::enable();
        }
    }
}

// 内存管理
pub mod memory {
    use heapless::{pool::Pool, Vec};
    
    // 静态内存池
    static mut MEMORY: [u8; 2048] = [0; 2048];
    static mut POOL: Pool<[u8; 64]> = Pool::new();
    
    /// 初始化内存池
    pub fn init_memory_pool() {
        unsafe {
            POOL.grow(&mut MEMORY);
        }
    }
    
    /// 分配内存块
    pub fn allocate_block() -> Option<heapless::pool::Node<[u8; 64]>> {
        unsafe { POOL.alloc() }
    }
    
    /// 获取可用内存统计
    pub fn get_memory_stats() -> MemoryStats {
        MemoryStats {
            total_size: 2048,
            used_size: 0, // 这里应该实现实际的统计
            free_size: 2048,
            fragmentation: 0.0,
        }
    }
    
    pub struct MemoryStats {
        pub total_size: usize,
        pub used_size: usize,
        pub free_size: usize,
        pub fragmentation: f32,
    }
}

// 测试支持
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_system_config() {
        let config = system::SystemConfig::default();
        assert_eq!(config.external_xtal_freq_hz, 12_000_000);
        assert_eq!(config.sys_freq_hz, 125_000_000);
        assert_eq!(config.usb_freq_hz, 48_000_000);
    }
    
    #[test]
    fn test_platform_error() {
        let error = PlatformError::InitializationFailed;
        assert!(matches!(error, PlatformError::InitializationFailed));
    }
    
    #[test]
    fn test_memory_stats() {
        let stats = memory::get_memory_stats();
        assert_eq!(stats.total_size, 2048);
        assert!(stats.free_size <= stats.total_size);
    }
}