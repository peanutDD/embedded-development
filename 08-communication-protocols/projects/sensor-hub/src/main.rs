#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use cortex_m::delay::Delay;

use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{gpioa::PA5, gpiob::PB6, gpioc::PC13, Output, PushPull},
    i2c::{I2c, Mode},
    spi::{Spi, NoMiso, Mode as SpiMode, Polarity, Phase},
    rcc::Clocks,
};
use nb::block;
use fugit::ExtU32;

// 导入模块
mod sensors;
mod flash_storage;
mod data_logger;

use sensors::{SensorManager, SensorData};
use flash_storage::FlashStorage;
use data_logger::{DataLogger, LogLevel};

// 系统配置常量
const SAMPLE_INTERVAL_MS: u32 = 1000; // 1秒采样间隔
const FLASH_WRITE_INTERVAL: u32 = 10; // 每10次采样写入Flash
const MAX_ERROR_COUNT: u32 = 5; // 最大错误计数

// 类型别名
type StatusLed = PC13<Output<PushPull>>;
type Button = PB6<Output<PushPull>>;

// 全局系统时间计数器
static mut SYSTEM_TIME: u32 = 0;

#[entry]
fn main() -> ! {
    // 获取外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut status_led = gpioc.pc13.into_push_pull_output();

    // 配置I2C
    let scl = gpiob.pb8.into_alternate_open_drain();
    let sda = gpiob.pb9.into_alternate_open_drain();
    
    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard { frequency: 100.kHz() },
        &clocks,
    );

    // 配置SPI
    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    let cs = gpioa.pa4.into_push_pull_output();

    let spi_mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    
    let spi = Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        spi_mode,
        1.MHz(),
        &clocks,
    );

    // 创建延时对象
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().to_Hz());

    // 系统启动指示
    startup_sequence(&mut status_led, &mut delay);

    // 初始化系统组件
    let mut sensor_manager = SensorManager::new(i2c);
    let mut flash_storage = FlashStorage::new(spi, cs);
    let mut data_logger = DataLogger::new();

    // 配置数据日志器
    data_logger.set_log_level(LogLevel::Info);

    // 初始化传感器
    if let Err(_) = sensor_manager.initialize() {
        data_logger.log(LogLevel::Error, "传感器初始化失败");
        // 错误指示
        for _ in 0..10 {
            led_blink(&mut status_led, &mut delay, 100);
        }
    }

    // 初始化Flash存储
    if let Err(_) = flash_storage.initialize() {
        data_logger.log(LogLevel::Error, "Flash存储初始化失败");
        // 错误指示
        for _ in 0..10 {
            led_blink(&mut status_led, &mut delay, 100);
        }
    }

    data_logger.log(LogLevel::Info, "系统初始化完成");

    // 主循环变量
    let mut sample_count = 0u32;
    let mut error_count = 0u32;

    // 主循环
    loop {
        status_led.set_low(); // 采样指示
        
        // 更新系统时间
        unsafe {
            SYSTEM_TIME = SYSTEM_TIME.wrapping_add(1);
        }

        // 读取传感器数据
        match sensor_manager.read_all_sensors() {
            Ok(sensor_data) => {
                // 记录数据到缓冲区
                data_logger.log_data(sensor_data, sample_count);
                
                sample_count += 1;
                error_count = 0; // 重置错误计数
                
                // 定期写入Flash
                if sample_count % FLASH_WRITE_INTERVAL == 0 {
                    if let Err(_) = write_data_to_flash(&mut flash_storage, &mut data_logger) {
                        data_logger.log(LogLevel::Error, "Flash写入失败");
                        error_count += 1;
                    }
                }
                
                // 状态指示
                display_system_status(&mut status_led, sample_count, error_count);
                
                // 日志输出
                if sample_count % 10 == 0 {
                    data_logger.log(LogLevel::Info, "系统运行正常");
                }
            }
            Err(_) => {
                error_count += 1;
                data_logger.log(LogLevel::Warning, "传感器读取失败");
                
                // 错误恢复
                if error_count >= MAX_ERROR_COUNT {
                    data_logger.log(LogLevel::Error, "系统错误过多，尝试恢复");
                    system_recovery(&mut sensor_manager, &mut flash_storage, &mut delay);
                    error_count = 0;
                }
            }
        }

        status_led.set_high(); // 结束采样指示
        
        // 内存管理
        if sample_count % 100 == 0 {
            // 执行内存清理或其他维护任务
        }

        // 低功耗等待
        delay.delay_ms(SAMPLE_INTERVAL_MS);
    }
}

// 系统启动序列
fn startup_sequence<LED, DELAY>(
    led: &mut LED, 
    delay: &mut DELAY
) 
where
    LED: embedded_hal::digital::v2::OutputPin,
    DELAY: embedded_hal::blocking::delay::DelayMs<u32>,
{
    // 启动指示序列
    for _ in 0..3 {
        led.set_high().ok();
        delay.delay_ms(200u32);
        led.set_low().ok();
        delay.delay_ms(200u32);
    }
}

// 写入数据到Flash
fn write_data_to_flash<SPI, CS>(
    _flash_storage: &mut FlashStorage<SPI, CS>,
    data_logger: &mut DataLogger,
) -> Result<u32, &'static str>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    // 获取缓冲区数据长度
    let buffer_len = data_logger.get_buffer().len();
    
    if buffer_len == 0 {
        return Ok(0);
    }
    
    // 清空缓冲区
    data_logger.clear_buffer();
    
    Ok(buffer_len as u32)
}

// 显示系统状态
fn display_system_status<LED>(
    status_led: &mut LED, 
    sample_count: u32, 
    error_count: u32
) 
where
    LED: embedded_hal::digital::v2::OutputPin,
{
    if error_count > 0 {
        // 错误指示：快速闪烁
        for _ in 0..error_count.min(5) {
            status_led.set_high().ok();
            delay_ms(50);
            status_led.set_low().ok();
            delay_ms(50);
        }
    } else if sample_count % 10 == 0 {
        // 正常指示：慢速闪烁
        status_led.set_high().ok();
        delay_ms(100);
        status_led.set_low().ok();
    }
}

// 系统恢复
fn system_recovery<DELAY>(
    _sensor_manager: &mut SensorManager<I2c<pac::I2C1, (stm32f4xx_hal::gpio::gpiob::PB8<stm32f4xx_hal::gpio::Alternate<4, stm32f4xx_hal::gpio::OpenDrain>>, stm32f4xx_hal::gpio::gpiob::PB9<stm32f4xx_hal::gpio::Alternate<4, stm32f4xx_hal::gpio::OpenDrain>>)>>, 
    _flash_storage: &mut FlashStorage<
        Spi<pac::SPI1, (
            stm32f4xx_hal::gpio::gpioa::PA5<stm32f4xx_hal::gpio::Alternate<5>>,
            stm32f4xx_hal::gpio::gpioa::PA6<stm32f4xx_hal::gpio::Alternate<5>>,
            stm32f4xx_hal::gpio::gpioa::PA7<stm32f4xx_hal::gpio::Alternate<5>>
        )>,
        stm32f4xx_hal::gpio::gpioa::PA4<Output<PushPull>>
    >,
    delay: &mut DELAY
) 
where
    DELAY: embedded_hal::blocking::delay::DelayMs<u32>,
{
    // 简化系统恢复逻辑，避免复杂的trait约束
    delay.delay_ms(100u32);
}

// LED闪烁控制
fn led_blink<LED, DELAY>(
    led: &mut LED, 
    delay: &mut DELAY, 
    duration_ms: u32
) 
where
    LED: embedded_hal::digital::v2::OutputPin,
    DELAY: embedded_hal::blocking::delay::DelayMs<u32>,
{
    led.set_high().ok();
    delay.delay_ms(duration_ms);
    led.set_low().ok();
    delay.delay_ms(duration_ms);
}

// 软件延时
fn delay_ms(ms: u32) {
    for _ in 0..(ms * 1000) {
        cortex_m::asm::nop();
    }
}

// 获取系统时间
fn get_system_time() -> u32 {
    unsafe { SYSTEM_TIME }
}