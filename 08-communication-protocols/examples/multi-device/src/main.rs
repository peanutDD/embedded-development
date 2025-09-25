#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    pac,
    i2c::I2c,
    spi::{Spi, Mode, Phase, Polarity},
    gpio::{Pin, Input, PullUp, Output, PushPull},
    timer::{Timer, Event},
    interrupt,
};
use defmt_rtt as _;
use heapless::Vec;

use multi_device::{
    BusManager, BusRequest, OperationType, Priority, DeviceType, DeviceStatus,
    CommunicationError, i2c_addresses,
};

/// 系统状态
static mut SYSTEM_STATE: Option<SystemState> = None;

/// 系统状态结构
struct SystemState {
    bus_manager: BusManager<I2c<pac::I2C1>, Spi<pac::SPI1>>,
    timer: Timer<pac::TIM2>,
    status_led: Pin<'C', 13, Output<PushPull>>,
    error_led: Pin<'C', 14, Output<PushPull>>,
    reset_button: Pin<'A', 0, Input<PullUp>>,
    tick_counter: u32,
    scan_interval: u32,
    last_scan_time: u32,
}

/// 演示模式
#[derive(Debug, Clone, Copy)]
enum DemoMode {
    DeviceScanning,
    DataCollection,
    PerformanceTest,
    ErrorRecovery,
}

#[entry]
fn main() -> ! {
    defmt::info!("多设备通信管理示例启动");
    
    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr
        .use_hse(8.MHz())
        .sysclk(84.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();
    
    defmt::info!("系统时钟配置完成: SYSCLK={}MHz", clocks.sysclk().raw() / 1_000_000);
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // I2C引脚配置
    let scl = gpiob.pb6.into_alternate_open_drain();
    let sda = gpiob.pb7.into_alternate_open_drain();
    
    // SPI引脚配置
    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    
    // 控制引脚配置
    let mut status_led = gpioc.pc13.into_push_pull_output();
    let mut error_led = gpioc.pc14.into_push_pull_output();
    let reset_button = gpioa.pa0.into_pull_up_input();
    
    status_led.set_high();
    error_led.set_high();
    
    // I2C配置
    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        400.kHz(),
        &clocks,
    );
    
    // SPI配置
    let spi_mode = Mode {
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
    
    defmt::info!("I2C和SPI接口初始化完成");
    
    // 创建总线管理器
    let mut bus_manager = BusManager::new();
    bus_manager.set_i2c(i2c);
    // 注意：这里需要将SPI转换为SpiDevice，实际项目中需要添加CS引脚管理
    // bus_manager.set_spi(spi);
    
    // 配置定时器
    let mut timer = Timer::new(dp.TIM2, &clocks);
    timer.start(100.Hz()); // 10ms间隔
    timer.listen(Event::Update);
    
    defmt::info!("定时器配置完成");
    
    // 创建系统状态
    let system_state = SystemState {
        bus_manager,
        timer,
        status_led,
        error_led,
        reset_button,
        tick_counter: 0,
        scan_interval: 500, // 5秒扫描一次
        last_scan_time: 0,
    };
    
    unsafe {
        SYSTEM_STATE = Some(system_state);
    }
    
    // 启用中断
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
    }
    
    defmt::info!("系统初始化完成，开始主循环");
    
    // 执行初始设备扫描
    initial_device_scan();
    
    let mut demo_mode = DemoMode::DeviceScanning;
    let mut mode_counter = 0;
    
    // 主循环
    loop {
        // 检查复位按键
        check_reset_button();
        
        // 处理总线请求
        process_bus_requests();
        
        // 根据演示模式执行不同操作
        match demo_mode {
            DemoMode::DeviceScanning => {
                device_scanning_demo();
            }
            DemoMode::DataCollection => {
                data_collection_demo();
            }
            DemoMode::PerformanceTest => {
                performance_test_demo();
            }
            DemoMode::ErrorRecovery => {
                error_recovery_demo();
            }
        }
        
        // 每30秒切换演示模式
        mode_counter += 1;
        if mode_counter >= 3000 { // 30秒 @ 100Hz
            demo_mode = match demo_mode {
                DemoMode::DeviceScanning => DemoMode::DataCollection,
                DemoMode::DataCollection => DemoMode::PerformanceTest,
                DemoMode::PerformanceTest => DemoMode::ErrorRecovery,
                DemoMode::ErrorRecovery => DemoMode::DeviceScanning,
            };
            mode_counter = 0;
            defmt::info!("切换到演示模式: {:?}", demo_mode);
        }
        
        // 短暂延时
        cortex_m::asm::delay(10_000);
    }
}

/// 初始设备扫描
fn initial_device_scan() {
    defmt::info!("执行初始设备扫描");
    
    unsafe {
        if let Some(ref mut state) = SYSTEM_STATE {
            // 扫描I2C设备
            match state.bus_manager.scan_i2c_devices() {
                Ok(devices) => {
                    defmt::info!("发现 {} 个I2C设备: {:?}", devices.len(), devices.as_slice());
                    
                    // 检查每个设备的状态
                    for device in state.bus_manager.get_devices() {
                        if device.device_type == DeviceType::I2c {
                            match state.bus_manager.check_device_status(device.id) {
                                Ok(status) => {
                                    defmt::info!("设备 {} (0x{:02X}): {:?}", 
                                        device.name.as_str(), device.address, status);
                                }
                                Err(e) => {
                                    defmt::error!("检查设备 {} 状态失败: {:?}", 
                                        device.name.as_str(), e);
                                }
                            }
                        }
                    }
                }
                Err(e) => {
                    defmt::error!("I2C设备扫描失败: {:?}", e);
                }
            }
            
            // 注册SPI设备（示例）
            let spi_devices = [
                (4, "W25Q128 Flash"),
                (8, "MCP3008 ADC"),
                (9, "MAX7219 Display"),
            ];
            
            for (cs_pin, name) in &spi_devices {
                match state.bus_manager.register_spi_device(*cs_pin, name) {
                    Ok(device_id) => {
                        defmt::info!("注册SPI设备: {} (ID: {})", name, device_id);
                    }
                    Err(e) => {
                        defmt::error!("注册SPI设备 {} 失败: {:?}", name, e);
                    }
                }
            }
        }
    }
}

/// 检查复位按键
fn check_reset_button() {
    unsafe {
        if let Some(ref mut state) = SYSTEM_STATE {
            if state.reset_button.is_low() {
                defmt::info!("检测到复位按键，重新扫描设备");
                
                // 重置统计信息
                state.bus_manager.reset_stats();
                
                // 重新扫描设备
                initial_device_scan();
                
                // 等待按键释放
                while state.reset_button.is_low() {
                    cortex_m::asm::delay(10_000);
                }
            }
        }
    }
}

/// 处理总线请求
fn process_bus_requests() {
    unsafe {
        if let Some(ref mut state) = SYSTEM_STATE {
            match state.bus_manager.process_requests() {
                Ok(processed_count) => {
                    if processed_count > 0 {
                        defmt::debug!("处理了 {} 个总线请求", processed_count);
                    }
                }
                Err(e) => {
                    defmt::error!("处理总线请求失败: {:?}", e);
                    state.error_led.set_low();
                }
            }
        }
    }
}

/// 设备扫描演示
fn device_scanning_demo() {
    unsafe {
        if let Some(ref mut state) = SYSTEM_STATE {
            // 定期重新扫描设备
            if state.tick_counter - state.last_scan_time >= state.scan_interval {
                defmt::info!("定期设备扫描");
                
                let online_count = state.bus_manager.get_online_device_count();
                defmt::info!("当前在线设备数量: {}", online_count);
                
                // 检查所有设备状态
                let device_count = state.bus_manager.get_devices().len();
                for i in 0..device_count {
                    if let Err(e) = state.bus_manager.check_device_status(i as u8) {
                        defmt::warn!("设备 {} 状态检查失败: {:?}", i, e);
                    }
                }
                
                state.last_scan_time = state.tick_counter;
            }
        }
    }
}

/// 数据采集演示
fn data_collection_demo() {
    unsafe {
        if let Some(ref mut state) = SYSTEM_STATE {
            // 模拟从传感器读取数据
            for device in state.bus_manager.get_devices() {
                if device.device_type == DeviceType::I2c && device.status == DeviceStatus::Online {
                    let request = BusRequest {
                        device_id: device.id,
                        priority: Priority::Normal,
                        operation_type: OperationType::Read,
                        data: Vec::new(),
                        timeout_ms: 100,
                    };
                    
                    if let Err(e) = state.bus_manager.add_request(request) {
                        defmt::warn!("添加读取请求失败: {:?}", e);
                    }
                }
            }
        }
    }
}

/// 性能测试演示
fn performance_test_demo() {
    unsafe {
        if let Some(ref mut state) = SYSTEM_STATE {
            let stats = state.bus_manager.get_global_stats();
            let (i2c_util, spi_util) = state.bus_manager.calculate_bus_utilization();
            
            defmt::info!("性能统计:");
            defmt::info!("  总操作: {}, 成功率: {:.1}%", 
                stats.total_operations, stats.success_rate());
            defmt::info!("  平均响应时间: {}μs", stats.average_response_time_us);
            defmt::info!("  I2C使用率: {}%, SPI使用率: {}%", i2c_util, spi_util);
        }
    }
}

/// 错误恢复演示
fn error_recovery_demo() {
    unsafe {
        if let Some(ref mut state) = SYSTEM_STATE {
            // 清理离线设备
            let before_count = state.bus_manager.get_devices().len();
            state.bus_manager.cleanup_offline_devices();
            let after_count = state.bus_manager.get_devices().len();
            
            if before_count != after_count {
                defmt::info!("清理了 {} 个离线设备", before_count - after_count);
            }
            
            // 检查错误状态
            let stats = state.bus_manager.get_global_stats();
            if stats.failed_operations > 0 {
                defmt::warn!("检测到 {} 个失败操作", stats.failed_operations);
                state.error_led.set_low();
            } else {
                state.error_led.set_high();
            }
        }
    }
}

/// 定时器中断处理
#[interrupt]
fn TIM2() {
    unsafe {
        if let Some(ref mut state) = SYSTEM_STATE {
            // 清除中断标志
            state.timer.clear_interrupt(Event::Update);
            
            // 更新系统时钟
            state.tick_counter = state.tick_counter.wrapping_add(1);
            state.bus_manager.update_tick();
            
            // 状态LED闪烁
            if state.tick_counter % 100 == 0 { // 每秒闪烁一次
                state.status_led.toggle();
            }
        }
    }
}