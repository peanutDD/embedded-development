# 启动过程和引导

## 概述

嵌入式系统的启动过程是系统从上电到正常运行的关键阶段。理解启动流程对于系统设计、调试和优化至关重要。本章详细介绍嵌入式系统的启动机制、引导加载程序设计和系统初始化过程。

## 学习目标

完成本章节后，你将掌握：
- 嵌入式系统完整的启动流程
- 引导加载程序的设计和实现
- 向量表和异常处理机制
- 系统初始化的最佳实践
- 故障恢复和安全启动技术

## 1. 系统启动流程

### 1.1 启动阶段概述

```rust
// 启动阶段枚举
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootStage {
    PowerOn,        // 上电阶段
    HardwareInit,   // 硬件初始化
    BootloaderInit, // 引导程序初始化
    ApplicationLoad,// 应用程序加载
    SystemInit,     // 系统初始化
    ApplicationRun, // 应用程序运行
}

// 启动状态跟踪
pub struct BootTracker {
    current_stage: BootStage,
    stage_timestamps: [u32; 6],
    error_count: u32,
    last_error: Option<BootError>,
}

#[derive(Debug, Clone, Copy)]
pub enum BootError {
    ClockInitFailed,
    MemoryTestFailed,
    FlashCorrupted,
    ApplicationInvalid,
    WatchdogTimeout,
    HardwareFault,
}

impl BootTracker {
    pub const fn new() -> Self {
        Self {
            current_stage: BootStage::PowerOn,
            stage_timestamps: [0; 6],
            error_count: 0,
            last_error: None,
        }
    }
    
    pub fn advance_stage(&mut self, new_stage: BootStage, timestamp: u32) {
        self.current_stage = new_stage;
        self.stage_timestamps[new_stage as usize] = timestamp;
    }
    
    pub fn record_error(&mut self, error: BootError) {
        self.error_count += 1;
        self.last_error = Some(error);
    }
    
    pub fn get_boot_time(&self) -> u32 {
        if let Some(&app_start) = self.stage_timestamps.get(BootStage::ApplicationRun as usize) {
            if let Some(&power_on) = self.stage_timestamps.get(BootStage::PowerOn as usize) {
                return app_start - power_on;
            }
        }
        0
    }
}
```

### 1.2 详细启动流程

```rust
use cortex_m::peripheral::{SCB, NVIC};
use stm32f4xx_hal::pac::{RCC, FLASH};

// 启动管理器
pub struct BootManager {
    tracker: BootTracker,
    config: BootConfig,
    recovery_mode: bool,
}

#[derive(Debug, Clone)]
pub struct BootConfig {
    pub enable_watchdog: bool,
    pub boot_timeout_ms: u32,
    pub memory_test_enabled: bool,
    pub secure_boot_enabled: bool,
    pub recovery_partition_offset: u32,
}

impl Default for BootConfig {
    fn default() -> Self {
        Self {
            enable_watchdog: true,
            boot_timeout_ms: 5000,
            memory_test_enabled: true,
            secure_boot_enabled: false,
            recovery_partition_offset: 0x20000,
        }
    }
}

impl BootManager {
    pub fn new(config: BootConfig) -> Self {
        Self {
            tracker: BootTracker::new(),
            config,
            recovery_mode: false,
        }
    }
    
    pub fn boot_sequence(&mut self) -> Result<(), BootError> {
        // 阶段1: 硬件初始化
        self.hardware_initialization()?;
        
        // 阶段2: 引导程序初始化
        self.bootloader_initialization()?;
        
        // 阶段3: 应用程序验证和加载
        self.application_loading()?;
        
        // 阶段4: 系统初始化
        self.system_initialization()?;
        
        // 阶段5: 启动应用程序
        self.start_application()?;
        
        Ok(())
    }
    
    fn hardware_initialization(&mut self) -> Result<(), BootError> {
        let timestamp = self.get_timestamp();
        self.tracker.advance_stage(BootStage::HardwareInit, timestamp);
        
        // 1. 时钟系统初始化
        self.init_clock_system()?;
        
        // 2. 内存系统初始化
        self.init_memory_system()?;
        
        // 3. 基本外设初始化
        self.init_basic_peripherals()?;
        
        // 4. 看门狗初始化
        if self.config.enable_watchdog {
            self.init_watchdog()?;
        }
        
        Ok(())
    }
    
    fn init_clock_system(&mut self) -> Result<(), BootError> {
        unsafe {
            let rcc = &*RCC::ptr();
            
            // 启用HSE时钟
            rcc.cr.modify(|_, w| w.hseon().set_bit());
            
            // 等待HSE就绪
            let mut timeout = 10000;
            while rcc.cr.read().hserdy().bit_is_clear() {
                timeout -= 1;
                if timeout == 0 {
                    self.tracker.record_error(BootError::ClockInitFailed);
                    return Err(BootError::ClockInitFailed);
                }
            }
            
            // 配置PLL
            rcc.pllcfgr.modify(|_, w| {
                w.pllsrc().hse()
                 .pllm().bits(8)    // 8MHz / 8 = 1MHz
                 .plln().bits(336)  // 1MHz * 336 = 336MHz
                 .pllp().div4()     // 336MHz / 4 = 84MHz
                 .pllq().bits(7)    // 336MHz / 7 = 48MHz
            });
            
            // 启用PLL
            rcc.cr.modify(|_, w| w.pllon().set_bit());
            
            // 等待PLL就绪
            timeout = 10000;
            while rcc.cr.read().pllrdy().bit_is_clear() {
                timeout -= 1;
                if timeout == 0 {
                    self.tracker.record_error(BootError::ClockInitFailed);
                    return Err(BootError::ClockInitFailed);
                }
            }
            
            // 配置Flash等待状态
            let flash = &*FLASH::ptr();
            flash.acr.modify(|_, w| w.latency().bits(2)); // 2 wait states for 84MHz
            
            // 切换到PLL时钟
            rcc.cfgr.modify(|_, w| w.sw().pll());
            
            // 等待时钟切换完成
            while !rcc.cfgr.read().sws().is_pll() {}
        }
        
        Ok(())
    }
    
    fn init_memory_system(&mut self) -> Result<(), BootError> {
        // 内存测试
        if self.config.memory_test_enabled {
            self.memory_test()?;
        }
        
        // 初始化堆栈
        self.init_stack()?;
        
        // 清零BSS段
        self.clear_bss()?;
        
        // 初始化数据段
        self.init_data()?;
        
        Ok(())
    }
    
    fn memory_test(&mut self) -> Result<(), BootError> {
        // 简单的内存测试
        const TEST_PATTERN: u32 = 0xDEADBEEF;
        const RAM_START: *mut u32 = 0x20000000 as *mut u32;
        const RAM_SIZE: usize = 128 * 1024; // 128KB
        
        unsafe {
            // 写入测试模式
            for i in 0..(RAM_SIZE / 4) {
                let addr = RAM_START.add(i);
                core::ptr::write_volatile(addr, TEST_PATTERN.wrapping_add(i as u32));
            }
            
            // 验证测试模式
            for i in 0..(RAM_SIZE / 4) {
                let addr = RAM_START.add(i);
                let expected = TEST_PATTERN.wrapping_add(i as u32);
                let actual = core::ptr::read_volatile(addr);
                
                if actual != expected {
                    self.tracker.record_error(BootError::MemoryTestFailed);
                    return Err(BootError::MemoryTestFailed);
                }
            }
            
            // 清零内存
            for i in 0..(RAM_SIZE / 4) {
                let addr = RAM_START.add(i);
                core::ptr::write_volatile(addr, 0);
            }
        }
        
        Ok(())
    }
    
    fn init_stack(&self) -> Result<(), BootError> {
        // 栈指针在启动时由硬件自动设置
        // 这里可以进行栈溢出保护设置
        Ok(())
    }
    
    fn clear_bss(&self) -> Result<(), BootError> {
        extern "C" {
            static mut __sbss: u32;
            static mut __ebss: u32;
        }
        
        unsafe {
            let start = &mut __sbss as *mut u32;
            let end = &mut __ebss as *mut u32;
            let count = end.offset_from(start) as usize;
            
            core::ptr::write_bytes(start, 0, count);
        }
        
        Ok(())
    }
    
    fn init_data(&self) -> Result<(), BootError> {
        extern "C" {
            static mut __sdata: u32;
            static mut __edata: u32;
            static __sidata: u32;
        }
        
        unsafe {
            let start = &mut __sdata as *mut u32;
            let end = &mut __edata as *mut u32;
            let init_data = &__sidata as *const u32;
            let count = end.offset_from(start) as usize;
            
            core::ptr::copy_nonoverlapping(init_data, start, count);
        }
        
        Ok(())
    }
    
    fn init_basic_peripherals(&self) -> Result<(), BootError> {
        // 初始化基本外设
        unsafe {
            let rcc = &*RCC::ptr();
            
            // 启用GPIO时钟
            rcc.ahb1enr.modify(|_, w| {
                w.gpioaen().set_bit()
                 .gpioben().set_bit()
                 .gpiocen().set_bit()
            });
            
            // 启用基本定时器时钟
            rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
        }
        
        Ok(())
    }
    
    fn init_watchdog(&self) -> Result<(), BootError> {
        // 初始化独立看门狗
        unsafe {
            let iwdg = &*stm32f4xx_hal::pac::IWDG::ptr();
            
            // 启用写访问
            iwdg.kr.write(|w| w.key().bits(0x5555));
            
            // 设置预分频器 (32kHz / 64 = 512Hz)
            iwdg.pr.write(|w| w.pr().bits(0x04));
            
            // 设置重载值 (512Hz / 512 = 1Hz, 即1秒超时)
            iwdg.rlr.write(|w| w.rl().bits(512));
            
            // 启动看门狗
            iwdg.kr.write(|w| w.key().bits(0xCCCC));
        }
        
        Ok(())
    }
    
    fn bootloader_initialization(&mut self) -> Result<(), BootError> {
        let timestamp = self.get_timestamp();
        self.tracker.advance_stage(BootStage::BootloaderInit, timestamp);
        
        // 初始化向量表
        self.init_vector_table()?;
        
        // 初始化中断系统
        self.init_interrupt_system()?;
        
        // 初始化调试接口
        self.init_debug_interface()?;
        
        Ok(())
    }
    
    fn init_vector_table(&self) -> Result<(), BootError> {
        extern "C" {
            static __vector_table: u32;
        }
        
        unsafe {
            let scb = &*SCB::ptr();
            let vector_table_addr = &__vector_table as *const u32 as u32;
            
            // 设置向量表偏移
            scb.vtor.write(vector_table_addr);
        }
        
        Ok(())
    }
    
    fn init_interrupt_system(&self) -> Result<(), BootError> {
        unsafe {
            // 设置中断优先级分组
            SCB::set_priority_grouping(3); // 4位抢占优先级，0位子优先级
            
            // 禁用所有中断
            let nvic = &*NVIC::ptr();
            for i in 0..8 {
                nvic.icer[i].write(0xFFFFFFFF);
            }
            
            // 清除所有挂起的中断
            for i in 0..8 {
                nvic.icpr[i].write(0xFFFFFFFF);
            }
        }
        
        Ok(())
    }
    
    fn init_debug_interface(&self) -> Result<(), BootError> {
        // 初始化调试接口（如果需要）
        Ok(())
    }
    
    fn application_loading(&mut self) -> Result<(), BootError> {
        let timestamp = self.get_timestamp();
        self.tracker.advance_stage(BootStage::ApplicationLoad, timestamp);
        
        // 验证应用程序
        self.verify_application()?;
        
        // 加载应用程序（如果需要）
        self.load_application()?;
        
        Ok(())
    }
    
    fn verify_application(&mut self) -> Result<(), BootError> {
        // 检查应用程序的完整性
        const APP_START: *const u32 = 0x08008000 as *const u32; // 应用程序起始地址
        
        unsafe {
            // 检查栈指针是否有效
            let stack_pointer = core::ptr::read_volatile(APP_START);
            if stack_pointer < 0x20000000 || stack_pointer > 0x20020000 {
                self.tracker.record_error(BootError::ApplicationInvalid);
                return Err(BootError::ApplicationInvalid);
            }
            
            // 检查复位向量是否有效
            let reset_vector = core::ptr::read_volatile(APP_START.add(1));
            if reset_vector < 0x08008000 || reset_vector > 0x08080000 {
                self.tracker.record_error(BootError::ApplicationInvalid);
                return Err(BootError::ApplicationInvalid);
            }
        }
        
        // 如果启用了安全启动，进行签名验证
        if self.config.secure_boot_enabled {
            self.verify_signature()?;
        }
        
        Ok(())
    }
    
    fn verify_signature(&self) -> Result<(), BootError> {
        // 简化的签名验证实现
        // 实际实现需要使用加密库
        Ok(())
    }
    
    fn load_application(&self) -> Result<(), BootError> {
        // 如果应用程序需要从外部存储加载到RAM
        // 这里实现加载逻辑
        Ok(())
    }
    
    fn system_initialization(&mut self) -> Result<(), BootError> {
        let timestamp = self.get_timestamp();
        self.tracker.advance_stage(BootStage::SystemInit, timestamp);
        
        // 初始化系统服务
        self.init_system_services()?;
        
        // 初始化外设
        self.init_peripherals()?;
        
        // 初始化文件系统（如果有）
        self.init_filesystem()?;
        
        Ok(())
    }
    
    fn init_system_services(&self) -> Result<(), BootError> {
        // 初始化系统服务
        Ok(())
    }
    
    fn init_peripherals(&self) -> Result<(), BootError> {
        // 初始化外设
        Ok(())
    }
    
    fn init_filesystem(&self) -> Result<(), BootError> {
        // 初始化文件系统
        Ok(())
    }
    
    fn start_application(&mut self) -> Result<(), BootError> {
        let timestamp = self.get_timestamp();
        self.tracker.advance_stage(BootStage::ApplicationRun, timestamp);
        
        // 跳转到应用程序
        self.jump_to_application()
    }
    
    fn jump_to_application(&self) -> Result<(), BootError> {
        const APP_START: *const u32 = 0x08008000 as *const u32;
        
        unsafe {
            // 读取应用程序的栈指针和复位向量
            let stack_pointer = core::ptr::read_volatile(APP_START);
            let reset_vector = core::ptr::read_volatile(APP_START.add(1));
            
            // 禁用中断
            cortex_m::interrupt::disable();
            
            // 设置栈指针
            cortex_m::register::msp::write(stack_pointer);
            
            // 跳转到应用程序
            let app_entry: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
            app_entry();
        }
    }
    
    fn get_timestamp(&self) -> u32 {
        // 返回当前时间戳（毫秒）
        // 实际实现需要使用系统定时器
        0
    }
    
    pub fn enter_recovery_mode(&mut self) -> Result<(), BootError> {
        self.recovery_mode = true;
        
        // 尝试从恢复分区启动
        self.load_recovery_application()?;
        
        Ok(())
    }
    
    fn load_recovery_application(&self) -> Result<(), BootError> {
        // 从恢复分区加载应用程序
        Ok(())
    }
    
    pub fn get_boot_statistics(&self) -> BootStatistics {
        BootStatistics {
            total_boot_time: self.tracker.get_boot_time(),
            error_count: self.tracker.error_count,
            last_error: self.tracker.last_error,
            current_stage: self.tracker.current_stage,
            stage_times: self.calculate_stage_times(),
        }
    }
    
    fn calculate_stage_times(&self) -> [u32; 5] {
        let mut stage_times = [0u32; 5];
        
        for i in 1..6 {
            if self.tracker.stage_timestamps[i] > 0 && self.tracker.stage_timestamps[i-1] > 0 {
                stage_times[i-1] = self.tracker.stage_timestamps[i] - self.tracker.stage_timestamps[i-1];
            }
        }
        
        stage_times
    }
}

#[derive(Debug, Clone)]
pub struct BootStatistics {
    pub total_boot_time: u32,
    pub error_count: u32,
    pub last_error: Option<BootError>,
    pub current_stage: BootStage,
    pub stage_times: [u32; 5], // 各阶段耗时
}
```

## 2. 引导加载程序设计

### 2.1 Bootloader架构

```rust
// 引导加载程序主结构
pub struct Bootloader {
    config: BootloaderConfig,
    flash_driver: FlashDriver,
    comm_interface: CommunicationInterface,
    update_manager: UpdateManager,
}

#[derive(Debug, Clone)]
pub struct BootloaderConfig {
    pub bootloader_version: [u8; 4],
    pub app_start_address: u32,
    pub app_max_size: u32,
    pub update_timeout_ms: u32,
    pub enable_uart_update: bool,
    pub enable_usb_update: bool,
    pub enable_ota_update: bool,
}

impl Default for BootloaderConfig {
    fn default() -> Self {
        Self {
            bootloader_version: [1, 0, 0, 0],
            app_start_address: 0x08008000,
            app_max_size: 480 * 1024, // 480KB
            update_timeout_ms: 30000,  // 30秒
            enable_uart_update: true,
            enable_usb_update: false,
            enable_ota_update: false,
        }
    }
}

// Flash驱动程序
pub struct FlashDriver {
    base_address: u32,
    sector_size: u32,
    total_size: u32,
}

impl FlashDriver {
    pub fn new(base_address: u32, sector_size: u32, total_size: u32) -> Self {
        Self {
            base_address,
            sector_size,
            total_size,
        }
    }
    
    pub fn erase_sector(&mut self, sector: u32) -> Result<(), FlashError> {
        unsafe {
            let flash = &*FLASH::ptr();
            
            // 解锁Flash
            flash.keyr.write(|w| w.key().bits(0x45670123));
            flash.keyr.write(|w| w.key().bits(0xCDEF89AB));
            
            // 等待Flash就绪
            while flash.sr.read().bsy().bit_is_set() {}
            
            // 设置扇区擦除
            flash.cr.modify(|_, w| {
                w.ser().set_bit()
                 .snb().bits(sector as u8)
                 .strt().set_bit()
            });
            
            // 等待操作完成
            while flash.sr.read().bsy().bit_is_set() {}
            
            // 检查错误
            let sr = flash.sr.read();
            if sr.pgaerr().bit_is_set() || sr.pgperr().bit_is_set() || sr.wrperr().bit_is_set() {
                return Err(FlashError::ProgramError);
            }
            
            // 清除扇区擦除位
            flash.cr.modify(|_, w| w.ser().clear_bit());
            
            // 锁定Flash
            flash.cr.modify(|_, w| w.lock().set_bit());
        }
        
        Ok(())
    }
    
    pub fn program_word(&mut self, address: u32, data: u32) -> Result<(), FlashError> {
        if address < self.base_address || address >= self.base_address + self.total_size {
            return Err(FlashError::InvalidAddress);
        }
        
        unsafe {
            let flash = &*FLASH::ptr();
            
            // 解锁Flash
            flash.keyr.write(|w| w.key().bits(0x45670123));
            flash.keyr.write(|w| w.key().bits(0xCDEF89AB));
            
            // 等待Flash就绪
            while flash.sr.read().bsy().bit_is_set() {}
            
            // 设置编程模式
            flash.cr.modify(|_, w| w.pg().set_bit().psize().bits(0b10)); // 32位编程
            
            // 写入数据
            core::ptr::write_volatile(address as *mut u32, data);
            
            // 等待操作完成
            while flash.sr.read().bsy().bit_is_set() {}
            
            // 检查错误
            let sr = flash.sr.read();
            if sr.pgaerr().bit_is_set() || sr.pgperr().bit_is_set() || sr.wrperr().bit_is_set() {
                return Err(FlashError::ProgramError);
            }
            
            // 清除编程位
            flash.cr.modify(|_, w| w.pg().clear_bit());
            
            // 锁定Flash
            flash.cr.modify(|_, w| w.lock().set_bit());
            
            // 验证写入
            let written = core::ptr::read_volatile(address as *const u32);
            if written != data {
                return Err(FlashError::VerifyError);
            }
        }
        
        Ok(())
    }
    
    pub fn read_word(&self, address: u32) -> Result<u32, FlashError> {
        if address < self.base_address || address >= self.base_address + self.total_size {
            return Err(FlashError::InvalidAddress);
        }
        
        unsafe {
            Ok(core::ptr::read_volatile(address as *const u32))
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum FlashError {
    InvalidAddress,
    ProgramError,
    EraseError,
    VerifyError,
    Locked,
}

// 通信接口
pub enum CommunicationInterface {
    Uart(UartInterface),
    Usb(UsbInterface),
    Ethernet(EthernetInterface),
}

pub struct UartInterface {
    baudrate: u32,
    timeout_ms: u32,
}

pub struct UsbInterface {
    vendor_id: u16,
    product_id: u16,
}

pub struct EthernetInterface {
    ip_address: [u8; 4],
    port: u16,
}

// 更新管理器
pub struct UpdateManager {
    current_version: [u8; 4],
    update_buffer: [u8; 4096],
    update_progress: UpdateProgress,
}

#[derive(Debug, Clone)]
pub struct UpdateProgress {
    pub total_bytes: u32,
    pub received_bytes: u32,
    pub current_sector: u32,
    pub status: UpdateStatus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateStatus {
    Idle,
    Receiving,
    Verifying,
    Installing,
    Completed,
    Failed(UpdateError),
}

#[derive(Debug, Clone, Copy)]
pub enum UpdateError {
    InvalidHeader,
    ChecksumMismatch,
    FlashError,
    TimeoutError,
    ProtocolError,
}

impl UpdateManager {
    pub fn new(current_version: [u8; 4]) -> Self {
        Self {
            current_version,
            update_buffer: [0; 4096],
            update_progress: UpdateProgress {
                total_bytes: 0,
                received_bytes: 0,
                current_sector: 0,
                status: UpdateStatus::Idle,
            },
        }
    }
    
    pub fn start_update(&mut self, total_size: u32) -> Result<(), UpdateError> {
        self.update_progress = UpdateProgress {
            total_bytes: total_size,
            received_bytes: 0,
            current_sector: 0,
            status: UpdateStatus::Receiving,
        };
        
        Ok(())
    }
    
    pub fn process_data(&mut self, data: &[u8]) -> Result<(), UpdateError> {
        if self.update_progress.status != UpdateStatus::Receiving {
            return Err(UpdateError::ProtocolError);
        }
        
        // 处理接收到的数据
        // 这里简化实现
        self.update_progress.received_bytes += data.len() as u32;
        
        if self.update_progress.received_bytes >= self.update_progress.total_bytes {
            self.update_progress.status = UpdateStatus::Verifying;
        }
        
        Ok(())
    }
    
    pub fn verify_update(&mut self) -> Result<(), UpdateError> {
        if self.update_progress.status != UpdateStatus::Verifying {
            return Err(UpdateError::ProtocolError);
        }
        
        // 验证更新数据的完整性
        // 这里简化实现
        
        self.update_progress.status = UpdateStatus::Installing;
        Ok(())
    }
    
    pub fn install_update(&mut self, flash_driver: &mut FlashDriver) -> Result<(), UpdateError> {
        if self.update_progress.status != UpdateStatus::Installing {
            return Err(UpdateError::ProtocolError);
        }
        
        // 安装更新
        // 这里简化实现
        
        self.update_progress.status = UpdateStatus::Completed;
        Ok(())
    }
    
    pub fn get_progress(&self) -> &UpdateProgress {
        &self.update_progress
    }
}

impl Bootloader {
    pub fn new(config: BootloaderConfig) -> Self {
        Self {
            config,
            flash_driver: FlashDriver::new(0x08000000, 16 * 1024, 1024 * 1024),
            comm_interface: CommunicationInterface::Uart(UartInterface {
                baudrate: 115200,
                timeout_ms: 1000,
            }),
            update_manager: UpdateManager::new([1, 0, 0, 0]),
        }
    }
    
    pub fn run(&mut self) -> ! {
        // 初始化硬件
        self.init_hardware();
        
        // 检查是否需要进入更新模式
        if self.should_enter_update_mode() {
            self.enter_update_mode();
        }
        
        // 验证应用程序
        match self.verify_application() {
            Ok(_) => {
                // 启动应用程序
                self.start_application();
            }
            Err(_) => {
                // 进入恢复模式
                self.enter_recovery_mode();
            }
        }
    }
    
    fn init_hardware(&mut self) {
        // 初始化基本硬件
    }
    
    fn should_enter_update_mode(&self) -> bool {
        // 检查更新标志
        // 检查按键状态
        // 检查通信接口
        false
    }
    
    fn enter_update_mode(&mut self) {
        // 进入固件更新模式
        loop {
            // 处理更新协议
            // 接收固件数据
            // 写入Flash
        }
    }
    
    fn verify_application(&self) -> Result<(), BootError> {
        // 验证应用程序完整性
        Ok(())
    }
    
    fn start_application(&self) -> ! {
        unsafe {
            let app_start = self.config.app_start_address as *const u32;
            let stack_pointer = core::ptr::read_volatile(app_start);
            let reset_vector = core::ptr::read_volatile(app_start.add(1));
            
            // 设置栈指针
            cortex_m::register::msp::write(stack_pointer);
            
            // 跳转到应用程序
            let app_entry: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
            app_entry();
        }
    }
    
    fn enter_recovery_mode(&self) -> ! {
        // 进入恢复模式
        loop {
            // 等待恢复操作
        }
    }
}
```

## 3. 向量表和异常处理

### 3.1 向量表结构

```rust
// 向量表定义
#[repr(C)]
pub struct VectorTable {
    pub initial_stack_pointer: u32,
    pub reset: unsafe extern "C" fn() -> !,
    pub nmi: unsafe extern "C" fn(),
    pub hard_fault: unsafe extern "C" fn(),
    pub mem_manage: unsafe extern "C" fn(),
    pub bus_fault: unsafe extern "C" fn(),
    pub usage_fault: unsafe extern "C" fn(),
    pub reserved1: [u32; 4],
    pub sv_call: unsafe extern "C" fn(),
    pub debug_monitor: unsafe extern "C" fn(),
    pub reserved2: u32,
    pub pend_sv: unsafe extern "C" fn(),
    pub sys_tick: unsafe extern "C" fn(),
    pub external_interrupts: [unsafe extern "C" fn(); 240],
}

// 默认异常处理程序
unsafe extern "C" fn default_handler() {
    loop {}
}

// 向量表实例
#[link_section = ".vector_table"]
#[no_mangle]
pub static VECTOR_TABLE: VectorTable = VectorTable {
    initial_stack_pointer: 0x20020000, // 栈顶地址
    reset: reset_handler,
    nmi: nmi_handler,
    hard_fault: hard_fault_handler,
    mem_manage: mem_manage_handler,
    bus_fault: bus_fault_handler,
    usage_fault: usage_fault_handler,
    reserved1: [0; 4],
    sv_call: sv_call_handler,
    debug_monitor: debug_monitor_handler,
    reserved2: 0,
    pend_sv: pend_sv_handler,
    sys_tick: sys_tick_handler,
    external_interrupts: [default_handler; 240],
};

// 复位处理程序
#[no_mangle]
unsafe extern "C" fn reset_handler() -> ! {
    // 初始化系统
    let mut boot_manager = BootManager::new(BootConfig::default());
    
    match boot_manager.boot_sequence() {
        Ok(_) => {
            // 启动成功，不应该到达这里
            loop {}
        }
        Err(error) => {
            // 启动失败，进入错误处理
            handle_boot_error(error);
        }
    }
}

// 异常处理程序
unsafe extern "C" fn nmi_handler() {
    // NMI异常处理
}

unsafe extern "C" fn hard_fault_handler() {
    // 硬件故障处理
    let scb = &*SCB::ptr();
    let hfsr = scb.hfsr.read();
    
    // 分析故障原因
    if hfsr & (1 << 30) != 0 {
        // 强制硬件故障
    }
    
    if hfsr & (1 << 1) != 0 {
        // 向量表硬件故障
    }
    
    // 记录故障信息并重启
    system_reset();
}

unsafe extern "C" fn mem_manage_handler() {
    // 内存管理故障处理
}

unsafe extern "C" fn bus_fault_handler() {
    // 总线故障处理
}

unsafe extern "C" fn usage_fault_handler() {
    // 使用故障处理
}

unsafe extern "C" fn sv_call_handler() {
    // 系统调用处理
}

unsafe extern "C" fn debug_monitor_handler() {
    // 调试监控处理
}

unsafe extern "C" fn pend_sv_handler() {
    // PendSV处理（用于上下文切换）
}

unsafe extern "C" fn sys_tick_handler() {
    // 系统滴答处理
}

fn handle_boot_error(error: BootError) -> ! {
    // 处理启动错误
    match error {
        BootError::ClockInitFailed => {
            // 时钟初始化失败，尝试使用内部时钟
        }
        BootError::MemoryTestFailed => {
            // 内存测试失败，记录错误并重启
        }
        BootError::ApplicationInvalid => {
            // 应用程序无效，进入恢复模式
        }
        _ => {
            // 其他错误，重启系统
        }
    }
    
    system_reset();
}

fn system_reset() -> ! {
    unsafe {
        SCB::sys_reset();
    }
}
```

## 4. 系统初始化最佳实践

### 4.1 初始化顺序

```rust
// 系统初始化管理器
pub struct SystemInitializer {
    init_stages: Vec<InitStage, 16>,
    current_stage: usize,
    init_context: InitContext,
}

#[derive(Debug, Clone)]
pub struct InitStage {
    pub name: &'static str,
    pub priority: u8,
    pub required: bool,
    pub init_fn: fn(&mut InitContext) -> Result<(), InitError>,
    pub cleanup_fn: Option<fn(&mut InitContext)>,
}

#[derive(Debug)]
pub struct InitContext {
    pub system_clock_hz: u32,
    pub available_memory: u32,
    pub hardware_revision: u8,
    pub boot_reason: BootReason,
    pub error_count: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum BootReason {
    PowerOn,
    Reset,
    Watchdog,
    SoftwareReset,
    BrownOut,
    Unknown,
}

#[derive(Debug, Clone, Copy)]
pub enum InitError {
    HardwareNotFound,
    ConfigurationError,
    ResourceUnavailable,
    TimeoutError,
    DependencyMissing,
}

impl SystemInitializer {
    pub fn new() -> Self {
        let mut initializer = Self {
            init_stages: Vec::new(),
            current_stage: 0,
            init_context: InitContext {
                system_clock_hz: 0,
                available_memory: 0,
                hardware_revision: 0,
                boot_reason: BootReason::Unknown,
                error_count: 0,
            },
        };
        
        // 注册初始化阶段
        initializer.register_default_stages();
        initializer
    }
    
    fn register_default_stages(&mut self) {
        // 按优先级顺序注册初始化阶段
        let _ = self.add_stage(InitStage {
            name: "Clock System",
            priority: 1,
            required: true,
            init_fn: init_clock_system,
            cleanup_fn: None,
        });
        
        let _ = self.add_stage(InitStage {
            name: "Memory System",
            priority: 2,
            required: true,
            init_fn: init_memory_system,
            cleanup_fn: Some(cleanup_memory_system),
        });
        
        let _ = self.add_stage(InitStage {
            name: "Interrupt System",
            priority: 3,
            required: true,
            init_fn: init_interrupt_system,
            cleanup_fn: None,
        });
        
        let _ = self.add_stage(InitStage {
            name: "GPIO System",
            priority: 4,
            required: false,
            init_fn: init_gpio_system,
            cleanup_fn: None,
        });
        
        let _ = self.add_stage(InitStage {
            name: "Timer System",
            priority: 5,
            required: false,
            init_fn: init_timer_system,
            cleanup_fn: None,
        });
        
        let _ = self.add_stage(InitStage {
            name: "Communication",
            priority: 6,
            required: false,
            init_fn: init_communication,
            cleanup_fn: Some(cleanup_communication),
        });
    }
    
    pub fn add_stage(&mut self, stage: InitStage) -> Result<(), &'static str> {
        if self.init_stages.len() >= 16 {
            return Err("Too many init stages");
        }
        
        self.init_stages.push(stage).map_err(|_| "Failed to add stage")
    }
    
    pub fn initialize(&mut self) -> Result<(), InitError> {
        // 按优先级排序
        self.init_stages.sort_by_key(|stage| stage.priority);
        
        // 逐个执行初始化阶段
        for (index, stage) in self.init_stages.iter().enumerate() {
            self.current_stage = index;
            
            match (stage.init_fn)(&mut self.init_context) {
                Ok(_) => {
                    // 初始化成功
                }
                Err(error) => {
                    self.init_context.error_count += 1;
                    
                    if stage.required {
                        // 必需的阶段失败，执行清理并返回错误
                        self.cleanup_on_error(index);
                        return Err(error);
                    } else {
                        // 可选阶段失败，继续执行
                        continue;
                    }
                }
            }
        }
        
        Ok(())
    }
    
    fn cleanup_on_error(&mut self, failed_stage: usize) {
        // 逆序清理已初始化的阶段
        for i in (0..failed_stage).rev() {
            if let Some(cleanup_fn) = self.init_stages[i].cleanup_fn {
                cleanup_fn(&mut self.init_context);
            }
        }
    }
    
    pub fn get_init_status(&self) -> InitStatus {
        InitStatus {
            completed_stages: self.current_stage,
            total_stages: self.init_stages.len(),
            error_count: self.init_context.error_count,
            system_clock: self.init_context.system_clock_hz,
            available_memory: self.init_context.available_memory,
        }
    }
}

#[derive(Debug, Clone)]
pub struct InitStatus {
    pub completed_stages: usize,
    pub total_stages: usize,
    pub error_count: u32,
    pub system_clock: u32,
    pub available_memory: u32,
}

// 初始化函数实现
fn init_clock_system(ctx: &mut InitContext) -> Result<(), InitError> {
    // 时钟系统初始化
    ctx.system_clock_hz = 84_000_000; // 84MHz
    Ok(())
}

fn init_memory_system(ctx: &mut InitContext) -> Result<(), InitError> {
    // 内存系统初始化
    ctx.available_memory = 128 * 1024; // 128KB RAM
    Ok(())
}

fn cleanup_memory_system(ctx: &mut InitContext) {
    // 内存系统清理
    ctx.available_memory = 0;
}

fn init_interrupt_system(ctx: &mut InitContext) -> Result<(), InitError> {
    // 中断系统初始化
    Ok(())
}

fn init_gpio_system(ctx: &mut InitContext) -> Result<(), InitError> {
    // GPIO系统初始化
    Ok(())
}

fn init_timer_system(ctx: &mut InitContext) -> Result<(), InitError> {
    // 定时器系统初始化
    Ok(())
}

fn init_communication(ctx: &mut InitContext) -> Result<(), InitError> {
    // 通信系统初始化
    Ok(())
}

fn cleanup_communication(ctx: &mut InitContext) {
    // 通信系统清理
}
```

## 5. 故障恢复机制

### 5.1 故障检测和恢复

```rust
// 故障恢复管理器
pub struct FaultRecoveryManager {
    recovery_strategies: Vec<RecoveryStrategy, 8>,
    fault_history: Vec<FaultRecord, 32>,
    recovery_attempts: u32,
    max_recovery_attempts: u32,
}

#[derive(Debug, Clone)]
pub struct RecoveryStrategy {
    pub fault_type: FaultType,
    pub recovery_action: RecoveryAction,
    pub max_attempts: u32,
    pub timeout_ms: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultType {
    HardFault,
    MemoryFault,
    BusFault,
    UsageFault,
    WatchdogTimeout,
    ClockFailure,
    PowerFailure,
    CommunicationFailure,
}

#[derive(Debug, Clone, Copy)]
pub enum RecoveryAction {
    SoftReset,
    HardReset,
    ReconfigurePeripheral,
    SwitchToBackup,
    EnterSafeMode,
    RestartService,
}

#[derive(Debug, Clone)]
pub struct FaultRecord {
    pub fault_type: FaultType,
    pub timestamp: u32,
    pub recovery_action: RecoveryAction,
    pub recovery_successful: bool,
    pub additional_info: u32,
}

impl FaultRecoveryManager {
    pub fn new(max_recovery_attempts: u32) -> Self {
        let mut manager = Self {
            recovery_strategies: Vec::new(),
            fault_history: Vec::new(),
            recovery_attempts: 0,
            max_recovery_attempts,
        };
        
        manager.register_default_strategies();
        manager
    }
    
    fn register_default_strategies(&mut self) {
        // 注册默认恢复策略
        let strategies = [
            RecoveryStrategy {
                fault_type: FaultType::WatchdogTimeout,
                recovery_action: RecoveryAction::SoftReset,
                max_attempts: 3,
                timeout_ms: 1000,
            },
            RecoveryStrategy {
                fault_type: FaultType::ClockFailure,
                recovery_action: RecoveryAction::SwitchToBackup,
                max_attempts: 2,
                timeout_ms: 500,
            },
            RecoveryStrategy {
                fault_type: FaultType::CommunicationFailure,
                recovery_action: RecoveryAction::RestartService,
                max_attempts: 5,
                timeout_ms: 2000,
            },
            RecoveryStrategy {
                fault_type: FaultType::HardFault,
                recovery_action: RecoveryAction::HardReset,
                max_attempts: 1,
                timeout_ms: 0,
            },
        ];
        
        for strategy in &strategies {
            let _ = self.recovery_strategies.push(*strategy);
        }
    }
    
    pub fn handle_fault(&mut self, fault_type: FaultType, fault_info: u32) -> RecoveryResult {
        // 查找对应的恢复策略
        let strategy = self.recovery_strategies.iter()
            .find(|s| s.fault_type == fault_type)
            .copied()
            .unwrap_or(RecoveryStrategy {
                fault_type,
                recovery_action: RecoveryAction::SoftReset,
                max_attempts: 1,
                timeout_ms: 1000,
            });
        
        // 检查恢复尝试次数
        if self.recovery_attempts >= self.max_recovery_attempts {
            return RecoveryResult::MaxAttemptsExceeded;
        }
        
        // 执行恢复操作
        let recovery_successful = self.execute_recovery(&strategy, fault_info);
        
        // 记录故障和恢复信息
        let fault_record = FaultRecord {
            fault_type,
            timestamp: self.get_timestamp(),
            recovery_action: strategy.recovery_action,
            recovery_successful,
            additional_info: fault_info,
        };
        
        let _ = self.fault_history.push(fault_record);
        
        if recovery_successful {
            self.recovery_attempts = 0; // 重置计数器
            RecoveryResult::Success
        } else {
            self.recovery_attempts += 1;
            RecoveryResult::Failed
        }
    }
    
    fn execute_recovery(&self, strategy: &RecoveryStrategy, fault_info: u32) -> bool {
        match strategy.recovery_action {
            RecoveryAction::SoftReset => {
                self.perform_soft_reset()
            }
            RecoveryAction::HardReset => {
                self.perform_hard_reset()
            }
            RecoveryAction::ReconfigurePeripheral => {
                self.reconfigure_peripheral(fault_info)
            }
            RecoveryAction::SwitchToBackup => {
                self.switch_to_backup_system()
            }
            RecoveryAction::EnterSafeMode => {
                self.enter_safe_mode()
            }
            RecoveryAction::RestartService => {
                self.restart_service(fault_info)
            }
        }
    }
    
    fn perform_soft_reset(&self) -> bool {
        // 执行软件复位
        unsafe {
            SCB::sys_reset();
        }
    }
    
    fn perform_hard_reset(&self) -> bool {
        // 执行硬件复位
        // 通过外部复位引脚或看门狗复位
        true
    }
    
    fn reconfigure_peripheral(&self, peripheral_id: u32) -> bool {
        // 重新配置外设
        match peripheral_id {
            1 => self.reconfigure_uart(),
            2 => self.reconfigure_spi(),
            3 => self.reconfigure_i2c(),
            _ => false,
        }
    }
    
    fn reconfigure_uart(&self) -> bool {
        // 重新配置UART
        true
    }
    
    fn reconfigure_spi(&self) -> bool {
        // 重新配置SPI
        true
    }
    
    fn reconfigure_i2c(&self) -> bool {
        // 重新配置I2C
        true
    }
    
    fn switch_to_backup_system(&self) -> bool {
        // 切换到备用系统
        true
    }
    
    fn enter_safe_mode(&self) -> bool {
        // 进入安全模式
        true
    }
    
    fn restart_service(&self, service_id: u32) -> bool {
        // 重启服务
        true
    }
    
    fn get_timestamp(&self) -> u32 {
        // 获取当前时间戳
        0
    }
    
    pub fn get_fault_statistics(&self) -> FaultStatistics {
        let mut stats = FaultStatistics::default();
        
        for record in &self.fault_history {
            stats.total_faults += 1;
            
            match record.fault_type {
                FaultType::HardFault => stats.hard_faults += 1,
                FaultType::WatchdogTimeout => stats.watchdog_timeouts += 1,
                FaultType::ClockFailure => stats.clock_failures += 1,
                FaultType::CommunicationFailure => stats.communication_failures += 1,
                _ => stats.other_faults += 1,
            }
            
            if record.recovery_successful {
                stats.successful_recoveries += 1;
            }
        }
        
        stats.recovery_success_rate = if stats.total_faults > 0 {
            (stats.successful_recoveries as f32 / stats.total_faults as f32) * 100.0
        } else {
            0.0
        };
        
        stats
    }
}

#[derive(Debug, Clone, Copy)]
pub enum RecoveryResult {
    Success,
    Failed,
    MaxAttemptsExceeded,
}

#[derive(Debug, Default)]
pub struct FaultStatistics {
    pub total_faults: u32,
    pub hard_faults: u32,
    pub watchdog_timeouts: u32,
    pub clock_failures: u32,
    pub communication_failures: u32,
    pub other_faults: u32,
    pub successful_recoveries: u32,
    pub recovery_success_rate: f32,
}
```

## 6. 总结

启动过程和引导是嵌入式系统的基础，正确的启动流程确保系统能够可靠地从上电到正常运行。

### 关键要点

1. **启动流程**: 理解完整的启动阶段和各阶段的职责
2. **引导程序**: 设计可靠的引导加载程序和固件更新机制
3. **向量表**: 正确配置向量表和异常处理程序
4. **系统初始化**: 按正确顺序初始化各个子系统
5. **故障恢复**: 实现完善的故障检测和恢复机制
6. **安全启动**: 考虑安全性和完整性验证

### 设计原则

- **可靠性优先**: 确保启动过程的可靠性和稳定性
- **快速启动**: 优化启动时间，满足实时性要求
- **故障容错**: 具备故障检测和恢复能力
- **安全性**: 实现安全启动和完整性验证
- **可维护性**: 支持固件更新和远程维护

### 实践建议

- 建立完整的启动时序图和检查点
- 实现启动性能监控和优化
- 设计多级故障恢复机制
- 使用硬件看门狗确保系统可靠性
- 实现安全的固件更新机制

### 下一步

完成启动过程学习后，建议继续学习：
- [通信协议基础](./07-communication-protocols.md)
- [GPIO控制实践](../../04-gpio-control/README.md)
- [串口通信](../../05-serial-communication/README.md)