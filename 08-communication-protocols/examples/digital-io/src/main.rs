#![no_std]
#![no_main]

//! # 数字IO控制主程序
//! 
//! 演示数字IO控制的各种功能：
//! - 数字输出控制
//! - 数字输入处理
//! - 矩阵键盘扫描
//! - LED矩阵显示

use panic_halt as _;
use cortex_m_rt::entry;
use embedded_hal::digital::{InputPin, OutputPin};
use heapless::{Vec, String};
use critical_section::Mutex;
use core::cell::RefCell;

use digital_io::{
    DigitalOutputController, DigitalInputController,
    MatrixKeypadController, ShiftRegisterController,
    OutputConfig, InputConfig, ScanConfig, DebounceConfig,
    ShiftRegisterConfig, PinState, PinMode, EdgeDetection,
    KeyEvent, DriveStrength, SlewRate, OutputMode,
};

/// 系统控制器
pub struct SystemController {
    /// 数字输出控制器
    output_controller: DigitalOutputController<16>,
    /// 数字输入控制器
    input_controller: DigitalInputController<16>,
    /// 矩阵键盘控制器
    keypad_controller: MatrixKeypadController<4, 4>,
    /// 移位寄存器控制器
    shift_register: ShiftRegisterController<4>,
    /// 系统状态
    system_state: SystemState,
    /// 运行统计
    stats: SystemStats,
}

/// 系统状态
#[derive(Debug, Clone, Copy)]
pub struct SystemState {
    pub mode: OperationMode,
    pub led_pattern: LedPattern,
    pub input_mask: u16,
    pub output_mask: u16,
    pub system_time: u32,
}

/// 操作模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperationMode {
    Manual,      // 手动控制
    Automatic,   // 自动模式
    Pattern,     // 模式输出
    Interactive, // 交互模式
}

/// LED模式
#[derive(Debug, Clone, Copy)]
pub enum LedPattern {
    Off,
    On,
    Blink,
    Chase,
    Fade,
    Random,
}

/// 系统统计
#[derive(Debug, Default)]
pub struct SystemStats {
    pub uptime: u32,
    pub input_events: u32,
    pub output_changes: u32,
    pub key_presses: u32,
    pub mode_changes: u32,
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            mode: OperationMode::Manual,
            led_pattern: LedPattern::Off,
            input_mask: 0,
            output_mask: 0,
            system_time: 0,
        }
    }
}

impl SystemController {
    /// 创建新的系统控制器
    pub fn new() -> Self {
        // 配置输出引脚
        let output_configs = [
            OutputConfig {
                mode: PinMode::PushPull,
                initial_state: PinState::Low,
                drive_strength: DriveStrength::Medium,
                slew_rate: SlewRate::Medium,
            }; 16
        ];
        
        // 配置输入引脚
        let input_configs = [
            InputConfig {
                mode: PinMode::InputPullUp,
                edge_detection: EdgeDetection::Both,
                enable_debounce: true,
                enable_interrupt: true,
            }; 16
        ];
        
        // 防抖配置
        let debounce_config = DebounceConfig {
            debounce_time_ms: 50,
            stable_count: 3,
            enable_hysteresis: true,
        };
        
        // 扫描配置
        let scan_config = ScanConfig {
            scan_interval_ms: 10,
            hold_threshold_ms: 500,
            repeat_interval_ms: 100,
            enable_ghost_detection: true,
        };
        
        // 移位寄存器配置
        let shift_config = ShiftRegisterConfig::default();
        
        Self {
            output_controller: DigitalOutputController::new(output_configs),
            input_controller: DigitalInputController::new(input_configs, debounce_config),
            keypad_controller: MatrixKeypadController::new(scan_config, debounce_config),
            shift_register: ShiftRegisterController::new(shift_config),
            system_state: SystemState::default(),
            stats: SystemStats::default(),
        }
    }
    
    /// 初始化系统
    pub fn init(&mut self) -> Result<(), ()> {
        // 设置初始输出状态
        self.output_controller.set_output_mode(OutputMode::Individual);
        
        // 初始化LED模式
        self.set_led_pattern(LedPattern::Blink)?;
        
        // 重置统计
        self.output_controller.reset_stats();
        self.input_controller.reset_stats();
        
        Ok(())
    }
    
    /// 主循环处理
    pub fn process(&mut self, timestamp: u32) -> Result<(), ()> {
        self.system_state.system_time = timestamp;
        self.stats.uptime = timestamp;
        
        // 处理输入
        self.process_inputs(timestamp)?;
        
        // 处理键盘
        self.process_keypad(timestamp)?;
        
        // 更新输出
        self.update_outputs(timestamp)?;
        
        // 处理LED模式
        self.process_led_pattern(timestamp)?;
        
        Ok(())
    }
    
    /// 处理输入
    fn process_inputs(&mut self, timestamp: u32) -> Result<(), ()> {
        // 模拟读取硬件输入状态
        let hardware_inputs = self.read_hardware_inputs();
        
        for (pin, &state) in hardware_inputs.iter().enumerate() {
            let pin_state = if state { PinState::High } else { PinState::Low };
            
            if let Ok(edge_detected) = self.input_controller.update_pin_state(pin, pin_state, timestamp) {
                if edge_detected {
                    self.stats.input_events += 1;
                    self.handle_input_event(pin, pin_state)?;
                }
            }
        }
        
        // 更新输入掩码
        if let Ok(input_states) = self.input_controller.read_pins(0xFFFF) {
            self.system_state.input_mask = input_states as u16;
        }
        
        Ok(())
    }
    
    /// 读取硬件输入（模拟）
    fn read_hardware_inputs(&self) -> [bool; 16] {
        // 在实际应用中，这里应该读取真实的硬件状态
        // 这里模拟一些输入变化
        let time = self.system_state.system_time;
        let mut inputs = [false; 16];
        
        // 模拟按钮输入
        inputs[0] = (time / 1000) % 2 == 0;  // 每秒切换
        inputs[1] = (time / 2000) % 2 == 0;  // 每2秒切换
        inputs[2] = (time / 500) % 2 == 0;   // 每0.5秒切换
        
        inputs
    }
    
    /// 处理输入事件
    fn handle_input_event(&mut self, pin: usize, state: PinState) -> Result<(), ()> {
        match pin {
            0 => {
                // 模式切换按钮
                if state == PinState::High {
                    self.cycle_operation_mode()?;
                }
            },
            1 => {
                // LED模式切换按钮
                if state == PinState::High {
                    self.cycle_led_pattern()?;
                }
            },
            2 => {
                // 重置按钮
                if state == PinState::High {
                    self.reset_system()?;
                }
            },
            _ => {
                // 其他输入处理
            }
        }
        
        Ok(())
    }
    
    /// 处理键盘
    fn process_keypad(&mut self, timestamp: u32) -> Result<(), ()> {
        // 模拟键盘行状态
        let row_states = self.scan_keypad_rows();
        let events = self.keypad_controller.scan_matrix(row_states, timestamp);
        
        for event in events {
            self.handle_keypad_event(event)?;
        }
        
        Ok(())
    }
    
    /// 扫描键盘行（模拟）
    fn scan_keypad_rows(&self) -> [bool; 4] {
        // 在实际应用中，这里应该扫描键盘矩阵
        // 模拟一些按键活动
        let time = self.system_state.system_time;
        [
            (time / 3000) % 2 == 0,
            false,
            false,
            false,
        ]
    }
    
    /// 处理键盘事件
    fn handle_keypad_event(&mut self, event: KeyEvent) -> Result<(), ()> {
        match event {
            KeyEvent::Press { row, col } => {
                self.stats.key_presses += 1;
                
                // 根据按键位置执行不同操作
                match (row, col) {
                    (0, 0) => self.set_led_pattern(LedPattern::On)?,
                    (0, 1) => self.set_led_pattern(LedPattern::Blink)?,
                    (0, 2) => self.set_led_pattern(LedPattern::Chase)?,
                    (0, 3) => self.set_led_pattern(LedPattern::Off)?,
                    _ => {}
                }
            },
            KeyEvent::Hold { row, col, duration } => {
                // 长按处理
                if duration > 1000 {
                    match (row, col) {
                        (0, 0) => self.reset_system()?,
                        _ => {}
                    }
                }
            },
            KeyEvent::Release { .. } => {
                // 释放处理
            },
            KeyEvent::Ghost => {
                // 幽灵按键检测
            }
        }
        
        Ok(())
    }
    
    /// 更新输出
    fn update_outputs(&mut self, _timestamp: u32) -> Result<(), ()> {
        // 根据系统状态更新输出
        match self.system_state.mode {
            OperationMode::Manual => {
                // 手动模式：直接映射输入到输出
                let input_mask = self.system_state.input_mask;
                self.output_controller.set_pins(0xFFFF, input_mask as u32)?;
            },
            OperationMode::Automatic => {
                // 自动模式：根据预设逻辑控制输出
                self.automatic_output_control()?;
            },
            OperationMode::Pattern => {
                // 模式输出：由LED模式控制
            },
            OperationMode::Interactive => {
                // 交互模式：响应用户输入
                self.interactive_output_control()?;
            }
        }
        
        self.stats.output_changes += 1;
        Ok(())
    }
    
    /// 自动输出控制
    fn automatic_output_control(&mut self) -> Result<(), ()> {
        let time = self.system_state.system_time;
        
        // 创建自动模式的输出模式
        let pattern = match (time / 1000) % 4 {
            0 => 0x0001,  // 单个LED
            1 => 0x0003,  // 两个LED
            2 => 0x000F,  // 四个LED
            3 => 0xFFFF,  // 全部LED
            _ => 0x0000,
        };
        
        self.output_controller.set_pins(0xFFFF, pattern)?;
        Ok(())
    }
    
    /// 交互输出控制
    fn interactive_output_control(&mut self) -> Result<(), ()> {
        // 根据输入状态创建交互式输出
        let input_mask = self.system_state.input_mask;
        
        // 输入反转输出
        let output_pattern = !input_mask;
        self.output_controller.set_pins(0xFFFF, output_pattern as u32)?;
        
        Ok(())
    }
    
    /// 处理LED模式
    fn process_led_pattern(&mut self, timestamp: u32) -> Result<(), ()> {
        match self.system_state.led_pattern {
            LedPattern::Off => {
                self.output_controller.set_pins(0xFF00, 0x0000)?;
            },
            LedPattern::On => {
                self.output_controller.set_pins(0xFF00, 0xFF00)?;
            },
            LedPattern::Blink => {
                let pattern = if (timestamp / 500) % 2 == 0 { 0xFF00 } else { 0x0000 };
                self.output_controller.set_pins(0xFF00, pattern)?;
            },
            LedPattern::Chase => {
                let pos = (timestamp / 200) % 8;
                let pattern = 0x0100 << pos;
                self.output_controller.set_pins(0xFF00, pattern)?;
            },
            LedPattern::Fade => {
                // 渐变效果（简化版）
                let intensity = ((timestamp / 100) % 20) as u32;
                let pattern = if intensity < 10 { 
                    (1 << intensity) - 1 
                } else { 
                    (1 << (20 - intensity)) - 1 
                } << 8;
                self.output_controller.set_pins(0xFF00, pattern)?;
            },
            LedPattern::Random => {
                // 伪随机模式
                let pattern = (timestamp.wrapping_mul(1103515245).wrapping_add(12345) >> 8) & 0xFF00;
                self.output_controller.set_pins(0xFF00, pattern)?;
            }
        }
        
        Ok(())
    }
    
    /// 循环操作模式
    fn cycle_operation_mode(&mut self) -> Result<(), ()> {
        self.system_state.mode = match self.system_state.mode {
            OperationMode::Manual => OperationMode::Automatic,
            OperationMode::Automatic => OperationMode::Pattern,
            OperationMode::Pattern => OperationMode::Interactive,
            OperationMode::Interactive => OperationMode::Manual,
        };
        
        self.stats.mode_changes += 1;
        Ok(())
    }
    
    /// 循环LED模式
    fn cycle_led_pattern(&mut self) -> Result<(), ()> {
        self.set_led_pattern(match self.system_state.led_pattern {
            LedPattern::Off => LedPattern::On,
            LedPattern::On => LedPattern::Blink,
            LedPattern::Blink => LedPattern::Chase,
            LedPattern::Chase => LedPattern::Fade,
            LedPattern::Fade => LedPattern::Random,
            LedPattern::Random => LedPattern::Off,
        })
    }
    
    /// 设置LED模式
    fn set_led_pattern(&mut self, pattern: LedPattern) -> Result<(), ()> {
        self.system_state.led_pattern = pattern;
        Ok(())
    }
    
    /// 重置系统
    fn reset_system(&mut self) -> Result<(), ()> {
        self.system_state = SystemState::default();
        self.stats = SystemStats::default();
        
        // 重置所有控制器
        self.output_controller.reset_stats();
        self.input_controller.reset_stats();
        
        // 清除所有输出
        self.output_controller.set_pins(0xFFFF, 0x0000)?;
        
        Ok(())
    }
    
    /// 获取系统状态
    pub fn get_system_state(&self) -> &SystemState {
        &self.system_state
    }
    
    /// 获取系统统计
    pub fn get_system_stats(&self) -> &SystemStats {
        &self.stats
    }
    
    /// 获取详细统计信息
    pub fn get_detailed_stats(&self) -> DetailedStats {
        DetailedStats {
            system: self.stats,
            output: *self.output_controller.get_stats(),
            input: *self.input_controller.get_stats(),
            keypad: *self.keypad_controller.get_stats(),
            shift_register: *self.shift_register.get_stats(),
        }
    }
}

/// 详细统计信息
#[derive(Debug)]
pub struct DetailedStats {
    pub system: SystemStats,
    pub output: digital_io::OutputStats,
    pub input: digital_io::InputStats,
    pub keypad: digital_io::KeypadStats,
    pub shift_register: digital_io::ShiftRegisterStats,
}

/// 全局系统控制器
static SYSTEM_CONTROLLER: Mutex<RefCell<Option<SystemController>>> = 
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 初始化系统控制器
    let mut system = SystemController::new();
    system.init().unwrap();
    
    // 存储到全局变量
    critical_section::with(|cs| {
        SYSTEM_CONTROLLER.borrow(cs).replace(Some(system));
    });
    
    let mut timestamp = 0u32;
    
    loop {
        // 更新时间戳
        timestamp = timestamp.wrapping_add(1);
        
        // 处理系统
        critical_section::with(|cs| {
            if let Some(ref mut system) = SYSTEM_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                let _ = system.process(timestamp);
                
                // 每1000次循环输出一次状态（模拟）
                if timestamp % 1000 == 0 {
                    let state = system.get_system_state();
                    let stats = system.get_system_stats();
                    
                    // 在实际应用中，这里可以通过串口或其他方式输出状态
                    // 这里只是模拟处理
                }
            }
        });
        
        // 简单延时
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }
    }
}

/// 中断处理程序示例
#[allow(non_snake_case)]
fn GPIO_IRQHandler() {
    critical_section::with(|cs| {
        if let Some(ref mut system) = SYSTEM_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
            // 处理GPIO中断
            let timestamp = system.get_system_state().system_time;
            let _ = system.process_inputs(timestamp);
        }
    });
}

/// 定时器中断处理程序示例
#[allow(non_snake_case)]
fn TIMER_IRQHandler() {
    critical_section::with(|cs| {
        if let Some(ref mut system) = SYSTEM_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
            // 处理定时器中断
            let timestamp = system.get_system_state().system_time.wrapping_add(1);
            let _ = system.process(timestamp);
        }
    });
}