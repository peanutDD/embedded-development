//! # 类型安全GPIO设计项目
//! 
//! 本项目展示如何使用Rust的类型系统构建零成本抽象的GPIO接口，
//! 实现编译时安全检查和状态管理。

#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::asm;

// RTT调试输出
use rtt_target::{rtt_init_print, rprintln};

// 类型级编程
use typenum::{U0, U1, U2, U3, U4, U5, Unsigned};
use generic_array::{GenericArray, ArrayLength};
use heapless::{Vec, String};

// 状态机
#[cfg(feature = "state-machines")]
use smlang::statemachine;

/// GPIO引脚状态的类型级表示
pub mod pin_states {
    /// 未配置状态
    pub struct Unconfigured;
    
    /// 输入状态
    pub struct Input<MODE> {
        _mode: core::marker::PhantomData<MODE>,
    }
    
    /// 输出状态
    pub struct Output<MODE> {
        _mode: core::marker::PhantomData<MODE>,
    }
    
    /// 模拟状态
    pub struct Analog;
    
    /// 输入模式
    pub mod input_modes {
        pub struct Floating;
        pub struct PullUp;
        pub struct PullDown;
    }
    
    /// 输出模式
    pub mod output_modes {
        pub struct PushPull;
        pub struct OpenDrain;
    }
}

use pin_states::*;

/// 类型安全的GPIO引脚抽象
pub struct Pin<const N: u8, STATE> {
    _pin_number: core::marker::PhantomData<typenum::consts::U8>,
    _state: core::marker::PhantomData<STATE>,
}

impl<const N: u8> Pin<N, Unconfigured> {
    /// 创建未配置的引脚
    pub const fn new() -> Self {
        Self {
            _pin_number: core::marker::PhantomData,
            _state: core::marker::PhantomData,
        }
    }
    
    /// 配置为输入模式
    pub fn into_input<MODE>(self) -> Pin<N, Input<MODE>> {
        rprintln!("配置引脚 {} 为输入模式", N);
        Pin {
            _pin_number: core::marker::PhantomData,
            _state: core::marker::PhantomData,
        }
    }
    
    /// 配置为输出模式
    pub fn into_output<MODE>(self) -> Pin<N, Output<MODE>> {
        rprintln!("配置引脚 {} 为输出模式", N);
        Pin {
            _pin_number: core::marker::PhantomData,
            _state: core::marker::PhantomData,
        }
    }
    
    /// 配置为模拟模式
    pub fn into_analog(self) -> Pin<N, Analog> {
        rprintln!("配置引脚 {} 为模拟模式", N);
        Pin {
            _pin_number: core::marker::PhantomData,
            _state: core::marker::PhantomData,
        }
    }
}

impl<const N: u8, MODE> Pin<N, Input<MODE>> {
    /// 读取输入值
    pub fn is_high(&self) -> bool {
        // 模拟读取硬件寄存器
        rprintln!("读取引脚 {} 输入状态", N);
        true // 模拟值
    }
    
    /// 读取输入值（低电平检测）
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }
    
    /// 重新配置为输出模式
    pub fn into_output<OMODE>(self) -> Pin<N, Output<OMODE>> {
        rprintln!("重新配置引脚 {} 从输入到输出", N);
        Pin {
            _pin_number: core::marker::PhantomData,
            _state: core::marker::PhantomData,
        }
    }
}

impl<const N: u8, MODE> Pin<N, Output<MODE>> {
    /// 设置输出高电平
    pub fn set_high(&mut self) {
        rprintln!("设置引脚 {} 输出高电平", N);
        // 实际硬件操作
    }
    
    /// 设置输出低电平
    pub fn set_low(&mut self) {
        rprintln!("设置引脚 {} 输出低电平", N);
        // 实际硬件操作
    }
    
    /// 切换输出状态
    pub fn toggle(&mut self) {
        rprintln!("切换引脚 {} 输出状态", N);
        // 实际硬件操作
    }
    
    /// 重新配置为输入模式
    pub fn into_input<IMODE>(self) -> Pin<N, Input<IMODE>> {
        rprintln!("重新配置引脚 {} 从输出到输入", N);
        Pin {
            _pin_number: core::marker::PhantomData,
            _state: core::marker::PhantomData,
        }
    }
}

/// 编译时引脚验证
pub trait PinValidator<const N: u8> {
    const IS_VALID: bool;
    
    fn validate() -> Result<(), &'static str> {
        if Self::IS_VALID {
            Ok(())
        } else {
            Err("无效的引脚编号")
        }
    }
}

/// 为有效引脚实现验证器
impl PinValidator<0> for Pin<0, Unconfigured> { const IS_VALID: bool = true; }
impl PinValidator<1> for Pin<1, Unconfigured> { const IS_VALID: bool = true; }
impl PinValidator<2> for Pin<2, Unconfigured> { const IS_VALID: bool = true; }
impl PinValidator<3> for Pin<3, Unconfigured> { const IS_VALID: bool = true; }
impl PinValidator<4> for Pin<4, Unconfigured> { const IS_VALID: bool = true; }
impl PinValidator<5> for Pin<5, Unconfigured> { const IS_VALID: bool = true; }

/// GPIO端口抽象
pub struct GpioPort<const PORT: char, const PIN_COUNT: usize> {
    pins: GenericArray<bool, typenum::U32>, // 最大32个引脚
}

impl<const PORT: char, const PIN_COUNT: usize> GpioPort<PORT, PIN_COUNT> {
    pub fn new() -> Self {
        rprintln!("初始化GPIO端口 {} ({}个引脚)", PORT, PIN_COUNT);
        Self {
            pins: GenericArray::default(),
        }
    }
    
    /// 获取引脚
    pub fn pin<const N: u8>(&self) -> Pin<N, Unconfigured> 
    where
        Pin<N, Unconfigured>: PinValidator<N>
    {
        Pin::<N, Unconfigured>::validate().expect("引脚验证失败");
        Pin::new()
    }
    
    /// 批量配置引脚
    pub fn configure_pins(&mut self, config: &[(u8, &str)]) {
        rprintln!("批量配置端口 {} 的引脚:", PORT);
        for &(pin_num, mode) in config {
            rprintln!("  引脚 {}: {}", pin_num, mode);
        }
    }
}

/// LED控制器 - 类型安全的LED抽象
pub struct Led<const N: u8> {
    pin: Pin<N, Output<output_modes::PushPull>>,
    is_active_high: bool,
}

impl<const N: u8> Led<N> {
    /// 创建LED控制器
    pub fn new(pin: Pin<N, Output<output_modes::PushPull>>, active_high: bool) -> Self {
        rprintln!("创建LED控制器 (引脚 {}, 高电平有效: {})", N, active_high);
        Self {
            pin,
            is_active_high: active_high,
        }
    }
    
    /// 点亮LED
    pub fn on(&mut self) {
        if self.is_active_high {
            self.pin.set_high();
        } else {
            self.pin.set_low();
        }
        rprintln!("LED {} 点亮", N);
    }
    
    /// 熄灭LED
    pub fn off(&mut self) {
        if self.is_active_high {
            self.pin.set_low();
        } else {
            self.pin.set_high();
        }
        rprintln!("LED {} 熄灭", N);
    }
    
    /// 切换LED状态
    pub fn toggle(&mut self) {
        self.pin.toggle();
        rprintln!("LED {} 切换状态", N);
    }
}

/// 按钮控制器 - 类型安全的按钮抽象
pub struct Button<const N: u8> {
    pin: Pin<N, Input<input_modes::PullUp>>,
    is_active_low: bool,
}

impl<const N: u8> Button<N> {
    /// 创建按钮控制器
    pub fn new(pin: Pin<N, Input<input_modes::PullUp>>, active_low: bool) -> Self {
        rprintln!("创建按钮控制器 (引脚 {}, 低电平有效: {})", N, active_low);
        Self {
            pin,
            is_active_low: active_low,
        }
    }
    
    /// 检查按钮是否被按下
    pub fn is_pressed(&self) -> bool {
        let pin_state = self.pin.is_high();
        let pressed = if self.is_active_low {
            !pin_state
        } else {
            pin_state
        };
        
        if pressed {
            rprintln!("按钮 {} 被按下", N);
        }
        pressed
    }
    
    /// 等待按钮按下
    pub fn wait_for_press(&self) {
        rprintln!("等待按钮 {} 按下...", N);
        while !self.is_pressed() {
            asm::nop();
        }
    }
}

/// GPIO状态机示例
#[cfg(feature = "state-machines")]
statemachine! {
    transitions: {
        *Idle + ButtonPress = Active,
        Active + Timeout = Idle,
        Active + ButtonPress = Idle,
    }
}

/// 零成本抽象演示
pub mod zero_cost {
    use super::*;
    
    /// 高级GPIO操作 - 编译时优化
    pub struct GpioOperations;
    
    impl GpioOperations {
        /// 原子操作 - 编译时内联
        #[inline(always)]
        pub fn atomic_set_multiple<const N1: u8, const N2: u8>(
            pin1: &mut Pin<N1, Output<output_modes::PushPull>>,
            pin2: &mut Pin<N2, Output<output_modes::PushPull>>,
            state1: bool,
            state2: bool,
        ) {
            // 编译器会将此优化为单个寄存器操作
            if state1 { pin1.set_high(); } else { pin1.set_low(); }
            if state2 { pin2.set_high(); } else { pin2.set_low(); }
        }
        
        /// 条件操作 - 编译时分支预测
        #[inline(always)]
        pub fn conditional_toggle<const N: u8>(
            pin: &mut Pin<N, Output<output_modes::PushPull>>,
            condition: bool,
        ) {
            if condition {
                pin.toggle();
            }
        }
    }
}

/// 编译时配置验证
pub mod compile_time_checks {
    use super::*;
    
    /// 引脚冲突检测
    pub struct PinConflictChecker<const PINS: &'static [u8]>;
    
    impl<const PINS: &'static [u8]> PinConflictChecker<PINS> {
        pub const fn check_conflicts() -> bool {
            // 编译时检查引脚冲突
            let mut i = 0;
            while i < PINS.len() {
                let mut j = i + 1;
                while j < PINS.len() {
                    if PINS[i] == PINS[j] {
                        return false; // 发现冲突
                    }
                    j += 1;
                }
                i += 1;
            }
            true
        }
    }
    
    /// 配置验证宏
    macro_rules! validate_pin_config {
        ($($pin:expr),*) => {
            const PINS: &[u8] = &[$($pin),*];
            const _: () = assert!(
                PinConflictChecker::<PINS>::check_conflicts(),
                "检测到引脚冲突"
            );
        };
    }
    
    pub(crate) use validate_pin_config;
}

/// 性能基准测试
pub mod benchmarks {
    use super::*;
    
    pub struct GpioBenchmark;
    
    impl GpioBenchmark {
        /// 测量GPIO操作延迟
        pub fn measure_gpio_latency() -> u32 {
            let start = cortex_m::peripheral::DWT::cycle_count();
            
            // 模拟GPIO操作
            for _ in 0..1000 {
                asm::nop();
            }
            
            let end = cortex_m::peripheral::DWT::cycle_count();
            end.wrapping_sub(start) / 1000
        }
        
        /// 测量类型转换开销
        pub fn measure_type_conversion_overhead() -> u32 {
            let pin = Pin::<0, Unconfigured>::new();
            
            let start = cortex_m::peripheral::DWT::cycle_count();
            
            let _output_pin = pin.into_output::<output_modes::PushPull>();
            
            let end = cortex_m::peripheral::DWT::cycle_count();
            end.wrapping_sub(start)
        }
    }
}

/// 应用程序示例
fn demonstrate_type_safety() {
    rprintln!("\n=== 类型安全GPIO演示 ===");
    
    // 创建GPIO端口
    let port_a = GpioPort::<'A', 16>::new();
    
    // 获取引脚并配置
    let pin0 = port_a.pin::<0>();
    let pin1 = port_a.pin::<1>();
    let pin2 = port_a.pin::<2>();
    
    // 配置LED (输出)
    let output_pin = pin0.into_output::<output_modes::PushPull>();
    let mut led = Led::new(output_pin, true);
    
    // 配置按钮 (输入)
    let input_pin = pin1.into_input::<input_modes::PullUp>();
    let button = Button::new(input_pin, true);
    
    // 配置模拟引脚
    let _analog_pin = pin2.into_analog();
    
    // 演示操作
    led.on();
    led.off();
    led.toggle();
    
    if button.is_pressed() {
        rprintln!("按钮被按下!");
    }
    
    // 编译时验证
    compile_time_checks::validate_pin_config!(0, 1, 2);
    rprintln!("编译时引脚配置验证通过");
}

fn demonstrate_zero_cost_abstractions() {
    rprintln!("\n=== 零成本抽象演示 ===");
    
    let port = GpioPort::<'B', 8>::new();
    let pin3 = port.pin::<3>().into_output::<output_modes::PushPull>();
    let mut pin4 = port.pin::<4>().into_output::<output_modes::PushPull>();
    
    // 这些操作在优化编译后将变成直接的寄存器操作
    zero_cost::GpioOperations::atomic_set_multiple(&mut pin4, &mut pin4, true, false);
    zero_cost::GpioOperations::conditional_toggle(&mut pin4, true);
    
    rprintln!("零成本抽象操作完成");
}

fn run_benchmarks() {
    rprintln!("\n=== 性能基准测试 ===");
    
    let latency = benchmarks::GpioBenchmark::measure_gpio_latency();
    rprintln!("GPIO操作延迟: {} 周期", latency);
    
    let conversion_overhead = benchmarks::GpioBenchmark::measure_type_conversion_overhead();
    rprintln!("类型转换开销: {} 周期", conversion_overhead);
    
    if conversion_overhead == 0 {
        rprintln!("✓ 零成本抽象验证成功 - 类型转换无运行时开销");
    }
}

#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    
    // 初始化DWT周期计数器
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();
    
    rprintln!("=== 类型安全GPIO设计项目 ===");
    rprintln!("展示Rust类型系统在嵌入式GPIO控制中的应用\n");
    
    // 演示类型安全
    demonstrate_type_safety();
    
    // 演示零成本抽象
    demonstrate_zero_cost_abstractions();
    
    // 运行性能基准测试
    run_benchmarks();
    
    rprintln!("\n=== 设计原则总结 ===");
    rprintln!("1. 类型安全: 编译时防止配置错误");
    rprintln!("2. 零成本抽象: 运行时无额外开销");
    rprintln!("3. 状态管理: 类型级状态跟踪");
    rprintln!("4. 编译时验证: 早期错误检测");
    rprintln!("5. 可组合性: 模块化设计");
    
    rprintln!("\n程序完成，进入无限循环");
    
    // 主循环
    loop {
        asm::wfi();
    }
}