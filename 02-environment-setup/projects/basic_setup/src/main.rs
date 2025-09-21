//! # 基础环境搭建验证项目
//! 
//! 本项目用于验证Rust嵌入式开发环境是否正确配置。
//! 包含工具链检查、目标架构验证、调试功能测试等。

#![no_std]
#![no_main]

use panic_halt as _;

#[cfg(feature = "rtt")]
use cortex_m_rtt::{rprint, rprintln};

#[cfg(feature = "semihosting")]
use cortex_m_semihosting::{debug, hprintln};

use cortex_m_rt::entry;
use cortex_m::asm;

/// 环境信息结构体
#[derive(Debug)]
struct EnvironmentInfo {
    target_arch: &'static str,
    target_os: &'static str,
    target_family: &'static str,
    pointer_width: usize,
}

impl EnvironmentInfo {
    /// 获取当前环境信息
    fn new() -> Self {
        Self {
            target_arch: env!("CARGO_CFG_TARGET_ARCH"),
            target_os: env!("CARGO_CFG_TARGET_OS"),
            target_family: env!("CARGO_CFG_TARGET_FAMILY"),
            pointer_width: core::mem::size_of::<*const ()>() * 8,
        }
    }

    /// 验证环境配置
    fn validate(&self) -> bool {
        // 检查是否为嵌入式目标
        let is_embedded = self.target_os == "none" && 
                         (self.target_arch == "arm" || 
                          self.target_arch == "riscv32" || 
                          self.target_arch == "xtensa");
        
        // 检查指针宽度
        let valid_pointer_width = match self.target_arch {
            "arm" => self.pointer_width == 32,
            "riscv32" => self.pointer_width == 32,
            "xtensa" => self.pointer_width == 32,
            _ => false,
        };

        is_embedded && valid_pointer_width
    }
}

/// 工具链验证模块
mod toolchain_check {
    /// 检查编译器版本信息
    pub fn check_compiler_info() -> &'static str {
        env!("RUSTC_VERSION")
    }

    /// 检查目标三元组
    pub fn check_target_triple() -> &'static str {
        env!("TARGET")
    }

    /// 检查优化级别
    pub fn check_optimization_level() -> &'static str {
        if cfg!(debug_assertions) {
            "Debug"
        } else {
            "Release"
        }
    }
}

/// 内存布局验证
mod memory_check {
    use core::mem;

    /// 检查基本类型大小
    pub fn check_type_sizes() -> TypeSizes {
        TypeSizes {
            u8_size: mem::size_of::<u8>(),
            u16_size: mem::size_of::<u16>(),
            u32_size: mem::size_of::<u32>(),
            u64_size: mem::size_of::<u64>(),
            usize_size: mem::size_of::<usize>(),
            ptr_size: mem::size_of::<*const ()>(),
        }
    }

    #[derive(Debug)]
    pub struct TypeSizes {
        pub u8_size: usize,
        pub u16_size: usize,
        pub u32_size: usize,
        pub u64_size: usize,
        pub usize_size: usize,
        pub ptr_size: usize,
    }

    impl TypeSizes {
        pub fn validate(&self) -> bool {
            self.u8_size == 1 &&
            self.u16_size == 2 &&
            self.u32_size == 4 &&
            self.u64_size == 8 &&
            self.usize_size == self.ptr_size
        }
    }
}

/// 功能特性检查
mod feature_check {
    /// 检查RTT功能
    #[cfg(feature = "rtt")]
    pub fn check_rtt() -> bool {
        true
    }

    #[cfg(not(feature = "rtt"))]
    pub fn check_rtt() -> bool {
        false
    }

    /// 检查半主机功能
    #[cfg(feature = "semihosting")]
    pub fn check_semihosting() -> bool {
        true
    }

    #[cfg(not(feature = "semihosting"))]
    pub fn check_semihosting() -> bool {
        false
    }

    /// 检查平台特定功能
    #[cfg(feature = "stm32f4")]
    pub fn check_stm32f4() -> bool {
        true
    }

    #[cfg(not(feature = "stm32f4"))]
    pub fn check_stm32f4() -> bool {
        false
    }

    #[cfg(feature = "nrf52")]
    pub fn check_nrf52() -> bool {
        true
    }

    #[cfg(not(feature = "nrf52"))]
    pub fn check_nrf52() -> bool {
        false
    }
}

/// 输出宏，根据可用功能选择输出方式
macro_rules! output {
    ($($arg:tt)*) => {
        #[cfg(feature = "rtt")]
        rprintln!($($arg)*);
        
        #[cfg(all(feature = "semihosting", not(feature = "rtt")))]
        hprintln!($($arg)*).ok();
    };
}

/// 主程序入口点
#[entry]
fn main() -> ! {
    output!("=== Rust嵌入式开发环境验证 ===");
    output!("");

    // 1. 环境信息检查
    let env_info = EnvironmentInfo::new();
    output!("环境信息:");
    output!("  目标架构: {}", env_info.target_arch);
    output!("  目标操作系统: {}", env_info.target_os);
    output!("  目标系列: {}", env_info.target_family);
    output!("  指针宽度: {} bits", env_info.pointer_width);
    
    let env_valid = env_info.validate();
    output!("  环境验证: {}", if env_valid { "✓ 通过" } else { "✗ 失败" });
    output!("");

    // 2. 工具链信息检查
    output!("工具链信息:");
    output!("  编译器版本: {}", toolchain_check::check_compiler_info());
    output!("  目标三元组: {}", toolchain_check::check_target_triple());
    output!("  优化级别: {}", toolchain_check::check_optimization_level());
    output!("");

    // 3. 内存布局检查
    let type_sizes = memory_check::check_type_sizes();
    output!("内存布局:");
    output!("  u8: {} bytes", type_sizes.u8_size);
    output!("  u16: {} bytes", type_sizes.u16_size);
    output!("  u32: {} bytes", type_sizes.u32_size);
    output!("  u64: {} bytes", type_sizes.u64_size);
    output!("  usize: {} bytes", type_sizes.usize_size);
    output!("  指针: {} bytes", type_sizes.ptr_size);
    
    let memory_valid = type_sizes.validate();
    output!("  内存布局验证: {}", if memory_valid { "✓ 通过" } else { "✗ 失败" });
    output!("");

    // 4. 功能特性检查
    output!("功能特性:");
    output!("  RTT支持: {}", if feature_check::check_rtt() { "✓ 启用" } else { "✗ 禁用" });
    output!("  半主机支持: {}", if feature_check::check_semihosting() { "✓ 启用" } else { "✗ 禁用" });
    output!("  STM32F4支持: {}", if feature_check::check_stm32f4() { "✓ 启用" } else { "✗ 禁用" });
    output!("  nRF52支持: {}", if feature_check::check_nrf52() { "✓ 启用" } else { "✗ 禁用" });
    output!("");

    // 5. 总体验证结果
    let overall_valid = env_valid && memory_valid;
    output!("=== 验证结果 ===");
    if overall_valid {
        output!("✓ 环境配置正确，可以开始嵌入式开发！");
    } else {
        output!("✗ 环境配置存在问题，请检查工具链安装");
    }
    output!("");

    // 6. 性能测试
    output!("=== 性能测试 ===");
    performance_test();

    // 7. 调试功能测试
    debug_test();

    output!("验证完成，程序将进入无限循环");
    
    // 进入无限循环
    loop {
        asm::wfi(); // 等待中断，节省功耗
    }
}

/// 性能测试函数
fn performance_test() {
    output!("执行性能测试...");
    
    // 简单的计算性能测试
    let start = cortex_m::peripheral::DWT::cycle_count();
    
    let mut sum = 0u32;
    for i in 0..1000 {
        sum = sum.wrapping_add(i);
    }
    
    let end = cortex_m::peripheral::DWT::cycle_count();
    let cycles = end.wrapping_sub(start);
    
    output!("  计算1000次累加耗时: {} 周期", cycles);
    output!("  结果: {}", sum);
}

/// 调试功能测试
fn debug_test() {
    output!("=== 调试功能测试 ===");
    
    #[cfg(feature = "rtt")]
    {
        output!("RTT调试输出正常工作");
        rprint!("RTT实时输出测试: ");
        for i in 0..5 {
            rprint!("{} ", i);
        }
        rprintln!("");
    }
    
    #[cfg(feature = "semihosting")]
    {
        output!("半主机调试输出正常工作");
        // 注意：半主机调试可能需要调试器连接
    }
    
    // 测试断点功能（在调试器中可见）
    cortex_m::asm::bkpt();
    
    output!("调试功能测试完成");
}

/// 错误处理测试
#[allow(dead_code)]
fn error_handling_test() {
    output!("=== 错误处理测试 ===");
    
    // 测试除零错误（在debug模式下会panic）
    #[cfg(debug_assertions)]
    {
        output!("Debug模式：启用溢出检查");
    }
    
    #[cfg(not(debug_assertions))]
    {
        output!("Release模式：禁用溢出检查");
    }
    
    output!("错误处理测试完成");
}