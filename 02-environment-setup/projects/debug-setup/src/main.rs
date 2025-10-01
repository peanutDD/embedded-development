//! # 调试工具配置验证项目
//!
//! 本项目用于验证各种调试工具的配置和功能，包括：
//! - RTT (Real Time Transfer) 实时传输
//! - 半主机调试 (Semihosting)
//! - 断点和单步调试
//! - 内存查看和修改
//! - 性能分析

#![no_std]
#![no_main]

use cortex_m::{asm, peripheral::DWT};
use cortex_m_rt::entry;
use panic_halt as _;

// RTT支持
use cortex_m_rtt::{rprint, rprintln};
use rtt_target::{rprintln as rtt_rprintln, rtt_init_print};

// 半主机支持
use cortex_m_semihosting::{debug, hprintln};

// 日志支持
use log::{debug, error, info, trace, warn};

/// 调试器状态
#[derive(Debug, Clone, Copy)]
enum DebuggerState {
  NotConnected,
  Connected,
  RTTActive,
  SemihostingActive,
}

/// 调试配置结构体
struct DebugConfig {
  rtt_enabled: bool,
  semihosting_enabled: bool,
  dwt_enabled: bool,
  breakpoints_enabled: bool,
}

impl DebugConfig {
  fn new() -> Self {
    Self {
      rtt_enabled: cfg!(feature = "rtt-log"),
      semihosting_enabled: cfg!(feature = "semihosting-log"),
      dwt_enabled: true,
      breakpoints_enabled: true,
    }
  }

  fn detect_debugger(&self) -> DebuggerState {
    // 尝试检测调试器连接状态
    // 这是一个简化的检测方法
    if self.rtt_enabled {
      DebuggerState::RTTActive
    } else if self.semihosting_enabled {
      DebuggerState::SemihostingActive
    } else {
      DebuggerState::NotConnected
    }
  }
}

/// 性能计数器
struct PerformanceCounter {
  start_cycle: u32,
  end_cycle: u32,
}

impl PerformanceCounter {
  fn new() -> Self {
    Self {
      start_cycle: 0,
      end_cycle: 0,
    }
  }

  fn start(&mut self) {
    self.start_cycle = DWT::cycle_count();
  }

  fn stop(&mut self) {
    self.end_cycle = DWT::cycle_count();
  }

  fn elapsed_cycles(&self) -> u32 {
    self.end_cycle.wrapping_sub(self.start_cycle)
  }
}

/// 内存区域信息
#[derive(Debug)]
struct MemoryRegion {
  name: &'static str,
  start_addr: usize,
  size: usize,
}

impl MemoryRegion {
  fn end_addr(&self) -> usize {
    self.start_addr + self.size
  }

  fn contains(&self, addr: usize) -> bool {
    addr >= self.start_addr && addr < self.end_addr()
  }
}

/// 调试输出宏
macro_rules! debug_output {
    ($($arg:tt)*) => {
        #[cfg(feature = "rtt-log")]
        {
            rprintln!($($arg)*);
            rtt_rprintln!($($arg)*);
        }

        #[cfg(feature = "semihosting-log")]
        {
            hprintln!($($arg)*).ok();
        }

        // 总是尝试log输出
        info!($($arg)*);
    };
}

/// 初始化调试系统
fn init_debug_system() -> DebugConfig {
  // 初始化RTT
  #[cfg(feature = "rtt-log")]
  {
    rtt_init_print!();
    rprintln!("RTT初始化完成");
  }

  // 初始化DWT（数据观察点和跟踪）
  let mut core = cortex_m::Peripherals::take().unwrap();
  core.DCB.enable_trace();
  core.DWT.enable_cycle_counter();

  debug_output!("调试系统初始化完成");

  DebugConfig::new()
}

/// 测试RTT功能
fn test_rtt_functionality() {
  debug_output!("=== RTT功能测试 ===");

  #[cfg(feature = "rtt-log")]
  {
    rprintln!("RTT基本输出测试");

    // 测试不同类型的输出
    rprintln!("整数输出: {}", 42);
    rprintln!("浮点输出: {:.2}", 3.14159);
    rprintln!("十六进制: 0x{:08X}", 0xDEADBEEF);
    rprintln!("二进制: 0b{:08b}", 0b10101010);

    // 测试实时输出
    rprint!("实时输出测试: ");
    for i in 0..10 {
      rprint!("{} ", i);
      // 小延迟以观察实时效果
      for _ in 0..10000 {
        asm::nop();
      }
    }
    rprintln!("");

    rprintln!("RTT功能测试完成");
  }

  #[cfg(not(feature = "rtt-log"))]
  {
    debug_output!("RTT功能未启用");
  }
}

/// 测试半主机功能
fn test_semihosting_functionality() {
  debug_output!("=== 半主机功能测试 ===");

  #[cfg(feature = "semihosting-log")]
  {
    hprintln!("半主机基本输出测试").ok();
    hprintln!("注意：半主机需要调试器支持").ok();

    // 测试不同的半主机功能
    hprintln!("系统信息输出测试").ok();
    hprintln!("时间戳: {}", cortex_m::peripheral::DWT::cycle_count()).ok();

    // 注意：某些半主机功能可能需要特定的调试器支持
    debug_output!("半主机功能测试完成");
  }

  #[cfg(not(feature = "semihosting-log"))]
  {
    debug_output!("半主机功能未启用");
  }
}

/// 测试断点功能
fn test_breakpoint_functionality() {
  debug_output!("=== 断点功能测试 ===");

  debug_output!("设置软件断点...");

  let mut counter = 0;

  // 循环中的断点测试
  for i in 0..5 {
    counter += i;

    if i == 2 {
      debug_output!("到达断点位置 (i={})", i);
      // 软件断点 - 在调试器中会停止
      asm::bkpt();
    }
  }

  debug_output!("断点测试完成，计数器值: {}", counter);
}

/// 测试内存查看功能
fn test_memory_inspection() {
  debug_output!("=== 内存查看功能测试 ===");

  // 定义一些测试数据
  let test_array: [u32; 8] = [
    0x12345678, 0x9ABCDEF0, 0xFEDCBA98, 0x76543210, 0x11111111, 0x22222222, 0x33333333, 0x44444444,
  ];

  let test_struct = TestStruct {
    field1: 0xAABBCCDD,
    field2: 0x1234,
    field3: 0x56,
  };

  // 输出内存地址信息
  debug_output!("测试数组地址: 0x{:08X}", test_array.as_ptr() as usize);
  debug_output!(
    "测试结构体地址: 0x{:08X}",
    &test_struct as *const _ as usize
  );

  // 输出内存内容
  debug_output!("数组内容:");
  for (i, &value) in test_array.iter().enumerate() {
    debug_output!("  [{}]: 0x{:08X}", i, value);
  }

  debug_output!("结构体内容:");
  debug_output!("  field1: 0x{:08X}", test_struct.field1);
  debug_output!("  field2: 0x{:04X}", test_struct.field2);
  debug_output!("  field3: 0x{:02X}", test_struct.field3);

  // 内存区域信息
  let memory_regions = get_memory_regions();
  debug_output!("内存区域信息:");
  for region in &memory_regions {
    debug_output!(
      "  {}: 0x{:08X} - 0x{:08X} ({} bytes)",
      region.name,
      region.start_addr,
      region.end_addr(),
      region.size
    );
  }
}

/// 测试结构体
#[repr(C)]
struct TestStruct {
  field1: u32,
  field2: u16,
  field3: u8,
}

/// 获取内存区域信息
fn get_memory_regions() -> [MemoryRegion; 4] {
  [
    MemoryRegion {
      name: "FLASH",
      start_addr: 0x08000000,
      size: 512 * 1024, // 512KB
    },
    MemoryRegion {
      name: "SRAM",
      start_addr: 0x20000000,
      size: 128 * 1024, // 128KB
    },
    MemoryRegion {
      name: "CCM",
      start_addr: 0x10000000,
      size: 64 * 1024, // 64KB
    },
    MemoryRegion {
      name: "PERIPHERAL",
      start_addr: 0x40000000,
      size: 512 * 1024 * 1024, // 512MB
    },
  ]
}

/// 测试性能分析功能
fn test_performance_analysis() {
  debug_output!("=== 性能分析功能测试 ===");

  let mut perf_counter = PerformanceCounter::new();

  // 测试简单计算的性能
  perf_counter.start();
  let mut sum = 0u32;
  for i in 0..1000 {
    sum = sum.wrapping_add(i * i);
  }
  perf_counter.stop();

  debug_output!("计算1000次平方和:");
  debug_output!("  结果: {}", sum);
  debug_output!("  耗时: {} 周期", perf_counter.elapsed_cycles());

  // 测试内存访问性能
  let test_data: [u32; 100] = [0x12345678; 100];

  perf_counter.start();
  let mut checksum = 0u32;
  for &value in &test_data {
    checksum ^= value;
  }
  perf_counter.stop();

  debug_output!("内存访问测试:");
  debug_output!("  校验和: 0x{:08X}", checksum);
  debug_output!("  耗时: {} 周期", perf_counter.elapsed_cycles());

  // 测试函数调用开销
  perf_counter.start();
  for _ in 0..100 {
    dummy_function();
  }
  perf_counter.stop();

  debug_output!("函数调用开销测试:");
  debug_output!("  100次调用耗时: {} 周期", perf_counter.elapsed_cycles());
  debug_output!(
    "  平均每次调用: {} 周期",
    perf_counter.elapsed_cycles() / 100
  );
}

/// 虚拟函数用于测试调用开销
#[inline(never)]
fn dummy_function() {
  asm::nop();
}

/// 测试日志系统
fn test_logging_system() {
  debug_output!("=== 日志系统测试 ===");

  // 测试不同级别的日志
  trace!("这是TRACE级别日志");
  debug!("这是DEBUG级别日志");
  info!("这是INFO级别日志");
  warn!("这是WARN级别日志");
  error!("这是ERROR级别日志");

  // 测试格式化日志
  let value = 42;
  let name = "测试";
  info!("格式化日志测试: {} = {}", name, value);

  debug_output!("日志系统测试完成");
}

/// 主程序入口
#[entry]
fn main() -> ! {
  // 初始化调试系统
  let debug_config = init_debug_system();

  debug_output!("=== Rust嵌入式调试工具验证 ===");
  debug_output!("");

  // 显示调试配置
  debug_output!("调试配置:");
  debug_output!("  RTT启用: {}", debug_config.rtt_enabled);
  debug_output!("  半主机启用: {}", debug_config.semihosting_enabled);
  debug_output!("  DWT启用: {}", debug_config.dwt_enabled);
  debug_output!("  断点启用: {}", debug_config.breakpoints_enabled);
  debug_output!("  调试器状态: {:?}", debug_config.detect_debugger());
  debug_output!("");

  // 执行各项调试功能测试
  test_rtt_functionality();
  debug_output!("");

  test_semihosting_functionality();
  debug_output!("");

  test_logging_system();
  debug_output!("");

  test_memory_inspection();
  debug_output!("");

  test_performance_analysis();
  debug_output!("");

  test_breakpoint_functionality();
  debug_output!("");

  debug_output!("=== 所有调试功能测试完成 ===");
  debug_output!("程序将进入无限循环，可以在调试器中观察状态");

  let mut loop_counter = 0u32;

  // 主循环 - 在调试器中可以观察变量变化
  loop {
    loop_counter = loop_counter.wrapping_add(1);

    // 每1000次循环输出一次状态
    if loop_counter % 1000 == 0 {
      debug_output!("循环计数: {}", loop_counter);
    }

    // 每10000次循环设置一个断点
    if loop_counter % 10000 == 0 {
      debug_output!("定期断点 - 循环计数: {}", loop_counter);
      asm::bkpt();
    }

    // 节省功耗
    asm::wfi();
  }
}
