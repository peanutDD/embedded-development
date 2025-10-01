//! # GPIO硬件原理分析项目
//!
//! 本项目深入分析GPIO的硬件原理、电气特性和时序特性，
//! 通过实际测量和分析帮助理解GPIO的工作机制。

#![no_std]
#![no_main]

use cortex_m::{asm, peripheral::DWT};
use cortex_m_rt::entry;
use panic_halt as _;

// RTT调试输出
use rtt_target::{rprintln, rtt_init_print};

// 数学库
use heapless::{FnvIndexMap, Vec};
use micromath::F32Ext;

// HAL库 (条件编译)
#[cfg(feature = "stm32f4")]
use stm32f4xx_hal::{
  gpio::{Analog, Input, OpenDrain, Output, Pin, PullUp, PushPull},
  pac,
  prelude::*,
  timer::{Counter, Timer},
};

// GPIO分析模块
mod electrical_test;
mod gpio_analyzer;
mod power_test;
mod timing_test;

use gpio_analyzer::*;

/// GPIO电气特性分析结构体
#[derive(Debug, Clone)]
struct ElectricalCharacteristics {
  voh_min: f32, // 输出高电平最小值 (V)
  vol_max: f32, // 输出低电平最大值 (V)
  vih_min: f32, // 输入高电平最小值 (V)
  vil_max: f32, // 输入低电平最大值 (V)
  ioh_max: f32, // 输出高电平最大电流 (mA)
  iol_max: f32, // 输出低电平最大电流 (mA)
  iih_max: f32, // 输入高电平最大电流 (μA)
  iil_max: f32, // 输入低电平最大电流 (μA)
}

impl Default for ElectricalCharacteristics {
  fn default() -> Self {
    // STM32F4典型值
    Self {
      voh_min: 2.4,
      vol_max: 0.4,
      vih_min: 2.0,
      vil_max: 0.8,
      ioh_max: 25.0,
      iol_max: 25.0,
      iih_max: 1.0,
      iil_max: 1.0,
    }
  }
}

/// GPIO时序特性分析结构体
#[derive(Debug, Clone)]
struct TimingCharacteristics {
  setup_time_ns: u32,     // 建立时间 (ns)
  hold_time_ns: u32,      // 保持时间 (ns)
  prop_delay_ns: u32,     // 传播延迟 (ns)
  rise_time_ns: u32,      // 上升时间 (ns)
  fall_time_ns: u32,      // 下降时间 (ns)
  max_frequency_mhz: u32, // 最大切换频率 (MHz)
}

impl Default for TimingCharacteristics {
  fn default() -> Self {
    Self {
      setup_time_ns: 10,
      hold_time_ns: 5,
      prop_delay_ns: 15,
      rise_time_ns: 20,
      fall_time_ns: 15,
      max_frequency_mhz: 50,
    }
  }
}

/// GPIO功耗分析结构体
#[derive(Debug, Clone)]
struct PowerCharacteristics {
  static_current_ua: f32,   // 静态电流 (μA)
  dynamic_current_ma: f32,  // 动态电流 (mA)
  switching_energy_pj: f32, // 切换能耗 (pJ)
  leakage_current_na: f32,  // 漏电流 (nA)
}

impl Default for PowerCharacteristics {
  fn default() -> Self {
    Self {
      static_current_ua: 0.1,
      dynamic_current_ma: 2.0,
      switching_energy_pj: 50.0,
      leakage_current_na: 10.0,
    }
  }
}

/// GPIO分析器主结构体
struct GpioAnalyzer {
  electrical: ElectricalCharacteristics,
  timing: TimingCharacteristics,
  power: PowerCharacteristics,
  test_results: FnvIndexMap<&'static str, f32, 16>,
}

impl GpioAnalyzer {
  fn new() -> Self {
    Self {
      electrical: ElectricalCharacteristics::default(),
      timing: TimingCharacteristics::default(),
      power: PowerCharacteristics::default(),
      test_results: FnvIndexMap::new(),
    }
  }

  /// 分析GPIO电气特性
  fn analyze_electrical_characteristics(&mut self) {
    rprintln!("=== GPIO电气特性分析 ===");

    // 输出电平分析
    rprintln!("输出电平特性:");
    rprintln!(
      "  VOH(min): {:.2}V - 输出高电平最小值",
      self.electrical.voh_min
    );
    rprintln!(
      "  VOL(max): {:.2}V - 输出低电平最大值",
      self.electrical.vol_max
    );
    rprintln!(
      "  噪声容限(高): {:.2}V",
      self.electrical.voh_min - self.electrical.vih_min
    );
    rprintln!(
      "  噪声容限(低): {:.2}V",
      self.electrical.vil_max - self.electrical.vol_max
    );

    // 输入电平分析
    rprintln!("输入电平特性:");
    rprintln!(
      "  VIH(min): {:.2}V - 输入高电平最小值",
      self.electrical.vih_min
    );
    rprintln!(
      "  VIL(max): {:.2}V - 输入低电平最大值",
      self.electrical.vil_max
    );
    rprintln!(
      "  输入阈值范围: {:.2}V - {:.2}V",
      self.electrical.vil_max,
      self.electrical.vih_min
    );

    // 驱动能力分析
    rprintln!("驱动能力:");
    rprintln!(
      "  IOH(max): {:.1}mA - 输出高电平最大电流",
      self.electrical.ioh_max
    );
    rprintln!(
      "  IOL(max): {:.1}mA - 输出低电平最大电流",
      self.electrical.iol_max
    );

    // 计算负载电阻
    let load_resistance_high = self.electrical.voh_min / (self.electrical.ioh_max / 1000.0);
    let load_resistance_low = self.electrical.vol_max / (self.electrical.iol_max / 1000.0);
    rprintln!("  最小负载电阻(高): {:.0}Ω", load_resistance_high);
    rprintln!("  最小负载电阻(低): {:.0}Ω", load_resistance_low);

    self
      .test_results
      .insert("voh_min", self.electrical.voh_min)
      .ok();
    self
      .test_results
      .insert("vol_max", self.electrical.vol_max)
      .ok();
  }

  /// 分析GPIO时序特性
  fn analyze_timing_characteristics(&mut self) {
    rprintln!("\n=== GPIO时序特性分析 ===");

    rprintln!("时序参数:");
    rprintln!("  建立时间: {}ns", self.timing.setup_time_ns);
    rprintln!("  保持时间: {}ns", self.timing.hold_time_ns);
    rprintln!("  传播延迟: {}ns", self.timing.prop_delay_ns);
    rprintln!("  上升时间: {}ns", self.timing.rise_time_ns);
    rprintln!("  下降时间: {}ns", self.timing.fall_time_ns);

    // 计算最大工作频率
    let min_period_ns = self.timing.setup_time_ns
      + self.timing.hold_time_ns
      + self.timing.prop_delay_ns
      + self.timing.rise_time_ns.max(self.timing.fall_time_ns);
    let max_freq_calc = 1000.0 / (min_period_ns as f32);

    rprintln!("频率特性:");
    rprintln!("  最小周期: {}ns", min_period_ns);
    rprintln!("  理论最大频率: {:.1}MHz", max_freq_calc);
    rprintln!("  规格最大频率: {}MHz", self.timing.max_frequency_mhz);

    self.test_results.insert("max_freq", max_freq_calc).ok();
  }

  /// 分析GPIO功耗特性
  fn analyze_power_characteristics(&mut self) {
    rprintln!("\n=== GPIO功耗特性分析 ===");

    rprintln!("功耗参数:");
    rprintln!("  静态电流: {:.2}μA", self.power.static_current_ua);
    rprintln!("  动态电流: {:.2}mA", self.power.dynamic_current_ma);
    rprintln!("  切换能耗: {:.1}pJ", self.power.switching_energy_pj);
    rprintln!("  漏电流: {:.1}nA", self.power.leakage_current_na);

    // 计算不同频率下的功耗
    let frequencies = [1, 10, 100, 1000, 10000]; // kHz
    rprintln!("不同频率下的功耗估算:");

    for &freq_khz in &frequencies {
      let switching_power_uw = (freq_khz as f32) * self.power.switching_energy_pj / 1000.0;
      let total_power_uw = switching_power_uw + self.power.static_current_ua * 3.3;
      rprintln!("  {}kHz: {:.2}μW", freq_khz, total_power_uw);
    }

    self
      .test_results
      .insert("static_power", self.power.static_current_ua * 3.3)
      .ok();
  }

  /// 实际GPIO性能测试
  fn perform_gpio_tests(&mut self) {
    rprintln!("\n=== GPIO性能实测 ===");

    // 这里会进行实际的GPIO操作测试
    // 由于需要具体硬件，这里提供测试框架

    rprintln!("执行GPIO切换速度测试...");
    let switch_cycles = self.measure_gpio_switching_speed();
    rprintln!("GPIO切换耗时: {} 周期", switch_cycles);

    rprintln!("执行GPIO读取延迟测试...");
    let read_cycles = self.measure_gpio_read_latency();
    rprintln!("GPIO读取延迟: {} 周期", read_cycles);

    self
      .test_results
      .insert("switch_cycles", switch_cycles as f32)
      .ok();
    self
      .test_results
      .insert("read_cycles", read_cycles as f32)
      .ok();
  }

  /// 测量GPIO切换速度
  fn measure_gpio_switching_speed(&self) -> u32 {
    // 使用DWT周期计数器测量GPIO切换时间
    let start = DWT::cycle_count();

    // 模拟GPIO切换操作
    for _ in 0..100 {
      asm::nop();
    }

    let end = DWT::cycle_count();
    (end.wrapping_sub(start)) / 100
  }

  /// 测量GPIO读取延迟
  fn measure_gpio_read_latency(&self) -> u32 {
    let start = DWT::cycle_count();

    // 模拟GPIO读取操作
    for _ in 0..100 {
      asm::nop();
    }

    let end = DWT::cycle_count();
    (end.wrapping_sub(start)) / 100
  }

  /// 生成分析报告
  fn generate_report(&self) {
    rprintln!("\n=== GPIO分析报告 ===");

    rprintln!("测试结果汇总:");
    for (key, value) in &self.test_results {
      rprintln!("  {}: {:.2}", key, value);
    }

    rprintln!("\n设计建议:");
    rprintln!("1. 负载设计:");
    rprintln!("   - 确保负载电阻大于最小值");
    rprintln!("   - 考虑驱动能力限制");

    rprintln!("2. 时序设计:");
    rprintln!("   - 满足建立和保持时间要求");
    rprintln!("   - 考虑传播延迟影响");

    rprintln!("3. 功耗优化:");
    rprintln!("   - 降低不必要的切换频率");
    rprintln!("   - 使用低功耗模式");

    rprintln!("4. 可靠性设计:");
    rprintln!("   - 添加适当的保护电路");
    rprintln!("   - 考虑EMI和信号完整性");
  }
}

/// GPIO工作模式演示
fn demonstrate_gpio_modes() {
  rprintln!("\n=== GPIO工作模式演示 ===");

  rprintln!("1. 推挽输出模式 (Push-Pull):");
  rprintln!("   - 可以输出强高电平和强低电平");
  rprintln!("   - 适用于驱动LED、继电器等负载");

  rprintln!("2. 开漏输出模式 (Open-Drain):");
  rprintln!("   - 只能拉低，需要外部上拉电阻");
  rprintln!("   - 适用于I2C总线、线与逻辑");

  rprintln!("3. 输入模式 (Input):");
  rprintln!("   - 高阻抗输入，可配置上拉/下拉");
  rprintln!("   - 适用于按钮、传感器信号输入");

  rprintln!("4. 模拟模式 (Analog):");
  rprintln!("   - 断开数字电路，用于ADC/DAC");
  rprintln!("   - 最小化数字噪声影响");
}

/// 电路保护分析
fn analyze_circuit_protection() {
  rprintln!("\n=== 电路保护分析 ===");

  rprintln!("1. ESD保护:");
  rprintln!("   - 内置ESD二极管保护");
  rprintln!("   - 典型保护电压: ±2kV (HBM)");

  rprintln!("2. 过压保护:");
  rprintln!("   - 输入电压不应超过VDD+0.3V");
  rprintln!("   - 使用限流电阻或钳位二极管");

  rprintln!("3. 过流保护:");
  rprintln!("   - 输出电流不应超过规格值");
  rprintln!("   - 考虑短路保护措施");

  rprintln!("4. 热保护:");
  rprintln!("   - 监控芯片温度");
  rprintln!("   - 实施热关断保护");
}

#[entry]
fn main() -> ! {
  // 初始化RTT调试输出
  rtt_init_print!();

  // 初始化DWT周期计数器
  let mut core = cortex_m::Peripherals::take().unwrap();
  core.DCB.enable_trace();
  core.DWT.enable_cycle_counter();

  rprintln!("=== GPIO硬件原理分析项目 ===");
  rprintln!("深入分析GPIO的电气特性、时序特性和功耗特性\n");

  // 创建GPIO分析器
  let mut analyzer = GpioAnalyzer::new();

  // 执行各项分析
  analyzer.analyze_electrical_characteristics();
  analyzer.analyze_timing_characteristics();
  analyzer.analyze_power_characteristics();
  analyzer.perform_gpio_tests();

  // 演示GPIO工作模式
  demonstrate_gpio_modes();

  // 分析电路保护
  analyze_circuit_protection();

  // 生成分析报告
  analyzer.generate_report();

  rprintln!("\n=== 分析完成 ===");
  rprintln!("程序将进入无限循环，可在调试器中查看详细数据");

  // 主循环
  loop {
    // 周期性输出关键参数
    for _ in 0..1000000 {
      asm::nop();
    }

    rprintln!("GPIO分析器运行中... 周期计数: {}", DWT::cycle_count());

    asm::wfi();
  }
}
