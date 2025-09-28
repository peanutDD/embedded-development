//! # HAL抽象层演示程序
//!
//! 演示GPIO HAL抽象层的各种功能和用法

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use log::info;
use panic_halt as _;

use hal_abstraction::{
  async_gpio::{AsyncGpioPin, GpioWaiter, WaitCondition},
  error::HalError,
  gpio::{GpioPin, InputPin, OutputPin, PinMode},
  interrupt::{InterruptManager, InterruptMode, SimpleCallback},
  platform::mock::MockPlatform,
};

#[entry]
fn main() -> ! {
  // 初始化日志
  env_logger::init();

  info!("启动GPIO HAL抽象层演示程序");

  // 演示基本GPIO操作
  demo_basic_gpio().unwrap();

  // 演示中断处理
  demo_interrupt_handling().unwrap();

  // 演示异步GPIO操作
  demo_async_gpio().unwrap();

  // 演示错误处理
  demo_error_handling();

  // 演示平台抽象
  demo_platform_abstraction().unwrap();

  info!("所有演示完成");

  loop {
    // 主循环
    cortex_m::asm::wfi();
  }
}

/// 演示基本GPIO操作
fn demo_basic_gpio() -> Result<(), HalError> {
  info!("=== 基本GPIO操作演示 ===");

  let mut platform = MockPlatform::new();

  // 获取输出引脚
  let mut led_pin = platform.get_pin(13)?;
  led_pin.set_mode(PinMode::Output)?;

  info!("LED引脚配置为输出模式");

  // 控制LED
  for i in 0..5 {
    led_pin.set_high()?;
    info!("LED开启 ({})", i + 1);
    delay_ms(500);

    led_pin.set_low()?;
    info!("LED关闭 ({})", i + 1);
    delay_ms(500);
  }

  // 获取输入引脚
  let mut button_pin = platform.get_pin(2)?;
  button_pin.set_mode(PinMode::Input)?;

  info!("按钮引脚配置为输入模式");

  // 读取按钮状态
  for i in 0..3 {
    let state = button_pin.is_high()?;
    info!(
      "按钮状态读取 {}: {}",
      i + 1,
      if state { "按下" } else { "释放" }
    );
    delay_ms(1000);
  }

  Ok(())
}

/// 演示中断处理
fn demo_interrupt_handling() -> Result<(), HalError> {
  info!("=== 中断处理演示 ===");

  let mut platform = MockPlatform::new();
  let mut interrupt_manager = InterruptManager::<16>::new();

  // 配置中断引脚
  let mut interrupt_pin = platform.get_pin(3)?;
  interrupt_pin.set_mode(PinMode::Input)?;

  // 创建中断回调
  let callback = Box::new(SimpleCallback::new(1, || {
    info!("中断触发！");
  }));

  // 注册中断回调
  interrupt_manager.register_callback(3, callback)?;

  info!("中断回调已注册");

  // 模拟中断触发
  for i in 0..3 {
    info!("模拟中断触发 {}", i + 1);
    interrupt_manager.handle_interrupt(3);
    delay_ms(1000);
  }

  // 获取中断统计信息
  if let Some(stats) = interrupt_manager.get_statistics(3) {
    info!(
      "中断统计: 总数={}, 处理={}, 丢失={}, 成功率={}%",
      stats.total_interrupts,
      stats.handled_interrupts,
      stats.missed_interrupts,
      stats.success_rate()
    );
  }

  Ok(())
}

/// 演示异步GPIO操作
fn demo_async_gpio() -> Result<(), HalError> {
  info!("=== 异步GPIO操作演示 ===");

  let mut platform = MockPlatform::new();

  // 获取异步引脚
  let mut async_pin = platform.get_pin(4)?;
  async_pin.set_mode(PinMode::Input)?;

  info!("配置异步GPIO引脚");

  // 创建GPIO等待器
  let mut waiter = GpioWaiter::new(async_pin, WaitCondition::High);
  waiter = waiter.with_timeout(5000); // 5秒超时

  info!("创建GPIO等待器，等待高电平信号");

  // 在实际应用中，这里会使用async/await
  // 这里只是演示API的使用
  info!("异步等待演示完成");

  Ok(())
}

/// 演示错误处理
fn demo_error_handling() {
  info!("=== 错误处理演示 ===");

  let mut platform = MockPlatform::new();

  // 尝试访问不存在的引脚
  match platform.get_pin(100) {
    Ok(_) => info!("意外成功获取引脚100"),
    Err(e) => info!("预期错误: {:?}", e),
  }

  // 尝试在错误模式下操作引脚
  if let Ok(mut pin) = platform.get_pin(5) {
    // 在输入模式下尝试设置输出
    let _ = pin.set_mode(PinMode::Input);
    match pin.set_high() {
      Ok(_) => info!("意外成功设置输出"),
      Err(e) => info!("预期错误: {:?}", e),
    }
  }

  info!("错误处理演示完成");
}

/// 演示平台抽象
fn demo_platform_abstraction() -> Result<(), HalError> {
  info!("=== 平台抽象演示 ===");

  let platform = MockPlatform::new();
  let platform_info = platform.get_platform_info();

  info!("平台信息:");
  info!("  名称: {}", platform_info.name);
  info!("  引脚数量: {}", platform_info.pin_count);
  info!("  支持的特性: {:?}", platform_info.features);
  info!("  时钟频率: {} MHz", platform_info.clock_frequency_mhz);

  // 演示引脚能力查询
  for pin_num in 0..8 {
    if let Ok(pin) = platform.get_pin(pin_num) {
      let capabilities = pin.get_capabilities();
      info!("引脚 {} 能力: {:?}", pin_num, capabilities);
    }
  }

  Ok(())
}

/// 简单延时函数 (模拟)
fn delay_ms(_ms: u32) {
  // 在实际应用中，这里应该使用系统定时器或延时函数
  // 这里只是一个占位符
  for _ in 0..1000 {
    cortex_m::asm::nop();
  }
}

/// 演示高级GPIO功能
fn demo_advanced_features() -> Result<(), HalError> {
  info!("=== 高级GPIO功能演示 ===");

  let mut platform = MockPlatform::new();

  // 演示批量操作
  let pins = [
    platform.get_pin(8)?,
    platform.get_pin(9)?,
    platform.get_pin(10)?,
    platform.get_pin(11)?,
  ];

  info!("配置4个引脚用于批量操作");

  // 配置所有引脚为输出
  for (i, mut pin) in pins.into_iter().enumerate() {
    pin.set_mode(PinMode::Output)?;
    info!("引脚 {} 配置为输出", 8 + i);
  }

  // 演示引脚状态查询
  let mut input_pin = platform.get_pin(12)?;
  input_pin.set_mode(PinMode::Input)?;

  let pin_info = input_pin.get_pin_info();
  info!(
    "引脚12信息: 编号={}, 端口={}, 模式={:?}",
    pin_info.pin_number, pin_info.port, pin_info.mode
  );

  Ok(())
}

/// 演示性能测试
fn demo_performance_test() -> Result<(), HalError> {
  info!("=== 性能测试演示 ===");

  let mut platform = MockPlatform::new();
  let mut test_pin = platform.get_pin(15)?;
  test_pin.set_mode(PinMode::Output)?;

  info!("开始GPIO切换性能测试");

  let start_time = get_current_time_us();

  // 执行1000次切换
  for _ in 0..1000 {
    test_pin.set_high()?;
    test_pin.set_low()?;
  }

  let end_time = get_current_time_us();
  let duration = end_time - start_time;

  info!("1000次GPIO切换耗时: {} 微秒", duration);
  info!("平均每次切换: {} 纳秒", duration * 1000 / 2000);

  Ok(())
}

/// 获取当前时间 (微秒) - 模拟实现
fn get_current_time_us() -> u32 {
  // 在实际应用中，这里应该使用系统定时器
  0
}

/// 演示配置验证
fn demo_configuration_validation() -> Result<(), HalError> {
  info!("=== 配置验证演示 ===");

  let mut platform = MockPlatform::new();

  // 测试有效配置
  let mut pin = platform.get_pin(1)?;

  // 测试模式切换
  pin.set_mode(PinMode::Output)?;
  info!("引脚1配置为输出模式 - 成功");

  pin.set_mode(PinMode::Input)?;
  info!("引脚1配置为输入模式 - 成功");

  // 测试无效操作
  match pin.set_high() {
    Ok(_) => info!("在输入模式下设置输出 - 意外成功"),
    Err(e) => info!("在输入模式下设置输出 - 预期失败: {:?}", e),
  }

  Ok(())
}

/// 演示资源管理
fn demo_resource_management() -> Result<(), HalError> {
  info!("=== 资源管理演示 ===");

  let mut platform = MockPlatform::new();

  // 获取多个引脚
  let pins: Result<Vec<_, 8>, _> = (0..8).map(|i| platform.get_pin(i)).collect();

  match pins {
    Ok(pin_vec) => {
      info!("成功获取 {} 个引脚", pin_vec.len());

      // 演示引脚释放
      drop(pin_vec);
      info!("引脚资源已释放");
    }
    Err(e) => {
      info!("获取引脚失败: {:?}", e);
    }
  }

  Ok(())
}
