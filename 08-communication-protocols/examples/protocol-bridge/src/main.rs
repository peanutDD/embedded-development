#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use heapless::Vec;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{Alternate, Input, OpenDrain, Output, Pin, PullUp, PushPull},
  i2c::{I2c, Mode as I2cMode},
  pac::{self, Peripherals, TIM2},
  prelude::*,
  rcc::Clocks,
  spi::{Mode, Phase, Polarity, Spi},
  timer::{Event, Timer},
};

// 导入库模块
use protocol_bridge::{
  bridge::{BridgeConfig, BridgeOperation, BridgeState, ProtocolBridge},
  i2c_slave::{I2cSlave, I2cSlaveManager, SlaveConfig},
  protocol::{OperationType, ProtocolMessage},
  spi_master::{BitOrder, SpiDeviceConfig, SpiDeviceManager, SpiMaster, SpiMode},
};

/// 系统状态
#[derive(Debug, Clone, Copy)]
struct SystemState {
  uptime_ms: u32,
  bridge_enabled: bool,
  error_count: u32,
  last_activity_ms: u32,
}

impl SystemState {
  fn new() -> Self {
    Self {
      uptime_ms: 0,
      bridge_enabled: true,
      error_count: 0,
      last_activity_ms: 0,
    }
  }

  fn update_time(&mut self, delta_ms: u32) {
    self.uptime_ms = self.uptime_ms.wrapping_add(delta_ms);
  }

  fn record_activity(&mut self) {
    self.last_activity_ms = self.uptime_ms;
  }

  fn increment_error(&mut self) {
    self.error_count = self.error_count.wrapping_add(1);
  }
}

/// 演示模式
#[derive(Debug, Clone, Copy, PartialEq)]
enum DemoMode {
  BridgeTest,      // 桥接测试
  DeviceScan,      // 设备扫描
  PerformanceTest, // 性能测试
  ErrorRecovery,   // 错误恢复测试
}

impl DemoMode {
  fn next(self) -> Self {
    match self {
      Self::BridgeTest => Self::DeviceScan,
      Self::DeviceScan => Self::PerformanceTest,
      Self::PerformanceTest => Self::ErrorRecovery,
      Self::ErrorRecovery => Self::BridgeTest,
    }
  }
}

/// 按键状态
#[derive(Debug, Clone, Copy)]
struct ButtonState {
  pressed: bool,
  last_press_ms: u32,
  debounce_ms: u32,
}

impl ButtonState {
  fn new() -> Self {
    Self {
      pressed: false,
      last_press_ms: 0,
      debounce_ms: 50,
    }
  }

  fn update(&mut self, pin_state: bool, current_time_ms: u32) -> bool {
    let elapsed = current_time_ms.wrapping_sub(self.last_press_ms);

    if !self.pressed && !pin_state && elapsed > self.debounce_ms {
      self.pressed = true;
      self.last_press_ms = current_time_ms;
      true
    } else if self.pressed && pin_state {
      self.pressed = false;
      false
    } else {
      false
    }
  }
}

/// LED控制器
struct LedController {
  status_led: Pin<Output<PushPull>>,
  error_led: Pin<Output<PushPull>>,
  activity_led: Pin<Output<PushPull>>,
  blink_state: bool,
  last_blink_ms: u32,
  blink_interval_ms: u32,
}

impl LedController {
  fn new(
    status_led: Pin<Output<PushPull>>,
    error_led: Pin<Output<PushPull>>,
    activity_led: Pin<Output<PushPull>>,
  ) -> Self {
    Self {
      status_led,
      error_led,
      activity_led,
      blink_state: false,
      last_blink_ms: 0,
      blink_interval_ms: 500,
    }
  }

  fn update(
    &mut self,
    system_state: &SystemState,
    bridge_state: BridgeState,
    current_time_ms: u32,
  ) {
    // 状态LED闪烁
    let elapsed = current_time_ms.wrapping_sub(self.last_blink_ms);
    if elapsed >= self.blink_interval_ms {
      self.blink_state = !self.blink_state;
      self.last_blink_ms = current_time_ms;

      if self.blink_state {
        self.status_led.set_high();
      } else {
        self.status_led.set_low();
      }
    }

    // 错误LED
    if system_state.error_count > 0 || bridge_state == BridgeState::Error {
      self.error_led.set_high();
    } else {
      self.error_led.set_low();
    }

    // 活动LED
    let activity_elapsed = current_time_ms.wrapping_sub(system_state.last_activity_ms);
    if activity_elapsed < 100 {
      self.activity_led.set_high();
    } else {
      self.activity_led.set_low();
    }
  }

  fn set_blink_interval(&mut self, interval_ms: u32) {
    self.blink_interval_ms = interval_ms;
  }
}

/// 应用控制器
struct ApplicationController {
  bridge: ProtocolBridge,
  system_state: SystemState,
  demo_mode: DemoMode,
  button_state: ButtonState,
  led_controller: LedController,
  last_demo_update_ms: u32,
  demo_update_interval_ms: u32,
}

impl ApplicationController {
  fn new(bridge: ProtocolBridge, led_controller: LedController) -> Self {
    Self {
      bridge,
      system_state: SystemState::new(),
      demo_mode: DemoMode::BridgeTest,
      button_state: ButtonState::new(),
      led_controller,
      last_demo_update_ms: 0,
      demo_update_interval_ms: 2000,
    }
  }

  fn update(&mut self, button_pin: bool, current_time_ms: u32) {
    // 更新系统时间
    self.system_state.update_time(1);

    // 处理按键
    if self.button_state.update(button_pin, current_time_ms) {
      self.handle_button_press();
    }

    // 处理桥接请求
    let responses = self.bridge.process_requests(current_time_ms);
    if !responses.is_empty() {
      self.system_state.record_activity();
      defmt::info!("Processed {} bridge responses", responses.len());
    }

    // 更新演示模式
    let demo_elapsed = current_time_ms.wrapping_sub(self.last_demo_update_ms);
    if demo_elapsed >= self.demo_update_interval_ms {
      self.update_demo_mode(current_time_ms);
      self.last_demo_update_ms = current_time_ms;
    }

    // 更新LED
    self
      .led_controller
      .update(&self.system_state, self.bridge.get_state(), current_time_ms);

    // 清理过期响应
    self.bridge.cleanup_expired_responses(current_time_ms);
  }

  fn handle_button_press(&mut self) {
    self.demo_mode = self.demo_mode.next();
    self.system_state.record_activity();

    defmt::info!("Switched to demo mode: {:?}", self.demo_mode);

    // 根据模式调整LED闪烁间隔
    match self.demo_mode {
      DemoMode::BridgeTest => self.led_controller.set_blink_interval(500),
      DemoMode::DeviceScan => self.led_controller.set_blink_interval(250),
      DemoMode::PerformanceTest => self.led_controller.set_blink_interval(100),
      DemoMode::ErrorRecovery => self.led_controller.set_blink_interval(1000),
    }
  }

  fn update_demo_mode(&mut self, current_time_ms: u32) {
    match self.demo_mode {
      DemoMode::BridgeTest => {
        self.run_bridge_test(current_time_ms);
      }
      DemoMode::DeviceScan => {
        self.run_device_scan(current_time_ms);
      }
      DemoMode::PerformanceTest => {
        self.run_performance_test(current_time_ms);
      }
      DemoMode::ErrorRecovery => {
        self.run_error_recovery_test(current_time_ms);
      }
    }
  }

  fn run_bridge_test(&mut self, current_time_ms: u32) {
    // 创建测试消息
    let mut test_message = ProtocolMessage::new(1, OperationType::Read, 0x10);
    let _ = test_message.add_data(&[0x01, 0x02, 0x03, 0x04]);
    test_message.calculate_crc();

    // 添加桥接请求
    match self
      .bridge
      .add_request(BridgeOperation::I2cToSpi, test_message, current_time_ms)
    {
      Ok(request_id) => {
        defmt::debug!("Added bridge test request {}", request_id);
        self.system_state.record_activity();
      }
      Err(e) => {
        defmt::error!("Failed to add bridge request: {:?}", e);
        self.system_state.increment_error();
      }
    }
  }

  fn run_device_scan(&mut self, current_time_ms: u32) {
    // 这里应该扫描I2C和SPI设备
    defmt::info!("Running device scan...");

    let stats = self.bridge.get_stats();
    defmt::info!(
      "Bridge stats - Total: {}, Success: {}, Failed: {}",
      stats.total_requests,
      stats.successful_conversions,
      stats.failed_conversions
    );

    self.system_state.record_activity();
  }

  fn run_performance_test(&mut self, current_time_ms: u32) {
    // 创建多个测试消息进行性能测试
    for i in 0..3 {
      let mut test_message = ProtocolMessage::new(
        (i % 3) + 1,
        if i % 2 == 0 {
          OperationType::Read
        } else {
          OperationType::Write
        },
        0x20 + i as u16,
      );
      let _ = test_message.add_data(&[i as u8, (i + 1) as u8]);
      test_message.calculate_crc();

      let _ = self
        .bridge
        .add_request(BridgeOperation::I2cToSpi, test_message, current_time_ms);
    }

    defmt::debug!("Performance test: added 3 requests");
    self.system_state.record_activity();
  }

  fn run_error_recovery_test(&mut self, current_time_ms: u32) {
    // 测试错误恢复机制
    let stats = self.bridge.get_stats();

    if stats.failed_conversions > 0 || stats.timeout_errors > 0 {
      defmt::info!(
        "Error recovery - Errors: {}, Timeouts: {}",
        stats.failed_conversions,
        stats.timeout_errors
      );

      // 重置统计信息
      self.bridge.reset_stats();
      defmt::info!("Reset bridge statistics");
    }

    // 清空队列
    let (req_count, resp_count) = self.bridge.get_queue_status();
    if req_count > 5 || resp_count > 5 {
      self.bridge.clear_queues();
      defmt::info!("Cleared bridge queues");
    }

    self.system_state.record_activity();
  }

  fn get_stats(&self) -> &SystemState {
    &self.system_state
  }
}

#[entry]
fn main() -> ! {
  defmt::info!("Protocol Bridge Example Starting...");

  // 获取外设
  let dp = Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(25.MHz())
    .sysclk(84.MHz())
    .pclk1(42.MHz())
    .pclk2(84.MHz())
    .freeze();

  defmt::info!("System clock: {} Hz", clocks.sysclk().raw());

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // I2C引脚 (PB6: SCL, PB7: SDA)
  let scl = gpiob.pb6.into_alternate_open_drain();
  let sda = gpiob.pb7.into_alternate_open_drain();

  // SPI引脚 (PA5: SCK, PA6: MISO, PA7: MOSI)
  let sck = gpioa.pa5.into_alternate();
  let miso = gpioa.pa6.into_alternate();
  let mosi = gpioa.pa7.into_alternate();

  // SPI片选引脚
  let cs1 = gpioa.pa4.into_push_pull_output();
  let cs2 = gpioa.pa3.into_push_pull_output();
  let cs3 = gpioa.pa2.into_push_pull_output();

  // LED引脚
  let status_led = gpioc.pc13.into_push_pull_output();
  let error_led = gpioc.pc14.into_push_pull_output();
  let activity_led = gpioc.pc15.into_push_pull_output();

  // 按键引脚
  let button_pin = gpioa.pa0.into_pull_up_input();

  // 初始化I2C
  let i2c = I2c::new(
    dp.I2C1,
    (scl, sda),
    I2cMode::Standard {
      frequency: 100.kHz(),
    },
    &clocks,
  );

  // 初始化SPI
  let spi_mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
  };

  let spi = Spi::new(dp.SPI1, (sck, miso, mosi), spi_mode, 1.MHz(), &clocks);

  // 创建I2C从设备管理器
  let mut i2c_manager = I2cSlaveManager::new();

  // 添加I2C从设备
  let slave_config = SlaveConfig::default();
  let i2c_slave = I2cSlave::new(i2c, slave_config);
  if let Err(e) = i2c_manager.add_slave(i2c_slave) {
    defmt::error!("Failed to add I2C slave: {:?}", e);
  }

  // 创建SPI设备管理器
  let mut spi_manager = SpiDeviceManager::new();

  // 创建SPI主设备
  let mut spi_master = SpiMaster::new(spi);

  // 添加SPI设备
  let spi_configs = [
    SpiDeviceConfig {
      device_id: 1,
      cs_pin: 4,
      frequency: 1_000_000,
      mode: SpiMode::Mode0,
      bit_order: BitOrder::MsbFirst,
      timeout_ms: 100,
    },
    SpiDeviceConfig {
      device_id: 2,
      cs_pin: 3,
      frequency: 2_000_000,
      mode: SpiMode::Mode0,
      bit_order: BitOrder::MsbFirst,
      timeout_ms: 100,
    },
    SpiDeviceConfig {
      device_id: 3,
      cs_pin: 2,
      frequency: 500_000,
      mode: SpiMode::Mode1,
      bit_order: BitOrder::MsbFirst,
      timeout_ms: 100,
    },
  ];

  let cs_pins = [cs1, cs2, cs3];
  for (config, cs_pin) in spi_configs.iter().zip(cs_pins.into_iter()) {
    if let Err(e) = spi_master.add_device(*config, cs_pin) {
      defmt::error!("Failed to add SPI device {}: {:?}", config.device_id, e);
    }
  }

  if let Err(e) = spi_manager.add_master(spi_master) {
    defmt::error!("Failed to add SPI master: {:?}", e);
  }

  // 创建协议桥接器
  let bridge_config = BridgeConfig::default();
  let mut bridge = ProtocolBridge::new(i2c_manager, spi_manager, bridge_config);

  if let Err(e) = bridge.init() {
    defmt::error!("Failed to initialize bridge: {:?}", e);
  }

  // 创建LED控制器
  let led_controller = LedController::new(status_led, error_led, activity_led);

  // 创建应用控制器
  let mut app_controller = ApplicationController::new(bridge, led_controller);

  // 配置定时器
  let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
  timer.start(1000.Hz()).unwrap(); // 1ms中断
  timer.listen(Event::Update);

  defmt::info!("Protocol bridge initialized successfully");
  defmt::info!("Press button to cycle through demo modes");

  // 启动序列
  for i in 0..3 {
    cortex_m::asm::delay(clocks.sysclk().raw() / 10); // 100ms延迟
    defmt::info!("Starting in {}...", 3 - i);
  }

  let mut system_time_ms = 0u32;
  let mut last_stats_ms = 0u32;

  // 主循环
  loop {
    // 检查定时器中断
    if timer.wait().is_ok() {
      system_time_ms = system_time_ms.wrapping_add(1);
    }

    // 读取按键状态
    let button_pressed = button_pin.is_low();

    // 更新应用控制器
    app_controller.update(button_pressed, system_time_ms);

    // 定期输出统计信息
    let stats_elapsed = system_time_ms.wrapping_sub(last_stats_ms);
    if stats_elapsed >= 10000 {
      // 每10秒
      let stats = app_controller.get_stats();
      let bridge_stats = app_controller.bridge.get_stats();

      defmt::info!("=== System Stats ===");
      defmt::info!(
        "Uptime: {}s, Errors: {}",
        stats.uptime_ms / 1000,
        stats.error_count
      );
      defmt::info!(
        "Bridge - Requests: {}, Success: {}, Failed: {}",
        bridge_stats.total_requests,
        bridge_stats.successful_conversions,
        bridge_stats.failed_conversions
      );
      defmt::info!("Queue size: {:?}", app_controller.bridge.get_queue_status());

      last_stats_ms = system_time_ms;
    }

    // 短暂延迟
    cortex_m::asm::delay(clocks.sysclk().raw() / 10000); // 0.1ms
  }
}
