#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
  adc::{config::AdcConfig, Adc},
  gpio::{gpioa::*, gpiob::*, gpioc::*, Input, Output, PullUp, PushPull},
  i2c::I2c,
  prelude::*,
  pwr::{PowerMode, Pwr},
  rcc::{Clocks, RccExt},
  rtc::{Rtc, RtcConfig},
  stm32,
  timer::{Event, Timer},
};

use heapless::{
  pool::{Node, Pool},
  Vec,
};
use micromath::F32Ext;

/// 传感器数据结构
#[derive(Debug, Clone, Copy)]
pub struct SensorReading {
  pub timestamp: u32,
  pub temperature: f32,     // 温度 (°C)
  pub humidity: f32,        // 湿度 (%)
  pub pressure: f32,        // 压力 (hPa)
  pub battery_voltage: u16, // 电池电压 (mV)
}

/// 低功耗传感器节点
pub struct LowPowerSensorNode {
  // 硬件外设
  i2c: I2c<
    stm32::I2C1,
    (
      PB8<stm32f4xx_hal::gpio::Alternate<4>>,
      PB9<stm32f4xx_hal::gpio::Alternate<4>>,
    ),
  >,
  adc: Adc<stm32::ADC1>,
  battery_pin: PA1<stm32f4xx_hal::gpio::Analog>,
  status_led: PC13<Output<PushPull>>,
  wake_button: PA0<Input<PullUp>>,

  // 电源管理
  pwr: Pwr,
  rtc: Rtc,

  // 传感器状态
  sensor_config: SensorConfig,
  power_config: PowerConfig,

  // 数据缓冲
  data_buffer: Vec<SensorReading, 32>,

  // 系统状态
  system_state: SystemState,
  last_reading_time: u32,
  wake_count: u32,
}

/// 传感器配置
#[derive(Clone, Copy)]
pub struct SensorConfig {
  pub sampling_interval: u32,     // 采样间隔 (秒)
  pub temperature_threshold: f32, // 温度阈值
  pub humidity_threshold: f32,    // 湿度阈值
  pub pressure_threshold: f32,    // 压力阈值
  pub enable_adaptive: bool,      // 启用自适应采样
}

/// 电源配置
#[derive(Clone, Copy)]
pub struct PowerConfig {
  pub low_battery_threshold: u16,      // 低电量阈值 (mV)
  pub critical_battery_threshold: u16, // 临界电量阈值 (mV)
  pub sleep_mode: SleepMode,           // 睡眠模式
  pub wake_sources: WakeSources,       // 唤醒源
}

/// 睡眠模式
#[derive(Clone, Copy, PartialEq)]
pub enum SleepMode {
  Sleep,   // 睡眠模式
  Stop,    // 停止模式
  Standby, // 待机模式
}

/// 唤醒源
#[derive(Clone, Copy)]
pub struct WakeSources {
  pub rtc_alarm: bool,       // RTC闹钟
  pub external_pin: bool,    // 外部引脚
  pub threshold_event: bool, // 阈值事件
}

/// 系统状态
#[derive(Clone, Copy, PartialEq)]
pub enum SystemState {
  Normal,      // 正常运行
  PowerSaving, // 节能模式
  LowBattery,  // 低电量
  Critical,    // 临界状态
  Shutdown,    // 关机
}

impl LowPowerSensorNode {
  pub fn new(
    i2c: I2c<
      stm32::I2C1,
      (
        PB8<stm32f4xx_hal::gpio::Alternate<4>>,
        PB9<stm32f4xx_hal::gpio::Alternate<4>>,
      ),
    >,
    adc: Adc<stm32::ADC1>,
    battery_pin: PA1<stm32f4xx_hal::gpio::Analog>,
    status_led: PC13<Output<PushPull>>,
    wake_button: PA0<Input<PullUp>>,
    pwr: Pwr,
    rtc: Rtc,
  ) -> Self {
    Self {
      i2c,
      adc,
      battery_pin,
      status_led,
      wake_button,
      pwr,
      rtc,
      sensor_config: SensorConfig {
        sampling_interval: 60,      // 60秒
        temperature_threshold: 2.0, // 2°C变化
        humidity_threshold: 5.0,    // 5%变化
        pressure_threshold: 1.0,    // 1hPa变化
        enable_adaptive: true,
      },
      power_config: PowerConfig {
        low_battery_threshold: 3400,      // 3.4V
        critical_battery_threshold: 3200, // 3.2V
        sleep_mode: SleepMode::Stop,
        wake_sources: WakeSources {
          rtc_alarm: true,
          external_pin: true,
          threshold_event: false,
        },
      },
      data_buffer: Vec::new(),
      system_state: SystemState::Normal,
      last_reading_time: 0,
      wake_count: 0,
    }
  }

  /// 初始化传感器
  pub fn initialize(&mut self) -> Result<(), &'static str> {
    // 初始化状态LED
    self.status_led.set_high();

    // 配置RTC闹钟
    self.setup_rtc_alarm()?;

    // 初始化传感器
    self.initialize_sensors()?;

    // 设置初始电源模式
    self.configure_power_mode();

    // 状态指示
    self.blink_status_led(3, 200);

    Ok(())
  }

  /// 主运行循环
  pub fn run(&mut self) -> ! {
    loop {
      // 检查唤醒原因
      let wake_reason = self.check_wake_reason();
      self.wake_count += 1;

      // 读取电池电压
      let battery_voltage = self.read_battery_voltage();

      // 更新系统状态
      self.update_system_state(battery_voltage);

      // 根据系统状态执行相应操作
      match self.system_state {
        SystemState::Normal => {
          self.normal_operation(wake_reason);
        }
        SystemState::PowerSaving => {
          self.power_saving_operation(wake_reason);
        }
        SystemState::LowBattery => {
          self.low_battery_operation(wake_reason);
        }
        SystemState::Critical => {
          self.critical_operation(wake_reason);
        }
        SystemState::Shutdown => {
          self.shutdown_operation();
        }
      }

      // 进入睡眠模式
      self.enter_sleep_mode();
    }
  }

  /// 正常运行模式
  fn normal_operation(&mut self, wake_reason: WakeReason) {
    match wake_reason {
      WakeReason::RtcAlarm => {
        // 定时采样
        if let Ok(reading) = self.read_all_sensors() {
          self.process_sensor_data(reading);

          // 自适应采样调整
          if self.sensor_config.enable_adaptive {
            self.adjust_sampling_interval(&reading);
          }
        }

        // 设置下次闹钟
        self.set_next_alarm();
      }
      WakeReason::ExternalPin => {
        // 用户按键或外部事件
        self.handle_external_event();
      }
      WakeReason::ThresholdEvent => {
        // 阈值触发事件
        self.handle_threshold_event();
      }
      _ => {}
    }

    // 状态指示
    self.blink_status_led(1, 100);
  }

  /// 节能运行模式
  fn power_saving_operation(&mut self, wake_reason: WakeReason) {
    match wake_reason {
      WakeReason::RtcAlarm => {
        // 降低采样频率
        if let Ok(reading) = self.read_essential_sensors() {
          self.process_sensor_data(reading);
        }

        // 延长采样间隔
        let extended_interval = self.sensor_config.sampling_interval * 2;
        self.set_alarm_interval(extended_interval);
      }
      WakeReason::ExternalPin => {
        self.handle_external_event();
      }
      _ => {}
    }

    // 简化状态指示
    self.quick_blink_status_led();
  }

  /// 低电量运行模式
  fn low_battery_operation(&mut self, wake_reason: WakeReason) {
    match wake_reason {
      WakeReason::RtcAlarm => {
        // 仅读取关键传感器
        if let Ok(reading) = self.read_critical_sensors() {
          // 仅保存关键数据
          if self.is_critical_data(&reading) {
            self.save_critical_data(reading);
          }
        }

        // 大幅延长采样间隔
        let emergency_interval = self.sensor_config.sampling_interval * 4;
        self.set_alarm_interval(emergency_interval);
      }
      WakeReason::ExternalPin => {
        // 简化处理
        self.handle_emergency_event();
      }
      _ => {}
    }

    // 警告指示
    self.warning_blink_status_led();
  }

  /// 临界运行模式
  fn critical_operation(&mut self, wake_reason: WakeReason) {
    match wake_reason {
      WakeReason::RtcAlarm => {
        // 仅保存系统状态
        self.save_system_state();

        // 极长采样间隔
        let critical_interval = self.sensor_config.sampling_interval * 8;
        self.set_alarm_interval(critical_interval);
      }
      WakeReason::ExternalPin => {
        // 检查是否需要恢复
        if self.check_recovery_condition() {
          self.system_state = SystemState::LowBattery;
        }
      }
      _ => {}
    }

    // 错误指示
    self.error_blink_status_led();
  }

  /// 关机操作
  fn shutdown_operation(&mut self) -> ! {
    // 保存关键数据
    self.save_emergency_data();

    // 关闭所有外设
    self.shutdown_peripherals();

    // 最后状态指示
    self.final_status_indication();

    // 进入深度睡眠
    self.enter_deep_sleep();
  }

  /// 读取所有传感器
  fn read_all_sensors(&mut self) -> Result<SensorReading, &'static str> {
    let timestamp = self.get_timestamp();

    // 读取温湿度传感器 (SHT30)
    let (temperature, humidity) = self.read_sht30()?;

    // 读取压力传感器 (BMP280)
    let pressure = self.read_bmp280()?;

    // 读取电池电压
    let battery_voltage = self.read_battery_voltage();

    Ok(SensorReading {
      timestamp,
      temperature,
      humidity,
      pressure,
      battery_voltage,
    })
  }

  /// 读取基本传感器
  fn read_essential_sensors(&mut self) -> Result<SensorReading, &'static str> {
    let timestamp = self.get_timestamp();

    // 仅读取温度和电池电压
    let temperature = self.read_temperature_only()?;
    let battery_voltage = self.read_battery_voltage();

    Ok(SensorReading {
      timestamp,
      temperature,
      humidity: 0.0,
      pressure: 0.0,
      battery_voltage,
    })
  }

  /// 读取关键传感器
  fn read_critical_sensors(&mut self) -> Result<SensorReading, &'static str> {
    let timestamp = self.get_timestamp();
    let battery_voltage = self.read_battery_voltage();

    Ok(SensorReading {
      timestamp,
      temperature: 0.0,
      humidity: 0.0,
      pressure: 0.0,
      battery_voltage,
    })
  }

  /// 读取SHT30温湿度传感器
  fn read_sht30(&mut self) -> Result<(f32, f32), &'static str> {
    // SHT30 I2C地址
    const SHT30_ADDR: u8 = 0x44;

    // 发送测量命令
    let cmd = [0x2C, 0x06]; // 高精度测量
    self
      .i2c
      .write(SHT30_ADDR, &cmd)
      .map_err(|_| "SHT30 write failed")?;

    // 等待测量完成
    cortex_m::asm::delay(15000); // 15ms

    // 读取数据
    let mut data = [0u8; 6];
    self
      .i2c
      .read(SHT30_ADDR, &mut data)
      .map_err(|_| "SHT30 read failed")?;

    // 解析温度数据
    let temp_raw = ((data[0] as u16) << 8) | (data[1] as u16);
    let temperature = -45.0 + 175.0 * (temp_raw as f32) / 65535.0;

    // 解析湿度数据
    let hum_raw = ((data[3] as u16) << 8) | (data[4] as u16);
    let humidity = 100.0 * (hum_raw as f32) / 65535.0;

    Ok((temperature, humidity))
  }

  /// 读取BMP280压力传感器
  fn read_bmp280(&mut self) -> Result<f32, &'static str> {
    // BMP280 I2C地址
    const BMP280_ADDR: u8 = 0x76;

    // 读取压力数据寄存器
    let reg = [0xF7];
    self
      .i2c
      .write(BMP280_ADDR, &reg)
      .map_err(|_| "BMP280 write failed")?;

    let mut data = [0u8; 3];
    self
      .i2c
      .read(BMP280_ADDR, &mut data)
      .map_err(|_| "BMP280 read failed")?;

    // 解析压力数据 (简化计算)
    let pressure_raw = ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);
    let pressure = pressure_raw as f32 / 256.0; // 简化转换

    Ok(pressure)
  }

  /// 仅读取温度
  fn read_temperature_only(&mut self) -> Result<f32, &'static str> {
    let (temperature, _) = self.read_sht30()?;
    Ok(temperature)
  }

  /// 读取电池电压
  fn read_battery_voltage(&mut self) -> u16 {
    let sample: u16 = self.adc.read(&mut self.battery_pin).unwrap_or(0);
    // 转换为毫伏 (假设分压比为1:1)
    (sample * 3300) / 4095
  }

  /// 处理传感器数据
  fn process_sensor_data(&mut self, reading: SensorReading) {
    // 添加到缓冲区
    if self.data_buffer.len() >= 32 {
      self.data_buffer.remove(0); // 移除最旧的数据
    }
    self.data_buffer.push(reading).ok();

    // 检查阈值
    if self.check_thresholds(&reading) {
      self.handle_threshold_event();
    }

    self.last_reading_time = reading.timestamp;
  }

  /// 检查阈值
  fn check_thresholds(&self, reading: &SensorReading) -> bool {
    if let Some(last_reading) = self.data_buffer.last() {
      let temp_diff = (reading.temperature - last_reading.temperature).abs();
      let hum_diff = (reading.humidity - last_reading.humidity).abs();
      let press_diff = (reading.pressure - last_reading.pressure).abs();

      temp_diff > self.sensor_config.temperature_threshold
        || hum_diff > self.sensor_config.humidity_threshold
        || press_diff > self.sensor_config.pressure_threshold
    } else {
      false
    }
  }

  /// 自适应采样间隔调整
  fn adjust_sampling_interval(&mut self, reading: &SensorReading) {
    if self.data_buffer.len() < 3 {
      return;
    }

    // 计算变化率
    let change_rate = self.calculate_change_rate();

    // 根据变化率调整采样间隔
    let base_interval = 60; // 基础间隔60秒
    let new_interval = if change_rate > 0.5 {
      base_interval / 2 // 变化快，增加采样频率
    } else if change_rate < 0.1 {
      base_interval * 2 // 变化慢，降低采样频率
    } else {
      base_interval // 保持正常频率
    };

    self.sensor_config.sampling_interval = new_interval.max(10).min(300); // 限制在10-300秒
  }

  /// 计算变化率
  fn calculate_change_rate(&self) -> f32 {
    if self.data_buffer.len() < 3 {
      return 0.0;
    }

    let len = self.data_buffer.len();
    let recent = &self.data_buffer[len - 1];
    let previous = &self.data_buffer[len - 3];

    let temp_change = (recent.temperature - previous.temperature).abs();
    let hum_change = (recent.humidity - previous.humidity).abs() / 100.0;
    let press_change = (recent.pressure - previous.pressure).abs() / 1000.0;

    (temp_change + hum_change + press_change) / 3.0
  }

  /// 更新系统状态
  fn update_system_state(&mut self, battery_voltage: u16) {
    self.system_state = if battery_voltage < self.power_config.critical_battery_threshold {
      SystemState::Critical
    } else if battery_voltage < self.power_config.low_battery_threshold {
      SystemState::LowBattery
    } else if self.wake_count > 100 {
      SystemState::PowerSaving
    } else {
      SystemState::Normal
    };
  }

  /// 进入睡眠模式
  fn enter_sleep_mode(&mut self) {
    // 关闭状态LED
    self.status_led.set_low();

    // 根据配置进入相应睡眠模式
    match self.power_config.sleep_mode {
      SleepMode::Sleep => {
        cortex_m::asm::wfi(); // 等待中断
      }
      SleepMode::Stop => {
        self.pwr.stop_mode(&mut self.rtc);
      }
      SleepMode::Standby => {
        self.pwr.standby_mode();
      }
    }
  }

  /// 状态指示方法
  fn blink_status_led(&mut self, count: u8, delay_ms: u32) {
    for _ in 0..count {
      self.status_led.set_high();
      cortex_m::asm::delay(delay_ms * 1000);
      self.status_led.set_low();
      cortex_m::asm::delay(delay_ms * 1000);
    }
  }

  fn quick_blink_status_led(&mut self) {
    self.status_led.set_high();
    cortex_m::asm::delay(50000);
    self.status_led.set_low();
  }

  fn warning_blink_status_led(&mut self) {
    for _ in 0..2 {
      self.status_led.set_high();
      cortex_m::asm::delay(100000);
      self.status_led.set_low();
      cortex_m::asm::delay(100000);
    }
  }

  fn error_blink_status_led(&mut self) {
    for _ in 0..5 {
      self.status_led.set_high();
      cortex_m::asm::delay(50000);
      self.status_led.set_low();
      cortex_m::asm::delay(50000);
    }
  }

  fn final_status_indication(&mut self) {
    for _ in 0..10 {
      self.status_led.set_high();
      cortex_m::asm::delay(100000);
      self.status_led.set_low();
      cortex_m::asm::delay(100000);
    }
  }

  // 其他辅助方法的简化实现
  fn setup_rtc_alarm(&mut self) -> Result<(), &'static str> {
    Ok(())
  }
  fn initialize_sensors(&mut self) -> Result<(), &'static str> {
    Ok(())
  }
  fn configure_power_mode(&mut self) {}
  fn check_wake_reason(&self) -> WakeReason {
    WakeReason::RtcAlarm
  }
  fn set_next_alarm(&mut self) {}
  fn set_alarm_interval(&mut self, _interval: u32) {}
  fn handle_external_event(&mut self) {}
  fn handle_threshold_event(&mut self) {}
  fn handle_emergency_event(&mut self) {}
  fn save_critical_data(&mut self, _reading: SensorReading) {}
  fn save_system_state(&mut self) {}
  fn save_emergency_data(&mut self) {}
  fn shutdown_peripherals(&mut self) {}
  fn enter_deep_sleep(&mut self) -> ! {
    loop {
      cortex_m::asm::wfi();
    }
  }
  fn get_timestamp(&self) -> u32 {
    self.wake_count
  }
  fn is_critical_data(&self, _reading: &SensorReading) -> bool {
    true
  }
  fn check_recovery_condition(&self) -> bool {
    false
  }
}

/// 唤醒原因
#[derive(Clone, Copy, PartialEq)]
pub enum WakeReason {
  RtcAlarm,
  ExternalPin,
  ThresholdEvent,
  Unknown,
}

#[entry]
fn main() -> ! {
  // 获取外设
  let cp = cortex_m::Peripherals::take().unwrap();
  let dp = stm32::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(8.mhz())
    .sysclk(84.mhz())
    .pclk1(42.mhz())
    .pclk2(84.mhz())
    .freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // I2C配置
  let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
  let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
  let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);

  // ADC配置
  let adc_config = AdcConfig::default();
  let adc = Adc::adc1(dp.ADC1, true, adc_config);
  let battery_pin = gpioa.pa1.into_analog();

  // GPIO配置
  let status_led = gpioc.pc13.into_push_pull_output();
  let wake_button = gpioa.pa0.into_pull_up_input();

  // 电源管理
  let pwr = Pwr::new(dp.PWR);

  // RTC配置
  let rtc_config = RtcConfig::default();
  let rtc = Rtc::rtc(dp.RTC, &mut dp.PWR, &mut rcc.bkp, rtc_config);

  // 创建传感器节点
  let mut sensor_node =
    LowPowerSensorNode::new(i2c, adc, battery_pin, status_led, wake_button, pwr, rtc);

  // 初始化
  sensor_node.initialize().unwrap();

  // 运行主循环
  sensor_node.run()
}
