#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use heapless::Vec;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{Alternate, Input, Output, Pin, PullUp, PushPull},
  i2c::I2c,
  interrupt, pac,
  prelude::*,
  timer::{Event, Timer},
};

use io_expander::{pin_groups, pin_mask, Pcf8574Manager, Pin as Pcf8574Pin, PinState};

/// 系统状态
static mut SYSTEM_STATE: Option<SystemState> = None;

/// 系统状态结构
struct SystemState {
  pcf_manager: Pcf8574Manager<I2c<pac::I2C1>>,
  timer: Timer<pac::TIM2>,
  interrupt_pin: Pin<'A', 0, Input<PullUp>>,
  status_led: Pin<'C', 13, Output<PushPull>>,
  tick_counter: u32,
  led_pattern: LedPattern,
  button_states: [ButtonState; 4],
}

/// LED模式
#[derive(Debug, Clone, Copy)]
enum LedPattern {
  Off,
  AllOn,
  Blink,
  Chase,
  Random,
  Binary,
}

/// 按键状态
#[derive(Debug, Clone, Copy)]
struct ButtonState {
  current: bool,
  last: bool,
  debounce_counter: u8,
  press_count: u32,
}

impl ButtonState {
  fn new() -> Self {
    Self {
      current: false,
      last: false,
      debounce_counter: 0,
      press_count: 0,
    }
  }

  fn update(&mut self, raw_state: bool) -> bool {
    const DEBOUNCE_THRESHOLD: u8 = 3;

    if raw_state != self.current {
      self.debounce_counter += 1;
      if self.debounce_counter >= DEBOUNCE_THRESHOLD {
        self.current = raw_state;
        self.debounce_counter = 0;

        // 检测按下事件（从高到低）
        if self.last && !self.current {
          self.press_count += 1;
          self.last = self.current;
          return true;
        }
        self.last = self.current;
      }
    } else {
      self.debounce_counter = 0;
    }

    false
  }
}

/// PCF8574设备配置
struct DeviceConfig {
  address: u8,
  input_pins: u8,
  output_pins: u8,
  description: &'static str,
}

const DEVICE_CONFIGS: &[DeviceConfig] = &[
  DeviceConfig {
    address: 0x20,
    input_pins: pin_groups::BUTTONS, // P4-P7 用于按键
    output_pins: pin_groups::LEDS,   // P0-P3 用于LED
    description: "主控制器",
  },
  DeviceConfig {
    address: 0x21,
    input_pins: 0x00,
    output_pins: 0xFF, // 全部用于LED输出
    description: "LED扩展板",
  },
];

#[entry]
fn main() -> ! {
  defmt::info!("PCF8574 I2C IO扩展器示例启动");

  // 初始化外设
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc
    .cfgr
    .use_hse(8.MHz())
    .sysclk(84.MHz())
    .pclk1(42.MHz())
    .pclk2(84.MHz())
    .freeze();

  defmt::info!(
    "系统时钟配置完成: SYSCLK={}MHz",
    clocks.sysclk().raw() / 1_000_000
  );

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // I2C引脚配置
  let scl = gpiob.pb6.into_alternate_open_drain();
  let sda = gpiob.pb7.into_alternate_open_drain();

  // 中断引脚配置
  let interrupt_pin = gpioa.pa0.into_pull_up_input();

  // 状态LED配置
  let mut status_led = gpioc.pc13.into_push_pull_output();
  status_led.set_high();

  // I2C配置
  let i2c = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

  defmt::info!("I2C接口初始化完成");

  // 创建PCF8574管理器
  let mut pcf_manager = Pcf8574Manager::new(i2c);

  // 扫描设备
  defmt::info!("扫描I2C总线上的PCF8574设备...");
  let found_devices = pcf_manager.pcf_mut().scan_devices();
  defmt::info!(
    "发现 {} 个设备: {:?}",
    found_devices.len(),
    found_devices.as_slice()
  );

  // 初始化设备
  for config in DEVICE_CONFIGS {
    if found_devices.contains(&config.address) {
      match pcf_manager.init_device(config.address, config.input_pins, config.output_pins) {
        Ok(_) => {
          defmt::info!(
            "设备 0x{:02X} ({}) 初始化成功",
            config.address,
            config.description
          );
        }
        Err(e) => {
          defmt::error!("设备 0x{:02X} 初始化失败: {:?}", config.address, e);
        }
      }
    } else {
      defmt::warn!(
        "设备 0x{:02X} ({}) 未找到",
        config.address,
        config.description
      );
    }
  }

  // 配置定时器
  let mut timer = Timer::new(dp.TIM2, &clocks);
  timer.start(10.Hz());
  timer.listen(Event::Update);

  defmt::info!("定时器配置完成");

  // 创建系统状态
  let system_state = SystemState {
    pcf_manager,
    timer,
    interrupt_pin,
    status_led,
    tick_counter: 0,
    led_pattern: LedPattern::Chase,
    button_states: [ButtonState::new(); 4],
  };

  unsafe {
    SYSTEM_STATE = Some(system_state);
  }

  // 启用中断
  unsafe {
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
  }

  defmt::info!("系统初始化完成，开始主循环");

  // 执行启动序列
  startup_sequence();

  // 主循环
  loop {
    // 检查中断引脚
    check_interrupt_pin();

    // 处理按键
    process_buttons();

    // 更新显示
    update_display();

    // 短暂延时
    cortex_m::asm::delay(100_000);
  }
}

/// 启动序列
fn startup_sequence() {
  defmt::info!("执行启动序列");

  unsafe {
    if let Some(ref mut state) = SYSTEM_STATE {
      // 所有LED闪烁3次
      for _ in 0..3 {
        // 点亮所有LED
        for config in DEVICE_CONFIGS {
          if let Ok(_) = state.pcf_manager.pcf_mut().set_pins(
            config.address,
            config.output_pins,
            PinState::Low, // PCF8574是低电平有效
          ) {
            defmt::debug!("设备 0x{:02X} LED点亮", config.address);
          }
        }

        cortex_m::asm::delay(8_400_000); // 约100ms

        // 熄灭所有LED
        for config in DEVICE_CONFIGS {
          if let Ok(_) =
            state
              .pcf_manager
              .pcf_mut()
              .set_pins(config.address, config.output_pins, PinState::High)
          {
            defmt::debug!("设备 0x{:02X} LED熄灭", config.address);
          }
        }

        cortex_m::asm::delay(8_400_000); // 约100ms
      }

      defmt::info!("启动序列完成");
    }
  }
}

/// 检查中断引脚
fn check_interrupt_pin() {
  unsafe {
    if let Some(ref mut state) = SYSTEM_STATE {
      if state.interrupt_pin.is_low() {
        defmt::debug!("检测到中断信号");

        // 检查所有设备的中断状态
        for config in DEVICE_CONFIGS {
          match state.pcf_manager.check_interrupts(config.address) {
            Ok(Some(changed_pins)) => {
              defmt::info!(
                "设备 0x{:02X} 引脚变化: 0b{:08b}",
                config.address,
                changed_pins
              );
            }
            Ok(None) => {}
            Err(e) => {
              defmt::error!("检查设备 0x{:02X} 中断失败: {:?}", config.address, e);
            }
          }
        }
      }
    }
  }
}

/// 处理按键
fn process_buttons() {
  unsafe {
    if let Some(ref mut state) = SYSTEM_STATE {
      // 读取主控制器的按键状态
      if let Ok(button_data) = state
        .pcf_manager
        .pcf_mut()
        .read_pins(0x20, pin_groups::BUTTONS)
      {
        // 处理4个按键
        for i in 0..4 {
          let pin_mask = 1 << (i + 4); // P4-P7
          let raw_state = (button_data & pin_mask) == 0; // 按键按下时为低电平

          if state.button_states[i].update(raw_state) {
            defmt::info!(
              "按键 {} 被按下 (总计: {}次)",
              i,
              state.button_states[i].press_count
            );

            // 根据按键切换LED模式
            match i {
              0 => state.led_pattern = LedPattern::Off,
              1 => state.led_pattern = LedPattern::AllOn,
              2 => state.led_pattern = LedPattern::Blink,
              3 => {
                state.led_pattern = match state.led_pattern {
                  LedPattern::Chase => LedPattern::Random,
                  LedPattern::Random => LedPattern::Binary,
                  _ => LedPattern::Chase,
                };
              }
              _ => {}
            }

            defmt::info!("LED模式切换为: {:?}", state.led_pattern);
          }
        }
      }
    }
  }
}

/// 更新显示
fn update_display() {
  unsafe {
    if let Some(ref mut state) = SYSTEM_STATE {
      match state.led_pattern {
        LedPattern::Off => {
          // 关闭所有LED
          for config in DEVICE_CONFIGS {
            let _ = state.pcf_manager.pcf_mut().set_pins(
              config.address,
              config.output_pins,
              PinState::High,
            );
          }
        }
        LedPattern::AllOn => {
          // 点亮所有LED
          for config in DEVICE_CONFIGS {
            let _ = state.pcf_manager.pcf_mut().set_pins(
              config.address,
              config.output_pins,
              PinState::Low,
            );
          }
        }
        LedPattern::Blink => {
          // 闪烁模式在定时器中断中处理
        }
        LedPattern::Chase => {
          // 追逐模式在定时器中断中处理
        }
        LedPattern::Random => {
          // 随机模式在定时器中断中处理
        }
        LedPattern::Binary => {
          // 二进制计数模式在定时器中断中处理
        }
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

      // 增加计数器
      state.tick_counter = state.tick_counter.wrapping_add(1);

      // 切换状态LED
      if state.tick_counter % 50 == 0 {
        // 每5秒切换一次
        state.status_led.toggle();
      }

      // 根据LED模式更新显示
      match state.led_pattern {
        LedPattern::Blink => {
          let blink_state = if (state.tick_counter % 10) < 5 {
            PinState::Low
          } else {
            PinState::High
          };

          for config in DEVICE_CONFIGS {
            let _ =
              state
                .pcf_manager
                .pcf_mut()
                .set_pins(config.address, config.output_pins, blink_state);
          }
        }
        LedPattern::Chase => {
          // 追逐效果
          let position = (state.tick_counter / 5) % 8;
          let led_mask = 1 << position;

          // 主控制器
          if let Ok(_) = state
            .pcf_manager
            .pcf_mut()
            .write_device(0x20, 0xF0 | (!led_mask & 0x0F))
          {}

          // LED扩展板
          if let Ok(_) = state.pcf_manager.pcf_mut().write_device(0x21, !led_mask) {}
        }
        LedPattern::Random => {
          // 伪随机模式
          if state.tick_counter % 5 == 0 {
            let random_pattern = (state.tick_counter * 17 + 23) as u8;

            for config in DEVICE_CONFIGS {
              let pattern = if config.output_pins == 0xFF {
                !random_pattern
              } else {
                0xF0 | (!(random_pattern & 0x0F))
              };

              let _ = state
                .pcf_manager
                .pcf_mut()
                .write_device(config.address, pattern);
            }
          }
        }
        LedPattern::Binary => {
          // 二进制计数
          let count = (state.tick_counter / 10) as u8;

          // 主控制器显示低4位
          if let Ok(_) = state
            .pcf_manager
            .pcf_mut()
            .write_device(0x20, 0xF0 | (!(count & 0x0F)))
          {}

          // LED扩展板显示高4位
          if let Ok(_) = state
            .pcf_manager
            .pcf_mut()
            .write_device(0x21, !(count >> 4))
          {}
        }
        _ => {}
      }
    }
  }
}
