#![no_std]
#![no_main]

use cortex_m_rt::entry;
use input_capture::{RotaryEncoder, RotationDirection};
use micromath::F32Ext;
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::gpioc::PC13,
  gpio::{
    gpioa::{PA0, PA1},
    Input, PullUp,
  },
  gpio::{
    gpiob::{PB0, PB1, PB2, PB3, PB4, PB5},
    Input as GpioInput, Output, PushPull,
  },
  prelude::*,
  stm32,
  timer::Timer,
};

type StatusLed = PB0<Output<PushPull>>;
type DirectionLed = PB1<Output<PushPull>>;
type SpeedLed1 = PB2<Output<PushPull>>;
type SpeedLed2 = PB3<Output<PushPull>>;
type SpeedLed3 = PB4<Output<PushPull>>;
type PositionLed = PB5<Output<PushPull>>;

type ResetButton = PC13<GpioInput<PullUp>>;
type EncoderA = PA0<Input<PullUp>>;
type EncoderB = PA1<Input<PullUp>>;

#[entry]
fn main() -> ! {
  // 获取设备外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  let mut status_led = gpiob.pb0.into_push_pull_output();
  let mut direction_led = gpiob.pb1.into_push_pull_output();
  let mut speed_led1 = gpiob.pb2.into_push_pull_output();
  let mut speed_led2 = gpiob.pb3.into_push_pull_output();
  let mut speed_led3 = gpiob.pb4.into_push_pull_output();
  let mut position_led = gpiob.pb5.into_push_pull_output();

  let reset_button = gpioc.pc13.into_pull_up_input();
  let encoder_a = gpioa.pa0.into_pull_up_input();
  let encoder_b = gpioa.pa1.into_pull_up_input();

  // 配置延时定时器
  let mut delay = Timer::tim2(dp.TIM2, &clocks).delay_ms();

  // 系统启动指示
  status_led.set_high();
  delay.delay_ms(1000u32);

  // 创建编码器读取器
  let mut encoder = RotaryEncoder::new(360, 84_000_000); // 360步/转
  let mut button_debouncer = ButtonDebouncer::new();
  let mut display_mode = DisplayMode::Position;
  let mut demo_counter = 0u32;
  let mut last_timestamp = 0u32;

  // 编码器状态跟踪
  let mut last_a_state = encoder_a.is_high();
  let mut last_b_state = encoder_b.is_high();

  loop {
    // 读取编码器状态
    let a_state = encoder_a.is_high();
    let b_state = encoder_b.is_high();

    // 检测状态变化
    if a_state != last_a_state || b_state != last_b_state {
      encoder.update(a_state, b_state, last_timestamp);
      last_a_state = a_state;
      last_b_state = b_state;
    }

    // 按钮防抖处理
    let button_pressed = button_debouncer.update(reset_button.is_low());

    if button_pressed {
      // 切换显示模式或重置位置
      match display_mode {
        DisplayMode::Position => {
          display_mode = DisplayMode::Velocity;
        }
        DisplayMode::Velocity => {
          display_mode = DisplayMode::Direction;
        }
        DisplayMode::Direction => {
          display_mode = DisplayMode::Angle;
        }
        DisplayMode::Angle => {
          encoder.reset_position();
          display_mode = DisplayMode::Position;
        }
      }

      // 模式切换指示
      for _ in 0..3 {
        status_led.set_low();
        delay.delay_ms(100u32);
        status_led.set_high();
        delay.delay_ms(100u32);
      }
    }

    // 显示编码器状态
    display_encoder_status(
      &encoder,
      display_mode,
      &mut direction_led,
      &mut speed_led1,
      &mut speed_led2,
      &mut speed_led3,
      &mut position_led,
      demo_counter,
    );

    demo_counter += 1;
    last_timestamp += 10; // 模拟时间戳增加

    // 状态LED心跳
    if demo_counter % 100 == 0 {
      status_led.toggle();
    }

    delay.delay_ms(10u32);
  }
}

/// 显示编码器状态
fn display_encoder_status(
  encoder: &RotaryEncoder,
  mode: DisplayMode,
  direction_led: &mut impl embedded_hal::digital::v2::OutputPin,
  speed_led1: &mut impl embedded_hal::digital::v2::OutputPin,
  speed_led2: &mut impl embedded_hal::digital::v2::OutputPin,
  speed_led3: &mut impl embedded_hal::digital::v2::OutputPin,
  position_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  // 方向指示
  match encoder.get_direction() {
    RotationDirection::Clockwise => direction_led.set_high().ok(),
    RotationDirection::CounterClockwise => direction_led.set_low().ok(),
  }

  match mode {
    DisplayMode::Position => {
      // 位置模式：根据位置显示LED模式
      let position = encoder.get_position().abs() as u32;
      let pattern = position % 8;

      speed_led1.set_state(((pattern & 0x01) != 0).into()).ok();
      speed_led2.set_state(((pattern & 0x02) != 0).into()).ok();
      speed_led3.set_state(((pattern & 0x04) != 0).into()).ok();

      // 位置LED根据位置奇偶性闪烁
      position_led.set_state((position % 2 == 0).into()).ok();
    }

    DisplayMode::Velocity => {
      // 速度模式：根据速度显示LED数量
      let velocity = encoder.get_velocity_rpm().abs();

      speed_led1.set_state((velocity > 10.0).into()).ok();
      speed_led2.set_state((velocity > 50.0).into()).ok();
      speed_led3.set_state((velocity > 100.0).into()).ok();

      // 位置LED根据速度快慢闪烁
      let blink_rate = if velocity > 50.0 { 5 } else { 20 };
      position_led
        .set_state(((counter / blink_rate) % 2 == 0).into())
        .ok();
    }

    DisplayMode::Direction => {
      // 方向模式：显示旋转方向
      match encoder.get_direction() {
        RotationDirection::Clockwise => {
          // 顺时针：LED依次点亮
          let phase = (counter / 10) % 4;
          speed_led1.set_state((phase == 0).into()).ok();
          speed_led2.set_state((phase == 1).into()).ok();
          speed_led3.set_state((phase == 2).into()).ok();
          position_led.set_state((phase == 3).into()).ok();
        }
        RotationDirection::CounterClockwise => {
          // 逆时针：LED反向依次点亮
          let phase = (counter / 10) % 4;
          position_led.set_state((phase == 0).into()).ok();
          speed_led3.set_state((phase == 1).into()).ok();
          speed_led2.set_state((phase == 2).into()).ok();
          speed_led1.set_state((phase == 3).into()).ok();
        }
      }
    }

    DisplayMode::Angle => {
      // 角度模式：根据角度显示
      let angle = encoder.get_angle_degrees().abs();
      let sector = ((angle / 90.0) as u32) % 4;

      speed_led1.set_state((sector == 0).into()).ok();
      speed_led2.set_state((sector == 1).into()).ok();
      speed_led3.set_state((sector == 2).into()).ok();
      position_led.set_state((sector == 3).into()).ok();
    }
  }
}

/// 显示模式
#[derive(Clone, Copy)]
enum DisplayMode {
  Position,  // 位置显示
  Velocity,  // 速度显示
  Direction, // 方向显示
  Angle,     // 角度显示
}

/// 按钮防抖器
struct ButtonDebouncer {
  last_state: bool,
  stable_state: bool,
  counter: u8,
}

impl ButtonDebouncer {
  fn new() -> Self {
    Self {
      last_state: false,
      stable_state: false,
      counter: 0,
    }
  }

  fn update(&mut self, current_state: bool) -> bool {
    if current_state != self.last_state {
      self.counter = 0;
      self.last_state = current_state;
    } else {
      if self.counter < 5 {
        self.counter += 1;
      }
    }

    if self.counter >= 5 && current_state != self.stable_state {
      self.stable_state = current_state;
      return current_state; // 返回按下事件
    }

    false
  }
}
