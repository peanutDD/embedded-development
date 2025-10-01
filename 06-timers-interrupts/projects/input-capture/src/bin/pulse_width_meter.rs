#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use input_capture::{CaptureEvent, EdgeType, MeasurementResult};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::gpioc::PC13,
  gpio::{
    gpioa::{PA0, PA1},
    Floating, Input,
  },
  gpio::{
    gpiob::{PB0, PB1, PB2, PB3},
    Input as GpioInput, Output, PullUp, PushPull,
  },
  interrupt,
  prelude::*,
  stm32,
  timer::{Channel, Event, Timer},
};

type StatusLed = PB0<Output<PushPull>>;
type MeasureLed = PB1<Output<PushPull>>;
type ValidLed = PB2<Output<PushPull>>;
type ErrorLed = PB3<Output<PushPull>>;
type ModeButton = PC13<GpioInput<PullUp>>;

type InputPin1 = PA0<Input<Floating>>;
type InputPin2 = PA1<Input<Floating>>;

// 全局变量
static mut CAPTURE_QUEUE: Queue<CaptureEvent, 32> = Queue::new();
static mut CAPTURE_PRODUCER: Option<Producer<CaptureEvent, 32>> = None;
static mut CAPTURE_CONSUMER: Option<Consumer<CaptureEvent, 32>> = None;

static TIMER_FREQUENCY: AtomicU32 = AtomicU32::new(84_000_000);

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
  let mut measure_led = gpiob.pb1.into_push_pull_output();
  let mut valid_led = gpiob.pb2.into_push_pull_output();
  let mut error_led = gpiob.pb3.into_push_pull_output();
  let mode_button = gpioc.pc13.into_pull_up_input();

  // 配置输入捕获引脚
  let _input_pin1 = gpioa.pa0.into_floating_input();
  let _input_pin2 = gpioa.pa1.into_floating_input();

  // 初始化队列
  let (producer, consumer) = unsafe { CAPTURE_QUEUE.split() };
  unsafe {
    CAPTURE_PRODUCER = Some(producer);
    CAPTURE_CONSUMER = Some(consumer);
  }

  // 配置输入捕获定时器
  let mut capture_timer = Timer::tim2(dp.TIM2, &clocks);
  capture_timer.start(1.mhz()); // 1MHz = 1μs分辨率
  capture_timer.listen(Event::TimeOut);

  // 启用NVIC中断
  unsafe {
    NVIC::unmask(stm32::Interrupt::TIM2);
  }

  // 配置延时定时器
  let mut delay = Timer::tim3(dp.TIM3, &clocks).delay_ms();

  // 系统启动指示
  status_led.set_high();
  delay.delay_ms(1000u32);

  // 脉冲宽度测量器
  let mut pulse_width_meter = PulseWidthMeter::new(84_000_000);
  let mut button_debouncer = ButtonDebouncer::new();
  let mut measurement_mode = MeasurementMode::SinglePulse;
  let mut demo_counter = 0u32;

  loop {
    // 按钮防抖处理
    let button_pressed = button_debouncer.update(mode_button.is_low());

    if button_pressed {
      // 切换测量模式
      measurement_mode = match measurement_mode {
        MeasurementMode::SinglePulse => MeasurementMode::DutyCycle,
        MeasurementMode::DutyCycle => MeasurementMode::Frequency,
        MeasurementMode::Frequency => MeasurementMode::Continuous,
        MeasurementMode::Continuous => MeasurementMode::SinglePulse,
      };

      // 重置测量器
      pulse_width_meter.reset();

      // 模式切换指示
      for _ in 0..3 {
        status_led.set_low();
        delay.delay_ms(100u32);
        status_led.set_high();
        delay.delay_ms(100u32);
      }
    }

    // 处理捕获事件
    if let Some(consumer) = unsafe { CAPTURE_CONSUMER.as_mut() } {
      while let Some(event) = consumer.dequeue() {
        measure_led.set_high();

        let result = pulse_width_meter.add_capture_event(event);

        match result {
          Ok(_) => {
            valid_led.set_high();
            error_led.set_low();
          }
          Err(_) => {
            error_led.set_high();
            valid_led.set_low();
          }
        }

        delay.delay_ms(50u32);
        measure_led.set_low();
        delay.delay_ms(50u32);
      }
    }

    // 显示测量结果
    display_measurement_results(
      &pulse_width_meter,
      measurement_mode,
      &mut valid_led,
      &mut error_led,
      demo_counter,
    );

    demo_counter += 1;

    // 状态LED心跳
    if demo_counter % 100 == 0 {
      status_led.toggle();
    }

    delay.delay_ms(10u32);
  }
}

/// 显示测量结果
fn display_measurement_results(
  meter: &PulseWidthMeter,
  mode: MeasurementMode,
  valid_led: &mut impl embedded_hal::digital::v2::OutputPin,
  error_led: &mut impl embedded_hal::digital::v2::OutputPin,
  counter: u32,
) {
  let measurement = meter.get_measurement();

  match mode {
    MeasurementMode::SinglePulse => {
      // 单脉冲模式：根据脉冲宽度闪烁
      if measurement.pulse_width_us > 0.0 {
        let blink_rate = if measurement.pulse_width_us < 1000.0 {
          10
        } else {
          20
        };
        valid_led
          .set_state(((counter / blink_rate) % 2 == 0).into())
          .ok();
      }
    }

    MeasurementMode::DutyCycle => {
      // 占空比模式：根据占空比显示
      if measurement.duty_cycle > 0.0 {
        let duty_percent = (measurement.duty_cycle * 100.0) as u32;
        let on_time = duty_percent / 10; // 0-10的范围
        let cycle_pos = (counter / 10) % 10;
        valid_led.set_state((cycle_pos < on_time).into()).ok();
      }
    }

    MeasurementMode::Frequency => {
      // 频率模式：根据频率快慢闪烁
      if measurement.frequency_hz > 0.0 {
        let blink_rate = if measurement.frequency_hz > 100.0 {
          5
        } else {
          15
        };
        valid_led
          .set_state(((counter / blink_rate) % 2 == 0).into())
          .ok();
      }
    }

    MeasurementMode::Continuous => {
      // 连续模式：持续显示测量状态
      if meter.is_measuring() {
        valid_led.set_state(((counter / 5) % 2 == 0).into()).ok();
      } else {
        valid_led.set_low().ok();
      }
    }
  }

  // 错误指示
  if meter.has_error() {
    error_led.set_state(((counter / 20) % 2 == 0).into()).ok();
  } else {
    error_led.set_low().ok();
  }
}

/// 脉冲宽度测量器
struct PulseWidthMeter {
  timer_frequency: u32,
  last_rising_edge: Option<u32>,
  last_falling_edge: Option<u32>,
  pulse_width_us: f32,
  period_us: f32,
  duty_cycle: f32,
  frequency_hz: f32,
  measurement_count: u32,
  error_count: u32,
  measuring: bool,
  pulse_buffer: heapless::Vec<f32, 100>,
}

impl PulseWidthMeter {
  fn new(timer_frequency: u32) -> Self {
    Self {
      timer_frequency,
      last_rising_edge: None,
      last_falling_edge: None,
      pulse_width_us: 0.0,
      period_us: 0.0,
      duty_cycle: 0.0,
      frequency_hz: 0.0,
      measurement_count: 0,
      error_count: 0,
      measuring: false,
      pulse_buffer: heapless::Vec::new(),
    }
  }

  fn add_capture_event(&mut self, event: CaptureEvent) -> Result<(), &'static str> {
    self.measuring = true;

    match event.edge_type {
      EdgeType::Rising => {
        // 计算周期
        if let Some(last_rising) = self.last_rising_edge {
          let period_ticks = event.timestamp.wrapping_sub(last_rising);
          self.period_us = (period_ticks as f32 * 1_000_000.0) / self.timer_frequency as f32;

          if self.period_us > 0.0 {
            self.frequency_hz = 1_000_000.0 / self.period_us;
          }
        }

        self.last_rising_edge = Some(event.timestamp);
      }

      EdgeType::Falling => {
        // 计算脉冲宽度
        if let Some(rising_time) = self.last_rising_edge {
          let pulse_ticks = event.timestamp.wrapping_sub(rising_time);
          self.pulse_width_us = (pulse_ticks as f32 * 1_000_000.0) / self.timer_frequency as f32;

          // 计算占空比
          if self.period_us > 0.0 {
            self.duty_cycle = self.pulse_width_us / self.period_us;
          }

          // 添加到缓冲区
          if self.pulse_buffer.push(self.pulse_width_us).is_err() {
            self.pulse_buffer.clear();
            self.pulse_buffer.push(self.pulse_width_us).ok();
          }

          self.measurement_count += 1;
        } else {
          self.error_count += 1;
          return Err("Missing rising edge");
        }

        self.last_falling_edge = Some(event.timestamp);
      }
    }

    Ok(())
  }

  fn get_measurement(&self) -> MeasurementResult {
    MeasurementResult {
      frequency_hz: self.frequency_hz,
      period_us: self.period_us,
      duty_cycle: self.duty_cycle,
      pulse_width_us: self.pulse_width_us,
      pulse_count: self.measurement_count as u64,
      measurement_time_ms: 0, // 简化实现
    }
  }

  fn is_measuring(&self) -> bool {
    self.measuring
  }

  fn has_error(&self) -> bool {
    self.error_count > 0
  }

  fn reset(&mut self) {
    self.last_rising_edge = None;
    self.last_falling_edge = None;
    self.pulse_width_us = 0.0;
    self.period_us = 0.0;
    self.duty_cycle = 0.0;
    self.frequency_hz = 0.0;
    self.measurement_count = 0;
    self.error_count = 0;
    self.measuring = false;
    self.pulse_buffer.clear();
  }

  fn get_average_pulse_width(&self) -> f32 {
    if self.pulse_buffer.is_empty() {
      return 0.0;
    }

    let sum: f32 = self.pulse_buffer.iter().sum();
    sum / self.pulse_buffer.len() as f32
  }
}

/// 测量模式
#[derive(Clone, Copy)]
enum MeasurementMode {
  SinglePulse, // 单脉冲测量
  DutyCycle,   // 占空比测量
  Frequency,   // 频率测量
  Continuous,  // 连续测量
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

// 中断处理函数
#[interrupt]
fn TIM2() {
  // 模拟输入捕获事件
  static mut COUNTER: u32 = 0;

  unsafe {
    *COUNTER += 1;

    // 模拟边沿检测
    let edge_type = if *COUNTER % 2 == 0 {
      EdgeType::Rising
    } else {
      EdgeType::Falling
    };

    let event = CaptureEvent {
      timestamp: *COUNTER * 1000, // 模拟时间戳
      edge_type,
      channel: 0,
    };

    if let Some(producer) = CAPTURE_PRODUCER.as_mut() {
      producer.enqueue(event).ok();
    }
  }

  // 清除中断标志
  unsafe {
    let tim2 = &(*stm32::TIM2::ptr());
    tim2.sr.modify(|_, w| w.uif().clear_bit());
  }
}
