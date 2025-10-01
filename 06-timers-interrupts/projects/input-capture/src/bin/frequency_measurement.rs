#![no_std]
#![no_main]

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless::spsc::{Consumer, Producer, Queue};
use input_capture::{CaptureEvent, CaptureMode, EdgeType, FrequencyMeter, MeasurementResult};
use libm::{fabsf, sqrtf};
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
  gpio::{Input, Output, Pin, PullUp, PushPull},
  interrupt, pac,
  prelude::*,
  rcc::Clocks,
  timer::{CounterUs, Event, Timer},
};

// 频率测量事件类型
#[derive(Debug, Clone, Copy)]
enum FrequencyEvent {
  CaptureRising(u32),        // 上升沿捕获
  CaptureFalling(u32),       // 下降沿捕获
  MeasurementComplete,       // 测量完成
  CalibrationStart,          // 校准开始
  CalibrationEnd,            // 校准结束
  ModeChange(FrequencyMode), // 模式切换
}

#[derive(Debug, Clone, Copy)]
enum FrequencyMode {
  SingleChannel, // 单通道测量
  DualChannel,   // 双通道测量
  Continuous,    // 连续测量
  Triggered,     // 触发测量
  HighPrecision, // 高精度测量
}

// 频率测量管理器
struct FrequencyMeasurementManager {
  frequency_meter1: FrequencyMeter,
  frequency_meter2: FrequencyMeter,
  current_mode: FrequencyMode,
  measurement_active: bool,
  calibration_active: bool,
  reference_frequency: f32,
  statistics: FrequencyStatistics,
  event_producer: Producer<'static, FrequencyEvent, 64>,
}

#[derive(Debug, Default)]
struct FrequencyStatistics {
  total_measurements: u32,
  successful_measurements: u32,
  failed_measurements: u32,
  min_frequency: f32,
  max_frequency: f32,
  avg_frequency: f32,
  frequency_stability: f32,
  measurement_time_ms: u32,
  calibration_count: u32,
  mode_changes: u32,
}

impl FrequencyMeasurementManager {
  fn new(timer_frequency: u32, event_producer: Producer<'static, FrequencyEvent, 64>) -> Self {
    Self {
      frequency_meter1: FrequencyMeter::new(timer_frequency),
      frequency_meter2: FrequencyMeter::new(timer_frequency),
      current_mode: FrequencyMode::SingleChannel,
      measurement_active: false,
      calibration_active: false,
      reference_frequency: 1000.0, // 1kHz参考频率
      statistics: FrequencyStatistics {
        min_frequency: f32::MAX,
        max_frequency: 0.0,
        ..Default::default()
      },
      event_producer,
    }
  }

  fn set_mode(&mut self, mode: FrequencyMode) {
    self.current_mode = mode;
    self.statistics.mode_changes += 1;
    rprintln!("频率测量模式切换到: {:?}", mode);
  }

  fn start_measurement(&mut self) {
    self.measurement_active = true;
    self.frequency_meter1.reset();
    self.frequency_meter2.reset();
    rprintln!("开始频率测量");
  }

  fn stop_measurement(&mut self) {
    self.measurement_active = false;
    rprintln!("停止频率测量");
  }

  fn process_capture(
    &mut self,
    channel: u8,
    timestamp: u32,
    edge_type: EdgeType,
  ) -> Result<(), &'static str> {
    if !self.measurement_active {
      return Ok(());
    }

    match self.current_mode {
      FrequencyMode::SingleChannel => {
        if channel == 1 {
          self.frequency_meter1.add_capture(timestamp)?;
        }
      }
      FrequencyMode::DualChannel => {
        if channel == 1 {
          self.frequency_meter1.add_capture(timestamp)?;
        } else if channel == 2 {
          self.frequency_meter2.add_capture(timestamp)?;
        }
      }
      FrequencyMode::Continuous => {
        // 连续测量模式，两个通道都处理
        if channel == 1 {
          self.frequency_meter1.add_capture(timestamp)?;
        } else if channel == 2 {
          self.frequency_meter2.add_capture(timestamp)?;
        }
      }
      FrequencyMode::Triggered => {
        // 触发测量模式，只在特定条件下测量
        if edge_type == EdgeType::Rising && channel == 1 {
          self.frequency_meter1.add_capture(timestamp)?;
        }
      }
      FrequencyMode::HighPrecision => {
        // 高精度模式，使用两个通道进行差分测量
        if channel == 1 {
          self.frequency_meter1.add_capture(timestamp)?;
        } else if channel == 2 {
          self.frequency_meter2.add_capture(timestamp)?;
        }
      }
    }

    Ok(())
  }

  fn update_measurements(&mut self, dt_ms: u32) -> Result<(), &'static str> {
    if !self.measurement_active {
      return Ok(());
    }

    // 更新频率计算
    let freq1 = self.frequency_meter1.calculate_frequency();
    let freq2 = self.frequency_meter2.calculate_frequency();

    // 根据模式处理测量结果
    let measured_frequency = match self.current_mode {
      FrequencyMode::SingleChannel => freq1,
      FrequencyMode::DualChannel => {
        // 双通道模式，可以比较两个频率
        if freq1 > 0.0 && freq2 > 0.0 {
          (freq1 + freq2) / 2.0 // 平均值
        } else if freq1 > 0.0 {
          freq1
        } else {
          freq2
        }
      }
      FrequencyMode::Continuous => freq1,
      FrequencyMode::Triggered => freq1,
      FrequencyMode::HighPrecision => {
        // 高精度模式，使用差分测量提高精度
        if freq1 > 0.0 && freq2 > 0.0 {
          // 使用两个通道的相位差进行高精度测量
          let phase_diff = fabsf(freq1 - freq2);
          if phase_diff < freq1 * 0.01 {
            // 1%误差内
            (freq1 + freq2) / 2.0
          } else {
            freq1 // 使用主通道
          }
        } else {
          freq1
        }
      }
    };

    // 更新统计信息
    if measured_frequency > 0.0 {
      self.update_statistics(measured_frequency, dt_ms);
    }

    Ok(())
  }

  fn update_statistics(&mut self, frequency: f32, dt_ms: u32) {
    self.statistics.total_measurements += 1;
    self.statistics.measurement_time_ms += dt_ms;

    if frequency > 0.0 {
      self.statistics.successful_measurements += 1;

      if frequency < self.statistics.min_frequency {
        self.statistics.min_frequency = frequency;
      }
      if frequency > self.statistics.max_frequency {
        self.statistics.max_frequency = frequency;
      }

      // 计算平均频率（简化的移动平均）
      let alpha = 0.1; // 平滑因子
      if self.statistics.avg_frequency == 0.0 {
        self.statistics.avg_frequency = frequency;
      } else {
        self.statistics.avg_frequency =
          self.statistics.avg_frequency * (1.0 - alpha) + frequency * alpha;
      }

      // 计算频率稳定性（标准差的简化估计）
      let deviation = fabsf(frequency - self.statistics.avg_frequency);
      self.statistics.frequency_stability =
        self.statistics.frequency_stability * 0.9 + deviation * 0.1;
    } else {
      self.statistics.failed_measurements += 1;
    }
  }

  fn start_calibration(&mut self) {
    self.calibration_active = true;
    self.statistics.calibration_count += 1;
    rprintln!("开始频率校准");
  }

  fn end_calibration(&mut self) {
    self.calibration_active = false;
    rprintln!("频率校准完成");
  }

  fn get_measurement_result(&self) -> MeasurementResult {
    match self.current_mode {
      FrequencyMode::SingleChannel | FrequencyMode::Continuous | FrequencyMode::Triggered => {
        self.frequency_meter1.get_measurement()
      }
      FrequencyMode::DualChannel | FrequencyMode::HighPrecision => {
        let result1 = self.frequency_meter1.get_measurement();
        let result2 = self.frequency_meter2.get_measurement();

        // 合并两个通道的结果
        MeasurementResult {
          frequency_hz: (result1.frequency_hz + result2.frequency_hz) / 2.0,
          period_us: (result1.period_us + result2.period_us) / 2.0,
          duty_cycle: (result1.duty_cycle + result2.duty_cycle) / 2.0,
          pulse_width_us: (result1.pulse_width_us + result2.pulse_width_us) / 2.0,
          pulse_count: result1.pulse_count + result2.pulse_count,
          measurement_time_ms: result1.measurement_time_ms.max(result2.measurement_time_ms),
        }
      }
    }
  }

  fn send_event(&mut self, event: FrequencyEvent) {
    if self.event_producer.enqueue(event).is_err() {
      rprintln!("警告: 事件队列已满");
    }
  }
}

static mut FREQUENCY_EVENT_QUEUE: Queue<FrequencyEvent, 64> = Queue::new();
static mut CAPTURE_TIMESTAMP: u32 = 0;

#[entry]
fn main() -> ! {
  rtt_init_print!();
  rprintln!("频率测量系统启动");

  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 配置按钮
  let button1 = gpioc.pc13.into_pull_up_input(); // 模式切换
  let button2 = gpioc.pc14.into_pull_up_input(); // 测量控制

  // 配置LED指示灯
  let mut status_led = gpiob.pb0.into_push_pull_output(); // 系统状态
  let mut measurement_led = gpiob.pb1.into_push_pull_output(); // 测量活动
  let mut ch1_led = gpiob.pb2.into_push_pull_output(); // 通道1活动
  let mut ch2_led = gpiob.pb3.into_push_pull_output(); // 通道2活动
  let mut mode_led = gpiob.pb4.into_push_pull_output(); // 模式指示
  let mut precision_led = gpiob.pb5.into_push_pull_output(); // 精度指示
  let mut calibration_led = gpiob.pb6.into_push_pull_output(); // 校准指示
  let mut error_led = gpiob.pb7.into_push_pull_output(); // 错误指示

  // 配置输入捕获引脚
  let capture_pin1 = gpioa.pa0.into_alternate(); // TIM2_CH1
  let capture_pin2 = gpioa.pa1.into_alternate(); // TIM2_CH2

  // 配置定时器用于输入捕获
  let mut timer2 = Timer::new(dp.TIM2, &clocks).counter_hz();
  timer2.start(1.MHz()).unwrap(); // 1MHz计数频率，1us分辨率
  timer2.listen(Event::Update);

  // 配置系统定时器
  let mut delay = cp.SYST.delay(&clocks);

  // 创建事件队列
  let (event_producer, mut event_consumer) = unsafe { FREQUENCY_EVENT_QUEUE.split() };

  // 创建频率测量管理器
  let mut freq_manager = FrequencyMeasurementManager::new(
    1_000_000, // 1MHz定时器频率
    event_producer,
  );

  // 启用中断
  unsafe {
    NVIC::unmask(pac::Interrupt::TIM2);
  }

  // 系统变量
  let mut last_button1_state = button1.is_high();
  let mut last_button2_state = button2.is_high();
  let mut button1_debounce = 0u32;
  let mut button2_debounce = 0u32;
  let mut system_tick = 0u32;
  let mut last_measurement_time = 0u32;

  rprintln!("频率测量系统就绪");

  loop {
    let current_time = system_tick;
    system_tick = system_tick.wrapping_add(1);

    // 按钮1处理（模式切换）
    let button1_state = button1.is_high();
    if button1_state != last_button1_state {
      button1_debounce = current_time;
    }

    if current_time.wrapping_sub(button1_debounce) > 50 && button1_state && !last_button1_state {
      let new_mode = match freq_manager.current_mode {
        FrequencyMode::SingleChannel => FrequencyMode::DualChannel,
        FrequencyMode::DualChannel => FrequencyMode::Continuous,
        FrequencyMode::Continuous => FrequencyMode::Triggered,
        FrequencyMode::Triggered => FrequencyMode::HighPrecision,
        FrequencyMode::HighPrecision => FrequencyMode::SingleChannel,
      };
      freq_manager.send_event(FrequencyEvent::ModeChange(new_mode));
    }
    last_button1_state = button1_state;

    // 按钮2处理（测量控制）
    let button2_state = button2.is_high();
    if button2_state != last_button2_state {
      button2_debounce = current_time;
    }

    if current_time.wrapping_sub(button2_debounce) > 50 && button2_state && !last_button2_state {
      if freq_manager.measurement_active {
        freq_manager.stop_measurement();
      } else {
        freq_manager.start_measurement();
      }
    }
    last_button2_state = button2_state;

    // 处理事件队列
    while let Some(event) = event_consumer.dequeue() {
      match event {
        FrequencyEvent::CaptureRising(timestamp) => {
          if let Err(e) = freq_manager.process_capture(1, timestamp, EdgeType::Rising) {
            rprintln!("处理上升沿捕获错误: {}", e);
            error_led.set_high();
          } else {
            error_led.set_low();
            ch1_led.set_high();
          }
        }
        FrequencyEvent::CaptureFalling(timestamp) => {
          if let Err(e) = freq_manager.process_capture(1, timestamp, EdgeType::Falling) {
            rprintln!("处理下降沿捕获错误: {}", e);
            error_led.set_high();
          } else {
            error_led.set_low();
            ch1_led.set_low();
          }
        }
        FrequencyEvent::MeasurementComplete => {
          let result = freq_manager.get_measurement_result();
          rprintln!(
            "测量完成 - 频率: {:.2} Hz, 周期: {:.2} us",
            result.frequency_hz,
            result.period_us
          );
        }
        FrequencyEvent::CalibrationStart => {
          freq_manager.start_calibration();
        }
        FrequencyEvent::CalibrationEnd => {
          freq_manager.end_calibration();
        }
        FrequencyEvent::ModeChange(mode) => {
          freq_manager.set_mode(mode);
        }
      }
    }

    // 更新测量
    if let Err(e) = freq_manager.update_measurements(10) {
      rprintln!("更新测量错误: {}", e);
      error_led.set_high();
    }

    // LED指示更新
    // 系统状态LED（心跳）
    if system_tick % 100 == 0 {
      status_led.toggle();
    }

    // 测量活动指示
    if freq_manager.measurement_active {
      measurement_led.set_high();
    } else {
      measurement_led.set_low();
    }

    // 模式指示LED（不同模式不同闪烁频率）
    let mode_blink_period = match freq_manager.current_mode {
      FrequencyMode::SingleChannel => 2000,
      FrequencyMode::DualChannel => 1000,
      FrequencyMode::Continuous => 500,
      FrequencyMode::Triggered => 200,
      FrequencyMode::HighPrecision => 100,
    };

    if system_tick % mode_blink_period < mode_blink_period / 2 {
      mode_led.set_high();
    } else {
      mode_led.set_low();
    }

    // 精度指示（根据频率稳定性）
    let stability = freq_manager.statistics.frequency_stability;
    if stability < 1.0 {
      precision_led.set_high(); // 高精度
    } else if stability < 10.0 {
      if system_tick % 500 < 250 {
        precision_led.set_high(); // 中等精度
      } else {
        precision_led.set_low();
      }
    } else {
      precision_led.set_low(); // 低精度
    }

    // 校准指示
    if freq_manager.calibration_active {
      calibration_led.set_high();
    } else {
      calibration_led.set_low();
    }

    // 通道2活动指示（双通道模式下）
    match freq_manager.current_mode {
      FrequencyMode::DualChannel | FrequencyMode::HighPrecision => {
        if system_tick % 200 < 100 {
          ch2_led.set_high();
        } else {
          ch2_led.set_low();
        }
      }
      _ => {
        ch2_led.set_low();
      }
    }

    // 定期输出统计信息
    if system_tick % 5000 == 0 {
      rprintln!("=== 频率测量统计信息 ===");
      rprintln!("总测量次数: {}", freq_manager.statistics.total_measurements);
      rprintln!(
        "成功测量: {}",
        freq_manager.statistics.successful_measurements
      );
      rprintln!("失败测量: {}", freq_manager.statistics.failed_measurements);
      rprintln!(
        "频率范围: {:.2} - {:.2} Hz",
        freq_manager.statistics.min_frequency,
        freq_manager.statistics.max_frequency
      );
      rprintln!("平均频率: {:.2} Hz", freq_manager.statistics.avg_frequency);
      rprintln!(
        "频率稳定性: {:.3} Hz",
        freq_manager.statistics.frequency_stability
      );
      rprintln!(
        "测量时间: {} ms",
        freq_manager.statistics.measurement_time_ms
      );
      rprintln!("校准次数: {}", freq_manager.statistics.calibration_count);
      rprintln!("模式切换: {}", freq_manager.statistics.mode_changes);
      rprintln!("当前模式: {:?}", freq_manager.current_mode);

      let result = freq_manager.get_measurement_result();
      rprintln!(
        "当前测量 - 频率: {:.2} Hz, 周期: {:.2} us, 占空比: {:.1}%",
        result.frequency_hz,
        result.period_us,
        result.duty_cycle * 100.0
      );
    }

    delay.delay_ms(10u32);
  }
}

// 定时器中断处理
#[interrupt]
fn TIM2() {
  // 这里应该处理输入捕获中断
  // 实际实现需要根据具体的HAL库接口
  unsafe {
    CAPTURE_TIMESTAMP = CAPTURE_TIMESTAMP.wrapping_add(1);
    // 发送捕获事件到队列
    // 这里简化处理，实际需要读取捕获寄存器
  }
}
