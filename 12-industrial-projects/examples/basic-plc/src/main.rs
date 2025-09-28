#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
  gpio::{Input, Output, Pin, PullUp, PushPull},
  i2c::I2c,
  prelude::*,
  serial::{Config, Serial},
  spi::{Mode, Phase, Polarity, Spi},
  stm32,
  timer::{Event, Timer},
};

use heapless::{FnvIndexMap, Vec};
use nb;

/// PLC系统配置
const MAX_DIGITAL_INPUTS: usize = 16;
const MAX_DIGITAL_OUTPUTS: usize = 16;
const MAX_ANALOG_INPUTS: usize = 8;
const MAX_ANALOG_OUTPUTS: usize = 4;
const SCAN_CYCLE_MS: u32 = 10; // 10ms扫描周期

/// 数字输入状态
#[derive(Clone, Copy, PartialEq)]
pub struct DigitalInput {
  pub current: bool,
  pub previous: bool,
  pub rising_edge: bool,
  pub falling_edge: bool,
  pub debounce_counter: u8,
}

impl DigitalInput {
  pub fn new() -> Self {
    Self {
      current: false,
      previous: false,
      rising_edge: false,
      falling_edge: false,
      debounce_counter: 0,
    }
  }

  pub fn update(&mut self, raw_input: bool) {
    // 防抖处理
    if raw_input == self.current {
      self.debounce_counter = 0;
    } else {
      self.debounce_counter += 1;
      if self.debounce_counter >= 3 {
        // 3个扫描周期确认
        self.previous = self.current;
        self.current = raw_input;
        self.debounce_counter = 0;

        // 边沿检测
        self.rising_edge = self.current && !self.previous;
        self.falling_edge = !self.current && self.previous;
      }
    }
  }
}

/// 数字输出状态
#[derive(Clone, Copy)]
pub struct DigitalOutput {
  pub state: bool,
  pub pulse_timer: u16,
  pub pulse_duration: u16,
}

impl DigitalOutput {
  pub fn new() -> Self {
    Self {
      state: false,
      pulse_timer: 0,
      pulse_duration: 0,
    }
  }

  pub fn set(&mut self, state: bool) {
    self.state = state;
    self.pulse_timer = 0;
  }

  pub fn pulse(&mut self, duration_ms: u16) {
    self.state = true;
    self.pulse_timer = duration_ms / SCAN_CYCLE_MS as u16;
    self.pulse_duration = self.pulse_timer;
  }

  pub fn update(&mut self) {
    if self.pulse_timer > 0 {
      self.pulse_timer -= 1;
      if self.pulse_timer == 0 {
        self.state = false;
      }
    }
  }
}

/// 模拟输入
#[derive(Clone, Copy)]
pub struct AnalogInput {
  pub raw_value: u16,
  pub scaled_value: f32,
  pub min_scale: f32,
  pub max_scale: f32,
  pub filter_buffer: [u16; 8],
  pub filter_index: usize,
}

impl AnalogInput {
  pub fn new(min_scale: f32, max_scale: f32) -> Self {
    Self {
      raw_value: 0,
      scaled_value: 0.0,
      min_scale,
      max_scale,
      filter_buffer: [0; 8],
      filter_index: 0,
    }
  }

  pub fn update(&mut self, raw_value: u16) {
    // 滑动平均滤波
    self.filter_buffer[self.filter_index] = raw_value;
    self.filter_index = (self.filter_index + 1) % self.filter_buffer.len();

    let sum: u32 = self.filter_buffer.iter().map(|&x| x as u32).sum();
    self.raw_value = (sum / self.filter_buffer.len() as u32) as u16;

    // 线性缩放
    let normalized = self.raw_value as f32 / 4095.0; // 12位ADC
    self.scaled_value = self.min_scale + normalized * (self.max_scale - self.min_scale);
  }
}

/// 梯形图逻辑元素
#[derive(Clone, Copy, PartialEq)]
pub enum LadderElement {
  NormallyOpen(u8),   // 常开触点
  NormallyClosed(u8), // 常闭触点
  Coil(u8),           // 线圈
  Timer(u8),          // 定时器
  Counter(u8),        // 计数器
  Compare(u8),        // 比较器
}

/// 梯形图指令
#[derive(Clone, Copy)]
pub struct LadderInstruction {
  pub element: LadderElement,
  pub operand: u16,
}

/// 定时器类型
#[derive(Clone, Copy, PartialEq)]
pub enum TimerType {
  OnDelay,  // 通电延时
  OffDelay, // 断电延时
  Pulse,    // 脉冲
}

/// 定时器状态
#[derive(Clone, Copy)]
pub struct Timer {
  pub timer_type: TimerType,
  pub preset: u32,   // 预设值(ms)
  pub current: u32,  // 当前值(ms)
  pub input: bool,   // 输入状态
  pub output: bool,  // 输出状态
  pub done: bool,    // 完成标志
  pub enabled: bool, // 使能标志
}

impl Timer {
  pub fn new(timer_type: TimerType, preset: u32) -> Self {
    Self {
      timer_type,
      preset,
      current: 0,
      input: false,
      output: false,
      done: false,
      enabled: false,
    }
  }

  pub fn update(&mut self, input: bool, scan_time_ms: u32) {
    self.input = input;

    match self.timer_type {
      TimerType::OnDelay => {
        if input && !self.enabled {
          self.enabled = true;
          self.current = 0;
        } else if !input {
          self.enabled = false;
          self.current = 0;
          self.output = false;
          self.done = false;
        }

        if self.enabled {
          self.current += scan_time_ms;
          if self.current >= self.preset {
            self.output = true;
            self.done = true;
          }
        }
      }

      TimerType::OffDelay => {
        if input {
          self.output = true;
          self.current = 0;
          self.enabled = false;
        } else if !self.enabled {
          self.enabled = true;
          self.current = 0;
        }

        if self.enabled {
          self.current += scan_time_ms;
          if self.current >= self.preset {
            self.output = false;
            self.done = true;
          }
        }
      }

      TimerType::Pulse => {
        if input && !self.enabled {
          self.enabled = true;
          self.current = 0;
          self.output = true;
        } else if !input {
          self.enabled = false;
        }

        if self.enabled {
          self.current += scan_time_ms;
          if self.current >= self.preset {
            self.output = false;
            self.done = true;
            self.enabled = false;
          }
        }
      }
    }
  }

  pub fn reset(&mut self) {
    self.current = 0;
    self.output = false;
    self.done = false;
    self.enabled = false;
  }
}

/// 计数器
#[derive(Clone, Copy)]
pub struct Counter {
  pub preset: u32,
  pub current: u32,
  pub count_input: bool,
  pub reset_input: bool,
  pub output: bool,
  pub done: bool,
  pub prev_count_input: bool,
}

impl Counter {
  pub fn new(preset: u32) -> Self {
    Self {
      preset,
      current: 0,
      count_input: false,
      reset_input: false,
      output: false,
      done: false,
      prev_count_input: false,
    }
  }

  pub fn update(&mut self, count_input: bool, reset_input: bool) {
    self.reset_input = reset_input;

    if reset_input {
      self.current = 0;
      self.output = false;
      self.done = false;
    } else {
      // 上升沿计数
      if count_input && !self.prev_count_input {
        self.current += 1;
        if self.current >= self.preset {
          self.output = true;
          self.done = true;
        }
      }
    }

    self.prev_count_input = count_input;
    self.count_input = count_input;
  }
}

/// PLC系统主结构
pub struct PlcSystem {
  // I/O系统
  pub digital_inputs: [DigitalInput; MAX_DIGITAL_INPUTS],
  pub digital_outputs: [DigitalOutput; MAX_DIGITAL_OUTPUTS],
  pub analog_inputs: [AnalogInput; MAX_ANALOG_INPUTS],
  pub analog_outputs: [f32; MAX_ANALOG_OUTPUTS],

  // 内部继电器
  pub internal_relays: [bool; 256],

  // 定时器和计数器
  pub timers: [Timer; 32],
  pub counters: [Counter; 32],

  // 梯形图程序
  pub ladder_program: Vec<LadderInstruction, 1024>,

  // 系统状态
  pub scan_time_ms: u32,
  pub cycle_count: u32,
  pub run_mode: bool,
  pub error_flags: u32,

  // 通信状态
  pub modbus_enabled: bool,
  pub ethernet_enabled: bool,
}

impl PlcSystem {
  pub fn new() -> Self {
    let mut system = Self {
      digital_inputs: [DigitalInput::new(); MAX_DIGITAL_INPUTS],
      digital_outputs: [DigitalOutput::new(); MAX_DIGITAL_OUTPUTS],
      analog_inputs: [
        AnalogInput::new(0.0, 10.0),    // 0-10V
        AnalogInput::new(4.0, 20.0),    // 4-20mA
        AnalogInput::new(-10.0, 10.0),  // ±10V
        AnalogInput::new(0.0, 100.0),   // 0-100%
        AnalogInput::new(0.0, 1000.0),  // 0-1000 units
        AnalogInput::new(-50.0, 150.0), // Temperature
        AnalogInput::new(0.0, 10.0),    // Pressure
        AnalogInput::new(0.0, 100.0),   // Flow
      ],
      analog_outputs: [0.0; MAX_ANALOG_OUTPUTS],
      internal_relays: [false; 256],
      timers: [Timer::new(TimerType::OnDelay, 1000); 32],
      counters: [Counter::new(100); 32],
      ladder_program: Vec::new(),
      scan_time_ms: SCAN_CYCLE_MS,
      cycle_count: 0,
      run_mode: false,
      error_flags: 0,
      modbus_enabled: false,
      ethernet_enabled: false,
    };

    // 初始化示例梯形图程序
    system.init_example_program();

    system
  }

  /// 初始化示例梯形图程序
  fn init_example_program(&mut self) {
    // 示例程序：启动/停止控制
    // Rung 1: 启动按钮 (DI0) 或 运行继电器 (M0) 且 非停止按钮 (DI1) -> 运行继电器 (M0)
    self
      .ladder_program
      .push(LadderInstruction {
        element: LadderElement::NormallyOpen(0), // DI0
        operand: 0,
      })
      .ok();

    self
      .ladder_program
      .push(LadderInstruction {
        element: LadderElement::NormallyOpen(100), // M0 (内部继电器)
        operand: 0,
      })
      .ok();

    self
      .ladder_program
      .push(LadderInstruction {
        element: LadderElement::NormallyClosed(1), // DI1 (停止按钮)
        operand: 0,
      })
      .ok();

    self
      .ladder_program
      .push(LadderInstruction {
        element: LadderElement::Coil(100), // M0
        operand: 0,
      })
      .ok();

    // Rung 2: 运行继电器 (M0) -> 定时器 T0 (2秒延时)
    self
      .ladder_program
      .push(LadderInstruction {
        element: LadderElement::NormallyOpen(100), // M0
        operand: 0,
      })
      .ok();

    self
      .ladder_program
      .push(LadderInstruction {
        element: LadderElement::Timer(0), // T0
        operand: 2000,                    // 2秒
      })
      .ok();

    // Rung 3: 定时器 T0 完成 -> 输出 DO0 (电机启动)
    self
      .ladder_program
      .push(LadderInstruction {
        element: LadderElement::NormallyOpen(200), // T0 完成位
        operand: 0,
      })
      .ok();

    self
      .ladder_program
      .push(LadderInstruction {
        element: LadderElement::Coil(0), // DO0
        operand: 0,
      })
      .ok();
  }

  /// 执行扫描周期
  pub fn scan_cycle(&mut self) {
    if !self.run_mode {
      return;
    }

    // 1. 输入扫描
    self.input_scan();

    // 2. 程序执行
    self.execute_program();

    // 3. 输出更新
    self.output_update();

    // 4. 系统维护
    self.system_maintenance();

    self.cycle_count += 1;
  }

  /// 输入扫描
  fn input_scan(&mut self) {
    // 更新数字输入（这里需要实际的硬件读取）
    for i in 0..MAX_DIGITAL_INPUTS {
      let raw_input = self.read_digital_input_hardware(i);
      self.digital_inputs[i].update(raw_input);
    }

    // 更新模拟输入
    for i in 0..MAX_ANALOG_INPUTS {
      let raw_value = self.read_analog_input_hardware(i);
      self.analog_inputs[i].update(raw_value);
    }
  }

  /// 程序执行
  fn execute_program(&mut self) {
    let mut rung_result = true;
    let mut i = 0;

    while i < self.ladder_program.len() {
      let instruction = self.ladder_program[i];

      match instruction.element {
        LadderElement::NormallyOpen(addr) => {
          let input_state = self.get_input_state(addr);
          rung_result = rung_result && input_state;
        }

        LadderElement::NormallyClosed(addr) => {
          let input_state = self.get_input_state(addr);
          rung_result = rung_result && !input_state;
        }

        LadderElement::Coil(addr) => {
          self.set_output_state(addr, rung_result);
          rung_result = true; // 新的梯级开始
        }

        LadderElement::Timer(timer_id) => {
          if timer_id < 32 {
            let preset = instruction.operand as u32;
            self.timers[timer_id as usize].preset = preset;
            self.timers[timer_id as usize].update(rung_result, self.scan_time_ms);
            rung_result = self.timers[timer_id as usize].output;
          }
        }

        LadderElement::Counter(counter_id) => {
          if counter_id < 32 {
            let reset_input = false; // 简化处理
            self.counters[counter_id as usize].update(rung_result, reset_input);
            rung_result = self.counters[counter_id as usize].output;
          }
        }

        LadderElement::Compare(_) => {
          // 比较指令的实现
          // 这里简化处理
        }
      }

      i += 1;
    }
  }

  /// 输出更新
  fn output_update(&mut self) {
    // 更新数字输出
    for i in 0..MAX_DIGITAL_OUTPUTS {
      self.digital_outputs[i].update();
      self.write_digital_output_hardware(i, self.digital_outputs[i].state);
    }

    // 更新模拟输出
    for i in 0..MAX_ANALOG_OUTPUTS {
      self.write_analog_output_hardware(i, self.analog_outputs[i]);
    }
  }

  /// 系统维护
  fn system_maintenance(&mut self) {
    // 清除边沿检测标志
    for input in &mut self.digital_inputs {
      input.rising_edge = false;
      input.falling_edge = false;
    }

    // 检查系统错误
    self.check_system_errors();

    // 通信处理
    if self.modbus_enabled {
      self.handle_modbus_communication();
    }

    if self.ethernet_enabled {
      self.handle_ethernet_communication();
    }
  }

  /// 获取输入状态
  fn get_input_state(&self, addr: u8) -> bool {
    match addr {
      0..=15 => self.digital_inputs[addr as usize].current,
      100..=199 => self.internal_relays[(addr - 100) as usize],
      200..=231 => self.timers[(addr - 200) as usize].done,
      240..=255 => self.counters[(addr - 240) as usize].done,
      _ => false,
    }
  }

  /// 设置输出状态
  fn set_output_state(&mut self, addr: u8, state: bool) {
    match addr {
      0..=15 => self.digital_outputs[addr as usize].set(state),
      100..=199 => self.internal_relays[(addr - 100) as usize] = state,
      _ => {}
    }
  }

  /// 读取数字输入硬件（需要实际实现）
  fn read_digital_input_hardware(&self, _channel: usize) -> bool {
    // 这里应该读取实际的GPIO状态
    false
  }

  /// 读取模拟输入硬件（需要实际实现）
  fn read_analog_input_hardware(&self, _channel: usize) -> u16 {
    // 这里应该读取实际的ADC值
    0
  }

  /// 写入数字输出硬件（需要实际实现）
  fn write_digital_output_hardware(&self, _channel: usize, _state: bool) {
    // 这里应该控制实际的GPIO输出
  }

  /// 写入模拟输出硬件（需要实际实现）
  fn write_analog_output_hardware(&self, _channel: usize, _value: f32) {
    // 这里应该控制实际的DAC输出
  }

  /// 检查系统错误
  fn check_system_errors(&mut self) {
    // 检查扫描时间超限
    if self.scan_time_ms > 50 {
      // 50ms超时
      self.error_flags |= 0x01;
    }

    // 检查通信错误
    // 检查I/O模块错误
    // 等等...
  }

  /// 处理Modbus通信
  fn handle_modbus_communication(&mut self) {
    // Modbus通信处理
    // 读取保持寄存器、线圈等
    // 更新输入寄存器、离散输入等
  }

  /// 处理以太网通信
  fn handle_ethernet_communication(&mut self) {
    // 以太网通信处理
    // Web界面、数据记录等
  }

  /// 启动PLC
  pub fn start(&mut self) {
    self.run_mode = true;
    self.cycle_count = 0;
    self.error_flags = 0;

    // 重置所有定时器和计数器
    for timer in &mut self.timers {
      timer.reset();
    }

    for counter in &mut self.counters {
      counter.current = 0;
      counter.output = false;
      counter.done = false;
    }
  }

  /// 停止PLC
  pub fn stop(&mut self) {
    self.run_mode = false;

    // 清除所有输出
    for output in &mut self.digital_outputs {
      output.set(false);
    }

    for output in &mut self.analog_outputs {
      *output = 0.0;
    }

    // 清除内部继电器
    for relay in &mut self.internal_relays {
      *relay = false;
    }
  }

  /// 获取系统状态
  pub fn get_status(&self) -> PlcStatus {
    PlcStatus {
      run_mode: self.run_mode,
      cycle_count: self.cycle_count,
      scan_time_ms: self.scan_time_ms,
      error_flags: self.error_flags,
      digital_input_count: self.digital_inputs.iter().filter(|di| di.current).count() as u8,
      digital_output_count: self.digital_outputs.iter().filter(|do_| do_.state).count() as u8,
    }
  }
}

/// PLC状态信息
#[derive(Clone, Copy)]
pub struct PlcStatus {
  pub run_mode: bool,
  pub cycle_count: u32,
  pub scan_time_ms: u32,
  pub error_flags: u32,
  pub digital_input_count: u8,
  pub digital_output_count: u8,
}

/// 主函数
#[entry]
fn main() -> ! {
  // 获取设备外设
  let cp = cortex_m::Peripherals::take().unwrap();
  let dp = stm32::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // 配置系统定时器
  let mut timer = Timer::new(dp.TIM2, &clocks);
  timer.start(SCAN_CYCLE_MS.millis());
  timer.listen(Event::TimeOut);

  // 创建PLC系统
  let mut plc = PlcSystem::new();

  // 启动PLC
  plc.start();

  // 主循环
  loop {
    // 等待定时器中断
    nb::block!(timer.wait()).unwrap();

    // 执行PLC扫描周期
    plc.scan_cycle();

    // 可选：处理其他任务
    // - 通信处理
    // - 诊断更新
    // - 用户界面更新
  }
}
