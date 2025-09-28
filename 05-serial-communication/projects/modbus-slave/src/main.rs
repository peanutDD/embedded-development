//! Modbus RTU 从站实现
//!
//! 本项目实现了一个完整的Modbus RTU从站，支持：
//! - 标准Modbus功能码
//! - 多种数据类型（线圈、离散输入、保持寄存器、输入寄存器）
//! - CRC校验
//! - 异常响应
//! - 可配置的从站地址和波特率

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use heapless::{FnvIndexMap, Vec};
use panic_halt as _;
use stm32f4xx_hal::{
  gpio::{Alternate, Pin},
  pac::{self, interrupt, USART1},
  prelude::*,
  rcc::RccExt,
  serial::{Config, Rx, Serial, Tx},
  timer::{Event, Timer},
};

// Modbus配置常量
const SLAVE_ADDRESS: u8 = 1;
const BAUD_RATE: u32 = 9600;
const FRAME_TIMEOUT_MS: u32 = 10;
const MAX_FRAME_SIZE: usize = 256;
const MAX_COILS: u16 = 1000;
const MAX_DISCRETE_INPUTS: u16 = 1000;
const MAX_HOLDING_REGISTERS: u16 = 1000;
const MAX_INPUT_REGISTERS: u16 = 1000;

// 全局串口句柄
static G_SERIAL_TX: Mutex<RefCell<Option<Tx<USART1>>>> = Mutex::new(RefCell::new(None));
static G_SERIAL_RX: Mutex<RefCell<Option<Rx<USART1>>>> = Mutex::new(RefCell::new(None));

// 接收缓冲区和状态
static mut RX_BUFFER: [u8; MAX_FRAME_SIZE] = [0; MAX_FRAME_SIZE];
static RX_INDEX: AtomicU16 = AtomicU16::new(0);
static FRAME_READY: AtomicBool = AtomicBool::new(false);
static LAST_CHAR_TIME: AtomicU32 = AtomicU32::new(0);

// Modbus数据存储
static mut MODBUS_DATA: ModbusData = ModbusData::new();

#[entry]
fn main() -> ! {
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

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let tx_pin = gpioa.pa9.into_alternate::<7>();
  let rx_pin = gpioa.pa10.into_alternate::<7>();

  // 配置串口
  let config = Config::default()
    .baudrate(BAUD_RATE.bps())
    .wordlength_8()
    .parity_none()
    .stopbits(stm32f4xx_hal::serial::StopBits::STOP1);

  let mut serial = Serial::new(dp.USART1, (tx_pin, rx_pin), config, &clocks).unwrap();

  // 启用接收中断
  serial.listen(stm32f4xx_hal::serial::Event::Rxne);

  let (tx, rx) = serial.split();

  // 存储到全局变量
  free(|cs| {
    G_SERIAL_TX.borrow(cs).replace(Some(tx));
    G_SERIAL_RX.borrow(cs).replace(Some(rx));
  });

  // 配置帧超时定时器
  let mut timer = Timer::new(dp.TIM2, &clocks).start_count_down(1000.Hz()); // 1ms定时器
  timer.listen(Event::TimeOut);

  // 启用中断
  unsafe {
    pac::NVIC::unmask(interrupt::USART1);
    pac::NVIC::unmask(interrupt::TIM2);
  }

  // 初始化Modbus数据
  unsafe {
    MODBUS_DATA.initialize();
  }

  let mut delay = cp.SYST.delay(&clocks);
  let mut led_state = false;
  let mut counter = 0u32;

  loop {
    // 检查是否有完整的Modbus帧
    if FRAME_READY.load(Ordering::Acquire) {
      let frame_length = RX_INDEX.load(Ordering::Acquire) as usize;

      if frame_length >= 4 {
        // 最小Modbus帧长度
        let frame = unsafe { &RX_BUFFER[..frame_length] };

        // 处理Modbus帧
        if let Some(response) = process_modbus_frame(frame) {
          send_response(&response);
        }
      }

      // 重置接收状态
      RX_INDEX.store(0, Ordering::Release);
      FRAME_READY.store(false, Ordering::Release);
    }

    // 更新模拟数据（模拟传感器读数等）
    counter += 1;
    if counter % 10000 == 0 {
      unsafe {
        MODBUS_DATA.update_simulated_data(counter);
      }
      led_state = !led_state;
    }

    delay.delay_us(100u32);
  }
}

/// 处理Modbus帧
fn process_modbus_frame(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  // 检查最小帧长度
  if frame.len() < 4 {
    return None;
  }

  // 验证CRC
  if !verify_crc(frame) {
    return None;
  }

  let slave_addr = frame[0];
  let function_code = frame[1];

  // 检查从站地址
  if slave_addr != SLAVE_ADDRESS && slave_addr != 0 {
    return None; // 不是发给我们的
  }

  // 处理不同的功能码
  match function_code {
    0x01 => read_coils(frame),
    0x02 => read_discrete_inputs(frame),
    0x03 => read_holding_registers(frame),
    0x04 => read_input_registers(frame),
    0x05 => write_single_coil(frame),
    0x06 => write_single_register(frame),
    0x0F => write_multiple_coils(frame),
    0x10 => write_multiple_registers(frame),
    _ => Some(create_exception_response(slave_addr, function_code, 0x01)), // 非法功能码
  }
}

/// 读取线圈 (0x01)
fn read_coils(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  if frame.len() < 6 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  let start_addr = u16::from_be_bytes([frame[2], frame[3]]);
  let quantity = u16::from_be_bytes([frame[4], frame[5]]);

  // 检查数量范围
  if quantity == 0 || quantity > 2000 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  // 检查地址范围
  if start_addr.saturating_add(quantity) > MAX_COILS {
    return Some(create_exception_response(frame[0], frame[1], 0x02));
  }

  let byte_count = ((quantity + 7) / 8) as u8;
  let mut response = Vec::new();

  // 构建响应
  let _ = response.push(frame[0]); // 从站地址
  let _ = response.push(frame[1]); // 功能码
  let _ = response.push(byte_count); // 字节数

  // 读取线圈数据
  unsafe {
    for byte_idx in 0..byte_count {
      let mut byte_value = 0u8;

      for bit_idx in 0..8 {
        let coil_addr = start_addr + (byte_idx as u16 * 8) + bit_idx as u16;
        if coil_addr < start_addr + quantity {
          if MODBUS_DATA.get_coil(coil_addr) {
            byte_value |= 1 << bit_idx;
          }
        }
      }

      let _ = response.push(byte_value);
    }
  }

  // 添加CRC
  add_crc(&mut response);
  Some(response)
}

/// 读取离散输入 (0x02)
fn read_discrete_inputs(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  if frame.len() < 6 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  let start_addr = u16::from_be_bytes([frame[2], frame[3]]);
  let quantity = u16::from_be_bytes([frame[4], frame[5]]);

  if quantity == 0 || quantity > 2000 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  if start_addr.saturating_add(quantity) > MAX_DISCRETE_INPUTS {
    return Some(create_exception_response(frame[0], frame[1], 0x02));
  }

  let byte_count = ((quantity + 7) / 8) as u8;
  let mut response = Vec::new();

  let _ = response.push(frame[0]);
  let _ = response.push(frame[1]);
  let _ = response.push(byte_count);

  unsafe {
    for byte_idx in 0..byte_count {
      let mut byte_value = 0u8;

      for bit_idx in 0..8 {
        let input_addr = start_addr + (byte_idx as u16 * 8) + bit_idx as u16;
        if input_addr < start_addr + quantity {
          if MODBUS_DATA.get_discrete_input(input_addr) {
            byte_value |= 1 << bit_idx;
          }
        }
      }

      let _ = response.push(byte_value);
    }
  }

  add_crc(&mut response);
  Some(response)
}

/// 读取保持寄存器 (0x03)
fn read_holding_registers(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  if frame.len() < 6 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  let start_addr = u16::from_be_bytes([frame[2], frame[3]]);
  let quantity = u16::from_be_bytes([frame[4], frame[5]]);

  if quantity == 0 || quantity > 125 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  if start_addr.saturating_add(quantity) > MAX_HOLDING_REGISTERS {
    return Some(create_exception_response(frame[0], frame[1], 0x02));
  }

  let byte_count = (quantity * 2) as u8;
  let mut response = Vec::new();

  let _ = response.push(frame[0]);
  let _ = response.push(frame[1]);
  let _ = response.push(byte_count);

  unsafe {
    for i in 0..quantity {
      let reg_addr = start_addr + i;
      let reg_value = MODBUS_DATA.get_holding_register(reg_addr);
      let bytes = reg_value.to_be_bytes();
      let _ = response.push(bytes[0]);
      let _ = response.push(bytes[1]);
    }
  }

  add_crc(&mut response);
  Some(response)
}

/// 读取输入寄存器 (0x04)
fn read_input_registers(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  if frame.len() < 6 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  let start_addr = u16::from_be_bytes([frame[2], frame[3]]);
  let quantity = u16::from_be_bytes([frame[4], frame[5]]);

  if quantity == 0 || quantity > 125 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  if start_addr.saturating_add(quantity) > MAX_INPUT_REGISTERS {
    return Some(create_exception_response(frame[0], frame[1], 0x02));
  }

  let byte_count = (quantity * 2) as u8;
  let mut response = Vec::new();

  let _ = response.push(frame[0]);
  let _ = response.push(frame[1]);
  let _ = response.push(byte_count);

  unsafe {
    for i in 0..quantity {
      let reg_addr = start_addr + i;
      let reg_value = MODBUS_DATA.get_input_register(reg_addr);
      let bytes = reg_value.to_be_bytes();
      let _ = response.push(bytes[0]);
      let _ = response.push(bytes[1]);
    }
  }

  add_crc(&mut response);
  Some(response)
}

/// 写单个线圈 (0x05)
fn write_single_coil(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  if frame.len() < 6 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  let coil_addr = u16::from_be_bytes([frame[2], frame[3]]);
  let coil_value = u16::from_be_bytes([frame[4], frame[5]]);

  if coil_addr >= MAX_COILS {
    return Some(create_exception_response(frame[0], frame[1], 0x02));
  }

  // 检查线圈值（只能是0x0000或0xFF00）
  let coil_state = match coil_value {
    0x0000 => false,
    0xFF00 => true,
    _ => return Some(create_exception_response(frame[0], frame[1], 0x03)),
  };

  unsafe {
    MODBUS_DATA.set_coil(coil_addr, coil_state);
  }

  // 回显请求（成功响应）
  let mut response = Vec::new();
  for &byte in frame.iter().take(frame.len() - 2) {
    // 除了CRC
    let _ = response.push(byte);
  }
  add_crc(&mut response);
  Some(response)
}

/// 写单个寄存器 (0x06)
fn write_single_register(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  if frame.len() < 6 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  let reg_addr = u16::from_be_bytes([frame[2], frame[3]]);
  let reg_value = u16::from_be_bytes([frame[4], frame[5]]);

  if reg_addr >= MAX_HOLDING_REGISTERS {
    return Some(create_exception_response(frame[0], frame[1], 0x02));
  }

  unsafe {
    MODBUS_DATA.set_holding_register(reg_addr, reg_value);
  }

  // 回显请求（成功响应）
  let mut response = Vec::new();
  for &byte in frame.iter().take(frame.len() - 2) {
    let _ = response.push(byte);
  }
  add_crc(&mut response);
  Some(response)
}

/// 写多个线圈 (0x0F)
fn write_multiple_coils(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  if frame.len() < 7 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  let start_addr = u16::from_be_bytes([frame[2], frame[3]]);
  let quantity = u16::from_be_bytes([frame[4], frame[5]]);
  let byte_count = frame[6] as usize;

  if quantity == 0 || quantity > 1968 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  if start_addr.saturating_add(quantity) > MAX_COILS {
    return Some(create_exception_response(frame[0], frame[1], 0x02));
  }

  let expected_bytes = ((quantity + 7) / 8) as usize;
  if byte_count != expected_bytes || frame.len() < 7 + byte_count + 2 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  // 写入线圈数据
  unsafe {
    for i in 0..quantity {
      let byte_idx = (i / 8) as usize;
      let bit_idx = i % 8;
      let coil_addr = start_addr + i;

      if byte_idx < byte_count {
        let byte_value = frame[7 + byte_idx];
        let coil_state = (byte_value & (1 << bit_idx)) != 0;
        MODBUS_DATA.set_coil(coil_addr, coil_state);
      }
    }
  }

  // 构建响应
  let mut response = Vec::new();
  let _ = response.push(frame[0]); // 从站地址
  let _ = response.push(frame[1]); // 功能码
  let _ = response.push(frame[2]); // 起始地址高字节
  let _ = response.push(frame[3]); // 起始地址低字节
  let _ = response.push(frame[4]); // 数量高字节
  let _ = response.push(frame[5]); // 数量低字节

  add_crc(&mut response);
  Some(response)
}

/// 写多个寄存器 (0x10)
fn write_multiple_registers(frame: &[u8]) -> Option<Vec<u8, MAX_FRAME_SIZE>> {
  if frame.len() < 7 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  let start_addr = u16::from_be_bytes([frame[2], frame[3]]);
  let quantity = u16::from_be_bytes([frame[4], frame[5]]);
  let byte_count = frame[6] as usize;

  if quantity == 0 || quantity > 123 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  if start_addr.saturating_add(quantity) > MAX_HOLDING_REGISTERS {
    return Some(create_exception_response(frame[0], frame[1], 0x02));
  }

  let expected_bytes = (quantity * 2) as usize;
  if byte_count != expected_bytes || frame.len() < 7 + byte_count + 2 {
    return Some(create_exception_response(frame[0], frame[1], 0x03));
  }

  // 写入寄存器数据
  unsafe {
    for i in 0..quantity {
      let reg_addr = start_addr + i;
      let byte_offset = 7 + (i * 2) as usize;
      let reg_value = u16::from_be_bytes([frame[byte_offset], frame[byte_offset + 1]]);
      MODBUS_DATA.set_holding_register(reg_addr, reg_value);
    }
  }

  // 构建响应
  let mut response = Vec::new();
  let _ = response.push(frame[0]); // 从站地址
  let _ = response.push(frame[1]); // 功能码
  let _ = response.push(frame[2]); // 起始地址高字节
  let _ = response.push(frame[3]); // 起始地址低字节
  let _ = response.push(frame[4]); // 数量高字节
  let _ = response.push(frame[5]); // 数量低字节

  add_crc(&mut response);
  Some(response)
}

/// 创建异常响应
fn create_exception_response(
  slave_addr: u8,
  function_code: u8,
  exception_code: u8,
) -> Vec<u8, MAX_FRAME_SIZE> {
  let mut response = Vec::new();
  let _ = response.push(slave_addr);
  let _ = response.push(function_code | 0x80); // 设置异常标志
  let _ = response.push(exception_code);
  add_crc(&mut response);
  response
}

/// 发送响应
fn send_response(response: &[u8]) {
  free(|cs| {
    if let Some(ref mut tx) = G_SERIAL_TX.borrow(cs).borrow_mut().as_mut() {
      for &byte in response {
        nb::block!(tx.write(byte)).ok();
      }
    }
  });
}

/// 验证CRC
fn verify_crc(frame: &[u8]) -> bool {
  if frame.len() < 4 {
    return false;
  }

  let data = &frame[..frame.len() - 2];
  let received_crc = u16::from_le_bytes([frame[frame.len() - 2], frame[frame.len() - 1]]);
  let calculated_crc = calculate_crc(data);

  received_crc == calculated_crc
}

/// 添加CRC到帧
fn add_crc(frame: &mut Vec<u8, MAX_FRAME_SIZE>) {
  let crc = calculate_crc(frame);
  let crc_bytes = crc.to_le_bytes();
  let _ = frame.push(crc_bytes[0]);
  let _ = frame.push(crc_bytes[1]);
}

/// 计算CRC-16
fn calculate_crc(data: &[u8]) -> u16 {
  let mut crc = 0xFFFFu16;

  for &byte in data {
    crc ^= byte as u16;

    for _ in 0..8 {
      if crc & 1 != 0 {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  crc
}

/// Modbus数据存储
struct ModbusData {
  coils: [bool; MAX_COILS as usize],
  discrete_inputs: [bool; MAX_DISCRETE_INPUTS as usize],
  holding_registers: [u16; MAX_HOLDING_REGISTERS as usize],
  input_registers: [u16; MAX_INPUT_REGISTERS as usize],
}

impl ModbusData {
  const fn new() -> Self {
    Self {
      coils: [false; MAX_COILS as usize],
      discrete_inputs: [false; MAX_DISCRETE_INPUTS as usize],
      holding_registers: [0; MAX_HOLDING_REGISTERS as usize],
      input_registers: [0; MAX_INPUT_REGISTERS as usize],
    }
  }

  fn initialize(&mut self) {
    // 初始化一些测试数据
    for i in 0..10 {
      self.coils[i] = i % 2 == 0;
      self.discrete_inputs[i] = i % 3 == 0;
      self.holding_registers[i] = (i * 100) as u16;
      self.input_registers[i] = (i * 50) as u16;
    }
  }

  fn update_simulated_data(&mut self, counter: u32) {
    // 模拟传感器数据更新
    let base_value = (counter / 1000) as u16;

    // 更新输入寄存器（模拟传感器读数）
    self.input_registers[0] = base_value; // 温度
    self.input_registers[1] = base_value + 100; // 湿度
    self.input_registers[2] = base_value + 200; // 压力

    // 更新离散输入（模拟开关状态）
    self.discrete_inputs[0] = (counter / 5000) % 2 == 0; // 门开关
    self.discrete_inputs[1] = (counter / 3000) % 2 == 0; // 报警状态
  }

  fn get_coil(&self, addr: u16) -> bool {
    if (addr as usize) < self.coils.len() {
      self.coils[addr as usize]
    } else {
      false
    }
  }

  fn set_coil(&mut self, addr: u16, value: bool) {
    if (addr as usize) < self.coils.len() {
      self.coils[addr as usize] = value;
    }
  }

  fn get_discrete_input(&self, addr: u16) -> bool {
    if (addr as usize) < self.discrete_inputs.len() {
      self.discrete_inputs[addr as usize]
    } else {
      false
    }
  }

  fn get_holding_register(&self, addr: u16) -> u16 {
    if (addr as usize) < self.holding_registers.len() {
      self.holding_registers[addr as usize]
    } else {
      0
    }
  }

  fn set_holding_register(&mut self, addr: u16, value: u16) {
    if (addr as usize) < self.holding_registers.len() {
      self.holding_registers[addr as usize] = value;
    }
  }

  fn get_input_register(&self, addr: u16) -> u16 {
    if (addr as usize) < self.input_registers.len() {
      self.input_registers[addr as usize]
    } else {
      0
    }
  }
}

/// 串口接收中断
#[interrupt]
fn USART1() {
  free(|cs| {
    if let Some(ref mut rx) = G_SERIAL_RX.borrow(cs).borrow_mut().as_mut() {
      if let Ok(byte) = rx.read() {
        let current_time = get_system_time_ms();
        let last_time = LAST_CHAR_TIME.load(Ordering::Acquire);

        // 检查帧间隔
        if current_time.saturating_sub(last_time) > FRAME_TIMEOUT_MS {
          // 新帧开始
          RX_INDEX.store(0, Ordering::Release);
        }

        let index = RX_INDEX.load(Ordering::Acquire) as usize;
        if index < MAX_FRAME_SIZE {
          unsafe {
            RX_BUFFER[index] = byte;
          }
          RX_INDEX.store((index + 1) as u16, Ordering::Release);
        }

        LAST_CHAR_TIME.store(current_time, Ordering::Release);
      }
    }
  });
}

/// 定时器中断 - 检测帧结束
#[interrupt]
fn TIM2() {
  // 清除中断标志
  unsafe {
    (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit());
  }

  let current_time = get_system_time_ms();
  let last_time = LAST_CHAR_TIME.load(Ordering::Acquire);

  // 检查是否超过帧超时时间
  if current_time.saturating_sub(last_time) >= FRAME_TIMEOUT_MS
    && RX_INDEX.load(Ordering::Acquire) > 0
    && !FRAME_READY.load(Ordering::Acquire)
  {
    FRAME_READY.store(true, Ordering::Release);
  }
}

/// 获取系统时间（毫秒）
fn get_system_time_ms() -> u32 {
  // 这里应该实现实际的系统时间获取
  // 简化实现，使用静态计数器
  static COUNTER: AtomicU32 = AtomicU32::new(0);
  COUNTER.fetch_add(1, Ordering::Relaxed)
}
