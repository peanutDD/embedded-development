#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{prelude::*, stm32};

// 数学运算模块
pub mod math {
  use heapless::FnvIndexMap;

  /// 素数缓存结构体，用于存储已计算的素数结果
  pub struct PrimeCache {
    // 使用FnvIndexMap存储数字和对应的素数判断结果
    cache: FnvIndexMap<u32, bool, 32>,
  }

  impl PrimeCache {
    /// 创建新的素数缓存
    pub fn new() -> Self {
      Self {
        cache: FnvIndexMap::new(),
      }
    }

    /// 检查数字是否为素数，优先使用缓存
    pub fn is_prime(&mut self, n: u32) -> bool {
      // 先检查缓存中是否已有结果
      if let Some(&result) = self.cache.get(&n) {
        return result;
      }

      // 计算结果
      let result = is_prime_internal(n);

      // 尝试存入缓存（如果缓存已满则忽略）
      let _ = self.cache.insert(n, result);

      result
    }

    /// 获取缓存大小
    pub fn cache_size(&self) -> usize {
      self.cache.len()
    }

    /// 清空缓存
    pub fn clear(&mut self) {
      self.cache.clear();
    }
  }

  /// 计算两个数的最大公约数
  pub fn gcd(mut a: u32, mut b: u32) -> u32 {
    while b != 0 {
      let temp = b;
      b = a % b;
      a = temp;
    }
    a
  }

  /// 计算阶乘
  pub fn factorial(n: u32) -> Option<u32> {
    if n > 12 {
      return None; // 防止溢出
    }

    let mut result = 1;
    for i in 1..=n {
      result *= i;
    }
    Some(result)
  }

  /// 判断是否为质数（内部实现）
  fn is_prime_internal(n: u32) -> bool {
    if n < 2 {
      return false;
    }
    if n == 2 {
      return true;
    }
    if n % 2 == 0 {
      return false;
    }

    let sqrt_n = (n as f32).sqrt() as u32;
    for i in (3..=sqrt_n).step_by(2) {
      if n % i == 0 {
        return false;
      }
    }
    true
  }

  /// 判断是否为质数（公共API，保持兼容性）
  pub fn is_prime(n: u32) -> bool {
    is_prime_internal(n)
  }

  /// 计算斐波那契数列
  pub fn fibonacci(n: u32) -> u32 {
    match n {
      0 => 0,
      1 => 1,
      _ => {
        let mut a = 0;
        let mut b = 1;
        for _ in 2..=n {
          let temp = a + b;
          a = b;
          b = temp;
        }
        b
      }
    }
  }
}

// 数据结构模块
pub mod data_structures {
  use heapless::Vec;

  /// 固定大小的栈
  pub struct Stack<T, const N: usize> {
    data: Vec<T, N>,
  }

  impl<T, const N: usize> Stack<T, N> {
    pub fn new() -> Self {
      Self { data: Vec::new() }
    }

    pub fn push(&mut self, item: T) -> Result<(), T> {
      self.data.push(item)
    }

    pub fn pop(&mut self) -> Option<T> {
      self.data.pop()
    }

    pub fn peek(&self) -> Option<&T> {
      self.data.last()
    }

    pub fn is_empty(&self) -> bool {
      self.data.is_empty()
    }

    pub fn is_full(&self) -> bool {
      self.data.is_full()
    }

    pub fn len(&self) -> usize {
      self.data.len()
    }

    pub fn capacity(&self) -> usize {
      N
    }
  }

  /// 环形缓冲区
  pub struct RingBuffer<T, const N: usize> {
    data: [Option<T>; N],
    head: usize,
    tail: usize,
    full: bool,
  }

  impl<T: Copy + Default, const N: usize> RingBuffer<T, N> {
    pub fn new() -> Self {
      Self {
        data: [None; N],
        head: 0,
        tail: 0,
        full: false,
      }
    }

    pub fn push(&mut self, item: T) -> Option<T> {
      let old_value = if self.full {
        self.data[self.tail].take()
      } else {
        None
      };

      self.data[self.head] = Some(item);
      self.head = (self.head + 1) % N;

      if self.full {
        self.tail = (self.tail + 1) % N;
      }

      if self.head == self.tail {
        self.full = true;
      }

      old_value
    }

    pub fn pop(&mut self) -> Option<T> {
      if self.is_empty() {
        return None;
      }

      let item = self.data[self.tail].take();
      self.tail = (self.tail + 1) % N;
      self.full = false;

      item
    }

    pub fn is_empty(&self) -> bool {
      !self.full && self.head == self.tail
    }

    pub fn is_full(&self) -> bool {
      self.full
    }

    pub fn len(&self) -> usize {
      if self.full {
        N
      } else if self.head >= self.tail {
        self.head - self.tail
      } else {
        N - self.tail + self.head
      }
    }

    pub fn capacity(&self) -> usize {
      N
    }
  }
}

// 状态机模块
pub mod state_machine {
  #[derive(Debug, Clone, Copy, PartialEq)]
  pub enum LedState {
    Off,
    On,
    Blinking,
    Fading,
  }

  #[derive(Debug, Clone, Copy, PartialEq)]
  pub enum LedEvent {
    TurnOn,
    TurnOff,
    StartBlinking,
    StartFading,
    Timeout,
  }

  pub struct LedStateMachine {
    state: LedState,
    blink_count: u32,
    fade_level: u8,
  }

  impl LedStateMachine {
    pub fn new() -> Self {
      Self {
        state: LedState::Off,
        blink_count: 0,
        fade_level: 0,
      }
    }

    pub fn current_state(&self) -> LedState {
      self.state
    }

    pub fn handle_event(&mut self, event: LedEvent) -> Result<(), &'static str> {
      let new_state = match (self.state, event) {
        (LedState::Off, LedEvent::TurnOn) => LedState::On,
        (LedState::On, LedEvent::TurnOff) => LedState::Off,
        (LedState::Off, LedEvent::StartBlinking) => {
          self.blink_count = 0;
          LedState::Blinking
        }
        (LedState::Blinking, LedEvent::TurnOff) => LedState::Off,
        (LedState::Blinking, LedEvent::Timeout) => {
          self.blink_count += 1;
          if self.blink_count >= 10 {
            LedState::Off
          } else {
            LedState::Blinking
          }
        }
        (LedState::Off, LedEvent::StartFading) => {
          self.fade_level = 0;
          LedState::Fading
        }
        (LedState::Fading, LedEvent::TurnOff) => LedState::Off,
        (LedState::Fading, LedEvent::Timeout) => {
          self.fade_level = (self.fade_level + 10).min(255);
          if self.fade_level >= 255 {
            LedState::On
          } else {
            LedState::Fading
          }
        }
        _ => return Err("Invalid state transition"),
      };

      self.state = new_state;
      Ok(())
    }

    pub fn get_blink_count(&self) -> u32 {
      self.blink_count
    }

    pub fn get_fade_level(&self) -> u8 {
      self.fade_level
    }
  }
}

// 传感器数据处理模块
pub mod sensor {
  use heapless::Vec;

  pub struct TemperatureSensor {
    readings: Vec<f32, 10>,
    calibration_offset: f32,
  }

  impl TemperatureSensor {
    pub fn new(calibration_offset: f32) -> Self {
      Self {
        readings: Vec::new(),
        calibration_offset,
      }
    }

    pub fn add_reading(&mut self, raw_value: u16) -> Result<f32, &'static str> {
      // 模拟ADC到温度的转换
      let voltage = (raw_value as f32 / 4095.0) * 3.3;
      let temperature = (voltage - 0.5) * 100.0 + self.calibration_offset;

      if self.readings.is_full() {
        self.readings.remove(0);
      }

      self
        .readings
        .push(temperature)
        .map_err(|_| "Failed to add reading")?;

      Ok(temperature)
    }

    pub fn get_average(&self) -> Option<f32> {
      if self.readings.is_empty() {
        return None;
      }

      let sum: f32 = self.readings.iter().sum();
      Some(sum / self.readings.len() as f32)
    }

    pub fn get_min_max(&self) -> Option<(f32, f32)> {
      if self.readings.is_empty() {
        return None;
      }

      let mut min = self.readings[0];
      let mut max = self.readings[0];

      for &reading in self.readings.iter().skip(1) {
        if reading < min {
          min = reading;
        }
        if reading > max {
          max = reading;
        }
      }

      Some((min, max))
    }

    pub fn is_temperature_stable(&self, threshold: f32) -> bool {
      if self.readings.len() < 3 {
        return false;
      }

      let recent_readings = &self.readings[self.readings.len() - 3..];
      let avg = recent_readings.iter().sum::<f32>() / 3.0;

      recent_readings
        .iter()
        .all(|&reading| (reading - avg).abs() < threshold)
    }

    pub fn clear_readings(&mut self) {
      self.readings.clear();
    }

    pub fn reading_count(&self) -> usize {
      self.readings.len()
    }
  }
}

// 通信协议模块
pub mod protocol {
  use heapless::Vec;

  #[derive(Debug, Clone, Copy, PartialEq)]
  pub enum MessageType {
    Data = 0x01,
    Ack = 0x02,
    Nack = 0x03,
    Ping = 0x04,
    Pong = 0x05,
  }

  #[derive(Debug, Clone, PartialEq)]
  pub struct Message {
    pub msg_type: MessageType,
    pub sequence: u8,
    pub payload: Vec<u8, 32>,
    pub checksum: u8,
  }

  impl Message {
    pub fn new(msg_type: MessageType, sequence: u8, payload: &[u8]) -> Result<Self, &'static str> {
      if payload.len() > 32 {
        return Err("Payload too large");
      }

      let mut msg_payload = Vec::new();
      for &byte in payload {
        msg_payload
          .push(byte)
          .map_err(|_| "Failed to add payload")?;
      }

      let checksum = Self::calculate_checksum(msg_type, sequence, &msg_payload);

      Ok(Self {
        msg_type,
        sequence,
        payload: msg_payload,
        checksum,
      })
    }

    fn calculate_checksum(msg_type: MessageType, sequence: u8, payload: &[u8]) -> u8 {
      let mut checksum = msg_type as u8;
      checksum = checksum.wrapping_add(sequence);
      for &byte in payload {
        checksum = checksum.wrapping_add(byte);
      }
      !checksum
    }

    pub fn serialize(&self) -> Vec<u8, 64> {
      let mut buffer = Vec::new();

      // 添加帧头
      buffer.push(0xAA).unwrap();
      buffer.push(0x55).unwrap();

      // 添加消息类型
      buffer.push(self.msg_type as u8).unwrap();

      // 添加序列号
      buffer.push(self.sequence).unwrap();

      // 添加载荷长度
      buffer.push(self.payload.len() as u8).unwrap();

      // 添加载荷
      for &byte in &self.payload {
        buffer.push(byte).unwrap();
      }

      // 添加校验和
      buffer.push(self.checksum).unwrap();

      buffer
    }

    pub fn deserialize(data: &[u8]) -> Result<Self, &'static str> {
      if data.len() < 6 {
        return Err("Data too short");
      }

      // 检查帧头
      if data[0] != 0xAA || data[1] != 0x55 {
        return Err("Invalid frame header");
      }

      let msg_type = match data[2] {
        0x01 => MessageType::Data,
        0x02 => MessageType::Ack,
        0x03 => MessageType::Nack,
        0x04 => MessageType::Ping,
        0x05 => MessageType::Pong,
        _ => return Err("Invalid message type"),
      };

      let sequence = data[3];
      let payload_len = data[4] as usize;

      if data.len() < 6 + payload_len {
        return Err("Incomplete message");
      }

      let payload_data = &data[5..5 + payload_len];
      let received_checksum = data[5 + payload_len];

      let mut payload = Vec::new();
      for &byte in payload_data {
        payload.push(byte).map_err(|_| "Payload too large")?;
      }

      let calculated_checksum = Self::calculate_checksum(msg_type, sequence, &payload);

      if received_checksum != calculated_checksum {
        return Err("Checksum mismatch");
      }

      Ok(Self {
        msg_type,
        sequence,
        payload,
        checksum: received_checksum,
      })
    }

    pub fn is_valid(&self) -> bool {
      let calculated_checksum =
        Self::calculate_checksum(self.msg_type, self.sequence, &self.payload);
      calculated_checksum == self.checksum
    }
  }

  pub struct ProtocolHandler {
    next_sequence: u8,
    pending_acks: Vec<u8, 16>,
  }

  impl ProtocolHandler {
    pub fn new() -> Self {
      Self {
        next_sequence: 0,
        pending_acks: Vec::new(),
      }
    }

    pub fn create_data_message(&mut self, payload: &[u8]) -> Result<Message, &'static str> {
      let msg = Message::new(MessageType::Data, self.next_sequence, payload)?;
      self
        .pending_acks
        .push(self.next_sequence)
        .map_err(|_| "Too many pending ACKs")?;
      self.next_sequence = self.next_sequence.wrapping_add(1);
      Ok(msg)
    }

    pub fn create_ack_message(&self, sequence: u8) -> Result<Message, &'static str> {
      Message::new(MessageType::Ack, sequence, &[])
    }

    pub fn handle_ack(&mut self, sequence: u8) -> bool {
      if let Some(pos) = self.pending_acks.iter().position(|&s| s == sequence) {
        self.pending_acks.remove(pos);
        true
      } else {
        false
      }
    }

    pub fn get_pending_acks(&self) -> &[u8] {
      &self.pending_acks
    }

    pub fn has_pending_acks(&self) -> bool {
      !self.pending_acks.is_empty()
    }
  }
}

#[entry]
fn main() -> ! {
  // 初始化外设
  let dp = stm32::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.freeze();

  let gpioa = dp.GPIOA.split();
  let mut led = gpioa.pa5.into_push_pull_output();

  // 运行一些基本测试
  run_basic_tests();

  // 主循环
  loop {
    led.set_high();
    cortex_m::asm::delay(1_000_000);
    led.set_low();
    cortex_m::asm::delay(1_000_000);
  }
}

fn run_basic_tests() {
  // 测试数学函数
  assert_eq!(math::gcd(48, 18), 6);
  assert_eq!(math::factorial(5), Some(120));
  assert!(math::is_prime(17));
  assert_eq!(math::fibonacci(10), 55);

  // 测试素数缓存
  let mut prime_cache = math::PrimeCache::new();
  assert!(prime_cache.is_prime(17));
  assert!(prime_cache.is_prime(23));
  assert!(!prime_cache.is_prime(25));
  assert_eq!(prime_cache.cache_size(), 3);

  // 测试数据结构
  let mut stack = data_structures::Stack::<i32, 5>::new();
  assert!(stack.is_empty());
  stack.push(1).unwrap();
  stack.push(2).unwrap();
  assert_eq!(stack.pop(), Some(2));
  assert_eq!(stack.len(), 1);

  // 测试状态机
  let mut led_sm = state_machine::LedStateMachine::new();
  assert_eq!(led_sm.current_state(), state_machine::LedState::Off);
  led_sm
    .handle_event(state_machine::LedEvent::TurnOn)
    .unwrap();
  assert_eq!(led_sm.current_state(), state_machine::LedState::On);
}

// 单元测试
#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_gcd() {
    assert_eq!(math::gcd(48, 18), 6);
    assert_eq!(math::gcd(17, 13), 1);
    assert_eq!(math::gcd(100, 25), 25);
    assert_eq!(math::gcd(0, 5), 5);
    assert_eq!(math::gcd(5, 0), 5);
  }

  #[test]
  fn test_factorial() {
    assert_eq!(math::factorial(0), Some(1));
    assert_eq!(math::factorial(1), Some(1));
    assert_eq!(math::factorial(5), Some(120));
    assert_eq!(math::factorial(10), Some(3628800));
    assert_eq!(math::factorial(13), None); // 溢出
  }

  #[test]
  fn test_is_prime() {
    assert!(!math::is_prime(0));
    assert!(!math::is_prime(1));
    assert!(math::is_prime(2));
    assert!(math::is_prime(3));
    assert!(!math::is_prime(4));
    assert!(math::is_prime(17));
    assert!(!math::is_prime(25));
    assert!(math::is_prime(97));
  }

  #[test]
  fn test_prime_cache() {
    let mut cache = math::PrimeCache::new();

    // 测试基本功能
    assert!(cache.is_prime(17));
    assert!(!cache.is_prime(25));

    // 测试缓存大小
    assert_eq!(cache.cache_size(), 2);

    // 测试缓存命中（重复查询不应增加缓存大小）
    assert!(cache.is_prime(17));
    assert_eq!(cache.cache_size(), 2);

    // 测试清空缓存
    cache.clear();
    assert_eq!(cache.cache_size(), 0);

    // 测试缓存一致性
    assert!(cache.is_prime(7));
    assert!(cache.is_prime(11));
    assert!(cache.is_prime(13));
    assert!(!cache.is_prime(15));
    assert_eq!(cache.cache_size(), 4);
  }

  #[test]
  fn test_fibonacci() {
    assert_eq!(math::fibonacci(0), 0);
    assert_eq!(math::fibonacci(1), 1);
    assert_eq!(math::fibonacci(2), 1);
    assert_eq!(math::fibonacci(10), 55);
    assert_eq!(math::fibonacci(15), 610);
  }

  #[test]
  fn test_stack() {
    let mut stack = data_structures::Stack::<i32, 3>::new();

    assert!(stack.is_empty());
    assert!(!stack.is_full());
    assert_eq!(stack.len(), 0);
    assert_eq!(stack.capacity(), 3);

    stack.push(1).unwrap();
    stack.push(2).unwrap();
    stack.push(3).unwrap();

    assert!(stack.is_full());
    assert_eq!(stack.len(), 3);
    assert!(stack.push(4).is_err());

    assert_eq!(stack.peek(), Some(&3));
    assert_eq!(stack.pop(), Some(3));
    assert_eq!(stack.pop(), Some(2));
    assert_eq!(stack.pop(), Some(1));
    assert_eq!(stack.pop(), None);

    assert!(stack.is_empty());
  }

  #[test]
  fn test_ring_buffer() {
    let mut buffer = data_structures::RingBuffer::<i32, 3>::new();

    assert!(buffer.is_empty());
    assert!(!buffer.is_full());
    assert_eq!(buffer.len(), 0);

    buffer.push(1);
    buffer.push(2);
    buffer.push(3);

    assert!(buffer.is_full());
    assert_eq!(buffer.len(), 3);

    // 覆盖最旧的元素
    let old = buffer.push(4);
    assert_eq!(old, Some(1));

    assert_eq!(buffer.pop(), Some(2));
    assert_eq!(buffer.pop(), Some(3));
    assert_eq!(buffer.pop(), Some(4));
    assert_eq!(buffer.pop(), None);
  }

  #[test]
  fn test_led_state_machine() {
    let mut sm = state_machine::LedStateMachine::new();

    assert_eq!(sm.current_state(), state_machine::LedState::Off);

    sm.handle_event(state_machine::LedEvent::TurnOn).unwrap();
    assert_eq!(sm.current_state(), state_machine::LedState::On);

    sm.handle_event(state_machine::LedEvent::TurnOff).unwrap();
    assert_eq!(sm.current_state(), state_machine::LedState::Off);

    sm.handle_event(state_machine::LedEvent::StartBlinking)
      .unwrap();
    assert_eq!(sm.current_state(), state_machine::LedState::Blinking);

    // 模拟10次超时事件
    for _ in 0..10 {
      sm.handle_event(state_machine::LedEvent::Timeout).unwrap();
    }
    assert_eq!(sm.current_state(), state_machine::LedState::Off);
    assert_eq!(sm.get_blink_count(), 10);
  }

  #[test]
  fn test_temperature_sensor() {
    let mut sensor = sensor::TemperatureSensor::new(0.0);

    // 添加一些读数
    sensor.add_reading(2048).unwrap(); // ~25°C
    sensor.add_reading(2150).unwrap(); // ~27.5°C
    sensor.add_reading(2000).unwrap(); // ~23.8°C

    assert_eq!(sensor.reading_count(), 3);

    let avg = sensor.get_average().unwrap();
    assert!((avg - 25.4).abs() < 1.0);

    let (min, max) = sensor.get_min_max().unwrap();
    assert!(min < max);

    assert!(!sensor.is_temperature_stable(1.0));

    sensor.clear_readings();
    assert_eq!(sensor.reading_count(), 0);
    assert!(sensor.get_average().is_none());
  }

  #[test]
  fn test_message_protocol() {
    let payload = b"Hello, World!";
    let msg = protocol::Message::new(protocol::MessageType::Data, 42, payload).unwrap();

    assert!(msg.is_valid());
    assert_eq!(msg.msg_type, protocol::MessageType::Data);
    assert_eq!(msg.sequence, 42);
    assert_eq!(msg.payload.as_slice(), payload);

    let serialized = msg.serialize();
    let deserialized = protocol::Message::deserialize(&serialized).unwrap();

    assert_eq!(msg, deserialized);
    assert!(deserialized.is_valid());
  }

  #[test]
  fn test_protocol_handler() {
    let mut handler = protocol::ProtocolHandler::new();

    let msg1 = handler.create_data_message(b"test1").unwrap();
    let msg2 = handler.create_data_message(b"test2").unwrap();

    assert_eq!(msg1.sequence, 0);
    assert_eq!(msg2.sequence, 1);
    assert_eq!(handler.get_pending_acks().len(), 2);

    assert!(handler.handle_ack(0));
    assert_eq!(handler.get_pending_acks().len(), 1);

    assert!(!handler.handle_ack(5)); // 不存在的序列号
    assert_eq!(handler.get_pending_acks().len(), 1);

    assert!(handler.handle_ack(1));
    assert!(!handler.has_pending_acks());
  }
}
