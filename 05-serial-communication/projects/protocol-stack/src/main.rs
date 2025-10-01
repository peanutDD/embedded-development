//! 分层协议栈实现 - 完整的通信协议栈
//!
//! 本项目实现了一个完整的分层协议栈，包括：
//! - 物理层 (Physical Layer) - 电气信号处理
//! - 数据链路层 (Data Link Layer) - 帧同步和错误检测
//! - 网络层 (Network Layer) - 路由和寻址
//! - 传输层 (Transport Layer) - 可靠传输和流控制
//! - 应用层 (Application Layer) - 应用协议和数据处理

#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::asm;
use cortex_m_rt::entry;
use heapless::{FnvIndexMap, String, Vec};
use nb::block;
use panic_halt as _;

#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal;
#[cfg(feature = "stm32")]
use stm32f4xx_hal as hal;

/// 协议栈层次定义
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LayerType {
  Physical = 1,
  DataLink = 2,
  Network = 3,
  Transport = 4,
  Application = 5,
}

/// 协议数据单元 (PDU)
#[derive(Debug, Clone)]
pub struct ProtocolDataUnit {
  /// 数据内容
  pub data: Vec<u8, 256>,
  /// 源层
  pub source_layer: LayerType,
  /// 目标层
  pub target_layer: LayerType,
  /// 协议类型
  pub protocol_type: u16,
  /// 时间戳
  pub timestamp: u32,
  /// 优先级
  pub priority: Priority,
}

/// 优先级定义
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
  Low = 0,
  Normal = 1,
  High = 2,
  Critical = 3,
}

/// 物理层实现
pub struct PhysicalLayer {
  /// 信号调制类型
  modulation: ModulationType,
  /// 传输速率
  baud_rate: u32,
  /// 信号功率
  signal_power: f32,
  /// 噪声水平
  noise_level: f32,
  /// 信号质量指标
  signal_quality: SignalQuality,
}

/// 调制类型
#[derive(Debug, Clone, Copy)]
pub enum ModulationType {
  /// 非归零码 (NRZ)
  NRZ,
  /// 曼彻斯特编码
  Manchester,
  /// 差分曼彻斯特编码
  DifferentialManchester,
  /// 4B/5B编码
  FourBFiveB,
}

/// 信号质量指标
#[derive(Debug, Clone)]
pub struct SignalQuality {
  /// 信噪比 (dB)
  pub snr: f32,
  /// 误码率
  pub bit_error_rate: f32,
  /// 眼图开度
  pub eye_opening: f32,
  /// 抖动 (ps)
  pub jitter: f32,
}

impl PhysicalLayer {
  pub fn new(modulation: ModulationType, baud_rate: u32) -> Self {
    Self {
      modulation,
      baud_rate,
      signal_power: 3.3, // 3.3V logic level
      noise_level: 0.1,  // 100mV noise
      signal_quality: SignalQuality {
        snr: 20.0, // 20dB
        bit_error_rate: 1e-6,
        eye_opening: 0.8,
        jitter: 100.0, // 100ps
      },
    }
  }

  /// 编码数据
  pub fn encode(&self, data: &[u8]) -> Result<Vec<u8, 512>, PhysicalLayerError> {
    let mut encoded = Vec::new();

    match self.modulation {
      ModulationType::NRZ => {
        // NRZ编码：直接传输
        for &byte in data {
          encoded
            .push(byte)
            .map_err(|_| PhysicalLayerError::BufferOverflow)?;
        }
      }
      ModulationType::Manchester => {
        // 曼彻斯特编码：每个位用两个符号表示
        for &byte in data {
          for bit_pos in 0..8 {
            let bit = (byte >> bit_pos) & 1;
            if bit == 1 {
              // 1 -> 01
              encoded
                .push(0)
                .map_err(|_| PhysicalLayerError::BufferOverflow)?;
              encoded
                .push(1)
                .map_err(|_| PhysicalLayerError::BufferOverflow)?;
            } else {
              // 0 -> 10
              encoded
                .push(1)
                .map_err(|_| PhysicalLayerError::BufferOverflow)?;
              encoded
                .push(0)
                .map_err(|_| PhysicalLayerError::BufferOverflow)?;
            }
          }
        }
      }
      ModulationType::DifferentialManchester => {
        // 差分曼彻斯特编码
        let mut last_level = 0u8;
        for &byte in data {
          for bit_pos in 0..8 {
            let bit = (byte >> bit_pos) & 1;
            if bit == 1 {
              // 1: 电平不变，然后跳变
              encoded
                .push(last_level)
                .map_err(|_| PhysicalLayerError::BufferOverflow)?;
              last_level = 1 - last_level;
              encoded
                .push(last_level)
                .map_err(|_| PhysicalLayerError::BufferOverflow)?;
            } else {
              // 0: 电平跳变，然后再跳变
              last_level = 1 - last_level;
              encoded
                .push(last_level)
                .map_err(|_| PhysicalLayerError::BufferOverflow)?;
              last_level = 1 - last_level;
              encoded
                .push(last_level)
                .map_err(|_| PhysicalLayerError::BufferOverflow)?;
            }
          }
        }
      }
      ModulationType::FourBFiveB => {
        // 4B/5B编码：每4位数据编码为5位符号
        for &byte in data {
          let high_nibble = (byte >> 4) & 0x0F;
          let low_nibble = byte & 0x0F;

          let high_encoded = self.encode_4b5b(high_nibble);
          let low_encoded = self.encode_4b5b(low_nibble);

          encoded
            .push(high_encoded)
            .map_err(|_| PhysicalLayerError::BufferOverflow)?;
          encoded
            .push(low_encoded)
            .map_err(|_| PhysicalLayerError::BufferOverflow)?;
        }
      }
    }

    Ok(encoded)
  }

  /// 解码数据
  pub fn decode(&self, encoded_data: &[u8]) -> Result<Vec<u8, 256>, PhysicalLayerError> {
    let mut decoded = Vec::new();

    match self.modulation {
      ModulationType::NRZ => {
        // NRZ解码：直接接收
        for &byte in encoded_data {
          decoded
            .push(byte)
            .map_err(|_| PhysicalLayerError::BufferOverflow)?;
        }
      }
      ModulationType::Manchester => {
        // 曼彻斯特解码
        if encoded_data.len() % 16 != 0 {
          return Err(PhysicalLayerError::InvalidEncoding);
        }

        for chunk in encoded_data.chunks(16) {
          let mut byte = 0u8;
          for bit_pos in 0..8 {
            let symbol1 = chunk[bit_pos * 2];
            let symbol2 = chunk[bit_pos * 2 + 1];

            let bit = if symbol1 == 0 && symbol2 == 1 {
              1
            } else if symbol1 == 1 && symbol2 == 0 {
              0
            } else {
              return Err(PhysicalLayerError::InvalidEncoding);
            };

            byte |= bit << bit_pos;
          }
          decoded
            .push(byte)
            .map_err(|_| PhysicalLayerError::BufferOverflow)?;
        }
      }
      _ => {
        // 其他编码方式的解码实现
        return Err(PhysicalLayerError::UnsupportedModulation);
      }
    }

    Ok(decoded)
  }

  /// 4B/5B编码表
  fn encode_4b5b(&self, nibble: u8) -> u8 {
    match nibble {
      0x0 => 0b11110,
      0x1 => 0b01001,
      0x2 => 0b10100,
      0x3 => 0b10101,
      0x4 => 0b01010,
      0x5 => 0b01011,
      0x6 => 0b01110,
      0x7 => 0b01111,
      0x8 => 0b10010,
      0x9 => 0b10011,
      0xA => 0b10110,
      0xB => 0b10111,
      0xC => 0b11010,
      0xD => 0b11011,
      0xE => 0b11100,
      0xF => 0b11101,
      _ => 0b00000, // 无效符号
    }
  }

  /// 测量信号质量
  pub fn measure_signal_quality(&mut self, received_data: &[u8]) -> SignalQuality {
    // 简化的信号质量测量
    let mut error_count = 0;
    let total_bits = received_data.len() * 8;

    // 模拟误码检测
    for &byte in received_data {
      if byte == 0xFF || byte == 0x00 {
        error_count += 1; // 简化的错误检测
      }
    }

    let ber = if total_bits > 0 {
      error_count as f32 / total_bits as f32
    } else {
      0.0
    };

    self.signal_quality.bit_error_rate = ber;
    self.signal_quality.snr = if ber > 0.0 {
      -10.0 * ber.log10()
    } else {
      40.0 // 高信噪比
    };

    self.signal_quality.clone()
  }
}

/// 物理层错误类型
#[derive(Debug)]
pub enum PhysicalLayerError {
  BufferOverflow,
  InvalidEncoding,
  UnsupportedModulation,
  SignalQualityTooLow,
}

/// 数据链路层实现
pub struct DataLinkLayer {
  /// 帧同步模式
  frame_sync: FrameSyncMode,
  /// 错误检测方法
  error_detection: ErrorDetectionMethod,
  /// 流控制
  flow_control: FlowControlMethod,
  /// 帧统计
  frame_stats: FrameStatistics,
}

/// 帧同步模式
#[derive(Debug, Clone, Copy)]
pub enum FrameSyncMode {
  /// 字符同步
  CharacterSync,
  /// 位同步
  BitSync,
  /// 标志同步
  FlagSync,
}

/// 错误检测方法
#[derive(Debug, Clone, Copy)]
pub enum ErrorDetectionMethod {
  /// 奇偶校验
  Parity,
  /// 校验和
  Checksum,
  /// CRC-16
  CRC16,
  /// CRC-32
  CRC32,
}

/// 流控制方法
#[derive(Debug, Clone, Copy)]
pub enum FlowControlMethod {
  /// 无流控制
  None,
  /// XON/XOFF
  XonXoff,
  /// RTS/CTS
  RtsCts,
  /// 停等协议
  StopAndWait,
  /// 滑动窗口
  SlidingWindow,
}

/// 帧统计信息
#[derive(Debug, Clone)]
pub struct FrameStatistics {
  pub frames_sent: u32,
  pub frames_received: u32,
  pub frames_with_errors: u32,
  pub retransmissions: u32,
  pub throughput: f32, // frames per second
}

/// 数据帧结构
#[derive(Debug, Clone)]
pub struct DataFrame {
  /// 帧头
  pub header: FrameHeader,
  /// 数据载荷
  pub payload: Vec<u8, 128>,
  /// 帧尾
  pub trailer: FrameTrailer,
}

/// 帧头
#[derive(Debug, Clone)]
pub struct FrameHeader {
  /// 同步字段
  pub sync: u16,
  /// 帧类型
  pub frame_type: FrameType,
  /// 序列号
  pub sequence_number: u8,
  /// 数据长度
  pub length: u8,
  /// 源地址
  pub source_address: u8,
  /// 目标地址
  pub destination_address: u8,
}

/// 帧类型
#[derive(Debug, Clone, Copy)]
pub enum FrameType {
  Data = 0x01,
  Ack = 0x02,
  Nack = 0x03,
  Control = 0x04,
}

/// 帧尾
#[derive(Debug, Clone)]
pub struct FrameTrailer {
  /// 错误检测码
  pub error_check: u32,
  /// 结束标志
  pub end_flag: u16,
}

impl DataLinkLayer {
  pub fn new(
    frame_sync: FrameSyncMode,
    error_detection: ErrorDetectionMethod,
    flow_control: FlowControlMethod,
  ) -> Self {
    Self {
      frame_sync,
      error_detection,
      flow_control,
      frame_stats: FrameStatistics {
        frames_sent: 0,
        frames_received: 0,
        frames_with_errors: 0,
        retransmissions: 0,
        throughput: 0.0,
      },
    }
  }

  /// 封装数据为帧
  pub fn encapsulate(&mut self, data: &[u8], dest_addr: u8) -> Result<DataFrame, DataLinkError> {
    if data.len() > 128 {
      return Err(DataLinkError::PayloadTooLarge);
    }

    let mut payload = Vec::new();
    for &byte in data {
      payload
        .push(byte)
        .map_err(|_| DataLinkError::BufferOverflow)?;
    }

    let header = FrameHeader {
      sync: 0xAA55, // 同步字段
      frame_type: FrameType::Data,
      sequence_number: (self.frame_stats.frames_sent % 256) as u8,
      length: data.len() as u8,
      source_address: 0x01, // 假设本地地址为0x01
      destination_address: dest_addr,
    };

    // 计算错误检测码
    let error_check = self.calculate_error_check(&header, &payload)?;

    let trailer = FrameTrailer {
      error_check,
      end_flag: 0x55AA,
    };

    self.frame_stats.frames_sent += 1;

    Ok(DataFrame {
      header,
      payload,
      trailer,
    })
  }

  /// 解封装帧数据
  pub fn decapsulate(&mut self, frame: &DataFrame) -> Result<Vec<u8, 128>, DataLinkError> {
    // 验证同步字段
    if frame.header.sync != 0xAA55 {
      return Err(DataLinkError::SyncError);
    }

    // 验证结束标志
    if frame.trailer.end_flag != 0x55AA {
      return Err(DataLinkError::FramingError);
    }

    // 验证长度
    if frame.payload.len() != frame.header.length as usize {
      return Err(DataLinkError::LengthError);
    }

    // 验证错误检测码
    let calculated_check = self.calculate_error_check(&frame.header, &frame.payload)?;
    if calculated_check != frame.trailer.error_check {
      self.frame_stats.frames_with_errors += 1;
      return Err(DataLinkError::ChecksumError);
    }

    self.frame_stats.frames_received += 1;
    Ok(frame.payload.clone())
  }

  /// 计算错误检测码
  fn calculate_error_check(
    &self,
    header: &FrameHeader,
    payload: &[u8],
  ) -> Result<u32, DataLinkError> {
    match self.error_detection {
      ErrorDetectionMethod::Checksum => {
        let mut sum = 0u32;
        sum += header.sync as u32;
        sum += header.frame_type as u32;
        sum += header.sequence_number as u32;
        sum += header.length as u32;
        sum += header.source_address as u32;
        sum += header.destination_address as u32;

        for &byte in payload {
          sum += byte as u32;
        }

        Ok(sum & 0xFFFF)
      }
      ErrorDetectionMethod::CRC16 => {
        // 简化的CRC-16实现
        let mut crc = 0xFFFFu16;
        let polynomial = 0x1021u16; // CRC-16-CCITT

        // 处理头部
        let header_bytes = [
          (header.sync >> 8) as u8,
          header.sync as u8,
          header.frame_type as u8,
          header.sequence_number,
          header.length,
          header.source_address,
          header.destination_address,
        ];

        for &byte in &header_bytes {
          crc ^= (byte as u16) << 8;
          for _ in 0..8 {
            if crc & 0x8000 != 0 {
              crc = (crc << 1) ^ polynomial;
            } else {
              crc <<= 1;
            }
          }
        }

        // 处理载荷
        for &byte in payload {
          crc ^= (byte as u16) << 8;
          for _ in 0..8 {
            if crc & 0x8000 != 0 {
              crc = (crc << 1) ^ polynomial;
            } else {
              crc <<= 1;
            }
          }
        }

        Ok(crc as u32)
      }
      _ => Ok(0), // 其他方法的简化实现
    }
  }

  /// 处理流控制
  pub fn handle_flow_control(&mut self, frame: &DataFrame) -> FlowControlAction {
    match self.flow_control {
      FlowControlMethod::None => FlowControlAction::Continue,
      FlowControlMethod::StopAndWait => {
        // 停等协议：发送后等待ACK
        if matches!(frame.header.frame_type, FrameType::Data) {
          FlowControlAction::WaitForAck
        } else {
          FlowControlAction::Continue
        }
      }
      FlowControlMethod::XonXoff => {
        // XON/XOFF流控制
        if self.frame_stats.frames_received > 100 {
          FlowControlAction::SendXoff
        } else {
          FlowControlAction::Continue
        }
      }
      _ => FlowControlAction::Continue,
    }
  }
}

/// 流控制动作
#[derive(Debug)]
pub enum FlowControlAction {
  Continue,
  WaitForAck,
  SendXoff,
  SendXon,
  Pause,
}

/// 数据链路层错误
#[derive(Debug)]
pub enum DataLinkError {
  PayloadTooLarge,
  BufferOverflow,
  SyncError,
  FramingError,
  LengthError,
  ChecksumError,
  SequenceError,
}

/// 网络层实现
pub struct NetworkLayer {
  /// 本地地址
  local_address: NetworkAddress,
  /// 路由表
  routing_table: FnvIndexMap<NetworkAddress, NextHop, 16>,
  /// 分片管理
  fragmentation: FragmentationManager,
}

/// 网络地址
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NetworkAddress(pub u32);

/// 下一跳信息
#[derive(Debug, Clone)]
pub struct NextHop {
  pub gateway: NetworkAddress,
  pub interface: u8,
  pub metric: u16,
}

/// 分片管理器
pub struct FragmentationManager {
  /// 分片缓存
  fragment_cache: FnvIndexMap<u16, FragmentBuffer, 8>,
  /// 下一个分片ID
  next_fragment_id: u16,
}

/// 分片缓存
#[derive(Debug)]
pub struct FragmentBuffer {
  pub fragments: Vec<Fragment, 8>,
  pub total_length: u16,
  pub received_length: u16,
  pub timestamp: u32,
}

/// 分片
#[derive(Debug, Clone)]
pub struct Fragment {
  pub offset: u16,
  pub data: Vec<u8, 64>,
  pub more_fragments: bool,
}

/// 网络数据包
#[derive(Debug, Clone)]
pub struct NetworkPacket {
  pub header: NetworkHeader,
  pub payload: Vec<u8, 256>,
}

/// 网络头部
#[derive(Debug, Clone)]
pub struct NetworkHeader {
  pub version: u8,
  pub header_length: u8,
  pub type_of_service: u8,
  pub total_length: u16,
  pub identification: u16,
  pub flags: u8,
  pub fragment_offset: u16,
  pub time_to_live: u8,
  pub protocol: u8,
  pub header_checksum: u16,
  pub source_address: NetworkAddress,
  pub destination_address: NetworkAddress,
}

impl NetworkLayer {
  pub fn new(local_address: NetworkAddress) -> Self {
    Self {
      local_address,
      routing_table: FnvIndexMap::new(),
      fragmentation: FragmentationManager {
        fragment_cache: FnvIndexMap::new(),
        next_fragment_id: 1,
      },
    }
  }

  /// 添加路由
  pub fn add_route(&mut self, dest: NetworkAddress, next_hop: NextHop) -> Result<(), NetworkError> {
    self
      .routing_table
      .insert(dest, next_hop)
      .map_err(|_| NetworkError::RoutingTableFull)?;
    Ok(())
  }

  /// 路由数据包
  pub fn route_packet(&self, packet: &NetworkPacket) -> Result<NextHop, NetworkError> {
    // 检查是否为本地地址
    if packet.header.destination_address == self.local_address {
      return Err(NetworkError::LocalDelivery);
    }

    // 查找路由表
    if let Some(next_hop) = self.routing_table.get(&packet.header.destination_address) {
      Ok(next_hop.clone())
    } else {
      // 查找默认路由
      if let Some(default_route) = self.routing_table.get(&NetworkAddress(0)) {
        Ok(default_route.clone())
      } else {
        Err(NetworkError::NoRoute)
      }
    }
  }

  /// 分片数据包
  pub fn fragment_packet(
    &mut self,
    packet: &NetworkPacket,
    mtu: u16,
  ) -> Result<Vec<NetworkPacket, 8>, NetworkError> {
    if packet.payload.len() <= mtu as usize {
      // 不需要分片
      let mut result = Vec::new();
      result
        .push(packet.clone())
        .map_err(|_| NetworkError::BufferOverflow)?;
      return Ok(result);
    }

    let mut fragments = Vec::new();
    let fragment_id = self.fragmentation.next_fragment_id;
    self.fragmentation.next_fragment_id = self.fragmentation.next_fragment_id.wrapping_add(1);

    let data_per_fragment = (mtu - 20) as usize; // 假设IP头部20字节
    let total_fragments = (packet.payload.len() + data_per_fragment - 1) / data_per_fragment;

    for i in 0..total_fragments {
      let offset = i * data_per_fragment;
      let end = core::cmp::min(offset + data_per_fragment, packet.payload.len());
      let is_last = i == total_fragments - 1;

      let mut fragment_data = Vec::new();
      for j in offset..end {
        fragment_data
          .push(packet.payload[j])
          .map_err(|_| NetworkError::BufferOverflow)?;
      }

      let fragment_header = NetworkHeader {
        version: packet.header.version,
        header_length: packet.header.header_length,
        type_of_service: packet.header.type_of_service,
        total_length: (20 + fragment_data.len()) as u16,
        identification: fragment_id,
        flags: if is_last { 0 } else { 0x20 }, // More Fragments flag
        fragment_offset: (offset / 8) as u16,
        time_to_live: packet.header.time_to_live,
        protocol: packet.header.protocol,
        header_checksum: 0, // 需要重新计算
        source_address: packet.header.source_address,
        destination_address: packet.header.destination_address,
      };

      let fragment = NetworkPacket {
        header: fragment_header,
        payload: fragment_data,
      };

      fragments
        .push(fragment)
        .map_err(|_| NetworkError::BufferOverflow)?;
    }

    Ok(fragments)
  }

  /// 重组分片
  pub fn reassemble_fragments(
    &mut self,
    fragment: NetworkPacket,
  ) -> Result<Option<NetworkPacket>, NetworkError> {
    let fragment_id = fragment.header.identification;
    let offset = (fragment.header.fragment_offset * 8) as usize;
    let more_fragments = (fragment.header.flags & 0x20) != 0;

    // 获取或创建分片缓存
    let fragment_buffer = self
      .fragmentation
      .fragment_cache
      .entry(fragment_id)
      .or_insert(FragmentBuffer {
        fragments: Vec::new(),
        total_length: 0,
        received_length: 0,
        timestamp: 0, // 实际应用中会使用系统时间
      });

    // 添加分片
    let fragment_info = Fragment {
      offset: offset as u16,
      data: fragment.payload.clone(),
      more_fragments,
    };

    fragment_buffer
      .fragments
      .push(fragment_info)
      .map_err(|_| NetworkError::FragmentBufferFull)?;
    fragment_buffer.received_length += fragment.payload.len() as u16;

    // 如果是最后一个分片，设置总长度
    if !more_fragments {
      fragment_buffer.total_length = offset as u16 + fragment.payload.len() as u16;
    }

    // 检查是否所有分片都已接收
    if fragment_buffer.total_length > 0
      && fragment_buffer.received_length >= fragment_buffer.total_length
    {
      // 重组数据包
      let mut reassembled_data = Vec::new();

      // 按偏移量排序分片
      fragment_buffer.fragments.sort_by_key(|f| f.offset);

      for fragment in &fragment_buffer.fragments {
        for &byte in &fragment.data {
          reassembled_data
            .push(byte)
            .map_err(|_| NetworkError::BufferOverflow)?;
        }
      }

      let reassembled_packet = NetworkPacket {
        header: NetworkHeader {
          version: fragment.header.version,
          header_length: fragment.header.header_length,
          type_of_service: fragment.header.type_of_service,
          total_length: (20 + reassembled_data.len()) as u16,
          identification: fragment_id,
          flags: 0,
          fragment_offset: 0,
          time_to_live: fragment.header.time_to_live,
          protocol: fragment.header.protocol,
          header_checksum: fragment.header.header_checksum,
          source_address: fragment.header.source_address,
          destination_address: fragment.header.destination_address,
        },
        payload: reassembled_data,
      };

      // 清理分片缓存
      self.fragmentation.fragment_cache.remove(&fragment_id);

      Ok(Some(reassembled_packet))
    } else {
      Ok(None) // 还需要更多分片
    }
  }
}

/// 网络层错误
#[derive(Debug)]
pub enum NetworkError {
  RoutingTableFull,
  NoRoute,
  LocalDelivery,
  BufferOverflow,
  FragmentBufferFull,
  InvalidPacket,
}

/// 传输层实现
pub struct TransportLayer {
  /// 连接管理
  connections: FnvIndexMap<ConnectionId, Connection, 8>,
  /// 下一个连接ID
  next_connection_id: u16,
  /// 可靠传输配置
  reliability_config: ReliabilityConfig,
}

/// 连接标识
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ConnectionId {
  pub local_port: u16,
  pub remote_port: u16,
  pub remote_address: NetworkAddress,
}

/// 连接状态
#[derive(Debug)]
pub struct Connection {
  pub state: ConnectionState,
  pub send_sequence: u32,
  pub receive_sequence: u32,
  pub send_window: u16,
  pub receive_window: u16,
  pub retransmit_queue: Vec<TransportSegment, 16>,
  pub receive_buffer: Vec<u8, 512>,
}

/// 连接状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConnectionState {
  Closed,
  Listen,
  SynSent,
  SynReceived,
  Established,
  FinWait1,
  FinWait2,
  CloseWait,
  Closing,
  LastAck,
  TimeWait,
}

/// 可靠传输配置
#[derive(Debug, Clone)]
pub struct ReliabilityConfig {
  pub enable_acknowledgment: bool,
  pub enable_retransmission: bool,
  pub retransmit_timeout: u32, // milliseconds
  pub max_retransmit_attempts: u8,
  pub window_size: u16,
}

/// 传输段
#[derive(Debug, Clone)]
pub struct TransportSegment {
  pub header: TransportHeader,
  pub payload: Vec<u8, 256>,
}

/// 传输头部
#[derive(Debug, Clone)]
pub struct TransportHeader {
  pub source_port: u16,
  pub destination_port: u16,
  pub sequence_number: u32,
  pub acknowledgment_number: u32,
  pub flags: u8,
  pub window_size: u16,
  pub checksum: u16,
  pub urgent_pointer: u16,
}

// 传输层标志位
pub const FLAG_FIN: u8 = 0x01;
pub const FLAG_SYN: u8 = 0x02;
pub const FLAG_RST: u8 = 0x04;
pub const FLAG_PSH: u8 = 0x08;
pub const FLAG_ACK: u8 = 0x10;
pub const FLAG_URG: u8 = 0x20;

impl TransportLayer {
  pub fn new() -> Self {
    Self {
      connections: FnvIndexMap::new(),
      next_connection_id: 1,
      reliability_config: ReliabilityConfig {
        enable_acknowledgment: true,
        enable_retransmission: true,
        retransmit_timeout: 1000, // 1 second
        max_retransmit_attempts: 3,
        window_size: 1024,
      },
    }
  }

  /// 建立连接
  pub fn establish_connection(
    &mut self,
    remote_address: NetworkAddress,
    remote_port: u16,
    local_port: u16,
  ) -> Result<ConnectionId, TransportError> {
    let connection_id = ConnectionId {
      local_port,
      remote_port,
      remote_address,
    };

    if self.connections.contains_key(&connection_id) {
      return Err(TransportError::ConnectionExists);
    }

    let connection = Connection {
      state: ConnectionState::SynSent,
      send_sequence: 1000, // 初始序列号
      receive_sequence: 0,
      send_window: self.reliability_config.window_size,
      receive_window: self.reliability_config.window_size,
      retransmit_queue: Vec::new(),
      receive_buffer: Vec::new(),
    };

    self
      .connections
      .insert(connection_id, connection)
      .map_err(|_| TransportError::ConnectionTableFull)?;

    Ok(connection_id)
  }

  /// 发送数据
  pub fn send_data(
    &mut self,
    connection_id: ConnectionId,
    data: &[u8],
  ) -> Result<TransportSegment, TransportError> {
    let connection = self
      .connections
      .get_mut(&connection_id)
      .ok_or(TransportError::ConnectionNotFound)?;

    if connection.state != ConnectionState::Established {
      return Err(TransportError::ConnectionNotEstablished);
    }

    if data.len() > 256 {
      return Err(TransportError::DataTooLarge);
    }

    let mut payload = Vec::new();
    for &byte in data {
      payload
        .push(byte)
        .map_err(|_| TransportError::BufferOverflow)?;
    }

    let segment = TransportSegment {
      header: TransportHeader {
        source_port: connection_id.local_port,
        destination_port: connection_id.remote_port,
        sequence_number: connection.send_sequence,
        acknowledgment_number: connection.receive_sequence,
        flags: FLAG_PSH | FLAG_ACK,
        window_size: connection.receive_window,
        checksum: 0, // 需要计算
        urgent_pointer: 0,
      },
      payload,
    };

    // 更新发送序列号
    connection.send_sequence += data.len() as u32;

    // 如果启用重传，添加到重传队列
    if self.reliability_config.enable_retransmission {
      connection
        .retransmit_queue
        .push(segment.clone())
        .map_err(|_| TransportError::RetransmitQueueFull)?;
    }

    Ok(segment)
  }

  /// 接收数据
  pub fn receive_data(
    &mut self,
    segment: TransportSegment,
  ) -> Result<Option<Vec<u8, 256>>, TransportError> {
    let connection_id = ConnectionId {
      local_port: segment.header.destination_port,
      remote_port: segment.header.source_port,
      remote_address: NetworkAddress(0), // 需要从网络层获取
    };

    let connection = self
      .connections
      .get_mut(&connection_id)
      .ok_or(TransportError::ConnectionNotFound)?;

    // 检查序列号
    if segment.header.sequence_number != connection.receive_sequence {
      return Err(TransportError::SequenceError);
    }

    // 更新接收序列号
    connection.receive_sequence += segment.payload.len() as u32;

    // 处理ACK
    if segment.header.flags & FLAG_ACK != 0 {
      self.process_acknowledgment(connection, segment.header.acknowledgment_number);
    }

    // 返回数据
    if !segment.payload.is_empty() {
      Ok(Some(segment.payload))
    } else {
      Ok(None)
    }
  }

  /// 处理确认
  fn process_acknowledgment(&mut self, connection: &mut Connection, ack_number: u32) {
    // 从重传队列中移除已确认的段
    connection.retransmit_queue.retain(|segment| {
      let segment_end = segment.header.sequence_number + segment.payload.len() as u32;
      segment_end > ack_number
    });
  }

  /// 关闭连接
  pub fn close_connection(
    &mut self,
    connection_id: ConnectionId,
  ) -> Result<TransportSegment, TransportError> {
    let connection = self
      .connections
      .get_mut(&connection_id)
      .ok_or(TransportError::ConnectionNotFound)?;

    connection.state = ConnectionState::FinWait1;

    let fin_segment = TransportSegment {
      header: TransportHeader {
        source_port: connection_id.local_port,
        destination_port: connection_id.remote_port,
        sequence_number: connection.send_sequence,
        acknowledgment_number: connection.receive_sequence,
        flags: FLAG_FIN | FLAG_ACK,
        window_size: connection.receive_window,
        checksum: 0,
        urgent_pointer: 0,
      },
      payload: Vec::new(),
    };

    Ok(fin_segment)
  }
}

/// 传输层错误
#[derive(Debug)]
pub enum TransportError {
  ConnectionExists,
  ConnectionNotFound,
  ConnectionNotEstablished,
  ConnectionTableFull,
  DataTooLarge,
  BufferOverflow,
  RetransmitQueueFull,
  SequenceError,
  ChecksumError,
}

/// 应用层实现
pub struct ApplicationLayer {
  /// 应用协议处理器
  protocol_handlers: FnvIndexMap<u16, ProtocolHandler, 8>,
  /// 会话管理
  sessions: FnvIndexMap<SessionId, Session, 8>,
  /// 下一个会话ID
  next_session_id: u32,
}

/// 协议处理器
pub struct ProtocolHandler {
  pub protocol_id: u16,
  pub name: String<32>,
  pub handler_fn: fn(&[u8]) -> Result<Vec<u8, 256>, ApplicationError>,
}

/// 会话标识
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SessionId(pub u32);

/// 会话
#[derive(Debug)]
pub struct Session {
  pub id: SessionId,
  pub protocol: u16,
  pub state: SessionState,
  pub data_buffer: Vec<u8, 512>,
  pub timestamp: u32,
}

/// 会话状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SessionState {
  Inactive,
  Active,
  Suspended,
  Terminated,
}

impl ApplicationLayer {
  pub fn new() -> Self {
    let mut layer = Self {
      protocol_handlers: FnvIndexMap::new(),
      sessions: FnvIndexMap::new(),
      next_session_id: 1,
    };

    // 注册默认协议处理器
    layer.register_default_handlers();
    layer
  }

  /// 注册默认协议处理器
  fn register_default_handlers(&mut self) {
    // HTTP协议处理器
    let http_handler = ProtocolHandler {
      protocol_id: 80,
      name: String::from("HTTP"),
      handler_fn: Self::handle_http,
    };
    let _ = self.protocol_handlers.insert(80, http_handler);

    // MQTT协议处理器
    let mqtt_handler = ProtocolHandler {
      protocol_id: 1883,
      name: String::from("MQTT"),
      handler_fn: Self::handle_mqtt,
    };
    let _ = self.protocol_handlers.insert(1883, mqtt_handler);

    // 自定义协议处理器
    let custom_handler = ProtocolHandler {
      protocol_id: 8080,
      name: String::from("CUSTOM"),
      handler_fn: Self::handle_custom,
    };
    let _ = self.protocol_handlers.insert(8080, custom_handler);
  }

  /// HTTP协议处理器
  fn handle_http(data: &[u8]) -> Result<Vec<u8, 256>, ApplicationError> {
    // 简化的HTTP处理
    let mut response = Vec::new();
    let http_response = b"HTTP/1.1 200 OK\r\nContent-Length: 13\r\n\r\nHello, World!";

    for &byte in http_response {
      response
        .push(byte)
        .map_err(|_| ApplicationError::BufferOverflow)?;
    }

    Ok(response)
  }

  /// MQTT协议处理器
  fn handle_mqtt(data: &[u8]) -> Result<Vec<u8, 256>, ApplicationError> {
    if data.is_empty() {
      return Err(ApplicationError::InvalidData);
    }

    // 简化的MQTT CONNACK响应
    let mut response = Vec::new();
    let connack = [0x20, 0x02, 0x00, 0x00]; // CONNACK with return code 0

    for &byte in &connack {
      response
        .push(byte)
        .map_err(|_| ApplicationError::BufferOverflow)?;
    }

    Ok(response)
  }

  /// 自定义协议处理器
  fn handle_custom(data: &[u8]) -> Result<Vec<u8, 256>, ApplicationError> {
    let mut response = Vec::new();

    // 回显数据
    for &byte in data {
      response
        .push(byte)
        .map_err(|_| ApplicationError::BufferOverflow)?;
    }

    Ok(response)
  }

  /// 创建会话
  pub fn create_session(&mut self, protocol: u16) -> Result<SessionId, ApplicationError> {
    let session_id = SessionId(self.next_session_id);
    self.next_session_id += 1;

    let session = Session {
      id: session_id,
      protocol,
      state: SessionState::Active,
      data_buffer: Vec::new(),
      timestamp: 0, // 实际应用中使用系统时间
    };

    self
      .sessions
      .insert(session_id, session)
      .map_err(|_| ApplicationError::SessionTableFull)?;

    Ok(session_id)
  }

  /// 处理应用数据
  pub fn process_data(
    &mut self,
    session_id: SessionId,
    data: &[u8],
  ) -> Result<Vec<u8, 256>, ApplicationError> {
    let session = self
      .sessions
      .get(&session_id)
      .ok_or(ApplicationError::SessionNotFound)?;

    let handler = self
      .protocol_handlers
      .get(&session.protocol)
      .ok_or(ApplicationError::ProtocolNotSupported)?;

    (handler.handler_fn)(data)
  }

  /// 关闭会话
  pub fn close_session(&mut self, session_id: SessionId) -> Result<(), ApplicationError> {
    if let Some(mut session) = self.sessions.get_mut(&session_id) {
      session.state = SessionState::Terminated;
      Ok(())
    } else {
      Err(ApplicationError::SessionNotFound)
    }
  }
}

/// 应用层错误
#[derive(Debug)]
pub enum ApplicationError {
  SessionTableFull,
  SessionNotFound,
  ProtocolNotSupported,
  InvalidData,
  BufferOverflow,
  ProcessingError,
}

/// 完整协议栈
pub struct ProtocolStack {
  physical: PhysicalLayer,
  data_link: DataLinkLayer,
  network: NetworkLayer,
  transport: TransportLayer,
  application: ApplicationLayer,
}

impl ProtocolStack {
  pub fn new(local_address: NetworkAddress) -> Self {
    Self {
      physical: PhysicalLayer::new(ModulationType::NRZ, 115200),
      data_link: DataLinkLayer::new(
        FrameSyncMode::FlagSync,
        ErrorDetectionMethod::CRC16,
        FlowControlMethod::StopAndWait,
      ),
      network: NetworkLayer::new(local_address),
      transport: TransportLayer::new(),
      application: ApplicationLayer::new(),
    }
  }

  /// 发送数据（从应用层到物理层）
  pub fn send(
    &mut self,
    data: &[u8],
    dest_addr: NetworkAddress,
    dest_port: u16,
    local_port: u16,
  ) -> Result<Vec<u8, 512>, StackError> {
    // 应用层处理
    let session_id = self
      .application
      .create_session(8080)
      .map_err(StackError::Application)?;

    // 传输层处理
    let connection_id = self
      .transport
      .establish_connection(dest_addr, dest_port, local_port)
      .map_err(StackError::Transport)?;
    let transport_segment = self
      .transport
      .send_data(connection_id, data)
      .map_err(StackError::Transport)?;

    // 网络层处理
    let mut network_payload = Vec::new();
    // 序列化传输段头部
    let header_bytes = [
      (transport_segment.header.source_port >> 8) as u8,
      transport_segment.header.source_port as u8,
      (transport_segment.header.destination_port >> 8) as u8,
      transport_segment.header.destination_port as u8,
      // ... 其他头部字段
    ];
    for &byte in &header_bytes {
      network_payload
        .push(byte)
        .map_err(|_| StackError::BufferOverflow)?;
    }
    for &byte in &transport_segment.payload {
      network_payload
        .push(byte)
        .map_err(|_| StackError::BufferOverflow)?;
    }

    let network_packet = NetworkPacket {
      header: NetworkHeader {
        version: 4,
        header_length: 5,
        type_of_service: 0,
        total_length: (20 + network_payload.len()) as u16,
        identification: 1,
        flags: 0,
        fragment_offset: 0,
        time_to_live: 64,
        protocol: 6, // TCP
        header_checksum: 0,
        source_address: self.network.local_address,
        destination_address: dest_addr,
      },
      payload: network_payload,
    };

    // 数据链路层处理
    let mut datalink_payload = Vec::new();
    // 序列化网络包
    for &byte in &network_packet.payload {
      datalink_payload
        .push(byte)
        .map_err(|_| StackError::BufferOverflow)?;
    }

    let frame = self
      .data_link
      .encapsulate(&datalink_payload, dest_addr.0 as u8)
      .map_err(StackError::DataLink)?;

    // 物理层处理
    let mut physical_data = Vec::new();
    // 序列化帧
    for &byte in &frame.payload {
      physical_data
        .push(byte)
        .map_err(|_| StackError::BufferOverflow)?;
    }

    let encoded_data = self
      .physical
      .encode(&physical_data)
      .map_err(StackError::Physical)?;

    Ok(encoded_data)
  }

  /// 接收数据（从物理层到应用层）
  pub fn receive(&mut self, encoded_data: &[u8]) -> Result<Vec<u8, 256>, StackError> {
    // 物理层处理
    let decoded_data = self
      .physical
      .decode(encoded_data)
      .map_err(StackError::Physical)?;

    // 数据链路层处理
    // 这里需要从解码数据重构DataFrame，简化处理
    let dummy_frame = DataFrame {
      header: FrameHeader {
        sync: 0xAA55,
        frame_type: FrameType::Data,
        sequence_number: 0,
        length: decoded_data.len() as u8,
        source_address: 0x02,
        destination_address: 0x01,
      },
      payload: decoded_data.clone(),
      trailer: FrameTrailer {
        error_check: 0,
        end_flag: 0x55AA,
      },
    };

    let datalink_payload = self
      .data_link
      .decapsulate(&dummy_frame)
      .map_err(StackError::DataLink)?;

    // 网络层处理（简化）
    // 传输层处理（简化）
    // 应用层处理（简化）

    Ok(datalink_payload)
  }
}

/// 协议栈错误
#[derive(Debug)]
pub enum StackError {
  Physical(PhysicalLayerError),
  DataLink(DataLinkError),
  Network(NetworkError),
  Transport(TransportError),
  Application(ApplicationError),
  BufferOverflow,
}

#[entry]
fn main() -> ! {
  // 初始化协议栈
  let mut stack = ProtocolStack::new(NetworkAddress(0x0A000001)); // 10.0.0.1

  // 添加路由
  let _ = stack.network.add_route(
    NetworkAddress(0x0A000002), // 10.0.0.2
    NextHop {
      gateway: NetworkAddress(0x0A000001),
      interface: 0,
      metric: 1,
    },
  );

  // 测试数据
  let test_data = b"Hello, Protocol Stack!";
  let dest_addr = NetworkAddress(0x0A000002);

  loop {
    // 发送数据测试
    match stack.send(test_data, dest_addr, 80, 8080) {
      Ok(encoded_data) => {
        // 模拟接收相同数据
        match stack.receive(&encoded_data) {
          Ok(_received_data) => {
            // 数据处理成功
          }
          Err(_e) => {
            // 处理接收错误
          }
        }
      }
      Err(_e) => {
        // 处理发送错误
      }
    }

    // 在实际应用中，这里会：
    // 1. 处理来自各层的中断
    // 2. 管理连接状态
    // 3. 处理定时器事件
    // 4. 执行协议栈维护任务

    asm::wfi(); // 等待中断
  }
}
