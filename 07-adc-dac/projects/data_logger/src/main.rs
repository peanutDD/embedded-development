#![no_std]
#![no_main]

extern crate alloc;
use alloc::{vec::Vec, string::String, format};
use linked_list_allocator::LockedHeap;

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::{asm, peripheral::DWT};
use stm32f4xx_hal::{
    pac::{self, ADC1, ADC2, ADC3, DMA2, TIM2, SDIO, RTC},
    prelude::*,
    adc::{Adc, config::{AdcConfig, SampleTime, Sequence, Eoc, Scan, Resolution}},
    dma::{Stream0, Channel0, StreamsTuple, Transfer, MemoryToPeripheral, config::DmaConfig},
    timer::{Timer, Event},
    gpio::{Analog, Pin, Output, PushPull},
    rcc::Clocks,
    rtc::{Rtc, DateTimeFilter},
    sdio::{Sdio, ClockFreq},
};
use heapless::{Vec as HeaplessVec, FnvIndexMap, String as HeaplessString};
use serde::{Serialize, Deserialize};
use postcard;
use crc::{Crc, CRC_32_ISO_HDLC};
use embedded_sdmmc::{SdCard, VolumeIdx, Directory, File, Mode, Controller, TimeSource, Timestamp};
use fugit::{Duration, Instant};
use critical_section::Mutex;
use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
    mem::MaybeUninit,
};

// 全局堆分配器
#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

// 堆内存区域
static mut HEAP: [MaybeUninit<u8>; 32768] = [MaybeUninit::uninit(); 32768];

/// 数据记录器配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggerConfig {
    pub sample_rate: u32,
    pub channels: HeaplessVec<ChannelConfig, 8>,
    pub buffer_size: usize,
    pub file_rotation_size: u32,
    pub compression_enabled: bool,
    pub encryption_enabled: bool,
    pub timestamp_enabled: bool,
}

/// 通道配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChannelConfig {
    pub id: u8,
    pub name: HeaplessString<16>,
    pub enabled: bool,
    pub gain: f32,
    pub offset: f32,
    pub unit: HeaplessString<8>,
    pub min_value: f32,
    pub max_value: f32,
}

/// 数据样本
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataSample {
    pub timestamp: u64,
    pub channel_id: u8,
    pub raw_value: u16,
    pub scaled_value: f32,
    pub quality: SampleQuality,
}

/// 样本质量标志
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum SampleQuality {
    Good,
    Saturated,
    Underrange,
    Overrange,
    Invalid,
}

/// 数据包
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPacket {
    pub header: PacketHeader,
    pub samples: HeaplessVec<DataSample, 256>,
    pub checksum: u32,
}

/// 数据包头
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PacketHeader {
    pub packet_id: u32,
    pub timestamp: u64,
    pub sample_count: u16,
    pub compression_type: CompressionType,
    pub encryption_type: EncryptionType,
}

/// 压缩类型
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum CompressionType {
    None,
    RunLength,
    Delta,
    Huffman,
}

/// 加密类型
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum EncryptionType {
    None,
    Aes128,
    Aes256,
}

/// 文件系统状态
#[derive(Debug, Clone)]
pub struct FileSystemStatus {
    pub total_space: u64,
    pub free_space: u64,
    pub current_file_size: u32,
    pub files_created: u32,
    pub write_errors: u32,
}

/// 数据记录器主结构
pub struct DataLogger {
    config: LoggerConfig,
    adc_channels: HeaplessVec<Adc<ADC1>, 3>,
    sample_buffer: HeaplessVec<DataSample, 1024>,
    packet_buffer: HeaplessVec<DataPacket, 16>,
    file_system: Option<Controller<SdCard<Sdio>, DummyTimeSource>>,
    current_file: Option<File>,
    statistics: LoggerStatistics,
    status: LoggerStatus,
    packet_counter: u32,
}

/// 记录器统计信息
#[derive(Debug, Clone)]
pub struct LoggerStatistics {
    pub samples_collected: u64,
    pub packets_written: u32,
    pub bytes_written: u64,
    pub write_errors: u32,
    pub buffer_overruns: u32,
    pub compression_ratio: f32,
    pub average_sample_rate: f32,
}

/// 记录器状态
#[derive(Debug, Clone, Copy)]
pub enum LoggerStatus {
    Stopped,
    Starting,
    Running,
    Paused,
    Error(ErrorCode),
}

/// 错误代码
#[derive(Debug, Clone, Copy)]
pub enum ErrorCode {
    AdcError,
    FileSystemError,
    BufferOverflow,
    CompressionError,
    EncryptionError,
    ConfigurationError,
}

/// 时间源实现
pub struct DummyTimeSource;

impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 54, // 2024年
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// 数据压缩器
pub struct DataCompressor {
    compression_type: CompressionType,
    compression_buffer: HeaplessVec<u8, 2048>,
}

/// 数据加密器
pub struct DataEncryptor {
    encryption_type: EncryptionType,
    key: [u8; 32],
    iv: [u8; 16],
}

/// 全局状态
static SAMPLE_READY: AtomicBool = AtomicBool::new(false);
static CURRENT_SAMPLE: Mutex<RefCell<Option<DataSample>>> = Mutex::new(RefCell::new(None));

impl DataLogger {
    pub fn new(config: LoggerConfig) -> Self {
        Self {
            config,
            adc_channels: HeaplessVec::new(),
            sample_buffer: HeaplessVec::new(),
            packet_buffer: HeaplessVec::new(),
            file_system: None,
            current_file: None,
            statistics: LoggerStatistics::new(),
            status: LoggerStatus::Stopped,
            packet_counter: 0,
        }
    }

    /// 初始化ADC通道
    pub fn init_adc_channels(&mut self, adc1: Adc<ADC1>) -> Result<(), ErrorCode> {
        // 配置ADC参数
        let adc_config = AdcConfig::default()
            .resolution(Resolution::Twelve)
            .end_of_conversion_interrupt(Eoc::Conversion)
            .scan(Scan::Enabled);

        if self.adc_channels.push(adc1).is_err() {
            return Err(ErrorCode::ConfigurationError);
        }

        Ok(())
    }

    /// 初始化文件系统
    pub fn init_filesystem(&mut self, sdio: Sdio) -> Result<(), ErrorCode> {
        let sd_card = SdCard::new(sdio, ClockFreq::F25Mhz);
        let time_source = DummyTimeSource;
        
        match Controller::new(sd_card, time_source) {
            Ok(controller) => {
                self.file_system = Some(controller);
                Ok(())
            }
            Err(_) => Err(ErrorCode::FileSystemError),
        }
    }

    /// 开始数据记录
    pub fn start_logging(&mut self) -> Result<(), ErrorCode> {
        self.status = LoggerStatus::Starting;
        
        // 创建新的数据文件
        self.create_new_file()?;
        
        // 清空缓冲区
        self.sample_buffer.clear();
        self.packet_buffer.clear();
        
        // 重置统计信息
        self.statistics = LoggerStatistics::new();
        
        self.status = LoggerStatus::Running;
        Ok(())
    }

    /// 停止数据记录
    pub fn stop_logging(&mut self) -> Result<(), ErrorCode> {
        // 写入剩余数据
        self.flush_buffers()?;
        
        // 关闭当前文件
        if let Some(_file) = self.current_file.take() {
            // 文件会在drop时自动关闭
        }
        
        self.status = LoggerStatus::Stopped;
        Ok(())
    }

    /// 处理ADC样本
    pub fn process_sample(&mut self, channel_id: u8, raw_value: u16) -> Result<(), ErrorCode> {
        if !matches!(self.status, LoggerStatus::Running) {
            return Ok(());
        }

        // 查找通道配置
        let channel_config = self.config.channels.iter()
            .find(|ch| ch.id == channel_id && ch.enabled)
            .ok_or(ErrorCode::ConfigurationError)?;

        // 计算缩放值
        let scaled_value = (raw_value as f32 * channel_config.gain) + channel_config.offset;
        
        // 检查数据质量
        let quality = self.check_sample_quality(scaled_value, channel_config);

        // 创建数据样本
        let sample = DataSample {
            timestamp: DWT::cycle_count() as u64,
            channel_id,
            raw_value,
            scaled_value,
            quality,
        };

        // 添加到缓冲区
        if self.sample_buffer.push(sample).is_err() {
            self.statistics.buffer_overruns += 1;
            return Err(ErrorCode::BufferOverflow);
        }

        self.statistics.samples_collected += 1;

        // 检查是否需要创建数据包
        if self.sample_buffer.len() >= 256 {
            self.create_data_packet()?;
        }

        Ok(())
    }

    /// 检查样本质量
    fn check_sample_quality(&self, value: f32, config: &ChannelConfig) -> SampleQuality {
        if value < config.min_value {
            SampleQuality::Underrange
        } else if value > config.max_value {
            SampleQuality::Overrange
        } else if value >= config.max_value * 0.95 {
            SampleQuality::Saturated
        } else {
            SampleQuality::Good
        }
    }

    /// 创建数据包
    fn create_data_packet(&mut self) -> Result<(), ErrorCode> {
        if self.sample_buffer.is_empty() {
            return Ok(());
        }

        // 创建数据包头
        let header = PacketHeader {
            packet_id: self.packet_counter,
            timestamp: DWT::cycle_count() as u64,
            sample_count: self.sample_buffer.len() as u16,
            compression_type: if self.config.compression_enabled {
                CompressionType::Delta
            } else {
                CompressionType::None
            },
            encryption_type: if self.config.encryption_enabled {
                EncryptionType::Aes128
            } else {
                EncryptionType::None
            },
        };

        // 复制样本数据
        let mut samples = HeaplessVec::new();
        for sample in &self.sample_buffer {
            if samples.push(*sample).is_err() {
                break;
            }
        }

        // 计算校验和
        let serialized = postcard::to_vec::<_, 2048>(&samples)
            .map_err(|_| ErrorCode::CompressionError)?;
        let crc = Crc::<u32>::new(&CRC_32_ISO_HDLC);
        let checksum = crc.checksum(&serialized);

        // 创建数据包
        let packet = DataPacket {
            header,
            samples,
            checksum,
        };

        // 添加到包缓冲区
        if self.packet_buffer.push(packet).is_err() {
            return Err(ErrorCode::BufferOverflow);
        }

        // 清空样本缓冲区
        self.sample_buffer.clear();
        self.packet_counter += 1;

        // 检查是否需要写入文件
        if self.packet_buffer.len() >= 8 {
            self.write_packets_to_file()?;
        }

        Ok(())
    }

    /// 写入数据包到文件
    fn write_packets_to_file(&mut self) -> Result<(), ErrorCode> {
        if let Some(ref mut file_system) = self.file_system {
            if let Some(ref mut file) = self.current_file {
                for packet in &self.packet_buffer {
                    // 序列化数据包
                    let serialized = postcard::to_vec::<_, 4096>(packet)
                        .map_err(|_| ErrorCode::CompressionError)?;

                    // 写入文件
                    match file.write(&serialized) {
                        Ok(bytes_written) => {
                            self.statistics.bytes_written += bytes_written as u64;
                            self.statistics.packets_written += 1;
                        }
                        Err(_) => {
                            self.statistics.write_errors += 1;
                            return Err(ErrorCode::FileSystemError);
                        }
                    }
                }

                // 清空包缓冲区
                self.packet_buffer.clear();

                // 检查文件大小是否需要轮转
                if self.statistics.bytes_written > self.config.file_rotation_size as u64 {
                    self.rotate_file()?;
                }
            }
        }

        Ok(())
    }

    /// 创建新文件
    fn create_new_file(&mut self) -> Result<(), ErrorCode> {
        if let Some(ref mut file_system) = self.file_system {
            // 生成文件名
            let filename = format!("data_{:08}.log", self.packet_counter);
            
            // 打开根目录
            let mut volume = file_system.get_volume(VolumeIdx(0))
                .map_err(|_| ErrorCode::FileSystemError)?;
            let root_dir = file_system.open_root_dir(&volume)
                .map_err(|_| ErrorCode::FileSystemError)?;

            // 创建新文件
            let file = file_system.open_file_in_dir(&mut volume, &root_dir, &filename, Mode::ReadWriteCreateOrTruncate)
                .map_err(|_| ErrorCode::FileSystemError)?;

            self.current_file = Some(file);
            self.statistics.bytes_written = 0;
        }

        Ok(())
    }

    /// 文件轮转
    fn rotate_file(&mut self) -> Result<(), ErrorCode> {
        // 关闭当前文件
        if let Some(_file) = self.current_file.take() {
            // 文件会在drop时自动关闭
        }

        // 创建新文件
        self.create_new_file()?;

        Ok(())
    }

    /// 刷新缓冲区
    fn flush_buffers(&mut self) -> Result<(), ErrorCode> {
        // 创建剩余样本的数据包
        if !self.sample_buffer.is_empty() {
            self.create_data_packet()?;
        }

        // 写入剩余数据包
        if !self.packet_buffer.is_empty() {
            self.write_packets_to_file()?;
        }

        Ok(())
    }

    /// 获取统计信息
    pub fn get_statistics(&self) -> &LoggerStatistics {
        &self.statistics
    }

    /// 获取状态
    pub fn get_status(&self) -> LoggerStatus {
        self.status
    }

    /// 更新配置
    pub fn update_config(&mut self, config: LoggerConfig) -> Result<(), ErrorCode> {
        if matches!(self.status, LoggerStatus::Running) {
            return Err(ErrorCode::ConfigurationError);
        }

        self.config = config;
        Ok(())
    }
}

impl LoggerStatistics {
    pub fn new() -> Self {
        Self {
            samples_collected: 0,
            packets_written: 0,
            bytes_written: 0,
            write_errors: 0,
            buffer_overruns: 0,
            compression_ratio: 1.0,
            average_sample_rate: 0.0,
        }
    }

    /// 计算平均采样率
    pub fn calculate_sample_rate(&mut self, elapsed_time: f32) {
        if elapsed_time > 0.0 {
            self.average_sample_rate = self.samples_collected as f32 / elapsed_time;
        }
    }
}

impl DataCompressor {
    pub fn new(compression_type: CompressionType) -> Self {
        Self {
            compression_type,
            compression_buffer: HeaplessVec::new(),
        }
    }

    /// 压缩数据
    pub fn compress(&mut self, data: &[u8]) -> Result<&[u8], ErrorCode> {
        self.compression_buffer.clear();

        match self.compression_type {
            CompressionType::None => Ok(data),
            CompressionType::RunLength => self.run_length_encode(data),
            CompressionType::Delta => self.delta_encode(data),
            CompressionType::Huffman => {
                // 简化的霍夫曼编码实现
                Err(ErrorCode::CompressionError)
            }
        }
    }

    /// 行程长度编码
    fn run_length_encode(&mut self, data: &[u8]) -> Result<&[u8], ErrorCode> {
        if data.is_empty() {
            return Ok(&[]);
        }

        let mut i = 0;
        while i < data.len() {
            let current_byte = data[i];
            let mut count = 1u8;

            // 计算连续相同字节的数量
            while i + count as usize < data.len() && 
                  data[i + count as usize] == current_byte && 
                  count < 255 {
                count += 1;
            }

            // 添加到压缩缓冲区
            if self.compression_buffer.push(count).is_err() ||
               self.compression_buffer.push(current_byte).is_err() {
                return Err(ErrorCode::CompressionError);
            }

            i += count as usize;
        }

        Ok(&self.compression_buffer)
    }

    /// 差分编码
    fn delta_encode(&mut self, data: &[u8]) -> Result<&[u8], ErrorCode> {
        if data.is_empty() {
            return Ok(&[]);
        }

        // 第一个字节保持不变
        if self.compression_buffer.push(data[0]).is_err() {
            return Err(ErrorCode::CompressionError);
        }

        // 后续字节存储差值
        for i in 1..data.len() {
            let delta = data[i].wrapping_sub(data[i - 1]);
            if self.compression_buffer.push(delta).is_err() {
                return Err(ErrorCode::CompressionError);
            }
        }

        Ok(&self.compression_buffer)
    }
}

impl DataEncryptor {
    pub fn new(encryption_type: EncryptionType) -> Self {
        Self {
            encryption_type,
            key: [0; 32],
            iv: [0; 16],
        }
    }

    /// 设置加密密钥
    pub fn set_key(&mut self, key: &[u8]) {
        let len = key.len().min(32);
        self.key[..len].copy_from_slice(&key[..len]);
    }

    /// 设置初始化向量
    pub fn set_iv(&mut self, iv: &[u8]) {
        let len = iv.len().min(16);
        self.iv[..len].copy_from_slice(&iv[..len]);
    }

    /// 加密数据
    pub fn encrypt(&self, data: &[u8]) -> Result<Vec<u8>, ErrorCode> {
        match self.encryption_type {
            EncryptionType::None => Ok(data.to_vec()),
            EncryptionType::Aes128 | EncryptionType::Aes256 => {
                // 简化的AES加密实现
                // 实际应用中应使用专业的加密库
                let mut encrypted = data.to_vec();
                for (i, byte) in encrypted.iter_mut().enumerate() {
                    *byte ^= self.key[i % self.key.len()];
                }
                Ok(encrypted)
            }
        }
    }
}

#[entry]
fn main() -> ! {
    // 初始化堆分配器
    unsafe { ALLOCATOR.lock().init(HEAP.as_ptr() as usize, HEAP.len()) }

    // 初始化RTT日志
    rtt_target::rtt_init_print!();
    rtt_target::rprintln!("数据记录器系统启动");

    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟到168MHz
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

    // 启用DWT周期计数器
    let mut dwt = cp.DWT;
    let dcb = cp.DCB;
    dwt.enable_cycle_counter(&dcb);

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    // ADC输入引脚
    let adc_pin1 = gpioa.pa0.into_analog();
    let adc_pin2 = gpioa.pa1.into_analog();
    let adc_pin3 = gpioa.pa2.into_analog();

    // 状态LED
    let mut status_led = gpiod.pd12.into_push_pull_output();

    // 配置ADC
    let adc_config = AdcConfig::default()
        .resolution(Resolution::Twelve)
        .end_of_conversion_interrupt(Eoc::Conversion)
        .scan(Scan::Enabled);
    
    let adc1 = Adc::adc1(dp.ADC1, true, adc_config);

    // 配置SDIO
    let sdio = Sdio::new(
        dp.SDIO,
        (
            gpioc.pc8.into_alternate(),  // D0
            gpioc.pc9.into_alternate(),  // D1
            gpioc.pc10.into_alternate(), // D2
            gpioc.pc11.into_alternate(), // D3
            gpioc.pc12.into_alternate(), // CLK
            gpiod.pd2.into_alternate(),  // CMD
        ),
        ClockFreq::F25Mhz,
        &clocks,
    );

    // 创建记录器配置
    let mut config = LoggerConfig {
        sample_rate: 1000, // 1kHz
        channels: HeaplessVec::new(),
        buffer_size: 1024,
        file_rotation_size: 1024 * 1024, // 1MB
        compression_enabled: true,
        encryption_enabled: false,
        timestamp_enabled: true,
    };

    // 添加通道配置
    let channel1 = ChannelConfig {
        id: 0,
        name: HeaplessString::from("CH1"),
        enabled: true,
        gain: 3.3 / 4096.0,
        offset: 0.0,
        unit: HeaplessString::from("V"),
        min_value: 0.0,
        max_value: 3.3,
    };
    config.channels.push(channel1).ok();

    let channel2 = ChannelConfig {
        id: 1,
        name: HeaplessString::from("CH2"),
        enabled: true,
        gain: 3.3 / 4096.0,
        offset: 0.0,
        unit: HeaplessString::from("V"),
        min_value: 0.0,
        max_value: 3.3,
    };
    config.channels.push(channel2).ok();

    // 创建数据记录器
    let mut data_logger = DataLogger::new(config);

    // 初始化ADC通道
    if let Err(e) = data_logger.init_adc_channels(adc1) {
        rtt_target::rprintln!("ADC初始化失败: {:?}", e);
    }

    // 初始化文件系统
    if let Err(e) = data_logger.init_filesystem(sdio) {
        rtt_target::rprintln!("文件系统初始化失败: {:?}", e);
    }

    // 配置采样定时器
    let mut timer = Timer::new(dp.TIM2, &clocks);
    timer.start(Duration::<u32, 1, 1_000_000>::from_ticks(1000)); // 1kHz采样率
    timer.listen(Event::Update);

    rtt_target::rprintln!("数据记录器配置完成");

    // 开始数据记录
    if let Err(e) = data_logger.start_logging() {
        rtt_target::rprintln!("启动记录失败: {:?}", e);
    } else {
        rtt_target::rprintln!("数据记录已开始");
        status_led.set_high();
    }

    let mut sample_counter = 0u32;
    let mut last_report_time = DWT::cycle_count();

    // 主循环
    loop {
        // 模拟ADC采样
        let raw_value1 = (sample_counter % 4096) as u16;
        let raw_value2 = ((sample_counter * 2) % 4096) as u16;

        // 处理样本
        if let Err(e) = data_logger.process_sample(0, raw_value1) {
            rtt_target::rprintln!("处理样本失败: {:?}", e);
        }

        if let Err(e) = data_logger.process_sample(1, raw_value2) {
            rtt_target::rprintln!("处理样本失败: {:?}", e);
        }

        sample_counter += 1;

        // 每秒生成一次报告
        let current_time = DWT::cycle_count();
        let elapsed_cycles = current_time.wrapping_sub(last_report_time);
        let elapsed_seconds = elapsed_cycles as f32 / 168_000_000.0;

        if elapsed_seconds >= 5.0 {
            let stats = data_logger.get_statistics();
            let status = data_logger.get_status();

            rtt_target::rprintln!("=== 数据记录器状态报告 ===");
            rtt_target::rprintln!("状态: {:?}", status);
            rtt_target::rprintln!("已采集样本: {}", stats.samples_collected);
            rtt_target::rprintln!("已写入数据包: {}", stats.packets_written);
            rtt_target::rprintln!("已写入字节: {}", stats.bytes_written);
            rtt_target::rprintln!("写入错误: {}", stats.write_errors);
            rtt_target::rprintln!("缓冲区溢出: {}", stats.buffer_overruns);
            rtt_target::rprintln!("平均采样率: {:.1} Hz", stats.average_sample_rate);

            last_report_time = current_time;
        }

        // 短暂延时
        asm::delay(168_000); // 约1ms延时
    }
}