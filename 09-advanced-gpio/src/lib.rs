#![no_std]
#![deny(unsafe_code)]
#![warn(missing_docs)]

//! # 高级GPIO技术库
//! 
//! 本库提供嵌入式系统中的高级GPIO控制技术，包括：
//! - GPIO扩展器控制
//! - 引脚多路复用管理
//! - 信号调理和滤波
//! - 功耗管理优化
//! - 高速IO操作
//! - 模拟开关控制

use heapless::{Vec, FnvIndexMap};
use critical_section::Mutex;
use core::cell::RefCell;
use embedded_hal::digital::{InputPin, OutputPin};

/// GPIO扩展器特征
pub trait GpioExpander {
    /// 错误类型
    type Error;
    
    /// 设置引脚方向
    fn set_direction(&mut self, pin: u8, output: bool) -> Result<(), Self::Error>;
    
    /// 设置输出状态
    fn set_output(&mut self, pin: u8, high: bool) -> Result<(), Self::Error>;
    
    /// 读取输入状态
    fn read_input(&mut self, pin: u8) -> Result<bool, Self::Error>;
    
    /// 批量设置输出
    fn set_outputs(&mut self, mask: u16, values: u16) -> Result<(), Self::Error>;
    
    /// 批量读取输入
    fn read_inputs(&mut self) -> Result<u16, Self::Error>;
}

/// 引脚多路复用器特征
pub trait PinMultiplexer {
    /// 错误类型
    type Error;
    
    /// 选择通道
    fn select_channel(&mut self, channel: u8) -> Result<(), Self::Error>;
    
    /// 获取当前通道
    fn current_channel(&self) -> u8;
    
    /// 获取通道数量
    fn channel_count(&self) -> u8;
    
    /// 禁用所有通道
    fn disable_all(&mut self) -> Result<(), Self::Error>;
}

/// 信号调理器特征
pub trait SignalConditioner {
    /// 错误类型
    type Error;
    
    /// 设置增益
    fn set_gain(&mut self, gain: f32) -> Result<(), Self::Error>;
    
    /// 设置偏移
    fn set_offset(&mut self, offset: f32) -> Result<(), Self::Error>;
    
    /// 启用滤波
    fn enable_filter(&mut self, enable: bool) -> Result<(), Self::Error>;
    
    /// 设置滤波参数
    fn set_filter_params(&mut self, cutoff: f32, order: u8) -> Result<(), Self::Error>;
}

/// GPIO扩展器类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExpanderType {
    /// I2C接口扩展器
    I2C,
    /// SPI接口扩展器
    SPI,
    /// 移位寄存器
    ShiftRegister,
    /// 并行扩展器
    Parallel,
}

/// 引脚配置
#[derive(Debug, Clone, Copy)]
pub struct PinConfig {
    /// 引脚方向（true为输出）
    pub direction: bool,
    /// 上拉使能
    pub pull_up: bool,
    /// 下拉使能
    pub pull_down: bool,
    /// 开漏输出
    pub open_drain: bool,
    /// 驱动强度
    pub drive_strength: DriveStrength,
    /// 转换速率
    pub slew_rate: SlewRate,
}

/// 驱动强度
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriveStrength {
    /// 低驱动强度 (2mA)
    Low,
    /// 中等驱动强度 (4mA)
    Medium,
    /// 高驱动强度 (8mA)
    High,
    /// 最高驱动强度 (12mA)
    Maximum,
}

/// 转换速率
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SlewRate {
    /// 慢速转换
    Slow,
    /// 中速转换
    Medium,
    /// 快速转换
    Fast,
    /// 最快转换
    Maximum,
}

/// I2C GPIO扩展器
pub struct I2cGpioExpander<I2C> {
    /// I2C接口
    i2c: I2C,
    /// 设备地址
    address: u8,
    /// 引脚配置
    pin_configs: [PinConfig; 16],
    /// 输出状态缓存
    output_cache: u16,
    /// 方向寄存器缓存
    direction_cache: u16,
    /// 统计信息
    stats: ExpanderStats,
}

/// SPI GPIO扩展器
pub struct SpiGpioExpander<SPI, CS> {
    /// SPI接口
    spi: SPI,
    /// 片选引脚
    cs: CS,
    /// 引脚配置
    pin_configs: [PinConfig; 16],
    /// 输出状态缓存
    output_cache: u16,
    /// 方向寄存器缓存
    direction_cache: u16,
    /// 统计信息
    stats: ExpanderStats,
}

/// 移位寄存器GPIO扩展器
pub struct ShiftRegisterExpander<DATA, CLK, LATCH> {
    /// 数据引脚
    data: DATA,
    /// 时钟引脚
    clock: CLK,
    /// 锁存引脚
    latch: LATCH,
    /// 寄存器数量
    register_count: u8,
    /// 输出状态缓存
    output_cache: Vec<u8, 8>,
    /// 统计信息
    stats: ExpanderStats,
}

/// 引脚多路复用器
pub struct PinMux<SEL0, SEL1, SEL2, EN> {
    /// 选择引脚0
    sel0: SEL0,
    /// 选择引脚1
    sel1: Option<SEL1>,
    /// 选择引脚2
    sel2: Option<SEL2>,
    /// 使能引脚
    enable: Option<EN>,
    /// 当前通道
    current_channel: u8,
    /// 通道数量
    channel_count: u8,
    /// 通道映射
    channel_mapping: FnvIndexMap<u8, ChannelConfig, 8>,
    /// 统计信息
    stats: MuxStats,
}

/// 通道配置
#[derive(Debug, Clone, Copy)]
pub struct ChannelConfig {
    /// 选择值
    pub select_value: u8,
    /// 是否启用
    pub enabled: bool,
    /// 延迟时间（微秒）
    pub switch_delay_us: u16,
}

/// 模拟开关控制器
pub struct AnalogSwitchController<CTRL> {
    /// 控制引脚
    control_pins: Vec<CTRL, 8>,
    /// 开关配置
    switch_configs: Vec<SwitchConfig, 16>,
    /// 当前状态
    current_state: u16,
    /// 开关矩阵
    switch_matrix: SwitchMatrix,
    /// 统计信息
    stats: SwitchStats,
}

/// 开关配置
#[derive(Debug, Clone, Copy)]
pub struct SwitchConfig {
    /// 开关ID
    pub id: u8,
    /// 控制引脚掩码
    pub control_mask: u8,
    /// 开关类型
    pub switch_type: SwitchType,
    /// 导通电阻
    pub on_resistance: f32,
    /// 开关时间
    pub switch_time_us: u16,
}

/// 开关类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SwitchType {
    /// 单刀单掷
    SPST,
    /// 单刀双掷
    SPDT,
    /// 双刀单掷
    DPST,
    /// 双刀双掷
    DPDT,
}

/// 开关矩阵
pub struct SwitchMatrix {
    /// 矩阵尺寸
    pub rows: u8,
    pub cols: u8,
    /// 连接状态
    pub connections: [[bool; 8]; 8],
    /// 交叉点开关
    pub crosspoints: Vec<CrossPoint, 64>,
}

/// 交叉点开关
#[derive(Debug, Clone, Copy)]
pub struct CrossPoint {
    /// 行索引
    pub row: u8,
    /// 列索引
    pub col: u8,
    /// 连接状态
    pub connected: bool,
    /// 控制位
    pub control_bit: u8,
}

/// 信号调理器
pub struct SignalConditioner<AMP, FILTER> {
    /// 放大器控制
    amplifier: AMP,
    /// 滤波器控制
    filter: FILTER,
    /// 当前增益
    current_gain: f32,
    /// 当前偏移
    current_offset: f32,
    /// 滤波器参数
    filter_params: FilterParams,
    /// 校准数据
    calibration: CalibrationData,
    /// 统计信息
    stats: ConditionerStats,
}

/// 滤波器参数
#[derive(Debug, Clone, Copy)]
pub struct FilterParams {
    /// 滤波器类型
    pub filter_type: FilterType,
    /// 截止频率
    pub cutoff_frequency: f32,
    /// 滤波器阶数
    pub order: u8,
    /// 品质因子
    pub q_factor: f32,
}

/// 滤波器类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FilterType {
    /// 低通滤波器
    LowPass,
    /// 高通滤波器
    HighPass,
    /// 带通滤波器
    BandPass,
    /// 带阻滤波器
    BandStop,
}

/// 校准数据
#[derive(Debug, Clone, Copy)]
pub struct CalibrationData {
    /// 零点偏移
    pub zero_offset: f32,
    /// 增益校正
    pub gain_correction: f32,
    /// 线性度校正
    pub linearity_correction: [f32; 8],
    /// 温度系数
    pub temperature_coefficient: f32,
}

/// 功耗管理器
pub struct PowerManager<CTRL> {
    /// 电源控制引脚
    power_controls: Vec<CTRL, 8>,
    /// 电源域配置
    power_domains: Vec<PowerDomain, 8>,
    /// 当前状态
    current_state: PowerState,
    /// 功耗监控
    power_monitor: PowerMonitor,
    /// 统计信息
    stats: PowerStats,
}

/// 电源域
#[derive(Debug, Clone, Copy)]
pub struct PowerDomain {
    /// 域ID
    pub id: u8,
    /// 控制引脚
    pub control_pin: u8,
    /// 电压等级
    pub voltage_level: f32,
    /// 最大电流
    pub max_current: f32,
    /// 启动时间
    pub startup_time_us: u16,
    /// 关断时间
    pub shutdown_time_us: u16,
}

/// 电源状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PowerState {
    /// 全功率模式
    FullPower,
    /// 低功耗模式
    LowPower,
    /// 睡眠模式
    Sleep,
    /// 深度睡眠模式
    DeepSleep,
    /// 关机模式
    Shutdown,
}

/// 功耗监控器
#[derive(Debug, Clone, Copy)]
pub struct PowerMonitor {
    /// 当前功耗
    pub current_power: f32,
    /// 平均功耗
    pub average_power: f32,
    /// 峰值功耗
    pub peak_power: f32,
    /// 总能耗
    pub total_energy: f32,
    /// 监控时间
    pub monitor_time: u32,
}

/// 高速IO控制器
pub struct HighSpeedIoController<IO> {
    /// IO引脚
    io_pins: Vec<IO, 32>,
    /// 时序配置
    timing_config: TimingConfig,
    /// 信号完整性配置
    signal_integrity: SignalIntegrityConfig,
    /// DMA配置
    dma_config: Option<DmaConfig>,
    /// 统计信息
    stats: HighSpeedStats,
}

/// 时序配置
#[derive(Debug, Clone, Copy)]
pub struct TimingConfig {
    /// 建立时间
    pub setup_time_ns: u16,
    /// 保持时间
    pub hold_time_ns: u16,
    /// 传播延迟
    pub propagation_delay_ns: u16,
    /// 时钟频率
    pub clock_frequency: u32,
    /// 占空比
    pub duty_cycle: u8,
}

/// 信号完整性配置
#[derive(Debug, Clone, Copy)]
pub struct SignalIntegrityConfig {
    /// 阻抗匹配
    pub impedance_matching: bool,
    /// 终端电阻
    pub termination_resistance: f32,
    /// 差分信号
    pub differential_signaling: bool,
    /// 预加重
    pub pre_emphasis: bool,
    /// 均衡
    pub equalization: bool,
}

/// DMA配置
#[derive(Debug, Clone, Copy)]
pub struct DmaConfig {
    /// DMA通道
    pub channel: u8,
    /// 传输模式
    pub transfer_mode: DmaTransferMode,
    /// 缓冲区大小
    pub buffer_size: u16,
    /// 优先级
    pub priority: DmaPriority,
}

/// DMA传输模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaTransferMode {
    /// 单次传输
    Single,
    /// 突发传输
    Burst,
    /// 循环传输
    Circular,
    /// 双缓冲
    DoubleBuffer,
}

/// DMA优先级
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaPriority {
    Low,
    Medium,
    High,
    VeryHigh,
}

/// 扩展器统计
#[derive(Debug, Default)]
pub struct ExpanderStats {
    pub read_operations: u32,
    pub write_operations: u32,
    pub direction_changes: u32,
    pub batch_operations: u32,
    pub error_count: u32,
}

/// 多路复用器统计
#[derive(Debug, Default)]
pub struct MuxStats {
    pub channel_switches: u32,
    pub switch_time_total: u32,
    pub error_count: u32,
}

/// 开关统计
#[derive(Debug, Default)]
pub struct SwitchStats {
    pub switch_operations: u32,
    pub matrix_reconfigurations: u32,
    pub connection_count: u32,
    pub error_count: u32,
}

/// 调理器统计
#[derive(Debug, Default)]
pub struct ConditionerStats {
    pub gain_adjustments: u32,
    pub offset_adjustments: u32,
    pub filter_reconfigurations: u32,
    pub calibration_updates: u32,
}

/// 功耗统计
#[derive(Debug, Default)]
pub struct PowerStats {
    pub state_transitions: u32,
    pub power_on_time: u32,
    pub power_off_time: u32,
    pub total_energy_consumed: f32,
}

/// 高速IO统计
#[derive(Debug, Default)]
pub struct HighSpeedStats {
    pub data_transfers: u32,
    pub timing_violations: u32,
    pub signal_integrity_issues: u32,
    pub dma_transfers: u32,
}

// 实现默认配置
impl Default for PinConfig {
    fn default() -> Self {
        Self {
            direction: false,
            pull_up: false,
            pull_down: false,
            open_drain: false,
            drive_strength: DriveStrength::Medium,
            slew_rate: SlewRate::Medium,
        }
    }
}

impl Default for FilterParams {
    fn default() -> Self {
        Self {
            filter_type: FilterType::LowPass,
            cutoff_frequency: 1000.0,
            order: 2,
            q_factor: 0.707,
        }
    }
}

impl Default for CalibrationData {
    fn default() -> Self {
        Self {
            zero_offset: 0.0,
            gain_correction: 1.0,
            linearity_correction: [0.0; 8],
            temperature_coefficient: 0.0,
        }
    }
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            setup_time_ns: 10,
            hold_time_ns: 10,
            propagation_delay_ns: 5,
            clock_frequency: 1_000_000,
            duty_cycle: 50,
        }
    }
}

impl Default for SignalIntegrityConfig {
    fn default() -> Self {
        Self {
            impedance_matching: false,
            termination_resistance: 50.0,
            differential_signaling: false,
            pre_emphasis: false,
            equalization: false,
        }
    }
}

// 基础测试
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_pin_config_default() {
        let config = PinConfig::default();
        assert_eq!(config.direction, false);
        assert_eq!(config.drive_strength, DriveStrength::Medium);
        assert_eq!(config.slew_rate, SlewRate::Medium);
    }
    
    #[test]
    fn test_filter_params_default() {
        let params = FilterParams::default();
        assert_eq!(params.filter_type, FilterType::LowPass);
        assert_eq!(params.cutoff_frequency, 1000.0);
        assert_eq!(params.order, 2);
    }
    
    #[test]
    fn test_power_state_transitions() {
        let states = [
            PowerState::FullPower,
            PowerState::LowPower,
            PowerState::Sleep,
            PowerState::DeepSleep,
            PowerState::Shutdown,
        ];
        
        for state in &states {
            match state {
                PowerState::FullPower => assert_eq!(*state, PowerState::FullPower),
                PowerState::LowPower => assert_eq!(*state, PowerState::LowPower),
                PowerState::Sleep => assert_eq!(*state, PowerState::Sleep),
                PowerState::DeepSleep => assert_eq!(*state, PowerState::DeepSleep),
                PowerState::Shutdown => assert_eq!(*state, PowerState::Shutdown),
            }
        }
    }
    
    #[test]
    fn test_switch_types() {
        let types = [
            SwitchType::SPST,
            SwitchType::SPDT,
            SwitchType::DPST,
            SwitchType::DPDT,
        ];
        
        for switch_type in &types {
            match switch_type {
                SwitchType::SPST => assert_eq!(*switch_type, SwitchType::SPST),
                SwitchType::SPDT => assert_eq!(*switch_type, SwitchType::SPDT),
                SwitchType::DPST => assert_eq!(*switch_type, SwitchType::DPST),
                SwitchType::DPDT => assert_eq!(*switch_type, SwitchType::DPDT),
            }
        }
    }
    
    #[test]
    fn test_dma_priorities() {
        let priorities = [
            DmaPriority::Low,
            DmaPriority::Medium,
            DmaPriority::High,
            DmaPriority::VeryHigh,
        ];
        
        for priority in &priorities {
            match priority {
                DmaPriority::Low => assert_eq!(*priority, DmaPriority::Low),
                DmaPriority::Medium => assert_eq!(*priority, DmaPriority::Medium),
                DmaPriority::High => assert_eq!(*priority, DmaPriority::High),
                DmaPriority::VeryHigh => assert_eq!(*priority, DmaPriority::VeryHigh),
            }
        }
    }
}