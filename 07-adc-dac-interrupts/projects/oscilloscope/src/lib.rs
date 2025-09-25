#![no_std]

use heapless::Vec;

/// 示波器触发类型
#[derive(Clone, Copy, Debug)]
pub enum TriggerType {
    None,           // 无触发
    Rising,         // 上升沿触发
    Falling,        // 下降沿触发
    Edge,           // 边沿触发
    Level,          // 电平触发
}

/// 示波器触发配置
#[derive(Clone, Copy, Debug)]
pub struct TriggerConfig {
    pub trigger_type: TriggerType,
    pub threshold: u16,         // 触发阈值
    pub hysteresis: u16,        // 滞回
    pub channel: u8,            // 触发通道
    pub pre_trigger: u16,       // 预触发采样数
    pub post_trigger: u16,      // 后触发采样数
}

impl Default for TriggerConfig {
    fn default() -> Self {
        Self {
            trigger_type: TriggerType::None,
            threshold: 2048,
            hysteresis: 50,
            channel: 0,
            pre_trigger: 100,
            post_trigger: 900,
        }
    }
}

/// 示波器时基配置
#[derive(Clone, Copy, Debug)]
pub struct TimebaseConfig {
    pub sample_rate: u32,       // 采样率 (Hz)
    pub time_per_div: u32,      // 时间/格 (us)
    pub horizontal_position: i16, // 水平位置偏移
    pub record_length: u16,     // 记录长度
}

impl Default for TimebaseConfig {
    fn default() -> Self {
        Self {
            sample_rate: 100000,    // 100kHz
            time_per_div: 1000,     // 1ms/div
            horizontal_position: 0,
            record_length: 1000,
        }
    }
}

/// 示波器通道配置
#[derive(Clone, Copy, Debug)]
pub struct ChannelConfig {
    pub enabled: bool,
    pub voltage_per_div: u16,   // 电压/格 (mV)
    pub vertical_position: i16, // 垂直位置偏移
    pub coupling: CouplingType, // 耦合方式
    pub probe_ratio: u8,        // 探头比例 (1x, 10x, 100x)
    pub bandwidth_limit: bool,  // 带宽限制
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            voltage_per_div: 1000,  // 1V/div
            vertical_position: 0,
            coupling: CouplingType::DC,
            probe_ratio: 1,
            bandwidth_limit: false,
        }
    }
}

/// 耦合类型
#[derive(Clone, Copy, Debug)]
pub enum CouplingType {
    DC,     // 直流耦合
    AC,     // 交流耦合
    Ground, // 接地
}

/// 示波器数据缓冲区
pub struct OscilloscopeBuffer<const N: usize> {
    pub data: Vec<u16, N>,
    pub timestamp: u32,
    pub trigger_position: Option<u16>,
    pub sample_rate: u32,
}

impl<const N: usize> OscilloscopeBuffer<N> {
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            timestamp: 0,
            trigger_position: None,
            sample_rate: 100000,
        }
    }

    pub fn add_sample(&mut self, sample: u16) -> bool {
        if self.data.len() < N {
            self.data.push(sample).is_ok()
        } else {
            false
        }
    }

    pub fn clear(&mut self) {
        self.data.clear();
        self.trigger_position = None;
    }

    pub fn is_full(&self) -> bool {
        self.data.len() >= N
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn get_sample(&self, index: usize) -> Option<u16> {
        self.data.get(index).copied()
    }
}

/// 多通道示波器缓冲区
pub struct MultiChannelBuffer<const CHANNELS: usize, const SAMPLES: usize> {
    pub channels: [OscilloscopeBuffer<SAMPLES>; CHANNELS],
    pub active_channels: u8,
    pub synchronized: bool,
}

impl<const CHANNELS: usize, const SAMPLES: usize> MultiChannelBuffer<CHANNELS, SAMPLES> {
    pub fn new() -> Self {
        Self {
            channels: [const { OscilloscopeBuffer::new() }; CHANNELS],
            active_channels: 0,
            synchronized: true,
        }
    }

    pub fn add_samples(&mut self, samples: &[u16]) -> bool {
        if samples.len() != CHANNELS {
            return false;
        }

        let mut success = true;
        for (i, &sample) in samples.iter().enumerate() {
            if i < CHANNELS && (self.active_channels & (1 << i)) != 0 {
                success &= self.channels[i].add_sample(sample);
            }
        }
        success
    }

    pub fn enable_channel(&mut self, channel: usize, enabled: bool) {
        if channel < CHANNELS {
            if enabled {
                self.active_channels |= 1 << channel;
            } else {
                self.active_channels &= !(1 << channel);
            }
        }
    }

    pub fn is_channel_enabled(&self, channel: usize) -> bool {
        channel < CHANNELS && (self.active_channels & (1 << channel)) != 0
    }

    pub fn clear_all(&mut self) {
        for channel in &mut self.channels {
            channel.clear();
        }
    }
}

/// 触发检测器
pub struct TriggerDetector {
    config: TriggerConfig,
    previous_sample: u16,
    trigger_state: TriggerState,
    pre_trigger_count: u16,
    post_trigger_count: u16,
}

#[derive(Clone, Copy, Debug)]
enum TriggerState {
    WaitingForTrigger,
    PreTrigger,
    PostTrigger,
    Complete,
}

impl TriggerDetector {
    pub fn new(config: TriggerConfig) -> Self {
        Self {
            config,
            previous_sample: 0,
            trigger_state: TriggerState::WaitingForTrigger,
            pre_trigger_count: 0,
            post_trigger_count: 0,
        }
    }

    pub fn update_config(&mut self, config: TriggerConfig) {
        self.config = config;
        self.reset();
    }

    pub fn reset(&mut self) {
        self.trigger_state = TriggerState::WaitingForTrigger;
        self.pre_trigger_count = 0;
        self.post_trigger_count = 0;
    }

    pub fn process_sample(&mut self, sample: u16) -> TriggerResult {
        match self.config.trigger_type {
            TriggerType::None => TriggerResult::Continue,
            _ => self.check_trigger_condition(sample),
        }
    }

    fn check_trigger_condition(&mut self, sample: u16) -> TriggerResult {
        let triggered = match self.config.trigger_type {
            TriggerType::Rising => {
                self.previous_sample < self.config.threshold.saturating_sub(self.config.hysteresis) &&
                sample > self.config.threshold.saturating_add(self.config.hysteresis)
            },
            TriggerType::Falling => {
                self.previous_sample > self.config.threshold.saturating_add(self.config.hysteresis) &&
                sample < self.config.threshold.saturating_sub(self.config.hysteresis)
            },
            TriggerType::Edge => {
                (self.previous_sample < self.config.threshold.saturating_sub(self.config.hysteresis) &&
                 sample > self.config.threshold.saturating_add(self.config.hysteresis)) ||
                (self.previous_sample > self.config.threshold.saturating_add(self.config.hysteresis) &&
                 sample < self.config.threshold.saturating_sub(self.config.hysteresis))
            },
            TriggerType::Level => {
                sample > self.config.threshold
            },
            TriggerType::None => false,
        };

        self.previous_sample = sample;

        match self.trigger_state {
            TriggerState::WaitingForTrigger => {
                if triggered {
                    self.trigger_state = TriggerState::PreTrigger;
                    self.pre_trigger_count = 0;
                    TriggerResult::Triggered
                } else {
                    TriggerResult::Continue
                }
            },
            TriggerState::PreTrigger => {
                self.pre_trigger_count += 1;
                if self.pre_trigger_count >= self.config.pre_trigger {
                    self.trigger_state = TriggerState::PostTrigger;
                    self.post_trigger_count = 0;
                }
                TriggerResult::Continue
            },
            TriggerState::PostTrigger => {
                self.post_trigger_count += 1;
                if self.post_trigger_count >= self.config.post_trigger {
                    self.trigger_state = TriggerState::Complete;
                    TriggerResult::Complete
                } else {
                    TriggerResult::Continue
                }
            },
            TriggerState::Complete => TriggerResult::Complete,
        }
    }

    pub fn is_complete(&self) -> bool {
        matches!(self.trigger_state, TriggerState::Complete)
    }

    pub fn get_trigger_position(&self) -> Option<u16> {
        match self.trigger_state {
            TriggerState::PostTrigger | TriggerState::Complete => {
                Some(self.config.pre_trigger)
            },
            _ => None,
        }
    }
}

/// 触发结果
#[derive(Clone, Copy, Debug)]
pub enum TriggerResult {
    Continue,   // 继续采样
    Triggered,  // 触发事件发生
    Complete,   // 采样完成
}

/// 示波器测量功能
pub struct OscilloscopeMeasurement;

impl OscilloscopeMeasurement {
    /// 计算峰峰值
    pub fn peak_to_peak(data: &[u16]) -> u16 {
        if data.is_empty() {
            return 0;
        }
        let max = data.iter().max().unwrap_or(&0);
        let min = data.iter().min().unwrap_or(&0);
        max.saturating_sub(*min)
    }

    /// 计算平均值
    pub fn average(data: &[u16]) -> u16 {
        if data.is_empty() {
            return 0;
        }
        let sum: u32 = data.iter().map(|&x| x as u32).sum();
        (sum / data.len() as u32) as u16
    }

    /// 计算RMS值
    pub fn rms(data: &[u16]) -> u16 {
        if data.is_empty() {
            return 0;
        }
        let avg = Self::average(data) as i32;
        let sum_squares: u32 = data.iter()
            .map(|&x| {
                let diff = x as i32 - avg;
                (diff * diff) as u32
            })
            .sum();
        ((sum_squares / data.len() as u32) as f32).sqrt() as u16
    }

    /// 测量频率
    pub fn frequency(data: &[u16], sample_rate: u32) -> Option<u32> {
        if data.len() < 3 {
            return None;
        }

        let avg = Self::average(data);
        let mut zero_crossings = 0;
        let mut last_above = data[0] > avg;

        for &sample in data.iter().skip(1) {
            let current_above = sample > avg;
            if current_above != last_above {
                zero_crossings += 1;
            }
            last_above = current_above;
        }

        if zero_crossings < 2 {
            return None;
        }

        // 频率 = (过零点数 / 2) / (总时间)
        let total_time = data.len() as f32 / sample_rate as f32;
        let frequency = (zero_crossings as f32 / 2.0) / total_time;
        Some(frequency as u32)
    }

    /// 测量占空比
    pub fn duty_cycle(data: &[u16]) -> u8 {
        if data.is_empty() {
            return 0;
        }

        let avg = Self::average(data);
        let high_samples = data.iter().filter(|&&x| x > avg).count();
        ((high_samples * 100) / data.len()) as u8
    }

    /// 测量上升时间 (10% 到 90%)
    pub fn rise_time(data: &[u16], sample_rate: u32) -> Option<u32> {
        if data.len() < 10 {
            return None;
        }

        let max_val = *data.iter().max()?;
        let min_val = *data.iter().min()?;
        let range = max_val.saturating_sub(min_val);
        
        let threshold_10 = min_val + range / 10;
        let threshold_90 = min_val + (range * 9) / 10;

        let mut start_idx = None;
        let mut end_idx = None;

        for (i, &sample) in data.iter().enumerate() {
            if start_idx.is_none() && sample >= threshold_10 {
                start_idx = Some(i);
            }
            if start_idx.is_some() && sample >= threshold_90 {
                end_idx = Some(i);
                break;
            }
        }

        if let (Some(start), Some(end)) = (start_idx, end_idx) {
            let time_diff = (end - start) as f32 / sample_rate as f32;
            Some((time_diff * 1_000_000.0) as u32) // 返回微秒
        } else {
            None
        }
    }
}

/// 示波器显示缓冲区
pub struct DisplayBuffer<const WIDTH: usize, const HEIGHT: usize> {
    pub pixels: [[bool; WIDTH]; HEIGHT],
    pub grid_enabled: bool,
    pub cursor_x: usize,
    pub cursor_y: usize,
}

impl<const WIDTH: usize, const HEIGHT: usize> DisplayBuffer<WIDTH, HEIGHT> {
    pub fn new() -> Self {
        Self {
            pixels: [[false; WIDTH]; HEIGHT],
            grid_enabled: true,
            cursor_x: WIDTH / 2,
            cursor_y: HEIGHT / 2,
        }
    }

    pub fn clear(&mut self) {
        for row in &mut self.pixels {
            for pixel in row {
                *pixel = false;
            }
        }
    }

    pub fn draw_waveform(&mut self, data: &[u16], channel: usize) {
        if data.is_empty() {
            return;
        }

        let x_scale = WIDTH as f32 / data.len() as f32;
        let y_scale = HEIGHT as f32 / 4096.0; // 12-bit ADC

        for (i, &sample) in data.iter().enumerate() {
            let x = (i as f32 * x_scale) as usize;
            let y = HEIGHT - 1 - ((sample as f32 * y_scale) as usize).min(HEIGHT - 1);
            
            if x < WIDTH && y < HEIGHT {
                self.pixels[y][x] = true;
            }
        }
    }

    pub fn draw_grid(&mut self) {
        if !self.grid_enabled {
            return;
        }

        // 绘制垂直网格线
        for x in (0..WIDTH).step_by(WIDTH / 10) {
            for y in 0..HEIGHT {
                if y % 4 == 0 {
                    self.pixels[y][x] = true;
                }
            }
        }

        // 绘制水平网格线
        for y in (0..HEIGHT).step_by(HEIGHT / 8) {
            for x in 0..WIDTH {
                if x % 4 == 0 {
                    self.pixels[y][x] = true;
                }
            }
        }
    }

    pub fn draw_cursor(&mut self) {
        // 绘制十字光标
        if self.cursor_x < WIDTH {
            for y in 0..HEIGHT {
                self.pixels[y][self.cursor_x] = true;
            }
        }
        
        if self.cursor_y < HEIGHT {
            for x in 0..WIDTH {
                self.pixels[self.cursor_y][x] = true;
            }
        }
    }

    pub fn set_pixel(&mut self, x: usize, y: usize, value: bool) {
        if x < WIDTH && y < HEIGHT {
            self.pixels[y][x] = value;
        }
    }

    pub fn get_pixel(&self, x: usize, y: usize) -> bool {
        if x < WIDTH && y < HEIGHT {
            self.pixels[y][x]
        } else {
            false
        }
    }
}

/// ADC电压转换
pub fn adc_to_voltage(adc_value: u16, vref: u16) -> u16 {
    ((adc_value as u32 * vref as u32) / 4095) as u16
}

/// 电压转ADC值
pub fn voltage_to_adc(voltage: u16, vref: u16) -> u16 {
    ((voltage as u32 * 4095) / vref as u32) as u16
}