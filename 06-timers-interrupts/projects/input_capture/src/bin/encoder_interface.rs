#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, PushPull, Pin, Input, PullUp},
    timer::{Timer, Event, CounterUs},
    rcc::Clocks,
    interrupt,
};
use heapless::spsc::{Queue, Producer, Consumer};
use heapless::{Vec, Deque};
use rtt_target::{rprintln, rtt_init_print};
use input_capture::{CaptureEvent, EdgeType, CaptureMode};
use libm::{fabsf, sqrtf, fminf, fmaxf, atan2f};
use cortex_m::peripheral::NVIC;

// 编码器事件类型
#[derive(Debug, Clone, Copy)]
enum EncoderEvent {
    PositionUpdate(i32),     // 位置更新
    VelocityUpdate(f32),     // 速度更新
    DirectionChange(Direction), // 方向改变
    IndexPulse(u32),         // 索引脉冲
    CalibrationRequest,      // 校准请求
    ModeChange(EncoderMode), // 模式切换
    ErrorDetected(EncoderError), // 错误检测
}

#[derive(Debug, Clone, Copy)]
enum EncoderMode {
    Quadrature,    // 正交编码器
    Incremental,   // 增量编码器
    Absolute,      // 绝对编码器
    Velocity,      // 速度测量
    Position,      // 位置测量
    Differential,  // 差分编码器
}

#[derive(Debug, Clone, Copy)]
enum Direction {
    Clockwise,        // 顺时针
    CounterClockwise, // 逆时针
    Stopped,          // 停止
}

#[derive(Debug, Clone, Copy)]
enum EncoderError {
    MissedPulse,      // 丢失脉冲
    PhaseError,       // 相位错误
    IndexMismatch,    // 索引不匹配
    VelocityOverflow, // 速度溢出
    PositionOverflow, // 位置溢出
    CalibrationFailed, // 校准失败
}

// 正交编码器解码器
struct QuadratureDecoder {
    last_a_state: bool,
    last_b_state: bool,
    position: i32,
    direction: Direction,
    pulse_count: u32,
    error_count: u32,
    resolution: u32, // 每转脉冲数
}

impl QuadratureDecoder {
    fn new(resolution: u32) -> Self {
        Self {
            last_a_state: false,
            last_b_state: false,
            position: 0,
            direction: Direction::Stopped,
            pulse_count: 0,
            error_count: 0,
            resolution,
        }
    }

    fn process_signals(&mut self, a_state: bool, b_state: bool) -> Result<Option<i32>, EncoderError> {
        let a_changed = a_state != self.last_a_state;
        let b_changed = b_state != self.last_b_state;

        if !a_changed && !b_changed {
            return Ok(None); // 无变化
        }

        // 正交编码器状态机
        let old_state = (self.last_a_state as u8) << 1 | (self.last_b_state as u8);
        let new_state = (a_state as u8) << 1 | (b_state as u8);

        let delta = match (old_state, new_state) {
            // 正向序列: 00 -> 01 -> 11 -> 10 -> 00
            (0b00, 0b01) | (0b01, 0b11) | (0b11, 0b10) | (0b10, 0b00) => {
                self.direction = Direction::Clockwise;
                1
            }
            // 反向序列: 00 -> 10 -> 11 -> 01 -> 00
            (0b00, 0b10) | (0b10, 0b11) | (0b11, 0b01) | (0b01, 0b00) => {
                self.direction = Direction::CounterClockwise;
                -1
            }
            // 无效转换
            _ => {
                self.error_count += 1;
                return Err(EncoderError::PhaseError);
            }
        };

        self.position = self.position.wrapping_add(delta);
        self.pulse_count += 1;
        self.last_a_state = a_state;
        self.last_b_state = b_state;

        Ok(Some(self.position))
    }

    fn get_angle_degrees(&self) -> f32 {
        (self.position as f32) / (self.resolution as f32) * 360.0
    }

    fn get_revolutions(&self) -> f32 {
        (self.position as f32) / (self.resolution as f32)
    }

    fn reset_position(&mut self) {
        self.position = 0;
    }

    fn set_position(&mut self, position: i32) {
        self.position = position;
    }
}

// 速度计算器
struct VelocityCalculator {
    position_history: Deque<(u32, i32), 32>, // (时间戳, 位置)
    velocity_buffer: Deque<f32, 16>,
    last_velocity: f32,
    timer_frequency: u32,
    filter_alpha: f32, // 低通滤波器系数
}

impl VelocityCalculator {
    fn new(timer_frequency: u32) -> Self {
        Self {
            position_history: Deque::new(),
            velocity_buffer: Deque::new(),
            last_velocity: 0.0,
            timer_frequency,
            filter_alpha: 0.1, // 10% 新值，90% 旧值
        }
    }

    fn update(&mut self, timestamp: u32, position: i32) -> f32 {
        // 添加新的位置记录
        if self.position_history.len() >= 32 {
            self.position_history.pop_front();
        }
        let _ = self.position_history.push_back((timestamp, position));

        // 计算速度（需要至少两个点）
        if self.position_history.len() < 2 {
            return 0.0;
        }

        // 使用最近的两个点计算瞬时速度
        let (t2, p2) = self.position_history[self.position_history.len() - 1];
        let (t1, p1) = self.position_history[self.position_history.len() - 2];

        let dt = t2.wrapping_sub(t1) as f32 / self.timer_frequency as f32;
        let dp = (p2 - p1) as f32;

        if dt > 0.0 {
            let raw_velocity = dp / dt; // 脉冲/秒
            
            // 低通滤波
            self.last_velocity = self.filter_alpha * raw_velocity + (1.0 - self.filter_alpha) * self.last_velocity;
            
            // 添加到速度缓冲区
            if self.velocity_buffer.len() >= 16 {
                self.velocity_buffer.pop_front();
            }
            let _ = self.velocity_buffer.push_back(self.last_velocity);
        }

        self.last_velocity
    }

    fn get_rpm(&self, resolution: u32) -> f32 {
        self.last_velocity / resolution as f32 * 60.0
    }

    fn get_average_velocity(&self) -> f32 {
        if self.velocity_buffer.is_empty() {
            return 0.0;
        }

        let sum: f32 = self.velocity_buffer.iter().sum();
        sum / self.velocity_buffer.len() as f32
    }

    fn get_velocity_variance(&self) -> f32 {
        if self.velocity_buffer.len() < 2 {
            return 0.0;
        }

        let avg = self.get_average_velocity();
        let variance_sum: f32 = self.velocity_buffer.iter()
            .map(|&v| (v - avg) * (v - avg))
            .sum();
        
        variance_sum / (self.velocity_buffer.len() - 1) as f32
    }
}

// 编码器接口管理器
struct EncoderInterfaceManager {
    decoder: QuadratureDecoder,
    velocity_calc: VelocityCalculator,
    current_mode: EncoderMode,
    measurement_active: bool,
    calibration_active: bool,
    index_position: Option<i32>,
    last_index_timestamp: u32,
    statistics: EncoderStatistics,
    event_producer: Producer<'static, EncoderEvent, 64>,
}

#[derive(Debug, Default)]
struct EncoderStatistics {
    total_pulses: u32,
    total_revolutions: f32,
    max_velocity: f32,
    min_velocity: f32,
    avg_velocity: f32,
    position_updates: u32,
    direction_changes: u32,
    index_pulses: u32,
    error_count: u32,
    calibration_count: u32,
    measurement_time_ms: u32,
}

impl EncoderInterfaceManager {
    fn new(
        resolution: u32,
        timer_frequency: u32,
        event_producer: Producer<'static, EncoderEvent, 64>,
    ) -> Self {
        Self {
            decoder: QuadratureDecoder::new(resolution),
            velocity_calc: VelocityCalculator::new(timer_frequency),
            current_mode: EncoderMode::Quadrature,
            measurement_active: false,
            calibration_active: false,
            index_position: None,
            last_index_timestamp: 0,
            statistics: EncoderStatistics::default(),
            event_producer,
        }
    }

    fn set_mode(&mut self, mode: EncoderMode) {
        self.current_mode = mode;
        rprintln!("编码器模式切换到: {:?}", mode);
    }

    fn start_measurement(&mut self) {
        self.measurement_active = true;
        rprintln!("开始编码器测量");
    }

    fn stop_measurement(&mut self) {
        self.measurement_active = false;
        rprintln!("停止编码器测量");
    }

    fn process_quadrature_signals(&mut self, timestamp: u32, a_state: bool, b_state: bool) -> Result<(), EncoderError> {
        if !self.measurement_active {
            return Ok(());
        }

        // 处理正交信号
        match self.decoder.process_signals(a_state, b_state) {
            Ok(Some(new_position)) => {
                self.statistics.position_updates += 1;
                self.statistics.total_pulses += 1;

                // 检查方向变化
                let old_direction = self.decoder.direction;
                if old_direction != self.decoder.direction {
                    self.statistics.direction_changes += 1;
                    self.send_event(EncoderEvent::DirectionChange(self.decoder.direction));
                }

                // 更新速度
                let velocity = self.velocity_calc.update(timestamp, new_position);
                
                // 更新统计信息
                if velocity > self.statistics.max_velocity {
                    self.statistics.max_velocity = velocity;
                }
                if velocity < self.statistics.min_velocity || self.statistics.min_velocity == 0.0 {
                    self.statistics.min_velocity = velocity;
                }
                self.statistics.avg_velocity = self.velocity_calc.get_average_velocity();

                // 根据模式处理
                match self.current_mode {
                    EncoderMode::Quadrature => {
                        self.send_event(EncoderEvent::PositionUpdate(new_position));
                        if self.statistics.position_updates % 10 == 0 {
                            rprintln!("位置: {}, 角度: {:.1}°, 转数: {:.3}", 
                                     new_position, self.decoder.get_angle_degrees(), self.decoder.get_revolutions());
                        }
                    }
                    EncoderMode::Velocity => {
                        self.send_event(EncoderEvent::VelocityUpdate(velocity));
                        if self.statistics.position_updates % 20 == 0 {
                            rprintln!("速度: {:.2} 脉冲/秒, {:.1} RPM", 
                                     velocity, self.velocity_calc.get_rpm(self.decoder.resolution));
                        }
                    }
                    EncoderMode::Position => {
                        if self.statistics.position_updates % 5 == 0 {
                            rprintln!("精确位置: {}, 角度: {:.2}°", 
                                     new_position, self.decoder.get_angle_degrees());
                        }
                    }
                    _ => {
                        // 其他模式的处理
                    }
                }
            }
            Ok(None) => {
                // 无位置变化
            }
            Err(error) => {
                self.statistics.error_count += 1;
                self.send_event(EncoderEvent::ErrorDetected(error));
                return Err(error);
            }
        }

        Ok(())
    }

    fn process_index_pulse(&mut self, timestamp: u32) -> Result<(), &'static str> {
        if !self.measurement_active {
            return Ok(());
        }

        self.statistics.index_pulses += 1;
        self.last_index_timestamp = timestamp;

        // 记录索引位置
        self.index_position = Some(self.decoder.position);
        
        // 计算转数
        if let Some(index_pos) = self.index_position {
            let revolutions = index_pos as f32 / self.decoder.resolution as f32;
            self.statistics.total_revolutions = revolutions;
        }

        self.send_event(EncoderEvent::IndexPulse(timestamp));
        rprintln!("索引脉冲检测 - 位置: {}, 转数: {:.3}", 
                 self.decoder.position, self.statistics.total_revolutions);

        Ok(())
    }

    fn start_calibration(&mut self) {
        self.calibration_active = true;
        self.statistics.calibration_count += 1;
        
        // 重置位置到索引位置
        if let Some(index_pos) = self.index_position {
            self.decoder.set_position(index_pos);
        } else {
            self.decoder.reset_position();
        }
        
        rprintln!("开始编码器校准");
    }

    fn end_calibration(&mut self) {
        self.calibration_active = false;
        rprintln!("编码器校准完成");
    }

    fn update_measurements(&mut self, dt_ms: u32) -> Result<(), &'static str> {
        self.statistics.measurement_time_ms += dt_ms;
        Ok(())
    }

    fn get_current_status(&self) -> EncoderStatus {
        EncoderStatus {
            position: self.decoder.position,
            angle_degrees: self.decoder.get_angle_degrees(),
            revolutions: self.decoder.get_revolutions(),
            velocity_pps: self.velocity_calc.last_velocity,
            velocity_rpm: self.velocity_calc.get_rpm(self.decoder.resolution),
            direction: self.decoder.direction,
            mode: self.current_mode,
            index_position: self.index_position,
            error_count: self.decoder.error_count,
        }
    }

    fn send_event(&mut self, event: EncoderEvent) {
        if self.event_producer.enqueue(event).is_err() {
            rprintln!("警告: 事件队列已满");
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct EncoderStatus {
    position: i32,
    angle_degrees: f32,
    revolutions: f32,
    velocity_pps: f32,  // 脉冲/秒
    velocity_rpm: f32,  // 转/分
    direction: Direction,
    mode: EncoderMode,
    index_position: Option<i32>,
    error_count: u32,
}

static mut ENCODER_EVENT_QUEUE: Queue<EncoderEvent, 64> = Queue::new();
static mut ENCODER_TIMESTAMP: u32 = 0;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("编码器接口系统启动");

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
    let button3 = gpioc.pc15.into_pull_up_input(); // 校准/复位

    // 配置LED指示灯
    let mut status_led = gpiob.pb0.into_push_pull_output();      // 系统状态
    let mut measurement_led = gpiob.pb1.into_push_pull_output(); // 测量活动
    let mut position_led = gpiob.pb2.into_push_pull_output();    // 位置指示
    let mut velocity_led = gpiob.pb3.into_push_pull_output();    // 速度指示
    let mut direction_led = gpiob.pb4.into_push_pull_output();   // 方向指示
    let mut index_led = gpiob.pb5.into_push_pull_output();       // 索引指示
    let mut calibration_led = gpiob.pb6.into_push_pull_output(); // 校准指示
    let mut error_led = gpiob.pb7.into_push_pull_output();       // 错误指示

    // 配置编码器输入引脚
    let encoder_a = gpioa.pa0.into_pull_up_input(); // 编码器A相
    let encoder_b = gpioa.pa1.into_pull_up_input(); // 编码器B相
    let encoder_z = gpioa.pa2.into_pull_up_input(); // 编码器Z相（索引）

    // 配置定时器
    let mut timer2 = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer2.start(1.MHz()).unwrap(); // 1MHz计数频率
    timer2.listen(Event::Update);

    // 配置系统定时器
    let mut delay = cp.SYST.delay(&clocks);

    // 创建事件队列
    let (event_producer, mut event_consumer) = unsafe {
        ENCODER_EVENT_QUEUE.split()
    };

    // 创建编码器接口管理器
    let mut encoder_manager = EncoderInterfaceManager::new(
        1000,    // 1000 脉冲/转
        1_000_000, // 1MHz定时器频率
        event_producer
    );

    // 启用中断
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }

    // 系统变量
    let mut last_button1_state = button1.is_high();
    let mut last_button2_state = button2.is_high();
    let mut last_button3_state = button3.is_high();
    let mut button1_debounce = 0u32;
    let mut button2_debounce = 0u32;
    let mut button3_debounce = 0u32;
    let mut system_tick = 0u32;
    
    // 编码器状态
    let mut last_encoder_a = encoder_a.is_high();
    let mut last_encoder_b = encoder_b.is_high();
    let mut last_encoder_z = encoder_z.is_high();

    rprintln!("编码器接口系统就绪");

    loop {
        let current_time = system_tick;
        system_tick = system_tick.wrapping_add(1);

        // 读取编码器信号
        let encoder_a_state = encoder_a.is_high();
        let encoder_b_state = encoder_b.is_high();
        let encoder_z_state = encoder_z.is_high();

        // 处理编码器A/B相信号
        if encoder_a_state != last_encoder_a || encoder_b_state != last_encoder_b {
            if let Err(error) = encoder_manager.process_quadrature_signals(
                current_time, encoder_a_state, encoder_b_state
            ) {
                rprintln!("编码器信号处理错误: {:?}", error);
                error_led.set_high();
            } else {
                error_led.set_low();
            }
        }
        last_encoder_a = encoder_a_state;
        last_encoder_b = encoder_b_state;

        // 处理编码器Z相（索引）信号
        if encoder_z_state && !last_encoder_z {
            if let Err(e) = encoder_manager.process_index_pulse(current_time) {
                rprintln!("索引脉冲处理错误: {}", e);
                error_led.set_high();
            } else {
                index_led.set_high();
            }
        } else if !encoder_z_state && last_encoder_z {
            index_led.set_low();
        }
        last_encoder_z = encoder_z_state;

        // 按钮1处理（模式切换）
        let button1_state = button1.is_high();
        if button1_state != last_button1_state {
            button1_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button1_debounce) > 50 && button1_state && !last_button1_state {
            let new_mode = match encoder_manager.current_mode {
                EncoderMode::Quadrature => EncoderMode::Incremental,
                EncoderMode::Incremental => EncoderMode::Absolute,
                EncoderMode::Absolute => EncoderMode::Velocity,
                EncoderMode::Velocity => EncoderMode::Position,
                EncoderMode::Position => EncoderMode::Differential,
                EncoderMode::Differential => EncoderMode::Quadrature,
            };
            encoder_manager.send_event(EncoderEvent::ModeChange(new_mode));
        }
        last_button1_state = button1_state;

        // 按钮2处理（测量控制）
        let button2_state = button2.is_high();
        if button2_state != last_button2_state {
            button2_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button2_debounce) > 50 && button2_state && !last_button2_state {
            if encoder_manager.measurement_active {
                encoder_manager.stop_measurement();
            } else {
                encoder_manager.start_measurement();
            }
        }
        last_button2_state = button2_state;

        // 按钮3处理（校准/复位）
        let button3_state = button3.is_high();
        if button3_state != last_button3_state {
            button3_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button3_debounce) > 50 && button3_state && !last_button3_state {
            if encoder_manager.calibration_active {
                encoder_manager.end_calibration();
            } else {
                encoder_manager.start_calibration();
            }
        }
        last_button3_state = button3_state;

        // 处理事件队列
        while let Some(event) = event_consumer.dequeue() {
            match event {
                EncoderEvent::PositionUpdate(position) => {
                    position_led.set_high();
                }
                EncoderEvent::VelocityUpdate(velocity) => {
                    if velocity.abs() > 1.0 {
                        velocity_led.set_high();
                    } else {
                        velocity_led.set_low();
                    }
                }
                EncoderEvent::DirectionChange(direction) => {
                    match direction {
                        Direction::Clockwise => direction_led.set_high(),
                        Direction::CounterClockwise => {
                            if system_tick % 200 < 100 {
                                direction_led.set_high();
                            } else {
                                direction_led.set_low();
                            }
                        }
                        Direction::Stopped => direction_led.set_low(),
                    }
                }
                EncoderEvent::IndexPulse(_) => {
                    // 索引脉冲已在主循环中处理
                }
                EncoderEvent::CalibrationRequest => {
                    encoder_manager.start_calibration();
                }
                EncoderEvent::ModeChange(mode) => {
                    encoder_manager.set_mode(mode);
                }
                EncoderEvent::ErrorDetected(error) => {
                    rprintln!("编码器错误: {:?}", error);
                    error_led.set_high();
                }
            }
        }

        // 更新测量
        if let Err(e) = encoder_manager.update_measurements(10) {
            rprintln!("更新测量错误: {}", e);
            error_led.set_high();
        }

        // LED指示更新
        // 系统状态LED（心跳）
        if system_tick % 100 == 0 {
            status_led.toggle();
        }

        // 测量活动指示
        if encoder_manager.measurement_active {
            measurement_led.set_high();
        } else {
            measurement_led.set_low();
        }

        // 位置LED渐暗（表示最近有位置更新）
        if system_tick % 50 == 0 {
            position_led.set_low();
        }

        // 校准指示
        if encoder_manager.calibration_active {
            calibration_led.set_high();
        } else {
            calibration_led.set_low();
        }

        // 定期输出统计信息
        if system_tick % 5000 == 0 {
            rprintln!("=== 编码器接口统计信息 ===");
            rprintln!("总脉冲数: {}", encoder_manager.statistics.total_pulses);
            rprintln!("总转数: {:.3}", encoder_manager.statistics.total_revolutions);
            rprintln!("位置更新: {}", encoder_manager.statistics.position_updates);
            rprintln!("方向变化: {}", encoder_manager.statistics.direction_changes);
            rprintln!("索引脉冲: {}", encoder_manager.statistics.index_pulses);
            rprintln!("错误计数: {}", encoder_manager.statistics.error_count);
            rprintln!("校准次数: {}", encoder_manager.statistics.calibration_count);
            rprintln!("测量时间: {} ms", encoder_manager.statistics.measurement_time_ms);
            
            let status = encoder_manager.get_current_status();
            rprintln!("当前状态:");
            rprintln!("  位置: {}", status.position);
            rprintln!("  角度: {:.2}°", status.angle_degrees);
            rprintln!("  转数: {:.3}", status.revolutions);
            rprintln!("  速度: {:.2} 脉冲/秒, {:.1} RPM", status.velocity_pps, status.velocity_rpm);
            rprintln!("  方向: {:?}", status.direction);
            rprintln!("  模式: {:?}", status.mode);
            if let Some(index_pos) = status.index_position {
                rprintln!("  索引位置: {}", index_pos);
            }
            
            let velocity_stats = encoder_manager.velocity_calc.get_velocity_variance();
            rprintln!("速度统计:");
            rprintln!("  最大速度: {:.2} 脉冲/秒", encoder_manager.statistics.max_velocity);
            rprintln!("  最小速度: {:.2} 脉冲/秒", encoder_manager.statistics.min_velocity);
            rprintln!("  平均速度: {:.2} 脉冲/秒", encoder_manager.statistics.avg_velocity);
            rprintln!("  速度方差: {:.3}", velocity_stats);
        }

        delay.delay_ms(10u32);
    }
}

// 定时器中断处理
#[interrupt]
fn TIM2() {
    unsafe {
        ENCODER_TIMESTAMP = ENCODER_TIMESTAMP.wrapping_add(1);
    }
}