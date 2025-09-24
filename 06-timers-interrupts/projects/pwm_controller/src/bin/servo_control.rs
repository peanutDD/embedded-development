#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, PushPull, Pin},
    timer::{Timer, Channel1, Channel2, Channel3, Channel4, PwmHz},
    rcc::Clocks,
};
use heapless::spsc::{Queue, Producer, Consumer};
use rtt_target::{rprintln, rtt_init_print};
use pwm_controller::{ServoController, PwmChannel};
use libm::{sinf, cosf, fabsf};

// 舵机控制事件类型
#[derive(Debug, Clone, Copy)]
enum ServoEvent {
    MoveToAngle(u8, f32),        // 舵机ID, 目标角度
    SetSpeed(u8, f32),           // 舵机ID, 移动速度
    Calibrate(u8),               // 舵机ID
    Stop(u8),                    // 舵机ID
    PatternChange(ServoPattern), // 模式切换
}

#[derive(Debug, Clone, Copy)]
enum ServoPattern {
    Manual,      // 手动控制
    Sweep,       // 扫描模式
    Sequence,    // 序列动作
    Synchronized,// 同步控制
    Random,      // 随机运动
}

// PWM通道包装器
struct PwmChannelWrapper<TIM, CHANNEL> {
    pwm: PwmHz<TIM, CHANNEL>,
    duty_cycle: f32,
    frequency: u32,
}

impl<TIM, CHANNEL> PwmChannelWrapper<TIM, CHANNEL>
where
    PwmHz<TIM, CHANNEL>: embedded_hal::pwm::SetDutyCycle,
{
    fn new(pwm: PwmHz<TIM, CHANNEL>, frequency: u32) -> Self {
        Self {
            pwm,
            duty_cycle: 0.0,
            frequency,
        }
    }
}

impl<TIM, CHANNEL> PwmChannel for PwmChannelWrapper<TIM, CHANNEL>
where
    PwmHz<TIM, CHANNEL>: embedded_hal::pwm::SetDutyCycle,
{
    type Error = ();

    fn set_duty_cycle(&mut self, duty: f32) -> Result<(), Self::Error> {
        self.duty_cycle = duty.clamp(0.0, 1.0);
        let max_duty = self.pwm.get_max_duty();
        let duty_value = (max_duty as f32 * self.duty_cycle) as u16;
        self.pwm.set_duty(duty_value);
        Ok(())
    }

    fn set_frequency(&mut self, freq: u32) -> Result<(), Self::Error> {
        self.frequency = freq;
        Ok(())
    }

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.pwm.enable();
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.pwm.disable();
        Ok(())
    }

    fn get_duty_cycle(&self) -> f32 {
        self.duty_cycle
    }

    fn get_frequency(&self) -> u32 {
        self.frequency
    }
}

// 多舵机控制管理器
struct MultiServoManager<PWM1, PWM2, PWM3, PWM4> {
    servo1: ServoController<PWM1>,
    servo2: ServoController<PWM2>,
    servo3: ServoController<PWM3>,
    servo4: ServoController<PWM4>,
    current_pattern: ServoPattern,
    pattern_phase: f32,
    pattern_speed: f32,
    statistics: ServoStatistics,
    event_producer: Producer<'static, ServoEvent, 32>,
}

#[derive(Debug, Default)]
struct ServoStatistics {
    total_runtime: u32,
    pattern_changes: u32,
    movements: u32,
    calibrations: u32,
    total_angle_moved: f32,
    max_speed: f32,
    servo_errors: u32,
}

impl<PWM1, PWM2, PWM3, PWM4> MultiServoManager<PWM1, PWM2, PWM3, PWM4>
where
    PWM1: PwmChannel,
    PWM2: PwmChannel,
    PWM3: PwmChannel,
    PWM4: PwmChannel,
{
    fn new(
        servo1: ServoController<PWM1>,
        servo2: ServoController<PWM2>,
        servo3: ServoController<PWM3>,
        servo4: ServoController<PWM4>,
        event_producer: Producer<'static, ServoEvent, 32>,
    ) -> Self {
        Self {
            servo1,
            servo2,
            servo3,
            servo4,
            current_pattern: ServoPattern::Manual,
            pattern_phase: 0.0,
            pattern_speed: 1.0,
            statistics: ServoStatistics::default(),
            event_producer,
        }
    }

    fn set_pattern(&mut self, pattern: ServoPattern) {
        self.current_pattern = pattern;
        self.pattern_phase = 0.0;
        self.statistics.pattern_changes += 1;
        rprintln!("舵机模式切换到: {:?}", pattern);
    }

    fn update_pattern(&mut self, dt_ms: u32) -> Result<(), &'static str> {
        let dt = dt_ms as f32 / 1000.0; // 转换为秒
        self.pattern_phase += dt * self.pattern_speed;

        match self.current_pattern {
            ServoPattern::Manual => {
                // 手动模式，不自动更新
            }
            ServoPattern::Sweep => {
                // 扫描模式：所有舵机同步扫描
                let angle = (sinf(self.pattern_phase) + 1.0) * 90.0; // 0-180度
                self.servo1.move_to_angle(angle, 90.0)?; // 90度/秒
                self.servo2.move_to_angle(angle, 90.0)?;
                self.servo3.move_to_angle(angle, 90.0)?;
                self.servo4.move_to_angle(angle, 90.0)?;
            }
            ServoPattern::Sequence => {
                // 序列模式：舵机依次运动
                let cycle_time = 8.0; // 8秒一个周期
                let phase = (self.pattern_phase % cycle_time) / cycle_time;
                
                if phase < 0.25 {
                    // 第一个舵机运动
                    let angle = phase * 4.0 * 180.0;
                    self.servo1.move_to_angle(angle, 180.0)?;
                } else if phase < 0.5 {
                    // 第二个舵机运动
                    let angle = (phase - 0.25) * 4.0 * 180.0;
                    self.servo2.move_to_angle(angle, 180.0)?;
                } else if phase < 0.75 {
                    // 第三个舵机运动
                    let angle = (phase - 0.5) * 4.0 * 180.0;
                    self.servo3.move_to_angle(angle, 180.0)?;
                } else {
                    // 第四个舵机运动
                    let angle = (phase - 0.75) * 4.0 * 180.0;
                    self.servo4.move_to_angle(angle, 180.0)?;
                }
            }
            ServoPattern::Synchronized => {
                // 同步模式：不同相位的正弦运动
                let base_angle = 90.0; // 中心角度
                let amplitude = 45.0;  // 摆动幅度
                
                let angle1 = base_angle + amplitude * sinf(self.pattern_phase);
                let angle2 = base_angle + amplitude * sinf(self.pattern_phase + 1.57); // 90度相位差
                let angle3 = base_angle + amplitude * sinf(self.pattern_phase + 3.14); // 180度相位差
                let angle4 = base_angle + amplitude * sinf(self.pattern_phase + 4.71); // 270度相位差
                
                self.servo1.move_to_angle(angle1, 120.0)?;
                self.servo2.move_to_angle(angle2, 120.0)?;
                self.servo3.move_to_angle(angle3, 120.0)?;
                self.servo4.move_to_angle(angle4, 120.0)?;
            }
            ServoPattern::Random => {
                // 随机模式：每2秒随机移动到新位置
                if (self.pattern_phase as u32) % 2000 == 0 {
                    // 简化的伪随机数生成
                    let seed = (self.pattern_phase * 1000.0) as u32;
                    let angle1 = ((seed * 17) % 180) as f32;
                    let angle2 = (((seed + 1) * 23) % 180) as f32;
                    let angle3 = (((seed + 2) * 31) % 180) as f32;
                    let angle4 = (((seed + 3) * 37) % 180) as f32;
                    
                    self.servo1.move_to_angle(angle1, 60.0)?;
                    self.servo2.move_to_angle(angle2, 60.0)?;
                    self.servo3.move_to_angle(angle3, 60.0)?;
                    self.servo4.move_to_angle(angle4, 60.0)?;
                }
            }
        }

        // 更新所有舵机
        self.servo1.update(dt_ms)?;
        self.servo2.update(dt_ms)?;
        self.servo3.update(dt_ms)?;
        self.servo4.update(dt_ms)?;

        Ok(())
    }

    fn move_servo(&mut self, servo_id: u8, angle: f32, speed: f32) -> Result<(), &'static str> {
        match servo_id {
            1 => self.servo1.move_to_angle(angle, speed)?,
            2 => self.servo2.move_to_angle(angle, speed)?,
            3 => self.servo3.move_to_angle(angle, speed)?,
            4 => self.servo4.move_to_angle(angle, speed)?,
            _ => return Err("无效的舵机ID"),
        }
        self.statistics.movements += 1;
        Ok(())
    }

    fn get_servo_angle(&self, servo_id: u8) -> Result<f32, &'static str> {
        match servo_id {
            1 => Ok(self.servo1.get_current_angle()),
            2 => Ok(self.servo2.get_current_angle()),
            3 => Ok(self.servo3.get_current_angle()),
            4 => Ok(self.servo4.get_current_angle()),
            _ => Err("无效的舵机ID"),
        }
    }

    fn calibrate_servo(&mut self, servo_id: u8) -> Result<(), &'static str> {
        // 校准过程：移动到0度、90度、180度
        match servo_id {
            1 => {
                self.servo1.move_to_angle(0.0, 30.0)?;
                // 实际应用中需要等待移动完成
                self.servo1.move_to_angle(90.0, 30.0)?;
                self.servo1.move_to_angle(180.0, 30.0)?;
                self.servo1.move_to_angle(90.0, 30.0)?; // 回到中心
            }
            2 => {
                self.servo2.move_to_angle(0.0, 30.0)?;
                self.servo2.move_to_angle(90.0, 30.0)?;
                self.servo2.move_to_angle(180.0, 30.0)?;
                self.servo2.move_to_angle(90.0, 30.0)?;
            }
            3 => {
                self.servo3.move_to_angle(0.0, 30.0)?;
                self.servo3.move_to_angle(90.0, 30.0)?;
                self.servo3.move_to_angle(180.0, 30.0)?;
                self.servo3.move_to_angle(90.0, 30.0)?;
            }
            4 => {
                self.servo4.move_to_angle(0.0, 30.0)?;
                self.servo4.move_to_angle(90.0, 30.0)?;
                self.servo4.move_to_angle(180.0, 30.0)?;
                self.servo4.move_to_angle(90.0, 30.0)?;
            }
            _ => return Err("无效的舵机ID"),
        }
        self.statistics.calibrations += 1;
        rprintln!("舵机 {} 校准完成", servo_id);
        Ok(())
    }

    fn update_statistics(&mut self, dt_ms: u32) {
        self.statistics.total_runtime += dt_ms;
        
        // 计算总的角度移动量（简化计算）
        let angle_delta = fabsf(self.servo1.get_target_angle() - self.servo1.get_current_angle()) +
                         fabsf(self.servo2.get_target_angle() - self.servo2.get_current_angle()) +
                         fabsf(self.servo3.get_target_angle() - self.servo3.get_current_angle()) +
                         fabsf(self.servo4.get_target_angle() - self.servo4.get_current_angle());
        
        self.statistics.total_angle_moved += angle_delta * (dt_ms as f32 / 1000.0);
    }

    fn send_event(&mut self, event: ServoEvent) {
        if self.event_producer.enqueue(event).is_err() {
            rprintln!("警告: 事件队列已满");
        }
    }
}

static mut SERVO_EVENT_QUEUE: Queue<ServoEvent, 32> = Queue::new();

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("多舵机控制系统启动");

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 配置按钮（模式切换）
    let button = gpioc.pc13.into_pull_up_input();

    // 配置LED指示灯
    let mut status_led = gpiob.pb0.into_push_pull_output();      // 系统状态
    let mut servo_led = gpiob.pb1.into_push_pull_output();       // 舵机活动指示
    let mut servo1_led = gpiob.pb2.into_push_pull_output();      // 舵机1状态
    let mut servo2_led = gpiob.pb3.into_push_pull_output();      // 舵机2状态
    let mut servo3_led = gpiob.pb4.into_push_pull_output();      // 舵机3状态
    let mut servo4_led = gpiob.pb5.into_push_pull_output();      // 舵机4状态
    let mut pattern_led = gpiob.pb6.into_push_pull_output();     // 模式指示
    let mut error_led = gpiob.pb7.into_push_pull_output();       // 错误指示

    // 配置PWM引脚（四个舵机）
    let pwm_pins_tim2 = (
        gpioa.pa0.into_alternate(),  // TIM2_CH1 - 舵机1
        gpioa.pa1.into_alternate(),  // TIM2_CH2 - 舵机2
    );
    
    let pwm_pins_tim3 = (
        gpioa.pa6.into_alternate(),  // TIM3_CH1 - 舵机3
        gpioa.pa7.into_alternate(),  // TIM3_CH2 - 舵机4
    );

    // 配置定时器
    let mut timer2 = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer2.start(1.kHz()).unwrap();

    // 配置PWM（50Hz，舵机标准频率）
    let pwm_freq = 50.Hz(); // 50Hz PWM频率，舵机标准
    let mut pwm2 = dp.TIM2.pwm_hz(pwm_pins_tim2, pwm_freq, &clocks);
    let mut pwm3 = dp.TIM3.pwm_hz(pwm_pins_tim3, pwm_freq, &clocks);
    
    let (pwm2_ch1, pwm2_ch2) = pwm2.split();
    let (pwm3_ch1, pwm3_ch2) = pwm3.split();
    
    let servo1_pwm = PwmChannelWrapper::new(pwm2_ch1, 50);
    let servo2_pwm = PwmChannelWrapper::new(pwm2_ch2, 50);
    let servo3_pwm = PwmChannelWrapper::new(pwm3_ch1, 50);
    let servo4_pwm = PwmChannelWrapper::new(pwm3_ch2, 50);

    // 创建事件队列
    let (event_producer, mut event_consumer) = unsafe {
        SERVO_EVENT_QUEUE.split()
    };

    // 创建舵机控制器（标准舵机：1-2ms脉宽，20ms周期）
    let servo1 = ServoController::new(servo1_pwm, 1000, 2000); // 1-2ms脉宽
    let servo2 = ServoController::new(servo2_pwm, 1000, 2000);
    let servo3 = ServoController::new(servo3_pwm, 1000, 2000);
    let servo4 = ServoController::new(servo4_pwm, 1000, 2000);
    
    // 创建多舵机管理器
    let mut servo_manager = MultiServoManager::new(
        servo1, servo2, servo3, servo4, event_producer
    );

    // 系统定时器
    let mut delay = cp.SYST.delay(&clocks);
    let mut last_button_state = button.is_high();
    let mut button_debounce = 0u32;
    let mut system_tick = 0u32;
    let mut demo_phase = 0u32;

    rprintln!("多舵机控制系统就绪");

    loop {
        let current_time = system_tick;
        system_tick = system_tick.wrapping_add(1);

        // 按钮处理（模式切换）
        let button_state = button.is_high();
        if button_state != last_button_state {
            button_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button_debounce) > 50 && button_state && !last_button_state {
            // 按钮按下，切换控制模式
            let new_pattern = match servo_manager.current_pattern {
                ServoPattern::Manual => ServoPattern::Sweep,
                ServoPattern::Sweep => ServoPattern::Sequence,
                ServoPattern::Sequence => ServoPattern::Synchronized,
                ServoPattern::Synchronized => ServoPattern::Random,
                ServoPattern::Random => ServoPattern::Manual,
            };
            servo_manager.send_event(ServoEvent::PatternChange(new_pattern));
        }
        last_button_state = button_state;

        // 处理事件队列
        while let Some(event) = event_consumer.dequeue() {
            match event {
                ServoEvent::MoveToAngle(servo_id, angle) => {
                    if let Err(e) = servo_manager.move_servo(servo_id, angle, 90.0) {
                        rprintln!("舵机移动错误: {}", e);
                        error_led.set_high();
                    } else {
                        error_led.set_low();
                    }
                }
                ServoEvent::SetSpeed(servo_id, speed) => {
                    // 设置舵机速度（这里简化处理）
                    rprintln!("设置舵机 {} 速度为 {}", servo_id, speed);
                    if speed > servo_manager.statistics.max_speed {
                        servo_manager.statistics.max_speed = speed;
                    }
                }
                ServoEvent::Calibrate(servo_id) => {
                    if let Err(e) = servo_manager.calibrate_servo(servo_id) {
                        rprintln!("舵机校准错误: {}", e);
                        error_led.set_high();
                    } else {
                        error_led.set_low();
                    }
                }
                ServoEvent::Stop(servo_id) => {
                    // 停止舵机（保持当前位置）
                    if let Ok(current_angle) = servo_manager.get_servo_angle(servo_id) {
                        if let Err(e) = servo_manager.move_servo(servo_id, current_angle, 0.0) {
                            rprintln!("舵机停止错误: {}", e);
                        }
                    }
                }
                ServoEvent::PatternChange(pattern) => {
                    servo_manager.set_pattern(pattern);
                }
            }
        }

        // 更新舵机模式
        if let Err(e) = servo_manager.update_pattern(10) {
            rprintln!("模式更新错误: {}", e);
            error_led.set_high();
            servo_manager.statistics.servo_errors += 1;
        }

        // 更新统计信息
        servo_manager.update_statistics(10);

        // LED指示更新
        // 系统状态LED（心跳）
        if system_tick % 100 == 0 {
            status_led.toggle();
        }

        // 舵机活动指示
        let any_moving = (servo_manager.servo1.get_current_angle() - servo_manager.servo1.get_target_angle()).abs() > 1.0 ||
                        (servo_manager.servo2.get_current_angle() - servo_manager.servo2.get_target_angle()).abs() > 1.0 ||
                        (servo_manager.servo3.get_current_angle() - servo_manager.servo3.get_target_angle()).abs() > 1.0 ||
                        (servo_manager.servo4.get_current_angle() - servo_manager.servo4.get_target_angle()).abs() > 1.0;
        
        if any_moving {
            servo_led.set_high();
        } else {
            servo_led.set_low();
        }

        // 各舵机状态指示（根据角度调制LED亮度）
        let servo1_angle = servo_manager.servo1.get_current_angle();
        let servo1_intensity = (servo1_angle / 180.0 * 100.0) as u32;
        if system_tick % 100 < servo1_intensity {
            servo1_led.set_high();
        } else {
            servo1_led.set_low();
        }

        let servo2_angle = servo_manager.servo2.get_current_angle();
        let servo2_intensity = (servo2_angle / 180.0 * 100.0) as u32;
        if system_tick % 100 < servo2_intensity {
            servo2_led.set_high();
        } else {
            servo2_led.set_low();
        }

        let servo3_angle = servo_manager.servo3.get_current_angle();
        let servo3_intensity = (servo3_angle / 180.0 * 100.0) as u32;
        if system_tick % 100 < servo3_intensity {
            servo3_led.set_high();
        } else {
            servo3_led.set_low();
        }

        let servo4_angle = servo_manager.servo4.get_current_angle();
        let servo4_intensity = (servo4_angle / 180.0 * 100.0) as u32;
        if system_tick % 100 < servo4_intensity {
            servo4_led.set_high();
        } else {
            servo4_led.set_low();
        }

        // 模式指示（不同模式不同闪烁频率）
        let pattern_blink_period = match servo_manager.current_pattern {
            ServoPattern::Manual => 1000,
            ServoPattern::Sweep => 500,
            ServoPattern::Sequence => 200,
            ServoPattern::Synchronized => 100,
            ServoPattern::Random => 300,
        };
        
        if system_tick % pattern_blink_period < pattern_blink_period / 2 {
            pattern_led.set_high();
        } else {
            pattern_led.set_low();
        }

        // 定期输出统计信息
        if system_tick % 5000 == 0 {
            rprintln!("=== 舵机控制统计信息 ===");
            rprintln!("运行时间: {}ms", servo_manager.statistics.total_runtime);
            rprintln!("模式切换次数: {}", servo_manager.statistics.pattern_changes);
            rprintln!("移动次数: {}", servo_manager.statistics.movements);
            rprintln!("校准次数: {}", servo_manager.statistics.calibrations);
            rprintln!("总角度移动: {:.1}度", servo_manager.statistics.total_angle_moved);
            rprintln!("最大速度: {:.1}度/秒", servo_manager.statistics.max_speed);
            rprintln!("错误次数: {}", servo_manager.statistics.servo_errors);
            rprintln!("当前模式: {:?}", servo_manager.current_pattern);
            rprintln!("舵机角度 - 1: {:.1}°, 2: {:.1}°, 3: {:.1}°, 4: {:.1}°",
                     servo1_angle, servo2_angle, servo3_angle, servo4_angle);
        }

        demo_phase = demo_phase.wrapping_add(1);
        delay.delay_ms(10u32);
    }
}