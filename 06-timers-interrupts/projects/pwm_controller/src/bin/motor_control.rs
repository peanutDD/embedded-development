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
use pwm_controller::{MotorDriver, PwmChannel};

// 电机事件类型
#[derive(Debug, Clone, Copy)]
enum MotorEvent {
    SpeedChange(f32),
    DirectionChange,
    Brake,
    Coast,
    AccelerationChange(f32),
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
        // 注意：实际应用中需要重新配置定时器频率
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

// 电机控制管理器
struct MotorControlManager<PWM1, PWM2> {
    motor_driver: MotorDriver<PWM1, PWM2>,
    control_mode: MotorControlMode,
    statistics: MotorStatistics,
    event_producer: Producer<'static, MotorEvent, 32>,
}

#[derive(Debug, Clone, Copy)]
enum MotorControlMode {
    Manual,      // 手动控制
    PID,         // PID控制
    Velocity,    // 速度控制
    Position,    // 位置控制
}

#[derive(Debug, Default)]
struct MotorStatistics {
    total_runtime: u32,
    direction_changes: u32,
    brake_events: u32,
    max_speed_reached: f32,
    average_speed: f32,
    power_consumption: f32,
}

impl<PWM1, PWM2> MotorControlManager<PWM1, PWM2>
where
    PWM1: PwmChannel,
    PWM2: PwmChannel,
{
    fn new(
        motor_driver: MotorDriver<PWM1, PWM2>,
        event_producer: Producer<'static, MotorEvent, 32>,
    ) -> Self {
        Self {
            motor_driver,
            control_mode: MotorControlMode::Manual,
            statistics: MotorStatistics::default(),
            event_producer,
        }
    }

    fn set_control_mode(&mut self, mode: MotorControlMode) {
        self.control_mode = mode;
        rprintln!("电机控制模式切换到: {:?}", mode);
    }

    fn handle_speed_control(&mut self, target_speed: f32) -> Result<(), &'static str> {
        match self.control_mode {
            MotorControlMode::Manual => {
                self.motor_driver.set_speed(target_speed)?;
            }
            MotorControlMode::Velocity => {
                // 实现速度控制算法
                let current_speed = self.motor_driver.get_current_speed();
                let error = target_speed - current_speed;
                let control_output = self.velocity_controller(error);
                self.motor_driver.set_speed(control_output)?;
            }
            MotorControlMode::PID => {
                // 实现PID控制
                let control_output = self.pid_controller(target_speed);
                self.motor_driver.set_speed(control_output)?;
            }
            MotorControlMode::Position => {
                // 位置控制模式
                let control_output = self.position_controller(target_speed);
                self.motor_driver.set_speed(control_output)?;
            }
        }
        Ok(())
    }

    fn velocity_controller(&self, error: f32) -> f32 {
        // 简单的比例控制器
        const KP: f32 = 0.8;
        error * KP
    }

    fn pid_controller(&self, target: f32) -> f32 {
        // 简化的PID控制器实现
        const KP: f32 = 1.0;
        const KI: f32 = 0.1;
        const KD: f32 = 0.05;
        
        let current = self.motor_driver.get_current_speed();
        let error = target - current;
        
        // 这里应该维护积分和微分项的历史
        error * KP // 简化版本，只有比例项
    }

    fn position_controller(&self, target_position: f32) -> f32 {
        // 位置控制器实现
        // 这里需要位置反馈，简化处理
        let current_speed = self.motor_driver.get_current_speed();
        if target_position > 0.0 {
            current_speed.min(target_position)
        } else {
            current_speed.max(target_position)
        }
    }

    fn update_statistics(&mut self, dt_ms: u32) {
        self.statistics.total_runtime += dt_ms;
        let current_speed = self.motor_driver.get_current_speed().abs();
        
        if current_speed > self.statistics.max_speed_reached {
            self.statistics.max_speed_reached = current_speed;
        }
        
        // 更新平均速度（简化计算）
        self.statistics.average_speed = 
            (self.statistics.average_speed * 0.99) + (current_speed * 0.01);
    }

    fn send_event(&mut self, event: MotorEvent) {
        if self.event_producer.enqueue(event).is_err() {
            rprintln!("警告: 事件队列已满");
        }
    }
}

static mut MOTOR_EVENT_QUEUE: Queue<MotorEvent, 32> = Queue::new();

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("电机控制系统启动");

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
    let mut motor_led = gpiob.pb1.into_push_pull_output();       // 电机运行状态
    let mut forward_led = gpiob.pb2.into_push_pull_output();     // 正转指示
    let mut backward_led = gpiob.pb3.into_push_pull_output();    // 反转指示
    let mut speed_led = gpiob.pb4.into_push_pull_output();       // 速度指示
    let mut brake_led = gpiob.pb5.into_push_pull_output();       // 制动指示
    let mut mode_led = gpiob.pb6.into_push_pull_output();        // 控制模式指示
    let mut error_led = gpiob.pb7.into_push_pull_output();       // 错误指示

    // 配置PWM引脚
    let pwm_pins = (
        gpioa.pa0.into_alternate(),  // TIM2_CH1 - 正转PWM
        gpioa.pa1.into_alternate(),  // TIM2_CH2 - 反转PWM
    );

    // 配置定时器
    let mut timer2 = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer2.start(1.kHz()).unwrap();

    // 配置PWM
    let pwm_freq = 20.kHz(); // 20kHz PWM频率，适合电机控制
    let mut pwm = dp.TIM2.pwm_hz(pwm_pins, pwm_freq, &clocks);
    
    let pwm_forward = PwmChannelWrapper::new(pwm.split().0, 20000);
    let pwm_backward = PwmChannelWrapper::new(pwm.split().1, 20000);

    // 创建事件队列
    let (event_producer, mut event_consumer) = unsafe {
        MOTOR_EVENT_QUEUE.split()
    };

    // 创建电机驱动器
    let motor_driver = MotorDriver::new(pwm_forward, pwm_backward);
    
    // 创建电机控制管理器
    let mut motor_manager = MotorControlManager::new(motor_driver, event_producer);

    // 系统定时器
    let mut delay = cp.SYST.delay(&clocks);
    let mut last_button_state = button.is_high();
    let mut button_debounce = 0u32;
    let mut system_tick = 0u32;
    let mut demo_phase = 0u32;

    rprintln!("电机控制系统就绪");

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
            let new_mode = match motor_manager.control_mode {
                MotorControlMode::Manual => MotorControlMode::PID,
                MotorControlMode::PID => MotorControlMode::Velocity,
                MotorControlMode::Velocity => MotorControlMode::Position,
                MotorControlMode::Position => MotorControlMode::Manual,
            };
            motor_manager.set_control_mode(new_mode);
            motor_manager.send_event(MotorEvent::SpeedChange(0.0)); // 重置速度
        }
        last_button_state = button_state;

        // 处理事件队列
        while let Some(event) = event_consumer.dequeue() {
            match event {
                MotorEvent::SpeedChange(speed) => {
                    if let Err(e) = motor_manager.handle_speed_control(speed) {
                        rprintln!("速度控制错误: {}", e);
                        error_led.set_high();
                    } else {
                        error_led.set_low();
                    }
                }
                MotorEvent::DirectionChange => {
                    motor_manager.statistics.direction_changes += 1;
                    rprintln!("方向改变，总次数: {}", motor_manager.statistics.direction_changes);
                }
                MotorEvent::Brake => {
                    if let Err(e) = motor_manager.motor_driver.enable_brake() {
                        rprintln!("制动错误: {}", e);
                    } else {
                        motor_manager.statistics.brake_events += 1;
                        brake_led.set_high();
                    }
                }
                MotorEvent::Coast => {
                    if let Err(e) = motor_manager.motor_driver.disable_brake() {
                        rprintln!("取消制动错误: {}", e);
                    } else {
                        brake_led.set_low();
                    }
                }
                MotorEvent::AccelerationChange(accel) => {
                    motor_manager.motor_driver.set_acceleration(accel);
                    rprintln!("加速度设置为: {}", accel);
                }
            }
        }

        // 演示不同控制模式
        match motor_manager.control_mode {
            MotorControlMode::Manual => {
                mode_led.set_low();
                // 手动控制演示：正弦波速度
                let speed = libm::sinf(demo_phase as f32 * 0.01) * 0.8;
                motor_manager.send_event(MotorEvent::SpeedChange(speed));
            }
            MotorControlMode::PID => {
                mode_led.set_high();
                // PID控制演示：阶跃响应
                let target_speed = if (demo_phase / 1000) % 2 == 0 { 0.6 } else { -0.6 };
                motor_manager.send_event(MotorEvent::SpeedChange(target_speed));
            }
            MotorControlMode::Velocity => {
                // 速度控制演示：斜坡信号
                if demo_phase % 100 == 0 {
                    mode_led.toggle();
                }
                let ramp_speed = ((demo_phase % 2000) as f32 / 1000.0 - 1.0) * 0.7;
                motor_manager.send_event(MotorEvent::SpeedChange(ramp_speed));
            }
            MotorControlMode::Position => {
                // 位置控制演示：方波信号
                if demo_phase % 200 == 0 {
                    mode_led.toggle();
                }
                let position = if (demo_phase / 1500) % 2 == 0 { 0.5 } else { -0.5 };
                motor_manager.send_event(MotorEvent::SpeedChange(position));
            }
        }

        // 更新电机驱动器
        if let Err(e) = motor_manager.motor_driver.update(10) {
            rprintln!("电机更新错误: {}", e);
            error_led.set_high();
        }

        // 更新统计信息
        motor_manager.update_statistics(10);

        // LED指示更新
        let current_speed = motor_manager.motor_driver.get_current_speed();
        
        // 系统状态LED（心跳）
        if system_tick % 100 == 0 {
            status_led.toggle();
        }

        // 电机运行状态
        if current_speed.abs() > 0.1 {
            motor_led.set_high();
        } else {
            motor_led.set_low();
        }

        // 方向指示
        if current_speed > 0.1 {
            forward_led.set_high();
            backward_led.set_low();
        } else if current_speed < -0.1 {
            forward_led.set_low();
            backward_led.set_high();
        } else {
            forward_led.set_low();
            backward_led.set_low();
        }

        // 速度指示（PWM调光）
        let speed_intensity = (current_speed.abs() * 100.0) as u32;
        if system_tick % 100 < speed_intensity {
            speed_led.set_high();
        } else {
            speed_led.set_low();
        }

        // 定期输出统计信息
        if system_tick % 5000 == 0 {
            rprintln!("=== 电机统计信息 ===");
            rprintln!("运行时间: {}ms", motor_manager.statistics.total_runtime);
            rprintln!("方向改变次数: {}", motor_manager.statistics.direction_changes);
            rprintln!("制动次数: {}", motor_manager.statistics.brake_events);
            rprintln!("最大速度: {:.2}", motor_manager.statistics.max_speed_reached);
            rprintln!("平均速度: {:.2}", motor_manager.statistics.average_speed);
            rprintln!("当前速度: {:.2}", current_speed);
            rprintln!("控制模式: {:?}", motor_manager.control_mode);
        }

        demo_phase = demo_phase.wrapping_add(1);
        delay.delay_ms(10u32);
    }
}