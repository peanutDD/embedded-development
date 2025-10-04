#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{gpioa::PA0, gpioc::PC13, Input, Output, PullUp, PushPull},
    timer::{CounterUs, Event},
};
use cortex_m::{interrupt, peripheral::NVIC};
use heapless::{Vec, spsc::{Producer, Consumer, Queue}};

// ==================== 分层架构模式 ====================

// 硬件抽象层 (HAL)
trait HardwareAbstraction {
    fn init(&mut self) -> Result<(), HalError>;
    fn read_sensor(&self) -> Result<f32, HalError>;
    fn write_actuator(&mut self, value: u8) -> Result<(), HalError>;
    fn get_timestamp(&self) -> u32;
}

#[derive(Debug, Clone, Copy)]
enum HalError {
    InitializationFailed,
    ReadError,
    WriteError,
    HardwareNotReady,
}

// STM32F4 HAL实现
struct Stm32F4Hal {
    led: Option<PC13<Output<PushPull>>>,
    button: Option<PA0<Input<PullUp>>>,
    timestamp_counter: u32,
    initialized: bool,
}

impl Stm32F4Hal {
    fn new() -> Self {
        Self {
            led: None,
            button: None,
            timestamp_counter: 0,
            initialized: false,
        }
    }
    
    fn set_led(&mut self, led: PC13<Output<PushPull>>) {
        self.led = Some(led);
    }
    
    fn set_button(&mut self, button: PA0<Input<PullUp>>) {
        self.button = Some(button);
    }
    
    fn tick(&mut self) {
        self.timestamp_counter = self.timestamp_counter.wrapping_add(1);
    }
}

impl HardwareAbstraction for Stm32F4Hal {
    fn init(&mut self) -> Result<(), HalError> {
        if self.led.is_some() && self.button.is_some() {
            self.initialized = true;
            Ok(())
        } else {
            Err(HalError::InitializationFailed)
        }
    }
    
    fn read_sensor(&self) -> Result<f32, HalError> {
        if !self.initialized {
            return Err(HalError::HardwareNotReady);
        }
        
        if let Some(ref button) = self.button {
            // 模拟传感器读取：按钮按下时返回高值
            if button.is_low() {
                Ok(100.0)
            } else {
                Ok(25.0)
            }
        } else {
            Err(HalError::ReadError)
        }
    }
    
    fn write_actuator(&mut self, value: u8) -> Result<(), HalError> {
        if !self.initialized {
            return Err(HalError::HardwareNotReady);
        }
        
        if let Some(ref mut led) = self.led {
            if value > 128 {
                led.set_low();
            } else {
                led.set_high();
            }
            Ok(())
        } else {
            Err(HalError::WriteError)
        }
    }
    
    fn get_timestamp(&self) -> u32 {
        self.timestamp_counter
    }
}

// 设备驱动层
struct SensorDriver<H: HardwareAbstraction> {
    hal: H,
    calibration_offset: f32,
    filter_alpha: f32,
    filtered_value: f32,
}

impl<H: HardwareAbstraction> SensorDriver<H> {
    fn new(hal: H) -> Self {
        Self {
            hal,
            calibration_offset: 0.0,
            filter_alpha: 0.1,
            filtered_value: 0.0,
        }
    }
    
    fn read_filtered(&mut self) -> Result<f32, HalError> {
        let raw_value = self.hal.read_sensor()?;
        let calibrated_value = raw_value + self.calibration_offset;
        
        // 简单的低通滤波
        self.filtered_value = self.filter_alpha * calibrated_value + 
                             (1.0 - self.filter_alpha) * self.filtered_value;
        
        Ok(self.filtered_value)
    }
    
    fn calibrate(&mut self, reference_value: f32) -> Result<(), HalError> {
        let current_value = self.hal.read_sensor()?;
        self.calibration_offset = reference_value - current_value;
        Ok(())
    }
}

// 应用服务层
struct ControlService<H: HardwareAbstraction> {
    sensor_driver: SensorDriver<H>,
    setpoint: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    previous_error: f32,
    output_limits: (f32, f32),
}

impl<H: HardwareAbstraction> ControlService<H> {
    fn new(sensor_driver: SensorDriver<H>) -> Self {
        Self {
            sensor_driver,
            setpoint: 50.0,
            kp: 1.0,
            ki: 0.1,
            kd: 0.01,
            integral: 0.0,
            previous_error: 0.0,
            output_limits: (0.0, 255.0),
        }
    }
    
    fn update(&mut self) -> Result<u8, HalError> {
        let current_value = self.sensor_driver.read_filtered()?;
        let error = self.setpoint - current_value;
        
        // PID控制算法
        self.integral += error;
        let derivative = error - self.previous_error;
        
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        
        // 限制输出范围
        let limited_output = output.max(self.output_limits.0).min(self.output_limits.1);
        
        self.previous_error = error;
        
        // 写入执行器
        self.sensor_driver.hal.write_actuator(limited_output as u8)?;
        
        Ok(limited_output as u8)
    }
    
    fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }
    
    fn tune_pid(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }
}

// ==================== 状态机模式 ====================

// 状态机特征
trait State<Context, Event> {
    fn handle_event(&self, context: &mut Context, event: Event) -> Option<Box<dyn State<Context, Event>>>;
    fn entry_action(&self, context: &mut Context) {}
    fn exit_action(&self, context: &mut Context) {}
    fn state_name(&self) -> &'static str;
}

// 系统状态机上下文
struct SystemContext {
    temperature: f32,
    target_temperature: f32,
    heater_power: u8,
    fan_speed: u8,
    error_count: u32,
    uptime: u32,
}

impl SystemContext {
    fn new() -> Self {
        Self {
            temperature: 25.0,
            target_temperature: 50.0,
            heater_power: 0,
            fan_speed: 0,
            error_count: 0,
            uptime: 0,
        }
    }
}

// 系统事件
#[derive(Debug, Clone, Copy)]
enum SystemEvent {
    PowerOn,
    PowerOff,
    TemperatureUpdate(f32),
    SetTarget(f32),
    Error,
    Reset,
    Timeout,
}

// 状态定义
struct IdleState;
struct HeatingState;
struct CoolingState;
struct ErrorState;

impl State<SystemContext, SystemEvent> for IdleState {
    fn handle_event(&self, context: &mut SystemContext, event: SystemEvent) -> Option<Box<dyn State<SystemContext, SystemEvent>>> {
        match event {
            SystemEvent::TemperatureUpdate(temp) => {
                context.temperature = temp;
                if temp < context.target_temperature - 2.0 {
                    Some(Box::new(HeatingState))
                } else if temp > context.target_temperature + 2.0 {
                    Some(Box::new(CoolingState))
                } else {
                    None
                }
            }
            SystemEvent::SetTarget(target) => {
                context.target_temperature = target;
                None
            }
            SystemEvent::Error => {
                context.error_count += 1;
                Some(Box::new(ErrorState))
            }
            _ => None,
        }
    }
    
    fn entry_action(&self, context: &mut SystemContext) {
        context.heater_power = 0;
        context.fan_speed = 0;
    }
    
    fn state_name(&self) -> &'static str {
        "Idle"
    }
}

impl State<SystemContext, SystemEvent> for HeatingState {
    fn handle_event(&self, context: &mut SystemContext, event: SystemEvent) -> Option<Box<dyn State<SystemContext, SystemEvent>>> {
        match event {
            SystemEvent::TemperatureUpdate(temp) => {
                context.temperature = temp;
                if temp >= context.target_temperature {
                    Some(Box::new(IdleState))
                } else {
                    // 调整加热功率
                    let temp_diff = context.target_temperature - temp;
                    context.heater_power = (temp_diff * 10.0).min(100.0) as u8;
                    None
                }
            }
            SystemEvent::Error => {
                context.error_count += 1;
                Some(Box::new(ErrorState))
            }
            _ => None,
        }
    }
    
    fn entry_action(&self, context: &mut SystemContext) {
        context.heater_power = 50;
        context.fan_speed = 0;
    }
    
    fn state_name(&self) -> &'static str {
        "Heating"
    }
}

impl State<SystemContext, SystemEvent> for CoolingState {
    fn handle_event(&self, context: &mut SystemContext, event: SystemEvent) -> Option<Box<dyn State<SystemContext, SystemEvent>>> {
        match event {
            SystemEvent::TemperatureUpdate(temp) => {
                context.temperature = temp;
                if temp <= context.target_temperature {
                    Some(Box::new(IdleState))
                } else {
                    // 调整风扇速度
                    let temp_diff = temp - context.target_temperature;
                    context.fan_speed = (temp_diff * 20.0).min(100.0) as u8;
                    None
                }
            }
            SystemEvent::Error => {
                context.error_count += 1;
                Some(Box::new(ErrorState))
            }
            _ => None,
        }
    }
    
    fn entry_action(&self, context: &mut SystemContext) {
        context.heater_power = 0;
        context.fan_speed = 30;
    }
    
    fn state_name(&self) -> &'static str {
        "Cooling"
    }
}

impl State<SystemContext, SystemEvent> for ErrorState {
    fn handle_event(&self, context: &mut SystemContext, event: SystemEvent) -> Option<Box<dyn State<SystemContext, SystemEvent>>> {
        match event {
            SystemEvent::Reset => {
                context.error_count = 0;
                Some(Box::new(IdleState))
            }
            _ => None,
        }
    }
    
    fn entry_action(&self, context: &mut SystemContext) {
        context.heater_power = 0;
        context.fan_speed = 100; // 全速风扇用于安全
    }
    
    fn state_name(&self) -> &'static str {
        "Error"
    }
}

// 状态机
struct StateMachine<Context, Event> {
    current_state: Box<dyn State<Context, Event>>,
    context: Context,
}

impl<Context, Event> StateMachine<Context, Event> {
    fn new(initial_state: Box<dyn State<Context, Event>>, context: Context) -> Self {
        Self {
            current_state: initial_state,
            context,
        }
    }
    
    fn handle_event(&mut self, event: Event) {
        if let Some(new_state) = self.current_state.handle_event(&mut self.context, event) {
            self.current_state.exit_action(&mut self.context);
            self.current_state = new_state;
            self.current_state.entry_action(&mut self.context);
        }
    }
    
    fn get_current_state_name(&self) -> &'static str {
        self.current_state.state_name()
    }
    
    fn get_context(&self) -> &Context {
        &self.context
    }
}

// ==================== 事件驱动架构 ====================

// 事件定义
#[derive(Debug, Clone, Copy)]
struct Event {
    event_type: EventType,
    source: u8,
    timestamp: u32,
    data: EventData,
}

#[derive(Debug, Clone, Copy)]
enum EventType {
    SensorReading,
    UserInput,
    Timer,
    System,
    Error,
}

#[derive(Debug, Clone, Copy)]
union EventData {
    sensor_value: f32,
    user_command: u8,
    timer_id: u8,
    error_code: u32,
}

// 事件处理器特征
trait EventHandler {
    fn handle_event(&mut self, event: Event) -> Result<(), EventError>;
    fn can_handle(&self, event_type: EventType) -> bool;
    fn get_priority(&self) -> u8;
}

#[derive(Debug, Clone, Copy)]
enum EventError {
    HandlerNotFound,
    ProcessingError,
    InvalidEvent,
}

// 传感器事件处理器
struct SensorEventHandler {
    priority: u8,
    last_reading: f32,
    reading_count: u32,
}

impl SensorEventHandler {
    fn new(priority: u8) -> Self {
        Self {
            priority,
            last_reading: 0.0,
            reading_count: 0,
        }
    }
}

impl EventHandler for SensorEventHandler {
    fn handle_event(&mut self, event: Event) -> Result<(), EventError> {
        if event.event_type == EventType::SensorReading {
            unsafe {
                self.last_reading = event.data.sensor_value;
                self.reading_count += 1;
                
                // 处理传感器数据
                if self.last_reading > 80.0 {
                    // 温度过高，触发冷却事件
                    // 这里可以发布新的事件
                }
            }
            Ok(())
        } else {
            Err(EventError::InvalidEvent)
        }
    }
    
    fn can_handle(&self, event_type: EventType) -> bool {
        event_type == EventType::SensorReading
    }
    
    fn get_priority(&self) -> u8 {
        self.priority
    }
}

// 用户输入事件处理器
struct UserInputHandler {
    priority: u8,
    last_command: u8,
}

impl UserInputHandler {
    fn new(priority: u8) -> Self {
        Self {
            priority,
            last_command: 0,
        }
    }
}

impl EventHandler for UserInputHandler {
    fn handle_event(&mut self, event: Event) -> Result<(), EventError> {
        if event.event_type == EventType::UserInput {
            unsafe {
                self.last_command = event.data.user_command;
                
                // 处理用户命令
                match self.last_command {
                    1 => {
                        // 增加设定温度
                    }
                    2 => {
                        // 减少设定温度
                    }
                    3 => {
                        // 系统复位
                    }
                    _ => {}
                }
            }
            Ok(())
        } else {
            Err(EventError::InvalidEvent)
        }
    }
    
    fn can_handle(&self, event_type: EventType) -> bool {
        event_type == EventType::UserInput
    }
    
    fn get_priority(&self) -> u8 {
        self.priority
    }
}

// 事件调度器
struct EventDispatcher {
    handlers: Vec<Box<dyn EventHandler>, 8>,
    event_queue: Queue<Event, 32>,
}

impl EventDispatcher {
    fn new() -> (Self, Producer<Event, 32>, Consumer<Event, 32>) {
        let queue = Queue::new();
        let (producer, consumer) = queue.split();
        
        let dispatcher = Self {
            handlers: Vec::new(),
            event_queue: Queue::new(),
        };
        
        (dispatcher, producer, consumer)
    }
    
    fn register_handler(&mut self, handler: Box<dyn EventHandler>) -> Result<(), EventError> {
        self.handlers.push(handler).map_err(|_| EventError::ProcessingError)
    }
    
    fn dispatch_events(&mut self, consumer: &mut Consumer<Event, 32>) {
        while let Some(event) = consumer.dequeue() {
            self.process_event(event);
        }
    }
    
    fn process_event(&mut self, event: Event) {
        // 按优先级排序处理器
        self.handlers.sort_by_key(|h| core::cmp::Reverse(h.get_priority()));
        
        for handler in &mut self.handlers {
            if handler.can_handle(event.event_type) {
                if let Ok(_) = handler.handle_event(event) {
                    break; // 事件已处理
                }
            }
        }
    }
}

// ==================== 组件化设计 ====================

// 组件接口
trait Component {
    fn initialize(&mut self) -> Result<(), ComponentError>;
    fn update(&mut self) -> Result<(), ComponentError>;
    fn shutdown(&mut self) -> Result<(), ComponentError>;
    fn get_status(&self) -> ComponentStatus;
    fn get_name(&self) -> &'static str;
}

#[derive(Debug, Clone, Copy)]
enum ComponentError {
    InitializationFailed,
    UpdateFailed,
    ShutdownFailed,
    NotInitialized,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ComponentStatus {
    Uninitialized,
    Initializing,
    Running,
    Error,
    Shutdown,
}

// 温度控制组件
struct TemperatureControlComponent {
    status: ComponentStatus,
    current_temperature: f32,
    target_temperature: f32,
    control_output: u8,
    update_count: u32,
}

impl TemperatureControlComponent {
    fn new() -> Self {
        Self {
            status: ComponentStatus::Uninitialized,
            current_temperature: 25.0,
            target_temperature: 50.0,
            control_output: 0,
            update_count: 0,
        }
    }
    
    fn set_target(&mut self, target: f32) {
        self.target_temperature = target;
    }
    
    fn get_temperature(&self) -> f32 {
        self.current_temperature
    }
}

impl Component for TemperatureControlComponent {
    fn initialize(&mut self) -> Result<(), ComponentError> {
        self.status = ComponentStatus::Initializing;
        
        // 初始化温度传感器和执行器
        // 这里是模拟初始化
        
        self.status = ComponentStatus::Running;
        Ok(())
    }
    
    fn update(&mut self) -> Result<(), ComponentError> {
        if self.status != ComponentStatus::Running {
            return Err(ComponentError::NotInitialized);
        }
        
        // 模拟温度读取
        self.current_temperature += (self.target_temperature - self.current_temperature) * 0.1;
        
        // 计算控制输出
        let error = self.target_temperature - self.current_temperature;
        self.control_output = (error * 10.0).abs().min(100.0) as u8;
        
        self.update_count += 1;
        Ok(())
    }
    
    fn shutdown(&mut self) -> Result<(), ComponentError> {
        self.status = ComponentStatus::Shutdown;
        self.control_output = 0;
        Ok(())
    }
    
    fn get_status(&self) -> ComponentStatus {
        self.status
    }
    
    fn get_name(&self) -> &'static str {
        "TemperatureControl"
    }
}

// 组件管理器
struct ComponentManager {
    components: Vec<Box<dyn Component>, 8>,
    update_interval_ms: u32,
    last_update_time: u32,
}

impl ComponentManager {
    fn new(update_interval_ms: u32) -> Self {
        Self {
            components: Vec::new(),
            update_interval_ms,
            last_update_time: 0,
        }
    }
    
    fn add_component(&mut self, component: Box<dyn Component>) -> Result<(), ComponentError> {
        self.components.push(component).map_err(|_| ComponentError::InitializationFailed)
    }
    
    fn initialize_all(&mut self) -> Result<(), ComponentError> {
        for component in &mut self.components {
            component.initialize()?;
        }
        Ok(())
    }
    
    fn update_all(&mut self, current_time: u32) -> Result<(), ComponentError> {
        if current_time - self.last_update_time >= self.update_interval_ms {
            for component in &mut self.components {
                if component.get_status() == ComponentStatus::Running {
                    component.update()?;
                }
            }
            self.last_update_time = current_time;
        }
        Ok(())
    }
    
    fn shutdown_all(&mut self) -> Result<(), ComponentError> {
        for component in &mut self.components {
            component.shutdown()?;
        }
        Ok(())
    }
    
    fn get_component_status(&self) -> Vec<(&'static str, ComponentStatus), 8> {
        let mut status_list = Vec::new();
        for component in &self.components {
            let _ = status_list.push((component.get_name(), component.get_status()));
        }
        status_list
    }
}

// 全局变量
static mut HAL: Option<Stm32F4Hal> = None;
static mut STATE_MACHINE: Option<StateMachine<SystemContext, SystemEvent>> = None;
static mut EVENT_DISPATCHER: Option<EventDispatcher> = None;
static mut EVENT_PRODUCER: Option<Producer<Event, 32>> = None;
static mut EVENT_CONSUMER: Option<Consumer<Event, 32>> = None;
static mut COMPONENT_MANAGER: Option<ComponentManager> = None;

#[entry]
fn main() -> ! {
    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    
    let led = gpioc.pc13.into_push_pull_output();
    let button = gpioa.pa0.into_pull_up_input();
    
    // 初始化HAL
    let mut hal = Stm32F4Hal::new();
    hal.set_led(led);
    hal.set_button(button);
    let _ = hal.init();
    
    // 初始化状态机
    let context = SystemContext::new();
    let state_machine = StateMachine::new(Box::new(IdleState), context);
    
    // 初始化事件系统
    let (mut dispatcher, producer, mut consumer) = EventDispatcher::new();
    
    // 注册事件处理器
    let sensor_handler = Box::new(SensorEventHandler::new(10));
    let user_handler = Box::new(UserInputHandler::new(5));
    
    let _ = dispatcher.register_handler(sensor_handler);
    let _ = dispatcher.register_handler(user_handler);
    
    // 初始化组件管理器
    let mut component_manager = ComponentManager::new(100); // 100ms更新间隔
    let temp_component = Box::new(TemperatureControlComponent::new());
    let _ = component_manager.add_component(temp_component);
    let _ = component_manager.initialize_all();
    
    // 存储到全局变量
    unsafe {
        HAL = Some(hal);
        STATE_MACHINE = Some(state_machine);
        EVENT_DISPATCHER = Some(dispatcher);
        EVENT_PRODUCER = Some(producer);
        EVENT_CONSUMER = Some(consumer);
        COMPONENT_MANAGER = Some(component_manager);
    }
    
    // 配置定时器
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(10.millis()).unwrap();
    timer.listen(Event::Update);
    
    // 启用定时器中断
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }
    
    let mut loop_counter = 0u32;
    
    loop {
        // 处理事件
        unsafe {
            if let (Some(ref mut dispatcher), Some(ref mut consumer)) = 
                (EVENT_DISPATCHER.as_mut(), EVENT_CONSUMER.as_mut()) {
                dispatcher.dispatch_events(consumer);
            }
        }
        
        // 更新组件
        unsafe {
            if let (Some(ref mut manager), Some(ref hal)) = 
                (COMPONENT_MANAGER.as_mut(), HAL.as_ref()) {
                let _ = manager.update_all(hal.get_timestamp());
            }
        }
        
        // 定期发送事件
        loop_counter += 1;
        if loop_counter >= 10000 {
            unsafe {
                if let (Some(ref mut producer), Some(ref hal)) = 
                    (EVENT_PRODUCER.as_mut(), HAL.as_ref()) {
                    
                    // 发送传感器读取事件
                    if let Ok(sensor_value) = hal.read_sensor() {
                        let event = Event {
                            event_type: EventType::SensorReading,
                            source: 1,
                            timestamp: hal.get_timestamp(),
                            data: EventData { sensor_value },
                        };
                        let _ = producer.enqueue(event);
                    }
                    
                    // 发送状态机事件
                    if let Some(ref mut sm) = STATE_MACHINE.as_mut() {
                        if let Ok(temp) = hal.read_sensor() {
                            sm.handle_event(SystemEvent::TemperatureUpdate(temp));
                        }
                    }
                }
            }
            loop_counter = 0;
        }
        
        // 短暂延时
        cortex_m::asm::delay(1000);
    }
}

// 定时器中断处理
#[interrupt]
fn TIM2() {
    unsafe {
        let timer = &*pac::TIM2::ptr();
        timer.sr.modify(|_, w| w.uif().clear_bit());
        
        // 更新HAL时间戳
        if let Some(ref mut hal) = HAL.as_mut() {
            hal.tick();
        }
    }
}